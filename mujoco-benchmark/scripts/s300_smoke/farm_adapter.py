"""Exact static greenhouse visuals for the S300 MuJoCo adapter.

The USD instance roots are the batching unit.  All original triangles and
diffuse colours are retained, but material-compatible meshes within a stem or
truss are combined and identical assets are shared.  This avoids creating
29,000 MuJoCo geoms for the current compact greenhouse.  Background plants are
visual only; the selected target plant is exported by the caller.

Run this module with the existing asset Python (USD), not the physics Python.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics


GUTTER_PANELS = ('Gutter_1_Base', 'Gutter_1_Sidewall', 'GutterFoldedLip',
                 'GutterEndCap', 'Gutter_SlabSupportRib')


def _nums(values):
    return ' '.join(format(float(v), '.12g') for v in np.asarray(values).ravel())


def _sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def _under(path, roots):
    return any(path == p or path.startswith(p + '/') for p in roots)


def _pose(transform):
    """Proper rotation and residual affine; reflection/scale is baked in mesh."""
    linear = transform[:3, :3]
    u, _, vt = np.linalg.svd(linear)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1
        rotation = u @ vt
    # Normalize only round-off from the rigid transform's SVD. Identical USD
    # instances then share asset bytes instead of duplicating 27 million
    # vertices because their rotations produce 1e-16 floating point noise.
    residual = np.round(rotation.T @ linear, 12)
    q = Rotation.from_matrix(rotation).as_quat()
    return transform[:3, 3], np.r_[q[3], q[:3]], residual


def _faces(mesh):
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    if counts.sum() != len(indices) or (counts < 3).any():
        raise ValueError('Invalid source mesh topology: ' + str(mesh.GetPath()))
    if (counts == 3).all():
        return indices.reshape(-1, 3).copy()
    triangles, offset = [], 0
    for count in counts:
        triangles.extend((indices[offset], indices[offset + j], indices[offset + j + 1])
                         for j in range(1, int(count) - 1))
        offset += int(count)
    return np.asarray(triangles, dtype=np.int64)


def _material(prim, stage, memo):
    targets = prim.GetRelationship('material:binding').GetTargets()
    material = UsdShade.Material(stage.GetPrimAtPath(targets[0])) if targets else None
    if not material:
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    path = str(material.GetPath()) if material else ''
    if path and path in memo:
        return memo[path]
    color = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
    rgba = list(color[0]) + [1.] if color else [.5, .5, .5, 1.]
    roughness, metallic, textures = .5, 0., []
    if material:
        shader = material.ComputeSurfaceSource()[0]
        if shader:
            for input_name, channel in (('diffuseColor', 'rgb'), ('opacity', 'alpha'),
                                        ('roughness', 'roughness'), ('metallic', 'metallic')):
                input_ = shader.GetInput(input_name)
                if not input_:
                    continue
                value = input_.Get()
                if input_.HasConnectedSource():
                    textures.append(input_name)
                if value is None:
                    continue
                if channel == 'rgb':
                    rgba[:3] = list(value)
                elif channel == 'alpha':
                    rgba[3] = float(value)
                elif channel == 'roughness':
                    roughness = float(value)
                else:
                    metallic = float(value)
    result = (tuple(map(float, rgba)), roughness, metallic, tuple(textures))
    if path:
        memo[path] = result
    return result


def add_background(stage, manifest, *, world, assets, output,
                   excluded_roots=(), excluded_paths=(), cache_dir=None,
                   structural_collision='gutter-ground', structural_geom_attrs=None,
                   ground_collision='source-box',
                   greenhouse_root='/World/Greenhouse', ground_path='/World/Ground',
                   prefix='farm'):
    """Append greenhouse geometry and return an auditable inventory.

    ``world`` and ``assets`` are the caller's worldbody/asset XML elements.
    ``output`` is the model directory, whose compiler meshdir must be meshes.
    Cache files may be reused between exports; the source USD is never edited.

    structural_collision: gutter-ground (18 authored channel panels plus source
    ground), authored-house (every enabled source house mesh plus source ground)
    or visual-only.  Background plants never acquire colliders.  The authored-
    house option uses each source mesh's convex hull, not native PhysX cooking.
    Other dynamic or already-exported actors must be excluded by the caller.

    ground_collision: source-box retains the authored finite solid. top-plane
    keeps that box as a visual and explicitly uses an infinite contact plane
    at the authored top face, so a fast released fruit cannot cross the entire
    thin box in one discrete step. No mass, friction or contact settings change;
    the infinite extent is a declared collision representation variant.
    """
    if structural_collision not in ('gutter-ground', 'authored-house', 'visual-only'):
        raise ValueError('Unknown structural collision scope')
    if ground_collision not in ('source-box', 'top-plane'):
        raise ValueError('Unknown ground collision representation')
    if UsdGeom.GetStageUpAxis(stage) != 'Z' or UsdGeom.GetStageMetersPerUnit(stage) != 1.:
        raise ValueError('Expected the current metre / Z-up source stage')
    if any(g.get('name', '').startswith(prefix + '_') for g in world.iter('geom')):
        raise ValueError('Background already appended under this prefix')
    started = time.perf_counter()
    output = Path(output).resolve()
    local_dir = output / 'meshes' / prefix
    local_dir.mkdir(parents=True, exist_ok=True)
    cache_dir = Path(cache_dir).resolve() if cache_dir else local_dir
    cache_dir.mkdir(parents=True, exist_ok=True)
    roots = tuple(str(p).rstrip('/') for p in excluded_roots)
    paths = set(map(str, excluded_paths))
    cache = UsdGeom.XformCache()
    material_cache, local_materials, mesh_assets, prototype_groups = {}, {}, {}, {}
    mesh_asset_inventory = []
    inventory, collisions, sources, unsupported, hidden = [], [], [], [], []
    counts = dict(source_meshes=0, source_vertices=0, source_triangles=0,
                  instanced_groups=0, visual_geoms=0, cached_asset_reuses=0)
    background_bounds = [np.full(3, np.inf), np.full(3, -np.inf)]

    def matrix(prim):
        return np.asarray(cache.GetLocalToWorldTransform(prim), float).T

    def excluded(prim):
        path = str(prim.GetPath())
        return path in paths or _under(path, roots)

    def file_asset(vertices, faces):
        if len(vertices) < 3 or not np.isfinite(vertices).all():
            raise ValueError('Invalid background geometry')
        digest = hashlib.sha256(b's300-exact-background-obj-v2-triangle-padding')
        for array in (np.asarray(vertices, '<f8'), np.asarray(faces, '<i8')):
            digest.update(str(array.shape).encode()); digest.update(array.tobytes())
        key = digest.hexdigest()
        if key in mesh_assets:
            counts['cached_asset_reuses'] += 1
            return mesh_assets[key]
        cached = cache_dir / (key + '.obj')
        if not cached.exists():
            # Atomic replacement prevents another export from seeing partial OBJ.
            temporary = cached.with_suffix('.' + str(os.getpid()) + '.tmp')
            with temporary.open('w') as stream:
                stream.write('# Exact USD triangles; no decimation\n')
                # MuJoCo requires at least four mesh vertices even for a
                # visual-only triangle. Duplicate one unreferenced vertex;
                # source faces, their positions and rendered shape stay exact.
                obj_vertices = np.vstack([vertices, vertices[:1]]) if len(vertices) == 3 else vertices
                np.savetxt(stream, obj_vertices, fmt='v %.12g %.12g %.12g')
                np.savetxt(stream, faces + 1, fmt='f %d %d %d')
            temporary.replace(cached)
        else:
            counts['cached_asset_reuses'] += 1
        destination = local_dir / cached.name
        if cached != destination and not destination.exists():
            try:
                os.link(cached, destination)
            except OSError:
                import shutil
                shutil.copy2(cached, destination)
        name = prefix + '_mesh_' + key[:24]
        ET.SubElement(assets, 'mesh', name=name,
                      file=str(Path(prefix) / destination.name), inertia='shell')
        mesh_assets[key] = name
        mesh_asset_inventory.append(dict(name=name, geometry_sha256=key,
                                         file=str(destination), file_sha256=_sha(destination),
                                         source_vertices=len(vertices), triangles=len(faces),
                                         unreferenced_compatibility_vertices=int(len(vertices) == 3)))
        return name

    def material_asset(material):
        if material not in local_materials:
            rgba, roughness, metallic, texture_channels = material
            name = prefix + '_material_' + str(len(local_materials))
            ET.SubElement(assets, 'material', name=name, rgba=_nums(rgba),
                          shininess=str(max(0., min(1., 1. - roughness))),
                          specular=str(.3 + .5 * max(0., min(1., metallic))))
            local_materials[material] = name
            if texture_channels:
                unsupported.append(dict(material=name, connected_channels=list(texture_channels),
                                        interpretation='source scalar material values only'))
        return local_materials[material]

    def make_groups(root):
        root_inv = np.linalg.inv(matrix(root))
        groups, paths_local = {}, []
        range_ = Usd.PrimRange(root, Usd.TraverseInstanceProxies())
        for prim in range_:
            if excluded(prim):
                continue
            if not prim.IsA(UsdGeom.Mesh):
                if prim.IsA(UsdGeom.Gprim):
                    unsupported.append(dict(path=str(prim.GetPath()), type=prim.GetTypeName()))
                continue
            mesh = UsdGeom.Mesh(prim)
            if mesh.ComputeVisibility() == UsdGeom.Tokens.invisible:
                hidden.append(str(prim.GetPath())); continue
            vertices = np.asarray(mesh.GetPointsAttr().Get(), float)
            faces = _faces(mesh)
            local = root_inv @ matrix(prim)
            vertices = vertices @ local[:3, :3].T + local[:3, 3]
            if (mesh.GetOrientationAttr().Get() == 'leftHanded') != (np.linalg.det(local[:3, :3]) < 0):
                faces = faces[:, ::-1]
            key = _material(prim, stage, material_cache)
            group = groups.setdefault(key, dict(vertices=[], faces=[], offsets=0, source=[]))
            group['vertices'].append(vertices)
            group['faces'].append(faces + group['offsets'])
            group['offsets'] += len(vertices)
            row = dict(relative_path=str(prim.GetPath())[len(str(root.GetPath())):],
                       vertices=len(vertices), triangles=len(faces))
            group['source'].append(row); paths_local.append(row)
        return [(key, np.concatenate(g['vertices']), np.concatenate(g['faces']), g['source'])
                for key, g in groups.items()], paths_local

    def emit_group(root):
        root_path = str(root.GetPath())
        if excluded(root):
            return
        if UsdGeom.Imageable(root).ComputeVisibility() == UsdGeom.Tokens.invisible:
            hidden.append(root_path); return
        # Prototype IDs are only used within this one source stage. File cache
        # identity uses geometry bytes and thus remains valid across stages.
        prototype = str(root.GetPrototype().GetPath()) if root.IsInstance() else root_path
        exclusions_inside = any(_under(p, (root_path,)) for p in paths | set(roots))
        binding = tuple(map(str, root.GetRelationship('material:binding').GetTargets()))
        group_key = (prototype, binding, root_path if exclusions_inside else '')
        if group_key not in prototype_groups:
            prototype_groups[group_key] = make_groups(root)
        groups, source_rows = prototype_groups[group_key]
        pos, quaternion, residual = _pose(matrix(root))
        counts['source_meshes'] += len(source_rows)
        counts['source_vertices'] += sum(r['vertices'] for r in source_rows)
        counts['source_triangles'] += sum(r['triangles'] for r in source_rows)
        counts['instanced_groups'] += int(root.IsInstance())
        rotation = Rotation.from_quat(np.r_[quaternion[1:], quaternion[0]]).as_matrix()
        for material, local_vertices, local_faces, source_rows_group in groups:
            vertices = local_vertices @ residual.T
            faces = local_faces[:, ::-1] if np.linalg.det(residual) < 0 else local_faces
            mesh_name = file_asset(vertices, faces)
            name = prefix + '_visual_' + str(counts['visual_geoms'])
            ET.SubElement(world, 'geom', name=name, type='mesh', mesh=mesh_name,
                          material=material_asset(material), pos=_nums(pos), quat=_nums(quaternion),
                          contype='0', conaffinity='0', density='0', group='2',
                          margin='0', gap='0')
            world_vertices = vertices @ rotation.T + pos
            background_bounds[0] = np.minimum(background_bounds[0], world_vertices.min(0))
            background_bounds[1] = np.maximum(background_bounds[1], world_vertices.max(0))
            inventory.append(dict(name=name, source_root=root_path, mesh=mesh_name,
                                  source_meshes=len(source_rows_group), vertices=len(vertices),
                                  triangles=len(faces), position_world_m=pos.tolist(),
                                  quaternion_wxyz=quaternion.tolist(),
                                  residual_affine=residual.tolist(), collision=False))
            counts['visual_geoms'] += 1
        sources.append(dict(root=root_path, prototype=prototype,
                            source_meshes=len(source_rows), source_geometry=source_rows))

    greenhouse = stage.GetPrimAtPath(greenhouse_root)
    if not greenhouse:
        raise ValueError('Missing greenhouse root ' + greenhouse_root)
    # Non-proxy traversal stops at instance roots. Each instance is then batched
    # by source material; ordinary meshes keep their exact local geometry.
    for prim in Usd.PrimRange(greenhouse):
        if prim.IsInstance() or prim.IsA(UsdGeom.Mesh):
            emit_group(prim)

    collision_attrs = dict(contype='8', conaffinity='7', density='0', group='3',
                           rgba='0 0 0 0')
    collision_attrs.update(structural_geom_attrs or {})
    if structural_collision != 'visual-only':
        for prim in Usd.PrimRange(greenhouse, Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())
            if excluded(prim) or '/House/' not in path or not prim.IsA(UsdGeom.Mesh):
                continue
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                continue
            if structural_collision == 'gutter-ground' and not any(
                    part.startswith(GUTTER_PANELS) for part in path.split('/')):
                continue
            mesh = UsdGeom.Mesh(prim)
            vertices = np.asarray(mesh.GetPointsAttr().Get(), float)
            faces = _faces(mesh)
            pos, quaternion, residual = _pose(matrix(prim))
            vertices = vertices @ residual.T
            if (mesh.GetOrientationAttr().Get() == 'leftHanded') != (np.linalg.det(residual) < 0):
                faces = faces[:, ::-1]
            name = prefix + '_structural_' + str(len(collisions))
            lo, hi = vertices.min(0), vertices.max(0)
            distinct = np.unique(np.round(vertices, 9), axis=0)
            is_box = len(distinct) == 8 and np.all(np.minimum(abs(vertices-lo), abs(vertices-hi)) < 1e-8)
            if is_box:
                rotation = Rotation.from_quat(np.r_[quaternion[1:], quaternion[0]]).as_matrix()
                attrs = dict(type='box', pos=_nums(pos + rotation @ ((lo+hi)/2)),
                             quat=_nums(quaternion), size=_nums((hi-lo)/2))
                approximation = 'exact_source_box'
            else:
                attrs = dict(type='mesh', mesh=file_asset(vertices, faces),
                             pos=_nums(pos), quat=_nums(quaternion))
                approximation = 'per_source_mesh_convex_hull; native PhysX cooked hull not reproduced'
            ET.SubElement(world, 'geom', name=name, **attrs, **collision_attrs)
            collisions.append(dict(name=name, source_path=path, classification='gutter' if
                                   any(part.startswith(GUTTER_PANELS) for part in path.split('/')) else 'house',
                                   approximation=approximation, source_collision_enabled=True))

    ground = stage.GetPrimAtPath(ground_path)
    if ground and not excluded(ground) and UsdGeom.Imageable(ground).ComputeVisibility() != 'invisible':
        if not ground.IsA(UsdGeom.Cube):
            raise ValueError('Expected source ground cube; refusing an invented replacement')
        pos, quaternion, residual = _pose(matrix(ground))
        if not np.allclose(residual, np.diag(np.diag(residual)), atol=1e-10):
            raise ValueError('Sheared source ground cube needs an exact mesh')
        size = abs(np.diag(residual)) * float(UsdGeom.Cube(ground).GetSizeAttr().Get()) / 2
        name = prefix + '_ground'
        attrs = dict(name=name, type='box', pos=_nums(pos), quat=_nums(quaternion), size=_nums(size),
                     material=material_asset(_material(ground, stage, material_cache)))
        enabled = bool(ground.HasAPI(UsdPhysics.CollisionAPI) and
                       UsdPhysics.CollisionAPI(ground).GetCollisionEnabledAttr().Get() is not False)
        if structural_collision != 'visual-only' and enabled:
            if ground_collision == 'source-box':
                ET.SubElement(world, 'geom', **attrs, **{k:v for k,v in collision_attrs.items() if k != 'rgba'})
                collisions.append(dict(name=name, source_path=ground_path, classification='ground',
                                       approximation='exact_source_box', source_collision_enabled=True))
            else:
                rotation=Rotation.from_quat(np.r_[quaternion[1:],quaternion[0]]).as_matrix()
                if not np.allclose(rotation[:2,2],0.,atol=1e-10) or not np.isclose(abs(rotation[2,2]),1.,atol=1e-10):
                    raise ValueError('Top-plane ground variant requires a horizontal source top face')
                top=pos.copy();top[2]+=size[2]
                visual_attrs=dict(attrs,name=name+'_visual')
                ET.SubElement(world,'geom',**visual_attrs,contype='0',conaffinity='0',density='0',group='2')
                ET.SubElement(world,'geom',name=name,type='plane',pos=_nums(top),quat='1 0 0 0',
                              size=_nums([size[0],size[1],.01]),**collision_attrs)
                collisions.append(dict(name=name,source_path=ground_path,classification='ground',
                                       approximation='infinite horizontal plane at exact source box top',
                                       source_collision_enabled=True,source_box_retained_visually=True,
                                       top_world_z_m=float(top[2]),infinite_collision_extent=True,
                                       finite_source_half_extents_m=size.tolist()))
        else:
            ET.SubElement(world, 'geom', **attrs, contype='0', conaffinity='0', density='0', group='2')
    else:
        ground_path = None

    layers = [dict(path=layer.realPath, sha256=_sha(layer.realPath)) for layer in stage.GetUsedLayers()
              if layer.realPath and Path(layer.realPath).is_file()]
    report = dict(schema='s300-mujoco-exact-static-greenhouse-v1',
                  source_layers=layers, source_manifest_target=manifest.get('plant', {}).get('source_plant'),
                  collision_scope=structural_collision, background_plant_collision=False,
                  ground_collision=ground_collision,
                  background_plant_dofs=0, target_plant_owned_by_caller=True,
                  source_ground=ground_path, excluded_roots=list(roots), excluded_paths=sorted(paths),
                  counts=dict(counts, unique_mesh_assets=len(mesh_assets), material_assets=len(local_materials),
                              structural_collision_geoms=len(collisions)),
                  bounds_world_m=[b.tolist() for b in background_bounds],
                  export_wall_s=time.perf_counter()-started, asset_cache=str(cache_dir),
                  geometry_inventory=inventory, structural_inventory=collisions,
                  mesh_asset_inventory=mesh_asset_inventory,
                  source_inventory=sources, hidden=hidden, unsupported=unsupported,
                  triangle_decimation=False, source_modified=False,
                  material_interpretation='Original scalar diffuse/opacity/roughness/metallic; lighting and shader appearance differ between renderers',
                  training_eligible=False)
    (output / (prefix + '_background.json')).write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report
