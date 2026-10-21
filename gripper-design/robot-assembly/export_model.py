#!/usr/bin/env python3
"""Export the current measured S300 assembly; never write source USD/physics.

Run with nvidia-sim/.asset-venv/bin/python. Preserves visual triangles,
material subsets and measured body transforms, excludes collision-only copies.
"""
import argparse
from collections import defaultdict
import hashlib
import gzip
import json
from pathlib import Path
import sys

import numpy as np
import trimesh
from pxr import Gf, Usd, UsdGeom, UsdShade

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'nvidia-sim/harvest_s300'))
from motion_pose_display import MeasuredPoseDisplay
from motion_kinematics import transmission_transforms

ROOTS = ('/World/HarvestRobot', '/World/ActuatedTool')
LABELS = dict(chassis='S300 기체', lift='리프트 고정부', carriage='승강 받침대', arm='RB5 로봇팔', gripper='벨트 그리퍼')
# Source is Z-up in metres; glTF is Y-up in metres.
TO_GLTF = np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]])


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def category(path):
    if '/real_lift_carriage/' in path:
        return 'carriage'
    if path.startswith('/World/ActuatedTool') or '/tcp/' in path:
        return 'gripper'
    if '/S300Carrier/' in path:
        return 'chassis'
    if '/world/visuals/' in path:
        return 'lift'
    return 'arm'


def material_values(prim, fallback):
    rgb, metallic, roughness = np.array(fallback, float), 0., .55
    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if material:
        for p in Usd.PrimRange(material.GetPrim()):
            if not p.IsA(UsdShade.Shader):
                continue
            shader = UsdShade.Shader(p)
            for names, kind in ((('diffuseColor', 'diffuse_color_constant'), 'rgb'),
                                (('metallic', 'metallic_constant'), 'metallic'),
                                (('roughness', 'reflection_roughness_constant'), 'roughness')):
                for name in names:
                    inp = shader.GetInput(name)
                    value = inp.Get() if inp else None
                    if value is None:
                        continue
                    if inp.GetConnectedSource():
                        raise ValueError('Textured shader needs explicit export support: ' + str(prim.GetPath()))
                    if kind == 'rgb': rgb = np.array(value, float)
                    elif kind == 'metallic': metallic = float(value)
                    else: roughness = float(value)
                    break
    return tuple(np.clip(rgb, 0, 1)), metallic, roughness


def source_mesh(prim):
    if prim.IsA(UsdGeom.Mesh):
        m = UsdGeom.Mesh(prim)
        points = np.asarray(m.GetPointsAttr().Get(), float)
        counts = np.asarray(m.GetFaceVertexCountsAttr().Get())
        if not np.all(counts == 3):
            raise ValueError('Non-triangle source: ' + str(prim.GetPath()))
        faces = np.asarray(m.GetFaceVertexIndicesAttr().Get(), int).reshape(-1, 3)
        normals = np.asarray(m.GetNormalsAttr().Get() or [], float)
        if len(normals):
            mode = m.GetNormalsInterpolation()
            if mode == 'faceVarying':
                points = points[faces.ravel()]
                faces = np.arange(len(points)).reshape(-1, 3)
            elif mode == 'uniform':
                points = points[faces.ravel()]
                normals = np.repeat(normals, 3, axis=0)
                faces = np.arange(len(points)).reshape(-1, 3)
            elif mode != 'vertex':
                raise ValueError('Unsupported authored normals: ' + str(mode))
            if len(normals) != len(points):
                raise ValueError('Normals/vertices mismatch')
        colors = m.GetDisplayColorAttr().Get()
        vertex_colors = None
        if colors is not None and len(colors) > 1:
            colors = np.asarray(m.GetDisplayColorPrimvar().ComputeFlattened(), float)
            if m.GetDisplayColorPrimvar().GetInterpolation() != 'vertex' or len(colors) != len(points):
                raise ValueError('Unsupported varying colors: ' + str(prim.GetPath()))
            vertex_colors = colors
        rgb = colors[0] if colors is not None else (.65, .65, .65)
        parts = []
        used = np.zeros(len(faces), bool)
        for sub in UsdGeom.Subset.GetGeomSubsets(m, UsdGeom.Tokens.face):
            ids = np.asarray(sub.GetIndicesAttr().Get(), int)
            if np.any(ids < 0) or np.any(ids >= len(faces)) or np.any(used[ids]):
                raise ValueError('Invalid material subset')
            used[ids] = True
            parts.append((ids, material_values(sub.GetPrim(), rgb)))
        if not np.all(used):
            parts.append((np.flatnonzero(~used), material_values(prim, rgb)))
        return points, faces, normals, vertex_colors, parts, m.GetOrientationAttr().Get() == 'leftHanded'
    if prim.IsA(UsdGeom.Cube):
        m = trimesh.creation.box(extents=[UsdGeom.Cube(prim).GetSizeAttr().Get()]*3)
    elif prim.IsA(UsdGeom.Cylinder):
        c = UsdGeom.Cylinder(prim)
        m = trimesh.creation.cylinder(radius=c.GetRadiusAttr().Get(), height=c.GetHeightAttr().Get(), sections=64)
        axis = c.GetAxisAttr().Get()
        if axis != 'Z':
            m.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], [1, 0, 0] if axis == 'X' else [0, 1, 0]))
    else:
        raise ValueError('Unsupported visible geometry: ' + str(prim.GetPath()))
    col = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
    return m.vertices, m.faces, m.vertex_normals, None, [(np.arange(len(m.faces)), material_values(prim, col[0] if col else (.65, .65, .65)))], False


def export(assembly, states, output, frame):
    source = assembly / 'composed.usdc'
    before = sha(source)
    lift_info = json.loads((assembly/'real_lift_mount.json').read_text()) if (assembly/'real_lift_mount.json').exists() else None
    stage = Usd.Stage.Open(str(source))
    if UsdGeom.GetStageUpAxis(stage) != 'Z' or UsdGeom.GetStageMetersPerUnit(stage) != 1.:
        raise ValueError('Expected current metre/Z-up assembly')
    with np.load(states, allow_pickle=False) as data:
        paths = data['body_paths'].tolist()
        ids = [i for i, p in enumerate(paths) if p.startswith(ROOTS)]
        poses = data['poses'][frame, ids]
        stamp = float(data['time'][frame])
        robot_q = data['robot_q'][frame]
    display = MeasuredPoseDisplay(stage, [paths[i] for i in ids])
    display.update(poses)
    gap = float(.044 - robot_q[-2] + robot_q[-1])
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for name, tf in transmission_transforms(gap).items():
            prim = stage.GetPrimAtPath('/World/ActuatedTool/fixed/Transmission/' + name)
            if prim:
                UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(tf.T.tolist()))
    cache = UsdGeom.XformCache()
    origin = np.array([0., poses[[paths[i] for i in ids].index('/World/HarvestRobot/Robot/world'), 1], 0.])
    buckets = defaultdict(list)
    inventory = []
    expected_bounds = [np.full(3, np.inf), np.full(3, -np.inf)]
    for root in ROOTS:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if not prim.IsA(UsdGeom.Gprim): continue
            path = str(prim.GetPath())
            if '/collisions/' in path or UsdGeom.Imageable(prim).ComputeVisibility() == 'invisible': continue
            if UsdGeom.Imageable(prim).ComputePurpose() == 'guide': continue
            points, faces, normals, colors, parts, left_handed = source_mesh(prim)
            tf = np.asarray(cache.GetLocalToWorldTransform(prim)).T
            vertices = ((points @ tf[:3, :3].T + tf[:3, 3]) - origin) @ TO_GLTF.T
            if not np.isfinite(vertices).all(): raise ValueError('Nonfinite mesh')
            expected_bounds[0] = np.minimum(expected_bounds[0], vertices.min(0))
            expected_bounds[1] = np.maximum(expected_bounds[1], vertices.max(0))
            if len(normals):
                normals = normals @ np.linalg.inv(tf[:3, :3]) @ TO_GLTF.T
                normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-20)
            if left_handed != (np.linalg.det(tf[:3, :3]) < 0): faces = faces[:, ::-1]
            group = category(path)
            for face_ids, material in parts:
                if not len(face_ids): continue
                selected_faces = faces[face_ids]
                unique, inverse = np.unique(selected_faces.ravel(), return_inverse=True)
                selected_normals = normals[unique] if len(normals) else None
                mesh = trimesh.Trimesh(vertices=vertices[unique], faces=inverse.reshape(-1, 3),
                    vertex_normals=selected_normals, process=False)
                if colors is not None:
                    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=np.rint(np.c_[colors[unique], np.ones(len(unique))]*255).astype(np.uint8))
                    key = (group, 'vertex_colors')
                else:
                    key = (group, *material)
                # Weld only at 1e-9 m storage precision, preserving normal/color
                # seams; source triangles are never decimated or removed.
                mesh.merge_vertices(merge_norm=False, merge_tex=False, digits_vertex=9)
                buckets[key].append(mesh)
            inventory.append(dict(path=path, group=group, triangles=len(faces)))
    scene = trimesh.Scene()
    counts = defaultdict(lambda: dict(meshes=0, triangles=0))
    for group in LABELS: scene.graph.update(frame_to=group, matrix=np.eye(4))
    for i, (key, pieces) in enumerate(buckets.items()):
        merged = trimesh.util.concatenate(pieces)
        if key[1] != 'vertex_colors':
            _, rgb, metallic, roughness = key
            merged.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                name=f'material_{i}', baseColorFactor=np.rint([*np.multiply(rgb, 255), 255]).astype(np.uint8),
                metallicFactor=metallic, roughnessFactor=roughness, doubleSided=True))
        name = f'{key[0]}_{i}'
        scene.add_geometry(merged, geom_name=name, node_name=name, parent_node_name=key[0])
        counts[key[0]]['meshes'] += 1
        counts[key[0]]['triangles'] += len(merged.faces)
    output.mkdir(parents=True, exist_ok=True)
    dest = output / 's300_lift_rb5_gripper.glb'
    dest.write_bytes(trimesh.exchange.gltf.export_glb(scene, include_normals=True))
    compressed = dest.with_suffix('.glb.gz')
    compressed.write_bytes(gzip.compress(dest.read_bytes(), compresslevel=6, mtime=0))
    loaded = trimesh.load(dest, force='scene', process=False)
    triangles = sum(len(m.faces) for m in loaded.geometry.values())
    if triangles != sum(i['triangles'] for i in inventory): raise ValueError('Lost source triangles')
    err = float(np.max(np.abs(loaded.bounds - np.array(expected_bounds))))
    if err > 1e-6: raise ValueError('Export bounds differ from source: ' + str(err))
    if before != sha(source): raise ValueError('Source asset changed')
    info = dict(schema='farmily-robot-assembly-web-v1', title='S300 · 리프트 · RB5 · 벨트 그리퍼',
        source_assembly=str(assembly), source_sha256=before, states=str(states), states_sha256=sha(states),
        frame=frame, simulation_time_s=stamp, gripper='V5.1-based simulation-modified tool, not standalone V6',
        source_origin_world_m=origin.tolist(), export_axes='glTF Y-up; x=x, y=z, z=-y; metres',
        file=dest.name, bytes=dest.stat().st_size, sha256=sha(dest), bounds_m=loaded.bounds.tolist(),
        compressed_file=compressed.name, compressed_bytes=compressed.stat().st_size,
        triangles=triangles, visual_source_meshes=len(inventory), source_surface_preserved=True,
        maximum_bounds_error_m=err, components={k: dict(label=LABELS[k], **v) for k, v in counts.items()},
        source_inventory=inventory, simulation_rerun=False)
    if lift_info:
        info['lift'] = dict(reference_height_m=float(robot_q[1]), range_m=lift_info['range_m'],
                            moving_nodes=['carriage','arm','gripper'],
                            fixed_nodes=['chassis','lift'], virtual_shelf_removed=True,
                            correction=lift_info, preview='Rigid assembly motion along existing lift joint; not live physics')
    (output / 'model.json').write_text(json.dumps(info, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: info[k] for k in ('file', 'bytes', 'triangles', 'visual_source_meshes', 'maximum_bounds_error_m', 'components')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('assembly', type=Path)
    p.add_argument('states', type=Path)
    p.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'models')
    p.add_argument('--frame', type=int, default=0)
    a = p.parse_args()
    export(a.assembly, a.states, a.output, a.frame)
