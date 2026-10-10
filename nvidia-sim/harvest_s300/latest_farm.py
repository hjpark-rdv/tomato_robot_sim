"""Full-resolution USD plants using the reviewed v8/v9 and GLB-Y layout.

This is an asset adapter, not a new physics or collection pipeline. The saved
MuJoCo layout supplies identities/angles; original USD/GLB supply all triangles.
"""
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'mujoco-benchmark/scripts'))
sys.path.insert(0, str(ROOT / 'nvidia-sim/rl'))
from view_glb_truss import read_glb
from elastic_geometry import tube_centerline, resample_rod

LAYOUT = ROOT / 'mujoco-benchmark/validation/covered_d1_server/covered_scene_metadata.json'
STEM_V8 = ROOT / 'nvidia-sim/env_usd/tomato_stem_v8_with_rotated90_cluster_harvestable_FIXED.usd'
STEM_V9 = ROOT / 'nvidia-sim/env_usd/tomato_stem_leaves_v9.glb'
GLB_DIR = ROOT / 'nvidia-sim/env_usd/tomato_rotate_glb'
FIXED = Rotation.from_euler('x', 90, degrees=True).as_matrix()


def colored_normals(vertices, faces):
    # No merging, decimation, face removal or convexification of visuals.
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False).vertex_normals


def truss_meshes(path, placement):
    """Mirror the existing GLB builder's authored fruit correction, then scale.

    Keep every primitive (including repeated names). Fruit06 and its calyx are
    excluded per the reviewed profile; its pedicel is retained.
    """
    meshes = [(n, v.copy(), f, c) for n, v, f, c in read_glb(path, True)]
    for fruit_id, offset in placement.get('fruit_offsets', {}).items():
        suffix = f'{int(fruit_id):02d}'
        offset = np.asarray(offset, float)
        proximal = next(v for n, v, _, _ in meshes if n == 'Pedicel_proximal_' + suffix)
        centers, _ = tube_centerline(proximal, 12)
        lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(centers, axis=0), axis=1))]
        for name, v, _, _ in meshes:
            tokens = name.split('_')
            calyx = name.startswith('Calyx_') and int(tokens[2] if tokens[1] == 'Hub' else tokens[1]) == int(fruit_id)
            if name.startswith(('Fruit_' + suffix, 'Pedicel_distal_' + suffix)) or calyx:
                v += offset
            elif name.startswith('Pedicel_proximal_' + suffix):
                nearest = np.argmin(np.linalg.norm(v[:, None, :] - centers[None, :, :], axis=2), axis=1)
                v += offset * (lengths[nearest] / lengths[-1])[:, None]
    ped, _ = tube_centerline(next(v for n, v, _, _ in meshes if n == 'Truss_01_Peduncle'), 14)
    removed = set(placement['remove_fruits'])
    included = []
    for name, v, f, c in meshes:
        tokens = name.split('_')
        fruit_id = (int(tokens[1]) if name.startswith('Fruit_') else
                    int(tokens[2] if tokens[1] == 'Hub' else tokens[1]) if name.startswith('Calyx_') else None)
        if fruit_id in removed:
            continue
        included.append((name, (v - ped[0]) @ FIXED.T * placement['truss_scale'], f, c))
    return included, ped


def surface_attachment(points, radii, segment, fraction, ped, angle):
    """Same surface construction as build_glb_physics.attachment_frame.

    The existing capsule +Z points to the root. There is no tilt-to-stem rotation.
    """
    a, b = points[segment:segment + 2]
    axis = (a - b) / np.linalg.norm(a - b)
    rotation = FIXED @ Rotation.from_euler('y', angle, degrees=True).as_matrix()
    heading = rotation @ (ped[3] - ped[0])
    normal = heading - axis * np.dot(axis, heading)
    if np.linalg.norm(normal) < 1e-9:
        normal = rotation[:, 0] - axis * np.dot(axis, rotation[:, 0])
    normal /= np.linalg.norm(normal)
    return (a + b) / 2 + (2 * fraction - 1) * (a - b) / 2 + normal * max(radii[segment:segment + 2])


def build_latest(output, farm_source, house_scale=.5, layout_path=LAYOUT):
    from build_scene import new_stage, mesh, sha
    data = json.loads(layout_path.read_text())
    placements = data['neighbor_placements']
    if len(placements) != 68:
        raise ValueError('Unexpected reviewed layout')
    assets = output.parent
    records = []
    inputs = {str(p): sha(p) for p in (layout_path, STEM_V8, STEM_V9, farm_source)}
    v9 = read_glb(STEM_V9, True)
    # Per-object v9 area-weighted colours, also used to recolour the legacy stem.
    palette = {}
    for name, v, f, c in v9:
        area = np.linalg.norm(np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]]), axis=1).sum()
        acc = palette.setdefault(name, [np.zeros(4), 0.])
        acc[0] += np.array(c) * area
        acc[1] += area
    palette = {k: (v[0] / v[1]).tolist() for k, v in palette.items() if v[1] > 0}
    main = next(v for n, v, _, _ in v9 if n == 'MainStem')
    source_points, source_radii = tube_centerline(main @ FIXED.T * .5, 24)
    anchor = source_points[0]
    points, radii = resample_rod(source_points - anchor, source_radii, 16)
    v9 = [(n, v @ FIXED.T * .5 - anchor, f, c) for n, v, f, c in v9]

    source = Usd.Stage.Open(str(STEM_V8))
    cache = UsdGeom.XformCache()
    v8 = []
    color_aliases = {}
    for p in source.Traverse():
        if not p.IsA(UsdGeom.Mesh) or not p.GetName().startswith('STEM_'):
            continue
        m = UsdGeom.Mesh(p)
        counts = np.asarray(m.GetFaceVertexCountsAttr().Get())
        if not np.all(counts == 3):
            raise ValueError('Expected original triangular stem mesh')
        tf = np.asarray(cache.GetLocalToWorldTransform(p))
        v = (np.asarray(m.GetPointsAttr().Get()) @ tf[:3, :3] + tf[3, :3]) * .5
        name = p.GetName().removeprefix('STEM_')
        color_name = name
        # USD import suffixes repeated primitives with numbers or material hashes.
        if color_name not in palette and name.rsplit('_', 1)[0] in palette:
            color_name = name.rsplit('_', 1)[0]
        if color_name not in palette:
            raise ValueError('Missing v9 colour for ' + name)
        if color_name != name:
            color_aliases[name] = color_name
        f = np.asarray(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        v8.append((name, v - anchor, f, palette[color_name]))
    legacy_main = next(v for n, v, _, _ in v8 if n == 'MainStem')
    legacy_points, _ = tube_centerline(legacy_main, 24)
    skeleton_error = float(np.max(np.abs(legacy_points - (source_points - anchor))))
    if skeleton_error > 1e-5:
        raise ValueError('Stem skeleton mismatch: ' + str(skeleton_error))

    def template(name, meshes):
        path = assets / (name + '.usdc')
        s = new_stage(path, '/Plant')
        inventory = []
        for i, (label, v, f, c) in enumerate(meshes):
            prim = mesh(s, '/Plant/mesh_' + str(i), v, f, c[:3], normals=colored_normals(v, f))
            prim.GetPrim().SetCustomDataByKey('sourceName', label)
            inventory.append(dict(name=label, vertices=len(v), triangles=len(f)))
        s.GetRootLayer().Save()
        record = dict(asset=path.name, primitives=len(meshes), triangles=sum(x['triangles'] for x in inventory),
                      vertices=sum(x['vertices'] for x in inventory), inventory=inventory)
        records.append(record)
        return path

    stems = [template('stem_v8_recolored', v8), template('stem_v9_full', v9)]
    trusses = {}
    for plant in placements:
        for truss in plant['trusses']:
            filename = Path(truss['source_glb']).name
            if filename not in trusses:
                path = GLB_DIR / filename
                if sha(path) != truss['source_sha256']:
                    raise ValueError('Reviewed GLB hash mismatch: ' + filename)
                inputs[str(path)] = sha(path)
                meshes, ped = truss_meshes(path, truss)
                trusses[filename] = (template('truss_' + str(len(trusses)), meshes), ped, truss)
    s = new_stage(output, '/World')
    # Original house structure only: the old 54 plants are not composed here.
    house = UsdGeom.Xform.Define(s, '/World/House')
    house.AddScaleOp().Set(Gf.Vec3d(house_scale))
    # External prototype references below Environment are instanceable. Collision
    # APIs are authored on their shared mesh definitions in a dedicated sublayer.
    collision = assets / 'house_collision.usda'
    hs = Usd.Stage.CreateNew(str(collision))
    hs.GetRootLayer().subLayerPaths = [str(farm_source)]
    hs.SetDefaultPrim(hs.GetPrimAtPath('/World'))
    for p in hs.TraverseAll():
        if p.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('none')
    hs.GetRootLayer().Save()
    house.GetPrim().GetReferences().AddReference(str(collision))
    s.OverridePrim('/World/House/Plants').SetActive(False)

    # The reviewed layout used a 0.5 house. Only root positions follow house size;
    # stems/trusses remain at the approved physical 0.5 GLB size, never doubled.
    root_scale = house_scale / .5
    UsdGeom.Xform.Define(s, '/World/Plants')
    output_placements = []
    max_attachment_error = 0.
    for plant in placements:
        name = '/World/Plants/Plant_' + f"{plant['id']:02d}"
        p = UsdGeom.Xform.Define(s, name)
        pos = np.array(plant['position']) * root_scale
        yaw = plant['yaw_deg']
        p.AddTranslateOp().Set(Gf.Vec3d(*pos))
        p.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(yaw)
        st = UsdGeom.Xform.Define(s, name + '/Stem')
        st.GetPrim().GetReferences().AddReference(str(stems[plant['stem_variant']]))
        st.GetPrim().SetInstanceable(True)
        placed = []
        for i, truss in enumerate(plant['trusses']):
            path, ped, profile = trusses[Path(truss['source_glb']).name]
            for key in ('remove_fruits', 'fruit_offsets', 'truss_scale'):
                if truss.get(key, {}) != profile.get(key, {}):
                    raise ValueError('Inconsistent shared GLB profile')
            # Undo only the plant yaw to preserve the reviewed surface point.
            ry = Rotation.from_euler('z', yaw, degrees=True)
            local = ry.inv().apply(np.array(truss['attachment_world_m']) - plant['position'])
            expected = surface_attachment(points, radii, truss['stem_segment'], truss['stem_fraction'], ped, truss['y_deg'])
            error = float(np.linalg.norm(local - expected))
            max_attachment_error = max(max_attachment_error, error)
            if error > 1e-5:
                raise ValueError('Stored attachment does not match original stem surface: ' + str(error))
            t = UsdGeom.Xform.Define(s, name + f'/Truss_{i:02d}')
            t.GetPrim().GetReferences().AddReference(str(path))
            t.GetPrim().SetInstanceable(True)
            t.AddTranslateOp().Set(Gf.Vec3d(*local))
            # Rx(90) Ry(theta) == Rz(theta) Rx(90). Template includes only Rx(90).
            t.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(truss['y_deg'])
            placed.append(dict(**truss, new_attachment_world_m=(pos + ry.apply(local)).tolist(),
                               attachment_error_m=error))
        output_placements.append(dict(id=plant['id'], stem_variant=plant['stem_variant'],
                                      position=pos.tolist(), yaw_deg=yaw, trusses=placed))
    s.GetRootLayer().Save()
    report = dict(mode='reviewed_v8_v9_glb_full', plants=len(placements),
                  stem_variants=dict(Counter(p['stem_variant'] for p in placements)),
                  trusses=sum(len(p['trusses']) for p in placements), templates=records,
                  source_sha256=inputs, source_layout=str(layout_path), house_scale=house_scale,
                  root_position_scale_from_reviewed_layout=root_scale, stem_glb_scale=.5,
                  truss_glb_scale=.5, max_attachment_error_m=max_attachment_error,
                  stem_centerline_consistency_m=skeleton_error, geometry_reduction=False,
                  legacy_color_primitive_aliases=color_aliases,
                  triangle_collision=True, placements=output_placements,
                  dynamics='full original visual triangles as fixed collision meshes; elasticity not attached',
                  scope='plant asset/layout correction; no harvesting or contact validation')
    output.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return report
