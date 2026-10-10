"""Source topology and actual composed USD frame checks for the plant adapter."""
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics
from latest_farm import STEM_V8, STEM_V9, GLB_DIR, FIXED, read_glb, tube_centerline, truss_meshes


def validate_latest(folder, stage, report):
    checks = {}
    assets = folder / 'assets'
    checks['old_farm_plants_inactive'] = not stage.GetPrimAtPath('/World/Greenhouse/House/Plants').IsActive()
    plants = stage.GetPrimAtPath('/World/Greenhouse/Plants')
    checks['reviewed_68_plants_present'] = len(plants.GetChildren()) == 68
    checks['both_stem_variants_present'] = dict(Counter(p['stem_variant'] for p in report['placements'])) == {0: 36, 1: 32}
    cache = UsdGeom.XformCache()
    frame_errors = []
    for p in report['placements']:
        path = '/World/Greenhouse/Plants/Plant_' + f"{p['id']:02d}"
        world = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)))
        frame_errors.append(np.max(np.abs(world[3, :3] - p['position'])))
        for i, t in enumerate(p['trusses']):
            prim = stage.GetPrimAtPath(path + f'/Truss_{i:02d}')
            tf = np.asarray(cache.GetLocalToWorldTransform(prim))
            # Source GLB Y-up -> world Z-up, plus only the chosen GLB Y rotation
            # and the required opposite-side yaw of the entire plant.
            expected = Rotation.from_euler('z', p['yaw_deg'], degrees=True).as_matrix() @ FIXED @ Rotation.from_euler('y', t['y_deg'], degrees=True).as_matrix()
            actual = tf[:3, :3].T @ FIXED
            frame_errors.append(np.max(np.abs(actual - expected)))
            frame_errors.append(np.max(np.abs(tf[3, :3] - t['new_attachment_world_m'])))
    checks['composed_Y_only_rotation_and_attachment_frames'] = max(frame_errors) < 1e-9
    checks['stored_surface_attachment_matches_source_stem'] = report['max_attachment_error_m'] < 1e-5
    checks['stem_and_truss_size_remain_half'] = report['stem_glb_scale'] == report['truss_glb_scale'] == .5
    checks['truss_angle_range_0_180'] = all(0 <= t['y_deg'] <= 180 for p in report['placements'] for t in p['trusses'])
    checks['reviewed_106_trusses_present'] = sum(len(p['trusses']) for p in report['placements']) == 106
    raw_v9 = read_glb(STEM_V9, True)
    anchor = tube_centerline(next(v for n, v, _, _ in raw_v9 if n == 'MainStem') @ FIXED.T * .5, 24)[0][0]
    sources = {'stem_v9_full.usdc': [(n, v @ FIXED.T * .5 - anchor, f, c) for n, v, f, c in raw_v9]}
    old = Usd.Stage.Open(str(STEM_V8))
    cc = UsdGeom.XformCache()
    legacy = []
    for p in old.Traverse():
        if p.IsA(UsdGeom.Mesh) and p.GetName().startswith('STEM_'):
            m = UsdGeom.Mesh(p); tf = np.asarray(cc.GetLocalToWorldTransform(p))
            v = (np.asarray(m.GetPointsAttr().Get()) @ tf[:3, :3] + tf[3, :3]) * .5 - anchor
            legacy.append((p.GetName().removeprefix('STEM_'), v, np.asarray(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3), None))
    sources['stem_v8_recolored.usdc'] = legacy
    truss_order = {}
    for p in report['placements']:
        for t in p['trusses']:
            filename = Path(t['source_glb']).name
            if filename not in truss_order:
                asset = 'truss_' + str(len(truss_order)) + '.usdc'
                truss_order[filename] = asset
                sources[asset] = truss_meshes(GLB_DIR / filename, t)[0]
    topology = True; finite = True; collisions = True; fruits = True
    triangles = {}; point_error = 0.
    for asset, meshes in sources.items():
        s = Usd.Stage.Open(str(assets / asset))
        usd_meshes = [p for p in s.Traverse() if p.IsA(UsdGeom.Mesh)]
        topology &= len(usd_meshes) == len(meshes)
        triangles[asset] = 0
        fruit_names = []
        for i, (name, v, faces, _) in enumerate(meshes):
            p = s.GetPrimAtPath('/Plant/mesh_' + str(i)); m = UsdGeom.Mesh(p)
            actual = np.asarray(m.GetPointsAttr().Get())
            topology &= np.array_equal(np.asarray(m.GetFaceVertexIndicesAttr().Get()), faces.ravel())
            topology &= len(actual) == len(v) and np.all(np.asarray(m.GetFaceVertexCountsAttr().Get()) == 3)
            point_error = max(point_error, float(np.max(np.abs(actual - v))))
            finite &= np.isfinite(actual).all() and np.isfinite(np.asarray(m.GetNormalsAttr().Get())).all()
            collisions &= p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() == 'none'
            triangles[asset] += len(faces)
            if name.startswith('Fruit_'): fruit_names.append(name)
        if asset.startswith('truss_'):
            fruits &= len(fruit_names) == 10 and 'Fruit_06' not in fruit_names
    checks['all_original_included_vertices_and_faces_preserved'] = bool(topology and point_error < 1e-6)
    checks['full_v9_524809_triangles_preserved'] = triangles['stem_v9_full.usdc'] == 524809
    checks['finite_plant_vertices_and_normals'] = bool(finite)
    checks['full_triangle_collisions_attached'] = bool(collisions)
    checks['five_truss_profiles_have_ten_fruits_without_06'] = len(truss_order) == 5 and bool(fruits)
    return checks, dict(template_triangles=triangles, max_vertex_roundoff_m=point_error,
                        max_frame_error=max(frame_errors), elasticity_validated=False)
