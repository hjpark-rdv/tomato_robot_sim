"""Recover authored per-vertex robot colors for an existing packed mesh.

Run this offline helper with .asset-venv. The geometry and physics are retained;
source/compiled vertex and face correspondence must agree before colors copy.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def restore(export_path, geometry_path, output_path, report_path):
    from pxr import Usd, UsdGeom, UsdPhysics

    export = json.loads(export_path.read_text())
    source = Path(export['source_assembly'])/'composed.usdc'
    if digest(source) != export['source_usdc_sha256']:
        raise ValueError('Authored robot color source hash mismatch')
    stage = Usd.Stage.Open(str(source))
    cache = UsdGeom.XformCache()
    with np.load(geometry_path, allow_pickle=False) as arrays:
        data = {k: arrays[k] for k in arrays.files}
    rows = {r['name']: r for r in export['geometry']}
    names, colors, reports = [], [], []
    for i, name in enumerate(data['geom_names']):
        row = rows.get(str(name))
        if not row or not row.get('owner', '').startswith(('Robot_', 'Tool_')):
            continue
        prim = stage.GetPrimAtPath(row['source'])
        if not prim or not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        authored = mesh.GetDisplayColorPrimvar().ComputeFlattened()
        if authored is None:
            continue
        source_colors = np.asarray(authored, float)
        if len(source_colors) <= 1:
            continue
        if mesh.GetDisplayColorPrimvar().GetInterpolation() != 'vertex':
            raise ValueError('Unsupported authored color interpolation: '+row['source'])
        vertices = np.asarray(mesh.GetPointsAttr().Get(), float)
        if source_colors.shape != vertices.shape or not np.isfinite(source_colors).all():
            raise ValueError('Invalid per-vertex color array: '+row['source'])
        owner = prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner = owner.GetParent()
        if not owner:
            raise ValueError('No authored rigid owner for robot colors')
        world = np.asarray(cache.GetLocalToWorldTransform(prim)).T
        parent = np.asarray(cache.GetLocalToWorldTransform(owner)).T
        local = np.linalg.inv(parent) @ world
        original = vertices @ local[:3, :3].T + local[:3, 3]
        mid = int(data['geom_dataid'][i])
        if mid < 0:
            raise ValueError('Colored source mesh is no longer a triangle mesh')
        va, vn = int(data['mesh_vertadr'][mid]), int(data['mesh_vertnum'][mid])
        fa, fn = int(data['mesh_faceadr'][mid]), int(data['mesh_facenum'][mid])
        rotation = Rotation.from_quat(data['geom_quat'][i][[1, 2, 3, 0]]).as_matrix()
        compiled = data['mesh_vert'][va:va+vn] @ rotation.T + data['geom_pos'][i]
        if compiled.shape != original.shape:
            raise ValueError('Compiled vertex count differs from authored colors')
        error = float(np.max(np.abs(compiled-original)))
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), int)
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), int)
        if error > 2e-6 or not np.all(counts == 3) or len(counts) != fn or not np.array_equal(
                data['mesh_face'][fa:fa+fn], indices.reshape(-1, 3)):
            raise ValueError(f'Source/compiled mesh correspondence failed: {name}, {error}')
        rgba = np.c_[np.rint(np.clip(source_colors, 0, 1)*255).astype(np.uint8),
                     np.full(vn, round(float(data['geom_rgba'][i, 3])*255), np.uint8)]
        authored_color_count = int(len(np.unique(rgba, axis=0)))
        # In this CAD, yellow marks the 32 oblique rubber rollers; blue marks
        # fasteners, hubs and side plates. Restore their physical appearance.
        world_vertices = vertices @ world[:3, :3].T + world[:3, 3]
        wheel = (abs(world_vertices[:, 0]) > .28) & (world_vertices[:, 2] < .25)
        yellow = (rgba[:, 0] > 240) & (rgba[:, 1] > 240) & (rgba[:, 2] < 100)
        blue_metal = (rgba[:, 0] > 100) & (rgba[:, 2].astype(int)-rgba[:, 0] > 12) & (
            rgba[:, 2].astype(int)-rgba[:, 1] > 5)
        rubber = wheel & yellow
        metal = wheel & blue_metal
        rgba[rubber, :3] = [7, 9, 12]
        rgba[metal, :3] = [160, 166, 174]
        names.append(str(name)); colors.append(rgba)
        reports.append(dict(geom=str(name), source=row['source'], vertex_count=vn,
            authored_color_count=authored_color_count,
            rubber_roller_vertices_darkened=int(np.count_nonzero(rubber)),
            wheel_metal_vertices_neutralized=int(np.count_nonzero(metal)),
            source_compiled_vertex_max_error_m=error))
        # Scalar-only consumers get a neutral average rather than the first
        # vertex's yellow. The actual viewer uses the complete color array.
        data['geom_rgba'][i, :3] = rgba[:, :3].mean(0)/255.
        data['geom_matid'][i] = -1
    if not names:
        raise ValueError('No authored multi-vertex color meshes found')
    data.update(vertex_color_geom_names=np.asarray(names),
        vertex_color_offsets=np.r_[0, np.cumsum([len(c) for c in colors])],
        vertex_color_rgba=np.concatenate(colors))
    np.savez_compressed(output_path, **data)
    report = dict(schema='s300-authored-robot-vertex-colors-v1', source_usdc=str(source),
        source_usdc_sha256=export['source_usdc_sha256'], geometry_unchanged=True,
        physics_changed=False, meshes=reports, output_geometry_sha256=digest(output_path),
        wheel_palette='Dark rubber rollers; neutral silver hubs, plates and fasteners',
        controls='Authored screen, buttons and chassis vertex colors retained')
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('export', type=Path)
    parser.add_argument('geometry', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    restore(args.export, args.geometry, args.output, args.report)
