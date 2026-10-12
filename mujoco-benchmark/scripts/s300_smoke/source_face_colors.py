"""Preserve authored USD face materials and face-constant vertex colours.

The palette is a loss-bounded encoding of source colours, not a painted texture.
It changes only render materials and UVs on existing meshes. Each triangle uses
one palette texel, retaining the recorded vertices, faces, normals and physics.
Unsupported gradients or differing PBR constants fail explicitly.
"""
from __future__ import annotations

import numpy as np


def extract_face_colors(prim, read_material):
    from pxr import UsdGeom, UsdShade
    if not prim.IsA(UsdGeom.Mesh):
        return None
    mesh = UsdGeom.Mesh(prim)
    subsets = UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets()
    color = mesh.GetDisplayColorPrimvar()
    colors = color.ComputeFlattened() if color else None
    base = read_material(prim)
    vertex_colors = colors is not None and len(colors) > 1 and not base['material_path']
    if not subsets and not vertex_colors:
        return None
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), np.int64)
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), np.int64)
    points = np.asarray(mesh.GetPointsAttr().Get(), float)
    starts = np.r_[0, np.cumsum(counts)]
    corners, polygon = [], []
    for i, count in enumerate(counts):
        for j in range(1, count - 1):
            corners.append([starts[i], starts[i] + j, starts[i] + j + 1])
            polygon.append(i)
    corners = np.asarray(corners, np.int64)
    polygon = np.asarray(polygon, np.int64)
    faces = indices[corners]
    face_rgba = np.tile(base['rgba'], (len(faces), 1))
    records = []
    materials = [base]
    if vertex_colors:
        colors = np.asarray(colors, float)
        interpolation = str(color.GetInterpolation())
        if interpolation in ('vertex', 'varying'):
            corner_colors = colors[faces]
        elif interpolation == 'faceVarying':
            corner_colors = colors[corners]
        elif interpolation == 'uniform':
            corner_colors = np.repeat(colors[polygon, None], 3, axis=1)
        else:
            raise ValueError('Unsupported displayColor interpolation: ' + interpolation)
        if not np.isfinite(corner_colors).all():
            raise ValueError('Nonfinite source vertex colour')
        if not np.all(corner_colors == corner_colors[:, :1]):
            raise ValueError('Source colour gradient requires vertex interpolation; cannot use a face palette')
        face_rgba[:, :3] = corner_colors[:, 0]
    covered = np.zeros(len(counts), bool)
    effective = []
    for subset in subsets:
        selected = np.asarray(subset.GetIndicesAttr().Get(), np.int64)
        if np.any(selected < 0) or np.any(selected >= len(counts)) or np.any(covered[selected]):
            raise ValueError('Overlapping or invalid source material face subset')
        covered[selected] = True
        material = read_material(subset.GetPrim())
        if not material['material_path']:
            raise ValueError('Source material subset has no bound material')
        face_rgba[np.isin(polygon, selected)] = material['rgba']
        effective.append(material)
        records.append(dict(source=str(subset.GetPath()), faces=len(selected), material=material))
    if effective:
        materials = effective + ([] if covered.all() else [base])
    reference = materials[0]
    for material in materials:
        if material['connected_channels']:
            raise ValueError('Textured subset cannot be replaced by a colour palette')
        if any(material[k] != reference[k] for k in ('roughness', 'metallic', 'color_space')):
            raise ValueError('Face materials with differing PBR constants require separate render materials')
    if np.any(face_rgba[:, 3] != 1):
        raise ValueError('Nonopaque face colours need a separate opacity representation')
    palette, labels = np.unique(face_rgba, axis=0, return_inverse=True)
    return dict(vertices=points, faces=faces, labels=labels.astype(np.int32),
                palette=palette, material=reference, subsets=records,
                source=str(prim.GetPath()), triangles=len(faces),
                subset_faces=int(covered.sum()), source_polygons=len(counts),
                display_color_interpolation=str(color.GetInterpolation()) if vertex_colors else None,
                method='material_subsets' if subsets else 'face_constant_vertex_colors')


def verify_surface_mapping(original, geom_id, vertices, faces, tolerance=2e-6):
    """Prove source/body triangles correspond to compiled mesh face ordering."""
    import mujoco as mj
    mesh_id = int(original.geom_dataid[geom_id])
    if original.geom_type[geom_id] != mj.mjtGeom.mjGEOM_MESH:
        raise ValueError('Face colour source is not a native mesh')
    va = int(original.mesh_vertadr[mesh_id]); vn = int(original.mesh_vertnum[mesh_id])
    fa = int(original.mesh_faceadr[mesh_id]); fn = int(original.mesh_facenum[mesh_id])
    rotation = np.empty(9)
    mj.mju_quat2Mat(rotation, original.geom_quat[geom_id])
    compiled = (original.mesh_vert[va:va + vn].astype(float) @ rotation.reshape(3, 3).T
                + original.geom_pos[geom_id])
    actual = compiled[original.mesh_face[fa:fa + fn]]
    expected = vertices[faces]
    if actual.shape != expected.shape:
        raise ValueError('Source/native face topology mismatch')
    error = float(np.max(np.abs(actual - expected)))
    if not np.isfinite(error) or error > tolerance:
        raise ValueError(f'Source/native face mapping mismatch: {error:g} m')
    return mesh_id, error


def apply_face_palettes(spec, original, records, arrays, encode_rgba):
    import mujoco as mj
    changed, evidence = set(), []
    for row in records:
        key = row['key']; name = row['geometry']
        geom_id = mj.mj_name2id(original, mj.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            raise ValueError('Missing face-coloured source geometry: ' + name)
        mesh_id, error = verify_surface_mapping(original, geom_id,
            arrays[key + '_vertices'], arrays[key + '_faces'])
        if mesh_id in changed:
            raise ValueError('Shared mesh requires distinct face palette validation')
        changed.add(mesh_id)
        palette = np.asarray(arrays[key + '_palette'])
        encoded = np.array([encode_rgba(dict(row['material'], rgba=rgba.tolist())) for rgba in palette])
        if np.any(encoded < 0) or np.any(encoded > 1):
            raise ValueError('Source colour exceeds supported palette texture range')
        rgb8 = np.rint(encoded[:, :3] * 255).astype(np.uint8)
        # Every face has constant UV: no surface derivatives or neighbouring
        # palette interpolation. Padding also keeps samples away from borders.
        texture = np.repeat(np.repeat(rgb8[:, None, :], 16, axis=0), 16, axis=1)
        texture_name = key + '_palette'
        spec.add_texture(name=texture_name, type=mj.mjtTexture.mjTEXTURE_2D,
            width=16, height=len(palette) * 16, nchannel=3,
            colorspace=mj.mjtColorSpace.mjCOLORSPACE_SRGB,
            data=texture.ravel().tolist())
        material = spec.add_material(name=key + '_material', rgba=[1, 1, 1, 1],
            roughness=row['material']['roughness'], metallic=row['material']['metallic'],
            shininess=1 - row['material']['roughness'],
            specular=.3 + .5 * row['material']['metallic'])
        material.textures[mj.mjtTextureRole.mjTEXROLE_RGB] = texture_name
        uv = np.column_stack((np.full(len(palette), .5), (np.arange(len(palette)) + .5) / len(palette)))
        labels = arrays[key + '_labels']
        mesh = spec.mesh(mj.mj_id2name(original, mj.mjtObj.mjOBJ_MESH, mesh_id))
        mesh.usertexcoord = uv.ravel().tolist()
        mesh.userfacetexcoord = np.repeat(labels[:, None], 3, axis=1).ravel().tolist()
        spec.geom(name).material = material.name
        spec.geom(name).rgba = [1, 1, 1, 1]
        evidence.append(dict(geometry=name, source=row['source'], method=row['method'],
            triangles=len(labels), colours=len(palette), subsets=len(row['subsets']),
            source_native_triangle_max_error_m=error,
            encoded_rgb_max_quantization_error=float(np.max(np.abs(rgb8 / 255. - encoded[:, :3])))))
    return changed, evidence
