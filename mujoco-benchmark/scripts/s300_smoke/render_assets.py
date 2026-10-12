"""Recover authored USD materials and blended branch skin for visual replay only.

The saved physics MJB remains immutable. A separate render cache adds materials
and original skin assets to the existing XML, then copies the exact recorded
physical parameter arrays back from the MJB. No simulation step is performed.
USD extraction runs in the existing asset Python; normal replay needs no pxr.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import weakref
import xml.etree.ElementTree as ET

import numpy as np

SCHEMA = 's300-authored-render-assets-v6'
DEFAULT_CACHE = Path('/root/docker_share/mujoko_debugging_data/20261012_s300_mujoco_quality/asset_cache')
_TRANSMISSION_CONTEXTS = weakref.WeakKeyDictionary()
ASSET_PYTHON = Path(__file__).resolve().parents[3] / 'nvidia-sim/.asset-venv/bin/python'
# These arrays are appearance only. geom_group is intentionally preserved,
# because filtering a group must never silently change its collision metadata.
VISUAL_ARRAY_PREFIXES = ('mat_', 'tex_', 'skin_', 'light_', 'cam_')
VISUAL_ARRAY_NAMES = {'geom_rgba', 'geom_matid', 'mesh_normal', 'mesh_facenormal',
                      'mesh_texcoord', 'mesh_facetexcoord', 'mesh_texcoordadr', 'mesh_texcoordnum'}
PHYSICS_PREFIXES = ('body_', 'jnt_', 'dof_', 'geom_', 'mesh_', 'eq_', 'tendon_',
                    'actuator_', 'pair_', 'exclude_', 'flex_', 'sensor_', 'hfield_')
PHYSICS_SCALARS = ('nq', 'nv', 'nu', 'na', 'nbody', 'njnt', 'ngeom', 'nmesh',
                   'neq', 'ntendon', 'nsensor', 'nmocap', 'nflex', 'npair', 'nexclude')


def file_sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def _json_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def physics_arrays(model):
    """All native physical arrays; render-only skins/material arrays excluded."""
    for name in sorted(dir(model)):
        if not name.startswith(PHYSICS_PREFIXES) or name in VISUAL_ARRAY_NAMES:
            continue
        value = getattr(model, name)
        if isinstance(value, np.ndarray):
            yield name, value


def physics_parameter_hash(model):
    digest = hashlib.sha256()
    for name in PHYSICS_SCALARS:
        digest.update((name + ':' + str(getattr(model, name))).encode())
    for name, value in physics_arrays(model):
        digest.update(name.encode()); digest.update(str(value.dtype).encode())
        digest.update(str(value.shape).encode()); digest.update(value.tobytes())
    for name in sorted(dir(model.opt)):
        if name.startswith('_'):
            continue
        value = getattr(model.opt, name)
        if isinstance(value, np.ndarray):
            digest.update(name.encode()); digest.update(value.tobytes())
        elif isinstance(value, (int, float, bool)):
            digest.update((name + ':' + repr(value)).encode())
    return digest.hexdigest()


def restore_physics_parameters(source, destination):
    """Retain recorded collision masks, solver options and physical ordering."""
    for name in PHYSICS_SCALARS:
        if getattr(source, name) != getattr(destination, name):
            raise ValueError('Render model changed physical topology: ' + name)
    import mujoco as mj
    for kind, count in ((mj.mjtObj.mjOBJ_BODY, source.nbody),
                        (mj.mjtObj.mjOBJ_JOINT, source.njnt),
                        (mj.mjtObj.mjOBJ_GEOM, source.ngeom),
                        (mj.mjtObj.mjOBJ_MESH, source.nmesh),
                        (mj.mjtObj.mjOBJ_ACTUATOR, source.nu),
                        (mj.mjtObj.mjOBJ_EQUALITY, source.neq)):
        if any(mj.mj_id2name(source, kind, i) != mj.mj_id2name(destination, kind, i)
               for i in range(count)):
            raise ValueError('Render model changed physical object ordering: ' + str(kind))
    for name, array in physics_arrays(source):
        target = getattr(destination, name)
        if target.shape != array.shape or target.dtype != array.dtype:
            raise ValueError('Render model physical array shape changed: ' + name)
        target[:] = array
    for name in dir(source.opt):
        if name.startswith('_'):
            continue
        value = getattr(source.opt, name)
        if isinstance(value, np.ndarray):
            getattr(destination.opt, name)[:] = value
        elif isinstance(value, (int, float, bool)):
            setattr(destination.opt, name, value)
    if physics_parameter_hash(source) != physics_parameter_hash(destination):
        raise ValueError('Render model physical parameter hash differs')


def read_usd_material(prim):
    """Read authored PreviewSurface or OmniPBR constants, without recolouring."""
    from pxr import Usd, UsdGeom, UsdShade
    color_attribute = UsdGeom.Gprim(prim).GetDisplayColorAttr()
    colors = color_attribute.Get()
    if not colors:
        color_attribute = None
    rgba = [*map(float, colors[0]), 1.] if colors else [.5, .5, .5, 1.]
    opacity = UsdGeom.Gprim(prim).GetDisplayOpacityAttr().Get()
    if opacity:
        rgba[3] = float(opacity[0])
    targets = prim.GetRelationship('material:binding').GetTargets()
    material = UsdShade.Material(prim.GetStage().GetPrimAtPath(targets[0])) if targets else None
    if not material:
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    shader = material.ComputeSurfaceSource()[0] if material else None
    shader_kind = 'displayColor'
    if material and not shader:
        # MDL bindings have no universal surface output. Inspect their authored
        # Shader, rather than treating the white lift CAD as generic grey.
        shaders = [UsdShade.Shader(child) for child in material.GetPrim().GetChildren()
                   if child.IsA(UsdShade.Shader)]
        shader = shaders[0] if shaders else None
    roughness, metallic = .5, 0.
    authored, connections = {}, []
    if shader:
        shader_kind = str(shader.GetIdAttr().Get() or
                          shader.GetPrim().GetAttribute('info:mdl:sourceAsset:subIdentifier').Get() or
                          'unknown_shader')
        names = dict(rgb=('diffuseColor', 'diffuse_color_constant'),
                     alpha=('opacity', 'opacity_constant'),
                     roughness=('roughness', 'reflection_roughness_constant'),
                     metallic=('metallic', 'metallic_constant'))
        for channel, choices in names.items():
            for key in choices:
                input_ = shader.GetInput(key)
                if not input_:
                    continue
                if input_.HasConnectedSource():
                    connections.append(key)
                value = input_.Get()
                if value is not None:
                    if channel == 'rgb':
                        rgba[:3] = list(map(float, value))
                        color_attribute = input_.GetAttr()
                    elif channel == 'alpha':
                        rgba[3] = float(value)
                    elif channel == 'roughness':
                        roughness = float(value)
                    else:
                        metallic = float(value)
                    authored[channel] = key
                    break
    explicit_color_space = str(color_attribute.GetColorSpace()) if color_attribute else ''
    # USD colour attributes are scene-linear Rec.709 when no colour space is
    # explicitly declared. MuJoCo Filament reads its BaseColorFactor as sRGB.
    resolved_color_space = str(Usd.ColorSpaceAPI.ComputeColorSpaceName(color_attribute, None)) if color_attribute else ''
    color_space = resolved_color_space or explicit_color_space or ('linear_rec709' if color_attribute else 'render_fallback_srgb')
    if shader:
        connections = sorted(set(connections) | {str(i.GetBaseName()) for i in shader.GetInputs() if i.HasConnectedSource()})
    result = dict(rgba=rgba, roughness=roughness, metallic=metallic,
                  color_space=color_space, explicit_color_space=explicit_color_space,
                  inherited_or_explicit_color_space=resolved_color_space,
                  color_attribute=str(color_attribute.GetPath()) if color_attribute else '',
                  material_path=str(material.GetPath()) if material else '',
                  shader_kind=shader_kind, authored_channels=authored,
                  connected_channels=connections,
                  missing_pbr_defaults={'roughness': .5, 'metallic': 0.})
    if not np.isfinite([*rgba, roughness, metallic]).all():
        raise ValueError('Nonfinite source material: ' + str(prim.GetPath()))
    result['source_material_sha256'] = _json_sha(result)
    return result


def extract_render_assets(model_dir, output):
    """USD-side extraction: compact metadata plus exact original skin weights."""
    from pxr import Usd, UsdGeom
    model_dir, output = Path(model_dir), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    meta = json.loads((model_dir / 'export.json').read_text())
    xml = ET.parse(model_dir / 'scene.xml').getroot()
    xml_geoms = {g.get('name'): g for g in xml.iter('geom') if g.get('name')}
    xml_materials = {m.get('name'): m for m in xml.findall('asset/material')}
    stages = {}
    source_files = {}
    def stage(assembly):
        path = Path(assembly) / 'composed.usdc'
        if str(path) not in stages:
            stages[str(path)] = Usd.Stage.Open(str(path))
            if not stages[str(path)]:
                raise ValueError('Could not open source USD: ' + str(path))
            source_files[str(path)] = file_sha(path)
        return stages[str(path)]

    windows = meta.get('windows') or [dict(prefix='', source_assembly=meta['source_assembly'],
                                         source_manifest=meta['source_manifest'])]
    window_by_prefix = sorted(windows, key=lambda w: len(w['prefix']), reverse=True)
    materials, hidden, skipped = {}, [], []
    for row in meta.get('geometry', []):
        window = next((w for w in window_by_prefix
                       if not w['prefix'] or row['name'].startswith(w['prefix'])), windows[0])
        st = stage(window['source_assembly'])
        prim = st.GetPrimAtPath(row['source'])
        if not prim:
            raise ValueError('Source visual geometry missing: ' + row['source'])
        if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
            hidden.append(row['name'])
        materials[row['name']] = read_usd_material(prim)

    # The exact static farm adapter groups meshes by scalar material. Recover
    # each authored tuple from the source group, matched to its existing MJCF
    # scalar tuple (never infer metalness from an object's class or colour).
    st = stage(meta['source_assembly'])
    roots = {}
    for row in meta.get('farm', {}).get('geometry_inventory', []):
        root_path = row['source_root']
        if root_path not in roots:
            values = []
            prim = st.GetPrimAtPath(root_path)
            for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
                if child.IsA(UsdGeom.Mesh):
                    values.append(read_usd_material(child))
            roots[root_path] = values
        mat_name = xml_geoms[row['name']].get('material')
        original = xml_materials[mat_name]
        rgb = np.fromstring(original.get('rgba', '.5 .5 .5 1'), sep=' ')
        rough = 1. - float(original.get('shininess', '.5'))
        spec = float(original.get('specular', '.3'))
        matches = [v for v in roots[root_path]
                   if np.max(np.abs(np.array(v['rgba']) - rgb)) < 1e-6
                   and abs(v['roughness'] - rough) < 1e-6
                   and abs(.3 + .5 * v['metallic'] - spec) < 1e-6]
        if matches:
            materials[row['name']] = matches[0]
        else:
            # Older farm export did not resolve OmniPBR MDL. Keep its scalar
            # display colour but explicitly label the unbound provenance.
            skipped.append(dict(geometry=row['name'], source=root_path,
                                reason='Existing batched scalar material did not match source tuple'))

    # The source ground cube is emitted separately from batched farm meshes.
    # Its unshaded displayColor needs explicit (declared, unmeasured) PBR
    # defaults too, rather than Filament's legacy shininess conversion.
    for row in meta.get('farm', {}).get('structural_inventory', []):
        if row.get('classification') != 'ground':
            continue
        visible_name = row['name'] + '_visual' if row['name'] + '_visual' in xml_geoms else row['name']
        geom = xml_geoms[visible_name]
        original_rgba = np.fromstring(geom.get('rgba', '.5 .5 .5 1'), sep=' ')
        if len(original_rgba) == 4 and original_rgba[3] == 0:
            continue
        materials[visible_name] = read_usd_material(st.GetPrimAtPath(row['source_path']))

    # Source visual-only transmission components share the original CAD
    # mechanism function. Bind exact source rest transforms, including any
    # authored parent transform, before recovering compiled mesh PCA frames.
    linkage_file = Path(__file__).resolve().parents[3] / 'nvidia-sim/harvest_s300/transmission_geometry.py'
    source_files[str(linkage_file)] = file_sha(linkage_file)
    transforms = UsdGeom.XformCache()
    def usd_tf(path):
        return np.asarray(transforms.GetLocalToWorldTransform(st.GetPrimAtPath(path)), float).T
    tool_world = usd_tf('/World/ActuatedTool/fixed')
    transmission_root = '/World/ActuatedTool/fixed/Transmission'
    components, transmission_geoms = {}, []
    if st.GetPrimAtPath(transmission_root):
        parent_in_tool = np.linalg.inv(tool_world) @ usd_tf(transmission_root)
        for row in meta.get('robot', {}).get('geometry', []):
            if not row['source'].startswith(transmission_root + '/'):
                continue
            if row['collision']:
                skipped.append(dict(source=row['source'], reason='Physical transmission collider is not animated'))
                continue
            component = row['source'][len(transmission_root) + 1:].split('/')[0]
            if component not in ('crank', 'linkL', 'linkR'):
                skipped.append(dict(source=row['source'], reason='Unknown source transmission component'))
                continue
            component_path = transmission_root + '/' + component
            components[component] = dict(source_component_in_tool=(np.linalg.inv(tool_world) @ usd_tf(component_path)).tolist())
            transmission_geoms.append(dict(name=row['name'], component=component))
    else:
        parent_in_tool = np.eye(4)
    transmission = dict(body='Tool_fixed', nominal_gap_m=.044, jaws=['jaw_L', 'jaw_R'],
                        parent_in_tool=parent_in_tool.tolist(), components=components,
                        geoms=transmission_geoms, function_source=str(linkage_file),
                        function_source_sha256=file_sha(linkage_file))
    skins, arrays = [], {}
    for window_index, window in enumerate(windows):
        assembly = Path(window['source_assembly'])
        manifest = json.loads(Path(window['source_manifest']).read_text())
        mapping = {b['path']: window['prefix'] + b['name']
                   for b in manifest['plant']['bodies'] + manifest['plant']['fruits']}
        source_skin = assembly / 'skin.npz'
        source_files[str(source_skin)] = file_sha(source_skin)
        source_files[str(Path(window['source_manifest']))] = file_sha(window['source_manifest'])
        st = stage(assembly)
        with np.load(source_skin, allow_pickle=False) as source:
            for i, path in enumerate(manifest['plant']['skin_paths']):
                ids = source[f'{i}_ids']
                weights = source[f'{i}_weights']
                body_paths = source[f'{i}_body_paths'].tolist()
                if any(p not in mapping for p in body_paths):
                    skipped.append(dict(source=path, reason='Skin includes omitted body chain'))
                    continue
                prim = st.GetPrimAtPath(path)
                mesh = UsdGeom.Mesh(prim)
                counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
                if not np.all(counts == 3):
                    raise ValueError('Original blended skin is not triangulated: ' + path)
                key = f'w{window_index}_skin{i}'
                arrays[key + '_vertices'] = source[f'{i}_vertices']
                arrays[key + '_faces'] = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32).reshape(-1, 3)
                arrays[key + '_rest'] = source[f'{i}_rest']
                arrays[key + '_ids'] = ids
                arrays[key + '_weights'] = weights
                skins.append(dict(key=key, source=path, bodies=[mapping[p] for p in body_paths],
                                  material=read_usd_material(prim), window=window_index,
                                  vertices=len(arrays[key + '_vertices']),
                                  triangles=len(arrays[key + '_faces'])))
    np.savez_compressed(output / 'skin_arrays.npz', **arrays)
    result = dict(schema=SCHEMA, source_model_xml=str(model_dir / 'scene.xml'),
                  source_xml_sha256=file_sha(model_dir / 'scene.xml'),
                  export_sha256=file_sha(model_dir / 'export.json'), source_files=source_files,
                  geometry_materials=materials, source_invisible_geometry=hidden,
                  skins=skins, transmission=transmission, skipped=skipped, source_skin_arrays_sha256=file_sha(output / 'skin_arrays.npz'),
                  limitations=['Connected source textures/normal maps are not synthesized.',
                               'Source RGB/PBR constants are retained; renderer illumination and response differ.'])
    (output / 'render_assets.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def ensure_source_assets(model_dir, cache_root=DEFAULT_CACHE):
    model_dir, cache_root = Path(model_dir), Path(cache_root)
    key = _json_sha(dict(schema=SCHEMA, xml=file_sha(model_dir / 'scene.xml'),
                        export=file_sha(model_dir / 'export.json')))
    folder = cache_root / key
    metadata = folder / 'render_assets.json'
    if not metadata.exists():
        folder.mkdir(parents=True, exist_ok=True)
        with (folder / 'source_extraction.log').open('w') as log:
            subprocess.run([str(ASSET_PYTHON), str(Path(__file__).resolve()),
                            'extract', str(model_dir), str(folder)], stdout=log,
                           stderr=subprocess.STDOUT, check=True)
    assets = json.loads(metadata.read_text())
    if assets['schema'] != SCHEMA or assets['source_xml_sha256'] != file_sha(model_dir / 'scene.xml'):
        raise ValueError('Render source model SHA mismatch')
    for path, expected in assets['source_files'].items():
        if file_sha(path) != expected:
            raise ValueError('Render source asset changed: ' + path)
    if file_sha(folder / 'skin_arrays.npz') != assets['source_skin_arrays_sha256']:
        raise ValueError('Cached source skin SHA mismatch')
    return folder, assets


def prepare_filament_skin_arrays(vertices, faces, ids, weights):
    """Work around MuJoCo 3.15 Filament's indexed-skin allocation defect.

    Its mesh index buffer expects vertex_count == 3 * face_count. Preserve
    every referenced shared vertex and face adjacency, then append harmless
    unreferenced copies until this equality holds. This keeps original smooth
    normals and surface triangles, unlike unindexing every triangle corner.
    Native skin inflate must stay zero, because unused vertices have no normal.
    """
    vertices = np.asarray(vertices); faces = np.asarray(faces)
    ids = np.asarray(ids); weights = np.asarray(weights)
    target = int(faces.size); removed = 0
    if len(vertices) > target:
        used = np.unique(faces)
        lookup = np.full(len(vertices), -1, dtype=np.int32); lookup[used] = np.arange(len(used))
        removed = len(vertices) - len(used)
        vertices, ids, weights = vertices[used], ids[used], weights[used]
        faces = lookup[faces]
    padding = target - len(vertices)
    if padding:
        vertices = np.concatenate((vertices, np.repeat(vertices[:1], padding, axis=0)))
        ids = np.concatenate((ids, np.repeat(ids[:1], padding, axis=0)))
        weights = np.concatenate((weights, np.repeat(weights[:1], padding, axis=0)))
    if len(vertices) != target:
        raise ValueError('Filament indexed skin buffer compatibility failed')
    return vertices, faces, ids, weights, dict(padding_vertices=padding,
                                             removed_unreferenced_vertices=removed)


def filament_rgba(values):
    """Preserve source linear colour through Filament's sRGB API boundary.

    Source raw RGBA stays in sidecar evidence. Alpha, authored roughness and
    metallic remain unchanged. Explicit sRGB/raw values are not re-encoded;
    unknown non-Rec.709 gamut spaces are refused instead of silently guessed.
    """
    rgba = np.array(values['rgba'], dtype=float)
    source = values.get('color_space', 'render_fallback_srgb')
    label = source.lower().replace('_', '').replace('-', '').replace(' ', '')
    linear_spaces = {'linear', 'linearrec709', 'linrec709', 'linrec709scene',
                     'linrec709display', 'linearsrgb', 'scenelinear'}
    encoded_spaces = {'srgb', 'srgbrec709', 'srgbrec709scene', 'srgbrec709display',
                      'renderfallbacksrgb', 'raw'}
    if label in linear_spaces:
        if np.any(rgba[:3] < 0):
            raise ValueError('Negative scene-linear source RGB is unsupported')
        rgb = rgba[:3]
        rgba[:3] = np.where(rgb <= .0031308, 12.92 * rgb,
                            1.055 * np.power(rgb, 1 / 2.4) - .055)
    elif label not in encoded_spaces:
        raise ValueError('Unsupported source material colour space: ' + source)
    return rgba.tolist()


def compile_render_model(original, model_dir, assets, arrays, *, profile=None):
    """Create native visual skins/PBR materials with unchanged physics fields."""
    import mujoco as mj
    if profile and profile.get('backend') == 'filament' and any(
            v.get('connected_channels') for v in list(assets['geometry_materials'].values()) +
            [skin['material'] for skin in assets.get('skins', [])]):
        raise ValueError('Connected source textures are not supported by the current render asset adapter; refusing invented UV/texture appearance')
    spec = mj.MjSpec.from_file(str(Path(model_dir) / 'scene.xml'))
    source_materials = {}
    lighting_policy = None
    if profile and profile.get('backend') == 'filament':
        # A declared neutral display IBL replaces the packaged warm HDR. This
        # affects light/texture arrays only, never physical state or colour.
        spec.add_texture(name='neutral_display_ibl', type=mj.mjtTexture.mjTEXTURE_CUBE,
                         builtin=mj.mjtBuiltin.mjBUILTIN_FLAT, width=32, height=32,
                         rgb1=[.5, .5, .5], rgb2=[.5, .5, .5])
        source_key_light_count = len(spec.lights)
        source_key_light_types = [str(light.type).split('.')[-1].removeprefix('mjLIGHT_').lower() for light in spec.lights]
        for key_light in spec.lights:
            key_light.intensity = 15000.
        spec.worldbody.add_light(name='neutral_daylight_environment',
            type=mj.mjtLightType.mjLIGHT_IMAGE, texture='neutral_display_ibl',
            intensity=15000., castshadow=False)
        lighting_policy = dict(name='neutral_daylight', environment_lux=15000.,
            source_key_light_intensity=15000., source_key_light_intensity_unit='candela',
            source_key_light_types=source_key_light_types,
            source_key_lights=source_key_light_count,
            ibl_rgb=[.5, .5, .5], cubemap_face_size=32, render_only=True,
            interpretation='Neutral display lighting; not a measured farm illumination calibration')
    if profile:
        for name, value in profile.get('numeric_flags', {}).items():
            spec.add_numeric(name=name, data=[value])
        for name, value in profile.get('text_flags', {}).items():
            spec.add_text(name=name, data=value)
    mesh_zero_uv = 0
    if profile and profile.get('backend') == 'filament':
        # MuJoCo Filament's official PBR shader requires UV0 even when all
        # source texture samplers are constants. A zero UV has no effect on
        # these untextured surfaces and prevents per-frame missing-attribute
        # warnings / undefined shader input. Existing source UVs stay intact.
        for mesh_id in range(original.nmesh):
            if original.mesh_texcoordnum[mesh_id] > 0:
                continue
            mesh_name = mj.mj_id2name(original, mj.mjtObj.mjOBJ_MESH, mesh_id)
            mesh = spec.mesh(mesh_name)
            vertex_count = int(original.mesh_vertnum[mesh_id])
            first = int(original.mesh_faceadr[mesh_id]); count = int(original.mesh_facenum[mesh_id])
            mesh.usertexcoord = np.zeros((vertex_count, 2)).ravel().tolist()
            mesh.userfacetexcoord = original.mesh_face[first:first + count].ravel().tolist()
            mesh_zero_uv += 1
    def material(values):
        key = _json_sha(dict(rgba=filament_rgba(values) if profile and profile.get('backend') == 'filament' else values['rgba'], roughness=values['roughness'], metallic=values['metallic']))
        if key not in source_materials:
            name = 'source_render_' + key[:20]
            spec.add_material(name=name, rgba=filament_rgba(values) if profile and profile.get('backend') == 'filament' else values['rgba'], roughness=values['roughness'],
                              metallic=values['metallic'],
                              shininess=1. - values['roughness'],
                              specular=.3 + .5 * values['metallic'])
            source_materials[key] = name
        return source_materials[key]
    for geom_name, values in assets['geometry_materials'].items():
        geom = spec.geom(geom_name)
        if geom is None:
            raise ValueError('Source material geometry missing in render model: ' + geom_name)
        geom.material = material(values)
        # Material colour is authoritative only for source-visible geometry.
        # Hidden colliders still collide in the original physical model.
        geom.rgba = filament_rgba(values) if profile and profile.get('backend') == 'filament' else values['rgba']
    for geom_name in assets['source_invisible_geometry']:
        spec.geom(geom_name).rgba[3] = 0.
        spec.geom(geom_name).material = ''  # Filament material RGBA overrides geom alpha.
    render_skin_vertices = 0; skin_padding = 0; removed_unreferenced_vertices = 0; skin_zero_uv = 0
    for skin in assets['skins']:
        key = skin['key']; ids = arrays[key + '_ids']; weights = arrays[key + '_weights']
        vertices = arrays[key + '_vertices']; faces = arrays[key + '_faces']
        if profile and profile.get('backend') == 'filament':
            vertices, faces, ids, weights, compatibility = prepare_filament_skin_arrays(vertices, faces, ids, weights)
            skin_padding += compatibility['padding_vertices']
            removed_unreferenced_vertices += compatibility['removed_unreferenced_vertices']
        render_skin_vertices += len(vertices)
        skin_texcoord = None
        if profile and profile.get('backend') == 'filament':
            # Current extracted source skins have no UVs or texture bindings.
            # Include padded compatibility vertices in the UV0 buffer too.
            skin_texcoord = np.zeros((len(vertices), 2)).ravel().tolist()
            skin_zero_uv += 1
        vertex_ids, vertex_weights = [], []
        for body_index in range(len(skin['bodies'])):
            influenced_vertices, influences = np.where(ids == body_index)
            vertex_ids.append(influenced_vertices.tolist())
            vertex_weights.append(weights[influenced_vertices, influences].tolist())
        # Source skin_points uses R_body * (vertex - source_rest) + body_pos.
        # Identity bind rotations reproduce that exact source convention.
        spec.add_skin(name=key, material=material(skin['material']), group=2, inflate=0.,
                      vert=vertices.ravel().tolist(), texcoord=skin_texcoord,
                      face=faces.ravel().tolist(),
                      bodyname=skin['bodies'], bindpos=arrays[key + '_rest'].ravel().tolist(),
                      bindquat=np.tile([1., 0., 0., 0.], (len(skin['bodies']), 1)).ravel().tolist(),
                      vertid=vertex_ids, vertweight=vertex_weights)
    model = spec.compile()
    restore_physics_parameters(original, model)
    # Explicitly retain any original UV values/face mapping from the recorded
    # MJB even if they differ from an exported OBJ's visual data.
    for mesh_id in range(original.nmesh):
        count = int(original.mesh_texcoordnum[mesh_id])
        if not count:
            continue
        if int(model.mesh_texcoordnum[mesh_id]) != count:
            raise ValueError('Render model changed an existing source UV topology')
        src_first = int(original.mesh_texcoordadr[mesh_id]); dst_first = int(model.mesh_texcoordadr[mesh_id])
        model.mesh_texcoord[dst_first:dst_first + count] = original.mesh_texcoord[src_first:src_first + count]
        first = int(original.mesh_faceadr[mesh_id]); face_count = int(original.mesh_facenum[mesh_id])
        model.mesh_facetexcoord[first:first + face_count] = original.mesh_facetexcoord[first:first + face_count]
    # Explicit colour alpha restores source-hidden collision proxies without
    # changing geom_contype/conaffinity or any body/joint/contact parameters.
    for geom_name in assets['source_invisible_geometry']:
        geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, geom_name)
        model.geom_rgba[geom_id, 3] = 0.
        model.geom_matid[geom_id] = -1
    return model, dict(materials_added=len(source_materials), original_skin_meshes=len(assets['skins']),
                       original_skin_vertices=sum(s['vertices'] for s in assets['skins']),
                       render_skin_vertices=render_skin_vertices,
                       filament_skin_padding_vertices=skin_padding,
                       removed_unreferenced_skin_vertices=removed_unreferenced_vertices,
                       filament_skin_workaround='Unused padding vertices preserve original indexed surface/smooth normals; inflate=0',
                       hidden_source_collision_visuals=len(assets['source_invisible_geometry']),
                       lighting_policy=lighting_policy,
                       untextured_mesh_zero_uv0_assets=mesh_zero_uv,
                       untextured_skin_zero_uv0_assets=skin_zero_uv,
                       uv0_compatibility='Zero UV0 for untextured meshes/skins; authored UVs retained; official PBR shader requires UV0',
                       source_colour_evidence_preserved=True,
                       physics_parameter_hash_schema='s300-physical-arrays-v2 (all texture coordinate fields excluded)',
                       filament_colour_transfer='USD scene-linear Rec.709 RGB encoded to sRGB at native Filament API; alpha unchanged')


def load_render_model(original, model_dir, *, cache_root=DEFAULT_CACHE, original_mjb=None,
                      quality='balanced'):
    """Return ``(display_model, evidence)``; original MJB/model never modified."""
    import mujoco as mj
    if quality == 'fast':
        return original, dict(quality=quality, source_appearance_recovery=False,
                              physics_parameter_sha256=physics_parameter_hash(original))
    from quality_profiles import quality_profile
    profile = quality_profile(quality)
    started = time.perf_counter()
    before = physics_parameter_hash(original)
    folder, assets = ensure_source_assets(model_dir, cache_root)
    input_mjb_hash = file_sha(original_mjb) if original_mjb else None
    cache_key = _json_sha(dict(source_assets=file_sha(folder / 'render_assets.json'),
                              native_version=mj.__version__, original_mjb=input_mjb_hash,
                              physics_parameter_sha256=before, quality=quality,
                              numeric_flags=profile['numeric_flags'], text_flags=profile['text_flags']))
    cache = folder / ('render_' + cache_key + '.mjb')
    cache_meta = cache.with_suffix('.json')
    if cache.exists() and cache_meta.exists():
        result = json.loads(cache_meta.read_text())
        if file_sha(cache) != result['render_mjb_sha256']:
            raise ValueError('Cached render model SHA mismatch')
        model = mj.MjModel.from_binary_path(str(cache))
        if physics_parameter_hash(model) != before:
            raise ValueError('Cached render model physics differs')
        reused = True
    else:
        with np.load(folder / 'skin_arrays.npz', allow_pickle=False) as arrays:
            model, details = compile_render_model(original, model_dir, assets, arrays, profile=profile)
        mj.mj_saveModel(model, str(cache), None)
        result = dict(schema=SCHEMA, cache_key=cache_key, **details,
                      source_original_mjb_sha256=input_mjb_hash,
                      source_assets_sha256=file_sha(folder / 'render_assets.json'),
                      render_mjb_sha256=file_sha(cache), physics_parameter_sha256=before)
        cache_meta.write_text(json.dumps(result, indent=2) + '\n')
        reused = False
    if physics_parameter_hash(original) != before:
        raise ValueError('Original model changed during render setup')
    if original_mjb and file_sha(original_mjb) != input_mjb_hash:
        raise ValueError('Original saved MJB changed during render setup')
    return model, dict(result, quality=quality, source_appearance_recovery=True,
                       numeric_flags=profile['numeric_flags'], text_flags=profile['text_flags'],
                       cache_reused=reused, render_cache=str(cache),
                       setup_wall_s=time.perf_counter() - started,
                       source_material_count=len(assets['geometry_materials']),
                       connected_source_texture_geoms=sum(bool(v['connected_channels'])
                                                          for v in assets['geometry_materials'].values()),
                       limitations=assets['limitations'], skipped_sources=assets['skipped'],
                       transmission=assets.get('transmission', {}))


def update_render_visuals(model, data, evidence):
    """Animate only source visual linkage geoms from measured slider state.

    Changes derived display geom_xpos/geom_xmat only, after mj_kinematics has
    restored real body poses. No qpos/qvel/constraint/model parameter writes.
    Both Filament and classic mjv_updateScene read these measured world arrays.
    """
    import mujoco as mj
    from scipy.spatial.transform import Rotation as R
    transmission = evidence.get('transmission')
    if not transmission or not transmission.get('geoms'):
        return dict(transmission_visual_geoms=0)
    context = _TRANSMISSION_CONTEXTS.get(model)
    if context is None:
        context = {}
        helper = Path(transmission['function_source'])
        if file_sha(helper) != transmission['function_source_sha256']:
            raise ValueError('Source CAD linkage function SHA mismatch')
        sys.path.insert(0, str(helper.parent))
        from transmission_geometry import transmission_transforms
        context['function'] = transmission_transforms
        context['bodyid'] = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, transmission['body'])
        context['qposadr'] = [int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)])
                                              for name in transmission['jaws']]
        bound = []
        for row in transmission['geoms']:
            geom = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, row['name'])
            if geom < 0:
                raise ValueError('Source visual transmission geometry is missing: ' + row['name'])
            if model.geom_contype[geom] or model.geom_conaffinity[geom]:
                raise ValueError('Physical transmission geometry must not be visually animated: ' + row['name'])
            local = np.eye(4)
            local[:3, :3] = R.from_quat(np.roll(model.geom_quat[geom], -1)).as_matrix()
            local[:3, 3] = model.geom_pos[geom]
            component = row['component']
            rest = np.array(transmission['components'][component]['source_component_in_tool'])
            bound.append((geom, component, np.linalg.inv(rest) @ local))
        context['bound'] = bound
        _TRANSMISSION_CONTEXTS[model] = context
    left, right = context['qposadr']
    gap = transmission['nominal_gap_m'] + data.qpos[right] - data.qpos[left]
    try:
        transforms = context['function'](float(gap))
    except ValueError:
        # Retain the actual measured state and disclose a mechanism-domain
        # violation; never clamp slider qpos or invent a valid closed pose.
        return dict(transmission_visual_geoms=0, actual_gap_m=float(gap),
                    transmission_animation_outside_source_domain=True)
    body = context['bodyid']
    world = np.eye(4); world[:3, :3] = data.xmat[body].reshape(3, 3); world[:3, 3] = data.xpos[body]
    parent = np.array(transmission['parent_in_tool'])
    for geom, component, rest in context['bound']:
        pose = world @ parent @ transforms[component] @ rest
        data.geom_xpos[geom] = pose[:3, 3]
        data.geom_xmat[geom] = pose[:3, :3].ravel()
    return dict(transmission_visual_geoms=len(context['bound']), actual_gap_m=float(gap),
                transmission_animation_outside_source_domain=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['extract'])
    parser.add_argument('model', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = extract_render_assets(args.model, args.output)
    print(json.dumps(dict(output=str(args.output), geometry_materials=len(result['geometry_materials']),
                          skins=len(result['skins']), skipped=len(result['skipped'])), indent=2))
