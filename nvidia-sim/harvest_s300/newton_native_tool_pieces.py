"""Replace one named tool hull with hash-bound native PhysX convex pieces.

This is an explicit collision-model correction, not an exact trajectory or
PhysX/MuJoCo force-equivalence claim. Existing initial state, commands, bodies,
joints, attachments and contact material parameters remain unchanged. No
physics step, GPU execution or source-asset mutation occurs during preparation.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree

from newton_fruit_proxy import _write_obj, geom_body_vertices, mesh_vertices
from newton_gripper_variant import CONTACT_FIELDS
from newton_inputs import sha256

SCHEMA = 's300-native-convex-pieces-v1'
TARGET_GEOM = 's300_g0620'
TARGET_BODY = 'Tool_L'


def _json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def _same(label, left, right, checks):
    a, b = np.asarray(left), np.asarray(right)
    checks[label] = bool(a.shape == b.shape and np.array_equal(a, b))


def _mesh_graph(model, mesh):
    adr = int(model.mesh_graphadr[mesh])
    size = 2 + 3 * int(model.mesh_graph[adr]) + 6 * int(model.mesh_graph[adr + 1]) if adr >= 0 else 0
    return model.mesh_graph[adr:adr + size] if adr >= 0 else np.array([], dtype=int)


def verify_invariants(old, new, piece_names, *, target_geom=TARGET_GEOM):
    """Compare every protected physical field and unselected mesh by identity."""
    checks = {}
    for name in ('nbody', 'njnt', 'nq', 'nv', 'nu', 'neq', 'nexclude', 'npair', 'nsite', 'ntendon', 'nkey'):
        _same(name, getattr(old, name), getattr(new, name), checks)
    layout = {'body_geomadr', 'body_geomnum', 'body_bvhadr', 'body_bvhnum'}
    for name in dir(old):
        if ((name.startswith(('body_', 'jnt_', 'dof_', 'actuator_', 'eq_', 'site_', 'tendon_'))
             and name not in layout) or name in ('qpos0', 'qpos_spring', 'exclude_signature')):
            value = getattr(old, name)
            if isinstance(value, np.ndarray):
                _same(name, value, getattr(new, name), checks)
    old_geoms = {old.geom(g).name: g for g in range(old.ngeom)}
    new_geoms = {new.geom(g).name: g for g in range(new.ngeom)}
    expected = (set(old_geoms) - {target_geom}) | set(piece_names)
    _same('exact_geom_identity_set', sorted(expected), sorted(new_geoms), checks)
    protected_meshes = set()
    for name in sorted(set(old_geoms) - {target_geom}):
        a, b = old_geoms[name], new_geoms[name]
        for field in dir(old):
            if field.startswith('geom_') and field != 'geom_dataid':
                value = getattr(old, field)
                if isinstance(value, np.ndarray):
                    _same('geom:' + name + '.' + field, value[a], getattr(new, field)[b], checks)
        if old.geom_type[a] == mj.mjtGeom.mjGEOM_MESH:
            ma, mb = int(old.geom_dataid[a]), int(new.geom_dataid[b])
            _same('geom:' + name + '.mesh_name', old.mesh(ma).name, new.mesh(mb).name, checks)
            protected_meshes.add(old.mesh(ma).name)
        else:
            _same('geom:' + name + '.dataid', old.geom_dataid[a], new.geom_dataid[b], checks)
    for name in sorted(protected_meshes):
        a, b = old.mesh(name).id, new.mesh(name).id
        _same('mesh:' + name + '.vertices', mesh_vertices(old, a), mesh_vertices(new, b), checks)
        for field in ('mesh_pos', 'mesh_quat', 'mesh_scale', 'mesh_extrema'):
            _same('mesh:' + name + '.' + field, getattr(old, field)[a], getattr(new, field)[b], checks)
        for field, adr, count in (('mesh_face', 'mesh_faceadr', 'mesh_facenum'),
                                  ('mesh_normal', 'mesh_normaladr', 'mesh_normalnum'),
                                  ('mesh_facenormal', 'mesh_faceadr', 'mesh_facenum')):
            oa, on = int(getattr(old, adr)[a]), int(getattr(old, count)[a])
            na, nn = int(getattr(new, adr)[b]), int(getattr(new, count)[b])
            _same('mesh:' + name + '.' + field, getattr(old, field)[oa:oa + on],
                  getattr(new, field)[na:na + nn], checks)
        _same('mesh:' + name + '.compiled_collision_graph', _mesh_graph(old, a), _mesh_graph(new, b), checks)
    target = old.geom(target_geom).id
    for name in piece_names:
        geom = new.geom(name).id
        _same('piece:' + name + '.body', old.geom_bodyid[target], new.geom_bodyid[geom], checks)
        for field in CONTACT_FIELDS:
            _same('piece:' + name + '.' + field, getattr(old, field)[target], getattr(new, field)[geom], checks)
    for field in dir(old.opt):
        if field.startswith('_'):
            continue
        value = getattr(old.opt, field)
        if isinstance(value, (np.ndarray, int, float, bool, np.integer, np.floating)):
            _same('option.' + field, value, getattr(new.opt, field), checks)
    for field in ('meanmass', 'meaninertia'):
        _same('stat.' + field, getattr(old.stat, field), getattr(new.stat, field), checks)
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise ValueError('Protected physical fields changed: ' + repr(failed[:30]))
    return dict(passed=True, exact_numeric_comparison=True, checks=checks,
        protected_meshes_compared=sorted(protected_meshes),
        intentional_changes='Only selected tool collider decomposition and packed geometry/mesh/BVH addresses change. An unused selected old mesh may be discarded under the unchanged compiler policy; its original asset bytes remain preserved.')


def _checked_piece_source(path, source_sha, metadata, old, *, target_geom=TARGET_GEOM,
                          target_body=TARGET_BODY):
    path = Path(path).resolve(strict=True)
    value = json.loads(path.read_text())
    if (value.get('schema') != SCHEMA or value.get('coordinate_frame') != 'body_local'
            or value.get('target_geom') != target_geom or value.get('target_body') != target_body):
        raise ValueError('Exact native tool-piece schema, body_local frame and protected target identity are required')
    if value.get('source_input_model_sha256') != source_sha:
        raise ValueError('Native-piece source physical MJB SHA mismatch')
    geom = old.geom(target_geom).id
    if (old.body(int(old.geom_bodyid[geom])).name != target_body or
            old.geom_type[geom] != mj.mjtGeom.mjGEOM_MESH or
            not (old.geom_contype[geom] or old.geom_conaffinity[geom])):
        raise ValueError('Selected original collider must be the active explicitly named tool-body mesh')
    source = value.get('source', {})
    for key in ('usd_path', 'source_scene_sha256', 'source_collider_path', 'owner_path'):
        if not isinstance(source.get(key), str) or not source[key]:
            raise ValueError('Native-piece original source provenance is required: ' + key)
    usd = Path(source['usd_path']).resolve(strict=True)
    if sha256(usd) != source['source_scene_sha256']:
        raise ValueError('Native-piece source USD SHA mismatch')
    rows = [row for row in metadata.get('robot', {}).get('geometry', []) if row['name'] == target_geom]
    if len(rows) != 1 or rows[0].get('source') != source['source_collider_path'] or rows[0].get('owner') != target_body:
        raise ValueError('Native-piece collider source differs from exact robot geometry ownership')
    pieces = value.get('pieces')
    if not isinstance(pieces, list) or not pieces:
        raise ValueError('Native convex cooking returned no pieces')
    vertices = []
    for index, piece in enumerate(pieces):
        points = np.asarray(piece.get('vertices_body_local_m'), dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 4 or not np.isfinite(points).all():
            raise ValueError('Finite nondegenerate body-local vertices required for piece ' + str(index))
        try:
            hull = ConvexHull(points)
        except QhullError as error:
            raise ValueError('Degenerate native convex piece: ' + str(index)) from error
        if hull.volume <= 0:
            raise ValueError('Native convex piece must have positive 3D volume')
        vertices.append(points)
    return value, vertices


def _faces(points):
    hull = ConvexHull(points)
    faces = hull.simplices.copy()
    center = points[hull.vertices].mean(axis=0)
    for face in faces:
        v = points[face]
        if np.dot(np.cross(v[1] - v[0], v[2] - v[0]), v.mean(axis=0) - center) < 0:
            face[1], face[2] = face[2], face[1]
    return faces


def _source_asset(root, mesh):
    compiler = root.find('compiler')
    filename = Path(mesh.get('file', ''))
    if not mesh.get('file'):
        return None
    if not filename.is_absolute():
        filename = Path(compiler.get('meshdir', '.')) / filename
    return filename.resolve(strict=True)


def prepare(source, output, pieces_json, *, coordinate_tolerance_m=1e-7,
            target_geom=TARGET_GEOM, target_body=TARGET_BODY):
    source, output = Path(source).resolve(strict=True), Path(output).resolve()
    if source == output or output.exists():
        raise FileExistsError('Use a new output folder: ' + str(output))
    if not np.isfinite(coordinate_tolerance_m) or coordinate_tolerance_m <= 0:
        raise ValueError('Positive finite coordinate roundtrip tolerance required')
    if any(not isinstance(name, str) or not name for name in (target_geom, target_body)):
        raise ValueError('Exact nonempty target geometry and body names required')
    started = time.perf_counter()
    provenance = json.loads((source / 'inputs.json').read_text())
    protected_files = {}
    for name, evidence in provenance['input_files'].items():
        relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Prepared input path escapes source: ' + name)
        actual = sha256(source / name)
        if not evidence.get('sha256') or actual != evidence['sha256']:
            raise ValueError('Prepared source SHA mismatch: ' + name)
        protected_files[name] = actual
    for name in ('input_scene.xml', 'input_model.mjb', 'initial_state.npz', 'commands.npz', 'source_export.json'):
        if name not in protected_files:
            raise ValueError('Missing authoritative prepared input hash: ' + name)
    old = mj.MjModel.from_binary_path(str(source / 'input_model.mjb'))
    metadata = json.loads((source / 'source_export.json').read_text())
    piece_source, ideals = _checked_piece_source(pieces_json, protected_files['input_model.mjb'], metadata, old,
        target_geom=target_geom, target_body=target_body)
    if old.npair:
        raise ValueError('Explicit geometry-pair remapping is not supported by this opt-in correction')
    tree = ET.parse(source / 'input_scene.xml');root = tree.getroot()
    compiler, assets = root.find('compiler'), root.find('asset')
    if compiler is None or compiler.get('inertiafromgeom') != 'false' or assets is None:
        raise ValueError('Explicit inertials and inertiafromgeom=false are required')
    old_geom = old.geom(target_geom).id
    old_mesh = old.mesh(int(old.geom_dataid[old_geom])).name
    original_assets = [element for element in assets.findall('mesh') if element.get('name') == old_mesh]
    selected = [(parent, child) for parent in root.iter() for child in parent
                if child.tag == 'geom' and child.get('name') == target_geom]
    if len(selected) != 1 or len(original_assets) != 1:
        raise ValueError('Expected one exact selected XML geometry and original mesh asset')
    original_asset = _source_asset(root, original_assets[0])
    original_asset_sha = sha256(original_asset) if original_asset is not None else None
    expected_obj_sha = piece_source['source'].get('current_mujoco_obj_sha256')
    if expected_obj_sha is not None and expected_obj_sha != original_asset_sha:
        raise ValueError('Native-piece reference OBJ SHA differs from the selected original mesh asset')
    parent, original = selected[0]
    if parent.tag != 'body' or parent.get('name') != target_body or parent.find('inertial') is None:
        raise ValueError('Selected source body must retain an explicit inertial')
    output.mkdir(parents=True, exist_ok=False)
    mesh_dir = output / 'native_tool_meshes';mesh_dir.mkdir()
    original_snapshot = None
    if original_asset is not None:
        original_snapshot = output / ('original_selected_tool_mesh' + original_asset.suffix)
        shutil.copy2(original_asset, original_snapshot)
        if sha256(original_snapshot) != original_asset_sha:
            raise ValueError('Original selected mesh snapshot bytes differ')
    piece_names = [target_geom] + [target_geom + '__native' + f'{i:03d}' for i in range(1, len(ideals))]
    existing_geoms = {g.get('name') for g in root.iter('geom')}
    if set(piece_names[1:]) & existing_geoms:
        raise ValueError('New native-piece geometry identity collides with an existing collider')
    existing_meshes = {a.get('name') for a in assets.findall('mesh')}
    insertion = list(parent).index(original)
    parent.remove(original)
    piece_rows = []
    for index, (name, points) in enumerate(zip(piece_names, ideals)):
        mesh_name = old_mesh + '__native_piece' + f'{index:03d}'
        if mesh_name in existing_meshes:
            raise ValueError('New native-piece mesh identity collides with an existing asset')
        obj = mesh_dir / (mesh_name + '.obj')
        _write_obj(obj, points, _faces(points))
        assets.append(ET.Element('mesh', name=mesh_name, file=str(obj), scale='1 1 1', maxhullvert='-1'))
        geom = copy.deepcopy(original)
        for field in ('euler', 'axisangle', 'xyaxes', 'zaxis', 'fromto', 'size'):
            geom.attrib.pop(field, None)
        geom.set('name', name);geom.set('type', 'mesh');geom.set('mesh', mesh_name)
        # Native cooked vertices already include collider-to-owner placement.
        # Applying the previous mesh frame again would move/rotate them twice.
        geom.set('pos', '0 0 0');geom.set('quat', '1 0 0 0')
        parent.insert(insertion + index, geom)
        piece_rows.append(dict(index=index, geom=name, mesh=mesh_name,
            vertices_body_local_count=len(points), obj=str(obj), obj_sha256=sha256(obj)))
    # Keep every existing dependency needed by the signed prepared format.
    modified = {'input_scene.xml', 'input_model.mjb', 'source_export.json'}
    for name in protected_files:
        if name in modified:
            continue
        destination = output / name;destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / name, destination)
    if (source / 'sampled_plan.json').is_file() and not (output / 'sampled_plan.json').exists():
        shutil.copy2(source / 'sampled_plan.json', output / 'sampled_plan.json')
    shutil.copy2(pieces_json, output / 'native_tool_pieces_source.json')
    tree.write(output / 'input_scene.xml', encoding='unicode')
    new = mj.MjModel.from_xml_path(str(output / 'input_scene.xml'))
    new.dof_damping[:] = old.dof_damping
    new.dof_length[:] = old.dof_length
    for geom in range(new.ngeom):
        name = new.geom(geom).name
        reference = old_geom if name in piece_names else old.geom(name).id
        new.geom_surfacevel[geom] = old.geom_surfacevel[reference]
    new.flg_surfacevel = old.flg_surfacevel
    for field in dir(old.opt):
        if field.startswith('_'):
            continue
        value = getattr(old.opt, field)
        if isinstance(value, np.ndarray):
            getattr(new.opt, field)[:] = value
        elif isinstance(value, (int, float, bool, np.integer, np.floating)):
            setattr(new.opt, field, value)
    invariants = verify_invariants(old, new, piece_names, target_geom=target_geom)
    for row, ideal in zip(piece_rows, ideals):
        actual = geom_body_vertices(new, new.geom(row['geom']).id)
        forward_error = float(cKDTree(ideal).query(actual)[0].max())
        reverse_error = float(cKDTree(actual).query(ideal)[0].max())
        if max(forward_error, reverse_error) > coordinate_tolerance_m:
            raise ValueError('Body-local native-piece compilation moved vertices: ' + row['geom'])
        row.update(compiled_vertex_to_source_max_distance_m=forward_error,
            source_vertex_to_compiled_max_distance_m=reverse_error,
            coordinate_roundtrip_tolerance_m=coordinate_tolerance_m)
    initial = dict(np.load(source / 'initial_state.npz', allow_pickle=False))
    old_data, new_data = mj.MjData(old), mj.MjData(new)
    for data in (old_data, new_data):
        for field in ('qpos', 'qvel', 'ctrl', 'eq_active'):
            getattr(data, field)[:] = initial[field]
    # Kinematics only: no contact solve, integration or simulation step.
    mj.mj_kinematics(old, old_data);mj.mj_kinematics(new, new_data)
    if not np.array_equal(old_data.xpos, new_data.xpos) or not np.array_equal(old_data.xquat, new_data.xquat):
        raise ValueError('Initial body transforms changed')
    variant = dict(mode='native_physx_tool_convex_pieces', selected_geom=target_geom,
        selected_body=target_body, source_inputs=str(source),
        source_mjb_sha256=protected_files['input_model.mjb'],
        native_piece_json_sha256=sha256(pieces_json), native_piece_source=piece_source['source'],
        upstream_physics_model_variant=metadata.get('physics_model_variant'),
        physical_model_changed=True, collision_decomposition_changed=True,
        exact_trajectory_equivalence_claimed=False, physx_contact_force_equivalence_claimed=False,
        source_ngeom=old.ngeom, variant_ngeom=new.ngeom, source_nmesh=old.nmesh, variant_nmesh=new.nmesh,
        original_nv=old.nv, variant_nv=new.nv, retained_body_mass_inertia_joints=True,
        initial_body_transforms_bit_exact=True, initial_state_and_commands_byte_preserved=True,
        compiler_policy_unchanged=True, invariant_proof=invariants, pieces=piece_rows,
        original_selected_mesh_asset=dict(xml=original_assets[0].attrib,
            path=str(original_asset) if original_asset is not None else None,
            sha256=original_asset_sha, preserved_in_source=True,
            snapshot=str(original_snapshot) if original_snapshot is not None else None,
            snapshot_byte_exact=original_snapshot is not None,
            compiled_unused_asset_may_be_discarded=compiler.get('discardvisual') == 'true'),
        no_parent_resume_sha_bypass=True, training_eligible=False, harvest_success=None,
        limitations=['This changed collider model requires fresh full-path physics validation.',
            'Original native PhysX cooked hull pieces are reused as geometry; engine contact manifolds/forces need not match PhysX.',
            'An old-parent measured state is incompatible with the new physical MJB SHA; no continuation bypass is added.'])
    metadata['physics_model_variant'] = variant
    def replace_rows(rows):
        result = []
        for item in rows:
            if item.get('name') != target_geom:
                result.append(item)
            else:
                for index, name in enumerate(piece_names):
                    result.append(dict(item, name=name, native_piece_index=index,
                        native_convex_piece_source_json_sha256=variant['native_piece_json_sha256'],
                        source_approximation='native_physx_cooked_convex_piece',
                        mujoco_collision='one_convex_hull_per_original_native_physx_piece'))
        return result
    metadata['geometry'] = replace_rows(metadata.get('geometry', []))
    if 'robot' in metadata:
        metadata['robot']['geometry'] = replace_rows(metadata['robot'].get('geometry', []))
    mj.mj_saveModel(new, str(output / 'input_model.mjb'))
    _json(output / 'source_export.json', metadata)
    _json(output / 'native_tool_pieces_proof.json', variant)
    if sha256(source / 'initial_state.npz') != sha256(output / 'initial_state.npz') or sha256(source / 'commands.npz') != sha256(output / 'commands.npz'):
        raise ValueError('Initial state or command bytes changed')
    if original_asset is not None and sha256(original_asset) != original_asset_sha:
        raise ValueError('Original selected mesh asset changed')
    unchanged = all(sha256(source / name) == digest for name, digest in protected_files.items())
    if not unchanged:
        raise ValueError('Prepared source files changed')
    report = copy.deepcopy(provenance)
    filenames = set(protected_files) | {'native_tool_pieces_source.json', 'native_tool_pieces_proof.json'}
    if (output / 'sampled_plan.json').exists():
        filenames.add('sampled_plan.json')
    filenames.update(str(path.relative_to(output)) for path in mesh_dir.glob('*.obj'))
    if original_snapshot is not None:
        filenames.add(original_snapshot.name)
    report['input_files'] = {name: dict(path=str(output / name), sha256=sha256(output / name)) for name in sorted(filenames)}
    report['physics_model_variant'] = variant
    report['model_counts'] = dict(nbody=new.nbody, nq=new.nq, nv=new.nv, ngeom=new.ngeom, nu=new.nu, neq=new.neq)
    report['source_prepared_files_sha256'] = protected_files
    report['source_prepared_files_unchanged'] = unchanged
    report['physical_steps'] = 0
    report['variant_prepare_seconds'] = time.perf_counter() - started
    report.setdefault('code_sha256', {})[str(Path(__file__).resolve())] = sha256(__file__)
    report['source_asset_dependencies'] = [*report.get('source_asset_dependencies', []),
        *[dict(path=str(output / name), sha256=sha256(output / name)) for name in sorted(filenames)
          if name.startswith('native_tool_meshes/')]]
    _json(output / 'inputs.json', report)
    return variant


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', type=Path)
    parser.add_argument('native_pieces_json', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--target-geom', default=TARGET_GEOM,
        help='Exact opt-in collider name; must match the native source JSON (default preserves part161 workflow)')
    parser.add_argument('--target-body', default=TARGET_BODY,
        help='Exact owning tool body; must match the source JSON and original model')
    args = parser.parse_args()
    value = prepare(args.inputs, args.output, args.native_pieces_json,
        target_geom=args.target_geom, target_body=args.target_body)
    print(json.dumps({key: value[key] for key in ('selected_geom', 'source_ngeom', 'variant_ngeom',
        'source_nmesh', 'variant_nmesh', 'original_nv', 'variant_nv')}, indent=2))


if __name__ == '__main__':
    main()
