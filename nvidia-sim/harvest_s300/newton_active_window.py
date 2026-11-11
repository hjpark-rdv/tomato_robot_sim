"""Opt-in measured-state active-plant reduction for the existing Newton runner.

Compile exactly one selected plant's articulated joints and every detached
fruit. Other plants remain named static collision bodies at their measured
poses. Their still-attached CONNECT equalities remain active and present;
static/static equalities contribute no solver rows and are not release events.
The canonical full state bank is never replaced with a smaller address layout.
No GPU execution or physics integration occurs in this module.
"""
from __future__ import annotations

import argparse
import copy
from collections.abc import Mapping
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import shutil
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from newton_inputs import sha256
from newton_continuation import VerificationContext
from newton_prepared_assets import _relative_name, link_manifest_assets

SCHEMA = 's300-newton-active-window-v1'
QWIDTH = {int(mj.mjtJoint.mjJNT_FREE): 7, int(mj.mjtJoint.mjJNT_BALL): 4,
          int(mj.mjtJoint.mjJNT_SLIDE): 1, int(mj.mjtJoint.mjJNT_HINGE): 1}
VWIDTH = {int(mj.mjtJoint.mjJNT_FREE): 6, int(mj.mjtJoint.mjJNT_BALL): 3,
          int(mj.mjtJoint.mjJNT_SLIDE): 1, int(mj.mjtJoint.mjJNT_HINGE): 1}


def _json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def _load_npz(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key].copy() for key in archive.files}


def _verify_scoped_manifest(root, input_files, context):
    """Verify every original once and reject changes for this prepare call.

    The final source and output-link passes still validate every canonical
    manifest name and expected SHA. Reuse is restricted to the same resolved
    regular file with unchanged device/inode/size/mtime/ctime, including the
    alias target check. No digest or model survives this prepare invocation.
    """
    if not isinstance(input_files, Mapping):
        raise TypeError('input_files must map relative names to SHA256 evidence')
    root = Path(root).resolve(strict=True)
    if not root.is_dir():
        raise NotADirectoryError(root)

    def verify(item):
        name, evidence = item
        relative = _relative_name(name)
        expected = evidence.get('sha256') if isinstance(evidence, Mapping) else None
        if not isinstance(expected, str) or not re.fullmatch(r'[0-9a-fA-F]{64}', expected):
            raise ValueError('Manifest file has no valid SHA256: ' + name)
        actual = context.file_sha256(root / relative)
        if actual != expected.lower():
            raise ValueError('Input SHA256 mismatch: ' + name)
        return name, actual

    with ThreadPoolExecutor(max_workers=8, thread_name_prefix='newton-prepare-hash') as pool:
        return dict(pool.map(verify, input_files.items()))


def _names(model, kind, count):
    return [mj.mj_id2name(model, kind, i) for i in range(count)]


def _named_map(model, kind, count):
    result = _names(model, kind, count)
    if any(name is None for name in result) or len(set(result)) != len(result):
        raise ValueError('Active-window mapping requires unique nonempty names: ' + str(kind))
    return {name: i for i, name in enumerate(result)}


def joint_maps(full, reduced):
    """Return exact retained joint identities and old/new q/v coordinate maps."""
    full_names = _named_map(full, mj.mjtObj.mjOBJ_JOINT, full.njnt)
    reduced_names = _named_map(reduced, mj.mjtObj.mjOBJ_JOINT, reduced.njnt)
    rows, full_q, reduced_q, full_v, reduced_v = [], [], [], [], []
    for name, new in reduced_names.items():
        if name not in full_names:
            raise ValueError('Unknown reduced joint identity: ' + name)
        old = full_names[name]
        if full.jnt_type[old] != reduced.jnt_type[new]:
            raise ValueError('Reduced joint type changed: ' + name)
        kind = int(full.jnt_type[old]); qw, vw = QWIDTH[kind], VWIDTH[kind]
        oq, nq = int(full.jnt_qposadr[old]), int(reduced.jnt_qposadr[new])
        ov, nv = int(full.jnt_dofadr[old]), int(reduced.jnt_dofadr[new])
        full_q.extend(range(oq, oq + qw)); reduced_q.extend(range(nq, nq + qw))
        full_v.extend(range(ov, ov + vw)); reduced_v.extend(range(nv, nv + vw))
        rows.append(dict(name=name, joint_type=kind, full_joint=old, reduced_joint=new,
                         full_qpos=oq, reduced_qpos=nq, qwidth=qw,
                         full_dof=ov, reduced_dof=nv, vwidth=vw))
    if len(full_q) != reduced.nq or len(full_v) != reduced.nv:
        raise ValueError('Incomplete named joint coordinate coverage')
    return dict(joints=rows, full_qpos=full_q, reduced_qpos=reduced_q,
                full_dofs=full_v, reduced_dofs=reduced_v)


def _single(value, shape, name):
    result = np.asarray(value)
    if result.shape == (1, *shape):
        result = result[0]
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError('Canonical state shape/nonfinite mismatch: ' + name)
    return result.copy()


def canonical_state(model, initial, supplied=None):
    """Normalize measured arrays without silently truncating a world dimension."""
    result = {key: np.asarray(value).copy() for key, value in initial.items()}
    supplied = {} if supplied is None else dict(supplied)
    shapes = dict(qpos=(model.nq,), qvel=(model.nv,), eq_active=(model.neq,),
                  ctrl=(model.nu,), runtime_dof_damping=(model.nv,),
                  plant_preload=(model.nv,), runtime_dof_length=(model.nv,),
                  qacc_warmstart=(model.nv,))
    for name, shape in shapes.items():
        if name in supplied:
            result[name] = _single(supplied[name], shape, name)
        elif name in result:
            result[name] = _single(result[name], shape, name)
    for name in ('qpos', 'qvel', 'eq_active', 'ctrl', 'runtime_dof_damping', 'plant_preload'):
        if name not in result:
            raise ValueError('Missing canonical full state: ' + name)
    if result['eq_active'].dtype.kind != 'b':
        raise ValueError('Canonical attachment activity must remain boolean')
    time_value = np.asarray(supplied.get('time', initial.get('time', [0.0])), dtype=float).reshape(-1)
    if time_value.shape != (1,) or not np.isfinite(time_value).all() or time_value[0] < 0:
        raise ValueError('Finite nonnegative single-world physics time required')
    result['time'] = time_value
    if 'body_names' in supplied and not np.array_equal(supplied['body_names'], initial['body_names']):
        raise ValueError('Canonical body-name order mismatch')
    return result


def _data(model, state):
    data = mj.MjData(model)
    for name in ('qpos', 'qvel', 'eq_active', 'ctrl'):
        getattr(data, name)[:] = state[name]
    data.time = float(state['time'][0])
    if 'qacc_warmstart' in state:
        data.qacc_warmstart[:] = state['qacc_warmstart']
    mj.mj_kinematics(model, data)
    return data


def _window_selection(model, metadata, selected):
    selected = str(selected)
    windows = metadata.get('windows')
    if not isinstance(windows, list) or not windows:
        raise ValueError('Explicit nonempty window source metadata is required')
    ids = [str(window['id']) for window in windows]
    if len(set(ids)) != len(ids) or selected not in ids:
        raise ValueError('Unknown or ambiguous active window: ' + selected)
    names = set(_names(model, mj.mjtObj.mjOBJ_BODY, model.nbody))
    owned, seen = {}, set()
    for window in windows:
        declared = list(window['bodies'])
        current = set(declared) & names
        if len(set(declared)) != len(declared) or not current or seen & current:
            raise ValueError('Ambiguous/missing window body ownership: ' + str(window['id']))
        owned[str(window['id'])] = current; seen |= current
    return selected, owned


def _attachment_state(model, metadata, state, owned, selected):
    specs = metadata['attachments']; frozen, detached, active = [], [], []
    for spec in specs:
        name, anchor = spec['name'], spec['anchor']
        owners = [key for key, values in owned.items() if name in values]
        if len(owners) != 1 or anchor not in owned[owners[0]]:
            raise ValueError('Fruit/anchor exact window ownership mismatch: ' + name)
        eq = model.equality(name + '_attachment').id
        objects = [int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq])]
        if model.eq_objtype[eq] == mj.mjtObj.mjOBJ_SITE:
            objects = [int(model.site_bodyid[obj]) for obj in objects]
        elif model.eq_objtype[eq] != mj.mjtObj.mjOBJ_BODY:
            raise ValueError('Unsupported attachment equality object type: ' + name)
        if (model.eq_type[eq] != mj.mjtEq.mjEQ_CONNECT or
                set(objects) != {model.body(name).id, model.body(anchor).id}):
            raise ValueError('Fruit CONNECT body mapping mismatch: ' + name)
        if not state['eq_active'][eq]:
            body = model.body(name).id
            adr = int(model.body_jntadr[body])
            if model.body_jntnum[body] != 1 or model.jnt_type[adr] != mj.mjtJoint.mjJNT_FREE:
                raise ValueError('Detached fruit must retain one native free joint: ' + name)
            detached.append(name)
        elif owners[0] == selected:
            active.append(name)
        else:
            frozen.append(name)
    if len({spec['name'] for spec in specs}) != len(specs):
        raise ValueError('Duplicate canonical attachment identity')
    return frozen, detached, active


def _freeze_xml(tree, full, full_data, frozen_bodies):
    elements = {element.get('name'): element for element in tree.iter('body')}
    for name in sorted(frozen_bodies):
        body = full.body(name).id
        parent = int(full.body_parentid[body])
        element = elements.get(name)
        if element is None:
            raise ValueError('Missing canonical XML body: ' + name)
        # Keep topology, local collider/inertial frames and all visual flags.
        # Remove only this inactive body's actual dynamic coordinates.
        for child in list(element):
            if child.tag in ('joint', 'freejoint'):
                element.remove(child)
        parent_rotation = Rotation.from_quat(full_data.xquat[parent, [1, 2, 3, 0]])
        rotation = Rotation.from_quat(full_data.xquat[body, [1, 2, 3, 0]])
        position = parent_rotation.inv().apply(full_data.xpos[body] - full_data.xpos[parent])
        quaternion = (parent_rotation.inv() * rotation).as_quat()[[3, 0, 1, 2]]
        for field in ('euler', 'axisangle', 'xyaxes', 'zaxis'):
            element.attrib.pop(field, None)
        element.set('pos', ' '.join(format(float(x), '.17g') for x in position))
        element.set('quat', ' '.join(format(float(x), '.17g') for x in quaternion))


def _same(name, a, b, checks):
    a, b = np.asarray(a), np.asarray(b)
    checks[name] = bool(a.shape == b.shape and np.array_equal(a, b))


def _preserve_and_verify(full, reduced, mapping):
    """Restore only compiled numeric constants; check all retained physics."""
    fq, rq = mapping['full_qpos'], mapping['reduced_qpos']
    fv, rv = mapping['full_dofs'], mapping['reduced_dofs']
    for name in ('qpos0', 'qpos_spring'):
        getattr(reduced, name)[rq] = getattr(full, name)[fq]
    for name in ('dof_damping', 'dof_length', 'dof_armature', 'dof_frictionloss'):
        getattr(reduced, name)[rv] = getattr(full, name)[fv]
    # Body-based CONNECT anchors may be re-derived against the measured frozen
    # XML frames; retain their authoritative local anchors from the signed MJB.
    reduced.eq_data[:] = full.eq_data
    # Compiler derives actuator normalization at the reference pose; freezing
    # inactive trees must not silently change the existing robot drive scale.
    reduced.actuator_acc0[:] = full.actuator_acc0
    reduced.geom_surfacevel[:] = full.geom_surfacevel
    # discardvisual may leave a conservative source body-margin cache even
    # when its original visual geoms are absent. Preserve that signed cache
    # through compact-asset XML compilation without changing any geom margin.
    for body in np.flatnonzero(full.body_margin != reduced.body_margin):
        if (full.body(body).name != reduced.body(body).name or
                full.body_geomnum[body] != 0 or reduced.body_geomnum[body] != 0):
            raise ValueError('Body margin changed on nonempty or mismapped body: ' + full.body(body).name)
        reduced.body_margin[body] = full.body_margin[body]
    reduced.flg_surfacevel = full.flg_surfacevel
    for name in dir(full.opt):
        if name.startswith('_'):
            continue
        value = getattr(full.opt, name)
        if isinstance(value, np.ndarray):
            getattr(reduced.opt, name)[:] = value
        elif isinstance(value, (int, float, bool, np.integer, np.floating)):
            setattr(reduced.opt, name, value)
    # Solver normalization must not change merely because inert joints froze.
    for name in ('meanmass', 'meaninertia', 'meansize', 'extent'):
        setattr(reduced.stat, name, getattr(full.stat, name))
    reduced.stat.center[:] = full.stat.center
    checks = {}
    for kind, count, label in ((mj.mjtObj.mjOBJ_BODY, full.nbody, 'body'),
                                (mj.mjtObj.mjOBJ_GEOM, full.ngeom, 'geom'),
                                (mj.mjtObj.mjOBJ_EQUALITY, full.neq, 'equality'),
                                (mj.mjtObj.mjOBJ_ACTUATOR, full.nu, 'actuator')):
        _same(label + '_names_and_order', _names(full, kind, count),
              _names(reduced, kind, getattr(reduced, {'body': 'nbody', 'geom': 'ngeom',
                   'equality': 'neq', 'actuator': 'nu'}[label])), checks)
    for name in ('body_parentid', 'body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_margin',
                 'eq_type', 'eq_obj1id', 'eq_obj2id', 'eq_data', 'eq_solref', 'eq_solimp',
                 'eq_active0', 'exclude_signature'):
        _same(name, getattr(full, name), getattr(reduced, name), checks)
    for name in dir(full):
        if name.startswith(('geom_', 'actuator_', 'site_')) and name != 'actuator_trnid':
            value = getattr(full, name)
            if isinstance(value, np.ndarray):
                _same(name, value, getattr(reduced, name), checks)
    # Removing inactive plant joints shifts packed joint IDs when the robot
    # appears after a plant in the XML. The compiled transmission must still
    # target the exact same joint name and retain every transmission parameter.
    # Do not copy the old packed ID: it could command a different new joint.
    for actuator in range(full.nu):
        label = 'actuator_transmission:' + full.actuator(actuator).name
        kind = int(full.actuator_trntype[actuator])
        if kind in (mj.mjtTrn.mjTRN_JOINT, mj.mjtTrn.mjTRN_JOINTINPARENT):
            old, new = int(full.actuator_trnid[actuator, 0]), int(reduced.actuator_trnid[actuator, 0])
            _same(label + ':joint_name', full.joint(old).name, reduced.joint(new).name, checks)
            _same(label + ':second_reference', full.actuator_trnid[actuator, 1],
                  reduced.actuator_trnid[actuator, 1], checks)
        else:
            # All body/site identities remain in the same order. Unsupported
            # transmission layouts keep the previous strict numeric check.
            _same(label + ':exact_references', full.actuator_trnid[actuator],
                  reduced.actuator_trnid[actuator], checks)
    for row in mapping['joints']:
        old, new = row['full_joint'], row['reduced_joint']
        for name in ('jnt_type', 'jnt_pos', 'jnt_axis', 'jnt_stiffness', 'jnt_range',
                     'jnt_limited', 'jnt_solref', 'jnt_solimp', 'jnt_margin'):
            _same('joint:' + row['name'] + ':' + name, getattr(full, name)[old],
                  getattr(reduced, name)[new], checks)
    for name in ('dof_damping', 'dof_length', 'dof_armature', 'dof_frictionloss'):
        _same(name, getattr(full, name)[fv], getattr(reduced, name)[rv], checks)
    # No mesh is removed; verify source vertex/face and compiled placements.
    for name in ('mesh_vert', 'mesh_face', 'mesh_pos', 'mesh_quat', 'mesh_scale',
                 'mesh_vertadr', 'mesh_vertnum', 'mesh_faceadr', 'mesh_facenum'):
        _same(name, getattr(full, name), getattr(reduced, name), checks)
    failures = [name for name, passed in checks.items() if not passed]
    if failures:
        raise ValueError('Active-window protected collider/material/robot fields changed: ' + repr(failures[:25]))
    return dict(passed=True, exact_checks=len(checks), checks=checks,
        packed_joint_transmission_references_compared_by_exact_joint_name=True)


def _reduced_initial(full, reduced, initial, state, mapping):
    result = {key: value.copy() for key, value in initial.items()}
    fq, rq, fv, rv = (np.asarray(mapping[key], dtype=int) for key in
                     ('full_qpos', 'reduced_qpos', 'full_dofs', 'reduced_dofs'))
    result['qpos'] = np.empty(reduced.nq); result['qpos'][rq] = state['qpos'][fq]
    result['qvel'] = np.empty(reduced.nv); result['qvel'][rv] = state['qvel'][fv]
    for name in ('plant_preload', 'runtime_dof_damping', 'runtime_dof_length', 'qacc_warmstart'):
        if name in state:
            result[name] = np.empty(reduced.nv); result[name][rv] = state[name][fv]
    for name in ('eq_active', 'ctrl', 'time'):
        result[name] = state[name].copy()
    inverse_q = {int(old): int(new) for old, new in zip(fq, rq)}
    inverse_v = {int(old): int(new) for old, new in zip(fv, rv)}
    result['robot_qpos_addresses'] = np.asarray([inverse_q[int(x)] for x in initial['robot_qpos_addresses']], np.int32)
    result['robot_dof_addresses'] = np.asarray([inverse_v[int(x)] for x in initial['robot_dof_addresses']], np.int32)
    result['plant_dofs'] = np.asarray([inverse_v[int(x)] for x in initial['plant_dofs'] if int(x) in inverse_v], np.int64)
    result['actuator_ids'] = initial['actuator_ids'].copy()
    return result


def prepare(source, output, active_window_id, *, bank_state=None, pose_tolerance=1e-10,
            freeze_all_plants=False):
    """Build a new prepared input; ``bank_state`` always uses canonical addresses.

    Time may be nonzero. A caller must use the dedicated active-window lineage
    and initial-clock restoration, never bypass an old MJB resume SHA check.
    Return a proof dictionary; authoritative arrays are written to the output.
    """
    source, output = Path(source).resolve(strict=True), Path(output).resolve()
    if output.exists() or output == source:
        raise FileExistsError('Use a new active-window output: ' + str(output))
    if not np.isfinite(pose_tolerance) or pose_tolerance <= 0:
        raise ValueError('Positive finite pose verification tolerance required')
    started = time.perf_counter()
    context = VerificationContext()
    context.file_sha256(source / 'inputs.json')
    provenance = json.loads((source / 'inputs.json').read_text())
    protected = _verify_scoped_manifest(source, provenance['input_files'], context)
    for name in ('input_scene.xml', 'input_model.mjb', 'initial_state.npz', 'commands.npz', 'source_export.json'):
        if name not in protected:
            raise ValueError('Missing authoritative source input hash: ' + name)
    full = context.model(source / 'input_model.mjb', expected_sha256=protected['input_model.mjb'])
    metadata = json.loads((source / 'source_export.json').read_text())
    initial = _load_npz(source / 'initial_state.npz')
    state = canonical_state(full, initial, bank_state)
    selected, owned = _window_selection(full, metadata, active_window_id)
    frozen_names, detached_names, active_names = _attachment_state(full, metadata, state, owned, selected)
    frozen_bodies = set().union(*(names for key, names in owned.items() if key != selected)) - set(detached_names)
    dynamic_bodies = owned[selected] | set(detached_names)
    if freeze_all_plants:
        # Explicit ideal harvesting has no plant-contact or detachment dynamics.
        # Keep every body/collider at its measured pose, including thick stems;
        # only the existing robot coordinates continue to integrate on the GPU.
        if metadata.get('geometric_harvest', {}).get('mode') != 'main_stem_only_geometric_harvest':
            raise ValueError('All-plant freezing requires explicit geometric harvest mode')
        if detached_names:
            raise ValueError('Cannot freeze an already physically detached fruit')
        frozen_bodies = set().union(*owned.values())
        dynamic_bodies = set()
        frozen_names = [spec['name'] for spec in metadata['attachments']]
        active_names = []
    for body in frozen_bodies:
        # A dynamic child under a frozen body is supported via its measured
        # parent frame. A frozen child under a moving body is not this model.
        parent = full.body(int(full.body_parentid[full.body(body).id])).name
        if parent in dynamic_bodies:
            raise ValueError('Cannot freeze inactive body under active dynamic plant: ' + body)
    full_data = _data(full, state)
    # A measured final-state archive need not contain body pose arrays. Keep
    # the canonical snapshot internally consistent with its actual qpos.
    state['body_positions'] = full_data.xpos.copy()
    state['body_quaternions_wxyz'] = full_data.xquat.copy()
    tree = ET.parse(source / 'input_scene.xml')
    compiler = tree.getroot().find('compiler')
    if compiler is None or compiler.get('inertiafromgeom') != 'false' or compiler.get('fusestatic') != 'false':
        raise ValueError('Explicit inertials and fusestatic=false required')
    if full.ntendon or full.nkey:
        raise ValueError('Named tendon/keyframe reduction requires explicit support')
    _freeze_xml(tree, full, full_data, frozen_bodies)
    output.mkdir(parents=True, exist_ok=False)
    generated = {'input_scene.xml', 'input_model.mjb', 'initial_state.npz', 'source_export.json',
                 'canonical_full_state.npz', 'active_window_proof.json'}
    shared = {name: provenance['input_files'][name] for name in protected if name not in generated}
    link_manifest_assets(source, output, shared)
    if (source / 'sampled_plan.json').is_file() and not (output / 'sampled_plan.json').exists():
        shutil.copy2(source / 'sampled_plan.json', output / 'sampled_plan.json')
    tree.write(output / 'input_scene.xml', encoding='unicode')
    reduced = mj.MjModel.from_xml_path(str(output / 'input_scene.xml'))
    mapping = joint_maps(full, reduced)
    invariants = _preserve_and_verify(full, reduced, mapping)
    reduced_initial = _reduced_initial(full, reduced, initial, state, mapping)
    reduced_data = _data(reduced, reduced_initial)
    position_error = float(np.max(np.abs(full_data.xpos - reduced_data.xpos)))
    quaternion_error = float(np.max(np.minimum(np.linalg.norm(full_data.xquat - reduced_data.xquat, axis=1),
                                                np.linalg.norm(full_data.xquat + reduced_data.xquat, axis=1))))
    if max(position_error, quaternion_error) > pose_tolerance:
        raise ValueError('Measured snapshot body poses changed during reduction')
    reduced_initial['body_positions'] = reduced_data.xpos.copy()
    reduced_initial['body_quaternions_wxyz'] = reduced_data.xquat.copy()
    mj.mj_forward(reduced, reduced_data)  # Kinematics/contact/constraint assembly, no integration.
    frozen_eqids = [reduced.equality(name + '_attachment').id for name in frozen_names]
    frozen_rows = int(np.count_nonzero((reduced_data.efc_type[:reduced_data.nefc] == mj.mjtConstraint.mjCNSTR_EQUALITY) &
                                      np.isin(reduced_data.efc_id[:reduced_data.nefc], frozen_eqids)))
    if frozen_rows or not np.all(reduced_initial['eq_active'][frozen_eqids]):
        raise ValueError('Frozen attached equality must remain active with zero solver rows')
    for name in detached_names:
        body = reduced.body(name).id; joint = int(reduced.body_jntadr[body])
        if reduced.body_jntnum[body] != 1 or reduced.jnt_type[joint] != mj.mjtJoint.mjJNT_FREE:
            raise ValueError('Detached fruit was accidentally frozen')
    fv, rv = mapping['full_dofs'], mapping['reduced_dofs']
    fq, rq = mapping['full_qpos'], mapping['reduced_qpos']
    identity_checks = dict(qpos=np.array_equal(state['qpos'][fq], reduced_initial['qpos'][rq]),
        qvel=np.array_equal(state['qvel'][fv], reduced_initial['qvel'][rv]),
        ctrl=np.array_equal(state['ctrl'], reduced_initial['ctrl']),
        eq_active=np.array_equal(state['eq_active'], reduced_initial['eq_active']),
        damping=np.array_equal(state['runtime_dof_damping'][fv], reduced_initial['runtime_dof_damping'][rv]),
        preload=np.array_equal(state['plant_preload'][fv], reduced_initial['plant_preload'][rv]))
    if not all(identity_checks.values()):
        raise ValueError('Measured dynamic coordinates/control/attachment state changed')
    proof = dict(schema=SCHEMA, canonical_inputs=str(source),
        canonical_mjb_sha256=protected['input_model.mjb'], active_window_id=selected,
        active_body_names=sorted(dynamic_bodies), frozen_body_names=sorted(frozen_bodies),
        all_plants_static=bool(freeze_all_plants),
        frozen_attachment_names=frozen_names, detached_dynamic_attachment_names=detached_names,
        active_attachment_names=active_names,
        full_counts=dict(nq=full.nq, nv=full.nv, nbody=full.nbody, ngeom=full.ngeom, neq=full.neq),
        reduced_counts=dict(nq=reduced.nq, nv=reduced.nv, nbody=reduced.nbody, ngeom=reduced.ngeom, neq=reduced.neq),
        joint_mapping=mapping, dynamic_state_preservation=identity_checks,
        body_position_max_abs_difference_m=position_error, body_quaternion_sign_invariant_error=quaternion_error,
        pose_verification_tolerance=pose_tolerance, physical_model_changed=True,
        inactive_plant_dynamic_response_removed=True, collider_geometry_preserved=True,
        frozen_eq_active_preserved=True, frozen_eq_solver_rows=frozen_rows,
        static_constraint_check=dict(physical_steps=0, cpu_forward_only=True, total_nefc=int(reduced_data.nefc)),
        protected_physics=invariants, initial_clock_s=float(state['time'][0]),
        solver_and_contact_cache_rebuilt=True, old_mjb_resume_sha_bypass=False,
        inactive_state_policy='Full bank stores inactive measured coordinates/velocities without integration; reduced model fixes their poses. Retained dynamic and detached fruit coordinates alone are written back.',
        exact_full_dynamics_equivalence_claimed=False, training_eligible=False, harvest_success=None)
    metadata['active_window'] = {key: value for key, value in proof.items() if key not in ('protected_physics', 'joint_mapping')}
    metadata['active_harvest_attachment_names'] = [spec['name'] for spec in metadata['attachments'] if spec['name'] in owned[selected]]
    np.savez_compressed(output / 'initial_state.npz', **reduced_initial)
    np.savez_compressed(output / 'canonical_full_state.npz', **state)
    mj.mj_saveModel(reduced, str(output / 'input_model.mjb'))
    proof['reduced_mjb_sha256'] = sha256(output / 'input_model.mjb')
    _json(output / 'source_export.json', metadata)
    _json(output / 'active_window_proof.json', proof)
    try:
        _verify_scoped_manifest(source, provenance['input_files'], context)
    except ValueError as error:
        raise ValueError('Canonical source assets were modified: ' + str(error)) from error
    report = copy.deepcopy(provenance)
    files = set(protected) | {'active_window_proof.json', 'canonical_full_state.npz'}
    if (output / 'sampled_plan.json').is_file():
        files.add('sampled_plan.json')
    shared_hashes = _verify_scoped_manifest(output, shared, context)
    report['input_files'] = {name: dict(path=str(output / name),
        sha256=shared_hashes[name] if name in shared_hashes else sha256(output / name)) for name in sorted(files)}
    report['model_counts'] = dict(nbody=reduced.nbody, nq=reduced.nq, nv=reduced.nv, ngeom=reduced.ngeom, nu=reduced.nu, neq=reduced.neq)
    report['active_window'] = {key: value for key, value in proof.items() if key not in ('protected_physics', 'joint_mapping')}
    report['canonical_prepared_files_sha256'] = protected
    report['canonical_prepared_files_unchanged'] = True
    report['physical_steps'] = 0
    report['active_window_prepare_seconds'] = time.perf_counter() - started
    report.setdefault('code_sha256', {})[str(Path(__file__).resolve())] = sha256(__file__)
    # Recheck every observed source and symlink after all compile/serialization
    # work. This is evidence of same-call reuse, never a cross-call cache.
    report['verification_scope'] = context.evidence()
    _json(output / 'inputs.json', report)
    return proof


def lift_state(source, reduced_inputs, reduced_state, bank_state=None, *, verification_context=None):
    """Merge measured reduced final arrays into a canonical full address bank.

    Removed inactive joints retain their recorded bank values. No simulated
    time evolution or reactivation of equalities is invented for them.
    """
    checked_sha = sha256 if verification_context is None else verification_context.file_sha256
    read_model = mj.MjModel.from_binary_path if verification_context is None else verification_context.model
    source, reduced_inputs = Path(source).resolve(strict=True), Path(reduced_inputs).resolve(strict=True)
    proof = json.loads((reduced_inputs / 'active_window_proof.json').read_text())
    if proof.get('schema') != SCHEMA or checked_sha(source / 'input_model.mjb') != proof['canonical_mjb_sha256']:
        raise ValueError('Canonical full model SHA mismatch while lifting reduced state')
    if checked_sha(reduced_inputs / 'input_model.mjb') != proof['reduced_mjb_sha256']:
        raise ValueError('Reduced model SHA mismatch while lifting state')
    full = read_model(str(source / 'input_model.mjb'))
    reduced = read_model(str(reduced_inputs / 'input_model.mjb'))
    initial = _load_npz(source / 'initial_state.npz')
    saved_bank = _load_npz(reduced_inputs / 'canonical_full_state.npz')
    bank = canonical_state(full, initial, saved_bank if bank_state is None else bank_state)
    start = _load_npz(reduced_inputs / 'initial_state.npz')
    state = canonical_state(reduced, start, reduced_state)
    if float(state['time'][0]) < float(saved_bank['time'][0]):
        raise ValueError('Reduced final state rolls back canonical physics time')
    mapping = joint_maps(full, reduced)
    if mapping != proof['joint_mapping']:
        raise ValueError('Reduced joint identity mapping differs from signed proof')
    fq, rq, fv, rv = (mapping[key] for key in ('full_qpos', 'reduced_qpos', 'full_dofs', 'reduced_dofs'))
    result = {key: value.copy() for key, value in bank.items()}
    result['qpos'][fq] = state['qpos'][rq]
    result['qvel'][fv] = state['qvel'][rv]
    for name in ('runtime_dof_damping', 'qacc_warmstart'):
        if name in state:
            if name not in result:
                result[name] = np.zeros(full.nv)
            result[name][fv] = state[name][rv]
    result['eq_active'] = state['eq_active'].copy()
    result['ctrl'] = state['ctrl'].copy()
    result['time'] = state['time'].copy()
    # Immutable frozen equality activity must never masquerade as a release.
    frozen_eqids = [full.equality(name + '_attachment').id for name in proof['frozen_attachment_names']]
    if not np.all(result['eq_active'][frozen_eqids]):
        raise ValueError('Frozen attachment was spuriously released by the reduced executor')
    if np.any(result['eq_active'] & ~bank['eq_active']):
        raise ValueError('Reduced state reactivated an already released attachment')
    full_data, reduced_data = _data(full, result), _data(reduced, state)
    position_error = float(np.max(np.abs(full_data.xpos - reduced_data.xpos)))
    quaternion_error = float(np.max(np.minimum(np.linalg.norm(full_data.xquat - reduced_data.xquat, axis=1),
                                                np.linalg.norm(full_data.xquat + reduced_data.xquat, axis=1))))
    if max(position_error, quaternion_error) > proof['pose_verification_tolerance']:
        raise ValueError('Lifted canonical world poses disagree with measured reduced poses')
    result['body_positions'] = full_data.xpos.copy()
    result['body_quaternions_wxyz'] = full_data.xquat.copy()
    evidence = dict(schema='s300-newton-active-window-lift-v1',
        canonical_mjb_sha256=proof['canonical_mjb_sha256'], reduced_mjb_sha256=proof['reduced_mjb_sha256'],
        active_window_id=proof['active_window_id'], mapped_joint_count=len(mapping['joints']),
        retained_qpos_exact=bool(np.array_equal(result['qpos'][fq], state['qpos'][rq])),
        retained_qvel_exact=bool(np.array_equal(result['qvel'][fv], state['qvel'][rv])),
        frozen_eq_active_preserved=True, attachment_reactivation=False,
        body_position_max_abs_difference_m=position_error, body_quaternion_sign_invariant_error=quaternion_error,
        physics_time_s=float(result['time'][0]), physical_steps=0,
        inactive_coordinates_held_without_integration=True, training_eligible=False)
    return result, evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--active-window', required=True)
    parser.add_argument('--bank-state', type=Path)
    args = parser.parse_args()
    proof = prepare(args.inputs, args.output, args.active_window,
                    bank_state=_load_npz(args.bank_state) if args.bank_state else None)
    print(json.dumps({key: proof[key] for key in ('active_window_id', 'full_counts', 'reduced_counts',
        'frozen_attachment_names', 'detached_dynamic_attachment_names', 'frozen_eq_solver_rows')}, indent=2))


if __name__ == '__main__':
    main()
