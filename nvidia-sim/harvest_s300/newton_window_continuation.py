"""Explicit verified transitions between different active-window MJB layouts.

The legacy continuation contract remains unchanged: its identical-MJB check is
first applied to the completed parent against that parent's own signed inputs.
A separate certificate then binds the canonical full bank, named reduction and
new initial state. No numerical state editing or physics execution occurs here.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import re
import json
from pathlib import Path

import mujoco as mj
import numpy as np

from newton_active_window import lift_state, joint_maps
from newton_continuation import load_resume, file_sha256, VerificationContext
from newton_prepared_assets import verify_manifest_files, _relative_name

SCHEMA = 's300-newton-active-window-transition-v1'
# CPU kinematics versus recorded float32 GPU poses, not a collision tolerance.
GPU_POSE_POSITION_TOLERANCE_M = 1e-5
GPU_POSE_QUATERNION_TOLERANCE = 1e-5


def _read(path, *, context=None):
    if context is not None:
        context.file_sha256(path)
    return json.loads(Path(path).read_text())


def _npz(path, *, context=None):
    if context is not None:
        context.file_sha256(path)
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key].copy() for key in archive.files}


def _unsigned_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _prepared(path, *, context=None):
    path = Path(path).resolve(strict=True)
    proof = _read(path / 'inputs.json', context=context)
    try:
        if context is None:
            verify_manifest_files(path, proof['input_files'])
        else:
            # Reuse the existing canonical path validator and its eight-read
            # limit. This scope verifies every unique original file once;
            # subsequent references recheck stable file identity, not bytes.
            def verify(item):
                name, evidence = item
                relative = _relative_name(name)
                expected = evidence.get('sha256') if isinstance(evidence, dict) else None
                if not isinstance(expected, str) or not re.fullmatch(r'[0-9a-fA-F]{64}', expected):
                    raise ValueError('Manifest file has no valid SHA256: ' + name)
                if context.file_sha256(path / relative) != expected.lower():
                    raise ValueError('Input SHA256 mismatch: ' + name)
            with ThreadPoolExecutor(max_workers=8, thread_name_prefix='newton-scope-hash') as pool:
                tuple(pool.map(verify, proof['input_files'].items()))
    except ValueError as error:
        raise ValueError('Transition prepared input SHA/path verification failed: ' + str(error)) from error
    return proof


def _policy(value):
    from newton_contact_policy import validate_config
    return validate_config(_read(value) if isinstance(value, (str, Path)) else value)


def _recorded_terminal(parent, summary, state, proof, full_model, lifted, *, context=None):
    if not summary.get('states_sha256') or file_sha256(parent / 'states.npz', context=context) != summary['states_sha256']:
        raise ValueError('Parent actual states SHA mismatch')
    with np.load(parent / 'states.npz', allow_pickle=False) as recorded:
        for name in ('qpos', 'qvel', 'eq_active'):
            if not np.array_equal(recorded[name][-1], state[name][0]):
                raise ValueError('Parent recorded final state differs from terminal snapshot: ' + name)
        if float(recorded['time'][-1]) != float(state['time'][0]):
            raise ValueError('Parent recorded and terminal clocks differ')
        poses = recorded['poses'][-1].copy()
        paths = recorded['body_paths'].tolist()
    mapping = proof.get('source_body_to_native_name', {})
    expected_names = [full_model.body(i).name for i in range(1, full_model.nbody)]
    if len(set(mapping.values())) != len(mapping) or set(mapping.values()) != set(expected_names):
        raise ValueError('Parent display mapping does not cover exact named bodies')
    reverse = {value: key for key, value in mapping.items()}
    expected_paths = [reverse[name] for name in expected_names]
    if paths != expected_paths or poses.shape != (full_model.nbody - 1, 7) or not np.isfinite(poses).all():
        raise ValueError('Parent actual pose path/order/nonfinite mismatch')
    data = mj.MjData(full_model);data.qpos[:] = lifted['qpos'];mj.mj_kinematics(full_model, data)
    expected_q = data.xquat[1:, [1, 2, 3, 0]]
    position_error = float(np.max(np.abs(data.xpos[1:] - poses[:, :3])))
    quaternion_error = float(np.max(np.minimum(np.linalg.norm(expected_q - poses[:, 3:], axis=1),
                                               np.linalg.norm(expected_q + poses[:, 3:], axis=1))))
    if position_error > GPU_POSE_POSITION_TOLERANCE_M or quaternion_error > GPU_POSE_QUATERNION_TOLERANCE:
        raise ValueError('Parent recorded actual poses disagree with lifted terminal joint state')
    return poses, paths, dict(position_max_abs_difference_m=position_error,
        quaternion_sign_invariant_difference=quaternion_error,
        position_tolerance_m=GPU_POSE_POSITION_TOLERANCE_M,
        quaternion_tolerance=GPU_POSE_QUATERNION_TOLERANCE,
        scope='CPU kinematic reconstruction versus stored float32 GPU body poses; no collision policy relaxation')


def checked_window_parent(parent, common, initial, meta, contact_policy, expected_index, *, verification_context=None):
    """Return full-bank equivalents of the existing checked_parent six-tuple."""
    context = verification_context if verification_context is not None else VerificationContext()
    parent, common = Path(parent).resolve(strict=True), Path(common).resolve(strict=True)
    summary = _read(parent / 'summary.json', context=context)
    parent_inputs = Path(summary['input']).resolve(strict=True)
    proof = _prepared(parent_inputs, context=context)
    _prepared(common, context=context)
    sequence = proof.get('same_bed_sequence', {})
    if sequence.get('index') != expected_index or sequence.get('source_common') != str(common):
        raise ValueError('Window parent is not the preceding selected canonical route segment')
    parent_initial = _npz(parent_inputs / 'initial_state.npz', context=context)
    parent_meta = _read(parent_inputs / 'source_export.json', context=context)
    if parent_meta['attachments'] != meta['attachments']:
        raise ValueError('Window parent canonical attachment metadata differs')
    if bool(parent_meta.get('released_gutter_diagnostic', False)) != bool(meta.get('released_gutter_diagnostic', False)):
        raise ValueError('Window transition released-gutter policy differs')
    # This is deliberately the original identical-MJB verification, against
    # the parent's own model. Nothing weakens newton_continuation.load_resume.
    _, terminal, history, legacy = load_resume(parent, parent_inputs,
        parent_initial, parent_meta, policy=_policy(contact_policy), verification_context=context)
    bank, lifting = lift_state(common, parent_inputs, terminal, verification_context=context)
    if not np.array_equal(bank['body_names'], initial['body_names']):
        raise ValueError('Window canonical body order differs from prepared route')
    full_model = context.model(common / 'input_model.mjb')
    poses, paths, pose_check = _recorded_terminal(parent, summary, terminal, proof, full_model, bank, context=context)
    full_terminal = dict(time=bank['time'].copy(), body_names=bank['body_names'].copy(),
        attachment_equality_ids=np.asarray([full_model.equality(spec['name'] + '_attachment').id for spec in meta['attachments']], np.int32))
    for name in ('qpos', 'qvel', 'eq_active', 'ctrl', 'runtime_dof_damping', 'qacc_warmstart'):
        if name in bank:
            full_terminal[name] = bank[name][None].copy()
    evidence = dict(schema='s300-newton-checked-window-parent-v1',
        parent_run=str(parent), parent_summary_sha256=context.file_sha256(parent / 'summary.json'),
        canonical_common=str(common), canonical_mjb_sha256=context.file_sha256(common / 'input_model.mjb'),
        parent_physical_model_mjb_sha256=legacy['physical_model_mjb_sha256'],
        original_parent_same_mjb_resume_check=legacy, canonical_lift=lifting,
        recorded_actual_pose_check=pose_check, initial_clock_s=float(bank['time'][0]),
        parent_effective_gpu_options=summary.get('effective_gpu_options'),
        inherited_release_events=len(history), solver_instance_restarted=True,
        contact_constraint_cache_rebuilt=True, measured_state_not_rolled_back=True,
        training_eligible=False, harvest_success=None)
    evidence['verification_scope'] = context.evidence()
    return bank, full_terminal, history, evidence, poses, paths


def _compare_bank(expected, actual):
    if set(expected) != set(actual):
        raise ValueError('Transition canonical bank field identity differs')
    for name in expected:
        if not np.array_equal(expected[name], actual[name]):
            raise ValueError('Transition canonical bank differs from measured parent: ' + name)


def _child_state(inputs, full_bank, *, context=None):
    proof = _read(inputs / 'active_window_proof.json', context=context)
    initial = _npz(inputs / 'initial_state.npz', context=context)
    mapping = proof['joint_mapping']
    for name, old, new in (
            ('qpos', 'full_qpos', 'reduced_qpos'), ('qvel', 'full_dofs', 'reduced_dofs'),
            ('runtime_dof_damping', 'full_dofs', 'reduced_dofs'),
            ('plant_preload', 'full_dofs', 'reduced_dofs'),
            ('runtime_dof_length', 'full_dofs', 'reduced_dofs'),
            ('qacc_warmstart', 'full_dofs', 'reduced_dofs')):
        if name in full_bank:
            if name not in initial or not np.array_equal(initial[name][mapping[new]], full_bank[name][mapping[old]]):
                raise ValueError('Transition reduced initial differs from canonical bank: ' + name)
    for name in ('ctrl', 'eq_active', 'time', 'body_names', 'actuator_ids', 'actuator_kp'):
        if name not in initial or not np.array_equal(initial[name], full_bank[name]):
            raise ValueError('Transition reduced scalar/identity state differs from canonical bank: ' + name)
    # Existing runner uploads floating state to GPU float32. Newly activated
    # coordinates have no previous device representation; make that one-time
    # encoding explicit so strict device readback compares the actual dtype.
    state = dict(time=np.asarray(initial['time'], np.float32), body_names=initial['body_names'].copy())
    model = (mj.MjModel.from_binary_path(str(inputs / 'input_model.mjb')) if context is None
             else context.model(inputs / 'input_model.mjb', expected_sha256=proof['reduced_mjb_sha256']))
    meta = _read(inputs / 'source_export.json', context=context)
    canonical_mjb = Path(proof['canonical_inputs']) / 'input_model.mjb'
    if file_sha256(canonical_mjb, context=context) != proof['canonical_mjb_sha256']:
        raise ValueError('Transition canonical source MJB SHA mismatch')
    full_model = (mj.MjModel.from_binary_path(str(canonical_mjb)) if context is None
                  else context.model(canonical_mjb, expected_sha256=proof['canonical_mjb_sha256']))
    if joint_maps(full_model, model) != mapping:
        raise ValueError('Transition named joint mapping changed')
    state['attachment_equality_ids'] = np.asarray([model.equality(spec['name'] + '_attachment').id for spec in meta['attachments']], np.int32)
    for name in ('qpos', 'qvel', 'eq_active', 'ctrl', 'runtime_dof_damping', 'qacc_warmstart'):
        if name in initial:
            state[name] = np.asarray(initial[name][None], bool if name == 'eq_active' else np.float32).copy()
    return initial, state


def _encoding_evidence(initial, state):
    errors = {}
    for name in ('time', 'qpos', 'qvel', 'ctrl', 'runtime_dof_damping', 'qacc_warmstart'):
        if name in state:
            original = np.asarray(initial[name], dtype=np.float64)
            encoded = np.asarray(state[name], dtype=np.float64).reshape(original.shape)
            errors[name] = float(np.max(np.abs(original - encoded))) if original.size else 0.
    return dict(dtype='float32', maximum_abs_encoding_error_by_field=errors,
        canonical_full_bank_and_prepared_initial_unchanged=True,
        interpretation='Existing GPU runner dtype conversion, explicitly encoded for new active coordinates; not a collision/physics tolerance change')


def create_transition(inputs, parent, common, *, verification_context=None):
    """Create a new bound certificate after strict parent and child validation."""
    context = verification_context if verification_context is not None else VerificationContext()
    inputs, parent, common = (Path(path).resolve(strict=True) for path in (inputs, parent, common))
    proof = _prepared(inputs, context=context)
    meta, common_meta = _read(inputs / 'source_export.json', context=context), _read(common / 'source_export.json', context=context)
    common_meta['released_gutter_diagnostic'] = meta.get('released_gutter_diagnostic', False)
    initial = _npz(common / 'initial_state.npz', context=context)
    summary = _read(parent / 'summary.json', context=context)
    policy = summary['contact_policy']['config']
    index = proof.get('same_bed_sequence', {}).get('index')
    if not isinstance(index, int) or index < 1 or proof['same_bed_sequence'].get('source_common') != str(common):
        raise ValueError('Transition requires a subsequent selected route segment')
    bank, _, history, evidence, _, _ = checked_window_parent(parent, common, initial, common_meta, policy, index - 1, verification_context=context)
    _compare_bank(bank, _npz(inputs / 'canonical_full_state.npz', context=context))
    child_initial, child_state = _child_state(inputs, bank, context=context)
    certificate = dict(schema=SCHEMA, inputs=str(inputs), parent_run=str(parent), canonical_common=str(common),
        expected_parent_index=index - 1, new_sequence_index=index,
        input_manifest_sha256=context.file_sha256(inputs / 'inputs.json'),
        child_files={name: context.file_sha256(inputs / name) for name in
            ('input_model.mjb', 'initial_state.npz', 'source_export.json', 'canonical_full_state.npz', 'active_window_proof.json')},
        canonical_manifest_sha256=context.file_sha256(common / 'inputs.json'),
        canonical_mjb_sha256=context.file_sha256(common / 'input_model.mjb'),
        parent_summary_sha256=context.file_sha256(parent / 'summary.json'),
        parent_final_states_sha256=context.file_sha256(parent / 'final_states.npz'),
        contact_policy=policy, inherited_release_events=history,
        backend_state_encoding=_encoding_evidence(child_initial, child_state),
        initial_clock_s=float(bank['time'][0]), parent_validation=evidence,
        explicit_collision_model_transition=True, legacy_identical_mjb_resume_unchanged=True,
        solver_instance_restarted=True, training_eligible=False, harvest_success=None)
    certificate['verification_scope'] = context.evidence()
    certificate['certificate_sha256'] = _unsigned_sha(certificate)
    path = inputs / 'window_transition.json'
    if path.exists():
        raise FileExistsError('Do not overwrite an existing transition certificate')
    path.write_text(json.dumps(certificate, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    return path


def load_transition(cert, inputs, initial, metadata, policy, *, verification_context=None):
    """Recompute verified canonical lineage and return legacy runner tuple shape."""
    context = verification_context if verification_context is not None else VerificationContext()
    cert, inputs = Path(cert).resolve(strict=True), Path(inputs).resolve(strict=True)
    value = _read(cert, context=context)
    if value.get('schema') != SCHEMA or value.get('certificate_sha256') != _unsigned_sha({k: v for k, v in value.items() if k != 'certificate_sha256'}):
        raise ValueError('Window transition certificate digest mismatch')
    if value['inputs'] != str(inputs) or value['contact_policy'] != _policy(policy):
        raise ValueError('Window transition input path or contact policy mismatch')
    bindings = [(inputs / 'inputs.json', value['input_manifest_sha256']),
        (Path(value['canonical_common']) / 'inputs.json', value['canonical_manifest_sha256']),
        (Path(value['canonical_common']) / 'input_model.mjb', value['canonical_mjb_sha256']),
        (Path(value['parent_run']) / 'summary.json', value['parent_summary_sha256']),
        (Path(value['parent_run']) / 'final_states.npz', value['parent_final_states_sha256'])]
    bindings += [(inputs / name, digest) for name, digest in value['child_files'].items()]
    for path, digest in bindings:
        if context.file_sha256(path) != digest:
            raise ValueError('Window transition bound SHA mismatch: ' + str(path))
    _prepared(inputs, context=context)
    if metadata != _read(inputs / 'source_export.json', context=context):
        raise ValueError('Window transition runtime metadata differs from signed prepared metadata')
    for name, array in _npz(inputs / 'initial_state.npz', context=context).items():
        if name not in initial or not np.array_equal(initial[name], array):
            raise ValueError('Window transition runtime initial state differs: ' + name)
    common = Path(value['canonical_common'])
    common_meta = _read(common / 'source_export.json', context=context)
    common_meta['released_gutter_diagnostic'] = metadata.get('released_gutter_diagnostic', False)
    bank, _, history, evidence, _, _ = checked_window_parent(value['parent_run'], common,
        _npz(common / 'initial_state.npz', context=context), common_meta, policy, value['expected_parent_index'], verification_context=context)
    _compare_bank(bank, _npz(inputs / 'canonical_full_state.npz', context=context))
    if history != value['inherited_release_events'] or float(bank['time'][0]) != value['initial_clock_s']:
        raise ValueError('Window transition inherited history/time changed')
    restored, resume = _child_state(inputs, bank, context=context)
    encoding = _encoding_evidence(restored, resume)
    if encoding != value['backend_state_encoding']:
        raise ValueError('Window transition backend state encoding changed')
    continuation = dict(schema=SCHEMA, parent_run=value['parent_run'],
        parent_summary_sha256=value['parent_summary_sha256'],
        parent_final_states_sha256=value['parent_final_states_sha256'],
        physical_model_mjb_sha256=value['child_files']['input_model.mjb'],
        parent_physical_model_mjb_sha256=evidence['parent_physical_model_mjb_sha256'],
        canonical_mjb_sha256=value['canonical_mjb_sha256'], initial_clock_s=float(bank['time'][0]),
        inherited_release_events=len(history), qpos_qvel_eq_ctrl_damping_preserved=True,
        qacc_warmstart_restored='qacc_warmstart' in resume,
        parent_effective_gpu_options=evidence['parent_effective_gpu_options'],
        solver_instance_restarted=True, contact_constraint_cache_rebuilt=True,
        measured_state_not_rolled_back=True, explicit_active_window_transition=True,
        certificate=str(cert), certificate_sha256=value['certificate_sha256'],
        backend_state_encoding=encoding,
        legacy_identical_mjb_resume_unchanged=True,
        limitations=['Inactive plants keep measured fixed collider poses; this is not full simultaneous elasticity.',
                    'Fresh solver/contact caches are rebuilt; only named measured state and history continue.'])
    continuation['verification_scope'] = context.evidence()
    return restored, resume, history, continuation
