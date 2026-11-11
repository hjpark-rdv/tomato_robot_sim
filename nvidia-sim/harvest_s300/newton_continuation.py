"""Verified measured-state continuation for the existing Newton GPU executor.

This module does not execute physics or create plans. A continuation starts a
new solver instance on exactly the same signed MJB and restores the measured
terminal state; solver contact caches other than qacc_warmstart are rebuilt.
"""
from __future__ import annotations

import hashlib
import json
import os
import stat
import threading
from pathlib import Path

import numpy as np


def file_sha256(path, *, context=None):
    if context is not None:
        return context.file_sha256(path)
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


class VerificationContext:
    """Caller-owned verified digest/read-only model reuse, never a global cache.

    Each distinct resolved source is hashed once from its original bytes. A
    repeated request rechecks device/inode/size/mtime/ctime and symlink identity
    before reusing that digest. A changed source is rejected within this scope.
    Models are shared only by the read-only verification functions, keyed by
    that exact stable source rather than by a digest of another mutable file.
    """
    def __init__(self, *, local_model_cache=None, scope='call'):
        if scope not in ('call','patrol'):raise ValueError('Unknown verification lifetime')
        self.scope=scope
        self._local_model_cache = None if local_model_cache is None else Path(local_model_cache)
        self._guard = threading.RLock()
        self._files = {}
        self._aliases = {}
        self._hash_reads = self._hash_hits = self._model_reads = self._model_hits = 0
        self._bytes_read = self._bytes_avoided = 0

    @staticmethod
    def _identity(value):
        return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)

    def _entry(self, path):
        requested = Path(os.path.abspath(os.fspath(path)))
        resolved = requested.resolve(strict=True)
        info = resolved.stat()
        if not stat.S_ISREG(info.st_mode):
            raise ValueError('Verification source must be a regular file: ' + str(path))
        identity = self._identity(info)
        with self._guard:
            old_alias = self._aliases.setdefault(str(requested), str(resolved))
            if old_alias != str(resolved):
                raise ValueError('Verification source changed within scope: ' + str(path))
            entry = self._files.get(str(resolved))
            if entry is None:
                entry = dict(identity=identity, sha256=None, model=None, lock=threading.RLock())
                self._files[str(resolved)] = entry
            elif entry['identity'] != identity:
                raise ValueError('Verification source changed within scope: ' + str(path))
        return resolved, entry

    def _stable(self, resolved, entry):
        if self._identity(Path(resolved).stat()) != entry['identity']:
            raise ValueError('Verification source changed within scope: ' + str(resolved))

    def file_sha256(self, path):
        resolved, entry = self._entry(path)
        with entry['lock']:
            self._stable(resolved, entry)
            if entry['sha256'] is not None:
                with self._guard:
                    self._hash_hits += 1; self._bytes_avoided += entry['identity'][2]
                return entry['sha256']
            digest = hashlib.sha256(); count = 0
            fd = os.open(resolved, os.O_RDONLY | os.O_CLOEXEC | os.O_NONBLOCK)
            with os.fdopen(fd, 'rb') as stream:
                before = os.fstat(stream.fileno())
                if not stat.S_ISREG(before.st_mode) or self._identity(before) != entry['identity']:
                    raise ValueError('Verification source changed before hashing: ' + str(resolved))
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(block); count += len(block)
                if self._identity(os.fstat(stream.fileno())) != entry['identity'] or count != before.st_size:
                    raise ValueError('Verification source changed while hashing: ' + str(resolved))
            self._stable(resolved, entry)
            entry['sha256'] = digest.hexdigest()
            with self._guard:
                self._hash_reads += 1; self._bytes_read += count
            return entry['sha256']

    def model(self, path, *, expected_sha256=None):
        """Return the verified MjModel for read-only kinematic/name queries only."""
        actual = self.file_sha256(path)
        if expected_sha256 is not None and actual != expected_sha256:
            raise ValueError('Verification model SHA mismatch: ' + str(path))
        resolved, entry = self._entry(path)
        with entry['lock']:
            self._stable(resolved, entry)
            if entry['model'] is None:
                import mujoco as mj
                # Always hash each original above. Loading a byte-identical
                # already existing local cache avoids a second NAS model read;
                # the cache is independently hashed in this same scope. No
                # model/cache file is created, updated or trusted by filename.
                read_path = resolved
                if self._local_model_cache is not None:
                    candidate = self._local_model_cache / (actual + '.mjb')
                    if candidate.is_file():
                        if self.file_sha256(candidate) != actual:
                            raise ValueError('Verification model cache SHA mismatch: ' + str(candidate))
                        read_path = candidate.resolve(strict=True)
                value = mj.MjModel.from_binary_path(str(read_path))
                self._stable(resolved, entry)
                self._entry(read_path)
                entry['model'] = value
                entry['model_read_path'] = str(read_path)
                entry['verified_local_model_cache_used'] = read_path != resolved
                with self._guard: self._model_reads += 1
            else:
                with self._guard: self._model_hits += 1
            return entry['model']

    def readonly_json(self,path):
        """Reuse parsed immutable input metadata after the same file checks.

        Internal callers must not mutate nested values; take a top-level copy
        for runtime flags. This never caches changing summaries/checkpoints.
        """
        self.file_sha256(path)
        resolved,entry=self._entry(path)
        with entry['lock']:
            self._stable(resolved,entry)
            if 'json_value' not in entry:
                entry['json_value']=json.loads(resolved.read_text())
                self._stable(resolved,entry)
            return entry['json_value']

    def evidence(self):
        with self._guard:
            for requested, resolved in self._aliases.items():
                if str(Path(requested).resolve(strict=True)) != resolved:
                    raise ValueError('Verification source changed within scope: ' + requested)
            for path, entry in self._files.items(): self._stable(path, entry)
            return dict(schema='s300-call-scoped-file-model-verification-v1',
                scope=self.scope,unique_file_sha_reads=self._hash_reads, stable_sha_reuses=self._hash_hits,
                unique_read_only_model_loads=self._model_reads, stable_read_only_model_reuses=self._model_hits,
                verified_local_model_loads=sum(bool(e.get('verified_local_model_cache_used')) for e in self._files.values()),
                original_bytes_hashed=self._bytes_read, repeated_hash_bytes_avoided=self._bytes_avoided,
                original_file_sha_checks_preserved=True, file_identity_change_rejected=True,
                global_or_cross_call_cache=self.scope!='call',global_cache=False,
                models_used_only_by_read_only_verification=True,
                sources=[dict(path=path, sha256=entry['sha256'],
                    identity=list(entry['identity']), read_only_model_loaded=entry['model'] is not None,
                    model_read_path=entry.get('model_read_path'),
                    verified_local_model_cache_used=entry.get('verified_local_model_cache_used', False))
                    for path, entry in self._files.items()])


def model_identity(initial, metadata):
    """Bind exact body/attachment and robot address order to the MJB SHA."""
    value = dict(body_names=np.asarray(initial['body_names']).tolist(),
        attachments=metadata['attachments'],
        robot_qpos_addresses=np.asarray(initial['robot_qpos_addresses']).tolist(),
        robot_dof_addresses=np.asarray(initial['robot_dof_addresses']).tolist(),
        actuator_ids=np.asarray(initial['actuator_ids']).tolist())
    value['sha256'] = hashlib.sha256(json.dumps(value, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    return value


def _checked_json(path, expected=None, *, context=None):
    path = Path(path)
    if expected is not None and file_sha256(path, context=context) != expected:
        raise ValueError('Continuation SHA mismatch: ' + str(path))
    return json.loads(path.read_text())


def load_resume(previous, inputs, initial, metadata, *, worlds=1, policy=None, verification_context=None):
    """Return a replacement initial dictionary and auditable measured lineage.

    Shapes must match exactly; no truncation, joint remapping, defaults or
    tolerance-based pose correction is used. Legacy snapshots lacking runtime
    damping/control are deliberately insufficient for a physical continuation.
    """
    def checked_sha(path):
        return file_sha256(path, context=verification_context)
    if worlds != 1:
        raise ValueError('Measured continuation currently requires exactly one GPU world')
    previous, inputs = Path(previous).resolve(), Path(inputs).resolve()
    summary_path = previous / 'summary.json'
    summary = _checked_json(summary_path, context=verification_context)
    if (summary.get('backend') != 'gpu' or summary.get('worlds') != worlds or
            summary.get('status') != 'completed' or summary.get('stop') is not None or
            any(summary.get(key) is not True for key in
                ('finite_recorded_state', 'qacc_finite', 'solver_capacity_valid',
                 'device_command_buffers_match_schedule', 'controller_tick_matches_recorded_steps')) or
            summary.get('actual_steps') != summary.get('requested_steps') or
            int(summary.get('actual_steps', 0)) < 1):
        raise ValueError('Continuation requires a fully completed finite, capacity-valid GPU execution')
    previous_policy = summary.get('contact_policy')
    if policy is None or not previous_policy or previous_policy.get('policy_passed') is not True:
        raise ValueError('Continuation requires the previous exact contact policy to have passed')
    if previous_policy.get('config') != policy:
        raise ValueError('Continuation contact policy differs from the completed parent')
    if bool(summary.get('released_gutter_diagnostic', False)) != bool(metadata.get('released_gutter_diagnostic', False)):
        raise ValueError('Continuation released-gutter policy differs from the completed parent')
    commands_path = previous / 'executed_commands.npz'
    expected_commands_sha = summary.get('executed_commands_sha256')
    if not expected_commands_sha or checked_sha(commands_path) != expected_commands_sha:
        raise ValueError('Continuation executed command schedule SHA mismatch or absent')
    with np.load(commands_path, allow_pickle=False) as archive:
        if len(archive['time']) != summary['actual_steps']:
            raise ValueError('Continuation parent stopped before its complete command timeline')
        online=summary.get('online_camera_boundary')
        if online is not None:
            proposed=previous/'proposed_commands.npz'
            if (online.get('completion_kind')!='online_observation_endpoint' or
                    online.get('executed_steps')!=summary['actual_steps'] or
                    checked_sha(proposed)!=online.get('proposed_commands_sha256')):
                raise ValueError('Online observation endpoint certificate differs')
            with np.load(proposed,allow_pickle=False) as full:
                if len(full['time'])!=online.get('maximum_planned_steps'):
                    raise ValueError('Online proposed timeline length differs')
                for field in ('command','phase','time','dt','belt_surface_velocity_local_y_m_s'):
                    if not np.array_equal(archive[field],full[field][:summary['actual_steps']]):
                        raise ValueError('Online executed schedule is not the exact proposed prefix: '+field)
    parent_inputs = Path(summary['input'])
    parent_proof = _checked_json(parent_inputs / 'inputs.json', summary['input_sha256'], context=verification_context)
    source_sha = checked_sha(inputs / 'input_model.mjb')
    expected_sha = parent_proof['input_files']['input_model.mjb']['sha256']
    if source_sha != expected_sha or checked_sha(parent_inputs / 'input_model.mjb') != source_sha:
        raise ValueError('Continuation requires the identical source physical MJB SHA')
    parent_initial_path = parent_inputs / 'initial_state.npz'
    parent_initial_sha = parent_proof['input_files'].get('initial_state.npz', {}).get('sha256')
    if parent_initial_sha:
        if checked_sha(parent_initial_path) != parent_initial_sha:
            raise ValueError('Continuation parent prepared initial state SHA mismatch')
        with np.load(parent_initial_path, allow_pickle=False) as archive:
            for field in ('plant_preload', 'runtime_dof_length', 'actuator_kp'):
                if field in initial and (field not in archive or not np.array_equal(initial[field], archive[field])):
                    raise ValueError('Continuation fixed physics adapter parameter changed: ' + field)
    identity = model_identity(initial, metadata)
    if summary.get('continuation_model_identity') != identity:
        raise ValueError('Continuation body/attachment/robot model identity mismatch or absent')
    state_path = previous / 'final_states.npz'
    if not summary.get('final_states_sha256') or checked_sha(state_path) != summary['final_states_sha256']:
        raise ValueError('Continuation final-state SHA mismatch or absent')
    with np.load(state_path, allow_pickle=False) as archive:
        state = {key: archive[key].copy() for key in archive.files}
    required = ('time', 'qpos', 'qvel', 'eq_active', 'ctrl', 'runtime_dof_damping',
                'body_names', 'attachment_equality_ids')
    if any(key not in state for key in required):
        raise ValueError('Continuation snapshot lacks actual control/damping/body identity')
    if not np.array_equal(state['body_names'], initial['body_names']):
        raise ValueError('Continuation snapshot body order mismatch')
    time = np.asarray(state['time'])
    if time.shape != (1,) or not np.isfinite(time).all() or time[0] <= 0:
        raise ValueError('Continuation needs a finite positive measured terminal clock')
    if summary.get('final_physics_clock_s') != float(time[0]):
        raise ValueError('Continuation final clock disagrees with the completed parent summary')
    result = {key: np.asarray(value).copy() for key, value in initial.items()}
    for key in ('qpos', 'qvel', 'eq_active', 'ctrl', 'runtime_dof_damping'):
        expected_shape = (worlds, *np.asarray(initial[key]).shape)
        if state[key].shape != expected_shape or not np.isfinite(state[key]).all():
            raise ValueError('Continuation snapshot shape/nonfinite mismatch: ' + key)
        result[key] = state[key][0].copy()
    if state['eq_active'].dtype.kind != 'b':
        raise ValueError('Continuation attachment activity must retain boolean values')
    warmstart = state.get('qacc_warmstart')
    if warmstart is not None and (warmstart.shape != state['qvel'].shape or not np.isfinite(warmstart).all()):
        raise ValueError('Continuation qacc_warmstart shape/nonfinite mismatch')
    equality_ids = state['attachment_equality_ids']
    if (equality_ids.shape != (len(metadata['attachments']),) or
            equality_ids.dtype.kind not in 'iu' or len(set(equality_ids)) != len(equality_ids) or
            np.any(equality_ids < 0) or np.any(equality_ids >= len(result['eq_active']))):
        raise ValueError('Continuation attachment equality identity mismatch')
    if summary.get('cumulative_break_events_sha256'):
        inherited = _checked_json(previous / 'cumulative_break_events.json',
                                summary['cumulative_break_events_sha256'], context=verification_context)
    else:
        inherited = summary.get('breaks', [])
    names = {row['name'] for row in metadata['attachments']}
    seen = set()
    for row in inherited:
        key = (row.get('world', 0), row.get('name'))
        if key[0] != 0 or key[1] not in names or key in seen:
            raise ValueError('Continuation contains duplicate/unknown inherited release identity')
        if not np.isfinite(row['release_effective_time_s']) or not 0 <= row['release_effective_time_s'] <= time[0]:
            raise ValueError('Continuation inherited release lies outside the measured clock')
        seen.add(key)
    # The snapshot equality order is also bound by the exact same MJB. Parent
    # events cannot claim that an attachment is released while it is active.
    for row in inherited:
        index = [spec['name'] for spec in metadata['attachments']].index(row['name'])
        if result['eq_active'][equality_ids[index]]:
            raise ValueError('Continuation release history disagrees with active attachment state')
    evidence = dict(schema='s300-newton-measured-continuation-v1',
        parent_run=str(previous), parent_summary_sha256=checked_sha(summary_path),
        parent_final_states_sha256=summary['final_states_sha256'],
        physical_model_mjb_sha256=source_sha, exact_model_identity=identity,
        initial_clock_s=float(time[0]), inherited_release_events=len(inherited),
        qpos_qvel_eq_ctrl_damping_preserved=True,
        qacc_warmstart_restored=warmstart is not None,
        parent_effective_gpu_options=summary.get('effective_gpu_options'),
        solver_instance_restarted=True, contact_constraint_cache_rebuilt=True,
        measured_state_not_rolled_back=True,
        limitations=['Fresh solver contact/constraint caches are rebuilt; this is state-preserving segmented execution, not one persistent solver instance.'])
    if verification_context is not None:
        evidence['verification_scope'] = verification_context.evidence()
    return result, state, inherited, evidence


def verify_command_start(timeline, initial):
    """Require the new plan to start at the exact measured robot configuration."""
    expected = initial['qpos'][initial['robot_qpos_addresses']]
    actual = np.asarray(timeline['command'])[0]
    if actual.shape != expected.shape or not np.allclose(actual, expected, rtol=0, atol=1e-6):
        raise ValueError('Continuation commands do not start at the measured final robot q')
    return float(np.max(np.abs(actual - expected)))


def cumulative_breaks(inherited, new):
    result = [dict(row) for row in inherited]
    seen = {(row.get('world', 0), row['name']) for row in result}
    for row in new:
        key = (row.get('world', 0), row['name'])
        if key in seen:
            raise ValueError('Continuation would recount an already released fruit')
        seen.add(key)
        result.append(dict(row))
    return result
