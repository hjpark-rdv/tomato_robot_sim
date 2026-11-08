"""Exact terminal-state proof for one opt-in planning-only software halt.

This never promotes the parent motion to completed, edits an artifact, restores
an earlier state, or makes an early negative path audit a positive certificate.
"""
import json
from pathlib import Path

import numpy as np

from compact_motion_assets import file_sha256
from native_route_checkpoint import load_checkpoint
from resume_native_state import require_completed_checkpoint, imported_break_markers
from robot_controller_state import resolve_robot_controller_state


def verify_planning_boundary_resume(assembly, manifest, checkpoint, metadata, arrays, summary, trial):
    """Require the completed boundary to equal every stored final native value."""
    # Import at call time to reuse the existing classifier without a second
    # implementation or a module import cycle through the row controller.
    from run_row_harvest import unexecuted_planning_failure
    assembly, checkpoint, trial = map(lambda p: Path(p).resolve(), (assembly, checkpoint, trial))
    summary_path = trial/'summary.json'
    summary_sha = file_sha256(summary_path)
    if (json.loads(summary_path.read_text()) != summary or summary.get('completed') is not False or
            summary.get('stop_reason') != 'gt_group_planning_inconclusive' or
            summary.get('initialization_valid') is not True or
            summary.get('actual_self_collision_audit_passed') is not True):
        raise ValueError('Planning boundary resume requires the exact known planning-only parent halt')
    require_completed_checkpoint(metadata)
    if (checkpoint.parent.name != 'cycle_checkpoints' or checkpoint.parent.parent != trial or
            metadata['manifest'] != manifest or Path(metadata['source_assembly']).resolve() != assembly):
        raise ValueError('Planning boundary checkpoint/source identity mismatch')
    final_path = trial/'route_checkpoint'
    final, terminal = load_checkpoint(final_path, expected_scene_sha256=manifest['scene_sha256'])
    if (final['manifest'] != manifest or Path(final['source_assembly']).resolve() != assembly or
            summary.get('model_sha256') != manifest['scene_sha256']):
        raise ValueError('Planning boundary final snapshot/model identity mismatch')
    stamp = metadata['time_s']
    if summary.get('simulated_s') != stamp or final['time_s'] != stamp:
        raise ValueError('Planning boundary checkpoint precedes or differs from the final physical time')
    if set(arrays) != set(terminal) or any(not np.array_equal(arrays[k], terminal[k]) for k in arrays):
        raise ValueError('Planning boundary/final native arrays or mappings differ')
    if (metadata.get('controller_state') != final.get('controller_state') or
            metadata.get('controller_state') is None):
        raise ValueError('Planning boundary/final native controller targets differ or are absent')
    controller = resolve_robot_controller_state(metadata, arrays, checkpoint)
    breaks_path = trial/'break_events.json'
    if (metadata['break_events'] != final['break_events'] or
            metadata['break_events'] != json.loads(breaks_path.read_text())):
        raise ValueError('Planning boundary/final native break events differ')
    sequence_path = trial/'truss_sequence.json'
    sequence = json.loads(sequence_path.read_text())
    previous = metadata['sequence']
    if sequence != final['sequence'] or sequence.get('termination') != summary['stop_reason']:
        raise ValueError('Planning boundary final sequence/termination mismatch')
    for key, value in previous.items():
        if key not in {'cycle_plans', 'termination', 'checkpoint_capture_reason', 'checkpoint_physics_valid'}:
            if sequence.get(key) != value:
                raise ValueError('Planning-only halt changed the completed physical ledger: '+key)
    completed = previous['cycle_plans']
    records = sequence.get('cycle_plans', [])
    if records[:len(completed)] != completed or len(records) <= len(completed):
        raise ValueError('Planning boundary completed-cycle prefix was changed or has no rejected suffix')
    states_path = trial/'states.npz'
    with np.load(states_path, allow_pickle=False) as states:
        required = {'time', 'body_paths', 'poses', 'robot_q', 'command', 'phase', 'cycle_index'}
        if not required <= set(states.files):
            raise ValueError('Planning boundary requires the final actual trace mappings and controls')
        times = states['time']; count = len(times)
        steps = summary.get('physics_steps')
        if (times.ndim != 1 or count < 2 or not np.isfinite(times).all() or
                np.any(np.diff(times) <= 0) or times[-1] != stamp or
                isinstance(steps, bool) or not isinstance(steps, int) or steps != count-1 or
                completed[-1].get('end_frame_index') != count-1):
            raise ValueError('Planning boundary is not the last completed actual physics frame')
        if (not np.array_equal(states['body_paths'].astype(str), arrays['body_paths'].astype(str)) or
                states['poses'].shape != (count, len(arrays['body_paths']), 7) or
                states['robot_q'].shape != (count, len(arrays['robot_q'])) or
                states['command'].shape != (count, len(arrays['robot_q']))):
            raise ValueError('Planning boundary actual trace/native identity mapping mismatch')
        for key, value in [('poses', arrays['body_poses_world']), ('robot_q', arrays['robot_q']),
                           ('command', np.asarray(metadata['controller_state']['position_targets']))]:
            recorded = states[key]
            if (recorded.dtype.kind not in 'fi' or not np.isfinite(recorded).all() or
                    not np.array_equal(np.asarray(value).astype(recorded.dtype), recorded[-1])):
                raise ValueError('Planning boundary differs from final actual trace '+key)
        if (states['phase'].shape != (count,) or states['cycle_index'].shape != (count,) or
                str(states['phase'][-1]) != 'hold' or
                int(states['cycle_index'][-1]) != completed[-1]['index']):
            raise ValueError('Planning boundary trace did not finish at the completed hold')
    proofs = []
    for record in records[len(completed):]:
        heading = record.get('gt_group_heading_search', {})
        environment = record.get('environment_preflight', {})
        index = record.get('index')
        if (isinstance(index, bool) or not isinstance(index, int) or index != len(completed)+len(proofs) or
                record.get('status') not in {'planning_rejected', 'planning_inconclusive'} or
                record.get('start_s') != stamp or record.get('measured_frame_index') != count-1 or
                heading.get('boundary_time_s') != stamp or heading.get('native_state_reset') is not False or
                Path(heading.get('checkpoint', '')).resolve() != checkpoint or
                Path(environment.get('checkpoint', '')).resolve() != checkpoint):
            raise ValueError('Planning boundary suffix was executed, reset, or belongs to another measured state')
        folder = trial/'cycles'/f'{index:03d}'
        proof = unexecuted_planning_failure(dict(returncode=1, timeout=False, summary=None), folder, assembly,
                                            allow_early_blocked_environment=True)
        if proof is None or proof.get('reason') != 'nominal_environment_proposal_blocked_early_witness':
            raise ValueError('Planning boundary suffix lacks its exact known early-negative environment witness')
        audit_path = folder/'full_environment_preflight.json'
        audit = json.loads(audit_path.read_text())
        binding = audit.get('checkpoint_proof', {})
        dependency = manifest.get('compact_assets', {}).get('materialized_dependency')
        dependency_sha = None if dependency is None else dependency['sha256']
        if (Path(binding.get('checkpoint', '')).resolve() != checkpoint or
                binding.get('metadata_sha256') != metadata['metadata_sha256'] or
                binding.get('npz_sha256') != metadata['checkpoint_npz_sha256'] or
                binding.get('checkpoint_json_sha256') != file_sha256(checkpoint/'checkpoint.json') or
                binding.get('source_materialized_dependency_sha256') != dependency_sha or
                binding.get('current_materialized_dependency_sha256') != dependency_sha or
                binding.get('native_break_events') != imported_break_markers(metadata)):
            raise ValueError('Planning boundary environment audit checkpoint identity mismatch')
        proof['candidate_physics_started'] = proof.pop('native_physics_started')
        proof['native_session_already_active'] = True
        proofs.append(proof)
    if file_sha256(summary_path) != summary_sha:
        raise ValueError('Planning boundary parent summary changed during verification')
    return dict(schema='s300-terminal-planning-boundary-resume-v1',
        parent_motion_completed=False, original_stop_reason=summary['stop_reason'],
        clean_physical_boundary=True, resuming_without_rollback=True,
        checkpoint=str(checkpoint), checkpoint_metadata_sha256=metadata['metadata_sha256'],
        checkpoint_npz_sha256=metadata['checkpoint_npz_sha256'], time_s=stamp,
        parent_summary=str(summary_path), parent_summary_sha256=summary_sha,
        final_raw_checkpoint=str(final_path), final_metadata_sha256=final['metadata_sha256'],
        final_checkpoint_npz_sha256=final['checkpoint_npz_sha256'],
        final_native_arrays_controller_and_breaks_exact=True,
        actual_states=str(states_path), actual_states_sha256=file_sha256(states_path),
        actual_final_frame_index=count-1, physical_steps_after_completed_boundary=0,
        controller_state=controller['provenance'], planning_only_suffix=proofs,
        parent_artifacts_modified=False, harvest_completion_credit=0,
        continuation='Same exact measured terminal state and controller; a new PhysX solver starts without warm-start caches',
        training_eligible=False, harvest_success=None)
