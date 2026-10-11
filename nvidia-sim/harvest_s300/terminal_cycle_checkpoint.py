"""Publish a missed final cycle marker from the exact measured final snapshot.

Compatibility only: a one-cycle run may exit its loop at the exact planned
endpoint before the next loop-boundary checkpoint call. This module never
steps physics or changes measured state. It requires finished native/audit
evidence and leaves the original final route checkpoint untouched.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from compact_motion_assets import file_sha256
from native_route_checkpoint import load_checkpoint,save_checkpoint
from resume_native_state import require_completed_checkpoint

SCHEMA='s300-exact-terminal-cycle-publication-v1'


def _digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),
        ensure_ascii=False,allow_nan=False).encode()).hexdigest()


def _source_evidence(raw_checkpoint):
    raw_checkpoint=Path(raw_checkpoint).resolve()
    if raw_checkpoint.name!='route_checkpoint':
        raise ValueError('Publication requires the exact raw final route_checkpoint')
    metadata,arrays=load_checkpoint(raw_checkpoint);trial=raw_checkpoint.parent
    summary_path=trial/'summary.json';audit_path=trial/'self_collision_actual.json';states_path=trial/'states.npz'
    summary=json.loads(summary_path.read_text());audit=json.loads(audit_path.read_text())
    timestamp=metadata['time_s'];sequence=metadata.get('sequence')
    if (summary.get('mode')!='harvest' or summary.get('completed') is not True or
            summary.get('initialization_valid') is not True or summary.get('stop_reason') is not None or
            summary.get('actual_self_collision_audit_passed') is not True or
            summary.get('model_sha256')!=metadata['scene_sha256'] or
            summary.get('simulated_s')!=timestamp or
            Path(summary.get('scene','')).resolve()!=Path(metadata['source_scene']) or
            summary.get('fixture_only') is not False or summary.get('disabled_background_colliders')!=0):
        raise ValueError('Final native summary is incomplete, invalid, stopped or not the exact source state')
    if (not isinstance(sequence,dict) or sequence!=summary.get('truss_sequence') or
            not isinstance(sequence.get('cycle_plans'),list) or not sequence['cycle_plans']):
        raise ValueError('No exact completed native manipulation cycle in the raw final snapshot')
    record=sequence['cycle_plans'][-1]
    planned_end=record.get('planned_end_s');end=record.get('end_s')
    if (record.get('status')!='motion_completed' or record.get('stop_reason') is not None or
            isinstance(planned_end,bool) or not isinstance(planned_end,(int,float)) or
            not np.isfinite(planned_end) or isinstance(end,bool) or not isinstance(end,(int,float)) or
            not np.isfinite(end) or abs(end-timestamp)>1e-8 or abs(planned_end-timestamp)>1e-8 or
            isinstance(record.get('start_s'),bool) or not isinstance(record.get('start_s'),(int,float)) or
            not 0<=record['start_s']<timestamp):
        raise ValueError('Raw final cycle did not complete at its exact planned/native endpoint')
    if metadata.get('controller_state') is None:
        raise ValueError('Final publication requires exact native drive-target capture')
    hz=summary.get('physics_settings',{}).get('hz')
    if isinstance(hz,bool) or not isinstance(hz,(int,float)) or not np.isfinite(hz) or hz<=0:
        raise ValueError('Exact source native physics Hz is required')
    if (audit.get('passed') is not True or audit.get('complete') is not True or
            audit.get('scene_sha256')!=metadata['scene_sha256'] or
            audit.get('input_sha256')!=file_sha256(states_path) or audit.get('physics_rerun') is not False):
        raise ValueError('Final publication requires the complete exact stored-state audit')
    with np.load(states_path,allow_pickle=False) as stored:
        time=np.asarray(stored['time']);poses=np.asarray(stored['poses']);q=np.asarray(stored['robot_q'])
        paths=list(map(str,stored['body_paths']))
    expected=np.arange(round(timestamp*hz)+1)/hz
    if (time.shape!=expected.shape or not np.isfinite(time).all() or
            not np.allclose(time,expected,rtol=0.,atol=1e-8) or abs(time[-1]-timestamp)>1e-8 or
            audit.get('samples')!=len(time) or summary.get('physics_steps')!=len(time)-1 or
            record.get('end_frame_index')!=len(time)-1 or
            paths!=list(map(str,arrays['body_paths'])) or
            poses.shape!=(len(time),len(paths),7) or not np.isfinite(poses).all() or
            q.shape!=(len(time),len(arrays['robot_dof_names'])) or not np.isfinite(q).all() or
            not np.array_equal(poses[-1],arrays['body_poses_world']) or
            not np.array_equal(q[-1],arrays['robot_q'])):
        raise ValueError('Stored native trace does not cover the exact final measured snapshot')
    proof=dict(schema=SCHEMA,raw_final_checkpoint=str(raw_checkpoint),
        raw_metadata_sha256=metadata['metadata_sha256'],raw_npz_sha256=metadata['checkpoint_npz_sha256'],
        source_summary=str(summary_path),source_summary_sha256=file_sha256(summary_path),
        actual_self_audit=str(audit_path),actual_self_audit_sha256=file_sha256(audit_path),
        actual_states=str(states_path),actual_states_sha256=file_sha256(states_path),
        source_sequence_sha256=_digest(sequence),scene_sha256=metadata['scene_sha256'],
        materialized_dependency=copy.deepcopy(metadata['manifest'].get('compact_assets',{}).get('materialized_dependency')),
        actual_final_time_s=timestamp,native_hz=float(hz),actual_samples=len(time),
        measured_arrays_unchanged=True,native_controller_targets_unchanged=True,
        native_break_events_unchanged=True,raw_checkpoint_unchanged=True,
        physics_executed=False,cycle_invented=False,
        scope='Offline missing final marker publication only; exact existing completed native cycle and final measured state')
    return metadata,arrays,proof


def _published_sequence(metadata,proof):
    sequence=copy.deepcopy(metadata['sequence'])
    sequence.update(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True,
        terminal_snapshot_publication=proof)
    return sequence


def verify_publication(metadata,arrays,checkpoint):
    proof=metadata.get('sequence',{}).get('terminal_snapshot_publication')
    if not isinstance(proof,dict) or proof.get('schema')!=SCHEMA:
        raise ValueError('Missing exact terminal snapshot publication proof')
    raw,source_arrays,expected=_source_evidence(proof['raw_final_checkpoint'])
    checkpoint=Path(checkpoint).resolve()
    if checkpoint.parent!=Path(proof['raw_final_checkpoint']).parent/'cycle_checkpoints':
        raise ValueError('Published cycle must retain exact original trial provenance')
    if proof!=expected or metadata['sequence']!=_published_sequence(raw,expected):
        raise ValueError('Terminal publication source/proof/sequence changed')
    for field in ('source_assembly','source_scene','scene_sha256','source_plant','manifest','time_s','break_events','controller_state'):
        if metadata.get(field)!=raw.get(field):raise ValueError('Terminal publication changed native identity/state metadata: '+field)
    if set(arrays)!=set(source_arrays) or any(
            arrays[key].dtype!=source_arrays[key].dtype or not np.array_equal(arrays[key],source_arrays[key])for key in arrays):
        raise ValueError('Terminal publication changed measured native arrays')
    require_completed_checkpoint(metadata)
    return copy.deepcopy(proof)


def publish_final_cycle(raw_checkpoint,output):
    raw_checkpoint,output=map(lambda path:Path(path).resolve(),(raw_checkpoint,output))
    if output.exists():raise FileExistsError(output)
    if output.parent!=raw_checkpoint.parent/'cycle_checkpoints':
        raise ValueError('New cycle checkpoint must stay in the original native trial')
    metadata,arrays,proof=_source_evidence(raw_checkpoint)
    sequence=_published_sequence(metadata,proof)
    controller=metadata['controller_state']
    saved=save_checkpoint(output,assembly=metadata['source_assembly'],scene_sha256=metadata['scene_sha256'],
        time_s=metadata['time_s'],**arrays,breaks=metadata['break_events'],manifest=metadata['manifest'],
        sequence=sequence,source_plant=metadata['source_plant'],
        robot_position_targets=controller['position_targets'],robot_velocity_targets=controller['velocity_targets'])
    restored,values=load_checkpoint(output)
    verify_publication(restored,values,output)
    if restored!=saved:raise ValueError('Published checkpoint round-trip metadata mismatch')
    return dict(checkpoint=str(output),metadata_sha256=saved['metadata_sha256'],proof=proof,
        training_eligible=False,harvest_success=None)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw_checkpoint',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();print(json.dumps(publish_final_cycle(args.raw_checkpoint,args.output),indent=2))
