"""Preserve proven drive commands separately from measured articulation state."""
import json
from pathlib import Path

import numpy as np

from compact_motion_assets import file_sha256


def _vector(value,names,label):
    if not isinstance(value,list) or len(value)!=len(names) or any(isinstance(x,bool) for x in value):
        raise ValueError('Controller '+label+' target shape/type mismatch')
    result=np.asarray(value,dtype=float)
    if result.shape!=(len(names),) or not np.isfinite(result).all():
        raise ValueError('Controller '+label+' targets must be finite')
    return result


def resolve_robot_controller_state(metadata,arrays,checkpoint):
    """Return native targets or an exactly matched logged-setter fallback.

    Missing legacy evidence stays explicit unknown. It is never converted to
    measured q/qvel under the name of controller-state preservation.
    """
    names=list(map(str,arrays['robot_dof_names']))
    common=dict(checkpoint=str(Path(checkpoint).resolve()),checkpoint_metadata_sha256=metadata['metadata_sha256'],
        measured_q_is_not_drive_target=True,measured_qvel_is_not_drive_velocity_target=True)
    state=metadata.get('controller_state')
    if state is not None:
        if (state.get('schema')!='native_robot_drive_targets_v1' or state.get('robot_dof_names')!=names or
                state.get('position_source')!='native_get_dof_position_targets' or
                state.get('velocity_source')!='native_get_dof_velocity_targets'):
            raise ValueError('Native controller target identity/provenance mismatch')
        position=_vector(state.get('position_targets'),names,'position')
        velocity=_vector(state.get('velocity_targets'),names,'velocity')
        return dict(position_targets=position,velocity_targets=velocity,
            provenance=dict(common,source='checkpoint_native_drive_getters',controller_continuity_verified=True,
                position_targets_verified=True,velocity_targets_verified=True,
                position_source=state['position_source'],velocity_source=state['velocity_source']))
    trial=Path(checkpoint).resolve().parent.parent
    paths={name:trial/name for name in ('states.npz','summary.json','native_metadata.json','code/run_motion.py')}
    if not all(path.is_file() for path in paths.values()):
        return dict(position_targets=None,velocity_targets=None,
            provenance=dict(common,source='legacy_controller_targets_unknown',controller_continuity_verified=False,
                position_targets_verified=False,velocity_targets_verified=False,
                missing_proof_files=[str(path) for path in paths.values() if not path.is_file()],
                velocity_source='unknown; preserve authored source drive velocity, never measured qvel'))
    assembly=Path(metadata['source_assembly']).resolve()
    if json.loads((assembly/'motion_manifest.json').read_text())!=metadata['manifest']:
        raise ValueError('Logged controller fallback source assembly/manifest mismatch')
    summary=json.loads(paths['summary.json'].read_text());native=json.loads(paths['native_metadata.json'].read_text())
    if (Path(summary.get('scene','')).resolve()!=Path(metadata['source_scene']).resolve() or
            summary.get('model_sha256')!=metadata['scene_sha256'] or
            native.get('model_sha256')!=metadata['scene_sha256'] or native.get('robot_dofs')!=names):
        raise ValueError('Logged controller fallback source model/native DOF identity mismatch')
    with np.load(paths['states.npz'],allow_pickle=False) as data:
        required={'time','robot_q','poses','body_paths','command','phase'}
        if not required<=set(data.files):raise ValueError('Logged controller fallback trace fields missing')
        stamps=np.asarray(data['time']);matches=np.flatnonzero(np.abs(stamps-metadata['time_s'])<1e-8)
        if len(matches)!=1:raise ValueError('Logged controller fallback checkpoint timestamp is not unique')
        frame=int(matches[0]);q=np.asarray(data['robot_q']);commands=np.asarray(data['command'])
        if q.shape!=(len(stamps),len(names)) or commands.shape!=q.shape or not np.isfinite(commands).all():
            raise ValueError('Logged controller fallback native command shape/finite mismatch')
        if not np.array_equal(q[frame],arrays['robot_q']):raise ValueError('Logged controller fallback measured q frame mismatch')
        source_paths=list(map(str,data['body_paths']));wanted_paths=list(map(str,arrays['body_paths']))
        if len(source_paths)!=len(set(source_paths)) or set(source_paths)!=set(wanted_paths):
            raise ValueError('Logged controller fallback body identity mismatch')
        reorder=[source_paths.index(path) for path in wanted_paths]
        if not np.array_equal(data['poses'][frame,reorder],arrays['body_poses_world']):
            raise ValueError('Logged controller fallback measured body frame mismatch')
        last=metadata.get('sequence',{}).get('cycle_plans',[{}])[-1]
        if last.get('end_frame_index') is not None and last['end_frame_index']!=frame:
            raise ValueError('Logged controller fallback cycle/frame index mismatch')
        position=commands[frame].astype(float).copy();phase=str(data['phase'][frame])
    return dict(position_targets=position,velocity_targets=None,
        provenance=dict(common,source='exact_matched_logged_position_setter_command',controller_continuity_verified=False,
            position_targets_verified=True,velocity_targets_verified=False,
            frame_index=frame,timestamp_s=float(metadata['time_s']),phase=phase,
            source_assembly=str(assembly),source_scene_sha256=metadata['scene_sha256'],
            source_file_sha256={str(path):file_sha256(path) for path in paths.values()},
            position_source='states.command; logged setter command, not a native getter',
            velocity_source='unknown; preserve authored source drive velocity, never measured qvel'))


def rebase_rail_controller_state(state,arrays,rail_name):
    """A coordinate-origin change preserves the same world-space setpoint."""
    result=dict(state);result['provenance']=dict(state['provenance'])
    if state['position_targets'] is not None:
        result['position_targets']=state['position_targets'].copy()
        index=list(map(str,arrays['robot_dof_names'])).index(rail_name)
        result['position_targets'][index]-=arrays['robot_q'][index]
        result['provenance']['rail_position_target_rebase']=dict(name=rail_name,
            source_target_m=float(state['position_targets'][index]),source_measured_q_m=float(arrays['robot_q'][index]),
            current_target_m=float(result['position_targets'][index]),world_station_setpoint_preserved=True)
    return result


def restore_controller_targets(robot,state,ids,*,to_native=lambda a:a,fallback_position=None):
    """Restore proven commands; legacy unknown velocity remains authored."""
    position=state['position_targets'];velocity=state['velocity_targets']
    if position is None:
        if fallback_position is None:raise ValueError('Proven controller position target required')
        position=np.asarray(fallback_position).copy()
    robot.set_dof_position_targets(to_native(position[None].copy()),ids)
    if velocity is not None:robot.set_dof_velocity_targets(to_native(velocity[None].copy()),ids)
    report=dict(state['provenance'],position_targets=position.tolist(),
        velocity_targets=None if velocity is None else velocity.tolist(),
        unknown_position_measured_hold_fallback=state['position_targets'] is None,
        velocity_drive_left_authored=velocity is None)
    for label,value in (('position',position),('velocity',velocity)):
        getter=getattr(robot,'get_dof_'+label+'_targets',None)
        if getter is None or value is None:continue
        native=getter();native=native.numpy() if hasattr(native,'numpy') else np.asarray(native)
        if native.shape!=(1,len(value)) or not np.isfinite(native).all():
            raise ValueError('Restored native controller '+label+' target shape/finite mismatch')
        report[label+'_targets_max_abs_error']=float(np.max(np.abs(native[0]-value)))
        if report[label+'_targets_max_abs_error']>1e-5:
            raise ValueError('Native controller '+label+' target restore did not reproduce the proven setpoint')
    return report
