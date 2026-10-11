"""Exact restore precedence and measured rail origins for native continuation.

An archived window-start checkpoint remains provenance. Once that window has
produced a later valid checkpoint, only the latter supplies measured state.
Rail geometry uses measured world displacement; a drive setpoint is recorded
separately and never substituted for measured state.
"""
import copy

import numpy as np


def route_restore_selection(manifest, resume_context, *, num_envs=1, replicate=False):
    archive = manifest.get('route_checkpoint')
    if archive is None:
        return dict(prepare_route_start=False, superseded_archive=None)
    if num_envs != 1 or replicate:
        raise ValueError('Active-window native route restore requires one unreplicated world')
    if resume_context is None:
        return dict(prepare_route_start=True, superseded_archive=None)
    metadata = resume_context['metadata']
    if (metadata['source_plant'] != manifest['plant']['source_plant'] or
            metadata['manifest'].get('route_checkpoint') != archive):
        raise ValueError('Later same-plant checkpoint does not preserve the exact archived route origin')
    return dict(prepare_route_start=False, superseded_archive=dict(
        schema='s300-later-native-checkpoint-restore-precedence-v1',
        archived_route_checkpoint=copy.deepcopy(archive),
        selected_sameplant_checkpoint=copy.deepcopy(resume_context['report']),
        selected_checkpoint_metadata_sha256=metadata['metadata_sha256'],
        archived_restore_executed=False, selected_restore_executed='pending native state verification',
        timestamp_scope='Checkpoint timestamps belong to their own native engine instances; numeric ordering across instances is not inferred',
        physical_history_reset=False, bitwise_engine_continuity=False))


def measured_resume_rail_origin(manifest, resume_context):
    """Return measured same-asset rail displacement plus its proven hold target."""
    rail = manifest.get('rail')
    if resume_context is None or rail is None:
        return None
    metadata, state = resume_context['metadata'], resume_context['arrays']
    if metadata['manifest'].get('rail') != rail:
        raise ValueError('Same-plant native resume changed the physical rail definition')
    axis = np.asarray(rail['axis_world'], dtype=float)
    if axis.shape != (3,) or not np.array_equal(axis, [0., 1., 0.]):
        raise ValueError('Measured rail continuation requires the actual world-Y rail axis')
    paths = list(map(str, state['body_paths']))
    base = rail['base_body']
    if len(paths) != len(set(paths)) or base not in paths:
        raise ValueError('Complete unique measured rail base identity required')
    poses = np.asarray(state['body_poses_world'], dtype=float)
    source_position = np.asarray(rail['source_base_position_world_m'], dtype=float)
    names = list(map(str, state['robot_dof_names']))
    q = np.asarray(state['robot_q'], dtype=float)
    if (poses.shape != (len(paths), 7) or source_position.shape != (3,) or
            len(names) != len(set(names)) or rail['dof_name'] not in names or
            q.shape != (len(names),) or not np.isfinite(np.r_[poses.ravel(), source_position, q]).all()):
        raise ValueError('Finite complete measured rail/body native state required')
    measured_position = poses[paths.index(base), :3]
    displacement = float(measured_position[1]-source_position[1])
    station = float(rail['initial_station_y_m']+displacement)
    limits = np.asarray(rail['limits_relative_m'], dtype=float)
    if limits.shape != (2,) or not np.isfinite(limits).all() or not limits[0]-1e-6 <= displacement <= limits[1]+1e-6:
        raise ValueError('Measured rail origin exceeds the unchanged physical joint limits')
    targets = resume_context.get('controller_state', {}).get('position_targets')
    if targets is None:
        raise ValueError('Rail resume requires proven prior native position targets; measured q is not a hold setpoint')
    targets = np.asarray(targets, dtype=float)
    if targets.shape != q.shape or not np.isfinite(targets).all():
        raise ValueError('Finite exact native controller target order required')
    index = names.index(rail['dof_name'])
    return dict(schema='s300-measured-resume-rail-origin-v1', dof_name=rail['dof_name'],
        axis_world=axis.tolist(), displacement_m=displacement, station_y_m=station,
        measured_base_world_m=measured_position.tolist(), source_base_world_m=source_position.tolist(),
        nonrail_base_delta_m=(measured_position-source_position-axis*displacement).tolist(),
        native_measured_rail_q_m=float(q[index]), native_position_target_m=float(targets[index]),
        measured_world_minus_native_q_m=float(displacement-q[index]),
        checkpoint_metadata_sha256=metadata['metadata_sha256'],
        controller_provenance=copy.deepcopy(resume_context['controller_state']['provenance']),
        planning_geometry_origin='measured base world displacement',
        native_rail_command='unchanged proven prior drive target; no travel phase',
        controller_target_is_not_measured_displacement=True, physical_history_reset=False)


def attach_resume_rail_hold(phases, metadata, origin):
    """Stamp measured checker offsets and prior drive commands independently."""
    result = copy.deepcopy(metadata)
    if len(result['phases']) != len(phases):
        raise ValueError('Continuation phase metadata does not match the actual motion phases')
    for phase, row in zip(phases, result['phases']):
        if row['name'] != phase[0] or 'rail_waypoint_positions_m' in row:
            raise ValueError('Rail resume hold cannot replace an existing travel command')
        count = len(row['waypoint_q'])
        if count != len(phase[2]) or count < 2:
            raise ValueError('Complete continuation joint waypoint arrays required')
        row['rail_waypoint_positions_m'] = [origin['displacement_m']]*count
        row['rail_native_command_positions_m'] = [origin['native_position_target_m']]*count
    result['resume_rail_origin'] = copy.deepcopy(origin)
    result['rail_motion_scope'] = 'Measured existing station and preserved native drive target; no added rail motion phase'
    return phases, result


def initial_plan_rail_position(plan):
    """A continued plan starts at its recorded measured station, never zero."""
    origin = plan.get('resume_rail_origin')
    if origin is None:
        # Existing travel metadata remains at the destination on subsequent
        # harvest cycles, which intentionally omit the already executed move.
        destination = plan.get('rail_destination')
        if destination is not None and plan.get('phases') and plan['phases'][0]['name'] != 'rail_move':
            value = destination.get('displacement_m')
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not np.isfinite(value):
                raise ValueError('Malformed physical rail destination')
            return float(value)
        return 0.
    value = origin.get('displacement_m')
    if (origin.get('schema') != 's300-measured-resume-rail-origin-v1' or
            origin.get('axis_world') != [0., 1., 0.] or
            not isinstance(value, (int, float)) or isinstance(value, bool) or not np.isfinite(value)):
        raise ValueError('Malformed measured native continuation rail origin')
    travel = plan.get('resume_rail_travel')
    if travel is not None:
        destination = plan.get('rail_destination', {})
        end = destination.get('displacement_m')
        move_first = travel.get('move_first')
        if (travel.get('schema') != 's300-measured-resume-rail-travel-v1' or
                type(move_first) is not bool or not isinstance(end, (int, float)) or
                isinstance(end, bool) or not np.isfinite(end) or
                travel.get('saved_measured_start_displacement_m') != value):
            raise ValueError('Malformed explicit resumed physical rail travel')
        if not move_first:
            return float(end)
    return float(value)


def resumed_rail_destination(manifest, station_y_m, speed_m_s, origin):
    """Bound both measured path and prior drive-command changes at the same speed."""
    from rail_motion import rail_destination
    destination = rail_destination(manifest, station_y_m, speed_m_s)
    distances = [abs(destination['displacement_m']-origin[key])
                 for key in ('displacement_m','native_position_target_m')]
    destination['duration_s'] = max(1., 1.5*max(distances)/speed_m_s)
    destination['saved_measured_start_displacement_m'] = origin['displacement_m']
    destination['saved_native_start_position_target_m'] = origin['native_position_target_m']
    destination['measured_start_station_y_m'] = origin['station_y_m']
    destination['source_checkpoint_metadata_sha256'] = origin['checkpoint_metadata_sha256']
    return destination


def attach_resumed_rail_travel(phases, metadata, start_q, destination, origin, *, move_first):
    """Existing physical rail phase from measured state; no plant/body transform writes."""
    from rail_motion import attach_rail_commands
    phases, result = attach_rail_commands(phases,metadata,start_q,destination,move_first=move_first)
    for row in result['phases']:
        count = len(row['waypoint_q'])
        moving = move_first and row['name'] == 'rail_move'
        row['rail_waypoint_positions_m'] = (np.linspace(origin['displacement_m'],
            destination['displacement_m'],count).tolist() if moving else [destination['displacement_m']]*count)
        row['rail_native_command_positions_m'] = (np.linspace(origin['native_position_target_m'],
            destination['displacement_m'],count).tolist() if moving else [destination['displacement_m']]*count)
    result['resume_rail_origin'] = copy.deepcopy(origin)
    result['resume_rail_travel'] = dict(schema='s300-measured-resume-rail-travel-v1',
        move_first=bool(move_first),saved_measured_start_displacement_m=origin['displacement_m'],
        saved_native_start_position_target_m=origin['native_position_target_m'],
        destination_displacement_m=destination['displacement_m'],
        physical_history_reset=False, native_drive_and_measured_state_separate=True,
        native_validation='pending; nominal collision audit does not prove physical arrival')
    result['rail_motion_scope']='Explicit physical prismatic travel from validated measured checkpoint; active plant/native break inventory retained'
    return phases,result
