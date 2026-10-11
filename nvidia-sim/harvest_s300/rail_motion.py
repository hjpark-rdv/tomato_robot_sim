"""Commands for the authored physical S300 rail joint; no transform writes."""
import copy
import numpy as np


def rail_destination(manifest, station_y_m, speed_m_s=.08):
    rail=manifest.get('rail')
    if not rail:raise ValueError('Rail motion requires a derived physical rail assembly')
    values=np.asarray([station_y_m,speed_m_s],float)
    if not np.isfinite(values).all() or not 0<speed_m_s<=.2:
        raise ValueError('Finite station and rail speed within (0,.2] m/s required')
    displacement=float(station_y_m-rail['initial_station_y_m'])
    low,high=rail['limits_relative_m']
    if not low<=displacement<=high:raise ValueError('Station exceeds authored physical rail limits')
    # Smoothstep peaks at 1.5 times average velocity.
    return dict(station_y_m=float(station_y_m),displacement_m=displacement,
                nominal_speed_limit_m_s=float(speed_m_s),
                duration_s=max(1.,1.5*abs(displacement)/speed_m_s),
                dof_name=rail['dof_name'],axis_world=rail['axis_world'])


def station_kinematics(kin, destination):
    result=copy.deepcopy(kin)
    result.world[:3,3]+=np.asarray(destination['axis_world'])*destination['displacement_m']
    return result


def attach_rail_commands(phases, metadata, start_q, destination, *, move_first):
    phases=list(phases)
    metadata=copy.deepcopy(metadata)
    if move_first:
        phase=('rail_move',destination['duration_s'],np.array([start_q,start_q]),.044,0.,0.)
        phases.insert(0,phase)
        metadata['phases'].insert(0,dict(name=phase[0],duration_s=phase[1],waypoint_q=phase[2].tolist(),
            end_gap_m=.044,belt_local_y_m_s=[0.,0.]))
        if metadata.get('belt_cycle'):
            metadata['belt_cycle']['start_after_settle_s']+=destination['duration_s']
    for row in metadata['phases']:
        count=len(row['waypoint_q'])
        row['rail_waypoint_positions_m']=(np.linspace(0.,destination['displacement_m'],count).tolist()
            if move_first and row['name']=='rail_move' else [destination['displacement_m']]*count)
    metadata['rail_destination']=destination
    metadata['rail_motion_scope']='Physical prismatic joint actuation, measured base tracking; no world-transform teleport'
    return phases,metadata


def rail_command(row, smooth_fraction):
    # Continued motion keeps its measured geometric origin distinct from the
    # prior drive setpoint. Legacy/new-travel phases retain the original key.
    values=np.asarray(row.get('rail_native_command_positions_m',row['rail_waypoint_positions_m']),float)
    if len(values)<2 or not np.isfinite(values).all():raise ValueError('Invalid rail command waypoints')
    f=float(np.clip(smooth_fraction,0.,1.))*(len(values)-1);lo=min(int(f),len(values)-2)
    return float(values[lo]*(1-(f-lo))+values[lo+1]*(f-lo))
