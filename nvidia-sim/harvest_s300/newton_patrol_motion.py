"""Distance-based timing for the geometric camera patrol's existing paths.

The original IK waypoints and physical drive limits are retained. These are
simulation command-speed limits, not certified real RB5 operating limits.
"""
import copy
import numpy as np


VELOCITY = np.array([.12, .12, .8, .8, .8, 1., 1., 1., .06, .06])
ACCELERATION = np.array([.3, .3, 1.6, 1.6, 1.6, 2., 2., 2., .3, .3])


def motion_duration(waypoints, *, minimum=.08):
    points=np.asarray(waypoints,float)
    if points.ndim!=2 or points.shape[1]!=10 or len(points)<2 or not np.isfinite(points).all():
        raise ValueError('Finite ten-axis path required')
    # The existing sampler smoothsteps the path parameter, then linearly
    # interpolates the uniformly spaced IK waypoints. Bound its peak speed
    # by the largest interval derivative, not merely endpoint displacement.
    derivative=np.max(np.abs(np.diff(points,axis=0)),axis=0)*(len(points)-1)
    return float(max(minimum,np.max(1.5*derivative/VELOCITY),
                     np.sqrt(np.max(6.*derivative/ACCELERATION))))


def fluid_plan(plan):
    """Retain geometric entry/return geometry; eliminate repeated jaw cycling."""
    if plan.get('harvest_mode')!='main_stem_only_geometric_harvest':
        raise ValueError('Fluid timing is restricted to the geometric harvest policy')
    result=copy.deepcopy(plan);rows=[];timing=[]
    previous=np.asarray(result['initial_command'],float)
    for row in result['phases']:
        old=float(row['duration_s']);name=row['name']
        # No reason to close at ready and reopen at the next target.
        if name in ('ready_hold','ready_close'):continue
        qs=np.asarray(row['waypoint_q'],float)
        rail=np.asarray(row.get('rail_native_command_positions_m',
            row.get('rail_waypoint_positions_m',[previous[0]]*len(qs))),float)
        gap=.096
        jaws=np.column_stack((np.linspace(previous[8],-(gap-.044)/2,len(qs)),
                              np.linspace(previous[9],(gap-.044)/2,len(qs))))
        points=np.column_stack((rail,qs,jaws))
        row['end_gap_m']=gap
        duration=motion_duration(points)
        if name=='prepare_open_hold':duration=.10
        elif name=='geometric_capture_hold':duration=.15
        elif name=='hold':duration=.10
        row['duration_s']=duration
        row['belt_local_y_m_s']=[0.,0.]
        timing.append(dict(phase=name,previous_s=old,duration_s=duration))
        rows.append(row);previous=points[-1]
    result.update(phases=rows,settle_seconds=1/220.,
        total_duration_s=1/220.+sum(row['duration_s'] for row in rows),
        motion_timing=dict(profile='fluid',original_duration_s=plan['total_duration_s'],
            maximum_axis_velocity=VELOCITY.tolist(),parameter_acceleration_limits=ACCELERATION.tolist(),
            phases=timing,keeps_gripper_open_between_harvests=True,
            unchanged_ik_waypoints=True,physical_drive_limits_changed=False))
    return result


def rail_duration(distance,start_speed,end_speed):
    """Choose a monotone cubic under the same velocity/acceleration bounds."""
    if distance<=0 or min(start_speed,end_speed)<0:
        raise ValueError('Forward cruise requires positive distance/nonnegative velocity')
    for duration in np.linspace(distance/VELOCITY[0],max(3*distance/VELOCITY[0],
                                                       2*np.sqrt(6*distance/ACCELERATION[0]),.2),512):
        # p(u)=a*u^3+b*u^2+c*u, derivatives have analytic extrema.
        a=-2*distance+duration*(start_speed+end_speed)
        b=3*distance-duration*(2*start_speed+end_speed)
        c=duration*start_speed
        extrema=[0.,1.]
        if abs(a)>1e-12 and 0<-b/(3*a)<1:extrema.append(-b/(3*a))
        velocity=np.array([(3*a*u*u+2*b*u+c)/duration for u in extrema])
        acceleration=np.array([2*b,6*a+2*b])/duration**2
        if velocity.min()>=-1e-8 and velocity.max()<=VELOCITY[0]+1e-5 and np.abs(acceleration).max()<=ACCELERATION[0]:
            return float(duration)
    raise ValueError('No monotone rail cruise fits the command speed/acceleration limits')


def fluid_scan(plan, *, rail_velocity=0., cruise=False, cruise_speed=None):
    result=copy.deepcopy(plan);row=result['phases'][0]
    previous=np.asarray(result['initial_command'],float)
    points=np.column_stack((row['rail_waypoint_positions_m'],row['waypoint_q'],
                            np.tile(previous[8:],(2,1))))
    row['duration_s']=motion_duration(points,minimum=.10)
    distance=float(points[-1,0]-points[0,0])
    if distance>1e-6:
        start_speed=float(np.clip(rail_velocity,0.,VELOCITY[0]))
        end_speed=float((VELOCITY[0] if cruise_speed is None else cruise_speed) if cruise else 0.)
        if not np.isfinite(end_speed) or not 0<=end_speed<=VELOCITY[0]:
            raise ValueError('Cruise speed must be within the rail velocity limit')
        if start_speed>1e-4 or cruise:
            row['duration_s']=rail_duration(distance,start_speed,end_speed)
            row['rail_boundary_velocities_m_s']=[start_speed,end_speed]
    rows=[row]
    if distance>1e-6 and not cruise:
        stop=copy.deepcopy(row);stop.pop('rail_boundary_velocities_m_s',None)
        stop.update(name='rail_brake_settle',duration_s=.10,
                    waypoint_q=[row['waypoint_q'][-1]]*2,
                    rail_waypoint_positions_m=[row['rail_waypoint_positions_m'][-1]]*2)
        rows.append(stop)
    result.update(phases=rows,settle_seconds=1/220.,total_duration_s=1/220.+sum(r['duration_s'] for r in rows),
                  motion_timing=dict(profile='fluid',fixed_station_hold_removed=True,cruise=cruise))
    return result


def cruise_route(command,end,*,step_m=.15,rail_velocity=0.):
    """A bounded preview route, interrupted by real moving-camera decisions.

    Only the existing executor advances it. The full route is never executed
    without the caller's observation callback at intervals no greater than20cm.
    """
    from run_newton_camera_patrol import scan_plan
    if not np.isfinite(step_m) or not 0<step_m<=.2:
        raise ValueError('Forward camera route requires 0 < step <= .2m')
    current=np.asarray(command,float).copy();rows=[]
    while end-current[0]>1e-6:
        target=min(end,current[0]+step_m)
        cruise=target<end-1e-6
        # A measured start station rarely divides the remaining route exactly.
        # Limit the preceding boundary speed by the remaining braking distance
        # instead of asking a full-speed final millimetre segment to stop. Also
        # bound acceleration for observation steps shorter than the usual15cm.
        # v² <= a*d keeps margin relative to the constant-acceleration2*a*d
        # bound; rail_duration still audits the cubic's exact extrema.
        start_speed=float(np.clip(rail_velocity,0.,VELOCITY[0]))
        next_speed=float(min(VELOCITY[0],
            np.sqrt(start_speed**2+ACCELERATION[0]*(target-current[0])),
            np.sqrt(ACCELERATION[0]*max(0.,end-target)))) if cruise else 0.
        part=fluid_scan(scan_plan(current,destination=target),rail_velocity=rail_velocity,
                        cruise=cruise,cruise_speed=next_speed)
        rows.extend(part['phases'])
        current[0]=target;rail_velocity=next_speed
    if not rows:raise ValueError('Empty forward camera route')
    return dict(schema='s300-camera-patrol-online-cruise-v1',initial_command=np.asarray(command).tolist(),
        initial_gap_m=float(.044+command[9]-command[8]),settle_seconds=1/220.,phases=rows,
        total_duration_s=1/220.+sum(r['duration_s'] for r in rows),
        rolling_camera_required=True,observation_step_m=step_m,
        collision_certified=False,training_eligible=False,harvest_success=None)
