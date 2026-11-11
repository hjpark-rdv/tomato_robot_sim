"""Keep an existing audited preparation as a camera scan, before entry."""
import copy

import numpy as np
from scipy.optimize import least_squares


def _ready_open_phases(start):
    """Exact retained prefix of existing joint/ready-opening harvest commands."""
    return [(name,duration,np.array([start,start]),gap,0.,0.) for name,duration,gap in
        [('ready_hold',.5,.044),('prepare_open',1.,.096),('prepare_open_hold',.5,.096)]]


def retained_camera_arm_hold(kin,start,resume_context):
    """Retain six native arm setpoints without converting lift bias to height."""
    start=np.asarray(start,float)
    if resume_context is None:return start.copy(),None
    from native_route_checkpoint import load_checkpoint
    from robot_controller_state import resolve_robot_controller_state
    metadata=resume_context['metadata'];checkpoint=resume_context['report']['checkpoint']
    saved,arrays=load_checkpoint(checkpoint,expected_scene_sha256=metadata['scene_sha256'])
    if saved!=metadata:raise ValueError('Camera hold checkpoint metadata mismatch')
    controller=resolve_robot_controller_state(saved,arrays,checkpoint)
    if controller['provenance'].get('position_source')!='native_get_dof_position_targets':
        raise ValueError('Resumed camera arm hold requires proven native getter targets')
    names=list(map(str,arrays['robot_dof_names']));indices=[names.index(name)for name in kin.names]
    if len(indices)!=7 or start.shape!=(7,) or not np.array_equal(start,arrays['robot_q'][indices]):
        raise ValueError('Camera hold must retain the exact measured seven-joint start')
    command=start.copy();command[1:]=np.asarray(controller['position_targets'])[indices[1:]]
    return command,dict(source='Unchanged six-arm native getter targets; actual lift desired plus existing drive compensation',
        checkpoint=checkpoint,checkpoint_metadata_sha256=saved['metadata_sha256'],
        checkpoint_npz_sha256=saved['checkpoint_npz_sha256'],controller=controller['provenance'],
        measured_start_q=start.tolist(),command_start_q=command.tolist(),
        retained_arm_joint_names=list(kin.names[1:]),lift_desired_m=float(start[0]),
        native_lift_drive_target_m=float(controller['position_targets'][indices[0]]),
        lift_drive_target_not_used_as_physical_height=True,physical_state_modified=False)


def verify_retained_camera_arm_hold(kin,plan,checkpoint):
    """Bind this camera prefix's distinct measured state and hold command."""
    from pathlib import Path
    from native_route_checkpoint import load_checkpoint
    if (not plan.get('camera_scan',{}).get('direct_prefix') or checkpoint is None or
            plan.get('retained_camera_arm_hold') is None):
        raise ValueError('Retained camera hold requires an exact checkpoint and direct camera prefix')
    checkpoint=str(Path(checkpoint).resolve());metadata,arrays=load_checkpoint(checkpoint)
    command,proof=retained_camera_arm_hold(kin,plan['start'],
        dict(metadata=metadata,arrays=arrays,report=dict(checkpoint=checkpoint)))
    if (plan['retained_camera_arm_hold']!=proof or
            not np.array_equal(plan.get('command_start_q'),command) or
            not np.array_equal(plan['phases'][0]['waypoint_q'][0],command)):
        raise ValueError('Camera hold command/provenance differs from exact native getter targets')
    return proof


def direct_scan_prefix(kin,start,*,ready_lift_offset_m=0.,preparation_gap_m=None,
                       scan_lift_height_m=None,
                       arm_hold_command_q=None,arm_hold_provenance=None,
                       preparation='horizontal_ready',horizontal_ready_connector='joint',
                       preparation_opening='ready',harvest_profile='belt_open_close_cycle'):
    """Existing stationary-open/ready-lift commands, without unused harvest IK.

    This camera-only prefix does not propose or validate any insertion pose.
    Existing rail attachment, camera pan, whole-path audits and native execution
    remain the caller's responsibility. It never changes an initial body pose.
    """
    from motion_kinematics import BELT_PROFILES,ready_lift_branch
    if (preparation!='horizontal_ready' or horizontal_ready_connector!='joint' or
            preparation_opening!='ready' or harvest_profile not in BELT_PROFILES):
        raise ValueError('Direct camera scan requires existing horizontal-ready joint/ready-opening belt mode')
    start=np.asarray(start,float)
    if start.shape!=(7,) or not np.isfinite(start).all():raise ValueError('Finite actual start[7] required')
    measured_start=start.copy()
    if arm_hold_command_q is not None:
        command=np.asarray(arm_hold_command_q,float)
        if command.shape!=(7,) or not np.isfinite(command).all() or command[0]!=start[0]:
            raise ValueError('Camera arm hold must preserve actual lift desired; native lift bias is not a height')
        if not isinstance(arm_hold_provenance,dict) or arm_hold_provenance.get('physical_state_modified')is not False:
            raise ValueError('Explicit measured/retained camera arm command provenance required')
        start=command.copy()
    if (isinstance(ready_lift_offset_m,(bool,np.bool_)) or not np.isscalar(ready_lift_offset_m) or
            not np.isfinite(ready_lift_offset_m) or abs(ready_lift_offset_m)>.4):
        raise ValueError('Finite existing ready-lift offset within400mm required')
    if preparation_gap_m is not None and (not np.isscalar(preparation_gap_m) or
            not np.isfinite(preparation_gap_m) or not .028<=preparation_gap_m<=.096):
        raise ValueError('Existing requested preparation gap must be28..96mm')
    if scan_lift_height_m is not None and ready_lift_offset_m:
        raise ValueError('Absolute camera scan height and TCP-holding ready-lift redistribution are distinct modes')
    phases=_ready_open_phases(start);lift=None
    if ready_lift_offset_m:
        waypoints,lift=ready_lift_branch(kin,start,ready_lift_offset_m)
        phases.append(('prepare_lift_branch',4.,waypoints,.096,0.,0.))
    scan_lift=None
    if scan_lift_height_m is not None:
        from camera_lift_vantage import lift_scan_transfer
        transfer,scan_lift=lift_scan_transfer(kin,start,scan_lift_height_m)
        phases.extend(transfer)
    if hasattr(kin,'bounds'):
        bounds=np.asarray(kin.bounds,float)
        if bounds.shape!=(2,7) or not np.isfinite(bounds).all() or np.any(bounds[0]>bounds[1]):
            raise ValueError('Finite existing seven-joint limits required')
        for name,_,qs,_,_,_ in phases:
            if np.any(qs<bounds[0]-1e-8) or np.any(qs>bounds[1]+1e-8):
                raise RuntimeError('Planned joint limits exceeded in '+name)
    metadata=dict(start=measured_start.tolist(),preparation=preparation,
        horizontal_ready_connector=horizontal_ready_connector,
        preparation_gap_m=preparation_gap_m,ready_lift_branch=lift,
        preparation_opening=dict(location='ready',requested_transfer_gap_m=preparation_gap_m,
            effective_transfer_gap_m=.096,feedback_before_phases=['prepare_lift_branch','camera_pan'],
            required_gap_m=.096,tracking_tolerance_m=.0005),
        phases=[dict(name=n,duration_s=d,waypoint_q=qs.tolist(),end_gap_m=g,
            belt_local_y_m_s=[left,right]) for n,d,qs,g,left,right in phases],
        camera_scan=dict(purpose='map_based_preparation_only',direct_prefix=True,
            harvest_entry_executed=False,harvest_region_selected=False,
            harvest_staging_ik_evaluated=False,harvest_approach_ik_evaluated=False,
            suffix='Actual camera observation then existing full harvest IK/collision/native checks required'),
        scope='Existing ready/open/optional ready-lift commands only; no harvest reachability claim')
    if arm_hold_command_q is not None:
        metadata.update(command_start_q=start.tolist(),retained_camera_arm_hold=copy.deepcopy(arm_hold_provenance))
    if scan_lift is not None:
        metadata['camera_scan']['physical_lift']=scan_lift
        metadata['preparation_opening']['feedback_before_phases'].insert(0,'camera_lift_vantage')
    return phases,metadata


def camera_cycle_budget(*,close_seconds=6.,belt_seconds=2.,belt_ramp_seconds=.5,
                        ready_lift_offset_m=0.,entry_duration_scale=1.,rail_duration_s=0.):
    """Independent finite loop budget, not a trajectory or IK feasibility proof.

    The existing direct connector supplies both six-second joint legs, and
    retime_entry supplies entry/withdraw scaling. Remaining durations match the
    horizontal-ready belt cycle and are checked against plan_motion in tests.
    """
    from motion_kinematics import preparation_phases,belt_speed_fraction
    from motion_timing import retime_entry
    values=np.asarray([close_seconds,belt_seconds,belt_ramp_seconds,ready_lift_offset_m,rail_duration_s],float)
    if (not np.isfinite(values).all() or close_seconds<=0 or belt_seconds<=0 or
            abs(ready_lift_offset_m)>.4 or rail_duration_s<0):
        raise ValueError('Finite positive belt/close and bounded ready-lift/rail budget required')
    belt_speed_fraction(0.,belt_seconds,belt_ramp_seconds)
    q=np.zeros(7)
    ready=sum(row[1] for row in _ready_open_phases(q))
    connector=sum(row[1] for row in preparation_phases(q,q,'direct'))
    entries=[(name,4.,np.array([q,q]),.096,0.,0.) for name in ('approach','withdraw')]
    entry_meta=dict(phases=[dict(name=row[0],duration_s=row[1]) for row in entries])
    entries,_=retime_entry(entries,entry_meta,entry_duration_scale)
    components=dict(ready_open=ready,preparation_joint=connector,
        approach_and_withdraw=sum(row[1] for row in entries),
        belt_cycle=max(1.,float(belt_ramp_seconds))+2*float(close_seconds)+float(belt_seconds)+3.,
        return_joint=connector,ready_close_and_hold=2.,
        ready_lift_and_return=8. if ready_lift_offset_m else 0.,rail=float(rail_duration_s))
    duration=float(sum(components.values()))
    return duration,dict(schema='s300-camera-finite-cycle-time-budget-v1',duration_s=duration,
        components_s=components,source='Existing joint preparation and retime_entry; horizontal-ready belt timing',
        budget_only=True,harvest_staging_ik_evaluated=False,harvest_approach_ik_evaluated=False,
        scan_duration_is_not_harvest_cycle_duration=True,physical_commands_changed=False)


def scan_prefix(phases,metadata):
    indices=[i for i,p in enumerate(phases) if p[0]=='approach']
    if len(indices)!=1 or indices[0]<1:raise ValueError('One existing approach with a preparation prefix is required')
    count=indices[0]
    # The camera is mounted on base-yaw link1. Arm staging can rotate its
    # view away from the crop: expose before that joint branch, after lift.
    turns=[i for i,p in enumerate(phases[:count]) if p[0]=='prepare_joint_branch']
    if turns:count=min(count,turns[0])
    if count<1:raise ValueError('Camera scan requires a nonempty safe preparation prefix')
    result=copy.deepcopy(metadata)
    selected=phases[:count]
    result['phases']=result['phases'][:count]
    result['camera_scan']=dict(purpose='map_based_preparation_only',
        harvest_entry_executed=False,harvest_region_selected=False,
        suffix='New camera-visible selection and existing IK/collision planning required after actual arrival',
        removed_phase_names=[p[0] for p in phases[count:]])
    return selected,result


def append_scan_pan(phases,metadata,kin,scan_target_world_m,body_from_optical,*,
                    view='target-point',rail_axis_world=(0.,1.,0.),
                    body_path='/World/HarvestRobot/Robot/link1',row_downward_pitch_deg=0.):
    """Plan physical base-yaw aiming at a scan point or normal to the row.

    Only the existing base joint moves. The fixed camera attachment is read
    through FK; this helper never edits a camera, robot body or initial state.
    The caller must audit and execute the added joint path before observing.
    ``kin`` must already represent the intended physical rail destination.
    """
    if not phases or len(metadata.get('phases',[]))!=len(phases):
        raise ValueError('Aligned nonempty scan phases/metadata required')
    target=np.asarray(scan_target_world_m,float);mount=np.asarray(body_from_optical,float)
    if view not in ('target-point','row-front'):
        raise ValueError('Camera scan view must be target-point or row-front')
    if not np.isfinite(row_downward_pitch_deg) or not 0<=row_downward_pitch_deg<=30:
        raise ValueError('Explicit row camera downward pitch must be within 0..30deg')
    if view!='row-front' and row_downward_pitch_deg:
        raise ValueError('Downward row pitch requires row-front view')
    axis=np.asarray(rail_axis_world,float)
    if axis.shape!=(3,) or not np.isfinite(axis).all() or np.linalg.norm(axis[:2])<1e-9 or abs(axis[2])>1e-8:
        raise ValueError('Finite horizontal physical rail axis required')
    axis=axis[:2]/np.linalg.norm(axis[:2])
    if target.shape!=(3,) or not np.isfinite(target).all():
        raise ValueError('Finite map-based scan target XYZ required')
    if (mount.shape!=(4,4) or not np.isfinite(mount).all() or
            not np.allclose(mount[3],[0,0,0,1],rtol=0,atol=1e-9) or
            not np.allclose(mount[:3,:3].T@mount[:3,:3],np.eye(3),rtol=0,atol=1e-7) or
            np.linalg.det(mount[:3,:3])<.999999):
        raise ValueError('Finite fixed rigid body-from-optical mount required')
    if 'base' not in kin.names:raise ValueError('Existing base yaw joint required')
    index=kin.names.index('base');bounds=np.asarray(kin.bounds,float)
    start=np.asarray(phases[-1][2],float)[-1].copy();gap=float(phases[-1][3])
    if (start.shape!=(len(kin.names),) or bounds.shape!=(2,len(start)) or
            not np.isfinite(np.r_[start,bounds.ravel()]).all() or
            np.any(start<bounds[0]-1e-8) or np.any(start>bounds[1]+1e-8) or
            bounds[0,index]>=bounds[1,index]):
        raise ValueError('Finite bounded scan terminal joint configuration required')
    if not np.isfinite(gap) or abs(gap-.096)>.0005:
        raise ValueError('Camera pan requires the preceding stationary open96mm scan gap')
    body=body_path
    # Choose the crop-facing normal once. Do not aim obliquely along the row
    # or flip sides as the mounted sensor moves around the base joint.
    initial_frames=kin.frames(start)
    if body not in initial_frames:raise ValueError('FK lacks the mounted camera body: '+body)
    origin=np.asarray(initial_frames[body],float)@mount
    normal=np.array([-axis[1],axis[0]])
    side=float(np.dot(target[:2]-origin[:2,3],normal))
    if view=='row-front' and abs(side)<1e-8:
        raise ValueError('Scan target must identify one side of the rail')
    normal*=1. if side>=0 else -1.
    pitch=np.deg2rad(row_downward_pitch_deg)
    row_forward=np.r_[normal*np.cos(pitch),-np.sin(pitch)]
    def geometry(yaw):
        q=start.copy();q[index]=float(yaw)
        frames=kin.frames(q)
        if body not in frames:raise ValueError('FK lacks the mounted camera body: '+body)
        camera=np.asarray(frames[body],float)@mount
        forward=camera[:3,2] if view=='row-front' else camera[:2,2]
        inward=row_forward if view=='row-front' else target[:2]-camera[:2,3]
        if not np.isfinite(camera).all() or min(np.linalg.norm(forward),np.linalg.norm(inward))<1e-9:
            raise ValueError('Finite nondegenerate horizontal camera/scan direction required')
        forward=forward/np.linalg.norm(forward);inward=inward/np.linalg.norm(inward)
        error=(float(np.arctan2(np.linalg.norm(np.cross(forward,inward)),np.dot(forward,inward)))
            if view=='row-front' else
            float(np.arctan2(forward[0]*inward[1]-forward[1]*inward[0],np.dot(forward,inward))))
        return q,camera,forward,inward,error
    _,before,_,_,initial_error=geometry(start[index])
    lower,upper=float(bounds[0,index]),float(bounds[1,index])
    seeds=[float(np.clip(start[index]+initial_error+turn*2*np.pi,lower,upper)) for turn in range(-2,3)]
    seeds.append(float(np.clip(start[index],lower,upper)))
    if view=='row-front':
        # A side-mounted arm's base axis is horizontal. Horizontal heading
        # alone cannot distinguish a forward camera from one aimed skyward.
        # Search the bounded existing joint and require the full optical axis.
        seeds.extend(np.linspace(lower+1e-8,upper-1e-8,9).tolist())
    choices=[]
    for seed in sorted(set(seeds)):
        def residual(yaw):
            _,_,forward,inward,_=geometry(yaw[0]);return forward-inward
        solved=least_squares(residual,[seed],bounds=([lower],[upper]),
                             ftol=1e-12,xtol=1e-12,gtol=1e-12,max_nfev=100)
        q,camera,_,_,error=geometry(solved.x[0])
        tolerance=1e-3 if view=='row-front' else 1e-5
        if abs(error)<=tolerance:
            choices.append((abs(q[index]-start[index]),abs(error),q,camera,error))
    if not choices:raise RuntimeError('Finite bounded camera-pan yaw solution not found; target remains a scan proposal')
    _,_,goal,after,final_error=min(choices,key=lambda value:value[:2])
    count=max(2,int(np.ceil(abs(goal[index]-start[index])/np.deg2rad(3)))+1)
    waypoints=np.tile(start,(count,1));waypoints[:,index]=np.linspace(start[index],goal[index],count)
    selected=list(phases);result=copy.deepcopy(metadata)
    pan_duration=4.
    if view=='row-front':
        pan_duration=float(np.clip(4.*abs(goal[index]-start[index])/np.deg2rad(30.),.5,4.))
    for name,duration,qs in [('camera_pan',pan_duration,waypoints),('camera_pan_hold',.5,np.array([goal,goal]))]:
        selected.append((name,duration,qs,gap,0.,0.))
        row=dict(name=name,duration_s=duration,waypoint_q=qs.tolist(),end_gap_m=gap,belt_local_y_m_s=[0.,0.])
        for field in ('rail_waypoint_positions_m','rail_native_command_positions_m'):
            if field in metadata['phases'][-1]:
                values=np.asarray(metadata['phases'][-1][field],float)
                if values.ndim!=1 or not len(values) or not np.isfinite(values).all():
                    raise ValueError('Finite existing rail-hold command required during camera pan')
                row[field]=[float(values[-1])]*len(qs)
        result['phases'].append(row)
    result.setdefault('camera_scan',{})['physical_pan']=dict(
        view=view,rail_axis_world=list(rail_axis_world),
        row_normal_world=[*normal.tolist(),0.],
        row_downward_pitch_deg=float(row_downward_pitch_deg),
        desired_optical_forward_world=row_forward.tolist() if view=='row-front' else None,
        longitudinal_target_offset_m=float(np.dot(target[:2]-after[:2,3],axis)),
        map_scan_target_world_m=target.tolist(),map_used_for_scan_aim_only=True,
        harvest_region_selected=False,harvest_entry_executed=False,
        joint_name='base',joint_index=index,start_q=start.tolist(),goal_q=goal.tolist(),
        commanded_yaw_change_rad=float(goal[index]-start[index]),
        camera_world_before=before.tolist(),camera_world_after=after.tolist(),
        optical_forward_world_after=after[:3,2].tolist(),
        horizontal_heading_error_before_rad=initial_error,horizontal_heading_error_after_rad=final_error,
        duration_s=pan_duration,hold_s=.5,gap_m=gap,belt_local_y_m_s=[0.,0.],
        optical_axis_goal_tolerance_rad=1e-3 if view=='row-front' else 1e-5,
        camera_transform_edited=False,body_state_teleported=False,
        collision_certified=False,actual_arrival_and_new_camera_observation_required=True)
    return selected,result
