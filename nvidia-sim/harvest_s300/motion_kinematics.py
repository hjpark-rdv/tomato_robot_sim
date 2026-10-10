"""Tool-frame adapter for the existing URDF FK/least-squares IK approach.

Unlike the old ring frame, this endpoint is the actual V5.1 CAD origin. Joint
frames/limits come from the composed USD. No physical body is teleported by FK.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from pxr import UsdGeom, UsdPhysics
from motion_assets import ROBOT, TOOL


def transmission_transforms(gap):
    """CAD shape_at linkage equations, driven by measured slider gap (metres)."""
    if not .0279<=gap<=.0961:raise ValueError('Measured gap exceeds CAD mechanism travel')
    def theta(g):
        s=g/2+.018
        return np.arccos(np.clip((s*s+.025**2-.045**2)/(2*s*.025),-1,1))
    angle=theta(gap);nominal=theta(.044);pivot=np.array([0.,-.031,-.018])
    out={};t=np.eye(4);t[:3,:3]=R.from_rotvec([0,-(angle-nominal),0]).as_matrix()
    t[:3,3]=pivot-t[:3,:3]@pivot;out['crank']=t
    for side,sign in [('linkL',-1),('linkR',1)]:
        def ends(g,a):
            return np.array([sign*.025*np.cos(a),-.031,-.018+sign*.025*np.sin(a)]),np.array([sign*(g/2+.018),-.031,-.018])
        a0,b0=ends(.044,nominal);a,b=ends(gap,angle)
        change=np.arctan2((b-a)[2],(b-a)[0])-np.arctan2((b0-a0)[2],(b0-a0)[0])
        t=np.eye(4);t[:3,:3]=R.from_rotvec([0,-change,0]).as_matrix();t[:3,3]=a-t[:3,:3]@a0
        out[side]=t
    return out


class ToolKinematics:
    def __init__(self, stage):
        c=UsdGeom.XformCache()
        self.names=['farmily_lift_height_joint','base','shoulder','elbow','wrist1','wrist2','wrist3']
        self.chain=[]; lower=[]; upper=[]
        for name in self.names:
            p=stage.GetPrimAtPath(ROBOT+'/joints/'+name);j=UsdPhysics.Joint(p)
            bodies=[stage.GetPrimAtPath(x[0]) for x in (j.GetBody0Rel().GetTargets(),j.GetBody1Rel().GetTargets())]
            poses=[np.asarray(c.GetLocalToWorldTransform(b)).T for b in bodies]
            if not self.chain:self.world=poses[0]
            # Authored body transforms are zero-joint poses in this robot USD.
            origin=np.linalg.inv(poses[0])@poses[1]
            axis=np.eye(3)['XYZ'.index(p.GetAttribute('physics:axis').Get())]
            q=j.GetLocalRot1Attr().Get();axis=R.from_quat([*q.GetImaginary(),q.GetReal()]).apply(axis)
            slider=p.IsA(UsdPhysics.PrismaticJoint)
            self.chain.append((origin,axis,slider,str(bodies[1].GetPath())))
            scale=1. if slider else np.pi/180
            lower.append(p.GetAttribute('physics:lowerLimit').Get()*scale)
            upper.append(p.GetAttribute('physics:upperLimit').Get()*scale)
        self.bounds=np.array([lower,upper])
        link=np.asarray(c.GetLocalToWorldTransform(stage.GetPrimAtPath(ROBOT+'/link6'))).T
        tool=np.asarray(c.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/ActuatedTool/fixed'))).T
        self.offset=np.linalg.inv(link)@tool

    def frames(self,q):
        t=self.world.copy();result={}
        for value,(origin,axis,slider,path) in zip(q,self.chain):
            t=t@origin
            if slider:t[:3,3]+=t[:3,:3]@(axis*value)
            else:t[:3,:3]=t[:3,:3]@R.from_rotvec(axis*value).as_matrix()
            result[path]=t.copy()
        return result

    def fk(self,q):
        return list(self.frames(q).values())[-1]@self.offset

    def ik(self,pose,seed):
        def residual(q):
            t=self.fk(q)
            return np.r_[(t[:3,3]-pose[:3,3])*5,
                         R.from_matrix(pose[:3,:3]@t[:3,:3].T).as_rotvec(),(q-seed)*.002]
        result=least_squares(residual,np.clip(seed,*self.bounds),bounds=self.bounds,max_nfev=200)
        t=self.fk(result.x)
        return result.x,float(np.linalg.norm(t[:3,3]-pose[:3,3])),float(R.from_matrix(t[:3,:3]@pose[:3,:3].T).magnitude())


def grasp_pose(center, inward, depth=.065, pitch_deg=0.):
    """CAD +Y faces the fruit; fruit within belt height, depth measured behind nose."""
    y=np.asarray(inward,float);y[2]=0;y/=np.linalg.norm(y)
    z=np.array([0.,0.,1.]);x=np.cross(y,z)
    tilt=R.from_rotvec(x*np.deg2rad(pitch_deg))
    y=tilt.apply(y);z=tilt.apply(z)
    t=np.eye(4);t[:3,:3]=np.column_stack((x,y,z))
    # Active belt span Y=0..158 mm, vertical common interval Z=13..61 mm.
    t[:3,3]=center-t[:3,:3]@np.array([0.,.158-depth,.037])
    return t


def preparation_phases(start, qprep, preparation, recorded_prefix=None):
    """Continuous preparation commands; all profiles retain the physical scene.

    ``low_turn_then_lift`` delays lift travel until the base has turned. It is
    an opt-in path candidate for the known upper-tool/leaf collision during the
    original turn, not a collision-certified path or an initialization reset.
    ``tight_fold_then_turn`` reduces the turning radius before returning to the
    unchanged preparation configuration; its 44 mm gap reduces outer jaw width.
    ``tight_fold_open_approach`` retains that narrow preparation and then holds
    the staging pose for two seconds while opening to 60 mm before approach.
    """
    start, qprep = np.asarray(start, dtype=float), np.asarray(qprep, dtype=float)
    if recorded_prefix is not None and preparation not in ('tight_fold_then_turn', 'tight_fold_open_approach', 'tight_fold_level_approach'):
        raise ValueError('Recorded fold prefix requires a tight-fold preparation profile')
    if preparation == 'direct':
        return [('prepare', 6., np.array([start, qprep]), .060, 0., 0.)]
    if preparation == 'horizontal_ready':
        # Only used to rank branches here. plan_motion replaces this joint
        # interpolation with a Cartesian path from the authored ready state.
        return [('prepare_horizontal', 4., np.array([start, qprep]), .044, 0., 0.)]
    fold = start.copy(); fold[2:4] = qprep[2:4]
    if preparation == 'fold_then_turn':
        orient = qprep.copy(); orient[1] = start[1]
        return [('prepare_fold', 6., np.array([start, fold]), .060, 0., 0.),
                ('prepare_orient', 4., np.array([fold, orient]), .060, 0., 0.),
                ('prepare_turn', 4., np.array([orient, qprep]), .060, 0., 0.)]
    if preparation == 'low_turn_then_lift':
        orient = qprep.copy(); orient[:2] = start[:2]
        turned = qprep.copy(); turned[0] = start[0]
        return [('prepare_fold', 6., np.array([start, fold]), .060, 0., 0.),
                ('prepare_orient_low', 4., np.array([fold, orient]), .060, 0., 0.),
                ('prepare_turn_low', 4., np.array([orient, turned]), .060, 0., 0.),
                ('prepare_lift', 4., np.array([turned, qprep]), .060, 0., 0.)]
    if preparation in ('tight_fold_then_turn', 'tight_fold_open_approach', 'tight_fold_level_approach'):
        tight = fold.copy(); tight[0] = qprep[0]; tight[2:4] = [1.3, -2.3]
        initial = [('prepare_fold', 6., np.array([start, fold]), .044, 0., 0.),
                   ('prepare_tight_fold', 4., np.array([fold, tight]), .044, 0., 0.)]
        if recorded_prefix is not None:
            if len(recorded_prefix)!=2:
                raise ValueError('Recorded fold prefix must contain exactly its first two phases')
            initial=[];previous=start
            for expected,phase in zip(('prepare_fold','prepare_tight_fold'),recorded_prefix):
                if len(phase)!=6:
                    raise ValueError('Recorded fold prefix phase must contain six fields')
                name,duration,qs,gap,left,right=phase
                qs=np.asarray(qs,dtype=float)
                if name!=expected or qs.ndim!=2 or qs.shape[1]!=7 or len(qs)<2 or not np.isfinite(qs).all():
                    raise ValueError('Recorded fold prefix has invalid phase or waypoint shape')
                if not np.isfinite([duration,gap,left,right]).all() or duration<=0 or gap!=.044 or left!=0. or right!=0.:
                    raise ValueError('Recorded fold prefix must preserve the narrow, stationary-belt command')
                if not np.array_equal(qs[0],previous):
                    raise ValueError('Recorded fold prefix must begin at start and remain continuous')
                initial.append((name,float(duration),qs.copy(),float(gap),float(left),float(right)))
                previous=qs[-1]
            # Keep the recorded lift/fold state until the new final extension.
            tight=previous.copy()
        orient = tight.copy(); orient[4:] = qprep[4:]
        if preparation=='tight_fold_level_approach':
            # RB5 shoulder, elbow and wrist1 axes are parallel. Compensating
            # their joint sum keeps the requested tool rotation while the arm
            # extends. FK on the composed robot still verifies this profile.
            orient[4]+=qprep[2]+qprep[3]-tight[2]-tight[3]
        turned = orient.copy(); turned[1] = qprep[1]
        phases = initial+[('prepare_orient_tight', 4., np.array([tight, orient]), .044, 0., 0.),
                ('prepare_turn_tight', 4., np.array([orient, turned]), .044, 0., 0.),
                ('prepare_extend', 4., np.array([turned, qprep]), .044, 0., 0.)]
        if preparation in ('tight_fold_open_approach','tight_fold_level_approach'):
            phases.append(('prepare_open', 2., np.array([qprep, qprep]), .060, 0., 0.))
        return phases
    raise ValueError('Unknown preparation profile ' + preparation)


def plan_motion(kin,center,start=None,preparation='fold_then_turn', *,
                pitch_deg=0.,yaw_offset_deg=0.,recorded_prefix=None,entry_only=False):
    """Plan one fixed entry orientation, without silently changing its pitch.

    CAD +Y is the insertion direction and CAD +Z points up at pitch zero.
    Yaw offset is an explicit rotation about world +Z; it preserves horizontal
    entry. Alternative joint branches may be tried, but the requested pose and
    the 100 mm staging distance remain unchanged. Nonzero pitch is a deliberate
    diagnostic option, never an IK fallback.
    """
    center=np.asarray(center,dtype=float)
    if entry_only and preparation!='horizontal_ready':
        raise ValueError('Entry-only demonstration requires horizontal_ready')
    start=np.zeros(7) if start is None else np.asarray(start,dtype=float)
    if center.shape!=(3,) or start.shape!=(7,) or not np.isfinite(center).all() or not np.isfinite(start).all():
        raise ValueError('Finite center[3] and start[7] are required')
    if not np.isfinite(pitch_deg) or not -90.<=pitch_deg<=90.:
        raise ValueError('Entry pitch must be finite and within -90..90 degrees')
    if not np.isfinite(yaw_offset_deg) or not -180.<=yaw_offset_deg<=180.:
        raise ValueError('Entry yaw offset must be finite and within -180..180 degrees')
    inward=center-np.r_[kin.world[:2,3],center[2]]
    if np.linalg.norm(inward[:2])<1e-9:
        raise ValueError('Target must have a horizontal direction from the robot base')
    inward=R.from_euler('z',yaw_offset_deg,degrees=True).apply(inward)
    goal=grasp_pose(center,inward,pitch_deg=pitch_deg)
    direction=goal[:3,1]
    prep=goal.copy();prep[:3,3]-=direction*.10
    poses=[]
    for fraction in np.linspace(0,1,61):
        pose=prep.copy();pose[:3,3]+=direction*.10*fraction;poses.append(pose)
    seeds=[np.array([.05,b,s,e,w,v,0.]) for b in (-np.pi/2,np.pi/2)
           for s,e,w in [(-1.1,1.8,-.7),(.9,-1.7,.8)] for v in (-np.pi/2,np.pi/2)] + [start]
    attempts=[];feasible=[]
    for index,seed in enumerate(seeds):
        q,pe,re=kin.ik(prep,seed)
        accepted=bool(np.isfinite(q).all() and np.isfinite([pe,re]).all() and pe<=.002 and re<=.02)
        attempt=dict(seed_index=index,preparation_position_error_m=float(pe),
                     preparation_orientation_error_rad=float(re),
                     status='preparation_feasible_suffix_not_evaluated' if accepted else 'preparation_ik_failed')
        attempts.append(attempt)
        if accepted:
            prefix=preparation_phases(start,q,preparation,recorded_prefix)
            # A joint-space ordering heuristic, not a collision certificate.
            length=sum(float(np.linalg.norm(np.diff(phase[2],axis=0),axis=1).sum()) for phase in prefix)
            attempt['preparation_joint_space_length']=length
            feasible.append((length,index,q))
    if not feasible:
        best=min(attempts,key=lambda a:a['preparation_position_error_m']+.1*a['preparation_orientation_error_rad'])
        raise RuntimeError(f'Fixed entry preparation IK not found (pitch={pitch_deg}, yaw_offset={yaw_offset_deg}): '
                           f"position={best['preparation_position_error_m']} orientation={best['preparation_orientation_error_rad']}; pitch was not relaxed")
    waypoints=None;selected=None;ready_waypoints=None
    for _,index,qprep in sorted(feasible,key=lambda item:item[0]):
        ready_branch=None
        if preparation=='horizontal_ready':
            ready_pose=kin.fk(start)
            if (abs(ready_pose[2,3]-prep[2,3])>.002 or
                    R.from_matrix(ready_pose[:3,:3]@prep[:3,:3].T).magnitude()>.02):
                raise ValueError('Horizontal ready start must have the entry height and orientation')
            ready_branch=[start.copy()];qr=start.copy()
            for u in np.linspace(0,1,81)[1:]:
                pose=prep.copy();pose[:3,3]=ready_pose[:3,3]*(1-u)+prep[:3,3]*u
                qr,pe,re=kin.ik(pose,qr)
                if not np.isfinite(qr).all() or not np.isfinite([pe,re]).all() or pe>.002 or re>.02:
                    break
                ready_branch.append(qr.copy())
            if len(ready_branch)!=81:
                attempts[index]['status']='horizontal_preparation_ik_failed'
                continue
            # Continue on this exact branch; never jump to the other prep IK.
            qprep=ready_branch[-1].copy()
        q=qprep.copy();branch=[];errors=[]
        for tick,pose in enumerate(poses):
            q,pe,re=kin.ik(pose,q)
            if not np.isfinite(q).all() or not np.isfinite([pe,re]).all() or pe>.002 or re>.02:
                attempts[index].update(status='approach_ik_failed',failed_fraction=tick/60.,
                                       approach_position_error_m=float(pe),approach_orientation_error_rad=float(re))
                break
            branch.append(q.copy());errors.append([float(pe),float(re)])
        else:
            waypoints=branch;selected=index
            ready_waypoints=ready_branch
            attempts[index]['status']='selected_complete_approach'
            break
    if waypoints is None:
        raise RuntimeError(f'Fixed entry approach IK not found for {len(feasible)} preparation branches '
                           f'(pitch={pitch_deg}, yaw_offset={yaw_offset_deg}); pitch was not relaxed')
    q=waypoints[-1]
    # Slow arm motion, followed by measured jaw closing, transport, twist, withdraw.
    prefix=preparation_phases(start,qprep,preparation,recorded_prefix)
    approach_gap=.044 if preparation=='tight_fold_then_turn' else .060
    if preparation=='horizontal_ready':
        approach_gap=.096  # Existing CAD/slider maximum; open before the cluster.
        prefix=[('ready_hold',.5,np.array([start,start]),.044,0.,0.),
                ('prepare_open',1.,np.array([start,start]),approach_gap,0.,0.),
                ('prepare_horizontal',4.,np.array(ready_waypoints),approach_gap,0.,0.)]
    entry_seconds=4. if preparation=='horizontal_ready' else 8.
    phases=prefix+[('approach',entry_seconds,np.array(waypoints),approach_gap,0.,0.),
            ('close',3.,np.array([q,q]),.028,0.,0.),
            ('infeed',2.,np.array([q,q]),.028,-.020,-.020),
            ('twist',3.,np.array([q,q]),.028,-.015,.015),
            ('withdraw',entry_seconds,np.array(waypoints[::-1]),.028,-.005,-.005),
            ('hold',1.,np.array([qprep,qprep]),.028,0.,0.)]
    if entry_only:
        phases=prefix+[('approach',entry_seconds,np.array(waypoints),approach_gap,0.,0.),
            ('entry_hold',2.,np.array([q,q]),approach_gap,0.,0.),
            ('withdraw',entry_seconds,np.array(waypoints[::-1]),approach_gap,0.,0.),
            ('return_ready',4.,np.array(ready_waypoints[::-1]),approach_gap,0.,0.),
            ('ready_close',1.,np.array([start,start]),.044,0.,0.),
            ('hold',1.,np.array([start,start]),.044,0.,0.)]
    if hasattr(kin,'bounds'):
        for name,_,qs,_,_,_ in phases:
            if np.any(qs<kin.bounds[0]-1e-8) or np.any(qs>kin.bounds[1]+1e-8):
                raise RuntimeError('Planned joint limits exceeded in '+name)
    extension_fk=None
    if preparation=='tight_fold_level_approach' and hasattr(kin,'fk'):
        extension=next(p for p in prefix if p[0]=='prepare_extend')
        samples=[kin.fk(extension[2][0]*(1-u)+extension[2][-1]*u) for u in np.linspace(0,1,81)]
        drift=max(float(R.from_matrix(pose[:3,:3]@goal[:3,:3].T).magnitude()) for pose in samples)
        extension_fk=dict(sample_count=len(samples),orientation_error_max_rad=drift,
                          scope='FK on the composed RB5; actual native tracking is separate')
        if drift>.02:
            raise RuntimeError(f'Level preparation FK orientation check failed: {drift} rad')
    return phases,dict(prep_pose=prep.tolist(),goal_pose=goal.tolist(),start=start.tolist(),
                       prep_q=qprep.tolist(),goal_q=q.tolist(),ik_errors=[pe,re],pitch_deg=float(pitch_deg),
                       yaw_offset_deg=float(yaw_offset_deg),entry_orientation_policy='fixed_requested_pitch_and_yaw',
                       entry_only=bool(entry_only),
                       approach_direction_world=direction.tolist(),approach_distance_m=.10,
                       approach_waypoint_poses=[pose.tolist() for pose in poses],
                       approach_waypoint_ik_errors=errors,selected_seed_index=selected,ik_attempts=attempts,
                       recorded_preparation_prefix_reused=recorded_prefix is not None,
                       preparation_extension_fk=extension_fk,
                       approach_waypoint_fk_poses=([kin.fk(qi).tolist() for qi in waypoints] if hasattr(kin,'fk') else None),
                       preparation=preparation,approach_gap_m=approach_gap,
                       preparation_validation=('Cartesian level ready-to-staging IK; native collision audit required' if preparation=='horizontal_ready'
                           else 'Existing preparation command profile reused; native collision audit required for the selected branch'),
                       phases=[dict(name=n,duration_s=d,waypoint_q=qs.tolist(),
                           end_gap_m=g,belt_local_y_m_s=[l,r]) for n,d,qs,g,l,r in phases],
                       scope='diagnostic IK path, contact audit during physical execution; not collision-certified planning')
