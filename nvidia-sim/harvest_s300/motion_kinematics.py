"""Tool-frame adapter for the existing URDF FK/least-squares IK approach.

Unlike the old ring frame, this endpoint is the actual V5.1 CAD origin. Joint
frames/limits come from the composed USD. No physical body is teleported by FK.
"""
import numpy as np
from numbers import Integral
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from pxr import UsdGeom, UsdPhysics
from motion_assets import ROBOT, TOOL
from transmission_geometry import transmission_transforms

BELT_PROFILES = ('belt_spin_release', 'belt_open_close_cycle')


def cycle_opening_check(gaps_m, required_gap_m=.096, tolerance_m=.0005):
    """Measured opening before reclose; never substitute the command for feedback."""
    gaps=np.asarray(gaps_m,dtype=float)
    return dict(actual_gap_m=gaps.tolist(),required_gap_m=required_gap_m,
                tolerance_m=tolerance_m,
                reached=bool(gaps.size and np.isfinite(gaps).all() and
                             (np.abs(gaps-required_gap_m)<=tolerance_m).all()))


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

    def ik_at_lift(self,pose,seed,lift_m):
        """Solve the six arm joints at an explicitly fixed existing lift height."""
        if not np.isfinite(lift_m) or not self.bounds[0,0]<=lift_m<=self.bounds[1,0]:
            raise ValueError('Requested lift branch exceeds the existing joint limits')
        seed=np.asarray(seed,float)
        def assemble(arm):return np.r_[float(lift_m),arm]
        def residual(arm):
            t=self.fk(assemble(arm))
            return np.r_[(t[:3,3]-pose[:3,3])*5,
                         R.from_matrix(pose[:3,:3]@t[:3,:3].T).as_rotvec(),(arm-seed[1:])*.002]
        result=least_squares(residual,np.clip(seed[1:],self.bounds[0,1:],self.bounds[1,1:]),
                             bounds=(self.bounds[0,1:],self.bounds[1,1:]),max_nfev=200)
        q=assemble(result.x);t=self.fk(q)
        return q,float(np.linalg.norm(t[:3,3]-pose[:3,3])),float(R.from_matrix(t[:3,:3]@pose[:3,:3].T).magnitude())


def ready_lift_branch(kin,start,offset_m):
    """Continuous lift/arm redistribution while the ready TCP stays stationary.

    This is an explicit alternative physical path, not a changed initial state.
    Native tracking and the complete arm/tool collision screen remain required.
    """
    if not hasattr(kin,'ik_at_lift'):
        raise ValueError('Explicit lift branch requires constrained lift IK')
    ready=kin.fk(start);target=float(start[0]+offset_m);rows=[np.asarray(start,float).copy()];errors=[]
    for lift in np.linspace(float(start[0]),target,81)[1:]:
        q,pe,re=kin.ik_at_lift(ready,rows[-1],float(lift))
        if not np.isfinite(q).all() or not np.isfinite([pe,re]).all() or pe>.002 or re>.02:
            raise RuntimeError(f'Ready lift redistribution IK not found at lift={lift}: position={pe} orientation={re}')
        rows.append(q.copy());errors.append([float(pe),float(re)])
    return np.asarray(rows),dict(offset_m=float(offset_m),fixed_lift_m=target,
                                ready_tcp_pose=ready.tolist(),ik_errors=errors,
                                scope='Continuous TCP-holding lift redistribution; full collision audit and native tracking required')


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


def belt_speed_fraction(elapsed_s,duration_s,ramp_s):
    """Smooth startup/shutdown inside the requested spin duration, without added time."""
    if not np.isfinite([elapsed_s,duration_s,ramp_s]).all() or duration_s<=0 or not 0<=ramp_s<=duration_s/2:
        raise ValueError('Belt ramp must be finite and between zero and half the spin duration')
    if ramp_s==0:return 1.
    u=float(np.clip(min(elapsed_s,duration_s-elapsed_s)/ramp_s,0.,1.))
    return u*u*(3-2*u)


def equivalent_joint_goal_near_start(kin,goal,start):
    """Nearest in-limit 2π representation, proven equal for every FK link.

    This changes an opt-in joint interpolation, not the physical initial state
    or requested staging pose. Prismatic joints are never wrapped. Endpoint FK
    equality does not certify the intervening path; full collision checks and
    native tracking remain required.
    """
    original=np.asarray(goal,float);start=np.asarray(start,float)
    bounds=np.asarray(getattr(kin,'bounds',None),float)
    chain=getattr(kin,'chain',None)
    if (original.shape!=(7,) or start.shape!=(7,) or bounds.shape!=(2,7)
            or not np.isfinite(np.r_[original,start,bounds.ravel()]).all()
            or np.any(bounds[0]>bounds[1]) or chain is None or len(chain)!=7
            or not hasattr(kin,'frames')):
        raise ValueError('Turn normalization requires exact finite seven-joint bounds, joint types and link FK')
    if np.any(original<bounds[0]) or np.any(original>bounds[1]):
        raise ValueError('Original staging joint goal exceeds existing limits')
    result=original.copy();turns=[]
    for index,item in enumerate(chain):
        if len(item)<3 or not isinstance(item[2],(bool,np.bool_)):
            raise ValueError('Turn normalization requires explicit revolute/prismatic joint identity')
        if item[2]:turns.append(0);continue
        lower=int(np.ceil((bounds[0,index]-original[index])/(2*np.pi)))
        upper=int(np.floor((bounds[1,index]-original[index])/(2*np.pi)))
        choices=[(abs(original[index]+2*np.pi*n-start[index]),abs(n),n,
                  original[index]+2*np.pi*n) for n in range(lower,upper+1)
                 if bounds[0,index]<=original[index]+2*np.pi*n<=bounds[1,index]]
        if not choices:raise ValueError('No equivalent staging joint within existing limits')
        _,_,n,value=min(choices);result[index]=value;turns.append(int(n))
    original_frames=kin.frames(original);canonical_frames=kin.frames(result)
    if not original_frames or set(original_frames)!=set(canonical_frames):
        raise ValueError('Turn normalization changed FK link mapping')
    errors={}
    for path,frame in original_frames.items():
        a=np.asarray(frame,float);b=np.asarray(canonical_frames[path],float)
        if a.shape!=(4,4) or b.shape!=(4,4) or not np.isfinite(np.r_[a.ravel(),b.ravel()]).all():
            raise ValueError('Nonfinite or malformed turn-normalization link FK')
        errors[path]=float(np.max(np.abs(a-b)))
    if max(errors.values())>1e-10:
        raise ValueError('2π normalization changed a physical FK link pose')
    return result,dict(enabled=True,schema='s300-opt-in-bounded-joint-turns-v1',
        branch_start_q=start.tolist(),original_staging_q=original.tolist(),canonical_staging_q=result.tolist(),
        integer_turn_shifts=turns,original_joint_displacement=(original-start).tolist(),
        canonical_joint_displacement=(result-start).tolist(),existing_bounds=bounds.tolist(),
        all_link_fk_max_abs_error=max(errors.values()),link_fk_errors=errors,link_fk_tolerance=1e-10,
        collision_certified=False,physical_initial_state_changed=False,
        scope='Equivalent staging endpoint only; opt-in interpolation and subsequent IK require complete self/environment/native validation')


def plan_motion(kin,center,start=None,preparation='fold_then_turn', *,
                pitch_deg=0.,yaw_offset_deg=0.,recorded_prefix=None,entry_only=False,
                harvest_profile='legacy',belt_speed_m_s=.15,belt_seconds=2.,close_seconds=3.,belt_ramp_seconds=0.,
                belt_infeed_speed_m_s=0.,grasp_gap_m=.028,grasp_height_offset_m=0.,grasp_lateral_offset_m=0.,
                grasp_translation_world_m=(0.,0.,0.),preparation_height_first=False,grasp_depth_m=.065,
                approach_direction_world=None,preparation_gap_m=None,ready_lift_offset_m=0.,
                preparation_opening='ready',
                max_grasp_translation_m=.5,allow_measured_ready_start=False,
                horizontal_ready_connector='cartesian',horizontal_ready_seed_index=None,
                horizontal_ready_normalize_turns=False,approach_distance_m=.10):
    """Plan one fixed entry orientation, without silently changing its pitch.

    CAD +Y is the insertion direction and CAD +Z points up at pitch zero.
    Yaw offset is an explicit rotation about world +Z; it preserves horizontal
    entry. Alternative joint branches may be tried, but the requested pose and
    the default 100 mm staging distance remain unchanged. An explicit staging
    distance moves only the retracted preparation pose and insertion waypoints;
    final grasp position and entry rotation are unchanged. Nonzero pitch is a deliberate
    diagnostic option, never an IK fallback. An explicit ready lift offset adds
    a continuous TCP-holding redistribution and constrains the existing lift
    throughout the entry; it never substitutes a different initial state.
    Open at the stationary ready pose before any arm/lift preparation motion.
    A narrow preparation gap is used only by explicit legacy-staging replay.
    An explicit measured-ready option connects the actual initial FK pose to
    level staging through physical IK waypoints. It does not replace initial
    joints, relax the final entry pose, or certify the connector as collision-free.
    The opt-in joint connector retains a globally solved staging IK branch and
    physically interpolates into it; insertion remains fixed and horizontal.
    An explicit lateral offset shifts the arm goal along the requested CAD +X
    axis. It preserves the fruit pose, entry rotation and insertion direction;
    negative offset puts the unchanged fruit farther toward tool +X.
    """
    center=np.asarray(center,dtype=float)
    if entry_only and preparation!='horizontal_ready':
        raise ValueError('Entry-only demonstration requires horizontal_ready')
    if preparation_height_first and preparation!='horizontal_ready':
        raise ValueError('Height-first transfer requires horizontal_ready')
    if not isinstance(allow_measured_ready_start,(bool,np.bool_)) or (
            allow_measured_ready_start and (preparation!='horizontal_ready' or
                                           harvest_profile not in BELT_PROFILES)):
        raise ValueError('Measured ready start requires a boolean and horizontal-ready belt motion')
    if horizontal_ready_connector not in ('cartesian','joint') or (
            horizontal_ready_connector!='cartesian' and
            (preparation!='horizontal_ready' or harvest_profile not in BELT_PROFILES)):
        raise ValueError('Horizontal ready connector must be cartesian or joint; joint requires horizontal-ready belt motion')
    if not isinstance(horizontal_ready_normalize_turns,(bool,np.bool_)) or (
            horizontal_ready_normalize_turns and
            (horizontal_ready_connector!='joint' or preparation!='horizontal_ready' or harvest_profile not in BELT_PROFILES)):
        raise ValueError('Turn normalization requires a boolean and the horizontal-ready joint belt connector')
    if horizontal_ready_seed_index is not None and (
            isinstance(horizontal_ready_seed_index,bool) or not isinstance(horizontal_ready_seed_index,Integral) or
            not 0<=horizontal_ready_seed_index<=8 or horizontal_ready_connector!='joint'):
        raise ValueError('Explicit ready seed must be an integer within 0..8 and requires the joint connector')
    if not np.isfinite(grasp_depth_m) or not .015<=grasp_depth_m<=.10:
        raise ValueError('Grasp depth must be finite and within 15..100 mm behind the nose')
    if grasp_depth_m!=.065 and harvest_profile not in BELT_PROFILES:
        raise ValueError('Grasp depth override requires a belt profile')
    if harvest_profile not in ('legacy',*BELT_PROFILES):
        raise ValueError('Unknown harvest profile')
    if harvest_profile in BELT_PROFILES and (entry_only or preparation!='horizontal_ready'):
        raise ValueError('Belt-spin-release requires horizontal_ready and is not entry-only')
    if (isinstance(approach_distance_m,(bool,np.bool_)) or not np.isscalar(approach_distance_m)
            or not np.isfinite(approach_distance_m) or not .05<=approach_distance_m<=.30
            or approach_distance_m!=.10 and (preparation!='horizontal_ready' or harvest_profile not in BELT_PROFILES)):
        raise ValueError('Entry retract requires finite 50..300 mm; an override requires horizontal-ready belt motion')
    if preparation_gap_m is not None:
        if (preparation!='horizontal_ready' or harvest_profile not in BELT_PROFILES or
                not np.isfinite(preparation_gap_m) or not .028<=preparation_gap_m<=.096):
            raise ValueError('Preparation gap requires horizontal-ready belt motion and 28..96 mm')
    if preparation_opening not in ('ready','legacy-staging'):
        raise ValueError('Preparation opening must be ready or legacy-staging')
    if (not np.isfinite(ready_lift_offset_m) or abs(ready_lift_offset_m)>.4 or
            ready_lift_offset_m and (preparation!='horizontal_ready' or harvest_profile not in BELT_PROFILES)):
        raise ValueError('Ready lift offset requires horizontal-ready belt motion and finite +/-400 mm')
    if not np.isfinite(max_grasp_translation_m) or not 0<max_grasp_translation_m<=2.:
        raise ValueError('Maximum grasp translation must be finite, positive and at most 2 m')
    if not np.isfinite(belt_speed_m_s) or belt_speed_m_s<0:
        raise ValueError('Differential belt speed must be finite and nonnegative')
    if not np.isfinite(belt_seconds) or belt_seconds<=0:
        raise ValueError('Belt duration must be finite and positive')
    if not np.isfinite(close_seconds) or close_seconds<=0:
        raise ValueError('Close duration must be finite and positive')
    if close_seconds!=3. and harvest_profile not in BELT_PROFILES:
        raise ValueError('Close duration override requires belt_spin_release')
    if not np.isfinite(grasp_gap_m) or not .028<=grasp_gap_m<=.096:
        raise ValueError('Grasp gap must be within existing mechanical travel 28..96 mm')
    if grasp_gap_m!=.028 and harvest_profile not in BELT_PROFILES:
        raise ValueError('Grasp gap override requires belt_spin_release')
    belt_speed_fraction(0.,belt_seconds,belt_ramp_seconds)
    if belt_ramp_seconds and harvest_profile not in BELT_PROFILES:raise ValueError('Belt ramp requires a belt profile')
    if not np.isfinite(belt_infeed_speed_m_s) or belt_infeed_speed_m_s<0 or (belt_infeed_speed_m_s and harvest_profile not in BELT_PROFILES):
        raise ValueError('Nonnegative belt infeed requires belt_spin_release')
    if belt_speed_m_s==0 and (harvest_profile not in BELT_PROFILES or belt_infeed_speed_m_s==0):
        raise ValueError('Zero differential speed requires a belt profile with positive inward speed')
    start=np.zeros(7) if start is None else np.asarray(start,dtype=float)
    if center.shape!=(3,) or start.shape!=(7,) or not np.isfinite(center).all() or not np.isfinite(start).all():
        raise ValueError('Finite center[3] and start[7] are required')
    if (not np.isfinite(grasp_lateral_offset_m) or abs(grasp_lateral_offset_m)>.020 or
            grasp_lateral_offset_m and (preparation!='horizontal_ready' or harvest_profile not in BELT_PROFILES)):
        raise ValueError('Grasp lateral offset requires horizontal-ready belt motion and finite +/-20 mm')
    target_center=center.copy()
    lift_branch=None;lift_metadata=None;branch_start=start
    if ready_lift_offset_m:
        lift_branch,lift_metadata=ready_lift_branch(kin,start,ready_lift_offset_m)
        branch_start=lift_branch[-1]
    solve=(lambda pose,seed:kin.ik_at_lift(pose,seed,lift_metadata['fixed_lift_m'])) if lift_metadata else kin.ik
    if not np.isfinite(grasp_height_offset_m) or abs(grasp_height_offset_m)>.05:
        raise ValueError('Grasp height offset must be finite and within +/-50 mm')
    if grasp_height_offset_m and harvest_profile not in BELT_PROFILES:
        raise ValueError('Grasp height override requires belt_spin_release')
    center=center.copy();center[2]+=grasp_height_offset_m
    if not np.isfinite(pitch_deg) or not -90.<=pitch_deg<=90.:
        raise ValueError('Entry pitch must be finite and within -90..90 degrees')
    if not np.isfinite(yaw_offset_deg) or not -180.<=yaw_offset_deg<=180.:
        raise ValueError('Entry yaw offset must be finite and within -180..180 degrees')
    if approach_direction_world is None:
        inward=center-np.r_[kin.world[:2,3],center[2]]
        if np.linalg.norm(inward[:2])<1e-9:
            raise ValueError('Target must have a horizontal direction from the robot base')
        inward=R.from_euler('z',yaw_offset_deg,degrees=True).apply(inward)
    else:
        inward=np.asarray(approach_direction_world,dtype=float).copy()
        if (preparation!='horizontal_ready' or pitch_deg!=0 or yaw_offset_deg!=0 or
                inward.shape!=(3,) or not np.isfinite(inward).all() or
                abs(inward[2])>1e-9 or np.linalg.norm(inward[:2])<1e-9):
            raise ValueError('Truss direction requires horizontal_ready, zero pitch/yaw offset and finite XY direction')
        inward[2]=0.
    goal=grasp_pose(center,inward,depth=grasp_depth_m,pitch_deg=pitch_deg)
    lateral_translation=goal[:3,0]*float(grasp_lateral_offset_m)
    # Keep the zero/default path's arithmetic and command arrays exact. Only
    # the requested arm aim moves; source fruit/body transforms are untouched.
    if grasp_lateral_offset_m:
        goal[:3,3]+=lateral_translation;center=center+lateral_translation
    translation=np.asarray(grasp_translation_world_m,dtype=float)
    if translation.shape!=(3,) or not np.isfinite(translation).all() or np.linalg.norm(translation)>max_grasp_translation_m:
        raise ValueError('Grasp translation must be finite XYZ within '+str(max_grasp_translation_m)+' m')
    if np.any(translation) and (preparation!='horizontal_ready' or harvest_profile not in BELT_PROFILES):
        raise ValueError('Translated grasp requires horizontal-ready belt motion')
    # Move only the arm goal; retain the established horizontal tool orientation
    # and ready configuration. The physical fruit/plant is never repositioned.
    goal[:3,3]+=translation;center=center+translation
    direction=goal[:3,1]
    prep=goal.copy();prep[:3,3]-=direction*approach_distance_m
    poses=[]
    for fraction in np.linspace(0,1,61):
        pose=prep.copy();pose[:3,3]+=direction*approach_distance_m*fraction;poses.append(pose)
    seeds=[np.array([.05,b,s,e,w,v,0.]) for b in (-np.pi/2,np.pi/2)
           for s,e,w in [(-1.1,1.8,-.7),(.9,-1.7,.8)] for v in (-np.pi/2,np.pi/2)] + [start]
    attempts=[];feasible=[]
    for index,seed in enumerate(seeds):
        q,pe,re=solve(prep,seed)
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
    if horizontal_ready_seed_index is not None:
        feasible=[item for item in feasible if item[1]==horizontal_ready_seed_index]
        if not feasible:
            raise RuntimeError('Explicit staging IK seed not feasible: '+str(horizontal_ready_seed_index))
    waypoints=None;selected=None;ready_waypoints=None;measured_ready_metadata=None;joint_ready_metadata=None
    for _,index,qprep in sorted(feasible,key=lambda item:item[0]):
        ready_branch=None
        if preparation=='horizontal_ready' and horizontal_ready_connector=='joint':
            # Preserve this actual global staging IK branch. The Cartesian
            # connector can get trapped on the measured ready branch even when
            # several staging configurations exist; this explicit alternative
            # moves real joints and is subject to the same complete FCL audit.
            normalization=None
            if horizontal_ready_normalize_turns:
                qprep,normalization=equivalent_joint_goal_near_start(kin,qprep,branch_start)
            ready_branch=[branch_start.copy(),qprep.copy()]
            staging_fk=kin.fk(qprep)
            joint_ready_metadata=dict(initial_q=start.tolist(),initial_fk_pose=kin.fk(start).tolist(),
                branch_start_q=branch_start.tolist(),staging_q=qprep.tolist(),staging_fk_pose=staging_fk.tolist(),
                staging_position_error_m=float(np.linalg.norm(staging_fk[:3,3]-prep[:3,3])),
                staging_orientation_error_rad=float(R.from_matrix(staging_fk[:3,:3]@prep[:3,:3].T).magnitude()),
                staging_seed_index=int(index),collision_certified=False,
                scope='Actual measured joints to retained global staging IK; preparation may rotate; fixed horizontal insertion and full native/FCL validation remain required')
            if normalization is not None:joint_ready_metadata['turn_normalization']=normalization
        elif preparation=='horizontal_ready':
            ready_pose=kin.fk(start)
            # An explicit aim-height offset is reached during preparation;
            # preserve the authored ready pose and keep the final 100 mm entry level.
            orientation_invalid=(R.from_matrix(ready_pose[:3,:3]@prep[:3,:3].T).magnitude()>.02
                                 if approach_direction_world is None else
                                 np.linalg.norm(ready_pose[:3,2]-[0.,0.,1.])>.002)
            if not allow_measured_ready_start and (
                    abs(ready_pose[2,3]-(prep[2,3]-grasp_height_offset_m-translation[2]))>.002 or orientation_invalid):
                raise ValueError('Horizontal ready start must have the entry height and orientation')
            via=ready_pose.copy();via[2,3]=prep[2,3]
            if allow_measured_ready_start:
                # A resumed robot may have lift sag and small roll/pitch tracking
                # error; a raised/mobile base may legitimately change ready
                # height. Reach level at the height leg's end using actual q
                # commands, then yaw toward the truss. Never set the native
                # rigid pose or discard the measured starting rotation.
                horizontal_y=ready_pose[:3,1].copy();horizontal_y[2]=0.
                if np.linalg.norm(horizontal_y)<1e-9:
                    horizontal_y=prep[:3,1].copy()
                horizontal_y/=np.linalg.norm(horizontal_y)
                via[:3,:3]=np.column_stack((np.cross(horizontal_y,[0.,0.,1.]),
                                             horizontal_y,[0.,0.,1.]))
            legs=[(ready_pose,via),(via,prep)] if preparation_height_first else [(ready_pose,prep)]
            transfer_poses=[]
            for begin,end in legs:
                for u in np.linspace(0,1,81)[1:]:
                    pose=prep.copy();pose[:3,3]=begin[:3,3]*(1-u)+end[:3,3]*u
                    if allow_measured_ready_start:
                        relative=R.from_matrix(end[:3,:3]@begin[:3,:3].T).as_rotvec()
                        pose[:3,:3]=R.from_rotvec(relative*u).as_matrix()@begin[:3,:3]
                    elif approach_direction_world is not None:
                        # Rotate only around world Z during preparation, smoothly
                        # from the unchanged ready pose. Insertion keeps fixed yaw.
                        delta=R.from_matrix(end[:3,:3]@begin[:3,:3].T).as_rotvec()[2]
                        pose[:3,:3]=R.from_euler('z',delta*u).as_matrix()@begin[:3,:3]
                    transfer_poses.append(pose)
            ready_branch=[branch_start.copy()];qr=branch_start.copy()
            for pose in transfer_poses:
                qr,pe,re=solve(pose,qr)
                if not np.isfinite(qr).all() or not np.isfinite([pe,re]).all() or pe>.002 or re>.02:
                    break
                ready_branch.append(qr.copy())
            if len(ready_branch)!=len(transfer_poses)+1:
                attempts[index].update(status='horizontal_preparation_ik_failed',
                    failed_transfer_waypoint=len(ready_branch),transfer_waypoint_count=len(transfer_poses),
                    preparation_connector_position_error_m=float(pe),
                    preparation_connector_orientation_error_rad=float(re))
                continue
            # Continue on this exact branch; never jump to the other prep IK.
            qprep=ready_branch[-1].copy()
            if allow_measured_ready_start:
                measured_ready_metadata=dict(
                    initial_q=start.tolist(),initial_fk_pose=ready_pose.tolist(),
                    level_via_pose=via.tolist() if preparation_height_first else None,
                    staging_pose=prep.tolist(),
                    initial_height_difference_m=float(ready_pose[2,3]-(prep[2,3]-grasp_height_offset_m-translation[2])),
                    initial_up_axis_error=float(np.linalg.norm(ready_pose[:3,2]-[0.,0.,1.])),
                    sample_count=len(ready_branch),
                    interpolation='actual FK translation plus full SO(3) rotation; level staging followed by fixed entry',
                    collision_certified=False,
                    scope='Physical joint commands from measured start; full FCL audit and actual native tracking required')
        q=qprep.copy();branch=[];errors=[]
        for tick,pose in enumerate(poses):
            q,pe,re=solve(pose,q)
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
        evaluated=[attempts[index]['status']for _,index,_ in feasible]
        if allow_measured_ready_start and evaluated and all(status=='horizontal_preparation_ik_failed'for status in evaluated):
            raise RuntimeError(f'Measured ready-to-staging connector IK not found for {len(feasible)} feasible staging branches; '
                               'fixed horizontal insertion was not evaluated; target pose was not classified impossible')
        raise RuntimeError(f'Fixed entry approach IK not found for {len(feasible)} preparation branches '
                           f'(pitch={pitch_deg}, yaw_offset={yaw_offset_deg}); pitch was not relaxed')
    q=waypoints[-1]
    # Slow arm motion, followed by measured jaw closing, transport, twist, withdraw.
    prefix=preparation_phases(start,qprep,preparation,recorded_prefix)
    approach_gap=.044 if preparation=='tight_fold_then_turn' else .060
    if preparation=='horizontal_ready':
        approach_gap=.096  # Existing CAD/slider maximum; open before the cluster.
        prefix=[('ready_hold',.5,np.array([start,start]),.044,0.,0.),
                ('prepare_open',1.,np.array([start,start] if preparation_opening=='ready' else [branch_start,branch_start]),approach_gap,0.,0.),
                ('prepare_horizontal',4.,np.array(ready_waypoints),approach_gap,0.,0.)]
        return_phases=[('return_ready',4.,np.array(ready_waypoints[::-1]),approach_gap,0.,0.)]
        if horizontal_ready_connector=='joint':
            # Reuse the existing direct joint interpolation duration. All
            # positions/rotations during this preparation are real FK results;
            # only the final requested insertion is constrained to level yaw.
            direct=preparation_phases(branch_start,qprep,'direct')[0]
            prefix=prefix[:2]+[('prepare_joint_branch',direct[1],direct[2],approach_gap,0.,0.)]
            return_phases=[('return_joint_branch',direct[1],direct[2][::-1],approach_gap,0.,0.)]
        elif preparation_height_first:
            prefix=prefix[:2]+[
                ('prepare_height',3.,np.array(ready_waypoints[:81]),approach_gap,0.,0.),
                ('prepare_horizontal',4.,np.array(ready_waypoints[80:]),approach_gap,0.,0.)]
            return_phases=[('return_ready',4.,np.array(ready_waypoints[80:][::-1]),approach_gap,0.,0.),
                           ('return_height',3.,np.array(ready_waypoints[:81][::-1]),approach_gap,0.,0.)]
        if preparation_gap_m is not None and preparation_opening=='legacy-staging':
            # Narrow only the transfer. Opening is completed at staging before
            # the same fully open Cartesian insertion; physical feedback is
            # checked by the runner rather than inferred from these commands.
            transfer=[(name,duration,qs,float(preparation_gap_m),left,right)
                      for name,duration,qs,_,left,right in prefix[2:]]
            prefix=[prefix[0],('prepare_gap',1.,np.array([branch_start,branch_start]),float(preparation_gap_m),0.,0.),
                    *transfer,('prepare_open',1.,np.array([qprep,qprep]),approach_gap,0.,0.)]
        if lift_branch is not None:
            if preparation_opening=='ready':
                prefix=[*prefix[:2],('prepare_lift_branch',4.,lift_branch,approach_gap,0.,0.),*prefix[2:]]
            else:
                prefix=[prefix[0],('prepare_lift_branch',4.,lift_branch,.044,0.,0.),*prefix[1:]]
            return_phases += [('return_lift_branch',4.,lift_branch[::-1],approach_gap,0.,0.)]
        if preparation_opening=='ready':
            # Give the physical jaws time to settle at the commanded maximum
            # before checking feedback or moving the lift/arm. Do not loosen
            # the measured opening tolerance to hide tracking lag.
            prefix.insert(2,('prepare_open_hold',.5,np.array([start,start]),approach_gap,0.,0.))
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
            *return_phases,
            ('ready_close',1.,np.array([start,start]),.044,0.,0.),
            ('hold',1.,np.array([start,start]),.044,0.,0.)]
    if harvest_profile=='belt_spin_release':
        # Opposed belt contact velocities rotate the fruit; the arm holds its
        # entry orientation. Detachment is left to the native breakable joint.
        phases=prefix+[('approach',entry_seconds,np.array(waypoints),approach_gap,0.,0.),
            ('close',float(close_seconds),np.array([q,q]),float(grasp_gap_m),0.,0.),
            ('belt_spin',float(belt_seconds),np.array([q,q]),float(grasp_gap_m),
             -float(belt_speed_m_s)+float(belt_infeed_speed_m_s),float(belt_speed_m_s)+float(belt_infeed_speed_m_s)),
            ('release_open',1.,np.array([q,q]),approach_gap,0.,0.),
            ('withdraw',entry_seconds,np.array(waypoints[::-1]),approach_gap,0.,0.),
            *return_phases,
            ('ready_close',1.,np.array([start,start]),.044,0.,0.),
            ('hold',1.,np.array([start,start]),.044,0.,0.)]
    belt_cycle=None
    if harvest_profile=='belt_open_close_cycle':
        # Native loaded-sphere evidence (20261011) shows that negative local-Y
        # PhysX surface commands transport toward CAD +Y, the tool mouth.
        # Positive common commands therefore feed toward CAD -Y, the interior.
        # Zero common infeed preserves the historical opposed commands exactly.
        left=-float(belt_speed_m_s)+float(belt_infeed_speed_m_s)
        right=float(belt_speed_m_s)+float(belt_infeed_speed_m_s)
        # Two complete close/open repetitions in one insertion. Keep the belts
        # running through BOTH openings; stop only after the final open hold.
        # Native separation during either close is an observation, not a failure.
        cycle=[('belt_start_open',max(1.,float(belt_ramp_seconds)),np.array([q,q]),approach_gap,left,right),
               ('close',float(close_seconds),np.array([q,q]),float(grasp_gap_m),left,right),
               ('cycle_open',1.,np.array([q,q]),approach_gap,left,right),
               ('cycle_open_hold',.5,np.array([q,q]),approach_gap,left,right),
               ('cycle_reclose',float(close_seconds),np.array([q,q]),float(grasp_gap_m),left,right),
               ('belt_finish',float(belt_seconds),np.array([q,q]),float(grasp_gap_m),left,right),
               ('cycle_open_final',1.,np.array([q,q]),approach_gap,left,right),
               ('cycle_open_final_hold',.5,np.array([q,q]),approach_gap,left,right)]
        phases=prefix+[('approach',entry_seconds,np.array(waypoints),approach_gap,0.,0.)]+cycle+[
            ('withdraw',entry_seconds,np.array(waypoints[::-1]),approach_gap,0.,0.),
            *return_phases,
            ('ready_close',1.,np.array([start,start]),.044,0.,0.),
            ('hold',1.,np.array([start,start]),.044,0.,0.)]
        belt_cycle=dict(start_after_settle_s=sum(p[1] for p in prefix)+entry_seconds,
                        duration_s=sum(p[1] for p in cycle),phases=[p[0] for p in cycle],
                        full_open_gap_m=approach_gap,reclose_requires_measured_open=True,
                        opening_tracking_tolerance_m=.0005,
                        close_open_repetitions=2,
                        repetition_phases=[dict(close='close',open='cycle_open',open_hold='cycle_open_hold'),
                                           dict(close='cycle_reclose',open='cycle_open_final',open_hold='cycle_open_final_hold')],
                        withdraw_requires_measured_open=True,
                        detachment_during_close_is_allowed=True,
                        finish='second complete opening with belts running, verify open, stop belts and withdraw open')
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
                       yaw_offset_deg=float(yaw_offset_deg),entry_orientation_policy=(
                           'truss_xy_yaw_only' if approach_direction_world is not None else 'fixed_requested_pitch_and_yaw'),
                       entry_only=bool(entry_only),
                       harvest_profile=harvest_profile,
                       belt_cycle=belt_cycle,
                       grasp_lateral_offset_m=float(grasp_lateral_offset_m),
                       grasp_lateral_translation_world_m=lateral_translation.tolist(),
                       target_center_world_m=target_center.tolist(),
                       grasp_translation_world_m=translation.tolist(),
                       max_grasp_translation_m=float(max_grasp_translation_m),
                       preparation_height_first=bool(preparation_height_first),
                       ready_lift_branch=lift_metadata,
                       measured_ready_connector=measured_ready_metadata,
                       joint_ready_connector=joint_ready_metadata,
                       horizontal_ready_connector=horizontal_ready_connector,
                       horizontal_ready_seed_index=None if horizontal_ready_seed_index is None else int(horizontal_ready_seed_index),
                       preparation_height_first_applied=bool(preparation_height_first and horizontal_ready_connector=='cartesian'),
                       preparation_gap_m=float(preparation_gap_m) if preparation_gap_m is not None else None,
                       preparation_opening=(dict(location=preparation_opening,
                           requested_transfer_gap_m=preparation_gap_m,
                           effective_transfer_gap_m=(approach_gap if preparation_opening=='ready' or preparation_gap_m is None else float(preparation_gap_m)),
                           feedback_before_phases=([p[0] for p in prefix if p[0] not in ('ready_hold','prepare_open','prepare_open_hold')]+['approach']
                               if preparation_opening=='ready' else ['approach']),
                           required_gap_m=approach_gap,tracking_tolerance_m=.0005)
                           if preparation=='horizontal_ready' else None),
                       grasp_depth_m=float(grasp_depth_m),
                       grasp_height_offset_m=float(grasp_height_offset_m),grasp_center_world_m=center.tolist(),
                       belt_spin=(dict(duration_s=float(belt_seconds),surface_speed_m_s=float(belt_speed_m_s),
                           opposed=bool(belt_infeed_speed_m_s<belt_speed_m_s),arm_rotates=False,closed_gap_m=float(grasp_gap_m),close_seconds=float(close_seconds),
                           ramp_seconds=float(belt_ramp_seconds),
                           infeed_speed_m_s=float(belt_infeed_speed_m_s),
                           infeed_command_convention='native_positive_local_y_inward_v2',
                           inward_transport_tool_axis=[0.,-1.,0.],
                           detachment='native fruit-pedicel joint break only; no timed release')
                           if harvest_profile in BELT_PROFILES else None),
                       approach_direction_world=direction.tolist(),approach_distance_m=float(approach_distance_m),
                       approach_waypoint_poses=[pose.tolist() for pose in poses],
                       approach_waypoint_ik_errors=errors,selected_seed_index=selected,ik_attempts=attempts,
                       recorded_preparation_prefix_reused=recorded_prefix is not None,
                       preparation_extension_fk=extension_fk,
                       approach_waypoint_fk_poses=([kin.fk(qi).tolist() for qi in waypoints] if hasattr(kin,'fk') else None),
                       preparation=preparation,approach_gap_m=approach_gap,
                       preparation_validation=('Cartesian ready-to-staging IK with explicit height adjustment; entry orientation fixed; native collision audit required' if preparation=='horizontal_ready' and grasp_height_offset_m
                           else 'Cartesian level ready-to-staging IK; native collision audit required' if preparation=='horizontal_ready'
                           else 'Existing preparation command profile reused; native collision audit required for the selected branch'),
                       phases=[dict(name=n,duration_s=d,waypoint_q=qs.tolist(),
                           end_gap_m=g,belt_local_y_m_s=[l,r]) for n,d,qs,g,l,r in phases],
                       scope='diagnostic IK path, contact audit during physical execution; not collision-certified planning')
