"""Native V6 IK/OMPL paths whose goals come only from fresh camera groups."""
from pathlib import Path
from contextlib import contextmanager,nullcontext
import numpy as np
import mujoco as mj

from side_farm import read,FarmObstacles
from side_farm_motion import jaw_cycle_points
from newton_inputs import sha256
from newton_patrol_motion import motion_duration,fluid_scan,cruise_route
from run_newton_camera_patrol import scan_plan
from preview_grip_workspace import GripPlanner,OPEN_JAWS,joint_connection,approach_path,checked_edge,PlanningError
from preview_posture import side_posture_policy
from preview_side_search import SideTargetSearch
from preview_workspace_motion import CAPTURE_CENTER,capture
from newton_scene_chunks import PackedScene
from plan_adapter import sample_plan

SIMULATION_SPEED_FACTOR = 1.35
LEVEL_ENTRY_TOLERANCE_DEG = 1.5
PREGRASP_CLEARANCE_M = .10
ENTRY_LINE_TOLERANCE_M = .005
MOUTH_CORNERS = np.array([[x,.158,z]for x in (-.10,.10)for z in (-.005,.079)])


def staged_entry_spec(group,centre,rotation):
    """Keep the roller mouth outside the observed fruit, then insert axially."""
    direction=np.asarray(rotation,float)[:,1]
    observed={r['name']:r for r in group['camera_region']['accepted_observations']}
    if not set(group['fruits'])<=set(observed):raise PlanningError('Pregrasp requires the selected visible fruit estimates')
    surfaces=[]
    for name in group['fruits']:
        row=observed[name];point=np.asarray(row['estimated_center_world_m'],float)
        radius=float(row['known_bounding_radius_m'])
        if point.shape!=(3,)or not np.isfinite(point).all()or not np.isfinite(radius)or radius<0:
            raise PlanningError('Invalid observed fruit bound for pregrasp')
        surfaces.append(float(point@direction-radius))
    near=min(surfaces)
    mouth=float(np.asarray(centre)@direction+.158-CAPTURE_CENTER[1])
    distance=max(.10,mouth-near+PREGRASP_CLEARANCE_M+.005)
    return dict(direction_world=direction.tolist(),rotation_world=np.asarray(rotation).tolist(),
        start_capture_m=(np.asarray(centre)-distance*direction).tolist(),end_capture_m=np.asarray(centre).tolist(),
        stroke_m=distance,roller_clearance_m=PREGRASP_CLEARANCE_M,near_surface_projection_m=near,
        position_source='Selected fresh visible depth centre estimates and known bounding radii',
        maximum_line_error_m=ENTRY_LINE_TOLERANCE_M)


def pregrasp_rejection(data,tool,spec):
    direction=np.asarray(spec['direction_world']);rotation=data.xmat[tool].reshape(3,3)
    leading=float(np.max((MOUTH_CORNERS@rotation.T+data.xpos[tool])@direction))
    limit=spec['near_surface_projection_m']-spec['roller_clearance_m']
    if leading>limit+1e-6:
        return dict(rule='roller_entered_crop_before_straight_approach',excess_m=leading-limit)
    return None


def entry_line_error(position,spec):
    delta=np.asarray(position)-spec['start_capture_m'];direction=np.asarray(spec['direction_world'])
    along=float(delta@direction)
    return float(np.linalg.norm(delta-along*direction)),along


def entry_rejection(data,tool,spec,*,tolerance=.002):
    error,along=entry_line_error(capture(data,tool),spec)
    if error>tolerance or not -tolerance<=along<=spec['stroke_m']+tolerance:
        return dict(rule='nonlinear_gripper_insertion',line_error_m=error,along_m=along)
    rotation=data.xmat[tool].reshape(3,3)
    angular=float(np.rad2deg(np.arccos(np.clip((np.trace(np.asarray(spec['rotation_world']).T@rotation)-1.)/2.,-1.,1.))))
    if angular>3.:return dict(rule='gripper_rotated_during_insertion',error_deg=angular)
    return None


def responsive_duration(points, minimum=.08):
    return max(minimum, motion_duration(points,minimum=0.)/SIMULATION_SPEED_FACTOR)


def is_entry_phase(phase):
    return phase in ('approach','inserted_hold','geometric_capture_hold','withdraw') or str(phase).startswith('grip_')


def level_entry_error(rotation):
    return float(np.rad2deg(np.arccos(np.clip(np.asarray(rotation)[2,2],-1.,1.))))


class LocalCheckedPlanner(GripPlanner):
    """Reuse exact accepted states/undirected edges within one planning call.

    This model's farm and tool geometry are immutable during a call. No
    rounded states, failed checks or measured physics audits use this cache.
    """
    def begin_local_checks(self):
        self.checked_states={};self.checked_edges={};self.checked_nine_axis_edges={};self.reuse_checks=True

    def end_local_checks(self):
        self.reuse_checks=False;self.checked_states.clear();self.checked_edges.clear();del self.checked_nine_axis_edges

    @contextmanager
    def constraint(self,callback=None,key=None):
        previous=(getattr(self,'state_constraint',None),getattr(self,'state_constraint_key',None))
        self.state_constraint,self.state_constraint_key=callback,key
        try:yield
        finally:self.state_constraint,self.state_constraint_key=previous

    def constraint_accepts(self):
        callback=getattr(self,'state_constraint',None)
        reason=callback(self.data)if callback else None
        if reason:self.stats['last_posture_rejection']=reason
        return reason is None

    def valid_full(self,q,**kwargs):
        if not getattr(self,'reuse_checks',False):return super().valid_full(q,**kwargs)and self.constraint_accepts()
        q=np.asarray(q,float)
        if q.shape!=(10,) or self.expired():return super().valid_full(q,**kwargs)
        key=(q.tobytes(),kwargs.get('measured',False))
        if key in self.checked_states:
            self.obstacles.update(float(q[1]));self.data.qpos[:]=q;mj.mj_kinematics(self.model,self.data)
            self.stats['exact_state_reuses']=self.stats.get('exact_state_reuses',0)+1
            return self.constraint_accepts()
        valid=super().valid_full(q,**kwargs)
        if valid:self.checked_states[key]=True
        return valid and self.constraint_accepts()

    def edge(self,a,b,**kwargs):
        if not getattr(self,'reuse_checks',False):return super().edge(a,b,**kwargs)
        if self.expired():return super().edge(a,b,**kwargs)
        ends=tuple(sorted(( (np.asarray(q,float).shape,np.asarray(q,float).tobytes())for q in (a,b))))
        key=(getattr(self,'state_constraint_key',None),ends,kwargs.get('full',False),kwargs.get('measured',False))
        if key in self.checked_edges:
            # Restore the normal endpoint FK contract as well as the verdict.
            valid=self.valid_full(b,measured=kwargs.get('measured',False)) if kwargs.get('full') else self.valid_arm(b)
            self.stats['exact_edge_reuses']=self.stats.get('exact_edge_reuses',0)+1
            return valid
        valid=super().edge(a,b,**kwargs)
        if valid:self.checked_edges[key]=True
        return valid


def observed_capture_goal(group, timestamp):
    region=group.get('camera_region',{});obs=region.get('observation',{})
    if (obs.get('stale')is not False or abs(float(obs.get('timestamp_s',-1))-timestamp)>1e-6
            or not obs.get('observation_id') or not set(group['fruits'])<=set(region.get('observable_eligible_fruits',[]))):
        raise PlanningError('Harvest goal requires a fresh observed eligible fruit group')
    selection=group['group_selection'];forward=np.array(selection['direction_world'],float)
    forward/=np.linalg.norm(forward)
    rotation=np.column_stack((np.cross(forward,[0.,0.,1.]),forward,[0.,0.,1.]))
    height=np.mean(selection['tool_height_interval_m'])
    tool_origin=np.asarray(selection['belt_aim_world_m'])-rotation@np.array([0.,.158-selection['depth_m'],height])
    return tool_origin+rotation@CAPTURE_CENTER


class SideCameraMotion:
    def __init__(self,scene,initial,home):
        self.scene=scene;self.home=np.asarray(home,float);self.meta=scene.meta
        model=mj.MjModel.from_binary_path(str(scene.assets/'inputs/input_model.mjb'))
        self.planner=LocalCheckedPlanner(model,initial,FarmObstacles(model,scene.config['side_installation']['obstacle_boxes']),
            posture_guard=True,posture_policy=side_posture_policy(scene.robot,self.meta['gripper_camera_mount']),
            model_sha256=sha256(scene.assets/'inputs/input_model.mjb'))
        self.planner.bounds[1,1]=scene.config['side_camera_patrol']['lift_control_headroom']['working_max_m']
        self.planner.posture_guard.camera_clearance.enable_rigid_cache()
        self.nominal=np.array(read(self.meta['grip_motion_file'])['nominal_tool_rotation_world'])
        self.prior=read(Path(self.meta['grip_motion_file']).parent/'grip_coverage.json')['grid']
        self.search=SideTargetSearch(self.planner,self.nominal,self.prior,orientations=((0.,0.),(0.,-12.),(0.,12.)),
            lift_ik_first=True,approach_distance=.06,endpoint_accept=self.grasp_clearance,max_cartesian_step=.005)

    def pregrasp_constraint(self,spec):
        key=('pregrasp',tuple(spec['direction_world']),spec['near_surface_projection_m'],spec['roller_clearance_m'])
        return self.planner.constraint(lambda data:pregrasp_rejection(data,self.planner.tool_id,spec),key)

    def entry_constraint(self,spec):
        key=('entry',tuple(spec['start_capture_m']),tuple(spec['end_capture_m']),tuple(np.ravel(spec['rotation_world'])))
        return self.planner.constraint(lambda data:entry_rejection(data,self.planner.tool_id,spec),key)

    def grasp_clearance(self,state):
        """A hold target must clear contacts, even within the motion's 1mm cap.

        A sub-millimetre nominal wrist/gripper overlap can deflect the actual
        servo by several millimetres indefinitely; waiting longer cannot fix it.
        """
        p=self.planner;q=p.initial.copy();q[1:]=state
        p.obstacles.update(float(q[1]));p.data.qpos[:]=q
        mj.mj_kinematics(p.model,p.data);mj.mj_collision(p.model,p.data)
        for contact in p.data.contact:
            if (p.robot_geoms[contact.geom1]or p.robot_geoms[contact.geom2]) and contact.dist<.0005:
                p.stats['last_posture_rejection']=dict(rule='grasp_contact_clearance',
                    geom1=p.model.geom(contact.geom1).name,geom2=p.model.geom(contact.geom2).name,
                    distance_m=float(contact.dist),minimum_m=.0005)
                return False
        return True

    def _anchor(self,q,*,open_jaws=False):
        if not self.planner.valid_full(q,measured=True):raise PlanningError('Measured start rejected: '+str(self.planner.stats))
        goal=np.clip(q,self.planner.bounds[:,0],self.planner.bounds[:,1])
        gap=.044+goal[9]-goal[8]
        goal[8:]=OPEN_JAWS if open_jaws else [-(gap-.044)/2.,(gap-.044)/2.]
        if not self.planner.valid_full(goal):raise PlanningError('Nominal start connector goal rejected')
        return goal

    def _prepend_connector(self,plan,q,anchor):
        if not np.array_equal(q,anchor):
            phase=dict(name='measured_start_connector',duration_s=responsive_duration([q,anchor],minimum=.04),
                waypoint_q=[q[1:8].tolist(),anchor[1:8].tolist()],rail_waypoint_positions_m=[float(q[0]),float(anchor[0])],
                end_gap_m=float(.044+anchor[9]-anchor[8]),belt_local_y_m_s=[0.,0.])
            plan['phases'].insert(0,phase);plan['total_duration_s']+=phase['duration_s']
        plan['initial_command']=q.tolist();plan['initial_gap_m']=float(.044+q[9]-q[8])
        return plan

    def _base(self,q,phases,**extra):
        return dict(schema='s300-v6-visible-camera-motion-v1',initial_command=q.tolist(),
            initial_gap_m=float(.044+q[9]-q[8]),settle_seconds=1/220.,phases=phases,
            total_duration_s=1/220.+sum(p['duration_s']for p in phases),
            harvest_mode='main_stem_only_geometric_harvest',physical_detachment_enabled=False,
            collision_certified=True,model_sha256=self.scene.provenance['physical_model_sha256'],
            training_eligible=False,harvest_success=None,**extra)

    def _check(self,plan,*,sampled=False):
        previous=np.array(plan['initial_command'],float)
        for phase in plan['phases']:
            points=np.column_stack((phase['rail_waypoint_positions_m'],phase['waypoint_q'],
                np.tile(OPEN_JAWS,(len(phase['waypoint_q']),1))))
            # Jaws are independently audited for the complete two-cycle motion
            # below; transfer/rail edges use fully open jaws conservatively.
            points[0]=previous
            gap=phase['end_gap_m'];points[-1,8:]=[-(gap-.044)/2.,(gap-.044)/2.]
            for a,b in zip(points[:-1],points[1:]):
                reuse=(getattr(self.planner,'reuse_checks',False)and phase['name']!='measured_start_connector'
                    and a[0]==b[0]==self.planner.initial[0])
                spec=plan.get('staged_entry')
                constraint=(self.pregrasp_constraint(spec)if spec and phase['name']in ('transfer','pregrasp_align','return')else
                    self.entry_constraint(spec)if spec and is_entry_phase(phase['name'])else nullcontext())
                with constraint:
                    valid=(checked_edge(self.planner,a[1:],b[1:]) if reuse else
                        self.planner.edge(a,b,full=True,measured=phase['name']=='measured_start_connector'))
                if not valid:
                    raise PlanningError('Native full path rejected: '+str(self.planner.stats))
            previous=points[-1]
        if sampled:
            for t in np.arange(0.,plan['total_duration_s']+1/300.,1/300.):
                sample=sample_plan(plan,min(t,plan['total_duration_s']))
                if not self.planner.valid_full(sample['command'],measured=sample['phase']in ('settle','measured_start_connector')):
                    raise PlanningError('Cruise interpolation rejected: '+str(self.planner.stats))
        return plan

    def scan(self,q,destination=None,*,cruise_end=None,step=.15,rail_velocity=0.):
        q=np.array(q,float)
        anchor=self._anchor(q)
        self.planner.initial=anchor.copy();self.planner.fixed=anchor.copy()
        if cruise_end is not None:
            plan=cruise_route(anchor,cruise_end,step_m=step,rail_velocity=rail_velocity/SIMULATION_SPEED_FACTOR)
            for phase in plan['phases']:
                phase['waypoint_q']=[self.home[1:8].tolist()]*len(phase['waypoint_q'])
                phase['end_gap_m']=.096
            plan['phases'][0]['waypoint_q'][0]=anchor[1:8].tolist()
            for phase in plan['phases']:
                phase['duration_s']/=SIMULATION_SPEED_FACTOR
                if 'rail_boundary_velocities_m_s' in phase:
                    phase['rail_boundary_velocities_m_s']=(np.array(phase['rail_boundary_velocities_m_s'])*SIMULATION_SPEED_FACTOR).tolist()
            plan['total_duration_s']=plan['settle_seconds']+sum(row['duration_s']for row in plan['phases'])
            plan['collision_certified']=True
            return self._check(self._prepend_connector(plan,q,anchor),sampled=True)
        plan=fluid_scan(scan_plan(anchor,destination=destination,ready_q=self.home[1:8]),rail_velocity=rail_velocity/SIMULATION_SPEED_FACTOR)
        for phase in plan['phases']:
            phase['end_gap_m']=.096
            phase['duration_s']/=SIMULATION_SPEED_FACTOR
            if 'rail_boundary_velocities_m_s' in phase:
                phase['rail_boundary_velocities_m_s']=(np.array(phase['rail_boundary_velocities_m_s'])*SIMULATION_SPEED_FACTOR).tolist()
        plan['total_duration_s']=plan['settle_seconds']+sum(row['duration_s']for row in plan['phases'])
        plan['collision_certified']=True
        return self._check(self._prepend_connector(plan,q,anchor))

    def grip(self,q,group,timestamp):
        self.planner.begin_local_checks()
        try:return self._grip(q,group,timestamp)
        finally:self.planner.end_local_checks()

    def _grip(self,q,group,timestamp):
        q=np.array(q,float);centre=observed_capture_goal(group,timestamp)
        opened=self._anchor(q,open_jaws=True)
        p=self.planner;p.initial=opened.copy();p.fixed=opened.copy()
        # Keep the last successful IK seed across observed groups. Shift only
        # the static hints into the actual measured rail frame.
        original=self.search.prior
        self.search.prior=[dict(r,target_capture_m=(np.array(r['target_capture_m'])+[0.,q[0],0.]).tolist())for r in original]
        side='left'if group['group_selection']['direction_world'][0]<0 else 'right'
        try:state,rotation,ik=self.search.solve(centre,side,0,np.array([centre-.005,centre+.005]),
            approach_distance=lambda rotation:staged_entry_spec(group,centre,rotation)['stroke_m'])
        finally:self.search.prior=original
        if state is None:raise PlanningError('Finite observed-goal IK search: '+str(ik))
        approach=np.array(ik.pop('approach_positions'))
        entry=staged_entry_spec(group,centre,rotation)
        with self.pregrasp_constraint(entry):transfer,route=joint_connection(p,opened[1:],approach[0])
        phases=[];current=q.copy()
        def full(points):return np.column_stack((np.full(len(points),opened[0]),points))
        target=dict(side=side,centre=centre.tolist(),observed_fruits=group['fruits'],
            observation=group['camera_region']['observation'],ik=ik,transfer_planner=route,
            target_source='Fresh visible optical-depth estimates; authored fruit centres not used')
        def add(name,points,minimum=.15):
            nonlocal current
            points=np.asarray(points,float)
            np.testing.assert_allclose(points[0],current,rtol=0,atol=1e-9)
            phases.append(dict(name=name,duration_s=responsive_duration(points,minimum=minimum),
                waypoint_q=points[:,1:8].tolist(),rail_waypoint_positions_m=points[:,0].tolist(),
                end_gap_m=float(.044+points[-1,9]-points[-1,8]),belt_local_y_m_s=[0.,0.],target=target))
            current=points[-1].copy()
        add('measured_start_connector',[q,opened],.025)
        cautious=len(group['fruits'])==1 or group.get('coverage_proposal',{}).get('candidate',0)>0
        entry_minimum=1.0 if cautious else .45
        add('transfer',full(transfer));add('pregrasp_align',[current,current],.30)
        add('approach',full(approach),entry_minimum)
        add('inserted_hold',[current,current],.08)
        for name,points,duration in jaw_cycle_points(current,cycles=2):
            minimum=.08 if 'open_hold' in name else .30 if 'hold' in name else .45
            add(name,points,minimum)
        add('withdraw',full(approach[::-1]),entry_minimum)
        add('return',full(transfer[::-1]))
        home=self.home.copy();home[0]=opened[0]
        add('camera_ready',[current,home],.025)
        return self._check(self._base(q,phases,targets=[target],grip_cycles=2,observed_group=group,staged_entry=entry))

    def audit_measured(self,trial,plan):
        states=dict(np.load(Path(trial)/'states.npz',allow_pickle=False))
        np.testing.assert_allclose(states['qpos'][0],plan['initial_command'],rtol=0,atol=1e-6)
        maximum_angle=0.;maximum_level_error=0.;entry_count=0;minimum_left=1.
        spec=plan.get('staged_entry');maximum_line_error=0.;minimum_standoff=float('inf');ready=None
        if spec:
            phases=[p['name']for p in plan['phases']]
            if not phases.index('transfer')<phases.index('pregrasp_align')<phases.index('approach')<phases.index('grip_close_1'):
                raise ValueError('Harvest requires transfer, pregrasp alignment, then straight insertion before closing')
        for q,phase in zip(states['qpos'],states['phase']):
            if not self.planner.valid_full(q,measured=True):raise ValueError('Measured V6 posture rejected: '+str(self.planner.stats))
            rot=self.planner.data.xmat[self.planner.tool_id].reshape(3,3)
            fwd=rot@np.array(self.meta['gripper_camera_mount']['body_from_camera_optical'])[:3,2]
            minimum_left=min(minimum_left,float(-fwd[0]/np.linalg.norm(fwd[:2])))
            if is_entry_phase(phase):
                maximum_angle=max(maximum_angle,float(np.rad2deg(np.arccos(np.clip(-rot[0,1],-1,1)))));entry_count+=1
                maximum_level_error=max(maximum_level_error,level_entry_error(rot))
                if spec:
                    reason=entry_rejection(self.planner.data,self.planner.tool_id,spec,tolerance=ENTRY_LINE_TOLERANCE_M)
                    if reason:raise ValueError('Measured straight insertion rejected: '+str(reason))
                    maximum_line_error=max(maximum_line_error,entry_line_error(capture(self.planner.data,self.planner.tool_id),spec)[0])
            if spec and phase in ('transfer','pregrasp_align','return'):
                leading=float(np.max((MOUTH_CORNERS@rot.T+self.planner.data.xpos[self.planner.tool_id])@spec['direction_world']))
                clearance=spec['near_surface_projection_m']-leading;minimum_standoff=min(minimum_standoff,clearance)
                if clearance<PREGRASP_CLEARANCE_M-ENTRY_LINE_TOLERANCE_M:
                    raise ValueError('Measured roller entered crop before the straight insertion phase')
                if phase=='pregrasp_align':
                    ready=float(np.linalg.norm(capture(self.planner.data,self.planner.tool_id)-spec['start_capture_m']))
        if spec and (ready is None or ready>.005):raise ValueError('Measured pregrasp alignment not reached before insertion')
        if maximum_angle>15.:raise ValueError('Measured insertion exceeded left-normal 15 degree cone')
        if maximum_level_error>LEVEL_ENTRY_TOLERANCE_DEG:raise ValueError('Measured gripper tilted during level insertion')
        strokes=[]
        for i in (1,2):
            indices=np.flatnonzero(states['phase']==f'grip_close_{i}')
            if not len(indices):continue
            gaps=.044+states['qpos'][indices,9]-states['qpos'][indices,8]
            if gaps.min()>.040:raise ValueError('Actual jaw stroke did not close')
            strokes.append(dict(cycle=i,minimum_gap_m=float(gaps.min())))
        if plan.get('grip_cycles')and len(strokes)!=2:raise ValueError('Exactly two actual jaw closures required')
        result=dict(status='passed',state_checks=len(states['qpos']),entry_samples=entry_count,
            maximum_entry_angle_deg=maximum_angle,maximum_level_entry_error_deg=maximum_level_error,
            level_entry_limit_deg=LEVEL_ENTRY_TOLERANCE_DEG,minimum_camera_left_alignment=minimum_left,measured_strokes=strokes)
        if spec:result['staged_entry']=dict(status='passed',minimum_roller_clearance_before_entry_m=minimum_standoff,
            pregrasp_position_error_m=ready,maximum_line_error_m=maximum_line_error,
            requested_standoff_m=PREGRASP_CLEARANCE_M,stroke_m=spec['stroke_m'])
        return result
