"""Checked bilateral grip poses for the concept assemblies (no dynamics).

Uses the existing native FK, IK and OMPL checker. New bin/lift/motor boxes are
added only to the private preview copy, using disabled hidden-plant geometry
slots. Neither the production model nor the CAD renderer is mutated.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import time

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from newton_inputs import sha256
from newton_ompl import Planner, PlanningError, MAX_EDGE_STEP, PENETRATION_LIMIT, ob, og
from newton_patrol_motion import VELOCITY, ACCELERATION
from newton_viser import geom_mesh
from preview_posture import POSTURE_POLICY, PostureGuard, side_posture_policy
from preview_workspace_motion import (
    CAPTURE_CENTER, MAX_LIFT_STEP, WorkspaceMotion, capture, collision_model,
    edge_steps, refit_body_bvh,
)

OPEN_JAWS = np.array([-.026, .026])
CLOSED_JAWS = np.array([.004, -.004])
REGIONS = {
    'right': np.array([[.30, -.15, .30], [.78, .80, 1.04]]),
    'left': np.array([[-.78, -.15, .30], [-.30, .80, 1.04]]),
}


def layout_boxes(scene, meta, displayed, bin_parts, lift, *, layout_key=None):
    from preview_side_collision import CHASSIS_ASSEMBLIES, SLICED_LIFT_FEET, cad_boxes
    boxes = [dict(p, name='bin/'+p['name']) for p in bin_parts]
    fixed = set(meta['source_fixed_lift_geometry'])
    for gid in range(scene.ngeom):
        name=scene.geom(gid).name
        if layout_key=='side' and name in CHASSIS_ASSEMBLIES:
            boxes.extend(cad_boxes(scene,gid,displayed[int(scene.geom_bodyid[gid])]))
            continue
        if scene.geom(gid).name not in fixed:
            continue
        bid = int(scene.geom_bodyid[gid])
        if layout_key=='side' and name in SLICED_LIFT_FEET:
            boxes.extend(cad_boxes(scene,gid,displayed[bid],sliced=True,prefix='lift/'))
            continue
        vertices = np.asarray(geom_mesh(scene, gid).vertices)
        world = vertices@displayed[bid, :3, :3].T+displayed[bid, :3, 3]
        low, high = world.min(0), world.max(0)
        boxes.append(dict(name='lift/'+scene.geom(gid).name,
            position=((low+high)/2).tolist(), dimensions=(high-low).tolist()))
    if layout_key == 'side':
        # The vertical carriage and adapter move with the lift. The original
        # horizontal plate and removed motor must not remain ghost obstacles.
        shift=np.asarray(meta['layouts']['side']['shift_world_m'])
        low,high=np.asarray(meta['carriage_zero_lift_bounds_m'])+shift
        low=low.copy();high=high.copy();low[2]+=lift;high[2]+=lift
        boxes.append(dict(name='moving_lift_carriage',position=((low+high)/2).tolist(),
            dimensions=(high-low).tolist(),follows_lift=True,reference_lift_m=lift))
        side=meta['layouts']['side'];size=np.asarray(side['adapter_size_m'])
        centre=np.array([(low[0]+high[0])/2.,low[1]-size[1]/2.,
                         high[2]-side['mount_centre_below_carriage_top_m']])
        boxes.append(dict(name='moving_side_adapter',position=centre.tolist(),dimensions=size.tolist(),
            follows_lift=True,reference_lift_m=lift))
        return boxes
    low, high = np.asarray(meta['source_motor_world_bounds_m'])
    corners = np.array([[x,y,z] for x in (low[0],high[0])
                        for y in (low[1],high[1]) for z in (low[2],high[2])])
    delta = displayed[meta['motor_id']]@np.linalg.inv(displayed[meta['chassis_id']])
    world = corners@delta[:3,:3].T+delta[:3,3]
    low, high = world.min(0), world.max(0)
    boxes.append(dict(name='moving_lift_motor', position=((low+high)/2).tolist(),
        dimensions=(high-low).tolist(), follows_lift=True, reference_lift_m=lift))
    return boxes


class BoxObstacles:
    """Native box contacts, with BVH updates and an optional moving motor."""
    def __init__(self, model, initial_q, boxes):
        self.model, self.boxes = model, boxes
        data = mj.MjData(model)
        data.qpos[:] = initial_q
        mj.mj_kinematics(model, data)
        slots = []
        for gid in range(model.ngeom):
            bid = int(model.geom_bodyid[gid])
            if (model.geom_contype[gid] == model.geom_conaffinity[gid] == 0 and
                    not model.body(bid).name.startswith(('Robot_', 'Tool_')) and
                    model.body_geomnum[bid] == 1 and model.body_bvhnum[bid] > 0):
                slots.append(gid)
        if len(slots) < len(boxes):
            raise ValueError('Not enough disabled hidden-plant slots for preview obstacles')
        self.rows = []
        self.aliases = {}
        for gid, box in zip(slots, boxes):
            bid = int(model.geom_bodyid[gid])
            rotation = data.xmat[bid].reshape(3,3)
            local = rotation.T@(np.asarray(box['position'])-data.xpos[bid])
            model.geom_type[gid] = mj.mjtGeom.mjGEOM_BOX
            model.geom_dataid[gid] = -1
            model.geom_sameframe[gid] = 0
            model.geom_pos[gid] = local
            model.geom_quat[gid] = Rotation.from_matrix(rotation.T).as_quat()[[3,0,1,2]]
            model.geom_size[gid] = np.asarray(box['dimensions'])/2
            model.geom_aabb[gid] = np.r_[np.zeros(3), model.geom_size[gid]]
            model.geom_rbound[gid] = np.linalg.norm(model.geom_size[gid])
            # New obstacles contact moving robot links/plate only. Their
            # attachment to the fixed chassis must not be treated as collision.
            model.geom_contype[gid], model.geom_conaffinity[gid] = 0, 4
            self.aliases[model.geom(gid).name] = box['name']
            self.rows.append(dict(gid=gid, bid=bid, local=local.copy(),
                direction=rotation.T@np.array([0.,0.,1.]), box=box))
        for gid in range(model.ngeom):
            body_name = model.body(int(model.geom_bodyid[gid])).name
            if (body_name.startswith(('Robot_link','Tool_','Robot_farmily_lift_platform'))
                    and (model.geom_contype[gid] or model.geom_conaffinity[gid])):
                model.geom_contype[gid] |= 4
        # The compiled broad phase caches each body's OR of geometry masks;
        # changing geom masks alone leaves new obstacles invisible to it.
        model.body_contype[:] = 0
        model.body_conaffinity[:] = 0
        np.bitwise_or.at(model.body_contype,model.geom_bodyid,model.geom_contype)
        np.bitwise_or.at(model.body_conaffinity,model.geom_bodyid,model.geom_conaffinity)
        mj.mj_setConst(model, data)
        for bid in {r['bid'] for r in self.rows}:
            # A single primitive has no BVH nodes; its updated geom_aabb and
            # geom_rbound are used directly by the native broad phase.
            if model.body_bvhnum[bid]: refit_body_bvh(model, bid)
        self.lift = None

    def update(self, lift):
        if self.lift == lift: return
        for row in self.rows:
            box = row['box']
            if not box.get('follows_lift'): continue
            self.model.geom_pos[row['gid']] = row['local']+row['direction']*(lift-box['reference_lift_m'])
            if self.model.body_bvhnum[row['bid']]: refit_body_bvh(self.model, row['bid'])
        self.lift = lift


class GripPlanner(Planner):
    def __init__(self, model, initial, obstacles, *, posture_guard=False, posture_policy=None, **kwargs):
        self.obstacles = obstacles
        self.posture_guard = PostureGuard(model, obstacles, policy=posture_policy) if posture_guard else None
        super().__init__(model, initial, **kwargs)
        if self.posture_guard:
            self.bounds = self.posture_guard.tighten_bounds(self.bounds)
            bounds = ob.RealVectorBounds(6)
            for i, (low, high) in enumerate(self.bounds[2:8]):
                bounds.setLow(i, float(low)); bounds.setHigh(i, float(high))
            self.space.setBounds(bounds)

    def valid_full(self, q, **kwargs):
        if self.posture_guard:
            q = np.asarray(q, float)
            if q.shape != (10,) or not np.isfinite(q).all(): return False
            reason = self.posture_guard.joint_rejection(q)
            if reason:
                self.stats['last_posture_rejection'] = reason
                return False
            self.data.qpos[:] = q
            mj.mj_kinematics(self.model,self.data)
            reason = self.posture_guard.geometry_rejection(self.data)
            if reason:
                self.stats['last_posture_rejection'] = reason
                return False
        self.obstacles.update(float(q[1]))
        result = super().valid_full(q, **kwargs)
        if not result and 'last_collision' in self.stats:
            c = self.stats['last_collision']
            for key in ('geom1','geom2'):
                c[key] = self.obstacles.aliases.get(c[key], c[key])
        if result and self.posture_guard:
            reason = self.posture_guard.rejection(self.data)
            if reason:
                self.stats['last_posture_rejection'] = reason
                return False
        return result

    def ik_with_lift(self,goal,seed):
        """Same damped Jacobian IK with the real lift as a seventh variable."""
        target=np.asarray(goal['position']);rotation=Rotation.from_quat(goal['quaternion_xyzw']).as_matrix()
        scale=np.r_[.25,np.ones(6)];state=np.asarray(seed,float).copy()
        limits=self.bounds[1:8]
        state=np.clip(state,limits[:,0],limits[:,1])
        for _ in range(200 if self.posture_guard else 140):
            self.fixed[1]=state[0]
            pos,orient=self._pose(state[1:])
            angular=Rotation.from_matrix(rotation@orient.T).as_rotvec();delta=target-pos
            if np.linalg.norm(delta)<.001 and np.linalg.norm(angular)<.01:
                if self.valid_arm(state[1:]):return state.copy()
                if not self.posture_guard or self.posture_guard.forearm_pitch(self.data)<=10.:break
            jacp=np.zeros((3,self.model.nv));jacr=np.zeros_like(jacp)
            mj.mj_jacBody(self.model,self.data,jacp,jacr,self.tool_id)
            jac=np.vstack((jacp[:,1:8],.25*jacr[:,1:8]))*scale[None,:]
            error=np.r_[delta,.25*angular]
            inverse=jac.T@np.linalg.solve(jac@jac.T+np.eye(6)*1e-5,np.eye(6))
            change=inverse@error
            if self.posture_guard:
                # Use the lift redundancy to keep the elbow above the wrist,
                # rather than stopping at the first folded Cartesian solution.
                elbow_jac=np.zeros_like(jacp);unused=np.zeros_like(jacp)
                wrist_jac=np.zeros_like(jacp)
                mj.mj_jacBody(self.model,self.data,elbow_jac,unused,self.posture_guard.elbow_body)
                mj.mj_jacBody(self.model,self.data,wrist_jac,unused,self.posture_guard.wrist_body)
                slope=(wrist_jac-elbow_jac)[2,1:8]*scale
                gradient=-slope*max(0.,self.posture_guard.forearm_pitch(self.data)+5.)*.04
                change+=(np.eye(7)-inverse@jac)@gradient
            change*=min(1.,.12/max(1e-12,float(np.max(np.abs(change)))))
            next_state=np.clip(state+scale*change,limits[:,0],limits[:,1])
            if np.max(np.abs(next_state-state))<1e-8:break
            state=next_state
        raise PlanningError('No collision-valid lift+arm IK in bounded search')

    def ik_at_lift(self, goal, lift, seed, elbow_branch):
        """Probe the lift redundancy explicitly without changing posture limits."""
        if elbow_branch not in (-1, 1):
            raise ValueError('Elbow branch must be -1 or +1')
        if not self.bounds[1,0] <= lift <= self.bounds[1,1]:
            raise PlanningError('Requested lift outside actual limits')
        limits=self.bounds[2:8].copy()
        if self.posture_guard:
            limits[2]=[-math.radians(140.),-math.radians(20.)] if elbow_branch<0 else [math.radians(20.),math.radians(140.)]
        arm=np.clip(np.asarray(seed,float),limits[:,0],limits[:,1])
        target=np.asarray(goal['position'],float)
        rotation=Rotation.from_quat(goal['quaternion_xyzw']).as_matrix()
        self.fixed=self.initial.copy();self.fixed[1]=lift;self.fixed[8:10]=OPEN_JAWS
        for _ in range(100):
            pos,orient=self._pose(arm)
            delta=target-pos;angular=Rotation.from_matrix(rotation@orient.T).as_rotvec()
            if np.linalg.norm(delta)<.001 and np.linalg.norm(angular)<.01:
                if self.valid_arm(arm):return np.r_[lift,arm]
                break
            jacp=np.zeros((3,self.model.nv));jacr=np.zeros_like(jacp)
            mj.mj_jacBody(self.model,self.data,jacp,jacr,self.tool_id)
            jac=np.vstack((jacp[:,2:8],.25*jacr[:,2:8]))
            change=jac.T@np.linalg.solve(jac@jac.T+np.eye(6)*1e-5,np.r_[delta,.25*angular])
            change*=min(1.,.18/max(1e-12,float(np.max(np.abs(change)))))
            next_arm=np.clip(arm+change,limits[:,0],limits[:,1])
            if np.max(np.abs(next_arm-arm))<1e-8:break
            arm=next_arm
        raise PlanningError('No collision-valid Cartesian IK at this lift/branch')


class StorageGripMotion(WorkspaceMotion):
    def sample(self, elapsed):
        q, sample = super().sample(elapsed)
        segment = self.segments[sample['segment']]
        phase = segment['phase']
        if sample['reverse']:
            phase = {'approach':'retreat','retreat':'approach','close':'open','open':'close'}.get(phase,phase)
        sample.update(lift_height_m=float(q[0]), jaw_positions_m=q[7:9].tolist(),
            side=segment['side'], phase=phase, coverage_index=segment.get('coverage_index'))
        return q, sample


def state_steps(a,b):
    return max(edge_steps(a[:7],b[:7]), int(math.ceil(np.max(np.abs(b[7:]-a[7:]))/.0005)))


def segment_duration(points):
    derivative = np.max(np.abs(np.diff(points,axis=0)),axis=0)*(len(points)-1)
    return 1.1*max(.3,float(np.max(1.875*derivative/VELOCITY[1:10])),
        float(np.sqrt(np.max(5.774*derivative/ACCELERATION[1:10]))))


def checked_edge(planner,a,b,accept=None):
    steps = state_steps(a,b)
    for u in np.linspace(0.,1.,steps+1):
        full=planner.initial.copy();full[1:10]=a+u*(b-a)
        if not planner.valid_full(full):return False
        if accept is not None and not accept():return False
    return True


def require_continuous_segments(segments,initial):
    """Equivalent hinge turns must still connect through an actual trajectory."""
    current=np.asarray(initial)[1:10]
    for segment in segments:
        points=np.asarray(segment['points'])
        if points.ndim!=2 or points.shape[1]!=9 or len(points)<2 or not np.isfinite(points).all():
            raise ValueError('Grip segment requires finite nine-axis points')
        if not np.allclose(points[0],current,atol=1e-9,rtol=0):
            raise ValueError('Discontinuous grip trajectory; joint-turn teleport rejected')
        current=points[-1]


def joint_connection(planner,a,b):
    """Reuse native checks and OMPL, scaling the lift to its .5mm resolution."""
    if checked_edge(planner,a,b):return np.array([a,b]),'direct'
    if not np.array_equal(a[7:],b[7:]):raise PlanningError('Transfer requires fixed open jaws')
    scale=np.r_[.25,np.ones(6)]
    space=ob.RealVectorStateSpace(7)
    bounds=ob.RealVectorBounds(7)
    for i,(lo,hi)in enumerate(planner.bounds[1:8]/scale[:,None]):
        bounds.setLow(i,float(lo));bounds.setHigh(i,float(hi))
    space.setBounds(bounds)
    space.setLongestValidSegmentFraction(MAX_EDGE_STEP/space.getMaximumExtent())
    si=ob.SpaceInformation(space)
    def valid(state):
        q=planner.initial.copy();q[1:8]=np.array([state[i]for i in range(7)])*scale;q[8:10]=a[7:]
        return planner.valid_full(q)
    si.setStateValidityChecker(valid);si.setup()
    problem=ob.ProblemDefinition(si);start=space.allocState();goal=space.allocState()
    for i in range(7):start[i]=float(a[i]/scale[i]);goal[i]=float(b[i]/scale[i])
    problem.setStartAndGoalStates(start,goal)
    rrt=og.RRTConnect(si);rrt.setRange(.25);rrt.setProblemDefinition(problem);rrt.setup()
    planner.deadline=time.monotonic()+3.
    try:
        rrt.solve(2.5)
        if not problem.hasExactSolution():raise PlanningError('No collision-free seven-axis transfer within 2.5s')
        path=problem.getSolutionPath()
        points=np.array([np.r_[np.array([path.getState(k)[j]for j in range(7)])*scale,a[7:]]
                         for k in range(path.getStateCount())])
    finally:
        planner.deadline=math.inf
    # Shortcut only through independently checked edges; retain exact endpoints.
    compact=[points[0]];i=0
    while i<len(points)-1:
        for j in range(len(points)-1,i,-1):
            if checked_edge(planner,points[i],points[j]):break
        compact.append(points[j]);i=j
    return np.asarray(compact),'OMPL RRTConnect (lift + six arm axes)'


def approach_path(planner,state,rotation,centre,distance=.07):
    """A 70mm straight insertion, preserving the declared side-facing pose."""
    direction=rotation@np.array([0.,1.,0.])
    points=[];seed=state[1:7].copy()
    planner.fixed=planner.initial.copy();planner.fixed[1]=state[0];planner.fixed[8:10]=OPEN_JAWS
    for offset in np.linspace(distance,0.,6):
        target=centre-offset*direction
        seed=planner.ik(dict(position=target-rotation@CAPTURE_CENTER,
            quaternion_xyzw=Rotation.from_matrix(rotation).as_quat()),seed)
        current=np.r_[state[0],seed,OPEN_JAWS]
        if points and not checked_edge(planner,points[-1],current):
            raise PlanningError('Straight grip insertion collides')
        points.append(current)
    return np.asarray(points)


def nearest_joint_turns(state,reference,bounds):
    """Equivalent hinge turns inside the actual limits, close to the start."""
    state=state.copy()
    for i in range(1,7):
        values=[state[i]+k*2*math.pi for k in (-2,-1,0,1,2)]
        values=[v for v in values if bounds[i+1,0]<=v<=bounds[i+1,1]]
        if not values:
            raise PlanningError('No equivalent joint turn inside the working limits')
        state[i]=min(values,key=lambda v:abs(v-reference[i]))
    return state


def plan_storage_motion(source, initial, meta, displayed, scene, bin_parts, output,*,profile='wide',layout_key='storage',seed_coverage=None,nominal_rotation_world=None):
    """Scan both marked sides, then retain only audited grip cycles."""
    started = time.monotonic()
    q = np.asarray(initial['qpos']).copy()
    model, installation = collision_model(source,q,meta,displayed,output,layout_key=layout_key)
    boxes = layout_boxes(scene,meta,displayed,bin_parts,q[1],layout_key=layout_key)
    obstacles = BoxObstacles(model,q,boxes)
    planner = GripPlanner(model,initial,obstacles,
                          posture_guard=layout_key in ('conveyor','side'),
                          posture_policy=side_posture_policy(scene,meta.get('gripper_camera_mount')) if layout_key=='side' else None,
                          model_sha256=installation['collision_configuration_sha256'])
    planner.fixed = q.copy()
    if not planner.valid_full(q):
        raise ValueError(f'Grip start rejected: {planner.stats}')
    nominal = (displayed[planner.tool_id,:3,:3].copy() if nominal_rotation_world is None
               else np.asarray(nominal_rotation_world,float).copy())
    if nominal.shape!=(3,3) or not np.isfinite(nominal).all():raise ValueError('Expected a finite world goal rotation')
    np.testing.assert_allclose(nominal.T@nominal,np.eye(3),atol=1e-8,rtol=0)
    if profile not in ('wide','near'):raise ValueError('Unknown grip workspace profile')
    regions=REGIONS if profile=='wide'else{
        'right':np.array([[.30,-.15,.30],[.50,.80,1.30]]),
        'left':np.array([[-.50,-.15,.30],[-.30,.80,1.30]])}
    xs=(.35,.55,.75)if profile=='wide'else(.305,.35,.40,.45,.495)
    ys=(-.10,.15,.40,.65,.78)if profile=='wide'else(-.10,.10,.30,.50,.75)
    zs=(.40,.65,.90,1.02)if profile=='wide'else(.40,.55,.70,.85,1.,1.15,1.28)
    if profile=='near' and layout_key=='side':
        zs=(.40,.55,.60,.65,.70,.85,1.,1.15,1.28)
    approach_distance=.07 if profile=='wide'else .04
    prior=[]
    if seed_coverage is not None:
        seed_coverage=Path(seed_coverage)
        old=json.loads(seed_coverage.read_text());old_report=json.loads((seed_coverage.parent/'grip_motion.json').read_text())
        if (old_report['source_model_sha256']!=meta['model_sha256']or
                old_report['source_geometry_sha256']!=meta['preview_geometry_sha256']or
                (layout_key!='side' and not np.allclose([b['position']+b['dimensions'] for b in old['obstacle_boxes']],
                                [b['position']+b['dimensions'] for b in boxes],atol=1e-10,rtol=0))):
            raise ValueError('Pose seed fixture differs from current conveyor')
        # An old pose can seed IK after changing the parked posture, but it is
        # accepted only after fresh working-posture and collision checks.
        prior=old['grid']
    rows = []
    solutions = {}
    side_search=None
    if profile=='near' and layout_key=='side':
        from preview_side_search import SideTargetSearch
        side_search=SideTargetSearch(planner,nominal,prior)
    for side in ('right','left'):
        rotation = (Rotation.from_euler('z',180,degrees=True).as_matrix()@nominal
                    if side == 'left' else nominal)
        seed = q[2:8].copy()
        seed_lift=q[1:8].copy()
        for x in xs:
            for y in ys:
                # Keep samples inside the requested upper face. The IK's
                # 2mm stopping tolerance must never enlarge the red region.
                for z in zs:
                    centre = np.array([x if side=='right' else -x,y,z])
                    row = dict(side=side,target_capture_m=centre.tolist(),reachable=False,
                               grip_cycle_checked=False)
                    index = len(rows)
                    last_reason = 'No bounded IK solution'
                    planner.stats.pop('last_collision',None)
                    if side_search is not None:
                        state,orient,details=side_search.solve(centre,side,index,regions[side])
                        row.update(details)
                        last_reason=row.get('reason',last_reason)
                        if state is not None:solutions[index]=(state,orient)
                    elif profile=='near':
                        alternative=q[1:8].copy();alternative[2]*=-1;alternative[3]*=-1
                        seeds=[seed_lift.copy(),q[1:8].copy(),alternative]
                        if planner.posture_guard:
                            for elbow in (-110.,110.):
                                branch=q[1:8].copy()
                                branch[1]=q[2]+(math.pi if side=='left'else 0.)
                                branch[2]=math.radians(110. if elbow<0 else -110.)
                                branch[3]=math.radians(elbow)
                                branch[4]=math.radians(-90. if elbow<0 else 90.)
                                seeds.append(branch)
                        old=prior[index]if prior else None
                        if old is not None and old['reachable'] and 'positions' in old:
                            np.testing.assert_allclose(old['target_capture_m'],centre,atol=1e-12)
                            old_state=np.asarray(old['positions']);full=q.copy();full[1:10]=old_state
                            if planner.valid_full(full)and np.linalg.norm(capture(planner.data,planner.tool_id)-centre)<.002:
                                row.update(old,grip_cycle_checked=False);row.pop('cycle_rejection',None)
                                orient=(np.array(old['rotation_world']) if 'rotation_world'in old else
                                    Rotation.from_euler('z',old['yaw_deg'],degrees=True).as_matrix()@rotation@
                                    Rotation.from_euler('y',old.get('roll_deg',0.),degrees=True).as_matrix())
                                solutions[index]=(old_state,orient);seed_lift=old_state[:7].copy()
                        choices=[(roll,yaw)for roll in (0.,180.)for yaw in (0.,-14.,14.)]
                        for roll,yaw in ([]if row['reachable']else choices):
                            orient=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@rotation@Rotation.from_euler('y',roll,degrees=True).as_matrix()
                            for pose_seed in seeds:
                                planner.fixed=q.copy();planner.fixed[8:10]=OPEN_JAWS
                                try:
                                    state7=planner.ik_with_lift(dict(position=centre-orient@CAPTURE_CENTER,
                                        quaternion_xyzw=Rotation.from_matrix(orient).as_quat()),pose_seed)
                                    actual=capture(planner.data,planner.tool_id);state=np.r_[state7,OPEN_JAWS]
                                    if np.any(actual<regions[side][0])or np.any(actual>regions[side][1]):
                                        raise PlanningError('Measured grip centre outside near-side region')
                                    row.update(reachable=True,measured_capture_m=actual.tolist(),
                                        lift_height_m=float(state[0]),ik_error_m=float(np.linalg.norm(actual-centre)),
                                        yaw_deg=yaw,roll_deg=roll,positions=state.tolist())
                                    solutions[index]=(state,orient);seed_lift=state7.copy();seed=state7[1:].copy()
                                    break
                                except PlanningError as error:last_reason=str(error)
                            if row['reachable']:break
                    for lift in (()if profile=='near'else dict.fromkeys([float(np.clip(z-.15,.10,.75)),.75,.45,.15])):
                        planner.fixed = q.copy();planner.fixed[1]=lift;planner.fixed[8:10]=OPEN_JAWS
                        # Probe both elbow branches rather than declaring a
                        # point unreachable from one local Jacobian seed.
                        seeds=[seed,q[2:8].copy()]
                        alternative=seed.copy();alternative[2]*=-1;alternative[1]*=-1
                        seeds.append(alternative)
                        for pose_seed in seeds:
                            try:
                                arm = planner.ik(dict(position=centre-rotation@CAPTURE_CENTER,
                                    quaternion_xyzw=Rotation.from_matrix(rotation).as_quat()),pose_seed)
                                actual = capture(planner.data,planner.tool_id)
                                state = np.r_[lift,arm,OPEN_JAWS]
                                row.update(reachable=True,measured_capture_m=actual.tolist(),
                                    lift_height_m=lift,ik_error_m=float(np.linalg.norm(actual-centre)),
                                    yaw_deg=0.,positions=state.tolist())
                                solutions[index]=(state,rotation)
                                seed=arm.copy()
                                break
                            except PlanningError as error:last_reason=str(error)
                        if row['reachable']:break
                    if not row['reachable']:
                        row['reason']=last_reason
                        if 'last_collision' in planner.stats:row['last_collision']=dict(planner.stats['last_collision'])
                    rows.append(row)
            print(f'{layout_key} {profile} scan {side} X={x:.3f}: {sum(r["reachable"] for r in rows)}/{len(rows)} reachable',flush=True)
    def save_coverage():
        (output/'grip_coverage.json').write_text(json.dumps(dict(
            regions={k:v.tolist()for k,v in regions.items()},profile=profile,grid=rows,
            obstacle_boxes=boxes,physics_steps=0,
            posture_policy=planner.posture_guard.policy if planner.posture_guard else None,
            failure_meaning='No solution in bounded search; not proof of geometric impossibility'),indent=2)+'\n')
    save_coverage()
    segments=[]
    def segment(points,phase,side,index=None,route=None):
        points=np.asarray(points)
        def within_region():
            if phase not in ('approach','close','open','retreat')or index is None:return True
            orient=planner.data.xmat[planner.tool_id].reshape(3,3)
            expected=nominal if side=='right'else Rotation.from_euler('z',180,degrees=True).as_matrix()@nominal
            delta=orient@expected.T
            if abs(math.degrees(math.atan2(delta[1,0],delta[0,0])))>15.:return False
            actual=capture(planner.data,planner.tool_id)
            return phase not in ('close','open')or (np.all(actual>=regions[side][0])and np.all(actual<=regions[side][1]))
        for a,b in zip(points[:-1],points[1:]):
            if not checked_edge(planner,a,b,within_region):raise PlanningError('Grip cycle edge/heading/region rejected')
        measured=[]
        for state in (points[0],points[-1]):
            full=q.copy();full[1:10]=state
            if not planner.valid_full(full):raise PlanningError('Grip cycle endpoint rejected')
            measured.append(capture(planner.data,planner.tool_id).tolist())
        return dict(points=points,duration_s=segment_duration(points),phase=phase,side=side,
            coverage_index=index,connector_planner=route,entry=not segments,
            start_capture_m=measured[0],target_capture_m=measured[1])
    current=q[1:10].copy()
    opened=current.copy();opened[7:]=OPEN_JAWS
    segments.append(segment([current,opened],'open','right'))
    current=opened
    # Prefer spread-out targets, with up to eight complete cycles per side.
    for side in ('right','left'):
        if side == 'left':
            # A short joint-distance choice can rotate behind the cabinet.
            # Return along the already audited right-side transfer edges,
            # then rotate the folded arm toward the clear front of the cart.
            right_transfers=[s for s in segments if s['phase']=='transfer']
            for old in right_transfers[::-1]:
                points=old['points'][::-1].copy()
                np.testing.assert_allclose(current,points[0],atol=1e-10)
                segments.append(segment(points,'transfer','left'))
                current=points[-1].copy()
            np.testing.assert_allclose(current,opened,atol=1e-10)
            if not planner.posture_guard:
                for sign in (1,-1):
                    swept=current.copy();swept[1]+=sign*math.pi
                    if checked_edge(planner,current,swept):break
                else:raise PlanningError('No collision-free folded-arm side change')
                segments.append(segment([current,swept],'transfer','left',route='Audited folded-base half turn'))
                current=swept
        candidates=[i for i in solutions if rows[i]['side']==side]
        visited=[];completed=0
        cycle_limit=8 if profile=='wide'else len(candidates)
        while candidates and completed<cycle_limit:
            if not visited:
                first=np.array([.35,-.10,.9])if side=='right'else np.array([-.75,-.10,1.04])
                index=min(candidates,key=lambda i:np.linalg.norm(np.array(rows[i]['target_capture_m'])-first))
            else:
                index=max(candidates,key=lambda i:min(np.linalg.norm(
                    np.array(rows[i]['target_capture_m'])-np.array(rows[j]['target_capture_m']))for j in visited))
            candidates.remove(index);visited.append(index)
            state,rotation=solutions[index];centre=np.array(rows[index]['target_capture_m'])
            try:
                state=nearest_joint_turns(state,current,planner.bounds)
                approach=approach_path(planner,state,rotation,centre,approach_distance)
                transfer,route=joint_connection(planner,current,approach[0])
                closed=approach[-1].copy();closed[7:]=CLOSED_JAWS
                additions=[segment(transfer,'transfer',side,index,route),
                    segment(approach,'approach',side,index),
                    segment([approach[-1],closed],'close',side,index),
                    segment([closed,approach[-1]],'open',side,index),
                    segment(approach[::-1],'retreat',side,index)]
                segments.extend(additions);current=approach[0].copy();completed+=1
                rows[index].update(grip_cycle_checked=True,transfer_planner=route,
                    approach_distance_m=approach_distance,closed_gap_m=.036)
                print(f'{layout_key} checked grip {side} {completed}: {centre.round(3)}',flush=True)
            except PlanningError as error:
                rows[index]['cycle_rejection']=str(error)
            save_coverage()
    return finish_grip_motion(planner,q,meta,installation,boxes,regions,rows,segments,
        nominal,output,started,profile,layout_key,approach_distance)


def finish_grip_motion(planner,q,meta,installation,boxes,regions,rows,segments,
                       nominal,output,started,profile,layout_key,approach_distance,*,start_search=None):
    """Export only exact stored paths that pass a fresh, dense independent audit."""
    require_continuous_segments(segments,q)
    (output/'grip_coverage.json').write_text(json.dumps(dict(
        regions={k:v.tolist()for k,v in regions.items()},profile=profile,grid=rows,
        obstacle_boxes=boxes,physics_steps=0,
        posture_policy=planner.posture_guard.policy if planner.posture_guard else None,
        failure_meaning='No solution in bounded search; not proof of geometric impossibility'),indent=2)+'\n')
    summary={side:dict(tested=sum(r['side']==side for r in rows),
        pose_solutions=sum(r['side']==side and r['reachable']for r in rows),
        complete_grip_cycles=sum(r['side']==side and r['grip_cycle_checked']for r in rows))for side in regions}
    if any(s['complete_grip_cycles']==0 for s in summary.values()):
        raise ValueError(f'No complete grip cycle on one marked side: {summary}')
    candidate_file=output/'grip_candidate_trajectory.npz'
    np.savez_compressed(candidate_file,positions=np.concatenate([s['points']for s in segments]),
        segment_offsets=np.r_[0,np.cumsum([len(s['points'])for s in segments])])
    (output/'grip_candidate_segments.json').write_text(json.dumps(
        [{k:v for k,v in s.items()if k!='points'}for s in segments],indent=2)+'\n')
    # Independently re-audit the exact stored trajectory, including the new
    # obstacles, both jaws, and transfer movements between the two sides.
    checks=0;worst=0.;max_arm=0.;max_lift=0.;max_jaw=0.;max_heading=0.;max_entry=0.;lifts=[]
    if planner.posture_guard:planner.posture_guard.reset_audit()
    for s in segments:
        for a,b in zip(s['points'][:-1],s['points'][1:]):
            n=state_steps(a,b)
            max_arm=max(max_arm,float(np.linalg.norm(b[1:7]-a[1:7])/n))
            max_lift=max(max_lift,float(abs(b[0]-a[0])/n))
            max_jaw=max(max_jaw,float(np.max(np.abs(b[7:]-a[7:]))/n))
            for u in np.linspace(0.,1.,n+1):
                state=a+u*(b-a);full=q.copy();full[1:10]=state
                if not planner.valid_full(full):raise ValueError('Independent bilateral path audit failed')
                checks+=1;lifts.append(float(state[0]))
                if s['phase'] in ('approach','close','open','retreat')and s.get('coverage_index')is not None:
                    actual=capture(planner.data,planner.tool_id);region=regions[s['side']]
                    # The 70mm pregrasp connector may lie toward the chassis;
                    # only the grip itself is required inside the drawn region.
                    if s['phase']in('close','open')and (np.any(actual<region[0])or np.any(actual>region[1])):
                        raise ValueError('Gripping centre outside the marked side')
                    expected=nominal if s['side']=='right'else Rotation.from_euler('z',180,degrees=True).as_matrix()@nominal
                    delta=planner.data.xmat[planner.tool_id].reshape(3,3)@expected.T
                    max_heading=max(max_heading,abs(math.degrees(math.atan2(delta[1,0],delta[0,0]))))
                    actual_forward=planner.data.xmat[planner.tool_id].reshape(3,3)[:,1]
                    max_entry=max(max_entry,math.degrees(math.acos(float(np.clip(np.dot(actual_forward,expected[:,1]),-1.,1.)))))
                for c in planner.data.contact:
                    if planner.robot_geoms[int(c.geom1)]or planner.robot_geoms[int(c.geom2)]:
                        worst=max(worst,-float(c.dist))
    if max_heading>15.:raise ValueError('Bilateral grip heading exceeds 15 degrees')
    if max_entry>15.:raise ValueError('Bilateral grip entry direction exceeds the 15 degree cone')
    candidate_file.replace(output/'grip_trajectory.npz')
    report=dict(schema='s300-v4-bilateral-grip-preview-v1',source_model_sha256=meta['model_sha256'],
        source_initial_sha256=meta['initial_sha256'],source_geometry_sha256=meta['preview_geometry_sha256'],
        **installation,regions={k:v.tolist()for k,v in regions.items()},summary=summary,
        profile=profile,layout_key=layout_key,
        nominal_tool_rotation_world=np.asarray(nominal).tolist(),
        posture_policy=planner.posture_guard.policy if planner.posture_guard else None,
        posture_audit=dict(planner.posture_guard.audit) if planner.posture_guard else None,
        obstacle_boxes=boxes,obstacle_scope=('Bin, lift mast, sliced CAD feet, moving carriage and side adapter; disconnected chassis CAD bounded per component; motor and horizontal plate removed; native robot self contacts retained'
            if layout_key=='side' else 'Bin walls/floor/ribs, conservative lift CAD AABBs and moving motor AABB; native robot/chassis contacts retained'),
        initial_positions=q.tolist(),capture_center_tool_m=CAPTURE_CENTER.tolist(),
        capture_half_size_tool_m=[.048,.079,.042],accepted_path_checks=checks,
        maximum_sampled_penetration_m=max(0.,worst),penetration_limit_m=PENETRATION_LIMIT,
        maximum_audit_joint_step_rad=max_arm,maximum_audit_lift_step_m=max_lift,
        maximum_audit_jaw_step_m=max_jaw,maximum_abs_heading_deg=max_heading,
        maximum_entry_deviation_deg=max_entry,
        visited_lift_bounds_m=[min(lifts),max(lifts)],
        physics_steps=0,grasp_force_or_fruit_retention_validated=False,
        approach_distance_m=approach_distance,test_target_radius_m=.018,
        loop='Forward/reverse of audited motions; reverse labels swap approach/retreat and close/open',
        scope='Kinematic reach/entry/jaw-cycle check, not Newton dynamics or physical fruit grasp validation',
        trajectory_file='grip_trajectory.npz',trajectory_sha256=sha256(output/'grip_trajectory.npz'),
        coverage_file='grip_coverage.json',coverage_sha256=sha256(output/'grip_coverage.json'),
        segments=[{k:v for k,v in s.items()if k!='points'}for s in segments],planning_seconds=time.monotonic()-started)
    if start_search is not None:report['start_search']=start_search
    motion=StorageGripMotion(segments,report)
    report['loop_duration_s']=motion.loop_duration
    (output/'grip_motion.json').write_text(json.dumps(report,indent=2)+'\n')
    return motion


def load_storage_motion(path,meta,source,initial,displayed,scene,bin_parts,validation_output,*,require_posture_guard=False):
    path=Path(path)
    report=json.loads(path.read_text())
    layout_key=report.get('layout_key')
    guarded=require_posture_guard or layout_key in ('conveyor','side')
    expected_policy=side_posture_policy(scene,meta.get('gripper_camera_mount') if meta else None) if layout_key=='side' else POSTURE_POLICY
    if guarded and report.get('posture_policy')!=expected_policy:
        raise ValueError('Grip plan lacks the current camera/mast/working-posture audit; replan required')
    for field,expected in [('source_model_sha256',meta['model_sha256']),
                           ('source_initial_sha256',meta['initial_sha256']),
                           ('source_geometry_sha256',meta['preview_geometry_sha256'])]:
        if report[field]!=expected:raise ValueError(f'Storage preview source mismatch: {field}')
    for kind in ('trajectory','coverage'):
        if sha256(path.parent/report[kind+'_file'])!=report[kind+'_sha256']:
            raise ValueError('Storage preview audited artifact SHA mismatch: '+kind)
    np.testing.assert_allclose(initial['qpos'],report['initial_positions'],atol=1e-12,rtol=0)
    validation_output=Path(validation_output);validation_output.mkdir(exist_ok=False)
    model,installation=collision_model(source,initial['qpos'],meta,displayed,validation_output,layout_key=layout_key)
    if installation['collision_patch_sha256']!=report['collision_patch_sha256']:
        raise ValueError('Storage preview installation differs from audited mount')
    boxes=layout_boxes(scene,meta,displayed,bin_parts,float(initial['qpos'][1]),layout_key=layout_key)
    if json.loads(json.dumps(boxes))!=report['obstacle_boxes']:
        raise ValueError('Storage preview obstacles differ from audited bin/lift/motor')
    obstacles=BoxObstacles(model,initial['qpos'],boxes)
    planner=GripPlanner(model,initial,obstacles,posture_guard=guarded,posture_policy=expected_policy,
        model_sha256=installation['collision_configuration_sha256'])
    arrays=np.load(path.parent/report['trajectory_file'],allow_pickle=False)
    points=arrays['positions'];offsets=arrays['segment_offsets']
    segments=[dict(s,points=points[offsets[i]:offsets[i+1]])for i,s in enumerate(report['segments'])]
    require_continuous_segments(segments,initial['qpos'])
    for segment in segments:
        def within_goal():
            if ('nominal_tool_rotation_world' not in report or
                    segment['phase'] not in ('approach','close','open','retreat')or
                    segment.get('coverage_index') is None):return True
            expected=np.array(report['nominal_tool_rotation_world'])
            if segment['side']=='left':expected=Rotation.from_euler('z',180,degrees=True).as_matrix()@expected
            actual=planner.data.xmat[planner.tool_id].reshape(3,3)
            angle=math.degrees(math.acos(float(np.clip(np.dot(actual[:,1],expected[:,1]),-1.,1.))))
            if angle>15.:return False
            if segment['phase'] in ('close','open'):
                centre=capture(planner.data,planner.tool_id);region=np.array(report['regions'][segment['side']])
                return np.all(centre>=region[0])and np.all(centre<=region[1])
            return True
        for a,b in zip(segment['points'][:-1],segment['points'][1:]):
            if not checked_edge(planner,a,b,within_goal):
                raise ValueError('Loaded grip trajectory failed fresh collision audit')
    (validation_output/'loaded_plan_validation.json').write_text(json.dumps(dict(
        source=str(path.resolve()),trajectory_sha256=report['trajectory_sha256'],
        installation_matches=True,obstacles_match=True,fresh_edge_audit_passed=True,
        state_checks=planner.stats.get('state_checks',0),physics_steps=0,
        posture_policy=planner.posture_guard.policy if guarded else None,
        posture_audit=dict(planner.posture_guard.audit) if guarded else None),indent=2)+'\n')
    return StorageGripMotion(segments,report)
