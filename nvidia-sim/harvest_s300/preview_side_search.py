"""Finite lift sweep for the V6 camera-up, near-side gripping preview."""
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation

from preview_grip_workspace import OPEN_JAWS, PlanningError, nearest_joint_turns
from preview_workspace_motion import CAPTURE_CENTER, capture
from preview_start_search import RigidToolProbe, entry_rotation, make_cycle


class SideTargetSearch:
    def __init__(self, planner, nominal, prior, *, orientations=None, lift_ik_first=False, approach_distance=.04,
                 endpoint_accept=None,max_cartesian_step=None):
        self.planner, self.nominal = planner, nominal
        self.orientations = ((0.,0.),(-14.,0.),(14.,0.),(0.,-14.),(0.,14.)) if orientations is None else tuple(orientations)
        self.lift_ik_first = lift_ik_first
        self.approach_distance = approach_distance
        self.endpoint_accept = endpoint_accept
        self.max_cartesian_step = max_cartesian_step
        # Old files are IK hints only. Choose legal equivalent turns before
        # seeding IK; never wrap an already connected trajectory for playback.
        self.prior = []
        for row in prior:
            if not row.get('reachable') or 'positions' not in row:
                continue
            try:
                state = nearest_joint_turns(np.asarray(row['positions']),
                                            planner.initial[1:10], planner.bounds)
            except PlanningError:
                continue
            self.prior.append(dict(row, positions=state.tolist()))
        self.rigid = RigidToolProbe(planner.model, planner.initial, planner.obstacles, planner.tool_id)
        self.previous = {}

    def solve(self, centre, side, index, region, *, approach_distance=None):
        planner = self.planner
        # A continuous path cannot switch elbow sign while respecting the
        # 20..140 degree working range. Search the actual starting family.
        branch = int(np.sign(planner.initial[4]))
        nearby = sorted([r for r in self.prior if r['side']==side and
                         int(np.sign(r['positions'][3]))==branch],
                        key=lambda r: np.linalg.norm(np.asarray(r['target_capture_m'])-centre))
        seeds = []
        if side in self.previous:
            seeds.append(self.previous[side])
        if self.lift_ik_first:seeds.append(planner.initial[1:8].copy())
        seeds += [np.asarray(r['positions'][:7]) for r in nearby[:6]]
        if not self.lift_ik_first:seeds.append(planner.initial[1:8].copy())
        unique = []
        for state in seeds:
            if not any(np.linalg.norm(state[1:]-a[1:]) < .05 for a in unique):
                unique.append(state)
        seeds = unique[:3]
        attempts = 0
        reasons = Counter()
        blocked = []
        for pitch, yaw in self.orientations:
            orient = entry_rotation(self.nominal, side, yaw, pitch, 0.)
            goal = dict(position=centre-orient@CAPTURE_CENTER,
                        quaternion_xyzw=Rotation.from_matrix(orient).as_quat())
            rejection = self.rigid.rejection(goal['position'], orient)
            if rejection:
                blocked.append(dict(pitch_deg=pitch, yaw_deg=yaw, **rejection))
                continue
            low,high=planner.bounds[1]
            hints = [float(np.clip(r['positions'][0]+centre[2]-r['target_capture_m'][2],low,high))
                     for r in nearby[:2]]
            lifts = list(dict.fromkeys([*hints,*[h for h in (0.,.15,.30,.45,.60)if low<=h<=high],float(high)]))
            probes = ([(None,seed) for seed in seeds] if self.lift_ik_first else [])
            probes += [(lift,seed) for lift in lifts for seed in seeds]
            for lift, seed in probes:
                attempts += 1
                planner.stats.pop('last_collision',None)
                planner.stats.pop('last_posture_rejection',None)
                try:
                    state7 = (planner.ik_with_lift(goal,seed) if lift is None else
                              planner.ik_at_lift(goal,lift,seed[1:],branch))
                    if int(np.sign(state7[3])) != branch:
                        raise PlanningError('Lift IK changed elbow branch')
                    state = np.r_[state7,OPEN_JAWS]
                    if self.endpoint_accept is not None and not self.endpoint_accept(state):
                        raise PlanningError('Grasp endpoint lacks contact clearance')
                    actual = capture(planner.data,planner.tool_id)
                    if np.any(actual<region[0]) or np.any(actual>region[1]):
                        raise PlanningError('Grip centre outside the marked region')
                    from preview_grip_workspace import approach_path
                    distance=self.approach_distance if approach_distance is None else approach_distance(orient)
                    approach = approach_path(planner,state,orient,centre,distance,max_cartesian_step=self.max_cartesian_step)
                    cycle = make_cycle(planner,state,orient,centre,index,side,yaw,pitch,0.,approach=approach)
                    if self.endpoint_accept is not None and not self.endpoint_accept(cycle['closed']):
                        raise PlanningError('Closed grasp endpoint lacks contact clearance')
                    # Re-evaluate the end of insertion, not the last state
                    # visited by the independent approach/closing audits.
                    state=cycle['state'];full=planner.initial.copy();full[1:10]=state
                    if not planner.valid_full(full):
                        raise PlanningError('Grip endpoint fails posture/collision guard')
                    actual=capture(planner.data,planner.tool_id)
                    self.previous[side]=state[:7].copy()
                    return state,orient,dict(reachable=True,positions=state.tolist(),
                        measured_capture_m=actual.tolist(),lift_height_m=float(state[0]),
                        ik_error_m=float(np.linalg.norm(actual-centre)),yaw_deg=yaw,pitch_deg=pitch,
                        roll_deg=0.,rotation_world=orient.tolist(),ik_probes=attempts,
                        camera_above_grip_m=planner.posture_guard.camera_gap(planner.data),
                        approach_positions=cycle['approach'].tolist())
                except PlanningError as error:
                    rejection=planner.stats.get('last_posture_rejection')
                    contact=planner.stats.get('last_collision')
                    reason=(rejection['rule'] if rejection else
                            f"{contact['geom1']} / {contact['body2']}" if contact else str(error))
                    reasons[reason]+=1
        return None,None,dict(reachable=False,ik_probes=attempts,
            reason='No complete local grip cycle in finite camera-up lift sweep',
            search_elbow_branch=branch,rejection_counts=dict(reasons),blocked_tool_orientations=blocked)
