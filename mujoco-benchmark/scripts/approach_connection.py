"""Bounded OMPL joint-space approach connector, not a contact controller.

No physics is stepped. A path is accepted only after exact-goal and independent
per-joint edge checks. Timeout and IK exhaustion never prove infeasibility.
The adapter supplies the existing FCL + MuJoCo scene validity predicate.
OMPL 2.0.1 Python bindings are used only by ompl_connect().
"""
from __future__ import annotations
from dataclasses import dataclass
import copy
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation


class LimitReached(RuntimeError):
    """An experiment budget was exhausted; not an unreachable-target label."""


def vector(value, n=None):
    value = np.asarray(value, dtype=float)
    if value.ndim != 1 or (n is not None and value.shape != (n,)) or not np.isfinite(value).all():
        raise ValueError('Expected finite vector')
    return value.copy()


@dataclass
class CheckBudget:
    max_checks: int = 100000
    seconds: float = 120.

    def __post_init__(self):
        if (isinstance(self.max_checks, bool) or not isinstance(self.max_checks, int)
                or self.max_checks < 1 or not math.isfinite(self.seconds) or self.seconds <= 0):
            raise ValueError('Positive finite check/time budgets required')
        self.started = time.monotonic()
        self.checks = 0

    def tick(self):
        if self.checks >= self.max_checks:
            raise LimitReached('state_check_budget')
        if self.remaining() <= 0:
            raise LimitReached('wall_time_budget')
        self.checks += 1

    def remaining(self):
        return self.seconds - (time.monotonic() - self.started)


class JointSpace:
    def __init__(self, bounds, steps):
        bounds = np.asarray(bounds, float)
        if bounds.ndim != 2 or bounds.shape[0] != 2 or not np.isfinite(bounds).all():
            raise ValueError('Finite 2 x DOF bounds required')
        self.low, self.high = bounds.copy()
        self.span = self.high - self.low
        self.steps = vector(steps, len(self.low))
        if not len(self.low) or np.any(self.span <= 0) or np.any(self.steps <= 0):
            raise ValueError('Positive ranges and sampling steps required')
        self.n = len(self.low)

    def contains(self, q):
        q = vector(q, self.n)
        return bool(np.all(q >= self.low) and np.all(q <= self.high))

    def normalize(self, q):
        return (vector(q, self.n) - self.low) / self.span

    def denormalize(self, u):
        return self.low + vector(u, self.n) * self.span

    def edge_samples(self, a, b):
        a, b = vector(a, self.n), vector(b, self.n)
        n = max(1, int(np.ceil(np.max(np.abs(b-a)/self.steps))))
        for u in np.linspace(0., 1., n+1):
            yield a + (b-a)*u


def check_polyline(path, space, valid):
    """Independent discrete check, including all endpoints and edge interiors."""
    path = np.asarray(path, float)
    if path.ndim != 2 or path.shape[1] != space.n or len(path) < 1 or not np.isfinite(path).all():
        raise ValueError('Malformed path')
    edges = zip(path[:-1], path[1:]) if len(path) > 1 else [(path[0], path[0])]
    for index, (a, b) in enumerate(edges):
        for q in space.edge_samples(a, b):
            if not space.contains(q) or not valid(q):
                return dict(passed=False, edge=index, q=q.tolist())
    return dict(passed=True)


def ompl_connect(start, goal, space, valid, solve_seconds=10., range_fraction=.1):
    """Use OMPL RRTConnect; never treat approximate PlannerStatus truthiness as success.

    OMPL's resolution is derived from physical per-joint steps after normalizing
    metres/radians. No periodic wrapping is done across bounded joint limits.
    Run independent seeded comparisons in fresh processes (OMPL has global RNG).
    """
    start, goal = vector(start, space.n), vector(goal, space.n)
    if not math.isfinite(solve_seconds) or solve_seconds <= 0:
        raise ValueError('Positive solve_seconds required')
    if not math.isfinite(range_fraction) or not 0 < range_fraction <= 1:
        raise ValueError('range_fraction must be in (0,1]')
    for name, q in [('start', start), ('goal', goal)]:
        if not space.contains(q) or not valid(q):
            return dict(status='invalid_'+name, path=None, impossible=None)
    straight = check_polyline([start, goal], space, valid)
    if straight['passed']:
        return dict(status='exact', method='direct_checked', path=np.stack([start,goal]), impossible=None)
    from ompl import base as ob, geometric as og
    state_space = ob.RealVectorStateSpace(space.n)
    bounds = ob.RealVectorBounds(space.n); bounds.setLow(0.); bounds.setHigh(1.)
    state_space.setBounds(bounds)
    # Maximum extent is sqrt(n); each normalized edge component is then <= its
    # physical step limit. Final independent checks do not rely on this alone.
    resolution = min(.01, float(np.min(space.steps/space.span))/math.sqrt(space.n))
    state_space.setLongestValidSegmentFraction(resolution)
    setup = og.SimpleSetup(state_space)
    callback_errors = []
    def state_valid(state):
        try:
            if callback_errors: return False
            q = space.denormalize([state[i] for i in range(space.n)])
            return space.contains(q) and bool(valid(q))
        except Exception as exc:
            if not callback_errors: callback_errors.append(exc)
            return False
    setup.setStateValidityChecker(state_valid)
    s, g = state_space.allocState(), state_space.allocState()
    for i, x in enumerate(space.normalize(start)): s[i] = float(x)
    for i, x in enumerate(space.normalize(goal)): g[i] = float(x)
    setup.setStartAndGoalStates(s, g, 1e-9)
    planner = og.RRTConnect(setup.getSpaceInformation()); planner.setRange(range_fraction)
    setup.setPlanner(planner)
    status = setup.solve(solve_seconds)
    if callback_errors: raise callback_errors[0]
    if not setup.haveExactSolutionPath():
        return dict(status='no_exact_path_within_budget', planner_status=str(status), path=None, impossible=None)
    solution = setup.getSolutionPath()
    points = np.array([space.denormalize([state[i] for i in range(space.n)]) for state in solution.getStates()])
    if (len(points) < 2 or not np.allclose(points[0],start,atol=1e-8,rtol=0)
            or not np.allclose(points[-1],goal,atol=1e-8,rtol=0)):
        return dict(status='endpoint_mismatch', path=None, impossible=None)
    points[0], points[-1] = start, goal
    checked = check_polyline(points, space, valid)
    if not checked['passed']:
        return dict(status='postcheck_blocked', details=checked, path=None, impossible=None)
    return dict(status='exact', method='OMPL_RRTConnect', path=points,
                resolution_fraction=resolution, impossible=None)


def ik_pool(kin, position, rotation, start, space, valid, count=8, seed=0):
    """Bounded multi-start IK. Returned errors are checked against actual FK."""
    if isinstance(count,bool) or not isinstance(count,int) or not 1 <= count <= 32:
        raise ValueError('IK seed budget must be 1..32')
    p = vector(position,3); start = vector(start,space.n)
    rng = np.random.default_rng(seed)
    seeds = [start]
    if count > 1: seeds.append((space.low+space.high)/2)
    while len(seeds) < count:
        if len(seeds) % 2:
            seeds.append(rng.uniform(space.low,space.high))
        else:
            seeds.append(np.clip(start+rng.normal(size=space.n)*space.span*.18,space.low,space.high))
    accepted, records = [], []
    for index, initial in enumerate(seeds):
        q, _, _ = kin.ik(p,rotation,initial)
        q=vector(q,space.n); actual_p, actual_r=kin.fk(q)
        pe=float(np.linalg.norm(actual_p-p)); re=float((rotation*actual_r.inv()).magnitude())
        row=dict(seed_index=index, position_error_m=pe, rotation_error_rad=re, q=q.tolist())
        if not space.contains(q) or pe > .002 or re > .03:
            row['status']='ik_not_found'
        elif any(np.max(np.abs(q-old)/space.steps) < 1 for old in accepted):
            row['status']='duplicate_branch'
        elif not valid(q):
            row['status']='goal_collision'
        else:
            accepted.append(q);row['status']='valid_goal'
        records.append(row)
    accepted.sort(key=lambda q:float(np.linalg.norm((q-start)/space.span)))
    return accepted, records


def retime(path, velocity, hz=60., max_seconds=180.):
    """Smoothstep on each checked joint edge; no geometric shortcut or overshoot.

    Stops at each vertex. This is deliberately not a jerk-optimal trajectory.
    First row is the original start; final row equals the route endpoint exactly.
    """
    path=np.asarray(path,float)
    if path.ndim!=2 or len(path)<2 or not np.isfinite(path).all():raise ValueError('Malformed path')
    velocity=vector(velocity,path.shape[1])
    if np.any(velocity<=0) or not np.isfinite([hz,max_seconds]).all() or min(hz,max_seconds)<=0:
        raise ValueError('Positive timing parameters required')
    chunks=[path[:1]]; count=1
    for a,b in zip(path[:-1],path[1:]):
        if np.array_equal(a,b):continue
        n=max(2,int(math.ceil(1.5*float(np.max(np.abs(b-a)/velocity))*hz)))
        count+=n
        if (count-1)/hz>max_seconds:raise LimitReached('trajectory_duration_budget')
        u=np.linspace(0,1,n+1)[1:];u=u*u*(3-2*u)
        segment=a[None]+u[:,None]*(b-a)[None];segment[-1]=b
        chunks.append(segment)
    rows=np.concatenate(chunks)
    if len(rows)==1:rows=np.repeat(rows,2,axis=0)
    if np.any(np.abs(np.diff(rows,axis=0))*hz>velocity+1e-8):raise RuntimeError('Timing speed invariant')
    return rows


def handoff_index(params):
    """Replace ALL leading preapproach/approach poses, not merely the first edge."""
    if params.get('trajectory_mode')!='diagnostic_pose_waypoints_v1':
        raise ValueError('Only explicit existing pose-waypoint candidates are supported')
    rows=params['pose_waypoints']
    if len(rows)<2 or rows[0]['phase']!='preapproach':raise ValueError('Missing preapproach')
    i=0
    while i+1<len(rows) and rows[i+1]['phase'] in ('preapproach','approach'):i+=1
    if i==len(rows)-1:raise ValueError('Missing local manipulation suffix')
    return i


def suffix_parameters(params, q, kin):
    """Plan from reached IK branch, never teleport back to the initial branch."""
    i=handoff_index(params);p,r=kin.fk(q)
    result=copy.deepcopy(params)
    result['pose_waypoints']=[dict(phase='preapproach',ring_position_xyz=np.asarray(p).tolist(),
                                  orientation_xyzw=r.as_quat().tolist(),minimum_seconds=0.)]
    result['pose_waypoints']+=copy.deepcopy(params['pose_waypoints'][i+1:])
    return result


def stitch(prefix, suffix, phases, start, hz=60., limits=None):
    prefix=np.asarray(prefix,float);suffix=np.asarray(suffix,float);start=vector(start)
    if (prefix.ndim!=2 or suffix.ndim!=2 or prefix.shape[1]!=len(start) or suffix.shape[1]!=len(start)
            or len(prefix)<2 or len(suffix)<1 or len(phases)!=len(suffix)
            or not np.isfinite(prefix).all() or not np.isfinite(suffix).all()
            or not np.array_equal(prefix[0],start)):
        raise ValueError('Malformed or wrong-start trace')
    commands=np.concatenate([prefix,suffix])
    if limits is not None and np.any(np.abs(np.diff(commands,axis=0))*hz>vector(limits,len(start))+1e-6):
        raise ValueError('Discontinuous or over-speed join')
    tags=['ready']+['preapproach']*(len(prefix)-1)+list(phases)
    return [dict(joints=start.tolist(),command=q.tolist(),phase=p) for q,p in zip(commands,tags)]
