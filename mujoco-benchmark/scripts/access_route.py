"""Bounded collision-aware access planning, not a contact/hooking controller.

OMPL RRTConnect supplies the planner. Existing FCL and native MuJoCo geoms
supply validity through callbacks. Every returned edge is independently
rechecked; exact endpoint, bounds and timed-command checks are mandatory.
The scene is frozen. Discrete checks are not swept-volume or hardware safety.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable
import numpy as np


class SearchBudget(RuntimeError):
    pass


@dataclass
class Budget:
    seconds: float = 60.0
    max_checks: int = 30000

    def __post_init__(self):
        if isinstance(self.seconds, bool) or not math.isfinite(self.seconds) or self.seconds <= 0:
            raise ValueError('Positive finite seconds required')
        if isinstance(self.max_checks, bool) or not isinstance(self.max_checks, int) or self.max_checks < 1:
            raise ValueError('Positive integer check budget required')
        self.started = time.monotonic()
        self.checks = 0

    def tick(self):
        if self.expired():
            raise SearchBudget('time_or_state_check_budget')
        self.checks += 1

    def expired(self):
        return self.checks >= self.max_checks or time.monotonic() - self.started >= self.seconds

    def remaining(self):
        return max(0.0, self.seconds - (time.monotonic() - self.started))


def finite_vector(value, size=None):
    q = np.asarray(value, dtype=float)
    if q.ndim != 1 or not len(q) or (size is not None and len(q) != size) or not np.isfinite(q).all():
        raise ValueError('Expected a finite joint vector')
    return q


def checked_edge(a, b, steps, valid: Callable, *, include_start=True):
    a = finite_vector(a)
    b = finite_vector(b, len(a)); steps = finite_vector(steps, len(a))
    if np.any(steps <= 0):
        raise ValueError('Positive per-joint sampling steps required')
    n = max(1, int(math.ceil(float(np.max(np.abs(b-a)/steps)))))
    for k in range(0 if include_start else 1, n+1):
        if not valid(a+(b-a)*(k/n)):
            return False
    return True


def checked_path(path, steps, valid):
    q = np.asarray(path, dtype=float)
    if q.ndim != 2 or len(q) < 2 or not np.isfinite(q).all():
        raise ValueError('At least two finite path states required')
    return all(checked_edge(a, b, steps, valid) for a, b in zip(q[:-1], q[1:]))


def timed_polyline(path, velocity_limits, dt=1/60):
    """Cubic stop-at-vertex timing preserves each collision-checked line.

    Not jerk-optimal or an acceleration/torque guarantee. Do not fit a spline
    through these vertices without independently validating the new geometry.
    """
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError('Invalid polyline')
    speed = finite_vector(velocity_limits, path.shape[1])
    if np.any(speed <= 0) or not math.isfinite(dt) or dt <= 0:
        raise ValueError('Positive speed and sample interval required')
    commands = [path[0].copy()]
    for a, b in zip(path[:-1], path[1:]):
        duration = max(1.5*float(np.max(np.abs(b-a)/speed)), dt)
        n = max(1, int(math.ceil(duration/dt)))
        u = np.arange(1, n+1, dtype=float)/n
        u = u*u*(3-2*u)
        commands.extend(a+(b-a)*x for x in u)
    result = np.asarray(commands)
    if (np.abs(np.diff(result, axis=0))/dt > speed+1e-8).any():
        raise RuntimeError('Retiming exceeded declared velocity limit')
    return result


def ik_options(kin, position, rotation, start, valid, steps, *, seeds=8, max_solutions=4,
               seed=0, position_tolerance_m=.002, rotation_tolerance_rad=.03, budget=None):
    """Keep several distinct, collision-valid solutions rather than one warm start.

    Residuals are recomputed with FK. Seed exhaustion is not an unreachable
    certificate. Each expensive IK call is bounded by its existing solver;
    the wall budget is checked before and after it, not by interrupting it.
    """
    start = finite_vector(start); lo, hi = np.asarray(kin.bounds, dtype=float)
    steps = finite_vector(steps, len(start))
    if lo.shape != start.shape or hi.shape != start.shape or np.any(hi <= lo):
        raise ValueError('Finite, ordered scalar-joint bounds required')
    if not np.isfinite([lo, hi]).all() or np.any(start < lo) or np.any(start > hi) or np.any(steps <= 0):
        raise ValueError('Invalid bounds/start/resolution')
    if isinstance(seeds, bool) or not isinstance(seeds, int) or not 1 <= seeds <= 64 or isinstance(max_solutions, bool) or not isinstance(max_solutions, int) or not 1 <= max_solutions <= seeds:
        raise ValueError('Bound seeds/solutions to 1..64')
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError('Nonnegative seed required')
    rng = np.random.default_rng(seed)
    guesses = [start.copy()]
    if seeds > 1: guesses.append((lo+hi)*.5)
    guesses.extend(rng.uniform(lo+1e-7, hi-1e-7) for _ in range(max(0, seeds-len(guesses))))
    accepted, records = [], []
    for index, guess in enumerate(guesses):
        if budget is not None and budget.expired(): raise SearchBudget('ik_wall_or_check_budget')
        sol, _, _ = kin.ik(position, rotation, guess)
        sol = finite_vector(sol, len(start))
        actual_p, actual_r = kin.fk(sol)
        pe = float(np.linalg.norm(np.asarray(actual_p)-position))
        re = float((actual_r*rotation.inv()).magnitude())
        record = dict(seed_index=index, position_error_m=pe, rotation_error_rad=re)
        if budget is not None and budget.expired(): raise SearchBudget('ik_wall_or_check_budget')
        if not np.isfinite([pe, re]).all(): raise ValueError('Nonfinite IK/FK residual')
        if np.any(sol < lo) or np.any(sol > hi) or pe > position_tolerance_m or re > rotation_tolerance_rad:
            record['status'] = 'residual_or_bounds_rejected'
        elif any(np.max(np.abs(sol-q)/steps) <= 1.0 for q in accepted):
            record['status'] = 'duplicate_solution'
        elif not valid(sol):
            record['status'] = 'goal_collision'
        else:
            accepted.append(sol.copy()); record['status'] = 'collision_valid_goal'
            record['joints'] = sol.tolist()
        records.append(record)
        if len(accepted) >= max_solutions: break
    # Prefer nearby branches without erasing alternatives.
    accepted.sort(key=lambda q: float(np.linalg.norm((q-start)/(hi-lo))))
    return accepted, records


def ompl_connect(start, goal, bounds, steps, valid, *, seconds=10.0, seed=0):
    """Pinned OMPL 2.x Nanobind API. Approximate solutions are NEVER returned.

    Each call is normally isolated in a CLI worker. OMPL's RNG seed is global;
    repeated calls in one process do not promise bitwise RNG reproducibility.
    The returned states, seed, version and final audit are the replay evidence.
    """
    from ompl import base as ob, geometric as og, util as ou
    from importlib.metadata import version
    start = finite_vector(start); goal = finite_vector(goal, len(start))
    lo, hi = np.asarray(bounds, dtype=float); steps = finite_vector(steps, len(start))
    if lo.shape != start.shape or hi.shape != start.shape or not np.isfinite([lo,hi]).all() or np.any(hi <= lo):
        raise ValueError('Invalid joint bounds')
    if np.any(steps <= 0) or not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('Invalid planning budget/resolution')
    if np.any(start < lo) or np.any(start > hi) or np.any(goal < lo) or np.any(goal > hi):
        raise ValueError('Start/goal out of joint bounds')
    if not valid(start) or not valid(goal):
        return None, dict(status='invalid_start_or_goal', planner='OMPL RRTConnect')
    if checked_edge(start, goal, steps, valid):
        return np.array([start, goal]), dict(status='exact', planner='checked_direct_connection', seed=seed)
    # Normalize by joint range, not raw mixed metres and radians.
    span = hi-lo
    space = ob.RealVectorStateSpace(len(start))
    limits = ob.RealVectorBounds(len(start)); limits.setLow(0.); limits.setHigh(1.)
    space.setBounds(limits)
    # An L2 edge segment this small bounds EVERY coordinate change by steps.
    length = float(np.min(steps/span))
    space.setLongestValidSegmentFraction(min(.01, length/math.sqrt(len(start))))
    ss = og.SimpleSetup(space)
    faults = []
    def is_valid(state):
        if faults: return False
        try:
            q = lo+span*np.asarray([state[i] for i in range(len(start))])
            return bool(valid(q))
        except Exception as exc:
            faults.append(exc)
            return False
    ss.setStateValidityChecker(is_valid)
    a = space.allocState(); b = space.allocState()
    for i in range(len(start)):
        a[i] = float((start[i]-lo[i])/span[i]); b[i] = float((goal[i]-lo[i])/span[i])
    ss.setStartAndGoalStates(a, b, 1e-9)
    # Seeding before constructing the planner is supported by OMPL; ownership
    # and callback references remain alive until all queries finish.
    ou.RNG.setSeed(int(seed))
    planner = og.RRTConnect(ss.getSpaceInformation()); planner.setRange(.15)
    ss.setPlanner(planner)
    solved = ss.solve(float(seconds))
    if faults: raise faults[0]
    if not solved or not ss.haveExactSolutionPath():
        return None, dict(status='not_found_within_budget', planner='OMPL RRTConnect', version=version('ompl'))
    raw = ss.getSolutionPath()
    path = np.array([lo+span*np.array([raw.getState(k)[i] for i in range(len(start))])
                     for k in range(raw.getStateCount())])
    if np.max(np.abs(path[0]-start)) > 1e-8 or np.max(np.abs(path[-1]-goal)) > 1e-7:
        raise RuntimeError('Planner did not reach requested exact endpoint')
    path[0] = start; path[-1] = goal
    if not checked_path(path, steps, valid):
        raise RuntimeError('Independent final edge validation failed')
    return path, dict(status='exact', planner='OMPL RRTConnect', version=version('ompl'),
                      seed=seed, vertices=len(path), normalized_resolution=length)


class SceneValidity:
    """Reuses environment_preflight.MuJoCoScene plus the original FCL checker."""
    def __init__(self, scene, self_checker, policy, bounds, budget, phases=('preapproach','approach')):
        self.scene, self.self_checker, self.policy = scene, self_checker, policy
        self.lo, self.hi = np.asarray(bounds, dtype=float)
        self.budget, self.phases = budget, tuple(phases)
        self.last_blocker = None
        if not scene.robot_names or not scene.environment_names:
            raise ValueError('Missing collision inventory')
        self.permissions = {}
        for row in policy.allowed_contacts:
            if row['robot_geom'] not in scene.robot_names or row['environment_geom'] not in scene.environment_names:
                raise ValueError('Unknown contact identity')
            self.permissions.setdefault((row['robot_geom'],row['environment_geom']),set()).update(row['phases'])

    def __call__(self, q):
        from environment_preflight import nearby_indices
        self.budget.tick(); q = finite_vector(q, len(self.lo))
        self.last_blocker = None
        if np.any(q < self.lo) or np.any(q > self.hi):
            self.last_blocker = dict(kind='joint_bounds'); return False
        hit = self.self_checker.check(q)
        if hit:
            self.last_blocker = dict(kind='self_collision', pair=str(hit)); return False
        s = self.scene; s.set_robot(q)
        horizon = self.policy.clearance_m+.01
        for i, name in enumerate(s.robot_names):
            indices = nearby_indices(s.robot_positions[i], s.robot_radii[i],
                                     s.environment_positions, s.environment_radii, horizon)
            for j in indices:
                other = s.environment_names[j]
                if self.phases and set(self.phases) <= self.permissions.get((name,other),set()): continue
                distance, segment = s.distance(i, int(j), horizon)
                if not math.isfinite(distance) or not np.isfinite(segment).all():
                    raise ValueError('Nonfinite native distance')
                if distance <= self.policy.clearance_m:
                    self.last_blocker = dict(kind='environment_collision', robot_geom=name,
                                             environment_geom=other, distance_m=float(distance))
                    return False
        return True
