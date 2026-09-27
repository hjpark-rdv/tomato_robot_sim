"""Bounded, target-relative motion-family search; not an RGB-D policy.

Existing diagnostic_pose_waypoints_v1 / IK / FCL / MuJoCo remain the executor.
Families change the whole approach, not only the last seating pose. Curved
families are sampled curves (piecewise Cartesian segments in the old planner),
NOT a claim of globally smooth joint motion or collision-free trajectories.
"""
from __future__ import annotations

from collections import Counter
import copy
import hashlib
import math
import re

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.stats import qmc

FAMILIES = ('under_center', 'side_mouth', 'flank_left', 'flank_right', 'pivot_sweep')
SCHEMA = 'motion_family_search_v1'
# Experimental SEARCH ranges, not physical safety limits. Units are explicit.
RANGES = {
    'azimuth_deg': (-110., 110.), 'elevation_deg': (-30., 30.),
    'roll_deg': (-65., 65.), 'pitch_deg': (-45., 45.),
    'pre_distance_m': (.12, .20), 'under_clearance_m': (.004, .020),
    'lateral_m': (-.012, .012), 'sweep_m': (.025, .070),
    'target_fraction': (.25, .75), 'seat_gap_m': (.0002, .0012),
    'verify_m': (.002, .006), 'entry_twist_deg': (-80., 80.),
}


def vector(x, name):
    x = np.asarray(x, dtype=float)
    if x.shape != (3,) or not np.isfinite(x).all():
        raise ValueError(f'{name}: expected finite 3-vector')
    return x


def identity(x):
    if not isinstance(x, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', x):
        raise ValueError('Unsafe identity')
    return x


def digest(x):
    import json
    return hashlib.sha256(json.dumps(x, sort_keys=True, allow_nan=False).encode()).hexdigest()


def parameters(count, seed):
    if isinstance(count, bool) or not isinstance(count, int) or count < 1 or count > 64 or count & (count-1):
        raise ValueError('samples_per_family must be a power of two in 1..64')
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError('seed must be a nonnegative integer')
    unit = qmc.Sobol(d=len(RANGES)+2, scramble=True, seed=seed).random_base2(int(math.log2(count)))
    rows = []
    for u in unit:
        p = {k: float(lo+(hi-lo)*u[i]) for i, (k, (lo, hi)) in enumerate(RANGES.items())}
        p.update(target_selector=float(u[-2]), wire_selector=float(u[-1]))
        rows.append(p)
    return rows


def orientation(heading, roll, pitch):
    outward = vector(heading, 'heading').copy(); outward[2] = 0.
    if np.linalg.norm(outward) < 1e-8:
        raise ValueError('Horizontal robot-to-target reference direction is undefined')
    outward /= np.linalg.norm(outward)
    down = np.array([0., 0., -1.])
    base = Rotation.from_matrix(np.column_stack((outward, down, np.cross(outward, down))))
    # These are HOOK-local x/z rotations; not world Euler roll/pitch conventions.
    return base * Rotation.from_euler('xz', [roll, pitch], degrees=True)


def mix_rotation(a, b, u):
    return Slerp([0., 1.], Rotation.from_quat([a.as_quat(), b.as_quat()]))(float(u))


def bezier(a, b, c, d, u):
    return (1-u)**3*a + 3*(1-u)**2*u*b + 3*(1-u)*u*u*c + u**3*d


def pose(phase, p, r, seconds=0.):
    return dict(phase=phase, ring_position_xyz=vector(p, 'pose').tolist(),
                orientation_xyzw=r.as_quat().tolist(), minimum_seconds=float(seconds))


def _timed_append(rows, phase, p, r, speed=.010):
    previous = rows[-1]
    a = np.array(previous['ring_position_xyz']); ra = Rotation.from_quat(previous['orientation_xyzw'])
    # Split long translations to respect the existing <=10s waypoint contract.
    duration = max(1.5*np.linalg.norm(p-a)/speed, (r*ra.inv()).magnitude()/.15, .2)
    n = max(1, math.ceil(duration/9.))
    for j in range(1, n+1):
        u = j/n
        rows.append(pose(phase, a+(p-a)*u, mix_rotation(ra, r, u), duration/n))


def make_candidate(family, p, *, center, radius, heading, targets, rear_wires,
                   ring_radius, candidate_id, parent_id=None):
    from hook_seating_geometry import seating_goal, seating_geometry
    identity(candidate_id)
    if family not in FAMILIES:
        raise ValueError('Unknown family')
    if isinstance(radius, bool) or not np.isfinite(radius) or radius <= 0:
        raise ValueError('Positive target radius required')
    for key, (lo, hi) in RANGES.items():
        value = p[key]
        if isinstance(value, bool) or not np.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'Parameter outside declared range: {key}')
    if not targets or not rear_wires:
        raise ValueError('Missing native target/wire inventory')
    for key in ('target_selector', 'wire_selector'):
        if not 0 <= p[key] < 1:
            raise ValueError('Invalid discrete selector')
    c = vector(center, 'center')
    h = Rotation.from_euler('z', p['azimuth_deg'], degrees=True).apply(vector(heading, 'heading'))
    h[2] = 0.; h /= np.linalg.norm(h)
    up = np.array([0., 0., 1.]); side = np.cross(h, up)
    travel = math.cos(math.radians(p['elevation_deg']))*h + math.sin(math.radians(p['elevation_deg']))*up
    if 'robot_facing_mouth' in p and not isinstance(p['robot_facing_mouth'], bool):
        raise ValueError('robot_facing_mouth must be an explicit boolean')
    robot_facing_mouth = family == 'side_mouth' and p.get('robot_facing_mouth', False)
    final_r = orientation(-h if robot_facing_mouth else h, p['roll_deg'], p['pitch_deg'])
    target = targets[min(int(p['target_selector']*len(targets)), len(targets)-1)]
    wire = rear_wires[min(int(p['wire_selector']*len(rear_wires)), len(rear_wires)-1)]
    goal = seating_goal(target, wire, final_r, fraction=p['target_fraction'], surface_gap_m=p['seat_gap_m'])
    end = np.array(goal['ring_position_xyz'])
    # This field is evidence, not a candidate-generation hard success gate.
    end_geometry = seating_geometry(target, end, final_r, rear_wires,
                                   ring_radius=ring_radius, max_surface_gap_m=.0015)
    low = c-up*(radius+p['under_clearance_m']) + side*p['lateral_m']
    entry_r = orientation(h, 0., 0.)
    if family == 'side_mouth':
        entry_r = final_r * Rotation.from_euler('x', p['entry_twist_deg'], degrees=True)
        point = np.array(goal['target_point_world'])
        gate = point-entry_r.apply([ring_radius+radius+.015, 0., 0.])
        # +X is the open mouth. Point it toward the target (-h), while
        # preparing on the robot side (+h). Flipping only start would
        # approach through the closed rear arc. Default geometry is retained.
        start = gate + (travel if robot_facing_mouth else -travel)*p['pre_distance_m']
        rows = [pose('preapproach', start, entry_r)]
        _timed_append(rows, 'approach', gate, entry_r, .020)
        # The target moves from the +X opening toward the -X rear arc.
        mouth = point-entry_r.apply([.004, 0., 0.])
        _timed_append(rows, 'insert', mouth, entry_r)
    else:
        start = low+travel*p['pre_distance_m']
        rows = [pose('preapproach', start, entry_r)]
        gate = low+h*(ring_radius+radius+.018)
        _timed_append(rows, 'approach', gate, entry_r, .020)
        if family in ('flank_left', 'flank_right'):
            sign = 1. if family == 'flank_left' else -1.
            bend = side*p['sweep_m']*sign
            # Both position and orientation change during the flank sweep.
            turn_r = final_r*Rotation.from_euler('x', p['entry_twist_deg'], degrees=True)
            for u in (.25, .5, .75, 1.):
                point = bezier(gate, gate+bend, low+bend, low, u)
                _timed_append(rows, 'insert', point, mix_rotation(entry_r, turn_r, u))
            entry_r = turn_r
        else:
            _timed_append(rows, 'insert', low, entry_r)
        if family == 'pivot_sweep':
            target_local = entry_r.inv().apply(c-low)
            # Changing orientation must also move RING centre, not the wrist origin.
            for u in (.33, .66, 1.):
                r = mix_rotation(entry_r, final_r, u)
                _timed_append(rows, 'insert', c-r.apply(target_local), r)
            entry_r = final_r
        if family == 'under_center':
            # Transit and final seating are different positions; target contact is still allowed.
            _timed_append(rows, 'seat', c+up*(radius+.006), entry_r)
    _timed_append(rows, 'seat', end, final_r, .003)
    rows.append(pose('hold', end, final_r, 1.1))
    normal = final_r.apply(np.array(goal['target_point_ring'])-(wire.a+wire.b)*.5)
    normal /= np.linalg.norm(normal)
    # Search a real loading movement; never equate proximity with mechanical retention.
    _timed_append(rows, 'verify', end+normal*p['verify_m'], final_r, .003)
    rows.append(pose('verify', end+normal*p['verify_m'], final_r, .5))
    if len(rows) > 16:
        raise ValueError('Existing planner waypoint budget exceeded; candidate rejected, not silently truncated')
    return dict(candidate_id=candidate_id, trajectory_mode='diagnostic_pose_waypoints_v1',
                family=family, search_parameters=copy.deepcopy(p), parent_id=parent_id,
                pose_waypoints=rows, diagnostic_only=True, training_eligible=False,
                hook_success=None, offline_teacher=True, geometry=end_geometry,
                target_geom=target.name, selected_rear_wire=wire.name,
                action_schema='explicit_ring_pose_waypoints_v1',
                scope='SIM-GT search proposal; sampled path; requires full robot check and forward execution')


def generate_candidates(geometry, per_family, seed, *, parents=(), families=FAMILIES,
                        robot_facing_mouth=False):
    """Round-robin family order protects diversity under early budgets."""
    if not families or len(set(families)) != len(families) or not set(families) <= set(FAMILIES):
        raise ValueError('Choose unique, known motion families')
    configs = []
    for p in parameters(per_family, seed):
        for family in families:
            values = copy.deepcopy(p)
            if robot_facing_mouth and family == 'side_mouth':
                values['robot_facing_mouth'] = True
            configs.append((family, values, None))
    for parent in parents:
        if parent.get('family') not in FAMILIES:
            continue
        # Bounded refinement changes approach and orientation, not just a final millimetre.
        for sign in (-1., 1.):
            p = copy.deepcopy(parent['search_parameters'])
            for key, delta in (('azimuth_deg', 12.), ('roll_deg', 12.), ('sweep_m', .010)):
                lo, hi = RANGES[key]; p[key] = float(np.clip(p[key]+sign*delta, lo, hi))
            configs.append((parent['family'], p, parent['candidate_id']))
    candidates, rejected = [], []
    for i, (family, p, parent) in enumerate(configs):
        cid = f'mf_{seed}_{i:05d}'
        try:
            candidates.append(make_candidate(family, p, **geometry, candidate_id=cid, parent_id=parent))
        except ValueError as error:
            rejected.append(dict(candidate_id=cid, family=family, parent_id=parent,
                                 result='proposal_rejected', reason=str(error),
                                 physics_executed=False, training_eligible=False))
    return candidates, rejected


def summarize_target(records):
    """A search miss is not a proof of impossibility; keep all denominators."""
    counts = Counter(r['outcome'] for r in records)
    evidence = counts['contact_retention_evidence']
    near = counts['geometric_candidate']
    status = ('contact_evidence_found' if evidence else 'geometric_candidate_found' if near else
              'unresolved_within_budget')
    return dict(status=status, candidate_outcomes=dict(counts), candidates=len(records),
                physics_attempts=sum(bool(r.get('physics_executed')) for r in records),
                families=sorted({r['family'] for r in records}),
                impossible=None, impossibility_claim='none; numerical IK/search failures are not certificates',
                hook_success=None, training_eligible=False)


def trial_outcome(result, samples):
    if result.get('error'):
        return 'execution_error'
    if not result.get('physics_executed'):
        return ('path_blocked' if result.get('audit_status') == 'blocked' else
                'audit_only_clear' if result.get('audit_status') == 'sampled_clear' else 'audit_inconclusive')
    stops = set(result.get('stop_reasons', []))
    if stops & {'physics_penetration', 'invalid_physics'}:
        return 'physics_invalid'
    if 'forbidden_force' in stops:
        return 'other_object_contact'
    if stops:
        return 'experimental_limit_stop'
    if not result.get('completed'):
        return 'incomplete'
    if result.get('authorized_contact_evidence', {}).get('contact_retention_evidence'):
        return 'contact_retention_evidence'
    # The old 1.5mm geometric band remains a PROGRESS signal only.
    if any(r.get('seated') for r in samples):
        return 'geometric_candidate'
    return 'motion_completed_no_capture_evidence'
