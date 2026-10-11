"""Opt-in arm-only sway during the last attached fruit's existing closes.

The existing phase sampler already supports Nx7 joint waypoints. This helper
changes only those waypoints, uses the existing IK, and leaves phase timing,
gripper travel, belt commands, plant state and native break rules untouched.
Neither the IK result nor this local FK check certifies a collision-free path.
"""
import copy
import hashlib
import json
from numbers import Integral, Real

import numpy as np
from scipy.spatial.transform import Rotation


SCHEMA = 's300-last-native-fruit-arm-grasp-sway-v2'
VELOCITY_LIMITS = np.array([.08] + [.35] * 6)
WAYPOINT_COUNT = 161
SWAY_START = .75


def _digest(phases):
    return hashlib.sha256(json.dumps(phases, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _pose(kin, q):
    pose = np.asarray(kin.fk(q), dtype=float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all() or
            not np.allclose(pose[3], [0., 0., 0., 1.], atol=1e-10) or
            not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-8) or
            not np.isclose(np.linalg.det(pose[:3, :3]), 1., atol=1e-8)):
        raise ValueError('Finite proper FK pose required for last-fruit arm sway')
    return pose


def apply_last_fruit_grasp_sway(kin, phases, plan, *, amplitude_m=None,
                               fruits=None, breaks=(), group=None, hz=120,
                               joint_velocity_limits=None):
    """Return existing phase commands, optionally adding a small CAD-X sway.

    Only an exact selected fruit which is the sole native-attached fruit of its
    own truss activates the helper. ``fruits`` and ``breaks`` are the existing
    exact joint inventory; a caller-supplied Boolean does not authorize motion.
    None/zero and an ineligible selection return the *original* objects.

    The path is parameterized by the existing smoothstep phase coordinate:
    offset = 0 through s=.75, then amplitude*sin(2*pi*(s-.75)/.25).
    Uniform IK waypoints preserve its fixed
    yaw/pitch and return to the original seven-joint hold exactly. Every phase
    keeps its original duration, target gap and belt velocities. Speed is
    bounded for the actual piecewise joint interpolation, including the outer
    smoothstep's derivative <=1.5/duration; duration is never silently extended.
    """
    if amplitude_m is None:
        return phases, plan
    if (isinstance(amplitude_m, (bool, np.bool_)) or not isinstance(amplitude_m, Real)
            or not np.isfinite(amplitude_m) or not 0. <= amplitude_m <= .0015):
        raise ValueError('Arm sway amplitude must be finite 0..1.5mm')
    if amplitude_m == 0.:
        return phases, plan
    if (not isinstance(hz, Integral) or isinstance(hz, (bool, np.bool_)) or
            not 1 <= hz <= 1000):
        raise ValueError('Finite integer command verification Hz must be 1..1000')
    if not isinstance(fruits, (list, tuple)) or not fruits:
        raise ValueError('Exact native fruit-joint inventory required for arm sway')
    if any(not isinstance(f, dict) or not isinstance(f.get('name'), str) or
           not isinstance(f.get('joint'), str) or not f['joint'] for f in fruits):
        raise ValueError('Exact native fruit name and joint mappings required')
    if (len({f['name'] for f in fruits}) != len(fruits) or
            len({f['joint'] for f in fruits}) != len(fruits)):
        raise ValueError('Duplicate native fruit name or joint mapping')
    from single_remaining_aim import selected_aim
    _, _, inventory = selected_aim(fruits, breaks, group, height_mm=0., depth_mm=65.)
    if not inventory['last_remaining_fruit']:
        return phases, plan
    if (plan.get('preparation') != 'horizontal_ready' or
            plan.get('harvest_profile') != 'belt_open_close_cycle' or
            plan.get('entry_only') is not False or plan.get('pitch_deg') != 0.):
        raise ValueError('Last-fruit sway requires the existing horizontal full belt cycle')
    rows = plan.get('phases')
    if not isinstance(rows, list) or len(rows) != len(phases):
        raise ValueError('Exact executable/serialized phase mapping required')
    selected = {name: [i for i, p in enumerate(phases) if p[0] == name]
                for name in ('close', 'cycle_reclose')}
    if any(len(indices) != 1 for indices in selected.values()):
        raise ValueError('Exactly one close and reclose phase required')
    indices = [selected[name][0] for name in selected]
    if indices != sorted(indices):
        raise ValueError('Close must precede reclose')
    for phase, row in zip(phases, rows):
        if len(phase) != 6:
            raise ValueError('Existing phase must have six command fields')
        name, duration, qs, gap, left, right = phase
        if (row.get('name') != name or row.get('duration_s') != duration or
                row.get('end_gap_m') != gap or row.get('belt_local_y_m_s') != [left, right] or
                not np.array_equal(np.asarray(row.get('waypoint_q')), qs)):
            raise ValueError('Executable and saved phase commands differ')
    hold = np.asarray(phases[indices[0]][2], dtype=float)
    if (hold.ndim != 2 or hold.shape[1] != 7 or len(hold) < 2 or
            not np.isfinite(hold).all() or not np.array_equal(hold, np.tile(hold[0], (len(hold), 1)))):
        raise ValueError('Sway may wrap only an existing static seven-joint grasp hold')
    q0 = hold[0].copy()
    if not np.array_equal(np.asarray(plan.get('goal_q')), q0):
        raise ValueError('Existing grasp goal does not match closing hold')
    for i in indices:
        name, duration, qs, gap, left, right = phases[i]
        q = np.asarray(qs, float)
        if (q.ndim != 2 or q.shape[1] != 7 or len(q) < 2 or
                not np.array_equal(q, np.tile(q0, (len(q), 1))) or
                not np.isfinite([duration, gap, left, right]).all() or duration <= 0.):
            raise ValueError('Both original closing phases must hold the same finite goal')
        if (gap != plan['belt_spin']['closed_gap_m'] or left == 0. or right == 0.):
            raise ValueError('Existing grasp gap and continuous belt command required')
        rail = rows[i].get('rail_waypoint_positions_m')
        if rail is not None and (len(rail) != len(q) or not np.isfinite(rail).all() or
                                 not np.all(np.asarray(rail) == rail[0])):
            raise ValueError('Arm sway cannot change a moving rail phase')
    bounds = np.asarray(getattr(kin, 'bounds', None), float)
    caps = VELOCITY_LIMITS.copy() if joint_velocity_limits is None else np.asarray(joint_velocity_limits, float)
    if (bounds.shape != (2, 7) or not np.isfinite(bounds).all() or np.any(bounds[0] > bounds[1])
            or np.any(q0 < bounds[0]) or np.any(q0 > bounds[1]) or
            caps.shape != (7,) or not np.isfinite(caps).all() or np.any(caps <= 0.)):
        raise ValueError('Existing seven-joint bounds and positive speed caps required')
    original_pose = _pose(kin, q0)
    if np.linalg.norm(original_pose[:3, 2] - [0., 0., 1.]) > 1e-4:
        raise ValueError('Existing grasp must have horizontal yaw-only orientation')
    fixed_lift = (plan.get('ready_lift_branch') or {}).get('fixed_lift_m')
    if fixed_lift is not None:
        if (not hasattr(kin, 'ik_at_lift') or not np.isfinite(fixed_lift) or
                abs(float(fixed_lift) - q0[0]) > 1e-10):
            raise ValueError('Existing constrained lift branch cannot be changed by arm sway')
        solve = lambda pose, seed: kin.ik_at_lift(pose, seed, fixed_lift)
    else:
        solve = kin.ik
    coordinates = np.linspace(0., 1., WAYPOINT_COUNT)
    offsets = np.zeros(WAYPOINT_COUNT)
    active = coordinates > SWAY_START
    offsets[active] = float(amplitude_m) * np.sin(
        2 * np.pi * (coordinates[active] - SWAY_START) / (1. - SWAY_START))
    offsets[[0, -1]] = 0.
    path = [q0.copy()]
    errors = []
    position_tolerance = min(.002, amplitude_m * .05)
    orientation_tolerance = 1e-4
    for offset in offsets[1:-1]:
        if offset == 0.:
            path.append(q0.copy())
            errors.append([0., 0.])
            continue
        pose = original_pose.copy()
        pose[:3, 3] += original_pose[:3, 0] * offset
        q, pe, re = solve(pose, path[-1])
        q = np.asarray(q, float)
        if (q.shape != (7,) or not np.isfinite(q).all() or
                not np.isfinite([pe, re]).all() or np.any(q < bounds[0]) or np.any(q > bounds[1])
                or pe > position_tolerance or re > orientation_tolerance):
            raise RuntimeError('Last-fruit sway IK failed; branch/bounds/tolerances were not relaxed')
        path.append(q.copy())
        errors.append([float(pe), float(re)])
    path.append(q0.copy())
    path = np.asarray(path)
    phase_proofs = []
    # Verify the exact existing control interpolation. A waypoint-only check
    # would miss an IK branch discontinuity or orientation drift between rows.
    for i in indices:
        name, duration, *_ = phases[i]
        peak_bound = 1.5 * (len(path) - 1) * np.max(np.abs(np.diff(path, axis=0)), axis=0) / duration
        if np.any(peak_bound > caps + 1e-10):
            raise RuntimeError('Last-fruit sway exceeds existing segment velocity caps; duration unchanged')
        max_position_error = 0.; max_orientation_error = 0.
        measured_offsets = []
        sampled_count = int(np.ceil(duration * hz)) + 1
        for u in np.linspace(0., 1., sampled_count):
            smooth = u * u * (3 - 2 * u)
            f = smooth * (len(path) - 1); lo = min(int(f), len(path) - 2); alpha = f - lo
            q = path[lo] * (1 - alpha) + path[lo + 1] * alpha
            actual = _pose(kin, q)
            expected_offset = offsets[lo] * (1 - alpha) + offsets[lo + 1] * alpha
            delta = actual[:3, 3] - original_pose[:3, 3]
            pe = float(np.linalg.norm(delta - original_pose[:3, 0] * expected_offset))
            re = float(Rotation.from_matrix(actual[:3, :3] @ original_pose[:3, :3].T).magnitude())
            max_position_error = max(max_position_error, pe)
            max_orientation_error = max(max_orientation_error, re)
            measured_offsets.append(float(delta @ original_pose[:3, 0]))
        if max_position_error > position_tolerance or max_orientation_error > orientation_tolerance:
            raise RuntimeError('Actual interpolated sway FK changed the requested translation/orientation')
        if min(measured_offsets) > -.95 * amplitude_m or max(measured_offsets) < .95 * amplitude_m:
            raise RuntimeError('Sway name was added without the actual requested bidirectional FK motion')
        phase_proofs.append(dict(name=name, duration_s=float(duration), sample_count=sampled_count,
            commanded_waypoint_count=len(path), max_position_error_m=max_position_error,
            max_orientation_error_rad=max_orientation_error, cad_x_offset_range_m=[min(measured_offsets), max(measured_offsets)],
            segment_joint_velocity_upper_bound_m_or_rad_s=peak_bound.tolist()))
    revised = list(phases)
    metadata = copy.deepcopy(plan)
    for i in indices:
        name, duration, _, gap, left, right = phases[i]
        revised[i] = (name, duration, path.copy(), gap, left, right)
        metadata['phases'][i]['waypoint_q'] = path.tolist()
        if 'rail_waypoint_positions_m' in metadata['phases'][i]:
            metadata['phases'][i]['rail_waypoint_positions_m'] = [rows[i]['rail_waypoint_positions_m'][0]] * len(path)
    metadata['last_fruit_grasp_sway'] = dict(schema=SCHEMA, enabled=True, amplitude_m=float(amplitude_m),
        exact_native_inventory=inventory, changed_phase_names=[phases[i][0] for i in indices],
        original_hold_q=q0.tolist(), original_hold_fk_pose=original_pose.tolist(), fixed_lift_m=fixed_lift,
        parameterization='CAD +X offset=0 for s<=.75, then amplitude*sin(2*pi*(s-.75)/.25); existing s=smoothstep(phase_time/duration)',
        active_smoothstep_coordinate_interval=[SWAY_START, 1.],
        commanded_positive_peak_s=.8125, commanded_negative_peak_s=.9375,
        endpoint_q_exact=True, yaw_pitch_unchanged=True, duration_jaw_belt_commands_unchanged=True,
        source_phase_commands_sha256=_digest(rows), modified_phase_commands_sha256=_digest(metadata['phases']),
        ik_errors=errors, numerical_position_tolerance_m=position_tolerance,
        numerical_orientation_tolerance_rad=orientation_tolerance, joint_speed_caps_m_or_rad_s=caps.tolist(),
        command_fk_proofs=phase_proofs, collision_certified=False, native_physics_executed=False,
        plant_or_fruit_pose_authored=False, physics_parameters_changed=False, training_eligible=False,
        harvest_success=None, scope='Existing IK/phase representation only; complete self/environment audit and actual native tracking required')
    return revised, metadata
