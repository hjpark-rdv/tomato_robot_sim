"""Opt-in timestep experiment within existing stationary belt_finish rows.

The original plan remains the command authority. All rows outside the selected
hold, and both hold endpoints, are copied exactly. Only physics sampling changes;
this is an accuracy experiment, not a numerically equivalent solver optimization.
"""
from __future__ import annotations

import math
import numpy as np


def validate_belt_hold_hz(value, contact_hz=None):
    if value is not None and (not np.isfinite(value) or value <= 0 or
            contact_hz is not None and value > contact_hz):
        raise ValueError('belt_hold_hz must be positive, finite, and no higher than contact hz')
    return value


def _validate_stationary_plan(plan):
    previous_gap = float(plan.get('initial_gap_m', .044))
    previous_rail = float(plan['initial_command'][0])
    found = False
    for row in plan['phases']:
        q = np.asarray(row['waypoint_q'], float)
        rail = np.asarray(row.get('rail_native_command_positions_m',
            row.get('rail_waypoint_positions_m', [previous_rail] * len(q))), float)
        gap = float(row['end_gap_m'])
        if row['name'] == 'belt_finish':
            found = True
            belt = np.asarray(row['belt_local_y_m_s'], float)
            if (not np.isfinite(q).all() or not np.isfinite(rail).all() or
                    not np.all(q == q[0]) or not np.all(rail == rail[0]) or gap != previous_gap):
                raise ValueError('belt_finish must have stationary arm, rail, and jaw commands')
            if not np.isfinite(belt).all() or not np.any(belt != 0):
                raise ValueError('belt_finish must retain its existing running belt command')
        previous_gap, previous_rail = gap, float(rail[-1])
    if not found:
        raise ValueError('The existing plan contains no belt_finish phase')


def resample_belt_hold(timeline, plan, requested_hz, contact_hz, *, source_start_s=0.):
    """Return a new timeline and evidence, without mutating any source array.

    Each original contiguous belt_finish interval is divided into ceil(T*Hz)
    equal steps. Hence actual dt <= 1/requested_hz, without an arbitrarily short
    boundary remainder. The original endpoints, neighboring samples, belt ramp,
    phase/control policy, and total execution duration are retained.
    """
    from plan_adapter import sample_plan
    from run_smoke import belt_velocity_from_physx

    validate_belt_hold_hz(requested_hz, contact_hz)
    _validate_stationary_plan(plan)
    phases = np.asarray(timeline['phase'])
    selected = phases == 'belt_finish'
    evidence = dict(requested_hz=float(requested_hz), contact_hz=float(contact_hz),
        phase='belt_finish', diagnostic=True, accuracy_qualified=False,
        stationary_plan_verified=True, outside_rows_preserved_exactly=True,
        interval_endpoints_preserved_exactly=True, model_material_release_contact_parameters_changed=False,
        timestep_changed=False, intervals=[])
    if not selected.any():
        return timeline, evidence
    row_fields = ('time', 'source_sample_time_s', 'source_command_record_time_s',
                  'command', 'phase', 'authored_belt_speed', 'belt_surface_velocity_local_y_m_s')
    chunks = {key: [] for key in row_fields}
    ends = []
    i, total = 0, len(phases)
    while i < total:
        j = i + 1
        while j < total and bool(selected[j]) == bool(selected[i]):
            j += 1
        a, b = float(timeline['time_edges'][i]), float(timeline['time_edges'][j])
        n = j - i
        if selected[i]:
            # A few ulps at an integer step count must not create an extra step.
            product = (b - a) * requested_hz
            count = max(1, math.ceil(product - 16 * np.finfo(float).eps * max(1., abs(product))))
            count = min(count, n)
            evidence['intervals'].append(dict(start_s=a, end_s=b, original_steps=n,
                steps=count, effective_hz=count / (b - a), dt_s=(b - a) / count))
        else:
            count = n
        if count == n:
            for key in row_fields:
                chunks[key].append(timeline[key][i:j])
            ends.append(timeline['time_edges'][i + 1:j + 1])
        else:
            edge = np.linspace(a, b, count + 1)
            stamp = source_start_s + edge[:-1]
            rows = [sample_plan(plan, float(t)) for t in stamp]
            if any(row['phase'] != 'belt_finish' for row in rows):
                raise ValueError('Hold resampling crossed an existing belt_finish phase boundary')
            command = np.asarray([row['command'] for row in rows])
            # Equal authored waypoints can differ by rounding in interpolation.
            reference = timeline['command'][i]
            if (not np.allclose(timeline['command'][i:j], reference, rtol=0, atol=1e-13) or
                    not np.allclose(command, reference, rtol=0, atol=1e-13)):
                raise ValueError('belt_finish samples contain changing robot commands')
            belt = np.asarray([row['authored_belt_speed'] for row in rows])
            replacement = dict(time=edge[:-1], source_sample_time_s=stamp,
                source_command_record_time_s=stamp.copy(), command=command,
                phase=np.asarray([row['phase'] for row in rows], dtype=phases.dtype),
                authored_belt_speed=belt,
                belt_surface_velocity_local_y_m_s=belt_velocity_from_physx(belt))
            for key in row_fields:
                chunks[key].append(replacement[key])
            ends.append(edge[1:])
            evidence['timestep_changed'] = True
        i = j
    if not evidence['timestep_changed']:
        return timeline, evidence
    result = dict(timeline)
    result.update({key: np.concatenate(parts) for key, parts in chunks.items()})
    result['time_edges'] = np.r_[timeline['time_edges'][0], np.concatenate(ends)]
    result['dt'] = np.diff(result['time_edges'])
    # Equal subdivisions need not belong to the original integer master clock.
    result.pop('time_edges_master_ticks', None)
    result.pop('master_hz', None)
    hold_dt = result['dt'][result['phase'] == 'belt_finish']
    evidence.update(actual_hold_dt_min_s=float(hold_dt.min()),
        actual_hold_dt_max_s=float(hold_dt.max()),
        graph_timesteps_s=np.unique(np.round(result['dt'], 12)).tolist())
    return result, evidence
