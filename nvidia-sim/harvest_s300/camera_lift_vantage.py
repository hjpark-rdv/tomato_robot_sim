"""Camera scan placement using an existing physical lift transfer.

This produces ordinary waypoints for the existing planner and native runner.
It does not edit the measured initial state, robot mount, or plant geometry.
"""
import numpy as np

from lift_motion import plan_lift_cycle


def lift_scan_transfer(kin, start, height_m):
    """Move the real carriage while retaining all six measured arm angles."""
    start = np.asarray(start, dtype=float)
    bounds = np.asarray(kin.bounds, dtype=float)
    if (start.shape != (7,) or bounds.shape != (2, 7)
            or not np.isfinite(np.r_[start, bounds.ravel()]).all()
            or np.any(bounds[0] > bounds[1])
            or isinstance(height_m, (bool, np.bool_))
            or not np.isscalar(height_m) or not np.isfinite(height_m)
            or not bounds[0, 0] <= height_m <= bounds[1, 0]):
        raise ValueError('Camera lift vantage requires finite measured q and an in-limit absolute lift height')
    if np.any(start < bounds[0]) or np.any(start > bounds[1]):
        raise ValueError('Measured camera lift start exceeds the existing joint limits')
    if height_m == start[0]:
        return [], dict(enabled=True, absolute_lift_height_m=float(height_m),
                        unchanged=True, duration_s=0., physical_initial_state_changed=False)
    # Keep the established six-second lift-only interpolation. Selecting one
    # transfer from the existing cycle does not execute a hidden return cycle.
    cycle, _ = plan_lift_cycle(start, bounds[0], bounds[1],
                              low=min(float(start[0]), float(height_m)),
                              high=max(float(start[0]), float(height_m)))
    transfer = cycle[0 if height_m > start[0] else 2]
    _, duration, qs, _, _, _ = transfer
    phase = ('camera_lift_vantage', duration, qs.copy(), .096, 0., 0.)
    return [phase], dict(enabled=True, absolute_lift_height_m=float(height_m),
        start_measured_q=start.tolist(), end_nominal_q=qs[-1].tolist(),
        unchanged=False, duration_s=float(duration),
        interpolation='Existing lift_motion.plan_lift_cycle transfer; six arm joint angles held',
        physical_initial_state_changed=False, collision_certified=False,
        scope='Physical carriage placement before camera pan; complete path and actual tracking checks required')
