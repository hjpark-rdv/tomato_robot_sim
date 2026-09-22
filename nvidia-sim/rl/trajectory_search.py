"""Small fixed-scene trajectory pilot. Metres/degrees, no perception or RL.

HOOK_FRAME is the existing CAD half-ring centre: +X toward the opening/robot,
+Y normal to the ring (world down here), +Z lateral in the ring plane.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
from geometry import RING_RADIUS, WIRE_RADIUS

BOUNDS = dict(approach_azimuth_deg=(-45., 45.), entry_clearance_m=(.000, .010),
              lateral_offset_m=(-.008, .008), insertion_distance_m=(.025, .060),
              lift_forward_angle_deg=(-15., 15.), lift_distance_m=(.020, .045))


def candidates(count, seed):
    unit = qmc.Sobol(d=6, scramble=True, seed=seed).random_base2(int(math.ceil(math.log2(count))))[:count]
    values = qmc.scale(unit, [v[0] for v in BOUNDS.values()], [v[1] for v in BOUNDS.values()])
    return [dict(zip(BOUNDS, map(float, row)), candidate_id=f'candidate_{i:05d}',
                 trajectory_mode='staged6d', stage='coarse', parent_id=None, seed=seed, sobol_index=i,
                 pre_hook_distance_m=.17, hook_roll_deg=0., approach_elevation_deg=0.)
            for i, row in enumerate(values)]


def waypoints(center, neck, params, radius):
    if radius is None or not np.isfinite(radius) or radius <= 0:
        raise ValueError('staged6d requires the actual target collision radius')
    c = np.asarray(center, dtype=float)
    a = np.deg2rad(params['approach_azimuth_deg'])
    outward = np.array([np.cos(a), np.sin(a), 0.])
    up = np.array([0., 0., 1.]); lateral = np.cross(outward, -up)
    r = Rotation.from_matrix(np.column_stack([outward, -up, lateral]))
    # Entire ring is below the fruit collision sphere, not initially inserted.
    low = c + lateral*params['lateral_offset_m'] - up*(radius+WIRE_RADIUS+params['entry_clearance_m'])
    entry = low + outward*(RING_RADIUS+radius+.010)
    pre = low + outward*params['pre_hook_distance_m']
    insert = entry - outward*params['insertion_distance_m']
    tilt = np.deg2rad(params['lift_forward_angle_deg'])
    lift = insert + params['lift_distance_m']*(up*np.cos(tilt)-outward*np.sin(tilt))
    return r, [('preapproach', pre), ('entry', entry), ('insert', insert), ('rise', lift)], -outward


def center_region(local_center, radius):
    """Aperture-plane centre entry, not just proximity or contact.

    Fruit cross-section must fit inside the ring's circular clearance, with
    its centre on the rear half, within 2mm of the plane. Partial progress only.
    Entry history additionally requires an initially outside centre.
    """
    x, y, z = np.asarray(local_center, dtype=float)
    fits = x <= -.001 and np.hypot(x, z)+radius <= RING_RADIUS-WIRE_RADIUS
    return bool(fits and abs(y) <= .002)
