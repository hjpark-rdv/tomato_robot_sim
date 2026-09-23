"""Small fixed-scene trajectory pilot. Metres/degrees, no perception or RL.

HOOK_FRAME is the existing CAD half-ring centre: +X toward the opening/robot,
+Y normal to the ring (world down here), +Z lateral in the ring plane.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
from geometry import RING_RADIUS, WIRE_RADIUS

# Keep the staged6d file/CLI schema, but sample only azimuth for new datasets.
# Other motion parameters use the former interval midpoints, except user-fixed clearance/offset.
SAMPLING = 'scrambled_sobol_azimuth_only'
FIXED = dict(entry_clearance_m=.002, lateral_offset_m=0., insertion_distance_m=.0425,
             lift_forward_angle_deg=0., lift_distance_m=.0325)
BOUNDS = dict(approach_azimuth_deg=(-90., 90.), **{k:(v,v) for k,v in FIXED.items()})


def candidates(count, seed):
    if count < 1:
        raise ValueError('Candidate count must be positive')
    unit = qmc.Sobol(d=1, scramble=True, seed=seed).random_base2(int(math.ceil(math.log2(count))))[:count,0]
    low, high = BOUNDS['approach_azimuth_deg']
    angles = low + (high-low)*unit
    return [dict(approach_azimuth_deg=float(angle), **FIXED,
                 candidate_id=f'candidate_{i:05d}', sampling=SAMPLING, lift_profile='diagonal_45_return',
                 trajectory_mode='staged6d', stage='coarse', parent_id=None, seed=seed, sobol_index=i,
                 pre_hook_distance_m=.17, hook_roll_deg=0., approach_elevation_deg=0.)
            for i, angle in enumerate(angles)]


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
    points=[('preapproach', pre), ('entry', entry), ('insert', insert)]
    if params.get('lift_profile')=='diagonal_45_return':
        if abs(params['lift_forward_angle_deg'])>1e-12:
            raise ValueError('45-degree return lift requires a vertical net endpoint')
        mid=insert+params['lift_distance_m']*.5*(up-outward)
        points.append(('rise_mid',mid))
    points.append(('rise',lift))
    return r, points, -outward


def center_region(local_center, radius):
    """Aperture-plane centre entry, not just proximity or contact.

    Fruit cross-section must fit inside the ring's circular clearance, with
    its centre on the rear half, within 20mm of the plane. Partial progress only.
    Entry history additionally requires an initially outside centre.

    NOTE: Plane tolerance relaxed from 2mm to 20mm (2026-09-23).
    At 30fps (33ms/frame) recording, ±2mm was too narrow to reliably capture
    the passage moment. ±20mm still requires radial containment inside the ring
    clearance (RING_RADIUS - WIRE_RADIUS - fruit_radius = ~14.9mm for half-scale),
    so false positives from mere proximity remain prevented.
    """
    x, y, z = np.asarray(local_center, dtype=float)
    fits = x <= -.001 and np.hypot(x, z)+radius <= RING_RADIUS-WIRE_RADIUS
    return bool(fits and abs(y) <= .020)
