"""Read-only convex-halfspace controls for capsule/tool contact evidence.

These tests describe geometric intersection of saved poses. They do not advance
physics, estimate forces, establish real material damage, or define a global
physical-validity threshold. Absence of a fully buried radius ball does not
prove clearance: a segment can cross a thin local solid without fitting a ball.
"""
from __future__ import annotations

import numpy as np


def segment_inside_convex_interval(equations, start, end, erosion=0.0):
    """Return t in[0,1] whose segment points lie in every eroded halfspace.

    ``equations`` has normalized rows(n_x,n_y,n_z,offset), inside<=0.
    Positive erosion shrinks the solid. With erosion=capsule_radius, every
    accepted center contains an entire radius ball inside the original solid.
    This is an exact finite collection of linear inequalities, apart from IEEE
    arithmetic. Callers may add an explicit positive guard to the radius.
    """
    planes = np.asarray(equations, dtype=float)
    a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    if planes.ndim != 2 or planes.shape[1] != 4 or len(planes) < 4:
        raise ValueError("At least four finite normalized convex halfspaces required")
    if a.shape != (3,) or b.shape != (3,) or not all(np.isfinite(v).all() for v in (planes,a,b)):
        raise ValueError("Finite3D segment and halfspaces required")
    if not np.isfinite(erosion) or erosion < 0:
        raise ValueError("Finite nonnegative erosion required")
    if not np.allclose(np.linalg.norm(planes[:,:3],axis=1),1.,rtol=0,atol=1e-12):
        raise ValueError("Normalized plane normals required for metric erosion")
    delta = b-a
    lo, hi = 0., 1.
    for normal, offset in zip(planes[:,:3], planes[:,3]+erosion):
        value, slope = float(normal@a+offset), float(normal@delta)
        if slope == 0:
            if value > 0: return None
        elif slope > 0: hi = min(hi,-value/slope)
        else: lo = max(lo,-value/slope)
        if lo > hi: return None
    if hi < 0 or lo > 1: return None
    return [max(0.,lo),min(1.,hi)]


def capsule_segment_review(equations, endpoints, radius_m, *, guard_m=1e-8):
    points = np.asarray(endpoints,dtype=float)
    if points.shape != (2,3) or not np.isfinite(radius_m) or radius_m <= 0:
        raise ValueError("Two finite endpoints and a positive capsule radius required")
    if not np.isfinite(guard_m) or guard_m < 0: raise ValueError("Finite nonnegative guard required")
    planes = np.asarray(equations,dtype=float)
    axis = segment_inside_convex_interval(planes,*points)
    core = segment_inside_convex_interval(planes,*points,erosion=radius_m+guard_m)
    signed = np.max(points@planes[:,:3].T+planes[:,3],axis=1)
    length = float(np.linalg.norm(points[1]-points[0]))
    return dict(axis_inside_interval=axis,fully_buried_radius_ball_interval=core,
        axis_length_inside_m=0. if axis is None else float((axis[1]-axis[0])*length),
        axis_length_with_full_radius_ball_inside_m=0. if core is None else float((core[1]-core[0])*length),
        centerline_endpoint_max_halfspace_signed_m=signed.tolist(),
        both_centerline_endpoints_outside=bool(np.all(signed>0)),
        axis_crosses_solid_between_two_outside_ends=bool(axis is not None and np.all(signed>0) and 0<axis[0]<axis[1]<1),
        scope="Convex proxy geometry of saved pose only; no force, material compression, or global validity claim.")
