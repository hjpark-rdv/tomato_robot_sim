"""Target-relative seating GOALS, not executable paths or harvesting labels.

All capsules are finite centreline segments with physical (uninflated) radii.
Wire coordinates use the authored RING centre, +X opening, +Y plane normal.
No sensor, contact force, plant deformation, IK or whole-robot inference occurs.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

_EPS = 1e-12


def _vector(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError(f"{name} must be a finite 3-vector")
    return a


def _rotation(value):
    if not isinstance(value, Rotation) or value.as_matrix().shape != (3, 3):
        raise ValueError("Expected one scipy Rotation, not a rotation batch")
    if not np.isfinite(value.as_matrix()).all():
        raise ValueError("Non-finite rotation")
    return value


def _positive(value, name, zero=False):
    if isinstance(value, (bool, np.bool_)) or not np.isscalar(value):
        raise ValueError(f"{name} must be a finite scalar")
    value = float(value)
    if not math.isfinite(value) or value < 0 or (value == 0 and not zero):
        raise ValueError(f"Invalid {name}")
    return value


@dataclass(frozen=True)
class Capsule:
    name: str
    a: np.ndarray
    b: np.ndarray
    radius: float

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("Capsule needs an exact nonempty identity")
        for attr in ("a", "b"):
            v = _vector(getattr(self, attr), attr).copy()
            v.flags.writeable = False
            object.__setattr__(self, attr, v)
        object.__setattr__(self, "radius", _positive(self.radius, "radius"))
        if np.linalg.norm(self.b - self.a) <= _EPS:
            raise ValueError("Capsule centreline has zero length")

    def json(self):
        return dict(name=self.name, a=self.a.tolist(), b=self.b.tolist(), radius_m=self.radius)


def segment_distance(a, b, c, d):
    """Exact finite segment minimum: interior stationary point plus four edges."""
    a, b, c, d = (_vector(v, n) for v, n in zip((a, b, c, d), "abcd"))
    u, v, w = b - a, d - c, a - c
    uu, vv, uv = float(u @ u), float(v @ v), float(u @ v)
    candidates = []
    for s in (0., 1.):
        t = np.clip(v @ (w + s*u) / vv, 0., 1.) if vv > _EPS**2 else 0.
        candidates.append((s, t))
    for t in (0., 1.):
        s = np.clip(u @ (t*v - w) / uu, 0., 1.) if uu > _EPS**2 else 0.
        candidates.append((s, t))
    det = uu*vv - uv*uv
    if det > 1e-14 * uu*vv and uu > _EPS**2 and vv > _EPS**2:
        uw, vw = float(u @ w), float(v @ w)
        s, t = (uv*vw - vv*uw)/det, (uu*vw - uv*uw)/det
        if 0 <= s <= 1 and 0 <= t <= 1:
            candidates.append((s, t))
    return min(float(np.linalg.norm(w + s*u - t*v)) for s, t in candidates)


def capsule_gap(first: Capsule, second: Capsule):
    return segment_distance(first.a, first.b, second.a, second.b) - first.radius - second.radius


def in_ring_frame(capsule: Capsule, ring_position, rotation):
    p, r = _vector(ring_position, "ring_position"), _rotation(rotation)
    return Capsule(capsule.name, r.inv().apply(capsule.a-p), r.inv().apply(capsule.b-p), capsule.radius)


def seating_geometry(target_world, ring_position, rotation, rear_wires, *, ring_radius,
                     max_surface_gap_m, roundoff_m=1e-9):
    """Conservative geometric candidate, explicitly NOT contact/retention success.

    Unlike the legacy proxy, negative wire clearance cannot pass just because
    it is less than an upper gap bound. Require a finite centreline crossing;
    endpoint-only or coplanar contacts are outside this diagnostic's scope.
    """
    ring_radius = _positive(ring_radius, "ring_radius")
    max_gap = _positive(max_surface_gap_m, "max_surface_gap_m", zero=True)
    roundoff = _positive(roundoff_m, "roundoff_m", zero=True)
    if roundoff > 1e-7:
        raise ValueError("roundoff_m is numerical tolerance, not penetration allowance")
    wires = tuple(rear_wires)
    if not wires or len({w.name for w in wires}) != len(wires):
        raise ValueError("Need uniquely named rear wire capsules")
    target = in_ring_frame(target_world, ring_position, rotation)
    gaps = [capsule_gap(target, w) for w in wires]
    idx = int(np.argmin(gaps))
    direction = target.b - target.a
    reasons, crossing = [], None
    if abs(direction[1]) <= _EPS:
        reasons.append("coplanar_or_parallel_axis")
    else:
        t = float(-target.a[1]/direction[1])
        if not 0. <= t <= 1.:
            reasons.append("no_finite_centreline_plane_crossing")
        else:
            crossing = target.a + t*direction
            if crossing[0] >= 0.:
                reasons.append("not_rear_half")
            if np.linalg.norm(crossing[[0, 2]]) >= ring_radius - max(w.radius for w in wires):
                reasons.append("not_inside_wire_inner_footprint")
    if min(gaps) < -roundoff:
        reasons.append("intersects_rear_wire")
    if min(gaps) > max_gap + roundoff:
        reasons.append("too_far_from_rear_wire")
    return dict(geometric_seating_candidate=not reasons, reasons=reasons,
                minimum_rear_surface_gap_m=float(gaps[idx]), nearest_rear_wire=wires[idx].name,
                centreline_plane_crossing_ring=None if crossing is None else crossing.tolist(),
                hook_success=None, training_eligible=False,
                scope="selected finite target capsule vs rear wires only; not force, retention or path validity")


def aligned_rotation(target: Capsule, preferred: Rotation):
    """Align plane normal to the capsule axis; preserve preferred facing sign."""
    preferred = _rotation(preferred)
    normal = target.b - target.a
    normal /= np.linalg.norm(normal)
    if normal @ preferred.apply([0., 1., 0.]) < 0:
        normal = -normal
    outward = preferred.apply([1., 0., 0.])
    outward -= normal*(outward @ normal)
    if np.linalg.norm(outward) <= 1e-8:
        outward = preferred.apply([0., 0., 1.])
        outward -= normal*(outward @ normal)
    outward /= np.linalg.norm(outward)
    lateral = np.cross(outward, normal)
    return Rotation.from_matrix(np.column_stack((outward, normal, lateral)))


def seating_goal(target: Capsule, wire: Capsule, rotation: Rotation, *,
                 fraction, surface_gap_m):
    """Place a target interior point at the INSIDE of one actual rear wire.

    The common normal of the two centreline directions gives an exact finite
    capsule gap at their interior witness points. Other wire segments and the
    finite plane crossing must subsequently be checked with seating_geometry.
    """
    rotation = _rotation(rotation)
    if isinstance(fraction, bool) or not math.isfinite(fraction) or not 0 < fraction < 1:
        raise ValueError("fraction must be strictly inside (0, 1)")
    gap = _positive(surface_gap_m, "surface_gap_m", zero=True)
    midpoint = (wire.a + wire.b)*.5
    inward = np.array([-midpoint[0], 0., -midpoint[2]])
    if midpoint[0] >= 0 or np.linalg.norm(inward) <= _EPS:
        raise ValueError("Wire is not in the rear half of the ring-centred frame")
    axis = rotation.inv().apply(target.b-target.a)
    tangent = wire.b-wire.a
    axis /= np.linalg.norm(axis)
    tangent /= np.linalg.norm(tangent)
    normal = np.cross(axis, tangent)
    if np.linalg.norm(normal) < 1e-8:
        raise ValueError("Parallel target/wire axes do not define this interior seating goal")
    normal /= np.linalg.norm(normal)
    alignment = float(normal @ (inward/np.linalg.norm(inward)))
    if abs(alignment) < 1e-8:
        raise ValueError("No distinguishable inward contact normal")
    if alignment < 0:
        normal = -normal
    target_point_local = midpoint + normal*(wire.radius+target.radius+gap)
    target_point_world = target.a + fraction*(target.b-target.a)
    position = target_point_world - rotation.apply(target_point_local)
    target_local = in_ring_frame(target, position, rotation)
    achieved = capsule_gap(target_local, wire)
    if abs(achieved-gap) > 1e-8:
        raise ValueError("Finite-segment seating witness failed verification")
    return dict(ring_position_xyz=position.tolist(), orientation_xyzw=rotation.as_quat().tolist(),
                target_geom=target.name, rear_wire_geom=wire.name, target_fraction=float(fraction),
                target_point_world=target_point_world.tolist(), target_point_ring=target_point_local.tolist(),
                requested_surface_gap_m=gap, chosen_wire_surface_gap_m=float(achieved),
                diagnostic_only=True, offline_teacher_goal=True, physics_executed=False,
                training_eligible=False, hook_success=None,
                scope="SIM-GT geometric pose proposal; no IK, path or holding success implied")
