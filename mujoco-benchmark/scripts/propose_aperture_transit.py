"""Offline waypoint proposals: centre the fruit during transit, seat afterwards.

Transforms the server's *unexecuted* under-then-seat candidates. No physics,
IK, contact policy, evaluator or source result is changed. Geometry uses a
supplied enclosing fruit radius; it does not validate the whole hook or robot.
The exact command trace MUST pass the existing native environment checker.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

SCHEMA = "fruit_centred_aperture_transit_v1"
_ALLOWED = {"preapproach", "approach", "insert", "seat", "hold", "verify"}


def _positive(x, name):
    if isinstance(x, (bool, np.bool_)) or not np.isscalar(x):
        raise ValueError(f"{name} must be a positive finite scalar")
    x = float(x)
    if not math.isfinite(x) or x <= 0:
        raise ValueError(f"{name} must be a positive finite scalar")
    return x


def _point(x):
    x = np.asarray(x, dtype=float)
    if x.shape != (3,) or not np.isfinite(x).all():
        raise ValueError("Expected finite xyz")
    return x


def _read_pose(row):
    p = _point(row["ring_position_xyz"])
    q = np.asarray(row["orientation_xyzw"], dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-6:
        raise ValueError("Expected a normalized xyzw quaternion")
    if row["phase"] not in _ALLOWED:
        raise ValueError("Unsupported diagnostic phase")
    seconds = float(row.get("minimum_seconds", 0))
    if not math.isfinite(seconds) or not 0 <= seconds <= 10:
        raise ValueError("Source minimum_seconds outside planner limits")
    return p, Rotation.from_quat(q)


def pivot_position(fruit_center, fruit_in_ring, rotation):
    """Keep a chosen fruit-in-ring coordinate while rotating about the fruit.

    RING world position p = fruit_world - R @ fruit_in_ring.
    This is NOT rotation about the Hook body origin or wrist flange.
    """
    return _point(fruit_center) - rotation.apply(_point(fruit_in_ring))


def build_candidate(source, fruit_center, fruit_radius, *, wire_radius, aperture_inradius_m,
                    plane_buffer_m, transport_speed_m_s=.010,
                    seat_speed_m_s=.002, max_turn_deg=5., max_waypoints=16):
    """Propose centred transit for the source's one insert -> one seat pattern.

    plane_buffer_m is an explicit planning buffer, not a physics parameter.
    Holding the final lateral offset during ascent is deliberately avoided.
    This family only handles a goal on the opposite side of the fruit plane.
    Rejection means this family is unsupported, NOT physical unreachability.
    """
    c = _point(fruit_center)
    radius = _positive(fruit_radius, "fruit_radius")
    wire = _positive(wire_radius, "wire_radius")
    buffer = _positive(plane_buffer_m, "plane_buffer_m")
    aperture = _positive(aperture_inradius_m, "aperture_inradius_m")
    if radius+buffer >= aperture:
        raise ValueError("Fruit bound does not fit this centred-transit family with the requested buffer")
    fast = _positive(transport_speed_m_s, "transport_speed_m_s")
    slow = _positive(seat_speed_m_s, "seat_speed_m_s")
    turn = math.radians(_positive(max_turn_deg, "max_turn_deg"))
    if turn > math.radians(10):
        raise ValueError("Use at most 10 degrees per fruit-pivot waypoint")
    if isinstance(max_waypoints, bool) or not isinstance(max_waypoints, int) or not 2 <= max_waypoints <= 16:
        raise ValueError("The existing diagnostic planner accepts at most 16 waypoints")
    if source.get("trajectory_mode") != "diagnostic_pose_waypoints_v1" or source.get("diagnostic_only") is not True or source.get("training_eligible") is not False:
        raise ValueError("Only explicit diagnostic candidates can be transformed")
    if not isinstance(source.get("candidate_id"), str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,100}", source["candidate_id"]):
        raise ValueError("Unsafe source candidate identity")
    rows = source["pose_waypoints"]
    if not rows or rows[0].get("phase") != "preapproach":
        raise ValueError("Need unchanged normal-start preapproach prefix")
    for row in rows:
        _read_pose(row)
    insert_ids = [i for i, r in enumerate(rows) if r["phase"] == "insert"]
    seat_ids = [i for i, r in enumerate(rows) if r["phase"] == "seat"]
    if len(insert_ids) != 1 or len(seat_ids) != 1 or seat_ids[0] != insert_ids[0]+1 or insert_ids[0] < 2:
        raise ValueError("Requires the recorded one-insert, one-seat template; do not guess another schema")
    i, j = insert_ids[0], seat_ids[0]
    if any(r["phase"] not in ("preapproach", "approach") for r in rows[:i]):
        raise ValueError("Prefix contains a manipulation phase")
    if any(r["phase"] not in ("hold", "verify") for r in rows[j+1:]):
        raise ValueError("Unexpected suffix; preserve it manually instead")
    prior_p, prior_r = _read_pose(rows[i-1])
    old_insert, _ = _read_pose(rows[i])
    goal, goal_r = _read_pose(rows[j])
    goal_local = goal_r.inv().apply(c-goal)
    if goal_local[1] <= 0:
        raise ValueError("Goal is not on the opposite plane side for this transit family")
    low_depth = max(radius+wire+buffer, float(prior_r.inv().apply(old_insert-c)[1]))
    high_depth = max(radius+wire+buffer, float(goal_local[1]))
    out = copy.deepcopy(rows[:i])

    def append_pose(phase, p, r, speed):
        # The existing planner otherwise treats 'seat' as generic 35 mm/s.
        # minimum_seconds enforces a conservative peak-speed bound for its
        # cubic timing without changing production speed dispatch.
        start_p, start_r = _read_pose(out[-1])
        delta = (r*start_r.inv()).as_rotvec()
        distance = float(np.linalg.norm(p-start_p))
        angle = float(np.linalg.norm(delta))
        duration = max(1.5*distance/speed, angle/.15, .2)
        count = max(1, math.ceil(duration/9.5))
        if len(out)+count > max_waypoints:
            raise ValueError("Waypoint budget exceeded; no automatic budget expansion")
        for k in range(1, count+1):
            u = k/count
            ri = Rotation.from_rotvec(delta*u)*start_r
            pi = start_p+(p-start_p)*u
            out.append(dict(phase=phase, ring_position_xyz=pi.tolist(),
                            orientation_xyzw=ri.as_quat().tolist(),
                            minimum_seconds=duration/count))

    # 1. Enter the aperture below the fruit with the fruit centered, not at
    # the final rear-wire offset. The prefix remains exactly as authored.
    low_local = np.array([0., -low_depth, 0.])
    append_pose("insert", pivot_position(c, low_local, prior_r), prior_r, fast)

    # 2. Change orientation while still clear below the fruit. Each waypoint
    # preserves the fruit-centred pivot. Cartesian interpolation between them
    # is a chord approximation and the compiled joint path still needs audit.
    delta = (goal_r*prior_r.inv()).as_rotvec()
    n = math.ceil(float(np.linalg.norm(delta))/turn)
    for k in range(1, n+1):
        r = Rotation.from_rotvec(delta*(k/n))*prior_r
        append_pose("insert", pivot_position(c, low_local, r), r, fast)

    # 3. Lift along the ring normal while the fruit's projected centre is at
    # the aperture centre. Do not start the rear seating translation early.
    upper = pivot_position(c, [0., high_depth, 0.], goal_r)
    append_pose("seat", upper, goal_r, fast)

    # 4. Lateral transfer with the plane clear of the entire fruit bound;
    # then settle normally to the original goal (if these are distinct).
    transfer = goal + goal_r.apply([0., float(goal_local[1])-high_depth, 0.])
    append_pose("seat", transfer, goal_r, slow)
    if np.linalg.norm(transfer-goal) > 1e-10:
        append_pose("seat", goal, goal_r, slow)

    out.extend(copy.deepcopy(rows[j+1:]))
    if len(out) > max_waypoints:
        raise ValueError("Suffix exceeds the existing 16-waypoint limit")
    for row in out:
        _read_pose(row)
    result = copy.deepcopy(source)
    result.update(candidate_id=source["candidate_id"]+"_aperture", pose_waypoints=out,
                  diagnostic_only=True, offline_teacher_trial=True,
                  training_eligible=False, physics_executed=False, hook_success=None,
                  routing="Centre fruit in aperture during plane crossing, then translate to original seating goal")
    result["connection_proposal"] = dict(
        schema=SCHEMA, source_candidate=source["candidate_id"],
        fruit_center_world=c.tolist(), fruit_bound_radius_m=radius,
        wire_radius_m=wire, aperture_inradius_m=aperture, plane_buffer_m=buffer,
        centred_below_plane_m=low_depth, centred_above_plane_m=high_depth,
        goal_fruit_in_ring=goal_local.tolist(),
        transport_speed_m_s=fast, seat_speed_m_s=slow,
        max_turn_deg=math.degrees(turn),
        full_geometry_checked=False, whole_robot_IK_checked=False,
        source_hold_verify_unchanged=True,
        source_verify_is_not_load_validation=True,
        requires=["verify_enclosing_fruit_bound", "all_32_wire_and_mount_geometry",
                  "native_whole_robot_and_environment_path_check", "forward_physics",
                  "independent_loaded_retention_control"],
        scope="Geometric proposal only. Prefix and legacy verification are not revalidated.")
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source-candidates", type=Path, required=True)
    p.add_argument("--planning-inputs", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--wire-radius-m", type=float, required=True)
    p.add_argument("--plane-buffer-m", type=float, required=True)
    p.add_argument("--aperture-inradius-m", type=float, required=True,
                   help="Measured lower bound: RING centre to every wire surface")
    p.add_argument("--candidate", action="append", help="Exact source ID; may repeat")
    args = p.parse_args()
    paths = [args.source_candidates.resolve(), args.planning_inputs.resolve()]
    out = args.output.resolve()
    if out.exists() or any(root.parent == out or root.parent in out.parents for root in paths):
        p.error("Use a new output directory outside input directories")
    inputs = json.loads(paths[1].read_text())["inputs"]
    candidates = json.loads(paths[0].read_text())
    if not isinstance(candidates, list):
        p.error("Expected a candidate list")
    ids = [r["candidate_id"] for r in candidates]
    if len(set(ids)) != len(ids):
        p.error("Duplicate source candidate identity")
    if args.candidate and set(args.candidate)-set(ids):
        p.error("Unknown requested candidate")
    chosen = [r for r in candidates if not args.candidate or r["candidate_id"] in args.candidate]
    if not 1 <= len(chosen) <= 12:
        p.error("This bounded proposal batch supports 1..12 source candidates")
    accepted, rejected = [], []
    for row in chosen:
        try:
            accepted.append(build_candidate(row, inputs["geometry"][0], inputs["target_radius"],
                                            wire_radius=args.wire_radius_m,
                                            aperture_inradius_m=args.aperture_inradius_m,
                                            plane_buffer_m=args.plane_buffer_m))
        except ValueError as exc:
            rejected.append(dict(candidate_id=row["candidate_id"], reason=str(exc),
                                 not_a_physical_impossibility_label=True))
    report = dict(schema=SCHEMA, proposed=len(accepted), rejected_proposals=rejected,
                  source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in paths},
                  code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  physics_executed=False, training_eligible=False, hook_success=None,
                  scope="No IK, full-scene clearance or contact force was evaluated")
    out.mkdir(parents=True, exist_ok=False)
    (out/"candidates.json").write_text(json.dumps(accepted, indent=2, allow_nan=False)+"\n")
    (out/"proposal_report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report, indent=2))
    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
