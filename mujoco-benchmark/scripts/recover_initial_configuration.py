"""Recover one canonical robot start q against the covered collision model.

The search is q-only.  Robot base, obstacles, masks, margins, materials and
actuators are immutable.  If no q is found within budget, the result remains
unresolved_within_budget / impossible=None.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import mujoco as mj
import numpy as np

from approach_connection import JointSpace
from initial_configuration_recovery import (
    StartSearchBudget, StartStateEvaluator, choose_canonical, search_start)
from covered_environment_result import idle_validity


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def robot_bodies(model):
    ids = set()
    for bid in range(1, model.nbody):
        name = model.body(bid).name or ""
        if name.startswith("Robot_") or name == "Hook" or int(model.body_parentid[bid]) in ids:
            ids.add(bid)
    return ids


def geom_category(model, gid, robot_ids):
    name = model.geom(gid).name or ""
    bid = int(model.geom_bodyid[gid])
    if bid in robot_ids:
        return "robot"
    if name.startswith("neighbor_"):
        return "background_obstacle"
    if name.startswith("gutter_collision_"):
        return "structural_obstacle"
    return "active_plant"


def hold_validate(engine, q, seconds=2.0):
    """Matched hold using RobotEngine preload, with start-contact evidence."""
    q = np.asarray(q, dtype=float)
    if q.shape != engine.initial.shape or not np.isfinite(q).all():
        raise ValueError("Invalid hold q")
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be positive")
    m, d = engine.model, engine.data
    ids = robot_bodies(m)

    mj.mj_resetData(m, d)
    d.qpos[engine.qids] = q
    d.ctrl[engine.aids] = q
    d.qfrc_applied[:] = engine.preload
    mj.mj_forward(m, d)
    x0 = d.xpos.copy()
    q0 = d.qpos[engine.qids].copy()
    robot_body_list = np.array(sorted(ids), dtype=int)
    robot_x0 = d.xpos[robot_body_list].copy() if len(robot_body_list) else np.empty((0, 3))

    worst = {}
    initial = []
    max_body_disp = 0.0
    max_robot_body_disp = 0.0
    max_joint_drift = 0.0

    def contacts():
        rows = []
        for contact in d.contact:
            if float(contact.dist) >= 0:
                continue
            categories = [geom_category(m, int(contact.geom1), ids),
                          geom_category(m, int(contact.geom2), ids)]
            rows.append(dict(
                a=m.geom(int(contact.geom1)).name,
                b=m.geom(int(contact.geom2)).name,
                categories=categories,
                penetration_m=-float(contact.dist)))
        return sorted(rows, key=lambda row: -row["penetration_m"])

    initial = contacts()
    steps = int(round(seconds / m.opt.timestep))
    for step in range(steps + 1):
        mj.mj_forward(m, d)
        for row in contacts():
            key = (row["a"], row["b"])
            if key not in worst or row["penetration_m"] > worst[key]["penetration_m"]:
                worst[key] = dict(row, time_s=float(d.time))
        if m.nbody > 1:
            max_body_disp = max(
                max_body_disp,
                float(np.max(np.linalg.norm(d.xpos[1:] - x0[1:], axis=1))))
        if len(robot_body_list):
            max_robot_body_disp = max(
                max_robot_body_disp,
                float(np.max(np.linalg.norm(d.xpos[robot_body_list] - robot_x0, axis=1))))
        max_joint_drift = max(
            max_joint_drift,
            float(np.max(np.abs(d.qpos[engine.qids] - q0))))
        if step < steps:
            mj.mj_step(m, d)

    record = dict(
        finite=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()),
        warning_counts=d.warning.number.tolist(),
        initial_contacts=initial,
        worst_contacts=sorted(worst.values(), key=lambda row: -row["penetration_m"]),
        max_body_displacement_m=max_body_disp,
        max_robot_body_displacement_m=max_robot_body_disp,
        max_robot_joint_drift=max_joint_drift,
        seconds=seconds,
        hz=float(1.0 / m.opt.timestep))
    validity = idle_validity(record)
    initial_robot_overlap = [
        row for row in initial if "robot" in set(row["categories"])]
    if initial_robot_overlap:
        validity = dict(validity)
        validity["passed"] = False
        validity["reasons"] = list(dict.fromkeys(
            list(validity["reasons"]) + ["initial_robot_overlap"]))
        validity["initial_robot_overlap"] = initial_robot_overlap
    record["validity"] = validity
    return record


def make_engine(run, model_path=None, reference_path=None, hz=240):
    from robot_engine import RobotEngine
    run = Path(run)
    manifest = read(run / "manifest.json")
    assets = run / "replay_assets"
    model_path = Path(model_path) if model_path else assets / "model.mjb"
    reference_path = Path(reference_path) if reference_path else assets / "reference.json"
    return RobotEngine(
        model_path, assets / "initial_trace.json", hz,
        reference=reference_path, target=manifest["target"])


def static_validate_run(q, run, selfchecker, kin, *, seconds=2.0, hz=240):
    from environment_preflight import MuJoCoScene, Policy
    engine = make_engine(run, hz=hz)
    policy = Policy(clearance_m=0.0, max_distance_queries=3000000, timeout_s=120.0)
    scene = MuJoCoScene(engine, policy)
    space = JointSpace(kin.bounds, scene.joint_steps)
    budget = StartSearchBudget(max_candidates=1, max_distance_queries=3000000, seconds=120.0)
    evaluator = StartStateEvaluator(scene, selfchecker, space, budget)
    geometry = evaluator.evaluate(q)
    hold = hold_validate(engine, q, seconds=seconds) if geometry["valid"] else None
    passed = bool(geometry["valid"] and hold and hold["validity"]["passed"])
    return dict(run=str(Path(run).resolve()), passed=passed,
                geometry=geometry, hold=hold,
                model_sha256=sha(Path(run) / "replay_assets/model.mjb"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_run", type=Path,
                        help="Run providing robot planning model, original q and target")
    parser.add_argument("covered_scene", type=Path,
                        help="Folder containing covered model.mjb/reference.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-candidates", type=int, default=512)
    parser.add_argument("--max-distance-queries", type=int, default=3000000)
    parser.add_argument("--search-seconds", type=float, default=180.0)
    parser.add_argument("--samples-per-radius", type=int, default=24)
    parser.add_argument("--radii", default=".01,.025,.05,.1,.2,.4",
                        help="Normalized joint-space search radii; search budget, not safety limits")
    parser.add_argument("--max-valid", type=int, default=12)
    parser.add_argument("--hold-candidates", type=int, default=6)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--physics-hz", type=int, default=240)
    parser.add_argument("--seed", type=int, default=20260928)
    parser.add_argument("--validation-run", action="append", type=Path, default=[],
                        help="Additional D1 run on which the same canonical q must validate")
    args = parser.parse_args(argv)

    source = args.source_run.resolve()
    covered = args.covered_scene.resolve()
    out = args.output.resolve()
    if out.exists():
        parser.error("Use a new output directory")
    if source == out or source in out.parents or covered == out or covered in out.parents:
        parser.error("Output must not overwrite source/covered assets")
    model = covered / "model.mjb"
    reference = covered / "reference.json"
    if not model.is_file() or not reference.is_file():
        parser.error("covered_scene needs model.mjb and reference.json")
    radii = tuple(float(x) for x in args.radii.split(",") if x.strip())
    if not radii:
        parser.error("No search radii")
    out.mkdir(parents=True)

    import plan_candidates as planner
    from environment_preflight import MuJoCoScene, Policy

    planner.initialize(str(source))
    env, kin, selfchecker = planner.MODEL
    original = np.asarray(planner.START, dtype=float)
    covered_engine = make_engine(
        source, model_path=model, reference_path=reference, hz=args.physics_hz)
    if not np.allclose(original, covered_engine.initial, atol=1e-12, rtol=0):
        raise ValueError("Planner/source initial q mismatch")

    policy = Policy(clearance_m=0.0,
                    max_distance_queries=args.max_distance_queries,
                    timeout_s=args.search_seconds)
    scene = MuJoCoScene(covered_engine, policy)
    space = JointSpace(kin.bounds, scene.joint_steps)
    budget = StartSearchBudget(
        max_candidates=args.max_candidates,
        max_distance_queries=args.max_distance_queries,
        seconds=args.search_seconds)
    evaluator = StartStateEvaluator(scene, selfchecker, space, budget)
    search = search_start(
        original, space, evaluator, radii=radii,
        samples_per_radius=args.samples_per_radius,
        seed=args.seed, max_valid=args.max_valid)

    hold_results = {}
    for candidate in search["ranked_valid"][:args.hold_candidates]:
        q = np.asarray(candidate["q"], dtype=float)
        hold_results[tuple(float(x) for x in q)] = hold_validate(
            covered_engine, q, seconds=args.hold_seconds)

    canonical = choose_canonical(search, hold_results)
    validation = []
    if canonical is not None:
        q = np.asarray(canonical["q"], dtype=float)
        for run in args.validation_run:
            validation.append(static_validate_run(
                q, run.resolve(), selfchecker, kin,
                seconds=args.hold_seconds, hz=args.physics_hz))
        canonical["all_requested_validation_runs_passed"] = all(
            row["passed"] for row in validation)

    original_record = search["records"][0] if search["records"] else None
    diagnostic = None
    if original_record and original_record.get("closest_segment_world"):
        segment = np.asarray(original_record["closest_segment_world"], dtype=float)
        diagnostic = dict(
            original_closest_pair=original_record.get("closest_pair"),
            original_signed_distance_m=original_record.get("minimum_distance_m"),
            closest_segment_world=segment.tolist(),
            witness_delta_m=(segment[1] - segment[0]).tolist(),
            note="Witness vector is diagnostic only; it is not permission to move the robot base.")

    result = dict(
        schema="initial_configuration_recovery_v1",
        status=("canonical_start_found" if canonical is not None else
                "unresolved_within_budget"),
        impossible=None,
        source_run=str(source),
        covered_scene=str(covered),
        source_model_sha256=sha(source / "replay_assets/model.mjb"),
        covered_model_sha256=sha(model),
        original_q=original.tolist(),
        search_contract=dict(
            q_only=True, robot_base_fixed=True, collision_geometry_unchanged=True,
            contact_masks_unchanged=True, materials_unchanged=True,
            margins_unchanged=True, actuators_unchanged=True,
            radii=list(radii), samples_per_radius=args.samples_per_radius,
            seed=args.seed),
        search=search,
        hold_results=list(hold_results.values()),
        canonical=canonical,
        requested_validation_runs=validation,
        diagnostic_if_unresolved=diagnostic,
        wall_s=time.monotonic() - budget.started,
        training_eligible=False, hook_success=None)
    write(out / "start_recovery.json", result)
    if canonical is not None:
        write(out / "canonical_start.json", dict(
            schema="canonical_robot_start_v1",
            q=canonical["q"],
            original_q=original.tolist(),
            source_run=str(source),
            covered_model_sha256=result["covered_model_sha256"],
            selection=canonical["search"],
            hold=canonical["hold"],
            validation_runs=validation,
            training_eligible=False, hook_success=None, impossible=None))
    print(json.dumps(dict(
        status=result["status"],
        evaluated_candidates=search["evaluated_candidates"],
        valid_candidates=search["valid_candidates"],
        validation_runs=len(validation),
        all_validation_passed=(canonical or {}).get(
            "all_requested_validation_runs_passed")), indent=2))
    if canonical is None:
        return 2
    if validation and not canonical["all_requested_validation_runs_passed"]:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
