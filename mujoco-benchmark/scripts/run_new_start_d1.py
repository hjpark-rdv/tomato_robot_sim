"""Prepare and optionally run the frozen D1 round from a recovered start q.

Historical candidate/target/local-design selection is preserved, but every
prefix is replanned from the new canonical start.  Old recorded prefix commands
are never copied or spliced onto the new q.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from materialize_new_start_run import materialize
import prepare_recovery_batch as historical


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def source_root(campaign, side_campaign, query):
    if query["cohort"] == "side_self_collision":
        if side_campaign is None:
            raise ValueError("Side query exists but no side campaign was supplied")
        return Path(side_campaign).resolve() / "cases" / query["scene"] / query["target"] / "run"
    return Path(campaign).resolve() / "cases" / query["scene"] / query["target"] / "run"


def prepare(campaign, prefix_report, start_file, output, planning_python, *,
            prefix_targets=4, local_targets=2, model_cache=None,
            side_campaign=None):
    campaign = Path(campaign).resolve()
    prefix_report = Path(prefix_report).resolve()
    start_file = Path(start_file).resolve()
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("Use a new output directory")
    start = read(start_file)
    if start.get("schema") != "canonical_robot_start_v1":
        raise ValueError("Expected canonical_robot_start_v1")
    q = start.get("q")
    if not isinstance(q, list) or not q:
        raise ValueError("Canonical start q missing")

    # Freeze exactly the already-reviewed D1 evidence selection first.  This
    # directory is historical bookkeeping only; its old-start commands are not run.
    selection_root = output / "historical_selection"
    old = historical.prepare(
        campaign, prefix_report, selection_root, planning_python,
        prefix_targets=prefix_targets, local_targets=local_targets,
        model_cache=model_cache, side_campaign=side_campaign)

    start_runs = output / "new_start_runs"
    plans = output / "plans"
    queries = []
    materialized = {}
    for old_query in old["queries"]:
        root = source_root(campaign, side_campaign, old_query)
        key = str(root.resolve())
        if key not in materialized:
            identity = f"{old_query['scene']}__{old_query['target']}"
            derived = start_runs / identity
            provenance = materialize(root, start_file, derived)
            materialized[key] = dict(run=str(derived), provenance=provenance)
        derived = Path(materialized[key]["run"])
        policy = root.parent / "base_policy.json"
        qid = old_query["query_id"]
        destination = plans / qid
        command = [
            str(planning_python),
            str(Path(__file__).with_name("plan_approach_connection.py")),
            str(derived),
            "--candidate", old_query["candidate"],
            "--base-policy", str(policy),
            "--output", str(destination),
            "--local-design", old_query["design"],
            "--ik-seeds", "8",
            "--max-branches", "3",
            "--solve-seconds", "8",
            "--total-seconds", "120",
            "--seed", "20260928",
        ]
        if model_cache:
            command.extend(["--model-cache", str(Path(model_cache).resolve())])
        queries.append(dict(
            query_id=qid,
            scene=old_query["scene"],
            target=old_query["target"],
            candidate=old_query["candidate"],
            family=old_query.get("family"),
            design=old_query["design"],
            historical_cohort=old_query["cohort"],
            source_run=str(root),
            new_start_run=str(derived),
            base_policy=str(policy),
            command=command,
            output=str(destination),
            original_prefix_reuse_allowed=False,
            prefix_mode="new_start_replanned_prefix",
            physics_executed=False,
            training_eligible=False,
            hook_success=None,
            impossible=None))

    doc = dict(
        schema="new_start_d1_round_v1",
        canonical_start=str(start_file),
        canonical_q=q,
        historical_selection=str(selection_root / "recovery_queries.json"),
        historical_query_count=len(old["queries"]),
        queries=queries,
        materialized_runs=list(materialized.values()),
        planning_budget=len(queries),
        physics_budget=8,
        selection_scope="Same evidence-selected D1 targets/designs; all prefixes replanned from canonical new start",
        recorded_prefix_reuse_allowed=False,
        training_eligible=False, hook_success=None, impossible=None)
    write(output / "new_start_queries.json", doc)
    return doc


def classify_connection(result):
    status = result.get("status", "missing_result")
    if status == "invalid_initial_configuration":
        return "initial_state_invalid"
    if status == "no_valid_ik_goal_found":
        return "ik_not_found_within_budget"
    if status in ("budget_exhausted", "no_complete_connection_found"):
        return "unresolved_within_budget"
    if status in ("retimed_prefix_environment_rejected",):
        return "approach_environment_blocked"
    if status == "whole_path_passed":
        return "whole_path_audit_passed"
    if status == "prefix_clear_suffix_planning_rejected":
        return "local_suffix_planning_rejected"
    if status in ("whole_self_check_failed", "retimed_prefix_self_check_failed"):
        return "self_collision_path"
    if status == "prefix_clear_suffix_environment_rejected":
        hit = None
        for attempt in result.get("attempts", []):
            if attempt.get("first_violation"):
                hit = attempt["first_violation"]
        phases = set((hit or {}).get("phases", []))
        if phases & {"insert"}:
            return "insert_environment_blocked"
        if phases & {"seat", "hold", "verify"}:
            return "seat_environment_blocked"
        return "local_environment_blocked"
    if status in ("execution_error", "source_changed", "cache_restore_error"):
        return status
    return status


def run_plans(doc, output, batch_seconds=1500.0):
    from run_motion_family_search import run_command
    output = Path(output)
    rows = []
    began = time.monotonic()
    for query in doc["queries"]:
        row = {k: query[k] for k in (
            "query_id", "scene", "target", "candidate", "design",
            "historical_cohort", "new_start_run", "base_policy")}
        remaining = batch_seconds - (time.monotonic() - began)
        if remaining < 180.0:
            row.update(status="not_evaluated_batch_budget",
                       classification="unresolved_within_budget",
                       physics_executed=False)
        else:
            try:
                code = run_command(
                    query["command"],
                    output / f"{query['query_id']}.log", 180.0)
                path = Path(query["output"]) / "connection_result.json"
                result = read(path) if path.is_file() else {}
                row.update(
                    returncode=code,
                    status=result.get("status", "missing_result"),
                    classification=classify_connection(result),
                    prefix_found=result.get("prefix_found", False),
                    prefix_reused=result.get("prefix_reused", False),
                    whole_path_passed=result.get("whole_path_passed", False),
                    runnable_run=result.get("runnable_run"),
                    approach_only_runs=result.get("approach_only_runs", []),
                    connection_result=str(path),
                    source_unchanged=result.get("source_unchanged"),
                    physics_executed=False)
                if row["prefix_reused"]:
                    row.update(status="invalid_new_start_prefix_reuse",
                               classification="execution_error",
                               whole_path_passed=False, runnable_run=None)
                if code not in (0, 2):
                    row.update(status="planner_process_error",
                               classification="execution_error",
                               whole_path_passed=False, runnable_run=None)
            except TimeoutError:
                row.update(status="planner_process_timeout",
                           classification="unresolved_within_budget",
                           physics_executed=False)
        rows.append(row)
        write(output / "new_start_plan_results.json", dict(
            rows=rows,
            reported_queries=len(rows),
            evaluated_queries=sum("returncode" in r for r in rows),
            whole_paths=sum(bool(r.get("whole_path_passed")) for r in rows),
            physics_executed=False,
            wall_s=time.monotonic() - began,
            hook_success=None, impossible=None))
    return rows


def run_physics(doc, plan_rows, output, native_python, *,
                max_executions=8, timeout_s=300.0, render=False):
    """Execute only full paths that already passed the whole environment audit."""
    from run_motion_family_search import run_command
    output = Path(output)
    by_id = {q["query_id"]: q for q in doc["queries"]}
    trials = []
    eligible = [row for row in plan_rows
                if row.get("whole_path_passed") and row.get("runnable_run")]
    for row in eligible[:max_executions]:
        query = by_id[row["query_id"]]
        trial = output / "physics" / row["query_id"]
        command = [
            str(native_python),
            str(Path(__file__).with_name("target_fruit_contact_trial.py")),
            str(row["runnable_run"]),
            "--candidate", row["candidate"],
            "--base-policy", query["base_policy"],
            "--output", str(trial),
            "--search-contact-policy", "--execute",
            "--max-target-force-n", "5",
            "--max-target-displacement-m", "0.02",
        ]
        log = output / f"{row['query_id']}_physics.log"
        try:
            code = run_command(command, log, timeout_s)
        except TimeoutError:
            trials.append(dict(query_id=row["query_id"], status="physics_timeout",
                               physics_executed=False))
            continue
        result_path = trial / "target_contact_trial.json"
        result = read(result_path) if result_path.is_file() else {}
        record = dict(
            query_id=row["query_id"], returncode=code,
            status=("physics_completed" if result.get("completed") else
                    result.get("reason", "physics_stopped_or_failed")),
            physics_executed=bool(result.get("physics_executed")),
            completed=bool(result.get("completed")),
            stop_reasons=result.get("stop_reasons", []),
            trial=str(trial),
            hook_success=None, training_eligible=False)
        if render and result.get("physics_executed") and (trial / "trial_states.npz").is_file():
            video = trial / "recorded_physics.mp4"
            render_command = [
                str(native_python),
                str(Path(__file__).with_name("render_contact_diagnostic.py")),
                str(row["runnable_run"]),
                "--states", str(trial / "trial_states.npz"),
                "--output", str(video),
                "--candidate", row["candidate"],
                "--label", "recorded physics qpos",
            ]
            try:
                render_code = run_command(
                    render_command,
                    output / f"{row['query_id']}_render.log", timeout_s)
                record.update(video=str(video) if render_code == 0 else None,
                              render_returncode=render_code)
            except TimeoutError:
                record.update(video=None, render_status="timeout")
        trials.append(record)
        write(output / "new_start_physics_results.json", dict(
            eligible_full_paths=len(eligible),
            execution_cap=max_executions,
            trials=trials,
            actual_physics_executions=sum(bool(x.get("physics_executed")) for x in trials),
            completed=sum(bool(x.get("completed")) for x in trials),
            hook_success=None, impossible=None))
    if not trials:
        write(output / "new_start_physics_results.json", dict(
            eligible_full_paths=len(eligible), execution_cap=max_executions,
            trials=[], actual_physics_executions=0, completed=0,
            hook_success=None, impossible=None))
    return trials


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, required=True)
    parser.add_argument("--prefix-report", type=Path, required=True)
    parser.add_argument("--canonical-start", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--side-campaign", type=Path)
    parser.add_argument("--planning-python", required=True)
    parser.add_argument("--native-python")
    parser.add_argument("--model-cache", type=Path)
    parser.add_argument("--run-plans", action="store_true")
    parser.add_argument("--run-physics", action="store_true")
    parser.add_argument("--render-physics", action="store_true")
    parser.add_argument("--batch-seconds", type=float, default=1500.0)
    args = parser.parse_args(argv)
    if args.run_physics and not args.run_plans:
        parser.error("--run-physics requires --run-plans in this bounded runner")
    if args.run_physics and not args.native_python:
        parser.error("--run-physics requires --native-python")

    doc = prepare(
        args.campaign, args.prefix_report, args.canonical_start,
        args.output, args.planning_python, model_cache=args.model_cache,
        side_campaign=args.side_campaign)
    print(json.dumps(dict(
        queries=len(doc["queries"]),
        recorded_prefix_reuse_allowed=False,
        physics_budget=doc["physics_budget"]), indent=2))
    if not args.run_plans:
        return 0
    rows = run_plans(doc, args.output, args.batch_seconds)
    if args.run_physics:
        run_physics(
            doc, rows, args.output, args.native_python,
            max_executions=doc["physics_budget"],
            render=args.render_physics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
