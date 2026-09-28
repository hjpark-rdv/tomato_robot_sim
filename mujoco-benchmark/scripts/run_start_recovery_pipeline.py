"""One-command orchestration: recover start q, then run the bounded D1 round.

This wrapper prevents the previous stop-and-ask loop.  A failed individual
candidate/query is evidence, not a reason to ask for another prompt.  The only
hard stop before D1 is failure to produce a canonical q-only start contract.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def summarize(start_result, plan_result=None, physics_result=None):
    canonical = start_result.get("canonical")
    if not canonical:
        return dict(
            case="CASE0_INITIAL_CONFIGURATION",
            bottleneck="no canonical q-only start found within bounded search",
            evidence_level="covered-scene start search",
            impossible=None)
    rows = (plan_result or {}).get("rows", [])
    evaluated = [r for r in rows if r.get("status") != "not_evaluated_batch_budget"]
    if not evaluated:
        return dict(
            case="UNRESOLVED_D1_NOT_EVALUATED",
            bottleneck="canonical start exists but no D1 query was evaluated",
            evidence_level="start validated only", impossible=None)
    if all(r.get("classification") == "initial_state_invalid" for r in evaluated):
        return dict(
            case="CASE0_INITIAL_CONFIGURATION",
            bottleneck="canonical dense start is invalid in all evaluated D1 scene models",
            evidence_level="D1 static start checks", impossible=None)
    prefix = [r for r in evaluated if r.get("prefix_found")]
    whole = [r for r in evaluated if r.get("whole_path_passed")]
    if not prefix:
        return dict(
            case="CASE1_APPROACH_PLANNING",
            bottleneck="new start valid for at least one query but no staging prefix was found",
            evidence_level="bounded IK/OMPL", impossible=None)
    if not whole:
        return dict(
            case="CASE2_LOCAL_MANIPULATION",
            bottleneck="at least one staging prefix passed audit; no complete local path passed",
            evidence_level="planning/audit only; not physical staging success", impossible=None)
    trials = (physics_result or {}).get("trials", [])
    executed = [r for r in trials if r.get("physics_executed")]
    if not executed:
        return dict(
            case="CASE4_RUNTIME_VALIDATION_PENDING",
            bottleneck="whole paths exist but no actual manipulation physics execution completed its gate",
            evidence_level="whole-path planning/audit", impossible=None)
    completed = [r for r in executed if r.get("completed")]
    if not completed:
        return dict(
            case="CASE4_PHYSICS_TRACKING_OR_CONTACT",
            bottleneck="actual physics started but every executed manipulation stopped or failed",
            evidence_level="recorded physics", impossible=None)
    return dict(
        case="CASE3_LOCAL_PHYSICS_EVIDENCE_AVAILABLE",
        bottleneck="at least one bounded manipulation rollout completed; inspect contact/retention evidence",
        evidence_level="recorded physics; not a harvest-success claim", impossible=None)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dense-source-run", type=Path, required=True)
    parser.add_argument("--covered-scene", type=Path, required=True)
    parser.add_argument("--campaign", type=Path, required=True)
    parser.add_argument("--prefix-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--side-campaign", type=Path)
    parser.add_argument("--planning-python", required=True)
    parser.add_argument("--native-python", required=True)
    parser.add_argument("--model-cache", type=Path)
    parser.add_argument("--render-physics", action="store_true")
    parser.add_argument("--start-timeout", type=float, default=300.0)
    parser.add_argument("--d1-timeout", type=float, default=2100.0)
    args = parser.parse_args(argv)

    out = args.output.resolve()
    if out.exists():
        parser.error("Use a new output directory")
    out.mkdir(parents=True)
    from run_motion_family_search import run_command

    start_out = out / "start_recovery"
    start_command = [
        str(args.planning_python),
        str(Path(__file__).with_name("recover_initial_configuration.py")),
        str(args.dense_source_run.resolve()),
        str(args.covered_scene.resolve()),
        "--output", str(start_out),
    ]
    begin = time.monotonic()
    try:
        start_code = run_command(
            start_command, out / "start_recovery.log", args.start_timeout)
    except TimeoutError:
        summary = dict(
            schema="start_recovery_pipeline_v1",
            status="start_recovery_timeout",
            start_returncode=None,
            final=dict(case="CASE0_INITIAL_CONFIGURATION",
                       bottleneck="start recovery process timed out",
                       evidence_level="bounded search", impossible=None),
            wall_s=time.monotonic() - begin)
        write(out / "pipeline_summary.json", summary)
        return 2

    start_result_path = start_out / "start_recovery.json"
    start_result = read(start_result_path) if start_result_path.is_file() else {}
    canonical = start_out / "canonical_start.json"
    if start_code != 0 or not canonical.is_file():
        summary = dict(
            schema="start_recovery_pipeline_v1",
            status="start_recovery_unresolved",
            start_returncode=start_code,
            start_result=str(start_result_path),
            final=summarize(start_result),
            d1_not_started=True,
            wall_s=time.monotonic() - begin)
        write(out / "pipeline_summary.json", summary)
        return 2

    d1_out = out / "d1"
    d1_command = [
        str(args.planning_python),
        str(Path(__file__).with_name("run_new_start_d1.py")),
        "--campaign", str(args.campaign.resolve()),
        "--prefix-report", str(args.prefix_report.resolve()),
        "--canonical-start", str(canonical),
        "--output", str(d1_out),
        "--planning-python", str(args.planning_python),
        "--native-python", str(args.native_python),
        "--run-plans", "--run-physics",
    ]
    if args.side_campaign:
        d1_command.extend(["--side-campaign", str(args.side_campaign.resolve())])
    if args.model_cache:
        d1_command.extend(["--model-cache", str(args.model_cache.resolve())])
    if args.render_physics:
        d1_command.append("--render-physics")

    try:
        d1_code = run_command(d1_command, out / "d1.log", args.d1_timeout)
        d1_timeout = False
    except TimeoutError:
        d1_code = None
        d1_timeout = True

    plan_path = d1_out / "new_start_plan_results.json"
    physics_path = d1_out / "new_start_physics_results.json"
    plans = read(plan_path) if plan_path.is_file() else None
    physics = read(physics_path) if physics_path.is_file() else None
    final = summarize(start_result, plans, physics)
    summary = dict(
        schema="start_recovery_pipeline_v1",
        status=("d1_timeout" if d1_timeout else "completed_bounded_pipeline"),
        start_returncode=start_code,
        d1_returncode=d1_code,
        start_result=str(start_result_path),
        canonical_start=str(canonical),
        plan_results=str(plan_path) if plan_path.is_file() else None,
        physics_results=str(physics_path) if physics_path.is_file() else None,
        final=final,
        training_eligible=False, hook_success=None, impossible=None,
        wall_s=time.monotonic() - begin)
    write(out / "pipeline_summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 0 if not d1_timeout else 2


if __name__ == "__main__":
    raise SystemExit(main())
