"""Preplan a complete non-lift tomato batch, then execute cached trajectories."""

from __future__ import annotations

import copy
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter

from rbpodo_tomato_harvest.harvest_planner import (
    CartesianHarvestPlanner,
    HarvestMotionPlan,
)


EVENT_PREFIX = "__HARVEST_PREPLANNED_BATCH_EVENT__"
CONFIG_ENV = "HARVEST_PREPLANNED_BATCH_CONFIG"


def _emit(phase: str, **values) -> None:
    print(
        f"{EVENT_PREFIX}{json.dumps({'phase': phase, **values})}",
        flush=True,
    )


def _final_state(
    planner: CartesianHarvestPlanner,
    plan: HarvestMotionPlan,
):
    for trajectories in (
        plan.return_pick_ready_trajectory,
        plan.outward_retreat_trajectory,
        plan.after_wait_trajectory,
    ):
        if trajectories:
            return planner._trajectory_end_state(trajectories)
    raise ValueError("preplanned harvest has no final trajectory")


def _start_state_error_deg(planner, expected_positions: dict) -> float:
    current = planner._wait_for_current_joint_positions(timeout_sec=2.0)
    errors = [
        abs(float(current[name]) - float(expected))
        for name, expected in expected_positions.items()
        if name in current
    ]
    if len(errors) != len(expected_positions):
        return math.inf
    return math.degrees(max(errors, default=0.0))


def main(args=None) -> None:
    config = json.loads(os.environ.get(CONFIG_ENV, "{}"))
    tomato_count = int(config.get("tomato_count", 0))
    continuous_arc = bool(config.get("continuous_arc", False))
    execute = bool(config.get("execute", True))
    if tomato_count <= 0:
        raise SystemExit("tomato_count must be greater than zero")

    rclpy.init(args=args)
    planner = CartesianHarvestPlanner()
    exit_code = 1
    cached: list[tuple[int, HarvestMotionPlan, dict]] = []
    next_start_state = None
    next_start_pose = None
    try:
        _emit(
            "preplan_started",
            tomato_count=tomato_count,
            continuous_arc=continuous_arc,
        )
        for index in range(tomato_count):
            use_arc = continuous_arc and index > 0
            return_to_ready = not continuous_arc or index == tomato_count - 1
            planner.set_parameters(
                [
                    Parameter(
                        "tomato_frame",
                        value=f"detected_tomato_{index}_tf",
                    ),
                    Parameter("continuous_transition", value=use_arc),
                    Parameter(
                        "return_to_pick_ready",
                        value=return_to_ready,
                    ),
                    Parameter("retreat_after_harvest", value=False),
                ]
            )
            # CartesianHarvestPlanner caches this frequently used target name
            # at construction time; keep the cache aligned with the parameter.
            planner.tomato_frame = f"detected_tomato_{index}_tf"
            _emit(
                "planning",
                index=index,
                number=index + 1,
                total=tomato_count,
                continuous_arc=use_arc,
            )
            plan = planner.plan(
                start_state_override=next_start_state,
                start_pose_override=(next_start_pose if use_arc else None),
            )
            report = copy.deepcopy(planner.last_plan_report)
            success = plan is not None
            _emit(
                "planned",
                index=index,
                success=success,
                report=report,
            )
            if not success:
                _emit(
                    "aborted",
                    index=index,
                    reason="PREPLANNING_FAILED",
                    report=report,
                )
                raise SystemExit(1)
            cached.append((index, plan, report))
            next_start_state = _final_state(planner, plan)
            next_start_pose = copy.deepcopy(plan.end_planning_pose)

        _emit("preplan_complete", tomato_count=tomato_count)
        if not execute:
            exit_code = 0
            _emit("complete", tomato_count=tomato_count, executed=False)
            return

        expected_start = cached[0][2].get("start_joint_positions", {})
        start_error_deg = _start_state_error_deg(planner, expected_start)
        tolerance_deg = float(config.get("start_tolerance_deg", 3.0))
        if not math.isfinite(start_error_deg) or start_error_deg > tolerance_deg:
            report = {
                "failure_stage": "PREPLANNED_START_STATE_MISMATCH",
                "failure_planner_type": "validation",
                "failure_reason": "ROBOT_MOVED_AFTER_PREPLANNING",
                "start_error_deg": start_error_deg,
                "execution_attempted": False,
            }
            _emit("aborted", index=0, reason="START_STATE_CHANGED", report=report)
            raise SystemExit(1)

        for number, (index, plan, report) in enumerate(cached, start=1):
            planner.last_plan_report = report
            _emit(
                "executing",
                index=index,
                number=number,
                total=tomato_count,
            )
            started = time.monotonic()
            success = planner.execute(plan)
            report["execution_requested"] = True
            report["execution_attempted"] = True
            report["execution_success"] = bool(success)
            report["execution_duration_sec"] = round(
                time.monotonic() - started,
                6,
            )
            if not success:
                report["failure_stage"] = "EXECUTION"
                report["failure_planner_type"] = "execution"
                report["failure_reason"] = "CACHED_TRAJECTORY_EXECUTION_FAILED"
            _emit(
                "executed",
                index=index,
                success=bool(success),
                report=report,
            )
            if not success:
                _emit(
                    "aborted",
                    index=index,
                    reason="EXECUTION_FAILED",
                    report=report,
                )
                raise SystemExit(1)

        exit_code = 0
        _emit("complete", tomato_count=tomato_count, executed=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        _emit("aborted", index=-1, reason="INTERRUPTED", report={})
    except Exception as error:
        planner.get_logger().error(
            f"Preplanned tomato batch failed: {error!r}"
        )
        _emit(
            "aborted",
            index=-1,
            reason="UNEXPECTED_EXCEPTION",
            report={
                "failure_stage": "UNEXPECTED_EXCEPTION",
                "failure_reason": repr(error),
            },
        )
    finally:
        planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
