"""Preplan a complete non-lift tomato batch, then execute cached trajectories."""

from __future__ import annotations

import copy
from dataclasses import replace
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
from rbpodo_tomato_harvest.tomato_harvest_stepper import (
    reverse_trajectory_group,
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
        plan.approach_trajectory,
        plan.preapproach_trajectory,
        plan.pick_ready_trajectory,
        plan.arc_reverse_recovery_trajectory,
    ):
        if trajectories:
            return planner._trajectory_end_state(trajectories)
    raise ValueError("preplanned harvest has no final trajectory")


def _trajectory_group(value) -> tuple:
    if not value:
        return ()
    if isinstance(value, (list, tuple)):
        return tuple(value)
    return (value,)


def _history_after_pick_ready(plan: HarvestMotionPlan) -> list:
    """Return the cached path from PICK_READY to this plan's final pose."""
    if plan.return_pick_ready_trajectory:
        return []
    history = []
    for trajectories in (
        plan.preapproach_trajectory,
        plan.approach_trajectory,
        plan.after_wait_trajectory,
        plan.outward_retreat_trajectory,
    ):
        history.extend(_trajectory_group(trajectories))
    return history


def _candidate_failure_is_retryable(report: dict) -> bool:
    """Retry stochastic motion-planning failures, not fixed geometry/TF errors."""
    stage = str(report.get("failure_stage") or "")
    reason = str(report.get("failure_reason") or "")
    if stage in {
        "TARGET_GEOMETRY",
        "TF_TARGET",
        "TF_ROBOT_TOOL",
        "DYNAMIC_BASE_TF_SYNC",
        "STEP_CYCLE_CONFIGURATION",
    }:
        return False
    return reason not in {
        "DEADLINE_REQUIRES_ANGLE_OVER_MAXIMUM",
        "INVALID_TOMATO_ORIENTATION",
        "INVALID_GRIPPER_STEM_ORDER",
        "TF_NOT_FOUND",
        "TOOL_TF_NOT_FOUND",
    }


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
    candidate_attempts = int(config.get("candidate_attempts", 3))
    if candidate_attempts <= 0:
        raise SystemExit("candidate_attempts must be greater than zero")
    harvest_stage_limit = config.get("harvest_stage_limit")
    if harvest_stage_limit is not None:
        harvest_stage_limit = int(harvest_stage_limit)
        if harvest_stage_limit not in (3, 4):
            raise SystemExit("harvest_stage_limit must be 3, 4, or null")
    if tomato_count <= 0:
        raise SystemExit("tomato_count must be greater than zero")

    rclpy.init(args=args)
    planner = CartesianHarvestPlanner()
    exit_code = 1
    cached: list[tuple[int, HarvestMotionPlan, dict]] = []
    next_start_state = None
    next_start_pose = None
    trajectory_history_from_ready: list = []
    skipped_count = 0
    try:
        _emit(
            "preplan_started",
            tomato_count=tomato_count,
            continuous_arc=continuous_arc,
        )
        ready_state = (
            planner._pick_ready_robot_state() if continuous_arc else None
        )
        for index in range(tomato_count):
            use_arc = continuous_arc and bool(cached)
            # Arc mode first validates this tomato independently from READY.
            # The already validated motion is retained even when its incoming
            # Arc cannot be planned.
            candidate_is_independent = continuous_arc
            return_to_ready = not continuous_arc
            planner.set_parameters(
                [
                    Parameter(
                        "tomato_frame",
                        value=f"detected_tomato_{index}_tf",
                    ),
                    Parameter(
                        "continuous_transition",
                        value=False if candidate_is_independent else use_arc,
                    ),
                    Parameter(
                        "return_to_pick_ready",
                        value=return_to_ready,
                    ),
                    Parameter("retreat_after_harvest", value=False),
                    Parameter(
                        "stepwise_plan",
                        value=harvest_stage_limit is not None,
                    ),
                    Parameter(
                        "step_cycle_only",
                        value=harvest_stage_limit is not None,
                    ),
                    Parameter(
                        "step_cycle_last_stage",
                        value=harvest_stage_limit or 5,
                    ),
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
            plan = None
            report = {}
            attempts = candidate_attempts if continuous_arc else 1
            for attempt in range(1, attempts + 1):
                plan = planner.plan(
                    start_state_override=(
                        ready_state
                        if candidate_is_independent
                        else next_start_state
                    ),
                    start_pose_override=(
                        None
                        if candidate_is_independent
                        else (next_start_pose if use_arc else None)
                    ),
                    arc_failure_reverse_trajectory=(
                        reverse_trajectory_group(
                            trajectory_history_from_ready
                        )
                        if (
                            not candidate_is_independent
                            and use_arc
                            and trajectory_history_from_ready
                        )
                        else ()
                    ),
                )
                report = copy.deepcopy(planner.last_plan_report)
                report["independent_candidate_plan"] = bool(
                    candidate_is_independent
                )
                report["candidate_attempt"] = attempt
                report["candidate_attempt_limit"] = attempts
                if plan is not None or not _candidate_failure_is_retryable(
                    report
                ):
                    break
                if attempt < attempts:
                    _emit(
                        "candidate_retry",
                        index=index,
                        attempt=attempt,
                        next_attempt=attempt + 1,
                        attempt_limit=attempts,
                        failure_stage=report.get(
                            "failure_stage", "UNKNOWN"
                        ),
                        failure_reason=report.get(
                            "failure_reason", "UNKNOWN"
                        ),
                    )
            success = plan is not None
            if not success:
                _emit(
                    "planned",
                    index=index,
                    success=False,
                    report=report,
                )
                skipped_count += 1
                _emit(
                    "skipped",
                    index=index,
                    reason="PREPLANNING_FAILED",
                    report=report,
                )
                continue

            if continuous_arc:
                if not cached:
                    # Candidate trajectories begin exactly at READY. Add one
                    # live-current -> READY prefix only to the first accepted
                    # tomato so execution starts from the actual robot state.
                    live_start_positions = (
                        planner._wait_for_current_joint_positions(
                            timeout_sec=2.0
                        )
                    )
                    initial_ready = planner._plan_pick_ready()
                    if initial_ready is None:
                        report.update(
                            {
                                "success": False,
                                "failure_stage": "INITIAL_PICK_READY",
                                "failure_planner_type": "ompl",
                                "failure_reason": (
                                    "INITIAL_PICK_READY_PLANNING_FAILED"
                                ),
                            }
                        )
                        _emit(
                            "aborted",
                            index=index,
                            reason="INITIAL_PICK_READY_PLANNING_FAILED",
                            report=report,
                        )
                        raise SystemExit(1)
                    ready_trajectory, display_start_state = initial_ready
                    plan = replace(
                        plan,
                        pick_ready_trajectory=ready_trajectory,
                        display_start_state=display_start_state,
                    )
                    if live_start_positions:
                        report["start_joint_positions"] = {
                            str(name): float(position)
                            for name, position in live_start_positions.items()
                        }
                    report.update(
                        {
                            "continuous_transition": False,
                            "independent_candidate_reused": True,
                            "initial_pick_ready_prefix": True,
                        }
                    )
                    trajectory_history_from_ready = (
                        _history_after_pick_ready(plan)
                    )
                else:
                    transition = planner.plan_continuous_transition_only(
                        next_start_state,
                        next_start_pose,
                        plan.preapproach_planning_pose,
                        plan.outward_axis,
                    )
                    transition_report = copy.deepcopy(
                        planner.last_plan_report
                    )
                    transition_stages = list(
                        transition_report.get("stages", [])
                    )
                    report["continuous_transition"] = True
                    report["arc_transition_report"] = transition_report
                    if transition is not None:
                        plan = replace(
                            plan,
                            arc_reverse_recovery_trajectory=(),
                            pick_ready_trajectory=(),
                            preapproach_trajectory=transition,
                        )
                        report.update(
                            {
                                "continuous_transition_direct": True,
                                "continuous_transition_arc": True,
                                "continuous_reverse_recovery": False,
                                "continuous_arc": transition_report.get(
                                    "continuous_arc", {}
                                ),
                            }
                        )
                        report["stages"] = (
                            transition_stages
                            + list(report.get("stages", []))
                        )
                        trajectory_history_from_ready.extend(
                            _history_after_pick_ready(plan)
                        )
                    else:
                        reverse_to_ready = reverse_trajectory_group(
                            trajectory_history_from_ready
                        )
                        plan = replace(
                            plan,
                            arc_reverse_recovery_trajectory=reverse_to_ready,
                            pick_ready_trajectory=(),
                        )
                        for stage in transition_stages:
                            stage.update(
                                {"discarded": True, "recovered": True}
                            )
                        report.update(
                            {
                                "continuous_transition_direct": False,
                                "continuous_transition_arc": False,
                                "continuous_reverse_recovery": True,
                                "recovery_used": True,
                                "recovery_success": True,
                                "recovery_stage": (
                                    "CACHED_REVERSE_THEN_REUSE_"
                                    "INDEPENDENT_PLAN"
                                ),
                                "recovery_reason": (
                                    "Arc 실패 후 기존 성공 경로를 역재생해 "
                                    "PICK_READY로 복귀하고, 다음 토마토의 "
                                    "독립 검증 trajectory를 재사용"
                                ),
                                "reverse_recovery_trajectory_count": len(
                                    reverse_to_ready
                                ),
                            }
                        )
                        report["stages"] = (
                            transition_stages
                            + list(report.get("stages", []))
                        )
                        trajectory_history_from_ready = (
                            _history_after_pick_ready(plan)
                        )

            _emit(
                "planned",
                index=index,
                success=True,
                report=report,
            )
            cached.append((index, plan, report))
            next_start_state = _final_state(planner, plan)
            next_start_pose = copy.deepcopy(plan.end_planning_pose)
            if not continuous_arc:
                if report.get("continuous_reverse_recovery"):
                    trajectory_history_from_ready = (
                        _history_after_pick_ready(plan)
                    )
                elif not plan.return_pick_ready_trajectory:
                    trajectory_history_from_ready.extend(
                        _history_after_pick_ready(plan)
                    )
                else:
                    trajectory_history_from_ready = []

        if not cached:
            _emit(
                "aborted",
                index=-1,
                reason="NO_PLANNABLE_TOMATOES",
                report={
                    "failure_stage": "PREPLANNING",
                    "failure_reason": "ALL_TOMATO_PLANS_FAILED",
                    "planned_count": 0,
                    "skipped_count": skipped_count,
                },
            )
            raise SystemExit(1)

        if (
            continuous_arc
            and not cached[-1][1].return_pick_ready_trajectory
            and trajectory_history_from_ready
        ):
            index, final_plan, final_report = cached[-1]
            reverse_to_ready = reverse_trajectory_group(
                trajectory_history_from_ready
            )
            final_plan = replace(
                final_plan,
                return_pick_ready_trajectory=reverse_to_ready,
                return_pick_ready_is_cached_reverse=True,
            )
            final_report["final_return_policy"] = (
                "CACHED_TRAJECTORY_REVERSE_TO_PICK_READY"
            )
            final_report["final_return_trajectory_count"] = len(
                reverse_to_ready
            )
            cached[-1] = (index, final_plan, final_report)

        _emit(
            "preplan_complete",
            tomato_count=tomato_count,
            planned_count=len(cached),
            skipped_count=skipped_count,
        )
        if not execute:
            exit_code = 0
            _emit(
                "complete",
                tomato_count=tomato_count,
                planned_count=len(cached),
                skipped_count=skipped_count,
                executed_count=0,
                executed=False,
            )
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
                total=len(cached),
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
                number=number,
                total=len(cached),
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
        _emit(
            "complete",
            tomato_count=tomato_count,
            planned_count=len(cached),
            skipped_count=skipped_count,
            executed_count=len(cached),
            executed=True,
        )
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
