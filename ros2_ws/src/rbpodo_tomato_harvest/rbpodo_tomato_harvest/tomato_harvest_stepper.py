"""Persist one detailed harvest plan and execute it one stage at a time."""

from __future__ import annotations

import json
import math
import sys
import time
from copy import deepcopy

import rclpy
from builtin_interfaces.msg import Duration as DurationMessage
from rclpy.executors import ExternalShutdownException

from rbpodo_tomato_harvest.harvest_planner import (
    CartesianHarvestPlanner,
    HarvestMotionPlan,
)


EVENT_PREFIX = "__HARVEST_STEPPER_EVENT__"
CYCLE_LAST_STAGE_INDEX = 5


def _emit(event: str, **values) -> None:
    print(
        f"{EVENT_PREFIX}{json.dumps({'event': event, **values})}",
        flush=True,
    )


def _trajectory_group(value) -> tuple:
    if not value:
        return ()
    if isinstance(value, (list, tuple)):
        return tuple(value)
    return (value,)


def _duration_nanoseconds(duration) -> int:
    return int(duration.sec) * 1_000_000_000 + int(duration.nanosec)


def _duration_message(nanoseconds: int) -> DurationMessage:
    seconds, remainder = divmod(max(0, int(nanoseconds)), 1_000_000_000)
    return DurationMessage(sec=seconds, nanosec=remainder)


def reverse_robot_trajectory(trajectory):
    """Reverse one cached joint trajectory without replanning its path."""
    reversed_trajectory = deepcopy(trajectory)
    joint_trajectory = reversed_trajectory.joint_trajectory
    points = list(joint_trajectory.points)
    if not points:
        return reversed_trajectory
    if reversed_trajectory.multi_dof_joint_trajectory.points:
        raise ValueError("multi-DOF trajectories are not supported")
    first_time = _duration_nanoseconds(points[0].time_from_start)
    total_time = _duration_nanoseconds(points[-1].time_from_start)
    reversed_points = []
    for source in reversed(points):
        point = deepcopy(source)
        point.time_from_start = _duration_message(
            first_time
            + total_time
            - _duration_nanoseconds(source.time_from_start)
        )
        if point.velocities:
            point.velocities = [-float(value) for value in point.velocities]
        reversed_points.append(point)
    joint_trajectory.points = reversed_points
    return reversed_trajectory


def reverse_trajectory_group(trajectories) -> tuple:
    """Reverse segment order and every trajectory inside the group."""
    return tuple(
        reverse_robot_trajectory(trajectory)
        for trajectory in reversed(_trajectory_group(trajectories))
    )


def step_stage_specs(
    plan: HarvestMotionPlan,
    wait_seconds: float,
    forward_distance_m: float = 0.040,
    custom_stage_deltas_m=None,
) -> list[dict]:
    """Return the ordered, cached execution groups exposed in the GUI."""
    approach = tuple(plan.step_approach_trajectories)
    if len(approach) != 5:
        raise ValueError(
            "stepwise plan must contain five detailed approach groups"
        )
    if custom_stage_deltas_m is None:
        custom_stage_deltas_m = (
            (0.010, 0.0, 0.0),
            (float(forward_distance_m), 0.0, 0.0),
            (0.020, 0.0, 0.020),
            (0.0, 0.0, 0.020),
            (-0.050, 0.0, 0.0),
        )
    custom_stage_deltas_m = tuple(
        tuple(float(value) for value in stage_delta)
        for stage_delta in custom_stage_deltas_m
    )
    if (
        len(custom_stage_deltas_m) != 5
        or any(len(stage_delta) != 3 for stage_delta in custom_stage_deltas_m)
    ):
        raise ValueError("custom stage deltas must contain five XYZ triples")

    def delta_detail(stage_index: int) -> str:
        xyz_mm = tuple(
            value * 1000.0 for value in custom_stage_deltas_m[stage_index]
        )
        return (
            f"tip 로컬 X {xyz_mm[0]:+.1f} / "
            f"Y {xyz_mm[1]:+.1f} / Z {xyz_mm[2]:+.1f} mm"
        )

    return [
        {
            "key": "MOVE_TO_READY",
            "label": "현재 자세 → PICK_READY",
            "detail": "선택한 시작 자세로 이동",
            "kind": "trajectory",
            "trajectories": _trajectory_group(plan.pick_ready_trajectory),
        },
        {
            "key": "READY_TO_PREAPPROACH",
            "label": "PICK_READY → PRE_APPROACH",
            "detail": "토마토 외곽 사전 접근",
            "kind": "trajectory",
            "trajectories": _trajectory_group(plan.preapproach_trajectory),
        },
        {
            "key": "PREAPPROACH_TO_TARGET",
            "label": "PRE_APPROACH → 접근 목표",
            "detail": delta_detail(0),
            "kind": "trajectory",
            "trajectories": _trajectory_group(approach[0]),
        },
        {
            "key": "FORWARD_X",
            "label": "접근 목표 → 앞으로 이동",
            "detail": delta_detail(1),
            "kind": "trajectory",
            "trajectories": _trajectory_group(approach[1]),
        },
        {
            "key": "LIFT_Z20_FORWARD_X20",
            "label": "위로 1차 이동",
            "detail": delta_detail(2),
            "kind": "trajectory",
            "trajectories": _trajectory_group(approach[2]),
        },
        {
            "key": "LIFT_Z20_SECOND",
            "label": "위로 2차 이동",
            "detail": f"{delta_detail(3)} / 5→6 곡선 Cartesian",
            "kind": "trajectory",
            "trajectories": _trajectory_group(approach[3]),
        },
        {
            "key": "BACK_X50_FIRST",
            "label": "뒤로 1차 이동",
            "detail": delta_detail(4),
            "kind": "trajectory",
            "trajectories": _trajectory_group(approach[4]),
        },
        {
            "key": "BACK_X10_SECOND",
            "label": "뒤로 2차 이동",
            "detail": "tip 로컬 -X 10 mm",
            "kind": "trajectory",
            "trajectories": _trajectory_group(plan.after_wait_trajectory),
        },
        {
            "key": "HARVEST_WAIT",
            "label": "리니어모터 대기",
            "detail": f"{max(0.0, float(wait_seconds)):.2f}초 대기",
            "kind": "wait",
            "wait_seconds": max(0.0, float(wait_seconds)),
            "trajectories": (),
        },
        {
            "key": "RETURN_READY",
            "label": "현재 자세 → PICK_READY",
            "detail": "constrained OMPL 복귀",
            "kind": "trajectory",
            "trajectories": _trajectory_group(
                plan.return_pick_ready_trajectory
            ),
        },
    ]


def _end_positions(trajectories, previous: dict[str, float]) -> dict[str, float]:
    group = _trajectory_group(trajectories)
    if not group:
        return dict(previous)
    trajectory = group[-1].joint_trajectory
    if not trajectory.points:
        return dict(previous)
    result = dict(previous)
    result.update(
        zip(trajectory.joint_names, trajectory.points[-1].positions)
    )
    return {str(name): float(value) for name, value in result.items()}


def _start_error_deg(planner, expected: dict[str, float]) -> float:
    if not expected:
        return math.inf
    current = planner._wait_for_current_joint_positions(timeout_sec=2.0)
    errors = [
        abs(float(current[name]) - float(value))
        for name, value in expected.items()
        if name in current
    ]
    if len(errors) != len(expected):
        return math.inf
    return math.degrees(max(errors, default=0.0))


def _stage_metadata(stages: list[dict]) -> list[dict]:
    return [
        {
            "index": index,
            "key": stage["key"],
            "label": stage["label"],
            "detail": stage["detail"],
            "kind": stage["kind"],
            "trajectory_count": len(stage.get("trajectories", ())),
        }
        for index, stage in enumerate(stages)
    ]


def cycle_last_stage_index(command: dict, stage_count: int) -> int:
    """Validate the GUI-selected final stage for a 1↔X cycle."""
    last_index = int(
        command.get("last_stage_index", CYCLE_LAST_STAGE_INDEX)
    )
    maximum = min(CYCLE_LAST_STAGE_INDEX, int(stage_count) - 1)
    if last_index < 0 or last_index > maximum:
        raise ValueError(
            f"반복 마지막 단계는 1~{maximum + 1} 범위여야 합니다."
        )
    return last_index


def _execute_cached_stage(planner, stage: dict, index: int, reverse: bool) -> bool:
    direction = "reverse" if reverse else "forward"
    expected = (
        stage["expected_end"] if reverse else stage["expected_start"]
    )
    error_deg = _start_error_deg(planner, expected)
    if not math.isfinite(error_deg) or error_deg > 3.0:
        _emit(
            "stage_failed",
            index=index,
            key=stage["key"],
            direction=direction,
            reason="ROBOT_STATE_CHANGED",
            start_error_deg=error_deg,
        )
        return False

    label = stage["label"]
    execution_label = f"{label} 역재생" if reverse else label
    _emit(
        "stage_started",
        index=index,
        key=stage["key"],
        label=label,
        direction=direction,
    )
    started = time.monotonic()
    if stage["kind"] == "wait":
        time.sleep(float(stage["wait_seconds"]))
        success = True
    else:
        trajectories = stage["trajectories"]
        if reverse:
            trajectories = reverse_trajectory_group(trajectories)
        success = planner._execute_trajectory_group(
            trajectories,
            execution_label,
        )
    duration = time.monotonic() - started
    if not success:
        _emit(
            "stage_failed",
            index=index,
            key=stage["key"],
            direction=direction,
            reason="TRAJECTORY_EXECUTION_FAILED",
            duration_sec=round(duration, 6),
        )
        return False
    _emit(
        "stage_completed",
        index=index,
        key=stage["key"],
        direction=direction,
        duration_sec=round(duration, 6),
    )
    return True


def main(args=None) -> None:
    rclpy.init(args=args)
    planner = CartesianHarvestPlanner()
    exit_code = 1
    try:
        plan = planner.plan()
        report = planner.last_plan_report
        if plan is None:
            _emit("plan_failed", report=report)
            raise SystemExit(1)

        wait_seconds = float(planner.get_parameter("harvest_wait_sec").value)
        forward_distance_m = float(
            planner.get_parameter("harvest_x_forward").value
        )
        custom_stage_deltas_m = tuple(
            tuple(
                float(
                    planner.get_parameter(
                        f"step_stage_{stage_number}_{axis_name}_delta"
                    ).value
                )
                for axis_name in ("x", "y", "z")
            )
            for stage_number in range(3, 8)
        )
        stages = step_stage_specs(
            plan,
            wait_seconds,
            forward_distance_m,
            custom_stage_deltas_m,
        )
        expected = {
            str(name): float(value)
            for name, value in report.get("start_joint_positions", {}).items()
        }
        for stage in stages:
            stage["expected_start"] = dict(expected)
            expected = _end_positions(stage.get("trajectories", ()), expected)
            stage["expected_end"] = dict(expected)
        _emit(
            "planned",
            stages=_stage_metadata(stages),
            report=report,
        )

        next_index = 0
        failed = False
        for line in sys.stdin:
            try:
                command = json.loads(line)
            except json.JSONDecodeError as error:
                _emit("command_error", message=f"JSON 오류: {error}")
                continue
            action = str(command.get("command", ""))
            if action == "close":
                exit_code = 0
                _emit("closed", next_index=next_index)
                break
            if failed:
                _emit("command_error", message="실패한 스텝 세션입니다.")
                continue
            if action == "execute_previous":
                if next_index <= 0:
                    _emit(
                        "command_error",
                        message="현재 위치보다 이전에 실행된 단계가 없습니다.",
                    )
                    continue
                target_index = next_index - 1
                if not _execute_cached_stage(
                    planner,
                    stages[target_index],
                    target_index,
                    reverse=True,
                ):
                    failed = True
                    continue
                next_index = target_index
                _emit(
                    "paused",
                    next_index=next_index,
                    direction="reverse",
                )
                continue
            cycle_reverse = action == "execute_cycle_reverse"
            if cycle_reverse:
                try:
                    cycle_last_index = cycle_last_stage_index(
                        command,
                        len(stages),
                    )
                except (TypeError, ValueError) as error:
                    _emit("command_error", message=str(error))
                    continue
                if next_index != cycle_last_index + 1:
                    _emit(
                        "command_error",
                        message=(
                            f"먼저 1→{cycle_last_index + 1} 연속 동작을 "
                            "완료하세요."
                        ),
                    )
                    continue
                for index in range(cycle_last_index, -1, -1):
                    if not _execute_cached_stage(
                        planner,
                        stages[index],
                        index,
                        reverse=True,
                    ):
                        failed = True
                        break
                    next_index = index
                if not failed:
                    next_index = 0
                    _emit("cycle_reset", next_index=next_index)
                continue

            if next_index >= len(stages):
                _emit(
                    "command_error",
                    message=(
                        "전체 단계를 완료했습니다. 이전 단계 역순 실행 또는 "
                        "세션 종료를 선택하세요."
                    ),
                )
                continue
            if action == "execute_next":
                target_index = next_index
            elif action == "execute_cycle_forward":
                if next_index != 0:
                    _emit(
                        "command_error",
                        message="5→1 역순 복귀를 먼저 완료하세요.",
                    )
                    continue
                try:
                    target_index = cycle_last_stage_index(
                        command,
                        len(stages),
                    )
                except (TypeError, ValueError) as error:
                    _emit("command_error", message=str(error))
                    continue
            elif action == "execute_through":
                target_index = int(command.get("stage_index", -1))
                if target_index < next_index or target_index >= len(stages):
                    _emit(
                        "command_error",
                        message=(
                            f"실행 가능한 단계는 {next_index + 1}~"
                            f"{len(stages)}입니다."
                        ),
                    )
                    continue
            else:
                _emit("command_error", message=f"알 수 없는 명령: {action}")
                continue

            while next_index <= target_index:
                stage = stages[next_index]
                if not _execute_cached_stage(
                    planner,
                    stage,
                    next_index,
                    reverse=False,
                ):
                    failed = True
                    break
                next_index += 1
            if failed:
                continue
            if next_index >= len(stages):
                _emit("session_complete", next_index=next_index)
                continue
            _emit("paused", next_index=next_index, direction="forward")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        _emit("internal_error", message=repr(error))
    finally:
        planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
