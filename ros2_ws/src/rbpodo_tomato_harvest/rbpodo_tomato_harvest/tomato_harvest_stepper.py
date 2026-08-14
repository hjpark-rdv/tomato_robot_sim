"""Persist one detailed harvest plan and execute it one stage at a time."""

from __future__ import annotations

import json
import math
import sys
import time
from copy import deepcopy

import rclpy
from builtin_interfaces.msg import Duration as DurationMessage
from moveit_msgs.msg import RobotState
from rclpy.executors import ExternalShutdownException

from rbpodo_tomato_harvest.harvest_planner import (
    CartesianHarvestPlanner,
    HarvestMotionPlan,
)


EVENT_PREFIX = "__HARVEST_STEPPER_EVENT__"
CYCLE_LAST_STAGE_INDEX = 6


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
    wrist_rotation_deg: float = 10.0,
    ready_state_name: str = "PICK_READY",
) -> list[dict]:
    """Return the ordered, cached execution groups exposed in the GUI."""
    approach = tuple(plan.step_approach_trajectories)
    if len(approach) != 6:
        raise ValueError(
            "stepwise plan must contain six detailed approach groups"
        )
    approach_waypoints = tuple(
        getattr(plan, "step_approach_waypoints", ())
    )
    if not approach_waypoints:
        approach_waypoints = ((),) * 6
    if len(approach_waypoints) != 6:
        raise ValueError(
            "stepwise plan must contain six detailed approach waypoint groups"
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
    ready_state_name = str(ready_state_name).strip() or "PICK_READY"
    wrist_enabled = bool(_trajectory_group(approach[2]))

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
            "label": f"현재 자세 → {ready_state_name}",
            "detail": "선택한 시작 자세로 이동",
            "kind": "trajectory",
            "trajectories": _trajectory_group(plan.pick_ready_trajectory),
        },
        {
            "key": "READY_TO_PREAPPROACH",
            "label": f"{ready_state_name} → PRE_APPROACH",
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
            "cartesian_waypoints": approach_waypoints[0],
        },
        {
            "key": "FORWARD_X",
            "label": "접근 목표 → 앞으로 이동",
            "detail": delta_detail(1),
            "kind": "trajectory",
            "trajectories": _trajectory_group(approach[1]),
            "cartesian_waypoints": approach_waypoints[1],
        },
        {
            "key": "TCP_WRIST_OSCILLATION",
            "label": "TCP 제자리 회전",
            "detail": (
                f"wrist3만 -{abs(float(wrist_rotation_deg)):g}° → "
                f"+{abs(float(wrist_rotation_deg)):g}° → 원래 각도"
                if wrist_enabled
                else "사용 안 함 (체크 해제)"
            ),
            "kind": "trajectory" if wrist_enabled else "skip",
            "trajectories": _trajectory_group(approach[2]),
            "cartesian_waypoints": (),
        },
        {
            "key": "LIFT_Z20_FORWARD_X20",
            "label": "위로 1차 이동",
            "detail": f"{delta_detail(2)} / 5→6 곡선 Cartesian",
            "kind": "trajectory",
            "trajectories": _trajectory_group(approach[3]),
            "cartesian_waypoints": approach_waypoints[3],
        },
        {
            "key": "LIFT_Z20_SECOND",
            "label": "위로 2차 이동",
            "detail": f"{delta_detail(3)} / 6→7 곡선 Cartesian",
            "kind": "trajectory",
            "trajectories": _trajectory_group(approach[4]),
            "cartesian_waypoints": approach_waypoints[4],
        },
        {
            "key": "BACK_X50_FIRST",
            "label": "뒤로 1차 이동",
            "detail": delta_detail(4),
            "kind": "trajectory",
            "trajectories": _trajectory_group(approach[5]),
            "cartesian_waypoints": approach_waypoints[5],
        },
        {
            "key": "BACK_X10_SECOND",
            "label": "뒤로 2차 이동",
            "detail": "tip 로컬 -X 10 mm",
            "kind": "trajectory",
            "trajectories": _trajectory_group(plan.after_wait_trajectory),
            "cartesian_waypoints": tuple(
                getattr(plan, "after_wait_waypoints", ())
            ),
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
            "label": f"현재 자세 → {ready_state_name}",
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


def continuous_cartesian_stage_blocks(
    stages: list[dict],
    start_index: int,
    target_index: int,
) -> list[dict]:
    """Find mergeable Cartesian runs without crossing hard stage boundaries."""
    blocks = []
    cursor = max(0, int(start_index))
    final_index = min(int(target_index), len(stages) - 1)
    while cursor <= final_index:
        stage = stages[cursor]
        if not (
            stage.get("kind") == "trajectory"
            and stage.get("cartesian_waypoints")
        ):
            cursor += 1
            continue

        block_start = cursor
        block_end = cursor - 1
        movement_count = 0
        waypoints = []
        stage_indices = []
        while cursor <= final_index:
            candidate = stages[cursor]
            candidate_waypoints = tuple(
                candidate.get("cartesian_waypoints", ())
            )
            if candidate.get("kind") == "trajectory" and candidate_waypoints:
                movement_count += 1
                waypoints.extend(candidate_waypoints)
                stage_indices.append(cursor)
                block_end = cursor
                cursor += 1
                continue
            if candidate.get("kind") == "skip":
                stage_indices.append(cursor)
                block_end = cursor
                cursor += 1
                continue
            break

        if movement_count >= 2:
            blocks.append(
                {
                    "start_index": block_start,
                    "end_index": block_end,
                    "stage_indices": tuple(stage_indices),
                    "waypoints": tuple(waypoints),
                }
            )
    return blocks


def _robot_state_from_positions(positions: dict[str, float]) -> RobotState:
    state = RobotState()
    state.is_diff = False
    state.joint_state.name = list(positions)
    state.joint_state.position = [
        float(positions[name]) for name in state.joint_state.name
    ]
    return state


def _plan_continuous_cartesian_blocks(
    planner,
    stages: list[dict],
    start_index: int,
    target_index: int,
):
    """Plan every merged run before any cached stage starts executing."""
    planned = {}
    blocks = continuous_cartesian_stage_blocks(
        stages,
        start_index,
        target_index,
    )
    _emit(
        "continuous_planning",
        blocks=[
            {
                "start_index": block["start_index"],
                "end_index": block["end_index"],
                "stage_indices": block["stage_indices"],
                "waypoint_count": len(block["waypoints"]),
            }
            for block in blocks
        ],
    )
    for block in blocks:
        first = block["start_index"]
        last = block["end_index"]
        label = f"Step {first + 1}-{last + 1} continuous Cartesian"
        trajectory = planner._plan_cartesian(
            block["waypoints"],
            _robot_state_from_positions(stages[first]["expected_start"]),
            label,
            terminal_failure=True,
        )
        if trajectory is None:
            _emit(
                "continuous_failed",
                start_index=first,
                end_index=last,
                stage_indices=block["stage_indices"],
                reason="CONTINUOUS_CARTESIAN_PLAN_FAILED",
            )
            return None
        planned[first] = {
            **block,
            "trajectory": trajectory,
            "label": label,
        }
    return planned


def _execute_continuous_cartesian_block(
    planner,
    stages: list[dict],
    block: dict,
) -> bool:
    first = int(block["start_index"])
    last = int(block["end_index"])
    error_deg = _start_error_deg(planner, stages[first]["expected_start"])
    if not math.isfinite(error_deg) or error_deg > 3.0:
        _emit(
            "continuous_failed",
            start_index=first,
            end_index=last,
            stage_indices=block["stage_indices"],
            reason="ROBOT_STATE_CHANGED",
            start_error_deg=error_deg,
        )
        return False

    _emit(
        "continuous_started",
        start_index=first,
        end_index=last,
        stage_indices=block["stage_indices"],
        label=block["label"],
    )
    started = time.monotonic()
    success = planner._execute_trajectory_group(
        (block["trajectory"],),
        block["label"],
    )
    duration = time.monotonic() - started
    if not success:
        _emit(
            "continuous_failed",
            start_index=first,
            end_index=last,
            stage_indices=block["stage_indices"],
            reason="TRAJECTORY_EXECUTION_FAILED",
            duration_sec=round(duration, 6),
        )
        return False
    _emit(
        "continuous_completed",
        start_index=first,
        end_index=last,
        stage_indices=block["stage_indices"],
        duration_sec=round(duration, 6),
    )
    return True


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
    elif stage["kind"] == "skip":
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
            for stage_number in (3, 4, 6, 7, 8)
        )
        stages = step_stage_specs(
            plan,
            wait_seconds,
            forward_distance_m,
            custom_stage_deltas_m,
            float(
                planner.get_parameter(
                    "harvest_tcp_wrist_rotation_deg"
                ).value
            ),
            str(
                planner.get_parameter("pick_ready_state_name").value
            ),
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
                        message=(
                            f"{CYCLE_LAST_STAGE_INDEX + 1}→1 역순 복귀를 "
                            "먼저 완료하세요."
                        ),
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

            continuous_blocks = {}
            if (
                action == "execute_through"
                and bool(command.get("merge_cartesian", False))
            ):
                continuous_blocks = _plan_continuous_cartesian_blocks(
                    planner,
                    stages,
                    next_index,
                    target_index,
                )
                if continuous_blocks is None:
                    continue

            while next_index <= target_index:
                continuous_block = continuous_blocks.get(next_index)
                if continuous_block is not None:
                    if not _execute_continuous_cartesian_block(
                        planner,
                        stages,
                        continuous_block,
                    ):
                        failed = True
                        break
                    next_index = int(continuous_block["end_index"]) + 1
                    continue
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
