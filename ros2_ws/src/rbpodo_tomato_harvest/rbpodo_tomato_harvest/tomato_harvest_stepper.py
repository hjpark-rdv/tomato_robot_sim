"""Persist one detailed harvest plan and execute it one stage at a time."""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from copy import deepcopy

import rclpy
from builtin_interfaces.msg import Duration as DurationMessage
from moveit_msgs.msg import RobotState
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

from rbpodo_tomato_harvest.harvest_planner import (
    CartesianHarvestPlanner,
    HarvestMotionPlan,
)


EVENT_PREFIX = "__HARVEST_STEPPER_EVENT__"
CYCLE_LAST_STAGE_INDEX = 7
SPEED_CONTROLLED_STAGE_NUMBERS = (*range(4, 10), 13)
SERVO_COMMAND_TOPIC = "/linear_motor/servo10_command"
LINEAR_MOTOR_PIN8_TOPIC = "/linear_motor/pin8"
LINEAR_MOTOR_PIN9_TOPIC = "/linear_motor/pin9"
LINEAR_MOTOR_RUN_SECONDS = 3.0
SERVO_CLOSE_ANGLE_DEG = 110.0
SERVO_OPEN_ANGLE_DEG = 159.0
SERVO_DWELL_SECONDS = 0.7
PLAN_ATTEMPT_COUNT = 4


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


def scale_robot_trajectory_speed(trajectory, speed_percent: float):
    """Return a cached trajectory retimed to 10~100% of planned speed."""
    percent = float(speed_percent)
    if not math.isfinite(percent) or not 10.0 <= percent <= 100.0:
        raise ValueError("stage speed percent must be between 10 and 100")
    scale = percent / 100.0
    scaled = deepcopy(trajectory)

    acceleration_scale = scale * scale
    for point in scaled.joint_trajectory.points:
        nanoseconds = _duration_nanoseconds(point.time_from_start)
        point.time_from_start = _duration_message(
            round(nanoseconds / scale)
        )
        if point.velocities:
            point.velocities = [
                float(value) * scale for value in point.velocities
            ]
        if point.accelerations:
            point.accelerations = [
                float(value) * acceleration_scale
                for value in point.accelerations
            ]
    for point in scaled.multi_dof_joint_trajectory.points:
        nanoseconds = _duration_nanoseconds(point.time_from_start)
        point.time_from_start = _duration_message(
            round(nanoseconds / scale)
        )
        for twist in point.velocities:
            for vector in (twist.linear, twist.angular):
                vector.x *= scale
                vector.y *= scale
                vector.z *= scale
        for twist in point.accelerations:
            for vector in (twist.linear, twist.angular):
                vector.x *= acceleration_scale
                vector.y *= acceleration_scale
                vector.z *= acceleration_scale
    return scaled


def scale_trajectory_group_speed(trajectories, speed_percent: float) -> tuple:
    """Retiming helper for one stage containing one or more trajectories."""
    return tuple(
        scale_robot_trajectory_speed(trajectory, speed_percent)
        for trajectory in _trajectory_group(trajectories)
    )


def scale_trajectory_group_speeds(
    trajectories,
    speed_percents,
) -> tuple:
    """Apply one explicit speed to each cached trajectory in a group."""
    sequence = _trajectory_group(trajectories)
    percents = tuple(float(value) for value in speed_percents)
    if len(sequence) != len(percents):
        raise ValueError(
            "trajectory group and speed percentage counts must match"
        )
    return tuple(
        scale_robot_trajectory_speed(trajectory, percent)
        for trajectory, percent in zip(sequence, percents)
    )


def _continuous_joint_velocities(
    positions,
    durations,
    maximum_velocities,
):
    """Estimate conservative C1-continuous velocities at path samples."""
    point_count = len(positions)
    joint_count = len(positions[0])
    velocities = [[0.0] * joint_count for _ in range(point_count)]
    for point_index in range(1, point_count - 1):
        previous_dt = durations[point_index - 1]
        next_dt = durations[point_index]
        for joint_index in range(joint_count):
            previous_slope = (
                positions[point_index][joint_index]
                - positions[point_index - 1][joint_index]
            ) / previous_dt
            next_slope = (
                positions[point_index + 1][joint_index]
                - positions[point_index][joint_index]
            ) / next_dt
            if previous_slope * next_slope <= 0.0:
                continue
            velocity = math.copysign(
                min(abs(previous_slope), abs(next_slope)),
                previous_slope,
            )
            point_velocity_limit = min(
                maximum_velocities[point_index - 1],
                maximum_velocities[point_index],
            )
            velocities[point_index][joint_index] = max(
                -point_velocity_limit,
                min(point_velocity_limit, velocity),
            )
    return velocities


def _cubic_segment_limit_ratios(
    start_positions,
    end_positions,
    start_velocities,
    end_velocities,
    duration: float,
    maximum_velocity: float,
    maximum_acceleration: float,
) -> tuple[float, float]:
    """Return peak velocity/acceleration ratios for one Hermite segment."""
    velocity_ratio = 0.0
    acceleration_ratio = 0.0
    for q0, q1, v0, v1 in zip(
        start_positions,
        end_positions,
        start_velocities,
        end_velocities,
    ):
        # Velocity of a cubic Hermite segment is A*s^2+B*s+C, s in [0, 1].
        coefficient_a = (
            6.0 * (q0 - q1) + 3.0 * duration * (v0 + v1)
        ) / duration
        coefficient_b = (
            -6.0 * q0
            + 6.0 * q1
            - 4.0 * duration * v0
            - 2.0 * duration * v1
        ) / duration
        velocity_candidates = [abs(v0), abs(v1)]
        if abs(coefficient_a) > 1e-12:
            critical = -coefficient_b / (2.0 * coefficient_a)
            if 0.0 < critical < 1.0:
                critical_velocity = (
                    coefficient_a * critical * critical
                    + coefficient_b * critical
                    + v0
                )
                velocity_candidates.append(abs(critical_velocity))
        velocity_ratio = max(
            velocity_ratio,
            max(velocity_candidates) / maximum_velocity,
        )

        acceleration_start = (
            6.0 * (q1 - q0) / (duration * duration)
            - (4.0 * v0 + 2.0 * v1) / duration
        )
        acceleration_end = (
            -6.0 * (q1 - q0) / (duration * duration)
            + (2.0 * v0 + 4.0 * v1) / duration
        )
        acceleration_ratio = max(
            acceleration_ratio,
            abs(acceleration_start) / maximum_acceleration,
            abs(acceleration_end) / maximum_acceleration,
        )
    return velocity_ratio, acceleration_ratio


def _scalar_profile_candidate(
    positions,
    maximum_velocities,
    maximum_acceleration: float,
    velocity_factor: float,
):
    """Build and validate one density-independent path timing candidate.

    The old retimer accelerated from zero within the very first sampled
    Cartesian segment.  Consequently, making ``max_step`` smaller added more
    acceleration ramps and made an otherwise identical path much slower.  A
    scalar path profile instead accelerates over the cumulative path length,
    so sampling density does not become an artificial speed limit.
    """
    candidate_velocities = [
        float(limit) * float(velocity_factor)
        for limit in maximum_velocities
    ]
    nominal_lengths = []
    for index, velocity_limit in enumerate(candidate_velocities):
        nominal_lengths.append(
            max(
                abs(end - start) / velocity_limit
                for start, end in zip(
                    positions[index],
                    positions[index + 1],
                )
            )
        )
    cumulative_lengths = [0.0]
    for length in nominal_lengths:
        cumulative_lengths.append(cumulative_lengths[-1] + length)
    total_length = cumulative_lengths[-1]
    if total_length <= 1e-12:
        raise ValueError("merged trajectory path length is zero")

    # dq/ds never exceeds the largest candidate joint speed.  Limiting the
    # scalar acceleration by a_max / max(dq/ds) therefore supplies a safe
    # initial ramp before the exact cubic check below.
    scalar_acceleration = maximum_acceleration / max(candidate_velocities)
    profile_speeds = []
    for path_position in cumulative_lengths:
        acceleration_speed = math.sqrt(
            max(0.0, 2.0 * scalar_acceleration * path_position)
        )
        deceleration_speed = math.sqrt(
            max(
                0.0,
                2.0
                * scalar_acceleration
                * (total_length - path_position),
            )
        )
        profile_speeds.append(
            min(1.0, acceleration_speed, deceleration_speed)
        )

    durations = []
    for index, length in enumerate(nominal_lengths):
        speed_sum = profile_speeds[index] + profile_speeds[index + 1]
        if speed_sum <= 1e-9:
            # A two-point path has no interior sample on which the triangular
            # profile can expose its peak speed.
            durations.append(
                2.0 * math.sqrt(length / scalar_acceleration)
            )
        else:
            durations.append(2.0 * length / speed_sum)

    point_velocities = _continuous_joint_velocities(
        positions,
        durations,
        candidate_velocities,
    )
    required_global_scale = 1.0
    for index, duration in enumerate(durations):
        velocity_ratio, acceleration_ratio = _cubic_segment_limit_ratios(
            positions[index],
            positions[index + 1],
            point_velocities[index],
            point_velocities[index + 1],
            duration,
            maximum_velocities[index],
            maximum_acceleration,
        )
        required_global_scale = max(
            required_global_scale,
            velocity_ratio,
            math.sqrt(acceleration_ratio),
        )

    # Uniform scaling preserves the smooth scalar profile.  Scaling isolated
    # tiny segments was the source of the previous cascading slowdown.
    required_global_scale *= 1.001
    durations = [
        duration * required_global_scale for duration in durations
    ]
    point_velocities = _continuous_joint_velocities(
        positions,
        durations,
        maximum_velocities,
    )
    return durations, point_velocities


def merge_and_retime_robot_trajectories(
    trajectories,
    *,
    maximum_velocity: float = 3.14,
    maximum_acceleration: float = 4.0,
    speed_percents=None,
):
    """Merge cached paths and retime all boundaries as one smooth trajectory.

    Joint positions are never modified.  Duplicate boundary samples are
    removed, while timestamps and waypoint velocities are recomputed together
    so an OMPL/Cartesian boundary is not forced to zero velocity.
    """
    sequence = _trajectory_group(trajectories)
    if not sequence:
        raise ValueError("at least one trajectory is required")
    maximum_velocity = float(maximum_velocity)
    maximum_acceleration = float(maximum_acceleration)
    if (
        not math.isfinite(maximum_velocity)
        or maximum_velocity <= 0.0
        or not math.isfinite(maximum_acceleration)
        or maximum_acceleration <= 0.0
    ):
        raise ValueError(
            "positive finite velocity and acceleration are required"
        )
    if speed_percents is None:
        speed_percents = (100.0,) * len(sequence)
    speed_percents = tuple(float(value) for value in speed_percents)
    if len(speed_percents) != len(sequence) or any(
        not math.isfinite(value) or not 10.0 <= value <= 100.0
        for value in speed_percents
    ):
        raise ValueError(
            "one speed percentage between 10 and 100 is required for "
            "each trajectory"
        )

    joint_names = list(sequence[0].joint_trajectory.joint_names)
    if not joint_names:
        raise ValueError("trajectory joint names are empty")
    positions = []
    segment_speed_scales = []
    for trajectory, speed_percent in zip(sequence, speed_percents):
        if trajectory.multi_dof_joint_trajectory.points:
            raise ValueError("multi-DOF trajectories cannot be merged")
        source_names = list(trajectory.joint_trajectory.joint_names)
        if (
            len(source_names) != len(joint_names)
            or len(set(source_names)) != len(source_names)
            or set(source_names) != set(joint_names)
        ):
            raise ValueError("trajectory joint names do not match")
        source_index = {name: index for index, name in enumerate(source_names)}
        points = list(trajectory.joint_trajectory.points)
        if not points:
            continue
        for point in points:
            reordered = [
                float(point.positions[source_index[name]])
                for name in joint_names
            ]
            if len(reordered) != len(joint_names) or not all(
                math.isfinite(value) for value in reordered
            ):
                raise ValueError(
                    "trajectory point positions are incomplete or non-finite"
                )
            if positions and all(
                abs(left - right) <= 1e-9
                for left, right in zip(positions[-1], reordered)
            ):
                continue
            if positions:
                segment_speed_scales.append(speed_percent / 100.0)
            positions.append(reordered)

    if len(positions) < 2:
        raise ValueError("merged trajectory needs at least two unique points")
    maximum_velocities = [
        maximum_velocity * scale for scale in segment_speed_scales
    ]
    # Trying lower cruise speeds is not an arbitrary slowdown.  On a sharply
    # curved joint path, reaching the requested maximum briefly can require a
    # longer global acceleration profile than staying below it.  Select the
    # shortest validated candidate that never exceeds the user's limit.
    candidates = []
    for velocity_factor in (
        1.0,
        0.9,
        0.8,
        0.7,
        0.6,
        0.5,
        0.4,
        0.3,
        0.2,
        0.1,
    ):
        candidate = _scalar_profile_candidate(
            positions,
            maximum_velocities,
            maximum_acceleration,
            velocity_factor,
        )
        candidates.append(candidate)
    durations, velocities = min(
        candidates,
        key=lambda candidate: sum(candidate[0]),
    )
    for index, duration in enumerate(durations):
        velocity_ratio, acceleration_ratio = _cubic_segment_limit_ratios(
            positions[index],
            positions[index + 1],
            velocities[index],
            velocities[index + 1],
            duration,
            maximum_velocities[index],
            maximum_acceleration,
        )
        if velocity_ratio > 1.001 or acceleration_ratio > 1.001:
            raise ValueError(
                "merged trajectory exceeds velocity or acceleration limits"
            )
    merged = deepcopy(sequence[0])
    merged.joint_trajectory.joint_names = joint_names
    merged.joint_trajectory.points = []
    merged.multi_dof_joint_trajectory.points = []
    elapsed = 0.0
    for index, (point_positions, point_velocities) in enumerate(
        zip(positions, velocities)
    ):
        if index:
            elapsed += durations[index - 1]
        point = JointTrajectoryPoint()
        point.positions = point_positions
        point.velocities = point_velocities
        point.accelerations = []
        point.effort = []
        point.time_from_start = _duration_message(round(elapsed * 1e9))
        merged.joint_trajectory.points.append(point)
    return merged


def step_stage_specs(
    plan: HarvestMotionPlan,
    wait_seconds: float,
    forward_distance_m: float = 0.040,
    custom_stage_deltas_m=None,
    wrist_rotation_deg: float = 10.0,
    ready_state_name: str = "PICK_READY",
    servo_speed_percent: float = 50.0,
    servo_close_angle_deg: float = SERVO_CLOSE_ANGLE_DEG,
    linear_motor_extend_seconds: float = LINEAR_MOTOR_RUN_SECONDS,
    forward_wave_enabled: bool = False,
    stage_speed_percents=None,
    preapproach_final_speed_percent: float = 30.0,
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
    servo_speed_percent = float(servo_speed_percent)
    if (
        not math.isfinite(servo_speed_percent)
        or not 1.0 <= servo_speed_percent <= 100.0
    ):
        raise ValueError("servo speed percent must be between 1 and 100")
    servo_close_angle_deg = float(servo_close_angle_deg)
    if (
        not math.isfinite(servo_close_angle_deg)
        or not 10.0 <= servo_close_angle_deg <= 173.0
    ):
        raise ValueError("servo close angle must be between 10 and 173")
    linear_motor_extend_seconds = float(linear_motor_extend_seconds)
    if (
        not math.isfinite(linear_motor_extend_seconds)
        or not 0.0 <= linear_motor_extend_seconds <= 60.0
    ):
        raise ValueError(
            "linear motor extend time must be between 0 and 60 seconds"
        )
    if stage_speed_percents is None:
        stage_speed_percents = (
            30.0,
            30.0,
            100.0,
            30.0,
            30.0,
            30.0,
            30.0,
        )
    stage_speed_percents = tuple(
        float(value) for value in stage_speed_percents
    )
    if len(stage_speed_percents) != len(SPEED_CONTROLLED_STAGE_NUMBERS) or any(
        not math.isfinite(value) or not 10.0 <= value <= 100.0
        for value in stage_speed_percents
    ):
        raise ValueError(
            "stage speed percentages must contain seven values between "
            "10 and 100"
        )
    speed_by_stage = dict(
        zip(SPEED_CONTROLLED_STAGE_NUMBERS, stage_speed_percents)
    )
    preapproach_final_speed_percent = float(
        preapproach_final_speed_percent
    )
    if not math.isfinite(preapproach_final_speed_percent) or not (
        10.0 <= preapproach_final_speed_percent <= 100.0
    ):
        raise ValueError(
            "A-to-PRE_APPROACH speed percent must be between 10 and 100"
        )
    preapproach_trajectories = _trajectory_group(
        plan.preapproach_trajectory
    )
    preapproach_uses_via = bool(
        getattr(
            plan,
            "preapproach_via_enabled",
            len(preapproach_trajectories) >= 2,
        )
    )
    if preapproach_uses_via:
        via_trajectory_count = int(
            getattr(plan, "preapproach_via_trajectory_count", 1)
        )
        via_trajectory_count = max(
            1,
            min(via_trajectory_count, len(preapproach_trajectories)),
        )
        reinspection_trajectories = preapproach_trajectories[
            :via_trajectory_count
        ]
        final_preapproach_trajectories = preapproach_trajectories[
            via_trajectory_count:
        ]
    else:
        reinspection_trajectories = ()
        final_preapproach_trajectories = preapproach_trajectories
    final_preapproach_speed = (
        preapproach_final_speed_percent if preapproach_uses_via else 100.0
    )
    return_trajectories = _trajectory_group(
        plan.return_pick_ready_trajectory
    )
    # `_plan_return_pick_ready_via()` appends the final constrained OMPL
    # READY trajectory last. Apply the stage-13 slider only to the preceding
    # current-pose -> clearance-A segment(s), preserving the OMPL scale.
    return_trajectory_speeds = (100.0,) * len(return_trajectories)
    if len(return_trajectories) >= 2:
        return_trajectory_speeds = (
            *((speed_by_stage[13],) * (len(return_trajectories) - 1)),
            100.0,
        )
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
            "key": "READY_TO_REINSPECTION_A",
            "label": (
                f"{ready_state_name} → 재촬영 A"
                if preapproach_uses_via
                else "재촬영 A 건너뜀"
            ),
            "detail": (
                "RGB 광축상 토마토 150mm 재촬영 위치"
                if preapproach_uses_via
                else "사용 안 함 (Direct PRE_APPROACH)"
            ),
            "kind": "trajectory" if reinspection_trajectories else "skip",
            "trajectories": reinspection_trajectories,
            "trajectory_speed_percents": (
                (100.0,) * len(reinspection_trajectories)
            ),
        },
        {
            "key": "REINSPECTION_A_TO_PREAPPROACH",
            "label": (
                "재촬영 A → PRE_APPROACH"
                if preapproach_uses_via
                else f"{ready_state_name} → PRE_APPROACH"
            ),
            "detail": (
                f"최종 진입 자세 이동 / 속도 "
                f"{preapproach_final_speed_percent:g}%"
                if preapproach_uses_via
                else "Direct PRE_APPROACH 이동"
            ),
            "kind": (
                "trajectory" if final_preapproach_trajectories else "skip"
            ),
            "trajectories": final_preapproach_trajectories,
            "trajectory_speed_percents": (
                (final_preapproach_speed,)
                * len(final_preapproach_trajectories)
            ),
        },
        {
            "key": "PREAPPROACH_TO_TARGET",
            "label": "PRE_APPROACH → 접근 목표",
            "detail": delta_detail(0),
            "kind": "trajectory",
            "speed_percent": speed_by_stage[4],
            "trajectories": _trajectory_group(approach[0]),
            "cartesian_waypoints": approach_waypoints[0],
        },
        {
            "key": "FORWARD_X",
            "label": "접근 목표 → 앞으로 이동",
            "detail": (
                f"{delta_detail(1)} / Z축 ±5mm × 3회 웨이브 Cartesian"
                if forward_wave_enabled
                else f"{delta_detail(1)} / 직선 Cartesian"
            ),
            "kind": "trajectory",
            "speed_percent": speed_by_stage[5],
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
            "speed_percent": speed_by_stage[6],
            "trajectories": _trajectory_group(approach[2]),
            "cartesian_waypoints": (),
        },
        {
            "key": "LIFT_Z20_FORWARD_X20",
            "label": "위로 1차 이동",
            "detail": f"{delta_detail(2)} / 6→7 곡선 Cartesian",
            "kind": "trajectory",
            "speed_percent": speed_by_stage[7],
            "trajectories": _trajectory_group(approach[3]),
            "cartesian_waypoints": approach_waypoints[3],
        },
        {
            "key": "LIFT_Z20_SECOND",
            "label": "위로 2차 이동",
            "detail": f"{delta_detail(3)} / 7→8 곡선 Cartesian",
            "kind": "trajectory",
            "speed_percent": speed_by_stage[8],
            "trajectories": _trajectory_group(approach[4]),
            "cartesian_waypoints": approach_waypoints[4],
        },
        {
            "key": "BACK_X50_FIRST",
            "label": "뒤로 1차 이동",
            "detail": delta_detail(4),
            "kind": "trajectory",
            "speed_percent": speed_by_stage[9],
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
            "key": "LINEAR_MOTOR_EXTEND",
            "label": "리니어모터 늘림",
            "detail": f"{linear_motor_extend_seconds:g}초 늘림 → 자동 정지",
            "kind": "linear_motor",
            "linear_motor_command": "extend",
            "linear_motor_duration_seconds": linear_motor_extend_seconds,
            "trajectories": (),
        },
        {
            "key": "SERVO_CLOSE_OPEN",
            "label": "서보 닫기 → 열기",
            "detail": (
                f"{servo_close_angle_deg:g}° 닫기 → "
                f"{SERVO_DWELL_SECONDS:.1f}초 대기 → "
                f"{SERVO_OPEN_ANGLE_DEG:g}° 열기 → "
                f"{SERVO_DWELL_SECONDS:.1f}초 대기 → "
                f"리니어모터 {LINEAR_MOTOR_RUN_SECONDS:.1f}초 줄임 "
                "(백그라운드 자동 정지) "
                f"(속도 {servo_speed_percent:g}%)"
            ),
            "kind": "servo_sequence",
            "close_angle_deg": servo_close_angle_deg,
            "open_angle_deg": SERVO_OPEN_ANGLE_DEG,
            "speed_percent": servo_speed_percent,
            "dwell_seconds": SERVO_DWELL_SECONDS,
            "retract_duration_seconds": LINEAR_MOTOR_RUN_SECONDS,
            "trajectories": (),
        },
        {
            "key": "RETURN_READY",
            "label": f"현재 자세 → {ready_state_name}",
            "detail": (
                "카메라 재촬영 A로 이탈 후 constrained OMPL 복귀 / "
                f"현재→A 속도 {speed_by_stage[13]:g}%"
            ),
            "kind": "trajectory",
            "speed_percent": speed_by_stage[13],
            "trajectories": return_trajectories,
            "trajectory_speed_percents": return_trajectory_speeds,
        },
    ]


def _build_step_stages(planner, plan: HarvestMotionPlan) -> list[dict]:
    """Build the GUI stage cache from the planner's current parameters."""
    return step_stage_specs(
        plan,
        float(planner.get_parameter("harvest_wait_sec").value),
        float(planner.get_parameter("harvest_x_forward").value),
        tuple(
            tuple(
                float(
                    planner.get_parameter(
                        f"step_stage_{stage_number}_{axis_name}_delta"
                    ).value
                )
                for axis_name in ("x", "y", "z")
            )
            for stage_number in (4, 5, 7, 8, 9)
        ),
        float(
            planner.get_parameter("harvest_tcp_wrist_rotation_deg").value
        ),
        str(planner.get_parameter("pick_ready_state_name").value),
        float(planner.get_parameter("step_servo_speed_percent").value),
        float(planner.get_parameter("step_servo_close_angle_deg").value),
        float(
            planner.get_parameter("step_linear_motor_extend_seconds").value
        ),
        bool(planner.get_parameter("harvest_forward_wave_enabled").value),
        stage_speed_percents=tuple(
            float(
                planner.get_parameter(
                    f"step_stage_{stage_number}_speed_percent"
                ).value
            )
            for stage_number in SPEED_CONTROLLED_STAGE_NUMBERS
        ),
        preapproach_final_speed_percent=float(
            planner.get_parameter(
                "step_preapproach_final_speed_percent"
            ).value
        ),
    )


def _current_robot_state(planner) -> RobotState:
    positions = planner._wait_for_current_joint_positions(timeout_sec=2.0)
    required_names = [
        str(name)
        for name in planner.get_parameter("pick_ready_joint_names").value
    ]
    if any(name not in positions for name in required_names):
        raise RuntimeError("CURRENT_JOINT_STATE_UNAVAILABLE")
    state = RobotState()
    state.is_diff = False
    state.joint_state.name = required_names
    state.joint_state.position = [positions[name] for name in required_names]
    return state


def _bind_stage_publishers(
    stages,
    servo_topic,
    servo_publisher,
    pin8_topic,
    pin9_topic,
    pin8_publisher,
    pin9_publisher,
) -> None:
    for stage in stages:
        if stage["kind"] == "servo_sequence":
            stage["servo_topic"] = servo_topic
            stage["servo_publisher"] = servo_publisher
        if stage["kind"] in {"linear_motor", "servo_sequence"}:
            stage["linear_motor_pin8_topic"] = pin8_topic
            stage["linear_motor_pin9_topic"] = pin9_topic
            stage["linear_motor_pin8_publisher"] = pin8_publisher
            stage["linear_motor_pin9_publisher"] = pin9_publisher


def _set_expected_states(stages, first_index, initial_positions) -> None:
    expected = {
        str(name): float(value)
        for name, value in initial_positions.items()
    }
    for stage in stages[first_index:]:
        stage["expected_start"] = dict(expected)
        expected = _end_positions(stage.get("trajectories", ()), expected)
        stage["expected_end"] = dict(expected)


def _plan_with_retries(
    planner,
    *,
    context: str,
    start_state_override=None,
    maximum_attempts: int = PLAN_ATTEMPT_COUNT,
):
    """Retry a complete step plan without moving the robot."""
    maximum_attempts = max(1, int(maximum_attempts))
    for attempt in range(1, maximum_attempts + 1):
        plan = planner.plan(start_state_override=start_state_override)
        if plan is not None:
            return plan
        if attempt < maximum_attempts:
            _emit(
                "plan_retry",
                context=context,
                failed_attempt=attempt,
                next_attempt=attempt + 1,
                maximum_attempts=maximum_attempts,
                report=planner.last_plan_report,
            )
    return None


def _replan_after_refinement(
    planner,
    stages,
    offset_xyz,
    servo_resources,
) -> tuple[list[dict], dict]:
    """Rebuild stage 3 onward while the robot remains at reinspection A."""
    offset_xyz = tuple(float(value) for value in offset_xyz)
    if len(offset_xyz) != 3 or any(
        not math.isfinite(value) for value in offset_xyz
    ):
        raise ValueError("refined offset must contain finite XYZ values")

    planner.set_parameters(
        [
            Parameter(
                f"tomato_position_offset_{axis_name}",
                Parameter.Type.DOUBLE,
                value,
            )
            for axis_name, value in zip(("x", "y", "z"), offset_xyz)
        ]
    )
    current_state = _current_robot_state(planner)
    refined_plan = _plan_with_retries(
        planner,
        context="refinement",
        start_state_override=current_state,
    )
    if refined_plan is None:
        raise RuntimeError(
            f"REFINED_TARGET_PLAN_FAILED_AFTER_{PLAN_ATTEMPT_COUNT}_ATTEMPTS"
        )
    refined_via_pose = getattr(refined_plan, "preapproach_via_pose", None)
    if refined_via_pose is None:
        raise RuntimeError("REFINED_REINSPECTION_POSE_UNAVAILABLE")

    # The full refined plan contains READY -> corrected A.  The robot is
    # already at the old A, so replace that unused segment with a direct,
    # collision-checked old-A -> corrected-A bridge.  Stage 3 then continues
    # with the freshly planned corrected-A -> PRE_APPROACH trajectory.
    corrected_a_bridge = None
    for attempt in range(1, PLAN_ATTEMPT_COUNT + 1):
        corrected_a_bridge = planner._plan_cartesian_with_ompl_fallback(
            [refined_via_pose],
            current_state,
            "Refined center: current A to corrected camera waypoint",
            pregrasp=True,
        )
        if corrected_a_bridge is not None:
            break
        if attempt < PLAN_ATTEMPT_COUNT:
            _emit(
                "plan_retry",
                context="refinement_bridge",
                failed_attempt=attempt,
                next_attempt=attempt + 1,
                maximum_attempts=PLAN_ATTEMPT_COUNT,
                report=planner.last_plan_report,
            )
    if corrected_a_bridge is None:
        raise RuntimeError(
            "REFINED_REINSPECTION_BRIDGE_FAILED_AFTER_"
            f"{PLAN_ATTEMPT_COUNT}_ATTEMPTS"
        )

    refined_stages = _build_step_stages(planner, refined_plan)
    first_replanned_index = 2
    refined_stage = refined_stages[first_replanned_index]
    refined_stage["trajectories"] = (
        *tuple(corrected_a_bridge),
        *tuple(refined_stage.get("trajectories", ())),
    )
    stage_speed = float(
        planner.get_parameter("step_preapproach_final_speed_percent").value
    )
    refined_stage["trajectory_speed_percents"] = (
        (stage_speed,) * len(refined_stage["trajectories"])
    )

    updated_stages = [*stages[:first_replanned_index], *refined_stages[2:]]
    _bind_stage_publishers(updated_stages, *servo_resources)
    current_positions = dict(
        zip(
            current_state.joint_state.name,
            current_state.joint_state.position,
        )
    )
    _set_expected_states(
        updated_stages,
        first_replanned_index,
        current_positions,
    )
    return updated_stages, planner.last_plan_report


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
            "speed_percent": float(stage.get("speed_percent", 100.0)),
            "expected_start": {
                str(name): float(value)
                for name, value in stage.get("expected_start", {}).items()
            },
            "expected_end": {
                str(name): float(value)
                for name, value in stage.get("expected_end", {}).items()
            },
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
            movement_speed_percents = [
                float(stages[index].get("speed_percent", 100.0))
                for index in stage_indices
                if stages[index].get("kind") == "trajectory"
            ]
            blocks.append(
                {
                    "start_index": block_start,
                    "end_index": block_end,
                    "stage_indices": tuple(stage_indices),
                    "waypoints": tuple(waypoints),
                    # A merged trajectory cannot change its time scale at the
                    # original stage boundaries.  Respect every requested
                    # limit by using the slowest included movement stage.
                    "speed_percent": min(movement_speed_percents),
                }
            )
    return blocks


def continuous_all_trajectory_stage_blocks(
    stages: list[dict],
    start_index: int,
    target_index: int,
) -> list[dict]:
    """Find runs that can be sent as one OMPL+Cartesian trajectory."""
    blocks = []
    cursor = max(0, int(start_index))
    final_index = min(int(target_index), len(stages) - 1)
    while cursor <= final_index:
        if stages[cursor].get("kind") not in {"trajectory", "skip"}:
            cursor += 1
            continue
        block_start = cursor
        stage_indices = []
        trajectory_count = 0
        while cursor <= final_index:
            stage = stages[cursor]
            if stage.get("kind") not in {"trajectory", "skip"}:
                break
            stage_indices.append(cursor)
            trajectory_count += len(_trajectory_group(stage.get("trajectories")))
            cursor += 1
        if trajectory_count >= 2:
            blocks.append(
                {
                    "start_index": block_start,
                    "end_index": stage_indices[-1],
                    "stage_indices": tuple(stage_indices),
                }
            )
    return blocks


def _prepare_all_trajectory_blocks(
    planner,
    stages: list[dict],
    start_index: int,
    target_index: int,
):
    """Scale each stage, concatenate all paths, and globally retime them."""
    planned = {}
    blocks = continuous_all_trajectory_stage_blocks(
        stages,
        start_index,
        target_index,
    )
    _emit(
        "continuous_planning",
        mode="ompl_cartesian",
        blocks=[dict(block) for block in blocks],
    )
    for block in blocks:
        trajectories = []
        trajectory_speed_percents = []
        ompl_velocity_percent = 100.0 * float(
            planner.get_parameter("pick_ready_velocity_scale").value
        )
        maximum_acceleration = 4.0 * float(
            planner.get_parameter("pick_ready_acceleration_scale").value
        )
        for stage_index in block["stage_indices"]:
            stage = stages[stage_index]
            if stage.get("kind") != "trajectory":
                continue
            stage_trajectories = _trajectory_group(
                stage.get("trajectories")
            )
            trajectories.extend(stage_trajectories)
            explicit_speeds = stage.get("trajectory_speed_percents")
            if explicit_speeds is not None:
                explicit_speeds = tuple(
                    float(value) for value in explicit_speeds
                )
                if len(explicit_speeds) != len(stage_trajectories):
                    _emit(
                        "continuous_failed",
                        mode="ompl_cartesian",
                        start_index=block["start_index"],
                        end_index=block["end_index"],
                        stage_indices=block["stage_indices"],
                        reason="TRAJECTORY_SPEED_COUNT_MISMATCH",
                    )
                    return None
                trajectory_speed_percents.extend(explicit_speeds)
            else:
                trajectory_speed_percents.extend(
                    [
                        float(
                            stage.get(
                                "speed_percent",
                                ompl_velocity_percent,
                            )
                        )
                    ]
                    * len(stage_trajectories)
                )
        try:
            merged = merge_and_retime_robot_trajectories(
                trajectories,
                maximum_acceleration=maximum_acceleration,
                speed_percents=trajectory_speed_percents,
            )
        except (IndexError, TypeError, ValueError) as error:
            _emit(
                "continuous_failed",
                mode="ompl_cartesian",
                start_index=block["start_index"],
                end_index=block["end_index"],
                stage_indices=block["stage_indices"],
                reason="OMPL_CARTESIAN_MERGE_FAILED",
                detail=str(error),
            )
            return None
        label = (
            f"Step {block['start_index'] + 1}-"
            f"{block['end_index'] + 1} OMPL+Cartesian merged"
        )
        merged_points = merged.joint_trajectory.points
        merged_duration = (
            _duration_nanoseconds(merged_points[-1].time_from_start) / 1e9
        )
        planned[block["start_index"]] = {
            **block,
            "trajectory": merged,
            "label": label,
            "mode": "ompl_cartesian",
            "prescaled": True,
            "requested_speed_min_percent": min(
                trajectory_speed_percents
            ),
            "requested_speed_max_percent": max(
                trajectory_speed_percents
            ),
            "maximum_acceleration": maximum_acceleration,
            "retimed_point_count": len(merged_points),
            "retimed_duration_sec": merged_duration,
        }
    return planned


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
        mode="cartesian",
        blocks=[
            {
                "start_index": block["start_index"],
                "end_index": block["end_index"],
                "stage_indices": block["stage_indices"],
                "waypoint_count": len(block["waypoints"]),
                "speed_percent": block["speed_percent"],
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
                mode="cartesian",
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
            "mode": "cartesian",
            "prescaled": False,
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
            mode=block.get("mode", "cartesian"),
            start_index=first,
            end_index=last,
            stage_indices=block["stage_indices"],
            reason="ROBOT_STATE_CHANGED",
            start_error_deg=error_deg,
        )
        return False

    _emit(
        "continuous_started",
        mode=block.get("mode", "cartesian"),
        start_index=first,
        end_index=last,
        stage_indices=block["stage_indices"],
        label=block["label"],
        speed_percent=block.get("speed_percent", 100.0),
        requested_speed_min_percent=block.get(
            "requested_speed_min_percent"
        ),
        requested_speed_max_percent=block.get(
            "requested_speed_max_percent"
        ),
        maximum_acceleration=block.get("maximum_acceleration"),
        retimed_point_count=block.get("retimed_point_count"),
        retimed_duration_sec=block.get("retimed_duration_sec"),
    )
    started = time.monotonic()
    speed_percent = float(block.get("speed_percent", 100.0))
    execution_trajectory = block["trajectory"]
    if not block.get("prescaled", False):
        execution_trajectory = scale_robot_trajectory_speed(
            execution_trajectory,
            speed_percent,
        )
    speed_suffix = (
        "전체 재타이밍"
        if block.get("prescaled", False)
        else f"속도 {speed_percent:g}%"
    )
    success = planner._execute_trajectory_group(
        (execution_trajectory,),
        f"{block['label']} ({speed_suffix})",
    )
    duration = time.monotonic() - started
    if not success:
        _emit(
            "continuous_failed",
            mode=block.get("mode", "cartesian"),
            start_index=first,
            end_index=last,
            stage_indices=block["stage_indices"],
            reason="TRAJECTORY_EXECUTION_FAILED",
            duration_sec=round(duration, 6),
        )
        return False
    _emit(
        "continuous_completed",
        mode=block.get("mode", "cartesian"),
        start_index=first,
        end_index=last,
        stage_indices=block["stage_indices"],
        duration_sec=round(duration, 6),
    )
    return True


def _linear_motor_publish(stage: dict, pin8_high: bool, pin9_high: bool) -> None:
    pin8_message = Bool()
    pin8_message.data = bool(pin8_high)
    pin9_message = Bool()
    pin9_message.data = bool(pin9_high)
    stage["linear_motor_pin8_publisher"].publish(pin8_message)
    stage["linear_motor_pin9_publisher"].publish(pin9_message)


def _execute_linear_motor_action(
    planner,
    stage: dict,
    *,
    command: str | None = None,
    duration_seconds: float | None = None,
    wait_for_completion: bool = True,
) -> bool:
    pin8_publisher = stage.get("linear_motor_pin8_publisher")
    pin9_publisher = stage.get("linear_motor_pin9_publisher")
    pin8_topic = str(
        stage.get("linear_motor_pin8_topic", LINEAR_MOTOR_PIN8_TOPIC)
    )
    pin9_topic = str(
        stage.get("linear_motor_pin9_topic", LINEAR_MOTOR_PIN9_TOPIC)
    )
    if pin8_publisher is None or pin9_publisher is None:
        planner.get_logger().error("Linear motor PIN publishers are unavailable")
        return False

    subscriber_deadline = time.monotonic() + 1.0
    while (
        (
            planner.count_subscribers(pin8_topic) < 1
            or planner.count_subscribers(pin9_topic) < 1
        )
        and time.monotonic() < subscriber_deadline
    ):
        time.sleep(0.05)
    if (
        planner.count_subscribers(pin8_topic) < 1
        or planner.count_subscribers(pin9_topic) < 1
    ):
        planner.get_logger().error(
            "Linear motor PIN subscriber is unavailable: "
            f"{pin8_topic}, {pin9_topic}"
        )
        return False

    selected_command = str(
        command or stage.get("linear_motor_command", "")
    )
    levels = {
        "extend": (True, False),
        "retract": (False, True),
    }
    if selected_command not in levels:
        planner.get_logger().error(
            f"Unsupported linear motor command: {selected_command}"
        )
        return False
    duration = max(
        0.0,
        float(
            stage.get("linear_motor_duration_seconds", 0.0)
            if duration_seconds is None
            else duration_seconds
        ),
    )

    # Match the manual GUI's safe switching sequence: neutral first, then the
    # requested direction.  Always return both outputs LOW when finished.
    _linear_motor_publish(stage, False, False)
    time.sleep(0.1)
    pin8_high, pin9_high = levels[selected_command]
    _linear_motor_publish(stage, pin8_high, pin9_high)
    planner.get_logger().info(
        f"Linear motor {selected_command} started: duration={duration:.1f}s, "
        f"PIN8={'HIGH' if pin8_high else 'LOW'}, "
        f"PIN9={'HIGH' if pin9_high else 'LOW'}"
    )
    if not wait_for_completion:
        def stop_motor() -> None:
            _linear_motor_publish(stage, False, False)
            planner.get_logger().info(
                f"Linear motor {selected_command} background complete; "
                "PIN8/PIN9 LOW"
            )

        timer = threading.Timer(duration, stop_motor)
        # Keep the process alive long enough to publish the safety LOW even if
        # the step session is closed immediately after starting the actuator.
        timer.daemon = False
        pending_timers = getattr(planner, "_linear_motor_stop_timers", None)
        if pending_timers is None:
            pending_timers = []
            setattr(planner, "_linear_motor_stop_timers", pending_timers)
        pending_timers.append(timer)
        timer.start()
        planner.get_logger().info(
            "Linear motor retract is running in background; continuing to "
            "the next robot stage without waiting"
        )
        return True

    try:
        time.sleep(duration)
    finally:
        _linear_motor_publish(stage, False, False)
    planner.get_logger().info(
        f"Linear motor {selected_command} complete; PIN8/PIN9 LOW"
    )
    return True


def _execute_servo_sequence(planner, stage: dict) -> bool:
    publisher = stage.get("servo_publisher")
    topic = str(stage.get("servo_topic", SERVO_COMMAND_TOPIC))
    if publisher is None:
        planner.get_logger().error("Servo command publisher is unavailable")
        return False

    subscriber_deadline = time.monotonic() + 1.0
    while (
        planner.count_subscribers(topic) < 1
        and time.monotonic() < subscriber_deadline
    ):
        time.sleep(0.05)
    if planner.count_subscribers(topic) < 1:
        planner.get_logger().error(
            f"Servo command subscriber is unavailable: {topic}"
        )
        return False

    speed_percent = float(stage["speed_percent"])
    dwell_seconds = max(0.0, float(stage["dwell_seconds"]))
    for angle_deg, action in (
        (float(stage["close_angle_deg"]), "close"),
        (float(stage["open_angle_deg"]), "open"),
    ):
        message = Float64MultiArray()
        message.data = [angle_deg, speed_percent]
        publisher.publish(message)
        planner.get_logger().info(
            f"Servo {action} command: angle={angle_deg:.0f} deg, "
            f"speed={speed_percent:g}%, dwell={dwell_seconds:.1f}s"
        )
        time.sleep(dwell_seconds)

    retract_duration = max(
        0.0,
        float(stage.get("retract_duration_seconds", 0.0)),
    )
    if retract_duration > 0.0 and not _execute_linear_motor_action(
        planner,
        stage,
        command="retract",
        duration_seconds=retract_duration,
        wait_for_completion=False,
    ):
        return False
    return True


def _wait_for_background_linear_motor_stops(planner) -> None:
    """Keep the ROS node alive until every scheduled safety LOW is sent."""
    for timer in tuple(getattr(planner, "_linear_motor_stop_timers", ())):
        timer.join()


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
        speed_percent=float(stage.get("speed_percent", 100.0)),
    )
    started = time.monotonic()
    failure_reason = "TRAJECTORY_EXECUTION_FAILED"
    if stage["kind"] == "wait":
        time.sleep(float(stage["wait_seconds"]))
        success = True
    elif stage["kind"] == "linear_motor":
        # Reverse trajectory traversal does not replay actuator operations.
        success = True if reverse else _execute_linear_motor_action(planner, stage)
        failure_reason = "LINEAR_MOTOR_COMMAND_FAILED"
    elif stage["kind"] == "skip":
        success = True
    elif stage["kind"] == "servo_sequence":
        # The sequence finishes open, so reverse traversal has no servo-side
        # state to restore and intentionally performs no command.
        success = True if reverse else _execute_servo_sequence(planner, stage)
        failure_reason = "SERVO_COMMAND_FAILED"
    else:
        trajectories = stage["trajectories"]
        trajectory_speed_percents = stage.get(
            "trajectory_speed_percents"
        )
        if reverse:
            trajectories = reverse_trajectory_group(trajectories)
            if trajectory_speed_percents is not None:
                trajectory_speed_percents = tuple(
                    reversed(tuple(trajectory_speed_percents))
                )
        if trajectory_speed_percents is None:
            speed_percent = float(stage.get("speed_percent", 100.0))
            trajectories = scale_trajectory_group_speed(
                trajectories,
                speed_percent,
            )
            speed_label = f"속도 {speed_percent:g}%"
        else:
            trajectories = scale_trajectory_group_speeds(
                trajectories,
                trajectory_speed_percents,
            )
            speed_label = "구간속도 " + "/".join(
                f"{float(value):g}%"
                for value in trajectory_speed_percents
            )
        success = planner._execute_trajectory_group(
            trajectories,
            f"{execution_label} ({speed_label})",
        )
    duration = time.monotonic() - started
    if not success:
        _emit(
            "stage_failed",
            index=index,
            key=stage["key"],
            direction=direction,
            reason=failure_reason,
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
    planner.declare_parameter(
        "step_servo_command_topic",
        SERVO_COMMAND_TOPIC,
    )
    planner.declare_parameter("step_servo_speed_percent", 50.0)
    planner.declare_parameter(
        "step_preapproach_final_speed_percent",
        30.0,
    )
    planner.declare_parameter(
        "step_servo_close_angle_deg",
        SERVO_CLOSE_ANGLE_DEG,
    )
    planner.declare_parameter(
        "step_linear_motor_extend_seconds",
        LINEAR_MOTOR_RUN_SECONDS,
    )
    planner.declare_parameter(
        "step_linear_motor_pin8_topic",
        LINEAR_MOTOR_PIN8_TOPIC,
    )
    planner.declare_parameter(
        "step_linear_motor_pin9_topic",
        LINEAR_MOTOR_PIN9_TOPIC,
    )
    exit_code = 1
    try:
        plan = _plan_with_retries(planner, context="initial_step_plan")
        report = planner.last_plan_report
        if plan is None:
            _emit("plan_failed", report=report)
            raise SystemExit(1)

        stages = _build_step_stages(planner, plan)
        servo_topic = str(
            planner.get_parameter("step_servo_command_topic").value
        )
        servo_publisher = planner.create_publisher(
            Float64MultiArray,
            servo_topic,
            10,
        )
        pin8_topic = str(
            planner.get_parameter("step_linear_motor_pin8_topic").value
        )
        pin9_topic = str(
            planner.get_parameter("step_linear_motor_pin9_topic").value
        )
        pin8_publisher = planner.create_publisher(Bool, pin8_topic, 10)
        pin9_publisher = planner.create_publisher(Bool, pin9_topic, 10)
        _bind_stage_publishers(
            stages,
            servo_topic,
            servo_publisher,
            pin8_topic,
            pin9_topic,
            pin8_publisher,
            pin9_publisher,
        )
        expected = {
            str(name): float(value)
            for name, value in report.get("start_joint_positions", {}).items()
        }
        _set_expected_states(stages, 0, expected)
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
            if action == "replan_after_refinement":
                if next_index != 2:
                    _emit(
                        "replan_failed",
                        reason="ROBOT_NOT_AT_REINSPECTION_A",
                        next_index=next_index,
                    )
                    continue
                _emit(
                    "refinement_replanning",
                    next_index=next_index,
                    offset_xyz=command.get("offset_xyz", ()),
                )
                try:
                    stages, report = _replan_after_refinement(
                        planner,
                        stages,
                        command.get("offset_xyz", ()),
                        (
                            servo_topic,
                            servo_publisher,
                            pin8_topic,
                            pin9_topic,
                            pin8_publisher,
                            pin9_publisher,
                        ),
                    )
                except Exception as error:
                    _emit(
                        "replan_failed",
                        reason=str(error),
                        report=planner.last_plan_report,
                        next_index=next_index,
                    )
                    continue
                _emit(
                    "refinement_replanned",
                    stages=_stage_metadata(stages),
                    report=report,
                    next_index=next_index,
                )
                resume_command = command.get("resume_command")
                if not isinstance(resume_command, dict):
                    _emit("paused", next_index=next_index, direction="forward")
                    continue
                command = resume_command
                action = str(command.get("command", ""))
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
            merge_cartesian = bool(command.get("merge_cartesian", False))
            merge_all_trajectories = bool(
                command.get("merge_all_trajectories", False)
            )
            if merge_cartesian and merge_all_trajectories:
                _emit(
                    "command_error",
                    message="두 경로 합치기 옵션을 동시에 사용할 수 없습니다.",
                )
                continue
            if action == "execute_through" and merge_all_trajectories:
                continuous_blocks = _prepare_all_trajectory_blocks(
                    planner,
                    stages,
                    next_index,
                    target_index,
                )
                if continuous_blocks is None:
                    continue
            elif action == "execute_through" and merge_cartesian:
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
        _wait_for_background_linear_motor_stops(planner)
        planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
