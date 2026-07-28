import csv
import json
import math
import os
import queue
import random
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from collections import Counter, deque
from datetime import datetime
from pathlib import Path
from tkinter import font as tkfont
from tkinter import messagebox, ttk

import rclpy
from action_msgs.srv import CancelGoal
from farmily_tomato_interfaces.msg import TomatoDetectionArray
from farmily_tomato_interfaces.srv import DetectTomatoes
from geometry_msgs.msg import Point
from moveit_msgs.msg import RobotState
from rbpodo_msgs.srv import SetSpeedBar, TaskStop
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


PLANNER_CONFIGS = {
    "Cartesian": ("ompl", "RRTConnect", "cartesian"),
    "OMPL / RRTConnect": ("ompl", "RRTConnect", "planner"),
    "CHOMP": ("chomp", "RRTConnect", "planner"),
    "PILZ / LIN": ("pilz_industrial_motion_planner", "LIN", "planner"),
}
HARVEST_RESULT_NAMESPACE = "harvest_plan_result"
HARVEST_SWEEP_NAMESPACE = "harvest_sweep_result"
SWEEP_RESULT_PREFIX = "__HARVEST_RESULT__"
PLAN_RESULT_PREFIX = "__HARVEST_PLAN_RESULT__"
SWEEP_CSV_FIELDS = (
    "timestamp",
    "case",
    "tomato",
    "x",
    "y",
    "z",
    "rotation_deg",
    "success",
    "pipeline",
    "planner_id",
    "preapproach_mode",
    "execute_motion",
    "execution_attempted",
    "execution_success",
    "execution_duration_sec",
    "failure_stage",
    "failure_planner_type",
    "failure_reason",
    "recovery_used",
    "recovery_success",
    "recovery_stage",
    "recovery_reason",
    "moveit_error_code",
    "cartesian_fraction",
    "required_fraction",
    "cartesian_fallbacks",
    "adaptive_grasp_rotation_deg",
    "adaptive_grasp_current_error_deg",
    "adaptive_grasp_selected_error_deg",
    "duration_sec",
)


def cartesian_fallback_summary(report) -> dict:
    """Summarize successful constrained-OMPL recovery of Cartesian failures."""
    fallbacks = [
        item
        for item in report.get("cartesian_fallbacks", [])
        if item.get("success")
    ]
    if not fallbacks:
        return {}

    stages = [
        str(item.get("cartesian_stage") or item.get("segment") or "CARTESIAN")
        for item in fallbacks
    ]
    reasons = [
        (
            f"{stage}: "
            f"{item.get('cartesian_reason') or 'CARTESIAN_FAILED'}"
            " → CONSTRAINED_OMPL_SUCCESS"
        )
        for stage, item in zip(stages, fallbacks)
    ]
    ompl_stages = [
        str(stage)
        for item in fallbacks
        for stage in item.get("ompl_stages", [])
    ]
    return {
        "failure_stage": " / ".join(stages),
        "failure_planner_type": "cartesian",
        "failure_reason": "; ".join(reasons),
        "recovery_used": True,
        "recovery_success": True,
        "recovery_stage": " / ".join(ompl_stages),
        "recovery_reason": "Cartesian 실패 후 constrained OMPL 성공",
    }


def adaptive_rotation_was_applied(report, epsilon_deg: float = 1e-6) -> bool:
    """Return whether adaptive grasp geometry rotated away from tomato -X."""
    adaptive_grasp = report.get("adaptive_grasp", {})
    return abs(float(adaptive_grasp.get("applied_rotation_deg", 0.0))) > abs(
        float(epsilon_deg)
    )


def adaptive_rotation_degrees(report) -> float:
    """Return signed pre-grasp rotation toward tomato local ±Y."""
    adaptive_grasp = report.get("adaptive_grasp", {})
    return max(
        -90.0,
        min(90.0, float(adaptive_grasp.get("applied_rotation_deg", 0.0))),
    )


def adaptive_approach_axis_local(report):
    """Return the exact planned approach axis in tomato-local coordinates."""
    adaptive_grasp = report.get("adaptive_grasp", {})
    values = adaptive_grasp.get("approach_axis_tomato_local", [])
    if len(values) < 2:
        return None
    x = float(values[0])
    y = float(values[1])
    length = math.hypot(x, y)
    if length <= 1e-9:
        return None
    return (x / length, y / length)


def concise_plan_report(report) -> str:
    """Format one compact, readable GUI log block from a planner report."""
    success = bool(report.get("success"))
    result = "성공" if success else "실패"
    target = str(report.get("tomato_frame", "UNKNOWN"))
    adaptive = report.get("adaptive_grasp", {})
    lines = [
        f"Plan {result}: {target} | "
        f"보정={float(adaptive.get('applied_rotation_deg', 0.0)):.1f}°"
    ]
    for item in report.get("stages", []):
        stage = str(item.get("stage", "UNKNOWN"))
        planner = str(item.get("planner_type", "unknown"))
        if item.get("success") and item.get("discarded"):
            status = "성공·폐기"
        else:
            status = "성공" if item.get("success") else "실패"
        details = []
        duration = item.get("duration_sec")
        if duration is not None:
            details.append(f"소요={float(duration):.2f}s")
        if "cartesian_fraction" in item:
            details.append(
                f"fraction={100.0 * float(item['cartesian_fraction']):.1f}%"
            )
        reason = str(item.get("reason", ""))
        if reason:
            details.append(reason)
        suffix = f" | {', '.join(details)}" if details else ""
        lines.append(f"  [{status}] {stage} ({planner}){suffix}")

    if not success:
        lines.append(
            "  최종 실패: "
            f"{report.get('failure_stage', 'UNKNOWN')} / "
            f"{report.get('failure_reason', 'UNKNOWN')}"
        )

    joint_ranges = report.get("joint_ranges", [])
    if joint_ranges:
        lines.append("  관절범위°: joint 시작 / 최소 / 최대 / span")
        for item in joint_ranges:
            lines.append(
                f"    {item.get('joint_name', '?')}: "
                f"{float(item.get('start_deg', 0.0)):.1f} / "
                f"{float(item.get('min_deg', 0.0)):.1f} / "
                f"{float(item.get('max_deg', 0.0)):.1f} / "
                f"{float(item.get('span_deg', 0.0)):.1f}"
            )
    return "\n".join(lines)


def sweep_stage_summary(record) -> str:
    """Return a compact one-line summary suitable for a Treeview cell."""
    failure_stage = str(record.get("failure_stage") or "")
    recovery_stage = str(record.get("recovery_stage") or "")
    if failure_stage and record.get("recovery_success"):
        return f"{failure_stage} → {recovery_stage or '복구 성공'}"
    if failure_stage:
        return failure_stage
    return "전체 단계 성공" if record.get("success") else "실패 단계 정보 없음"


def sweep_execution_duration_text(record) -> str:
    """Format full robot sequence time, or mark a plan-only result."""
    if not record.get("execution_attempted"):
        return "-"
    try:
        duration = float(record.get("execution_duration_sec"))
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(duration) or duration < 0.0:
        return "-"
    return f"{duration:.2f}"


def sweep_stage_detail(record) -> str:
    """Format every planning stage as a readable multiline result."""
    lines = []
    for index, item in enumerate(record.get("stages", []), start=1):
        stage = str(item.get("stage") or "UNKNOWN")
        planner = str(item.get("planner_type") or "unknown")
        if item.get("success") and item.get("discarded"):
            status = "성공·폐기"
        else:
            status = "성공" if item.get("success") else "실패"
        metrics = []
        if item.get("duration_sec") is not None:
            metrics.append(f"소요={float(item['duration_sec']):.2f}s")
        if item.get("cartesian_fraction") is not None:
            metrics.append(
                f"fraction={100.0 * float(item['cartesian_fraction']):.1f}%"
            )
        if item.get("required_fraction") is not None:
            metrics.append(
                f"요구={100.0 * float(item['required_fraction']):.1f}%"
            )
        if item.get("reason"):
            metrics.append(f"사유={item['reason']}")
        suffix = f" | {' | '.join(metrics)}" if metrics else ""
        lines.append(f"{index}. [{status}] {stage} ({planner}){suffix}")

    if record.get("recovery_success"):
        lines.append(
            "복구 결과: 성공"
            f" | {record.get('recovery_stage') or '대체 경로'}"
            f" | {record.get('recovery_reason') or '사유 정보 없음'}"
        )
    if record.get("success"):
        lines.append("최종 결과: 성공")
    else:
        lines.append(
            "최종 결과: 실패"
            f" | {record.get('failure_stage') or 'UNKNOWN'}"
            f" | {record.get('failure_reason') or '사유 정보 없음'}"
        )
    return "\n".join(lines)


def is_critical_process_output(line: str) -> bool:
    """Keep only unstructured fatal output not represented in plan JSON."""
    text = str(line).strip()
    return (
        text.startswith("Traceback")
        or text.startswith("File ")
        or "Exception:" in text
        or "Error:" in text
        or "Segmentation fault" in text
    )


def _inclusive_step_values(
    begin: float,
    finish: float,
    increment: float,
    label: str,
) -> list[float]:
    """Return begin-to-finish values, including the exact finish value."""
    begin = float(begin)
    finish = float(finish)
    increment = abs(float(increment))
    distance = abs(finish - begin)

    if distance < 1e-12:
        return [begin]
    if increment <= 0.0:
        raise ValueError(f"{label} 변화량은 0보다 커야 합니다.")

    direction = 1.0 if finish >= begin else -1.0
    interval_count = int(math.ceil(distance / increment))
    values = [
        begin + direction * min(index * increment, distance)
        for index in range(interval_count + 1)
    ]
    values[-1] = finish
    return values


def generate_sweep_cases(
    start,
    end,
    step,
    randomized,
    rng=None,
    maximum_cases: int = 500,
) -> list[tuple[float, float, float, float]]:
    """Generate XYZ positions, then test every rotation at each position.

    X/Y/Z retain the previous behavior: deterministic axes advance together and
    random axes are sampled once per XYZ position. Rotation is never randomized;
    every value from its start to end is tested before moving to the next XYZ
    position.
    """
    starts = [float(value) for value in start]
    ends = [float(value) for value in end]
    steps = [abs(float(value)) for value in step]
    random_flags = [bool(value) for value in randomized]
    if not all(len(values) == 4 for values in (starts, ends, steps, random_flags)):
        raise ValueError(
            "start, end, step and randomized must contain four values"
        )

    # X/Y/Z determine how many distinct positions are generated.
    position_interval_counts = []
    has_random_position_range = False
    for begin, finish, increment, use_random in zip(
        starts[:3], ends[:3], steps[:3], random_flags[:3]
    ):
        distance = abs(finish - begin)
        if distance < 1e-12:
            continue
        if use_random:
            has_random_position_range = True
            continue
        if increment <= 0.0:
            raise ValueError(
                "변경되는 X/Y/Z 항목의 변화량은 0보다 커야 합니다."
            )
        position_interval_counts.append(int(math.ceil(distance / increment)))

    if not position_interval_counts and has_random_position_range:
        raise ValueError(
            "XYZ 랜덤 범위를 반복하려면 종료 기준이 될 "
            "비랜덤 X/Y/Z 변화 축을 하나 이상 설정하세요."
        )

    position_intervals = (
        min(position_interval_counts) if position_interval_counts else 0
    )
    position_count = position_intervals + 1

    rotation_values = _inclusive_step_values(
        starts[3],
        ends[3],
        steps[3],
        "회전",
    )
    total_case_count = position_count * len(rotation_values)
    if total_case_count > maximum_cases:
        raise ValueError(
            f"자동 테스트는 최대 {maximum_cases}개 케이스까지 실행할 수 있습니다. "
            f"현재 설정: XYZ 위치 {position_count}개 × "
            f"회전 {len(rotation_values)}개 = {total_case_count}개"
        )

    random_source = rng or random.Random()
    cases = []
    for position_index in range(position_count):
        xyz = []
        for begin, finish, increment, use_random in zip(
            starts[:3], ends[:3], steps[:3], random_flags[:3]
        ):
            distance = abs(finish - begin)
            if use_random and distance >= 1e-12:
                xyz.append(
                    random_source.uniform(min(begin, finish), max(begin, finish))
                )
            else:
                direction = 1.0 if finish >= begin else -1.0
                moved = min(position_index * increment, distance)
                xyz.append(begin + direction * moved)

        # Keep this XYZ position fixed while every rotation is tested.
        for rotation in rotation_values:
            cases.append((xyz[0], xyz[1], xyz[2], rotation))

    return cases


def harvest_command(
    tomato_index: int,
    execute: bool,
    planning_pipeline_id: str = "ompl",
    planner_id: str = "RRTConnect",
    preapproach_mode: str = "cartesian",
    publish_display_trajectory: bool = True,
    velocity_scale: float = 0.20,
    acceleration_scale: float = 0.20,
    harvest_wait_sec: float = 2.0,
    python_executable: str | None = None,
) -> list[str]:
    """Build the isolated harvest planner command used by the GUI."""
    if tomato_index < 0:
        raise ValueError("tomato_index must be zero or greater")
    planner_config = (
        planning_pipeline_id,
        planner_id,
        preapproach_mode,
    )
    if planner_config not in PLANNER_CONFIGS.values():
        raise ValueError(
            f"unsupported planner: pipeline={planning_pipeline_id} "
            f"planner_id={planner_id}"
        )
    for name, value in (
        ("velocity_scale", velocity_scale),
        ("acceleration_scale", acceleration_scale),
    ):
        if not 0.0 < float(value) <= 1.0:
            raise ValueError(f"{name} must be greater than 0 and at most 1")
    harvest_wait_sec = float(harvest_wait_sec)
    if not math.isfinite(harvest_wait_sec) or harvest_wait_sec < 0.0:
        raise ValueError("harvest_wait_sec must be a finite value of zero or greater")
    executable = python_executable or sys.executable
    return [
        executable,
        "-m",
        "rbpodo_tomato_harvest.tomato_harvest_test",
        "--ros-args",
        "-p",
        f"tomato_frame:=detected_tomato_{tomato_index}_tf",
        "-p",
        f"execute:={'true' if execute else 'false'}",
        "-p",
        f"planning_pipeline_id:={planning_pipeline_id}",
        "-p",
        f"planner_id:={planner_id}",
        "-p",
        f"preapproach_mode:={preapproach_mode}",
        "-p",
        "publish_display_trajectory:="
        f"{'true' if publish_display_trajectory else 'false'}",
        "-p",
        f"pick_ready_velocity_scale:={float(velocity_scale)}",
        "-p",
        f"pick_ready_acceleration_scale:={float(acceleration_scale)}",
        "-p",
        f"harvest_wait_sec:={harvest_wait_sec}",
    ]


def harvest_all_jobs(tomato_count: int) -> list[tuple[int, bool]]:
    """Build plan-only/execute jobs for every detected tomato in order."""
    if tomato_count < 0:
        raise ValueError("tomato_count must be zero or greater")
    return [
        (index, execute)
        for index in range(tomato_count)
        for execute in (False, True)
    ]


def tomato_stem_arrow_length(
    center,
    stem_point,
    source_to_parent_quaternion=None,
    stem_margin: float = 0.008,
    minimum_length: float = 0.015,
) -> float:
    """Return the horizontal tomato-center to just-before-stem distance."""
    x = float(stem_point.x) - float(center.x)
    y = float(stem_point.y) - float(center.y)
    z = float(stem_point.z) - float(center.z)
    if source_to_parent_quaternion is not None:
        qx, qy, qz, qw = [float(value) for value in source_to_parent_quaternion]
        rotated_x = (
            (1 - 2 * (qy * qy + qz * qz)) * x
            + 2 * (qx * qy - qz * qw) * y
            + 2 * (qx * qz + qy * qw) * z
        )
        rotated_y = (
            2 * (qx * qy + qz * qw) * x
            + (1 - 2 * (qx * qx + qz * qz)) * y
            + 2 * (qy * qz - qx * qw) * z
        )
        horizontal_distance = math.hypot(rotated_x, rotated_y)
    else:
        horizontal_distance = math.sqrt(x * x + y * y + z * z)
    return max(float(minimum_length), horizontal_distance - float(stem_margin))


def harvest_result_marker(
    tomato_index: int,
    success: bool,
    arrow_length: float,
    adaptive_rotation_applied: bool = False,
    adaptive_rotation_deg: float = 0.0,
    approach_axis_local=None,
) -> Marker:
    """Create a result arrow at a detected tomato TF.

    Failure arrows retain the established tomato +X approach reference.
    Because this arrow is opposite the tomato-to-pre-grasp position vector,
    a pre-grasp rotation toward +Y appears as a +X-to--Y arrow rotation.
    """
    if tomato_index < 0:
        raise ValueError("tomato_index must be zero or greater")
    if arrow_length <= 0.0:
        raise ValueError("arrow_length must be greater than zero")
    marker = Marker()
    marker.header.frame_id = f"detected_tomato_{tomato_index}_tf"
    marker.ns = HARVEST_RESULT_NAMESPACE
    marker.id = tomato_index
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    if approach_axis_local is not None:
        axis_x, axis_y = approach_axis_local
        endpoint = Point(
            x=float(arrow_length) * float(axis_x),
            y=float(arrow_length) * float(axis_y),
        )
    elif success:
        endpoint = Point(x=float(arrow_length))
    else:
        angle = math.radians(
            max(-90.0, min(90.0, float(adaptive_rotation_deg)))
        )
        endpoint = Point(
            x=float(arrow_length) * math.cos(angle),
            y=-float(arrow_length) * math.sin(angle),
        )
    marker.points = [Point(), endpoint]
    marker.scale.x = 0.008
    marker.scale.y = 0.016
    marker.scale.z = 0.020
    if not success:
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif adaptive_rotation_applied:
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 1.0
    else:
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    marker.color.a = 1.0
    return marker


def sweep_result_marker(
    case_id: int,
    success: bool,
    arrow_length: float,
    parent_frame: str,
    transform,
    adaptive_rotation_applied: bool = False,
    adaptive_rotation_deg: float = 0.0,
    approach_axis_local=None,
) -> Marker:
    """Create a result arrow frozen in the robot-base coordinate frame."""
    marker = harvest_result_marker(
        0,
        success,
        arrow_length,
        adaptive_rotation_applied=adaptive_rotation_applied,
        adaptive_rotation_deg=adaptive_rotation_deg,
        approach_axis_local=approach_axis_local,
    )
    marker.header.frame_id = parent_frame
    marker.ns = HARVEST_SWEEP_NAMESPACE
    marker.id = case_id
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    marker.pose.position.x = translation.x
    marker.pose.position.y = translation.y
    marker.pose.position.z = translation.z
    marker.pose.orientation.x = rotation.x
    marker.pose.orientation.y = rotation.y
    marker.pose.orientation.z = rotation.z
    marker.pose.orientation.w = rotation.w
    return marker


def scene_parameters(position, rotation_deg: float) -> list[Parameter]:
    """Build scene position and main-vine-axis rotation parameters."""
    coordinates = [float(value) for value in position]
    if len(coordinates) != 3:
        raise ValueError("position must contain exactly three values")
    position_value = ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
        double_array_value=coordinates,
    )
    rotation_value = ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE,
        double_value=float(rotation_deg),
    )
    return [
        Parameter(name="object_position", value=position_value),
        Parameter(name="tomato_z_spin_deg", value=rotation_value),
    ]


def cancel_all_goals_request() -> CancelGoal.Request:
    """Build the ROS action request whose zero ID/time cancels every goal."""
    request = CancelGoal.Request()
    request.goal_info.goal_id.uuid = [0] * 16
    request.goal_info.stamp.sec = 0
    request.goal_info.stamp.nanosec = 0
    return request


class HarvestGui(Node):
    """Tkinter operator panel for tomato detection and harvest testing."""

    def __init__(self) -> None:
        super().__init__("tomato_harvest_gui")
        self.declare_parameter(
            "camera_service", "/fake_tomato_camera/detect_tomatoes"
        )
        self.declare_parameter(
            "detections_topic", "/tomato_detection/detections"
        )
        self.declare_parameter("scene_node", "/tomato_scene_node")
        self.declare_parameter(
            "hardware_speed_service", "/rbpodo_hardware/set_speed_bar"
        )
        self.declare_parameter(
            "hardware_stop_service", "/rbpodo_hardware/task_stop"
        )
        self.declare_parameter(
            "moveit_cancel_service", "/execute_trajectory/_action/cancel_goal"
        )
        self.declare_parameter(
            "controller_cancel_service",
            "/joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal",
        )
        self.declare_parameter("default_planner", "cartesian")
        self.declare_parameter(
            "result_markers_topic", "/harvest_result_markers"
        )
        self.declare_parameter("result_marker_parent_frame", "link0")
        self.declare_parameter("result_arrow_stem_margin", 0.008)
        self.declare_parameter("result_arrow_minimum_length", 0.015)
        self.declare_parameter(
            "results_directory",
            str(Path.home() / "farmily_tomato" / "harvest_results"),
        )

        camera_service = str(self.get_parameter("camera_service").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        scene_node = str(self.get_parameter("scene_node").value).rstrip("/")
        self.camera_client = self.create_client(DetectTomatoes, camera_service)
        self.scene_get_client = self.create_client(
            GetParameters, f"{scene_node}/get_parameters"
        )
        self.scene_set_client = self.create_client(
            SetParameters, f"{scene_node}/set_parameters"
        )
        self.hardware_speed_service = str(
            self.get_parameter("hardware_speed_service").value
        )
        self.hardware_speed_client = self.create_client(
            SetSpeedBar, self.hardware_speed_service
        )
        self.hardware_stop_service = str(
            self.get_parameter("hardware_stop_service").value
        )
        self.hardware_stop_client = self.create_client(
            TaskStop, self.hardware_stop_service
        )
        self.moveit_cancel_service = str(
            self.get_parameter("moveit_cancel_service").value
        )
        self.moveit_cancel_client = self.create_client(
            CancelGoal, self.moveit_cancel_service
        )
        self.controller_cancel_service = str(
            self.get_parameter("controller_cancel_service").value
        )
        self.controller_cancel_client = self.create_client(
            CancelGoal, self.controller_cancel_service
        )
        self.create_subscription(
            TomatoDetectionArray,
            detections_topic,
            self._detections_callback,
            10,
        )
        result_marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.result_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("result_markers_topic").value),
            result_marker_qos,
        )
        self.rviz_goal_state_publisher = self.create_publisher(
            RobotState,
            "/rviz/moveit/update_custom_goal_state",
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_service = camera_service
        self.scene_node = scene_node
        self.detected_tomatoes = []
        self.detection_signature = None
        self.detection_generation = 0
        self.verified_plan = None
        self.harvest_process = None
        self.harvest_plan_report = {}
        self.last_failure_robot_state = None
        self.process_queue = queue.Queue()
        self.batch_active = False
        self.batch_jobs = deque()
        self.batch_generation = None
        self.batch_total = 0
        self.batch_completed = 0
        self.batch_skipped = 0
        self.batch_planner = None
        self.batch_harvest_wait_sec = 2.0
        self.harvest_results: dict[int, bool] = {}
        self.harvest_result_adaptive_rotation: dict[int, bool] = {}
        self.harvest_result_adaptive_rotation_deg: dict[int, float] = {}
        self.harvest_result_approach_axis_local: dict[int, tuple] = {}
        self.result_arrow_lengths: dict[int, float] = {}
        self.result_detection_frame = ""
        self.sweep_markers: list[Marker] = []
        self.sweep_marker_next_id = 0
        self.sweep_worker_process = None
        self.sweep_request_id = 0
        self.sweep_pending_verification = None
        self.sweep_session_dir = None
        self.sweep_started_monotonic = 0.0
        self.sweep_success_count = 0
        self.sweep_recovery_count = 0
        self.sweep_failure_counts = Counter()
        self.sweep_result_records = {}
        self.preserve_sweep_markers_on_scene_set = False
        self.sweep_active = False
        self.sweep_cancel_requested = False
        self.sweep_execute_motion = False
        self.sweep_harvest_wait_sec = 2.0
        self.sweep_cases = deque()
        self.sweep_case_total = 0
        self.sweep_case_number = 0
        self.sweep_total = 0
        self.sweep_completed = 0
        self.sweep_case_tomato_total = 0
        self.sweep_case_tomato_completed = 0
        self.sweep_tomato_queue = deque()
        self.sweep_current_case = None
        self.closing = False

        self.root = tk.Tk()
        self.root.title("Farmily Tomato Harvest")
        self.root.maxsize(1680, 900)
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        initial_width = min(1600, max(1180, screen_width - 40))
        initial_height = min(840, max(680, screen_height - 60))
        self.root.geometry(f"{initial_width}x{initial_height}")
        self.root.minsize(
            min(1280, initial_width),
            min(720, initial_height),
        )
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        signal.signal(signal.SIGINT, self._signal_close)
        signal.signal(signal.SIGTERM, self._signal_close)
        self._configure_korean_font()

        self.selected_tomato = tk.StringVar(value="")
        default_planner = str(self.get_parameter("default_planner").value)
        planner_label = next(
            (
                label
                for label, config in PLANNER_CONFIGS.items()
                if (
                    (default_planner == "cartesian" and config[2] == "cartesian")
                    or (
                        config[0] == default_planner
                        and config[2] == "planner"
                    )
                )
            ),
            "OMPL / RRTConnect",
        )
        self.selected_planner = tk.StringVar(value=planner_label)
        self.scene_x = tk.StringVar(value="0.355")
        self.scene_y = tk.StringVar(value="-0.375")
        self.scene_z = tk.StringVar(value="0.340")
        self.scene_rotation = tk.StringVar(value="45.0")
        self.speed_bar_percent = tk.StringVar(value="10")
        self.motion_velocity_percent = tk.StringVar(value="20")
        self.motion_acceleration_percent = tk.StringVar(value="20")
        self.linear_motor_wait_sec = tk.StringVar(value="2.0")
        self.motion_velocity_scale = 0.20
        self.motion_acceleration_scale = 0.20
        self.status = tk.StringVar(value="MoveIt과 카메라 서비스를 확인해 주세요.")
        self._build_ui()
        self.root.after(50, self._spin_ros)
        self.root.after(50, self._drain_process_queue)
        self.root.after(400, self.read_scene_position)

    def _configure_korean_font(self) -> None:
        available = set(tkfont.families(self.root))
        preferred = (
            "Noto Sans CJK KR",
            "NanumGothic",
            "Droid Sans Fallback",
        )
        family = next((name for name in preferred if name in available), None)
        if family is None:
            return
        for name in (
            "TkDefaultFont",
            "TkTextFont",
            "TkFixedFont",
            "TkMenuFont",
            "TkHeadingFont",
            "TkCaptionFont",
            "TkSmallCaptionFont",
            "TkIconFont",
            "TkTooltipFont",
        ):
            try:
                tkfont.nametofont(name, self.root).configure(family=family)
            except tk.TclError:
                pass

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=8)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=0)
        outer.rowconfigure(1, weight=1)
        outer.rowconfigure(3, weight=1)

        top_controls = ttk.Frame(outer)
        top_controls.grid(row=0, column=0, sticky="ew")
        top_controls.columnconfigure(0, weight=1)

        camera_frame = ttk.LabelFrame(
            top_controls, text="1. 카메라 검출", padding=8
        )
        camera_frame.grid(row=0, column=0, sticky="ew")
        camera_frame.columnconfigure(1, weight=1)
        ttk.Label(camera_frame, text="서비스").grid(row=0, column=0, sticky="w")
        ttk.Label(camera_frame, text=self.camera_service).grid(
            row=0, column=1, sticky="w", padx=8
        )
        self.detect_button = ttk.Button(
            camera_frame,
            text="토마토 촬영 / 검출",
            command=self.detect_tomatoes,
        )
        self.detect_button.grid(row=0, column=2, padx=(8, 0))

        list_frame = ttk.LabelFrame(outer, text="2. 검출된 토마토 선택", padding=8)
        list_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        list_frame.columnconfigure(0, weight=1)
        list_frame.rowconfigure(0, weight=1)
        columns = ("index", "frame", "source_id", "x", "y", "z")
        self.tomato_tree = ttk.Treeview(
            list_frame,
            columns=columns,
            show="headings",
            height=6,
            selectmode="browse",
        )
        headings = {
            "index": "번호",
            "frame": "수확 TF",
            "source_id": "카메라 ID",
            "x": "Camera X (m)",
            "y": "Camera Y (m)",
            "z": "Camera Z (m)",
        }
        widths = {
            "index": 52,
            "frame": 180,
            "source_id": 110,
            "x": 100,
            "y": 100,
            "z": 100,
        }
        for column in columns:
            self.tomato_tree.heading(column, text=headings[column])
            self.tomato_tree.column(
                column,
                width=widths[column],
                anchor="center",
                stretch=column in ("frame", "source_id"),
            )
        scrollbar = ttk.Scrollbar(
            list_frame, orient="vertical", command=self.tomato_tree.yview
        )
        self.tomato_tree.configure(yscrollcommand=scrollbar.set)
        self.tomato_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.tomato_tree.bind("<<TreeviewSelect>>", self._tree_selection_changed)

        motion_frame = ttk.LabelFrame(outer, text="3. 수확 모션", padding=8)
        motion_frame.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        motion_frame.columnconfigure(1, weight=1)
        ttk.Label(motion_frame, text="선택 토마토").grid(
            row=0, column=0, sticky="w"
        )
        self.tomato_combo = ttk.Combobox(
            motion_frame,
            textvariable=self.selected_tomato,
            state="readonly",
            width=28,
        )
        self.tomato_combo.grid(row=0, column=1, sticky="w", padx=8)
        self.tomato_combo.bind("<<ComboboxSelected>>", self._combo_selection_changed)
        self.plan_button = ttk.Button(
            motion_frame,
            text="Plan-only 확인",
            command=lambda: self.start_harvest(False),
            state="disabled",
        )
        self.plan_button.grid(row=0, column=2, padx=(8, 4))
        self.execute_button = ttk.Button(
            motion_frame,
            text="실제 수확 실행",
            command=lambda: self.start_harvest(True),
            state="disabled",
        )
        self.execute_button.grid(row=0, column=3, padx=(4, 0))
        ttk.Label(motion_frame, text="플래너").grid(
            row=1, column=0, sticky="w", pady=(8, 0)
        )
        self.planner_combo = ttk.Combobox(
            motion_frame,
            textvariable=self.selected_planner,
            values=list(PLANNER_CONFIGS),
            state="readonly",
            width=28,
        )
        self.planner_combo.grid(
            row=1, column=1, sticky="w", padx=8, pady=(8, 0)
        )
        self.planner_combo.bind(
            "<<ComboboxSelected>>", self._planner_selection_changed
        )
        self.harvest_all_button = ttk.Button(
            motion_frame,
            text="검출 토마토 전체 연속 수확",
            command=self.start_harvest_all,
            state="disabled",
        )
        self.harvest_all_button.grid(
            row=1,
            column=2,
            columnspan=2,
            sticky="ew",
            padx=(8, 0),
            pady=(8, 0),
        )
        self.clear_markers_button = ttk.Button(
            motion_frame,
            text="결과 마커 지우기",
            command=self.clear_harvest_result_markers,
            state="disabled",
        )
        self.clear_markers_button.grid(
            row=2,
            column=2,
            columnspan=2,
            sticky="ew",
            padx=(8, 0),
            pady=(8, 0),
        )
        self.failure_goal_button = ttk.Button(
            motion_frame,
            text="실패 자세 → RViz Goal",
            command=self.show_failure_goal_state,
            state="disabled",
        )
        self.failure_goal_button.grid(
            row=2,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(8, 0),
        )
        ttk.Label(motion_frame, text="리니어모터 대기시간").grid(
            row=3, column=0, sticky="w", pady=(8, 0)
        )
        wait_input = ttk.Frame(motion_frame)
        wait_input.grid(row=3, column=1, sticky="w", padx=8, pady=(8, 0))
        self.linear_motor_wait_entry = ttk.Entry(
            wait_input,
            textvariable=self.linear_motor_wait_sec,
            width=8,
        )
        self.linear_motor_wait_entry.grid(row=0, column=0)
        ttk.Label(wait_input, text="초").grid(row=0, column=1, padx=(4, 0))
        ttk.Label(
            motion_frame,
            text=(
                "개별 실제 실행은 Plan-only 성공 후 활성화됩니다. "
                "전체 수확은 토마토마다 Plan-only 후 실제 실행합니다."
            ),
        ).grid(row=4, column=0, columnspan=4, sticky="w", pady=(8, 0))

        scene_frame = ttk.LabelFrame(
            top_controls, text="토마토 줄기 위치 / 회전", padding=8
        )
        scene_frame.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        for column in range(10):
            scene_frame.columnconfigure(column, weight=0)
        for column, (label, variable) in enumerate(
            (("X", self.scene_x), ("Y", self.scene_y), ("Z", self.scene_z))
        ):
            base = column * 2
            ttk.Label(scene_frame, text=label).grid(row=0, column=base, padx=(0, 4))
            ttk.Entry(scene_frame, textvariable=variable, width=8).grid(
                row=0, column=base + 1, padx=(0, 6)
            )
        ttk.Label(scene_frame, text="회전(°)").grid(
            row=0, column=6, padx=(0, 4)
        )
        ttk.Spinbox(
            scene_frame,
            from_=-360.0,
            to=360.0,
            increment=5.0,
            textvariable=self.scene_rotation,
            width=8,
        ).grid(row=0, column=7, padx=(0, 6))
        self.read_scene_button = ttk.Button(
            scene_frame,
            text="현재값 읽기",
            command=self.read_scene_position,
        )
        self.read_scene_button.grid(row=0, column=8, padx=(4, 4))
        self.set_scene_button = ttk.Button(
            scene_frame,
            text="위치/회전 적용",
            command=self.set_scene_position,
        )
        self.set_scene_button.grid(row=0, column=9, padx=(4, 0))
        ttk.Label(
            scene_frame,
            text="회전은 메인 줄기 축을 기준으로 가지와 토마토 전체에 적용됩니다.",
        ).grid(row=1, column=0, columnspan=10, sticky="w", pady=(8, 0))

        speed_frame = ttk.LabelFrame(
            top_controls, text="로봇 이동 속도", padding=8
        )
        speed_frame.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        speed_items = (
            ("RB Speed Bar (%)", self.speed_bar_percent),
            ("OMPL/Joint 속도 (%)", self.motion_velocity_percent),
            ("OMPL/Joint 가속도 (%)", self.motion_acceleration_percent),
        )
        for index, (label, variable) in enumerate(speed_items):
            base = index * 2
            ttk.Label(speed_frame, text=label).grid(
                row=0, column=base, padx=(0 if index == 0 else 10, 4)
            )
            ttk.Spinbox(
                speed_frame,
                from_=1,
                to=100,
                increment=5,
                textvariable=variable,
                width=6,
            ).grid(row=0, column=base + 1)
        self.apply_speed_button = ttk.Button(
            speed_frame,
            text="속도 적용",
            command=self.apply_motion_speed,
        )
        self.apply_speed_button.grid(row=0, column=6, padx=(12, 0))
        ttk.Label(
            speed_frame,
            text=(
                "실제 로봇: Speed Bar + OMPL/Joint 적용  |  "
                "시뮬레이션: OMPL/Joint만 적용 (Cartesian 제외)"
            ),
        ).grid(row=1, column=0, columnspan=7, sticky="w", pady=(6, 0))

        log_frame = ttk.LabelFrame(outer, text="상태 및 실행 로그", padding=8)
        log_frame.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, height=6, wrap="word", state="disabled")
        log_scrollbar = ttk.Scrollbar(
            log_frame, orient="vertical", command=self.log_text.yview
        )
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_scrollbar.grid(row=0, column=1, sticky="ns")
        ttk.Label(outer, textvariable=self.status, anchor="w").grid(
            row=4, column=0, sticky="ew", pady=(6, 0)
        )

        sweep_frame = ttk.LabelFrame(
            outer, text="4. 줄기 위치/회전 Plan 자동 테스트", padding=8
        )
        sweep_frame.grid(
            row=0, column=1, rowspan=5, sticky="nsew", padx=(8, 0)
        )
        self._build_sweep_ui(sweep_frame)

    def _build_sweep_ui(self, frame) -> None:
        self.sweep_inputs = {}
        defaults = {
            "start": ("0.55", "-0.4", "0.2", "0"),
            "end": ("0.55", "0.4", "0.8", "180"),
            "step": ("0.010", "0.1", "0.02", "30"),
        }
        keys = ("x", "y", "z", "rotation")
        for prefix, values in defaults.items():
            for key, value in zip(keys, values):
                self.sweep_inputs[f"{prefix}_{key}"] = tk.StringVar(value=value)
        for key in keys:
            self.sweep_inputs[f"random_{key}"] = tk.BooleanVar(
                value=key == "z"
            )

        headers = ("항목", "시작", "종료", "변화량", "랜덤")
        for column, header in enumerate(headers):
            ttk.Label(frame, text=header).grid(row=0, column=column, padx=4)
        rows = (
            ("X (m)", "x"),
            ("Y (m)", "y"),
            ("Z (m)", "z"),
            ("회전 (°)", "rotation"),
        )
        for row, (label, key) in enumerate(rows, start=1):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w")
            for column, prefix in enumerate(("start", "end", "step"), start=1):
                ttk.Entry(
                    frame,
                    textvariable=self.sweep_inputs[f"{prefix}_{key}"],
                    width=10,
                ).grid(row=row, column=column, padx=3, pady=3)
            random_checkbox = ttk.Checkbutton(
                frame,
                variable=self.sweep_inputs[f"random_{key}"],
            )
            if key == "rotation":
                random_checkbox.configure(state="disabled")
            random_checkbox.grid(row=row, column=4)

        self.sweep_summary = tk.StringVar(value="대기 중")
        self.sweep_execute_motion_var = tk.BooleanVar(value=False)
        ttk.Label(
            frame,
            text=(
                "각 XYZ 위치에서 전체 회전 범위를 Plan한 뒤 다음 위치로 이동합니다.\n"
                "랜덤 축은 위치마다 한 번 추출하며, 검출 결과가 없으면 건너뜁니다."
            ),
        ).grid(row=5, column=0, columnspan=5, sticky="w", pady=(6, 3))
        self.sweep_execute_checkbox = ttk.Checkbutton(
            frame,
            text="실제 로봇 실행",
            variable=self.sweep_execute_motion_var,
        )
        self.sweep_execute_checkbox.grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(2, 0)
        )
        self.sweep_start_button = ttk.Button(
            frame, text="자동 실행", command=self.start_sweep
        )
        self.sweep_start_button.grid(
            row=6, column=2, columnspan=2, sticky="ew", padx=(6, 0), pady=(2, 0)
        )
        self.sweep_stop_button = ttk.Button(
            frame,
            text="자동 테스트 + 로봇 정지",
            command=self.stop_sweep,
            state="disabled",
        )
        self.sweep_stop_button.grid(
            row=6, column=4, sticky="ew", padx=(6, 0), pady=(2, 0)
        )
        ttk.Label(frame, textvariable=self.sweep_summary).grid(
            row=7, column=0, columnspan=5, sticky="w", pady=(5, 0)
        )
        ttk.Separator(frame, orient="horizontal").grid(
            row=8, column=0, columnspan=5, sticky="ew", pady=5
        )
        self.sweep_statistics = tk.StringVar(
            value="완료 0 | 성공 0 | 실패 0 | 대체성공 0 | 성공률 0.0%"
        )
        self.sweep_failure_summary = tk.StringVar(value="실패 단계: 없음")
        self.sweep_result_path = tk.StringVar(value="결과 파일: 생성 전")
        ttk.Label(frame, text="실시간 통계").grid(
            row=9, column=0, columnspan=5, sticky="w"
        )
        ttk.Label(frame, textvariable=self.sweep_statistics).grid(
            row=10, column=0, columnspan=5, sticky="w", pady=(4, 0)
        )
        ttk.Label(
            frame,
            textvariable=self.sweep_failure_summary,
            wraplength=430,
        ).grid(row=11, column=0, columnspan=5, sticky="w", pady=(4, 0))
        ttk.Label(
            frame,
            textvariable=self.sweep_result_path,
            wraplength=430,
        ).grid(row=12, column=0, columnspan=5, sticky="w", pady=(4, 8))

        result_columns = (
            "case",
            "tomato",
            "result",
            "stage",
            "time",
            "sequence_time",
        )
        self.sweep_result_tree = ttk.Treeview(
            frame,
            columns=result_columns,
            show="headings",
            height=4,
            style="Sweep.Treeview",
        )
        ttk.Style(self.root).configure("Sweep.Treeview", rowheight=28)
        headings = {
            "case": "케이스",
            "tomato": "토마토",
            "result": "결과",
            "stage": "실패/복구 단계",
            "time": "시간(s)",
            "sequence_time": "실제 시퀀스(s)",
        }
        widths = {
            "case": 55,
            "tomato": 55,
            "result": 55,
            "stage": 235,
            "time": 70,
            "sequence_time": 100,
        }
        for column in result_columns:
            self.sweep_result_tree.heading(column, text=headings[column])
            self.sweep_result_tree.column(
                column,
                width=widths[column],
                anchor="center",
                stretch=column == "stage",
            )
        result_scrollbar = ttk.Scrollbar(
            frame,
            orient="vertical",
            command=self.sweep_result_tree.yview,
        )
        self.sweep_result_tree.configure(yscrollcommand=result_scrollbar.set)
        self.sweep_result_tree.bind(
            "<<TreeviewSelect>>",
            self._sweep_result_selected,
        )
        self.sweep_result_tree.grid(
            row=13, column=0, columnspan=4, sticky="nsew", pady=(0, 4)
        )
        result_scrollbar.grid(row=13, column=4, sticky="ns", pady=(0, 4))
        detail_frame = ttk.LabelFrame(
            frame,
            text="선택/최근 결과 상세",
            padding=4,
        )
        detail_frame.grid(
            row=14, column=0, columnspan=5, sticky="ew", pady=(3, 0)
        )
        detail_frame.columnconfigure(0, weight=1)
        self.sweep_result_detail = tk.Text(
            detail_frame,
            height=7,
            wrap="word",
            state="disabled",
        )
        detail_scrollbar = ttk.Scrollbar(
            detail_frame,
            orient="vertical",
            command=self.sweep_result_detail.yview,
        )
        self.sweep_result_detail.configure(
            yscrollcommand=detail_scrollbar.set
        )
        self.sweep_result_detail.grid(row=0, column=0, sticky="ew")
        detail_scrollbar.grid(row=0, column=1, sticky="ns")
        self._set_sweep_result_detail("없음")
        self.replay_sweep_button = ttk.Button(
            frame,
            text="선택 환경 재현",
            command=self.replay_selected_sweep_scene,
            state="disabled",
        )
        self.replay_sweep_button.grid(
            row=15,
            column=0,
            columnspan=5,
            sticky="ew",
            pady=(4, 0),
        )
        frame.rowconfigure(13, weight=1)

    def _append_log(self, message: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message.rstrip() + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    @staticmethod
    def _percent_to_scale(value: str, label: str) -> float:
        try:
            percent = float(value)
        except ValueError as error:
            raise ValueError(f"{label}은 숫자로 입력하세요.") from error
        if not 1.0 <= percent <= 100.0:
            raise ValueError(f"{label}은 1~100% 범위로 입력하세요.")
        return percent / 100.0

    @staticmethod
    def _wait_seconds(value: str) -> float:
        try:
            seconds = float(value)
        except ValueError as error:
            raise ValueError(
                "리니어모터 대기시간은 초 단위 숫자로 입력하세요."
            ) from error
        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError("리니어모터 대기시간은 0초 이상이어야 합니다.")
        return seconds

    def apply_motion_speed(self) -> None:
        """Apply GUI planning scales and request the RB controller speed bar."""
        try:
            speed_bar = self._percent_to_scale(
                self.speed_bar_percent.get(), "RB Speed Bar"
            )
            velocity = self._percent_to_scale(
                self.motion_velocity_percent.get(), "OMPL/Joint 속도"
            )
            acceleration = self._percent_to_scale(
                self.motion_acceleration_percent.get(), "OMPL/Joint 가속도"
            )
        except ValueError as error:
            messagebox.showerror("속도 입력 오류", str(error))
            return

        self.motion_velocity_scale = velocity
        self.motion_acceleration_scale = acceleration
        self._invalidate_plan()
        self._append_log(
            "[적용됨] OMPL/Joint 계획 속도: "
            f"OMPL/Joint 속도={velocity * 100:.0f}%, "
            f"가속도={acceleration * 100:.0f}% (다음 Plan부터)"
        )

        if not self.hardware_speed_client.service_is_ready():
            if not self.hardware_speed_client.wait_for_service(timeout_sec=0.05):
                self.status.set(
                    "시뮬레이션 속도 적용 완료 — OMPL/Joint만 적용, "
                    "RB Speed Bar·Cartesian은 미적용"
                )
                self._append_log(
                    "[시뮬레이션/하드웨어 미연결] RB Speed Bar 서비스가 "
                    "없어 Speed Bar 변경을 생략했습니다. "
                    "시뮬레이션 모드에서는 정상입니다. "
                    f"서비스={self.hardware_speed_service}"
                )
                self._append_log(
                    "[적용 범위] 이번 설정은 OMPL/Joint 궤적에만 적용됩니다. "
                    "Cartesian 궤적 속도와 RB Speed Bar에는 적용되지 않았습니다."
                )
                return

        request = SetSpeedBar.Request()
        request.speed = speed_bar
        self.apply_speed_button.configure(state="disabled")
        future = self.hardware_speed_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._speed_bar_applied(completed, speed_bar)
        )

    def _speed_bar_applied(self, future, requested_scale: float) -> None:
        self.apply_speed_button.configure(state="normal")
        try:
            response = future.result()
        except Exception as error:
            self.status.set(
                "계획 속도만 적용됨 — RB Speed Bar 서비스 호출 실패"
            )
            self._append_log(
                "[부분 적용] OMPL/Joint 계획 속도는 적용됐지만 "
                f"RB Speed Bar 서비스 호출은 실패했습니다: {error}"
            )
            return
        if not response.success:
            self.status.set(
                "계획 속도만 적용됨 — RB 컨트롤러가 Speed Bar 변경을 거부함"
            )
            self._append_log(
                "[부분 적용] OMPL/Joint 계획 속도는 적용됐지만 "
                "RB 컨트롤러가 Speed Bar 변경을 거부했습니다: "
                f"요청={requested_scale * 100:.0f}%"
            )
            return
        self.status.set(
            "실제 로봇 속도 적용 완료 — "
            f"RB Speed Bar {requested_scale * 100:.0f}%, "
            f"OMPL/Joint {self.motion_velocity_scale * 100:.0f}%"
        )
        self._append_log(
            "[실제 로봇 적용됨] "
            f"RB Speed Bar={requested_scale * 100:.0f}%, "
            f"OMPL/Joint 속도={self.motion_velocity_scale * 100:.0f}%, "
            f"가속도={self.motion_acceleration_scale * 100:.0f}%"
        )

    def _set_sweep_result_detail(self, message: str) -> None:
        self.sweep_result_detail.configure(state="normal")
        self.sweep_result_detail.delete("1.0", "end")
        self.sweep_result_detail.insert("1.0", message.rstrip())
        self.sweep_result_detail.configure(state="disabled")

    def _publish_harvest_result_markers(self) -> None:
        if not self.harvest_results and not self.sweep_markers:
            return
        message = MarkerArray()
        message.markers = [
            harvest_result_marker(
                index,
                success,
                self.result_arrow_lengths.get(index, 0.04),
                adaptive_rotation_applied=(
                    self.harvest_result_adaptive_rotation.get(index, False)
                ),
                adaptive_rotation_deg=(
                    self.harvest_result_adaptive_rotation_deg.get(index, 0.0)
                ),
                approach_axis_local=(
                    self.harvest_result_approach_axis_local.get(index)
                ),
            )
            for index, success in sorted(self.harvest_results.items())
        ]
        message.markers.extend(self.sweep_markers)
        self.result_marker_publisher.publish(message)

    def _set_harvest_result(
        self,
        tomato_index: int,
        success: bool,
        adaptive_rotation_applied: bool = False,
        adaptive_rotation_deg: float = 0.0,
        approach_axis_local=None,
    ) -> None:
        self._update_result_arrow_length(tomato_index)
        self.harvest_results[tomato_index] = success
        self.harvest_result_adaptive_rotation[tomato_index] = bool(
            success and adaptive_rotation_applied
        )
        self.harvest_result_adaptive_rotation_deg[tomato_index] = float(
            adaptive_rotation_deg
        )
        if approach_axis_local is None:
            self.harvest_result_approach_axis_local.pop(tomato_index, None)
        else:
            self.harvest_result_approach_axis_local[tomato_index] = tuple(
                float(value) for value in approach_axis_local[:2]
            )
        self.clear_markers_button.configure(state="normal")
        self._publish_harvest_result_markers()

    def _clear_harvest_results(self) -> None:
        marker = Marker()
        marker.action = Marker.DELETEALL
        message = MarkerArray()
        message.markers = [marker]
        self.result_marker_publisher.publish(message)
        self.harvest_results.clear()
        self.harvest_result_adaptive_rotation.clear()
        self.harvest_result_adaptive_rotation_deg.clear()
        self.harvest_result_approach_axis_local.clear()
        self.sweep_markers.clear()
        self.sweep_marker_next_id = 0
        self.clear_markers_button.configure(state="disabled")

    def clear_harvest_result_markers(self) -> None:
        self._clear_harvest_results()
        self.result_arrow_lengths.clear()
        self.status.set("수확 결과 마커를 지웠습니다.")
        self._append_log("수확 결과 마커 전체 삭제")

    def show_failure_goal_state(self) -> None:
        state_data = self.last_failure_robot_state
        if not state_data:
            messagebox.showinfo(
                "실패 자세 없음",
                "먼저 실패한 Plan-only 결과가 필요합니다.",
            )
            return
        joint_names = [
            str(name) for name in state_data.get("joint_names", [])
        ]
        joint_positions = [
            float(position)
            for position in state_data.get("joint_positions", [])
        ]
        if not joint_names or len(joint_names) != len(joint_positions):
            messagebox.showerror(
                "실패 자세 오류",
                "저장된 실패 관절 상태가 올바르지 않습니다.",
            )
            return
        message = RobotState()
        message.is_diff = False
        message.joint_state.header.stamp = self.get_clock().now().to_msg()
        message.joint_state.name = joint_names
        message.joint_state.position = joint_positions
        self.rviz_goal_state_publisher.publish(message)
        stage = state_data.get("stage", "UNKNOWN")
        self.status.set(f"{stage} 실패 직전 자세를 RViz Goal에 표시했습니다.")
        self._append_log(
            f"RViz Query Goal State 갱신: stage={stage}, "
            f"trajectory_point={state_data.get('trajectory_point_index', -1)}"
        )
        if self.rviz_goal_state_publisher.get_subscription_count() == 0:
            self._append_log(
                "RViz goal-state 구독자가 없습니다. RViz를 재시작하고 "
                "MoveIt_Allow_External_Program 설정을 확인하세요."
            )

    def _read_sweep_inputs(self):
        keys = ("x", "y", "z", "rotation")
        try:
            start = [
                float(self.sweep_inputs[f"start_{key}"].get()) for key in keys
            ]
            end = [
                float(self.sweep_inputs[f"end_{key}"].get()) for key in keys
            ]
            step = [
                float(self.sweep_inputs[f"step_{key}"].get()) for key in keys
            ]
        except ValueError as error:
            raise ValueError("시작, 종료, 변화량에 숫자를 입력하세요.") from error
        randomized = [
            self.sweep_inputs[f"random_{key}"].get() for key in keys
        ]
        # Rotation is exhaustive, not random. Only X/Y/Z support random sampling.
        randomized[3] = False
        random_seed = random.SystemRandom().randrange(0, 2**32)
        cases = generate_sweep_cases(
            start,
            end,
            step,
            randomized,
            rng=random.Random(random_seed),
        )
        return cases, {
            "start": start,
            "end": end,
            "step": step,
            "randomized": randomized,
            "random_seed": random_seed,
        }

    def _start_sweep_session(self, cases, input_config) -> None:
        base_directory = Path(
            str(self.get_parameter("results_directory").value)
        ).expanduser()
        session_name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        session_directory = base_directory / session_name
        session_directory.mkdir(parents=True, exist_ok=False)
        pipeline, planner_id, preapproach_mode = (
            self._selected_planner_config()
        )
        metadata = {
            "created_at": datetime.now().astimezone().isoformat(),
            "pipeline": pipeline,
            "planner_id": planner_id,
            "preapproach_mode": preapproach_mode,
            "execute_motion": self.sweep_execute_motion,
            "linear_motor_wait_sec": self.sweep_harvest_wait_sec,
            "tomato_selection": "all_detected_tomatoes_per_case",
            "case_count": len(cases),
            "plan_count": None,
            "input": input_config,
            "cases": [
                {
                    "case": index,
                    "x": values[0],
                    "y": values[1],
                    "z": values[2],
                    "rotation_deg": values[3],
                }
                for index, values in enumerate(cases, start=1)
            ],
        }
        (session_directory / "session.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        with (session_directory / "results.csv").open(
            "w", encoding="utf-8", newline=""
        ) as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=SWEEP_CSV_FIELDS)
            writer.writeheader()
        (session_directory / "results.jsonl").touch()

        self.sweep_session_dir = session_directory
        self.sweep_started_monotonic = time.monotonic()
        self.sweep_success_count = 0
        self.sweep_recovery_count = 0
        self.sweep_failure_counts.clear()
        self.sweep_result_records.clear()
        for item in self.sweep_result_tree.get_children():
            self.sweep_result_tree.delete(item)
        self.replay_sweep_button.configure(state="disabled")
        self.sweep_statistics.set(
            "완료 0 | 성공 0 | 실패 0 | 대체성공 0 | 성공률 0.0%"
        )
        self.sweep_failure_summary.set("실패 단계: 없음")
        self._set_sweep_result_detail("없음")
        self.sweep_result_path.set(f"결과 파일: {session_directory}")

    def _save_sweep_result(self, record) -> None:
        if self.sweep_session_dir is None:
            return
        with (self.sweep_session_dir / "results.jsonl").open(
            "a", encoding="utf-8"
        ) as jsonl_file:
            jsonl_file.write(json.dumps(record, ensure_ascii=False) + "\n")
            jsonl_file.flush()
        with (self.sweep_session_dir / "results.csv").open(
            "a", encoding="utf-8", newline=""
        ) as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=SWEEP_CSV_FIELDS)
            writer.writerow({field: record.get(field, "") for field in SWEEP_CSV_FIELDS})
            csv_file.flush()

    def _update_sweep_statistics(self, record) -> None:
        if record["success"]:
            self.sweep_success_count += 1
            if record.get("recovery_success"):
                self.sweep_recovery_count += 1
        else:
            self.sweep_failure_counts[
                record.get("failure_stage") or "UNKNOWN"
            ] += 1
        completed = self.sweep_completed + 1
        failures = completed - self.sweep_success_count
        success_rate = 100.0 * self.sweep_success_count / completed
        elapsed = max(0.0, time.monotonic() - self.sweep_started_monotonic)
        average = elapsed / completed
        self.sweep_statistics.set(
            f"완료 {completed} | 성공 {self.sweep_success_count} | "
            f"실패 {failures} | 대체성공 {self.sweep_recovery_count} | "
            f"성공률 {success_rate:.1f}% | 평균 {average:.2f}s"
        )
        if self.sweep_failure_counts:
            failures_by_stage = ", ".join(
                f"{stage} {count}"
                for stage, count in self.sweep_failure_counts.most_common()
            )
            self.sweep_failure_summary.set(f"실패 단계: {failures_by_stage}")
        else:
            self.sweep_failure_summary.set("실패 단계: 없음")

        item_id = self.sweep_result_tree.insert(
            "",
            0,
            values=(
                record["case"],
                record["tomato"],
                "성공" if record["success"] else "실패",
                record.get("display_stage_summary", ""),
                f"{record.get('duration_sec', 0.0):.2f}",
                sweep_execution_duration_text(record),
            ),
        )
        self.sweep_result_records[item_id] = dict(record)
        self.sweep_result_tree.selection_set(item_id)
        result_text = "성공" if record["success"] else "실패"
        detail = record.get("display_stage_detail") or "단계 정보 없음"
        adaptive_detail = (
            f"적응 접근각 "
            f"{record.get('adaptive_grasp_rotation_deg', 0.0):+.1f}° "
            f"(로봇 방향 오차 "
            f"{record.get('adaptive_grasp_current_error_deg', 0.0):.1f}°"
            f"→{record.get('adaptive_grasp_selected_error_deg', 0.0):.1f}°)"
        )
        sequence_duration = sweep_execution_duration_text(record)
        if sequence_duration != "-":
            sequence_duration += "s"
        self._set_sweep_result_detail(
            f"최근 결과 상세: 케이스 {record['case']}, "
            f"토마토 {record['tomato']}, {result_text}\n"
            f"계획 {float(record.get('duration_sec', 0.0)):.2f}s | "
            f"실제 전체 시퀀스 {sequence_duration}\n"
            f"{adaptive_detail}\n{detail}"
        )
        children = self.sweep_result_tree.get_children()
        for old_item in children[100:]:
            self.sweep_result_tree.delete(old_item)
            self.sweep_result_records.pop(old_item, None)

    def _selected_sweep_record(self):
        selection = self.sweep_result_tree.selection()
        if not selection:
            return None
        return self.sweep_result_records.get(selection[0])

    def _sweep_result_selected(self, _event=None) -> None:
        record = self._selected_sweep_record()
        if record is None:
            self.replay_sweep_button.configure(state="disabled")
            return
        result_text = "성공" if record["success"] else "실패"
        detail = record.get("display_stage_detail") or "단계 정보 없음"
        adaptive_detail = (
            f"적응 접근각 "
            f"{record.get('adaptive_grasp_rotation_deg', 0.0):+.1f}° "
            f"(로봇 방향 오차 "
            f"{record.get('adaptive_grasp_current_error_deg', 0.0):.1f}°"
            f"→{record.get('adaptive_grasp_selected_error_deg', 0.0):.1f}°)"
        )
        self._set_sweep_result_detail(
            f"선택 결과: 케이스 {record['case']}, "
            f"토마토 {record['tomato']}, {result_text}\n"
            f"환경 X={record['x']:.4f}, Y={record['y']:.4f}, "
            f"Z={record['z']:.4f}, 회전={record['rotation_deg']:.1f}°\n"
            f"{adaptive_detail}\n{detail}"
        )
        busy = (
            self.harvest_process is not None
            or self.batch_active
            or self.sweep_active
            or self.sweep_worker_process is not None
        )
        self.replay_sweep_button.configure(
            state="disabled" if busy else "normal"
        )

    def replay_selected_sweep_scene(self) -> None:
        record = self._selected_sweep_record()
        if record is None:
            messagebox.showinfo(
                "결과 선택 필요",
                "재현할 자동 실행 결과 항목을 먼저 선택하세요.",
            )
            return
        if (
            self.harvest_process is not None
            or self.batch_active
            or self.sweep_active
            or self.sweep_worker_process is not None
        ):
            messagebox.showinfo(
                "실행 중",
                "현재 작업이 끝난 후 선택 환경을 재현하세요.",
            )
            return

        self.scene_x.set(f"{float(record['x']):.4f}")
        self.scene_y.set(f"{float(record['y']):.4f}")
        self.scene_z.set(f"{float(record['z']):.4f}")
        self.scene_rotation.set(f"{float(record['rotation_deg']):.1f}")
        self.preserve_sweep_markers_on_scene_set = True
        self._append_log(
            f"자동 실행 환경 재현 요청: case={record['case']}, "
            f"tomato={record['tomato']}, "
            f"X={float(record['x']):.4f}, Y={float(record['y']):.4f}, "
            f"Z={float(record['z']):.4f}, "
            f"회전={float(record['rotation_deg']):.1f}°"
        )
        self.set_scene_position()

    def start_sweep(self) -> None:
        if (
            self.harvest_process is not None
            or self.batch_active
            or self.sweep_active
            or self.sweep_worker_process is not None
        ):
            messagebox.showinfo("실행 중", "현재 작업이 끝날 때까지 기다려 주세요.")
            return
        if not self.scene_set_client.service_is_ready():
            if not self.scene_set_client.wait_for_service(timeout_sec=0.05):
                messagebox.showerror(
                    "장면 노드 연결 실패", f"연결되지 않음: {self.scene_node}"
                )
                return
        if not self.camera_client.service_is_ready():
            if not self.camera_client.wait_for_service(timeout_sec=0.05):
                messagebox.showerror(
                    "카메라 연결 실패", f"연결되지 않음: {self.camera_service}"
                )
                return
        try:
            harvest_wait_sec = self._wait_seconds(
                self.linear_motor_wait_sec.get()
            )
        except ValueError as error:
            messagebox.showerror("대기시간 입력 오류", str(error))
            return
        execute_motion = bool(self.sweep_execute_motion_var.get())
        if execute_motion and not messagebox.askyesno(
            "자동 실제 로봇 실행",
            "자동 테스트에서 Plan에 성공한 모든 토마토 모션을 실제로 "
            "실행합니다.\n\n"
            "각 수확 후 PICK_READY로 복귀하며, 실제 실행 실패 시 자동 "
            "테스트를 즉시 중단합니다.\n"
            "로봇 주변이 안전한지 확인했습니까?",
            icon="warning",
        ):
            return
        self.sweep_execute_motion = execute_motion
        self.sweep_harvest_wait_sec = harvest_wait_sec
        try:
            cases, input_config = self._read_sweep_inputs()
            input_config["linear_motor_wait_sec"] = harvest_wait_sec
            self._start_sweep_session(cases, input_config)
        except (OSError, ValueError) as error:
            messagebox.showerror("자동 테스트 입력 오류", str(error))
            return
        if not self._start_sweep_worker():
            messagebox.showerror(
                "Planner 시작 실패", "자동 테스트 Planner를 시작하지 못했습니다."
            )
            return

        self.sweep_active = True
        self.sweep_cancel_requested = False
        self.sweep_cases = deque(cases)
        self.sweep_case_total = len(cases)
        self.sweep_case_number = 0
        self.sweep_total = 0
        self.sweep_completed = 0
        self.sweep_case_tomato_total = 0
        self.sweep_case_tomato_completed = 0
        self.sweep_tomato_queue.clear()
        self.sweep_current_case = None
        self.sweep_pending_verification = None
        self.sweep_summary.set(
            f"케이스 0 / {self.sweep_case_total} — 테스트 시작"
        )
        self._append_log(
            f"자동 {'Plan+실제 실행' if self.sweep_execute_motion else 'Plan'} "
            f"테스트 시작: {self.sweep_case_total}개 케이스, "
            "각 케이스에서 검출된 토마토 전체를 처리, "
            f"리니어모터 대기={self.sweep_harvest_wait_sec:.2f}s"
        )
        self._set_busy(True)
        self._start_next_sweep_case()

    def _start_sweep_worker(self) -> bool:
        command = [
            sys.executable,
            "-m",
            "rbpodo_tomato_harvest.tomato_harvest_worker",
        ]
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=environment,
            )
        except OSError as error:
            self._append_log(f"지속 Planner 시작 오류: {error}")
            return False
        self.sweep_worker_process = process
        thread = threading.Thread(
            target=self._read_sweep_worker,
            args=(process,),
            daemon=True,
        )
        thread.start()
        self._append_log("자동 테스트용 지속 Planner 프로세스 시작")
        return True

    def _read_sweep_worker(self, process) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                line = line.rstrip()
                if line.startswith(SWEEP_RESULT_PREFIX):
                    try:
                        result = json.loads(line[len(SWEEP_RESULT_PREFIX):])
                    except json.JSONDecodeError as error:
                        self.process_queue.put(
                            ("log", f"Planner 결과 해석 실패: {error}")
                        )
                        continue
                    self.process_queue.put(("sweep_result", result, process))
                elif is_critical_process_output(line):
                    self.process_queue.put(("log", line))
        return_code = process.wait()
        self.process_queue.put(("sweep_worker_done", return_code, process))

    def _shutdown_sweep_worker(self, force: bool = False) -> None:
        process = self.sweep_worker_process
        if process is None or process.poll() is not None:
            return
        if force:
            process.terminate()
            return
        try:
            if process.stdin is not None:
                process.stdin.write('{"command":"quit"}\n')
                process.stdin.flush()
        except (BrokenPipeError, OSError):
            process.terminate()

    def stop_sweep(self) -> None:
        if not self.sweep_active:
            return
        self.sweep_cancel_requested = True
        self.sweep_cases.clear()
        self.sweep_tomato_queue.clear()
        self.status.set("자동 테스트 중지 및 로봇 모션 정지 명령 전송 중...")
        self._append_log(
            "[정지 요청] 남은 자동 테스트를 취소하고 현재 로봇 모션 정지를 "
            "요청합니다."
        )
        self._request_robot_motion_stop()
        if self.harvest_process is not None:
            self.harvest_process.terminate()
        self._shutdown_sweep_worker(force=True)
        self._finish_sweep(
            "자동 테스트 중지 완료 — MoveIt/controller 취소 및 RB 정지를 요청했습니다."
        )

    def _request_robot_motion_stop(self) -> None:
        """Cancel active ROS trajectories and request an RB controller stop."""
        self._request_action_cancel(
            self.moveit_cancel_client,
            "MoveIt execute_trajectory",
            self.moveit_cancel_service,
        )
        self._request_action_cancel(
            self.controller_cancel_client,
            "joint_trajectory_controller",
            self.controller_cancel_service,
        )

        if not self.hardware_stop_client.service_is_ready():
            if not self.hardware_stop_client.wait_for_service(timeout_sec=0.05):
                self._append_log(
                    "[시뮬레이션] RB task_stop 서비스가 없어 하드웨어 정지는 "
                    "생략했습니다. MoveIt/controller trajectory 취소만 적용됩니다. "
                    f"서비스={self.hardware_stop_service}"
                )
                return
        request = TaskStop.Request()
        request.timeout = 2.0
        future = self.hardware_stop_client.call_async(request)
        future.add_done_callback(self._hardware_stop_completed)
        self._append_log("[정지 명령 전송] 실제 RB5 task_stop 요청")

    def _request_action_cancel(self, client, label: str, service_name: str) -> None:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.05):
                self._append_log(
                    f"[정지 경고] {label} 취소 서비스를 찾지 못했습니다: "
                    f"{service_name}"
                )
                return
        future = client.call_async(cancel_all_goals_request())
        future.add_done_callback(
            lambda completed, action_label=label: self._action_cancel_completed(
                completed, action_label
            )
        )
        self._append_log(f"[정지 명령 전송] {label} 활성 goal 전체 취소")

    def _action_cancel_completed(self, future, label: str) -> None:
        try:
            response = future.result()
        except Exception as error:
            self._append_log(f"[정지 실패] {label} 취소 서비스 오류: {error}")
            return
        if response.return_code == CancelGoal.Response.ERROR_NONE:
            self._append_log(
                f"[정지 확인] {label} 취소 수락: "
                f"goal {len(response.goals_canceling)}개"
            )
            return
        result_names = {
            CancelGoal.Response.ERROR_REJECTED: "취소 거부/활성 goal 없음",
            CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID: "goal을 찾지 못함",
            CancelGoal.Response.ERROR_GOAL_TERMINATED: "goal이 이미 종료됨",
        }
        detail = result_names.get(
            response.return_code, f"return_code={response.return_code}"
        )
        self._append_log(f"[정지 확인] {label}: {detail}")

    def _hardware_stop_completed(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self._append_log(f"[정지 실패] 실제 RB5 task_stop 오류: {error}")
            return
        if response.success:
            self._append_log("[정지 확인] 실제 RB5 task_stop 성공")
        else:
            self._append_log("[정지 실패] 실제 RB5가 task_stop 요청을 거부했습니다.")

    def _finish_sweep(self, message: str) -> None:
        completed = self.sweep_completed
        total = self.sweep_total
        if self.sweep_session_dir is not None:
            elapsed = max(
                0.0, time.monotonic() - self.sweep_started_monotonic
            )
            summary = {
                "finished_at": datetime.now().astimezone().isoformat(),
                "message": message,
                "completed": completed,
                "total": total,
                "success": self.sweep_success_count,
                "failure": completed - self.sweep_success_count,
                "cartesian_return_recovery": self.sweep_recovery_count,
                "success_rate": (
                    self.sweep_success_count / completed if completed else 0.0
                ),
                "elapsed_sec": elapsed,
                "failure_stages": dict(self.sweep_failure_counts),
                "execute_motion": self.sweep_execute_motion,
            }
            try:
                (self.sweep_session_dir / "summary.json").write_text(
                    json.dumps(summary, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
            except OSError as error:
                self._append_log(f"통계 요약 저장 실패: {error}")
        self.sweep_active = False
        self.sweep_cancel_requested = False
        self.sweep_cases.clear()
        self.sweep_tomato_queue.clear()
        self.sweep_current_case = None
        self.sweep_pending_verification = None
        self._shutdown_sweep_worker()
        self._set_busy(False)
        self.sweep_summary.set(f"전체 완료 {completed}개 — {message}")
        self.status.set(message)
        self._append_log(message)

    def _start_next_sweep_case(self) -> None:
        if not self.sweep_active:
            return
        if self.sweep_cancel_requested:
            self._finish_sweep("사용자가 자동 테스트를 중지했습니다.")
            return
        if not self.sweep_cases:
            self._finish_sweep(
                f"자동 Plan 테스트 완료: {self.sweep_completed}개"
            )
            return

        self.sweep_current_case = self.sweep_cases.popleft()
        self.sweep_case_number += 1
        x, y, z, rotation = self.sweep_current_case
        self.scene_x.set(f"{x:.4f}")
        self.scene_y.set(f"{y:.4f}")
        self.scene_z.set(f"{z:.4f}")
        self.scene_rotation.set(f"{rotation:.1f}")
        request = SetParameters.Request()
        request.parameters = scene_parameters((x, y, z), rotation)
        future = self.scene_set_client.call_async(request)
        future.add_done_callback(self._sweep_scene_set_done)
        case_number = self.sweep_case_number
        self.sweep_summary.set(
            f"케이스 {case_number} / {self.sweep_case_total} — 위치/회전 적용 중"
        )
        self._append_log(
            f"[케이스 {case_number}/{self.sweep_case_total}] "
            f"X={x:.4f}, Y={y:.4f}, Z={z:.4f}, 회전={rotation:.1f}°"
        )

    def _sweep_scene_set_done(self, future) -> None:
        if not self.sweep_active or self.sweep_cancel_requested:
            self._finish_sweep("사용자가 자동 테스트를 중지했습니다.")
            return
        try:
            response = future.result()
            failures = [result for result in response.results if not result.successful]
            if failures:
                reasons = "; ".join(result.reason for result in failures)
                raise RuntimeError(reasons or "장면 변경 거부")
        except Exception as error:
            self._finish_sweep(f"자동 테스트 장면 적용 실패: {error}")
            return
        self.detection_signature = None
        self.sweep_summary.set(
            f"케이스 {self.sweep_case_number} / {self.sweep_case_total} — 촬영/검출 중"
        )
        future = self.camera_client.call_async(DetectTomatoes.Request())
        future.add_done_callback(self._sweep_detection_done)

    def _sweep_detection_done(self, future) -> None:
        if not self.sweep_active or self.sweep_cancel_requested:
            self._finish_sweep("사용자가 자동 테스트를 중지했습니다.")
            return
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)

            self._detections_callback(response.detections)
            tomato_count = len(self.detected_tomatoes)

            self.sweep_tomato_queue = deque(range(tomato_count))
            self.sweep_case_tomato_total = tomato_count
            self.sweep_case_tomato_completed = 0
            self.sweep_total += tomato_count

            self._append_log(
                f"[케이스 {self.sweep_case_number}/"
                f"{self.sweep_case_total}] 검출 토마토 {tomato_count}개"
            )
        except Exception as error:
            self._finish_sweep(f"자동 테스트 검출 실패: {error}")
            return

        if not self.sweep_tomato_queue:
            self.sweep_summary.set(
                f"케이스 {self.sweep_case_number} / "
                f"{self.sweep_case_total} — 검출 토마토 없음, 다음 케이스 이동"
            )
            self.root.after(100, self._start_next_sweep_case)
            return

        self.sweep_summary.set(
            f"케이스 {self.sweep_case_number} / {self.sweep_case_total} — "
            f"토마토 0 / {self.sweep_case_tomato_total} 준비 완료"
        )
        self.root.after(100, self._start_sweep_plan)

    def _start_sweep_plan(self) -> None:
        if not self.sweep_active or self.sweep_cancel_requested:
            return
        if not self.sweep_tomato_queue:
            self.root.after(100, self._start_next_sweep_case)
            return
        tomato_index = self.sweep_tomato_queue.popleft()
        self.tomato_combo.current(tomato_index)
        target_item = str(tomato_index)
        self.tomato_tree.selection_set(target_item)
        self.tomato_tree.focus(target_item)
        self.tomato_tree.see(target_item)
        pipeline, planner_id, preapproach_mode = (
            self._selected_planner_config()
        )
        verification = (
            self.detection_generation,
            tomato_index,
            pipeline,
            planner_id,
            preapproach_mode,
        )
        current_number = self.sweep_case_tomato_completed + 1
        self.sweep_summary.set(
            f"케이스 {self.sweep_case_number} / {self.sweep_case_total} — "
            f"토마토 {current_number} / {self.sweep_case_tomato_total} "
            f"(index {tomato_index}) "
            f"{'Plan+실행' if self.sweep_execute_motion else 'Plan'} 중"
        )
        process = self.sweep_worker_process
        if process is None or process.poll() is not None or process.stdin is None:
            self._finish_sweep("자동 테스트 지속 Planner가 종료되었습니다.")
            return
        self.sweep_request_id += 1
        self.sweep_pending_verification = (
            self.sweep_request_id,
            verification,
        )
        request = {
            "request_id": self.sweep_request_id,
            "tomato_frame": f"detected_tomato_{tomato_index}_tf",
            "planning_pipeline_id": pipeline,
            "planner_id": planner_id,
            "preapproach_mode": preapproach_mode,
            "execute": self.sweep_execute_motion,
            "velocity_scale": self.motion_velocity_scale,
            "acceleration_scale": self.motion_acceleration_scale,
            "harvest_wait_sec": self.sweep_harvest_wait_sec,
        }
        try:
            process.stdin.write(json.dumps(request) + "\n")
            process.stdin.flush()
        except (BrokenPipeError, OSError) as error:
            self._finish_sweep(f"지속 Planner 요청 전송 실패: {error}")

    def _source_to_result_parent_quaternion(self, source_frame: str):
        parent_frame = str(
            self.get_parameter("result_marker_parent_frame").value
        )
        if source_frame == parent_frame:
            return (0.0, 0.0, 0.0, 1.0)
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                source_frame,
                Time(),
            )
        except TransformException as error:
            self._append_log(
                f"화살표 길이용 TF를 찾지 못해 3D 거리를 사용합니다: {error}"
            )
            return None
        rotation = transform.transform.rotation
        return (rotation.x, rotation.y, rotation.z, rotation.w)

    def _update_result_arrow_length(self, tomato_index: int) -> None:
        if not 0 <= tomato_index < len(self.detected_tomatoes):
            return
        source_to_parent_quaternion = self._source_to_result_parent_quaternion(
            self.result_detection_frame
        )
        minimum_length = float(
            self.get_parameter("result_arrow_minimum_length").value
        )
        if source_to_parent_quaternion is None:
            self.result_arrow_lengths[tomato_index] = minimum_length
            return
        detection = self.detected_tomatoes[tomato_index]
        self.result_arrow_lengths[tomato_index] = tomato_stem_arrow_length(
            detection.center,
            detection.stem_point,
            source_to_parent_quaternion,
            float(self.get_parameter("result_arrow_stem_margin").value),
            minimum_length,
        )

    def _detection_key(self, message: TomatoDetectionArray):
        stamp = message.header.stamp
        points = tuple(
            (
                detection.id,
                detection.center.x,
                detection.center.y,
                detection.center.z,
                detection.stem_point.x,
                detection.stem_point.y,
                detection.stem_point.z,
            )
            for detection in message.detections
        )
        return (stamp.sec, stamp.nanosec, message.header.frame_id, points)

    def _detections_callback(self, message: TomatoDetectionArray) -> None:
        signature = self._detection_key(message)
        if signature == self.detection_signature:
            return
        self.result_arrow_lengths.clear()
        self.result_detection_frame = message.header.frame_id
        self.detection_signature = signature
        self.detected_tomatoes = list(message.detections)
        self.detection_generation += 1
        self._invalidate_plan()

        for item in self.tomato_tree.get_children():
            self.tomato_tree.delete(item)
        choices = []
        for index, detection in enumerate(self.detected_tomatoes):
            frame = f"detected_tomato_{index}_tf"
            choices.append(f"{index}: {frame}")
            center = detection.center
            self.tomato_tree.insert(
                "",
                "end",
                iid=str(index),
                values=(
                    index,
                    frame,
                    detection.id,
                    f"{center.x:.4f}",
                    f"{center.y:.4f}",
                    f"{center.z:.4f}",
                ),
            )
        self.tomato_combo.configure(values=choices)
        busy = (
            self.harvest_process is not None
            or self.batch_active
            or self.sweep_active
        )
        if choices:
            self.tomato_combo.current(0)
            self.tomato_tree.selection_set("0")
            self.tomato_tree.focus("0")
            self.plan_button.configure(state="disabled" if busy else "normal")
            self.harvest_all_button.configure(
                state="disabled" if busy else "normal"
            )
        else:
            self.selected_tomato.set("")
            self.plan_button.configure(state="disabled")
            self.harvest_all_button.configure(state="disabled")
        self.status.set(f"토마토 {len(choices)}개 검출됨")
        self._append_log(
            f"[{message.header.frame_id}] 새 검출 결과: 토마토 {len(choices)}개"
        )

    def detect_tomatoes(self) -> None:
        if not self.camera_client.service_is_ready():
            if not self.camera_client.wait_for_service(timeout_sec=0.05):
                self.status.set(f"카메라 서비스 연결 안 됨: {self.camera_service}")
                self._append_log(
                    f"카메라 서비스를 먼저 실행하세요: {self.camera_service}"
                )
                return
        self.detect_button.configure(state="disabled")
        self.status.set("카메라 촬영 및 토마토 검출 요청 중...")
        self._append_log(f"서비스 호출: {self.camera_service}")
        future = self.camera_client.call_async(DetectTomatoes.Request())
        future.add_done_callback(self._detection_service_done)

    def _detection_service_done(self, future) -> None:
        self.detect_button.configure(state="normal")
        try:
            response = future.result()
        except Exception as error:
            self.status.set("카메라 서비스 호출 실패")
            self._append_log(f"카메라 서비스 오류: {error}")
            return
        if not response.success:
            self.status.set("토마토 검출 실패")
            self._append_log(f"검출 실패: {response.message}")
            return
        self._detections_callback(response.detections)
        self.status.set(response.message)
        self._append_log(response.message)

    def _selected_index(self) -> int | None:
        value = self.selected_tomato.get()
        if not value:
            return None
        try:
            return int(value.split(":", maxsplit=1)[0])
        except (ValueError, IndexError):
            return None

    def _tree_selection_changed(self, _event=None) -> None:
        selection = self.tomato_tree.selection()
        if not selection:
            return
        index = int(selection[0])
        self.tomato_combo.current(index)
        self._selection_changed()

    def _combo_selection_changed(self, _event=None) -> None:
        index = self._selected_index()
        if index is not None and self.tomato_tree.exists(str(index)):
            self.tomato_tree.selection_set(str(index))
            self.tomato_tree.focus(str(index))
            self.tomato_tree.see(str(index))
        self._selection_changed()

    def _selection_changed(self) -> None:
        self._invalidate_plan()
        index = self._selected_index()
        if index is not None:
            self.status.set(f"토마토 {index} 선택됨 — Plan-only를 먼저 실행하세요.")

    def _planner_selection_changed(self, _event=None) -> None:
        self._invalidate_plan()
        self._clear_harvest_results()
        self.status.set(
            f"{self.selected_planner.get()} 선택됨 — Plan-only를 다시 실행하세요."
        )

    def _selected_planner_config(self) -> tuple[str, str, str]:
        return PLANNER_CONFIGS.get(
            self.selected_planner.get(),
            PLANNER_CONFIGS["OMPL / RRTConnect"],
        )

    def _invalidate_plan(self) -> None:
        self.verified_plan = None
        self.execute_button.configure(state="disabled")

    def start_harvest(self, execute: bool) -> None:
        index = self._selected_index()
        if index is None:
            messagebox.showwarning("토마토 선택", "수확할 토마토를 먼저 선택하세요.")
            return
        if self.harvest_process is not None or self.batch_active or self.sweep_active:
            messagebox.showinfo("실행 중", "현재 모션 작업이 끝날 때까지 기다려 주세요.")
            return
        try:
            harvest_wait_sec = self._wait_seconds(
                self.linear_motor_wait_sec.get()
            )
        except ValueError as error:
            messagebox.showerror("대기시간 입력 오류", str(error))
            return
        pipeline, planner_id, preapproach_mode = (
            self._selected_planner_config()
        )
        verification = (
            self.detection_generation,
            index,
            pipeline,
            planner_id,
            preapproach_mode,
        )
        if execute and self.verified_plan != verification:
            messagebox.showwarning(
                "Plan-only 필요",
                "현재 토마토의 Plan-only가 성공한 뒤 실제 수확을 실행할 수 있습니다.",
            )
            return
        if execute and not messagebox.askyesno(
            "실제 로봇 수확 실행",
            f"토마토 {index}의 전체 수확 모션을 실제로 실행할까요?\n\n"
            "로봇 주변이 안전하고 교시 모드가 해제되었는지 확인하세요.",
            icon="warning",
        ):
            return

        self._launch_harvest_process(
            index,
            execute,
            verification,
            harvest_wait_sec=harvest_wait_sec,
        )

    def start_harvest_all(self) -> None:
        if self.harvest_process is not None or self.batch_active or self.sweep_active:
            messagebox.showinfo("실행 중", "현재 모션 작업이 끝날 때까지 기다려 주세요.")
            return
        tomato_count = len(self.detected_tomatoes)
        if tomato_count == 0:
            messagebox.showwarning("토마토 검출", "수확할 토마토를 먼저 검출하세요.")
            return
        try:
            harvest_wait_sec = self._wait_seconds(
                self.linear_motor_wait_sec.get()
            )
        except ValueError as error:
            messagebox.showerror("대기시간 입력 오류", str(error))
            return
        if not messagebox.askyesno(
            "검출 토마토 전체 연속 수확",
            f"검출된 토마토 {tomato_count}개를 순서대로 실제 수확할까요?\n\n"
            "Plan-only 실패 토마토는 건너뛰며, 실제 실행 실패 시 중단됩니다.\n"
            "로봇 주변이 안전하고 교시 모드가 해제되었는지 확인하세요.",
            icon="warning",
        ):
            return

        self.batch_active = True
        self.batch_jobs = deque(harvest_all_jobs(tomato_count))
        self.batch_generation = self.detection_generation
        self.batch_total = tomato_count
        self.batch_completed = 0
        self.batch_skipped = 0
        self.batch_planner = self._selected_planner_config()
        self.batch_harvest_wait_sec = harvest_wait_sec
        self._invalidate_plan()
        self._clear_harvest_results()
        self._append_log(
            f"전체 연속 수확 시작: 토마토 {tomato_count}개, "
            f"planner={self.batch_planner[0]}/{self.batch_planner[1]}, "
            f"preapproach={self.batch_planner[2]}, "
            f"리니어모터 대기={self.batch_harvest_wait_sec:.2f}s"
        )
        self._set_busy(True)
        self._start_next_batch_job()

    def _start_next_batch_job(self) -> None:
        if not self.batch_active:
            return
        if self.batch_generation != self.detection_generation:
            self._finish_batch(False, "검출 결과가 변경되어 전체 수확을 중단했습니다.")
            return
        if not self.batch_jobs:
            self._finish_batch(
                True,
                f"전체 연속 수확 완료: 수확 {self.batch_completed}개, "
                f"계획 실패 {self.batch_skipped}개",
            )
            return

        index, execute = self.batch_jobs.popleft()
        pipeline, planner_id, preapproach_mode = self.batch_planner
        verification = (
            self.batch_generation,
            index,
            pipeline,
            planner_id,
            preapproach_mode,
        )
        if execute and self.verified_plan != verification:
            self._finish_batch(
                False,
                f"토마토 {index}의 Plan-only 검증이 없어 전체 수확을 중단했습니다.",
            )
            return

        if self.tomato_tree.exists(str(index)):
            self.tomato_combo.current(index)
            self.tomato_tree.selection_set(str(index))
            self.tomato_tree.focus(str(index))
            self.tomato_tree.see(str(index))
        if not self._launch_harvest_process(
            index,
            execute,
            verification,
            harvest_wait_sec=self.batch_harvest_wait_sec,
        ):
            self._finish_batch(
                False,
                f"토마토 {index} 작업 프로세스를 시작하지 못해 전체 수확을 중단했습니다.",
            )

    def _launch_harvest_process(
        self,
        index: int,
        execute: bool,
        verification,
        harvest_wait_sec: float,
    ) -> bool:
        pipeline, planner_id, preapproach_mode = verification[2:5]
        command = harvest_command(
            index,
            execute,
            planning_pipeline_id=pipeline,
            planner_id=planner_id,
            preapproach_mode=preapproach_mode,
            publish_display_trajectory=not self.sweep_active,
            velocity_scale=self.motion_velocity_scale,
            acceleration_scale=self.motion_acceleration_scale,
            harvest_wait_sec=harvest_wait_sec,
        )
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        mode = "실제 수확" if execute else "Plan-only"
        if not execute:
            self.harvest_plan_report = {}
            self.last_failure_robot_state = None
            self.failure_goal_button.configure(state="disabled")
        batch_prefix = (
            f"[전체 {self.batch_completed + self.batch_skipped + 1}/"
            f"{self.batch_total}] "
            if self.batch_active
            else ""
        )
        self._append_log(
            f"{batch_prefix}{mode} 시작: detected_tomato_{index}_tf "
            f"planner={pipeline}/{planner_id}, "
            f"preapproach={preapproach_mode}, "
            f"리니어모터 대기={harvest_wait_sec:.2f}s"
        )
        self.status.set(f"{batch_prefix}{mode} 실행 중...")
        self._set_busy(True)
        try:
            self.harvest_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=environment,
            )
        except OSError as error:
            self.harvest_process = None
            self._set_busy(False)
            self.status.set(f"{mode} 시작 실패")
            self._append_log(f"프로세스 시작 오류: {error}")
            return False
        thread = threading.Thread(
            target=self._read_process,
            args=(self.harvest_process, execute, verification),
            daemon=True,
        )
        thread.start()
        return True

    def _read_process(self, process, execute: bool, verification) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                line = line.rstrip()
                if line.startswith(PLAN_RESULT_PREFIX):
                    try:
                        report = json.loads(line[len(PLAN_RESULT_PREFIX):])
                    except json.JSONDecodeError as error:
                        self.process_queue.put(
                            ("log", f"Plan 결과 해석 실패: {error}")
                        )
                        continue
                    self.process_queue.put(("plan_report", report, process))
                elif is_critical_process_output(line):
                    self.process_queue.put(("log", line))
        return_code = process.wait()
        self.process_queue.put(
            ("done", return_code, execute, verification, process)
        )

    def _drain_process_queue(self) -> None:
        while True:
            try:
                item = self.process_queue.get_nowait()
            except queue.Empty:
                break
            if item[0] == "log":
                self._append_log(item[1])
                continue
            if item[0] == "sweep_result":
                _, result, process = item
                if process is not self.sweep_worker_process:
                    continue
                pending = self.sweep_pending_verification
                if pending is None:
                    self._append_log("대기 중인 요청이 없어 Planner 결과를 무시합니다.")
                    continue
                request_id, verification = pending
                if int(result.get("request_id", -1)) != request_id:
                    self._append_log(
                        f"Planner 요청 ID 불일치: expected={request_id}, "
                        f"received={result.get('request_id')}"
                    )
                    continue
                self.sweep_pending_verification = None
                if result.get("error"):
                    self._append_log(f"지속 Planner 오류: {result['error']}")
                report = result.get("report") or {}
                if not bool(result.get("success")) and not report.get(
                    "failure_stage"
                ):
                    report = {
                        **report,
                        "failure_stage": "WORKER_ERROR",
                        "failure_planner_type": "worker",
                        "failure_reason": result.get("error")
                        or "UNKNOWN_WORKER_FAILURE",
                    }
                self._handle_sweep_plan_done(
                    0 if bool(result.get("success")) else 1,
                    verification,
                    report,
                )
                continue
            if item[0] == "plan_report":
                _, report, process = item
                if process is not self.harvest_process:
                    continue
                self.harvest_plan_report = report
                self._append_log(concise_plan_report(report))
                failure_state = report.get("failure_robot_state")
                if failure_state:
                    self.last_failure_robot_state = failure_state
                continue
            if item[0] == "sweep_worker_done":
                _, return_code, process = item
                if process is not self.sweep_worker_process:
                    continue
                self.sweep_worker_process = None
                if self.sweep_active:
                    self._finish_sweep(
                        f"자동 테스트 지속 Planner가 비정상 종료했습니다. "
                        f"(종료 코드 {return_code})"
                    )
                continue
            _, return_code, execute, verification, process = item
            if process is not self.harvest_process:
                continue
            self.harvest_process = None
            if self.sweep_active:
                self._handle_sweep_plan_done(return_code, verification)
                continue
            if self.batch_active:
                self._handle_batch_job_done(return_code, execute, verification)
                continue
            self._set_busy(False)
            mode = "실제 수확" if execute else "Plan-only"
            if return_code == 0:
                self._set_harvest_result(
                    verification[1],
                    True,
                    adaptive_rotation_applied=adaptive_rotation_was_applied(
                        self.harvest_plan_report
                    ),
                    adaptive_rotation_deg=adaptive_rotation_degrees(
                        self.harvest_plan_report
                    ),
                    approach_axis_local=adaptive_approach_axis_local(
                        self.harvest_plan_report
                    ),
                )
                self.status.set(f"{mode} 완료")
                self._append_log(f"{mode} 완료 (종료 코드 0)")
                if not execute and verification[0] == self.detection_generation:
                    self.verified_plan = verification
                    if self._selected_index() == verification[1]:
                        self.execute_button.configure(state="normal")
                else:
                    self._invalidate_plan()
            else:
                self._set_harvest_result(
                    verification[1],
                    False,
                    adaptive_rotation_deg=adaptive_rotation_degrees(
                        self.harvest_plan_report
                    ),
                    approach_axis_local=adaptive_approach_axis_local(
                        self.harvest_plan_report
                    ),
                )
                self.status.set(f"{mode} 실패 — 로그를 확인하세요.")
                self._append_log(f"{mode} 실패 (종료 코드 {return_code})")
                if self.last_failure_robot_state:
                    self.failure_goal_button.configure(state="normal")
                    stage = self.last_failure_robot_state.get(
                        "stage", "UNKNOWN"
                    )
                    self._append_log(
                        f"{stage} 실패 직전 자세를 저장했습니다. "
                        "'실패 자세 → RViz Goal' 버튼으로 확인할 수 있습니다."
                    )
                self._invalidate_plan()
        if not self.closing:
            self.root.after(50, self._drain_process_queue)

    def _handle_sweep_plan_done(self, return_code, verification, report=None) -> None:
        if self.sweep_cancel_requested:
            self._finish_sweep("사용자가 자동 테스트를 중지했습니다.")
            return
        tomato_index = verification[1]
        report = report or {}
        adaptive_grasp = report.get("adaptive_grasp", {})
        success = return_code == 0
        x, y, z, rotation = self.sweep_current_case
        record = {
            "timestamp": datetime.now().astimezone().isoformat(),
            "case": self.sweep_case_number,
            "tomato": tomato_index,
            "x": x,
            "y": y,
            "z": z,
            "rotation_deg": rotation,
            "success": success,
            "pipeline": verification[2],
            "planner_id": verification[3],
            "preapproach_mode": verification[4],
            "execute_motion": self.sweep_execute_motion,
            "execution_attempted": bool(
                report.get("execution_attempted", False)
            ),
            "execution_success": report.get("execution_success", ""),
            "execution_duration_sec": report.get(
                "execution_duration_sec", ""
            ),
            "failure_stage": report.get("failure_stage", ""),
            "failure_planner_type": report.get(
                "failure_planner_type", ""
            ),
            "failure_reason": report.get("failure_reason", ""),
            "recovery_used": bool(report.get("recovery_used", False)),
            "recovery_success": bool(report.get("recovery_success", False)),
            "recovery_stage": report.get("recovery_stage", ""),
            "recovery_reason": report.get("recovery_reason", ""),
            "moveit_error_code": report.get("moveit_error_code", ""),
            "cartesian_fraction": report.get("cartesian_fraction", ""),
            "required_fraction": report.get("required_fraction", ""),
            "duration_sec": report.get("duration_sec", 0.0),
            "stages": report.get("stages", []),
            "cartesian_fallbacks": report.get("cartesian_fallbacks", []),
            "adaptive_grasp_rotation_deg": adaptive_grasp.get(
                "applied_rotation_deg", 0.0
            ),
            "adaptive_grasp_current_error_deg": adaptive_grasp.get(
                "current_robot_error_deg", 0.0
            ),
            "adaptive_grasp_selected_error_deg": adaptive_grasp.get(
                "selected_robot_error_deg", 0.0
            ),
        }
        fallback_summary = cartesian_fallback_summary(report)
        if fallback_summary:
            record.update(fallback_summary)
        record["display_stage_summary"] = sweep_stage_summary(record)
        record["display_stage_detail"] = sweep_stage_detail(record)
        self._save_sweep_result(record)
        self._update_sweep_statistics(record)
        self._update_result_arrow_length(tomato_index)
        parent_frame = str(
            self.get_parameter("result_marker_parent_frame").value
        )
        tomato_frame = f"detected_tomato_{tomato_index}_tf"
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                tomato_frame,
                Time(),
            )
        except TransformException as error:
            self._append_log(f"결과 마커용 TF 조회 실패: {error}")
            transform = None

        if transform is not None:
            marker = sweep_result_marker(
                self.sweep_marker_next_id,
                success,
                self.result_arrow_lengths.get(tomato_index, 0.04),
                parent_frame,
                transform,
                adaptive_rotation_applied=(
                    success
                    and abs(
                        float(record["adaptive_grasp_rotation_deg"])
                    )
                    > 1e-6
                ),
                adaptive_rotation_deg=float(
                    record["adaptive_grasp_rotation_deg"]
                ),
                approach_axis_local=adaptive_approach_axis_local(report),
            )
            self.sweep_marker_next_id += 1
            self.sweep_markers.append(marker)
            self.clear_markers_button.configure(state="normal")
            self._publish_harvest_result_markers()
        self.sweep_completed += 1
        self.sweep_case_tomato_completed += 1
        result = "성공" if return_code == 0 else "실패"
        operation = "Plan+실행" if self.sweep_execute_motion else "Plan"
        self.sweep_summary.set(
            f"케이스 {self.sweep_case_number} / {self.sweep_case_total} — "
            f"토마토 {self.sweep_case_tomato_completed} / "
            f"{self.sweep_case_tomato_total} {operation} {result} "
            f"(전체 완료 {self.sweep_completed})"
        )
        self._append_log(
            f"[케이스 {self.sweep_case_number}/{self.sweep_case_total}] "
            f"[토마토 {self.sweep_case_tomato_completed}/"
            f"{self.sweep_case_tomato_total}] "
            f"index {tomato_index} {operation} {result} "
            f"(전체 완료 {self.sweep_completed})"
        )
        if record["execution_attempted"] and not record["execution_success"]:
            self._finish_sweep(
                f"토마토 {tomato_index} 실제 실행 실패로 자동 테스트를 "
                "중단했습니다."
            )
            return
        self.root.after(100, self._start_sweep_plan)

    def _handle_batch_job_done(self, return_code, execute: bool, verification) -> None:
        index = verification[1]
        mode = "실제 수확" if execute else "Plan-only"
        if return_code != 0:
            self._set_harvest_result(
                index,
                False,
                adaptive_rotation_deg=adaptive_rotation_degrees(
                    self.harvest_plan_report
                ),
                approach_axis_local=adaptive_approach_axis_local(
                    self.harvest_plan_report
                ),
            )
            if not execute:
                self.batch_skipped += 1
                if self.batch_jobs and self.batch_jobs[0] == (index, True):
                    self.batch_jobs.popleft()
                self._invalidate_plan()
                self._append_log(
                    f"[전체 {self.batch_completed + self.batch_skipped}/"
                    f"{self.batch_total}] 토마토 {index} Plan-only 실패 — 건너뜀"
                )
                self._start_next_batch_job()
                return
            self._finish_batch(
                False,
                f"토마토 {index} {mode} 실패로 전체 수확을 중단했습니다. "
                f"(종료 코드 {return_code})",
            )
            return
        if verification[0] != self.detection_generation:
            self._finish_batch(False, "검출 결과가 변경되어 전체 수확을 중단했습니다.")
            return

        self._set_harvest_result(
            index,
            True,
            adaptive_rotation_applied=adaptive_rotation_was_applied(
                self.harvest_plan_report
            ),
            adaptive_rotation_deg=adaptive_rotation_degrees(
                self.harvest_plan_report
            ),
            approach_axis_local=adaptive_approach_axis_local(
                self.harvest_plan_report
            ),
        )
        self._append_log(
            f"[전체 {self.batch_completed + self.batch_skipped + 1}/"
            f"{self.batch_total}] "
            f"토마토 {index} {mode} 완료"
        )
        if execute:
            self.batch_completed += 1
            self._invalidate_plan()
        else:
            self.verified_plan = verification
        self._start_next_batch_job()

    def _finish_batch(self, success: bool, message: str) -> None:
        self.batch_active = False
        self.batch_jobs.clear()
        self.batch_generation = None
        self.batch_planner = None
        self._invalidate_plan()
        self._set_busy(False)
        self.status.set(message)
        result = "완료" if success else "중단"
        self._append_log(f"[전체 연속 수확 {result}] {message}")

    def _set_busy(self, busy: bool) -> None:
        state = "disabled" if busy else "normal"
        self.detect_button.configure(state=state)
        self.read_scene_button.configure(state=state)
        self.set_scene_button.configure(state=state)
        self.apply_speed_button.configure(state=state)
        self.linear_motor_wait_entry.configure(state=state)
        self.tomato_combo.configure(state="disabled" if busy else "readonly")
        self.planner_combo.configure(state="disabled" if busy else "readonly")
        self.harvest_all_button.configure(
            state="disabled" if busy or not self.detected_tomatoes else "normal"
        )
        self.clear_markers_button.configure(
            state=(
                "disabled"
                if busy or (not self.harvest_results and not self.sweep_markers)
                else "normal"
            )
        )
        self.failure_goal_button.configure(
            state=(
                "disabled"
                if busy or not self.last_failure_robot_state
                else "normal"
            )
        )
        self.sweep_start_button.configure(state=state)
        self.sweep_execute_checkbox.configure(state=state)
        self.sweep_stop_button.configure(
            state="normal" if self.sweep_active else "disabled"
        )
        self.replay_sweep_button.configure(
            state=(
                "disabled"
                if busy or self._selected_sweep_record() is None
                else "normal"
            )
        )
        if busy or not self.detected_tomatoes:
            self.plan_button.configure(state="disabled")
        else:
            self.plan_button.configure(state="normal")
        if busy:
            self.execute_button.configure(state="disabled")
        elif self.verified_plan == (
            self.detection_generation,
            self._selected_index(),
            *self._selected_planner_config(),
        ):
            self.execute_button.configure(state="normal")

    def read_scene_position(self) -> None:
        if not self.scene_get_client.service_is_ready():
            if not self.scene_get_client.wait_for_service(timeout_sec=0.05):
                self._append_log(
                    f"장면 노드 연결 대기 중: {self.scene_node}"
                )
                return
        request = GetParameters.Request()
        request.names = ["object_position", "tomato_z_spin_deg"]
        future = self.scene_get_client.call_async(request)
        future.add_done_callback(self._scene_position_received)

    def _scene_position_received(self, future) -> None:
        try:
            response = future.result()
            values = list(response.values[0].double_array_value)
            if len(values) != 3:
                raise ValueError(f"object_position 길이가 {len(values)}입니다.")
            rotation_deg = float(response.values[1].double_value)
        except Exception as error:
            self._append_log(f"줄기 위치 읽기 실패: {error}")
            return
        self.scene_x.set(f"{values[0]:.4f}")
        self.scene_y.set(f"{values[1]:.4f}")
        self.scene_z.set(f"{values[2]:.4f}")
        self.scene_rotation.set(f"{rotation_deg:.1f}")
        self.status.set("현재 토마토 줄기 위치와 회전을 읽었습니다.")

    def set_scene_position(self) -> None:
        try:
            position = [
                float(self.scene_x.get()),
                float(self.scene_y.get()),
                float(self.scene_z.get()),
            ]
            rotation_deg = float(self.scene_rotation.get())
        except ValueError:
            self.preserve_sweep_markers_on_scene_set = False
            messagebox.showerror(
                "입력 오류", "줄기 X, Y, Z와 회전 각도에 숫자를 입력하세요."
            )
            return
        if not self.scene_set_client.service_is_ready():
            if not self.scene_set_client.wait_for_service(timeout_sec=0.05):
                self.preserve_sweep_markers_on_scene_set = False
                self.status.set(f"장면 노드 연결 안 됨: {self.scene_node}")
                return

        request = SetParameters.Request()
        request.parameters = scene_parameters(position, rotation_deg)
        future = self.scene_set_client.call_async(request)
        future.add_done_callback(self._scene_position_set)
        self.status.set("토마토 줄기 위치와 회전 적용 중...")

    def _scene_position_set(self, future) -> None:
        preserve_sweep_markers = self.preserve_sweep_markers_on_scene_set
        self.preserve_sweep_markers_on_scene_set = False
        try:
            response = future.result()
            failed_results = [
                result for result in response.results if not result.successful
            ]
        except Exception as error:
            self.status.set("토마토 줄기 위치 변경 실패")
            self._append_log(f"줄기 위치 변경 오류: {error}")
            return
        if failed_results:
            reasons = "; ".join(result.reason for result in failed_results)
            self.status.set("토마토 줄기 위치/회전 변경 거부됨")
            self._append_log(f"줄기 위치/회전 변경 거부: {reasons}")
            return
        self.detected_tomatoes = []
        if preserve_sweep_markers:
            self.harvest_results.clear()
            self.harvest_result_adaptive_rotation.clear()
            self._publish_harvest_result_markers()
            self.clear_markers_button.configure(
                state="normal" if self.sweep_markers else "disabled"
            )
        else:
            self._clear_harvest_results()
        self.result_arrow_lengths.clear()
        self.result_detection_frame = ""
        self.detection_signature = None
        self.detection_generation += 1
        for item in self.tomato_tree.get_children():
            self.tomato_tree.delete(item)
        self.tomato_combo.configure(values=[])
        self.selected_tomato.set("")
        self.plan_button.configure(state="disabled")
        self.harvest_all_button.configure(state="disabled")
        self._invalidate_plan()
        prefix = "선택 환경 재현 완료" if preserve_sweep_markers else "적용 완료"
        self.status.set(
            f"{prefix} — 카메라 검출을 다시 실행하세요."
        )
        self._append_log(
            "줄기 위치/회전을 변경했습니다. "
            + (
                "누적 결과 마커는 유지했습니다. "
                if preserve_sweep_markers
                else ""
            )
            + "기존 검출/계획은 사용하지 말고 다시 검출하세요."
        )

    def _spin_ros(self) -> None:
        if self.closing or not rclpy.ok():
            return
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except ExternalShutdownException:
            return
        self.root.after(30, self._spin_ros)

    def _on_close(self) -> None:
        if self.harvest_process is not None or self.sweep_worker_process is not None:
            if not messagebox.askyesno(
                "모션 작업 실행 중",
                "현재 프로세스를 종료하고 GUI를 닫을까요?\n"
                "프로세스 종료가 이미 전송된 로봇 궤적을 정지시키지는 않습니다.",
                icon="warning",
            ):
                return
            if self.harvest_process is not None:
                self.harvest_process.terminate()
            self._shutdown_sweep_worker(force=True)
        self.closing = True
        self.root.quit()

    def _signal_close(self, _signum, _frame) -> None:
        if self.harvest_process is not None:
            self.harvest_process.terminate()
        self._shutdown_sweep_worker(force=True)
        self.closing = True
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HarvestGui()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
