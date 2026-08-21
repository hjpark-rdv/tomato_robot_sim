import csv
import io
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
from PIL import Image, ImageDraw, ImageFont, ImageTk, UnidentifiedImageError
from action_msgs.srv import CancelGoal
from farmily_tomato_interfaces.msg import TomatoDetectionArray
from farmily_tomato_interfaces.srv import DebugFrame, DetectTomatoes
from geometry_msgs.msg import Point
from moveit_msgs.msg import RobotState
from rbpodo_msgs.srv import Eval, SetSpeedBar, TaskStop
from rbpodo_tomato_harvest.feedback_viewer import PlotCanvas, robot_top_projection
from rbpodo_tomato_harvest.tomato_tf_generator import (
    ANGLE_REFERENCE_BASE_TO_CENTER,
    ANGLE_REFERENCE_CALYX_TO_STEM,
    ANGLE_REFERENCE_CENTER_TO_STEM,
    ANGLE_REFERENCE_MODES,
    angle_reference_label,
    clustered_height_order,
    harvest_tf_frame_id,
)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image as RosImage
from std_msgs.msg import Bool, Float64, Float64MultiArray, Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


PLANNER_CONFIGS = {
    "Cartesian": ("ompl", "RRTConnect", "cartesian"),
    "OMPL / RRTConnect": ("ompl", "RRTConnect", "planner"),
    "CHOMP": ("chomp", "RRTConnect", "planner"),
    "PILZ / LIN": ("pilz_industrial_motion_planner", "LIN", "planner"),
}
GUI_PLANNER_CONFIG = PLANNER_CONFIGS["Cartesian"]
PICK_READY_STATES = ("PICK_READY", "PICK_READY_RIGHT")
NAMED_POSE_STATES = (*PICK_READY_STATES, "CAPTURE_LEFT", "CAPTURE_RIGHT")
CAMERA_SOURCE_FAKE = "Fake tomato"
CAMERA_SOURCE_REAL = "실제 /detect_tomatoes"
CAMERA_SOURCE_OPTIONS = (CAMERA_SOURCE_FAKE, CAMERA_SOURCE_REAL)
SERVO10_MAX_SPEED_DEG_PER_SEC = 180.0
SERVO10_DEFAULT_SPEED_PERCENT = 50
GRIPPER_EXTEND_AUTO_STOP_SECONDS = 3.0
ANGLE_REFERENCE_DISPLAY_OPTIONS = tuple(
    angle_reference_label(mode) for mode in ANGLE_REFERENCE_MODES
)
ANGLE_REFERENCE_MODE_BY_LABEL = {
    angle_reference_label(mode): mode for mode in ANGLE_REFERENCE_MODES
}


def pick_ready_state_for_capture_pose(
    capture_pose: str | None,
    default_state: str = "PICK_READY",
    target_link0_x: float | None = None,
) -> str:
    """Select the matching harvest-ready state for a camera pose."""
    pose = str(capture_pose or "").strip()
    if pose == "CAPTURE_RIGHT":
        return "PICK_READY_RIGHT"
    if pose == "CAPTURE_LEFT":
        return "PICK_READY"
    if target_link0_x is not None:
        target_x = float(target_link0_x)
        if not math.isfinite(target_x):
            raise ValueError("target_link0_x must be finite")
        # The RIGHT-side view sees the plant behind Link0.  This geometry is
        # still available after restarting the GUI, unlike transient camera
        # pose bookkeeping.
        return "PICK_READY_RIGHT" if target_x < 0.0 else "PICK_READY"
    state = str(default_state).strip()
    if state not in PICK_READY_STATES:
        raise ValueError(f"unsupported ready state: {state}")
    return state


VISION_REVIEW_ISSUES = {
    "문제 없음": "NO_ISSUE",
    "토마토 중심 좌표 불일치": "TOMATO_XYZ_MISMATCH",
    "줄기 좌표 불일치": "VINE_XYZ_MISMATCH",
    "토마토-줄기 매칭 불일치": "TOMATO_VINE_PAIRING_MISMATCH",
    "진입 방향/각도 이상": "APPROACH_DIRECTION_MISMATCH",
    "로봇 접근각 보정 과다": "ROBOT_REACHABILITY_CORRECTION_EXCESSIVE",
    "IK/충돌/Plan 실패": "ROBOT_PLAN_FAILURE",
    "기타": "OTHER",
}
VISION_REVIEW_OPTIONS = tuple(VISION_REVIEW_ISSUES)
BATCH_HARVEST_STAGE_OPTIONS = ("전체 수확", "3단계까지", "4단계까지")
STEP_CUSTOM_STAGE_DEFAULTS_MM = {
    3: (10.0, 0.0, 0.0),
    4: (40.0, 0.0, 0.0),
    6: (20.0, 0.0, 20.0),
    7: (0.0, 0.0, 20.0),
    8: (-50.0, 0.0, 0.0),
}
STEP_CUSTOM_SPEED_STAGE_NUMBERS = (*range(3, 9), 12)
STEP_CUSTOM_SPEED_DEFAULT_PERCENT_BY_STAGE = {
    stage_number: (100.0 if stage_number == 5 else 30.0)
    for stage_number in STEP_CUSTOM_SPEED_STAGE_NUMBERS
}
STEP_CUSTOM_SPEED_MIN_PERCENT = 10.0
STEP_CUSTOM_SPEED_MAX_PERCENT = 100.0
STEP_PREAPPROACH_FINAL_SPEED_DEFAULT_PERCENT = 30.0
STEP_CUSTOM_DELTA_LIMIT_MM = 200.0
SERVO10_CLOSE_ANGLE_DEG = 110
SERVO10_OPEN_ANGLE_DEG = 159
DETECTION_MARKER_NAMESPACE = "detected_tomato_preview"
HARVEST_RESULT_NAMESPACE = "harvest_plan_result"
HARVEST_SWEEP_NAMESPACE = "harvest_sweep_result"
HARVEST_APPROACH_NAMESPACE = "harvest_actual_approach"
HARVEST_SWEEP_APPROACH_NAMESPACE = "harvest_sweep_actual_approach"
SWEEP_RESULT_PREFIX = "__HARVEST_RESULT__"
PLAN_RESULT_PREFIX = "__HARVEST_PLAN_RESULT__"
NAMED_POSE_RESULT_PREFIX = "__NAMED_POSE_RESULT__"
PREPLANNED_BATCH_EVENT_PREFIX = "__HARVEST_PREPLANNED_BATCH_EVENT__"
STEPPER_EVENT_PREFIX = "__HARVEST_STEPPER_EVENT__"
PREPLANNED_BATCH_CONFIG_ENV = "HARVEST_PREPLANNED_BATCH_CONFIG"
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
    "pick_ready_state",
    "execute_motion",
    "continuous_transition",
    "return_to_pick_ready",
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
    """Return the final planned approach axis in tomato-local coordinates.

    Prefer the concrete final preapproach-to-target segment.  This geometry is
    generated after adaptive correction and therefore remains authoritative
    for failed plans as well.  The adaptive field is retained as a fallback
    for reports produced before approach geometry was available.
    """
    geometry = report.get("approach_geometry", {})
    preapproach = geometry.get("preapproach_position", [])
    target = geometry.get("target_position", [])
    if len(preapproach) >= 2 and len(target) >= 2:
        x = float(target[0]) - float(preapproach[0])
        y = float(target[1]) - float(preapproach[1])
        length = math.hypot(x, y)
        if length > 1e-9:
            return (x / length, y / length)

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
        f"시작/복귀={report.get('pick_ready_state_name', 'PICK_READY')} | "
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


def tomato_motion_result_text(
    execute: bool,
    success: bool,
    report,
) -> str:
    """Return the compact per-tomato result shown in the harvest table."""
    report = report or {}
    if report.get("step_cycle_only"):
        stage = int(report.get("step_cycle_last_stage", 0))
        mode = "실행" if execute else "Plan"
        return f"{stage}단계 {mode} {'성공' if success else '실패'}"
    if success:
        return "수확 성공" if execute else "Plan 성공"
    if not execute:
        return "Plan 실패"
    if not bool(report.get("execution_attempted", False)):
        return "실행 전 재계획 실패"
    return "수확 실행 실패"


def harvest_failure_summary(report) -> str:
    """Format the stage and reason behind a failed harvest child process."""
    report = report or {}
    stage = str(report.get("failure_stage") or "UNKNOWN")
    reason = str(report.get("failure_reason") or "원인 미보고")
    return f"{stage} / {reason}"


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


def harvest_statistics_record(
    *,
    case,
    tomato_index: int,
    scene,
    success: bool,
    verification,
    execute_motion: bool,
    report,
) -> dict:
    """Build one live-statistics row from a final tomato result."""
    x, y, z, rotation = scene
    adaptive_grasp = report.get("adaptive_grasp", {})
    record = {
        "timestamp": datetime.now().astimezone().isoformat(),
        "case": case,
        "tomato": tomato_index,
        "x": x,
        "y": y,
        "z": z,
        "rotation_deg": rotation,
        "success": bool(success),
        "pipeline": verification[2],
        "planner_id": verification[3],
        "preapproach_mode": verification[4],
        "pick_ready_state": (
            verification[5] if len(verification) > 5 else "PICK_READY"
        ),
        "execute_motion": bool(execute_motion),
        "continuous_transition": bool(
            report.get("continuous_transition", False)
        ),
        "return_to_pick_ready": bool(
            report.get("return_to_pick_ready", True)
        ),
        "execution_attempted": bool(
            report.get("execution_attempted", False)
        ),
        "execution_success": report.get("execution_success", ""),
        "execution_duration_sec": report.get("execution_duration_sec", ""),
        "failure_stage": report.get("failure_stage", ""),
        "failure_planner_type": report.get("failure_planner_type", ""),
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
    return record


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
    """Generate deterministic positions and sample every checked random axis.

    Deterministic X/Y/Z axes advance together until the earliest axis reaches
    its end. If none changes, one XYZ position is still generated. A
    deterministic rotation is exhaustively tested at every XYZ position; a
    randomized rotation contributes one sampled angle per XYZ position.
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
    for begin, finish, increment, use_random in zip(
        starts[:3], ends[:3], steps[:3], random_flags[:3]
    ):
        distance = abs(finish - begin)
        if distance < 1e-12:
            continue
        if use_random:
            continue
        if increment <= 0.0:
            raise ValueError(
                "변경되는 X/Y/Z 항목의 변화량은 0보다 커야 합니다."
            )
        position_interval_counts.append(int(math.ceil(distance / increment)))

    position_intervals = (
        min(position_interval_counts) if position_interval_counts else 0
    )
    position_count = position_intervals + 1

    random_rotation = random_flags[3] and abs(ends[3] - starts[3]) >= 1e-12
    rotation_values = (
        [None]
        if random_rotation
        else _inclusive_step_values(
            starts[3],
            ends[3],
            steps[3],
            "회전",
        )
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
        for rotation_value in rotation_values:
            xyz = []
            for begin, finish, increment, use_random in zip(
                starts[:3], ends[:3], steps[:3], random_flags[:3]
            ):
                distance = abs(finish - begin)
                if use_random and distance >= 1e-12:
                    xyz.append(
                        random_source.uniform(
                            min(begin, finish), max(begin, finish)
                        )
                    )
                else:
                    direction = 1.0 if finish >= begin else -1.0
                    moved = min(position_index * increment, distance)
                    xyz.append(begin + direction * moved)
            rotation = (
                random_source.uniform(
                    min(starts[3], ends[3]), max(starts[3], ends[3])
                )
                if random_rotation
                else rotation_value
            )
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
    harvest_x_forward_m: float = 0.040,
    harvest_tcp_wrist_oscillation_enabled: bool = True,
    harvest_tcp_wrist_rotation_deg: float = 10.0,
    continuous_transition: bool = False,
    return_to_pick_ready: bool = True,
    retreat_after_harvest: bool = False,
    harvest_stage_limit: int | None = None,
    pick_ready_state_name: str = "PICK_READY",
    prefer_robot_direction: bool = False,
    adaptive_grasp_max_rotation_deg: float = 45.0,
    tomato_frame: str | None = None,
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
    pick_ready_state_name = str(pick_ready_state_name)
    if pick_ready_state_name not in PICK_READY_STATES:
        raise ValueError(
            f"unsupported ready state: {pick_ready_state_name}"
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
    harvest_x_forward_m = float(harvest_x_forward_m)
    if not 0.010 <= harvest_x_forward_m <= 0.070:
        raise ValueError(
            "harvest_x_forward_m must be between 0.010 and 0.070"
        )
    harvest_tcp_wrist_rotation_deg = float(harvest_tcp_wrist_rotation_deg)
    if (
        not math.isfinite(harvest_tcp_wrist_rotation_deg)
        or not 1.0 <= harvest_tcp_wrist_rotation_deg <= 45.0
    ):
        raise ValueError(
            "harvest_tcp_wrist_rotation_deg must be between 1 and 45"
        )
    if harvest_stage_limit is not None:
        harvest_stage_limit = int(harvest_stage_limit)
        if harvest_stage_limit not in (3, 4):
            raise ValueError("harvest_stage_limit must be 3, 4, or None")
    adaptive_grasp_max_rotation_deg = float(
        adaptive_grasp_max_rotation_deg
    )
    if (
        not math.isfinite(adaptive_grasp_max_rotation_deg)
        or not 0.0 <= adaptive_grasp_max_rotation_deg <= 90.0
    ):
        raise ValueError(
            "adaptive_grasp_max_rotation_deg must be between 0 and 90"
        )
    executable = python_executable or sys.executable
    tomato_frame = str(
        tomato_frame or f"detected_tomato_{tomato_index}_tf"
    ).strip()
    if not tomato_frame:
        raise ValueError("tomato_frame must not be empty")
    command = [
        executable,
        "-m",
        "rbpodo_tomato_harvest.tomato_harvest_test",
        "--ros-args",
        "-p",
        f"tomato_frame:={tomato_frame}",
        "-p",
        f"execute:={'true' if execute else 'false'}",
        "-p",
        f"planning_pipeline_id:={planning_pipeline_id}",
        "-p",
        f"planner_id:={planner_id}",
        "-p",
        f"preapproach_mode:={preapproach_mode}",
        "-p",
        f"pick_ready_state_name:={pick_ready_state_name}",
        "-p",
        "publish_display_trajectory:="
        f"{'true' if publish_display_trajectory else 'false'}",
        "-p",
        f"pick_ready_velocity_scale:={float(velocity_scale)}",
        "-p",
        f"pick_ready_acceleration_scale:={float(acceleration_scale)}",
        "-p",
        f"harvest_wait_sec:={harvest_wait_sec}",
        "-p",
        f"harvest_x_forward:={harvest_x_forward_m}",
        "-p",
        "harvest_tcp_wrist_oscillation_enabled:="
        f"{'true' if harvest_tcp_wrist_oscillation_enabled else 'false'}",
        "-p",
        "harvest_tcp_wrist_rotation_deg:="
        f"{harvest_tcp_wrist_rotation_deg}",
        "-p",
        "continuous_transition:="
        f"{'true' if continuous_transition else 'false'}",
        "-p",
        "return_to_pick_ready:="
        f"{'true' if return_to_pick_ready else 'false'}",
        "-p",
        "retreat_after_harvest:="
        f"{'true' if retreat_after_harvest else 'false'}",
        "-p",
        "adaptive_grasp_prefer_robot_direction:="
        f"{'true' if prefer_robot_direction else 'false'}",
        "-p",
        "adaptive_grasp_max_rotation_deg:="
        f"{adaptive_grasp_max_rotation_deg}",
    ]
    if harvest_stage_limit is not None:
        command.extend(
            [
                "-p",
                "stepwise_plan:=true",
                "-p",
                "step_cycle_only:=true",
                "-p",
                f"step_cycle_last_stage:={harvest_stage_limit}",
            ]
        )
    return command


def named_pose_command(
    state_name: str,
    *,
    velocity_scale: float = 0.20,
    acceleration_scale: float = 0.20,
    python_executable: str | None = None,
) -> list[str]:
    """Build a constrained-OMPL Plan & Execute command for an SRDF pose."""
    state_name = str(state_name)
    if state_name not in NAMED_POSE_STATES:
        raise ValueError(f"unsupported named pose: {state_name}")
    for name, value in (
        ("velocity_scale", velocity_scale),
        ("acceleration_scale", acceleration_scale),
    ):
        if not 0.0 < float(value) <= 1.0:
            raise ValueError(f"{name} must be greater than 0 and at most 1")
    executable = python_executable or sys.executable
    return [
        executable,
        "-m",
        "rbpodo_tomato_harvest.named_pose_move",
        "--ros-args",
        "-r",
        "__node:=named_pose_move",
        "-p",
        f"pick_ready_state_name:={state_name}",
        "-p",
        # RViz named-state planning targets the stored joint values almost
        # exactly.  The harvest planner's wider default tolerance (0.005 rad)
        # lets OMPL choose a different endpoint inside the tolerance band on
        # every click, which makes repeated named-pose moves creep slightly.
        "pick_ready_joint_tolerance:=0.0001",
        "-p",
        "joint_planning_pipeline_id:=ompl",
        "-p",
        "joint_planner_id:=RRTConnect",
        "-p",
        f"pick_ready_velocity_scale:={float(velocity_scale)}",
        "-p",
        f"pick_ready_acceleration_scale:={float(acceleration_scale)}",
        "-p",
        "publish_display_trajectory:=true",
    ]


def preplanned_batch_command(
    tomato_count: int,
    *,
    continuous_arc: bool,
    tomato_frames: list[str] | tuple[str, ...] | None = None,
    execute: bool = True,
    planning_pipeline_id: str = "ompl",
    planner_id: str = "RRTConnect",
    preapproach_mode: str = "cartesian",
    velocity_scale: float = 0.2,
    acceleration_scale: float = 0.2,
    harvest_wait_sec: float = 2.0,
    harvest_x_forward_m: float = 0.040,
    harvest_tcp_wrist_rotation_deg: float = 10.0,
    harvest_stage_limit: int | None = None,
    pick_ready_state_name: str = "PICK_READY",
    prefer_robot_direction: bool = False,
    adaptive_grasp_max_rotation_deg: float = 45.0,
    python_executable: str | None = None,
) -> tuple[list[str], dict[str, str]]:
    """Build the complete-batch planner command and private config env."""
    if tomato_count <= 0:
        raise ValueError("tomato_count must be greater than zero")
    if tomato_frames is None:
        tomato_frames = tuple(
            f"detected_tomato_{index}_tf" for index in range(tomato_count)
        )
    else:
        tomato_frames = tuple(str(frame).strip() for frame in tomato_frames)
        if len(tomato_frames) != tomato_count or not all(tomato_frames):
            raise ValueError(
                "tomato_frames must contain one non-empty frame per tomato"
            )
    command = harvest_command(
        0,
        execute,
        tomato_frame=tomato_frames[0],
        planning_pipeline_id=planning_pipeline_id,
        planner_id=planner_id,
        preapproach_mode=preapproach_mode,
        publish_display_trajectory=True,
        velocity_scale=velocity_scale,
        acceleration_scale=acceleration_scale,
        harvest_wait_sec=harvest_wait_sec,
        harvest_x_forward_m=harvest_x_forward_m,
        harvest_tcp_wrist_rotation_deg=harvest_tcp_wrist_rotation_deg,
        continuous_transition=False,
        return_to_pick_ready=True,
        retreat_after_harvest=False,
        harvest_stage_limit=harvest_stage_limit,
        pick_ready_state_name=pick_ready_state_name,
        prefer_robot_direction=prefer_robot_direction,
        adaptive_grasp_max_rotation_deg=(
            adaptive_grasp_max_rotation_deg
        ),
        python_executable=python_executable,
    )
    command[2] = (
        "rbpodo_tomato_harvest.tomato_harvest_preplanned_batch"
    )
    environment = {
        PREPLANNED_BATCH_CONFIG_ENV: json.dumps(
            {
                "tomato_count": int(tomato_count),
                "tomato_frames": list(tomato_frames),
                "continuous_arc": bool(continuous_arc),
                "execute": bool(execute),
                "start_tolerance_deg": 3.0,
                "harvest_stage_limit": harvest_stage_limit,
                "candidate_attempts": 3,
            }
        )
    }
    return command, environment


def stepper_command(
    tomato_index: int,
    planning_pipeline_id: str = "ompl",
    planner_id: str = "RRTConnect",
    preapproach_mode: str = "cartesian",
    velocity_scale: float = 0.20,
    acceleration_scale: float = 0.20,
    harvest_wait_sec: float = 2.0,
    pick_ready_state_name: str = "PICK_READY",
    cycle_only: bool = False,
    cycle_last_stage: int = 5,
    cycle_forward_distance_m: float = 0.040,
    tcp_wrist_oscillation_enabled: bool = True,
    tcp_wrist_rotation_deg: float = 10.0,
    custom_stage_deltas_m: tuple[tuple[float, float, float], ...] | None = None,
    stage_speed_percents: tuple[float, ...] | None = None,
    preapproach_final_speed_percent: float = (
        STEP_PREAPPROACH_FINAL_SPEED_DEFAULT_PERCENT
    ),
    preapproach_via_enabled: bool = True,
    prefer_robot_direction: bool = False,
    adaptive_grasp_max_rotation_deg: float = 45.0,
    servo_speed_percent: float = SERVO10_DEFAULT_SPEED_PERCENT,
    servo_close_angle_deg: float = SERVO10_CLOSE_ANGLE_DEG,
    linear_motor_extend_seconds: float = 3.0,
    forward_wave_enabled: bool = False,
    tomato_frame: str | None = None,
    python_executable: str | None = None,
) -> list[str]:
    """Build the persistent detailed-step planner command."""
    cycle_last_stage = int(cycle_last_stage)
    if not 1 <= cycle_last_stage <= 7:
        raise ValueError("cycle_last_stage must be between 1 and 7")
    cycle_forward_distance_m = float(cycle_forward_distance_m)
    if not 0.010 <= cycle_forward_distance_m <= 0.070:
        raise ValueError(
            "cycle_forward_distance_m must be between 0.010 and 0.070"
        )
    if custom_stage_deltas_m is None:
        custom_stage_deltas_m = tuple(
            (
                cycle_forward_distance_m
                if stage_number == 4 and axis_index == 0
                else STEP_CUSTOM_STAGE_DEFAULTS_MM[stage_number][axis_index]
                / 1000.0
            )
            for stage_number in STEP_CUSTOM_STAGE_DEFAULTS_MM
            for axis_index in range(3)
        )
        custom_stage_deltas_m = tuple(
            custom_stage_deltas_m[index:index + 3]
            for index in range(0, len(custom_stage_deltas_m), 3)
        )
    try:
        custom_stage_deltas_m = tuple(
            tuple(float(value) for value in stage_delta)
            for stage_delta in custom_stage_deltas_m
        )
    except (TypeError, ValueError) as error:
        raise ValueError(
            "custom_stage_deltas_m must contain five XYZ triples"
        ) from error
    if (
        len(custom_stage_deltas_m) != 5
        or any(len(stage_delta) != 3 for stage_delta in custom_stage_deltas_m)
    ):
        raise ValueError(
            "custom_stage_deltas_m must contain five XYZ triples"
        )
    if any(
        not math.isfinite(value) or abs(value) > 0.200
        for stage_delta in custom_stage_deltas_m
        for value in stage_delta
    ):
        raise ValueError(
            "custom stage XYZ values must be finite and within +/-0.200 m"
        )
    if stage_speed_percents is None:
        stage_speed_percents = tuple(
            STEP_CUSTOM_SPEED_DEFAULT_PERCENT_BY_STAGE[stage_number]
            for stage_number in STEP_CUSTOM_SPEED_STAGE_NUMBERS
        )
    try:
        stage_speed_percents = tuple(
            float(value) for value in stage_speed_percents
        )
    except (TypeError, ValueError) as error:
        raise ValueError(
            "stage_speed_percents must contain seven numeric percentages"
        ) from error
    if len(stage_speed_percents) != len(STEP_CUSTOM_SPEED_STAGE_NUMBERS):
        raise ValueError(
            "stage_speed_percents must contain seven percentages"
        )
    if any(
        not math.isfinite(value)
        or not STEP_CUSTOM_SPEED_MIN_PERCENT
        <= value
        <= STEP_CUSTOM_SPEED_MAX_PERCENT
        for value in stage_speed_percents
    ):
        raise ValueError("stage speed percentages must be between 10 and 100")
    preapproach_final_speed_percent = float(
        preapproach_final_speed_percent
    )
    if not math.isfinite(preapproach_final_speed_percent) or not (
        STEP_CUSTOM_SPEED_MIN_PERCENT
        <= preapproach_final_speed_percent
        <= STEP_CUSTOM_SPEED_MAX_PERCENT
    ):
        raise ValueError(
            "preapproach_final_speed_percent must be between 10 and 100"
        )
    tcp_wrist_rotation_deg = float(tcp_wrist_rotation_deg)
    if (
        not math.isfinite(tcp_wrist_rotation_deg)
        or not 1.0 <= tcp_wrist_rotation_deg <= 45.0
    ):
        raise ValueError("tcp_wrist_rotation_deg must be between 1 and 45")
    servo_speed_percent = float(servo_speed_percent)
    if (
        not math.isfinite(servo_speed_percent)
        or not 1.0 <= servo_speed_percent <= 100.0
    ):
        raise ValueError("servo_speed_percent must be between 1 and 100")
    servo_close_angle_deg = float(servo_angle_degrees(servo_close_angle_deg))
    linear_motor_extend_seconds = step_linear_motor_extend_seconds(
        linear_motor_extend_seconds
    )
    command = harvest_command(
        tomato_index,
        True,
        tomato_frame=tomato_frame,
        planning_pipeline_id=planning_pipeline_id,
        planner_id=planner_id,
        preapproach_mode=preapproach_mode,
        publish_display_trajectory=True,
        velocity_scale=velocity_scale,
        acceleration_scale=acceleration_scale,
        harvest_wait_sec=harvest_wait_sec,
        harvest_x_forward_m=cycle_forward_distance_m,
        harvest_tcp_wrist_oscillation_enabled=(
            tcp_wrist_oscillation_enabled
        ),
        harvest_tcp_wrist_rotation_deg=tcp_wrist_rotation_deg,
        continuous_transition=False,
        return_to_pick_ready=not cycle_only,
        retreat_after_harvest=False,
        pick_ready_state_name=pick_ready_state_name,
        prefer_robot_direction=prefer_robot_direction,
        adaptive_grasp_max_rotation_deg=(
            adaptive_grasp_max_rotation_deg
        ),
        python_executable=python_executable,
    )
    command[2] = "rbpodo_tomato_harvest.tomato_harvest_stepper"
    command.extend(["-p", "stepwise_plan:=true"])
    command.extend(
        [
            "-p",
            "harvest_forward_wave_enabled:="
            f"{'true' if forward_wave_enabled else 'false'}",
        ]
    )
    command.extend(["-p", "step_custom_stage_deltas_enabled:=true"])
    for stage_number, stage_delta in zip(
        STEP_CUSTOM_STAGE_DEFAULTS_MM, custom_stage_deltas_m
    ):
        for axis_name, value in zip(("x", "y", "z"), stage_delta):
            command.extend(
                [
                    "-p",
                    f"step_stage_{stage_number}_{axis_name}_delta:={value}",
                ]
            )
    for stage_number, speed_percent in zip(
        STEP_CUSTOM_SPEED_STAGE_NUMBERS,
        stage_speed_percents,
    ):
        command.extend(
            [
                "-p",
                f"step_stage_{stage_number}_speed_percent:={speed_percent}",
            ]
        )
    command.extend(
        [
            "-p",
            "step_preapproach_final_speed_percent:="
            f"{preapproach_final_speed_percent}",
        ]
    )
    command.extend(
        [
            "-p",
            "preapproach_via_enabled:="
            f"{'true' if preapproach_via_enabled else 'false'}",
        ]
    )
    command.extend(["-p", f"step_cycle_last_stage:={cycle_last_stage}"])
    command.extend(
        ["-p", f"step_servo_speed_percent:={servo_speed_percent}"]
    )
    command.extend(
        ["-p", f"step_servo_close_angle_deg:={servo_close_angle_deg}"]
    )
    command.extend(
        [
            "-p",
            "step_linear_motor_extend_seconds:="
            f"{linear_motor_extend_seconds}",
        ]
    )
    command.extend(
        ["-p", f"step_cycle_only:={'true' if cycle_only else 'false'}"]
    )
    return command


def repeat_cycle_command(direction: str, last_stage_number: int) -> dict:
    """Build the persistent stepper command for a selected 1↔X range."""
    stage_number = int(last_stage_number)
    if not 1 <= stage_number <= 7:
        raise ValueError("repeat last stage must be between 1 and 7")
    actions = {
        "forward": "execute_cycle_forward",
        "reverse": "execute_cycle_reverse",
    }
    if direction not in actions:
        raise ValueError(f"unsupported repeat direction: {direction}")
    return {
        "command": actions[direction],
        "last_stage_index": stage_number - 1,
    }


def repeat_stage_command(
    direction: str,
    next_index: int,
    last_stage_number: int,
) -> dict | None:
    """Return one cached-stage command, or None when the cycle is complete."""
    stage_number = int(last_stage_number)
    index = int(next_index)
    if not 1 <= stage_number <= 7:
        raise ValueError("repeat last stage must be between 1 and 7")
    if direction == "forward":
        if index < 0 or index > stage_number:
            raise ValueError("forward repeat index is outside the cycle")
        return None if index == stage_number else {"command": "execute_next"}
    if direction == "reverse":
        if index < 0 or index > stage_number:
            raise ValueError("reverse repeat index is outside the cycle")
        return None if index == 0 else {"command": "execute_previous"}
    raise ValueError(f"unsupported repeat direction: {direction}")


def repeat_forward_distance_m(value) -> float:
    """Validate the operator's stage-four distance and convert mm to m."""
    try:
        millimeters = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("4단계 진입 길이는 숫자로 입력하세요.") from error
    if not math.isfinite(millimeters) or not 10.0 <= millimeters <= 70.0:
        raise ValueError("4단계 진입 길이는 10~70 mm 범위여야 합니다.")
    return millimeters / 1000.0


def step_custom_stage_deltas_m(values) -> tuple[tuple[float, float, float], ...]:
    """Validate GUI positional-stage XYZ millimetres and convert to metres."""
    converted = []
    for stage_number in STEP_CUSTOM_STAGE_DEFAULTS_MM:
        try:
            stage_values = values[stage_number]
            raw_values = (
                stage_values["x"],
                stage_values["y"],
                stage_values["z"],
            )
            xyz_mm = tuple(float(value) for value in raw_values)
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"{stage_number}단계 X/Y/Z는 숫자로 입력하세요."
            ) from error
        if any(not math.isfinite(value) for value in xyz_mm):
            raise ValueError(
                f"{stage_number}단계 X/Y/Z는 유한한 숫자여야 합니다."
            )
        if any(abs(value) > STEP_CUSTOM_DELTA_LIMIT_MM for value in xyz_mm):
            raise ValueError(
                f"{stage_number}단계 각 축은 "
                f"±{STEP_CUSTOM_DELTA_LIMIT_MM:g} mm 범위여야 합니다."
            )
        converted.append(tuple(value / 1000.0 for value in xyz_mm))
    return tuple(converted)


def step_stage_speed_percents(values) -> tuple[float, ...]:
    """Validate GUI speed percentages for detailed motion stages."""
    converted = []
    for stage_number in STEP_CUSTOM_SPEED_STAGE_NUMBERS:
        try:
            percent = float(values[stage_number])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"{stage_number}단계 속도는 숫자로 입력하세요."
            ) from error
        if not math.isfinite(percent) or not (
            STEP_CUSTOM_SPEED_MIN_PERCENT
            <= percent
            <= STEP_CUSTOM_SPEED_MAX_PERCENT
        ):
            raise ValueError(
                f"{stage_number}단계 속도는 "
                f"{STEP_CUSTOM_SPEED_MIN_PERCENT:g}~"
                f"{STEP_CUSTOM_SPEED_MAX_PERCENT:g}% 범위여야 합니다."
            )
        converted.append(percent)
    return tuple(converted)


def adaptive_grasp_max_rotation_degrees(value) -> float:
    """Validate the operator-configured adaptive grasp rotation limit."""
    try:
        degrees = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("최대 보정각은 도 단위 숫자로 입력하세요.") from error
    if not math.isfinite(degrees) or not 0.0 <= degrees <= 90.0:
        raise ValueError("최대 보정각은 0~90° 범위로 입력하세요.")
    return degrees


def servo_angle_degrees(value) -> int:
    """Validate and normalize a pin 10 servo angle from the GUI."""
    try:
        degrees = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("서보 각도는 숫자로 입력하세요.") from error
    if not math.isfinite(degrees) or not 10.0 <= degrees <= 173.0:
        raise ValueError("서보 각도는 10~173° 범위여야 합니다.")
    return int(round(degrees))


def step_linear_motor_extend_seconds(value) -> float:
    """Validate the detailed-step linear motor extension duration."""
    try:
        seconds = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            "리니어모터 늘림 시간은 초 단위 숫자로 입력하세요."
        ) from error
    if not math.isfinite(seconds) or not 0.0 <= seconds <= 60.0:
        raise ValueError("리니어모터 늘림 시간은 0~60초 범위여야 합니다.")
    return seconds


def servo_speed_percent(value) -> int:
    """Validate and normalize a PIN10 servo speed percentage."""
    try:
        percent = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("서보 속도는 숫자로 입력하세요.") from error
    if not math.isfinite(percent) or not 1.0 <= percent <= 100.0:
        raise ValueError("서보 속도는 1~100% 범위여야 합니다.")
    return int(round(percent))


def servo_speed_degrees_per_second(percent) -> float | None:
    """Return the provisional servo rate; 100% means immediate movement."""
    normalized = servo_speed_percent(percent)
    if normalized >= 100:
        return None
    return SERVO10_MAX_SPEED_DEG_PER_SEC * normalized / 100.0


def lift_harvest_target_height_mm(
    tomato_world_z_m: float,
    offset_m: float = 0.40,
    minimum_mm: float = 0.0,
    maximum_mm: float = 750.0,
) -> float:
    """Return the bounded Bottom-relative lift target for one tomato."""
    values = (
        float(tomato_world_z_m),
        float(offset_m),
        float(minimum_mm),
        float(maximum_mm),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("lift harvest height inputs must be finite")
    tomato_z, offset, lower, upper = values
    if offset < 0.0:
        raise ValueError("lift harvest offset must be zero or greater")
    if upper < lower:
        raise ValueError("lift harvest maximum must not be below minimum")
    requested_mm = (tomato_z - offset) * 1000.0
    return max(lower, min(upper, requested_mm))


def transformed_point_xyz(point, transform) -> tuple[float, float, float]:
    """Transform a point with a geometry_msgs TransformStamped value."""
    x = float(point.x)
    y = float(point.y)
    z = float(point.z)
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    qx = float(rotation.x)
    qy = float(rotation.y)
    qz = float(rotation.z)
    qw = float(rotation.w)
    return (
        float(translation.x)
        + (1 - 2 * (qy * qy + qz * qz)) * x
        + 2 * (qx * qy - qz * qw) * y
        + 2 * (qx * qz + qy * qw) * z,
        float(translation.y)
        + 2 * (qx * qy + qz * qw) * x
        + (1 - 2 * (qx * qx + qz * qz)) * y
        + 2 * (qy * qz - qx * qw) * z,
        float(translation.z)
        + 2 * (qx * qz - qy * qw) * x
        + 2 * (qy * qz + qx * qw) * y
        + (1 - 2 * (qx * qx + qy * qy)) * z,
    )


def detection_message_sorted_by_height(
    message: TomatoDetectionArray,
    transform=None,
) -> TomatoDetectionArray:
    """Copy a detection snapshot in descending world-height order."""
    centers = []
    for detection in message.detections:
        center = detection.center
        if transform is None:
            centers.append(
                (float(center.x), float(center.y), float(center.z))
            )
        else:
            centers.append(transformed_point_xyz(center, transform))
    order = clustered_height_order(
        centers,
        [detection.id for detection in message.detections],
    )
    sorted_message = TomatoDetectionArray()
    sorted_message.header = message.header
    sorted_message.detections = [
        message.detections[index] for index in order
    ]
    return sorted_message


def harvest_all_jobs(tomato_count: int) -> list[tuple[int, bool]]:
    """Build plan-only/execute jobs for every detected tomato in order."""
    if tomato_count < 0:
        raise ValueError("tomato_count must be zero or greater")
    return [
        (index, execute)
        for index in range(tomato_count)
        for execute in (False, True)
    ]


def batch_harvest_stage_limit(selection: str) -> int | None:
    """Return the per-tomato final stage selected for a whole batch."""
    value = str(selection).strip()
    if value == "전체 수확":
        return None
    limits = {"3단계까지": 3, "4단계까지": 4}
    if value not in limits:
        raise ValueError(f"지원하지 않는 수확 종료 단계입니다: {value}")
    return limits[value]


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


def detection_angle_segment(
    detection,
    angle_reference_mode: str,
    base_point=None,
):
    """Return source and target points for the selected approach axis."""
    mode = str(angle_reference_mode)
    if mode == ANGLE_REFERENCE_CENTER_TO_STEM:
        return detection.center, detection.stem_point
    if mode == ANGLE_REFERENCE_CALYX_TO_STEM:
        return detection.calyx_point, detection.stem_point
    if mode == ANGLE_REFERENCE_BASE_TO_CENTER:
        if base_point is None:
            raise ValueError(
                "로봇 베이스→토마토 중심 모드에는 베이스 원점이 필요합니다."
            )
        return base_point, detection.center
    raise ValueError(f"지원하지 않는 진입각 기준입니다: {mode}")


def recommend_entry_point(
    tomato_xyz,
    angle_origin_xyz,
    angle_target_xyz,
    length_m: float = 0.04,
) -> tuple[float, float, float]:
    """Return the same Recommend pre-grasp point used by the target plot."""
    tomato = tuple(float(value) for value in tomato_xyz)
    origin = tuple(float(value) for value in angle_origin_xyz)
    target = tuple(float(value) for value in angle_target_xyz)
    length = float(length_m)
    if any(len(values) != 3 for values in (tomato, origin, target)):
        raise ValueError("진입각 좌표는 X, Y, Z 3개 값이어야 합니다.")
    if not all(
        math.isfinite(value)
        for values in (tomato, origin, target)
        for value in values
    ):
        raise ValueError("진입각 좌표는 유한값이어야 합니다.")
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("Recommend 표시 길이는 0보다 커야 합니다.")
    outward = tuple(origin[axis] - target[axis] for axis in range(3))
    outward_norm = math.sqrt(sum(value * value for value in outward))
    if outward_norm <= 1e-9:
        outward = (1.0, 0.0, 0.0)
        outward_norm = 1.0
    return tuple(
        tomato[axis] + outward[axis] / outward_norm * length
        for axis in range(3)
    )


def selection_plot_recommend_screen_direction(
    tomato_xyz,
    angle_origin_xyz,
    angle_target_xyz,
    keep_robot_below_tomato: bool = True,
) -> tuple[float, float]:
    """Return the Recommend arrow direction exactly as drawn by PlotCanvas.

    The selected-tomato plot uses a Link0 X-Y top view, not the camera image
    plane.  Its canvas also performs a half turn for targets behind Link0 so
    that the robot remains below the tomato.  Camera overlays must apply the
    same two rules or their arrow can disagree with the selected-target plot.
    """
    tomato = tuple(float(value) for value in tomato_xyz)
    origin = tuple(float(value) for value in angle_origin_xyz)
    target = tuple(float(value) for value in angle_target_xyz)
    if any(len(values) != 3 for values in (tomato, origin, target)):
        raise ValueError("진입각 좌표는 X, Y, Z 3개 값이어야 합니다.")
    if not all(
        math.isfinite(value)
        for values in (tomato, origin, target)
        for value in values
    ):
        raise ValueError("진입각 좌표는 유한값이어야 합니다.")

    tomato_2d = robot_top_projection(tomato)
    origin_2d = robot_top_projection(origin)
    target_2d = robot_top_projection(target)
    plot_dx = target_2d[0] - origin_2d[0]
    plot_dy = target_2d[1] - origin_2d[1]
    if keep_robot_below_tomato and tomato_2d[1] < -1e-9:
        plot_dx = -plot_dx
        plot_dy = -plot_dy

    # Plot coordinates grow upward; image/Tk canvas coordinates grow down.
    screen_dx = plot_dx
    screen_dy = -plot_dy
    norm = math.hypot(screen_dx, screen_dy)
    if norm <= 1e-9:
        raise ValueError("진입 방향의 X-Y 길이가 0입니다.")
    return (screen_dx / norm, screen_dy / norm)


def detected_tomato_marker_array(
    detections: TomatoDetectionArray,
    diameter: float = 0.0175,
    stem_diameter: float = 0.006,
    approach_length: float = 0.06,
    angle_reference_mode: str = ANGLE_REFERENCE_CENTER_TO_STEM,
    base_point=None,
) -> MarkerArray:
    """Create tomato, stem-point, and approach-direction preview markers."""
    if not detections.header.frame_id:
        raise ValueError("검출 메시지의 header.frame_id가 비어 있습니다.")
    if not math.isfinite(diameter) or diameter <= 0.0:
        raise ValueError("검출 토마토 마커 지름은 0보다 커야 합니다.")
    if not math.isfinite(stem_diameter) or stem_diameter <= 0.0:
        raise ValueError("검출 줄기점 마커 지름은 0보다 커야 합니다.")
    if not math.isfinite(approach_length) or approach_length <= 0.0:
        raise ValueError("검출 진입 방향 마커 길이는 0보다 커야 합니다.")

    clear = Marker()
    clear.action = Marker.DELETEALL
    markers = [clear]
    for index, detection in enumerate(detections.detections):
        marker_id = index * 3
        center = Point(
            x=float(detection.center.x),
            y=float(detection.center.y),
            z=float(detection.center.z),
        )
        stem = Point(
            x=float(detection.stem_point.x),
            y=float(detection.stem_point.y),
            z=float(detection.stem_point.z),
        )
        angle_origin_value, angle_target_value = detection_angle_segment(
            detection,
            angle_reference_mode,
            base_point=base_point,
        )
        angle_origin = Point(
            x=float(angle_origin_value.x),
            y=float(angle_origin_value.y),
            z=float(angle_origin_value.z),
        )
        angle_target = Point(
            x=float(angle_target_value.x),
            y=float(angle_target_value.y),
            z=float(angle_target_value.z),
        )

        tomato_marker = Marker()
        tomato_marker.header.frame_id = detections.header.frame_id
        tomato_marker.ns = DETECTION_MARKER_NAMESPACE
        tomato_marker.id = marker_id
        tomato_marker.type = Marker.SPHERE
        tomato_marker.action = Marker.ADD
        tomato_marker.pose.position = center
        tomato_marker.pose.orientation.w = 1.0
        tomato_marker.scale.x = float(diameter)
        tomato_marker.scale.y = float(diameter)
        tomato_marker.scale.z = float(diameter)
        tomato_marker.color.r = 1.0
        tomato_marker.color.g = 0.15
        tomato_marker.color.b = 0.55
        tomato_marker.color.a = 0.85
        markers.append(tomato_marker)

        stem_marker = Marker()
        stem_marker.header.frame_id = detections.header.frame_id
        stem_marker.ns = DETECTION_MARKER_NAMESPACE
        stem_marker.id = marker_id + 1
        stem_marker.type = Marker.SPHERE
        stem_marker.action = Marker.ADD
        stem_marker.pose.position = stem
        stem_marker.pose.orientation.w = 1.0
        stem_marker.scale.x = float(stem_diameter)
        stem_marker.scale.y = float(stem_diameter)
        stem_marker.scale.z = float(stem_diameter)
        stem_marker.color.r = 0.2
        stem_marker.color.g = 1.0
        stem_marker.color.b = 0.2
        stem_marker.color.a = 0.95
        markers.append(stem_marker)

        axis_x = angle_target.x - angle_origin.x
        axis_y = angle_target.y - angle_origin.y
        axis_z = angle_target.z - angle_origin.z
        axis_norm = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)
        if axis_norm <= 1e-9:
            continue
        direction_scale = float(approach_length) / axis_norm
        approach_start = Point(
            x=center.x - axis_x * direction_scale,
            y=center.y - axis_y * direction_scale,
            z=center.z - axis_z * direction_scale,
        )
        approach_marker = Marker()
        approach_marker.header.frame_id = detections.header.frame_id
        approach_marker.ns = DETECTION_MARKER_NAMESPACE
        approach_marker.id = marker_id + 2
        approach_marker.type = Marker.ARROW
        approach_marker.action = Marker.ADD
        approach_marker.pose.orientation.w = 1.0
        approach_marker.points = [approach_start, center]
        approach_marker.scale.x = 0.004
        approach_marker.scale.y = 0.010
        approach_marker.scale.z = 0.014
        approach_marker.color.r = 0.1
        approach_marker.color.g = 0.8
        approach_marker.color.b = 1.0
        approach_marker.color.a = 0.9
        markers.append(approach_marker)

    message = MarkerArray()
    message.markers = markers
    return message


def harvest_result_marker(
    tomato_index: int,
    success: bool,
    arrow_length: float,
    adaptive_rotation_applied: bool = False,
    adaptive_rotation_deg: float = 0.0,
    approach_axis_local=None,
    tomato_frame: str | None = None,
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
    marker.header.frame_id = str(
        tomato_frame or f"detected_tomato_{tomato_index}_tf"
    )
    marker.ns = HARVEST_RESULT_NAMESPACE
    marker.id = tomato_index
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
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
    if success:
        marker.points = [Point(), endpoint]
    else:
        # A failed result must still read as an approach direction: place the
        # tail behind the tomato and point the arrow head at the tomato center.
        marker.points = [
            Point(x=-endpoint.x, y=-endpoint.y, z=-endpoint.z),
            Point(),
        ]
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


def actual_approach_marker(
    marker_id: int,
    report,
    namespace: str = HARVEST_APPROACH_NAMESPACE,
):
    """Create an orange arrow for the exact planned preapproach segment."""
    geometry = report.get("approach_geometry", {})
    frame_id = str(geometry.get("frame_id", ""))
    preapproach = geometry.get("preapproach_position", [])
    target = geometry.get("target_position", [])
    if not frame_id or len(preapproach) < 3 or len(target) < 3:
        return None
    values = [float(value) for value in (*preapproach[:3], *target[:3])]
    if not all(math.isfinite(value) for value in values):
        return None

    # The physical preapproach -> target segment is only 10 mm and ends at the
    # configured standoff, which leaves the arrow visually detached from the
    # tomato marker. Extend the same straight approach line to its nearest
    # horizontal point to the tomato origin. This preserves the actual
    # approach position/direction and only lengthens its visualization.
    direction_x = values[3] - values[0]
    direction_y = values[4] - values[1]
    direction_z = values[5] - values[2]
    direction_xy_squared = (
        direction_x * direction_x + direction_y * direction_y
    )
    extension = 1.0
    if direction_xy_squared > 1e-12:
        nearest = -(
            values[0] * direction_x + values[1] * direction_y
        ) / direction_xy_squared
        extension = max(1.0, min(10.0, nearest))
    display_endpoint = Point(
        x=values[0] + direction_x * extension,
        y=values[1] + direction_y * extension,
        z=values[2] + direction_z * extension,
    )

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.points = [
        Point(x=values[0], y=values[1], z=values[2]),
        display_endpoint,
    ]
    marker.scale.x = 0.003
    marker.scale.y = 0.008
    marker.scale.z = 0.004
    marker.color.r = 1.0
    marker.color.g = 0.45
    marker.color.b = 0.0
    marker.color.a = 1.0
    return marker


def actual_approach_marker_length(report) -> float | None:
    """Return the displayed orange approach-arrow length for this report."""
    marker = actual_approach_marker(0, report)
    if marker is None or len(marker.points) < 2:
        return None
    start, end = marker.points[:2]
    length = math.sqrt(
        (float(end.x) - float(start.x)) ** 2
        + (float(end.y) - float(start.y)) ** 2
        + (float(end.z) - float(start.z)) ** 2
    )
    return length if length > 1e-9 else None


def result_arrow_length_for_report(
    success: bool | None,
    default_length: float,
    report,
) -> float:
    """Use the orange approach length for a failed result arrow."""
    if success is False:
        failed_length = actual_approach_marker_length(report or {})
        if failed_length is not None:
            return failed_length
    return float(default_length)


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


def gripper_relay_power_script() -> str:
    """Keep RB DOUT0/DOUT8 HIGH and force every other DOUT LOW."""
    # Bit 0 + bit 8 = 1 + 256 = 257.
    return "set_dout_bit_combination(0,15,257,0)"


def gripper_relay_power_off_script() -> str:
    """Force RB DOUT0..15 LOW, including relay outputs 0 and 8."""
    return "set_dout_bit_combination(0,15,0,0)"


def linear_motor_pin_values(command: str) -> tuple[bool, bool]:
    """Return Arduino PIN8/PIN9 levels for one motor command."""
    output_levels = {
        "extend": (True, False),
        "retract": (False, True),
        "stop": (False, False),
    }
    try:
        return output_levels[command]
    except KeyError as error:
        message = f"지원하지 않는 그리퍼 스트로크 명령: {command}"
        raise ValueError(message) from error


def gripper_output_startup_scripts() -> tuple[str, ...]:
    """Return the RB relay-power initialization command."""
    return (gripper_relay_power_script(),)


def linear_motor_pin_sequence(
    command: str,
) -> tuple[tuple[bool, bool], ...]:
    """Return the break-before-make Arduino pin sequence."""
    target = linear_motor_pin_values(command)
    neutral = linear_motor_pin_values("stop")
    if command == "stop":
        return (neutral,)
    return (neutral, target)


def camera_service_for_source(
    source: str,
    fake_service: str,
    real_service: str,
) -> str:
    """Return the ROS service selected by the camera-source combobox."""
    services = {
        CAMERA_SOURCE_FAKE: str(fake_service),
        CAMERA_SOURCE_REAL: str(real_service),
    }
    try:
        return services[source]
    except KeyError as error:
        raise ValueError(f"지원하지 않는 카메라 검출 소스: {source}") from error


def decode_compressed_result_image(data) -> Image.Image:
    """Decode a ROS CompressedImage payload into an independent RGB image."""
    payload = bytes(data)
    if not payload:
        raise ValueError("압축 이미지 데이터가 비어 있습니다.")
    try:
        with Image.open(io.BytesIO(payload)) as source:
            source.load()
            return source.convert("RGB")
    except (UnidentifiedImageError, OSError) as error:
        raise ValueError("JPEG/PNG 압축 이미지를 해석할 수 없습니다.") from error


def decode_raw_result_image(message: RosImage) -> Image.Image:
    """Decode common 8-bit ROS Image encodings into an independent RGB image."""
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    encoding = str(message.encoding).strip().lower()
    formats = {
        "rgb8": ("RGB", "RGB", 3),
        "bgr8": ("RGB", "BGR", 3),
        "rgba8": ("RGBA", "RGBA", 4),
        "bgra8": ("RGBA", "BGRA", 4),
        "mono8": ("L", "L", 1),
    }
    if width <= 0 or height <= 0:
        raise ValueError("원본 이미지 크기가 올바르지 않습니다.")
    if encoding not in formats:
        raise ValueError(f"지원하지 않는 원본 이미지 인코딩: {encoding}")
    mode, raw_mode, channels = formats[encoding]
    minimum_step = width * channels
    if step < minimum_step:
        raise ValueError("원본 이미지 step이 픽셀 폭보다 작습니다.")
    payload = bytes(message.data)
    if len(payload) < step * height:
        raise ValueError("원본 이미지 데이터 길이가 부족합니다.")
    try:
        image = Image.frombytes(
            mode,
            (width, height),
            payload,
            "raw",
            raw_mode,
            step,
            1,
        )
    except (TypeError, ValueError) as error:
        raise ValueError("원본 이미지 픽셀을 해석할 수 없습니다.") from error
    return image.convert("RGB")


def camera_xyz_to_image_pixel(
    xyz,
    camera_matrix,
    camera_info_size,
    image_size,
) -> tuple[float, float]:
    """Project an optical-frame XYZ point into the displayed image."""
    x, y, z = (float(value) for value in xyz)
    matrix = tuple(float(value) for value in camera_matrix)
    info_width, info_height = (int(value) for value in camera_info_size)
    image_width, image_height = (int(value) for value in image_size)
    if len(matrix) != 9 or not all(math.isfinite(value) for value in matrix):
        raise ValueError("CameraInfo K 행렬이 올바르지 않습니다.")
    if not all(math.isfinite(value) for value in (x, y, z)) or z <= 0.0:
        raise ValueError("카메라 좌표 Z는 0보다 큰 유한값이어야 합니다.")
    if min(info_width, info_height, image_width, image_height) <= 0:
        raise ValueError("카메라 또는 이미지 해상도가 올바르지 않습니다.")
    fx, fy = matrix[0], matrix[4]
    cx, cy = matrix[2], matrix[5]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("CameraInfo 초점거리가 올바르지 않습니다.")
    source_u = fx * x / z + cx
    source_v = fy * y / z + cy
    return (
        source_u * image_width / info_width,
        source_v * image_height / info_height,
    )


def _label_font(image_height: int, size_scale: float = 1.0):
    base_size = max(16, min(28, int(round(image_height * 0.028))))
    font_size = max(10, int(round(base_size * float(size_scale))))
    font_paths = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf",
    )
    for path in font_paths:
        try:
            return ImageFont.truetype(path, font_size)
        except OSError:
            continue
    return ImageFont.load_default()


def _rectangle_overlap_area(first, second) -> float:
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    return max(0.0, right - left) * max(0.0, bottom - top)


def detection_label_layout(
    anchors,
    text_sizes,
    image_size,
    padding_scale: float = 1.0,
    preferred_sides=None,
):
    """Place compact labels around detections while avoiding tomatoes/labels."""
    width, height = (int(value) for value in image_size)
    if len(anchors) != len(text_sizes):
        raise ValueError("라벨 좌표와 텍스트 크기 개수가 다릅니다.")
    sides = (
        tuple(0 for _anchor in anchors)
        if preferred_sides is None
        else tuple(int(value) for value in preferred_sides)
    )
    if len(sides) != len(anchors):
        raise ValueError("라벨 좌우 배치 개수가 다릅니다.")
    if not anchors:
        return []
    center_x = sum(float(point[0]) for point in anchors) / len(anchors)
    center_y = sum(float(point[1]) for point in anchors) / len(anchors)
    margin = 6.0
    padding_x = 8.0 * float(padding_scale)
    padding_y = 5.0 * float(padding_scale)
    placed = []
    directions = tuple(
        (math.cos(math.radians(angle)), math.sin(math.radians(angle)))
        for angle in range(0, 360, 30)
    )
    for index, (anchor, text_size) in enumerate(zip(anchors, text_sizes)):
        anchor_x, anchor_y = (float(value) for value in anchor)
        text_width, text_height = (float(value) for value in text_size)
        box_width = text_width + padding_x * 2.0
        box_height = text_height + padding_y * 2.0
        outward_angle = math.atan2(anchor_y - center_y, anchor_x - center_x)
        if math.hypot(anchor_x - center_x, anchor_y - center_y) < 2.0:
            outward_angle = math.radians((index * 137.5) % 360.0)
        ordered_directions = sorted(
            directions,
            key=lambda direction: abs(
                math.atan2(
                    math.sin(math.atan2(direction[1], direction[0]) - outward_angle),
                    math.cos(math.atan2(direction[1], direction[0]) - outward_angle),
                )
            ),
        )
        best = None
        candidate_centers = []
        if sides[index] == 0:
            for radius in (52.0, 78.0, 108.0, 140.0):
                for direction_x, direction_y in ordered_directions:
                    candidate_centers.append(
                        (
                            anchor_x + direction_x * radius,
                            anchor_y + direction_y * radius,
                        )
                    )
        else:
            vertical_steps = (0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5)
            for column in range(5):
                horizontal_offset = (
                    box_width / 2.0
                    + 22.0
                    + column * (box_width + 8.0)
                )
                for vertical_step in vertical_steps:
                    candidate_centers.append(
                        (
                            anchor_x + sides[index] * horizontal_offset,
                            anchor_y
                            + vertical_step * (box_height + 6.0),
                        )
                    )

        seen_rectangles = set()
        for box_center_x, box_center_y in candidate_centers:
            left = min(
                max(margin, box_center_x - box_width / 2.0),
                max(margin, width - margin - box_width),
            )
            top = min(
                max(margin, box_center_y - box_height / 2.0),
                max(margin, height - margin - box_height),
            )
            rectangle = (left, top, left + box_width, top + box_height)
            rectangle_center_x = (rectangle[0] + rectangle[2]) / 2.0
            if (
                sides[index] < 0
                and rectangle_center_x >= anchor_x
            ) or (
                sides[index] > 0
                and rectangle_center_x <= anchor_x
            ):
                side_violation = 1
            else:
                side_violation = 0
            rectangle_key = tuple(round(value, 3) for value in rectangle)
            if rectangle_key in seen_rectangles:
                continue
            seen_rectangles.add(rectangle_key)
            spaced_rectangle = (
                rectangle[0] - 4.0,
                rectangle[1] - 4.0,
                rectangle[2] + 4.0,
                rectangle[3] + 4.0,
            )
            overlap_areas = [
                _rectangle_overlap_area(spaced_rectangle, existing)
                for existing in placed
            ]
            overlap_count = sum(area > 0.0 for area in overlap_areas)
            overlap_area = sum(overlap_areas)
            expanded = (
                rectangle[0] - 6.0,
                rectangle[1] - 6.0,
                rectangle[2] + 6.0,
                rectangle[3] + 6.0,
            )
            covered_tomatoes = 0
            for tomato_x, tomato_y in anchors:
                if (
                    expanded[0] <= tomato_x <= expanded[2]
                    and expanded[1] <= tomato_y <= expanded[3]
                ):
                    covered_tomatoes += 1
            placement_cost = (
                abs(rectangle_center_x - anchor_x)
                + 3.0
                * abs((rectangle[1] + rectangle[3]) / 2.0 - anchor_y)
            )
            candidate = (
                side_violation,
                overlap_count + covered_tomatoes,
                overlap_area,
                covered_tomatoes,
                placement_cost,
                rectangle,
            )
            if best is None or candidate[:-1] < best[:-1]:
                best = candidate
        rectangle = best[-1]
        placed.append(rectangle)
    return placed


def draw_detection_label_overlay(
    image: Image.Image,
    detections,
    camera_matrix,
    camera_info_size,
) -> tuple[Image.Image, int]:
    """Draw ID labels, leader lines, and Recommend entry arrows."""
    output = image.convert("RGB").copy()
    projected = []
    for detection in detections:
        if len(detection) not in (2, 3, 4):
            raise ValueError("오버레이 검출 항목 형식이 올바르지 않습니다.")
        label, xyz = detection[:2]
        recommend_xyz = detection[2] if len(detection) >= 3 else None
        recommend_direction = detection[3] if len(detection) == 4 else None
        try:
            pixel = camera_xyz_to_image_pixel(
                xyz,
                camera_matrix,
                camera_info_size,
                output.size,
            )
        except ValueError:
            continue
        if (
            -10.0 <= pixel[0] <= output.width + 10.0
            and -10.0 <= pixel[1] <= output.height + 10.0
        ):
            recommend_pixel = None
            if recommend_xyz is not None:
                try:
                    recommend_pixel = camera_xyz_to_image_pixel(
                        recommend_xyz,
                        camera_matrix,
                        camera_info_size,
                        output.size,
                    )
                except ValueError:
                    recommend_pixel = None
            if recommend_direction is not None:
                direction_x = float(recommend_direction[0])
                direction_y = float(recommend_direction[1])
                direction_norm = math.hypot(direction_x, direction_y)
                recommend_direction = (
                    (direction_x / direction_norm, direction_y / direction_norm)
                    if direction_norm > 1e-9
                    else None
                )
            if recommend_direction is None and recommend_pixel is not None:
                direction_x = pixel[0] - recommend_pixel[0]
                direction_y = pixel[1] - recommend_pixel[1]
                direction_norm = math.hypot(direction_x, direction_y)
                if direction_norm > 1e-9:
                    recommend_direction = (
                        direction_x / direction_norm,
                        direction_y / direction_norm,
                    )
            projected.append(
                (str(label), pixel, recommend_pixel, recommend_direction)
            )
    if not projected:
        return output, 0

    card_scale = 0.80
    layout_padding_scale = 0.35
    font = _label_font(output.height, card_scale)
    measure = ImageDraw.Draw(output)
    scale = max(1.0, output.height / 720.0)
    arrow_visual_size = max(26.0, 32.0 * scale) * card_scale
    arrow_text_gap = max(2.0, 3.0 * scale) * card_scale
    text_sizes = []
    layout_sizes = []
    for label, _pixel, _recommend_pixel, recommend_direction in projected:
        bounds = measure.textbbox((0, 0), label, font=font)
        text_size = (bounds[2] - bounds[0], bounds[3] - bounds[1])
        text_sizes.append(text_size)
        layout_sizes.append(
            (
                text_size[0] + arrow_visual_size + arrow_text_gap
                if recommend_direction is not None
                else text_size[0],
                max(text_size[1], arrow_visual_size)
                if recommend_direction is not None
                else text_size[1],
            )
        )
    anchors = [
        pixel for _label, pixel, _recommend_pixel, _direction in projected
    ]
    preferred_sides = []
    for _label, _pixel, _recommend_pixel, recommend_direction in projected:
        if recommend_direction is None:
            preferred_sides.append(0)
            continue
        # Arrow points left→right: label belongs left of the tomato.
        # Arrow points right→left: label belongs right of the tomato.
        preferred_sides.append(
            -1 if float(recommend_direction[0]) >= 0.0 else 1
        )
    rectangles = detection_label_layout(
        anchors,
        layout_sizes,
        output.size,
        padding_scale=layout_padding_scale,
        preferred_sides=preferred_sides,
    )

    layer = Image.new("RGBA", output.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    line_width = max(2, int(round(3 * scale)))
    connection_line_width = max(3, int(round(4 * scale)))
    color = (238, 101, 43, 255)
    recommend_color = (15, 108, 148, 255)
    label_outline_color = color
    for (
        label,
        (anchor_x, anchor_y),
        recommend_pixel,
        recommend_direction,
    ), rectangle, text_size in zip(
        projected,
        rectangles,
        text_sizes,
    ):
        left, top, right, bottom = rectangle
        line_start = (
            min(max(anchor_x, left), right),
            min(max(anchor_y, top), bottom),
        )
        vector_x = anchor_x - line_start[0]
        vector_y = anchor_y - line_start[1]
        vector_length = math.hypot(vector_x, vector_y)
        if vector_length > 1e-6:
            connection_x = vector_x / vector_length
            connection_y = vector_y / vector_length
            connection_head_length = max(7.0, 9.0 * scale)
            connection_head_half_width = max(4.0, 5.0 * scale)
            connection_head_base = (
                anchor_x - connection_x * connection_head_length,
                anchor_y - connection_y * connection_head_length,
            )
            connection_perpendicular = (-connection_y, connection_x)
            draw.line(
                (line_start, connection_head_base),
                fill=color,
                width=connection_line_width,
            )
            draw.polygon(
                (
                    (anchor_x, anchor_y),
                    (
                        connection_head_base[0]
                        + connection_perpendicular[0]
                        * connection_head_half_width,
                        connection_head_base[1]
                        + connection_perpendicular[1]
                        * connection_head_half_width,
                    ),
                    (
                        connection_head_base[0]
                        - connection_perpendicular[0]
                        * connection_head_half_width,
                        connection_head_base[1]
                        - connection_perpendicular[1]
                        * connection_head_half_width,
                    ),
                ),
                fill=color,
            )
        draw.rounded_rectangle(
            rectangle,
            radius=max(4, int(round(10 * scale * card_scale))),
            fill=(255, 255, 255, 178),
            outline=label_outline_color,
            width=line_width,
        )
        content_padding = max(2.5, 3.5 * scale) * card_scale
        text_y = top + (bottom - top - text_size[1]) / 2.0 - 1.0
        if recommend_direction is None:
            text_x = left + (right - left - text_size[0]) / 2.0
            draw.text(
                (text_x, text_y),
                label,
                font=font,
                fill=(0, 0, 0, 255),
            )
            continue
        direction_x = float(recommend_direction[0])
        direction_y = float(recommend_direction[1])
        direction_norm = math.hypot(direction_x, direction_y)
        if direction_norm <= 1e-6:
            continue
        direction_x /= direction_norm
        direction_y /= direction_norm
        arrow_length = min(
            arrow_visual_size,
            bottom - top - content_padding * 2.0,
        )
        half_length = arrow_length / 2.0
        if direction_x >= 0.0:
            text_x = left + content_padding
            midpoint_x = right - content_padding - half_length
        else:
            midpoint_x = left + content_padding + half_length
            text_x = right - content_padding - text_size[0]
        midpoint_y = (top + bottom) / 2.0
        draw.text(
            (text_x, text_y),
            label,
            font=font,
            fill=(0, 0, 0, 255),
        )
        arrow_start = (
            midpoint_x - direction_x * half_length,
            midpoint_y - direction_y * half_length,
        )
        arrow_end = (
            midpoint_x + direction_x * half_length,
            midpoint_y + direction_y * half_length,
        )
        arrow_head_length = max(10.0, 13.0 * scale) * card_scale
        arrow_head_half_width = max(6.0, 7.0 * scale) * card_scale
        head_base_x = arrow_end[0] - direction_x * arrow_head_length
        head_base_y = arrow_end[1] - direction_y * arrow_head_length
        perpendicular_x = -direction_y
        perpendicular_y = direction_x
        draw.line(
            (arrow_start, (head_base_x, head_base_y)),
            fill=recommend_color,
            width=max(3, int(round(4 * scale * card_scale))),
        )
        draw.polygon(
            (
                arrow_end,
                (
                    head_base_x
                    + perpendicular_x * arrow_head_half_width,
                    head_base_y
                    + perpendicular_y * arrow_head_half_width,
                ),
                (
                    head_base_x
                    - perpendicular_x * arrow_head_half_width,
                    head_base_y
                    - perpendicular_y * arrow_head_half_width,
                ),
            ),
            fill=recommend_color,
        )
    return Image.alpha_composite(output.convert("RGBA"), layer).convert("RGB"), len(projected)


def camera_target_record_text(
    *,
    target_frame: str,
    camera_frame: str,
    camera_id: str,
    tomato_xyz,
    vine_xyz,
    calyx_xyz=None,
    angle_reference: str = "center_to_stem",
    detection_timestamp: float,
    plan_report: dict | None = None,
    robot_frame_id: str | None = None,
    robot_tomato_xyz=None,
    robot_vine_xyz=None,
    robot_calyx_xyz=None,
    robot_angle_origin_xyz=None,
    robot_angle_target_xyz=None,
    review_issue: str = "문제 없음",
    review_note: str = "",
) -> str:
    """Format one active target as valid JSON for a future vision service."""

    def xyz(values, label: str) -> list[float]:
        coordinates = tuple(float(value) for value in values)
        if len(coordinates) != 3 or not all(
            math.isfinite(value) for value in coordinates
        ):
            raise ValueError(f"{label}는 유한한 X, Y, Z 값이어야 합니다.")
        return [round(value, 9) for value in coordinates]

    def optional_float(value):
        if value is None:
            return None
        number = float(value)
        return number if math.isfinite(number) else None

    tomato = xyz(tomato_xyz, "토마토 중심 좌표")
    vine = xyz(vine_xyz, "줄기 좌표")
    calyx = (
        xyz(calyx_xyz, "꼭지 좌표") if calyx_xyz is not None else None
    )
    angle_reference = str(angle_reference).strip()
    if angle_reference not in ANGLE_REFERENCE_MODES:
        raise ValueError(f"지원하지 않는 진입각 기준입니다: {angle_reference}")
    if angle_reference == "calyx_to_stem" and calyx is None:
        raise ValueError("꼭지→줄기 진입각에는 calyx_point 좌표가 필요합니다.")
    tomato_vine_distance = math.dist(tomato, vine)
    report = dict(plan_report or {})
    adaptive = dict(report.get("adaptive_grasp") or {})
    geometry = dict(report.get("approach_geometry") or {})
    exact_robot_coordinates = (
        robot_tomato_xyz is not None and robot_vine_xyz is not None
    )
    if exact_robot_coordinates:
        robot_tomato = xyz(robot_tomato_xyz, "robot.tomato_xyz")
        robot_vine = xyz(robot_vine_xyz, "robot.vine_xyz")
        robot_calyx = (
            xyz(robot_calyx_xyz, "robot.calyx_xyz")
            if robot_calyx_xyz is not None
            else None
        )
        robot_angle_origin = (
            xyz(robot_angle_origin_xyz, "robot.angle_origin_xyz")
            if robot_angle_origin_xyz is not None
            else None
        )
        robot_angle_target = (
            xyz(robot_angle_target_xyz, "robot.angle_target_xyz")
            if robot_angle_target_xyz is not None
            else None
        )
    else:
        robot_tomato = geometry.get("tomato_xyz")
        robot_vine = geometry.get("vine_xyz")
        robot_calyx = None
        robot_angle_origin = None
        robot_angle_target = None
    if (
        not exact_robot_coordinates
        and robot_tomato is not None
        and robot_vine is not None
    ):
        robot_tomato = xyz(robot_tomato, "robot.tomato_xyz")
        virtual_vine = xyz(robot_vine, "robot.vine_xyz")
        direction = [
            virtual_vine[axis] - robot_tomato[axis]
            for axis in range(3)
        ]
        direction_length = math.sqrt(sum(value * value for value in direction))
        if direction_length <= 1e-9:
            robot_vine = None
        else:
            robot_vine = [
                round(
                    robot_tomato[axis]
                    + direction[axis]
                    / direction_length
                    * tomato_vine_distance,
                    9,
                )
                for axis in range(3)
            ]
    recommend_angle = optional_float(
        geometry.get(
            "recommend_rotation_deg",
            adaptive.get("geometric_preferred_rotation_deg"),
        )
    )
    final_angle = optional_float(
        geometry.get(
            "final_rotation_deg",
            adaptive.get("applied_rotation_deg"),
        )
    )
    correction_angle = (
        round(final_angle - recommend_angle, 6)
        if final_angle is not None and recommend_angle is not None
        else None
    )
    selected_issue = str(review_issue).strip() or "문제 없음"
    issue_definition = VISION_REVIEW_ISSUES.get(selected_issue)
    if issue_definition is None:
        issue_definition = "CUSTOM"
    no_issue = issue_definition == "NO_ISSUE"
    note = str(review_note).strip()
    issue = {"code": issue_definition, "label": selected_issue}
    if no_issue and note:
        issue.update(
            {
                "code": "GENERAL_NOTE",
                "label": "비고 전달",
            }
        )

    payload = {
        "target_id": str(target_frame).removesuffix("_tf"),
        "timestamp": float(detection_timestamp),
        "camera_id": str(camera_id),
        "vision": {
            "frame_id": str(camera_frame),
            "tomato_xyz": tomato,
            "calyx_xyz": calyx,
            "vine_xyz": vine,
            "angle_reference": angle_reference,
            "tomato_vine_distance_m": round(tomato_vine_distance, 9),
        },
        "robot": {
            "plan_success": (
                bool(report.get("success")) if report else None
            ),
            "frame_id": (
                str(robot_frame_id)
                if exact_robot_coordinates and robot_frame_id
                else geometry.get("planning_frame_id")
            ),
            "coordinate_source": (
                "detection_tf_snapshot"
                if exact_robot_coordinates
                else "legacy_planning_geometry"
            ),
            "reference_link": geometry.get(
                "pregrasp_reference_link"
            ),
            "tomato_xyz": robot_tomato,
            "calyx_xyz": robot_calyx,
            "vine_xyz": robot_vine,
            "angle_reference": angle_reference,
            "angle_origin_xyz": robot_angle_origin,
            "angle_target_xyz": robot_angle_target,
            "recommend_pregrasp_xyz": geometry.get(
                "recommend_pregrasp_xyz"
            ),
            "recommend_angle_deg": recommend_angle,
            "final_pregrasp_xyz": geometry.get("final_pregrasp_xyz"),
            "final_angle_deg": final_angle,
            "correction_angle_deg": correction_angle,
        },
        "review": {
            "status": "NO_ISSUE" if no_issue and not note else "REVIEW_REQUIRED",
            "issue": None if no_issue and not note else issue,
            "note": note,
        },
    }
    return json.dumps(
        payload,
        ensure_ascii=False,
        indent=2,
        allow_nan=False,
    ) + "\n"


def tomato_selection_plot_scene(
    tomato_xyz,
    vine_xyz,
    calyx_xyz=None,
    angle_origin_xyz=None,
    angle_target_xyz=None,
    plan_report: dict | None = None,
    recommend_length_m: float = 0.04,
) -> dict:
    """Build an embedded side-view scene for one selected detection."""

    def xyz(values, label: str) -> tuple[float, float, float]:
        coordinates = tuple(float(value) for value in values)
        if len(coordinates) != 3 or not all(
            math.isfinite(value) for value in coordinates
        ):
            raise ValueError(f"{label}는 유한한 X, Y, Z 값이어야 합니다.")
        return coordinates

    tomato = xyz(tomato_xyz, "토마토 중심 좌표")
    vine = xyz(vine_xyz, "줄기점 좌표")
    calyx = (
        None if calyx_xyz is None else xyz(calyx_xyz, "꼭지 좌표")
    )
    angle_origin = (
        tomato
        if angle_origin_xyz is None
        else xyz(angle_origin_xyz, "진입각 시작점 좌표")
    )
    angle_target = (
        vine
        if angle_target_xyz is None
        else xyz(angle_target_xyz, "진입각 목표점 좌표")
    )
    length = float(recommend_length_m)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("Recommend 표시 길이는 0보다 커야 합니다.")
    recommend = recommend_entry_point(
        tomato,
        angle_origin,
        angle_target,
        length,
    )

    report = dict(plan_report or {})
    geometry = dict(report.get("approach_geometry") or {})
    local_preapproach_values = geometry.get("preapproach_position")
    if local_preapproach_values is not None:
        local_preapproach = xyz(
            local_preapproach_values,
            "토마토 TF 기준 최종 pre-grasp 좌표",
        )
        direction_x = angle_target[0] - angle_origin[0]
        direction_y = angle_target[1] - angle_origin[1]
        direction_norm = math.hypot(direction_x, direction_y)
        if direction_norm <= 1e-9:
            raise ValueError("진입각 기준의 수평 방향이 너무 짧습니다.")
        tomato_x = (
            direction_x / direction_norm,
            direction_y / direction_norm,
        )
        tomato_y = (-tomato_x[1], tomato_x[0])
        # RViz의 노란 actual-approach marker도 동일한 tomato-frame
        # preapproach_position을 사용한다. 절대 final_pregrasp_xyz를 현재
        # 검출 중심과 섞지 않고 같은 로컬 벡터를 현재 TF 축으로 변환해야
        # 재검출 이후에도 두 화면의 방향이 일치한다.
        final = (
            tomato[0]
            + tomato_x[0] * local_preapproach[0]
            + tomato_y[0] * local_preapproach[1],
            tomato[1]
            + tomato_x[1] * local_preapproach[0]
            + tomato_y[1] * local_preapproach[1],
            tomato[2] + local_preapproach[2],
        )
    else:
        final_values = geometry.get("final_pregrasp_xyz")
        final = (
            xyz(final_values, "최종 pre-grasp 좌표")
            if final_values is not None
            else None
        )
    return {
        "robot_tomato": tomato,
        "robot_vine": vine,
        "robot_calyx": calyx,
        "robot_angle_origin": angle_origin,
        "robot_angle_target": angle_target,
        "recommend": recommend,
        "final": final,
    }


def debug_frame_request(json_text: str) -> DebugFrame.Request:
    """Wrap one valid feedback JSON object in a DebugFrame request."""
    text = str(json_text)
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as error:
        raise ValueError(f"비전 피드백 JSON 형식이 올바르지 않습니다: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError("비전 피드백 JSON 최상위 값은 object여야 합니다.")
    request = DebugFrame.Request()
    request.request_text = text
    return request


class HarvestGui(Node):
    """Tkinter operator panel for tomato detection and harvest testing."""

    def __init__(self) -> None:
        super().__init__("tomato_harvest_gui")
        self.declare_parameter(
            "camera_service", "/fake_tomato_camera/detect_tomatoes"
        )
        self.declare_parameter("real_camera_service", "/detect_tomatoes")
        self.declare_parameter("capture_camera_service", "/capture_camera")
        self.declare_parameter("debug_frame_service", "/debug_frame")
        self.declare_parameter(
            "result_image_topic",
            "/tomato_vision/result_image",
        )
        self.declare_parameter(
            "vision_result_image_topic",
            "/tomato_vision/result_image_raw",
        )
        self.declare_parameter(
            "camera_color_image_topic",
            "/tomato_vision/camera_preview",
        )
        self.declare_parameter(
            "camera_color_info_topic",
            "/camera/d435/color/camera_info",
        )
        self.declare_parameter("default_camera_source", "real")
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
            "hardware_eval_service", "/rbpodo_hardware/eval"
        )
        self.declare_parameter(
            "linear_motor_node_name", "/pin89_serial_node"
        )
        self.declare_parameter(
            "linear_motor_launch_package", "arduino_linear_motor"
        )
        self.declare_parameter(
            "linear_motor_launch_file", "pin89_serial.launch.py"
        )
        self.declare_parameter("linear_motor_serial_port", "/dev/ttyUSB0")
        self.declare_parameter(
            "linear_motor_pin8_topic", "/linear_motor/pin8"
        )
        self.declare_parameter(
            "linear_motor_pin9_topic", "/linear_motor/pin9"
        )
        self.declare_parameter(
            "linear_motor_servo10_angle_topic",
            "/linear_motor/servo10_angle_deg",
        )
        self.declare_parameter(
            "linear_motor_servo10_command_topic",
            "/linear_motor/servo10_command",
        )
        self.declare_parameter(
            "linear_motor_serial_status_topic",
            "/linear_motor/serial_status",
        )
        self.declare_parameter(
            "moveit_cancel_service", "/execute_trajectory/_action/cancel_goal"
        )
        self.declare_parameter(
            "controller_cancel_service",
            "/joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal",
        )
        self.declare_parameter(
            "result_markers_topic", "/harvest_result_markers"
        )
        self.declare_parameter(
            "detection_markers_topic", "/detected_tomato_markers"
        )
        self.declare_parameter("detection_marker_diameter", 0.0175)
        self.declare_parameter("detection_stem_marker_diameter", 0.006)
        self.declare_parameter("detection_approach_marker_length", 0.06)
        self.declare_parameter("result_marker_parent_frame", "world")
        self.declare_parameter("result_arrow_stem_margin", 0.008)
        self.declare_parameter("result_arrow_minimum_length", 0.015)
        self.declare_parameter(
            "results_directory",
            str(Path.home() / "farmily_tomato" / "harvest_results"),
        )
        self.declare_parameter(
            "camera_target_records_directory",
            str(
                Path.home()
                / "farmily_tomato"
                / "camera_target_records"
            ),
        )
        self.declare_parameter("camera_target_record_robot_frame", "link0")
        self.declare_parameter("lift_node_name", "/lift_controller_node")
        self.declare_parameter(
            "lift_bottom_calibration_topic",
            "/lift_control/find_bottom_limit",
        )
        self.declare_parameter(
            "lift_move_height_topic",
            "/lift_control/move_height",
        )
        self.declare_parameter("lift_stop_topic", "/lift_control/stop")
        self.declare_parameter(
            "lift_simulated_move_height_topic",
            "/lift_simulation/control/move_height",
        )
        self.declare_parameter(
            "lift_simulated_stop_topic",
            "/lift_simulation/control/stop",
        )
        self.declare_parameter(
            "lift_simulated_current_height_topic",
            "/lift_simulation/status/current_height",
        )
        self.declare_parameter(
            "lift_simulation_mode_topic",
            "/lift_simulation/status/enabled",
        )
        self.declare_parameter(
            "lift_current_height_topic",
            "/lift_status/current_height",
        )
        self.declare_parameter(
            "lift_bottom_status_topic",
            "/lift_status/bottom_limit_found",
        )
        self.declare_parameter("lift_launch_package", "farmily_uv_lift")
        self.declare_parameter(
            "lift_launch_file",
            "farmily_lift_controller_launch.py",
        )
        self.declare_parameter("lift_harvest_world_frame", "world")
        self.declare_parameter("lift_harvest_offset_m", 0.40)
        self.declare_parameter("lift_minimum_height_mm", 0.0)
        self.declare_parameter("lift_maximum_height_mm", 750.0)
        self.declare_parameter("lift_target_tolerance_mm", 1.0)
        self.declare_parameter("lift_move_timeout_sec", 60.0)
        self.declare_parameter("detected_tf_sync_tolerance_m", 0.003)
        self.declare_parameter(
            "detected_tf_sync_orientation_tolerance_deg",
            2.0,
        )
        self.declare_parameter("detected_tf_sync_timeout_sec", 8.0)
        self.declare_parameter(
            "detected_tf_ready_topic",
            "/tomato_tf_generator/status/ready",
        )
        self.declare_parameter(
            "tf_angle_reference_service",
            "/tomato_tf_generator/set_parameters",
        )
        self.declare_parameter("angle_reference_robot_base_frame", "link0")

        camera_service = str(self.get_parameter("camera_service").value)
        real_camera_service = str(
            self.get_parameter("real_camera_service").value
        )
        detections_topic = str(self.get_parameter("detections_topic").value)
        scene_node = str(self.get_parameter("scene_node").value).rstrip("/")
        self.fake_camera_service = camera_service
        self.real_camera_service = real_camera_service
        self.camera_clients = {
            CAMERA_SOURCE_FAKE: self.create_client(
                DetectTomatoes,
                self.fake_camera_service,
            ),
            CAMERA_SOURCE_REAL: self.create_client(
                DetectTomatoes,
                self.real_camera_service,
            ),
        }
        default_camera_source = str(
            self.get_parameter("default_camera_source").value
        ).strip().lower()
        initial_camera_source = (
            CAMERA_SOURCE_REAL
            if default_camera_source == "real"
            else CAMERA_SOURCE_FAKE
        )
        self.camera_client = self.camera_clients[initial_camera_source]
        self.camera_service = camera_service_for_source(
            initial_camera_source,
            self.fake_camera_service,
            self.real_camera_service,
        )
        self.capture_camera_service = str(
            self.get_parameter("capture_camera_service").value
        )
        self.capture_camera_client = self.create_client(
            Trigger,
            self.capture_camera_service,
        )
        self.tf_angle_reference_service = str(
            self.get_parameter("tf_angle_reference_service").value
        )
        self.tf_angle_reference_client = self.create_client(
            SetParameters,
            self.tf_angle_reference_service,
        )
        self.debug_frame_service = str(
            self.get_parameter("debug_frame_service").value
        )
        self.debug_frame_client = self.create_client(
            DebugFrame,
            self.debug_frame_service,
        )
        self.result_image_topic = str(
            self.get_parameter("result_image_topic").value
        )
        self.vision_result_image_topic = str(
            self.get_parameter("vision_result_image_topic").value
        )
        self.camera_color_image_topic = str(
            self.get_parameter("camera_color_image_topic").value
        )
        self.camera_color_info_topic = str(
            self.get_parameter("camera_color_info_topic").value
        )
        reliable_image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        camera_info_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.camera_color_image_subscription = self.create_subscription(
            RosImage,
            self.camera_color_image_topic,
            self._camera_color_image_callback,
            reliable_image_qos,
        )
        self.vision_result_image_subscription = self.create_subscription(
            RosImage,
            self.vision_result_image_topic,
            self._vision_result_image_callback,
            reliable_image_qos,
        )
        self.camera_color_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_color_info_topic,
            self._camera_color_info_callback,
            camera_info_qos,
        )
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
        self.hardware_eval_service = str(
            self.get_parameter("hardware_eval_service").value
        )
        self.hardware_eval_client = self.create_client(
            Eval, self.hardware_eval_service
        )
        self.linear_motor_node_name = str(
            self.get_parameter("linear_motor_node_name").value
        )
        self.linear_motor_launch_package = str(
            self.get_parameter("linear_motor_launch_package").value
        )
        self.linear_motor_launch_file = str(
            self.get_parameter("linear_motor_launch_file").value
        )
        self.linear_motor_serial_port = str(
            self.get_parameter("linear_motor_serial_port").value
        )
        self.linear_motor_pin8_topic = str(
            self.get_parameter("linear_motor_pin8_topic").value
        )
        self.linear_motor_pin9_topic = str(
            self.get_parameter("linear_motor_pin9_topic").value
        )
        self.linear_motor_servo10_angle_topic = str(
            self.get_parameter("linear_motor_servo10_angle_topic").value
        )
        self.linear_motor_servo10_command_topic = str(
            self.get_parameter("linear_motor_servo10_command_topic").value
        )
        self.linear_motor_serial_status_topic = str(
            self.get_parameter("linear_motor_serial_status_topic").value
        )
        self.linear_motor_pin8_publisher = self.create_publisher(
            Bool,
            self.linear_motor_pin8_topic,
            10,
        )
        self.linear_motor_pin9_publisher = self.create_publisher(
            Bool,
            self.linear_motor_pin9_topic,
            10,
        )
        self.linear_motor_servo10_angle_publisher = self.create_publisher(
            Float64,
            self.linear_motor_servo10_angle_topic,
            10,
        )
        self.linear_motor_servo10_command_publisher = self.create_publisher(
            Float64MultiArray,
            self.linear_motor_servo10_command_topic,
            10,
        )
        linear_motor_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.linear_motor_serial_status_subscription = (
            self.create_subscription(
                String,
                self.linear_motor_serial_status_topic,
                self._linear_motor_serial_status_callback,
                linear_motor_status_qos,
            )
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
        # Some camera implementations expose detections only in the service
        # response. Republish real-camera responses on the shared detection
        # topic so the TF generator and every other consumer receive the same
        # snapshot as the GUI.
        self.detections_publisher = self.create_publisher(
            TomatoDetectionArray,
            detections_topic,
            10,
        )
        self.detected_tf_ready_topic = str(
            self.get_parameter("detected_tf_ready_topic").value
        )
        detected_tf_ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Header,
            self.detected_tf_ready_topic,
            self._detected_tf_ready_callback,
            detected_tf_ready_qos,
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
        self.detection_marker_diameter = float(
            self.get_parameter("detection_marker_diameter").value
        )
        self.detection_stem_marker_diameter = float(
            self.get_parameter("detection_stem_marker_diameter").value
        )
        self.detection_approach_marker_length = float(
            self.get_parameter("detection_approach_marker_length").value
        )
        self.detection_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("detection_markers_topic").value),
            result_marker_qos,
        )
        self.rviz_goal_state_publisher = self.create_publisher(
            RobotState,
            "/rviz/moveit/update_custom_goal_state",
            10,
        )
        self.lift_node_name = str(
            self.get_parameter("lift_node_name").value
        )
        self.lift_bottom_calibration_topic = str(
            self.get_parameter("lift_bottom_calibration_topic").value
        )
        self.lift_move_height_topic = str(
            self.get_parameter("lift_move_height_topic").value
        )
        self.lift_stop_topic = str(
            self.get_parameter("lift_stop_topic").value
        )
        self.lift_simulated_move_height_topic = str(
            self.get_parameter("lift_simulated_move_height_topic").value
        )
        self.lift_simulated_stop_topic = str(
            self.get_parameter("lift_simulated_stop_topic").value
        )
        self.lift_simulated_current_height_topic = str(
            self.get_parameter("lift_simulated_current_height_topic").value
        )
        self.lift_simulation_mode_topic = str(
            self.get_parameter("lift_simulation_mode_topic").value
        )
        self.lift_current_height_topic = str(
            self.get_parameter("lift_current_height_topic").value
        )
        self.lift_bottom_status_topic = str(
            self.get_parameter("lift_bottom_status_topic").value
        )
        self.lift_launch_package = str(
            self.get_parameter("lift_launch_package").value
        )
        self.lift_launch_file = str(
            self.get_parameter("lift_launch_file").value
        )
        self.lift_harvest_world_frame = str(
            self.get_parameter("lift_harvest_world_frame").value
        )
        self.lift_harvest_offset_m = float(
            self.get_parameter("lift_harvest_offset_m").value
        )
        self.lift_minimum_height_mm = float(
            self.get_parameter("lift_minimum_height_mm").value
        )
        self.lift_maximum_height_mm = float(
            self.get_parameter("lift_maximum_height_mm").value
        )
        self.lift_target_tolerance_mm = float(
            self.get_parameter("lift_target_tolerance_mm").value
        )
        self.lift_move_timeout_sec = float(
            self.get_parameter("lift_move_timeout_sec").value
        )
        self.detected_tf_sync_tolerance_m = float(
            self.get_parameter("detected_tf_sync_tolerance_m").value
        )
        self.detected_tf_sync_orientation_tolerance_deg = float(
            self.get_parameter(
                "detected_tf_sync_orientation_tolerance_deg"
            ).value
        )
        self.detected_tf_sync_timeout_sec = float(
            self.get_parameter("detected_tf_sync_timeout_sec").value
        )
        self.lift_bottom_calibration_publisher = self.create_publisher(
            Bool,
            self.lift_bottom_calibration_topic,
            10,
        )
        self.lift_move_height_publisher = self.create_publisher(
            Float64,
            self.lift_move_height_topic,
            10,
        )
        self.lift_stop_publisher = self.create_publisher(
            Bool,
            self.lift_stop_topic,
            10,
        )
        self.lift_simulated_move_height_publisher = self.create_publisher(
            Float64,
            self.lift_simulated_move_height_topic,
            10,
        )
        self.lift_simulated_stop_publisher = self.create_publisher(
            Bool,
            self.lift_simulated_stop_topic,
            10,
        )
        self.lift_current_height_subscription = self.create_subscription(
            Float64,
            self.lift_current_height_topic,
            self._lift_current_height_callback,
            10,
        )
        self.lift_bottom_status_subscription = self.create_subscription(
            Bool,
            self.lift_bottom_status_topic,
            self._lift_bottom_status_callback,
            10,
        )
        self.lift_simulated_height_subscription = self.create_subscription(
            Float64,
            self.lift_simulated_current_height_topic,
            self._lift_simulated_height_callback,
            10,
        )
        self.lift_simulation_mode_subscription = self.create_subscription(
            Bool,
            self.lift_simulation_mode_topic,
            self._lift_simulation_mode_callback,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scene_node = scene_node
        self.detected_tomatoes = []
        self.latest_detection_message = None
        self.active_camera_target_index = None
        self.active_camera_target_context = ""
        self.active_camera_target_plan_report = {}
        self.detected_tomato_expected_world_positions = {}
        self.detected_tomato_expected_world_x_axes = {}
        self.detected_tomato_record_positions = {}
        self.detected_tomato_record_stem_positions = {}
        self.detected_tomato_record_calyx_positions = {}
        self.detected_tomato_record_angle_origins = {}
        self.detected_tomato_record_angle_targets = {}
        self.current_detection_stamp_ns = 0
        self.detected_tf_ready_stamp_ns = 0
        self.detection_signature = None
        self.detection_generation = 0
        self.verified_plan = None
        self.harvest_process = None
        self.step_process = None
        self.step_stages: list[dict] = []
        self.step_next_index = 0
        self.step_execution_in_progress = False
        self.step_execution_confirmed = False
        self.step_session_failed = False
        self.step_session_verification = None
        self.step_session_mode = None
        self.repeat_cycle_last_index = 4
        self.repeat_cycle_forward_distance_m = 0.040
        self.repeat_batch_active = False
        self.repeat_batch_queue = deque()
        self.repeat_batch_total = 0
        self.repeat_batch_completed = 0
        self.repeat_batch_plan_failures = 0
        self.repeat_batch_current_index = None
        self.repeat_batch_current_outcome = None
        self.repeat_run_direction = None
        self.repeat_pause_requested = False
        self.repeat_paused = False
        self.harvest_plan_report = {}
        self.named_pose_report = {}
        self.named_pose_active_state = None
        self.last_completed_named_pose = None
        self.pending_detection_capture_pose = None
        self.detection_capture_pose = None
        self.last_failure_robot_state = None
        self.process_queue = queue.Queue()
        self.batch_active = False
        self.batch_jobs = deque()
        self.batch_generation = None
        self.batch_total = 0
        self.batch_completed = 0
        self.batch_skipped = 0
        self.batch_planner = None
        self.batch_pick_ready_state = "PICK_READY"
        self.batch_harvest_wait_sec = 2.0
        self.batch_harvest_x_forward_m = 0.040
        self.batch_tcp_wrist_rotation_deg = 10.0
        self.batch_harvest_stage_limit = None
        self.batch_continuous_mode = False
        self.batch_lift_harvest_mode = False
        self.batch_preplan_mode = False
        self.batch_execute_motion = True
        self.batch_prefer_robot_direction = False
        self.batch_adaptive_grasp_max_rotation_deg = 45.0
        self.batch_preplan_failure_message = ""
        self.batch_scene = (0.0, 0.0, 0.0, 0.0)
        self.tomato_motion_results: dict[int, str] = {}
        self.harvest_results: dict[int, bool] = {}
        self.harvest_result_adaptive_rotation: dict[int, bool] = {}
        self.harvest_result_adaptive_rotation_deg: dict[int, float] = {}
        self.harvest_result_approach_axis_local: dict[int, tuple] = {}
        self.harvest_result_approach_reports: dict[int, dict] = {}
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
        self.sweep_tcp_wrist_rotation_deg = 10.0
        self.sweep_continuous_mode = False
        self.sweep_lift_harvest_mode = False
        self.sweep_pick_ready_state = "PICK_READY"
        self.sweep_prefer_robot_direction = False
        self.sweep_adaptive_grasp_max_rotation_deg = 45.0
        self.sweep_cases = deque()
        self.sweep_case_total = 0
        self.sweep_case_number = 0
        self.sweep_total = 0
        self.sweep_completed = 0
        self.sweep_case_tomato_total = 0
        self.sweep_case_tomato_completed = 0
        self.sweep_tomato_queue = deque()
        self.sweep_current_case = None
        self.lift_launch_process = None
        self.lift_node_online = False
        self.lift_simulation_mode = False
        self.lift_calibration_active = False
        self.lift_calibrated = False
        self.lift_last_height_mm = None
        self.lift_harvest_pending = None
        self.linear_motor_launch_process = None
        self.linear_motor_node_online = False
        self.linear_motor_serial_connected = False
        self.linear_motor_serial_status_received = False
        self.linear_motor_pin_state_initialized = False
        self.gripper_stroke_request_id = 0
        self.gripper_outputs_initialized = False
        self.gripper_power_requested = True
        self.gripper_command_in_progress = False
        self.gripper_service_connected = False
        self.capture_camera_in_progress = False
        self.latest_result_image = None
        self.result_image_photo = None
        self.result_image_render_job = None
        self.latest_camera_color_image = None
        self.latest_camera_color_display_image = None
        self.camera_color_image_photo = None
        self.camera_color_image_render_job = None
        self.latest_camera_color_info = None
        self.ui_busy = False
        self.closing = False

        self.root = tk.Tk()
        self.root.title("Farmily Tomato Harvest")
        self.root.maxsize(1920, 1080)
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        initial_width = min(1840, max(1300, screen_width - 40))
        initial_height = min(1020, max(760, screen_height - 60))
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
        self.camera_source_var = tk.StringVar(value=initial_camera_source)
        self.camera_service_display = tk.StringVar(value=self.camera_service)
        self.camera_target_display = tk.StringVar(
            value="현재 Plan 대상 없음"
        )
        self.camera_review_issue_var = tk.StringVar(value="문제 없음")
        self.camera_review_note_var = tk.StringVar(value="")
        self.result_image_status = tk.StringVar(
            value=f"Vision Result 대기: {self.vision_result_image_topic}"
        )
        self.camera_color_image_status = tk.StringVar(
            value=f"Camera Color 이미지 대기: {self.camera_color_image_topic}"
        )
        self.show_detection_markers_var = tk.BooleanVar(value=True)
        self.angle_reference_mode_var = tk.StringVar(
            value=angle_reference_label(ANGLE_REFERENCE_CALYX_TO_STEM)
        )
        self.named_pose_var = tk.StringVar(value="PICK_READY")
        self.pick_ready_state_var = tk.StringVar(value="PICK_READY")
        self.scene_x = tk.StringVar(value="0.355")
        self.scene_y = tk.StringVar(value="-0.375")
        self.scene_z = tk.StringVar(value="0.340")
        self.scene_rotation = tk.StringVar(value="45.0")
        self.motion_velocity_percent = tk.StringVar(value="20")
        self.motion_acceleration_percent = tk.StringVar(value="20")
        self.linear_motor_wait_sec = tk.StringVar(value="2.0")
        self.servo10_angle_deg_var = tk.StringVar(
            value=str(SERVO10_CLOSE_ANGLE_DEG)
        )
        self.servo10_speed_enabled_var = tk.BooleanVar(value=False)
        self.servo10_speed_percent_var = tk.StringVar(
            value=str(SERVO10_DEFAULT_SPEED_PERCENT)
        )
        self.servo10_speed_display = tk.StringVar(
            value="사용자 지정 해제: 기본 50% (약 90.0°/s) 적용"
        )
        self.harvest_forward_distance_mm_var = tk.StringVar(value="40")
        self.tcp_wrist_rotation_deg_var = tk.StringVar(value="10")
        self.step_servo_close_angle_deg_var = tk.StringVar(
            value=str(SERVO10_CLOSE_ANGLE_DEG)
        )
        self.step_linear_motor_extend_sec_var = tk.StringVar(value="3.0")
        self.step_forward_wave_enabled_var = tk.BooleanVar(value=False)
        self.step_tcp_wrist_oscillation_enabled_var = tk.BooleanVar(
            value=False
        )
        self.prefer_robot_direction_var = tk.BooleanVar(value=True)
        self.adaptive_grasp_max_rotation_var = tk.StringVar(value="5.0")
        self.continuous_harvest_var = tk.BooleanVar(value=False)
        self.batch_harvest_stage_var = tk.StringVar(value="전체 수확")
        self.lift_harvest_var = tk.BooleanVar(value=False)
        self.preplan_all_var = tk.BooleanVar(value=False)
        self.step_execution_enabled_var = tk.BooleanVar(value=False)
        self.step_merge_cartesian_var = tk.BooleanVar(value=False)
        self.step_merge_all_trajectories_var = tk.BooleanVar(value=False)
        self.step_custom_delta_vars = {
            stage_number: {
                axis_name: tk.StringVar(value=f"{value:g}")
                for axis_name, value in zip(
                    ("x", "y", "z"),
                    STEP_CUSTOM_STAGE_DEFAULTS_MM[stage_number],
                )
            }
            for stage_number in STEP_CUSTOM_STAGE_DEFAULTS_MM
        }
        self.step_stage_speed_percent_vars = {
            stage_number: tk.DoubleVar(
                value=STEP_CUSTOM_SPEED_DEFAULT_PERCENT_BY_STAGE[stage_number]
            )
            for stage_number in STEP_CUSTOM_SPEED_STAGE_NUMBERS
        }
        self.step_stage_speed_display_vars = {
            stage_number: tk.StringVar(
                value=(
                    f"{STEP_CUSTOM_SPEED_DEFAULT_PERCENT_BY_STAGE[stage_number]:.0f}%"
                )
            )
            for stage_number in STEP_CUSTOM_SPEED_STAGE_NUMBERS
        }
        self.step_preapproach_final_speed_percent_var = tk.DoubleVar(
            value=STEP_PREAPPROACH_FINAL_SPEED_DEFAULT_PERCENT
        )
        self.step_preapproach_via_enabled_var = tk.BooleanVar(value=True)
        self.step_preapproach_final_speed_display_var = tk.StringVar(
            value=(
                f"{STEP_PREAPPROACH_FINAL_SPEED_DEFAULT_PERCENT:.0f}%"
            )
        )
        self.step_status = tk.StringVar(
            value="토마토를 선택하고 스텝 Plan을 생성하세요."
        )
        self.repeat_execution_enabled_var = tk.BooleanVar(value=False)
        self.repeat_last_stage_var = tk.StringVar(value="5")
        self.repeat_forward_distance_mm_var = tk.StringVar(value="40")
        self.repeat_status = tk.StringVar(
            value="토마토를 선택하고 반복 테스트 Plan을 생성하세요."
        )
        self.lift_node_status = tk.StringVar(value="노드 확인 중")
        self.lift_current_height = tk.StringVar(value="-- mm")
        self.lift_target_height = tk.StringVar(value="10.0")
        self.lift_calibration_status = tk.StringVar(
            value="Bottom calibration 필요"
        )
        self.gripper_stroke_status = tk.StringVar(value="서비스 확인 중")
        self.linear_motor_node_status = tk.StringVar(value="실행 안 됨")
        self.linear_motor_serial_status = tk.StringVar(
            value=f"TTY 연결 대기: {self.linear_motor_serial_port}"
        )
        self.motion_velocity_scale = 0.20
        self.motion_acceleration_scale = 0.20
        self.hardware_speed_bar_applied = False
        self.hardware_speed_bar_request_pending = False
        self.hardware_speed_bar_failure_logged = False
        self.status = tk.StringVar(value="MoveIt과 카메라 서비스를 확인해 주세요.")
        self._build_ui()
        self._refresh_lift_node_status()
        self._refresh_linear_motor_node_status()
        self._refresh_gripper_stroke_status()
        self.root.after(500, self._maintain_hardware_speed_bar)
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
        style = ttk.Style(self.root)
        style.configure("Treeview", rowheight=28)
        style.configure("Treeview.Heading", padding=(6, 6))
        style.configure("Action.TButton", padding=(10, 7))
        style.configure("Compact.TButton", padding=(8, 5))
        style.configure("Status.TLabel", padding=(8, 5))

        outer = ttk.Frame(self.root, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(0, weight=1)

        self.main_notebook = ttk.Notebook(outer)
        self.main_notebook.grid(row=0, column=0, sticky="nsew")
        harvest_tab = ttk.Frame(self.main_notebook, padding=10)
        sweep_tab = ttk.Frame(self.main_notebook, padding=10)
        equipment_tab = ttk.Frame(self.main_notebook, padding=10)
        step_tab = ttk.Frame(self.main_notebook, padding=10)
        repeat_tab = ttk.Frame(self.main_notebook, padding=10)
        self.main_notebook.add(harvest_tab, text="  수확 작업  ")
        self.main_notebook.add(step_tab, text="  스텝 실행  ")
        self.main_notebook.add(repeat_tab, text="  접근 반복 테스트  ")
        self.main_notebook.add(sweep_tab, text="  자동 테스트  ")
        self.main_notebook.add(equipment_tab, text="  장면 · 속도 · 리프트  ")

        self._build_harvest_ui(harvest_tab)
        self._build_step_ui(step_tab)
        self._build_repeat_ui(repeat_tab)
        self._build_sweep_ui(sweep_tab)
        self._build_equipment_ui(equipment_tab)

        ttk.Label(
            outer,
            textvariable=self.status,
            anchor="w",
            style="Status.TLabel",
            relief="groove",
        ).grid(row=1, column=0, sticky="ew", pady=(6, 0))

    def _build_harvest_ui(self, frame) -> None:
        frame.columnconfigure(0, weight=1, minsize=560, uniform="harvest")
        frame.columnconfigure(1, weight=1, minsize=560, uniform="harvest")
        frame.rowconfigure(0, weight=1)

        # Camera and motion controls share a compact notebook at the top. The
        # tomato table receives all remaining vertical room so large detection
        # sets can be inspected without constantly scrolling.
        left_panel = ttk.Frame(frame)
        left_panel.grid(row=0, column=0, sticky="nsew")
        left_panel.columnconfigure(0, weight=1)
        left_panel.rowconfigure(0, weight=0, minsize=265)
        left_panel.rowconfigure(1, weight=1, minsize=390)

        right_panel = ttk.Frame(frame)
        right_panel.grid(row=0, column=1, sticky="nsew")
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(
            0, weight=3, minsize=380, uniform="harvest_right"
        )
        right_panel.rowconfigure(
            1, weight=2, minsize=230, uniform="harvest_right"
        )

        controls_frame = ttk.Frame(left_panel)
        controls_frame.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(0, 5),
            pady=(0, 5),
        )
        controls_frame.columnconfigure(0, weight=1)
        controls_frame.rowconfigure(0, weight=1)

        self.harvest_controls_notebook = ttk.Notebook(controls_frame)
        self.harvest_controls_notebook.grid(
            row=0, column=0, sticky="nsew"
        )
        camera_frame = ttk.Frame(
            self.harvest_controls_notebook,
            padding=8,
        )
        self.harvest_controls_notebook.add(
            camera_frame,
            text="  카메라 검출  ",
        )
        camera_frame.columnconfigure(1, weight=1)
        camera_frame.columnconfigure(3, weight=1)
        ttk.Label(camera_frame, text="검출 소스", foreground="#666666").grid(
            row=0, column=0, sticky="w"
        )
        self.camera_source_combo = ttk.Combobox(
            camera_frame,
            textvariable=self.camera_source_var,
            values=CAMERA_SOURCE_OPTIONS,
            state="readonly",
            width=20,
        )
        self.camera_source_combo.grid(
            row=0, column=1, sticky="ew", padx=(8, 0)
        )
        self.camera_source_combo.bind(
            "<<ComboboxSelected>>",
            self._camera_source_changed,
        )
        ttk.Label(camera_frame, text="서비스", foreground="#666666").grid(
            row=0, column=2, sticky="w", padx=(16, 0)
        )
        ttk.Label(
            camera_frame,
            textvariable=self.camera_service_display,
        ).grid(
            row=0, column=3, sticky="w", padx=(8, 0)
        )
        self.detection_markers_checkbox = ttk.Checkbutton(
            camera_frame,
            text="검출 마커 표시",
            variable=self.show_detection_markers_var,
            command=self._detection_marker_option_changed,
        )
        self.detection_markers_checkbox.grid(
            row=1,
            column=0,
            sticky="w",
            pady=(8, 0),
        )
        self.angle_reference_mode_combo = ttk.Combobox(
            camera_frame,
            textvariable=self.angle_reference_mode_var,
            values=ANGLE_REFERENCE_DISPLAY_OPTIONS,
            state="readonly",
            width=25,
        )
        self.angle_reference_mode_combo.grid(
            row=1,
            column=1,
            sticky="ew",
            padx=(8, 0),
            pady=(8, 0),
        )
        self.angle_reference_mode_combo.bind(
            "<<ComboboxSelected>>",
            self._angle_reference_mode_changed,
        )
        self.detect_button = ttk.Button(
            camera_frame,
            text="토마토 촬영 / 검출",
            command=self.detect_tomatoes,
            style="Action.TButton",
        )
        self.detect_button.grid(
            row=2,
            column=0,
            sticky="w",
            pady=(8, 0),
        )
        self.capture_camera_button = ttk.Button(
            camera_frame,
            text="카메라 캡처",
            command=self.capture_camera,
            style="Compact.TButton",
        )
        self.capture_camera_button.grid(
            row=2,
            column=1,
            sticky="w",
            padx=(8, 0),
            pady=(8, 0),
        )

        ttk.Label(
            camera_frame,
            text="저장 자세",
            foreground="#666666",
        ).grid(
            row=1, column=2, sticky="w", padx=(16, 0), pady=(8, 0)
        )
        self.named_pose_combo = ttk.Combobox(
            camera_frame,
            textvariable=self.named_pose_var,
            values=NAMED_POSE_STATES,
            state="readonly",
            width=20,
        )
        self.named_pose_combo.grid(
            row=1,
            column=3,
            sticky="ew",
            padx=(8, 0),
            pady=(8, 0),
        )
        self.named_pose_combo.bind(
            "<<ComboboxSelected>>",
            self._named_pose_selection_changed,
        )
        self.named_pose_button = ttk.Button(
            camera_frame,
            text="Plan & Execute",
            command=self.move_to_named_pose,
            style="Action.TButton",
        )
        self.named_pose_button.grid(
            row=2,
            column=3,
            sticky="w",
            padx=(8, 0),
            pady=(8, 0),
        )
        ttk.Label(
            camera_frame,
            textvariable=self.camera_target_display,
            foreground="#444444",
        ).grid(
            row=3,
            column=0,
            columnspan=4,
            sticky="w",
            pady=(8, 0),
        )
        self.save_camera_target_button = ttk.Button(
            camera_frame,
            text="비전 피드백 JSON 저장/전송",
            command=self.save_active_camera_target_record,
            state="disabled",
            style="Compact.TButton",
        )
        self.save_camera_target_button.grid(
            row=5,
            column=0,
            columnspan=4,
            sticky="w",
            pady=(8, 0),
        )
        ttk.Label(
            camera_frame,
            text="문제 유형",
            foreground="#666666",
        ).grid(row=4, column=0, sticky="w", pady=(8, 0))
        self.camera_review_issue_combo = ttk.Combobox(
            camera_frame,
            textvariable=self.camera_review_issue_var,
            values=VISION_REVIEW_OPTIONS,
            state="normal",
            width=20,
        )
        self.camera_review_issue_combo.grid(
            row=4,
            column=1,
            sticky="ew",
            padx=(8, 0),
            pady=(8, 0),
        )
        ttk.Label(
            camera_frame,
            text="비고",
            foreground="#666666",
        ).grid(
            row=4, column=2, sticky="w", padx=(16, 0), pady=(8, 0)
        )
        self.camera_review_note_entry = ttk.Entry(
            camera_frame,
            textvariable=self.camera_review_note_var,
        )
        self.camera_review_note_entry.grid(
            row=4,
            column=3,
            sticky="ew",
            padx=(8, 0),
            pady=(8, 0),
        )

        list_frame = ttk.LabelFrame(
            left_panel, text="검출된 토마토", padding=8
        )
        list_frame.grid(
            row=1,
            column=0,
            sticky="nsew",
            padx=(0, 5),
            pady=(5, 0),
        )
        list_frame.columnconfigure(0, weight=1)
        list_frame.rowconfigure(0, weight=1)
        columns = (
            "index",
            "frame",
            "motion_result",
            "source_id",
            "x",
            "y",
            "z",
        )
        self.tomato_tree = ttk.Treeview(
            list_frame,
            columns=columns,
            show="headings",
            height=15,
            selectmode="browse",
        )
        headings = {
            "index": "번호",
            "frame": "수확 TF",
            "motion_result": "모션 결과",
            "source_id": "카메라 ID",
            "x": "Camera X (m)",
            "y": "Camera Y (m)",
            "z": "Camera Z (m)",
        }
        self.tomato_tree_headings = headings
        for column in columns:
            self.tomato_tree.heading(column, text=headings[column])
            self.tomato_tree.column(
                column,
                width=80,
                minwidth=40,
                anchor="center",
                stretch=False,
            )
        vertical_scrollbar = ttk.Scrollbar(
            list_frame, orient="vertical", command=self.tomato_tree.yview
        )
        horizontal_scrollbar = ttk.Scrollbar(
            list_frame,
            orient="horizontal",
            command=self.tomato_tree.xview,
        )
        self.tomato_tree.configure(
            yscrollcommand=vertical_scrollbar.set,
            xscrollcommand=horizontal_scrollbar.set,
        )
        self.tomato_tree.grid(row=0, column=0, sticky="nsew")
        vertical_scrollbar.grid(row=0, column=1, sticky="ns")
        horizontal_scrollbar.grid(row=1, column=0, sticky="ew")
        self.tomato_tree.bind("<<TreeviewSelect>>", self._tree_selection_changed)

        motion_frame = ttk.Frame(
            self.harvest_controls_notebook,
            padding=8,
        )
        self.harvest_controls_notebook.add(
            motion_frame,
            text="  수확 모션  ",
        )
        motion_frame.columnconfigure(1, weight=1)
        motion_frame.columnconfigure(3, weight=1)
        ttk.Label(motion_frame, text="선택 토마토").grid(
            row=0, column=0, sticky="w"
        )
        self.tomato_combo = ttk.Combobox(
            motion_frame,
            textvariable=self.selected_tomato,
            state="readonly",
        )
        self.tomato_combo.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(8, 12),
            pady=(0, 8),
        )
        self.tomato_combo.bind("<<ComboboxSelected>>", self._combo_selection_changed)
        action_frame = ttk.Frame(motion_frame)
        action_frame.grid(
            row=0,
            column=2,
            columnspan=2,
            sticky="e",
        )
        self.plan_button = ttk.Button(
            action_frame,
            text="Plan-only 확인",
            command=lambda: self.start_harvest(False),
            state="disabled",
            style="Action.TButton",
        )
        self.plan_button.grid(row=0, column=0, padx=(0, 3))
        self.execute_button = ttk.Button(
            action_frame,
            text="실제 수확 실행",
            command=lambda: self.start_harvest(True),
            state="disabled",
            style="Action.TButton",
        )
        self.execute_button.grid(row=0, column=1, padx=3)
        self.harvest_all_button = ttk.Button(
            action_frame,
            text="전체 연속 수확",
            command=self.start_harvest_all,
            state="disabled",
            style="Action.TButton",
        )
        self.harvest_all_button.grid(
            row=0,
            column=3,
            padx=3,
        )
        self.harvest_all_plan_button = ttk.Button(
            action_frame,
            text="전체 연속 Plan",
            command=self.start_harvest_all_plan,
            state="disabled",
            style="Action.TButton",
        )
        self.harvest_all_plan_button.grid(
            row=0,
            column=2,
            padx=3,
        )
        self.motion_stop_button = ttk.Button(
            action_frame,
            text="모션 정지",
            command=self.stop_active_motion,
            state="disabled",
            style="Action.TButton",
        )
        self.motion_stop_button.grid(
            row=0,
            column=4,
            padx=(3, 0),
        )
        ttk.Separator(motion_frame, orient="horizontal").grid(
            row=1, column=0, columnspan=4, sticky="ew", pady=(4, 8)
        )

        options = ttk.LabelFrame(motion_frame, text="수확 옵션", padding=8)
        options.grid(row=2, column=0, columnspan=4, sticky="ew")
        options.columnconfigure(1, weight=1)
        options.columnconfigure(3, weight=1)
        ttk.Label(options, text="시작/복귀 자세").grid(
            row=0, column=0, sticky="w"
        )
        self.pick_ready_state_combo = ttk.Combobox(
            options,
            textvariable=self.pick_ready_state_var,
            values=PICK_READY_STATES,
            state="readonly",
            width=20,
        )
        self.pick_ready_state_combo.grid(
            row=0, column=1, sticky="ew", padx=(8, 0)
        )
        self.pick_ready_state_combo.bind(
            "<<ComboboxSelected>>",
            self._pick_ready_state_changed,
        )
        ttk.Label(options, text="리니어모터 대기").grid(
            row=0, column=2, sticky="w", padx=(16, 0)
        )
        wait_input = ttk.Frame(options)
        wait_input.grid(
            row=0, column=3, sticky="w", padx=(8, 0)
        )
        self.linear_motor_wait_entry = ttk.Spinbox(
            wait_input,
            textvariable=self.linear_motor_wait_sec,
            from_=0.0,
            to=86400.0,
            increment=1.0,
            wrap=False,
            command=self._linear_motor_wait_changed,
            width=7,
        )
        self.linear_motor_wait_entry.grid(row=0, column=0)
        self.linear_motor_wait_entry.bind(
            "<FocusOut>", self._linear_motor_wait_changed
        )
        self.linear_motor_wait_entry.bind(
            "<Return>", self._linear_motor_wait_changed
        )
        ttk.Label(wait_input, text="초").grid(row=0, column=1, padx=(4, 0))
        ttk.Label(options, text="토마토별 종료 단계").grid(
            row=1,
            column=0,
            sticky="w",
            pady=(8, 0),
        )
        self.batch_harvest_stage_combo = ttk.Combobox(
            options,
            textvariable=self.batch_harvest_stage_var,
            values=BATCH_HARVEST_STAGE_OPTIONS,
            state="readonly",
            width=20,
        )
        self.batch_harvest_stage_combo.grid(
            row=1,
            column=1,
            sticky="ew",
            padx=(8, 0),
            pady=(8, 0),
        )
        ttk.Label(options, text="4단계 진입 길이 (mm)").grid(
            row=1,
            column=2,
            sticky="w",
            padx=(16, 0),
            pady=(8, 0),
        )
        self.harvest_forward_distance_spinbox = ttk.Spinbox(
            options,
            textvariable=self.harvest_forward_distance_mm_var,
            from_=10,
            to=70,
            increment=1,
            width=7,
            command=self._harvest_forward_distance_changed,
        )
        self.harvest_forward_distance_spinbox.grid(
            row=1,
            column=3,
            sticky="w",
            padx=(8, 0),
            pady=(8, 0),
        )
        self.harvest_forward_distance_spinbox.bind(
            "<FocusOut>", self._harvest_forward_distance_changed
        )
        self.harvest_forward_distance_spinbox.bind(
            "<Return>", self._harvest_forward_distance_changed
        )
        ttk.Label(options, text="5단계 TCP 회전각 (°)").grid(
            row=2,
            column=0,
            sticky="w",
            pady=(8, 0),
        )
        self.harvest_tcp_wrist_rotation_spinbox = ttk.Spinbox(
            options,
            textvariable=self.tcp_wrist_rotation_deg_var,
            from_=1,
            to=45,
            increment=1,
            format="%.0f",
            width=7,
            command=self._invalidate_plan,
        )
        self.harvest_tcp_wrist_rotation_spinbox.grid(
            row=2,
            column=1,
            sticky="w",
            padx=(8, 0),
            pady=(8, 0),
        )
        self.harvest_tcp_wrist_rotation_spinbox.bind(
            "<FocusOut>", lambda _event: self._invalidate_plan()
        )
        self.harvest_tcp_wrist_rotation_spinbox.bind(
            "<Return>", lambda _event: self._invalidate_plan()
        )
        self.prefer_robot_direction_checkbox = ttk.Checkbutton(
            options,
            text="진입각: Recommend보다 로봇 방향 우선",
            variable=self.prefer_robot_direction_var,
            command=self._adaptive_grasp_mode_changed,
        )
        self.prefer_robot_direction_checkbox.grid(
            row=3,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(8, 0),
        )
        ttk.Label(options, text="최대 보정각").grid(
            row=2,
            column=2,
            sticky="w",
            padx=(16, 0),
            pady=(8, 0),
        )
        rotation_input = ttk.Frame(options)
        rotation_input.grid(
            row=2,
            column=3,
            sticky="w",
            padx=(8, 0),
            pady=(8, 0),
        )
        self.adaptive_grasp_max_rotation_entry = ttk.Spinbox(
            rotation_input,
            textvariable=self.adaptive_grasp_max_rotation_var,
            from_=0.0,
            to=90.0,
            increment=1.0,
            wrap=False,
            command=self._adaptive_grasp_mode_changed,
            width=7,
        )
        self.adaptive_grasp_max_rotation_entry.grid(row=0, column=0)
        self.adaptive_grasp_max_rotation_entry.bind(
            "<FocusOut>", self._adaptive_grasp_mode_changed
        )
        self.adaptive_grasp_max_rotation_entry.bind(
            "<Return>", self._adaptive_grasp_mode_changed
        )
        ttk.Label(rotation_input, text="° (0~90)").grid(
            row=0, column=1, padx=(4, 0)
        )
        self.continuous_harvest_checkbox = ttk.Checkbutton(
            options,
            text="연속 수확: 식물 바깥 arc 경유",
            variable=self.continuous_harvest_var,
        )
        self.continuous_harvest_checkbox.grid(
            row=3,
            column=2,
            columnspan=2,
            sticky="w",
            pady=(8, 0),
        )
        self.lift_harvest_checkbox = ttk.Checkbutton(
            options,
            text="리프트 수확: 토마토보다 40cm 낮게",
            variable=self.lift_harvest_var,
            command=self._lift_harvest_mode_changed,
        )
        self.lift_harvest_checkbox.grid(
            row=4,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(6, 0),
        )
        self.preplan_all_checkbox = ttk.Checkbutton(
            options,
            text="전체 모션 사전계획 후 저장 trajectory 실행",
            variable=self.preplan_all_var,
            command=self._preplan_all_mode_changed,
        )
        self.preplan_all_checkbox.grid(
            row=4,
            column=2,
            columnspan=2,
            sticky="w",
            pady=(6, 0),
        )

        utility = ttk.Frame(motion_frame)
        utility.grid(row=3, column=0, columnspan=4, sticky="w", pady=(8, 0))
        self.clear_markers_button = ttk.Button(
            utility,
            text="마커 지우기",
            command=self.clear_harvest_result_markers,
            state="disabled",
            style="Compact.TButton",
        )
        self.clear_markers_button.grid(
            row=0,
            column=1,
            padx=(4, 0),
        )
        self.failure_goal_button = ttk.Button(
            utility,
            text="실패 자세 → RViz Goal",
            command=self.show_failure_goal_state,
            state="disabled",
            style="Compact.TButton",
        )
        self.failure_goal_button.grid(
            row=0,
            column=0,
            padx=(0, 4),
        )
        ttk.Label(
            motion_frame,
            text="Plan-only 성공 후 실제 실행이 활성화되며, 성공한 실행은 반복할 수 있습니다.",
            foreground="#666666",
            wraplength=680,
            justify="left",
        ).grid(row=4, column=0, columnspan=4, sticky="w", pady=(8, 0))

        result_image_frame = ttk.LabelFrame(
            right_panel,
            text="토마토 검출 결과 이미지",
            padding=6,
        )
        result_image_frame.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(5, 0),
            pady=(0, 5),
        )
        result_image_frame.columnconfigure(0, weight=1)
        result_image_frame.rowconfigure(0, weight=1)

        self.result_image_notebook = ttk.Notebook(result_image_frame)
        self.result_image_notebook.grid(row=0, column=0, sticky="nsew")

        result_image_tab = ttk.Frame(self.result_image_notebook, padding=4)
        result_image_tab.columnconfigure(0, weight=1)
        result_image_tab.rowconfigure(1, weight=1)
        self.result_image_notebook.add(
            result_image_tab,
            text="Vision Result",
        )
        ttk.Label(
            result_image_tab,
            textvariable=self.result_image_status,
            foreground="#666666",
            anchor="w",
        ).grid(row=0, column=0, sticky="ew", pady=(0, 4))
        self.result_image_label = tk.Label(
            result_image_tab,
            text="원본 이미지와 검출 좌표를 기다리는 중입니다.",
            background="#202020",
            foreground="#dddddd",
            anchor="center",
            relief="sunken",
            borderwidth=1,
        )
        self.result_image_label.grid(row=1, column=0, sticky="nsew")
        self.result_image_label.bind(
            "<Configure>",
            self._schedule_result_image_render,
        )

        camera_color_image_tab = ttk.Frame(
            self.result_image_notebook,
            padding=4,
        )
        camera_color_image_tab.columnconfigure(0, weight=1)
        camera_color_image_tab.rowconfigure(1, weight=1)
        self.result_image_notebook.add(
            camera_color_image_tab,
            text="Camera Color Raw",
        )
        ttk.Label(
            camera_color_image_tab,
            textvariable=self.camera_color_image_status,
            foreground="#666666",
            anchor="w",
        ).grid(row=0, column=0, sticky="ew", pady=(0, 4))
        self.camera_color_image_label = tk.Label(
            camera_color_image_tab,
            text="Camera Color 이미지가 표시됩니다.",
            background="#202020",
            foreground="#dddddd",
            anchor="center",
            relief="sunken",
            borderwidth=1,
        )
        self.camera_color_image_label.grid(
            row=1,
            column=0,
            sticky="nsew",
        )
        self.camera_color_image_label.bind(
            "<Configure>",
            self._schedule_camera_color_image_render,
        )
        self.result_image_notebook.select(camera_color_image_tab)
        self.result_image_notebook.bind(
            "<<NotebookTabChanged>>",
            self._image_notebook_tab_changed,
        )

        log_frame = ttk.LabelFrame(right_panel, text="실행 로그", padding=6)
        log_frame.grid(
            row=1,
            column=0,
            sticky="nsew",
            padx=(5, 0),
            pady=(5, 0),
        )
        log_frame.columnconfigure(0, weight=3, minsize=360)
        log_frame.columnconfigure(1, weight=2, minsize=300)
        log_frame.rowconfigure(0, weight=1)
        log_text_frame = ttk.Frame(log_frame)
        log_text_frame.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(0, 5),
        )
        log_text_frame.columnconfigure(0, weight=1)
        log_text_frame.rowconfigure(0, weight=1)
        self.log_text = tk.Text(
            log_text_frame,
            height=5,
            wrap="word",
            state="disabled",
            padx=6,
            pady=4,
        )
        log_scrollbar = ttk.Scrollbar(
            log_text_frame,
            orient="vertical",
            command=self.log_text.yview,
        )
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_scrollbar.grid(row=0, column=1, sticky="ns")
        self.selected_tomato_plot = PlotCanvas(
            log_frame,
            "선택 토마토 진입 방향",
            empty_message="검출된 토마토를 선택하세요.",
            show_legend=False,
            marker_scale=0.7,
            stem_marker_scale=0.5,
            show_point_labels=False,
            keep_robot_below_tomato=True,
        )
        self.selected_tomato_plot.grid(
            row=0,
            column=1,
            sticky="nsew",
            padx=(5, 0),
        )

    def _build_step_ui(self, frame) -> None:
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        setup = ttk.LabelFrame(frame, text="스텝 수확 Plan", padding=10)
        setup.grid(row=0, column=0, sticky="ew")
        setup.columnconfigure(1, weight=1)
        ttk.Label(setup, text="선택 토마토").grid(row=0, column=0, sticky="w")
        self.step_tomato_combo = ttk.Combobox(
            setup,
            textvariable=self.selected_tomato,
            state="readonly",
        )
        self.step_tomato_combo.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(8, 12),
        )
        self.step_tomato_combo.bind(
            "<<ComboboxSelected>>",
            self._combo_selection_changed,
        )
        self.step_plan_button = ttk.Button(
            setup,
            text="스텝 Plan 생성",
            command=self.start_step_session,
            state="disabled",
            style="Action.TButton",
        )
        self.step_plan_button.grid(row=0, column=2, padx=(0, 12))
        self.step_execution_checkbox = ttk.Checkbutton(
            setup,
            text="실제 로봇 스텝 실행 허용",
            variable=self.step_execution_enabled_var,
            command=self._update_step_controls,
        )
        self.step_execution_checkbox.grid(row=0, column=3, sticky="e")
        ttk.Label(
            setup,
            text=(
                "Plan 생성은 로봇을 움직이지 않습니다. 실행 허용 체크 후에도 "
                "다음 단계는 순서대로 실행하며, 바로 이전 단계만 캐시된 "
                "trajectory를 역순으로 실행할 수 있습니다."
            ),
            foreground="#9a4f00",
            wraplength=1050,
            justify="left",
        ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(8, 0))

        step_content = ttk.Frame(frame)
        step_content.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        step_content.columnconfigure(0, weight=3, minsize=700)
        step_content.columnconfigure(1, weight=1, minsize=520)
        step_content.rowconfigure(0, weight=1)

        stages = ttk.LabelFrame(step_content, text="수확 단계", padding=8)
        stages.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        stages.columnconfigure(0, weight=1)
        stages.rowconfigure(0, weight=1)
        columns = ("number", "motion", "detail", "status")
        self.step_tree = ttk.Treeview(
            stages,
            columns=columns,
            show="headings",
            selectmode="browse",
            height=12,
        )
        for column, heading, width, stretch in (
            ("number", "단계", 65, False),
            ("motion", "동작", 270, True),
            ("detail", "이동량 / 방식", 230, True),
            ("status", "상태", 120, False),
        ):
            self.step_tree.heading(column, text=heading)
            self.step_tree.column(
                column,
                width=width,
                anchor="center",
                stretch=stretch,
            )
        step_scrollbar = ttk.Scrollbar(
            stages,
            orient="vertical",
            command=self.step_tree.yview,
        )
        self.step_tree.configure(yscrollcommand=step_scrollbar.set)
        self.step_tree.grid(row=0, column=0, sticky="nsew")
        step_scrollbar.grid(row=0, column=1, sticky="ns")

        custom = ttk.LabelFrame(
            step_content,
            text="단계별 tip 로컬 XYZ (mm) · 실행 속도",
            padding=10,
        )
        custom.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        for column in range(6):
            custom.columnconfigure(column, weight=1)
        custom.columnconfigure(4, weight=3)
        for column, text_value in enumerate(
            ("단계", "X", "Y", "Z", "속도", "%")
        ):
            ttk.Label(
                custom,
                text=text_value,
                anchor="center",
            ).grid(row=0, column=column, sticky="ew", padx=3)
        self.step_custom_delta_entries = []
        self.step_stage_speed_scales = []
        ttk.Label(
            custom,
            text="A→PRE",
            anchor="center",
        ).grid(row=1, column=0, sticky="ew", padx=3, pady=4)
        self.step_preapproach_via_checkbox = ttk.Checkbutton(
            custom,
            text="경유점 A 사용 (PICK_READY → A → PRE_APPROACH)",
            variable=self.step_preapproach_via_enabled_var,
            command=self._step_preapproach_via_changed,
        )
        self.step_preapproach_via_checkbox.grid(
            row=1,
            column=1,
            columnspan=3,
            sticky="w",
            padx=3,
            pady=4,
        )
        preapproach_speed_scale = ttk.Scale(
            custom,
            from_=STEP_CUSTOM_SPEED_MIN_PERCENT,
            to=STEP_CUSTOM_SPEED_MAX_PERCENT,
            orient="horizontal",
            variable=self.step_preapproach_final_speed_percent_var,
            command=self._step_preapproach_final_speed_changed,
        )
        preapproach_speed_scale.grid(
            row=1,
            column=4,
            sticky="ew",
            padx=(6, 3),
            pady=4,
        )
        self.step_preapproach_final_speed_scale = (
            preapproach_speed_scale
        )
        self.step_stage_speed_scales.append(preapproach_speed_scale)
        ttk.Label(
            custom,
            textvariable=self.step_preapproach_final_speed_display_var,
            anchor="e",
            width=5,
        ).grid(row=1, column=5, sticky="e", padx=3, pady=4)
        for row, stage_number in enumerate(
            STEP_CUSTOM_SPEED_STAGE_NUMBERS,
            start=2,
        ):
            ttk.Label(
                custom,
                text=f"{stage_number}단계",
                anchor="center",
            ).grid(row=row, column=0, sticky="ew", padx=3, pady=4)
            if stage_number in self.step_custom_delta_vars:
                for column, axis_name in enumerate(
                    ("x", "y", "z"),
                    start=1,
                ):
                    entry = ttk.Spinbox(
                        custom,
                        textvariable=(
                            self.step_custom_delta_vars[stage_number][axis_name]
                        ),
                        from_=-200,
                        to=200,
                        increment=1,
                        format="%.0f",
                        width=7,
                        justify="center",
                    )
                    entry.grid(
                        row=row,
                        column=column,
                        sticky="ew",
                        padx=3,
                        pady=4,
                    )
                    self.step_custom_delta_entries.append(entry)
            else:
                for column in range(1, 4):
                    ttk.Label(
                        custom,
                        text="—",
                        anchor="center",
                        foreground="#888888",
                    ).grid(
                        row=row,
                        column=column,
                        sticky="ew",
                        padx=3,
                        pady=4,
                    )
            speed_scale = ttk.Scale(
                custom,
                from_=STEP_CUSTOM_SPEED_MIN_PERCENT,
                to=STEP_CUSTOM_SPEED_MAX_PERCENT,
                orient="horizontal",
                variable=self.step_stage_speed_percent_vars[stage_number],
                command=lambda value, number=stage_number: (
                    self._step_stage_speed_changed(number, value)
                ),
            )
            speed_scale.grid(
                row=row,
                column=4,
                sticky="ew",
                padx=(6, 3),
                pady=4,
            )
            self.step_stage_speed_scales.append(speed_scale)
            ttk.Label(
                custom,
                textvariable=self.step_stage_speed_display_vars[stage_number],
                anchor="e",
                width=5,
            ).grid(row=row, column=5, sticky="e", padx=3, pady=4)
        ttk.Label(
            custom,
            text="5단계 TCP 회전각",
            anchor="center",
        ).grid(row=9, column=0, sticky="ew", padx=3, pady=(10, 4))
        self.tcp_wrist_rotation_spinbox = ttk.Spinbox(
            custom,
            textvariable=self.tcp_wrist_rotation_deg_var,
            from_=1,
            to=45,
            increment=1,
            format="%.0f",
            width=8,
            justify="center",
        )
        self.tcp_wrist_rotation_spinbox.grid(
            row=9,
            column=1,
            sticky="ew",
            padx=3,
            pady=(10, 4),
        )
        ttk.Label(custom, text="° (-X → +X → 원점)").grid(
            row=9,
            column=2,
            columnspan=4,
            sticky="w",
            padx=3,
            pady=(10, 4),
        )
        self.step_custom_delta_entries.append(
            self.tcp_wrist_rotation_spinbox
        )
        self.step_tcp_wrist_oscillation_checkbox = ttk.Checkbutton(
            custom,
            text="TCP 좌우 흔들기 사용",
            variable=self.step_tcp_wrist_oscillation_enabled_var,
            command=self._step_wrist_oscillation_changed,
        )
        self.step_tcp_wrist_oscillation_checkbox.grid(
            row=10,
            column=0,
            columnspan=6,
            sticky="w",
            padx=3,
            pady=(5, 0),
        )
        ttk.Label(
            custom,
            text="서보 닫기 각도",
            anchor="center",
        ).grid(row=11, column=0, sticky="ew", padx=3, pady=(10, 4))
        self.step_servo_close_angle_spinbox = ttk.Spinbox(
            custom,
            textvariable=self.step_servo_close_angle_deg_var,
            from_=10,
            to=173,
            increment=1,
            format="%.0f",
            width=8,
            justify="center",
        )
        self.step_servo_close_angle_spinbox.grid(
            row=11,
            column=1,
            sticky="ew",
            padx=3,
            pady=(10, 4),
        )
        ttk.Label(custom, text="° (10~173°)").grid(
            row=11,
            column=2,
            columnspan=4,
            sticky="w",
            padx=3,
            pady=(10, 4),
        )
        self.step_custom_delta_entries.append(
            self.step_servo_close_angle_spinbox
        )
        ttk.Label(
            custom,
            text="리니어모터 늘림 시간",
            anchor="center",
        ).grid(row=12, column=0, sticky="ew", padx=3, pady=(10, 4))
        self.step_linear_motor_extend_spinbox = ttk.Spinbox(
            custom,
            textvariable=self.step_linear_motor_extend_sec_var,
            from_=0.0,
            to=60.0,
            increment=1.0,
            format="%.1f",
            width=8,
            justify="center",
        )
        self.step_linear_motor_extend_spinbox.grid(
            row=12,
            column=1,
            sticky="ew",
            padx=3,
            pady=(10, 4),
        )
        ttk.Label(custom, text="초 (0~60초)").grid(
            row=12,
            column=2,
            columnspan=4,
            sticky="w",
            padx=3,
            pady=(10, 4),
        )
        self.step_custom_delta_entries.append(
            self.step_linear_motor_extend_spinbox
        )
        self.step_forward_wave_checkbox = ttk.Checkbutton(
            custom,
            text="4단계 Z축 웨이브 사용 (±5mm × 3회)",
            variable=self.step_forward_wave_enabled_var,
            command=self._step_forward_wave_changed,
        )
        self.step_forward_wave_checkbox.grid(
            row=13,
            column=0,
            columnspan=6,
            sticky="w",
            padx=3,
            pady=(8, 0),
        )
        ttk.Label(
            custom,
            text=(
                "XYZ 값은 tomato_gripper_tip 로컬 이동량입니다.\n"
                "입력 범위: 축별 -200~+200 mm (화살표 1회 = 1 mm)\n"
                "단계별 속도: 기존 계획 궤적 대비 10~100%\n"
                "서보 닫기 각도: 10~173° (화살표 1회 = 1°)\n"
                "리니어모터 늘림: 0~60초 (화살표 1회 = 1초)\n"
                "Plan 생성 후에는 세션 종료까지 잠깁니다."
            ),
            foreground="#666666",
            justify="left",
        ).grid(
            row=14,
            column=0,
            columnspan=6,
            sticky="w",
            pady=(10, 0),
        )

        controls = ttk.Frame(frame)
        controls.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        controls.columnconfigure(6, weight=1)
        self.step_previous_button = ttk.Button(
            controls,
            text="이전 단계 역순 실행",
            command=self.execute_previous_step,
            state="disabled",
            style="Action.TButton",
        )
        self.step_previous_button.grid(row=0, column=0, padx=(0, 6))
        self.step_next_button = ttk.Button(
            controls,
            text="다음 단계 실행",
            command=self.execute_next_step,
            state="disabled",
            style="Action.TButton",
        )
        self.step_next_button.grid(row=0, column=1, padx=6)
        self.step_execute_to_button = ttk.Button(
            controls,
            text="선택 단계까지 실행",
            command=self.execute_steps_through_selection,
            state="disabled",
            style="Action.TButton",
        )
        self.step_execute_to_button.grid(row=0, column=2, padx=6)
        self.step_merge_cartesian_checkbox = ttk.Checkbutton(
            controls,
            text="접근 Cartesian 경로 합쳐서 실행",
            variable=self.step_merge_cartesian_var,
            command=lambda: self._step_merge_option_changed("cartesian"),
        )
        self.step_merge_cartesian_checkbox.grid(
            row=1,
            column=0,
            columnspan=3,
            sticky="w",
            pady=(7, 0),
        )
        self.step_merge_all_checkbox = ttk.Checkbutton(
            controls,
            text="OMPL 포함 전체경로 합쳐서 실행",
            variable=self.step_merge_all_trajectories_var,
            command=lambda: self._step_merge_option_changed("all"),
        )
        self.step_merge_all_checkbox.grid(
            row=1,
            column=3,
            columnspan=3,
            sticky="w",
            padx=(12, 0),
            pady=(7, 0),
        )
        self.step_stop_button = ttk.Button(
            controls,
            text="모션 즉시 정지",
            command=self.stop_active_motion,
            state="disabled",
            style="Action.TButton",
        )
        self.step_stop_button.grid(row=0, column=3, padx=6)
        self.step_close_button = ttk.Button(
            controls,
            text="스텝 세션 종료",
            command=self.close_step_session,
            state="disabled",
            style="Compact.TButton",
        )
        self.step_close_button.grid(row=0, column=4, padx=6)
        self.step_preapproach_goal_button = ttk.Button(
            controls,
            text="PRE_APPROACH → RViz Goal",
            command=self.show_preapproach_goal_state,
            state="disabled",
            style="Compact.TButton",
        )
        self.step_preapproach_goal_button.grid(
            row=3,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(7, 0),
        )
        ttk.Label(
            controls,
            textvariable=self.step_status,
            anchor="w",
        ).grid(row=0, column=5, sticky="ew", padx=(14, 0))
        ttk.Label(
            controls,
            text=(
                "Cartesian 합치기는 연속 Cartesian만 다시 계획합니다. "
                "OMPL 포함 합치기는 캐시된 OMPL·Cartesian 관절 경로를 "
                "전체 재타이밍하여 한 trajectory로 실행합니다."
            ),
            foreground="#666666",
            anchor="w",
        ).grid(
            row=2,
            column=0,
            columnspan=6,
            sticky="w",
            padx=(0, 0),
            pady=(7, 0),
        )

    def _build_repeat_ui(self, frame) -> None:
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        setup = ttk.LabelFrame(frame, text="접근 1↔X 반복 Plan", padding=10)
        setup.grid(row=0, column=0, sticky="ew")
        setup.columnconfigure(1, weight=1)
        ttk.Label(setup, text="선택 토마토").grid(row=0, column=0, sticky="w")
        self.repeat_tomato_combo = ttk.Combobox(
            setup,
            textvariable=self.selected_tomato,
            state="readonly",
        )
        self.repeat_tomato_combo.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(8, 12),
        )
        self.repeat_tomato_combo.bind(
            "<<ComboboxSelected>>",
            self._combo_selection_changed,
        )
        ttk.Label(setup, text="마지막 단계 X").grid(
            row=0, column=2, sticky="w"
        )
        self.repeat_last_stage_combo = ttk.Combobox(
            setup,
            textvariable=self.repeat_last_stage_var,
            values=("1", "2", "3", "4", "5", "6", "7"),
            state="readonly",
            width=5,
        )
        self.repeat_last_stage_combo.grid(
            row=0, column=3, sticky="w", padx=(8, 12)
        )
        self.repeat_last_stage_combo.bind(
            "<<ComboboxSelected>>",
            self._repeat_last_stage_changed,
        )
        self.repeat_plan_button = ttk.Button(
            setup,
            text="반복 테스트 Plan 생성",
            command=self.start_repeat_session,
            state="disabled",
            style="Action.TButton",
        )
        self.repeat_plan_button.grid(row=0, column=4, padx=(0, 12))
        self.repeat_execution_checkbox = ttk.Checkbutton(
            setup,
            text="실제 로봇 반복 실행 허용",
            variable=self.repeat_execution_enabled_var,
            command=self._update_step_controls,
        )
        self.repeat_execution_checkbox.grid(row=0, column=5, sticky="e")
        ttk.Label(setup, text="4단계 진입 길이 (mm)").grid(
            row=1, column=2, sticky="w", pady=(8, 0)
        )
        self.repeat_forward_distance_spinbox = ttk.Spinbox(
            setup,
            textvariable=self.repeat_forward_distance_mm_var,
            from_=10,
            to=70,
            increment=1,
            width=7,
        )
        self.repeat_forward_distance_spinbox.grid(
            row=1,
            column=3,
            sticky="w",
            padx=(8, 12),
            pady=(8, 0),
        )
        ttk.Label(
            setup,
            text=(
                "1→X는 현재 자세부터 선택 단계까지 자동 실행합니다. "
                "X→1은 저장된 joint trajectory를 재계획 없이 완전히 역재생합니다. "
                "전체 실행은 검출된 모든 토마토에 같은 동작을 순서대로 적용합니다."
            ),
            foreground="#9a4f00",
            wraplength=1150,
            justify="left",
        ).grid(row=2, column=0, columnspan=6, sticky="w", pady=(8, 0))

        stages = ttk.LabelFrame(frame, text="반복 대상 단계", padding=8)
        stages.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        stages.columnconfigure(0, weight=1)
        stages.rowconfigure(0, weight=1)
        columns = ("number", "motion", "detail", "status")
        self.repeat_tree = ttk.Treeview(
            stages,
            columns=columns,
            show="headings",
            selectmode="none",
            height=7,
        )
        for column, heading, width, stretch in (
            ("number", "단계", 65, False),
            ("motion", "동작", 410, True),
            ("detail", "이동량 / 방식", 320, True),
            ("status", "상태", 180, False),
        ):
            self.repeat_tree.heading(column, text=heading)
            self.repeat_tree.column(
                column,
                width=width,
                anchor="center",
                stretch=stretch,
            )
        self.repeat_tree.grid(row=0, column=0, sticky="nsew")

        controls = ttk.Frame(frame)
        controls.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        controls.columnconfigure(5, weight=1)
        self.repeat_forward_button = ttk.Button(
            controls,
            text="1 → 5 연속 진입",
            command=self.execute_repeat_forward,
            state="disabled",
            style="Action.TButton",
        )
        self.repeat_forward_button.grid(row=0, column=0, padx=(0, 6))
        self.repeat_reverse_button = ttk.Button(
            controls,
            text="5 → 1 역순 복귀",
            command=self.execute_repeat_reverse,
            state="disabled",
            style="Action.TButton",
        )
        self.repeat_reverse_button.grid(row=0, column=1, padx=6)
        self.repeat_all_button = ttk.Button(
            controls,
            text="전체 토마토 1 → 5 → 1",
            command=self.start_repeat_all,
            state="disabled",
            style="Action.TButton",
        )
        self.repeat_all_button.grid(row=0, column=2, padx=6)
        self.repeat_pause_button = ttk.Button(
            controls,
            text="일시 정지",
            command=self.toggle_repeat_pause,
            state="disabled",
            style="Action.TButton",
        )
        self.repeat_pause_button.grid(row=0, column=3, padx=6)
        self.repeat_stop_button = ttk.Button(
            controls,
            text="모션 즉시 정지",
            command=self.stop_active_motion,
            state="disabled",
            style="Action.TButton",
        )
        self.repeat_stop_button.grid(row=0, column=4, padx=6)
        self.repeat_close_button = ttk.Button(
            controls,
            text="반복 세션 종료",
            command=self.close_step_session,
            state="disabled",
            style="Compact.TButton",
        )
        self.repeat_close_button.grid(row=0, column=5, padx=6)
        ttk.Label(
            controls,
            textvariable=self.repeat_status,
            anchor="w",
        ).grid(row=0, column=6, sticky="ew", padx=(14, 0))

    def _build_equipment_ui(self, frame) -> None:
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(4, weight=1)

        scene_frame = ttk.LabelFrame(
            frame, text="토마토 줄기 위치 / 회전", padding=10
        )
        scene_frame.grid(row=0, column=0, sticky="ew")
        for column in range(9):
            scene_frame.columnconfigure(column, weight=0)
        scene_frame.columnconfigure(8, weight=1)
        for column, (label, variable) in enumerate(
            (("X (m)", self.scene_x), ("Y (m)", self.scene_y), ("Z (m)", self.scene_z))
        ):
            base = column * 2
            ttk.Label(scene_frame, text=label).grid(
                row=0, column=base, sticky="w", padx=(0 if column == 0 else 12, 4)
            )
            ttk.Entry(scene_frame, textvariable=variable, width=9).grid(
                row=0, column=base + 1
            )
        ttk.Label(scene_frame, text="회전 (°)").grid(
            row=0, column=6, padx=(12, 4)
        )
        ttk.Spinbox(
            scene_frame,
            from_=-360.0,
            to=360.0,
            increment=5.0,
            textvariable=self.scene_rotation,
            width=9,
        ).grid(row=0, column=7)
        actions = ttk.Frame(scene_frame)
        actions.grid(row=0, column=8, sticky="e", padx=(20, 0))
        self.read_scene_button = ttk.Button(
            actions,
            text="현재값 읽기",
            command=self.read_scene_position,
            style="Compact.TButton",
        )
        self.read_scene_button.grid(row=0, column=0, padx=(0, 6))
        self.set_scene_button = ttk.Button(
            actions,
            text="위치 / 회전 적용",
            command=self.set_scene_position,
            style="Action.TButton",
        )
        self.set_scene_button.grid(row=0, column=1)
        ttk.Label(
            scene_frame,
            text="메인 줄기 축을 기준으로 가지와 토마토 전체가 함께 이동·회전합니다.",
            foreground="#666666",
        ).grid(row=1, column=0, columnspan=9, sticky="w", pady=(8, 0))

        speed_frame = ttk.LabelFrame(frame, text="로봇 이동 속도", padding=10)
        speed_frame.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(
            speed_frame,
            text="RB Speed Bar: 자동 100%",
            foreground="#1b6e1b",
        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=(0, 18))
        speed_items = (
            ("OMPL / Joint 속도", self.motion_velocity_percent),
            ("OMPL / Joint 가속도", self.motion_acceleration_percent),
        )
        for index, (label, variable) in enumerate(speed_items):
            base = 2 + index * 2
            ttk.Label(speed_frame, text=f"{label} (%)").grid(
                row=0, column=base, padx=(0 if index == 0 else 18, 5)
            )
            ttk.Spinbox(
                speed_frame,
                from_=1,
                to=100,
                increment=5,
                textvariable=variable,
                width=7,
            ).grid(row=0, column=base + 1)
        self.apply_speed_button = ttk.Button(
            speed_frame,
            text="속도 적용",
            command=self.apply_motion_speed,
            style="Action.TButton",
        )
        self.apply_speed_button.grid(row=0, column=6, padx=(20, 0))
        ttk.Label(
            speed_frame,
            text=(
                "OMPL/Joint 값은 OMPL 계획에만 적용됩니다. "
                "Cartesian 스텝은 스텝 실행 탭의 단계별 속도를 사용합니다."
            ),
            foreground="#666666",
        ).grid(row=1, column=0, columnspan=7, sticky="w", pady=(8, 0))

        lift_frame = ttk.LabelFrame(frame, text="UV 리프트 제어", padding=10)
        lift_frame.grid(row=2, column=0, sticky="new", pady=(10, 0))
        self._build_lift_ui(lift_frame)

        gripper_frame = ttk.LabelFrame(
            frame, text="그리퍼 스트로크 제어", padding=10
        )
        gripper_frame.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        self._build_gripper_stroke_ui(gripper_frame)

    def _build_gripper_stroke_ui(self, frame) -> None:
        """Build Arduino serial and linear-motor direction controls."""
        frame.columnconfigure(6, weight=1)
        ttk.Label(frame, text="Arduino 노드").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            frame,
            textvariable=self.linear_motor_node_status,
            width=16,
        ).grid(row=0, column=1, sticky="w", padx=(8, 8))
        self.linear_motor_launch_button = ttk.Button(
            frame,
            text="Arduino 노드 실행",
            command=self.launch_linear_motor_node,
            style="Compact.TButton",
        )
        self.linear_motor_launch_button.grid(
            row=0, column=2, sticky="w", padx=(0, 20)
        )
        ttk.Label(frame, text="TTY 상태").grid(
            row=0, column=3, sticky="w"
        )
        ttk.Label(
            frame,
            textvariable=self.linear_motor_serial_status,
            width=54,
        ).grid(
            row=0,
            column=4,
            columnspan=3,
            sticky="w",
            padx=(8, 0),
        )
        ttk.Separator(frame, orient="horizontal").grid(
            row=1,
            column=0,
            columnspan=7,
            sticky="ew",
            pady=10,
        )
        ttk.Label(frame, text="출력 상태").grid(
            row=2, column=0, sticky="w"
        )
        ttk.Label(
            frame,
            textvariable=self.gripper_stroke_status,
            width=38,
        ).grid(row=2, column=1, sticky="w", padx=(8, 20))
        self.gripper_extend_button = ttk.Button(
            frame,
            text="늘림 (3초)",
            command=lambda: self.control_gripper_stroke("extend"),
            style="Action.TButton",
            width=12,
            state="disabled",
        )
        self.gripper_extend_button.grid(row=2, column=2, padx=(0, 6))
        self.gripper_retract_button = ttk.Button(
            frame,
            text="줄임",
            command=lambda: self.control_gripper_stroke("retract"),
            style="Action.TButton",
            width=12,
            state="disabled",
        )
        self.gripper_retract_button.grid(row=2, column=3, padx=(0, 6))
        self.gripper_stop_button = ttk.Button(
            frame,
            text="정지",
            command=lambda: self.control_gripper_stroke("stop"),
            style="Action.TButton",
            width=12,
            state="disabled",
        )
        self.gripper_stop_button.grid(row=2, column=4)
        self.gripper_power_on_button = ttk.Button(
            frame,
            text="전원 ON",
            command=lambda: self.control_gripper_power(True),
            style="Action.TButton",
            width=12,
            state="disabled",
        )
        self.gripper_power_on_button.grid(row=2, column=5, padx=(12, 6))
        self.gripper_power_off_button = ttk.Button(
            frame,
            text="전원 OFF",
            command=lambda: self.control_gripper_power(False),
            style="Compact.TButton",
            width=12,
            state="disabled",
        )
        self.gripper_power_off_button.grid(row=2, column=6)
        ttk.Label(
            frame,
            text=(
                "RB: DOUT0·DOUT8 HIGH, 나머지 DOUT LOW · "
                "Arduino: 늘림 PIN8 HIGH/PIN9 LOW 후 3초 자동 정지, "
                "줄임 PIN8 LOW/PIN9 HIGH, 정지 모두 LOW"
            ),
            foreground="#666666",
        ).grid(row=5, column=0, columnspan=7, sticky="w", pady=(8, 0))

        ttk.Label(frame, text="PIN10 서보 각도").grid(
            row=3, column=0, sticky="w", pady=(10, 0)
        )
        self.servo10_angle_entry = ttk.Spinbox(
            frame,
            from_=10,
            to=173,
            increment=1,
            textvariable=self.servo10_angle_deg_var,
            width=8,
        )
        self.servo10_angle_entry.grid(
            row=3, column=1, sticky="w", padx=(8, 4), pady=(10, 0)
        )
        ttk.Label(frame, text="°").grid(
            row=3, column=1, sticky="w", padx=(78, 0), pady=(10, 0)
        )
        self.servo10_angle_button = ttk.Button(
            frame,
            text="각도 전송",
            command=self.send_servo10_angle,
            style="Action.TButton",
            width=12,
            state="disabled",
        )
        self.servo10_angle_button.grid(
            row=3, column=2, sticky="w", pady=(10, 0)
        )
        self.servo10_close_button = ttk.Button(
            frame,
            text=f"{SERVO10_CLOSE_ANGLE_DEG}° 닫기",
            command=lambda: self.send_servo10_angle(
                SERVO10_CLOSE_ANGLE_DEG
            ),
            style="Action.TButton",
            width=12,
            state="disabled",
        )
        self.servo10_close_button.grid(
            row=3,
            column=3,
            sticky="w",
            padx=(6, 0),
            pady=(10, 0),
        )
        self.servo10_open_button = ttk.Button(
            frame,
            text=f"{SERVO10_OPEN_ANGLE_DEG}° 열기",
            command=lambda: self.send_servo10_angle(
                SERVO10_OPEN_ANGLE_DEG
            ),
            style="Action.TButton",
            width=12,
            state="disabled",
        )
        self.servo10_open_button.grid(
            row=3,
            column=4,
            sticky="w",
            padx=(6, 0),
            pady=(10, 0),
        )
        ttk.Label(
            frame,
            text="직접 각도 전송 또는 닫기/열기 위치로 즉시 이동합니다.",
            foreground="#666666",
        ).grid(
            row=3,
            column=5,
            columnspan=2,
            sticky="w",
            padx=(8, 0),
            pady=(10, 0),
        )

        self.servo10_speed_checkbox = ttk.Checkbutton(
            frame,
            text="서보 속도 조정",
            variable=self.servo10_speed_enabled_var,
            command=self._servo10_speed_option_changed,
        )
        self.servo10_speed_checkbox.grid(
            row=4,
            column=0,
            sticky="w",
            pady=(8, 0),
        )
        speed_input = ttk.Frame(frame)
        speed_input.grid(
            row=4,
            column=1,
            sticky="w",
            padx=(8, 4),
            pady=(8, 0),
        )
        self.servo10_speed_spinbox = ttk.Spinbox(
            speed_input,
            from_=1,
            to=100,
            increment=1,
            textvariable=self.servo10_speed_percent_var,
            command=self._servo10_speed_option_changed,
            width=7,
            state="disabled",
        )
        self.servo10_speed_spinbox.grid(row=0, column=0)
        self.servo10_speed_spinbox.bind(
            "<FocusOut>", self._servo10_speed_option_changed
        )
        self.servo10_speed_spinbox.bind(
            "<Return>", self._servo10_speed_option_changed
        )
        ttk.Label(speed_input, text="%").grid(row=0, column=1, padx=(3, 0))
        ttk.Label(
            frame,
            textvariable=self.servo10_speed_display,
            foreground="#666666",
        ).grid(
            row=4,
            column=2,
            columnspan=5,
            sticky="w",
            padx=(4, 0),
            pady=(8, 0),
        )

    def _servo10_speed_option_changed(self, _event=None) -> None:
        enabled = bool(self.servo10_speed_enabled_var.get())
        self.servo10_speed_spinbox.configure(
            state="normal" if enabled else "disabled"
        )
        if not enabled:
            self.servo10_speed_display.set(
                "사용자 지정 해제: 기본 50% (약 90.0°/s) 적용"
            )
            return
        try:
            percent = servo_speed_percent(
                self.servo10_speed_percent_var.get()
            )
        except ValueError as error:
            self.servo10_speed_display.set(str(error))
            return
        self.servo10_speed_percent_var.set(str(percent))
        degrees_per_second = servo_speed_degrees_per_second(percent)
        if degrees_per_second is None:
            self.servo10_speed_display.set(
                "100%: 목표 각도로 즉시 이동"
            )
        else:
            self.servo10_speed_display.set(
                f"{percent}%: 임시 기준 약 {degrees_per_second:.1f}°/s "
                f"(100% 기준 {SERVO10_MAX_SPEED_DEG_PER_SEC:.0f}°/s)"
            )

    def _build_lift_ui(self, frame) -> None:
        """Build controls backed by the farmily_uv_lift ROS topics."""
        frame.columnconfigure(6, weight=1)
        ttk.Label(frame, text="노드 상태").grid(row=0, column=0, sticky="w")
        ttk.Label(
            frame,
            textvariable=self.lift_node_status,
            width=18,
        ).grid(row=0, column=1, sticky="w", padx=(8, 16))
        self.lift_launch_button = ttk.Button(
            frame,
            text="노드 실행",
            command=self.launch_lift_node,
            style="Compact.TButton",
        )
        self.lift_launch_button.grid(row=0, column=2, padx=(0, 6))
        self.lift_calibration_button = ttk.Button(
            frame,
            text="Bottom Calibration",
            command=self.start_lift_bottom_calibration,
            state="disabled",
            style="Compact.TButton",
        )
        self.lift_calibration_button.grid(row=0, column=3, padx=(0, 12))
        ttk.Label(
            frame,
            textvariable=self.lift_calibration_status,
            width=24,
        ).grid(row=0, column=4, sticky="w")

        ttk.Separator(frame, orient="horizontal").grid(
            row=1, column=0, columnspan=7, sticky="ew", pady=10
        )
        ttk.Label(frame, text="현재 높이").grid(row=2, column=0, sticky="w")
        ttk.Label(
            frame,
            textvariable=self.lift_current_height,
            font="TkHeadingFont",
            width=16,
        ).grid(row=2, column=1, sticky="w", padx=(8, 16))
        ttk.Label(frame, text="목표 높이").grid(
            row=2,
            column=2,
            sticky="w",
        )
        self.lift_target_height_entry = ttk.Entry(
            frame,
            textvariable=self.lift_target_height,
            width=10,
            state="disabled",
        )
        self.lift_target_height_entry.grid(
            row=2,
            column=3,
            sticky="w",
            padx=(8, 4),
        )
        ttk.Label(frame, text="mm").grid(row=2, column=4, sticky="w")
        lift_actions = ttk.Frame(frame)
        lift_actions.grid(row=2, column=5, columnspan=2, sticky="e", padx=(20, 0))
        self.lift_move_button = ttk.Button(
            lift_actions,
            text="높이 이동",
            command=self.move_lift_to_height,
            state="disabled",
            style="Action.TButton",
        )
        self.lift_move_button.grid(row=0, column=0, padx=(0, 6))
        self.lift_stop_button = ttk.Button(
            lift_actions,
            text="이동 정지",
            command=self.stop_lift_motion,
            state="disabled",
            style="Action.TButton",
        )
        self.lift_stop_button.grid(row=0, column=1)
        ttk.Label(
            frame,
            text=(
                "높이는 Bottom 기준 mm입니다. 노드가 감지되면 노드 실행 버튼은 자동으로 비활성화됩니다."
            ),
            foreground="#666666",
        ).grid(
            row=3,
            column=0,
            columnspan=7,
            sticky="w",
            pady=(10, 0),
        )

    def _build_sweep_ui(self, frame) -> None:
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(2, weight=3, minsize=250)
        frame.rowconfigure(3, weight=2, minsize=140)
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

        top = ttk.Frame(frame)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(0, weight=0)
        top.columnconfigure(1, weight=1)

        range_frame = ttk.LabelFrame(
            top,
            text="줄기 위치 / 회전 범위",
            padding=8,
        )
        range_frame.grid(row=0, column=0, sticky="nw")
        headers = ("항목", "시작", "종료", "변화량", "랜덤")
        for column, header in enumerate(headers):
            ttk.Label(range_frame, text=header).grid(
                row=0, column=column, padx=5, pady=(0, 4)
            )
        rows = (
            ("X (m)", "x"),
            ("Y (m)", "y"),
            ("Z (m)", "z"),
            ("회전 (°)", "rotation"),
        )
        for row, (label, key) in enumerate(rows, start=1):
            ttk.Label(range_frame, text=label).grid(
                row=row, column=0, sticky="w", padx=(0, 8), pady=3
            )
            for column, prefix in enumerate(("start", "end", "step"), start=1):
                variable = self.sweep_inputs[f"{prefix}_{key}"]
                if key in ("x", "y", "z"):
                    # XYZ values use metres internally. One arrow click changes
                    # the selected value by exactly 1 mm.
                    widget = ttk.Spinbox(
                        range_frame,
                        textvariable=variable,
                        from_=0.001 if prefix == "step" else -10.0,
                        to=10.0,
                        increment=0.001,
                        format="%.3f",
                        width=11,
                    )
                else:
                    widget = ttk.Entry(
                        range_frame,
                        textvariable=variable,
                        width=11,
                    )
                widget.grid(row=row, column=column, padx=4, pady=3)
            random_checkbox = ttk.Checkbutton(
                range_frame,
                variable=self.sweep_inputs[f"random_{key}"],
            )
            random_checkbox.grid(row=row, column=4)

        self.sweep_summary = tk.StringVar(value="대기 중")
        self.sweep_execute_motion_var = tk.BooleanVar(value=False)
        run_frame = ttk.LabelFrame(top, text="자동 테스트 실행", padding=10)
        run_frame.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
        run_frame.columnconfigure(0, weight=1)
        ttk.Label(
            run_frame,
            text=(
                "각 XYZ 위치에서 회전 범위를 테스트합니다. 랜덤 축은 케이스마다 "
                "새 값을 사용하며, 검출 토마토가 없으면 다음 케이스로 이동합니다."
            ),
            wraplength=650,
            justify="left",
            foreground="#666666",
        ).grid(row=0, column=0, sticky="ew")

        harvest_modes = ttk.Frame(run_frame)
        harvest_modes.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(harvest_modes, text="시작/복귀 자세").grid(
            row=0, column=0, sticky="w", padx=(0, 8)
        )
        self.sweep_pick_ready_state_combo = ttk.Combobox(
            harvest_modes,
            textvariable=self.pick_ready_state_var,
            values=PICK_READY_STATES,
            state="readonly",
            width=20,
        )
        self.sweep_pick_ready_state_combo.grid(
            row=0, column=1, sticky="w"
        )
        self.sweep_pick_ready_state_combo.bind(
            "<<ComboboxSelected>>",
            self._pick_ready_state_changed,
        )
        self.sweep_continuous_harvest_checkbox = ttk.Checkbutton(
            harvest_modes,
            text="연속 수확: 식물 바깥 arc 경유",
            variable=self.continuous_harvest_var,
        )
        self.sweep_continuous_harvest_checkbox.grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )
        self.sweep_lift_harvest_checkbox = ttk.Checkbutton(
            harvest_modes,
            text="리프트 수확: 토마토보다 40cm 낮게",
            variable=self.lift_harvest_var,
            command=self._lift_harvest_mode_changed,
        )
        self.sweep_lift_harvest_checkbox.grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(5, 0)
        )

        self.sweep_execute_checkbox = ttk.Checkbutton(
            run_frame,
            text="실제 로봇 실행",
            variable=self.sweep_execute_motion_var,
        )
        self.sweep_execute_checkbox.grid(
            row=2, column=0, sticky="w", pady=(8, 0)
        )
        run_actions = ttk.Frame(run_frame)
        run_actions.grid(row=3, column=0, sticky="w", pady=(10, 0))
        self.sweep_start_button = ttk.Button(
            run_actions,
            text="자동 실행",
            command=self.start_sweep,
            style="Action.TButton",
        )
        self.sweep_start_button.grid(row=0, column=0, padx=(0, 6))
        self.sweep_stop_button = ttk.Button(
            run_actions,
            text="테스트 / 로봇 정지",
            command=self.stop_sweep,
            state="disabled",
            style="Action.TButton",
        )
        self.sweep_stop_button.grid(row=0, column=1)
        ttk.Label(
            run_frame,
            textvariable=self.sweep_summary,
            wraplength=650,
            justify="left",
        ).grid(row=4, column=0, sticky="ew", pady=(12, 0))

        statistics_frame = ttk.Frame(frame)
        statistics_frame.grid(row=1, column=0, sticky="ew", pady=(10, 6))
        statistics_frame.columnconfigure(0, weight=1)
        statistics_frame.columnconfigure(1, weight=1)
        statistics_frame.columnconfigure(2, weight=2)
        self.sweep_statistics = tk.StringVar(
            value="완료 0 | 성공 0 | 실패 0 | 대체성공 0 | 성공률 0.0%"
        )
        self.sweep_failure_summary = tk.StringVar(value="실패 단계: 없음")
        self.sweep_result_path = tk.StringVar(value="결과 파일: 생성 전")
        ttk.Label(
            statistics_frame,
            textvariable=self.sweep_statistics,
            font="TkHeadingFont",
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            statistics_frame,
            textvariable=self.sweep_failure_summary,
            wraplength=480,
        ).grid(row=0, column=1, sticky="w", padx=(20, 0))
        ttk.Label(
            statistics_frame,
            textvariable=self.sweep_result_path,
            anchor="e",
        ).grid(row=0, column=2, sticky="e", padx=(20, 0))

        result_frame = ttk.LabelFrame(frame, text="실시간 결과", padding=6)
        result_frame.grid(row=2, column=0, sticky="nsew")
        result_frame.columnconfigure(0, weight=1)
        result_frame.rowconfigure(0, weight=1)
        result_columns = (
            "case",
            "tomato",
            "result",
            "stage",
            "time",
            "sequence_time",
        )
        self.sweep_result_tree = ttk.Treeview(
            result_frame,
            columns=result_columns,
            show="headings",
            height=10,
            style="Sweep.Treeview",
        )
        ttk.Style(self.root).configure("Sweep.Treeview", rowheight=30)
        headings = {
            "case": "케이스",
            "tomato": "토마토",
            "result": "결과",
            "stage": "실패/복구 단계",
            "time": "시간(s)",
            "sequence_time": "실제 시퀀스(s)",
        }
        widths = {
            "case": 75,
            "tomato": 75,
            "result": 80,
            "stage": 520,
            "time": 100,
            "sequence_time": 130,
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
            result_frame,
            orient="vertical",
            command=self.sweep_result_tree.yview,
        )
        self.sweep_result_tree.configure(yscrollcommand=result_scrollbar.set)
        self.sweep_result_tree.bind(
            "<<TreeviewSelect>>",
            self._sweep_result_selected,
        )
        self.sweep_result_tree.grid(
            row=0, column=0, sticky="nsew"
        )
        result_scrollbar.grid(row=0, column=1, sticky="ns")

        detail_frame = ttk.LabelFrame(
            frame,
            text="선택/최근 결과 상세",
            padding=6,
        )
        detail_frame.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        detail_frame.columnconfigure(0, weight=1)
        detail_frame.rowconfigure(0, weight=1)
        self.sweep_result_detail = tk.Text(
            detail_frame,
            height=6,
            wrap="word",
            state="disabled",
            padx=6,
            pady=4,
        )
        detail_scrollbar = ttk.Scrollbar(
            detail_frame,
            orient="vertical",
            command=self.sweep_result_detail.yview,
        )
        self.sweep_result_detail.configure(
            yscrollcommand=detail_scrollbar.set
        )
        self.sweep_result_detail.grid(row=0, column=0, sticky="nsew")
        detail_scrollbar.grid(row=0, column=1, sticky="ns")
        self._set_sweep_result_detail("없음")
        self.replay_sweep_button = ttk.Button(
            detail_frame,
            text="선택 환경 재현",
            command=self.replay_selected_sweep_scene,
            state="disabled",
            style="Compact.TButton",
        )
        self.replay_sweep_button.grid(row=1, column=0, sticky="w", pady=(6, 0))

    def _append_log(self, message: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message.rstrip() + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    @staticmethod
    def _qualified_node_name(name: str, namespace: str) -> str:
        namespace = "/" + str(namespace).strip("/")
        if namespace == "/":
            return "/" + str(name).strip("/")
        return f"{namespace}/{str(name).strip('/')}"

    def _lift_node_is_running(self) -> bool:
        target = "/" + self.lift_node_name.strip("/")
        return target in {
            self._qualified_node_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        }

    def _linear_motor_node_is_running(self) -> bool:
        target = "/" + self.linear_motor_node_name.strip("/")
        return target in {
            self._qualified_node_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        }

    def _update_linear_motor_launch_control(self) -> None:
        launch_running = (
            self.linear_motor_launch_process is not None
            and self.linear_motor_launch_process.poll() is None
        )
        self.linear_motor_launch_button.configure(
            state=(
                "disabled"
                if self.ui_busy
                or self.linear_motor_node_online
                or launch_running
                else "normal"
            )
        )

    def _refresh_linear_motor_node_status(self) -> None:
        if self.closing:
            return
        was_online = self.linear_motor_node_online
        self.linear_motor_node_online = self._linear_motor_node_is_running()
        launch_starting = (
            self.linear_motor_launch_process is not None
            and self.linear_motor_launch_process.poll() is None
        )
        if self.linear_motor_node_online:
            self.linear_motor_node_status.set("실행 중")
            if not was_online:
                self.linear_motor_serial_status.set(
                    f"TTY 연결 확인 중: {self.linear_motor_serial_port}"
                )
                self._append_log(
                    f"[Arduino 리니어모터] 노드 연결됨: "
                    f"{self.linear_motor_node_name}"
                )
            if (
                self.linear_motor_serial_connected
                and not self.linear_motor_pin_state_initialized
            ):
                self._initialize_linear_motor_pin_state()
                self._initialize_gripper_outputs()
        else:
            self.linear_motor_node_status.set(
                "실행 시작 중" if launch_starting else "실행 안 됨"
            )
            self.linear_motor_serial_connected = False
            self.linear_motor_serial_status_received = False
            self.linear_motor_pin_state_initialized = False
            if not launch_starting:
                self.linear_motor_serial_status.set(
                    f"TTY 연결 대기: {self.linear_motor_serial_port}"
                )
            if was_online:
                self._append_log(
                    f"[Arduino 리니어모터] 노드 연결 끊김: "
                    f"{self.linear_motor_node_name}"
                )
        self._update_linear_motor_launch_control()
        self._update_gripper_stroke_controls()
        self.root.after(500, self._refresh_linear_motor_node_status)

    def _linear_motor_serial_status_callback(self, message: String) -> None:
        status = str(message.data).strip()
        self.linear_motor_serial_status_received = True
        if status == "connected":
            was_connected = self.linear_motor_serial_connected
            self.linear_motor_serial_connected = True
            self.linear_motor_serial_status.set(
                f"연결됨: {self.linear_motor_serial_port}"
            )
            if not was_connected:
                self._append_log(
                    f"[Arduino 리니어모터] TTY 연결 성공: "
                    f"{self.linear_motor_serial_port}"
                )
            self._initialize_linear_motor_pin_state()
            self._initialize_gripper_outputs()
        else:
            self.linear_motor_serial_connected = False
            self.linear_motor_pin_state_initialized = False
            self.linear_motor_serial_status.set(f"TTY 오류: {status}")
            self.gripper_stroke_status.set("Arduino TTY 연결 필요")
            self._append_log(
                f"[Arduino 리니어모터] TTY 연결 실패: {status}"
            )
        self._update_gripper_stroke_controls()

    def _publish_linear_motor_pin_levels(
        self,
        pin8_high: bool,
        pin9_high: bool,
    ) -> None:
        pin8_message = Bool()
        pin8_message.data = bool(pin8_high)
        pin9_message = Bool()
        pin9_message.data = bool(pin9_high)
        self.linear_motor_pin8_publisher.publish(pin8_message)
        self.linear_motor_pin9_publisher.publish(pin9_message)

    def send_servo10_angle(self, preset_angle=None) -> None:
        """Send a manual pin 10 servo angle through the serial bridge."""
        try:
            angle_deg = servo_angle_degrees(
                self.servo10_angle_deg_var.get()
                if preset_angle is None
                else preset_angle
            )
            speed_percent = (
                servo_speed_percent(self.servo10_speed_percent_var.get())
                if self.servo10_speed_enabled_var.get()
                else SERVO10_DEFAULT_SPEED_PERCENT
            )
        except ValueError as error:
            self.status.set(str(error))
            messagebox.showerror("PIN10 서보 각도 오류", str(error))
            return

        if not self.linear_motor_node_online:
            self.status.set("서보 각도 미전송 — Arduino 노드가 없습니다.")
            self._append_log(
                "[PIN10 서보 미전송] Arduino 노드를 먼저 실행하세요."
            )
            return
        if not self.linear_motor_serial_connected:
            self.status.set("서보 각도 미전송 — Arduino TTY 연결이 없습니다.")
            self._append_log(
                "[PIN10 서보 미전송] Arduino TTY 연결을 확인하세요."
            )
            return
        if self.count_subscribers(self.linear_motor_servo10_command_topic) < 1:
            self.status.set("서보 명령 미전송 — COMMAND 토픽 구독자가 없습니다.")
            self._append_log(
                "[PIN10 서보 미전송] serial bridge를 최신 빌드로 "
                "다시 실행하세요."
            )
            return

        message = Float64MultiArray()
        message.data = [float(angle_deg), float(speed_percent)]
        self.linear_motor_servo10_command_publisher.publish(message)
        self.servo10_angle_deg_var.set(str(angle_deg))
        speed_text = (
            "즉시"
            if speed_percent >= 100
            else f"{servo_speed_degrees_per_second(speed_percent):.1f}°/s"
        )
        self.status.set(
            f"PIN10 서보 {angle_deg}° 명령 전송 완료 ({speed_text})"
        )
        self._append_log(
            f"[PIN10 서보] 목표={angle_deg}°, 속도={speed_percent}% "
            f"({speed_text}) — 도착 후 해당 각도를 유지합니다."
        )

    def _initialize_linear_motor_pin_state(self) -> bool:
        if not self.linear_motor_serial_connected:
            return False
        if (
            self.count_subscribers(self.linear_motor_pin8_topic) < 1
            or self.count_subscribers(self.linear_motor_pin9_topic) < 1
        ):
            self.linear_motor_pin_state_initialized = False
            self.linear_motor_serial_status.set(
                "TTY 연결됨 / PIN 토픽 구독 대기"
            )
            return False
        pin8_high, pin9_high = linear_motor_pin_values("stop")
        self._publish_linear_motor_pin_levels(pin8_high, pin9_high)
        self.linear_motor_pin_state_initialized = True
        self._append_log(
            "[Arduino 리니어모터] 안전 초기화: PIN8=LOW, PIN9=LOW"
        )
        return True

    def launch_linear_motor_node(self) -> None:
        if self._linear_motor_node_is_running():
            self.linear_motor_node_online = True
            self.linear_motor_node_status.set("실행 중")
            self._update_linear_motor_launch_control()
            return
        if (
            self.linear_motor_launch_process is not None
            and self.linear_motor_launch_process.poll() is None
        ):
            return
        port = Path(self.linear_motor_serial_port)
        if not port.exists():
            message = f"TTY 장치가 없습니다: {port}"
            self.linear_motor_node_status.set("실행 실패")
            self.linear_motor_serial_status.set(message)
            self._append_log(f"[Arduino 리니어모터] {message}")
            messagebox.showerror("Arduino 노드 실행 실패", message)
            return
        if not os.access(port, os.R_OK | os.W_OK):
            message = f"TTY 읽기/쓰기 권한이 없습니다: {port}"
            self.linear_motor_node_status.set("실행 실패")
            self.linear_motor_serial_status.set(message)
            self._append_log(f"[Arduino 리니어모터] {message}")
            messagebox.showerror("Arduino 노드 실행 실패", message)
            return
        command = [
            "ros2",
            "launch",
            self.linear_motor_launch_package,
            self.linear_motor_launch_file,
            f"port:={self.linear_motor_serial_port}",
        ]
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except (OSError, ValueError) as error:
            self.linear_motor_node_status.set("실행 실패")
            self.linear_motor_serial_status.set(str(error))
            messagebox.showerror(
                "Arduino 노드 실행 실패",
                f"Arduino launch를 시작하지 못했습니다.\n{error}",
            )
            self._append_log(
                f"[Arduino 리니어모터] launch 실행 실패: {error}"
            )
            return
        self.linear_motor_launch_process = process
        self.linear_motor_node_status.set("실행 시작 중")
        self.linear_motor_serial_status.set(
            f"TTY 연결 확인 중: {self.linear_motor_serial_port}"
        )
        self._update_linear_motor_launch_control()
        self._append_log(
            "[Arduino 리니어모터] launch 실행: " + " ".join(command)
        )
        threading.Thread(
            target=self._read_linear_motor_launch_output,
            args=(process,),
            daemon=True,
        ).start()

    def _read_linear_motor_launch_output(self, process) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                line = line.rstrip()
                if line:
                    self.process_queue.put(
                        ("linear_motor_log", line, process)
                    )
        return_code = process.wait()
        self.process_queue.put(
            ("linear_motor_done", return_code, process)
        )

    def _update_lift_controls(self) -> None:
        online = bool(self.lift_node_online)
        simulation = bool(self.lift_simulation_mode)
        launch_running = (
            self.lift_launch_process is not None
            and self.lift_launch_process.poll() is None
        )
        self.lift_launch_button.configure(
            state=(
                "disabled"
                if self.ui_busy or simulation or online or launch_running
                else "normal"
            )
        )
        self.lift_calibration_button.configure(
            state=(
                "normal"
                if online
                and not simulation
                and not self.ui_busy
                and not self.lift_calibration_active
                else "disabled"
            )
        )
        height_enabled = (
            simulation or (online and self.lift_calibrated)
        ) and not self.ui_busy
        height_state = "normal" if height_enabled else "disabled"
        self.lift_target_height_entry.configure(state=height_state)
        self.lift_move_button.configure(state=height_state)
        self.lift_stop_button.configure(
            state="normal" if simulation or online else "disabled"
        )

    def _refresh_lift_node_status(self) -> None:
        if self.closing:
            return
        if self.lift_simulation_mode:
            self.lift_node_online = False
            self.lift_node_status.set("시뮬레이션 (RViz)")
            self.lift_calibration_active = False
            self.lift_calibrated = True
            self.lift_calibration_status.set("Calibration 불필요")
            self._update_lift_controls()
            self.root.after(500, self._refresh_lift_node_status)
            return
        was_online = self.lift_node_online
        self.lift_node_online = self._lift_node_is_running()
        launch_starting = (
            self.lift_launch_process is not None
            and self.lift_launch_process.poll() is None
        )
        if self.lift_node_online:
            self.lift_node_status.set("실행 중")
            if not was_online:
                if not self.lift_calibrated:
                    self.lift_calibration_status.set(
                        "Bottom calibration 필요"
                    )
                self._append_log(
                    f"[리프트] 노드 연결됨: {self.lift_node_name}"
                )
        else:
            self.lift_node_status.set(
                "실행 시작 중" if launch_starting else "실행 안 됨"
            )
            self.lift_calibration_active = False
            self.lift_calibrated = False
            self.lift_last_height_mm = None
            self.lift_current_height.set("-- mm")
            self.lift_calibration_status.set(
                "노드 시작 대기" if launch_starting else "노드 실행 필요"
            )
            if was_online:
                self._append_log(
                    f"[리프트] 노드 연결 끊김: {self.lift_node_name}"
                )
        self._update_lift_controls()
        self.root.after(500, self._refresh_lift_node_status)

    def _lift_current_height_callback(self, message: Float64) -> None:
        if self.lift_simulation_mode:
            return
        height_mm = float(message.data)
        if not math.isfinite(height_mm):
            return
        self.lift_last_height_mm = height_mm
        self.lift_current_height.set(f"{height_mm:.2f} mm")
        if not self.lift_calibration_active:
            self.lift_calibrated = True
            self.lift_calibration_status.set("Calibration 완료")
        self._update_lift_controls()

    def _lift_simulated_height_callback(self, message: Float64) -> None:
        if not self.lift_simulation_mode:
            return
        height_mm = float(message.data)
        if not math.isfinite(height_mm):
            return
        self.lift_last_height_mm = height_mm
        self.lift_current_height.set(f"{height_mm:.2f} mm")
        self._update_lift_controls()

    def _lift_simulation_mode_callback(self, message: Bool) -> None:
        simulation = bool(message.data)
        if simulation == self.lift_simulation_mode:
            return
        self.lift_simulation_mode = simulation
        if simulation:
            self.lift_node_online = False
            self.lift_calibration_active = False
            self.lift_calibrated = True
            self.lift_node_status.set("시뮬레이션 (RViz)")
            self.lift_calibration_status.set("Calibration 불필요")
            self._append_log(
                "[리프트] 시뮬레이션 모드: 실제 리프트 명령 없이 "
                "RViz 모델 높이만 변경합니다."
            )
        else:
            self.lift_calibrated = False
            self.lift_current_height.set("-- mm")
            self.lift_calibration_status.set("Bottom calibration 필요")
            self._append_log(
                "[리프트] 실제 모드: lift_controller_node와 Bottom "
                "calibration이 필요합니다."
            )
        self._update_lift_controls()

    def _lift_bottom_status_callback(self, message: Bool) -> None:
        if self.lift_simulation_mode:
            return
        self.lift_calibration_active = False
        self.lift_calibrated = bool(message.data)
        if message.data:
            self.lift_calibration_status.set("Calibration 완료")
            self._append_log(
                "[리프트] Bottom calibration 완료 및 10 mm 후퇴 완료"
            )
        else:
            self.lift_calibration_status.set("Calibration 실패")
            self._append_log(
                "[리프트] Bottom calibration 실패 — 리프트 노드 로그를 "
                "확인하세요."
            )
        self._update_lift_controls()

    def launch_lift_node(self) -> None:
        if self.lift_simulation_mode:
            self._append_log(
                "[리프트] 시뮬레이션에서는 실제 리프트 노드를 실행하지 "
                "않습니다."
            )
            return
        if self._lift_node_is_running():
            self.lift_node_online = True
            self.lift_node_status.set("실행 중")
            self._update_lift_controls()
            return
        if (
            self.lift_launch_process is not None
            and self.lift_launch_process.poll() is None
        ):
            return
        command = [
            "ros2",
            "launch",
            self.lift_launch_package,
            self.lift_launch_file,
        ]
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except (OSError, ValueError) as error:
            messagebox.showerror(
                "리프트 노드 실행 실패",
                f"리프트 launch를 시작하지 못했습니다.\n{error}",
            )
            self._append_log(f"[리프트] launch 실행 실패: {error}")
            return
        self.lift_launch_process = process
        self.lift_node_status.set("실행 시작 중")
        self._update_lift_controls()
        self._append_log(
            "[리프트] launch 실행: " + " ".join(command)
        )
        threading.Thread(
            target=self._read_lift_launch_output,
            args=(process,),
            daemon=True,
        ).start()

    def _read_lift_launch_output(self, process) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                line = line.rstrip()
                if line:
                    self.process_queue.put(("lift_log", line, process))
        return_code = process.wait()
        self.process_queue.put(("lift_done", return_code, process))

    def start_lift_bottom_calibration(self) -> None:
        if self.lift_simulation_mode:
            self._append_log(
                "[리프트] 시뮬레이션에서는 Bottom calibration이 필요하지 "
                "않습니다."
            )
            return
        if not self.lift_node_online or self.count_subscribers(
            self.lift_bottom_calibration_topic
        ) < 1:
            messagebox.showerror(
                "리프트 연결 오류",
                "리프트 노드가 calibration 명령을 구독하고 있지 않습니다.",
            )
            return
        if not messagebox.askyesno(
            "Bottom calibration 실행",
            "리프트가 Bottom limit 방향으로 실제 이동합니다.\n"
            "주변이 안전하고 CAN 연결이 정상인지 확인했습니까?",
            icon="warning",
        ):
            return
        message = Bool()
        message.data = True
        self.lift_bottom_calibration_publisher.publish(message)
        self.lift_calibration_active = True
        self.lift_calibrated = False
        self.lift_calibration_status.set("Calibration 진행 중")
        self._update_lift_controls()
        self._append_log(
            f"[리프트] Bottom calibration 명령 발행: "
            f"{self.lift_bottom_calibration_topic}"
        )

    @staticmethod
    def _parse_lift_height(value: str, maximum_mm: float = 750.0) -> float:
        try:
            height_mm = float(value)
        except ValueError as error:
            raise ValueError("리프트 목표 높이는 mm 단위 숫자로 입력하세요.") from error
        if not math.isfinite(height_mm) or height_mm < 0.0:
            raise ValueError("리프트 목표 높이는 0 mm 이상이어야 합니다.")
        if height_mm > float(maximum_mm):
            raise ValueError(
                f"리프트 목표 높이는 {float(maximum_mm):.0f} mm 이하여야 합니다."
            )
        return height_mm

    def _tomato_world_height_m(self, tomato_index: int) -> float:
        tomato_frame = self._tomato_frame(tomato_index)
        transform = self.tf_buffer.lookup_transform(
            self.lift_harvest_world_frame,
            tomato_frame,
            Time(),
        )
        return float(transform.transform.translation.z)

    def _selected_angle_reference_mode(self) -> str:
        label = str(self.angle_reference_mode_var.get())
        try:
            return ANGLE_REFERENCE_MODE_BY_LABEL[label]
        except KeyError as error:
            raise ValueError(
                f"지원하지 않는 진입각 기준입니다: {label}"
            ) from error

    def _robot_base_origin_in_frame(self, target_frame: str) -> Point:
        base_frame = str(
            self.get_parameter("angle_reference_robot_base_frame").value
        )
        if target_frame == base_frame:
            return Point(x=0.0, y=0.0, z=0.0)
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            base_frame,
            Time(),
        )
        translation = transform.transform.translation
        return Point(
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
        )

    def _cache_detected_tomato_world_positions(
        self,
        message: TomatoDetectionArray,
    ) -> None:
        """Snapshot detection centers in world before the lift starts moving."""
        source_frame = str(message.header.frame_id)
        world_frame = self.lift_harvest_world_frame
        if source_frame == world_frame:
            transform = None
        else:
            transform = self.tf_buffer.lookup_transform(
                world_frame,
                source_frame,
                Time(),
            )
        positions = {}
        x_axes = {}
        mode = self._selected_angle_reference_mode()
        base_world = (
            self._robot_base_origin_in_frame(world_frame)
            if mode == ANGLE_REFERENCE_BASE_TO_CENTER
            else None
        )
        for index, detection in enumerate(message.detections):
            center = detection.center
            angle_origin, angle_target = detection_angle_segment(
                detection,
                mode,
                base_point=base_world,
            )
            if transform is None:
                center_world = (
                    float(center.x),
                    float(center.y),
                    float(center.z),
                )
                angle_origin_world = (
                    float(angle_origin.x),
                    float(angle_origin.y),
                    float(angle_origin.z),
                )
                angle_target_world = (
                    float(angle_target.x),
                    float(angle_target.y),
                    float(angle_target.z),
                )
            else:
                center_world = transformed_point_xyz(center, transform)
                angle_origin_world = (
                    (
                        float(base_world.x),
                        float(base_world.y),
                        float(base_world.z),
                    )
                    if mode == ANGLE_REFERENCE_BASE_TO_CENTER
                    else transformed_point_xyz(angle_origin, transform)
                )
                angle_target_world = transformed_point_xyz(
                    angle_target,
                    transform,
                )
            positions[index] = center_world
            horizontal_x = angle_target_world[0] - angle_origin_world[0]
            horizontal_y = angle_target_world[1] - angle_origin_world[1]
            horizontal_norm = math.hypot(horizontal_x, horizontal_y)
            if horizontal_norm > 1e-9:
                x_axes[index] = (
                    horizontal_x / horizontal_norm,
                    horizontal_y / horizontal_norm,
                )
        self.detected_tomato_expected_world_positions = positions
        self.detected_tomato_expected_world_x_axes = x_axes

    def _cache_detected_tomato_record_positions(
        self,
        message: TomatoDetectionArray,
    ) -> None:
        """Snapshot exact center/stem coordinates for RViz-matching records."""
        source_frame = str(message.header.frame_id)
        target_frame = str(
            self.get_parameter("camera_target_record_robot_frame").value
        )
        if source_frame == target_frame:
            transform = None
        else:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
        centers = {}
        stems = {}
        calyxes = {}
        angle_origins = {}
        angle_targets = {}
        mode = self._selected_angle_reference_mode()
        base_target = (
            self._robot_base_origin_in_frame(target_frame)
            if mode == ANGLE_REFERENCE_BASE_TO_CENTER
            else None
        )
        for index, detection in enumerate(message.detections):
            angle_origin, angle_target = detection_angle_segment(
                detection,
                mode,
                base_point=base_target,
            )
            if transform is None:
                centers[index] = (
                    float(detection.center.x),
                    float(detection.center.y),
                    float(detection.center.z),
                )
                stems[index] = (
                    float(detection.stem_point.x),
                    float(detection.stem_point.y),
                    float(detection.stem_point.z),
                )
                calyxes[index] = (
                    float(detection.calyx_point.x),
                    float(detection.calyx_point.y),
                    float(detection.calyx_point.z),
                )
                angle_origins[index] = (
                    float(angle_origin.x),
                    float(angle_origin.y),
                    float(angle_origin.z),
                )
                angle_targets[index] = (
                    float(angle_target.x),
                    float(angle_target.y),
                    float(angle_target.z),
                )
            else:
                centers[index] = transformed_point_xyz(
                    detection.center,
                    transform,
                )
                stems[index] = transformed_point_xyz(
                    detection.stem_point,
                    transform,
                )
                calyxes[index] = transformed_point_xyz(
                    detection.calyx_point,
                    transform,
                )
                angle_origins[index] = (
                    (
                        float(base_target.x),
                        float(base_target.y),
                        float(base_target.z),
                    )
                    if mode == ANGLE_REFERENCE_BASE_TO_CENTER
                    else transformed_point_xyz(angle_origin, transform)
                )
                angle_targets[index] = transformed_point_xyz(
                    angle_target,
                    transform,
                )
        self.detected_tomato_record_positions = centers
        self.detected_tomato_record_stem_positions = stems
        self.detected_tomato_record_calyx_positions = calyxes
        self.detected_tomato_record_angle_origins = angle_origins
        self.detected_tomato_record_angle_targets = angle_targets

    def _sort_detection_message_by_height(
        self,
        message: TomatoDetectionArray,
    ) -> TomatoDetectionArray:
        """Mirror the TF generator's cluster-priority ID assignment."""
        source_frame = str(message.header.frame_id)
        world_frame = self.lift_harvest_world_frame
        transform = None
        if source_frame != world_frame:
            transform = self.tf_buffer.lookup_transform(
                world_frame,
                source_frame,
                Time(),
            )
        return detection_message_sorted_by_height(message, transform)

    def _detected_tomato_tf_sync_error_m(
        self,
        tomato_index: int,
    ) -> float | None:
        expected = self.detected_tomato_expected_world_positions.get(
            int(tomato_index)
        )
        if expected is None:
            return None
        tomato_frame = self._tomato_frame(tomato_index)
        transform = self.tf_buffer.lookup_transform(
            self.lift_harvest_world_frame,
            tomato_frame,
            Time(),
        )
        translation = transform.transform.translation
        return math.sqrt(
            (float(translation.x) - expected[0]) ** 2
            + (float(translation.y) - expected[1]) ** 2
            + (float(translation.z) - expected[2]) ** 2
        )

    def _detected_tomato_tf_orientation_error_deg(
        self,
        tomato_index: int,
    ) -> float | None:
        expected = self.detected_tomato_expected_world_x_axes.get(
            int(tomato_index)
        )
        if expected is None:
            return None
        tomato_frame = self._tomato_frame(tomato_index)
        transform = self.tf_buffer.lookup_transform(
            self.lift_harvest_world_frame,
            tomato_frame,
            Time(),
        )
        rotation = transform.transform.rotation
        qx = float(rotation.x)
        qy = float(rotation.y)
        qz = float(rotation.z)
        qw = float(rotation.w)
        actual_x = 1.0 - 2.0 * (qy * qy + qz * qz)
        actual_y = 2.0 * (qx * qy + qz * qw)
        actual_norm = math.hypot(actual_x, actual_y)
        if actual_norm <= 1e-9:
            return None
        dot = (
            expected[0] * actual_x / actual_norm
            + expected[1] * actual_y / actual_norm
        )
        return math.degrees(math.acos(max(-1.0, min(1.0, dot))))

    @staticmethod
    def _stamp_nanoseconds(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _detected_tf_ready_callback(self, message: Header) -> None:
        self.detected_tf_ready_stamp_ns = self._stamp_nanoseconds(
            message.stamp
        )

    def _detected_tf_positions_are_synchronized(self) -> tuple[bool, str]:
        """Verify that every reused detected TF belongs to this snapshot."""
        tomato_count = len(self.detected_tomatoes)
        expected_count = len(self.detected_tomato_expected_world_positions)
        expected_axis_count = len(
            self.detected_tomato_expected_world_x_axes
        )
        if (
            expected_count != tomato_count
            or expected_axis_count != tomato_count
        ):
            return (
                False,
                f"검출 중심/방향 world 좌표 {expected_count}/"
                f"{expected_axis_count}/{tomato_count}개 준비",
            )

        tolerance_m = max(0.0, self.detected_tf_sync_tolerance_m)
        orientation_tolerance_deg = max(
            0.0,
            self.detected_tf_sync_orientation_tolerance_deg,
        )
        for index in range(tomato_count):
            frame_reader = getattr(self, "_tomato_frame", None)
            tomato_frame = (
                frame_reader(index)
                if frame_reader is not None
                else f"detected_tomato_{index}_tf"
            )
            try:
                error_m = self._detected_tomato_tf_sync_error_m(index)
                orientation_error_deg = (
                    self._detected_tomato_tf_orientation_error_deg(index)
                )
            except TransformException as error:
                return False, f"{tomato_frame} 조회 대기: {error}"
            if error_m is None:
                return False, f"토마토 {index}의 검출 중심 좌표 없음"
            if error_m > tolerance_m:
                return (
                    False,
                    f"{tomato_frame} 위치 오차 "
                    f"{error_m * 1000.0:.1f}mm > "
                    f"{tolerance_m * 1000.0:.1f}mm",
                )
            if orientation_error_deg is None:
                return False, f"{tomato_frame} 방향 비교 불가"
            if orientation_error_deg > orientation_tolerance_deg:
                return (
                    False,
                    f"{tomato_frame} X축 오차 "
                    f"{orientation_error_deg:.1f}° > "
                    f"{orientation_tolerance_deg:.1f}°",
                )
        return True, ""

    def _detected_tf_generation_is_ready(self) -> bool:
        """Return whether the TF generator finished the current detection."""
        expected_stamp = int(
            getattr(self, "current_detection_stamp_ns", 0)
        )
        topic = str(getattr(self, "detected_tf_ready_topic", ""))
        if expected_stamp <= 0 or not topic:
            return True
        count_publishers = getattr(self, "count_publishers", None)
        if count_publishers is None or count_publishers(topic) < 1:
            # Backward compatibility with an external/older TF generator.
            return True
        return int(getattr(self, "detected_tf_ready_stamp_ns", 0)) == expected_stamp

    def _publish_automatic_lift_target(self, height_mm: float) -> None:
        message = Float64()
        message.data = float(height_mm)
        if self.lift_simulation_mode:
            topic = self.lift_simulated_move_height_topic
            if self.count_subscribers(topic) < 1:
                raise RuntimeError(
                    "RViz lift joint 시뮬레이터가 실행 중이지 않습니다."
                )
            self.lift_simulated_move_height_publisher.publish(message)
            return
        if not self.lift_node_online:
            raise RuntimeError("실제 리프트 노드가 연결되지 않았습니다.")
        if not self.lift_calibrated:
            raise RuntimeError("Bottom calibration이 완료되지 않았습니다.")
        if self.count_subscribers(self.lift_move_height_topic) < 1:
            raise RuntimeError(
                "리프트 노드가 높이 이동 명령을 구독하고 있지 않습니다."
            )
        self.lift_move_height_publisher.publish(message)

    def _prepare_lift_for_tomato(
        self,
        tomato_index: int,
        enabled: bool,
        on_ready,
        on_error,
        tf_deadline: float | None = None,
    ) -> None:
        """Move to the tomato-relative lift target, then continue asynchronously."""
        if not enabled:
            self.root.after(0, on_ready)
            return
        deadline = (
            time.monotonic()
            + max(0.1, float(self.detected_tf_sync_timeout_sec))
            if tf_deadline is None
            else float(tf_deadline)
        )
        try:
            if not self._detected_tf_generation_is_ready():
                if time.monotonic() < deadline:
                    self.root.after(
                        50,
                        lambda: self._prepare_lift_for_tomato(
                            tomato_index,
                            enabled,
                            on_ready,
                            on_error,
                            deadline,
                        ),
                    )
                    return
                on_error(
                    "새 검출 결과에 대한 토마토 TF 생성 완료 신호를 "
                    f"{self.detected_tf_sync_timeout_sec:.1f}초 안에 "
                    "받지 못했습니다."
                )
                return
            tf_sync_error_m = self._detected_tomato_tf_sync_error_m(
                tomato_index
            )
            tf_sync_tolerance_m = max(
                0.0,
                self.detected_tf_sync_tolerance_m,
            )
            if (
                tf_sync_error_m is not None
                and tf_sync_error_m > tf_sync_tolerance_m
            ):
                if time.monotonic() < deadline:
                    self.root.after(
                        50,
                        lambda: self._prepare_lift_for_tomato(
                            tomato_index,
                            enabled,
                            on_ready,
                            on_error,
                            deadline,
                        ),
                    )
                    return
                on_error(
                    f"{self._tomato_frame(tomato_index)}가 새 검출 위치로 "
                    "갱신되지 않았습니다: "
                    f"위치 오차={tf_sync_error_m * 1000.0:.1f}mm, "
                    f"허용={tf_sync_tolerance_m * 1000.0:.1f}mm"
                )
                return
            tomato_world_z_m = self._tomato_world_height_m(tomato_index)
            requested_mm = (
                tomato_world_z_m - self.lift_harvest_offset_m
            ) * 1000.0
            target_mm = lift_harvest_target_height_mm(
                tomato_world_z_m,
                self.lift_harvest_offset_m,
                self.lift_minimum_height_mm,
                self.lift_maximum_height_mm,
            )
        except TransformException as error:
            if time.monotonic() < deadline:
                self.root.after(
                    100,
                    lambda: self._prepare_lift_for_tomato(
                        tomato_index,
                        enabled,
                        on_ready,
                        on_error,
                        deadline,
                    ),
                )
                return
            on_error(
                f"{self._tomato_frame(tomato_index)}의 지면 기준 높이를 "
                f"계산하지 못했습니다: {error}"
            )
            return
        except ValueError as error:
            on_error(f"리프트 수확 높이 계산 오류: {error}")
            return

        clamped = not math.isclose(requested_mm, target_mm, abs_tol=1e-6)
        self.lift_target_height.set(f"{target_mm:.1f}")
        self._append_log(
            f"[리프트 수확] 토마토 {tomato_index}: "
            f"world Z={tomato_world_z_m:.3f}m, "
            f"요청={requested_mm:.1f}mm, 목표={target_mm:.1f}mm"
            f"{' (0~750mm 제한 적용)' if clamped else ''}"
        )
        tolerance = max(0.0, self.lift_target_tolerance_mm)
        if (
            self.lift_last_height_mm is not None
            and abs(self.lift_last_height_mm - target_mm) <= tolerance
        ):
            self._append_log(
                f"[리프트 수확] 현재 높이 {self.lift_last_height_mm:.1f}mm가 "
                f"목표 허용오차 ±{tolerance:.1f}mm 안에 있어 이동을 생략합니다."
            )
            self.root.after(0, on_ready)
            return

        try:
            self._publish_automatic_lift_target(target_mm)
        except RuntimeError as error:
            on_error(str(error))
            return

        pending = {
            "tomato_index": int(tomato_index),
            "target_mm": float(target_mm),
            "deadline": time.monotonic() + max(1.0, self.lift_move_timeout_sec),
            "on_ready": on_ready,
            "on_error": on_error,
        }
        self.lift_harvest_pending = pending
        mode = "RViz 시뮬레이션" if self.lift_simulation_mode else "실제 리프트"
        self.status.set(
            f"{mode} 이동 중: 토마토 {tomato_index}, 목표 {target_mm:.1f}mm"
        )
        self._append_log(
            f"[리프트 수확] {mode} 이동 명령 전송 — 완료 확인 후 수확 계획 시작"
        )
        self.root.after(100, lambda: self._poll_lift_harvest_target(pending))

    def _poll_lift_harvest_target(self, pending) -> None:
        if self.lift_harvest_pending is not pending:
            return
        target_mm = float(pending["target_mm"])
        tolerance = max(0.0, self.lift_target_tolerance_mm)
        current_mm = self.lift_last_height_mm
        if current_mm is not None and abs(current_mm - target_mm) <= tolerance:
            self.lift_harvest_pending = None
            self._append_log(
                f"[리프트 수확] 목표 도달: 현재={current_mm:.1f}mm, "
                f"목표={target_mm:.1f}mm — 수확 계획을 계속합니다."
            )
            pending["on_ready"]()
            return
        if time.monotonic() >= float(pending["deadline"]):
            self.lift_harvest_pending = None
            stop_message = Bool()
            stop_message.data = True
            publisher = (
                self.lift_simulated_stop_publisher
                if self.lift_simulation_mode
                else self.lift_stop_publisher
            )
            publisher.publish(stop_message)
            current_text = "수신 없음" if current_mm is None else f"{current_mm:.1f}mm"
            pending["on_error"](
                f"리프트 목표 도달 시간 초과: 목표={target_mm:.1f}mm, "
                f"현재={current_text}"
            )
            return
        self.root.after(100, lambda: self._poll_lift_harvest_target(pending))

    def move_lift_to_height(self) -> None:
        try:
            height_mm = self._parse_lift_height(
                self.lift_target_height.get(),
                getattr(self, "lift_maximum_height_mm", 750.0),
            )
        except ValueError as error:
            messagebox.showerror("리프트 높이 입력 오류", str(error))
            return
        if self.lift_simulation_mode:
            if self.count_subscribers(
                self.lift_simulated_move_height_topic
            ) < 1:
                messagebox.showerror(
                    "리프트 시뮬레이션 연결 오류",
                    "RViz lift joint 시뮬레이터가 실행 중이지 않습니다.",
                )
                return
            message = Float64()
            message.data = height_mm
            self.lift_simulated_move_height_publisher.publish(message)
            self._append_log(
                f"[리프트 시뮬레이션] RViz 목표 높이={height_mm:.2f} mm "
                "(실제 모터 명령 없음)"
            )
            self.status.set(
                f"RViz 리프트 높이 {height_mm:.2f} mm 적용 — 실제 모터 미동작"
            )
            return
        if not self.lift_node_online or not self.lift_calibrated:
            messagebox.showerror(
                "리프트 이동 불가",
                "리프트 노드 연결과 Bottom calibration을 먼저 완료하세요.",
            )
            return
        if self.count_subscribers(self.lift_move_height_topic) < 1:
            messagebox.showerror(
                "리프트 연결 오류",
                "리프트 노드가 높이 이동 명령을 구독하고 있지 않습니다.",
            )
            return
        if not messagebox.askyesno(
            "리프트 높이 이동",
            f"리프트를 Bottom 기준 {height_mm:.2f} mm 높이로 이동할까요?",
            icon="warning",
        ):
            return
        message = Float64()
        message.data = height_mm
        self.lift_move_height_publisher.publish(message)
        self._append_log(
            f"[리프트] 높이 이동 명령 발행: 목표={height_mm:.2f} mm, "
            f"topic={self.lift_move_height_topic}"
        )
        self.status.set(f"리프트 목표 높이 {height_mm:.2f} mm 이동 명령 전송")

    def stop_lift_motion(self) -> None:
        """Stop lift motion without waiting for a confirmation dialog."""
        simulation = self.lift_simulation_mode
        topic = (
            self.lift_simulated_stop_topic if simulation else self.lift_stop_topic
        )
        publisher = (
            self.lift_simulated_stop_publisher
            if simulation
            else self.lift_stop_publisher
        )
        if (not simulation and not self.lift_node_online) or self.count_subscribers(
            topic
        ) < 1:
            messagebox.showerror(
                "리프트 연결 오류",
                "리프트 노드가 정지 명령을 구독하고 있지 않습니다.",
            )
            return
        calibration_was_active = self.lift_calibration_active
        message = Bool()
        message.data = True
        publisher.publish(message)
        self.lift_calibration_active = False
        if calibration_was_active and not simulation:
            self.lift_calibrated = False
            self.lift_calibration_status.set("Calibration 중지됨")
        self._update_lift_controls()
        self._append_log(
            (
                "[리프트 시뮬레이션] 현재 RViz 높이 유지"
                if simulation
                else f"[리프트] 이동 정지 명령 발행: {topic}"
            )
        )
        self.status.set(
            "RViz 리프트 높이 유지 — 실제 모터 미동작"
            if simulation
            else "리프트 이동 정지 명령 전송"
        )

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

    def _refresh_gripper_stroke_status(self) -> None:
        """Keep the manual gripper control's hardware availability visible."""
        if self.closing:
            return
        service_ready = self.hardware_eval_client.service_is_ready()
        if service_ready:
            if not self.gripper_service_connected:
                self.gripper_service_connected = True
                self.gripper_outputs_initialized = False
                self.gripper_stroke_status.set("DOUT 안전 초기화 대기")
            if not self.linear_motor_node_online:
                self.gripper_stroke_status.set("Arduino 노드 실행 필요")
            elif not self.linear_motor_serial_connected:
                self.gripper_stroke_status.set("Arduino TTY 연결 필요")
            elif not self.linear_motor_pin_state_initialized:
                self.gripper_stroke_status.set("PIN8/9 안전 초기화 대기")
            elif not self.gripper_power_requested:
                self.gripper_stroke_status.set(
                    "전원 OFF (DOUT0·DOUT8 LOW)"
                )
            elif (
                not self.gripper_outputs_initialized
                and not self.gripper_command_in_progress
            ):
                self._initialize_gripper_outputs()
        else:
            if (
                self.gripper_service_connected
                or self.gripper_outputs_initialized
                or self.gripper_command_in_progress
            ):
                self.gripper_stroke_request_id += 1
            self.gripper_service_connected = False
            self.gripper_outputs_initialized = False
            self.gripper_command_in_progress = False
            self.gripper_stroke_status.set(
                "하드웨어 서비스 연결 안 됨"
            )
        self._update_gripper_stroke_controls()
        self.root.after(1000, self._refresh_gripper_stroke_status)

    def _update_gripper_stroke_controls(self) -> None:
        service_ready = self.hardware_eval_client.service_is_ready()
        enabled = (
            service_ready
            and self.gripper_outputs_initialized
            and self.gripper_power_requested
            and self.linear_motor_node_online
            and self.linear_motor_serial_connected
            and self.linear_motor_pin_state_initialized
            and not self.gripper_command_in_progress
        )
        state = "normal" if enabled else "disabled"
        for button in (
            self.gripper_extend_button,
            self.gripper_retract_button,
            self.gripper_stop_button,
        ):
            button.configure(state=state)
        power_on_enabled = (
            service_ready
            and not self.gripper_command_in_progress
            and not self.gripper_outputs_initialized
            and self.linear_motor_node_online
            and self.linear_motor_serial_connected
            and self.linear_motor_pin_state_initialized
        )
        power_off_enabled = (
            service_ready
            and not self.gripper_command_in_progress
        )
        self.gripper_power_on_button.configure(
            state="normal" if power_on_enabled else "disabled"
        )
        self.gripper_power_off_button.configure(
            state="normal" if power_off_enabled else "disabled"
        )
        servo_enabled = (
            self.linear_motor_node_online
            and self.linear_motor_serial_connected
            and self.count_subscribers(
                self.linear_motor_servo10_command_topic
            ) >= 1
        )
        servo_state = "normal" if servo_enabled else "disabled"
        for button in (
            self.servo10_angle_button,
            self.servo10_close_button,
            self.servo10_open_button,
        ):
            button.configure(state=servo_state)

    def _initialize_gripper_outputs(self) -> None:
        """Set direction LOW before applying power to the relay outputs."""
        if (
            self.gripper_outputs_initialized
            or self.gripper_command_in_progress
            or not self.gripper_power_requested
            or not self.hardware_eval_client.service_is_ready()
            or not self.linear_motor_serial_connected
            or not self.linear_motor_pin_state_initialized
        ):
            return
        self.gripper_stroke_request_id += 1
        request_id = self.gripper_stroke_request_id
        self.gripper_command_in_progress = True
        self.gripper_stroke_status.set(
            "안전 초기화: DOUT0·DOUT8 HIGH"
        )
        self._update_gripper_stroke_controls()
        self._append_log(
            "[그리퍼 DOUT 초기화] DOUT0=HIGH, DOUT8=HIGH, "
            "나머지 DOUT=LOW로 적용합니다."
        )
        self._send_gripper_script_sequence(
            gripper_output_startup_scripts(),
            request_id=request_id,
            label="안전 초기화",
            output_description=(
                "DOUT0=HIGH, DOUT8=HIGH, 나머지 DOUT=LOW"
            ),
            initialization=True,
        )

    def control_gripper_power(self, enabled: bool) -> None:
        """Turn the RB relay supply on or off using DOUT0 and DOUT8."""
        enabled = bool(enabled)
        if not self.hardware_eval_client.service_is_ready():
            if not self.hardware_eval_client.wait_for_service(
                timeout_sec=0.05
            ):
                self.gripper_stroke_status.set(
                    "하드웨어 서비스 연결 안 됨"
                )
                self._append_log(
                    "[그리퍼 전원 미적용] rbpodo eval 서비스가 없습니다."
                )
                return
        if self.gripper_command_in_progress:
            self.status.set("이전 그리퍼 출력 명령 처리 중입니다.")
            return
        if enabled and (
            not self.linear_motor_node_online
            or not self.linear_motor_serial_connected
            or not self.linear_motor_pin_state_initialized
        ):
            self.gripper_stroke_status.set(
                "전원 ON 불가 — Arduino PIN 안전 초기화 필요"
            )
            self._append_log(
                "[그리퍼 전원 ON 보류] Arduino 노드·TTY·PIN8/9 "
                "안전 초기화를 먼저 완료하세요."
            )
            return

        self.gripper_stroke_request_id += 1
        request_id = self.gripper_stroke_request_id
        self.gripper_command_in_progress = True
        label = "전원 ON" if enabled else "전원 OFF"
        script = (
            gripper_relay_power_script()
            if enabled
            else gripper_relay_power_off_script()
        )
        self.gripper_stroke_status.set(f"{label} 명령 전송 중")
        self.status.set(f"그리퍼 {label} 명령 전송 중...")
        self._update_gripper_stroke_controls()
        self._append_log(
            f"[그리퍼 {label}] "
            + (
                "DOUT0=HIGH, DOUT8=HIGH"
                if enabled
                else "DOUT0=LOW, DOUT8=LOW"
            )
        )
        request = Eval.Request()
        request.script = script
        future = self.hardware_eval_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._gripper_power_command_completed(
                completed,
                request_id=request_id,
                enabled=enabled,
                script=script,
            )
        )

    def _gripper_power_command_completed(
        self,
        future,
        *,
        request_id: int,
        enabled: bool,
        script: str,
    ) -> None:
        if request_id != self.gripper_stroke_request_id:
            return
        success = False
        try:
            response = future.result()
            success = bool(response.success)
        except Exception as error:
            self._append_log(
                f"[그리퍼 전원 실패] 서비스 호출 오류: {error}"
            )
        self.gripper_command_in_progress = False
        label = "ON" if enabled else "OFF"
        if not success:
            self.gripper_stroke_status.set(f"전원 {label} 실패")
            self.status.set(f"그리퍼 전원 {label} 실패")
            self._append_log(
                f"[그리퍼 전원 실패] RB 컨트롤러가 거부: {script}"
            )
            self._update_gripper_stroke_controls()
            return

        self.gripper_power_requested = enabled
        self.gripper_outputs_initialized = enabled
        if enabled:
            self.gripper_stroke_status.set(
                "제어 준비 (DOUT0·8 HIGH / Arduino PIN)"
            )
            self.status.set("그리퍼 전원 ON 완료")
            description = "DOUT0=HIGH, DOUT8=HIGH"
        else:
            self.gripper_stroke_status.set(
                "전원 OFF (DOUT0·DOUT8 LOW)"
            )
            self.status.set("그리퍼 전원 OFF 완료")
            description = "DOUT0=LOW, DOUT8=LOW"
        self._append_log(f"[그리퍼 전원 {label} 완료] {description}")
        self._update_gripper_stroke_controls()

    def control_gripper_stroke(self, command: str) -> None:
        """Drive the gripper linear actuator through RB control-box DOUTs."""
        labels = {
            "extend": ("늘림", "PIN8=HIGH, PIN9=LOW"),
            "retract": ("줄임", "PIN8=LOW, PIN9=HIGH"),
            "stop": ("정지", "PIN8=LOW, PIN9=LOW"),
        }
        try:
            label, output_description = labels[command]
            pin_sequence = linear_motor_pin_sequence(command)
        except (KeyError, ValueError) as error:
            self._append_log(
                f"[그리퍼 스트로크] 잘못된 명령: {error}"
            )
            return

        if not self.hardware_eval_client.service_is_ready():
            service_ready = self.hardware_eval_client.wait_for_service(
                timeout_sec=0.05
            )
            if not service_ready:
                self.gripper_stroke_status.set(
                    "하드웨어 서비스 연결 안 됨"
                )
                self.status.set(
                    "그리퍼 스트로크 미적용 — "
                    "실제 로봇 하드웨어 연결 안 됨"
                )
                self._append_log(
                    "[그리퍼 스트로크 미적용] rbpodo eval 서비스가 "
                    "없습니다. 시뮬레이션에서는 DOUT이 출력되지 "
                    "않습니다. "
                    f"서비스={self.hardware_eval_service}"
                )
                return

        if not self.gripper_outputs_initialized:
            self.gripper_stroke_status.set("DOUT 안전 초기화 필요")
            self.status.set(
                "그리퍼 명령 보류 — DOUT 안전 초기화가 완료되지 않았습니다."
            )
            self._append_log(
                f"[그리퍼 스트로크 보류] {label}: 안전 초기화 후 "
                "다시 실행하세요."
            )
            self._initialize_gripper_outputs()
            return
        if (
            not self.linear_motor_serial_connected
            or not self.linear_motor_pin_state_initialized
        ):
            self.gripper_stroke_status.set("Arduino PIN 제어 준비 안 됨")
            self.status.set(
                "그리퍼 명령 보류 — Arduino TTY/PIN 연결을 확인하세요."
            )
            return
        if self.gripper_command_in_progress:
            self.status.set("이전 그리퍼 DOUT 명령 처리 중입니다.")
            return

        self.gripper_stroke_request_id += 1
        request_id = self.gripper_stroke_request_id
        self.gripper_command_in_progress = True
        self.gripper_stroke_status.set(f"{label} 명령 전송 중")
        self._update_gripper_stroke_controls()
        self.status.set(f"그리퍼 스트로크 {label} 명령 전송 중...")
        self._append_log(
            f"[그리퍼 스트로크 요청] {label}: 먼저 PIN8/9 LOW, "
            f"이후 {output_description}"
        )
        self._start_linear_motor_pin_sequence(
            pin_sequence,
            command=command,
            request_id=request_id,
            label=label,
            output_description=output_description,
        )

    def _start_linear_motor_pin_sequence(
        self,
        pin_sequence: tuple[tuple[bool, bool], ...],
        *,
        command: str,
        request_id: int,
        label: str,
        output_description: str,
    ) -> None:
        if request_id != self.gripper_stroke_request_id:
            return
        neutral_pin8, neutral_pin9 = pin_sequence[0]
        self._publish_linear_motor_pin_levels(
            neutral_pin8,
            neutral_pin9,
        )
        self._append_log(
            f"[Arduino PIN 단계] {label} 1/{len(pin_sequence)}: "
            f"PIN8={'HIGH' if neutral_pin8 else 'LOW'}, "
            f"PIN9={'HIGH' if neutral_pin9 else 'LOW'}"
        )
        if len(pin_sequence) == 1:
            self.root.after(
                50,
                lambda: self._finish_linear_motor_pin_command(
                    request_id,
                    label,
                    output_description,
                ),
            )
            return
        self.root.after(
            100,
            lambda: self._apply_linear_motor_pin_target(
                pin_sequence,
                command=command,
                request_id=request_id,
                label=label,
                output_description=output_description,
            ),
        )

    def _apply_linear_motor_pin_target(
        self,
        pin_sequence: tuple[tuple[bool, bool], ...],
        *,
        command: str,
        request_id: int,
        label: str,
        output_description: str,
    ) -> None:
        if request_id != self.gripper_stroke_request_id:
            return
        if (
            not self.linear_motor_serial_connected
            or not self.linear_motor_pin_state_initialized
        ):
            self.gripper_command_in_progress = False
            self.gripper_stroke_status.set("Arduino TTY 연결 끊김")
            self.status.set(f"그리퍼 {label} 실패 — Arduino 연결 끊김")
            self._update_gripper_stroke_controls()
            return
        target_pin8, target_pin9 = pin_sequence[-1]
        self._publish_linear_motor_pin_levels(target_pin8, target_pin9)
        self._append_log(
            f"[Arduino PIN 단계] {label} 2/{len(pin_sequence)}: "
            f"PIN8={'HIGH' if target_pin8 else 'LOW'}, "
            f"PIN9={'HIGH' if target_pin9 else 'LOW'}"
        )
        self._finish_linear_motor_pin_command(
            request_id,
            label,
            output_description,
        )
        if command == "extend":
            duration = GRIPPER_EXTEND_AUTO_STOP_SECONDS
            self._append_log(
                f"[그리퍼 스트로크 자동 정지 예약] {duration:.1f}초 후 "
                "PIN8/PIN9를 LOW로 변경합니다."
            )
            self.root.after(
                int(round(duration * 1000.0)),
                lambda: self._auto_stop_gripper_extension(
                    request_id,
                    duration,
                ),
            )

    def _auto_stop_gripper_extension(
        self,
        request_id: int,
        duration_seconds: float,
    ) -> None:
        """Stop an unchanged extend command after its bounded run time."""
        if self.closing or request_id != self.gripper_stroke_request_id:
            return
        self.gripper_stroke_request_id += 1
        self._publish_linear_motor_pin_levels(False, False)
        self.gripper_command_in_progress = False
        self.gripper_stroke_status.set("늘림 3초 완료 — 자동 정지")
        self.status.set("그리퍼 늘림 3초 완료 — PIN8/PIN9 LOW")
        self._append_log(
            "[그리퍼 스트로크 자동 정지] "
            f"늘림 {float(duration_seconds):.1f}초 완료: "
            "PIN8=LOW, PIN9=LOW"
        )
        self._update_gripper_stroke_controls()

    def _finish_linear_motor_pin_command(
        self,
        request_id: int,
        label: str,
        output_description: str,
    ) -> None:
        if request_id != self.gripper_stroke_request_id:
            return
        self.gripper_command_in_progress = False
        self.gripper_stroke_status.set(f"{label} 명령 완료")
        self.status.set(f"그리퍼 스트로크 {label} 출력 적용 완료")
        self._append_log(
            f"[그리퍼 스트로크 적용됨] {label}: {output_description}"
        )
        self._update_gripper_stroke_controls()

    def _send_gripper_script_sequence(
        self,
        scripts: tuple[str, ...],
        *,
        request_id: int,
        label: str,
        output_description: str,
        initialization: bool,
        index: int = 0,
    ) -> None:
        if request_id != self.gripper_stroke_request_id:
            return
        request = Eval.Request()
        request.script = scripts[index]
        self._append_log(
            f"[그리퍼 DOUT 단계] {label} {index + 1}/{len(scripts)}: "
            f"{request.script}"
        )
        future = self.hardware_eval_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._gripper_script_step_completed(
                completed,
                scripts=scripts,
                request_id=request_id,
                label=label,
                output_description=output_description,
                initialization=initialization,
                index=index,
            )
        )

    def _gripper_script_step_completed(
        self,
        future,
        *,
        scripts: tuple[str, ...],
        request_id: int,
        label: str,
        output_description: str,
        initialization: bool,
        index: int,
    ) -> None:
        """Advance only after the RB controller accepted the prior step."""
        if request_id != self.gripper_stroke_request_id:
            return
        try:
            response = future.result()
        except Exception as error:
            self._append_log(
                f"[그리퍼 DOUT 실패] {label} {index + 1}/{len(scripts)} "
                "서비스 호출 오류: "
                f"{error}"
            )
            self._finish_gripper_script_sequence(
                request_id,
                label,
                output_description,
                initialization,
                success=False,
            )
            return

        if not response.success:
            self._append_log(
                f"[그리퍼 DOUT 실패] RB 컨트롤러가 {label} "
                f"{index + 1}/{len(scripts)} 단계를 거부했습니다: "
                f"{scripts[index]}"
            )
            self._finish_gripper_script_sequence(
                request_id,
                label,
                output_description,
                initialization,
                success=False,
            )
            return

        next_index = index + 1
        if next_index < len(scripts):
            if initialization:
                self.gripper_stroke_status.set(
                    "안전 초기화: 다음 DOUT 단계"
                )
            self._send_gripper_script_sequence(
                scripts,
                request_id=request_id,
                label=label,
                output_description=output_description,
                initialization=initialization,
                index=next_index,
            )
            return
        self._finish_gripper_script_sequence(
            request_id,
            label,
            output_description,
            initialization,
            success=True,
        )

    def _finish_gripper_script_sequence(
        self,
        request_id: int,
        label: str,
        output_description: str,
        initialization: bool,
        *,
        success: bool,
    ) -> None:
        if request_id != self.gripper_stroke_request_id:
            return
        self.gripper_command_in_progress = False
        if not success:
            self.gripper_outputs_initialized = False
            self.gripper_stroke_status.set(f"{label} 명령 실패")
            self.status.set(
                f"그리퍼 {label} 실패 — DOUT 안전 초기화가 필요합니다."
            )
            self._update_gripper_stroke_controls()
            return
        if initialization:
            self.gripper_outputs_initialized = True
            self.gripper_stroke_status.set(
                "제어 준비 (DOUT0·8 HIGH / Arduino PIN)"
            )
            self.status.set("그리퍼 DOUT 안전 초기화 완료")
            self._append_log(
                "[그리퍼 DOUT 초기화 완료] "
                f"{output_description}"
            )
        else:
            self.gripper_stroke_status.set(f"{label} 명령 완료")
            self.status.set(
                f"그리퍼 스트로크 {label} 출력 적용 완료"
            )
            self._append_log(
                f"[그리퍼 스트로크 적용됨] {label}: "
                f"{output_description}"
            )
        self._update_gripper_stroke_controls()

    def apply_motion_speed(self) -> None:
        """Apply the OMPL planning scales; RB Speed Bar stays automatic."""
        try:
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
        if self.hardware_speed_bar_applied:
            self.status.set(
                "속도 적용 완료 — RB Speed Bar 자동 100%, "
                f"OMPL/Joint {velocity * 100:.0f}%"
            )
        else:
            self.status.set(
                "OMPL/Joint 속도 적용 완료 — 시뮬레이션 또는 "
                "RB 하드웨어 연결 대기 중"
            )

    def _maintain_hardware_speed_bar(self) -> None:
        """Keep a connected RB controller at an operator-independent 100%."""
        if not rclpy.ok():
            return
        service_ready = self.hardware_speed_client.service_is_ready()
        if not service_ready:
            self.hardware_speed_bar_applied = False
        elif (
            not self.hardware_speed_bar_applied
            and not self.hardware_speed_bar_request_pending
        ):
            request = SetSpeedBar.Request()
            request.speed = 1.0
            self.hardware_speed_bar_request_pending = True
            future = self.hardware_speed_client.call_async(request)
            future.add_done_callback(self._automatic_speed_bar_applied)
        self.root.after(2000, self._maintain_hardware_speed_bar)

    def _automatic_speed_bar_applied(self, future) -> None:
        self.hardware_speed_bar_request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.hardware_speed_bar_applied = False
            if not self.hardware_speed_bar_failure_logged:
                self._append_log(
                    "[RB Speed Bar 자동 설정 실패] 100% 요청 중 오류: "
                    f"{error}; 연결 상태를 확인하며 자동 재시도합니다."
                )
                self.hardware_speed_bar_failure_logged = True
            return
        if not response.success:
            self.hardware_speed_bar_applied = False
            if not self.hardware_speed_bar_failure_logged:
                self._append_log(
                    "[RB Speed Bar 자동 설정 거부] 컨트롤러가 100% 요청을 "
                    "거부했습니다. 자동 재시도합니다."
                )
                self.hardware_speed_bar_failure_logged = True
            return
        first_success = not self.hardware_speed_bar_applied
        self.hardware_speed_bar_applied = True
        self.hardware_speed_bar_failure_logged = False
        if first_success:
            self._append_log(
                "[실제 로봇 RB Speed Bar] 자동 100% 설정 완료 — "
                "사용자 조정 없이 유지합니다."
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
                tomato_frame=self._tomato_frame(index),
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
        message.markers.extend(
            marker
            for index, report in sorted(
                self.harvest_result_approach_reports.items()
            )
            if (
                marker := actual_approach_marker(index, report)
            ) is not None
        )
        message.markers.extend(self.sweep_markers)
        self.result_marker_publisher.publish(message)

    def _set_harvest_result(
        self,
        tomato_index: int,
        success: bool,
        adaptive_rotation_applied: bool = False,
        adaptive_rotation_deg: float = 0.0,
        approach_axis_local=None,
        plan_report=None,
        motion_result_text: str | None = None,
    ) -> None:
        self._update_result_arrow_length(
            tomato_index,
            success=success,
            plan_report=plan_report,
        )
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
        if actual_approach_marker(tomato_index, plan_report or {}) is None:
            self.harvest_result_approach_reports.pop(tomato_index, None)
        else:
            self.harvest_result_approach_reports[tomato_index] = dict(
                plan_report
            )
        if motion_result_text is not None:
            self._set_tomato_motion_result(
                tomato_index,
                motion_result_text,
            )
        self.clear_markers_button.configure(state="normal")
        self._publish_harvest_result_markers()
        if self._selected_index() == int(tomato_index):
            self._update_selected_tomato_plot()

    def _set_tomato_motion_result(self, tomato_index: int, result: str) -> None:
        """Update the current detection row without disturbing its selection."""
        self.tomato_motion_results[int(tomato_index)] = str(result)
        item = str(tomato_index)
        if self.tomato_tree.exists(item):
            self.tomato_tree.set(item, "motion_result", str(result))
            autofit = getattr(self, "_autofit_tomato_tree_columns", None)
            if autofit is not None:
                autofit(columns=("motion_result",))

    def _autofit_tomato_tree_columns(self, columns=None) -> None:
        """Size tomato-table columns from their heading and current values."""
        tree = self.tomato_tree
        headings = self.tomato_tree_headings
        target_columns = tuple(columns or tree["columns"])
        cell_font = tkfont.nametofont("TkDefaultFont", self.root)
        heading_font = tkfont.nametofont("TkHeadingFont", self.root)
        cell_padding = 24
        heading_padding = 28

        for column in target_columns:
            width = heading_font.measure(headings[column]) + heading_padding
            for item in tree.get_children(""):
                value = tree.set(item, column)
                width = max(
                    width,
                    cell_font.measure(str(value)) + cell_padding,
                )
            tree.column(
                column,
                width=max(40, width),
                minwidth=max(40, width),
                stretch=False,
            )

    def _reset_tomato_motion_results(self, result: str = "미실행") -> None:
        self.tomato_motion_results.clear()
        for index in range(len(self.detected_tomatoes)):
            self._set_tomato_motion_result(index, result)

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
        self.harvest_result_approach_reports.clear()
        self.sweep_markers.clear()
        self.sweep_marker_next_id = 0
        self.clear_markers_button.configure(state="disabled")
        self._update_selected_tomato_plot()

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

    def show_preapproach_goal_state(self, *, quiet: bool = False) -> bool:
        """Publish the planned PRE_APPROACH joint state as the RViz query goal."""
        stage = next(
            (
                item
                for item in self.step_stages
                if item.get("key") == "READY_TO_PREAPPROACH"
            ),
            None,
        )
        joint_positions = (
            stage.get("expected_end", {}) if stage is not None else {}
        )
        if not isinstance(joint_positions, dict) or not joint_positions:
            if not quiet:
                messagebox.showinfo(
                    "PRE_APPROACH 자세 없음",
                    "먼저 스텝 Plan을 실행해 PRE_APPROACH 자세를 계산하세요.",
                )
            return False

        try:
            names = [str(name) for name in joint_positions]
            positions = [float(joint_positions[name]) for name in names]
        except (TypeError, ValueError):
            if not quiet:
                messagebox.showerror(
                    "PRE_APPROACH 자세 오류",
                    "Plan 결과의 PRE_APPROACH 관절값이 올바르지 않습니다.",
                )
            return False

        message = RobotState()
        message.is_diff = False
        message.joint_state.header.stamp = self.get_clock().now().to_msg()
        message.joint_state.name = names
        message.joint_state.position = positions
        self.rviz_goal_state_publisher.publish(message)

        self.status.set("PRE_APPROACH 자세를 RViz Query Goal에 표시했습니다.")
        joints_deg = ", ".join(
            f"{name}={math.degrees(position):.1f}°"
            for name, position in zip(names, positions)
        )
        self._append_log(
            "[RViz Query Goal] PRE_APPROACH 관절 자세 발행: " + joints_deg
        )
        if self.rviz_goal_state_publisher.get_subscription_count() == 0:
            self._append_log(
                "[RViz Query Goal] 구독자가 없습니다. RViz MoveIt 패널의 "
                "Allow External Program 설정을 확인하세요."
            )
        return True

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
            "pick_ready_state": self.sweep_pick_ready_state,
            "execute_motion": self.sweep_execute_motion,
            "linear_motor_wait_sec": self.sweep_harvest_wait_sec,
            "continuous_harvest": self.sweep_continuous_mode,
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
        self._reset_live_statistics(f"결과 파일: {session_directory}")

    def _reset_live_statistics(self, result_path_text: str) -> None:
        """Clear the shared live table for a new sweep or batch run."""
        self.sweep_started_monotonic = time.monotonic()
        self.sweep_completed = 0
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
        self.sweep_result_path.set(result_path_text)

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
            or self.step_process is not None
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
            pick_ready_state = self._selected_pick_ready_state()
            prefer_robot_direction, adaptive_max_rotation = (
                self._adaptive_grasp_options()
            )
            tcp_wrist_rotation_deg = float(
                self.tcp_wrist_rotation_deg_var.get()
            )
            if not 1.0 <= tcp_wrist_rotation_deg <= 45.0:
                raise ValueError("5단계 TCP 회전각은 1~45도 범위여야 합니다.")
        except ValueError as error:
            messagebox.showerror("대기시간 입력 오류", str(error))
            return
        execute_motion = bool(self.sweep_execute_motion_var.get())
        continuous_mode = bool(
            execute_motion and self.continuous_harvest_var.get()
        )
        lift_harvest_requested = bool(self.lift_harvest_var.get())
        lift_harvest_mode = bool(
            lift_harvest_requested
            and (execute_motion or self.lift_simulation_mode)
        )
        if lift_harvest_requested and not lift_harvest_mode:
            self._append_log(
                "[리프트 수확] 실제 모드의 자동 Plan-only 테스트에서는 "
                "리프트를 물리적으로 움직이지 않습니다. '실제 로봇 실행'을 "
                "체크해야 리프트 수확이 적용됩니다."
            )
        if continuous_mode and lift_harvest_mode:
            transition_message = (
                "연속+리프트 수확에서는 식물 바깥 안전 위치로 후퇴한 뒤 "
                "리프트를 조정하고 다음 pre-grasp를 새로 계획합니다.\n"
            )
        elif continuous_mode:
            transition_message = (
                "연속 수확 모드에서는 현재 post-wait 자세에서 다음 "
                "pre-grasp까지 식물 바깥쪽 arc 경로로 이동하며, 마지막 "
                f"토마토 이후에만 {pick_ready_state} 자세로 복귀합니다.\n"
            )
        else:
            transition_message = (
                f"각 수확 후 {pick_ready_state} 자세로 복귀합니다.\n"
            )
        if lift_harvest_mode:
            transition_message += (
                "각 토마토보다 400mm 낮은 높이로 리프트를 자동 이동합니다.\n"
            )
        transition_message += (
            f"선택한 시작/최종 복귀 자세는 {pick_ready_state}입니다.\n"
        )
        if execute_motion and not messagebox.askyesno(
            "자동 실제 로봇 실행",
            "자동 테스트에서 Plan에 성공한 모든 토마토 모션을 실제로 "
            "실행합니다.\n\n"
            f"{transition_message}"
            "실제 실행 실패 시 자동 테스트를 즉시 중단합니다.\n"
            "로봇 주변이 안전한지 확인했습니까?",
            icon="warning",
        ):
            return
        self.sweep_execute_motion = execute_motion
        self.sweep_harvest_wait_sec = harvest_wait_sec
        self.sweep_tcp_wrist_rotation_deg = tcp_wrist_rotation_deg
        self.sweep_continuous_mode = continuous_mode
        self.sweep_lift_harvest_mode = lift_harvest_mode
        self.sweep_pick_ready_state = pick_ready_state
        self.sweep_prefer_robot_direction = prefer_robot_direction
        self.sweep_adaptive_grasp_max_rotation_deg = adaptive_max_rotation
        try:
            cases, input_config = self._read_sweep_inputs()
            input_config["linear_motor_wait_sec"] = harvest_wait_sec
            input_config["continuous_harvest"] = continuous_mode
            input_config["lift_harvest"] = lift_harvest_mode
            input_config["pick_ready_state"] = pick_ready_state
            input_config["prefer_robot_direction"] = (
                prefer_robot_direction
            )
            input_config["adaptive_grasp_max_rotation_deg"] = (
                adaptive_max_rotation
            )
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
            f"리니어모터 대기={self.sweep_harvest_wait_sec:.2f}s, "
            f"연속 arc 전환 모드={self.sweep_continuous_mode}, "
            f"리프트 수확 모드={self.sweep_lift_harvest_mode}, "
            "진입각 로봇방향 우선="
            f"{self.sweep_prefer_robot_direction}, "
            "최대 보정각="
            f"{self.sweep_adaptive_grasp_max_rotation_deg:g}°, "
            f"시작/복귀 자세={self.sweep_pick_ready_state}"
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
        # A step may currently be driving the Arduino linear actuator.  Force
        # both direction outputs LOW before terminating its worker process.
        if hasattr(self, "_publish_linear_motor_pin_levels"):
            self._publish_linear_motor_pin_levels(False, False)
            self._append_log(
                "[정지 요청] 리니어모터 PIN8/PIN9도 LOW로 정지했습니다."
            )
        if self.lift_harvest_pending is not None:
            self.lift_harvest_pending = None
            stop_message = Bool()
            stop_message.data = True
            publisher = (
                self.lift_simulated_stop_publisher
                if self.lift_simulation_mode
                else self.lift_stop_publisher
            )
            publisher.publish(stop_message)
            self._append_log("[정지 요청] 자동 리프트 높이 이동도 정지했습니다.")
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
                "pick_ready_state": self.sweep_pick_ready_state,
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
        self.sweep_lift_harvest_mode = False
        self.lift_harvest_pending = None
        self._clear_active_camera_target()
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
            self.sweep_pick_ready_state,
            self.sweep_prefer_robot_direction,
            self.sweep_adaptive_grasp_max_rotation_deg,
        )
        self._set_active_camera_target(tomato_index, "자동 테스트")
        current_number = self.sweep_case_tomato_completed + 1
        continuous_transition = (
            self.sweep_continuous_mode and current_number > 1
        )
        return_to_pick_ready = (
            not self.sweep_continuous_mode
            or current_number == self.sweep_case_tomato_total
        )
        retreat_after_harvest = bool(
            self.sweep_lift_harvest_mode
            and self.sweep_continuous_mode
            and current_number < self.sweep_case_tomato_total
        )
        self.sweep_summary.set(
            f"케이스 {self.sweep_case_number} / {self.sweep_case_total} — "
            f"토마토 {current_number} / {self.sweep_case_tomato_total} "
            f"(index {tomato_index}) "
            f"{'Plan+실행' if self.sweep_execute_motion else 'Plan'} 중"
        )

        self._prepare_lift_for_tomato(
            tomato_index,
            self.sweep_lift_harvest_mode,
            lambda: self._send_sweep_plan_request(
                tomato_index,
                verification,
                continuous_transition,
                return_to_pick_ready,
                retreat_after_harvest,
            ),
            lambda message: self._handle_sweep_lift_preparation_error(
                tomato_index,
                verification,
                message,
            ),
        )

    def _send_sweep_plan_request(
        self,
        tomato_index: int,
        verification,
        continuous_transition: bool,
        return_to_pick_ready: bool,
        retreat_after_harvest: bool,
    ) -> None:
        if not self.sweep_active or self.sweep_cancel_requested:
            return
        process = self.sweep_worker_process
        if process is None or process.poll() is not None or process.stdin is None:
            self._finish_sweep("자동 테스트 지속 Planner가 종료되었습니다.")
            return
        self.sweep_request_id += 1
        self.sweep_pending_verification = (
            self.sweep_request_id,
            verification,
        )
        pipeline, planner_id, preapproach_mode = verification[2:5]
        request = {
            "request_id": self.sweep_request_id,
            "tomato_frame": self._tomato_frame(tomato_index),
            "planning_pipeline_id": pipeline,
            "planner_id": planner_id,
            "preapproach_mode": preapproach_mode,
            "pick_ready_state_name": verification[5],
            "execute": self.sweep_execute_motion,
            "velocity_scale": self.motion_velocity_scale,
            "acceleration_scale": self.motion_acceleration_scale,
            "harvest_wait_sec": self.sweep_harvest_wait_sec,
            "harvest_tcp_wrist_rotation_deg": (
                self.sweep_tcp_wrist_rotation_deg
            ),
            "continuous_transition": continuous_transition,
            "return_to_pick_ready": return_to_pick_ready,
            "retreat_after_harvest": retreat_after_harvest,
            "prefer_robot_direction": (
                self.sweep_prefer_robot_direction
            ),
            "adaptive_grasp_max_rotation_deg": (
                self.sweep_adaptive_grasp_max_rotation_deg
            ),
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

    def _update_result_arrow_length(
        self,
        tomato_index: int,
        success: bool | None = None,
        plan_report=None,
    ) -> None:
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
        else:
            detection = self.detected_tomatoes[tomato_index]
            self.result_arrow_lengths[tomato_index] = tomato_stem_arrow_length(
                detection.center,
                detection.stem_point,
                source_to_parent_quaternion,
                float(self.get_parameter("result_arrow_stem_margin").value),
                minimum_length,
            )
        self.result_arrow_lengths[tomato_index] = (
            result_arrow_length_for_report(
                success,
                (
                    max(
                        self.result_arrow_lengths[tomato_index],
                        self.detection_approach_marker_length,
                    )
                    if success is False
                    else self.result_arrow_lengths[tomato_index]
                ),
                plan_report,
            )
        )

    def _detection_key(self, message: TomatoDetectionArray):
        stamp = message.header.stamp
        points = tuple(
            (
                detection.id,
                detection.center.x,
                detection.center.y,
                detection.center.z,
                detection.calyx_point.x,
                detection.calyx_point.y,
                detection.calyx_point.z,
                detection.stem_point.x,
                detection.stem_point.y,
                detection.stem_point.z,
            )
            for detection in message.detections
        )
        return (
            stamp.sec,
            stamp.nanosec,
            message.header.frame_id,
            self._selected_angle_reference_mode(),
            points,
        )

    def _detections_callback(self, message: TomatoDetectionArray) -> None:
        sorter = getattr(self, "_sort_detection_message_by_height", None)
        if sorter is not None:
            try:
                message = sorter(message)
            except (TransformException, ValueError) as error:
                self._append_log(
                    "[검출 높이 정렬 경고] world 기준 좌표를 계산하지 못해 "
                    f"카메라 배열 순서를 유지합니다: {error}"
                )
        signature = self._detection_key(message)
        if signature == self.detection_signature:
            return
        capture_pose = self.pending_detection_capture_pose
        self.pending_detection_capture_pose = None
        if capture_pose is not None:
            self.detection_capture_pose = str(capture_pose)
            ready_state = pick_ready_state_for_capture_pose(
                self.detection_capture_pose,
                self.pick_ready_state_var.get(),
            )
            self.pick_ready_state_var.set(ready_state)
            self._append_log(
                f"[촬영 자세 연동] {self.detection_capture_pose} 검출 → "
                f"수확 시작/복귀 자세 {ready_state} 자동 선택"
            )
        self._clear_active_camera_target()
        self.latest_detection_message = message
        self.result_arrow_lengths.clear()
        self.result_detection_frame = message.header.frame_id
        self.detection_signature = signature
        self.current_detection_stamp_ns = self._stamp_nanoseconds(
            message.header.stamp
        )
        self.detected_tomatoes = list(message.detections)
        self._update_detection_image_overlay()
        self.tomato_motion_results.clear()
        try:
            self._cache_detected_tomato_world_positions(message)
        except TransformException as error:
            self.detected_tomato_expected_world_positions = {}
            self.detected_tomato_expected_world_x_axes = {}
            self._append_log(
                "[검출 TF 동기화 경고] 검출 중심의 world 좌표를 저장하지 "
                f"못했습니다: {error}"
            )
        try:
            self._cache_detected_tomato_record_positions(message)
        except (TransformException, ValueError) as error:
            self.detected_tomato_record_positions = {}
            self.detected_tomato_record_stem_positions = {}
            self.detected_tomato_record_calyx_positions = {}
            self.detected_tomato_record_angle_origins = {}
            self.detected_tomato_record_angle_targets = {}
            record_frame = str(
                self.get_parameter("camera_target_record_robot_frame").value
            )
            self._append_log(
                "[피드백 좌표 경고] 검출 중심/줄기점의 "
                f"{record_frame} 좌표를 "
                f"저장하지 못했습니다: {error}"
            )
        self.detection_generation += 1
        self._invalidate_plan()
        if self.show_detection_markers_var.get():
            self._publish_detection_markers(
                message,
            )

        for item in self.tomato_tree.get_children():
            self.tomato_tree.delete(item)
        choices = []
        for index, detection in enumerate(self.detected_tomatoes):
            frame = harvest_tf_frame_id(detection.id, index)
            choices.append(f"{index}: {frame}")
            center = detection.center
            self.tomato_tree.insert(
                "",
                "end",
                iid=str(index),
                values=(
                    index,
                    frame,
                    "미실행",
                    detection.id,
                    f"{center.x:.4f}",
                    f"{center.y:.4f}",
                    f"{center.z:.4f}",
                ),
            )
        self._autofit_tomato_tree_columns()
        self.tomato_combo.configure(values=choices)
        self.step_tomato_combo.configure(values=choices)
        self.repeat_tomato_combo.configure(values=choices)
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
            self.harvest_all_plan_button.configure(
                state="disabled" if busy else "normal"
            )
        else:
            self.selected_tomato.set("")
            self.plan_button.configure(state="disabled")
            self.harvest_all_button.configure(state="disabled")
            self.harvest_all_plan_button.configure(state="disabled")
        self.status.set(f"토마토 {len(choices)}개 검출됨")
        self._append_log(
            f"[{message.header.frame_id}] 새 검출 결과: 토마토 {len(choices)}개"
        )
        if choices:
            # Synchronize the visible combo as well as the planner parameter.
            # This also restores RIGHT correctly for a retained detection after
            # restarting only the GUI.
            self._selected_pick_ready_state()
        self._update_selected_tomato_plot()
        self._update_step_controls()
        update_target_controls = getattr(
            self, "_update_camera_target_record_controls", None
        )
        if update_target_controls is not None:
            update_target_controls()

    def detect_tomatoes(self) -> None:
        if not self.camera_client.service_is_ready():
            if not self.camera_client.wait_for_service(timeout_sec=0.05):
                self.status.set(f"카메라 서비스 연결 안 됨: {self.camera_service}")
                self._append_log(
                    f"카메라 서비스를 먼저 실행하세요: {self.camera_service}"
                )
                self.pending_detection_capture_pose = None
                return
        self.pending_detection_capture_pose = (
            str(self.named_pose_var.get()).strip()
            if str(self.named_pose_var.get()).strip()
            in ("CAPTURE_LEFT", "CAPTURE_RIGHT")
            else self.last_completed_named_pose
            if self.last_completed_named_pose
            in ("CAPTURE_LEFT", "CAPTURE_RIGHT")
            else None
        )
        if not self.tf_angle_reference_client.service_is_ready():
            if not self.tf_angle_reference_client.wait_for_service(
                timeout_sec=0.10
            ):
                self.status.set("토마토 TF 진입각 설정 서비스 연결 안 됨")
                self._append_log(
                    "진입각 기준을 TF 생성기에 전달할 수 없습니다: "
                    f"{self.tf_angle_reference_service}"
                )
                self.pending_detection_capture_pose = None
                return
        self.detect_button.configure(state="disabled")
        self.camera_source_combo.configure(state="disabled")
        self.angle_reference_mode_combo.configure(state="disabled")
        self.status.set("TF 진입각 기준 동기화 중...")
        request = self._angle_reference_parameter_request()
        future = self.tf_angle_reference_client.call_async(request)
        future.add_done_callback(self._angle_reference_before_detection_done)

    def _angle_reference_parameter_request(self) -> SetParameters.Request:
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="angle_reference_mode",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=self._selected_angle_reference_mode(),
                ),
            )
        ]
        return request

    def _angle_reference_before_detection_done(self, future) -> None:
        try:
            response = future.result()
            if not response.results or not response.results[0].successful:
                reason = (
                    response.results[0].reason
                    if response.results
                    else "설정 응답이 비어 있습니다."
                )
                raise RuntimeError(reason or "TF 생성기 설정 거부")
        except Exception as error:
            self.pending_detection_capture_pose = None
            self.detect_button.configure(state="normal")
            self.camera_source_combo.configure(
                state="disabled" if self.ui_busy else "readonly"
            )
            self.angle_reference_mode_combo.configure(
                state="disabled" if self.ui_busy else "readonly"
            )
            self.status.set("TF 진입각 기준 동기화 실패")
            self._append_log(f"진입각 기준 동기화 실패: {error}")
            return
        reference = angle_reference_label(
            self._selected_angle_reference_mode()
        )
        self.status.set("카메라 촬영 및 토마토 검출 요청 중...")
        self._append_log(
            f"[{self.camera_source_var.get()}] 서비스 호출: "
            f"{self.camera_service}, 진입각 기준={reference}"
        )
        future = self.camera_client.call_async(DetectTomatoes.Request())
        future.add_done_callback(self._detection_service_done)

    def capture_camera(self) -> None:
        if not self.capture_camera_client.service_is_ready():
            if not self.capture_camera_client.wait_for_service(
                timeout_sec=0.05
            ):
                self.status.set(
                    f"카메라 캡처 서비스 연결 안 됨: "
                    f"{self.capture_camera_service}"
                )
                self._append_log(
                    "[카메라 캡처 실패] 서비스를 먼저 실행하세요: "
                    f"{self.capture_camera_service}"
                )
                return
        self.capture_camera_in_progress = True
        self.capture_camera_button.configure(state="disabled")
        self.detect_button.configure(state="disabled")
        self.camera_source_combo.configure(state="disabled")
        self.status.set("카메라 캡처 요청 중...")
        self._append_log(
            f"[카메라 캡처] 서비스 호출: {self.capture_camera_service}"
        )
        future = self.capture_camera_client.call_async(Trigger.Request())
        future.add_done_callback(self._capture_camera_done)

    def _capture_camera_done(self, future) -> None:
        self.capture_camera_in_progress = False
        self.capture_camera_button.configure(state="normal")
        detection_state = "disabled" if self.ui_busy else "normal"
        self.detect_button.configure(state=detection_state)
        self.camera_source_combo.configure(
            state="disabled" if self.ui_busy else "readonly"
        )
        try:
            response = future.result()
        except Exception as error:
            self.status.set("카메라 캡처 서비스 호출 실패")
            self._append_log(f"[카메라 캡처 오류] {error}")
            return
        if response.success:
            message = response.message or "카메라 캡처 완료"
            self.status.set(message)
            self._append_log(f"[카메라 캡처 성공] {message}")
            return
        message = response.message or "카메라 캡처 실패"
        self.status.set(message)
        self._append_log(f"[카메라 캡처 실패] {message}")

    def _result_image_callback(self, message: CompressedImage) -> None:
        try:
            image = decode_compressed_result_image(message.data)
        except ValueError as error:
            self.result_image_status.set("결과 이미지 디코딩 실패")
            self._append_log(f"[검출 결과 이미지 오류] {error}")
            return
        self.latest_result_image = image
        image_format = str(message.format or "compressed")
        self.result_image_status.set(
            f"{self.result_image_topic} · {image.width}×{image.height} · "
            f"{image_format}"
        )
        self._schedule_result_image_render()

    def _schedule_result_image_render(self, _event=None) -> None:
        if self.latest_result_image is None or self.closing:
            return
        if self.result_image_render_job is not None:
            try:
                self.root.after_cancel(self.result_image_render_job)
            except tk.TclError:
                pass
        self.result_image_render_job = self.root.after(
            60,
            self._render_result_image,
        )

    def _render_result_image(self) -> None:
        self.result_image_render_job = None
        if self.latest_result_image is None or self.closing:
            return
        maximum_width = max(120, self.result_image_label.winfo_width() - 8)
        maximum_height = max(100, self.result_image_label.winfo_height() - 8)
        display_image = self.latest_result_image.copy()
        display_image.thumbnail(
            (maximum_width, maximum_height),
            Image.Resampling.LANCZOS,
        )
        self.result_image_photo = ImageTk.PhotoImage(display_image)
        self.result_image_label.configure(
            image=self.result_image_photo,
            text="",
        )

    def _camera_color_image_callback(self, message: RosImage) -> None:
        try:
            image = decode_raw_result_image(message)
        except ValueError as error:
            self.camera_color_image_status.set(
                "Camera Color 이미지 디코딩 실패"
            )
            self._append_log(f"[Camera Color 이미지 오류] {error}")
            return
        self.latest_camera_color_image = image
        image_format = str(message.encoding or "raw")
        self.camera_color_image_status.set(
            f"{self.camera_color_image_topic} · "
            f"{image.width}×{image.height} · {image_format}"
        )
        self._update_detection_image_overlay()

    def _vision_result_image_callback(self, message: RosImage) -> None:
        try:
            image = decode_raw_result_image(message)
        except ValueError as error:
            self.result_image_status.set("Vision Result 디코딩 실패")
            self._append_log(f"[Vision Result 이미지 오류] {error}")
            return
        self.latest_result_image = image
        image_format = str(message.encoding or "raw")
        self.result_image_status.set(
            f"{self.vision_result_image_topic} · "
            f"{image.width}×{image.height} · {image_format}"
        )
        self._schedule_result_image_render()

    def _camera_color_info_callback(self, message: CameraInfo) -> None:
        signature = (
            int(message.width),
            int(message.height),
            str(message.header.frame_id),
            tuple(float(value) for value in message.k),
        )
        previous = self.latest_camera_color_info
        previous_signature = None
        if previous is not None:
            previous_signature = (
                int(previous.width),
                int(previous.height),
                str(previous.header.frame_id),
                tuple(float(value) for value in previous.k),
            )
        self.latest_camera_color_info = message
        if signature != previous_signature:
            self._update_detection_image_overlay()

    def _update_detection_image_overlay(self) -> None:
        image = self.latest_camera_color_image
        if image is None:
            self.camera_color_image_status.set(
                f"Camera Color 이미지 대기: {self.camera_color_image_topic}"
            )
            return
        camera_info = self.latest_camera_color_info
        if camera_info is None:
            self.latest_camera_color_display_image = image.copy()
            self.camera_color_image_status.set(
                f"CameraInfo 대기: {self.camera_color_info_topic}"
            )
            self._schedule_camera_color_image_render()
            return
        detection_message = self.latest_detection_message
        if detection_message is None or not detection_message.detections:
            self.latest_camera_color_display_image = image.copy()
            self.camera_color_image_status.set(
                f"{self.camera_color_image_topic} · 검출 좌표 대기"
            )
            self._schedule_camera_color_image_render()
            return

        detection_frame = str(detection_message.header.frame_id).strip()
        camera_frame = str(camera_info.header.frame_id).strip()
        transform = None
        if detection_frame and camera_frame and detection_frame != camera_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    camera_frame,
                    detection_frame,
                    Time(),
                )
            except TransformException as error:
                self.latest_camera_color_display_image = image.copy()
                self.camera_color_image_status.set(
                    f"좌표 변환 대기: {detection_frame} → {camera_frame}"
                )
                self._append_log(
                    "[검출 이미지 오버레이 보류] camera XYZ 프레임을 "
                    f"변환할 수 없습니다: {error}"
                )
                self._schedule_camera_color_image_render()
                return

        robot_frame = str(
            self.get_parameter("camera_target_record_robot_frame").value
        ).strip()
        robot_transform = None
        robot_angle_available = bool(detection_frame and robot_frame)
        if robot_angle_available and detection_frame != robot_frame:
            try:
                robot_transform = self.tf_buffer.lookup_transform(
                    robot_frame,
                    detection_frame,
                    Time(),
                )
            except TransformException as error:
                robot_angle_available = False
                self._append_log(
                    "[카메라 진입각 오버레이 보류] 선택 토마토 그래프의 "
                    f"{robot_frame} 좌표를 계산할 수 없습니다: {error}"
                )

        angle_mode = self._selected_angle_reference_mode()
        angle_base_point = None
        angle_available = True
        if angle_mode == ANGLE_REFERENCE_BASE_TO_CENTER:
            try:
                angle_base_point = self._robot_base_origin_in_frame(
                    detection_frame
                )
            except TransformException as error:
                angle_available = False
                self._append_log(
                    "[카메라 진입각 오버레이 보류] 로봇 베이스 "
                    f"좌표를 계산할 수 없습니다: {error}"
                )

        projected_detections = []
        for index, detection in enumerate(detection_message.detections):
            full_label = str(detection.id).strip()
            label = full_label.rsplit("/", 1)[-1] if full_label else f"T{index}"
            if transform is None:
                xyz = (
                    float(detection.center.x),
                    float(detection.center.y),
                    float(detection.center.z),
                )
            else:
                xyz = transformed_point_xyz(detection.center, transform)
            recommend_xyz = None
            recommend_direction = None
            if angle_available:
                try:
                    angle_origin, angle_target = detection_angle_segment(
                        detection,
                        angle_mode,
                        base_point=angle_base_point,
                    )
                    if transform is None:
                        angle_origin_xyz = (
                            float(angle_origin.x),
                            float(angle_origin.y),
                            float(angle_origin.z),
                        )
                        angle_target_xyz = (
                            float(angle_target.x),
                            float(angle_target.y),
                            float(angle_target.z),
                        )
                    else:
                        angle_origin_xyz = transformed_point_xyz(
                            angle_origin,
                            transform,
                        )
                        angle_target_xyz = transformed_point_xyz(
                            angle_target,
                            transform,
                        )
                    recommend_xyz = recommend_entry_point(
                        xyz,
                        angle_origin_xyz,
                        angle_target_xyz,
                    )
                    if robot_angle_available:
                        if robot_transform is None:
                            robot_tomato_xyz = (
                                float(detection.center.x),
                                float(detection.center.y),
                                float(detection.center.z),
                            )
                            robot_angle_origin_xyz = (
                                float(angle_origin.x),
                                float(angle_origin.y),
                                float(angle_origin.z),
                            )
                            robot_angle_target_xyz = (
                                float(angle_target.x),
                                float(angle_target.y),
                                float(angle_target.z),
                            )
                        else:
                            robot_tomato_xyz = transformed_point_xyz(
                                detection.center,
                                robot_transform,
                            )
                            robot_angle_origin_xyz = transformed_point_xyz(
                                angle_origin,
                                robot_transform,
                            )
                            robot_angle_target_xyz = transformed_point_xyz(
                                angle_target,
                                robot_transform,
                            )
                        recommend_direction = (
                            selection_plot_recommend_screen_direction(
                                robot_tomato_xyz,
                                robot_angle_origin_xyz,
                                robot_angle_target_xyz,
                            )
                        )
                except ValueError:
                    recommend_xyz = None
                    recommend_direction = None
            projected_detections.append(
                (label, xyz, recommend_xyz, recommend_direction)
            )
        overlay, projected_count = draw_detection_label_overlay(
            image,
            projected_detections,
            camera_info.k,
            (camera_info.width, camera_info.height),
        )
        self.latest_camera_color_display_image = overlay
        self.camera_color_image_status.set(
            f"사용자 오버레이 {projected_count}/{len(projected_detections)}개 · "
            f"원본={self.camera_color_image_topic}"
        )
        self._schedule_camera_color_image_render()

    def _schedule_camera_color_image_render(self, _event=None) -> None:
        if self.latest_camera_color_display_image is None or self.closing:
            return
        if self.camera_color_image_render_job is not None:
            try:
                self.root.after_cancel(self.camera_color_image_render_job)
            except tk.TclError:
                pass
        self.camera_color_image_render_job = self.root.after(
            60,
            self._render_camera_color_image,
        )

    def _render_camera_color_image(self) -> None:
        self.camera_color_image_render_job = None
        if self.latest_camera_color_display_image is None or self.closing:
            return
        maximum_width = max(
            120,
            self.camera_color_image_label.winfo_width() - 8,
        )
        maximum_height = max(
            100,
            self.camera_color_image_label.winfo_height() - 8,
        )
        display_image = self.latest_camera_color_display_image.copy()
        display_image.thumbnail(
            (maximum_width, maximum_height),
            Image.Resampling.LANCZOS,
        )
        self.camera_color_image_photo = ImageTk.PhotoImage(display_image)
        self.camera_color_image_label.configure(
            image=self.camera_color_image_photo,
            text="",
        )

    def _image_notebook_tab_changed(self, _event=None) -> None:
        self._schedule_result_image_render()
        self._schedule_camera_color_image_render()

    def _detection_service_done(self, future) -> None:
        self.detect_button.configure(state="normal")
        self.angle_reference_mode_combo.configure(
            state="disabled" if self.ui_busy else "readonly"
        )
        self.camera_source_combo.configure(
            state="disabled" if self.ui_busy else "readonly"
        )
        try:
            response = future.result()
        except Exception as error:
            self.pending_detection_capture_pose = None
            self.status.set("카메라 서비스 호출 실패")
            self._append_log(f"카메라 서비스 오류: {error}")
            return
        if not response.success:
            self.pending_detection_capture_pose = None
            self.status.set("토마토 검출 실패")
            self._append_log(f"검출 실패: {response.message}")
            return
        if self.camera_source_var.get() == CAMERA_SOURCE_REAL:
            self.detections_publisher.publish(response.detections)
            self._append_log(
                "[실제 카메라] 검출 결과를 TF 생성용 토픽에 전달: "
                f"{len(response.detections.detections)}개"
            )
        self._detections_callback(response.detections)
        self.status.set(response.message)
        self._append_log(response.message)

    def _camera_source_changed(self, _event=None) -> None:
        """Switch detection service and invalidate coordinates from the old source."""
        source = str(self.camera_source_var.get())
        try:
            service = camera_service_for_source(
                source,
                self.fake_camera_service,
                self.real_camera_service,
            )
            client = self.camera_clients[source]
        except (KeyError, ValueError) as error:
            self.status.set("카메라 검출 소스 선택 오류")
            self._append_log(f"카메라 검출 소스 변경 실패: {error}")
            return

        self.camera_service = service
        self.camera_client = client
        self.camera_service_display.set(service)
        clear_active_target = getattr(
            self, "_clear_active_camera_target", None
        )
        if clear_active_target is not None:
            clear_active_target()
        self.detected_tomatoes = []
        self.latest_detection_message = None
        update_overlay = getattr(
            self,
            "_update_detection_image_overlay",
            None,
        )
        if update_overlay is not None:
            update_overlay()
        self.pending_detection_capture_pose = None
        self.detection_capture_pose = None
        self.detected_tomato_expected_world_positions.clear()
        self.detected_tomato_expected_world_x_axes.clear()
        self.detected_tomato_record_positions.clear()
        self.detected_tomato_record_stem_positions.clear()
        self.detected_tomato_record_calyx_positions.clear()
        self.detected_tomato_record_angle_origins.clear()
        self.detected_tomato_record_angle_targets.clear()
        self.tomato_motion_results.clear()
        self.result_arrow_lengths.clear()
        self.result_detection_frame = ""
        self.detection_signature = None
        self.current_detection_stamp_ns = 0
        self.detection_generation += 1
        for item in self.tomato_tree.get_children():
            self.tomato_tree.delete(item)
        self.tomato_combo.configure(values=[])
        step_combo = getattr(self, "step_tomato_combo", None)
        if step_combo is not None:
            step_combo.configure(values=[])
        repeat_combo = getattr(self, "repeat_tomato_combo", None)
        if repeat_combo is not None:
            repeat_combo.configure(values=[])
        self.selected_tomato.set("")
        self.plan_button.configure(state="disabled")
        self.execute_button.configure(state="disabled")
        self.harvest_all_button.configure(state="disabled")
        self.harvest_all_plan_button.configure(state="disabled")
        self._invalidate_plan()
        self._clear_detection_markers()
        self.status.set(f"{source} 선택됨 — 토마토 검출을 실행하세요.")
        self._append_log(
            f"[카메라 소스 변경] {source}: {service}. "
            "이전 검출 좌표와 Plan 상태를 초기화했습니다."
        )
        self._update_selected_tomato_plot()
        self._update_step_controls()
        update_target_controls = getattr(
            self, "_update_camera_target_record_controls", None
        )
        if update_target_controls is not None:
            update_target_controls()

    def _publish_detection_markers(
        self,
        message: TomatoDetectionArray,
    ) -> None:
        try:
            mode = self._selected_angle_reference_mode()
            base_point = (
                self._robot_base_origin_in_frame(message.header.frame_id)
                if mode == ANGLE_REFERENCE_BASE_TO_CENTER
                else None
            )
            markers = detected_tomato_marker_array(
                message,
                diameter=self.detection_marker_diameter,
                stem_diameter=self.detection_stem_marker_diameter,
                approach_length=self.detection_approach_marker_length,
                angle_reference_mode=mode,
                base_point=base_point,
            )
        except (TransformException, ValueError) as error:
            self._append_log(f"검출 토마토 마커 생성 실패: {error}")
            return
        self.detection_marker_publisher.publish(markers)
        self._append_log(
            f"[검출 마커] 토마토·줄기점·진입 방향 "
            f"{len(message.detections)}세트 표시: "
            f"핑크=중심, 초록=줄기점, 하늘색=진입 방향, "
            "각도기준="
            f"{angle_reference_label(mode)}, "
            f"frame={message.header.frame_id}"
        )

    def _angle_reference_mode_changed(self, _event=None) -> None:
        mode = self._selected_angle_reference_mode()
        reference = angle_reference_label(mode)
        self._invalidate_plan()
        self.detection_signature = None
        self.detected_tomato_expected_world_x_axes.clear()
        self.detected_tomato_record_angle_origins.clear()
        self.detected_tomato_record_angle_targets.clear()
        self.plan_button.configure(state="disabled")
        self.harvest_all_button.configure(state="disabled")
        self.harvest_all_plan_button.configure(state="disabled")
        self.status.set(
            f"진입각 기준: {reference} — 토마토 촬영/검출을 다시 실행하세요."
        )
        self._append_log(
            f"[진입각 기준 변경] {reference}. 기존 검출 Plan은 무효화했습니다."
        )
        if self.latest_detection_message is not None:
            if self.show_detection_markers_var.get():
                self._publish_detection_markers(self.latest_detection_message)
            self._update_selected_tomato_plot()
        if not self.tf_angle_reference_client.service_is_ready():
            return
        request = self._angle_reference_parameter_request()
        future = self.tf_angle_reference_client.call_async(request)
        future.add_done_callback(self._angle_reference_mode_setting_done)

    def _angle_reference_mode_setting_done(self, future) -> None:
        try:
            response = future.result()
            if not response.results or not response.results[0].successful:
                reason = (
                    response.results[0].reason
                    if response.results
                    else "설정 응답이 비어 있습니다."
                )
                raise RuntimeError(reason or "설정 거부")
        except Exception as error:
            self._append_log(f"[진입각 기준 설정 경고] {error}")
            return
        self._append_log(
            "[TF 생성기] 진입각 기준 적용: "
            f"{angle_reference_label(self._selected_angle_reference_mode())}"
        )

    def _clear_detection_markers(self) -> None:
        marker = Marker()
        marker.action = Marker.DELETEALL
        message = MarkerArray()
        message.markers = [marker]
        self.detection_marker_publisher.publish(message)

    def _detection_marker_option_changed(self) -> None:
        if not self.show_detection_markers_var.get():
            self._clear_detection_markers()
            self.status.set("검출 토마토 마커 표시를 해제했습니다.")
            self._append_log("[검출 마커] 핑크색 토마토 마커 삭제")
            return
        if self.latest_detection_message is None:
            self.status.set("마커 표시 활성화 — 토마토 검출을 실행하세요.")
            return
        self._publish_detection_markers(
            self.latest_detection_message,
        )
        self.status.set("현재 검출 토마토 마커를 표시했습니다.")

    def _set_active_camera_target(self, index: int, context: str) -> None:
        """Pin the detection used by the active planner or step session."""
        if index < 0 or index >= len(self.detected_tomatoes):
            self._clear_active_camera_target()
            return
        self.active_camera_target_index = int(index)
        self.active_camera_target_context = str(context)
        self.active_camera_target_plan_report = {}
        self._update_camera_target_record_controls()

    def _clear_active_camera_target(self, index: int | None = None) -> None:
        if (
            index is not None
            and self.active_camera_target_index != int(index)
        ):
            return
        self.active_camera_target_index = None
        self.active_camera_target_context = ""
        self.active_camera_target_plan_report = {}
        self._update_camera_target_record_controls()

    def _set_active_camera_target_plan_report(self, report: dict) -> None:
        index = self.active_camera_target_index
        if index is None or not report:
            return
        report_target = str(report.get("tomato_frame") or "")
        expected = self._tomato_frame(index)
        if report_target and report_target != expected:
            return
        self.active_camera_target_plan_report = dict(report)

    def _active_camera_target_state(self) -> str:
        if self.repeat_paused:
            return "paused"
        if self.repeat_pause_requested:
            return "pause_requested"
        if self.step_execution_in_progress:
            return "executing"
        if self.step_process is not None:
            return "planning_or_step_ready"
        if self.harvest_process is not None or self.sweep_pending_verification:
            return "planning_or_executing"
        return "selected_or_plan_complete"

    def _camera_target_index_for_record(self) -> int | None:
        """Prefer the running target, then fall back to the GUI selection."""
        index = self.active_camera_target_index
        if index is None:
            index = self._selected_index()
        if index is None or not (0 <= index < len(self.detected_tomatoes)):
            return None
        return int(index)

    def _update_camera_target_record_controls(self) -> None:
        display = getattr(self, "camera_target_display", None)
        button = getattr(self, "save_camera_target_button", None)
        index = self._camera_target_index_for_record()
        if index is None:
            if display is not None:
                display.set("현재 Plan 대상 없음")
            if button is not None:
                button.configure(state="disabled")
            return

        detection = self.detected_tomatoes[index]
        center = detection.center
        camera_id = str(detection.id) or "(빈 ID)"
        if display is not None:
            target_label = (
                "현재 Plan 대상"
                if self.active_camera_target_index is not None
                else "선택 대상"
            )
            display.set(
                f"{target_label} #{index} | Camera ID={camera_id} | "
                f"X={center.x:.4f}, Y={center.y:.4f}, Z={center.z:.4f} m"
            )
        if button is not None:
            # This button intentionally remains usable while planning,
            # executing, or pausing an access-repeat session.
            button.configure(state="normal")

    def save_active_camera_target_record(self) -> None:
        """Save and send the active target's camera-space feedback JSON."""
        index = self._camera_target_index_for_record()
        if index is None:
            messagebox.showwarning(
                "저장할 Plan 대상 없음",
                "검출된 토마토를 선택하거나 Plan을 시작하세요.",
            )
            return

        detection = self.detected_tomatoes[index]
        tomato_frame = self._tomato_frame(index)
        center = detection.center
        timestamp = datetime.now().astimezone()
        camera_frame = self.result_detection_frame
        if self.latest_detection_message is not None:
            camera_frame = self.latest_detection_message.header.frame_id
        stamp = (
            self.latest_detection_message.header.stamp
            if self.latest_detection_message is not None
            else None
        )
        detection_timestamp = (
            float(stamp.sec) + float(stamp.nanosec) * 1e-9
            if stamp is not None and (stamp.sec or stamp.nanosec)
            else timestamp.timestamp()
        )
        try:
            report = self.active_camera_target_plan_report
            if not report:
                candidate_report = self.harvest_plan_report or {}
                expected_frame = tomato_frame
                if candidate_report.get("tomato_frame") == expected_frame:
                    report = candidate_report
            contents = camera_target_record_text(
                target_frame=tomato_frame,
                camera_frame=camera_frame,
                camera_id=str(detection.id),
                tomato_xyz=(center.x, center.y, center.z),
                vine_xyz=(
                    detection.stem_point.x,
                    detection.stem_point.y,
                    detection.stem_point.z,
                ),
                calyx_xyz=(
                    detection.calyx_point.x,
                    detection.calyx_point.y,
                    detection.calyx_point.z,
                ),
                angle_reference=self._selected_angle_reference_mode(),
                detection_timestamp=detection_timestamp,
                plan_report=report,
                robot_frame_id=str(
                    self.get_parameter(
                        "camera_target_record_robot_frame"
                    ).value
                ),
                robot_tomato_xyz=(
                    self.detected_tomato_record_positions.get(index)
                ),
                robot_vine_xyz=(
                    self.detected_tomato_record_stem_positions.get(index)
                ),
                robot_calyx_xyz=(
                    self.detected_tomato_record_calyx_positions.get(index)
                ),
                robot_angle_origin_xyz=(
                    self.detected_tomato_record_angle_origins.get(index)
                ),
                robot_angle_target_xyz=(
                    self.detected_tomato_record_angle_targets.get(index)
                ),
                review_issue=self.camera_review_issue_var.get(),
                review_note=self.camera_review_note_var.get(),
            )
            directory = Path(
                str(
                    self.get_parameter(
                        "camera_target_records_directory"
                    ).value
                )
            ).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / (
                timestamp.strftime("%Y%m%d_%H%M%S_%f")
                + f"_detected_tomato_{index}_feedback.txt"
            )
            path.write_text(contents, encoding="utf-8")
        except (OSError, TypeError, ValueError) as error:
            self.status.set("현재 Plan 대상 정보 저장 실패")
            self._append_log(f"[카메라 좌표 저장 실패] {error}")
            messagebox.showerror("카메라 좌표 저장 실패", str(error))
            return

        self.status.set(f"카메라 좌표 저장 완료: {path.name}")
        self._append_log(
            f"[카메라 좌표 저장] 대상={tomato_frame}, "
            f"camera_id={detection.id}, "
            f"XYZ=({center.x:.6f}, {center.y:.6f}, {center.z:.6f}) m, "
            f"상태={self._active_camera_target_state()}, "
            f"문제유형={self.camera_review_issue_var.get()}, 파일={path}"
        )
        self._send_debug_frame(contents, index, path)

    def _send_debug_frame(self, contents: str, index: int, path: Path) -> None:
        """Send the exact locally saved JSON through DebugFrame.srv."""
        try:
            request = debug_frame_request(contents)
        except ValueError as error:
            self.status.set("비전 피드백 JSON 전송 준비 실패")
            self._append_log(f"[비전 피드백 전송 실패] {error}")
            return
        if not self.debug_frame_client.service_is_ready():
            if not self.debug_frame_client.wait_for_service(timeout_sec=0.05):
                self.status.set(
                    f"JSON 저장 완료 / 서비스 연결 안 됨: "
                    f"{self.debug_frame_service}"
                )
                self._append_log(
                    "[비전 피드백 전송 보류] 로컬 JSON은 저장했지만 "
                    f"서비스가 연결되지 않았습니다: {self.debug_frame_service}"
                )
                return
        self.status.set("비전 피드백 JSON 저장 완료 / 서비스 응답 대기 중...")
        self._append_log(
            f"[비전 피드백 전송 요청] service={self.debug_frame_service}, "
            f"대상={self._tomato_frame(index)}, bytes="
            f"{len(contents.encode('utf-8'))}"
        )
        future = self.debug_frame_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._debug_frame_done(
                completed,
                index,
                path,
            )
        )

    def _debug_frame_done(self, future, index: int, path: Path) -> None:
        """Report the DebugFrame service response without losing local data."""
        try:
            response = future.result()
        except Exception as error:
            self.status.set("비전 피드백 JSON 서비스 호출 실패")
            self._append_log(
                f"[비전 피드백 전송 오류] 대상={self._tomato_frame(index)}, "
                f"오류={error}, 로컬파일={path}"
            )
            return
        message = str(response.message).strip()
        if response.success:
            self.status.set(
                message or f"비전 피드백 JSON 전송 완료: tomato {index}"
            )
            self._append_log(
                f"[비전 피드백 전송 성공] 대상={self._tomato_frame(index)}, "
                f"응답={message or '(메시지 없음)'}, 로컬파일={path}"
            )
            return
        self.status.set(message or "비전 피드백 JSON 전송 거부")
        self._append_log(
            f"[비전 피드백 전송 실패] 대상={self._tomato_frame(index)}, "
            f"응답={message or '(메시지 없음)'}, 로컬파일={path}"
        )

    def _selected_index(self) -> int | None:
        value = self.selected_tomato.get()
        if not value:
            return None
        try:
            return int(value.split(":", maxsplit=1)[0])
        except (ValueError, IndexError):
            return None

    def _tomato_frame(self, index: int) -> str:
        """Return the harvest TF assigned to one sorted camera detection."""
        tomato_index = int(index)
        if 0 <= tomato_index < len(self.detected_tomatoes):
            return harvest_tf_frame_id(
                self.detected_tomatoes[tomato_index].id,
                tomato_index,
            )
        return f"detected_tomato_{tomato_index}_tf"

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
        self._update_selected_tomato_plot()
        self._update_step_controls()
        self._update_camera_target_record_controls()

    def _selected_tomato_plan_report(self, index: int) -> dict:
        report = self.harvest_result_approach_reports.get(int(index))
        if report:
            return report
        if (
            self.active_camera_target_index == int(index)
            and self.active_camera_target_plan_report
        ):
            return self.active_camera_target_plan_report
        report = self.harvest_plan_report or {}
        if str(report.get("tomato_frame") or "") == self._tomato_frame(index):
            return report
        return {}

    def _update_selected_tomato_plot(self) -> None:
        plot = getattr(self, "selected_tomato_plot", None)
        if plot is None:
            return
        index = self._selected_index()
        if index is None:
            plot.clear("검출된 토마토를 선택하세요.")
            return
        tomato = self.detected_tomato_record_positions.get(index)
        vine = self.detected_tomato_record_stem_positions.get(index)
        calyx = self.detected_tomato_record_calyx_positions.get(index)
        angle_origin = self.detected_tomato_record_angle_origins.get(index)
        angle_target = self.detected_tomato_record_angle_targets.get(index)
        if (
            tomato is None
            or vine is None
            or calyx is None
            or angle_origin is None
            or angle_target is None
        ):
            plot.clear(
                "선택 토마토의 link0 좌표가 없습니다.\n"
                "카메라 검출을 다시 실행하세요."
            )
            return
        try:
            scene = tomato_selection_plot_scene(
                tomato,
                vine,
                calyx,
                angle_origin,
                angle_target,
                self._selected_tomato_plan_report(index),
            )
        except (TypeError, ValueError) as error:
            plot.clear(f"선택 토마토 그래프 생성 실패:\n{error}")
            return
        plot.set_scene(scene)

    def _selected_planner_config(self) -> tuple[str, str, str]:
        """Return the production GUI's fixed Cartesian-first configuration."""
        return GUI_PLANNER_CONFIG

    def _selected_pick_ready_state(self) -> str:
        state_name = str(self.pick_ready_state_var.get())
        if state_name not in PICK_READY_STATES:
            raise ValueError(f"지원하지 않는 시작 자세입니다: {state_name}")
        capture_pose = self.detection_capture_pose
        if capture_pose not in ("CAPTURE_LEFT", "CAPTURE_RIGHT"):
            selected_pose = str(self.named_pose_var.get()).strip()
            capture_pose = (
                selected_pose
                if selected_pose in ("CAPTURE_LEFT", "CAPTURE_RIGHT")
                else self.last_completed_named_pose
                if self.last_completed_named_pose
                in ("CAPTURE_LEFT", "CAPTURE_RIGHT")
                else None
            )
        index = self._selected_index()
        if index is None and self.detected_tomato_record_positions:
            index = min(self.detected_tomato_record_positions)
        tomato = self.detected_tomato_record_positions.get(index)
        target_x = float(tomato[0]) if tomato is not None else None
        resolved = pick_ready_state_for_capture_pose(
            capture_pose,
            state_name,
            target_x,
        )
        if resolved != state_name:
            self.pick_ready_state_var.set(resolved)
        return resolved

    def _named_pose_selection_changed(self, _event=None) -> None:
        pose = str(self.named_pose_var.get()).strip()
        if pose not in ("CAPTURE_LEFT", "CAPTURE_RIGHT"):
            return
        ready_state = pick_ready_state_for_capture_pose(pose)
        self.pick_ready_state_var.set(ready_state)
        self._invalidate_plan()
        self.status.set(
            f"촬영 자세 {pose} 선택 — 수확 시작/복귀를 "
            f"{ready_state}(으)로 설정했습니다."
        )
        self._append_log(
            f"[촬영 자세 선택] {pose} → 시작/복귀 {ready_state}"
        )

    def _pick_ready_state_changed(self, _event=None) -> None:
        state_name = self._selected_pick_ready_state()
        self._invalidate_plan()
        self.status.set(
            f"시작/복귀 자세를 {state_name}(으)로 변경했습니다. "
            "Plan-only를 다시 실행하세요."
        )
        self._append_log(
            f"수확 시작/복귀 자세 선택: {state_name} "
            "(다음 Plan부터 적용)"
        )

    def _adaptive_grasp_options(self) -> tuple[bool, float]:
        return (
            bool(self.prefer_robot_direction_var.get()),
            adaptive_grasp_max_rotation_degrees(
                self.adaptive_grasp_max_rotation_var.get()
            ),
        )

    def _step_wrist_oscillation_changed(self) -> None:
        """Invalidate cached step plans when stage 5 is toggled."""
        self._invalidate_plan()
        enabled = bool(self.step_tcp_wrist_oscillation_enabled_var.get())
        if hasattr(self, "tcp_wrist_rotation_spinbox"):
            self.tcp_wrist_rotation_spinbox.configure(
                state=(
                    "normal"
                    if enabled and not self.ui_busy and self.step_process is None
                    else "disabled"
                )
            )
        self.status.set(
            "스텝 5 TCP 좌우 흔들기 사용 — 스텝 Plan을 다시 생성하세요."
            if enabled
            else "스텝 5 TCP 좌우 흔들기 제외 — 스텝 Plan을 다시 생성하세요."
        )
        self._append_log(
            f"[스텝 옵션] TCP 좌우 흔들기: "
            f"{'사용' if enabled else '사용 안 함'}"
        )
        self._update_step_controls()

    def _step_stage_speed_changed(self, stage_number: int, value) -> None:
        """Keep the detailed-stage speed slider on whole percentages."""
        number = int(stage_number)
        if number not in self.step_stage_speed_percent_vars:
            return
        try:
            percent = min(
                STEP_CUSTOM_SPEED_MAX_PERCENT,
                max(STEP_CUSTOM_SPEED_MIN_PERCENT, round(float(value))),
            )
        except (TypeError, ValueError):
            return
        self.step_stage_speed_percent_vars[number].set(percent)
        self.step_stage_speed_display_vars[number].set(f"{percent:.0f}%")

    def _step_preapproach_final_speed_changed(self, value) -> None:
        """Round and display the A-to-PRE_APPROACH-only speed."""
        try:
            percent = min(
                STEP_CUSTOM_SPEED_MAX_PERCENT,
                max(STEP_CUSTOM_SPEED_MIN_PERCENT, round(float(value))),
            )
        except (TypeError, ValueError):
            return
        self.step_preapproach_final_speed_percent_var.set(percent)
        self.step_preapproach_final_speed_display_var.set(
            f"{percent:.0f}%"
        )

    def _step_preapproach_via_changed(self) -> None:
        """Refresh controls after selecting via-A or direct approach."""
        enabled = bool(self.step_preapproach_via_enabled_var.get())
        self.status.set(
            "2단계: 경유점 A를 거쳐 PRE_APPROACH로 이동합니다."
            if enabled
            else "2단계: 경유점 A 없이 PRE_APPROACH로 직접 이동합니다."
        )
        self._append_log(
            "[스텝 옵션] PRE_APPROACH 진입: "
            + ("경유점 A 사용" if enabled else "Direct")
        )
        self._update_step_controls()

    def _step_merge_option_changed(self, selected: str) -> None:
        """Keep the two mutually exclusive trajectory merge modes clear."""
        if selected == "cartesian" and self.step_merge_cartesian_var.get():
            self.step_merge_all_trajectories_var.set(False)
        elif selected == "all" and self.step_merge_all_trajectories_var.get():
            self.step_merge_cartesian_var.set(False)
        self._update_step_controls()

    def _step_forward_wave_changed(self) -> None:
        """Invalidate cached step plans when the stage-4 wave is toggled."""
        self._invalidate_plan()
        enabled = bool(self.step_forward_wave_enabled_var.get())
        self.status.set(
            "스텝 4 Z축 웨이브 사용 — 스텝 Plan을 다시 생성하세요."
            if enabled
            else "스텝 4 직선 Cartesian 사용 — 스텝 Plan을 다시 생성하세요."
        )
        self._append_log(
            f"[스텝 옵션] 4단계 Z축 웨이브: "
            f"{'사용 (±5mm × 3회)' if enabled else '사용 안 함'}"
        )
        self._update_step_controls()

    def _linear_motor_wait_changed(self, _event=None) -> None:
        try:
            wait_seconds = self._wait_seconds(
                self.linear_motor_wait_sec.get()
            )
        except ValueError as error:
            self._invalidate_plan()
            self.status.set(str(error))
            return
        self._invalidate_plan()
        self.status.set(
            f"리니어모터 대기시간 {wait_seconds:g}초 — "
            "Plan-only를 다시 실행하세요."
        )
        self._append_log(
            f"리니어모터 대기시간 변경: {wait_seconds:g}초"
        )

    def _harvest_forward_distance_changed(self, _event=None) -> None:
        try:
            forward_distance_m = repeat_forward_distance_m(
                self.harvest_forward_distance_mm_var.get()
            )
        except ValueError as error:
            self._invalidate_plan()
            self.status.set(str(error))
            return
        self._invalidate_plan()
        self.status.set(
            f"수확 4단계 진입 길이 {forward_distance_m * 1000.0:g}mm — "
            "Plan-only를 다시 실행하세요."
        )
        self._append_log(
            "수확 4단계 진입 길이 변경: "
            f"{forward_distance_m * 1000.0:g}mm"
        )

    def _adaptive_grasp_mode_changed(self, _event=None) -> None:
        try:
            prefer_robot, maximum = self._adaptive_grasp_options()
        except ValueError as error:
            self._invalidate_plan()
            self.status.set(str(error))
            return
        self._invalidate_plan()
        mode = "로봇 방향 우선" if prefer_robot else "Recommend 우선"
        self.status.set(
            f"진입각 선택: {mode}, 최대 보정각 {maximum:g}° — "
            "Plan-only를 다시 실행하세요."
        )
        self._append_log(
            f"수확 진입각 옵션 변경: mode={mode}, "
            f"max_rotation={maximum:g}°"
        )

    def _invalidate_plan(self) -> None:
        self.verified_plan = None
        self.execute_button.configure(state="disabled")

    def _verification_matches_current_selection(self, verification) -> bool:
        if verification is None:
            return False
        option_reader = getattr(self, "_adaptive_grasp_options", None)
        if option_reader is None:
            prefer_robot_direction, adaptive_max_rotation = False, 45.0
        else:
            prefer_robot_direction, adaptive_max_rotation = option_reader()
        expected = (
            self.detection_generation,
            self._selected_index(),
            *self._selected_planner_config(),
            self._selected_pick_ready_state(),
            prefer_robot_direction,
            adaptive_max_rotation,
        )
        # Step sessions append one Boolean for the optional wrist oscillation.
        # Handle it before the legacy extended-plan tuple below, where the next
        # element is a numeric forward distance.  Without this distinction a
        # Boolean was parsed as a distance and every cached step plan appeared
        # invalid even though no operator setting had changed.
        if (
            len(verification) == len(expected) + 2
            and all(type(value) is bool for value in verification[-2:])
        ):
            wrist_option = getattr(
                self,
                "step_tcp_wrist_oscillation_enabled_var",
                None,
            )
            wave_option = getattr(
                self,
                "step_forward_wave_enabled_var",
                None,
            )
            if wrist_option is None or wave_option is None:
                return False
            return verification == expected + (
                bool(wrist_option.get()),
                bool(wave_option.get()),
            )
        if (
            len(verification) == len(expected) + 1
            and type(verification[-1]) is bool
        ):
            wrist_option = getattr(
                self,
                "step_tcp_wrist_oscillation_enabled_var",
                None,
            )
            if wrist_option is None:
                return False
            return verification == expected + (bool(wrist_option.get()),)
        if len(verification) > len(expected):
            distance_variable = getattr(
                self, "harvest_forward_distance_mm_var", None
            )
            if distance_variable is None:
                return False
            try:
                forward_distance_m = repeat_forward_distance_m(
                    distance_variable.get()
                )
            except ValueError:
                return False
            wrist_variable = getattr(
                self, "tcp_wrist_rotation_deg_var", None
            )
            wrist_rotation = (
                float(wrist_variable.get())
                if wrist_variable is not None
                else 10.0
            )
            extended = expected + (forward_distance_m, wrist_rotation)
            if len(verification) == len(extended):
                return verification == extended
            return verification == expected + (forward_distance_m,)
        if len(verification) > 6:
            return verification == expected
        if len(verification) > 5:
            return verification == expected[:6]
        return (
            verification == expected[:5]
            and expected[5] == "PICK_READY"
        )

    def _lift_harvest_mode_changed(self) -> None:
        self._invalidate_plan()
        enabled = bool(self.lift_harvest_var.get())
        if enabled and bool(self.preplan_all_var.get()):
            self.preplan_all_var.set(False)
            self._append_log(
                "리프트 수확에서는 전체 모션 사전계획을 사용할 수 없어 "
                "사전계획 옵션을 해제했습니다."
            )
        self._update_preplan_checkbox_state()
        self.status.set(
            "리프트 수확 모드 활성화 — Plan-only를 다시 실행하세요."
            if enabled
            else "리프트 수확 모드 해제 — Plan-only를 다시 실행하세요."
        )

    def _preplan_all_mode_changed(self) -> None:
        enabled = bool(self.preplan_all_var.get())
        if enabled and bool(self.lift_harvest_var.get()):
            self.preplan_all_var.set(False)
            messagebox.showwarning(
                "사전계획 사용 불가",
                "리프트 수확에서는 전체 모션 사전계획을 사용할 수 "
                "없습니다. 리프트 수확을 해제한 뒤 사용하세요.",
            )
            return
        self._invalidate_plan()
        self.status.set(
            "전체 모션 사전계획 활성화 — 전체 연속 수확 시작 전에 "
            "모든 trajectory를 계산합니다."
            if enabled
            else "전체 모션 사전계획 해제"
        )

    def _update_preplan_checkbox_state(self) -> None:
        disabled = self.ui_busy or bool(self.lift_harvest_var.get())
        self.preplan_all_checkbox.configure(
            state="disabled" if disabled else "normal"
        )

    def _handle_lift_preparation_error(self, message: str) -> None:
        self.lift_harvest_pending = None
        self._append_log(f"[리프트 수확 실패] {message}")
        self.status.set(f"리프트 수확 준비 실패 — {message}")
        if self.batch_active:
            self._finish_batch(False, f"리프트 수확 준비 실패: {message}")
        elif self.sweep_active:
            self._finish_sweep(f"리프트 수확 준비 실패: {message}")
        else:
            self._set_busy(False)
            messagebox.showerror("리프트 수확 준비 실패", message)

    def _handle_sweep_lift_preparation_error(
        self,
        tomato_index: int,
        verification,
        message: str,
    ) -> None:
        """Record a detection-TF race without aborting the whole sweep."""
        text = str(message)
        tf_sync_failure = (
            text.startswith("detected_tomato_")
            or text.startswith("새 검출 결과에 대한 토마토 TF")
            or "지면 기준 높이를 계산하지 못했습니다" in text
        )
        if not tf_sync_failure:
            self._handle_lift_preparation_error(text)
            return

        self.lift_harvest_pending = None
        self._append_log(
            f"[리프트 TF 동기화 실패] 토마토 {tomato_index}: {text} — "
            "해당 결과를 실패로 기록하고 자동 테스트를 계속합니다."
        )
        report = {
            "failure_stage": "LIFT_TF_SYNC",
            "failure_planner_type": "tf",
            "failure_reason": text,
            "execution_attempted": False,
            "suppress_result_marker": True,
            "stages": [
                {
                    "stage": "LIFT_TF_SYNC",
                    "planner_type": "tf",
                    "success": False,
                    "reason": text,
                }
            ],
        }
        self._handle_sweep_plan_done(1, verification, report)

    def start_step_session(self) -> None:
        self._start_step_session("manual")

    def start_repeat_session(self) -> None:
        self.repeat_run_direction = None
        self.repeat_pause_requested = False
        self.repeat_paused = False
        self.repeat_pause_button.configure(text="일시 정지")
        self._start_step_session("repeat")

    def _repeat_last_stage_number(self) -> int:
        try:
            stage_number = int(self.repeat_last_stage_var.get())
        except (TypeError, ValueError) as error:
            raise ValueError("반복 마지막 단계는 1~7 중에서 선택하세요.") from error
        if not 1 <= stage_number <= 7:
            raise ValueError("반복 마지막 단계는 1~7 중에서 선택하세요.")
        return stage_number

    def _repeat_last_stage_changed(self, _event=None) -> None:
        try:
            stage_number = self._repeat_last_stage_number()
        except ValueError:
            return
        self.repeat_forward_button.configure(
            text=f"1 → {stage_number} 연속 진입"
        )
        self.repeat_reverse_button.configure(
            text=f"{stage_number} → 1 역순 복귀"
        )
        self.repeat_all_button.configure(
            text=f"전체 토마토 1 → {stage_number} → 1"
        )

    def start_repeat_all(self) -> None:
        if (
            self.harvest_process is not None
            or self.step_process is not None
            or self.batch_active
            or self.sweep_active
            or self.repeat_batch_active
        ):
            messagebox.showinfo(
                "실행 중",
                "현재 모션 작업 또는 스텝 세션을 먼저 종료하세요.",
            )
            return
        if not self.detected_tomatoes:
            messagebox.showwarning(
                "토마토 검출",
                "접근 반복 테스트할 토마토를 먼저 검출하세요.",
            )
            return
        if not self.repeat_execution_enabled_var.get():
            messagebox.showwarning(
                "실제 실행 잠금",
                "'실제 로봇 반복 실행 허용'을 먼저 체크하세요.",
            )
            return
        try:
            stage_number = self._repeat_last_stage_number()
            forward_distance_m = repeat_forward_distance_m(
                self.repeat_forward_distance_mm_var.get()
            )
            self._wait_seconds(self.linear_motor_wait_sec.get())
            self._selected_pick_ready_state()
        except ValueError as error:
            messagebox.showerror("접근 반복 설정 오류", str(error))
            return
        tomato_count = len(self.detected_tomatoes)
        if not messagebox.askyesno(
            "전체 토마토 접근 반복 실행",
            f"검출된 토마토 {tomato_count}개에 대해 각각 "
            f"1→{stage_number}→1 동작을 실제 로봇에서 실행할까요?\n\n"
            "각 토마토마다 새 Plan을 계산합니다. 실제 trajectory 실행이 "
            "실패하면 안전을 위해 전체 작업을 중단합니다.",
            icon="warning",
        ):
            return

        self.repeat_cycle_last_index = stage_number - 1
        self.repeat_cycle_forward_distance_m = forward_distance_m
        self.repeat_batch_active = True
        self.repeat_batch_queue = deque(range(tomato_count))
        self.repeat_batch_total = tomato_count
        self.repeat_batch_completed = 0
        self.repeat_batch_plan_failures = 0
        self.repeat_batch_current_index = None
        self.repeat_batch_current_outcome = None
        self.repeat_run_direction = None
        self.repeat_pause_requested = False
        self.repeat_paused = False
        self.repeat_pause_button.configure(text="일시 정지")
        self._append_log(
            f"[전체 접근 반복 시작] 토마토 {tomato_count}개, "
            f"각 토마토 1→{stage_number}→1, "
            f"4단계 진입={forward_distance_m * 1000.0:.1f} mm"
        )
        self.status.set("전체 토마토 접근 반복 테스트 시작...")
        self._set_busy(True)
        self.root.after(50, self._start_next_repeat_batch_tomato)

    def _start_next_repeat_batch_tomato(self) -> None:
        if not self.repeat_batch_active:
            return
        if self.repeat_pause_requested:
            self.repeat_paused = True
            self.repeat_status.set(
                "전체 접근 반복 일시 정지 — 계속 실행을 누르면 다음 "
                "토마토부터 재개합니다."
            )
            self.status.set(self.repeat_status.get())
            self._update_step_controls()
            return
        if not self.repeat_batch_queue:
            success_count = (
                self.repeat_batch_completed
                - self.repeat_batch_plan_failures
            )
            self._finish_repeat_batch(
                True,
                f"전체 접근 반복 완료: 성공 {success_count}개, "
                f"Plan 실패 {self.repeat_batch_plan_failures}개",
            )
            return

        index = int(self.repeat_batch_queue.popleft())
        self.repeat_batch_current_index = index
        self.repeat_batch_current_outcome = "planning"
        self.tomato_combo.current(index)
        self.tomato_tree.selection_set(str(index))
        self.tomato_tree.focus(str(index))
        self.tomato_tree.see(str(index))
        stage_number = self.repeat_cycle_last_index + 1
        progress = self.repeat_batch_completed + 1
        self.repeat_status.set(
            f"전체 {progress}/{self.repeat_batch_total}: 토마토 {index} "
            f"1→{stage_number}→1 Plan 계산 중..."
        )
        if not self._start_step_session("repeat"):
            self._finish_repeat_batch(
                False,
                f"토마토 {index} 반복 Planner를 시작하지 못했습니다.",
            )

    def _finish_repeat_batch(self, success: bool, message: str) -> None:
        self.repeat_batch_active = False
        self.repeat_batch_queue.clear()
        self.repeat_batch_current_index = None
        self.repeat_batch_current_outcome = None
        self.repeat_run_direction = None
        self.repeat_pause_requested = False
        self.repeat_paused = False
        self.step_session_mode = None
        self.step_session_verification = None
        self.step_stages = []
        self.step_next_index = 0
        self.step_execution_in_progress = False
        self.step_execution_confirmed = False
        self._clear_active_camera_target()
        self._set_busy(False)
        if hasattr(self, "repeat_pause_button"):
            self.repeat_pause_button.configure(text="일시 정지")
        self.repeat_status.set(message)
        self.status.set(message)
        result = "완료" if success else "중단"
        self._append_log(f"[전체 접근 반복 {result}] {message}")

    def _start_step_session(self, mode: str) -> bool:
        index = self._selected_index()
        if index is None:
            messagebox.showwarning(
                "토마토 선택",
                "반복 테스트할 토마토를 먼저 선택하세요."
                if mode == "repeat"
                else "스텝 실행할 토마토를 먼저 선택하세요.",
            )
            return False
        if (
            self.harvest_process is not None
            or self.step_process is not None
            or self.batch_active
            or self.sweep_active
        ):
            messagebox.showinfo(
                "실행 중",
                "현재 모션 작업 또는 스텝 세션을 먼저 종료하세요.",
            )
            return False
        cycle_forward_distance_m = 0.040
        custom_stage_deltas_m = None
        stage_speed_percents = tuple(
            STEP_CUSTOM_SPEED_DEFAULT_PERCENT_BY_STAGE[stage_number]
            for stage_number in STEP_CUSTOM_SPEED_STAGE_NUMBERS
        )
        preapproach_final_speed_percent = (
            STEP_PREAPPROACH_FINAL_SPEED_DEFAULT_PERCENT
        )
        preapproach_via_enabled = bool(
            self.step_preapproach_via_enabled_var.get()
        )
        try:
            harvest_wait_sec = self._wait_seconds(
                self.linear_motor_wait_sec.get()
            )
            pick_ready_state = pick_ready_state_for_capture_pose(
                self.detection_capture_pose,
                self._selected_pick_ready_state(),
            )
            if self.pick_ready_state_var.get() != pick_ready_state:
                self.pick_ready_state_var.set(pick_ready_state)
            prefer_robot_direction, adaptive_max_rotation = (
                self._adaptive_grasp_options()
            )
            tcp_wrist_oscillation_enabled = bool(
                self.step_tcp_wrist_oscillation_enabled_var.get()
            )
            tcp_wrist_rotation_deg = 10.0
            if tcp_wrist_oscillation_enabled:
                tcp_wrist_rotation_deg = float(
                    self.tcp_wrist_rotation_deg_var.get()
                )
                if not 1.0 <= tcp_wrist_rotation_deg <= 45.0:
                    raise ValueError(
                        "5단계 TCP 회전각은 1~45도 범위여야 합니다."
                    )
            servo_close_angle_deg = servo_angle_degrees(
                self.step_servo_close_angle_deg_var.get()
            )
            linear_motor_extend_seconds = (
                step_linear_motor_extend_seconds(
                    self.step_linear_motor_extend_sec_var.get()
                )
            )
            forward_wave_enabled = bool(
                self.step_forward_wave_enabled_var.get()
            )
            if mode == "repeat":
                self.repeat_cycle_last_index = (
                    self._repeat_last_stage_number() - 1
                )
                self.repeat_cycle_forward_distance_m = (
                    repeat_forward_distance_m(
                        self.repeat_forward_distance_mm_var.get()
                    )
                )
                cycle_forward_distance_m = (
                    self.repeat_cycle_forward_distance_m
                )
            else:
                custom_stage_deltas_m = step_custom_stage_deltas_m(
                    {
                        stage_number: {
                            axis_name: variable.get()
                            for axis_name, variable in axis_variables.items()
                        }
                        for stage_number, axis_variables in (
                            self.step_custom_delta_vars.items()
                        )
                    }
                )
                stage_speed_percents = step_stage_speed_percents(
                    {
                        stage_number: variable.get()
                        for stage_number, variable in (
                            self.step_stage_speed_percent_vars.items()
                        )
                    }
                )
                preapproach_final_speed_percent = float(
                    self.step_preapproach_final_speed_percent_var.get()
                )
                if not (
                    STEP_CUSTOM_SPEED_MIN_PERCENT
                    <= preapproach_final_speed_percent
                    <= STEP_CUSTOM_SPEED_MAX_PERCENT
                ):
                    raise ValueError(
                        "A→PRE_APPROACH 속도는 10~100% 범위여야 합니다."
                    )
        except ValueError as error:
            messagebox.showerror("스텝 Plan 설정 오류", str(error))
            return False
        pipeline, planner_id, preapproach_mode = (
            self._selected_planner_config()
        )
        command = stepper_command(
            index,
            tomato_frame=self._tomato_frame(index),
            planning_pipeline_id=pipeline,
            planner_id=planner_id,
            preapproach_mode=preapproach_mode,
            velocity_scale=self.motion_velocity_scale,
            acceleration_scale=self.motion_acceleration_scale,
            harvest_wait_sec=harvest_wait_sec,
            pick_ready_state_name=pick_ready_state,
            cycle_only=(mode == "repeat"),
            cycle_last_stage=self.repeat_cycle_last_index + 1,
            cycle_forward_distance_m=cycle_forward_distance_m,
            tcp_wrist_oscillation_enabled=(
                tcp_wrist_oscillation_enabled
            ),
            tcp_wrist_rotation_deg=tcp_wrist_rotation_deg,
            custom_stage_deltas_m=custom_stage_deltas_m,
            stage_speed_percents=stage_speed_percents,
            preapproach_final_speed_percent=(
                preapproach_final_speed_percent
            ),
            preapproach_via_enabled=preapproach_via_enabled,
            prefer_robot_direction=prefer_robot_direction,
            adaptive_grasp_max_rotation_deg=adaptive_max_rotation,
            servo_speed_percent=(
                servo_speed_percent(self.servo10_speed_percent_var.get())
                if self.servo10_speed_enabled_var.get()
                else SERVO10_DEFAULT_SPEED_PERCENT
            ),
            servo_close_angle_deg=servo_close_angle_deg,
            linear_motor_extend_seconds=linear_motor_extend_seconds,
            forward_wave_enabled=forward_wave_enabled,
        )
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
            messagebox.showerror(
                "스텝 Planner 시작 실패",
                str(error),
            )
            return False

        self.step_process = process
        self.step_stages = []
        self.step_next_index = 0
        self.step_execution_in_progress = False
        self.step_execution_confirmed = self.repeat_batch_active
        self.step_session_failed = False
        self.step_session_mode = mode
        self.step_session_verification = (
            self.detection_generation,
            index,
            pipeline,
            planner_id,
            preapproach_mode,
            pick_ready_state,
            prefer_robot_direction,
            adaptive_max_rotation,
            tcp_wrist_oscillation_enabled,
            forward_wave_enabled,
        )
        self._set_active_camera_target(
            index,
            "접근 반복 테스트" if mode == "repeat" else "스텝 실행",
        )
        for item in self.step_tree.get_children():
            self.step_tree.delete(item)
        for item in self.repeat_tree.get_children():
            self.repeat_tree.delete(item)
        if mode == "repeat":
            stage_number = self.repeat_cycle_last_index + 1
            self.repeat_status.set(
                f"토마토 {index} 접근 1→{stage_number}→1 Plan 계산 중..."
            )
            self.status.set(
                "접근 반복 Plan 계산 중 — 로봇은 움직이지 않습니다."
            )
        else:
            self.step_status.set(f"토마토 {index} 전체 스텝 Plan 계산 중...")
            self.status.set("스텝 Plan 계산 중 — 로봇은 움직이지 않습니다.")
        self._append_log(
            f"[{'접근 반복' if mode == 'repeat' else '스텝'} Plan] "
            f"토마토 {index}, 시작={pick_ready_state}, "
            f"planner={pipeline}/{planner_id}, preapproach={preapproach_mode}"
            + (
                f", 4단계 진입={cycle_forward_distance_m * 1000.0:.1f} mm"
                if mode == "repeat"
                else ", 3/4/6/7/8단계 tip 로컬 XYZ="
                + str(
                    [
                        [round(value * 1000.0, 3) for value in stage_delta]
                        for stage_delta in custom_stage_deltas_m
                    ]
                )
                + (
                    f" mm, 5단계 TCP 좌우 흔들기 "
                    f"±{tcp_wrist_rotation_deg:g}°"
                    if tcp_wrist_oscillation_enabled
                    else " mm, 5단계 TCP 좌우 흔들기 사용 안 함"
                )
            )
        )
        self._set_busy(True)
        threading.Thread(
            target=self._read_step_process,
            args=(process,),
            daemon=True,
        ).start()
        return True

    def _read_step_process(self, process) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                line = line.rstrip()
                if line.startswith(STEPPER_EVENT_PREFIX):
                    try:
                        event = json.loads(
                            line[len(STEPPER_EVENT_PREFIX):]
                        )
                    except json.JSONDecodeError as error:
                        self.process_queue.put(
                            ("log", f"스텝 이벤트 해석 실패: {error}")
                        )
                        continue
                    self.process_queue.put(
                        ("stepper_event", event, process)
                    )
                elif is_critical_process_output(line):
                    self.process_queue.put(("log", line))
        return_code = process.wait()
        self.process_queue.put(("stepper_done", return_code, process))

    def _handle_stepper_event(self, event: dict, process) -> None:
        if process is not self.step_process:
            return
        event_report = event.get("report") or {}
        if event_report:
            self._set_active_camera_target_plan_report(event_report)
        if self.step_session_mode == "repeat":
            self._handle_repeat_event(event)
            self._update_step_controls()
            return
        event_name = str(event.get("event", ""))
        if event_name == "planned":
            self.step_stages = list(event.get("stages", []))
            self.step_next_index = 0
            report = event.get("report") or {}
            self.harvest_plan_report = report
            for stage in self.step_stages:
                index = int(stage["index"])
                self.step_tree.insert(
                    "",
                    "end",
                    iid=str(index),
                    values=(
                        index + 1,
                        stage.get("label", ""),
                        stage.get("detail", ""),
                        "실행 대기",
                    ),
                )
            if self.step_stages:
                self.step_tree.selection_set("0")
                self.step_tree.focus("0")
            tomato_index = self.step_session_verification[1]
            self._set_harvest_result(
                tomato_index,
                True,
                adaptive_rotation_applied=adaptive_rotation_was_applied(
                    report
                ),
                adaptive_rotation_deg=adaptive_rotation_degrees(report),
                approach_axis_local=adaptive_approach_axis_local(report),
                plan_report=report,
                motion_result_text="스텝 Plan 완료",
            )
            self.step_status.set(
                f"스텝 Plan 완료 — 총 {len(self.step_stages)}단계"
            )
            self.status.set(
                "스텝 Plan 완료 — 실행 허용 후 다음 단계를 실행하세요."
            )
            self._append_log(
                f"[스텝 Plan 완료] {len(self.step_stages)}단계 캐시됨. "
                "로봇은 아직 움직이지 않았습니다."
            )
            self.show_preapproach_goal_state(quiet=True)
        elif event_name == "plan_failed":
            report = event.get("report") or {}
            self.step_status.set(
                "스텝 Plan 실패 — " + harvest_failure_summary(report)
            )
            self.status.set("스텝 Plan 실패")
            self._append_log(
                "[스텝 Plan 실패] " + harvest_failure_summary(report)
            )
        elif event_name == "stage_started":
            index = int(event.get("index", -1))
            reverse = str(event.get("direction", "forward")) == "reverse"
            self.step_execution_in_progress = True
            if self.step_tree.exists(str(index)):
                self.step_tree.set(
                    str(index),
                    "status",
                    "역순 실행 중" if reverse else "실행 중",
                )
                self.step_tree.selection_set(str(index))
                self.step_tree.see(str(index))
            self.step_status.set(
                f"{index + 1}단계 "
                f"{'역순 ' if reverse else ''}실행 중: "
                f"{event.get('label', '')}"
            )
            self.status.set(self.step_status.get())
        elif event_name == "stage_completed":
            index = int(event.get("index", -1))
            reverse = str(event.get("direction", "forward")) == "reverse"
            self.step_execution_in_progress = False
            self.step_next_index = int(
                event.get("next_index", index if reverse else index + 1)
            )
            duration = float(event.get("duration_sec", 0.0))
            if self.step_tree.exists(str(index)):
                self.step_tree.set(
                    str(index),
                    "status",
                    (
                        f"역순 복귀 ({duration:.2f}s)"
                        if reverse
                        else f"완료 ({duration:.2f}s)"
                    ),
                )
            if self.step_next_index < len(self.step_stages):
                next_item = str(self.step_next_index)
                self.step_tree.selection_set(next_item)
                self.step_tree.focus(next_item)
                self.step_tree.see(next_item)
            self._append_log(
                f"[스텝 {index + 1}/{len(self.step_stages)} "
                f"{'역순 복귀' if reverse else '완료'}] "
                f"{duration:.2f}s"
            )
        elif event_name == "continuous_planning":
            blocks = list(event.get("blocks", []))
            merge_mode = str(event.get("mode", "cartesian"))
            mode_label = (
                "OMPL 포함 전체경로"
                if merge_mode == "ompl_cartesian"
                else "연속 Cartesian 경로"
            )
            if blocks:
                ranges = ", ".join(
                    f"{int(block['start_index']) + 1}~"
                    f"{int(block['end_index']) + 1}단계"
                    for block in blocks
                )
                message = f"{mode_label} 결합 중: {ranges}"
            else:
                message = (
                    f"합칠 수 있는 {mode_label} 구간이 없어 "
                    "기존 단계 실행을 사용합니다."
                )
            self.step_status.set(message)
            self.status.set(message)
            self._append_log(f"[스텝 연속 경로] {message}")
        elif event_name == "continuous_started":
            start_index = int(event.get("start_index", -1))
            end_index = int(event.get("end_index", start_index))
            merge_mode = str(event.get("mode", "cartesian"))
            mode_label = (
                "OMPL+Cartesian 통합"
                if merge_mode == "ompl_cartesian"
                else "연속 Cartesian"
            )
            self.step_execution_in_progress = True
            for index in event.get(
                "stage_indices", range(start_index, end_index + 1)
            ):
                if self.step_tree.exists(str(index)):
                    self.step_tree.set(str(index), "status", "연속 실행 중")
            if self.step_tree.exists(str(start_index)):
                self.step_tree.selection_set(str(start_index))
                self.step_tree.see(str(start_index))
            message = (
                f"{start_index + 1}~{end_index + 1}단계 "
                f"{mode_label} 실행 중"
            )
            if merge_mode == "ompl_cartesian":
                minimum_speed = event.get(
                    "requested_speed_min_percent"
                )
                maximum_speed = event.get(
                    "requested_speed_max_percent"
                )
                planned_duration = event.get("retimed_duration_sec")
                if (
                    minimum_speed is not None
                    and maximum_speed is not None
                    and planned_duration is not None
                ):
                    message += (
                        f" — 지정속도 {float(minimum_speed):.0f}~"
                        f"{float(maximum_speed):.0f}%, "
                        f"예상 {float(planned_duration):.2f}s"
                    )
            self.step_status.set(message)
            self.status.set(message)
            self._append_log(f"[스텝 통합 실행] {message}")
        elif event_name == "continuous_completed":
            start_index = int(event.get("start_index", -1))
            end_index = int(event.get("end_index", start_index))
            duration = float(event.get("duration_sec", 0.0))
            self.step_execution_in_progress = False
            self.step_next_index = end_index + 1
            for index in event.get(
                "stage_indices", range(start_index, end_index + 1)
            ):
                if self.step_tree.exists(str(index)):
                    self.step_tree.set(
                        str(index),
                        "status",
                        f"연속 완료 ({duration:.2f}s)",
                    )
            if self.step_next_index < len(self.step_stages):
                next_item = str(self.step_next_index)
                self.step_tree.selection_set(next_item)
                self.step_tree.focus(next_item)
                self.step_tree.see(next_item)
            self._append_log(
                f"[스텝 연속 실행 완료] {start_index + 1}~"
                f"{end_index + 1}단계 / {duration:.2f}s"
            )
        elif event_name == "continuous_failed":
            start_index = int(event.get("start_index", -1))
            end_index = int(event.get("end_index", start_index))
            reason = str(event.get("reason", "UNKNOWN"))
            detail = str(event.get("detail", "")).strip()
            start_error = event.get("start_error_deg")
            reason_text = reason
            if start_error is not None:
                reason_text += f" (시작 오차 {float(start_error):.2f}°)"
            if detail:
                reason_text += f" ({detail})"
            plan_only_failure = reason in {
                "CONTINUOUS_CARTESIAN_PLAN_FAILED",
                "OMPL_CARTESIAN_MERGE_FAILED",
            }
            self.step_execution_in_progress = False
            if not plan_only_failure:
                self.step_session_failed = True
            status_prefix = "연속 계획 실패" if plan_only_failure else "연속 실행 실패"
            for index in event.get(
                "stage_indices", range(start_index, end_index + 1)
            ):
                if self.step_tree.exists(str(index)):
                    self.step_tree.set(
                        str(index),
                        "status",
                        f"{status_prefix}: {reason_text}",
                    )
            message = (
                f"{start_index + 1}~{end_index + 1}단계 "
                f"{status_prefix}: {reason_text}"
            )
            if plan_only_failure:
                message += " — 체크 해제 후 기존 단계 실행 가능"
            self.step_status.set(message)
            self.status.set(message)
            self._append_log(f"[스텝 {status_prefix}] {message}")
        elif event_name == "stage_failed":
            index = int(event.get("index", -1))
            self.step_execution_in_progress = False
            self.step_session_failed = True
            reason = str(event.get("reason", "UNKNOWN"))
            start_error = event.get("start_error_deg")
            if start_error is not None:
                reason_text = f"{reason} (시작 오차 {float(start_error):.2f}°)"
            else:
                reason_text = reason
            if self.step_tree.exists(str(index)):
                self.step_tree.set(
                    str(index),
                    "status",
                    f"실패: {reason_text}",
                )
            self.step_status.set(f"{index + 1}단계 실패: {reason_text}")
            self.status.set(self.step_status.get())
            self._append_log(
                f"[스텝 실행 실패] 단계 {index + 1}: {reason_text}. "
                "현재 캐시는 더 실행할 수 없습니다."
            )
        elif event_name == "paused":
            self.step_execution_in_progress = False
            self.step_next_index = int(
                event.get("next_index", self.step_next_index)
            )
            if str(event.get("direction", "forward")) == "reverse":
                self.step_status.set(
                    f"{self.step_next_index + 1}단계 시작점으로 역순 복귀 — "
                    "정방향 재실행 또는 추가 역행 가능"
                )
            else:
                self.step_status.set(
                    f"{self.step_next_index}단계 완료 — 다음 단계 실행 대기"
                )
        elif event_name == "session_complete":
            self.step_execution_in_progress = False
            self.step_next_index = len(self.step_stages)
            self.step_status.set(
                "전체 스텝 실행 완료 — 이전 단계 역순 실행 또는 세션 종료 가능"
            )
            self.status.set(self.step_status.get())
            self._append_log(
                "[스텝 실행 완료] 전체 단계가 완료되었습니다. "
                "필요하면 바로 이전 단계를 역순 실행할 수 있습니다."
            )
        elif event_name in {"command_error", "internal_error"}:
            message = str(event.get("message", "알 수 없는 오류"))
            self.step_status.set(message)
            self._append_log(f"[스텝 프로세스 오류] {message}")
        elif event_name == "closed":
            self.step_status.set("스텝 세션 종료됨")
        self._update_step_controls()

    def toggle_repeat_pause(self) -> None:
        """Pause automatic repeat safely after the active cached stage."""
        automatic_active = bool(
            self.repeat_batch_active
            or self.repeat_run_direction in {"forward", "reverse"}
        )
        if not automatic_active:
            return
        if not self.repeat_pause_requested:
            self.repeat_pause_requested = True
            self.repeat_pause_button.configure(text="계속 실행")
            if self.step_execution_in_progress:
                message = "일시 정지 예약 — 현재 단계 완료 후 정지합니다."
            else:
                self.repeat_paused = True
                message = "접근 반복 일시 정지 — 계속 실행을 누르세요."
            self.repeat_status.set(message)
            self.status.set(message)
            self._append_log(f"[접근 반복 일시 정지] {message}")
            self._update_step_controls()
            return

        self.repeat_pause_requested = False
        self.repeat_paused = False
        self.repeat_pause_button.configure(text="일시 정지")
        self._append_log(
            "[접근 반복 계속 실행] 저장된 진행 단계부터 재개합니다."
        )
        if self.repeat_run_direction in {"forward", "reverse"}:
            self.root.after(20, self._continue_repeat_automation)
        elif self.repeat_batch_active:
            if self.step_process is None:
                self.root.after(20, self._start_next_repeat_batch_tomato)
            elif not self.step_stages:
                self.repeat_status.set(
                    "전체 접근 반복 재개 — 현재 토마토 Plan 계산 중..."
                )
            elif self.repeat_batch_current_outcome == "planned":
                self.repeat_run_direction = "forward"
                self.root.after(20, self._continue_repeat_automation)
        self._update_step_controls()

    def _continue_repeat_automation(self) -> None:
        """Send exactly one cached stage, allowing pause between stages."""
        direction = self.repeat_run_direction
        if direction not in {"forward", "reverse"}:
            return
        if self.step_execution_in_progress:
            return
        if self.repeat_pause_requested:
            self.repeat_paused = True
            arrow = (
                f"1→{self.repeat_cycle_last_index + 1}"
                if direction == "forward"
                else f"{self.repeat_cycle_last_index + 1}→1"
            )
            self.repeat_status.set(
                f"{arrow} 일시 정지 — 다음 실행 단계 "
                f"{self.step_next_index + 1}, 계속 실행을 누르세요."
            )
            self.status.set(self.repeat_status.get())
            self._update_step_controls()
            return

        command = repeat_stage_command(
            direction,
            self.step_next_index,
            self.repeat_cycle_last_index + 1,
        )
        if command is None:
            if direction == "forward":
                self._complete_repeat_forward_cycle()
            else:
                self._complete_repeat_reverse_cycle()
            return
        self.repeat_paused = False
        self._send_step_command(command)

    def _complete_repeat_forward_cycle(self) -> None:
        stage_number = self.repeat_cycle_last_index + 1
        self.step_execution_in_progress = False
        self._append_log(
            f"[접근 반복 정방향 완료] 1→{stage_number}단계가 "
            "연속 실행되었습니다."
        )
        if self.repeat_batch_active:
            self.repeat_run_direction = "reverse"
            self.repeat_status.set(
                f"1→{stage_number} 진입 완료 — {stage_number}→1 "
                "역순 복귀 준비"
            )
            self.status.set(self.repeat_status.get())
            self.root.after(20, self._continue_repeat_automation)
        else:
            self.repeat_run_direction = None
            self.repeat_status.set(
                f"1→{stage_number} 진입 완료 — {stage_number}→1 "
                "역순 복귀를 실행하세요."
            )
            self.status.set(self.repeat_status.get())
            self._update_step_controls()

    def _complete_repeat_reverse_cycle(self) -> None:
        stage_number = self.repeat_cycle_last_index + 1
        self.step_execution_in_progress = False
        self.step_next_index = 0
        self.repeat_run_direction = None
        for index in range(min(stage_number, len(self.step_stages))):
            if self.repeat_tree.exists(str(index)):
                self.repeat_tree.set(str(index), "status", "반복 대기")
        self.repeat_status.set(
            f"{stage_number}→1 역순 복귀 완료 — "
            f"1→{stage_number}를 다시 실행할 수 있습니다."
        )
        self.status.set(self.repeat_status.get())
        self._append_log(
            "[접근 반복 역방향 완료] 캐시된 동일 경로로 "
            f"{stage_number}→1단계 복귀했습니다."
        )
        if self.repeat_batch_active:
            self.repeat_batch_current_outcome = "success"
            tomato_index = self.repeat_batch_current_index
            if tomato_index is not None:
                self._set_harvest_result(
                    tomato_index,
                    True,
                    adaptive_rotation_applied=(
                        adaptive_rotation_was_applied(
                            self.harvest_plan_report
                        )
                    ),
                    adaptive_rotation_deg=adaptive_rotation_degrees(
                        self.harvest_plan_report
                    ),
                    approach_axis_local=adaptive_approach_axis_local(
                        self.harvest_plan_report
                    ),
                    plan_report=self.harvest_plan_report,
                    motion_result_text=f"접근 1→{stage_number}→1 완료",
                )
            self.root.after(50, self.close_step_session)
        else:
            self._update_step_controls()

    def _handle_repeat_event(self, event: dict) -> None:
        event_name = str(event.get("event", ""))
        stage_number = self.repeat_cycle_last_index + 1
        if event_name == "planned":
            self.step_stages = list(event.get("stages", []))
            self.step_next_index = 0
            report = event.get("report") or {}
            self.harvest_plan_report = report
            for stage in self.step_stages[:stage_number]:
                index = int(stage["index"])
                self.repeat_tree.insert(
                    "",
                    "end",
                    iid=str(index),
                    values=(
                        index + 1,
                        stage.get("label", ""),
                        stage.get("detail", ""),
                        "정방향 대기",
                    ),
                )
            tomato_index = self.step_session_verification[1]
            self._set_harvest_result(
                tomato_index,
                True,
                adaptive_rotation_applied=adaptive_rotation_was_applied(
                    report
                ),
                adaptive_rotation_deg=adaptive_rotation_degrees(report),
                approach_axis_local=adaptive_approach_axis_local(report),
                plan_report=report,
                motion_result_text="접근 반복 Plan 완료",
            )
            self.repeat_status.set(
                f"반복 Plan 완료 — 1→{stage_number} 연속 진입을 "
                "실행할 수 있습니다."
            )
            self.status.set(self.repeat_status.get())
            self._append_log(
                f"[접근 반복 Plan 완료] 1~{stage_number}단계 정방향과 "
                "역재생 경로가 "
                "캐시되었습니다. 로봇은 아직 움직이지 않았습니다."
            )
            if self.repeat_batch_active:
                self.repeat_batch_current_outcome = "planned"
                self.repeat_run_direction = "forward"
                self.root.after(50, self._continue_repeat_automation)
        elif event_name == "plan_failed":
            report = event.get("report") or {}
            self.repeat_status.set(
                "반복 Plan 실패 — " + harvest_failure_summary(report)
            )
            self.status.set("접근 반복 Plan 실패")
            self._append_log(
                "[접근 반복 Plan 실패] " + harvest_failure_summary(report)
            )
            if self.repeat_batch_active:
                self.repeat_batch_current_outcome = "plan_failed"
                tomato_index = self.repeat_batch_current_index
                if tomato_index is not None:
                    self._set_harvest_result(
                        tomato_index,
                        False,
                        plan_report=report,
                        motion_result_text="접근 반복 Plan 실패",
                    )
        elif event_name == "stage_started":
            index = int(event.get("index", -1))
            reverse = event.get("direction") == "reverse"
            self.step_execution_in_progress = True
            if self.repeat_tree.exists(str(index)):
                self.repeat_tree.set(
                    str(index),
                    "status",
                    "역방향 실행 중" if reverse else "정방향 실행 중",
                )
                self.repeat_tree.see(str(index))
            arrow = (
                f"{stage_number}→1" if reverse else f"1→{stage_number}"
            )
            self.repeat_status.set(
                f"{arrow} 실행 중 — {index + 1}단계: "
                f"{event.get('label', '')}"
            )
            self.status.set(self.repeat_status.get())
        elif event_name == "stage_completed":
            index = int(event.get("index", -1))
            reverse = event.get("direction") == "reverse"
            self.step_execution_in_progress = False
            self.step_next_index = index if reverse else index + 1
            duration = float(event.get("duration_sec", 0.0))
            if self.repeat_tree.exists(str(index)):
                self.repeat_tree.set(
                    str(index),
                    "status",
                    (
                        f"역복귀 완료 ({duration:.2f}s)"
                        if reverse
                        else f"진입 완료 ({duration:.2f}s)"
                    ),
                )
        elif event_name == "paused":
            self.step_execution_in_progress = False
            self.step_next_index = int(event.get("next_index", 5))
            event_direction = str(event.get("direction", "forward"))
            if self.repeat_run_direction == event_direction:
                self.root.after(20, self._continue_repeat_automation)
            elif event_direction == "forward":
                self._complete_repeat_forward_cycle()
            else:
                self._complete_repeat_reverse_cycle()
        elif event_name == "cycle_reset":
            self._complete_repeat_reverse_cycle()
        elif event_name == "stage_failed":
            index = int(event.get("index", -1))
            self.step_execution_in_progress = False
            self.step_session_failed = True
            direction = (
                "역방향" if event.get("direction") == "reverse" else "정방향"
            )
            reason = str(event.get("reason", "UNKNOWN"))
            start_error = event.get("start_error_deg")
            if start_error is not None:
                reason += f" (시작 오차 {float(start_error):.2f}°)"
            if self.repeat_tree.exists(str(index)):
                self.repeat_tree.set(
                    str(index),
                    "status",
                    f"{direction} 실패: {reason}",
                )
            self.repeat_status.set(
                f"{direction} {index + 1}단계 실패: {reason}"
            )
            self.status.set(self.repeat_status.get())
            self._append_log(
                f"[접근 반복 실패] {direction} {index + 1}단계: {reason}. "
                "새 Plan을 생성해야 합니다."
            )
            if self.repeat_batch_active:
                self.repeat_batch_current_outcome = "execution_failed"
                tomato_index = self.repeat_batch_current_index
                if tomato_index is not None:
                    failure_report = dict(self.harvest_plan_report)
                    failure_report.update(
                        {
                            "success": False,
                            "failure_stage": (
                                f"REPEAT_STAGE_{index + 1}_EXECUTION"
                            ),
                            "failure_planner_type": "execution",
                            "failure_reason": reason,
                            "execution_attempted": True,
                            "execution_success": False,
                        }
                    )
                    self._set_harvest_result(
                        tomato_index,
                        False,
                        adaptive_rotation_deg=adaptive_rotation_degrees(
                            self.harvest_plan_report
                        ),
                        approach_axis_local=adaptive_approach_axis_local(
                            self.harvest_plan_report
                        ),
                        plan_report=failure_report,
                        motion_result_text=(
                            f"접근 반복 {direction} {index + 1}단계 실패"
                        ),
                    )
                self._request_robot_motion_stop()
                process = self.step_process
                if process is not None and process.poll() is None:
                    process.terminate()
        elif event_name in {"command_error", "internal_error"}:
            message = str(event.get("message", "알 수 없는 오류"))
            self.repeat_status.set(message)
            self._append_log(f"[접근 반복 프로세스 오류] {message}")
            if self.repeat_batch_active:
                self.repeat_batch_current_outcome = "command_error"
                process = self.step_process
                if process is not None and process.poll() is None:
                    process.terminate()
        elif event_name == "closed":
            self.repeat_status.set("접근 반복 세션 종료됨")

    def _send_step_command(self, command: dict) -> bool:
        process = self.step_process
        repeat_mode = self.step_session_mode == "repeat"
        if process is None or process.poll() is not None or process.stdin is None:
            messagebox.showwarning(
                "스텝 세션 없음",
                "반복 테스트 Plan을 다시 생성하세요."
                if repeat_mode
                else "스텝 Plan을 다시 생성하세요.",
            )
            return False
        execution_enabled = (
            self.repeat_execution_enabled_var.get()
            if repeat_mode
            else self.step_execution_enabled_var.get()
        )
        if not execution_enabled:
            messagebox.showwarning(
                "실제 실행 잠금",
                "'실제 로봇 반복 실행 허용'을 먼저 체크하세요."
                if repeat_mode
                else "'실제 로봇 스텝 실행 허용'을 먼저 체크하세요.",
            )
            return False
        if not self._verification_matches_current_selection(
            self.step_session_verification
        ):
            messagebox.showerror(
                "스텝 Plan 무효",
                "검출 결과, 토마토 선택, 시작 자세 또는 진입각 설정이 "
                "변경되었습니다. 스텝 Plan을 다시 생성하세요.",
            )
            return False
        if not self.step_execution_confirmed:
            execution_message = (
                "캐시된 1~6단계 trajectory를 정방향 또는 역방향으로 "
                "실제 로봇에서 실행합니다.\n\n"
                if repeat_mode
                else "캐시된 trajectory를 실제 로봇에서 단계별로 "
                "실행합니다.\n\n"
            )
            execution_message += (
                "주변이 안전하고 교시 모드가 해제되었는지 확인했습니까?"
            )
            if not messagebox.askyesno(
                "실제 로봇 접근 반복 실행"
                if repeat_mode
                else "실제 로봇 스텝 실행",
                execution_message,
                icon="warning",
            ):
                return False
            self.step_execution_confirmed = True
        try:
            process.stdin.write(json.dumps(command) + "\n")
            process.stdin.flush()
        except (BrokenPipeError, OSError) as error:
            self._append_log(f"스텝 명령 전송 실패: {error}")
            return False
        self.step_execution_in_progress = True
        self._update_step_controls()
        return True

    def execute_next_step(self) -> None:
        self._send_step_command({"command": "execute_next"})

    def execute_previous_step(self) -> None:
        self._send_step_command({"command": "execute_previous"})

    def execute_repeat_forward(self) -> None:
        self.repeat_run_direction = "forward"
        self.repeat_pause_requested = False
        self.repeat_paused = False
        self.repeat_pause_button.configure(text="일시 정지")
        self._continue_repeat_automation()

    def execute_repeat_reverse(self) -> None:
        self.repeat_run_direction = "reverse"
        self.repeat_pause_requested = False
        self.repeat_paused = False
        self.repeat_pause_button.configure(text="일시 정지")
        self._continue_repeat_automation()

    def execute_steps_through_selection(self) -> None:
        selection = self.step_tree.selection()
        if not selection:
            messagebox.showwarning("단계 선택", "실행할 마지막 단계를 선택하세요.")
            return
        target = int(selection[0])
        if target < self.step_next_index:
            messagebox.showinfo(
                "이미 완료된 단계",
                "완료된 단계는 현재 세션에서 다시 실행할 수 없습니다.",
            )
            return
        merge_cartesian = bool(self.step_merge_cartesian_var.get())
        merge_all_trajectories = bool(
            self.step_merge_all_trajectories_var.get()
        )
        if merge_all_trajectories:
            self.step_status.set(
                "OMPL 포함 전체경로 결합·재타이밍 요청 중..."
            )
        elif merge_cartesian:
            self.step_status.set("연속 Cartesian 경로 사전계획 요청 중...")
        self._send_step_command(
            {
                "command": "execute_through",
                "stage_index": target,
                "merge_cartesian": merge_cartesian,
                "merge_all_trajectories": merge_all_trajectories,
            }
        )

    def close_step_session(self) -> None:
        process = self.step_process
        if process is None:
            return
        if self.step_execution_in_progress:
            if messagebox.askyesno(
                "실행 중인 스텝 정지",
                "현재 로봇 모션을 즉시 정지하고 스텝 세션을 종료할까요?",
                icon="warning",
            ):
                self.stop_active_motion()
            return
        if not self.step_stages:
            process.terminate()
            self.step_process = None
            self._clear_active_camera_target()
            self._set_busy(False)
            if self.step_session_mode == "repeat":
                self.repeat_status.set("접근 반복 Plan 계산 취소")
            else:
                self.step_status.set("스텝 Plan 계산 취소")
            self.step_session_mode = None
            return
        try:
            if process.stdin is not None:
                process.stdin.write(json.dumps({"command": "close"}) + "\n")
                process.stdin.flush()
        except (BrokenPipeError, OSError):
            process.terminate()
        if self.step_session_mode == "repeat":
            self.repeat_status.set("접근 반복 세션 종료 중...")
        else:
            self.step_status.set("스텝 세션 종료 중...")
        self._update_step_controls()

    def _update_step_controls(self) -> None:
        if not hasattr(self, "step_plan_button"):
            return
        process = self.step_process
        active = process is not None and process.poll() is None
        planned = active and bool(self.step_stages)
        manual_active = active and self.step_session_mode == "manual"
        repeat_active = active and self.step_session_mode == "repeat"
        executable = (
            planned
            and manual_active
            and not self.step_session_failed
            and self.step_execution_enabled_var.get()
            and not self.step_execution_in_progress
            and self.step_next_index < len(self.step_stages)
        )
        reverse_executable = (
            planned
            and manual_active
            and not self.step_session_failed
            and self.step_execution_enabled_var.get()
            and not self.step_execution_in_progress
            and self.step_next_index > 0
        )
        self.step_plan_button.configure(
            state=(
                "normal"
                if not active and not self.ui_busy and self.detected_tomatoes
                else "disabled"
            )
        )
        self.step_next_button.configure(
            state="normal" if executable else "disabled"
        )
        self.step_previous_button.configure(
            state="normal" if reverse_executable else "disabled"
        )
        self.step_execute_to_button.configure(
            state="normal" if executable else "disabled"
        )
        self.step_preapproach_goal_button.configure(
            state="normal" if planned and manual_active else "disabled"
        )
        self.step_merge_cartesian_checkbox.configure(
            state=(
                "normal"
                if manual_active and not self.step_execution_in_progress
                else "disabled"
            )
        )
        self.step_merge_all_checkbox.configure(
            state=(
                "normal"
                if manual_active and not self.step_execution_in_progress
                else "disabled"
            )
        )
        self.step_stop_button.configure(
            state="normal" if manual_active else "disabled"
        )
        self.step_close_button.configure(
            state="normal" if manual_active else "disabled"
        )
        self.step_tomato_combo.configure(
            state="disabled" if active or self.ui_busy else "readonly"
        )
        custom_state = "disabled" if active or self.ui_busy else "normal"
        for entry in self.step_custom_delta_entries:
            entry.configure(state=custom_state)
        for scale in self.step_stage_speed_scales:
            scale.configure(state=custom_state)
        self.step_preapproach_via_checkbox.configure(state=custom_state)
        if not self.step_preapproach_via_enabled_var.get():
            self.step_preapproach_final_speed_scale.configure(
                state="disabled"
            )
        self.step_tcp_wrist_oscillation_checkbox.configure(
            state="disabled" if active or self.ui_busy else "normal"
        )
        self.step_forward_wave_checkbox.configure(
            state="disabled" if active or self.ui_busy else "normal"
        )
        if not self.step_tcp_wrist_oscillation_enabled_var.get():
            self.tcp_wrist_rotation_spinbox.configure(state="disabled")
        repeat_executable = (
            planned
            and repeat_active
            and not self.step_session_failed
            and self.repeat_execution_enabled_var.get()
            and not self.step_execution_in_progress
            and not getattr(self, "repeat_batch_active", False)
            and self.repeat_run_direction is None
            and not self.repeat_pause_requested
        )
        repeat_automatic_active = bool(
            self.repeat_batch_active
            or self.repeat_run_direction in {"forward", "reverse"}
        )
        self.repeat_plan_button.configure(
            state=(
                "normal"
                if not active and not self.ui_busy and self.detected_tomatoes
                else "disabled"
            )
        )
        self.repeat_forward_button.configure(
            state=(
                "normal"
                if repeat_executable and self.step_next_index == 0
                else "disabled"
            )
        )
        self.repeat_reverse_button.configure(
            state=(
                "normal"
                if (
                    repeat_executable
                    and self.step_next_index
                    == self.repeat_cycle_last_index + 1
                )
                else "disabled"
            )
        )
        self.repeat_all_button.configure(
            state=(
                "normal"
                if (
                    not active
                    and not self.ui_busy
                    and self.detected_tomatoes
                )
                else "disabled"
            )
        )
        self.repeat_pause_button.configure(
            state="normal" if repeat_automatic_active else "disabled",
            text=(
                "계속 실행"
                if self.repeat_pause_requested
                else "일시 정지"
            ),
        )
        self.repeat_stop_button.configure(
            state=(
                "normal"
                if repeat_active or self.repeat_batch_active
                else "disabled"
            )
        )
        self.repeat_close_button.configure(
            state=(
                "normal"
                if repeat_active and not self.repeat_batch_active
                else "disabled"
            )
        )
        self.repeat_tomato_combo.configure(
            state="disabled" if active or self.ui_busy else "readonly"
        )
        self.repeat_last_stage_combo.configure(
            state="disabled" if active or self.ui_busy else "readonly"
        )
        self.repeat_forward_distance_spinbox.configure(
            state="disabled" if active or self.ui_busy else "normal"
        )

    def start_harvest(self, execute: bool) -> None:
        index = self._selected_index()
        if index is None:
            messagebox.showwarning("토마토 선택", "수확할 토마토를 먼저 선택하세요.")
            return
        if (
            self.harvest_process is not None
            or self.step_process is not None
            or self.batch_active
            or self.sweep_active
        ):
            messagebox.showinfo("실행 중", "현재 모션 작업이 끝날 때까지 기다려 주세요.")
            return
        try:
            harvest_wait_sec = self._wait_seconds(
                self.linear_motor_wait_sec.get()
            )
            harvest_x_forward_m = repeat_forward_distance_m(
                self.harvest_forward_distance_mm_var.get()
            )
            tcp_wrist_rotation_deg = float(
                self.tcp_wrist_rotation_deg_var.get()
            )
            if not 1.0 <= tcp_wrist_rotation_deg <= 45.0:
                raise ValueError("5단계 TCP 회전각은 1~45도 범위여야 합니다.")
            pick_ready_state = self._selected_pick_ready_state()
            prefer_robot_direction, adaptive_max_rotation = (
                self._adaptive_grasp_options()
            )
        except ValueError as error:
            messagebox.showerror("수확 모션 설정 오류", str(error))
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
            pick_ready_state,
            prefer_robot_direction,
            adaptive_max_rotation,
            harvest_x_forward_m,
            tcp_wrist_rotation_deg,
        )
        if execute and not self._verification_matches_current_selection(
            self.verified_plan
        ):
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

        lift_mode = bool(self.lift_harvest_var.get())
        self._set_busy(True)
        self._prepare_lift_for_tomato(
            index,
            lift_mode,
            lambda: self._launch_harvest_process(
                index,
                execute,
                verification,
                harvest_wait_sec=harvest_wait_sec,
                harvest_x_forward_m=harvest_x_forward_m,
                harvest_tcp_wrist_rotation_deg=tcp_wrist_rotation_deg,
                prefer_robot_direction=prefer_robot_direction,
                adaptive_grasp_max_rotation_deg=adaptive_max_rotation,
            ),
            self._handle_lift_preparation_error,
        )

    def move_to_named_pose(self) -> None:
        """Plan and execute a selected SRDF group state."""
        if (
            self.harvest_process is not None
            or self.step_process is not None
            or self.batch_active
            or self.sweep_active
        ):
            messagebox.showinfo("실행 중", "현재 모션 작업이 끝날 때까지 기다려 주세요.")
            return
        state_name = str(self.named_pose_var.get()).strip()
        if state_name not in NAMED_POSE_STATES:
            messagebox.showerror(
                "저장 자세 오류",
                f"지원하지 않는 저장 자세입니다: {state_name}",
            )
            return
        if not messagebox.askyesno(
            "저장 자세 Plan & Execute",
            f"현재 자세에서 {state_name}(으)로 계획하고 실제 이동할까요?\n\n"
            "로봇 주변이 안전하고 교시 모드가 해제되었는지 확인하세요.",
            icon="warning",
        ):
            return

        command = named_pose_command(
            state_name,
            velocity_scale=self.motion_velocity_scale,
            acceleration_scale=self.motion_acceleration_scale,
        )
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        self.named_pose_report = {}
        self.status.set(f"저장 자세 {state_name} Plan & Execute 중...")
        self._append_log(
            f"[저장 자세 이동] {state_name} 계획 및 실행 시작 — "
            "constrained OMPL/RRTConnect"
        )
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
            self.named_pose_active_state = state_name
        except OSError as error:
            self.harvest_process = None
            self._set_busy(False)
            self.status.set(f"저장 자세 {state_name} 실행 시작 실패")
            self._append_log(f"[저장 자세 이동] 프로세스 시작 오류: {error}")
            return
        thread = threading.Thread(
            target=self._read_named_pose_process,
            args=(self.harvest_process, state_name),
            daemon=True,
        )
        thread.start()

    def _read_named_pose_process(self, process, state_name: str) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                line = line.rstrip()
                if line.startswith(NAMED_POSE_RESULT_PREFIX):
                    try:
                        report = json.loads(
                            line[len(NAMED_POSE_RESULT_PREFIX):]
                        )
                    except json.JSONDecodeError as error:
                        self.process_queue.put(
                            ("log", f"저장 자세 결과 해석 실패: {error}")
                        )
                        continue
                    self.process_queue.put(
                        ("named_pose_report", report, process)
                    )
                elif is_critical_process_output(line):
                    self.process_queue.put(("log", line))
        return_code = process.wait()
        self.process_queue.put(
            ("named_pose_done", return_code, state_name, process)
        )

    def start_harvest_all_plan(self) -> None:
        """Plan the complete detected-tomato sequence without execution."""
        self.start_harvest_all(execute_motion=False)

    def start_harvest_all(self, execute_motion: bool = True) -> None:
        if (
            self.harvest_process is not None
            or self.step_process is not None
            or self.batch_active
            or self.sweep_active
        ):
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
            harvest_x_forward_m = repeat_forward_distance_m(
                self.harvest_forward_distance_mm_var.get()
            )
            tcp_wrist_rotation_deg = float(
                self.tcp_wrist_rotation_deg_var.get()
            )
            if not 1.0 <= tcp_wrist_rotation_deg <= 45.0:
                raise ValueError("5단계 TCP 회전각은 1~45도 범위여야 합니다.")
            pick_ready_state = self._selected_pick_ready_state()
            prefer_robot_direction, adaptive_max_rotation = (
                self._adaptive_grasp_options()
            )
            harvest_stage_limit = batch_harvest_stage_limit(
                self.batch_harvest_stage_var.get()
            )
        except ValueError as error:
            messagebox.showerror("전체 연속 수확 설정 오류", str(error))
            return
        continuous_mode = bool(self.continuous_harvest_var.get())
        lift_mode = bool(self.lift_harvest_var.get())
        preplan_mode = (
            bool(self.preplan_all_var.get()) if execute_motion else True
        )
        if continuous_mode and not lift_mode and not preplan_mode:
            preplan_mode = True
            self.preplan_all_var.set(True)
            self._append_log(
                "식물바깥 Arc 안전 복구를 위해 전체 모션 사전계획을 "
                "자동 활성화했습니다. Arc 실패 시 실행 이력을 역재생해 "
                f"{pick_ready_state}로 복귀합니다."
            )
        if harvest_stage_limit is not None and lift_mode:
            messagebox.showwarning(
                "단계 제한과 리프트 수확 동시 사용 불가",
                "3·4단계 종료 테스트는 리프트를 움직이기 전의 접근 동작만 "
                "실행하므로 리프트 수확과 함께 사용할 수 없습니다.",
            )
            return
        if preplan_mode and lift_mode:
            messagebox.showwarning(
                "사전계획 사용 불가",
                "리프트 수확에서는 전체 모션 사전계획을 사용할 수 "
                "없습니다.",
            )
            return
        if continuous_mode and lift_mode:
            transition_message = (
                "토마토 사이에는 식물 바깥 안전 위치로 후퇴한 뒤 리프트를 "
                "조정하고, 변경된 높이에서 다음 pre-grasp arc를 새로 "
                "계획합니다.\n"
            )
        elif continuous_mode:
            if harvest_stage_limit is None:
                transition_start = "현재 post-wait 자세"
            else:
                transition_start = f"현재 {harvest_stage_limit}단계 자세"
            transition_message = (
                f"토마토 사이에는 {transition_start}에서 다음 pre-grasp로 "
                "식물 바깥쪽 arc 경로를 따라 이동하고, 마지막에는 "
                f"{pick_ready_state} 자세로 복귀합니다.\n"
            )
        else:
            transition_message = (
                f"각 토마토 수확 후 {pick_ready_state} 자세로 "
                "복귀합니다.\n"
            )
        if lift_mode:
            transition_message += (
                "각 토마토의 world 높이보다 400mm 낮게 리프트를 자동 "
                "배치합니다.\n"
            )
        transition_message += (
            f"선택한 시작/최종 복귀 자세는 {pick_ready_state}입니다.\n"
        )
        transition_message += (
            "수확 4단계 진입 길이는 "
            f"{harvest_x_forward_m * 1000.0:g}mm입니다.\n"
        )
        transition_message += (
            "진입각은 "
            + (
                "로봇 방향 우선"
                if prefer_robot_direction
                else "Recommend 우선"
            )
            + f", 최대 보정각은 {adaptive_max_rotation:g}°입니다.\n"
        )
        if harvest_stage_limit is not None:
            transition_message += (
                f"각 토마토는 수확 {harvest_stage_limit}단계까지만 실행하고 "
                "대기·후퇴 단계는 실행하지 않습니다.\n"
            )
        if preplan_mode:
            if execute_motion:
                transition_message += (
                    "각 토마토 trajectory를 PICK_READY 기준으로 먼저 독립 "
                    "계획하고, 성공한 토마토의 저장 trajectory만 실행합니다.\n"
                )
            else:
                transition_message += (
                    "각 토마토 trajectory를 PICK_READY 기준으로 독립 계획하고 "
                    "실제 로봇에는 실행하지 않습니다.\n"
                )
            if not execute_motion:
                failure_policy = (
                    "계획 실패 토마토는 건너뛰며 모든 성공·실패 결과만 "
                    "화면에 표시합니다.\n"
                )
            elif continuous_mode:
                failure_policy = (
                    "토마토 자체 계획 실패는 건너뜁니다. 토마토 사이 Arc가 "
                    "실패하면 검증된 경로를 역재생해 PICK_READY로 복귀한 뒤 "
                    "다음 토마토의 독립 trajectory를 사용합니다.\n"
                )
            else:
                failure_policy = (
                    "사전계획 실패 토마토는 건너뛰며, 실제 trajectory 실행 "
                    "실패 시 전체 작업을 중단합니다.\n"
                )
        else:
            failure_policy = (
                "Plan-only 실패 토마토는 건너뛰며, 실제 실행 실패 시 "
                "중단됩니다.\n"
            )
        operation_title = (
            "검출 토마토 전체 연속 수확"
            if execute_motion
            else "검출 토마토 전체 연속 Plan"
        )
        operation_prompt = (
            "실행할까요?"
            if execute_motion
            else "실제 로봇을 움직이지 않고 전체 경로를 계산할까요?"
        )
        safety_notice = (
            "로봇 주변이 안전하고 교시 모드가 해제되었는지 확인하세요."
            if execute_motion
            else "계획 결과와 실패 단계는 토마토 목록 및 실행 로그에 표시됩니다."
        )
        if not messagebox.askyesno(
            operation_title,
            f"검출된 토마토 {tomato_count}개 전체를 순서대로 "
            f"{operation_prompt}\n\n"
            f"{transition_message}"
            f"{failure_policy}"
            f"{safety_notice}",
            icon="warning" if execute_motion else "question",
        ):
            return

        self.batch_active = True
        self.batch_jobs = (
            deque()
            if preplan_mode
            else deque(harvest_all_jobs(tomato_count))
        )
        self.batch_generation = self.detection_generation
        self.batch_total = tomato_count
        self.batch_completed = 0
        self.batch_skipped = 0
        self.batch_planner = self._selected_planner_config()
        self.batch_pick_ready_state = pick_ready_state
        self.batch_harvest_wait_sec = harvest_wait_sec
        self.batch_harvest_x_forward_m = harvest_x_forward_m
        self.batch_tcp_wrist_rotation_deg = tcp_wrist_rotation_deg
        self.batch_harvest_stage_limit = harvest_stage_limit
        self.batch_continuous_mode = continuous_mode
        self.batch_lift_harvest_mode = lift_mode
        self.batch_preplan_mode = preplan_mode
        self.batch_execute_motion = bool(execute_motion)
        self.batch_prefer_robot_direction = prefer_robot_direction
        self.batch_adaptive_grasp_max_rotation_deg = adaptive_max_rotation
        self.batch_preplan_failure_message = ""
        try:
            self.batch_scene = (
                float(self.scene_x.get()),
                float(self.scene_y.get()),
                float(self.scene_z.get()),
                float(self.scene_rotation.get()),
            )
        except ValueError:
            self.batch_scene = ("", "", "", "")
        self.sweep_session_dir = None
        operation_name = (
            "전체 연속 수확" if execute_motion else "전체 연속 Plan"
        )
        self._reset_live_statistics(
            f"결과 파일: {operation_name}은 실시간 화면 표시 전용"
        )
        self.sweep_summary.set(
            f"{operation_name} — 토마토 0 / {self.batch_total}"
        )
        self._invalidate_plan()
        self._clear_harvest_results()
        self._reset_tomato_motion_results("대기")
        self._append_log(
            f"{operation_name} 시작: 검출 토마토 {tomato_count}개 전체, "
            f"토마토별 종료 단계="
            f"{harvest_stage_limit or '전체 수확'}, "
            f"planner={self.batch_planner[0]}/{self.batch_planner[1]}, "
            f"preapproach={self.batch_planner[2]}, "
            f"리니어모터 대기={self.batch_harvest_wait_sec:.2f}s, "
            "4단계 진입="
            f"{self.batch_harvest_x_forward_m * 1000.0:g}mm, "
            f"5단계 TCP 회전=±{self.batch_tcp_wrist_rotation_deg:g}°, "
            f"연속 arc 전환 모드={self.batch_continuous_mode}, "
            f"리프트 수확 모드={self.batch_lift_harvest_mode}, "
            f"전체 사전계획 모드={self.batch_preplan_mode}, "
            f"실제 실행={self.batch_execute_motion}, "
            f"진입각 로봇방향 우선={self.batch_prefer_robot_direction}, "
            "최대 보정각="
            f"{self.batch_adaptive_grasp_max_rotation_deg:g}°, "
            f"시작/복귀 자세={self.batch_pick_ready_state}"
        )
        self._set_busy(True)
        if self.batch_preplan_mode:
            self._launch_preplanned_batch_process()
        else:
            self._start_next_batch_job()

    def _launch_preplanned_batch_process(self) -> None:
        pipeline, planner_id, preapproach_mode = self.batch_planner
        command, extra_environment = preplanned_batch_command(
            self.batch_total,
            continuous_arc=self.batch_continuous_mode,
            tomato_frames=[
                self._tomato_frame(index)
                for index in range(self.batch_total)
            ],
            execute=self.batch_execute_motion,
            planning_pipeline_id=pipeline,
            planner_id=planner_id,
            preapproach_mode=preapproach_mode,
            velocity_scale=self.motion_velocity_scale,
            acceleration_scale=self.motion_acceleration_scale,
            harvest_wait_sec=self.batch_harvest_wait_sec,
            harvest_x_forward_m=self.batch_harvest_x_forward_m,
            harvest_tcp_wrist_rotation_deg=(
                self.batch_tcp_wrist_rotation_deg
            ),
            harvest_stage_limit=self.batch_harvest_stage_limit,
            pick_ready_state_name=self.batch_pick_ready_state,
            prefer_robot_direction=self.batch_prefer_robot_direction,
            adaptive_grasp_max_rotation_deg=(
                self.batch_adaptive_grasp_max_rotation_deg
            ),
        )
        environment = os.environ.copy()
        environment.update(extra_environment)
        environment["PYTHONUNBUFFERED"] = "1"
        self.status.set("전체 trajectory Plan 시작...")
        self._append_log(
            f"[전체 연속 Plan] 토마토 {self.batch_total}개 trajectory "
            f"계산 시작 — Arc={self.batch_continuous_mode}, "
            f"실제 실행={self.batch_execute_motion}"
        )
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
            self._finish_batch(
                False,
                f"전체 사전계획 프로세스 시작 실패: {error}",
            )
            return
        thread = threading.Thread(
            target=self._read_preplanned_batch_process,
            args=(self.harvest_process,),
            daemon=True,
        )
        thread.start()

    def _read_preplanned_batch_process(self, process) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                line = line.rstrip()
                if line.startswith(PREPLANNED_BATCH_EVENT_PREFIX):
                    try:
                        event = json.loads(
                            line[len(PREPLANNED_BATCH_EVENT_PREFIX):]
                        )
                    except json.JSONDecodeError as error:
                        self.process_queue.put(
                            ("log", f"전체 사전계획 결과 해석 실패: {error}")
                        )
                        continue
                    self.process_queue.put(
                        ("preplanned_batch_event", event, process)
                    )
                elif is_critical_process_output(line):
                    self.process_queue.put(("log", line))
        return_code = process.wait()
        self.process_queue.put(
            ("preplanned_batch_done", return_code, process)
        )

    def _handle_preplanned_batch_event(self, event: dict) -> None:
        phase = str(event.get("phase", ""))
        index = int(event.get("index", -1))
        report = event.get("report") or {}
        if phase == "preplan_started":
            self.status.set(
                f"전체 사전계획 중: 0/{self.batch_total}"
            )
            return
        if phase == "planning":
            self._set_active_camera_target(index, "전체 사전계획")
            number = int(event.get("number", index + 1))
            self.status.set(
                f"전체 사전계획 중: {number}/{self.batch_total} "
                f"(토마토 {index})"
            )
            self._set_tomato_motion_result(index, "사전계획 중")
            return
        if phase == "candidate_retry":
            next_attempt = int(event.get("next_attempt", 0))
            attempt_limit = int(event.get("attempt_limit", 3))
            failure_stage = str(
                event.get("failure_stage", "UNKNOWN")
            )
            failure_reason = str(
                event.get("failure_reason", "UNKNOWN")
            )
            self.status.set(
                f"토마토 {index} 독립 계획 재시도 "
                f"{next_attempt}/{attempt_limit}"
            )
            self._set_tomato_motion_result(
                index,
                f"독립 계획 재시도 {next_attempt}/{attempt_limit}",
            )
            self._append_log(
                f"[전체 사전계획] 토마토 {index} 독립 계획 재시도 "
                f"{next_attempt}/{attempt_limit} — "
                f"{failure_stage} / {failure_reason}"
            )
            return
        if phase == "planned":
            success = bool(event.get("success"))
            self.harvest_plan_report = report
            if not success:
                self.batch_skipped += 1
            self._set_harvest_result(
                index,
                success,
                adaptive_rotation_applied=(
                    success and adaptive_rotation_was_applied(report)
                ),
                adaptive_rotation_deg=adaptive_rotation_degrees(report),
                approach_axis_local=adaptive_approach_axis_local(report),
                plan_report=report,
                motion_result_text=(
                    "사전계획 완료"
                    if success
                    else "사전계획 실패 · 건너뜀"
                ),
            )
            self._append_log(
                f"[전체 사전계획 {index + 1}/{self.batch_total}] "
                f"토마토 {index} {'성공' if success else '실패'}"
                + (
                    ""
                    if success
                    else f" — {harvest_failure_summary(report)}"
                )
                + (" · 건너뜀" if not success else "")
            )
            self._clear_active_camera_target(index)
            return
        if phase == "skipped":
            return
        if phase == "preplan_complete":
            planned_count = int(event.get("planned_count", 0))
            skipped_count = int(
                event.get("skipped_count", self.batch_skipped)
            )
            self._clear_active_camera_target()
            if self.batch_execute_motion:
                self.status.set(
                    "전체 사전계획 완료 — "
                    f"성공 {planned_count}, 건너뜀 {skipped_count}; "
                    "저장 trajectory 실행 시작"
                )
                self._append_log(
                    "[전체 사전계획 완료] "
                    f"성공 {planned_count}개, 계획 실패 건너뜀 "
                    f"{skipped_count}개. 성공한 저장 trajectory만 "
                    "재계획 없이 실행합니다."
                )
            else:
                self.status.set(
                    "전체 연속 Plan 완료 — "
                    f"성공 {planned_count}, 실패 {skipped_count}"
                )
                self._append_log(
                    "[전체 연속 Plan 완료] "
                    f"성공 {planned_count}개, 실패 {skipped_count}개. "
                    "실제 로봇 모션은 실행하지 않았습니다."
                )
            return
        if phase == "executing":
            self._set_active_camera_target(index, "전체 사전계획 실행")
            number = int(event.get("number", index + 1))
            execution_total = int(event.get("total", self.batch_total))
            self.status.set(
                f"저장 trajectory 실행 중: {number}/{execution_total} "
                f"(토마토 {index})"
            )
            self._set_tomato_motion_result(index, "수확 실행 중")
            return
        if phase == "executed":
            success = bool(event.get("success"))
            number = int(event.get("number", index + 1))
            execution_total = int(event.get("total", self.batch_total))
            self.harvest_plan_report = report
            self._set_harvest_result(
                index,
                success,
                adaptive_rotation_applied=(
                    success and adaptive_rotation_was_applied(report)
                ),
                adaptive_rotation_deg=adaptive_rotation_degrees(report),
                approach_axis_local=adaptive_approach_axis_local(report),
                plan_report=report,
                motion_result_text=tomato_motion_result_text(
                    True,
                    success,
                    report,
                ),
            )
            verification = (
                self.batch_generation,
                index,
                *self.batch_planner,
                self.batch_pick_ready_state,
                self.batch_prefer_robot_direction,
                self.batch_adaptive_grasp_max_rotation_deg,
            )
            self._record_batch_statistics(
                0 if success else 1,
                True,
                verification,
            )
            if success:
                self.batch_completed += 1
            self._append_log(
                f"[저장 trajectory 실행 {number}/{execution_total}] "
                f"토마토 {index} {'성공' if success else '실패'}"
            )
            self._clear_active_camera_target(index)
            return
        if phase == "aborted":
            self._clear_active_camera_target(index)
            reason = str(event.get("reason", "UNKNOWN"))
            detail = harvest_failure_summary(report)
            self.batch_preplan_failure_message = (
                f"전체 사전계획 수확 중단: 토마토 {index}, "
                f"{reason}, {detail}"
            )
            self._append_log(
                f"[전체 사전계획 중단] {self.batch_preplan_failure_message}"
            )
            return
        if phase == "complete":
            executed_count = int(
                event.get("executed_count", self.batch_completed)
            )
            skipped_count = int(
                event.get("skipped_count", self.batch_skipped)
            )
            self._clear_active_camera_target()
            if self.batch_execute_motion:
                self._append_log(
                    "[전체 사전계획 실행 완료] 성공 trajectory "
                    f"{executed_count}개 실행, 계획 실패 "
                    f"{skipped_count}개 건너뜀."
                )

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
            self.batch_pick_ready_state,
            self.batch_prefer_robot_direction,
            self.batch_adaptive_grasp_max_rotation_deg,
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
        continuous_transition = (
            self.batch_continuous_mode and self.batch_completed > 0
        )
        return_to_pick_ready = (
            not self.batch_continuous_mode or index == self.batch_total - 1
        )
        retreat_after_harvest = bool(
            self.batch_lift_harvest_mode
            and self.batch_continuous_mode
            and index < self.batch_total - 1
        )

        def launch_job() -> None:
            if not self.batch_active:
                return
            if not self._launch_harvest_process(
                index,
                execute,
                verification,
                harvest_wait_sec=self.batch_harvest_wait_sec,
                harvest_x_forward_m=self.batch_harvest_x_forward_m,
                harvest_tcp_wrist_rotation_deg=(
                    self.batch_tcp_wrist_rotation_deg
                ),
                continuous_transition=continuous_transition,
                return_to_pick_ready=return_to_pick_ready,
                retreat_after_harvest=retreat_after_harvest,
                harvest_stage_limit=self.batch_harvest_stage_limit,
                prefer_robot_direction=(
                    self.batch_prefer_robot_direction
                ),
                adaptive_grasp_max_rotation_deg=(
                    self.batch_adaptive_grasp_max_rotation_deg
                ),
            ):
                self._finish_batch(
                    False,
                    f"토마토 {index} 작업 프로세스를 시작하지 못해 "
                    "전체 수확을 중단했습니다.",
                )

        self._prepare_lift_for_tomato(
            index,
            self.batch_lift_harvest_mode,
            launch_job,
            self._handle_lift_preparation_error,
        )

    def _launch_harvest_process(
        self,
        index: int,
        execute: bool,
        verification,
        harvest_wait_sec: float,
        harvest_x_forward_m: float = 0.040,
        harvest_tcp_wrist_rotation_deg: float | None = None,
        continuous_transition: bool = False,
        return_to_pick_ready: bool = True,
        retreat_after_harvest: bool = False,
        harvest_stage_limit: int | None = None,
        prefer_robot_direction: bool | None = None,
        adaptive_grasp_max_rotation_deg: float | None = None,
    ) -> bool:
        pipeline, planner_id, preapproach_mode = verification[2:5]
        pick_ready_state = (
            verification[5] if len(verification) > 5 else "PICK_READY"
        )
        if prefer_robot_direction is None:
            prefer_robot_direction = bool(
                self.prefer_robot_direction_var.get()
            )
        if adaptive_grasp_max_rotation_deg is None:
            adaptive_grasp_max_rotation_deg = (
                adaptive_grasp_max_rotation_degrees(
                    self.adaptive_grasp_max_rotation_var.get()
                )
            )
        if harvest_tcp_wrist_rotation_deg is None:
            harvest_tcp_wrist_rotation_deg = float(
                self.tcp_wrist_rotation_deg_var.get()
            )
        command = harvest_command(
            index,
            execute,
            tomato_frame=self._tomato_frame(index),
            planning_pipeline_id=pipeline,
            planner_id=planner_id,
            preapproach_mode=preapproach_mode,
            publish_display_trajectory=not self.sweep_active,
            velocity_scale=self.motion_velocity_scale,
            acceleration_scale=self.motion_acceleration_scale,
            harvest_wait_sec=harvest_wait_sec,
            harvest_x_forward_m=harvest_x_forward_m,
            harvest_tcp_wrist_rotation_deg=(
                harvest_tcp_wrist_rotation_deg
            ),
            continuous_transition=continuous_transition,
            return_to_pick_ready=return_to_pick_ready,
            retreat_after_harvest=retreat_after_harvest,
            harvest_stage_limit=harvest_stage_limit,
            pick_ready_state_name=pick_ready_state,
            prefer_robot_direction=prefer_robot_direction,
            adaptive_grasp_max_rotation_deg=(
                adaptive_grasp_max_rotation_deg
            ),
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
        if harvest_stage_limit is not None and return_to_pick_ready:
            end_label = f"{harvest_stage_limit}단계→{pick_ready_state}"
        elif harvest_stage_limit is not None:
            end_label = f"{harvest_stage_limit}단계 유지"
        elif return_to_pick_ready:
            end_label = pick_ready_state
        elif retreat_after_harvest:
            end_label = "리프트 안전 후퇴"
        else:
            end_label = "post-wait 유지"
        self._append_log(
            f"{batch_prefix}{mode} 시작: {self._tomato_frame(index)} "
            f"planner={pipeline}/{planner_id}, "
            f"preapproach={preapproach_mode}, "
            "angle_mode="
            f"{'robot_priority' if prefer_robot_direction else 'recommend_priority'}, "
            f"max_rotation={adaptive_grasp_max_rotation_deg:g}°, "
            f"시작/복귀 자세={pick_ready_state}, "
            f"리니어모터 대기={harvest_wait_sec:.2f}s, "
            f"4단계 진입={harvest_x_forward_m * 1000.0:g}mm, "
            f"5단계 TCP 회전=±{harvest_tcp_wrist_rotation_deg:g}°, "
            f"종료단계={harvest_stage_limit or '전체'}, "
            f"시작={'현재→바깥 arc→pre-grasp' if continuous_transition else pick_ready_state}, "
            f"종료={end_label}"
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
        if self.sweep_active:
            context = "자동 테스트"
        elif self.batch_active:
            context = "전체 연속 수확"
        else:
            context = "실제 수확" if execute else "Plan-only"
        self._set_active_camera_target(index, context)
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
            if item[0] == "linear_motor_log":
                _, line, process = item
                if process is self.linear_motor_launch_process:
                    self._append_log(f"[Arduino 리니어모터] {line}")
                continue
            if item[0] == "linear_motor_done":
                _, return_code, process = item
                if process is not self.linear_motor_launch_process:
                    continue
                self.linear_motor_launch_process = None
                self.linear_motor_node_online = False
                self.linear_motor_serial_connected = False
                self.linear_motor_pin_state_initialized = False
                self.linear_motor_node_status.set("실행 안 됨")
                self.linear_motor_serial_status.set(
                    f"launch 종료 코드 {return_code}"
                )
                self._append_log(
                    f"[Arduino 리니어모터] launch 종료 "
                    f"(종료 코드 {return_code})"
                )
                self._update_linear_motor_launch_control()
                self._update_gripper_stroke_controls()
                continue
            if item[0] == "lift_log":
                _, line, process = item
                if process is self.lift_launch_process:
                    self._append_log(f"[리프트] {line}")
                continue
            if item[0] == "lift_done":
                _, return_code, process = item
                if process is not self.lift_launch_process:
                    continue
                self.lift_launch_process = None
                if return_code != 0:
                    self._append_log(
                        f"[리프트] launch 종료 (종료 코드 {return_code})"
                    )
                if not self.lift_node_online:
                    self.lift_node_status.set("실행 안 됨")
                self._update_lift_controls()
                continue
            if item[0] == "log":
                self._append_log(item[1])
                continue
            if item[0] == "named_pose_report":
                _, report, process = item
                if process is self.harvest_process:
                    self.named_pose_report = report
                continue
            if item[0] == "named_pose_done":
                _, return_code, state_name, process = item
                if process is not self.harvest_process:
                    continue
                self.harvest_process = None
                self.named_pose_active_state = None
                self._set_busy(False)
                duration = float(
                    self.named_pose_report.get("duration_sec", 0.0)
                )
                if return_code == 0:
                    self.last_completed_named_pose = state_name
                    if state_name in ("CAPTURE_LEFT", "CAPTURE_RIGHT"):
                        ready_state = pick_ready_state_for_capture_pose(
                            state_name
                        )
                        self.pick_ready_state_var.set(ready_state)
                    message = (
                        f"저장 자세 {state_name} 이동 완료 "
                        f"({duration:.2f}s)"
                    )
                else:
                    failure_stage = self.named_pose_report.get(
                        "failure_stage", "UNKNOWN"
                    )
                    failure_reason = self.named_pose_report.get(
                        "failure_reason", "UNKNOWN"
                    )
                    message = (
                        f"저장 자세 {state_name} 이동 실패 — "
                        f"{failure_stage}: {failure_reason}"
                    )
                self.status.set(message)
                self._append_log(f"[저장 자세 이동] {message}")
                continue
            if item[0] == "preplanned_batch_event":
                _, event, process = item
                if process is self.harvest_process and self.batch_active:
                    self._handle_preplanned_batch_event(event)
                continue
            if item[0] == "preplanned_batch_done":
                _, return_code, process = item
                if process is not self.harvest_process:
                    continue
                self.harvest_process = None
                self._clear_active_camera_target()
                if not self.batch_active:
                    continue
                if return_code == 0:
                    if self.batch_execute_motion:
                        message = (
                            "전체 사전계획 수확 완료: 저장된 trajectory로 "
                            f"토마토 {self.batch_completed}개 수확, "
                            f"계획 실패 {self.batch_skipped}개 건너뜀"
                        )
                    else:
                        planned_count = self.batch_total - self.batch_skipped
                        message = (
                            "전체 연속 Plan 완료: "
                            f"성공 {planned_count}개, 실패 "
                            f"{self.batch_skipped}개, 실제 실행 없음"
                        )
                    self._finish_batch(True, message)
                else:
                    self._finish_batch(
                        False,
                        self.batch_preplan_failure_message
                        or (
                            "전체 사전계획 프로세스가 비정상 종료했습니다. "
                            f"(종료 코드 {return_code})"
                        ),
                    )
                continue
            if item[0] == "stepper_event":
                _, event, process = item
                self._handle_stepper_event(event, process)
                continue
            if item[0] == "stepper_done":
                _, return_code, process = item
                if process is not self.step_process:
                    continue
                session_mode = self.step_session_mode
                completed_target_index = self.active_camera_target_index
                self.step_process = None
                self.step_execution_in_progress = False
                if self.repeat_batch_active and session_mode == "repeat":
                    outcome = self.repeat_batch_current_outcome
                    tomato_index = self.repeat_batch_current_index
                    self.step_session_mode = None
                    self.step_session_verification = None
                    self._clear_active_camera_target(completed_target_index)
                    self.step_stages = []
                    self.step_next_index = 0
                    if outcome in {"success", "plan_failed"}:
                        self.repeat_batch_completed += 1
                        if outcome == "plan_failed":
                            self.repeat_batch_plan_failures += 1
                        self._append_log(
                            f"[전체 접근 반복 {self.repeat_batch_completed}/"
                            f"{self.repeat_batch_total}] 토마토 {tomato_index}: "
                            f"{'완료' if outcome == 'success' else 'Plan 실패'}"
                        )
                        self.root.after(
                            100,
                            self._start_next_repeat_batch_tomato,
                        )
                    else:
                        self._finish_repeat_batch(
                            False,
                            f"토마토 {tomato_index} 실행 실패로 전체 접근 "
                            f"반복을 중단했습니다. (종료 코드 {return_code})",
                        )
                    continue
                self._set_busy(False)
                if session_mode == "repeat":
                    self.repeat_run_direction = None
                    self.repeat_pause_requested = False
                    self.repeat_paused = False
                    self.repeat_pause_button.configure(text="일시 정지")
                if return_code != 0:
                    self._append_log(
                        f"[{'접근 반복' if session_mode == 'repeat' else '스텝'} "
                        f"세션 종료] 프로세스 종료 코드 {return_code}"
                    )
                    if (
                        session_mode == "repeat"
                        and self.repeat_status.get().endswith("계산 중...")
                    ):
                        self.repeat_status.set("접근 반복 Plan 실패")
                    elif self.step_status.get().endswith("계산 중..."):
                        self.step_status.set("스텝 Plan 실패")
                self.step_session_mode = None
                self._clear_active_camera_target(completed_target_index)
                self._update_step_controls()
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
                self._set_active_camera_target_plan_report(report)
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
            self._clear_active_camera_target(verification[1])
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
                    plan_report=self.harvest_plan_report,
                    motion_result_text=tomato_motion_result_text(
                        execute,
                        True,
                        self.harvest_plan_report,
                    ),
                )
                self.status.set(f"{mode} 완료")
                self._append_log(f"{mode} 완료 (종료 코드 0)")
                if self._verification_matches_current_selection(verification):
                    self.verified_plan = verification
                    self.execute_button.configure(state="normal")
                    if execute:
                        self._append_log(
                            "실제 수확 성공 — 현재 Plan-only 검증을 유지하여 "
                            "동일 토마토를 다시 실행할 수 있습니다."
                        )
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
                    plan_report=self.harvest_plan_report,
                    motion_result_text=tomato_motion_result_text(
                        execute,
                        False,
                        self.harvest_plan_report,
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
        self._clear_active_camera_target(verification[1])
        if self.sweep_cancel_requested:
            self._finish_sweep("사용자가 자동 테스트를 중지했습니다.")
            return
        tomato_index = verification[1]
        report = report or {}
        success = return_code == 0
        record = harvest_statistics_record(
            case=self.sweep_case_number,
            tomato_index=tomato_index,
            scene=self.sweep_current_case,
            success=success,
            verification=verification,
            execute_motion=self.sweep_execute_motion,
            report=report,
        )
        self._save_sweep_result(record)
        self._update_sweep_statistics(record)
        self._update_result_arrow_length(
            tomato_index,
            success=success,
            plan_report=report,
        )
        transform = None
        parent_frame = str(
            self.get_parameter("result_marker_parent_frame").value
        )
        if not report.get("suppress_result_marker"):
            tomato_frame = self._tomato_frame(tomato_index)
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent_frame,
                    tomato_frame,
                    Time(),
                )
            except TransformException as error:
                self._append_log(f"결과 마커용 TF 조회 실패: {error}")

        if transform is not None:
            marker_id = self.sweep_marker_next_id
            marker = sweep_result_marker(
                marker_id,
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
            approach_marker = actual_approach_marker(
                marker_id,
                report,
                namespace=HARVEST_SWEEP_APPROACH_NAMESPACE,
            )
            if approach_marker is not None:
                approach_marker.header.frame_id = parent_frame
                approach_marker.pose = marker.pose
                self.sweep_markers.append(approach_marker)
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
        self._set_tomato_motion_result(
            tomato_index,
            tomato_motion_result_text(
                self.sweep_execute_motion,
                success,
                report,
            ),
        )
        if record["execution_attempted"] and not record["execution_success"]:
            self._finish_sweep(
                f"토마토 {tomato_index} 실제 실행 실패로 자동 테스트를 "
                "중단했습니다."
            )
            return
        self.root.after(100, self._start_sweep_plan)

    def _record_batch_statistics(
        self,
        return_code: int,
        execute: bool,
        verification,
    ) -> None:
        """Append one final tomato outcome to the shared live table."""
        record = harvest_statistics_record(
            case="전체",
            tomato_index=verification[1],
            scene=self.batch_scene,
            success=return_code == 0,
            verification=verification,
            execute_motion=execute,
            report=self.harvest_plan_report or {},
        )
        self._update_sweep_statistics(record)
        self.sweep_completed += 1
        self.sweep_summary.set(
            f"전체 연속 수확 — 토마토 {self.sweep_completed} / "
            f"{self.batch_total} 결과 집계"
        )

    def _handle_batch_job_done(self, return_code, execute: bool, verification) -> None:
        index = verification[1]
        mode = "실제 수확" if execute else "Plan-only"
        if return_code != 0:
            self._record_batch_statistics(return_code, execute, verification)
            self._set_harvest_result(
                index,
                False,
                adaptive_rotation_deg=adaptive_rotation_degrees(
                    self.harvest_plan_report
                ),
                approach_axis_local=adaptive_approach_axis_local(
                    self.harvest_plan_report
                ),
                plan_report=self.harvest_plan_report,
                motion_result_text=tomato_motion_result_text(
                    execute,
                    False,
                    self.harvest_plan_report,
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
            failure_detail = harvest_failure_summary(
                self.harvest_plan_report
            )
            if not bool(
                (self.harvest_plan_report or {}).get(
                    "execution_attempted",
                    False,
                )
            ):
                failure_message = (
                    f"토마토 {index} 실제 모션 시작 전 재계획 실패로 "
                    "전체 수확을 중단했습니다. 실제 trajectory는 "
                    f"실행되지 않았습니다. ({failure_detail}, "
                    f"종료 코드 {return_code})"
                )
            else:
                failure_message = (
                    f"토마토 {index} 실제 trajectory 실행 실패로 전체 "
                    f"수확을 중단했습니다. ({failure_detail}, "
                    f"종료 코드 {return_code})"
                )
            self._finish_batch(
                False,
                failure_message,
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
            plan_report=self.harvest_plan_report,
            motion_result_text=tomato_motion_result_text(
                execute,
                True,
                self.harvest_plan_report,
            ),
        )
        self._append_log(
            f"[전체 {self.batch_completed + self.batch_skipped + 1}/"
            f"{self.batch_total}] "
            f"토마토 {index} {mode} 완료"
        )
        if execute:
            self._record_batch_statistics(return_code, execute, verification)
            self.batch_completed += 1
            self._invalidate_plan()
        else:
            self.verified_plan = verification
        self._start_next_batch_job()

    def _finish_batch(self, success: bool, message: str) -> None:
        operation_name = (
            "전체 연속 수확"
            if getattr(self, "batch_execute_motion", True)
            else "전체 연속 Plan"
        )
        self.batch_active = False
        self.batch_jobs.clear()
        self.batch_generation = None
        self.batch_planner = None
        self.batch_pick_ready_state = "PICK_READY"
        self.batch_harvest_stage_limit = None
        self.batch_harvest_x_forward_m = 0.040
        self.batch_continuous_mode = False
        self.batch_lift_harvest_mode = False
        self.batch_preplan_mode = False
        self.batch_execute_motion = True
        self.batch_prefer_robot_direction = False
        self.batch_adaptive_grasp_max_rotation_deg = 45.0
        self.batch_preplan_failure_message = ""
        self.lift_harvest_pending = None
        self._clear_active_camera_target()
        self._invalidate_plan()
        self._set_busy(False)
        self.status.set(message)
        self.sweep_summary.set(
            f"{operation_name} 완료 — {message}"
        )
        result = "완료" if success else "중단"
        self._append_log(f"[{operation_name} {result}] {message}")

    def stop_active_motion(self) -> None:
        """Stop an individual, batch, or automatic harvest operation."""
        if self.sweep_active:
            self.stop_sweep()
            return
        step_process = getattr(self, "step_process", None)
        process = step_process or self.harvest_process
        lift_pending = getattr(self, "lift_harvest_pending", None)
        if (
            process is None
            and not self.batch_active
            and not getattr(self, "repeat_batch_active", False)
            and lift_pending is None
        ):
            return

        was_batch = self.batch_active
        was_repeat_batch = getattr(self, "repeat_batch_active", False)
        was_step = step_process is not None
        named_pose_state = getattr(self, "named_pose_active_state", None)
        step_mode = getattr(self, "step_session_mode", None)
        self.status.set("수확 작업 중지 및 로봇 모션 정지 명령 전송 중...")
        self._append_log(
            "[정지 요청] 현재 수확 작업을 취소하고 MoveIt/controller 및 "
            "실제 RB5 정지를 요청합니다."
        )
        self._request_robot_motion_stop()
        # A step may currently be driving the Arduino linear actuator. Force
        # both direction outputs LOW before terminating its worker process.
        pin_stop = getattr(self, "_publish_linear_motor_pin_levels", None)
        if pin_stop is not None:
            pin_stop(False, False)
            self._append_log(
                "[정지 요청] 리니어모터 PIN8/PIN9도 LOW로 정지했습니다."
            )
        if lift_pending is not None:
            self.lift_harvest_pending = None
            simulation = self.lift_simulation_mode
            publisher = (
                self.lift_simulated_stop_publisher
                if simulation
                else self.lift_stop_publisher
            )
            stop_message = Bool()
            stop_message.data = True
            publisher.publish(stop_message)
            self._append_log(
                "[정지 요청] 리프트 수확 높이 이동도 함께 정지했습니다."
            )

        # Detach first so the reader thread's eventual exit event cannot be
        # mistaken for a planning or execution failure after user cancellation.
        if was_step:
            self.step_process = None
            self.step_execution_in_progress = False
            self.step_session_failed = False
            self.step_session_verification = None
            self.step_session_mode = None
            self.repeat_run_direction = None
            self.repeat_pause_requested = False
            self.repeat_paused = False
            if hasattr(self, "repeat_pause_button"):
                self.repeat_pause_button.configure(text="일시 정지")
        else:
            self.harvest_process = None
            self.named_pose_active_state = None
        clear_active_target = getattr(
            self, "_clear_active_camera_target", None
        )
        if clear_active_target is not None:
            clear_active_target()
        if process is not None and process.poll() is None:
            process.terminate()

        self._invalidate_plan()
        if was_batch:
            message = (
                "사용자가 전체 연속 수확을 중지했습니다. "
                "MoveIt/controller 취소 및 RB 정지를 요청했습니다."
            )
        elif was_repeat_batch:
            message = (
                "사용자가 전체 토마토 접근 반복을 중지했습니다. "
                "MoveIt/controller 취소 및 RB 정지를 요청했습니다."
            )
            self.repeat_status.set(message)
        elif was_step:
            if step_mode == "repeat":
                message = (
                    "접근 반복 모션 즉시 정지 완료 — MoveIt/controller "
                    "취소 및 RB 정지를 요청했습니다. 새 반복 Plan이 필요합니다."
                )
                self.repeat_status.set(message)
            else:
                message = (
                    "스텝 모션 즉시 정지 완료 — MoveIt/controller 취소 및 "
                    "RB 정지를 요청했습니다. 새 스텝 Plan이 필요합니다."
                )
                self.step_status.set(message)
        elif named_pose_state:
            message = (
                f"저장 자세 {named_pose_state} 이동 중지 완료 — "
                "MoveIt/controller 취소 및 RB 정지를 요청했습니다."
            )
        else:
            message = (
                "수확 모션 중지 완료 — MoveIt/controller 취소 및 RB 정지를 "
                "요청했습니다."
            )
        if was_batch:
            self._finish_batch(False, message)
        elif was_repeat_batch:
            self._finish_repeat_batch(False, message)
        else:
            self._set_busy(False)
            self.status.set(message)
            self._append_log(message)

    def _set_busy(self, busy: bool) -> None:
        self.ui_busy = bool(busy)
        state = "disabled" if busy else "normal"
        detection_state = (
            "disabled"
            if busy or self.capture_camera_in_progress
            else "normal"
        )
        self.detect_button.configure(state=detection_state)
        self.capture_camera_button.configure(
            state="disabled" if self.capture_camera_in_progress else "normal"
        )
        self.camera_source_combo.configure(
            state=(
                "disabled"
                if busy or self.capture_camera_in_progress
                else "readonly"
            )
        )
        self.angle_reference_mode_combo.configure(
            state="disabled" if busy else "readonly"
        )
        self.named_pose_combo.configure(
            state="disabled" if busy else "readonly"
        )
        self.named_pose_button.configure(state=state)
        self.read_scene_button.configure(state=state)
        self.set_scene_button.configure(state=state)
        self.apply_speed_button.configure(state=state)
        self.linear_motor_wait_entry.configure(state=state)
        self.harvest_forward_distance_spinbox.configure(state=state)
        self.harvest_tcp_wrist_rotation_spinbox.configure(state=state)
        self.prefer_robot_direction_checkbox.configure(state=state)
        self.adaptive_grasp_max_rotation_entry.configure(state=state)
        ready_state = "disabled" if busy else "readonly"
        self.pick_ready_state_combo.configure(state=ready_state)
        self.sweep_pick_ready_state_combo.configure(state=ready_state)
        self.continuous_harvest_checkbox.configure(state=state)
        self.batch_harvest_stage_combo.configure(
            state="disabled" if busy else "readonly"
        )
        self.lift_harvest_checkbox.configure(state=state)
        self._update_preplan_checkbox_state()
        self.sweep_continuous_harvest_checkbox.configure(state=state)
        self.sweep_lift_harvest_checkbox.configure(state=state)
        self.step_execution_checkbox.configure(state=state)
        self.repeat_execution_checkbox.configure(state=state)
        self.tomato_combo.configure(state="disabled" if busy else "readonly")
        self.harvest_all_button.configure(
            state="disabled" if busy or not self.detected_tomatoes else "normal"
        )
        self.harvest_all_plan_button.configure(
            state="disabled" if busy or not self.detected_tomatoes else "normal"
        )
        self.motion_stop_button.configure(state="normal" if busy else "disabled")
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
        elif self._verification_matches_current_selection(
            self.verified_plan
        ):
            self.execute_button.configure(state="normal")
        self._update_step_controls()
        self._update_lift_controls()
        self._update_linear_motor_launch_control()
        self._update_gripper_stroke_controls()
        self._update_camera_target_record_controls()

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
        self.latest_detection_message = None
        self._update_detection_image_overlay()
        self.detected_tomato_expected_world_positions.clear()
        self.detected_tomato_expected_world_x_axes.clear()
        self.detected_tomato_record_positions.clear()
        self.detected_tomato_record_stem_positions.clear()
        self.detected_tomato_record_calyx_positions.clear()
        self.detected_tomato_record_angle_origins.clear()
        self.detected_tomato_record_angle_targets.clear()
        self._clear_active_camera_target()
        self.tomato_motion_results.clear()
        if preserve_sweep_markers:
            self.harvest_results.clear()
            self.harvest_result_adaptive_rotation.clear()
            self.harvest_result_adaptive_rotation_deg.clear()
            self.harvest_result_approach_axis_local.clear()
            self.harvest_result_approach_reports.clear()
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
        self.step_tomato_combo.configure(values=[])
        self.repeat_tomato_combo.configure(values=[])
        self.selected_tomato.set("")
        self.plan_button.configure(state="disabled")
        self.harvest_all_button.configure(state="disabled")
        self.harvest_all_plan_button.configure(state="disabled")
        self._invalidate_plan()
        self._clear_detection_markers()
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
        self._update_selected_tomato_plot()

    def _spin_ros(self) -> None:
        if self.closing or not rclpy.ok():
            return
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except ExternalShutdownException:
            return
        self.root.after(30, self._spin_ros)

    def _on_close(self) -> None:
        if (
            self.harvest_process is not None
            or self.step_process is not None
            or self.sweep_worker_process is not None
        ):
            if not messagebox.askyesno(
                "모션 작업 실행 중",
                "현재 프로세스를 종료하고 GUI를 닫을까요?\n"
                "프로세스 종료가 이미 전송된 로봇 궤적을 정지시키지는 않습니다.",
                icon="warning",
            ):
                return
            if self.harvest_process is not None:
                self.harvest_process.terminate()
            if self.step_process is not None:
                self.step_process.terminate()
            self._shutdown_sweep_worker(force=True)
        self.closing = True
        self.root.quit()

    def _signal_close(self, _signum, _frame) -> None:
        if self.harvest_process is not None:
            self.harvest_process.terminate()
        if self.step_process is not None:
            self.step_process.terminate()
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
