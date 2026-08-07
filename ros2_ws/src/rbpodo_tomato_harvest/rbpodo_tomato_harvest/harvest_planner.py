import copy
import math
import time
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    GenericTrajectory,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


def load_srdf_group_state(
    srdf_path,
    state_name: str,
    group_name: str,
    joint_names,
) -> list[float]:
    """Load an ordered named joint state from an SRDF file."""
    path = Path(srdf_path)
    root = ET.parse(path).getroot()
    group_state = next(
        (
            element
            for element in root.findall("group_state")
            if element.get("name") == state_name
            and element.get("group") == group_name
        ),
        None,
    )
    if group_state is None:
        raise ValueError(
            f"SRDF group_state not found: {state_name}/{group_name} ({path})"
        )
    values = {
        str(joint.get("name")): float(joint.get("value"))
        for joint in group_state.findall("joint")
    }
    missing = [str(name) for name in joint_names if str(name) not in values]
    if missing:
        raise ValueError(
            f"SRDF group_state {state_name} is missing joints: "
            + ", ".join(missing)
        )
    return [values[str(name)] for name in joint_names]


def default_pick_ready_joint_positions(
    joint_names,
    state_name: str = "PICK_READY",
) -> list[float]:
    """Load the selected ready state from the installed MoveIt SRDF."""
    srdf_path = (
        Path(get_package_share_directory("rbpodo_moveit_config"))
        / "config"
        / "rbpodo.srdf"
    )
    return load_srdf_group_state(
        srdf_path,
        str(state_name),
        "mainpulation",
        joint_names,
    )


def revolute_position_error(position: float, target: float) -> float:
    """Return shortest absolute angular error for equivalent joint turns."""
    difference = float(position) - float(target)
    return abs(math.atan2(math.sin(difference), math.cos(difference)))


@dataclass(frozen=True)
class HarvestGeometry:
    """Poses and axes used for one tomato Cartesian approach."""

    target_pose: Pose
    preapproach_pose: Pose
    outward_axis: np.ndarray
    vine_point: np.ndarray
    tomato_position: np.ndarray


@dataclass(frozen=True)
class TipLocalHarvestMotion:
    """Tool-frame waypoints before and after the harvest dwell."""

    before_wait_waypoints: tuple[Pose, ...]
    after_wait_pose: Pose


@dataclass(frozen=True)
class AdaptiveApproachDirection:
    """Robot-facing grasp direction selected from tomato-local axes."""

    outward_axis: np.ndarray
    applied_rotation_deg: float
    current_robot_error_deg: float
    selected_robot_error_deg: float


@dataclass(frozen=True)
class ApproachRotationEvaluation:
    """Fast IK feasibility result for one tomato-local approach angle."""

    rotation_deg: float
    feasible: bool
    joint_distance: float = math.inf
    moveit_error_code: int = MoveItErrorCodes.FAILURE
    rejection_reason: str = ""


@dataclass(frozen=True)
class DeadlineRotationGuard:
    """Minimum signed correction that keeps pre-grasp on the robot side."""

    active: bool
    minimum_rotation_deg: float
    nominal_robot_side_dot: float


@dataclass(frozen=True)
class HarvestMotionPlan:
    """Trajectories for the complete PICK_READY-to-harvest sequence."""

    pick_ready_trajectory: object
    preapproach_trajectory: object
    approach_trajectory: object
    after_wait_trajectory: object
    return_pick_ready_trajectory: object
    display_start_state: RobotState
    arc_reverse_recovery_trajectory: object = ()
    outward_retreat_trajectory: object = ()
    end_planning_pose: Pose | None = None
    step_approach_trajectories: tuple = ()
    return_pick_ready_is_cached_reverse: bool = False
    preapproach_planning_pose: Pose | None = None
    outward_axis: tuple = ()


def make_centered_joint_path_constraints(
    joint_positions: dict[str, float],
    tolerance_rad: float,
) -> Constraints:
    """Limit selected joints around their planning-stage start positions."""
    constraints = Constraints()
    constraints.name = "OMPL start-centered joint range limits"
    for joint_name, position in joint_positions.items():
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = str(joint_name)
        joint_constraint.position = float(position)
        joint_constraint.tolerance_above = float(tolerance_rad)
        joint_constraint.tolerance_below = float(tolerance_rad)
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    return constraints


def summarize_joint_trajectory_ranges(
    trajectories,
    start_positions: dict[str, float] | None = None,
) -> list[dict]:
    """Return per-joint extrema across one or more RobotTrajectory messages."""
    samples: dict[str, list[float]] = {}
    joint_order: list[str] = []
    for robot_trajectory in trajectories:
        trajectory = robot_trajectory.joint_trajectory
        names = list(trajectory.joint_names)
        for name in names:
            if name not in samples:
                samples[name] = []
                joint_order.append(name)
        for point in trajectory.points:
            for name, position in zip(names, point.positions):
                samples[name].append(float(position))

    summary = []
    for name in joint_order:
        values = samples[name]
        if not values:
            continue
        start = (
            float(start_positions[name])
            if start_positions is not None and name in start_positions
            else values[0]
        )
        minimum = min(start, *values)
        maximum = max(start, *values)
        summary.append(
            {
                "joint_name": name,
                "start_rad": start,
                "min_rad": minimum,
                "max_rad": maximum,
                "span_rad": maximum - minimum,
                "start_deg": math.degrees(start),
                "min_deg": math.degrees(minimum),
                "max_deg": math.degrees(maximum),
                "span_deg": math.degrees(maximum - minimum),
                "sample_count": len(values),
            }
        )
    return summary


def format_joint_trajectory_ranges(
    summary: list[dict],
    heading: str = "Pre-grasp까지의 관절 범위 (단위: deg)",
) -> str:
    """Format trajectory extrema as a fixed-width GUI/log table."""
    lines = [
        heading,
        "  Joint       Start       Min        Max       Span   Points",
        "  ---------- ---------  ---------  ---------  --------- ------",
    ]
    for item in summary:
        lines.append(
            f"  {item['joint_name']:<10} "
            f"{item['start_deg']:>9.2f}  "
            f"{item['min_deg']:>9.2f}  "
            f"{item['max_deg']:>9.2f}  "
            f"{item['span_deg']:>9.2f} "
            f"{item['sample_count']:>6}"
        )
    return "\n".join(lines)


def joint_span_violations(trajectories, maximum_span_deg: float) -> list[dict]:
    """Return joints whose cached path exceeds the allowed angular span."""
    limit = max(0.0, float(maximum_span_deg))
    ranges = summarize_joint_trajectory_ranges(trajectories)
    return [
        item
        for item in ranges
        if float(item["span_deg"]) > limit + 1e-9
    ]


def trajectory_joint_safety_violations(
    trajectories,
    start_positions: dict[str, float],
    limited_joint_names,
    maximum_span_deg: float,
    wrist3_maximum_span_deg: float,
    maximum_step_deg: float,
) -> list[dict]:
    """Reject long-turn joint branches and discontinuities before execution."""
    sequence = (
        tuple(trajectories)
        if isinstance(trajectories, (list, tuple))
        else (() if not trajectories else (trajectories,))
    )
    limited_names = [str(name) for name in limited_joint_names]
    safety_names = list(dict.fromkeys([*limited_names, "wrist3"]))
    samples = {
        name: [float(start_positions[name])]
        for name in safety_names
        if name in start_positions
    }
    maximum_steps = {name: 0.0 for name in safety_names}
    previous = {
        str(name): float(position)
        for name, position in start_positions.items()
    }

    for robot_trajectory in sequence:
        joint_trajectory = robot_trajectory.joint_trajectory
        names = [str(name) for name in joint_trajectory.joint_names]
        for point in joint_trajectory.points:
            for name, position in zip(names, point.positions):
                if name not in safety_names:
                    continue
                value = float(position)
                if name in previous:
                    maximum_steps[name] = max(
                        maximum_steps[name],
                        abs(value - previous[name]),
                    )
                samples.setdefault(name, []).append(value)
                previous[name] = value

    violations = []
    for name in safety_names:
        values = samples.get(name, [])
        if not values:
            violations.append(
                {
                    "joint_name": name,
                    "reason": "START_POSITION_MISSING",
                }
            )
            continue
        span_deg = math.degrees(max(values) - min(values))
        max_step_deg = math.degrees(maximum_steps.get(name, 0.0))
        span_limit = (
            float(wrist3_maximum_span_deg)
            if name == "wrist3"
            else float(maximum_span_deg)
        )
        reasons = []
        if span_deg > span_limit + 1e-9:
            reasons.append("SPAN_LIMIT_EXCEEDED")
        if max_step_deg > float(maximum_step_deg) + 1e-9:
            reasons.append("POINT_JUMP_LIMIT_EXCEEDED")
        if reasons:
            violations.append(
                {
                    "joint_name": name,
                    "reason": "+".join(reasons),
                    "span_deg": float(span_deg),
                    "maximum_span_deg": float(span_limit),
                    "max_step_deg": float(max_step_deg),
                    "maximum_step_deg": float(maximum_step_deg),
                }
            )
    return violations


def _unit(vector: np.ndarray, label: str) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError(f"Cannot normalize {label}: vector is nearly zero")
    return vector / norm


def quaternion_from_rotation(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to an xyzw quaternion."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.array([qx, qy, qz, qw], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def rotation_from_pose(pose: Pose) -> np.ndarray:
    """Return the base<-pose rotation matrix."""
    q = pose.orientation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def planning_pose_from_tip_pose(
    tip_pose: Pose,
    planning_to_tip_translation,
    planning_to_tip_rotation,
) -> Pose:
    """Convert a desired tip pose into an equivalent planning-link pose.

    ``planning_to_tip_*`` describes the fixed planning-link -> tip transform.
    The returned pose places the planning link so that the rigidly attached tip
    reaches exactly ``tip_pose``.
    """
    tip_translation = np.asarray(planning_to_tip_translation, dtype=float)
    tip_rotation = np.asarray(planning_to_tip_rotation, dtype=float)
    if tip_translation.shape != (3,):
        raise ValueError("planning_to_tip_translation must contain three values")
    if tip_rotation.shape != (3, 3):
        raise ValueError("planning_to_tip_rotation must be a 3x3 matrix")

    base_to_tip_rotation = rotation_from_pose(tip_pose)
    base_to_planning_rotation = base_to_tip_rotation @ tip_rotation.T
    base_to_tip_translation = np.array(
        [tip_pose.position.x, tip_pose.position.y, tip_pose.position.z],
        dtype=float,
    )
    base_to_planning_translation = (
        base_to_tip_translation
        - base_to_planning_rotation @ tip_translation
    )

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (
        float(base_to_planning_translation[0]),
        float(base_to_planning_translation[1]),
        float(base_to_planning_translation[2]),
    )
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion_from_rotation(base_to_planning_rotation)
    return pose


def _normalized_quaternion(pose: Pose) -> np.ndarray:
    quaternion = np.array(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        dtype=float,
    )
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError("pose orientation quaternion is nearly zero")
    return quaternion / norm


def _slerp_quaternion(start: np.ndarray, finish: np.ndarray, ratio: float):
    start = np.asarray(start, dtype=float)
    finish = np.asarray(finish, dtype=float)
    dot = float(np.dot(start, finish))
    if dot < 0.0:
        finish = -finish
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        result = start + float(ratio) * (finish - start)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    sine = math.sin(angle)
    return (
        math.sin((1.0 - float(ratio)) * angle) / sine * start
        + math.sin(float(ratio) * angle) / sine * finish
    )


def make_continuous_arc_waypoints(
    start_pose: Pose,
    target_pose: Pose,
    outward_axis,
    minimum_clearance: float = 0.12,
    maximum_clearance: float = 0.25,
    waypoint_count: int = 7,
) -> tuple[Pose, ...]:
    """Create a smooth, horizontally outward-bowing TCP transition.

    The lift can create a large Z difference between the retained harvest pose
    and the next pre-grasp.  The clearance bow must therefore stay in the XY
    plane; projecting it against the 3D chord introduces an artificial Z term
    that makes the arm dip below both poses after a lift move.
    """
    count = max(2, int(waypoint_count))
    start = np.array(
        [start_pose.position.x, start_pose.position.y, start_pose.position.z],
        dtype=float,
    )
    target = np.array(
        [target_pose.position.x, target_pose.position.y, target_pose.position.z],
        dtype=float,
    )
    delta = target - start
    distance = float(np.linalg.norm(delta))
    if distance < 1e-6:
        return (target_pose,)

    requested_outward = np.array(outward_axis, dtype=float, copy=True)
    requested_outward[2] = 0.0
    requested_outward = _unit(requested_outward, "continuous arc outward axis")
    arc_direction = requested_outward

    minimum = max(0.0, float(minimum_clearance))
    maximum = max(minimum, float(maximum_clearance))
    clearance = min(maximum, max(minimum, 0.5 * distance))
    start_quaternion = _normalized_quaternion(start_pose)
    target_quaternion = _normalized_quaternion(target_pose)

    waypoints = []
    for index in range(1, count + 1):
        ratio = index / count
        position = (
            start
            + ratio * delta
            + arc_direction * clearance * math.sin(math.pi * ratio)
        )
        quaternion = _slerp_quaternion(
            start_quaternion,
            target_quaternion,
            ratio,
        )
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = (float(value) for value in quaternion)
        waypoints.append(pose)
    return tuple(waypoints)


def make_tip_local_harvest_motion(
    start_pose: Pose,
    x_forward: float = 0.070,
    first_z_lift: float = 0.040,
    first_x_back: float = 0.050,
    second_z_lift: float = 0.010,
    second_x_back: float = 0.010,
) -> TipLocalHarvestMotion:
    """Build the post-contact sequence along tomato_gripper_tip X/Z axes."""
    rotation = rotation_from_pose(start_pose)
    tip_x = rotation[:, 0]
    tip_z = rotation[:, 2]
    position = np.array(
        [start_pose.position.x, start_pose.position.y, start_pose.position.z],
        dtype=float,
    )

    def moved(delta: np.ndarray) -> Pose:
        nonlocal position
        position = position + delta
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )
        pose.orientation = start_pose.orientation
        return pose

    before_wait = (
        moved(tip_x * float(x_forward)),
        moved(tip_z * float(first_z_lift)),
        moved(-tip_x * float(first_x_back)),
        moved(tip_z * float(second_z_lift)),
    )
    after_wait = moved(-tip_x * float(second_x_back))
    return TipLocalHarvestMotion(before_wait, after_wait)


def stemward_and_outward_from_tomato_rotation(
    tomato_rotation,
) -> tuple[np.ndarray, np.ndarray]:
    """Extract horizontal stemward (+X) and robot-side (-X) axes."""
    rotation = np.asarray(tomato_rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("tomato_rotation must be a 3x3 matrix")
    tomato_x = rotation[:, 0]
    horizontal_stemward = np.array([tomato_x[0], tomato_x[1], 0.0])
    stemward = _unit(horizontal_stemward, "detected tomato stem direction")
    return stemward, -stemward


def outward_from_tomato_rotation(
    tomato_rotation,
    rotation_deg: float,
) -> np.ndarray:
    """Rotate tomato-local -X toward local +Y by a signed angle."""
    rotation = np.asarray(tomato_rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("tomato_rotation must be a 3x3 matrix")
    angle = math.radians(float(rotation_deg))
    local_outward = np.array(
        [-math.cos(angle), math.sin(angle), 0.0],
        dtype=float,
    )
    world_outward = rotation @ local_outward
    world_outward[2] = 0.0
    return _unit(world_outward, "rotated tomato approach direction")


def deadline_safe_rotation_guard(
    tomato_rotation,
    tomato_position,
    robot_position,
    margin_deg: float = 0.0,
) -> DeadlineRotationGuard:
    """Return the ideal boundary angle when nominal pre-grasp is too deep."""
    rotation = np.asarray(tomato_rotation, dtype=float)
    tomato = np.asarray(tomato_position, dtype=float)
    robot = np.asarray(robot_position, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("tomato_rotation must be a 3x3 matrix")
    if tomato.shape != (3,) or robot.shape != (3,):
        raise ValueError("tomato_position and robot_position must be 3D")

    nominal_outward = outward_from_tomato_rotation(rotation, 0.0)
    robotward_vector = robot - tomato
    if float(np.linalg.norm(robotward_vector)) < 1e-9:
        return DeadlineRotationGuard(False, 0.0, 1.0)
    robotward = _unit(robotward_vector, "tomato-to-robot direction")
    robot_side_dot = float(np.dot(nominal_outward, robotward))
    current_error_deg = math.degrees(
        math.acos(max(-1.0, min(1.0, robot_side_dot)))
    )
    safety_margin_deg = min(90.0, max(0.0, float(margin_deg)))
    maximum_safe_error_deg = 90.0 - safety_margin_deg
    if current_error_deg <= maximum_safe_error_deg + 1e-9:
        return DeadlineRotationGuard(False, 0.0, robot_side_dot)

    preferred = adaptive_outward_toward_robot(
        tomato_rotation=rotation,
        tomato_position=tomato,
        robot_position=robot,
        max_rotation_deg=90.0,
        deadband_deg=0.0,
    )
    direction_sign = (
        1.0 if preferred.applied_rotation_deg >= 0.0 else -1.0
    )
    # The candidate approach remains level, but the base can be substantially
    # below the tomato. Convert the 3D cone limit into the corresponding,
    # narrower XY angular limit before calculating the required correction.
    horizontal_robotward = np.array(
        [robotward[0], robotward[1], 0.0],
        dtype=float,
    )
    horizontal_scale = float(np.linalg.norm(horizontal_robotward))
    required_dot = math.cos(math.radians(maximum_safe_error_deg))
    if horizontal_scale <= 1e-9 or horizontal_scale < required_dot - 1e-9:
        boundary_magnitude = 90.0
    else:
        horizontal_robotward = _unit(
            horizontal_robotward,
            "horizontal tomato-to-robot direction",
        )
        horizontal_error_deg = math.degrees(
            math.acos(
                max(
                    -1.0,
                    min(
                        1.0,
                        float(np.dot(nominal_outward, horizontal_robotward)),
                    ),
                )
            )
        )
        maximum_horizontal_error_deg = math.degrees(
            math.acos(
                max(-1.0, min(1.0, required_dot / horizontal_scale))
            )
        )
        boundary_magnitude = max(
            0.0,
            horizontal_error_deg - maximum_horizontal_error_deg,
        )
    return DeadlineRotationGuard(
        True,
        direction_sign * min(90.0, boundary_magnitude),
        robot_side_dot,
    )


def robot_side_approach_error_deg(
    outward_axis,
    tomato_position,
    robot_position,
) -> float:
    """Return the 3D angle between an approach and tomato-to-robot."""
    outward = np.asarray(outward_axis, dtype=float)
    tomato = np.asarray(tomato_position, dtype=float)
    robot = np.asarray(robot_position, dtype=float)
    if outward.shape != (3,):
        raise ValueError("outward_axis must be 3D")
    if tomato.shape != (3,) or robot.shape != (3,):
        raise ValueError("tomato_position and robot_position must be 3D")

    robotward_vector = robot - tomato
    if float(np.linalg.norm(robotward_vector)) < 1e-9:
        return 0.0
    outward_direction = _unit(outward, "approach direction")
    robotward = _unit(robotward_vector, "tomato-to-robot direction")
    return math.degrees(
        math.acos(
            max(-1.0, min(1.0, float(np.dot(outward_direction, robotward))))
        )
    )


def is_within_robot_side_approach_sector(
    outward_axis,
    tomato_position,
    robot_position,
    margin_deg: float = 0.0,
) -> bool:
    """Whether an approach is inside the robot-facing 180-2*margin sector."""
    safety_margin_deg = min(90.0, max(0.0, float(margin_deg)))
    maximum_error_deg = 90.0 - safety_margin_deg
    return (
        robot_side_approach_error_deg(
            outward_axis,
            tomato_position,
            robot_position,
        )
        <= maximum_error_deg + 1e-9
    )


def select_minimum_feasible_rotation(
    evaluator,
    max_rotation_deg: float = 90.0,
    coarse_step_deg: float = 10.0,
    resolution_deg: float = 1.0,
    preferred_sign: float = 1.0,
    minimum_abs_rotation_deg: float = 0.0,
    allow_opposite_sign: bool = True,
) -> tuple[ApproachRotationEvaluation | None, tuple[ApproachRotationEvaluation, ...]]:
    """
    Find the smallest feasible |angle| using fast endpoint IK checks.

    OMPL is deliberately not involved.  Angles are checked in increasing
    absolute order on both sides, then the first feasible interval is refined
    by binary search.  Joint distance only breaks ties between equally small
    rotations.
    """
    maximum = max(0.0, min(90.0, float(max_rotation_deg)))
    coarse_step = max(0.1, float(coarse_step_deg))
    resolution = max(0.1, float(resolution_deg))
    first_sign = 1.0 if float(preferred_sign) >= 0.0 else -1.0
    minimum = max(0.0, min(maximum, float(minimum_abs_rotation_deg)))
    evaluations: list[ApproachRotationEvaluation] = []
    cache: dict[float, ApproachRotationEvaluation] = {}

    def evaluate(angle: float) -> ApproachRotationEvaluation:
        normalized = round(float(angle), 6)
        if normalized not in cache:
            result = evaluator(normalized)
            if not isinstance(result, ApproachRotationEvaluation):
                raise TypeError(
                    "rotation evaluator must return ApproachRotationEvaluation"
                )
            cache[normalized] = result
            evaluations.append(result)
        return cache[normalized]

    if minimum <= 1e-9:
        zero = evaluate(0.0)
        if zero.feasible or maximum <= 0.0:
            return (zero if zero.feasible else None), tuple(evaluations)
    elif maximum <= 0.0:
        return None, tuple(evaluations)

    previous_magnitude = minimum
    magnitude = minimum
    while magnitude <= maximum + 1e-9:
        endpoints = [evaluate(first_sign * magnitude)]
        if allow_opposite_sign:
            endpoints.append(evaluate(-first_sign * magnitude))
        feasible_endpoints = [item for item in endpoints if item.feasible]
        if feasible_endpoints:
            refined: list[ApproachRotationEvaluation] = []
            for endpoint in feasible_endpoints:
                sign = 1.0 if endpoint.rotation_deg >= 0.0 else -1.0
                lower = previous_magnitude
                upper = magnitude
                best = endpoint
                while upper - lower > resolution:
                    middle = 0.5 * (lower + upper)
                    candidate = evaluate(sign * middle)
                    if candidate.feasible:
                        upper = middle
                        best = candidate
                    else:
                        lower = middle
                refined.append(best)
            selected = min(
                refined,
                key=lambda item: (
                    abs(item.rotation_deg),
                    item.joint_distance,
                    0 if item.rotation_deg * first_sign >= 0.0 else 1,
                ),
            )
            return selected, tuple(evaluations)

        if math.isclose(magnitude, maximum, abs_tol=1e-9):
            break
        previous_magnitude = magnitude
        magnitude = min(
            maximum,
            magnitude + coarse_step,
        )

    return None, tuple(evaluations)


def select_robotward_feasible_rotation(
    evaluator,
    desired_rotation_deg: float,
    max_rotation_deg: float = 90.0,
    coarse_step_deg: float = 10.0,
    resolution_deg: float = 1.0,
    minimum_abs_rotation_deg: float = 0.0,
) -> tuple[ApproachRotationEvaluation | None, tuple[ApproachRotationEvaluation, ...]]:
    """Find the feasible angle closest to the requested robotward angle.

    Candidates move from the robotward target back toward the recommend angle.
    Once a feasible coarse endpoint is found, the boundary toward the robot is
    refined so the returned angle remains as robot-facing as possible.
    """
    maximum = max(0.0, min(90.0, float(max_rotation_deg)))
    coarse_step = max(0.1, float(coarse_step_deg))
    resolution = max(0.1, float(resolution_deg))
    desired = max(
        -maximum,
        min(maximum, float(desired_rotation_deg)),
    )
    sign = 1.0 if desired >= 0.0 else -1.0
    minimum = max(
        0.0,
        min(maximum, float(minimum_abs_rotation_deg)),
    )
    target = max(minimum, abs(desired))
    evaluations: list[ApproachRotationEvaluation] = []
    cache: dict[float, ApproachRotationEvaluation] = {}

    def evaluate(angle: float) -> ApproachRotationEvaluation:
        normalized = round(float(angle), 6)
        if normalized not in cache:
            result = evaluator(normalized)
            if not isinstance(result, ApproachRotationEvaluation):
                raise TypeError(
                    "rotation evaluator must return ApproachRotationEvaluation"
                )
            cache[normalized] = result
            evaluations.append(result)
        return cache[normalized]

    magnitude = target
    previous_infeasible = None
    while magnitude >= minimum - 1e-9:
        candidate = evaluate(sign * magnitude)
        if candidate.feasible:
            best = candidate
            if previous_infeasible is not None:
                lower = magnitude
                upper = previous_infeasible
                while upper - lower > resolution:
                    middle = 0.5 * (lower + upper)
                    refined = evaluate(sign * middle)
                    if refined.feasible:
                        lower = middle
                        best = refined
                    else:
                        upper = middle
            return best, tuple(evaluations)
        if math.isclose(magnitude, minimum, abs_tol=1e-9):
            break
        previous_infeasible = magnitude
        magnitude = max(minimum, magnitude - coarse_step)

    return None, tuple(evaluations)


def adaptive_outward_toward_robot(
    tomato_rotation,
    tomato_position,
    robot_position,
    max_rotation_deg: float = 90.0,
    deadband_deg: float = 10.0,
) -> AdaptiveApproachDirection:
    """Rotate tomato -X toward the robot through local ±Y."""
    rotation = np.asarray(tomato_rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("tomato_rotation must be a 3x3 matrix")

    stemward, current_outward = (
        stemward_and_outward_from_tomato_rotation(rotation)
    )
    tomato_y = np.array([rotation[0, 1], rotation[1, 1], 0.0], dtype=float)
    tomato_y = tomato_y - stemward * float(np.dot(tomato_y, stemward))
    tomato_y = _unit(tomato_y, "detected tomato +Y direction")

    tomato = np.asarray(tomato_position, dtype=float)
    robot = np.asarray(robot_position, dtype=float)
    if tomato.shape != (3,) or robot.shape != (3,):
        raise ValueError("tomato_position and robot_position must be 3D")
    robotward_horizontal = np.array(
        [robot[0] - tomato[0], robot[1] - tomato[1], 0.0],
        dtype=float,
    )
    if float(np.linalg.norm(robotward_horizontal)) < 1e-9:
        return AdaptiveApproachDirection(
            current_outward,
            0.0,
            0.0,
            0.0,
        )
    robotward = _unit(robotward_horizontal, "tomato-to-robot direction")

    def signed_angle(source: np.ndarray, target: np.ndarray) -> float:
        cross_z = float(source[0] * target[1] - source[1] * target[0])
        return math.atan2(cross_z, float(np.dot(source, target)))

    toward_y_angle = signed_angle(current_outward, tomato_y)
    toward_y_sign = 1.0 if toward_y_angle >= 0.0 else -1.0
    robot_angle = signed_angle(current_outward, robotward)
    robot_angle_in_local_y = toward_y_sign * robot_angle
    current_error_deg = abs(math.degrees(robot_angle))

    maximum = max(0.0, min(90.0, float(max_rotation_deg)))
    deadband = max(0.0, float(deadband_deg))
    applied_rotation_deg = 0.0
    if abs(robot_angle_in_local_y) > math.radians(deadband):
        applied_rotation_deg = max(
            -maximum,
            min(
                maximum,
                math.degrees(robot_angle_in_local_y),
            ),
        )

    selected_outward = outward_from_tomato_rotation(
        rotation,
        applied_rotation_deg,
    )
    selected_error_deg = abs(
        math.degrees(signed_angle(selected_outward, robotward))
    )
    return AdaptiveApproachDirection(
        selected_outward,
        applied_rotation_deg,
        current_error_deg,
        selected_error_deg,
    )


def make_harvest_geometry(
    tomato_position,
    vine_origin,
    vine_axis,
    tip_standoff: float,
    tip_below_center: float,
    preapproach_clearance: float,
    outward_hint=None,
    tip_rotation_from_gripper=None,
) -> HarvestGeometry:
    """
    Create a level gripper pose with tomato between tip and stem side.

    The physical gripper's +X axis points from the tomato toward the robot and
    its -X axis points toward the tomato. Its +Y axis points vertically down,
    which puts the gripper's X-Z plane parallel to the ground (nominal roll
    -90 deg). The fixed gripper-to-tip rotation is then composed into the
    MoveIt target-link orientation. If no approach-direction hint is supplied,
    the direction away from the nearest point on the vine is used.
    """
    tomato = np.asarray(tomato_position, dtype=float)
    vine_center = np.asarray(vine_origin, dtype=float)
    vine_direction = _unit(np.asarray(vine_axis, dtype=float), "vine axis")
    closest_vine_point = vine_center + vine_direction * float(
        np.dot(tomato - vine_center, vine_direction)
    )

    radial = tomato - closest_vine_point
    direction = radial if outward_hint is None else np.asarray(outward_hint, dtype=float)
    horizontal_direction = np.array([direction[0], direction[1], 0.0], dtype=float)
    outward = _unit(horizontal_direction, "horizontal approach direction")
    world_up = np.array([0.0, 0.0, 1.0], dtype=float)
    gripper_down = -world_up
    gripper_lateral = _unit(
        np.cross(outward, gripper_down), "gripper lateral axis"
    )
    gripper_rotation = np.column_stack((outward, gripper_down, gripper_lateral))
    if tip_rotation_from_gripper is None:
        tip_rotation_from_gripper = np.eye(3, dtype=float)
    tip_rotation_from_gripper = np.asarray(tip_rotation_from_gripper, dtype=float)
    if tip_rotation_from_gripper.shape != (3, 3):
        raise ValueError("tip_rotation_from_gripper must be a 3x3 matrix")
    tip_rotation = gripper_rotation @ tip_rotation_from_gripper
    quaternion = quaternion_from_rotation(tip_rotation)

    target_position = (
        tomato
        + outward * max(0.0, float(tip_standoff))
        - world_up * max(0.0, float(tip_below_center))
    )
    preapproach_position = target_position + outward * max(
        0.0, float(preapproach_clearance)
    )

    def pose_at(position: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        return pose

    return HarvestGeometry(
        target_pose=pose_at(target_position),
        preapproach_pose=pose_at(preapproach_position),
        outward_axis=outward,
        vine_point=closest_vine_point,
        tomato_position=tomato,
    )


class CartesianHarvestPlanner(Node):
    """Plan and optionally execute a level Cartesian tomato approach."""

    def __init__(self) -> None:
        super().__init__("tomato_cartesian_harvest_test")
        self.declare_parameter("base_frame", "link0")
        self.declare_parameter("tomato_frame", "detected_tomato_3_tf")
        self.declare_parameter("gripper_link", "tomato_gripper")
        self.declare_parameter("tip_link", "tomato_gripper_tip")
        self.declare_parameter("planning_link", "tcp")
        self.declare_parameter("group_name", "mainpulation")
        self.declare_parameter("robot_model_id", "rb")
        pick_ready_joint_names = [
            "base",
            "shoulder",
            "elbow",
            "wrist1",
            "wrist2",
            "wrist3",
        ]
        self.declare_parameter(
            "pick_ready_joint_names",
            pick_ready_joint_names,
        )
        self.declare_parameter("pick_ready_state_name", "PICK_READY")
        self.declare_parameter(
            "pick_ready_joint_positions",
            default_pick_ready_joint_positions(
                pick_ready_joint_names,
                str(self.get_parameter("pick_ready_state_name").value),
            ),
        )
        self.declare_parameter(
            "display_passive_joint_names",
            ["farmily_lift_height_joint"],
        )
        self.declare_parameter(
            "dynamic_base_joint_name",
            "farmily_lift_height_joint",
        )
        self.declare_parameter("dynamic_base_parent_frame", "world")
        self.declare_parameter("dynamic_base_sync_timeout_sec", 1.0)
        self.declare_parameter("pick_ready_joint_tolerance", 0.005)
        self.declare_parameter("pick_ready_planning_time", 10.0)
        self.declare_parameter("pick_ready_planning_attempts", 5)
        self.declare_parameter("ompl_pose_planning_time", 2.0)
        self.declare_parameter("ompl_pose_planning_attempts", 2)
        self.declare_parameter("pick_ready_velocity_scale", 0.20)
        self.declare_parameter("pick_ready_acceleration_scale", 0.20)
        self.declare_parameter("planning_pipeline_id", "ompl")
        self.declare_parameter("planner_id", "RRTConnect")
        self.declare_parameter("preapproach_mode", "cartesian")
        self.declare_parameter("continuous_transition", False)
        self.declare_parameter("continuous_arc_min_clearance", 0.12)
        self.declare_parameter("continuous_arc_max_clearance", 0.25)
        self.declare_parameter("continuous_arc_waypoint_count", 7)
        self.declare_parameter("continuous_arc_max_joint_span_deg", 120.0)
        self.declare_parameter("return_to_pick_ready", True)
        self.declare_parameter("retreat_after_harvest", False)
        self.declare_parameter("lift_retreat_clearance", 0.12)
        self.declare_parameter("joint_planning_pipeline_id", "ompl")
        self.declare_parameter("joint_planner_id", "RRTConnect")
        self.declare_parameter(
            "ompl_limited_joint_names",
            ["base", "shoulder", "elbow", "wrist1", "wrist2"],
        )
        self.declare_parameter("ompl_joint_tolerance_deg", 120.0)
        self.declare_parameter("preapproach_position_tolerance", 0.005)
        self.declare_parameter("preapproach_orientation_tolerance", 0.05)
        self.declare_parameter("adaptive_grasp_enabled", True)
        self.declare_parameter("adaptive_grasp_max_rotation_deg", 45.0)
        self.declare_parameter(
            "adaptive_grasp_prefer_robot_direction", False
        )
        self.declare_parameter("adaptive_grasp_deadband_deg", 10.0)
        self.declare_parameter("adaptive_grasp_ik_timeout_sec", 0.05)
        self.declare_parameter("adaptive_grasp_ik_service_wait_sec", 0.5)
        self.declare_parameter("adaptive_grasp_search_step_deg", 10.0)
        self.declare_parameter("adaptive_grasp_search_resolution_deg", 1.0)
        self.declare_parameter("adaptive_grasp_deadline_margin_deg", 0.0)
        self.declare_parameter("tip_standoff", 0.025)
        self.declare_parameter("tip_below_center", 0.018)
        self.declare_parameter("preapproach_clearance", 0.010)
        self.declare_parameter("harvest_x_forward", 0.070)
        self.declare_parameter("harvest_first_z_lift", 0.040)
        self.declare_parameter("harvest_first_x_back", 0.050)
        self.declare_parameter("harvest_second_z_lift", 0.010)
        self.declare_parameter("harvest_wait_sec", 2.0)
        self.declare_parameter("harvest_second_x_back", 0.010)
        self.declare_parameter("max_step", 0.005)
        self.declare_parameter("jump_threshold", 2.0)
        self.declare_parameter(
            "cartesian_revolute_jump_threshold_deg", 20.0
        )
        self.declare_parameter(
            "cartesian_prismatic_jump_threshold_m", 0.02
        )
        self.declare_parameter("trajectory_safety_max_span_deg", 120.0)
        self.declare_parameter(
            "trajectory_safety_wrist3_max_span_deg", 180.0
        )
        self.declare_parameter("trajectory_safety_max_step_deg", 45.0)
        self.declare_parameter("minimum_fraction", 0.98)
        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("execution_timeout_sec", 60.0)
        self.declare_parameter("publish_display_trajectory", True)
        self.declare_parameter("stepwise_plan", False)
        self.declare_parameter("step_cycle_only", False)
        self.declare_parameter("step_cycle_last_stage", 5)
        self.declare_parameter("execute", False)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tomato_frame = str(self.get_parameter("tomato_frame").value)
        self.gripper_link = str(self.get_parameter("gripper_link").value)
        self.tip_link = str(self.get_parameter("tip_link").value)
        self.planning_link = str(self.get_parameter("planning_link").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.robot_model_id = str(self.get_parameter("robot_model_id").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self.display_publisher = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )
        self._latest_joint_positions: dict[str, float] = {}
        self._joint_state_received_monotonic: dict[str, float] = {}
        self._joint_state_stamp_nanoseconds: dict[str, int] = {}
        self._plan_start_joint_positions: dict[str, float] = {}
        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )
        self.last_plan_report = {}
        self._trajectory_range_records = []

    def _joint_state_callback(self, message: JointState) -> None:
        received_monotonic = time.monotonic()
        stamp_nanoseconds = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        for name, position in zip(message.name, message.position):
            if math.isfinite(position):
                joint_name = str(name)
                self._latest_joint_positions[joint_name] = float(position)
                self._joint_state_received_monotonic[joint_name] = (
                    received_monotonic
                )
                self._joint_state_stamp_nanoseconds[joint_name] = (
                    stamp_nanoseconds
                )

    def _synchronize_dynamic_base_transform(self) -> bool:
        """Wait for a fresh lift joint sample and its resulting base TF.

        The persistent automatic-test worker does not spin while waiting for its
        next stdin request.  Without this barrier its TF buffer can still contain
        the previous lift height even though the GUI has observed that the lift
        already reached the next target.
        """
        joint_name = str(
            self.get_parameter("dynamic_base_joint_name").value
        ).strip()
        if not joint_name:
            return True
        parent_frame = str(
            self.get_parameter("dynamic_base_parent_frame").value
        ).strip()
        timeout_sec = max(
            0.1,
            float(
                self.get_parameter("dynamic_base_sync_timeout_sec").value
            ),
        )
        requested_monotonic = float(
            self.last_plan_report.get(
                "_started_monotonic",
                time.monotonic(),
            )
        )
        joint_deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and time.monotonic() < joint_deadline:
            received = self._joint_state_received_monotonic.get(
                joint_name, 0.0
            )
            if received >= requested_monotonic:
                break
            rclpy.spin_once(self, timeout_sec=0.02)
        else:
            self.get_logger().error(
                "동적 베이스 동기화 실패: 새 리프트 관절 상태를 "
                f"{timeout_sec:.1f}초 안에 받지 못했습니다 ({joint_name})."
            )
            return False

        joint_stamp = self._joint_state_stamp_nanoseconds.get(joint_name, 0)
        # Give the matching robot_state_publisher TF its own full timeout.
        # Previously the joint-state wait and TF wait shared one deadline, so a
        # valid TF could be rejected immediately when the joint update arrived
        # near the end of the first phase.
        tf_deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < tf_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent_frame,
                    self.base_frame,
                    Time(),
                )
            except TransformException:
                continue
            tf_stamp = (
                int(transform.header.stamp.sec) * 1_000_000_000
                + int(transform.header.stamp.nanosec)
            )
            if joint_stamp <= 0 or tf_stamp >= joint_stamp:
                lift_position = self._latest_joint_positions.get(joint_name)
                self.last_plan_report["dynamic_base_sync"] = {
                    "joint_name": joint_name,
                    "joint_position_m": lift_position,
                    "parent_frame": parent_frame,
                    "base_frame": self.base_frame,
                    "base_height_m": float(
                        transform.transform.translation.z
                    ),
                    "joint_stamp_nanoseconds": joint_stamp,
                    "tf_stamp_nanoseconds": tf_stamp,
                }
                self.get_logger().info(
                    "동적 베이스 동기화 완료: "
                    f"{joint_name}={lift_position:.4f}m, "
                    f"{parent_frame}->{self.base_frame} Z="
                    f"{transform.transform.translation.z:.4f}m"
                )
                return True

        self.get_logger().error(
            "동적 베이스 동기화 실패: 새 리프트 관절 상태 이후의 "
            f"{parent_frame}->{self.base_frame} TF를 "
            f"{timeout_sec:.1f}초 안에 받지 못했습니다."
        )
        return False

    def _wait_for_current_joint_positions(
        self,
        timeout_sec: float = 2.0,
    ) -> dict[str, float]:
        required_names = [
            str(name)
            for name in self.get_parameter("pick_ready_joint_names").value
        ]
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            if all(name in self._latest_joint_positions for name in required_names):
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        return {
            name: self._latest_joint_positions[name]
            for name in required_names
            if name in self._latest_joint_positions
        }

    def _complete_display_start_state(
        self,
        start_state: RobotState,
    ) -> RobotState:
        """Keep passive joints at their live positions in RViz trajectory playback.

        MoveIt arm trajectories contain only the six planning-group joints.  RViz
        otherwise initializes an omitted lift joint at zero while playing the
        trajectory, which makes a correctly planned lifted harvest look as if it
        were executed below the tomato.
        """
        state = copy.deepcopy(start_state)
        names = [str(name) for name in state.joint_state.name]
        positions = [float(value) for value in state.joint_state.position]
        current_positions = dict(self._latest_joint_positions)
        required_names = [
            str(name)
            for name in self.get_parameter("pick_ready_joint_names").value
        ]
        passive_names = [
            str(name)
            for name in self.get_parameter("display_passive_joint_names").value
        ]

        for name in (*required_names, *passive_names):
            if name in names or name not in current_positions:
                continue
            names.append(name)
            positions.append(float(current_positions[name]))

        state.joint_state.name = names
        state.joint_state.position = positions
        if all(name in names for name in (*required_names, *passive_names)):
            state.is_diff = False
        return state

    def _begin_plan_report(
        self,
        start_state_override: RobotState | None = None,
    ) -> None:
        started_monotonic = time.monotonic()
        self._trajectory_range_records = []
        if start_state_override is None:
            self._plan_start_joint_positions = (
                self._wait_for_current_joint_positions()
            )
        else:
            self._plan_start_joint_positions = {
                str(name): float(position)
                for name, position in zip(
                    start_state_override.joint_state.name,
                    start_state_override.joint_state.position,
                )
            }
        self.last_plan_report = {
            "success": False,
            "tomato_frame": self.tomato_frame,
            "pipeline": str(self.get_parameter("planning_pipeline_id").value),
            "planner_id": str(self.get_parameter("planner_id").value),
            "preapproach_mode": str(
                self.get_parameter("preapproach_mode").value
            ),
            "continuous_transition": bool(
                self.get_parameter("continuous_transition").value
            ),
            "return_to_pick_ready": bool(
                self.get_parameter("return_to_pick_ready").value
            ),
            "pick_ready_state_name": str(
                self.get_parameter("pick_ready_state_name").value
            ),
            "retreat_after_harvest": bool(
                self.get_parameter("retreat_after_harvest").value
            ),
            "start_joint_positions": dict(
                self._plan_start_joint_positions
            ),
            "stages": [],
            "_started_monotonic": started_monotonic,
        }

    def _record_plan_stage(
        self,
        stage: str,
        planner_type: str,
        success: bool,
        duration_sec: float,
        reason: str = "",
        terminal_failure: bool = True,
        **details,
    ) -> None:
        record = {
            "stage": stage,
            "planner_type": planner_type,
            "success": bool(success),
            "duration_sec": round(float(duration_sec), 6),
            "reason": reason,
        }
        record.update(details)
        self.last_plan_report.setdefault("stages", []).append(record)
        if (
            not success
            and terminal_failure
            and "failure_stage" not in self.last_plan_report
        ):
            self.last_plan_report["failure_stage"] = stage
            self.last_plan_report["failure_planner_type"] = planner_type
            self.last_plan_report["failure_reason"] = reason
            self.last_plan_report.update(
                {
                    key: value
                    for key, value in details.items()
                    if key in {
                        "moveit_error_code",
                        "cartesian_fraction",
                        "required_fraction",
                    }
                }
            )

    def _record_trajectory_range_input(
        self,
        stage: str,
        trajectory,
        accepted: bool,
        pregrasp: bool,
    ) -> None:
        point_count = len(trajectory.joint_trajectory.points)
        if point_count:
            self._trajectory_range_records.append(
                {
                    "stage": stage,
                    "trajectory": trajectory,
                    "accepted": bool(accepted),
                    "pregrasp": bool(pregrasp),
                    "point_count": point_count,
                }
            )

    def _record_failure_robot_state(self, stage: str, trajectory) -> None:
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            return
        last_point = joint_trajectory.points[-1]
        self.last_plan_report["failure_robot_state"] = {
            "stage": stage,
            "joint_names": [str(name) for name in joint_trajectory.joint_names],
            "joint_positions": [
                float(position) for position in last_point.positions
            ],
            "trajectory_point_index": len(joint_trajectory.points) - 1,
        }

    def _finish_plan_report(self, success: bool) -> None:
        started = self.last_plan_report.pop(
            "_started_monotonic", time.monotonic()
        )
        self.last_plan_report["success"] = bool(success)
        self.last_plan_report["duration_sec"] = round(
            time.monotonic() - started, 6
        )
        records = [
            record
            for record in self._trajectory_range_records
            if (
                record["pregrasp"]
                and record["stage"]
                not in self.last_plan_report.get(
                    "discarded_trajectory_stages", []
                )
                and (record["accepted"] or not success)
            )
        ]
        joint_ranges = summarize_joint_trajectory_ranges(
            [record["trajectory"] for record in records],
            start_positions=self._plan_start_joint_positions,
        )
        self.last_plan_report["joint_ranges"] = joint_ranges
        self.last_plan_report["joint_range_stages"] = [
            {
                "stage": record["stage"],
                "accepted": record["accepted"],
                "pregrasp": record["pregrasp"],
                "point_count": record["point_count"],
            }
            for record in records
        ]
        if joint_ranges:
            heading = (
                "Pre-grasp까지의 관절 범위 (단위: deg)"
                if success
                else "실패한 Plan의 Pre-grasp까지 관절 범위 "
                "(실패 지점이 Pre-grasp이면 부분 궤적 포함, 단위: deg)"
            )
            self.get_logger().info(
                format_joint_trajectory_ranges(joint_ranges, heading)
            )
        else:
            self.get_logger().info(
                "계획 궤적 관절 범위: 표시할 trajectory point가 없습니다."
            )

    def _lookup_transform(self, child_frame: str, parent_frame: str | None = None):
        target_frame = self.base_frame if parent_frame is None else parent_frame
        timeout = max(0.1, float(self.get_parameter("service_timeout_sec").value))
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < deadline:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame, child_frame, Time()
                )
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().error(
            f"TF not found within {timeout:.1f}s: {target_frame} -> {child_frame}"
        )
        return None

    @staticmethod
    def _translation(transform) -> np.ndarray:
        value = transform.transform.translation
        return np.array([value.x, value.y, value.z], dtype=float)

    @staticmethod
    def _rotation_matrix(transform) -> np.ndarray:
        q = transform.transform.rotation
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=float,
        )

    def _apply_ompl_joint_path_constraints(
        self,
        request,
        pipeline: str,
        start_state: RobotState | None,
    ) -> bool:
        if pipeline.strip().lower() != "ompl":
            return True
        return self._apply_centered_joint_path_constraints(
            request,
            start_state,
            "OMPL",
        )

    def _apply_centered_joint_path_constraints(
        self,
        request,
        start_state: RobotState | None,
        context: str,
    ) -> bool:
        """Apply the configured start-centered limits to any planner request."""
        joint_names = [
            str(name)
            for name in self.get_parameter("ompl_limited_joint_names").value
        ]
        if start_state is not None and start_state.joint_state.name:
            start_positions = dict(
                zip(
                    start_state.joint_state.name,
                    start_state.joint_state.position,
                )
            )
        else:
            start_positions = dict(self._plan_start_joint_positions)
            if not start_positions:
                start_positions = self._wait_for_current_joint_positions()
        missing_names = [
            name for name in joint_names if name not in start_positions
        ]
        if missing_names:
            self.get_logger().error(
                f"{context} 시작 자세 중심 constraint를 만들 수 없습니다. "
                f"관절 상태 누락: {', '.join(missing_names)}"
            )
            return False

        tolerance_deg = float(
            self.get_parameter("ompl_joint_tolerance_deg").value
        )
        constrained_positions = {
            name: float(start_positions[name]) for name in joint_names
        }
        request.path_constraints = make_centered_joint_path_constraints(
            constrained_positions,
            math.radians(tolerance_deg),
        )
        centers = ", ".join(
            f"{name}={math.degrees(position):.1f}°"
            for name, position in constrained_positions.items()
        )
        self.get_logger().info(
            f"{context} 시작 자세 중심 constraint ±{tolerance_deg:.1f}°: "
            f"{centers}; wrist3=제외"
        )
        return True

    @staticmethod
    def _state_positions(state: RobotState | None) -> dict[str, float]:
        if state is None:
            return {}
        return {
            str(name): float(position)
            for name, position in zip(
                state.joint_state.name,
                state.joint_state.position,
            )
        }

    def _trajectory_safety_violations(
        self,
        trajectories,
        start_state: RobotState | None = None,
        start_positions: dict[str, float] | None = None,
    ) -> list[dict]:
        positions = dict(start_positions or {})
        if not positions:
            positions = self._state_positions(start_state)
        if not positions:
            positions = dict(self._plan_start_joint_positions)
        return trajectory_joint_safety_violations(
            trajectories,
            positions,
            self.get_parameter("ompl_limited_joint_names").value,
            float(
                self.get_parameter("trajectory_safety_max_span_deg").value
            ),
            float(
                self.get_parameter(
                    "trajectory_safety_wrist3_max_span_deg"
                ).value
            ),
            float(
                self.get_parameter("trajectory_safety_max_step_deg").value
            ),
        )

    def _log_trajectory_safety_failure(
        self,
        label: str,
        violations: list[dict],
    ) -> None:
        details = ", ".join(
            (
                f"{item['joint_name']} span="
                f"{item.get('span_deg', float('nan')):.1f}°/"
                f"{item.get('maximum_span_deg', float('nan')):.1f}°, "
                f"step={item.get('max_step_deg', float('nan')):.1f}°/"
                f"{item.get('maximum_step_deg', float('nan')):.1f}°"
            )
            for item in violations
        )
        self.get_logger().error(
            f"{label} 궤적 폐기: 관절 안전 한도 위반 ({details})"
        )

    def _plan_pick_ready(
        self,
        start_state: RobotState | None = None,
        label: str = "PICK_READY",
    ):
        joint_names = [
            str(name) for name in self.get_parameter("pick_ready_joint_names").value
        ]
        joint_positions = [
            float(position)
            for position in self.get_parameter("pick_ready_joint_positions").value
        ]
        ready_state_name = str(
            self.get_parameter("pick_ready_state_name").value
        )
        if len(joint_names) != 6 or len(joint_names) != len(joint_positions):
            self.get_logger().error(
                f"{ready_state_name} must contain exactly six joint values"
            )
            pipeline = str(
                self.get_parameter("joint_planning_pipeline_id").value
            )
            stage = (
                f"{pipeline.upper()}_PICK_READY"
                if label == "PICK_READY"
                else f"{pipeline.upper()}_RETURN_PICK_READY"
            )
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                0.0,
                "INVALID_PICK_READY_CONFIGURATION",
            )
            return None

        self.get_logger().info(
            f"{label} planning target uses SRDF state {ready_state_name}"
        )

        return self._plan_joint_target(
            joint_names,
            joint_positions,
            start_state=start_state,
            label=label,
        )

    def plan_and_execute_named_state(self) -> bool:
        """Move from the live robot state to the selected SRDF group state."""
        self._begin_plan_report()
        state_name = str(self.get_parameter("pick_ready_state_name").value)
        planned = self._plan_pick_ready(label="PICK_READY")
        if planned is None:
            self._finish_plan_report(False)
            return False

        trajectory, display_start_state = planned
        if bool(self.get_parameter("publish_display_trajectory").value):
            display = DisplayTrajectory()
            display.model_id = self.robot_model_id
            display.trajectory_start = self._complete_display_start_state(
                display_start_state
            )
            display.trajectory.append(trajectory)
            self.display_publisher.publish(display)

        execution_started = time.monotonic()
        success = self._execute_trajectory_group(
            trajectory,
            f"SRDF named pose {state_name}",
        )
        execution_duration = time.monotonic() - execution_started
        self.last_plan_report["execution_requested"] = True
        self.last_plan_report["execution_attempted"] = True
        self.last_plan_report["execution_success"] = bool(success)
        self.last_plan_report["execution_duration_sec"] = round(
            execution_duration,
            6,
        )
        if not success:
            self._record_plan_stage(
                "EXECUTION_NAMED_POSE",
                "execution",
                False,
                execution_duration,
                "TRAJECTORY_EXECUTION_FAILED",
            )
        self._finish_plan_report(success)
        return success

    def _plan_joint_target(
        self,
        joint_names,
        joint_positions,
        start_state: RobotState | None,
        label: str,
        pipeline_id: str | None = None,
        planner_id: str | None = None,
        stage_name: str | None = None,
        reference_trajectory=None,
    ):
        stage_started = time.monotonic()
        pipeline = pipeline_id or str(
            self.get_parameter("joint_planning_pipeline_id").value
        )
        selected_planner_id = planner_id or str(
            self.get_parameter("joint_planner_id").value
        )
        stage = stage_name or (
            f"{pipeline.upper()}_PICK_READY"
            if label == "PICK_READY"
            else f"{pipeline.upper()}_RETURN_PICK_READY"
        )
        timeout = max(1.0, float(self.get_parameter("service_timeout_sec").value))
        if not self.move_group_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("MoveIt move_action server is unavailable")
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "ACTION_SERVER_UNAVAILABLE",
            )
            return None
        if not joint_names or len(joint_names) != len(joint_positions):
            self.get_logger().error(f"{label} joint target is invalid")
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "INVALID_JOINT_TARGET",
            )
            return None

        tolerance = max(
            0.0001, float(self.get_parameter("pick_ready_joint_tolerance").value)
        )
        constraints = Constraints()
        constraints.name = label
        for name, position in zip(joint_names, joint_positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.pipeline_id = pipeline
        goal.request.planner_id = selected_planner_id
        goal.request.num_planning_attempts = int(
            self.get_parameter("pick_ready_planning_attempts").value
        )
        goal.request.allowed_planning_time = float(
            self.get_parameter("pick_ready_planning_time").value
        )
        goal.request.max_velocity_scaling_factor = float(
            self.get_parameter("pick_ready_velocity_scale").value
        )
        goal.request.max_acceleration_scaling_factor = float(
            self.get_parameter("pick_ready_acceleration_scale").value
        )
        if start_state is None:
            goal.request.start_state.is_diff = True
        else:
            goal.request.start_state = start_state
        goal.request.goal_constraints.append(constraints)
        if not self._apply_ompl_joint_path_constraints(
            goal.request,
            pipeline,
            start_state,
        ):
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "OMPL_CONSTRAINT_START_STATE_MISSING",
            )
            return None
        if reference_trajectory is not None:
            reference = GenericTrajectory()
            reference.header.frame_id = self.base_frame
            reference.joint_trajectory.append(
                reference_trajectory.joint_trajectory
            )
            goal.request.reference_trajectories.append(reference)
        goal.planning_options.plan_only = True
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        goal_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=timeout)
        goal_handle = goal_future.result() if goal_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"MoveIt rejected the {label} planning request")
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "GOAL_REJECTED_OR_RESPONSE_TIMEOUT",
            )
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        wrapped_result = result_future.result() if result_future.done() else None
        if wrapped_result is None:
            self.get_logger().error(f"{label} planning timed out")
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "PLANNING_RESULT_TIMEOUT",
            )
            return None
        result = wrapped_result.result
        trajectory = result.planned_trajectory
        point_count = len(trajectory.joint_trajectory.points)
        moveit_success = (
            result.error_code.val == MoveItErrorCodes.SUCCESS and point_count > 0
        )
        safety_violations = (
            self._trajectory_safety_violations(
                trajectory,
                start_state=result.trajectory_start,
            )
            if moveit_success
            else []
        )
        success = moveit_success and not safety_violations
        self._record_trajectory_range_input(
            stage,
            trajectory,
            success,
            pregrasp=(
                label == "PICK_READY"
                or stage.endswith("_PREAPPROACH")
            ),
        )
        if safety_violations:
            self._log_trajectory_safety_failure(label, safety_violations)
        self.get_logger().info(
            f"{label} plan success={success} error_code={result.error_code.val} "
            f"points={point_count} planning_time={result.planning_time:.3f}s"
        )
        if not success:
            self._record_failure_robot_state(stage, trajectory)
            reason = (
                "JOINT_SAFETY_LIMIT_EXCEEDED"
                if safety_violations
                else "MOVEIT_PLANNING_FAILED"
            )
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                reason,
                moveit_error_code=int(result.error_code.val),
                point_count=point_count,
                planning_time_sec=float(result.planning_time),
                trajectory_safety_violations=safety_violations,
            )
            return None
        self._record_plan_stage(
            stage,
            pipeline,
            True,
            time.monotonic() - stage_started,
            moveit_error_code=int(result.error_code.val),
            point_count=point_count,
            planning_time_sec=float(result.planning_time),
        )
        return trajectory, result.trajectory_start

    def _plan_chomp_pose_target(
        self,
        planning_target_pose: Pose,
        start_state: RobotState,
    ):
        pipeline = str(self.get_parameter("planning_pipeline_id").value)
        planner_id = str(self.get_parameter("planner_id").value)
        seed_trajectory = self._plan_pose_target(
            planning_target_pose,
            start_state,
            "CHOMP pre-approach endpoint seed",
            pipeline_id="pilz_industrial_motion_planner",
            planner_id="LIN",
            stage_name="CHOMP_PREAPPROACH_SEED",
            report_trajectory=False,
        )
        if seed_trajectory is None:
            return None
        seed_end = self._trajectory_end_state(seed_trajectory)
        result = self._plan_joint_target(
            list(seed_end.joint_state.name),
            list(seed_end.joint_state.position),
            start_state=start_state,
            label="TCP planned pre-approach",
            pipeline_id=pipeline,
            planner_id=planner_id,
            stage_name=f"{pipeline.upper()}_PREAPPROACH",
            reference_trajectory=seed_trajectory,
        )
        return result[0] if result is not None else None

    def _plan_pose_target(
        self,
        target_pose: Pose,
        start_state: RobotState,
        label: str,
        pipeline_id: str | None = None,
        planner_id: str | None = None,
        stage_name: str | None = None,
        report_trajectory: bool = True,
        pregrasp: bool = True,
    ):
        stage_started = time.monotonic()
        pipeline = pipeline_id or str(
            self.get_parameter("planning_pipeline_id").value
        )
        selected_planner_id = planner_id or str(
            self.get_parameter("planner_id").value
        )
        stage = stage_name or f"{pipeline.upper()}_PREAPPROACH"
        timeout = max(1.0, float(self.get_parameter("service_timeout_sec").value))
        if not self.move_group_client.wait_for_server(timeout_sec=timeout):
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "ACTION_SERVER_UNAVAILABLE",
            )
            return None

        position_tolerance = max(
            0.0001,
            float(
                self.get_parameter("preapproach_position_tolerance").value
            ),
        )
        orientation_tolerance = max(
            0.0001,
            float(
                self.get_parameter("preapproach_orientation_tolerance").value
            ),
        )

        position_region = SolidPrimitive()
        position_region.type = SolidPrimitive.SPHERE
        position_region.dimensions = [position_tolerance]
        region_pose = Pose()
        region_pose.position = target_pose.position
        region_pose.orientation.w = 1.0

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(position_region)
        bounding_volume.primitive_poses.append(region_pose)

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.planning_link
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.planning_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = label
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.pipeline_id = pipeline
        goal.request.planner_id = selected_planner_id
        if pipeline.strip().lower() == "ompl":
            goal.request.num_planning_attempts = max(
                1,
                int(
                    self.get_parameter(
                        "ompl_pose_planning_attempts"
                    ).value
                ),
            )
            goal.request.allowed_planning_time = max(
                0.1,
                float(
                    self.get_parameter(
                        "ompl_pose_planning_time"
                    ).value
                ),
            )
        else:
            goal.request.num_planning_attempts = int(
                self.get_parameter("pick_ready_planning_attempts").value
            )
            goal.request.allowed_planning_time = float(
                self.get_parameter("pick_ready_planning_time").value
            )
        goal.request.max_velocity_scaling_factor = float(
            self.get_parameter("pick_ready_velocity_scale").value
        )
        goal.request.max_acceleration_scaling_factor = float(
            self.get_parameter("pick_ready_acceleration_scale").value
        )
        goal.request.start_state = start_state
        goal.request.goal_constraints.append(constraints)
        if not self._apply_ompl_joint_path_constraints(
            goal.request,
            pipeline,
            start_state,
        ):
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "OMPL_CONSTRAINT_START_STATE_MISSING",
            )
            return None
        goal.planning_options.plan_only = True
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        goal_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=timeout)
        goal_handle = goal_future.result() if goal_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "GOAL_REJECTED_OR_RESPONSE_TIMEOUT",
            )
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        wrapped_result = result_future.result() if result_future.done() else None
        if wrapped_result is None:
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "PLANNING_RESULT_TIMEOUT",
            )
            return None

        result = wrapped_result.result
        trajectory = result.planned_trajectory
        point_count = len(trajectory.joint_trajectory.points)
        moveit_success = (
            result.error_code.val == MoveItErrorCodes.SUCCESS
            and point_count > 0
        )
        safety_violations = (
            self._trajectory_safety_violations(
                trajectory,
                start_state=start_state,
            )
            if moveit_success
            else []
        )
        success = moveit_success and not safety_violations
        if report_trajectory:
            self._record_trajectory_range_input(
                stage,
                trajectory,
                success,
                pregrasp=pregrasp,
            )
        self.get_logger().info(
            f"{label} plan success={success} pipeline={pipeline} "
            f"planner={selected_planner_id} error_code={result.error_code.val} "
            f"points={point_count} planning_time={result.planning_time:.3f}s"
        )
        if safety_violations:
            self._log_trajectory_safety_failure(label, safety_violations)
        reason = (
            ""
            if success
            else (
                "JOINT_SAFETY_LIMIT_EXCEEDED"
                if safety_violations
                else "MOVEIT_PLANNING_FAILED"
            )
        )
        if not success:
            self._record_failure_robot_state(stage, trajectory)
        self._record_plan_stage(
            stage,
            pipeline,
            success,
            time.monotonic() - stage_started,
            reason,
            moveit_error_code=int(result.error_code.val),
            point_count=point_count,
            planning_time_sec=float(result.planning_time),
            planner_id=selected_planner_id,
            trajectory_safety_violations=safety_violations,
        )
        return trajectory if success else None

    @staticmethod
    def _trajectory_end_state(planned_trajectory) -> RobotState:
        if isinstance(planned_trajectory, (list, tuple)):
            if not planned_trajectory:
                raise ValueError("trajectory sequence is empty")
            planned_trajectory = planned_trajectory[-1]
        state = RobotState()
        state.is_diff = False
        trajectory = planned_trajectory.joint_trajectory
        state.joint_state.name = list(trajectory.joint_names)
        state.joint_state.position = list(trajectory.points[-1].positions)
        return state

    def _plan_cartesian(
        self,
        waypoints,
        start_state: RobotState,
        label: str,
        terminal_failure: bool = True,
    ):
        stage_started = time.monotonic()
        stage_names = {
            "TCP direct pre-approach": "CARTESIAN_PREAPPROACH",
            "Continuous arc pre-approach": "CARTESIAN_CONTINUOUS_ARC",
            "Approach and pre-wait harvest": "CARTESIAN_APPROACH",
            "Post-wait harvest": "CARTESIAN_POST_WAIT",
            "Fallback return PICK_READY": "CARTESIAN_RETURN_PICK_READY",
        }
        stage = stage_names.get(label, f"CARTESIAN_{label.upper()}")
        timeout = max(0.1, float(self.get_parameter("service_timeout_sec").value))
        if not self.cartesian_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("MoveIt compute_cartesian_path service is unavailable")
            self._record_plan_stage(
                stage,
                "cartesian",
                False,
                time.monotonic() - stage_started,
                "CARTESIAN_SERVICE_UNAVAILABLE",
                terminal_failure=terminal_failure,
            )
            return None

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.start_state = start_state
        request.group_name = self.group_name
        request.link_name = self.planning_link
        request.waypoints = list(waypoints)
        request.max_step = max(0.0001, float(self.get_parameter("max_step").value))
        request.jump_threshold = max(
            0.0, float(self.get_parameter("jump_threshold").value)
        )
        request.revolute_jump_threshold = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        "cartesian_revolute_jump_threshold_deg"
                    ).value
                ),
            )
        )
        request.prismatic_jump_threshold = max(
            0.0,
            float(
                self.get_parameter(
                    "cartesian_prismatic_jump_threshold_m"
                ).value
            ),
        )
        request.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)
        if not self._apply_centered_joint_path_constraints(
            request,
            start_state,
            "Cartesian",
        ):
            self._record_plan_stage(
                stage,
                "cartesian",
                False,
                time.monotonic() - stage_started,
                "CARTESIAN_CONSTRAINT_START_STATE_MISSING",
                terminal_failure=terminal_failure,
            )
            return None

        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        response = future.result() if future.done() else None
        if response is None:
            self.get_logger().error(f"{label} Cartesian planning timed out")
            self._record_plan_stage(
                stage,
                "cartesian",
                False,
                time.monotonic() - stage_started,
                "CARTESIAN_SERVICE_TIMEOUT",
                terminal_failure=terminal_failure,
            )
            return None

        point_count = len(response.solution.joint_trajectory.points)
        minimum_fraction = float(self.get_parameter("minimum_fraction").value)
        moveit_success = (
            response.error_code.val == MoveItErrorCodes.SUCCESS
            and response.fraction >= minimum_fraction
            and point_count > 0
        )
        safety_violations = (
            self._trajectory_safety_violations(
                response.solution,
                start_state=start_state,
            )
            if moveit_success
            else []
        )
        success = moveit_success and not safety_violations
        self._record_trajectory_range_input(
            stage,
            response.solution,
            success,
            pregrasp=(
                stage
                in {
                    "CARTESIAN_PREAPPROACH",
                    "CARTESIAN_CONTINUOUS_ARC",
                }
            ),
        )
        self.get_logger().info(
            f"{label} Cartesian success={success} "
            f"fraction={response.fraction:.3f}/{minimum_fraction:.3f} "
            f"points={point_count} error_code={response.error_code.val}"
        )
        if safety_violations:
            self._log_trajectory_safety_failure(label, safety_violations)
        reason = ""
        if not success:
            if safety_violations:
                reason = "JOINT_SAFETY_LIMIT_EXCEEDED"
            elif response.fraction < minimum_fraction:
                reason = "CARTESIAN_FRACTION_LOW"
            elif response.error_code.val != MoveItErrorCodes.SUCCESS:
                reason = "MOVEIT_CARTESIAN_FAILED"
            else:
                reason = "EMPTY_CARTESIAN_TRAJECTORY"
            if terminal_failure:
                self._record_failure_robot_state(stage, response.solution)
        self._record_plan_stage(
            stage,
            "cartesian",
            success,
            time.monotonic() - stage_started,
            reason,
            terminal_failure=terminal_failure,
            moveit_error_code=int(response.error_code.val),
            cartesian_fraction=float(response.fraction),
            required_fraction=minimum_fraction,
            point_count=point_count,
            trajectory_safety_violations=safety_violations,
            fallback_planner=(
                "ompl" if not success and not terminal_failure else ""
            ),
        )
        return response.solution if success else None

    def _plan_cartesian_with_ompl_fallback(
        self,
        waypoints,
        start_state: RobotState,
        label: str,
        pregrasp: bool,
        maximum_joint_span_deg: float | None = None,
    ):
        """Use Cartesian first, then constrained OMPL for each failed waypoint."""
        waypoint_list = list(waypoints)
        cartesian = self._plan_cartesian(
            waypoint_list,
            start_state,
            label,
            terminal_failure=False,
        )
        if cartesian is not None:
            violations = (
                joint_span_violations(
                    [cartesian],
                    maximum_joint_span_deg,
                )
                if maximum_joint_span_deg is not None
                else []
            )
            if not violations:
                return (cartesian,)
            if self._trajectory_range_records:
                self._trajectory_range_records[-1]["accepted"] = False
            violation_text = ", ".join(
                f"{item['joint_name']}={item['span_deg']:.1f}°"
                for item in violations
            )
            self._record_plan_stage(
                "CONTINUOUS_ARC_JOINT_SPAN",
                "trajectory_validation",
                False,
                0.0,
                "JOINT_SPAN_LIMIT_EXCEEDED",
                maximum_joint_span_deg=float(maximum_joint_span_deg),
                violations=[
                    {
                        "joint_name": item["joint_name"],
                        "span_deg": float(item["span_deg"]),
                    }
                    for item in violations
                ],
            )
            self.get_logger().error(
                f"{label} 결과 폐기: 관절 span 안전 한도 "
                f"{float(maximum_joint_span_deg):.1f}° 초과 "
                f"({violation_text})"
            )
        recorded_stages = self.last_plan_report.get("stages", [])
        cartesian_failure = (
            dict(recorded_stages[-1]) if recorded_stages else {}
        )

        stage_names = {
            "TCP direct pre-approach": "PREAPPROACH",
            "Continuous arc pre-approach": "CONTINUOUS_ARC",
            "Approach and pre-wait harvest": "APPROACH",
            "Post-wait harvest": "POST_WAIT",
        }
        fallback_stage = stage_names.get(
            label,
            label.upper().replace(" ", "_").replace("-", "_"),
        )
        tolerance_deg = float(
            self.get_parameter("ompl_joint_tolerance_deg").value
        )
        self.get_logger().warning(
            f"{label} Cartesian 실패: 동일 waypoint를 시작 자세 기준 "
            f"±{tolerance_deg:.1f}° joint constraint가 적용된 "
            "OMPL로 재계획합니다."
        )

        trajectories = []
        waypoint_start = start_state
        planner_id = str(self.get_parameter("joint_planner_id").value)
        for index, waypoint in enumerate(waypoint_list, start=1):
            waypoint_label = (
                f"{label} OMPL fallback "
                f"({index}/{len(waypoint_list)})"
            )
            trajectory = self._plan_pose_target(
                waypoint,
                waypoint_start,
                waypoint_label,
                pipeline_id="ompl",
                planner_id=planner_id,
                stage_name=(
                    f"OMPL_FALLBACK_{fallback_stage}_{index}"
                ),
                pregrasp=pregrasp,
            )
            if trajectory is None:
                self.get_logger().error(
                    f"{label} OMPL fallback 실패: "
                    f"waypoint {index}/{len(waypoint_list)}"
                )
                return None
            trajectories.append(trajectory)
            waypoint_start = self._trajectory_end_state(trajectory)

        fallback_record = {
                "segment": fallback_stage,
                "cartesian_stage": cartesian_failure.get("stage", ""),
                "cartesian_reason": cartesian_failure.get("reason", ""),
                "cartesian_fraction": cartesian_failure.get(
                    "cartesian_fraction", ""
                ),
                "required_fraction": cartesian_failure.get(
                    "required_fraction", ""
                ),
                "waypoint_count": len(waypoint_list),
                "planner": f"ompl/{planner_id}",
                "ompl_stages": [
                    f"OMPL_FALLBACK_{fallback_stage}_{index}"
                    for index in range(1, len(waypoint_list) + 1)
                ],
                "success": True,
            }
        self.last_plan_report.setdefault("cartesian_fallbacks", []).append(
            fallback_record
        )
        self.get_logger().info(
            f"{label} OMPL fallback 성공: "
            f"{len(trajectories)}개 waypoint"
        )
        violations = (
            joint_span_violations(
                trajectories,
                maximum_joint_span_deg,
            )
            if maximum_joint_span_deg is not None
            else []
        )
        if violations:
            fallback_record.update(
                {
                    "success": False,
                    "reason": "JOINT_SPAN_LIMIT_EXCEEDED",
                }
            )
            violation_text = ", ".join(
                f"{item['joint_name']}={item['span_deg']:.1f}°"
                for item in violations
            )
            self._record_plan_stage(
                "CONTINUOUS_ARC_OMPL_JOINT_SPAN",
                "trajectory_validation",
                False,
                0.0,
                "JOINT_SPAN_LIMIT_EXCEEDED",
                maximum_joint_span_deg=float(maximum_joint_span_deg),
                violations=[
                    {
                        "joint_name": item["joint_name"],
                        "span_deg": float(item["span_deg"]),
                    }
                    for item in violations
                ],
            )
            self.get_logger().error(
                f"{label} OMPL fallback 결과 폐기: 관절 span 안전 한도 "
                f"{float(maximum_joint_span_deg):.1f}° 초과 "
                f"({violation_text})"
            )
            return None
        return tuple(trajectories)

    def _plan_preapproach(
        self,
        preapproach_pose: Pose,
        pick_ready_end: RobotState,
    ):
        mode = str(self.get_parameter("preapproach_mode").value)
        if mode == "cartesian":
            return self._plan_cartesian_with_ompl_fallback(
                [preapproach_pose],
                pick_ready_end,
                "TCP direct pre-approach",
                pregrasp=True,
            )
        if mode == "planner":
            pipeline = str(self.get_parameter("planning_pipeline_id").value)
            if pipeline == "chomp":
                trajectory = self._plan_chomp_pose_target(
                    preapproach_pose,
                    pick_ready_end,
                )
            else:
                trajectory = self._plan_pose_target(
                    preapproach_pose,
                    pick_ready_end,
                    "TCP planned pre-approach",
                )
            return (trajectory,) if trajectory is not None else None
        self.get_logger().error(f"Unsupported preapproach_mode: {mode}")
        self._record_plan_stage(
            "PREAPPROACH_CONFIGURATION",
            "internal",
            False,
            0.0,
            "UNSUPPORTED_PREAPPROACH_MODE",
            preapproach_mode=mode,
        )
        return None

    def _pick_ready_robot_state(self) -> RobotState:
        state = RobotState()
        state.is_diff = False
        state.joint_state.name = [
            str(name)
            for name in self.get_parameter("pick_ready_joint_names").value
        ]
        state.joint_state.position = [
            float(position)
            for position in self.get_parameter(
                "pick_ready_joint_positions"
            ).value
        ]
        if len(state.joint_state.name) != 6 or len(
            state.joint_state.name
        ) != len(state.joint_state.position):
            raise ValueError("PICK_READY must contain exactly six joint values")
        return state

    def _evaluate_preapproach_ik(
        self,
        pose: Pose,
        seed_state: RobotState,
        constraints: Constraints,
        rotation_deg: float,
    ) -> ApproachRotationEvaluation:
        timeout_sec = max(
            0.01,
            float(self.get_parameter("adaptive_grasp_ik_timeout_sec").value),
        )
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.robot_state = copy.deepcopy(seed_state)
        request.ik_request.constraints = copy.deepcopy(constraints)
        request.ik_request.avoid_collisions = bool(
            self.get_parameter("avoid_collisions").value
        )
        request.ik_request.ik_link_name = self.planning_link
        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = self.base_frame
        stamped_pose.header.stamp = self.get_clock().now().to_msg()
        stamped_pose.pose = copy.deepcopy(pose)
        request.ik_request.pose_stamped = stamped_pose
        request.ik_request.timeout = Duration(seconds=timeout_sec).to_msg()

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_sec + 0.10,
        )
        response = future.result() if future.done() else None
        if response is None:
            if not future.done():
                future.cancel()
            return ApproachRotationEvaluation(
                rotation_deg=float(rotation_deg),
                feasible=False,
                moveit_error_code=MoveItErrorCodes.TIMED_OUT,
            )

        error_code = int(response.error_code.val)
        if error_code != MoveItErrorCodes.SUCCESS:
            return ApproachRotationEvaluation(
                rotation_deg=float(rotation_deg),
                feasible=False,
                moveit_error_code=error_code,
            )

        seed_positions = {
            str(name): float(position)
            for name, position in zip(
                seed_state.joint_state.name,
                seed_state.joint_state.position,
            )
        }
        solution_positions = {
            str(name): float(position)
            for name, position in zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        }
        shared_names = [
            name for name in seed_positions if name in solution_positions
        ]
        if not shared_names:
            return ApproachRotationEvaluation(
                rotation_deg=float(rotation_deg),
                feasible=False,
                moveit_error_code=MoveItErrorCodes.INVALID_ROBOT_STATE,
            )
        squared_distance = 0.0
        for name in shared_names:
            difference = math.atan2(
                math.sin(solution_positions[name] - seed_positions[name]),
                math.cos(solution_positions[name] - seed_positions[name]),
            )
            squared_distance += difference * difference
        return ApproachRotationEvaluation(
            rotation_deg=float(rotation_deg),
            feasible=True,
            joint_distance=math.sqrt(squared_distance),
            moveit_error_code=error_code,
        )

    def _select_minimum_ik_approach(
        self,
        tomato_position: np.ndarray,
        tomato_rotation: np.ndarray,
        stemward: np.ndarray,
        gripper_to_tip_rotation: np.ndarray,
        planning_to_tip_translation: np.ndarray,
        planning_to_tip_rotation: np.ndarray,
    ) -> tuple[AdaptiveApproachDirection | None, dict]:
        maximum = float(
            self.get_parameter("adaptive_grasp_max_rotation_deg").value
        )
        prefer_robot_direction = bool(
            self.get_parameter(
                "adaptive_grasp_prefer_robot_direction"
            ).value
        )
        deadband = float(
            self.get_parameter("adaptive_grasp_deadband_deg").value
        )
        geometric_preference = adaptive_outward_toward_robot(
            tomato_rotation=tomato_rotation,
            tomato_position=tomato_position,
            robot_position=[0.0, 0.0, 0.0],
            max_rotation_deg=maximum,
            deadband_deg=deadband,
        )
        deadline_guard = deadline_safe_rotation_guard(
            tomato_rotation=tomato_rotation,
            tomato_position=tomato_position,
            robot_position=[0.0, 0.0, 0.0],
            margin_deg=float(
                self.get_parameter(
                    "adaptive_grasp_deadline_margin_deg"
                ).value
            ),
        )
        deadline_margin_deg = float(
            self.get_parameter("adaptive_grasp_deadline_margin_deg").value
        )
        allowed_sector_deg = 180.0 - 2.0 * min(
            90.0,
            max(0.0, deadline_margin_deg),
        )
        sector_rejection_reason = (
            f"PREGRASP_OUTSIDE_{allowed_sector_deg:g}_DEG_SECTOR"
        )
        report = {
            "selection_mode": (
                "robot_direction_priority"
                if prefer_robot_direction
                else "minimum_ik_angle"
            ),
            "prefer_robot_direction": prefer_robot_direction,
            "geometric_preferred_rotation_deg": float(
                geometric_preference.applied_rotation_deg
            ),
            "fallback_used": False,
            "deadline_guard_active": bool(deadline_guard.active),
            "deadline_minimum_rotation_deg": float(
                deadline_guard.minimum_rotation_deg
            ),
            "deadline_nominal_robot_side_dot": float(
                deadline_guard.nominal_robot_side_dot
            ),
            "allowed_robot_side_sector_deg": float(allowed_sector_deg),
        }
        if (
            deadline_guard.active
            and abs(deadline_guard.minimum_rotation_deg) > maximum + 1e-9
        ):
            report.update(
                {
                    "selection_mode": "deadline_rejected",
                    "fallback_used": False,
                    "fallback_reason": "DEADLINE_REQUIRES_ANGLE_OVER_MAXIMUM",
                    "ik_evaluation_count": 0,
                }
            )
            return None, report

        def direction_for_rotation(
            rotation_deg: float,
        ) -> AdaptiveApproachDirection:
            outward = outward_from_tomato_rotation(
                tomato_rotation,
                rotation_deg,
            )
            selected_error = robot_side_approach_error_deg(
                outward,
                tomato_position,
                [0.0, 0.0, 0.0],
            )
            nominal_error = robot_side_approach_error_deg(
                outward_from_tomato_rotation(tomato_rotation, 0.0),
                tomato_position,
                [0.0, 0.0, 0.0],
            )
            return AdaptiveApproachDirection(
                outward_axis=outward,
                applied_rotation_deg=float(rotation_deg),
                current_robot_error_deg=float(nominal_error),
                selected_robot_error_deg=float(selected_error),
            )

        fallback_rotation = float(
            geometric_preference.applied_rotation_deg
        )
        if deadline_guard.active and (
            fallback_rotation * deadline_guard.minimum_rotation_deg <= 0.0
            or abs(fallback_rotation)
            < abs(deadline_guard.minimum_rotation_deg)
        ):
            fallback_rotation = float(
                deadline_guard.minimum_rotation_deg
            )
        safe_geometric_fallback = direction_for_rotation(fallback_rotation)
        if not is_within_robot_side_approach_sector(
            safe_geometric_fallback.outward_axis,
            tomato_position,
            [0.0, 0.0, 0.0],
            margin_deg=deadline_margin_deg,
        ):
            report.update(
                {
                    "selection_mode": "deadline_rejected",
                    "fallback_used": False,
                    "fallback_reason": sector_rejection_reason,
                    "ik_evaluation_count": 0,
                }
            )
            return None, report

        wait_sec = max(
            0.0,
            float(
                self.get_parameter("adaptive_grasp_ik_service_wait_sec").value
            ),
        )
        if not self.ik_client.wait_for_service(timeout_sec=wait_sec):
            report.update(
                {
                    "selection_mode": "geometric_fallback",
                    "fallback_used": True,
                    "fallback_reason": "COMPUTE_IK_SERVICE_UNAVAILABLE",
                    "ik_evaluation_count": 0,
                }
            )
            return safe_geometric_fallback, report

        ready_state = self._pick_ready_robot_state()
        ready_positions = dict(
            zip(
                ready_state.joint_state.name,
                ready_state.joint_state.position,
            )
        )
        limited_names = [
            str(name)
            for name in self.get_parameter("ompl_limited_joint_names").value
        ]
        tolerance = math.radians(
            float(self.get_parameter("ompl_joint_tolerance_deg").value)
        )
        constraints = make_centered_joint_path_constraints(
            {
                name: float(ready_positions[name])
                for name in limited_names
                if name in ready_positions
            },
            tolerance,
        )
        virtual_vine_origin = tomato_position + stemward

        def evaluate(rotation_deg: float) -> ApproachRotationEvaluation:
            outward = outward_from_tomato_rotation(
                tomato_rotation,
                rotation_deg,
            )
            # This is a hard candidate-domain restriction, not a post-plan
            # warning.  Out-of-sector poses never reach IK or OMPL.
            if not is_within_robot_side_approach_sector(
                outward,
                tomato_position,
                [0.0, 0.0, 0.0],
                margin_deg=deadline_margin_deg,
            ):
                return ApproachRotationEvaluation(
                    rotation_deg=float(rotation_deg),
                    feasible=False,
                    moveit_error_code=(
                        MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS
                    ),
                    rejection_reason=sector_rejection_reason,
                )
            geometry = make_harvest_geometry(
                tomato_position=tomato_position,
                vine_origin=virtual_vine_origin,
                vine_axis=[0.0, 0.0, 1.0],
                tip_standoff=float(self.get_parameter("tip_standoff").value),
                tip_below_center=float(
                    self.get_parameter("tip_below_center").value
                ),
                preapproach_clearance=float(
                    self.get_parameter("preapproach_clearance").value
                ),
                outward_hint=outward,
                tip_rotation_from_gripper=gripper_to_tip_rotation,
            )
            planning_pose = planning_pose_from_tip_pose(
                geometry.preapproach_pose,
                planning_to_tip_translation,
                planning_to_tip_rotation,
            )
            return self._evaluate_preapproach_ik(
                planning_pose,
                ready_state,
                constraints,
                rotation_deg,
            )

        preferred_rotation = float(
            geometric_preference.applied_rotation_deg
        )
        if math.isclose(preferred_rotation, 0.0, abs_tol=1e-9):
            no_deadband_preference = adaptive_outward_toward_robot(
                tomato_rotation=tomato_rotation,
                tomato_position=tomato_position,
                robot_position=[0.0, 0.0, 0.0],
                max_rotation_deg=maximum,
                deadband_deg=0.0,
            )
            preferred_rotation = float(
                no_deadband_preference.applied_rotation_deg
            )
        search_step = float(
            self.get_parameter("adaptive_grasp_search_step_deg").value
        )
        search_resolution = float(
            self.get_parameter(
                "adaptive_grasp_search_resolution_deg"
            ).value
        )
        minimum_rotation = (
            abs(deadline_guard.minimum_rotation_deg)
            if deadline_guard.active
            else 0.0
        )
        if prefer_robot_direction:
            desired_rotation = preferred_rotation
            if deadline_guard.active:
                deadline_sign = (
                    1.0
                    if deadline_guard.minimum_rotation_deg >= 0.0
                    else -1.0
                )
                if desired_rotation * deadline_sign <= 0.0:
                    desired_rotation = deadline_sign * maximum
                elif abs(desired_rotation) < minimum_rotation:
                    desired_rotation = deadline_sign * minimum_rotation
            selected, evaluations = select_robotward_feasible_rotation(
                evaluate,
                desired_rotation_deg=desired_rotation,
                max_rotation_deg=maximum,
                coarse_step_deg=search_step,
                resolution_deg=search_resolution,
                minimum_abs_rotation_deg=minimum_rotation,
            )
        else:
            selected, evaluations = select_minimum_feasible_rotation(
                evaluate,
                max_rotation_deg=maximum,
                coarse_step_deg=search_step,
                resolution_deg=search_resolution,
                preferred_sign=(
                    deadline_guard.minimum_rotation_deg
                    if deadline_guard.active
                    else preferred_rotation
                ),
                minimum_abs_rotation_deg=minimum_rotation,
                allow_opposite_sign=not deadline_guard.active,
            )
        report["ik_evaluation_count"] = len(evaluations)
        report["ik_evaluations"] = [
            {
                "rotation_deg": float(item.rotation_deg),
                "feasible": bool(item.feasible),
                "joint_distance_rad": (
                    float(item.joint_distance)
                    if math.isfinite(item.joint_distance)
                    else None
                ),
                "moveit_error_code": int(item.moveit_error_code),
                "rejection_reason": str(item.rejection_reason),
            }
            for item in evaluations
        ]
        if selected is None:
            report.update(
                {
                    "selection_mode": "geometric_fallback",
                    "fallback_used": True,
                    "fallback_reason": "NO_CONSTRAINT_VALID_IK_CANDIDATE",
                }
            )
            return safe_geometric_fallback, report

        report.update(
            {
                "selected_rotation_deg": float(selected.rotation_deg),
                "selected_joint_distance_rad": float(
                    selected.joint_distance
                ),
            }
        )
        return direction_for_rotation(selected.rotation_deg), report

    def _state_matches_pick_ready(self, state: RobotState | None) -> bool:
        if state is None:
            return False
        positions = {
            str(name): float(position)
            for name, position in zip(
                state.joint_state.name,
                state.joint_state.position,
            )
        }
        ready = self._pick_ready_robot_state()
        tolerance = max(
            math.radians(2.0),
            float(self.get_parameter("pick_ready_joint_tolerance").value),
        )
        return all(
            name in positions
            and revolute_position_error(positions[name], target) <= tolerance
            for name, target in zip(
                ready.joint_state.name,
                ready.joint_state.position,
            )
        )

    def _plan_continuous_preapproach(
        self,
        preapproach_pose: Pose,
        outward_axis,
        start_state_override: RobotState | None = None,
        start_pose_override: Pose | None = None,
        arc_failure_reverse_trajectory=(),
    ):
        """Plan an outward arc, unwinding the cached path if it fails."""
        stage_start = len(self.last_plan_report.get("stages", []))
        fallback_start = len(
            self.last_plan_report.get("cartesian_fallbacks", [])
        )
        range_start = len(self._trajectory_range_records)
        current_planning_tf = (
            None
            if start_pose_override is not None
            else self._lookup_transform(self.planning_link)
        )
        arc_plan = None
        if start_pose_override is not None:
            current_pose = copy.deepcopy(start_pose_override)
        elif current_planning_tf is not None:
            current_pose = Pose()
            current_pose.position.x = float(
                current_planning_tf.transform.translation.x
            )
            current_pose.position.y = float(
                current_planning_tf.transform.translation.y
            )
            current_pose.position.z = float(
                current_planning_tf.transform.translation.z
            )
            current_pose.orientation = current_planning_tf.transform.rotation
        else:
            current_pose = None

        if current_pose is not None:
            arc_waypoints = make_continuous_arc_waypoints(
                current_pose,
                preapproach_pose,
                outward_axis,
                minimum_clearance=float(
                    self.get_parameter("continuous_arc_min_clearance").value
                ),
                maximum_clearance=float(
                    self.get_parameter("continuous_arc_max_clearance").value
                ),
                waypoint_count=int(
                    self.get_parameter("continuous_arc_waypoint_count").value
                ),
            )
            if start_state_override is None:
                current_state = RobotState()
                current_state.is_diff = True
            else:
                current_state = copy.deepcopy(start_state_override)
            arc_plan = self._plan_cartesian_with_ompl_fallback(
                arc_waypoints,
                current_state,
                "Continuous arc pre-approach",
                pregrasp=True,
                maximum_joint_span_deg=float(
                    self.get_parameter(
                        "continuous_arc_max_joint_span_deg"
                    ).value
                ),
            )
            self.last_plan_report["continuous_arc"] = {
                "waypoint_count": len(arc_waypoints),
                "maximum_joint_span_deg": float(
                    self.get_parameter(
                        "continuous_arc_max_joint_span_deg"
                    ).value
                ),
                "minimum_clearance_m": float(
                    self.get_parameter("continuous_arc_min_clearance").value
                ),
                "maximum_clearance_m": float(
                    self.get_parameter("continuous_arc_max_clearance").value
                ),
                "outward_axis": [float(value) for value in outward_axis],
            }
        else:
            self._record_plan_stage(
                "CONTINUOUS_ARC_START_TF",
                "tf",
                False,
                0.0,
                "PLANNING_LINK_TF_NOT_FOUND",
            )

        if arc_plan is not None:
            if start_state_override is None:
                display_start_state = RobotState()
                display_start_state.is_diff = True
            else:
                display_start_state = copy.deepcopy(start_state_override)
            self.last_plan_report["continuous_transition_direct"] = True
            self.last_plan_report["continuous_transition_arc"] = True
            self.get_logger().info(
                "연속 수확 arc 전환 성공: 현재 post-wait 자세에서 식물 "
                "바깥쪽 반원 경로를 거쳐 다음 pre-grasp로 이동"
            )
            return (), (), arc_plan, display_start_state

        arc_stages = self.last_plan_report.get("stages", [])[stage_start:]
        for stage in arc_stages:
            stage.update({"discarded": True, "recovered": True})
        del self._trajectory_range_records[range_start:]
        fallbacks = self.last_plan_report.get("cartesian_fallbacks", [])
        arc_fallbacks = fallbacks[fallback_start:]
        if arc_fallbacks:
            self.last_plan_report["continuous_arc_fallbacks"] = list(
                arc_fallbacks
            )
            del fallbacks[fallback_start:]
        arc_failure = dict(arc_stages[-1]) if arc_stages else {}
        discarded_stages = self.last_plan_report.setdefault(
            "discarded_trajectory_stages", []
        )
        for stage in arc_stages:
            stage_name = str(stage.get("stage", ""))
            if stage_name and stage_name not in discarded_stages:
                discarded_stages.append(stage_name)
        for key in (
            "failure_stage",
            "failure_planner_type",
            "failure_reason",
            "moveit_error_code",
            "cartesian_fraction",
            "required_fraction",
        ):
            self.last_plan_report.pop(key, None)

        reverse_trajectories = tuple(arc_failure_reverse_trajectory or ())
        if not reverse_trajectories:
            self.get_logger().error(
                "연속 수확 arc 전환 실패: 안전하게 역재생할 이전 궤적이 "
                "없으므로 신규 OMPL PICK_READY 우회 경로를 만들지 않습니다."
            )
            self.last_plan_report.update(
                {
                    "continuous_transition_direct": False,
                    "continuous_transition_arc": False,
                    "failure_stage": "CONTINUOUS_ARC_PREAPPROACH",
                    "failure_planner_type": arc_failure.get(
                        "planner_type", "cartesian/ompl"
                    ),
                    "failure_reason": "ARC_FAILED_WITHOUT_REVERSE_HISTORY",
                    "recovery_used": False,
                    "recovery_success": False,
                }
            )
            return None

        self.get_logger().warning(
            "연속 수확 arc 전환 실패: 지금까지 실행한 연속 수확 궤적을 "
            "역순으로 따라 PICK_READY까지 안전 복귀한 뒤 다음 "
            "pre-grasp를 계획합니다."
        )
        for index, trajectory in enumerate(reverse_trajectories, start=1):
            self._record_trajectory_range_input(
                f"ARC_REVERSE_RECOVERY_{index}",
                trajectory,
                True,
                True,
            )
        pick_ready_end = self._trajectory_end_state(reverse_trajectories)
        fallback_preapproach = self._plan_preapproach(
            preapproach_pose,
            pick_ready_end,
        )
        if fallback_preapproach is None:
            return None
        self.last_plan_report.update(
            {
                "continuous_transition_direct": False,
                "continuous_transition_arc": False,
                "failure_stage": "CONTINUOUS_ARC_PREAPPROACH",
                "failure_planner_type": arc_failure.get(
                    "planner_type", "cartesian/ompl"
                ),
                "failure_reason": arc_failure.get(
                    "reason", "ARC_PLANNING_FAILED"
                ),
                "recovery_used": True,
                "recovery_success": True,
                "recovery_stage": "CACHED_TRAJECTORY_REVERSE_TO_PICK_READY",
                "recovery_reason": (
                    "arc 실패 후 기존 성공 궤적을 역재생하여 "
                    "PICK_READY 복귀 성공"
                ),
                "continuous_reverse_recovery": True,
                "reverse_recovery_trajectory_count": len(
                    reverse_trajectories
                ),
            }
        )
        if start_state_override is None:
            display_start_state = RobotState()
            display_start_state.is_diff = True
        else:
            display_start_state = copy.deepcopy(start_state_override)
        return (
            reverse_trajectories,
            (),
            fallback_preapproach,
            display_start_state,
        )

    def plan_continuous_transition_only(
        self,
        start_state: RobotState,
        start_pose: Pose,
        target_preapproach_pose: Pose,
        outward_axis,
    ):
        """Plan only the inter-tomato arc to a cached pre-grasp pose.

        The next tomato's READY-to-harvest motion is planned independently by
        the batch worker.  A failure here therefore rejects only the arc; the
        worker can still reverse its cached path to READY and reuse the
        already validated next-tomato trajectory.
        """
        self._begin_plan_report(start_state)
        if not self._synchronize_dynamic_base_transform():
            self._record_plan_stage(
                "DYNAMIC_BASE_TF_SYNC",
                "tf",
                False,
                0.0,
                "FRESH_LIFT_TF_NOT_RECEIVED",
            )
            self._finish_plan_report(False)
            return None

        waypoints = make_continuous_arc_waypoints(
            start_pose,
            target_preapproach_pose,
            outward_axis,
            minimum_clearance=float(
                self.get_parameter("continuous_arc_min_clearance").value
            ),
            maximum_clearance=float(
                self.get_parameter("continuous_arc_max_clearance").value
            ),
            waypoint_count=int(
                self.get_parameter("continuous_arc_waypoint_count").value
            ),
        )
        transition = self._plan_cartesian_with_ompl_fallback(
            waypoints,
            copy.deepcopy(start_state),
            "Continuous arc pre-approach",
            pregrasp=True,
            maximum_joint_span_deg=float(
                self.get_parameter(
                    "continuous_arc_max_joint_span_deg"
                ).value
            ),
        )
        self.last_plan_report["continuous_arc"] = {
            "waypoint_count": len(waypoints),
            "maximum_joint_span_deg": float(
                self.get_parameter(
                    "continuous_arc_max_joint_span_deg"
                ).value
            ),
            "minimum_clearance_m": float(
                self.get_parameter("continuous_arc_min_clearance").value
            ),
            "maximum_clearance_m": float(
                self.get_parameter("continuous_arc_max_clearance").value
            ),
            "outward_axis": [float(value) for value in outward_axis],
        }
        success = transition is not None
        self.last_plan_report["continuous_transition_direct"] = success
        self.last_plan_report["continuous_transition_arc"] = success
        self._finish_plan_report(success)
        if not success:
            self.get_logger().warning(
                "토마토 간 Arc 전환 계획 실패: 다음 토마토의 독립 계획은 "
                "유지하고 cached reverse 복구를 사용합니다."
            )
            return None
        return tuple(transition)

    def plan(
        self,
        start_state_override: RobotState | None = None,
        start_pose_override: Pose | None = None,
        arc_failure_reverse_trajectory=(),
    ):
        self._begin_plan_report(start_state_override)
        if not self._synchronize_dynamic_base_transform():
            self._record_plan_stage(
                "DYNAMIC_BASE_TF_SYNC",
                "tf",
                False,
                0.0,
                "FRESH_LIFT_TF_NOT_RECEIVED",
            )
            self._finish_plan_report(False)
            return None
        try:
            result = self._plan_impl(
                start_state_override=start_state_override,
                start_pose_override=start_pose_override,
                arc_failure_reverse_trajectory=(
                    arc_failure_reverse_trajectory
                ),
            )
        except Exception as error:
            self._record_plan_stage(
                "UNEXPECTED_EXCEPTION",
                "internal",
                False,
                0.0,
                repr(error),
            )
            self._finish_plan_report(False)
            raise
        self._finish_plan_report(result is not None)
        return result

    def _plan_impl(
        self,
        start_state_override: RobotState | None = None,
        start_pose_override: Pose | None = None,
        arc_failure_reverse_trajectory=(),
    ):
        tomato_tf = self._lookup_transform(self.tomato_frame)
        if tomato_tf is None:
            self._record_plan_stage(
                "TF_TARGET",
                "tf",
                False,
                0.0,
                "TF_NOT_FOUND",
            )
            return None
        tomato_position = self._translation(tomato_tf)
        tomato_rotation = self._rotation_matrix(tomato_tf)
        try:
            stemward, outward_hint = stemward_and_outward_from_tomato_rotation(
                tomato_rotation
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            self._record_plan_stage(
                "TARGET_GEOMETRY",
                "geometry",
                False,
                0.0,
                "INVALID_TOMATO_ORIENTATION",
            )
            return None

        tip_tf = self._lookup_transform(self.tip_link)
        gripper_to_tip_tf = self._lookup_transform(
            self.tip_link, parent_frame=self.gripper_link
        )
        planning_to_tip_tf = self._lookup_transform(
            self.tip_link, parent_frame=self.planning_link
        )
        if (
            tip_tf is None
            or gripper_to_tip_tf is None
            or planning_to_tip_tf is None
        ):
            self._record_plan_stage(
                "TF_ROBOT_TOOL",
                "tf",
                False,
                0.0,
                "TOOL_TF_NOT_FOUND",
            )
            return None

        adaptive_enabled = bool(
            self.get_parameter("adaptive_grasp_enabled").value
        )
        selection_report = {
            "selection_mode": "disabled",
            "fallback_used": False,
            "ik_evaluation_count": 0,
        }
        if adaptive_enabled:
            approach_direction, selection_report = (
                self._select_minimum_ik_approach(
                    tomato_position=tomato_position,
                    tomato_rotation=tomato_rotation,
                    stemward=stemward,
                    gripper_to_tip_rotation=self._rotation_matrix(
                        gripper_to_tip_tf
                    ),
                    planning_to_tip_translation=self._translation(
                        planning_to_tip_tf
                    ),
                    planning_to_tip_rotation=self._rotation_matrix(
                        planning_to_tip_tf
                    ),
                )
            )
            if approach_direction is None:
                self.last_plan_report["adaptive_grasp"] = {
                    "enabled": True,
                    **selection_report,
                }
                self.get_logger().error(
                    "접근각 선택 실패: deadline을 지키기 위한 최소 보정각이 "
                    "설정된 최대 보정각을 초과합니다."
                )
                self._record_plan_stage(
                    "TARGET_GEOMETRY",
                    "geometry",
                    False,
                    0.0,
                    "DEADLINE_REQUIRES_ANGLE_OVER_MAXIMUM",
                )
                return None
            outward_hint = approach_direction.outward_axis
        else:
            approach_direction = AdaptiveApproachDirection(
                outward_hint,
                0.0,
                0.0,
                0.0,
            )

        self.last_plan_report["adaptive_grasp"] = {
            "enabled": adaptive_enabled,
            "applied_rotation_deg": float(
                approach_direction.applied_rotation_deg
            ),
            "current_robot_error_deg": float(
                approach_direction.current_robot_error_deg
            ),
            "selected_robot_error_deg": float(
                approach_direction.selected_robot_error_deg
            ),
            "outward_axis": [
                float(value) for value in approach_direction.outward_axis
            ],
            "approach_axis_tomato_local": [
                float(value)
                for value in (
                    tomato_rotation.T
                    @ (-approach_direction.outward_axis)
                )
            ],
            **selection_report,
        }
        fallback_text = (
            f", fallback={selection_report.get('fallback_reason')}"
            if selection_report.get("fallback_used")
            else ""
        )
        self.get_logger().info(
            "접근각 선택: "
            f"mode={selection_report.get('selection_mode')} "
            f"rotation={approach_direction.applied_rotation_deg:+.1f}° "
            f"IK검사={selection_report.get('ik_evaluation_count', 0)}회 "
            f"robot_error={approach_direction.current_robot_error_deg:.1f}°"
            f"->{approach_direction.selected_robot_error_deg:.1f}°"
            f"{fallback_text}"
        )

        virtual_vine_origin = tomato_position + stemward
        current_tip_position = self._translation(tip_tf)
        geometry = make_harvest_geometry(
            tomato_position=tomato_position,
            vine_origin=virtual_vine_origin,
            vine_axis=[0.0, 0.0, 1.0],
            tip_standoff=float(self.get_parameter("tip_standoff").value),
            tip_below_center=float(self.get_parameter("tip_below_center").value),
            preapproach_clearance=float(
                self.get_parameter("preapproach_clearance").value
            ),
            outward_hint=outward_hint,
            tip_rotation_from_gripper=self._rotation_matrix(gripper_to_tip_tf),
        )
        recommend_rotation_deg = float(
            selection_report.get("geometric_preferred_rotation_deg", 0.0)
        )
        recommend_outward = outward_from_tomato_rotation(
            tomato_rotation,
            recommend_rotation_deg,
        )
        recommend_geometry = make_harvest_geometry(
            tomato_position=tomato_position,
            vine_origin=virtual_vine_origin,
            vine_axis=[0.0, 0.0, 1.0],
            tip_standoff=float(self.get_parameter("tip_standoff").value),
            tip_below_center=float(
                self.get_parameter("tip_below_center").value
            ),
            preapproach_clearance=float(
                self.get_parameter("preapproach_clearance").value
            ),
            outward_hint=recommend_outward,
            tip_rotation_from_gripper=self._rotation_matrix(
                gripper_to_tip_tf
            ),
        )

        preapproach_world = np.array(
            [
                geometry.preapproach_pose.position.x,
                geometry.preapproach_pose.position.y,
                geometry.preapproach_pose.position.z,
            ],
            dtype=float,
        )
        target_world = np.array(
            [
                geometry.target_pose.position.x,
                geometry.target_pose.position.y,
                geometry.target_pose.position.z,
            ],
            dtype=float,
        )
        self.last_plan_report["approach_geometry"] = {
            "frame_id": self.tomato_frame,
            "planning_frame_id": self.base_frame,
            "pregrasp_reference_link": self.tip_link,
            "tomato_xyz": [
                float(value) for value in geometry.tomato_position
            ],
            "vine_xyz": [
                float(value) for value in geometry.vine_point
            ],
            "recommend_pregrasp_xyz": [
                float(recommend_geometry.preapproach_pose.position.x),
                float(recommend_geometry.preapproach_pose.position.y),
                float(recommend_geometry.preapproach_pose.position.z),
            ],
            "final_pregrasp_xyz": [
                float(geometry.preapproach_pose.position.x),
                float(geometry.preapproach_pose.position.y),
                float(geometry.preapproach_pose.position.z),
            ],
            "preapproach_position": [
                float(value)
                for value in (
                    tomato_rotation.T
                    @ (preapproach_world - tomato_position)
                )
            ],
            "target_position": [
                float(value)
                for value in (
                    tomato_rotation.T @ (target_world - tomato_position)
                )
            ],
        }

        target = geometry.target_pose.position
        pre = geometry.preapproach_pose.position
        current_tip = current_tip_position
        order_dot = float(
            np.dot(
                geometry.tomato_position
                - np.array([target.x, target.y, target.z], dtype=float),
                geometry.vine_point - geometry.tomato_position,
            )
        )
        self.get_logger().info(
            f"Target={self.tomato_frame} current_tip={current_tip.round(4).tolist()} "
            f"stemward={stemward.round(4).tolist()} "
            f"outward={geometry.outward_axis.round(4).tolist()} "
            f"preapproach=({pre.x:.4f}, {pre.y:.4f}, {pre.z:.4f}) "
            f"target=({target.x:.4f}, {target.y:.4f}, {target.z:.4f}) "
            f"tip_below={float(self.get_parameter('tip_below_center').value):.4f}m "
            f"tip-tomato-stem_order={'OK' if order_dot > 0.0 else 'INVALID'}"
        )
        if order_dot <= 0.0:
            self.get_logger().error(
                "Tomato is not between the gripper tip and detected stem direction"
            )
            self._record_plan_stage(
                "TARGET_GEOMETRY",
                "geometry",
                False,
                0.0,
                "INVALID_GRIPPER_STEM_ORDER",
            )
            return None

        tip_motion = make_tip_local_harvest_motion(
            geometry.target_pose,
            x_forward=float(self.get_parameter("harvest_x_forward").value),
            first_z_lift=float(
                self.get_parameter("harvest_first_z_lift").value
            ),
            first_x_back=float(
                self.get_parameter("harvest_first_x_back").value
            ),
            second_z_lift=float(
                self.get_parameter("harvest_second_z_lift").value
            ),
            second_x_back=float(
                self.get_parameter("harvest_second_x_back").value
            ),
        )
        planning_to_tip_translation = self._translation(planning_to_tip_tf)
        planning_to_tip_rotation = self._rotation_matrix(planning_to_tip_tf)

        def as_planning_pose(tip_pose: Pose) -> Pose:
            return planning_pose_from_tip_pose(
                tip_pose,
                planning_to_tip_translation,
                planning_to_tip_rotation,
            )

        preapproach_planning_pose = as_planning_pose(geometry.preapproach_pose)
        stepwise_plan = bool(self.get_parameter("stepwise_plan").value)
        step_cycle_only = bool(self.get_parameter("step_cycle_only").value)
        cycle_last_stage = int(
            self.get_parameter("step_cycle_last_stage").value
        )
        if step_cycle_only and not 1 <= cycle_last_stage <= 5:
            self._record_plan_stage(
                "STEP_CYCLE_CONFIGURATION",
                "configuration",
                False,
                0.0,
                "INVALID_STEP_CYCLE_LAST_STAGE",
                requested_stage=cycle_last_stage,
            )
            return None
        continuous_transition = bool(
            self.get_parameter("continuous_transition").value
        )
        pick_ready_end = None
        arc_reverse_recovery_trajectory = ()
        if continuous_transition:
            continuous_plan = self._plan_continuous_preapproach(
                preapproach_planning_pose,
                geometry.outward_axis,
                start_state_override=start_state_override,
                start_pose_override=start_pose_override,
                arc_failure_reverse_trajectory=(
                    arc_failure_reverse_trajectory
                ),
            )
            if continuous_plan is None:
                return None
            (
                arc_reverse_recovery_trajectory,
                pick_ready_trajectory,
                preapproach_trajectory,
                display_start_state,
            ) = continuous_plan
        else:
            if self._state_matches_pick_ready(start_state_override):
                pick_ready_trajectory = ()
                display_start_state = copy.deepcopy(start_state_override)
                pick_ready_end = copy.deepcopy(start_state_override)
                self._record_plan_stage(
                    "PREPLANNED_PICK_READY_REUSE",
                    "cached_state",
                    True,
                    0.0,
                    "ALREADY_AT_SELECTED_READY_STATE",
                )
            else:
                pick_ready_plan = self._plan_pick_ready(
                    start_state_override
                )
                if pick_ready_plan is None:
                    return None
                pick_ready_trajectory, display_start_state = pick_ready_plan
                pick_ready_end = self._trajectory_end_state(
                    pick_ready_trajectory
                )
            if step_cycle_only and cycle_last_stage == 1:
                preapproach_trajectory = ()
            else:
                preapproach_trajectory = self._plan_preapproach(
                    preapproach_planning_pose,
                    pick_ready_end,
                )
                if preapproach_trajectory is None:
                    return None
        if preapproach_trajectory:
            preapproach_end = self._trajectory_end_state(
                preapproach_trajectory
            )
        elif pick_ready_end is not None:
            preapproach_end = copy.deepcopy(pick_ready_end)
        else:
            self._record_plan_stage(
                "STEP_CYCLE_CONFIGURATION",
                "configuration",
                False,
                0.0,
                "STEP_CYCLE_END_STATE_MISSING",
            )
            return None

        approach_waypoints = (
            as_planning_pose(geometry.target_pose),
            *(as_planning_pose(pose) for pose in tip_motion.before_wait_waypoints),
        )
        step_approach_trajectories = ()
        if stepwise_plan:
            step_labels = (
                "Step preapproach to target",
                "Step tip +X forward",
                "Step tip +Z lift",
                "Step tip -X back",
                "Step tip +Z second lift",
            )
            step_groups = []
            flattened_approach = []
            step_start = preapproach_end
            cycle_waypoint_count = (
                max(0, cycle_last_stage - 2) if step_cycle_only else 5
            )
            for waypoint, label in zip(
                approach_waypoints[:cycle_waypoint_count],
                step_labels[:cycle_waypoint_count],
            ):
                group = self._plan_cartesian_with_ompl_fallback(
                    [waypoint],
                    step_start,
                    label,
                    pregrasp=False,
                )
                if group is None:
                    return None
                group = tuple(group)
                step_groups.append(group)
                flattened_approach.extend(group)
                step_start = self._trajectory_end_state(group)
            if step_cycle_only:
                step_groups.extend([()] * (5 - len(step_groups)))
            step_approach_trajectories = tuple(step_groups)
            approach_trajectory = tuple(flattened_approach)
        else:
            approach_trajectory = self._plan_cartesian_with_ompl_fallback(
                approach_waypoints,
                preapproach_end,
                "Approach and pre-wait harvest",
                pregrasp=False,
            )
            if approach_trajectory is None:
                return None

        approach_end = (
            self._trajectory_end_state(approach_trajectory)
            if approach_trajectory
            else copy.deepcopy(preapproach_end)
        )
        after_wait_trajectory = ()
        after_wait_end = approach_end
        outward_retreat_trajectory = ()
        return_pick_ready_trajectory = ()
        if step_cycle_only:
            retreat_after_harvest = False
            return_to_pick_ready = bool(
                self.get_parameter("return_to_pick_ready").value
            )
            if cycle_last_stage <= 2:
                final_planning_pose = copy.deepcopy(
                    preapproach_planning_pose
                )
            else:
                final_planning_pose = copy.deepcopy(
                    approach_waypoints[cycle_last_stage - 3]
                )
            if return_to_pick_ready:
                return_pick_ready_plan = self._plan_pick_ready(
                    approach_end,
                    label="RETURN_PICK_READY",
                )
                if return_pick_ready_plan is None:
                    return None
                return_pick_ready_trajectory, _ = return_pick_ready_plan
        else:
            after_wait_trajectory = self._plan_cartesian_with_ompl_fallback(
                [as_planning_pose(tip_motion.after_wait_pose)],
                approach_end,
                "Post-wait harvest",
                pregrasp=False,
            )
            if after_wait_trajectory is None:
                return None

            after_wait_end = self._trajectory_end_state(after_wait_trajectory)
            retreat_after_harvest = bool(
                self.get_parameter("retreat_after_harvest").value
            )
            if retreat_after_harvest:
                clearance = max(
                    0.0,
                    float(self.get_parameter("lift_retreat_clearance").value),
                )
                retreat_tip_pose = Pose()
                retreat_tip_pose.position.x = float(
                    tip_motion.after_wait_pose.position.x
                    + geometry.outward_axis[0] * clearance
                )
                retreat_tip_pose.position.y = float(
                    tip_motion.after_wait_pose.position.y
                    + geometry.outward_axis[1] * clearance
                )
                retreat_tip_pose.position.z = float(
                    tip_motion.after_wait_pose.position.z
                    + geometry.outward_axis[2] * clearance
                )
                retreat_tip_pose.orientation = tip_motion.after_wait_pose.orientation
                outward_retreat_trajectory = (
                    self._plan_cartesian_with_ompl_fallback(
                        [as_planning_pose(retreat_tip_pose)],
                        after_wait_end,
                        "Lift-safe outward retreat",
                        pregrasp=False,
                    )
                )
                if outward_retreat_trajectory is None:
                    return None
                self.last_plan_report["lift_safe_retreat"] = {
                    "clearance_m": clearance,
                    "outward_axis": [
                        float(value) for value in geometry.outward_axis
                    ],
                }

            return_to_pick_ready = bool(
                self.get_parameter("return_to_pick_ready").value
            )
            if retreat_after_harvest and return_to_pick_ready:
                self.get_logger().error(
                    "retreat_after_harvest and return_to_pick_ready cannot both be true"
                )
                self._record_plan_stage(
                    "LIFT_SAFE_RETREAT_CONFIGURATION",
                    "configuration",
                    False,
                    0.0,
                    "RETREAT_AND_RETURN_BOTH_ENABLED",
                )
                return None
            if return_to_pick_ready:
                return_pick_ready_plan = self._plan_pick_ready(
                    after_wait_end,
                    label="RETURN_PICK_READY",
                )
                if return_pick_ready_plan is None:
                    return None
                return_pick_ready_trajectory, _ = return_pick_ready_plan
            final_planning_pose = copy.deepcopy(
                as_planning_pose(tip_motion.after_wait_pose)
            )

        display_start_state = self._complete_display_start_state(
            display_start_state
        )
        plan = HarvestMotionPlan(
            pick_ready_trajectory=pick_ready_trajectory,
            preapproach_trajectory=preapproach_trajectory,
            approach_trajectory=approach_trajectory,
            after_wait_trajectory=after_wait_trajectory,
            outward_retreat_trajectory=outward_retreat_trajectory,
            return_pick_ready_trajectory=return_pick_ready_trajectory,
            display_start_state=display_start_state,
            arc_reverse_recovery_trajectory=(
                arc_reverse_recovery_trajectory
            ),
            end_planning_pose=final_planning_pose,
            step_approach_trajectories=step_approach_trajectories,
            preapproach_planning_pose=copy.deepcopy(
                preapproach_planning_pose
            ),
            outward_axis=tuple(
                float(value) for value in geometry.outward_axis
            ),
        )
        planned_trajectories = []
        for segment in (
            arc_reverse_recovery_trajectory,
            pick_ready_trajectory,
            preapproach_trajectory,
            approach_trajectory,
            after_wait_trajectory,
            outward_retreat_trajectory,
            return_pick_ready_trajectory,
        ):
            if isinstance(segment, (list, tuple)):
                planned_trajectories.extend(segment)
            elif segment is not None:
                planned_trajectories.append(segment)
        if bool(self.get_parameter("publish_display_trajectory").value):
            display = DisplayTrajectory()
            display.model_id = self.robot_model_id
            display.trajectory_start = display_start_state
            display.trajectory.extend(planned_trajectories)
            self.display_publisher.publish(display)
        if step_cycle_only:
            self.last_plan_report["step_cycle_only"] = True
            self.last_plan_report["step_cycle_last_stage"] = (
                cycle_last_stage
            )
            self.get_logger().info(
                f"Approach repeat plan ready through stage "
                f"{cycle_last_stage}. Later stages were intentionally "
                "not planned."
            )
            return plan
        if return_to_pick_ready:
            finish_label = "constrained OMPL RETURN_PICK_READY"
        elif retreat_after_harvest:
            finish_label = "lift-safe outward retreat"
        else:
            finish_label = "keep post-wait pose"
        if continuous_transition:
            start_label = (
                "continuous outward arc pre-grasp"
                if self.last_plan_report.get(
                    "continuous_transition_arc", False
                )
                else "cached reverse-to-PICK_READY fallback pre-grasp"
            )
        else:
            start_label = "PICK_READY"
        self.get_logger().info(
            "Full harvest plan ready: "
            f"{start_label} "
            "-> direct TCP pre-approach "
            f"({self.get_parameter('preapproach_mode').value}/"
            f"{self.get_parameter('planning_pipeline_id').value}) -> "
            f"{self.planning_link}-based Cartesian-first approach -> "
            f"+X{float(self.get_parameter('harvest_x_forward').value) * 1000.0:.0f}mm "
            "-> +Z40mm -> -X30mm -> +Z10mm -> wait -> -X30mm "
            f"-> {finish_label} "
            "(Cartesian 실패 구간은 constrained OMPL fallback)"
        )
        return plan

    def execute(self, plan: HarvestMotionPlan) -> bool:
        if not bool(self.get_parameter("execute").value):
            self.get_logger().info(
                "Plan-only test complete. Set execute:=true only after inspecting the RViz path."
            )
            return True

        if plan.arc_reverse_recovery_trajectory:
            if not self._execute_trajectory_sequence(
                plan.arc_reverse_recovery_trajectory,
                "Arc failure cached reverse recovery to PICK_READY",
            ):
                return False
            self.get_logger().info(
                "Arc 실패 복구 완료: 기존 성공 궤적을 역순으로 따라 "
                "PICK_READY에 도착했습니다."
            )

        if plan.pick_ready_trajectory and not self._execute_trajectory_group(
            plan.pick_ready_trajectory, "PICK_READY"
        ):
            return False
        if not self._execute_trajectory_sequence(
            plan.preapproach_trajectory,
            "Pre-approach",
        ):
            return False
        if not self._execute_trajectory_sequence(
            plan.approach_trajectory,
            "Approach and pre-wait harvest",
        ):
            return False

        if bool(self.get_parameter("step_cycle_only").value):
            if plan.return_pick_ready_trajectory:
                return_label = (
                    "CACHED REVERSE RETURN_PICK_READY"
                    if plan.return_pick_ready_is_cached_reverse
                    else "OMPL RETURN_PICK_READY"
                )
                if not self._execute_trajectory_group(
                    plan.return_pick_ready_trajectory,
                    return_label,
                ):
                    return False
                self.get_logger().info(
                    "Limited-stage batch segment complete; robot returned "
                    "to PICK_READY"
                    + (
                        " by reversing the cached successful path."
                        if plan.return_pick_ready_is_cached_reverse
                        else "."
                    )
                )
            else:
                self.get_logger().info(
                    "Limited-stage batch segment complete; current stage "
                    "pose is retained for the next continuous arc."
                )
            return True

        wait_seconds = max(
            0.0, float(self.get_parameter("harvest_wait_sec").value)
        )
        self.get_logger().info(f"Holding harvest pose for {wait_seconds:.2f}s")
        time.sleep(wait_seconds)

        if not self._execute_trajectory_sequence(
            plan.after_wait_trajectory,
            "Post-wait harvest",
        ):
            return False

        if plan.outward_retreat_trajectory:
            if not self._execute_trajectory_sequence(
                plan.outward_retreat_trajectory,
                "Lift-safe outward retreat",
            ):
                return False
            self.get_logger().info(
                "Lift-safe outward retreat complete; the lift may now move."
            )

        if plan.return_pick_ready_trajectory:
            return_label = (
                "CACHED REVERSE RETURN_PICK_READY"
                if plan.return_pick_ready_is_cached_reverse
                else "OMPL RETURN_PICK_READY"
            )
            if not self._execute_trajectory_group(
                plan.return_pick_ready_trajectory,
                return_label,
            ):
                return False
            self.get_logger().info(
                "Harvest sequence complete; robot returned to PICK_READY"
                + (
                    " by reversing the cached successful path."
                    if plan.return_pick_ready_is_cached_reverse
                    else "."
                )
            )
        elif plan.outward_retreat_trajectory:
            self.get_logger().info(
                "Harvest sequence complete; robot is holding the lift-safe "
                "outward retreat pose."
            )
        else:
            self.get_logger().info(
                "Harvest sequence complete; post-wait pose is retained for "
                "the next continuous tomato transition."
            )
        return True

    def _execute_trajectory_group(self, trajectories, label: str) -> bool:
        if isinstance(trajectories, (list, tuple)):
            return self._execute_trajectory_sequence(trajectories, label)
        return self._execute_trajectory(trajectories, label)

    def _execute_trajectory_sequence(self, trajectories, label: str) -> bool:
        sequence = (
            tuple(trajectories)
            if isinstance(trajectories, (list, tuple))
            else (trajectories,)
        )
        for index, trajectory in enumerate(sequence, start=1):
            segment_label = (
                label
                if len(sequence) == 1
                else f"{label} ({index}/{len(sequence)})"
            )
            if not self._execute_trajectory(trajectory, segment_label):
                return False
        return True

    def _execute_trajectory(self, trajectory, label: str) -> bool:
        current_positions = self._wait_for_current_joint_positions(
            timeout_sec=2.0
        )
        safety_violations = self._trajectory_safety_violations(
            trajectory,
            start_positions=current_positions,
        )
        if safety_violations:
            self._log_trajectory_safety_failure(
                f"{label} 실행 전 검사",
                safety_violations,
            )
            self._record_plan_stage(
                "EXECUTION_TRAJECTORY_SAFETY",
                "trajectory_validation",
                False,
                0.0,
                "UNSAFE_CACHED_TRAJECTORY",
                trajectory_safety_violations=safety_violations,
                execution_label=label,
            )
            return False

        timeout = max(1.0, float(self.get_parameter("execution_timeout_sec").value))
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt execute_trajectory action is unavailable")
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result() if goal_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"MoveIt rejected {label} trajectory execution")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        wrapped_result = result_future.result() if result_future.done() else None
        success = (
            wrapped_result is not None
            and wrapped_result.result.error_code.val == MoveItErrorCodes.SUCCESS
        )
        error_code = (
            wrapped_result.result.error_code.val if wrapped_result is not None else "timeout"
        )
        self.get_logger().info(
            f"{label} execution success={success} error_code={error_code}"
        )
        return success
