import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
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
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


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
class HarvestMotionPlan:
    """Trajectories for the complete PICK_READY-to-harvest sequence."""

    pick_ready_trajectory: object
    pre_rotation_trajectory: object
    preapproach_trajectory: object
    approach_trajectory: object
    after_wait_trajectory: object
    display_start_state: RobotState


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


def local_y_alignment_delta(current_pose: Pose, target_pose: Pose) -> float:
    """Project the current-to-target orientation change onto local +Y."""
    relative_rotation = (
        rotation_from_pose(current_pose).T @ rotation_from_pose(target_pose)
    )
    sine = float(relative_rotation[0, 2] - relative_rotation[2, 0])
    cosine = float(relative_rotation[0, 0] + relative_rotation[2, 2])
    if math.hypot(sine, cosine) < 1e-12:
        raise ValueError("Target orientation has no unique wrist3 projection")
    return math.atan2(sine, cosine)


def pose_rotated_about_local_y(pose: Pose, angle_rad: float) -> Pose:
    """Keep position fixed and rotate a pose about its local +Y axis."""
    sine = math.sin(float(angle_rad))
    cosine = math.cos(float(angle_rad))
    local_y_rotation = np.array(
        [
            [cosine, 0.0, sine],
            [0.0, 1.0, 0.0],
            [-sine, 0.0, cosine],
        ],
        dtype=float,
    )
    target = Pose()
    target.position.x = float(pose.position.x)
    target.position.y = float(pose.position.y)
    target.position.z = float(pose.position.z)
    (
        target.orientation.x,
        target.orientation.y,
        target.orientation.z,
        target.orientation.w,
    ) = quaternion_from_rotation(rotation_from_pose(pose) @ local_y_rotation)
    return target


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


def make_tip_local_harvest_motion(
    start_pose: Pose,
    x_forward: float = 0.050,
    first_z_lift: float = 0.020,
    first_x_back: float = 0.015,
    second_z_lift: float = 0.010,
    second_x_back: float = 0.030,
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
        self.declare_parameter(
            "pick_ready_joint_names",
            ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"],
        )
        self.declare_parameter(
            "pick_ready_joint_positions",
            [
                1.543469497691539,
                1.181013622733317547,
                -2.058567995722523,
                -0.6538175657861676,
                -1.5872183101827066,
                3.269133302406451,
            ],
        )
        self.declare_parameter("pick_ready_joint_tolerance", 0.005)
        self.declare_parameter("pick_ready_planning_time", 10.0)
        self.declare_parameter("pick_ready_planning_attempts", 5)
        self.declare_parameter("pick_ready_velocity_scale", 0.20)
        self.declare_parameter("pick_ready_acceleration_scale", 0.20)
        self.declare_parameter("planning_pipeline_id", "ompl")
        self.declare_parameter("planner_id", "RRTConnect")
        self.declare_parameter("preapproach_mode", "cartesian")
        self.declare_parameter("joint_planning_pipeline_id", "ompl")
        self.declare_parameter("joint_planner_id", "RRTConnect")
        self.declare_parameter(
            "ompl_limited_joint_names",
            ["base", "shoulder", "elbow", "wrist1", "wrist2"],
        )
        self.declare_parameter("ompl_joint_tolerance_deg", 120.0)
        self.declare_parameter("preapproach_position_tolerance", 0.005)
        self.declare_parameter("preapproach_orientation_tolerance", 0.05)
        self.declare_parameter("tip_standoff", 0.025)
        self.declare_parameter("tip_below_center", 0.018)
        self.declare_parameter("preapproach_clearance", 0.040)
        self.declare_parameter("harvest_x_forward", 0.050)
        self.declare_parameter("harvest_first_z_lift", 0.020)
        self.declare_parameter("harvest_first_x_back", 0.015)
        self.declare_parameter("harvest_second_z_lift", 0.010)
        self.declare_parameter("harvest_wait_sec", 2.0)
        self.declare_parameter("harvest_second_x_back", 0.030)
        self.declare_parameter("max_step", 0.005)
        self.declare_parameter("jump_threshold", 0.0)
        self.declare_parameter("minimum_fraction", 0.98)
        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("execution_timeout_sec", 60.0)
        self.declare_parameter("publish_display_trajectory", True)
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
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self.display_publisher = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )
        self._latest_joint_positions: dict[str, float] = {}
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
        for name, position in zip(message.name, message.position):
            if math.isfinite(position):
                self._latest_joint_positions[str(name)] = float(position)

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

    def _begin_plan_report(self) -> None:
        self._trajectory_range_records = []
        self._plan_start_joint_positions = (
            self._wait_for_current_joint_positions()
        )
        self.last_plan_report = {
            "success": False,
            "tomato_frame": self.tomato_frame,
            "pipeline": str(self.get_parameter("planning_pipeline_id").value),
            "planner_id": str(self.get_parameter("planner_id").value),
            "preapproach_mode": str(
                self.get_parameter("preapproach_mode").value
            ),
            "start_joint_positions": dict(
                self._plan_start_joint_positions
            ),
            "stages": [],
            "_started_monotonic": time.monotonic(),
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
            if record["pregrasp"] and (record["accepted"] or not success)
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
                "OMPL 시작 자세 중심 constraint를 만들 수 없습니다. "
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
            f"OMPL 시작 자세 중심 constraint ±{tolerance_deg:.1f}°: "
            f"{centers}; wrist3=제외"
        )
        return True

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
        if len(joint_names) != 6 or len(joint_names) != len(joint_positions):
            self.get_logger().error("PICK_READY must contain exactly six joint values")
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

        return self._plan_joint_target(
            joint_names,
            joint_positions,
            start_state=start_state,
            label=label,
        )

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
        success = (
            result.error_code.val == MoveItErrorCodes.SUCCESS and point_count > 0
        )
        self._record_trajectory_range_input(
            stage,
            trajectory,
            success,
            pregrasp=(
                label == "PICK_READY"
                or stage.endswith("_PREAPPROACH")
            ),
        )
        self.get_logger().info(
            f"{label} plan success={success} error_code={result.error_code.val} "
            f"points={point_count} planning_time={result.planning_time:.3f}s"
        )
        if not success:
            self._record_failure_robot_state(stage, trajectory)
            self._record_plan_stage(
                stage,
                pipeline,
                False,
                time.monotonic() - stage_started,
                "MOVEIT_PLANNING_FAILED",
                moveit_error_code=int(result.error_code.val),
                point_count=point_count,
                planning_time_sec=float(result.planning_time),
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

    def _plan_cartesian_pre_rotation(
        self,
        preapproach_pose: Pose,
        pick_ready_end: RobotState,
    ):
        pick_ready_pose = self._compute_planning_link_pose(
            pick_ready_end,
            stage="FK_PRE_ROTATION",
            planner_type="fk",
        )
        if pick_ready_pose is None:
            return None

        try:
            rotation_delta = local_y_alignment_delta(
                pick_ready_pose,
                preapproach_pose,
            )
            rotation_pose = pose_rotated_about_local_y(
                pick_ready_pose,
                rotation_delta,
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            self._record_plan_stage(
                "CARTESIAN_PRE_ROTATION",
                "cartesian",
                False,
                0.0,
                "PRE_ROTATION_ORIENTATION_AMBIGUOUS",
            )
            return None
        self.get_logger().info(
            "Cartesian TCP pre-rotation at the PICK_READY position: "
            f"local +Y delta={math.degrees(rotation_delta):+.1f}°"
        )
        return self._plan_cartesian_with_ompl_fallback(
            [rotation_pose],
            pick_ready_end,
            "TCP in-place pre-rotation",
            pregrasp=True,
        )

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
        success = (
            result.error_code.val == MoveItErrorCodes.SUCCESS
            and point_count > 0
        )
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
        reason = "" if success else "MOVEIT_PLANNING_FAILED"
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

    def _compute_planning_link_pose(
        self,
        robot_state: RobotState,
        stage: str = "CARTESIAN_RETURN_FK",
        planner_type: str = "cartesian",
    ):
        stage_started = time.monotonic()
        timeout = max(0.1, float(self.get_parameter("service_timeout_sec").value))
        if not self.fk_client.wait_for_service(timeout_sec=timeout):
            self._record_plan_stage(
                stage,
                planner_type,
                False,
                time.monotonic() - stage_started,
                "FK_SERVICE_UNAVAILABLE",
            )
            return None
        request = GetPositionFK.Request()
        request.header.frame_id = self.base_frame
        request.fk_link_names = [self.planning_link]
        request.robot_state = robot_state
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        response = future.result() if future.done() else None
        success = (
            response is not None
            and response.error_code.val == MoveItErrorCodes.SUCCESS
            and bool(response.pose_stamped)
        )
        if not success:
            error_code = (
                int(response.error_code.val) if response is not None else 0
            )
            self._record_plan_stage(
                stage,
                planner_type,
                False,
                time.monotonic() - stage_started,
                "FK_FAILED_OR_TIMEOUT",
                moveit_error_code=error_code,
            )
            return None
        self._record_plan_stage(
            stage,
            planner_type,
            True,
            time.monotonic() - stage_started,
            moveit_error_code=int(response.error_code.val),
        )
        return response.pose_stamped[0].pose

    def _plan_cartesian(
        self,
        waypoints,
        start_state: RobotState,
        label: str,
        terminal_failure: bool = True,
    ):
        stage_started = time.monotonic()
        stage_names = {
            "TCP in-place pre-rotation": "CARTESIAN_PRE_ROTATION",
            "TCP direct pre-approach": "CARTESIAN_PREAPPROACH",
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
        request.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)

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
        success = (
            response.error_code.val == MoveItErrorCodes.SUCCESS
            and response.fraction >= minimum_fraction
            and point_count > 0
        )
        self._record_trajectory_range_input(
            stage,
            response.solution,
            success,
            pregrasp=(
                stage == "CARTESIAN_PRE_ROTATION"
                or stage == "CARTESIAN_PREAPPROACH"
            ),
        )
        self.get_logger().info(
            f"{label} Cartesian success={success} "
            f"fraction={response.fraction:.3f}/{minimum_fraction:.3f} "
            f"points={point_count} error_code={response.error_code.val}"
        )
        reason = ""
        if not success:
            if response.fraction < minimum_fraction:
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
            return (cartesian,)
        recorded_stages = self.last_plan_report.get("stages", [])
        cartesian_failure = (
            dict(recorded_stages[-1]) if recorded_stages else {}
        )

        stage_names = {
            "TCP in-place pre-rotation": "PRE_ROTATION",
            "TCP direct pre-approach": "PREAPPROACH",
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

        self.last_plan_report.setdefault("cartesian_fallbacks", []).append(
            {
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
        )
        self.get_logger().info(
            f"{label} OMPL fallback 성공: "
            f"{len(trajectories)}개 waypoint"
        )
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

    def plan(self):
        self._begin_plan_report()
        try:
            result = self._plan_impl()
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

    def _plan_impl(self):
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

        pick_ready_plan = self._plan_pick_ready()
        if pick_ready_plan is None:
            return None
        pick_ready_trajectory, display_start_state = pick_ready_plan

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
        pick_ready_end = self._trajectory_end_state(pick_ready_trajectory)
        pre_rotation_trajectory = self._plan_cartesian_pre_rotation(
            preapproach_planning_pose,
            pick_ready_end,
        )
        if pre_rotation_trajectory is None:
            return None
        pre_rotation_end = self._trajectory_end_state(
            pre_rotation_trajectory
        )
        preapproach_trajectory = self._plan_preapproach(
            preapproach_planning_pose,
            pre_rotation_end,
        )
        if preapproach_trajectory is None:
            return None
        preapproach_end = self._trajectory_end_state(preapproach_trajectory)

        approach_waypoints = (
            as_planning_pose(geometry.target_pose),
            *(as_planning_pose(pose) for pose in tip_motion.before_wait_waypoints),
        )
        approach_trajectory = self._plan_cartesian_with_ompl_fallback(
            approach_waypoints,
            preapproach_end,
            "Approach and pre-wait harvest",
            pregrasp=False,
        )
        if approach_trajectory is None:
            return None

        approach_end = self._trajectory_end_state(approach_trajectory)
        after_wait_trajectory = self._plan_cartesian_with_ompl_fallback(
            [as_planning_pose(tip_motion.after_wait_pose)],
            approach_end,
            "Post-wait harvest",
            pregrasp=False,
        )
        if after_wait_trajectory is None:
            return None

        plan = HarvestMotionPlan(
            pick_ready_trajectory=pick_ready_trajectory,
            pre_rotation_trajectory=pre_rotation_trajectory,
            preapproach_trajectory=preapproach_trajectory,
            approach_trajectory=approach_trajectory,
            after_wait_trajectory=after_wait_trajectory,
            display_start_state=display_start_state,
        )
        planned_trajectories = [pick_ready_trajectory]
        for segment in (
            pre_rotation_trajectory,
            preapproach_trajectory,
            approach_trajectory,
            after_wait_trajectory,
        ):
            planned_trajectories.extend(segment)
        if bool(self.get_parameter("publish_display_trajectory").value):
            display = DisplayTrajectory()
            display.model_id = self.robot_model_id
            display.trajectory_start = display_start_state
            display.trajectory.extend(planned_trajectories)
            self.display_publisher.publish(display)
        self.get_logger().info(
            "Full harvest plan ready: PICK_READY -> Cartesian-first TCP "
            "pre-rotation -> TCP pre-approach "
            f"({self.get_parameter('preapproach_mode').value}/"
            f"{self.get_parameter('planning_pipeline_id').value}) -> "
            f"{self.planning_link}-based Cartesian-first approach -> "
            "+X50mm -> +Z20mm -> -X15mm -> +Z10mm -> wait -> -X30mm "
            "(Cartesian 실패 구간은 constrained OMPL fallback)"
        )
        return plan

    def execute(self, plan: HarvestMotionPlan) -> bool:
        if not bool(self.get_parameter("execute").value):
            self.get_logger().info(
                "Plan-only test complete. Set execute:=true only after inspecting the RViz path."
            )
            return True

        if not self._execute_trajectory(plan.pick_ready_trajectory, "PICK_READY"):
            return False
        if not self._execute_trajectory_sequence(
            plan.pre_rotation_trajectory,
            "TCP pre-rotation",
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

        self.get_logger().info(
            "Harvest sequence complete; keeping the final post-wait pose."
        )
        return True

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
