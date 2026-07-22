import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MoveItErrorCodes,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class HarvestGeometry:
    """Poses and axes used for one tomato Cartesian approach."""

    target_pose: Pose
    preapproach_pose: Pose
    outward_axis: np.ndarray
    vine_point: np.ndarray
    tomato_position: np.ndarray


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
    """Create a level gripper pose with tomato between tip and main vine.

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
        self.declare_parameter("tomato_frame", "tomato_3_tf")
        self.declare_parameter("vine_frame", "main_vine_tf")
        self.declare_parameter("gripper_link", "tomato_gripper")
        self.declare_parameter("tip_link", "tomato_gripper_tip")
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
                1.1810132733317547,
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
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("approach_yaw_deg", -175.12)
        self.declare_parameter("tip_standoff", 0.025)
        self.declare_parameter("tip_below_center", 0.018)
        self.declare_parameter("preapproach_clearance", 0.040)
        self.declare_parameter("max_step", 0.005)
        self.declare_parameter("jump_threshold", 0.0)
        self.declare_parameter("minimum_fraction", 0.98)
        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("execution_timeout_sec", 60.0)
        self.declare_parameter("execute", False)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tomato_frame = str(self.get_parameter("tomato_frame").value)
        self.vine_frame = str(self.get_parameter("vine_frame").value)
        self.gripper_link = str(self.get_parameter("gripper_link").value)
        self.tip_link = str(self.get_parameter("tip_link").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.robot_model_id = str(self.get_parameter("robot_model_id").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self.display_publisher = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )
        self._pick_ready_trajectory = None
        self._pick_ready_start_state = None

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

    def _plan_pick_ready(self) -> bool:
        timeout = max(1.0, float(self.get_parameter("service_timeout_sec").value))
        if not self.move_group_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("MoveIt move_action server is unavailable")
            return False

        joint_names = [
            str(name) for name in self.get_parameter("pick_ready_joint_names").value
        ]
        joint_positions = [
            float(position)
            for position in self.get_parameter("pick_ready_joint_positions").value
        ]
        if len(joint_names) != 6 or len(joint_names) != len(joint_positions):
            self.get_logger().error("PICK_READY must contain exactly six joint values")
            return False

        tolerance = max(
            0.0001, float(self.get_parameter("pick_ready_joint_tolerance").value)
        )
        constraints = Constraints()
        constraints.name = "PICK_READY"
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
        goal.request.pipeline_id = str(
            self.get_parameter("planning_pipeline_id").value
        )
        goal.request.planner_id = str(self.get_parameter("planner_id").value)
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
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = True
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        goal_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=timeout)
        goal_handle = goal_future.result() if goal_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the PICK_READY planning request")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        wrapped_result = result_future.result() if result_future.done() else None
        if wrapped_result is None:
            self.get_logger().error("PICK_READY planning timed out")
            return False
        result = wrapped_result.result
        trajectory = result.planned_trajectory
        point_count = len(trajectory.joint_trajectory.points)
        success = (
            result.error_code.val == MoveItErrorCodes.SUCCESS and point_count > 0
        )
        self.get_logger().info(
            f"PICK_READY plan success={success} error_code={result.error_code.val} "
            f"points={point_count} planning_time={result.planning_time:.3f}s"
        )
        if not success:
            return False
        self._pick_ready_trajectory = trajectory
        self._pick_ready_start_state = result.trajectory_start
        return True

    def _pick_ready_end_state(self) -> RobotState:
        state = RobotState()
        state.is_diff = False
        trajectory = self._pick_ready_trajectory.joint_trajectory
        state.joint_state.name = list(trajectory.joint_names)
        state.joint_state.position = list(trajectory.points[-1].positions)
        return state

    def plan(self):
        if not self._plan_pick_ready():
            return None

        execute_requested = bool(self.get_parameter("execute").value)
        if execute_requested and not self.execute(
            self._pick_ready_trajectory, label="PICK_READY"
        ):
            return None

        tomato_tf = self._lookup_transform(self.tomato_frame)
        vine_tf = self._lookup_transform(self.vine_frame)
        tip_tf = self._lookup_transform(self.tip_link)
        gripper_to_tip_tf = self._lookup_transform(
            self.tip_link, parent_frame=self.gripper_link
        )
        if (
            tomato_tf is None
            or vine_tf is None
            or tip_tf is None
            or gripper_to_tip_tf is None
        ):
            return None

        tomato_position = self._translation(tomato_tf)
        vine_origin = self._translation(vine_tf)
        vine_axis = self._rotation_matrix(vine_tf)[:, 2]
        current_tip_position = self._translation(tip_tf)
        approach_yaw = math.radians(
            float(self.get_parameter("approach_yaw_deg").value)
        )
        outward_hint = np.array(
            [math.cos(approach_yaw), math.sin(approach_yaw), 0.0], dtype=float
        )
        geometry = make_harvest_geometry(
            tomato_position=tomato_position,
            vine_origin=vine_origin,
            vine_axis=vine_axis,
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
            f"preapproach=({pre.x:.4f}, {pre.y:.4f}, {pre.z:.4f}) "
            f"target=({target.x:.4f}, {target.y:.4f}, {target.z:.4f}) "
            f"tip_below={float(self.get_parameter('tip_below_center').value):.4f}m "
            f"tip-tomato-vine_order={'OK' if order_dot > 0.0 else 'INVALID'}"
        )
        if order_dot <= 0.0:
            self.get_logger().error("Tomato is not between the gripper tip and main vine")
            return None

        timeout = max(0.1, float(self.get_parameter("service_timeout_sec").value))
        if not self.cartesian_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("MoveIt compute_cartesian_path service is unavailable")
            return None

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        if execute_requested:
            request.start_state.is_diff = True
        else:
            request.start_state = self._pick_ready_end_state()
        request.group_name = self.group_name
        request.link_name = self.tip_link
        request.waypoints = [geometry.preapproach_pose, geometry.target_pose]
        request.max_step = max(0.0001, float(self.get_parameter("max_step").value))
        request.jump_threshold = max(
            0.0, float(self.get_parameter("jump_threshold").value)
        )
        request.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)

        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        response = future.result() if future.done() else None
        if response is None:
            self.get_logger().error("Cartesian planning service timed out")
            return None

        point_count = len(response.solution.joint_trajectory.points)
        minimum_fraction = float(self.get_parameter("minimum_fraction").value)
        success = (
            response.error_code.val == MoveItErrorCodes.SUCCESS
            and response.fraction >= minimum_fraction
            and point_count > 0
        )
        self.get_logger().info(
            f"Cartesian result success={success} fraction={response.fraction:.3f}/"
            f"{minimum_fraction:.3f} points={point_count} "
            f"error_code={response.error_code.val}"
        )
        if not success:
            return None

        display = DisplayTrajectory()
        display.model_id = self.robot_model_id
        if execute_requested:
            display.trajectory_start = response.start_state
        else:
            display.trajectory_start = self._pick_ready_start_state
            display.trajectory.append(self._pick_ready_trajectory)
        display.trajectory.append(response.solution)
        self.display_publisher.publish(display)
        return response.solution

    def execute(self, trajectory, label: str = "Cartesian approach") -> bool:
        if not bool(self.get_parameter("execute").value):
            self.get_logger().info(
                "Plan-only test complete. Set execute:=true only after inspecting the RViz path."
            )
            return True

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
