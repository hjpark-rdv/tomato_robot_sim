import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import ParameterDescriptor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tomato_moveit_teach.geometry import quaternion_from_matrix, target_stem_axis


PALM_DEPTH = 0.012
FINGER_LEN = 0.075 * 0.70
OPENING = 0.0875 * 0.60
FINGER_THICKNESS = 0.007
GRASP_Z = PALM_DEPTH + FINGER_LEN * 0.62
FINGERTIP_Z = PALM_DEPTH + FINGER_LEN
ROBOT_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class MoveItGraspPlanner(Node):
    def __init__(self, node_name: str = "moveit_grasp_planner") -> None:
        super().__init__(node_name)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("repo_root", "/root/pybullet_ur_approach")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("trajectory_controller_action", "/fake_ur5_controller/follow_joint_trajectory")
        self.declare_parameter("group_name", "ur5_harvest")
        self.declare_parameter("target_link", "grasp_tf")
        self.declare_parameter("object_position", [0.455, -0.175, 0.34])
        self.declare_parameter("scene_y_offset", 0.0)
        self.declare_parameter("robot_y_offset", 0.0)
        self.declare_parameter("object_radius", 0.036)
        self.declare_parameter("tomato_radius_scale", 0.5)
        numeric_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("tomato_z_spin_deg", 0.0, numeric_descriptor)
        self.declare_parameter("taught_tomato_z_spin_deg", -40.0, numeric_descriptor)
        self.declare_parameter("right_tomato_z_spin_offset_deg", -30.0, numeric_descriptor)
        self.declare_parameter("left_tomato_z_spin_offset_deg", 250.0, numeric_descriptor)
        self.declare_parameter("randomize_individual_stem_spin", False)
        self.declare_parameter("individual_stem_spin_seed", 0, numeric_descriptor)
        self.declare_parameter("individual_stem_spin_min_deg", 0.0, numeric_descriptor)
        self.declare_parameter("individual_stem_spin_max_deg", 360.0, numeric_descriptor)
        self.declare_parameter("position_tolerance", 0.003)
        self.declare_parameter("orientation_tolerance_deg", 1.0)
        self.declare_parameter("allowed_planning_time", 8.0)
        self.declare_parameter("planning_attempts", 8)
        self.declare_parameter("planning_pipeline_id", "ompl")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("move_group_action_timeout_sec", 30.0)
        self.declare_parameter("use_home_start_state_for_planning", False)
        self.declare_parameter("trajectory_action_timeout_sec", 30.0)
        self.declare_parameter("velocity_scale", 0.35)
        self.declare_parameter("acceleration_scale", 0.35)
        self.declare_parameter("execute", False)
        self.declare_parameter("try_both_approach_sides", False)
        self.declare_parameter("tomato_y_back_approach_sign", 1.0)
        self.declare_parameter("target_tolerance", 0.003)
        self.declare_parameter("rule_axis_tolerance_deg", 1.0)
        self.declare_parameter("blue_alignment_tolerance_deg", 6.0)
        self.declare_parameter("min_grasp_tf_z_above_center", 0.005)
        self.declare_parameter("min_tool_tip_z_above_center", 0.005)
        self.declare_parameter("tool_length", 0.15)
        self.declare_parameter("tool_radius", 0.006)
        self.declare_parameter("tool_tilt_x_deg", 0.0)
        self.declare_parameter("tool_tilt_y_deg", -45.0)
        self.declare_parameter("tool_bend_straight_ratio", 0.5)
        self.declare_parameter("tool_gripper_under_offset_ratio", 0.5)
        self.declare_parameter("gripper_opening", OPENING)
        self.declare_parameter("gripper_x_roll_deg", 90.0)
        self.declare_parameter("gripper_spin_zero_offset_deg", 90.0)
        self.declare_parameter("gripper_z_rot_deg", 0.0)
        self.declare_parameter("use_start_relative_pose_joint_constraints", True)
        self.declare_parameter("start_relative_joint_state_timeout_sec", 0.5)
        self.declare_parameter("shoulder_pan_start_goal_constraint_deg", 120.0)
        self.declare_parameter("wrist_1_start_goal_constraint_deg", 120.0)
        self.declare_parameter("wrist_2_start_goal_constraint_deg", 90.0)
        self.declare_parameter("start_relative_joint_constraint_weight", 1.0)
        self.declare_parameter("use_trajectory_naturalness_filter", True)
        self.declare_parameter("use_wrist2_vine_guard", True)
        self.declare_parameter("wrist2_vine_guard_link", "wrist_2_link")
        self.declare_parameter("wrist2_vine_guard_links", ["wrist_2_link", "wrist_3_link", "tool0"])
        self.declare_parameter("wrist2_vine_guard_margin", 0.0)
        self.declare_parameter("wrist2_vine_guard_sample_stride", 1)
        self.declare_parameter("max_trajectory_waypoint_jump_deg", 90.0)
        self.declare_parameter("max_trajectory_joint_path_deg", 240.0)
        self.declare_parameter("max_trajectory_total_path_deg", 620.0)
        self.declare_parameter("max_trajectory_start_goal_joint_delta_deg", 170.0)
        self.declare_parameter("max_shoulder_pan_path_deg", 130.0)
        self.declare_parameter("max_wrist_path_deg", 220.0)
        self.declare_parameter("limit_wrist_2_from_start", True)
        self.declare_parameter("max_wrist_2_start_delta_deg", 90.0)
        self.declare_parameter("trajectory_total_motion_weight", 1.0)
        self.declare_parameter("trajectory_max_joint_motion_weight", 2.0)
        self.declare_parameter("trajectory_max_waypoint_jump_weight", 4.0)
        self.declare_parameter("trajectory_start_goal_motion_weight", 1.0)
        self.declare_parameter("trajectory_planning_time_weight", 0.05)
        self.declare_parameter("grasp_precision_contact_weight", 4.0)
        self.declare_parameter("grasp_precision_axis_weight", 2.0)
        self.declare_parameter("grasp_precision_alignment_weight", 2.0)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.target_link = str(self.get_parameter("target_link").value)
        self.base_object_position = np.array(self.get_parameter("object_position").value, dtype=float)
        self.object_position = self.base_object_position.copy()
        self.scene_y_offset = float(self.get_parameter("scene_y_offset").value)
        self.robot_y_offset = float(self.get_parameter("robot_y_offset").value)
        if abs(self.robot_y_offset) < 1e-12 and abs(self.scene_y_offset) > 1e-12:
            self.robot_y_offset = self.scene_y_offset
        self.object_radius = float(self.get_parameter("object_radius").value)
        self.tomato_radius_scale = float(self.get_parameter("tomato_radius_scale").value)
        self.tomato_radius = self.object_radius * self.tomato_radius_scale
        self.tomato_z_spin_deg = float(self.get_parameter("tomato_z_spin_deg").value)
        self.taught_tomato_z_spin_deg = float(self.get_parameter("taught_tomato_z_spin_deg").value)
        self.right_tomato_z_spin_offset_deg = float(self.get_parameter("right_tomato_z_spin_offset_deg").value)
        self.left_tomato_z_spin_offset_deg = float(self.get_parameter("left_tomato_z_spin_offset_deg").value)
        self.randomize_individual_stem_spin = bool(self.get_parameter("randomize_individual_stem_spin").value)
        self.individual_stem_spin_seed = int(self.get_parameter("individual_stem_spin_seed").value)
        self.individual_stem_spin_min_deg = float(self.get_parameter("individual_stem_spin_min_deg").value)
        self.individual_stem_spin_max_deg = float(self.get_parameter("individual_stem_spin_max_deg").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.orientation_tolerance = math.radians(float(self.get_parameter("orientation_tolerance_deg").value))
        self.tool_length = float(self.get_parameter("tool_length").value)
        self.tool_radius = float(self.get_parameter("tool_radius").value)
        self.tool_tilt_x_rad = math.radians(float(self.get_parameter("tool_tilt_x_deg").value))
        self.tool_tilt_y_rad = math.radians(float(self.get_parameter("tool_tilt_y_deg").value))
        self.tool_bend_straight_ratio = min(
            max(float(self.get_parameter("tool_bend_straight_ratio").value), 0.0),
            0.95,
        )
        self.tool_gripper_under_offset_ratio = float(
            self.get_parameter("tool_gripper_under_offset_ratio").value
        )
        self.gripper_opening = float(self.get_parameter("gripper_opening").value)
        self.gripper_x_roll_rad = math.radians(float(self.get_parameter("gripper_x_roll_deg").value))
        self.gripper_spin_zero_offset_rad = math.radians(
            float(self.get_parameter("gripper_spin_zero_offset_deg").value)
        )
        self.gripper_z_rot_rad = math.radians(float(self.get_parameter("gripper_z_rot_deg").value))
        self.execute = bool(self.get_parameter("execute").value)
        scene_object_position = self._scene_position(self.base_object_position)
        self.main_start, self.main_end = self._vine_points()
        self.target_main_vine_spin_deg = self._individual_main_vine_spin_deg(4, 0)
        self.object_position = self._spin_point_around_main_vine(
            scene_object_position,
            self.target_main_vine_spin_deg,
        )
        self.base_stem_axis = target_stem_axis(self.base_object_position)
        self.stem_axis = self._spin_vector_around_main_vine(
            self.base_stem_axis,
            self.target_main_vine_spin_deg,
        )
        base_branch_start = self._branch_start_for_down_angle(
            self.main_start,
            self.main_end,
            scene_object_position,
            45.0,
        )
        self.branch_start = self._spin_point_around_main_vine(base_branch_start, self.target_main_vine_spin_deg)
        map_main_start, map_main_end = self._vine_points(self.base_object_position, apply_scene_offset=False)
        map_branch_start = self._branch_start_for_down_angle(
            map_main_start,
            map_main_end,
            self.base_object_position,
            45.0,
        )
        self.target_z_spin_deg = self._local_tomato_z_spin_deg(map_branch_start, self.base_object_position)

        self.client = ActionClient(self, MoveGroup, str(self.get_parameter("move_group_action").value))
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_controller_action").value),
        )
        self.display_trajectory_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
        self.latest_joint_positions: dict[str, float] = {}
        self._fk_chain_cache: dict[str, list[dict[str, object]]] = {}
        self.create_subscription(JointState, "joint_states", self._joint_state_callback, 10)

    def _tomato_y_back_approach_sign(self) -> float:
        return 1.0 if float(self.get_parameter("tomato_y_back_approach_sign").value) >= 0.0 else -1.0

    def plan(self) -> bool:
        self.get_logger().info("Waiting for MoveIt move_group action server...")
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("MoveIt action server was not found. Is teach.launch.py running?")
            return False

        approach_signs = [1.0, -1.0] if bool(self.get_parameter("try_both_approach_sides").value) else [1.0]
        for approach_sign in approach_signs:
            candidate = self._target_pose_candidate(approach_sign)
            target_pose = candidate["pose"]
            self.get_logger().info(
                "Planning to "
                f"{self.target_link} approach_sign={approach_sign:+.0f} "
                f"xyz=({target_pose.position.x:.4f}, "
                f"{target_pose.position.y:.4f}, {target_pose.position.z:.4f}), "
                f"tool_tip_z_above_center={candidate['tool_tip_z_above_center']:.4f}, "
                f"grasp_tf_z_above_center={candidate['grasp_tf_z_above_center']:.4f}, "
                f"tool0_z_minus_gripper_link_z={candidate['tool0_z_minus_gripper_link_z']:.4f}, "
                f"x_axis_offset={candidate['tool_tip_x_axis_offset']:.4f}, "
                f"calyx_finger={candidate['calyx_finger']}, "
                f"calyx_contact_err={candidate['calyx_contact_error']:.4f}, "
                f"opposite_finger={candidate['opposite_finger']}, "
                f"opposite_contact_err={candidate['opposite_contact_error']:.4f}, "
                f"calyx_align_deg={candidate['calyx_alignment_angle_deg']:.2f}, "
                f"execute={self.execute}"
            )
            rejection_reasons = self._candidate_rejection_reasons(candidate)
            if rejection_reasons:
                self.get_logger().warn(
                    "Skipping candidate because " + "; ".join(rejection_reasons)
                )
                continue
            if self._plan_to_pose(target_pose):
                return True

        return False

    def _plan_to_pose(self, target_pose: Pose) -> bool:
        return bool(self._move_group_to_pose(target_pose)["success"])

    def _move_group_to_pose(self, target_pose: Pose, plan_only: bool | None = None) -> dict[str, object]:
        return self._move_group_to_constraints(self._pose_constraint(target_pose), plan_only, "pose")

    def _move_group_to_pose_with_joint_constraints(
        self,
        target_pose: Pose,
        joint_positions: dict[str, float],
        tolerance: float = 0.002,
        plan_only: bool | None = None,
        goal_label: str = "pose_with_joints",
    ) -> dict[str, object]:
        constraints = self._pose_constraint(target_pose)
        constraints.name = goal_label
        for joint_name, joint_position in joint_positions.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = str(joint_name)
            joint_constraint.position = float(joint_position)
            joint_constraint.tolerance_above = float(tolerance)
            joint_constraint.tolerance_below = float(tolerance)
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return self._move_group_to_constraints(constraints, plan_only, goal_label)

    def _joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_positions = {
            str(name): float(position)
            for name, position in zip(msg.name, msg.position)
        }

    def _wait_for_joint_positions(self) -> dict[str, float]:
        timeout_sec = float(self.get_parameter("start_relative_joint_state_timeout_sec").value)
        deadline = self.get_clock().now() + Duration(seconds=max(timeout_sec, 0.0))
        while rclpy.ok() and not self.latest_joint_positions and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return dict(self.latest_joint_positions)

    def _move_group_to_joint_positions(
        self,
        joint_positions: dict[str, float],
        tolerance: float = 0.01,
        plan_only: bool | None = None,
    ) -> dict[str, object]:
        constraints = Constraints()
        constraints.name = "joint_goal"
        for joint_name, joint_position in joint_positions.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = str(joint_name)
            joint_constraint.position = float(joint_position)
            joint_constraint.tolerance_above = float(tolerance)
            joint_constraint.tolerance_below = float(tolerance)
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return self._move_group_to_constraints(constraints, plan_only, "joint")

    def _move_group_to_constraints(
        self,
        constraints: Constraints,
        plan_only: bool | None = None,
        goal_label: str = "target",
    ) -> dict[str, object]:
        if plan_only is None:
            plan_only = not self.execute

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.pipeline_id = str(self.get_parameter("planning_pipeline_id").value)
        goal.request.planner_id = str(self.get_parameter("planner_id").value)
        goal.request.num_planning_attempts = int(self.get_parameter("planning_attempts").value)
        goal.request.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        goal.request.max_velocity_scaling_factor = float(self.get_parameter("velocity_scale").value)
        goal.request.max_acceleration_scaling_factor = float(self.get_parameter("acceleration_scale").value)
        if bool(self.get_parameter("use_home_start_state_for_planning").value):
            joint_names = [str(name) for name in self.get_parameter("home_joint_names").value]
            joint_positions = [float(position) for position in self.get_parameter("home_joint_positions").value]
            if len(joint_names) == len(joint_positions):
                goal.request.start_state.is_diff = False
                goal.request.start_state.joint_state.name = joint_names
                goal.request.start_state.joint_state.position = joint_positions
            else:
                goal.request.start_state.is_diff = True
                self.get_logger().warn(
                    "use_home_start_state_for_planning was requested, but home_joint_names and "
                    "home_joint_positions lengths do not match; falling back to current start state."
                )
        else:
            goal.request.start_state.is_diff = True
        goal.request.workspace_parameters.header.frame_id = self.base_frame
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = -0.05
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.3
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        action_timeout = float(self.get_parameter("move_group_action_timeout_sec").value)
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=action_timeout)
        if not future.done():
            self.get_logger().error(
                f"MoveIt did not return a {goal_label} goal handle within {action_timeout:.1f}s."
            )
            return {"success": False, "error_code": None, "trajectory": None, "planning_time": 0.0}
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the planning goal.")
            return {"success": False, "error_code": None, "trajectory": None, "planning_time": 0.0}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=action_timeout)
        if not result_future.done():
            self.get_logger().error(
                f"MoveIt did not return a {goal_label} planning result within {action_timeout:.1f}s."
            )
            return {"success": False, "error_code": None, "trajectory": None, "planning_time": 0.0}
        action_result = result_future.result()
        if action_result is None:
            self.get_logger().error("MoveIt returned no result.")
            return {"success": False, "error_code": None, "trajectory": None, "planning_time": 0.0}

        result = action_result.result
        trajectory = result.planned_trajectory if plan_only else result.executed_trajectory
        if not trajectory.joint_trajectory.points and result.planned_trajectory.joint_trajectory.points:
            trajectory = result.planned_trajectory
        point_count = len(trajectory.joint_trajectory.points)
        joint_names = ", ".join(trajectory.joint_trajectory.joint_names)
        success = result.error_code.val == MoveItErrorCodes.SUCCESS
        self.get_logger().info(
            f"MoveIt result success={success} error_code={result.error_code.val} "
            f"goal={goal_label} plan_only={bool(plan_only)} planning_time={result.planning_time:.3f}s "
            f"points={point_count} joints=[{joint_names}]"
        )
        return {
            "success": success,
            "error_code": result.error_code.val,
            "trajectory": trajectory,
            "trajectory_start": result.trajectory_start,
            "planning_time": float(result.planning_time),
            "point_count": point_count,
            "joint_names": list(trajectory.joint_trajectory.joint_names),
        }

    def _publish_display_trajectory(self, trajectory, trajectory_start=None) -> bool:
        if trajectory is None or not trajectory.joint_trajectory.points:
            self.get_logger().warn("Cannot publish an empty display trajectory.")
            return False
        try:
            if not self.context.ok():
                self.get_logger().warn("Cannot publish display trajectory because ROS context is already closed.")
                return False
        except RCLError as exc:
            self.get_logger().warn(f"Cannot publish display trajectory because ROS context is invalid: {exc}")
            return False
        msg = DisplayTrajectory()
        msg.model_id = self.group_name
        if trajectory_start is not None:
            msg.trajectory_start = trajectory_start
        msg.trajectory.append(trajectory)
        try:
            self.display_trajectory_pub.publish(msg)
        except RCLError as exc:
            self.get_logger().warn(f"Could not publish display trajectory: {exc}")
            return False
        self.get_logger().info(
            f"Published display trajectory points={len(trajectory.joint_trajectory.points)} "
            f"joints={list(trajectory.joint_trajectory.joint_names)}"
        )
        return True

    def _execute_joint_trajectory(self, trajectory) -> bool:
        if trajectory is None or not trajectory.joint_trajectory.points:
            self.get_logger().error("Cannot execute an empty trajectory.")
            return False
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowJointTrajectory action server was not found.")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        action_timeout = float(self.get_parameter("trajectory_action_timeout_sec").value)
        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=action_timeout)
        if not future.done():
            self.get_logger().error(
                f"Trajectory controller did not return a goal handle within {action_timeout:.1f}s."
            )
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory controller rejected the selected trajectory.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=action_timeout)
        if not result_future.done():
            self.get_logger().error(f"Trajectory controller did not finish within {action_timeout:.1f}s.")
            return False
        action_result = result_future.result()
        if action_result is None:
            self.get_logger().error("Trajectory controller returned no result.")
            return False
        result = action_result.result
        success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info(
            f"Trajectory execution success={success} error_code={result.error_code} "
            f"message={result.error_string!r}"
        )
        return success

    def _execute_joint_positions_direct(
        self,
        joint_names: list[str],
        joint_positions: list[float],
        duration_sec: float,
        label: str = "direct_joint",
    ) -> bool:
        if len(joint_names) != len(joint_positions):
            self.get_logger().error(
                f"Cannot execute {label}: joint_names and joint_positions length mismatch "
                f"({len(joint_names)} != {len(joint_positions)})."
            )
            return False
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowJointTrajectory action server was not found.")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(position) for position in joint_positions]
        point.velocities = [0.0] * len(joint_names)
        point.accelerations = [0.0] * len(joint_names)
        point.time_from_start = Duration(seconds=max(float(duration_sec), 0.1)).to_msg()
        goal.trajectory.points.append(point)

        action_timeout = max(
            float(self.get_parameter("trajectory_action_timeout_sec").value),
            float(duration_sec) + 5.0,
        )
        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=action_timeout)
        if not future.done():
            self.get_logger().error(
                f"Trajectory controller did not return a {label} goal handle within {action_timeout:.1f}s."
            )
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Trajectory controller rejected the {label} trajectory.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=action_timeout)
        if not result_future.done():
            self.get_logger().error(f"Trajectory controller did not finish {label} within {action_timeout:.1f}s.")
            return False
        action_result = result_future.result()
        if action_result is None:
            self.get_logger().error(f"Trajectory controller returned no {label} result.")
            return False
        result = action_result.result
        success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info(
            f"Direct joint trajectory {label} success={success} error_code={result.error_code} "
            f"duration={float(duration_sec):.2f}s message={result.error_string!r}"
        )
        return success

    def _trajectory_naturalness_metrics(self, trajectory, planning_time: float = 0.0) -> dict[str, object]:
        joint_names = list(trajectory.joint_trajectory.joint_names)
        points = list(trajectory.joint_trajectory.points)
        if not joint_names or not points:
            return {
                "valid": False,
                "reason": "empty trajectory",
                "joint_names": joint_names,
                "point_count": len(points),
                "score": float("inf"),
            }

        positions = []
        for point in points:
            if len(point.positions) != len(joint_names):
                continue
            positions.append([float(value) for value in point.positions])
        if not positions:
            return {
                "valid": False,
                "reason": "trajectory has no complete position points",
                "joint_names": joint_names,
                "point_count": len(points),
                "score": float("inf"),
            }

        position_array = np.array(positions, dtype=float)
        last_stamp = points[-1].time_from_start
        trajectory_duration_sec = float(last_stamp.sec) + float(last_stamp.nanosec) * 1e-9
        step_deltas = np.diff(position_array, axis=0) if len(position_array) > 1 else np.zeros((0, len(joint_names)))
        abs_steps = np.abs(step_deltas)
        joint_path_motion_all = np.sum(abs_steps, axis=0) if len(abs_steps) else np.zeros(len(joint_names))
        start_goal_delta_all = np.abs(position_array[-1] - position_array[0])
        arm_indices = [index for index, name in enumerate(joint_names) if name in ROBOT_ARM_JOINTS]
        metric_indices = arm_indices if arm_indices else list(range(len(joint_names)))
        metric_abs_steps = abs_steps[:, metric_indices] if len(abs_steps) and metric_indices else np.zeros((0, 0))
        metric_joint_path_motion = (
            joint_path_motion_all[metric_indices] if metric_indices else np.zeros(0, dtype=float)
        )
        metric_start_goal_delta = (
            start_goal_delta_all[metric_indices] if metric_indices else np.zeros(0, dtype=float)
        )
        total_path_motion = float(np.sum(metric_joint_path_motion))
        max_joint_path_motion = float(np.max(metric_joint_path_motion)) if len(metric_joint_path_motion) else 0.0
        max_waypoint_jump = float(np.max(metric_abs_steps)) if metric_abs_steps.size else 0.0
        max_start_goal_delta = float(np.max(metric_start_goal_delta)) if len(metric_start_goal_delta) else 0.0

        path_by_joint = {name: float(value) for name, value in zip(joint_names, joint_path_motion_all)}
        start_goal_by_joint = {name: float(value) for name, value in zip(joint_names, start_goal_delta_all)}
        reference_delta_by_joint = self._trajectory_reference_delta_by_joint(joint_names, position_array)
        wrist2_vine_guard = self._wrist2_vine_guard_metrics(joint_names, position_array)
        score = (
            float(self.get_parameter("trajectory_total_motion_weight").value) * total_path_motion
            + float(self.get_parameter("trajectory_max_joint_motion_weight").value) * max_joint_path_motion
            + float(self.get_parameter("trajectory_max_waypoint_jump_weight").value) * max_waypoint_jump
            + float(self.get_parameter("trajectory_start_goal_motion_weight").value) * max_start_goal_delta
            + float(self.get_parameter("trajectory_planning_time_weight").value) * float(planning_time)
        )
        return {
            "valid": True,
            "joint_names": joint_names,
            "point_count": len(points),
            "naturalness_metric_joints": [joint_names[index] for index in metric_indices],
            "score": float(score),
            "planning_time": float(planning_time),
            "trajectory_duration_sec": trajectory_duration_sec,
            "total_path_motion": total_path_motion,
            "max_joint_path_motion": max_joint_path_motion,
            "max_waypoint_jump": max_waypoint_jump,
            "max_start_goal_delta": max_start_goal_delta,
            "path_by_joint": path_by_joint,
            "start_goal_by_joint": start_goal_by_joint,
            "reference_delta_by_joint": reference_delta_by_joint,
            "wrist2_vine_guard": wrist2_vine_guard,
        }

    def _wrist2_vine_guard_metrics(self, joint_names: list[str], position_array: np.ndarray) -> dict[str, object]:
        if not bool(self.get_parameter("use_wrist2_vine_guard").value):
            return {"enabled": False}

        link_names = [str(value) for value in self.get_parameter("wrist2_vine_guard_links").value]
        if not link_names:
            link_names = [str(self.get_parameter("wrist2_vine_guard_link").value)]
        chains = []
        for link_name in link_names:
            try:
                chains.append((link_name, self._fk_chain_to_link(link_name)))
            except RuntimeError as exc:
                return {"enabled": True, "ok": False, "reason": str(exc), "link_name": link_name}

        vine_mid_xy = ((self.main_start + self.main_end) * 0.5)[:2]
        robot_to_vine = np.array(vine_mid_xy, dtype=float)
        norm = float(np.linalg.norm(robot_to_vine))
        if norm < 1e-9:
            return {
                "enabled": True,
                "ok": False,
                "reason": "main vine is too close to the robot origin for a directional guard",
                "link_name": link_name,
            }
        direction = robot_to_vine / norm
        vine_projection = float(np.dot(vine_mid_xy, direction))
        margin = float(self.get_parameter("wrist2_vine_guard_margin").value)
        stride = max(1, int(self.get_parameter("wrist2_vine_guard_sample_stride").value))

        sample_indices = list(range(0, len(position_array), stride))
        if not sample_indices or sample_indices[-1] != len(position_array) - 1:
            sample_indices.append(len(position_array) - 1)

        max_over = -float("inf")
        worst_projection = 0.0
        worst_index = 0
        worst_position = np.zeros(3, dtype=float)
        worst_link_name = link_names[0]
        for index in sample_indices:
            joint_positions = {name: float(value) for name, value in zip(joint_names, position_array[index])}
            for link_name, chain in chains:
                link_position = self._fk_link_position(chain, joint_positions)
                projection = float(np.dot(link_position[:2], direction))
                over = projection - vine_projection - margin
                if over > max_over:
                    max_over = over
                    worst_projection = projection
                    worst_index = int(index)
                    worst_position = link_position
                    worst_link_name = link_name

        if max_over == -float("inf"):
            max_over = 0.0

        return {
            "enabled": True,
            "ok": bool(max_over <= 0.0),
            "link_name": worst_link_name,
            "link_names": list(link_names),
            "max_over_vine": float(max_over),
            "margin": float(margin),
            "vine_projection": float(vine_projection),
            "worst_projection": float(worst_projection),
            "worst_index": int(worst_index),
            "worst_position": [float(value) for value in worst_position],
        }

    def _trajectory_reference_delta_by_joint(
        self,
        joint_names: list[str],
        position_array: np.ndarray,
    ) -> dict[str, float]:
        if "wrist_2_joint" not in joint_names or position_array.size == 0:
            return {}
        joint_index = joint_names.index("wrist_2_joint")
        fallback_reference = float(position_array[0, joint_index])
        reference = self._trajectory_reference_joint_position("wrist_2_joint", fallback_reference)
        deltas = np.abs(position_array[:, joint_index] - reference)
        return {"wrist_2_joint": float(np.max(deltas))}

    def _trajectory_reference_joint_position(self, joint_name: str, fallback: float) -> float:
        if self.has_parameter("home_joint_names") and self.has_parameter("home_joint_positions"):
            joint_names = [str(name) for name in self.get_parameter("home_joint_names").value]
            joint_positions = [float(position) for position in self.get_parameter("home_joint_positions").value]
            if joint_name in joint_names and len(joint_names) == len(joint_positions):
                return float(joint_positions[joint_names.index(joint_name)])
        return float(fallback)

    def _fk_chain_to_link(self, tip_link: str, root_link: str = "base_link") -> list[dict[str, object]]:
        cache_key = f"{root_link}->{tip_link}"
        if cache_key in self._fk_chain_cache:
            return self._fk_chain_cache[cache_key]

        repo_root = Path(str(self.get_parameter("repo_root").value)).expanduser()
        urdf_path = repo_root / "assets" / "ur5" / "urdf" / "ur5.urdf"
        if not urdf_path.exists():
            raise RuntimeError(f"UR5 URDF not found: {urdf_path}")

        robot = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
        joints_by_child: dict[str, dict[str, object]] = {}
        for joint in robot.findall("joint"):
            child = joint.find("child")
            parent = joint.find("parent")
            if child is None or parent is None:
                continue
            child_link = str(child.attrib["link"])
            joints_by_child[child_link] = {
                "name": str(joint.attrib["name"]),
                "type": str(joint.attrib.get("type", "fixed")),
                "parent": str(parent.attrib["link"]),
                "child": child_link,
                "origin": joint.find("origin"),
                "axis": joint.find("axis"),
            }

        chain: list[dict[str, object]] = []
        link = tip_link
        while link != root_link:
            if link not in joints_by_child:
                raise RuntimeError(f"Could not find URDF chain from {root_link} to {tip_link}; stopped at {link}")
            joint = joints_by_child[link]
            chain.append(joint)
            link = str(joint["parent"])
        chain.reverse()
        self._fk_chain_cache[cache_key] = chain
        return chain

    def _fk_link_position(self, chain: list[dict[str, object]], joint_positions: dict[str, float]) -> np.ndarray:
        transform = np.eye(4)
        for joint in chain:
            transform = transform @ self._urdf_origin_matrix(joint["origin"])
            if str(joint["type"]) in {"revolute", "continuous"}:
                transform = transform @ self._axis_angle_matrix(
                    self._urdf_axis_vector(joint["axis"]),
                    float(joint_positions.get(str(joint["name"]), 0.0)),
                )
        return np.array(transform[:3, 3], dtype=float)

    def _urdf_origin_matrix(self, origin) -> np.ndarray:
        if origin is None:
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
        else:
            xyz = [float(value) for value in origin.attrib.get("xyz", "0 0 0").split()]
            rpy = [float(value) for value in origin.attrib.get("rpy", "0 0 0").split()]

        transform = np.eye(4)
        transform[:3, :3] = self._rpy_matrix(*rpy)
        transform[:3, 3] = xyz
        return transform

    def _urdf_axis_vector(self, axis) -> np.ndarray:
        if axis is None:
            return np.array([0.0, 0.0, 1.0], dtype=float)
        vector = np.array([float(value) for value in axis.attrib.get("xyz", "0 0 1").split()], dtype=float)
        norm = float(np.linalg.norm(vector))
        if norm < 1e-9:
            return np.array([0.0, 0.0, 1.0], dtype=float)
        return vector / norm

    def _rpy_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        return rz @ ry @ rx

    def _axis_angle_matrix(self, axis: np.ndarray, angle: float) -> np.ndarray:
        x, y, z = axis
        c = math.cos(angle)
        s = math.sin(angle)
        one_c = 1.0 - c
        rotation = np.array(
            [
                [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
                [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
                [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
            ],
            dtype=float,
        )
        transform = np.eye(4)
        transform[:3, :3] = rotation
        return transform

    def _trajectory_naturalness_rejection_reasons(self, metrics: dict[str, object]) -> list[str]:
        if not bool(metrics.get("valid", False)):
            return [str(metrics.get("reason", "invalid trajectory"))]
        reasons: list[str] = []
        wrist2_guard = metrics.get("wrist2_vine_guard", {})
        if isinstance(wrist2_guard, dict) and bool(wrist2_guard.get("enabled", False)):
            if not bool(wrist2_guard.get("ok", False)):
                if "max_over_vine" in wrist2_guard:
                    reasons.append(
                        f"{wrist2_guard.get('link_name', 'wrist_2_link')} crossed beyond the main vine "
                        f"by {float(wrist2_guard['max_over_vine']):.4f}m "
                        f"(margin={float(wrist2_guard.get('margin', 0.0)):.4f}m)"
                    )
                else:
                    reasons.append(str(wrist2_guard.get("reason", "wrist2 vine guard failed")))

        if not bool(self.get_parameter("use_trajectory_naturalness_filter").value):
            return reasons

        max_jump = math.radians(float(self.get_parameter("max_trajectory_waypoint_jump_deg").value))
        max_joint_path = math.radians(float(self.get_parameter("max_trajectory_joint_path_deg").value))
        max_total_path = math.radians(float(self.get_parameter("max_trajectory_total_path_deg").value))
        max_start_goal = math.radians(float(self.get_parameter("max_trajectory_start_goal_joint_delta_deg").value))
        max_shoulder_pan = math.radians(float(self.get_parameter("max_shoulder_pan_path_deg").value))
        max_wrist_path = math.radians(float(self.get_parameter("max_wrist_path_deg").value))
        max_wrist_2_start_delta = math.radians(float(self.get_parameter("max_wrist_2_start_delta_deg").value))

        if float(metrics["max_waypoint_jump"]) > max_jump:
            reasons.append(
                "waypoint joint jump is too large "
                f"({math.degrees(float(metrics['max_waypoint_jump'])):.1f}deg > {math.degrees(max_jump):.1f}deg)"
            )
        if float(metrics["max_joint_path_motion"]) > max_joint_path:
            reasons.append(
                "single joint path motion is too large "
                f"({math.degrees(float(metrics['max_joint_path_motion'])):.1f}deg > "
                f"{math.degrees(max_joint_path):.1f}deg)"
            )
        if float(metrics["total_path_motion"]) > max_total_path:
            reasons.append(
                "total joint path motion is too large "
                f"({math.degrees(float(metrics['total_path_motion'])):.1f}deg > "
                f"{math.degrees(max_total_path):.1f}deg)"
            )
        if float(metrics["max_start_goal_delta"]) > max_start_goal:
            reasons.append(
                "start-goal joint delta is too large "
                f"({math.degrees(float(metrics['max_start_goal_delta'])):.1f}deg > "
                f"{math.degrees(max_start_goal):.1f}deg)"
            )

        path_by_joint = metrics.get("path_by_joint", {})
        if isinstance(path_by_joint, dict):
            shoulder_pan_path = float(path_by_joint.get("shoulder_pan_joint", 0.0))
            if shoulder_pan_path > max_shoulder_pan:
                reasons.append(
                    "shoulder_pan path motion is too large "
                    f"({math.degrees(shoulder_pan_path):.1f}deg > {math.degrees(max_shoulder_pan):.1f}deg)"
                )
            for joint_name in ("wrist_1_joint", "wrist_2_joint", "wrist_3_joint"):
                wrist_path = float(path_by_joint.get(joint_name, 0.0))
                if wrist_path > max_wrist_path:
                    reasons.append(
                        f"{joint_name} path motion is too large "
                        f"({math.degrees(wrist_path):.1f}deg > {math.degrees(max_wrist_path):.1f}deg)"
                    )
        if bool(self.get_parameter("limit_wrist_2_from_start").value):
            reference_delta_by_joint = metrics.get("reference_delta_by_joint", {})
            if isinstance(reference_delta_by_joint, dict):
                wrist_2_start_delta = float(reference_delta_by_joint.get("wrist_2_joint", 0.0))
                if wrist_2_start_delta > max_wrist_2_start_delta:
                    reasons.append(
                        "wrist_2_joint is too far from initial pose "
                        f"({math.degrees(wrist_2_start_delta):.1f}deg > "
                        f"{math.degrees(max_wrist_2_start_delta):.1f}deg)"
                    )
        return reasons

    def _trajectory_naturalness_summary(self, metrics: dict[str, object]) -> str:
        if not bool(metrics.get("valid", False)):
            return str(metrics.get("reason", "invalid trajectory"))
        path_by_joint = metrics.get("path_by_joint", {})
        reference_delta_by_joint = metrics.get("reference_delta_by_joint", {})
        shoulder = 0.0
        wrist_max = 0.0
        wrist_2_start_delta = 0.0
        if isinstance(path_by_joint, dict):
            shoulder = math.degrees(float(path_by_joint.get("shoulder_pan_joint", 0.0)))
            wrist_max = max(
                math.degrees(float(path_by_joint.get(joint_name, 0.0)))
                for joint_name in ("wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
            )
        if isinstance(reference_delta_by_joint, dict):
            wrist_2_start_delta = math.degrees(float(reference_delta_by_joint.get("wrist_2_joint", 0.0)))
        wrist2_guard = metrics.get("wrist2_vine_guard", {})
        guard_note = ""
        if isinstance(wrist2_guard, dict) and bool(wrist2_guard.get("enabled", False)):
            if "max_over_vine" in wrist2_guard:
                guard_note = (
                    f" vine_guard_link={wrist2_guard.get('link_name', 'unknown')}"
                    f" vine_guard_over={float(wrist2_guard['max_over_vine']):+.4f}m"
                )
            else:
                guard_note = " vine_guard_over=unavailable"
        return (
            f"score={float(metrics['score']):.3f} "
            f"duration={float(metrics.get('trajectory_duration_sec', 0.0)):.3f}s "
            f"points={int(metrics['point_count'])} "
            f"total_path={math.degrees(float(metrics['total_path_motion'])):.1f}deg "
            f"max_joint_path={math.degrees(float(metrics['max_joint_path_motion'])):.1f}deg "
            f"max_jump={math.degrees(float(metrics['max_waypoint_jump'])):.1f}deg "
            f"start_goal_max={math.degrees(float(metrics['max_start_goal_delta'])):.1f}deg "
            f"shoulder_pan_path={shoulder:.1f}deg "
            f"wrist_max_path={wrist_max:.1f}deg "
            f"wrist_2_from_start={wrist_2_start_delta:.1f}deg"
            f"{guard_note}"
        )

    def _grasp_precision_score(self, candidate: dict[str, object]) -> float:
        target_tolerance = max(float(self.get_parameter("target_tolerance").value), 1e-6)
        x_axis_tolerance = max(min(target_tolerance, self.tomato_radius * 0.15), 1e-6)
        axis_tolerance = max(float(self.get_parameter("rule_axis_tolerance_deg").value), 1e-6)
        blue_alignment_tolerance = max(float(self.get_parameter("blue_alignment_tolerance_deg").value), 1e-6)
        contact_weight = float(self.get_parameter("grasp_precision_contact_weight").value)
        axis_weight = float(self.get_parameter("grasp_precision_axis_weight").value)
        alignment_weight = float(self.get_parameter("grasp_precision_alignment_weight").value)

        return float(
            float(candidate.get("tool_tip_target_error", 0.0)) / target_tolerance
            + float(candidate.get("tool_tip_x_axis_offset", 0.0)) / x_axis_tolerance
            + contact_weight * float(candidate.get("calyx_contact_error", candidate.get("blue_contact_error", 0.0)))
            / target_tolerance
            + contact_weight * float(
                candidate.get("opposite_contact_error", candidate.get("orange_contact_error", 0.0))
            )
            / target_tolerance
            + axis_weight * float(candidate.get("rod_tomato_z_parallel_angle_deg", 0.0)) / axis_tolerance
            + axis_weight * float(candidate.get("gripper_tomato_y_parallel_angle_deg", 0.0)) / axis_tolerance
            + alignment_weight
            * float(candidate.get("calyx_alignment_angle_deg", candidate.get("blue_alignment_angle_deg", 0.0)))
            / blue_alignment_tolerance
        )

    def _feedback(self, feedback_msg) -> None:
        state = getattr(feedback_msg.feedback, "state", "")
        if state:
            self.get_logger().info(f"MoveIt state: {state}")

    def _pose_constraint(self, target_pose: Pose) -> Constraints:
        constraints = Constraints()
        constraints.name = "tomato_grasp_pose"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self.position_tolerance]

        region = BoundingVolume()
        region.primitives.append(primitive)
        region.primitive_poses.append(self._position_only_pose(target_pose))

        position = PositionConstraint()
        position.header.frame_id = self.base_frame
        position.link_name = self.target_link
        position.constraint_region = region
        position.weight = 1.0
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.base_frame
        orientation.link_name = self.target_link
        orientation.orientation = target_pose.orientation
        orientation.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        self._append_start_relative_joint_constraints(constraints)
        return constraints

    def _append_start_relative_joint_constraints(self, constraints: Constraints) -> None:
        if not bool(self.get_parameter("use_start_relative_pose_joint_constraints").value):
            return
        joint_positions = self._wait_for_joint_positions()
        if not joint_positions:
            self.get_logger().warn(
                "Skipping start-relative joint constraints because no /joint_states sample is available."
            )
            return

        specs = [
            (
                "shoulder_pan_joint",
                float(self.get_parameter("shoulder_pan_start_goal_constraint_deg").value),
            ),
            (
                "wrist_2_joint",
                float(self.get_parameter("wrist_2_start_goal_constraint_deg").value),
            ),
            (
                "wrist_1_joint",
                float(self.get_parameter("wrist_1_start_goal_constraint_deg").value),
            ),
        ]
        weight = float(self.get_parameter("start_relative_joint_constraint_weight").value)
        applied = []
        for joint_name, tolerance_deg in specs:
            if joint_name not in joint_positions:
                continue
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_positions[joint_name])
            tolerance_rad = math.radians(max(float(tolerance_deg), 0.0))
            joint_constraint.tolerance_above = tolerance_rad
            joint_constraint.tolerance_below = tolerance_rad
            joint_constraint.weight = weight
            constraints.joint_constraints.append(joint_constraint)
            applied.append(
                f"{joint_name}={math.degrees(joint_constraint.position):.1f}deg +/-{float(tolerance_deg):.1f}deg"
            )
        if applied:
            self.get_logger().info("Applied start-relative pose joint constraints: " + ", ".join(applied))

    def _candidate_rejection_reasons(self, candidate: dict[str, object]) -> list[str]:
        if bool(candidate.get("rule_grasp", False)):
            return self._rule_grasp_rejection_reasons(candidate)

        reasons: list[str] = []
        target_tolerance = float(self.get_parameter("target_tolerance").value)
        x_axis_tolerance = min(target_tolerance, self.tomato_radius * 0.15)
        blue_alignment_tolerance = float(self.get_parameter("blue_alignment_tolerance_deg").value)
        min_grasp_z = float(self.get_parameter("min_grasp_tf_z_above_center").value)
        min_tool_tip_z = float(self.get_parameter("min_tool_tip_z_above_center").value)
        if float(candidate["grasp_tf_z_above_center"]) < min_grasp_z:
            reasons.append(
                f"grasp_tf is below required world-Z height "
                f"({candidate['grasp_tf_z_above_center']:.4f} < {min_grasp_z:.4f})"
            )
        if float(candidate["tool_tip_z_above_center"]) < min_tool_tip_z:
            reasons.append(
                f"tool_tip is below required world-Z height "
                f"({candidate['tool_tip_z_above_center']:.4f} < {min_tool_tip_z:.4f})"
            )
        if float(candidate["tool_tip_target_error"]) > target_tolerance:
            reasons.append(
                f"tool_tip target error is too large "
                f"({candidate['tool_tip_target_error']:.4f} > {target_tolerance:.4f})"
            )
        if float(candidate["tool_tip_x_axis_offset"]) > x_axis_tolerance:
            reasons.append(
                f"tool_tip is not on tomato X axis "
                f"({candidate['tool_tip_x_axis_offset']:.4f} > {x_axis_tolerance:.4f})"
            )
        if float(candidate["calyx_alignment_angle_deg"]) > blue_alignment_tolerance:
            reasons.append(
                f"selected calyx finger is not facing the calyx "
                f"({candidate['calyx_finger']}={candidate['calyx_alignment_angle_deg']:.2f}deg > "
                f"{blue_alignment_tolerance:.2f}deg)"
            )
        if float(candidate["calyx_contact_error"]) > target_tolerance:
            reasons.append(
                f"calyx fingertip contact error is too large "
                f"({candidate['calyx_finger']}={candidate['calyx_contact_error']:.4f} > {target_tolerance:.4f})"
            )
        if float(candidate["opposite_contact_error"]) > target_tolerance:
            reasons.append(
                f"opposite fingertip contact error is too large "
                f"({candidate['opposite_finger']}={candidate['opposite_contact_error']:.4f} > {target_tolerance:.4f})"
            )
        return reasons

    def _rule_grasp_rejection_reasons(self, candidate: dict[str, object]) -> list[str]:
        reasons: list[str] = []
        target_tolerance = float(self.get_parameter("target_tolerance").value)
        axis_tolerance = float(self.get_parameter("rule_axis_tolerance_deg").value)
        blue_alignment_tolerance = float(self.get_parameter("blue_alignment_tolerance_deg").value)
        if float(candidate["rod_tomato_z_parallel_angle_deg"]) > axis_tolerance:
            reasons.append(
                f"rod is not parallel to tomato Z "
                f"({candidate['rod_tomato_z_parallel_angle_deg']:.2f}deg > {axis_tolerance:.2f}deg)"
            )
        if float(candidate["gripper_tomato_y_parallel_angle_deg"]) > axis_tolerance:
            reasons.append(
                f"gripper is not parallel to tomato Y "
                f"({candidate['gripper_tomato_y_parallel_angle_deg']:.2f}deg > {axis_tolerance:.2f}deg)"
            )
        if float(candidate["calyx_alignment_angle_deg"]) > blue_alignment_tolerance:
            reasons.append(
                f"selected calyx finger is not facing the calyx "
                f"({candidate['calyx_finger']}={candidate['calyx_alignment_angle_deg']:.2f}deg > "
                f"{blue_alignment_tolerance:.2f}deg)"
            )
        if float(candidate["calyx_contact_error"]) > target_tolerance:
            reasons.append(
                f"calyx fingertip contact error is too large "
                f"({candidate['calyx_finger']}={candidate['calyx_contact_error']:.4f} > {target_tolerance:.4f})"
            )
        if float(candidate["opposite_contact_error"]) > target_tolerance:
            reasons.append(
                f"opposite fingertip contact error is too large "
                f"({candidate['opposite_finger']}={candidate['opposite_contact_error']:.4f} > {target_tolerance:.4f})"
            )
        return reasons

    def _target_pose_candidate(self, approach_sign: float) -> dict[str, object]:
        tomato_x, tomato_y, tomato_z = self._tomato_frame_axes()
        tomato_rotation = np.column_stack((tomato_x, tomato_y, tomato_z))
        return self._tomato_rule_grasp_candidate(self.object_position, tomato_rotation, approach_sign)

    def _tomato_rule_grasp_candidate(
        self,
        object_position: np.ndarray,
        tomato_rotation: np.ndarray,
        approach_sign: float,
        gripper_spin_rad: float | None = None,
        gripper_x_sign: float = -1.0,
    ) -> dict[str, object]:
        if gripper_spin_rad is None:
            gripper_spin_rad = self.gripper_z_rot_rad
        tomato_y = tomato_rotation[:, 1]
        tomato_z = tomato_rotation[:, 2]
        gripper_x = float(gripper_x_sign) * tomato_z
        gripper_z = float(approach_sign) * tomato_y
        gripper_y = np.cross(gripper_z, gripper_x)
        gripper_y /= np.linalg.norm(gripper_y)
        gripper_z = np.cross(gripper_x, gripper_y)
        gripper_z /= np.linalg.norm(gripper_z)
        rotation = np.column_stack((gripper_x, gripper_y, gripper_z))
        object_position = np.array(object_position, dtype=float)
        target_position = object_position - gripper_z * (FINGERTIP_Z - GRASP_Z)
        pose = self._plan_pose_from_grasp(target_position, rotation, gripper_spin_rad)

        metrics = self._candidate_metrics(target_position, rotation, tomato_rotation, object_position, gripper_spin_rad)
        local_gripper_rotation = tomato_rotation.T @ rotation
        rod_parallel = abs(float(np.dot(local_gripper_rotation[:, 0], np.array([0.0, 0.0, 1.0]))))
        gripper_y_parallel = abs(float(np.dot(local_gripper_rotation[:, 2], np.array([0.0, 1.0, 0.0]))))
        rod_parallel = float(np.clip(rod_parallel, -1.0, 1.0))
        gripper_y_parallel = float(np.clip(gripper_y_parallel, -1.0, 1.0))
        return {
            "pose": pose,
            "grasp_pose": self._pose_from_position_rotation(target_position, rotation),
            "rule_grasp": True,
            "object_position": object_position,
            "tomato_rotation": tomato_rotation,
            "approach_sign": float(approach_sign),
            "gripper_x_sign": float(gripper_x_sign),
            "gripper_spin_deg": float(math.degrees(gripper_spin_rad)),
            "rod_tomato_z_parallel_angle_deg": float(math.degrees(math.acos(rod_parallel))),
            "gripper_tomato_y_parallel_angle_deg": float(math.degrees(math.acos(gripper_y_parallel))),
            **metrics,
        }

    def _plan_pose_from_grasp(
        self,
        grasp_tf_position: np.ndarray,
        gripper_rotation: np.ndarray,
        gripper_spin_rad: float | None = None,
    ) -> Pose:
        if gripper_spin_rad is None:
            gripper_spin_rad = self.gripper_z_rot_rad
        spin_link_position, spin_link_rotation = self._spin_link_pose_from_grasp(grasp_tf_position, gripper_rotation)
        if self.target_link == "tool_gripper_spin_link":
            return self._pose_from_position_rotation(spin_link_position, spin_link_rotation)
        if self.target_link == "tool_tip_link":
            tool_tip_rotation = spin_link_rotation @ self._gripper_z_spin_matrix(
                self._gripper_physical_spin_rad(gripper_spin_rad)
            ).T
            return self._pose_from_position_rotation(spin_link_position, tool_tip_rotation)
        return self._pose_from_position_rotation(grasp_tf_position, gripper_rotation)

    def _spin_link_pose_from_grasp(
        self,
        grasp_tf_position: np.ndarray,
        gripper_rotation: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        spin_link_position = grasp_tf_position - gripper_rotation @ np.array([0.0, 0.0, GRASP_Z], dtype=float)
        spin_link_rotation = gripper_rotation @ self._gripper_x_roll_matrix().T
        return spin_link_position, spin_link_rotation

    def _pose_from_position_rotation(self, position: np.ndarray, rotation: np.ndarray) -> Pose:
        qx, qy, qz, qw = quaternion_from_matrix(rotation)
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _gripper_x_roll_matrix(self) -> np.ndarray:
        c = math.cos(self.gripper_x_roll_rad)
        s = math.sin(self.gripper_x_roll_rad)
        return np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, c, -s],
                [0.0, s, c],
            ],
            dtype=float,
        )

    def _gripper_z_spin_matrix(self, angle_rad: float) -> np.ndarray:
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def _gripper_physical_spin_rad(self, logical_spin_rad: float) -> float:
        return float(logical_spin_rad) + self.gripper_spin_zero_offset_rad

    def _tool_tilt_matrix(self) -> np.ndarray:
        cx = math.cos(self.tool_tilt_x_rad)
        sx = math.sin(self.tool_tilt_x_rad)
        cy = math.cos(self.tool_tilt_y_rad)
        sy = math.sin(self.tool_tilt_y_rad)
        rx = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, cx, -sx],
                [0.0, sx, cx],
            ],
            dtype=float,
        )
        ry = np.array(
            [
                [cy, 0.0, sy],
                [0.0, 1.0, 0.0],
                [-sy, 0.0, cy],
            ],
            dtype=float,
        )
        return ry @ rx

    def _tool_tip_offset_from_tool0(self) -> np.ndarray:
        straight_len = self.tool_length * self.tool_bend_straight_ratio
        slanted_len = self.tool_length - straight_len
        tool_tip_x = straight_len + slanted_len * math.cos(self.tool_tilt_y_rad)
        tool_tip_z = -slanted_len * math.sin(self.tool_tilt_y_rad)
        under_offset = self.tool_radius * 5.0 * self.tool_gripper_under_offset_ratio
        local_z = self._tool_tilt_matrix()[:, 2]
        return np.array([tool_tip_x, 0.0, tool_tip_z], dtype=float) + local_z * under_offset

    def _candidate_metrics(
        self,
        grasp_tf_position: np.ndarray,
        gripper_rotation: np.ndarray,
        tomato_rotation: np.ndarray,
        object_position: np.ndarray | None = None,
        gripper_spin_rad: float | None = None,
    ) -> dict[str, object]:
        if gripper_spin_rad is None:
            gripper_spin_rad = self.gripper_z_rot_rad
        if object_position is None:
            object_position = self.object_position
        object_position = np.array(object_position, dtype=float)
        tool_tip_offset = np.array([0.0, 0.0, -GRASP_Z], dtype=float)
        tool_tip_position = grasp_tf_position + gripper_rotation @ tool_tip_offset
        tool_tip_local = tomato_rotation.T @ (tool_tip_position - object_position)
        tool_tip_target_local = np.array([tool_tip_local[0], 0.0, 0.0], dtype=float)
        tool_tip_target_position = object_position + tomato_rotation @ tool_tip_target_local

        blue_tip_offset, orange_tip_offset = self._grasp_tf_to_inner_fingertip_offsets()
        blue_tip_position = grasp_tf_position + gripper_rotation @ blue_tip_offset
        orange_tip_position = grasp_tf_position + gripper_rotation @ orange_tip_offset

        contact_half_span = self._tomato_contact_half_span()
        calyx_contact_target = object_position + tomato_rotation @ np.array([0.0, 0.0, contact_half_span])
        opposite_contact_target = object_position + tomato_rotation @ np.array([0.0, 0.0, -contact_half_span])
        local_gripper_rotation = tomato_rotation.T @ gripper_rotation
        blue_alignment = float(np.dot(-local_gripper_rotation[:, 0], np.array([0.0, 0.0, 1.0])))
        blue_alignment = float(np.clip(blue_alignment, -1.0, 1.0))
        orange_alignment = float(np.dot(local_gripper_rotation[:, 0], np.array([0.0, 0.0, 1.0])))
        orange_alignment = float(np.clip(orange_alignment, -1.0, 1.0))
        blue_alignment_angle = float(math.degrees(math.acos(blue_alignment)))
        orange_alignment_angle = float(math.degrees(math.acos(orange_alignment)))
        blue_calyx_contact_error = float(np.linalg.norm(blue_tip_position - calyx_contact_target))
        orange_opposite_contact_error = float(np.linalg.norm(orange_tip_position - opposite_contact_target))
        orange_calyx_contact_error = float(np.linalg.norm(orange_tip_position - calyx_contact_target))
        blue_opposite_contact_error = float(np.linalg.norm(blue_tip_position - opposite_contact_target))
        target_tolerance = max(float(self.get_parameter("target_tolerance").value), 1e-6)
        alignment_tolerance = max(float(self.get_parameter("blue_alignment_tolerance_deg").value), 1e-6)
        blue_calyx_score = max(
            blue_calyx_contact_error / target_tolerance,
            orange_opposite_contact_error / target_tolerance,
            blue_alignment_angle / alignment_tolerance,
        )
        orange_calyx_score = max(
            orange_calyx_contact_error / target_tolerance,
            blue_opposite_contact_error / target_tolerance,
            orange_alignment_angle / alignment_tolerance,
        )
        if orange_calyx_score < blue_calyx_score:
            calyx_finger = "orange"
            opposite_finger = "blue"
            calyx_contact_error = orange_calyx_contact_error
            opposite_contact_error = blue_opposite_contact_error
            calyx_alignment_angle = orange_alignment_angle
            calyx_assignment_score = orange_calyx_score
        else:
            calyx_finger = "blue"
            opposite_finger = "orange"
            calyx_contact_error = blue_calyx_contact_error
            opposite_contact_error = orange_opposite_contact_error
            calyx_alignment_angle = blue_alignment_angle
            calyx_assignment_score = blue_calyx_score
        tool_gripper_link_position, tool_tip_rotation = self._spin_link_pose_from_grasp(
            grasp_tf_position,
            gripper_rotation,
        )
        tool_tip_rotation = tool_tip_rotation @ self._gripper_z_spin_matrix(
            self._gripper_physical_spin_rad(gripper_spin_rad)
        ).T
        tool0_rotation = tool_tip_rotation @ self._tool_tilt_matrix().T
        tool0_position = tool_gripper_link_position - tool0_rotation @ self._tool_tip_offset_from_tool0()

        return {
            "tool_tip_position": tool_tip_position,
            "tool0_position": tool0_position,
            "tool_gripper_link_position": tool_gripper_link_position,
            "tool_tip_target_position": tool_tip_target_position,
            "tool0_z_minus_gripper_link_z": float(tool0_position[2] - tool_gripper_link_position[2]),
            "tool_tip_z_above_center": float(tool_tip_position[2] - object_position[2]),
            "grasp_tf_z_above_center": float(grasp_tf_position[2] - object_position[2]),
            "tool_tip_target_error": float(np.linalg.norm(tool_tip_position - tool_tip_target_position)),
            "tool_tip_x_axis_offset": float(np.linalg.norm(tool_tip_local[1:])),
            "blue_contact_error": blue_calyx_contact_error,
            "orange_contact_error": orange_opposite_contact_error,
            "blue_opposite_contact_error": blue_opposite_contact_error,
            "orange_calyx_contact_error": orange_calyx_contact_error,
            "blue_alignment": blue_alignment,
            "orange_alignment": orange_alignment,
            "blue_alignment_angle_deg": blue_alignment_angle,
            "orange_alignment_angle_deg": orange_alignment_angle,
            "calyx_finger": calyx_finger,
            "opposite_finger": opposite_finger,
            "calyx_contact_error": calyx_contact_error,
            "opposite_contact_error": opposite_contact_error,
            "calyx_alignment_angle_deg": calyx_alignment_angle,
            "calyx_assignment_score": calyx_assignment_score,
        }

    def _grasp_tf_to_inner_fingertip_offsets(self) -> tuple[np.ndarray, np.ndarray]:
        half_opening = self.gripper_opening * 0.5
        cap_inner_half_width = FINGER_THICKNESS * 0.8
        tip_z_from_grasp_tf = FINGERTIP_Z - GRASP_Z
        blue_tip = np.array([-half_opening + cap_inner_half_width, 0.0, tip_z_from_grasp_tf], dtype=float)
        orange_tip = np.array([half_opening - cap_inner_half_width, 0.0, tip_z_from_grasp_tf], dtype=float)
        return blue_tip, orange_tip

    def _tomato_contact_half_span(self) -> float:
        blue_tip, orange_tip = self._grasp_tf_to_inner_fingertip_offsets()
        gripper_half_span = float((orange_tip[0] - blue_tip[0]) * 0.5)
        return min(gripper_half_span, self.tomato_radius * 1.06)

    def _position_only_pose(self, source: Pose) -> Pose:
        pose = Pose()
        pose.position = source.position
        pose.orientation.w = 1.0
        return pose

    def _scene_translation(self) -> np.ndarray:
        return np.array([0.0, -float(getattr(self, "robot_y_offset", 0.0)), 0.0], dtype=float)

    def _scene_position(self, position) -> np.ndarray:
        return np.array(position, dtype=float) + self._scene_translation()

    def _vine_points(self, object_position=None, apply_scene_offset: bool = True):
        if object_position is None:
            object_position = self.base_object_position
        object_position = np.array(object_position, dtype=float)
        if apply_scene_offset:
            object_position = self._scene_position(object_position)
        main_start = object_position + np.array(
            [-self.object_radius * 0.75, self.object_radius * 1.35, -self.object_radius * 6.00]
        )
        main_end = object_position + np.array(
            [-self.object_radius * 0.35, self.object_radius * 1.15, self.object_radius * 6.80]
        )
        return main_start, main_end

    def _main_vine_axis(self) -> np.ndarray:
        axis = self.main_end - self.main_start
        axis /= np.linalg.norm(axis)
        return axis

    def _main_vine_spin_matrix(self, z_spin_deg=None) -> np.ndarray:
        axis = self._main_vine_axis()
        angle = math.radians(self.tomato_z_spin_deg if z_spin_deg is None else float(z_spin_deg))
        c = math.cos(angle)
        s = math.sin(angle)
        axis_cross = np.array(
            [
                [0.0, -axis[2], axis[1]],
                [axis[2], 0.0, -axis[0]],
                [-axis[1], axis[0], 0.0],
            ],
            dtype=float,
        )
        return c * np.eye(3) + s * axis_cross + (1.0 - c) * np.outer(axis, axis)

    def _spin_vector_around_main_vine(self, vector, z_spin_deg=None) -> np.ndarray:
        return self._main_vine_spin_matrix(z_spin_deg) @ np.array(vector, dtype=float)

    def _spin_point_around_main_vine(self, point, z_spin_deg=None) -> np.ndarray:
        point = np.array(point, dtype=float)
        return self.main_start + self._spin_vector_around_main_vine(point - self.main_start, z_spin_deg)

    def _point_on_segment_at_z(self, start, end, z_value):
        start = np.array(start, dtype=float)
        end = np.array(end, dtype=float)
        z_delta = float(end[2] - start[2])
        if abs(z_delta) < 1e-6:
            return (start + end) * 0.5
        t = float(np.clip((z_value - start[2]) / z_delta, 0.0, 1.0))
        return (1.0 - t) * start + t * end

    def _branch_start_for_down_angle(self, main_start, main_end, tomato_pos, down_angle_deg):
        start = np.array(main_start, dtype=float)
        end = np.array(main_end, dtype=float)
        tomato_pos = np.array(tomato_pos, dtype=float)
        z_min = float(min(start[2], end[2]))
        z_max = float(max(start[2], end[2]))
        slope = math.tan(math.radians(float(down_angle_deg)))

        def residual(z_value: float) -> float:
            point = self._point_on_segment_at_z(start, end, z_value)
            horizontal_distance = float(np.linalg.norm(point[:2] - tomato_pos[:2]))
            return float(point[2] - tomato_pos[2] - horizontal_distance * slope)

        low = max(z_min, float(tomato_pos[2]))
        high = z_max
        if residual(low) >= 0.0:
            return self._point_on_segment_at_z(start, end, low)
        if residual(high) <= 0.0:
            return self._point_on_segment_at_z(start, end, high)
        for _ in range(32):
            mid = (low + high) * 0.5
            if residual(mid) < 0.0:
                low = mid
            else:
                high = mid
        return self._point_on_segment_at_z(start, end, (low + high) * 0.5)

    def _is_left_of_vine_from_robot(self, branch_anchor, tomato_pos) -> bool:
        anchor_xy = np.array(branch_anchor[:2], dtype=float)
        tomato_xy = np.array(tomato_pos[:2], dtype=float)
        view_forward = anchor_xy.copy()
        if float(np.linalg.norm(view_forward)) < 1e-6:
            view_forward = tomato_xy.copy()
        if float(np.linalg.norm(view_forward)) < 1e-6:
            return False
        view_forward /= np.linalg.norm(view_forward)
        robot_left = np.array([-view_forward[1], view_forward[0]], dtype=float)
        tomato_side = tomato_xy - anchor_xy
        return float(np.dot(tomato_side, robot_left)) > 0.0

    def _local_tomato_z_spin_deg(self, branch_start, tomato_pos) -> float:
        z_spin_deg = self.taught_tomato_z_spin_deg
        if self._is_left_of_vine_from_robot(branch_start, tomato_pos):
            z_spin_deg += float(getattr(self, "left_tomato_z_spin_offset_deg", 0.0))
        else:
            z_spin_deg += float(getattr(self, "right_tomato_z_spin_offset_deg", 0.0))
        return z_spin_deg

    def _individual_stem_spin_offset_deg(self, tomato_index: int, row_index: int = 0) -> float:
        if not bool(getattr(self, "randomize_individual_stem_spin", False)):
            return 0.0
        lower = float(getattr(self, "individual_stem_spin_min_deg", 0.0))
        upper = float(getattr(self, "individual_stem_spin_max_deg", 360.0))
        if upper < lower:
            lower, upper = upper, lower
        seed = int(getattr(self, "individual_stem_spin_seed", 0))
        raw = math.sin(
            (seed + 1) * 12.9898
            + (int(row_index) + 1) * 37.719
        ) * 43758.5453123
        fraction = raw - math.floor(raw)
        return lower + (upper - lower) * fraction

    def _individual_main_vine_spin_deg(self, tomato_index: int, row_index: int = 0) -> float:
        return float(self.tomato_z_spin_deg) + self._individual_stem_spin_offset_deg(tomato_index, row_index)

    def _tomato_frame_axes(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self._spun_tomato_frame_axes_for(
            self.base_stem_axis,
            self.target_z_spin_deg,
            self.target_main_vine_spin_deg,
        )

    def _spun_tomato_frame_axes_for(
        self,
        base_stem_axis,
        z_spin_deg,
        main_vine_spin_deg=None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        base_axes = np.column_stack(self._tomato_frame_axes_for(base_stem_axis, z_spin_deg))
        spun_axes = self._main_vine_spin_matrix(main_vine_spin_deg) @ base_axes
        return spun_axes[:, 0], spun_axes[:, 1], spun_axes[:, 2]

    def _tomato_frame_axes_for(self, stem_axis, z_spin_deg) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        tomato_z_axis = np.array(stem_axis, dtype=float)
        tomato_z_axis /= np.linalg.norm(tomato_z_axis)
        x_hint = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(tomato_z_axis, x_hint))) > 0.95:
            x_hint = np.array([0.0, 1.0, 0.0], dtype=float)
        tomato_y_axis = np.cross(tomato_z_axis, x_hint)
        tomato_y_axis /= np.linalg.norm(tomato_y_axis)
        tomato_x_axis = np.cross(tomato_y_axis, tomato_z_axis)
        tomato_x_axis /= np.linalg.norm(tomato_x_axis)

        spin_rad = math.radians(float(z_spin_deg))
        spun_x_axis = math.cos(spin_rad) * tomato_x_axis + math.sin(spin_rad) * tomato_y_axis
        spun_y_axis = -math.sin(spin_rad) * tomato_x_axis + math.cos(spin_rad) * tomato_y_axis
        spun_x_axis /= np.linalg.norm(spun_x_axis)
        spun_y_axis /= np.linalg.norm(spun_y_axis)
        return spun_x_axis, spun_y_axis, tomato_z_axis


def main() -> None:
    rclpy.init()
    node = MoveItGraspPlanner()
    try:
        success = node.plan()
        if not success:
            raise SystemExit(1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
