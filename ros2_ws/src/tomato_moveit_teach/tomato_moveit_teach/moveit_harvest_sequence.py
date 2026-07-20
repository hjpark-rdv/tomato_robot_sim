import copy
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.srv import GetCartesianPath
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from tomato_moveit_teach.geometry import quaternion_from_matrix, target_stem_axis
from tomato_moveit_teach.moveit_grasp_planner import MoveItGraspPlanner
from tomato_moveit_teach.trajectory_dataset import predict_good_probability, read_json


def matrix_from_quaternion(quaternion) -> np.ndarray:
    x, y, z, w = [float(value) for value in quaternion]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def transform_matrix(position, quaternion) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = matrix_from_quaternion(quaternion)
    transform[:3, 3] = np.array(position, dtype=float)
    return transform


def pose_from_matrix(transform: np.ndarray) -> Pose:
    qx, qy, qz, qw = quaternion_from_matrix(transform[:3, :3])
    pose = Pose()
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


class MoveItHarvestSequence(MoveItGraspPlanner):
    def __init__(self) -> None:
        super().__init__(node_name="moveit_harvest_sequence")
        self.declare_parameter("use_taught_relative_grasp", True)
        self.declare_parameter("relative_translation", [0.009, -0.022, 0.0])
        self.declare_parameter("relative_rotation_xyzw", [-0.519, 0.484, 0.494, 0.502])
        self.declare_parameter("side", "both")
        self.declare_parameter("include_target_tomato", True)
        self.declare_parameter("target_tomato_index", -1)
        self.declare_parameter("start_rank", 1)
        self.declare_parameter("max_tomatoes", 0)
        self.declare_parameter("continue_on_failure", True)
        self.declare_parameter("allow_empty_harvest_targets", False)
        self.declare_parameter("plan_candidates_only", False)
        self.declare_parameter("preplan_visible_harvest_trajectories", False)
        self.declare_parameter("preplan_reset_gripper_spin_before_each_target", True)
        self.declare_parameter("preplan_execute_partial_success", True)
        self.declare_parameter("stop_after_grasp_pose", False)
        self.declare_parameter("post_grasp_position_tolerance", 0.004)
        self.declare_parameter("post_grasp_orientation_tolerance_deg", 2.0)
        self.declare_parameter("post_grasp_tf_timeout", 2.0)
        self.declare_parameter("clear_target_tomato_collision", True)
        self.declare_parameter("allow_target_tomato_touch", True)
        self.declare_parameter("attach_tomato_visual_after_grasp", True)
        self.declare_parameter("target_collision_id", "target_tomato")
        self.declare_parameter("use_right_rule_grasp_pose", True)
        self.declare_parameter("use_side_spin_rule_grasp_pose", True)
        self.declare_parameter("right_gripper_spin_deg", 0.0)
        self.declare_parameter("left_gripper_spin_deg", 0.0)
        self.declare_parameter("use_gripper_spin_search", True)
        self.declare_parameter("planner_selects_gripper_spin", False)
        self.declare_parameter("gripper_spin_step_deg", 30.0)
        self.declare_parameter("min_gripper_spin_deg", -180.0)
        self.declare_parameter("max_gripper_spin_deg", 180.0)
        self.declare_parameter("min_grasp_gripper_spin_deg", -90.0)
        self.declare_parameter("max_grasp_gripper_spin_deg", 90.0)
        self.declare_parameter("grasp_gripper_spin_limit_tolerance_deg", 10.0)
        self.declare_parameter("grasp_gripper_spin_final_tolerance_deg", 0.5)
        self.declare_parameter("max_gripper_spin_candidates", 0)
        self.declare_parameter("force_gripper_x_sign", 0.0)
        self.declare_parameter("twist_gripper_after_grasp", True)
        self.declare_parameter("grasp_twist_direction_mode", "tomato_z_to_y")
        self.declare_parameter("grasp_twist_angle_deg", 90.0)
        self.declare_parameter("grasp_twist_segment_duration_sec", 0.25)
        self.declare_parameter("grasp_twist_steps_per_segment", 8)
        self.declare_parameter("select_best_trajectory_candidate", True)
        self.declare_parameter("force_valid_candidate_rank", 0)
        self.declare_parameter("best_trajectory_selection_metric", "score")
        self.declare_parameter("trajectory_selector_model_path", "")
        self.declare_parameter("plan_retries_per_grasp_candidate", 8)
        self.declare_parameter("use_excellent_trajectory_early_exit", False)
        self.declare_parameter("excellent_trajectory_score_threshold", 15.0)
        self.declare_parameter("grasp_diagnostics_output_path", "")
        self.declare_parameter("grasp_diagnostics_include_all_valid_candidates", True)
        self.declare_parameter("use_cartesian_grasp_approach", False)
        self.declare_parameter("cartesian_approach_mode", "direct")
        self.declare_parameter("cartesian_full_path_from_current", True)
        self.declare_parameter("cartesian_pregrasp_distance", 0.06)
        self.declare_parameter("cartesian_top_view_lane_distance", 0.12)
        self.declare_parameter("cartesian_top_view_diagonal_distance", 0.12)
        self.declare_parameter("cartesian_top_view_diagonal_lane_extra", 0.05)
        self.declare_parameter("cartesian_top_view_lane_sign", 1.0)
        self.declare_parameter("cartesian_top_view_a_fraction", 0.5)
        self.declare_parameter("cartesian_top_view_c_pre_distance", 0.02)
        self.declare_parameter("cartesian_top_view_reference_point", "tool_tip_link")
        self.declare_parameter("cartesian_stop_at_c_pre", False)
        self.declare_parameter("cartesian_max_step", 0.005)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("cartesian_prismatic_jump_threshold", 0.0)
        self.declare_parameter("cartesian_revolute_jump_threshold", 0.0)
        self.declare_parameter("cartesian_avoid_collisions", True)
        self.declare_parameter("cartesian_lock_gripper_spin", False)
        self.declare_parameter("cartesian_fixed_gripper_spin_tolerance_deg", 0.5)
        self.declare_parameter("cartesian_fraction_threshold", 0.95)
        self.declare_parameter("cartesian_service_timeout_sec", 8.0)
        self.declare_parameter("cartesian_step_duration_sec", 0.05)
        self.declare_parameter("cartesian_slow_b_to_c", True)
        self.declare_parameter("cartesian_b_to_c_step_duration_sec", 0.12)
        self.declare_parameter("cartesian_current_pose_timeout_sec", 0.6)
        self.declare_parameter("cartesian_fallback_to_joint_grasp", False)
        self.declare_parameter("joint_plan_goal_pose", "cartesian_start")
        self.declare_parameter("return_home_on_start", True)
        self.declare_parameter("return_home_on_finish", True)
        self.declare_parameter("return_home_via_reverse_grasp_path", True)
        self.declare_parameter("reverse_grasp_path_fallback_to_planned_home", True)
        self.declare_parameter("return_home_before_each_target", True)
        self.declare_parameter("home_plan_retries", 8)
        self.declare_parameter(
            "home_joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
                "tool_bend_joint",
                "tool_gripper_z_joint",
            ],
        )
        self.declare_parameter(
            "home_joint_positions",
            [
                -1.4379491029641744,
                -1.1907401577003016,
                -2.116742584205267,
                -1.3810410438314462,
                1.5765461198564021,
                -8.504489148035646e-05,
                0.6673960827876584,
                0.035107627975484376,
            ],
        )
        self.declare_parameter("home_joint_tolerance", 0.01)
        self.declare_parameter("home_gripper_spin_deg", 2.0115189117106738)
        self.declare_parameter("use_direct_home_trajectory", True)
        self.declare_parameter("direct_home_duration_sec", 2.0)
        self.declare_parameter("disable_tomato_collision_for_rule_test", True)
        self.declare_parameter("sync_scene_on_start", True)
        self.declare_parameter("reset_harvested_tomatoes_on_start", True)
        self.declare_parameter("scene_update_wait_sec", 1.2)
        self.declare_parameter("show_vine_row", True)
        self.declare_parameter("use_nearest_vine_row_for_harvest", True)
        self.declare_parameter("vine_row_start_y", 0.0)
        self.declare_parameter("vine_row_end_y", 5.0)
        self.declare_parameter("vine_row_spacing_y", 0.5)
        self.declare_parameter("harvest_camera_visible_only", False)
        self.declare_parameter("harvest_camera_frame", "tool_camera_depth_optical_frame")
        self.declare_parameter("harvest_camera_horizontal_fov_deg", 87.0)
        self.declare_parameter("harvest_camera_vertical_fov_deg", 58.0)
        self.declare_parameter("harvest_camera_near_m", 0.03)
        self.declare_parameter("harvest_camera_far_m", 1.20)
        self.declare_parameter("harvest_camera_visibility_timeout_sec", 2.0)
        self.declare_parameter("sync_robot_workspace_box_on_start", False)
        self.declare_parameter("include_robot_workspace_box_collision", False)
        self.declare_parameter("robot_workspace_box_rear_x", -0.50)
        self.declare_parameter("robot_workspace_box_front_x", 0.62)
        self.declare_parameter("robot_workspace_box_half_y", 0.50)
        self.declare_parameter("robot_workspace_box_ceiling_z", 0.80)
        self.declare_parameter("robot_workspace_box_wall_thickness", 0.040)
        self.declare_parameter("robot_workspace_box_height", 1.20)
        self.declare_parameter("tomato_scene_set_parameters_service", "/tomato_scene_node/set_parameters")
        self.declare_parameter("mobile_base_set_parameters_service", "/mobile_base_tf_publisher/set_parameters")
        self.declare_parameter("joint_state_set_parameters_service", "/home_joint_state_publisher/set_parameters")
        self.declare_parameter(
            "target_touch_links",
            [
                "tool_gripper_spin_link",
                "tool_gripper_link",
                "tool_rod_link",
                "tool_bent_rod_link",
                "tool_tip_link",
                "grasp_tf",
                "wrist_3_link",
            ],
        )
        self.declare_parameter("remove_target_tomato_service", "remove_target_tomato")
        self.left_tomato_z_spin_offset_deg = float(
            self.get_parameter("left_tomato_z_spin_offset_deg").value
        )
        self.target_z_spin_deg = self._local_tomato_z_spin_deg(self.branch_start, self.base_object_position)
        self.side = str(self.get_parameter("side").value).lower()
        self.include_target_tomato = bool(self.get_parameter("include_target_tomato").value)
        self.show_vine_row = bool(self.get_parameter("show_vine_row").value)
        self.use_nearest_vine_row_for_harvest = bool(
            self.get_parameter("use_nearest_vine_row_for_harvest").value
        )
        self.vine_row_start_y = float(self.get_parameter("vine_row_start_y").value)
        self.vine_row_end_y = float(self.get_parameter("vine_row_end_y").value)
        self.vine_row_spacing_y = float(self.get_parameter("vine_row_spacing_y").value)
        self.harvested_tomato_ids: set[str] = set()
        self.current_gripper_spin_deg = self._clamp_gripper_spin_deg(
            float(self.get_parameter("home_gripper_spin_deg").value)
        )
        self.last_grasp_record: dict[str, object] | None = None
        self._trajectory_selector_model: dict[str, object] | None = None
        self._preplan_collect_selected_record = False
        self._preplan_last_selected_record: dict[str, object] | None = None
        self._preplan_last_valid_records: list[dict[str, object]] = []
        self._preplan_last_selection_reason = ""
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.get_scene_client = self.create_client(GetPlanningScene, "get_planning_scene")
        self.cartesian_path_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_param_client = self.create_client(
            SetParameters,
            str(self.get_parameter("tomato_scene_set_parameters_service").value),
        )
        scene_set_service = str(self.get_parameter("tomato_scene_set_parameters_service").value)
        scene_get_service = (
            scene_set_service[: -len("/set_parameters")] + "/get_parameters"
            if scene_set_service.endswith("/set_parameters")
            else "/tomato_scene_node/get_parameters"
        )
        self.scene_get_param_client = self.create_client(GetParameters, scene_get_service)
        self.mobile_base_param_client = self.create_client(
            SetParameters,
            str(self.get_parameter("mobile_base_set_parameters_service").value),
        )
        self.joint_state_param_client = self.create_client(
            SetParameters,
            str(self.get_parameter("joint_state_set_parameters_service").value),
        )
        self.planning_scene_pub = self.create_publisher(PlanningScene, "planning_scene", 10)
        self.remove_target_client = self.create_client(
            Trigger,
            str(self.get_parameter("remove_target_tomato_service").value),
        )

    def run_sequence(self) -> bool:
        plan_candidates_only = bool(self.get_parameter("plan_candidates_only").value)
        self.execute = not plan_candidates_only
        if bool(self.get_parameter("reset_harvested_tomatoes_on_start").value):
            self.harvested_tomato_ids.clear()
        else:
            loaded_harvested_ids = self._load_scene_harvested_tomato_ids()
            if loaded_harvested_ids is None:
                return False
            self.harvested_tomato_ids = loaded_harvested_ids
        if bool(self.get_parameter("sync_scene_on_start").value):
            self._sync_scene_for_current_run()

        if bool(self.get_parameter("return_home_on_start").value) and not plan_candidates_only:
            if not self._move_to_home_pose("start"):
                return False

        if bool(self.get_parameter("use_taught_relative_grasp").value):
            return self._run_taught_relative_harvest_sequence()

        if self._approach_to_grasp() is None:
            if bool(self.get_parameter("return_home_on_finish").value):
                self._move_to_home_pose("finish_after_failed_grasp")
            return False
        if not self._complete_grasp_by_returning_home_then_harvest("target_tomato"):
            return False
        self.get_logger().info("Harvest sequence complete after returning home with the grasped tomato.")
        return True

    def _move_to_home_pose(self, phase: str, reset_gripper_spin: bool = True) -> bool:
        self.get_logger().info(f"Moving to initial home pose at harvest {phase}.")
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("MoveIt action server was not found. Cannot move to home pose.")
            return False

        joint_names = [str(name) for name in self.get_parameter("home_joint_names").value]
        joint_positions = [float(position) for position in self.get_parameter("home_joint_positions").value]
        if len(joint_names) != len(joint_positions):
            self.get_logger().error(
                "home_joint_names and home_joint_positions must have the same length "
                f"({len(joint_names)} != {len(joint_positions)})."
            )
            return False

        home_joints = dict(zip(joint_names, joint_positions))
        if reset_gripper_spin:
            home_gripper_spin_deg = self._clamp_gripper_spin_deg(float(self.get_parameter("home_gripper_spin_deg").value))
            gripper_spin_note = f"{home_gripper_spin_deg:.1f}deg"
            if "tool_gripper_z_joint" in home_joints:
                home_joints["tool_gripper_z_joint"] = math.radians(home_gripper_spin_deg)
            self._set_runtime_gripper_spin_deg(home_gripper_spin_deg)
        else:
            gripper_spin_note = f"preserve_current({self.current_gripper_spin_deg:.1f}deg)"
            if "tool_gripper_z_joint" in home_joints:
                home_joints["tool_gripper_z_joint"] = math.radians(self.current_gripper_spin_deg)
        joint_positions = [float(home_joints[name]) for name in joint_names]
        self.get_logger().info(
            "Home joint target "
            + ", ".join(f"{name}={position:.4f}rad" for name, position in home_joints.items())
            + f", gripper_spin={gripper_spin_note}"
        )

        if bool(self.get_parameter("use_direct_home_trajectory").value):
            return self._execute_joint_positions_direct(
                joint_names,
                joint_positions,
                float(self.get_parameter("direct_home_duration_sec").value),
                label=f"home_{phase}",
            )

        valid_records: list[tuple[dict[str, object], dict[str, object], int]] = []
        home_plan_retries = max(1, int(self.get_parameter("home_plan_retries").value))
        for retry_index in range(1, home_plan_retries + 1):
            plan_result = self._move_group_to_joint_positions(
                home_joints,
                tolerance=float(self.get_parameter("home_joint_tolerance").value),
                plan_only=True,
            )
            if not bool(plan_result["success"]):
                self.get_logger().warn(
                    f"Could not plan home trajectory at harvest {phase} "
                    f"retry={retry_index}/{home_plan_retries}."
                )
                continue

            trajectory_metrics = self._trajectory_naturalness_metrics(
                plan_result["trajectory"],
                float(plan_result["planning_time"]),
            )
            self.get_logger().info(
                f"Home trajectory candidate phase={phase} retry={retry_index}/{home_plan_retries} "
                f"{self._trajectory_naturalness_summary(trajectory_metrics)}"
            )
            rejection_reasons = self._trajectory_naturalness_rejection_reasons(trajectory_metrics)
            if rejection_reasons:
                self.get_logger().warn("Skipping home trajectory because " + "; ".join(rejection_reasons))
                continue
            valid_records.append((plan_result, trajectory_metrics, retry_index))

        if not valid_records:
            self.get_logger().error(f"Could not plan a valid home trajectory at harvest {phase}.")
            return False

        plan_result, trajectory_metrics, retry_index = min(
            valid_records,
            key=lambda record: float(record[1]["score"]),
        )
        self.get_logger().info(
            f"Selected home trajectory phase={phase} retry={retry_index}/{home_plan_retries} "
            f"{self._trajectory_naturalness_summary(trajectory_metrics)} "
            f"from {len(valid_records)} valid candidate(s)"
        )

        if not self.execute:
            return True
        return self._execute_joint_trajectory(plan_result["trajectory"])

    def _sync_scene_for_current_run(self) -> None:
        parameters = [
            self._scene_double_parameter("tomato_z_spin_deg", self.tomato_z_spin_deg),
            self._scene_double_parameter("taught_tomato_z_spin_deg", self.taught_tomato_z_spin_deg),
            self._scene_double_parameter("right_tomato_z_spin_offset_deg", self.right_tomato_z_spin_offset_deg),
            self._scene_double_parameter("left_tomato_z_spin_offset_deg", self.left_tomato_z_spin_offset_deg),
            self._scene_bool_parameter("randomize_individual_stem_spin", self.randomize_individual_stem_spin),
            self._scene_double_parameter("individual_stem_spin_seed", self.individual_stem_spin_seed),
            self._scene_double_parameter("individual_stem_spin_min_deg", self.individual_stem_spin_min_deg),
            self._scene_double_parameter("individual_stem_spin_max_deg", self.individual_stem_spin_max_deg),
            self._scene_double_parameter("scene_y_offset", self.scene_y_offset),
            self._scene_double_parameter("robot_y_offset", self.robot_y_offset),
            self._scene_double_parameter("tomato_radius_scale", self.tomato_radius_scale),
            self._scene_bool_parameter("show_vine_row", self.show_vine_row),
            self._scene_double_parameter("vine_row_start_y", self.vine_row_start_y),
            self._scene_double_parameter("vine_row_end_y", self.vine_row_end_y),
            self._scene_double_parameter("vine_row_spacing_y", self.vine_row_spacing_y),
            self._scene_string_parameter("attached_tomato_id", ""),
            self._scene_string_array_parameter("harvested_tomato_ids", self.harvested_tomato_ids),
            self._scene_string_array_parameter("suppressed_collision_ids", self._base_suppressed_collision_ids()),
        ]
        self._set_mobile_base_robot_offset()
        if bool(self.get_parameter("sync_robot_workspace_box_on_start").value):
            parameters.extend(self._robot_workspace_box_scene_parameters())
        if not self._set_scene_parameters(parameters, timeout_sec=5.0):
            self.get_logger().warn(
                "tomato_scene_node was not synchronized; RViz/planning scene may still show the previous tomato angle."
            )
            return

        wait_sec = float(self.get_parameter("scene_update_wait_sec").value)
        self.get_logger().info(
            "Synchronized tomato_scene_node for this harvest run: "
            f"tomato_z_spin_deg={self.tomato_z_spin_deg:.1f}, "
            f"taught_tomato_z_spin_deg={self.taught_tomato_z_spin_deg:.1f}, "
            f"right_tomato_z_spin_offset_deg={self.right_tomato_z_spin_offset_deg:.1f}, "
            f"left_tomato_z_spin_offset_deg={self.left_tomato_z_spin_offset_deg:.1f}, "
            f"individual_stem_spin={'on' if self.randomize_individual_stem_spin else 'off'}, "
            f"individual_stem_spin_seed={self.individual_stem_spin_seed}, "
            f"robot_y_offset={self.robot_y_offset:.3f}, "
            f"harvest_vine_row_y={self._nearest_vine_row_offset():.3f}, "
            f"tomato_radius_scale={self.tomato_radius_scale:.2f}, "
            f"robot_workspace_box={'on' if bool(self.get_parameter('include_robot_workspace_box_collision').value) else 'off'}, "
            f"waiting {wait_sec:.1f}s for RViz/PlanningScene update"
        )
        deadline = time.monotonic() + max(wait_sec, 0.0)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.1, max(deadline - time.monotonic(), 0.0)))

    def _set_mobile_base_robot_offset(self) -> None:
        if not self.mobile_base_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "mobile_base_tf_publisher parameter service was not found; world->base_link TF may keep its previous offset."
            )
            return
        parameter = Parameter()
        parameter.name = "robot_y_offset"
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = float(self.robot_y_offset)
        request = SetParameters.Request()
        request.parameters = [parameter]
        future = self.mobile_base_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        response = future.result()
        if response is None or not response.results or not response.results[0].successful:
            reason = ""
            if response is not None and response.results:
                reason = response.results[0].reason
            self.get_logger().warn(f"Could not update mobile_base_tf_publisher robot_y_offset: {reason}")

    def _robot_workspace_box_scene_parameters(self) -> list[Parameter]:
        include_box = bool(self.get_parameter("include_robot_workspace_box_collision").value)
        rear_x = float(self.get_parameter("robot_workspace_box_rear_x").value)
        front_x = float(self.get_parameter("robot_workspace_box_front_x").value)
        half_y = abs(float(self.get_parameter("robot_workspace_box_half_y").value))
        ceiling_z = float(self.get_parameter("robot_workspace_box_ceiling_z").value)
        wall_thickness = max(float(self.get_parameter("robot_workspace_box_wall_thickness").value), 1e-4)
        wall_height = max(float(self.get_parameter("robot_workspace_box_height").value), 1e-4)
        x_depth = max(abs(front_x - rear_x) + wall_thickness, wall_thickness)
        y_width = max(half_y * 2.0 + wall_thickness, wall_thickness)
        return [
            self._scene_bool_parameter("include_ground_collision", include_box),
            self._scene_bool_parameter("include_ceiling_collision", include_box),
            self._scene_double_parameter("ceiling_z", ceiling_z),
            self._scene_double_parameter("ceiling_thickness", wall_thickness),
            self._scene_double_parameter("ceiling_width", x_depth),
            self._scene_double_parameter("ceiling_depth", y_width),
            self._scene_bool_parameter("include_robot_rear_wall_collision", include_box),
            self._scene_double_parameter("robot_rear_wall_x", rear_x),
            self._scene_double_parameter("robot_rear_wall_thickness", wall_thickness),
            self._scene_double_parameter("robot_rear_wall_width", y_width),
            self._scene_double_parameter("robot_rear_wall_height", wall_height),
            self._scene_bool_parameter("include_robot_front_wall_collision", include_box),
            self._scene_double_parameter("robot_front_wall_x", front_x),
            self._scene_double_parameter("robot_front_wall_thickness", wall_thickness),
            self._scene_double_parameter("robot_front_wall_width", y_width),
            self._scene_double_parameter("robot_front_wall_height", wall_height),
            self._scene_bool_parameter("include_robot_side_wall_collision", include_box),
            self._scene_double_parameter("robot_side_wall_y", half_y),
            self._scene_double_parameter("robot_side_wall_thickness", wall_thickness),
            self._scene_double_parameter("robot_side_wall_depth", x_depth),
            self._scene_double_parameter("robot_side_wall_height", wall_height),
        ]

    def _scene_bool_parameter(self, name: str, value: bool) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = bool(value)
        return parameter

    def _scene_double_parameter(self, name: str, value: float) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = float(value)
        return parameter

    def _scene_double_array_parameter(self, name: str, values) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        parameter.value.double_array_value = [float(value) for value in values]
        return parameter

    def _scene_string_parameter(self, name: str, value: str) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_STRING
        parameter.value.string_value = str(value)
        return parameter

    def _scene_string_array_parameter(self, name: str, values: set[str]) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_STRING_ARRAY
        parameter.value.string_array_value = sorted(values)
        return parameter

    def _run_taught_relative_harvest_sequence(self) -> bool:
        if self.side not in {"left", "right", "both"}:
            self.get_logger().error("side must be one of: left, right, both")
            return False
        self.get_logger().info("Waiting for MoveIt move_group action server...")
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("MoveIt action server was not found. Is teach.launch.py running?")
            return False

        entries = self._selected_tomato_entries()
        if bool(self.get_parameter("harvest_camera_visible_only").value):
            entries = self._filter_camera_visible_tomato_entries(entries)
        target_tomato_index = int(self.get_parameter("target_tomato_index").value)
        if target_tomato_index >= 0:
            self.get_logger().info(
                "Using exact tomato index selection: "
                f"target_tomato_index={target_tomato_index}; side/start_rank/max_tomatoes are ignored."
            )
        else:
            start_rank = max(1, int(self.get_parameter("start_rank").value))
            if start_rank > 1:
                entries = entries[start_rank - 1 :]
            max_tomatoes = int(self.get_parameter("max_tomatoes").value)
            if max_tomatoes > 0:
                entries = entries[:max_tomatoes]
        if not entries:
            if (
                target_tomato_index < 0
                and bool(self.get_parameter("allow_empty_harvest_targets").value)
            ):
                self.get_logger().info(
                    f"No tomato entries matched side={self.side!r} target_tomato_index={target_tomato_index}; "
                    "treating this as an empty harvest step and continuing."
                )
                return True
            self.get_logger().error(
                f"No tomato entries matched side={self.side!r} target_tomato_index={target_tomato_index}."
            )
            return False

        use_side_spin_rule_grasp = bool(self.get_parameter("use_side_spin_rule_grasp_pose").value)
        use_right_rule_grasp = bool(self.get_parameter("use_right_rule_grasp_pose").value)
        relative_transform = None if (use_side_spin_rule_grasp or use_right_rule_grasp) else self._relative_grasp_transform()
        continue_on_failure = bool(self.get_parameter("continue_on_failure").value)
        if (
            bool(self.get_parameter("preplan_visible_harvest_trajectories").value)
            and not bool(self.get_parameter("plan_candidates_only").value)
        ):
            return self._preplan_then_execute_harvest_sequence(
                entries,
                relative_transform,
                use_side_spin_rule_grasp,
                use_right_rule_grasp,
                continue_on_failure,
            )
        results = []

        for rank, entry in enumerate(entries, start=1):
            object_id = self._entry_collision_id(entry)
            use_rule_grasp = use_side_spin_rule_grasp or (use_right_rule_grasp and str(entry["side"]) == "right")
            if bool(self.get_parameter("return_home_before_each_target").value) and not bool(
                self.get_parameter("plan_candidates_only").value
            ):
                first_target_already_homed = rank == 1 and bool(self.get_parameter("return_home_on_start").value)
                if not first_target_already_homed:
                    if not self._move_to_home_pose(f"before_target_{object_id}"):
                        self.get_logger().error(f"Could not move home before approaching {object_id}.")
                        if not continue_on_failure:
                            break
                        results.append((entry, False))
                        continue
            self.get_logger().info(
                f"Harvesting {'side-spin rule' if use_rule_grasp else 'taught'} grasp target "
                f"side={entry['side']} rank={rank}/{len(entries)} index={entry['index']} "
                f"object_id={object_id} "
                f"vine_row_y={float(entry.get('vine_row_y_offset', 0.0)):.3f} "
                f"tomato_xyz=({entry['position'][0]:.4f}, {entry['position'][1]:.4f}, {entry['position'][2]:.4f})"
            )
            self._prepare_tomato_for_grasp(object_id)
            if use_rule_grasp:
                success = self._try_side_spin_rule_grasp(entry)
            else:
                if relative_transform is None:
                    relative_transform = self._relative_grasp_transform()
                base_to_grasp = self._base_to_tomato_transform(entry) @ relative_transform
                target_pose = self._plan_pose_from_grasp(base_to_grasp[:3, 3], base_to_grasp[:3, :3])
                self.get_logger().info(
                    "Taught grasp target pose "
                    f"{self.target_link}_xyz=({target_pose.position.x:.4f}, "
                    f"{target_pose.position.y:.4f}, {target_pose.position.z:.4f})"
                )
                plan_success = self._plan_to_pose(target_pose)
                success = plan_success and self._verify_reached_pose(target_pose)
            reached_grasp = success
            if success and not bool(self.get_parameter("plan_candidates_only").value):
                if bool(self.get_parameter("stop_after_grasp_pose").value):
                    self.get_logger().info(
                        f"Stopped at grasp pose for {object_id}; post-grasp twist, "
                        "home return, harvest marking, and tomato removal are skipped."
                    )
                    results.append((entry, success))
                    break
                if self._cartesian_stop_at_c_pre_enabled():
                    self.get_logger().info(
                        f"Stopped at Cartesian C_pre for {object_id}; harvest completion, "
                        "post-grasp twist, home return, and tomato removal are skipped."
                    )
                    results.append((entry, success))
                    break
                success = self._complete_grasp_by_returning_home_then_harvest(object_id)
            results.append((entry, success))
            if success:
                if bool(self.get_parameter("plan_candidates_only").value):
                    self.get_logger().info(f"Completed plan-only candidate collection for {object_id}.")
                else:
                    self.get_logger().info(f"Completed grasp -> home -> harvest for {object_id}.")
            else:
                self._sync_scene_tomato_lists(self.harvested_tomato_ids)
                self.get_logger().error(f"Harvest failed before completing grasp -> home -> harvest for {object_id}.")
                if reached_grasp:
                    self.get_logger().error(
                        "Stopping remaining harvest targets because the robot reached a grasp pose "
                        "but could not return home cleanly."
                    )
                    break
                if not continue_on_failure:
                    break

        success_count = sum(1 for _, success in results if success)
        summary = ", ".join(
            f"{entry['side']}#{rank}:idx{entry['index']}={'OK' if success else 'FAIL'}"
            for rank, (entry, success) in enumerate(results, start=1)
        )
        self.get_logger().info(f"Harvest sequence summary: {summary}")
        self.get_logger().info(f"Harvested count: {success_count}/{len(entries)}")
        return bool(results) and success_count == len(entries)

    def _preplan_then_execute_harvest_sequence(
        self,
        entries: list[dict[str, object]],
        relative_transform: np.ndarray | None,
        use_side_spin_rule_grasp: bool,
        use_right_rule_grasp: bool,
        continue_on_failure: bool,
    ) -> bool:
        self.get_logger().info(
            "Preplan-visible mode enabled: planning all selected/camera-visible grasp trajectories first, "
            "then executing the valid plans continuously."
        )
        if self._cartesian_stop_at_c_pre_enabled():
            self.get_logger().warn(
                "preplan_visible_harvest_trajectories is enabled together with cartesian_stop_at_c_pre; "
                "execution will stop at C_pre for the first executed target."
            )
        if bool(self.get_parameter("stop_after_grasp_pose").value):
            self.get_logger().warn(
                "preplan_visible_harvest_trajectories is enabled together with stop_after_grasp_pose; "
                "execution will stop at the first reached grasp pose."
            )

        planned_records: list[dict[str, object]] = []
        plan_results: list[tuple[dict[str, object], bool]] = []
        reset_spin_before_plan = bool(self.get_parameter("preplan_reset_gripper_spin_before_each_target").value)
        home_gripper_spin_deg = self._clamp_gripper_spin_deg(
            float(self.get_parameter("home_gripper_spin_deg").value)
        )

        for rank, entry in enumerate(entries, start=1):
            object_id = self._entry_collision_id(entry)
            use_rule_grasp = use_side_spin_rule_grasp or (
                use_right_rule_grasp and str(entry["side"]) == "right"
            )
            self.get_logger().info(
                f"Preplanning target rank={rank}/{len(entries)} side={entry['side']} "
                f"index={entry['index']} object_id={object_id} "
                f"vine_row_y={float(entry.get('vine_row_y_offset', 0.0)):.3f}"
            )
            if reset_spin_before_plan:
                self._set_runtime_gripper_spin_deg(home_gripper_spin_deg, log=False)
            record = self._preplan_grasp_record_for_entry(
                entry,
                object_id,
                use_rule_grasp,
                relative_transform,
            )
            success = record is not None
            plan_results.append((entry, success))
            if success:
                record["preplanned_rank"] = rank
                record["preplanned_object_id"] = object_id
                planned_records.append(record)
                metrics = record.get("trajectory_metrics", {})
                duration = (
                    float(metrics.get("trajectory_duration_sec", 0.0))
                    if isinstance(metrics, dict)
                    else 0.0
                )
                self.get_logger().info(
                    f"Preplanned OK rank={rank}/{len(entries)} object_id={object_id} "
                    f"duration={duration:.3f}s"
                )
            else:
                self._sync_scene_tomato_lists(self.harvested_tomato_ids)
                self.get_logger().error(
                    f"Preplanning failed for rank={rank}/{len(entries)} object_id={object_id}."
                )
                if not continue_on_failure:
                    break

        failed_plan_count = sum(1 for _, success in plan_results if not success)
        if not planned_records:
            self.get_logger().error("Preplan-visible mode found no executable grasp trajectory.")
            return False
        if failed_plan_count and not bool(self.get_parameter("preplan_execute_partial_success").value):
            self.get_logger().error(
                f"Preplan-visible mode found {failed_plan_count} failed target(s); "
                "preplan_execute_partial_success is false, so execution is cancelled."
            )
            return False

        self.get_logger().info(
            f"Preplan-visible mode planning complete: valid={len(planned_records)}/{len(entries)} "
            f"failed={failed_plan_count}. Executing preplanned trajectories now."
        )
        if (
            bool(self.get_parameter("return_home_on_start").value)
            or bool(self.get_parameter("return_home_before_each_target").value)
            or reset_spin_before_plan
        ):
            if not self._move_to_home_pose("before_preplanned_execution"):
                return False

        execution_results: list[tuple[dict[str, object], bool]] = []
        for execution_rank, record in enumerate(planned_records, start=1):
            entry = record["entry"]
            object_id = str(record.get("preplanned_object_id", self._entry_collision_id(entry)))
            if bool(self.get_parameter("return_home_before_each_target").value) and execution_rank > 1:
                if not self._move_to_home_pose(f"before_preplanned_target_{object_id}"):
                    self.get_logger().error(f"Could not move home before executing preplanned {object_id}.")
                    execution_results.append((entry, False))
                    if not continue_on_failure:
                        break
                    continue

            self.get_logger().info(
                f"Executing preplanned target {execution_rank}/{len(planned_records)} "
                f"object_id={object_id} side={entry['side']} index={entry['index']}"
            )
            self._prepare_tomato_for_grasp(object_id)
            success = self._execute_selected_grasp_record(record)
            reached_grasp = success
            if success:
                if bool(self.get_parameter("stop_after_grasp_pose").value):
                    self.get_logger().info(
                        f"Stopped at grasp pose for preplanned {object_id}; remaining preplanned targets are skipped."
                    )
                    execution_results.append((entry, success))
                    break
                if self._cartesian_stop_at_c_pre_enabled():
                    self.get_logger().info(
                        f"Stopped at Cartesian C_pre for preplanned {object_id}; remaining preplanned targets are skipped."
                    )
                    execution_results.append((entry, success))
                    break
                success = self._complete_grasp_by_returning_home_then_harvest(object_id)

            execution_results.append((entry, success))
            if success:
                self.get_logger().info(f"Completed preplanned grasp -> home -> harvest for {object_id}.")
            else:
                self._sync_scene_tomato_lists(self.harvested_tomato_ids)
                self.get_logger().error(f"Preplanned harvest failed for {object_id}.")
                if reached_grasp:
                    self.get_logger().error(
                        "Stopping remaining preplanned targets because the robot reached a grasp pose "
                        "but could not return home cleanly."
                    )
                    break
                if not continue_on_failure:
                    break

        success_count = sum(1 for _, success in execution_results if success)
        plan_summary = ", ".join(
            f"{entry['side']}#idx{entry['index']}={'PLAN_OK' if success else 'PLAN_FAIL'}"
            for entry, success in plan_results
        )
        execution_summary = ", ".join(
            f"{entry['side']}#idx{entry['index']}={'OK' if success else 'FAIL'}"
            for entry, success in execution_results
        )
        self.get_logger().info(f"Preplan-visible planning summary: {plan_summary}")
        self.get_logger().info(f"Preplan-visible execution summary: {execution_summary}")
        self.get_logger().info(f"Preplanned harvested count: {success_count}/{len(entries)}")
        if (
            bool(self.get_parameter("stop_after_grasp_pose").value)
            or self._cartesian_stop_at_c_pre_enabled()
        ):
            return bool(execution_results) and success_count > 0

        if bool(self.get_parameter("preplan_execute_partial_success").value):
            return bool(execution_results) and success_count == len(planned_records)
        return bool(execution_results) and success_count == len(entries)

    def _set_preplan_valid_records(
        self,
        valid_records: list[dict[str, object]],
        latest_record: dict[str, object] | None = None,
    ) -> None:
        self._preplan_last_valid_records = list(valid_records)
        if latest_record is None:
            return
        try:
            self._on_preplan_valid_candidate_record(latest_record, self._preplan_last_valid_records)
        except Exception as exc:
            self.get_logger().warn(f"Preplan valid-candidate callback failed: {exc}")

    def _on_preplan_valid_candidate_record(
        self,
        _record: dict[str, object],
        _valid_records: list[dict[str, object]],
    ) -> None:
        pass

    def _preplan_grasp_record_for_entry(
        self,
        entry: dict[str, object],
        object_id: str,
        use_rule_grasp: bool,
        relative_transform: np.ndarray | None,
    ) -> dict[str, object] | None:
        self._prepare_tomato_for_grasp(object_id)
        if use_rule_grasp:
            previous_collect = self._preplan_collect_selected_record
            previous_record = self._preplan_last_selected_record
            planned_valid_records: list[dict[str, object]] = []
            planned_selection_reason = ""
            self._preplan_collect_selected_record = True
            self._preplan_last_selected_record = None
            self._preplan_last_valid_records = []
            self._preplan_last_selection_reason = ""
            try:
                if not self._try_side_spin_rule_grasp(entry):
                    planned_valid_records = list(self._preplan_last_valid_records)
                    planned_selection_reason = str(self._preplan_last_selection_reason)
                    return None
                record = self._preplan_last_selected_record
                planned_valid_records = list(self._preplan_last_valid_records)
                planned_selection_reason = str(self._preplan_last_selection_reason)
            finally:
                self._preplan_collect_selected_record = previous_collect
                self._preplan_last_selected_record = previous_record
                self._preplan_last_valid_records = planned_valid_records
                self._preplan_last_selection_reason = planned_selection_reason
            return record

        if relative_transform is None:
            relative_transform = self._relative_grasp_transform()
        base_to_grasp = self._base_to_tomato_transform(entry) @ relative_transform
        target_pose = self._plan_pose_from_grasp(base_to_grasp[:3, 3], base_to_grasp[:3, :3])
        plan_result = self._move_group_to_pose(target_pose, plan_only=True)
        if not bool(plan_result["success"]):
            return None
        trajectory_metrics = self._trajectory_naturalness_metrics(
            plan_result["trajectory"],
            float(plan_result["planning_time"]),
        )
        rejection_reasons = self._trajectory_naturalness_rejection_reasons(trajectory_metrics)
        if rejection_reasons:
            self.get_logger().warn(
                "Skipping preplanned taught trajectory because " + "; ".join(rejection_reasons)
            )
            return None
        return {
            "entry": entry,
            "candidate": {},
            "target_pose": target_pose,
            "planning_pose": target_pose,
            "gripper_spin_deg": self.current_gripper_spin_deg,
            "requested_gripper_spin_deg": self.current_gripper_spin_deg,
            "planner_selects_gripper_spin": False,
            "joint_plan_goal_pose": "grasp",
            "cartesian_pregrasp_planned": False,
            "cartesian_full_path_planned": False,
            "gripper_x_sign": 0.0,
            "plan_retry": 1,
            "plan_retry_count": 1,
            "plan_result": plan_result,
            "trajectory_metrics": trajectory_metrics,
            "grasp_precision_score": 0.0,
        }

    def _complete_grasp_by_returning_home_then_harvest(self, object_id: str) -> bool:
        self.get_logger().info(
            f"Ideal gripper grasp pose reached for {object_id}; running post-grasp twist before returning home."
        )
        if bool(self.get_parameter("attach_tomato_visual_after_grasp").value):
            self._attach_tomato_visual_to_gripper(object_id)
        if bool(self.get_parameter("twist_gripper_after_grasp").value):
            self._run_post_grasp_twist()
        if bool(self.get_parameter("return_home_on_finish").value):
            if not self._return_home_after_grasp(object_id):
                self.get_logger().error(f"Could not return home after grasping {object_id}; harvest is not confirmed.")
                return False
        self.harvested_tomato_ids.add(object_id)
        self._sync_scene_tomato_lists(self.harvested_tomato_ids)
        self._clear_attached_tomato_visual()
        self.get_logger().info(f"Returned home with {object_id}; now marking it as harvested.")
        return True

    def _return_home_after_grasp(self, object_id: str) -> bool:
        if bool(self.get_parameter("return_home_via_reverse_grasp_path").value):
            if self._execute_reverse_grasp_path_home(object_id):
                return True
            if not bool(self.get_parameter("reverse_grasp_path_fallback_to_planned_home").value):
                return False
            self.get_logger().warn(
                "Reverse grasp path home failed or was unavailable; falling back to planned home trajectory."
            )
        return self._move_to_home_pose(f"after_grasp_before_harvest_{object_id}", reset_gripper_spin=False)

    def _execute_reverse_grasp_path_home(self, object_id: str) -> bool:
        record = self.last_grasp_record
        if not record:
            self.get_logger().warn(f"Cannot return {object_id} home via reverse path: no last grasp record.")
            return False
        trajectories = record.get("executed_grasp_trajectories", [])
        if not isinstance(trajectories, list) or not trajectories:
            self.get_logger().warn(
                f"Cannot return {object_id} home via reverse path: no executed grasp trajectory was recorded."
            )
            return False

        self.get_logger().info(
            f"Returning home by reversing {len(trajectories)} executed grasp trajectory segment(s)."
        )
        for segment_index, trajectory in enumerate(reversed(trajectories), start=1):
            reversed_trajectory = self._reverse_robot_trajectory(trajectory)
            if reversed_trajectory is None:
                self.get_logger().warn(f"Reverse grasp segment {segment_index} is empty or invalid.")
                return False
            point_count = len(reversed_trajectory.joint_trajectory.points)
            self.get_logger().info(
                f"Executing reverse grasp segment {segment_index}/{len(trajectories)} "
                f"points={point_count}"
            )
            if not self._execute_joint_trajectory(reversed_trajectory):
                return False
            self.current_gripper_spin_deg = self._trajectory_final_joint_deg(
                reversed_trajectory,
                "tool_gripper_z_joint",
                self.current_gripper_spin_deg,
            )
        return True

    def _reverse_robot_trajectory(self, trajectory):
        if trajectory is None or not trajectory.joint_trajectory.points:
            return None

        reversed_trajectory = copy.deepcopy(trajectory)
        source_points = list(trajectory.joint_trajectory.points)
        final_time_sec = self._duration_msg_seconds(source_points[-1].time_from_start)
        reversed_points = []
        for source_point in reversed(source_points):
            point = copy.deepcopy(source_point)
            source_time_sec = self._duration_msg_seconds(source_point.time_from_start)
            point.time_from_start = Duration(seconds=max(0.0, final_time_sec - source_time_sec)).to_msg()
            if point.velocities:
                point.velocities = [-float(value) for value in point.velocities]
            reversed_points.append(point)
        reversed_trajectory.joint_trajectory.points = reversed_points
        return reversed_trajectory

    @staticmethod
    def _duration_msg_seconds(duration_msg) -> float:
        return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9

    def _try_side_spin_rule_grasp(self, entry: dict[str, object]) -> bool:
        tomato_transform = self._base_to_tomato_transform(entry)
        object_position = tomato_transform[:3, 3]
        tomato_rotation = tomato_transform[:3, :3]
        preferred_approach_sign = self._tomato_y_back_approach_sign()
        if bool(self.get_parameter("try_both_approach_sides").value):
            approach_signs = [preferred_approach_sign, -preferred_approach_sign]
        else:
            approach_signs = [preferred_approach_sign]
        force_gripper_x_sign = float(self.get_parameter("force_gripper_x_sign").value)
        if force_gripper_x_sign > 0.0:
            gripper_x_signs = [1.0]
        elif force_gripper_x_sign < 0.0:
            gripper_x_signs = [-1.0]
        else:
            gripper_x_signs = [-1.0, 1.0]
        if force_gripper_x_sign != 0.0:
            self.get_logger().info(
                "Forced gripper_x_sign candidate set: "
                f"{gripper_x_signs[0]:+.0f}"
            )
        select_best = bool(self.get_parameter("select_best_trajectory_candidate").value)
        plan_retries = max(1, int(self.get_parameter("plan_retries_per_grasp_candidate").value))
        use_early_exit = bool(self.get_parameter("use_excellent_trajectory_early_exit").value)
        excellent_score_threshold = float(self.get_parameter("excellent_trajectory_score_threshold").value)
        planner_selects_gripper_spin = bool(self.get_parameter("planner_selects_gripper_spin").value)
        use_cartesian_grasp_approach = bool(self.get_parameter("use_cartesian_grasp_approach").value)
        use_full_cartesian_path = (
            use_cartesian_grasp_approach
            and self._cartesian_full_path_from_current_enabled()
        )
        valid_records: list[dict[str, object]] = []

        if planner_selects_gripper_spin:
            spin_candidates: list[float | None] = [None]
            self.get_logger().info(
                "Planner-selected gripper spin mode is enabled; "
                "skipping explicit tool_gripper_z_joint spin candidates."
            )
        else:
            spin_candidates = list(self._gripper_spin_candidates_for_entry(entry))

        for spin_index, gripper_spin_deg in enumerate(spin_candidates, start=1):
            if gripper_spin_deg is None:
                candidate_spin_deg = self._gripper_spin_deg_for_entry(entry)
                spin_label = "free"
            else:
                candidate_spin_deg = float(gripper_spin_deg)
                spin_label = f"{candidate_spin_deg:.1f}deg"
            gripper_spin_rad = math.radians(candidate_spin_deg)

            for approach_sign in approach_signs:
                for gripper_x_sign in gripper_x_signs:
                    candidate = self._tomato_rule_grasp_candidate(
                        object_position,
                        tomato_rotation,
                        approach_sign,
                        gripper_spin_rad,
                        gripper_x_sign=gripper_x_sign,
                    )
                    candidate["entry_index"] = int(entry["index"])
                    candidate["entry_side"] = str(entry["side"])
                    candidate["entry_vine_row_index"] = int(entry.get("vine_row_index", 0))
                    candidate["entry_vine_row_y_offset"] = float(entry.get("vine_row_y_offset", 0.0))
                    target_pose = candidate["pose"]
                    planning_pose, planning_pose_label = self._joint_plan_pose_for_candidate(
                        target_pose,
                        candidate,
                        use_cartesian_grasp_approach,
                    )
                    self.get_logger().info(
                        "Side-spin rule grasp candidate "
                        f"side={entry['side']} "
                        f"spin_candidate={spin_index} "
                        f"planned_gripper_spin={spin_label} "
                        f"tomato_y_back_approach_sign={approach_sign:+.0f} "
                        f"gripper_x_sign={gripper_x_sign:+.0f} "
                        f"{self.target_link}_xyz=({target_pose.position.x:.4f}, "
                        f"{target_pose.position.y:.4f}, {target_pose.position.z:.4f}), "
                        f"tool0_z_minus_gripper_link_z={candidate['tool0_z_minus_gripper_link_z']:.4f}, "
                        f"rod_z_parallel={candidate['rod_tomato_z_parallel_angle_deg']:.2f}deg, "
                        f"gripper_y_parallel={candidate['gripper_tomato_y_parallel_angle_deg']:.2f}deg, "
                        f"calyx_finger={candidate['calyx_finger']}, "
                        f"calyx_contact_err={candidate['calyx_contact_error']:.4f}, "
                        f"opposite_finger={candidate['opposite_finger']}, "
                        f"opposite_contact_err={candidate['opposite_contact_error']:.4f}, "
                        f"calyx_align_deg={candidate['calyx_alignment_angle_deg']:.2f}"
                    )
                    rejection_reasons = self._right_rule_rejection_reasons(candidate)
                    if rejection_reasons:
                        self.get_logger().warn(
                            "Skipping side-spin rule candidate because " + "; ".join(rejection_reasons)
                        )
                        continue

                    for plan_retry in range(1, plan_retries + 1):
                        if use_full_cartesian_path:
                            if not planner_selects_gripper_spin:
                                self._set_runtime_gripper_spin_deg(candidate_spin_deg)
                            plan_result = self._compute_cartesian_path_to_pose(
                                target_pose,
                                waypoints=self._cartesian_grasp_waypoints(
                                    target_pose,
                                    include_start=True,
                                    candidate=candidate,
                                ),
                                fixed_gripper_spin_deg=None
                                if planner_selects_gripper_spin
                                else candidate_spin_deg,
                            )
                        elif planner_selects_gripper_spin:
                            plan_result = self._move_group_to_pose(
                                planning_pose,
                                plan_only=True,
                            )
                        else:
                            plan_result = self._move_group_to_pose_with_joint_constraints(
                                planning_pose,
                                {"tool_gripper_z_joint": gripper_spin_rad},
                                tolerance=math.radians(0.25),
                                plan_only=True,
                                goal_label="cartesian_start_pose_with_tool_spin"
                                if use_cartesian_grasp_approach
                                else "grasp_pose_with_tool_spin",
                            )
                        if not bool(plan_result["success"]):
                            cartesian_fraction = plan_result.get("fraction") if isinstance(plan_result, dict) else None
                            cartesian_note = (
                                f" fraction={float(cartesian_fraction):.3f}"
                                if cartesian_fraction is not None
                                else ""
                            )
                            self.get_logger().warn(
                                "Side-spin rule candidate did not produce a valid MoveIt plan "
                                f"spin={spin_label} gripper_x_sign={gripper_x_sign:+.0f} "
                                f"retry={plan_retry}/{plan_retries}"
                                f"{cartesian_note}"
                            )
                            continue
                        planned_final_spin_deg = self._trajectory_final_joint_deg(
                            plan_result["trajectory"],
                            "tool_gripper_z_joint",
                            candidate_spin_deg,
                        )
                        grasp_spin_rejection_reason = self._grasp_gripper_spin_trajectory_rejection_reason(
                            plan_result["trajectory"],
                            planned_final_spin_deg
                        )
                        if grasp_spin_rejection_reason:
                            self.get_logger().warn(
                                "Skipping trajectory candidate because " + grasp_spin_rejection_reason
                            )
                            continue
                        trajectory_metrics = self._trajectory_naturalness_metrics(
                            plan_result["trajectory"],
                            float(plan_result["planning_time"]),
                        )
                        self.get_logger().info(
                            "Side-spin rule trajectory candidate "
                            f"side={entry['side']} spin={spin_label} "
                            f"planned_final_spin={planned_final_spin_deg:.1f}deg "
                            f"gripper_x_sign={gripper_x_sign:+.0f} "
                            f"retry={plan_retry}/{plan_retries} "
                            f"grasp_precision_score={self._grasp_precision_score(candidate):.3f} "
                            f"{self._trajectory_naturalness_summary(trajectory_metrics)}"
                        )
                        trajectory_rejection_reasons = self._trajectory_naturalness_rejection_reasons(trajectory_metrics)
                        if trajectory_rejection_reasons:
                            self.get_logger().warn(
                                "Skipping trajectory candidate because " + "; ".join(trajectory_rejection_reasons)
                            )
                            continue

                        record = {
                            "entry": entry,
                            "candidate": candidate,
                            "target_pose": target_pose,
                            "planning_pose": planning_pose,
                            "gripper_spin_deg": planned_final_spin_deg,
                            "requested_gripper_spin_deg": None
                            if planner_selects_gripper_spin
                            else candidate_spin_deg,
                            "planner_selects_gripper_spin": planner_selects_gripper_spin,
                            "joint_plan_goal_pose": planning_pose_label,
                            "cartesian_pregrasp_planned": use_cartesian_grasp_approach and not use_full_cartesian_path,
                            "cartesian_full_path_planned": use_full_cartesian_path,
                            "gripper_x_sign": gripper_x_sign,
                            "plan_retry": plan_retry,
                            "plan_retry_count": plan_retries,
                            "plan_result": plan_result,
                            "trajectory_metrics": trajectory_metrics,
                            "grasp_precision_score": self._grasp_precision_score(candidate),
                        }
                        if not select_best:
                            self._set_preplan_valid_records([record], record)
                            self._preplan_last_selection_reason = "first_valid_candidate"
                            self._write_grasp_diagnostics(
                                record,
                                [record],
                                selection_reason="first_valid_candidate",
                            )
                            return self._finish_selected_grasp_record(record)
                        if (
                            use_early_exit
                            and excellent_score_threshold > 0.0
                            and float(trajectory_metrics["score"]) <= excellent_score_threshold
                        ):
                            self.get_logger().info(
                                "Using side-spin trajectory early because it is already excellent "
                                f"(score={float(trajectory_metrics['score']):.3f} <= {excellent_score_threshold:.3f})."
                            )
                            early_records = valid_records + [record]
                            self._set_preplan_valid_records(early_records, record)
                            self._preplan_last_selection_reason = "excellent_early_exit"
                            self._write_grasp_diagnostics(
                                record,
                                early_records,
                                selection_reason="excellent_early_exit",
                            )
                            return self._finish_selected_grasp_record(record)
                        valid_records.append(record)
                        self._set_preplan_valid_records(valid_records, record)

        if not valid_records:
            self._set_preplan_valid_records([])
            self._preplan_last_selection_reason = "no_valid_candidates"
            return False

        forced_rank = int(self.get_parameter("force_valid_candidate_rank").value)
        if forced_rank > 0:
            selection_metric = str(self.get_parameter("best_trajectory_selection_metric").value).strip().lower()
            ranked_records = sorted(valid_records, key=self._best_grasp_record_sort_key)
            self._set_preplan_valid_records(ranked_records)
            if forced_rank > len(ranked_records):
                self.get_logger().error(
                    f"force_valid_candidate_rank={forced_rank} was requested, "
                    f"but only {len(ranked_records)} valid candidate(s) were found."
                )
                self._preplan_last_selection_reason = f"forced_valid_candidate_rank_{forced_rank}_out_of_range"
                return False
            forced_record = ranked_records[forced_rank - 1]
            forced_metrics = forced_record["trajectory_metrics"]
            self.get_logger().info(
                "Selected forced side-spin trajectory "
                f"sorted_rank={forced_rank}/{len(ranked_records)} "
                f"rank_metric={selection_metric} "
                f"side={entry['side']} spin={float(forced_record['gripper_spin_deg']):.1f}deg "
                f"spin_mode={'free' if bool(forced_record.get('planner_selects_gripper_spin', False)) else 'fixed'} "
                f"gripper_x_sign={float(forced_record.get('gripper_x_sign', -1.0)):+.0f} "
                f"retry={int(forced_record.get('plan_retry', 1))}/{int(forced_record.get('plan_retry_count', 1))} "
                f"grasp_precision_score={float(forced_record['grasp_precision_score']):.3f} "
                f"{self._trajectory_naturalness_summary(forced_metrics)}"
            )
            self._write_grasp_diagnostics(
                forced_record,
                ranked_records,
                selection_reason=f"forced_valid_candidate_rank_{forced_rank}",
            )
            self._preplan_last_selection_reason = f"forced_valid_candidate_rank_{forced_rank}"
            return self._finish_selected_grasp_record(forced_record)

        selection_metric = str(self.get_parameter("best_trajectory_selection_metric").value).strip().lower()
        ranked_records = sorted(valid_records, key=self._best_grasp_record_sort_key)
        best_record = ranked_records[0]
        best_metrics = best_record["trajectory_metrics"]
        self.get_logger().info(
            "Selected best side-spin trajectory "
            f"selection_metric={selection_metric} "
            f"side={entry['side']} spin={float(best_record['gripper_spin_deg']):.1f}deg "
            f"spin_mode={'free' if bool(best_record.get('planner_selects_gripper_spin', False)) else 'fixed'} "
            f"gripper_x_sign={float(best_record.get('gripper_x_sign', -1.0)):+.0f} "
            f"retry={int(best_record.get('plan_retry', 1))}/{int(best_record.get('plan_retry_count', 1))} "
            f"grasp_precision_score={float(best_record['grasp_precision_score']):.3f} "
            f"{self._trajectory_naturalness_summary(best_metrics)} "
            f"from {len(ranked_records)} valid candidate(s)"
        )
        self._set_preplan_valid_records(ranked_records)
        self._preplan_last_selection_reason = "selected_best"
        self._write_grasp_diagnostics(best_record, ranked_records, selection_reason="selected_best")
        return self._finish_selected_grasp_record(best_record)

    def _best_grasp_record_sort_key(self, record: dict[str, object]) -> tuple[float, ...]:
        metric = str(self.get_parameter("best_trajectory_selection_metric").value).strip().lower()
        trajectory_metrics = record.get("trajectory_metrics", {})
        precision_score = float(record.get("grasp_precision_score", float("inf")))
        score = float(trajectory_metrics.get("score", float("inf"))) if isinstance(trajectory_metrics, dict) else float("inf")
        duration = (
            float(trajectory_metrics.get("trajectory_duration_sec", float("inf")))
            if isinstance(trajectory_metrics, dict)
            else float("inf")
        )
        if metric in {"learned", "ai", "model", "trajectory_selector"}:
            probability = self._trajectory_selector_good_probability(record)
            return (1.0 - probability, duration, precision_score, score)
        if metric in {"duration", "trajectory_duration", "trajectory_duration_sec", "time"}:
            return (duration, precision_score, score)
        if metric in {"precision", "grasp_precision"}:
            return (precision_score, score, duration)
        return (precision_score, score, duration)

    def _trajectory_selector_good_probability(self, record: dict[str, object]) -> float:
        model_path = str(self.get_parameter("trajectory_selector_model_path").value).strip()
        if not model_path:
            self.get_logger().warn(
                "best_trajectory_selection_metric=learned was requested, but trajectory_selector_model_path is empty."
            )
            return 0.0
        if self._trajectory_selector_model is None:
            try:
                self._trajectory_selector_model = read_json(model_path)
                self.get_logger().info(f"Loaded trajectory selector model: {model_path}")
            except Exception as exc:
                self.get_logger().error(f"Could not load trajectory selector model {model_path}: {exc}")
                self._trajectory_selector_model = {}
        if not self._trajectory_selector_model:
            return 0.0
        try:
            return predict_good_probability(self._record_diagnostics(record), self._trajectory_selector_model)
        except Exception as exc:
            self.get_logger().error(f"Could not score trajectory with learned selector: {exc}")
            return 0.0

    def _finish_selected_grasp_record(self, record: dict[str, object]) -> bool:
        plan_result = record.get("plan_result", {})
        trajectory = plan_result.get("trajectory") if isinstance(plan_result, dict) else None
        trajectory_start = plan_result.get("trajectory_start") if isinstance(plan_result, dict) else None
        if bool(getattr(self, "_preplan_collect_selected_record", False)):
            self._preplan_last_selected_record = record
            self.last_grasp_record = record
            self._publish_display_trajectory(trajectory, trajectory_start)
            self.get_logger().info(
                "Preplan collection mode is enabled; selected trajectory was recorded but not executed."
            )
            return True
        if bool(self.get_parameter("plan_candidates_only").value):
            self.last_grasp_record = record
            self._publish_display_trajectory(trajectory, trajectory_start)
            self.get_logger().info(
                "Plan-candidates-only mode is enabled; selected trajectory was recorded but not executed."
            )
            return True
        return self._execute_selected_grasp_record(record)

    def _pose_to_diagnostics(self, pose: Pose) -> dict[str, object]:
        return {
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation_xyzw": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }

    def _jsonable_diagnostics(self, value):
        if isinstance(value, np.ndarray):
            return self._jsonable_diagnostics(value.tolist())
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, Pose):
            return self._pose_to_diagnostics(value)
        if isinstance(value, dict):
            return {str(key): self._jsonable_diagnostics(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._jsonable_diagnostics(item) for item in value]
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        return str(value)

    def _trajectory_endpoint_diagnostics(self, trajectory) -> dict[str, object]:
        if trajectory is None:
            return {}
        joint_names = list(trajectory.joint_trajectory.joint_names)
        points = list(trajectory.joint_trajectory.points)
        if not joint_names or not points:
            return {"joint_names": joint_names, "point_count": len(points)}
        start_positions = {
            name: float(position)
            for name, position in zip(joint_names, points[0].positions)
        }
        final_positions = {
            name: float(position)
            for name, position in zip(joint_names, points[-1].positions)
        }
        return {
            "joint_names": joint_names,
            "point_count": len(points),
            "start_positions_rad": start_positions,
            "final_positions_rad": final_positions,
            "start_positions_deg": {
                name: math.degrees(position)
                for name, position in start_positions.items()
            },
            "final_positions_deg": {
                name: math.degrees(position)
                for name, position in final_positions.items()
            },
        }

    def _trajectory_full_diagnostics(self, trajectory) -> dict[str, object]:
        if trajectory is None:
            return {}
        joint_names = list(trajectory.joint_trajectory.joint_names)
        points = []
        for index, point in enumerate(trajectory.joint_trajectory.points):
            points.append(
                {
                    "index": int(index),
                    "time_from_start_sec": self._duration_msg_seconds(point.time_from_start),
                    "positions": [float(value) for value in point.positions],
                    "velocities": [float(value) for value in point.velocities],
                    "accelerations": [float(value) for value in point.accelerations],
                }
            )
        return {
            "joint_names": joint_names,
            "points": points,
        }

    def _record_diagnostics(self, record: dict[str, object]) -> dict[str, object]:
        plan_result = record.get("plan_result", {})
        trajectory = plan_result.get("trajectory") if isinstance(plan_result, dict) else None
        candidate = record.get("candidate", {})
        entry = record.get("entry", {})
        return self._jsonable_diagnostics(
            {
                "entry": entry,
                "target_link": self.target_link,
                "target_pose": record.get("target_pose"),
                "planning_pose": record.get("planning_pose"),
                "gripper_spin_deg": record.get("gripper_spin_deg"),
                "requested_gripper_spin_deg": record.get("requested_gripper_spin_deg"),
                "planner_selects_gripper_spin": record.get("planner_selects_gripper_spin"),
                "joint_plan_goal_pose": record.get("joint_plan_goal_pose"),
                "cartesian_pregrasp_planned": record.get("cartesian_pregrasp_planned"),
                "cartesian_full_path_planned": record.get("cartesian_full_path_planned"),
                "cartesian_fraction": plan_result.get("fraction") if isinstance(plan_result, dict) else None,
                "gripper_x_sign": record.get("gripper_x_sign"),
                "plan_retry": record.get("plan_retry"),
                "plan_retry_count": record.get("plan_retry_count"),
                "planning_time": plan_result.get("planning_time") if isinstance(plan_result, dict) else None,
                "trajectory_endpoints": self._trajectory_endpoint_diagnostics(trajectory),
                "trajectory": self._trajectory_full_diagnostics(trajectory),
                "grasp_precision_score": record.get("grasp_precision_score"),
                "trajectory_metrics": record.get("trajectory_metrics"),
                "candidate_metrics": candidate,
            }
        )

    def _pose_diagnostics_from_msg(self, pose) -> dict[str, object]:
        return {
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation_xyzw": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }

    @staticmethod
    def _numeric_msg_field(value, default: int = 0) -> int:
        if isinstance(value, (bytes, bytearray)):
            if not value:
                return default
            return int.from_bytes(value[:1], byteorder="little", signed=False)
        return int(value)

    def _collision_object_diagnostics(self, obj: CollisionObject) -> dict[str, object]:
        primitives = []
        for primitive, pose in zip(obj.primitives, obj.primitive_poses):
            primitives.append(
                {
                    "type": self._numeric_msg_field(primitive.type),
                    "dimensions": [float(value) for value in primitive.dimensions],
                    "pose": self._pose_diagnostics_from_msg(pose),
                }
            )
        return {
            "id": str(obj.id),
            "frame_id": str(obj.header.frame_id),
            "operation": self._numeric_msg_field(obj.operation),
            "primitives": primitives,
        }

    def _allowed_collision_matrix_diagnostics(self, matrix: AllowedCollisionMatrix) -> dict[str, object]:
        return {
            "entry_names": [str(name) for name in matrix.entry_names],
            "entry_values": [
                [bool(value) for value in row.enabled]
                for row in matrix.entry_values
            ],
            "default_entry_names": [str(name) for name in matrix.default_entry_names],
            "default_entry_values": [bool(value) for value in matrix.default_entry_values],
        }

    def _collision_scene_snapshot(self) -> dict[str, object]:
        snapshot: dict[str, object] = {
            "available": False,
            "objects": [],
            "allowed_collision_matrix": {},
            "harvested_tomato_ids": sorted(self.harvested_tomato_ids),
            "suppressed_collision_ids": sorted(self._base_suppressed_collision_ids() | set(self.harvested_tomato_ids)),
            "params": {
                "disable_tomato_collision_for_rule_test": bool(
                    self.get_parameter("disable_tomato_collision_for_rule_test").value
                ),
                "clear_target_tomato_collision": bool(self.get_parameter("clear_target_tomato_collision").value),
                "allow_target_tomato_touch": bool(self.get_parameter("allow_target_tomato_touch").value),
            },
        }
        if not self.get_scene_client.wait_for_service(timeout_sec=0.5):
            snapshot["reason"] = "get_planning_scene service unavailable"
            return snapshot
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        future = self.get_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            snapshot["reason"] = "get_planning_scene did not return"
            return snapshot
        scene = future.result().scene
        snapshot["available"] = True
        snapshot["objects"] = [
            self._collision_object_diagnostics(obj)
            for obj in scene.world.collision_objects
        ]
        snapshot["allowed_collision_matrix"] = self._allowed_collision_matrix_diagnostics(
            scene.allowed_collision_matrix
        )
        return snapshot

    def _write_grasp_diagnostics(
        self,
        selected_record: dict[str, object],
        valid_records: list[dict[str, object]],
        selection_reason: str,
    ) -> None:
        output_path = str(self.get_parameter("grasp_diagnostics_output_path").value).strip()
        if not output_path:
            return
        include_all = bool(self.get_parameter("grasp_diagnostics_include_all_valid_candidates").value)
        payload: dict[str, object] = {
            "stamp_unix": time.time(),
            "selection_reason": selection_reason,
            "side": self.side,
            "tomato_z_spin_deg": self.tomato_z_spin_deg,
            "taught_tomato_z_spin_deg": self.taught_tomato_z_spin_deg,
            "right_tomato_z_spin_offset_deg": self.right_tomato_z_spin_offset_deg,
            "left_tomato_z_spin_offset_deg": self.left_tomato_z_spin_offset_deg,
            "randomize_individual_stem_spin": self.randomize_individual_stem_spin,
            "individual_stem_spin_seed": self.individual_stem_spin_seed,
            "individual_stem_spin_min_deg": self.individual_stem_spin_min_deg,
            "individual_stem_spin_max_deg": self.individual_stem_spin_max_deg,
            "target_tomato_index": int(self.get_parameter("target_tomato_index").value),
            "plan_candidates_only": bool(self.get_parameter("plan_candidates_only").value),
            "select_best_trajectory_candidate": bool(self.get_parameter("select_best_trajectory_candidate").value),
            "force_valid_candidate_rank": int(self.get_parameter("force_valid_candidate_rank").value),
            "best_trajectory_selection_metric": str(
                self.get_parameter("best_trajectory_selection_metric").value
            ),
            "use_excellent_trajectory_early_exit": bool(self.get_parameter("use_excellent_trajectory_early_exit").value),
            "excellent_trajectory_score_threshold": float(
                self.get_parameter("excellent_trajectory_score_threshold").value
            ),
            "valid_candidate_count": len(valid_records),
            "collision_scene": self._collision_scene_snapshot(),
            "selected": self._record_diagnostics(selected_record),
        }
        if include_all:
            payload["valid_candidates"] = [
                self._record_diagnostics(record)
                for record in valid_records
            ]

        path = Path(output_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
        self.get_logger().info(f"Wrote grasp diagnostics: {path}")

    def _execute_selected_grasp_record(self, record: dict[str, object]) -> bool:
        if bool(self.get_parameter("use_cartesian_grasp_approach").value):
            return self._execute_selected_grasp_record_with_cartesian_approach(record)

        gripper_spin_deg = float(record["gripper_spin_deg"])
        target_pose = record["target_pose"]
        plan_result = record["plan_result"]
        trajectory = plan_result["trajectory"]
        trajectory_joint_names = list(trajectory.joint_trajectory.joint_names) if trajectory is not None else []
        spin_is_planned = "tool_gripper_z_joint" in trajectory_joint_names
        if not spin_is_planned:
            self._set_runtime_gripper_spin_deg(gripper_spin_deg)
        else:
            self.get_logger().info(
                f"Executing grasp with tool_gripper_z_joint inside MoveIt trajectory "
                f"(selected spin={gripper_spin_deg:.1f}deg)."
            )
        if not self.execute:
            self.last_grasp_record = record
            return True
        if not self._execute_joint_trajectory(trajectory):
            return False
        record["executed_grasp_trajectories"] = [trajectory]
        if spin_is_planned:
            self.current_gripper_spin_deg = self._trajectory_final_joint_deg(trajectory, "tool_gripper_z_joint", gripper_spin_deg)
        reached = self._verify_reached_pose(target_pose)
        if reached:
            self.last_grasp_record = record
        return reached

    def _execute_selected_grasp_record_with_cartesian_approach(self, record: dict[str, object]) -> bool:
        gripper_spin_deg = float(record["gripper_spin_deg"])
        target_pose = record["target_pose"]
        if bool(record.get("cartesian_full_path_planned", False)):
            return self._execute_selected_grasp_record_with_full_cartesian_path(record)

        candidate = record.get("candidate") if isinstance(record.get("candidate"), dict) else None
        start_pose = record.get("planning_pose")
        if not isinstance(start_pose, Pose):
            start_pose = self._cartesian_start_pose_for_target(target_pose, candidate=candidate)
        waypoints = self._cartesian_grasp_waypoints(target_pose, candidate=candidate)
        final_cartesian_pose = waypoints[-1] if waypoints else target_pose
        pregrasp_distance = float(self.get_parameter("cartesian_pregrasp_distance").value)
        approach_mode = self._cartesian_approach_mode()
        self.get_logger().info(
            "Executing grasp with Cartesian approach "
            f"mode={approach_mode} "
            f"stop_at_c_pre={self._cartesian_stop_at_c_pre_enabled()} "
            f"pregrasp_distance={pregrasp_distance:.3f}m selected_spin={gripper_spin_deg:.1f}deg "
            f"start_{self.target_link}_xyz=({start_pose.position.x:.4f}, "
            f"{start_pose.position.y:.4f}, {start_pose.position.z:.4f}) "
            f"grasp_{self.target_link}_xyz=({target_pose.position.x:.4f}, "
            f"{target_pose.position.y:.4f}, {target_pose.position.z:.4f}) "
            f"waypoints={len(waypoints)}"
        )

        pregrasp_result = record.get("plan_result") if bool(record.get("cartesian_pregrasp_planned", False)) else None
        if not isinstance(pregrasp_result, dict):
            pregrasp_result = self._move_group_to_pose_with_joint_constraints(
                start_pose,
                {"tool_gripper_z_joint": math.radians(gripper_spin_deg)},
                tolerance=math.radians(0.5),
                plan_only=True,
                goal_label="cartesian_start_pose_with_tool_spin",
            )
        if not bool(pregrasp_result["success"]):
            self.get_logger().error("Could not plan to Cartesian start pose.")
            return self._fallback_or_fail_joint_grasp(record)

        pregrasp_metrics = self._trajectory_naturalness_metrics(
            pregrasp_result["trajectory"],
            float(pregrasp_result["planning_time"]),
        )
        self.get_logger().info(
            "Cartesian start trajectory "
            f"{self._trajectory_naturalness_summary(pregrasp_metrics)}"
        )
        pregrasp_rejection_reasons = self._trajectory_naturalness_rejection_reasons(pregrasp_metrics)
        if pregrasp_rejection_reasons:
            self.get_logger().error(
                "Skipping Cartesian start trajectory because " + "; ".join(pregrasp_rejection_reasons)
            )
            return self._fallback_or_fail_joint_grasp(record)

        if not self.execute:
            self.last_grasp_record = record
            return True

        if not self._execute_joint_trajectory(pregrasp_result["trajectory"]):
            return False
        executed_grasp_trajectories = [pregrasp_result["trajectory"]]
        self.current_gripper_spin_deg = self._trajectory_final_joint_deg(
            pregrasp_result["trajectory"],
            "tool_gripper_z_joint",
            gripper_spin_deg,
        )

        cartesian_result = self._compute_cartesian_path_to_pose(
            target_pose,
            waypoints=waypoints,
            fixed_gripper_spin_deg=gripper_spin_deg,
        )
        if not bool(cartesian_result["success"]):
            self.get_logger().error(
                "Could not compute a valid Cartesian approach "
                f"fraction={float(cartesian_result.get('fraction', 0.0)):.3f} "
                f"error_code={cartesian_result.get('error_code')}"
            )
            return self._fallback_or_fail_joint_grasp(record)

        cartesian_metrics = self._trajectory_naturalness_metrics(
            cartesian_result["trajectory"],
            float(cartesian_result["planning_time"]),
        )
        self.get_logger().info(
            "Cartesian approach trajectory "
            f"fraction={float(cartesian_result['fraction']):.3f} "
            f"{self._trajectory_naturalness_summary(cartesian_metrics)}"
        )
        cartesian_rejection_reasons = self._trajectory_naturalness_rejection_reasons(cartesian_metrics)
        if cartesian_rejection_reasons:
            self.get_logger().error(
                "Skipping Cartesian approach because " + "; ".join(cartesian_rejection_reasons)
            )
            return self._fallback_or_fail_joint_grasp(record)

        if not self._execute_joint_trajectory(cartesian_result["trajectory"]):
            return False
        executed_grasp_trajectories.append(cartesian_result["trajectory"])
        record["executed_grasp_trajectories"] = executed_grasp_trajectories
        self.current_gripper_spin_deg = self._trajectory_final_joint_deg(
            cartesian_result["trajectory"],
            "tool_gripper_z_joint",
            gripper_spin_deg,
        )
        reached = self._verify_reached_pose(final_cartesian_pose)
        if reached:
            self.last_grasp_record = record
        return reached

    def _execute_selected_grasp_record_with_full_cartesian_path(self, record: dict[str, object]) -> bool:
        gripper_spin_deg = float(record["gripper_spin_deg"])
        target_pose = record["target_pose"]
        plan_result = record.get("plan_result", {})
        if not isinstance(plan_result, dict) or not bool(plan_result.get("success", False)):
            self.get_logger().error("Selected full Cartesian record does not contain a valid trajectory.")
            return self._fallback_or_fail_joint_grasp(record)

        trajectory = plan_result["trajectory"]
        candidate = record.get("candidate") if isinstance(record.get("candidate"), dict) else None
        waypoints = self._cartesian_grasp_waypoints(target_pose, include_start=True, candidate=candidate)
        final_cartesian_pose = waypoints[-1] if waypoints else target_pose
        self.get_logger().info(
            "Executing grasp with full Cartesian path from current state "
            f"mode={self._cartesian_approach_mode()} "
            f"stop_at_c_pre={self._cartesian_stop_at_c_pre_enabled()} "
            f"selected_spin={gripper_spin_deg:.1f}deg "
            f"fraction={float(plan_result.get('fraction', 0.0)):.3f} "
            f"waypoints={len(waypoints)}"
        )
        metrics = self._trajectory_naturalness_metrics(
            trajectory,
            float(plan_result.get("planning_time", 0.0)),
        )
        self.get_logger().info(
            "Full Cartesian trajectory "
            f"{self._trajectory_naturalness_summary(metrics)}"
        )

        if not self.execute:
            self.last_grasp_record = record
            return True

        if not self._execute_joint_trajectory(trajectory):
            return False
        record["executed_grasp_trajectories"] = [trajectory]
        self.current_gripper_spin_deg = self._trajectory_final_joint_deg(
            trajectory,
            "tool_gripper_z_joint",
            gripper_spin_deg,
        )
        reached = self._verify_reached_pose(final_cartesian_pose)
        if reached:
            self.last_grasp_record = record
        return reached

    def _fallback_or_fail_joint_grasp(self, record: dict[str, object]) -> bool:
        if not bool(self.get_parameter("cartesian_fallback_to_joint_grasp").value):
            return False
        self.get_logger().warn("Falling back to the original joint-space grasp execution.")
        original_value = self.get_parameter("use_cartesian_grasp_approach").value
        try:
            self.set_parameters([Parameter("use_cartesian_grasp_approach", value=False)])
            return self._execute_selected_grasp_record(record)
        finally:
            self.set_parameters([Parameter("use_cartesian_grasp_approach", value=bool(original_value))])

    def _pregrasp_pose_for_cartesian_approach(self, target_pose: Pose, candidate: dict[str, object] | None = None) -> Pose:
        distance = max(0.0, float(self.get_parameter("cartesian_pregrasp_distance").value))
        approach_axis = self._cartesian_xy_approach_axis(target_pose, candidate=candidate)
        pregrasp_pose = self._offset_pose_xy(target_pose, -approach_axis * distance)
        return pregrasp_pose

    def _cartesian_approach_mode(self) -> str:
        return str(self.get_parameter("cartesian_approach_mode").value).strip().lower()

    def _cartesian_full_path_from_current_enabled(self) -> bool:
        mode = self._cartesian_approach_mode()
        if mode in {"full", "full_cartesian", "top_view_full", "full_top_view", "xy_dogleg_full"}:
            return True
        if mode in {"staged", "top_view_staged", "xy_dogleg_staged"}:
            return False
        return bool(self.get_parameter("cartesian_full_path_from_current").value)

    def _cartesian_stop_at_c_pre_enabled(self) -> bool:
        return (
            bool(self.get_parameter("use_cartesian_grasp_approach").value)
            and bool(self.get_parameter("cartesian_stop_at_c_pre").value)
        )

    def _cartesian_start_pose_for_target(self, target_pose: Pose, candidate: dict[str, object] | None = None) -> Pose:
        if self._cartesian_approach_mode() in {
            "top_view",
            "top_view_dogleg",
            "xy_dogleg",
            "top_view_full",
            "full_top_view",
            "xy_dogleg_full",
            "top_view_staged",
            "xy_dogleg_staged",
        }:
            return self._top_view_cartesian_poses(target_pose, candidate=candidate)["a"]
        return self._pregrasp_pose_for_cartesian_approach(target_pose, candidate=candidate)

    def _joint_plan_pose_for_candidate(
        self,
        target_pose: Pose,
        candidate: dict[str, object] | None,
        use_cartesian_grasp_approach: bool,
    ) -> tuple[Pose, str]:
        if not use_cartesian_grasp_approach:
            return target_pose, "grasp"

        requested = str(self.get_parameter("joint_plan_goal_pose").value).strip().lower()
        if requested in {"", "cartesian_start", "start", "a"}:
            return self._cartesian_start_pose_for_target(target_pose, candidate=candidate), "a"

        if self._cartesian_approach_mode() in {
            "top_view",
            "top_view_dogleg",
            "xy_dogleg",
            "top_view_full",
            "full_top_view",
            "xy_dogleg_full",
            "top_view_staged",
            "xy_dogleg_staged",
        }:
            poses = self._top_view_cartesian_poses(target_pose, candidate=candidate)
            if requested in {"b", "pregrasp"}:
                return poses["b"], "b"
            if requested in {"c_pre", "c-pre", "pre_c", "pre-grasp-c", "cartesian_c_pre"}:
                return poses["c_pre"], "c_pre"
            if requested in {"c", "grasp", "target"}:
                return poses["c"], "grasp"
            self.get_logger().warn(
                f"Unknown joint_plan_goal_pose={requested!r}; falling back to Cartesian A/start pose."
            )
            return poses["a"], "a"

        if requested in {"b", "pregrasp", "c_pre", "c-pre", "pre_c"}:
            return self._pregrasp_pose_for_cartesian_approach(target_pose, candidate=candidate), "pregrasp"
        return target_pose, "grasp"

    def _cartesian_grasp_waypoints(
        self,
        target_pose: Pose,
        include_start: bool = False,
        candidate: dict[str, object] | None = None,
    ) -> list[Pose]:
        if self._cartesian_approach_mode() in {
            "top_view",
            "top_view_dogleg",
            "xy_dogleg",
            "top_view_full",
            "full_top_view",
            "xy_dogleg_full",
            "top_view_staged",
            "xy_dogleg_staged",
        }:
            poses = self._top_view_cartesian_poses(target_pose, candidate=candidate)
            waypoints = [poses["b"], poses["c_pre"]]
            if not self._cartesian_stop_at_c_pre_enabled():
                waypoints.append(poses["c"])
            if include_start:
                waypoints.insert(0, poses["a"])
            return waypoints
        return [target_pose]

    def _top_view_cartesian_poses(
        self,
        target_pose: Pose,
        candidate: dict[str, object] | None = None,
    ) -> dict[str, Pose]:
        approach_axis = self._cartesian_tomato_y_approach_axis(target_pose, candidate=candidate)

        pregrasp_distance = max(0.0, float(self.get_parameter("cartesian_pregrasp_distance").value))
        c_pre_distance = max(0.0, float(self.get_parameter("cartesian_top_view_c_pre_distance").value))
        if pregrasp_distance > 1e-9 and c_pre_distance >= pregrasp_distance:
            requested_c_pre_distance = c_pre_distance
            c_pre_distance = pregrasp_distance * 0.5
            self.get_logger().warn(
                "cartesian_top_view_c_pre_distance must be between C and B. "
                f"requested={requested_c_pre_distance:.3f}m >= B distance={pregrasp_distance:.3f}m; "
                f"using {c_pre_distance:.3f}m instead."
            )

        b_pose = self._pose_on_tomato_y_axis_from_target(target_pose, approach_axis, pregrasp_distance, candidate)
        c_pre_pose = self._pose_on_tomato_y_axis_from_target(target_pose, approach_axis, c_pre_distance, candidate)
        a_pose = self._top_view_a_pose_between_current_and_b(b_pose, target_pose, approach_axis)
        return {
            "entry": a_pose,
            "a": a_pose,
            "pregrasp": b_pose,
            "b": b_pose,
            "c_pre": c_pre_pose,
            "grasp": target_pose,
            "c": target_pose,
        }

    def _pose_on_tomato_y_axis_from_target(
        self,
        target_pose: Pose,
        approach_axis: np.ndarray,
        distance_behind_target: float,
        candidate: dict[str, object] | None,
    ) -> Pose:
        origin_xyz = self._tomato_axis_origin_xyz(candidate)
        axis_xyz = np.array([float(approach_axis[0]), float(approach_axis[1]), float(approach_axis[2])], dtype=float)
        axis_norm = float(np.linalg.norm(axis_xyz))
        if axis_norm < 1e-9:
            return self._offset_pose_xy(target_pose, -approach_axis * distance_behind_target)

        axis_xyz = axis_xyz / axis_norm
        target_xyz = np.array(
            [target_pose.position.x, target_pose.position.y, target_pose.position.z],
            dtype=float,
        )
        reference_xyz, reference_label = self._cartesian_top_view_reference_xyz(target_pose, candidate)
        target_from_reference_xyz = target_xyz - reference_xyz

        if origin_xyz is not None:
            target_reference_axis_distance = float(np.dot(reference_xyz - origin_xyz, axis_xyz))
            reference_pose_xyz = (
                origin_xyz
                + axis_xyz * (target_reference_axis_distance - float(distance_behind_target))
            )
        else:
            reference_pose_xyz = reference_xyz - axis_xyz * float(distance_behind_target)
        pose_xyz = reference_pose_xyz + target_from_reference_xyz

        pose = Pose()
        pose.position.x = float(pose_xyz[0])
        pose.position.y = float(pose_xyz[1])
        pose.position.z = float(pose_xyz[2])
        pose.orientation = target_pose.orientation

        axis_xy = np.array([axis_xyz[0], axis_xyz[1]], dtype=float)
        axis_xy_norm = float(np.linalg.norm(axis_xy))
        if axis_xy_norm > 1e-9:
            axis_xy = axis_xy / axis_xy_norm
        if origin_xyz is not None:
            reference_axis_distance = float(np.dot(reference_pose_xyz - origin_xyz, axis_xyz))
            target_reference_axis_distance = float(np.dot(reference_xyz - origin_xyz, axis_xyz))
            target_reference_off_axis = float(
                np.linalg.norm((reference_xyz - origin_xyz) - axis_xyz * target_reference_axis_distance)
            )
            reference_point_off_axis = float(
                np.linalg.norm((reference_pose_xyz - origin_xyz) - axis_xyz * reference_axis_distance)
            )
            if axis_xy_norm > 1e-9:
                target_reference_axis_distance_xy = float(
                    np.dot(reference_xyz[:2] - origin_xyz[:2], axis_xy)
                )
                reference_axis_distance_xy = float(
                    np.dot(reference_pose_xyz[:2] - origin_xyz[:2], axis_xy)
                )
                target_reference_off_axis_xy = float(
                    np.linalg.norm(
                        (reference_xyz[:2] - origin_xyz[:2])
                        - axis_xy * target_reference_axis_distance_xy
                    )
                )
                reference_point_off_axis_xy = float(
                    np.linalg.norm(
                        (reference_pose_xyz[:2] - origin_xyz[:2])
                        - axis_xy * reference_axis_distance_xy
                    )
                )
            else:
                target_reference_off_axis_xy = float("nan")
                reference_point_off_axis_xy = float("nan")
            self.get_logger().info(
                "Top-view Cartesian point placed so reference enters along tomato 3D Y-axis "
                f"reference={reference_label} "
                f"behind={distance_behind_target:.3f}m "
                f"axis_origin=({origin_xyz[0]:.4f},{origin_xyz[1]:.4f},{origin_xyz[2]:.4f}) "
                f"axis=({axis_xyz[0]:+.4f},{axis_xyz[1]:+.4f},{axis_xyz[2]:+.4f}) "
                f"reference_target_axis_offset_3d={target_reference_off_axis:.4f}m "
                f"reference_point_axis_offset_3d={reference_point_off_axis:.4f}m "
                f"reference_target_axis_offset_xy={target_reference_off_axis_xy:.4f}m "
                f"reference_point_axis_offset_xy={reference_point_off_axis_xy:.4f}m "
                f"reference_xyz=({reference_pose_xyz[0]:.4f},{reference_pose_xyz[1]:.4f},{reference_pose_xyz[2]:.4f}) "
                f"{self.target_link}_xyz=({pose_xyz[0]:.4f},{pose_xyz[1]:.4f},{pose_xyz[2]:.4f})"
            )
        else:
            self.get_logger().info(
                "Top-view Cartesian point placed so reference enters along tomato 3D Y-axis "
                f"reference={reference_label} "
                f"behind={distance_behind_target:.3f}m "
                f"axis=({axis_xyz[0]:+.4f},{axis_xyz[1]:+.4f},{axis_xyz[2]:+.4f}) "
                f"reference_xyz=({reference_pose_xyz[0]:.4f},{reference_pose_xyz[1]:.4f},{reference_pose_xyz[2]:.4f}) "
                f"{self.target_link}_xyz=({pose_xyz[0]:.4f},{pose_xyz[1]:.4f},{pose_xyz[2]:.4f})"
            )
        return pose

    def _cartesian_top_view_reference_xyz(
        self,
        target_pose: Pose,
        candidate: dict[str, object] | None,
    ) -> tuple[np.ndarray, str]:
        target_xyz = np.array(
            [target_pose.position.x, target_pose.position.y, target_pose.position.z],
            dtype=float,
        )
        reference_name = str(self.get_parameter("cartesian_top_view_reference_point").value).strip().lower()
        if not isinstance(candidate, dict):
            return target_xyz, self.target_link

        key_by_name = {
            "tool_tip": "tool_tip_position",
            "tool_tip_link": "tool_tip_position",
            "tool_gripper_spin_link": "tool_tip_position",
            "tool_gripper_link": "tool_gripper_link_position",
            "gripper_link": "tool_gripper_link_position",
            "blue_fingertip": "blue_fingertip_position",
            "blue_tip": "blue_fingertip_position",
            "orange_fingertip": "orange_fingertip_position",
            "orange_tip": "orange_fingertip_position",
            "calyx_fingertip": "calyx_fingertip_position",
            "calyx_tip": "calyx_fingertip_position",
            "opposite_fingertip": "opposite_fingertip_position",
            "opposite_tip": "opposite_fingertip_position",
            "h": "opposite_fingertip_position",
            "grasp_tf": None,
            "target_link": None,
            self.target_link.lower(): None,
        }
        key = key_by_name.get(reference_name, "tool_tip_position")
        if key is None:
            return target_xyz, self.target_link

        position = self._candidate_cartesian_reference_position(candidate, key)
        if position is None:
            return target_xyz, self.target_link
        try:
            reference_position = np.array(position, dtype=float)
        except (TypeError, ValueError):
            return target_xyz, self.target_link
        if reference_position.shape[0] < 3:
            return target_xyz, self.target_link
        return np.array([reference_position[0], reference_position[1], reference_position[2]], dtype=float), reference_name

    def _candidate_cartesian_reference_position(
        self,
        candidate: dict[str, object],
        key: str,
    ) -> np.ndarray | None:
        if key in {"blue_fingertip_position", "orange_fingertip_position"}:
            return self._candidate_fingertip_position(
                candidate,
                "blue" if key == "blue_fingertip_position" else "orange",
            )
        if key == "calyx_fingertip_position":
            finger = str(candidate.get("calyx_finger", "blue")).lower()
            return self._candidate_fingertip_position(candidate, finger)
        if key == "opposite_fingertip_position":
            finger = str(candidate.get("opposite_finger", "orange")).lower()
            return self._candidate_fingertip_position(candidate, finger)
        position = candidate.get(key)
        if position is None:
            return None
        try:
            return np.array(position, dtype=float)
        except (TypeError, ValueError):
            return None

    def _candidate_fingertip_position(
        self,
        candidate: dict[str, object],
        finger: str,
    ) -> np.ndarray | None:
        grasp_pose = candidate.get("grasp_pose")
        if not isinstance(grasp_pose, Pose):
            return None
        grasp_position = np.array(
            [
                float(grasp_pose.position.x),
                float(grasp_pose.position.y),
                float(grasp_pose.position.z),
            ],
            dtype=float,
        )
        grasp_rotation = matrix_from_quaternion(
            [
                grasp_pose.orientation.x,
                grasp_pose.orientation.y,
                grasp_pose.orientation.z,
                grasp_pose.orientation.w,
            ]
        )
        blue_offset, orange_offset = self._grasp_tf_to_inner_fingertip_offsets()
        if str(finger).lower() == "blue":
            return grasp_position + grasp_rotation @ blue_offset
        if str(finger).lower() == "orange":
            return grasp_position + grasp_rotation @ orange_offset
        return None

    def _tomato_axis_origin_xyz(self, candidate: dict[str, object] | None) -> np.ndarray | None:
        tf_context = self._tomato_tf_axis_context(candidate)
        if tf_context is not None:
            origin_xyz, _, frame_id = tf_context
            self.get_logger().info(
                f"Using RViz tomato TF origin for Cartesian top-view 3D line frame={frame_id} "
                f"origin=({origin_xyz[0]:.4f},{origin_xyz[1]:.4f},{origin_xyz[2]:.4f})"
            )
            return origin_xyz
        if not isinstance(candidate, dict):
            return None
        object_position = candidate.get("object_position")
        if object_position is None:
            return None
        try:
            position = np.array(object_position, dtype=float)
        except (TypeError, ValueError):
            return None
        if position.shape[0] < 3:
            return None
        return np.array([position[0], position[1], position[2]], dtype=float)

    def _top_view_a_pose_between_current_and_b(
        self,
        b_pose: Pose,
        target_pose: Pose,
        approach_axis: np.ndarray,
    ) -> Pose:
        current_pose = self._current_target_link_pose()
        if current_pose is None:
            self.get_logger().warn(
                f"Could not read current {self.target_link} pose for top-view A point; "
                "falling back to B pose."
            )
            return self._copy_pose_with_target_orientation(b_pose, target_pose)

        fraction = float(self.get_parameter("cartesian_top_view_a_fraction").value)
        fraction = float(np.clip(fraction, 0.0, 1.0))
        axis_xy = np.array([float(approach_axis[0]), float(approach_axis[1])], dtype=float)
        axis_xy_norm = float(np.linalg.norm(axis_xy))
        if axis_xy_norm < 1e-9:
            self.get_logger().warn(
                "Could not compute tomato Y-axis top-view direction for A point; "
                "falling back to the old current-to-B interpolation."
            )
            a_pose = Pose()
            a_pose.position.x = float(current_pose.position.x + (b_pose.position.x - current_pose.position.x) * fraction)
            a_pose.position.y = float(current_pose.position.y + (b_pose.position.y - current_pose.position.y) * fraction)
            a_pose.position.z = float(b_pose.position.z)
            a_pose.orientation = target_pose.orientation
            return a_pose

        axis_xy = axis_xy / axis_xy_norm
        perpendicular_xy = np.array([-axis_xy[1], axis_xy[0]], dtype=float)
        current_xy = np.array([current_pose.position.x, current_pose.position.y], dtype=float)
        b_xy = np.array([b_pose.position.x, b_pose.position.y], dtype=float)
        current_from_b_xy = current_xy - b_xy

        current_along_axis_offset = float(np.dot(current_from_b_xy, axis_xy))
        current_perpendicular_offset = float(np.dot(current_from_b_xy, perpendicular_xy))
        a_perpendicular_offset = current_perpendicular_offset * fraction

        if abs(a_perpendicular_offset) < 1e-4:
            lane_distance = max(0.0, float(self.get_parameter("cartesian_top_view_lane_distance").value))
            lane_sign = 1.0 if float(self.get_parameter("cartesian_top_view_lane_sign").value) >= 0.0 else -1.0
            a_perpendicular_offset = lane_sign * lane_distance
            self.get_logger().warn(
                "Current gripper is almost on the B->C axis; "
                f"using fallback A lane offset={a_perpendicular_offset:.3f}m."
            )

        a_xy = b_xy + perpendicular_xy * a_perpendicular_offset
        a_pose = Pose()
        a_pose.position.x = float(a_xy[0])
        a_pose.position.y = float(a_xy[1])
        a_pose.position.z = float(b_pose.position.z)
        a_pose.orientation = target_pose.orientation
        self.get_logger().info(
            "Top-view Cartesian A point doglegs into B instead of cutting diagonally "
            f"fraction={fraction:.2f} "
            f"tomato_y_axis_xy=({axis_xy[0]:+.4f},{axis_xy[1]:+.4f}) "
            f"A_to_B_perpendicular_axis=({perpendicular_xy[0]:+.4f},{perpendicular_xy[1]:+.4f}) "
            f"current_along_BC_axis_offset={current_along_axis_offset:.4f}m "
            f"A_along_BC_axis_offset=0.0000m "
            f"current_perpendicular_offset={current_perpendicular_offset:.4f}m "
            f"A_perpendicular_offset={a_perpendicular_offset:.4f}m "
            f"current_{self.target_link}_xyz=({current_pose.position.x:.4f},{current_pose.position.y:.4f},{current_pose.position.z:.4f}) "
            f"B_xyz=({b_pose.position.x:.4f},{b_pose.position.y:.4f},{b_pose.position.z:.4f}) "
            f"A_xyz=({a_pose.position.x:.4f},{a_pose.position.y:.4f},{a_pose.position.z:.4f})"
        )
        return a_pose

    def _current_target_link_pose(self) -> Pose | None:
        timeout_sec = max(0.0, float(self.get_parameter("cartesian_current_pose_timeout_sec").value))
        deadline = time.monotonic() + timeout_sec
        last_exception: TransformException | None = None
        transform = None
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.target_link,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=min(0.10, max(0.0, remaining))),
                )
                break
            except TransformException as exc:
                last_exception = exc
                if remaining <= 0.0:
                    break
                rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, remaining)))

        if transform is None:
            if last_exception is not None:
                self.get_logger().warn(
                    f"Could not look up current {self.target_link} pose within {timeout_sec:.2f}s: {last_exception}"
                )
            return None

        pose = Pose()
        pose.position.x = float(transform.transform.translation.x)
        pose.position.y = float(transform.transform.translation.y)
        pose.position.z = float(transform.transform.translation.z)
        pose.orientation.x = float(transform.transform.rotation.x)
        pose.orientation.y = float(transform.transform.rotation.y)
        pose.orientation.z = float(transform.transform.rotation.z)
        pose.orientation.w = float(transform.transform.rotation.w)
        return pose

    def _copy_pose_with_target_orientation(self, pose: Pose, target_pose: Pose) -> Pose:
        copied = Pose()
        copied.position.x = float(pose.position.x)
        copied.position.y = float(pose.position.y)
        copied.position.z = float(pose.position.z)
        copied.orientation = target_pose.orientation
        return copied

    def _cartesian_xy_approach_axis(
        self,
        target_pose: Pose,
        candidate: dict[str, object] | None = None,
    ) -> np.ndarray:
        candidate_axis = self._candidate_tomato_y_approach_axis_xy(candidate)
        if candidate_axis is not None:
            return candidate_axis

        rotation = matrix_from_quaternion(
            [
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            ]
        )
        approach_axis = rotation[:, 2]
        approach_axis = np.array([approach_axis[0], approach_axis[1], 0.0], dtype=float)
        norm = float(np.linalg.norm(approach_axis))
        if norm < 1e-6:
            approach_axis = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            approach_axis = approach_axis / norm
        return approach_axis

    def _cartesian_tomato_y_approach_axis(
        self,
        target_pose: Pose,
        candidate: dict[str, object] | None = None,
    ) -> np.ndarray:
        candidate_axis = self._candidate_tomato_y_approach_axis(candidate)
        if candidate_axis is not None:
            return candidate_axis

        rotation = matrix_from_quaternion(
            [
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            ]
        )
        approach_axis = rotation[:, 2]
        norm = float(np.linalg.norm(approach_axis))
        if norm < 1e-6:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return approach_axis / norm

    def _candidate_tomato_y_approach_axis_xy(self, candidate: dict[str, object] | None) -> np.ndarray | None:
        axis = self._candidate_tomato_y_approach_axis(candidate)
        if axis is None:
            return None
        axis_xy = np.array([axis[0], axis[1], 0.0], dtype=float)
        norm = float(np.linalg.norm(axis_xy))
        if norm < 1e-9:
            return None
        return axis_xy / norm

    def _candidate_tomato_y_approach_axis(self, candidate: dict[str, object] | None) -> np.ndarray | None:
        tf_context = self._tomato_tf_axis_context(candidate)
        if tf_context is not None:
            _, axis, frame_id = tf_context
            self.get_logger().info(
                f"Using RViz tomato TF 3D Y-axis for Cartesian top-view approach frame={frame_id} "
                f"axis=({axis[0]:+.4f},{axis[1]:+.4f},{axis[2]:+.4f})"
            )
            return axis
        if not isinstance(candidate, dict):
            return None
        tomato_rotation = candidate.get("tomato_rotation")
        if tomato_rotation is None:
            return None
        try:
            rotation = np.array(tomato_rotation, dtype=float)
        except (TypeError, ValueError):
            return None
        if rotation.shape != (3, 3):
            return None
        approach_sign = float(candidate.get("approach_sign", self._tomato_y_back_approach_sign()))
        axis = approach_sign * rotation[:, 1]
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            return None
        return axis / norm

    def _tomato_tf_axis_context(
        self,
        candidate: dict[str, object] | None,
    ) -> tuple[np.ndarray, np.ndarray, str] | None:
        if not isinstance(candidate, dict):
            return None
        cached = candidate.get("_tomato_tf_axis_context")
        if isinstance(cached, tuple) and len(cached) == 3:
            return cached

        if self._candidate_is_vine_row_target(candidate):
            return None

        frame_id = self._tomato_tf_frame_id(candidate)
        if not frame_id:
            return None

        timeout_sec = max(0.0, float(self.get_parameter("cartesian_current_pose_timeout_sec").value))
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=min(0.10, timeout_sec)),
            )
        except TransformException:
            return None

        origin_xyz = np.array(
            [
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                float(transform.transform.translation.z),
            ],
            dtype=float,
        )
        rotation = matrix_from_quaternion(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        )
        approach_sign = float(candidate.get("approach_sign", self._tomato_y_back_approach_sign()))
        axis = approach_sign * rotation[:, 1]
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            return None

        context = (origin_xyz, axis / norm, frame_id)
        candidate["_tomato_tf_axis_context"] = context
        return context

    def _candidate_is_vine_row_target(self, candidate: dict[str, object]) -> bool:
        try:
            row_index = int(candidate.get("entry_vine_row_index", 0))
        except (TypeError, ValueError):
            row_index = 0
        try:
            row_y_offset = float(candidate.get("entry_vine_row_y_offset", 0.0))
        except (TypeError, ValueError):
            row_y_offset = 0.0
        return row_index != 0 or abs(row_y_offset) > 1e-9

    def _tomato_tf_frame_id(self, candidate: dict[str, object]) -> str | None:
        frame_id = candidate.get("tomato_tf_frame")
        if isinstance(frame_id, str) and frame_id:
            return frame_id
        entry_index = candidate.get("entry_index")
        try:
            return f"tomato_{int(entry_index)}_tf"
        except (TypeError, ValueError):
            return None

    def _offset_pose_xy(self, pose: Pose, offset: np.ndarray) -> Pose:
        offset = np.array(offset, dtype=float)
        shifted = Pose()
        shifted.position.x = float(pose.position.x + offset[0])
        shifted.position.y = float(pose.position.y + offset[1])
        shifted.position.z = float(pose.position.z)
        shifted.orientation = pose.orientation
        return shifted

    def _compute_cartesian_path_to_pose(
        self,
        target_pose: Pose,
        waypoints: list[Pose] | None = None,
        fixed_gripper_spin_deg: float | None = None,
    ) -> dict[str, object]:
        if waypoints is None:
            waypoints = [target_pose]
        if not waypoints:
            waypoints = [target_pose]
        if self._cartesian_approach_mode() in {
            "top_view",
            "top_view_dogleg",
            "xy_dogleg",
            "top_view_full",
            "full_top_view",
            "xy_dogleg_full",
            "top_view_staged",
            "xy_dogleg_staged",
        }:
            stop_at_c_pre = self._cartesian_stop_at_c_pre_enabled()
            if stop_at_c_pre and len(waypoints) == 3:
                labels = ["A", "B", "C_pre"]
            elif stop_at_c_pre and len(waypoints) == 2:
                labels = ["B", "C_pre"]
            elif len(waypoints) == 4:
                labels = ["A", "B", "C_pre", "C"]
            elif len(waypoints) == 3:
                labels = ["B", "C_pre", "C"]
            else:
                labels = ["B", "C"]
        else:
            labels = []
        waypoint_summary_parts = []
        for index, pose in enumerate(waypoints):
            label = labels[index] if index < len(labels) else f"W{index + 1}"
            waypoint_summary_parts.append(
                f"{label}=({pose.position.x:.4f},{pose.position.y:.4f},{pose.position.z:.4f})"
            )
        waypoint_summary = " -> ".join(waypoint_summary_parts)
        self.get_logger().info(f"Cartesian waypoints for {self.target_link}: {waypoint_summary}")

        timeout_sec = float(self.get_parameter("cartesian_service_timeout_sec").value)
        if not self.cartesian_path_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("compute_cartesian_path service was not found.")
            return {"success": False, "error_code": None, "trajectory": None, "planning_time": 0.0, "fraction": 0.0}

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.start_state.is_diff = True
        request.group_name = self.group_name
        request.link_name = self.target_link
        request.waypoints = list(waypoints)
        request.max_step = max(1e-4, float(self.get_parameter("cartesian_max_step").value))
        request.jump_threshold = max(0.0, float(self.get_parameter("cartesian_jump_threshold").value))
        request.prismatic_jump_threshold = max(
            0.0,
            float(self.get_parameter("cartesian_prismatic_jump_threshold").value),
        )
        request.revolute_jump_threshold = max(
            0.0,
            float(self.get_parameter("cartesian_revolute_jump_threshold").value),
        )
        request.avoid_collisions = bool(self.get_parameter("cartesian_avoid_collisions").value)
        if (
            fixed_gripper_spin_deg is not None
            and bool(self.get_parameter("cartesian_lock_gripper_spin").value)
        ):
            tolerance_rad = math.radians(
                max(0.0, float(self.get_parameter("cartesian_fixed_gripper_spin_tolerance_deg").value))
            )
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = "tool_gripper_z_joint"
            joint_constraint.position = math.radians(float(fixed_gripper_spin_deg))
            joint_constraint.tolerance_above = tolerance_rad
            joint_constraint.tolerance_below = tolerance_rad
            joint_constraint.weight = 1.0
            constraints = Constraints()
            constraints.name = "fixed_tool_gripper_z_joint_for_cartesian_path"
            constraints.joint_constraints = [joint_constraint]
            request.path_constraints = constraints
            self.get_logger().info(
                "Cartesian path locks tool_gripper_z_joint "
                f"to {float(fixed_gripper_spin_deg):.1f}deg "
                f"+/-{math.degrees(tolerance_rad):.1f}deg"
            )

        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(f"compute_cartesian_path did not return within {timeout_sec:.1f}s.")
            return {"success": False, "error_code": None, "trajectory": None, "planning_time": 0.0, "fraction": 0.0}
        response = future.result()
        if response is None:
            self.get_logger().error("compute_cartesian_path returned no response.")
            return {"success": False, "error_code": None, "trajectory": None, "planning_time": 0.0, "fraction": 0.0}

        trajectory = response.solution
        self._ensure_trajectory_timing(trajectory)
        self._retime_top_view_b_to_c_segment(trajectory, waypoints)
        fraction = float(response.fraction)
        point_count = len(trajectory.joint_trajectory.points)
        fraction_threshold = float(self.get_parameter("cartesian_fraction_threshold").value)
        success = (
            response.error_code.val == 1
            and fraction >= fraction_threshold
            and point_count > 0
        )
        joint_names = ", ".join(trajectory.joint_trajectory.joint_names)
        self.get_logger().info(
            "compute_cartesian_path result "
            f"success={success} error_code={response.error_code.val} "
            f"fraction={fraction:.3f}/{fraction_threshold:.3f} "
            f"points={point_count} joints=[{joint_names}] "
            f"max_step={request.max_step:.4f} avoid_collisions={request.avoid_collisions}"
        )
        return {
            "success": success,
            "error_code": response.error_code.val,
            "trajectory": trajectory,
            "planning_time": 0.0,
            "fraction": fraction,
            "point_count": point_count,
            "joint_names": list(trajectory.joint_trajectory.joint_names),
        }

    def _retime_top_view_b_to_c_segment(self, trajectory, waypoints: list[Pose]) -> None:
        if not bool(self.get_parameter("cartesian_slow_b_to_c").value):
            return
        if self._cartesian_approach_mode() not in {
            "top_view",
            "top_view_dogleg",
            "xy_dogleg",
            "top_view_full",
            "full_top_view",
            "xy_dogleg_full",
            "top_view_staged",
            "xy_dogleg_staged",
        }:
            return
        if len(waypoints) < 3:
            return

        points = list(trajectory.joint_trajectory.points)
        if len(points) < 2:
            return

        current_pose = self._current_target_link_pose()
        path_poses = ([current_pose] if current_pose is not None else []) + list(waypoints)
        if len(path_poses) < 2:
            return

        positions = [
            np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            for pose in path_poses
        ]
        segment_lengths = [
            float(np.linalg.norm(positions[index + 1] - positions[index]))
            for index in range(len(positions) - 1)
        ]
        total_length = float(sum(segment_lengths))
        if total_length <= 1e-9:
            return

        # Full path: current -> A -> B -> C_pre -> C, so B->C starts after two segments.
        # Staged path: current(A) -> B -> C_pre -> C, so B->C starts after one segment.
        # When stopping at C_pre, slow only the final B->C_pre segment.
        if self._cartesian_stop_at_c_pre_enabled():
            before_b_to_c_segments = max(0, len(segment_lengths) - 1)
        elif len(waypoints) >= 4:
            before_b_to_c_segments = 2 if current_pose is not None else 1
        else:
            before_b_to_c_segments = 1 if current_pose is not None else 0
        before_b_to_c_segments = max(0, min(before_b_to_c_segments, len(segment_lengths)))
        before_b_to_c_length = float(sum(segment_lengths[:before_b_to_c_segments]))
        split_fraction = float(np.clip(before_b_to_c_length / total_length, 0.0, 1.0))
        split_index = int(round(split_fraction * max(0, len(points) - 1)))
        split_index = max(1, min(split_index, len(points) - 1))

        base_step_sec = max(0.01, float(self.get_parameter("cartesian_step_duration_sec").value))
        b_to_c_step_sec = max(base_step_sec, float(self.get_parameter("cartesian_b_to_c_step_duration_sec").value))
        joint_count = len(trajectory.joint_trajectory.joint_names)

        elapsed_sec = 0.0
        for index, point in enumerate(points):
            step_sec = base_step_sec if index < split_index else b_to_c_step_sec
            elapsed_sec += step_sec
            point.time_from_start = Duration(seconds=elapsed_sec).to_msg()
            if len(point.velocities) != joint_count:
                point.velocities = [0.0] * joint_count
            if len(point.accelerations) != joint_count:
                point.accelerations = [0.0] * joint_count

        self.get_logger().info(
            "Retimed top-view Cartesian B->C segment "
            f"split_point={split_index}/{len(points)} "
            f"base_step={base_step_sec:.3f}s "
            f"b_to_c_step={b_to_c_step_sec:.3f}s "
            f"before_b_to_c_length={before_b_to_c_length:.4f}m "
            f"total_length={total_length:.4f}m"
        )

    def _ensure_trajectory_timing(self, trajectory) -> None:
        points = list(trajectory.joint_trajectory.points)
        if not points:
            return
        needs_timing = False
        previous = -1.0
        for point in points:
            stamp = point.time_from_start
            current = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            if current <= previous:
                needs_timing = True
                break
            previous = current
        if not needs_timing and previous > 0.0:
            return

        step_sec = max(0.01, float(self.get_parameter("cartesian_step_duration_sec").value))
        joint_count = len(trajectory.joint_trajectory.joint_names)
        for index, point in enumerate(points, start=1):
            point.time_from_start = Duration(seconds=step_sec * index).to_msg()
            if len(point.velocities) != joint_count:
                point.velocities = [0.0] * joint_count
            if len(point.accelerations) != joint_count:
                point.accelerations = [0.0] * joint_count

    def _trajectory_final_joint_deg(self, trajectory, joint_name: str, fallback_deg: float) -> float:
        joint_names = list(trajectory.joint_trajectory.joint_names)
        points = list(trajectory.joint_trajectory.points)
        if joint_name not in joint_names or not points:
            return float(fallback_deg)
        joint_index = joint_names.index(joint_name)
        final_point = points[-1]
        if len(final_point.positions) <= joint_index:
            return float(fallback_deg)
        return math.degrees(float(final_point.positions[joint_index]))

    def _gripper_spin_deg_for_entry(self, entry: dict[str, object]) -> float:
        if str(entry["side"]) == "left":
            return self._clamp_grasp_gripper_spin_deg(float(self.get_parameter("left_gripper_spin_deg").value))
        return self._clamp_grasp_gripper_spin_deg(float(self.get_parameter("right_gripper_spin_deg").value))

    def _gripper_spin_candidates_for_entry(self, entry: dict[str, object]) -> list[float]:
        min_spin_deg = float(self.get_parameter("min_grasp_gripper_spin_deg").value)
        max_spin_deg = float(self.get_parameter("max_grasp_gripper_spin_deg").value)
        if min_spin_deg > max_spin_deg:
            min_spin_deg, max_spin_deg = max_spin_deg, min_spin_deg
        preferred_spin_deg = self._clamp_grasp_gripper_spin_deg(self._gripper_spin_deg_for_entry(entry))
        if not bool(self.get_parameter("use_gripper_spin_search").value):
            return [preferred_spin_deg]

        step_deg = abs(float(self.get_parameter("gripper_spin_step_deg").value))
        if step_deg <= 0.0:
            step_deg = 30.0
        max_candidates = int(self.get_parameter("max_gripper_spin_candidates").value)
        span = max_spin_deg - min_spin_deg
        max_unique = max(1, int(math.floor(span / step_deg)) + 1)

        candidates: list[float] = []
        seen: set[int] = set()

        def add(angle_deg: float) -> None:
            if angle_deg < min_spin_deg - 1e-9 or angle_deg > max_spin_deg + 1e-9:
                return
            clamped = self._clamp_grasp_gripper_spin_deg(angle_deg)
            key = int(round(clamped * 1000.0))
            if key in seen:
                return
            seen.add(key)
            candidates.append(clamped)

        add(preferred_spin_deg)
        for offset_index in range(1, max_unique + 1):
            offset = step_deg * offset_index
            add(preferred_spin_deg + offset)
            add(preferred_spin_deg - offset)
            if len(candidates) >= max_unique:
                break

        if max_candidates > 0:
            candidates = candidates[:max_candidates]

        self.get_logger().info(
            "Gripper spin search candidates "
            f"side={entry['side']} preferred={preferred_spin_deg:.1f}deg "
            f"range=[{min_spin_deg:.1f}, {max_spin_deg:.1f}]deg "
            f"step={step_deg:.1f}deg count={len(candidates)}: "
            + ", ".join(f"{angle:.1f}" for angle in candidates)
        )
        return candidates

    def _clamp_grasp_gripper_spin_deg(self, angle_deg: float) -> float:
        min_spin_deg = float(self.get_parameter("min_grasp_gripper_spin_deg").value)
        max_spin_deg = float(self.get_parameter("max_grasp_gripper_spin_deg").value)
        if min_spin_deg > max_spin_deg:
            min_spin_deg, max_spin_deg = max_spin_deg, min_spin_deg
        min_joint_spin_deg = float(self.get_parameter("min_gripper_spin_deg").value)
        max_joint_spin_deg = float(self.get_parameter("max_gripper_spin_deg").value)
        if min_joint_spin_deg > max_joint_spin_deg:
            min_joint_spin_deg, max_joint_spin_deg = max_joint_spin_deg, min_joint_spin_deg
        min_spin_deg = max(min_spin_deg, min_joint_spin_deg)
        max_spin_deg = min(max_spin_deg, max_joint_spin_deg)
        return float(np.clip(float(angle_deg), min_spin_deg, max_spin_deg))

    def _grasp_gripper_spin_rejection_reason(self, angle_deg: float) -> str:
        min_spin_deg = float(self.get_parameter("min_grasp_gripper_spin_deg").value)
        max_spin_deg = float(self.get_parameter("max_grasp_gripper_spin_deg").value)
        if min_spin_deg > max_spin_deg:
            min_spin_deg, max_spin_deg = max_spin_deg, min_spin_deg
        angle_deg = float(angle_deg)
        tolerance_deg = max(
            0.0,
            float(self.get_parameter("grasp_gripper_spin_final_tolerance_deg").value),
        )
        if angle_deg < min_spin_deg - tolerance_deg or angle_deg > max_spin_deg + tolerance_deg:
            return (
                "final gripper spin at grasp is outside approach limit "
                f"({angle_deg:.1f}deg not in [{min_spin_deg:.1f}, {max_spin_deg:.1f}]deg "
                f"+/-{tolerance_deg:.1f}deg tolerance)"
            )
        return ""

    def _grasp_gripper_spin_trajectory_rejection_reason(self, trajectory, fallback_angle_deg: float) -> str:
        if trajectory is None or not trajectory.joint_trajectory.points:
            return self._grasp_gripper_spin_rejection_reason(fallback_angle_deg)

        joint_names = list(trajectory.joint_trajectory.joint_names)
        if "tool_gripper_z_joint" not in joint_names:
            return self._grasp_gripper_spin_rejection_reason(fallback_angle_deg)

        joint_index = joint_names.index("tool_gripper_z_joint")
        min_spin_deg = float(self.get_parameter("min_grasp_gripper_spin_deg").value)
        max_spin_deg = float(self.get_parameter("max_grasp_gripper_spin_deg").value)
        if min_spin_deg > max_spin_deg:
            min_spin_deg, max_spin_deg = max_spin_deg, min_spin_deg
        path_tolerance_deg = max(
            0.0,
            float(self.get_parameter("grasp_gripper_spin_limit_tolerance_deg").value),
        )
        points = list(trajectory.joint_trajectory.points)

        final_point = points[-1]
        final_angle_deg = fallback_angle_deg
        if len(final_point.positions) > joint_index:
            final_angle_deg = math.degrees(float(final_point.positions[joint_index]))
        final_rejection_reason = self._grasp_gripper_spin_rejection_reason(final_angle_deg)
        if final_rejection_reason:
            return final_rejection_reason

        for point_index, point in enumerate(points[:-1], start=1):
            if len(point.positions) <= joint_index:
                continue
            angle_deg = math.degrees(float(point.positions[joint_index]))
            if angle_deg < min_spin_deg - path_tolerance_deg or angle_deg > max_spin_deg + path_tolerance_deg:
                return (
                    "gripper spin during approach trajectory is outside approach limit "
                    f"(point={point_index}/{len(points)} angle={angle_deg:.1f}deg "
                    f"not in [{min_spin_deg:.1f}, {max_spin_deg:.1f}]deg "
                    f"+/-{path_tolerance_deg:.1f}deg path tolerance)"
                )
        return ""

    def _clamp_gripper_spin_deg(self, angle_deg: float) -> float:
        min_spin_deg = float(self.get_parameter("min_gripper_spin_deg").value)
        max_spin_deg = float(self.get_parameter("max_gripper_spin_deg").value)
        if min_spin_deg > max_spin_deg:
            min_spin_deg, max_spin_deg = max_spin_deg, min_spin_deg
        return float(np.clip(float(angle_deg), min_spin_deg, max_spin_deg))

    def _run_post_grasp_twist(self) -> None:
        base_spin_deg = float(getattr(self, "current_gripper_spin_deg", 0.0))
        requested_direction_sign = self._post_grasp_twist_direction_sign()
        twist_angle_deg = abs(float(self.get_parameter("grasp_twist_angle_deg").value))
        direction_sign = self._bounded_post_grasp_twist_direction_sign(
            base_spin_deg,
            requested_direction_sign,
            twist_angle_deg,
        )
        duration_sec = max(0.0, float(self.get_parameter("grasp_twist_segment_duration_sec").value))
        steps = max(1, int(self.get_parameter("grasp_twist_steps_per_segment").value))
        requested_targets = self._post_grasp_twist_targets(
            base_spin_deg,
            direction_sign,
            twist_angle_deg,
        )

        self.get_logger().info(
            "Post-grasp gripper Z twist start "
            f"mode={str(self.get_parameter('grasp_twist_direction_mode').value).strip().lower()} "
            f"base={base_spin_deg:.1f}deg direction={direction_sign:+.0f} "
            f"requested_direction={requested_direction_sign:+.0f} "
            f"relative_angle={direction_sign * twist_angle_deg:+.1f}deg "
            f"duration_per_segment={duration_sec:.2f}s steps={steps}"
        )
        for segment_index, target_command in enumerate(requested_targets, start=1):
            if isinstance(target_command, tuple) and target_command[0] == "wrap":
                wrap_target_deg = float(target_command[1])
                self.get_logger().info(
                    "Post-grasp gripper twist wraps equivalent joint angle "
                    f"{self.current_gripper_spin_deg:.1f}deg -> {wrap_target_deg:.1f}deg"
                )
                self._set_runtime_gripper_spin_deg(wrap_target_deg, log=False)
                continue
            requested_target = float(target_command)
            target_deg = self._clamp_gripper_spin_deg(requested_target)
            if abs(target_deg - requested_target) > 1e-6:
                self.get_logger().info(
                    "Post-grasp gripper twist target clamped "
                    f"segment={segment_index} requested={requested_target:.1f}deg applied={target_deg:.1f}deg"
                )
            self._animate_gripper_spin_to(target_deg, duration_sec, steps)
        self.get_logger().info(
            f"Post-grasp gripper Z twist complete; returned to {self.current_gripper_spin_deg:.1f}deg."
        )

    def _post_grasp_twist_targets(
        self,
        base_spin_deg: float,
        direction_sign: float,
        twist_angle_deg: float,
    ) -> list[float | tuple[str, float]]:
        base_spin_deg = float(base_spin_deg)
        direction_sign = 1.0 if float(direction_sign) >= 0.0 else -1.0
        twist_angle_deg = abs(float(twist_angle_deg))
        min_spin_deg = float(self.get_parameter("min_gripper_spin_deg").value)
        max_spin_deg = float(self.get_parameter("max_gripper_spin_deg").value)
        if min_spin_deg > max_spin_deg:
            min_spin_deg, max_spin_deg = max_spin_deg, min_spin_deg

        requested_target = base_spin_deg + direction_sign * twist_angle_deg
        span_deg = max_spin_deg - min_spin_deg
        if span_deg >= 359.0 and requested_target > max_spin_deg:
            overflow_deg = requested_target - max_spin_deg
            wrapped_target = min_spin_deg + overflow_deg
            return [max_spin_deg, ("wrap", min_spin_deg), wrapped_target, min_spin_deg, ("wrap", max_spin_deg), base_spin_deg]
        if span_deg >= 359.0 and requested_target < min_spin_deg:
            underflow_deg = min_spin_deg - requested_target
            wrapped_target = max_spin_deg - underflow_deg
            return [min_spin_deg, ("wrap", max_spin_deg), wrapped_target, max_spin_deg, ("wrap", min_spin_deg), base_spin_deg]
        return [requested_target, base_spin_deg]

    def _bounded_post_grasp_twist_direction_sign(
        self,
        base_spin_deg: float,
        direction_sign: float,
        twist_angle_deg: float,
    ) -> float:
        direction_sign = 1.0 if float(direction_sign) >= 0.0 else -1.0
        min_spin_deg = float(self.get_parameter("min_gripper_spin_deg").value)
        max_spin_deg = float(self.get_parameter("max_gripper_spin_deg").value)
        if min_spin_deg > max_spin_deg:
            min_spin_deg, max_spin_deg = max_spin_deg, min_spin_deg

        requested_target = float(base_spin_deg) + direction_sign * float(twist_angle_deg)
        opposite_target = float(base_spin_deg) - direction_sign * float(twist_angle_deg)
        requested_clamped = float(np.clip(requested_target, min_spin_deg, max_spin_deg))
        opposite_clamped = float(np.clip(opposite_target, min_spin_deg, max_spin_deg))
        requested_motion = abs(requested_clamped - float(base_spin_deg))
        opposite_motion = abs(opposite_clamped - float(base_spin_deg))

        if requested_motion + 1e-6 < min(opposite_motion, abs(float(twist_angle_deg)) * 0.5):
            mode = str(self.get_parameter("grasp_twist_direction_mode").value).strip().lower()
            if mode in {"tomato_z_to_y", "tomato_z_y", "z_to_y"}:
                self.get_logger().warn(
                    "Post-grasp gripper twist direction is blocked by the spin joint limit, "
                    "but tomato_z_to_y mode will not flip it to the opposite visual direction "
                    f"base={base_spin_deg:.1f}deg requested_target={requested_target:.1f}deg "
                    f"clamped={requested_clamped:.1f}deg requested_motion={requested_motion:.1f}deg "
                    f"range=[{min_spin_deg:.1f}, {max_spin_deg:.1f}]deg"
                )
                return direction_sign
            flipped_direction = -direction_sign
            self.get_logger().info(
                "Post-grasp gripper twist direction flipped because the requested first segment "
                "is blocked by the spin joint limit "
                f"base={base_spin_deg:.1f}deg requested_target={requested_target:.1f}deg "
                f"clamped={requested_clamped:.1f}deg requested_motion={requested_motion:.1f}deg "
                f"opposite_target={opposite_target:.1f}deg opposite_clamped={opposite_clamped:.1f}deg "
                f"opposite_motion={opposite_motion:.1f}deg "
                f"range=[{min_spin_deg:.1f}, {max_spin_deg:.1f}]deg "
                f"direction={direction_sign:+.0f}->{flipped_direction:+.0f}"
            )
            return flipped_direction

        return direction_sign

    def _post_grasp_twist_direction_sign(self) -> float:
        mode = str(self.get_parameter("grasp_twist_direction_mode").value).strip().lower()
        if mode in {"positive", "plus", "+", "+1", "ccw"}:
            return 1.0
        if mode in {"negative", "minus", "-", "-1", "cw"}:
            return -1.0
        if mode in {"tomato_z_to_y", "tomato_z_y", "z_to_y"}:
            return self._tomato_z_to_y_twist_direction_sign()
        if mode not in {"calyx_stem", "stem_calyx", "auto"}:
            self.get_logger().warn(
                f"Unknown grasp_twist_direction_mode={mode!r}; falling back to calyx_stem."
            )
        return self._calyx_stem_twist_direction_sign()

    def _tomato_z_to_y_twist_direction_sign(self) -> float:
        record = self.last_grasp_record
        if not record:
            self.get_logger().warn("No last grasp record is available; using positive post-grasp twist direction.")
            return 1.0

        entry = record.get("entry", {})
        candidate = record.get("candidate", {})
        if not isinstance(entry, dict) or not isinstance(candidate, dict):
            self.get_logger().warn("Last grasp record has no entry/candidate data; using positive twist direction.")
            return 1.0

        return self._tomato_z_to_y_twist_direction_sign_from(entry, candidate)

    def _tomato_z_to_y_twist_direction_sign_from(
        self,
        entry: dict[str, object],
        candidate: dict[str, object],
        log: bool = True,
    ) -> float:
        tomato_rotation = np.array(candidate.get("tomato_rotation"), dtype=float)
        if tomato_rotation.shape != (3, 3):
            tomato_rotation = self._base_to_tomato_transform(entry)[:3, :3]

        spin_axis = self._tomato_z_to_y_reference_axis(tomato_rotation, candidate)
        tomato_y = tomato_rotation[:, 1]
        tomato_z = tomato_rotation[:, 2]
        spin_axis_norm = float(np.linalg.norm(spin_axis))
        tomato_y_norm = float(np.linalg.norm(tomato_y))
        tomato_z_norm = float(np.linalg.norm(tomato_z))
        if spin_axis_norm < 1e-9 or tomato_y_norm < 1e-9 or tomato_z_norm < 1e-9:
            self.get_logger().warn(
                "Tomato Z-to-Y twist direction has a degenerate axis; using positive direction."
            )
            return 1.0

        spin_axis = spin_axis / spin_axis_norm
        tomato_y = tomato_y / tomato_y_norm
        tomato_z = tomato_z / tomato_z_norm

        positive_toward_y = float(np.dot(np.cross(spin_axis, tomato_z), tomato_y))
        if abs(positive_toward_y) < 1e-6:
            self.get_logger().warn(
                "Tomato Z-to-Y twist direction is nearly ambiguous; using positive direction. "
                f"positive_toward_y={positive_toward_y:+.6f}"
            )
            return 1.0

        direction_sign = 1.0 if positive_toward_y > 0.0 else -1.0
        if log:
            self.get_logger().info(
                "Tomato Z-to-Y post-grasp twist direction "
                f"positive_toward_y={positive_toward_y:+.4f} direction={direction_sign:+.0f} "
                f"spin_axis=({spin_axis[0]:+.4f},{spin_axis[1]:+.4f},{spin_axis[2]:+.4f}) "
                f"tomato_y=({tomato_y[0]:+.4f},{tomato_y[1]:+.4f},{tomato_y[2]:+.4f}) "
                f"tomato_z=({tomato_z[0]:+.4f},{tomato_z[1]:+.4f},{tomato_z[2]:+.4f})"
            )
        return direction_sign

    def _tomato_z_to_y_reference_axis(
        self,
        tomato_rotation: np.ndarray,
        candidate: dict[str, object],
    ) -> np.ndarray:
        approach_sign = float(candidate.get("approach_sign", self._tomato_y_back_approach_sign()))
        gripper_x_sign = float(candidate.get("gripper_x_sign", 1.0))
        return approach_sign * gripper_x_sign * tomato_rotation[:, 0]

    def _calyx_stem_twist_direction_sign(self) -> float:
        record = self.last_grasp_record
        if not record:
            self.get_logger().warn("No last grasp record is available; using positive post-grasp twist direction.")
            return 1.0

        entry = record.get("entry", {})
        candidate = record.get("candidate", {})
        if not isinstance(entry, dict) or not isinstance(candidate, dict):
            self.get_logger().warn("Last grasp record has no entry/candidate data; using positive twist direction.")
            return 1.0

        object_position = np.array(candidate.get("object_position", entry.get("position")), dtype=float)
        tomato_rotation = np.array(candidate.get("tomato_rotation"), dtype=float)
        if tomato_rotation.shape != (3, 3):
            tomato_rotation = self._base_to_tomato_transform(entry)[:3, :3]
        branch_start = np.array(entry.get("branch_start", self.branch_start), dtype=float)
        approach_sign = float(candidate.get("approach_sign", self._tomato_y_back_approach_sign()))

        tomato_z = tomato_rotation[:, 2]
        calyx_position = object_position + tomato_z * self._tomato_contact_half_span()
        stem_from_calyx_local = tomato_rotation.T @ (branch_start - calyx_position)
        stem_x = float(stem_from_calyx_local[0])
        if abs(stem_x) < 1e-6:
            self.get_logger().warn(
                "Calyx-stem relation is nearly centered on tomato X; using positive post-grasp twist direction. "
                f"stem_from_calyx_local=({stem_from_calyx_local[0]:.4f}, "
                f"{stem_from_calyx_local[1]:.4f}, {stem_from_calyx_local[2]:.4f})"
            )
            return 1.0

        direction_sign = -math.copysign(1.0, stem_x) * math.copysign(1.0, approach_sign)
        self.get_logger().info(
            "Calyx-stem post-grasp twist direction "
            f"stem_from_calyx_local=({stem_from_calyx_local[0]:.4f}, "
            f"{stem_from_calyx_local[1]:.4f}, {stem_from_calyx_local[2]:.4f}) "
            f"approach_sign={approach_sign:+.0f} direction={direction_sign:+.0f}"
        )
        return direction_sign

    def _animate_gripper_spin_to(self, target_deg: float, duration_sec: float, steps: int) -> None:
        start_deg = float(getattr(self, "current_gripper_spin_deg", target_deg))
        target_deg = self._clamp_gripper_spin_deg(target_deg)
        if abs(target_deg - start_deg) < 1e-6:
            self.get_logger().info(
                f"Post-grasp gripper twist segment skipped because it is already at {target_deg:.1f}deg."
            )
            return
        step_duration_sec = duration_sec / max(1, steps)
        self.get_logger().info(
            f"Post-grasp gripper twist segment {start_deg:.1f}deg -> {target_deg:.1f}deg "
            f"over {duration_sec:.2f}s"
        )
        for step_index in range(1, steps + 1):
            alpha = float(step_index) / float(steps)
            angle_deg = start_deg + (target_deg - start_deg) * alpha
            self._set_runtime_gripper_spin_deg(angle_deg, log=False)
            if step_duration_sec <= 0.0:
                continue
            deadline = time.monotonic() + step_duration_sec
            while rclpy.ok() and time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                rclpy.spin_once(self, timeout_sec=min(0.02, max(0.0, remaining)))
        self.get_logger().info(f"Post-grasp gripper twist segment reached {self.current_gripper_spin_deg:.1f}deg.")

    def _right_rule_rejection_reasons(self, candidate: dict[str, object]) -> list[str]:
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
                f"({candidate['opposite_finger']}={candidate['opposite_contact_error']:.4f} > "
                f"{target_tolerance:.4f})"
            )
        return reasons

    def _approach_to_grasp(self) -> dict[str, object] | None:
        self.get_logger().info("Waiting for MoveIt move_group action server...")
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("MoveIt action server was not found. Is teach.launch.py running?")
            return None

        approach_signs = [1.0, -1.0] if bool(self.get_parameter("try_both_approach_sides").value) else [1.0]
        for approach_sign in approach_signs:
            candidate = self._target_pose_candidate(approach_sign)
            target_pose = candidate["pose"]
            self.get_logger().info(
                "Harvest approach candidate "
                f"approach_sign={approach_sign:+.0f} "
                f"xyz=({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f}), "
                f"tool_tip_z_above_center={candidate['tool_tip_z_above_center']:.4f}, "
                f"grasp_tf_z_above_center={candidate['grasp_tf_z_above_center']:.4f}, "
                f"x_axis_offset={candidate['tool_tip_x_axis_offset']:.4f}, "
                f"blue_contact_err={candidate['blue_contact_error']:.4f}, "
                f"orange_contact_err={candidate['orange_contact_error']:.4f}, "
                f"blue_align_deg={candidate['blue_alignment_angle_deg']:.2f}"
            )
            rejection_reasons = self._candidate_rejection_reasons(candidate)
            if rejection_reasons:
                self.get_logger().warn("Skipping candidate because " + "; ".join(rejection_reasons))
                continue
            if self._plan_to_pose(target_pose):
                self.get_logger().info("Approach execution succeeded.")
                return candidate

        self.get_logger().error("No MoveIt grasp candidate satisfied the PyBullet-style grasp conditions.")
        return None

    def _relative_grasp_transform(self) -> np.ndarray:
        relative_translation = [float(value) for value in self.get_parameter("relative_translation").value]
        relative_rotation = [float(value) for value in self.get_parameter("relative_rotation_xyzw").value]
        self.get_logger().info(
            "Using taught tomato_tf -> grasp_tf relative pose: "
            f"xyz=({relative_translation[0]:.4f}, {relative_translation[1]:.4f}, {relative_translation[2]:.4f}), "
            f"xyzw=({relative_rotation[0]:.4f}, {relative_rotation[1]:.4f}, "
            f"{relative_rotation[2]:.4f}, {relative_rotation[3]:.4f})"
        )
        return transform_matrix(relative_translation, relative_rotation)

    def _verify_reached_pose(self, target_pose: Pose) -> bool:
        timeout = float(self.get_parameter("post_grasp_tf_timeout").value)
        target_position = np.array(
            [target_pose.position.x, target_pose.position.y, target_pose.position.z],
            dtype=float,
        )
        target_quaternion = [
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ]
        target_rotation = matrix_from_quaternion(target_quaternion)
        position_tolerance = float(self.get_parameter("post_grasp_position_tolerance").value)
        orientation_tolerance = float(self.get_parameter("post_grasp_orientation_tolerance_deg").value)

        deadline = time.monotonic() + max(timeout, 0.0)
        best_error: tuple[float, float] | None = None
        last_exception: TransformException | None = None
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining < 0.0 and best_error is not None:
                break
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.target_link,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=min(0.10, max(0.0, remaining))),
                )
            except TransformException as exc:
                last_exception = exc
                if remaining <= 0.0:
                    break
                rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, remaining)))
                continue

            actual_position = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ],
                dtype=float,
            )
            actual_quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
            actual_rotation = matrix_from_quaternion(actual_quaternion)
            rotation_delta = target_rotation.T @ actual_rotation
            trace_value = float(np.clip((np.trace(rotation_delta) - 1.0) * 0.5, -1.0, 1.0))
            position_error = float(np.linalg.norm(actual_position - target_position))
            orientation_error = float(math.degrees(math.acos(trace_value)))
            if best_error is None or (position_error / position_tolerance, orientation_error / orientation_tolerance) < (
                best_error[0] / position_tolerance,
                best_error[1] / orientation_tolerance,
            ):
                best_error = (position_error, orientation_error)

            if position_error <= position_tolerance and orientation_error <= orientation_tolerance:
                self.get_logger().info(
                    "Reached grasp pose check: "
                    f"position_error={position_error:.4f}m/{position_tolerance:.4f}m, "
                    f"orientation_error={orientation_error:.2f}deg/{orientation_tolerance:.2f}deg"
                )
                return True
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, remaining)))

        if best_error is None:
            self.get_logger().error(f"Could not verify reached grasp pose from TF: {last_exception}")
            return False

        position_error, orientation_error = best_error
        self.get_logger().info(
            "Reached grasp pose check best observed: "
            f"position_error={position_error:.4f}m/{position_tolerance:.4f}m, "
            f"orientation_error={orientation_error:.2f}deg/{orientation_tolerance:.2f}deg"
        )
        if position_error > position_tolerance:
            self.get_logger().error("Reached grasp pose position error is too large; tomato will not be removed.")
            return False
        if orientation_error > orientation_tolerance:
            self.get_logger().error("Reached grasp pose orientation error is too large; tomato will not be removed.")
            return False
        return True

    def _prepare_tomato_for_grasp(self, object_id: str) -> None:
        suppressed = self._base_suppressed_collision_ids() | set(self.harvested_tomato_ids)
        suppressed.add(object_id)
        self._set_scene_string_array("suppressed_collision_ids", suppressed)
        self._publish_collision_removals(suppressed)
        if bool(self.get_parameter("clear_target_tomato_collision").value):
            self._remove_tomato_collision(object_id)
        if bool(self.get_parameter("allow_target_tomato_touch").value):
            self._allow_tomato_touch(object_id)

    def _attach_tomato_visual_to_gripper(self, object_id: str) -> None:
        record = self.last_grasp_record
        if not record:
            self.get_logger().warn(f"Cannot attach {object_id} visually: no last grasp record is available.")
            return

        target_pose = record.get("target_pose")
        candidate = record.get("candidate", {})
        if not isinstance(target_pose, Pose) or not isinstance(candidate, dict):
            self.get_logger().warn(f"Cannot attach {object_id} visually: selected grasp record is incomplete.")
            return

        object_position = np.array(candidate.get("object_position", []), dtype=float)
        if object_position.size != 3:
            entry = record.get("entry", {})
            if isinstance(entry, dict):
                object_position = np.array(entry.get("position", []), dtype=float)
        if object_position.size != 3:
            self.get_logger().warn(f"Cannot attach {object_id} visually: tomato position is unavailable.")
            return

        target_position = np.array(
            [target_pose.position.x, target_pose.position.y, target_pose.position.z],
            dtype=float,
        )
        target_rotation = matrix_from_quaternion(
            [
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            ]
        )
        attached_offset = target_rotation.T @ (object_position - target_position)
        parameters = [
            self._scene_string_parameter("attached_tomato_id", object_id),
            self._scene_string_parameter("attached_tomato_frame", self.target_link),
            self._scene_double_array_parameter("attached_tomato_offset_xyz", attached_offset),
        ]
        if not self._set_scene_parameters(parameters, timeout_sec=2.0):
            self.get_logger().warn(f"Could not attach {object_id} marker to {self.target_link}.")
            return
        self.get_logger().info(
            f"Attached {object_id} visual marker to {self.target_link} "
            f"offset=({attached_offset[0]:+.4f}, {attached_offset[1]:+.4f}, {attached_offset[2]:+.4f})"
        )

    def _clear_attached_tomato_visual(self) -> None:
        if not self._set_scene_parameters(
            [
                self._scene_string_parameter("attached_tomato_id", ""),
                self._scene_double_array_parameter("attached_tomato_offset_xyz", [0.0, 0.0, 0.0]),
            ],
            timeout_sec=2.0,
        ):
            self.get_logger().warn("Could not clear attached tomato marker.")

    def _sync_scene_tomato_lists(self, harvested: set[str]) -> None:
        self._set_scene_string_array("harvested_tomato_ids", harvested)
        suppressed = self._base_suppressed_collision_ids() | set(harvested)
        self._set_scene_string_array("suppressed_collision_ids", suppressed)
        self._publish_collision_removals(suppressed)

    def _base_suppressed_collision_ids(self) -> set[str]:
        if not bool(self.get_parameter("disable_tomato_collision_for_rule_test").value):
            return set()
        if not (
            bool(self.get_parameter("use_side_spin_rule_grasp_pose").value)
            or bool(self.get_parameter("use_right_rule_grasp_pose").value)
        ):
            return set()
        if str(getattr(self, "side", "both")) not in {"left", "right", "both"}:
            return set()
        return self._all_tomato_collision_ids()

    def _all_tomato_collision_ids(self) -> set[str]:
        ids = {"target_tomato"}
        for entry in self._tomato_layout_entries():
            if bool(entry["target"]):
                continue
            ids.add(f"other_tomato_{int(entry['index'])}")
        return ids

    def _set_scene_string_array(self, name: str, values: set[str]) -> None:
        parameter = self._scene_string_array_parameter(name, values)
        if not self._set_scene_parameters([parameter], timeout_sec=5.0):
            self.get_logger().warn(f"tomato_scene_node parameter service was not found; {name} was not updated.")

    def _load_scene_harvested_tomato_ids(self) -> set[str] | None:
        if not self.scene_get_param_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "tomato_scene_node get_parameters service was not found; refusing to reset preserved harvested state."
            )
            return None
        request = GetParameters.Request()
        request.names = ["harvested_tomato_ids"]
        future = self.scene_get_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.values:
            self.get_logger().error("Could not read harvested_tomato_ids from tomato_scene_node.")
            return None
        values = {
            str(value)
            for value in response.values[0].string_array_value
            if str(value)
        }
        if values:
            self.get_logger().info(f"Loaded preserved harvested tomatoes from scene: {sorted(values)}")
        return values

    def _set_scene_parameters(self, parameters: list[Parameter], timeout_sec: float = 2.0) -> bool:
        if not self.scene_param_client.wait_for_service(timeout_sec=timeout_sec):
            return False

        request = SetParameters.Request()
        request.parameters = parameters
        future = self.scene_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None:
            names = ", ".join(parameter.name for parameter in parameters)
            self.get_logger().warn(f"Could not update tomato_scene_node parameters: {names}")
            return False

        success = True
        for parameter, result in zip(parameters, response.results):
            if not result.successful:
                self.get_logger().warn(
                    f"Could not update tomato_scene_node {parameter.name}: {result.reason}"
                )
                success = False
        return success

    def _set_runtime_gripper_spin_deg(self, gripper_spin_deg: float, log: bool = True) -> None:
        gripper_spin_deg = self._clamp_gripper_spin_deg(gripper_spin_deg)
        parameter = self._scene_double_parameter("gripper_z_rot_deg", gripper_spin_deg)
        if not self.joint_state_param_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                "home_joint_state_publisher parameter service was not found; "
                f"gripper_z_rot_deg={gripper_spin_deg:.1f} was not applied."
            )
            return

        request = SetParameters.Request()
        request.parameters = [parameter]
        future = self.joint_state_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.results or not response.results[0].successful:
            reason = ""
            if response is not None and response.results:
                reason = response.results[0].reason
            self.get_logger().warn(
                f"Could not update gripper_z_rot_deg={gripper_spin_deg:.1f}. {reason}".strip()
            )
            return
        self.current_gripper_spin_deg = gripper_spin_deg
        if log:
            self.get_logger().info(f"Applied runtime gripper spin outside MoveIt plan: {gripper_spin_deg:.1f}deg")

    def _publish_collision_removals(self, object_ids: set[str]) -> None:
        if not object_ids:
            return
        scene = PlanningScene()
        scene.is_diff = True
        for object_id in sorted(object_ids):
            collision_object = CollisionObject()
            collision_object.header.frame_id = self.base_frame
            collision_object.id = object_id
            collision_object.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(collision_object)
        self.planning_scene_pub.publish(scene)

    def _remove_tomato_collision(self, object_id: str) -> None:
        if not self.apply_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("ApplyPlanningScene service was not found; target collision was not removed.")
            return

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_object)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().warn(f"Could not remove target collision object: {collision_object.id}")

    def _allow_tomato_touch(self, object_id: str) -> None:
        if not self.apply_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("ApplyPlanningScene service was not found; target touch was not allowed.")
            return
        if not self.get_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("GetPlanningScene service was not found; target touch was not allowed.")
            return

        acm = self._current_allowed_collision_matrix()
        if acm is None:
            self.get_logger().warn(f"Could not read allowed collision matrix for: {object_id}")
            return

        names = list(acm.entry_names)
        rows = [list(row.enabled) for row in acm.entry_values]

        def ensure_name(name: str) -> int:
            if name in names:
                index = names.index(name)
            else:
                index = len(names)
                names.append(name)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
            for row in rows:
                while len(row) < len(names):
                    row.append(False)
            while len(rows) < len(names):
                rows.append([False] * len(names))
            return index

        object_index = ensure_name(object_id)
        for link_name in [str(link) for link in self.get_parameter("target_touch_links").value]:
            link_index = ensure_name(link_name)
            rows[object_index][link_index] = True
            rows[link_index][object_index] = True

        merged = AllowedCollisionMatrix()
        merged.entry_names = names
        merged.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]
        merged.default_entry_names = list(acm.default_entry_names)
        merged.default_entry_values = list(acm.default_entry_values)

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = merged

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().warn(f"Could not allow target tomato touch links for: {object_id}")

    def _vine_row_offsets(self) -> list[float]:
        start = float(getattr(self, "vine_row_start_y", 0.0))
        end = float(getattr(self, "vine_row_end_y", 0.0))
        spacing = abs(float(getattr(self, "vine_row_spacing_y", 0.5)))
        if spacing < 1e-6:
            return [start]
        step = spacing if end >= start else -spacing
        offsets = []
        value = start
        if step > 0.0:
            while value <= end + 1e-9:
                offsets.append(value)
                value += step
        else:
            while value >= end - 1e-9:
                offsets.append(value)
                value += step
        return offsets

    def _nearest_vine_row_offset(self) -> float:
        if not bool(getattr(self, "use_nearest_vine_row_for_harvest", True)):
            return 0.0
        offsets = self._vine_row_offsets()
        if not offsets:
            return 0.0
        robot_y = float(getattr(self, "robot_y_offset", 0.0))
        return float(min(offsets, key=lambda offset: abs(float(offset) - robot_y)))

    def _vine_row_index_for_offset(self, y_offset: float) -> int:
        for row_index, row_y in enumerate(self._vine_row_offsets()):
            if abs(float(row_y) - float(y_offset)) < 1e-6:
                return row_index
        return 0

    def _entry_collision_id(self, entry: dict[str, object]) -> str:
        row_index = int(entry.get("vine_row_index", 0))
        row_y_offset = float(entry.get("vine_row_y_offset", 0.0))
        if row_index != 0 or abs(row_y_offset) > 1e-9:
            return f"vine_row_tomato_{row_index}_{int(entry['index'])}"
        if bool(entry["target"]):
            return str(self.get_parameter("target_collision_id").value)
        return f"other_tomato_{int(entry['index'])}"

    def _selected_tomato_entries(self) -> list[dict[str, object]]:
        entries = self._tomato_layout_entries()
        if self.harvested_tomato_ids:
            before_count = len(entries)
            entries = [
                entry
                for entry in entries
                if self._entry_collision_id(entry) not in self.harvested_tomato_ids
            ]
            removed_count = before_count - len(entries)
            if removed_count > 0:
                self.get_logger().info(
                    f"Skipping {removed_count} already harvested tomato(s): "
                    f"{sorted(self.harvested_tomato_ids)}"
                )
        target_tomato_index = int(self.get_parameter("target_tomato_index").value)
        if target_tomato_index >= 0:
            selected = [entry for entry in entries if int(entry["index"]) == target_tomato_index]
            return sorted(selected, key=lambda entry: float(entry["position"][2]), reverse=True)
        if not self.include_target_tomato:
            entries = [entry for entry in entries if not bool(entry["target"])]
        if self.side == "both":
            selected = entries
        else:
            selected = [entry for entry in entries if str(entry["side"]) == self.side]
        return sorted(selected, key=lambda entry: float(entry["position"][2]), reverse=True)

    def _lookup_camera_from_base_matrix(self) -> np.ndarray | None:
        camera_frame = str(self.get_parameter("harvest_camera_frame").value)
        timeout_sec = max(0.0, float(self.get_parameter("harvest_camera_visibility_timeout_sec").value))
        deadline = time.monotonic() + timeout_sec
        last_error: Exception | None = None
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    camera_frame,
                    self.base_frame,
                    Time(seconds=0, nanoseconds=0),
                    timeout=Duration(seconds=0.05),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                return transform_matrix(
                    [translation.x, translation.y, translation.z],
                    [rotation.x, rotation.y, rotation.z, rotation.w],
                )
            except TransformException as exc:
                last_error = exc
                if time.monotonic() >= deadline:
                    break
                rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().error(
            "Camera-visible harvest filtering is enabled, but camera TF was unavailable: "
            f"{self.base_frame} -> {camera_frame}: {last_error}"
        )
        return None

    def _camera_visibility_for_entry(
        self,
        entry: dict[str, object],
        camera_from_base: np.ndarray,
    ) -> tuple[bool, np.ndarray, float, float]:
        position_base = np.array(entry["position"], dtype=float)
        position_camera = (camera_from_base @ np.array([*position_base, 1.0], dtype=float))[:3]
        z = float(position_camera[2])
        near_m = max(0.0, float(self.get_parameter("harvest_camera_near_m").value))
        far_m = max(near_m, float(self.get_parameter("harvest_camera_far_m").value))
        if z < near_m or z > far_m:
            return False, position_camera, math.inf, math.inf

        horizontal_angle = math.atan2(float(position_camera[0]), z)
        vertical_angle = math.atan2(float(position_camera[1]), z)
        half_horizontal = math.radians(float(self.get_parameter("harvest_camera_horizontal_fov_deg").value) * 0.5)
        half_vertical = math.radians(float(self.get_parameter("harvest_camera_vertical_fov_deg").value) * 0.5)
        radius_angle = math.atan2(float(getattr(self, "tomato_radius", self.object_radius)), max(z, 1e-6))
        visible = (
            abs(horizontal_angle) <= half_horizontal + radius_angle
            and abs(vertical_angle) <= half_vertical + radius_angle
        )
        return visible, position_camera, math.degrees(horizontal_angle), math.degrees(vertical_angle)

    def _filter_camera_visible_tomato_entries(self, entries: list[dict[str, object]]) -> list[dict[str, object]]:
        if not entries:
            return entries
        camera_from_base = self._lookup_camera_from_base_matrix()
        if camera_from_base is None:
            return []

        visible_entries = []
        hidden_summaries = []
        for entry in entries:
            visible, camera_xyz, horizontal_deg, vertical_deg = self._camera_visibility_for_entry(entry, camera_from_base)
            object_id = self._entry_collision_id(entry)
            if visible:
                visible_entry = dict(entry)
                visible_entry["camera_xyz"] = [
                    float(camera_xyz[0]),
                    float(camera_xyz[1]),
                    float(camera_xyz[2]),
                ]
                visible_entry["camera_horizontal_deg"] = float(horizontal_deg)
                visible_entry["camera_vertical_deg"] = float(vertical_deg)
                visible_entries.append(visible_entry)
                self.get_logger().info(
                    "Camera-visible harvest target accepted: "
                    f"{object_id} idx={entry['index']} side={entry['side']} "
                    f"camera_xyz=({camera_xyz[0]:.3f}, {camera_xyz[1]:.3f}, {camera_xyz[2]:.3f}) "
                    f"angles=({horizontal_deg:.1f}deg, {vertical_deg:.1f}deg)"
                )
            else:
                hidden_summaries.append(
                    f"{object_id}:idx{entry['index']} "
                    f"cam=({camera_xyz[0]:.2f},{camera_xyz[1]:.2f},{camera_xyz[2]:.2f})"
                )
        if hidden_summaries:
            self.get_logger().info(
                "Camera-visible harvest target filter removed "
                f"{len(hidden_summaries)}/{len(entries)} tomato(s): " + ", ".join(hidden_summaries)
            )
        self.get_logger().info(
            f"Camera-visible harvest target filter kept {len(visible_entries)}/{len(entries)} tomato(s)."
        )
        return visible_entries

    def _tomato_layout_entries(self) -> list[dict[str, object]]:
        row_y_offset = self._nearest_vine_row_offset()
        row_index = self._vine_row_index_for_offset(row_y_offset)
        row_shift = np.array([0.0, row_y_offset, 0.0], dtype=float)
        main_start, main_end = self._vine_points()
        map_main_start, map_main_end = self._vine_points(apply_scene_offset=False)
        scene_translation = self._scene_translation()
        tomato_specs = [
            {"offset": [0.20, -0.35, 5.35], "stem_segment_len": 1.20, "target": False},
            {"offset": [0.45, 2.85, 4.25], "stem_segment_len": 1.30, "target": False},
            {"offset": [-0.15, -0.60, 2.95], "stem_segment_len": 1.20, "target": False},
            {"offset": [0.25, 2.55, 1.60], "stem_segment_len": 1.35, "target": False},
            {"offset": [0.0, 0.0, 0.0], "stem_segment_len": 1.35, "target": True},
            {"offset": [0.55, 2.80, -1.35], "stem_segment_len": 1.30, "target": False},
            {"offset": [-0.30, -0.45, -2.80], "stem_segment_len": 1.15, "target": False},
            {"offset": [0.25, 1.75, -4.70], "stem_segment_len": 1.10, "target": False},
        ]
        entries = []
        for index, spec in enumerate(tomato_specs):
            base_tomato_pos = self.base_object_position + np.array(spec["offset"], dtype=float) * self.object_radius
            if bool(spec["target"]):
                base_tomato_pos = self.base_object_position.copy()
            scene_tomato_pos = base_tomato_pos + scene_translation
            main_vine_spin_deg = self._individual_main_vine_spin_deg(index, row_index)
            tomato_pos = self._spin_point_around_main_vine(scene_tomato_pos, main_vine_spin_deg)
            base_branch_start = self._branch_start_for_down_angle(
                map_main_start,
                map_main_end,
                base_tomato_pos,
                45.0,
            )
            branch_start = self._spin_point_around_main_vine(
                base_branch_start + scene_translation,
                main_vine_spin_deg,
            )
            base_stem_axis = target_stem_axis(base_tomato_pos)
            stem_axis = self._spin_vector_around_main_vine(base_stem_axis, main_vine_spin_deg)
            is_left = self._is_left_of_vine_from_robot(branch_start, tomato_pos)
            z_spin_deg = self._local_tomato_z_spin_deg(base_branch_start, base_tomato_pos)
            entries.append(
                {
                    "index": index,
                    "target": bool(spec["target"]),
                    "position": tomato_pos + row_shift,
                    "branch_start": branch_start + row_shift,
                    "base_stem_axis": base_stem_axis,
                    "stem_axis": stem_axis,
                    "z_spin_deg": z_spin_deg,
                    "main_vine_spin_deg": main_vine_spin_deg,
                    "side": "left" if is_left else "right",
                    "vine_row_index": row_index,
                    "vine_row_y_offset": row_y_offset,
                }
            )
        if abs(row_y_offset) > 1e-9:
            for entry in entries:
                entry["side"] = (
                    "left"
                    if self._is_left_of_vine_from_robot(entry["branch_start"], entry["position"])
                    else "right"
                )
        return entries

    def _base_to_tomato_transform(self, entry: dict[str, object]) -> np.ndarray:
        tomato_x, tomato_y, tomato_z = self._spun_tomato_frame_axes_for(
            entry.get("base_stem_axis", entry["stem_axis"]),
            float(entry["z_spin_deg"]),
            float(entry.get("main_vine_spin_deg", self.tomato_z_spin_deg)),
        )
        transform = np.eye(4)
        transform[:3, :3] = np.column_stack((tomato_x, tomato_y, tomato_z))
        transform[:3, 3] = np.array(entry["position"], dtype=float)
        return transform

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

    def _tomato_frame_axes_for(self, stem_axis, z_spin_deg):
        tomato_z_axis = np.array(stem_axis, dtype=float)
        tomato_z_axis /= np.linalg.norm(tomato_z_axis)
        x_hint = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(tomato_z_axis, x_hint))) > 0.95:
            x_hint = np.array([0.0, 1.0, 0.0])
        tomato_y_axis = np.cross(tomato_z_axis, x_hint)
        tomato_y_axis /= np.linalg.norm(tomato_y_axis)
        tomato_x_axis = np.cross(tomato_y_axis, tomato_z_axis)
        tomato_x_axis /= np.linalg.norm(tomato_x_axis)
        spin_rad = math.radians(float(z_spin_deg))
        c = math.cos(spin_rad)
        s = math.sin(spin_rad)
        spun_x_axis = c * tomato_x_axis + s * tomato_y_axis
        spun_y_axis = -s * tomato_x_axis + c * tomato_y_axis
        spun_x_axis /= np.linalg.norm(spun_x_axis)
        spun_y_axis /= np.linalg.norm(spun_y_axis)
        return spun_x_axis, spun_y_axis, tomato_z_axis

    def _current_allowed_collision_matrix(self) -> AllowedCollisionMatrix | None:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None:
            return None
        return response.scene.allowed_collision_matrix

    def _remove_harvested_target_marker(self) -> bool:
        if not self.remove_target_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("remove_target_tomato service was not found; RViz tomato marker may remain visible.")
            return True
        future = self.remove_target_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("remove_target_tomato returned no response.")
            return False
        if not response.success:
            self.get_logger().error(f"remove_target_tomato failed: {response.message}")
            return False
        self.get_logger().info(response.message)
        return True


def main() -> None:
    rclpy.init()
    node = MoveItHarvestSequence()
    try:
        success = node.run_sequence()
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
