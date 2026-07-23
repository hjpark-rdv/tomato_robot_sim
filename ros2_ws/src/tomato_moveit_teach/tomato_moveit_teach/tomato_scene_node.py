import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from tomato_moveit_teach.geometry import orientation_from_z_axis, quaternion_from_matrix, target_stem_axis


class TomatoSceneNode(Node):
    def __init__(self) -> None:
        super().__init__("tomato_scene_node")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("floor_frame", "floor_tf")
        self.declare_parameter("tomato_frame", "tomato_tf")
        self.declare_parameter("main_vine_frame", "main_vine_tf")
        self.declare_parameter("object_position", [0.455, -0.175, 0.34])
        numeric_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("scene_y_offset", 0.0, numeric_descriptor)
        self.declare_parameter("robot_y_offset", 0.0, numeric_descriptor)
        self.declare_parameter("object_radius", 0.036, numeric_descriptor)
        self.declare_parameter("tomato_radius_scale", 0.5, numeric_descriptor)
        self.declare_parameter("main_vine_visual_length_scale", 5.0, numeric_descriptor)
        self.declare_parameter("show_vine_row", True)
        self.declare_parameter("vine_row_start_y", 0.0, numeric_descriptor)
        self.declare_parameter("vine_row_end_y", 5.0, numeric_descriptor)
        self.declare_parameter("vine_row_spacing_y", 0.5, numeric_descriptor)
        self.declare_parameter("tomato_z_spin_deg", 0.0, numeric_descriptor)
        self.declare_parameter("taught_tomato_z_spin_deg", -40.0, numeric_descriptor)
        self.declare_parameter("right_tomato_z_spin_offset_deg", -30.0, numeric_descriptor)
        self.declare_parameter("left_tomato_z_spin_offset_deg", 250.0, numeric_descriptor)
        self.declare_parameter("randomize_individual_stem_spin", False)
        self.declare_parameter("individual_stem_spin_seed", 0, numeric_descriptor)
        self.declare_parameter("individual_stem_spin_min_deg", 0.0, numeric_descriptor)
        self.declare_parameter("individual_stem_spin_max_deg", 360.0, numeric_descriptor)
        self.declare_parameter("show_target_tomato_only", False)
        self.declare_parameter("show_other_tomatoes", True)
        self.declare_parameter("publish_planning_scene", True)
        self.declare_parameter("include_main_vine_collision", True)
        self.declare_parameter("include_other_tomato_collision", True)
        self.declare_parameter("include_target_tomato_collision", False)
        self.declare_parameter("include_target_branch_collision", False)
        self.declare_parameter("include_ground_collision", True)
        self.declare_parameter("include_ceiling_collision", True)
        self.declare_parameter("ceiling_z", 0.80, numeric_descriptor)
        self.declare_parameter("ceiling_thickness", 0.040, numeric_descriptor)
        self.declare_parameter("ceiling_width", 1.60, numeric_descriptor)
        self.declare_parameter("ceiling_depth", 1.60, numeric_descriptor)
        self.declare_parameter("include_rear_wall_collision", True)
        self.declare_parameter("rear_wall_distance", 0.10, numeric_descriptor)
        self.declare_parameter("rear_wall_thickness", 0.035, numeric_descriptor)
        self.declare_parameter("rear_wall_width", 0.75, numeric_descriptor)
        self.declare_parameter("rear_wall_height", 1.10, numeric_descriptor)
        self.declare_parameter("include_robot_rear_wall_collision", True)
        self.declare_parameter("robot_rear_wall_x", -0.50, numeric_descriptor)
        self.declare_parameter("robot_rear_wall_thickness", 0.040, numeric_descriptor)
        self.declare_parameter("robot_rear_wall_width", 1.20, numeric_descriptor)
        self.declare_parameter("robot_rear_wall_height", 1.20, numeric_descriptor)
        self.declare_parameter("include_robot_front_wall_collision", False)
        self.declare_parameter("robot_front_wall_x", 0.62, numeric_descriptor)
        self.declare_parameter("robot_front_wall_thickness", 0.040, numeric_descriptor)
        self.declare_parameter("robot_front_wall_width", 1.20, numeric_descriptor)
        self.declare_parameter("robot_front_wall_height", 1.20, numeric_descriptor)
        self.declare_parameter("include_robot_side_wall_collision", True)
        self.declare_parameter("robot_side_wall_y", 0.50, numeric_descriptor)
        self.declare_parameter("robot_side_wall_thickness", 0.040, numeric_descriptor)
        self.declare_parameter("robot_side_wall_depth", 1.60, numeric_descriptor)
        self.declare_parameter("robot_side_wall_height", 1.20, numeric_descriptor)
        self.declare_parameter("include_robot_pedestal_collision", False)
        self.declare_parameter("robot_pedestal_size_x", 0.40, numeric_descriptor)
        self.declare_parameter("robot_pedestal_size_y", 0.40, numeric_descriptor)
        self.declare_parameter("robot_pedestal_height", 0.18, numeric_descriptor)
        self.declare_parameter("robot_pedestal_top_z", 0.0, numeric_descriptor)
        self.declare_parameter("include_rail_pipe_collision", True)
        self.declare_parameter("rail_pipe_length", 10.0, numeric_descriptor)
        self.declare_parameter("rail_pipe_diameter", 0.050, numeric_descriptor)
        self.declare_parameter("rail_pipe_center_spacing", 0.55, numeric_descriptor)
        self.declare_parameter("rail_pipe_center_z", -0.127, numeric_descriptor)
        self.declare_parameter(
            "suppressed_collision_ids",
            [""],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter(
            "harvested_tomato_ids",
            [""],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter("attached_tomato_id", "")
        self.declare_parameter("attached_tomato_frame", "grasp_tf")
        self.declare_parameter("attached_tomato_offset_xyz", [0.0, 0.0, 0.0], numeric_descriptor)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.floor_frame = str(self.get_parameter("floor_frame").value)
        self.tomato_frame = str(self.get_parameter("tomato_frame").value)
        self.main_vine_frame = str(self.get_parameter("main_vine_frame").value)
        self.base_object_position = np.array(self.get_parameter("object_position").value, dtype=float)
        self.object_position = self.base_object_position.copy()
        self.scene_y_offset = float(self.get_parameter("scene_y_offset").value)
        self.robot_y_offset = float(self.get_parameter("robot_y_offset").value)
        self.object_radius = float(self.get_parameter("object_radius").value)
        self.tomato_radius_scale = float(self.get_parameter("tomato_radius_scale").value)
        self.tomato_radius = self.object_radius * self.tomato_radius_scale
        self.main_vine_visual_length_scale = float(self.get_parameter("main_vine_visual_length_scale").value)
        self.show_vine_row = bool(self.get_parameter("show_vine_row").value)
        self.vine_row_start_y = float(self.get_parameter("vine_row_start_y").value)
        self.vine_row_end_y = float(self.get_parameter("vine_row_end_y").value)
        self.vine_row_spacing_y = float(self.get_parameter("vine_row_spacing_y").value)
        self.tomato_z_spin_deg = float(self.get_parameter("tomato_z_spin_deg").value)
        self.taught_tomato_z_spin_deg = float(self.get_parameter("taught_tomato_z_spin_deg").value)
        self.right_tomato_z_spin_offset_deg = float(self.get_parameter("right_tomato_z_spin_offset_deg").value)
        self.left_tomato_z_spin_offset_deg = float(self.get_parameter("left_tomato_z_spin_offset_deg").value)
        self.randomize_individual_stem_spin = bool(self.get_parameter("randomize_individual_stem_spin").value)
        self.individual_stem_spin_seed = int(self.get_parameter("individual_stem_spin_seed").value)
        self.individual_stem_spin_min_deg = float(self.get_parameter("individual_stem_spin_min_deg").value)
        self.individual_stem_spin_max_deg = float(self.get_parameter("individual_stem_spin_max_deg").value)
        self.show_target_tomato_only = bool(self.get_parameter("show_target_tomato_only").value)
        self.show_other_tomatoes = bool(self.get_parameter("show_other_tomatoes").value)
        self.publish_planning_scene = bool(self.get_parameter("publish_planning_scene").value)
        self.include_main_vine_collision = bool(
            self.get_parameter("include_main_vine_collision").value
        )
        self.include_other_tomato_collision = bool(self.get_parameter("include_other_tomato_collision").value)
        self.include_target_tomato_collision = bool(self.get_parameter("include_target_tomato_collision").value)
        self.include_target_branch_collision = bool(self.get_parameter("include_target_branch_collision").value)
        self.include_ground_collision = bool(self.get_parameter("include_ground_collision").value)
        self.include_ceiling_collision = bool(self.get_parameter("include_ceiling_collision").value)
        self.ceiling_z = float(self.get_parameter("ceiling_z").value)
        self.ceiling_thickness = float(self.get_parameter("ceiling_thickness").value)
        self.ceiling_width = float(self.get_parameter("ceiling_width").value)
        self.ceiling_depth = float(self.get_parameter("ceiling_depth").value)
        self.include_rear_wall_collision = bool(self.get_parameter("include_rear_wall_collision").value)
        self.rear_wall_distance = float(self.get_parameter("rear_wall_distance").value)
        self.rear_wall_thickness = float(self.get_parameter("rear_wall_thickness").value)
        self.rear_wall_width = float(self.get_parameter("rear_wall_width").value)
        self.rear_wall_height = float(self.get_parameter("rear_wall_height").value)
        self.include_robot_rear_wall_collision = bool(self.get_parameter("include_robot_rear_wall_collision").value)
        self.robot_rear_wall_x = float(self.get_parameter("robot_rear_wall_x").value)
        self.robot_rear_wall_thickness = float(self.get_parameter("robot_rear_wall_thickness").value)
        self.robot_rear_wall_width = float(self.get_parameter("robot_rear_wall_width").value)
        self.robot_rear_wall_height = float(self.get_parameter("robot_rear_wall_height").value)
        self.include_robot_front_wall_collision = bool(self.get_parameter("include_robot_front_wall_collision").value)
        self.robot_front_wall_x = float(self.get_parameter("robot_front_wall_x").value)
        self.robot_front_wall_thickness = float(self.get_parameter("robot_front_wall_thickness").value)
        self.robot_front_wall_width = float(self.get_parameter("robot_front_wall_width").value)
        self.robot_front_wall_height = float(self.get_parameter("robot_front_wall_height").value)
        self.include_robot_side_wall_collision = bool(self.get_parameter("include_robot_side_wall_collision").value)
        self.robot_side_wall_y = float(self.get_parameter("robot_side_wall_y").value)
        self.robot_side_wall_thickness = float(self.get_parameter("robot_side_wall_thickness").value)
        self.robot_side_wall_depth = float(self.get_parameter("robot_side_wall_depth").value)
        self.robot_side_wall_height = float(self.get_parameter("robot_side_wall_height").value)
        self.include_robot_pedestal_collision = bool(
            self.get_parameter("include_robot_pedestal_collision").value
        )
        self.robot_pedestal_size_x = float(
            self.get_parameter("robot_pedestal_size_x").value
        )
        self.robot_pedestal_size_y = float(
            self.get_parameter("robot_pedestal_size_y").value
        )
        self.robot_pedestal_height = float(
            self.get_parameter("robot_pedestal_height").value
        )
        self.robot_pedestal_top_z = float(
            self.get_parameter("robot_pedestal_top_z").value
        )
        self.include_rail_pipe_collision = bool(self.get_parameter("include_rail_pipe_collision").value)
        self.rail_pipe_length = float(self.get_parameter("rail_pipe_length").value)
        self.rail_pipe_diameter = float(self.get_parameter("rail_pipe_diameter").value)
        self.rail_pipe_center_spacing = float(self.get_parameter("rail_pipe_center_spacing").value)
        self.rail_pipe_center_z = float(self.get_parameter("rail_pipe_center_z").value)
        self.attached_tomato_id = str(self.get_parameter("attached_tomato_id").value)
        self.attached_tomato_frame = str(self.get_parameter("attached_tomato_frame").value)
        self.attached_tomato_offset_xyz = np.array(
            self.get_parameter("attached_tomato_offset_xyz").value,
            dtype=float,
        )
        self._recompute_layout()
        self._planning_scene_applied = False
        self.target_harvested = False
        self._target_collision_removal_applied = not (
            self.include_target_tomato_collision or self.include_target_branch_collision
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "tomato_markers", 10)
        self.planning_scene_pub = self.create_publisher(PlanningScene, "planning_scene", 10)
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.remove_target_service = self.create_service(Trigger, "remove_target_tomato", self.remove_target_tomato)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self.timer = self.create_timer(0.05, self.publish_scene)
        self.scene_timer = self.create_timer(1.0, self.publish_planning_scene_diff)
        self.get_logger().info(
            f"Publishing {self.tomato_frame} at {self.object_position.tolist()} in {self.base_frame}, "
            f"main_vine_frame={self.main_vine_frame}, tomato_z_spin_deg={self.tomato_z_spin_deg:.1f}, "
            f"taught_tomato_z_spin_deg={self.taught_tomato_z_spin_deg:.1f}, "
            f"individual_stem_spin={'on' if self.randomize_individual_stem_spin else 'off'}, "
            f"individual_stem_spin_seed={self.individual_stem_spin_seed}"
        )

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        layout_changed = False
        harvest_state_changed = False
        next_harvested: set[str] | None = None
        try:
            for parameter in parameters:
                if parameter.name == "object_position":
                    self.base_object_position = np.array(parameter.value, dtype=float)
                    layout_changed = True
                elif parameter.name == "scene_y_offset":
                    self.scene_y_offset = float(parameter.value)
                    self.robot_y_offset = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_y_offset":
                    self.robot_y_offset = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "object_radius":
                    self.object_radius = float(parameter.value)
                    self.tomato_radius = self.object_radius * self.tomato_radius_scale
                    layout_changed = True
                elif parameter.name == "tomato_radius_scale":
                    self.tomato_radius_scale = float(parameter.value)
                    self.tomato_radius = self.object_radius * self.tomato_radius_scale
                    layout_changed = True
                elif parameter.name == "main_vine_visual_length_scale":
                    self.main_vine_visual_length_scale = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "show_vine_row":
                    self.show_vine_row = bool(parameter.value)
                elif parameter.name == "vine_row_start_y":
                    self.vine_row_start_y = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "vine_row_end_y":
                    self.vine_row_end_y = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "vine_row_spacing_y":
                    self.vine_row_spacing_y = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "tomato_z_spin_deg":
                    self.tomato_z_spin_deg = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "taught_tomato_z_spin_deg":
                    self.taught_tomato_z_spin_deg = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "right_tomato_z_spin_offset_deg":
                    self.right_tomato_z_spin_offset_deg = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "left_tomato_z_spin_offset_deg":
                    self.left_tomato_z_spin_offset_deg = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "randomize_individual_stem_spin":
                    self.randomize_individual_stem_spin = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "individual_stem_spin_seed":
                    self.individual_stem_spin_seed = int(parameter.value)
                    layout_changed = True
                elif parameter.name == "individual_stem_spin_min_deg":
                    self.individual_stem_spin_min_deg = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "individual_stem_spin_max_deg":
                    self.individual_stem_spin_max_deg = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "show_target_tomato_only":
                    self.show_target_tomato_only = bool(parameter.value)
                elif parameter.name == "show_other_tomatoes":
                    self.show_other_tomatoes = bool(parameter.value)
                elif parameter.name == "publish_planning_scene":
                    self.publish_planning_scene = bool(parameter.value)
                elif parameter.name == "include_main_vine_collision":
                    self.include_main_vine_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_other_tomato_collision":
                    self.include_other_tomato_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_target_tomato_collision":
                    self.include_target_tomato_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_target_branch_collision":
                    self.include_target_branch_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_ground_collision":
                    self.include_ground_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_ceiling_collision":
                    self.include_ceiling_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "ceiling_z":
                    self.ceiling_z = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "ceiling_thickness":
                    self.ceiling_thickness = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "ceiling_width":
                    self.ceiling_width = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "ceiling_depth":
                    self.ceiling_depth = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_rear_wall_collision":
                    self.include_rear_wall_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "rear_wall_distance":
                    self.rear_wall_distance = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "rear_wall_thickness":
                    self.rear_wall_thickness = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "rear_wall_width":
                    self.rear_wall_width = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "rear_wall_height":
                    self.rear_wall_height = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_robot_rear_wall_collision":
                    self.include_robot_rear_wall_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_rear_wall_x":
                    self.robot_rear_wall_x = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_rear_wall_thickness":
                    self.robot_rear_wall_thickness = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_rear_wall_width":
                    self.robot_rear_wall_width = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_rear_wall_height":
                    self.robot_rear_wall_height = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_robot_front_wall_collision":
                    self.include_robot_front_wall_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_front_wall_x":
                    self.robot_front_wall_x = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_front_wall_thickness":
                    self.robot_front_wall_thickness = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_front_wall_width":
                    self.robot_front_wall_width = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_front_wall_height":
                    self.robot_front_wall_height = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_robot_side_wall_collision":
                    self.include_robot_side_wall_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_side_wall_y":
                    self.robot_side_wall_y = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_side_wall_thickness":
                    self.robot_side_wall_thickness = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_side_wall_depth":
                    self.robot_side_wall_depth = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_side_wall_height":
                    self.robot_side_wall_height = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_robot_pedestal_collision":
                    self.include_robot_pedestal_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_pedestal_size_x":
                    self.robot_pedestal_size_x = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_pedestal_size_y":
                    self.robot_pedestal_size_y = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_pedestal_height":
                    self.robot_pedestal_height = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "robot_pedestal_top_z":
                    self.robot_pedestal_top_z = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "include_rail_pipe_collision":
                    self.include_rail_pipe_collision = bool(parameter.value)
                    layout_changed = True
                elif parameter.name == "rail_pipe_length":
                    self.rail_pipe_length = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "rail_pipe_diameter":
                    self.rail_pipe_diameter = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "rail_pipe_center_spacing":
                    self.rail_pipe_center_spacing = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "rail_pipe_center_z":
                    self.rail_pipe_center_z = float(parameter.value)
                    layout_changed = True
                elif parameter.name == "harvested_tomato_ids":
                    next_harvested = {str(object_id) for object_id in parameter.value if str(object_id)}
                    harvest_state_changed = True
                elif parameter.name == "suppressed_collision_ids":
                    harvest_state_changed = True
                elif parameter.name == "attached_tomato_id":
                    self.attached_tomato_id = str(parameter.value)
                    harvest_state_changed = True
                elif parameter.name == "attached_tomato_frame":
                    self.attached_tomato_frame = str(parameter.value)
                    harvest_state_changed = True
                elif parameter.name == "attached_tomato_offset_xyz":
                    offset = np.array(parameter.value, dtype=float)
                    if offset.size != 3:
                        return SetParametersResult(
                            successful=False,
                            reason="attached_tomato_offset_xyz must contain exactly 3 values.",
                        )
                    self.attached_tomato_offset_xyz = offset
                    harvest_state_changed = True
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if layout_changed:
            self._recompute_layout()
            self._planning_scene_applied = False
            harvest_state_changed = True
            self.get_logger().info(
                "Updated tomato scene layout: "
                f"tomato_z_spin_deg={self.tomato_z_spin_deg:.1f}, "
                f"taught_tomato_z_spin_deg={self.taught_tomato_z_spin_deg:.1f}, "
                f"right_tomato_z_spin_offset_deg={self.right_tomato_z_spin_offset_deg:.1f}, "
                f"left_tomato_z_spin_offset_deg={self.left_tomato_z_spin_offset_deg:.1f}, "
                f"individual_stem_spin={'on' if self.randomize_individual_stem_spin else 'off'}, "
                f"individual_stem_spin_seed={self.individual_stem_spin_seed}, "
                f"robot_y_offset={self.robot_y_offset:.3f}, "
                f"target_xyz=({self.object_position[0]:.4f}, "
                f"{self.object_position[1]:.4f}, {self.object_position[2]:.4f})"
            )

        if harvest_state_changed:
            harvested = self._harvested_tomato_ids() if next_harvested is None else next_harvested
            self.target_harvested = "target_tomato" in harvested
            self._target_collision_removal_applied = not (
                self.include_target_tomato_collision or self.include_target_branch_collision
            )
            self._planning_scene_applied = False

        return SetParametersResult(successful=True)

    def _attached_tomato_id(self) -> str:
        return str(getattr(self, "attached_tomato_id", "")).strip()

    def _scene_translation(self) -> np.ndarray:
        return np.array([0.0, -float(self.robot_y_offset), 0.0], dtype=float)

    def _scene_position(self, position) -> np.ndarray:
        return np.array(position, dtype=float) + self._scene_translation()

    def _recompute_layout(self) -> None:
        scene_object_position = self._scene_position(self.base_object_position)
        self.main_start, self.main_end = self._main_vine_points(scene_object_position)
        self.main_visual_start, self.main_visual_end = self._scaled_main_vine_points(
            self.main_start,
            self.main_end,
        )
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
        self.branch_start, self.calyx_tip = self._target_vine_points(scene_object_position)
        map_main_start, map_main_end = self._main_vine_points(self.base_object_position)
        map_branch_start = self._branch_start_for_down_angle(
            map_main_start,
            map_main_end,
            self.base_object_position,
            45.0,
        )
        self.target_z_spin_deg = self._local_tomato_z_spin_deg(map_branch_start, self.base_object_position)
        self.tomato_entries = self._tomato_layout_entries()

    def publish_scene(self) -> None:
        now = self.get_clock().now().to_msg()
        transforms = [
            self._floor_transform(now),
            self._main_vine_transform(now),
            self._tomato_transform(
                now,
                self.tomato_frame,
                self.object_position,
                self.base_stem_axis,
                self.target_z_spin_deg,
                self.target_main_vine_spin_deg,
            ),
        ]
        transforms.extend(
            self._tomato_transform(
                now,
                f"tomato_{int(entry['index'])}_tf",
                entry["position"],
                entry["base_stem_axis"],
                float(entry["z_spin_deg"]),
                float(entry["main_vine_spin_deg"]),
            )
            for entry in self.tomato_entries
        )
        self.tf_broadcaster.sendTransform(transforms)
        self.marker_pub.publish(self._markers(now))

    def _tomato_transform(
        self,
        stamp,
        child_frame_id: str,
        position,
        base_stem_axis,
        z_spin_deg,
        main_vine_spin_deg=None,
    ) -> TransformStamped:
        position = np.array(position, dtype=float)
        tomato_x_axis, tomato_y_axis, tomato_z_axis = self._spun_tomato_frame_axes_for(
            base_stem_axis,
            z_spin_deg,
            main_vine_spin_deg,
        )
        tomato_base = self._matrix_from_position_axes(position, tomato_x_axis, tomato_y_axis, tomato_z_axis)
        main_base = self._main_vine_matrix_in_base()
        tomato_in_main = np.linalg.inv(main_base) @ tomato_base
        return self._transform_from_matrix(stamp, self.main_vine_frame, child_frame_id, tomato_in_main)

    def _floor_transform(self, stamp) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.floor_frame
        transform.transform.rotation.w = 1.0
        return transform

    def _main_vine_transform(self, stamp) -> TransformStamped:
        main_base = self._main_vine_matrix_in_base()
        main_in_floor = np.eye(4)
        main_in_floor[:3, :3] = main_base[:3, :3]
        main_in_floor[:3, 3] = main_base[:3, 3] - self._scene_translation()
        return self._transform_from_matrix(stamp, self.floor_frame, self.main_vine_frame, main_in_floor)

    def _main_vine_matrix_in_base(self) -> np.ndarray:
        center = (self.main_start + self.main_end) * 0.5
        main_x_axis, main_y_axis, main_z_axis = self._main_vine_frame_axes()
        return self._matrix_from_position_axes(center, main_x_axis, main_y_axis, main_z_axis)

    def _matrix_from_position_axes(self, position, x_axis, y_axis, z_axis) -> np.ndarray:
        matrix = np.eye(4)
        matrix[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
        matrix[:3, 3] = np.array(position, dtype=float)
        return matrix

    def _transform_from_matrix(self, stamp, parent_frame: str, child_frame: str, matrix: np.ndarray) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = str(parent_frame)
        transform.child_frame_id = str(child_frame)
        transform.transform.translation.x = float(matrix[0, 3])
        transform.transform.translation.y = float(matrix[1, 3])
        transform.transform.translation.z = float(matrix[2, 3])
        qx, qy, qz, qw = quaternion_from_matrix(matrix[:3, :3])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    def remove_target_tomato(self, _request, response):
        self.target_harvested = True
        self._planning_scene_applied = False
        self._target_collision_removal_applied = not (
            self.include_target_tomato_collision or self.include_target_branch_collision
        )
        self.get_logger().info("Target tomato harvested: hiding tomato markers and removing target collision objects.")
        response.success = True
        response.message = "Target tomato markers hidden."
        return response

    def publish_planning_scene_diff(self) -> None:
        if not self.publish_planning_scene:
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = self._collision_objects()
        self.planning_scene_pub.publish(scene)

        if self._planning_scene_applied or not self.apply_scene_client.service_is_ready():
            return

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        future.add_done_callback(self._apply_planning_scene_done)

    def _apply_planning_scene_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Failed to apply tomato planning scene: {exc}")
            return
        self._planning_scene_applied = bool(response.success)
        if self._planning_scene_applied:
            names = ", ".join(obj.id for obj in self._collision_objects())
            if self.target_harvested:
                self._target_collision_removal_applied = True
            self.get_logger().info(f"Applied MoveIt collision objects: {names}")

    def _collision_objects(self) -> list[CollisionObject]:
        objects = []
        suppressed = self._suppressed_collision_ids()
        harvested = self._harvested_tomato_ids()
        target_harvested = self.target_harvested or "target_tomato" in harvested
        if self.include_main_vine_collision and not self.show_target_tomato_only:
            if "main_vine" not in suppressed:
                objects.append(
                    self._cylinder_collision_object(
                        "main_vine",
                        self.main_visual_start,
                        self.main_visual_end,
                        self.object_radius * 0.15,
                    )
                )
        if target_harvested:
            if not self._target_collision_removal_applied:
                if self.include_target_tomato_collision:
                    objects.append(self._remove_collision_object("target_tomato"))
                if self.include_target_branch_collision:
                    objects.append(self._remove_collision_object("target_branch"))
        elif self.include_target_tomato_collision and "target_tomato" not in suppressed:
            objects.append(self._sphere_collision_object("target_tomato", self.object_position, self.tomato_radius))
        if (
            self.include_target_branch_collision
            and not target_harvested
            and not self.show_target_tomato_only
            and "target_branch" not in suppressed
        ):
            objects.append(
                self._cylinder_collision_object(
                    "target_branch",
                    self.branch_start,
                    self.calyx_tip,
                    self.object_radius * 0.06,
                )
            )
        if self.include_other_tomato_collision and not self.show_target_tomato_only:
            objects.extend(self._other_tomato_collision_objects())
        if self.include_ground_collision:
            objects.append(self._box_collision_object("ground_guard", [0.0, 0.0, -0.180], [1.6, 1.6, 0.030]))
        if self.include_ceiling_collision:
            ceiling_position, ceiling_size = self._ceiling_pose_and_size()
            objects.append(self._box_collision_object("ceiling_guard", ceiling_position, ceiling_size))
        if self.include_rear_wall_collision:
            wall_position, wall_size, wall_orientation = self._rear_wall_pose_and_size()
            objects.append(
                self._box_collision_object(
                    "tomato_rear_guard_wall",
                    wall_position,
                    wall_size,
                    wall_orientation,
                )
            )
        if self.include_robot_rear_wall_collision:
            wall_position, wall_size = self._robot_rear_wall_pose_and_size()
            objects.append(self._box_collision_object("robot_rear_guard_wall", wall_position, wall_size))
        if self.include_robot_front_wall_collision:
            wall_position, wall_size = self._robot_front_wall_pose_and_size()
            objects.append(self._box_collision_object("robot_front_guard_wall", wall_position, wall_size))
        if self.include_robot_side_wall_collision:
            for wall_id, wall_position, wall_size in self._robot_side_wall_specs():
                objects.append(self._box_collision_object(wall_id, wall_position, wall_size))
        if self.include_robot_pedestal_collision:
            pedestal_position, pedestal_size = self._robot_pedestal_pose_and_size()
            objects.append(
                self._box_collision_object(
                    "robot_base_pedestal",
                    pedestal_position,
                    pedestal_size,
                )
            )
        if self.include_rail_pipe_collision:
            for object_id, start, end, radius in self._rail_pipe_specs():
                objects.append(self._cylinder_collision_object(object_id, start, end, radius))
        for object_id in sorted(harvested):
            objects.append(self._remove_collision_object(object_id))
        for object_id in sorted(suppressed):
            objects.append(self._remove_collision_object(object_id))
        return objects

    def _suppressed_collision_ids(self) -> set[str]:
        try:
            return {
                str(object_id)
                for object_id in self.get_parameter("suppressed_collision_ids").value
                if str(object_id)
            }
        except (TypeError, ValueError):
            return set()

    def _harvested_tomato_ids(self) -> set[str]:
        try:
            return {
                str(object_id)
                for object_id in self.get_parameter("harvested_tomato_ids").value
                if str(object_id)
            }
        except (TypeError, ValueError):
            return set()

    def _other_tomato_collision_objects(self) -> list[CollisionObject]:
        objects = []
        suppressed = self._suppressed_collision_ids()
        harvested = self._harvested_tomato_ids()
        for entry in self.tomato_entries:
            if bool(entry["target"]):
                continue
            object_id = f"other_tomato_{int(entry['index'])}"
            if object_id in suppressed or object_id in harvested:
                continue
            objects.append(
                self._sphere_collision_object(
                    object_id,
                    entry["position"],
                    self.tomato_radius,
                )
            )
        return objects

    def _sphere_collision_object(self, object_id: str, position, radius: float) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self.base_frame
        obj.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(radius)]
        obj.primitives.append(primitive)
        obj.primitive_poses.append(self._pose(position))
        obj.operation = CollisionObject.ADD
        return obj

    def _remove_collision_object(self, object_id: str) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self.base_frame
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        return obj

    def _box_collision_object(self, object_id: str, position, size, orientation=None) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self.base_frame
        obj.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in size]
        obj.primitives.append(primitive)
        obj.primitive_poses.append(self._pose(position, orientation))
        obj.operation = CollisionObject.ADD
        return obj

    def _cylinder_collision_object(self, object_id: str, start, end, radius: float) -> CollisionObject:
        start = np.array(start, dtype=float)
        end = np.array(end, dtype=float)
        axis = end - start
        obj = CollisionObject()
        obj.header.frame_id = self.base_frame
        obj.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(np.linalg.norm(axis)), float(radius)]
        obj.primitives.append(primitive)
        obj.primitive_poses.append(self._pose((start + end) * 0.5, orientation_from_z_axis(axis)))
        obj.operation = CollisionObject.ADD
        return obj

    def _pose(self, position, orientation=None) -> Pose:
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        if orientation is None:
            pose.orientation.w = 1.0
        else:
            pose.orientation.x = float(orientation[0])
            pose.orientation.y = float(orientation[1])
            pose.orientation.z = float(orientation[2])
            pose.orientation.w = float(orientation[3])
        return pose

    def _markers(self, stamp) -> MarkerArray:
        markers = MarkerArray()
        if self.show_target_tomato_only:
            self._append_target_only_markers(markers, stamp)
            self._append_rear_wall_marker(markers, stamp)
            self._append_robot_rear_wall_marker(markers, stamp)
            self._append_robot_front_wall_marker(markers, stamp)
            self._append_ceiling_marker(markers, stamp)
            self._append_robot_side_wall_markers(markers, stamp)
            self._append_robot_pedestal_marker(markers, stamp)
            self._append_rail_pipe_markers(markers, stamp)
            self._append_vine_row_markers(markers, stamp)
            return markers

        markers.markers.append(
            self._cylinder_marker(
                2,
                stamp,
                "main_vine",
                self.main_visual_start,
                self.main_visual_end,
                self.object_radius * 0.15,
            )
        )
        self._append_rear_wall_marker(markers, stamp)
        self._append_robot_rear_wall_marker(markers, stamp)
        self._append_robot_front_wall_marker(markers, stamp)
        self._append_ceiling_marker(markers, stamp)
        self._append_robot_side_wall_markers(markers, stamp)
        self._append_robot_pedestal_marker(markers, stamp)
        self._append_rail_pipe_markers(markers, stamp)
        self._append_vine_row_markers(markers, stamp)

        attached_tomato_id = self._attached_tomato_id()
        target_harvested = (
            self.target_harvested
            or "target_tomato" in self._harvested_tomato_ids()
            or attached_tomato_id == "target_tomato"
        )
        if target_harvested:
            for ns, marker_ids in (
                ("target_tomato", [0]),
                ("tomato_positive_x_surface", [1]),
                ("target_calyx_stem", [4]),
                ("target_calyx_leaf", list(range(5, 11))),
            ):
                for delete_id in marker_ids:
                    markers.markers.append(self._delete_marker(delete_id, stamp, ns))
            markers.markers.append(
                self._cylinder_marker(
                    3,
                    stamp,
                    "target_branch",
                    self.branch_start,
                    self.calyx_tip,
                    self.object_radius * 0.06,
                )
            )
        else:
            marker_id = 0
            markers.markers.append(
                self._sphere_marker(
                    marker_id,
                    stamp,
                    "target_tomato",
                    self.object_position,
                    self.tomato_radius,
                    [0.92, 0.05, 0.035, 1.0],
                )
            )
            marker_id += 1

            markers.markers.append(self._tomato_positive_x_ring_marker(marker_id, stamp))
            marker_id += 1

            marker_id += 1

            markers.markers.append(
                self._cylinder_marker(
                    marker_id,
                    stamp,
                    "target_branch",
                    self.branch_start,
                    self.calyx_tip,
                    self.object_radius * 0.06,
                )
            )
            marker_id += 1

            stem_start = self.object_position + self.stem_axis * self.tomato_radius
            stem_end = self.calyx_tip
            markers.markers.append(
                self._cylinder_marker(
                    marker_id,
                    stamp,
                    "target_calyx_stem",
                    stem_start,
                    stem_end,
                    self.tomato_radius * 0.11,
                    [0.02, 0.66, 0.12, 1.0],
                )
            )
            marker_id += 1

            for leaf_index in range(6):
                markers.markers.append(self._leaf_marker(marker_id, stamp, leaf_index))
                marker_id += 1

        if self.show_other_tomatoes:
            self._append_other_tomato_markers(markers, stamp)
        self._append_attached_tomato_marker(markers, stamp)
        return markers

    def _append_target_only_markers(self, markers: MarkerArray, stamp) -> None:
        attached_tomato_id = self._attached_tomato_id()
        if (
            self.target_harvested
            or "target_tomato" in self._harvested_tomato_ids()
            or attached_tomato_id == "target_tomato"
        ):
            markers.markers.append(self._delete_marker(0, stamp, "target_tomato"))
        else:
            markers.markers.append(
                self._sphere_marker(
                    0,
                    stamp,
                    "target_tomato",
                    self.object_position,
                    self.tomato_radius,
                    [0.92, 0.05, 0.035, 1.0],
                )
            )

        delete_specs = (
            ("main_vine", [2]),
            ("tomato_positive_x_surface", [1]),
            ("target_branch", [3]),
            ("target_calyx_stem", [4]),
            ("target_calyx_leaf", list(range(5, 11))),
            ("other_tomato", [entry["index"] for entry in self.tomato_entries if not bool(entry["target"])]),
            (
                "other_branch",
                [
                    branch_id
                    for entry in self.tomato_entries
                    if not bool(entry["target"])
                    for branch_id in (int(entry["index"]) * 2, int(entry["index"]) * 2 + 1)
                ],
            ),
            ("other_calyx_stem", [entry["index"] for entry in self.tomato_entries if not bool(entry["target"])]),
            (
                "other_calyx_leaf",
                [
                    int(entry["index"]) * 10 + leaf_index
                    for entry in self.tomato_entries
                    if not bool(entry["target"])
                    for leaf_index in range(6)
                ],
            ),
        )
        for ns, marker_ids in delete_specs:
            for marker_id in marker_ids:
                markers.markers.append(self._delete_marker(int(marker_id), stamp, ns))
        self._append_attached_tomato_marker(markers, stamp)

    def _delete_marker(self, marker_id, stamp, ns) -> Marker:
        marker = Marker()
        self._set_visual_marker_header(marker)
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _set_visual_marker_header(self, marker: Marker) -> None:
        marker.header.frame_id = self.world_frame
        # RViz can flicker when base_link markers are stamped slightly ahead of
        # the latest base_link TF. Time(0) tells RViz to use the newest TF.
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0

    def _world_visual_position(self, position) -> np.ndarray:
        return np.array(position, dtype=float) - self._scene_translation()

    def _visual_pose(self, position, orientation=None) -> Pose:
        return self._pose(self._world_visual_position(position), orientation)

    def _rear_wall_pose_and_size(self) -> tuple[np.ndarray, list[float], tuple[float, float, float, float]]:
        points = [np.array(entry["position"], dtype=float) for entry in self.tomato_entries]
        points.extend([np.array(self.main_start, dtype=float), np.array(self.main_end, dtype=float)])
        points_array = np.array(points, dtype=float)
        crop_center_xy = np.mean(points_array[:, :2], axis=0)
        rear_normal_xy = crop_center_xy.copy()
        normal_norm = float(np.linalg.norm(rear_normal_xy))
        if normal_norm < 1e-6:
            rear_normal_xy = np.array([1.0, 0.0], dtype=float)
        else:
            rear_normal_xy /= normal_norm
        tangent_xy = np.array([-rear_normal_xy[1], rear_normal_xy[0]], dtype=float)

        projections = points_array[:, :2] @ rear_normal_xy
        tangent_projections = points_array[:, :2] @ tangent_xy
        wall_projection = float(np.max(projections) + self.rear_wall_distance + self.rear_wall_thickness * 0.5)
        tangent_projection = float(np.mean(tangent_projections))
        wall_xy = rear_normal_xy * wall_projection + tangent_xy * tangent_projection

        wall_position = np.array([wall_xy[0], wall_xy[1], self.rear_wall_height * 0.5 - 0.05], dtype=float)
        wall_size = [
            float(self.rear_wall_thickness),
            float(self.rear_wall_width),
            float(self.rear_wall_height),
        ]
        wall_x = np.array([rear_normal_xy[0], rear_normal_xy[1], 0.0], dtype=float)
        wall_y = np.array([tangent_xy[0], tangent_xy[1], 0.0], dtype=float)
        wall_z = np.array([0.0, 0.0, 1.0], dtype=float)
        wall_orientation = quaternion_from_matrix(np.column_stack((wall_x, wall_y, wall_z)))
        return wall_position, wall_size, wall_orientation

    def _append_rear_wall_marker(self, markers: MarkerArray, stamp) -> None:
        if not self.include_rear_wall_collision:
            markers.markers.append(self._delete_marker(0, stamp, "tomato_rear_guard_wall"))
            return
        wall_position, wall_size, wall_orientation = self._rear_wall_pose_and_size()
        marker = Marker()
        self._set_visual_marker_header(marker)
        marker.ns = "tomato_rear_guard_wall"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self._visual_pose(wall_position, wall_orientation)
        marker.scale.x = wall_size[0]
        marker.scale.y = wall_size[1]
        marker.scale.z = wall_size[2]
        marker.color.r = 0.42
        marker.color.g = 0.02
        marker.color.b = 0.95
        marker.color.a = 0.26
        markers.markers.append(marker)

    def _robot_rear_wall_pose_and_size(self) -> tuple[np.ndarray, list[float]]:
        wall_position = np.array(
            [self.robot_rear_wall_x, 0.0, self.robot_rear_wall_height * 0.5 - 0.05],
            dtype=float,
        )
        wall_size = [
            float(self.robot_rear_wall_thickness),
            float(self.robot_rear_wall_width),
            float(self.robot_rear_wall_height),
        ]
        return wall_position, wall_size

    def _append_robot_rear_wall_marker(self, markers: MarkerArray, stamp) -> None:
        if not self.include_robot_rear_wall_collision:
            markers.markers.append(self._delete_marker(0, stamp, "robot_rear_guard_wall"))
            return
        wall_position, wall_size = self._robot_rear_wall_pose_and_size()
        marker = Marker()
        self._set_visual_marker_header(marker)
        marker.ns = "robot_rear_guard_wall"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self._visual_pose(wall_position)
        marker.scale.x = wall_size[0]
        marker.scale.y = wall_size[1]
        marker.scale.z = wall_size[2]
        marker.color.r = 0.02
        marker.color.g = 0.55
        marker.color.b = 0.95
        marker.color.a = 0.24
        markers.markers.append(marker)

    def _robot_front_wall_pose_and_size(self) -> tuple[np.ndarray, list[float]]:
        wall_position = np.array(
            [self.robot_front_wall_x, 0.0, self.robot_front_wall_height * 0.5 - 0.05],
            dtype=float,
        )
        wall_size = [
            float(self.robot_front_wall_thickness),
            float(self.robot_front_wall_width),
            float(self.robot_front_wall_height),
        ]
        return wall_position, wall_size

    def _append_robot_front_wall_marker(self, markers: MarkerArray, stamp) -> None:
        if not self.include_robot_front_wall_collision:
            markers.markers.append(self._delete_marker(0, stamp, "robot_front_guard_wall"))
            return
        wall_position, wall_size = self._robot_front_wall_pose_and_size()
        marker = Marker()
        self._set_visual_marker_header(marker)
        marker.ns = "robot_front_guard_wall"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self._visual_pose(wall_position)
        marker.scale.x = wall_size[0]
        marker.scale.y = wall_size[1]
        marker.scale.z = wall_size[2]
        marker.color.r = 0.95
        marker.color.g = 0.06
        marker.color.b = 0.03
        marker.color.a = 0.24
        markers.markers.append(marker)

    def _robot_side_wall_specs(self) -> list[tuple[str, np.ndarray, list[float]]]:
        wall_size = [
            float(self.robot_side_wall_depth),
            float(self.robot_side_wall_thickness),
            float(self.robot_side_wall_height),
        ]
        z_center = self.robot_side_wall_height * 0.5 - 0.05
        y_offset = abs(float(self.robot_side_wall_y))
        return [
            ("robot_left_guard_wall", np.array([0.0, y_offset, z_center], dtype=float), wall_size),
            ("robot_right_guard_wall", np.array([0.0, -y_offset, z_center], dtype=float), wall_size),
        ]

    def _append_robot_side_wall_markers(self, markers: MarkerArray, stamp) -> None:
        if not self.include_robot_side_wall_collision:
            markers.markers.append(self._delete_marker(0, stamp, "robot_left_guard_wall"))
            markers.markers.append(self._delete_marker(0, stamp, "robot_right_guard_wall"))
            return
        for wall_id, wall_position, wall_size in self._robot_side_wall_specs():
            marker = Marker()
            self._set_visual_marker_header(marker)
            marker.ns = wall_id
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = self._visual_pose(wall_position)
            marker.scale.x = wall_size[0]
            marker.scale.y = wall_size[1]
            marker.scale.z = wall_size[2]
            marker.color.r = 0.02
            marker.color.g = 0.80
            marker.color.b = 0.55
            marker.color.a = 0.22
            markers.markers.append(marker)

    def _robot_pedestal_pose_and_size(self) -> tuple[np.ndarray, list[float]]:
        height = max(0.001, float(self.robot_pedestal_height))
        position = np.array(
            [0.0, 0.0, float(self.robot_pedestal_top_z) - height * 0.5],
            dtype=float,
        )
        size = [
            max(0.001, float(self.robot_pedestal_size_x)),
            max(0.001, float(self.robot_pedestal_size_y)),
            height,
        ]
        return position, size

    def _append_robot_pedestal_marker(self, markers: MarkerArray, stamp) -> None:
        if not self.include_robot_pedestal_collision:
            markers.markers.append(
                self._delete_marker(0, stamp, "robot_base_pedestal")
            )
            return
        pedestal_position, pedestal_size = self._robot_pedestal_pose_and_size()
        marker = Marker()
        self._set_visual_marker_header(marker)
        marker.ns = "robot_base_pedestal"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self._visual_pose(pedestal_position)
        marker.scale.x = pedestal_size[0]
        marker.scale.y = pedestal_size[1]
        marker.scale.z = pedestal_size[2]
        marker.color.r = 0.18
        marker.color.g = 0.20
        marker.color.b = 0.24
        marker.color.a = 0.85
        markers.markers.append(marker)

    def _rail_pipe_specs(self) -> list[tuple[str, np.ndarray, np.ndarray, float]]:
        half_length = max(0.0, float(self.rail_pipe_length)) * 0.5
        half_spacing = max(0.0, float(self.rail_pipe_center_spacing)) * 0.5
        radius = max(0.0, float(self.rail_pipe_diameter)) * 0.5
        z_center = float(self.rail_pipe_center_z)
        scene_translation = self._scene_translation()
        return [
            (
                "rail_pipe_pos_x",
                np.array([half_spacing, -half_length, z_center], dtype=float) + scene_translation,
                np.array([half_spacing, half_length, z_center], dtype=float) + scene_translation,
                radius,
            ),
            (
                "rail_pipe_neg_x",
                np.array([-half_spacing, -half_length, z_center], dtype=float) + scene_translation,
                np.array([-half_spacing, half_length, z_center], dtype=float) + scene_translation,
                radius,
            ),
        ]

    def _append_rail_pipe_markers(self, markers: MarkerArray, stamp) -> None:
        if not self.include_rail_pipe_collision:
            markers.markers.append(self._delete_marker(0, stamp, "rail_pipe"))
            markers.markers.append(self._delete_marker(1, stamp, "rail_pipe"))
            return
        for marker_id, (_object_id, start, end, radius) in enumerate(self._rail_pipe_specs()):
            markers.markers.append(
                self._cylinder_marker(
                    marker_id,
                    stamp,
                    "rail_pipe",
                    start,
                    end,
                    radius,
                    [0.78, 0.78, 0.74, 1.0],
                )
            )

    def _vine_row_offsets(self) -> list[float]:
        start = float(self.vine_row_start_y)
        end = float(self.vine_row_end_y)
        spacing = abs(float(self.vine_row_spacing_y))
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

    def _vine_row_specs(self) -> list[tuple[int, np.ndarray, np.ndarray, float]]:
        specs = []
        radius = self.object_radius * 0.15
        for marker_id, y_offset in enumerate(self._vine_row_offsets()):
            if abs(y_offset) < 1e-9:
                continue
            shift = np.array([0.0, y_offset, 0.0], dtype=float)
            specs.append((marker_id, self.main_visual_start + shift, self.main_visual_end + shift, radius))
        return specs

    def _append_vine_row_markers(self, markers: MarkerArray, stamp) -> None:
        max_delete_count = min(len(self._vine_row_offsets()) + 2, 512)
        if not self.show_vine_row:
            for marker_id in range(max_delete_count):
                markers.markers.append(self._delete_marker(marker_id, stamp, "vine_row_main_vine"))
            self._append_vine_row_tomato_deletes(markers, stamp, max_delete_count)
            return
        active_ids = set()
        for marker_id, start, end, radius in self._vine_row_specs():
            active_ids.add(marker_id)
            markers.markers.append(
                self._cylinder_marker(
                    marker_id,
                    stamp,
                    "vine_row_main_vine",
                    start,
                    end,
                    radius,
                    [0.24, 0.40, 0.18, 0.82],
                )
            )
        for marker_id in range(max_delete_count):
            if marker_id not in active_ids:
                markers.markers.append(self._delete_marker(marker_id, stamp, "vine_row_main_vine"))
        self._append_vine_row_tomato_markers(markers, stamp)

    def _append_vine_row_tomato_deletes(self, markers: MarkerArray, stamp, row_count: int) -> None:
        for row_index in range(row_count):
            self._append_vine_row_tomato_row_deletes(markers, stamp, row_index)

    def _append_vine_row_tomato_row_deletes(self, markers: MarkerArray, stamp, row_index: int) -> None:
        tomato_count = len(self.tomato_entries)
        for tomato_index in range(tomato_count):
            base_id = row_index * 1000 + tomato_index
            markers.markers.append(self._delete_marker(base_id, stamp, "vine_row_tomato"))
            markers.markers.append(self._delete_marker(base_id * 2, stamp, "vine_row_branch"))
            markers.markers.append(self._delete_marker(base_id * 2 + 1, stamp, "vine_row_branch"))
            markers.markers.append(self._delete_marker(base_id, stamp, "vine_row_calyx_stem"))
            for leaf_index in range(6):
                markers.markers.append(
                    self._delete_marker(base_id * 10 + leaf_index, stamp, "vine_row_calyx_leaf")
                )

    def _append_vine_row_tomato_markers(self, markers: MarkerArray, stamp) -> None:
        active_row_ids = set()
        harvested = self._harvested_tomato_ids()
        attached_tomato_id = self._attached_tomato_id()
        for row_index, y_offset in enumerate(self._vine_row_offsets()):
            if abs(y_offset) < 1e-9:
                continue
            active_row_ids.add(row_index)
            for entry in self._tomato_layout_entries(row_index=row_index, row_y_offset=y_offset):
                tomato_index = int(entry["index"])
                base_id = row_index * 1000 + tomato_index
                object_id = f"vine_row_tomato_{row_index}_{tomato_index}"
                tomato_pos = np.array(entry["position"], dtype=float)
                branch_start = np.array(entry["branch_start"], dtype=float)
                branch_elbow = np.array(entry["branch_elbow"], dtype=float)
                calyx_tip = np.array(entry["calyx_tip"], dtype=float)
                stem_axis = np.array(entry["stem_axis"], dtype=float)
                base_stem_axis = np.array(entry["base_stem_axis"], dtype=float)
                z_spin_deg = float(entry["z_spin_deg"])
                main_vine_spin_deg = float(entry["main_vine_spin_deg"])
                fruit_hidden = object_id in harvested or object_id == attached_tomato_id

                if fruit_hidden:
                    markers.markers.append(self._delete_marker(base_id, stamp, "vine_row_tomato"))
                    markers.markers.append(self._delete_marker(base_id, stamp, "vine_row_calyx_stem"))
                    for leaf_index in range(6):
                        markers.markers.append(
                            self._delete_marker(base_id * 10 + leaf_index, stamp, "vine_row_calyx_leaf")
                        )
                else:
                    markers.markers.append(
                        self._sphere_marker(
                            base_id,
                            stamp,
                            "vine_row_tomato",
                            tomato_pos,
                            self.tomato_radius,
                            [0.88, 0.04, 0.03, 0.95],
                        )
                    )
                    markers.markers.append(
                        self._cylinder_marker(
                            base_id,
                            stamp,
                            "vine_row_calyx_stem",
                            tomato_pos + stem_axis * self.tomato_radius,
                            calyx_tip,
                            self.tomato_radius * 0.11,
                            [0.02, 0.60, 0.11, 0.95],
                        )
                    )
                    for leaf_index in range(6):
                        markers.markers.append(
                            self._leaf_marker_for(
                                base_id * 10 + leaf_index,
                                stamp,
                                "vine_row_calyx_leaf",
                                leaf_index,
                                tomato_pos,
                                base_stem_axis,
                                z_spin_deg,
                                main_vine_spin_deg,
                            )
                        )
                markers.markers.append(
                    self._cylinder_marker(
                        base_id * 2,
                        stamp,
                        "vine_row_branch",
                        branch_start,
                        branch_elbow,
                        self.object_radius * 0.06,
                        [0.23, 0.39, 0.18, 0.90],
                    )
                )
                markers.markers.append(
                    self._cylinder_marker(
                        base_id * 2 + 1,
                        stamp,
                        "vine_row_branch",
                        branch_elbow,
                        calyx_tip,
                        self.object_radius * 0.06,
                        [0.23, 0.39, 0.18, 0.90],
                    )
                )

        max_delete_count = min(len(self._vine_row_offsets()) + 2, 512)
        for row_index in range(max_delete_count):
            if row_index not in active_row_ids:
                self._append_vine_row_tomato_row_deletes(markers, stamp, row_index)

    def _ceiling_pose_and_size(self) -> tuple[np.ndarray, list[float]]:
        ceiling_position = np.array(
            [0.0, 0.0, self.ceiling_z + self.ceiling_thickness * 0.5],
            dtype=float,
        )
        ceiling_size = [
            float(self.ceiling_width),
            float(self.ceiling_depth),
            float(self.ceiling_thickness),
        ]
        return ceiling_position, ceiling_size

    def _append_ceiling_marker(self, markers: MarkerArray, stamp) -> None:
        if not self.include_ceiling_collision:
            markers.markers.append(self._delete_marker(0, stamp, "ceiling_guard"))
            return
        ceiling_position, ceiling_size = self._ceiling_pose_and_size()
        marker = Marker()
        self._set_visual_marker_header(marker)
        marker.ns = "ceiling_guard"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self._visual_pose(ceiling_position)
        marker.scale.x = ceiling_size[0]
        marker.scale.y = ceiling_size[1]
        marker.scale.z = ceiling_size[2]
        marker.color.r = 0.95
        marker.color.g = 0.78
        marker.color.b = 0.12
        marker.color.a = 0.20
        markers.markers.append(marker)

    def _main_vine_points(self, object_position=None):
        if object_position is None:
            object_position = self.base_object_position
        object_position = np.array(object_position, dtype=float)
        main_start = object_position + np.array(
            [-self.object_radius * 0.75, self.object_radius * 1.35, -self.object_radius * 6.00]
        )
        main_end = object_position + np.array(
            [-self.object_radius * 0.35, self.object_radius * 1.15, self.object_radius * 6.80]
        )
        return main_start, main_end

    def _scaled_main_vine_points(self, main_start, main_end):
        start = np.array(main_start, dtype=float)
        end = np.array(main_end, dtype=float)
        center = (start + end) * 0.5
        half_axis = (end - start) * 0.5
        scale = max(float(self.main_vine_visual_length_scale), 1e-6)
        return center - half_axis * scale, center + half_axis * scale

    def _target_vine_points(self, object_position=None):
        if object_position is None:
            object_position = self._scene_position(self.base_object_position)
        object_position = np.array(object_position, dtype=float)
        node_z = float(object_position[2] + self.object_radius * 1.25)
        branch_start = self._point_on_segment_at_z(self.main_start, self.main_end, node_z)
        calyx_tip = self.object_position + self.stem_axis * (self.tomato_radius * 1.26 + 0.01)
        return branch_start, calyx_tip

    def _tomato_layout_entries(self, row_index: int = 0, row_y_offset: float = 0.0):
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
        scene_translation = self._scene_translation()
        map_main_start, map_main_end = self._main_vine_points(self.base_object_position)
        row_shift = np.array([0.0, float(row_y_offset), 0.0], dtype=float)
        entries = []
        for index, spec in enumerate(tomato_specs):
            map_tomato_pos = self.base_object_position + np.array(spec["offset"], dtype=float) * self.object_radius
            if bool(spec["target"]):
                map_tomato_pos = self.base_object_position.copy()
            scene_tomato_pos = map_tomato_pos + scene_translation
            main_vine_spin_deg = self._individual_main_vine_spin_deg(index, row_index)
            tomato_pos = self._spin_point_around_main_vine(scene_tomato_pos, main_vine_spin_deg) + row_shift
            map_branch_start = self._branch_start_for_down_angle(
                map_main_start,
                map_main_end,
                map_tomato_pos,
                45.0,
            )
            branch_start = (
                self._spin_point_around_main_vine(map_branch_start + scene_translation, main_vine_spin_deg)
                + row_shift
            )
            base_stem_axis = target_stem_axis(map_tomato_pos)
            stem_axis = self._spin_vector_around_main_vine(base_stem_axis, main_vine_spin_deg)
            calyx_tip = tomato_pos + stem_axis * (self.tomato_radius * 1.26 + 0.01)
            branch_vector = calyx_tip - branch_start
            branch_distance = float(np.linalg.norm(branch_vector))
            if branch_distance < 1e-6:
                branch_direction = stem_axis
            else:
                branch_direction = branch_vector / branch_distance
            straight_stem_length = min(
                self.object_radius * float(spec["stem_segment_len"]),
                max(branch_distance, 0.0) * 0.65,
            )
            branch_elbow = calyx_tip - branch_direction * straight_stem_length
            z_spin_deg = self._local_tomato_z_spin_deg(map_branch_start, map_tomato_pos)
            entries.append(
                {
                    "index": index,
                    "target": bool(spec["target"]),
                    "position": tomato_pos,
                    "branch_start": branch_start,
                    "branch_elbow": branch_elbow,
                    "calyx_tip": calyx_tip,
                    "base_stem_axis": base_stem_axis,
                    "stem_axis": stem_axis,
                    "z_spin_deg": z_spin_deg,
                    "main_vine_spin_deg": main_vine_spin_deg,
                }
            )
        return entries

    def _append_other_tomato_markers(self, markers: MarkerArray, stamp) -> None:
        harvested = self._harvested_tomato_ids()
        attached_tomato_id = self._attached_tomato_id()
        for entry in self.tomato_entries:
            if bool(entry["target"]):
                continue
            marker_id = int(entry["index"])
            object_id = f"other_tomato_{marker_id}"
            if object_id in harvested or object_id == attached_tomato_id:
                markers.markers.append(self._delete_marker(marker_id, stamp, "other_tomato"))
                markers.markers.append(self._delete_marker(marker_id, stamp, "other_calyx_stem"))
                for leaf_index in range(6):
                    markers.markers.append(
                        self._delete_marker(marker_id * 10 + leaf_index, stamp, "other_calyx_leaf")
                    )
                branch_start = np.array(entry["branch_start"], dtype=float)
                branch_elbow = np.array(entry["branch_elbow"], dtype=float)
                calyx_tip = np.array(entry["calyx_tip"], dtype=float)
                markers.markers.append(
                    self._cylinder_marker(
                        marker_id * 2,
                        stamp,
                        "other_branch",
                        branch_start,
                        branch_elbow,
                        self.object_radius * 0.06,
                        [0.25, 0.42, 0.20, 1.0],
                    )
                )
                markers.markers.append(
                    self._cylinder_marker(
                        marker_id * 2 + 1,
                        stamp,
                        "other_branch",
                        branch_elbow,
                        calyx_tip,
                        self.object_radius * 0.06,
                        [0.25, 0.42, 0.20, 1.0],
                    )
                )
                continue
            tomato_pos = np.array(entry["position"], dtype=float)
            base_stem_axis = np.array(entry["base_stem_axis"], dtype=float)
            stem_axis = np.array(entry["stem_axis"], dtype=float)
            z_spin_deg = float(entry["z_spin_deg"])
            main_vine_spin_deg = float(entry["main_vine_spin_deg"])
            branch_start = np.array(entry["branch_start"], dtype=float)
            branch_elbow = np.array(entry["branch_elbow"], dtype=float)
            calyx_tip = np.array(entry["calyx_tip"], dtype=float)

            markers.markers.append(
                self._sphere_marker(
                    marker_id,
                    stamp,
                    "other_tomato",
                    tomato_pos,
                    self.tomato_radius,
                    [0.92, 0.05, 0.035, 1.0],
                )
            )
            markers.markers.append(
                self._cylinder_marker(
                    marker_id * 2,
                    stamp,
                    "other_branch",
                    branch_start,
                    branch_elbow,
                    self.object_radius * 0.06,
                    [0.25, 0.42, 0.20, 1.0],
                )
            )
            markers.markers.append(
                self._cylinder_marker(
                    marker_id * 2 + 1,
                    stamp,
                    "other_branch",
                    branch_elbow,
                    calyx_tip,
                    self.object_radius * 0.06,
                    [0.25, 0.42, 0.20, 1.0],
                )
            )
            markers.markers.append(
                self._cylinder_marker(
                    marker_id,
                    stamp,
                    "other_calyx_stem",
                    tomato_pos + stem_axis * self.tomato_radius,
                    calyx_tip,
                    self.tomato_radius * 0.11,
                    [0.02, 0.66, 0.12, 1.0],
                )
            )
            for leaf_index in range(6):
                markers.markers.append(
                    self._leaf_marker_for(
                        marker_id * 10 + leaf_index,
                        stamp,
                        "other_calyx_leaf",
                        leaf_index,
                        tomato_pos,
                        base_stem_axis,
                        z_spin_deg,
                        main_vine_spin_deg,
                    )
                )

    def _append_attached_tomato_marker(self, markers: MarkerArray, stamp) -> None:
        attached_tomato_id = self._attached_tomato_id()
        if not attached_tomato_id:
            markers.markers.append(self._delete_marker(0, stamp, "attached_tomato"))
            return

        marker = Marker()
        marker.header.frame_id = str(self.attached_tomato_frame)
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "attached_tomato"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.position.x = float(self.attached_tomato_offset_xyz[0])
        marker.pose.position.y = float(self.attached_tomato_offset_xyz[1])
        marker.pose.position.z = float(self.attached_tomato_offset_xyz[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.tomato_radius * 2.0
        marker.scale.y = self.tomato_radius * 2.0
        marker.scale.z = self.tomato_radius * 2.0
        marker.color.r = 0.92
        marker.color.g = 0.05
        marker.color.b = 0.035
        marker.color.a = 1.0
        markers.markers.append(marker)

    def _sphere_marker(self, marker_id, stamp, ns, pos, radius, color) -> Marker:
        marker = Marker()
        self._set_visual_marker_header(marker)
        pos = self._world_visual_position(pos)
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(pos[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = radius * 2.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _tomato_positive_x_ring_marker(self, marker_id, stamp) -> Marker:
        tomato_x_axis, tomato_y_axis, tomato_z_axis = self._tomato_frame_axes()
        center = self.object_position + tomato_x_axis * self.tomato_radius * 1.018
        ring_radius = self.tomato_radius * 0.18

        marker = Marker()
        self._set_visual_marker_header(marker)
        marker.ns = "tomato_positive_x_surface"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.tomato_radius * 0.045
        marker.color.r = 0.42
        marker.color.g = 0.02
        marker.color.b = 0.95
        marker.color.a = 1.0

        segment_count = 40
        for index in range(segment_count + 1):
            angle = 2.0 * math.pi * index / segment_count
            point = center + ring_radius * (
                math.cos(angle) * tomato_y_axis + math.sin(angle) * tomato_z_axis
            )
            point = self._world_visual_position(point)
            marker.points.append(Point(x=float(point[0]), y=float(point[1]), z=float(point[2])))
        return marker

    def _cylinder_marker(self, marker_id, stamp, ns, start, end, radius, color=None) -> Marker:
        if color is None:
            color = [0.24, 0.40, 0.18, 1.0]
        start = self._world_visual_position(start)
        end = self._world_visual_position(end)
        axis = end - start
        length = float(np.linalg.norm(axis))
        marker = Marker()
        self._set_visual_marker_header(marker)
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float((start[0] + end[0]) * 0.5)
        marker.pose.position.y = float((start[1] + end[1]) * 0.5)
        marker.pose.position.z = float((start[2] + end[2]) * 0.5)
        qx, qy, qz, qw = orientation_from_z_axis(axis)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = length
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _leaf_marker(self, marker_id, stamp, leaf_index) -> Marker:
        return self._leaf_marker_for(
            marker_id,
            stamp,
            "target_calyx_leaf",
            leaf_index,
            self.object_position,
            self.base_stem_axis,
            self.target_z_spin_deg,
            self.target_main_vine_spin_deg,
        )

    def _leaf_marker_for(
        self,
        marker_id,
        stamp,
        ns,
        leaf_index,
        object_position,
        stem_axis,
        z_spin_deg,
        main_vine_spin_deg=None,
    ) -> Marker:
        basis_x, basis_y, axis = self._spun_tomato_frame_axes_for(
            stem_axis,
            z_spin_deg,
            main_vine_spin_deg,
        )
        object_position = np.array(object_position, dtype=float)
        angle = 2.0 * math.pi * leaf_index / 6.0
        radial = math.cos(angle) * basis_x + math.sin(angle) * basis_y
        center = object_position + axis * (self.tomato_radius * 1.04) + radial * (self.tomato_radius * 0.24)
        center = self._world_visual_position(center)
        marker = Marker()
        self._set_visual_marker_header(marker)
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        qx, qy, qz, qw = orientation_from_z_axis(axis)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = self.tomato_radius * 0.72
        marker.scale.y = self.tomato_radius * 0.20
        marker.scale.z = self.tomato_radius * 0.08
        marker.color.r = 0.0
        marker.color.g = 0.52
        marker.color.b = 0.08
        marker.color.a = 1.0
        return marker

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

    def _local_tomato_z_spin_deg(self, branch_start, tomato_pos) -> float:
        z_spin_deg = self.taught_tomato_z_spin_deg
        if self._is_left_of_vine_from_robot(branch_start, tomato_pos):
            z_spin_deg += self.left_tomato_z_spin_offset_deg
        else:
            z_spin_deg += self.right_tomato_z_spin_offset_deg
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

    def _tomato_frame_axes(self):
        return self._spun_tomato_frame_axes_for(
            self.base_stem_axis,
            self.target_z_spin_deg,
            self.target_main_vine_spin_deg,
        )

    def _main_vine_frame_axes(self):
        return self._tomato_frame_axes_for(self.main_end - self.main_start, self.tomato_z_spin_deg)

    def _spun_tomato_frame_axes_for(self, base_stem_axis, z_spin_deg, main_vine_spin_deg=None):
        base_axes = np.column_stack(self._tomato_frame_axes_for(base_stem_axis, z_spin_deg))
        spun_axes = self._main_vine_spin_matrix(main_vine_spin_deg) @ base_axes
        return spun_axes[:, 0], spun_axes[:, 1], spun_axes[:, 2]

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
        tomato_x_axis = spun_x_axis
        tomato_y_axis = spun_y_axis
        return tomato_x_axis, tomato_y_axis, tomato_z_axis


def main() -> None:
    rclpy.init()
    node = TomatoSceneNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
