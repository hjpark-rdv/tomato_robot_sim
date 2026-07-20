import math
import io
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_robot_description(
    package_share: Path,
    repo_root: Path,
    mobile_base_size_x: float,
    mobile_base_size_y: float,
    mobile_base_size_z: float,
    mobile_base_center_z: float,
    camera_enabled: bool,
    camera_offset_z: float,
    camera_size_x: float,
    camera_size_y: float,
    camera_size_z: float,
    tool_length: float,
    tool_radius: float,
    tool_tilt_x_deg: float,
    tool_tilt_y_deg: float,
    tool_bend_straight_ratio: float,
    tool_gripper_under_offset_ratio: float,
    gripper_opening: float,
    gripper_x_roll_deg: float,
    gripper_spin_zero_offset_rad: float,
    gripper_z_rot_rad: float,
) -> str:
    urdf_path = repo_root / "assets" / "ur5" / "urdf" / "ur5.urdf"
    urdf = urdf_path.read_text(encoding="utf-8")
    mesh_root = (repo_root / "assets" / "ur5" / "meshes").resolve()
    urdf = urdf.replace('filename="../meshes/', f'filename="file://{mesh_root}/')
    urdf = append_mobile_base(
        urdf,
        mobile_base_size_x,
        mobile_base_size_y,
        mobile_base_size_z,
        mobile_base_center_z,
    )
    return append_tool_with_camera_xacro(
        urdf,
        package_share,
        camera_enabled,
        camera_offset_z,
        camera_size_x,
        camera_size_y,
        camera_size_z,
        tool_length,
        tool_radius,
        tool_tilt_x_deg,
        tool_bend_straight_ratio,
        tool_gripper_under_offset_ratio,
        gripper_opening,
        gripper_x_roll_deg,
        gripper_spin_zero_offset_rad,
    )


def append_mobile_base(
    urdf: str,
    size_x: float,
    size_y: float,
    size_z: float,
    center_z: float,
) -> str:
    mobile_base_xml = f"""
  <material name="mobile_base_dark">
    <color rgba="0.02 0.18 0.08 1.0"/>
  </material>
  <link name="mobile_base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{float(size_x):.6f} {float(size_y):.6f} {float(size_z):.6f}"/>
      </geometry>
      <material name="mobile_base_dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{float(size_x):.6f} {float(size_y):.6f} {float(size_z):.6f}"/>
      </geometry>
    </collision>
  </link>
  <joint name="base_link_to_mobile_base" type="fixed">
    <parent link="base_link"/>
    <child link="mobile_base_link"/>
    <origin xyz="0 0 {float(center_z):.6f}" rpy="0 0 0"/>
  </joint>
"""
    return urdf.replace("</robot>", f"{mobile_base_xml}\n</robot>")


def append_tool_with_camera_xacro(
    urdf: str,
    package_share: Path,
    camera_enabled: bool,
    camera_offset_z: float,
    camera_size_x: float,
    camera_size_y: float,
    camera_size_z: float,
    tool_length: float,
    tool_radius: float,
    tool_tilt_x_deg: float,
    tool_bend_straight_ratio: float,
    tool_gripper_under_offset_ratio: float,
    gripper_opening: float,
    gripper_x_roll_deg: float,
    gripper_spin_zero_offset_rad: float,
) -> str:
    xacro_path = package_share / "urdf" / "tomato_tool_with_camera.urdf.xacro"
    if not xacro_path.exists():
        raise FileNotFoundError(f"Tool/camera xacro file does not exist: {xacro_path}")

    if "xmlns:xacro" not in urdf:
        urdf = urdf.replace("<robot ", '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" ', 1)

    tool_xacro_xml = f"""
  <xacro:include filename="{xacro_path}"/>
  <xacro:tomato_tool_with_camera
    enable_tool_camera="{str(bool(camera_enabled)).lower()}"
    tool_camera_offset_z="{float(camera_offset_z):.12f}"
    tool_camera_size_x="{float(camera_size_x):.12f}"
    tool_camera_size_y="{float(camera_size_y):.12f}"
    tool_camera_size_z="{float(camera_size_z):.12f}"
    tool_length="{float(tool_length):.12f}"
    tool_radius="{float(tool_radius):.12f}"
    tool_tilt_x_deg="{float(tool_tilt_x_deg):.12f}"
    tool_bend_straight_ratio="{float(tool_bend_straight_ratio):.12f}"
    tool_gripper_under_offset_ratio="{float(tool_gripper_under_offset_ratio):.12f}"
    gripper_opening="{float(gripper_opening):.12f}"
    gripper_x_roll_deg="{float(gripper_x_roll_deg):.12f}"
    gripper_spin_zero_offset_rad="{float(gripper_spin_zero_offset_rad):.12f}"/>
"""
    xacro_urdf = urdf.replace("</robot>", f"{tool_xacro_xml}\n</robot>")
    doc = xacro.parse(io.StringIO(xacro_urdf))
    xacro.process_doc(doc)
    return doc.toxml()


def launch_setup(context, *args, **kwargs):
    package_share = Path(get_package_share_directory("tomato_moveit_teach"))
    config_dir = package_share / "config"
    rviz_config = package_share / "rviz" / "teach.rviz"

    repo_root = Path(LaunchConfiguration("repo_root").perform(context)).expanduser().resolve()
    object_position = [
        float(LaunchConfiguration("object_x").perform(context)),
        float(LaunchConfiguration("object_y").perform(context)),
        float(LaunchConfiguration("object_z").perform(context)),
    ]
    scene_y_offset = float(LaunchConfiguration("scene_y_offset").perform(context))
    robot_y_offset = float(LaunchConfiguration("robot_y_offset").perform(context))
    object_radius = float(LaunchConfiguration("object_radius").perform(context))
    tomato_radius_scale = float(LaunchConfiguration("tomato_radius_scale").perform(context))
    main_vine_visual_length_scale = float(LaunchConfiguration("main_vine_visual_length_scale").perform(context))
    tomato_z_spin_deg = float(LaunchConfiguration("tomato_z_spin_deg").perform(context))
    taught_tomato_z_spin_deg = float(LaunchConfiguration("taught_tomato_z_spin_deg").perform(context))
    right_tomato_z_spin_offset_deg = float(
        LaunchConfiguration("right_tomato_z_spin_offset_deg").perform(context)
    )
    left_tomato_z_spin_offset_deg = float(
        LaunchConfiguration("left_tomato_z_spin_offset_deg").perform(context)
    )
    randomize_individual_stem_spin = LaunchConfiguration("randomize_individual_stem_spin").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    individual_stem_spin_seed = int(float(LaunchConfiguration("individual_stem_spin_seed").perform(context)))
    individual_stem_spin_min_deg = float(LaunchConfiguration("individual_stem_spin_min_deg").perform(context))
    individual_stem_spin_max_deg = float(LaunchConfiguration("individual_stem_spin_max_deg").perform(context))
    output_path = str(Path(LaunchConfiguration("output_path").perform(context)).expanduser().resolve())
    mobile_base_size_x = float(LaunchConfiguration("mobile_base_size_x").perform(context))
    mobile_base_size_y = float(LaunchConfiguration("mobile_base_size_y").perform(context))
    mobile_base_size_z = float(LaunchConfiguration("mobile_base_size_z").perform(context))
    mobile_base_center_z = float(LaunchConfiguration("mobile_base_center_z").perform(context))
    camera_enabled = LaunchConfiguration("enable_tool_camera").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    camera_offset_z = float(LaunchConfiguration("tool_camera_offset_z").perform(context))
    camera_size_x = float(LaunchConfiguration("tool_camera_size_x").perform(context))
    camera_size_y = float(LaunchConfiguration("tool_camera_size_y").perform(context))
    camera_size_z = float(LaunchConfiguration("tool_camera_size_z").perform(context))
    camera_width = int(LaunchConfiguration("tool_camera_width").perform(context))
    camera_height = int(LaunchConfiguration("tool_camera_height").perform(context))
    camera_fps = float(LaunchConfiguration("tool_camera_fps").perform(context))
    camera_depth_m = float(LaunchConfiguration("tool_camera_synthetic_depth_m").perform(context))
    camera_horizontal_fov_deg = float(LaunchConfiguration("tool_camera_horizontal_fov_deg").perform(context))
    camera_vertical_fov_deg = float(LaunchConfiguration("tool_camera_vertical_fov_deg").perform(context))
    tool_length = float(LaunchConfiguration("tool_length").perform(context))
    tool_radius = float(LaunchConfiguration("tool_radius").perform(context))
    tool_tilt_x_deg = float(LaunchConfiguration("tool_tilt_x_deg").perform(context))
    tool_tilt_y_deg = float(LaunchConfiguration("tool_tilt_y_deg").perform(context))
    tool_tilt_y_rad = tool_tilt_y_deg * 3.141592653589793 / 180.0
    tool_bend_straight_ratio = float(LaunchConfiguration("tool_bend_straight_ratio").perform(context))
    tool_gripper_under_offset_ratio = float(
        LaunchConfiguration("tool_gripper_under_offset_ratio").perform(context)
    )
    gripper_opening = float(LaunchConfiguration("gripper_opening").perform(context))
    gripper_x_roll_deg = float(LaunchConfiguration("gripper_x_roll_deg").perform(context))
    gripper_spin_zero_offset_deg = float(
        LaunchConfiguration("gripper_spin_zero_offset_deg").perform(context)
    )
    gripper_spin_zero_offset_rad = gripper_spin_zero_offset_deg * 3.141592653589793 / 180.0
    gripper_z_rot_deg = float(LaunchConfiguration("gripper_z_rot_deg").perform(context))
    gripper_z_rot = gripper_z_rot_deg * 3.141592653589793 / 180.0
    include_ground_collision = LaunchConfiguration("include_ground_collision").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    include_ceiling_collision = LaunchConfiguration("include_ceiling_collision").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    ceiling_z = float(LaunchConfiguration("ceiling_z").perform(context))
    ceiling_thickness = float(LaunchConfiguration("ceiling_thickness").perform(context))
    ceiling_width = float(LaunchConfiguration("ceiling_width").perform(context))
    ceiling_depth = float(LaunchConfiguration("ceiling_depth").perform(context))
    include_rear_wall_collision = LaunchConfiguration("include_rear_wall_collision").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    rear_wall_distance = float(LaunchConfiguration("rear_wall_distance").perform(context))
    rear_wall_thickness = float(LaunchConfiguration("rear_wall_thickness").perform(context))
    rear_wall_width = float(LaunchConfiguration("rear_wall_width").perform(context))
    rear_wall_height = float(LaunchConfiguration("rear_wall_height").perform(context))
    include_robot_rear_wall_collision = LaunchConfiguration("include_robot_rear_wall_collision").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    robot_rear_wall_x = float(LaunchConfiguration("robot_rear_wall_x").perform(context))
    robot_rear_wall_thickness = float(LaunchConfiguration("robot_rear_wall_thickness").perform(context))
    robot_rear_wall_width = float(LaunchConfiguration("robot_rear_wall_width").perform(context))
    robot_rear_wall_height = float(LaunchConfiguration("robot_rear_wall_height").perform(context))
    include_robot_front_wall_collision = LaunchConfiguration("include_robot_front_wall_collision").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    robot_front_wall_x = float(LaunchConfiguration("robot_front_wall_x").perform(context))
    robot_front_wall_thickness = float(LaunchConfiguration("robot_front_wall_thickness").perform(context))
    robot_front_wall_width = float(LaunchConfiguration("robot_front_wall_width").perform(context))
    robot_front_wall_height = float(LaunchConfiguration("robot_front_wall_height").perform(context))
    include_robot_side_wall_collision = LaunchConfiguration("include_robot_side_wall_collision").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    robot_side_wall_y = float(LaunchConfiguration("robot_side_wall_y").perform(context))
    robot_side_wall_thickness = float(LaunchConfiguration("robot_side_wall_thickness").perform(context))
    robot_side_wall_depth = float(LaunchConfiguration("robot_side_wall_depth").perform(context))
    robot_side_wall_height = float(LaunchConfiguration("robot_side_wall_height").perform(context))
    include_rail_pipe_collision = LaunchConfiguration("include_rail_pipe_collision").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    rail_pipe_length = float(LaunchConfiguration("rail_pipe_length").perform(context))
    rail_pipe_diameter = float(LaunchConfiguration("rail_pipe_diameter").perform(context))
    rail_pipe_center_spacing = float(LaunchConfiguration("rail_pipe_center_spacing").perform(context))
    rail_pipe_center_z = float(LaunchConfiguration("rail_pipe_center_z").perform(context))
    show_target_tomato_only = LaunchConfiguration("show_target_tomato_only").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    show_vine_row = LaunchConfiguration("show_vine_row").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    vine_row_start_y = float(LaunchConfiguration("vine_row_start_y").perform(context))
    vine_row_end_y = float(LaunchConfiguration("vine_row_end_y").perform(context))
    vine_row_spacing_y = float(LaunchConfiguration("vine_row_spacing_y").perform(context))
    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        "tool_bend_joint",
        "tool_gripper_z_joint",
    ]
    joint_positions = [
        -1.4379491029641744,
        -1.1907401577003016,
        -2.116742584205267,
        -1.3810410438314462,
        1.5765461198564021,
        -8.504489148035646e-05,
        0.6673960827876584,
        0.035107627975484376,
    ]

    robot_description = {
        "robot_description": load_robot_description(
            package_share,
            repo_root,
            mobile_base_size_x,
            mobile_base_size_y,
            mobile_base_size_z,
            mobile_base_center_z,
            camera_enabled,
            camera_offset_z,
            camera_size_x,
            camera_size_y,
            camera_size_z,
            tool_length,
            tool_radius,
            tool_tilt_x_deg,
            tool_tilt_y_deg,
            tool_bend_straight_ratio,
            tool_gripper_under_offset_ratio,
            gripper_opening,
            gripper_x_roll_deg,
            gripper_spin_zero_offset_rad,
            gripper_z_rot,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": (config_dir / "ur5.srdf").read_text(encoding="utf-8")
    }
    robot_description_kinematics = {"robot_description_kinematics": load_yaml(config_dir / "kinematics.yaml")}
    robot_description_planning = {"robot_description_planning": load_yaml(config_dir / "joint_limits.yaml")}
    ompl_planning = load_yaml(config_dir / "ompl_planning.yaml")
    moveit_controllers = load_yaml(config_dir / "moveit_controllers.yaml")

    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning,
        moveit_controllers,
        {
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": True,
            "capabilities": "",
            "disable_capabilities": "",
        },
    ]

    rviz_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning,
    ]

    return [
        Node(
            package="tomato_moveit_teach",
            executable="mobile_base_tf_publisher",
            parameters=[
                {
                    "world_frame": "world",
                    "base_frame": "base_link",
                    "robot_y_offset": robot_y_offset,
                }
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="tomato_moveit_teach",
            executable="tool_depth_camera_publisher",
            parameters=[
                {
                    "depth_frame_id": "tool_camera_depth_optical_frame",
                    "color_frame_id": "tool_camera_color_optical_frame",
                    "width": camera_width,
                    "height": camera_height,
                    "fps": camera_fps,
                    "horizontal_fov_deg": camera_horizontal_fov_deg,
                    "vertical_fov_deg": camera_vertical_fov_deg,
                    "synthetic_depth_m": camera_depth_m,
                    "publish_point_cloud": True,
                    "point_cloud_stride": 4,
                }
            ],
            condition=IfCondition(LaunchConfiguration("enable_tool_camera")),
            output="screen",
        ),
        Node(
            package="tomato_moveit_teach",
            executable="home_joint_state_publisher",
            parameters=[
                {
                    "output_path": output_path,
                    "joint_names": joint_names,
                    "joint_positions": joint_positions,
                    "gripper_spin_joint_name": "tool_gripper_z_joint",
                    "gripper_z_rot_deg": gripper_z_rot_deg,
                }
            ],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=move_group_params,
            output="screen",
        ),
        Node(
            package="tomato_moveit_teach",
            executable="tomato_scene_node",
            parameters=[
                {
                    "base_frame": "base_link",
                    "floor_frame": "floor_tf",
                    "tomato_frame": "tomato_tf",
                    "object_position": object_position,
                    "scene_y_offset": scene_y_offset,
                    "robot_y_offset": robot_y_offset,
                    "object_radius": object_radius,
                    "tomato_radius_scale": tomato_radius_scale,
                    "main_vine_visual_length_scale": main_vine_visual_length_scale,
                    "show_vine_row": show_vine_row,
                    "vine_row_start_y": vine_row_start_y,
                    "vine_row_end_y": vine_row_end_y,
                    "vine_row_spacing_y": vine_row_spacing_y,
                    "tomato_z_spin_deg": tomato_z_spin_deg,
                    "taught_tomato_z_spin_deg": taught_tomato_z_spin_deg,
                    "right_tomato_z_spin_offset_deg": right_tomato_z_spin_offset_deg,
                    "left_tomato_z_spin_offset_deg": left_tomato_z_spin_offset_deg,
                    "randomize_individual_stem_spin": randomize_individual_stem_spin,
                    "individual_stem_spin_seed": individual_stem_spin_seed,
                    "individual_stem_spin_min_deg": individual_stem_spin_min_deg,
                    "individual_stem_spin_max_deg": individual_stem_spin_max_deg,
                    "show_target_tomato_only": show_target_tomato_only,
                    "show_other_tomatoes": not show_target_tomato_only,
                    "include_other_tomato_collision": not show_target_tomato_only,
                    "include_ground_collision": include_ground_collision,
                    "include_ceiling_collision": include_ceiling_collision,
                    "ceiling_z": ceiling_z,
                    "ceiling_thickness": ceiling_thickness,
                    "ceiling_width": ceiling_width,
                    "ceiling_depth": ceiling_depth,
                    "include_rear_wall_collision": include_rear_wall_collision,
                    "rear_wall_distance": rear_wall_distance,
                    "rear_wall_thickness": rear_wall_thickness,
                    "rear_wall_width": rear_wall_width,
                    "rear_wall_height": rear_wall_height,
                    "include_robot_rear_wall_collision": include_robot_rear_wall_collision,
                    "robot_rear_wall_x": robot_rear_wall_x,
                    "robot_rear_wall_thickness": robot_rear_wall_thickness,
                    "robot_rear_wall_width": robot_rear_wall_width,
                    "robot_rear_wall_height": robot_rear_wall_height,
                    "include_robot_front_wall_collision": include_robot_front_wall_collision,
                    "robot_front_wall_x": robot_front_wall_x,
                    "robot_front_wall_thickness": robot_front_wall_thickness,
                    "robot_front_wall_width": robot_front_wall_width,
                    "robot_front_wall_height": robot_front_wall_height,
                    "include_robot_side_wall_collision": include_robot_side_wall_collision,
                    "robot_side_wall_y": robot_side_wall_y,
                    "robot_side_wall_thickness": robot_side_wall_thickness,
                    "robot_side_wall_depth": robot_side_wall_depth,
                    "robot_side_wall_height": robot_side_wall_height,
                    "include_rail_pipe_collision": include_rail_pipe_collision,
                    "rail_pipe_length": rail_pipe_length,
                    "rail_pipe_diameter": rail_pipe_diameter,
                    "rail_pipe_center_spacing": rail_pipe_center_spacing,
                    "rail_pipe_center_z": rail_pipe_center_z,
                }
            ],
            output="screen",
        ),
        Node(
            package="tomato_moveit_teach",
            executable="grasp_pose_saver",
            parameters=[
                {
                    "reference_frame": "tomato_tf",
                    "tool_frame": "grasp_tf",
                    "output_path": output_path,
                }
            ],
            output="screen",
        ),
        Node(
            package="tomato_moveit_teach",
            executable="planned_joint_saver",
            parameters=[
                robot_description,
                {
                    "output_path": output_path,
                    "reference_frame": "base_link",
                    "tool_frame": "grasp_tf",
                    "gripper_joint_names": ["tool_bend_joint", "tool_gripper_z_joint"],
                }
            ],
            output="screen",
        ),
        Node(
            package="tomato_moveit_teach",
            executable="teach_grasp_marker",
            parameters=[
                {
                    "reference_frame": "tomato_tf",
                    "grasp_frame": "taught_grasp_tf",
                    "object_radius": object_radius * tomato_radius_scale,
                    "output_path": output_path,
                }
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            parameters=rviz_params,
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("repo_root", default_value="/root/pybullet_ur_approach"),
            DeclareLaunchArgument("object_x", default_value="0.455"),
            DeclareLaunchArgument("object_y", default_value="-0.175"),
            DeclareLaunchArgument("object_z", default_value="0.34"),
            DeclareLaunchArgument("scene_y_offset", default_value="0.0"),
            DeclareLaunchArgument("robot_y_offset", default_value="0.0"),
            DeclareLaunchArgument("object_radius", default_value="0.036"),
            DeclareLaunchArgument("tomato_radius_scale", default_value="0.5"),
            DeclareLaunchArgument("main_vine_visual_length_scale", default_value="5.0"),
            DeclareLaunchArgument("show_vine_row", default_value="true"),
            DeclareLaunchArgument("vine_row_start_y", default_value="0.0"),
            DeclareLaunchArgument("vine_row_end_y", default_value="5.0"),
            DeclareLaunchArgument("vine_row_spacing_y", default_value="0.5"),
            DeclareLaunchArgument("tomato_z_spin_deg", default_value="0.0"),
            DeclareLaunchArgument("taught_tomato_z_spin_deg", default_value="-40.0"),
            DeclareLaunchArgument("right_tomato_z_spin_offset_deg", default_value="-30.0"),
            DeclareLaunchArgument("left_tomato_z_spin_offset_deg", default_value="250.0"),
            DeclareLaunchArgument("randomize_individual_stem_spin", default_value="false"),
            DeclareLaunchArgument("individual_stem_spin_seed", default_value="0"),
            DeclareLaunchArgument("individual_stem_spin_min_deg", default_value="0.0"),
            DeclareLaunchArgument("individual_stem_spin_max_deg", default_value="360.0"),
            DeclareLaunchArgument("output_path", default_value="/root/pybullet_ur_approach/taught_grasps_moveit.json"),
            DeclareLaunchArgument("mobile_base_size_x", default_value="0.60"),
            DeclareLaunchArgument("mobile_base_size_y", default_value="1.00"),
            DeclareLaunchArgument("mobile_base_size_z", default_value="0.10"),
            DeclareLaunchArgument("mobile_base_center_z", default_value="-0.05"),
            DeclareLaunchArgument("enable_tool_camera", default_value="true"),
            DeclareLaunchArgument("tool_camera_offset_z", default_value="0.05"),
            DeclareLaunchArgument("tool_camera_size_x", default_value="0.085"),
            DeclareLaunchArgument("tool_camera_size_y", default_value="0.028"),
            DeclareLaunchArgument("tool_camera_size_z", default_value="0.028"),
            DeclareLaunchArgument("tool_camera_width", default_value="640"),
            DeclareLaunchArgument("tool_camera_height", default_value="480"),
            DeclareLaunchArgument("tool_camera_fps", default_value="15.0"),
            DeclareLaunchArgument("tool_camera_horizontal_fov_deg", default_value="87.0"),
            DeclareLaunchArgument("tool_camera_vertical_fov_deg", default_value="58.0"),
            DeclareLaunchArgument("tool_camera_synthetic_depth_m", default_value="1.20"),
            DeclareLaunchArgument("tool_length", default_value="0.15"),
            DeclareLaunchArgument("tool_radius", default_value="0.006"),
            DeclareLaunchArgument("tool_tilt_x_deg", default_value="0.0"),
            DeclareLaunchArgument("tool_tilt_y_deg", default_value="-45.0"),
            DeclareLaunchArgument("tool_bend_straight_ratio", default_value="0.5"),
            DeclareLaunchArgument("tool_gripper_under_offset_ratio", default_value="0.5"),
            DeclareLaunchArgument("gripper_opening", default_value="0.0525"),
            DeclareLaunchArgument("gripper_x_roll_deg", default_value="90.0"),
            DeclareLaunchArgument("gripper_spin_zero_offset_deg", default_value="90.0"),
            DeclareLaunchArgument("gripper_z_rot_deg", default_value="2.0115189117106738"),
            DeclareLaunchArgument("include_ground_collision", default_value="true"),
            DeclareLaunchArgument("include_ceiling_collision", default_value="true"),
            DeclareLaunchArgument("ceiling_z", default_value="0.80"),
            DeclareLaunchArgument("ceiling_thickness", default_value="0.040"),
            DeclareLaunchArgument("ceiling_width", default_value="1.60"),
            DeclareLaunchArgument("ceiling_depth", default_value="1.60"),
            DeclareLaunchArgument("include_rear_wall_collision", default_value="true"),
            DeclareLaunchArgument("rear_wall_distance", default_value="0.10"),
            DeclareLaunchArgument("rear_wall_thickness", default_value="0.035"),
            DeclareLaunchArgument("rear_wall_width", default_value="0.75"),
            DeclareLaunchArgument("rear_wall_height", default_value="1.10"),
            DeclareLaunchArgument("include_robot_rear_wall_collision", default_value="true"),
            DeclareLaunchArgument("robot_rear_wall_x", default_value="-0.50"),
            DeclareLaunchArgument("robot_rear_wall_thickness", default_value="0.040"),
            DeclareLaunchArgument("robot_rear_wall_width", default_value="1.20"),
            DeclareLaunchArgument("robot_rear_wall_height", default_value="1.20"),
            DeclareLaunchArgument("include_robot_front_wall_collision", default_value="false"),
            DeclareLaunchArgument("robot_front_wall_x", default_value="0.62"),
            DeclareLaunchArgument("robot_front_wall_thickness", default_value="0.040"),
            DeclareLaunchArgument("robot_front_wall_width", default_value="1.20"),
            DeclareLaunchArgument("robot_front_wall_height", default_value="1.20"),
            DeclareLaunchArgument("include_robot_side_wall_collision", default_value="true"),
            DeclareLaunchArgument("robot_side_wall_y", default_value="0.50"),
            DeclareLaunchArgument("robot_side_wall_thickness", default_value="0.040"),
            DeclareLaunchArgument("robot_side_wall_depth", default_value="1.60"),
            DeclareLaunchArgument("robot_side_wall_height", default_value="1.20"),
            DeclareLaunchArgument("include_rail_pipe_collision", default_value="true"),
            DeclareLaunchArgument("rail_pipe_length", default_value="10.0"),
            DeclareLaunchArgument("rail_pipe_diameter", default_value="0.050"),
            DeclareLaunchArgument("rail_pipe_center_spacing", default_value="0.55"),
            DeclareLaunchArgument("rail_pipe_center_z", default_value="-0.127"),
            DeclareLaunchArgument("show_target_tomato_only", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
