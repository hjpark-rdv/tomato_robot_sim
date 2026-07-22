import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


robot_ip = LaunchConfiguration("robot_ip")
use_fake_hardware = LaunchConfiguration("use_fake_hardware")
fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
model_id = LaunchConfiguration("model_id")
cb_simulation = LaunchConfiguration("cb_simulation")
activate_arm_controller = LaunchConfiguration("activate_arm_controller")
show_tomato_scene = LaunchConfiguration("show_tomato_scene")
enable_detected_tomato_tf = LaunchConfiguration("enable_detected_tomato_tf")

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.0.2.7",
            description="RB Cobot Control Box IP Address",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="True if there's no RB Cobot Control Box",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="True when use fake sensor commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cb_simulation",
            default_value="Simulation",
            description="Select RB Control Box mode, Simulation or Real",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "model_id",
            default_value="rb5_farmily",
            description="RB Series currently using",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_arm_controller",
            default_value="true",
            description="Activate the trajectory controller at startup",
        )
    )
    declared_arguments.extend(
        [
            DeclareLaunchArgument(
                "show_tomato_scene",
                default_value="true",
                description="Show the procedural tomato vine scene in RViz",
            ),
            DeclareLaunchArgument("tomato_x", default_value="0.455"),
            DeclareLaunchArgument("tomato_y", default_value="-0.175"),
            DeclareLaunchArgument("tomato_z", default_value="0.34"),
            DeclareLaunchArgument("tomato_radius_scale", default_value="0.5"),
            DeclareLaunchArgument(
                "show_vine_row",
                default_value="false",
                description="Repeat the first tomato vine along the Y axis",
            ),
            DeclareLaunchArgument(
                "publish_tomato_collisions",
                default_value="false",
                description="Add tomato plants to the MoveIt planning scene as collision objects",
            ),
            DeclareLaunchArgument(
                "enable_detected_tomato_tf",
                default_value="true",
                description="Enable service-driven camera tomato TF generation",
            ),
            DeclareLaunchArgument(
                "tomato_camera_frame",
                default_value="d435_color_optical_frame",
                description="Parent camera frame for newly detected tomato TFs",
            ),
            DeclareLaunchArgument(
                "auto_create_detected_tomato_tf",
                default_value="true",
                description="Create a TF as soon as a new camera detection pair arrives",
            ),
        ]
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    tomato_position = [
        float(LaunchConfiguration("tomato_x").perform(context)),
        float(LaunchConfiguration("tomato_y").perform(context)),
        float(LaunchConfiguration("tomato_z").perform(context)),
    ]
    tomato_radius_scale = float(
        LaunchConfiguration("tomato_radius_scale").perform(context)
    )
    show_vine_row_value = (
        LaunchConfiguration("show_vine_row").perform(context).lower()
        in {"1", "true", "yes", "on"}
    )
    publish_tomato_collisions = (
        LaunchConfiguration("publish_tomato_collisions").perform(context).lower()
        in {"1", "true", "yes", "on"}
    )

    mappings = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "fake_sensor_commands": fake_sensor_commands,
        "model_id": model_id,
        "cb_simulation": cb_simulation,
    }

    moveit_config = (
        MoveItConfigsBuilder("rbpodo")
        .robot_description(file_path="config/rbpodo.urdf.xacro", mappings=mappings)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rbpodo_moveit_config"), "config", rviz_base]
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rbpodo_bringup"),
        "config",
        "controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    active_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        condition=IfCondition(activate_arm_controller),
    )

    inactive_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        condition=UnlessCondition(activate_arm_controller),
    )

    tomato_scene_node = Node(
        package="tomato_moveit_teach",
        executable="tomato_scene_node",
        name="tomato_scene_node",
        parameters=[
            {
                "base_frame": "link0",
                "world_frame": "world",
                "floor_frame": "tomato_floor_tf",
                "tomato_frame": "tomato_tf",
                "object_position": tomato_position,
                "tomato_radius_scale": tomato_radius_scale,
                "show_vine_row": show_vine_row_value,
                "publish_planning_scene": publish_tomato_collisions,
                "include_other_tomato_collision": publish_tomato_collisions,
                "include_target_tomato_collision": False,
                "include_target_branch_collision": False,
                "include_ground_collision": False,
                "include_ceiling_collision": False,
                "include_rear_wall_collision": False,
                "include_robot_rear_wall_collision": False,
                "include_robot_front_wall_collision": False,
                "include_robot_side_wall_collision": False,
                "include_rail_pipe_collision": False,
            }
        ],
        condition=IfCondition(show_tomato_scene),
        output="screen",
    )

    tomato_tf_generator = Node(
        package="rbpodo_tomato_harvest",
        executable="tomato_tf_generator",
        name="tomato_tf_generator",
        parameters=[
            {
                "camera_frame": LaunchConfiguration("tomato_camera_frame"),
                "sky_frame": "link0",
                "auto_create_on_detection": LaunchConfiguration(
                    "auto_create_detected_tomato_tf"
                ),
            }
        ],
        condition=IfCondition(enable_detected_tomato_tf),
        output="screen",
    )

    nodes_to_start = [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        active_arm_controller_spawner,
        inactive_arm_controller_spawner,
        tomato_scene_node,
        tomato_tf_generator,
    ]

    return nodes_to_start
