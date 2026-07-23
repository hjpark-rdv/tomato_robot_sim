from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_service",
                default_value="/fake_tomato_camera/detect_tomatoes",
            ),
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/tomato_detection/detections",
            ),
            DeclareLaunchArgument(
                "scene_node",
                default_value="/tomato_scene_node",
            ),
            DeclareLaunchArgument(
                "default_planner",
                default_value="cartesian",
            ),
            Node(
                package="rbpodo_tomato_harvest",
                executable="harvest_gui",
                name="tomato_harvest_gui",
                output="screen",
                parameters=[
                    {
                        "camera_service": LaunchConfiguration("camera_service"),
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "scene_node": LaunchConfiguration("scene_node"),
                        "default_planner": LaunchConfiguration("default_planner"),
                    }
                ],
            ),
        ]
    )
