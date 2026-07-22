from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_frame",
                default_value="d435_color_optical_frame",
            ),
            DeclareLaunchArgument(
                "stem_frame",
                default_value="main_vine_tf",
            ),
            Node(
                package="rbpodo_tomato_harvest",
                executable="fake_camera_service",
                name="fake_tomato_camera",
                output="screen",
                parameters=[
                    {
                        "camera_frame": LaunchConfiguration("camera_frame"),
                        "stem_frame": LaunchConfiguration("stem_frame"),
                    }
                ],
            ),
        ]
    )
