from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            DeclareLaunchArgument(
                "sorting_frame",
                default_value="link0",
            ),
            DeclareLaunchArgument(
                "vertical_column_xy_tolerance",
                default_value="0.05",
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
                        "sorting_frame": LaunchConfiguration("sorting_frame"),
                        "vertical_column_xy_tolerance": ParameterValue(
                            LaunchConfiguration("vertical_column_xy_tolerance"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
