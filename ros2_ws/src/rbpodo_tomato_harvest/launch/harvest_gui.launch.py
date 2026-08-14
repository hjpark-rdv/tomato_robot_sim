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
                "real_camera_service",
                default_value="/detect_tomatoes",
            ),
            DeclareLaunchArgument(
                "default_camera_source",
                default_value="real",
                description="Initial GUI camera source: fake or real",
            ),
            DeclareLaunchArgument(
                "camera_color_image_topic",
                default_value="/tomato_vision/result_image",
                description="Compressed image topic shown in Camera Color Raw tab",
            ),
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/tomato_detection/detections",
            ),
            DeclareLaunchArgument(
                "scene_node",
                default_value="/tomato_scene_node",
            ),
            Node(
                package="rbpodo_tomato_harvest",
                executable="harvest_gui",
                name="tomato_harvest_gui",
                output="screen",
                parameters=[
                    {
                        "camera_service": LaunchConfiguration("camera_service"),
                        "real_camera_service": LaunchConfiguration(
                            "real_camera_service"
                        ),
                        "default_camera_source": LaunchConfiguration(
                            "default_camera_source"
                        ),
                        "camera_color_image_topic": LaunchConfiguration(
                            "camera_color_image_topic"
                        ),
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "scene_node": LaunchConfiguration("scene_node"),
                    }
                ],
            ),
        ]
    )
