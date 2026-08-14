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
                "vision_result_image_topic",
                default_value="/tomato_vision/result_image_raw",
                description="sensor_msgs/Image shown unchanged in Vision Result tab",
            ),
            DeclareLaunchArgument(
                "camera_color_image_topic",
                default_value="/tomato_vision/camera_preview",
                description="sensor_msgs/Image topic shown in Camera Color Raw tab",
            ),
            DeclareLaunchArgument(
                "camera_color_info_topic",
                default_value="/camera/d435/color/camera_info",
                description="CameraInfo used to project detection XYZ onto the raw image",
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
                        "vision_result_image_topic": LaunchConfiguration(
                            "vision_result_image_topic"
                        ),
                        "camera_color_image_topic": LaunchConfiguration(
                            "camera_color_image_topic"
                        ),
                        "camera_color_info_topic": LaunchConfiguration(
                            "camera_color_info_topic"
                        ),
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "scene_node": LaunchConfiguration("scene_node"),
                    }
                ],
            ),
        ]
    )
