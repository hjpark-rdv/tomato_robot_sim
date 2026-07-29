from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    can_interface = 'can2'

    return LaunchDescription([
        Node(
            package='farmily_uv_lift',
            executable='lift_controller_node',
            name='lift_controller_node',
            output='screen',
            parameters=[{'can_interface': can_interface}]
        )
    ])
