import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MobileBaseTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mobile_base_tf_publisher")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("robot_x_offset", 0.0)
        self.declare_parameter("robot_y_offset", 0.0)
        self.declare_parameter("robot_z_offset", 0.0)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.robot_x_offset = float(self.get_parameter("robot_x_offset").value)
        self.robot_y_offset = float(self.get_parameter("robot_y_offset").value)
        self.robot_z_offset = float(self.get_parameter("robot_z_offset").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_transform)

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "robot_x_offset":
                self.robot_x_offset = float(parameter.value)
            elif parameter.name == "robot_y_offset":
                self.robot_y_offset = float(parameter.value)
            elif parameter.name == "robot_z_offset":
                self.robot_z_offset = float(parameter.value)
            elif parameter.name == "world_frame":
                self.world_frame = str(parameter.value)
            elif parameter.name == "base_frame":
                self.base_frame = str(parameter.value)
        return SetParametersResult(successful=True)

    def publish_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = float(self.robot_x_offset)
        transform.transform.translation.y = float(self.robot_y_offset)
        transform.transform.translation.z = float(self.robot_z_offset)
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MobileBaseTfPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
