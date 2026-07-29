import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64


def height_mm_to_joint_m(
    height_mm: float,
    minimum_m: float = 0.0,
    maximum_m: float = 1.20,
) -> float:
    """Convert the physical Bottom-relative height to a bounded URDF joint."""
    height_m = float(height_mm) / 1000.0
    if not math.isfinite(height_m):
        raise ValueError("lift height must be finite")
    lower = float(minimum_m)
    upper = float(maximum_m)
    if upper < lower:
        raise ValueError("lift maximum height must not be below minimum height")
    return max(lower, min(upper, height_m))


class LiftJointStatePublisher(Node):
    """Publish the real lift height as the RB5 lift prismatic joint state."""

    def __init__(self) -> None:
        super().__init__("farmily_lift_joint_state_publisher")
        self.declare_parameter(
            "height_topic",
            "/lift_status/current_height",
        )
        self.declare_parameter(
            "joint_states_topic",
            "/joint_states",
        )
        self.declare_parameter(
            "simulation_command_topic",
            "/lift_simulation/control/move_height",
        )
        self.declare_parameter(
            "simulation_stop_topic",
            "/lift_simulation/control/stop",
        )
        self.declare_parameter(
            "simulation_height_topic",
            "/lift_simulation/status/current_height",
        )
        self.declare_parameter(
            "simulation_mode_topic",
            "/lift_simulation/status/enabled",
        )
        self.declare_parameter(
            "joint_name",
            "farmily_lift_height_joint",
        )
        self.declare_parameter("initial_height_m", 0.0)
        self.declare_parameter("minimum_height_m", 0.0)
        self.declare_parameter("maximum_height_m", 1.20)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("simulate_commands", False)

        self.joint_name = str(self.get_parameter("joint_name").value)
        self.simulate_commands = bool(
            self.get_parameter("simulate_commands").value
        )
        self.minimum_height_m = float(
            self.get_parameter("minimum_height_m").value
        )
        self.maximum_height_m = float(
            self.get_parameter("maximum_height_m").value
        )
        self.current_height_m = height_mm_to_joint_m(
            1000.0 * float(self.get_parameter("initial_height_m").value),
            self.minimum_height_m,
            self.maximum_height_m,
        )
        self.joint_state_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            10,
        )
        self.subscription = self.create_subscription(
            Float64,
            str(self.get_parameter("height_topic").value),
            self._real_height_callback,
            10,
        )
        self.simulation_height_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("simulation_height_topic").value),
            10,
        )
        self.simulation_mode_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("simulation_mode_topic").value),
            10,
        )
        self.simulation_command_subscription = self.create_subscription(
            Float64,
            str(self.get_parameter("simulation_command_topic").value),
            self._simulation_height_command_callback,
            10,
        )
        self.simulation_stop_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter("simulation_stop_topic").value),
            self._simulation_stop_callback,
            10,
        )
        publish_rate_hz = max(
            1.0,
            float(self.get_parameter("publish_rate_hz").value),
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish)
        self.get_logger().info(
            f"Lift joint bridge ready: {self.joint_name}="
            f"{self.current_height_m:.3f} m, "
            f"simulation={self.simulate_commands}"
        )

    def _real_height_callback(self, message: Float64) -> None:
        if self.simulate_commands:
            return
        self._height_callback(message)

    def _height_callback(self, message: Float64) -> None:
        try:
            bounded_height = height_mm_to_joint_m(
                message.data,
                self.minimum_height_m,
                self.maximum_height_m,
            )
        except ValueError as error:
            self.get_logger().error(f"Invalid lift height: {error}")
            return
        raw_height_m = float(message.data) / 1000.0
        if not math.isclose(raw_height_m, bounded_height, abs_tol=1e-9):
            self.get_logger().warning(
                f"Lift height {raw_height_m:.3f} m was clamped to "
                f"{bounded_height:.3f} m"
            )
        self.current_height_m = bounded_height

    def _simulation_height_command_callback(self, message: Float64) -> None:
        if not self.simulate_commands:
            return
        self._height_callback(message)
        self.get_logger().info(
            "Simulated lift height set to "
            f"{self.current_height_m * 1000.0:.2f} mm"
        )

    def _simulation_stop_callback(self, message: Bool) -> None:
        if self.simulate_commands and message.data:
            self.get_logger().info(
                "Simulated lift stop received; current height is retained"
            )

    def _publish(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [self.joint_name]
        message.position = [self.current_height_m]
        self.joint_state_publisher.publish(message)
        simulation_mode = Bool()
        simulation_mode.data = self.simulate_commands
        self.simulation_mode_publisher.publish(simulation_mode)
        if self.simulate_commands:
            simulated_height = Float64()
            simulated_height.data = self.current_height_m * 1000.0
            self.simulation_height_publisher.publish(simulated_height)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiftJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
