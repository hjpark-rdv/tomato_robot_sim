"""ROS 2 bridge for Arduino Mega digital outputs and a pin 10 servo."""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

try:
    import serial
    from serial import SerialException
except ImportError:  # Allows ROS package discovery even before pyserial is installed.
    serial = None
    SerialException = Exception


class Pin89SerialNode(Node):
    """Forward digital output and servo angle topics to Arduino."""

    def __init__(self):
        super().__init__('pin89_serial_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('reconnect_period_s', 2.0)
        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._serial = None
        self._lock = threading.Lock()

        self.create_subscription(Bool, '/linear_motor/pin8', self._pin8_callback, 10)
        self.create_subscription(Bool, '/linear_motor/pin9', self._pin9_callback, 10)
        self.create_subscription(
            Float64,
            '/linear_motor/servo10_angle_deg',
            self._servo10_angle_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            '/linear_motor/servo10_command',
            self._servo10_command_callback,
            10,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String,
            '/linear_motor/serial_status',
            status_qos,
        )
        self.create_timer(self.get_parameter('reconnect_period_s').value, self._connect)
        self._connect()

    def _connect(self):
        if self._serial is not None and self._serial.is_open:
            return
        if serial is None:
            self.get_logger().error('pyserial is missing. Install: sudo apt install python3-serial')
            return
        try:
            self._serial = serial.Serial(self._port, self._baudrate, timeout=0.2)
            self.get_logger().info(f'Connected to Arduino at {self._port} ({self._baudrate} baud)')
            self._publish_status('connected')
        except SerialException as exc:
            self._serial = None
            self._publish_status(f'disconnected: {exc}')

    def _send_command(self, command):
        wire_command = f'{command}\n'
        with self._lock:
            if self._serial is None or not self._serial.is_open:
                self.get_logger().warning(
                    f'Cannot send {command}: serial is disconnected'
                )
                return
            try:
                self._serial.write(wire_command.encode('ascii'))
                self._serial.flush()
            except SerialException as exc:
                self.get_logger().error(f'Serial write failed: {exc}')
                self._publish_status(f'disconnected: {exc}')
                try:
                    self._serial.close()
                except SerialException:
                    pass
                self._serial = None

    def _pin8_callback(self, message):
        self._send_command(f'PIN 8 {1 if message.data else 0}')

    def _pin9_callback(self, message):
        self._send_command(f'PIN 9 {1 if message.data else 0}')

    def _servo10_angle_callback(self, message):
        angle = float(message.data)
        if not math.isfinite(angle) or angle < 10.0 or angle > 173.0:
            self.get_logger().error(
                f'Rejected servo angle {angle}: expected 10..173 degrees'
            )
            return
        angle_deg = int(round(angle))
        self._send_command(f'ANGLE 10 {angle_deg}')
        self.get_logger().info(f'Servo pin 10 angle command: {angle_deg} deg')

    def _servo10_command_callback(self, message):
        if len(message.data) != 2:
            self.get_logger().error(
                'Rejected servo command: expected [angle_deg, speed_percent]'
            )
            return
        angle = float(message.data[0])
        speed_percent = float(message.data[1])
        if not math.isfinite(angle) or angle < 10.0 or angle > 173.0:
            self.get_logger().error(
                f'Rejected servo angle {angle}: expected 10..173 degrees'
            )
            return
        if (
            not math.isfinite(speed_percent)
            or speed_percent < 1.0
            or speed_percent > 100.0
        ):
            self.get_logger().error(
                f'Rejected servo speed {speed_percent}: expected 1..100 percent'
            )
            return
        angle_deg = int(round(angle))
        speed = int(round(speed_percent))
        self._send_command(f'ANGLE 10 {angle_deg} {speed}')
        self.get_logger().info(
            f'Servo pin 10 command: {angle_deg} deg, {speed}% speed'
        )

    def _publish_status(self, text):
        message = String()
        message.data = text
        self._status_pub.publish(message)

    def destroy_node(self):
        if self._serial is not None:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Pin89SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
