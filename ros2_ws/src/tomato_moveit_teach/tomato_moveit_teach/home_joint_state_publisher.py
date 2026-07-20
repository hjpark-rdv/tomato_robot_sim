import json
import math
import time
from pathlib import Path

from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class HomeJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("home_joint_state_publisher")
        self.declare_parameter(
            "joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
                "tool_bend_joint",
                "tool_gripper_z_joint",
            ],
        )
        self.declare_parameter(
            "joint_positions",
            [
                -1.4379491029641744,
                -1.1907401577003016,
                -2.116742584205267,
                -1.3810410438314462,
                1.5765461198564021,
                -8.504489148035646e-05,
                0.6673960827876584,
                0.035107627975484376,
            ],
        )
        self.declare_parameter("controller_action_name", "/fake_ur5_controller/follow_joint_trajectory")
        self.declare_parameter("output_path", "/root/pybullet_ur_approach/taught_grasps_moveit.json")
        self.declare_parameter("gripper_spin_joint_name", "tool_gripper_z_joint")
        self.declare_parameter("gripper_z_rot_deg", 2.0115189117106738)
        self.declare_parameter("min_gripper_z_rot_deg", -180.0)
        self.declare_parameter("max_gripper_z_rot_deg", 180.0)
        self.joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        self.joint_positions = [float(pos) for pos in self.get_parameter("joint_positions").value]
        self.position_by_joint = dict(zip(self.joint_names, self.joint_positions))
        self.gripper_spin_joint_name = str(self.get_parameter("gripper_spin_joint_name").value)
        self._set_gripper_spin_deg(float(self.get_parameter("gripper_z_rot_deg").value))
        self.output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.05, self.publish_joint_state)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("controller_action_name").value),
            self.execute_trajectory,
        )
        self.save_service = self.create_service(Trigger, "save_current_joints", self.save_current_joints)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self.get_logger().info("Publishing UR5/tool joint_states and accepting fake FollowJointTrajectory goals")

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "gripper_z_rot_deg":
                try:
                    self._set_gripper_spin_deg(float(parameter.value))
                except (TypeError, ValueError) as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
        self.publish_joint_state()
        return SetParametersResult(successful=True)

    def _set_gripper_spin_deg(self, angle_deg: float) -> None:
        if self.gripper_spin_joint_name not in self.position_by_joint:
            return
        min_angle_deg = float(self.get_parameter("min_gripper_z_rot_deg").value)
        max_angle_deg = float(self.get_parameter("max_gripper_z_rot_deg").value)
        if min_angle_deg > max_angle_deg:
            min_angle_deg, max_angle_deg = max_angle_deg, min_angle_deg
        angle_deg = min(max(float(angle_deg), min_angle_deg), max_angle_deg)
        self.position_by_joint[self.gripper_spin_joint_name] = math.radians(angle_deg)
        self.get_logger().info(
            f"{self.gripper_spin_joint_name} set to {angle_deg:.2f}deg "
            f"({self.position_by_joint[self.gripper_spin_joint_name]:.4f}rad)"
        )

    def publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.position_by_joint[name] for name in self.joint_names]
        self.publisher.publish(msg)

    def execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()
        if not trajectory.joint_names or not trajectory.points:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Trajectory has no joints or points."
            goal_handle.abort()
            return result

        unknown_joints = [name for name in trajectory.joint_names if name not in self.position_by_joint]
        if unknown_joints:
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"Unknown joints: {unknown_joints}"
            goal_handle.abort()
            return result

        self.get_logger().info(f"Executing fake trajectory with {len(trajectory.points)} points")
        previous_time = 0.0
        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Trajectory canceled."
                return result

            point_time = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            sleep_time = max(0.0, point_time - previous_time)
            previous_time = point_time
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            for joint_name, position in zip(trajectory.joint_names, point.positions):
                self.position_by_joint[joint_name] = float(position)
            self.publish_joint_state()
            self._publish_feedback(goal_handle, trajectory.joint_names, point)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Fake trajectory execution complete."
        return result

    def _publish_feedback(self, goal_handle, joint_names, desired_point) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(joint_names)
        feedback.desired = desired_point
        feedback.actual.positions = [self.position_by_joint[name] for name in joint_names]
        feedback.error.positions = [
            desired - actual for desired, actual in zip(feedback.desired.positions, feedback.actual.positions)
        ]
        goal_handle.publish_feedback(feedback)

    def save_current_joints(self, _request, response):
        now = self.get_clock().now().to_msg()
        positions_rad = {name: float(self.position_by_joint[name]) for name in self.joint_names}
        payload = {
            "source": "moveit_current_joint_state",
            "stamp": {
                "sec": int(now.sec),
                "nanosec": int(now.nanosec),
            },
            "planning_group": "ur5_harvest",
            "joint_names": list(self.joint_names),
            "joint_positions_rad": positions_rad,
            "joint_positions_deg": {name: math.degrees(value) for name, value in positions_rad.items()},
        }
        self._append_json(payload)
        response.success = True
        response.message = f"Saved current joint state to {self.output_path}"
        self.get_logger().info(response.message)
        return response

    def _append_json(self, payload: dict) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        if self.output_path.exists():
            try:
                data = json.loads(self.output_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                data = []
        else:
            data = []
        if not isinstance(data, list):
            data = [data]
        data.append(payload)
        self.output_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    rclpy.init()
    node = HomeJointStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
