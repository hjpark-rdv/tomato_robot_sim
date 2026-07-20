import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from tomato_moveit_teach.geometry import quaternion_from_matrix


class PlannedJointSaver(Node):
    def __init__(self) -> None:
        super().__init__("planned_joint_saver")
        self.declare_parameter("output_path", "/root/pybullet_ur_approach/taught_grasps_moveit.json")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("tool_frame", "grasp_tf")
        self.declare_parameter(
            "robot_joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        self.declare_parameter(
            "gripper_joint_names",
            ["tool_bend_joint", "tool_gripper_z_joint"],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )

        self.output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.tool_frame = str(self.get_parameter("tool_frame").value)
        self.robot_description = str(self.get_parameter("robot_description").value)
        self.robot_joint_names = [str(name) for name in self.get_parameter("robot_joint_names").value]
        self.gripper_joint_names = [str(name) for name in self.get_parameter("gripper_joint_names").value]
        self.required_joint_names = self.robot_joint_names + self.gripper_joint_names
        self.chain = self._build_chain(self.reference_frame, self.tool_frame)
        self.latest_plan = None
        self.latest_joint_positions: dict[str, float] = {}

        self.subscription = self.create_subscription(
            DisplayTrajectory,
            "display_planned_path",
            self.display_planned_path_callback,
            10,
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            10,
        )
        self.save_service = self.create_service(Trigger, "save_taught_grasp", self.save_planned_joints)
        self.legacy_plan_service = self.create_service(Trigger, "save_planned_joints", self.save_planned_joints)
        self.legacy_teach_service = self.create_service(Trigger, "save_teached_grasp", self.save_planned_joints)
        self.get_logger().info("Press Plan in RViz, then call /save_taught_grasp")

    def display_planned_path_callback(self, msg: DisplayTrajectory) -> None:
        if not msg.trajectory:
            return
        trajectory = msg.trajectory[-1].joint_trajectory
        if not trajectory.joint_names or not trajectory.points:
            return

        final_point = trajectory.points[-1]
        if len(final_point.positions) != len(trajectory.joint_names):
            return

        self.latest_plan = {
            "stamp": {
                "sec": int(trajectory.header.stamp.sec),
                "nanosec": int(trajectory.header.stamp.nanosec),
            },
            "joint_names": list(trajectory.joint_names),
            "joint_positions": [float(position) for position in final_point.positions],
        }
        self.get_logger().info("Latest planned goal joints are ready to save")

    def joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.latest_joint_positions[str(name)] = float(position)

    def save_planned_joints(self, _request, response):
        if self.latest_plan is None:
            response.success = False
            response.message = "No planned trajectory received yet. Press Plan in RViz first."
            return response

        joint_positions = dict(zip(self.latest_plan["joint_names"], self.latest_plan["joint_positions"]))
        for name in self.gripper_joint_names:
            if name not in joint_positions and name in self.latest_joint_positions:
                joint_positions[name] = self.latest_joint_positions[name]

        missing_joint_names = [name for name in self.required_joint_names if name not in joint_positions]
        if missing_joint_names:
            response.success = False
            response.message = f"Planned trajectory is missing joints: {missing_joint_names}"
            return response

        ee_pose = self._forward_kinematics(joint_positions)
        payload = {
            "source": "moveit_taught_grasp",
            "reference_frame": self.reference_frame,
            "tool_frame": self.tool_frame,
            "ee_pose": ee_pose,
            "robot_joint_positions_rad": {name: float(joint_positions[name]) for name in self.robot_joint_names},
            "gripper_joint_positions_rad": {name: float(joint_positions[name]) for name in self.gripper_joint_names},
        }
        self._append_json(payload)
        response.success = True
        response.message = f"Saved planned grasp pose and joints to {self.output_path}"
        self.get_logger().info(response.message)
        return response

    def _build_chain(self, root_link: str, tip_link: str) -> list[dict]:
        if not self.robot_description:
            raise RuntimeError("robot_description parameter is empty; cannot compute end effector pose")

        robot = ET.fromstring(self.robot_description)
        joints_by_child = {}
        for joint in robot.findall("joint"):
            child = joint.find("child")
            parent = joint.find("parent")
            if child is None or parent is None:
                continue
            joints_by_child[child.attrib["link"]] = {
                "name": joint.attrib["name"],
                "type": joint.attrib.get("type", "fixed"),
                "parent": parent.attrib["link"],
                "child": child.attrib["link"],
                "origin": joint.find("origin"),
                "axis": joint.find("axis"),
            }

        chain = []
        link = tip_link
        while link != root_link:
            if link not in joints_by_child:
                raise RuntimeError(f"Could not find URDF chain from {root_link} to {tip_link}; stopped at {link}")
            joint = joints_by_child[link]
            chain.append(joint)
            link = joint["parent"]
        chain.reverse()
        return chain

    def _forward_kinematics(self, joint_positions: dict[str, float]) -> dict:
        transform = np.eye(4)
        for joint in self.chain:
            transform = transform @ self._origin_matrix(joint["origin"])
            if joint["type"] in {"revolute", "continuous"}:
                transform = transform @ self._axis_angle_matrix(
                    self._axis_vector(joint["axis"]),
                    float(joint_positions.get(joint["name"], 0.0)),
                )

        qx, qy, qz, qw = quaternion_from_matrix(transform[:3, :3])
        return {
            "position": {
                "x": float(transform[0, 3]),
                "y": float(transform[1, 3]),
                "z": float(transform[2, 3]),
            },
            "orientation_xyzw": {
                "x": qx,
                "y": qy,
                "z": qz,
                "w": qw,
            },
        }

    def _origin_matrix(self, origin) -> np.ndarray:
        if origin is None:
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
        else:
            xyz = [float(value) for value in origin.attrib.get("xyz", "0 0 0").split()]
            rpy = [float(value) for value in origin.attrib.get("rpy", "0 0 0").split()]

        transform = np.eye(4)
        transform[:3, :3] = self._rpy_matrix(*rpy)
        transform[:3, 3] = xyz
        return transform

    def _axis_vector(self, axis) -> np.ndarray:
        if axis is None:
            return np.array([0.0, 0.0, 1.0], dtype=float)
        vector = np.array([float(value) for value in axis.attrib.get("xyz", "0 0 1").split()], dtype=float)
        norm = np.linalg.norm(vector)
        if norm < 1e-9:
            return np.array([0.0, 0.0, 1.0], dtype=float)
        return vector / norm

    def _rpy_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        return rz @ ry @ rx

    def _axis_angle_matrix(self, axis: np.ndarray, angle: float) -> np.ndarray:
        x, y, z = axis
        c = np.cos(angle)
        s = np.sin(angle)
        one_c = 1.0 - c
        rotation = np.array(
            [
                [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
                [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
                [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
            ],
            dtype=float,
        )
        transform = np.eye(4)
        transform[:3, :3] = rotation
        return transform

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
    node = PlannedJointSaver()
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
