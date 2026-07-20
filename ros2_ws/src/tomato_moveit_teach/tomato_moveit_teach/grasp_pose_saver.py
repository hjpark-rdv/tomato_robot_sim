import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class GraspPoseSaver(Node):
    def __init__(self) -> None:
        super().__init__("grasp_pose_saver")
        self.declare_parameter("reference_frame", "tomato_tf")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("output_path", "/root/pybullet_ur_approach/taught_grasps_moveit.json")

        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.tool_frame = str(self.get_parameter("tool_frame").value)
        self.output_path = Path(str(self.get_parameter("output_path").value)).expanduser()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.service = self.create_service(Trigger, "save_grasp_pose", self.save_callback)
        self.get_logger().info(
            f"Ready. Call /save_grasp_pose to save {self.reference_frame} -> {self.tool_frame} into {self.output_path}"
        )

    def save_callback(self, _request, response):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.tool_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            response.success = False
            response.message = f"TF lookup failed: {exc}"
            return response

        payload = {
            "stamp": {
                "sec": int(transform.header.stamp.sec),
                "nanosec": int(transform.header.stamp.nanosec),
            },
            "reference_frame": self.reference_frame,
            "tool_frame": self.tool_frame,
            "transform": {
                "translation": {
                    "x": float(transform.transform.translation.x),
                    "y": float(transform.transform.translation.y),
                    "z": float(transform.transform.translation.z),
                },
                "rotation_xyzw": {
                    "x": float(transform.transform.rotation.x),
                    "y": float(transform.transform.rotation.y),
                    "z": float(transform.transform.rotation.z),
                    "w": float(transform.transform.rotation.w),
                },
            },
        }
        self._append_json(payload)
        response.success = True
        response.message = f"Saved grasp pose to {self.output_path}"
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
    node = GraspPoseSaver()
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
