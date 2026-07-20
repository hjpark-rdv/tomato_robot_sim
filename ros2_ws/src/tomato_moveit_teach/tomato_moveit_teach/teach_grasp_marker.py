import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


class TeachGraspMarker(Node):
    def __init__(self) -> None:
        super().__init__("teach_grasp_marker")
        self.declare_parameter("reference_frame", "tomato_tf")
        self.declare_parameter("grasp_frame", "taught_grasp_tf")
        self.declare_parameter("marker_name", "grasp_target")
        self.declare_parameter("object_radius", 0.036)
        self.declare_parameter("output_path", "/root/pybullet_ur_approach/taught_grasps_moveit.json")

        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.grasp_frame = str(self.get_parameter("grasp_frame").value)
        self.marker_name = str(self.get_parameter("marker_name").value)
        self.object_radius = float(self.get_parameter("object_radius").value)
        self.output_path = Path(str(self.get_parameter("output_path").value)).expanduser()

        self.current_pose = Pose()
        self.current_pose.position.x = self.object_radius * 0.45
        self.current_pose.orientation.w = 1.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.server = InteractiveMarkerServer(self, "teach_grasp")
        self._insert_marker()
        self.save_service = self.create_service(Trigger, "save_teach_marker_pose", self.save_callback)
        self.timer = self.create_timer(0.05, self.publish_tf)
        self.get_logger().info(
            f"Drag /teach_grasp in RViz, then call /save_teach_marker_pose to save {self.reference_frame} -> {self.grasp_frame}"
        )

    def _insert_marker(self) -> None:
        marker = InteractiveMarker()
        marker.header.frame_id = self.reference_frame
        marker.name = self.marker_name
        marker.description = "grasp target"
        marker.scale = max(self.object_radius * 3.0, 0.12)
        marker.pose = self.current_pose

        visual = InteractiveMarkerControl()
        visual.always_visible = True
        visual.markers.append(self._sphere_marker())
        marker.controls.append(visual)

        for axis_name, orientation in (
            ("x", (1.0, 1.0, 0.0, 0.0)),
            ("y", (1.0, 0.0, 1.0, 0.0)),
            ("z", (1.0, 0.0, 0.0, 1.0)),
        ):
            marker.controls.append(self._axis_control(f"rotate_{axis_name}", orientation, InteractiveMarkerControl.ROTATE_AXIS))
            marker.controls.append(self._axis_control(f"move_{axis_name}", orientation, InteractiveMarkerControl.MOVE_AXIS))

        self.server.insert(
            marker,
            feedback_callback=self.feedback_callback,
            feedback_type=InteractiveMarkerFeedback.POSE_UPDATE,
        )
        self.server.applyChanges()

    def _sphere_marker(self) -> Marker:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = self.object_radius * 0.45
        marker.scale.y = self.object_radius * 0.45
        marker.scale.z = self.object_radius * 0.45
        marker.color.r = 0.95
        marker.color.g = 0.78
        marker.color.b = 0.05
        marker.color.a = 0.85
        return marker

    def _axis_control(self, name: str, orientation, interaction_mode: int) -> InteractiveMarkerControl:
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = orientation[0]
        control.orientation.x = orientation[1]
        control.orientation.y = orientation[2]
        control.orientation.z = orientation[3]
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = interaction_mode
        return control

    def feedback_callback(self, feedback: InteractiveMarkerFeedback) -> None:
        self.current_pose = feedback.pose
        self.publish_tf()

    def publish_tf(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.reference_frame
        transform.child_frame_id = self.grasp_frame
        transform.transform.translation.x = float(self.current_pose.position.x)
        transform.transform.translation.y = float(self.current_pose.position.y)
        transform.transform.translation.z = float(self.current_pose.position.z)
        transform.transform.rotation = self.current_pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def save_callback(self, _request, response):
        payload = {
            "source": "rviz_interactive_marker",
            "reference_frame": self.reference_frame,
            "tool_frame": self.grasp_frame,
            "transform": {
                "translation": {
                    "x": float(self.current_pose.position.x),
                    "y": float(self.current_pose.position.y),
                    "z": float(self.current_pose.position.z),
                },
                "rotation_xyzw": {
                    "x": float(self.current_pose.orientation.x),
                    "y": float(self.current_pose.orientation.y),
                    "z": float(self.current_pose.orientation.z),
                    "w": float(self.current_pose.orientation.w),
                },
            },
        }
        self._append_json(payload)
        response.success = True
        response.message = f"Saved taught grasp marker pose to {self.output_path}"
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

    def destroy_node(self) -> bool:
        try:
            self.server.shutdown()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TeachGraspMarker()
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
