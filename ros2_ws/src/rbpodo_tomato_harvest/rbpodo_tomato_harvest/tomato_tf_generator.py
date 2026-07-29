import math

import numpy as np
import rclpy
from farmily_tomato_interfaces.msg import TomatoDetectionArray
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _unit(vector: np.ndarray, label: str) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError(f"{label} vector is too short")
    return vector / norm


def parent_frame_tomato_rotation(
    sky_axis_in_parent,
    stem_direction_in_parent,
) -> np.ndarray:
    """Return parent<-tomato rotation with Z skyward and X stemward."""
    z_axis = _unit(np.asarray(sky_axis_in_parent, dtype=float), "sky axis")
    stem_direction = np.asarray(stem_direction_in_parent, dtype=float)
    horizontal_stem = stem_direction - np.dot(stem_direction, z_axis) * z_axis
    x_axis = _unit(horizontal_stem, "horizontal stem direction")
    y_axis = _unit(np.cross(z_axis, x_axis), "tomato Y axis")
    return np.column_stack((x_axis, y_axis, z_axis))


def quaternion_from_rotation(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.array([qx, qy, qz, qw], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def rotation_from_quaternion(quaternion) -> np.ndarray:
    x, y, z, w = [float(value) for value in quaternion]
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


class TomatoTfGenerator(Node):
    """Snapshot camera detections as persistent, base-parented tomato TFs."""

    def __init__(self) -> None:
        super().__init__("tomato_tf_generator")
        self.declare_parameter("camera_frame", "d435_color_optical_frame")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("sky_frame", "link0")
        self.declare_parameter("center_topic", "/tomato_detection/center")
        self.declare_parameter("stem_point_topic", "/tomato_detection/stem_point")
        self.declare_parameter(
            "detections_topic", "/tomato_detection/detections"
        )
        self.declare_parameter("tf_prefix", "detected_tomato_")
        self.declare_parameter("start_index", 0)
        self.declare_parameter("auto_create_on_detection", True)
        self.declare_parameter("maximum_detection_age_sec", 2.0)
        self.declare_parameter("transform_timeout_sec", 2.0)
        self.declare_parameter("broadcast_rate_hz", 20.0)

        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.sky_frame = str(self.get_parameter("sky_frame").value)
        self.tf_prefix = str(self.get_parameter("tf_prefix").value)
        self.start_index = int(self.get_parameter("start_index").value)
        self.next_index = self.start_index
        self.center: PointStamped | None = None
        self.stem_point: PointStamped | None = None
        self.center_updated = False
        self.stem_point_updated = False
        self.transforms: dict[str, TransformStamped] = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("center_topic").value),
            self._center_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("stem_point_topic").value),
            self._stem_point_callback,
            10,
        )
        self.create_subscription(
            TomatoDetectionArray,
            str(self.get_parameter("detections_topic").value),
            self._detections_callback,
            10,
        )
        self.create_service(Trigger, "~/create_tf", self._create_tf)
        period = 1.0 / max(1.0, float(self.get_parameter("broadcast_rate_hz").value))
        self.create_timer(period, self._broadcast_transforms)
        self.get_logger().info(
            "Ready to create parent-frame-fixed tomato TFs: "
            f"service=/{self.get_name()}/create_tf input={self.camera_frame} "
            f"parent={self.parent_frame} sky={self.sky_frame} auto_create="
            f"{self.get_parameter('auto_create_on_detection').value}"
        )

    def _center_callback(self, message: PointStamped) -> None:
        self.center = message
        self.center_updated = True
        self._try_auto_create()

    def _stem_point_callback(self, message: PointStamped) -> None:
        self.stem_point = message
        self.stem_point_updated = True
        self._try_auto_create()

    def _try_auto_create(self) -> None:
        if not bool(self.get_parameter("auto_create_on_detection").value):
            return
        if not self.center_updated or not self.stem_point_updated:
            return

        self._clear_detected_transforms()
        success, message = self._store_current_detection()
        if success:
            self.center_updated = False
            self.stem_point_updated = False
            self.get_logger().info(message)
        else:
            self.get_logger().warning(f"토마토 TF 자동 생성 대기: {message}")

    def _detections_callback(self, message: TomatoDetectionArray) -> None:
        if not bool(self.get_parameter("auto_create_on_detection").value):
            return
        self._clear_detected_transforms()
        if not message.detections:
            self.get_logger().warning(
                "카메라 검출 결과가 비어 있어 기존 토마토 TF를 제거했습니다."
            )
            return

        created = 0
        failures = []
        for detection in message.detections:
            center = PointStamped()
            center.header = message.header
            center.point = detection.center
            stem_point = PointStamped()
            stem_point.header = message.header
            stem_point.point = detection.stem_point
            success, result = self._store_detection(center, stem_point)
            if success:
                created += 1
            else:
                failures.append(f"{detection.id}: {result}")

        self.get_logger().info(
            f"카메라 검출 {len(message.detections)}개 중 TF {created}개 생성 완료"
        )
        for failure in failures:
            self.get_logger().warning(f"토마토 TF 생성 실패: {failure}")

    def _clear_detected_transforms(self) -> None:
        self.transforms.clear()
        self.next_index = self.start_index

    def _message_is_fresh(self, message: PointStamped) -> bool:
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds == 0:
            return True
        maximum_age = max(
            0.0, float(self.get_parameter("maximum_detection_age_sec").value)
        )
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return -0.1 <= age <= maximum_age

    def _lookup_transform(self, target_frame: str, source_frame: str):
        timeout = max(
            0.1, float(self.get_parameter("transform_timeout_sec").value)
        )
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=timeout),
            )
        except TransformException as error:
            raise RuntimeError(
                f"TF lookup failed: {target_frame} <- {source_frame}: {error}"
            ) from error

    def _point_in_parent(self, message: PointStamped) -> np.ndarray:
        point = np.array(
            [message.point.x, message.point.y, message.point.z], dtype=float
        )
        source_frame = str(message.header.frame_id) or self.camera_frame
        if source_frame == self.parent_frame:
            return point
        transform = self._lookup_transform(self.parent_frame, source_frame)
        rotation = rotation_from_quaternion(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        )
        translation = transform.transform.translation
        return rotation @ point + np.array(
            [translation.x, translation.y, translation.z], dtype=float
        )

    def _sky_axis_in_parent(self) -> np.ndarray:
        if self.sky_frame == self.parent_frame:
            return np.array([0.0, 0.0, 1.0], dtype=float)
        transform = self._lookup_transform(self.parent_frame, self.sky_frame)
        rotation = rotation_from_quaternion(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        )
        return rotation @ np.array([0.0, 0.0, 1.0], dtype=float)

    def _store_current_detection(self) -> tuple[bool, str]:
        if self.center is None or self.stem_point is None:
            return False, "토마토 중심과 줄기 점 검출값이 모두 필요합니다."
        return self._store_detection(self.center, self.stem_point)

    def _store_detection(
        self,
        center_message: PointStamped,
        stem_point_message: PointStamped,
    ) -> tuple[bool, str]:
        if not self._message_is_fresh(center_message) or not self._message_is_fresh(
            stem_point_message
        ):
            return False, "검출값이 오래되었습니다. 카메라 검출을 다시 수행하세요."

        try:
            center = self._point_in_parent(center_message)
            stem_point = self._point_in_parent(stem_point_message)
            rotation = parent_frame_tomato_rotation(
                self._sky_axis_in_parent(), stem_point - center
            )
        except (RuntimeError, ValueError) as error:
            return False, str(error)

        child_frame = f"{self.tf_prefix}{self.next_index}_tf"
        transform = TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(center[0])
        transform.transform.translation.y = float(center[1])
        transform.transform.translation.z = float(center[2])
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = quaternion_from_rotation(rotation)
        self.transforms[child_frame] = transform
        self.next_index += 1
        self._broadcast_transforms()

        message = (
            f"{child_frame} 생성 완료: parent={self.parent_frame}, "
            f"center={center.round(4).tolist()}"
        )
        return True, message

    def _create_tf(self, _request: Trigger.Request, response: Trigger.Response):
        self._clear_detected_transforms()
        response.success, response.message = self._store_current_detection()
        if response.success:
            self.center_updated = False
            self.stem_point_updated = False
            self.get_logger().info(response.message)
        return response

    def _broadcast_transforms(self) -> None:
        if not self.transforms:
            return
        stamp = self.get_clock().now().to_msg()
        for transform in self.transforms.values():
            transform.header.stamp = stamp
        self.tf_broadcaster.sendTransform(list(self.transforms.values()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TomatoTfGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
