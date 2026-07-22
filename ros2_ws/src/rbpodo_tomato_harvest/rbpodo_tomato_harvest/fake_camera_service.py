import rclpy
from farmily_tomato_interfaces.msg import TomatoDetection, TomatoDetectionArray
from farmily_tomato_interfaces.srv import DetectTomatoes
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class FakeCameraService(Node):
    """Publish simulated camera detections from the procedural tomato TFs."""

    def __init__(self) -> None:
        super().__init__("fake_tomato_camera")
        self.declare_parameter("camera_frame", "d435_color_optical_frame")
        self.declare_parameter(
            "tomato_frames",
            [f"tomato_{index}_tf" for index in range(8)],
        )
        self.declare_parameter("stem_frame", "main_vine_tf")
        self.declare_parameter(
            "detections_topic", "/tomato_detection/detections"
        )
        self.declare_parameter("transform_timeout_sec", 2.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detections_publisher = self.create_publisher(
            TomatoDetectionArray,
            str(self.get_parameter("detections_topic").value),
            10,
        )
        self.create_service(
            DetectTomatoes,
            "~/detect_tomatoes",
            self._detect_tomatoes,
        )
        self.get_logger().info(
            "Fake tomato camera ready: "
            "service=/fake_tomato_camera/detect_tomatoes"
        )

    def _lookup_point(self, source_frame: str) -> Point:
        camera_frame = str(self.get_parameter("camera_frame").value)
        timeout = max(
            0.1, float(self.get_parameter("transform_timeout_sec").value)
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=timeout),
            )
        except TransformException as error:
            raise RuntimeError(
                f"TF lookup failed: {camera_frame} <- {source_frame}: {error}"
            ) from error

        translation = transform.transform.translation
        return Point(x=translation.x, y=translation.y, z=translation.z)

    def _detect_tomatoes(
        self,
        _request: DetectTomatoes.Request,
        response: DetectTomatoes.Response,
    ):
        tomato_frames = [
            str(frame) for frame in self.get_parameter("tomato_frames").value
        ]
        stem_frame = str(self.get_parameter("stem_frame").value)
        detections = TomatoDetectionArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = str(self.get_parameter("camera_frame").value)

        try:
            stem_point = self._lookup_point(stem_frame)
            for tomato_frame in tomato_frames:
                center = self._lookup_point(tomato_frame)
                detection = TomatoDetection()
                detection.id = tomato_frame.removesuffix("_tf")
                detection.center = center
                detection.stem_point = stem_point
                detections.detections.append(detection)
        except RuntimeError as error:
            response.success = False
            response.message = str(error)
            return response

        self.detections_publisher.publish(detections)
        response.success = True
        response.message = (
            f"토마토 {len(detections.detections)}개 검출 결과 발행 완료: "
            f"camera={detections.header.frame_id}"
        )
        response.detections = detections
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeCameraService()
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
