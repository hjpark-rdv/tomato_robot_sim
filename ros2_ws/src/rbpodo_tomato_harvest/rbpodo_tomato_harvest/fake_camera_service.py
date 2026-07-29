import math

import rclpy
from farmily_tomato_interfaces.msg import (
    TomatoDetection,
    TomatoDetectionArray,
)
from farmily_tomato_interfaces.srv import DetectTomatoes
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def vertical_harvest_order(points, xy_tolerance: float = 0.05) -> list[int]:
    """Order points by vertical columns, starting at the global highest point.

    Points within ``xy_tolerance`` of a column center share that column. Each
    column is harvested from high Z to low Z. After the first/highest column,
    the next column is the one requiring the least horizontal travel from the
    previous column's bottom point.
    """
    tolerance = float(xy_tolerance)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("xy_tolerance must be a finite value of zero or greater")

    coordinates = []
    for point in points:
        if len(point) != 3:
            raise ValueError("every tomato point must contain X, Y and Z")
        coordinate = tuple(float(value) for value in point)
        if not all(math.isfinite(value) for value in coordinate):
            raise ValueError("tomato coordinates must be finite")
        coordinates.append(coordinate)
    if not coordinates:
        return []

    highest_first = sorted(
        range(len(coordinates)),
        key=lambda index: (
            -coordinates[index][2],
            coordinates[index][0],
            coordinates[index][1],
            index,
        ),
    )
    columns = []
    for point_index in highest_first:
        x, y, _ = coordinates[point_index]
        candidates = []
        for column_index, column in enumerate(columns):
            center_x = sum(coordinates[index][0] for index in column) / len(column)
            center_y = sum(coordinates[index][1] for index in column) / len(column)
            distance = math.hypot(x - center_x, y - center_y)
            if distance <= tolerance + 1e-12:
                candidates.append((distance, column_index))
        if candidates:
            _, column_index = min(candidates)
            columns[column_index].append(point_index)
        else:
            columns.append([point_index])

    for column in columns:
        column.sort(
            key=lambda index: (
                -coordinates[index][2],
                coordinates[index][0],
                coordinates[index][1],
                index,
            )
        )

    first_point = highest_first[0]
    first_column_index = next(
        index for index, column in enumerate(columns) if first_point in column
    )
    ordered_columns = [columns.pop(first_column_index)]
    while columns:
        previous_bottom = ordered_columns[-1][-1]
        previous_x, previous_y, _ = coordinates[previous_bottom]

        def transition_key(column):
            next_top = column[0]
            next_x, next_y, next_z = coordinates[next_top]
            return (
                math.hypot(next_x - previous_x, next_y - previous_y),
                -next_z,
                next_x,
                next_y,
                next_top,
            )

        next_column = min(columns, key=transition_key)
        columns.remove(next_column)
        ordered_columns.append(next_column)

    return [index for column in ordered_columns for index in column]


class FakeCameraService(Node):
    """Publish simulated camera detections from the procedural tomato TFs."""

    def __init__(self) -> None:
        super().__init__("fake_tomato_camera")

        self.declare_parameter(
            "camera_frame",
            "d435_color_optical_frame",
        )
        self.declare_parameter(
            "tomato_frames",
            [f"tomato_{index}_tf" for index in range(8)],
        )
        self.declare_parameter(
            "stem_frame",
            "main_vine_tf",
        )
        self.declare_parameter("sorting_frame", "link0")
        self.declare_parameter("vertical_column_xy_tolerance", 0.05)
        self.declare_parameter(
            "detections_topic",
            "/tomato_detection/detections",
        )
        self.declare_parameter(
            "transform_timeout_sec",
            2.0,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

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

    def _lookup_point(self, source_frame: str, target_frame: str | None = None) -> Point:
        """Return the source-frame origin expressed in the target frame."""

        target_frame = target_frame or str(
            self.get_parameter("camera_frame").value
        )
        timeout = max(
            0.1,
            float(
                self.get_parameter(
                    "transform_timeout_sec"
                ).value
            ),
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=timeout),
            )
        except TransformException as error:
            raise RuntimeError(
                f"TF lookup failed: "
                f"{target_frame} <- {source_frame}: {error}"
            ) from error

        translation = transform.transform.translation

        return Point(
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
        )

    def _detect_tomatoes(
        self,
        _request: DetectTomatoes.Request,
        response: DetectTomatoes.Response,
    ):
        tomato_frames = [
            str(frame)
            for frame in self.get_parameter(
                "tomato_frames"
            ).value
        ]
        stem_frame = str(
            self.get_parameter("stem_frame").value
        )
        camera_frame = str(
            self.get_parameter("camera_frame").value
        )
        sorting_frame = str(self.get_parameter("sorting_frame").value)
        column_tolerance = float(
            self.get_parameter("vertical_column_xy_tolerance").value
        )

        detections = TomatoDetectionArray()
        detections.header.stamp = (
            self.get_clock().now().to_msg()
        )
        detections.header.frame_id = camera_frame

        excluded_count = 0
        accepted = []

        try:
            # main_vine_tf의 위치를 카메라 좌표계로 변환
            stem_point = self._lookup_point(stem_frame)

            self.get_logger().info(
                f"Filter reference: {stem_frame}, "
                f"z={stem_point.z:.4f} m "
                f"in {camera_frame}"
            )

            for tomato_frame in tomato_frames:
                center = self._lookup_point(tomato_frame)

                # main_vine_tf보다 Z가 작은 토마토만 검출 결과에 포함
                if center.z >= stem_point.z:
                    excluded_count += 1

                    self.get_logger().info(
                        f"Tomato filtered out: "
                        f"frame={tomato_frame}, "
                        f"tomato_z={center.z:.4f}, "
                        f"stem_z={stem_point.z:.4f}"
                    )
                    continue

                detection = TomatoDetection()
                detection.id = tomato_frame.removesuffix("_tf")
                detection.center = center
                detection.stem_point = stem_point
                sorting_point = self._lookup_point(
                    tomato_frame,
                    sorting_frame,
                )
                accepted.append((detection, tomato_frame, sorting_point))

        except RuntimeError as error:
            response.success = False
            response.message = str(error)
            return response

        try:
            order = vertical_harvest_order(
                [
                    (item[2].x, item[2].y, item[2].z)
                    for item in accepted
                ],
                column_tolerance,
            )
        except ValueError as error:
            response.success = False
            response.message = f"토마토 수확 순서 계산 실패: {error}"
            return response

        for detected_index, accepted_index in enumerate(order):
            detection, tomato_frame, sorting_point = accepted[accepted_index]
            detections.detections.append(detection)
            self.get_logger().info(
                f"Detection order {detected_index}: "
                f"detected_tomato_{detected_index}_tf <- {tomato_frame}, "
                f"{sorting_frame}_xyz=("
                f"{sorting_point.x:.4f}, {sorting_point.y:.4f}, "
                f"{sorting_point.z:.4f})"
            )

        self.detections_publisher.publish(detections)

        response.success = True
        response.message = (
            f"토마토 {len(detections.detections)}개 검출 결과 발행 완료 "
            f"(제외 {excluded_count}개): "
            f"조건=tomato_z < main_vine_z, "
            f"camera={camera_frame}, "
            f"main_vine_z={stem_point.z:.4f}, "
            f"sorting=vertical_columns({sorting_frame}, "
            f"xy_tolerance={column_tolerance:.3f}m)"
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
