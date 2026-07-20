import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def matrix_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def matrix_from_pose(pose: Pose) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = matrix_from_quaternion(
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    matrix[:3, 3] = [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    ]
    return matrix


def matrix_from_transform(transform) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    rotation = transform.rotation
    translation = transform.translation
    matrix[:3, :3] = matrix_from_quaternion(
        float(rotation.x),
        float(rotation.y),
        float(rotation.z),
        float(rotation.w),
    )
    matrix[:3, 3] = [
        float(translation.x),
        float(translation.y),
        float(translation.z),
    ]
    return matrix


class ToolDepthCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("tool_depth_camera_publisher")
        self.declare_parameter("depth_frame_id", "tool_camera_depth_optical_frame")
        self.declare_parameter("color_frame_id", "tool_camera_color_optical_frame")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("horizontal_fov_deg", 87.0)
        self.declare_parameter("vertical_fov_deg", 58.0)
        self.declare_parameter("synthetic_depth_m", 1.20)
        self.declare_parameter("depth_noise_m", 0.006)
        self.declare_parameter("publish_point_cloud", True)
        self.declare_parameter("point_cloud_stride", 4)
        self.declare_parameter("render_marker_scene", True)
        self.declare_parameter("marker_topic", "/tomato_markers")
        self.declare_parameter("max_render_markers", 700)
        self.declare_parameter("marker_render_timeout_sec", 0.005)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_camera_frustum", True)
        self.declare_parameter("camera_frustum_topic", "/tomato_markers")
        self.declare_parameter("camera_frustum_depth_m", 0.80)
        self.declare_parameter("camera_frustum_line_width_m", 0.006)

        self.depth_frame_id = str(self.get_parameter("depth_frame_id").value)
        self.color_frame_id = str(self.get_parameter("color_frame_id").value)
        self.width = max(1, int(self.get_parameter("width").value))
        self.height = max(1, int(self.get_parameter("height").value))
        self.fps = max(0.1, float(self.get_parameter("fps").value))
        self.horizontal_fov_deg = float(self.get_parameter("horizontal_fov_deg").value)
        self.vertical_fov_deg = float(self.get_parameter("vertical_fov_deg").value)
        self.synthetic_depth_m = max(0.05, float(self.get_parameter("synthetic_depth_m").value))
        self.depth_noise_m = max(0.0, float(self.get_parameter("depth_noise_m").value))
        self.publish_point_cloud = bool(self.get_parameter("publish_point_cloud").value)
        self.point_cloud_stride = max(1, int(self.get_parameter("point_cloud_stride").value))
        self.render_marker_scene = bool(self.get_parameter("render_marker_scene").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.max_render_markers = max(1, int(self.get_parameter("max_render_markers").value))
        self.marker_render_timeout_sec = max(
            0.0,
            float(self.get_parameter("marker_render_timeout_sec").value),
        )
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_camera_frustum = bool(self.get_parameter("publish_camera_frustum").value)
        self.camera_frustum_topic = str(self.get_parameter("camera_frustum_topic").value)
        self.camera_frustum_depth_m = max(0.05, float(self.get_parameter("camera_frustum_depth_m").value))
        self.camera_frustum_line_width_m = max(
            0.001,
            float(self.get_parameter("camera_frustum_line_width_m").value),
        )

        self.fx = self.width / (2.0 * math.tan(math.radians(self.horizontal_fov_deg) * 0.5))
        self.fy = self.height / (2.0 * math.tan(math.radians(self.vertical_fov_deg) * 0.5))
        self.cx = (self.width - 1.0) * 0.5
        self.cy = (self.height - 1.0) * 0.5

        self.depth_pub = self.create_publisher(Image, "camera/depth/image_rect_raw", 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, "camera/depth/camera_info", 10)
        self.color_pub = self.create_publisher(Image, "camera/color/image_raw", 10)
        self.color_info_pub = self.create_publisher(CameraInfo, "camera/color/camera_info", 10)
        self.points_pub = self.create_publisher(PointCloud2, "camera/depth/color/points", 5)
        if self.publish_camera_frustum:
            self.frustum_pub = self.create_publisher(MarkerArray, self.camera_frustum_topic, 10)

        self._depth_mm = self._make_depth_image_mm()
        self._color = self._make_color_image()
        self._markers: dict[tuple[str, int], Marker] = {}
        self._last_tf_warn_sec = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        if self.render_marker_scene:
            self.marker_sub = self.create_subscription(
                MarkerArray,
                self.marker_topic,
                self._markers_callback,
                10,
            )

        self.timer = self.create_timer(1.0 / self.fps, self.publish_camera)
        self.get_logger().info(
            "Publishing D435-like tool depth camera topics: "
            f"{self.width}x{self.height}@{self.fps:.1f}Hz, "
            f"depth_frame={self.depth_frame_id}, color_frame={self.color_frame_id}, "
            f"marker_render={'on' if self.render_marker_scene else 'off'}, "
            f"frustum_marker={'on' if self.publish_camera_frustum else 'off'}"
        )

    def _make_depth_image_mm(self) -> np.ndarray:
        y = np.linspace(-1.0, 1.0, self.height, dtype=np.float32)[:, None]
        x = np.linspace(-1.0, 1.0, self.width, dtype=np.float32)[None, :]
        ripple = 0.5 * np.sin(3.0 * x) * np.cos(2.0 * y)
        depth_m = self.synthetic_depth_m + self.depth_noise_m * ripple
        depth_m = np.maximum(depth_m, 0.05)
        return np.round(depth_m * 1000.0).astype("<u2")

    def _make_color_image(self) -> np.ndarray:
        x = np.linspace(35, 110, self.width, dtype=np.uint8)[None, :]
        y = np.linspace(25, 80, self.height, dtype=np.uint8)[:, None]
        red = np.broadcast_to(x, (self.height, self.width))
        green = np.broadcast_to(y, (self.height, self.width))
        blue = np.full((self.height, self.width), 130, dtype=np.uint8)
        return np.dstack((red, green, blue)).astype(np.uint8)

    def _make_point_cloud_xyz(self, depth_mm: np.ndarray) -> np.ndarray:
        depth = depth_mm.astype(np.float32) * 0.001
        stride = self.point_cloud_stride
        v_coords, u_coords = np.mgrid[0:self.height:stride, 0:self.width:stride].astype(np.float32)
        z = depth[::stride, ::stride]
        valid = z > 0.0
        x = (u_coords - self.cx) * z / self.fx
        y = (v_coords - self.cy) * z / self.fy
        return np.column_stack((x[valid], y[valid], z[valid])).astype(np.float32)

    def _markers_callback(self, msg: MarkerArray) -> None:
        for marker in msg.markers:
            key = (str(marker.ns), int(marker.id))
            if marker.action == Marker.DELETEALL:
                self._markers.clear()
            elif marker.action == Marker.DELETE:
                self._markers.pop(key, None)
            elif marker.action == Marker.ADD:
                self._markers[key] = marker

    def _lookup_camera_from_frame(self, frame_id: str) -> np.ndarray | None:
        lookup_time = Time(seconds=0, nanoseconds=0)
        timeout = Duration(seconds=self.marker_render_timeout_sec)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.depth_frame_id,
                frame_id,
                lookup_time,
                timeout=timeout,
            )
            return matrix_from_transform(transform.transform)
        except TransformException as exc:
            if frame_id == self.world_frame:
                camera_from_world = self._lookup_camera_from_world_via_base(lookup_time, timeout)
                if camera_from_world is not None:
                    return camera_from_world
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec - self._last_tf_warn_sec > 2.0:
                self.get_logger().warn(
                    f"Could not render camera marker scene yet: {frame_id} -> {self.depth_frame_id}: {exc}"
                )
                self._last_tf_warn_sec = now_sec
            return None

    def _lookup_camera_from_world_via_base(self, lookup_time: Time, timeout: Duration) -> np.ndarray | None:
        try:
            camera_from_base = self.tf_buffer.lookup_transform(
                self.depth_frame_id,
                self.base_frame,
                lookup_time,
                timeout=timeout,
            )
            base_from_world = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.world_frame,
                lookup_time,
                timeout=timeout,
            )
            return matrix_from_transform(camera_from_base.transform) @ matrix_from_transform(base_from_world.transform)
        except TransformException:
            return None

    def _marker_rgb(self, marker: Marker) -> np.ndarray:
        return np.array(
            [
                int(np.clip(float(marker.color.r), 0.0, 1.0) * 255.0),
                int(np.clip(float(marker.color.g), 0.0, 1.0) * 255.0),
                int(np.clip(float(marker.color.b), 0.0, 1.0) * 255.0),
            ],
            dtype=np.uint8,
        )

    def _draw_disc(
        self,
        color_image: np.ndarray,
        depth_m: np.ndarray,
        center_camera: np.ndarray,
        radius_m: float,
        rgb: np.ndarray,
    ) -> None:
        z = float(center_camera[2])
        if z <= 0.02:
            return
        u = int(round(self.cx + self.fx * float(center_camera[0]) / z))
        v = int(round(self.cy + self.fy * float(center_camera[1]) / z))
        pixel_radius = max(1, int(round(self.fx * max(float(radius_m), 0.002) / z)))
        if u + pixel_radius < 0 or u - pixel_radius >= self.width:
            return
        if v + pixel_radius < 0 or v - pixel_radius >= self.height:
            return
        x_min = max(u - pixel_radius, 0)
        x_max = min(u + pixel_radius + 1, self.width)
        y_min = max(v - pixel_radius, 0)
        y_max = min(v + pixel_radius + 1, self.height)
        yy, xx = np.ogrid[y_min:y_max, x_min:x_max]
        mask = (xx - u) ** 2 + (yy - v) ** 2 <= pixel_radius**2
        object_depth = max(z - float(radius_m) * 0.35, 0.001)
        depth_slice = depth_m[y_min:y_max, x_min:x_max]
        update = mask & (object_depth < depth_slice)
        if not np.any(update):
            return
        depth_slice[update] = object_depth
        color_slice = color_image[y_min:y_max, x_min:x_max]
        color_slice[update] = rgb

    def _draw_segment(
        self,
        color_image: np.ndarray,
        depth_m: np.ndarray,
        start_camera: np.ndarray,
        end_camera: np.ndarray,
        radius_m: float,
        rgb: np.ndarray,
    ) -> None:
        segment = end_camera - start_camera
        length = float(np.linalg.norm(segment))
        samples = int(np.clip(length / max(float(radius_m), 0.003) * 1.5, 3, 80))
        for index in range(samples + 1):
            t = index / max(samples, 1)
            point = (1.0 - t) * start_camera + t * end_camera
            self._draw_disc(color_image, depth_m, point, radius_m, rgb)

    def _render_marker(
        self,
        marker: Marker,
        camera_from_frame: np.ndarray,
        color_image: np.ndarray,
        depth_m: np.ndarray,
    ) -> None:
        if float(marker.color.a) <= 0.01:
            return
        rgb = self._marker_rgb(marker)
        marker_from_local = matrix_from_pose(marker.pose)
        camera_from_marker = camera_from_frame @ marker_from_local
        center = camera_from_marker[:3, 3]

        if marker.type == Marker.SPHERE:
            radius = max(float(marker.scale.x), float(marker.scale.y), float(marker.scale.z)) * 0.5
            self._draw_disc(color_image, depth_m, center, radius, rgb)
            return

        if marker.type == Marker.CYLINDER:
            radius = max(float(marker.scale.x), float(marker.scale.y)) * 0.5
            half_length = float(marker.scale.z) * 0.5
            axis = camera_from_marker[:3, 2]
            self._draw_segment(color_image, depth_m, center - axis * half_length, center + axis * half_length, radius, rgb)
            return

        if marker.type == Marker.LINE_STRIP:
            if len(marker.points) < 2:
                return
            radius = max(float(marker.scale.x), 0.002)
            points = []
            for point in marker.points:
                local = np.array([float(point.x), float(point.y), float(point.z), 1.0], dtype=float)
                points.append((camera_from_frame @ local)[:3])
            for start, end in zip(points, points[1:]):
                self._draw_segment(color_image, depth_m, start, end, radius, rgb)
            return

        if marker.type == Marker.CUBE:
            radius = max(float(marker.scale.x), float(marker.scale.y), float(marker.scale.z)) * 0.5
            self._draw_disc(color_image, depth_m, center, radius, rgb)

    def _should_render_camera_marker(self, marker: Marker) -> bool:
        ns = str(marker.ns)
        if ns in {
            "main_vine",
            "vine_row_main_vine",
            "target_tomato",
            "other_tomato",
            "vine_row_tomato",
            "target_branch",
            "other_branch",
            "vine_row_branch",
            "target_calyx_stem",
            "other_calyx_stem",
            "vine_row_calyx_stem",
            "target_calyx_leaf",
            "other_calyx_leaf",
            "vine_row_calyx_leaf",
            "tomato_positive_x_surface",
        }:
            return True
        return False

    def _render_marker_scene(self, color_image: np.ndarray, depth_mm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if not self.render_marker_scene or not self._markers:
            return color_image, depth_mm

        depth_m = depth_mm.astype(np.float32) * 0.001
        frame_transforms: dict[str, np.ndarray | None] = {}
        rendered_count = 0
        markers = [marker for marker in self._markers.values() if self._should_render_camera_marker(marker)]
        # Render nearer/larger fruit before visual helper lines if the marker list is large.
        markers.sort(key=lambda marker: 0 if "tomato" in str(marker.ns) else 1)
        for marker in markers[: self.max_render_markers]:
            frame_id = str(marker.header.frame_id)
            if not frame_id:
                continue
            if frame_id not in frame_transforms:
                camera_from_frame = self._lookup_camera_from_frame(frame_id)
                frame_transforms[frame_id] = camera_from_frame
            camera_from_frame = frame_transforms[frame_id]
            if camera_from_frame is None:
                continue
            self._render_marker(marker, frame_transforms[frame_id], color_image, depth_m)
            rendered_count += 1
        if rendered_count == 0:
            return color_image, depth_mm
        return color_image, np.round(depth_m * 1000.0).astype("<u2")

    def _camera_info(self, frame_id: str, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            self.fx,
            0.0,
            self.cx,
            0.0,
            self.fy,
            self.cy,
            0.0,
            0.0,
            1.0,
        ]
        info.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        info.p = [
            self.fx,
            0.0,
            self.cx,
            0.0,
            0.0,
            self.fy,
            self.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return info

    def _publish_frustum_marker(self, stamp) -> None:
        if not self.publish_camera_frustum:
            return

        depth = self.camera_frustum_depth_m
        half_width = depth * math.tan(math.radians(self.horizontal_fov_deg) * 0.5)
        half_height = depth * math.tan(math.radians(self.vertical_fov_deg) * 0.5)

        origin = Point(x=0.0, y=0.0, z=0.0)
        top_left = Point(x=-half_width, y=-half_height, z=depth)
        top_right = Point(x=half_width, y=-half_height, z=depth)
        bottom_right = Point(x=half_width, y=half_height, z=depth)
        bottom_left = Point(x=-half_width, y=half_height, z=depth)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.depth_frame_id
        marker.ns = "camera_frustum"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.camera_frustum_line_width_m
        marker.color.r = 0.05
        marker.color.g = 0.75
        marker.color.b = 1.0
        marker.color.a = 0.85
        marker.points = [
            origin,
            top_left,
            origin,
            top_right,
            origin,
            bottom_right,
            origin,
            bottom_left,
            top_left,
            top_right,
            top_right,
            bottom_right,
            bottom_right,
            bottom_left,
            bottom_left,
            top_left,
        ]

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.frustum_pub.publish(marker_array)

    def publish_camera(self) -> None:
        stamp = self.get_clock().now().to_msg()
        color_image = self._color.copy()
        depth_mm = self._depth_mm.copy()
        color_image, depth_mm = self._render_marker_scene(color_image, depth_mm)

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.depth_frame_id
        depth_msg.height = self.height
        depth_msg.width = self.width
        depth_msg.encoding = "16UC1"
        depth_msg.is_bigendian = False
        depth_msg.step = self.width * 2
        depth_msg.data = depth_mm.tobytes()
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(self._camera_info(self.depth_frame_id, stamp))

        color_msg = Image()
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = self.color_frame_id
        color_msg.height = self.height
        color_msg.width = self.width
        color_msg.encoding = "rgb8"
        color_msg.is_bigendian = False
        color_msg.step = self.width * 3
        color_msg.data = color_image.tobytes()
        self.color_pub.publish(color_msg)
        self.color_info_pub.publish(self._camera_info(self.color_frame_id, stamp))
        self._publish_frustum_marker(stamp)

        if self.publish_point_cloud:
            points_xyz = self._make_point_cloud_xyz(depth_mm)
            points_msg = PointCloud2()
            points_msg.header.stamp = stamp
            points_msg.header.frame_id = self.depth_frame_id
            points_msg.height = 1
            points_msg.width = int(points_xyz.shape[0])
            points_msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            points_msg.is_bigendian = False
            points_msg.point_step = 12
            points_msg.row_step = points_msg.point_step * points_msg.width
            points_msg.is_dense = True
            points_msg.data = points_xyz.tobytes()
            self.points_pub.publish(points_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ToolDepthCameraPublisher()
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
