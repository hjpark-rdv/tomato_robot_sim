import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformException, TransformListener

from tomato_moveit_teach.geometry import quaternion_from_matrix, target_stem_axis
from tomato_moveit_teach.moveit_grasp_planner import MoveItGraspPlanner


def matrix_from_quaternion(quaternion) -> np.ndarray:
    x, y, z, w = [float(value) for value in quaternion]
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


def transform_matrix(position, quaternion) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = matrix_from_quaternion(quaternion)
    transform[:3, 3] = np.array(position, dtype=float)
    return transform


def pose_from_matrix(transform: np.ndarray) -> Pose:
    qx, qy, qz, qw = quaternion_from_matrix(transform[:3, :3])
    pose = Pose()
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


class MoveItRightTomatoPoseTest(MoveItGraspPlanner):
    def __init__(self) -> None:
        super().__init__(node_name="moveit_right_tomato_pose_test")
        self.declare_parameter("reference_frame", "tomato_tf")
        self.declare_parameter("tool_frame", "grasp_tf")
        self.declare_parameter("left_tomato_z_spin_offset_deg", 250.0)
        self.declare_parameter("side", "right")
        self.declare_parameter("include_target_tomato", True)
        self.declare_parameter("use_current_tf", True)
        self.declare_parameter("relative_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("relative_rotation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("clear_tested_tomato_collision", True)
        self.declare_parameter("tomato_scene_set_parameters_service", "/tomato_scene_node/set_parameters")
        self.declare_parameter("allow_tested_tomato_touch", True)
        self.declare_parameter(
            "target_touch_links",
            [
                "tool_gripper_spin_link",
                "tool_gripper_link",
                "tool_rod_link",
                "tool_bent_rod_link",
                "tool_tip_link",
                "grasp_tf",
                "wrist_3_link",
            ],
        )

        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.tool_frame = str(self.get_parameter("tool_frame").value)
        self.left_tomato_z_spin_offset_deg = float(
            self.get_parameter("left_tomato_z_spin_offset_deg").value
        )
        self.target_z_spin_deg = self._local_tomato_z_spin_deg(self.branch_start, self.base_object_position)
        self.side = str(self.get_parameter("side").value).lower()
        self.include_target_tomato = bool(self.get_parameter("include_target_tomato").value)
        self.use_current_tf = bool(self.get_parameter("use_current_tf").value)
        self.relative_translation = [
            float(value) for value in self.get_parameter("relative_translation").value
        ]
        self.relative_rotation_xyzw = [
            float(value) for value in self.get_parameter("relative_rotation_xyzw").value
        ]
        self.clear_tested_tomato_collision = bool(
            self.get_parameter("clear_tested_tomato_collision").value
        )
        self.allow_tested_tomato_touch = bool(
            self.get_parameter("allow_tested_tomato_touch").value
        )
        self.target_touch_links = [
            str(link) for link in self.get_parameter("target_touch_links").value
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.get_scene_client = self.create_client(GetPlanningScene, "get_planning_scene")
        self.scene_param_client = self.create_client(
            SetParameters,
            str(self.get_parameter("tomato_scene_set_parameters_service").value),
        )
        self.planning_scene_pub = self.create_publisher(PlanningScene, "planning_scene", 10)
        self.suppressed_collision_ids: set[str] = set()
        self.suppression_timer = self.create_timer(0.1, self._publish_suppressed_collision_removals)

    def run_test(self) -> bool:
        if self.side not in {"left", "right", "both"}:
            self.get_logger().error("side must be one of: left, right, both")
            return False
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("MoveIt action server was not found. Is teach.launch.py running?")
            return False
        if (
            (self.clear_tested_tomato_collision or self.allow_tested_tomato_touch)
            and not self.apply_scene_client.wait_for_service(timeout_sec=5.0)
        ):
            self.get_logger().error("ApplyPlanningScene service was not found.")
            return False
        if self.allow_tested_tomato_touch and not self.get_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("GetPlanningScene service was not found.")
            return False
        if self.clear_tested_tomato_collision and not self.scene_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("tomato_scene_node parameter service was not found.")
            return False

        relative_transform = self._lookup_current_relative_grasp()
        if relative_transform is None:
            return False

        entries = self._selected_tomato_entries()
        if not entries:
            self.get_logger().error(f"No tomato entries matched side={self.side!r}.")
            return False

        results = []
        for rank, entry in enumerate(entries, start=1):
            base_to_tomato = self._base_to_tomato_transform(entry)
            base_to_grasp = base_to_tomato @ relative_transform
            target_pose = self._plan_pose_from_grasp(base_to_grasp[:3, 3], base_to_grasp[:3, :3])
            self.get_logger().info(
                "Testing taught relative grasp on "
                f"side={entry['side']} rank={rank} index={entry['index']} "
                f"tomato_xyz=({entry['position'][0]:.4f}, {entry['position'][1]:.4f}, {entry['position'][2]:.4f}) "
                f"{self.target_link}_xyz=({target_pose.position.x:.4f}, "
                f"{target_pose.position.y:.4f}, {target_pose.position.z:.4f})"
            )
            if self.clear_tested_tomato_collision:
                self._remove_tested_tomato_collision(entry)
            if self.allow_tested_tomato_touch:
                self._allow_tested_tomato_touch(entry)
            success = self._plan_to_pose(target_pose)
            results.append((entry, success))

        success_count = sum(1 for _, success in results if success)
        summary = ", ".join(
            f"{entry['side']}#{rank}:idx{entry['index']}={'OK' if success else 'FAIL'}"
            for rank, (entry, success) in enumerate(results, start=1)
        )
        self.get_logger().info(f"Right tomato pose test summary: {summary}")
        self.get_logger().info(f"Success count: {success_count}/{len(results)}")
        return success_count == len(results)

    def _remove_tested_tomato_collision(self, entry: dict[str, object]) -> None:
        object_id = "target_tomato" if bool(entry["target"]) else f"other_tomato_{int(entry['index'])}"
        self.suppressed_collision_ids.add(object_id)
        self._set_tomato_scene_suppressed_collision_ids()
        self._publish_suppressed_collision_removals()

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.base_frame
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_object)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().warn(f"Could not remove tested tomato collision object: {object_id}")
        else:
            self.get_logger().info(f"Temporarily removed tested tomato collision object: {object_id}")

    def _set_tomato_scene_suppressed_collision_ids(self) -> None:
        parameter = Parameter()
        parameter.name = "suppressed_collision_ids"
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_STRING_ARRAY
        parameter.value.string_array_value = sorted(self.suppressed_collision_ids)

        request = SetParameters.Request()
        request.parameters = [parameter]
        future = self.scene_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.results or not response.results[0].successful:
            reason = "" if response is None or not response.results else response.results[0].reason
            self.get_logger().warn(f"Could not update tomato_scene_node suppressed collisions: {reason}")

    def _allow_tested_tomato_touch(self, entry: dict[str, object]) -> None:
        object_id = "target_tomato" if bool(entry["target"]) else f"other_tomato_{int(entry['index'])}"
        acm = self._current_allowed_collision_matrix()
        if acm is None:
            self.get_logger().warn(f"Could not read allowed collision matrix for: {object_id}")
            return

        names = list(acm.entry_names)
        rows = [list(row.enabled) for row in acm.entry_values]

        def ensure_name(name: str) -> int:
            if name in names:
                index = names.index(name)
            else:
                index = len(names)
                names.append(name)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
            for row in rows:
                while len(row) < len(names):
                    row.append(False)
            while len(rows) < len(names):
                rows.append([False] * len(names))
            return index

        object_index = ensure_name(object_id)
        for link_name in self.target_touch_links:
            link_index = ensure_name(link_name)
            rows[object_index][link_index] = True
            rows[link_index][object_index] = True

        merged = AllowedCollisionMatrix()
        merged.entry_names = names
        merged.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]
        merged.default_entry_names = list(acm.default_entry_names)
        merged.default_entry_values = list(acm.default_entry_values)

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = merged

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().warn(f"Could not allow tested tomato touch links for: {object_id}")
        else:
            self.get_logger().info(
                f"Allowed tested tomato touch: {object_id} with {', '.join(self.target_touch_links)}"
            )

    def _current_allowed_collision_matrix(self) -> AllowedCollisionMatrix | None:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None:
            return None
        return response.scene.allowed_collision_matrix

    def _publish_suppressed_collision_removals(self) -> None:
        if not self.suppressed_collision_ids:
            return
        scene = PlanningScene()
        scene.is_diff = True
        for object_id in sorted(self.suppressed_collision_ids):
            collision_object = CollisionObject()
            collision_object.header.frame_id = self.base_frame
            collision_object.id = object_id
            collision_object.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(collision_object)
        self.planning_scene_pub.publish(scene)

    def _lookup_current_relative_grasp(self) -> np.ndarray | None:
        if not self.use_current_tf:
            matrix = transform_matrix(self.relative_translation, self.relative_rotation_xyzw)
            self.get_logger().info(
                f"Using provided {self.reference_frame} -> {self.tool_frame}: "
                f"xyz=({self.relative_translation[0]:.4f}, "
                f"{self.relative_translation[1]:.4f}, {self.relative_translation[2]:.4f}), "
                f"xyzw=({self.relative_rotation_xyzw[0]:.4f}, "
                f"{self.relative_rotation_xyzw[1]:.4f}, {self.relative_rotation_xyzw[2]:.4f}, "
                f"{self.relative_rotation_xyzw[3]:.4f})"
            )
            return matrix

        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    self.tool_frame,
                    rclpy.time.Time(),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                matrix = transform_matrix(
                    [translation.x, translation.y, translation.z],
                    [rotation.x, rotation.y, rotation.z, rotation.w],
                )
                self.get_logger().info(
                    f"Using current {self.reference_frame} -> {self.tool_frame}: "
                    f"xyz=({translation.x:.4f}, {translation.y:.4f}, {translation.z:.4f}), "
                    f"xyzw=({rotation.x:.4f}, {rotation.y:.4f}, {rotation.z:.4f}, {rotation.w:.4f})"
                )
                return matrix
            except TransformException:
                continue
        self.get_logger().error(f"Could not read TF {self.reference_frame} -> {self.tool_frame}.")
        return None

    def _selected_tomato_entries(self) -> list[dict[str, object]]:
        entries = self._tomato_layout_entries()
        if not self.include_target_tomato:
            entries = [entry for entry in entries if not bool(entry["target"])]
        if self.side == "both":
            selected = entries
        else:
            selected = [entry for entry in entries if str(entry["side"]) == self.side]
        return sorted(selected, key=lambda entry: float(entry["position"][2]), reverse=True)

    def _tomato_layout_entries(self) -> list[dict[str, object]]:
        main_start, main_end = self._vine_points()
        tomato_specs = [
            {"offset": [0.20, -0.35, 5.35], "stem_segment_len": 1.20, "target": False},
            {"offset": [0.45, 2.85, 4.25], "stem_segment_len": 1.30, "target": False},
            {"offset": [-0.15, -0.60, 2.95], "stem_segment_len": 1.20, "target": False},
            {"offset": [0.25, 2.55, 1.60], "stem_segment_len": 1.35, "target": False},
            {"offset": [0.0, 0.0, 0.0], "stem_segment_len": 1.35, "target": True},
            {"offset": [0.55, 2.80, -1.35], "stem_segment_len": 1.30, "target": False},
            {"offset": [-0.30, -0.45, -2.80], "stem_segment_len": 1.15, "target": False},
            {"offset": [0.25, 1.75, -4.70], "stem_segment_len": 1.10, "target": False},
        ]
        entries = []
        for index, spec in enumerate(tomato_specs):
            base_tomato_pos = self.base_object_position + np.array(spec["offset"], dtype=float) * self.object_radius
            if bool(spec["target"]):
                base_tomato_pos = self.base_object_position.copy()
            tomato_pos = self._spin_point_around_main_vine(base_tomato_pos)
            base_branch_start = self._branch_start_for_down_angle(
                main_start,
                main_end,
                base_tomato_pos,
                45.0,
            )
            branch_start = self._spin_point_around_main_vine(base_branch_start)
            base_stem_axis = target_stem_axis(base_tomato_pos)
            stem_axis = self._spin_vector_around_main_vine(base_stem_axis)
            is_left = self._is_left_of_vine_from_robot(branch_start, tomato_pos)
            z_spin_deg = self._local_tomato_z_spin_deg(base_branch_start, base_tomato_pos)
            entries.append(
                {
                    "index": index,
                    "target": bool(spec["target"]),
                    "position": tomato_pos,
                    "base_stem_axis": base_stem_axis,
                    "stem_axis": stem_axis,
                    "z_spin_deg": z_spin_deg,
                    "side": "left" if is_left else "right",
                }
            )
        return entries

    def _base_to_tomato_transform(self, entry: dict[str, object]) -> np.ndarray:
        tomato_x, tomato_y, tomato_z = self._spun_tomato_frame_axes_for(
            entry.get("base_stem_axis", entry["stem_axis"]),
            float(entry["z_spin_deg"]),
        )
        transform = np.eye(4)
        transform[:3, :3] = np.column_stack((tomato_x, tomato_y, tomato_z))
        transform[:3, 3] = np.array(entry["position"], dtype=float)
        return transform

    def _vine_points(self):
        main_start = self.base_object_position + np.array(
            [-self.object_radius * 0.75, self.object_radius * 1.35, -self.object_radius * 6.00]
        )
        main_end = self.base_object_position + np.array(
            [-self.object_radius * 0.35, self.object_radius * 1.15, self.object_radius * 6.80]
        )
        return main_start, main_end

    def _point_on_segment_at_z(self, start, end, z_value):
        start = np.array(start, dtype=float)
        end = np.array(end, dtype=float)
        z_delta = float(end[2] - start[2])
        if abs(z_delta) < 1e-6:
            return (start + end) * 0.5
        t = float(np.clip((z_value - start[2]) / z_delta, 0.0, 1.0))
        return (1.0 - t) * start + t * end

    def _branch_start_for_down_angle(self, main_start, main_end, tomato_pos, down_angle_deg):
        start = np.array(main_start, dtype=float)
        end = np.array(main_end, dtype=float)
        tomato_pos = np.array(tomato_pos, dtype=float)
        z_min = float(min(start[2], end[2]))
        z_max = float(max(start[2], end[2]))
        slope = math.tan(math.radians(float(down_angle_deg)))

        def residual(z_value: float) -> float:
            point = self._point_on_segment_at_z(start, end, z_value)
            horizontal_distance = float(np.linalg.norm(point[:2] - tomato_pos[:2]))
            return float(point[2] - tomato_pos[2] - horizontal_distance * slope)

        low = max(z_min, float(tomato_pos[2]))
        high = z_max
        if residual(low) >= 0.0:
            return self._point_on_segment_at_z(start, end, low)
        if residual(high) <= 0.0:
            return self._point_on_segment_at_z(start, end, high)
        for _ in range(32):
            mid = (low + high) * 0.5
            if residual(mid) < 0.0:
                low = mid
            else:
                high = mid
        return self._point_on_segment_at_z(start, end, (low + high) * 0.5)

    def _is_left_of_vine_from_robot(self, branch_anchor, tomato_pos) -> bool:
        anchor_xy = np.array(branch_anchor[:2], dtype=float)
        tomato_xy = np.array(tomato_pos[:2], dtype=float)
        view_forward = anchor_xy.copy()
        if float(np.linalg.norm(view_forward)) < 1e-6:
            view_forward = tomato_xy.copy()
        if float(np.linalg.norm(view_forward)) < 1e-6:
            return False
        view_forward /= np.linalg.norm(view_forward)
        robot_left = np.array([-view_forward[1], view_forward[0]], dtype=float)
        tomato_side = tomato_xy - anchor_xy
        return float(np.dot(tomato_side, robot_left)) > 0.0

    def _tomato_frame_axes_for(self, stem_axis, z_spin_deg):
        tomato_z_axis = np.array(stem_axis, dtype=float)
        tomato_z_axis /= np.linalg.norm(tomato_z_axis)
        x_hint = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(tomato_z_axis, x_hint))) > 0.95:
            x_hint = np.array([0.0, 1.0, 0.0])
        tomato_y_axis = np.cross(tomato_z_axis, x_hint)
        tomato_y_axis /= np.linalg.norm(tomato_y_axis)
        tomato_x_axis = np.cross(tomato_y_axis, tomato_z_axis)
        tomato_x_axis /= np.linalg.norm(tomato_x_axis)
        spin_rad = math.radians(float(z_spin_deg))
        c = math.cos(spin_rad)
        s = math.sin(spin_rad)
        spun_x_axis = c * tomato_x_axis + s * tomato_y_axis
        spun_y_axis = -s * tomato_x_axis + c * tomato_y_axis
        spun_x_axis /= np.linalg.norm(spun_x_axis)
        spun_y_axis /= np.linalg.norm(spun_y_axis)
        return spun_x_axis, spun_y_axis, tomato_z_axis


def main() -> None:
    rclpy.init()
    node = MoveItRightTomatoPoseTest()
    try:
        success = node.run_test()
        if not success:
            raise SystemExit(1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
