import argparse
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_FRAMES = [
    "tool0",
    "tool_gripper_spin_link",
    "tool_gripper_link",
    "tool_tip_link",
    "tomato_tf",
    "tomato_0_tf",
    "tomato_1_tf",
    "tomato_2_tf",
    "tomato_3_tf",
    "tomato_4_tf",
    "tomato_5_tf",
    "tomato_6_tf",
    "tomato_7_tf",
]


def _quaternion_to_matrix(q):
    x, y, z, w = [float(v) for v in q]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _quaternion_angle_deg(q_a, q_b) -> float:
    rot_a = _quaternion_to_matrix(q_a)
    rot_b = _quaternion_to_matrix(q_b)
    delta = rot_a.T @ rot_b
    trace_value = float(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(trace_value))


def _read_snapshot(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, list):
        if not data:
            raise RuntimeError(f"{path} is an empty snapshot list")
        return data[-1]
    if not isinstance(data, dict):
        raise RuntimeError(f"{path} does not contain a snapshot object")
    return data


class PoseSnapshotNode(Node):
    def __init__(self, args) -> None:
        super().__init__("pose_snapshot")
        self.args = args
        self.latest_joint_state: JointState | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, "joint_states", self._joint_state_callback, 10)

    def _joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def capture(self) -> dict:
        deadline = self.get_clock().now() + Duration(seconds=float(self.args.timeout))
        while rclpy.ok() and self.latest_joint_state is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest_joint_state is None:
            raise RuntimeError("No /joint_states message was received before timeout")

        tf_warmup_deadline = self.get_clock().now() + Duration(seconds=min(1.0, float(self.args.timeout)))
        while rclpy.ok() and self.get_clock().now() < tf_warmup_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        frames = {}
        frame_deadline = self.get_clock().now() + Duration(seconds=float(self.args.timeout))
        pending_frames = list(self.args.frames)
        while rclpy.ok() and pending_frames and self.get_clock().now() < frame_deadline:
            next_pending = []
            for frame in pending_frames:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.args.reference_frame,
                        frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.05),
                    )
                except TransformException:
                    next_pending.append(frame)
                    continue
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                frames[frame] = {
                    "translation": {
                        "x": float(translation.x),
                        "y": float(translation.y),
                        "z": float(translation.z),
                    },
                    "rotation_xyzw": {
                        "x": float(rotation.x),
                        "y": float(rotation.y),
                        "z": float(rotation.z),
                        "w": float(rotation.w),
                    },
                }
            pending_frames = next_pending
            if pending_frames:
                rclpy.spin_once(self, timeout_sec=0.05)

        joint_state = self.latest_joint_state
        joint_positions_rad = {
            name: float(position)
            for name, position in zip(joint_state.name, joint_state.position)
        }
        stamp = self.get_clock().now().to_msg()
        return {
            "label": self.args.label,
            "stamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "reference_frame": self.args.reference_frame,
            "joint_positions_rad": joint_positions_rad,
            "joint_positions_deg": {
                name: math.degrees(position)
                for name, position in joint_positions_rad.items()
            },
            "frames": frames,
            "missing_frames": pending_frames,
        }


def _write_snapshot(path: Path, snapshot: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(snapshot, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _translation_vector(frame_payload: dict) -> np.ndarray:
    translation = frame_payload["translation"]
    return np.array([translation["x"], translation["y"], translation["z"]], dtype=float)


def _rotation_xyzw(frame_payload: dict) -> list[float]:
    rotation = frame_payload["rotation_xyzw"]
    return [rotation["x"], rotation["y"], rotation["z"], rotation["w"]]


def compare_snapshots(before_path: Path, after_path: Path, frames: list[str]) -> None:
    before = _read_snapshot(before_path)
    after = _read_snapshot(after_path)
    print(f"before: {before.get('label', before_path.name)}")
    print(f"after : {after.get('label', after_path.name)}")
    print("")

    before_joints = before.get("joint_positions_rad", {})
    after_joints = after.get("joint_positions_rad", {})
    joint_names = sorted(set(before_joints) & set(after_joints))
    print("[joint delta]")
    for name in joint_names:
        delta = math.degrees(float(after_joints[name]) - float(before_joints[name]))
        before_deg = math.degrees(float(before_joints[name]))
        after_deg = math.degrees(float(after_joints[name]))
        print(f"{name:24s} before={before_deg:8.3f}deg after={after_deg:8.3f}deg delta={delta:8.3f}deg")

    print("")
    print("[frame delta]")
    before_frames = before.get("frames", {})
    after_frames = after.get("frames", {})
    for frame in frames:
        if frame not in before_frames or frame not in after_frames:
            continue
        before_pos = _translation_vector(before_frames[frame])
        after_pos = _translation_vector(after_frames[frame])
        position_delta = after_pos - before_pos
        position_error = float(np.linalg.norm(position_delta))
        angle_error = _quaternion_angle_deg(_rotation_xyzw(before_frames[frame]), _rotation_xyzw(after_frames[frame]))
        print(
            f"{frame:24s} pos_delta=({position_delta[0]:+.4f}, {position_delta[1]:+.4f}, "
            f"{position_delta[2]:+.4f})m norm={position_error:.4f}m rot_delta={angle_error:.3f}deg"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture or compare ROS TF/joint pose snapshots.")
    parser.add_argument("--label", default="pose_snapshot")
    parser.add_argument("--output", default="/root/pybullet_ur_approach/pose_snapshots/pose_snapshot.json")
    parser.add_argument("--reference-frame", default="base_link")
    parser.add_argument("--frames", nargs="*", default=DEFAULT_FRAMES)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--compare", nargs=2, metavar=("BEFORE_JSON", "AFTER_JSON"))
    args = parser.parse_args()

    if args.compare:
        compare_snapshots(Path(args.compare[0]).expanduser(), Path(args.compare[1]).expanduser(), list(args.frames))
        return

    rclpy.init()
    node = PoseSnapshotNode(args)
    try:
        snapshot = node.capture()
        output_path = Path(args.output).expanduser()
        _write_snapshot(output_path, snapshot)
        print(f"Saved pose snapshot to {output_path}")
        if snapshot["missing_frames"]:
            print("Missing frames: " + ", ".join(snapshot["missing_frames"]))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
