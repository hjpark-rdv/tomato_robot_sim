import argparse
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


PALM_DEPTH = 0.012
FINGER_LEN = 0.075 * 0.70
OPENING = 0.0875 * 0.60
FINGER_THICKNESS = 0.007
GRASP_Z = PALM_DEPTH + FINGER_LEN * 0.62
FINGERTIP_Z = PALM_DEPTH + FINGER_LEN


def quaternion_to_matrix(q) -> np.ndarray:
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


def transform_to_position_rotation(transform) -> tuple[np.ndarray, np.ndarray]:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    position = np.array([translation.x, translation.y, translation.z], dtype=float)
    matrix = quaternion_to_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    return position, matrix


def vector_angle_deg(a: np.ndarray, b: np.ndarray, absolute: bool = False) -> float:
    a = np.array(a, dtype=float)
    b = np.array(b, dtype=float)
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    if a_norm < 1e-12 or b_norm < 1e-12:
        return 180.0
    dot = float(np.dot(a / a_norm, b / b_norm))
    if absolute:
        dot = abs(dot)
    dot = float(np.clip(dot, -1.0, 1.0))
    return float(math.degrees(math.acos(dot)))


def pass_fail(ok: bool) -> str:
    return "PASS" if ok else "FAIL"


class GraspPoseDiagnostics(Node):
    def __init__(self, args) -> None:
        super().__init__("grasp_pose_diagnostics")
        self.args = args
        self.latest_joint_state: JointState | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, "joint_states", self._joint_state_callback, 10)

    def _joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _lookup(self, frame: str):
        return self.tf_buffer.lookup_transform(
            self.args.reference_frame,
            frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.1),
        )

    def wait_for_inputs(self) -> None:
        deadline = self.get_clock().now() + Duration(seconds=float(self.args.timeout))
        while rclpy.ok() and self.latest_joint_state is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest_joint_state is None:
            raise RuntimeError("No /joint_states message was received before timeout")

        required_frames = [
            self.args.tomato_frame,
            self.args.grasp_frame,
            self.args.tool0_frame,
            self.args.gripper_link_frame,
        ]
        pending = list(required_frames)
        while rclpy.ok() and pending and self.get_clock().now() < deadline:
            next_pending = []
            for frame in pending:
                try:
                    self._lookup(frame)
                except TransformException:
                    next_pending.append(frame)
            pending = next_pending
            if pending:
                rclpy.spin_once(self, timeout_sec=0.05)
        if pending:
            raise RuntimeError("Missing TF frames before timeout: " + ", ".join(pending))

    def evaluate(self) -> dict[str, object]:
        tomato_pos, tomato_rot = transform_to_position_rotation(self._lookup(self.args.tomato_frame))
        grasp_pos, gripper_rot = transform_to_position_rotation(self._lookup(self.args.grasp_frame))
        tool0_pos, _tool0_rot = transform_to_position_rotation(self._lookup(self.args.tool0_frame))
        gripper_link_pos, _gripper_link_rot = transform_to_position_rotation(self._lookup(self.args.gripper_link_frame))

        tomato_y = tomato_rot[:, 1]
        tomato_z = tomato_rot[:, 2]
        gripper_x = gripper_rot[:, 0]
        gripper_z = gripper_rot[:, 2]
        local_gripper_rot = tomato_rot.T @ gripper_rot

        tool_tip_offset = np.array([0.0, 0.0, -GRASP_Z], dtype=float)
        tool_tip_pos = grasp_pos + gripper_rot @ tool_tip_offset
        tool_tip_local = tomato_rot.T @ (tool_tip_pos - tomato_pos)
        tool_tip_target_local = np.array([tool_tip_local[0], 0.0, 0.0], dtype=float)
        tool_tip_target_pos = tomato_pos + tomato_rot @ tool_tip_target_local

        half_opening = float(self.args.gripper_opening) * 0.5
        cap_inner_half_width = FINGER_THICKNESS * 0.8
        tip_z_from_grasp_tf = FINGERTIP_Z - GRASP_Z
        blue_tip_offset = np.array([-half_opening + cap_inner_half_width, 0.0, tip_z_from_grasp_tf], dtype=float)
        orange_tip_offset = np.array([half_opening - cap_inner_half_width, 0.0, tip_z_from_grasp_tf], dtype=float)
        blue_tip_pos = grasp_pos + gripper_rot @ blue_tip_offset
        orange_tip_pos = grasp_pos + gripper_rot @ orange_tip_offset

        gripper_half_span = float((orange_tip_offset[0] - blue_tip_offset[0]) * 0.5)
        tomato_radius = float(self.args.object_radius) * float(self.args.tomato_radius_scale)
        contact_half_span = min(gripper_half_span, tomato_radius * 1.06)
        calyx_target = tomato_pos + tomato_rot @ np.array([0.0, 0.0, contact_half_span], dtype=float)
        opposite_target = tomato_pos + tomato_rot @ np.array([0.0, 0.0, -contact_half_span], dtype=float)

        blue_alignment = float(np.clip(np.dot(-local_gripper_rot[:, 0], np.array([0.0, 0.0, 1.0])), -1.0, 1.0))
        orange_alignment = float(np.clip(np.dot(local_gripper_rot[:, 0], np.array([0.0, 0.0, 1.0])), -1.0, 1.0))
        blue_alignment_angle = float(math.degrees(math.acos(blue_alignment)))
        orange_alignment_angle = float(math.degrees(math.acos(orange_alignment)))

        blue_calyx_error = float(np.linalg.norm(blue_tip_pos - calyx_target))
        orange_opposite_error = float(np.linalg.norm(orange_tip_pos - opposite_target))
        orange_calyx_error = float(np.linalg.norm(orange_tip_pos - calyx_target))
        blue_opposite_error = float(np.linalg.norm(blue_tip_pos - opposite_target))

        target_tolerance = max(float(self.args.target_tolerance), 1e-6)
        alignment_tolerance = max(float(self.args.blue_alignment_tolerance_deg), 1e-6)
        blue_calyx_score = max(
            blue_calyx_error / target_tolerance,
            orange_opposite_error / target_tolerance,
            blue_alignment_angle / alignment_tolerance,
        )
        orange_calyx_score = max(
            orange_calyx_error / target_tolerance,
            blue_opposite_error / target_tolerance,
            orange_alignment_angle / alignment_tolerance,
        )
        if orange_calyx_score < blue_calyx_score:
            calyx_finger = "orange"
            opposite_finger = "blue"
            calyx_contact_error = orange_calyx_error
            opposite_contact_error = blue_opposite_error
            calyx_alignment_angle = orange_alignment_angle
        else:
            calyx_finger = "blue"
            opposite_finger = "orange"
            calyx_contact_error = blue_calyx_error
            opposite_contact_error = orange_opposite_error
            calyx_alignment_angle = blue_alignment_angle

        approach_dot = float(np.dot(gripper_z, tomato_y))
        approach_sign = 1.0 if approach_dot >= 0.0 else -1.0
        preferred_approach_sign = 1.0 if float(self.args.preferred_approach_sign) >= 0.0 else -1.0
        spin_deg = None
        if self.latest_joint_state is not None and self.args.spin_joint in self.latest_joint_state.name:
            index = list(self.latest_joint_state.name).index(self.args.spin_joint)
            spin_deg = math.degrees(float(self.latest_joint_state.position[index]))

        return {
            "tomato_frame": self.args.tomato_frame,
            "tomato_position": tomato_pos,
            "grasp_position": grasp_pos,
            "tool_tip_position": tool_tip_pos,
            "tool0_position": tool0_pos,
            "tool_gripper_link_position": gripper_link_pos,
            "spin_deg": spin_deg,
            "approach_dot": approach_dot,
            "approach_sign": approach_sign,
            "preferred_approach_sign": preferred_approach_sign,
            "rod_tomato_z_parallel_angle_deg": vector_angle_deg(gripper_x, tomato_z, absolute=True),
            "gripper_tomato_y_parallel_angle_deg": vector_angle_deg(gripper_z, tomato_y, absolute=True),
            "gripper_tomato_y_signed_angle_deg": vector_angle_deg(gripper_z, preferred_approach_sign * tomato_y),
            "tool0_z_minus_gripper_link_z": float(tool0_pos[2] - gripper_link_pos[2]),
            "tool_tip_z_above_center": float(tool_tip_pos[2] - tomato_pos[2]),
            "grasp_tf_z_above_center": float(grasp_pos[2] - tomato_pos[2]),
            "tool_tip_target_error": float(np.linalg.norm(tool_tip_pos - tool_tip_target_pos)),
            "tool_tip_x_axis_offset": float(np.linalg.norm(tool_tip_local[1:])),
            "blue_calyx_contact_error": blue_calyx_error,
            "orange_opposite_contact_error": orange_opposite_error,
            "orange_calyx_contact_error": orange_calyx_error,
            "blue_opposite_contact_error": blue_opposite_error,
            "blue_alignment_angle_deg": blue_alignment_angle,
            "orange_alignment_angle_deg": orange_alignment_angle,
            "calyx_finger": calyx_finger,
            "opposite_finger": opposite_finger,
            "calyx_contact_error": calyx_contact_error,
            "opposite_contact_error": opposite_contact_error,
            "calyx_alignment_angle_deg": calyx_alignment_angle,
        }

    def print_report(self, metrics: dict[str, object]) -> None:
        target_tolerance = float(self.args.target_tolerance)
        tomato_radius = float(self.args.object_radius) * float(self.args.tomato_radius_scale)
        x_axis_tolerance = min(target_tolerance, tomato_radius * 0.15)
        axis_tolerance = float(self.args.rule_axis_tolerance_deg)
        alignment_tolerance = float(self.args.blue_alignment_tolerance_deg)
        checks = [
            (
                "approach side",
                float(metrics["approach_sign"]) == float(metrics["preferred_approach_sign"]),
                f"actual={metrics['approach_sign']:+.0f}, required={metrics['preferred_approach_sign']:+.0f}, "
                f"dot={metrics['approach_dot']:+.4f}",
            ),
            (
                "rod X parallel tomato Z",
                float(metrics["rod_tomato_z_parallel_angle_deg"]) <= axis_tolerance,
                f"{metrics['rod_tomato_z_parallel_angle_deg']:.3f}deg <= {axis_tolerance:.3f}deg",
            ),
            (
                "gripper Z parallel tomato Y",
                float(metrics["gripper_tomato_y_parallel_angle_deg"]) <= axis_tolerance,
                f"{metrics['gripper_tomato_y_parallel_angle_deg']:.3f}deg <= {axis_tolerance:.3f}deg",
            ),
            (
                "gripper Z on required tomato-Y side",
                float(metrics["gripper_tomato_y_signed_angle_deg"]) <= axis_tolerance,
                f"{metrics['gripper_tomato_y_signed_angle_deg']:.3f}deg <= {axis_tolerance:.3f}deg",
            ),
            (
                "tool_tip target error",
                float(metrics["tool_tip_target_error"]) <= target_tolerance,
                f"{metrics['tool_tip_target_error']:.4f}m <= {target_tolerance:.4f}m",
            ),
            (
                "tool_tip on tomato X axis",
                float(metrics["tool_tip_x_axis_offset"]) <= x_axis_tolerance,
                f"{metrics['tool_tip_x_axis_offset']:.4f}m <= {x_axis_tolerance:.4f}m",
            ),
            (
                "calyx finger alignment",
                float(metrics["calyx_alignment_angle_deg"]) <= alignment_tolerance,
                f"{metrics['calyx_finger']}={metrics['calyx_alignment_angle_deg']:.3f}deg <= "
                f"{alignment_tolerance:.3f}deg",
            ),
            (
                "calyx fingertip contact",
                float(metrics["calyx_contact_error"]) <= target_tolerance,
                f"{metrics['calyx_finger']}={metrics['calyx_contact_error']:.4f}m <= {target_tolerance:.4f}m",
            ),
            (
                "opposite fingertip contact",
                float(metrics["opposite_contact_error"]) <= target_tolerance,
                f"{metrics['opposite_finger']}={metrics['opposite_contact_error']:.4f}m <= "
                f"{target_tolerance:.4f}m",
            ),
        ]

        print(f"tomato_frame: {metrics['tomato_frame']}")
        if metrics["spin_deg"] is not None:
            print(f"{self.args.spin_joint}: {metrics['spin_deg']:.3f}deg")
        print(
            "tomato_xyz: "
            f"({metrics['tomato_position'][0]:+.4f}, {metrics['tomato_position'][1]:+.4f}, "
            f"{metrics['tomato_position'][2]:+.4f})"
        )
        print(
            "grasp_tf_xyz: "
            f"({metrics['grasp_position'][0]:+.4f}, {metrics['grasp_position'][1]:+.4f}, "
            f"{metrics['grasp_position'][2]:+.4f})"
        )
        print(
            "tool_tip_xyz: "
            f"({metrics['tool_tip_position'][0]:+.4f}, {metrics['tool_tip_position'][1]:+.4f}, "
            f"{metrics['tool_tip_position'][2]:+.4f})"
        )
        print(f"tool0_z_minus_gripper_link_z: {metrics['tool0_z_minus_gripper_link_z']:+.4f}m (info only)")
        print("")
        print("[finger assignment]")
        print(
            f"selected: {metrics['calyx_finger']} -> calyx, {metrics['opposite_finger']} -> opposite"
        )
        print(
            "blue->calyx/orange->opposite: "
            f"blue_err={metrics['blue_calyx_contact_error']:.4f}m, "
            f"orange_err={metrics['orange_opposite_contact_error']:.4f}m, "
            f"blue_align={metrics['blue_alignment_angle_deg']:.3f}deg"
        )
        print(
            "orange->calyx/blue->opposite: "
            f"orange_err={metrics['orange_calyx_contact_error']:.4f}m, "
            f"blue_err={metrics['blue_opposite_contact_error']:.4f}m, "
            f"orange_align={metrics['orange_alignment_angle_deg']:.3f}deg"
        )
        print("")
        print("[condition checks]")
        for name, ok, detail in checks:
            print(f"{pass_fail(ok):4s} {name:34s} {detail}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate the current gripper pose against tomato grasp rules.")
    parser.add_argument("--reference-frame", default="base_link")
    parser.add_argument("--tomato-frame", default="tomato_tf")
    parser.add_argument("--grasp-frame", default="grasp_tf")
    parser.add_argument("--tool0-frame", default="tool0")
    parser.add_argument("--gripper-link-frame", default="tool_gripper_link")
    parser.add_argument("--spin-joint", default="tool_gripper_z_joint")
    parser.add_argument("--preferred-approach-sign", type=float, default=1.0)
    parser.add_argument("--object-radius", type=float, default=0.036)
    parser.add_argument("--tomato-radius-scale", type=float, default=0.5)
    parser.add_argument("--gripper-opening", type=float, default=OPENING)
    parser.add_argument("--target-tolerance", type=float, default=0.003)
    parser.add_argument("--rule-axis-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--blue-alignment-tolerance-deg", type=float, default=6.0)
    parser.add_argument("--min-tool0-z-above-gripper-link", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = GraspPoseDiagnostics(args)
    try:
        node.wait_for_inputs()
        node.print_report(node.evaluate())
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
