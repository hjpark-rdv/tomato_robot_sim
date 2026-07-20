import math
import tkinter as tk
from tkinter import ttk

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [
    ("shoulder_pan_joint", -180.0, 180.0, -84.209168),
    ("shoulder_lift_joint", -130.0, 45.0, -97.939591),
    ("elbow_joint", -15.0, 180.0, 102.075036),
    ("wrist_1_joint", -180.0, 180.0, 87.640981),
    ("wrist_2_joint", -180.0, 180.0, 90.036766),
    ("wrist_3_joint", -180.0, 180.0, -4.545913),
    ("tool_bend_joint", -45.0, 45.0, 44.995947),
    ("tool_gripper_z_joint", -180.0, 180.0, 0.007919),
]

ACTION_NAME = "/fake_ur5_controller/follow_joint_trajectory"
SAVE_SERVICE_NAME = "/save_current_joints"


class JointControlGui(Node):
    def __init__(self) -> None:
        super().__init__("joint_control_gui")
        self.joint_names = [name for name, _, _, _ in JOINTS]
        self.current_positions_rad = {
            name: math.radians(home_deg) for name, _, _, home_deg in JOINTS
        }
        self.action_client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self.save_client = self.create_client(Trigger, SAVE_SERVICE_NAME)
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 20)

        self.root = tk.Tk()
        self.root.title("UR5 Tomato Joint Control")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.scales = {}
        self.value_labels = {}
        self._syncing_sliders = False
        self._auto_send_job = None
        self._last_status = tk.StringVar(value="Waiting for /joint_states")
        self._auto_send = tk.BooleanVar(value=False)
        self._move_time = tk.DoubleVar(value=0.35)

        self._build_ui()
        self.root.after(50, self._spin_ros)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=12)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self._syncing_sliders = True
        try:
            for row, (name, min_deg, max_deg, home_deg) in enumerate(JOINTS):
                ttk.Label(outer, text=name, width=24).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=3)
                scale = ttk.Scale(
                    outer,
                    from_=min_deg,
                    to=max_deg,
                    orient="horizontal",
                    command=lambda _value, joint_name=name: self._on_slider_changed(joint_name),
                )
                scale.set(home_deg)
                scale.grid(row=row, column=1, sticky="ew", pady=3)
                label = ttk.Label(outer, text=f"{home_deg:8.2f} deg", width=14, anchor="e")
                label.grid(row=row, column=2, sticky="e", padx=(8, 0), pady=3)
                self.scales[name] = scale
                self.value_labels[name] = label
        finally:
            self._syncing_sliders = False

        outer.columnconfigure(1, weight=1)
        controls = ttk.Frame(outer)
        controls.grid(row=len(JOINTS), column=0, columnspan=3, sticky="ew", pady=(12, 4))
        for column in range(7):
            controls.columnconfigure(column, weight=0)
        controls.columnconfigure(6, weight=1)

        ttk.Button(controls, text="Send", command=self.send_current_goal).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(controls, text="Read Current", command=self.read_current_into_sliders).grid(
            row=0, column=1, padx=(0, 6)
        )
        ttk.Button(controls, text="Home", command=self.send_home_goal).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(controls, text="Save Current", command=self.save_current_joints).grid(
            row=0, column=3, padx=(0, 12)
        )
        ttk.Checkbutton(controls, text="Auto send", variable=self._auto_send).grid(row=0, column=4, padx=(0, 12))
        ttk.Label(controls, text="Move time").grid(row=0, column=5, padx=(0, 4))
        ttk.Spinbox(controls, from_=0.05, to=5.0, increment=0.05, textvariable=self._move_time, width=7).grid(
            row=0, column=6, sticky="w"
        )

        ttk.Label(outer, textvariable=self._last_status, anchor="w").grid(
            row=len(JOINTS) + 1, column=0, columnspan=3, sticky="ew", pady=(8, 0)
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        updated = False
        for name, position in zip(msg.name, msg.position):
            if name in self.current_positions_rad:
                self.current_positions_rad[name] = float(position)
                updated = True
        if updated and not self._syncing_sliders:
            self.read_current_into_sliders(set_status=False)

    def _on_slider_changed(self, joint_name: str) -> None:
        if self._syncing_sliders:
            return
        value_deg = float(self.scales[joint_name].get())
        self.value_labels[joint_name].configure(text=f"{value_deg:8.2f} deg")
        if self._auto_send.get():
            if self._auto_send_job is not None:
                self.root.after_cancel(self._auto_send_job)
            self._auto_send_job = self.root.after(180, self.send_current_goal)

    def read_current_into_sliders(self, set_status: bool = True) -> None:
        self._syncing_sliders = True
        try:
            for name in self.joint_names:
                value_deg = math.degrees(self.current_positions_rad[name])
                self.scales[name].set(value_deg)
                self.value_labels[name].configure(text=f"{value_deg:8.2f} deg")
        finally:
            self._syncing_sliders = False
        if set_status:
            self._last_status.set("Loaded current /joint_states into sliders")

    def send_home_goal(self) -> None:
        self._syncing_sliders = True
        try:
            for name, _, _, home_deg in JOINTS:
                self.scales[name].set(home_deg)
                self.value_labels[name].configure(text=f"{home_deg:8.2f} deg")
        finally:
            self._syncing_sliders = False
        self.send_current_goal()

    def send_current_goal(self) -> None:
        if not rclpy.ok():
            return
        if not self.action_client.server_is_ready():
            if not self.action_client.wait_for_server(timeout_sec=0.05):
                self._last_status.set(f"Action server not ready: {ACTION_NAME}")
                return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [math.radians(float(self.scales[name].get())) for name in self.joint_names]
        move_time = max(0.05, float(self._move_time.get()))
        point.time_from_start.sec = int(move_time)
        point.time_from_start.nanosec = int((move_time - int(move_time)) * 1e9)
        goal.trajectory.points = [point]

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._last_status.set(f"Sent joint goal over {move_time:.2f}s")

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._last_status.set(f"Goal send failed: {exc}")
            return
        if not goal_handle.accepted:
            self._last_status.set("Joint goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:
            self._last_status.set(f"Goal result failed: {exc}")
            return
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self._last_status.set("Joint goal complete")
        else:
            self._last_status.set(f"Joint goal failed: {result.error_string}")

    def save_current_joints(self) -> None:
        if not self.save_client.service_is_ready():
            if not self.save_client.wait_for_service(timeout_sec=0.05):
                self._last_status.set(f"Save service not ready: {SAVE_SERVICE_NAME}")
                return
        future = self.save_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_save_result)
        self._last_status.set("Saving current joints...")

    def _on_save_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._last_status.set(f"Save failed: {exc}")
            return
        self._last_status.set(response.message if response.success else f"Save failed: {response.message}")

    def _spin_ros(self) -> None:
        if not rclpy.ok():
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(50, self._spin_ros)

    def _on_close(self) -> None:
        if self._auto_send_job is not None:
            self.root.after_cancel(self._auto_send_job)
            self._auto_send_job = None
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = JointControlGui()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
