import math
import time
import sys
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import TransformException

from tomato_moveit_teach.moveit_harvest_sequence import MoveItHarvestSequence, matrix_from_quaternion
from tomato_moveit_teach.trajectory_dataset import append_jsonl, make_label_sample


class TrajectoryLabelerGui(MoveItHarvestSequence):
    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("trajectory_label_dataset_path", "/root/pybullet_ur_approach/trajectory_dataset/labels.jsonl")
        self.visible_entries: list[dict[str, object]] = []
        self.latest_record: dict[str, object] | None = None
        self.latest_payload: dict[str, object] | None = None
        self.latest_valid_records: list[dict[str, object]] = []
        self.liked_candidate_indices: set[int] = set()
        self._populating_candidate_tree = False
        self._gui_closed = False
        self.current_planning_request_id = ""
        self._candidate_id_counter = 0
        self.current_rank = 1

    def run_gui(self) -> None:
        root = tk.Tk()
        self.root = root
        root.title("Tomato Trajectory Labeler")
        root.geometry("1120x720")

        top = ttk.Frame(root, padding=8)
        top.pack(fill=tk.BOTH, expand=True)

        status_bar = ttk.Frame(top)
        status_bar.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(status_bar, text="State").pack(side=tk.LEFT, padx=(0, 6))
        self.state_var = tk.StringVar(value="READY")
        self.state_label = ttk.Label(status_bar, textvariable=self.state_var, width=18, anchor=tk.CENTER)
        self.state_label.pack(side=tk.LEFT)
        self.status_var = tk.StringVar(value="Start RViz/MoveIt first, then capture visible tomatoes.")
        ttk.Label(status_bar, textvariable=self.status_var, anchor=tk.W).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

        controls = ttk.Frame(top)
        controls.pack(fill=tk.X)

        self.action_buttons: list[ttk.Button] = []
        self.capture_button = ttk.Button(controls, text="Capture Visible Tomatoes", command=self._ui_capture_visible)
        self.capture_button.pack(side=tk.LEFT, padx=4)
        self.action_buttons.append(self.capture_button)
        self.plan_button = ttk.Button(controls, text="Plan Selected", command=self._ui_plan_selected)
        self.plan_button.pack(side=tk.LEFT, padx=4)
        self.action_buttons.append(self.plan_button)
        self.next_button = ttk.Button(controls, text="Next Candidate Rank", command=self._ui_next_candidate)
        self.next_button.pack(side=tk.LEFT, padx=4)
        self.action_buttons.append(self.next_button)

        ttk.Label(controls, text="Rank").pack(side=tk.LEFT, padx=(20, 4))
        self.rank_var = tk.StringVar(value="1")
        ttk.Entry(controls, width=5, textvariable=self.rank_var).pack(side=tk.LEFT)

        planning = ttk.LabelFrame(top, text="Planning Options", padding=6)
        planning.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(planning, text="Preset").pack(side=tk.LEFT, padx=(0, 4))
        self.planning_preset_var = tk.StringVar(value="current")
        preset = ttk.Combobox(
            planning,
            width=9,
            textvariable=self.planning_preset_var,
            values=("current", "quick", "review", "collect"),
            state="readonly",
        )
        preset.pack(side=tk.LEFT, padx=2)
        preset.bind("<<ComboboxSelected>>", lambda _event: self._ui_apply_planning_preset())

        self.plan_retries_var = tk.StringVar(value=str(int(self.get_parameter("plan_retries_per_grasp_candidate").value)))
        self.planning_attempts_var = tk.StringVar(value=str(int(self.get_parameter("planning_attempts").value)))
        self.allowed_time_var = tk.StringVar(value=f"{float(self.get_parameter('allowed_planning_time').value):.2f}")
        self.action_timeout_var = tk.StringVar(value=f"{float(self.get_parameter('move_group_action_timeout_sec').value):.2f}")
        self.early_exit_var = tk.BooleanVar(value=bool(self.get_parameter("use_excellent_trajectory_early_exit").value))
        self.spin_search_var = tk.BooleanVar(value=bool(self.get_parameter("use_gripper_spin_search").value))
        self.select_best_var = tk.BooleanVar(value=bool(self.get_parameter("select_best_trajectory_candidate").value))
        self.selection_metric_var = tk.StringVar(value=str(self.get_parameter("best_trajectory_selection_metric").value))

        for label, variable, width in (
            ("retries", self.plan_retries_var, 4),
            ("attempts", self.planning_attempts_var, 4),
            ("time", self.allowed_time_var, 5),
            ("timeout", self.action_timeout_var, 5),
        ):
            ttk.Label(planning, text=label).pack(side=tk.LEFT, padx=(10, 2))
            ttk.Entry(planning, width=width, textvariable=variable).pack(side=tk.LEFT)

        ttk.Checkbutton(planning, text="early exit", variable=self.early_exit_var).pack(side=tk.LEFT, padx=(10, 2))
        ttk.Checkbutton(planning, text="spin search", variable=self.spin_search_var).pack(side=tk.LEFT, padx=2)
        ttk.Checkbutton(planning, text="best", variable=self.select_best_var).pack(side=tk.LEFT, padx=2)
        ttk.Label(planning, text="metric").pack(side=tk.LEFT, padx=(10, 2))
        ttk.Combobox(
            planning,
            width=9,
            textvariable=self.selection_metric_var,
            values=("score", "duration", "precision", "learned"),
            state="readonly",
        ).pack(side=tk.LEFT)
        ttk.Button(planning, text="Apply", command=self._ui_apply_planning_options).pack(side=tk.LEFT, padx=(10, 2))

        rail = ttk.Frame(top)
        rail.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(rail, text="Rail / robot_y_offset").pack(side=tk.LEFT, padx=4)
        for label, delta in (("-0.30m", -0.30), ("+0.30m", +0.30), ("-1.00m", -1.00), ("+1.00m", +1.00)):
            button = ttk.Button(rail, text=label, command=lambda value=delta: self._ui_move_rail(value))
            button.pack(side=tk.LEFT, padx=2)
            self.action_buttons.append(button)
        self.rail_var = tk.StringVar(value=f"{self.robot_y_offset:.3f}")
        ttk.Entry(rail, width=8, textvariable=self.rail_var).pack(side=tk.LEFT, padx=(14, 2))
        self.move_to_y_button = ttk.Button(rail, text="Move To Y", command=self._ui_move_rail_absolute)
        self.move_to_y_button.pack(side=tk.LEFT, padx=2)
        self.action_buttons.append(self.move_to_y_button)

        middle = ttk.PanedWindow(top, orient=tk.HORIZONTAL)
        middle.pack(fill=tk.BOTH, expand=True, pady=8)

        left = ttk.Frame(middle, padding=4)
        right = ttk.Frame(middle, padding=4)
        middle.add(left, weight=1)
        middle.add(right, weight=2)

        ttk.Label(left, text="Visible / selectable tomatoes").pack(anchor=tk.W)
        self.tomato_list = tk.Listbox(left, height=22)
        self.tomato_list.pack(fill=tk.BOTH, expand=True)

        label_frame = ttk.Frame(left)
        label_frame.pack(fill=tk.X, pady=8)
        self.save_good_button = ttk.Button(label_frame, text="Save Good", command=lambda: self._ui_save_label("good", 1.0))
        self.save_good_button.pack(side=tk.LEFT, padx=2)
        self.action_buttons.append(self.save_good_button)
        self.save_bad_button = ttk.Button(label_frame, text="Save Bad", command=lambda: self._ui_save_label("bad", 0.0))
        self.save_bad_button.pack(side=tk.LEFT, padx=2)
        self.action_buttons.append(self.save_bad_button)
        ttk.Label(label_frame, text="Score").pack(side=tk.LEFT, padx=(12, 2))
        self.score_var = tk.StringVar(value="5")
        ttk.Entry(label_frame, width=4, textvariable=self.score_var).pack(side=tk.LEFT)
        self.save_score_button = ttk.Button(label_frame, text="Save Score", command=self._ui_save_score)
        self.save_score_button.pack(side=tk.LEFT, padx=2)
        self.action_buttons.append(self.save_score_button)

        candidate_frame = ttk.LabelFrame(right, text="Successful Plans", padding=4)
        candidate_frame.pack(fill=tk.BOTH, expand=True)
        candidate_columns = ("liked", "rank", "score", "duration", "spin", "sign", "points", "retry")
        self.candidate_tree = ttk.Treeview(
            candidate_frame,
            columns=candidate_columns,
            show="headings",
            height=12,
            selectmode="browse",
        )
        for column, label, width in (
            ("liked", "Like", 48),
            ("rank", "Rank", 48),
            ("score", "Score", 70),
            ("duration", "Duration", 76),
            ("spin", "Spin", 72),
            ("sign", "XSign", 56),
            ("points", "Pts", 48),
            ("retry", "Retry", 64),
        ):
            self.candidate_tree.heading(column, text=label)
            self.candidate_tree.column(column, width=width, anchor=tk.CENTER, stretch=False)
        self.candidate_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        candidate_scroll = ttk.Scrollbar(candidate_frame, orient=tk.VERTICAL, command=self.candidate_tree.yview)
        candidate_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.candidate_tree.configure(yscrollcommand=candidate_scroll.set)
        self.candidate_tree.bind("<<TreeviewSelect>>", self._ui_candidate_selected)
        self.candidate_tree.bind("<Button-1>", self._ui_candidate_tree_click)

        candidate_buttons = ttk.Frame(right)
        candidate_buttons.pack(fill=tk.X, pady=(6, 8))
        show_button = ttk.Button(candidate_buttons, text="Show Selected Plan", command=self._ui_show_selected_candidate)
        show_button.pack(side=tk.LEFT, padx=2)
        self.action_buttons.append(show_button)
        toggle_button = ttk.Button(candidate_buttons, text="Toggle Like", command=self._ui_toggle_selected_candidate_like)
        toggle_button.pack(side=tk.LEFT, padx=2)
        self.action_buttons.append(toggle_button)
        save_checked_button = ttk.Button(candidate_buttons, text="Save Checked Good", command=self._ui_save_checked_good)
        save_checked_button.pack(side=tk.LEFT, padx=2)
        self.action_buttons.append(save_checked_button)

        ttk.Label(right, text="Status / sample details").pack(anchor=tk.W)
        self.status_text = tk.Text(right, height=12)
        self.status_text.pack(fill=tk.BOTH, expand=True)

        root.protocol("WM_DELETE_WINDOW", self._ui_close)
        self._set_status("Ready. Start RViz/MoveIt first, then capture visible tomatoes.")
        root.mainloop()

    def _ui_close(self) -> None:
        self._gui_closed = True
        if hasattr(self, "root"):
            self.root.destroy()

    def _set_status(self, text: str, state: str | None = None) -> None:
        timestamp = time.strftime("%H:%M:%S")
        if hasattr(self, "status_var"):
            self.status_var.set(text)
        if state and hasattr(self, "state_var"):
            self.state_var.set(state)
        self.status_text.insert(tk.END, f"[{timestamp}] {text}\n")
        self.status_text.see(tk.END)
        if hasattr(self, "root"):
            self.root.update_idletasks()
        self.get_logger().info(text)

    def _set_busy(self, busy: bool, text: str | None = None, state: str | None = None) -> None:
        if hasattr(self, "action_buttons"):
            button_state = tk.DISABLED if busy else tk.NORMAL
            for button in self.action_buttons:
                button.configure(state=button_state)
        if hasattr(self, "root"):
            self.root.configure(cursor="watch" if busy else "")
        if text:
            self._set_status(text, state=state)
        if hasattr(self, "root"):
            self.root.update()

    def _ui_capture_visible(self) -> None:
        try:
            self._set_busy(True, "Capturing camera-visible tomatoes ...", "CAPTURING")
            dependency_error = self._capture_dependency_error()
            if dependency_error:
                raise RuntimeError(dependency_error)
            if bool(self.get_parameter("sync_scene_on_start").value):
                self._sync_scene_for_current_run()
            entries = self._selected_tomato_entries()
            entries = self._filter_camera_visible_tomato_entries(entries)
            self.visible_entries = entries
            self.latest_record = None
            self.latest_payload = None
            self.latest_valid_records = []
            self.liked_candidate_indices.clear()
            self._populate_candidate_tree()
            self.tomato_list.delete(0, tk.END)
            for entry in entries:
                object_id = self._entry_collision_id(entry)
                pos = entry["position"]
                self.tomato_list.insert(
                    tk.END,
                    f"{object_id} idx={entry['index']} side={entry['side']} "
                    f"row={entry.get('vine_row_index', 0)} "
                    f"xyz=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})",
                )
            self._set_status(f"Capture done: {len(entries)} camera-visible tomato(s).", "CAPTURE DONE")
        except Exception as exc:
            messagebox.showerror("Capture failed", str(exc))
            self._set_status(f"Capture failed: {exc}", "CAPTURE FAILED")
        finally:
            self._set_busy(False)

    def _running_full_node_names(self) -> set[str]:
        names: set[str] = set()
        try:
            for name, namespace in self.get_node_names_and_namespaces():
                namespace = str(namespace or "/")
                if namespace == "/":
                    names.add(f"/{name}")
                else:
                    names.add(f"{namespace.rstrip('/')}/{name}")
                names.add(str(name))
        except Exception:
            return names
        return names

    def _wait_for_node_name(self, node_name: str, timeout_sec: float = 1.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok():
            if node_name in self._running_full_node_names():
                return True
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def _camera_tf_ready(self, timeout_sec: float = 1.0) -> tuple[bool, str]:
        camera_frame = str(self.get_parameter("harvest_camera_frame").value)
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        last_error = ""
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    camera_frame,
                    self.base_frame,
                    Time(seconds=0, nanoseconds=0),
                    timeout=Duration(seconds=0.05),
                )
                return True, ""
            except TransformException as exc:
                last_error = str(exc)
                if time.monotonic() >= deadline:
                    break
                rclpy.spin_once(self, timeout_sec=0.05)
        return False, f"{self.base_frame} -> {camera_frame}: {last_error}"

    def _capture_dependency_error(self) -> str:
        errors: list[str] = []
        camera_timeout = max(2.0, float(self.get_parameter("harvest_camera_visibility_timeout_sec").value))
        if not self._wait_for_node_name("/tomato_scene_node", timeout_sec=min(camera_timeout, 5.0)):
            errors.append("/tomato_scene_node is not running.")
        if bool(self.get_parameter("harvest_camera_visible_only").value):
            ready, reason = self._camera_tf_ready(timeout_sec=camera_timeout)
            if not ready:
                errors.append(f"camera TF is unavailable ({reason}).")
        if not errors:
            return ""
        return (
            "Capture requires the tomato RViz/MoveIt launch to be running first.\n"
            + "\n".join(f"- {error}" for error in errors)
            + "\n\nStart it in another terminal with:\n"
            "  cd /root/pybullet_ur_approach\n"
            "  ./run_tomato_rviz.sh"
        )

    def _selected_entry(self) -> dict[str, object] | None:
        selection = self.tomato_list.curselection()
        if not selection:
            return None
        index = int(selection[0])
        if index < 0 or index >= len(self.visible_entries):
            return None
        return self.visible_entries[index]

    def _set_force_rank(self, rank: int) -> None:
        rank = max(1, int(rank))
        self.current_rank = rank
        self.rank_var.set(str(rank))
        self.set_parameters([Parameter("force_valid_candidate_rank", value=rank)])

    def _candidate_iid(self, index: int) -> str:
        return f"candidate_{index}"

    def _new_planning_request_id(self) -> str:
        self._candidate_id_counter = 0
        self.current_planning_request_id = f"req_{int(time.time() * 1000)}"
        return self.current_planning_request_id

    def _ensure_planning_request_id(self) -> str:
        if not self.current_planning_request_id:
            return self._new_planning_request_id()
        return self.current_planning_request_id

    def _assign_candidate_ids(self, records: list[dict[str, object]]) -> None:
        request_id = self._ensure_planning_request_id()
        for record in records:
            record["planning_request_id"] = request_id
            if not record.get("candidate_id"):
                self._candidate_id_counter += 1
                record["candidate_id"] = f"cand_{self._candidate_id_counter:06d}"

    def _candidate_index_from_iid(self, iid: str) -> int | None:
        if not iid.startswith("candidate_"):
            return None
        try:
            index = int(iid.split("_", 1)[1])
        except ValueError:
            return None
        if index < 0 or index >= len(self.latest_valid_records):
            return None
        return index

    def _format_candidate_row(self, index: int, record: dict[str, object]) -> tuple[str, str, str, str, str, str, str, str]:
        metrics = record.get("trajectory_metrics", {})
        if not isinstance(metrics, dict):
            metrics = {}
        score = metrics.get("score", "?")
        duration = metrics.get("trajectory_duration_sec", "?")
        points = metrics.get("point_count", "?")
        spin = record.get("gripper_spin_deg", "?")
        sign = record.get("gripper_x_sign", "?")
        retry = f"{record.get('plan_retry', '?')}/{record.get('plan_retry_count', '?')}"
        return (
            "[x]" if index in self.liked_candidate_indices else "[ ]",
            str(index + 1),
            f"{float(score):.3f}" if isinstance(score, (int, float)) else str(score),
            f"{float(duration):.3f}s" if isinstance(duration, (int, float)) else str(duration),
            f"{float(spin):.1f}deg" if isinstance(spin, (int, float)) else str(spin),
            f"{float(sign):+.0f}" if isinstance(sign, (int, float)) else str(sign),
            str(points),
            retry,
        )

    def _populate_candidate_tree(self, selected_record: dict[str, object] | None = None) -> None:
        if not hasattr(self, "candidate_tree"):
            return
        self._populating_candidate_tree = True
        try:
            self.candidate_tree.delete(*self.candidate_tree.get_children())
            selected_iid = ""
            for index, record in enumerate(self.latest_valid_records):
                iid = self._candidate_iid(index)
                self.candidate_tree.insert("", tk.END, iid=iid, values=self._format_candidate_row(index, record))
                if selected_record is record:
                    selected_iid = iid
            if selected_iid:
                self.candidate_tree.selection_set(selected_iid)
                self.candidate_tree.focus(selected_iid)
                self.candidate_tree.see(selected_iid)
        finally:
            self._populating_candidate_tree = False

    def _refresh_candidate_row(self, index: int) -> None:
        iid = self._candidate_iid(index)
        if self.candidate_tree.exists(iid):
            self.candidate_tree.item(iid, values=self._format_candidate_row(index, self.latest_valid_records[index]))

    def _live_candidate_status_text(self, index: int, record: dict[str, object]) -> str:
        metrics = record.get("trajectory_metrics", {})
        if not isinstance(metrics, dict):
            metrics = {}
        score = metrics.get("score", "?")
        duration = metrics.get("trajectory_duration_sec", "?")
        spin = record.get("gripper_spin_deg", "?")
        sign = record.get("gripper_x_sign", "?")
        score_text = f"{float(score):.3f}" if isinstance(score, (int, float)) else str(score)
        duration_text = f"{float(duration):.3f}s" if isinstance(duration, (int, float)) else str(duration)
        spin_text = f"{float(spin):.1f}deg" if isinstance(spin, (int, float)) else str(spin)
        sign_text = f"{float(sign):+.0f}" if isinstance(sign, (int, float)) else str(sign)
        return (
            f"Found valid plan rank~{index + 1}: "
            f"score={score_text} duration={duration_text} spin={spin_text} xsign={sign_text}"
        )

    def _append_live_candidate_record(self, record: dict[str, object], valid_records: list[dict[str, object]]) -> None:
        if not hasattr(self, "candidate_tree"):
            return
        valid_records = list(valid_records)
        self._assign_candidate_ids(valid_records)
        if any(existing is record for existing in self.latest_valid_records):
            return
        can_append = (
            len(valid_records) == len(self.latest_valid_records) + 1
            and valid_records[-1] is record
            and all(existing is valid_records[index] for index, existing in enumerate(self.latest_valid_records))
        )
        if can_append:
            index = len(self.latest_valid_records)
            self.latest_valid_records.append(record)
            iid = self._candidate_iid(index)
            self.candidate_tree.insert("", tk.END, iid=iid, values=self._format_candidate_row(index, record))
            self.candidate_tree.see(iid)
        else:
            self.latest_valid_records = valid_records
            index = next((candidate_index for candidate_index, candidate in enumerate(valid_records) if candidate is record), 0)
            self._populate_candidate_tree(record)
        self.latest_payload = None
        text = self._live_candidate_status_text(index, record)
        if hasattr(self, "status_var"):
            self.status_var.set(text)
        if hasattr(self, "state_var"):
            self.state_var.set("PLANNING")
        if hasattr(self, "status_text"):
            timestamp = time.strftime("%H:%M:%S")
            self.status_text.insert(tk.END, f"[{timestamp}] {text}\n")
            self.status_text.see(tk.END)
        if hasattr(self, "root"):
            self.root.update_idletasks()

    def _on_preplan_valid_candidate_record(
        self,
        record: dict[str, object],
        valid_records: list[dict[str, object]],
    ) -> None:
        self._append_live_candidate_record(record, valid_records)

    def _selected_candidate_index(self) -> int | None:
        selection = self.candidate_tree.selection()
        if not selection:
            return None
        return self._candidate_index_from_iid(str(selection[0]))

    def _show_candidate_record(self, index: int) -> None:
        record = self.latest_valid_records[index]
        self.latest_record = record
        self.current_rank = index + 1
        self.rank_var.set(str(self.current_rank))
        self.latest_payload = None
        plan_result = record.get("plan_result", {})
        published = False
        if isinstance(plan_result, dict):
            published = self._publish_display_trajectory(
                plan_result.get("trajectory"),
                plan_result.get("trajectory_start"),
            )
        metrics = record.get("trajectory_metrics", {})
        score = metrics.get("score", "?") if isinstance(metrics, dict) else "?"
        duration = metrics.get("trajectory_duration_sec", "?") if isinstance(metrics, dict) else "?"
        rviz_status = "RViz path updated" if published else "RViz path was not updated"
        self._set_status(
            f"Showing candidate rank={self.current_rank} score={score} duration={duration}. {rviz_status}.",
            "PLAN SHOWN",
        )

    def _ui_candidate_selected(self, _event=None) -> None:
        if self._gui_closed or self._populating_candidate_tree:
            return
        index = self._selected_candidate_index()
        if index is not None:
            self._show_candidate_record(index)

    def _ui_candidate_tree_click(self, event) -> None:
        region = self.candidate_tree.identify("region", event.x, event.y)
        if region != "cell":
            return
        column = self.candidate_tree.identify_column(event.x)
        row = self.candidate_tree.identify_row(event.y)
        if column != "#1" or not row:
            return
        index = self._candidate_index_from_iid(row)
        if index is None:
            return
        self._toggle_candidate_like(index)
        return "break"

    def _toggle_candidate_like(self, index: int) -> None:
        if index in self.liked_candidate_indices:
            self.liked_candidate_indices.remove(index)
        else:
            self.liked_candidate_indices.add(index)
        self._refresh_candidate_row(index)
        self._set_status(
            f"Candidate rank={index + 1} like={'on' if index in self.liked_candidate_indices else 'off'}.",
            "LIKE UPDATED",
        )

    def _ui_show_selected_candidate(self) -> None:
        if self._gui_closed:
            return
        index = self._selected_candidate_index()
        if index is None:
            messagebox.showwarning("No plan selected", "Select a successful plan first.")
            return
        self._show_candidate_record(index)

    def _ui_toggle_selected_candidate_like(self) -> None:
        index = self._selected_candidate_index()
        if index is None:
            messagebox.showwarning("No plan selected", "Select a successful plan first.")
            return
        self._toggle_candidate_like(index)

    def _planning_option_values(self) -> dict[str, object]:
        return {
            "plan_retries_per_grasp_candidate": max(1, int(self.plan_retries_var.get())),
            "planning_attempts": max(1, int(self.planning_attempts_var.get())),
            "allowed_planning_time": max(0.05, float(self.allowed_time_var.get())),
            "move_group_action_timeout_sec": max(0.1, float(self.action_timeout_var.get())),
            "use_excellent_trajectory_early_exit": bool(self.early_exit_var.get()),
            "use_gripper_spin_search": bool(self.spin_search_var.get()),
            "select_best_trajectory_candidate": bool(self.select_best_var.get()),
            "best_trajectory_selection_metric": str(self.selection_metric_var.get()),
        }

    def _set_planning_option_vars(self, values: dict[str, object]) -> None:
        self.plan_retries_var.set(str(int(values["plan_retries_per_grasp_candidate"])))
        self.planning_attempts_var.set(str(int(values["planning_attempts"])))
        self.allowed_time_var.set(f"{float(values['allowed_planning_time']):.2f}")
        self.action_timeout_var.set(f"{float(values['move_group_action_timeout_sec']):.2f}")
        self.early_exit_var.set(bool(values["use_excellent_trajectory_early_exit"]))
        self.spin_search_var.set(bool(values["use_gripper_spin_search"]))
        self.select_best_var.set(bool(values["select_best_trajectory_candidate"]))
        self.selection_metric_var.set(str(values["best_trajectory_selection_metric"]))

    def _ui_apply_planning_preset(self) -> None:
        preset = self.planning_preset_var.get()
        if preset == "current":
            self._ui_apply_planning_options()
            return
        presets: dict[str, dict[str, object]] = {
            "quick": {
                "plan_retries_per_grasp_candidate": 1,
                "planning_attempts": 1,
                "allowed_planning_time": 0.5,
                "move_group_action_timeout_sec": 5.0,
                "use_excellent_trajectory_early_exit": True,
                "use_gripper_spin_search": False,
                "select_best_trajectory_candidate": True,
                "best_trajectory_selection_metric": "duration",
            },
            "review": {
                "plan_retries_per_grasp_candidate": 4,
                "planning_attempts": 1,
                "allowed_planning_time": 0.8,
                "move_group_action_timeout_sec": 8.0,
                "use_excellent_trajectory_early_exit": False,
                "use_gripper_spin_search": False,
                "select_best_trajectory_candidate": True,
                "best_trajectory_selection_metric": "duration",
            },
            "collect": {
                "plan_retries_per_grasp_candidate": 10,
                "planning_attempts": 1,
                "allowed_planning_time": 1.2,
                "move_group_action_timeout_sec": 12.0,
                "use_excellent_trajectory_early_exit": False,
                "use_gripper_spin_search": True,
                "select_best_trajectory_candidate": True,
                "best_trajectory_selection_metric": "score",
            },
        }
        self._set_planning_option_vars(presets[preset])
        self._ui_apply_planning_options()

    def _ui_apply_planning_options(self) -> bool:
        try:
            values = self._planning_option_values()
            self.set_parameters(
                [
                    Parameter("plan_retries_per_grasp_candidate", value=int(values["plan_retries_per_grasp_candidate"])),
                    Parameter("planning_attempts", value=int(values["planning_attempts"])),
                    Parameter("allowed_planning_time", value=float(values["allowed_planning_time"])),
                    Parameter("move_group_action_timeout_sec", value=float(values["move_group_action_timeout_sec"])),
                    Parameter(
                        "use_excellent_trajectory_early_exit",
                        value=bool(values["use_excellent_trajectory_early_exit"]),
                    ),
                    Parameter("use_gripper_spin_search", value=bool(values["use_gripper_spin_search"])),
                    Parameter("select_best_trajectory_candidate", value=bool(values["select_best_trajectory_candidate"])),
                    Parameter("best_trajectory_selection_metric", value=str(values["best_trajectory_selection_metric"])),
                ]
            )
            self._set_status(
                "Planning options applied: "
                f"preset={self.planning_preset_var.get()} "
                f"retries={values['plan_retries_per_grasp_candidate']} "
                f"attempts={values['planning_attempts']} "
                f"time={values['allowed_planning_time']:.2f}s "
                f"timeout={values['move_group_action_timeout_sec']:.2f}s "
                f"early_exit={values['use_excellent_trajectory_early_exit']} "
                f"spin_search={values['use_gripper_spin_search']} "
                f"metric={values['best_trajectory_selection_metric']}",
                "OPTIONS SET",
            )
            return True
        except Exception as exc:
            messagebox.showerror("Invalid planning options", str(exc))
            self._set_status(f"Invalid planning options: {exc}", "OPTION ERROR")
            return False

    def _ui_plan_selected(self) -> None:
        entry = self._selected_entry()
        if entry is None:
            messagebox.showwarning("No tomato selected", "Capture and select a tomato first.")
            return
        try:
            self._set_force_rank(int(self.rank_var.get()))
            if not self._ui_apply_planning_options():
                return
            object_id = self._entry_collision_id(entry)
            request_id = self._new_planning_request_id()
            self.latest_record = None
            self.latest_payload = None
            self.latest_valid_records = []
            self.liked_candidate_indices.clear()
            self._populate_candidate_tree()
            self._set_busy(
                True,
                f"Planning {object_id} request={request_id} with forced rank={self.current_rank}; "
                "wait until PLAN DONE/FAILED.",
                "PLANNING",
            )
            record = self._preplan_grasp_record_for_entry(
                entry,
                object_id,
                use_rule_grasp=True,
                relative_transform=None,
            )
            valid_records = list(getattr(self, "_preplan_last_valid_records", []))
            self._assign_candidate_ids(valid_records)
            if record is None:
                self.latest_record = None
                self.latest_payload = None
                self.latest_valid_records = valid_records
                self.liked_candidate_indices.clear()
                self._populate_candidate_tree()
                if valid_records:
                    self._set_status(
                        f"Plan failed to select rank={self.current_rank}, but {len(valid_records)} valid plan(s) were found.",
                        "RANK FAILED",
                    )
                else:
                    self._set_status(f"Plan failed: no valid candidate for {object_id} rank={self.current_rank}.", "PLAN FAILED")
                return
            self.latest_record = record
            if not valid_records:
                valid_records = [record]
                self._assign_candidate_ids(valid_records)
            self.latest_valid_records = valid_records
            self.liked_candidate_indices.clear()
            payload = self._diagnostics_payload_for_labeler(record, valid_records, "gui_selected")
            self.latest_payload = payload
            plan_result = record.get("plan_result", {})
            published = False
            if isinstance(plan_result, dict):
                published = self._publish_display_trajectory(
                    plan_result.get("trajectory"),
                    plan_result.get("trajectory_start"),
                )
            selected_index = next((index for index, candidate in enumerate(valid_records) if candidate is record), None)
            if selected_index is not None:
                self.current_rank = selected_index + 1
                self.rank_var.set(str(self.current_rank))
            self._populate_candidate_tree(record)
            metrics = record.get("trajectory_metrics", {})
            if isinstance(metrics, dict):
                duration = metrics.get("trajectory_duration_sec", "?")
                score = metrics.get("score", "?")
                points = metrics.get("point_count", "?")
                planning_time = metrics.get("planning_time", "?")
            else:
                duration = "?"
                score = "?"
                points = "?"
                planning_time = "?"
            self._set_status(
                f"PLAN DONE: {object_id} valid_plans={len(valid_records)} selected_rank={self.current_rank} "
                f"duration={duration} score={score} points={points} planning_time={planning_time}. "
                f"{'RViz path updated' if published else 'RViz path was not updated'}.",
                "PLAN DONE",
            )
        except Exception as exc:
            messagebox.showerror("Plan failed", str(exc))
            self._set_status(f"Plan failed: {exc}", "PLAN FAILED")
        finally:
            self._set_busy(False)

    def _ui_next_candidate(self) -> None:
        try:
            self._set_force_rank(int(self.rank_var.get()) + 1)
        except ValueError:
            self._set_force_rank(self.current_rank + 1)
        self._ui_plan_selected()

    def _diagnostics_payload_for_labeler(
        self,
        selected_record: dict[str, object],
        valid_records: list[dict[str, object]],
        selection_reason: str,
    ) -> dict[str, object]:
        self._assign_candidate_ids(valid_records)
        selected_diagnostics = self._record_diagnostics(selected_record)
        selected_diagnostics["planning_request_id"] = selected_record.get("planning_request_id", self.current_planning_request_id)
        selected_diagnostics["candidate_id"] = selected_record.get("candidate_id", "")
        valid_candidate_diagnostics = []
        for record in valid_records:
            diagnostics = self._record_diagnostics(record)
            diagnostics["planning_request_id"] = record.get("planning_request_id", self.current_planning_request_id)
            diagnostics["candidate_id"] = record.get("candidate_id", "")
            valid_candidate_diagnostics.append(diagnostics)
        return {
            "stamp_unix": time.time(),
            "planning_request_id": self.current_planning_request_id,
            "selection_reason": selection_reason,
            "side": self.side,
            "tomato_z_spin_deg": self.tomato_z_spin_deg,
            "target_tomato_index": int(self.get_parameter("target_tomato_index").value),
            "joint_plan_goal_pose": str(self.get_parameter("joint_plan_goal_pose").value),
            "valid_candidate_count": len(valid_records),
            "visible_tomatoes": [self._entry_collision_id(entry) for entry in self.visible_entries],
            "perception_observation": self._perception_observation_for_labeler(selected_record),
            "selected": selected_diagnostics,
            "valid_candidates": valid_candidate_diagnostics,
        }

    @staticmethod
    def _rpy_from_rotation(rotation: np.ndarray) -> tuple[float, float, float]:
        sy = math.sqrt(float(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0]))
        singular = sy < 1e-9
        if not singular:
            roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
            pitch = math.atan2(float(-rotation[2, 0]), sy)
            yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
        else:
            roll = math.atan2(float(-rotation[1, 2]), float(rotation[1, 1]))
            pitch = math.atan2(float(-rotation[2, 0]), sy)
            yaw = 0.0
        return roll, pitch, yaw

    @staticmethod
    def _xyz_list(position: np.ndarray) -> list[float]:
        return [float(position[0]), float(position[1]), float(position[2])]

    def _pose_dict_from_matrix(self, matrix: np.ndarray) -> dict[str, object]:
        rotation = np.array(matrix[:3, :3], dtype=float)
        roll, pitch, yaw = self._rpy_from_rotation(rotation)
        return {
            "xyz": self._xyz_list(np.array(matrix[:3, 3], dtype=float)),
            "rpy_rad": [float(roll), float(pitch), float(yaw)],
        }

    def _pose_dict_from_msg(self, pose) -> dict[str, object]:
        matrix = np.eye(4)
        matrix[:3, 3] = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
        matrix[:3, :3] = matrix_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        return self._pose_dict_from_matrix(matrix)

    @staticmethod
    def _rotation_from_z_axis(z_axis: np.ndarray) -> np.ndarray:
        z = np.array(z_axis, dtype=float)
        norm = float(np.linalg.norm(z))
        if norm < 1e-9:
            z = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            z /= norm
        x_hint = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(z, x_hint))) > 0.95:
            x_hint = np.array([0.0, 1.0, 0.0], dtype=float)
        y = np.cross(z, x_hint)
        y /= np.linalg.norm(y)
        x = np.cross(y, z)
        x /= np.linalg.norm(x)
        return np.column_stack((x, y, z))

    def _stem_pose_matrix_for_entry(self, entry: dict[str, object], calyx_position: np.ndarray) -> np.ndarray:
        branch_start = np.array(entry.get("branch_start", calyx_position), dtype=float)
        stem_axis = calyx_position - branch_start
        if float(np.linalg.norm(stem_axis)) < 1e-9:
            stem_axis = np.array(entry.get("stem_axis", [0.0, 0.0, 1.0]), dtype=float)
        matrix = np.eye(4)
        matrix[:3, :3] = self._rotation_from_z_axis(stem_axis)
        matrix[:3, 3] = branch_start
        return matrix

    def _perceived_tomato_entry(self, entry: dict[str, object]) -> dict[str, object]:
        tomato_matrix = self._base_to_tomato_transform(entry)
        tomato_position = tomato_matrix[:3, 3]
        tomato_z = tomato_matrix[:3, 2]
        calyx_position = tomato_position + tomato_z * self._tomato_contact_half_span()
        observation = {
            "object_id": self._entry_collision_id(entry),
            "index": int(entry["index"]),
            "tomato_pose_base": self._pose_dict_from_matrix(tomato_matrix),
            "calyx_position_base": self._xyz_list(calyx_position),
            "stem_pose_base": self._pose_dict_from_matrix(
                self._stem_pose_matrix_for_entry(entry, calyx_position)
            ),
        }
        return observation

    def _perception_observation_for_labeler(self, selected_record: dict[str, object]) -> dict[str, object]:
        selected_entry = selected_record.get("entry", {})

        selected_object_id = ""
        if isinstance(selected_entry, dict) and selected_entry:
            selected_object_id = self._entry_collision_id(selected_entry)

        selected_visible_entry = selected_entry
        for entry in self.visible_entries:
            if isinstance(entry, dict) and self._entry_collision_id(entry) == selected_object_id:
                selected_visible_entry = entry
                break

        selected_grasp_pose = {}
        target_pose = selected_record.get("target_pose")
        if target_pose is not None:
            selected_grasp_pose = self._pose_dict_from_msg(target_pose)

        return {
            "frame": self.base_frame,
            "selected_object_id": selected_object_id,
            "selected_tomato": self._perceived_tomato_entry(selected_visible_entry)
            if isinstance(selected_visible_entry, dict) and selected_visible_entry
            else {},
            "selected_grasp_pose_base": selected_grasp_pose,
        }

    @staticmethod
    def _label_save_summary(sample: dict[str, object], dataset_path: str) -> str:
        candidate = sample.get("candidate", {})
        trajectory = candidate.get("trajectory", {}) if isinstance(candidate, dict) else {}
        points = trajectory.get("points", []) if isinstance(trajectory, dict) else []
        point_count = len(points) if isinstance(points, list) else 0
        return (
            f"Saved sample_id={sample.get('sample_id')} "
            f"request={sample.get('planning_request_id')} "
            f"candidate={sample.get('candidate_id')} "
            f"label={sample.get('label')} score={sample.get('human_score')} "
            f"points={point_count} -> {dataset_path}"
        )

    def _ui_save_label(self, label: str, human_score: float) -> None:
        if self.latest_record is None:
            messagebox.showwarning("No plan", "Plan a selected tomato before saving a label.")
            return
        try:
            dataset_path = str(self.get_parameter("trajectory_label_dataset_path").value)
            if self.latest_payload is None:
                valid_records = self.latest_valid_records or [self.latest_record]
                self.latest_payload = self._diagnostics_payload_for_labeler(
                    self.latest_record,
                    valid_records,
                    f"gui_candidate_rank_{self.current_rank}",
                )
            sample = make_label_sample(
                self.latest_payload,
                self.latest_payload["selected"],
                label=label,
                human_score=human_score,
                note="gui_label",
                candidate_rank=self.current_rank,
            )
            append_jsonl(dataset_path, sample)
            self._set_status(self._label_save_summary(sample, dataset_path), "SAVED")
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))
            self._set_status(f"Save failed: {exc}", "SAVE FAILED")

    def _ui_save_score(self) -> None:
        try:
            score = float(self.score_var.get())
        except ValueError:
            messagebox.showwarning("Invalid score", "Score must be numeric.")
            return
        normalized = max(0.0, min(1.0, score / 5.0))
        label = "good" if normalized >= 0.5 else "bad"
        self._ui_save_label(label, normalized)

    def _ui_save_checked_good(self) -> None:
        if not self.latest_valid_records:
            messagebox.showwarning("No plans", "Plan a selected tomato before saving checked candidates.")
            return
        if not self.liked_candidate_indices:
            messagebox.showwarning("No checked plans", "Check one or more successful plans first.")
            return
        try:
            dataset_path = str(self.get_parameter("trajectory_label_dataset_path").value)
            saved = 0
            for index in sorted(self.liked_candidate_indices):
                if index < 0 or index >= len(self.latest_valid_records):
                    continue
                record = self.latest_valid_records[index]
                payload = self._diagnostics_payload_for_labeler(
                    record,
                    self.latest_valid_records,
                    "gui_checked_good",
                )
                sample = make_label_sample(
                    payload,
                    payload["selected"],
                    label="good",
                    human_score=1.0,
                    note="gui_checked_good",
                    candidate_rank=index + 1,
                )
                append_jsonl(dataset_path, sample)
                saved += 1
                self._set_status(self._label_save_summary(sample, dataset_path), "SAVED")
            self._set_status(f"Saved {saved} checked good plan(s).", "SAVED")
        except Exception as exc:
            messagebox.showerror("Save checked failed", str(exc))
            self._set_status(f"Save checked failed: {exc}", "SAVE FAILED")

    def _ui_move_rail(self, delta_y: float) -> None:
        try:
            self._set_busy(True, f"Moving robot_y_offset by {delta_y:+.3f}m ...", "MOVING RAIL")
            self.robot_y_offset += float(delta_y)
            self.rail_var.set(f"{self.robot_y_offset:.3f}")
            self._set_mobile_base_robot_offset()
            self._set_status(f"Moved robot_y_offset by {delta_y:+.3f}m -> {self.robot_y_offset:.3f}m", "RAIL DONE")
        except Exception as exc:
            messagebox.showerror("Rail move failed", str(exc))
            self._set_status(f"Rail move failed: {exc}", "RAIL FAILED")
        finally:
            self._set_busy(False)

    def _ui_move_rail_absolute(self) -> None:
        try:
            self.robot_y_offset = float(self.rail_var.get())
        except ValueError:
            messagebox.showwarning("Invalid rail offset", "Rail offset must be numeric.")
            return
        try:
            self._set_busy(True, f"Moving robot_y_offset -> {self.robot_y_offset:.3f}m ...", "MOVING RAIL")
            self.rail_var.set(f"{self.robot_y_offset:.3f}")
            self._set_mobile_base_robot_offset()
            self._set_status(f"Moved robot_y_offset -> {self.robot_y_offset:.3f}m", "RAIL DONE")
        except Exception as exc:
            messagebox.showerror("Rail move failed", str(exc))
            self._set_status(f"Rail move failed: {exc}", "RAIL FAILED")
        finally:
            self._set_busy(False)


def main() -> None:
    if any(arg in {"-h", "--help"} for arg in sys.argv[1:]):
        print(
            "usage: trajectory_labeler_gui [--ros-args -p name:=value ...]\n\n"
            "Tk GUI for capturing camera-visible tomatoes, planning C_pre joint trajectories, "
            "and saving Good/Bad/Score trajectory labels."
        )
        return
    rclpy.init()
    node = TrajectoryLabelerGui()
    try:
        node.run_gui()
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
