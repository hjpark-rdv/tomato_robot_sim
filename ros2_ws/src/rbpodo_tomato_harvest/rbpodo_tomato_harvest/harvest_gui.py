import os
import queue
import signal
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk

import rclpy
from farmily_tomato_interfaces.msg import TomatoDetectionArray
from farmily_tomato_interfaces.srv import DetectTomatoes
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


PLANNER_CONFIGS = {
    "OMPL / RRTConnect": ("ompl", "RRTConnectkConfigDefault"),
    "CHOMP": ("chomp", "RRTConnectkConfigDefault"),
    "PILZ / LIN": ("pilz_industrial_motion_planner", "LIN"),
}


def harvest_command(
    tomato_index: int,
    execute: bool,
    planning_pipeline_id: str = "ompl",
    planner_id: str = "RRTConnectkConfigDefault",
    python_executable: str | None = None,
) -> list[str]:
    """Build the isolated harvest planner command used by the GUI."""
    if tomato_index < 0:
        raise ValueError("tomato_index must be zero or greater")
    planner_config = (planning_pipeline_id, planner_id)
    if planner_config not in PLANNER_CONFIGS.values():
        raise ValueError(
            f"unsupported planner: pipeline={planning_pipeline_id} "
            f"planner_id={planner_id}"
        )
    executable = python_executable or sys.executable
    return [
        executable,
        "-m",
        "rbpodo_tomato_harvest.tomato_harvest_test",
        "--ros-args",
        "-p",
        f"tomato_frame:=detected_tomato_{tomato_index}_tf",
        "-p",
        f"execute:={'true' if execute else 'false'}",
        "-p",
        f"planning_pipeline_id:={planning_pipeline_id}",
        "-p",
        f"planner_id:={planner_id}",
    ]


def scene_parameters(position, rotation_deg: float) -> list[Parameter]:
    """Build scene position and main-vine-axis rotation parameters."""
    coordinates = [float(value) for value in position]
    if len(coordinates) != 3:
        raise ValueError("position must contain exactly three values")
    position_value = ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
        double_array_value=coordinates,
    )
    rotation_value = ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE,
        double_value=float(rotation_deg),
    )
    return [
        Parameter(name="object_position", value=position_value),
        Parameter(name="tomato_z_spin_deg", value=rotation_value),
    ]


class HarvestGui(Node):
    """Tkinter operator panel for tomato detection and harvest testing."""

    def __init__(self) -> None:
        super().__init__("tomato_harvest_gui")
        self.declare_parameter(
            "camera_service", "/fake_tomato_camera/detect_tomatoes"
        )
        self.declare_parameter(
            "detections_topic", "/tomato_detection/detections"
        )
        self.declare_parameter("scene_node", "/tomato_scene_node")
        self.declare_parameter("default_planner", "ompl")

        camera_service = str(self.get_parameter("camera_service").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        scene_node = str(self.get_parameter("scene_node").value).rstrip("/")
        self.camera_client = self.create_client(DetectTomatoes, camera_service)
        self.scene_get_client = self.create_client(
            GetParameters, f"{scene_node}/get_parameters"
        )
        self.scene_set_client = self.create_client(
            SetParameters, f"{scene_node}/set_parameters"
        )
        self.create_subscription(
            TomatoDetectionArray,
            detections_topic,
            self._detections_callback,
            10,
        )

        self.camera_service = camera_service
        self.scene_node = scene_node
        self.detected_tomatoes = []
        self.detection_signature = None
        self.detection_generation = 0
        self.verified_plan = None
        self.harvest_process = None
        self.process_queue = queue.Queue()
        self.closing = False

        self.root = tk.Tk()
        self.root.title("Farmily Tomato Harvest")
        self.root.minsize(820, 650)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        signal.signal(signal.SIGINT, self._signal_close)
        signal.signal(signal.SIGTERM, self._signal_close)
        self._configure_korean_font()

        self.selected_tomato = tk.StringVar(value="")
        default_planner = str(self.get_parameter("default_planner").value)
        planner_label = next(
            (
                label
                for label, config in PLANNER_CONFIGS.items()
                if config[0] == default_planner
            ),
            "OMPL / RRTConnect",
        )
        self.selected_planner = tk.StringVar(value=planner_label)
        self.scene_x = tk.StringVar(value="0.355")
        self.scene_y = tk.StringVar(value="-0.375")
        self.scene_z = tk.StringVar(value="0.340")
        self.scene_rotation = tk.StringVar(value="45.0")
        self.status = tk.StringVar(value="MoveIt과 카메라 서비스를 확인해 주세요.")
        self._build_ui()
        self.root.after(50, self._spin_ros)
        self.root.after(50, self._drain_process_queue)
        self.root.after(400, self.read_scene_position)

    def _configure_korean_font(self) -> None:
        available = set(tkfont.families(self.root))
        preferred = (
            "Noto Sans CJK KR",
            "NanumGothic",
            "Droid Sans Fallback",
        )
        family = next((name for name in preferred if name in available), None)
        if family is None:
            return
        for name in (
            "TkDefaultFont",
            "TkTextFont",
            "TkFixedFont",
            "TkMenuFont",
            "TkHeadingFont",
            "TkCaptionFont",
            "TkSmallCaptionFont",
            "TkIconFont",
            "TkTooltipFont",
        ):
            try:
                tkfont.nametofont(name, self.root).configure(family=family)
            except tk.TclError:
                pass

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=14)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(1, weight=1)
        outer.rowconfigure(4, weight=1)

        camera_frame = ttk.LabelFrame(outer, text="1. 카메라 검출", padding=10)
        camera_frame.grid(row=0, column=0, sticky="ew")
        camera_frame.columnconfigure(1, weight=1)
        ttk.Label(camera_frame, text="서비스").grid(row=0, column=0, sticky="w")
        ttk.Label(camera_frame, text=self.camera_service).grid(
            row=0, column=1, sticky="w", padx=8
        )
        self.detect_button = ttk.Button(
            camera_frame,
            text="토마토 촬영 / 검출",
            command=self.detect_tomatoes,
        )
        self.detect_button.grid(row=0, column=2, padx=(8, 0))

        list_frame = ttk.LabelFrame(outer, text="2. 검출된 토마토 선택", padding=10)
        list_frame.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        list_frame.columnconfigure(0, weight=1)
        list_frame.rowconfigure(0, weight=1)
        columns = ("index", "frame", "source_id", "x", "y", "z")
        self.tomato_tree = ttk.Treeview(
            list_frame,
            columns=columns,
            show="headings",
            height=8,
            selectmode="browse",
        )
        headings = {
            "index": "번호",
            "frame": "수확 TF",
            "source_id": "카메라 ID",
            "x": "Camera X (m)",
            "y": "Camera Y (m)",
            "z": "Camera Z (m)",
        }
        widths = {
            "index": 52,
            "frame": 180,
            "source_id": 110,
            "x": 100,
            "y": 100,
            "z": 100,
        }
        for column in columns:
            self.tomato_tree.heading(column, text=headings[column])
            self.tomato_tree.column(
                column,
                width=widths[column],
                anchor="center",
                stretch=column in ("frame", "source_id"),
            )
        scrollbar = ttk.Scrollbar(
            list_frame, orient="vertical", command=self.tomato_tree.yview
        )
        self.tomato_tree.configure(yscrollcommand=scrollbar.set)
        self.tomato_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.tomato_tree.bind("<<TreeviewSelect>>", self._tree_selection_changed)

        motion_frame = ttk.LabelFrame(outer, text="3. 수확 모션", padding=10)
        motion_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        motion_frame.columnconfigure(1, weight=1)
        ttk.Label(motion_frame, text="선택 토마토").grid(
            row=0, column=0, sticky="w"
        )
        self.tomato_combo = ttk.Combobox(
            motion_frame,
            textvariable=self.selected_tomato,
            state="readonly",
            width=28,
        )
        self.tomato_combo.grid(row=0, column=1, sticky="w", padx=8)
        self.tomato_combo.bind("<<ComboboxSelected>>", self._combo_selection_changed)
        self.plan_button = ttk.Button(
            motion_frame,
            text="Plan-only 확인",
            command=lambda: self.start_harvest(False),
            state="disabled",
        )
        self.plan_button.grid(row=0, column=2, padx=(8, 4))
        self.execute_button = ttk.Button(
            motion_frame,
            text="실제 수확 실행",
            command=lambda: self.start_harvest(True),
            state="disabled",
        )
        self.execute_button.grid(row=0, column=3, padx=(4, 0))
        ttk.Label(motion_frame, text="플래너").grid(
            row=1, column=0, sticky="w", pady=(8, 0)
        )
        self.planner_combo = ttk.Combobox(
            motion_frame,
            textvariable=self.selected_planner,
            values=list(PLANNER_CONFIGS),
            state="readonly",
            width=28,
        )
        self.planner_combo.grid(
            row=1, column=1, sticky="w", padx=8, pady=(8, 0)
        )
        self.planner_combo.bind(
            "<<ComboboxSelected>>", self._planner_selection_changed
        )
        ttk.Label(
            motion_frame,
            text="실제 실행은 현재 선택한 토마토의 Plan-only 성공 후 활성화됩니다.",
        ).grid(row=2, column=0, columnspan=4, sticky="w", pady=(8, 0))

        scene_frame = ttk.LabelFrame(
            outer, text="토마토 줄기 위치 / 회전", padding=10
        )
        scene_frame.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        for column in range(10):
            scene_frame.columnconfigure(column, weight=0)
        for column, (label, variable) in enumerate(
            (("X", self.scene_x), ("Y", self.scene_y), ("Z", self.scene_z))
        ):
            base = column * 2
            ttk.Label(scene_frame, text=label).grid(row=0, column=base, padx=(0, 4))
            ttk.Entry(scene_frame, textvariable=variable, width=10).grid(
                row=0, column=base + 1, padx=(0, 10)
            )
        ttk.Label(scene_frame, text="회전(°)").grid(
            row=0, column=6, padx=(0, 4)
        )
        ttk.Spinbox(
            scene_frame,
            from_=-360.0,
            to=360.0,
            increment=5.0,
            textvariable=self.scene_rotation,
            width=8,
        ).grid(row=0, column=7, padx=(0, 10))
        self.read_scene_button = ttk.Button(
            scene_frame,
            text="현재값 읽기",
            command=self.read_scene_position,
        )
        self.read_scene_button.grid(row=0, column=8, padx=(4, 4))
        self.set_scene_button = ttk.Button(
            scene_frame,
            text="위치/회전 적용",
            command=self.set_scene_position,
        )
        self.set_scene_button.grid(row=0, column=9, padx=(4, 0))
        ttk.Label(
            scene_frame,
            text="회전은 메인 줄기 축을 기준으로 가지와 토마토 전체에 적용됩니다.",
        ).grid(row=1, column=0, columnspan=10, sticky="w", pady=(8, 0))

        log_frame = ttk.LabelFrame(outer, text="상태 및 실행 로그", padding=10)
        log_frame.grid(row=4, column=0, sticky="nsew", pady=(10, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, height=10, wrap="word", state="disabled")
        log_scrollbar = ttk.Scrollbar(
            log_frame, orient="vertical", command=self.log_text.yview
        )
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_scrollbar.grid(row=0, column=1, sticky="ns")
        ttk.Label(outer, textvariable=self.status, anchor="w").grid(
            row=5, column=0, sticky="ew", pady=(8, 0)
        )

    def _append_log(self, message: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message.rstrip() + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _detection_key(self, message: TomatoDetectionArray):
        stamp = message.header.stamp
        points = tuple(
            (
                detection.id,
                detection.center.x,
                detection.center.y,
                detection.center.z,
                detection.stem_point.x,
                detection.stem_point.y,
                detection.stem_point.z,
            )
            for detection in message.detections
        )
        return (stamp.sec, stamp.nanosec, message.header.frame_id, points)

    def _detections_callback(self, message: TomatoDetectionArray) -> None:
        signature = self._detection_key(message)
        if signature == self.detection_signature:
            return
        self.detection_signature = signature
        self.detected_tomatoes = list(message.detections)
        self.detection_generation += 1
        self._invalidate_plan()

        for item in self.tomato_tree.get_children():
            self.tomato_tree.delete(item)
        choices = []
        for index, detection in enumerate(self.detected_tomatoes):
            frame = f"detected_tomato_{index}_tf"
            choices.append(f"{index}: {frame}")
            center = detection.center
            self.tomato_tree.insert(
                "",
                "end",
                iid=str(index),
                values=(
                    index,
                    frame,
                    detection.id,
                    f"{center.x:.4f}",
                    f"{center.y:.4f}",
                    f"{center.z:.4f}",
                ),
            )
        self.tomato_combo.configure(values=choices)
        if choices:
            self.tomato_combo.current(0)
            self.tomato_tree.selection_set("0")
            self.tomato_tree.focus("0")
            self.plan_button.configure(state="normal")
        else:
            self.selected_tomato.set("")
            self.plan_button.configure(state="disabled")
        self.status.set(f"토마토 {len(choices)}개 검출됨")
        self._append_log(
            f"[{message.header.frame_id}] 새 검출 결과: 토마토 {len(choices)}개"
        )

    def detect_tomatoes(self) -> None:
        if not self.camera_client.service_is_ready():
            if not self.camera_client.wait_for_service(timeout_sec=0.05):
                self.status.set(f"카메라 서비스 연결 안 됨: {self.camera_service}")
                self._append_log(
                    f"카메라 서비스를 먼저 실행하세요: {self.camera_service}"
                )
                return
        self.detect_button.configure(state="disabled")
        self.status.set("카메라 촬영 및 토마토 검출 요청 중...")
        self._append_log(f"서비스 호출: {self.camera_service}")
        future = self.camera_client.call_async(DetectTomatoes.Request())
        future.add_done_callback(self._detection_service_done)

    def _detection_service_done(self, future) -> None:
        self.detect_button.configure(state="normal")
        try:
            response = future.result()
        except Exception as error:
            self.status.set("카메라 서비스 호출 실패")
            self._append_log(f"카메라 서비스 오류: {error}")
            return
        if not response.success:
            self.status.set("토마토 검출 실패")
            self._append_log(f"검출 실패: {response.message}")
            return
        self._detections_callback(response.detections)
        self.status.set(response.message)
        self._append_log(response.message)

    def _selected_index(self) -> int | None:
        value = self.selected_tomato.get()
        if not value:
            return None
        try:
            return int(value.split(":", maxsplit=1)[0])
        except (ValueError, IndexError):
            return None

    def _tree_selection_changed(self, _event=None) -> None:
        selection = self.tomato_tree.selection()
        if not selection:
            return
        index = int(selection[0])
        self.tomato_combo.current(index)
        self._selection_changed()

    def _combo_selection_changed(self, _event=None) -> None:
        index = self._selected_index()
        if index is not None and self.tomato_tree.exists(str(index)):
            self.tomato_tree.selection_set(str(index))
            self.tomato_tree.focus(str(index))
            self.tomato_tree.see(str(index))
        self._selection_changed()

    def _selection_changed(self) -> None:
        self._invalidate_plan()
        index = self._selected_index()
        if index is not None:
            self.status.set(f"토마토 {index} 선택됨 — Plan-only를 먼저 실행하세요.")

    def _planner_selection_changed(self, _event=None) -> None:
        self._invalidate_plan()
        self.status.set(
            f"{self.selected_planner.get()} 선택됨 — Plan-only를 다시 실행하세요."
        )

    def _selected_planner_config(self) -> tuple[str, str]:
        return PLANNER_CONFIGS.get(
            self.selected_planner.get(),
            PLANNER_CONFIGS["OMPL / RRTConnect"],
        )

    def _invalidate_plan(self) -> None:
        self.verified_plan = None
        self.execute_button.configure(state="disabled")

    def start_harvest(self, execute: bool) -> None:
        index = self._selected_index()
        if index is None:
            messagebox.showwarning("토마토 선택", "수확할 토마토를 먼저 선택하세요.")
            return
        if self.harvest_process is not None:
            messagebox.showinfo("실행 중", "현재 모션 작업이 끝날 때까지 기다려 주세요.")
            return
        pipeline, planner_id = self._selected_planner_config()
        verification = (
            self.detection_generation,
            index,
            pipeline,
            planner_id,
        )
        if execute and self.verified_plan != verification:
            messagebox.showwarning(
                "Plan-only 필요",
                "현재 토마토의 Plan-only가 성공한 뒤 실제 수확을 실행할 수 있습니다.",
            )
            return
        if execute and not messagebox.askyesno(
            "실제 로봇 수확 실행",
            f"토마토 {index}의 전체 수확 모션을 실제로 실행할까요?\n\n"
            "로봇 주변이 안전하고 교시 모드가 해제되었는지 확인하세요.",
            icon="warning",
        ):
            return

        command = harvest_command(
            index,
            execute,
            planning_pipeline_id=pipeline,
            planner_id=planner_id,
        )
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        mode = "실제 수확" if execute else "Plan-only"
        self._append_log(
            f"{mode} 시작: detected_tomato_{index}_tf "
            f"planner={pipeline}/{planner_id}"
        )
        self.status.set(f"{mode} 실행 중...")
        self._set_busy(True)
        try:
            self.harvest_process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=environment,
            )
        except OSError as error:
            self.harvest_process = None
            self._set_busy(False)
            self.status.set(f"{mode} 시작 실패")
            self._append_log(f"프로세스 시작 오류: {error}")
            return
        thread = threading.Thread(
            target=self._read_process,
            args=(self.harvest_process, execute, verification),
            daemon=True,
        )
        thread.start()

    def _read_process(self, process, execute: bool, verification) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                self.process_queue.put(("log", line.rstrip()))
        return_code = process.wait()
        self.process_queue.put(
            ("done", return_code, execute, verification, process)
        )

    def _drain_process_queue(self) -> None:
        while True:
            try:
                item = self.process_queue.get_nowait()
            except queue.Empty:
                break
            if item[0] == "log":
                self._append_log(item[1])
                continue
            _, return_code, execute, verification, process = item
            if process is not self.harvest_process:
                continue
            self.harvest_process = None
            self._set_busy(False)
            mode = "실제 수확" if execute else "Plan-only"
            if return_code == 0:
                self.status.set(f"{mode} 완료")
                self._append_log(f"{mode} 완료 (종료 코드 0)")
                if not execute and verification[0] == self.detection_generation:
                    self.verified_plan = verification
                    if self._selected_index() == verification[1]:
                        self.execute_button.configure(state="normal")
                else:
                    self._invalidate_plan()
            else:
                self.status.set(f"{mode} 실패 — 로그를 확인하세요.")
                self._append_log(f"{mode} 실패 (종료 코드 {return_code})")
                self._invalidate_plan()
        if not self.closing:
            self.root.after(50, self._drain_process_queue)

    def _set_busy(self, busy: bool) -> None:
        state = "disabled" if busy else "normal"
        self.detect_button.configure(state=state)
        self.read_scene_button.configure(state=state)
        self.set_scene_button.configure(state=state)
        self.tomato_combo.configure(state="disabled" if busy else "readonly")
        self.planner_combo.configure(state="disabled" if busy else "readonly")
        if busy or not self.detected_tomatoes:
            self.plan_button.configure(state="disabled")
        else:
            self.plan_button.configure(state="normal")
        if busy:
            self.execute_button.configure(state="disabled")
        elif self.verified_plan == (
            self.detection_generation,
            self._selected_index(),
            *self._selected_planner_config(),
        ):
            self.execute_button.configure(state="normal")

    def read_scene_position(self) -> None:
        if not self.scene_get_client.service_is_ready():
            if not self.scene_get_client.wait_for_service(timeout_sec=0.05):
                self._append_log(
                    f"장면 노드 연결 대기 중: {self.scene_node}"
                )
                return
        request = GetParameters.Request()
        request.names = ["object_position", "tomato_z_spin_deg"]
        future = self.scene_get_client.call_async(request)
        future.add_done_callback(self._scene_position_received)

    def _scene_position_received(self, future) -> None:
        try:
            response = future.result()
            values = list(response.values[0].double_array_value)
            if len(values) != 3:
                raise ValueError(f"object_position 길이가 {len(values)}입니다.")
            rotation_deg = float(response.values[1].double_value)
        except Exception as error:
            self._append_log(f"줄기 위치 읽기 실패: {error}")
            return
        self.scene_x.set(f"{values[0]:.4f}")
        self.scene_y.set(f"{values[1]:.4f}")
        self.scene_z.set(f"{values[2]:.4f}")
        self.scene_rotation.set(f"{rotation_deg:.1f}")
        self.status.set("현재 토마토 줄기 위치와 회전을 읽었습니다.")

    def set_scene_position(self) -> None:
        try:
            position = [
                float(self.scene_x.get()),
                float(self.scene_y.get()),
                float(self.scene_z.get()),
            ]
            rotation_deg = float(self.scene_rotation.get())
        except ValueError:
            messagebox.showerror(
                "입력 오류", "줄기 X, Y, Z와 회전 각도에 숫자를 입력하세요."
            )
            return
        if not self.scene_set_client.service_is_ready():
            if not self.scene_set_client.wait_for_service(timeout_sec=0.05):
                self.status.set(f"장면 노드 연결 안 됨: {self.scene_node}")
                return

        request = SetParameters.Request()
        request.parameters = scene_parameters(position, rotation_deg)
        future = self.scene_set_client.call_async(request)
        future.add_done_callback(self._scene_position_set)
        self.status.set("토마토 줄기 위치와 회전 적용 중...")

    def _scene_position_set(self, future) -> None:
        try:
            response = future.result()
            failed_results = [
                result for result in response.results if not result.successful
            ]
        except Exception as error:
            self.status.set("토마토 줄기 위치 변경 실패")
            self._append_log(f"줄기 위치 변경 오류: {error}")
            return
        if failed_results:
            reasons = "; ".join(result.reason for result in failed_results)
            self.status.set("토마토 줄기 위치/회전 변경 거부됨")
            self._append_log(f"줄기 위치/회전 변경 거부: {reasons}")
            return
        self.detected_tomatoes = []
        self.detection_signature = None
        self.detection_generation += 1
        for item in self.tomato_tree.get_children():
            self.tomato_tree.delete(item)
        self.tomato_combo.configure(values=[])
        self.selected_tomato.set("")
        self.plan_button.configure(state="disabled")
        self._invalidate_plan()
        self.status.set(
            "줄기 위치/회전 적용 완료 — 카메라 검출을 다시 실행하세요."
        )
        self._append_log(
            "줄기 위치/회전을 변경했습니다. 기존 검출/계획은 사용하지 말고 "
            "다시 검출하세요."
        )

    def _spin_ros(self) -> None:
        if self.closing or not rclpy.ok():
            return
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except ExternalShutdownException:
            return
        self.root.after(30, self._spin_ros)

    def _on_close(self) -> None:
        if self.harvest_process is not None:
            if not messagebox.askyesno(
                "모션 작업 실행 중",
                "현재 프로세스를 종료하고 GUI를 닫을까요?\n"
                "프로세스 종료가 이미 전송된 로봇 궤적을 정지시키지는 않습니다.",
                icon="warning",
            ):
                return
            self.harvest_process.terminate()
        self.closing = True
        self.root.quit()

    def _signal_close(self, _signum, _frame) -> None:
        if self.harvest_process is not None:
            self.harvest_process.terminate()
        self.closing = True
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HarvestGui()
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
