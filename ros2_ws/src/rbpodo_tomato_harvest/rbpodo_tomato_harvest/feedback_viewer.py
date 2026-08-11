"""Standalone viewer for camera/robot tomato feedback JSON files."""

import argparse
import json
import math
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk


COLORS = {
    "robot": "#17324d",
    "tomato": "#e53935",
    "vine": "#43a047",
    "recommend": "#168aad",
    "final": "#fb8c00",
    "grid": "#d9e1e8",
    "text": "#243447",
}


def _xyz(value, field: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field} must contain exactly three coordinates")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{field} contains a non-finite coordinate")
    return result


def load_feedback(path) -> dict:
    """Load and minimally validate one saved feedback JSON/TXT file."""
    source = Path(path).expanduser()
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"파일을 읽을 수 없습니다: {error}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"JSON 형식이 아닙니다: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError("JSON 최상위 값은 object여야 합니다.")
    return payload


def feedback_files_newest_first(directory) -> list[Path]:
    """Return JSON/TXT feedback files ordered by newest modification time."""
    folder = Path(directory).expanduser()
    try:
        candidates = [
            path
            for path in folder.iterdir()
            if path.is_file() and path.suffix.lower() in {".txt", ".json"}
        ]
    except OSError:
        return []

    def sort_key(path: Path):
        try:
            modified_ns = path.stat().st_mtime_ns
        except OSError:
            modified_ns = -1
        return (modified_ns, path.name)

    return sorted(candidates, key=sort_key, reverse=True)


def feedback_scene(payload: dict) -> dict:
    """Extract the camera and link0 geometry required by the viewer."""
    vision = payload.get("vision") or {}
    robot = payload.get("robot") or {}
    required_robot_fields = (
        "tomato_xyz",
        "vine_xyz",
        "recommend_pregrasp_xyz",
        "final_pregrasp_xyz",
    )
    missing = [
        name for name in required_robot_fields if robot.get(name) is None
    ]
    if missing:
        raise ValueError(
            "로봇 좌표가 아직 없는 JSON입니다: "
            + ", ".join(missing)
            + ". Plan 완료 후 새 JSON을 저장하세요."
        )
    return {
        "target_id": str(payload.get("target_id") or "UNKNOWN"),
        "camera_id": str(payload.get("camera_id") or ""),
        "camera_frame": str(vision.get("frame_id") or "camera"),
        "planning_frame": str(robot.get("frame_id") or "link0"),
        "coordinate_source": str(
            robot.get("coordinate_source") or "legacy_planning_geometry"
        ),
        "reference_link": str(robot.get("reference_link") or "tool"),
        "camera_tomato": _xyz(vision.get("tomato_xyz"), "vision.tomato_xyz"),
        "camera_vine": _xyz(vision.get("vine_xyz"), "vision.vine_xyz"),
        "robot_tomato": _xyz(robot.get("tomato_xyz"), "robot.tomato_xyz"),
        "robot_vine": _xyz(robot.get("vine_xyz"), "robot.vine_xyz"),
        "recommend": _xyz(
            robot.get("recommend_pregrasp_xyz"),
            "robot.recommend_pregrasp_xyz",
        ),
        "final": _xyz(
            robot.get("final_pregrasp_xyz"),
            "robot.final_pregrasp_xyz",
        ),
        "recommend_angle_deg": robot.get("recommend_angle_deg"),
        "final_angle_deg": robot.get("final_angle_deg"),
        "correction_angle_deg": robot.get("correction_angle_deg"),
        "plan_success": robot.get("plan_success"),
        "review": payload.get("review") or {},
    }


def signed_planar_angle_deg(origin, first, second) -> float:
    """Return the shortest signed XY angle between two origin vectors."""
    first_angle = math.atan2(first[1] - origin[1], first[0] - origin[0])
    second_angle = math.atan2(second[1] - origin[1], second[0] - origin[0])
    delta = math.atan2(
        math.sin(second_angle - first_angle),
        math.cos(second_angle - first_angle),
    )
    return math.degrees(delta)


def extended_arrow_start(origin, start, factor: float = 3.0):
    """Extend a grasp arrow tail without moving its tomato-side arrowhead."""
    return tuple(
        origin[axis] + (start[axis] - origin[axis]) * float(factor)
        for axis in range(2)
    )


def raw_detection_recommend_point(tomato, vine, reference):
    """Place Recommend opposite the exact stem while retaining plot length."""
    outward_x = tomato[0] - vine[0]
    outward_y = tomato[1] - vine[1]
    outward_length = math.hypot(outward_x, outward_y)
    reference_length = math.hypot(
        reference[0] - tomato[0],
        reference[1] - tomato[1],
    )
    if outward_length <= 1e-9 or reference_length <= 1e-9:
        return reference
    return (
        tomato[0] + outward_x / outward_length * reference_length,
        tomato[1] + outward_y / outward_length * reference_length,
    )


def stem_label_position(
    tomato,
    actual_vine,
    display_distance: float = 54.0,
):
    """Place only the stem text away from its exact projected marker."""
    direction_x = actual_vine[0] - tomato[0]
    direction_y = actual_vine[1] - tomato[1]
    length = math.hypot(direction_x, direction_y)
    if length <= 1e-6:
        direction_x, direction_y, length = -1.0, -1.0, math.sqrt(2.0)
    return (
        actual_vine[0] + direction_x / length * float(display_distance),
        actual_vine[1] + direction_y / length * float(display_distance),
    )


def robot_top_projection(point):
    """Project link0 XYZ into the robot-centred top view used in RViz.

    Link0 +X (robot forward) is drawn upward and Link0 +Y is drawn to the
    left.  This preserves the physical X-Y approach angle instead of hiding
    it in a Y-Z side projection.
    """
    return (-float(point[1]), float(point[0]))


def projected_metric_radius(
    project,
    point,
    radius_m: float,
    minimum_px: float = 2.0,
) -> float:
    """Convert a physical plot radius in metres to the current pixel scale."""
    center_x, center_y = project(point)
    edge_x, edge_y = project((point[0] + float(radius_m), point[1]))
    return max(float(minimum_px), math.hypot(edge_x - center_x, edge_y - center_y))


def _bounds(points, minimum_span: float = 0.25):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    center_x = (min(xs) + max(xs)) * 0.5
    center_y = (min(ys) + max(ys)) * 0.5
    span_x = max(max(xs) - min(xs), minimum_span)
    span_y = max(max(ys) - min(ys), minimum_span)
    span = max(span_x, span_y) * 1.30
    return (
        center_x - span * 0.5,
        center_x + span * 0.5,
        center_y - span * 0.5,
        center_y + span * 0.5,
    )


class PlotCanvas(ttk.Frame):
    """Small dependency-free coordinate plot backed by Tk Canvas."""

    def __init__(
        self,
        parent,
        title: str,
        empty_message: str = "",
        show_legend: bool = True,
        marker_scale: float = 1.0,
        stem_marker_scale: float = 1.0,
        show_point_labels: bool = True,
    ):
        super().__init__(parent)
        ttk.Label(self, text=title, font=("TkDefaultFont", 11, "bold")).pack(
            anchor="w", padx=6, pady=(4, 0)
        )
        self.canvas = tk.Canvas(self, background="white", highlightthickness=1)
        self.canvas.pack(fill="both", expand=True, padx=4, pady=4)
        self.scene = None
        self.empty_message = str(empty_message)
        self.show_legend = bool(show_legend)
        self.marker_scale = max(0.1, float(marker_scale))
        self.stem_marker_scale = max(0.1, float(stem_marker_scale))
        self.show_point_labels = bool(show_point_labels)
        self.mode = "xy"
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def set_scene(self, scene: dict) -> None:
        self.scene = scene
        self.redraw()

    def clear(self, message: str | None = None) -> None:
        self.scene = None
        if message is not None:
            self.empty_message = str(message)
        self.redraw()

    def _projector(self, points):
        width = max(200, self.canvas.winfo_width())
        height = max(200, self.canvas.winfo_height())
        margin = 42
        x0, x1, y0, y1 = _bounds(points)
        scale = min(
            (width - margin * 2) / (x1 - x0),
            (height - margin * 2) / (y1 - y0),
        )

        def project(point):
            return (
                margin + (point[0] - x0) * scale,
                height - margin - (point[1] - y0) * scale,
            )

        return project, (x0, x1, y0, y1)

    def _grid(self, project, bounds, x_label: str, y_label: str) -> None:
        x0, x1, y0, y1 = bounds
        for ratio in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = x0 + (x1 - x0) * ratio
            y = y0 + (y1 - y0) * ratio
            px0, py0 = project((x, y0))
            px1, py1 = project((x, y1))
            self.canvas.create_line(px0, py0, px1, py1, fill=COLORS["grid"])
            px0, py0 = project((x0, y))
            px1, py1 = project((x1, y))
            self.canvas.create_line(px0, py0, px1, py1, fill=COLORS["grid"])
        self.canvas.create_text(55, 16, text=y_label, fill=COLORS["text"])
        self.canvas.create_text(
            max(100, self.canvas.winfo_width() - 45),
            max(100, self.canvas.winfo_height() - 18),
            text=x_label,
            fill=COLORS["text"],
        )

    def _point(
        self,
        project,
        point,
        color: str,
        label: str = "",
        radius=7,
        label_offset=(9, -10),
        outline: str = "white",
        outline_width: int = 2,
    ) -> None:
        x, y = project(point)
        self.canvas.create_oval(
            x - radius,
            y - radius,
            x + radius,
            y + radius,
            fill=color,
            outline=outline,
            width=outline_width,
        )
        if label:
            self.canvas.create_text(
                x + label_offset[0],
                y + label_offset[1],
                text=label,
                anchor="sw",
                fill=color,
            )

    def _arrow(
        self,
        project,
        start,
        end,
        color: str,
        label: str = "",
        width: int = 4,
        end_gap_px: float = 0.0,
    ) -> None:
        x0, y0 = project(start)
        x1, y1 = project(end)
        length = math.hypot(x1 - x0, y1 - y0)
        if length > 1e-6 and end_gap_px > 0.0:
            gap = min(float(end_gap_px), length * 0.45)
            x1 -= (x1 - x0) / length * gap
            y1 -= (y1 - y0) / length * gap
        self.canvas.create_line(
            x0,
            y0,
            x1,
            y1,
            fill=color,
            width=width,
            arrow=tk.LAST,
            arrowshape=(12, 15, 5),
        )
        if label:
            self.canvas.create_text(
                (x0 + x1) * 0.5,
                (y0 + y1) * 0.5 - 10,
                text=label,
                fill=color,
            )

    def _stem_marker(
        self,
        project,
        tomato,
        vine,
        radius_px: float,
        show_label: bool = True,
    ) -> None:
        """Draw the stem at its exact coordinate and offset only its label."""
        tomato_x, tomato_y = project(tomato)
        vine_x, vine_y = project(vine)
        if show_label:
            label_x, label_y = stem_label_position(
                (tomato_x, tomato_y),
                (vine_x, vine_y),
            )
            direction_x = label_x - vine_x
            direction_y = label_y - vine_y
            direction_length = max(
                1e-6,
                math.hypot(direction_x, direction_y),
            )
            unit_x = direction_x / direction_length
            unit_y = direction_y / direction_length
            self.canvas.create_line(
                vine_x + unit_x * (radius_px + 1.0),
                vine_y + unit_y * (radius_px + 1.0),
                label_x - unit_x * 18,
                label_y - unit_y * 18,
                fill=COLORS["vine"],
                width=2,
                dash=(4, 3),
            )
        self.canvas.create_oval(
            vine_x - radius_px,
            vine_y - radius_px,
            vine_x + radius_px,
            vine_y + radius_px,
            fill=COLORS["vine"],
            outline="#145a32",
            width=2,
        )
        if show_label:
            self.canvas.create_text(
                label_x,
                label_y,
                text="줄기점 (실좌표)",
                anchor="center",
                fill=COLORS["vine"],
            )

    def _legend(self, show_final: bool = True) -> None:
        items = [
            (COLORS["recommend"], "Recommend 진입"),
            (COLORS["tomato"], "토마토"),
            (COLORS["vine"], "줄기점"),
        ]
        if show_final:
            items.insert(1, (COLORS["final"], "Final 진입"))
        x = 58
        canvas_height = max(200, self.canvas.winfo_height())
        rectangle_bottom = canvas_height - 26
        rectangle_top = rectangle_bottom - 102
        y = rectangle_top + 18
        self.canvas.create_rectangle(
            36,
            rectangle_top,
            205,
            rectangle_bottom,
            fill="white",
            outline="#c7d1d9",
        )
        for color, label in items:
            self.canvas.create_line(x, y, x + 24, y, fill=color, width=5)
            self.canvas.create_text(
                x + 32,
                y,
                text=label,
                anchor="w",
                fill=COLORS["text"],
            )
            y += 23

    def _distance_line(self, project, start, end, label: str) -> None:
        x0, y0 = project(start)
        x1, y1 = project(end)
        self.canvas.create_line(
            x0,
            y0,
            x1,
            y1,
            fill="#607d8b",
            width=2,
            dash=(7, 5),
        )
        self.canvas.create_text(
            (x0 + x1) * 0.5,
            (y0 + y1) * 0.5 + 13,
            text=label,
            fill="#455a64",
        )

    def _angle_arc(self, project, origin, first, second, label: str) -> None:
        first_angle = math.atan2(first[1] - origin[1], first[0] - origin[0])
        delta = math.radians(signed_planar_angle_deg(origin, first, second))
        radius = max(
            0.025,
            min(
                math.dist(origin[:2], first[:2]),
                math.dist(origin[:2], second[:2]),
            )
            * 0.38,
        )
        points = []
        for step in range(25):
            angle = first_angle + delta * step / 24.0
            points.extend(
                project(
                    (
                        origin[0] + radius * math.cos(angle),
                        origin[1] + radius * math.sin(angle),
                    )
                )
            )
        self.canvas.create_line(*points, fill="#8e44ad", width=3)
        mid_angle = first_angle + delta * 0.5
        text_point = project(
            (
                origin[0] + radius * 1.35 * math.cos(mid_angle),
                origin[1] + radius * 1.35 * math.sin(mid_angle),
            )
        )
        self.canvas.create_text(*text_point, text=label, fill="#8e44ad")

    def redraw(self) -> None:
        self.canvas.delete("all")
        if self.scene is None:
            if self.empty_message:
                self.canvas.create_text(
                    max(100, self.canvas.winfo_width() * 0.5),
                    max(100, self.canvas.winfo_height() * 0.5),
                    text=self.empty_message,
                    fill=COLORS["text"],
                    justify="center",
                )
            return
        scene = self.scene
        tomato = scene["robot_tomato"]
        vine = scene["robot_vine"]
        recommend = scene["recommend"]
        final = scene.get("final")

        robot = (0.0, 0.0)
        tomato_2d = robot_top_projection(tomato)
        vine_2d = robot_top_projection(vine)
        planned_recommend_2d = robot_top_projection(recommend)
        recommend_2d = raw_detection_recommend_point(
            tomato_2d,
            vine_2d,
            planned_recommend_2d,
        )
        final_2d = robot_top_projection(final) if final is not None else None
        recommend_arrow_start = extended_arrow_start(
            tomato_2d,
            recommend_2d,
        )
        final_arrow_start = (
            extended_arrow_start(tomato_2d, final_2d)
            if final_2d is not None
            else None
        )
        points = [
            robot,
            tomato_2d,
            vine_2d,
            recommend_arrow_start,
        ]
        if final_arrow_start is not None:
            points.append(final_arrow_start)
        project, bounds = self._projector(points)
        tomato_radius_px = projected_metric_radius(
            project,
            tomato_2d,
            0.0175 * 0.5,
            minimum_px=4.0,
        ) * self.marker_scale
        stem_radius_px = projected_metric_radius(
            project,
            vine_2d,
            0.006 * 0.5,
            minimum_px=6.0,
        ) * self.marker_scale * self.stem_marker_scale
        self._grid(
            project,
            bounds,
            "Link0  +Y ← / → -Y (m)",
            "Link0 +X (전방)",
        )

        self._point(
            project,
            robot,
            COLORS["robot"],
            "로봇 base",
            10,
            label_offset=(12, -6),
        )
        base_distance = math.dist((0.0, 0.0, 0.0), tomato)
        self._distance_line(
            project,
            robot,
            tomato_2d,
            f"base→tomato {base_distance:.3f} m",
        )
        self._arrow(
            project,
            recommend_arrow_start,
            tomato_2d,
            COLORS["recommend"],
            width=7,
            end_gap_px=tomato_radius_px + 4.0,
        )
        if final_arrow_start is not None:
            self._arrow(
                project,
                final_arrow_start,
                tomato_2d,
                COLORS["final"],
                width=3,
                end_gap_px=tomato_radius_px + 4.0,
            )
        self._point(
            project,
            recommend_2d,
            COLORS["recommend"],
            "",
            4,
        )
        if final_2d is not None:
            self._point(
                project,
                final_2d,
                COLORS["final"],
                "",
                4,
            )
        self._point(
            project,
            tomato_2d,
            COLORS["tomato"],
            "토마토" if self.show_point_labels else "",
            tomato_radius_px,
            label_offset=(0, tomato_radius_px + 16.0),
            outline="#8e2c23",
            outline_width=2,
        )
        self._stem_marker(
            project,
            tomato_2d,
            vine_2d,
            stem_radius_px,
            show_label=self.show_point_labels,
        )
        if self.show_legend:
            self._legend(show_final=final_2d is not None)


class FeedbackViewer:
    def __init__(self, root: tk.Tk, initial_path=None):
        self.root = root
        self.root.title("Farmily Vision Feedback Viewer")
        self.root.geometry("980x860")
        self.root.minsize(760, 620)
        self.path_var = tk.StringVar(value=str(initial_path or ""))
        self.summary_var = tk.StringVar(value="JSON 파일을 선택하세요.")

        toolbar = ttk.Frame(root, padding=8)
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="피드백 파일").pack(side="left")
        ttk.Entry(toolbar, textvariable=self.path_var).pack(
            side="left", fill="x", expand=True, padx=8
        )
        ttk.Button(toolbar, text="파일 선택", command=self.choose_file).pack(
            side="left", padx=(0, 6)
        )
        ttk.Button(toolbar, text="그래프 열기", command=self.load_current).pack(
            side="left"
        )

        self.side_plot = PlotCanvas(
            root,
            "로봇 기준 평면도 (X-Y, 위에서 본 모습)",
        )
        self.side_plot.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        summary = ttk.Label(
            root,
            textvariable=self.summary_var,
            anchor="w",
            justify="left",
            relief="groove",
            padding=8,
        )
        summary.pack(fill="x", padx=8, pady=(0, 8))
        if initial_path:
            root.after(50, self.load_current)

    def choose_file(self) -> None:
        directory = (
            Path.home() / "farmily_tomato" / "camera_target_records"
        )
        paths = feedback_files_newest_first(directory)
        dialog = tk.Toplevel(self.root)
        dialog.title("비전 피드백 파일 선택 — 최신순")
        dialog.geometry("820x560")
        dialog.minsize(620, 380)
        dialog.transient(self.root)
        dialog.grab_set()

        ttk.Label(
            dialog,
            text=f"최근 파일이 위에 표시됩니다.  경로: {directory}",
            padding=(10, 10, 10, 6),
        ).pack(fill="x")

        table_frame = ttk.Frame(dialog, padding=(10, 0, 10, 8))
        table_frame.pack(fill="both", expand=True)
        table = ttk.Treeview(
            table_frame,
            columns=("modified", "filename"),
            show="headings",
            selectmode="browse",
        )
        table.heading("modified", text="수정 시각 (최신순)")
        table.heading("filename", text="파일 이름")
        table.column("modified", width=165, minwidth=145, stretch=False)
        table.column("filename", width=590, minwidth=320, stretch=True)
        scrollbar = ttk.Scrollbar(
            table_frame,
            orient="vertical",
            command=table.yview,
        )
        table.configure(yscrollcommand=scrollbar.set)
        table.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for index, path in enumerate(paths):
            try:
                modified = datetime.fromtimestamp(
                    path.stat().st_mtime
                ).strftime("%Y-%m-%d %H:%M:%S")
            except OSError:
                modified = "알 수 없음"
            table.insert(
                "",
                "end",
                iid=str(index),
                values=(modified, path.name),
            )

        def open_path(path: Path) -> None:
            self.path_var.set(str(path))
            dialog.destroy()
            self.load_current()

        def open_selected(_event=None) -> None:
            selection = table.selection()
            if not selection:
                return
            open_path(paths[int(selection[0])])

        def browse_other() -> None:
            path = filedialog.askopenfilename(
                parent=dialog,
                title="다른 비전 피드백 JSON/TXT 선택",
                initialdir=str(directory),
                filetypes=(
                    ("JSON text", "*.txt *.json"),
                    ("All files", "*"),
                ),
            )
            if path:
                open_path(Path(path))

        buttons = ttk.Frame(dialog, padding=(10, 0, 10, 10))
        buttons.pack(fill="x")
        ttk.Button(
            buttons,
            text="다른 위치...",
            command=browse_other,
        ).pack(side="left")
        ttk.Button(
            buttons,
            text="취소",
            command=dialog.destroy,
        ).pack(side="right")
        open_button = ttk.Button(
            buttons,
            text="선택 파일 열기",
            command=open_selected,
        )
        open_button.pack(side="right", padx=(0, 8))

        table.bind("<Double-1>", open_selected)
        table.bind("<Return>", open_selected)
        if paths:
            table.selection_set("0")
            table.focus("0")
            table.see("0")
            table.focus_set()
        else:
            open_button.configure(state="disabled")

    def load_current(self) -> None:
        try:
            payload = load_feedback(self.path_var.get())
            scene = feedback_scene(payload)
        except ValueError as error:
            messagebox.showerror("피드백 파일 오류", str(error))
            return
        self.side_plot.set_scene(scene)
        issue = (scene["review"].get("issue") or {}).get("label", "없음")
        note = str(scene["review"].get("note") or "-")
        base_distance = math.dist((0.0, 0.0, 0.0), scene["robot_tomato"])
        vine_distance = math.dist(scene["robot_tomato"], scene["robot_vine"])
        camera_depth_delta = (
            scene["camera_vine"][2] - scene["camera_tomato"][2]
        )
        if math.isclose(camera_depth_delta, 0.0, abs_tol=0.00005):
            camera_depth_text = "카메라와 거의 같은 깊이"
        elif camera_depth_delta < 0.0:
            camera_depth_text = "줄기점이 카메라에 더 가까움"
        else:
            camera_depth_text = "줄기점이 카메라에서 더 멂"
        if scene["coordinate_source"] == "detection_tf_snapshot":
            coordinate_text = "검출 시점 TF 실좌표"
        else:
            coordinate_text = "레거시 재구성 좌표 — 새 검출 후 다시 저장 필요"
        self.summary_var.set(
            f"대상: {scene['target_id']}  |  "
            f"Camera ID: {scene['camera_id']}  |  "
            f"Plan: {'성공' if scene['plan_success'] else '실패'}  |  "
            f"Recommend: {scene['recommend_angle_deg']}°  |  "
            f"Final: {scene['final_angle_deg']}°  |  "
            f"보정: {scene['correction_angle_deg']}°\n"
            f"base→tomato: {base_distance:.4f} m  |  "
            f"tomato→vine: {vine_distance:.4f} m\n"
            f"Camera ΔZ(vine-tomato): {camera_depth_delta:+.4f} m  |  "
            f"{camera_depth_text}\n"
            f"줄기 좌표 출처: {coordinate_text}\n"
            f"문제 유형: {issue}  |  비고: {note}"
        )


def _latest_feedback_file() -> Path | None:
    directory = Path.home() / "farmily_tomato" / "camera_target_records"
    candidates = feedback_files_newest_first(directory)
    return candidates[0] if candidates else None


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Visualize one Farmily vision-feedback JSON file."
    )
    parser.add_argument("file", nargs="?", help="feedback .txt/.json path")
    arguments = parser.parse_args(argv)
    initial_path = (
        Path(arguments.file).expanduser() if arguments.file else None
    )
    if initial_path is None:
        initial_path = _latest_feedback_file()
    root = tk.Tk()
    FeedbackViewer(root, initial_path)
    root.mainloop()


if __name__ == "__main__":
    main()
