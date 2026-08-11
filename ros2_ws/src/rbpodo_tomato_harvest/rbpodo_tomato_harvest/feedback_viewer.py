"""Standalone viewer for camera/robot tomato feedback JSON files."""

import argparse
import json
import math
import tkinter as tk
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


def stem_callout_position(
    tomato,
    recommend,
    actual_vine,
    display_distance: float = 54.0,
):
    """Place the stem beyond tomato along the Recommend approach ray."""
    direction_x = tomato[0] - recommend[0]
    direction_y = tomato[1] - recommend[1]
    length = math.hypot(direction_x, direction_y)
    if length <= 1e-6:
        direction_x = actual_vine[0] - tomato[0]
        direction_y = actual_vine[1] - tomato[1]
        length = math.hypot(direction_x, direction_y)
    if length <= 1e-6:
        direction_x, direction_y, length = 0.0, -1.0, 1.0
    return (
        tomato[0] + direction_x / length * float(display_distance),
        tomato[1] + direction_y / length * float(display_distance),
    )


def robot_side_projection(point):
    """Project link0 XYZ to a signed left/right (-Y) and height (Z) view."""
    return (-float(point[1]), float(point[2]))


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

    def __init__(self, parent, title: str):
        super().__init__(parent)
        ttk.Label(self, text=title, font=("TkDefaultFont", 11, "bold")).pack(
            anchor="w", padx=6, pady=(4, 0)
        )
        self.canvas = tk.Canvas(self, background="white", highlightthickness=1)
        self.canvas.pack(fill="both", expand=True, padx=4, pady=4)
        self.scene = None
        self.mode = "xy"
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def set_scene(self, scene: dict) -> None:
        self.scene = scene
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
    ) -> None:
        x, y = project(point)
        self.canvas.create_oval(
            x - radius,
            y - radius,
            x + radius,
            y + radius,
            fill=color,
            outline="white",
            width=2,
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

    def _stem_callout(self, project, tomato, vine, recommend) -> None:
        tomato_x, tomato_y = project(tomato)
        vine_x, vine_y = project(vine)
        recommend_x, recommend_y = project(recommend)
        display_x, display_y = stem_callout_position(
            (tomato_x, tomato_y),
            (recommend_x, recommend_y),
            (vine_x, vine_y),
        )
        direction_x = display_x - tomato_x
        direction_y = display_y - tomato_y
        direction_length = max(1e-6, math.hypot(direction_x, direction_y))
        unit_x = direction_x / direction_length
        unit_y = direction_y / direction_length
        self.canvas.create_line(
            tomato_x + unit_x * 19,
            tomato_y + unit_y * 19,
            display_x - unit_x * 8,
            display_y - unit_y * 8,
            fill=COLORS["vine"],
            width=2,
            dash=(4, 3),
        )
        self.canvas.create_oval(
            display_x - 7,
            display_y - 7,
            display_x + 7,
            display_y + 7,
            fill=COLORS["vine"],
            outline="white",
            width=2,
        )
        self.canvas.create_text(
            display_x + unit_x * 15,
            display_y + unit_y * 15,
            text="줄기점",
            anchor="center",
            fill=COLORS["vine"],
        )

    def _legend(self) -> None:
        items = (
            (COLORS["recommend"], "Recommend 진입"),
            (COLORS["final"], "Final 진입"),
            (COLORS["tomato"], "토마토"),
            (COLORS["vine"], "줄기점"),
        )
        x = 58
        y = 42
        self.canvas.create_rectangle(
            36,
            24,
            205,
            126,
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
            return
        scene = self.scene
        tomato = scene["robot_tomato"]
        vine = scene["robot_vine"]
        recommend = scene["recommend"]
        final = scene["final"]

        robot = (0.0, 0.0)
        tomato_2d = robot_side_projection(tomato)
        vine_2d = robot_side_projection(vine)
        recommend_2d = robot_side_projection(recommend)
        final_2d = robot_side_projection(final)
        recommend_arrow_start = extended_arrow_start(
            tomato_2d,
            recommend_2d,
        )
        final_arrow_start = extended_arrow_start(tomato_2d, final_2d)
        points = [
            robot,
            tomato_2d,
            vine_2d,
            recommend_arrow_start,
            final_arrow_start,
        ]
        project, bounds = self._projector(points)
        self._grid(
            project,
            bounds,
            "좌측 ←  Link0 -Y (m)  → 우측",
            "Z (m)",
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
            end_gap_px=24,
        )
        self._arrow(
            project,
            final_arrow_start,
            tomato_2d,
            COLORS["final"],
            width=3,
            end_gap_px=24,
        )
        self._point(
            project,
            recommend_2d,
            COLORS["recommend"],
            "",
            4,
        )
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
            "토마토",
            18,
            label_offset=(0, 34),
        )
        self._stem_callout(
            project,
            tomato_2d,
            vine_2d,
            recommend_2d,
        )
        self._legend()


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

        self.side_plot = PlotCanvas(root, "로봇 기준 좌우 측면도 (Y-Z)")
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
        path = filedialog.askopenfilename(
            title="비전 피드백 JSON/TXT 선택",
            initialdir=str(
                Path.home() / "farmily_tomato" / "camera_target_records"
            ),
            filetypes=(("JSON text", "*.txt *.json"), ("All files", "*")),
        )
        if path:
            self.path_var.set(path)
            self.load_current()

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
        self.summary_var.set(
            f"대상: {scene['target_id']}  |  "
            f"Camera ID: {scene['camera_id']}  |  "
            f"Plan: {'성공' if scene['plan_success'] else '실패'}  |  "
            f"Recommend: {scene['recommend_angle_deg']}°  |  "
            f"Final: {scene['final_angle_deg']}°  |  "
            f"보정: {scene['correction_angle_deg']}°\n"
            f"base→tomato: {base_distance:.4f} m  |  "
            f"tomato→vine: {vine_distance:.4f} m\n"
            f"문제 유형: {issue}  |  비고: {note}"
        )


def _latest_feedback_file() -> Path | None:
    directory = Path.home() / "farmily_tomato" / "camera_target_records"
    candidates = sorted(directory.glob("*_feedback.txt"))
    return candidates[-1] if candidates else None


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
