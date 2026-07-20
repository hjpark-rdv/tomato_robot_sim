import json
import math
import time
import uuid
from pathlib import Path
from typing import Any

import numpy as np


FEATURE_NAMES = [
    "trajectory_score",
    "trajectory_duration_sec",
    "planning_time",
    "point_count",
    "total_path_motion",
    "max_joint_path_motion",
    "max_waypoint_jump",
    "max_start_goal_delta",
    "grasp_precision_score",
    "gripper_spin_deg",
    "abs_gripper_spin_deg",
    "gripper_x_sign",
    "rod_tomato_z_parallel_angle_deg",
    "gripper_tomato_y_parallel_angle_deg",
    "calyx_alignment_angle_deg",
    "calyx_contact_error",
    "opposite_contact_error",
    "tool_tip_target_error",
    "tool_tip_x_axis_offset",
    "tool0_z_minus_gripper_link_z",
    "wrist2_vine_guard_max_over_m",
    "wrist2_vine_guard_ok",
    "path_shoulder_pan",
    "path_shoulder_lift",
    "path_elbow",
    "path_wrist_1",
    "path_wrist_2",
    "path_wrist_3",
    "path_tool_bend",
    "path_tool_gripper_spin",
    "start_goal_shoulder_pan",
    "start_goal_shoulder_lift",
    "start_goal_elbow",
    "start_goal_wrist_1",
    "start_goal_wrist_2",
    "start_goal_wrist_3",
    "start_goal_tool_bend",
    "start_goal_tool_gripper_spin",
]


def read_jsonl(path: str | Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    input_path = Path(path).expanduser()
    if not input_path.exists():
        return records
    with input_path.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    return records


def append_jsonl(path: str | Path, payload: dict[str, Any]) -> None:
    output_path = Path(path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(_json_safe(payload), ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")


def write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output_path = Path(path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(_json_safe(payload), ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def read_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).expanduser().read_text(encoding="utf-8"))


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def _nested_get(payload: dict[str, Any], *keys: str, default: Any = None) -> Any:
    value: Any = payload
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def _joint_metric(metrics: dict[str, Any], name: str, joint_name: str) -> float:
    value = _nested_get(metrics, name, joint_name, default=0.0)
    return _safe_float(value)


def _finite_float_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, dict):
        return {
            str(key): converted
            for key, item in value.items()
            if (converted := _json_safe(item)) is not None
        }
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return str(value)


def extract_features(candidate_record: dict[str, Any]) -> list[float]:
    metrics = candidate_record.get("trajectory_metrics", {})
    if not isinstance(metrics, dict):
        metrics = {}
    candidate = candidate_record.get("candidate_metrics", {})
    if not isinstance(candidate, dict):
        candidate = {}

    wrist2_guard = metrics.get("wrist2_vine_guard", {})
    if not isinstance(wrist2_guard, dict):
        wrist2_guard = {}

    gripper_spin = _safe_float(candidate_record.get("gripper_spin_deg", candidate.get("gripper_spin_deg", 0.0)))
    values = [
        _safe_float(metrics.get("score")),
        _safe_float(metrics.get("trajectory_duration_sec", metrics.get("duration"))),
        _safe_float(metrics.get("planning_time", candidate_record.get("planning_time"))),
        _safe_float(metrics.get("point_count", len(_nested_get(candidate_record, "trajectory", "points", default=[])))),
        _safe_float(metrics.get("total_path_motion", metrics.get("joint_path_length"))),
        _safe_float(metrics.get("max_joint_path_motion")),
        _safe_float(metrics.get("max_waypoint_jump")),
        _safe_float(metrics.get("max_start_goal_delta")),
        _safe_float(candidate_record.get("grasp_precision_score")),
        gripper_spin,
        abs(gripper_spin),
        _safe_float(candidate_record.get("gripper_x_sign", candidate.get("gripper_x_sign", 0.0))),
        _safe_float(candidate.get("rod_tomato_z_parallel_angle_deg")),
        _safe_float(candidate.get("gripper_tomato_y_parallel_angle_deg")),
        _safe_float(candidate.get("calyx_alignment_angle_deg")),
        _safe_float(candidate.get("calyx_contact_error")),
        _safe_float(candidate.get("opposite_contact_error")),
        _safe_float(candidate.get("tool_tip_target_error")),
        _safe_float(candidate.get("tool_tip_x_axis_offset")),
        _safe_float(candidate.get("tool0_z_minus_gripper_link_z")),
        _safe_float(wrist2_guard.get("max_over_vine")),
        1.0 if bool(wrist2_guard.get("ok", False)) else 0.0,
        _joint_metric(metrics, "path_by_joint", "shoulder_pan_joint"),
        _joint_metric(metrics, "path_by_joint", "shoulder_lift_joint"),
        _joint_metric(metrics, "path_by_joint", "elbow_joint"),
        _joint_metric(metrics, "path_by_joint", "wrist_1_joint"),
        _joint_metric(metrics, "path_by_joint", "wrist_2_joint"),
        _joint_metric(metrics, "path_by_joint", "wrist_3_joint"),
        _joint_metric(metrics, "path_by_joint", "tool_bend_joint"),
        _joint_metric(metrics, "path_by_joint", "tool_gripper_z_joint"),
        _joint_metric(metrics, "start_goal_by_joint", "shoulder_pan_joint"),
        _joint_metric(metrics, "start_goal_by_joint", "shoulder_lift_joint"),
        _joint_metric(metrics, "start_goal_by_joint", "elbow_joint"),
        _joint_metric(metrics, "start_goal_by_joint", "wrist_1_joint"),
        _joint_metric(metrics, "start_goal_by_joint", "wrist_2_joint"),
        _joint_metric(metrics, "start_goal_by_joint", "wrist_3_joint"),
        _joint_metric(metrics, "start_goal_by_joint", "tool_bend_joint"),
        _joint_metric(metrics, "start_goal_by_joint", "tool_gripper_z_joint"),
    ]
    return values


LABEL_SCHEMA = "trajectory_label.v1"


def _finite_sequence(value: Any, length: int | None = None, field_name: str = "value") -> list[float]:
    if isinstance(value, np.ndarray):
        raw = value.tolist()
    elif isinstance(value, dict) and {"x", "y", "z"}.issubset(value.keys()):
        raw = [value["x"], value["y"], value["z"]]
    elif isinstance(value, (list, tuple)):
        raw = list(value)
    else:
        raise ValueError(f"{field_name} must be a numeric sequence.")
    if length is not None and len(raw) != length:
        raise ValueError(f"{field_name} must have length {length}, got {len(raw)}.")
    result: list[float] = []
    for index, item in enumerate(raw):
        number = _finite_float_or_none(item)
        if number is None:
            raise ValueError(f"{field_name}[{index}] is not a finite number.")
        result.append(number)
    return result


def _xyz_from_payload(value: Any, field_name: str) -> list[float]:
    if isinstance(value, dict) and "xyz" in value:
        value = value["xyz"]
    if isinstance(value, dict) and "position" in value:
        value = value["position"]
    return _finite_sequence(value, length=3, field_name=field_name)


def _rpy_from_quaternion_xyzw(quaternion: list[float]) -> list[float]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("orientation quaternion has near-zero norm.")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [float(roll), float(pitch), float(yaw)]


def _pose_from_payload(value: Any, field_name: str) -> dict[str, list[float]]:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a pose dictionary.")
    xyz = _xyz_from_payload(value, f"{field_name}.xyz")
    if "rpy_rad" in value:
        rpy_rad = _finite_sequence(value["rpy_rad"], length=3, field_name=f"{field_name}.rpy_rad")
    elif "orientation_xyzw" in value:
        orientation = value["orientation_xyzw"]
        if not isinstance(orientation, dict):
            raise ValueError(f"{field_name}.orientation_xyzw must be a dictionary.")
        rpy_rad = _rpy_from_quaternion_xyzw(
            _finite_sequence(
                [orientation.get("x"), orientation.get("y"), orientation.get("z"), orientation.get("w")],
                length=4,
                field_name=f"{field_name}.orientation_xyzw",
            )
        )
    else:
        raise ValueError(f"{field_name} must include rpy_rad or orientation_xyzw.")
    return {"xyz": xyz, "rpy_rad": rpy_rad}


def _trajectory_from_candidate(candidate_record: dict[str, Any]) -> dict[str, Any]:
    trajectory = candidate_record.get("trajectory", {})
    if not isinstance(trajectory, dict):
        raise ValueError("candidate.trajectory must be a dictionary.")
    joint_names = trajectory.get("joint_names", [])
    if not isinstance(joint_names, list) or not joint_names:
        raise ValueError("candidate.trajectory.joint_names is required.")
    joint_names = [str(name) for name in joint_names]
    raw_points = trajectory.get("points", [])
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError("candidate.trajectory.points is required.")
    points: list[dict[str, Any]] = []
    for index, point in enumerate(raw_points):
        if not isinstance(point, dict):
            raise ValueError(f"candidate.trajectory.points[{index}] must be a dictionary.")
        t = _finite_float_or_none(point.get("t", point.get("time_from_start_sec")))
        if t is None:
            raise ValueError(f"candidate.trajectory.points[{index}].t is required.")
        q = _finite_sequence(
            point.get("q", point.get("positions")),
            length=len(joint_names),
            field_name=f"candidate.trajectory.points[{index}].q",
        )
        points.append({"t": t, "q": q})
    return {"joint_names": joint_names, "points": points}


def _start_state_from_candidate(candidate_record: dict[str, Any], trajectory: dict[str, Any]) -> dict[str, Any]:
    raw_start = candidate_record.get("start_state")
    joint_names = list(trajectory["joint_names"])
    if isinstance(raw_start, dict):
        raw_names = raw_start.get("joint_names", joint_names)
        raw_positions = raw_start.get("positions", raw_start.get("q"))
        if raw_positions is not None:
            names = [str(name) for name in raw_names]
            return {
                "joint_names": names,
                "positions": _finite_sequence(
                    raw_positions,
                    length=len(names),
                    field_name="candidate.start_state.positions",
                ),
            }
    first_point = trajectory["points"][0]
    return {"joint_names": joint_names, "positions": list(first_point["q"])}


def _put_metric(output: dict[str, float], name: str, *values: Any) -> None:
    for value in values:
        number = _finite_float_or_none(value)
        if number is not None:
            output[name] = number
            return


def _trajectory_metrics_from_candidate(candidate_record: dict[str, Any]) -> dict[str, float]:
    raw_metrics = candidate_record.get("trajectory_metrics", {})
    if not isinstance(raw_metrics, dict):
        raw_metrics = {}
    candidate_metrics = candidate_record.get("candidate_metrics", {})
    if not isinstance(candidate_metrics, dict):
        candidate_metrics = {}
    metrics: dict[str, float] = {}
    _put_metric(metrics, "duration", raw_metrics.get("duration"), raw_metrics.get("trajectory_duration_sec"))
    _put_metric(metrics, "joint_path_length", raw_metrics.get("joint_path_length"), raw_metrics.get("total_path_motion"))
    _put_metric(metrics, "approach_angle_error_deg", candidate_metrics.get("gripper_tomato_y_parallel_angle_deg"))
    # The current planner diagnostics do not provide real TCP samples or signed distances to every obstacle.
    # Those optional metrics are intentionally omitted instead of storing guessed values.
    return metrics


def _context_from_payload(diagnostics_payload: dict[str, Any]) -> dict[str, Any]:
    perception = diagnostics_payload.get("perception_observation", {})
    if not isinstance(perception, dict):
        perception = {}
    selected_tomato_raw = perception.get("selected_tomato", {})
    if not isinstance(selected_tomato_raw, dict):
        selected_tomato_raw = {}
    selected_object_id = str(
        perception.get("selected_object_id")
        or selected_tomato_raw.get("object_id")
        or ""
    )
    selected_tomato: dict[str, Any] = {
        "tomato_pose_base": _pose_from_payload(
            selected_tomato_raw.get("tomato_pose_base", {}),
            "context.perception_observation.selected_tomato.tomato_pose_base",
        ),
        "calyx_position_base": _xyz_from_payload(
            selected_tomato_raw.get("calyx_position_base"),
            "context.perception_observation.selected_tomato.calyx_position_base",
        ),
        "stem_pose_base": _pose_from_payload(
            selected_tomato_raw.get("stem_pose_base", {}),
            "context.perception_observation.selected_tomato.stem_pose_base",
        ),
    }
    if selected_object_id:
        selected_tomato["object_id"] = selected_object_id
    frame = str(perception.get("frame") or "base_link")
    return {
        "input_contract": "real_world_camera_pose_and_planner_trajectory.v1",
        "side": str(diagnostics_payload.get("side") or ""),
        "selected_object_id": selected_object_id,
        "perception_observation": {
            "frame": frame,
            "selected_grasp_pose_base": _pose_from_payload(
                perception.get("selected_grasp_pose_base", {}),
                "context.perception_observation.selected_grasp_pose_base",
            ),
            "selected_tomato": selected_tomato,
        },
    }


def _candidate_for_label(candidate_record: dict[str, Any]) -> dict[str, Any]:
    trajectory = _trajectory_from_candidate(candidate_record)
    candidate: dict[str, Any] = {
        "start_state": _start_state_from_candidate(candidate_record, trajectory),
        "planning_pose": _pose_from_payload(candidate_record.get("planning_pose", {}), "candidate.planning_pose"),
        "trajectory": trajectory,
    }
    gripper_spin_deg = _finite_float_or_none(candidate_record.get("gripper_spin_deg"))
    if gripper_spin_deg is not None:
        candidate["gripper_spin_deg"] = gripper_spin_deg
    gripper_x_sign = _finite_float_or_none(candidate_record.get("gripper_x_sign"))
    if gripper_x_sign is not None:
        candidate["gripper_x_sign"] = gripper_x_sign
    metrics = _trajectory_metrics_from_candidate(candidate_record)
    if metrics:
        candidate["trajectory_metrics"] = metrics
    return candidate


def _normalize_label_and_score(label: str, human_score: float | None) -> tuple[str, float]:
    label_normalized = str(label).strip().lower()
    if human_score is None:
        if label_normalized in {"good", "best", "ok", "positive", "1", "true"}:
            human_score = 1.0
        elif label_normalized in {"bad", "ng", "negative", "0", "false"}:
            human_score = 0.0
        else:
            human_score = 0.5
    score = _finite_float_or_none(human_score)
    if score is None:
        score = 0.5
    score = max(0.0, min(1.0, score))
    if label_normalized not in {"good", "bad"}:
        label_normalized = "good" if score >= 0.5 else "bad"
    return label_normalized, score


def _ids_for_label(
    diagnostics_payload: dict[str, Any],
    candidate_record: dict[str, Any],
    candidate_rank: int | None,
) -> tuple[str, str, str]:
    planning_request_id = str(
        diagnostics_payload.get("planning_request_id")
        or candidate_record.get("planning_request_id")
        or f"req_{int(time.time() * 1000)}"
    )
    fallback_rank = candidate_rank if candidate_rank is not None else "selected"
    candidate_id = str(
        candidate_record.get("candidate_id")
        or candidate_record.get("_label_candidate_id")
        or f"cand_{fallback_rank}_{uuid.uuid4().hex[:8]}"
    )
    sample_id = f"{planning_request_id}_{candidate_id}_{int(time.time() * 1000)}"
    return sample_id, planning_request_id, candidate_id


def validate_trajectory_label_record(record: dict[str, Any]) -> None:
    def require_dict(value: Any, field_name: str) -> dict[str, Any]:
        if not isinstance(value, dict):
            raise ValueError(f"{field_name} is required.")
        return value

    def require_list(value: Any, field_name: str, min_len: int = 1) -> list[Any]:
        if not isinstance(value, list) or len(value) < min_len:
            raise ValueError(f"{field_name} is required.")
        return value

    def validate_no_null_or_nonfinite(value: Any, path: str = "record") -> None:
        if value is None:
            raise ValueError(f"{path} must be omitted instead of null.")
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f"{path} is not finite.")
        if isinstance(value, dict):
            for key, item in value.items():
                validate_no_null_or_nonfinite(item, f"{path}.{key}")
        elif isinstance(value, list):
            for index, item in enumerate(value):
                validate_no_null_or_nonfinite(item, f"{path}[{index}]")

    if record.get("schema") != LABEL_SCHEMA:
        raise ValueError(f"schema must be {LABEL_SCHEMA}.")
    for field_name in ("sample_id", "planning_request_id", "candidate_id"):
        if not isinstance(record.get(field_name), str) or not record[field_name]:
            raise ValueError(f"{field_name} is required.")
    if record.get("label") not in {"good", "bad"}:
        raise ValueError("label must be 'good' or 'bad'.")
    score = _finite_float_or_none(record.get("human_score"))
    if score is None or score < 0.0 or score > 1.0:
        raise ValueError("human_score must be a finite value in [0, 1].")

    context = require_dict(record.get("context"), "context")
    perception = require_dict(context.get("perception_observation"), "context.perception_observation")
    require_dict(perception.get("selected_grasp_pose_base"), "context.perception_observation.selected_grasp_pose_base")
    selected_tomato = require_dict(
        perception.get("selected_tomato"),
        "context.perception_observation.selected_tomato",
    )
    require_dict(selected_tomato.get("tomato_pose_base"), "selected_tomato.tomato_pose_base")
    require_list(selected_tomato.get("calyx_position_base"), "selected_tomato.calyx_position_base", min_len=3)
    require_dict(selected_tomato.get("stem_pose_base"), "selected_tomato.stem_pose_base")

    candidate = require_dict(record.get("candidate"), "candidate")
    start_state = require_dict(candidate.get("start_state"), "candidate.start_state")
    start_joint_names = require_list(start_state.get("joint_names"), "candidate.start_state.joint_names")
    start_positions = require_list(start_state.get("positions"), "candidate.start_state.positions")
    if len(start_joint_names) != len(start_positions):
        raise ValueError("candidate.start_state joint_names/positions length mismatch.")
    require_dict(candidate.get("planning_pose"), "candidate.planning_pose")
    trajectory = require_dict(candidate.get("trajectory"), "candidate.trajectory")
    trajectory_joint_names = require_list(trajectory.get("joint_names"), "candidate.trajectory.joint_names")
    points = require_list(trajectory.get("points"), "candidate.trajectory.points")
    for index, point in enumerate(points):
        point = require_dict(point, f"candidate.trajectory.points[{index}]")
        if _finite_float_or_none(point.get("t")) is None:
            raise ValueError(f"candidate.trajectory.points[{index}].t is required.")
        q = require_list(point.get("q"), f"candidate.trajectory.points[{index}].q")
        if len(q) != len(trajectory_joint_names):
            raise ValueError(f"candidate.trajectory.points[{index}].q length mismatch.")

    forbidden_top_level = {"feature_names", "features", "note", "candidate_rank", "target_tomato_index"}
    present_forbidden = sorted(key for key in forbidden_top_level if key in record)
    if present_forbidden:
        raise ValueError(f"forbidden label fields are present: {present_forbidden}")
    forbidden_candidate_fields = {"entry", "collision_scene", "tomato_rotation", "trajectory_endpoints"}
    present_candidate_forbidden = sorted(key for key in forbidden_candidate_fields if key in candidate)
    if present_candidate_forbidden:
        raise ValueError(f"forbidden candidate fields are present: {present_candidate_forbidden}")

    validate_no_null_or_nonfinite(record)


def build_trajectory_label_record(
    diagnostics_payload: dict[str, Any],
    candidate_record: dict[str, Any],
    label: str,
    human_score: float | None = None,
    candidate_rank: int | None = None,
) -> dict[str, Any]:
    label_normalized, score = _normalize_label_and_score(label, human_score)
    sample_id, planning_request_id, candidate_id = _ids_for_label(
        diagnostics_payload,
        candidate_record,
        candidate_rank,
    )
    record = {
        "schema": LABEL_SCHEMA,
        "sample_id": sample_id,
        "planning_request_id": planning_request_id,
        "candidate_id": candidate_id,
        "stamp_unix": time.time(),
        "label": label_normalized,
        "human_score": score,
        "context": _context_from_payload(diagnostics_payload),
        "candidate": _candidate_for_label(candidate_record),
    }
    safe_record = _json_safe(record)
    validate_trajectory_label_record(safe_record)
    return safe_record


def make_label_sample(
    diagnostics_payload: dict[str, Any],
    candidate_record: dict[str, Any],
    label: str,
    human_score: float | None = None,
    note: str = "",
    candidate_rank: int | None = None,
) -> dict[str, Any]:
    _ = note
    return build_trajectory_label_record(
        diagnostics_payload,
        candidate_record,
        label=label,
        human_score=human_score,
        candidate_rank=candidate_rank,
    )


def iter_diagnostic_candidates(payload: dict[str, Any]) -> list[dict[str, Any]]:
    candidates = payload.get("valid_candidates", [])
    if isinstance(candidates, list) and candidates:
        return [candidate for candidate in candidates if isinstance(candidate, dict)]
    selected = payload.get("selected")
    if isinstance(selected, dict):
        return [selected]
    return []


def weak_label_samples_from_diagnostics(
    diagnostics_records: list[dict[str, Any]],
    good_fraction: float = 0.25,
    bad_fraction: float = 0.25,
    metric: str = "trajectory_duration_sec",
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for payload_index, payload in enumerate(diagnostics_records):
        candidates = iter_diagnostic_candidates(payload)
        if not candidates:
            continue

        def metric_value(candidate: dict[str, Any]) -> float:
            metrics = candidate.get("trajectory_metrics", {})
            if not isinstance(metrics, dict):
                metrics = {}
            return _safe_float(metrics.get(metric), default=float("inf"))

        ordered = sorted(enumerate(candidates, start=1), key=lambda item: metric_value(item[1]))
        count = len(ordered)
        good_count = max(1, int(round(count * max(0.0, min(1.0, good_fraction)))))
        bad_count = max(1, int(round(count * max(0.0, min(1.0, bad_fraction)))))
        good_ranks = ordered[:good_count]
        bad_ranks = ordered[-bad_count:]
        bad_rank_ids = {rank for rank, _ in bad_ranks}
        for rank, candidate in good_ranks:
            samples.append(
                make_label_sample(
                    payload,
                    candidate,
                    "good",
                    human_score=1.0,
                    note=f"weak_label_top_{good_fraction:.2f}_by_{metric}_payload_{payload_index}",
                    candidate_rank=rank,
                )
            )
        for rank, candidate in bad_ranks:
            if rank in bad_rank_ids:
                samples.append(
                    make_label_sample(
                        payload,
                        candidate,
                        "bad",
                        human_score=0.0,
                        note=f"weak_label_bottom_{bad_fraction:.2f}_by_{metric}_payload_{payload_index}",
                        candidate_rank=rank,
                    )
                )
    return samples


def _sigmoid(values: np.ndarray) -> np.ndarray:
    values = np.clip(values, -50.0, 50.0)
    return 1.0 / (1.0 + np.exp(-values))


def train_logistic_ranker(
    samples: list[dict[str, Any]],
    learning_rate: float = 0.08,
    epochs: int = 600,
    l2: float = 0.001,
) -> dict[str, Any]:
    rows = []
    labels = []
    for sample in samples:
        features = sample.get("features")
        if not isinstance(features, list) or len(features) != len(FEATURE_NAMES):
            features = extract_features(sample.get("candidate", {}))
        rows.append([_safe_float(value) for value in features])
        labels.append(1.0 if _safe_float(sample.get("human_score")) >= 0.5 else 0.0)
    if not rows:
        raise ValueError("No training samples were provided.")

    x = np.array(rows, dtype=float)
    y = np.array(labels, dtype=float)
    mean = np.mean(x, axis=0)
    std = np.std(x, axis=0)
    std[std < 1e-9] = 1.0
    x_norm = (x - mean) / std
    weights = np.zeros(x_norm.shape[1], dtype=float)
    bias = 0.0
    history = []
    for epoch in range(max(1, int(epochs))):
        logits = x_norm @ weights + bias
        pred = _sigmoid(logits)
        error = pred - y
        grad_w = (x_norm.T @ error) / len(y) + float(l2) * weights
        grad_b = float(np.mean(error))
        weights -= float(learning_rate) * grad_w
        bias -= float(learning_rate) * grad_b
        if epoch in {0, int(epochs) - 1} or (epoch + 1) % 100 == 0:
            eps = 1e-9
            loss = -float(np.mean(y * np.log(pred + eps) + (1.0 - y) * np.log(1.0 - pred + eps)))
            accuracy = float(np.mean((pred >= 0.5) == (y >= 0.5)))
            history.append({"epoch": epoch + 1, "loss": loss, "accuracy": accuracy})

    final_pred = _sigmoid(x_norm @ weights + bias)
    return {
        "schema": "tomato_trajectory_selector.linear_logistic.v1",
        "created_unix": time.time(),
        "feature_names": list(FEATURE_NAMES),
        "mean": mean.tolist(),
        "std": std.tolist(),
        "weights": weights.tolist(),
        "bias": float(bias),
        "training": {
            "sample_count": int(len(y)),
            "positive_count": int(np.sum(y >= 0.5)),
            "negative_count": int(np.sum(y < 0.5)),
            "final_accuracy": float(np.mean((final_pred >= 0.5) == (y >= 0.5))),
            "history": history,
        },
    }


def predict_good_probability(candidate_record: dict[str, Any], model: dict[str, Any]) -> float:
    feature_names = list(model.get("feature_names", FEATURE_NAMES))
    if feature_names != FEATURE_NAMES:
        # Current extractor is schema-bound; refuse silent feature reorder.
        raise ValueError("Model feature schema does not match the current trajectory feature extractor.")
    features = np.array(extract_features(candidate_record), dtype=float)
    mean = np.array(model["mean"], dtype=float)
    std = np.array(model["std"], dtype=float)
    weights = np.array(model["weights"], dtype=float)
    bias = float(model["bias"])
    if features.shape != weights.shape:
        raise ValueError(f"Feature shape {features.shape} does not match model weights {weights.shape}.")
    normalized = (features - mean) / std
    return float(_sigmoid(np.array([float(normalized @ weights + bias)]))[0])


def rank_candidates(payload: dict[str, Any], model: dict[str, Any]) -> list[dict[str, Any]]:
    ranked = []
    for rank, candidate in enumerate(iter_diagnostic_candidates(payload), start=1):
        probability = predict_good_probability(candidate, model)
        ranked.append(
            {
                "source_rank": rank,
                "learned_good_probability": probability,
                "trajectory_duration_sec": _safe_float(
                    _nested_get(candidate, "trajectory_metrics", "trajectory_duration_sec", default=0.0)
                ),
                "trajectory_score": _safe_float(_nested_get(candidate, "trajectory_metrics", "score", default=0.0)),
                "gripper_spin_deg": _safe_float(candidate.get("gripper_spin_deg")),
                "gripper_x_sign": _safe_float(candidate.get("gripper_x_sign")),
                "candidate": candidate,
            }
        )
    ranked.sort(key=lambda item: (-float(item["learned_good_probability"]), float(item["trajectory_duration_sec"])))
    for sorted_rank, item in enumerate(ranked, start=1):
        item["learned_rank"] = sorted_rank
    return ranked
