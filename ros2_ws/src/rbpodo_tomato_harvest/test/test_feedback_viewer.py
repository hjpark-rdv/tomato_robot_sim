import json
import math
import os

import pytest

from rbpodo_tomato_harvest.feedback_viewer import (
    extended_arrow_start,
    feedback_scene,
    feedback_files_newest_first,
    load_feedback,
    projected_metric_radius,
    raw_detection_recommend_point,
    robot_top_projection,
    signed_planar_angle_deg,
    stem_label_position,
)


def sample_payload():
    return {
        "target_id": "detected_tomato_1",
        "camera_id": "17",
        "vision": {
            "frame_id": "camera_color_optical_frame",
            "tomato_xyz": [0.612, -0.184, 1.247],
            "vine_xyz": [0.667, -0.172, 1.273],
        },
        "robot": {
            "plan_success": True,
            "frame_id": "link0",
            "coordinate_source": "detection_tf_snapshot",
            "reference_link": "tomato_gripper_tip",
            "tomato_xyz": [0.55, -0.20, 0.74],
            "vine_xyz": [0.60, -0.18, 0.74],
            "recommend_pregrasp_xyz": [0.48, -0.24, 0.70],
            "final_pregrasp_xyz": [0.50, -0.28, 0.70],
            "recommend_angle_deg": 14.2,
            "final_angle_deg": 36.8,
            "correction_angle_deg": 22.6,
        },
        "review": {
            "status": "REVIEW_REQUIRED",
            "issue": {"code": "VINE_XYZ_MISMATCH", "label": "줄기 좌표 불일치"},
            "note": "줄기 위치 확인 필요",
        },
    }


def test_feedback_scene_extracts_camera_and_robot_frames():
    scene = feedback_scene(sample_payload())

    assert scene["target_id"] == "detected_tomato_1"
    assert scene["camera_frame"] == "camera_color_optical_frame"
    assert scene["planning_frame"] == "link0"
    assert scene["robot_tomato"] == pytest.approx((0.55, -0.20, 0.74))
    assert scene["coordinate_source"] == "detection_tf_snapshot"
    assert scene["recommend"] == pytest.approx((0.48, -0.24, 0.70))
    assert scene["correction_angle_deg"] == pytest.approx(22.6)


def test_feedback_scene_requires_planned_robot_coordinates():
    payload = sample_payload()
    payload["robot"]["final_pregrasp_xyz"] = None

    with pytest.raises(ValueError, match="Plan 완료 후 새 JSON"):
        feedback_scene(payload)


def test_signed_planar_angle_uses_shortest_direction():
    positive = signed_planar_angle_deg((0, 0), (1, 0), (0, 1))
    negative = signed_planar_angle_deg((0, 0), (0, 1), (1, 0))

    assert positive == pytest.approx(90.0)
    assert negative == pytest.approx(-90.0)


def test_grasp_arrow_tail_is_extended_away_from_tomato():
    assert extended_arrow_start((1.0, 2.0), (0.8, 1.9)) == pytest.approx(
        (0.4, 1.7)
    )


def test_recommend_direction_is_opposite_exact_stem_point():
    point = raw_detection_recommend_point(
        tomato=(0.0, 0.0),
        vine=(-1.0, -1.0),
        reference=(0.0, -2.0),
    )

    # Keep the original 2 m display length, but point to the side opposite
    # the exact stem instead of inheriting the tip-height reference offset.
    assert point == pytest.approx((math.sqrt(2.0), math.sqrt(2.0)))


def test_stem_label_extends_from_actual_vine_not_recommend_direction():
    position = stem_label_position(
        tomato=(100.0, 100.0),
        actual_vine=(100.0, 40.0),
        display_distance=50.0,
    )

    # The real marker stays at (100, 40); only its label moves farther along
    # the actual tomato→vine direction.
    assert position == pytest.approx((100.0, -10.0))


def test_robot_top_projection_preserves_xy_approach_angle():
    # Link0 +X is graph-up, while link0 +Y is graph-left.
    assert robot_top_projection((0.42, 0.21, 0.47)) == pytest.approx(
        (-0.21, 0.42)
    )
    assert robot_top_projection((0.42, -0.17, 0.56)) == pytest.approx(
        (0.17, 0.42)
    )


def test_metric_marker_radius_uses_plot_scale_instead_of_fixed_pixels():
    def project(point):
        return (point[0] * 800.0, point[1] * -800.0)

    assert projected_metric_radius(
        project,
        (0.1, 0.2),
        0.00875,
    ) == pytest.approx(7.0)

    assert projected_metric_radius(
        project,
        (0.1, 0.2),
        0.003,
        minimum_px=6.0,
    ) == pytest.approx(6.0)


def test_load_feedback_reads_json_from_txt(tmp_path):
    path = tmp_path / "feedback.txt"
    path.write_text(json.dumps(sample_payload()), encoding="utf-8")

    assert load_feedback(path)["target_id"] == "detected_tomato_1"


def test_feedback_files_are_sorted_newest_first(tmp_path):
    oldest = tmp_path / "old_feedback.txt"
    newest = tmp_path / "new_feedback.json"
    ignored = tmp_path / "notes.csv"
    oldest.write_text("{}", encoding="utf-8")
    newest.write_text("{}", encoding="utf-8")
    ignored.write_text("ignored", encoding="utf-8")
    os.utime(oldest, ns=(1_000_000_000, 1_000_000_000))
    os.utime(newest, ns=(2_000_000_000, 2_000_000_000))

    assert feedback_files_newest_first(tmp_path) == [newest, oldest]
