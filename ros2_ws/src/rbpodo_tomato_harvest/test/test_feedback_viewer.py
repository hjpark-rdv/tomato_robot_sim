import json

import pytest

from rbpodo_tomato_harvest.feedback_viewer import (
    extended_arrow_start,
    feedback_scene,
    load_feedback,
    robot_side_projection,
    signed_planar_angle_deg,
    stem_callout_position,
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


def test_stem_callout_continues_recommend_approach_through_tomato():
    position = stem_callout_position(
        tomato=(100.0, 100.0),
        recommend=(60.0, 120.0),
        actual_vine=(100.0, 40.0),
        display_distance=50.0,
    )

    # Recommend → tomato points (+40, -20), so the callout must continue
    # in the same direction beyond the tomato rather than straight upward.
    assert position == pytest.approx((144.72136, 77.63932))


def test_robot_side_projection_preserves_left_right_sign():
    # RB link0 +Y is shown on the left side of the graph.
    assert robot_side_projection((0.42, 0.21, 0.47)) == pytest.approx(
        (-0.21, 0.47)
    )
    assert robot_side_projection((0.42, -0.17, 0.56)) == pytest.approx(
        (0.17, 0.56)
    )


def test_load_feedback_reads_json_from_txt(tmp_path):
    path = tmp_path / "feedback.txt"
    path.write_text(json.dumps(sample_payload()), encoding="utf-8")

    assert load_feedback(path)["target_id"] == "detected_tomato_1"
