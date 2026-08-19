from collections import deque
import io
import json
import math
import random
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image as PilImage

import rbpodo_tomato_harvest.harvest_gui as harvest_gui_module

from farmily_tomato_interfaces.msg import (
    TomatoDetection,
    TomatoDetectionArray,
)
from geometry_msgs.msg import Point, Pose, TransformStamped
from moveit_msgs.msg import RobotState, RobotTrajectory
from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import CameraInfo, CompressedImage, Image as RosImage
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from rbpodo_tomato_harvest.harvest_gui import (
    ANGLE_REFERENCE_CENTER_TO_STEM,
    CAMERA_SOURCE_FAKE,
    CAMERA_SOURCE_REAL,
    GUI_PLANNER_CONFIG,
    GRIPPER_EXTEND_AUTO_STOP_SECONDS,
    HarvestGui,
    NAMED_POSE_STATES,
    PICK_READY_STATES,
    PLANNER_CONFIGS,
    PREPLANNED_BATCH_CONFIG_ENV,
    adaptive_grasp_max_rotation_degrees,
    actual_approach_marker,
    actual_approach_marker_length,
    adaptive_approach_axis_local,
    adaptive_rotation_degrees,
    adaptive_rotation_was_applied,
    batch_harvest_stage_limit,
    cartesian_fallback_summary,
    cancel_all_goals_request,
    camera_service_for_source,
    camera_target_record_text,
    camera_xyz_to_image_pixel,
    concise_plan_report,
    decode_compressed_result_image,
    decode_raw_result_image,
    detection_label_layout,
    detection_message_sorted_by_height,
    detected_tomato_marker_array,
    draw_detection_label_overlay,
    debug_frame_request,
    generate_sweep_cases,
    gripper_output_startup_scripts,
    gripper_relay_power_off_script,
    gripper_relay_power_script,
    harvest_all_jobs,
    harvest_command,
    harvest_failure_summary,
    harvest_result_marker,
    harvest_statistics_record,
    is_critical_process_output,
    linear_motor_pin_sequence,
    linear_motor_pin_values,
    servo_angle_degrees,
    servo_speed_degrees_per_second,
    servo_speed_percent,
    lift_harvest_target_height_mm,
    named_pose_command,
    preplanned_batch_command,
    repeat_cycle_command,
    repeat_forward_distance_m,
    repeat_stage_command,
    result_arrow_length_for_report,
    selection_plot_recommend_screen_direction,
    scene_parameters,
    step_custom_stage_deltas_m,
    stepper_command,
    sweep_execution_duration_text,
    sweep_stage_detail,
    sweep_stage_summary,
    sweep_result_marker,
    tomato_stem_arrow_length,
    tomato_motion_result_text,
    tomato_selection_plot_scene,
    pick_ready_state_for_capture_pose,
    transformed_point_xyz,
)
from rbpodo_tomato_harvest.harvest_planner import (
    CartesianHarvestPlanner,
    HarvestMotionPlan,
    revolute_position_error,
)
import rbpodo_tomato_harvest.harvest_planner as harvest_planner_module
from rbpodo_tomato_harvest.tomato_harvest_preplanned_batch import (
    _candidate_failure_is_retryable,
    _plan_independent_candidate,
)
from rbpodo_tomato_harvest.tomato_harvest_worker import _apply_request


def test_revolute_position_error_treats_full_turn_as_same_joint_pose():
    assert revolute_position_error(
        math.radians(350.0), math.radians(-10.0)
    ) == pytest.approx(0.0, abs=1e-12)
    assert math.degrees(
        revolute_position_error(math.radians(179.0), math.radians(-179.0))
    ) == pytest.approx(2.0)


def test_preplanned_candidate_always_uses_canonical_ready_state():
    calls = []
    planner = SimpleNamespace(
        plan=lambda **kwargs: calls.append(kwargs) or "planned"
    )
    ready_state = object()

    assert _plan_independent_candidate(planner, ready_state) == "planned"
    assert calls == [
        {
            "start_state_override": ready_state,
            "start_pose_override": None,
            "arc_failure_reverse_trajectory": (),
        }
    ]


def test_generate_sweep_cases_stops_when_first_axis_reaches_end():
    cases = generate_sweep_cases(
        start=(0.0, 0.0, 0.0, 0.0),
        end=(0.02, 0.01, 0.0, 10.0),
        step=(0.01, 0.01, 0.0, 5.0),
        randomized=(False, False, False, False),
    )

    assert cases == [
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 5.0),
        (0.0, 0.0, 0.0, 10.0),
        (0.01, 0.01, 0.0, 0.0),
        (0.01, 0.01, 0.0, 5.0),
        (0.01, 0.01, 0.0, 10.0),
    ]


def test_camera_target_record_text_preserves_camera_id_and_xyz():
    record = camera_target_record_text(
        target_frame="detected_tomato_3_tf",
        camera_frame="d435_color_optical_frame",
        camera_id="camera-tomato-17",
        tomato_xyz=(0.1234567894, -0.2, 0.7654321),
        vine_xyz=(0.1534567894, -0.2, 0.7654321),
        detection_timestamp=1786002012.325,
        plan_report={
            "success": True,
            "tomato_frame": "detected_tomato_3_tf",
            "pipeline": "ompl",
            "planner_id": "RRTConnect",
            "adaptive_grasp": {
                "geometric_preferred_rotation_deg": 14.2,
                "applied_rotation_deg": 36.8,
                "selected_rotation_deg": 36.8,
            },
            "approach_geometry": {
                "planning_frame_id": "link0",
                "pregrasp_reference_link": "tomato_gripper_tip",
                "tomato_xyz": [0.55, -0.20, 0.74],
                "vine_xyz": [0.60, -0.18, 0.74],
                "recommend_pregrasp_xyz": [0.518, -0.205, 1.203],
                "recommend_rotation_deg": 0.0,
                "final_pregrasp_xyz": [0.541, -0.253, 1.198],
                "final_rotation_deg": 36.8,
            },
        },
        review_issue="줄기 좌표 불일치",
        review_note="영상보다 오른쪽으로 검출됨",
    )
    payload = json.loads(record)

    assert set(payload) == {
        "target_id",
        "timestamp",
        "camera_id",
        "vision",
        "robot",
        "review",
    }
    assert payload["target_id"] == "detected_tomato_3"
    assert payload["timestamp"] == pytest.approx(1786002012.325)
    assert payload["camera_id"] == "camera-tomato-17"
    assert payload["vision"]["frame_id"] == "d435_color_optical_frame"
    assert payload["vision"]["tomato_xyz"] == pytest.approx(
        [0.123456789, -0.2, 0.7654321]
    )
    assert payload["vision"]["vine_xyz"] == pytest.approx(
        [0.153456789, -0.2, 0.7654321]
    )
    assert payload["vision"]["calyx_xyz"] is None
    assert payload["vision"]["angle_reference"] == "center_to_stem"
    assert math.dist(
        payload["robot"]["tomato_xyz"],
        payload["robot"]["vine_xyz"],
    ) == pytest.approx(payload["vision"]["tomato_vine_distance_m"])
    assert payload["robot"]["recommend_angle_deg"] == pytest.approx(0.0)
    assert payload["robot"]["final_angle_deg"] == pytest.approx(36.8)
    assert payload["robot"]["correction_angle_deg"] == pytest.approx(36.8)
    assert set(payload["robot"]) == {
        "plan_success",
        "frame_id",
        "coordinate_source",
        "reference_link",
        "tomato_xyz",
        "calyx_xyz",
        "vine_xyz",
        "angle_reference",
        "angle_origin_xyz",
        "angle_target_xyz",
        "recommend_pregrasp_xyz",
        "recommend_angle_deg",
        "final_pregrasp_xyz",
        "final_angle_deg",
        "correction_angle_deg",
    }
    assert payload["review"]["status"] == "REVIEW_REQUIRED"
    assert payload["review"]["issue"]["code"] == (
        "VINE_XYZ_MISMATCH"
    )
    assert record.endswith("\n")


def test_camera_target_record_uses_exact_transformed_detection_points():
    record = camera_target_record_text(
        target_frame="detected_tomato_11_tf",
        camera_frame="d435_color_optical_frame",
        camera_id="C0:T0",
        tomato_xyz=(-0.0104, -0.0986, 0.5498),
        vine_xyz=(-0.0226, -0.0935, 0.5445),
        detection_timestamp=1.0,
        plan_report={
            "success": True,
            "approach_geometry": {
                "planning_frame_id": "link0",
                "tomato_xyz": [0.37, 0.15, 0.56],
                # This is the virtual, horizontal planning direction and must
                # not replace the exact transformed detection stem point.
                "vine_xyz": [0.36, 0.17, 0.56],
            },
        },
        robot_frame_id="link0",
        robot_tomato_xyz=(0.375, 0.150, 0.561),
        robot_vine_xyz=(0.370, 0.163, 0.555),
    )
    payload = json.loads(record)

    assert payload["robot"]["frame_id"] == "link0"
    assert payload["robot"]["coordinate_source"] == (
        "detection_tf_snapshot"
    )
    assert payload["robot"]["tomato_xyz"] == pytest.approx(
        [0.375, 0.150, 0.561]
    )
    assert payload["robot"]["vine_xyz"] == pytest.approx(
        [0.370, 0.163, 0.555]
    )


def test_camera_target_record_preserves_calyx_angle_reference():
    record = camera_target_record_text(
        target_frame="C0:T1",
        camera_frame="camera",
        camera_id="C0:T1",
        tomato_xyz=(0.10, 0.20, 0.30),
        calyx_xyz=(0.11, 0.22, 0.31),
        vine_xyz=(0.15, 0.28, 0.34),
        angle_reference="calyx_to_stem",
        detection_timestamp=1.0,
        robot_frame_id="link0",
        robot_tomato_xyz=(0.50, 0.10, 0.70),
        robot_calyx_xyz=(0.51, 0.12, 0.71),
        robot_vine_xyz=(0.55, 0.18, 0.74),
    )
    payload = json.loads(record)

    assert payload["vision"]["calyx_xyz"] == pytest.approx(
        [0.11, 0.22, 0.31]
    )
    assert payload["robot"]["calyx_xyz"] == pytest.approx(
        [0.51, 0.12, 0.71]
    )
    assert payload["robot"]["angle_reference"] == "calyx_to_stem"


def test_camera_target_record_preserves_base_to_center_angle_segment():
    record = camera_target_record_text(
        target_frame="C0:T2",
        camera_frame="camera",
        camera_id="C0:T2",
        tomato_xyz=(0.10, 0.20, 0.30),
        vine_xyz=(0.15, 0.28, 0.34),
        angle_reference="base_to_center",
        detection_timestamp=1.0,
        robot_frame_id="link0",
        robot_tomato_xyz=(0.50, 0.10, 0.70),
        robot_vine_xyz=(0.55, 0.18, 0.74),
        robot_angle_origin_xyz=(0.0, 0.0, 0.0),
        robot_angle_target_xyz=(0.50, 0.10, 0.70),
    )
    payload = json.loads(record)

    assert payload["robot"]["angle_reference"] == "base_to_center"
    assert payload["robot"]["angle_origin_xyz"] == pytest.approx(
        [0.0, 0.0, 0.0]
    )
    assert payload["robot"]["angle_target_xyz"] == pytest.approx(
        [0.50, 0.10, 0.70]
    )


def test_camera_target_record_text_rejects_nonfinite_coordinates():
    with pytest.raises(ValueError, match="유한한 X, Y, Z"):
        camera_target_record_text(
            target_frame="detected_tomato_0_tf",
            camera_frame="camera",
            camera_id="0",
            tomato_xyz=(0.1, math.nan, 0.3),
            vine_xyz=(0.2, 0.3, 0.4),
            detection_timestamp=1.0,
        )


def test_selected_tomato_plot_omits_final_until_plan_report_exists():
    without_plan = tomato_selection_plot_scene(
        tomato_xyz=(0.4, 0.2, 0.6),
        vine_xyz=(0.4, 0.22, 0.59),
    )
    with_plan = tomato_selection_plot_scene(
        tomato_xyz=(0.4, 0.2, 0.6),
        vine_xyz=(0.4, 0.22, 0.59),
        plan_report={
            "approach_geometry": {
                "final_pregrasp_xyz": [0.35, 0.17, 0.58],
            }
        },
    )

    assert without_plan["final"] is None
    assert with_plan["final"] == pytest.approx((0.35, 0.17, 0.58))
    assert math.dist(
        without_plan["robot_tomato"],
        without_plan["recommend"],
    ) == pytest.approx(0.04)


@pytest.mark.parametrize(
    ("capture_pose", "default_state", "expected"),
    [
        ("CAPTURE_RIGHT", "PICK_READY", "PICK_READY_RIGHT"),
        ("CAPTURE_LEFT", "PICK_READY_RIGHT", "PICK_READY"),
        (None, "PICK_READY_RIGHT", "PICK_READY_RIGHT"),
    ],
)
def test_capture_pose_selects_matching_pick_ready_state(
    capture_pose,
    default_state,
    expected,
):
    assert pick_ready_state_for_capture_pose(
        capture_pose,
        default_state,
    ) == expected


@pytest.mark.parametrize(
    ("target_x", "expected"),
    [(-0.42, "PICK_READY_RIGHT"), (0.42, "PICK_READY")],
)
def test_target_side_restores_ready_state_when_capture_metadata_is_missing(
    target_x,
    expected,
):
    assert pick_ready_state_for_capture_pose(
        None,
        "PICK_READY",
        target_x,
    ) == expected


def test_selected_tomato_plot_final_uses_same_local_vector_as_rviz_marker():
    tomato = (0.3461338312, 0.0992415167, 0.5873050332)
    stem = (0.6813608041, -0.8428958812, 0.5873050332)
    scene = tomato_selection_plot_scene(
        tomato_xyz=tomato,
        vine_xyz=stem,
        angle_origin_xyz=tomato,
        angle_target_xyz=stem,
        plan_report={
            "approach_geometry": {
                # Deliberately stale absolute coordinates must not affect the
                # embedded graph when RViz-local geometry is available.
                "final_pregrasp_xyz": [9.0, 9.0, 9.0],
                "preapproach_position": [-0.035, 0.0, -0.018],
            }
        },
    )

    direction = np.asarray(stem[:2]) - np.asarray(tomato[:2])
    direction /= np.linalg.norm(direction)
    expected_xy = np.asarray(tomato[:2]) + direction * -0.035
    assert scene["final"] == pytest.approx(
        (expected_xy[0], expected_xy[1], tomato[2] - 0.018)
    )


def test_selected_tomato_plot_uses_calyx_to_stem_direction():
    scene = tomato_selection_plot_scene(
        tomato_xyz=(0.4, 0.2, 0.6),
        vine_xyz=(0.43, 0.24, 0.6),
        calyx_xyz=(0.43, 0.20, 0.6),
        angle_origin_xyz=(0.43, 0.20, 0.6),
    )

    # calyx→stem is +Y, so Recommend is 40 mm along -Y from the tomato.
    assert scene["recommend"] == pytest.approx((0.4, 0.16, 0.6))
    assert scene["robot_angle_origin"] == pytest.approx((0.43, 0.20, 0.6))
    assert scene["robot_calyx"] == pytest.approx((0.43, 0.20, 0.6))


def test_selected_tomato_plot_uses_base_to_center_direction():
    scene = tomato_selection_plot_scene(
        tomato_xyz=(0.4, 0.2, 0.6),
        vine_xyz=(0.43, 0.24, 0.6),
        angle_origin_xyz=(0.0, 0.0, 0.0),
        angle_target_xyz=(0.4, 0.2, 0.6),
    )

    direction = np.asarray((0.4, 0.2, 0.6), dtype=float)
    direction /= np.linalg.norm(direction)
    expected = np.asarray((0.4, 0.2, 0.6)) - direction * 0.04
    assert scene["recommend"] == pytest.approx(expected)


def test_camera_overlay_direction_matches_selected_plot_top_view():
    direction = selection_plot_recommend_screen_direction(
        tomato_xyz=(0.50, 0.10, 0.70),
        angle_origin_xyz=(0.50, 0.10, 0.70),
        angle_target_xyz=(0.56, 0.14, 0.72),
    )

    # PlotCanvas maps Link0 (X,Y) to screen (-Y,-X).  The arrow points from
    # Recommend toward the tomato, i.e. along origin→target in X-Y.
    expected = np.asarray((-0.04, -0.06), dtype=float)
    expected /= np.linalg.norm(expected)
    assert direction == pytest.approx(expected)


def test_camera_overlay_direction_applies_same_right_capture_half_turn():
    direction = selection_plot_recommend_screen_direction(
        tomato_xyz=(-0.50, 0.10, 0.70),
        angle_origin_xyz=(-0.50, 0.10, 0.70),
        angle_target_xyz=(-0.44, 0.14, 0.72),
    )

    expected = np.asarray((0.04, 0.06), dtype=float)
    expected /= np.linalg.norm(expected)
    assert direction == pytest.approx(expected)


def test_debug_frame_request_preserves_feedback_json_text():
    contents = '{"target_id":"detected_tomato_2","review":{"note":"확인"}}\n'

    request = debug_frame_request(contents)

    assert request.request_text == contents


@pytest.mark.parametrize("contents", ["not-json", "[]", "null"])
def test_debug_frame_request_rejects_invalid_payload(contents):
    with pytest.raises(ValueError):
        debug_frame_request(contents)


def test_camera_target_record_prefers_running_target_then_selection():
    gui = SimpleNamespace(
        active_camera_target_index=2,
        detected_tomatoes=[object(), object(), object()],
        _selected_index=lambda: 1,
    )

    assert HarvestGui._camera_target_index_for_record(gui) == 2
    gui.active_camera_target_index = None
    assert HarvestGui._camera_target_index_for_record(gui) == 1


def test_sweep_execution_duration_text_distinguishes_execution_from_plan_only():
    assert sweep_execution_duration_text(
        {
            "execution_attempted": True,
            "execution_duration_sec": 18.376,
        }
    ) == "18.38"
    assert sweep_execution_duration_text(
        {
            "execution_attempted": False,
            "execution_duration_sec": 0.0,
        }
    ) == "-"


def test_tomato_motion_result_text_distinguishes_plan_and_execution_failures():
    assert tomato_motion_result_text(False, True, {}) == "Plan 성공"
    assert tomato_motion_result_text(False, False, {}) == "Plan 실패"
    assert tomato_motion_result_text(True, True, {}) == "수확 성공"
    assert tomato_motion_result_text(
        True,
        False,
        {"execution_attempted": False},
    ) == "실행 전 재계획 실패"
    assert tomato_motion_result_text(
        True,
        False,
        {"execution_attempted": True},
    ) == "수확 실행 실패"
    assert tomato_motion_result_text(
        True,
        True,
        {"step_cycle_only": True, "step_cycle_last_stage": 4},
    ) == "4단계 실행 성공"


def test_harvest_failure_summary_includes_stage_and_reason():
    assert harvest_failure_summary(
        {
            "failure_stage": "DYNAMIC_BASE_TF_SYNC",
            "failure_reason": "FRESH_LIFT_TF_NOT_RECEIVED",
        }
    ) == "DYNAMIC_BASE_TF_SYNC / FRESH_LIFT_TF_NOT_RECEIVED"


def test_lift_node_name_is_qualified_with_namespace():
    assert HarvestGui._qualified_node_name(
        "lift_controller_node", "/"
    ) == "/lift_controller_node"
    assert HarvestGui._qualified_node_name(
        "lift_controller_node", "/farmily"
    ) == "/farmily/lift_controller_node"


def test_lift_node_detection_matches_fully_qualified_ros_name():
    gui = SimpleNamespace(
        lift_node_name="/lift_controller_node",
        _qualified_node_name=HarvestGui._qualified_node_name,
        get_node_names_and_namespaces=lambda: [
            ("move_group", "/"),
            ("lift_controller_node", "/"),
        ],
    )

    assert HarvestGui._lift_node_is_running(gui) is True


def test_lift_height_parser_accepts_nonnegative_mm_only():
    assert HarvestGui._parse_lift_height("125.5") == pytest.approx(125.5)
    assert HarvestGui._parse_lift_height("0") == pytest.approx(0.0)
    with pytest.raises(ValueError, match="0 mm 이상"):
        HarvestGui._parse_lift_height("-0.1")
    with pytest.raises(ValueError, match="mm 단위 숫자"):
        HarvestGui._parse_lift_height("높이")
    with pytest.raises(ValueError, match="750 mm 이하"):
        HarvestGui._parse_lift_height("751")


def test_lift_harvest_target_is_40cm_below_tomato_and_bounded():
    assert lift_harvest_target_height_mm(0.55) == pytest.approx(150.0)
    assert lift_harvest_target_height_mm(0.10) == pytest.approx(0.0)
    assert lift_harvest_target_height_mm(1.20) == pytest.approx(750.0)


def test_lift_harvest_waits_for_target_before_starting_plan():
    published = []
    ready = []
    scheduled = []
    gui = SimpleNamespace(
        root=SimpleNamespace(
            after=lambda delay, callback: scheduled.append(callback)
        ),
        _tomato_world_height_m=lambda index: 0.55,
        lift_harvest_offset_m=0.40,
        lift_minimum_height_mm=0.0,
        lift_maximum_height_mm=750.0,
        lift_target_tolerance_mm=5.0,
        lift_move_timeout_sec=60.0,
        detected_tf_sync_tolerance_m=0.003,
        detected_tf_sync_timeout_sec=8.0,
        _detected_tf_generation_is_ready=lambda: True,
        _detected_tomato_tf_sync_error_m=lambda index: 0.0,
        lift_target_height=SimpleNamespace(set=lambda value: None),
        lift_last_height_mm=100.0,
        lift_simulation_mode=True,
        _append_log=lambda message: None,
        _publish_automatic_lift_target=lambda height: published.append(height),
        status=SimpleNamespace(set=lambda value: None),
        lift_harvest_pending=None,
    )
    gui._prepare_lift_for_tomato = lambda *args: (
        HarvestGui._prepare_lift_for_tomato(gui, *args)
    )

    HarvestGui._prepare_lift_for_tomato(
        gui,
        2,
        True,
        lambda: ready.append(True),
        lambda message: pytest.fail(message),
    )

    assert published == [pytest.approx(150.0)]
    assert ready == []
    gui.lift_last_height_mm = 150.0
    HarvestGui._poll_lift_harvest_target(gui, gui.lift_harvest_pending)
    assert ready == [True]
    assert gui.lift_harvest_pending is None


def test_lift_harvest_waits_until_detected_tf_matches_new_detection():
    scheduled = []
    published = []
    sync_errors = iter((0.25, 0.0))
    gui = SimpleNamespace(
        root=SimpleNamespace(
            after=lambda delay, callback: scheduled.append(callback)
        ),
        _detected_tomato_tf_sync_error_m=lambda index: next(sync_errors),
        detected_tf_sync_tolerance_m=0.003,
        detected_tf_sync_timeout_sec=8.0,
        _detected_tf_generation_is_ready=lambda: True,
        _tomato_world_height_m=lambda index: 0.55,
        lift_harvest_offset_m=0.40,
        lift_minimum_height_mm=0.0,
        lift_maximum_height_mm=750.0,
        lift_target_tolerance_mm=1.0,
        lift_move_timeout_sec=60.0,
        lift_target_height=SimpleNamespace(set=lambda value: None),
        lift_last_height_mm=0.0,
        lift_simulation_mode=True,
        _append_log=lambda message: None,
        _publish_automatic_lift_target=lambda height: published.append(height),
        status=SimpleNamespace(set=lambda value: None),
        lift_harvest_pending=None,
    )
    gui._prepare_lift_for_tomato = lambda *args: (
        HarvestGui._prepare_lift_for_tomato(gui, *args)
    )

    HarvestGui._prepare_lift_for_tomato(
        gui,
        0,
        True,
        lambda: None,
        lambda message: pytest.fail(message),
    )

    assert published == []
    assert len(scheduled) == 1
    scheduled.pop(0)()
    assert published == [pytest.approx(150.0)]


def test_lift_harvest_waits_for_matching_tf_generation_before_position_check():
    scheduled = []
    checks = []
    ready_states = iter((False, True))
    gui = SimpleNamespace(
        root=SimpleNamespace(
            after=lambda delay, callback: scheduled.append(callback)
        ),
        _detected_tf_generation_is_ready=lambda: next(ready_states),
        _detected_tomato_tf_sync_error_m=lambda index: (
            checks.append(index) or 0.0
        ),
        detected_tf_sync_tolerance_m=0.003,
        detected_tf_sync_timeout_sec=8.0,
        _tomato_world_height_m=lambda index: 0.55,
        lift_harvest_offset_m=0.40,
        lift_minimum_height_mm=0.0,
        lift_maximum_height_mm=750.0,
        lift_target_tolerance_mm=1.0,
        lift_move_timeout_sec=60.0,
        lift_target_height=SimpleNamespace(set=lambda value: None),
        lift_last_height_mm=150.0,
        lift_simulation_mode=True,
        _append_log=lambda message: None,
        _publish_automatic_lift_target=lambda height: None,
        status=SimpleNamespace(set=lambda value: None),
        lift_harvest_pending=None,
    )
    gui._prepare_lift_for_tomato = lambda *args: (
        HarvestGui._prepare_lift_for_tomato(gui, *args)
    )

    HarvestGui._prepare_lift_for_tomato(
        gui,
        0,
        True,
        lambda: None,
        lambda message: pytest.fail(message),
    )

    assert checks == []
    assert len(scheduled) == 1
    scheduled.pop(0)()
    assert checks == [0]


def test_detection_marker_sync_rejects_one_stale_tomato_tf():
    gui = SimpleNamespace(
        detected_tomatoes=[object(), object(), object()],
        detected_tomato_expected_world_positions={
            0: (0.5, 0.1, 0.7),
            1: (0.5, 0.0, 0.6),
            2: (0.5, -0.1, 0.5),
        },
        detected_tomato_expected_world_x_axes={
            0: (1.0, 0.0),
            1: (1.0, 0.0),
            2: (1.0, 0.0),
        },
        detected_tf_sync_tolerance_m=0.003,
        detected_tf_sync_orientation_tolerance_deg=2.0,
        _detected_tomato_tf_sync_error_m=lambda index: (
            0.075 if index == 2 else 0.0001
        ),
        _detected_tomato_tf_orientation_error_deg=lambda index: 0.0,
    )

    synchronized, reason = (
        HarvestGui._detected_tf_positions_are_synchronized(gui)
    )

    assert synchronized is False
    assert "detected_tomato_2_tf" in reason
    assert "75.0mm" in reason


def test_detection_marker_sync_rejects_stale_tomato_tf_direction():
    gui = SimpleNamespace(
        detected_tomatoes=[object()],
        detected_tomato_expected_world_positions={0: (0.5, 0.1, 0.7)},
        detected_tomato_expected_world_x_axes={0: (1.0, 0.0)},
        detected_tf_sync_tolerance_m=0.003,
        detected_tf_sync_orientation_tolerance_deg=2.0,
        _detected_tomato_tf_sync_error_m=lambda index: 0.0001,
        _detected_tomato_tf_orientation_error_deg=lambda index: 90.0,
    )

    synchronized, reason = (
        HarvestGui._detected_tf_positions_are_synchronized(gui)
    )

    assert synchronized is False
    assert "X축 오차 90.0°" in reason


def test_sweep_tf_sync_failure_is_recorded_without_stopping_sweep():
    events = []
    verification = (3, 0, "ompl", "RRTConnect", "cartesian")
    gui = SimpleNamespace(
        lift_harvest_pending=object(),
        _append_log=lambda message: events.append(("log", message)),
        _handle_sweep_plan_done=lambda code, checked, report: events.append(
            ("result", code, checked, report)
        ),
        _handle_lift_preparation_error=lambda message: events.append(
            ("abort", message)
        ),
    )

    HarvestGui._handle_sweep_lift_preparation_error(
        gui,
        0,
        verification,
        "detected_tomato_0_tf가 새 검출 위치로 갱신되지 않았습니다",
    )

    assert not any(event[0] == "abort" for event in events)
    result = next(event for event in events if event[0] == "result")
    assert result[1] == 1
    assert result[2] == verification
    assert result[3]["failure_stage"] == "LIFT_TF_SYNC"
    assert result[3]["suppress_result_marker"] is True


def test_transformed_point_xyz_applies_rotation_and_translation():
    transform = TransformStamped()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.translation.z = 3.0
    transform.transform.rotation.z = math.sqrt(0.5)
    transform.transform.rotation.w = math.sqrt(0.5)

    result = transformed_point_xyz(Point(x=0.5, y=0.0, z=0.25), transform)

    assert result == pytest.approx((1.0, 2.5, 3.25))


def test_lift_height_callback_updates_realtime_display_and_calibration():
    values = {}
    gui = SimpleNamespace(
        lift_last_height_mm=None,
        lift_simulation_mode=False,
        lift_calibration_active=False,
        lift_calibrated=False,
        lift_current_height=SimpleNamespace(
            set=lambda value: values.__setitem__("height", value)
        ),
        lift_calibration_status=SimpleNamespace(
            set=lambda value: values.__setitem__("calibration", value)
        ),
        _update_lift_controls=lambda: values.__setitem__("updated", True),
    )

    HarvestGui._lift_current_height_callback(gui, Float64(data=123.456))

    assert gui.lift_last_height_mm == pytest.approx(123.456)
    assert gui.lift_calibrated is True
    assert values == {
        "height": "123.46 mm",
        "calibration": "Calibration 완료",
        "updated": True,
    }


def test_successful_execution_verification_remains_reusable_for_same_target():
    verification = (
        7,
        2,
        "ompl",
        "RRTConnect",
        "cartesian",
        "PICK_READY_RIGHT",
    )
    gui = SimpleNamespace(
        detection_generation=7,
        _selected_index=lambda: 2,
        _selected_planner_config=lambda: (
            "ompl",
            "RRTConnect",
            "cartesian",
        ),
        _selected_pick_ready_state=lambda: "PICK_READY_RIGHT",
    )

    assert HarvestGui._verification_matches_current_selection(
        gui,
        verification,
    ) is True
    assert HarvestGui._verification_matches_current_selection(
        gui,
        (6, 2, "ompl", "RRTConnect", "cartesian", "PICK_READY_RIGHT"),
    ) is False


def test_plan_verification_changes_when_ready_state_changes():
    gui = SimpleNamespace(
        detection_generation=7,
        _selected_index=lambda: 2,
        _selected_planner_config=lambda: GUI_PLANNER_CONFIG,
        _selected_pick_ready_state=lambda: "PICK_READY_RIGHT",
    )

    assert HarvestGui._verification_matches_current_selection(
        gui,
        (7, 2, *GUI_PLANNER_CONFIG, "PICK_READY"),
    ) is False


def test_plan_verification_changes_when_adaptive_grasp_options_change():
    gui = SimpleNamespace(
        detection_generation=7,
        _selected_index=lambda: 2,
        _selected_planner_config=lambda: GUI_PLANNER_CONFIG,
        _selected_pick_ready_state=lambda: "PICK_READY_RIGHT",
        _adaptive_grasp_options=lambda: (True, 35.0),
    )

    assert HarvestGui._verification_matches_current_selection(
        gui,
        (7, 2, *GUI_PLANNER_CONFIG, "PICK_READY_RIGHT", True, 35.0),
    ) is True
    assert HarvestGui._verification_matches_current_selection(
        gui,
        (7, 2, *GUI_PLANNER_CONFIG, "PICK_READY_RIGHT", False, 35.0),
    ) is False


def test_step_plan_verification_handles_wrist_checkbox_as_boolean_option():
    wrist_enabled = SimpleNamespace(get=lambda: False)
    gui = SimpleNamespace(
        detection_generation=7,
        _selected_index=lambda: 2,
        _selected_planner_config=lambda: GUI_PLANNER_CONFIG,
        _selected_pick_ready_state=lambda: "PICK_READY_RIGHT",
        _adaptive_grasp_options=lambda: (False, 45.0),
        step_tcp_wrist_oscillation_enabled_var=wrist_enabled,
    )
    verification = (
        7,
        2,
        *GUI_PLANNER_CONFIG,
        "PICK_READY_RIGHT",
        False,
        45.0,
        False,
    )

    assert HarvestGui._verification_matches_current_selection(
        gui,
        verification,
    ) is True
    wrist_enabled.get = lambda: True
    assert HarvestGui._verification_matches_current_selection(
        gui,
        verification,
    ) is False
    assert HarvestGui._verification_matches_current_selection(
        gui,
        (7, 2, *GUI_PLANNER_CONFIG, "PICK_READY_RIGHT", True, 45.0),
    ) is False


def test_step_plan_verification_handles_wrist_and_wave_boolean_options():
    wrist_enabled = SimpleNamespace(get=lambda: False)
    wave_enabled = SimpleNamespace(get=lambda: True)
    gui = SimpleNamespace(
        detection_generation=7,
        _selected_index=lambda: 2,
        _selected_planner_config=lambda: GUI_PLANNER_CONFIG,
        _selected_pick_ready_state=lambda: "PICK_READY_RIGHT",
        _adaptive_grasp_options=lambda: (False, 45.0),
        step_tcp_wrist_oscillation_enabled_var=wrist_enabled,
        step_forward_wave_enabled_var=wave_enabled,
    )
    verification = (
        7,
        2,
        *GUI_PLANNER_CONFIG,
        "PICK_READY_RIGHT",
        False,
        45.0,
        False,
        True,
    )

    assert HarvestGui._verification_matches_current_selection(
        gui,
        verification,
    ) is True
    wave_enabled.get = lambda: False
    assert HarvestGui._verification_matches_current_selection(
        gui,
        verification,
    ) is False


def test_plan_verification_changes_when_harvest_forward_distance_changes():
    gui = SimpleNamespace(
        detection_generation=7,
        _selected_index=lambda: 2,
        _selected_planner_config=lambda: GUI_PLANNER_CONFIG,
        _selected_pick_ready_state=lambda: "PICK_READY_RIGHT",
        _adaptive_grasp_options=lambda: (False, 45.0),
        harvest_forward_distance_mm_var=SimpleNamespace(get=lambda: "35"),
    )

    assert HarvestGui._verification_matches_current_selection(
        gui,
        (
            7,
            2,
            *GUI_PLANNER_CONFIG,
            "PICK_READY_RIGHT",
            False,
            45.0,
            0.035,
        ),
    ) is True
    assert HarvestGui._verification_matches_current_selection(
        gui,
        (
            7,
            2,
            *GUI_PLANNER_CONFIG,
            "PICK_READY_RIGHT",
            False,
            45.0,
            0.040,
        ),
    ) is False


def test_empty_plan_verification_does_not_interrupt_busy_state_release():
    gui = SimpleNamespace(
        detection_generation=7,
        _selected_index=lambda: 2,
        _selected_planner_config=lambda: GUI_PLANNER_CONFIG,
        _selected_pick_ready_state=lambda: "PICK_READY_RIGHT",
    )

    assert HarvestGui._verification_matches_current_selection(gui, None) is False


def test_tomato_motion_result_updates_current_detection_row():
    values = {}
    gui = SimpleNamespace(
        tomato_motion_results={},
        tomato_tree=SimpleNamespace(
            exists=lambda item: item == "4",
            set=lambda item, column, value: values.__setitem__(
                (item, column),
                value,
            ),
        ),
    )

    HarvestGui._set_tomato_motion_result(gui, 4, "수확 성공")

    assert gui.tomato_motion_results == {4: "수확 성공"}
    assert values[("4", "motion_result")] == "수확 성공"


def test_lift_bottom_status_callback_enables_height_after_success():
    values = {}
    gui = SimpleNamespace(
        lift_simulation_mode=False,
        lift_calibration_active=True,
        lift_calibrated=False,
        lift_calibration_status=SimpleNamespace(
            set=lambda value: values.__setitem__("status", value)
        ),
        _append_log=lambda message: values.__setitem__("log", message),
        _update_lift_controls=lambda: values.__setitem__("updated", True),
    )

    HarvestGui._lift_bottom_status_callback(gui, Bool(data=True))

    assert gui.lift_calibration_active is False
    assert gui.lift_calibrated is True
    assert values["status"] == "Calibration 완료"
    assert "10 mm 후퇴 완료" in values["log"]
    assert values["updated"] is True


def test_lift_stop_publishes_immediately_and_cancels_calibration_state():
    published = []
    values = {}
    gui = SimpleNamespace(
        lift_node_online=True,
        lift_simulation_mode=False,
        lift_stop_topic="/lift_control/stop",
        count_subscribers=lambda topic: 1,
        lift_calibration_active=True,
        lift_calibrated=True,
        lift_stop_publisher=SimpleNamespace(
            publish=lambda message: published.append(message)
        ),
        lift_calibration_status=SimpleNamespace(
            set=lambda value: values.__setitem__("calibration", value)
        ),
        _update_lift_controls=lambda: values.__setitem__("updated", True),
        _append_log=lambda message: values.__setitem__("log", message),
        status=SimpleNamespace(
            set=lambda value: values.__setitem__("status", value)
        ),
    )

    HarvestGui.stop_lift_motion(gui)

    assert len(published) == 1
    assert published[0].data is True
    assert gui.lift_calibration_active is False
    assert gui.lift_calibrated is False
    assert values["calibration"] == "Calibration 중지됨"
    assert values["status"] == "리프트 이동 정지 명령 전송"
    assert values["updated"] is True


def test_lift_simulation_height_moves_rviz_without_real_publisher():
    published = []
    values = {}
    gui = SimpleNamespace(
        lift_target_height=SimpleNamespace(get=lambda: "325.0"),
        _parse_lift_height=HarvestGui._parse_lift_height,
        lift_simulation_mode=True,
        lift_simulated_move_height_topic=(
            "/lift_simulation/control/move_height"
        ),
        count_subscribers=lambda topic: 1,
        lift_simulated_move_height_publisher=SimpleNamespace(
            publish=lambda message: published.append(message)
        ),
        _append_log=lambda message: values.__setitem__("log", message),
        status=SimpleNamespace(
            set=lambda value: values.__setitem__("status", value)
        ),
    )

    HarvestGui.move_lift_to_height(gui)

    assert len(published) == 1
    assert published[0].data == pytest.approx(325.0)
    assert "실제 모터 명령 없음" in values["log"]
    assert "실제 모터 미동작" in values["status"]


def test_harvest_statistics_record_formats_final_batch_execution_result():
    record = harvest_statistics_record(
        case="전체",
        tomato_index=2,
        scene=(0.55, -0.1, 0.4, 30.0),
        success=True,
        verification=(7, 2, "ompl", "RRTConnect", "cartesian"),
        execute_motion=True,
        report={
            "execution_attempted": True,
            "execution_success": True,
            "execution_duration_sec": 12.5,
            "duration_sec": 1.25,
            "stages": [],
            "adaptive_grasp": {"applied_rotation_deg": 45.0},
        },
    )

    assert record["case"] == "전체"
    assert record["tomato"] == 2
    assert record["success"] is True
    assert record["execution_attempted"] is True
    assert record["execution_duration_sec"] == pytest.approx(12.5)
    assert record["display_stage_summary"] == "전체 단계 성공"
    assert "최종 결과: 성공" in record["display_stage_detail"]


def test_concise_plan_report_shows_only_stage_metrics_and_joint_ranges():
    summary = concise_plan_report(
        {
            "success": False,
            "tomato_frame": "detected_tomato_1_tf",
            "adaptive_grasp": {"applied_rotation_deg": 45.0},
            "stages": [
                {
                    "stage": "CARTESIAN_PREAPPROACH",
                    "planner_type": "cartesian",
                    "success": False,
                    "duration_sec": 0.35,
                    "cartesian_fraction": 0.125,
                    "reason": "CARTESIAN_FRACTION_LOW",
                },
                {
                    "stage": "OMPL_FALLBACK_PREAPPROACH_1",
                    "planner_type": "ompl",
                    "success": False,
                    "duration_sec": 2.15,
                    "planning_time_sec": 2.0,
                    "reason": "MOVEIT_PLANNING_FAILED",
                },
            ],
            "failure_stage": "OMPL_FALLBACK_PREAPPROACH_1",
            "failure_reason": "MOVEIT_PLANNING_FAILED",
            "joint_ranges": [
                {
                    "joint_name": "base",
                    "start_deg": 88.4,
                    "min_deg": 80.0,
                    "max_deg": 90.0,
                    "span_deg": 10.0,
                }
            ],
        }
    )

    assert (
        "Plan 실패: detected_tomato_1_tf | "
        "시작/복귀=PICK_READY | 보정=45.0°"
    ) in summary
    assert "소요=0.35s" in summary
    assert "fraction=12.5%" in summary
    assert "소요=2.15s" in summary
    assert "최종 실패: OMPL_FALLBACK_PREAPPROACH_1" in summary
    assert "base: 88.4 / 80.0 / 90.0 / 10.0" in summary


def test_process_output_filter_keeps_tracebacks_not_ros_info_noise():
    assert is_critical_process_output("Traceback (most recent call last):")
    assert is_critical_process_output("ValueError: bad target")
    assert not is_critical_process_output(
        "[INFO] [node]: Cartesian success=True fraction=1.0"
    )
    assert not is_critical_process_output(
        "[WARN] [node]: falling back to OMPL"
    )


def test_generate_sweep_cases_uses_exact_steps_until_earliest_end():
    cases = generate_sweep_cases(
        start=(0.0, 0.0, 0.0, 0.0),
        end=(1.0, 0.6, 0.0, 90.0),
        step=(0.1, 0.3, 0.0, 5.0),
        randomized=(False, False, False, False),
    )

    assert len(cases) == 57
    expected_rotations = [float(value) for value in range(0, 91, 5)]
    assert [case[:3] for case in cases[0:19]] == [(0.0, 0.0, 0.0)] * 19
    assert [case[:3] for case in cases[19:38]] == [(0.1, 0.3, 0.0)] * 19
    assert [case[:3] for case in cases[38:57]] == [(0.2, 0.6, 0.0)] * 19
    assert [case[3] for case in cases[0:19]] == expected_rotations
    assert [case[3] for case in cases[19:38]] == expected_rotations
    assert [case[3] for case in cases[38:57]] == expected_rotations


def test_generate_sweep_cases_randomizes_only_checked_axis():
    cases = generate_sweep_cases(
        start=(0.0, 1.0, 2.0, 0.0),
        end=(0.02, 3.0, 2.0, 10.0),
        step=(0.01, 1.0, 0.0, 5.0),
        randomized=(True, False, False, False),
        rng=random.Random(7),
    )

    assert len(cases) == 9
    assert all(0.0 <= case[0] <= 0.02 for case in cases)
    assert [case[1] for case in cases] == [1.0] * 3 + [2.0] * 3 + [3.0] * 3
    assert [case[2] for case in cases] == [2.0] * 9
    assert [case[3] for case in cases] == [0.0, 5.0, 10.0] * 3


def test_random_axis_step_does_not_control_termination():
    cases = generate_sweep_cases(
        start=(0.0, 0.0, 0.0, 0.0),
        end=(10.0, 0.02, 0.0, 10.0),
        step=(0.0, 0.01, 0.0, 5.0),
        randomized=(True, False, False, False),
        rng=random.Random(3),
    )

    assert len(cases) == 9
    assert all(0.0 <= case[0] <= 10.0 for case in cases)
    assert [case[1] for case in cases] == [0.0] * 3 + [0.01] * 3 + [0.02] * 3
    assert [case[3] for case in cases] == [0.0, 5.0, 10.0] * 3


def test_all_changing_xyz_random_generates_one_position_without_termination_axis():
    cases = generate_sweep_cases(
        start=(0.0, 0.0, 0.0, 0.0),
        end=(1.0, 1.0, 0.0, 0.0),
        step=(0.0, 0.0, 0.0, 0.0),
        randomized=(True, True, False, False),
        rng=random.Random(5),
    )

    assert len(cases) == 1
    assert 0.0 <= cases[0][0] <= 1.0
    assert 0.0 <= cases[0][1] <= 1.0
    assert cases[0][2:] == (0.0, 0.0)


def test_random_rotation_samples_once_for_each_deterministic_xyz_position():
    cases = generate_sweep_cases(
        start=(0.0, 0.0, 0.0, -90.0),
        end=(0.0, 0.02, 0.0, 90.0),
        step=(0.0, 0.01, 0.0, 0.0),
        randomized=(False, False, False, True),
        rng=random.Random(11),
    )

    assert len(cases) == 3
    assert [case[1] for case in cases] == [0.0, 0.01, 0.02]
    assert all(-90.0 <= case[3] <= 90.0 for case in cases)
    assert len({case[3] for case in cases}) == 3


def test_generate_sweep_cases_rejects_zero_step_for_changed_axis():
    with pytest.raises(ValueError):
        generate_sweep_cases(
            (0.0, 0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0),
            (False, False, False, False),
        )


def test_persistent_worker_updates_target_and_disables_display():
    received = []
    joint_names = [
        "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"
    ]
    node = SimpleNamespace(
        tomato_frame="old_frame",
        get_parameter=lambda name: SimpleNamespace(value=joint_names),
        set_parameters=lambda parameters: (
            received.extend(parameters)
            or [SimpleNamespace(successful=True, reason="") for _ in parameters]
        ),
    )

    _apply_request(
        node,
        {
            "tomato_frame": "detected_tomato_7_tf",
            "planning_pipeline_id": "chomp",
            "planner_id": "RRTConnect",
            "preapproach_mode": "planner",
            "velocity_scale": 0.35,
            "acceleration_scale": 0.25,
            "harvest_wait_sec": 4.25,
            "continuous_transition": True,
            "return_to_pick_ready": False,
            "retreat_after_harvest": True,
            "pick_ready_state_name": "PICK_READY_RIGHT",
            "prefer_robot_direction": True,
            "adaptive_grasp_max_rotation_deg": 37.5,
        },
    )

    values = {parameter.name: parameter.value for parameter in received}
    assert node.tomato_frame == "detected_tomato_7_tf"
    assert values["planning_pipeline_id"] == "chomp"
    assert values["preapproach_mode"] == "planner"
    assert values["execute"] is False
    assert values["publish_display_trajectory"] is False
    assert values["pick_ready_velocity_scale"] == pytest.approx(0.35)
    assert values["pick_ready_acceleration_scale"] == pytest.approx(0.25)
    assert values["harvest_wait_sec"] == pytest.approx(4.25)
    assert values["continuous_transition"] is True
    assert values["return_to_pick_ready"] is False
    assert values["retreat_after_harvest"] is True
    assert values["pick_ready_state_name"] == "PICK_READY_RIGHT"
    assert values["adaptive_grasp_prefer_robot_direction"] is True
    assert values["adaptive_grasp_max_rotation_deg"] == pytest.approx(37.5)
    assert len(values["pick_ready_joint_positions"]) == 6


def test_persistent_worker_enables_execution_only_when_requested():
    received = []
    joint_names = [
        "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"
    ]
    node = SimpleNamespace(
        tomato_frame="old_frame",
        get_parameter=lambda name: SimpleNamespace(value=joint_names),
        set_parameters=lambda parameters: (
            received.extend(parameters)
            or [SimpleNamespace(successful=True, reason="") for _ in parameters]
        ),
    )

    _apply_request(
        node,
        {
            "tomato_frame": "detected_tomato_2_tf",
            "execute": True,
        },
    )

    values = {parameter.name: parameter.value for parameter in received}
    assert values["execute"] is True
    assert values["publish_display_trajectory"] is False


def test_execute_returns_to_pick_ready_after_configured_wait(monkeypatch):
    events = []
    waits = []
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.harvest_planner.time.sleep",
        waits.append,
    )
    parameter_values = {
        "execute": True,
        "harvest_wait_sec": 2.75,
        "step_cycle_only": False,
    }
    planner = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(
            value=parameter_values[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: events.append(("log", message))
        ),
        _execute_trajectory=lambda trajectory, label: (
            events.append(("single", trajectory, label)) or True
        ),
        _execute_trajectory_sequence=lambda trajectories, label: (
            events.append(("sequence", trajectories, label)) or True
        ),
    )
    planner._execute_trajectory_group = lambda trajectories, label: (
        CartesianHarvestPlanner._execute_trajectory_group(
            planner,
            trajectories,
            label,
        )
    )
    plan = HarvestMotionPlan(
        pick_ready_trajectory="initial_ready",
        preapproach_trajectory=("preapproach",),
        approach_trajectory=("approach",),
        after_wait_trajectory=("post_wait",),
        return_pick_ready_trajectory="return_ready",
        display_start_state=RobotState(),
    )

    success = CartesianHarvestPlanner.execute(planner, plan)

    assert success is True
    assert waits == [pytest.approx(2.75)]
    assert [event for event in events if event[0] != "log"] == [
        ("single", "initial_ready", "PICK_READY"),
        ("sequence", ("preapproach",), "Pre-approach"),
        (
            "sequence",
            ("approach",),
            "Approach and pre-wait harvest",
        ),
        ("sequence", ("post_wait",), "Post-wait harvest"),
        ("single", "return_ready", "OMPL RETURN_PICK_READY"),
    ]
    assert events[-2] == (
        "single",
        "return_ready",
        "OMPL RETURN_PICK_READY",
    )
    assert "returned to PICK_READY" in events[-1][1]


def test_execution_safety_gate_blocks_unsafe_cached_trajectory():
    logged = []
    recorded = []
    violation = {
        "joint_name": "base",
        "reason": "SPAN_LIMIT_EXCEEDED",
        "span_deg": 385.0,
        "maximum_span_deg": 120.0,
        "max_step_deg": 5.0,
        "maximum_step_deg": 45.0,
    }
    planner = SimpleNamespace(
        _wait_for_current_joint_positions=lambda timeout_sec: {"base": 0.0},
        _trajectory_safety_violations=lambda trajectory, start_positions: [
            violation
        ],
        _log_trajectory_safety_failure=lambda label, violations: (
            logged.append((label, violations))
        ),
        _record_plan_stage=lambda *args, **kwargs: recorded.append(
            (args, kwargs)
        ),
    )

    success = CartesianHarvestPlanner._execute_trajectory(
        planner,
        "unsafe_cached_path",
        "접근 반복 역재생",
    )

    assert success is False
    assert logged == [
        ("접근 반복 역재생 실행 전 검사", [violation])
    ]
    assert recorded[0][0][0] == "EXECUTION_TRAJECTORY_SAFETY"
    assert recorded[0][0][4] == "UNSAFE_CACHED_TRAJECTORY"


def test_continuous_execute_skips_pick_ready_and_keeps_post_wait(monkeypatch):
    events = []
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.harvest_planner.time.sleep",
        lambda seconds: None,
    )
    planner = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(
            value={
                "execute": True,
                "harvest_wait_sec": 0.0,
                "step_cycle_only": False,
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: events.append(("log", message))
        ),
        _execute_trajectory=lambda trajectory, label: (
            events.append(("execute", trajectory, label)) or True
        ),
    )
    planner._execute_trajectory_sequence = lambda trajectories, label: (
        CartesianHarvestPlanner._execute_trajectory_sequence(
            planner,
            trajectories,
            label,
        )
    )
    plan = HarvestMotionPlan(
        pick_ready_trajectory=(),
        preapproach_trajectory=("direct_pregrasp",),
        approach_trajectory=("approach",),
        after_wait_trajectory=("post_wait",),
        return_pick_ready_trajectory=(),
        display_start_state=RobotState(),
    )

    assert CartesianHarvestPlanner.execute(planner, plan) is True
    executed = [event[1] for event in events if event[0] == "execute"]
    assert executed == ["direct_pregrasp", "approach", "post_wait"]
    assert "post-wait pose is retained" in events[-1][1]


def test_limited_stage_execute_skips_wait_and_post_wait(monkeypatch):
    events = []
    waits = []
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.harvest_planner.time.sleep",
        waits.append,
    )
    planner = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(
            value={
                "execute": True,
                "harvest_wait_sec": 2.0,
                "step_cycle_only": True,
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: events.append(("log", message))
        ),
        _execute_trajectory=lambda trajectory, label: (
            events.append(("execute", trajectory, label)) or True
        ),
    )
    planner._execute_trajectory_sequence = lambda trajectories, label: (
        CartesianHarvestPlanner._execute_trajectory_sequence(
            planner,
            trajectories,
            label,
        )
    )
    planner._execute_trajectory_group = lambda trajectories, label: (
        CartesianHarvestPlanner._execute_trajectory_group(
            planner,
            trajectories,
            label,
        )
    )
    plan = HarvestMotionPlan(
        pick_ready_trajectory=(),
        preapproach_trajectory=("arc_to_preapproach",),
        approach_trajectory=("through_stage_four",),
        after_wait_trajectory=("must_not_execute",),
        return_pick_ready_trajectory=(),
        display_start_state=RobotState(),
    )

    assert CartesianHarvestPlanner.execute(planner, plan) is True
    assert waits == []
    assert [event[1] for event in events if event[0] == "execute"] == [
        "arc_to_preapproach",
        "through_stage_four",
    ]
    assert "retained for the next continuous arc" in events[-1][1]


def test_arc_reverse_recovery_executes_before_next_preapproach():
    events = []
    planner = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(
            value={"execute": True, "step_cycle_only": True}[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: events.append(("log", message))
        ),
        _execute_trajectory=lambda trajectory, label: (
            events.append(("execute", trajectory, label)) or True
        ),
    )
    planner._execute_trajectory_sequence = lambda trajectories, label: (
        CartesianHarvestPlanner._execute_trajectory_sequence(
            planner,
            trajectories,
            label,
        )
    )
    planner._execute_trajectory_group = lambda trajectories, label: (
        CartesianHarvestPlanner._execute_trajectory_group(
            planner,
            trajectories,
            label,
        )
    )
    plan = HarvestMotionPlan(
        pick_ready_trajectory=(),
        preapproach_trajectory=("next_preapproach",),
        approach_trajectory=("next_target",),
        after_wait_trajectory=(),
        return_pick_ready_trajectory=(),
        display_start_state=RobotState(),
        arc_reverse_recovery_trajectory=("reverse_2", "reverse_1"),
    )

    assert CartesianHarvestPlanner.execute(planner, plan) is True
    assert [event[1] for event in events if event[0] == "execute"] == [
        "reverse_2",
        "reverse_1",
        "next_preapproach",
        "next_target",
    ]


def test_lift_continuous_execute_reaches_safe_retreat_before_next_tomato(
    monkeypatch,
):
    events = []
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.harvest_planner.time.sleep",
        lambda seconds: None,
    )
    planner = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(
            value={
                "execute": True,
                "harvest_wait_sec": 0.0,
                "step_cycle_only": False,
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: events.append(("log", message))
        ),
        _execute_trajectory=lambda trajectory, label: (
            events.append(("execute", trajectory, label)) or True
        ),
    )
    planner._execute_trajectory_sequence = lambda trajectories, label: (
        CartesianHarvestPlanner._execute_trajectory_sequence(
            planner,
            trajectories,
            label,
        )
    )
    plan = HarvestMotionPlan(
        pick_ready_trajectory=(),
        preapproach_trajectory=("direct_pregrasp",),
        approach_trajectory=("approach",),
        after_wait_trajectory=("post_wait",),
        return_pick_ready_trajectory=(),
        display_start_state=RobotState(),
        outward_retreat_trajectory=("safe_retreat",),
    )

    assert CartesianHarvestPlanner.execute(planner, plan) is True
    executed = [event[1] for event in events if event[0] == "execute"]
    assert executed == [
        "direct_pregrasp",
        "approach",
        "post_wait",
        "safe_retreat",
    ]
    assert "lift-safe outward retreat pose" in events[-1][1]


def test_continuous_preapproach_plans_outward_arc_trajectory():
    calls = []
    transform = TransformStamped()
    transform.transform.translation.z = 0.5
    transform.transform.rotation.w = 1.0
    planner = SimpleNamespace(
        last_plan_report={"stages": [], "cartesian_fallbacks": []},
        _trajectory_range_records=[],
        planning_link="tcp",
        get_parameter=lambda name: SimpleNamespace(
            value={
                "continuous_arc_min_clearance": 0.12,
                "continuous_arc_max_clearance": 0.25,
                "continuous_arc_waypoint_count": 7,
                "continuous_arc_max_joint_span_deg": 120.0,
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warning=lambda message: None,
        ),
        _lookup_transform=lambda frame: transform,
    )

    def plan_arc(waypoints, start_state, label, pregrasp, **kwargs):
        calls.append((waypoints, start_state, label, pregrasp))
        return ("arc_preapproach_trajectory",)

    planner._plan_cartesian_with_ompl_fallback = plan_arc
    target = Pose()
    target.position.x = 0.30
    target.position.z = 0.30
    target.orientation.w = 1.0

    result = CartesianHarvestPlanner._plan_continuous_preapproach(
        planner,
        target,
        outward_axis=[0.0, 1.0, 0.0],
    )

    assert result[0] == ()
    assert result[1] == ()
    assert result[2] == ("arc_preapproach_trajectory",)
    assert result[3].is_diff is True
    waypoints, start_state, label, pregrasp = calls[0]
    assert len(waypoints) == 7
    assert start_state.is_diff is True
    assert label == "Continuous arc pre-approach"
    assert pregrasp is True
    assert max(pose.position.y for pose in waypoints[:-1]) > 0.10
    assert waypoints[-1].position.x == pytest.approx(target.position.x)
    assert waypoints[-1].position.z == pytest.approx(target.position.z)
    assert planner.last_plan_report["continuous_transition_direct"] is True
    assert planner.last_plan_report["continuous_transition_arc"] is True


def test_preplanned_continuous_arc_uses_cached_end_state_and_pose():
    calls = []
    cached_state = RobotState()
    cached_state.is_diff = False
    cached_state.joint_state.name = ["base"]
    cached_state.joint_state.position = [0.4]
    cached_pose = Pose()
    cached_pose.position.x = -0.2
    cached_pose.position.y = 0.1
    cached_pose.position.z = 0.45
    cached_pose.orientation.w = 1.0
    planner = SimpleNamespace(
        last_plan_report={"stages": [], "cartesian_fallbacks": []},
        _trajectory_range_records=[],
        planning_link="tcp",
        get_parameter=lambda name: SimpleNamespace(
            value={
                "continuous_arc_min_clearance": 0.12,
                "continuous_arc_max_clearance": 0.25,
                "continuous_arc_waypoint_count": 7,
                "continuous_arc_max_joint_span_deg": 120.0,
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warning=lambda message: None,
        ),
        _lookup_transform=lambda frame: pytest.fail(
            "live TF must not be used for a cached arc start"
        ),
        _plan_cartesian_with_ompl_fallback=lambda waypoints, state, *args, **kwargs: (
            calls.append((waypoints, state)) or ("cached_arc",)
        ),
    )
    target = Pose()
    target.position.x = 0.3
    target.position.z = 0.3
    target.orientation.w = 1.0

    result = CartesianHarvestPlanner._plan_continuous_preapproach(
        planner,
        target,
        outward_axis=[0.0, 1.0, 0.0],
        start_state_override=cached_state,
        start_pose_override=cached_pose,
    )

    assert result[2] == ("cached_arc",)
    assert list(result[3].joint_state.position) == [0.4]
    assert list(calls[0][1].joint_state.position) == [0.4]
    expected_first_x = cached_pose.position.x + (
        target.position.x - cached_pose.position.x
    ) / 7.0
    assert calls[0][0][0].position.x == pytest.approx(expected_first_x)


def test_transition_only_arc_preserves_independently_planned_target():
    calls = []
    finished = []
    start_state = RobotState()
    start_state.joint_state.name = ["base"]
    start_state.joint_state.position = [0.2]
    start_pose = Pose()
    start_pose.position.x = -0.2
    start_pose.orientation.w = 1.0
    target_pose = Pose()
    target_pose.position.x = 0.3
    target_pose.orientation.w = 1.0
    planner = SimpleNamespace(
        last_plan_report={},
        _begin_plan_report=lambda state: planner.last_plan_report.update(
            {"stages": []}
        ),
        _synchronize_dynamic_base_transform=lambda: True,
        _finish_plan_report=finished.append,
        get_parameter=lambda name: SimpleNamespace(
            value={
                "continuous_arc_min_clearance": 0.12,
                "continuous_arc_max_clearance": 0.25,
                "continuous_arc_waypoint_count": 7,
                "continuous_arc_max_joint_span_deg": 120.0,
            }[name]
        ),
        _plan_cartesian_with_ompl_fallback=(
            lambda waypoints, state, *args, **kwargs: (
                calls.append((waypoints, state, args, kwargs))
                or ("arc_only",)
            )
        ),
        get_logger=lambda: SimpleNamespace(warning=lambda message: None),
    )

    transition = CartesianHarvestPlanner.plan_continuous_transition_only(
        planner,
        start_state,
        start_pose,
        target_pose,
        [0.0, 1.0, 0.0],
    )

    assert transition == ("arc_only",)
    assert finished == [True]
    assert len(calls[0][0]) == 7
    assert list(calls[0][1].joint_state.position) == [0.2]
    assert calls[0][3]["maximum_joint_span_deg"] == 120.0
    assert planner.last_plan_report["continuous_transition_arc"] is True


def test_arc_candidate_retries_only_stochastic_planning_failures():
    assert _candidate_failure_is_retryable(
        {
            "failure_stage": "OMPL_FALLBACK_PREAPPROACH_1",
            "failure_reason": "MOVEIT_PLANNING_FAILED",
        }
    ) is True
    assert _candidate_failure_is_retryable(
        {
            "failure_stage": "TARGET_GEOMETRY",
            "failure_reason": "DEADLINE_REQUIRES_ANGLE_OVER_MAXIMUM",
        }
    ) is False


def test_display_start_state_keeps_current_lift_height_for_rviz_playback():
    joint_names = [
        "base",
        "shoulder",
        "elbow",
        "wrist1",
        "wrist2",
        "wrist3",
    ]
    planner = SimpleNamespace(
        _latest_joint_positions={
            **{name: float(index) for index, name in enumerate(joint_names)},
            "farmily_lift_height_joint": 0.37,
        },
        get_parameter=lambda name: SimpleNamespace(
            value={
                "pick_ready_joint_names": joint_names,
                "display_passive_joint_names": [
                    "farmily_lift_height_joint"
                ],
            }[name]
        ),
    )
    start_state = RobotState()
    start_state.is_diff = True

    completed = CartesianHarvestPlanner._complete_display_start_state(
        planner,
        start_state,
    )

    values = dict(
        zip(completed.joint_state.name, completed.joint_state.position)
    )
    assert values["farmily_lift_height_joint"] == pytest.approx(0.37)
    assert all(name in values for name in joint_names)
    assert completed.is_diff is False
    assert start_state.joint_state.name == []


def test_display_start_state_preserves_moveit_arm_start_and_appends_lift():
    joint_names = [
        "base",
        "shoulder",
        "elbow",
        "wrist1",
        "wrist2",
        "wrist3",
    ]
    planner = SimpleNamespace(
        _latest_joint_positions={
            **{name: 9.0 for name in joint_names},
            "farmily_lift_height_joint": 0.12,
        },
        get_parameter=lambda name: SimpleNamespace(
            value={
                "pick_ready_joint_names": joint_names,
                "display_passive_joint_names": [
                    "farmily_lift_height_joint"
                ],
            }[name]
        ),
    )
    start_state = RobotState()
    start_state.joint_state.name = list(joint_names)
    start_state.joint_state.position = [0.1] * len(joint_names)

    completed = CartesianHarvestPlanner._complete_display_start_state(
        planner,
        start_state,
    )

    values = dict(
        zip(completed.joint_state.name, completed.joint_state.position)
    )
    assert values["base"] == pytest.approx(0.1)
    assert values["farmily_lift_height_joint"] == pytest.approx(0.12)


def test_dynamic_base_sync_uses_fresh_lift_joint_and_matching_tf(monkeypatch):
    transform = TransformStamped()
    transform.header.stamp.sec = 12
    transform.header.stamp.nanosec = 300
    transform.transform.translation.z = 0.42
    planner = SimpleNamespace(
        base_frame="link0",
        _latest_joint_positions={"farmily_lift_height_joint": 0.42},
        _joint_state_received_monotonic={
            "farmily_lift_height_joint": 10.0
        },
        _joint_state_stamp_nanoseconds={
            "farmily_lift_height_joint": 12_000_000_300
        },
        last_plan_report={},
        tf_buffer=SimpleNamespace(
            lookup_transform=lambda parent, child, stamp: transform
        ),
        get_parameter=lambda name: SimpleNamespace(
            value={
                "dynamic_base_joint_name": "farmily_lift_height_joint",
                "dynamic_base_parent_frame": "world",
                "dynamic_base_sync_timeout_sec": 1.0,
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            error=lambda message: None,
        ),
    )
    monkeypatch.setattr(harvest_planner_module.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(harvest_planner_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        harvest_planner_module.rclpy,
        "spin_once",
        lambda node, timeout_sec: None,
    )

    assert CartesianHarvestPlanner._synchronize_dynamic_base_transform(
        planner
    ) is True
    sync = planner.last_plan_report["dynamic_base_sync"]
    assert sync["joint_position_m"] == pytest.approx(0.42)
    assert sync["base_height_m"] == pytest.approx(0.42)
    assert sync["tf_stamp_nanoseconds"] >= sync["joint_stamp_nanoseconds"]


def test_continuous_arc_failure_reverses_cached_path_to_pick_ready():
    preapproach_state = RobotState()
    preapproach_state.joint_state.name = ["base", "shoulder"]
    preapproach_state.joint_state.position = [0.4, -0.8]
    display_start = RobotState()
    display_start.joint_state.name = ["base", "shoulder"]
    display_start.joint_state.position = [0.8, -1.0]
    transform = TransformStamped()
    transform.transform.translation.z = 0.5
    transform.transform.rotation.w = 1.0
    planner = SimpleNamespace(
        last_plan_report={"stages": [], "cartesian_fallbacks": []},
        _trajectory_range_records=[],
        planning_link="tcp",
        get_parameter=lambda name: SimpleNamespace(
            value={
                "continuous_arc_min_clearance": 0.12,
                "continuous_arc_max_clearance": 0.25,
                "continuous_arc_waypoint_count": 7,
                "continuous_arc_max_joint_span_deg": 120.0,
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warning=lambda message: None,
        ),
            _plan_preapproach=lambda pose, state, **kwargs: (
            planner.last_plan_report["stages"].append(
                {"stage": "CARTESIAN_PREAPPROACH", "success": True}
            )
            or ("seed_preapproach",)
        ),
        _record_trajectory_range_input=lambda *args, **kwargs: None,
        _lookup_transform=lambda frame: transform,
    )
    planner._trajectory_end_state = lambda trajectory: preapproach_state

    def fail_arc(*args, **kwargs):
        planner.last_plan_report["stages"].append(
            {
                "stage": "CARTESIAN_CONTINUOUS_ARC",
                "planner_type": "cartesian",
                "success": False,
                "reason": "CARTESIAN_FRACTION_LOW",
            }
        )
        planner.last_plan_report.update(
            {
                "failure_stage": "CARTESIAN_CONTINUOUS_ARC",
                "failure_planner_type": "cartesian",
                "failure_reason": "CARTESIAN_FRACTION_LOW",
            }
        )
        return None

    planner._plan_cartesian_with_ompl_fallback = fail_arc
    target = Pose()
    target.position.x = 0.30
    target.position.z = 0.30
    target.orientation.w = 1.0

    result = CartesianHarvestPlanner._plan_continuous_preapproach(
        planner,
        target,
        outward_axis=[0.0, 1.0, 0.0],
        start_state_override=display_start,
        arc_failure_reverse_trajectory=("reverse_previous_path",),
    )

    assert result == (
        ("reverse_previous_path",),
        (),
        ("seed_preapproach",),
        display_start,
    )
    assert planner.last_plan_report["continuous_transition_direct"] is False
    assert planner.last_plan_report["continuous_transition_arc"] is False
    assert planner.last_plan_report["recovery_success"] is True
    assert planner.last_plan_report["recovery_stage"] == (
        "CACHED_TRAJECTORY_REVERSE_TO_PICK_READY"
    )
    arc_stage = next(
        stage
        for stage in planner.last_plan_report["stages"]
        if stage["stage"] == "CARTESIAN_CONTINUOUS_ARC"
    )
    assert arc_stage["discarded"] is True


def test_continuous_arc_failure_without_history_does_not_plan_new_ready_path():
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    planner = SimpleNamespace(
        last_plan_report={"stages": [], "cartesian_fallbacks": []},
        _trajectory_range_records=[],
        planning_link="tcp",
        get_parameter=lambda name: SimpleNamespace(
            value={
                "continuous_arc_min_clearance": 0.12,
                "continuous_arc_max_clearance": 0.25,
                "continuous_arc_waypoint_count": 7,
                "continuous_arc_max_joint_span_deg": 120.0,
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warning=lambda message: None,
            error=lambda message: None,
        ),
        _lookup_transform=lambda frame: transform,
    )

    def fail_arc(*args, **kwargs):
        planner.last_plan_report["stages"].append(
            {
                "stage": "CARTESIAN_CONTINUOUS_ARC",
                "planner_type": "cartesian",
                "success": False,
                "reason": "CARTESIAN_FRACTION_LOW",
            }
        )
        return None

    planner._plan_cartesian_with_ompl_fallback = fail_arc
    target = Pose()
    target.orientation.w = 1.0

    result = CartesianHarvestPlanner._plan_continuous_preapproach(
        planner,
        target,
        outward_axis=[0.0, 1.0, 0.0],
    )

    assert result is None
    assert planner.last_plan_report["failure_reason"] == (
        "ARC_FAILED_WITHOUT_REVERSE_HISTORY"
    )
    assert planner.last_plan_report["recovery_used"] is False


def test_plan_report_keeps_first_failure_with_cartesian_details():
    planner = SimpleNamespace(last_plan_report={"stages": []})

    CartesianHarvestPlanner._record_plan_stage(
        planner,
        "CARTESIAN_APPROACH",
        "cartesian",
        False,
        0.35,
        "CARTESIAN_FRACTION_LOW",
        moveit_error_code=1,
        cartesian_fraction=0.84,
        required_fraction=0.98,
    )

    assert planner.last_plan_report["failure_stage"] == "CARTESIAN_APPROACH"
    assert planner.last_plan_report["failure_reason"] == "CARTESIAN_FRACTION_LOW"
    assert planner.last_plan_report["cartesian_fraction"] == pytest.approx(0.84)


def test_cartesian_fallback_summary_records_failure_reason_and_recovery():
    summary = cartesian_fallback_summary(
        {
            "cartesian_fallbacks": [
                {
                    "segment": "PREAPPROACH",
                    "cartesian_stage": "CARTESIAN_PREAPPROACH",
                    "cartesian_reason": "CARTESIAN_FRACTION_LOW",
                    "ompl_stages": ["OMPL_FALLBACK_PREAPPROACH_1"],
                    "success": True,
                }
            ]
        }
    )

    assert summary["failure_stage"] == "CARTESIAN_PREAPPROACH"
    assert summary["failure_reason"] == (
        "CARTESIAN_PREAPPROACH: CARTESIAN_FRACTION_LOW"
        " → CONSTRAINED_OMPL_SUCCESS"
    )
    assert summary["recovery_success"] is True
    assert summary["recovery_stage"] == "OMPL_FALLBACK_PREAPPROACH_1"


def test_sweep_stage_detail_shows_all_failure_and_recovery_lines():
    record = {
        "success": True,
        "failure_stage": "CARTESIAN_PREAPPROACH",
        "failure_reason": "CARTESIAN_FRACTION_LOW",
        "recovery_success": True,
        "recovery_stage": "OMPL_FALLBACK_PREAPPROACH_1",
        "recovery_reason": "Cartesian 실패 후 constrained OMPL 성공",
        "stages": [
            {
                "stage": "CARTESIAN_PREROTATE",
                "planner_type": "cartesian",
                "success": True,
                "duration_sec": 0.12,
                "cartesian_fraction": 1.0,
                "required_fraction": 0.98,
            },
            {
                "stage": "CARTESIAN_PREAPPROACH",
                "planner_type": "cartesian",
                "success": False,
                "duration_sec": 0.38,
                "cartesian_fraction": 0.45,
                "required_fraction": 0.98,
                "reason": "CARTESIAN_FRACTION_LOW",
            },
            {
                "stage": "OMPL_FALLBACK_PREAPPROACH_1",
                "planner_type": "ompl",
                "success": True,
                "duration_sec": 1.37,
                "planning_time_sec": 1.25,
            },
        ],
    }

    summary = sweep_stage_summary(record)
    detail = sweep_stage_detail(record)

    assert summary == (
        "CARTESIAN_PREAPPROACH → OMPL_FALLBACK_PREAPPROACH_1"
    )
    assert detail.count("\n") == 4
    assert "1. [성공] CARTESIAN_PREROTATE" in detail
    assert "2. [실패] CARTESIAN_PREAPPROACH" in detail
    assert "소요=0.38s" in detail
    assert "fraction=45.0%" in detail
    assert "사유=CARTESIAN_FRACTION_LOW" in detail
    assert "3. [성공] OMPL_FALLBACK_PREAPPROACH_1" in detail
    assert "소요=1.37s" in detail
    assert "복구 결과: 성공" in detail
    assert detail.endswith("최종 결과: 성공")


def test_sweep_stage_detail_shows_final_failure_reason():
    record = {
        "success": False,
        "failure_stage": "OMPL_FALLBACK_PREAPPROACH_1",
        "failure_reason": "MOVEIT_PLANNING_FAILED",
        "stages": [],
    }

    assert sweep_stage_summary(record) == "OMPL_FALLBACK_PREAPPROACH_1"
    assert sweep_stage_detail(record) == (
        "최종 결과: 실패 | OMPL_FALLBACK_PREAPPROACH_1"
        " | MOVEIT_PLANNING_FAILED"
    )


def test_replay_selected_sweep_scene_restores_recorded_environment():
    values = {}
    logs = []

    def variable(name):
        return SimpleNamespace(set=lambda value: values.__setitem__(name, value))

    gui = SimpleNamespace(
        _selected_sweep_record=lambda: {
            "case": 12,
            "tomato": 3,
            "x": 0.355,
            "y": -0.275,
            "z": 0.34,
            "rotation_deg": 45.0,
        },
        harvest_process=None,
        batch_active=False,
        sweep_active=False,
        sweep_worker_process=None,
        scene_x=variable("x"),
        scene_y=variable("y"),
        scene_z=variable("z"),
        scene_rotation=variable("rotation"),
        preserve_sweep_markers_on_scene_set=False,
        _append_log=logs.append,
        set_scene_position=lambda: values.__setitem__("applied", True),
    )

    HarvestGui.replay_selected_sweep_scene(gui)

    assert values == {
        "x": "0.3550",
        "y": "-0.2750",
        "z": "0.3400",
        "rotation": "45.0",
        "applied": True,
    }
    assert gui.preserve_sweep_markers_on_scene_set is True
    assert "case=12" in logs[0]


def test_harvest_command_builds_plan_only_command():
    command = harvest_command(
        3,
        False,
        planning_pipeline_id="chomp",
        preapproach_mode="planner",
        velocity_scale=0.35,
        acceleration_scale=0.25,
        harvest_wait_sec=3.5,
        harvest_x_forward_m=0.055,
        continuous_transition=True,
        return_to_pick_ready=False,
        retreat_after_harvest=True,
        pick_ready_state_name="PICK_READY_RIGHT",
        prefer_robot_direction=True,
        adaptive_grasp_max_rotation_deg=35.0,
        python_executable="/usr/bin/python3",
    )

    assert command[0:3] == [
        "/usr/bin/python3",
        "-m",
        "rbpodo_tomato_harvest.tomato_harvest_test",
    ]
    assert "tomato_frame:=detected_tomato_3_tf" in command
    assert "execute:=false" in command
    assert "planning_pipeline_id:=chomp" in command
    assert "planner_id:=RRTConnect" in command
    assert "preapproach_mode:=planner" in command
    assert "publish_display_trajectory:=true" in command
    assert "pick_ready_velocity_scale:=0.35" in command
    assert "pick_ready_acceleration_scale:=0.25" in command
    assert "harvest_wait_sec:=3.5" in command
    assert "harvest_x_forward:=0.055" in command
    assert "continuous_transition:=true" in command
    assert "return_to_pick_ready:=false" in command
    assert "retreat_after_harvest:=true" in command
    assert "pick_ready_state_name:=PICK_READY_RIGHT" in command
    assert "adaptive_grasp_prefer_robot_direction:=true" in command
    assert "adaptive_grasp_max_rotation_deg:=35.0" in command


def test_preplanned_batch_command_selects_worker_and_arc_mode():
    command, environment = preplanned_batch_command(
        5,
        continuous_arc=True,
        harvest_x_forward_m=0.035,
        pick_ready_state_name="PICK_READY_RIGHT",
        python_executable="/usr/bin/python3",
    )

    assert command[:3] == [
        "/usr/bin/python3",
        "-m",
        "rbpodo_tomato_harvest.tomato_harvest_preplanned_batch",
    ]
    assert "harvest_x_forward:=0.035" in command
    config = json.loads(environment[PREPLANNED_BATCH_CONFIG_ENV])
    assert config == {
        "tomato_count": 5,
        "tomato_frames": [
            "detected_tomato_0_tf",
            "detected_tomato_1_tf",
            "detected_tomato_2_tf",
            "detected_tomato_3_tf",
            "detected_tomato_4_tf",
        ],
        "continuous_arc": True,
        "execute": True,
        "start_tolerance_deg": 3.0,
        "harvest_stage_limit": None,
        "candidate_attempts": 3,
    }


def test_commands_forward_camera_target_tf_names():
    command = harvest_command(0, False, tomato_frame="C0:T7")
    assert "tomato_frame:=C0:T7" in command

    step_command = stepper_command(0, tomato_frame="C0:T7")
    assert "tomato_frame:=C0:T7" in step_command

    batch_command, environment = preplanned_batch_command(
        2,
        continuous_arc=True,
        tomato_frames=("C0:T7", "C0:T8"),
    )
    assert "tomato_frame:=C0:T7" in batch_command
    config = json.loads(environment[PREPLANNED_BATCH_CONFIG_ENV])
    assert config["tomato_frames"] == ["C0:T7", "C0:T8"]


def test_preplanned_batch_command_limits_each_tomato_stage_not_count():
    command, environment = preplanned_batch_command(
        5,
        continuous_arc=True,
        harvest_stage_limit=4,
    )

    config = json.loads(environment[PREPLANNED_BATCH_CONFIG_ENV])
    assert config["tomato_count"] == 5
    assert config["harvest_stage_limit"] == 4
    assert "step_cycle_last_stage:=4" in command


def test_preplanned_batch_command_can_plan_complete_sequence_without_execution():
    command, environment = preplanned_batch_command(
        5,
        continuous_arc=True,
        execute=False,
    )

    config = json.loads(environment[PREPLANNED_BATCH_CONFIG_ENV])
    assert config["execute"] is False
    assert "execute:=false" in command


def test_harvest_command_rejects_invalid_motion_scale():
    with pytest.raises(ValueError, match="velocity_scale"):
        harvest_command(0, False, velocity_scale=1.01)
    with pytest.raises(ValueError, match="acceleration_scale"):
        harvest_command(0, False, acceleration_scale=0.0)
    with pytest.raises(ValueError, match="harvest_wait_sec"):
        harvest_command(0, False, harvest_wait_sec=-0.1)
    with pytest.raises(ValueError, match="harvest_x_forward_m"):
        harvest_command(0, False, harvest_x_forward_m=0.009)
    with pytest.raises(ValueError, match="harvest_x_forward_m"):
        harvest_command(0, False, harvest_x_forward_m=0.071)
    with pytest.raises(ValueError, match="ready state"):
        harvest_command(
            0,
            False,
            pick_ready_state_name="UNKNOWN_READY",
        )
    with pytest.raises(ValueError, match="harvest_stage_limit"):
        harvest_command(0, False, harvest_stage_limit=5)


def test_harvest_command_can_stop_each_tomato_at_stage_four():
    command = harvest_command(
        7,
        True,
        harvest_stage_limit=4,
        continuous_transition=True,
        return_to_pick_ready=False,
    )

    assert "tomato_frame:=detected_tomato_7_tf" in command
    assert "stepwise_plan:=true" in command
    assert "step_cycle_only:=true" in command
    assert "step_cycle_last_stage:=4" in command


def test_gui_percent_to_scale_validates_operator_input():
    assert HarvestGui._percent_to_scale("30", "속도") == pytest.approx(0.3)
    with pytest.raises(ValueError, match="1~100%"):
        HarvestGui._percent_to_scale("0", "속도")
    with pytest.raises(ValueError, match="숫자"):
        HarvestGui._percent_to_scale("fast", "속도")


def test_gui_wait_seconds_validates_operator_input():
    assert HarvestGui._wait_seconds("2.5") == pytest.approx(2.5)
    assert HarvestGui._wait_seconds("0") == pytest.approx(0.0)
    with pytest.raises(ValueError, match="0초 이상"):
        HarvestGui._wait_seconds("-1")
    with pytest.raises(ValueError, match="숫자"):
        HarvestGui._wait_seconds("wait")


def test_cancel_all_goals_request_uses_zero_id_and_timestamp():
    request = cancel_all_goals_request()

    assert list(request.goal_info.goal_id.uuid) == [0] * 16
    assert request.goal_info.stamp.sec == 0
    assert request.goal_info.stamp.nanosec == 0


def test_gripper_output_startup_keeps_dout0_and_dout8_high():
    assert gripper_output_startup_scripts() == (
        "set_dout_bit_combination(0,15,257,0)",
    )
    assert gripper_relay_power_script() == (
        "set_dout_bit_combination(0,15,257,0)"
    )
    assert gripper_relay_power_off_script() == (
        "set_dout_bit_combination(0,15,0,0)"
    )


@pytest.mark.parametrize(
    ("command", "expected_sequence"),
    [
        (
            "extend",
            (
                (False, False),
                (True, False),
            ),
        ),
        (
            "retract",
            (
                (False, False),
                (False, True),
            ),
        ),
        ("stop", ((False, False),)),
    ],
)
def test_linear_motor_pin_commands_use_break_before_make_sequence(
    command,
    expected_sequence,
):
    assert linear_motor_pin_sequence(command) == expected_sequence


def test_linear_motor_pin_values_reject_unknown_command():
    with pytest.raises(ValueError, match="지원하지 않는"):
        linear_motor_pin_values("invalid")


def test_gripper_extend_target_schedules_three_second_auto_stop():
    scheduled = []
    published = []
    finished = []
    auto_stops = []
    gui = SimpleNamespace(
        gripper_stroke_request_id=7,
        linear_motor_serial_connected=True,
        linear_motor_pin_state_initialized=True,
        _publish_linear_motor_pin_levels=lambda pin8, pin9: published.append(
            (pin8, pin9)
        ),
        _append_log=lambda message: None,
        _finish_linear_motor_pin_command=lambda *args: finished.append(args),
        _auto_stop_gripper_extension=lambda *args: auto_stops.append(args),
        root=SimpleNamespace(
            after=lambda milliseconds, callback: scheduled.append(
                (milliseconds, callback)
            )
        ),
    )

    HarvestGui._apply_linear_motor_pin_target(
        gui,
        ((False, False), (True, False)),
        command="extend",
        request_id=7,
        label="늘림",
        output_description="PIN8=HIGH, PIN9=LOW",
    )

    assert published == [(True, False)]
    assert finished == [(7, "늘림", "PIN8=HIGH, PIN9=LOW")]
    assert scheduled[0][0] == 3000
    scheduled[0][1]()
    assert auto_stops == [(7, GRIPPER_EXTEND_AUTO_STOP_SECONDS)]


def test_gripper_extend_auto_stop_forces_both_pins_low():
    published = []
    status_messages = []
    logs = []
    gui = SimpleNamespace(
        closing=False,
        gripper_stroke_request_id=4,
        gripper_command_in_progress=False,
        _publish_linear_motor_pin_levels=lambda pin8, pin9: published.append(
            (pin8, pin9)
        ),
        gripper_stroke_status=SimpleNamespace(
            set=lambda message: status_messages.append(message)
        ),
        status=SimpleNamespace(set=lambda message: status_messages.append(message)),
        _append_log=lambda message: logs.append(message),
        _update_gripper_stroke_controls=lambda: None,
    )

    HarvestGui._auto_stop_gripper_extension(gui, 4, 3.0)

    assert gui.gripper_stroke_request_id == 5
    assert published == [(False, False)]
    assert any("자동 정지" in message for message in status_messages)
    assert any("PIN8=LOW, PIN9=LOW" in message for message in logs)


@pytest.mark.parametrize(
    ("value", "expected"),
    (("10", 10), ("90", 90), ("173", 173), ("89.6", 90)),
)
def test_servo_angle_degrees_normalizes_valid_values(value, expected):
    assert servo_angle_degrees(value) == expected


@pytest.mark.parametrize("value", ("", "nan", "9", "174", "invalid"))
def test_servo_angle_degrees_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        servo_angle_degrees(value)


@pytest.mark.parametrize(
    ("value", "expected"),
    (("1", 1), ("90", 90), ("100", 100), ("89.6", 90)),
)
def test_servo_speed_percent_normalizes_valid_values(value, expected):
    assert servo_speed_percent(value) == expected


@pytest.mark.parametrize("value", ("", "nan", "0", "101", "invalid"))
def test_servo_speed_percent_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        servo_speed_percent(value)


def test_servo_speed_rate_uses_provisional_max_and_100_is_immediate():
    assert servo_speed_degrees_per_second(90) == pytest.approx(162.0)
    assert servo_speed_degrees_per_second(100) is None


@pytest.mark.parametrize("preset_angle", (90, 170))
def test_servo_preset_button_publishes_requested_angle(preset_angle):
    published = []
    displayed = []
    gui = SimpleNamespace(
        servo10_angle_deg_var=SimpleNamespace(
            get=lambda: "10",
            set=lambda value: displayed.append(value),
        ),
        servo10_speed_enabled_var=SimpleNamespace(get=lambda: True),
        servo10_speed_percent_var=SimpleNamespace(get=lambda: "90"),
        linear_motor_node_online=True,
        linear_motor_serial_connected=True,
        linear_motor_servo10_command_topic="/linear_motor/servo10_command",
        count_subscribers=lambda _topic: 1,
        linear_motor_servo10_command_publisher=SimpleNamespace(
            publish=lambda message: published.append(message.data)
        ),
        status=SimpleNamespace(set=lambda _value: None),
        _append_log=lambda _value: None,
    )

    HarvestGui.send_servo10_angle(gui, preset_angle)

    assert [list(values) for values in published] == [
        [float(preset_angle), 90.0]
    ]
    assert displayed == [str(preset_angle)]


def test_servo_speed_unchecked_uses_default_fifty_percent():
    published = []
    gui = SimpleNamespace(
        servo10_angle_deg_var=SimpleNamespace(
            get=lambda: "170",
            set=lambda _value: None,
        ),
        servo10_speed_enabled_var=SimpleNamespace(get=lambda: False),
        # An old/custom entry value must not leak into unchecked operation.
        servo10_speed_percent_var=SimpleNamespace(get=lambda: "90"),
        linear_motor_node_online=True,
        linear_motor_serial_connected=True,
        linear_motor_servo10_command_topic="/linear_motor/servo10_command",
        count_subscribers=lambda _topic: 1,
        linear_motor_servo10_command_publisher=SimpleNamespace(
            publish=lambda message: published.append(list(message.data))
        ),
        status=SimpleNamespace(set=lambda _value: None),
        _append_log=lambda _value: None,
    )

    HarvestGui.send_servo10_angle(gui)

    assert published == [[170.0, 50.0]]


def test_stepper_command_enables_detailed_cached_plan():
    command = stepper_command(
        3,
        harvest_wait_sec=1.5,
        pick_ready_state_name="PICK_READY_RIGHT",
        python_executable="/usr/bin/python3",
    )

    assert command[:3] == [
        "/usr/bin/python3",
        "-m",
        "rbpodo_tomato_harvest.tomato_harvest_stepper",
    ]
    assert "tomato_frame:=detected_tomato_3_tf" in command
    assert "execute:=true" in command
    assert "pick_ready_state_name:=PICK_READY_RIGHT" in command
    assert "harvest_wait_sec:=1.5" in command
    assert "stepwise_plan:=true" in command
    assert "step_cycle_last_stage:=5" in command
    assert "harvest_x_forward:=0.04" in command
    assert "step_servo_speed_percent:=50.0" in command
    assert "step_servo_close_angle_deg:=100.0" in command
    assert "harvest_forward_wave_enabled:=false" in command
    assert command[-2:] == ["-p", "step_cycle_only:=false"]


def test_stepper_command_forwards_servo_speed_percent():
    command = stepper_command(3, servo_speed_percent=75)

    assert "step_servo_speed_percent:=75.0" in command


def test_stepper_command_forwards_servo_close_angle():
    command = stepper_command(3, servo_close_angle_deg=112)

    assert "step_servo_close_angle_deg:=112.0" in command


def test_stepper_command_can_enable_stage_four_forward_wave():
    command = stepper_command(3, forward_wave_enabled=True)

    assert "harvest_forward_wave_enabled:=true" in command


def test_stepper_command_can_limit_repeat_plan_to_selected_stage():
    command = stepper_command(
        1,
        cycle_only=True,
        cycle_last_stage=4,
        cycle_forward_distance_m=0.035,
    )

    assert "stepwise_plan:=true" in command
    assert "step_cycle_last_stage:=4" in command
    assert "harvest_x_forward:=0.035" in command
    assert "return_to_pick_ready:=false" in command
    assert command[-2:] == ["-p", "step_cycle_only:=true"]


def test_stepper_command_forwards_custom_stage_xyz_parameters():
    custom_deltas = (
        (0.011, 0.002, -0.003),
        (0.041, 0.004, 0.005),
        (0.021, -0.006, 0.022),
        (0.007, 0.008, 0.023),
        (-0.051, 0.009, -0.010),
    )

    command = stepper_command(1, custom_stage_deltas_m=custom_deltas)

    assert "step_custom_stage_deltas_enabled:=true" in command
    assert "step_stage_3_x_delta:=0.011" in command
    assert "step_stage_4_y_delta:=0.004" in command
    assert "step_stage_6_z_delta:=0.022" in command
    assert "step_stage_7_x_delta:=0.007" in command
    assert "step_stage_8_z_delta:=-0.01" in command
    assert "harvest_tcp_wrist_rotation_deg:=10.0" in command


def test_stepper_command_can_disable_tcp_wrist_oscillation():
    command = stepper_command(
        1,
        tcp_wrist_oscillation_enabled=False,
    )

    assert "harvest_tcp_wrist_oscillation_enabled:=false" in command


def test_stepper_command_rejects_invalid_repeat_last_stage():
    with pytest.raises(ValueError, match="between 1 and 7"):
        stepper_command(1, cycle_only=True, cycle_last_stage=8)
    with pytest.raises(ValueError, match="between 0.010 and 0.070"):
        stepper_command(1, cycle_forward_distance_m=0.071)


@pytest.mark.parametrize(
    ("millimeters", "meters"),
    [("10", 0.010), ("35.5", 0.0355), ("70", 0.070)],
)
def test_repeat_forward_distance_converts_operator_mm_to_m(
    millimeters,
    meters,
):
    assert repeat_forward_distance_m(millimeters) == pytest.approx(meters)


@pytest.mark.parametrize("value", ["0", "45", "90"])
def test_adaptive_grasp_max_rotation_accepts_gui_range(value):
    assert adaptive_grasp_max_rotation_degrees(value) == pytest.approx(
        float(value)
    )


@pytest.mark.parametrize("value", ["-0.1", "90.1", "invalid"])
def test_adaptive_grasp_max_rotation_rejects_invalid_input(value):
    with pytest.raises(ValueError, match="최대 보정각"):
        adaptive_grasp_max_rotation_degrees(value)


@pytest.mark.parametrize("value", ["9.9", "70.1", "invalid", "nan"])
def test_repeat_forward_distance_rejects_values_outside_gui_range(value):
    with pytest.raises(ValueError, match="10~70 mm|숫자"):
        repeat_forward_distance_m(value)


def test_step_custom_stage_deltas_converts_xyz_millimetres_to_metres():
    values = {
        3: {"x": "10", "y": "1", "z": "-2"},
        4: {"x": "40", "y": "3", "z": "4"},
        6: {"x": "20", "y": "-5", "z": "20"},
        7: {"x": "0", "y": "6", "z": "20"},
        8: {"x": "-50", "y": "7", "z": "0"},
    }

    actual = step_custom_stage_deltas_m(values)
    expected = (
        (0.010, 0.001, -0.002),
        (0.040, 0.003, 0.004),
        (0.020, -0.005, 0.020),
        (0.0, 0.006, 0.020),
        (-0.050, 0.007, 0.0),
    )
    for actual_stage, expected_stage in zip(actual, expected):
        assert actual_stage == pytest.approx(expected_stage)


@pytest.mark.parametrize("value", ["invalid", "nan", "201", "-201"])
def test_step_custom_stage_deltas_rejects_invalid_or_oversized_values(value):
    values = {
        stage: {"x": "0", "y": "0", "z": "0"}
        for stage in (3, 4, 6, 7, 8)
    }
    values[6]["y"] = value

    with pytest.raises(ValueError, match="6단계"):
        step_custom_stage_deltas_m(values)


@pytest.mark.parametrize(
    ("direction", "stage_number", "action", "last_index"),
    [
        ("forward", 1, "execute_cycle_forward", 0),
        ("forward", 4, "execute_cycle_forward", 3),
        ("reverse", 6, "execute_cycle_reverse", 5),
    ],
)
def test_repeat_cycle_command_converts_gui_stage_to_zero_based_index(
    direction,
    stage_number,
    action,
    last_index,
):
    assert repeat_cycle_command(direction, stage_number) == {
        "command": action,
        "last_stage_index": last_index,
    }


def test_repeat_cycle_command_rejects_invalid_stage_or_direction():
    with pytest.raises(ValueError, match="between 1 and 7"):
        repeat_cycle_command("forward", 8)
    with pytest.raises(ValueError, match="direction"):
        repeat_cycle_command("sideways", 3)


@pytest.mark.parametrize(
    ("direction", "next_index", "stage_number", "expected"),
    [
        ("forward", 0, 5, {"command": "execute_next"}),
        ("forward", 4, 5, {"command": "execute_next"}),
        ("forward", 5, 5, None),
        ("reverse", 5, 5, {"command": "execute_previous"}),
        ("reverse", 1, 5, {"command": "execute_previous"}),
        ("reverse", 0, 5, None),
    ],
)
def test_repeat_stage_command_advances_one_pause_safe_stage(
    direction,
    next_index,
    stage_number,
    expected,
):
    assert repeat_stage_command(direction, next_index, stage_number) == expected


def test_repeat_stage_command_rejects_invalid_progress_state():
    with pytest.raises(ValueError, match="outside the cycle"):
        repeat_stage_command("forward", 6, 5)
    with pytest.raises(ValueError, match="unsupported repeat direction"):
        repeat_stage_command("sideways", 0, 5)


def test_repeat_automation_sends_only_one_stage_before_next_event():
    commands = []
    gui = SimpleNamespace(
        repeat_run_direction="forward",
        step_execution_in_progress=False,
        repeat_pause_requested=False,
        repeat_cycle_last_index=4,
        step_next_index=2,
        repeat_paused=True,
        _send_step_command=lambda command: commands.append(command) or True,
        _complete_repeat_forward_cycle=lambda: pytest.fail(
            "forward cycle completed too early"
        ),
        _complete_repeat_reverse_cycle=lambda: pytest.fail(
            "reverse cycle completed unexpectedly"
        ),
    )

    HarvestGui._continue_repeat_automation(gui)

    assert commands == [{"command": "execute_next"}]
    assert not gui.repeat_paused


def test_repeat_execution_accepts_verification_with_adaptive_grasp_options(
    monkeypatch,
):
    writes = []
    verification = (
        7,
        2,
        *GUI_PLANNER_CONFIG,
        "PICK_READY_RIGHT",
        True,
        35.0,
    )
    gui = SimpleNamespace(
        step_process=SimpleNamespace(
            poll=lambda: None,
            stdin=SimpleNamespace(
                write=writes.append,
                flush=lambda: None,
            ),
        ),
        step_session_mode="repeat",
        repeat_execution_enabled_var=SimpleNamespace(get=lambda: True),
        step_execution_enabled_var=SimpleNamespace(get=lambda: True),
        step_session_verification=verification,
        step_execution_confirmed=True,
        _verification_matches_current_selection=(
            lambda current: current == verification
        ),
        _append_log=lambda _message: None,
        _update_step_controls=lambda: None,
    )
    monkeypatch.setattr(
        harvest_gui_module.messagebox,
        "showerror",
        lambda *_args, **_kwargs: pytest.fail(
            "unchanged adaptive grasp verification must remain valid"
        ),
    )

    assert HarvestGui._send_step_command(
        gui,
        {"command": "execute_next"},
    ) is True
    assert writes == [json.dumps({"command": "execute_next"}) + "\n"]


def test_repeat_automation_holds_at_stage_boundary_when_pause_requested():
    messages = []
    gui = SimpleNamespace(
        repeat_run_direction="reverse",
        step_execution_in_progress=False,
        repeat_pause_requested=True,
        repeat_cycle_last_index=4,
        step_next_index=3,
        repeat_paused=False,
        repeat_status=SimpleNamespace(
            set=messages.append,
            get=lambda: messages[-1],
        ),
        status=SimpleNamespace(set=messages.append),
        _update_step_controls=lambda: None,
        _send_step_command=lambda _command: pytest.fail(
            "paused automation must not send a trajectory"
        ),
    )

    HarvestGui._continue_repeat_automation(gui)

    assert gui.repeat_paused
    assert any("일시 정지" in message for message in messages)


def test_repeat_automation_finishes_reverse_only_after_reaching_stage_zero():
    completed = []
    gui = SimpleNamespace(
        repeat_run_direction="reverse",
        step_execution_in_progress=False,
        repeat_pause_requested=False,
        repeat_cycle_last_index=4,
        step_next_index=0,
        repeat_paused=False,
        _send_step_command=lambda _command: pytest.fail(
            "completed reverse cycle must not send another trajectory"
        ),
        _complete_repeat_forward_cycle=lambda: pytest.fail(
            "wrong completion direction"
        ),
        _complete_repeat_reverse_cycle=lambda: completed.append(True),
    )

    HarvestGui._continue_repeat_automation(gui)

    assert completed == [True]


def test_camera_service_for_source_maps_fake_and_real_services():
    assert camera_service_for_source(
        CAMERA_SOURCE_FAKE,
        "/fake_tomato_camera/detect_tomatoes",
        "/detect_tomatoes",
    ) == "/fake_tomato_camera/detect_tomatoes"
    assert camera_service_for_source(
        CAMERA_SOURCE_REAL,
        "/fake_tomato_camera/detect_tomatoes",
        "/detect_tomatoes",
    ) == "/detect_tomatoes"


def test_camera_service_for_source_rejects_unknown_source():
    with pytest.raises(ValueError, match="지원하지 않는 카메라"):
        camera_service_for_source("unknown", "/fake", "/real")


def test_angle_reference_parameter_request_sends_base_to_center_mode():
    gui = SimpleNamespace(
        _selected_angle_reference_mode=lambda: "base_to_center"
    )

    request = HarvestGui._angle_reference_parameter_request(gui)

    assert len(request.parameters) == 1
    assert request.parameters[0].name == "angle_reference_mode"
    assert request.parameters[0].value.type == ParameterType.PARAMETER_STRING
    assert request.parameters[0].value.string_value == "base_to_center"


def test_detected_tomato_marker_array_shows_center_stem_and_approach():
    detections = TomatoDetectionArray()
    detections.header.frame_id = "d435_color_optical_frame"
    detections.detections = [
        TomatoDetection(
            id="tomato_a",
            center=Point(x=0.12, y=-0.03, z=0.48),
            stem_point=Point(x=0.14, y=-0.03, z=0.48),
        ),
        TomatoDetection(
            id="tomato_b",
            center=Point(x=-0.08, y=0.06, z=0.72),
            stem_point=Point(x=-0.08, y=0.09, z=0.72),
        ),
    ]

    message = detected_tomato_marker_array(
        detections,
        diameter=0.04,
        stem_diameter=0.01,
        approach_length=0.06,
    )

    assert len(message.markers) == 7
    assert message.markers[0].action == message.markers[0].DELETEALL
    tomato = message.markers[1]
    assert tomato.header.frame_id == "d435_color_optical_frame"
    assert tomato.type == tomato.SPHERE
    assert tomato.pose.position == Point(x=0.12, y=-0.03, z=0.48)
    assert tomato.scale.x == pytest.approx(0.04)
    assert tomato.scale.y == pytest.approx(0.04)
    assert tomato.scale.z == pytest.approx(0.04)
    assert tomato.color.r == pytest.approx(1.0)
    assert tomato.color.g == pytest.approx(0.15)
    assert tomato.color.b == pytest.approx(0.55)
    assert tomato.color.a == pytest.approx(0.85)

    stem = message.markers[2]
    assert stem.type == stem.SPHERE
    assert stem.pose.position == Point(x=0.14, y=-0.03, z=0.48)
    assert stem.scale.x == pytest.approx(0.01)
    assert stem.color.r == pytest.approx(0.2)
    assert stem.color.g == pytest.approx(1.0)
    assert stem.color.b == pytest.approx(0.2)

    approach = message.markers[3]
    assert approach.type == approach.ARROW
    assert approach.points[0] == Point(x=0.06, y=-0.03, z=0.48)
    assert approach.points[1] == Point(x=0.12, y=-0.03, z=0.48)
    assert approach.color.r == pytest.approx(0.1)
    assert approach.color.g == pytest.approx(0.8)
    assert approach.color.b == pytest.approx(1.0)


def test_detection_marker_can_use_calyx_to_stem_for_approach_angle():
    detections = TomatoDetectionArray()
    detections.header.frame_id = "camera"
    detections.detections = [
        TomatoDetection(
            id="tomato_a",
            center=Point(x=0.0, y=0.0, z=0.5),
            calyx_point=Point(x=0.01, y=0.0, z=0.5),
            stem_point=Point(x=0.01, y=0.02, z=0.5),
        )
    ]

    center_markers = detected_tomato_marker_array(
        detections,
        approach_length=0.06,
    ).markers
    calyx_markers = detected_tomato_marker_array(
        detections,
        approach_length=0.06,
        angle_reference_mode="calyx_to_stem",
    ).markers
    base_markers = detected_tomato_marker_array(
        detections,
        approach_length=0.06,
        angle_reference_mode="base_to_center",
        base_point=Point(x=-0.10, y=0.0, z=0.5),
    ).markers

    assert center_markers[3].points[0].x < 0.0
    assert center_markers[3].points[0].y < 0.0
    assert calyx_markers[3].points[0] == Point(x=0.0, y=-0.06, z=0.5)
    assert calyx_markers[3].points[1] == Point(x=0.0, y=0.0, z=0.5)
    assert base_markers[3].points[0] == Point(x=-0.06, y=0.0, z=0.5)
    assert base_markers[3].points[1] == Point(x=0.0, y=0.0, z=0.5)


def test_detection_message_is_sorted_by_transformed_world_height():
    detections = TomatoDetectionArray()
    detections.header.frame_id = "camera"
    detections.detections = [
        TomatoDetection(
            id="low",
            center=Point(x=0.0, y=0.40, z=0.0),
            stem_point=Point(x=0.01, y=0.40, z=0.0),
        ),
        TomatoDetection(
            id="high",
            center=Point(x=0.0, y=0.90, z=0.0),
            stem_point=Point(x=0.01, y=0.90, z=0.0),
        ),
    ]
    transform = TransformStamped()
    # Camera +Y becomes world +Z (90 degrees around +X).
    transform.transform.rotation.x = math.sqrt(0.5)
    transform.transform.rotation.w = math.sqrt(0.5)

    sorted_message = detection_message_sorted_by_height(
        detections,
        transform,
    )

    assert [item.id for item in sorted_message.detections] == ["high", "low"]


def test_detection_message_prioritizes_cluster_summed_height():
    detections = TomatoDetectionArray()
    detections.header.frame_id = "world"
    specifications = (
        ("capture/cluster_1/tomato_0", 0.90),
        ("capture/cluster_1/tomato_1", 0.10),
        ("capture/cluster_2/tomato_0", 0.70),
        ("capture/cluster_2/tomato_1", 0.60),
    )
    detections.detections = [
        TomatoDetection(
            id=identifier,
            center=Point(x=0.0, y=0.0, z=z),
            stem_point=Point(x=0.01, y=0.0, z=z),
        )
        for identifier, z in specifications
    ]

    sorted_message = detection_message_sorted_by_height(detections)

    assert [item.id for item in sorted_message.detections] == [
        "capture/cluster_2/tomato_0",
        "capture/cluster_2/tomato_1",
        "capture/cluster_1/tomato_0",
        "capture/cluster_1/tomato_1",
    ]


def test_detection_message_groups_camera_cluster_before_internal_height():
    detections = TomatoDetectionArray()
    detections.header.frame_id = "world"
    specifications = (
        ("capture-C0:T7", 0.95),
        ("capture-C1:T2", 0.40),
        ("capture-C1:T3", 0.90),
        ("capture-C0:T8", 0.30),
    )
    detections.detections = [
        TomatoDetection(
            id=identifier,
            center=Point(x=0.0, y=0.0, z=z),
            stem_point=Point(x=0.01, y=0.0, z=z),
        )
        for identifier, z in specifications
    ]

    sorted_message = detection_message_sorted_by_height(detections)

    assert [item.id for item in sorted_message.detections] == [
        "capture-C1:T3",
        "capture-C1:T2",
        "capture-C0:T7",
        "capture-C0:T8",
    ]


def test_detected_tomato_marker_array_validates_frame_and_diameter():
    detections = TomatoDetectionArray()
    with pytest.raises(ValueError, match="frame_id"):
        detected_tomato_marker_array(detections)
    detections.header.frame_id = "camera"
    with pytest.raises(ValueError, match="지름"):
        detected_tomato_marker_array(detections, diameter=0.0)
    with pytest.raises(ValueError, match="줄기점"):
        detected_tomato_marker_array(detections, stem_diameter=0.0)
    with pytest.raises(ValueError, match="진입 방향"):
        detected_tomato_marker_array(detections, approach_length=0.0)


def test_detected_tomato_marker_array_uses_six_mm_stem_marker_by_default():
    detections = TomatoDetectionArray()
    detections.header.frame_id = "camera"
    detections.detections = [
        TomatoDetection(
            id="tomato_a",
            center=Point(x=0.0, y=0.0, z=0.5),
            stem_point=Point(x=0.02, y=0.0, z=0.5),
        )
    ]

    markers = detected_tomato_marker_array(detections).markers

    stem = markers[2]
    assert stem.scale.x == pytest.approx(0.006)
    assert stem.scale.y == pytest.approx(0.006)
    assert stem.scale.z == pytest.approx(0.006)


def test_actual_approach_marker_uses_exact_planned_segment():
    report = {
        "approach_geometry": {
            "frame_id": "detected_tomato_2_tf",
            "preapproach_position": [-0.035, 0.014, -0.018],
            "target_position": [-0.025, 0.010, -0.018],
        }
    }

    marker = actual_approach_marker(2, report)

    assert marker is not None
    assert marker.header.frame_id == "detected_tomato_2_tf"
    assert marker.id == 2
    assert marker.type == marker.ARROW
    assert marker.points[0] == Point(x=-0.035, y=0.014, z=-0.018)
    assert marker.points[1].x == pytest.approx(0.0, abs=1e-9)
    assert marker.points[1].y == pytest.approx(0.0, abs=1e-9)
    assert marker.points[1].z == pytest.approx(-0.018)
    assert marker.color.r == pytest.approx(1.0)
    assert marker.color.g == pytest.approx(0.45)
    assert marker.color.b == pytest.approx(0.0)


def test_actual_approach_marker_ignores_missing_geometry():
    assert actual_approach_marker(0, {}) is None


def test_actual_approach_marker_length_matches_displayed_orange_arrow():
    report = {
        "approach_geometry": {
            "frame_id": "detected_tomato_2_tf",
            "preapproach_position": [-0.035, 0.014, -0.018],
            "target_position": [-0.025, 0.010, -0.018],
        }
    }

    marker = actual_approach_marker(2, report)
    expected = math.dist(
        (marker.points[0].x, marker.points[0].y, marker.points[0].z),
        (marker.points[1].x, marker.points[1].y, marker.points[1].z),
    )

    assert actual_approach_marker_length(report) == pytest.approx(expected)
    assert result_arrow_length_for_report(False, 0.015, report) == (
        pytest.approx(expected)
    )
    assert result_arrow_length_for_report(True, 0.015, report) == (
        pytest.approx(0.015)
    )
    assert result_arrow_length_for_report(False, 0.06, {}) == pytest.approx(
        0.06
    )


def test_detection_markers_do_not_publish_orange_approach_preview():
    detections = TomatoDetectionArray()
    detections.header.frame_id = "camera"
    detections.detections = [
        TomatoDetection(
            id="tomato_a",
            center=Point(x=0.0, y=0.0, z=0.5),
            stem_point=Point(x=0.02, y=0.0, z=0.5),
        )
    ]
    published = []
    gui = SimpleNamespace(
        detection_marker_diameter=0.0175,
        detection_stem_marker_diameter=0.006,
        detection_approach_marker_length=0.06,
        _selected_angle_reference_mode=lambda: "center_to_stem",
        detection_marker_publisher=SimpleNamespace(
            publish=lambda message: published.append(message)
        ),
        _append_log=lambda _message: None,
    )

    HarvestGui._publish_detection_markers(
        gui,
        detections,
    )

    assert len(published) == 1
    assert len(published[0].markers) == 4
    assert all(
        not (
            marker.type == marker.ARROW
            and marker.color.r == pytest.approx(1.0)
            and marker.color.g == pytest.approx(0.45)
        )
        for marker in published[0].markers[1:]
    )


@pytest.mark.parametrize(
    ("camera_source", "expected_publish_count"),
    [
        (CAMERA_SOURCE_REAL, 1),
        (CAMERA_SOURCE_FAKE, 0),
    ],
)
def test_detection_service_response_republishes_only_real_camera_results(
    camera_source,
    expected_publish_count,
):
    detections = TomatoDetectionArray()
    detections.header.frame_id = "camera_color_optical_frame"
    published = []
    callbacks = []
    gui = SimpleNamespace(
        detect_button=SimpleNamespace(configure=lambda **kwargs: None),
        angle_reference_mode_combo=SimpleNamespace(
            configure=lambda **kwargs: None
        ),
        camera_source_combo=SimpleNamespace(configure=lambda **kwargs: None),
        ui_busy=False,
        camera_source_var=SimpleNamespace(get=lambda: camera_source),
        detections_publisher=SimpleNamespace(publish=published.append),
        _detections_callback=callbacks.append,
        status=SimpleNamespace(set=lambda value: None),
        _append_log=lambda value: None,
    )
    response = SimpleNamespace(
        success=True,
        message="검출 완료",
        detections=detections,
    )
    future = SimpleNamespace(result=lambda: response)

    HarvestGui._detection_service_done(gui, future)

    assert len(published) == expected_publish_count
    assert published in ([], [detections])
    assert callbacks == [detections]


def test_capture_camera_calls_trigger_service_and_locks_camera_controls():
    states = {}
    requests = []
    callbacks = []
    future = SimpleNamespace(add_done_callback=callbacks.append)
    client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: requests.append(request) or future,
    )
    gui = SimpleNamespace(
        capture_camera_client=client,
        capture_camera_service="/capture_camera",
        capture_camera_in_progress=False,
        capture_camera_button=SimpleNamespace(
            configure=lambda **kwargs: states.__setitem__("capture", kwargs)
        ),
        detect_button=SimpleNamespace(
            configure=lambda **kwargs: states.__setitem__("detect", kwargs)
        ),
        camera_source_combo=SimpleNamespace(
            configure=lambda **kwargs: states.__setitem__("source", kwargs)
        ),
        status=SimpleNamespace(
            set=lambda value: states.__setitem__("status", value)
        ),
        _append_log=lambda value: states.__setitem__("log", value),
        _capture_camera_done=lambda result: None,
    )

    HarvestGui.capture_camera(gui)

    assert gui.capture_camera_in_progress is True
    assert len(requests) == 1
    assert isinstance(requests[0], Trigger.Request)
    assert callbacks == [gui._capture_camera_done]
    assert states["capture"] == {"state": "disabled"}
    assert states["detect"] == {"state": "disabled"}
    assert states["source"] == {"state": "disabled"}
    assert states["status"] == "카메라 캡처 요청 중..."


@pytest.mark.parametrize(
    ("success", "response_message", "expected_log"),
    [
        (True, "이미지 저장 완료", "[카메라 캡처 성공] 이미지 저장 완료"),
        (False, "카메라 오류", "[카메라 캡처 실패] 카메라 오류"),
    ],
)
@pytest.mark.parametrize(
    ("ui_busy", "expected_detection_state", "expected_source_state"),
    [
        (False, "normal", "readonly"),
        (True, "disabled", "disabled"),
    ],
)
def test_capture_camera_response_updates_status_and_unlocks_capture_button(
    success,
    response_message,
    expected_log,
    ui_busy,
    expected_detection_state,
    expected_source_state,
):
    states = {}
    gui = SimpleNamespace(
        capture_camera_in_progress=True,
        ui_busy=ui_busy,
        capture_camera_button=SimpleNamespace(
            configure=lambda **kwargs: states.__setitem__("capture", kwargs)
        ),
        detect_button=SimpleNamespace(
            configure=lambda **kwargs: states.__setitem__("detect", kwargs)
        ),
        camera_source_combo=SimpleNamespace(
            configure=lambda **kwargs: states.__setitem__("source", kwargs)
        ),
        status=SimpleNamespace(
            set=lambda value: states.__setitem__("status", value)
        ),
        _append_log=lambda value: states.__setitem__("log", value),
    )
    response = SimpleNamespace(success=success, message=response_message)

    HarvestGui._capture_camera_done(
        gui,
        SimpleNamespace(result=lambda: response),
    )

    assert gui.capture_camera_in_progress is False
    assert states["capture"] == {"state": "normal"}
    assert states["detect"] == {"state": expected_detection_state}
    assert states["source"] == {"state": expected_source_state}
    assert states["status"] == response_message
    assert states["log"] == expected_log


def _compressed_test_image(width=8, height=4):
    output = io.BytesIO()
    PilImage.new("RGB", (width, height), color=(220, 30, 20)).save(
        output,
        format="JPEG",
    )
    return output.getvalue()


def _raw_test_image(width=8, height=4, encoding="rgb8"):
    message = RosImage()
    message.width = width
    message.height = height
    message.encoding = encoding
    channels = 1 if encoding == "mono8" else 3
    message.step = width * channels
    pixel = b"\xDC" if channels == 1 else b"\xDC\x1E\x14"
    message.data = pixel * (width * height)
    return message


def test_decode_compressed_result_image_returns_independent_rgb_image():
    image = decode_compressed_result_image(_compressed_test_image())

    assert image.mode == "RGB"
    assert image.size == (8, 4)


def test_decode_compressed_result_image_rejects_empty_or_invalid_payload():
    with pytest.raises(ValueError, match="비어"):
        decode_compressed_result_image(b"")
    with pytest.raises(ValueError, match="해석"):
        decode_compressed_result_image(b"not an image")


def test_decode_raw_result_image_supports_rgb_bgr_and_row_stride():
    rgb = _raw_test_image(width=4, height=2, encoding="rgb8")
    assert decode_raw_result_image(rgb).size == (4, 2)

    bgr = RosImage()
    bgr.width = 2
    bgr.height = 1
    bgr.encoding = "bgr8"
    bgr.step = 8
    bgr.data = bytes((10, 20, 30, 40, 50, 60, 0, 0))
    decoded = decode_raw_result_image(bgr)
    assert decoded.getpixel((0, 0)) == (30, 20, 10)
    assert decoded.getpixel((1, 0)) == (60, 50, 40)


def test_camera_xyz_projection_scales_camera_info_to_raw_image():
    pixel = camera_xyz_to_image_pixel(
        (0.1, -0.05, 1.0),
        (900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0),
        (1280, 720),
        (640, 360),
    )
    assert pixel == pytest.approx((365.0, 157.5))


def test_detection_label_layout_avoids_other_tomato_centers():
    anchors = ((100.0, 100.0), (115.0, 105.0), (105.0, 120.0))
    rectangles = detection_label_layout(
        anchors,
        ((55.0, 20.0),) * len(anchors),
        (400, 300),
    )
    for rectangle in rectangles:
        for point in anchors:
            assert not (
                rectangle[0] - 10.0 <= point[0] <= rectangle[2] + 10.0
                and rectangle[1] - 10.0 <= point[1] <= rectangle[3] + 10.0
            )


def test_detection_label_layout_enforces_arrow_based_left_right_side():
    anchors = ((220.0, 100.0), (420.0, 100.0))
    rectangles = detection_label_layout(
        anchors,
        ((90.0, 34.0), (90.0, 34.0)),
        (640, 360),
        preferred_sides=(-1, 1),
    )

    assert (rectangles[0][0] + rectangles[0][2]) / 2.0 < anchors[0][0]
    assert (rectangles[1][0] + rectangles[1][2]) / 2.0 > anchors[1][0]


def test_detection_label_layout_avoids_box_overlap_in_dense_cluster():
    anchors = tuple(
        (600.0 + (index % 3) * 14.0, 300.0 + (index // 3) * 14.0)
        for index in range(12)
    )
    rectangles = detection_label_layout(
        anchors,
        ((82.0, 34.0),) * len(anchors),
        (1280, 720),
        padding_scale=0.35,
        preferred_sides=tuple(-1 if index % 2 == 0 else 1 for index in range(12)),
    )

    for index, rectangle in enumerate(rectangles):
        for other in rectangles[index + 1:]:
            assert (
                harvest_gui_module._rectangle_overlap_area(rectangle, other)
                == 0.0
            )


def test_detection_label_layout_uses_outer_column_before_vertical_shift():
    anchors = ((640.0, 360.0), (640.0, 360.0))
    rectangles = detection_label_layout(
        anchors,
        ((76.0, 30.0), (76.0, 30.0)),
        (1280, 720),
        padding_scale=0.35,
        preferred_sides=(-1, -1),
    )
    centers = tuple(
        ((left + right) / 2.0, (top + bottom) / 2.0)
        for left, top, right, bottom in rectangles
    )

    assert centers[1][0] < centers[0][0]
    assert centers[1][1] == pytest.approx(centers[0][1])


def test_detection_overlay_draws_projected_labels_on_clean_image():
    image = PilImage.new("RGB", (640, 360), color=(240, 240, 240))
    overlay, count = draw_detection_label_overlay(
        image,
        (("C0:T1", (0.0, 0.0, 1.0)), ("C0:T2", (0.08, 0.04, 1.0))),
        (450.0, 0.0, 320.0, 0.0, 450.0, 180.0, 0.0, 0.0, 1.0),
        (640, 360),
    )
    assert count == 2
    assert overlay.size == image.size
    assert overlay.tobytes() != image.tobytes()


def test_detection_overlay_draws_blue_recommend_entry_arrow():
    image = PilImage.new("RGB", (640, 360), color=(240, 240, 240))
    overlay, count = draw_detection_label_overlay(
        image,
        (("C0:T1", (0.0, 0.0, 1.0), (-0.04, 0.0, 1.0)),),
        (450.0, 0.0, 320.0, 0.0, 450.0, 180.0, 0.0, 0.0, 1.0),
        (640, 360),
    )

    assert count == 1
    pixels = np.asarray(overlay)
    assert np.any(np.all(pixels == (15, 108, 148), axis=2))


def test_result_image_callback_decodes_and_schedules_gui_render():
    values = {}
    scheduled = []
    gui = SimpleNamespace(
        result_image_topic="/tomato_vision/result_image",
        result_image_status=SimpleNamespace(
            set=lambda value: values.__setitem__("status", value)
        ),
        latest_result_image=None,
        _schedule_result_image_render=lambda: scheduled.append(True),
        _append_log=lambda value: values.__setitem__("log", value),
    )
    message = CompressedImage()
    message.format = "jpeg"
    message.data = _compressed_test_image(width=12, height=6)

    HarvestGui._result_image_callback(gui, message)

    assert gui.latest_result_image.size == (12, 6)
    assert scheduled == [True]
    assert values["status"].endswith("12×6 · jpeg")
    assert "log" not in values


def test_camera_color_image_callback_uses_independent_tab_state():
    values = {}
    scheduled = []
    gui = SimpleNamespace(
        camera_color_image_topic="/tomato_vision/result_image",
        camera_color_image_status=SimpleNamespace(
            set=lambda value: values.__setitem__("status", value)
        ),
        latest_camera_color_image=None,
        _update_detection_image_overlay=lambda: scheduled.append("overlay"),
        _append_log=lambda value: values.__setitem__("log", value),
    )
    message = _raw_test_image(width=16, height=9, encoding="rgb8")

    HarvestGui._camera_color_image_callback(gui, message)

    assert gui.latest_camera_color_image.size == (16, 9)
    assert scheduled == ["overlay"]
    assert values["status"].endswith("16×9 · rgb8")
    assert "log" not in values


def test_vision_result_image_callback_displays_raw_without_overlay():
    values = {}
    scheduled = []
    gui = SimpleNamespace(
        vision_result_image_topic="/tomato_vision/result_image_raw",
        result_image_status=SimpleNamespace(
            set=lambda value: values.__setitem__("status", value)
        ),
        latest_result_image=None,
        _schedule_result_image_render=lambda: scheduled.append(True),
        _append_log=lambda value: values.__setitem__("log", value),
    )
    message = _raw_test_image(width=16, height=9, encoding="bgr8")

    HarvestGui._vision_result_image_callback(gui, message)

    assert gui.latest_result_image.size == (16, 9)
    assert scheduled == [True]
    assert values["status"].endswith("16×9 · bgr8")
    assert "log" not in values


def test_camera_color_info_callback_updates_overlay_only_when_changed():
    updates = []
    gui = SimpleNamespace(
        latest_camera_color_info=None,
        _update_detection_image_overlay=lambda: updates.append(True),
    )
    message = CameraInfo()
    message.width = 1280
    message.height = 720
    message.header.frame_id = "d435_color_optical_frame"
    message.k = [900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0]

    HarvestGui._camera_color_info_callback(gui, message)
    HarvestGui._camera_color_info_callback(gui, message)

    assert gui.latest_camera_color_info is message
    assert updates == [True]


def test_detection_overlay_is_rendered_only_in_camera_color_tab():
    source = PilImage.new("RGB", (640, 360), color=(240, 240, 240))
    vision_result = PilImage.new("RGB", (32, 18), color=(10, 20, 30))
    camera_info = CameraInfo()
    camera_info.width = 640
    camera_info.height = 360
    camera_info.header.frame_id = "d435_color_optical_frame"
    camera_info.k = [450.0, 0.0, 320.0, 0.0, 450.0, 180.0, 0.0, 0.0, 1.0]
    detection = TomatoDetection()
    detection.id = "capture/C0:T1"
    detection.center = Point(x=0.0, y=0.0, z=1.0)
    detection.stem_point = Point(x=0.04, y=0.0, z=1.0)
    detections = TomatoDetectionArray()
    detections.header.frame_id = "d435_color_optical_frame"
    detections.detections = [detection]
    scheduled = []
    gui = SimpleNamespace(
        latest_camera_color_image=source,
        latest_camera_color_display_image=None,
        latest_camera_color_info=camera_info,
        latest_detection_message=detections,
        latest_result_image=vision_result,
        camera_color_image_topic="/tomato_vision/camera_preview",
        camera_color_info_topic="/camera/d435/color/camera_info",
        camera_color_image_status=SimpleNamespace(set=lambda _value: None),
        _selected_angle_reference_mode=lambda: ANGLE_REFERENCE_CENTER_TO_STEM,
        get_parameter=lambda _name: SimpleNamespace(
            value="d435_color_optical_frame"
        ),
        _schedule_camera_color_image_render=lambda: scheduled.append(True),
        _append_log=lambda _value: None,
    )

    HarvestGui._update_detection_image_overlay(gui)

    assert gui.latest_camera_color_display_image.tobytes() != source.tobytes()
    assert gui.latest_result_image is vision_result
    assert scheduled == [True]


def test_camera_source_change_selects_client_and_clears_old_detection():
    values = {}
    deleted = []
    fake_client = object()
    real_client = object()
    gui = SimpleNamespace(
        camera_source_var=SimpleNamespace(get=lambda: CAMERA_SOURCE_REAL),
        fake_camera_service="/fake_tomato_camera/detect_tomatoes",
        real_camera_service="/detect_tomatoes",
        camera_clients={
            CAMERA_SOURCE_FAKE: fake_client,
            CAMERA_SOURCE_REAL: real_client,
        },
        camera_service_display=SimpleNamespace(
            set=lambda value: values.__setitem__("service", value)
        ),
        detected_tomatoes=[object()],
        detected_tomato_expected_world_positions={0: (1.0, 2.0, 3.0)},
        detected_tomato_expected_world_x_axes={0: (1.0, 0.0)},
        detected_tomato_record_positions={0: (1.0, 2.0, 3.0)},
        detected_tomato_record_stem_positions={0: (1.1, 2.1, 3.1)},
        detected_tomato_record_calyx_positions={0: (1.05, 2.05, 3.05)},
        detected_tomato_record_angle_origins={0: (1.0, 2.0, 3.0)},
        detected_tomato_record_angle_targets={0: (1.1, 2.1, 3.1)},
        tomato_motion_results={0: "Plan 성공"},
        result_arrow_lengths={0: 0.1},
        result_detection_frame="old_camera_frame",
        detection_signature=(1, 2, 3),
        current_detection_stamp_ns=123,
        detection_generation=4,
        tomato_tree=SimpleNamespace(
            get_children=lambda: ("0",),
            delete=deleted.append,
        ),
        tomato_combo=SimpleNamespace(
            configure=lambda **kwargs: values.__setitem__(
                "tomato_values", kwargs["values"]
            )
        ),
        selected_tomato=SimpleNamespace(
            set=lambda value: values.__setitem__("selected", value)
        ),
        plan_button=SimpleNamespace(configure=lambda **kwargs: None),
        execute_button=SimpleNamespace(configure=lambda **kwargs: None),
        harvest_all_button=SimpleNamespace(configure=lambda **kwargs: None),
        harvest_all_plan_button=SimpleNamespace(
            configure=lambda **kwargs: None
        ),
        _invalidate_plan=lambda: values.__setitem__("invalidated", True),
        _clear_detection_markers=lambda: values.__setitem__(
            "markers_cleared", True
        ),
        _update_selected_tomato_plot=lambda: values.__setitem__(
            "plot_cleared", True
        ),
        _update_step_controls=lambda: None,
        status=SimpleNamespace(
            set=lambda value: values.__setitem__("status", value)
        ),
        _append_log=lambda value: values.__setitem__("log", value),
    )

    HarvestGui._camera_source_changed(gui)

    assert gui.camera_client is real_client
    assert gui.camera_service == "/detect_tomatoes"
    assert gui.detected_tomatoes == []
    assert gui.detected_tomato_expected_world_positions == {}
    assert gui.detected_tomato_record_positions == {}
    assert gui.detected_tomato_record_stem_positions == {}
    assert gui.detected_tomato_record_calyx_positions == {}
    assert gui.detection_generation == 5
    assert deleted == ["0"]
    assert values["service"] == "/detect_tomatoes"
    assert values["tomato_values"] == []
    assert values["selected"] == ""
    assert values["invalidated"] is True
    assert values["markers_cleared"] is True
    assert values["plot_cleared"] is True


def test_stop_active_motion_cancels_individual_harvest_without_failure_result():
    events = []

    class RunningProcess:
        terminated = False

        def poll(self):
            return None

        def terminate(self):
            self.terminated = True

    process = RunningProcess()
    gui = SimpleNamespace(
        sweep_active=False,
        batch_active=False,
        harvest_process=process,
        status=SimpleNamespace(
            set=lambda message: events.append(("status", message))
        ),
        _append_log=lambda message: events.append(("log", message)),
        _request_robot_motion_stop=lambda: events.append(("robot_stop",)),
        _invalidate_plan=lambda: events.append(("invalidate",)),
        _set_busy=lambda busy: events.append(("busy", busy)),
    )

    HarvestGui.stop_active_motion(gui)

    assert gui.harvest_process is None
    assert process.terminated
    assert ("robot_stop",) in events
    assert ("invalidate",) in events
    assert ("busy", False) in events


def test_stop_active_motion_finishes_batch_as_user_cancellation():
    events = []
    process = SimpleNamespace(
        poll=lambda: None,
        terminate=lambda: events.append(("terminate",)),
    )
    gui = SimpleNamespace(
        sweep_active=False,
        batch_active=True,
        harvest_process=process,
        status=SimpleNamespace(
            set=lambda message: events.append(("status", message))
        ),
        _append_log=lambda message: events.append(("log", message)),
        _request_robot_motion_stop=lambda: events.append(("robot_stop",)),
        _invalidate_plan=lambda: events.append(("invalidate",)),
        _finish_batch=lambda success, message: events.append(
            ("finish", success, message)
        ),
    )

    HarvestGui.stop_active_motion(gui)

    assert gui.harvest_process is None
    assert ("terminate",) in events
    assert ("robot_stop",) in events
    finish = next(event for event in events if event[0] == "finish")
    assert finish[1] is False
    assert "사용자" in finish[2]


def test_stop_active_motion_delegates_automatic_test_stop():
    events = []
    gui = SimpleNamespace(
        sweep_active=True,
        stop_sweep=lambda: events.append(("stop_sweep",)),
    )

    HarvestGui.stop_active_motion(gui)

    assert events == [("stop_sweep",)]


def test_harvest_command_builds_execute_command():
    command = harvest_command(7, True, python_executable="python3")

    assert "tomato_frame:=detected_tomato_7_tf" in command
    assert "execute:=true" in command
    assert "planning_pipeline_id:=ompl" in command
    assert "planner_id:=RRTConnect" in command
    assert "preapproach_mode:=cartesian" in command
    assert "continuous_transition:=false" in command
    assert "return_to_pick_ready:=true" in command
    assert "retreat_after_harvest:=false" in command
    assert "pick_ready_state_name:=PICK_READY" in command


def test_harvest_command_can_disable_trajectory_display_for_automatic_test():
    command = harvest_command(0, False, publish_display_trajectory=False)

    assert "publish_display_trajectory:=false" in command


def test_named_pose_command_builds_constrained_ompl_execute_command():
    command = named_pose_command(
        "CAPTURE_LEFT",
        velocity_scale=0.35,
        acceleration_scale=0.25,
        python_executable="/usr/bin/python3",
    )

    assert command[:3] == [
        "/usr/bin/python3",
        "-m",
        "rbpodo_tomato_harvest.named_pose_move",
    ]
    assert "pick_ready_state_name:=CAPTURE_LEFT" in command
    assert "pick_ready_joint_tolerance:=0.0001" in command
    assert "joint_planning_pipeline_id:=ompl" in command
    assert "joint_planner_id:=RRTConnect" in command
    assert "pick_ready_velocity_scale:=0.35" in command
    assert "pick_ready_acceleration_scale:=0.25" in command
    assert "publish_display_trajectory:=true" in command


def test_named_pose_command_rejects_unknown_state():
    with pytest.raises(ValueError, match="named pose"):
        named_pose_command("UNKNOWN_POSE")


def test_harvest_command_builds_pilz_lin_command():
    command = harvest_command(
        3,
        False,
        planning_pipeline_id="pilz_industrial_motion_planner",
        planner_id="LIN",
        preapproach_mode="planner",
    )

    assert "planning_pipeline_id:=pilz_industrial_motion_planner" in command
    assert "planner_id:=LIN" in command
    assert "preapproach_mode:=planner" in command


def test_planner_options_include_cartesian_and_pipeline_modes():
    assert PICK_READY_STATES == ("PICK_READY", "PICK_READY_RIGHT")
    assert NAMED_POSE_STATES == (
        "PICK_READY",
        "PICK_READY_RIGHT",
        "CAPTURE_LEFT",
        "CAPTURE_RIGHT",
    )
    assert PLANNER_CONFIGS["Cartesian"] == (
        "ompl",
        "RRTConnect",
        "cartesian",
    )
    assert PLANNER_CONFIGS["OMPL / RRTConnect"] == (
        "ompl",
        "RRTConnect",
        "planner",
    )


def test_gui_planner_is_fixed_to_cartesian_first_mode():
    gui = SimpleNamespace()

    assert HarvestGui._selected_planner_config(gui) == (
        "ompl",
        "RRTConnect",
        "cartesian",
    )
    assert GUI_PLANNER_CONFIG == PLANNER_CONFIGS["Cartesian"]


@pytest.mark.parametrize(
    ("mode", "expected"),
    [
        ("cartesian", ("cartesian_trajectory",)),
        ("planner", ("pipeline_trajectory",)),
    ],
)
def test_preapproach_selection_routes_to_requested_planner(mode, expected):
    calls = []
    planner = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(
            value=mode if name == "preapproach_mode" else ""
        ),
        _plan_cartesian_with_ompl_fallback=lambda waypoints, start_state, label, pregrasp: (
            calls.append(("cartesian", waypoints, start_state, label))
            or ("cartesian_trajectory",)
        ),
        _plan_pose_target=lambda pose, start_state, label, **kwargs: (
            calls.append(("planner", pose, start_state, label))
            or "pipeline_trajectory"
        ),
        _trajectory_end_state=lambda trajectory: "planned_end_state",
    )

    result = CartesianHarvestPlanner._plan_preapproach(
        planner,
        "target_pose",
        "pick_ready_state",
    )

    assert result == expected
    assert calls[0][0] == mode


def test_chomp_preapproach_uses_ik_joint_target_route():
    calls = []
    planner = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(
            value={
                "preapproach_mode": "planner",
                "planning_pipeline_id": "chomp",
            }.get(name, "")
        ),
        _plan_chomp_pose_target=lambda pose, start_state: (
            calls.append((pose, start_state)) or "chomp_trajectory"
        ),
        _trajectory_end_state=lambda trajectory: "planned_end_state",
    )

    result = CartesianHarvestPlanner._plan_preapproach(
        planner,
        "planning_target_pose",
        "pick_ready_state",
    )

    assert result == ("chomp_trajectory",)
    assert calls == [("planning_target_pose", "pick_ready_state")]


def test_cartesian_failure_falls_back_to_constrained_ompl_per_waypoint():
    calls = []

    def trajectory(position):
        result = RobotTrajectory()
        result.joint_trajectory.joint_names = ["base"]
        result.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[position])
        ]
        return result

    fallback_trajectories = iter((trajectory(0.1), trajectory(0.2)))
    planner = SimpleNamespace(
        last_plan_report={
            "stages": [
                {
                    "stage": "CARTESIAN_APPROACH",
                    "reason": "CARTESIAN_FRACTION_LOW",
                    "cartesian_fraction": 0.4,
                    "required_fraction": 0.98,
                }
            ]
        },
        get_parameter=lambda name: SimpleNamespace(
            value={
                "ompl_joint_tolerance_deg": 120.0,
                "joint_planner_id": "RRTConnect",
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            warning=lambda message: None,
            info=lambda message: None,
            error=lambda message: None,
        ),
        _plan_cartesian=lambda *args, **kwargs: None,
        _plan_pose_target=lambda pose, start_state, label, **kwargs: (
            calls.append((pose, start_state, label, kwargs))
            or next(fallback_trajectories)
        ),
        _trajectory_end_state=lambda planned: (
            CartesianHarvestPlanner._trajectory_end_state(planned)
        ),
    )
    start_state = RobotState()

    result = CartesianHarvestPlanner._plan_cartesian_with_ompl_fallback(
        planner,
        ["first_pose", "second_pose"],
        start_state,
        "Approach and pre-wait harvest",
        pregrasp=False,
    )

    assert len(result) == 2
    assert calls[0][0] == "first_pose"
    assert calls[0][1] is start_state
    assert calls[0][3]["pipeline_id"] == "ompl"
    assert calls[0][3]["planner_id"] == "RRTConnect"
    assert calls[0][3]["pregrasp"] is False
    assert calls[1][0] == "second_pose"
    assert list(calls[1][1].joint_state.position) == [0.1]
    assert planner.last_plan_report["cartesian_fallbacks"] == [
        {
            "segment": "APPROACH",
            "cartesian_stage": "CARTESIAN_APPROACH",
            "cartesian_reason": "CARTESIAN_FRACTION_LOW",
            "cartesian_fraction": 0.4,
            "required_fraction": 0.98,
            "waypoint_count": 2,
            "planner": "ompl/RRTConnect",
            "ompl_stages": [
                "OMPL_FALLBACK_APPROACH_1",
                "OMPL_FALLBACK_APPROACH_2",
            ],
            "success": True,
        }
    ]


def test_harvest_command_rejects_negative_index():
    with pytest.raises(ValueError):
        harvest_command(-1, False)


def test_harvest_command_rejects_unknown_planner():
    with pytest.raises(ValueError):
        harvest_command(
            0,
            False,
            planning_pipeline_id="unknown",
        )


def test_harvest_all_jobs_plans_and_executes_each_tomato_in_order():
    assert harvest_all_jobs(3) == [
        (0, False),
        (0, True),
        (1, False),
        (1, True),
        (2, False),
        (2, True),
    ]


def test_harvest_all_jobs_accepts_empty_detection_list():
    assert harvest_all_jobs(0) == []


def test_harvest_all_jobs_rejects_negative_count():
    with pytest.raises(ValueError):
        harvest_all_jobs(-1)


def test_batch_harvest_stage_limit_uses_per_tomato_stage_number():
    assert batch_harvest_stage_limit("전체 수확") is None
    assert batch_harvest_stage_limit("3단계까지") == 3
    assert batch_harvest_stage_limit("4단계까지") == 4


def test_batch_harvest_stage_limit_rejects_tomato_number_selection():
    with pytest.raises(ValueError):
        batch_harvest_stage_limit("3번 토마토까지")


def test_success_marker_is_green_and_points_along_tomato_positive_x():
    marker = harvest_result_marker(3, True, 0.05)

    assert marker.header.frame_id == "detected_tomato_3_tf"
    assert marker.type == marker.ARROW
    assert len(marker.points) == 2
    assert marker.points[0] == Point()
    assert marker.points[1] == Point(x=0.05)
    assert marker.scale.x == 0.008
    assert (marker.color.r, marker.color.g, marker.color.b, marker.color.a) == (
        0.0,
        1.0,
        0.0,
        1.0,
    )


def test_adaptive_rotation_success_marker_is_sky_blue():
    marker = harvest_result_marker(
        0,
        True,
        0.04,
        adaptive_rotation_applied=True,
        approach_axis_local=(0.6, -0.8),
    )

    assert marker.points[1].x == pytest.approx(0.024)
    assert marker.points[1].y == pytest.approx(-0.032)
    assert (marker.color.r, marker.color.g, marker.color.b, marker.color.a) == (
        0.2,
        0.8,
        1.0,
        1.0,
    )


def test_nonrotated_success_marker_uses_exact_planner_axis():
    marker = harvest_result_marker(
        0,
        True,
        0.04,
        approach_axis_local=(1.0, 0.0),
    )

    assert marker.points[1].x == pytest.approx(0.04)
    assert marker.points[1].y == pytest.approx(0.0)


def test_failure_marker_is_red():
    marker = harvest_result_marker(0, False, 0.04)

    assert marker.points[0].x == pytest.approx(-0.04)
    assert marker.points[0].y == pytest.approx(0.0)
    assert marker.points[1] == Point()
    assert (marker.color.r, marker.color.g, marker.color.b, marker.color.a) == (
        1.0,
        0.0,
        0.0,
        1.0,
    )


def test_failure_marker_points_from_plus_x_toward_minus_y_after_rotation():
    marker = harvest_result_marker(
        0,
        False,
        0.04,
        adaptive_rotation_deg=45.0,
    )

    component = 0.04 / (2.0 ** 0.5)
    assert marker.points[0].x == pytest.approx(-component)
    assert marker.points[0].y == pytest.approx(component)
    assert marker.points[1] == Point()


def test_failure_marker_points_toward_plus_y_after_negative_rotation():
    marker = harvest_result_marker(
        0,
        False,
        0.04,
        adaptive_rotation_deg=-45.0,
    )

    component = 0.04 / (2.0 ** 0.5)
    assert marker.points[0].x == pytest.approx(-component)
    assert marker.points[0].y == pytest.approx(-component)
    assert marker.points[1] == Point()


def test_failure_marker_uses_exact_planner_approach_axis():
    marker = harvest_result_marker(
        0,
        False,
        0.04,
        adaptive_rotation_deg=0.0,
        approach_axis_local=(0.6, -0.8),
    )

    assert marker.points[0].x == pytest.approx(-0.024)
    assert marker.points[0].y == pytest.approx(0.032)
    assert marker.points[1] == Point()


def test_adaptive_rotation_report_requires_nonzero_applied_angle():
    assert adaptive_rotation_was_applied(
        {"adaptive_grasp": {"applied_rotation_deg": 15.0}}
    )
    assert not adaptive_rotation_was_applied(
        {"adaptive_grasp": {"applied_rotation_deg": 0.0}}
    )
    assert adaptive_rotation_degrees(
        {"adaptive_grasp": {"applied_rotation_deg": 45.0}}
    ) == pytest.approx(45.0)
    assert adaptive_rotation_was_applied(
        {"adaptive_grasp": {"applied_rotation_deg": -30.0}}
    )
    assert adaptive_rotation_degrees(
        {"adaptive_grasp": {"applied_rotation_deg": -45.0}}
    ) == pytest.approx(-45.0)
    assert adaptive_approach_axis_local(
        {
            "adaptive_grasp": {
                "approach_axis_tomato_local": [3.0, -4.0, 0.0]
            }
        }
    ) == pytest.approx((0.6, -0.8))


def test_final_approach_geometry_overrides_stale_recommend_axis_on_failure():
    report = {
        "success": False,
        "adaptive_grasp": {
            "approach_axis_tomato_local": [1.0, 0.0, 0.0],
        },
        "approach_geometry": {
            "preapproach_position": [-0.03, 0.03, -0.018],
            "target_position": [-0.02, 0.02, -0.018],
        },
    }

    assert adaptive_approach_axis_local(report) == pytest.approx(
        (2.0 ** -0.5, -(2.0 ** -0.5))
    )


def test_sweep_marker_is_frozen_in_robot_base_frame():
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.4, y=-0.2, z=0.7),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )

    marker = sweep_result_marker(12, True, 0.03, "link0", transform)

    assert marker.header.frame_id == "link0"
    assert marker.ns == "harvest_sweep_result"
    assert marker.id == 12
    assert marker.pose.position == Point(x=0.4, y=-0.2, z=0.7)


def test_sweep_success_marker_is_sky_blue_when_grasp_rotated():
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.4, y=-0.2, z=0.7),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )

    marker = sweep_result_marker(
        12,
        True,
        0.03,
        "link0",
        transform,
        adaptive_rotation_applied=True,
    )

    assert (marker.color.r, marker.color.g, marker.color.b, marker.color.a) == (
        0.2,
        0.8,
        1.0,
        1.0,
    )


def test_harvest_result_marker_rejects_negative_index():
    with pytest.raises(ValueError):
        harvest_result_marker(-1, True, 0.04)


def test_harvest_result_marker_rejects_nonpositive_length():
    with pytest.raises(ValueError):
        harvest_result_marker(0, True, 0.0)


def test_arrow_length_ends_before_stem_center_on_horizontal_x():
    center = Point(x=0.0, y=0.0, z=0.0)
    stem = Point(x=0.10, y=0.0, z=0.20)

    length = tomato_stem_arrow_length(
        center,
        stem,
        source_to_parent_quaternion=(0.0, 0.0, 0.0, 1.0),
        stem_margin=0.008,
    )

    assert length == pytest.approx(0.092)


def test_arrow_length_rotates_camera_vector_before_ground_projection():
    center = Point(x=0.0, y=0.0, z=0.0)
    stem = Point(x=0.0, y=0.0, z=0.10)
    half_sqrt_two = 2**-0.5

    length = tomato_stem_arrow_length(
        center,
        stem,
        source_to_parent_quaternion=(0.0, half_sqrt_two, 0.0, half_sqrt_two),
        stem_margin=0.008,
    )

    assert length == pytest.approx(0.092)


def test_batch_plan_failure_marks_yellow_skips_execute_and_continues():
    events = []
    gui = SimpleNamespace(
        batch_jobs=deque([(0, True), (1, False), (1, True)]),
        batch_skipped=0,
        batch_completed=0,
        batch_total=2,
        detection_generation=4,
        harvest_plan_report={
            "adaptive_grasp": {"applied_rotation_deg": 30.0}
        },
        _set_harvest_result=lambda index, success, **kwargs: events.append(
            ("marker", index, success)
        ),
        _record_batch_statistics=lambda return_code, execute, verification: (
            events.append(("statistics", return_code, execute, verification))
        ),
        _invalidate_plan=lambda: events.append(("invalidate",)),
        _append_log=lambda message: events.append(("log", message)),
        _start_next_batch_job=lambda: events.append(("next",)),
        _finish_batch=lambda success, message: events.append(
            ("finish", success, message)
        ),
    )

    HarvestGui._handle_batch_job_done(
        gui,
        return_code=1,
        execute=False,
        verification=(4, 0, "ompl", "RRTConnect", "planner"),
    )

    assert gui.batch_skipped == 1
    assert list(gui.batch_jobs) == [(1, False), (1, True)]
    assert ("marker", 0, False) in events
    assert any(event[0] == "statistics" for event in events)
    assert ("next",) in events
    assert not any(event[0] == "finish" for event in events)


def test_batch_execution_failure_marks_yellow_and_stops():
    events = []
    gui = SimpleNamespace(
        batch_jobs=deque([(1, False), (1, True)]),
        batch_skipped=0,
        batch_completed=0,
        batch_total=2,
        detection_generation=4,
        harvest_plan_report={
            "adaptive_grasp": {"applied_rotation_deg": 30.0},
            "execution_attempted": False,
            "failure_stage": "DYNAMIC_BASE_TF_SYNC",
            "failure_reason": "FRESH_LIFT_TF_NOT_RECEIVED",
        },
        _set_harvest_result=lambda index, success, **kwargs: events.append(
            ("marker", index, success)
        ),
        _record_batch_statistics=lambda return_code, execute, verification: (
            events.append(("statistics", return_code, execute, verification))
        ),
        _finish_batch=lambda success, message: events.append(
            ("finish", success, message)
        ),
    )

    HarvestGui._handle_batch_job_done(
        gui,
        return_code=1,
        execute=True,
        verification=(4, 0, "ompl", "RRTConnect", "planner"),
    )

    assert ("marker", 0, False) in events
    assert any(event[0] == "statistics" for event in events)
    finish = next(event for event in events if event[0] == "finish")
    assert finish[1] is False
    assert "실제 모션 시작 전 재계획 실패" in finish[2]
    assert "DYNAMIC_BASE_TF_SYNC" in finish[2]
    assert "실제 trajectory는 실행되지 않았습니다" in finish[2]


def test_record_batch_statistics_appends_one_final_tomato_row():
    records = []
    summaries = []
    gui = SimpleNamespace(
        batch_scene=(0.55, 0.1, 0.4, 60.0),
        harvest_plan_report={
            "execution_attempted": True,
            "execution_success": True,
            "execution_duration_sec": 9.25,
            "duration_sec": 0.75,
        },
        sweep_completed=0,
        batch_total=4,
        sweep_summary=SimpleNamespace(set=summaries.append),
        _update_sweep_statistics=records.append,
    )

    HarvestGui._record_batch_statistics(
        gui,
        return_code=0,
        execute=True,
        verification=(3, 1, "ompl", "RRTConnect", "cartesian"),
    )

    assert gui.sweep_completed == 1
    assert len(records) == 1
    assert records[0]["tomato"] == 1
    assert records[0]["execute_motion"] is True
    assert records[0]["execution_duration_sec"] == pytest.approx(9.25)
    assert summaries == ["전체 연속 수확 — 토마토 1 / 4 결과 집계"]


def test_scene_parameters_include_position_and_rotation():
    parameters = scene_parameters([0.43, -0.4, 0.4], 35.0)

    assert [parameter.name for parameter in parameters] == [
        "object_position",
        "tomato_z_spin_deg",
    ]
    assert parameters[0].value.type == ParameterType.PARAMETER_DOUBLE_ARRAY
    assert list(parameters[0].value.double_array_value) == [0.43, -0.4, 0.4]
    assert parameters[1].value.type == ParameterType.PARAMETER_DOUBLE
    assert parameters[1].value.double_value == 35.0


def test_scene_parameters_requires_three_coordinates():
    with pytest.raises(ValueError):
        scene_parameters([0.1, 0.2], 0.0)
