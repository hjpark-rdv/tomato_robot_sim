from collections import deque
import random
from types import SimpleNamespace

import pytest

from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterType

from rbpodo_tomato_harvest.harvest_gui import (
    HarvestGui,
    generate_sweep_cases,
    harvest_all_jobs,
    harvest_command,
    harvest_result_marker,
    scene_parameters,
    sweep_result_marker,
    tomato_stem_arrow_length,
)
from rbpodo_tomato_harvest.harvest_planner import CartesianHarvestPlanner
from rbpodo_tomato_harvest.tomato_harvest_worker import _apply_request


def test_generate_sweep_cases_stops_when_first_axis_reaches_end():
    cases = generate_sweep_cases(
        start=(0.0, 0.0, 0.0, 0.0),
        end=(0.02, 0.01, 0.0, 10.0),
        step=(0.01, 0.01, 0.0, 5.0),
        randomized=(False, False, False, False),
    )

    assert cases == [
        (0.0, 0.0, 0.0, 0.0),
        (0.01, 0.01, 0.0, 5.0),
    ]


def test_generate_sweep_cases_uses_exact_steps_until_earliest_end():
    cases = generate_sweep_cases(
        start=(0.0, 0.0, 0.0, 0.0),
        end=(1.0, 0.6, 0.0, 90.0),
        step=(0.1, 0.3, 0.0, 5.0),
        randomized=(False, False, False, False),
    )

    assert cases == [
        (0.0, 0.0, 0.0, 0.0),
        (0.1, 0.3, 0.0, 5.0),
        (0.2, 0.6, 0.0, 10.0),
    ]


def test_generate_sweep_cases_randomizes_only_checked_axis():
    cases = generate_sweep_cases(
        start=(0.0, 1.0, 2.0, 0.0),
        end=(0.02, 1.0, 2.0, 10.0),
        step=(0.01, 0.0, 0.0, 5.0),
        randomized=(True, False, False, False),
        rng=random.Random(7),
    )

    assert len(cases) == 3
    assert all(0.0 <= case[0] <= 0.02 for case in cases)
    assert [case[1:] for case in cases] == [
        (1.0, 2.0, 0.0),
        (1.0, 2.0, 5.0),
        (1.0, 2.0, 10.0),
    ]


def test_random_axis_step_does_not_control_termination():
    cases = generate_sweep_cases(
        start=(0.0, 0.0, 0.0, 0.0),
        end=(10.0, 0.0, 0.0, 10.0),
        step=(0.0, 0.0, 0.0, 5.0),
        randomized=(True, False, False, False),
        rng=random.Random(3),
    )

    assert len(cases) == 3
    assert all(0.0 <= case[0] <= 10.0 for case in cases)
    assert [case[3] for case in cases] == [0.0, 5.0, 10.0]


def test_all_changing_axes_random_requires_nonrandom_termination_axis():
    with pytest.raises(ValueError):
        generate_sweep_cases(
            start=(0.0, 0.0, 0.0, 0.0),
            end=(1.0, 1.0, 0.0, 0.0),
            step=(0.0, 0.0, 0.0, 0.0),
            randomized=(True, True, False, False),
        )


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
    node = SimpleNamespace(
        tomato_frame="old_frame",
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
            "planner_id": "RRTConnectkConfigDefault",
        },
    )

    values = {parameter.name: parameter.value for parameter in received}
    assert node.tomato_frame == "detected_tomato_7_tf"
    assert values["planning_pipeline_id"] == "chomp"
    assert values["execute"] is False
    assert values["publish_display_trajectory"] is False


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


def test_harvest_command_builds_plan_only_command():
    command = harvest_command(
        3,
        False,
        planning_pipeline_id="chomp",
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
    assert "planner_id:=RRTConnectkConfigDefault" in command
    assert "publish_display_trajectory:=true" in command


def test_harvest_command_builds_execute_command():
    command = harvest_command(7, True, python_executable="python3")

    assert "tomato_frame:=detected_tomato_7_tf" in command
    assert "execute:=true" in command
    assert "planning_pipeline_id:=ompl" in command
    assert "planner_id:=RRTConnectkConfigDefault" in command


def test_harvest_command_can_disable_trajectory_display_for_automatic_test():
    command = harvest_command(0, False, publish_display_trajectory=False)

    assert "publish_display_trajectory:=false" in command


def test_harvest_command_builds_pilz_lin_command():
    command = harvest_command(
        3,
        False,
        planning_pipeline_id="pilz_industrial_motion_planner",
        planner_id="LIN",
    )

    assert "planning_pipeline_id:=pilz_industrial_motion_planner" in command
    assert "planner_id:=LIN" in command


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


def test_failure_marker_is_yellow():
    marker = harvest_result_marker(0, False, 0.04)

    assert (marker.color.r, marker.color.g, marker.color.b, marker.color.a) == (
        1.0,
        1.0,
        0.0,
        1.0,
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
        _set_harvest_result=lambda index, success: events.append(
            ("marker", index, success)
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
        verification=(4, 0, "ompl", "RRTConnectkConfigDefault"),
    )

    assert gui.batch_skipped == 1
    assert list(gui.batch_jobs) == [(1, False), (1, True)]
    assert ("marker", 0, False) in events
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
        _set_harvest_result=lambda index, success: events.append(
            ("marker", index, success)
        ),
        _finish_batch=lambda success, message: events.append(
            ("finish", success, message)
        ),
    )

    HarvestGui._handle_batch_job_done(
        gui,
        return_code=1,
        execute=True,
        verification=(4, 0, "ompl", "RRTConnectkConfigDefault"),
    )

    assert ("marker", 0, False) in events
    assert any(event[0:2] == ("finish", False) for event in events)


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
