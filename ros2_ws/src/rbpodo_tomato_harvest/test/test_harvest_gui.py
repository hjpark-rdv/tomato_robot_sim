from collections import deque
import random
from types import SimpleNamespace

import pytest

from geometry_msgs.msg import Point, Pose, TransformStamped
from moveit_msgs.msg import RobotState, RobotTrajectory
from rcl_interfaces.msg import ParameterType
from trajectory_msgs.msg import JointTrajectoryPoint

from rbpodo_tomato_harvest.harvest_gui import (
    GUI_PLANNER_CONFIG,
    HarvestGui,
    PLANNER_CONFIGS,
    adaptive_approach_axis_local,
    adaptive_rotation_degrees,
    adaptive_rotation_was_applied,
    cartesian_fallback_summary,
    cancel_all_goals_request,
    concise_plan_report,
    generate_sweep_cases,
    harvest_all_jobs,
    harvest_command,
    harvest_result_marker,
    harvest_statistics_record,
    is_critical_process_output,
    scene_parameters,
    sweep_execution_duration_text,
    sweep_stage_detail,
    sweep_stage_summary,
    sweep_result_marker,
    tomato_stem_arrow_length,
)
from rbpodo_tomato_harvest.harvest_planner import (
    CartesianHarvestPlanner,
    HarvestMotionPlan,
)
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
        (0.0, 0.0, 0.0, 5.0),
        (0.0, 0.0, 0.0, 10.0),
        (0.01, 0.01, 0.0, 0.0),
        (0.01, 0.01, 0.0, 5.0),
        (0.01, 0.01, 0.0, 10.0),
    ]


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

    assert "Plan 실패: detected_tomato_1_tf | 보정=45.0°" in summary
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
            "planner_id": "RRTConnect",
            "preapproach_mode": "planner",
            "velocity_scale": 0.35,
            "acceleration_scale": 0.25,
            "harvest_wait_sec": 4.25,
            "continuous_transition": True,
            "return_to_pick_ready": False,
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


def test_persistent_worker_enables_execution_only_when_requested():
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


def test_continuous_execute_skips_pick_ready_and_keeps_post_wait(monkeypatch):
    events = []
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.harvest_planner.time.sleep",
        lambda seconds: None,
    )
    planner = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(
            value={"execute": True, "harvest_wait_sec": 0.0}[name]
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
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warning=lambda message: None,
        ),
        _lookup_transform=lambda frame: transform,
    )

    def plan_arc(waypoints, start_state, label, pregrasp):
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
    assert result[1] == ("arc_preapproach_trajectory",)
    assert result[2].is_diff is True
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


def test_continuous_arc_failure_falls_back_through_pick_ready():
    preapproach_state = RobotState()
    preapproach_state.joint_state.name = ["base", "shoulder"]
    preapproach_state.joint_state.position = [0.4, -0.8]
    display_start = RobotState()
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
            }[name]
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warning=lambda message: None,
        ),
        _plan_preapproach=lambda pose, state: (
            planner.last_plan_report["stages"].append(
                {"stage": "CARTESIAN_PREAPPROACH", "success": True}
            )
            or ("seed_preapproach",)
        ),
        _plan_pick_ready=lambda: ("pick_ready", display_start),
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
    )

    assert result == ("pick_ready", ("seed_preapproach",), display_start)
    assert planner.last_plan_report["continuous_transition_direct"] is False
    assert planner.last_plan_report["continuous_transition_arc"] is False
    assert planner.last_plan_report["recovery_success"] is True
    arc_stage = next(
        stage
        for stage in planner.last_plan_report["stages"]
        if stage["stage"] == "CARTESIAN_CONTINUOUS_ARC"
    )
    assert arc_stage["discarded"] is True


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
        continuous_transition=True,
        return_to_pick_ready=False,
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
    assert "continuous_transition:=true" in command
    assert "return_to_pick_ready:=false" in command


def test_harvest_command_rejects_invalid_motion_scale():
    with pytest.raises(ValueError, match="velocity_scale"):
        harvest_command(0, False, velocity_scale=1.01)
    with pytest.raises(ValueError, match="acceleration_scale"):
        harvest_command(0, False, acceleration_scale=0.0)
    with pytest.raises(ValueError, match="harvest_wait_sec"):
        harvest_command(0, False, harvest_wait_sec=-0.1)


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


def test_harvest_command_can_disable_trajectory_display_for_automatic_test():
    command = harvest_command(0, False, publish_display_trajectory=False)

    assert "publish_display_trajectory:=false" in command


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
        _plan_pose_target=lambda pose, start_state, label: (
            calls.append(("planner", pose, start_state, label))
            or "pipeline_trajectory"
        ),
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

    assert marker.points[1].x == pytest.approx(0.04)
    assert marker.points[1].y == pytest.approx(0.0)
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
    assert marker.points[1].x == pytest.approx(component)
    assert marker.points[1].y == pytest.approx(-component)


def test_failure_marker_points_toward_plus_y_after_negative_rotation():
    marker = harvest_result_marker(
        0,
        False,
        0.04,
        adaptive_rotation_deg=-45.0,
    )

    component = 0.04 / (2.0 ** 0.5)
    assert marker.points[1].x == pytest.approx(component)
    assert marker.points[1].y == pytest.approx(component)


def test_failure_marker_uses_exact_planner_approach_axis():
    marker = harvest_result_marker(
        0,
        False,
        0.04,
        adaptive_rotation_deg=0.0,
        approach_axis_local=(0.6, -0.8),
    )

    assert marker.points[1].x == pytest.approx(0.024)
    assert marker.points[1].y == pytest.approx(-0.032)


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
            "adaptive_grasp": {"applied_rotation_deg": 30.0}
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
    assert any(event[0:2] == ("finish", False) for event in events)


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
