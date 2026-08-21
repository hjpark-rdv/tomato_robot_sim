import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from rbpodo_tomato_harvest.tomato_harvest_stepper import (
    _stage_metadata,
    _execute_linear_motor_action,
    _execute_servo_sequence,
    continuous_all_trajectory_stage_blocks,
    continuous_cartesian_stage_blocks,
    cycle_last_stage_index,
    merge_and_retime_robot_trajectories,
    reverse_robot_trajectory,
    reverse_trajectory_group,
    scale_robot_trajectory_speed,
    step_stage_specs,
)


def test_stage_metadata_keeps_planned_joint_boundaries_for_rviz_goal():
    metadata = _stage_metadata(
        [
            {
                "key": "READY_TO_PREAPPROACH",
                "label": "PICK_READY → PRE_APPROACH",
                "detail": "test",
                "kind": "trajectory",
                "trajectories": (_trajectory("preapproach"),),
                "speed_percent": 30.0,
                "expected_start": {"base": 0.1, "wrist3": -0.2},
                "expected_end": {"base": 0.3, "wrist3": 0.4},
            }
        ]
    )

    assert metadata[0]["expected_start"] == {
        "base": pytest.approx(0.1),
        "wrist3": pytest.approx(-0.2),
    }
    assert metadata[0]["expected_end"] == {
        "base": pytest.approx(0.3),
        "wrist3": pytest.approx(0.4),
    }


@pytest.mark.parametrize(
    ("requested", "expected"),
    [(None, 6), (0, 0), (2, 2), (6, 6)],
)
def test_cycle_last_stage_index_accepts_gui_range(requested, expected):
    command = {}
    if requested is not None:
        command["last_stage_index"] = requested

    assert cycle_last_stage_index(command, 10) == expected


@pytest.mark.parametrize("requested", [-1, 7, "invalid"])
def test_cycle_last_stage_index_rejects_out_of_range_value(requested):
    with pytest.raises((TypeError, ValueError)):
        cycle_last_stage_index({"last_stage_index": requested}, 10)


def _trajectory(name):
    return SimpleNamespace(name=name)


def test_step_stage_specs_exposes_complete_harvest_sequence():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(plan, 2.25)

    assert [stage["key"] for stage in stages] == [
        "MOVE_TO_READY",
        "READY_TO_PREAPPROACH",
        "PREAPPROACH_TO_TARGET",
        "FORWARD_X",
        "TCP_WRIST_OSCILLATION",
        "LIFT_Z20_FORWARD_X20",
        "LIFT_Z20_SECOND",
        "BACK_X50_FIRST",
        "BACK_X10_SECOND",
        "LINEAR_MOTOR_EXTEND",
        "SERVO_CLOSE_OPEN",
        "RETURN_READY",
    ]
    assert stages[8]["trajectories"][0].name == "after_wait"
    assert stages[6]["detail"] == (
        "tip 로컬 X +0.0 / Y +0.0 / Z +20.0 mm / "
        "6→7 곡선 Cartesian"
    )
    assert stages[5]["detail"] == (
        "tip 로컬 X +20.0 / Y +0.0 / Z +20.0 mm / "
        "5→6 곡선 Cartesian"
    )
    assert stages[9]["kind"] == "linear_motor"
    assert stages[9]["linear_motor_command"] == "extend"
    assert stages[9]["linear_motor_duration_seconds"] == pytest.approx(3.0)
    assert stages[10]["kind"] == "servo_sequence"
    assert stages[10]["close_angle_deg"] == pytest.approx(110.0)
    assert stages[10]["open_angle_deg"] == pytest.approx(159.0)
    assert stages[10]["dwell_seconds"] == pytest.approx(0.7)
    assert stages[10]["retract_duration_seconds"] == pytest.approx(3.0)
    assert stages[11]["trajectories"][0].name == "return_ready"


def test_step_stage_specs_displays_configured_forward_distance():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(plan, 2.0, forward_distance_m=0.035)

    assert stages[3]["detail"] == (
        "tip 로컬 X +35.0 / Y +0.0 / Z +0.0 mm / "
        "직선 Cartesian"
    )


def test_step_stage_specs_displays_enabled_forward_wave():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(plan, 2.0, forward_wave_enabled=True)

    assert "Z축 ±5mm × 3회 웨이브 Cartesian" in stages[3]["detail"]


def test_step_stage_specs_assigns_individual_motion_stage_speeds():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=(
            _trajectory("return_via_a"),
            _trajectory("return_ready"),
        ),
    )

    stages = step_stage_specs(
        plan,
        2.0,
        stage_speed_percents=(30, 40, 50, 60, 70, 80, 35),
    )

    assert [stages[index]["speed_percent"] for index in range(2, 8)] == [
        30,
        40,
        50,
        60,
        70,
        80,
    ]
    assert stages[11]["speed_percent"] == pytest.approx(35.0)
    assert stages[11]["trajectory_speed_percents"] == pytest.approx(
        (35.0, 100.0)
    )


def test_step_stage_specs_defaults_only_cartesian_stages_to_thirty_percent():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(plan, 2.0)

    assert [stages[index]["speed_percent"] for index in range(2, 8)] == [
        30,
        30,
        100,
        30,
        30,
        30,
    ]
    assert stages[11]["trajectory_speed_percents"] == pytest.approx(
        (100.0,)
    )


def test_step_stage_specs_applies_speed_only_to_a_to_preapproach_segment():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=(
            _trajectory("ready_to_a"),
            _trajectory("a_to_preapproach"),
        ),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(
        plan,
        2.0,
        preapproach_final_speed_percent=45.0,
    )

    assert stages[1]["trajectory_speed_percents"] == (100.0, 45.0)
    assert "A→PRE 속도 45%" in stages[1]["detail"]


def test_step_stage_specs_labels_single_segment_as_direct_preapproach():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=(_trajectory("direct_preapproach"),),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(plan, 2.0)

    assert stages[1]["trajectory_speed_percents"] == (100.0,)
    assert stages[1]["detail"] == "경유점 A 없이 PRE_APPROACH 직접 접근"


def test_step_stage_specs_uses_configured_servo_close_angle():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(
        plan,
        2.0,
        servo_close_angle_deg=112.0,
    )

    assert stages[10]["close_angle_deg"] == pytest.approx(112.0)
    assert "112° 닫기" in stages[10]["detail"]


def test_step_stage_specs_uses_configured_linear_motor_extend_time():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(
        plan,
        2.0,
        linear_motor_extend_seconds=8.0,
    )

    assert stages[9]["linear_motor_duration_seconds"] == pytest.approx(8.0)
    assert stages[9]["detail"] == "8초 늘림 → 자동 정지"


def test_step_stage_specs_labels_right_ready_for_right_capture():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(
        plan,
        2.0,
        ready_state_name="PICK_READY_RIGHT",
    )

    assert stages[0]["label"] == "현재 자세 → PICK_READY_RIGHT"
    assert stages[1]["label"] == "PICK_READY_RIGHT → PRE_APPROACH"
    assert stages[-1]["label"] == "현재 자세 → PICK_READY_RIGHT"


def test_step_stage_specs_keeps_disabled_wrist_stage_as_skip_slot():
    approach_groups = [
        (_trajectory(f"approach_{index}"),) for index in range(6)
    ]
    approach_groups[2] = ()
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(approach_groups),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(plan, 2.0)

    assert stages[4]["key"] == "TCP_WRIST_OSCILLATION"
    assert stages[4]["kind"] == "skip"
    assert stages[4]["detail"] == "사용 안 함 (체크 해제)"
    assert stages[4]["trajectories"] == ()


def test_step_stage_specs_exposes_cartesian_waypoints_for_merging():
    waypoint_groups = tuple(
        ((f"waypoint_{index}",) if index != 2 else ())
        for index in range(6)
    )
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (() if index == 2 else (_trajectory(f"approach_{index}"),))
            for index in range(6)
        ),
        step_approach_waypoints=waypoint_groups,
        after_wait_trajectory=_trajectory("after_wait"),
        after_wait_waypoints=("waypoint_after_wait",),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(plan, 2.0)

    assert stages[2]["cartesian_waypoints"] == ("waypoint_0",)
    assert stages[4]["cartesian_waypoints"] == ()
    assert stages[8]["cartesian_waypoints"] == ("waypoint_after_wait",)


def test_continuous_blocks_cross_disabled_wrist_but_not_ready_or_wait():
    stages = [
        {"kind": "trajectory", "cartesian_waypoints": ()},
        {"kind": "trajectory", "cartesian_waypoints": ()},
        {"kind": "trajectory", "cartesian_waypoints": ("a",)},
        {"kind": "trajectory", "cartesian_waypoints": ("b",)},
        {"kind": "skip", "cartesian_waypoints": ()},
        {"kind": "trajectory", "cartesian_waypoints": ("c", "d")},
        {"kind": "trajectory", "cartesian_waypoints": ("e",)},
        {"kind": "trajectory", "cartesian_waypoints": ("f",)},
        {"kind": "trajectory", "cartesian_waypoints": ("g",)},
        {"kind": "wait", "cartesian_waypoints": ()},
    ]

    blocks = continuous_cartesian_stage_blocks(stages, 0, 9)

    assert len(blocks) == 1
    assert blocks[0]["start_index"] == 2
    assert blocks[0]["end_index"] == 8
    assert blocks[0]["stage_indices"] == tuple(range(2, 9))
    assert blocks[0]["waypoints"] == ("a", "b", "c", "d", "e", "f", "g")


def test_continuous_block_uses_slowest_included_stage_speed():
    stages = [
        {
            "kind": "trajectory",
            "cartesian_waypoints": (name,),
            "speed_percent": speed,
        }
        for name, speed in (("a", 80.0), ("b", 35.0), ("c", 60.0))
    ]

    blocks = continuous_cartesian_stage_blocks(stages, 0, 2)

    assert blocks[0]["speed_percent"] == pytest.approx(35.0)


def test_all_trajectory_blocks_include_ompl_and_cartesian_until_actuator():
    stages = [
        {"kind": "trajectory", "trajectories": (_trajectory("ompl"),)},
        {"kind": "trajectory", "trajectories": (_trajectory("cart"),)},
        {"kind": "skip", "trajectories": ()},
        {"kind": "trajectory", "trajectories": (_trajectory("cart2"),)},
        {"kind": "linear_motor", "trajectories": ()},
        {"kind": "trajectory", "trajectories": (_trajectory("return"),)},
    ]

    blocks = continuous_all_trajectory_stage_blocks(stages, 0, 5)

    assert len(blocks) == 1
    assert blocks[0]["start_index"] == 0
    assert blocks[0]["end_index"] == 3
    assert blocks[0]["stage_indices"] == (0, 1, 2, 3)


def test_continuous_blocks_split_at_enabled_wrist_trajectory():
    stages = [
        {"kind": "trajectory", "cartesian_waypoints": ("a",)},
        {"kind": "trajectory", "cartesian_waypoints": ("b",)},
        {"kind": "trajectory", "cartesian_waypoints": ()},
        {"kind": "trajectory", "cartesian_waypoints": ("c",)},
        {"kind": "trajectory", "cartesian_waypoints": ("d",)},
    ]

    blocks = continuous_cartesian_stage_blocks(stages, 0, 4)

    assert [(block["start_index"], block["end_index"]) for block in blocks] == [
        (0, 1),
        (3, 4),
    ]


def test_execute_servo_sequence_closes_waits_opens_and_waits(monkeypatch):
    published = []
    events = []
    planner = SimpleNamespace(
        count_subscribers=lambda _topic: 1,
        get_logger=lambda: SimpleNamespace(
            info=lambda _message: None,
            error=lambda _message: None,
        ),
    )
    stage = {
        "servo_publisher": SimpleNamespace(
            publish=lambda message: published.append(list(message.data))
        ),
        "servo_topic": "/linear_motor/servo10_command",
        "close_angle_deg": 90.0,
        "open_angle_deg": 170.0,
        "speed_percent": 50.0,
        "dwell_seconds": 0.7,
        "retract_duration_seconds": 3.0,
    }
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.tomato_harvest_stepper.time.sleep",
        lambda seconds: events.append(("sleep", seconds)),
    )
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.tomato_harvest_stepper._execute_linear_motor_action",
        lambda _planner, _stage, **kwargs: (
            events.append(("linear_motor", kwargs)) or True
        ),
    )

    stage["servo_publisher"] = SimpleNamespace(
        publish=lambda message: (
            published.append(list(message.data)),
            events.append(("servo", list(message.data))),
        )
    )

    assert _execute_servo_sequence(planner, stage) is True
    assert published == [[90.0, 50.0], [170.0, 50.0]]
    assert events == [
        ("servo", [90.0, 50.0]),
        ("sleep", 0.7),
        ("servo", [170.0, 50.0]),
        ("sleep", 0.7),
        (
            "linear_motor",
            {
                "command": "retract",
                "duration_seconds": 3.0,
                "wait_for_completion": False,
            },
        ),
    ]


def test_execute_linear_motor_action_extends_then_stops(monkeypatch):
    pin8_published = []
    pin9_published = []
    sleeps = []
    planner = SimpleNamespace(
        count_subscribers=lambda _topic: 1,
        get_logger=lambda: SimpleNamespace(
            info=lambda _message: None,
            error=lambda _message: None,
        ),
    )
    stage = {
        "linear_motor_pin8_publisher": SimpleNamespace(
            publish=lambda message: pin8_published.append(message.data)
        ),
        "linear_motor_pin9_publisher": SimpleNamespace(
            publish=lambda message: pin9_published.append(message.data)
        ),
        "linear_motor_command": "extend",
        "linear_motor_duration_seconds": 3.0,
    }
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.tomato_harvest_stepper.time.sleep",
        sleeps.append,
    )

    assert _execute_linear_motor_action(planner, stage) is True
    assert pin8_published == [False, True, False]
    assert pin9_published == [False, False, False]
    assert sleeps == [0.1, 3.0]


def test_execute_linear_motor_action_can_stop_in_background(monkeypatch):
    pin8_published = []
    pin9_published = []
    sleeps = []
    created_timers = []

    class FakeTimer:
        def __init__(self, interval, function):
            self.interval = interval
            self.function = function
            self.daemon = None
            self.started = False
            created_timers.append(self)

        def start(self):
            self.started = True

    planner = SimpleNamespace(
        count_subscribers=lambda _topic: 1,
        get_logger=lambda: SimpleNamespace(
            info=lambda _message: None,
            error=lambda _message: None,
        ),
    )
    stage = {
        "linear_motor_pin8_publisher": SimpleNamespace(
            publish=lambda message: pin8_published.append(message.data)
        ),
        "linear_motor_pin9_publisher": SimpleNamespace(
            publish=lambda message: pin9_published.append(message.data)
        ),
    }
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.tomato_harvest_stepper.time.sleep",
        sleeps.append,
    )
    monkeypatch.setattr(
        "rbpodo_tomato_harvest.tomato_harvest_stepper.threading.Timer",
        FakeTimer,
    )

    assert _execute_linear_motor_action(
        planner,
        stage,
        command="retract",
        duration_seconds=3.0,
        wait_for_completion=False,
    ) is True
    assert pin8_published == [False, False]
    assert pin9_published == [False, True]
    assert sleeps == [0.1]
    assert len(created_timers) == 1
    assert created_timers[0].interval == pytest.approx(3.0)
    assert created_timers[0].daemon is False
    assert created_timers[0].started is True

    created_timers[0].function()
    assert pin8_published == [False, False, False]
    assert pin9_published == [False, True, False]


def test_step_stage_specs_displays_custom_xyz_for_stages_three_to_seven():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(6)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )
    deltas = (
        (0.011, 0.002, -0.003),
        (0.041, 0.004, 0.005),
        (0.021, -0.006, 0.022),
        (0.007, 0.008, 0.023),
        (-0.051, 0.009, -0.010),
    )

    stages = step_stage_specs(
        plan,
        2.0,
        custom_stage_deltas_m=deltas,
    )

    assert stages[2]["detail"] == (
        "tip 로컬 X +11.0 / Y +2.0 / Z -3.0 mm"
    )
    assert stages[7]["detail"] == (
        "tip 로컬 X -51.0 / Y +9.0 / Z -10.0 mm"
    )


def test_step_stage_specs_requires_six_detailed_approach_groups():
    plan = SimpleNamespace(step_approach_trajectories=())

    with pytest.raises(ValueError, match="six detailed approach groups"):
        step_stage_specs(plan, 2.0)


def _robot_trajectory(name, positions):
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.header.frame_id = name
    trajectory.joint_trajectory.joint_names = ["joint0"]
    trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(
            positions=[position],
            velocities=[velocity],
            accelerations=[acceleration],
            time_from_start=Duration(sec=second),
        )
        for second, position, velocity, acceleration in positions
    ]
    return trajectory


def test_reverse_robot_trajectory_reverses_points_time_and_velocity():
    trajectory = _robot_trajectory(
        "forward",
        [
            (1, 1.0, 0.0, 0.1),
            (3, 2.0, 0.4, 0.2),
            (6, 3.0, 0.0, 0.3),
        ],
    )

    reversed_trajectory = reverse_robot_trajectory(trajectory)
    points = reversed_trajectory.joint_trajectory.points

    assert [point.positions[0] for point in points] == [3.0, 2.0, 1.0]
    assert [point.time_from_start.sec for point in points] == [1, 4, 6]
    assert [point.velocities[0] for point in points] == [0.0, -0.4, 0.0]
    assert [point.accelerations[0] for point in points] == [0.3, 0.2, 0.1]
    assert list(trajectory.joint_trajectory.points[0].positions) == [1.0]


def test_reverse_trajectory_group_reverses_segment_order():
    first = _robot_trajectory("first", [(0, 0.0, 0.0, 0.0)])
    second = _robot_trajectory("second", [(0, 1.0, 0.0, 0.0)])

    reversed_group = reverse_trajectory_group((first, second))

    assert [
        item.joint_trajectory.header.frame_id for item in reversed_group
    ] == ["second", "first"]


def test_scale_robot_trajectory_speed_retimes_velocity_and_acceleration():
    trajectory = _robot_trajectory(
        "stage",
        [
            (0, 1.0, 0.0, 0.0),
            (2, 2.0, 0.8, 0.4),
        ],
    )

    scaled = scale_robot_trajectory_speed(trajectory, 50.0)
    points = scaled.joint_trajectory.points

    assert points[1].time_from_start.sec == 4
    assert points[1].velocities[0] == pytest.approx(0.4)
    assert points[1].accelerations[0] == pytest.approx(0.1)
    assert trajectory.joint_trajectory.points[1].time_from_start.sec == 2


def test_merge_and_retime_removes_boundary_stop_between_two_paths():
    first = _robot_trajectory(
        "ompl",
        [
            (0, 0.0, 0.0, 0.0),
            (1, 0.4, 0.0, 0.0),
        ],
    )
    second = _robot_trajectory(
        "cartesian",
        [
            (0, 0.4, 0.0, 0.0),
            (1, 0.8, 0.0, 0.0),
        ],
    )

    merged = merge_and_retime_robot_trajectories((first, second))
    points = merged.joint_trajectory.points

    assert [point.positions[0] for point in points] == pytest.approx(
        [0.0, 0.4, 0.8]
    )
    assert points[1].velocities[0] > 0.0
    assert points[0].velocities[0] == pytest.approx(0.0)
    assert points[-1].velocities[0] == pytest.approx(0.0)
    times = [
        point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        for point in points
    ]
    assert times[0] == pytest.approx(0.0)
    assert times[0] < times[1] < times[2]


def test_merge_and_retime_uses_requested_speed_not_old_source_duration():
    trajectory = _robot_trajectory(
        "slow_source",
        [
            (0, 0.0, 0.0, 0.0),
            (100, 1.0, 0.0, 0.0),
        ],
    )

    merged = merge_and_retime_robot_trajectories(
        (trajectory,),
        maximum_acceleration=1000.0,
        speed_percents=(100.0,),
    )
    final = merged.joint_trajectory.points[-1].time_from_start
    duration = final.sec + final.nanosec / 1e9

    assert duration < 2.0


def test_merge_and_retime_lower_speed_percent_increases_duration():
    trajectory = _robot_trajectory(
        "path",
        [
            (0, 0.0, 0.0, 0.0),
            (1, 1.0, 0.0, 0.0),
            (2, 2.0, 0.0, 0.0),
        ],
    )

    full_speed = merge_and_retime_robot_trajectories(
        (trajectory,),
        maximum_acceleration=1000.0,
        speed_percents=(100.0,),
    )
    half_speed = merge_and_retime_robot_trajectories(
        (trajectory,),
        maximum_acceleration=1000.0,
        speed_percents=(50.0,),
    )

    def duration(result):
        final = result.joint_trajectory.points[-1].time_from_start
        return final.sec + final.nanosec / 1e9

    assert duration(half_speed) > duration(full_speed)


def test_merge_and_retime_duration_does_not_grow_with_sampling_density():
    def sampled_path(point_count):
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ["joint0", "joint1"]
        for index in range(point_count):
            ratio = index / (point_count - 1)
            trajectory.joint_trajectory.points.append(
                JointTrajectoryPoint(
                    positions=[
                        ratio,
                        0.3 * math.sin(math.pi * ratio),
                    ]
                )
            )
        return trajectory

    sparse = merge_and_retime_robot_trajectories(
        (sampled_path(50),),
        maximum_acceleration=2.4,
        speed_percents=(100.0,),
    )
    dense = merge_and_retime_robot_trajectories(
        (sampled_path(500),),
        maximum_acceleration=2.4,
        speed_percents=(100.0,),
    )

    def duration(result):
        final = result.joint_trajectory.points[-1].time_from_start
        return final.sec + final.nanosec / 1e9

    assert duration(dense) == pytest.approx(duration(sparse), rel=0.05)
