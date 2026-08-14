from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from rbpodo_tomato_harvest.tomato_harvest_stepper import (
    continuous_cartesian_stage_blocks,
    cycle_last_stage_index,
    reverse_robot_trajectory,
    reverse_trajectory_group,
    step_stage_specs,
)


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
        "HARVEST_WAIT",
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
    assert stages[9]["kind"] == "wait"
    assert stages[9]["wait_seconds"] == pytest.approx(2.25)
    assert stages[10]["trajectories"][0].name == "return_ready"


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
        "tip 로컬 X +35.0 / Y +0.0 / Z +0.0 mm"
    )


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
