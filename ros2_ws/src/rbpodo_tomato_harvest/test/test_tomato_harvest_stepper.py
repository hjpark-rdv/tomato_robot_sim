from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from rbpodo_tomato_harvest.tomato_harvest_stepper import (
    reverse_robot_trajectory,
    reverse_trajectory_group,
    step_stage_specs,
)


def _trajectory(name):
    return SimpleNamespace(name=name)


def test_step_stage_specs_exposes_complete_harvest_sequence():
    plan = SimpleNamespace(
        pick_ready_trajectory=_trajectory("ready"),
        preapproach_trajectory=_trajectory("preapproach"),
        step_approach_trajectories=tuple(
            (_trajectory(f"approach_{index}"),) for index in range(5)
        ),
        after_wait_trajectory=_trajectory("after_wait"),
        return_pick_ready_trajectory=_trajectory("return_ready"),
    )

    stages = step_stage_specs(plan, 2.25)

    assert [stage["key"] for stage in stages] == [
        "MOVE_TO_READY",
        "READY_TO_PREAPPROACH",
        "PREAPPROACH_TO_TARGET",
        "FORWARD_X70",
        "LIFT_Z40",
        "BACK_X50_FIRST",
        "LIFT_Z10",
        "BACK_X10_SECOND",
        "HARVEST_WAIT",
        "RETURN_READY",
    ]
    assert stages[7]["trajectories"][0].name == "after_wait"
    assert stages[8]["kind"] == "wait"
    assert stages[8]["wait_seconds"] == pytest.approx(2.25)
    assert stages[9]["trajectories"][0].name == "return_ready"


def test_step_stage_specs_requires_five_detailed_approach_groups():
    plan = SimpleNamespace(step_approach_trajectories=())

    with pytest.raises(ValueError, match="five detailed approach groups"):
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
