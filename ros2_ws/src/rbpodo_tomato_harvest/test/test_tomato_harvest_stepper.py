from types import SimpleNamespace

import pytest

from rbpodo_tomato_harvest.tomato_harvest_stepper import step_stage_specs


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
        "BACK_X15",
        "LIFT_Z10",
        "HARVEST_WAIT",
        "BACK_X30",
        "RETURN_READY",
    ]
    assert stages[7]["kind"] == "wait"
    assert stages[7]["wait_seconds"] == pytest.approx(2.25)
    assert stages[9]["trajectories"][0].name == "return_ready"


def test_step_stage_specs_requires_five_detailed_approach_groups():
    plan = SimpleNamespace(step_approach_trajectories=())

    with pytest.raises(ValueError, match="five detailed approach groups"):
        step_stage_specs(plan, 2.0)
