from types import SimpleNamespace

import pytest
from std_msgs.msg import Float64

from rbpodo_tomato_harvest.lift_joint_state_publisher import (
    LiftJointStatePublisher,
    height_mm_to_joint_m,
)


def test_height_mm_to_joint_m_converts_and_clamps_to_urdf_limits():
    assert height_mm_to_joint_m(350.0) == pytest.approx(0.35)
    assert height_mm_to_joint_m(-5.0) == pytest.approx(0.0)
    assert height_mm_to_joint_m(1400.0) == pytest.approx(0.75)


def test_height_callback_updates_prismatic_joint_position():
    warnings = []
    bridge = SimpleNamespace(
        minimum_height_m=0.0,
        maximum_height_m=0.75,
        current_height_m=0.0,
        get_logger=lambda: SimpleNamespace(
            error=lambda message: None,
            warning=lambda message: warnings.append(message),
        ),
    )

    LiftJointStatePublisher._height_callback(
        bridge,
        Float64(data=425.0),
    )

    assert bridge.current_height_m == pytest.approx(0.425)
    assert warnings == []


def test_height_callback_reports_when_value_exceeds_lift_range():
    warnings = []
    bridge = SimpleNamespace(
        minimum_height_m=0.0,
        maximum_height_m=0.75,
        current_height_m=0.0,
        get_logger=lambda: SimpleNamespace(
            error=lambda message: None,
            warning=lambda message: warnings.append(message),
        ),
    )

    LiftJointStatePublisher._height_callback(
        bridge,
        Float64(data=900.0),
    )

    assert bridge.current_height_m == pytest.approx(0.75)
    assert "clamped" in warnings[0]


def test_simulation_command_updates_joint_without_physical_lift():
    logs = []
    bridge = SimpleNamespace(
        simulate_commands=True,
        minimum_height_m=0.0,
        maximum_height_m=0.75,
        current_height_m=0.0,
        get_logger=lambda: SimpleNamespace(
            error=lambda message: None,
            warning=lambda message: None,
            info=lambda message: logs.append(message),
        ),
    )
    bridge._height_callback = lambda message: (
        LiftJointStatePublisher._height_callback(bridge, message)
    )

    LiftJointStatePublisher._simulation_height_command_callback(
        bridge,
        Float64(data=275.0),
    )

    assert bridge.current_height_m == pytest.approx(0.275)
    assert "275.00 mm" in logs[0]
