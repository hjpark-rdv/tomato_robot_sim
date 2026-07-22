import pytest

from rcl_interfaces.msg import ParameterType

from rbpodo_tomato_harvest.harvest_gui import harvest_command, scene_parameters


def test_harvest_command_builds_plan_only_command():
    command = harvest_command(3, False, python_executable="/usr/bin/python3")

    assert command[0:3] == [
        "/usr/bin/python3",
        "-m",
        "rbpodo_tomato_harvest.tomato_harvest_test",
    ]
    assert "tomato_frame:=detected_tomato_3_tf" in command
    assert "execute:=false" in command


def test_harvest_command_builds_execute_command():
    command = harvest_command(7, True, python_executable="python3")

    assert "tomato_frame:=detected_tomato_7_tf" in command
    assert "execute:=true" in command


def test_harvest_command_rejects_negative_index():
    with pytest.raises(ValueError):
        harvest_command(-1, False)


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
