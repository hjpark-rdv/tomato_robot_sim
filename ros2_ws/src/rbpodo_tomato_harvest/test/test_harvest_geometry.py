import numpy as np

from rbpodo_tomato_harvest.harvest_planner import make_harvest_geometry


def _position(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z])


def _rotation(pose):
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def test_tomato_is_between_tip_and_vine_and_tip_is_lower():
    tomato = np.array([0.45, -0.18, 0.34])
    result = make_harvest_geometry(
        tomato_position=tomato,
        vine_origin=[0.42, -0.14, 0.35],
        vine_axis=[0.0, 0.0, 1.0],
        tip_standoff=0.035,
        tip_below_center=0.010,
        preapproach_clearance=0.080,
    )

    tip = _position(result.target_pose)
    preapproach = _position(result.preapproach_pose)
    assert tip[2] == tomato[2] - 0.010
    assert np.dot(tomato - tip, result.vine_point - tomato) > 0.0
    assert np.dot(preapproach - tip, result.outward_axis) > 0.0


def test_gripper_longitudinal_axis_and_roll_are_level():
    result = make_harvest_geometry(
        tomato_position=[0.45, -0.18, 0.34],
        vine_origin=[0.42, -0.14, 0.35],
        vine_axis=[0.01, -0.01, 1.0],
        tip_standoff=0.035,
        tip_below_center=0.010,
        preapproach_clearance=0.080,
        outward_hint=[-1.0, -0.08, 0.02],
    )

    assert abs(result.outward_axis[2]) < 1e-12
    rotation = _rotation(result.target_pose)
    assert np.allclose(rotation[:, 0], result.outward_axis)
    assert np.allclose(rotation[:, 1], [0.0, 0.0, -1.0])
    assert abs(rotation[2, 2]) < 1e-12
