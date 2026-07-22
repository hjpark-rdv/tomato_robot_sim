import numpy as np
from geometry_msgs.msg import Pose

from rbpodo_tomato_harvest.harvest_planner import (
    make_harvest_geometry,
    make_tip_local_harvest_motion,
    planning_pose_from_tip_pose,
    quaternion_from_rotation,
    stemward_and_outward_from_tomato_rotation,
)


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


def test_tip_fixed_rotation_is_composed_after_level_gripper_rotation():
    tip_from_gripper = np.array(
        [
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, -1.0, 0.0],
        ]
    )
    result = make_harvest_geometry(
        tomato_position=[0.45, -0.18, 0.34],
        vine_origin=[0.42, -0.14, 0.35],
        vine_axis=[0.0, 0.0, 1.0],
        tip_standoff=0.025,
        tip_below_center=0.018,
        preapproach_clearance=0.040,
        outward_hint=[-1.0, 0.0, 0.0],
        tip_rotation_from_gripper=tip_from_gripper,
    )

    desired_gripper_rotation = np.column_stack(
        (
            result.outward_axis,
            np.array([0.0, 0.0, -1.0]),
            np.cross(result.outward_axis, np.array([0.0, 0.0, -1.0])),
        )
    )
    assert np.allclose(
        _rotation(result.target_pose),
        desired_gripper_rotation @ tip_from_gripper,
    )


def test_detected_tomato_x_axis_defines_opposite_outward_approach():
    rotation = np.array(
        [
            [0.6, -0.8, 0.0],
            [0.8, 0.6, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    stemward, outward = stemward_and_outward_from_tomato_rotation(rotation)

    assert np.allclose(stemward, [0.6, 0.8, 0.0])
    assert np.allclose(outward, [-0.6, -0.8, 0.0])


def test_post_harvest_motion_uses_tip_local_x_and_z_axes():
    start = Pose()
    start.position.x = 1.0
    start.position.y = 2.0
    start.position.z = 3.0
    start.orientation.z = np.sqrt(0.5)
    start.orientation.w = np.sqrt(0.5)

    motion = make_tip_local_harvest_motion(start)
    positions = [
        _position(pose)
        for pose in (*motion.before_wait_waypoints, motion.after_wait_pose)
    ]

    assert np.allclose(positions[0], [1.0, 2.050, 3.0])
    assert np.allclose(positions[1], [1.0, 2.050, 3.020])
    assert np.allclose(positions[2], [1.0, 2.035, 3.020])
    assert np.allclose(positions[3], [1.0, 2.035, 3.030])
    assert np.allclose(positions[4], [1.0, 2.005, 3.030])


def test_tip_goal_is_converted_to_equivalent_planning_link_goal():
    base_to_planning_rotation = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    base_to_planning_translation = np.array([0.4, -0.2, 0.5])
    planning_to_tip_rotation = np.array(
        [
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, -1.0, 0.0],
        ]
    )
    planning_to_tip_translation = np.array([-0.2116, -0.00075, 0.0])

    expected_tip_rotation = base_to_planning_rotation @ planning_to_tip_rotation
    expected_tip_translation = (
        base_to_planning_translation
        + base_to_planning_rotation @ planning_to_tip_translation
    )
    tip_goal = Pose()
    tip_goal.position.x, tip_goal.position.y, tip_goal.position.z = (
        expected_tip_translation
    )
    (
        tip_goal.orientation.x,
        tip_goal.orientation.y,
        tip_goal.orientation.z,
        tip_goal.orientation.w,
    ) = quaternion_from_rotation(expected_tip_rotation)

    planning_goal = planning_pose_from_tip_pose(
        tip_goal,
        planning_to_tip_translation,
        planning_to_tip_rotation,
    )

    assert np.allclose(_position(planning_goal), base_to_planning_translation)
    assert np.allclose(_rotation(planning_goal), base_to_planning_rotation)


def test_planning_link_goal_accounts_for_rotated_tip_offset():
    tip_goal = Pose()
    tip_goal.position.x = 0.1
    tip_goal.position.y = 0.2
    tip_goal.position.z = 0.3
    tip_goal.orientation.z = np.sqrt(0.5)
    tip_goal.orientation.w = np.sqrt(0.5)

    planning_goal = planning_pose_from_tip_pose(
        tip_goal,
        planning_to_tip_translation=[0.2, 0.0, 0.0],
        planning_to_tip_rotation=np.eye(3),
    )

    assert np.allclose(_position(planning_goal), [0.1, 0.0, 0.3])
    assert np.allclose(_rotation(planning_goal), _rotation(tip_goal))
