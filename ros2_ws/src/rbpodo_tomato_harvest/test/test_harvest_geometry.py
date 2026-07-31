import numpy as np
import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from rbpodo_tomato_harvest.harvest_planner import (
    CartesianHarvestPlanner,
    adaptive_outward_toward_robot,
    format_joint_trajectory_ranges,
    load_srdf_group_state,
    make_continuous_arc_waypoints,
    make_centered_joint_path_constraints,
    make_harvest_geometry,
    make_tip_local_harvest_motion,
    planning_pose_from_tip_pose,
    quaternion_from_rotation,
    stemward_and_outward_from_tomato_rotation,
    summarize_joint_trajectory_ranges,
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


def test_continuous_arc_bows_outward_and_finishes_at_target():
    start = Pose()
    start.orientation.w = 1.0
    target = Pose()
    target.position.x = 0.30
    target.orientation.z = np.sin(np.pi / 4.0)
    target.orientation.w = np.cos(np.pi / 4.0)

    outward_axis = np.array([0.0, 1.0, 0.25])
    waypoints = make_continuous_arc_waypoints(
        start,
        target,
        outward_axis=outward_axis,
        minimum_clearance=0.12,
        maximum_clearance=0.25,
        waypoint_count=7,
    )

    assert len(waypoints) == 7
    assert np.allclose(outward_axis, [0.0, 1.0, 0.25])
    assert max(pose.position.y for pose in waypoints[:-1]) >= 0.12
    assert np.allclose(_position(waypoints[-1]), _position(target))
    assert np.allclose(
        [
            waypoints[-1].orientation.x,
            waypoints[-1].orientation.y,
            waypoints[-1].orientation.z,
            waypoints[-1].orientation.w,
        ],
        [
            target.orientation.x,
            target.orientation.y,
            target.orientation.z,
            target.orientation.w,
        ],
    )
    for pose in waypoints:
        quaternion = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        assert np.linalg.norm(quaternion) == pytest.approx(1.0)


def test_continuous_arc_does_not_add_vertical_dip_after_lift_move():
    start = Pose()
    start.position.x = 0.20
    start.position.y = -0.10
    start.position.z = 0.48
    start.orientation.w = 1.0
    target = Pose()
    target.position.x = 0.50
    target.position.y = 0.20
    target.position.z = 0.12
    target.orientation.w = 1.0

    waypoints = make_continuous_arc_waypoints(
        start,
        target,
        outward_axis=[1.0, 0.0, 0.0],
        minimum_clearance=0.12,
        maximum_clearance=0.25,
        waypoint_count=7,
    )

    expected_z = [
        start.position.z
        + (index / 7.0) * (target.position.z - start.position.z)
        for index in range(1, 8)
    ]
    assert [pose.position.z for pose in waypoints] == pytest.approx(expected_z)
    assert all(
        target.position.z <= pose.position.z <= start.position.z
        for pose in waypoints
    )
    assert max(pose.position.x for pose in waypoints[:-1]) > target.position.x


def test_load_srdf_group_state_uses_requested_joint_order(tmp_path):
    srdf = tmp_path / "robot.srdf"
    srdf.write_text(
        """
<robot name="test">
  <group_state name="PICK_READY" group="mainpulation">
    <joint name="wrist3" value="6.0"/>
    <joint name="base" value="1.0"/>
    <joint name="shoulder" value="2.0"/>
  </group_state>
</robot>
""".strip(),
        encoding="utf-8",
    )

    positions = load_srdf_group_state(
        srdf,
        "PICK_READY",
        "mainpulation",
        ["base", "shoulder", "wrist3"],
    )

    assert positions == [1.0, 2.0, 6.0]


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


def test_adaptive_grasp_keeps_minus_x_when_it_already_faces_robot():
    result = adaptive_outward_toward_robot(
        tomato_rotation=np.eye(3),
        tomato_position=[1.0, 0.0, 0.4],
        robot_position=[0.0, 0.0, 0.0],
    )

    assert np.allclose(result.outward_axis, [-1.0, 0.0, 0.0])
    assert np.isclose(result.applied_rotation_deg, 0.0)
    assert np.isclose(result.current_robot_error_deg, 0.0)
    assert np.isclose(result.selected_robot_error_deg, 0.0)


def test_adaptive_grasp_rotates_minus_x_45_degrees_toward_local_y():
    result = adaptive_outward_toward_robot(
        tomato_rotation=np.eye(3),
        tomato_position=[0.0, 0.0, 0.4],
        robot_position=[-1.0, 1.0, 0.0],
    )

    expected = np.array([-1.0, 1.0, 0.0]) / np.sqrt(2.0)
    assert np.allclose(result.outward_axis, expected)
    assert np.isclose(result.applied_rotation_deg, 45.0)
    assert np.isclose(result.current_robot_error_deg, 45.0)
    assert np.isclose(result.selected_robot_error_deg, 0.0)


def test_adaptive_grasp_clamps_rotation_toward_local_y_to_45_degrees():
    result = adaptive_outward_toward_robot(
        tomato_rotation=np.eye(3),
        tomato_position=[0.0, 0.0, 0.4],
        robot_position=[0.0, 1.0, 0.0],
    )

    expected = np.array([-1.0, 1.0, 0.0]) / np.sqrt(2.0)
    assert np.allclose(result.outward_axis, expected)
    assert np.isclose(result.applied_rotation_deg, 45.0)
    assert np.isclose(result.current_robot_error_deg, 90.0)
    assert np.isclose(result.selected_robot_error_deg, 45.0)


def test_adaptive_grasp_rotates_toward_negative_local_y():
    result = adaptive_outward_toward_robot(
        tomato_rotation=np.eye(3),
        tomato_position=[0.0, 0.0, 0.4],
        robot_position=[-1.0, -1.0, 0.0],
    )

    expected = np.array([-1.0, -1.0, 0.0]) / np.sqrt(2.0)
    assert np.allclose(result.outward_axis, expected)
    assert np.isclose(result.applied_rotation_deg, -45.0)
    assert np.isclose(result.current_robot_error_deg, 45.0)
    assert np.isclose(result.selected_robot_error_deg, 0.0)


def test_adaptive_grasp_ignores_small_robot_alignment_error():
    angle = np.deg2rad(5.0)
    result = adaptive_outward_toward_robot(
        tomato_rotation=np.eye(3),
        tomato_position=[0.0, 0.0, 0.4],
        robot_position=[-np.cos(angle), np.sin(angle), 0.0],
        deadband_deg=10.0,
    )

    assert np.allclose(result.outward_axis, [-1.0, 0.0, 0.0])
    assert np.isclose(result.applied_rotation_deg, 0.0)
    assert np.isclose(result.current_robot_error_deg, 5.0)


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

    assert np.allclose(positions[0], [1.0, 2.070, 3.0])
    assert np.allclose(positions[1], [1.0, 2.070, 3.040])
    assert np.allclose(positions[2], [1.0, 2.055, 3.040])
    assert np.allclose(positions[3], [1.0, 2.055, 3.050])
    assert np.allclose(positions[4], [1.0, 2.025, 3.050])


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


def test_ompl_joint_constraints_use_start_centers_and_exclude_wrist3():
    start_positions = {
        "base": 0.4,
        "shoulder": -0.3,
        "elbow": 0.2,
        "wrist1": -0.1,
        "wrist2": 0.5,
    }
    constraints = make_centered_joint_path_constraints(
        start_positions,
        np.deg2rad(120.0),
    )

    constrained_names = [
        item.joint_name for item in constraints.joint_constraints
    ]
    assert constrained_names == [
        "base",
        "shoulder",
        "elbow",
        "wrist1",
        "wrist2",
    ]
    assert "wrist3" not in constrained_names
    for item in constraints.joint_constraints:
        assert np.isclose(item.position, start_positions[item.joint_name])
        assert np.isclose(item.tolerance_above, np.deg2rad(120.0))
        assert np.isclose(item.tolerance_below, np.deg2rad(120.0))


def test_joint_trajectory_summary_uses_joint_names_across_segments():
    first = RobotTrajectory()
    first.joint_trajectory.joint_names = ["base", "wrist3"]
    first.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[-1.0, 2.0]),
        JointTrajectoryPoint(positions=[0.5, 3.0]),
    ]
    second = RobotTrajectory()
    second.joint_trajectory.joint_names = ["wrist3", "base"]
    second.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[-2.0, 1.0]),
    ]

    summary = summarize_joint_trajectory_ranges(
        [first, second],
        start_positions={"base": 0.25, "wrist3": -0.5},
    )

    assert [item["joint_name"] for item in summary] == ["base", "wrist3"]
    assert np.isclose(summary[0]["start_rad"], 0.25)
    assert np.isclose(summary[0]["min_rad"], -1.0)
    assert np.isclose(summary[0]["max_rad"], 1.0)
    assert summary[0]["sample_count"] == 3
    assert np.isclose(summary[1]["start_rad"], -0.5)
    assert np.isclose(summary[1]["min_rad"], -2.0)
    assert np.isclose(summary[1]["max_rad"], 3.0)
    assert summary[1]["sample_count"] == 3
    formatted = format_joint_trajectory_ranges(summary)
    assert "Joint" in formatted
    assert "Start" in formatted
    assert "base" in formatted
    assert "wrist3" in formatted
    assert "Points" in formatted


def test_failure_robot_state_uses_last_valid_trajectory_point():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ["base", "wrist3"]
    trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[0.1, 0.2]),
        JointTrajectoryPoint(positions=[0.3, 0.4]),
    ]
    planner = type("Planner", (), {"last_plan_report": {}})()

    CartesianHarvestPlanner._record_failure_robot_state(
        planner,
        "CARTESIAN_PREAPPROACH",
        trajectory,
    )

    assert planner.last_plan_report["failure_robot_state"] == {
        "stage": "CARTESIAN_PREAPPROACH",
        "joint_names": ["base", "wrist3"],
        "joint_positions": [0.3, 0.4],
        "trajectory_point_index": 1,
    }
