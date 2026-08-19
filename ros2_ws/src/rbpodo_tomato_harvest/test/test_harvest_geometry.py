import numpy as np
import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import RobotState
from trajectory_msgs.msg import JointTrajectoryPoint

from rbpodo_tomato_harvest.harvest_planner import (
    ApproachRotationEvaluation,
    CartesianHarvestPlanner,
    adaptive_outward_toward_robot,
    deadline_safe_rotation_guard,
    format_joint_trajectory_ranges,
    load_srdf_group_state,
    make_continuous_arc_waypoints,
    make_centered_joint_path_constraints,
    make_harvest_geometry,
    make_preapproach_via_pose,
    make_tip_local_harvest_motion,
    make_tip_local_sine_wave_waypoints,
    make_tip_local_transition_curve_waypoints,
    make_wrist3_oscillation_trajectory,
    outward_from_tomato_rotation,
    planning_pose_from_tip_pose,
    quaternion_from_rotation,
    is_within_robot_side_approach_sector,
    joint_span_violations,
    robot_side_approach_error_deg,
    select_minimum_feasible_rotation,
    select_robotward_feasible_rotation,
    stemward_and_outward_from_tomato_rotation,
    summarize_joint_trajectory_ranges,
    translated_pose_in_local_frame,
    trajectory_joint_safety_violations,
)


def test_wrist3_oscillation_keeps_other_joints_fixed_and_returns_to_start():
    state = RobotState()
    state.joint_state.name = [
        "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"
    ]
    state.joint_state.position = [0.1, 0.2, -0.3, 0.4, -0.5, 1.0]

    trajectory = make_wrist3_oscillation_trajectory(
        state,
        10.0,
        maximum_step_deg=2.0,
        speed_deg_sec=20.0,
    )
    points = trajectory.joint_trajectory.points
    wrist_values = [point.positions[5] for point in points]

    assert points[0].positions == pytest.approx(state.joint_state.position)
    assert points[0].time_from_start.sec == 0
    assert points[0].time_from_start.nanosec == 0
    assert min(wrist_values) == pytest.approx(1.0 - np.deg2rad(10.0))
    assert max(wrist_values) == pytest.approx(1.0 + np.deg2rad(10.0))
    assert wrist_values[-1] == pytest.approx(1.0)
    assert all(point.positions[:5] == pytest.approx([0.1, 0.2, -0.3, 0.4, -0.5]) for point in points)
    assert len(points) == 21


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


def test_tip_local_second_lift_moves_only_along_tip_z():
    start = Pose()
    start.orientation.w = 1.0

    motion = make_tip_local_harvest_motion(start)
    before_wait = [_position(pose) for pose in motion.before_wait_waypoints]

    assert np.allclose(
        before_wait,
        [
            [0.040, 0.0, 0.0],
            [0.060, 0.0, 0.020],
            [0.060, 0.0, 0.040],
            [0.010, 0.0, 0.040],
        ],
    )
    assert _position(motion.after_wait_pose) == pytest.approx(
        [0.0, 0.0, 0.040]
    )


def test_custom_tip_local_stage_motion_supports_xyz_on_every_stage():
    start = Pose()
    start.orientation.w = 1.0
    target = translated_pose_in_local_frame(start, (0.010, 0.002, -0.003))

    motion = make_tip_local_harvest_motion(
        target,
        custom_stage_deltas=(
            (0.040, 0.004, 0.005),
            (0.020, -0.006, 0.020),
            (0.007, 0.008, 0.020),
            (-0.050, 0.009, -0.010),
        ),
    )

    assert np.allclose(_position(target), [0.010, 0.002, -0.003])
    assert np.allclose(
        [_position(pose) for pose in motion.before_wait_waypoints],
        [
            [0.050, 0.006, 0.002],
            [0.070, 0.000, 0.022],
            [0.077, 0.008, 0.042],
            [0.027, 0.017, 0.032],
        ],
    )


def test_second_lift_curve_preserves_endpoints_and_bows_in_local_x():
    start = Pose()
    start.orientation.w = 1.0
    target = Pose()
    target.position.z = 0.020
    target.orientation.w = 1.0

    waypoints = make_tip_local_transition_curve_waypoints(
        start,
        target,
        incoming_delta_xyz=(0.020, 0.0, 0.020),
        waypoint_count=7,
        control_ratio=0.5,
    )

    assert len(waypoints) == 7
    assert max(pose.position.x for pose in waypoints[:-1]) > 0.003
    assert all(pose.position.z >= 0.0 for pose in waypoints)
    assert all(
        earlier.position.z <= later.position.z
        for earlier, later in zip(waypoints, waypoints[1:])
    )
    assert _position(waypoints[-1]) == pytest.approx([0.0, 0.0, 0.020])


def test_forward_sine_wave_moves_in_local_x_and_returns_to_target_height():
    start = Pose()
    start.orientation.w = 1.0
    target = Pose()
    target.position.x = 0.040
    target.orientation.w = 1.0

    waypoints = make_tip_local_sine_wave_waypoints(start, target)
    positions = np.asarray([_position(pose) for pose in waypoints])

    assert len(waypoints) == 25
    assert np.all(np.diff(positions[:, 0]) > 0.0)
    assert np.max(positions[:, 2]) == pytest.approx(0.005)
    assert np.min(positions[:, 2]) == pytest.approx(-0.005)
    assert _position(waypoints[-1]) == pytest.approx([0.040, 0.0, 0.0])


def test_forward_sine_wave_uses_tip_local_z_for_rotated_pose():
    start = Pose()
    start.orientation.x = np.sin(np.pi / 4.0)
    start.orientation.w = np.cos(np.pi / 4.0)
    target = translated_pose_in_local_frame(start, (0.040, 0.0, 0.0))

    waypoints = make_tip_local_sine_wave_waypoints(start, target)
    positions = np.asarray([_position(pose) for pose in waypoints])

    # A +90 degree local-X rotation maps local Z onto world -Y.
    assert np.max(positions[:, 1]) == pytest.approx(0.005)
    assert np.min(positions[:, 1]) == pytest.approx(-0.005)
    assert _position(waypoints[-1]) == pytest.approx(_position(target))


def test_second_lift_curve_uses_tip_local_axes_for_rotated_pose():
    start = Pose()
    start.orientation.z = np.sin(np.pi / 4.0)
    start.orientation.w = np.cos(np.pi / 4.0)
    target = translated_pose_in_local_frame(start, (0.0, 0.0, 0.020))

    waypoints = make_tip_local_transition_curve_waypoints(
        start,
        target,
        incoming_delta_xyz=(0.020, 0.0, 0.020),
    )

    # Local +X is world +Y after a +90 degree Z rotation.
    assert max(pose.position.y for pose in waypoints[:-1]) > 0.003
    assert max(abs(pose.position.x) for pose in waypoints) < 1e-9
    assert _position(waypoints[-1]) == pytest.approx(_position(target))


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


def test_preapproach_via_pose_moves_outward_and_keeps_orientation():
    ready = Pose()
    ready.position.x = 0.10
    ready.position.y = 0.20
    ready.position.z = 0.15
    preapproach = Pose()
    preapproach.position.x = 0.40
    preapproach.position.y = -0.20
    preapproach.position.z = 0.35
    preapproach.orientation.x = 0.1
    preapproach.orientation.y = 0.2
    preapproach.orientation.z = 0.3
    preapproach.orientation.w = 0.9

    via = make_preapproach_via_pose(
        ready,
        [0.50, -0.30, 0.45],
        preapproach,
        [-1.0, -1.0, 0.0],
        interpolation_ratio=0.5,
        lateral_ratio=0.25,
    )

    # Midpoint is [0.30, -0.05, 0.30].  Recommend's component perpendicular
    # to the READY->crop chord points toward negative X/Y.
    assert _position(via) == pytest.approx(
        [0.175, -0.15, 0.30], abs=1e-9
    )
    assert via.orientation == preapproach.orientation
    assert via is not preapproach


@pytest.mark.parametrize("ratio", [-0.001, 0.0, 1.0, float("nan")])
def test_preapproach_via_pose_rejects_invalid_ratio(ratio):
    with pytest.raises(ValueError, match="interpolation_ratio"):
        make_preapproach_via_pose(
            Pose(), [1.0, 0.0, 0.0], Pose(), [0.0, 1.0, 0.0], ratio
        )


@pytest.mark.parametrize("ratio", [-0.001, float("nan")])
def test_preapproach_via_pose_rejects_invalid_lateral_ratio(ratio):
    with pytest.raises(ValueError, match="lateral_ratio"):
        make_preapproach_via_pose(
            Pose(),
            [1.0, 0.0, 0.0],
            Pose(),
            [0.0, 1.0, 0.0],
            lateral_ratio=ratio,
        )


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


def test_adaptive_grasp_rotates_toward_local_y_up_to_90_degrees():
    result = adaptive_outward_toward_robot(
        tomato_rotation=np.eye(3),
        tomato_position=[0.0, 0.0, 0.4],
        robot_position=[0.0, 1.0, 0.0],
    )

    expected = np.array([0.0, 1.0, 0.0])
    assert np.allclose(result.outward_axis, expected)
    assert np.isclose(result.applied_rotation_deg, 90.0)
    assert np.isclose(result.current_robot_error_deg, 90.0)
    assert np.isclose(result.selected_robot_error_deg, 0.0)


def test_adaptive_grasp_clamps_large_robot_error_to_90_degrees():
    angle = np.deg2rad(-20.0)
    result = adaptive_outward_toward_robot(
        tomato_rotation=np.eye(3),
        tomato_position=[0.0, 0.0, 0.4],
        robot_position=[np.cos(angle), np.sin(angle), 0.0],
    )

    assert np.allclose(result.outward_axis, [0.0, -1.0, 0.0])
    assert np.isclose(abs(result.applied_rotation_deg), 90.0)
    assert np.isclose(result.current_robot_error_deg, 160.0)
    assert np.isclose(result.selected_robot_error_deg, 70.0)


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


def test_outward_rotation_uses_signed_tomato_local_y_direction():
    assert np.allclose(
        outward_from_tomato_rotation(np.eye(3), 45.0),
        [-np.sqrt(0.5), np.sqrt(0.5), 0.0],
    )


def test_deadline_guard_is_inactive_when_nominal_pregrasp_is_robot_side():
    guard = deadline_safe_rotation_guard(
        tomato_rotation=np.eye(3),
        tomato_position=[0.0, 0.0, 0.4],
        robot_position=[-1.0, 1.0, 0.4],
        margin_deg=1.0,
    )

    assert not guard.active
    assert np.isclose(guard.minimum_rotation_deg, 0.0)
    assert guard.nominal_robot_side_dot > 0.0


def test_deadline_guard_starts_at_ideal_boundary_toward_robot():
    guard = deadline_safe_rotation_guard(
        tomato_rotation=np.eye(3),
        tomato_position=[0.0, 0.0, 0.4],
        robot_position=[1.0, 1.0, 0.4],
        margin_deg=15.0,
    )

    assert guard.active
    assert guard.minimum_rotation_deg == pytest.approx(60.0)
    selected = outward_from_tomato_rotation(
        np.eye(3),
        guard.minimum_rotation_deg,
    )
    robotward = np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0)
    assert np.dot(selected, robotward) > 0.0
    assert np.allclose(
        outward_from_tomato_rotation(np.eye(3), -90.0),
        [0.0, -1.0, 0.0],
        atol=1e-9,
    )


def test_deadline_guard_enforces_margin_even_just_inside_robot_side():
    angle = np.deg2rad(80.0)
    guard = deadline_safe_rotation_guard(
        tomato_rotation=np.eye(3),
        tomato_position=[0.0, 0.0, 0.4],
        robot_position=[-np.cos(angle), np.sin(angle), 0.4],
        margin_deg=15.0,
    )

    assert guard.active
    assert abs(guard.minimum_rotation_deg) == pytest.approx(5.0)
    selected = outward_from_tomato_rotation(
        np.eye(3),
        guard.minimum_rotation_deg,
    )
    robotward = np.array([-np.cos(angle), np.sin(angle), 0.0])
    selected_error = np.rad2deg(
        np.arccos(np.clip(np.dot(selected, robotward), -1.0, 1.0))
    )
    assert selected_error == pytest.approx(75.0)


def test_robot_side_sector_accepts_150_degrees_when_margin_is_15():
    tomato = [0.0, 0.0, 0.4]
    robot = [-1.0, 0.0, 0.4]
    boundary = np.deg2rad(75.0)
    just_outside = np.deg2rad(75.01)

    assert robot_side_approach_error_deg([-1.0, 0.0, 0.0], tomato, robot) == 0.0
    assert is_within_robot_side_approach_sector(
        [-np.cos(boundary), np.sin(boundary), 0.0],
        tomato,
        robot,
        margin_deg=15.0,
    )
    assert not is_within_robot_side_approach_sector(
        [-np.cos(just_outside), np.sin(just_outside), 0.0],
        tomato,
        robot,
        margin_deg=15.0,
    )


def test_robot_side_sector_defaults_to_robot_facing_180_degrees():
    tomato = [0.0, 0.0, 0.4]
    robot = [-1.0, 0.0, 0.4]
    boundary = np.deg2rad(90.0)
    just_outside = np.deg2rad(90.01)

    assert is_within_robot_side_approach_sector(
        [-np.cos(boundary), np.sin(boundary), 0.0], tomato, robot
    )
    assert not is_within_robot_side_approach_sector(
        [-np.cos(just_outside), np.sin(just_outside), 0.0], tomato, robot
    )


def test_robot_side_sector_includes_robot_height_in_deadline_check():
    tomato = [0.0, 0.0, 1.0]
    robot = [-1.0, 0.0, 0.0]
    horizontal_boundary = np.deg2rad(75.0)
    outward = [
        -np.cos(horizontal_boundary),
        np.sin(horizontal_boundary),
        0.0,
    ]

    assert robot_side_approach_error_deg(outward, tomato, robot) > 75.0
    assert not is_within_robot_side_approach_sector(
        outward,
        tomato,
        robot,
        margin_deg=15.0,
    )


def test_deadline_guard_rejects_captured_pose_when_45_degrees_is_insufficient():
    tomato = np.array([0.430, -0.219, 0.692])
    yaw = np.deg2rad(-143.620)
    rotation = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    guard = deadline_safe_rotation_guard(
        tomato_rotation=rotation,
        tomato_position=tomato,
        robot_position=[0.0, 0.0, 0.0],
        margin_deg=15.0,
    )

    assert guard.active
    assert abs(guard.minimum_rotation_deg) == pytest.approx(53.53, abs=0.02)
    assert abs(guard.minimum_rotation_deg) > 45.0


def test_minimum_ik_rotation_keeps_zero_when_nominal_pose_is_feasible():
    requested = []

    def evaluate(angle):
        requested.append(angle)
        return ApproachRotationEvaluation(angle, True, 0.25, 1)

    selected, evaluations = select_minimum_feasible_rotation(evaluate)

    assert selected is not None
    assert np.isclose(selected.rotation_deg, 0.0)
    assert requested == [0.0]
    assert len(evaluations) == 1


def test_minimum_ik_rotation_selects_closest_feasible_side_and_refines_it():
    def evaluate(angle):
        threshold = 28.0 if angle > 0.0 else 17.0
        feasible = abs(angle) >= threshold
        return ApproachRotationEvaluation(
            angle,
            feasible,
            abs(angle) / 100.0,
            1 if feasible else -31,
        )

    selected, evaluations = select_minimum_feasible_rotation(
        evaluate,
        max_rotation_deg=90.0,
        coarse_step_deg=10.0,
        resolution_deg=0.5,
        preferred_sign=1.0,
    )

    assert selected is not None
    assert selected.rotation_deg < 0.0
    assert 17.0 <= abs(selected.rotation_deg) < 17.5
    assert all(abs(item.rotation_deg) <= 20.0 for item in evaluations)


def test_minimum_ik_rotation_respects_deadline_bound_and_single_direction():
    def evaluate(angle):
        feasible = angle >= 35.0
        return ApproachRotationEvaluation(
            angle,
            feasible,
            abs(angle),
            1 if feasible else -31,
        )

    selected, evaluations = select_minimum_feasible_rotation(
        evaluate,
        max_rotation_deg=90.0,
        coarse_step_deg=10.0,
        resolution_deg=1.0,
        preferred_sign=1.0,
        minimum_abs_rotation_deg=30.0,
        allow_opposite_sign=False,
    )

    assert selected is not None
    assert selected.rotation_deg == pytest.approx(35.0)
    assert all(item.rotation_deg >= 30.0 for item in evaluations)


def test_minimum_ik_rotation_uses_joint_distance_to_break_angle_tie():
    def evaluate(angle):
        feasible = abs(angle) >= 20.0
        distance = 0.2 if angle < 0.0 else 1.5
        return ApproachRotationEvaluation(
            angle,
            feasible,
            distance,
            1 if feasible else -31,
        )

    selected, _ = select_minimum_feasible_rotation(
        evaluate,
        coarse_step_deg=10.0,
        resolution_deg=1.0,
        preferred_sign=1.0,
    )

    assert selected is not None
    assert selected.rotation_deg < 0.0


def test_minimum_ik_rotation_returns_none_when_every_angle_is_invalid():
    def evaluate(angle):
        return ApproachRotationEvaluation(angle, False, moveit_error_code=-31)

    selected, evaluations = select_minimum_feasible_rotation(
        evaluate,
        max_rotation_deg=30.0,
        coarse_step_deg=10.0,
    )

    assert selected is None
    assert {item.rotation_deg for item in evaluations} == {
        0.0,
        -10.0,
        10.0,
        -20.0,
        20.0,
        -30.0,
        30.0,
    }


def test_robotward_rotation_uses_requested_maximum_when_it_is_feasible():
    requested = []

    def evaluate(angle):
        requested.append(angle)
        return ApproachRotationEvaluation(angle, True, abs(angle), 1)

    selected, evaluations = select_robotward_feasible_rotation(
        evaluate,
        desired_rotation_deg=-45.0,
        max_rotation_deg=45.0,
    )

    assert selected is not None
    assert selected.rotation_deg == pytest.approx(-45.0)
    assert requested == [-45.0]
    assert len(evaluations) == 1


def test_robotward_rotation_refines_to_most_robot_facing_feasible_angle():
    def evaluate(angle):
        feasible = 10.0 <= angle <= 32.0
        return ApproachRotationEvaluation(
            angle,
            feasible,
            abs(angle),
            1 if feasible else -31,
        )

    selected, evaluations = select_robotward_feasible_rotation(
        evaluate,
        desired_rotation_deg=45.0,
        max_rotation_deg=45.0,
        coarse_step_deg=10.0,
        resolution_deg=0.25,
    )

    assert selected is not None
    assert 31.75 <= selected.rotation_deg <= 32.0
    assert all(item.rotation_deg >= 25.0 for item in evaluations)


def test_robotward_rotation_does_not_cross_deadline_minimum():
    def evaluate(angle):
        feasible = angle <= 25.0
        return ApproachRotationEvaluation(
            angle,
            feasible,
            abs(angle),
            1 if feasible else -31,
        )

    selected, evaluations = select_robotward_feasible_rotation(
        evaluate,
        desired_rotation_deg=45.0,
        max_rotation_deg=45.0,
        coarse_step_deg=10.0,
        minimum_abs_rotation_deg=30.0,
    )

    assert selected is None
    assert all(item.rotation_deg >= 30.0 for item in evaluations)


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

    assert np.allclose(positions[0], [1.0, 2.040, 3.0])
    assert np.allclose(positions[1], [1.0, 2.060, 3.020])
    assert np.allclose(positions[2], [1.0, 2.060, 3.040])
    assert np.allclose(positions[3], [1.0, 2.010, 3.040])
    assert np.allclose(positions[4], [1.0, 2.000, 3.040])


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


def test_joint_span_safety_gate_includes_wrist3():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ["base", "wrist3"]
    trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[0.0, 0.0]),
        JointTrajectoryPoint(
            positions=[np.deg2rad(90.0), np.deg2rad(121.0)]
        ),
    ]

    violations = joint_span_violations([trajectory], 120.0)

    assert [item["joint_name"] for item in violations] == ["wrist3"]
    assert violations[0]["span_deg"] == pytest.approx(121.0)


def test_trajectory_safety_rejects_c0_t6_style_long_joint_branch():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = [
        "base",
        "shoulder",
        "elbow",
        "wrist1",
        "wrist2",
        "wrist3",
    ]
    trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(
            positions=np.deg2rad(
                [-203.35, 346.05, -54.06, 170.58, 89.96, 192.74]
            )
        )
    ]
    start = dict(
        zip(
            trajectory.joint_trajectory.joint_names,
            np.deg2rad(
                [181.98, 66.51, -118.33, -36.23, -91.34, 88.14]
            ),
        )
    )

    violations = trajectory_joint_safety_violations(
        trajectory,
        start,
        ["base", "shoulder", "elbow", "wrist1", "wrist2"],
        maximum_span_deg=120.0,
        wrist3_maximum_span_deg=180.0,
        maximum_step_deg=45.0,
    )

    names = {item["joint_name"] for item in violations}
    assert {"base", "shoulder", "wrist1", "wrist2"} <= names
    base = next(item for item in violations if item["joint_name"] == "base")
    assert base["span_deg"] == pytest.approx(385.33)
    assert "SPAN_LIMIT_EXCEEDED" in base["reason"]


def test_trajectory_safety_allows_small_motion_and_limits_wrist3_separately():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ["base", "wrist3"]
    trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(positions=np.deg2rad([20.0, 150.0]))
    ]

    assert trajectory_joint_safety_violations(
        trajectory,
        {"base": 0.0, "wrist3": 0.0},
        ["base"],
        maximum_span_deg=120.0,
        wrist3_maximum_span_deg=180.0,
        maximum_step_deg=180.0,
    ) == []

    trajectory.joint_trajectory.points[0].positions = list(
        np.deg2rad([20.0, 181.0])
    )
    violations = trajectory_joint_safety_violations(
        trajectory,
        {"base": 0.0, "wrist3": 0.0},
        ["base"],
        maximum_span_deg=120.0,
        wrist3_maximum_span_deg=180.0,
        maximum_step_deg=180.0,
    )
    assert [item["joint_name"] for item in violations] == ["wrist3"]


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
