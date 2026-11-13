"""Real OMPL + native MuJoCo collision tests; no GPU or hardware required."""
import json
from pathlib import Path

import mujoco as mj
import numpy as np
import unittest
import tempfile

from newton_ompl import Planner, PlanningError, JOINT_NAMES, timed_trajectory, sha256
from newton_ros_commands import Commands


def scene(*, wall=False):
    # A six-axis configuration space whose first two axes translate a sphere
    # around an obstacle. This gives an unambiguous collision-detour test.
    body = '<body name="Tool_fixed"><geom name="tool" type="sphere" size=".05" mass=".1"/>'
    body += '<body><joint name="jaw_L" type="slide" axis="1 0 0" range="-.1 .1"/><inertial pos="0 0 0" mass=".01" diaginertia=".001 .001 .001"/></body>'
    body += '<body><joint name="jaw_R" type="slide" axis="1 0 0" range="-.1 .1"/><inertial pos="0 0 0" mass=".01" diaginertia=".001 .001 .001"/></body></body>'
    for index, name in reversed(list(enumerate(JOINT_NAMES[:8]))):
        kind = 'slide' if index < 4 else 'hinge'
        axis = '0 1 0' if index == 3 else '1 0 0'
        limits = '-1 1' if index in (2, 3) else '-.01 .01'
        body = f'<body name="Robot_{index}"><joint name="{name}" type="{kind}" axis="{axis}" range="{limits}"/><inertial pos="0 0 0" mass=".1" diaginertia=".001 .001 .001"/>{body}</body>'
    obstacle = '.2 2 .2' if wall else '.2 .25 .2'
    model = mj.MjModel.from_xml_string(f'<mujoco><compiler angle="radian"/><worldbody><geom name="thick_main_stem" type="box" size="{obstacle}"/>{body}</worldbody></mujoco>')
    q = np.zeros(10); q[2] = -.6
    planner = Planner(model, dict(qpos=q), model_sha256='test-model')
    return planner


def request(planner, goal=None, **overrides):
    q = planner.initial.tolist()
    result = dict(id='test', start_positions=q, measured_positions=q,
                  goal=goal or dict(joint_names=['base'], positions=[.6]), timeout_s=2.)
    result.update(overrides)
    return result


class NativeOMPLTests(unittest.TestCase):
    def test_real_ompl_finds_and_audits_collision_detour(self):
        planner = scene()
        masks = (planner.model.geom_contype.copy(), planner.model.geom_conaffinity.copy())
        response = planner.plan(request(planner))
        assert response['success'], response
        report = response['report']
        assert report['route'] == 'rrt_connect_detour'
        assert report['ompl_edge_checks'] > 0
        assert report['state_checks'] > 100
        path = np.asarray(report['path_positions'])
        assert np.max(np.abs(path[:, 1])) > .29
        assert np.allclose(path[0], planner.initial[2:8])
        assert abs(path[-1, 0]-.6) < 1e-9
        for a, b in zip(path[:-1], path[1:]):
            assert planner.edge(a, b)
        assert np.array_equal(masks[0], planner.model.geom_contype)
        assert np.array_equal(masks[1], planner.model.geom_conaffinity)
        commands = Commands(planner.names, planner.initial, planner.bounds)
        plans, _ = commands.trajectory(response['trajectory'], 0.)
        assert len(plans) == 6


    def test_native_collision_limit_and_invalid_goal(self):
        planner = scene()
        q = planner.initial.copy(); q[2] = .2495
        assert planner.valid_full(q)  # 0.5 mm overlap remains allowed.
        q[2] = .2485
        assert not planner.valid_full(q)  # 1.5 mm cannot be ignored.
        response = planner.plan(request(planner, dict(joint_names=['base'], positions=[0.])))
        assert not response['success']
        assert 'Goal' in response['error']
        assert 'thick_main_stem' in [response['report']['last_collision'][k] for k in ('geom1', 'geom2')]


    def test_impossible_path_times_out_without_approximate_execution(self):
        planner = scene(wall=True)
        response = planner.plan(request(planner, timeout_s=.2))
        assert not response['success']
        assert response['trajectory'] is None
        assert response['report']['planning_seconds'] < .6


    def test_measured_state_and_connector_are_checked(self):
        planner = scene()
        measured = planner.initial.copy(); measured[2] = .6
        response = planner.plan(request(planner, measured_positions=measured.tolist()))
        assert not response['success']
        assert 'connector' in response['error']


    def test_invalid_joint_goal_is_rejected(self):
        planner = scene()
        goals = [dict(joint_names=['root_joint'], positions=[0.]),
            dict(joint_names=['base', 'base'], positions=[0., 0.]),
            dict(joint_names=['base'], positions=[100.]),
            dict(joint_names=['base'], positions=[float('nan')])]
        for goal in goals:
            assert not planner.plan(request(planner, goal))['success']

    def test_only_measured_state_allows_existing_tracking_bounds_envelope(self):
        planner = scene()
        planner.bounds[0, 0] = 0.
        measured = planner.initial.copy(); measured[0] = -.0001
        goal = dict(joint_names=['base'], positions=[-.5])
        result = planner.plan(request(planner, goal, measured_positions=measured.tolist()))
        assert result['success'], result
        assert result['report']['measured_start_positions'][0] == -.0001
        violation = result['report']['measured_joint_bounds_violations'][0]
        assert violation['joint_name'] == 'root_joint'
        assert violation['excess'] == .0001
        assert violation['allowed_tracking_envelope'] == .002
        # Identical tiny limit violation is forbidden for commanded targets.
        result = planner.plan(request(planner, goal, start_positions=measured.tolist()))
        assert not result['success'] and 'Commanded start' in result['error']
        result = planner.plan(request(planner, dict(joint_names=['base'], positions=[1.0001])))
        assert not result['success'] and 'Goal' in result['error']
        # The measured allowance is bounded and early rejection preserves q.
        measured[0] = -.003
        result = planner.plan(request(planner, goal, measured_positions=measured.tolist()))
        assert not result['success'] and 'tracking bounds envelope' in result['error']
        assert result['report']['measured_start_positions'][0] == -.003
        assert result['report']['measured_joint_bounds_violations'][0]['excess'] == .003

    def test_measured_bounds_allowance_does_not_clamp_away_collision(self):
        planner = scene()
        # A measured +1.5mm rail excess moves the tool into the obstacle.
        # Clamping it to the upper bound would incorrectly hide penetration.
        planner.bounds[0] = [-.01, 0.]
        planner.initial[2] = -.25
        measured = planner.initial.copy(); measured[0] = .0015
        result = planner.plan(request(planner, dict(joint_names=['base'], positions=[-.5]),
                                      measured_positions=measured.tolist()))
        assert not result['success'] and 'collision' in result['error']
        assert result['report']['last_collision']['penetration_m'] > .001
        assert result['report']['measured_start_positions'][0] == .0015


    def test_pose_goal_is_tool_origin_in_world(self):
        planner = scene()
        position, matrix = planner._pose(planner.initial[2:8])
        q = np.zeros(4); mj.mju_mat2Quat(q, matrix.ravel())
        response = planner.plan(request(planner, dict(position=position.tolist(),
                          quaternion_xyzw=np.r_[q[1:], q[0]].tolist())))
        assert response['success'], response
        assert response['report']['ik_position_error_m'] < 1e-6
        assert response['report']['tool_frame'] == 'Tool_fixed body origin'


    def test_quintic_timing_matches_controller_without_zero_time_point(self):
        planner = scene()
        path = np.array([planner.initial[2:8], planner.initial[2:8] + [.1, .1, 0, 0, 0, 0]])
        trajectory = timed_trajectory(path, planner.names[2:8])
        assert trajectory['points'][0]['time_from_start'] > 0
        Commands(planner.names, planner.initial, planner.bounds).trajectory(trajectory, 0.)


    def test_loading_verifies_model_and_initial_state_hashes(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        tmp_path = Path(directory.name)
        planner = scene()
        mj.mj_saveModel(planner.model, str(tmp_path / 'input_model.mjb'))
        np.savez(tmp_path / 'initial_state.npz', qpos=planner.initial)
        files = {name: dict(sha256=sha256(tmp_path / name))
                 for name in ('input_model.mjb', 'initial_state.npz')}
        (tmp_path / 'inputs.json').write_text(json.dumps(dict(input_files=files)))
        verified = Planner.from_inputs(tmp_path)
        assert verified.model_sha256 == files['input_model.mjb']['sha256']
        np.savez(tmp_path / 'initial_state.npz', qpos=planner.initial + .1)
        with unittest.TestCase().assertRaisesRegex(PlanningError, 'SHA256'):
            Planner.from_inputs(tmp_path)


if __name__ == '__main__':
    unittest.main()
