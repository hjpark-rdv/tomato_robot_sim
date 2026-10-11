"""Requested lateral arm aim; no fruit relocation or physical success claim.

The redundant Cartesian fixture makes every IK command independently readable
through FK. Existing composed-robot/native tracking still requires its audit.
"""
import hashlib
import json
import unittest

import numpy as np

from motion_kinematics import grasp_pose, plan_motion
from test_motion_branch import RedundantCartesianRobot


class GraspLateralOffsetTests(unittest.TestCase):
    def setUp(self):
        self.kin = RedundantCartesianRobot()
        self.center = np.array([.4, 1., 1.2])
        ready = grasp_pose(self.center, self.center.copy())
        ready[:3, 3] -= ready[:3, 1] * .3
        self.start = self.kin.ik_at_lift(ready, np.zeros(7), .12)[0]
        self.options = dict(preparation='horizontal_ready',
            harvest_profile='belt_open_close_cycle', preparation_height_first=True,
            approach_direction_world=ready[:3, 1])

    def test_zero_retains_captured_two_repetition_commands_exactly(self):
        original, original_meta = plan_motion(self.kin, self.center, self.start, **self.options)
        explicit, explicit_meta = plan_motion(self.kin, self.center, self.start,
            **self.options, grasp_lateral_offset_m=0.)
        digest = hashlib.sha256(json.dumps(original_meta['phases'], sort_keys=True,
            separators=(',', ':')).encode()).hexdigest()
        self.assertEqual(digest, 'd7f3058ba28e6ec101d5404c6f06b62da57b14c3f3e8eda1ad04d563eefcca1a')
        self.assertEqual(original_meta, explicit_meta)
        for before, after in zip(original, explicit):
            self.assertEqual(before[:2], after[:2])
            np.testing.assert_array_equal(before[2], after[2])
            self.assertEqual(before[3:], after[3:])

    def test_negative_offset_moves_tool_left_and_unchanged_fruit_right_in_fk(self):
        _, nominal = plan_motion(self.kin, self.center, self.start, **self.options)
        phases, moved = plan_motion(self.kin, self.center, self.start,
            **self.options, grasp_lateral_offset_m=-.001)
        goal = np.asarray(nominal['goal_pose'])
        delta = -.001 * goal[:3, 0]
        before = np.asarray(nominal['approach_waypoint_poses'])
        after = np.asarray(moved['approach_waypoint_poses'])
        np.testing.assert_array_equal(before[:, :3, :3], after[:, :3, :3])
        np.testing.assert_allclose(after[:, :3, 3] - before[:, :3, 3],
            np.broadcast_to(delta, (61, 3)), atol=2e-15)
        entry = next(p for p in phases if p[0] == 'approach')
        fk = np.asarray([self.kin.fk(q) for q in entry[2]])
        np.testing.assert_allclose(fk, after, atol=2e-15)
        fruit_in_tool = (np.linalg.inv(fk[-1]) @ np.r_[self.center, 1.])[:3]
        np.testing.assert_allclose(fruit_in_tool, [.001, .093, .037], atol=2e-15)
        np.testing.assert_array_equal(moved['target_center_world_m'], self.center)
        np.testing.assert_allclose(moved['grasp_center_world_m'], self.center + delta, atol=2e-15)
        np.testing.assert_allclose(moved['grasp_lateral_translation_world_m'], delta, atol=2e-15)
        # This tests the actual command endpoints, including the untouched start
        # and return. No native pose-setting or plant transform exists here.
        np.testing.assert_array_equal(phases[0][2][0], self.start)
        np.testing.assert_array_equal(phases[-1][2][-1], self.start)
        for previous, following in zip(phases, phases[1:]):
            np.testing.assert_allclose(previous[2][-1], following[2][0], atol=2e-15)

    def test_offset_is_cad_x_after_yaw_not_fixed_world_x_and_translation_composes(self):
        direction = np.array([-.9608018565466831, .27723598694333844, 0.])
        options = dict(self.options, approach_direction_world=direction,
            grasp_translation_world_m=[.03, -.02, .01], grasp_height_offset_m=-.0015)
        center_before = self.center.copy()
        _, nominal = plan_motion(self.kin, self.center, self.start, **options)
        _, moved = plan_motion(self.kin, self.center, self.start,
            **options, grasp_lateral_offset_m=-.001)
        goal = np.asarray(nominal['goal_pose'])
        delta = -.001 * goal[:3, 0]
        np.testing.assert_allclose(delta,
            [-.00027723598694333844, -.0009608018565466831, 0.], atol=2e-15)
        np.testing.assert_array_equal(np.asarray(moved['goal_pose'])[:3, :3], goal[:3, :3])
        np.testing.assert_allclose(np.asarray(moved['goal_pose'])[:3, 3] - goal[:3, 3], delta, atol=2e-15)
        np.testing.assert_allclose(moved['grasp_center_world_m'],
            center_before + [0., 0., -.0015] + delta + [.03, -.02, .01], atol=2e-15)
        np.testing.assert_array_equal(self.center, center_before)

    def test_signed_mechanical_scope_limits_and_both_belt_profiles(self):
        for profile in ('belt_spin_release', 'belt_open_close_cycle'):
            for value in (-.020, .020):
                with self.subTest(profile=profile, value=value):
                    _, meta = plan_motion(self.kin, self.center, self.start,
                        **dict(self.options, harvest_profile=profile), grasp_lateral_offset_m=value)
                    self.assertEqual(meta['grasp_lateral_offset_m'], value)
        for value in (-.0200001, .0200001, float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'lateral offset'):
                plan_motion(self.kin, self.center, self.start, **self.options,
                    grasp_lateral_offset_m=value)
        for scope in (dict(preparation='direct', harvest_profile='belt_open_close_cycle'),
                      dict(preparation='horizontal_ready', harvest_profile='legacy'),
                      dict(preparation='fold_then_turn', harvest_profile='legacy')):
            with self.subTest(scope=scope), self.assertRaises(ValueError):
                plan_motion(self.kin, self.center, self.start, **scope, grasp_lateral_offset_m=.001)


if __name__ == '__main__':
    unittest.main()
