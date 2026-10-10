"""Horizontal entry contract, independent of the native simulator and IK solver.

These controls isolate the commanded CAD frame and planner branch handling.
They do not certify collision clearance or physical tracking.
"""
import unittest

import numpy as np

from motion_kinematics import plan_motion, preparation_phases


class PoseRecordingIK:
    """Accept requested poses, without exposing FK or a real robot model."""

    def __init__(self):
        self.world = np.eye(4)
        self.world[:3, 3] = [.1, -.2, .0]
        self.calls = []

    def ik(self, pose, seed):
        self.calls.append(np.asarray(pose).copy())
        q = np.asarray(seed, dtype=float).copy()
        q[:3] = pose[:3, 3]
        return q, 0., 0.


class TiltOnlyIK(PoseRecordingIK):
    """An available tilted solution must not silently replace level entry."""

    def ik(self, pose, seed):
        q, _, _ = super().ik(pose, seed)
        if abs(float(pose[2, 1])) > .1:
            return q, 0., 0.
        return q, .01, .1


class AlternateBranchIK(PoseRecordingIK):
    """The best preparation branch cannot continue, but another branch can."""

    def __init__(self, center):
        super().__init__()
        inward = np.asarray(center)[:2] - self.world[:2, 3]
        inward /= np.linalg.norm(inward)
        self.prep_position = np.asarray(center, dtype=float).copy()
        self.prep_position[:2] -= inward * (.093 + .10)
        self.prep_position[2] -= .037
        self.failed_continuations = 0

    def ik(self, pose, seed):
        self.calls.append(np.asarray(pose).copy())
        # Return a shorter preparation for branch 1, independently of the
        # planner's seed order: only the branch coordinate differs.
        q = np.zeros(7)
        at_preparation = np.linalg.norm(pose[:3, 3] - self.prep_position) < 1e-10
        # The original seeds have wrist3=0. The returned wrist3 value is just
        # this test's persistent branch identity, not a real robot joint.
        branch = int(round(seed[6]))
        if at_preparation and branch not in (1, 2):
            branch = 1 if seed[1] < 0 else 2
        q[:3] = pose[:3, 3]
        q[6] = branch
        if branch == 1 and not at_preparation:
            self.failed_continuations += 1
            return q, .005, 0.
        return q, (0. if branch == 1 else .0002), 0.


class HorizontalEntryTests(unittest.TestCase):
    center = np.array([.2, 1.3, 1.])

    def assert_level_entry(self, metadata):
        prep = np.asarray(metadata['prep_pose'])
        goal = np.asarray(metadata['goal_pose'])
        poses = np.asarray(metadata['approach_waypoint_poses'])
        self.assertEqual(poses.shape, (61, 4, 4))
        np.testing.assert_allclose(poses[:, 2, 1], 0., atol=1e-14)
        np.testing.assert_allclose(poses[:, :3, 2],
                                   np.tile([0., 0., 1.], (len(poses), 1)), atol=1e-14)
        np.testing.assert_allclose(poses[:, 2, 3], self.center[2] - .037, atol=1e-14)
        np.testing.assert_allclose(poses[:, :3, :3],
                                   np.repeat(goal[None, :3, :3], len(poses), axis=0), atol=1e-14)
        np.testing.assert_allclose(poses[0], prep, atol=1e-14)
        np.testing.assert_allclose(poses[-1], goal, atol=1e-14)
        direction = np.asarray(metadata['approach_direction_world'])
        self.assertAlmostEqual(np.linalg.norm(direction), 1., places=12)
        np.testing.assert_allclose(goal[:3, 3] - prep[:3, 3], direction * .10, atol=1e-14)
        self.assertEqual(metadata['approach_distance_m'], .10)
        # The fruit remains at the CAD belt's contact location; level entry
        # must not move the target or invent a different contact reference.
        np.testing.assert_allclose(goal[:3, :3] @ [0., .093, .037] + goal[:3, 3],
                                   self.center, atol=1e-14)

    def test_default_entry_is_level_and_keeps_full_standoff(self):
        kin = PoseRecordingIK()
        _, metadata = plan_motion(kin, self.center)
        self.assert_level_entry(metadata)
        self.assertEqual(metadata['pitch_deg'], 0.)
        self.assertEqual(metadata['yaw_offset_deg'], 0.)
        radial = self.center - np.r_[kin.world[:2, 3], self.center[2]]
        radial /= np.linalg.norm(radial)
        np.testing.assert_allclose(metadata['approach_direction_world'], radial, atol=1e-14)
        self.assertTrue(all(abs(pose[2, 1]) < 1e-14 for pose in kin.calls))

    def test_explicit_yaw_rotates_about_world_up_without_tilting(self):
        kin = PoseRecordingIK()
        yaw = 25.
        _, metadata = plan_motion(kin, self.center, yaw_offset_deg=yaw)
        self.assert_level_entry(metadata)
        radial = self.center[:2] - kin.world[:2, 3]
        radial /= np.linalg.norm(radial)
        angle = np.deg2rad(yaw)
        expected = np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]]) @ radial
        np.testing.assert_allclose(metadata['approach_direction_world'], np.r_[expected, 0.], atol=1e-14)
        self.assertEqual(metadata['yaw_offset_deg'], yaw)

    def test_tilted_solution_is_not_an_automatic_fallback(self):
        kin = TiltOnlyIK()
        with self.assertRaises(RuntimeError):
            plan_motion(kin, self.center)
        self.assertGreater(len(kin.calls), 0)
        self.assertTrue(all(abs(pose[2, 1]) < 1e-14 for pose in kin.calls))

    def test_explicit_pitch_is_recorded_and_never_substituted(self):
        kin = PoseRecordingIK()
        _, metadata = plan_motion(kin, self.center, pitch_deg=-60.)
        self.assertEqual(metadata['pitch_deg'], -60.)
        self.assertEqual(metadata['yaw_offset_deg'], 0.)
        poses = np.asarray(metadata['approach_waypoint_poses'])
        np.testing.assert_allclose(poses[:, 2, 1], -np.sin(np.deg2rad(60.)), atol=1e-14)
        self.assertAlmostEqual(poses[-1, 2, 3] - poses[0, 2, 3],
                               -.10 * np.sin(np.deg2rad(60.)), places=12)

    def test_another_branch_is_tried_when_best_prep_cannot_continue(self):
        kin = AlternateBranchIK(self.center)
        phases, metadata = plan_motion(kin, self.center)
        self.assertGreater(kin.failed_continuations, 0)
        self.assert_level_entry(metadata)
        approach = next(phase for phase in phases if phase[0] == 'approach')
        np.testing.assert_array_equal(approach[2][:, 6], np.full(len(approach[2]), 2.))
        self.assertTrue(metadata['ik_attempts'])

    def test_nonfinite_inputs_are_rejected_before_ik(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            variants = [dict(center=[value, 1.3, 1.]),
                        dict(center=self.center, start=[value, 0., 0., 0., 0., 0., 0.]),
                        dict(center=self.center, pitch_deg=value),
                        dict(center=self.center, yaw_offset_deg=value)]
            for kwargs in variants:
                with self.subTest(kwargs=kwargs):
                    kin = PoseRecordingIK()
                    with self.assertRaises(ValueError):
                        plan_motion(kin, **kwargs)
                    self.assertEqual(kin.calls, [])

    def test_zero_horizontal_target_direction_is_rejected(self):
        kin = PoseRecordingIK()
        with self.assertRaises(ValueError):
            plan_motion(kin, [.1, -.2, 1.])
        self.assertEqual(kin.calls, [])


class RecordedPrefixTests(unittest.TestCase):
    """Reuse measured safe preparation commands without replacing the start."""

    def setUp(self):
        self.start = np.zeros(7)
        self.prep = np.array([.01, -2.02, .32, -2.04, -1.42, -.47, 3.14])
        fold = self.start.copy()
        fold[2:4] = [.635, -2.063]
        tight = fold.copy()
        tight[0] = .24
        tight[2:4] = [1.3, -2.3]
        self.recorded = [
            ('prepare_fold', 6., np.array([self.start, fold]), .044, 0., 0.),
            ('prepare_tight_fold', 4., np.array([fold, tight]), .044, 0., 0.),
        ]

    def assert_continuous(self, phases):
        np.testing.assert_array_equal(phases[0][2][0], self.start)
        np.testing.assert_array_equal(phases[-1][2][-1], self.prep)
        for previous, following in zip(phases, phases[1:]):
            np.testing.assert_array_equal(previous[2][-1], following[2][0])

    def test_recorded_commands_are_preserved_and_connect_to_new_staging(self):
        phases = preparation_phases(self.start, self.prep,
                                    'tight_fold_open_approach', recorded_prefix=self.recorded)
        for recorded, actual in zip(self.recorded, phases[:2]):
            self.assertEqual(recorded[:2], actual[:2])
            self.assertEqual(recorded[3:], actual[3:])
            np.testing.assert_array_equal(recorded[2], actual[2])
        self.assert_continuous(phases)
        # The old prefix's measured lift is retained during orientation/turn;
        # reaching the new staging lift belongs to the subsequent extension.
        for phase in phases[2:4]:
            np.testing.assert_array_equal(phase[2][:, 0], [.24, .24])
        extend = next(phase for phase in phases if phase[0] == 'prepare_extend')
        self.assertEqual(extend[2][0, 0], .24)
        self.assertEqual(extend[2][-1, 0], self.prep[0])
        self.assertEqual(phases[-1][0], 'prepare_open')

    def test_planner_keeps_recorded_prefix_and_horizontal_full_entry(self):
        kin = PoseRecordingIK()
        phases, metadata = plan_motion(kin, [.2, 1.3, 1.], self.start,
                                        'tight_fold_open_approach', recorded_prefix=self.recorded)
        for recorded, actual in zip(self.recorded, phases[:2]):
            self.assertEqual(recorded[:2], actual[:2])
            self.assertEqual(recorded[3:], actual[3:])
            np.testing.assert_array_equal(recorded[2], actual[2])
        for previous, following in zip(phases[:5], phases[1:6]):
            np.testing.assert_array_equal(previous[2][-1], following[2][0])
        HorizontalEntryTests().assert_level_entry(metadata)

    def test_default_profile_is_unchanged_when_prefix_is_omitted(self):
        expected = preparation_phases(self.start, self.prep, 'tight_fold_open_approach')
        explicit = preparation_phases(self.start, self.prep,
                                      'tight_fold_open_approach', recorded_prefix=None)
        self.assertEqual(len(expected), len(explicit))
        for before, after in zip(expected, explicit):
            self.assertEqual(before[:2], after[:2])
            self.assertEqual(before[3:], after[3:])
            np.testing.assert_array_equal(before[2], after[2])

    def test_invalid_prefix_shape_start_and_continuity_are_rejected(self):
        def copy_prefix():
            return [(name, duration, q.copy(), gap, left, right)
                    for name, duration, q, gap, left, right in self.recorded]
        invalid = []
        wrong_shape = copy_prefix()
        wrong_shape[0] = (*wrong_shape[0][:2], np.zeros((2, 6)), *wrong_shape[0][3:])
        invalid.append(wrong_shape)
        wrong_start = copy_prefix()
        wrong_start[0][2][0, 0] = .01
        invalid.append(wrong_start)
        discontinuous = copy_prefix()
        discontinuous[1][2][0, 0] = .01
        invalid.append(discontinuous)
        nonfinite = copy_prefix()
        nonfinite[1][2][1, 2] = float('nan')
        invalid.append(nonfinite)
        invalid.extend([self.recorded[:1], self.recorded[::-1]])
        for recorded in invalid:
            with self.subTest(recorded=recorded):
                with self.assertRaises(ValueError):
                    preparation_phases(self.start, self.prep,
                                       'tight_fold_open_approach', recorded_prefix=recorded)

    def test_other_preparation_profiles_reject_recorded_tight_prefix(self):
        with self.assertRaises(ValueError):
            preparation_phases(self.start, self.prep, 'direct', recorded_prefix=self.recorded)

    def test_level_profile_keeps_pitch_joint_sum_during_extension(self):
        phases = preparation_phases(self.start, self.prep,
                                    'tight_fold_level_approach', recorded_prefix=self.recorded)
        extend = next(phase for phase in phases if phase[0] == 'prepare_extend')
        expected_sum = np.sum(self.prep[2:5])
        # The native executor interpolates joint commands between endpoints.
        # Check interior commands, not just the staging endpoint. This is an
        # algebraic contract; the actual robot's joint axes still require FK.
        for fraction in np.linspace(0., 1., 81):
            q = (1. - fraction) * extend[2][0] + fraction * extend[2][-1]
            self.assertAlmostEqual(np.sum(q[2:5]), expected_sum, places=12)
        turned = next(phase for phase in phases if phase[0] == 'prepare_turn_tight')
        np.testing.assert_array_equal(turned[2][0, 2:], turned[2][-1, 2:])
        self.assertAlmostEqual(np.sum(turned[2][0, 2:5]), expected_sum, places=12)

    def test_level_profile_preserves_recorded_prefix_and_continuity(self):
        phases = preparation_phases(self.start, self.prep,
                                    'tight_fold_level_approach', recorded_prefix=self.recorded)
        for recorded, actual in zip(self.recorded, phases[:2]):
            self.assertEqual(recorded[:2], actual[:2])
            self.assertEqual(recorded[3:], actual[3:])
            np.testing.assert_array_equal(recorded[2], actual[2])
        self.assert_continuous(phases)
        opening = phases[-1]
        self.assertEqual((opening[0], opening[1], opening[3:]),
                         ('prepare_open', 2., (.060, 0., 0.)))
        np.testing.assert_array_equal(opening[2], [self.prep, self.prep])

    def test_original_open_profile_retains_its_original_wrist_command(self):
        original = preparation_phases(self.start, self.prep,
                                      'tight_fold_open_approach', recorded_prefix=self.recorded)
        level = preparation_phases(self.start, self.prep,
                                   'tight_fold_level_approach', recorded_prefix=self.recorded)
        old_orient = next(phase for phase in original if phase[0] == 'prepare_orient_tight')
        new_orient = next(phase for phase in level if phase[0] == 'prepare_orient_tight')
        self.assertEqual(old_orient[2][-1, 4], self.prep[4])
        self.assertNotEqual(new_orient[2][-1, 4], old_orient[2][-1, 4])
        # The new opt-in compensation affects wrist1, not wrist2/wrist3 or the
        # recorded preparation commands retained by the original profile.
        np.testing.assert_array_equal(old_orient[2][-1, 5:], new_orient[2][-1, 5:])
        for recorded, actual in zip(self.recorded, original[:2]):
            np.testing.assert_array_equal(recorded[2], actual[2])


if __name__ == '__main__':
    unittest.main()
