"""Exact native selection, actual FK movement and unchanged harvest commands."""
import copy
import hashlib
import json
import unittest

import numpy as np

from last_fruit_grasp_sway import apply_last_fruit_grasp_sway
from motion_kinematics import grasp_pose, plan_motion
from test_motion_branch import RedundantCartesianRobot


class LastFruitGraspSwayTests(unittest.TestCase):
    def setUp(self):
        self.kin = RedundantCartesianRobot()
        center = np.array([.4, 1., 1.2])
        ready = grasp_pose(center, center.copy())
        ready[:3, 3] -= ready[:3, 1] * .3
        start = self.kin.ik_at_lift(ready, np.zeros(7), .12)[0]
        self.phases, self.plan = plan_motion(self.kin, center, start,
            preparation='horizontal_ready', harvest_profile='belt_open_close_cycle',
            preparation_height_first=True, approach_direction_world=ready[:3, 1], close_seconds=6.)
        self.fruits = [dict(name='Truss_02_Fruit_'+n,
            joint='/World/ElasticTarget/HarvestJoints/Truss_02_Fruit_'+n) for n in ('10', '11')]
        self.breaks = [dict(joint=self.fruits[1]['joint'], t=137.125)]
        self.group = dict(fruits=[self.fruits[0]['name']])
        self.options = dict(amplitude_m=.0015, fruits=self.fruits, breaks=self.breaks, group=self.group)

    def test_none_and_zero_return_original_commands_and_metadata_exactly(self):
        for amplitude in (None, 0.):
            a, b = apply_last_fruit_grasp_sway(None, self.phases, self.plan, amplitude_m=amplitude)
            self.assertIs(a, self.phases)
            self.assertIs(b, self.plan)
            self.assertNotIn('last_fruit_grasp_sway', b)
        # Requested two-repetition command capture is unchanged by zero sway.
        phases, plan = plan_motion(self.kin, np.array([.4, 1., 1.2]), self.plan['start'],
            preparation='horizontal_ready', harvest_profile='belt_open_close_cycle',
            preparation_height_first=True, approach_direction_world=self.plan['approach_direction_world'])
        a, b = apply_last_fruit_grasp_sway(None, phases, plan)
        digest = hashlib.sha256(json.dumps([p for p in b['phases'] if p['name']!='prepare_open_hold'], sort_keys=True,
            separators=(',', ':')).encode()).hexdigest()
        self.assertEqual(digest, 'd7f3058ba28e6ec101d5404c6f06b62da57b14c3f3e8eda1ad04d563eefcca1a')

    def test_last_selection_requires_exact_native_break_not_boolean_or_other_joint(self):
        for breaks in ([], [dict(joint='/World/OtherPlant/HarvestJoints/Truss_02_Fruit_11', t=1.)]):
            a, b = apply_last_fruit_grasp_sway(self.kin, self.phases, self.plan,
                **dict(self.options, breaks=breaks, group=dict(self.group, last_remaining_fruit=True)))
            self.assertIs(a, self.phases)
            self.assertIs(b, self.plan)
        a, b = apply_last_fruit_grasp_sway(self.kin, self.phases, self.plan,
            **dict(self.options, group=dict(fruits=[self.fruits[1]['name']])))
        self.assertIs(a, self.phases)
        self.assertIs(b, self.plan)

    def test_actual_fk_swings_both_directions_and_returns_without_changing_other_commands(self):
        original = copy.deepcopy(self.plan)
        original_arrays = [p[2].copy() for p in self.phases]
        phases, plan = apply_last_fruit_grasp_sway(self.kin, self.phases, self.plan, **self.options)
        self.assertEqual(original, self.plan)
        goal = self.kin.fk(self.plan['goal_q'])
        changed = []
        for index, (old, new) in enumerate(zip(self.phases, phases)):
            np.testing.assert_array_equal(old[2], original_arrays[index])
            self.assertEqual(old[:2], new[:2])
            self.assertEqual(old[3:], new[3:])
            if old[0] not in ('close', 'cycle_reclose'):
                np.testing.assert_array_equal(old[2], new[2])
                self.assertEqual(self.plan['phases'][index], plan['phases'][index])
                continue
            changed.append(old[0])
            self.assertEqual(new[2].shape, (161, 7))
            np.testing.assert_array_equal(new[2][0], old[2][0])
            np.testing.assert_array_equal(new[2][-1], old[2][-1])
            poses = np.asarray([self.kin.fk(q) for q in new[2]])
            np.testing.assert_allclose(poses[:, :3, :3],
                np.broadcast_to(goal[:3, :3], (161, 3, 3)), atol=1e-14)
            displacement = poses[:, :3, 3] - goal[:3, 3]
            s = np.linspace(0., 1., 161)
            expected = np.zeros(161)
            expected[s > .75] = .0015 * np.sin(2*np.pi*(s[s > .75]-.75)/.25)
            expected[-1] = 0.
            np.testing.assert_allclose(displacement, expected[:, None]*goal[:3, 0], atol=1e-14)
            np.testing.assert_array_equal(new[2][:121], np.tile(old[2][0], (121, 1)))
            # Peak shifts occur only after the existing smoothstep jaw travel
            # brings a 96mm opening near the fruit. Gap commands are untouched.
            self.assertAlmostEqual(s[np.argmax(expected)], .8125)
            self.assertAlmostEqual(s[np.argmin(expected)], .9375)
            self.assertAlmostEqual(.096 + (.028-.096)*s[np.argmax(expected)], .04075)
            self.assertAlmostEqual(.096 + (.028-.096)*s[np.argmin(expected)], .03225)
            self.assertAlmostEqual(float((displacement @ goal[:3, 0]).max()), .0015)
            self.assertAlmostEqual(float((displacement @ goal[:3, 0]).min()), -.0015)
        self.assertEqual(changed, ['close', 'cycle_reclose'])
        self.assertEqual(plan['belt_cycle'], self.plan['belt_cycle'])
        self.assertEqual(plan['goal_pose'], self.plan['goal_pose'])
        self.assertFalse(plan['last_fruit_grasp_sway']['collision_certified'])
        self.assertFalse(plan['last_fruit_grasp_sway']['native_physics_executed'])
        self.assertTrue(plan['last_fruit_grasp_sway']['exact_native_inventory']['last_remaining_fruit'])
        self.assertEqual([p['sample_count'] for p in plan['last_fruit_grasp_sway']['command_fk_proofs']], [721, 721])
        for old, new in zip(phases, phases[1:]):
            # The unchanged planner's other Cartesian connectors already have
            # sub-ULP endpoint arithmetic differences. The replaced grasp's
            # original endpoints were checked bit-for-bit above.
            np.testing.assert_allclose(old[2][-1], new[2][0], atol=1e-14, rtol=0.)

    def test_existing_fixed_lift_and_constant_rail_are_preserved(self):
        plan = copy.deepcopy(self.plan)
        plan['ready_lift_branch'] = dict(fixed_lift_m=plan['goal_q'][0])
        for row in plan['phases']:
            row['rail_waypoint_positions_m'] = [1.25]*len(row['waypoint_q'])
        phases, result = apply_last_fruit_grasp_sway(self.kin, self.phases, plan, **self.options)
        for p, row in zip(phases, result['phases']):
            self.assertEqual(row['rail_waypoint_positions_m'], [1.25]*len(p[2]))
            if p[0] in ('close', 'cycle_reclose'):
                np.testing.assert_array_equal(p[2][:, 0], np.full(len(p[2]), plan['goal_q'][0]))

    def test_exact_mapping_and_only_static_grasp_phases_are_required(self):
        for fruits in (self.fruits+self.fruits[:1],
                       [self.fruits[0], dict(self.fruits[1], joint=self.fruits[0]['joint'])]):
            with self.assertRaisesRegex(ValueError, 'Duplicate'):
                apply_last_fruit_grasp_sway(self.kin, self.phases, self.plan,
                    **dict(self.options, fruits=fruits))
        for key, value in (('preparation', 'direct'), ('entry_only', True), ('pitch_deg', 2.)):
            with self.assertRaises(ValueError):
                apply_last_fruit_grasp_sway(self.kin, self.phases, dict(self.plan, **{key:value}), **self.options)
        plan = copy.deepcopy(self.plan)
        close = next(row for row in plan['phases'] if row['name']=='close')
        close['waypoint_q'][-1][1] += .001
        phases = [(r['name'],r['duration_s'],np.asarray(r['waypoint_q']),r['end_gap_m'],*r['belt_local_y_m_s']) for r in plan['phases']]
        with self.assertRaisesRegex(ValueError, 'static'):
            apply_last_fruit_grasp_sway(self.kin, phases, plan, **self.options)

    def test_velocity_limit_catches_branch_jump_without_retiming(self):
        class Jump(RedundantCartesianRobot):
            def ik(self, pose, seed):
                q, pe, re = super().ik(pose, seed)
                q[4] += 2*np.pi
                return q, pe, re
        with self.assertRaisesRegex(RuntimeError, 'velocity'):
            apply_last_fruit_grasp_sway(Jump(), self.phases, self.plan, **self.options)
        with self.assertRaisesRegex(RuntimeError, 'velocity'):
            apply_last_fruit_grasp_sway(self.kin, self.phases, self.plan,
                **self.options, joint_velocity_limits=[1e-8]*7)

    def test_fk_verification_rejects_unchanged_ik_commands_despite_reported_zero_error(self):
        class NoMotion(RedundantCartesianRobot):
            def ik(self, pose, seed):
                return np.asarray(self.fixed).copy(), 0., 0.
        kin = NoMotion();kin.fixed = self.plan['goal_q']
        with self.assertRaisesRegex(RuntimeError, 'interpolated'):
            apply_last_fruit_grasp_sway(kin, self.phases, self.plan, **self.options)

    def test_invalid_amplitude_nonfinite_bounds_speed_and_moving_rail_fail_closed(self):
        for value in (True, -.001, .00150001, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                apply_last_fruit_grasp_sway(self.kin, self.phases, self.plan,
                    **dict(self.options, amplitude_m=value))
        for hz in (0, 1001, 120.5, True):
            with self.assertRaises(ValueError):
                apply_last_fruit_grasp_sway(self.kin, self.phases, self.plan, **self.options, hz=hz)
        plan = copy.deepcopy(self.plan)
        next(r for r in plan['phases'] if r['name']=='close')['rail_waypoint_positions_m']=[0., .001]
        with self.assertRaisesRegex(ValueError, 'moving rail'):
            apply_last_fruit_grasp_sway(self.kin, self.phases, plan, **self.options)


if __name__ == '__main__':
    unittest.main()
