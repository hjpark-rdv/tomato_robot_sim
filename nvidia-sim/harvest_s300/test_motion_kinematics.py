"""CAD linkage constraints and contact-gap regression for the motion adapter."""
import unittest
import numpy as np
from motion_kinematics import transmission_transforms, grasp_pose, preparation_phases, plan_motion


class LinkageTests(unittest.TestCase):
    def test_nominal_pose_is_identity(self):
        for tf in transmission_transforms(.044).values():
            np.testing.assert_allclose(tf,np.eye(4),atol=1e-14)

    def test_rod_endpoints_match_crank_and_slider_over_full_travel(self):
        # Independent CAD constraints: 25 mm crank, 45 mm rods, 18 mm
        # slider-to-contact-face offset. Evaluate transformed original pins.
        pivot=np.array([0.,-.031,-.018]);s0=.044/2+.018
        a0=np.arccos((s0*s0+.025**2-.045**2)/(2*s0*.025))
        for gap in np.linspace(.028,.096,41):
            transforms=transmission_transforms(gap)
            for name,sign in [('linkL',-1),('linkR',1)]:
                crank_pin=pivot+np.array([sign*.025*np.cos(a0),0,sign*.025*np.sin(a0)])
                slider_pin=pivot+np.array([sign*s0,0,0])
                def apply(tf,p):return tf[:3,:3]@p+tf[:3,3]
                link_a=apply(transforms[name],crank_pin);link_b=apply(transforms[name],slider_pin)
                np.testing.assert_allclose(link_a,apply(transforms['crank'],crank_pin),atol=1e-13)
                np.testing.assert_allclose(link_b,pivot+[sign*(gap/2+.018),0,0],atol=1e-13)
                self.assertAlmostEqual(np.linalg.norm(link_b-link_a),.045,places=12)

    def test_outside_cad_travel_rejected(self):
        for gap in (.020,.105):
            with self.assertRaises(ValueError):transmission_transforms(gap)

    def test_tilt_keeps_fruit_at_same_tool_contact_location(self):
        center=np.array([.2,1.3,1.]);inward=np.array([1.,2.,0.])
        for pitch in (-60.,-30.,0.,30.):
            tf=grasp_pose(center,inward,pitch_deg=pitch)
            np.testing.assert_allclose(tf[:3,:3]@np.array([0.,.093,.037])+tf[:3,3],center,atol=1e-14)
            self.assertAlmostEqual(np.linalg.det(tf[:3,:3]),1.,places=12)


class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.start = np.array([.05, .15, .2, -.1, .3, -.2, .1])
        self.prep = np.array([.24, -.95, .635, -2.063, .339, -1.355, .389])

    def test_original_profiles_retain_exact_commands(self):
        direct = preparation_phases(self.start, self.prep, 'direct')
        self.assertEqual([(p[0], p[1], p[3:]) for p in direct],
                         [('prepare', 6., (.060, 0., 0.))])
        np.testing.assert_array_equal(direct[0][2], [self.start, self.prep])
        # These intermediate configurations were used by the original profile.
        fold = np.array([.05, .15, .635, -2.063, .3, -.2, .1])
        orient = np.array([.24, .15, .635, -2.063, .339, -1.355, .389])
        original = preparation_phases(self.start, self.prep, 'fold_then_turn')
        self.assertEqual([(p[0], p[1], p[3:]) for p in original],
                         [('prepare_fold', 6., (.060, 0., 0.)),
                          ('prepare_orient', 4., (.060, 0., 0.)),
                          ('prepare_turn', 4., (.060, 0., 0.))])
        for phase, expected in zip(original, ([self.start, fold], [fold, orient], [orient, self.prep])):
            np.testing.assert_array_equal(phase[2], expected)

    def test_low_turn_delays_only_lift_and_keeps_path_continuous(self):
        start, prep = self.start.copy(), self.prep.copy()
        phases = preparation_phases(start, prep, 'low_turn_then_lift')
        np.testing.assert_array_equal(phases[0][2][0], start)
        np.testing.assert_array_equal(phases[-1][2][-1], prep)
        for before, after in zip(phases, phases[1:]):
            np.testing.assert_array_equal(before[2][-1], after[2][0])
        for phase in phases[:3]:
            np.testing.assert_array_equal(phase[2][:, 0], [start[0], start[0]])
        turn = phases[2][2]
        np.testing.assert_array_equal(turn[0, 2:], prep[2:])
        np.testing.assert_array_equal(turn[1, 2:], prep[2:])
        self.assertEqual(turn[0, 1], start[1])
        self.assertEqual(turn[1, 1], prep[1])
        lift = phases[-1][2]
        np.testing.assert_array_equal(lift[:, 1:], np.repeat(prep[None, 1:], 2, axis=0))
        self.assertTrue(all(phase[3:] == (.060, 0., 0.) for phase in phases))
        np.testing.assert_array_equal(start, self.start)
        np.testing.assert_array_equal(prep, self.prep)

    def test_tight_fold_reduces_turn_radius_before_extending(self):
        phases = preparation_phases(self.start, self.prep, 'tight_fold_then_turn')
        self.assertEqual([p[0] for p in phases],
                         ['prepare_fold', 'prepare_tight_fold', 'prepare_orient_tight',
                          'prepare_turn_tight', 'prepare_extend'])
        self.assertTrue(all(p[3:] == (.044, 0., 0.) for p in phases))
        np.testing.assert_array_equal(phases[0][2][0], self.start)
        np.testing.assert_array_equal(phases[-1][2][-1], self.prep)
        for before, after in zip(phases, phases[1:]):
            np.testing.assert_array_equal(before[2][-1], after[2][0])
        turn = phases[3][2]
        np.testing.assert_array_equal(turn[:, 2:4], [[1.3, -2.3], [1.3, -2.3]])
        np.testing.assert_array_equal(turn[:, 0], [self.prep[0], self.prep[0]])
        np.testing.assert_array_equal(turn[:, 4:], np.repeat(self.prep[None, 4:], 2, axis=0))
        self.assertEqual(turn[0, 1], self.start[1])
        self.assertEqual(turn[1, 1], self.prep[1])
        # Extension follows the base turn; it changes shoulder/elbow only.
        extend = phases[-1][2]
        np.testing.assert_array_equal(extend[:, [0, 1, 4, 5, 6]],
            np.repeat(self.prep[None, [0, 1, 4, 5, 6]], 2, axis=0))

    def test_wider_approach_preserves_narrow_prefix_and_opens_at_rest(self):
        original = preparation_phases(self.start, self.prep, 'tight_fold_then_turn')
        opened = preparation_phases(self.start, self.prep, 'tight_fold_open_approach')
        self.assertEqual(len(opened), len(original) + 1)
        for old, new in zip(original, opened):
            self.assertEqual(old[:2], new[:2])
            self.assertEqual(old[3:], new[3:])
            np.testing.assert_array_equal(old[2], new[2])
        held = opened[-1]
        self.assertEqual((held[0], held[1], held[3:]),
                         ('prepare_open', 2., (.060, 0., 0.)))
        np.testing.assert_array_equal(held[2], [self.prep, self.prep])
        np.testing.assert_array_equal(opened[-2][2][-1], held[2][0])

    def test_wider_approach_changes_only_opening_and_approach_gap(self):
        # Deterministic IK isolates command-profile regression from solver noise.
        class DeterministicIK:
            world = np.eye(4)
            def ik(self, pose, seed):
                q = np.asarray(seed).copy()
                q[:3] = pose[:3, 3]
                return q, 0., 0.
        kin = DeterministicIK()
        narrow, narrow_meta = plan_motion(kin, [.2, 1.3, 1.], self.start,
                                          'tight_fold_then_turn')
        opened, opened_meta = plan_motion(kin, [.2, 1.3, 1.], self.start,
                                          'tight_fold_open_approach')
        self.assertEqual(narrow_meta['approach_gap_m'], .044)
        self.assertEqual(opened_meta['approach_gap_m'], .060)
        opening_index = next(i for i, p in enumerate(opened) if p[0] == 'prepare_open')
        without_opening = opened[:opening_index] + opened[opening_index + 1:]
        for old, new in zip(narrow, without_opening):
            self.assertEqual(old[:2], new[:2])
            np.testing.assert_array_equal(old[2], new[2])
            self.assertEqual(old[4:], new[4:])
            self.assertEqual(new[3], .060 if new[0] == 'approach' else old[3])
        self.assertEqual(len(narrow), len(without_opening))


if __name__=='__main__':unittest.main()
