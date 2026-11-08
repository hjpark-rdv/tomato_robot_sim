import unittest
from types import SimpleNamespace

import numpy as np

from camera_lift_vantage import lift_scan_transfer


class CameraLiftVantageTests(unittest.TestCase):
    def setUp(self):
        self.kin = SimpleNamespace(bounds=np.array([[0.] + [-6.] * 6, [.75] + [6.] * 6]))
        self.start = np.array([.0275645945, 1.18, .23, 2.4985, .4362, 2.9869, 3.1404])

    def test_raise_is_real_lift_only_with_measured_start(self):
        original = self.start.copy()
        phases, proof = lift_scan_transfer(self.kin, self.start, .17723615845)
        name, duration, qs, gap, left, right = phases[0]
        self.assertEqual((name, duration, gap, left, right),
                         ('camera_lift_vantage', 6., .096, 0., 0.))
        np.testing.assert_array_equal(qs[0], original)
        np.testing.assert_array_equal(qs[:, 1:], np.array([original[1:], original[1:]]))
        self.assertEqual(qs[-1, 0], .17723615845)
        np.testing.assert_array_equal(self.start, original)
        self.assertFalse(proof['physical_initial_state_changed'])
        self.assertFalse(proof['collision_certified'])

    def test_lower_uses_existing_lower_transfer_without_hidden_raise(self):
        start = self.start.copy(); start[0] = .4
        phases, _ = lift_scan_transfer(self.kin, start, .1)
        self.assertEqual(len(phases), 1)
        np.testing.assert_array_equal(phases[0][2][0], start)
        self.assertEqual(phases[0][2][-1, 0], .1)

    def test_equal_height_has_no_physical_motion(self):
        phases, proof = lift_scan_transfer(self.kin, self.start, self.start[0])
        self.assertEqual(phases, [])
        self.assertTrue(proof['unchanged'])
        self.assertEqual(proof['duration_s'], 0.)

    def test_invalid_height_and_limits_never_clamp(self):
        for height in (float('nan'), float('inf'), True, -.001, .751):
            with self.subTest(height=height), self.assertRaises(ValueError):
                lift_scan_transfer(self.kin, self.start, height)
        bad = self.start.copy(); bad[4] = 6.01
        with self.assertRaises(ValueError):
            lift_scan_transfer(self.kin, bad, .2)


if __name__ == '__main__':
    unittest.main()
