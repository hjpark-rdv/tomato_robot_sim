"""Validate the shipped finite maximum-search profile against the actual CLI."""
import json
import io
from contextlib import redirect_stderr
from pathlib import Path
import unittest

from camera_plan_alternatives import camera_plan_alternatives
from test_run_motion_conditioning import actual_cli


class MaximumCameraProfileTests(unittest.TestCase):
    def test_profile_is_executable_and_keeps_physical_guards(self):
        arguments = json.loads(Path(__file__).with_name(
            'maximum_camera_patrol_arguments.json').read_text())
        # The existing campaign supplies its exact inventory per active window.
        parsed = actual_cli(arguments + ['--near-trusses-metadata', '/fixture/inventory.json'])
        alternatives = camera_plan_alternatives(parsed.camera_heading_offsets_deg,
            parsed.horizontal_ready_seed_index, seed_indices=parsed.camera_ik_seed_indices)
        self.assertEqual(len(alternatives), 18)
        self.assertLessEqual(1 + len(alternatives), parsed.max_truss_cycles)
        self.assertEqual(parsed.hz, 60)
        self.assertEqual(parsed.contact_limit_mm, 1.)
        self.assertEqual(parsed.fruit_contact_limit_mm, 1.)
        self.assertEqual(parsed.fruit_fruit_limit_mm, 1.)
        self.assertTrue(parsed.camera_harvest)
        self.assertTrue(parsed.capture_gt)
        self.assertEqual(parsed.preparation_gap_mm, 96.)
        self.assertEqual(parsed.single_fruit_when_remaining, 0)

    def test_heading_cap_remains_finite(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            actual_cli(['--camera-heading-offsets-deg', *map(str, range(10))])


if __name__ == '__main__':
    unittest.main()
