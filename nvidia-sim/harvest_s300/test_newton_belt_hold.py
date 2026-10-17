"""CPU regressions for the opt-in stationary hold timestep experiment."""
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from newton_inputs import DOF_NAMES, command_timeline
from plan_adapter import sample_plan
import run_newton_harvest as runner


class BeltHoldTimelineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        manifest = self.root / 'manifest.json'
        manifest.write_text(json.dumps(dict(rail=dict(initial_station_y_m=1.5))))
        self.meta = dict(source_trial=str(self.root), source_manifest=str(manifest))
        self.initial = np.zeros(len(DOF_NAMES))
        q = np.linspace(.02, 2., 7).tolist()
        self.plan = dict(settle_seconds=0., initial_command=self.initial.tolist(), phases=[
            dict(name=name, duration_s=duration, waypoint_q=[q.copy(), q.copy()],
                 end_gap_m=gap, belt_local_y_m_s=[.05, .05],
                 rail_waypoint_positions_m=[.27, .27])
            for name, duration, gap in [('prepare_open', .065, .096), ('close', .15, .028),
                ('belt_finish', 2., .028), ('cycle_open_final', .125, .096), ('hold', .07, .096)]])
        self.save()

    def save(self):
        (self.root / 'plan.json').write_text(json.dumps(self.plan))

    def sample(self, **kwargs):
        return command_timeline(self.meta, self.initial, 220, free_hz=60, **kwargs)

    def assert_timeline_bytes_equal(self, left, right):
        self.assertEqual(left.keys(), right.keys())
        for key in left:
            self.assertEqual(left[key].dtype, right[key].dtype, key)
            self.assertEqual(left[key].shape, right[key].shape, key)
            self.assertEqual(left[key].tobytes(), right[key].tobytes(), key)

    def test_default_and_none_preserve_all_bytes_and_description(self):
        baseline, description, _ = self.sample()
        actual, evidence, _ = self.sample(belt_hold_hz=None)
        self.assert_timeline_bytes_equal(baseline, actual)
        self.assertEqual(description, evidence)
        self.assertNotIn('belt_hold_frequency', evidence)

    def test_only_hold_rows_change_at_160_and_180_with_exact_outside_rows_and_endpoints(self):
        baseline, _, _ = self.sample()
        outside = baseline['phase'] != 'belt_finish'
        row_fields = ('time', 'dt', 'source_sample_time_s', 'source_command_record_time_s',
                      'command', 'phase', 'authored_belt_speed', 'belt_surface_velocity_local_y_m_s')
        for hz in (160, 180):
            with self.subTest(hz=hz):
                actual, description, plan = self.sample(belt_hold_hz=hz)
                other = actual['phase'] != 'belt_finish'
                for key in row_fields:
                    self.assertEqual(actual[key][other].tobytes(), baseline[key][outside].tobytes(), key)
                self.assertEqual(actual['time_edges'][-1], baseline['time_edges'][-1])
                selected = np.flatnonzero(actual['phase'] == 'belt_finish')
                original = np.flatnonzero(~outside)
                self.assertEqual(len(selected), 2 * hz)
                self.assertEqual(actual['time_edges'][selected[0]], baseline['time_edges'][original[0]])
                self.assertEqual(actual['time_edges'][selected[-1] + 1], baseline['time_edges'][original[-1] + 1])
                np.testing.assert_allclose(actual['dt'][selected], 1 / hz, rtol=0, atol=1e-15)
                for i in selected:
                    sampled = sample_plan(plan, actual['source_sample_time_s'][i])
                    np.testing.assert_array_equal(actual['command'][i], sampled['command'])
                    np.testing.assert_array_equal(actual['authored_belt_speed'][i], sampled['authored_belt_speed'])
                self.assertNotIn('time_edges_master_ticks', actual)
                self.assertIsNone(description['master_hz'])
                self.assertEqual(description['steps'], len(actual['time']))
                proof = description['belt_hold_frequency']
                self.assertTrue(proof['diagnostic'])
                self.assertFalse(proof['accuracy_qualified'])
                self.assertEqual(proof['graph_timesteps_s'], np.unique(np.round(actual['dt'], 12)).tolist())

    def test_non_grid_hold_uses_equal_steps_without_tiny_remainder(self):
        self.plan['phases'][2]['duration_s'] = .103
        self.save()
        baseline, _, _ = self.sample()
        actual, description, _ = self.sample(belt_hold_hz=160)
        hold = actual['dt'][actual['phase'] == 'belt_finish']
        reference = baseline['dt'][baseline['phase'] == 'belt_finish'].sum()
        self.assertEqual(len(hold), math.ceil(reference * 160 - 1e-10))
        self.assertLessEqual(hold.max(), 1 / 160 + 1e-15)
        self.assertLess(float(np.ptp(hold)), 1e-15)
        self.assertEqual(actual['time_edges'][-1], baseline['time_edges'][-1])
        self.assertGreaterEqual(description['belt_hold_frequency']['intervals'][0]['effective_hz'], 160)

    def test_equal_contact_frequency_is_byte_identical(self):
        baseline, _, _ = self.sample()
        actual, description, _ = self.sample(belt_hold_hz=220)
        self.assert_timeline_bytes_equal(baseline, actual)
        self.assertFalse(description['belt_hold_frequency']['timestep_changed'])

    def test_source_start_offset_preserves_existing_sampler_and_other_rows(self):
        baseline, _, _ = self.sample(start_s=.031)
        actual, _, plan = self.sample(start_s=.031, belt_hold_hz=160)
        outside, other = baseline['phase'] != 'belt_finish', actual['phase'] != 'belt_finish'
        for key in ('time', 'source_sample_time_s', 'command', 'dt'):
            self.assertEqual(baseline[key][outside].tobytes(), actual[key][other].tobytes())
        for i in np.flatnonzero(~other):
            np.testing.assert_array_equal(actual['command'][i], sample_plan(plan, actual['time'][i] + .031)['command'])

    def test_moving_arm_rail_jaw_and_stopped_belt_are_rejected(self):
        original = copy.deepcopy(self.plan)
        for field in ('arm', 'rail', 'jaw', 'belt'):
            self.plan = copy.deepcopy(original)
            row = self.plan['phases'][2]
            if field == 'arm': row['waypoint_q'][1][0] += 1e-9
            elif field == 'rail': row['rail_waypoint_positions_m'][1] += 1e-9
            elif field == 'jaw': row['end_gap_m'] += 1e-9
            else: row['belt_local_y_m_s'] = [0., 0.]
            self.save()
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'belt_finish'):
                self.sample(belt_hold_hz=160)

    def test_invalid_rate_recorded_and_frozen_combinations_are_rejected(self):
        for hz in (0, -1, 221, np.nan, np.inf):
            with self.subTest(hz=hz), self.assertRaises(ValueError):
                self.sample(belt_hold_hz=hz)
        for option in (dict(command_source='recorded'), dict(freeze_phase='belt_finish')):
            with self.assertRaisesRegex(ValueError, 'continuous plan'):
                self.sample(belt_hold_hz=160, **option)

    def test_diagnostic_ending_before_hold_keeps_original_timeline(self):
        baseline, _, _ = self.sample(seconds=.1)
        actual, description, _ = self.sample(seconds=.1, belt_hold_hz=160)
        self.assert_timeline_bytes_equal(baseline, actual)
        self.assertEqual(description['belt_hold_frequency']['intervals'], [])

    def test_belt_ramp_is_sampled_from_existing_plan(self):
        self.plan['belt_ramp_seconds'] = .5
        self.plan['belt_cycle'] = dict(start_after_settle_s=.215, duration_s=2., phases=['belt_finish'])
        self.save()
        actual, _, plan = self.sample(belt_hold_hz=160)
        selected = np.flatnonzero(actual['phase'] == 'belt_finish')
        self.assertGreater(np.ptp(actual['authored_belt_speed'][selected, 0]), .01)
        for i in selected:
            np.testing.assert_array_equal(actual['authored_belt_speed'][i],
                sample_plan(plan, actual['source_sample_time_s'][i])['authored_belt_speed'])


class BeltHoldOptionsTests(unittest.TestCase):
    def test_gpu_option_default_and_positive_rates(self):
        base = ['/tmp/inputs', '--output', '/tmp/output', '--backend', 'gpu', '--hz', '220']
        self.assertIsNone(runner.arguments(base).belt_hold_hz)
        for hz in (160, 180, 160.5, 220):
            self.assertEqual(runner.arguments(base + ['--belt-hold-hz', str(hz)]).belt_hold_hz, hz)

    def test_adaptive_cpu_nonfinite_and_excess_rates_rejected(self):
        base = ['/tmp/inputs', '--output', '/tmp/output', '--backend', 'gpu', '--hz', '220']
        for flags in (['--belt-hold-hz', '160', '--adaptive-hz'],
                      ['--belt-hold-hz', '160', '--backend', 'cpu'],
                      *[['--belt-hold-hz', str(x)] for x in (0, -1, 221, 'nan', 'inf')]):
            with self.subTest(flags=flags), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                runner.arguments(base + flags)


if __name__ == '__main__':
    unittest.main()
