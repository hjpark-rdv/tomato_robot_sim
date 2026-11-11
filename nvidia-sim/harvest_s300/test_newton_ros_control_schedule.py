"""Exercise live-buffer schedule selection without creating a physics engine."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

import run_newton_harvest as runner


class ReachedModelLoad(Exception):
    """Stop the real runner immediately before any model/GPU initialization."""


class OnlineScheduleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.inputs = self.root/'inputs'
        self.inputs.mkdir()
        self.description = dict(hz=220., free_hz=220., online_control=True, buffer_steps=2)
        self.timeline = dict(time_edges=np.arange(3)/220., time=np.arange(1, 3)/220.,
            dt=np.full(2, 1./220), command=np.zeros((2, 10)), phase=np.array(['ros_control']*2),
            belt_surface_velocity_local_y_m_s=np.zeros((2, 2)))
        np.savez(self.inputs/'initial_state.npz', runtime_dof_damping=np.zeros(10),
                 qpos=np.zeros(10), robot_qpos_addresses=np.arange(10))
        (self.inputs/'source_export.json').write_text(json.dumps(
            dict(active_window=dict(all_plants_static=True))))
        self.run_number = 0

    def run_until_model(self, options=(), error=ReachedModelLoad):
        self.run_number += 1
        output = self.root/f'run_{self.run_number}'
        (self.inputs/'inputs.json').write_text(json.dumps(dict(timeline=self.description)))
        np.savez(self.inputs/'commands.npz', **self.timeline)
        args = runner.arguments([str(self.inputs), '--output', str(output), '--backend', 'gpu',
            '--native-observables-only', '--recording', 'summary', '--batch', '2',
            '--hz', '220', '--free-hz', '220', *options])
        loader = Mock(side_effect=ReachedModelLoad)
        with patch.object(runner, 'verify_prepared_inputs', return_value=(self.inputs/'input_model.mjb', {}, {})), \
                patch.object(runner, 'model_identity', return_value={}), \
                patch.object(runner, 'command_timeline', side_effect=AssertionError('Saved patrol plan must not be resampled')) as sampler, \
                patch.object(runner.mj, 'MjModel', Mock(from_binary_path=loader)):
            with self.assertRaises(error):
                runner.run(args, online_session=object())
            sampler.assert_not_called()
            if error is ReachedModelLoad:
                loader.assert_called_once()
            else:
                loader.assert_not_called()
        return output

    def test_live_buffer_reaches_model_loading_without_reconstructing_saved_plan(self):
        output = self.run_until_model()
        with np.load(output/'executed_commands.npz') as actual:
            self.assertEqual(actual['command'].shape, (2, 10))
            for name, value in self.timeline.items():
                np.testing.assert_array_equal(actual[name], value)

    def test_frequency_or_belt_resampling_rejected_before_model_or_sampler(self):
        for options in (['--free-hz', '60'], ['--hz', '400'], ['--belt-hold-hz', '60']):
            with self.subTest(options=options):
                self.run_until_model(options, error=ValueError)

    def test_invalid_buffer_rejected_before_model_or_sampler(self):
        original = dict(self.timeline)
        for key, value in [('command', np.zeros((3, 10))), ('time', np.arange(3)/220.),
                           ('dt', np.full(2, 1./60)), ('time_edges', np.arange(4)/220.)]:
            with self.subTest(key=key):
                self.timeline = dict(original, **{key: value})
                self.run_until_model(error=ValueError)
        self.timeline = original
        self.description['online_control'] = False
        self.run_until_model(error=ValueError)

    def test_truncated_buffer_rejected_before_model(self):
        self.run_until_model(['--seconds', '0.004'], error=ValueError)


if __name__ == '__main__':
    unittest.main()
