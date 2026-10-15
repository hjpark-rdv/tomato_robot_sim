"""CPU health/presentation classification without running a simulation."""
import copy
import contextlib
import io
from types import SimpleNamespace
import unittest

import numpy as np

from newton_cpu import native_state_finite, record_display_error
from run_newton_harvest import arguments


class NativeGuardTests(unittest.TestCase):
    def test_default_backend_is_the_validated_cpu_path(self):
        self.assertEqual(arguments(['inputs','--output','out']).backend,'cpu')

    def test_gpu_rejects_cpu_options_instead_of_silently_ignoring_them(self):
        for option in (['--solver','CG'],['--solver-iterations','1'],
                       ['--solver-tolerance','1e-6'],['--jacobian','dense'],
                       ['--profile-timers'],['--threads','4'],
                       ['--native-observables-only'],['--start-gate','gate'],
                       ['--ready-file','ready']):
            with self.subTest(option=option),contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    arguments(['inputs','--output','out','--backend','gpu',*option])
                self.assertEqual(error.exception.code,2)

    def state(self):
        return SimpleNamespace(time=1., qpos=np.zeros(2), qvel=np.zeros(2), qacc=np.zeros(2))

    def test_nonfinite_acceleration_is_rejected_before_coordinates_become_nonfinite(self):
        state = self.state()
        self.assertTrue(native_state_finite(state))
        state.qacc[1] = np.inf
        self.assertFalse(native_state_finite(state))
        self.assertTrue(np.isfinite(state.qpos).all() and np.isfinite(state.qvel).all())
        state.qacc[1] = np.nan
        self.assertFalse(native_state_finite(state))

    def test_nonfinite_time_or_coordinates_are_rejected(self):
        for name in ('time', 'qpos', 'qvel'):
            state = self.state()
            if name == 'time':
                state.time = np.nan
            else:
                getattr(state, name)[0] = np.nan
            self.assertFalse(native_state_finite(state), name)

    def test_finite_large_acceleration_is_diagnostic_not_an_arbitrary_hard_gate(self):
        state = self.state()
        state.qacc[0] = 1.e20
        self.assertTrue(native_state_finite(state))

    def test_display_failure_does_not_reclassify_completed_physics_or_its_stop(self):
        report = dict(status='completed', stop=None, actual_steps=30, detached_per_world=[6])
        before = copy.deepcopy(report)
        evidence = record_display_error(report, 'final_frame', TimeoutError('viewer deadline'), simulated_s=1.)
        for key, value in before.items():
            self.assertEqual(report[key], value)
        self.assertFalse(report['gui_live_verified'])
        self.assertEqual(report['gui_status'], 'display_failed')
        self.assertEqual(evidence['stage'], 'final_frame')
        self.assertEqual(evidence['simulated_s'], 1.)


if __name__ == '__main__':
    unittest.main()
