"""Keep interrupted numerical trials out of complete-harvest speed claims."""
import importlib.util
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location(
    'alternative_solver_convergence',
    Path(__file__).with_name('alternative_solver_convergence.py'))
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


class ConvergenceEvidence(unittest.TestCase):
    def test_truncated_or_invalid_run_has_no_full_speedup(self):
        good = dict(full_existing_sequence_physically_valid=True, realtime_factor=2.)
        truncated = dict(full_existing_sequence_physically_valid=False, realtime_factor=8.)
        self.assertIsNone(comparison.qualified_full_speedup(good, truncated))
        self.assertIsNone(comparison.qualified_full_speedup(truncated, good))
        self.assertEqual(comparison.qualified_full_speedup(good, good), 1.)

    def test_partial_and_unexecuted_phases_are_not_speedup_comparisons(self):
        def row(steps, seconds, rtf):
            return dict(steps=steps, simulated_s=seconds, wall_s=seconds/rtf,
                        realtime_factor=rtf)
        reference = dict(close=row(800, 2., .8), open=row(400, 1., .9),
                         withdraw=row(600, 1.5, 1.4))
        trial = dict(close=row(800, 2., 1.), open=row(310, .775, .86))
        rows = comparison.compare_phase_performance(reference, trial)
        self.assertEqual(rows[0]['speedup'], 1.25)
        self.assertIsNone(rows[1]['speedup'])
        self.assertFalse(rows[1]['equal_executed_duration'])
        self.assertEqual(rows[1]['candidate_steps'], 310)
        self.assertIsNone(rows[2]['speedup'])
        self.assertIsNone(rows[2]['candidate_RTF'])
        self.assertEqual(rows[2]['candidate_steps'], 0)


if __name__ == '__main__':
    unittest.main()
