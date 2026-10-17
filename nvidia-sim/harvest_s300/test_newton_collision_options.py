"""CPU-only CCD option guards and collision profiler hook routing."""
import contextlib
import io
import unittest
from unittest import mock

import run_newton_harvest as runner
import newton_gpu_profile as profile


class CollisionOptionsTests(unittest.TestCase):
    def test_default_retains_native_block_size_and_each_explicit_size_is_accepted(self):
        base = ['/tmp/unused-inputs', '--output', '/tmp/unused-output', '--backend', 'gpu']
        self.assertIsNone(runner.arguments(base).convex_ccd_block_dim)
        for value in (32, 64, 128, 256):
            self.assertEqual(runner.arguments(base + ['--convex-ccd-block-dim', str(value)]).convex_ccd_block_dim, value)

    def test_unsupported_values_and_cpu_backend_are_rejected(self):
        base = ['/tmp/unused-inputs', '--output', '/tmp/unused-output']
        for extra in (['--convex-ccd-block-dim', '32'],
                      ['--backend', 'cpu', '--convex-ccd-block-dim', '64'],
                      *[['--backend', 'gpu', '--convex-ccd-block-dim', value]
                        for value in ('0', '-1', '16', '48', '512', '32.0')]):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()), \
                 self.assertRaises(SystemExit) as error:
                runner.arguments(base + extra)
            self.assertEqual(error.exception.code, 2)

    def test_collision_subphase_aliases_are_model_scoped_and_restored(self):
        names = ('sap_broadphase', 'nxn_broadphase', 'convex_narrowphase', 'primitive_narrowphase')
        selected, other = object(), object()
        with contextlib.ExitStack() as stack:
            original = {name: stack.enter_context(mock.patch.object(
                profile.collision_driver, name, return_value=name)) for name in names}
            stack.enter_context(mock.patch.object(profile.wp, 'Event', side_effect=lambda *a, **k: object()))
            record = stack.enter_context(mock.patch.object(profile.wp, 'record_event'))
            stack.enter_context(mock.patch.object(profile.wp, 'synchronize_device'))
            stack.enter_context(mock.patch.object(profile.wp, 'get_event_elapsed_time', return_value=.125))
            measured = profile.GraphEventProfile(selected, 'cpu')
            try:
                for name in names:
                    func = getattr(profile.collision_driver, name)
                    self.assertEqual(func(other, 'data', 'context'), name)
                    self.assertEqual(record.call_count, 0)
                    self.assertEqual(func(selected, 'data', 'context'), name)
                    self.assertEqual(record.call_count, 2)
                    record.reset_mock()
                    original[name].assert_has_calls([
                        mock.call(other, 'data', 'context'), mock.call(selected, 'data', 'context')])
                report = measured.read()
                for name in names:
                    self.assertEqual(report['collision_driver.'+name], .125)
                self.assertNotIn('collision_driver.collision', report)
            finally:
                measured.close()
            for name in names:
                self.assertIs(getattr(profile.collision_driver, name), original[name])


if __name__ == '__main__':
    unittest.main()
