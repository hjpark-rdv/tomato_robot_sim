"""Host parser checks for explicit dense-Hessian workspace capacity."""
import contextlib
import io
import unittest

import run_newton_harvest as runner
import run_newton_row_harvest as route


class DenseHessianCapacityOptionsTests(unittest.TestCase):
    def test_existing_default_is_128_for_single_runner_and_route(self):
        single = runner.arguments(['/tmp/unused-inputs', '--output', '/tmp/unused-output',
            '--backend', 'gpu', '--dense-hessian'])
        row = route.arguments(['--prepared-root', '/tmp/unused-route'])
        self.assertEqual(single.dense_hessian_capacity, 128)
        self.assertEqual(row.dense_hessian_capacity, 128)

    def test_capacity_256_is_an_explicit_opt_in_within_njmax(self):
        single = runner.arguments(['/tmp/unused-inputs', '--output', '/tmp/unused-output',
            '--backend', 'gpu', '--dense-hessian', '--njmax', '256',
            '--dense-hessian-capacity', '256'])
        row = route.arguments(['--prepared-root', '/tmp/unused-route', '--njmax', '256',
            '--dense-hessian-capacity', '256'])
        self.assertEqual(single.dense_hessian_capacity, 256)
        self.assertEqual(row.dense_hessian_capacity, 256)

    def test_zero_negative_and_capacity_above_constraint_storage_are_rejected(self):
        for value in ('0', '-1', '257'):
            cases = [
                (runner.arguments, ['/tmp/unused-inputs', '--output', '/tmp/unused-output',
                    '--backend', 'gpu', '--dense-hessian', '--njmax', '256']),
                (route.arguments, ['--prepared-root', '/tmp/unused-route', '--njmax', '256'])]
            for parse, base in cases:
                with self.subTest(value=value, parser=parse.__module__), \
                     contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    parse([*base, '--dense-hessian-capacity', value])
                self.assertEqual(error.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
