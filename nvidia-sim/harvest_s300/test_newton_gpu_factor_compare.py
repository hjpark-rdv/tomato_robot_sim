"""CPU-only input/oracle/failure-report controls for the factor microbenchmark."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

import bench_newton_gpu_factor_compare as bench


class FactorComparisonControls(unittest.TestCase):
    def args(self, *extra):
        return bench.arguments(['--output', '/tmp/unused-factor-comparison.json', *extra])

    def test_default_same_169_spd_and_native_padding(self):
        args = self.args()
        a, h, grad, oracle = bench.inputs(args)
        self.assertEqual((a.shape, h.shape, grad.shape), ((169, 169), (176, 176), (176,)))
        self.assertEqual(args.backends, list(bench.BACKENDS))
        self.assertFalse(args.include_done)
        np.testing.assert_allclose(a @ oracle, grad[:169], rtol=1e-12, atol=1e-12)
        np.testing.assert_array_equal(h[:169, 169:], 0.)
        np.testing.assert_array_equal(grad[169:], 0.)
        _, again, g_again, _ = bench.inputs(args)
        np.testing.assert_array_equal(h.view(np.uint32), again.view(np.uint32))
        np.testing.assert_array_equal(grad.view(np.uint32), g_again.view(np.uint32))

    def test_scaled_system_is_spd_and_float32_actual_input_is_the_oracle(self):
        args = self.args('--diagonal-span', '1000000')
        a, h, grad, oracle = bench.inputs(args)
        self.assertGreater(np.linalg.cond(a), 1e5)
        self.assertEqual(h.dtype, np.float32)
        upper = np.triu(h[:args.nv, :args.nv])
        np.testing.assert_array_equal(a, upper + upper.T - np.diag(upper.diagonal()))
        np.testing.assert_allclose(a @ oracle, grad[:args.nv], rtol=1e-10, atol=1e-10)

    def test_saved_hessian_uses_exact_upper_and_rhs_and_rejects_nonzero_padding(self):
        a, h, g, _ = bench.inputs(self.args())
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'h.npz'
            np.savez(path, h=h[None], grad=g[None])
            actual, loaded, rhs, _ = bench.inputs(self.args('--hessian', str(path)))
            np.testing.assert_array_equal(actual, a)
            np.testing.assert_array_equal(loaded.view(np.uint32), h.view(np.uint32))
            np.testing.assert_array_equal(rhs.view(np.uint32), g.view(np.uint32))
            with self.assertRaisesRegex(ValueError, 'scaling'):
                bench.inputs(self.args('--hessian', str(path), '--diagonal-span', '10'))
            h[0, -1] = 1.
            np.savez(path, h=h, grad=g)
            with self.assertRaisesRegex(ValueError, 'padding'):
                bench.inputs(self.args('--hessian', str(path)))

    def test_nonfinite_or_non_spd_input_is_not_benchmarked(self):
        _, h, g, _ = bench.inputs(self.args())
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'h.npz'
            for value, error in ((float('nan'), ValueError), (-1., np.linalg.LinAlgError)):
                h[0, 0] = value
                np.savez(path, h=h, grad=g)
                with self.assertRaises(error):
                    bench.inputs(self.args('--hessian', str(path)))

    def test_accuracy_rejects_bad_and_nonfinite_solution_without_hiding_failure(self):
        args = self.args()
        a = np.array([[2., .1], [.1, 3.]])
        grad = np.array([1., 2.])
        oracle = np.linalg.solve(a, grad)
        good = bench.accuracy(a, grad, oracle, -oracle, oracle@oracle, grad@oracle, args)
        self.assertTrue(good['numerically_valid'])
        bad = bench.accuracy(a, grad, oracle, -oracle*2, 1., 1., args)
        self.assertFalse(bad['numerically_valid'])
        self.assertGreater(bad['relative_residual'], .9)
        bad = bench.accuracy(a, grad, oracle, -oracle, 1., 1., args)
        self.assertFalse(bad['numerically_valid'])
        self.assertGreater(bad['search_dot_relative_error'], args.max_relative_solution_error)
        bad = bench.accuracy(a, grad, oracle, [float('nan'), 0.], 1., 1., args)
        self.assertFalse(bad['numerically_valid'])
        self.assertFalse(bad['finite'])
        json.dumps(bench.clean_json({'test': float('nan'), 'bad': bad}), allow_nan=False)

    def test_input_failure_is_saved_before_gpu_access_and_existing_report_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'report.json'
            args = self.args('--output', str(out), '--nv', '16')
            report = bench.benchmark(args)
            self.assertEqual(report['status'], 'failed')
            self.assertEqual(report['error_type'], 'ValueError')
            self.assertEqual(json.loads(out.read_text())['status'], 'failed')
            before = out.read_bytes()
            with self.assertRaises(FileExistsError):
                bench.benchmark(args)
            self.assertEqual(out.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
