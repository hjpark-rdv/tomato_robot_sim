"""Exact H-cache controls; no harvest success claim from these matrix fixtures."""
import os
import unittest
from types import SimpleNamespace
import numpy as np
import warp as wp
from newton_gpu_cusolver import CuSolverCholesky
from newton_gpu_factor_cache import CachedCuSolverCholesky
from test_newton_gpu_cusolver import dummy_solver, context


@unittest.skipUnless(os.getenv('S300_CUSOLVER_CUDA_TEST') == '1', 'Explicit CUDA opt-in required')
class FactorCacheCudaTests(unittest.TestCase):
    def test_live_rhs_changed_h_done_graph_reuse_and_indefinite_failure(self):
        device = wp.get_device('cuda:0')
        with wp.ScopedDevice(device):
            nv, n = 385, 400
            rng = np.random.default_rng(23)
            a = rng.normal(size=(nv, nv)).astype(np.float32) / np.sqrt(nv)
            h = np.eye(n, dtype=np.float32)
            h[:nv, :nv] = a @ a.T + 2 * np.eye(nv)
            grad = rng.normal(size=n).astype(np.float32)
            s = dummy_solver(nv, n, device)
            d = SimpleNamespace(nworld=1, nefc=wp.array([10], dtype=int))
            scenarios = [(h, grad), (h, grad * .3), (h.copy(), grad * -.2)]
            scenarios[2][0][0, 0] = np.nextafter(h[0, 0], np.float32(10))
            expected = []
            ctx = context(h, grad, n, device)
            with CuSolverCholesky(s) as ordinary:
                for matrix, rhs in scenarios:
                    ctx.h.assign(matrix[None]); ctx.grad.assign(rhs[None])
                    ordinary.factorize_solve(s.mjw_model, d, ctx)
                    expected.append((ctx.search.numpy(), ctx.search_dot.numpy(), ctx.newton_decrement.numpy()))
            with CachedCuSolverCholesky(s) as cached:
                with wp.ScopedCapture(device=device) as capture:
                    cached.factorize_solve(s.mjw_model, d, ctx, skip_noflip=True)
                for (matrix, rhs), reference in zip(scenarios, expected):
                    ctx.h.assign(matrix[None]); ctx.grad.assign(rhs[None])
                    wp.capture_launch(capture.graph)
                    cached.check_health()
                    for value, wanted in zip((ctx.search, ctx.search_dot, ctx.newton_decrement), reference):
                        np.testing.assert_array_equal(value.numpy(), wanted)
                self.assertEqual(cached.statistics(), dict(solves=3, factorizations=2, reused=1, inactive_calls=0))
                for done, changed in [(True, 1), (False, 0)]:
                    ctx.done.assign([done]); ctx.state_changed_count.assign([changed])
                    ctx.h.fill_(np.nan); ctx.grad.fill_(np.nan)
                    wp.capture_launch(capture.graph)
                    cached.check_health()
                    np.testing.assert_array_equal(ctx.search.numpy(), expected[-1][0])
                ctx.done.assign([False]); ctx.state_changed_count.assign([1])
                bad = np.eye(n, dtype=np.float32); bad[0, 0] = -1.
                ctx.h.assign(bad[None]); ctx.grad.assign(grad[None])
                wp.capture_launch(capture.graph)
                with self.assertRaisesRegex(RuntimeError, 'potrf_info=1'):
                    cached.check_health()
                self.assertTrue(np.isnan(ctx.search.numpy()[0, 0]))


if __name__ == '__main__':
    unittest.main()
