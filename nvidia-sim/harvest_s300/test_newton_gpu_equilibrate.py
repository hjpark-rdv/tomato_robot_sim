"""Same-equation equilibration controls; CUDA requires explicit opt-in."""
import os
from types import SimpleNamespace
import unittest

import numpy as np
import warp as wp

import newton_gpu_equilibrate as eq
from test_newton_gpu_cusolver import context, dummy_solver


def ill_conditioned_system(nv, n):
    rng = np.random.default_rng(502)
    z = rng.normal(size=(nv, nv)) / np.sqrt(nv)
    core = z @ z.T + np.eye(nv) * 2.
    diagonal = np.logspace(-4, 4, nv)
    matrix = (diagonal[:, None] * core * diagonal[None, :]).astype(np.float32)
    grad = np.r_[diagonal * rng.normal(size=nv), np.zeros(n - nv)].astype(np.float32)
    h = np.full((n, n), np.nan, np.float32)
    h[:nv, :nv] = np.triu(matrix) + np.tril(np.full((nv, nv), np.nan, np.float32), -1)
    return matrix, h, grad


def scratch(n, device):
    maximum = np.iinfo(np.int32).max
    return SimpleNamespace(scale=wp.empty(n, dtype=wp.float64, device=device),
        bad_diagonal=wp.array([maximum], dtype=wp.int32, device=device),
        bad_matrix=wp.array([maximum], dtype=wp.int32, device=device),
        factor=wp.empty((n, n), dtype=wp.float32, device=device),
        rhs=wp.empty(n, dtype=wp.float32, device=device),
        active=wp.zeros(1, dtype=wp.bool, device=device),
        info=wp.zeros(1, dtype=wp.int32, device=device),
        failure=wp.zeros(2, dtype=wp.int32, device=device))


def prepare(ctx, work, nv, n, device, skip_noflip=False):
    wp.launch(eq._scales, n, inputs=[nv, ctx.h, ctx.grad, ctx.done,
        ctx.state_changed_count, skip_noflip, work.scale, work.bad_diagonal, work.bad_matrix], device=device)
    wp.launch(eq._prepare_equilibrated, (n, n), inputs=[nv, ctx.h, ctx.grad,
        ctx.done, ctx.state_changed_count, skip_noflip, work.scale,
        work.factor, work.rhs, work.active, work.bad_matrix], device=device)


class EquilibrationCpuControls(unittest.TestCase):
    def test_same_equations_original_norm_and_source_preservation(self):
        nv, n = 5, 8
        a, h, g = ill_conditioned_system(nv, n)
        ctx = context(h, g, nv)
        work = scratch(n, "cpu")
        prepare(ctx, work, nv, n, "cpu")
        factor = work.factor.numpy()
        scaled = np.triu(factor) + np.triu(factor, 1).T
        np.testing.assert_allclose(np.diag(scaled), 1., rtol=2e-7)
        self.assertGreater(np.linalg.cond(a.astype(np.float64)), 1e12)
        self.assertLess(np.linalg.cond(scaled), 10.)
        y = np.linalg.solve(scaled.astype(np.float64), work.rhs.numpy().astype(np.float64)).astype(np.float32)
        wp.copy(work.rhs, wp.array(y, dtype=wp.float32, device="cpu"))
        wp.launch(eq._finalize_equilibrated(nv), 1, inputs=[ctx.grad, work.rhs,
            work.scale, work.active, work.info, work.info, work.failure,
            work.bad_diagonal, work.bad_matrix, ctx.search, ctx.search_dot,
            ctx.newton_decrement], device="cpu")
        x = -ctx.search.numpy()[0].astype(np.float64)
        residual = np.linalg.norm(a.astype(np.float64) @ x - g[:nv]) / np.linalg.norm(g[:nv])
        self.assertLess(residual, 1e-5)
        np.testing.assert_allclose(ctx.search_dot.numpy()[0], x @ x, rtol=2e-6)
        np.testing.assert_allclose(ctx.newton_decrement.numpy()[0], g[:nv] @ x, rtol=2e-6)
        np.testing.assert_array_equal(ctx.h.numpy()[0], h)
        np.testing.assert_array_equal(ctx.grad.numpy()[0], g)

    def test_nonpositive_diagonal_is_rejected_without_jitter_or_clamp(self):
        nv, n = 5, 8
        _, h, g = ill_conditioned_system(nv, n)
        h[2, 2] = 0.
        ctx = context(h, g, nv)
        work = scratch(n, "cpu")
        prepare(ctx, work, nv, n, "cpu")
        self.assertEqual(int(work.bad_diagonal.numpy()[0]), 2)
        self.assertTrue(np.isnan(work.scale.numpy()[2]))
        self.assertEqual(ctx.h.numpy()[0, 2, 2], 0.)

    def test_done_or_noflip_does_not_inspect_invalid_unused_hessian(self):
        for done, changed in [(True, 1), (False, 0)]:
            ctx = context(np.full((8, 8), np.nan, np.float32),
                np.full(8, np.nan, np.float32), 5, done=done, changed=changed)
            work = scratch(8, "cpu")
            prepare(ctx, work, 5, 8, "cpu", True)
            np.testing.assert_array_equal(work.factor.numpy(), np.eye(8))
            np.testing.assert_array_equal(work.rhs.numpy(), np.zeros(8))
            self.assertFalse(work.active.numpy()[0])
            self.assertEqual(int(work.bad_diagonal.numpy()[0]), np.iinfo(np.int32).max)


@unittest.skipUnless(os.environ.get("S300_CUSOLVER_CUDA_TEST") == "1", "Explicit CUDA test opt-in required")
class EquilibrationCudaControls(unittest.TestCase):
    def test_illconditioned_spd_gpu_solve_capture_and_original_residual(self):
        device = wp.get_device("cuda:0")
        nv, n = 65, 80
        a, h, g = ill_conditioned_system(nv, n)
        oracle = np.linalg.solve(a.astype(np.float64), g[:nv].astype(np.float64))
        ctx = context(h, g, nv, device)
        solver = dummy_solver(nv, n, device)
        data = SimpleNamespace(nworld=1, nefc=wp.ones(1, dtype=wp.int32, device=device))
        with eq.EquilibratedCuSolverCholesky(solver) as backend:
            backend.factorize_solve(solver.mjw_model, data, ctx)
            backend.check_health()
            with wp.ScopedCapture(device=device) as capture:
                backend.factorize_solve(solver.mjw_model, data, ctx)
            wp.capture_launch(capture.graph)
            backend.check_health()
            x = -ctx.search.numpy()[0].astype(np.float64)
            residual = np.linalg.norm(a.astype(np.float64) @ x - g[:nv]) / np.linalg.norm(g[:nv])
            self.assertLess(residual, 2e-5)
            np.testing.assert_allclose(x, oracle, rtol=5e-4, atol=1e-5)
            np.testing.assert_allclose(ctx.search_dot.numpy()[0], x @ x, rtol=3e-6)
            np.testing.assert_allclose(ctx.newton_decrement.numpy()[0], g[:nv] @ x, rtol=3e-6)
            np.testing.assert_array_equal(ctx.h.numpy()[0], h)
            np.testing.assert_array_equal(ctx.grad.numpy()[0], g)
            bad = h.copy()
            bad[3, 3] = -1.
            wp.copy(ctx.h, wp.array(bad[None], dtype=wp.float32, device=device))
            wp.capture_launch(capture.graph)
            with self.assertRaisesRegex(RuntimeError, "diagonal_dof=3"):
                backend.check_health()
            self.assertTrue(np.isnan(ctx.search.numpy()[0, 0]))


if __name__ == "__main__":
    unittest.main()
