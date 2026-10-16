"""Multi-world controls; CUDA tests require explicit opt-in and a free GPU."""
import ctypes
import json
import os
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import warp as wp

import newton_gpu_cusolver_batch as batch
from test_newton_gpu_cusolver import dummy_solver


@unittest.skipUnless(os.getenv('S300_CUSOLVER_CUDA_TEST') == '1', 'Explicit CUDA opt-in')
class SingleBatchedApiTests(unittest.TestCase):
    def test_one_world_batched_residual_repeat_and_timing(self):
        device = wp.get_device('cuda:0')
        with wp.ScopedDevice(device):
            n, nv = 400, 385
            rng = np.random.default_rng(71)
            a = rng.normal(size=(nv, nv)).astype(np.float32)/np.sqrt(nv)
            h = np.eye(n, dtype=np.float32)
            h[:nv, :nv] = a @ a.T + 2*np.eye(nv)
            grad = rng.normal(size=(1, n)).astype(np.float32)
            model = dummy_solver(nv, n, device)
            data = SimpleNamespace(nworld=1, nefc=wp.array([10], int))
            oracle = np.linalg.solve(h[:nv, :nv].astype(np.float64), grad[0, :nv])
            times = {}
            for force in (False, True):
                ctx = context(h[None], grad, nv, device)
                with batch.CuSolverCholeskyBatch(model, force_batched_single=force) as backend:
                    backend.factorize_solve(model.mjw_model, data, ctx)
                    backend.check_health()
                    np.testing.assert_allclose(ctx.search.numpy()[0], -oracle, rtol=2e-4, atol=2e-6)
                    reference = ctx.search.numpy()
                    with wp.ScopedCapture(device=device) as cap:
                        for _ in range(100): backend.factorize_solve(model.mjw_model, data, ctx)
                    wp.capture_launch(cap.graph)
                    start, end = wp.Event(device, enable_timing=True), wp.Event(device, enable_timing=True)
                    wp.record_event(start)
                    wp.capture_launch(cap.graph)
                    wp.record_event(end)
                    times['batched' if force else 'regular'] = wp.get_event_elapsed_time(start, end)/100
                    backend.check_health()
                    np.testing.assert_array_equal(ctx.search.numpy(), reference)
            print('SINGLE_API_MATRIX_MS=' + json.dumps(times))


def context(h, grad, nv, device="cpu", done=None, changed=None):
    worlds, n = grad.shape
    return SimpleNamespace(h=wp.array(h, dtype=wp.float32, device=device),
        grad=wp.array(grad, dtype=wp.float32, device=device),
        done=wp.array(np.zeros(worlds, bool) if done is None else done, dtype=wp.bool, device=device),
        state_changed_count=wp.array(np.ones(worlds, np.int32) if changed is None else changed,
                                    dtype=wp.int32, device=device),
        search=wp.full((worlds, nv), 123., dtype=wp.float32, device=device),
        search_dot=wp.full(worlds, 456., dtype=wp.float32, device=device),
        newton_decrement=wp.full(worlds, 789., dtype=wp.float32, device=device))


class CuSolverBatchHostTests(unittest.TestCase):
    def test_pointer_tables_are_disjoint_with_exact_byte_strides(self):
        np.testing.assert_array_equal(batch._pointer_values(4096, 4, 400 * 400 * 4),
            [4096, 644096, 1284096, 1924096])
        for inputs in [(0, 4, 10), (1000, 0, 10), (1000, 4, 0)]:
            with self.assertRaises(ValueError):
                batch._pointer_values(*inputs)

    def test_batched_api_uses_device_pointer_tables_and_one_rhs(self):
        for precision, prefix in [("float32", "S"), ("float64", "D")]:
            backend = object.__new__(batch.CuSolverCholeskyBatch)
            lib = mock.MagicMock()
            getattr(lib, f"cusolverDn{prefix}potrfBatched").return_value = 0
            getattr(lib, f"cusolverDn{prefix}potrsBatched").return_value = 0
            backend.api = SimpleNamespace(lib=lib, checked=mock.Mock())
            backend._configure_batched_api(precision)
            self.assertEqual(backend._potrf_batched.argtypes,
                [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p,
                 ctypes.c_int, ctypes.c_void_p, ctypes.c_int])
            backend.worlds, backend.n = 4, 400
            backend.batch_mode = "batched"
            backend.handle = "handle"
            backend.factor_pointers = SimpleNamespace(ptr=1000)
            backend.rhs_pointers = SimpleNamespace(ptr=2000)
            backend.factor_info = SimpleNamespace(ptr=3000)
            backend.solve_info = SimpleNamespace(ptr=4000)
            backend._factor_and_solve()
            getattr(lib, f"cusolverDn{prefix}potrfBatched").assert_called_once_with(
                "handle", 0, 400, 1000, 400, 3000, 4)
            getattr(lib, f"cusolverDn{prefix}potrsBatched").assert_called_once_with(
                "handle", 0, 400, 1, 1000, 400, 2000, 400, 4000, 4)

    def test_loop_mode_offsets_each_world_without_host_solver(self):
        backend = object.__new__(batch.CuSolverCholeskyBatch)
        backend.api = mock.Mock()
        backend.worlds, backend.n = 2, 400
        backend.batch_mode, backend.dtype = "loop", wp.float32
        backend.handle = "handle"
        backend.factor = SimpleNamespace(ptr=1000)
        backend.rhs = SimpleNamespace(ptr=2000)
        backend.factor_info = SimpleNamespace(ptr=3000)
        backend.solve_info = SimpleNamespace(ptr=4000)
        backend.workspace = SimpleNamespace(ptr=5000)
        backend.workspace_count = 64
        backend._factor_and_solve()
        self.assertEqual(backend.api.factor.call_args_list,
            [mock.call("handle", 400, 1000, 5000, 64, 3000),
             mock.call("handle", 400, 641000, 5000, 64, 3004)])
        self.assertEqual(backend.api.solve.call_args_list,
            [mock.call("handle", 400, 1000, 2000, 4000),
             mock.call("handle", 400, 641000, 3600, 4004)])

    def test_invalid_mode_or_worlds_rejected_before_initialization(self):
        solver = dummy_solver()
        solver.mjw_data.nworld = 0
        with self.assertRaisesRegex(ValueError, "positive"):
            batch.CuSolverCholeskyBatch(solver)
        solver.mjw_data.nworld = 2
        with self.assertRaisesRegex(ValueError, "batch_mode"):
            batch.CuSolverCholeskyBatch(solver, batch_mode="CPU")


class CuSolverBatchCpuControls(unittest.TestCase):
    def test_world_specific_done_noflip_rhs_and_failures(self):
        worlds, nv, n = 4, 3, 4
        h = np.broadcast_to(np.eye(n, dtype=np.float32), (worlds, n, n)).copy()
        h[0, :nv, :nv] *= 2.
        h[1:3] = np.nan
        h[3, :nv, :nv] *= 3.
        grad = np.array([[1., 2., 3., 777.], [np.nan] * n,
                         [np.nan] * n, [4., 5., 6., 777.]], np.float32)
        for dtype in (wp.float32, wp.float64):
            ctx = context(h, grad, nv, done=[False, True, False, False], changed=[1, 1, 0, 1])
            factor = wp.empty((worlds, n, n), dtype=dtype, device="cpu")
            rhs = wp.empty((worlds, n), dtype=dtype, device="cpu")
            active = wp.zeros(worlds, dtype=wp.bool, device="cpu")
            wp.launch(batch._prepare_many(dtype), (worlds, n, n),
                inputs=[nv, ctx.h, ctx.grad, ctx.done, ctx.state_changed_count, True,
                        factor, rhs, active], device="cpu")
            np.testing.assert_array_equal(active.numpy(), [True, False, False, True])
            np.testing.assert_array_equal(factor.numpy()[1:3], np.tile(np.eye(n), (2, 1, 1)))
            np.testing.assert_array_equal(rhs.numpy()[1:3], np.zeros((2, n)))
            solved = rhs.numpy()
            solved[0, :nv] /= 2.
            solved[3, :nv] /= 3.
            wp.copy(rhs, wp.array(solved, dtype=dtype, device="cpu"))
            factor_info = wp.array([0, 0, 0, 2], dtype=wp.int32, device="cpu")
            solve_info = wp.zeros(worlds, dtype=wp.int32, device="cpu")
            failure = wp.zeros(2 * worlds, dtype=wp.int32, device="cpu")
            wp.launch(batch._finalize_many(dtype, nv), worlds,
                inputs=[ctx.grad, rhs, active, factor_info, solve_info, False,
                        failure, ctx.search, ctx.search_dot, ctx.newton_decrement], device="cpu")
            np.testing.assert_allclose(ctx.search.numpy()[0], [-.5, -1., -1.5])
            np.testing.assert_array_equal(ctx.search.numpy()[1:3], np.full((2, nv), 123.))
            self.assertTrue(np.isnan(ctx.search.numpy()[3, 0]))
            np.testing.assert_array_equal(failure.numpy(), [0, 0, 0, 0, 0, 0, 2, 0])

    def test_batched_potrs_argument_error_is_global_and_recorded_per_world(self):
        worlds, n, nv = 2, 4, 3
        ctx = context(np.tile(np.eye(n, dtype=np.float32), (worlds, 1, 1)),
                      np.ones((worlds, n), np.float32), nv)
        rhs = wp.ones((worlds, n), dtype=wp.float32, device="cpu")
        active = wp.ones(worlds, dtype=wp.bool, device="cpu")
        info = wp.zeros(worlds, dtype=wp.int32, device="cpu")
        solve_info = wp.array([-2, 0], dtype=wp.int32, device="cpu")
        failure = wp.zeros(worlds * 2, dtype=wp.int32, device="cpu")
        wp.launch(batch._finalize_many(wp.float32, nv), worlds,
            inputs=[ctx.grad, rhs, active, info, solve_info, True,
                    failure, ctx.search, ctx.search_dot, ctx.newton_decrement], device="cpu")
        np.testing.assert_array_equal(failure.numpy(), [0, -2, 0, -2])
        self.assertTrue(np.isnan(ctx.search.numpy()[:, 0]).all())


@unittest.skipUnless(os.environ.get("S300_CUSOLVER_CUDA_TEST") == "1", "Explicit CUDA test opt-in required")
class CuSolverBatchCudaControls(unittest.TestCase):
    def test_four_independent_worlds_modes_precisions_capture_and_status(self):
        device = wp.get_device("cuda:0")
        worlds, nv, n = 4, 385, 400
        rng = np.random.default_rng(64)
        h = np.zeros((worlds, n, n), np.float32)
        grad = np.zeros((worlds, n), np.float32)
        oracle = []
        for world in range(worlds):
            z = rng.normal(size=(nv, nv)).astype(np.float32) / np.sqrt(nv)
            a = (z @ z.T + np.eye(nv, dtype=np.float32) * (2 + world)).astype(np.float32)
            h[world, :nv, :nv] = np.triu(a)
            grad[world, :nv] = rng.normal(size=nv) * (world + 1)
            oracle.append(np.linalg.solve(a.astype(np.float64), grad[world, :nv].astype(np.float64)))
        oracle = np.asarray(oracle)
        for mode in ("loop", "batched"):
            for precision in ("float32", "float64"):
                with self.subTest(mode=mode, precision=precision):
                    solver = dummy_solver(nv, n, device)
                    solver.mjw_data.nworld = worlds
                    ctx = context(h, grad, nv, device, done=[False, True, False, False], changed=[1, 1, 0, 1])
                    data = SimpleNamespace(nworld=worlds, nefc=wp.ones(worlds, dtype=wp.int32, device=device))
                    with batch.CuSolverCholeskyBatch(solver, precision=precision, batch_mode=mode) as backend:
                        backend.factorize_solve(solver.mjw_model, data, ctx, True, True)
                        backend.check_health()
                        np.testing.assert_allclose(ctx.search.numpy()[[0, 3]], -oracle[[0, 3]], rtol=2e-4, atol=3e-6)
                        np.testing.assert_array_equal(ctx.search.numpy()[1:3], np.full((2, nv), 123.))
                        ctx.done.zero_()
                        ctx.state_changed_count.fill_(1)
                        with wp.ScopedCapture(device=device) as capture:
                            backend.factorize_solve(solver.mjw_model, data, ctx)
                        wp.capture_launch(capture.graph)
                        backend.check_health()
                        np.testing.assert_allclose(ctx.search.numpy(), -oracle, rtol=2e-4, atol=3e-6)
                        # One world's indefinite Hessian must not contaminate
                        # another world or be mislabeled as a global failure.
                        bad = h.copy()
                        bad[2] = np.eye(n, dtype=np.float32)
                        bad[2, 0, 0] = -1.
                        wp.copy(ctx.h, wp.array(bad, dtype=wp.float32, device=device))
                        wp.capture_launch(capture.graph)
                        with self.assertRaisesRegex(RuntimeError, "world=2, potrf_info=1"):
                            backend.check_health()
                        self.assertTrue(np.isnan(ctx.search.numpy()[2, 0]))
                        np.testing.assert_allclose(ctx.search.numpy()[[0, 1, 3]], -oracle[[0, 1, 3]], rtol=2e-4, atol=3e-6)


if __name__ == "__main__":
    unittest.main()
