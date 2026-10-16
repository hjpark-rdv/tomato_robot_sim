"""CPU controls plus explicitly enabled CUDA numerical/capture controls.

Default invocation launches no CUDA work. Run the CUDA class only after the
device is free: S300_CUSOLVER_CUDA_TEST=1 python -m unittest -v
test_newton_gpu_cusolver.CuSolverCudaControls
"""
import ctypes
import os
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import warp as wp
from mujoco_warp._src import types as mt

import newton_gpu_cusolver as cs


def dummy_solver(nv=385, padded=400, device=None):
    return SimpleNamespace(use_mujoco_cpu=False, _step=0,
        mjw_model=SimpleNamespace(nv=nv, nv_pad=padded,
            opt=SimpleNamespace(solver=mt.SolverType.NEWTON)),
        mjw_data=SimpleNamespace(nworld=1,
            qpos=SimpleNamespace(device=device)))


def context(h, grad, nv, device="cpu", *, done=False, changed=1):
    return SimpleNamespace(h=wp.array(h[None], dtype=wp.float32, device=device),
        grad=wp.array(grad[None], dtype=wp.float32, device=device),
        done=wp.array([done], dtype=wp.bool, device=device),
        state_changed_count=wp.array([changed], dtype=wp.int32, device=device),
        search=wp.full((1, nv), 123., dtype=wp.float32, device=device),
        search_dot=wp.full(1, 456., dtype=wp.float32, device=device),
        newton_decrement=wp.full(1, 789., dtype=wp.float32, device=device))


class CuSolverHostTests(unittest.TestCase):
    def test_invalid_backend_world_or_solver_rejected_before_cuda(self):
        solver = dummy_solver()
        self.assertIs(cs.validate_solver(solver, "float32"), solver.mjw_model)
        for attribute, value, message in [("use_mujoco_cpu", True, "GPU"),
                                           ("_step", 1, "before stepping")]:
            s = dummy_solver()
            setattr(s, attribute, value)
            with self.assertRaisesRegex(ValueError, message):
                cs.validate_solver(s, "float32")
        solver.mjw_data.nworld = 2
        with self.assertRaisesRegex(ValueError, "one world"):
            cs.validate_solver(solver, "float32")
        solver = dummy_solver()
        solver.mjw_model.opt.solver = mt.SolverType.CG
        with self.assertRaisesRegex(ValueError, "Newton constraint"):
            cs.validate_solver(solver, "float32")
        with self.assertRaisesRegex(ValueError, "precision"):
            cs.validate_solver(dummy_solver(), "float16")
        with self.assertRaisesRegex(ValueError, "large blocked"):
            cs.validate_solver(dummy_solver(nv=3, padded=4), "float32")

    def test_missing_library_is_an_error_and_c_api_status_not_ignored(self):
        with self.assertRaises(FileNotFoundError):
            cs.find_cusolver_library("/definitely-missing/libcusolver.so")
        cs.CuSolverAPI.checked(0, "control")
        with self.assertRaisesRegex(RuntimeError, "potrf.*status 7"):
            cs.CuSolverAPI.checked(7, "potrf")

    def test_typed_single_and_double_api_calls_use_lower_column_major(self):
        for precision, prefix in [("float32", "S"), ("float64", "D")]:
            lib = mock.MagicMock()
            for name in ("cusolverDnCreate", "cusolverDnDestroy", "cusolverDnSetStream",
                         f"cusolverDn{prefix}potrf", f"cusolverDn{prefix}potrs"):
                getattr(lib, name).return_value = 0
            def buffer_size(*args):
                ctypes.cast(args[-1], ctypes.POINTER(ctypes.c_int))[0] = 128
                return 0
            getattr(lib, f"cusolverDn{prefix}potrf_bufferSize").side_effect = buffer_size
            with mock.patch.object(cs, "find_cusolver_library", return_value="fake"), \
                 mock.patch.object(cs.ctypes, "CDLL", return_value=lib):
                api = cs.CuSolverAPI(precision=precision)
            handle = ctypes.c_void_p(100)
            self.assertEqual(api.buffer_size(handle, 400, 1000), 128)
            api.factor(handle, 400, 1000, 2000, 128, 3000)
            api.solve(handle, 400, 1000, 4000, 5000)
            getattr(lib, f"cusolverDn{prefix}potrf").assert_called_once_with(
                handle, 0, 400, 1000, 400, 2000, 128, 3000)
            getattr(lib, f"cusolverDn{prefix}potrs").assert_called_once_with(
                handle, 0, 400, 1, 1000, 400, 4000, 400, 5000)

    def test_dispatch_is_limited_to_model_identity(self):
        model = object()
        backend = SimpleNamespace(model=model, factorize_solve=mock.Mock(), original=mock.Mock())
        other = object()
        with mock.patch.object(cs, "_installed_backend", backend):
            cs._dispatch(model, "data", "ctx", True, False)
            cs._dispatch(other, "data2", "ctx2", False, True)
        backend.factorize_solve.assert_called_once_with(model, "data", "ctx", True, False)
        backend.original.assert_called_once_with(other, "data2", "ctx2", False, True)


class CuSolverCpuKernelControls(unittest.TestCase):
    def test_upper_triangle_and_padding_map_to_cusolver_lower_column_major(self):
        nv, n = 3, 4
        h = np.array([[4., 1., .5, 99.], [-999., 3., .25, 99.],
                      [-999., -999., 2., 99.], [99., 99., 99., 99.]], np.float32)
        for dtype in (wp.float32, wp.float64):
            ctx = context(h, np.array([2., -3., 4., 777.], np.float32), nv)
            factor = wp.empty((n, n), dtype=dtype, device="cpu")
            rhs = wp.empty(n, dtype=dtype, device="cpu")
            active = wp.zeros(1, dtype=wp.bool, device="cpu")
            wp.launch(cs._prepare_kernel(dtype), (n, n),
                inputs=[nv, ctx.h, ctx.grad, ctx.done, ctx.state_changed_count, False,
                        factor, rhs, active], device="cpu")
            actual = factor.numpy()
            expected = np.zeros((n, n))
            expected[:nv, :nv] = np.triu(h[:nv, :nv])
            expected[-1, -1] = 1.
            np.testing.assert_array_equal(actual, expected)
            # Reinterpret the same row-major memory exactly as the C API does.
            column_major = actual.ravel().reshape((n, n), order="F")
            lower = np.tril(column_major)
            symmetric = lower + lower.T - np.diag(np.diag(lower))
            oracle = np.array([[4., 1., .5, 0.], [1., 3., .25, 0.],
                               [.5, .25, 2., 0.], [0., 0., 0., 1.]])
            np.testing.assert_array_equal(symmetric, oracle)
            np.testing.assert_array_equal(rhs.numpy(), [2., -3., 4., 0.])
            self.assertTrue(active.numpy()[0])

    def test_sign_norm_and_newton_decrement_match_spd_solution(self):
        matrix = np.array([[4., 1., .5], [1., 3., .25], [.5, .25, 2.]])
        grad = np.array([2., -3., 4., 0.], np.float32)
        solution = np.linalg.solve(matrix, grad[:3])
        for dtype in (wp.float32, wp.float64):
            ctx = context(np.eye(4, dtype=np.float32), grad, 3)
            rhs = wp.array(np.r_[solution, 0.], dtype=dtype, device="cpu")
            active = wp.array([True], dtype=wp.bool, device="cpu")
            info = wp.zeros(1, dtype=wp.int32, device="cpu")
            failure = wp.zeros(2, dtype=wp.int32, device="cpu")
            wp.launch(cs._finalize_kernel(dtype, 3, 4), 1,
                inputs=[ctx.grad, rhs, active, info, info, failure, ctx.search,
                        ctx.search_dot, ctx.newton_decrement], device="cpu")
            np.testing.assert_allclose(ctx.search.numpy()[0], -solution, rtol=1e-6)
            np.testing.assert_allclose(ctx.search_dot.numpy()[0], solution @ solution, rtol=1e-6)
            np.testing.assert_allclose(ctx.newton_decrement.numpy()[0], grad[:3] @ solution, rtol=1e-6)

    def test_done_and_noflip_keep_previous_direction_and_failed_factor_is_visible(self):
        n, nv = 4, 3
        for done, changed in [(True, 1), (False, 0)]:
            ctx = context(np.full((n, n), np.nan, np.float32),
                          np.full(n, np.nan, np.float32), nv, done=done, changed=changed)
            factor = wp.empty((n, n), dtype=wp.float32, device="cpu")
            rhs = wp.empty(n, dtype=wp.float32, device="cpu")
            active = wp.zeros(1, dtype=wp.bool, device="cpu")
            info = wp.zeros(1, dtype=wp.int32, device="cpu")
            failure = wp.zeros(2, dtype=wp.int32, device="cpu")
            wp.launch(cs._prepare_kernel(wp.float32), (n, n),
                inputs=[nv, ctx.h, ctx.grad, ctx.done, ctx.state_changed_count, True,
                        factor, rhs, active], device="cpu")
            np.testing.assert_array_equal(factor.numpy(), np.eye(n))
            np.testing.assert_array_equal(rhs.numpy(), np.zeros(n))
            wp.launch(cs._finalize_kernel(wp.float32, nv, n), 1,
                inputs=[ctx.grad, rhs, active, info, info, failure, ctx.search,
                        ctx.search_dot, ctx.newton_decrement], device="cpu")
            np.testing.assert_array_equal(ctx.search.numpy(), [[123.] * nv])
            self.assertEqual(ctx.search_dot.numpy()[0], 456.)
            self.assertEqual(ctx.newton_decrement.numpy()[0], 789.)
        active = wp.array([True], dtype=wp.bool, device="cpu")
        factor_info = wp.array([2], dtype=wp.int32, device="cpu")
        wp.launch(cs._finalize_kernel(wp.float32, nv, n), 1,
            inputs=[ctx.grad, rhs, active, factor_info, info, failure, ctx.search,
                    ctx.search_dot, ctx.newton_decrement], device="cpu")
        self.assertEqual(failure.numpy()[0], 2)
        self.assertTrue(np.isnan(ctx.search.numpy()[0, 0]))


@unittest.skipUnless(os.environ.get("S300_CUSOLVER_CUDA_TEST") == "1", "Explicit CUDA test opt-in required")
class CuSolverCudaControls(unittest.TestCase):
    def test_spd_numerics_graph_capture_skip_and_failure(self):
        device = wp.get_device("cuda:0")
        nv, n = 385, 400
        rng = np.random.default_rng(31)
        random = rng.normal(size=(nv, nv)).astype(np.float32) / np.sqrt(nv)
        matrix = (random @ random.T + np.eye(nv, dtype=np.float32) * 2).astype(np.float32)
        h = np.full((n, n), np.nan, np.float32)
        h[:nv, :nv] = np.triu(matrix) + np.tril(np.full((nv, nv), np.nan, np.float32), -1)
        grad = np.r_[rng.normal(size=nv), np.zeros(n - nv)].astype(np.float32)
        oracle = np.linalg.solve(matrix.astype(np.float64), grad[:nv].astype(np.float64))
        for precision in ("float32", "float64"):
            s = dummy_solver(nv, n, device)
            ctx = context(h, grad, nv, device)
            data = SimpleNamespace(nworld=1, nefc=wp.ones(1, dtype=wp.int32, device=device))
            with cs.CuSolverCholesky(s, precision=precision) as backend:
                backend.factorize_solve(s.mjw_model, data, ctx)
                backend.check_health()
                np.testing.assert_allclose(ctx.search.numpy()[0], -oracle, rtol=2e-4, atol=2e-6)
                with wp.ScopedCapture(device=device) as capture:
                    backend.factorize_solve(s.mjw_model, data, ctx)
                wp.capture_launch(capture.graph)
                backend.check_health()
                np.testing.assert_allclose(ctx.search.numpy()[0], -oracle, rtol=2e-4, atol=2e-6)
                np.testing.assert_allclose(ctx.search_dot.numpy()[0], oracle @ oracle, rtol=2e-5)
                np.testing.assert_allclose(ctx.newton_decrement.numpy()[0], grad[:nv] @ oracle, rtol=2e-5)
                # An indefinite Hessian is an explicit numerical failure,
                # never silently regularized or replaced by a CPU solve.
                bad = np.eye(n, dtype=np.float32)
                bad[0, 0] = -1.
                wp.copy(ctx.h, wp.array(bad[None], dtype=wp.float32, device=device))
                backend.factorize_solve(s.mjw_model, data, ctx)
                with self.assertRaisesRegex(RuntimeError, "potrf_info=1"):
                    backend.check_health()
                self.assertTrue(np.isnan(ctx.search.numpy()[0, 0]))


if __name__ == "__main__":
    unittest.main()
