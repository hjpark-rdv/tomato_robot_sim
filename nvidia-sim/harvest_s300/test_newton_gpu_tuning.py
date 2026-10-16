"""Host-only configuration tests; no CUDA kernels are initialized or launched."""
from types import SimpleNamespace
import unittest
from unittest import mock

from mujoco_warp._src import types as mt
import newton_gpu_tuning as tuning


class BlockedCholeskyTuningTests(unittest.TestCase):
    def solver(self, cone=mt.ConeType.ELLIPTIC):
        return SimpleNamespace(use_mujoco_cpu=False, _step=0,
            mjw_model=SimpleNamespace(nv=385, nv_pad=400,
                opt=SimpleNamespace(solver=mt.SolverType.NEWTON, cone=cone),
                block_dim=SimpleNamespace(update_gradient_cholesky_blocked=32)))

    def module_api(self):
        opts = {"enable_mathdx_gemm": False, "enable_mathdx_solver": None,
                "fast_math": False, "deterministic": "preserved"}
        kernel = SimpleNamespace(key="factor", module=SimpleNamespace(name="unique_factor"))
        return opts, kernel

    def test_default_makes_no_changes(self):
        opts, kernel = self.module_api()
        solver = self.solver()
        with mock.patch.object(tuning.mw_solver, "_update_gradient_cholesky_blocked", return_value=kernel), \
             mock.patch.object(tuning.wp, "get_module_options", return_value=opts), \
             mock.patch.object(tuning.wp, "set_module_options") as set_options:
            evidence = tuning.configure_blocked_cholesky(solver)
        set_options.assert_not_called()
        self.assertFalse(evidence["enabled"])
        self.assertEqual(evidence["block_dim"], 32)
        self.assertFalse(evidence["mathdx_acceleration_verified"])

    def test_explicit_mathdx_and_launch_options_preserve_other_options(self):
        opts, kernel = self.module_api()
        solver = self.solver()
        def update(values, **kwargs):
            opts.update(values)
        with mock.patch.object(tuning.mw_solver, "_update_gradient_cholesky_blocked", return_value=kernel) as factory, \
             mock.patch.object(tuning.wp, "get_module_options", return_value=opts), \
             mock.patch.object(tuning.wp, "set_module_options", side_effect=update):
            evidence = tuning.configure_blocked_cholesky(solver, block_dim=128,
                mathdx_gemm=True, mathdx_solver=True)
        factory.assert_called_once_with(16, 400, 385)
        self.assertEqual(solver.mjw_model.block_dim.update_gradient_cholesky_blocked, 128)
        self.assertTrue(opts["enable_mathdx_gemm"])
        self.assertTrue(opts["enable_mathdx_solver"])
        self.assertFalse(opts["fast_math"])
        self.assertEqual(opts["deterministic"], "preserved")
        self.assertEqual(evidence["kernels"][0]["before"]["enable_mathdx_gemm"], False)
        self.assertEqual(evidence["scalar_dtype"], "float32")

    def test_nonelliptic_tunes_initial_and_cached_factorization(self):
        opts, kernel = self.module_api()
        with mock.patch.object(tuning.mw_solver, "_update_gradient_cholesky_blocked", return_value=kernel) as initial, \
             mock.patch.object(tuning.mw_solver, "_update_gradient_cholesky_blocked_skip_unchanged", return_value=kernel) as cached, \
             mock.patch.object(tuning.wp, "get_module_options", return_value=opts), \
             mock.patch.object(tuning.wp, "set_module_options") as set_options:
            result = tuning.configure_blocked_cholesky(self.solver(mt.ConeType.PYRAMIDAL), mathdx_gemm=True)
        initial.assert_called_once_with(16, 400, 385)
        cached.assert_called_once_with(16, 400, 385, True)
        self.assertTrue(result["incremental"])
        self.assertEqual(set_options.call_count, 2)

    def test_actual_kernel_module_options_without_cuda_launch(self):
        solver = self.solver()
        kernel = tuning.mw_solver._update_gradient_cholesky_blocked(16, 400, 385)
        before = dict(tuning.wp.get_module_options(module=kernel.module))
        try:
            result = tuning.configure_blocked_cholesky(solver, mathdx_gemm=True)
            self.assertEqual(result["kernels"][0]["kernel"], kernel.key)
            self.assertTrue(tuning.wp.get_module_options(module=kernel.module)["enable_mathdx_gemm"])
        finally:
            tuning.wp.set_module_options(before, module=kernel.module)

    def test_rejects_late_or_wrong_backend_and_invalid_flags(self):
        solver = self.solver()
        solver._step = 1
        with self.assertRaisesRegex(ValueError, "before stepping"):
            tuning.configure_blocked_cholesky(solver, mathdx_gemm=True)
        solver._step = 0
        solver.use_mujoco_cpu = True
        with self.assertRaisesRegex(ValueError, "GPU backend"):
            tuning.configure_blocked_cholesky(solver)
        for value in (1, "true"):
            with self.assertRaisesRegex(ValueError, "bool or None"):
                tuning.configure_blocked_cholesky(self.solver(), mathdx_gemm=value)
        for value in (True, 16, 33, 1056):
            with self.assertRaisesRegex(ValueError, "warp multiple"):
                tuning.configure_blocked_cholesky(self.solver(), block_dim=value)
        solver = self.solver()
        solver.mjw_model.opt.solver = mt.SolverType.CG
        with self.assertRaisesRegex(ValueError, "Newton constraint"):
            tuning.configure_blocked_cholesky(solver)


if __name__ == "__main__":
    unittest.main()
