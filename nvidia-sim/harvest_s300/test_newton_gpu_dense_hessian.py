"""Dense constraint assembly controls; CUDA runs require explicit opt-in."""
import os
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import warp as wp
from mujoco_warp._src import solver as ms

import newton_gpu_dense_hessian as dh
from test_newton_gpu_woodbury import guarded_solver


class DenseHessianHostControls(unittest.TestCase):
    def test_static_guards(self):
        s = guarded_solver(); s.mjw_data.njmax = 128
        self.assertIs(dh.validate_dense_hessian(s, 128), s.mjw_model)
        s.mjw_model.nmaxcondim = 4
        with self.assertRaises(ValueError):
            dh.validate_dense_hessian(s, 128)
        s.mjw_model.nmaxcondim = 3
        with self.assertRaises(ValueError):
            dh.validate_dense_hessian(s, 129)

    def test_only_upper_real_dofs_are_added(self):
        with wp.ScopedDevice("cpu"):
            h = wp.full((1, 5, 5), 7., dtype=float)
            product = wp.array(np.arange(9, dtype=np.float32).reshape(3, 3), dtype=float)
            done = wp.array([False], dtype=bool)
            wp.launch(dh._add_upper, (3, 3), inputs=[product, done, h], device="cpu")
            expected = np.full((5, 5), 7., np.float32)
            for row in range(3):
                for col in range(row, 3):
                    expected[row, col] += row * 3 + col
            np.testing.assert_array_equal(h.numpy()[0], expected)
            done.fill_(True)
            wp.launch(dh._add_upper, (3, 3), inputs=[product, done, h], device="cpu")
            np.testing.assert_array_equal(h.numpy()[0], expected)

    def test_launch_scope_replaces_one_kernel_and_restores_on_failure(self):
        owner = dh.DenseConstraintHessian.__new__(dh.DenseConstraintHessian)
        owner.model = object(); owner.active = False; owner.calls = 0
        selected = SimpleNamespace(key="_JTDACJ_sparse__locals__kernel_123")
        other = SimpleNamespace(key="_update_gradient_grad")
        owner.assemble_or_native = mock.Mock()
        def native(m, d, ctx, compact=False):
            wp.launch(other, 5)
            wp.launch(selected, (1, 2, 32))
            return 42
        owner.original = native
        with mock.patch.object(wp, "launch") as launch:
            self.assertEqual(owner.update_gradient(owner.model, "data", SimpleNamespace()), 42)
            self.assertIs(wp.launch, launch)
            launch.assert_called_once_with(other, 5)
            self.assertEqual(owner.assemble_or_native.call_count, 1)
            owner.original = mock.Mock(side_effect=RuntimeError("control"))
            with self.assertRaisesRegex(RuntimeError, "control"):
                owner.update_gradient(owner.model, "data", SimpleNamespace())
            self.assertIs(wp.launch, launch)
            self.assertFalse(owner.active)


def cuda_fixture(rows=64, width=93, cone=False):
    from bench_newton_gpu_woodbury import fixture
    solver, ctx, _, _, _ = fixture(385, 400, rows)
    m, d = solver.mjw_model, solver.mjw_data
    d.njmax = 256
    rng = np.random.default_rng(12)
    J = rng.normal(scale=.2, size=(rows, width)).astype(np.float32)
    with wp.ScopedDevice("cuda:0"):
        d.efc.J_rownnz = wp.array(np.full((1, rows), width, np.int32), dtype=int)
        d.efc.J_rowadr = wp.array((np.arange(rows) * width)[None], dtype=int)
        d.efc.J_colind = wp.array(np.tile(np.arange(width), rows).reshape(1, 1, -1), dtype=int)
        d.efc.J = wp.array(J.reshape(1, 1, -1), dtype=float)
        d.efc.jtdaj_adr = wp.array(np.arange(rows)[None], dtype=int)
        d.efc.jtdaj_nrow = wp.ones((1, rows), dtype=int)
        d.efc.jtdaj_nblock = wp.array([rows], dtype=int)
        d.dof_cdof = wp.zeros((1, m.nv), dtype=int)
        if cone:
            states = d.efc.state.numpy(); states[0, -3:] = 4; d.efc.state.assign(states)
            types = d.efc.type.numpy(); types[0, -3:] = 7; d.efc.type.assign(types)
            Ja = ctx.Jaref.numpy(); Ja[0, -3:] = [-.1, .3, -.2]; ctx.Jaref.assign(Ja)
            d.contact.efc_address.assign([[rows - 3]])
            d.efc.jtdaj_adr = wp.array(np.arange(rows - 2)[None], dtype=int)
            nrow = np.ones((1, rows - 2), np.int32); nrow[0, -1] = 3
            d.efc.jtdaj_nrow = wp.array(nrow, dtype=int)
            d.efc.jtdaj_nblock.assign([rows - 2])
        kernel = ms._JTDACJ_sparse(False, m.opt.cone, 3)
        inputs = [m.opt.impratio_invsqrt, d.contact.friction, d.contact.dim,
            d.efc.id, d.efc.jtdaj_adr, d.efc.jtdaj_nrow, d.efc.jtdaj_nblock,
            d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J,
            d.efc.D, d.efc.state, d.dof_cdof, ctx.Jaref, ctx.done, 32]
    def init():
        wp.launch(ms._update_gradient_init_h_sparse(False), (1, m.nv_pad, m.nv_pad),
            inputs=[m.nv, m.M_elemid, d.qH, d.dof_cdof, ctx.done], outputs=[ctx.h], device="cuda:0")
    def native():
        wp.launch(kernel, (1, 32, 32), inputs=inputs, outputs=[ctx.h],
                  block_dim=32, device="cuda:0")
    return solver, ctx, init, native


@unittest.skipUnless(os.environ.get("S300_DENSE_HESSIAN_CUDA_TEST") == "1", "Explicit CUDA opt-in required")
class DenseHessianCudaControls(unittest.TestCase):
    def test_actual_native_upper_hessian_parity_graph_and_capacity_fallback(self):
        for cone in (False, True):
            self.run_control(cone)

    def run_control(self, cone):
        with wp.ScopedDevice("cuda:0"):
            solver, ctx, init, native = cuda_fixture(cone=cone)
            init(); native()
            reference = ctx.h.numpy().copy()
            previous = ms._update_gradient
            with dh.DenseConstraintHessian(solver, capacity=128) as backend:
                init(); backend.assemble_or_native(solver.mjw_model, solver.mjw_data, ctx, native)
                actual = ctx.h.numpy()
                np.testing.assert_allclose(np.triu(actual[0]), np.triu(reference[0]), rtol=3e-5, atol=3e-5)
                with wp.ScopedCapture(device="cuda:0") as cap:
                    init(); backend.assemble_or_native(solver.mjw_model, solver.mjw_data, ctx, native)
                for _ in range(5):
                    wp.capture_launch(cap.graph)
                    np.testing.assert_array_equal(ctx.h.numpy(), actual)
                self.assertEqual(backend.statistics()["native_capacity_fallbacks"], 0)
            self.assertIs(ms._update_gradient, previous)
            with dh.DenseConstraintHessian(solver, capacity=32) as backend:
                init(); backend.assemble_or_native(solver.mjw_model, solver.mjw_data, ctx, native)
                np.testing.assert_allclose(np.triu(ctx.h.numpy()[0]), np.triu(reference[0]), rtol=3e-5, atol=3e-5)
                self.assertEqual(backend.statistics()["native_capacity_fallbacks"], 1)


if __name__ == "__main__":
    unittest.main()
