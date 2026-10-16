"""Native reduction parity and controls; CUDA requires explicit opt-in."""
import os
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import warp as wp
from mujoco_warp._src import solver as ms

import newton_gpu_fixed_reductions as fr
from test_newton_gpu_woodbury import guarded_solver


def fixture(device="cpu", nv=9, rows=5, width=4, descending=False):
    rng = np.random.default_rng(7)
    with wp.ScopedDevice(device):
        columns, values, nnz, adr = [], [], [], []
        for row in range(rows):
            count = 0 if row == 1 else min(width, nv)
            adr.append(len(columns)); nnz.append(count)
            selected = np.sort(rng.choice(nv, count, replace=False))
            columns.extend((selected[::-1] if descending else selected).tolist())
            values.extend(rng.normal(size=count).tolist())
        def array(a, dtype):
            return wp.array(a, dtype=dtype)
        d = SimpleNamespace(nworld=1, njmax=rows, nefc=array([rows], int),
            efc=SimpleNamespace(J_rownnz=array([nnz], int), J_rowadr=array([adr], int),
                J_colind=array(np.asarray(columns).reshape(1, 1, -1), int),
                J=array(np.asarray(values).reshape(1, 1, -1), float),
                force=array(rng.normal(size=(1, rows)), float),
                Ma=array(rng.normal(size=(1, nv)), float)),
            qfrc_smooth=array(rng.normal(size=(1, nv)), float),
            qfrc_constraint=wp.full((1, nv), 123., dtype=float),
            dof_cdof=wp.empty((1, 0), dtype=int), qpos=wp.zeros((1, nv), dtype=float))
        ctx = SimpleNamespace(done=array([False], bool),
            state_changed_count=array([1], int), grad=wp.full((1, nv + 3), 234., dtype=float),
            grad_dot=wp.full(1, 345., dtype=float))
        failure = array([2147483647], int); counters = wp.zeros(2, dtype=int)
    return d, ctx, failure, counters


def qfrc(d, ctx, failure, counters, fixed):
    wp.launch(ms._zero_qfrc_constraint_sparse, (d.nworld, d.qfrc_constraint.shape[1]),
        inputs=[ctx.state_changed_count, ctx.done], outputs=[d.qfrc_constraint])
    if fixed:
        wp.launch(fr._validate_rows, (d.nworld, d.njmax),
            inputs=[d.qfrc_constraint.shape[1], d.nefc, d.efc.J_rownnz, d.efc.J_rowadr,
                d.efc.J_colind, ctx.state_changed_count, ctx.done, failure, counters])
        wp.launch(fr._qfrc_fixed, d.qfrc_constraint.shape,
            inputs=[d.nefc, d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J,
                d.efc.force, ctx.state_changed_count, ctx.done, failure, d.qfrc_constraint])
    else:
        wp.launch(ms._update_constraint_init_qfrc_constraint_sparse(False), (1, d.njmax),
            inputs=[d.nefc, d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J,
                d.efc.force, d.dof_cdof, ctx.state_changed_count, ctx.done], outputs=[d.qfrc_constraint])


def gradient(d, ctx, fixed, block_dim=32):
    ctx.grad_dot.zero_()
    if fixed:
        wp.launch_tiled(ms._update_gradient_grad_tiled, dim=d.nworld,
            inputs=[d.qfrc_constraint.shape[1], d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, ctx.done],
            outputs=[ctx.grad, ctx.grad_dot], block_dim=block_dim)
    else:
        wp.launch(ms._update_gradient_grad(False), d.qfrc_constraint.shape,
            inputs=[d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma,
                ctx.state_changed_count, ctx.done], outputs=[ctx.grad, ctx.grad_dot])


class FixedReductionCpuControls(unittest.TestCase):
    @unittest.skipUnless(os.getenv("S300_REDUCTION_SNAPSHOT"), "Explicit actual farm snapshot opt-in required")
    def test_actual_saved_farm_csr_against_native_cpu(self):
        z = np.load(os.environ["S300_REDUCTION_SNAPSHOT"])
        with wp.ScopedDevice("cpu"):
            def a(key, dtype):
                return wp.array(z[key], dtype=dtype)
            nv = z["qvel"].shape[1]
            d = SimpleNamespace(nworld=1, njmax=z["efc.J_rownnz"].shape[1],
                nefc=a("nefc", int), efc=SimpleNamespace(
                    J_rownnz=a("efc.J_rownnz", int), J_rowadr=a("efc.J_rowadr", int),
                    J_colind=a("efc.J_colind", int), J=a("efc.J", float), force=a("efc.force", float)),
                dof_cdof=wp.empty((1, 0), dtype=int),
                qfrc_constraint=wp.zeros((1, nv), dtype=float))
            ctx = SimpleNamespace(done=wp.array([False], dtype=bool),
                state_changed_count=wp.array([1], dtype=int))
            failure = wp.array([2147483647], dtype=int); counts = wp.zeros(2, dtype=int)
            qfrc(d, ctx, failure, counts, False); native = d.qfrc_constraint.numpy()
            qfrc(d, ctx, failure, counts, True)
            self.assertEqual(int(failure.numpy()[0]), 2147483647)
            np.testing.assert_allclose(d.qfrc_constraint.numpy(), native, rtol=3e-6, atol=3e-6)

    def test_descending_native_csr_order_is_supported(self):
        with wp.ScopedDevice("cpu"):
            d, ctx, failure, counts = fixture()
            qfrc(d, ctx, failure, counts, False); expected = d.qfrc_constraint.numpy()
            cols, values = d.efc.J_colind.numpy(), d.efc.J.numpy()
            nnz, adr = d.efc.J_rownnz.numpy()[0], d.efc.J_rowadr.numpy()[0]
            for n, a in zip(nnz, adr):
                cols[0, 0, a:a+n] = cols[0, 0, a:a+n][::-1]
                values[0, 0, a:a+n] = values[0, 0, a:a+n][::-1]
            d.efc.J_colind.assign(cols); d.efc.J.assign(values)
            qfrc(d, ctx, failure, counts, True)
            self.assertEqual(int(failure.numpy()[0]), 2147483647)
            np.testing.assert_array_equal(d.qfrc_constraint.numpy(), expected)

    def test_native_parity_done_and_unchanged(self):
        with wp.ScopedDevice("cpu"):
            d, ctx, failure, counts = fixture()
            qfrc(d, ctx, failure, counts, False); expected = d.qfrc_constraint.numpy()
            d.qfrc_constraint.fill_(123.)
            qfrc(d, ctx, failure, counts, True)
            np.testing.assert_array_equal(d.qfrc_constraint.numpy(), expected)
            gradient(d, ctx, False); g, n = ctx.grad.numpy(), ctx.grad_dot.numpy()
            gradient(d, ctx, True, block_dim=1)
            np.testing.assert_array_equal(ctx.grad.numpy(), g)
            np.testing.assert_allclose(ctx.grad_dot.numpy(), n, rtol=2e-6)
            for done, changed in [(True, 1), (False, 0)]:
                ctx.done.assign([done]); ctx.state_changed_count.assign([changed])
                d.qfrc_constraint.fill_(123.)
                qfrc(d, ctx, failure, counts, True)
                np.testing.assert_array_equal(d.qfrc_constraint.numpy(), np.full((1, 9), 123.))
            self.assertEqual(int(failure.numpy()[0]), 2147483647)

    def test_unsorted_csr_and_capacity_are_latched_not_silent(self):
        with wp.ScopedDevice("cpu"):
            for invalid in ("unsorted", "capacity"):
                d, ctx, failure, counts = fixture()
                if invalid == "unsorted":
                    cols = d.efc.J_colind.numpy(); cols[0, 0, :2] = cols[0, 0, :2][::-1]
                    d.efc.J_colind.assign(cols)
                else:
                    d.nefc.assign([d.njmax + 1])
                qfrc(d, ctx, failure, counts, True)
                self.assertNotEqual(int(failure.numpy()[0]), 2147483647)
                self.assertTrue(np.isnan(d.qfrc_constraint.numpy()).all())

    def test_feature_guards(self):
        s = guarded_solver()
        self.assertIs(fr.validate_fixed_reductions(s), s.mjw_model)
        s.mjw_model.ntendon = 1
        with self.assertRaises(ValueError):
            fr.validate_fixed_reductions(s)

    def test_composed_launch_scope_restores_enclosing_wrapper(self):
        owner = fr.FixedSolverReductions.__new__(fr.FixedSolverReductions)
        owner.closed = owner.active = False
        owner.device = "cpu"; owner.stream = SimpleNamespace(cuda_stream=0)
        selected, other = object(), object()
        replacement = mock.Mock()
        def native():
            wp.launch(other, 1); wp.launch(selected, 1)
        with mock.patch.object(wp, "get_stream", return_value=owner.stream), \
                mock.patch.object(wp, "launch") as enclosing:
            owner._scope_call(native, selected, replacement)
            self.assertIs(wp.launch, enclosing)
            enclosing.assert_called_once_with(other, 1)
            replacement.assert_called_once_with()
            with self.assertRaisesRegex(RuntimeError, "control"):
                owner._scope_call(lambda: (_ for _ in ()).throw(RuntimeError("control")), selected, replacement)
            self.assertIs(wp.launch, enclosing)
            self.assertFalse(owner.active)


@unittest.skipUnless(os.getenv("S300_FIXED_REDUCTIONS_CUDA_TEST") == "1", "Explicit CUDA test opt-in required")
class FixedReductionCudaControls(unittest.TestCase):
    def test_native_385_dof_parity_and_graph_repeat(self):
        with wp.ScopedDevice("cuda:0"):
            d, ctx, failure, counts = fixture("cuda:0", 385, 80, 93, descending=True)
            qfrc(d, ctx, failure, counts, False); expected = d.qfrc_constraint.numpy()
            qfrc(d, ctx, failure, counts, True)
            np.testing.assert_allclose(d.qfrc_constraint.numpy(), expected, rtol=3e-5, atol=3e-5)
            gradient(d, ctx, False); g, n = ctx.grad.numpy(), ctx.grad_dot.numpy()
            gradient(d, ctx, True, block_dim=256)
            np.testing.assert_array_equal(ctx.grad.numpy(), g)
            np.testing.assert_allclose(ctx.grad_dot.numpy(), n, rtol=3e-6)
            reference = d.qfrc_constraint.numpy(), ctx.grad.numpy(), ctx.grad_dot.numpy()
            with wp.ScopedCapture(device="cuda:0") as cap:
                qfrc(d, ctx, failure, counts, True); gradient(d, ctx, True, block_dim=256)
            for _ in range(5):
                wp.capture_launch(cap.graph)
                for actual, expected in zip((d.qfrc_constraint.numpy(), ctx.grad.numpy(), ctx.grad_dot.numpy()), reference):
                    np.testing.assert_array_equal(actual, expected)

    def test_model_scoped_public_wrappers_capture_and_restore(self):
        with wp.ScopedDevice("cuda:0"):
            d, ctx, failure, counts = fixture("cuda:0", 385, 80, 93)
            solver = guarded_solver(); solver.mjw_data = d
            solver.mjw_model.nv = 385
            solver.mjw_model.block_dim = SimpleNamespace(update_gradient_grad=256)
            def native_constraint(m, data, context, track_changes=False, stable_fast=False):
                qfrc(data, context, failure, counts, False)
            def native_gradient(m, data, context, compact=False):
                gradient(data, context, False)
            with mock.patch.object(ms, "_update_constraint", native_constraint), \
                    mock.patch.object(ms, "_update_gradient", native_gradient):
                with fr.FixedSolverReductions(solver) as backend:
                    ms._update_constraint(solver.mjw_model, d, ctx)
                    ms._update_gradient(solver.mjw_model, d, ctx)
                    baseline = d.qfrc_constraint.numpy(), ctx.grad.numpy(), ctx.grad_dot.numpy()
                    with wp.ScopedCapture(device="cuda:0") as capture:
                        ms._update_constraint(solver.mjw_model, d, ctx)
                        ms._update_gradient(solver.mjw_model, d, ctx)
                    wp.capture_launch(capture.graph)
                    backend.check_health()
                    for a, b in zip((d.qfrc_constraint.numpy(), ctx.grad.numpy(), ctx.grad_dot.numpy()), baseline):
                        np.testing.assert_array_equal(a, b)
                    self.assertGreater(backend.statistics()["actual_qfrc_rebuilds"], 0)
                self.assertIs(ms._update_constraint, native_constraint)
                self.assertIs(ms._update_gradient, native_gradient)


if __name__ == "__main__":
    unittest.main()
