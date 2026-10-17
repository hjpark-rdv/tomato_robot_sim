"""CPU controls for solve-local dense Jacobian reuse; no CUDA execution.

Real Warp sparse/curvature kernels exercise values and padding. A CPU GEMM
stand-in compares identical operands and results, not CUDA bitwise behavior.
Captured CUDA replay and full harvest validity remain separate GPU checks.
"""
import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import warp as wp
from mujoco_warp._src import solver as ms, types as mt

import newton_gpu_dense_hessian as dh
import run_newton_harvest as runner
import test_newton_gpu_hessian_buckets as buckets
from test_newton_gpu_woodbury import guarded_solver

array = buckets.array


def cpu_conditional(condition, *, on_true=None, on_false=None):
    callback = on_true if int(condition.numpy()[0]) else on_false
    if callback is not None:
        callback()


class CachedJacobianControls(unittest.TestCase):
    def backend(self, enabled=True, overflow=8):
        owner = buckets.HessianBucketControls().backend(overflow=overflow)
        owner.cache_jacobian = enabled
        owner.jacobian_solve_active = False
        owner.jacobian_counters = wp.zeros(3, dtype=int, device='cpu')
        owner.jacobian_overflow_selected = wp.zeros(1, dtype=int, device='cpu')
        return owner

    def fixture(self, owner, rows, seed=7):
        rng = np.random.default_rng(seed)
        total, nv = 9, owner.nv
        # Nonuniform sorted CSR, including missing columns and an empty row.
        nnz = np.array([3, 1, 2, 0, 3, 2, 1, 3, 2], np.int32)
        addresses = np.concatenate(([0], np.cumsum(nnz[:-1]))).astype(np.int32)
        columns = np.concatenate([np.sort(rng.choice(nv, int(n), replace=False)) for n in nnz])
        values = rng.normal(size=len(columns)).astype(np.float32)
        values[0] = np.float32(-0.)
        types, states, ids = (np.zeros((1, total), np.int32) for _ in range(3))
        states.fill(int(mt.ConstraintState.QUADRATIC))
        types[0, :3] = int(mt.ConstraintType.CONTACT_ELLIPTIC)
        states[0, :3] = int(mt.ConstraintState.CONE)
        Ja = rng.normal(size=(1, total)).astype(np.float32)
        Ja[0, :3] = [-.1, .3, -.2]
        efc = SimpleNamespace(type=array(types, int), id=array(ids, int),
            state=array(states, int), D=array(rng.uniform(.2, 3., (1, total))),
            J_rownnz=array(nnz[None], int), J_rowadr=array(addresses[None], int),
            J_colind=array(columns.reshape(1, 1, -1), int),
            J=array(values.reshape(1, 1, -1)))
        data = SimpleNamespace(nworld=1, nefc=array([rows], int), efc=efc,
            contact=SimpleNamespace(efc_address=array([[0]], int), dim=array([3], int),
                friction=array([[.7, .4, .02, .01, .01]], mt.vec5)))
        context = SimpleNamespace(done=array([False], bool), Jaref=array(Ja),
            h=wp.full((1, 4, 4), 7., dtype=float, device='cpu'))
        return data, context

    def gpu_standins(self, owner):
        stack = contextlib.ExitStack()
        stack.enter_context(mock.patch.object(wp, 'get_stream', return_value=owner.stream))
        stack.enter_context(mock.patch.object(wp, 'capture_if', side_effect=cpu_conditional))
        return stack

    def products(self, owner):
        calls = []
        def gemm(ta, tb, m, n, k, left, lda, right, ldb, output, ldc):
            calls.append(((ta, tb, m, n, k, lda, ldb, ldc),
                          left.numpy().copy(), right.numpy().copy()))
            value = right.numpy() @ left.numpy() if tb == 0 else right.numpy().T @ left.numpy()
            output.assign(value.astype(np.float32))
        owner.api = SimpleNamespace(gemm=gemm)
        return calls

    def test_opt_in_type_guard_and_parser(self):
        self.assertIs(dh.DenseConstraintHessian.__init__.__kwdefaults__['cache_jacobian'], False)
        solver = guarded_solver(); solver.mjw_data.njmax = 256
        self.assertIs(dh.validate_dense_hessian(solver, 128, cache_jacobian=True), solver.mjw_model)
        for value in (1, None, 'true'):
            with self.assertRaisesRegex(ValueError, 'cache_jacobian must be bool'):
                dh.validate_dense_hessian(solver, 128, cache_jacobian=value)
        base = ['/tmp/unused', '--output', '/tmp/unused-out', '--backend', 'gpu']
        self.assertFalse(runner.arguments(base).cache_dense_jacobian)
        self.assertTrue(runner.arguments(base + ['--dense-hessian', '--cache-dense-jacobian']).cache_dense_jacobian)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            runner.arguments(base + ['--cache-dense-jacobian'])

    def test_repeated_assemblies_preserve_exact_operands_and_cpu_results(self):
        for rows in (0, 3, 4, 5, 8):
            with self.subTest(rows=rows):
                baseline, cached = self.backend(False), self.backend(True)
                d0, c0 = self.fixture(baseline, rows)
                d1, c1 = self.fixture(cached, rows)
                calls0, calls1 = self.products(baseline), self.products(cached)
                def iterations(owner, data, ctx, compact=False):
                    for step in range(3):
                        # Curvature must be recomputed even though J is constant.
                        data.efc.D.assign(data.efc.D.numpy() * np.float32(1.125))
                        ctx.Jaref.assign(ctx.Jaref.numpy() * np.float32(.875))
                        owner.assemble_or_native(owner.model, data, ctx,
                            lambda: self.fail('Unexpected capacity fallback'))
                with self.gpu_standins(baseline):
                    iterations(baseline, d0, c0)
                cached.original_solve = lambda m, d, c, compact=False: iterations(cached, d, c, compact)
                with self.gpu_standins(cached), \
                     mock.patch.object(cached, '_write_jacobian', wraps=cached._write_jacobian) as writes:
                    cached.solve(cached.model, d1, c1)
                self.assertEqual(writes.call_count, 1)
                self.assertFalse(cached.jacobian_solve_active)
                self.assertEqual(len(calls0), 6)
                self.assertEqual(len(calls0), len(calls1))
                for old, new in zip(calls0, calls1):
                    self.assertEqual(old[0], new[0])
                    for x, y in zip(old[1:], new[1:]):
                        np.testing.assert_array_equal(x.view(np.uint32), y.view(np.uint32))
                np.testing.assert_array_equal(c0.h.numpy().view(np.uint32), c1.h.numpy().view(np.uint32))

    def test_each_solve_rebuilds_on_bucket_changes_reset_and_same_nefc_new_values(self):
        owner = self.backend()
        self.products(owner)
        data, ctx = self.fixture(owner, 4)
        owner.original_solve = lambda m, d, c, compact=False: owner.assemble_or_native(m, d, c, lambda: None)
        with self.gpu_standins(owner), \
             mock.patch.object(owner, '_write_jacobian', wraps=owner._write_jacobian) as writes:
            for step, rows in enumerate((4, 5, 8, 4, 4, 0)):
                data.nefc.assign([rows])
                data.efc.J.assign(data.efc.J.numpy() + np.float32(step + 1))
                # Simulate stale workspaces after reset/replay; preparation
                # must clear all padding and rewrite even unchanged row counts.
                owner.J.fill_(111.); owner.overflow.J.fill_(222.)
                owner.solve(owner.model, data, ctx)
                active = owner if rows <= owner.capacity else owner.overflow
                reference = wp.full_like(active.J, 333.)
                previous = active.J; active.J = reference
                owner._write_jacobian(data, workspace=None if active is owner else active)
                active.J = previous
                np.testing.assert_array_equal(previous.numpy().view(np.uint32), reference.numpy().view(np.uint32))
                self.assertFalse(owner.jacobian_solve_active)
            # Six solve writes plus six explicit reference writes above.
            self.assertEqual(writes.call_count, 12)
        stats = owner.statistics()
        self.assertEqual(stats['jacobian_prepared_solves'], 6)
        self.assertEqual(stats['jacobian_primary_builds'], 4)
        self.assertEqual(stats['jacobian_overflow_builds'], 2)

    def test_done_from_previous_solve_does_not_suppress_new_jacobian(self):
        owner = self.backend(); data, ctx = self.fixture(owner, 5)
        ctx.done.assign([True])
        owner.original_solve = mock.Mock(return_value=42)
        with self.gpu_standins(owner), \
             mock.patch.object(owner, '_write_jacobian', wraps=owner._write_jacobian) as writes:
            self.assertEqual(owner.solve(owner.model, data, ctx), 42)
        writes.assert_called_once_with(data, workspace=owner.overflow)

    def test_fallback_and_negative_rows_never_prepare_a_truncated_jacobian(self):
        for overflow, rows in ((None, 5), (8, 9), (8, -1)):
            with self.subTest(overflow=overflow, rows=rows):
                owner = self.backend(overflow=overflow); data, ctx = self.fixture(owner, rows)
                native = mock.Mock()
                owner.original_solve = lambda m, d, c, compact=False: owner.assemble_or_native(m, d, c, native)
                with self.gpu_standins(owner), mock.patch.object(owner, '_write_jacobian') as writes:
                    owner.solve(owner.model, data, ctx)
                writes.assert_not_called(); native.assert_called_once_with()
                self.assertEqual(owner.statistics()['jacobian_native_fallback_solves'], 1)

    def test_single_bucket_rebuilds_and_outside_scope_keeps_original_writer(self):
        owner = self.backend(overflow=None); self.products(owner)
        data, ctx = self.fixture(owner, 4)
        owner.original_solve = lambda m, d, c, compact=False: owner.assemble_or_native(m, d, c, lambda: None)
        with self.gpu_standins(owner), \
             mock.patch.object(owner, '_write_jacobian', wraps=owner._write_jacobian) as writes:
            owner.solve(owner.model, data, ctx)
            owner.assemble_or_native(owner.model, data, ctx, lambda: None)
            owner.assemble_or_native(owner.model, data, ctx, lambda: None)
        self.assertEqual(writes.call_count, 3)
        self.assertEqual(owner.statistics()['jacobian_prepared_solves'], 1)

    def test_exception_ends_scope_and_next_solve_rebuilds(self):
        owner = self.backend(); data, ctx = self.fixture(owner, 4)
        owner.original_solve = mock.Mock(side_effect=RuntimeError('control'))
        with self.gpu_standins(owner), mock.patch.object(owner, '_prepare_jacobian') as prepare:
            with self.assertRaisesRegex(RuntimeError, 'control'):
                owner.solve(owner.model, data, ctx)
            self.assertFalse(owner.jacobian_solve_active)
            owner.original_solve = mock.Mock(return_value=13)
            self.assertEqual(owner.solve(owner.model, data, ctx), 13)
            self.assertEqual(prepare.call_count, 2)

    def test_compact_nested_wrong_model_world_and_stream_are_rejected_before_preparation(self):
        owner = self.backend(); data, ctx = self.fixture(owner, 4)
        with self.gpu_standins(owner), mock.patch.object(owner, '_prepare_jacobian') as prepare:
            with self.assertRaisesRegex(RuntimeError, 'compacted'):
                owner.solve(owner.model, data, ctx, compact=True)
            owner.jacobian_solve_active = True
            with self.assertRaisesRegex(RuntimeError, 'Nested'):
                owner.solve(owner.model, data, ctx)
            owner.jacobian_solve_active = False
            with self.assertRaisesRegex(RuntimeError, 'model/world'):
                owner.solve(object(), data, ctx)
            data.nworld = 2
            with self.assertRaisesRegex(RuntimeError, 'model/world'):
                owner.solve(owner.model, data, ctx)
            data.nworld = 1
            with mock.patch.object(wp, 'get_stream', return_value=SimpleNamespace(cuda_stream=1)), \
                 self.assertRaisesRegex(RuntimeError, 'stream changed'):
                owner.solve(owner.model, data, ctx)
            prepare.assert_not_called()

    def test_dispatch_is_model_scoped_and_close_restores_both_hooks(self):
        owner = self.backend()
        owner.original_solve, owner.original = mock.Mock(), mock.Mock()
        owner.solve = mock.Mock()
        owner.api = SimpleNamespace(close=mock.Mock())
        owner.cusolver_api = SimpleNamespace(destroy=mock.Mock())
        owner.handle = object()
        with mock.patch.object(dh, '_owner', owner), \
             mock.patch.object(ms, '_solve', dh._solve_dispatch), \
             mock.patch.object(ms, '_update_gradient', dh._dispatch), \
             mock.patch.object(wp, 'synchronize_stream'):
            dh._solve_dispatch(owner.model, 'data', 'context')
            owner.solve.assert_called_once_with(owner.model, 'data', 'context', compact=False)
            other = object()
            dh._solve_dispatch(other, 'data', 'context')
            owner.original_solve.assert_called_once_with(other, 'data', 'context', compact=False)
            owner.close()
            self.assertIs(ms._solve, owner.original_solve)
            self.assertIs(ms._update_gradient, owner.original)
            self.assertIsNone(dh._owner)
            self.assertTrue(owner.closed)
            owner.close()
            owner.api.close.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
