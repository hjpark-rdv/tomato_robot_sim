"""CPU controls for the opt-in overflow-only dense Hessian bucket.

Routing and the existing Warp Jacobian/curvature/upper-add kernels run on CPU.
The BLAS stand-in checks workspace/dimensions and algebra; actual CUDA SGEMM
bitwise parity and nonlinear harvest behavior require separate GPU trials.
"""
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import warp as wp
from mujoco_warp._src import types as mt

import newton_gpu_dense_hessian as dh
from test_newton_gpu_woodbury import guarded_solver


def array(value, dtype=float):
    return wp.array(np.asarray(value), dtype=dtype, device='cpu')


def cpu_conditional(condition, *, on_true, on_false):
    (on_true if int(condition.numpy()[0]) else on_false)()


class HessianBucketControls(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()

    def backend(self, primary=4, overflow=8, nv=3):
        owner = dh.DenseConstraintHessian.__new__(dh.DenseConstraintHessian)
        owner.capacity, owner.nv, owner.device = primary, nv, 'cpu'
        owner.model = SimpleNamespace(nv_pad=nv + 1,
            opt=SimpleNamespace(impratio_invsqrt=array([1.])))
        owner.closed = False
        owner.block_curvature = False
        owner.stream = SimpleNamespace(cuda_stream=0)
        owner.selected = wp.zeros(1, dtype=int, device='cpu')
        owner.counters = wp.zeros(3, dtype=int, device='cpu')
        owner.evidence = {}
        for name, shape in dict(J=(primary, nv), B=(primary, nv), C=(primary, primary),
                                product=(nv, nv)).items():
            setattr(owner, name, wp.full(shape, 17., dtype=float, device='cpu'))
        owner.overflow_capacity = overflow
        owner.overflow = None
        if overflow is not None:
            owner.overflow = SimpleNamespace(capacity=overflow,
                J=wp.full((overflow, nv), 29., dtype=float, device='cpu'),
                B=wp.full((overflow, nv), 29., dtype=float, device='cpu'),
                C=wp.full((overflow, overflow), 29., dtype=float, device='cpu'),
                product=wp.full((nv, nv), 29., dtype=float, device='cpu'),
                selected=wp.zeros(1, dtype=int, device='cpu'),
                counters=wp.zeros(1, dtype=int, device='cpu'))
        return owner

    def test_default_capacity_and_block_product_are_unchanged_and_overflow_is_opt_in(self):
        defaults = dh.DenseConstraintHessian.__init__.__kwdefaults__
        self.assertEqual(defaults['capacity'], 128)
        self.assertIs(defaults['block_curvature'], False)
        self.assertIsNone(defaults['overflow_capacity'])
        solver = guarded_solver(); solver.mjw_data.njmax = 256
        self.assertIs(dh.validate_dense_hessian(solver, 128, overflow_capacity=256),
                      solver.mjw_model)
        for value in (True, 0, -1, 128, 127, 257, 256., '256'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'Overflow capacity'):
                dh.validate_dense_hessian(solver, 128, overflow_capacity=value)

    def test_boundaries_done_and_native_fallback_select_one_bucket_only(self):
        owner = self.backend(primary=128, overflow=256)
        owner._assemble = mock.Mock()
        native = mock.Mock()
        data = SimpleNamespace(nworld=1, nefc=array([0], int))
        context = SimpleNamespace(done=array([False], bool), h=SimpleNamespace(shape=(1, 4, 4)))
        with mock.patch.object(wp, 'get_stream', return_value=owner.stream), \
             mock.patch.object(wp, 'capture_if', side_effect=cpu_conditional):
            for count, expected in ((0, 'primary'), (128, 'primary'),
                                    (129, 'overflow'), (256, 'overflow'),
                                    (257, 'native'), (-1, 'native')):
                with self.subTest(count=count):
                    owner._assemble.reset_mock(); native.reset_mock()
                    data.nefc.assign([count])
                    owner.assemble_or_native(owner.model, data, context, native)
                    if expected == 'primary':
                        owner._assemble.assert_called_once_with(owner.model, data, context)
                        native.assert_not_called()
                    elif expected == 'overflow':
                        owner._assemble.assert_called_once_with(owner.model, data, context,
                            workspace=owner.overflow)
                        native.assert_not_called()
                    else:
                        owner._assemble.assert_not_called(); native.assert_called_once_with()
            data.nefc.assign([200]); context.done.assign([True])
            owner._assemble.reset_mock(); native.reset_mock()
            owner.assemble_or_native(owner.model, data, context, native)
            owner._assemble.assert_not_called(); native.assert_called_once_with()
        self.assertEqual(owner.statistics(), dict(dense_assemblies=4,
            native_capacity_fallbacks=2, maximum_observed_constraint_rows=257,
            primary_dense_assemblies=2, overflow_dense_assemblies=2,
            primary_capacity=128, overflow_capacity=256))

    def test_primary_gemm_arguments_are_identical_with_and_without_overflow(self):
        owner = self.backend(primary=128, overflow=256, nv=13)
        gemm = mock.Mock(); owner.api = SimpleNamespace(gemm=gemm)
        expected = [
            mock.call(0, 0, 13, 128, 128, owner.J, 13, owner.C, 128, owner.B, 13),
            mock.call(0, 1, 13, 13, 128, owner.B, 13, owner.J, 13, owner.product, 13)]
        owner._products()
        self.assertEqual(gemm.call_args_list, expected)
        gemm.reset_mock(); owner.overflow = None; owner.overflow_capacity = None
        owner._products()
        self.assertEqual(gemm.call_args_list, expected)

    def test_cpu_assembly_keeps_small_workspace_and_never_truncates_overflow_rows(self):
        for rows in (0, 4, 5, 8):
            with self.subTest(rows=rows):
                owner = self.backend()
                rng = np.random.default_rng(64)
                J = rng.normal(size=(8, owner.nv)).astype(np.float32)
                D = rng.uniform(.2, 3., size=(1, 8)).astype(np.float32)
                efc = SimpleNamespace(type=array(np.zeros((1, 8)), int),
                    id=array(np.zeros((1, 8)), int), state=array(np.ones((1, 8)), int),
                    D=array(D), J_rownnz=array(np.full((1, 8), owner.nv), int),
                    J_rowadr=array(np.arange(8)[None] * owner.nv, int),
                    J_colind=array(np.tile(np.arange(owner.nv), 8).reshape(1, 1, -1), int),
                    J=array(J.reshape(1, 1, -1)))
                data = SimpleNamespace(nworld=1, nefc=array([rows], int), efc=efc,
                    contact=SimpleNamespace(efc_address=array([[0]], int), dim=array([1], int),
                        friction=array(np.ones((1, 5)), mt.vec5)))
                context = SimpleNamespace(done=array([False], bool),
                    Jaref=array(np.zeros((1, 8))),
                    h=wp.full((1, 4, 4), 7., dtype=float, device='cpu'))
                calls = []
                def gemm(ta, tb, m, n, k, left, lda, right, ldb, output, ldc):
                    calls.append((ta, tb, m, n, k, left, lda, right, ldb, output, ldc))
                    self.assertEqual(ta, 0)
                    result = right.numpy() @ left.numpy() if tb == 0 else right.numpy().T @ left.numpy()
                    output.assign(result.astype(np.float32))
                owner.api = SimpleNamespace(gemm=gemm)
                native = mock.Mock()
                with mock.patch.object(wp, 'get_stream', return_value=owner.stream), \
                     mock.patch.object(wp, 'capture_if', side_effect=cpu_conditional):
                    owner.assemble_or_native(owner.model, data, context, native)
                native.assert_not_called()
                active = owner if rows <= owner.capacity else owner.overflow
                inactive = owner.overflow if active is owner else owner
                self.assertEqual([call[4] for call in calls], [active.capacity, active.capacity])
                self.assertIs(calls[0][5], active.J)
                self.assertIs(calls[1][9], active.product)
                # Actual Warp kernels build the sparse Jacobian and exact C.
                expected = np.full((4, 4), 7., dtype=np.float64)
                delta = J[:rows].astype(np.float64).T @ (D[0, :rows, None] * J[:rows]).astype(np.float64)
                for one in range(owner.nv):
                    for two in range(one, owner.nv): expected[one, two] += delta[one, two]
                np.testing.assert_allclose(context.h.numpy()[0], expected, rtol=3e-6, atol=3e-6)
                for name in ('J', 'B', 'C', 'product'):
                    np.testing.assert_array_equal(getattr(inactive, name).numpy(),
                        29. if inactive is owner.overflow else 17.)


if __name__ == '__main__':
    unittest.main()
