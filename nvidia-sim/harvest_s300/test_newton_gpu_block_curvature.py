"""CPU Warp controls for opt-in block C*J; no CUDA kernels are launched."""
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import warp as wp
from mujoco_warp._src import types as mt

import newton_gpu_dense_hessian as dh
from newton_gpu_woodbury import _curvature
from test_newton_gpu_woodbury import guarded_solver


class BlockCurvatureProductControls(unittest.TestCase):
    def compare_product(self, capacity, rows, *, contacts=(), friction=(.7, .4), seed=41):
        """Reference C is the existing coefficient kernel, then full dense C@J."""
        rng = np.random.default_rng(seed)
        nv = 13
        types = np.zeros((1, capacity), np.int32)
        ids = np.zeros((1, capacity), np.int32)
        states = np.ones((1, capacity), np.int32)
        D = rng.uniform(.1, 15., (1, capacity)).astype(np.float32)
        Ja = rng.uniform(-.5, .5, (1, capacity)).astype(np.float32)
        # Exercise equality, friction, limit, satisfied and both linear states.
        for row, state in enumerate((1, 0, 2, 3, 1, 1, 1)):
            if row < rows:
                states[0, row] = state
        if rows > 4:
            types[0, 4] = int(mt.ConstraintType.FRICTION_DOF)
        if rows > 5:
            types[0, 5] = int(mt.ConstraintType.LIMIT_JOINT)
        addresses = np.zeros((max(1, len(contacts)), 1), np.int32)
        dimensions = np.ones(max(1, len(contacts)), np.int32)
        for cid, (head, dim) in enumerate(contacts):
            addresses[cid, 0], dimensions[cid] = head, dim
            for row in range(head, min(head + dim, rows)):
                states[0, row] = int(mt.ConstraintState.CONE)
                types[0, row] = int(mt.ConstraintType.CONTACT_ELLIPTIC)
                ids[0, row] = cid
            Ja[0, head] = -.1
        fri = np.tile([*friction, .02, .01, .01], (len(dimensions), 1)).astype(np.float32)
        # Finite garbage in padding proves B does not retain old writes.
        J = rng.normal(size=(capacity, nv)).astype(np.float32)
        with wp.ScopedDevice("cpu"):
            def arr(value, dtype):
                return wp.array(value, dtype=dtype, device="cpu")
            count, typ, ident, state = (arr([rows], int), arr(types, int),
                                       arr(ids, int), arr(states, int))
            adr, dim = arr(addresses, int), arr(dimensions, int)
            C = wp.full((capacity, capacity), 123., dtype=float, device="cpu")
            wp.launch(_curvature, (capacity, capacity), inputs=[count, typ, ident,
                state, arr(D, float), arr(Ja, float), adr, dim, arr(fri, mt.vec5),
                arr([.8], float), C], device="cpu")
            B = wp.full((capacity, nv), 456., dtype=float, device="cpu")
            wp.launch(dh._block_curvature_product, (capacity, nv),
                inputs=[count, typ, ident, state, adr, dim, C, arr(J, float), B], device="cpu")
            actual, curvature = B.numpy(), C.numpy()
        reference = curvature.astype(np.float64) @ J.astype(np.float64)
        dense_fp32 = curvature @ J
        self.assertTrue(np.isfinite(actual).all())
        np.testing.assert_allclose(actual, reference, rtol=3e-6, atol=3e-6)
        np.testing.assert_allclose(actual, dense_fp32, rtol=3e-6, atol=3e-6)
        np.testing.assert_array_equal(actual[rows:], np.zeros((capacity - rows, nv), np.float32))
        np.testing.assert_allclose(J.T.astype(np.float64) @ actual.astype(np.float64),
                                   J.T.astype(np.float64) @ reference, rtol=5e-6, atol=1e-5)

    def test_regular_equality_friction_limit_and_linear_rows(self):
        self.compare_product(16, 11)

    def test_three_adjacent_cone_blocks_first_and_last_rows(self):
        self.compare_product(9, 9, contacts=((0, 3), (3, 3), (6, 3)))

    def test_mixed_rows_cones_and_capacity_padding(self):
        self.compare_product(16, 11, contacts=((0, 3), (8, 3)))

    def test_anisotropic_and_zero_friction(self):
        for friction in ((1.7, .03), (.001, .8), (0., 0.)):
            with self.subTest(friction=friction):
                self.compare_product(16, 11, contacts=((0, 3), (8, 3)), friction=friction)

    def test_partial_contact_boundary_and_dimension_one(self):
        self.compare_product(12, 9, contacts=((0, 1), (8, 3)))

    def test_empty_constraints_clear_private_product(self):
        self.compare_product(8, 0)

    def test_static_guards_and_opt_in_default(self):
        solver = guarded_solver()
        solver.mjw_data.njmax = 256
        self.assertIs(dh.validate_dense_hessian(solver, 128, True), solver.mjw_model)
        for value in (1, None, "true"):
            with self.assertRaisesRegex(ValueError, "must be bool"):
                dh.validate_dense_hessian(solver, 128, value)
        solver.mjw_model.nmaxcondim = 4
        with self.assertRaisesRegex(ValueError, "high-dimensional"):
            dh.validate_dense_hessian(solver, 128, True)
        self.assertIs(dh.DenseConstraintHessian.__init__.__kwdefaults__["block_curvature"], False)

    def test_runtime_capacity_fallback_and_done_are_unchanged(self):
        with wp.ScopedDevice("cpu"):
            for enabled in (False, True):
                backend = dh.DenseConstraintHessian.__new__(dh.DenseConstraintHessian)
                backend.model = SimpleNamespace(nv_pad=16)
                backend.closed = False
                backend.block_curvature = enabled
                backend.capacity = 8
                backend.device = "cpu"
                backend.stream = SimpleNamespace(cuda_stream=0)
                backend.selected = wp.zeros(1, dtype=int, device="cpu")
                backend.counters = wp.zeros(3, dtype=int, device="cpu")
                backend._assemble = mock.Mock()
                native = mock.Mock()
                d = SimpleNamespace(nworld=1, nefc=wp.array([8], dtype=int, device="cpu"))
                ctx = SimpleNamespace(done=wp.array([False], dtype=bool, device="cpu"),
                                      h=SimpleNamespace(shape=(1, 16, 16)))
                def conditional(condition, *, on_true, on_false):
                    (on_true if int(condition.numpy()[0]) else on_false)()
                with mock.patch.object(wp, "get_stream", return_value=backend.stream), \
                     mock.patch.object(wp, "capture_if", side_effect=conditional):
                    backend.assemble_or_native(backend.model, d, ctx, native)
                    backend._assemble.assert_called_once_with(backend.model, d, ctx)
                    native.assert_not_called()
                    d.nefc.assign([9])
                    backend.assemble_or_native(backend.model, d, ctx, native)
                    native.assert_called_once_with()
                    d.nefc.assign([8]); ctx.done.assign([True])
                    backend.assemble_or_native(backend.model, d, ctx, native)
                    self.assertEqual(backend._assemble.call_count, 1)
                    self.assertEqual(native.call_count, 2)
                np.testing.assert_array_equal(backend.counters.numpy(), [1, 1, 9])

    def test_opt_in_replaces_only_first_gemm(self):
        for enabled in (False, True):
            backend = dh.DenseConstraintHessian.__new__(dh.DenseConstraintHessian)
            backend.block_curvature = enabled
            backend.capacity, backend.nv, backend.device = 16, 13, "cpu"
            backend.J, backend.B, backend.C, backend.product = (object() for _ in range(4))
            backend.api = SimpleNamespace(gemm=mock.Mock())
            d = SimpleNamespace(nefc=object(), efc=SimpleNamespace(type=object(), id=object(), state=object()),
                                contact=SimpleNamespace(efc_address=object(), dim=object()))
            with mock.patch.object(wp, "launch") as launch:
                backend._products(d)
            self.assertEqual(launch.call_count, int(enabled))
            self.assertEqual(backend.api.gemm.call_count, 1 if enabled else 2)
            self.assertEqual(backend.api.gemm.call_args,
                mock.call(0, 1, 13, 13, 16, backend.B, 13, backend.J, 13, backend.product, 13))
            if enabled:
                self.assertIs(launch.call_args.args[0], dh._block_curvature_product)


if __name__ == "__main__":
    unittest.main()
