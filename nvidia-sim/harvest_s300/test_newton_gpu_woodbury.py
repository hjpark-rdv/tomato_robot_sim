"""No CUDA by default; opt in with S300_WOODBURY_CUDA_TEST=1.

Numerical controls exercise the actual installed native elliptic Hessian and
the real CUDA libraries, including residual-triggered full GPU fallback.
"""
import os
from types import SimpleNamespace
import unittest

import numpy as np
import warp as wp
from mujoco_warp._src import solver as ms, types as mt

import newton_gpu_woodbury as wb
from test_newton_gpu_cusolver import context


def guarded_solver():
    return SimpleNamespace(use_mujoco_cpu=False, _step=0,
        mjw_data=SimpleNamespace(nworld=1),
        mjw_model=SimpleNamespace(nv=75, is_sparse=True, nmaxcondim=3,
            nflex=0, ntendon=0, nefmK=0, has_flex_passive=False,
            has_non_simple_flex=False, efm0_active=False, has_fluid=False,
            opt=SimpleNamespace(solver=mt.SolverType.NEWTON,
                integrator=mt.IntegratorType.DISCRETE, cone=mt.ConeType.ELLIPTIC,
                enableflags=0)))


class WoodburyHostControls(unittest.TestCase):
    def test_features_rejected_before_cuda_handles(self):
        self.assertIs(wb.validate_woodbury(guarded_solver(), 16).__class__, SimpleNamespace)
        for attr, value in [("use_mujoco_cpu", True), ("_step", 1)]:
            s = guarded_solver(); setattr(s, attr, value)
            with self.assertRaises(ValueError):
                wb.validate_woodbury(s, 16)
        for attr, value in [("ntendon", 1), ("has_fluid", True), ("nmaxcondim", 4), ("is_sparse", False)]:
            s = guarded_solver(); setattr(s.mjw_model, attr, value)
            with self.assertRaises(ValueError):
                wb.validate_woodbury(s, 16)
        for rank in (True, 0, 75, 2.5):
            with self.assertRaises(ValueError):
                wb.validate_woodbury(guarded_solver(), rank)

    def test_basis_curvature_matches_native_sparse_hessian(self):
        rank, nv = 5, 7
        rng = np.random.default_rng(21)
        J = rng.normal(size=(rank, nv)).astype(np.float32)
        # Two quadratic equality rows and a single elliptic three-row block.
        D = np.array([[.5, 2., 13., 13., 13.]], np.float32)
        state = np.array([[1, 1, 4, 4, 4]], np.int32)
        typ = np.array([[0, 0, 7, 7, 7]], np.int32)
        jaref = np.array([[0., 0., -.1, .35, -.2]], np.float32)
        with wp.ScopedDevice("cpu"):
            def arr(a, dtype=None):
                return wp.array(a, dtype=dtype, device="cpu")
            nefc = arr([rank], int)
            types = arr(typ, int); ids = arr(np.zeros((1, rank), np.int32), int)
            states = arr(state, int); diagonal = arr(D, float); Ja = arr(jaref, float)
            addresses = arr([[2]], int); dims = arr([3], int)
            fri = arr([[.7, .4, .02, .01, .01]], mt.vec5)
            imp = arr([.8], float)
            C = wp.zeros((rank, rank), dtype=float, device="cpu")
            wp.launch(wb._curvature, (rank, rank), inputs=[nefc, types, ids,
                states, diagonal, Ja, addresses, dims, fri, imp, C], device="cpu")
            native_h = wp.zeros((1, nv, nv), dtype=float, device="cpu")
            wp.launch(ms._JTDACJ_sparse(False, mt.ConeType.ELLIPTIC, 3), (1, 2, 1),
                inputs=[imp, fri, dims, ids, arr([[0, 2]], int), arr([[2, 3]], int),
                    arr([2], int), arr(np.full((1, rank), nv, np.int32), int),
                    arr((np.arange(rank) * nv)[None], int),
                    arr(np.tile(np.arange(nv), rank).reshape(1, 1, -1), int),
                    arr(J.reshape(1, 1, -1), float), diagonal, states,
                    arr(np.zeros((1, nv), np.int32), int), Ja,
                    arr([False], bool), 2], outputs=[native_h], block_dim=1, device="cpu")
            expected = J.T @ C.numpy() @ J
            np.testing.assert_allclose(np.triu(native_h.numpy()[0]), np.triu(expected), rtol=3e-6, atol=3e-6)
            np.testing.assert_allclose(C.numpy(), C.numpy().T, rtol=0., atol=0.)

    def test_actual_hessian_residual_and_nonfinite_control(self):
        with wp.ScopedDevice("cpu"):
            h = np.array([[3., 1.], [999., 2.]], np.float32)
            grad = np.array([[4., 3.]], np.float32)
            errors = wp.zeros(2, dtype=wp.float64, device="cpu")
            fallback = wp.zeros(1, dtype=int, device="cpu")
            count = wp.zeros(6, dtype=int, device="cpu")
            maximum = wp.zeros(1, dtype=float, device="cpu")
            for x, rejected in [([1., 1.], False), ([1., 0.], True), ([float("nan"), 1.], True)]:
                wp.launch(wb._residual_rows, 2, inputs=[wp.array(h[None], dtype=float),
                    wp.array(grad, dtype=float), wp.array(x, dtype=float), errors], device="cpu")
                wp.launch(wb._residual_decision, 1, inputs=[wp.array(grad, dtype=float),
                    errors, wp.array([False], dtype=bool), 1.e-4, fallback, count, maximum], device="cpu")
                self.assertEqual(bool(fallback.numpy()[0]), rejected)
            self.assertEqual(int(count.numpy()[4]), 2)
            self.assertEqual(int(count.numpy()[5]), 1)

    def test_runtime_capacity_and_multi_dof_guard_and_cache_token(self):
        with wp.ScopedDevice("cpu"):
            rows = wp.array([5], dtype=int)
            nnz = wp.array([[1, 1]], dtype=int)
            done = wp.array([False], dtype=bool)
            time = wp.array([0.], dtype=float)
            cached = wp.array([-1.], dtype=float)
            selected = wp.zeros(1, dtype=int); refresh = wp.zeros(1, dtype=int)
            count = wp.zeros(6, dtype=int)
            def check(expected, new):
                wp.launch(wb._flags, 1, inputs=[rows, nnz, done, time, 16,
                    False, cached, selected, refresh, count], device="cpu")
                self.assertEqual((int(selected.numpy()[0]), int(refresh.numpy()[0])), (expected, new))
            check(1, 1); check(1, 0)
            time.assign([1.]); rows.assign([17]); check(0, 0)
            self.assertEqual(float(cached.numpy()[0]), 0.)
            rows.assign([5]); check(1, 1)
            nnz.assign([[1, 2]]); check(0, 0)
            self.assertEqual(int(count.numpy()[2]), 2)


@unittest.skipUnless(os.getenv("S300_WOODBURY_CUDA_TEST") == "1", "explicit CUDA test opt-in required")
class WoodburyCudaControls(unittest.TestCase):
    def fixture(self, rows=8):
        import mujoco as mj
        import mujoco_warp as mw
        from test_newton_gpu_sparse_factor import branched_chain_xml
        source = mj.MjModel.from_xml_string(branched_chain_xml())
        source.opt.cone = mj.mjtCone.mjCONE_ELLIPTIC
        native = mj.MjData(source); mj.mj_forward(source, native)
        with wp.ScopedDevice("cuda:0"):
            m = mw.put_model(source)
            d = mw.put_data(source, native, nworld=1, nconmax=8, njmax=32, njmax_nnz=32 * source.nv)
        m.opt.integrator = mt.IntegratorType.DISCRETE
        solver = SimpleNamespace(use_mujoco_cpu=False, _step=0, mj_model=source, mjw_model=m, mjw_data=d)
        idx = m.M_elemid.numpy(); sparse = d.M.numpy()[0]
        A = np.zeros((m.nv, m.nv), np.float32)
        for i in range(m.nv):
            for j in range(m.nv):
                address = idx[max(i, j), min(i, j)]
                if address >= 0:
                    A[i, j] = sparse[address]
        # qH may add physically authored implicit terms. Testing exactly this
        # positive native inertia backbone isolates the solver algebra.
        d.qH.assign(d.M)
        rng = np.random.default_rng(73)
        J = rng.normal(scale=.1, size=(rows, m.nv)).astype(np.float32)
        D = rng.uniform(.2, 2., rows).astype(np.float32)
        def fill(array, values):
            target = np.zeros(array.shape, np.float32)
            target.flat[:len(np.asarray(values).ravel())] = np.asarray(values).ravel()
            array.assign(target)
        d.nefc.assign(np.array([rows], np.int32))
        rownnz = np.zeros(d.efc.J_rownnz.shape, np.int32); rownnz[0, :rows] = m.nv
        rowadr = np.zeros(d.efc.J_rowadr.shape, np.int32); rowadr[0, :rows] = np.arange(rows) * m.nv
        cols = np.zeros(d.efc.J_colind.shape, np.int32); cols.flat[:J.size] = np.tile(np.arange(m.nv), rows)
        d.efc.J_rownnz.assign(rownnz); d.efc.J_rowadr.assign(rowadr); d.efc.J_colind.assign(cols)
        fill(d.efc.J, J); fill(d.efc.D, D)
        states = np.zeros(d.efc.state.shape, np.int32); states[0, :rows] = 1; d.efc.state.assign(states)
        H = A.astype(np.float64) + J.astype(np.float64).T @ (D[:, None] * J).astype(np.float64)
        padded = np.zeros((m.nv_pad, m.nv_pad), np.float32); padded[:m.nv, :m.nv] = H
        gradient = rng.normal(size=m.nv_pad).astype(np.float32)
        ctx = context(padded, gradient, m.nv, "cuda:0")
        ctx.Jaref = wp.zeros(d.efc.D.shape, dtype=float, device="cuda:0")
        return solver, ctx, H, gradient[:m.nv]

    def test_cuda_parity_graph_replay_and_explicit_numerical_fallback(self):
        solver, ctx, H, gradient = self.fixture()
        m, d = solver.mjw_model, solver.mjw_data
        previous = ms._cholesky_factorize_solve
        with wp.ScopedDevice("cuda:0"), wb.WoodburyCholesky(solver, rank_capacity=16) as backend:
            backend.factorize_solve(m, d, ctx)
            backend.check_health()
            solution = -ctx.search.numpy()[0]
            # Ill-conditioned native inertia has large individual solution
            # entries; use the same declared whole-vector residual metric.
            self.assertLess(np.linalg.norm(H @ solution - gradient) / np.linalg.norm(gradient), 1.e-4)
            self.assertEqual(backend.statistics()["numerical_residual_fallback_solves"], 0)
            reference = ctx.search.numpy()
            # Same source qH/J; private right-hand-sides overwritten each replay.
            with wp.ScopedCapture(device="cuda:0") as capture:
                backend.factorize_solve(m, d, ctx)
            for _ in range(8):
                ctx.search.fill_(99.)
                wp.capture_launch(capture.graph)
                np.testing.assert_array_equal(ctx.search.numpy(), reference)
            # Deliberately mismatch actual H against the assumed decomposition.
            # The native full GPU solve must replace the reduced answer.
            changed = ctx.h.numpy(); changed[0, 0, 0] += .2; ctx.h.assign(changed)
            backend.factorize_solve(m, d, ctx)
            backend.check_health()
            expected = H.copy(); expected[0, 0] += .2
            self.assertLess(np.linalg.norm(expected @ -ctx.search.numpy()[0] - gradient) / np.linalg.norm(gradient), 1.e-4)
            self.assertGreater(backend.statistics()["numerical_residual_fallback_solves"], 0)
        self.assertIs(ms._cholesky_factorize_solve, previous)

    def test_capacity_fallback_is_full_gpu_solve(self):
        solver, ctx, H, gradient = self.fixture(rows=20)
        with wp.ScopedDevice("cuda:0"), wb.WoodburyCholesky(solver, rank_capacity=16) as backend:
            backend.factorize_solve(solver.mjw_model, solver.mjw_data, ctx)
            backend.check_health()
            solution = -ctx.search.numpy()[0]
            self.assertLess(np.linalg.norm(H @ solution - gradient) / np.linalg.norm(gradient), 1.e-4)
            stats = backend.statistics()
            self.assertEqual(stats["reduced_solves"], 0)
            self.assertEqual(stats["full_fallback_solves"], 1)


if __name__ == "__main__":
    unittest.main()
