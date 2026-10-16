"""Reverse-Cholesky inertia controls; actual CUDA is explicitly opt-in.

These are algebra/adapter fixtures, not evidence of farm harvest validity or RTF.
The 75-DOF branched articulated model supplies native MW sparse topology and M.
"""
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import mujoco as mj
import mujoco_warp as mw
from mujoco_warp._src import smooth
import numpy as np
import warp as wp

from newton_gpu_inertia_cusolver import CuSolverInertiaFactor
from test_newton_gpu_sparse_factor import branched_chain_xml


def sparse_topology(source):
    rows = np.repeat(np.arange(source.nv), source.M_rownnz)
    return rows, np.asarray(source.M_colind)


def dense_matrix(source, values):
    rows, cols = sparse_topology(source)
    matrix = np.zeros((source.nv, source.nv), dtype=np.float64)
    matrix[rows, cols] = np.asarray(values, dtype=np.float64)
    matrix[cols, rows] = np.asarray(values, dtype=np.float64)
    return matrix


def factor_reconstruction(source, values, inverse_diagonal):
    rows, cols = sparse_topology(source)
    lower = np.eye(source.nv, dtype=np.float64)
    offdiagonal = rows != cols
    lower[rows[offdiagonal], cols[offdiagonal]] = np.asarray(values)[offdiagonal]
    diagonal = 1. / np.asarray(inverse_diagonal, dtype=np.float64)
    return lower.T @ (diagonal[:, None] * lower)


def relative_residual(matrix, solution, rhs):
    return np.linalg.norm(matrix @ solution - rhs) / np.linalg.norm(rhs)


def mixed_block_xml():
    """Independent six-DOF subtree adds a native packed dense factor prefix."""
    root = ET.fromstring(branched_chain_xml())
    parent = root.find("worldbody")
    for i in range(2):
        parent = ET.SubElement(parent, "body", name=f"independent_{i}", pos=".1 0 .04")
        for axis in ("1 0 0", "0 1 0", "0 0 1"):
            ET.SubElement(parent, "joint", axis=axis, armature=".001")
        ET.SubElement(parent, "geom", type="sphere", size=".01", mass=".1", contype="0", conaffinity="0")
    return ET.tostring(root, encoding="unicode")


def fixture(device, xml=None, source=None):
    source = source or mj.MjModel.from_xml_string(xml or branched_chain_xml())
    data = mj.MjData(source)
    rng = np.random.default_rng(17)
    data.qpos[:] = rng.uniform(-.2, .2, source.nq)
    mj.mj_forward(source, data)
    with wp.ScopedDevice(device):
        model = mw.put_model(source)
        device_data = mw.put_data(source, data, nworld=1, nconmax=0, njmax=0)
    return SimpleNamespace(use_mujoco_cpu=False, _step=0, mj_model=source,
        mjw_model=model, mjw_data=device_data)


class InertiaAlgebraCpuControls(unittest.TestCase):
    def test_reverse_cholesky_to_native_ldl_on_branched_articulated_inertia(self):
        solver = fixture("cpu")
        source, d = solver.mj_model, solver.mjw_data
        self.assertEqual(source.nv, 75)
        self.assertEqual(solver.mjw_model.qLD_block_total, 0)
        matrix = dense_matrix(source, d.M.numpy()[0])
        reverse_cholesky = np.linalg.cholesky(matrix[::-1, ::-1])
        rows, cols = sparse_topology(source)
        stored = reverse_cholesky.T
        diagonal = np.diag(stored)[::-1]
        factor = stored[source.nv - 1 - rows, source.nv - 1 - cols] / diagonal[rows]
        factor[rows == cols] = diagonal[rows[rows == cols]] ** 2
        reconstructed = factor_reconstruction(source, factor, 1. / diagonal ** 2)
        np.testing.assert_allclose(reconstructed, matrix, rtol=1e-12, atol=1e-15)
        rhs = np.random.default_rng(41).normal(size=source.nv)
        solution = np.linalg.solve(reverse_cholesky.T,
            np.linalg.solve(reverse_cholesky, rhs[::-1]))[::-1]
        self.assertLess(relative_residual(matrix, solution, rhs), 1e-10)

    def test_cpu_and_late_install_are_rejected_without_changing_native_hooks(self):
        solver = fixture("cpu")
        original = smooth._factor_i_sparse, smooth._solve_LD_sparse
        with self.assertRaisesRegex(ValueError, "actual CUDA"):
            CuSolverInertiaFactor(solver)
        solver._step = 1
        with self.assertRaisesRegex(ValueError, "before stepping"):
            CuSolverInertiaFactor(solver)
        self.assertEqual(original, (smooth._factor_i_sparse, smooth._solve_LD_sparse))


@unittest.skipUnless(os.environ.get("S300_CUSOLVER_CUDA_TEST") == "1",
    "Explicit CUDA test opt-in required")
class InertiaCudaControls(unittest.TestCase):
    def test_real_75dof_native_factor_solve_residual_two_buffers_and_graph_repeat(self):
        self.run_solve_control(fixture("cuda:0"))

    def test_mixed_dense_sparse_native_slice_dispatch_and_retained_buffers(self):
        solver = fixture("cuda:0", mixed_block_xml())
        self.assertGreater(solver.mjw_model.qLD_block_total, 0)
        self.assertNotEqual(solver.mjw_data.qLD.ptr,
            solver.mjw_data.qLD[:, solver.mjw_model.qLD_block_total:].ptr)
        self.run_solve_control(solver)

    @unittest.skipUnless(os.environ.get("S300_INERTIA_FARM_MODEL") and
        os.environ.get("S300_INERTIA_FARM_SNAPSHOT"), "Explicit local farm asset opt-in required")
    def test_actual_farm_saved_qh_qualification(self):
        source = mj.MjModel.from_binary_path(os.environ["S300_INERTIA_FARM_MODEL"])
        from newton_mjcf import create_newton_mjcf
        # Reuse the production source-model uploader, including its explicit
        # unsupported-autoreset compatibility handling and source restoration.
        engine = create_newton_mjcf(Path(os.environ["S300_INERTIA_FARM_MODEL"]).with_name("input_scene.xml"),
            source_model=source, device="cuda:0", nconmax=128, njmax=256)
        solver = engine.solver
        self.assertEqual(source.nv, 385)
        with np.load(os.environ["S300_INERTIA_FARM_SNAPSHOT"], allow_pickle=False) as snapshot:
            # Current DISCRETE physics factors qH, not undamped M. The separate
            # raw-M random-RHS stress remains unqualified; see the recorded
            # conditioning/residual diagnostics, never a relaxed tolerance.
            self.run_solve_control(solver, snapshot["M"], snapshot["qH"], qualified_keys=("qH",))

    def run_solve_control(self, solver, source_m=None, source_h=None, qualified_keys=("M", "qH")):
        m, d, source = solver.mjw_model, solver.mjw_data, solver.mj_model
        device = d.qpos.device
        rows, cols = sparse_topology(source)
        original_factor, original_solve = smooth._factor_i_sparse, smooth._solve_LD_sparse
        original_public_solve = smooth.solve_LD
        source_m = d.M.numpy().copy() if source_m is None else np.array(source_m, copy=True)
        self.assertEqual(source_m.shape, d.M.shape)
        wp.copy(d.M, wp.array(source_m, dtype=wp.float32, device=device))
        if source_h is None:
            source_h = source_m.copy()
            source_h[0, rows == cols] += np.linspace(.0003, .003, source.nv, dtype=np.float32)
        else:
            source_h = np.array(source_h, copy=True)
        self.assertEqual(source_h.shape, d.M.shape)
        h = wp.array(source_h, dtype=wp.float32, device=device)
        rhs_np = np.random.default_rng(101).normal(size=(1, source.nv)).astype(np.float32)
        rhs = wp.array(rhs_np, dtype=wp.float32, device=device)
        output_m, output_h = wp.empty_like(rhs), wp.empty_like(rhs)
        alias_h = wp.empty_like(rhs)
        native_m, native_h = wp.empty_like(rhs), wp.empty_like(rhs)
        native_l, native_d = wp.empty_like(d.qLD), wp.empty_like(d.qLDiagInv)
        matrices = [dense_matrix(source, source_m[0]), dense_matrix(source, source_h[0])]
        with wp.ScopedDevice(device):
            smooth.factor_solve_i(m, d, d.M, native_l, native_d, native_m, rhs)
            smooth.factor_solve_i(m, d, h, native_l, native_d, native_h, rhs)
            backend = CuSolverInertiaFactor(solver)
            try:
                # Native public dispatchers must use the same retained factors,
                # including both M and modified inertia-like H concurrently.
                def factor_and_solve():
                    smooth.factor_m(m, d)
                    smooth.factor_solve_i(m, d, h, d.qHLD, d.qHDiagInv, output_h, rhs)
                    smooth.solve_m(m, d, output_m, rhs)
                    wp.copy(alias_h, rhs)
                    smooth.solve_LD(m, d, d.qHLD, d.qHDiagInv, alias_h, alias_h)

                factor_and_solve()
                backend.check_health()
                np.testing.assert_array_equal(alias_h.numpy(), output_h.numpy())
                for key, matrix, output, native, factors, reciprocals in [
                    ("M", matrices[0], output_m, native_m, d.qLD, d.qLDiagInv),
                    ("qH", matrices[1], output_h, native_h, d.qHLD, d.qHDiagInv),
                ]:
                    if key not in qualified_keys:
                        continue
                    actual = output.numpy()[0].astype(np.float64)
                    oracle = np.linalg.solve(matrix, rhs_np[0].astype(np.float64))
                    self.assertLess(relative_residual(matrix, actual, rhs_np[0]), 5e-5)
                    self.assertLess(relative_residual(matrix, native.numpy()[0], rhs_np[0]), 5e-5)
                    self.assertLess(np.linalg.norm(actual - oracle) / np.linalg.norm(oracle), 2e-4)
                    # Native atomic LDL and Cholesky have different roundoff on
                    # this ill-conditioned articulated inertia. Compare vector
                    # error to the float64 oracle norm, avoiding cancellation-
                    # sensitive per-component ratios near zero.
                    self.assertLess(np.linalg.norm(actual - native.numpy()[0]) /
                        np.linalg.norm(oracle), 2e-4)
                    reconstructed = factor_reconstruction(source,
                        factors.numpy()[0, m.qLD_block_total:], reciprocals.numpy()[0])
                    self.assertLess(np.linalg.norm(reconstructed - matrix) / np.linalg.norm(matrix), 2e-6)
                before = output_m.numpy(), output_h.numpy()
                with wp.ScopedCapture(device=device) as capture:
                    factor_and_solve()
                for _ in range(3):
                    wp.capture_launch(capture.graph)
                    backend.check_health()
                    np.testing.assert_array_equal(output_m.numpy(), before[0])
                    np.testing.assert_array_equal(output_h.numpy(), before[1])
                    np.testing.assert_array_equal(alias_h.numpy(), before[1])
                # Captured RHS is live data, not an accidentally cached solution.
                rhs_changed = rhs_np * np.float32(.37)
                wp.copy(rhs, wp.array(rhs_changed, dtype=wp.float32, device=device))
                wp.capture_launch(capture.graph)
                backend.check_health()
                for key, matrix, output in zip(("M", "qH"), matrices, (output_m, output_h)):
                    if key not in qualified_keys:
                        continue
                    self.assertLess(relative_residual(matrix, output.numpy()[0], rhs_changed[0]), 5e-5)
                np.testing.assert_array_equal(d.M.numpy(), source_m)
                np.testing.assert_array_equal(h.numpy(), source_h)
                self.assertFalse(backend.evidence["host_solve"])
                self.assertFalse(backend.evidence["physical_parameters_changed"])
            finally:
                backend.close()
        self.assertIs(smooth._factor_i_sparse, original_factor)
        self.assertIs(smooth._solve_LD_sparse, original_solve)
        self.assertIs(smooth.solve_LD, original_public_solve)

    def test_non_spd_failure_is_latched_and_does_not_become_a_valid_solve(self):
        solver = fixture("cuda:0")
        m, d, source = solver.mjw_model, solver.mjw_data, solver.mj_model
        device = d.qpos.device
        rows, cols = sparse_topology(source)
        bad = d.M.numpy().copy()
        first_diagonal = np.flatnonzero((rows == 0) & (cols == 0))[0]
        bad[0, first_diagonal] = -1.
        bad_gpu = wp.array(bad, dtype=wp.float32, device=device)
        rhs = wp.ones((1, source.nv), dtype=wp.float32, device=device)
        output = wp.empty_like(rhs)
        with wp.ScopedDevice(device):
            backend = CuSolverInertiaFactor(solver)
            try:
                backend.factor(m, d, bad_gpu, d.qLD, d.qLDiagInv)
                self.assertGreater(int(backend.info.numpy()[0]), 0)
                with self.assertRaisesRegex(RuntimeError, "inertia factor failed"):
                    backend.check_health()
                self.assertTrue(np.isnan(d.qLD.numpy()).all())
                self.assertTrue(np.isnan(d.qLDiagInv.numpy()).all())
                # Successful future factorization must not silently erase history.
                backend.factor(m, d, d.M, d.qLD, d.qLDiagInv)
                with self.assertRaisesRegex(RuntimeError, "inertia factor failed"):
                    backend.check_health()
            finally:
                backend.close()


if __name__ == "__main__":
    unittest.main()
