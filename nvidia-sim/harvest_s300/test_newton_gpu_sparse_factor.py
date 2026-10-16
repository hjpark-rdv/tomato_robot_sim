"""Sparse inertia fusion tests on real CPU-backed MW buffers, no CUDA load."""
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import mujoco as mj
import mujoco_warp as mw
from mujoco_warp._src import smooth
import numpy as np
import warp as wp

from newton_gpu_sparse_factor import FusedSparseInertiaFactor, build_factor_schedule


def branched_chain_xml():
    root = ET.Element("mujoco")
    ET.SubElement(root, "option", gravity="0 0 0", jacobian="sparse")
    parent = ET.SubElement(root, "worldbody")
    branch = None
    for i in range(23):
        parent = ET.SubElement(parent, "body", name=f"body_{i}", pos="0 0 .03")
        if i == 8:
            branch = parent
        for axis in ("1 0 0", "0 1 0", "0 0 1"):
            ET.SubElement(parent, "joint", axis=axis, armature=".001")
        ET.SubElement(parent, "geom", type="sphere", size=".01", mass=".1", contype="0", conaffinity="0")
    for i in range(2):
        side = ET.SubElement(branch, "body", name=f"side_{i}", pos=f".03 {i * .03} 0")
        for axis in ("1 0 0", "0 1 0", "0 0 1"):
            ET.SubElement(side, "joint", axis=axis, armature=".001")
        ET.SubElement(side, "geom", type="sphere", size=".01", mass=".1", contype="0", conaffinity="0")
    return ET.tostring(root, encoding="unicode")


class FusedSparseFactorTests(unittest.TestCase):
    def fixture(self):
        source = mj.MjModel.from_xml_string(branched_chain_xml())
        data = mj.MjData(source)
        rng = np.random.default_rng(17)
        data.qpos[:] = rng.uniform(-.2, .2, source.nq)
        mj.mj_forward(source, data)
        with wp.ScopedDevice("cpu"):
            model = mw.put_model(source)
            device_data = mw.put_data(source, data, nworld=2, nconmax=0, njmax=0)
        return SimpleNamespace(use_mujoco_cpu=False, _step=0, mj_model=source,
                               mjw_model=model, mjw_data=device_data)

    def test_real_sparse_factor_exact_cpu_parity_and_hook_restoration(self):
        solver = self.fixture()
        m, d = solver.mjw_model, solver.mjw_data
        original = smooth._factor_i_sparse
        with wp.ScopedDevice("cpu"):
            baseline_l = wp.empty_like(d.M)
            baseline_d = wp.empty((d.nworld, m.nv), dtype=float)
            original(m, d, d.M, baseline_l, baseline_d)
            fused_l = wp.empty_like(d.M)
            fused_d = wp.empty((d.nworld, m.nv), dtype=float)
            with FusedSparseInertiaFactor(solver) as hook:
                self.assertEqual(hook.block_dim, 1)
                self.assertGreater(hook.nlevels, 32)
                smooth._factor_i_sparse(m, d, d.M, fused_l, fused_d)
                self.assertEqual(hook.calls, 1)
                np.testing.assert_array_equal(fused_l.numpy(), baseline_l.numpy())
                np.testing.assert_array_equal(fused_d.numpy(), baseline_d.numpy())
                self.assertFalse(hook.evidence["atomic_additions"])
                self.assertFalse(hook.evidence["equations_or_physical_parameters_changed"])
                self.assertEqual(hook.evidence["source_updates"], len(hook.schedule["ordered_updates"]))
            self.assertIs(smooth._factor_i_sparse, original)

    def test_other_model_uses_original_and_shape_mismatch_is_rejected(self):
        solver = self.fixture()
        m, d = solver.mjw_model, solver.mjw_data
        with wp.ScopedDevice("cpu"):
            l = wp.empty_like(d.M)
            diag = wp.empty((d.nworld, m.nv), dtype=float)
            with FusedSparseInertiaFactor(solver) as hook:
                called = []
                hook.original = lambda *a: called.append(a)
                other = object()
                smooth._factor_i_sparse(other, d, d.M, l, diag)
                self.assertIs(called[0][0], other)
                with self.assertRaisesRegex(RuntimeError, "workspace shape"):
                    hook.factor(m, d, d.M, l, wp.empty((2, 1), dtype=float))
                # Restore the actual owner before close resets the global hook.
                hook.original = self.saved_original

    def setUp(self):
        self.saved_original = smooth._factor_i_sparse

    def tearDown(self):
        self.assertIs(smooth._factor_i_sparse, self.saved_original)

    def test_schedule_rejects_same_level_dependencies(self):
        with self.assertRaisesRegex(ValueError, "row dependency"):
            build_factor_schedule([1, 2, 3], [0, 1, 3], [[0, 1, 1], [1, 2, 4]], [0, 2])

    def test_schedule_groups_all_destination_entries_without_losing_updates(self):
        schedule = build_factor_schedule([1, 2, 2], [0, 1, 3], [[0, 1, 1], [0, 2, 3]], [0, 2])
        np.testing.assert_array_equal(schedule["targets"], [[0, 0, 0, 2]])
        np.testing.assert_array_equal(schedule["ordered_updates"], [0, 1])


if __name__ == "__main__":
    unittest.main()
