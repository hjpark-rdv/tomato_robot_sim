"""Actual-backend controls for fixed-order COM/CRB/RNE reductions.

Default is CPU Warp. These controls establish equations and isolation, not
CUDA GUI repeatability or speed. Set NEWTON_GPU_STAGE_DEVICE=cuda:0 for CUDA.
"""

import os
from types import SimpleNamespace
import unittest

import mujoco as mj
import mujoco_warp as mw
from mujoco_warp._src import smooth
from mujoco_warp._src.types import vec10
import numpy as np
import warp as wp

from newton_gpu_tree_sums import (FixedOrderTreeSums, _plans, _sum_vec3, _sum_vec10,
                                 _sum_spatial, _flat_crb, _flat_rne)


TREE_XML = """<mujoco model="branched_tree_control">
  <option timestep=".0025" integrator="discrete" solver="Newton"
          jacobian="sparse" cone="elliptic" gravity=".1 -.2 -9.81"/>
  <default><geom size=".035" mass=".15" contype="0" conaffinity="0"/>
           <joint damping=".04"/></default>
  <worldbody>
    <body name="root" pos=".1 -.2 .8"><freejoint/><geom/>
      <body name="a" pos=".1 .2 .3"><joint axis="1 0 0"/><geom/>
        <body name="a1" pos=".2 0 .1"><joint axis="0 1 0"/><geom mass=".05"/></body>
        <body name="a2" pos="-.1 .2 .15"><joint axis="0 0 1"/><geom mass=".2"/></body>
      </body>
      <body name="b" pos="-.2 .1 .1"><joint axis="0 1 0"/><geom mass=".4"/>
        <body name="b1" pos=".1 -.2 .2"><joint axis="1 0 0"/><geom mass=".1"/></body>
      </body>
      <body name="c" pos=".1 -.1 .2"><joint axis="0 0 1"/><geom mass=".3"/></body>
    </body>
    <body name="root2" pos="-.5 .2 .6"><freejoint/><geom mass=".2"/>
      <body name="d" pos=".1 .1 .1"><joint axis="1 0 0"/><geom mass=".07"/></body>
    </body>
  </worldbody>
</mujoco>"""


class TestFixedOrderTreeSums(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()
        cls.device = os.environ.get("NEWTON_GPU_STAGE_DEVICE", "cpu")

    def fixture(self, nworld=2):
        model = mj.MjModel.from_xml_string(TREE_XML)
        data = mj.MjData(model)
        data.qpos[7:13] = [.2, -.3, .1, -.15, .25, -.2]
        data.qpos[-1] = .31
        data.qvel[:] = np.linspace(-.7, .9, model.nv)
        mj.mj_forward(model, data)
        with wp.ScopedDevice(self.device):
            backend_model = mw.put_model(model)
            backend_data = mw.put_data(model, data, nworld=nworld, nconmax=8, njmax=32)
        return SimpleNamespace(use_mujoco_cpu=False, _step=0, mj_model=model,
                               mjw_model=backend_model, mjw_data=backend_data), data

    @staticmethod
    def reference(values, plans):
        result = values.copy()
        for plan in plans:
            for group, parent in enumerate(plan.host_parents):
                for child in plan.host_children[plan.host_offsets[group]:plan.host_offsets[group + 1]]:
                    result[:, parent] += result[:, child]
        return result

    def test_three_vector_reductions_match_fixed_order_reference_in_multiple_worlds(self):
        run, _ = self.fixture()
        adapter = FixedOrderTreeSums(run)
        try:
            rng = np.random.default_rng(14)
            for kernel, dtype, width, plans in (
                    (_sum_vec3, wp.vec3, 3, adapter.include_world),
                    (_sum_vec10, vec10, 10, adapter.exclude_world),
                    (_sum_spatial, wp.spatial_vector, 6, adapter.include_world)):
                initial = rng.normal(size=(2, run.mj_model.nbody, width)).astype(np.float32)
                values = wp.array(initial, dtype=dtype, device=self.device)
                with wp.ScopedDevice(self.device):
                    adapter._accumulate(kernel, plans, values, 2)
                np.testing.assert_array_equal(values.numpy(), self.reference(initial, plans))
        finally:
            adapter.close()

    def test_world_parent_policy_and_level_no_alias_guards(self):
        run, _ = self.fixture()
        adapter = FixedOrderTreeSums(run)
        try:
            self.assertTrue(any(0 in p.host_parents for p in adapter.include_world))
            self.assertFalse(any(0 in p.host_parents for p in adapter.exclude_world))
            for plan in adapter.include_world:
                self.assertFalse(set(plan.host_parents) & set(plan.host_children))
                self.assertEqual(len(plan.host_parents), len(set(plan.host_parents)))
            with self.assertRaisesRegex(ValueError, "repeated body"):
                _plans(np.array([0, 0]), [np.array([0]), np.array([1, 1])], self.device, True)
            with self.assertRaisesRegex(ValueError, "level ordering"):
                _plans(np.array([0, 0]), [np.array([0, 1])], self.device, True)
        finally:
            adapter.close()

    def test_actual_forward_preserves_equations_and_physical_inputs(self):
        run, native = self.fixture()
        with wp.ScopedDevice(self.device):
            mw.forward(run.mjw_model, run.mjw_data)
        original = {key: getattr(run.mjw_data, key).numpy().copy()
                    for key in ("subtree_com", "cinert", "crb", "M", "cfrc_int", "qfrc_bias", "qacc")}
        qpos = run.mjw_data.qpos.numpy().copy()
        qvel = run.mjw_data.qvel.numpy().copy()
        mass = run.mjw_model.body_mass.numpy().copy()
        adapter = FixedOrderTreeSums(run)
        try:
            with wp.ScopedDevice(self.device):
                mw.forward(run.mjw_model, run.mjw_data)
            for key, expected in original.items():
                np.testing.assert_allclose(getattr(run.mjw_data, key).numpy(), expected,
                                           rtol=5e-5, atol=3e-5, err_msg=key)
            for key in ("subtree_com", "cinert", "crb", "qfrc_bias"):
                np.testing.assert_allclose(getattr(run.mjw_data, key).numpy()[0], getattr(native, key),
                                           rtol=8e-5, atol=3e-5, err_msg="native " + key)
            np.testing.assert_array_equal(run.mjw_data.qpos.numpy(), qpos)
            np.testing.assert_array_equal(run.mjw_data.qvel.numpy(), qvel)
            np.testing.assert_array_equal(run.mjw_model.body_mass.numpy(), mass)
        finally:
            adapter.close()

    def test_repeated_forward_is_exact_with_different_preexisting_derived_buffers(self):
        run, _ = self.fixture()
        adapter = FixedOrderTreeSums(run)
        try:
            with wp.ScopedDevice(self.device):
                mw.forward(run.mjw_model, run.mjw_data)
            expected = {key: getattr(run.mjw_data, key).numpy().copy()
                        for key in ("subtree_com", "crb", "M", "qfrc_bias")}
            for key in expected:
                getattr(run.mjw_data, key).fill_(123.)
            with wp.ScopedDevice(self.device):
                mw.forward(run.mjw_model, run.mjw_data)
            for key, values in expected.items():
                np.testing.assert_array_equal(getattr(run.mjw_data, key).numpy(), values)
        finally:
            adapter.close()

    def test_nonselected_model_uses_original_dispatch_and_close_restores_aliases(self):
        selected, _ = self.fixture()
        other, _ = self.fixture()
        original = {name: getattr(smooth, name) for name in ("com_pos", "crb", "_rne_cfrc_backward")}
        aliases = {name: getattr(mw, name, None) for name in original}
        adapter = FixedOrderTreeSums(selected)
        # Another backend model must retain the original function behavior.
        for name in ("_com_pos", "_crb", "_rne_backward"):
            # Wrappers hold bound methods captured at install, so verify via
            # output identity instead of replacing these methods afterwards.
            self.assertTrue(callable(getattr(adapter, name)))
        with wp.ScopedDevice(self.device):
            mw.forward(other.mjw_model, other.mjw_data)
        before = other.mjw_data.qfrc_bias.numpy().copy()
        adapter.close()
        adapter.close()
        for name, function in original.items():
            self.assertIs(getattr(smooth, name), function)
            self.assertIs(getattr(mw, name, None), aliases[name])
        with wp.ScopedDevice(self.device):
            mw.forward(other.mjw_model, other.mjw_data)
        np.testing.assert_array_equal(other.mjw_data.qfrc_bias.numpy(), before)

    def test_native_or_late_install_is_rejected(self):
        run, _ = self.fixture()
        run.use_mujoco_cpu = True
        with self.assertRaisesRegex(ValueError, "Warp execution"):
            FixedOrderTreeSums(run)
        run.use_mujoco_cpu = False
        run._step = 1
        with self.assertRaisesRegex(ValueError, "before stepping"):
            FixedOrderTreeSums(run)
        run._step = 0
        run.mjw_model.opt.enableflags |= mw.EnableBit.SLEEP
        with self.assertRaisesRegex(ValueError, "sleeping"):
            FixedOrderTreeSums(run)
        run.mjw_model.opt.enableflags &= ~mw.EnableBit.SLEEP
        run.mjw_model.has_fluid = True
        with self.assertRaisesRegex(ValueError, "fluid"):
            FixedOrderTreeSums(run)

    def test_flat_mode_preserves_equations_against_native_float64_reference(self):
        run, native = self.fixture()
        adapter = FixedOrderTreeSums(run, mode="flat")
        try:
            with wp.ScopedDevice(self.device):
                mw.forward(run.mjw_model, run.mjw_data)
            for key in ("subtree_com", "cinert", "crb", "qfrc_bias"):
                actual = getattr(run.mjw_data, key).numpy()
                expected = np.broadcast_to(getattr(native, key), actual.shape)
                np.testing.assert_allclose(actual, expected, rtol=8e-5, atol=3e-5,
                                           err_msg="flat/native " + key)
            self.assertTrue(adapter.evidence["float32_grouping_changed"])
            self.assertFalse(adapter.evidence["full_pipeline_determinism_claimed"])
            expected = run.mjw_data.qfrc_bias.numpy().copy()
            with wp.ScopedDevice(self.device):
                mw.forward(run.mjw_model, run.mjw_data)
            np.testing.assert_array_equal(run.mjw_data.qfrc_bias.numpy(), expected)
        finally:
            adapter.close()

    def test_flat_sum_error_is_bounded_against_float64_intrinsic_sum(self):
        run, _ = self.fixture()
        adapter = FixedOrderTreeSums(run, mode="flat")
        try:
            rng = np.random.default_rng(58)
            for kernel, dtype, width in ((_flat_crb, vec10, 10), (_flat_rne, wp.spatial_vector, 6)):
                initial = (rng.normal(size=(2, run.mj_model.nbody, width)) *
                           10. ** rng.uniform(-4, 4, size=(2, run.mj_model.nbody, width))).astype(np.float32)
                source = wp.array(initial, dtype=dtype, device=self.device)
                output = wp.empty(source.shape, dtype=dtype, device=self.device)
                with wp.ScopedDevice(self.device):
                    wp.launch(kernel, dim=source.shape,
                              inputs=[adapter.flat_offsets, adapter.flat_descendants, source], outputs=[output])
                expected = np.zeros_like(initial, dtype=np.float64)
                absolute_sum = np.zeros_like(expected)
                for body in range(run.mj_model.nbody):
                    ids = adapter.flat_host_descendants[adapter.flat_host_offsets[body]:adapter.flat_host_offsets[body + 1]]
                    if body == 0 and kernel is _flat_crb:
                        ids = [0]
                    expected[:, body] = initial[:, ids].astype(np.float64).sum(axis=1)
                    absolute_sum[:, body] = np.abs(initial[:, ids].astype(np.float64)).sum(axis=1)
                relative_to_absolute_sum = np.abs(output.numpy() - expected) / np.maximum(absolute_sum, 1e-30)
                self.assertLess(float(relative_to_absolute_sum.max()), 2e-6)
                np.testing.assert_array_equal(source.numpy(), initial)
        finally:
            adapter.close()


if __name__ == "__main__":
    unittest.main()
