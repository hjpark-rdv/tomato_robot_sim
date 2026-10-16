"""Bitwise multiset and real collision controls for canonical contact order.

The CPU Warp default validates the algorithm, not GPU repeatability/speed.
"""

import os
from unittest import mock
import unittest

import mujoco as mj
import mujoco_warp as mw
import numpy as np
import warp as wp

from newton_gpu_compat import install_connect_velocity_cache_refresh
from newton_gpu_deterministic import CanonicalContactOrder, _CONTACT_FIELDS
from test_newton_gpu_deterministic import FixtureSolver
from test_newton_gpu_stage_order import CONNECT_XML


class TestCanonicalContactOrder(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()
        cls.device = os.environ.get("NEWTON_GPU_STAGE_DEVICE", "cpu")

    def fixture(self):
        model = mj.MjModel.from_xml_string(CONNECT_XML.replace('jacobian="dense"', 'jacobian="sparse"'))
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        with wp.ScopedDevice(self.device):
            backend_model = mw.put_model(model)
            backend_data = mw.put_data(model, data, nconmax=8, njmax=32)
        return FixtureSolver(model, backend_model, backend_data)

    @staticmethod
    def snapshot(data):
        return {name: getattr(data.contact, name).numpy().copy() for name in _CONTACT_FIELDS}

    @staticmethod
    def row_bytes(snapshot, index):
        return b"".join(snapshot[field][index].tobytes() for field in _CONTACT_FIELDS
                        if snapshot[field].shape[0] > 0)

    def populate(self, run):
        rng = np.random.default_rng(31)
        data = run.mjw_data
        capacity = data.naconmax
        for name in _CONTACT_FIELDS:
            target = getattr(data.contact, name)
            values = target.numpy()
            if values.dtype.kind in "iu":
                values[:] = rng.integers(0, 3, size=values.shape)
            else:
                values[:] = rng.normal(size=values.shape)
            target.assign(values)
        # Exercise key order for signed zeros; raw bits must be preserved.
        pos = data.contact.pos.numpy()
        pos[0, 0], pos[1, 0] = np.float32(-0.), np.float32(+0.)
        data.contact.pos.assign(pos)
        data.nacon.assign(np.array([capacity - 2], np.int32))
        return capacity - 2

    def test_all_contact_fields_are_an_exact_permutation_and_inactive_tail_untouched(self):
        run = self.fixture()
        count = self.populate(run)
        before = self.snapshot(run.mjw_data)
        adapter = CanonicalContactOrder(run)
        try:
            with wp.ScopedDevice(self.device):
                adapter.sort_contacts()
            after = self.snapshot(run.mjw_data)
            self.assertEqual(sorted(self.row_bytes(before, i) for i in range(count)),
                             sorted(self.row_bytes(after, i) for i in range(count)))
            for name in _CONTACT_FIELDS:
                self.assertEqual(before[name][count:].tobytes(), after[name][count:].tobytes(), name)
        finally:
            adapter.close()

    def test_arbitrary_slot_permutations_produce_identical_canonical_payload(self):
        run = self.fixture()
        count = self.populate(run)
        original = self.snapshot(run.mjw_data)
        adapter = CanonicalContactOrder(run)
        try:
            with wp.ScopedDevice(self.device):
                adapter.sort_contacts()
            expected = self.snapshot(run.mjw_data)
            for permutation in (np.arange(count)[::-1], np.random.default_rng(7).permutation(count)):
                for name in _CONTACT_FIELDS:
                    values = original[name].copy()
                    if values.shape[0] > 0:
                        values[:count] = values[permutation]
                    getattr(run.mjw_data.contact, name).assign(values)
                with wp.ScopedDevice(self.device):
                    adapter.sort_contacts()
                actual = self.snapshot(run.mjw_data)
                for name in _CONTACT_FIELDS:
                    self.assertEqual(actual[name].tobytes(), expected[name].tobytes(), name)
        finally:
            adapter.close()

    def test_velocity_cache_callback_is_preserved_once_after_sort_and_close_restores_it(self):
        run = self.fixture()
        order = []
        run.mjw_model.callback.contactfilter = lambda _m, _d: order.append("existing filter")
        install_connect_velocity_cache_refresh(run.mjw_model, run.mjw_data)
        original_callback = run.mjw_model.callback.contactfilter
        adapter = CanonicalContactOrder(run)
        try:
            with mock.patch.object(adapter, "sort_contacts", side_effect=lambda: order.append("sort")), \
                 mock.patch.object(mw, "com_vel", side_effect=lambda _m, _d: order.append("current velocity")):
                run.mjw_model.callback.contactfilter(run.mjw_model, run.mjw_data)
            self.assertEqual(order, ["existing filter", "sort", "current velocity"])
        finally:
            adapter.close()
        self.assertIs(run.mjw_model.callback.contactfilter, original_callback)

    def test_zero_contact_forward_preserves_connect_reference_and_integration_state(self):
        run = self.fixture()
        velocity = np.linspace(-1., 1., run.mj_model.nv).astype(np.float32)[None]
        run.mjw_data.qvel.assign(velocity)
        install_connect_velocity_cache_refresh(run.mjw_model, run.mjw_data)
        with wp.ScopedDevice(self.device):
            mw.forward(run.mjw_model, run.mjw_data)
        reference = run.mjw_data.efc.aref.numpy().copy()
        qpos = run.mjw_data.qpos.numpy().copy()
        adapter = CanonicalContactOrder(run)
        try:
            with wp.ScopedDevice(self.device):
                mw.forward(run.mjw_model, run.mjw_data)
            np.testing.assert_array_equal(run.mjw_data.efc.aref.numpy(), reference)
            np.testing.assert_array_equal(run.mjw_data.qpos.numpy(), qpos)
            np.testing.assert_array_equal(run.mjw_data.qvel.numpy(), velocity)
        finally:
            adapter.close()


if __name__ == "__main__":
    unittest.main()
