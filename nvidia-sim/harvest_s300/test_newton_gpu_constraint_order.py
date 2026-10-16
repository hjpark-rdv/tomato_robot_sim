"""Constraint payload/group/contact permutation controls, including capture."""
import os
import unittest
from types import SimpleNamespace
import numpy as np
import warp as wp
from newton_gpu_constraint_order import CanonicalConstraintOrder, Rows


class ConstraintOrderTests(unittest.TestCase):
    def check_permutation(self, device):
        with wp.ScopedDevice(device):
            owner = CanonicalConstraintOrder.__new__(CanonicalConstraintOrder)
            owner.device = device
            owner.live, owner.scratch = Rows(), Rows()
            integers = {'type', 'id', 'J_rownnz', 'J_rowadr', 'state', 'island'}
            original = {}
            for i, field in enumerate(Rows.vars):
                dtype = int if field in integers else float
                values = (np.arange(8) + i*100).reshape(1, 8).astype(np.int32 if dtype is int else np.float32)
                if field == 'type': values[:] = [7, 7, 7, 1, 0, 0, 0, 3]
                if field == 'id': values[:] = [5, 5, 5, 2, 8, 8, 8, 4]
                original[field] = values.copy()
                array = wp.array(values, dtype=dtype)
                setattr(owner.live, field, array)
                setattr(owner.scratch, field, wp.empty_like(array))
            owner.rank = wp.empty((1, 8), int)
            owner.group_adr, owner.group_nrow = [wp.zeros((1, 8), int) for _ in range(2)]
            owner.data = SimpleNamespace(nworld=1, njmax=8, nefc=wp.array([8], int),
                nacon=wp.array([1], int), efc=SimpleNamespace(
                    jtdaj_nblock=wp.array([4], int),
                    jtdaj_adr=wp.array([[0, 3, 4, 7, 0, 0, 0, 0]], int),
                    jtdaj_nrow=wp.array([[3, 1, 3, 1, 0, 0, 0, 0]], int)),
                contact=SimpleNamespace(worldid=wp.array([0], int),
                    efc_address=wp.array([[0, -1, -1]], int)))
            owner.sort()
            order = [4, 5, 6, 3, 7, 0, 1, 2]
            for field in Rows.vars:
                np.testing.assert_array_equal(getattr(owner.live, field).numpy(), original[field][:, order])
            np.testing.assert_array_equal(owner.data.efc.jtdaj_adr.numpy()[0, :4], [0, 3, 4, 5])
            np.testing.assert_array_equal(owner.data.efc.jtdaj_nrow.numpy()[0, :4], [3, 1, 1, 3])
            np.testing.assert_array_equal(owner.data.contact.efc_address.numpy(), [[5, -1, -1]])
            if device != 'cpu':
                with wp.ScopedCapture(device=device) as capture:
                    owner.sort()
                for _ in range(3): wp.capture_launch(capture.graph)
            else:
                owner.sort()
            for field in Rows.vars:
                np.testing.assert_array_equal(getattr(owner.live, field).numpy(), original[field][:, order])
            np.testing.assert_array_equal(owner.data.contact.efc_address.numpy(), [[5, -1, -1]])

    def test_cpu_complete_payload_groups_contact_and_idempotence(self):
        self.check_permutation('cpu')

    @unittest.skipUnless(os.getenv('S300_CONSTRAINT_ORDER_CUDA_TEST') == '1', 'Explicit CUDA opt-in')
    def test_cuda_complete_payload_and_graph_idempotence(self):
        self.check_permutation('cuda:0')


if __name__ == '__main__': unittest.main()
