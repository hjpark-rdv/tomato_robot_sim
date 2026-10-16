"""Constraint payload/group/contact permutation controls, including capture."""
import os
import unittest
from types import SimpleNamespace
import numpy as np
import warp as wp
from newton_gpu_constraint_order import CanonicalConstraintOrder, Rows


class ConstraintOrderTests(unittest.TestCase):
    def check_multiworld(self, device, worlds):
        """Mixed row counts, ordering and world ids must never share addresses."""
        with wp.ScopedDevice(device):
            owner = CanonicalConstraintOrder.__new__(CanonicalConstraintOrder)
            owner.device = device
            owner.live, owner.scratch = Rows(), Rows()
            integers = {'type', 'id', 'J_rownnz', 'J_rowadr', 'state', 'island'}
            counts = np.array([8, 5, 0, 3][:worlds], np.int32)
            original, orders = {}, []
            types = np.array([[7, 7, 7, 1, 0, 0, 0, 3],
                              [5, 2, 2, 9, 0, 8, 6, 7],
                              [1, 3, 2, 0, 9, 5, 2, 6],
                              [3, 1, 2, 9, 8, 7, 6, 5]], np.int32)[:worlds]
            ids = np.array([[5, 5, 5, 2, 8, 8, 8, 4],
                            [8, 7, 3, 2, 1, 0, 9, 4],
                            [0, 1, 2, 3, 4, 5, 6, 7],
                            [5, 4, 3, 2, 1, 0, 9, 8]], np.int32)[:worlds]
            for w, count in enumerate(counts):
                orders.append(sorted(range(count), key=lambda row: (types[w, row], ids[w, row], row)))
            for i, field in enumerate(Rows.vars):
                dtype = int if field in integers else float
                values = (np.arange(worlds*8).reshape(worlds, 8) + i*100).astype(
                    np.int32 if dtype is int else np.float32)
                if field == 'type': values[:] = types
                if field == 'id': values[:] = ids
                original[field] = values.copy()
                array = wp.array(values, dtype=dtype)
                setattr(owner.live, field, array)
                setattr(owner.scratch, field, wp.empty_like(array))
            owner.rank = wp.empty((worlds, 8), int)
            owner.group_adr, owner.group_nrow = [wp.zeros((worlds, 8), int) for _ in range(2)]
            addresses = [[0, max(0, int(count)-1), -1] for count in counts]
            if worlds > 2: addresses[2] = [-1, -1, -1]
            owner.data = SimpleNamespace(nworld=worlds, njmax=8, nefc=wp.array(counts, int),
                nacon=wp.array([worlds], int), efc=SimpleNamespace(
                    jtdaj_nblock=wp.array(counts, int),
                    jtdaj_adr=wp.array(np.tile(np.arange(8), (worlds, 1)), int),
                    jtdaj_nrow=wp.ones((worlds, 8), int)),
                contact=SimpleNamespace(worldid=wp.array(np.arange(worlds), int),
                    efc_address=wp.array(addresses, int)))
            expected_addresses = np.array(addresses)
            for w, order in enumerate(orders):
                for c, old in enumerate(addresses[w]):
                    if 0 <= old < counts[w]: expected_addresses[w, c] = order.index(old)
            owner.sort()
            for field in Rows.vars:
                actual = getattr(owner.live, field).numpy()
                for w, order in enumerate(orders):
                    np.testing.assert_array_equal(actual[w, :counts[w]], original[field][w, order])
            for w, count in enumerate(counts):
                np.testing.assert_array_equal(owner.data.efc.jtdaj_adr.numpy()[w, :count], np.arange(count))
            np.testing.assert_array_equal(owner.data.contact.efc_address.numpy(), expected_addresses)
            if device != 'cpu':
                with wp.ScopedCapture(device=device) as capture:
                    owner.sort()
                for _ in range(3): wp.capture_launch(capture.graph)
            else:
                owner.sort()
            for field in Rows.vars:
                actual = getattr(owner.live, field).numpy()
                for w, order in enumerate(orders):
                    np.testing.assert_array_equal(actual[w, :counts[w]], original[field][w, order])
            np.testing.assert_array_equal(owner.data.contact.efc_address.numpy(), expected_addresses)

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

    def test_cpu_multiworld_independent_counts_and_contacts(self):
        for worlds in (2, 4):
            with self.subTest(worlds=worlds): self.check_multiworld('cpu', worlds)

    @unittest.skipUnless(os.getenv('S300_CONSTRAINT_ORDER_CUDA_TEST') == '1', 'Explicit CUDA opt-in')
    def test_cuda_complete_payload_and_graph_idempotence(self):
        self.check_permutation('cuda:0')

    @unittest.skipUnless(os.getenv('S300_CONSTRAINT_ORDER_CUDA_TEST') == '1', 'Explicit CUDA opt-in')
    def test_cuda_multiworld_graph_independent_counts_and_contacts(self):
        for worlds in (2, 4):
            with self.subTest(worlds=worlds): self.check_multiworld('cuda:0', worlds)


if __name__ == '__main__': unittest.main()
