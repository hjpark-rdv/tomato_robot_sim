"""Contact evidence regression tests; no Isaac process or GPU required."""
import unittest
import numpy as np

from motion_contacts import ContactDataIncomplete, TensorContactReader, _cpu_id_tensor


class FakeContacts:
    sensor_paths = ['/World/A', '/World/B']
    sensor_count = 2

    def __init__(self):
        self.ids = {11:'/World/A',22:'/World/B',33:'/World/Gutter'}
        self.lookups = []
        self.counts = np.array([3,2], dtype=np.uint32)
        self.starts = np.array([0,3], dtype=np.uint32)
        self.actor_ids = np.array([22,22,33,11,11] + [0]*11, dtype=np.uint64)
        self.forces = np.array([2.,4.,3.,2.,4.] + [0.]*11, dtype=np.float32)[:,None]
        self.separations = np.array([-.0006,.001,.002,-.0007,.001] + [-999.]*11, dtype=np.float32)[:,None]
        self.points = np.zeros((16,3), dtype=np.float32)
        self.points[1] = [0.,1.,0.];self.points[4] = self.points[1]
        self.normals = np.tile([1.,0.,0.], (16,1)).astype(np.float32)
        self.normals[3:5] *= -1

    def get_raw_contact_data(self, dt):
        self.last_dt = dt
        return self.forces,self.points,self.normals,self.separations,self.counts,self.starts,self.actor_ids

    def get_other_actor_paths_from_ids(self, ids):
        assert isinstance(ids,np.ndarray) and ids.dtype == np.uint64
        self.lookups.append(ids.tolist())
        return [self.ids.get(int(i),'') for i in ids]


class FakeSimulationView:
    def __init__(self, contacts):self.contacts=contacts
    def create_rigid_contact_view(self, paths, max_contact_data_count):
        self.paths=paths;self.capacity=max_contact_data_count
        return self.contacts


class ContactReaderTests(unittest.TestCase):
    def reader(self):
        data=FakeContacts()
        reader=TensorContactReader(FakeSimulationView(data),data.sensor_paths,max_contact_data_count=16)
        return reader,data

    def test_all_points_without_double_counting_sensors(self):
        reader,data=self.reader();events=reader.read(t=.25,phase='approach',dt=1/240)
        self.assertEqual(len(events),3)
        self.assertEqual(reader.last_summary['duplicate_sensor_points_removed'],2)
        self.assertEqual(sum(e['force_n'] for e in events),9.)
        self.assertEqual(events[0]['t'],.25)
        self.assertEqual(events[0]['phase'],'approach')
        self.assertEqual(data.last_dt,1/240)
        self.assertFalse(events[0]['collider_identity_known'])
        self.assertEqual(reader.last_summary['normal_force_sums_by_actor_pair_n'][0]['normal_force_sum_n'],6.)
        self.assertLess(reader.last_summary['raw_minimum_separation_m'],-.0005)
        self.assertLess(reader.last_summary['raw_minimum_separation_m'],reader.last_summary['minimum_separation_m'])

    def test_stale_tail_is_ignored_and_actor_cache_reused(self):
        reader,data=self.reader();reader.read(t=0.,phase='idle',dt=.01)
        reader.read(t=.01,phase='idle',dt=.01)
        self.assertEqual(len(data.lookups),1)
        self.assertGreater(reader.last_summary['raw_minimum_separation_m'],-1.)
        reader.clear_actor_cache();reader.read(t=.02,phase='idle',dt=.01)
        self.assertEqual(len(data.lookups),2)

    def test_unknown_actor_is_explicit_and_incomplete(self):
        reader,data=self.reader();del data.ids[33]
        events=reader.read(t=0.,phase='idle',dt=.01)
        self.assertFalse(reader.last_summary['complete'])
        self.assertEqual(reader.last_summary['unknown_actor_ids'],[33])
        self.assertEqual(events[-1]['b'],'UNKNOWN_ACTOR_ID:33')
        self.assertFalse(events[-1]['actor_identity_known'])

    def test_saturated_and_overlapping_buffers_rejected(self):
        reader,data=self.reader();data.counts[:]=[8,8];data.starts[:]=[0,8]
        with self.assertRaises(ContactDataIncomplete):reader.read(t=0.,phase='idle',dt=.01)
        reader,data=self.reader();data.starts[:]=[0,2]
        with self.assertRaises(ContactDataIncomplete):reader.read(t=0.,phase='idle',dt=.01)

    def test_zero_contacts_and_nonfinite_evidence(self):
        reader,data=self.reader();data.counts[:]=0
        data.starts[:]=np.iinfo(np.uint32).max
        self.assertEqual(reader.read(t=0.,phase='idle',dt=.01),[])
        self.assertTrue(reader.last_summary['complete'])
        reader,data=self.reader();data.forces[0]=np.nan
        with self.assertRaises(ContactDataIncomplete):reader.read(t=0.,phase='idle',dt=.01)

    def test_sensor_inventory_mismatch_rejected(self):
        data=FakeContacts()
        with self.assertRaises(ContactDataIncomplete):
            TensorContactReader(FakeSimulationView(data),['/World/A','/World/Other'])

    def test_installed_torch_unsigned_actor_storage(self):
        try:
            import torch
        except ImportError:
            self.skipTest('Torch is not installed in this offline asset environment')
        values=np.array([7,2**63+5,2**64-1],dtype=np.uint64)
        ids=_cpu_id_tensor(torch.zeros(3,dtype=torch.int64),values)
        self.assertEqual(ids.dtype,torch.int64)
        np.testing.assert_array_equal(ids.numpy().view(np.uint64),values)


if __name__ == '__main__':unittest.main()
