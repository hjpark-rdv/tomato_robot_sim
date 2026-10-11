import unittest
import numpy as np

from truss_sequence import select_group,planning_fingerprint
from row_harvest_targets import select_scoped_group


class RejectedPlanningAlternativeTests(unittest.TestCase):
    def setUp(self):
        self.fruits=[dict(name=f'Truss_02_Fruit_{i:02}',path=f'/fruit/{i}',joint=f'/joint/{i}')for i in (1,2)]
        self.paths=[f['path']for f in self.fruits]
        self.poses=np.array([[0,0,1,0,0,0,1],[.02,0,1.01,0,0,0,1]])

    def test_default_exact_pair_and_rejected_pair_tries_each_individual_once(self):
        for selector in (select_group,select_scoped_group):
            old=selector(self.fruits,[],self.paths,self.poses,{})
            self.assertEqual(old,selector(self.fruits,[],self.paths,self.poses,{},rejected_groups=()))
            rejected=[tuple(old['fruits'])]
            first=selector(self.fruits,[],self.paths,self.poses,{},rejected_groups=rejected)
            self.assertEqual(first['fruits'],[self.fruits[0]['name']])
            np.testing.assert_array_equal(first['measured_centroid_world_m'],self.poses[0,:3])
            self.assertTrue(first['planning_rejection_alternative']['centering_changed_to_individual'])
            self.assertEqual(first['planning_rejection_alternative']['parent_group'],old['fruits'])
            rejected.append(tuple(first['fruits']))
            second=selector(self.fruits,[],self.paths,self.poses,{},rejected_groups=rejected)
            self.assertEqual(second['fruits'],[self.fruits[1]['name']])
            rejected.append(tuple(second['fruits']))
            self.assertIsNone(selector(self.fruits,[],self.paths,self.poses,{},rejected_groups=rejected))
            # A caller clears the rejected set after physical state changes.
            self.assertEqual(old,selector(self.fruits,[],self.paths,self.poses,{},rejected_groups=()))

    def test_scoped_failures_never_expand_contacts_to_a_different_truss(self):
        other=dict(name='Truss_01_Fruit_01',path='/other',joint='/joint/other')
        fruits=self.fruits+[other];paths=self.paths+[other['path']]
        poses=np.vstack([self.poses,[0,0,2,0,0,0,1]])
        rejected=[tuple(f['name']for f in self.fruits),*( (f['name'],) for f in self.fruits)]
        selected=select_scoped_group(fruits,[],paths,poses,{},rejected_groups=rejected)
        self.assertEqual(selected['fruits'],[other['name']])
        self.assertEqual(selected['truss_key'],'Truss_01')

    def test_exact_fingerprint_changes_with_actual_state_or_any_planning_parameter(self):
        group=select_group(self.fruits,[],self.paths,self.poses,{})
        q=np.zeros(7);axis=dict(direction_world=[1,0,0]);params=dict(depth_m=.065,gap_m=.028,lift_offset_m=-.15)
        digest=planning_fingerprint(group,q,axis,params)
        self.assertEqual(digest,planning_fingerprint(group,q,axis,dict(reversed(list(params.items())))))
        changed_q=q.copy();changed_q[0]=1e-9
        self.assertNotEqual(digest,planning_fingerprint(group,changed_q,axis,params))
        moved=dict(group,measured_centroid_world_m=[1e-9,0,1.005])
        self.assertNotEqual(digest,planning_fingerprint(moved,q,axis,params))
        self.assertNotEqual(digest,planning_fingerprint(group,q,dict(direction_world=[0,1,0]),params))
        self.assertNotEqual(digest,planning_fingerprint(group,q,axis,dict(params,depth_m=.066)))
        with self.assertRaises(ValueError):planning_fingerprint(group,[float('nan')]*7,axis,params)
        with self.assertRaises(ValueError):planning_fingerprint(group,q,axis,dict(force=float('nan')))

    def test_unknown_duplicate_or_string_rejections_are_rejected(self):
        for groups in ([('unknown',)],[(self.fruits[0]['name'],)*2],[self.fruits[0]['name']]):
            for selector in (select_group,select_scoped_group):
                with self.assertRaises(ValueError):selector(self.fruits,[],self.paths,self.poses,{},rejected_groups=groups)


if __name__=='__main__':unittest.main()
