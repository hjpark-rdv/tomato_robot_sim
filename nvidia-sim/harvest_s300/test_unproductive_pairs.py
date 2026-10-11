"""Physical no-detachment evidence changes centering, never reachability labels."""
import copy
import unittest
import numpy as np

from truss_sequence import select_group,unproductive_pairs


class UnproductivePairsTest(unittest.TestCase):
    def setUp(self):
        self.names=['Truss_02_Fruit_01','Truss_02_Fruit_02','Truss_02_Fruit_03']
        self.joints={n:'/World/Plant/HarvestJoints/'+n for n in self.names}
        self.cycle=dict(fruits=self.names[:2],status='motion_completed',start_s=2.,
            planned_end_s=54.,end_s=54.,measured_frame_index=0,end_frame_index=6480,
            inventory_after=dict(remaining=self.names.copy(),detached=[]))

    def test_exact_completed_pair_is_canonical_and_deduplicated(self):
        other=copy.deepcopy(self.cycle);other['fruits'].reverse()
        self.assertEqual(unproductive_pairs([self.cycle,other],self.joints),{tuple(self.names[:2])})

    def test_one_or_both_selected_fruits_detached_is_productive(self):
        for detached in (self.names[:1],self.names[:2]):
            with self.subTest(detached=detached):
                cycle=copy.deepcopy(self.cycle)
                cycle['inventory_after']=dict(detached=detached,remaining=[n for n in self.names if n not in detached])
                self.assertEqual(unproductive_pairs([cycle],self.joints),set())

    def test_other_fruit_detachment_does_not_hide_missed_pair(self):
        self.cycle['inventory_after']=dict(detached=self.names[2:],remaining=self.names[:2])
        self.assertEqual(unproductive_pairs([self.cycle],self.joints),{tuple(self.names[:2])})

    def test_only_completed_physics_records_qualify(self):
        for status in ('stopped','planning_rejected','planning','executing',None):
            with self.subTest(status=status):
                cycle=dict(self.cycle,status=status)
                self.assertEqual(unproductive_pairs([cycle],self.joints),set())
        self.assertEqual(unproductive_pairs([dict(self.cycle,stop_reason='penetration_over_0_5mm')],self.joints),set())

    def test_timestamps_and_actual_sample_progression_are_required(self):
        overrides=[dict(end_s=1.),dict(end_s=float('nan')),dict(end_s=53.),
                   dict(end_s=True),dict(end_frame_index=0),dict(end_frame_index=None),
                   dict(measured_frame_index=-1),dict(measured_frame_index=True)]
        for override in overrides:
            with self.subTest(override=override):
                self.assertEqual(unproductive_pairs([dict(self.cycle,**override)],self.joints),set())

    def test_missing_unknown_duplicate_or_contradictory_inventory_is_ignored(self):
        inventories=[None,dict(remaining=self.names),dict(remaining=self.names[:2],detached=[]),
            dict(remaining=self.names+['unknown'],detached=[]),
            dict(remaining=self.names+self.names[:1],detached=[]),
            dict(remaining=self.names,detached=self.names[:1])]
        for after in inventories:
            with self.subTest(after=after):
                self.assertEqual(unproductive_pairs([dict(self.cycle,inventory_after=after)],self.joints),set())

    def test_before_inventory_if_present_must_confirm_attached_pair(self):
        before=dict(remaining=self.names.copy(),detached=[])
        self.assertEqual(unproductive_pairs([dict(self.cycle,inventory_before=before)],self.joints),{tuple(self.names[:2])})
        before=dict(remaining=self.names[1:],detached=self.names[:1])
        self.assertEqual(unproductive_pairs([dict(self.cycle,inventory_before=before)],self.joints),set())
        self.assertEqual(unproductive_pairs([dict(self.cycle,inventory_before=None)],self.joints),set())

    def test_unknown_single_duplicate_and_cross_truss_groups_are_ignored(self):
        cases=[self.names[:1],[self.names[0],self.names[0]],[self.names[0],'unknown']]
        for pair in cases:
            self.assertEqual(unproductive_pairs([dict(self.cycle,fruits=pair)],self.joints),set())
        other='Truss_01_Fruit_02';joints=dict(self.joints,**{other:'/World/Plant/HarvestJoints/'+other})
        cycle=dict(self.cycle,fruits=[self.names[0],other],inventory_after=dict(remaining=list(joints),detached=[]))
        self.assertEqual(unproductive_pairs([cycle],joints),set())

    def test_joint_identity_must_be_unique(self):
        joints=dict(self.joints);joints[self.names[1]]=joints[self.names[0]]
        for bad in (joints,{},[],{self.names[0]:None}):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):unproductive_pairs([self.cycle],bad)

    def test_existing_selector_defaults_unchanged_and_opt_in_splits_exact_pair(self):
        fruits=[dict(name=n,joint=self.joints[n],path='/fruit/'+n)for n in self.names]
        paths=[f['path']for f in fruits]
        poses=np.array([[0.,0.,1.2,0,0,0,1],[.03,0.,1.19,0,0,0,1],[.02,0.,1.,0,0,0,1]])
        attempts={n:1 for n in self.names}
        unchanged=select_group(fruits,[],paths,poses,attempts,order='highest-first')
        self.assertEqual(unchanged['fruits'],self.names[:2])
        split=select_group(fruits,[],paths,poses,attempts,order='highest-first',
            rejected_groups=unproductive_pairs([self.cycle],self.joints))
        self.assertEqual(split['fruits'],self.names[:1])
        np.testing.assert_array_equal(split['measured_centroid_world_m'],poses[0,:3])
        self.assertTrue(split['planning_rejection_alternative']['centering_changed_to_individual'])
        self.assertEqual(attempts,{n:1 for n in self.names})


if __name__=='__main__':unittest.main()
