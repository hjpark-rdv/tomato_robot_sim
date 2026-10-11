import copy,unittest
import numpy as np
from selected_cycle_evidence import build_selected_cycle_evidence

class SelectedCycleEvidenceTests(unittest.TestCase):
    def fixture(self):
        root='/World/ElasticTarget';fruits=[dict(name=f'Truss_02_Fruit_{i:02d}',path=f'{root}/Truss_02_Fruit_{i:02d}',joint=f'{root}/HarvestJoints/Truss_02_Fruit_{i:02d}') for i in (2,11)]
        manifest=dict(plant=dict(root=root,fruits=fruits),target=dict(name=fruits[1]['name']))
        cycle=dict(index=0,fruits=[fruits[0]['name']],start_s=2.,end_s=4.,status='motion_completed',measured_frame_index=0,end_frame_index=2)
        sequence=dict(cycle_plans=[cycle]);breaks=[dict(joint=fruits[1]['joint'],t=0.,type='imported_native_prior',native_new_event=False)]
        poses=np.zeros((3,2,7));poses[:,:,6]=1;poses[:,0,0]=[0,.005,.02]
        return manifest,sequence,breaks,[f['path'] for f in fruits],poses,np.array([0.,2.,4.])

    def test_original_target_prior_release_never_becomes_selected_cycle_success(self):
        m,s,b,n,p,t=self.fixture();r=build_selected_cycle_evidence(m,s,b,n,p,t,legacy_summary=dict(native_target_detached=True,target_displacement_max_m=3e-7))
        row=r['cycles'][0]['selected_fruits'][0]
        self.assertTrue(r['legacy_summary_fields']['native_target_detached']);self.assertEqual(r['legacy_manifest_target']['name'],'Truss_02_Fruit_11')
        self.assertFalse(row['native_detached_at_cycle_end']);self.assertFalse(r['cycles'][0]['all_selected_native_joints_detached_at_cycle_end'])
        self.assertAlmostEqual(row['displacement_max_from_measured_cycle_baseline_m'],.02)
        self.assertFalse(r['cycles'][0]['legacy_manifest_target_is_selected']);self.assertIsNone(r['harvest_success'])

    def test_actual_selected_break_force_and_baseline_are_exact(self):
        m,s,b,n,p,t=self.fixture();b.append(dict(joint=m['plant']['fruits'][0]['joint'],t=3.,phase='close'))
        event=dict(t=3.,a=n[0]+'/Collider',b='/World/ActuatedTool/L/part_122',force_n=.8)
        row=build_selected_cycle_evidence(m,s,b,n,p,t,[event])['cycles'][0]['selected_fruits'][0]
        self.assertFalse(row['native_detached_at_cycle_start']);self.assertTrue(row['native_detached_at_cycle_end'])
        self.assertEqual(len(row['new_native_break_events_during_cycle']),1);self.assertEqual(row['max_single_contact_point_force_n'],.8)

    def test_rejected_plan_is_retained_without_physics_or_inferred_displacement(self):
        m,s,b,n,p,t=self.fixture();s['cycle_plans'][0].update(status='planning_rejected');s['cycle_plans'][0].pop('end_s')
        r=build_selected_cycle_evidence(m,s,b);c=r['cycles'][0];self.assertFalse(c['recorded_physical_cycle']);self.assertIsNone(c['all_selected_native_joints_detached_at_cycle_end']);self.assertIsNone(c['selected_fruits'][0]['displacement_max_from_measured_cycle_baseline_m'])

    def test_preparation_collision_after_cycle_start_is_not_manipulation(self):
        m,s,b,n,p,t=self.fixture();s['cycle_plans'][0]['status']='stopped'
        row=build_selected_cycle_evidence(m,s,b,n,p,t,measured_phases=['settle','prepare_gap','prepare_joint_branch'])['cycles'][0]
        self.assertFalse(row['planned_manipulation_started']);self.assertTrue(row['stopped_before_planned_manipulation'])
        self.assertEqual(row['manipulation_start_evidence'],'actual recorded phase names')
        self.assertFalse(row['all_selected_native_joints_detached_at_cycle_end'])

    def test_approach_phase_and_exact_trace_mapping(self):
        m,s,b,n,p,t=self.fixture()
        row=build_selected_cycle_evidence(m,s,b,n,p,t,measured_phases=['settle','approach','close'])['cycles'][0]
        self.assertTrue(row['planned_manipulation_started'])
        for bad in (['settle','approach'],['settle','','close']):
            with self.assertRaises(ValueError):build_selected_cycle_evidence(m,s,b,n,p,t,measured_phases=bad)
        with self.assertRaises(ValueError):build_selected_cycle_evidence(m,s,b,measured_phases=['settle'])

    def test_bad_body_joint_mapping_frame_or_nonfinite_trace_fail(self):
        m,s,b,n,p,t=self.fixture();bad=copy.deepcopy(m);bad['plant']['fruits'][0]['joint']+='wrong'
        with self.assertRaises(ValueError):build_selected_cycle_evidence(bad,s,b,n,p,t)
        bad=copy.deepcopy(s);bad['cycle_plans'][0]['end_frame_index']=1
        with self.assertRaises(ValueError):build_selected_cycle_evidence(m,bad,b,n,p,t)
        p[1,0,0]=np.nan
        with self.assertRaises(ValueError):build_selected_cycle_evidence(m,s,b,n,p,t)

    def test_none_empty_cycle_and_stopped_initial_settle_keep_prior_break_distinct(self):
        m,s,b,n,p,t=self.fixture()
        for sequence in (None,{},dict(cycle_plans=[])):
            self.assertEqual(build_selected_cycle_evidence(m,sequence,b,n,p,t)['cycles'],[])
        cycle=s['cycle_plans'][0];cycle.update(status='stopped',end_s=0.,end_frame_index=0)
        b.append(dict(joint=m['plant']['fruits'][0]['joint'],t=0.,type='imported_native_prior',native_new_event=False))
        row=build_selected_cycle_evidence(m,s,b,n,p,t)['cycles'][0]
        self.assertTrue(row['stopped_before_planned_manipulation']);self.assertFalse(row['planned_manipulation_started'])
        fruit=row['selected_fruits'][0];self.assertTrue(fruit['native_detached_at_cycle_end'])
        self.assertEqual(fruit['new_native_break_events_during_cycle'],[])
        self.assertEqual(fruit['displacement_max_from_measured_cycle_baseline_m'],0.)

if __name__=='__main__':unittest.main()
