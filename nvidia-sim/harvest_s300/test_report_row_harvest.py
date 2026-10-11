import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from report_row_harvest import collect_results,write_report


class RowHarvestReportTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.assembly=self.root/'assembly';self.assembly.mkdir()
        self.trial=self.root/'trials_native/one';self.trial.mkdir(parents=True)
        self.names=['Truss_02_Fruit_01','Truss_02_Fruit_02']
        self.mapping={name:'/World/ElasticTarget/HarvestJoints/'+name for name in self.names}
        manifest=dict(plant=dict(source_plant='/World/Greenhouse/Plants/Plant_22',
            fruits=[dict(name=name,joint=joint) for name,joint in self.mapping.items()]),
            target=dict(name='Truss_02_Fruit_11'))
        (self.assembly/'motion_manifest.json').write_text(json.dumps(manifest))
        sequence=dict(fruit_names=self.names,fruit_joints=self.mapping,
            inventory=dict(detached=[self.names[0]],remaining=[self.names[1]]),termination='finite_plan_budget',
            cycle_plans=[dict(index=0,fruits=self.names,status='motion_completed'),
                         dict(index=1,fruits=[self.names[1]],status='planning_rejected',error='IK not found')])
        summary=dict(mode='harvest',scene=str(self.assembly/'scene.usda'),completed=True,stop_reason=None,
            simulated_s=5.,physics_wall_s=6.,native_target_detached=True,
            native_break_events=[dict(joint=self.mapping[self.names[0]],t=3.)],
            min_contact_separation_m=-.003,truss_sequence=sequence)
        (self.trial/'summary.json').write_text(json.dumps(summary))
        (self.trial/'leaf_contact_resolution.json').write_text(json.dumps([
            dict(certified_within_limit=True,penetration_upper_bound_m=.0001)]))
        (self.trial/'contact_lifecycle_scope.json').write_text(json.dumps(dict(
            enabled=True,deferred_invalid_contact_points=2,strict_0_5mm_passed=False)))

    def tearDown(self):self.tmp.cleanup()

    def test_exact_native_scope_partial_counts_and_measurement_times_stay_separate(self):
        report=write_report(self.root);row=report['trials'][0];inv=row['native_inventory']
        # Default summary target11 must not replace the selected 01/02 mapping.
        self.assertEqual(inv['detached_count'],1);self.assertEqual(inv['total_count'],2)
        self.assertEqual(inv['remaining'],[self.names[1]])
        self.assertFalse(inv['all_selected_joints_detached']);self.assertFalse(inv['collection_verified'])
        self.assertEqual(len(row['failed_cycle_plans']),1)
        self.assertEqual(row['raw_callback_max_penetration_mm'],3.)
        self.assertEqual(row['active_harvest_guard']['max_certified_pair_bound_mm'],.1)
        self.assertFalse(row['released_collection_diagnostics']['strict_full_scene_0_5mm_passed'])
        page=(self.root/'index.html').read_text();self.assertIn('분리 1/2',page)
        self.assertIn('IK not found',page);self.assertIn('호스트에서 여는 주소',page)
        self.assertFalse(report['training_eligible']);self.assertIsNone(report['harvest_success'])

    def test_duplicate_joint_mapping_and_future_event_do_not_inflate_detachment(self):
        path=self.trial/'summary.json';summary=json.loads(path.read_text())
        summary['truss_sequence']['fruit_joints'][self.names[1]]=self.mapping[self.names[0]]
        path.write_text(json.dumps(summary))
        inv=collect_results(self.root)['trials'][0]['native_inventory']
        self.assertFalse(inv['exact_mapping_valid']);self.assertIsNone(inv['detached_count'])
        summary['truss_sequence']['fruit_joints']=self.mapping
        summary['native_break_events'].append(dict(joint=self.mapping[self.names[1]],t=6.))
        path.write_text(json.dumps(summary));inv=collect_results(self.root)['trials'][0]['native_inventory']
        self.assertEqual(inv['detached_count'],1);self.assertEqual(len(inv['invalid_timestamp_events']),1)

    def test_actual_preparation_stop_correction_does_not_rewrite_original_summary(self):
        summary_path=self.trial/'summary.json'
        original=summary_path.read_bytes()
        (self.trial/'selected_cycle_evidence.json').write_text(json.dumps(dict(
            cycles=[dict(planned_manipulation_started=True)])))
        corrected=dict(cycles=[dict(planned_manipulation_started=False,
            actual_recorded_phases=['settle','prepare_joint_branch'],
            selected_fruits=[])])
        (self.trial/'selected_cycle_evidence_phase_corrected.json').write_text(json.dumps(corrected))
        result=collect_results(self.root)['trials'][0]
        self.assertFalse(result['selected_cycle_evidence']['cycles'][0]['planned_manipulation_started'])
        self.assertIn('selected_cycle_evidence_phase_corrected.json',result['evidence'])
        self.assertEqual(summary_path.read_bytes(),original)

    def test_legacy_target_detached_is_explicitly_labelled_separately_from_selected_fruit(self):
        manifest_path=self.assembly/'motion_manifest.json';manifest=json.loads(manifest_path.read_text())
        root='/World/ElasticTarget';manifest['plant']['root']=root
        for fruit in manifest['plant']['fruits']:fruit['path']=root+'/'+fruit['name']
        legacy=dict(name='Truss_02_Fruit_11',path=root+'/Truss_02_Fruit_11',joint=root+'/HarvestJoints/Truss_02_Fruit_11')
        manifest['plant']['fruits'].append(legacy);manifest_path.write_text(json.dumps(manifest))
        path=self.trial/'summary.json';summary=json.loads(path.read_text());summary['target_displacement_max_m']=3e-7
        summary['native_break_events']=[dict(joint=legacy['joint'],t=0.,type='imported_native_prior',native_new_event=False)]
        summary['truss_sequence']['cycle_plans']=[dict(index=0,fruits=[self.names[1]],status='motion_completed',start_s=2.,end_s=5.)]
        path.write_text(json.dumps(summary));report=write_report(self.root);row=report['trials'][0]
        self.assertTrue(row['legacy_manifest_target_fields']['native_target_detached'])
        selected=row['selected_cycle_evidence']['cycles'][0]['selected_fruits'][0]
        self.assertEqual(selected['name'],self.names[1]);self.assertFalse(selected['native_detached_at_cycle_end'])
        self.assertIsNone(selected['displacement_max_from_measured_cycle_baseline_m'])
        page=(self.root/'index.html').read_text();self.assertIn('원래 manifest 타깃 Truss_02_Fruit_11',page)
        self.assertIn('다른 회차의 조준 열매 결과로 해석하지 않습니다',page)

    def test_actual_rail_measurement_uses_native_dof_mapping(self):
        manifest_path=self.assembly/'motion_manifest.json';manifest=json.loads(manifest_path.read_text())
        manifest['rail']=dict(dof_name='root_joint',initial_station_y_m=1.6)
        manifest_path.write_text(json.dumps(manifest))
        (self.trial/'native_metadata.json').write_text(json.dumps(dict(robot_dofs=['jaw_L','root_joint','jaw_R'])))
        np.savez_compressed(self.trial/'states.npz',time=[0.,2.],robot_q=[[.02,0.,.02],[.02,.2,.02]])
        rail=collect_results(self.root)['trials'][0]['rail']
        self.assertAlmostEqual(rail['measured_travel_m'],.2)
        self.assertAlmostEqual(rail['final_station_y_m'],1.8)
        self.assertAlmostEqual(rail['maximum_measured_speed_m_s'],.1)

    def test_initial_settle_breaks_are_not_harvest_and_checkpoint_breaks_are_prior_evidence(self):
        path=self.trial/'summary.json';summary=json.loads(path.read_text())
        summary['native_break_events'][0]['phase']='settle'
        path.write_text(json.dumps(summary));row=write_report(self.root)['trials'][0]
        self.assertEqual(row['status'],'initial_state_failure')
        self.assertEqual(row['native_inventory']['initial_uncommanded_detached_count'],1)
        self.assertFalse(row['native_inventory']['harvest_evidence_usable'])
        self.assertIn('수확 증거 제외',(self.root/'index.html').read_text())
        event=summary['native_break_events'][0]
        event.update(t=0.,phase='belt_close',type='imported_native_prior',native_new_event=False,original_t=45.)
        summary['native_break_events'].append(dict(joint=self.mapping[self.names[1]],t=4.,phase='belt_close'))
        path.write_text(json.dumps(summary));inv=write_report(self.root)['trials'][0]['native_inventory']
        self.assertEqual(inv['detached_count'],2)
        self.assertEqual(inv['imported_prior_detached_count'],1)
        self.assertEqual(inv['new_native_detached_count'],1)
        self.assertTrue(inv['harvest_evidence_usable'])

    def test_declared_campaign_nested_trial_is_measured_and_blocked_stations_retained(self):
        campaign=self.root/'row_campaign_native';campaign.mkdir()
        name='station_000_plant_22';nested=campaign/'trials'/name/name;nested.mkdir(parents=True)
        (nested/'summary.json').write_bytes((self.trial/'summary.json').read_bytes())
        (nested.parent/'station_results.json').write_text(json.dumps([
            dict(name=name,returncode=0,total_wall_s=7.)]))
        report=dict(schema='s300-physical-rail-row-campaign-v1',executed=True,
            status='blocked_preserving_last_physical_history',selection=dict(stations=[dict(plant_id=22),dict(plant_id=23)]),
            stations=[dict(index=0,plant_id=22,station_y_m=1.6,status='completed_physics',physics_execution=True),
                      dict(index=1,plant_id=23,station_y_m=2.,status='route_blocked',physics_execution=False,
                           blocker='No valid completed checkpoint')])
        (campaign/'row_campaign.json').write_text(json.dumps(report))
        result=write_report(self.root);self.assertEqual(len(result['trials']),2)
        trial=next(row for row in result['trials'] if row['trial'].startswith('row_campaign_native/'))
        self.assertEqual(trial['returncode'],0);self.assertEqual(trial['native_inventory']['detached_count'],1)
        stations=result['row_campaigns'][0]['stations'];self.assertEqual(len(stations),2)
        self.assertIsNone(stations[1]['trial']);self.assertFalse(stations[1]['physics_execution'])
        page=(self.root/'index.html').read_text()
        self.assertIn('No valid completed checkpoint',page);self.assertIn('PhysX 재시작',page)

    def test_final_write_error_keeps_saved_partial_native_sequence_but_not_completion(self):
        path=self.trial/'summary.json';summary=json.loads(path.read_text());path.unlink()
        sequence=summary['truss_sequence'];sequence['native_break_events']=summary['native_break_events']+[
            dict(joint=self.mapping[self.names[1]],t=6.)]
        (self.trial/'truss_sequence.json').write_text(json.dumps(sequence))
        (self.trial/'native_stage_selection.json').write_text(json.dumps(dict(
            source_scene=summary['scene'],native_scene=str(self.assembly/'composed.usdc'))))
        (self.trial/'runtime_error.json').write_text(json.dumps(dict(error='evidence serialization failed')))
        np.savez_compressed(self.trial/'states.npz',time=[0.,5.],robot_q=[[0.],[0.]])
        row=collect_results(self.root)['trials'][0]
        self.assertEqual(row['status'],'runtime_error');self.assertIsNone(row['completed'])
        self.assertEqual(row['native_inventory']['detached_count'],1)
        self.assertEqual(len(row['native_inventory']['invalid_timestamp_events']),1)
        self.assertEqual(row['simulated_s'],5.)
        self.assertFalse(row['active_harvest_guard']['completed_without_penetration_stop'])

    def test_nested_route_initialization_failure_is_reported_without_harvest_credit(self):
        folder=self.root/'route_window_controls'/'trials_native_idle'/'control';folder.mkdir(parents=True)
        (folder/'runtime_settings.json').write_text(json.dumps(dict(device='cpu')))
        (folder/'native_stage_selection.json').write_text(json.dumps(dict(source_scene=str(self.assembly/'scene.usda'))))
        (folder/'initialization_failure.json').write_text(json.dumps(dict(valid=False,
            reason='new_native_joint_break_during_warmup',native_break_events=[
                dict(joint=self.mapping[self.names[0]],t=0.,phase='idle')])))
        (folder/'runtime_error.json').write_text(json.dumps(dict(error='initialization failed')))
        rows=collect_results(self.root)['trials'];row=next(r for r in rows if 'route_window_controls' in r['trial'])
        self.assertEqual(row['status'],'initial_state_failure')
        self.assertIsNone(row['native_inventory']);self.assertIsNone(row['completed'])
        self.assertEqual(row['stop_reason'],'new_native_joint_break_during_warmup')
        self.assertIn('initialization_failure.json',row['evidence'])


if __name__=='__main__':unittest.main()
