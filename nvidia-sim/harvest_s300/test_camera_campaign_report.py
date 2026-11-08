import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from camera_campaign_report import collect, write_report


class CameraCampaignReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def trial(self, name, plant=22, imported=False, trace=True):
        folder = self.root / name; folder.mkdir(parents=True)
        assembly = self.root / ('assembly_' + str(plant)); assembly.mkdir(exist_ok=True)
        source = f'/World/Greenhouse/Plants/Plant_{plant:02d}'
        fruit = 'Truss_02_Fruit_01'; prior = 'Truss_02_Fruit_02'
        mapping = {n: '/World/ElasticTarget/HarvestJoints/' + n for n in (fruit, prior)}
        manifest = dict(plant=dict(root='/World/ElasticTarget', source_plant=source,
            fruits=[dict(name=n, path='/World/ElasticTarget/' + n, joint=j) for n, j in mapping.items()]),
            target=dict(name=fruit))
        (assembly / 'motion_manifest.json').write_text(json.dumps(manifest))
        region = dict(plant_source=source, observation=dict(timestamp_s=1., observation_path='/source/camera.json'),
            observable_eligible_fruits=[fruit, prior], accepted_observations=[dict(name=fruit)])
        sequence = dict(fruit_names=[fruit, prior], fruit_joints=mapping,
            inventory=dict(detached=[fruit, prior], remaining=[]), termination='time_budget_exhausted_remaining_fruit',
            cycle_plans=[dict(index=0, fruits=[], purpose='camera_scan_prefix', harvest_attempt=False,
                start_s=0., end_s=1., status='motion_completed', measured_frame_index=0, end_frame_index=1),
                dict(index=1, fruits=[prior], truss_key='Truss_02', status='planning_rejected',
                     start_s=1., error='IK not found', camera_region=region),
                dict(index=2, fruits=[fruit], truss_key='Truss_02', status='motion_completed',
                     start_s=1., end_s=4., measured_frame_index=1, end_frame_index=4, camera_region=region)])
        breaks = [dict(joint=mapping[prior], t=0., type='imported_native_prior', native_new_event=False),
                  dict(joint=mapping[fruit], t=3., phase='close')]
        if imported:
            breaks[1].update(t=0., type='imported_native_prior', native_new_event=False)
        summary = dict(mode='harvest', completed=True, stop_reason=None, simulated_s=4., physics_wall_s=3.,
                       scene=str(assembly / 'scene.usda'), native_break_events=breaks)
        for path, data in (('summary.json', summary), ('truss_sequence.json', sequence),
                           ('break_events.json', breaks), ('camera_region_decisions.json', [dict(camera_region=region)]),
                           ('runtime_settings.json', dict(hz=60, contact_limit_mm=1., fruit_contact_limit_mm=1., fruit_fruit_limit_mm=1.))):
            (folder / path).write_text(json.dumps(data))
        if trace:
            np.savez_compressed(folder / 'states.npz', time=np.arange(5.),
                phase=np.array(['initial', 'prepare_lift_branch', 'approach', 'close', 'withdraw']),
                cycle_index=np.array([0, 0, 2, 2, 2]))
        return folder, sequence, mapping

    def test_incomplete_publication_keeps_accepted_route_separate_from_all_tests(self):
        self.trial('accepted')
        self.trial('independent', plant=25)
        draft = self.root / 'final_web_drafts'; draft.mkdir()
        goal = dict(passed=False, distinct_truss_count=5, minimum_distinct_trusses=6,
            software_fault_independent_branch_present=True,
            per_run=[dict(actual_route_history=True, credited_trusses=['Plant22/Truss02'], new_native_break_count=11),
                     dict(actual_route_history=False, credited_trusses=[], new_native_break_count=4)])
        (draft / 'goal_audit.json').write_text(json.dumps(goal))
        source = self.root / 'accepted/summary.json'; before = hashlib.sha256(source.read_bytes()).hexdigest()
        report = write_report(self.root)
        self.assertFalse(report['accepted_route_goal_audit']['passed'])
        self.assertEqual(report['accepted_route_native_detachments'], 11)
        self.assertEqual(report['totals_all_experiments']['truss_counts']['new_native_detached'], 2)
        page = (self.root / 'camera_campaign_report.html').read_text()
        self.assertIn('5/6송이 · 목표 미완료', page)
        self.assertIn('독립 재시도', page)
        self.assertEqual(before, hashlib.sha256(source.read_bytes()).hexdigest())

    def test_scans_rejections_actual_phases_and_imported_breaks_stay_separate(self):
        self.trial('pilot')
        report = collect(self.root); row = report['trials'][0]
        self.assertEqual(row['new_native_break_count'], 1)
        self.assertEqual(row['manipulation_native_break_count'], 1)
        self.assertEqual(row['contact_limits_mm']['contact_limit_mm'], 1.)
        self.assertEqual(row['cycles'][0]['scan_prefix'], True)
        self.assertFalse(row['cycles'][1]['plan_accepted'])
        self.assertIsNone(row['cycles'][1]['actual_approach'])
        self.assertTrue(row['cycles'][2]['actual_approach'])
        self.assertTrue(row['cycles'][2]['actual_belt'])
        self.assertTrue(row['cycles'][2]['motion_completed'])
        counts = report['totals_all_experiments']['cycle_counts']
        self.assertEqual(counts['scan_prefixes'], 1)
        self.assertEqual(counts['planning_rejections'], 1)
        self.assertEqual(counts['accepted_regions'], 1)

    def test_same_truss_number_on_other_plants_is_distinct_and_handoff_is_not_a_new_break(self):
        self.trial('first', plant=22)
        self.trial('other_plant', plant=25)
        self.trial('resumed', plant=22, imported=True)
        report = collect(self.root); total = report['totals_all_experiments']
        self.assertEqual(total['truss_counts']['new_native_detached'], 2)
        self.assertEqual(total['unique_native_detached_fruit_count'], 2)
        self.assertEqual(total['new_native_break_count'], 2)
        self.assertFalse(total['six_distinct_trusses_with_new_native_detachment'])

    def test_finite_unexecuted_windows_and_prephysics_error_pilots_are_preserved(self):
        campaign = self.root / 'campaign'; campaign.mkdir()
        selected = [dict(plant_id=22+i, truss_index=0, path=f'/Plants/Plant_{22+i}/Truss_00', station_y_m=i * .5)
                    for i in range(6)]
        (campaign / 'row_campaign.json').write_text(json.dumps(dict(schema='s300-physical-rail-row-campaign-v1',
            selection=dict(stations=selected), stations=[], executed=False, status='planned_only',
            native_continuity='Measured state handoff; previous active window frozen; PhysX restart')))
        pilot = self.root / 'prephysics_failure'; pilot.mkdir()
        (pilot / 'runtime_error.json').write_text(json.dumps(dict(error='Preparation plan unreachable')))
        (self.root / 'prephysics_failure.log').write_text('IK finite search failed\n')
        report = write_report(self.root)
        self.assertEqual(report['campaigns'][0]['not_executed_windows'], 6)
        self.assertEqual(report['campaigns'][0]['actual_result_summary']['trial_count'], 0)
        self.assertEqual(report['trials'][0]['status'], 'runtime_error')
        page = (self.root / 'camera_campaign_report.html').read_text()
        self.assertIn('prephysics_failure.log', page)
        self.assertIn('not_executed', page)
        self.assertIn('previous active window frozen', page)

    def test_scan_break_is_recorded_but_does_not_establish_harvested_truss(self):
        folder, sequence, mapping = self.trial('scan')
        sequence['cycle_plans'] = sequence['cycle_plans'][:1]
        (folder / 'truss_sequence.json').write_text(json.dumps(sequence))
        (folder / 'break_events.json').write_text(json.dumps([dict(joint=mapping['Truss_02_Fruit_01'], t=1., phase='prepare_lift_branch')]))
        row = collect(self.root)['trials'][0]
        self.assertEqual(row['new_native_break_count'], 1)
        self.assertEqual(row['manipulation_native_break_count'], 0)
        self.assertEqual(row['truss_keys']['new_native_detached'], [])

    def test_launch_requested_is_not_a_recorded_physics_claim(self):
        campaign = self.root / 'campaign'; campaign.mkdir()
        (campaign / 'row_campaign.json').write_text(json.dumps(dict(schema='s300-physical-rail-row-campaign-v1',
            selection=dict(stations=[dict(plant_id=22, truss_index=2, path='/Plants/Plant_22/Truss_02')]),
            stations=[dict(index=0, status='native_running', physics_execution=True)], executed=True)))
        row = collect(self.root)['campaigns'][0]
        self.assertIsNone(row['finite_windows'][0]['recorded_physics_execution'])
        self.assertEqual(row['launched_but_recorded_physics_unknown_windows'], 1)
        self.trial('campaign/trials/station_000_plant_22/station_000_plant_22')
        row = collect(self.root)['campaigns'][0]
        self.assertTrue(row['finite_windows'][0]['recorded_physics_execution'])
        self.assertEqual(row['actual_result_summary']['truss_counts']['actual_approach'], 1)

    def test_missing_actual_trace_does_not_fabricate_entry_and_source_files_unchanged(self):
        folder, _, _ = self.trial('no_trace', trace=False)
        before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()}
        report = write_report(self.root)
        self.assertIsNone(report['trials'][0]['cycles'][2]['actual_approach'])
        self.assertFalse(report['trials'][0]['cycles'][2]['motion_completed'])
        self.assertEqual(before, {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()})

    def test_terminal_retreat_is_recovery_and_never_scan_or_harvest_credit(self):
        folder,sequence,_=self.trial('recovery')
        sequence['cycle_plans']=[dict(index=0,fruits=[],purpose='terminal_preparation_recovery',
            harvest_attempt=False,status='motion_completed',start_s=0.,end_s=4.)]
        (folder/'truss_sequence.json').write_text(json.dumps(sequence))
        summary=json.loads((folder/'summary.json').read_text());summary['mode']='recovery'
        (folder/'summary.json').write_text(json.dumps(summary))
        np.savez_compressed(folder/'states.npz',time=np.arange(5.),
            phase=np.array(['initial','recovery_reverse','recovery_reverse','recovery_endpoint_hold','recovery_open_hold']),
            cycle_index=np.zeros(5,dtype=int))
        report=collect(self.root);row=report['trials'][0]['cycles'][0]
        self.assertTrue(row['recovery']);self.assertFalse(row['scan_prefix'])
        self.assertFalse(row['plan_accepted']);self.assertFalse(row['motion_completed'])
        self.assertEqual(report['totals_all_experiments']['cycle_counts']['recovery_cycles'],1)
        self.assertEqual(report['totals_all_experiments']['cycle_counts']['scan_prefixes'],0)
        self.assertEqual(report['totals_all_experiments']['truss_counts']['new_native_detached'],0)

    def test_nullable_idle_sequence_remains_a_measured_zero_harvest_trial(self):
        for standalone in (False, True):
            with self.subTest(standalone=standalone):
                folder = self.root / ('idle_' + str(standalone)); folder.mkdir()
                summary = dict(mode='idle', completed=True, stop_reason=None,
                               simulated_s=3., truss_sequence=None, native_break_events=[])
                (folder / 'summary.json').write_text(json.dumps(summary))
                (folder / 'runtime_settings.json').write_text(json.dumps(dict(hz=60)))
                if standalone:
                    (folder / 'truss_sequence.json').write_text('null\n')
                np.savez_compressed(folder / 'states.npz', time=np.arange(4.),
                                    phase=np.array(['initial', 'idle', 'idle', 'idle']))
                before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in folder.iterdir() if p.is_file()}
                report = write_report(self.root)
                row = next(row for row in report['trials'] if row['path'] == folder.name)
                self.assertEqual(row['status'], 'complete')
                self.assertTrue(row['measured_trace_available'])
                self.assertEqual(row['simulated_s'], 3.)
                self.assertEqual(row['cycles'], [])
                self.assertIsNone(row['native_inventory'])
                self.assertEqual(row['new_native_break_count'], 0)
                self.assertEqual(row['truss_keys']['new_native_detached'], [])
                self.assertEqual(report['totals_all_experiments']['truss_counts']['motion_completed'], 0)
                self.assertEqual(before, {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                         for p in folder.iterdir() if p.is_file()})

    def patrol_record(self,folder,*,index=0,action='harvest',status='completed_physics'):
        summary=json.loads((folder/'summary.json').read_text())
        sequence=json.loads((folder/'truss_sequence.json').read_text())
        fruit='Truss_02_Fruit_01';joint=sequence['fruit_joints'][fruit]
        summary['selected_cycle_evidence']=dict(cycles=[dict(index=2,status='motion_completed',
            recorded_physical_cycle=True,actual_trace_measurements_available=True,
            actual_recorded_phases=['approach','close','withdraw'],selected_fruits=[dict(
                name=fruit,native_joint=joint,native_detached_at_cycle_start=False,native_detached_at_cycle_end=True,
                new_native_break_events_during_cycle=[dict(joint=joint,t=3.,phase='close',native_new_event=True)])])])
        (folder/'summary.json').write_text(json.dumps(summary))
        assembly=Path(summary['scene']).parent
        source=json.loads((assembly/'motion_manifest.json').read_text())['plant']['source_plant']
        return dict(index=index,status=status,trial=str(folder),assembly=str(assembly),
            measured_station_y_m=.15,decision=dict(action=action,target_key=source+'/Truss_02',
                current_station_y_m=0.,next_station_y_m=.15,reason='actual_visible_group'),
            benchmark_result=dict(returncode=0,timeout=False,summary=summary))

    def patrol(self,name,records,*,prior=None,goal=6,declared_goal=False):
        folder=self.root/name;folder.mkdir(exist_ok=True)
        parent=None if prior is None else dict(root=str(prior),
            report_sha256=hashlib.sha256((prior/'camera_patrol.json').read_bytes()).hexdigest())
        data=dict(schema='s300-visible-camera-patrol-campaign-v1',status='running',executed=True,
            segments=records,prior_campaign=parent,goal_distinct_trusses=goal,goal_reached=declared_goal,
            measured_harvested_trusses=[],native_continuity='Measured checkpoints, explicit frozen active windows')
        (folder/'camera_patrol.json').write_text(json.dumps(data));return folder

    def test_patrol_goal_uses_actual_native_evidence_and_ignores_goal_flag(self):
        folder,_,_=self.trial('patrol/trials/harvest/harvest')
        record=self.patrol_record(folder)
        campaign=self.patrol('patrol',[record],goal=6,declared_goal=True)
        before=hashlib.sha256((folder/'summary.json').read_bytes()).hexdigest()
        report=write_report(self.root);row=report['patrol_campaigns'][0]
        self.assertEqual(row['achieved_distinct_trusses_from_native_evidence'],1)
        self.assertFalse(row['goal_verified_from_native_evidence']);self.assertTrue(row['declared_goal_reached'])
        self.assertEqual(row['classification_counts']['accepted_actual_harvest_segments'],1)
        self.assertTrue(row['in_progress']);self.assertIn('IN PROGRESS',(self.root/'camera_campaign_report.html').read_text())
        self.assertEqual(before,hashlib.sha256((folder/'summary.json').read_bytes()).hexdigest())
        self.assertEqual(row['finite_decisions'][0]['target_key'],'/World/Greenhouse/Plants/Plant_22/Truss_02')

    def test_patrol_parent_chain_references_same_actual_trial_only_once(self):
        folder,_,_=self.trial('parent/trials/harvest/harvest');record=self.patrol_record(folder)
        parent=self.patrol('parent',[record],goal=1)
        self.patrol('child',[dict(record,index=1)],prior=parent,goal=1)
        report=collect(self.root);child=next(row for row in report['patrol_campaigns'] if row['path'].startswith('child/'))
        self.assertTrue(child['continuation_chain_valid']);self.assertTrue(child['goal_verified_from_native_evidence'])
        self.assertEqual(len(child['accepted_actual_trial_paths']),1)
        self.assertEqual(child['achieved_distinct_trusses_from_native_evidence'],1)
        self.assertTrue(child['finite_decisions'][1]['duplicate_trial_reference'])
        self.assertEqual(child['classification_counts']['accepted_actual_harvest_segments'],1)
        self.assertEqual(len(report['accepted_patrol_chains']['unique_actual_trial_paths']),1)

    def test_patrol_scan_startup_failure_and_unexecuted_window_never_count_harvest(self):
        folder,_,_=self.trial('patrol/trials/scan/scan');scan=self.patrol_record(folder,action='scan')
        scan['decision']['reason']='initial_actual_camera_observation_required'
        unused=dict(index=1,status='unused_window_startup_deferred',decision=dict(action='harvest',
            target_key='/World/Greenhouse/Plants/Plant_23/Truss_00'),
            startup_preflight=dict(passed=False,status='placement_initialization_or_idle_physics_invalid'),
            startup_probe_cache=dict(actual_new_execution=True),blocker='native warmup joint break')
        self.patrol('patrol',[scan,unused],declared_goal=True)
        row=collect(self.root)['patrol_campaigns'][0]
        self.assertEqual(row['achieved_distinct_trusses_from_native_evidence'],0)
        self.assertFalse(row['goal_verified_from_native_evidence'])
        self.assertEqual(row['classification_counts']['initial_scans'],1)
        self.assertEqual(row['classification_counts']['startup_failures'],1)
        self.assertEqual(row['classification_counts']['unused_window_startup_deferred'],1)
        self.assertFalse(row['finite_decisions'][1]['recorded_physics_execution'])
        self.assertFalse(row['finite_decisions'][1]['startup_is_camera_capture'])

    def test_patrol_partial_physics_breaks_and_missing_trace_do_not_get_completion_credit(self):
        folder,_,_=self.trial('patrol/trials/stopped/stopped');stopped=self.patrol_record(folder,status='actual_physics_stopped')
        other,_,_=self.trial('patrol/trials/no_trace/no_trace',plant=25,trace=False)
        missing=self.patrol_record(other,index=1)
        self.patrol('patrol',[stopped,missing])
        report=collect(self.root);row=report['patrol_campaigns'][0]
        self.assertEqual(row['achieved_distinct_trusses_from_native_evidence'],0)
        self.assertEqual(row['classification_counts']['actual_physics_stops'],1)
        self.assertEqual(report['totals_all_experiments']['new_native_break_count'],2)

    def test_patrol_tampered_parent_chain_cannot_prove_goal(self):
        folder,_,_=self.trial('parent/trials/harvest/harvest');record=self.patrol_record(folder)
        parent=self.patrol('parent',[record],goal=1);self.patrol('child',[],prior=parent,goal=1)
        (parent/'camera_patrol.json').write_text((parent/'camera_patrol.json').read_text()+'\n')
        report=collect(self.root);child=next(row for row in report['patrol_campaigns'] if row['path'].startswith('child/'))
        self.assertFalse(child['continuation_chain_valid']);self.assertFalse(child['goal_verified_from_native_evidence'])
        self.assertTrue(any('SHA mismatch' in warning['error'] for warning in report['warnings']))

    def test_independent_one_truss_campaigns_are_not_combined_into_six_truss_goal(self):
        for index in range(6):
            name=f'independent_{index}'
            folder,_,_=self.trial(name+'/trials/harvest/harvest',plant=22+index)
            self.patrol(name,[self.patrol_record(folder)],goal=6)
        report=collect(self.root)
        self.assertEqual(report['accepted_patrol_chains']['distinct_native_truss_count'],6)
        self.assertTrue(all(not row['goal_verified_from_native_evidence'] for row in report['patrol_campaigns']))
        self.assertFalse(report['accepted_patrol_chains']['combines_independent_experiments_for_goal'])


if __name__ == '__main__':
    unittest.main()
