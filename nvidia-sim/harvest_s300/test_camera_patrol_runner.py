import copy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import camera_patrol_runner as runner


BASE=['--headless','--camera-harvest','--capture-gt','--whole-truss',
    '--native-materialized-stage','--verify-full-environment-preflight',
    '--verify-harvest-contact-after-step','--save-route-checkpoint','--checkpoint-cycle-states',
    '--mode','harvest','--plant-dynamics','full','--camera-scan-view','row-front']


class PatrolArgumentsTests(unittest.TestCase):
    def test_lift_scan_reuses_native_scan_and_exact_checkpoint_without_rail_move(self):
        args=runner.patrol_motion_arguments(BASE,'/tmp/inventory.json',
            decision=dict(action='lift_scan',next_lift_m=.08),checkpoint='/tmp/checkpoint')
        self.assertIn('--camera-scan-only',args)
        self.assertEqual(args[args.index('--camera-scan-lift-height-m')+1],'0.08')
        self.assertIn('--resume-native-checkpoint',args)
        self.assertIn('--retain-lift-drive-reference',args)
        self.assertNotIn('--rail-to-y',args)
        with self.assertRaises(ValueError):
            runner.patrol_motion_arguments(BASE,'/tmp/inventory.json',decision=dict(action='lift_scan',next_lift_m=float('nan')))
    def test_shared_quarantine_requires_conclusive_unused_whole_plant_idle_failure(self):
        row=dict(index=2,status='unused_window_startup_deferred',accepted_parent_unchanged=True,
            decision=dict(target_key='/World/Greenhouse/Plants/Plant_22/Truss_00',
                target=dict(source_plant='/World/Greenhouse/Plants/Plant_22')),
            startup_preflight=dict(status='placement_initialization_or_idle_physics_invalid',
                can_defer_unused_placement=True,native_physics_initialization_observed=True,
                accepted_campaign_motion=False,diagnostic_state_accepted=False))
        proof=runner.shared_initial_model_blocker(row,BASE,campaign='/tmp/campaign')
        self.assertEqual(proof['source_plant'],'/World/Greenhouse/Plants/Plant_22')
        self.assertFalse(proof['evidence']['other_targets_individually_tested'])
        self.assertEqual(proof['evidence']['reachability'],'unknown')
        for key,value in [('can_defer_unused_placement',False),('native_physics_initialization_observed',False),
                ('accepted_campaign_motion',True),('status','startup_probe_timeout_or_unknown')]:
            broken=copy.deepcopy(row);broken['startup_preflight'][key]=value
            self.assertIsNone(runner.shared_initial_model_blocker(broken,BASE))
        self.assertIsNone(runner.shared_initial_model_blocker(row,BASE+['--plant-interaction-variant','leaf-ignore']))
        self.assertIsNone(runner.shared_initial_model_blocker(row,[v if v!='full' else 'off' for v in BASE]))

    def test_independent_idle_probe_may_skip_sensor_rendering_but_keeps_physics_guards(self):
        base=BASE+['--hz','60','--contact-limit-mm','1','--scene-ccd']
        original=runner.startup_preflight_arguments(base)
        probe=runner.startup_preflight_arguments(base,physics_only=True)
        self.assertIn('--capture-gt',original);self.assertNotIn('--physics-only',original)
        self.assertNotIn('--capture-gt',probe);self.assertIn('--physics-only',probe)
        for value in ('--verify-full-environment-preflight','--verify-harvest-contact-after-step','--scene-ccd'):
            self.assertIn(value,probe)
        self.assertEqual(probe[probe.index('--hz')+1],'60')
        self.assertEqual(probe[probe.index('--contact-limit-mm')+1],'1')
    def test_scan_and_physical_forward_move_reuse_existing_native_flags(self):
        original=copy.deepcopy(BASE)
        args=runner.patrol_motion_arguments(BASE,'/tmp/inventory.json',
            decision=dict(action='move',current_station_y_m=1.,next_station_y_m=1.15),checkpoint='/tmp/checkpoint')
        self.assertEqual(BASE,original)
        for flag in ('--camera-scan-only','--resume-native-checkpoint','--allow-resume-rail-travel','--rail-to-y'):
            self.assertIn(flag,args)
        self.assertIn('--retain-lift-drive-reference',args)
        self.assertNotIn('--camera-target-truss',args)
        for distance in (-.1,.201):
            with self.assertRaises(ValueError):runner.patrol_motion_arguments(BASE,'/tmp/inventory.json',
                decision=dict(action='move',current_station_y_m=1.,next_station_y_m=1.+distance))

    def test_exact_target_one_actual_cycle_and_resume_without_station_teleport(self):
        args=runner.patrol_motion_arguments(BASE+['--camera-max-harvest-cycles','3'],'/tmp/inventory.json',
            decision=dict(action='harvest',target=dict(truss_key='Truss_02')),checkpoint='/tmp/checkpoint')
        self.assertEqual(args[args.index('--camera-target-truss')+1],'Truss_02')
        self.assertEqual(args[args.index('--camera-max-harvest-cycles')+1],'1')
        self.assertNotIn('--camera-scan-only',args);self.assertNotIn('--rail-to-y',args)
        self.assertIn('--retain-lift-drive-reference',args)

    def test_lift_drive_reference_requires_proven_resume_or_route_not_fresh_start(self):
        base=BASE+['--retain-lift-drive-reference']
        fresh=runner.patrol_motion_arguments(base,'/tmp/inventory.json',decision=dict(action='scan'))
        self.assertNotIn('--retain-lift-drive-reference',fresh)
        switched=runner.patrol_motion_arguments(base,'/tmp/inventory.json',decision=dict(action='scan'),
            checkpoint='/tmp/prior-checkpoint',new_window=True)
        self.assertEqual(switched.count('--retain-lift-drive-reference'),1)
        self.assertNotIn('--resume-native-checkpoint',switched)
        resumed=runner.patrol_motion_arguments(base,'/tmp/inventory.json',decision=dict(action='scan'),
            checkpoint='/tmp/current-checkpoint')
        self.assertEqual(resumed.count('--retain-lift-drive-reference'),1)

    def test_new_window_scan_and_harvest_preserve_one_reference_without_old_resume_flag(self):
        for decision in (dict(action='lift_scan',next_lift_m=.04),
                         dict(action='harvest',target=dict(truss_key='Truss_01'))):
            for base in (BASE,BASE+['--retain-lift-drive-reference']):
                before=list(base)
                args=runner.patrol_motion_arguments(base,'/tmp/inventory.json',
                    decision=decision,checkpoint='/tmp/exact-route-parent',new_window=True)
                self.assertEqual(base,before)
                self.assertEqual(args.count('--retain-lift-drive-reference'),1)
                self.assertNotIn('--resume-native-checkpoint',args)
                self.assertNotIn('--allow-resume-rail-travel',args)
                if decision['action']=='lift_scan':
                    self.assertEqual(args[args.index('--camera-scan-lift-height-m')+1],'0.04')

    def test_capture_and_complete_guards_required(self):
        for missing in ('--capture-gt','--headless','--verify-full-environment-preflight'):
            with self.assertRaises(ValueError):runner.patrol_motion_arguments([v for v in BASE if v!=missing],
                '/tmp/inventory.json',decision=dict(action='scan'))


class CameraPatrolRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.assembly=self.root/'assembly';self.assembly.mkdir()
        scene=self.assembly/'scene.usda';scene.write_text('immutable test scene')
        self.source='/World/Greenhouse/Plants/Plant_22'
        self.manifest=dict(scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),mounted_rgbd=dict(enabled=True),
            rail=dict(initial_station_y_m=0.),plant=dict(source_plant=self.source,fruits=[
                dict(name=f'Truss_00_Fruit_{i:02d}',joint=f'/native/j{i}') for i in (1,2)]))
        self.write(self.assembly/'motion_manifest.json',self.manifest)
        self.inventory=self.root/'inventory.json'
        self.write(self.inventory,dict(trusses=[dict(plant_id=22,truss_index=0,gutter_center_x=-.775,
            root_position_world_m=[-.68,91.,.3],fruit_centroid_world_m=[-.65,99.,11.],rail_near=True)]))
        self.calls=[];self.summaries={};self.events=[];self.station=0.
        self.scene_patch=patch.object(runner,'verify_compact_dependencies',return_value=None);self.scene_patch.start();self.addCleanup(self.scene_patch.stop)
        self.complete_patch=patch.object(runner,'completed_final_checkpoint',side_effect=self.completed)
        self.complete_patch.start();self.addCleanup(self.complete_patch.stop)
        self.measured_patch=patch.object(runner,'_measured_state',side_effect=self.measured)
        self.measured_patch.start();self.addCleanup(self.measured_patch.stop)
        self.calibration_patch=patch.object(runner,'ready_camera_service_calibration',return_value=dict(
            camera_foreaft_m=0.,camera_to_grasp_height_m=0.,source='fixture robot calibration'))
        self.calibration_patch.start();self.addCleanup(self.calibration_patch.stop)

    def write(self,path,value):path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))

    def completed(self,trial,summary):
        checkpoint=Path(trial)/'route_checkpoint';checkpoint.mkdir(exist_ok=True)
        metadata=dict(break_events=copy.deepcopy(self.events),sequence=copy.deepcopy(summary.get('truss_sequence',{})),
            scene_sha256=self.manifest['scene_sha256'],source_plant=self.source,time_s=summary['simulated_s'])
        self.write(checkpoint/'checkpoint.json',metadata)
        return checkpoint,metadata

    def measured(self,assembly,manifest,checkpoint):
        path=Path(checkpoint)/'checkpoint.json'
        metadata=(json.loads(path.read_text()) if path.is_file() else dict(break_events=copy.deepcopy(self.events)))
        return metadata,dict(measured_initial_station_y_m=self.station,
            verified_final_physical_state=True),self.station,.2

    def observation_files(self,trial,summary,*,empty=False):
        frame=trial/'rgbd/frame_00000';frame.mkdir(parents=True,exist_ok=True)
        objects=[] if empty else [dict(key=f'{self.source}/Truss_00/Fruit_{i:02d}',instance_id=100+i,
            class_name='fruit',red_proxy=True,visible_pixels=40,valid_depth_pixels=40,
            visible_surface_median_world_m=[-.6,.3+(i-1.5)*.02,1.]) for i in (1,2)]
        self.write(frame/'objects.json',dict(objects=objects))
        camera=[[0,0,-1,0],[1,0,0,self.station],[0,-1,0,1.],[0,0,0,1]]
        row=dict(timestamp_s=summary['simulated_s'],world_from_camera_optical=camera,
            physics_step=300,rgb='frame_00000/rgb.png',gt=dict(objects='objects.json'),render_did_not_advance_physics=True)
        (trial/'rgbd/frames.jsonl').write_text(json.dumps(row)+'\n')

    def benchmark(self,command,check):
        self.assertTrue(check);cases=Path(command[command.index('--output')-1]);output=Path(command[command.index('--output')+1])
        case=json.loads(cases.read_text())[0];name=case['name'];args=case['arguments'];trial=output/name;trial.mkdir(parents=True)
        self.calls.append(dict(args=args,trial=trial,assembly=command[3]))
        if '--rail-to-y' in args:self.station=float(args[args.index('--rail-to-y')+1])
        scan=dict(index=0,purpose='camera_scan_prefix',harvest_attempt=False,status='motion_completed',
            fruits=[],start_s=0.,end_s=5.,planned_end_s=5.,measured_frame_index=0,end_frame_index=1)
        summary=dict(completed=True,stop_reason=None,simulated_s=5.,
            truss_sequence=dict(cycle_plans=[scan],termination='camera_scan_complete_patrol_decision_required'),
            selected_cycle_evidence=dict(cycles=[],scans=[dict(index=0,status='motion_completed',
                harvest_attempt=False,actual_recorded_phases=['settle'])]))
        if '--camera-target-truss' in args:
            self.events=[dict(joint=f'/native/j{i}',native_new_event=True,phase='close') for i in (1,2)]
            summary['truss_sequence']['cycle_plans'].append(dict(index=1,status='motion_completed',
                fruits=['Truss_00_Fruit_01','Truss_00_Fruit_02'],truss_key='Truss_00',
                start_s=0.,end_s=5.,planned_end_s=5.,measured_frame_index=0,end_frame_index=1))
            summary['truss_sequence']['termination']='camera_physical_cycle_budget_reached_remaining_fruit_not_completion'
            summary['selected_cycle_evidence']['cycles']=[dict(index=1,status='motion_completed',recorded_physical_cycle=True,
                actual_trace_measurements_available=True,actual_recorded_phases=['close','cycle_reclose'],selected_fruits=[dict(
                    name='Truss_00_Fruit_01',native_joint='/native/j1',native_detached_at_cycle_start=False,
                    native_detached_at_cycle_end=True,new_native_break_events_during_cycle=[self.events[0]])])]
        self.observation_files(trial,summary,empty=bool(self.events));self.write(trial/'summary.json',summary)
        np.savez(trial/'states.npz',times=np.array([0.,5.]),robot_q=np.zeros((2,7)))
        self.write(output/(cases.stem+'_results.json'),[dict(timeout=False,returncode=0,summary=summary)])

    def test_visible_depth_drives_short_moves_and_native_cycle_then_distinct_goal(self):
        result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'out',BASE,
            execute=True,goal_trusses=1,max_decisions=8,benchmark_runner=self.benchmark)
        self.assertEqual(result['status'],'measured_distinct_truss_goal_reached')
        self.assertTrue(result['goal_reached']);self.assertEqual(result['achieved_distinct_trusses'],1)
        self.assertEqual(len(self.calls),3)  # scan,150mm, then bounded observed arm-service proposal
        self.assertEqual(self.calls[0]['args'][-1],'--camera-scan-only')
        travel=[float(call['args'][call['args'].index('--rail-to-y')+1]) for call in self.calls if '--rail-to-y'in call['args']]
        np.testing.assert_allclose(travel,[.15]);self.assertIn('--resume-native-checkpoint',self.calls[-1]['args'])
        ledger=json.loads((self.root/'out/camera_patrol_queue.json').read_text())
        self.assertEqual(len(ledger['native_detached_keys']),2)
        self.assertEqual(len(json.loads((self.root/'out/replay_segments.json').read_text())['segments']),3)

    def test_explicit_initial_height_reuses_actual_lift_scan_before_any_harvest_decision(self):
        with patch('patrol_lift_search.lift_bounds_for_assembly',return_value=[0.,.75]):
            result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'initial_lift',BASE,
                execute=True,max_decisions=1,benchmark_runner=self.benchmark,initial_lift_height_m=.04)
        row=result['segments'][0]
        self.assertEqual(row['decision']['action'],'lift_scan')
        self.assertNotIn('target',row['decision'])
        self.assertEqual(row['decision']['next_lift_m'],.04)
        args=self.calls[0]['args'];self.assertIn('--camera-scan-only',args)
        self.assertEqual(args[args.index('--camera-scan-lift-height-m')+1],'0.04')
        self.assertNotIn('--camera-target-truss',args);self.assertNotIn('--rail-to-y',args)
        self.assertIn('camera_observation',row);self.assertIn('actual_lift_scan',row)
        self.assertEqual(row['actual_lift_scan']['requested_height_m'],.04)
        self.assertFalse(result['initial_lift_scan_requested']['physical_initial_state_changed'])
        self.assertEqual(result['achieved_distinct_trusses'],0)

    def test_initial_height_rejects_nonfinite_out_of_physical_bounds_without_native_launch(self):
        with patch('patrol_lift_search.lift_bounds_for_assembly',return_value=[0.,.75]):
            for value in (True,-.001,.751,float('nan'),float('inf')):
                with self.subTest(value=value),self.assertRaisesRegex(ValueError,'actual physical lift limits'):
                    runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'invalid_initial',BASE,
                        execute=True,benchmark_runner=self.benchmark,initial_lift_height_m=value)
        self.assertEqual(self.calls,[]);self.assertFalse((self.root/'invalid_initial').exists())

    def planning_only_benchmark(self,command,check,*,unknown=False):
        self.benchmark(command,check)
        case=Path(command[command.index('--output')-1]);out=Path(command[command.index('--output')+1])
        row=json.loads(case.read_text())[0];args=row['arguments'];trial=out/row['name']
        if '--camera-target-truss' not in args:return
        self.events=[];summary=json.loads((trial/'summary.json').read_text())
        summary['truss_sequence']['cycle_plans']=summary['truss_sequence']['cycle_plans'][:1]+[
            dict(index=i,status='planning_rejected',fruits=['Truss_00_Fruit_01','Truss_00_Fruit_02'],
                truss_key='Truss_00',error='unexpected pipeline integration failure' if unknown else
                'Fixed entry preparation IK not found: finite fixture',start_s=5.) for i in range(1,12)]
        summary['truss_sequence']['termination']='attempt_budget_exhausted_remaining_fruit'
        summary['selected_cycle_evidence']['cycles']=[dict(index=i,status='planning_rejected',
            recorded_physical_cycle=False,planned_manipulation_started=False,
            actual_trace_measurements_available=False,actual_recorded_phases=[]) for i in range(1,12)]
        self.observation_files(trial,summary,empty=False)
        self.write(trial/'summary.json',summary)
        self.write(out/(case.stem+'_results.json'),[dict(timeout=False,returncode=0,summary=summary)])

    def test_finite_approach_failure_scans_another_actual_height_before_forward_move(self):
        with patch('patrol_lift_search.lift_bounds_for_assembly',return_value=[0.,.75]):
            report=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'lift_recovery',BASE,
                execute=True,max_decisions=4,benchmark_runner=self.planning_only_benchmark,adaptive_lift=True)
        actions=[row['decision']['action']for row in report['segments']]
        self.assertEqual(actions,['scan','move','harvest','lift_scan'])
        retry=report['segments'][3]
        self.assertEqual(retry['decision']['current_station_y_m'],retry['decision']['next_station_y_m'])
        self.assertIn('--camera-scan-lift-height-m',self.calls[3]['args'])
        self.assertEqual(retry['parent_checkpoint'],report['segments'][2]['checkpoint'])
        self.assertIn('actual_lift_scan',retry)
        self.assertEqual(report['achieved_distinct_trusses'],0)

    def budget_planning_only_benchmark(self,command,check):
        self.planning_only_benchmark(command,check)
        case=Path(command[command.index('--output')-1]);out=Path(command[command.index('--output')+1])
        row=json.loads(case.read_text())[0];trial=out/row['name']
        if '--camera-target-truss' not in row['arguments']:return
        folder=trial/'cycles/001';plan=folder/'plan_before_prefix_connection.json'
        self.write(plan,dict(unit_fixture=True,selected_seed_index=8))
        report=dict(schema='s300-resumed-camera-prefix-connection-v1',status='finite_budget_or_connection_error',
            error='LimitReached: wall_time_budget',source_files_unchanged=True,physics_steps=0,
            source_plan=str(plan.resolve()),source_assembly=str(self.assembly.resolve()),
            source_file_sha256={str(plan.resolve()):hashlib.sha256(plan.read_bytes()).hexdigest(),
                str((self.assembly/'scene.usda').resolve()):self.manifest['scene_sha256']},
            state_checks=287,proposal_budget=dict(max_state_checks=5000,seconds=15.),
            verified_plan=None,eligible_for_native_execution=False)
        self.write(folder/'prefix_connection/report.json',report)
        summary=json.loads((trial/'summary.json').read_text())
        summary.update(model_sha256=self.manifest['scene_sha256'],initialization_valid=True,
            actual_self_collision_audit_passed=True)
        summary['truss_sequence']['cycle_plans'][1]['error']='Camera prefix connector failed; see '+str(folder/'prefix_connection.log')
        self.write(trial/'summary.json',summary)
        self.write(out/(case.stem+'_results.json'),[dict(timeout=False,returncode=0,summary=summary)])

    def test_known_connector_wall_budget_is_finite_unknown_not_clear_and_not_generic_error(self):
        report=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'budget',BASE,
            execute=True,max_decisions=3,benchmark_runner=self.budget_planning_only_benchmark)
        record=report['segments'][-1];trial=Path(record['trial']);summary=json.loads((trial/'summary.json').read_text())
        metadata=json.loads((Path(record['checkpoint'])/'checkpoint.json').read_text())
        outcome=runner.completed_patrol_harvest_outcome(summary,metadata,record['decision'],trial)
        witness=outcome['evidence']['rejected_proposals'][0]
        self.assertEqual(witness['kind'],'finite_existing_connector_wall_time_budget_exhausted')
        self.assertFalse(witness['path_clear']);self.assertFalse(witness['impossible'])
        self.assertIn('unknown',witness['reachability']);self.assertEqual(outcome['actual_completed_cycles'],0)
        path=trial/'cycles/001/prefix_connection/report.json';original=json.loads(path.read_text())
        for change in (dict(error='RuntimeError: wall_time_budget'),dict(error='LimitReached: unknown_budget'),
                dict(physics_steps=1),dict(source_files_unchanged=False),dict(state_checks=0),dict(verified_plan='unexpected'),
                dict(source_file_sha256={}),dict(proposal_budget=dict(max_state_checks=5000,seconds=float('nan')))):
            with self.subTest(change=change):
                self.write(path,dict(original,**change))
                with self.assertRaisesRegex(ValueError,'Unknown/inconclusive'):
                    runner.completed_patrol_harvest_outcome(summary,metadata,record['decision'],trial)
        self.write(path,original)

    def test_only_exact_old_classifier_budget_stop_can_continue_at_new_actual_endpoint(self):
        first_root=self.root/'old_budget_classifier'
        first=runner.camera_patrol_campaign(self.assembly,self.inventory,first_root,BASE,
            execute=True,max_decisions=3,benchmark_runner=self.budget_planning_only_benchmark)
        path=first_root/'camera_patrol.json';report=json.loads(path.read_text());row=report['segments'][-1]
        expected='Unknown/inconclusive rejected camera proposal is not a forward-retry permission'
        report.update(status='blocked_preserving_physical_history',stop_reason=expected)
        row.update(status='pipeline_or_physical_blocker',blocker_type='ValueError',blocker=expected)
        row.pop('planning_deferral',None);row.pop('harvest_outcome',None);self.write(path,report)
        ledger=json.loads((first_root/'camera_patrol_queue.json').read_text());item=next(iter(ledger['queue'].values()))
        item['planning_failures']=[];item['visible_now']=True;item['service_probe_offset_m']=0.
        item.pop('service_probe_station_y_m',None);item.pop('service_probe_arrived',None)
        item['service_station_y_m']=item['observed_service_station_y_m']
        q=runner.CameraPatrol.from_dict(json.loads(self.inventory.read_text()),ledger)
        q.decide(current_station_y_m=self.station,active_source_plant=self.source)
        self.write(first_root/'camera_patrol_queue.json',q.to_dict())
        metadata=json.loads((Path(first['accepted_checkpoint'])/'checkpoint.json').read_text())
        kw=dict(assembly=self.assembly,checkpoint=first['accepted_checkpoint'],metadata=metadata)
        proof=runner.completed_classifier_deferral_recovery(report,first_root,**kw)
        self.assertIsNotNone(proof);self.assertFalse(proof['checkpoint_rollback']);self.assertEqual(proof['harvest_credit'],0)
        for change in (dict(status='actual_physics_stopped_preserving_terminal_state'),
                dict(stop_reason='penetration_over_contact_policy'),dict(accepted_checkpoint=str(self.root/'older'))):
            self.assertIsNone(runner.completed_classifier_deferral_recovery(dict(report,**change),first_root,**kw))
        trial=Path(row['trial']);summary=json.loads((trial/'summary.json').read_text())
        for change in (dict(completed=False),dict(stop_reason='penetration_over_contact_policy'),
                dict(initialization_valid=False),dict(actual_self_collision_audit_passed=False)):
            changed=copy.deepcopy(report);changed['segments'][-1]['benchmark_result']['summary']=dict(summary,**change)
            self.write(trial/'summary.json',dict(summary,**change))
            self.assertIsNone(runner.completed_classifier_deferral_recovery(changed,first_root,**kw))
        self.write(trial/'summary.json',summary)
        old_report=path.read_bytes();old_queue=(first_root/'camera_patrol_queue.json').read_bytes()
        second=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'reclassified_budget',BASE,
            execute=True,max_decisions=2,initial_checkpoint=first['accepted_checkpoint'],prior_campaign=first_root,
            benchmark_runner=self.budget_planning_only_benchmark)
        self.assertEqual(second['status'],'finite_decision_budget_exhausted',second.get('stop_reason'))
        self.assertEqual([r['decision']['action'] for r in second['segments']],['scan','move'])
        self.assertAlmostEqual(second['segments'][1]['decision']['next_station_y_m'],.2)
        self.assertFalse(second['prior_campaign']['strict_completed_classifier_deferral_recovery']['checkpoint_rollback'])
        self.assertEqual(second['prior_campaign']['accepted_checkpoint'],first['accepted_checkpoint'])
        self.assertEqual(path.read_bytes(),old_report)
        self.assertEqual((first_root/'camera_patrol_queue.json').read_bytes(),old_queue)
        self.assertEqual(second['achieved_distinct_trusses'],0)

    def test_completed_scan_with_all_finite_plan_rejections_accepts_new_checkpoint_then_moves50mm(self):
        result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'planning_only',BASE,
            execute=True,max_decisions=4,benchmark_runner=self.planning_only_benchmark)
        self.assertFalse(result['goal_reached']);self.assertEqual(result['achieved_distinct_trusses'],0)
        record=result['segments'][2];following=result['segments'][3]
        self.assertEqual(record['harvest_outcome']['actual_completed_cycles'],0)
        self.assertTrue(record['harvest_outcome']['requires_forward_probe'])
        self.assertEqual(record['actual_completed_physical_cycles_for_target'],0)
        self.assertEqual(following['decision']['action'],'move')
        self.assertAlmostEqual(following['decision']['next_station_y_m']-.15,.05)
        self.assertEqual(following['parent_checkpoint'],record['checkpoint'])
        args=self.calls[3]['args'];self.assertEqual(args[args.index('--resume-native-checkpoint')+1],record['checkpoint'])
        q=json.loads((self.root/'planning_only/camera_patrol_queue.json').read_text())
        failure=next(iter(q['queue'].values()))['planning_failures'][0]
        self.assertTrue(failure['native_physics_executed']);self.assertTrue(failure['native_observation_executed'])
        self.assertFalse(failure['harvest_motion_executed']);self.assertFalse(q['native_detached_keys'])

    def test_unknown_rejection_stops_after_preserving_new_physical_endpoint(self):
        def unknown(command,check):self.planning_only_benchmark(command,check,unknown=True)
        result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'unknown',BASE,
            execute=True,max_decisions=6,benchmark_runner=unknown)
        self.assertEqual(result['status'],'blocked_preserving_physical_history');self.assertEqual(len(self.calls),3)
        self.assertEqual(result['accepted_checkpoint'],str(self.calls[-1]['trial']/'route_checkpoint'))
        self.assertIn('Unknown/inconclusive',result['stop_reason'])
        q=json.loads((self.root/'unknown/camera_patrol_queue.json').read_text())
        self.assertFalse(next(iter(q['queue'].values()))['planning_failures'])

    def test_final_planning_metadata_requires_same_actual_state_and_keeps_original_checkpoint_path(self):
        trial=self.root/'same_state_trial';trial.mkdir()
        scan=dict(index=0,purpose='camera_scan_prefix',harvest_attempt=False,status='motion_completed',
            fruits=[],start_s=0.,end_s=5.,planned_end_s=5.,measured_frame_index=0,end_frame_index=1)
        rejected=dict(index=1,status='planning_rejected',fruits=['Truss_00_Fruit_01','Truss_00_Fruit_02'],
            truss_key='Truss_00',error='Fixed entry preparation IK not found: finite fixture')
        final_sequence=dict(cycle_plans=[scan,rejected],termination='attempt_budget_exhausted_remaining_fruit')
        accepted=dict(time_s=5.,metadata_sha256='accepted-metadata',checkpoint_npz_sha256='identical-array-file',
            source_plant=self.source,controller_state=dict(position_targets=[1.]),break_events=[],
            sequence=dict(cycle_plans=[scan],checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True))
        final=dict(accepted,metadata_sha256='final-planning-metadata',sequence=final_sequence)
        summary=dict(completed=True,stop_reason=None,simulated_s=5.,truss_sequence=final_sequence,
            selected_cycle_evidence=dict(scans=[dict(status='motion_completed',harvest_attempt=False,
                actual_recorded_phases=['settle'])],cycles=[dict(index=1,status='planning_rejected',
                recorded_physical_cycle=False,planned_manipulation_started=False)]))
        self.write(trial/'truss_sequence.json',final_sequence)
        original=trial/'cycle_checkpoints/000';final_path=trial/'route_checkpoint'
        arrays={key:np.zeros((3,),dtype=np.float64) for key in ('robot_q','robot_qvel','plant_q','preload')}
        data={str(original.resolve()):(copy.deepcopy(accepted),copy.deepcopy(arrays)),
            str(final_path.resolve()):(copy.deepcopy(final),copy.deepcopy(arrays))}
        def loaded(path):return copy.deepcopy(data[str(Path(path).resolve())])
        decision=dict(action='harvest',target=dict(truss_key='Truss_00'))
        with patch.object(runner,'load_checkpoint',side_effect=loaded):
            result=runner.completed_patrol_harvest_outcome(summary,accepted,decision,trial,checkpoint_path=original)
            proof=result['evidence']['same_state_final_planning_sequence_proof']
            self.assertTrue(result['requires_forward_probe']);self.assertEqual(result['actual_completed_cycles'],0)
            self.assertEqual(proof['accepted_checkpoint'],str(original.resolve()))
            self.assertFalse(proof['accepted_physical_checkpoint_path_changed'])
            self.assertFalse(proof['final_planning_checkpoint_used_for_resume'])
            for key in arrays:
                data[str(final_path.resolve())][1][key][0]=.001
                with self.subTest(array=key),self.assertRaisesRegex(ValueError,'native array differs'):
                    runner.completed_patrol_harvest_outcome(summary,accepted,decision,trial,checkpoint_path=original)
                data[str(final_path.resolve())][1][key][0]=0.
            for key,value in [('controller_state',dict(position_targets=[2.])),('break_events',[dict(joint='changed')]),('time_s',5.01)]:
                data[str(final_path.resolve())][0][key]=value
                with self.subTest(metadata=key),self.assertRaisesRegex(ValueError,'physical/controller/provenance'):
                    runner.completed_patrol_harvest_outcome(summary,accepted,decision,trial,checkpoint_path=original)
                data[str(final_path.resolve())][0][key]=copy.deepcopy(final[key])
            with self.assertRaisesRegex(ValueError,'explicit accepted checkpoint'):
                runner.completed_patrol_harvest_outcome(summary,accepted,decision,trial)
            corrupted=copy.deepcopy(accepted);corrupted['metadata_sha256']='unbound-input'
            with self.assertRaisesRegex(ValueError,'metadata differs'):
                runner.completed_patrol_harvest_outcome(summary,corrupted,decision,trial,checkpoint_path=original)
            changed=copy.deepcopy(final_sequence);changed['termination']='unknown'
            self.write(trial/'truss_sequence.json',changed)
            with self.assertRaisesRegex(ValueError,'disagrees'):
                runner.completed_patrol_harvest_outcome(summary,accepted,decision,trial,checkpoint_path=original)

    def test_prior_planning_only_endpoint_is_not_counted_or_deferred_twice(self):
        first=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'prior_plans',BASE,
            execute=True,max_decisions=3,benchmark_runner=self.planning_only_benchmark)
        second=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'continued_plans',BASE,
            execute=True,max_decisions=2,initial_checkpoint=first['accepted_checkpoint'],prior_campaign=self.root/'prior_plans',
            benchmark_runner=self.planning_only_benchmark)
        self.assertEqual(second['achieved_distinct_trusses'],0)
        self.assertEqual(second['segments'][0]['decision']['action'],'scan')
        self.assertEqual(second['segments'][1]['decision']['action'],'move')
        self.assertAlmostEqual(second['segments'][1]['decision']['next_station_y_m'],.2)
        q=json.loads((self.root/'continued_plans/camera_patrol_queue.json').read_text())
        self.assertEqual(len(next(iter(q['queue'].values()))['planning_failures']),1)

    def test_old_runner_zero_cycle_dispatch_is_reobserved_once_then_forward_probe_not_third_identical_harvest(self):
        first_root=self.root/'old_runner_fixture'
        first=runner.camera_patrol_campaign(self.assembly,self.inventory,first_root,BASE,
            execute=True,max_decisions=3,benchmark_runner=self.planning_only_benchmark)
        # Unit migration fixture: reproduce the old controller's bookkeeping;
        # leave its completed native summary/checkpoint/physical data intact.
        ledger=json.loads((first_root/'camera_patrol_queue.json').read_text())
        item=next(iter(ledger['queue'].values()));item['planning_failures']=[];item['visible_now']=True
        item.pop('service_probe_station_y_m',None);item.pop('service_probe_arrived',None)
        item['service_probe_offset_m']=0.;item['service_station_y_m']=item['observed_service_station_y_m']
        q=runner.CameraPatrol.from_dict(json.loads(self.inventory.read_text()),ledger)
        q.decide(current_station_y_m=self.station,active_source_plant=self.source)
        self.write(first_root/'camera_patrol_queue.json',q.to_dict())
        report=json.loads((first_root/'camera_patrol.json').read_text())
        report['segments'][-1].pop('harvest_outcome',None);report['segments'][-1].pop('planning_deferral',None)
        report['segments'][-1]['actual_completed_physical_cycles_for_target']=1  # old erroneous count must be ignored
        self.write(first_root/'camera_patrol.json',report)
        old_report=(first_root/'camera_patrol.json').read_bytes();old_queue=(first_root/'camera_patrol_queue.json').read_bytes()
        second=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'migrated',BASE,
            execute=True,max_decisions=2,initial_checkpoint=first['accepted_checkpoint'],prior_campaign=first_root,
            benchmark_runner=self.planning_only_benchmark)
        self.assertEqual(second['status'],'finite_decision_budget_exhausted',second.get('stop_reason'))
        self.assertEqual(second['segments'][0]['decision']['action'],'scan')
        self.assertEqual(second['segments'][1]['decision']['action'],'move')
        self.assertAlmostEqual(second['segments'][1]['decision']['next_station_y_m'],.2)
        self.assertEqual(second['achieved_distinct_trusses'],0)
        self.assertEqual(second['prior_campaign']['pending_latest_actual_scan_planning_failure']['outcome']['actual_completed_cycles'],0)
        self.assertTrue(second['segments'][0]['prior_completed_scan_planning_deferral'])
        migrated=json.loads((self.root/'migrated/camera_patrol_queue.json').read_text())
        failures=next(iter(migrated['queue'].values()))['planning_failures']
        self.assertEqual(len(failures),1);self.assertTrue(failures[0]['native_observation_executed'])
        self.assertFalse(failures[0]['harvest_motion_executed'])
        self.assertEqual((first_root/'camera_patrol.json').read_bytes(),old_report)
        self.assertEqual((first_root/'camera_patrol_queue.json').read_bytes(),old_queue)

    def test_actual_physics_stop_is_preserved_and_never_counted_or_retried(self):
        original=self.benchmark
        def stopped(command,check):
            original(command,check)
            output=Path(command[command.index('--output')+1]);cases=Path(command[command.index('--output')-1])
            path=output/(cases.stem+'_results.json');rows=json.loads(path.read_text())
            rows[0]['summary'].update(completed=False,stop_reason='penetration_over_contact_policy')
            self.write(path,rows)
        result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'stop',BASE,
            execute=True,max_decisions=8,benchmark_runner=stopped)
        self.assertEqual(result['status'],'actual_physics_stopped_preserving_terminal_state')
        self.assertFalse(result['goal_reached']);self.assertEqual(result['measured_harvested_trusses'],[])
        self.assertEqual(len(self.calls),1);self.assertTrue(Path(result['last_actual_trial']).exists())
        replay=json.loads((self.root/'stop/replay_segments.json').read_text())['segments']
        self.assertEqual(len(replay),1);self.assertTrue(replay[0]['actual_saved_state_present'])

    def test_restart_preserves_native_credit_and_queue_only_at_exact_accepted_endpoint(self):
        first=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'first',BASE,
            execute=True,goal_trusses=1,max_decisions=8,benchmark_runner=self.benchmark)
        second=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'second',BASE,
            execute=False,initial_checkpoint=first['accepted_checkpoint'],prior_campaign=self.root/'first')
        self.assertEqual(second['achieved_distinct_trusses'],1)
        self.assertEqual(second['measured_harvested_trusses'],first['measured_harvested_trusses'])
        self.assertTrue(second['prior_campaign']['reobserves_before_next_decision'])
        self.assertFalse(second['prior_campaign']['rollback'])
        queue=json.loads((self.root/'second/camera_patrol_queue.json').read_text())['queue']
        self.assertTrue(queue);self.assertTrue(all(not item['visible_now'] for item in queue.values()))
        with self.assertRaisesRegex(ValueError,'no rollback'):
            runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'rollback',BASE,
                execute=False,initial_checkpoint=self.root/'old-checkpoint',prior_campaign=self.root/'first')

    def test_restart_never_accepts_a_physical_stop_or_unproven_harvest_count(self):
        first=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'first',BASE,
            execute=True,goal_trusses=1,max_decisions=8,benchmark_runner=self.benchmark)
        path=self.root/'first/camera_patrol.json';report=json.loads(path.read_text())
        report['status']='actual_physics_stopped_preserving_terminal_state';self.write(path,report)
        with self.assertRaisesRegex(ValueError,'no rollback'):
            runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'stop',BASE,
                execute=False,initial_checkpoint=first['accepted_checkpoint'],prior_campaign=self.root/'first')
        report['status']=first['status'];report['measured_harvested_trusses'].append(self.source+'/Truss_99');self.write(path,report)
        with self.assertRaisesRegex(ValueError,'native cycle evidence'):
            runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'forged',BASE,
                execute=False,initial_checkpoint=first['accepted_checkpoint'],prior_campaign=self.root/'first')

    def test_restart_retains_fixed_calibration_instead_of_measured_window_q(self):
        first=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'first',BASE,
            execute=True,goal_trusses=1,max_decisions=8,benchmark_runner=self.benchmark)
        with patch.object(runner,'ready_camera_service_calibration',side_effect=AssertionError('Measured start is not calibration')):
            second=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'resume_calibrated',BASE,
                execute=False,initial_checkpoint=first['accepted_checkpoint'],prior_campaign=self.root/'first')
        self.assertEqual(second['camera_service_calibration']['camera_foreaft_m'],0.)
        self.assertEqual(second['camera_service_calibration']['prior_queue_sha256'],
            runner.file_sha256(self.root/'first/camera_patrol_queue.json'))

    def test_remembered_occluded_group_gets_bounded_forward_views_not_infinite_identical_scans(self):
        original=self.benchmark
        def occluded(command,check):
            original(command,check)
            if len(self.calls)>1:
                self.write(self.calls[-1]['trial']/'rgbd/frame_00000/objects.json',dict(objects=[]))
        report=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'occluded',BASE,
            execute=True,max_decisions=13,benchmark_runner=occluded)
        self.assertFalse(report['goal_reached']);self.assertEqual(report['measured_harvested_trusses'],[])
        self.assertTrue(any(row.get('camera_reacquisition') for row in report['segments']))
        self.assertFalse(any('--camera-target-truss' in call['args'] for call in self.calls))
        queue=json.loads((self.root/'occluded/camera_patrol_queue.json').read_text())['queue']
        item=next(iter(queue.values()))
        self.assertEqual(item['status'],'finite_blocked')
        self.assertTrue(all(row['impossible'] is False for row in item['blockers']))

    def test_planning_mode_launches_no_native_work_and_preserves_existing_folder(self):
        def forbidden(*args,**kwargs):raise AssertionError('Native must not launch in planning mode')
        result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'plan',BASE,
            execute=False,benchmark_runner=forbidden)
        self.assertEqual(result['status'],'planned_only');self.assertFalse(result['goal_reached'])
        with self.assertRaises(FileExistsError):runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'plan',BASE)

    def test_nonzero_native_exit_with_partial_saved_state_stays_replayable_but_never_continues(self):
        original=self.benchmark
        def failed(command,check):
            original(command,check)
            output=Path(command[command.index('--output')+1]);cases=Path(command[command.index('--output')-1])
            path=output/(cases.stem+'_results.json');rows=json.loads(path.read_text())
            rows[0].update(returncode=1,summary=None);self.write(path,rows)
            raise subprocess.CalledProcessError(1,command)
        result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'partial',BASE,
            execute=True,max_decisions=8,benchmark_runner=failed)
        self.assertEqual(result['status'],'blocked_preserving_physical_history')
        self.assertFalse(result['goal_reached']);self.assertEqual(len(self.calls),1)
        replay=json.loads((self.root/'partial/replay_segments.json').read_text())['segments']
        self.assertEqual(len(replay),1);self.assertFalse(replay[0]['completed'])
        self.assertEqual(replay[0]['physical_stop_reason'],'native_process_incomplete')

    def test_visible_camera_loader_rejects_unsynchronized_or_missing_real_exposure(self):
        trial=self.root/'camera';summary=dict(simulated_s=3.)
        self.observation_files(trial,summary)
        loaded=runner.latest_camera_observation(trial,summary)
        self.assertEqual(loaded['timestamp_s'],3.);self.assertTrue(Path(loaded['objects_path']).is_file())
        row=json.loads((trial/'rgbd/frames.jsonl').read_text());row['render_did_not_advance_physics']=False
        (trial/'rgbd/frames.jsonl').write_text(json.dumps(row)+'\n')
        with self.assertRaisesRegex(ValueError,'synchronized'):runner.latest_camera_observation(trial,summary)


if __name__=='__main__':unittest.main()
