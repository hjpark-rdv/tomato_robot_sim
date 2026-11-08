import copy
import json
from pathlib import Path
import tempfile
import subprocess
import unittest
from unittest.mock import patch

from compact_motion_assets import file_sha256
from run_row_harvest import (DEFAULT_ARGUMENTS, campaign, completed_final_checkpoint,
                             measured_harvested_trusses,
                             unexecuted_planning_failure,
                             station_heading_choice,station_heading_alternatives,station_heading_arguments,
                             native_tool_capture_paths,capture_window_native_tool_cache,verified_window_native_tool_cache,
                             motion_arguments,validate_target_truss_window,camera_campaign_arguments,validate_camera_scan_proposal_mode, select_stations,select_preferred_stations,
                             validated_initial_checkpoint,same_plant_resume_station_proof,startup_preflight_arguments,preflight_result,
                             current_plant_completion_evidence,exclude_completed_current_plant,
                             current_plant_budget_deferral_evidence,defer_exhausted_current_plant_proposals,
                             blocked_other_trusses_evidence,defer_blocked_other_truss_proposals)


def metadata(time=10.):
    return dict(time_s=time, metadata_sha256='a'*64,scene_sha256='c'*64,
        sequence=dict(checkpoint_capture_reason='completed_cycle', checkpoint_physics_valid=True,
                      cycle_plans=[dict(status='motion_completed', end_s=time, stop_reason=None)]))


def placement(plant_id, x=-.55, centroid_x=-.5, y=1., truss=2):
    return dict(plant_id=plant_id, truss_index=truss,
        path=f'/World/Greenhouse/Plants/Plant_{plant_id:02d}/Truss_{truss:02d}',
        root_position_world_m=[x, y, .32], fruit_centroid_world_m=[centroid_x, y, 1.],
        gutter_center_x=-.775, rail_near=x>-.775)


def proposal(row, station):
    return dict(plant_id=row['plant_id'], truss_index=row['truss_index'],
        queries=[dict(index=0, station_y_m=station, status='self_path_clear_full',
            full_self_audit_complete=True, full_self_audit_passed=True,
            group=dict(fruits=['Fruit_01', 'Fruit_02']), plan='old_proposal.json')])


class RunRowHarvestTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
        self.assembly=self.root/'assembly'; self.assembly.mkdir()
        (self.assembly/'scene.usda').write_text('#usda 1.0\n# actual source fixture\n')
        self.manifest=dict(scene_sha256=file_sha256(self.assembly/'scene.usda'),
            plant=dict(source_plant='/World/Greenhouse/Plants/Plant_22'),
            rail=dict(initial_station_y_m=1.6,base_body='/World/Robot/base',source_base_position_world_m=[0.,1.26,.66]))
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        rows=[placement(22, y=1.6), placement(19, y=.65), placement(20, y=.8),
              placement(0, x=-.9), placement(24, centroid_x=-.9), placement(25)]
        self.inventory=dict(trusses=rows)
        self.inventory_path=self.root/'inventory.json'; self.inventory_path.write_text(json.dumps(self.inventory))
        self.screen=dict(source_scene_sha256='b'*64, inventory_sha256=file_sha256(self.inventory_path),
                         results=[proposal(rows[0],1.4), proposal(rows[1],.66), proposal(rows[2],.81)])
        self.screen_path=self.root/'screen.json'; self.screen_path.write_text(json.dumps(self.screen))

    def tearDown(self): self.tmp.cleanup()

    def camera_scan_preferred(self,order=(20,19)):
        preferred=self.preferred(order=order)
        self.screen['results']=[row for row in self.screen['results'] if row['plant_id'] in order]
        for row in self.screen['results']:
            query=row['queries'][0]
            query.update(status='self_path_blocked_coarse',
                preparation_variant=dict(name='jointauto',horizontal_ready_connector='joint'),
                first_block=dict(phase='prepare_joint_branch',hits=[dict(a='tool',b='arm')]))
            query.pop('full_self_audit_complete');query.pop('full_self_audit_passed')
        original=self.root/'original_finite_screen.json';original.write_text(json.dumps(self.screen))
        for row in self.screen['results']:
            query=row['queries'][0];query.update(source_report=str(original),source_report_sha256=file_sha256(original))
        self.screen_path.write_text(json.dumps(self.screen));preferred['union_report_sha256']=file_sha256(self.screen_path)
        for candidate in preferred['candidates']:
            candidate.update(proposal_kind='camera_scan_station_only',source_query_status='self_path_blocked_coarse',
                full_self_audit_complete=None,full_self_audit_passed=None,
                preparation_variant=dict(name='jointauto',horizontal_ready_connector='joint'),
                selected_preparation_arguments=['--horizontal-ready-connector','joint'],
                source_report=str(original),source_report_sha256=file_sha256(original))
        return preferred

    def test_camera_scan_station_keeps_old_blocked_truth_and_finite_budget(self):
        preferred=self.camera_scan_preferred();before=self.screen_path.read_bytes()
        with self.assertRaisesRegex(ValueError,'explicit camera'):
            select_preferred_stations(self.inventory,self.screen,preferred,self.screen_path,
                current_plant_id=22,current_station_y_m=1.6)
        result=select_preferred_stations(self.inventory,self.screen,preferred,self.screen_path,
            current_plant_id=22,current_station_y_m=1.6,max_plants=1,allow_camera_scan_only=True)
        self.assertEqual([s['plant_id'] for s in result['stations']],[20])
        self.assertEqual([s['plant_id'] for s in result['budget_unselected']],[19])
        station=result['stations'][0]
        self.assertEqual(station['source_query_status'],'self_path_blocked_coarse')
        self.assertEqual(station['source_full_self_audit'],dict(complete=None,passed=None))
        self.assertIn('Unverified camera scan station only',station['certificate'])
        self.assertEqual(before,self.screen_path.read_bytes())
        self.assertTrue(all(q['status']=='self_path_blocked_coarse' for r in self.screen['results'] for q in r['queries']))

    def test_camera_scan_station_source_status_sha_and_joint_policy_fail_closed(self):
        preferred=self.camera_scan_preferred()
        def check(p):return select_preferred_stations(self.inventory,self.screen,p,self.screen_path,
            current_plant_id=22,current_station_y_m=1.6,allow_camera_scan_only=True)
        for field,value in [('source_query_status','self_path_clear_full'),('source_plan_sha256','0'*64),
                            ('source_report_sha256','0'*64),('full_self_audit_passed',True)]:
            bad=copy.deepcopy(preferred);bad['candidates'][0][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):check(bad)
        query=next(r for r in self.screen['results'] if r['plant_id']==preferred['candidates'][0]['plant_id'])['queries'][0]
        query['status']='ik_path_not_found_finite';preferred['candidates'][0]['source_query_status']=query['status']
        with self.assertRaisesRegex(ValueError,'original finite report'):check(preferred)

    def test_scan_only_mode_requires_actual_camera_startup_and_existing_guards(self):
        args=camera_campaign_arguments(DEFAULT_ARGUMENTS,camera_capture=True)+[
            '--camera-harvest','--verify-full-environment-preflight','--horizontal-ready-connector','joint']
        validate_camera_scan_proposal_mode(args,preflight_new_windows=True,preferred_selection_path='preferred.json')
        for flag in ['--camera-harvest','--capture-gt','--verify-full-environment-preflight',
                     '--verify-harvest-contact-after-step','--checkpoint-cycle-states']:
            with self.subTest(flag=flag),self.assertRaises(ValueError):
                validate_camera_scan_proposal_mode([a for a in args if a!=flag],
                    preflight_new_windows=True,preferred_selection_path='preferred.json')
        with self.assertRaises(ValueError):validate_camera_scan_proposal_mode(args,preflight_new_windows=False,preferred_selection_path='p')
        with self.assertRaises(ValueError):validate_camera_scan_proposal_mode(args,preflight_new_windows=True,preferred_selection_path=None)
        wrong=args.copy();wrong[-1]='cartesian'
        with self.assertRaisesRegex(ValueError,'joint/ready'):validate_camera_scan_proposal_mode(wrong,preflight_new_windows=True,preferred_selection_path='p')
        with self.assertRaises(ValueError):validate_camera_scan_proposal_mode(args+['--preparation-opening','transfer'],preflight_new_windows=True,preferred_selection_path='p')
        # The independent idle probe retains the real sensors and physical guards.
        from test_run_motion_conditioning import actual_cli
        idle=actual_cli(startup_preflight_arguments(args))
        self.assertEqual(idle.mode,'idle');self.assertTrue(idle.capture_gt)
        self.assertTrue(idle.verify_harvest_contact_after_step);self.assertEqual(idle.contact_limit_mm,1.)

    def test_non_camera_campaign_cannot_opt_into_scan_only_station_proposals(self):
        with self.assertRaisesRegex(ValueError,'GT camera mode'):
            campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'badscan',
                allow_camera_scan_only_proposals=True,preflight_new_windows=True,preferred_selection_path='p')

    def test_scan_station_still_runs_independent_startup_and_blocks_before_main_on_failure(self):
        preferred=self.camera_scan_preferred(order=(20,));path=self.root/'scan_preferred.json';path.write_text(json.dumps(preferred))
        self.manifest['mounted_rgbd']={'schema':'fixture'}
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        checkpoint,row,_=self.initial_checkpoint();calls=[]
        args=camera_campaign_arguments(DEFAULT_ARGUMENTS,camera_capture=True)+[
            '--camera-harvest','--verify-full-environment-preflight','--horizontal-ready-connector','joint']
        def switch(source,prior,output,plant,target,**kwargs):
            self.assertEqual((source,prior),(self.assembly.resolve(),checkpoint.resolve()))
            output.mkdir(parents=True);manifest=copy.deepcopy(self.manifest)
            manifest['plant']['source_plant']='/World/Greenhouse/Plants/Plant_20'
            (output/'motion_manifest.json').write_text(json.dumps(manifest))
            (output/'scene.usda').write_bytes((self.assembly/'scene.usda').read_bytes());return {}
        def runner(command,check):
            case_path=Path(command[4]);case=json.loads(case_path.read_text())[0];calls.append(case)
            self.assertTrue(case['name'].startswith('startup_'))
            self.assertIn('--capture-gt',case['arguments']);self.assertIn('--verify-harvest-contact-after-step',case['arguments'])
            folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
            result=dict(returncode=2,timeout=False,summary=None)
            trial=folder/case['name'];trial.mkdir();(trial/'runtime_error.json').write_text(json.dumps(dict(error='argument parser fixture failure')))
            (folder/(case_path.stem+'_results.json')).write_text(json.dumps([result]))
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'scan_startup_blocked',
                execute=True,max_plants=1,base_arguments=args,preferred_selection_path=path,
                initial_native_checkpoint=checkpoint,preflight_new_windows=True,
                allow_camera_scan_only_proposals=True,window_switch=switch,benchmark_runner=runner)
        self.assertEqual(len(calls),1)
        self.assertEqual(report['status'],'blocked_preserving_last_physical_history')
        self.assertFalse(report['stations'][0]['physics_execution'])
        self.assertNotIn('measured_harvest_budget',report)

    def harvest_evidence(self):
        joint='/World/ElasticTarget/HarvestJoints/Truss_02_Fruit_01'
        return dict(cycles=[dict(status='motion_completed',recorded_physical_cycle=True,
            actual_trace_measurements_available=True,actual_recorded_phases=['approach','close','withdraw'],
            selected_fruits=[dict(name='Truss_02_Fruit_01',native_joint=joint,
                native_detached_at_cycle_start=False,native_detached_at_cycle_end=True,
                new_native_break_events_during_cycle=[dict(joint=joint,phase='close')])])])

    def test_harvest_budget_excludes_scans_imported_breaks_and_incomplete_motion(self):
        value=dict(completed=True,stop_reason=None,selected_cycle_evidence=self.harvest_evidence())
        self.assertEqual(measured_harvested_trusses(value,'/Plant_22'),['/Plant_22/Truss_02'])
        variants=[]
        for field,replacement in [('status','stopped'),('recorded_physical_cycle',False),
                ('actual_trace_measurements_available',False),('actual_recorded_phases',['camera_pan'])]:
            bad=copy.deepcopy(value);bad['selected_cycle_evidence']['cycles'][0][field]=replacement;variants.append(bad)
        for field,replacement in [('native_detached_at_cycle_start',True),('native_detached_at_cycle_end',False)]:
            bad=copy.deepcopy(value);bad['selected_cycle_evidence']['cycles'][0]['selected_fruits'][0][field]=replacement;variants.append(bad)
        for field,replacement in [('phase','settle'),('native_new_event',False),('type','imported_native_prior'),('joint','other')]:
            bad=copy.deepcopy(value);bad['selected_cycle_evidence']['cycles'][0]['selected_fruits'][0]['new_native_break_events_during_cycle'][0][field]=replacement;variants.append(bad)
        for bad in variants:self.assertEqual(measured_harvested_trusses(bad,'/Plant_22'),[])
        for field,replacement in [('completed',False),('stop_reason','contact_limit')]:
            bad=copy.deepcopy(value);bad[field]=replacement
            self.assertEqual(measured_harvested_trusses(bad,'/Plant_22'),[])

    def test_completed_belt_cycle_counts_selected_break_while_belts_start_open(self):
        value=dict(completed=True,stop_reason=None,selected_cycle_evidence=self.harvest_evidence())
        fruit=value['selected_cycle_evidence']['cycles'][0]['selected_fruits'][0]
        fruit['new_native_break_events_during_cycle'][0]['phase']='belt_start_open'
        self.assertEqual(measured_harvested_trusses(value,'/Plant_22'),['/Plant_22/Truss_02'])
        value['selected_cycle_evidence']['cycles'][0]['status']='stopped'
        self.assertEqual(measured_harvested_trusses(value,'/Plant_22'),[])

    def planning_failure(self,trial):
        trial.mkdir(parents=True)
        error='Traceback (most recent call last):\n  File "motion_kinematics.py", line 431, in plan_motion\nRuntimeError: Fixed entry preparation IK not found (finite probe)\n'
        (trial/'runtime_error.json').write_text(json.dumps(dict(error=error)))
        return dict(returncode=1,timeout=False,summary=None)

    def test_unexecuted_planning_failure_needs_specific_error_and_no_physics_artifacts(self):
        trial=self.root/'trial';result=self.planning_failure(trial)
        proof=unexecuted_planning_failure(result,trial,self.assembly)
        self.assertEqual(proof['reason'],'finite_ik_or_waypoint_proposal_rejected')
        self.assertFalse(proof['native_physics_started']);self.assertEqual(proof['harvest_credit'],0)
        for artifact in ['states.npz','states_partial.npz','native_initialization_timing.json',
                         'resume_native_validation.json','summary.json','route_checkpoint','rgbd']:
            path=trial/artifact;path.write_text('actual initialization/state evidence')
            self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly),artifact)
            path.unlink()
        for change in [dict(timeout=True),dict(timeout=None),dict(returncode=None),dict(summary={'completed':False})]:
            self.assertIsNone(unexecuted_planning_failure(dict(result,**change),trial,self.assembly))
        (trial/'runtime_error.json').write_text(json.dumps(dict(error='Traceback (most recent call last):\nRuntimeError: unknown error')))
        self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly))

    def test_environment_planning_failure_requires_complete_hash_bound_collision(self):
        trial=self.root/'trial';result=self.planning_failure(trial)
        (trial/'runtime_error.json').write_text(json.dumps(dict(error='Traceback (most recent call last):\nRuntimeError: Rail environment preflight blocked commanded travel')))
        plan=trial/'plan.json';plan.write_text(json.dumps(dict(phases=[{'name':'prepare'}])))
        audit=dict(status='blocked',passed=False,complete=True,native_runs=0,physics_unchanged=True,
            hard_hits=[{'obstacle':'stem'}],source_sha256=dict(scene=self.manifest['scene_sha256'],
                manifest=file_sha256(self.assembly/'motion_manifest.json'),plan=file_sha256(plan),materialized_dependency=None))
        path=trial/'full_environment_preflight.json';path.write_text(json.dumps(audit))
        self.assertEqual(unexecuted_planning_failure(result,trial,self.assembly)['reason'],'nominal_environment_proposal_blocked')
        for field,value in [('complete',False),('status','inconclusive'),('native_runs',1),('hard_hits',[])]:
            path.write_text(json.dumps(dict(audit,**{field:value})))
            self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly))
        audit['source_sha256']['plan']='wrong';path.write_text(json.dumps(audit))
        self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly))

    def early_environment_failure(self):
        trial=self.root/'early_trial';result=self.planning_failure(trial)
        (trial/'runtime_error.json').write_text(json.dumps(dict(error='Traceback (most recent call last):\nRuntimeError: Rail environment preflight blocked commanded travel')))
        plan=trial/'plan.json';plan.write_text(json.dumps(dict(phases=[{'name':'prepare_joint_branch'}])))
        first=dict(robot_geom='/World/ActuatedTool/L/part_214',
            environment_geom='/World/Greenhouse/House/Environment/GutterSuspensionWire_005/Mesh',
            obstacle_class='gutter',phase='prepare_joint_branch',time_s=6.85)
        audit=dict(schema='s300-rail-environment-preflight-v1',status='blocked',passed=False,complete=False,
            native_runs=0,physics_unchanged=True,eligible_for_native_execution=False,
            stop_on_first_hard_hit=True,early_terminated=True,
            termination_reason='first_recorded_hard_intersection',samples=415,
            hard_intersection_sample_pairs=1,hard_hits=[first],first_hard_hit=first,
            last_sample_time_s=6.85,unexamined_suffix=dict(status='UNKNOWN',from_time_s=6.85,
                from_phase='prepare_joint_branch',includes_remaining_pairs_at_first_blocked_sample=True,
                other_collisions_absent=None,nominal_path_end_s=43.4),
            source_sha256=dict(scene=self.manifest['scene_sha256'],
                manifest=file_sha256(self.assembly/'motion_manifest.json'),plan=file_sha256(plan),materialized_dependency=None))
        path=trial/'full_environment_preflight.json';path.write_text(json.dumps(audit))
        return result,trial,path,audit

    def test_early_environment_negative_is_opt_in_and_never_promoted_to_full_path_clear(self):
        result,trial,path,audit=self.early_environment_failure()
        self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly))
        proof=unexecuted_planning_failure(result,trial,self.assembly,allow_early_blocked_environment=True)
        self.assertEqual(proof['reason'],'nominal_environment_proposal_blocked_early_witness')
        early=proof['evidence']['early_blocked_environment_witness']
        self.assertFalse(early['environment_audit_complete']);self.assertFalse(early['eligible_for_native_execution'])
        self.assertTrue(early['only_current_proposal_rejected']);self.assertIsNone(early['other_collisions_absent'])
        self.assertEqual(early['unexamined_suffix']['status'],'UNKNOWN')
        self.assertFalse(proof['accepted_physical_state_changed']);self.assertFalse(proof['checkpoint_rollback'])
        self.assertFalse(proof['native_physics_started']);self.assertEqual(proof['harvest_credit'],0)
        self.assertEqual(json.loads(path.read_text()),audit)  # source negative remains immutable

    def test_early_environment_negative_requires_all_known_witness_fields(self):
        result,trial,path,audit=self.early_environment_failure()
        mutations=[('schema','other'),('status','inconclusive'),('passed',True),('native_runs',1),
            ('native_runs',False),('physics_unchanged',False),('stop_on_first_hard_hit',False),
            ('early_terminated',False),('termination_reason','query_budget_exhausted'),
            ('eligible_for_native_execution',True),('error','query failed'),('hard_hits',[]),
            ('first_hard_hit',{}),('samples',0),('hard_intersection_sample_pairs',0),
            ('last_sample_time_s',7.)]
        for key,value in mutations:
            path.write_text(json.dumps(dict(audit,**{key:value})))
            self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly,
                allow_early_blocked_environment=True),(key,value))
        for key in ('stop_on_first_hard_hit','early_terminated','termination_reason','first_hard_hit','unexamined_suffix'):
            changed=copy.deepcopy(audit);del changed[key];path.write_text(json.dumps(changed))
            self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly,
                allow_early_blocked_environment=True),key)

    def test_early_environment_negative_requires_unknown_remaining_suffix_and_bound_hashes(self):
        result,trial,path,audit=self.early_environment_failure()
        for key,value in [('status','clear'),('from_time_s',7.),('from_phase','approach'),
                ('includes_remaining_pairs_at_first_blocked_sample',False),('other_collisions_absent',True),
                ('nominal_path_end_s',6.)]:
            changed=copy.deepcopy(audit);changed['unexamined_suffix'][key]=value;path.write_text(json.dumps(changed))
            self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly,
                allow_early_blocked_environment=True),(key,value))
        changed=copy.deepcopy(audit);del changed['unexamined_suffix']['other_collisions_absent']
        path.write_text(json.dumps(changed));self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly,
            allow_early_blocked_environment=True))
        for key in ('scene','plan','manifest','materialized_dependency'):
            changed=copy.deepcopy(audit);changed['source_sha256'][key]='wrong';path.write_text(json.dumps(changed))
            self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly,
                allow_early_blocked_environment=True),key)

    def test_early_environment_opt_in_does_not_accept_physics_artifacts_timeout_or_summary(self):
        result,trial,path,audit=self.early_environment_failure()
        for key,value in [('timeout',True),('summary',{'completed':False}),('returncode',None)]:
            self.assertIsNone(unexecuted_planning_failure(dict(result,**{key:value}),trial,self.assembly,
                allow_early_blocked_environment=True))
        for artifact in ('states.npz','summary.json','route_start_native_validation.json','warmup.json'):
            p=trial/artifact;p.write_text('native evidence')
            self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly,
                allow_early_blocked_environment=True),artifact);p.unlink()
        self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly,
            allow_early_blocked_environment=1))

    def test_self_planning_failure_requires_actual_hash_bound_hit(self):
        trial=self.root/'trial';result=self.planning_failure(trial)
        (trial/'runtime_error.json').write_text(json.dumps(dict(error='Traceback (most recent call last):\nCalledProcessError: nominal self preflight rejected')))
        plan=trial/'plan.json';plan.write_text('{"phases":[]}')
        audit=dict(passed=False,complete=False,samples=12,scene_sha256=self.manifest['scene_sha256'],
            input_sha256=file_sha256(plan),physics_rerun=False,phases=[dict(hits=[dict(a='arm',b='tool')])])
        path=trial/'self_collision_preflight.json';path.write_text(json.dumps(audit))
        self.assertEqual(unexecuted_planning_failure(result,trial,self.assembly)['reason'],'nominal_self_collision_proposal_blocked')
        audit['phases'][0]['hits']=[];path.write_text(json.dumps(audit))
        self.assertIsNone(unexecuted_planning_failure(result,trial,self.assembly))

    def test_first_same_plant_planning_failure_keeps_normal_parent_for_next_window(self):
        checkpoint,row,_=self.initial_checkpoint();switches=[]
        def runner(command,check):
            cases=Path(command[4]);case=json.loads(cases.read_text())[0]
            folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
            result=self.planning_failure(folder/case['name'])
            (folder/(cases.stem+'_results.json')).write_text(json.dumps([result]))
        def switch(source,prior,output,plant,target,*,near_trusses_metadata):
            switches.append((source,prior,plant));raise RuntimeError('Sentinel confirms exact accepted parent')
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'first_same',execute=True,max_plants=2,
                initial_native_checkpoint=checkpoint,window_switch=switch,benchmark_runner=runner,
                defer_unexecuted_planning_failures=True)
        self.assertEqual(report['stations'][0]['status'],'unexecuted_planning_failure_deferred')
        self.assertEqual(switches,[(self.assembly.resolve(),checkpoint.resolve(),19)])
        self.assertFalse(report['stations'][1]['physics_execution'])

    def heading_arguments(self):
        return list(DEFAULT_ARGUMENTS)+['--horizontal-ready-connector','joint',
            '--horizontal-ready-seed-index','8']

    def heading_planning_failure(self,command):
        """A fresh exact-plan blocked self audit, with no native artifacts."""
        cases=Path(command[4]);case=json.loads(cases.read_text())[0]
        folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
        trial=folder/case['name'];result=self.planning_failure(trial)
        choice=station_heading_choice(case['arguments'])
        plan=dict(selected_seed_index=choice['seed_index'],
            truss_entry_yaw_offset_deg=choice['heading_offset_deg'],
            approach_direction_world=[1.,0.,0.],phases=[dict(name='prepare_joint_branch')])
        (trial/'plan.json').write_text(json.dumps(plan))
        (trial/'runtime_error.json').write_text(json.dumps(dict(error=
            'Traceback (most recent call last):\nRuntimeError: Native self preflight blocked commanded path')))
        manifest=json.loads((Path(command[3])/'motion_manifest.json').read_text())
        audit=dict(passed=False,complete=True,samples=12,scene_sha256=manifest['scene_sha256'],
            input_sha256=file_sha256(trial/'plan.json'),physics_rerun=False,
            phases=[dict(hits=[dict(a='arm',b='tool')])])
        (trial/'self_collision_preflight.json').write_text(json.dumps(audit))
        result_path=folder/(cases.stem+'_results.json');result_path.write_text(json.dumps([result]))
        return case,trial,result,result_path

    def test_heading_alternatives_are_fresh_finite_and_try_same_yaw_other_branch_first(self):
        for yaw in (0.,-30.):
            args=self.heading_arguments()+['--truss-entry-yaw-offset-deg',str(yaw)]
            original=list(args);alternatives=station_heading_alternatives(args)
            keys=[(row['heading_offset_deg'],row['seed_index']) for row in alternatives]
            self.assertEqual(len(keys),9);self.assertEqual(len(set(keys)),9)
            self.assertNotIn((yaw,8),keys);self.assertEqual(keys[0],(0.,5))
            self.assertEqual(set(keys),{(heading,seed) for heading in (0.,-30.,30.,-60.,60.) for seed in (8,5)}-{(yaw,8)})
            self.assertEqual(args,original)
        auto=[token for token in self.heading_arguments()[:-2]]
        alternatives=station_heading_alternatives(auto,dict(selected_seed_index=8))
        self.assertNotIn((0.,8),[(row['heading_offset_deg'],row['seed_index']) for row in alternatives])

    def test_heading_override_changes_only_post_station_yaw_and_existing_seed(self):
        station=dict(plant_id=22,station_y_m=1.6,
            preparation_variant=dict(name='jointseed8',horizontal_ready_connector='joint',horizontal_ready_seed_index=8))
        original=motion_arguments(DEFAULT_ARGUMENTS,self.inventory_path,station)
        args=station_heading_arguments(original,dict(heading_offset_deg=-30.,seed_index=5))
        self.assertEqual(station_heading_choice(args),dict(heading_offset_deg=-30.,seed_index=5))
        self.assertEqual(args.count('--horizontal-ready-seed-index'),1)
        expected=list(original);i=expected.index('--horizontal-ready-seed-index');del expected[i:i+2]
        self.assertEqual(args,expected+['--truss-entry-yaw-offset-deg','-30.0','--horizontal-ready-seed-index','5'])
        self.assertEqual(station_heading_choice(original)['seed_index'],8)

    def test_heading_retries_default_off_preserves_nominal_case_and_command_arrays(self):
        calls=[]
        def runner(command,check):
            calls.append(command);self.heading_planning_failure(command)
        for number,options in enumerate(({},dict(heading_retries=False))):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/f'headingdefault{number}',
                execute=True,max_plants=1,base_arguments=self.heading_arguments(),benchmark_runner=runner,**options)
            self.assertNotIn('heading_retries',report);self.assertNotIn('heading_attempts',report['stations'][0])
            self.assertEqual(report['status'],'blocked_preserving_last_physical_history')
        self.assertEqual(len(calls),2)
        left,right=calls
        self.assertEqual(left[:4],right[:4]);self.assertEqual(left[7:],right[7:])
        self.assertEqual(Path(left[4]).name,Path(right[4]).name)
        self.assertEqual(Path(left[6]).name,Path(right[6]).name)
        self.assertEqual(json.loads(Path(left[4]).read_text()),json.loads(Path(right[4]).read_text()))

    def test_heading_retries_stop_at_nine_and_do_not_repeat_source_cache_or_policy(self):
        calls=[];source_scene=(self.assembly/'scene.usda').read_bytes();source_manifest=(self.assembly/'motion_manifest.json').read_bytes()
        def runner(command,check):
            calls.append(command);self.heading_planning_failure(command)
            if len(calls)==1:raise subprocess.CalledProcessError(1,command)
        cache=dict(cache=str(self.root/'exact_source_native_cache.json'))
        def group(assembly,path,args):
            return args+['--harvest-group-policy',str(path)],dict(policy=str(path),source_hashes_verified=True)
        args=self.heading_arguments();i=args.index('--truss-order');args[i+1]='highest-first'
        i=args.index('--single-fruit-when-remaining');args[i+1]='0'
        with patch('run_row_harvest.capture_window_native_tool_cache',return_value=cache) as captured,\
                patch('run_row_harvest.automatic_ripe_group_arguments',side_effect=group) as policy,\
                patch('run_row_harvest.switch_window') as unused_switch:
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'headingbounded',
                execute=True,max_plants=1,base_arguments=args,benchmark_runner=runner,
                heading_retries=True,auto_native_tool_cache=True,auto_ripe_group_policy=True,preflight_new_windows=True)
        self.assertEqual(len(calls),10);self.assertEqual(captured.call_count,1);self.assertEqual(policy.call_count,1)
        self.assertEqual(unused_switch.call_count,0)
        record=report['stations'][0];attempts=record['heading_attempts']
        self.assertTrue(record['heading_retry_budget_exhausted']);self.assertEqual(len(attempts),10)
        self.assertEqual(attempts[0]['alternative_key'],[0.,8]);self.assertEqual(attempts[1]['alternative_key'],[0.,5])
        self.assertEqual(len({row['trial'] for row in attempts}),10)
        self.assertTrue(all(row['status']=='proven_unexecuted_planning_failure' for row in attempts))
        for row in attempts:
            case=json.loads(Path(row['cases']).read_text())[0]
            self.assertEqual(case['arguments'][case['arguments'].index('--native-tool-collision-cache')+1],cache['cache'])
            self.assertEqual(case['arguments'][case['arguments'].index('--harvest-group-policy')+1],record['automatic_ripe_group_policy']['policy'])
            proof=row['fresh_plan_evidence'];self.assertFalse(proof['old_screen_collision_certificate_reused'])
            self.assertEqual(proof['selected_seed_evidence']['selected_staging_seed_index'],row['alternative_key'][1])
            self.assertTrue(Path(row['trial']).is_dir())
        self.assertEqual((self.assembly/'scene.usda').read_bytes(),source_scene)
        self.assertEqual((self.assembly/'motion_manifest.json').read_bytes(),source_manifest)

    def test_heading_retry_first_completed_actual_run_uses_its_own_final_checkpoint(self):
        calls=[]
        def runner(command,check):
            calls.append(command)
            if len(calls)==1:self.heading_planning_failure(command);return
            cases=Path(command[4]);case=json.loads(cases.read_text())[0];folder=Path(command[command.index('--output')+1])
            trial=folder/case['name'];(trial/'cycle_checkpoints/000').mkdir(parents=True)
            choice=station_heading_choice(case['arguments'])
            (trial/'plan.json').write_text(json.dumps(dict(selected_seed_index=choice['seed_index'],
                truss_entry_yaw_offset_deg=choice['heading_offset_deg'],approach_direction_world=[1.,0.,0.])))
            summary=self.final_summary(trial,dict(completed=True,stop_reason=None,initialization_valid=True,simulated_s=10.))
            (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=0,timeout=False,summary=summary)]))
        with patch('run_row_harvest.load_checkpoint',return_value=(metadata(),{})):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'headingcompleted',
                execute=True,max_plants=1,base_arguments=self.heading_arguments(),heading_retries=True,benchmark_runner=runner)
        self.assertEqual(len(calls),2);record=report['stations'][0]
        self.assertEqual(record['status'],'completed_physics',record)
        self.assertIn('_heading_01_',record['checkpoint'])
        self.assertEqual(record['heading_attempts'][1]['alternative_key'],[0.,5])
        self.assertFalse(report['rollback_performed'])

    def test_heading_retry_stops_after_partial_native_state_on_first_alternative(self):
        checkpoint,row,_=self.initial_checkpoint();calls=[]
        def runner(command,check):
            calls.append(command);case,trial,result,path=self.heading_planning_failure(command)
            if len(calls)==2:(trial/'states_partial.npz').write_bytes(b'Actual latest physical state; no retry or rollback')
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'headinglaterstate',
                execute=True,max_plants=1,base_arguments=self.heading_arguments(),heading_retries=True,
                initial_native_checkpoint=checkpoint,defer_unexecuted_planning_failures=True,benchmark_runner=runner)
        self.assertEqual(len(calls),2);record=report['stations'][0]
        self.assertEqual(report['status'],'blocked_preserving_last_physical_history')
        self.assertEqual(record['status'],'route_blocked');self.assertEqual(len(record['heading_attempts']),2)
        self.assertNotIn('planning_deferral',record);self.assertTrue(record['physics_execution'])
        self.assertTrue((Path(record['heading_attempts'][-1]['trial'])/'states_partial.npz').is_file())
        self.assertFalse(report['rollback_performed'])

    def test_heading_retries_prepare_new_window_startup_and_native_capture_once(self):
        checkpoint,row,_=self.initial_checkpoint();calls=[];switches=[]
        variant=dict(name='jointseed8',horizontal_ready_connector='joint',horizontal_ready_seed_index=8)
        for source in self.screen['results']:
            source['queries'][0].update(preparation_variant=variant,selected_staging_seed_index=8)
        preferred=self.preferred(order=(20,19,22))
        for candidate in preferred['candidates']:
            candidate.update(preparation_variant=variant,
                selected_preparation_arguments=['--horizontal-ready-connector','joint','--horizontal-ready-seed-index','8'])
        preferred_path=self.root/'headingpreferred.json';preferred_path.write_text(json.dumps(preferred))
        def switch(source,prior,output,plant,target,**kwargs):
            switches.append((source,prior,plant,target));output.mkdir(parents=True)
            manifest=copy.deepcopy(self.manifest);manifest['plant']['source_plant']=f'/World/Greenhouse/Plants/Plant_{plant:02d}'
            (output/'motion_manifest.json').write_text(json.dumps(manifest))
            (output/'scene.usda').write_bytes((self.assembly/'scene.usda').read_bytes());return {}
        def runner(command,check):
            cases=Path(command[4]);case=json.loads(cases.read_text())[0];calls.append(case)
            if case['name'].startswith('startup_'):
                folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
                (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=0,timeout=False,summary={'completed':True})]))
            else:self.heading_planning_failure(command)
        args=self.heading_arguments();i=args.index('--truss-order');args[i+1]='highest-first'
        i=args.index('--single-fruit-when-remaining');args[i+1]='0'
        args+=['--capture-tool-colliders','/World/ActuatedTool/L/part_209',
            '/World/ActuatedTool/R/part_249','/World/ActuatedTool/L/part_214']
        def policy(assembly,path,arguments):return arguments+['--harvest-group-policy',str(path)],dict(policy=str(path))
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())),\
                patch('run_row_harvest.preflight_result',return_value=dict(passed=True,native_physics_initialization_observed=True)),\
                patch('run_row_harvest.verified_window_native_tool_cache',return_value=dict(cache=str(self.root/'new_window_cache.json'))) as capture,\
                patch('run_row_harvest.capture_window_native_tool_cache') as duplicate_capture,\
                patch('run_row_harvest.automatic_ripe_group_arguments',side_effect=policy) as grouped:
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'headingnewwindow',
                execute=True,max_plants=1,base_arguments=args,heading_retries=True,
                initial_native_checkpoint=checkpoint,preferred_selection_path=preferred_path,preflight_new_windows=True,
                auto_native_tool_cache=True,auto_ripe_group_policy=True,defer_unexecuted_planning_failures=True,
                window_switch=switch,benchmark_runner=runner)
        self.assertEqual(len(switches),1);self.assertEqual(len(calls),11)
        self.assertEqual(sum(case['name'].startswith('startup_') for case in calls),1)
        capture_paths=native_tool_capture_paths(args);startup=calls[0]['arguments']
        self.assertEqual(startup[startup.index('--capture-tool-colliders')+1:],capture_paths)
        self.assertEqual(report['automatic_native_tool_cache']['capture_paths'],capture_paths)
        self.assertEqual(report['stations'][0]['startup_native_tool_capture_paths'],capture_paths)
        self.assertEqual(capture.call_args.kwargs['requested_paths'],capture_paths)
        self.assertEqual(capture.call_count,1);self.assertEqual(duplicate_capture.call_count,0);self.assertEqual(grouped.call_count,1)
        record=report['stations'][0];self.assertEqual(record['status'],'unexecuted_planning_failure_deferred')
        self.assertEqual(len(record['heading_attempts']),10)
        self.assertEqual(record['planning_deferral']['accepted_parent_checkpoint'],str(checkpoint.resolve()))
        self.assertFalse(record['physics_execution']);self.assertFalse(report['rollback_performed'])

    def test_native_tool_capture_requests_merge_defaults_and_validate_exact_scope(self):
        defaults=['/World/ActuatedTool/L/part_209','/World/ActuatedTool/R/part_331']
        self.assertEqual(native_tool_capture_paths(DEFAULT_ARGUMENTS),defaults)
        original=list(DEFAULT_ARGUMENTS)+['--capture-tool-colliders',defaults[0],
            '/World/ActuatedTool/R/part_249','/World/ActuatedTool/L/part_214',defaults[1],
            '/World/ActuatedTool/R/part_249','--seconds','3']
        before=list(original)
        self.assertEqual(native_tool_capture_paths(original),defaults+[
            '/World/ActuatedTool/R/part_249','/World/ActuatedTool/L/part_214'])
        self.assertEqual(original,before)
        for invalid in (['--capture-tool-colliders'],['--capture-tool-colliders','--seconds','3'],
                ['--capture-tool-colliders',''],['--capture-tool-colliders','/World/HarvestRobot/Robot/link6'],
                ['--capture-tool-colliders','/World/ActuatedTool'],
                ['--capture-tool-colliders','/World/ActuatedTool/../Robot/collider'],
                ['--capture-tool-colliders','/World/ActuatedTool//part_1'],
                ['--capture-tool-colliders='+defaults[0]],
                ['--capture-tool-colliders',defaults[0],'--capture-tool-colliders',defaults[1]]):
            with self.subTest(invalid=invalid),self.assertRaises(ValueError):native_tool_capture_paths(invalid)

    def test_existing_idle_capture_receives_all_extra_requested_native_tool_parts(self):
        calls=[];args=self.heading_arguments()+['--capture-tool-colliders',
            '/World/ActuatedTool/L/part_209','/World/ActuatedTool/R/part_249','/World/ActuatedTool/L/part_214']
        expected=native_tool_capture_paths(args)
        def runner(command,check):
            calls.append(command);cases=Path(command[4]);case=json.loads(cases.read_text())[0]
            values=case['arguments'];self.assertEqual(values[values.index('--capture-tool-colliders')+1:],expected)
            self.assertEqual(values[values.index('--mode')+1],'idle');self.assertEqual(values[values.index('--seconds')+1],'3')
            folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
            (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=0,timeout=False,
                summary=dict(mode='idle',completed=True,stop_reason=None,initialization_valid=True,
                             simulated_s=3.,rail_destination=None,native_new_break_count=0))]))
        with patch('run_row_harvest.verified_window_native_tool_cache',return_value=dict(cache='/exact/fresh/cache.json')) as loader:
            result=capture_window_native_tool_cache(self.assembly,args,self.root/'capture_expanded','expanded',
                '/benchmark/python','/native/python',120,runner)
        self.assertEqual(len(calls),1);self.assertEqual(loader.call_args.kwargs['requested_paths'],expected)
        proof=json.loads((self.root/'capture_expanded/probe_result.json').read_text())
        self.assertEqual(proof['capture_paths'],expected);self.assertFalse(proof['diagnostic_native_state_accepted'])
        self.assertEqual(result['cache'],'/exact/fresh/cache.json')

    def test_existing_exact_native_cache_cannot_be_rebound_to_unobserved_parts(self):
        from test_native_tool_hulls import NativeToolHullTests
        fixture=NativeToolHullTests();fixture.setUp()
        try:
            original=fixture.cache.read_bytes()
            proof=verified_window_native_tool_cache(fixture.assembly,fixture.cache,requested_paths=[fixture.collider])
            self.assertEqual(proof['requested_capture_paths'],[fixture.collider])
            with self.assertRaisesRegex(ValueError,'lacks requested collider observations'):
                verified_window_native_tool_cache(fixture.assembly,fixture.cache,
                    requested_paths=[fixture.collider,'/World/ActuatedTool/R/part_249'])
            self.assertEqual(fixture.cache.read_bytes(),original)
        finally:fixture.tearDown()

    def test_heading_retries_never_follow_state_initialization_timeout_or_unknown_evidence(self):
        for artifact in ('states.npz','states_partial.npz','native_metadata.json','warmup.json',
                'route_start_native_validation.json','native_initialization_timing.json','route_checkpoint','summary.json',
                'unknown_error','timeout','actual_stop','seed_mismatch','yaw_mismatch'):
            with self.subTest(artifact=artifact):
                calls=[]
                def runner(command,check):
                    calls.append(command);case,trial,result,path=self.heading_planning_failure(command)
                    if artifact=='unknown_error':
                        (trial/'runtime_error.json').write_text(json.dumps(dict(error='Traceback (most recent call last):\nRuntimeError: unknown failure')))
                        (trial/'self_collision_preflight.json').unlink()
                    elif artifact=='timeout':result['timeout']=True
                    elif artifact=='actual_stop':result.update(returncode=0,summary=dict(completed=False,stop_reason='actual_contact_stop'))
                    elif artifact in ('seed_mismatch','yaw_mismatch'):
                        plan=json.loads((trial/'plan.json').read_text())
                        plan['selected_seed_index' if artifact=='seed_mismatch' else 'truss_entry_yaw_offset_deg']=5 if artifact=='seed_mismatch' else 30.
                        (trial/'plan.json').write_text(json.dumps(plan))
                    else:(trial/artifact).write_bytes(b'Native initialization/state evidence; never retry')
                    path.write_text(json.dumps([result]))
                report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/('headingguard_'+artifact.replace('.','_')),
                    execute=True,max_plants=1,base_arguments=self.heading_arguments(),heading_retries=True,benchmark_runner=runner)
                self.assertEqual(len(calls),1);self.assertEqual(report['status'],'blocked_preserving_last_physical_history')
                self.assertFalse(report['rollback_performed'])

    def test_heading_retries_cli_opt_in_and_camera_mode_rejection(self):
        from run_row_harvest import main
        for enabled in (False,True):
            command=['run_row_harvest.py','assembly','inventory','screen','output']
            if enabled:command.append('--heading-retries')
            with patch('sys.argv',command),patch('run_row_harvest.campaign',return_value=dict(status='planned_only')) as runner,patch('builtins.print'):
                self.assertEqual(main(),0)
            self.assertIs(runner.call_args.kwargs['heading_retries'],enabled)
        for invalid in (None,1,'true'):
            with self.assertRaisesRegex(ValueError,'boolean heading'):
                campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'headingbad',heading_retries=invalid)
        with self.assertRaisesRegex(ValueError,'GT truss-axis joint'):
            campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'headingcamera',
                heading_retries=True,base_arguments=self.heading_arguments()+['--camera-harvest'])

    def test_unused_window_planning_deferral_keeps_exact_accepted_parent(self):
        checkpoint,row,_=self.initial_checkpoint();switches=[];commands=[]
        preferred=self.preferred(order=(20,19,22));preferred_path=self.root/'preferred.json';preferred_path.write_text(json.dumps(preferred))
        def switch(source,prior,output,plant,target,*,near_trusses_metadata):
            switches.append((source,prior,plant));output.mkdir(parents=True)
            manifest=copy.deepcopy(self.manifest);manifest['plant']['source_plant']=f'/World/Greenhouse/Plants/Plant_{plant:02d}'
            (output/'motion_manifest.json').write_text(json.dumps(manifest))
            (output/'scene.usda').write_bytes((self.assembly/'scene.usda').read_bytes())
            return {}
        def runner(command,check):
            commands.append(command);cases=Path(command[4]);case=json.loads(cases.read_text())[0]
            folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
            trial=folder/case['name']
            if len(commands)==1:result=self.planning_failure(trial)
            else:
                (trial/'cycle_checkpoints/000').mkdir(parents=True)
                summary=self.final_summary(trial,dict(completed=True,stop_reason=None,initialization_valid=True,simulated_s=10.))
                result=dict(returncode=0,timeout=False,summary=summary)
            (folder/(cases.stem+'_results.json')).write_text(json.dumps([result]))
        def loader(path,**kwargs):return (row,self.initial_arrays())if Path(path)==checkpoint else (metadata(),{})
        with patch('run_row_harvest.load_checkpoint',side_effect=loader):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'deferred',execute=True,max_plants=2,
                initial_native_checkpoint=checkpoint,preferred_selection_path=preferred_path,
                window_switch=switch,benchmark_runner=runner,defer_unexecuted_planning_failures=True)
        self.assertEqual(switches,[(self.assembly.resolve(),checkpoint.resolve(),20),(self.assembly.resolve(),checkpoint.resolve(),19)])
        self.assertEqual(report['stations'][0]['status'],'unexecuted_planning_failure_deferred')
        self.assertFalse(report['stations'][0]['physics_execution'])
        self.assertTrue(Path(report['stations'][0]['assembly']).is_dir())
        self.assertTrue(Path(report['stations'][0]['planning_deferral']['evidence']['runtime_error']['path']).is_file())
        self.assertEqual(report['stations'][1]['status'],'completed_physics')
        self.assertFalse(report['rollback_performed'])

    def test_planning_deferral_default_off_and_any_actual_state_never_deferred(self):
        for enabled,actual_state in [(False,False),(True,True)]:
            with self.subTest(enabled=enabled,actual_state=actual_state):
                case_root=self.root/f'case{enabled}';calls=[]
                def runner(command,check):
                    calls.append(command);cases=Path(command[4]);case=json.loads(cases.read_text())[0]
                    folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
                    trial=folder/case['name'];result=self.planning_failure(trial)
                    if actual_state:(trial/'states_partial.npz').write_bytes(b'actual stepped state')
                    (folder/(cases.stem+'_results.json')).write_text(json.dumps([result]))
                # The accepted parent fixture is supplied explicitly; the
                # first same-plant proposal avoids unrelated window fixtures.
                checkpoint,row,_=self.initial_checkpoint() if not enabled else (self.root/'previous_trial/cycle_checkpoints/000',metadata(),None)
                if enabled:
                    row.update(source_assembly=str(self.assembly),manifest=copy.deepcopy(self.manifest),scene_sha256=self.manifest['scene_sha256'])
                with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
                    report=campaign(self.assembly,self.inventory_path,self.screen_path,case_root,execute=True,max_plants=2,
                        initial_native_checkpoint=checkpoint,benchmark_runner=runner,
                        defer_unexecuted_planning_failures=enabled)
                self.assertEqual(report['status'],'blocked_preserving_last_physical_history')
                self.assertEqual(len(calls),1);self.assertEqual(len(report['stations']),1)
                self.assertTrue(report['stations'][0]['physics_execution'])

    def test_measured_harvest_budget_stops_before_loading_another_window(self):
        calls=[]
        def runner(command,check):
            calls.append(command);cases=Path(command[4]);name=json.loads(cases.read_text())[0]['name']
            folder=Path(command[command.index('--output')+1]);(folder/name/'cycle_checkpoints/000').mkdir(parents=True)
            summary=self.final_summary(folder/name,dict(completed=True,stop_reason=None,
                initialization_valid=True,simulated_s=10.,selected_cycle_evidence=self.harvest_evidence()))
            (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=0,timeout=False,summary=summary)]))
        with patch('run_row_harvest.load_checkpoint',return_value=(metadata(),{})):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'budget',
                execute=True,max_plants=3,stop_after_harvested_trusses=1,benchmark_runner=runner,
                window_switch=lambda *a,**k:self.fail('No extra window after measured harvest budget'))
        self.assertEqual(len(calls),1);self.assertEqual(report['status'],'measured_harvest_budget_reached')
        self.assertEqual(report['measured_harvest_budget']['achieved'],1)

    def test_camera_campaign_opt_in_keeps_legacy_commands_and_checkpoint_flags(self):
        original=list(DEFAULT_ARGUMENTS)
        self.assertEqual(camera_campaign_arguments(original),original)
        rendered=camera_campaign_arguments(original,camera_capture=True)
        self.assertEqual(original,DEFAULT_ARGUMENTS)
        self.assertEqual(rendered,[value for value in original if value!='--physics-only']+['--capture-gt'])
        arguments=motion_arguments(rendered,self.inventory_path,dict(station_y_m=1.6))
        for option in ('--headless','--capture-gt','--native-materialized-stage',
                       '--save-route-checkpoint','--checkpoint-cycle-states'):
            self.assertIn(option,arguments)
        self.assertNotIn('--physics-only',arguments)
        for option in ('--hz','--contact-limit-mm','--fruit-contact-limit-mm','--plant-dynamics'):
            self.assertEqual(arguments[arguments.index(option)+1],original[original.index(option)+1])

    def test_camera_motion_arguments_require_explicit_capture_and_one_rendered_environment(self):
        without=[value for value in DEFAULT_ARGUMENTS if value!='--physics-only']
        with self.assertRaisesRegex(ValueError,'headless whole-truss'):
            motion_arguments(without,self.inventory_path,dict(station_y_m=1.6))
        for flag in ('--capture-rgbd','--capture-gt'):
            accepted=motion_arguments(without+[flag],self.inventory_path,dict(station_y_m=1.6))
            self.assertIn(flag,accepted)
            with self.assertRaisesRegex(ValueError,'rendered headless'):
                motion_arguments(DEFAULT_ARGUMENTS+[flag],self.inventory_path,dict(station_y_m=1.6))
            for count in ('2','0'):
                with self.assertRaisesRegex(ValueError,'exactly one'):
                    motion_arguments(without+[flag,'--num-envs',count],self.inventory_path,dict(station_y_m=1.6))
            with self.assertRaisesRegex(ValueError,'reset/fixture'):
                motion_arguments(without+[flag,'--replicate'],self.inventory_path,dict(station_y_m=1.6))

    def test_camera_campaign_requires_mount_and_records_rendered_mode(self):
        with self.assertRaisesRegex(ValueError,'mounted_rgbd'):
            campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'missing_camera',
                     camera_capture=True,max_plants=1)
        self.assertFalse((self.root/'missing_camera').exists())
        self.manifest['mounted_rgbd']=dict(parent='/World/Robot/link1')
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'camera_plan',
                        camera_capture=True,max_plants=1)
        self.assertEqual(report['status'],'planned_only')
        self.assertEqual(report['camera_capture'],dict(enabled=True,gt=True,
            rendered_headless=True,physics_step_control='existing native runner'))

    def test_fresh_station_policy_preserves_screened_proposal_only_with_opt_in(self):
        additional=placement(22,y=1.3,truss=1)
        inventory=copy.deepcopy(self.inventory);inventory['trusses'].append(additional)
        screen=copy.deepcopy(self.screen);screen['results'].append(proposal(additional,1.26))
        legacy=select_stations(inventory,screen,22,1.6,max_plants=1)
        explicit=select_stations(inventory,screen,22,1.6,max_plants=1,pin_current_station=True)
        self.assertEqual(legacy,explicit)
        self.assertEqual(legacy['stations'][0]['station_y_m'],1.6)
        fresh=select_stations(inventory,screen,22,1.6,max_plants=1,pin_current_station=False)
        self.assertEqual(fresh['stations'][0]['station_y_m'],1.26)
        self.assertEqual(fresh['stations'][0]['truss_index'],1)
        self.assertTrue(fresh['stations'][0]['initial_current_plant'])

    def test_camera_harvest_fresh_plan_retains_screen_station_but_resume_pins_measured(self):
        self.manifest['mounted_rgbd']=dict(parent='/World/Robot/link1')
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        arguments=DEFAULT_ARGUMENTS+['--camera-harvest']
        fresh=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'fresh_camera',
                       camera_capture=True,base_arguments=arguments,max_plants=1)
        self.assertEqual(fresh['selection']['stations'][0]['station_y_m'],1.4)
        self.assertFalse(fresh['selection']['initial_station_policy']['pin_current_station'])
        checkpoint,row,_=self.initial_checkpoint()
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
            resumed=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'resumed_camera',
                camera_capture=True,base_arguments=arguments,max_plants=1,initial_native_checkpoint=checkpoint)
        self.assertAlmostEqual(resumed['selection']['stations'][0]['station_y_m'],1.6)
        self.assertTrue(resumed['selection']['initial_station_policy']['pin_current_station'])

    def final_summary(self,trial,summary,scene_sha256='c'*64):
        trial=Path(trial);trial.mkdir(parents=True,exist_ok=True)
        summary.setdefault('actual_self_collision_audit_passed',True)
        summary.setdefault('model_sha256',scene_sha256)
        (trial/'states.npz').write_bytes(b'exact immutable native trace fixture')
        (trial/'self_collision_actual.json').write_text(json.dumps(dict(passed=True,complete=True,
            scene_sha256=summary['model_sha256'],input_sha256=file_sha256(trial/'states.npz'),physics_rerun=False)))
        (trial/'summary.json').write_text(json.dumps(summary))
        return summary

    def initial_checkpoint(self,time=10.):
        checkpoint=self.root/'previous_trial/cycle_checkpoints/000'
        checkpoint.mkdir(parents=True)
        row=metadata(time);row.update(source_assembly=str(self.assembly),manifest=copy.deepcopy(self.manifest),
                                    scene_sha256=self.manifest['scene_sha256'])
        summary=dict(completed=True,stop_reason=None,initialization_valid=True,
                     actual_self_collision_audit_passed=True,
                     simulated_s=time,model_sha256=self.manifest['scene_sha256'])
        self.final_summary(checkpoint.parent.parent,summary,self.manifest['scene_sha256'])
        return checkpoint,row,summary

    def initial_arrays(self):
        return dict(body_paths=['/World/Robot/base'],body_poses_world=[[0.,1.26,.66,0.,0.,0.,1.]])

    def test_initial_checkpoint_requires_terminal_valid_parent_not_an_earlier_good_cycle(self):
        checkpoint,row,summary=self.initial_checkpoint()
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
            _,proof=validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)
            self.assertTrue(proof['verified_final_physical_state'])
            self.assertEqual(proof['time_s'],10.)
            source_summary=checkpoint.parent.parent/'summary.json'
            summary['completed']=False;summary['stop_reason']='penetration_over_0_5mm'
            source_summary.write_text(json.dumps(summary))
            with self.assertRaisesRegex(ValueError,'parent stopped'):
                validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)
            summary.update(completed=True,stop_reason=None,simulated_s=11.)
            source_summary.write_text(json.dumps(summary))
            with self.assertRaisesRegex(ValueError,'final measured state'):
                validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)
            summary.update(simulated_s=10.,model_sha256='0'*64)
            source_summary.write_text(json.dumps(summary))
            with self.assertRaisesRegex(ValueError,'summary/model SHA'):
                validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)

    def test_initial_checkpoint_waits_for_complete_actual_state_audit(self):
        checkpoint,row,summary=self.initial_checkpoint()
        source_summary=checkpoint.parent.parent/'summary.json'
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
            for value in (None,False):
                pending=dict(summary,actual_self_collision_audit_passed=value)
                source_summary.write_text(json.dumps(pending))
                with self.assertRaisesRegex(ValueError,'parent stopped'):
                    validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)
            pending=dict(summary);pending.pop('actual_self_collision_audit_passed')
            source_summary.write_text(json.dumps(pending))
            with self.assertRaisesRegex(ValueError,'parent stopped'):
                validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)
            source_summary.write_text(json.dumps(summary))
            self.assertTrue(validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)[1]['verified_final_physical_state'])

    def test_initial_checkpoint_materialized_identity_is_required_even_with_same_wrapper_sha(self):
        dependency=self.assembly/'composed.usdc';dependency.write_bytes(b'actual materialized model')
        self.manifest['compact_assets']=dict(materialized_dependency=dict(
            relative_path='composed.usdc',sha256=file_sha256(dependency)))
        checkpoint,row,_=self.initial_checkpoint()
        row['manifest']['compact_assets']['materialized_dependency']['sha256']='0'*64
        with patch('run_row_harvest.load_checkpoint',return_value=(row,{})):
            with self.assertRaisesRegex(ValueError,'materialized dependency SHA'):
                validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)
        row['manifest']=copy.deepcopy(self.manifest);row['source_assembly']=str(self.root/'different_assembly')
        with patch('run_row_harvest.load_checkpoint',return_value=(row,{})):
            with self.assertRaisesRegex(ValueError,'exact source assembly'):
                validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)

    def test_same_plant_initial_checkpoint_is_passed_to_native_without_window_reset(self):
        checkpoint,row,_=self.initial_checkpoint();commands=[];switches=[]
        def runner(command,check):
            commands.append(command);cases=Path(command[4]);name=json.loads(cases.read_text())[0]['name']
            folder=Path(command[command.index('--output')+1]);(folder/name/'cycle_checkpoints/000').mkdir(parents=True)
            summary=self.final_summary(folder/name,dict(completed=True,stop_reason=None,initialization_valid=True,simulated_s=10.))
            (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=0,timeout=False,summary=summary)]))
        def loader(path,**kwargs):return (row,self.initial_arrays()) if Path(path)==checkpoint else (metadata(),{})
        with patch('run_row_harvest.load_checkpoint',side_effect=loader):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'resume_same',
                execute=True,max_plants=1,initial_native_checkpoint=checkpoint,
                benchmark_runner=runner,window_switch=lambda *a,**k:switches.append(a))
        self.assertEqual(report['status'],'finite_campaign_completed');self.assertEqual(switches,[])
        arguments=json.loads(Path(commands[0][4]).read_text())[0]['arguments']
        self.assertEqual(arguments[arguments.index('--resume-native-checkpoint')+1],str(checkpoint))
        self.assertEqual(arguments[:len(DEFAULT_ARGUMENTS)],DEFAULT_ARGUMENTS)
        self.assertNotIn('--rail-to-y',arguments)
        self.assertTrue(report['stations'][0]['same_plant_initial_native_resume']['verified_final_physical_state'])

    def test_checkpoint_measured_station_supersedes_pretravel_asset_origin_for_selection(self):
        checkpoint,row,_=self.initial_checkpoint();arrays=self.initial_arrays()
        arrays['body_poses_world'][0][1]+=1.2
        with patch('run_row_harvest.load_checkpoint',return_value=(row,arrays)):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'moved_origin',
                initial_native_checkpoint=checkpoint,max_plants=1)
        self.assertAlmostEqual(report['initial_native_checkpoint']['measured_initial_station_y_m'],2.8)
        self.assertAlmostEqual(report['selection']['stations'][0]['station_y_m'],2.8)
        self.assertEqual(self.manifest['rail']['initial_station_y_m'],1.6)

    def test_same_plant_retains_exact_drive_command_despite_measured_tracking_error(self):
        self.manifest['rail']['dof_name']='root_joint'
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        checkpoint,row,_=self.initial_checkpoint();arrays=self.initial_arrays()
        requested=2.1540962239837422
        retained=2.154096212387085
        measured=retained+.0000403
        arrays['body_poses_world'][0][1]+=measured-self.manifest['rail']['initial_station_y_m']
        arrays['robot_dof_names']=['root_joint','base']
        arrays['robot_q']=[measured-self.manifest['rail']['initial_station_y_m'],.25]
        row['controller_state']=dict(robot_dof_names=['root_joint','base'],
            position_source='native_get_dof_position_targets',
            position_targets=[retained-self.manifest['rail']['initial_station_y_m'],.25])
        original=copy.deepcopy(arrays)
        with patch('run_row_harvest.load_checkpoint',return_value=(row,arrays)):
            _,proof=validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)
        accepted=same_plant_resume_station_proof(requested,proof)
        self.assertGreater(accepted['measured_request_difference_m'],1e-6)
        self.assertLess(accepted['retained_command_request_difference_m'],1e-6)
        self.assertTrue(accepted['unchanged_command_proven'])
        self.assertEqual(accepted['command_identity_tolerance_m'],1e-6)
        self.assertAlmostEqual(proof['measured_initial_station_y_m'],measured)
        self.assertEqual(arrays,original)
        self.assertFalse(accepted['physical_state_modified'])
        for new_station in (requested+.001,retained+1.01e-6):
            with self.assertRaisesRegex(ValueError,'additional physical rail travel'):
                same_plant_resume_station_proof(new_station,proof)
        # Legacy measured-station continuation remains accepted without a
        # retained-target certificate; no substitute target is fabricated.
        legacy=copy.deepcopy(proof);legacy['retained_native_rail_command']=None
        self.assertFalse(same_plant_resume_station_proof(measured,legacy)['unchanged_command_proven'])
        with self.assertRaisesRegex(ValueError,'additional physical rail travel'):
            same_plant_resume_station_proof(requested,legacy)

    def test_retained_rail_command_requires_native_target_identity(self):
        self.manifest['rail']['dof_name']='root_joint'
        checkpoint,row,_=self.initial_checkpoint();arrays=self.initial_arrays()
        arrays['robot_dof_names']=['root_joint','base']
        row['controller_state']=dict(robot_dof_names=['root_joint','base'],
            position_source='measured_robot_q',position_targets=[0.,0.])
        with patch('run_row_harvest.load_checkpoint',return_value=(row,arrays)):
            with self.assertRaisesRegex(ValueError,'native rail controller provenance'):
                validated_initial_checkpoint(self.assembly,self.manifest,checkpoint)

    def test_first_new_plant_uses_initial_checkpoint_for_measured_window_handoff(self):
        checkpoint,row,_=self.initial_checkpoint();calls=[]
        preferred=self.preferred(order=(20,22,19));preferred_path=self.root/'preferred.json'
        preferred_path.write_text(json.dumps(preferred))
        def switch(source,prior,output,plant,target,*,near_trusses_metadata):
            calls.append((source,prior,plant));raise RuntimeError('Sentinel after validated handoff')
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'resume_new',
                execute=True,max_plants=1,initial_native_checkpoint=checkpoint,
                preferred_selection_path=preferred_path,window_switch=switch,
                benchmark_runner=lambda *a,**k:self.fail('Must hand off before launching native'))
        self.assertEqual(calls,[(self.assembly.resolve(),checkpoint.resolve(),20)])
        self.assertFalse(report['stations'][0]['physics_execution'])
        self.assertFalse(report['rollback_performed'])

    def test_idle_preflight_keeps_physics_and_removes_all_rail_and_harvest_commands(self):
        source=motion_arguments(DEFAULT_ARGUMENTS+['--later-grasp-depth-mm','50',
            '--split-unproductive-pairs','--truss-entry-yaw-offset-deg','-30'],self.inventory_path,dict(station_y_m=.81,
                preparation_variant=dict(name='joint6',horizontal_ready_connector='joint',
                    horizontal_ready_seed_index=6)))
        arguments=startup_preflight_arguments(source)
        for option in ('--rail-to-y','--whole-truss','--harvest-profile',
                       '--save-route-checkpoint','--near-trusses-metadata',
                       '--horizontal-ready-connector','--later-grasp-depth-mm','--split-unproductive-pairs','--truss-entry-yaw-offset-deg'):
            self.assertNotIn(option,arguments)
        self.assertEqual(arguments[-4:],['--mode','idle','--seconds','3'])
        for option in ('--hz','--threads','--solver','--position-iterations',
                       '--velocity-iterations','--fruit-contact-limit-mm','--fruit-fruit-limit-mm'):
            self.assertEqual(arguments[arguments.index(option)+1],source[source.index(option)+1])
        for option in ('--scene-ccd','--gravity-neutral-warmup','--verify-leaf-contact-after-step',
                       '--verify-harvest-contact-after-step','--verify-structural-scene-query'):
            self.assertIn(option,arguments)
        passed=dict(returncode=0,timeout=False,summary=dict(mode='idle',completed=True,
            initialization_valid=True,stop_reason=None,simulated_s=3.,rail_destination=None,
            native_new_break_count=0))
        self.assertTrue(preflight_result(passed,self.root)['passed'])
        for patchvalue in (dict(rail_destination=dict(displacement_m=.2)),
                           dict(native_new_break_count=1),dict(simulated_s=2.9),dict(initialization_valid=None)):
            wrong=copy.deepcopy(passed);wrong['summary'].update(patchvalue)
            report=preflight_result(wrong,self.root)
            self.assertFalse(report['passed']);self.assertEqual(report['diagnostic_break_harvest_credit'],0)

    def test_static_farm_support_optin_is_default_off_and_removed_from_idle_probe(self):
        from test_run_motion_conditioning import actual_cli
        flag='--released-fruit-static-farm-support-diagnostic'
        self.assertFalse(actual_cli([]).released_fruit_static_farm_support_diagnostic)
        source=motion_arguments(DEFAULT_ARGUMENTS+[flag],self.inventory_path,dict(station_y_m=.81))
        self.assertTrue(actual_cli(source).released_fruit_static_farm_support_diagnostic)
        idle=startup_preflight_arguments(source)
        self.assertNotIn(flag,idle)
        parsed=actual_cli(idle)
        self.assertFalse(parsed.released_fruit_static_farm_support_diagnostic)
        self.assertEqual(parsed.contact_limit_mm,actual_cli(source).contact_limit_mm)
        self.assertEqual(parsed.fruit_contact_limit_mm,actual_cli(source).fruit_contact_limit_mm)

    def test_camera_campaign_idle_preflight_real_cli_keeps_sensor_and_guards(self):
        from test_run_motion_conditioning import actual_cli
        base=camera_campaign_arguments(DEFAULT_ARGUMENTS,camera_capture=True)
        source=motion_arguments(base+['--camera-harvest','--camera-max-harvest-cycles','1',
            '--camera-minimum-fruits','2','--camera-min-visible-pixels','12',
            '--truss-order','highest-first','--single-fruit-when-remaining','0',
            '--entry-retract-mm','200','--grasp-lateral-offset-mm','5',
            '--last-fruit-depth-mm','20','--last-fruit-height-offset-mm','-5',
            '--last-fruit-grasp-sway-mm','1','--camera-heading-offsets-deg','0','15','-15',
            '--horizontal-ready-connector','joint','--camera-ik-seed-indices','2','8',
            '--retain-lift-drive-reference','--use-captured-tool-colliders',
            '--verify-full-environment-preflight','--capture-tool-colliders','/World/ActuatedTool/R/part_331'],
            self.inventory_path,dict(station_y_m=1.6))
        idle=startup_preflight_arguments(source)
        for name in ['--camera-harvest','--camera-max-harvest-cycles','--camera-minimum-fruits','--camera-min-visible-pixels']:
            self.assertNotIn(name,idle)
        for name in ['--entry-retract-mm','--grasp-lateral-offset-mm','--last-fruit-depth-mm',
                     '--last-fruit-height-offset-mm','--last-fruit-grasp-sway-mm','--camera-heading-offsets-deg',
                     '--camera-ik-seed-indices','--retain-lift-drive-reference',
                     '--use-captured-tool-colliders','--capture-tool-colliders']:
            self.assertNotIn(name,idle)
        self.assertIn('--capture-gt',idle);self.assertNotIn('--physics-only',idle)
        parsed=actual_cli(idle)
        self.assertEqual(parsed.mode,'idle');self.assertEqual(parsed.seconds,3.)
        self.assertTrue(parsed.capture_gt);self.assertTrue(parsed.capture_rgbd)
        self.assertFalse(parsed.camera_harvest);self.assertEqual(parsed.camera_max_harvest_cycles,0)
        self.assertTrue(parsed.verify_leaf_contact_after_step)
        self.assertTrue(parsed.verify_harvest_contact_after_step)
        self.assertTrue(parsed.verify_structural_scene_query)
        source_parsed=actual_cli(source)
        self.assertEqual(source_parsed.camera_ik_seed_indices,[2,8])
        self.assertTrue(source_parsed.retain_lift_drive_reference)
        self.assertTrue(source_parsed.use_captured_tool_colliders)
        self.assertIsNone(parsed.camera_ik_seed_indices)
        self.assertFalse(parsed.retain_lift_drive_reference)
        self.assertFalse(parsed.use_captured_tool_colliders)
        for key in ['hz','solver','device','threads','position_iterations','velocity_iterations',
                'contact_limit_mm','fruit_contact_limit_mm','fruit_fruit_limit_mm','scene_ccd','plant_dynamics']:
            self.assertEqual(getattr(parsed,key),getattr(source_parsed,key),key)

    def test_server_actual_env_safe_200mm_arguments_make_valid_idle_cli(self):
        from test_run_motion_conditioning import actual_cli
        path=Path('/root/docker_share/mujoko_debugging_data/20261022_075300_camera_region_campaign/motion_arguments_env_safe.json')
        if not path.is_file():self.skipTest('Server input not installed; portable200mm CLI case has separate coverage')
        source=motion_arguments(json.loads(path.read_text()),self.inventory_path,dict(station_y_m=1.6))
        actual_cli(source)
        idle=startup_preflight_arguments(source);args=actual_cli(idle)
        self.assertEqual(args.mode,'idle');self.assertEqual(args.seconds,3.)
        self.assertTrue(args.capture_gt);self.assertTrue(args.verify_leaf_contact_after_step)
        self.assertEqual(args.entry_retract_mm,100.)
        self.assertNotIn('--entry-retract-mm',idle)

    def test_preflight_cli_and_runtime_errors_are_not_scene_failures(self):
        trial=self.root/'bad_probe';trial.mkdir()
        result=dict(returncode=1,timeout=False,summary=None)
        log=trial.with_suffix('.log');log.write_text('usage: run_motion.py\nrun_motion.py: error: invalid mode combination\n')
        proof=preflight_result(result,trial)
        self.assertEqual(proof['status'],'pipeline_cli_integration_error')
        self.assertFalse(proof['can_defer_unused_placement']);self.assertFalse(proof['native_physics_initialization_observed'])
        self.assertIn('native_process_log',proof['evidence'])
        log.unlink();(trial/'runtime_error.json').write_text(json.dumps(dict(error='ImportError: code integration failure')))
        proof=preflight_result(result,trial)
        self.assertEqual(proof['status'],'pipeline_runtime_integration_error')
        self.assertFalse(proof['can_defer_unused_placement'])
        self.assertFalse(preflight_result(dict(result,timeout=True),trial)['can_defer_unused_placement'])

    def test_common_startup_cli_error_stops_campaign_without_skipping_plants(self):
        checkpoint,row,_=self.initial_checkpoint();switches=[];commands=[]
        preferred=self.preferred(order=(20,19,22));path=self.root/'preferred.json';path.write_text(json.dumps(preferred))
        def switch(source,prior,output,plant,target,*,near_trusses_metadata):
            switches.append((source,prior,plant));output.mkdir(parents=True)
            manifest=copy.deepcopy(self.manifest);manifest['plant']['source_plant']=f'/World/Greenhouse/Plants/Plant_{plant:02d}'
            (output/'motion_manifest.json').write_text(json.dumps(manifest));return {}
        def runner(command,check):
            commands.append(command);cases=Path(command[4]);case=json.loads(cases.read_text())[0]
            folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
            trial=folder/case['name'];trial.with_suffix('.log').write_text('run_motion.py: error: common CLI bug\n')
            (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=1,timeout=False,summary=None)]))
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'cli_failure',execute=True,max_plants=2,
                initial_native_checkpoint=checkpoint,preferred_selection_path=path,preflight_new_windows=True,
                window_switch=switch,benchmark_runner=runner)
        self.assertEqual(len(commands),1);self.assertEqual(len(switches),1)
        self.assertEqual(report['status'],'blocked_preserving_last_physical_history')
        first=report['stations'][0];self.assertFalse(first['physics_execution'])
        self.assertFalse(first['startup_preflight']['native_probe_executed'])
        self.assertEqual(first['startup_preflight']['status'],'pipeline_cli_integration_error')
        self.assertFalse(report['rollback_performed'])

    def test_failed_unused_window_preflight_keeps_same_parent_for_next_proposal(self):
        checkpoint,row,_=self.initial_checkpoint();switches=[];commands=[]
        preferred=self.preferred(order=(20,19,22));preferred_path=self.root/'preferred.json'
        preferred_path.write_text(json.dumps(preferred))
        def switch(source,prior,output,plant,target,*,near_trusses_metadata):
            switches.append((source,prior,plant));output.mkdir(parents=True)
            manifest=copy.deepcopy(self.manifest);manifest['plant']['source_plant']=f'/World/Greenhouse/Plants/Plant_{plant:02d}'
            (output/'motion_manifest.json').write_text(json.dumps(manifest))
            return dict(cart_teleported=False)
        def runner(command,check):
            commands.append(command);cases=Path(command[4]);case=json.loads(cases.read_text())[0]
            arguments=case['arguments'];folder=Path(command[command.index('--output')+1]);trial=folder/case['name']
            trial.mkdir(parents=True)
            idle=arguments[arguments.index('--mode')+1]=='idle'
            if idle:
                self.assertNotIn('--rail-to-y',arguments)
                failure=case['name'].endswith('plant_20')
                summary=None if failure else dict(mode='idle',completed=True,stop_reason=None,
                    initialization_valid=True,simulated_s=3.,rail_destination=None,native_new_break_count=0)
                if failure:(trial/'initialization_failure.json').write_text(json.dumps(dict(native_new_break_count=4)))
            else:
                (trial/'cycle_checkpoints/000').mkdir(parents=True)
                summary=self.final_summary(trial,dict(completed=True,stop_reason=None,initialization_valid=True,simulated_s=10.))
                self.assertIn('--rail-to-y',arguments)
            result=[dict(returncode=1 if summary is None else 0,timeout=False,summary=summary)]
            (folder/(cases.stem+'_results.json')).write_text(json.dumps(result))
        def loader(path,**kwargs):return (row,self.initial_arrays()) if Path(path)==checkpoint else (metadata(),{})
        with patch('run_row_harvest.load_checkpoint',side_effect=loader):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'preflight_route',
                execute=True,max_plants=2,initial_native_checkpoint=checkpoint,
                preferred_selection_path=preferred_path,preflight_new_windows=True,
                window_switch=switch,benchmark_runner=runner)
        self.assertEqual(switches,[(self.assembly.resolve(),checkpoint.resolve(),20),
                                   (self.assembly.resolve(),checkpoint.resolve(),19)])
        self.assertEqual(len(commands),3)
        first,second=report['stations']
        self.assertEqual(first['status'],'unexecuted_window_startup_excluded')
        self.assertFalse(first['physics_execution']);self.assertFalse(first['accepted_physical_history_changed'])
        self.assertIn('initialization_failure.json',first['startup_preflight']['evidence'])
        self.assertEqual(second['status'],'completed_physics')
        self.assertFalse(report['rollback_performed'])

    def test_actual_motion_failure_after_passed_preflight_stops_without_trying_next_window(self):
        checkpoint,row,_=self.initial_checkpoint();switches=[]
        preferred=self.preferred(order=(20,19,22));preferred_path=self.root/'preferred.json'
        preferred_path.write_text(json.dumps(preferred))
        def switch(source,prior,output,plant,target,*,near_trusses_metadata):
            switches.append(plant);output.mkdir(parents=True)
            manifest=copy.deepcopy(self.manifest);manifest['plant']['source_plant']=f'/World/Greenhouse/Plants/Plant_{plant:02d}'
            (output/'motion_manifest.json').write_text(json.dumps(manifest));return {}
        def runner(command,check):
            cases=Path(command[4]);case=json.loads(cases.read_text())[0];args=case['arguments']
            folder=Path(command[command.index('--output')+1]);folder.mkdir(parents=True)
            idle=args[args.index('--mode')+1]=='idle'
            summary=(dict(mode='idle',completed=True,stop_reason=None,initialization_valid=True,
                          simulated_s=3.,rail_destination=None,native_new_break_count=0) if idle else
                     dict(completed=False,stop_reason='penetration_over_0_5mm',initialization_valid=True,simulated_s=8.))
            (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=0,timeout=False,summary=summary)]))
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'actual_fail',
                execute=True,max_plants=2,initial_native_checkpoint=checkpoint,
                preferred_selection_path=preferred_path,preflight_new_windows=True,
                window_switch=switch,benchmark_runner=runner)
        self.assertEqual(switches,[20]);self.assertEqual(report['status'],'blocked_preserving_last_physical_history')
        self.assertEqual(len(report['stations']),1);self.assertTrue(report['stations'][0]['physics_execution'])
        self.assertFalse(report['rollback_performed'])

    def preferred(self,order=(22,20,19)):
        rows=[]
        for plant in order:
            result=next(row for row in self.screen['results'] if row['plant_id']==plant)
            query=result['queries'][0];plan=self.root/f'plan_{plant}.json'
            plan.write_text(json.dumps(dict(plant=plant,station=query['station_y_m'])))
            query['plan']=str(plan)
            rows.append(dict(plant_id=plant,truss_index=result['truss_index'],query_index=query['index'],
                station_y_m=query['station_y_m'],source_plan=str(plan),source_plan_sha256=file_sha256(plan),
                preparation_variant=dict(name='baseline_cartesian',horizontal_ready_connector='cartesian'),
                selected_preparation_arguments=['--horizontal-ready-connector','cartesian'],
                full_self_audit_complete=True,full_self_audit_passed=True))
        self.screen_path.write_text(json.dumps(self.screen))
        return dict(schema='s300-next-row-fullself-proposal-selection-v1',union_report=str(self.screen_path),
            union_report_sha256=file_sha256(self.screen_path),current_plant_id=22,current_station_y_m=1.6,
            candidates=rows)

    def test_explicit_order_keeps_current_station_move_as_real_rail_command(self):
        preferred=self.preferred()
        selected=select_preferred_stations(self.inventory,self.screen,preferred,self.screen_path,
            current_plant_id=22,current_station_y_m=1.6000002,max_plants=2)
        self.assertEqual([row['plant_id'] for row in selected['stations']],[22,20])
        # The source proposal remains1.4; initial measured1.6000002 is not
        # overwritten or teleported to that proposed physical station.
        self.assertEqual(selected['stations'][0]['station_y_m'],1.4)
        self.assertEqual(selected['actual_initial_station_y_m'],1.6000002)
        args=motion_arguments(DEFAULT_ARGUMENTS,self.inventory_path,selected['stations'][0])
        self.assertEqual(args[args.index('--rail-to-y')+1],'1.4')
        self.assertEqual(selected['budget_unselected'][0]['plant_id'],19)

    def test_preferred_tampered_source_query_and_physical_arguments_fail_closed(self):
        preferred=self.preferred()
        def check(value):return select_preferred_stations(self.inventory,self.screen,value,self.screen_path,
            current_plant_id=22,current_station_y_m=1.6)
        wrong=copy.deepcopy(preferred);wrong['union_report_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'SHA'):check(wrong)
        wrong=copy.deepcopy(preferred);wrong['candidates'][0]['station_y_m']=1.5
        with self.assertRaisesRegex(ValueError,'station'):check(wrong)
        wrong=copy.deepcopy(preferred);wrong['candidates'][0]['selected_preparation_arguments']+=['--fruit-contact-limit-mm','5']
        with self.assertRaisesRegex(ValueError,'arguments'):check(wrong)
        wrong=copy.deepcopy(preferred);wrong['candidates'][0]['source_plan_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'plan/path/SHA'):check(wrong)
        wrong=copy.deepcopy(preferred);wrong['candidates'].pop()
        with self.assertRaisesRegex(ValueError,'silently omitted'):check(wrong)

    def test_previous_frozen_window_is_reported_and_never_rebuilt_from_original_source(self):
        preferred=self.preferred()
        selected=select_preferred_stations(self.inventory,self.screen,preferred,self.screen_path,
            current_plant_id=20,current_station_y_m=1.6,previous_window_plants=[22])
        self.assertEqual([row['plant_id'] for row in selected['stations']],[20,19])
        self.assertEqual(selected['previous_window_unselected'][0]['plant_id'],22)
        self.assertIn('not implemented',selected['previous_window_unselected'][0]['reachability'])
        self.assertEqual(selected['stations'][0]['station_y_m'],.81)

    def test_side_policy_and_finite_budget_never_claim_impossible(self):
        selected=select_stations(self.inventory,self.screen,22,1.6,max_plants=2)
        self.assertEqual([r['plant_id'] for r in selected['stations']],[22,19])
        self.assertEqual(selected['stations'][0]['station_y_m'],1.6)
        self.assertEqual(selected['stations'][1]['target_name'],'Truss_02_Fruit_01')
        self.assertEqual({r['plant_id'] for r in selected['side_excluded']},{0,24})
        self.assertEqual(selected['finite_screen_unknown'][0]['plant_id'],25)
        self.assertIn('not impossible',selected['finite_screen_unknown'][0]['reachability'])
        self.assertEqual(selected['budget_unselected'][0]['plant_id'],20)

    def test_screened_joint_branch_survives_route_selection_and_command_construction(self):
        variant=dict(name='joint6',horizontal_ready_connector='joint',horizontal_ready_seed_index=6)
        query=self.screen['results'][1]['queries'][0]
        query.update(preparation_variant=variant,selected_staging_seed_index=6)
        selected=select_stations(self.inventory,self.screen,22,1.6,max_plants=2)
        station=selected['stations'][1]
        self.assertEqual(station['preparation_variant'],variant)
        base=DEFAULT_ARGUMENTS+['--horizontal-ready-connector','cartesian']
        command=motion_arguments(base,self.inventory_path,station)
        self.assertEqual(command.count('--horizontal-ready-connector'),1)
        self.assertEqual(command[command.index('--horizontal-ready-connector')+1],'joint')
        self.assertEqual(command[command.index('--horizontal-ready-seed-index')+1],'6')
        # The argument override cannot change native fidelity/contact policy.
        for flag in ('--fruit-contact-limit-mm','--plant-dynamics','--hz'):
            self.assertEqual(command[command.index(flag)+1],DEFAULT_ARGUMENTS[DEFAULT_ARGUMENTS.index(flag)+1])
        query['selected_staging_seed_index']=0
        with self.assertRaisesRegex(ValueError,'staging branch'):
            select_stations(self.inventory,self.screen,22,1.6,max_plants=2)

    def test_legacy_route_command_is_unchanged_without_selected_variant(self):
        station=dict(station_y_m=1.6)
        expected=DEFAULT_ARGUMENTS+['--near-trusses-metadata',str(self.inventory_path.resolve()),
                                   '--initial-truss-first','--rail-to-y','1.6']
        self.assertEqual(motion_arguments(DEFAULT_ARGUMENTS,self.inventory_path,station),expected)
        with self.assertRaisesRegex(ValueError,'unsupported'):
            motion_arguments(DEFAULT_ARGUMENTS,self.inventory_path,
                dict(station,preparation_variant=dict(name='unsafe',horizontal_ready_connector='joint',
                    collision_filter='disabled')))

    def target_window_manifest(self,plant_id=22,truss=2):
        manifest=copy.deepcopy(self.manifest);root='/World/ElasticTarget'
        name=f'Truss_{truss:02d}_Fruit_01'
        target=dict(name=name,path=root+'/'+name,joint=root+'/HarvestJoints/'+name)
        manifest['plant'].update(source_plant=f'/World/Greenhouse/Plants/Plant_{plant_id:02d}',
            root=root,fruits=[dict(name=f'Truss_{t:02d}_Fruit_{n:02d}',
                path=root+f'/Truss_{t:02d}_Fruit_{n:02d}',
                joint=root+f'/HarvestJoints/Truss_{t:02d}_Fruit_{n:02d}')
                for t in range(3) for n in range(1,11)])
        manifest.update(target=target,plant_interaction_variant=dict(
            mode='target_truss_and_own_stem_only',target_root=root,target_truss=f'Truss_{truss:02d}'))
        return manifest

    def test_target_window_arguments_omit_only_near_scope_options_and_preserve_strict_defaults(self):
        station=dict(station_y_m=.66,preparation_variant=dict(name='joint8',
            horizontal_ready_connector='joint',horizontal_ready_seed_index=8))
        original=motion_arguments(DEFAULT_ARGUMENTS,self.inventory_path,station)
        expected=original.copy();i=expected.index('--near-trusses-metadata');del expected[i:i+2]
        expected.remove('--initial-truss-first')
        self.assertEqual(motion_arguments(DEFAULT_ARGUMENTS,self.inventory_path,station,
            target_truss_windows=True),expected)
        from test_run_motion_conditioning import actual_cli
        parsed=actual_cli(expected)
        self.assertIsNone(parsed.near_trusses_metadata);self.assertFalse(parsed.initial_truss_first)
        self.assertEqual(motion_arguments(DEFAULT_ARGUMENTS,self.inventory_path,station,
            target_truss_windows=False),original)
        for value in (None,1,'true'):
            with self.assertRaisesRegex(ValueError,'boolean'):
                motion_arguments(DEFAULT_ARGUMENTS,self.inventory_path,station,target_truss_windows=value)

    def test_target_window_validation_rejects_wrong_mode_source_and_fruit_mapping(self):
        manifest=self.target_window_manifest();station=dict(plant_id=22,truss_index=2,target_name='Truss_02_Fruit_01')
        before=copy.deepcopy(manifest)
        proof=validate_target_truss_window(manifest,station)
        self.assertEqual(proof['source_plant_fruit_count'],30)
        self.assertFalse(proof['entire_plant_completion_claim']);self.assertEqual(manifest,before)
        changes=[('plant_interaction_variant','mode','active_plant_and_own_stem_only'),
                 ('plant_interaction_variant','target_truss','Truss_01'),
                 ('plant_interaction_variant','target_root','/World/OtherPlant'),
                 ('plant','source_plant','/World/Other/Plant_22'),
                 ('target','path','/World/OtherPlant/Truss_02_Fruit_01'),
                 ('target','joint','/World/ElasticTarget/HarvestJoints/Truss_01_Fruit_01')]
        for section,field,value in changes:
            bad=copy.deepcopy(manifest);bad[section][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):validate_target_truss_window(bad,station)
        for field,value in [('plant_id',19),('truss_index',1),('target_name','Truss_01_Fruit_01')]:
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'station'):
                validate_target_truss_window(manifest,dict(station,**{field:value}))
        bad=copy.deepcopy(manifest);bad['plant']['fruits'][20]['joint']='/wrong/native/joint'
        with self.assertRaisesRegex(ValueError,'mapping'):validate_target_truss_window(bad,station)

    def test_target_window_planning_requires_matching_initial_station_and_keeps_full_state_scope(self):
        self.manifest=self.target_window_manifest()
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'target_plan',
            max_plants=1,target_truss_windows=True)
        self.assertEqual(report['status'],'planned_only')
        self.assertEqual(report['target_truss_windows']['initial_window']['source_plant_fruit_count'],30)
        self.assertFalse(report['target_truss_windows']['entire_plant_completion_claim'])
        self.manifest['target']=copy.deepcopy(self.manifest['plant']['fruits'][10])
        self.manifest['plant_interaction_variant']['target_truss']='Truss_01'
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        with self.assertRaisesRegex(ValueError,'station'):
            campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'wrong_initial_target',
                max_plants=1,target_truss_windows=True)

    def test_target_window_derived_mismatch_blocks_before_startup_or_native(self):
        self.manifest=self.target_window_manifest()
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        checkpoint,row,_=self.initial_checkpoint()
        preferred=self.preferred(order=(20,19,22));path=self.root/'target_preferred.json';path.write_text(json.dumps(preferred))
        calls=[]
        def switch(source,prior,output,plant,target,**kwargs):
            output.mkdir(parents=True);manifest=self.target_window_manifest(plant_id=plant,truss=1)
            (output/'motion_manifest.json').write_text(json.dumps(manifest))
            (output/'scene.usda').write_bytes((self.assembly/'scene.usda').read_bytes())
            return dict(source_state_preserved=True)
        with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())),\
                patch('run_row_harvest.current_plant_completion_evidence',return_value=dict(all_eligible_native_detached=False)):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'target_derived',
                execute=True,max_plants=1,target_truss_windows=True,initial_native_checkpoint=checkpoint,
                preferred_selection_path=path,preflight_new_windows=True,window_switch=switch,
                benchmark_runner=lambda *a,**k:calls.append(a))
        self.assertEqual(calls,[])
        self.assertEqual(report['status'],'blocked_preserving_last_physical_history')
        self.assertIn('station',report['stations'][0]['blocker'])
        self.assertFalse(report['stations'][0]['physics_execution'])

    def test_preferred_current_plant_can_be_budget_unselected_without_completion_credit(self):
        preferred=self.preferred(order=(20,19,22))
        selected=select_preferred_stations(self.inventory,self.screen,preferred,self.screen_path,
            current_plant_id=22,current_station_y_m=1.6,max_plants=2)
        self.assertEqual([s['plant_id'] for s in selected['stations']],[20,19])
        self.assertEqual([s['plant_id'] for s in selected['budget_unselected']],[22])
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        seq=summary['truss_sequence'];only=[n for n in seq['fruit_names'] if n.startswith('Truss_02_')]
        seq.update(fruit_names=only,fruit_joints={n:seq['fruit_joints'][n] for n in only},
            inventory=dict(detached=only,remaining=[]))
        path.write_text(json.dumps(summary));proof['final_native_summary_sha256']=file_sha256(path)
        completion=current_plant_completion_evidence(manifest,inventory,row,proof)
        self.assertFalse(completion['all_eligible_native_detached'])
        self.assertEqual(exclude_completed_current_plant(selected,completion,2),selected)

    def test_fast_materialization_is_explicit_and_does_not_add_default_switch_kwargs(self):
        checkpoint,row,_=self.initial_checkpoint()
        preferred=self.preferred(order=(20,19,22));path=self.root/'storage_preferred.json';path.write_text(json.dumps(preferred))
        for enabled in (False,True):
            seen=[]
            def switch(*args,**kwargs):
                seen.append(kwargs);raise RuntimeError('No native: capture the existing window-switch call')
            with patch('run_row_harvest.load_checkpoint',return_value=(row,self.initial_arrays())):
                report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/f'storage{enabled}',
                    execute=True,max_plants=1,initial_native_checkpoint=checkpoint,preferred_selection_path=path,
                    window_switch=switch,fast_materialization=enabled,
                    benchmark_runner=lambda *a,**k:self.fail('No native run in this contract test'))
            expected=dict(near_trusses_metadata=self.inventory_path.resolve())
            if enabled:expected['fast_materialization']=True
            self.assertEqual(seen,[expected])
            self.assertEqual('window_materialization' in report,enabled)
            if enabled:
                self.assertEqual(report['window_materialization']['mode'],'s300-whole-canonical-layer-storage-identity-v1')
                self.assertTrue(report['window_materialization']['fresh_native_validation_still_required'])
        for invalid in (None,1,'true'):
            with self.assertRaisesRegex(ValueError,'boolean exact'):
                campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'badstorage',fast_materialization=invalid)

    def test_fast_materialization_cli_reaches_existing_campaign_without_changing_default(self):
        from run_row_harvest import main
        for enabled in (False,True):
            command=['run_row_harvest.py','assembly','inventory','screen','output']
            if enabled:command.append('--fast-materialization')
            with patch('sys.argv',command),patch('run_row_harvest.campaign',return_value=dict(status='planned_only')) as runner,\
                    patch('builtins.print'):
                self.assertEqual(main(),0)
            self.assertIs(runner.call_args.kwargs['fast_materialization'],enabled)

    def test_target_window_startup_and_execution_both_keep_exact_manifest_selection(self):
        self.manifest=self.target_window_manifest()
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        checkpoint,row,_=self.initial_checkpoint();calls=[];switches=[]
        preferred=self.preferred(order=(20,19,22));path=self.root/'target_order.json';path.write_text(json.dumps(preferred))
        def switch(source,prior,output,plant,target,**kwargs):
            switches.append((source,prior,plant,target,kwargs));output.mkdir(parents=True)
            (output/'motion_manifest.json').write_text(json.dumps(self.target_window_manifest(plant_id=plant)))
            (output/'scene.usda').write_bytes((self.assembly/'scene.usda').read_bytes())
            return dict(source_state_preserved=True)
        def runner(command,check):
            cases=Path(command[4]);case=json.loads(cases.read_text())[0];arguments=case['arguments'];calls.append(arguments)
            self.assertNotIn('--near-trusses-metadata',arguments)
            self.assertNotIn('--initial-truss-first',arguments)
            folder=Path(command[command.index('--output')+1]);trial=folder/case['name'];trial.mkdir(parents=True)
            idle=arguments[arguments.index('--mode')+1]=='idle'
            if idle:
                self.assertNotIn('--rail-to-y',arguments)
                summary=dict(mode='idle',completed=True,stop_reason=None,initialization_valid=True,
                    simulated_s=3.,rail_destination=None,native_new_break_count=0)
            else:
                self.assertIn('--rail-to-y',arguments)
                (trial/'cycle_checkpoints/000').mkdir(parents=True)
                summary=self.final_summary(trial,dict(completed=True,stop_reason=None,initialization_valid=True,simulated_s=10.))
            (folder/(cases.stem+'_results.json')).write_text(json.dumps([dict(returncode=0,timeout=False,summary=summary)]))
        def loader(path,**kwargs):return (row,self.initial_arrays()) if Path(path)==checkpoint else (metadata(),{})
        with patch('run_row_harvest.load_checkpoint',side_effect=loader),\
                patch('run_row_harvest.current_plant_completion_evidence',return_value=dict(all_eligible_native_detached=False)):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'target_routed',
                execute=True,max_plants=1,target_truss_windows=True,initial_native_checkpoint=checkpoint,
                preferred_selection_path=path,preflight_new_windows=True,window_switch=switch,benchmark_runner=runner)
        self.assertEqual(len(calls),2);self.assertEqual(len(switches),1)
        self.assertEqual(switches[0][0:4],(self.assembly.resolve(),checkpoint.resolve(),20,'Truss_02_Fruit_01'))
        self.assertEqual(switches[0][4]['near_trusses_metadata'],self.inventory_path.resolve())
        self.assertEqual(report['status'],'finite_campaign_completed',report)
        self.assertEqual(report['stations'][0]['target_truss_window']['source_plant_fruit_count'],30)
        self.assertFalse(report['stations'][0]['target_truss_window']['entire_plant_completion_claim'])

    def test_planning_mode_never_builds_new_assets_or_launches_native(self):
        calls=[]
        report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'plan',
            benchmark_runner=lambda *a,**k:calls.append(a), window_switch=lambda *a,**k:calls.append(a))
        self.assertEqual(calls,[]); self.assertEqual(report['status'],'planned_only')
        self.assertTrue(report['old_screen_not_current_collision_certificate'])
        with self.assertRaises(FileExistsError): campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'plan')

    def test_invalid_station_stops_without_rollback_or_next_window(self):
        switches=[]
        def run(command,check):
            cases=Path(command[4]); folder=Path(command[command.index('--output')+1]); folder.mkdir(parents=True)
            name=json.loads(cases.read_text())[0]['name']
            result=[dict(name=name,returncode=0,timeout=False,summary=dict(completed=False,
                stop_reason='penetration_over_0_5mm',initialization_valid=True,simulated_s=173.))]
            (folder/(cases.stem+'_results.json')).write_text(json.dumps(result))
        report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'invalid',execute=True,
            benchmark_runner=run,window_switch=lambda *a:switches.append(a))
        self.assertEqual(switches,[]); self.assertEqual(len(report['stations']),1)
        self.assertEqual(report['status'],'blocked_preserving_last_physical_history')
        self.assertFalse(report['rollback_performed'])
        self.assertTrue(report['stations'][0]['physics_execution'])

    def test_valid_handoff_reuses_measured_checkpoint_then_commands_physical_rail(self):
        switches=[]; commands=[]
        def switch(source,checkpoint,output,plant,target,*,near_trusses_metadata=None):
            self.assertEqual(near_trusses_metadata,self.inventory_path.resolve())
            switches.append((source,checkpoint,plant,target)); output.mkdir(parents=True)
            manifest=copy.deepcopy(self.manifest); manifest['plant']['source_plant']=f'/World/Greenhouse/Plants/Plant_{plant:02d}'
            (output/'motion_manifest.json').write_text(json.dumps(manifest))
            return dict(measured_state_preserved=True,cart_teleported=False)
        def run(command,check):
            commands.append(command); cases=Path(command[4]); rows=json.loads(cases.read_text()); name=rows[0]['name']
            folder=Path(command[command.index('--output')+1]); trial=folder/name
            (trial/'cycle_checkpoints/002').mkdir(parents=True)
            summary=self.final_summary(trial,dict(completed=True,
                stop_reason=None,initialization_valid=True,simulated_s=10.,
                truss_sequence=dict(inventory=dict(remaining=[]))))
            result=[dict(name=name,returncode=0,timeout=False,summary=summary)]
            (folder/(cases.stem+'_results.json')).write_text(json.dumps(result))
        with patch('run_row_harvest.load_checkpoint',return_value=(metadata(),{})):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'valid',execute=True,
                max_plants=2,benchmark_runner=run,window_switch=switch)
        self.assertEqual(report['status'],'finite_campaign_completed'); self.assertEqual(len(switches),1)
        self.assertEqual(switches[0][2],19)
        self.assertEqual(switches[0][3],'Truss_02_Fruit_01')
        second_cases=json.loads(Path(commands[1][4]).read_text())[0]['arguments']
        self.assertEqual(float(second_cases[second_cases.index('--rail-to-y')+1]),.66)
        self.assertIn('--near-trusses-metadata',second_cases)
        self.assertTrue(report['stations'][1]['window_handoff']['measured_state_preserved'])

    def test_earlier_good_cycle_cannot_replace_later_final_physical_state(self):
        trial=self.root/'trial'; (trial/'cycle_checkpoints/002').mkdir(parents=True)
        summary=self.final_summary(trial,dict(completed=True,stop_reason=None,initialization_valid=True,simulated_s=11.))
        with patch('run_row_harvest.load_checkpoint',return_value=(metadata(10.),{})):
            with self.assertRaisesRegex(ValueError,'roll back'): completed_final_checkpoint(trial,summary)

    def test_final_route_snapshot_requires_explicit_cycle_marker_and_finished_actual_audit(self):
        trial=self.root/'one_cycle';route=trial/'route_checkpoint';route.mkdir(parents=True)
        summary=self.final_summary(trial,dict(completed=True,stop_reason=None,initialization_valid=True,simulated_s=10.))
        row=metadata()
        with patch('run_row_harvest.load_checkpoint',return_value=(row,{})):
            selected,_=completed_final_checkpoint(trial,summary)
            self.assertEqual(selected,route)
            for value in (None,False):
                pending=dict(summary,actual_self_collision_audit_passed=value)
                with self.assertRaisesRegex(ValueError,'no checkpoint rollback'):completed_final_checkpoint(trial,pending)
            raw=copy.deepcopy(row);raw['sequence'].pop('checkpoint_capture_reason')
            with patch('run_row_harvest.load_checkpoint',return_value=(raw,{})):
                with self.assertRaisesRegex(ValueError,'No explicitly valid'):completed_final_checkpoint(trial,summary)
            audit_path=trial/'self_collision_actual.json';audit=json.loads(audit_path.read_text())
            audit['complete']=False;audit_path.write_text(json.dumps(audit))
            with self.assertRaisesRegex(ValueError,'audit/model/stored-trace'):completed_final_checkpoint(trial,summary)
            audit['complete']=True;audit_path.write_text(json.dumps(audit))
            (trial/'states.npz').write_bytes(b'changed actual trace')
            with self.assertRaisesRegex(ValueError,'audit/model/stored-trace'):completed_final_checkpoint(trial,summary)

    def exact_scope_fixture(self):
        manifest=copy.deepcopy(self.manifest);root='/World/ElasticTarget'
        manifest['plant']['root']=root;manifest['plant']['fruits']=[]
        for truss in range(3):
            for number in range(1,11):
                name=f'Truss_{truss:02d}_Fruit_{number:02d}'
                manifest['plant']['fruits'].append(dict(name=name,path=root+'/'+name,
                    joint=root+'/HarvestJoints/'+name,center=[-.9 if truss==0 else -.5,1.6,1.+number*.01]))
        inventory=dict(trusses=[placement(22,y=1.6,truss=truss,centroid_x=-.9 if truss==0 else -.5)
                               for truss in range(3)])
        selected=[fruit for fruit in manifest['plant']['fruits']if not fruit['name'].startswith('Truss_00_')]
        events=[dict(joint=f['joint'],phase='close',t=1.+index*.1)for index,f in enumerate(selected)]
        sequence=dict(fruit_names=[f['name']for f in selected],fruit_joints={f['name']:f['joint']for f in selected},
            inventory=dict(detached=[f['name']for f in selected],remaining=[]),native_break_events=events)
        row=metadata();row.update(break_events=copy.deepcopy(events),sequence=dict(metadata()['sequence'],**copy.deepcopy(sequence)))
        summary=dict(completed=True,stop_reason=None,initialization_valid=True,simulated_s=10.,
                     native_break_events=copy.deepcopy(events),truss_sequence=sequence)
        path=self.root/'eligible_summary.json';path.write_text(json.dumps(summary))
        proof=dict(verified_final_physical_state=True,final_native_summary=str(path),
            final_native_summary_sha256=file_sha256(path),checkpoint='/original/verified/checkpoint',
            actual_self_audit_sha256='d'*64)
        return manifest,inventory,row,summary,path,proof

    def test_exact_all20_native_detachment_skips_current_plant_and_refills_finite_budget(self):
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        evidence=current_plant_completion_evidence(manifest,inventory,row,proof)
        self.assertEqual(evidence['eligible_truss_keys'],['Truss_01','Truss_02'])
        self.assertEqual(len(evidence['exact_eligible_fruits']),20)
        self.assertTrue(evidence['all_eligible_native_detached'])
        selected=select_stations(self.inventory,self.screen,22,1.6,max_plants=2)
        continued=exclude_completed_current_plant(selected,evidence,2)
        self.assertEqual([station['plant_id']for station in continued['stations']],[19,20])
        self.assertEqual(continued['already_detached_current_plant'][0]['plant_id'],22)
        self.assertEqual(continued['side_excluded'],selected['side_excluded'])
        self.assertEqual(continued['finite_screen_unknown'],selected['finite_screen_unknown'])
        self.assertIsNone(evidence['harvest_success']);self.assertFalse(evidence['collection_verified'])
        self.assertEqual(selected['stations'][0]['plant_id'],22)

    def exhausted_scope_fixture(self):
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        name='Truss_02_Fruit_10';joint='/World/ElasticTarget/HarvestJoints/'+name
        events=[event for event in row['break_events'] if event['joint']!=joint]
        for sequence in (row['sequence'],summary['truss_sequence']):
            sequence.update(native_break_events=copy.deepcopy(events),
                attempt_counts={name:6},max_attempts_per_fruit=6,
                inventory=dict(detached=[n for n in sequence['fruit_names'] if n!=name],remaining=[name]))
        row['break_events']=copy.deepcopy(events);summary['native_break_events']=copy.deepcopy(events)
        path.write_text(json.dumps(summary));proof['final_native_summary_sha256']=file_sha256(path)
        return manifest,inventory,row,summary,path,proof

    def test_operational_deferral_keeps_attached_unknown_and_never_promotes_native_completion(self):
        manifest,inventory,row,summary,path,proof=self.exhausted_scope_fixture()
        before=copy.deepcopy(row)
        evidence=current_plant_budget_deferral_evidence(manifest,inventory,row,proof,max_attempts=6)
        self.assertTrue(evidence['operationally_deferred'])
        self.assertFalse(evidence['all_eligible_native_detached'])
        self.assertEqual(evidence['still_attached_fruit_names'],['Truss_02_Fruit_10'])
        self.assertEqual(len(evidence['native_detached_names']),19)
        self.assertEqual(row,before)
        self.assertEqual(evidence['checkpoint'],proof['checkpoint'])
        selected=select_stations(self.inventory,self.screen,22,1.6,max_plants=2)
        result=defer_exhausted_current_plant_proposals(selected,evidence,2)
        self.assertEqual([r['plant_id'] for r in result['stations']],[19,20])
        self.assertEqual(result['deferred_current_plant'][0]['plant_id'],22)
        self.assertNotIn('already_detached_current_plant',result)
        self.assertEqual(result['side_excluded'],selected['side_excluded'])
        self.assertEqual(result['finite_screen_unknown'],selected['finite_screen_unknown'])
        self.assertIn('unknown',result['deferred_current_plant'][0]['reachability'])
        # Exact all20 completion exclusion remains unchanged and refuses19/20.
        native=current_plant_completion_evidence(manifest,inventory,row,proof)
        self.assertIs(exclude_completed_current_plant(selected,native,2),selected)

    def test_budget_deferral_does_not_discard_an_eligible_unattempted_fruit(self):
        manifest,inventory,row,summary,path,proof=self.exhausted_scope_fixture()
        for sequence in (row['sequence'],summary['truss_sequence']):sequence['attempt_counts']={}
        path.write_text(json.dumps(summary));proof['final_native_summary_sha256']=file_sha256(path)
        evidence=current_plant_budget_deferral_evidence(manifest,inventory,row,proof,max_attempts=6)
        self.assertFalse(evidence['operationally_deferred'])
        self.assertEqual(evidence['status'],'eligible_attempt_budget_remains')
        selected=select_stations(self.inventory,self.screen,22,1.6,max_plants=1)
        self.assertEqual(defer_exhausted_current_plant_proposals(selected,evidence,1)['stations'],selected['stations'])

    def test_deferral_rejects_changed_counts_budget_and_incomplete_selected_scope(self):
        manifest,inventory,row,summary,path,proof=self.exhausted_scope_fixture()
        with self.assertRaisesRegex(ValueError,'unchanged native attempt ledger'):
            current_plant_budget_deferral_evidence(manifest,inventory,row,proof,max_attempts=5)
        bad=copy.deepcopy(row);bad['sequence']['attempt_counts']['Truss_02_Fruit_10']=5
        with self.assertRaisesRegex(ValueError,'unchanged native attempt ledger'):
            current_plant_budget_deferral_evidence(manifest,inventory,bad,proof,max_attempts=6)
        for sequence in (row['sequence'],summary['truss_sequence']):
            sequence['attempt_counts']['Truss_02_Fruit_10']=True
        path.write_text(json.dumps(summary));proof['final_native_summary_sha256']=file_sha256(path)
        with self.assertRaisesRegex(ValueError,'invalid exact fruit identities or counts'):
            current_plant_budget_deferral_evidence(manifest,inventory,row,proof,max_attempts=6)
        for sequence in (row['sequence'],summary['truss_sequence']):
            sequence['attempt_counts']['Truss_02_Fruit_10']=6
            sequence['fruit_names']=[n for n in sequence['fruit_names']if n.startswith('Truss_02_')]
            sequence['fruit_joints']={n:j for n,j in sequence['fruit_joints'].items()if n.startswith('Truss_02_')}
            sequence['inventory']['detached']=[n for n in sequence['inventory']['detached']if n.startswith('Truss_02_')]
        path.write_text(json.dumps(summary));proof['final_native_summary_sha256']=file_sha256(path)
        with self.assertRaisesRegex(ValueError,'full eligible scope'):
            current_plant_budget_deferral_evidence(manifest,inventory,row,proof,max_attempts=6)

    def test_exhausted_current_plant_handoff_uses_same_exact_parent_for_new_window(self):
        manifest,inventory,row,summary,path,proof=self.exhausted_scope_fixture()
        self.manifest=manifest;(self.assembly/'motion_manifest.json').write_text(json.dumps(manifest))
        # Keep the other actual placement proposals in the exact inventory.
        self.inventory['trusses']+=inventory['trusses'][:2]
        self.inventory_path.write_text(json.dumps(self.inventory))
        self.screen['inventory_sha256']=file_sha256(self.inventory_path)
        self.screen_path.write_text(json.dumps(self.screen))
        checkpoint=Path(proof['checkpoint']);calls=[]
        proof['measured_initial_station_y_m']=1.6
        def switch(source,prior,output,plant,target,*,near_trusses_metadata):
            calls.append((source,prior,plant));raise RuntimeError('Sentinel after exact deferred-parent handoff')
        with patch('run_row_harvest.validated_initial_checkpoint',return_value=(row,proof)):
            result=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'deferred',
                execute=True,max_plants=1,initial_native_checkpoint=checkpoint,
                defer_exhausted_current_plant=True,window_switch=switch,
                benchmark_runner=lambda *a,**k:self.fail('No native before the preserved-parent window switch'))
        self.assertEqual(calls,[(self.assembly.resolve(),checkpoint.resolve(),19)])
        self.assertEqual(result['selection']['deferred_current_plant'][0]['plant_id'],22)
        self.assertFalse(result['selection']['current_plant_native_completion']['all_eligible_native_detached'])
        self.assertEqual(row['sequence']['inventory']['remaining'],['Truss_02_Fruit_10'])
        self.assertFalse(result['rollback_performed'])

    def test_budget_deferral_opt_in_rejects_missing_actual_final_parent(self):
        with self.assertRaisesRegex(ValueError,'verified final native checkpoint'):
            campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'not_a_parent',
                defer_exhausted_current_plant=True)

    def blocked_other_trusses_fixture(self):
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        fruits=[f for f in manifest['plant']['fruits']if f['name'].startswith('Truss_02_')]
        names=sorted(f['name']for f in fruits);events=[e for e in row['break_events']if 'Truss_02_'in e['joint']]
        sequence=dict(fruit_names=names,fruit_joints={f['name']:f['joint']for f in fruits},
            inventory=dict(detached=names,remaining=[]),native_break_events=copy.deepcopy(events),
            attempt_counts={names[0]:1},max_attempts_per_fruit=6)
        row.update(break_events=copy.deepcopy(events));row['sequence'].update(copy.deepcopy(sequence))
        summary.update(native_break_events=copy.deepcopy(events),truss_sequence=copy.deepcopy(sequence))
        path.write_text(json.dumps(summary));proof['final_native_summary_sha256']=file_sha256(path)
        checkpoint=self.root/'actual_parent/cycle_checkpoints/000';checkpoint.mkdir(parents=True)
        (checkpoint/'checkpoint.json').write_text('exact actual parent metadata fixture')
        (checkpoint/'checkpoint.npz').write_bytes(b'exact measured native state fixture')
        proof.update(checkpoint=str(checkpoint),measured_initial_station_y_m=1.6)
        (self.assembly/'motion_manifest.json').write_text(json.dumps(manifest))
        inventory_path=self.root/'exact_inventory.json';inventory_path.write_text(json.dumps(inventory))
        plan=self.root/'blocked_plan.json';deferred=sorted(f['name']for f in manifest['plant']['fruits']if f['name'].startswith('Truss_01_'))
        plan.write_text(json.dumps(dict(truss_axis=dict(truss_key='Truss_01',fruit_names=deferred))))
        hashes=dict(scene=file_sha256(self.assembly/'scene.usda'),manifest=file_sha256(self.assembly/'motion_manifest.json'),
                    materialized_dependency=None,plan=file_sha256(plan))
        audit=self.root/'blocked_env.json';audit.write_text(json.dumps(dict(complete=True,passed=False,status='blocked',
            native_runs=0,physics_unchanged=True,source_sha256=hashes,allowed_truss_key='Truss_01',
            hard_hits=[dict(robot_geom='/World/ActuatedTool/L/part_122',environment_geom='/World/Greenhouse/Plants/Plant_20/Stem/mesh_132',
                            obstacle_class='static_leaf_obstacle')],requested_hz=8,hard_intersection_sample_pairs=1)))
        declaration=dict(schema='s300-explicit-blocked-other-trusses-v1',source_plant=manifest['plant']['source_plant'],
            reachability='unknown',plant_complete=False,training_eligible=False,harvest_success=None,
            parent=dict(source_assembly=str(self.assembly.resolve()),scene_sha256=manifest['scene_sha256'],
                manifest_sha256=file_sha256(self.assembly/'motion_manifest.json'),materialized_dependency=None,
                inventory_sha256=file_sha256(inventory_path),checkpoint=str(checkpoint.resolve()),
                checkpoint_metadata_sha256=row['metadata_sha256'],checkpoint_json_sha256=file_sha256(checkpoint/'checkpoint.json'),
                checkpoint_npz_sha256=file_sha256(checkpoint/'checkpoint.npz'),
                final_native_summary_sha256=proof['final_native_summary_sha256'],actual_self_audit_sha256=proof['actual_self_audit_sha256']),
            completed_trusses=[dict(truss_key='Truss_02',fruit_names=names)],
            deferred_trusses=[dict(truss_key='Truss_01',still_attached_fruit_names=deferred,reason='finite_planning_budget_blocked',
                reachability='unknown',planning_blockers=[dict(source_assembly=str(self.assembly.resolve()),
                    plan=str(plan),plan_sha256=file_sha256(plan),environment_audit=str(audit),environment_audit_sha256=file_sha256(audit),
                    source_sha256=hashes,state_relation='separate_historical_diagnostic')])])
        declaration_path=self.root/'explicit_deferral.json';declaration_path.write_text(json.dumps(declaration))
        return manifest,inventory,inventory_path,row,summary,proof,declaration,declaration_path

    def test_explicit_blocked_deferral_preserves_unattempted_ten_and_original_twenty_completion_gate(self):
        manifest,inventory,inv_path,row,summary,proof,declaration,path=self.blocked_other_trusses_fixture()
        before=copy.deepcopy(row)
        evidence=blocked_other_trusses_evidence(self.assembly,manifest,inventory,inv_path,row,proof,path)
        self.assertTrue(evidence['operationally_deferred']);self.assertFalse(evidence['all_eligible_native_detached'])
        self.assertFalse(evidence['plant_complete']);self.assertEqual(len(evidence['still_attached_fruit_names']),10)
        self.assertEqual(row,before);self.assertFalse(evidence['attempt_counts_reset'])
        self.assertFalse(evidence['planning_blocker_proofs'][0]['current_terminal_reachability_certified'])
        selected=select_stations(self.inventory,self.screen,22,1.6,max_plants=1)
        result=defer_blocked_other_truss_proposals(selected,evidence,1)
        self.assertEqual(result['stations'][0]['plant_id'],19)
        self.assertNotIn('current_plant_finite_budget_deferral',result)
        self.assertNotIn('already_detached_current_plant',result)
        self.assertEqual(result['finite_screen_unknown'],selected['finite_screen_unknown'])
        native=current_plant_completion_evidence(manifest,inventory,row,proof)
        self.assertFalse(native['all_eligible_native_detached']);self.assertIs(exclude_completed_current_plant(selected,native,1),selected)
        # Ordinary exhaustion continues refusing unrecorded/unattempted Truss01.
        with self.assertRaisesRegex(ValueError,'full eligible scope'):
            current_plant_budget_deferral_evidence(manifest,inventory,row,proof,max_attempts=6)

    def test_explicit_deferral_rejects_wrong_terminal_parent_and_false_plant_completion(self):
        fixture=self.blocked_other_trusses_fixture();manifest,inventory,inv_path,row,summary,proof,good,path=fixture
        for field in ('checkpoint','checkpoint_metadata_sha256','checkpoint_npz_sha256','scene_sha256','actual_self_audit_sha256'):
            bad=copy.deepcopy(good);bad['parent'][field]='wrong-parent';path.write_text(json.dumps(bad))
            with self.assertRaisesRegex(ValueError,'parent/hash'):
                blocked_other_trusses_evidence(self.assembly,manifest,inventory,inv_path,row,proof,path)
        bad=copy.deepcopy(good);bad['plant_complete']=True;path.write_text(json.dumps(bad))
        with self.assertRaisesRegex(ValueError,'noncompletion'):
            blocked_other_trusses_evidence(self.assembly,manifest,inventory,inv_path,row,proof,path)

    def test_explicit_deferral_rejects_incomplete_fullscope_missing_native_break_and_wrong_attached_inventory(self):
        manifest,inventory,inv_path,row,summary,proof,good,path=self.blocked_other_trusses_fixture()
        for change in ('omit_completed','missing_attached','duplicate_scope','fake_completed'):
            bad=copy.deepcopy(good)
            if change=='omit_completed':bad['completed_trusses']=[]
            elif change=='missing_attached':bad['deferred_trusses'][0]['still_attached_fruit_names'].pop()
            elif change=='duplicate_scope':bad['deferred_trusses'].append(copy.deepcopy(bad['deferred_trusses'][0]))
            else:bad['completed_trusses'][0]['truss_key']='Truss_01';bad['deferred_trusses'][0]['truss_key']='Truss_02'
            path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):blocked_other_trusses_evidence(self.assembly,manifest,inventory,inv_path,row,proof,path)
        path.write_text(json.dumps(good));badrow=copy.deepcopy(row);badrow['break_events'].pop()
        with self.assertRaisesRegex(ValueError,'native break evidence disagree'):
            blocked_other_trusses_evidence(self.assembly,manifest,inventory,inv_path,badrow,proof,path)

    def test_explicit_deferral_requires_complete_blocked_environment_not_inconclusive_or_stale_plan(self):
        manifest,inventory,inv_path,row,summary,proof,good,path=self.blocked_other_trusses_fixture()
        blocker=good['deferred_trusses'][0]['planning_blockers'][0];audit=Path(blocker['environment_audit']);original=json.loads(audit.read_text())
        for changes in ({'complete':False},{'status':'inconclusive'},{'passed':True},{'allowed_truss_key':'Truss_02'},
                        {'hard_hits':[]},{'source_sha256':{}},{'native_runs':1}):
            audit.write_text(json.dumps(dict(original,**changes)));bad=copy.deepcopy(good)
            bad['deferred_trusses'][0]['planning_blockers'][0]['environment_audit_sha256']=file_sha256(audit)
            path.write_text(json.dumps(bad))
            with self.assertRaisesRegex(ValueError,'incomplete, clear, wrong target'):
                blocked_other_trusses_evidence(self.assembly,manifest,inventory,inv_path,row,proof,path)
        audit.write_text(json.dumps(original));path.write_text(json.dumps(good));Path(blocker['plan']).write_text('changed after report')
        with self.assertRaisesRegex(ValueError,'audit SHA'):
            blocked_other_trusses_evidence(self.assembly,manifest,inventory,inv_path,row,proof,path)

    def test_explicit_deferral_switches_from_same_valid_parent_and_route_failure_never_rolls_back(self):
        manifest,inventory,inv_path,row,summary,proof,good,path=self.blocked_other_trusses_fixture()
        self.manifest=manifest;self.inventory['trusses']=[r for r in self.inventory['trusses']if r['plant_id']!=22]+inventory['trusses']
        self.inventory_path.write_text(json.dumps(self.inventory));good['parent']['inventory_sha256']=file_sha256(self.inventory_path)
        path.write_text(json.dumps(good));self.screen['inventory_sha256']=file_sha256(self.inventory_path);self.screen_path.write_text(json.dumps(self.screen))
        calls=[]
        def switch(source,prior,output,plant,target,*,near_trusses_metadata):
            calls.append((source,prior,plant));raise RuntimeError('Blocked future window; preserve actual physical parent')
        with patch('run_row_harvest.validated_initial_checkpoint',return_value=(row,proof)):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'explicit_campaign',execute=True,
                max_plants=1,initial_native_checkpoint=Path(proof['checkpoint']),defer_blocked_other_trusses=path,
                window_switch=switch,benchmark_runner=lambda *a,**kw:self.fail('No native before the exact window switch'))
        self.assertEqual(calls,[(self.assembly.resolve(),Path(proof['checkpoint']).resolve(),19)])
        self.assertEqual(report['status'],'blocked_preserving_last_physical_history');self.assertFalse(report['rollback_performed'])
        self.assertFalse(report['selection']['current_plant_native_completion']['all_eligible_native_detached'])
        self.assertEqual(report['selection']['explicit_other_truss_operational_deferral']['still_attached_fruit_names'],good['deferred_trusses'][0]['still_attached_fruit_names'])
        self.assertFalse(report['stations'][0]['physics_execution'])

    def test_explicit_deferral_requires_actual_parent_and_cannot_mix_exhaustion_policy(self):
        with self.assertRaisesRegex(ValueError,'verified final native checkpoint'):
            campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'missing_parent',defer_blocked_other_trusses=self.root/'proof.json')
        with self.assertRaisesRegex(ValueError,'one explicit operational deferral'):
            campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'mixed',initial_native_checkpoint=self.root/'parent',
                defer_exhausted_current_plant=True,defer_blocked_other_trusses=self.root/'proof.json')

    def test_completed10_or_twenty_wrong_joint_events_never_skip_eligible20(self):
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        seq=summary['truss_sequence'];only=[name for name in seq['fruit_names']if name.startswith('Truss_02_')]
        seq.update(fruit_names=only,fruit_joints={name:seq['fruit_joints'][name]for name in only},
                   inventory=dict(detached=only,remaining=[]))
        path.write_text(json.dumps(summary));proof['final_native_summary_sha256']=file_sha256(path)
        self.assertFalse(current_plant_completion_evidence(manifest,inventory,row,proof)['all_eligible_native_detached'])
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        wrong=[dict(joint=f['joint'],phase='close',t=1.)for f in manifest['plant']['fruits']
               if not f['name'].startswith('Truss_01_')]
        row['break_events']=wrong;row['sequence']['native_break_events']=wrong
        summary['native_break_events']=wrong;seq=summary['truss_sequence']
        seq['inventory']=dict(detached=[name for name in seq['fruit_names']if name.startswith('Truss_02_')],
                              remaining=[name for name in seq['fruit_names']if name.startswith('Truss_01_')])
        path.write_text(json.dumps(summary));proof['final_native_summary_sha256']=file_sha256(path)
        evidence=current_plant_completion_evidence(manifest,inventory,row,proof)
        self.assertEqual(len(wrong),20);self.assertFalse(evidence['all_eligible_native_detached'])
        self.assertEqual(len(evidence['eligible_remaining_names']),10)

    def test_completion_rejects_fabricated_inventory_and_unproven_imported_event(self):
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        # A full detached list cannot manufacture the twentieth native event.
        row['break_events']=row['break_events'][:-1];row['sequence']['native_break_events']=row['break_events']
        summary['native_break_events']=row['break_events'];path.write_text(json.dumps(summary))
        proof['final_native_summary_sha256']=file_sha256(path)
        with self.assertRaisesRegex(ValueError,'differs from exact native break'):current_plant_completion_evidence(manifest,inventory,row,proof)
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        event=row['break_events'][0];event.update(type='imported_native_prior',native_new_event=False,
                                                original_event=dict(event),original_t=1.,source_checkpoint_time_s=10.)
        # The prior source model SHA is missing, so this is not native proof.
        summary['native_break_events']=copy.deepcopy(row['break_events'])
        row['sequence']['native_break_events']=copy.deepcopy(row['break_events'])
        path.write_text(json.dumps(summary));proof['final_native_summary_sha256']=file_sha256(path)
        with self.assertRaisesRegex(ValueError,'original event/source'):current_plant_completion_evidence(manifest,inventory,row,proof)

    def test_completion_requires_exact_fruit_body_native_joint_mapping_and_final_summary_hash(self):
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        wrong=copy.deepcopy(manifest)
        wrong['plant']['fruits'][10]['joint']='/World/ElasticTarget/Joints/a_different_branch'
        with self.assertRaisesRegex(ValueError,'harvest joint identity'):current_plant_completion_evidence(wrong,inventory,row,proof)
        path.write_text(json.dumps(dict(summary,changed_after_final_validation=True)))
        with self.assertRaisesRegex(ValueError,'summary SHA changed'):current_plant_completion_evidence(manifest,inventory,row,proof)

    def test_full_current_plant_campaign_handoff_preserves_initial_checkpoint_without_rerun(self):
        manifest,inventory,row,summary,path,proof=self.exact_scope_fixture()
        self.manifest=manifest;(self.assembly/'motion_manifest.json').write_text(json.dumps(manifest))
        # Preserve all other proposals/exclusions in the existing complete farm
        # inventory; only add the missing same-plant truss placement identities.
        self.inventory['trusses']=[p for p in self.inventory['trusses']if p['plant_id']!=22]+inventory['trusses']
        self.inventory_path.write_text(json.dumps(self.inventory));self.screen['inventory_sha256']=file_sha256(self.inventory_path)
        self.screen_path.write_text(json.dumps(self.screen))
        checkpoint,loaded,base=self.initial_checkpoint();loaded.update(break_events=row['break_events'])
        loaded['sequence'].update(summary['truss_sequence']);base.update(summary)
        self.final_summary(checkpoint.parent.parent,base,manifest['scene_sha256']);calls=[]
        def switch(source,prior,output,plant,target,*,near_trusses_metadata):
            calls.append((source,prior,plant));raise RuntimeError('Stop before native, after exact measured parent handoff')
        with patch('run_row_harvest.load_checkpoint',return_value=(loaded,self.initial_arrays())):
            report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'complete20',
                execute=True,max_plants=1,initial_native_checkpoint=checkpoint,window_switch=switch,
                benchmark_runner=lambda *a,**k:self.fail('An empty current plant must not be rerun'))
        self.assertEqual(calls,[(self.assembly.resolve(),checkpoint.resolve(),19)])
        self.assertTrue(report['selection']['current_plant_native_completion']['all_eligible_native_detached'])
        self.assertEqual(report['selection']['already_detached_current_plant'][0]['plant_id'],22)
        self.assertFalse(report['stations'][0]['physics_execution']);self.assertFalse(report['rollback_performed'])

    def test_managed_arguments_and_stale_inventory_are_rejected(self):
        with self.assertRaises(ValueError): motion_arguments(DEFAULT_ARGUMENTS+['--isolate-background'],self.inventory_path,dict(station_y_m=1.6))
        self.inventory_path.write_text(json.dumps(dict(trusses=[])))
        with self.assertRaisesRegex(ValueError,'inventory SHA'): campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'bad')

    def test_orchestration_python_is_explicit_and_separate_from_native(self):
        import sys
        report=campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'python',
            benchmark_python=sys.executable,native_python='/different/native/python.sh')
        self.assertEqual(report['benchmark_python'],str(Path(sys.executable).resolve()))
        self.assertEqual(report['native_python'],'/different/native/python.sh')
        with self.assertRaisesRegex(ValueError,'Benchmark Python'):
            campaign(self.assembly,self.inventory_path,self.screen_path,self.root/'absent',
                execute=True,benchmark_python=self.root/'missing')
        self.assertFalse((self.root/'absent').exists())


if __name__=='__main__':unittest.main()
