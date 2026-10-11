import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from compact_motion_assets import file_sha256
from run_row_harvest import (DEFAULT_ARGUMENTS, campaign, completed_final_checkpoint,
                             motion_arguments, select_stations,select_preferred_stations,
                             validated_initial_checkpoint,startup_preflight_arguments,preflight_result,
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
