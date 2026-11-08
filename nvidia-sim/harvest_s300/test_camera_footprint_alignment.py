"""A missing visible footprint requests a measured new view, never fake success."""
import ast,copy,json,tempfile,unittest
import numpy as np
from pathlib import Path
from unittest.mock import patch
import camera_patrol_runner as runner
import test_camera_patrol_runner as fixtures
import test_camera_forward_pass as forward_fixtures


class FootprintOutcomeTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.trial=Path(self.temp.name)
        self.scan=dict(index=0,purpose='camera_scan_prefix',status='motion_completed')
        self.sequence=dict(cycle_plans=[self.scan],termination='group_no_group_meets_minimum')
        self.summary=dict(completed=True,stop_reason=None,simulated_s=5.,truss_sequence=self.sequence,
            selected_cycle_evidence=dict(cycles=[],scans=[dict(status='motion_completed',harvest_attempt=False,actual_recorded_phases=['camera_pan'])]))
        self.metadata=dict(sequence=self.sequence)
        self.decision=dict(action='harvest',target=dict(truss_key='Truss_02'))
        self.history=[dict(reason='no_group_meets_minimum',minimum_fruits=2,best_geometric_count=1,
            camera_region=dict(selected_truss='Truss_02',observation=dict(stale=False,timestamp_s=5.,current_timestamp_s=5.,objects_path='/actual/frame10/objects.json'),
                accepted_observations=[dict(name='Truss_02_Fruit_03',estimated_center_world_m=[-.5,1.,1.0745]),dict(name='Truss_02_Fruit_07',estimated_center_world_m=[-.5,1.,1.016])]))]
        self.write()
        self.exposure=patch.object(runner,'latest_camera_observation',return_value=dict(objects_path='/actual/frame10/objects.json'))
        self.exposure.start();self.addCleanup(self.exposure.stop)
    def write(self):
        (self.trial/'camera_region_decisions.json').write_text(json.dumps(self.history))
    def outcome(self):return runner.completed_patrol_harvest_outcome(self.summary,self.metadata,self.decision,self.trial)
    def test_actual_zero_group_is_distinct_from_finite_ik_failure_or_success(self):
        result=self.outcome();self.assertEqual(result['outcome'],'camera_footprint_unavailable')
        self.assertTrue(result['requires_fresh_camera_alignment']);self.assertFalse(result['finite_planning_exhausted'])
        self.assertEqual(result['harvest_credit'],0);self.assertEqual(result['actual_completed_cycles'],0)
        self.assertEqual(result['evidence']['rejected_proposal_count'],0)
    def test_missing_stale_wrong_truss_or_old_receipt_is_not_alignment_permission(self):
        original=copy.deepcopy(self.history)
        for key,value in(('stale',True),('timestamp_s',4.),('current_timestamp_s',4.)):
            self.history=copy.deepcopy(original);self.history[0]['camera_region']['observation'][key]=value;self.write()
            with self.assertRaisesRegex(ValueError,'fresh terminal observation'):self.outcome()
        self.history=copy.deepcopy(original);self.history[0]['camera_region']['selected_truss']='Truss_00';self.write()
        with self.assertRaisesRegex(ValueError,'fresh terminal observation'):self.outcome()
        (self.trial/'camera_region_decisions.json').unlink()
        with self.assertRaises(ValueError):self.outcome()
    def test_below_visible_minimum_is_not_alignment_or_planning_exhaustion(self):
        for count in(0,1):
            self.history[0]['camera_region']['accepted_observations']=self.history[0]['camera_region']['accepted_observations'][:count];self.write()
            result=self.outcome();self.assertEqual(result['outcome'],'camera_visible_group_below_minimum')
            self.assertFalse(result['requires_fresh_camera_alignment']);self.assertFalse(result['requires_forward_probe'])
            self.assertFalse(result['finite_planning_exhausted']);self.assertEqual(result['harvest_credit'],0)
    def test_nonfinite_position_or_invalid_count_is_rejected(self):
        original=copy.deepcopy(self.history)
        for field,value in(('minimum_fruits',True),('best_geometric_count',float('nan')),('best_geometric_count',2)):
            self.history=copy.deepcopy(original);self.history[0][field]=value;self.write()
            with self.assertRaisesRegex(ValueError,'finite visible-fruit'):self.outcome()
        self.history=copy.deepcopy(original);self.history[0]['camera_region']['accepted_observations'][0]['estimated_center_world_m'][2]=float('nan');self.write()
        with self.assertRaisesRegex(ValueError,'finite visible-fruit'):self.outcome()
    def test_other_trial_exposure_is_rejected_even_at_same_five_second_timestamp(self):
        self.history[0]['camera_region']['observation']['objects_path']='/other/trial/frame10/objects.json';self.write()
        with self.assertRaisesRegex(ValueError,'exact terminal camera exposure'):self.outcome()
    def test_zero_group_publication_reconciliation_keeps_all_arrays_and_controller_exact(self):
        accepted=self.trial/'cycle_checkpoints/000';final_path=self.trial/'route_checkpoint'
        self.scan.update(end_s=5.,planned_end_s=5.,measured_frame_index=0,end_frame_index=1)
        accepted_sequence=dict(cycle_plans=[self.scan],termination=None,checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True)
        final_sequence=dict(self.sequence,checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True)
        accepted_meta=dict(time_s=5.,metadata_sha256='accepted',controller_state=dict(targets=[1.]),break_events=[],sequence=accepted_sequence)
        final_meta=dict(accepted_meta,metadata_sha256='final',sequence=final_sequence)
        self.summary['truss_sequence']=final_sequence;self.metadata=accepted_meta
        (self.trial/'truss_sequence.json').write_text(json.dumps(self.sequence))
        arrays={f'array_{i}':np.zeros(3)for i in range(10)}
        mapping={str(accepted):(accepted_meta,arrays),str(final_path):(final_meta,copy.deepcopy(arrays))}
        def load(path):return copy.deepcopy(mapping[str(path)])
        with patch.object(runner,'load_checkpoint',side_effect=load):
            outcome=runner.completed_patrol_harvest_outcome(self.summary,accepted_meta,self.decision,self.trial,checkpoint_path=accepted)
            proof=outcome['evidence']['same_state_final_planning_sequence_proof']
            self.assertTrue(proof['all_array_dtypes_shapes_bytes_equal']);self.assertEqual(len(proof['all_native_array_keys']),10)
            self.assertEqual(proof['saved_sequence_completion_marker_omissions'],dict(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True))
            for key in arrays:
                mapping[str(final_path)][1][key][0]=.001
                with self.assertRaisesRegex(ValueError,'native array differs'):
                    runner.completed_patrol_harvest_outcome(self.summary,accepted_meta,self.decision,self.trial,checkpoint_path=accepted)
                mapping[str(final_path)][1][key][0]=0.
            final_meta['controller_state']=dict(targets=[2.])
            with self.assertRaisesRegex(ValueError,'physical/controller/provenance'):
                runner.completed_patrol_harvest_outcome(self.summary,accepted_meta,self.decision,self.trial,checkpoint_path=accepted)
            final_meta['controller_state']=accepted_meta['controller_state']
            saved=dict(self.sequence,checkpoint_physics_valid=False);(self.trial/'truss_sequence.json').write_text(json.dumps(saved))
            with self.assertRaisesRegex(ValueError,'disagrees'):
                runner.completed_patrol_harvest_outcome(self.summary,accepted_meta,self.decision,self.trial,checkpoint_path=accepted)
    def test_actual_stop_unknown_termination_and_unknown_rejection_keep_blocking(self):
        self.summary['completed']=False
        with self.assertRaises(ValueError):self.outcome()
        self.summary['completed']=True;self.sequence['termination']='group_unknown_error'
        with self.assertRaises(ValueError):self.outcome()
        self.sequence['termination']='group_no_group_meets_minimum';self.sequence['cycle_plans'].append(dict(index=1,status='planning_rejected',error='unknown runtime error'))
        self.summary['selected_cycle_evidence']['cycles']=[dict(status='planning_rejected',recorded_physical_cycle=False,planned_manipulation_started=False)]
        with self.assertRaises(ValueError):self.outcome()


class AlignmentCallbackTests(unittest.TestCase):
    setUp=fixtures.CameraPatrolRunnerTests.setUp
    write=fixtures.CameraPatrolRunnerTests.write
    completed=fixtures.CameraPatrolRunnerTests.completed
    measured=fixtures.CameraPatrolRunnerTests.measured
    observation_files=fixtures.CameraPatrolRunnerTests.observation_files
    benchmark=fixtures.CameraPatrolRunnerTests.benchmark
    def zero_group_benchmark(self,command,check):
        self.benchmark(command,check)
        case=Path(command[command.index('--output')-1]);out=Path(command[command.index('--output')+1]);row=json.loads(case.read_text())[0]
        if '--camera-target-truss'not in row['arguments']:return
        trial=out/row['name'];self.events=[];summary=json.loads((trial/'summary.json').read_text())
        summary['truss_sequence']['cycle_plans']=summary['truss_sequence']['cycle_plans'][:1]
        summary['truss_sequence']['termination']='group_no_group_meets_minimum';summary['selected_cycle_evidence']['cycles']=[]
        self.observation_files(trial,summary,empty=False)
        self.write(trial/'camera_region_decisions.json',[dict(reason='no_group_meets_minimum',minimum_fruits=2,best_geometric_count=1,
            camera_region=dict(selected_truss='Truss_00',accepted_observations=[dict(name='Truss_00_Fruit_01',estimated_center_world_m=[-.5,.3,1.]),dict(name='Truss_00_Fruit_02',estimated_center_world_m=[-.5,.3,1.06])],
                observation=dict(stale=False,timestamp_s=5.,current_timestamp_s=5.,objects_path=str(trial/'rgbd/frame_00000/objects.json'))))])
        self.write(trial/'summary.json',summary);self.write(out/(case.stem+'_results.json'),[dict(timeout=False,returncode=0,summary=summary)])
    def test_forward_pass_reuses_existing_actual_lift_for_visibility_not_finite_deferral(self):
        self.manifest['plant_interaction_variant']=dict(target_truss='Truss_00');self.write(self.assembly/'motion_manifest.json',self.manifest)
        with patch.object(runner,'target_truss_camera_scope',return_value=dict(target_truss_window=dict(target_truss='Truss_00'))),\
                patch('patrol_lift_search.lift_bounds_for_assembly',return_value=[0.,.75]):
            result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'alignment',fixtures.BASE,
                execute=True,max_decisions=4,benchmark_runner=self.zero_group_benchmark,adaptive_lift=True,target_truss_windows=True,forward_pass=True)
        self.assertEqual([x['decision']['action']for x in result['segments']],['scan','move','harvest','lift_scan'])
        self.assertTrue(result['segments'][2]['harvest_outcome']['requires_fresh_camera_alignment'])
        self.assertFalse(result['segments'][2]['harvest_outcome']['finite_planning_exhausted'])
        ledger=json.loads((self.root/'alignment/camera_patrol_queue.json').read_text());item=ledger['queue'][self.source+'/Truss_00']
        self.assertEqual(item['blockers'],[]);self.assertEqual(item['planning_failures'],[])
        self.assertEqual(result['achieved_distinct_trusses'],0)


class InitialAlignmentResumeTests(unittest.TestCase):
    def test_validated_latest_receipt_proposes_lift_before_repeating_harvest(self):
        tree=ast.parse(Path(runner.__file__).read_text());function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='camera_patrol_campaign')
        block=next(n for n in function.body if isinstance(n,ast.If)and 'initial_lift_height_m is None' in ast.unparse(n.test))
        program=compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'actual_camera_alignment_initial_dispatch','exec')
        for wrong in(False,True):
            p,_=forward_fixtures.ForwardPassDecisions().fixture();key='/World/Greenhouse/Plants/Plant_20/Truss_00';item=p.queue[key]
            item['suggested_lift_m']=.14;item['visible_now']=False;item['last_observation']['objects_path']='/actual/latest/objects.json'
            pending=dict(target_key=key,outcome=dict(requires_fresh_camera_alignment=True,evidence=dict(camera_observation=dict(objects_path='/wrong'if wrong else '/actual/latest/objects.json'))))
            ns=dict(initial_lift_height_m=None,adaptive_lift=True,prior_pending_plan_failure=pending,patrol=p,copy=copy,
                station=1.,lift=.097,limits=None,lift_limits=[0.,.75],report={},decision=dict(action='scan'))
            exec(program,ns)
            self.assertFalse(p.queue[key]['visible_now']);self.assertEqual(p.native_detached_keys,set())
            if wrong:self.assertEqual(ns['decision']['action'],'scan');self.assertIsNotNone(ns['prior_pending_plan_failure'])
            else:
                self.assertEqual(ns['decision']['action'],'lift_scan');self.assertEqual(ns['decision']['next_lift_m'],.14)
                self.assertIsNone(ns['prior_pending_plan_failure']);self.assertEqual(len(item['camera_lift_search']),1)
                self.assertTrue(ns['report']['initial_camera_alignment_recovery']['requires_new_actual_observation_before_harvest'])


if __name__=='__main__':unittest.main()
