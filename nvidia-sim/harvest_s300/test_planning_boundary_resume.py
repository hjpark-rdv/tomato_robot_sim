"""Terminal planning-pause checks use real write-once checkpoint serialization."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from compact_motion_assets import file_sha256
from native_route_checkpoint import save_checkpoint, load_checkpoint
from resume_native_state import imported_break_markers
from run_row_harvest import validated_initial_checkpoint


class PlanningBoundaryResumeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.root = Path(self.tmp.name)
        self.assembly = self.root/'assembly'; self.assembly.mkdir()
        (self.assembly/'scene.usda').write_text('#usda 1.0\n# immutable checkpoint test\n')
        self.sha = file_sha256(self.assembly/'scene.usda')
        self.anchor = '/World/ElasticTarget/Pedicel_01'
        self.fruit = '/World/ElasticTarget/Truss_00_Fruit_01'
        self.joint = '/World/ElasticTarget/HarvestJoints/Truss_00_Fruit_01'
        self.base = '/World/HarvestRobot/Robot/world'
        self.manifest = dict(scene_sha256=self.sha,
            plant=dict(root='/World/ElasticTarget',source_plant='/World/Greenhouse/Plants/Plant_25',
                bodies=[dict(path=self.anchor)],fruits=[dict(path=self.fruit,joint=self.joint)]),
            target=dict(path=self.fruit),rail=dict(base_body=self.base,dof_name='root_joint',
                initial_station_y_m=2.,source_base_position_world_m=[0.,2.,.5]))
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))

    def tearDown(self): self.tmp.cleanup()

    def fixture(self, name='trial', final_updates=None, summary_updates=None, trace_change=None):
        trial=self.root/name; cp=trial/'cycle_checkpoints/000'
        completed=dict(index=0,status='motion_completed',end_s=2.,end_frame_index=2,stop_reason=None)
        sequence=dict(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True,
            cycle_plans=[completed],termination=None,attempt_counts={'Truss_00_Fruit_01':1})
        args=dict(assembly=self.assembly,scene_sha256=self.sha,time_s=2.,
            body_paths=[self.anchor,self.fruit,self.base],
            body_poses_world=[[0,0,0,0,0,0,1],[1,2,3,0,0,0,1],[0,2,.5,0,0,0,1]],
            body_velocities_world=np.arange(18,dtype=float).reshape(3,6)/100,
            robot_dof_names=['root_joint','lift','jaw_L','jaw_R'],
            robot_q=[.1,.4,.02,.03],robot_qvel=[.02,.11,-.12,.13],
            plant_dof_names=['Pedicel_01:0'],plant_q=[.21],plant_qvel=[.32],preload=[.43],
            robot_position_targets=[.10001,.408,.021,.031],robot_velocity_targets=[.001,.002,.003,.004],
            breaks=[dict(t=1.5,joint=self.joint,phase='close')],manifest=self.manifest,sequence=sequence)
        metadata=save_checkpoint(cp,**args)
        folder=trial/'cycles/001'; folder.mkdir(parents=True)
        rejected=dict(index=1,status='planning_inconclusive',start_s=2.,measured_frame_index=2,
            gt_group_heading_search=dict(checkpoint=str(cp),boundary_time_s=2.,native_state_reset=False),
            environment_preflight=dict(checkpoint=str(cp)))
        final_sequence=copy.deepcopy(sequence)
        final_sequence.pop('checkpoint_capture_reason');final_sequence.pop('checkpoint_physics_valid')
        final_sequence['termination']='gt_group_planning_inconclusive'
        final_sequence['cycle_plans'].append(rejected)
        final_args=copy.deepcopy(args);final_args['sequence']=final_sequence
        final_args.update(final_updates or {})
        save_checkpoint(trial/'route_checkpoint',**final_args)
        (trial/'truss_sequence.json').write_text(json.dumps(final_args['sequence']))
        (trial/'break_events.json').write_text(json.dumps(args['breaks']))
        trace=dict(time=np.array([0.,1.,2.]),body_paths=np.array(args['body_paths']),
            poses=np.repeat(np.array(args['body_poses_world'],np.float32)[None],3,axis=0),
            robot_q=np.repeat(np.array(args['robot_q'],np.float32)[None],3,axis=0),
            command=np.repeat(np.array(args['robot_position_targets'],np.float32)[None],3,axis=0),
            phase=np.array(['settle','close','hold']),cycle_index=np.array([0,0,0]))
        if trace_change: trace_change(trace)
        np.savez_compressed(trial/'states.npz',**trace)
        summary=dict(completed=False,stop_reason='gt_group_planning_inconclusive',initialization_valid=True,
            actual_self_collision_audit_passed=True,simulated_s=2.,physics_steps=2,model_sha256=self.sha)
        summary.update(summary_updates or {})
        (trial/'summary.json').write_text(json.dumps(summary))
        (trial/'self_collision_actual.json').write_text(json.dumps(dict(passed=True,complete=True,
            scene_sha256=self.sha,input_sha256=file_sha256(trial/'states.npz'),physics_rerun=False)))
        (folder/'plan.json').write_text(json.dumps(dict(phases=[dict(name='prepare_joint_branch')])))
        hit=dict(robot_geom='/World/ActuatedTool/L/part_214',environment_geom='/World/Gutter/Wire',
            obstacle_class='gutter',phase='prepare_joint_branch',time_s=1.)
        audit=dict(schema='s300-rail-environment-preflight-v1',status='blocked',passed=False,complete=False,
            stop_on_first_hard_hit=True,early_terminated=True,termination_reason='first_recorded_hard_intersection',
            eligible_for_native_execution=False,native_runs=0,physics_unchanged=True,samples=61,
            hard_intersection_sample_pairs=1,hard_hits=[hit],first_hard_hit=hit,last_sample_time_s=1.,
            unexamined_suffix=dict(status='UNKNOWN',from_time_s=1.,from_phase=hit['phase'],
                includes_remaining_pairs_at_first_blocked_sample=True,other_collisions_absent=None,nominal_path_end_s=3.),
            source_sha256=dict(scene=self.sha,manifest=file_sha256(self.assembly/'motion_manifest.json'),
                materialized_dependency=None,plan=file_sha256(folder/'plan.json')),
            checkpoint_proof=dict(checkpoint=str(cp),metadata_sha256=metadata['metadata_sha256'],
                checkpoint_json_sha256=file_sha256(cp/'checkpoint.json'),npz_sha256=metadata['checkpoint_npz_sha256'],
                source_materialized_dependency_sha256=None,current_materialized_dependency_sha256=None,
                native_break_events=imported_break_markers(metadata)))
        (folder/'full_environment_preflight.json').write_text(json.dumps(audit))
        (folder/'runtime_error.json').write_text(json.dumps(dict(error='Traceback (most recent call last):\nRuntimeError: Rail environment preflight blocked commanded travel\n')))
        return trial,cp

    def validate(self,cp,**kwargs):
        return validated_initial_checkpoint(self.assembly,self.manifest,cp,**kwargs)

    def test_default_rejects_optin_accepts_only_exact_terminal_boundary_without_edits(self):
        trial,cp=self.fixture();files=list(trial.rglob('*'));before={p:p.read_bytes()for p in files if p.is_file()}
        with self.assertRaisesRegex(ValueError,'parent stopped'):self.validate(cp)
        _,proof=self.validate(cp,allow_planning_boundary_resume=True)
        p=proof['planning_boundary_resume'];self.assertTrue(p['clean_physical_boundary'])
        self.assertTrue(p['resuming_without_rollback']);self.assertFalse(p['parent_motion_completed'])
        self.assertEqual(p['original_stop_reason'],'gt_group_planning_inconclusive')
        self.assertEqual(p['planning_only_suffix'][0]['candidate_physics_started'],False)
        self.assertIsNone(p['planning_only_suffix'][0]['evidence']['early_blocked_environment_witness']['other_collisions_absent'])
        self.assertEqual(before,{p:p.read_bytes()for p in before})

    def test_earlier_time_or_extra_actual_frame_cannot_roll_back(self):
        for i,changes in enumerate([dict(final_updates=dict(time_s=2.1)),dict(summary_updates=dict(simulated_s=2.1)),
                dict(summary_updates=dict(physics_steps=3)),dict(trace_change=lambda t:t['time'].__setitem__(-1,2.1))]):
            with self.subTest(changes=i):
                _,cp=self.fixture(f'case{i}',**changes)
                with self.assertRaises(ValueError):self.validate(cp,allow_planning_boundary_resume=True)

    def test_each_measured_array_and_controller_or_break_change_fails_even_with_valid_hashes(self):
        changes=[dict(robot_q=[.11,.4,.02,.03]),dict(robot_qvel=[.03,.11,-.12,.13]),
            dict(plant_q=[.22]),dict(plant_qvel=[.33]),dict(preload=[.44]),
            dict(body_velocities_world=np.ones((3,6))),
            dict(body_poses_world=[[0,0,0,0,0,0,1],[1.01,2,3,0,0,0,1],[0,2,.5,0,0,0,1]]),
            dict(robot_position_targets=[.10002,.408,.021,.031]),
            dict(robot_velocity_targets=[.002,.002,.003,.004]),
            dict(breaks=[dict(t=1.5,joint=self.joint,phase='cycle_open')])]
        for i,change in enumerate(changes):
            with self.subTest(change=list(change)):
                _,cp=self.fixture(f'changed{i}',final_updates=change)
                with self.assertRaises(ValueError):self.validate(cp,allow_planning_boundary_resume=True)

    def test_physics_initialization_or_actual_audit_failure_never_approved(self):
        for i,change in enumerate([dict(stop_reason='penetration_over_1mm'),dict(stop_reason='entry_open_not_reached'),
                dict(initialization_valid=False),dict(actual_self_collision_audit_passed=False)]):
            with self.subTest(change=change):
                _,cp=self.fixture(f'stop{i}',summary_updates=change)
                with self.assertRaises(ValueError):self.validate(cp,allow_planning_boundary_resume=True)

    def test_actual_trace_pose_q_control_and_hash_mismatches_fail(self):
        for i,key in enumerate(['poses','robot_q','command']):
            def change(t):t[key][-1].flat[0]+=.01
            trial,cp=self.fixture(f'trace{i}',trace_change=change)
            with self.subTest(key=key),self.assertRaises(ValueError):self.validate(cp,allow_planning_boundary_resume=True)
        trial,cp=self.fixture('hash');audit=trial/'self_collision_actual.json';x=json.loads(audit.read_text());x['input_sha256']='0'*64;audit.write_text(json.dumps(x))
        with self.assertRaisesRegex(ValueError,'audit/model/stored-trace'):self.validate(cp,allow_planning_boundary_resume=True)

    def test_missing_unknown_native_candidate_or_wrong_checkpoint_witness_fails(self):
        for i,kind in enumerate(['missing','unknown','native','checkpoint']):
            trial,cp=self.fixture(f'witness{i}');folder=trial/'cycles/001'
            if kind=='missing':(folder/'full_environment_preflight.json').unlink()
            elif kind=='unknown':(folder/'runtime_error.json').write_text(json.dumps(dict(error='Unknown integration failure')))
            elif kind=='native':(folder/'states.npz').write_bytes(b'candidate physics started')
            else:
                p=folder/'full_environment_preflight.json';x=json.loads(p.read_text());x['checkpoint_proof']['metadata_sha256']='0'*64;p.write_text(json.dumps(x))
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.validate(cp,allow_planning_boundary_resume=True)

    def test_changed_completed_ledger_and_executed_suffix_are_rejected(self):
        for i,field in enumerate(['status','attempt_counts']):
            trial,cp=self.fixture(f'ledger{i}');final=trial/'route_checkpoint/checkpoint.json'
            # Re-save a fully valid final snapshot with changed semantic state.
            m,a=load_checkpoint(trial/'route_checkpoint');sequence=copy.deepcopy(m['sequence'])
            if field=='status':sequence['cycle_plans'][-1]['status']='executing'
            else:sequence['attempt_counts']['Truss_00_Fruit_01']=2
            # File integrity itself is also part of the required guard.
            p=trial/'truss_sequence.json';p.write_text(json.dumps(sequence))
            with self.subTest(field=field),self.assertRaises(ValueError):self.validate(cp,allow_planning_boundary_resume=True)

    def test_cli_default_has_no_new_campaign_kwarg_and_optin_requires_checkpoint(self):
        import run_row_harvest as row
        arguments=['run_row_harvest.py','/assembly','/inventory','/screen','/output']
        with patch('sys.argv',arguments),patch('run_row_harvest.campaign',return_value=dict(status='planned_only'))as call:
            self.assertEqual(row.main(),0);self.assertNotIn('allow_planning_boundary_resume',call.call_args.kwargs)
        with patch('sys.argv',arguments+['--allow-planning-boundary-resume','--initial-native-checkpoint','/cp']),patch('run_row_harvest.campaign',return_value=dict(status='planned_only'))as call:
            self.assertEqual(row.main(),0);self.assertTrue(call.call_args.kwargs['allow_planning_boundary_resume'])
        with self.assertRaisesRegex(ValueError,'explicit exact initial'):
            row.campaign('/a','/i','/s','/o',allow_planning_boundary_resume=True)


if __name__=='__main__':unittest.main()
