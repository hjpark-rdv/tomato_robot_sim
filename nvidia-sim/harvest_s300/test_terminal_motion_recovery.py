import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from contextlib import redirect_stderr
import io

import numpy as np

from terminal_motion_recovery import validate_terminal_recovery,recovery_plan,require_same_recovery_settings


class TerminalMotionRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.assembly=self.root/'assembly';self.assembly.mkdir();(self.assembly/'scene.usda').write_text('# source model')
        self.digest=hashlib.sha256((self.assembly/'scene.usda').read_bytes()).hexdigest()
        self.manifest=dict(scene_sha256=self.digest)
        self.trial=self.root/'trial';self.trial.mkdir();self.checkpoint=self.trial/'route_checkpoint';self.checkpoint.mkdir()
        self.names=['root_joint','farmily_lift_height_joint','base','shoulder','elbow','wrist1','wrist2','wrist3','jaw_L','jaw_R']
        self.qs=np.zeros((5,10));self.qs[:,2]=[.1,.2,.3,.4,.5];self.qs[:,-2:]=[-.026,.026];self.qs[-1,-2]=-.017
        self.poses=np.zeros((5,1,7));self.poses[:,:,6]=1
        self.commands=self.qs.copy();self.commands[:,-2:]=[-.026,.026]
        self.phases=np.array(['initial','prepare_open_hold','prepare_joint_branch','prepare_joint_branch','prepare_joint_branch'])
        self.state=dict(robot_dof_names=self.names,robot_q=self.qs[-1].copy(),body_paths=['/World/Body'],body_poses_world=self.poses[-1].copy())
        self.meta=dict(source_assembly=str(self.assembly),manifest=self.manifest,scene_sha256=self.digest,time_s=4.,metadata_sha256='b'*64,
            controller_state=dict(robot_dof_names=self.names,position_source='native_get_dof_position_targets',position_targets=self.commands[-1].tolist()),
            sequence=dict(attempt_counts={'Truss_00_Fruit_01':1},cycle_plans=[dict(status='stopped',stop_reason='entry_open_not_reached',start_s=1.,end_s=4.,
                preparation_open_feedback=dict(approach=dict(reached=False)))]))
        self.runtime=dict(hz=60,contact_limit_mm=1.,solver='PGS',device='cpu',threads=16,position_iterations=64,velocity_iterations=4,
            fruit_contact_limit_mm=1.,fruit_fruit_limit_mm=1.,plant_dynamics='full',collision_filter='pairs',isolate_background=False,contact_backend='callback')
        self.summary=dict(completed=False,stop_reason='entry_open_not_reached',initialization_valid=True,actual_self_collision_audit_passed=True,
            model_sha256=self.digest,contact_limit_mm=1.,simulated_s=4.)
        self.write_sources()

    def tearDown(self):self.temp.cleanup()

    def write_sources(self):
        np.savez(self.trial/'states.npz',time=np.arange(5,dtype=float),robot_q=self.qs,command=self.commands,poses=self.poses,
            phase=self.phases,body_paths=np.array(['/World/Body']))
        for name,data in [('summary.json',self.summary),('runtime_settings.json',self.runtime),('self_collision_actual.json',
            dict(passed=True,complete=True,scene_sha256=self.digest,input_sha256=hashlib.sha256((self.trial/'states.npz').read_bytes()).hexdigest(),physics_rerun=False))]:
            (self.trial/name).write_text(json.dumps(data))

    def validate(self):return validate_terminal_recovery(self.assembly,self.manifest,self.checkpoint,self.meta,self.state)

    def camera_fixture(self):
        self.phases=np.array(['camera_pan','prepare_open_hold','prepare_camera_ompl_000',
            'prepare_camera_ompl_001','prepare_camera_ompl_001'])
        self.state['robot_qvel']=np.arange(10,dtype=float)*.001
        record=self.meta['sequence']['cycle_plans'][-1]
        record['plan']='cycles/002/plan.json';folder=self.trial/'cycles/002';folder.mkdir(parents=True)
        names=['prepare_camera_ompl_'+f'{i:03d}'for i in range(3)]
        plan=dict(phases=[dict(name=name,belt_local_y_m_s=[0.,0.],end_gap_m=.096)for name in names],
            preparation_opening=dict(required_gap_m=.096,tracking_tolerance_m=.0005,feedback_before_phases=names+['approach']))
        (folder/'plan.json').write_text(json.dumps(plan));reports=folder/'prefix_connection';reports.mkdir()
        report=reports/'report.json';report.write_text(json.dumps(dict(verified_plan_sha256=hashlib.sha256((folder/'plan.json').read_bytes()).hexdigest(),
            status='full_audits_passed',eligible_for_native_execution=True,full_hz=60,
            full_self=dict(passed=True,complete=True),full_environment=dict(passed=True,complete=True))))
        record['camera_prefix_connection']=dict(report=str(report))
        record['preparation_open_feedback']={names[0]:dict(reached=True,t=2.),
            names[1]:dict(reached=True,t=3.),names[2]:dict(reached=False,t=4.)}
        self.write_sources()
        return folder,record

    def test_exact_terminal_reverse_preserves_endpoints_jaw_commands_and_no_harvest_credit(self):
        before=copy.deepcopy(self.meta);proof=self.validate();phases,plan=recovery_plan(proof)
        reverse=next(row for row in plan['phases']if row['name']=='recovery_reverse')
        self.assertEqual(reverse['waypoint_q'],self.qs[1:,1:8][::-1].tolist())
        self.assertEqual(plan['start'],self.state['robot_q'][1:8].tolist())
        self.assertEqual(proof['retained_jaw_position_targets_m'],[-.026,.026])
        self.assertNotEqual(proof['measured_terminal_jaws_m'],[-.026,.026])
        self.assertTrue(all(row['end_gap_m']==.096 and row['belt_local_y_m_s']==[0.,0.]for row in plan['phases']))
        self.assertFalse(plan['harvest_attempt']);self.assertFalse(plan['training_eligible']);self.assertIsNone(plan['harvest_success'])
        self.assertEqual(self.meta,before)
        self.assertEqual(proof['previous_sequence']['attempt_counts'],{'Truss_00_Fruit_01':1})
        require_same_recovery_settings(proof,self.runtime)

    def test_earlier_good_cycle_cannot_replace_terminal(self):
        with self.assertRaisesRegex(ValueError,'terminal route_checkpoint'):
            validate_terminal_recovery(self.assembly,self.manifest,self.trial/'cycle_checkpoints/000',self.meta,self.state)
        self.meta['time_s']=3.
        with self.assertRaisesRegex(ValueError,'latest exact terminal'):
            self.validate()

    def test_camera_recovery_reverses_all_actual_connected_edges_but_not_completed_scan(self):
        folder,record=self.camera_fixture();before=copy.deepcopy(self.meta)
        actual_qvel=self.state['robot_qvel'].copy();proof=self.validate();phases,plan=recovery_plan(proof)
        self.assertEqual(proof['source_preparation_kind'],'camera_ompl_prefix')
        self.assertEqual(proof['source_trace_first_index'],1)
        self.assertEqual(proof['camera_prefix_proof']['actual_edges'],['prepare_camera_ompl_000','prepare_camera_ompl_001'])
        self.assertEqual(proof['camera_prefix_proof']['failed_next_phase'],'prepare_camera_ompl_002')
        self.assertFalse(proof['camera_prefix_proof']['scan_or_manipulation_reversed'])
        reverse=next(p for p in plan['phases']if p['name']=='recovery_reverse')
        self.assertEqual(reverse['waypoint_q'],self.qs[1:,1:8][::-1].tolist())
        self.assertEqual(plan['start'],self.state['robot_q'][1:8].tolist())
        self.assertEqual(proof['camera_prefix_proof']['measured_terminal_robot_qvel'],actual_qvel.tolist())
        self.assertEqual(proof['retained_jaw_position_targets_m'],[-.026,.026])
        self.assertFalse(plan['harvest_attempt']);self.assertEqual(self.meta,before)
        np.testing.assert_array_equal(self.state['robot_qvel'],actual_qvel)

    def test_camera_recovery_rejects_skipped_interrupted_scan_or_wrong_guard_edges(self):
        for mutate in (
            lambda:self.phases.__setitem__(2,'prepare_camera_ompl_001'),
            lambda:self.phases.__setitem__(3,'camera_pan'),
            lambda:self.phases.__setitem__(-1,'approach'),
            lambda:self.meta['sequence']['cycle_plans'][-1]['preparation_open_feedback']['prepare_camera_ompl_002'].update(t=3.9),
            lambda:self.meta['sequence']['cycle_plans'][-1]['preparation_open_feedback']['prepare_camera_ompl_000'].update(reached=False)):
            with self.subTest(mutate=mutate):
                self.camera_fixture();mutate();self.write_sources()
                with self.assertRaises(ValueError):self.validate()
                # Each subcase uses its own immutable-looking test assets.
                import shutil
                shutil.rmtree(self.trial/'cycles')

    def test_camera_recovery_rejects_changed_plan_certificate_or_missing_native_qvel(self):
        folder,_=self.camera_fixture()
        (folder/'plan.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'plan SHA'):self.validate()
        import shutil
        shutil.rmtree(self.trial/'cycles');folder,_=self.camera_fixture()
        self.state.pop('robot_qvel')
        with self.assertRaisesRegex(ValueError,'measured native robot qvel'):self.validate()

    def test_changed_model_or_trace_rejected(self):
        self.meta['manifest']=dict(other=True)
        with self.assertRaisesRegex(ValueError,'assembly/manifest/SHA'):self.validate()
        self.meta['manifest']=self.manifest
        (self.trial/'states.npz').write_bytes(b'changed trace')
        with self.assertRaisesRegex(ValueError,'actual-state self audit'):self.validate()

    def test_physics_stop_or_incomplete_audit_never_authorized(self):
        for stop in ['penetration_over_contact_policy','unexpected_runtime_error']:
            self.summary['stop_reason']=stop;self.write_sources()
            with self.assertRaisesRegex(ValueError,'safe 60Hz/1mm'):self.validate()
        self.summary['stop_reason']='entry_open_not_reached';self.write_sources()
        audit=json.loads((self.trial/'self_collision_actual.json').read_text());audit['complete']=False
        (self.trial/'self_collision_actual.json').write_text(json.dumps(audit))
        with self.assertRaisesRegex(ValueError,'actual-state self audit'):self.validate()

    def test_pose_or_joint_terminal_mismatch_rejected(self):
        for key in ('robot_q','body_poses_world'):
            original=self.state[key].copy();self.state[key].flat[0]+=.00001
            with self.assertRaisesRegex(ValueError,'latest exact terminal'):self.validate()
            self.state[key]=original

    def test_approach_or_discontinuous_branch_cannot_be_reversed(self):
        self.phases[-1]='approach';self.write_sources()
        with self.assertRaisesRegex(ValueError,'last preparation branch'):self.validate()
        self.phases[-1]='prepare_joint_branch';self.phases[3]='other';self.write_sources()
        with self.assertRaisesRegex(ValueError,'contiguous terminal'):self.validate()

    def test_measured_or_symmetric_narrow_jaw_targets_not_substituted(self):
        for source,targets in [('measured_q',[-.026,.026]),('native_get_dof_position_targets',[-.0215,.0215])]:
            self.meta['controller_state']['position_source']=source
            self.meta['controller_state']['position_targets'][-2:]=targets
            with self.assertRaisesRegex(ValueError,'retained separate native'):self.validate()

    def test_settings_cannot_weaken_guards_or_change_motor_solver_workload(self):
        proof=self.validate()
        for key,value in [('hz',120),('contact_limit_mm',2.),('isolate_background',True),('solver','TGS')]:
            changed=dict(self.runtime,**{key:value})
            with self.assertRaisesRegex(ValueError,'preserve source physics/guard'):require_same_recovery_settings(proof,changed)

    def test_retained_lift_reference_source_is_hard_blocked_until_runner_binding_exists(self):
        self.runtime['retain_lift_drive_reference']=True;self.camera_fixture()
        proof=self.validate();_,plan=recovery_plan(proof)
        self.assertEqual(plan['start'],self.state['robot_q'][1:8].tolist())
        for flags in ({'retain_lift_drive_reference':True},{'retain_lift_drive_reference':False},
                      {'retain_lift_drive_reference':True,'recovery_lift_reference_binding_verified':True}):
            with self.assertRaisesRegex(ValueError,'controller binding pending'):
                require_same_recovery_settings(proof,dict(self.runtime,**flags))
        # Real runtime_settings uses a whitelist that predates this opt-in.
        # The native summary still records the actual source control policy.
        self.runtime.pop('retain_lift_drive_reference')
        self.summary['lift_drive_reference']=dict(schema='s300-lift-drive-reference-v1',
            retained_target_minus_measured_m=.008315298706293106)
        self.write_sources();proof=self.validate()
        self.assertTrue(proof['recovery_controller_binding_pending'])
        self.assertEqual(proof['source_lift_drive_reference'],self.summary['lift_drive_reference'])
        with self.assertRaisesRegex(ValueError,'controller binding pending'):
            require_same_recovery_settings(proof,self.runtime)

    def test_real_cli_requires_latest_resume_and_no_harvest_rail_override(self):
        from test_run_motion_conditioning import actual_cli
        flags=['--mode','recovery','--hz','60','--resume-native-checkpoint','/exact/route_checkpoint','--save-route-checkpoint']
        parsed=actual_cli(flags)
        self.assertEqual(parsed.mode,'recovery');self.assertFalse(parsed.whole_truss)
        self.assertEqual(parsed.contact_limit_mm,1.)
        for invalid in (flags+['--whole-truss'],flags+['--rail-to-y','2.'],flags+['--seconds','1'],
                        ['--mode','recovery','--hz','60','--save-route-checkpoint'],
                        ['--mode','recovery','--hz','60','--resume-native-checkpoint','/exact/route_checkpoint']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(invalid)

    def test_actual_runner_holds_separate_jaw_targets_without_touching_arm(self):
        import ast
        from types import SimpleNamespace
        from test_run_motion_conditioning import LOOP,code
        branch=next(node for node in LOOP.body if isinstance(node,ast.If)and
            any(isinstance(child,ast.Subscript)and isinstance(child.value,ast.Name)and child.value.id=='recovery_proof'
                for child in ast.walk(node)))
        commands=np.arange(10,dtype=float)[None];original=commands.copy()
        env=dict(a=SimpleNamespace(mode='recovery'),qcommand=commands,jawids=[8,9],
            recovery_proof=dict(retained_jaw_position_targets_m=[-.026,.026]),route_start_context=None,gap=.087)
        exec(code([branch]),env)
        np.testing.assert_array_equal(commands[0,:8],original[0,:8])
        np.testing.assert_array_equal(commands[0,-2:],[-.026,.026])

    def test_actual_recovery_sequence_is_distinct_and_has_zero_new_attempts(self):
        import ast
        from types import SimpleNamespace
        from test_run_motion_conditioning import MAIN,code
        branch=next(node for node in MAIN.body if isinstance(node,ast.If)and
            any(isinstance(child,ast.keyword)and child.arg=='recovery_source_checkpoint_metadata_sha256'
                for child in ast.walk(node)))
        fruit=dict(name='Truss_00_Fruit_01',joint='/actual/joint',path='/actual/fruit')
        previous=dict(fruit_names=[fruit['name']],fruit_joints={fruit['name']:fruit['joint']},initial_axis=dict(direction_world=[0,1,0]),
            attempt_counts={fruit['name']:1},cycle_plans=[dict(start_s=8.5,end_s=16.5,status='stopped')])
        env=dict(a=SimpleNamespace(mode='recovery'),recovery_proof=dict(previous_sequence=previous,checkpoint_metadata_sha256='abc'),
            manifest=dict(plant=dict(fruits=[fruit])),initial_attempt_counts={fruit['name']:1},total=11.5,resume_context=dict(report={'source':'terminal'}))
        exec(code([branch]),env)
        sequence=env['sequence'];record=sequence['cycle_plans'][0]
        self.assertEqual(record['fruits'],[]);self.assertFalse(record['harvest_attempt'])
        self.assertEqual(record['purpose'],'terminal_preparation_recovery');self.assertEqual(record['start_s'],0.)
        self.assertEqual(sequence['attempt_counts'],previous['attempt_counts'])
        self.assertEqual(sequence['fruit_joints'],previous['fruit_joints'])

    def test_actual_recovery_retains_required_actual_pose_audit(self):
        import ast
        from types import SimpleNamespace
        from test_run_motion_conditioning import MAIN
        branch=next(node for node in MAIN.body if isinstance(node,ast.If)and
            any(isinstance(child,ast.Constant)and child.value=='self_collision_actual.json'for child in ast.walk(node)))
        expression=ast.Expression(body=branch.test);ast.fix_missing_locations(expression)
        self.assertTrue(eval(compile(expression,'actual-runner-audit','eval'),dict(a=SimpleNamespace(mode='recovery',rail_to_y=None),foliage_conditioner=None)))


if __name__=='__main__':unittest.main()
