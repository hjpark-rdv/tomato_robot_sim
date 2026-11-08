"""Exact terminal-state, original-clock and suffix-only audit regressions."""
import copy
import ast
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from contextlib import redirect_stderr
import io

import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics

from compact_motion_assets import file_sha256
from native_route_checkpoint import save_checkpoint,load_checkpoint
from terminal_motion_recovery import (validate_terminal_forward_suffix,recovery_plan,validate_suffix_plan_binding,
    _source_lift_reference,require_same_recovery_settings)
from motion_continuation import (source_command_sample,source_cycle_elapsed,execution_duration_s,
    terminal_suffix_samples,attach_resume_rail_hold)
from motion_self_collision import check_plan


class TerminalForwardSuffixTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.assembly=self.root/'assembly';self.assembly.mkdir();self.scene=self.assembly/'scene.usda'
        stage=Usd.Stage.CreateNew(str(self.scene));self.fruit='/World/ElasticTarget/Truss_00_Fruit_04'
        self.joint='/World/ElasticTarget/HarvestJoints/Truss_00_Fruit_04'
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,self.fruit).GetPrim())
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Sphere.Define(stage,self.fruit+'/Collider').GetPrim())
        UsdPhysics.FixedJoint.Define(stage,self.joint)
        self.support='/World/Greenhouse/House/Environment/RowSupply_20mm/Mesh'
        mesh=UsdGeom.Mesh.Define(stage,self.support)
        mesh.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0)]);mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True);stage.GetRootLayer().Save()
        self.sha=file_sha256(self.scene)
        self.manifest=dict(scene_sha256=self.sha,plant=dict(root='/World/ElasticTarget',source_plant='/World/Source/Plant_25',
            fruits=[dict(name='Truss_00_Fruit_04',path=self.fruit,joint=self.joint)]))
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        self.trial=self.root/'trial';self.folder=self.trial/'cycles/002';self.folder.mkdir(parents=True)
        q=np.repeat(np.arange(7,dtype=float)[None]/10,2,axis=0).tolist()
        self.plan=dict(start=q[0],phases=[dict(name=n,duration_s=d,waypoint_q=q,end_gap_m=g,belt_local_y_m_s=b,
            rail_waypoint_positions_m=[.2,.2],rail_native_command_positions_m=[.20001,.20001])for n,d,g,b in
            [('ready_hold',.5,.044,[0.,0.]),('close',.5,.028,[-.05,.05]),('cycle_open',1.,.096,[-.05,.05]),
             ('cycle_reclose',1.,.028,[-.05,.05]),('cycle_open_final',1.,.096,[-.05,.05]),('hold',1.,.044,[0.,0.])]])
        self.plan['resume_rail_origin']=dict(schema='s300-measured-resume-rail-origin-v1',axis_world=[0.,1.,0.],displacement_m=.2)
        self.plan_path=self.folder/'plan.json';self.plan_path.write_text(json.dumps(self.plan))
        self.tick=142;self.time=self.tick/60;self.start=1.
        sample=source_command_sample(self.plan,(self.tick-1)/60-self.start)
        self.names=['root_joint','farmily_lift_height_joint','base','shoulder','elbow','wrist1','wrist2','wrist3','jaw_L','jaw_R']
        self.command=np.r_[.20001,sample['q'],-(sample['gap_m']-.044)/2,(sample['gap_m']-.044)/2].astype(np.float32)
        self.q=self.command.copy();self.q[2]+=.0001;self.q[-2:]=[-.0007,.0013]
        self.breaks=[dict(t=2.1,phase='close',joint=self.joint)]
        self.sequence=dict(termination='penetration_over_contact_policy',cycle_plans=[dict(index=2,status='stopped',
            stop_reason='penetration_over_contact_policy',plan='cycles/002/plan.json',start_s=self.start,end_s=self.time,
            planned_end_s=6.,end_frame_index=self.tick,fruits=['Truss_00_Fruit_04'],truss_key='Truss_00')])
        self.runtime=dict(hz=60,contact_limit_mm=1.,experimental_tpu_fruit_close_2mm=False,solver='TGS',device='cpu',threads=1,
            position_iterations=64,velocity_iterations=4,fruit_contact_limit_mm=1.,fruit_fruit_limit_mm=1.,
            plant_dynamics='full',collision_filter='pairs',isolate_background=False,contact_backend='callback')
        self.summary=dict(completed=False,stop_reason='penetration_over_contact_policy',initialization_valid=True,
            actual_self_collision_audit_passed=True,model_sha256=self.sha,contact_limit_mm=1.,simulated_s=self.time,
            physics_steps=self.tick,native_break_events=self.breaks,leaf_poststep_verification=True,belt_off_control=False,
            contact_lifecycle_scope=dict(enabled=True,gripper_support_enabled=True,same_truss_enabled=True,same_plant_enabled=True,
                static_gutter_base_enabled=True,static_farm_support_enabled=True))
        self.cp=self.trial/'route_checkpoint'
        save_checkpoint(self.cp,assembly=self.assembly,scene_sha256=self.sha,time_s=self.time,
            body_paths=[self.fruit],body_poses_world=[[1,2,3,0,0,0,1]],body_velocities_world=[[.1,.2,.3,.4,.5,.6]],
            robot_dof_names=self.names,robot_q=self.q,robot_qvel=np.arange(10)/100,plant_dof_names=['stem_0'],
            plant_q=[.1],plant_qvel=[.2],preload=[.3],breaks=self.breaks,manifest=self.manifest,sequence=self.sequence,
            robot_position_targets=self.command,robot_velocity_targets=np.zeros(10))
        self.meta,self.arrays=load_checkpoint(self.cp)
        trace=dict(time=np.arange(self.tick+1)/60,body_paths=np.array([self.fruit]),
            robot_q=np.repeat(self.q[None],self.tick+1,axis=0),command=np.repeat(self.command[None],self.tick+1,axis=0),
            poses=np.repeat(np.array([[[1,2,3,0,0,0,1]]],np.float32),self.tick+1,axis=0),
            phase=np.repeat('cycle_open',self.tick+1),belt_surface_velocity_local_y_m_s=np.repeat(np.array([[-.05,.05]]),self.tick+1,axis=0))
        np.savez_compressed(self.trial/'states.npz',**trace)
        self.write('summary.json',self.summary);self.write('runtime_settings.json',self.runtime)
        self.write('truss_sequence.json',self.sequence);self.write('break_events.json',self.breaks)
        self.write('contacts.json',[dict(t=self.time,phase='cycle_open',a=self.fruit+'/Collider',b=self.support,separation_m=-.0017,force_n=0.)])
        self.write('solved_contact_policy.json',dict(verify_current_truss_fruit_contacts=True,verify_background_component_hulls=True,verify_structural_scene_queries=True))
        self.write('self_collision_actual.json',dict(passed=True,complete=True,scene_sha256=self.sha,
            input_sha256=file_sha256(self.trial/'states.npz'),physics_rerun=False))
        self.write('cycles/002/self_collision_preflight.json',dict(passed=True,complete=True,hz=60,input_sha256=file_sha256(self.plan_path)))
        self.write('cycles/002/full_environment_preflight.json',dict(passed=True,complete=True,native_hz=60,
            source_sha256=dict(plan=file_sha256(self.plan_path))))

    def tearDown(self):self.tmp.cleanup()
    def write(self,name,value):(self.trial/name).write_text(json.dumps(value))
    def validate(self,**kwargs):return validate_terminal_forward_suffix(self.assembly,self.manifest,self.cp,self.meta,self.arrays,**kwargs)

    def test_exact_terminal_proof_preserves_original_stop_state_and_source_commands(self):
        paths=[p for p in self.root.rglob('*')if p.is_file()];before={str(p):file_sha256(p)for p in paths}
        proof=self.validate();phases,plan=recovery_plan(proof);validate_suffix_plan_binding(plan)
        self.assertEqual(plan['phases'],self.plan['phases']);self.assertEqual(len(phases),6)
        self.assertFalse(proof['original_completed']);self.assertTrue(proof['original_stop_preserved'])
        self.assertTrue(proof['resuming_without_rollback']);self.assertFalse(proof['harvest_attempt'])
        self.assertNotEqual(plan['start'],self.plan['start']);self.assertEqual(plan['start'],self.q[1:8].tolist())
        self.assertEqual(before,{str(p):file_sha256(p)for p in paths})

    def test_source_clock_exact_operation_order_last_command_and_next_tick(self):
        _,plan=recovery_plan(self.validate());samples=terminal_suffix_samples(plan,60)
        for i,sample in enumerate(samples):
            original=source_command_sample(self.plan,(self.tick+i)/60-self.start)
            self.assertEqual(source_cycle_elapsed(plan,i/60),(self.tick+i)/60-self.start)
            self.assertEqual(sample['gap_m'],original['gap_m']);np.testing.assert_array_equal(sample['q'],original['q'])
            self.assertEqual(sample['belt_local_y_m_s'],original['belt_local_y_m_s'])
        self.assertEqual(samples[0]['phase'],'cycle_open');self.assertAlmostEqual(samples[0]['fraction'],.3666666666666667)
        self.assertAlmostEqual(execution_duration_s(plan),6.-self.time)
        self.assertEqual(source_cycle_elapsed(self.plan,.2),.2);self.assertEqual(execution_duration_s(self.plan),5.)
        last=source_command_sample(self.plan,(self.tick-1)/60-self.start)
        self.assertNotEqual(last['gap_m'],samples[0]['gap_m'])
        # Truncating the open phase and restarting its smoothstep would differ.
        restarted=.028+(.096-.028)*0.
        self.assertNotEqual(samples[0]['gap_m'],restarted)

    def test_original_lift_reference_is_getter_hash_bound_and_never_rebuilt_from_terminal_load(self):
        from lift_drive_reference import build_lift_drive_reference,apply_lift_drive_reference
        from robot_controller_state import resolve_robot_controller_state
        source=self.root/'original_source_checkpoint';source_q=self.arrays['robot_q'].copy();source_q[1]=.03
        source_targets=self.command.copy();source_targets[1]=.036256
        save_checkpoint(source,assembly=self.assembly,scene_sha256=self.sha,time_s=2.2,
            body_paths=self.arrays['body_paths'],body_poses_world=self.arrays['body_poses_world'],
            body_velocities_world=self.arrays['body_velocities_world'],robot_dof_names=self.names,
            robot_q=source_q,robot_qvel=self.arrays['robot_qvel'],plant_dof_names=self.arrays['plant_dof_names'],
            plant_q=self.arrays['plant_q'],plant_qvel=self.arrays['plant_qvel'],preload=self.arrays['preload'],
            breaks=self.breaks,manifest=self.manifest,sequence=dict(self.sequence,checkpoint_physics_valid=True),
            robot_position_targets=source_targets,robot_velocity_targets=np.zeros(10))
        metadata,arrays=load_checkpoint(source);controller=resolve_robot_controller_state(metadata,arrays,source)
        ref=build_lift_drive_reference(metadata,arrays,controller,enabled=True,camera_harvest=True,lift_bounds_m=[0.,.75])
        self.write('lift_drive_reference.json',ref);self.write('resume_input.json',dict(
            checkpoint=str(source),checkpoint_metadata_sha256=metadata['metadata_sha256']))
        summary=dict(self.summary,lift_drive_reference=ref);actual,binding=_source_lift_reference(self.trial,summary)
        self.assertEqual(actual,ref);self.assertEqual(binding['sha256'],file_sha256(self.trial/'lift_drive_reference.json'))
        # Execute the actual runner's ORIGINAL-reference selection, with a
        # deliberately different terminal height. No SDK or actual setter runs.
        tree=ast.parse(Path(__file__).with_name('run_motion.py').read_text())
        branch=next(n for n in ast.walk(tree)if isinstance(n,ast.If)and
            ast.unparse(n.test)=='a.resume_original_cycle_suffix' and any(
                isinstance(c,ast.Assign)and any(isinstance(x,ast.Name)and x.id=='lift_drive_reference'for x in c.targets)
                for c in n.body))
        env=dict(a=SimpleNamespace(resume_original_cycle_suffix=True),np=np,
            kin=SimpleNamespace(bounds=np.array([[0.]*7,[.75]+[4.]*6])),
            context=dict(terminal_recovery_proof=dict(source_lift_drive_reference=actual)))
        exec(compile(ast.Module(body=[branch],type_ignores=[]),'production_original_lift_reference','exec'),env)
        self.assertEqual(env['lift_drive_reference'],ref)
        command,_=apply_lift_drive_reference([.04],env['lift_drive_reference'],
            dof_names=['farmily_lift_height_joint'],bounds=np.array([[0.],[.75]]))
        self.assertEqual(command[0],.04+ref['retained_target_minus_measured_m'])
        proof=dict(schema='s300-terminal-forward-suffix-v1',source_runtime=self.runtime,
            source_lift_drive_reference=ref,source_lift_drive_reference_binding=binding)
        settings=dict(self.runtime,resume_original_cycle_suffix=True,retain_lift_drive_reference=True)
        require_same_recovery_settings(proof,settings)
        for changes in (dict(resume_original_cycle_suffix=False),dict(retain_lift_drive_reference=False)):
            with self.assertRaises(ValueError):require_same_recovery_settings(proof,dict(settings,**changes))
        bad=copy.deepcopy(ref);bad['retained_target_minus_measured_m']+=1e-6
        self.write('lift_drive_reference.json',bad)
        with self.assertRaises(ValueError):_source_lift_reference(self.trial,dict(summary,lift_drive_reference=bad))
        with self.assertRaises(ValueError):require_same_recovery_settings(proof,settings)

    def test_float64_control_log_requires_archived_exact_native_float32_conversion(self):
        from terminal_motion_recovery import _source_native_command_dtype
        logged=np.array([.006330691299808677],dtype=np.float64)
        native=np.array([.0063306912779808044],dtype=np.float64)
        self.assertFalse(np.array_equal(logged,native))
        directory=self.trial/'code';directory.mkdir()
        source=directory/'run_motion.py'
        source.write_text('def tensor(x,dtype=torch.float32):\n return torch.as_tensor(x,dtype=dtype)\ntarget_tensor.copy_(tensor(qcommand))\n')
        dtype,proof=_source_native_command_dtype(self.trial,logged)
        self.assertEqual(dtype,np.dtype('float32'));self.assertFalse(proof['tolerance_used'])
        np.testing.assert_array_equal(logged.astype(dtype),native.astype(dtype))
        altered=logged.copy();altered[0]+=1e-8
        self.assertFalse(np.array_equal(altered.astype(dtype),native.astype(dtype)))
        source.write_text(source.read_text().replace('torch.float32','torch.float64'))
        with self.assertRaises(ValueError):_source_native_command_dtype(self.trial,logged)

    def test_native_cli_allows_lift_reference_only_for_explicit_original_camera_suffix(self):
        from test_run_motion_conditioning import actual_cli
        args=['--mode','recovery','--resume-original-cycle-suffix','--resume-native-checkpoint','/terminal',
              '--verify-full-environment-preflight','--native-tool-collision-cache','/cache',
              '--retain-lift-drive-reference','--horizontal-ready-connector','joint',
              '--preparation','horizontal_ready','--hz','60','--released-fruit-pair-diagnostic','--save-route-checkpoint']
        parsed=actual_cli(args)
        self.assertTrue(parsed.resume_original_cycle_suffix);self.assertTrue(parsed.retain_lift_drive_reference)
        self.assertTrue(parsed.released_fruit_pair_diagnostic);self.assertFalse(parsed.camera_harvest)
        with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            actual_cli([x for x in args if x!='--resume-original-cycle-suffix'])

    def test_original_native_rail_hold_target_retained_geometry_uses_actual_terminal(self):
        phases,plan=recovery_plan(self.validate());origin=dict(displacement_m=.200002,native_position_target_m=float(self.command[0]))
        # The source command is serialized double; make the matching independent
        # drive getter exact here, as it is in the actual native saved source.
        origin['native_position_target_m']=.20001
        _,held=attach_resume_rail_hold(phases,plan,origin,preserve_existing_hold=True)
        self.assertEqual(held['phases'][0]['rail_native_command_positions_m'],[.20001,.20001])
        self.assertEqual(held['phases'][0]['rail_waypoint_positions_m'],[.200002,.200002]);validate_suffix_plan_binding(held)
        with self.assertRaises(ValueError):attach_resume_rail_hold(phases,plan,dict(displacement_m=.2,native_position_target_m=.21),preserve_existing_hold=True)
        with self.assertRaises(ValueError):attach_resume_rail_hold(phases,plan,origin)

    def test_suffix_checker_never_replays_prefix_and_checks_original_curve(self):
        _,plan=recovery_plan(self.validate())
        class Recorder:
            metadata={}
            def __init__(self):self.values=[]
            def check(self,q,gap,*,rail_position_m):self.values.append((q.copy(),gap,rail_position_m));return[]
        recorder=Recorder();report=check_plan(recorder,plan,hz=60)
        self.assertTrue(report['passed']);self.assertTrue(report['complete']);self.assertEqual(report['phases'][0]['phase'],'cycle_open')
        expected=terminal_suffix_samples(plan,60);self.assertEqual(len(recorder.values),len(expected))
        for actual,wanted in zip(recorder.values,expected):np.testing.assert_array_equal(actual[0],wanted['q']);self.assertEqual(actual[1],wanted['gap_m'])

    def test_changed_original_commands_clock_or_plan_certificate_rejected(self):
        _,plan=recovery_plan(self.validate())
        for field,value in [('duration_s',.9),('end_gap_m',.095),('belt_local_y_m_s',[0.,0.]),('waypoint_q',[[0.]*7]*2)]:
            changed=copy.deepcopy(plan);changed['phases'][2][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):validate_suffix_plan_binding(changed)
        changed=copy.deepcopy(plan);changed['terminal_forward_suffix']['initial_source_elapsed_s']-=1/60
        with self.assertRaises(ValueError):validate_suffix_plan_binding(changed)
        self.write('cycles/002/self_collision_preflight.json',dict(passed=True,complete=True,hz=60,input_sha256='0'*64))
        with self.assertRaises(ValueError):self.validate()

    def test_unreleased_different_pair_or_source_missing_policy_never_authorized(self):
        for change in [dict(t=2.),dict(b=self.fruit+'/Collider'),dict(b=self.support.replace('RowSupply','OtherSupport'))]:
            contact=dict(t=self.time,phase='cycle_open',a=self.fruit+'/Collider',b=self.support,separation_m=-.0017);contact.update(change)
            self.write('contacts.json',[contact])
            with self.subTest(change=change),self.assertRaises(ValueError):self.validate()
        self.summary['contact_lifecycle_scope']['static_farm_support_enabled']=False;self.write('summary.json',self.summary)
        with self.assertRaises(ValueError):self.validate()

    def test_earlier_checkpoint_actual_tip_array_controller_and_original_events_rejected(self):
        with self.assertRaisesRegex(ValueError,'latest terminal'):
            validate_terminal_forward_suffix(self.assembly,self.manifest,self.trial/'cycle_checkpoints/000',self.meta,self.arrays)
        for key in ['robot_q','robot_qvel','plant_q','plant_qvel','preload','body_velocities_world']:
            arrays=copy.deepcopy(self.arrays);arrays[key].flat[0]+=.01
            with self.subTest(key=key),self.assertRaises(ValueError):
                validate_terminal_forward_suffix(self.assembly,self.manifest,self.cp,self.meta,arrays)
        self.write('break_events.json',[])
        with self.assertRaises(ValueError):self.validate()

    def test_actual_audit_initialization_general_physics_and_terminal_frame_mismatch_fail(self):
        original=copy.deepcopy(self.summary)
        for field,value in [('stop_reason','nonfinite_state'),('initialization_valid',False),('actual_self_collision_audit_passed',False),
                ('simulated_s',self.time+1/60),('physics_steps',self.tick+1)]:
            self.summary=copy.deepcopy(original);self.summary[field]=value;self.write('summary.json',self.summary)
            with self.subTest(field=field),self.assertRaises(ValueError):self.validate()


if __name__=='__main__':unittest.main()
