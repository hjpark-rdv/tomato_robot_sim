"""Real saved checkpoint/source/report binding for opt-in canonical idle starts."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics

from compact_motion_assets import file_sha256
from conditioned_start_checkpoint import (conditioned_checkpoint_sequence,
    write_conditioned_start_marker,require_conditioned_checkpoint)
from foliage_conditioning import FoliageConditioning,save_imported_source
from native_route_checkpoint import save_checkpoint,load_checkpoint
from resume_native_state import prepare_resume,require_completed_checkpoint
import test_foliage_conditioning as foliage_fixture


class ConditionedStartCheckpointTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.assembly=self.root/'assembly';self.assembly.mkdir()
        self.stage,self.manifest,paths,poses,velocities=foliage_fixture.fixture()
        robot=['/World/HarvestRobot/Robot/world','/World/ActuatedTool/R',
               '/World/HarvestRobot/Robot/S300Carrier']
        for path in robot:UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(self.stage,path).GetPrim())
        self.paths=paths+robot;self.poses=np.zeros((len(self.paths),7));self.poses[:,6]=1.
        self.velocities=np.zeros((len(self.paths),6))
        self.joint='/World/ElasticTarget/HarvestJoints/Truss_00_Fruit_02'
        joint=UsdPhysics.FixedJoint.Define(self.stage,self.joint)
        joint.CreateBody0Rel().SetTargets([foliage_fixture.STEM]);joint.CreateBody1Rel().SetTargets([foliage_fixture.FRUIT])
        self.scene=self.assembly/'scene.usda';self.stage.GetRootLayer().Export(str(self.scene))
        self.sha=file_sha256(self.scene)
        self.manifest['plant'].update(source_plant='/World/Greenhouse/Plants/Plant_40')
        self.manifest['plant']['bodies'].append(dict(path=foliage_fixture.LEAF,name='Leaf_CompoundLeaf_01'))
        self.manifest['plant']['fruits'][0].update(joint=self.joint,anchor=foliage_fixture.STEM)
        self.manifest.update(scene_sha256=self.sha,target=dict(path=foliage_fixture.FRUIT))
        self.source_context=dict(source_assembly=str(self.assembly),scene_sha256=self.sha,
            native_scene_sha256=self.sha,source_plant=self.manifest['plant']['source_plant'])
        conditioning=FoliageConditioning(self.stage,self.manifest,self.paths,seconds=1.,hz=4,
            source_context=self.source_context)
        conditioning.begin(0.,self.poses,self.velocities)
        for t in np.arange(.25,2.01,.25):
            events=[foliage_fixture.contact(float(t))] if t<1. else []
            result=conditioning.before_geometry(float(t),events,[],self.poses,self.velocities)
            self.assertIsNone(conditioning.after_geometry(float(t),result['remaining_invalid']))
        self.report=conditioning.report();self.trial=self.root/'trial';self.trial.mkdir()
        save_imported_source(self.trial)
        self.report_path=self.trial/'foliage_conditioning.json';self.report_path.write_text(json.dumps(self.report))
        self.summary_path=self.trial/'summary.json'
        self.summary=dict(mode='idle',scene=str(self.scene),model_sha256=self.sha,simulated_s=2.,
            completed=True,initialization_valid=True,canonical_conditioned_start=True,
            actual_self_collision_audit_passed=True,
            stop_reason=None,native_new_break_count=0,rail_destination=None,
            fixture_only=False,disabled_background_colliders=0,physics_settings=dict(hz=4))
        self.summary_path.write_text(json.dumps(self.summary))
        self.native_path=self.trial/'native_stage_selection.json'
        self.native_path.write_text(json.dumps(dict(source_assembly=str(self.assembly),
            source_scene_sha256=self.sha,native_scene=str(self.scene),native_scene_sha256=self.sha)))
        self.capture=self.trial/'conditioned_start_checkpoint'
        self.args=dict(assembly=self.assembly,scene_sha256=self.sha,time_s=2.,body_paths=self.paths,
            body_poses_world=self.poses,body_velocities_world=self.velocities,
            robot_dof_names=['lift','jaw_L','jaw_R'],robot_q=[.4,.0,.0],robot_qvel=[.01,.0,.0],
            robot_position_targets=[.408,.0,.0],robot_velocity_targets=[0.,0.,0.],
            plant_dof_names=['leaf:x'],plant_q=[.02],plant_qvel=[.03],preload=[.4],
            breaks=[],manifest=self.manifest,sequence=conditioned_checkpoint_sequence(self.report))
        self.states_path=self.trial/'states.npz'
        np.savez_compressed(self.states_path,time=np.arange(9)/4,
            poses=np.tile(self.poses,(9,1,1)),robot_q=np.tile(self.args['robot_q'],(9,1)),
            body_paths=np.array(self.paths))
        self.audit_path=self.trial/'self_collision_actual.json'
        self.audit=dict(passed=True,complete=True,scene_sha256=self.sha,
            input_sha256=file_sha256(self.states_path),samples=9,physics_rerun=False)
        self.audit_path.write_text(json.dumps(self.audit))

    def tearDown(self):self.tmp.cleanup()

    def capture_with_marker(self):
        save_checkpoint(self.capture,**self.args)
        return write_conditioned_start_marker(self.capture,self.report_path,self.summary_path)

    def test_write_once_marker_binds_actual_idle_state_without_completed_cycle_records(self):
        marker=self.capture_with_marker();metadata,arrays=load_checkpoint(self.capture)
        self.assertFalse(marker['completed_cycle']);self.assertFalse(marker['harvest_attempt'])
        self.assertEqual(metadata['sequence']['cycle_plans'],[])
        self.assertEqual(metadata['time_s'],2.)
        self.assertEqual(marker['report_sha256'],file_sha256(self.report_path))
        self.assertEqual(marker['source_summary_sha256'],file_sha256(self.summary_path))
        self.assertEqual(marker['checkpoint_npz_sha256'],file_sha256(self.capture/'checkpoint.npz'))
        proof=require_conditioned_checkpoint(metadata,arrays,self.capture)
        self.assertEqual(proof['kind'],'canonical_conditioned_start')
        with self.assertRaises(FileExistsError):write_conditioned_start_marker(self.capture,self.report_path,self.summary_path)
        with self.assertRaises(ValueError):require_completed_checkpoint(metadata)

    def test_explicit_sameassembly_resume_preserves_measured_velocities_and_native_targets(self):
        self.capture_with_marker()
        with self.assertRaises(ValueError):prepare_resume(self.stage,self.assembly,self.manifest,self.capture)
        context=prepare_resume(self.stage,self.assembly,self.manifest,self.capture,allow_conditioned_start=True)
        self.assertEqual(context['checkpoint_kind'],'canonical_conditioned_start')
        self.assertFalse(context['report']['canonical_conditioned_start']['completed_cycle'])
        np.testing.assert_array_equal(context['arrays']['robot_qvel'],[.01,0.,0.])
        np.testing.assert_array_equal(context['controller_state']['position_targets'],[.408,0.,0.])
        np.testing.assert_array_equal(context['arrays']['robot_q'],[.4,0.,0.])
        self.assertEqual(context['metadata']['break_events'],[])
        for revision in ('allow_leaf_proxy_revision','allow_accuracy_proxy_revision','allow_attachment_transfer'):
            with self.assertRaises(ValueError):prepare_resume(self.stage,self.assembly,self.manifest,self.capture,
                allow_conditioned_start=True,**{revision:True})

    def test_missing_marker_or_changed_evidence_cannot_promote_old_failed_idle(self):
        save_checkpoint(self.capture,**self.args)
        with self.assertRaises(FileNotFoundError):prepare_resume(self.stage,self.assembly,self.manifest,self.capture,allow_conditioned_start=True)
        write_conditioned_start_marker(self.capture,self.report_path,self.summary_path)
        metadata,arrays=load_checkpoint(self.capture)
        for path in (self.report_path,self.summary_path,self.native_path,self.states_path,self.audit_path,
                     self.trial/'runtime_code_sources/foliage_conditioning.py'):
            original=path.read_bytes();path.write_bytes(original+b'\n')
            with self.assertRaises(ValueError):require_conditioned_checkpoint(metadata,arrays,self.capture)
            path.write_bytes(original)

    def test_failed_summary_timestamp_native_sha_and_controller_unknown_are_rejected(self):
        save_checkpoint(self.capture,**self.args)
        for values in (dict(completed=False),dict(initialization_valid=False),dict(canonical_conditioned_start=False),
                       dict(actual_self_collision_audit_passed=False),dict(actual_self_collision_audit_passed=None),
                       dict(stop_reason='penetration_over_0_5mm'),dict(simulated_s=2.25),
                       dict(native_new_break_count=1),dict(rail_destination=dict(displacement_m=.1)),
                       dict(fixture_only=True),dict(disabled_background_colliders=1),dict(physics_settings=dict(hz=120))):
            self.summary_path.write_text(json.dumps(dict(self.summary,**values)))
            with self.assertRaises(ValueError):write_conditioned_start_marker(self.capture,self.report_path,self.summary_path)
        self.summary_path.write_text(json.dumps(self.summary))
        original=json.loads(self.native_path.read_text());wrong=dict(original,native_scene_sha256='0'*64)
        self.native_path.write_text(json.dumps(wrong))
        with self.assertRaises(ValueError):write_conditioned_start_marker(self.capture,self.report_path,self.summary_path)
        self.native_path.write_text(json.dumps(original))
        other=dict(self.args);other.pop('robot_position_targets');other.pop('robot_velocity_targets')
        save_checkpoint(self.trial/'unknown_controller',**other)
        with self.assertRaises(ValueError):write_conditioned_start_marker(self.trial/'unknown_controller',self.report_path,self.summary_path)

    def test_strict_gaps_nonfinite_newbreaks_and_stability_failures_never_become_markers(self):
        for mutate in (lambda r:r.update(conditioning_seconds=3.1),
                       lambda r:r.update(strict_required_seconds=.5),
                       lambda r:r.update(strict_penetration_threshold_m=.001),
                       lambda r:r['native_step_records'][-1].update(t=3.),
                       lambda r:r['native_step_records'][-1].update(deferred_count=1),
                       lambda r:r.update(finite_state=False),lambda r:r.update(new_native_break_count=1),
                       lambda r:r['stability'].update(no_increasing_drift=False)):
            wrong=copy.deepcopy(self.report);mutate(wrong)
            with self.assertRaises(ValueError):conditioned_checkpoint_sequence(wrong)

    def test_terminal_native_velocity_and_collider_inventory_bind_to_the_source(self):
        wrong=copy.deepcopy(self.args);wrong['body_velocities_world']=self.velocities.copy()
        wrong['body_velocities_world'][0,0]=.01
        save_checkpoint(self.capture,**wrong)
        with self.assertRaisesRegex(ValueError,'terminal velocity'):write_conditioned_start_marker(self.capture,self.report_path,self.summary_path)
        wrong=copy.deepcopy(self.report);wrong['exact_compliant_leaf_colliders'][foliage_fixture.MESH]['source_group']='CompoundLeaf_02'
        self.report_path.write_text(json.dumps(wrong));args=dict(self.args,sequence=conditioned_checkpoint_sequence(wrong))
        other=self.trial/'wrong_source_mapping';save_checkpoint(other,**args)
        with self.assertRaisesRegex(ValueError,'source collider inventory'):write_conditioned_start_marker(other,self.report_path,self.summary_path)

    def test_checkpoint_cannot_skip_unobserved_steps_after_the_strict_window(self):
        wrong=copy.deepcopy(self.report);wrong['final_time_s']=2.25
        self.report_path.write_text(json.dumps(wrong))
        args=dict(self.args,time_s=2.25,sequence=conditioned_checkpoint_sequence(wrong))
        save_checkpoint(self.capture,**args)
        self.summary_path.write_text(json.dumps(dict(self.summary,simulated_s=2.25)))
        with self.assertRaisesRegex(ValueError,'actual final native time'):
            write_conditioned_start_marker(self.capture,self.report_path,self.summary_path)

    def test_metadata_marker_and_actual_final_timestamp_tampering_are_rejected(self):
        self.capture_with_marker();metadata,arrays=load_checkpoint(self.capture)
        wrong=copy.deepcopy(metadata);wrong['time_s']=1.75
        with self.assertRaises(ValueError):require_conditioned_checkpoint(wrong,arrays,self.capture)
        wrong=copy.deepcopy(metadata);wrong['checkpoint_npz_sha256']='0'*64
        with self.assertRaises(ValueError):require_conditioned_checkpoint(wrong,arrays,self.capture)
        marker_path=self.capture/'conditioned_start.json';marker=json.loads(marker_path.read_text())
        marker['actual_final_time_s']=1.75;marker_path.write_text(json.dumps(marker))
        with self.assertRaises(ValueError):require_conditioned_checkpoint(metadata,arrays,self.capture)

    def test_actual_audit_and_native_trace_cannot_skip_frames_or_change_terminal_state(self):
        save_checkpoint(self.capture,**self.args)
        for values in (dict(passed=False),dict(complete=False),dict(scene_sha256='other'),
                       dict(input_sha256='other'),dict(samples=8),dict(physics_rerun=True)):
            self.audit_path.write_text(json.dumps(dict(self.audit,**values)))
            with self.assertRaises(ValueError):write_conditioned_start_marker(self.capture,self.report_path,self.summary_path)
        self.audit_path.write_text(json.dumps(self.audit))
        original=self.states_path.read_bytes()
        for mutation in ('timestamp_gap','last_pose','last_q'):
            with np.load(self.states_path) as stored:changed={key:stored[key].copy()for key in stored.files}
            if mutation=='timestamp_gap':changed['time'][-2]-=.01
            elif mutation=='last_pose':changed['poses'][-1,0,0]+=.01
            else:changed['robot_q'][-1,0]+=.01
            np.savez_compressed(self.states_path,**changed)
            self.audit_path.write_text(json.dumps(dict(self.audit,input_sha256=file_sha256(self.states_path))))
            with self.assertRaisesRegex(ValueError,'exact final checkpoint'):
                write_conditioned_start_marker(self.capture,self.report_path,self.summary_path)
            self.states_path.write_bytes(original)


if __name__=='__main__':unittest.main()
