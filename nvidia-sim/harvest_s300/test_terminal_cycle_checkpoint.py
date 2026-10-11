"""Missing one-cycle final marker publication preserves actual native state."""
import copy
import json
from pathlib import Path
import unittest

import numpy as np

from compact_motion_assets import file_sha256
from native_route_checkpoint import save_checkpoint,load_checkpoint
from resume_native_state import require_completed_checkpoint
from terminal_cycle_checkpoint import publish_final_cycle,verify_publication
from run_row_harvest import validated_initial_checkpoint
import test_native_route_checkpoint as native_fixture


class TerminalCycleCheckpointTests(unittest.TestCase):
    def setUp(self):
        self.fixture=native_fixture.NativeRouteCheckpointTests();self.fixture.setUp()
        self.root=self.fixture.root;self.assembly=self.fixture.assembly
        self.trial=self.root/'trial';self.trial.mkdir()
        self.raw=self.trial/'route_checkpoint';self.output=self.trial/'cycle_checkpoints/000'
        self.args=copy.deepcopy(self.fixture.args)
        self.args.update(robot_position_targets=[.408,.028,.019],robot_velocity_targets=[.07,.02,-.03])
        self.args['manifest']['rail']=dict(base_body='/World/HarvestRobot/Robot/world',
            initial_station_y_m=1.6,source_base_position_world_m=[2.,3.,4.])
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.args['manifest']))
        self.record=dict(index=0,status='motion_completed',start_s=1.,planned_end_s=2.,end_s=2.,
            stop_reason=None,fruits=['Truss_02_Fruit_01'],measured_frame_index=0,end_frame_index=8,
            inventory_after=dict(detached=['Truss_02_Fruit_01'],remaining=[]))
        self.sequence=dict(cycle_plans=[self.record],fruit_names=['Truss_02_Fruit_01'],
            fruit_joints={'Truss_02_Fruit_01':self.fixture.joint},attempt_counts={'Truss_02_Fruit_01':1},
            inventory=dict(detached=['Truss_02_Fruit_01'],remaining=[]),termination='all_fruit_native_joints_detached')
        self.args['sequence']=self.sequence
        self.metadata=save_checkpoint(self.raw,**self.args)
        self.summary=dict(mode='harvest',completed=True,initialization_valid=True,stop_reason=None,
            actual_self_collision_audit_passed=True,scene=str(self.assembly/'scene.usda'),
            model_sha256=self.fixture.sha,simulated_s=2.,physics_steps=8,physics_settings=dict(hz=4),
            truss_sequence=self.sequence,fixture_only=False,disabled_background_colliders=0)
        self.summary_path=self.trial/'summary.json';self.summary_path.write_text(json.dumps(self.summary))
        self.states_path=self.trial/'states.npz'
        np.savez_compressed(self.states_path,time=np.arange(9)/4,
            poses=np.tile(self.args['body_poses_world'],(9,1,1)),
            robot_q=np.tile(self.args['robot_q'],(9,1)),body_paths=np.array(self.args['body_paths']))
        self.audit=dict(passed=True,complete=True,samples=9,scene_sha256=self.fixture.sha,
            input_sha256=file_sha256(self.states_path),physics_rerun=False)
        self.audit_path=self.trial/'self_collision_actual.json';self.audit_path.write_text(json.dumps(self.audit))

    def tearDown(self):self.fixture.tearDown()

    def test_exact_one_cycle_final_snapshot_is_published_without_new_state_or_events(self):
        before={name:(self.raw/name).read_bytes()for name in ('checkpoint.json','checkpoint.npz')}
        with self.assertRaises(ValueError):require_completed_checkpoint(self.metadata)
        result=publish_final_cycle(self.raw,self.output)
        old,original=load_checkpoint(self.raw);new,published=load_checkpoint(self.output)
        require_completed_checkpoint(new)
        self.assertEqual(new['time_s'],old['time_s']);self.assertEqual(new['break_events'],old['break_events'])
        self.assertEqual(new['controller_state'],old['controller_state'])
        self.assertEqual(new['sequence']['cycle_plans'],old['sequence']['cycle_plans'])
        self.assertFalse(result['proof']['cycle_invented']);self.assertFalse(result['proof']['physics_executed'])
        for key in original:
            np.testing.assert_array_equal(published[key],original[key]);self.assertEqual(published[key].dtype,original[key].dtype)
        self.assertEqual(before,{name:(self.raw/name).read_bytes()for name in before})
        self.assertNotEqual(published['robot_q'][0],new['controller_state']['position_targets'][0])
        self.assertTrue(np.any(published['robot_qvel']!=0));self.assertTrue(np.any(published['body_velocities_world']!=0))
        _,proof=validated_initial_checkpoint(self.assembly,self.args['manifest'],self.output)
        self.assertEqual(proof['time_s'],2.);self.assertTrue(proof['verified_final_physical_state'])

    def test_stopped_invalid_earlier_unfinished_and_pending_audit_are_rejected(self):
        for patch in (dict(completed=False),dict(initialization_valid=False),dict(stop_reason='penetration_over_0_5mm'),
                      dict(actual_self_collision_audit_passed=False),dict(actual_self_collision_audit_passed=None),
                      dict(simulated_s=2.25),dict(mode='idle'),dict(disabled_background_colliders=1),dict(fixture_only=True)):
            self.summary_path.write_text(json.dumps(dict(self.summary,**patch)))
            with self.assertRaises(ValueError):publish_final_cycle(self.raw,self.output)
            self.assertFalse(self.output.exists())
        self.summary_path.write_text(json.dumps(self.summary))
        for update in (dict(status='stopped'),dict(end_s=1.75),dict(planned_end_s=2.25),dict(end_frame_index=7)):
            args=copy.deepcopy(self.args);args['sequence']['cycle_plans'][-1].update(update)
            raw=self.root/'bad'/str(len(list(self.root.glob('bad/*'))))/'route_checkpoint'
            save_checkpoint(raw,**args)
            (raw.parent/'summary.json').write_text(json.dumps(dict(self.summary,truss_sequence=args['sequence'])))
            (raw.parent/'states.npz').write_bytes(self.states_path.read_bytes())
            (raw.parent/'self_collision_actual.json').write_text(json.dumps(self.audit))
            with self.assertRaises(ValueError):publish_final_cycle(raw,raw.parent/'cycle_checkpoints/000')

    def test_missing_failed_incomplete_or_unbound_actual_audit_is_rejected(self):
        original=self.audit_path.read_bytes();self.audit_path.unlink()
        with self.assertRaises(FileNotFoundError):publish_final_cycle(self.raw,self.output)
        self.audit_path.write_bytes(original)
        for patch in (dict(passed=False),dict(complete=False),dict(samples=8),
                      dict(scene_sha256='other'),dict(input_sha256='other'),dict(physics_rerun=True)):
            self.audit_path.write_text(json.dumps(dict(self.audit,**patch)))
            with self.assertRaises(ValueError):publish_final_cycle(self.raw,self.output)
            self.assertFalse(self.output.exists())

    def test_actual_trace_endposes_q_timestamps_and_source_hashes_are_not_guessable(self):
        original=self.states_path.read_bytes()
        for mutation in ('poses','robot_q','time','body_paths'):
            with np.load(self.states_path)as stored:changed={key:stored[key].copy()for key in stored.files}
            if mutation=='poses':changed['poses'][-1,0,0]+=.01
            elif mutation=='robot_q':changed['robot_q'][-1,0]+=.01
            elif mutation=='time':changed['time'][-2]-=.01
            else:changed['body_paths']=changed['body_paths'][::-1]
            np.savez_compressed(self.states_path,**changed)
            self.audit_path.write_text(json.dumps(dict(self.audit,input_sha256=file_sha256(self.states_path))))
            with self.assertRaisesRegex(ValueError,'exact final measured'):publish_final_cycle(self.raw,self.output)
            self.states_path.write_bytes(original)
        self.audit_path.write_text(json.dumps(self.audit))
        publish_final_cycle(self.raw,self.output);metadata,arrays=load_checkpoint(self.output)
        self.summary_path.write_text(json.dumps(dict(self.summary,stop_reason='changed_after_publication')))
        with self.assertRaises(ValueError):verify_publication(metadata,arrays,self.output)
        with self.assertRaises(ValueError):validated_initial_checkpoint(self.assembly,self.args['manifest'],self.output)

    def test_write_once_and_original_trial_identity_are_enforced(self):
        with self.assertRaisesRegex(ValueError,'original native trial'):publish_final_cycle(self.raw,self.root/'elsewhere/cycle_checkpoints/000')
        publish_final_cycle(self.raw,self.output)
        before=(self.output/'checkpoint.json').read_bytes()
        with self.assertRaises(FileExistsError):publish_final_cycle(self.raw,self.output)
        self.assertEqual(before,(self.output/'checkpoint.json').read_bytes())


if __name__=='__main__':unittest.main()
