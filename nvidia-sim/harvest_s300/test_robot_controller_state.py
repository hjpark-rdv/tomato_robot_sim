import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from robot_controller_state import resolve_robot_controller_state,rebase_rail_controller_state,restore_controller_targets


class RobotControllerStateTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.trial=self.root/'trial';self.checkpoint=self.trial/'cycle_checkpoints/000'
        self.checkpoint.mkdir(parents=True);self.assembly=self.root/'assembly';self.assembly.mkdir()
        self.names=['root_joint','lift','jaw_L','jaw_R']
        self.q=np.array([.3,.2,-.01,.02]);self.qvel=np.array([.01,-.03,.02,.04])
        self.target=self.q+np.array([.001,.0084,.002,-.002])
        self.paths=['/World/Robot/base','/World/Plant/fruit']
        self.poses=np.array([[0,1,2,0,0,0,1.],[3,4,5,0,0,0,1.]])
        self.manifest={'plant':{'source_plant':'Plant22'}}
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))
        self.metadata=dict(metadata_sha256='b'*64,source_assembly=str(self.assembly),
            source_scene=str(self.assembly/'scene.usda'),scene_sha256='a'*64,time_s=54.,
            manifest=self.manifest,sequence={'cycle_plans':[{'end_frame_index':1}]})
        self.arrays=dict(robot_dof_names=np.array(self.names),robot_q=self.q.copy(),robot_qvel=self.qvel.copy(),
            body_paths=np.array(self.paths),body_poses_world=self.poses.copy())

    def tearDown(self):self.tmp.cleanup()

    def native(self):
        self.metadata['controller_state']=dict(schema='native_robot_drive_targets_v1',robot_dof_names=self.names,
            position_targets=self.target.tolist(),velocity_targets=[0.,0.,0.,0.],
            position_source='native_get_dof_position_targets',velocity_source='native_get_dof_velocity_targets')

    def legacy(self,**changes):
        (self.trial/'code').mkdir(exist_ok=True)
        (self.trial/'code/run_motion.py').write_text('robot.set_dof_position_targets(command)\n')
        (self.trial/'summary.json').write_text(json.dumps(dict(scene=self.metadata['source_scene'],model_sha256='a'*64)))
        (self.trial/'native_metadata.json').write_text(json.dumps(dict(model_sha256='a'*64,robot_dofs=self.names)))
        values=dict(time=np.array([53.,54.]),robot_q=np.vstack([self.q,self.q]),
            poses=np.stack([self.poses,self.poses]),body_paths=np.array(self.paths),
            command=np.vstack([self.target,self.target]),phase=np.array(['retract','hold']))
        values.update(changes);np.savez(self.trial/'states.npz',**values)

    def test_native_drive_targets_are_separate_from_measured_q_and_qvel(self):
        self.native();state=resolve_robot_controller_state(self.metadata,self.arrays,self.checkpoint)
        np.testing.assert_array_equal(state['position_targets'],self.target)
        np.testing.assert_array_equal(state['velocity_targets'],np.zeros(4))
        self.assertFalse(np.array_equal(state['position_targets'],self.q))
        self.assertFalse(np.array_equal(state['velocity_targets'],self.qvel))
        self.assertTrue(state['provenance']['controller_continuity_verified'])

    def test_legacy_exact_trace_proves_position_setter_not_unknown_velocity(self):
        self.legacy();state=resolve_robot_controller_state(self.metadata,self.arrays,self.checkpoint)
        np.testing.assert_array_equal(state['position_targets'],self.target)
        self.assertIsNone(state['velocity_targets'])
        self.assertFalse(state['provenance']['controller_continuity_verified'])
        self.assertTrue(state['provenance']['position_targets_verified'])
        self.assertEqual(len(state['provenance']['source_file_sha256']),4)
        self.assertEqual(state['provenance']['timestamp_s'],54.)

    def test_missing_legacy_evidence_stays_unknown_not_measured_q(self):
        state=resolve_robot_controller_state(self.metadata,self.arrays,self.checkpoint)
        self.assertIsNone(state['position_targets']);self.assertIsNone(state['velocity_targets'])
        self.assertFalse(state['provenance']['position_targets_verified'])

    def test_native_wrong_order_or_boolean_not_accepted(self):
        self.native();self.metadata['controller_state']['robot_dof_names']=list(reversed(self.names))
        with self.assertRaises(ValueError):resolve_robot_controller_state(self.metadata,self.arrays,self.checkpoint)
        self.native();self.metadata['controller_state']['position_targets'][0]=True
        with self.assertRaises(ValueError):resolve_robot_controller_state(self.metadata,self.arrays,self.checkpoint)

    def test_legacy_model_timestamp_body_and_q_must_match_exactly(self):
        for mode in ('model','timestamp','body','q','frame'):
            with self.subTest(mode=mode):
                self.legacy();metadata=copy.deepcopy(self.metadata);arrays=copy.deepcopy(self.arrays)
                if mode=='model':metadata['scene_sha256']='c'*64
                if mode=='timestamp':metadata['time_s']=54.1
                if mode=='body':arrays['body_poses_world'][0,0]+=.000001
                if mode=='q':arrays['robot_q'][1]+=.000001
                if mode=='frame':metadata['sequence']['cycle_plans'][0]['end_frame_index']=0
                with self.assertRaises(ValueError):resolve_robot_controller_state(metadata,arrays,self.checkpoint)

    def test_body_order_can_differ_with_exact_identity_and_pose(self):
        self.legacy(body_paths=np.array(self.paths[::-1]),poses=np.stack([self.poses[::-1],self.poses[::-1]]))
        state=resolve_robot_controller_state(self.metadata,self.arrays,self.checkpoint)
        np.testing.assert_array_equal(state['position_targets'],self.target)

    def test_rail_rebase_preserves_same_world_target_not_zero_measured_hold(self):
        self.native();state=resolve_robot_controller_state(self.metadata,self.arrays,self.checkpoint)
        current=rebase_rail_controller_state(state,self.arrays,'root_joint')
        self.assertAlmostEqual(current['position_targets'][0],.001)
        np.testing.assert_array_equal(current['position_targets'][1:],self.target[1:])
        np.testing.assert_array_equal(current['velocity_targets'],state['velocity_targets'])
        np.testing.assert_array_equal(state['position_targets'],self.target)
        self.assertTrue(current['provenance']['rail_position_target_rebase']['world_station_setpoint_preserved'])

    def test_native_setter_failure_is_not_hidden_by_correct_measured_q(self):
        self.native();state=resolve_robot_controller_state(self.metadata,self.arrays,self.checkpoint)
        class FailedSetter:
            def set_dof_position_targets(inner,*args):pass
            def set_dof_velocity_targets(inner,*args):pass
            def get_dof_position_targets(inner):return self.q[None]
            def get_dof_velocity_targets(inner):return np.zeros((1,4))
        with self.assertRaisesRegex(ValueError,'proven setpoint'):
            restore_controller_targets(FailedSetter(),state,[0])


if __name__=='__main__':unittest.main()
