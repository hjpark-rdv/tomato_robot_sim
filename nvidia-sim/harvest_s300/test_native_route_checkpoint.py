import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from native_route_checkpoint import save_checkpoint,load_checkpoint,validate_checkpoint


class NativeRouteCheckpointTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.assembly=self.root/'assembly';self.assembly.mkdir()
        (self.assembly/'scene.usda').write_text('#usda 1.0\n# exact source fixture\n')
        self.sha=hashlib.sha256((self.assembly/'scene.usda').read_bytes()).hexdigest()
        self.anchor='/World/ElasticTarget/Pedicel_01'
        self.fruit='/World/ElasticTarget/Truss_02_Fruit_01'
        self.joint='/World/ElasticTarget/HarvestJoints/Truss_02_Fruit_01'
        self.manifest=dict(scene_sha256=self.sha,
            plant=dict(root='/World/ElasticTarget',source_plant='/World/Greenhouse/Plants/Plant_22',
                       bodies=[dict(path=self.anchor)],
                       fruits=[dict(path=self.fruit,joint=self.joint)]),
            target=dict(path=self.fruit))
        self.args=dict(assembly=self.assembly,scene_sha256=self.sha,time_s=2.,
            body_paths=[self.anchor,self.fruit,'/World/HarvestRobot/Robot/world','/World/ActuatedTool/R'],
            body_poses_world=np.array([[0,0,0,0,0,0,1],[1,2,3,0,0,0,1],
                                       [2,3,4,0,0,0,1],[3,4,5,0,0,0,1.]]),
            body_velocities_world=np.arange(24,dtype=float).reshape(4,6)/10,
            robot_dof_names=['lift','jaw_L','jaw_R'],robot_q=[.4,.02,.03],robot_qvel=[.11,-.12,.13],
            plant_dof_names=['Pedicel_01:0'],plant_q=[.21],plant_qvel=[.32],preload=[.43],
            breaks=[dict(t=1.5,joint=self.joint,phase='cycle_open')],manifest=self.manifest,
            sequence=dict(cycle_index=3,fruit_names=['Truss_02_Fruit_01'],
                          attempts={'Truss_02_Fruit_01':1},remaining=[]))

    def tearDown(self):self.tmp.cleanup()

    def save(self,name='checkpoint',**updates):
        args=dict(self.args);args.update(updates)
        return save_checkpoint(self.root/name,**args)

    def test_round_trip_preserves_measured_velocities_jaws_breaks_and_sequence(self):
        saved=self.save()
        metadata,arrays=load_checkpoint(self.root/'checkpoint',expected_scene_sha256=self.sha)
        self.assertEqual(saved,metadata)
        self.assertEqual(metadata['quaternion_order'],'xyzw')
        self.assertFalse(metadata['velocities_inferred']);self.assertFalse(metadata['training_eligible'])
        self.assertEqual(metadata['source_plant'],'/World/Greenhouse/Plants/Plant_22')
        self.assertEqual(metadata['break_events'],self.args['breaks'])
        self.assertEqual(metadata['sequence'],self.args['sequence'])
        for key in ('body_poses_world','body_velocities_world','robot_q','robot_qvel','plant_q','plant_qvel','preload'):
            np.testing.assert_array_equal(arrays[key],self.args[key])
        self.assertEqual(arrays['robot_dof_names'].tolist(),['lift','jaw_L','jaw_R'])
        self.assertEqual(metadata['robot_articulation_scope'],'One native articulation including jaws; no separate assumed tool articulation')

    def test_write_once_does_not_replace_existing_even_empty_output(self):
        self.save();paths=[self.root/'checkpoint'/name for name in ('checkpoint.json','checkpoint.npz')]
        before=[p.read_bytes() for p in paths]
        with self.assertRaises(FileExistsError):self.save(robot_qvel=[9.,8.,7.])
        self.assertEqual(before,[p.read_bytes() for p in paths])
        (self.root/'empty').mkdir()
        with self.assertRaises(FileExistsError):self.save('empty')

    def test_missing_velocity_or_bad_shape_nonfinite_values_are_rejected(self):
        bad=dict(self.args);bad.pop('body_velocities_world')
        with self.assertRaises(TypeError):save_checkpoint(self.root/'missing',**bad)
        for key,value in (('body_velocities_world',np.zeros((4,3))),('robot_qvel',[np.nan,0,0]),
                          ('plant_qvel',[np.inf]),('preload',[]),('robot_q',[[1,2,3]])):
            with self.subTest(key=key):
                with self.assertRaises(ValueError):self.save('bad',**{key:value})
                self.assertFalse((self.root/'bad').exists())

    def test_pose_and_exact_identity_mapping_fail_closed(self):
        invalid=self.args['body_poses_world'].copy();invalid[0,3:]=0.
        with self.assertRaises(ValueError):self.save(body_poses_world=invalid)
        with self.assertRaises(ValueError):self.save(body_paths=[self.anchor,self.fruit,self.fruit,'/World/ActuatedTool/R'])
        with self.assertRaises(ValueError):self.save(robot_dof_names=['lift','jaw_L','arm'])
        with self.assertRaises(ValueError):self.save(body_paths=[self.anchor,'/World/MissingFruit','/World/HarvestRobot/Robot/world','/World/ActuatedTool/R'])
        with self.assertRaises(ValueError):self.save(source_plant='/World/Greenhouse/Plants/Plant_99')

    def test_source_scene_and_expected_model_hash_mismatch_are_rejected(self):
        with self.assertRaises(ValueError):self.save(scene_sha256='f'*64)
        self.save()
        with self.assertRaises(ValueError):load_checkpoint(self.root/'checkpoint',expected_scene_sha256='e'*64)
        (self.assembly/'scene.usda').write_text('# source was changed\n')
        with self.assertRaises(ValueError):load_checkpoint(self.root/'checkpoint')

    def test_npz_and_json_mutation_are_detected(self):
        self.save()
        data_path=self.root/'checkpoint/checkpoint.npz';data_path.chmod(0o644)
        with data_path.open('ab') as stream:stream.write(b'tampered')
        with self.assertRaisesRegex(ValueError,'NPZ SHA'):load_checkpoint(self.root/'checkpoint')
        self.save('other')
        path=self.root/'other/checkpoint.json';path.chmod(0o644)
        metadata=json.loads(path.read_text());metadata['sequence']['cycle_index']=99
        path.write_text(json.dumps(metadata))
        with self.assertRaisesRegex(ValueError,'metadata SHA'):load_checkpoint(self.root/'other')

    def test_materialized_dependency_change_cannot_hide_behind_same_wrapper_sha(self):
        dependency=self.assembly/'composed.usdc';dependency.write_bytes(b'fixture materialized scene')
        manifest=copy.deepcopy(self.manifest)
        manifest['compact_assets']=dict(materialized_dependency=dict(relative_path='composed.usdc',
            sha256=hashlib.sha256(dependency.read_bytes()).hexdigest()))
        self.save(manifest=manifest)
        load_checkpoint(self.root/'checkpoint')
        dependency.write_bytes(b'changed materialized scene; wrapper stays identical')
        with self.assertRaisesRegex(ValueError,'compact dependency SHA'):load_checkpoint(self.root/'checkpoint')

    def test_native_break_inventory_and_capture_time_are_preserved_and_validated(self):
        for events in ([dict(t=2.1,joint=self.joint)],
                       [dict(t=1.,joint='/World/ElasticTarget/UnknownJoint')],
                       [dict(t=float('nan'),joint=self.joint)]):
            with self.subTest(events=events):
                with self.assertRaises(ValueError):self.save(breaks=events)
        with self.assertRaises(ValueError):self.save(time_s=-1)
        with self.assertRaises(ValueError):self.save(time_s=True)
        sequence=dict(self.args['sequence'],unknown=float('inf'))
        with self.assertRaises(ValueError):self.save(sequence=sequence)

    def test_array_order_schema_and_descriptors_are_checked_independently(self):
        self.save();metadata,arrays=load_checkpoint(self.root/'checkpoint')
        unsigned=copy.deepcopy(metadata);unsigned.pop('metadata_sha256')
        unsigned['mappings']['robot_dof_names'].reverse()
        with self.assertRaisesRegex(ValueError,'mapping/order'):validate_checkpoint(unsigned,arrays)
        unsigned=copy.deepcopy(metadata);unsigned.pop('metadata_sha256')
        unsigned['arrays']['robot_q']['dtype']='float32'
        with self.assertRaisesRegex(ValueError,'descriptor'):validate_checkpoint(unsigned,arrays)
        missing=dict(arrays);missing.pop('robot_qvel')
        with self.assertRaisesRegex(ValueError,'array schema'):validate_checkpoint(metadata,missing)

    def test_optional_native_targets_are_separate_from_measured_q_and_legacy_schema(self):
        self.save('legacy');legacy,legacy_arrays=load_checkpoint(self.root/'legacy')
        self.assertNotIn('controller_state',legacy)
        position=np.array([.408467391,.028,.019]);velocity=np.array([.07,.02,-.03])
        self.save(robot_position_targets=position,robot_velocity_targets=velocity)
        position[:]=99;velocity[:]=99
        metadata,arrays=load_checkpoint(self.root/'checkpoint');state=metadata['controller_state']
        self.assertEqual(set(arrays),set(legacy_arrays));self.assertEqual(metadata['arrays'],legacy['arrays'])
        np.testing.assert_array_equal(arrays['robot_q'],self.args['robot_q'])
        np.testing.assert_array_equal(arrays['robot_qvel'],self.args['robot_qvel'])
        self.assertEqual(state['position_targets'],[.408467391,.028,.019])
        self.assertEqual(state['velocity_targets'],[.07,.02,-.03])
        self.assertEqual(state['robot_dof_names'],self.args['robot_dof_names'])
        self.assertEqual(state['position_source'],'native_get_dof_position_targets')
        self.assertEqual(state['velocity_source'],'native_get_dof_velocity_targets')

    def test_optional_native_target_pair_shape_numeric_identity_and_units_fail_closed(self):
        for options in [dict(robot_position_targets=[1,2,3]),dict(robot_velocity_targets=[0,0,0]),
                        dict(robot_position_targets=[[1,2,3]],robot_velocity_targets=[0,0,0]),
                        dict(robot_position_targets=[1,np.nan,3],robot_velocity_targets=[0,0,0]),
                        dict(robot_position_targets=[1,2,3],robot_velocity_targets=[0,np.inf,0])]:
            with self.subTest(options=options):
                with self.assertRaises(ValueError):self.save('bad',**options)
                self.assertFalse((self.root/'bad').exists())
        self.save(robot_position_targets=[1,2,3],robot_velocity_targets=[0,0,0])
        metadata,arrays=load_checkpoint(self.root/'checkpoint')
        for key,value in [('robot_dof_names',['jaw_L','lift','jaw_R']),('position_targets',[1,2]),
                          ('velocity_targets',[0,False,0]),('units',dict(position='mm',velocity='mm/s')),
                          ('position_source','measured_robot_q')]:
            unsigned=copy.deepcopy(metadata);unsigned.pop('metadata_sha256')
            unsigned['controller_state'][key]=value
            with self.subTest(key=key):
                with self.assertRaises(ValueError):validate_checkpoint(unsigned,arrays)


if __name__=='__main__':unittest.main()
