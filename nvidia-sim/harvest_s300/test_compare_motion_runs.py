"""Physical trace comparison must reject incomplete or cross-env evidence."""
import json
import hashlib
from pathlib import Path
import tempfile
import unittest

import numpy as np

from compare_motion_runs import (compare_runs,compare_common_time,quaternion_angle,prismatic_joint,
                                 shared_timestamp_indices,contact_time_diagnostic,authored_first_frame_bias)


class ComparisonTests(unittest.TestCase):
    def write_run(self,root,*,environments=1,replicated=False,tensor=False):
        root.mkdir();paths=[]
        for environment in range(environments):
            prefix=f'/World/MotionEnvs/env_{environment}' if replicated else '/World'
            paths.extend([prefix+'/ElasticTarget/fruit',prefix+'/ActuatedTool/fixed'])
        poses=np.zeros((3,len(paths),7));poses[:,:,6]=1.;poses[:,:,0]=.123
        q=np.array([[0.,.1],[.001,.2],[.002,.3]])
        np.savez(root/'states.npz',time=[0.,.01,.02],poses=poses,robot_q=q,command=q,
                 all_environment_robot_q=np.repeat(q[:,None],environments,axis=1),
                 phase=['initial','approach','seat'],body_paths=np.array(paths))
        (root/'native_metadata.json').write_text(json.dumps(dict(robot_dofs=['lift_joint','base_joint'])))
        (root/'summary.json').write_text(json.dumps(dict(model_sha256='source',completed=True,stop_reason=None,
            physics_settings=dict(hz=100,solver='PGS',armature=.0005))))
        events=[]
        for environment in range(environments):
            events.append(dict(t=.01,phase='approach',a=paths[2*environment]+'/Collider',
                               b='/World/Greenhouse/Gutter/Collider',separation_m=-.0001,
                               force_n=2.,impulse_ns=[.01,.02,0.]))
        if tensor:
            for event in events:
                event.pop('impulse_ns');event.update(collider_identity_known=False,actor_identity_known=True,
                                                     force_metric='actor normal force')
        (root/'contacts.json').write_text(json.dumps(events))

    def test_replicated_paths_and_shared_gutter_compare_per_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            baseline=Path(directory)/'baseline';candidate=Path(directory)/'candidate'
            self.write_run(baseline);self.write_run(candidate,environments=2,replicated=True)
            result=compare_runs(baseline,candidate)
            self.assertTrue(result['passed']);self.assertEqual(result['source_environments'],2)
            self.assertTrue(all(row['contacts']['reference_points']==1 for row in result['environment_results']))

    def test_cross_environment_contact_is_not_hidden_by_path_normalization(self):
        with tempfile.TemporaryDirectory() as directory:
            baseline=Path(directory)/'baseline';candidate=Path(directory)/'candidate'
            self.write_run(baseline);self.write_run(candidate,environments=2,replicated=True)
            contacts=json.loads((candidate/'contacts.json').read_text())
            contacts.append(dict(contacts[0],b='/World/MotionEnvs/env_1/ElasticTarget/fruit/Collider'))
            (candidate/'contacts.json').write_text(json.dumps(contacts))
            result=compare_runs(baseline,candidate)
            self.assertFalse(result['passed']);self.assertEqual(result['cross_environment_contacts']['actual_count'],1)

    def test_truncated_trace_cannot_pass_by_matching_common_prefix(self):
        with tempfile.TemporaryDirectory() as directory:
            baseline=Path(directory)/'baseline';candidate=Path(directory)/'candidate'
            self.write_run(baseline);self.write_run(candidate)
            with np.load(candidate/'states.npz') as data:arrays={key:data[key] for key in data.files}
            for key in arrays:
                if key!='body_paths':arrays[key]=arrays[key][:2]
            np.savez(candidate/'states.npz',**arrays)
            result=compare_runs(baseline,candidate)
            self.assertFalse(result['passed']);self.assertTrue(result['environment_results'][0]['states']['partial_common_prefix_only'])

    def test_tensor_actor_normal_force_is_not_native_impulse_equivalence(self):
        with tempfile.TemporaryDirectory() as directory:
            baseline=Path(directory)/'baseline';candidate=Path(directory)/'candidate'
            self.write_run(baseline);self.write_run(candidate,tensor=True)
            result=compare_runs(baseline,candidate)
            self.assertFalse(result['passed']);self.assertIsNone(result['environment_results'][0]['contacts']['passed'])

    def test_contact_order_and_reversed_pair_impulse_sign_are_canonicalized(self):
        with tempfile.TemporaryDirectory() as directory:
            baseline=Path(directory)/'baseline';candidate=Path(directory)/'candidate'
            self.write_run(baseline);self.write_run(candidate)
            contacts=json.loads((baseline/'contacts.json').read_text());contacts.append(dict(contacts[0],separation_m=-.0002,impulse_ns=[.03,.04,0.]))
            (baseline/'contacts.json').write_text(json.dumps(contacts))
            reversed_contacts=[]
            for event in contacts[::-1]:
                reversed_contacts.append(dict(event,a=event['b'],b=event['a'],impulse_ns=(-np.array(event['impulse_ns'])).tolist()))
            (candidate/'contacts.json').write_text(json.dumps(reversed_contacts))
            result=compare_runs(baseline,candidate)
            self.assertTrue(result['passed']);self.assertTrue(result['environment_results'][0]['contacts']['exact_order_independent'])

    def test_quaternion_sign_does_not_change_orientation(self):
        q=np.array([.1,.2,.3,.9]);self.assertLess(float(quaternion_angle(q,-q)),1e-15)

    def test_original_lift_joint_is_measured_in_metres(self):
        self.assertTrue(prismatic_joint('farmily_lift_height_joint'))
        self.assertTrue(prismatic_joint('jaw_L'))
        self.assertFalse(prismatic_joint('wrist1'))

    def test_common_time_grid_pairs_stored_samples_without_interpolation(self):
        left,right=shared_timestamp_indices([0.,.005,.01,.015,.02],[0.,.01,.02])
        np.testing.assert_array_equal(left,[0,2,4]);np.testing.assert_array_equal(right,[0,1,2])
        for invalid in ([0.,0.,.02],[0.,float('nan')],[]):
            with self.assertRaises(ValueError):shared_timestamp_indices(invalid,[0.,.01])
        with self.assertRaises(ValueError):shared_timestamp_indices([0.],[0.],-1.)

    def test_common_time_keeps_initial_world_bias_separate_from_displacement(self):
        with tempfile.TemporaryDirectory() as directory:
            baseline=Path(directory)/'baseline';candidate=Path(directory)/'candidate'
            self.write_run(baseline);self.write_run(candidate)
            with np.load(candidate/'states.npz') as data:arrays={key:data[key] for key in data.files}
            arrays['poses'][:,:,1]+=.01
            np.savez(candidate/'states.npz',**arrays)
            result=compare_common_time(baseline,candidate)
            json.dumps(result,allow_nan=False)
            self.assertIsNone(result['passed']);self.assertIsNone(result['optimization_physics_equivalent'])
            self.assertAlmostEqual(result['maximum_world_position_error_m'],.01)
            self.assertAlmostEqual(result['initial_world']['maximum_position_difference_m'],.01)
            self.assertEqual(result['maximum_displacement_from_own_initial_error_m'],0.)
            self.assertTrue(all(row['commands_exact'] for row in result['native_robot_joint_metrics']))
            self.assertFalse(result['plant_joint_q_series']['available'])

    def test_common_time_no_overlap_is_explicitly_unavailable(self):
        with tempfile.TemporaryDirectory() as directory:
            baseline=Path(directory)/'baseline';candidate=Path(directory)/'candidate'
            self.write_run(baseline);self.write_run(candidate)
            with np.load(candidate/'states.npz') as data:arrays={key:data[key] for key in data.files}
            arrays['time']=arrays['time']+.001
            np.savez(candidate/'states.npz',**arrays)
            with self.assertRaisesRegex(ValueError,'No stored shared timestamps'):
                compare_common_time(baseline,candidate)

    def test_native_impulses_integrate_once_and_initial_records_stay_separate(self):
        a='/World/ActuatedTool/Collider';b='/World/ElasticTarget/fruit/Collider'
        event=dict(t=.01,a=a,b=b,phase='seat',separation_m=-.0001,force_n=10.,impulse_ns=[.01,.02,0.])
        events=[dict(event,t=0.,impulse_ns=[100.,0.,0.]),event,
                dict(event,t=.02,a=b,b=a,impulse_ns=[-.01,-.02,0.])]
        result=contact_time_diagnostic(events,{},0.,.02)
        self.assertEqual(result['warmup_and_start_records_excluded'],1)
        self.assertEqual(result['contact_point_records'],2)
        np.testing.assert_allclose(result['largest_pairs'][0]['integrated_canonical_impulse_ns'],[.02,.04,0.])
        self.assertIsNone(result['passed']);self.assertFalse(result['bit_exact_comparable'])
        tensor=dict(event);tensor.pop('impulse_ns');tensor['collider_identity_known']=False
        self.assertFalse(contact_time_diagnostic([tensor],{},0.,.02)['integrated_native_impulse_available'])

    def test_authored_rest_pose_does_not_assume_first_native_frame_is_zero(self):
        try:
            from pxr import Usd,UsdGeom
        except ImportError:
            self.skipTest('Offline USD bindings unavailable')
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);scene=root/'scene.usda';stage=Usd.Stage.CreateNew(str(scene))
            UsdGeom.Xform.Define(stage,'/World/ElasticTarget/fruit').AddTranslateOp().Set((1.,2.,3.))
            stage.GetRootLayer().Save()
            (root/'summary.json').write_text(json.dumps(dict(scene=str(scene),model_sha256=hashlib.sha256(scene.read_bytes()).hexdigest())))
            poses=np.array([[[1.,2.01,3.,0.,0.,0.,1.]]])
            result=authored_first_frame_bias(root,dict(poses=poses),{'/World/ElasticTarget/fruit':0})
            self.assertTrue(result['available']);self.assertTrue(result['scene_hash_matches_summary'])
            self.assertAlmostEqual(result['max_position_bias_m'],.01)
            self.assertEqual(result['max_rotation_bias_rad'],0.)


if __name__=='__main__':unittest.main()
