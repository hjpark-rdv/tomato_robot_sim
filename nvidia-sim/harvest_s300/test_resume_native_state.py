import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from compact_motion_assets import file_sha256
from native_route_checkpoint import save_checkpoint
from resume_native_state import (apply_resume, imported_break_markers,
                                 prepare_resume, require_completed_checkpoint,
                                 verify_resume)


class Articulation:
    def __init__(self, names):
        self.count = 1; self.max_dofs = len(names)
        self.shared_metatype = SimpleNamespace(dof_names=list(names))
        self.q = np.zeros((1, len(names))); self.qvel = self.q.copy()
        self.calls = []
    def set_dof_positions(self, value, ids): self.q = value.copy(); self.calls.append('q')
    def set_dof_velocities(self, value, ids): self.qvel = value.copy(); self.calls.append('qvel')
    def set_dof_position_targets(self, value, ids): self.targets = value.copy()
    def set_dof_velocity_targets(self, value, ids): self.velocity_targets = value.copy()
    def get_dof_position_targets(self):return self.targets
    def get_dof_velocity_targets(self):return self.velocity_targets
    def set_dof_actuation_forces(self, value, ids): self.preload = value.copy()
    def get_dof_positions(self): return self.q
    def get_dof_velocities(self): return self.qvel


class Bodies:
    def __init__(self, paths, poses, velocities):
        self.prim_paths = list(paths); self.poses = poses.copy(); self.velocities = velocities.copy()
        self.set_rows = []
    def get_transforms(self): return self.poses
    def get_velocities(self): return self.velocities
    def set_transforms(self, values, ids):
        assert values.shape == self.poses.shape
        self.poses[ids] = values[ids]; self.set_rows.append(list(map(int, ids)))
    def set_velocities(self, values, ids):
        assert values.shape == self.velocities.shape
        self.velocities[ids] = values[ids]


class ResumeNativeStateTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.root = Path(self.tmp.name)
        self.assembly = self.root/'assembly'; self.assembly.mkdir()
        self.scene = self.assembly/'scene.usda'; self.stage = Usd.Stage.CreateNew(str(self.scene))
        self.anchor = '/World/ElasticTarget/Pedicel_01'
        self.fruit = '/World/ElasticTarget/Truss_02_Fruit_01'
        self.joint = '/World/ElasticTarget/HarvestJoints/Truss_02_Fruit_01'
        self.carrier = '/World/HarvestRobot/Robot/S300Carrier'
        self.paths = [self.anchor, self.fruit, '/World/HarvestRobot/Robot/world',
                      '/World/ActuatedTool/R', self.carrier]
        for path in self.paths: UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(self.stage, path).GetPrim())
        joint = UsdPhysics.FixedJoint.Define(self.stage, self.joint)
        joint.CreateBody0Rel().SetTargets([self.anchor]); joint.CreateBody1Rel().SetTargets([self.fruit])
        self.stage.GetRootLayer().Save()
        self.sha = file_sha256(self.scene)
        self.manifest = dict(scene_sha256=self.sha,
            plant=dict(root='/World/ElasticTarget', source_plant='/World/Greenhouse/Plants/Plant_22',
                       bodies=[dict(path=self.anchor)],
                       fruits=[dict(path=self.fruit, joint=self.joint, anchor=self.anchor)]),
            target=dict(path=self.fruit), rail=dict(carrier_body=self.carrier))
        self.poses = np.column_stack([np.arange(15).reshape(5, 3)/10., np.tile([0, 0, 0, 1.], (5, 1))])
        self.velocities = np.arange(30).reshape(5, 6)/100.
        self.robot_names = ['lift', 'jaw_L', 'jaw_R']; self.plant_names = ['Pedicel_01:0']
        self.sequence = dict(checkpoint_capture_reason='completed_cycle', checkpoint_physics_valid=True,
            cycle_plans=[dict(status='motion_completed', end_s=158., stop_reason=None)])
        self.args = dict(assembly=self.assembly, scene_sha256=self.sha, time_s=158.,
            body_paths=self.paths, body_poses_world=self.poses, body_velocities_world=self.velocities,
            robot_dof_names=self.robot_names, robot_q=[.4, .02, .03], robot_qvel=[.11, -.12, .13],
            plant_dof_names=self.plant_names, plant_q=[.21], plant_qvel=[.32], preload=[.43],
            breaks=[dict(t=85.2, joint=self.joint, phase='cycle_close')],
            manifest=self.manifest, sequence=self.sequence)
        save_checkpoint(self.root/'checkpoint', **self.args)

    def tearDown(self): self.tmp.cleanup()

    def context(self):
        return prepare_resume(self.stage, self.assembly, self.manifest, self.root/'checkpoint')

    def views(self):
        robot, plant = Articulation(self.robot_names), Articulation(self.plant_names)
        order = [4, 2, 0, 3, 1]
        bodies = Bodies([self.paths[i] for i in order], self.poses[order], self.velocities[order])
        for path in (self.fruit, self.carrier):
            i = bodies.prim_paths.index(path); bodies.poses[i, :3] += 1.; bodies.velocities[i] = -2.
        return robot, plant, bodies

    def test_broken_joint_disable_is_session_only_and_preserves_native_proof(self):
        before = self.scene.read_bytes(); context = self.context()
        self.assertEqual(before, self.scene.read_bytes())
        self.assertFalse(UsdPhysics.Joint(self.stage.GetPrimAtPath(self.joint)).GetJointEnabledAttr().Get())
        pristine = Usd.Stage.Open(str(self.scene), Usd.Stage.LoadAll)
        # Cached stage session may be shared; direct authored root spec remains empty.
        self.assertFalse(self.stage.GetRootLayer().GetPrimAtPath(self.joint).HasInfo('physics:jointEnabled'))
        proof = context['report']['imported_native_prior'][0]
        self.assertEqual(proof['t'], 0.); self.assertEqual(proof['original_t'], 85.2)
        self.assertEqual(proof['type'], 'imported_native_prior'); self.assertFalse(proof['native_new_event'])
        self.assertEqual(context['metadata']['break_events'][0]['t'], 85.2)
        self.assertEqual(context['report']['initial_native_break_count'], 1)

    def test_restore_has_nonzero_measured_velocities_and_full_native_body_buffers(self):
        context = self.context(); robot, plant, bodies = self.views()
        report = apply_resume(context, robot, plant, bodies)
        np.testing.assert_array_equal(robot.q[0], self.args['robot_q'])
        np.testing.assert_array_equal(robot.qvel[0], self.args['robot_qvel'])
        np.testing.assert_array_equal(plant.qvel[0], self.args['plant_qvel'])
        np.testing.assert_array_equal(plant.preload[0], self.args['preload'])
        for path in self.paths:
            i = bodies.prim_paths.index(path); j = self.paths.index(path)
            np.testing.assert_array_equal(bodies.poses[i], self.poses[j])
            np.testing.assert_array_equal(bodies.velocities[i], self.velocities[j])
        rows = bodies.set_rows[0]
        self.assertEqual({bodies.prim_paths[i] for i in rows}, {self.fruit, self.carrier})
        self.assertEqual(report['max_position_error_m'], 0.)
        self.assertFalse(report['articulated_link_transforms_set_directly'])
        self.assertFalse(report['bitwise_engine_continuity'])

    def test_native_controller_targets_restored_without_substituting_q_or_qvel(self):
        position=[.4084,.021,.028];velocity=[0.,0.,0.]
        args=dict(self.args,robot_position_targets=position,robot_velocity_targets=velocity)
        save_checkpoint(self.root/'native_targets',**args)
        context=prepare_resume(self.stage,self.assembly,self.manifest,self.root/'native_targets')
        robot,plant,bodies=self.views();report=apply_resume(context,robot,plant,bodies)
        np.testing.assert_array_equal(robot.q[0],self.args['robot_q'])
        np.testing.assert_array_equal(robot.qvel[0],self.args['robot_qvel'])
        np.testing.assert_array_equal(robot.targets[0],position)
        np.testing.assert_array_equal(robot.velocity_targets[0],velocity)
        self.assertEqual(report['restored_controller_state']['position_targets_max_abs_error'],0.)
        self.assertEqual(report['restored_controller_state']['velocity_targets_max_abs_error'],0.)

    def test_native_dof_order_or_missing_body_is_a_hard_fail(self):
        context = self.context(); robot, plant, bodies = self.views()
        robot.shared_metatype.dof_names.reverse()
        with self.assertRaisesRegex(ValueError, 'DOF name/order'): apply_resume(context, robot, plant, bodies)
        robot, plant, bodies = self.views(); bodies.prim_paths[-1] = '/World/WrongBody'
        with self.assertRaisesRegex(ValueError, 'identity coverage'): apply_resume(context, robot, plant, bodies)

    def test_stopped_or_undocumented_checkpoint_cannot_resume(self):
        context = self.context()
        for updates in ({'checkpoint_physics_valid': False}, {'checkpoint_capture_reason': 'final_stop'},
                        {'checkpoint_physics_valid': None}, {'cycle_plans': [dict(status='stopped', end_s=158.)]}):
            metadata = copy.deepcopy(context['metadata']); metadata['sequence'].update(updates)
            with self.assertRaises(ValueError): require_completed_checkpoint(metadata)

    def test_exact_joint_body_mapping_is_checked_before_any_session_write(self):
        # A source/session mapping bug is not treated as an already detached fruit.
        joint = UsdPhysics.Joint(self.stage.GetPrimAtPath(self.joint))
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            joint.GetBody1Rel().SetTargets(['/World/WrongFruit'])
        with self.assertRaisesRegex(ValueError, 'body mapping'): self.context()
        self.assertIsNone(self.stage.GetSessionLayer().GetAttributeAtPath(self.joint+'.physics:jointEnabled'))

    def test_numeric_comparison_reports_every_body_without_promoting_success(self):
        context = self.context(); robot, plant, bodies = self.views()
        apply_resume(context, robot, plant, bodies)
        i = bodies.prim_paths.index(self.anchor); bodies.poses[i, 0] += .0007
        bodies.poses[i, 3:] *= -1.
        report = verify_resume(context, robot, plant, bodies)
        self.assertAlmostEqual(report['max_position_error_m'], .0007)
        self.assertEqual(report['max_orientation_error_rad'], 0.)
        self.assertEqual(report['worst_position_error_m_body'], self.anchor)
        self.assertEqual(report['native_validity'], 'pending')

    def test_quaternion_norm_rounding_is_separate_from_real_rotation_error(self):
        from scipy.spatial.transform import Rotation
        context=self.context();robot,plant,bodies=self.views()
        apply_resume(context,robot,plant,bodies)
        i=bodies.prim_paths.index(self.anchor)
        expected=context['arrays']['body_poses_world'][self.paths.index(self.anchor),3:]
        expected[:]=np.array([.21,-.31,.11,.91])/np.linalg.norm([.21,-.31,.11,.91])
        expected[:]*=.99999994
        bodies.poses[i,3:]=-expected*1.00000002
        report=verify_resume(context,robot,plant,bodies)
        self.assertLess(report['max_orientation_error_rad'],1e-12)
        bodies.poses[i,3:]=(Rotation.from_rotvec([0.,0.,.013])*Rotation.from_quat(expected)).as_quat()
        report=verify_resume(context,robot,plant,bodies)
        self.assertAlmostEqual(report['max_orientation_error_rad'],.013,places=12)

    def test_derived_model_requires_explicit_and_exact_leaf_revision_parent_proof(self):
        derived = self.root/'derived'; derived.mkdir()
        (derived/'scene.usda').write_text(self.scene.read_text()+'\n# approved proxy revision fixture\n')
        manifest = copy.deepcopy(self.manifest); manifest['scene_sha256'] = file_sha256(derived/'scene.usda')
        manifest['leaf_collision_geometry'] = dict(enabled=True, source_assembly=str(self.assembly),
            source_scene_sha256=self.sha, approximation='convexDecomposition',
            authored_triangles_retained=True, non_approximation_properties_preserved=True,
            source_file_sha256={str(self.scene): self.sha},
            geometry_audit=dict(world_vertex_max_abs_error_m=0., rigid_pose_max_abs_error=0., collider_pose_max_abs_error=0.),
            collider_changes=[dict(path='/World/ElasticTarget/Leaf03/Mesh', before='convexHull', after='convexDecomposition')])
        with self.assertRaisesRegex(ValueError, 'exact same'): prepare_resume(self.stage, derived, manifest, self.root/'checkpoint')
        context = prepare_resume(self.stage, derived, manifest, self.root/'checkpoint', allow_leaf_proxy_revision=True)
        self.assertEqual(context['report']['explicit_proxy_revision']['original_scene_sha256'], self.sha)
        manifest['plant']['fruits'][0]['path'] = '/World/ChangedIdentity'
        with self.assertRaisesRegex(ValueError, 'state identity'): prepare_resume(self.stage, derived, manifest, self.root/'checkpoint', allow_leaf_proxy_revision=True)

    def test_attachment_transfer_does_not_authorize_proxy_changes(self):
        before=self.stage.GetSessionLayer().ExportToString()
        for proxy_flag in ('allow_leaf_proxy_revision','allow_accuracy_proxy_revision'):
            with self.subTest(proxy_flag=proxy_flag):
                with self.assertRaisesRegex(ValueError,'authorizations cannot be combined'):
                    prepare_resume(self.stage,self.assembly,self.manifest,self.root/'checkpoint',
                        allow_attachment_transfer=True,**{proxy_flag:True})
                self.assertEqual(before,self.stage.GetSessionLayer().ExportToString())
        derived=self.root/'unproved_attachment_transfer';derived.mkdir()
        (derived/'scene.usda').write_text(self.scene.read_text()+'\n# unproved attachment transfer\n')
        manifest=copy.deepcopy(self.manifest)
        manifest['scene_sha256']=file_sha256(derived/'scene.usda')
        with self.assertRaisesRegex(ValueError,'exact checkpoint/source parent proof'):
            prepare_resume(self.stage,derived,manifest,self.root/'checkpoint',allow_attachment_transfer=True)
        self.assertEqual(before,self.stage.GetSessionLayer().ExportToString())

    def test_tool_accuracy_and_leaf_revision_permissions_are_separate(self):
        before = self.stage.GetSessionLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, 'Choose one explicit proxy revision proof'):
            prepare_resume(self.stage, self.assembly, self.manifest, self.root/'checkpoint',
                allow_leaf_proxy_revision=True, allow_accuracy_proxy_revision=True)
        self.assertEqual(before, self.stage.GetSessionLayer().ExportToString())
        derived = self.root/'unproved_tool_revision'; derived.mkdir()
        (derived/'scene.usda').write_text(self.scene.read_text()+'\n# tool cooker revision without proof\n')
        manifest = copy.deepcopy(self.manifest)
        manifest['scene_sha256'] = file_sha256(derived/'scene.usda')
        manifest['collision_proxy_accuracy'] = dict(enabled=True)
        with self.assertRaisesRegex(ValueError, 'leaf proxy revision'):
            prepare_resume(self.stage, derived, manifest, self.root/'checkpoint',
                allow_leaf_proxy_revision=True)
        self.assertEqual(before, self.stage.GetSessionLayer().ExportToString())


if __name__ == '__main__': unittest.main()
