"""Cold-stage serialization conservation controls; no Isaac/native imports."""
from pathlib import Path
import tempfile
import unittest

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from motion_materialize import materialize_replication_stage
from motion_replication import ENVS_ROOT, SOURCE_ROOTS, replicate_motion
from motion_stage_metrics import stage_metrics
from test_consolidate_background import fixture


def cloned_fixture(directory, num_envs=1, device='cpu'):
    source, _ = fixture(directory)
    stage = Usd.Stage.Open(str(source/'scene.usda'))
    UsdGeom.Xform.Define(stage, SOURCE_ROOTS[0]+'/Robot')
    link = UsdGeom.Xform.Define(stage, SOURCE_ROOTS[0]+'/Robot/link').GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(link); UsdPhysics.MassAPI.Apply(link).CreateMassAttr(2.)
    fixed = UsdGeom.Xform.Define(stage, SOURCE_ROOTS[1]+'/fixed').GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(fixed)
    UsdPhysics.MassAPI.Apply(fixed).CreateMassAttr(1.25)
    collider = UsdGeom.Cube.Define(stage, str(fixed.GetPath())+'/Collider')
    collider.CreateSizeAttr(.02); UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    attachment = UsdPhysics.FixedJoint.Define(stage, SOURCE_ROOTS[1]+'/Joints/attachment')
    attachment.CreateBody0Rel().SetTargets([link.GetPath()])
    attachment.CreateBody1Rel().SetTargets([fixed.GetPath()])
    scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1)); scene.CreateGravityMagnitudeAttr(9.81)
    stage.GetRootLayer().Save()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        stage.GetPrimAtPath('/World/ElasticTarget/Joints/Spring').CreateAttribute(
            'physxJoint:armature', Sdf.ValueTypeNames.Float).Set(.0005)
        stage.GetPrimAtPath('/World/ElasticTarget/Joints/Spring').GetAttribute(
            'drive:rotX:physics:stiffness').Set(2.4)
        scene.GetPrim().CreateAttribute('physxScene:timeStepsPerSecond', Sdf.ValueTypeNames.UInt).Set(120)
    manifest = dict(target=dict(path='/World/ElasticTarget/Fruit',
                               joint='/World/ElasticTarget/HarvestJoints/Fruit'),
                    tool=dict(belts=[]),plant=dict(bodies=[dict(path='/World/ElasticTarget/Fruit')]))
    replication = replicate_motion(stage, manifest, num_envs, device=device, native_replication=False)
    return source, stage, replication


class MaterializationTests(unittest.TestCase):
    def test_both_modes_preserve_clone_parameters_shared_instances_and_empty_session(self):
        for mode in ('layer-stack','composition'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                source, stage, replication = cloned_fixture(directory)
                source_bytes = (source/'scene.usda').read_bytes()
                output = Path(directory)/(mode+'.usdc')
                result = materialize_replication_stage(stage, output, replication, mode=mode)
                reopened = Usd.Stage.Open(str(output))
                self.assertTrue(result['preservation']['passed'])
                self.assertEqual(result['preservation']['maximum_world_matrix_error'], 0.)
                self.assertEqual(stage_metrics(reopened), stage_metrics(stage))
                self.assertEqual(result['instance_count'], 2)
                self.assertEqual(result['prototype_count'], 1)
                self.assertEqual(len(reopened.GetSessionLayer().rootPrims), 0)
                clone = ENVS_ROOT+'/env_0/ElasticTarget'
                self.assertEqual(reopened.GetPrimAtPath(clone+'/Joints/Spring').GetAttribute(
                    'physxJoint:armature').Get(), stage.GetPrimAtPath(clone+'/Joints/Spring').GetAttribute(
                    'physxJoint:armature').Get())
                self.assertAlmostEqual(reopened.GetPrimAtPath(clone+'/Joints/Spring').GetAttribute(
                    'drive:rotX:physics:stiffness').Get(), 2.4, places=6)
                self.assertEqual(reopened.GetPrimAtPath(clone+'/HarvestJoints/Fruit').GetAttribute(
                    'physics:breakForce').Get(), 3.)
                self.assertEqual(reopened.GetPrimAtPath(clone+'/HarvestJoints/Fruit').GetRelationship(
                    'physics:body1').GetTargets(), [Sdf.Path(clone+'/Fruit')])
                self.assertEqual(source_bytes, (source/'scene.usda').read_bytes())
                self.assertTrue(output.with_suffix('.materialization.json').exists())
                self.assertFalse(result['native_failure_resolution_proven'])

    def test_two_environment_joints_filters_and_independent_overrides_survive_reopen(self):
        for device in ('cpu','cuda:0'):
            with self.subTest(device=device), tempfile.TemporaryDirectory() as directory:
                _, stage, replication = cloned_fixture(directory, 2, device)
                # An env_1 opinion must not change inherited env_0 dynamics.
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    stage.GetPrimAtPath(ENVS_ROOT+'/env_1/ElasticTarget/Fruit').GetAttribute(
                        'physics:mass').Set(.031)
                result = materialize_replication_stage(stage, Path(directory)/'two.usdc',
                                                      replication,device=device)
                reopened = Usd.Stage.Open(result['file'])
                self.assertEqual(result['num_envs'], 2)
                self.assertFalse(result['native_replication_callbacks_carried'])
                self.assertEqual(len(result['relationship_scope_checks']),2)
                for i in range(2):
                    root = ENVS_ROOT+'/env_'+str(i)
                    attachment = reopened.GetPrimAtPath(root+'/ActuatedTool/Joints/attachment')
                    self.assertEqual(attachment.GetRelationship('physics:body0').GetTargets(),
                                     [Sdf.Path(root+'/HarvestRobot/Robot/link')])
                    self.assertEqual(attachment.GetRelationship('physics:body1').GetTargets(),
                                     [Sdf.Path(root+'/ActuatedTool/fixed')])
                    group = UsdPhysics.CollisionGroup(reopened.GetPrimAtPath(
                        '/World/MotionIsolation/group_'+str(i)))
                    self.assertEqual(group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets(),
                                     [Sdf.Path(root)])
                    self.assertEqual(group.GetFilteredGroupsRel().GetTargets(),
                                     [Sdf.Path('/World/MotionIsolation/group_'+str(1-i))])
                    fruit = reopened.GetPrimAtPath(root+'/ElasticTarget/Fruit')
                    self.assertAlmostEqual(fruit.GetAttribute('physics:mass').Get(), .02 if i==0 else .031, places=6)
                self.assertTrue(result['preservation']['passed'])
                self.assertEqual(result['preservation']['bodies'],8)
                self.assertEqual(result['preservation']['joints'],6)
                self.assertEqual(result['instance_count'],2)

    def test_existing_output_invalid_bounds_devices_or_gpu_callbacks_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            _, stage, replication = cloned_fixture(directory)
            output = Path(directory)/'existing.usdc'; output.write_bytes(b'original')
            with self.assertRaises(FileExistsError):
                materialize_replication_stage(stage, output, replication)
            self.assertEqual(output.read_bytes(), b'original')
            output = Path(directory)/'new.usdc'
            for count in (0,9,True):
                candidate = dict(replication, metadata=dict(replication['metadata'],num_envs=count))
                with self.assertRaisesRegex(ValueError,'1..8 environments'):
                    materialize_replication_stage(stage, output, candidate)
            for overrides in [dict(native_replication=True),dict(native_replication=0)]:
                candidate = dict(replication, metadata=dict(replication['metadata'], **overrides))
                with self.assertRaisesRegex(ValueError,'native_replication=False'):
                    materialize_replication_stage(stage, output, candidate)
            for device in ('cuda','cuda:1','CPU',1,[]):
                with self.assertRaisesRegex(ValueError,'device must'):
                    materialize_replication_stage(stage, output, replication,device=device)
            self.assertFalse(output.exists())

    def test_eight_environment_bound_preserves_every_negative_group_and_joint_scope(self):
        with tempfile.TemporaryDirectory() as directory:
            _, stage, replication = cloned_fixture(directory,8)
            result = materialize_replication_stage(stage,Path(directory)/'eight.usdc',replication)
            reopened = Usd.Stage.Open(result['file'])
            self.assertEqual(result['preservation']['bodies'],32)
            self.assertEqual(result['preservation']['joints'],24)
            self.assertEqual(len(result['relationship_scope_checks']),8)
            for i in range(8):
                group = UsdPhysics.CollisionGroup(reopened.GetPrimAtPath(
                    '/World/MotionIsolation/group_'+str(i)))
                self.assertEqual(set(group.GetFilteredGroupsRel().GetTargets()),
                                 {Sdf.Path('/World/MotionIsolation/group_'+str(j))
                                  for j in range(8) if j!=i})
                self.assertEqual(result['relationship_scope_checks'][i]['joints'],3)


if __name__ == '__main__':
    unittest.main()
