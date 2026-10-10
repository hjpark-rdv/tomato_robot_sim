"""Offline regression checks for preservation of articulation dynamics."""
import unittest

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from motion_replication import SOURCE_ROOTS, _assert_relationship_scope, _compare, _migrate_dynamic_groups, _snapshot, remap_path, replicate_motion


class PhysicsConservationTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(self.stage, '/World/Plant').GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(root)
        root.CreateAttribute('physxArticulation:solverPositionIterationCount',
                             Sdf.ValueTypeNames.Int).Set(64)
        body = UsdGeom.Xform.Define(self.stage, '/World/Plant/body').GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        self.joint = UsdPhysics.Joint.Define(self.stage, '/World/Plant/joint')
        self.joint.CreateBody1Rel().SetTargets([body.GetPath()])
        self.drive = UsdPhysics.DriveAPI.Apply(self.joint.GetPrim(), 'rotX')
        self.drive.CreateStiffnessAttr(13.0)
        self.drive.CreateDampingAttr(2.0)
        self.limit = UsdPhysics.LimitAPI.Apply(self.joint.GetPrim(), 'rotY')
        self.limit.CreateLowAttr(-30.0)
        self.limit.CreateHighAttr(30.0)
        self.reference = _snapshot(self.stage, ('/World/Plant',))

    def test_changed_drive_stiffness_is_rejected(self):
        self.drive.GetStiffnessAttr().Set(14.0)
        with self.assertRaisesRegex(RuntimeError, 'attrs differs'):
            _compare(self.reference, _snapshot(self.stage, ('/World/Plant',)), 'clone')

    def test_changed_joint_limit_is_rejected(self):
        self.limit.GetHighAttr().Set(45.0)
        with self.assertRaisesRegex(RuntimeError, 'attrs differs'):
            _compare(self.reference, _snapshot(self.stage, ('/World/Plant',)), 'clone')

    def test_changed_articulation_solver_is_rejected(self):
        self.stage.GetPrimAtPath('/World/Plant').GetAttribute(
            'physxArticulation:solverPositionIterationCount').Set(32)
        with self.assertRaisesRegex(RuntimeError, 'attrs differs'):
            _compare(self.reference, _snapshot(self.stage, ('/World/Plant',)), 'clone')

    def test_missing_filtered_pair_schema_is_rejected_even_with_same_targets(self):
        body = self.stage.GetPrimAtPath('/World/Plant/body')
        api = UsdPhysics.FilteredPairsAPI.Apply(body)
        api.CreateFilteredPairsRel().SetTargets(['/World/Plant/joint'])
        reference = _snapshot(self.stage, ('/World/Plant',))
        body.RemoveAPI(UsdPhysics.FilteredPairsAPI)
        with self.assertRaisesRegex(RuntimeError, 'applied_schemas differs'):
            _compare(reference, _snapshot(self.stage, ('/World/Plant',)), 'clone')

    def test_property_remapping_preserves_neighbor_namespaces(self):
        mapping = {'/World/Plant': '/World/Env/Plant'}
        self.assertEqual(remap_path('/World/Plant.drive:rotX:physics:stiffness', mapping),
                         '/World/Env/Plant.drive:rotX:physics:stiffness')
        self.assertEqual(remap_path('/World/PlantOther/body', mapping), '/World/PlantOther/body')

    def test_fixture_policy_is_preserved_without_duplicate_membership(self):
        active = UsdPhysics.CollisionGroup.Define(self.stage, '/World/FixtureGroups/Active')
        active.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(list(SOURCE_ROOTS))
        background = UsdPhysics.CollisionGroup.Define(self.stage, '/World/FixtureGroups/Background')
        background.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/Greenhouse/Plants'])
        background.CreateFilteredGroupsRel().SetTargets([active.GetPath()])
        envs = ['/World/MotionEnvs/env_0', '/World/MotionEnvs/env_1']
        groups = []
        for i, env in enumerate(envs):
            group = UsdPhysics.CollisionGroup.Define(self.stage, f'/World/MotionIsolation/group_{i}')
            group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([env])
            group.CreateFilteredGroupsRel().SetTargets([f'/World/MotionIsolation/group_{1-i}'])
            groups.append(group)
        _migrate_dynamic_groups(self.stage, envs)
        self.assertEqual(active.GetCollidersCollectionAPI().GetIncludesRel().GetTargets(), [])
        self.assertEqual(background.GetFilteredGroupsRel().GetTargets(), [g.GetPath() for g in groups])
        for i, group in enumerate(groups):
            self.assertEqual(group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets(),
                             [Sdf.Path(envs[i])])
            self.assertEqual(group.GetFilteredGroupsRel().GetTargets(), [groups[1-i].GetPath()])

    def test_partial_existing_policy_requires_explicit_adapter(self):
        active = UsdPhysics.CollisionGroup.Define(self.stage, '/World/PartialGroup')
        active.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([SOURCE_ROOTS[0]])
        with self.assertRaisesRegex(ValueError, 'partial/mixed dynamic collision group'):
            _migrate_dynamic_groups(self.stage, ['/World/MotionEnvs/env_0'])

    def test_original_filtered_pair_reference_is_rejected(self):
        env = '/World/MotionEnvs/env_0'
        body = UsdGeom.Xform.Define(self.stage, env + '/ElasticTarget/body').GetPrim()
        UsdPhysics.FilteredPairsAPI.Apply(body).CreateFilteredPairsRel().SetTargets(
            ['/World/ElasticTarget/neighbor'])
        with self.assertRaisesRegex(RuntimeError, 'original dynamic root'):
            _assert_relationship_scope(self.stage, env)

    def test_filtered_pair_scope_counts_exact_mapped_targets(self):
        env = '/World/MotionEnvs/env_0'
        body = UsdGeom.Xform.Define(self.stage, env + '/ElasticTarget/body').GetPrim()
        UsdPhysics.FilteredPairsAPI.Apply(body).CreateFilteredPairsRel().SetTargets(
            [env + '/ElasticTarget/neighbor'])
        result = _assert_relationship_scope(self.stage, env)
        self.assertEqual(result['filtered_pair_targets'], 1)

    def test_reference_variant_preserves_session_physics_overrides(self):
        stage = Usd.Stage.CreateInMemory()
        for root in SOURCE_ROOTS:
            UsdGeom.Xform.Define(stage, root)
        UsdGeom.Xform.Define(stage, '/World/PhysicsScene')
        body = UsdGeom.Xform.Define(stage, '/World/ElasticTarget/fruit').GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        joint = UsdPhysics.FixedJoint.Define(stage, '/World/ElasticTarget/joint')
        joint.CreateBody1Rel().SetTargets([body.GetPath()])
        joint.CreateBody0Rel().SetTargets(['/World/ActuatedTool/fixed'])
        material = UsdGeom.Xform.Define(stage, '/World/Looks/PhysicalMaterial').GetPrim()
        body.CreateRelationship('material:binding:physics').SetTargets([material.GetPath()])
        joint.CreateBreakForceAttr(3.0)
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            joint.GetBreakForceAttr().Set(2.0)
            UsdPhysics.FilteredPairsAPI.Apply(body).CreateFilteredPairsRel().SetTargets(
                ['/World/ActuatedTool/fixed'])
        original_root = stage.GetRootLayer().ExportToString()
        manifest = {'target': {'path': str(body.GetPath()), 'joint': str(joint.GetPath())},
                    'tool': {'belts': []}, 'plant': {'bodies': [{'path': str(body.GetPath())}]}}
        result = replicate_motion(stage, manifest, 1, device='cpu', source_copy='reference')
        target = result['environments'][0]['target']
        copied_joint = stage.GetPrimAtPath(result['environments'][0]['target_joint'])
        self.assertEqual(copied_joint.GetAttribute('physics:breakForce').Get(), 2.0)
        self.assertEqual(copied_joint.GetRelationship('physics:body1').GetTargets(), [Sdf.Path(target)])
        self.assertEqual(copied_joint.GetRelationship('physics:body0').GetTargets(),
                         [Sdf.Path('/World/MotionEnvs/env_0/ActuatedTool/fixed')])
        self.assertEqual(stage.GetPrimAtPath(target).GetRelationship('material:binding:physics').GetTargets(),
                         [material.GetPath()])
        self.assertEqual(stage.GetRootLayer().ExportToString(), original_root)
        self.assertEqual(result['metadata']['source_copy'], 'reference')
        self.assertFalse(stage.GetPrimAtPath('/World/ElasticTarget').IsActive())
        self.assertTrue(stage.GetPrimAtPath(target).IsActive())


if __name__ == '__main__':
    unittest.main()
