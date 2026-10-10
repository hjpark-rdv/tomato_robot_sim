"""Offline USD checks for the explicit, different-dynamics truss variant."""
from pathlib import Path
import tempfile
import unittest

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from motion_resolution import lock_far_trusses


ROOT = '/World/ElasticTarget'


def inventory(stage):
    """Independent full prim inventory, including multi-apply joint attrs."""
    return {str(p.GetPath()): dict(
        type=p.GetTypeName(), schemas=p.GetAppliedSchemas(),
        attrs={a.GetName(): str(a.Get()) for a in p.GetAttributes()},
        rels={r.GetName(): [str(t) for t in r.GetTargets()] for r in p.GetRelationships()},
        custom=p.GetCustomData()) for p in stage.Traverse()}


def plant(stage):
    root = UsdGeom.Xform.Define(stage, ROOT).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    material = UsdShade.Material.Define(stage, '/World/Materials/Plant')
    UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateDynamicFrictionAttr(.7)
    names = ('Stem_MainStem_0', 'Stem_MainStem_1',
             'Truss_02_Rachis_1', 'Truss_00_Rachis_1')
    for index, name in enumerate(names):
        path = ROOT + '/' + name
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(.1 * index, 0, .5))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
        mass.CreateMassAttr(.03 + .01 * index)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.0001, .0002, .0003))
        collider = UsdGeom.Mesh.Define(stage, path + '/Collider')
        collider.CreatePointsAttr([Gf.Vec3f(0), Gf.Vec3f(.01, 0, 0), Gf.Vec3f(0, .02, 0)])
        collider.CreateFaceVertexCountsAttr([3])
        collider.CreateFaceVertexIndicesAttr([0, 1, 2])
        UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(collider.GetPrim()).CreateApproximationAttr('convexHull')
        UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(material, materialPurpose='physics')
        UsdPhysics.FilteredPairsAPI.Apply(collider.GetPrim()).CreateFilteredPairsRel().SetTargets(
            [ROOT + '/Stem_MainStem_0/Collider'])
        if index == 0:
            joint = UsdPhysics.FixedJoint.Define(stage, ROOT + '/Joints/' + name)
        else:
            joint = UsdPhysics.Joint.Define(stage, ROOT + '/Joints/' + name)
            for axis in ('rotX', 'rotY', 'rotZ'):
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
                drive.CreateStiffnessAttr(1.2 + index)
                drive.CreateDampingAttr(.08)
                drive.CreateTargetPositionAttr(0.)
                # Native original unlocked axes may be infinite.
                limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
                limit.CreateLowAttr(float('-inf'))
                limit.CreateHighAttr(float('inf'))
            joint.GetPrim().CreateAttribute('physxJoint:armature', Sdf.ValueTypeNames.Float).Set(.0005)
            joint.CreateBody0Rel().SetTargets([ROOT + '/Stem_MainStem_0'])
        joint.CreateBody1Rel().SetTargets([path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(.01, .02, .03))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    for truss in ('Truss_00', 'Truss_02'):
        fruit_path = ROOT + '/' + truss + '_Fruit_00'
        fruit = UsdGeom.Sphere.Define(stage, fruit_path)
        fruit.CreateRadiusAttr(.018)
        UsdPhysics.RigidBodyAPI.Apply(fruit.GetPrim())
        UsdPhysics.CollisionAPI.Apply(fruit.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(fruit.GetPrim()).CreateMassAttr(.025)
        joint = UsdPhysics.FixedJoint.Define(stage, ROOT + '/Joints/Harvest_' + truss + '_Fruit_00')
        joint.CreateBody0Rel().SetTargets([ROOT + '/' + truss + '_Rachis_1'])
        joint.CreateBody1Rel().SetTargets([fruit_path])
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateBreakForceAttr(3.)
        joint.CreateBreakTorqueAttr(.08)
    return dict(plant=dict(root=ROOT), target=dict(
        path=ROOT + '/Truss_02_Fruit_00', name='Truss_02_Fruit_00',
        anchor=ROOT + '/Truss_02_Rachis_1'))


class ResolutionTests(unittest.TestCase):
    def test_only_far_rotational_limits_change_and_source_file_survives(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'source.usda'
            authored = Usd.Stage.CreateNew(str(path))
            manifest = plant(authored)
            authored.GetRootLayer().Save()
            source = path.read_bytes()
            stage = Usd.Stage.Open(str(path))
            before = inventory(stage)
            result = lock_far_trusses(stage, manifest)
            after = inventory(stage)
            far = ROOT + '/Joints/Truss_00_Rachis_1'
            expected = {f'limit:{axis}:physics:{suffix}'
                        for axis in ('rotX', 'rotY', 'rotZ') for suffix in ('low', 'high')}
            changed = {name for name in before[far]['attrs']
                       if before[far]['attrs'][name] != after[far]['attrs'][name]}
            self.assertEqual(changed, expected)
            for name in expected:
                self.assertEqual(float(after[far]['attrs'][name]),
                                 1. if name.endswith(':low') else -1.)
                after[far]['attrs'][name] = before[far]['attrs'][name]
            # This includes meshes, mass/inertia, drives, armature, filters,
            # materials and both native breakable harvest joints.
            self.assertEqual(before, after)
            self.assertEqual(result['affected_joint_count'], 1)
            self.assertEqual(result['removed_rotational_dofs'], 3)
            self.assertEqual(result['original_articulation_rotational_dofs'], 9)
            self.assertEqual(result['expected_articulation_rotational_dofs'], 6)
            self.assertEqual(result['target_truss_prefix'], 'Truss_02_')
            self.assertTrue(result['different_dynamics'])
            self.assertFalse(result['training_eligible'])
            self.assertIsNone(result['harvest_success'])
            self.assertEqual(path.read_bytes(), source)
            self.assertEqual(stage.GetRootLayer().ExportToString(), authored.GetRootLayer().ExportToString())
            session = stage.GetSessionLayer().ExportToString()
            self.assertIn('Truss_00_Rachis_1', session)
            self.assertNotIn('Truss_02_Rachis_1', session)
            self.assertNotIn('Harvest_', session)

    def test_wrong_target_identity_rejects_without_mutation(self):
        stage = Usd.Stage.CreateInMemory()
        manifest = plant(stage)
        manifest['target']['anchor'] = ROOT + '/Truss_00_Rachis_1'
        before = inventory(stage)
        with self.assertRaisesRegex(ValueError, 'different truss identities'):
            lock_far_trusses(stage, manifest)
        self.assertEqual(inventory(stage), before)
        self.assertTrue(stage.GetSessionLayer().empty)

    def test_incomplete_or_prelocked_spring_rejects_before_any_override(self):
        for malformed in ('incomplete', 'prelocked'):
            with self.subTest(malformed=malformed):
                stage = Usd.Stage.CreateInMemory()
                manifest = plant(stage)
                far = stage.GetPrimAtPath(ROOT + '/Joints/Truss_00_Rachis_1')
                if malformed == 'incomplete':
                    far.RemoveAPI(UsdPhysics.DriveAPI, 'rotZ')
                else:
                    limit = UsdPhysics.LimitAPI(far, 'rotY')
                    limit.GetLowAttr().Set(1.)
                    limit.GetHighAttr().Set(-1.)
                before = inventory(stage)
                with self.assertRaises(ValueError):
                    lock_far_trusses(stage, manifest)
                self.assertEqual(inventory(stage), before)
                self.assertTrue(stage.GetSessionLayer().empty)

    def test_repeated_application_does_not_reclassify_locked_axes(self):
        stage = Usd.Stage.CreateInMemory()
        manifest = plant(stage)
        lock_far_trusses(stage, manifest)
        before = inventory(stage)
        with self.assertRaisesRegex(ValueError, 'original unlocked spring axis'):
            lock_far_trusses(stage, manifest)
        self.assertEqual(inventory(stage), before)


if __name__ == '__main__':
    unittest.main()
