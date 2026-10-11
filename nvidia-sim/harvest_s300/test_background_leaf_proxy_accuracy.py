import copy
import unittest

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from background_foliage import apply_scope
from background_leaf_proxy_accuracy import (EXACT_MAPPING, EXACT_PATHS, EXACT_SOURCE_STEM,
                                            apply_accuracy, audit)
from collision_proxy_accuracy import API, CONFIG, PREFIX


def fixture():
    stage = Usd.Stage.CreateInMemory(); UsdGeom.Xform.Define(stage, '/World')
    stem = UsdGeom.Xform.Define(stage, EXACT_SOURCE_STEM)
    stem.AddTranslateOp().Set(Gf.Vec3d(.1, .2, .3))
    for index, name in [(0, 'MainStem')] + [(i, f'CompoundLeaf_{1 if i < 131 else 2:02}') for i in range(127, 135)]:
        mesh = UsdGeom.Mesh.Define(stage, EXACT_SOURCE_STEM + '/mesh_' + str(index))
        mesh.CreatePointsAttr([[0, 0, 0], [.03, 0, 0], [0, .01, 0], [0, 0, .002]])
        mesh.CreateFaceVertexCountsAttr([3] * 4)
        mesh.CreateFaceVertexIndicesAttr([0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3])
        mesh.GetPrim().SetCustomDataByKey('sourceName', name)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
        mesh.GetPrim().CreateAttribute('physxCollision:contactOffset', Sdf.ValueTypeNames.Float).Set(.0005)
        mesh.GetPrim().CreateRelationship('material:binding:physics').SetTargets(['/World/Material'])
    material = UsdPhysics.MaterialAPI.Apply(UsdGeom.Scope.Define(stage, '/World/Material').GetPrim())
    material.CreateStaticFrictionAttr(.8); material.CreateDynamicFrictionAttr(.7)
    scope = dict(critical_neighbor_leaf_groups=[dict(stem=EXACT_SOURCE_STEM,
        group=f'CompoundLeaf_{group:02}', exact_intersected_meshes=[EXACT_SOURCE_STEM + f'/mesh_{index}'])
        for group, index in ((1, 127), (2, 131))])
    report = apply_scope(stage, scope)
    return stage, dict(background_compliant_foliage=report)


class BackgroundLeafAccuracyTest(unittest.TestCase):
    def test_exact_eight_only_geometry_mass_material_filter_and_drive_preserved(self):
        source, manifest = fixture(); derived = Usd.Stage.Open(source.Flatten())
        report = apply_accuracy(derived, manifest)
        self.assertEqual(report['exact_collider_paths'], EXACT_PATHS)
        self.assertEqual(report['geometry_audit']['world_vertex_max_abs_error_m'], 0.)
        self.assertEqual(report['geometry_audit']['rigid_pose_max_abs_error'], 0.)
        self.assertTrue(report['non_cooking_properties_preserved'])
        self.assertFalse(report['cad_fixture_certificate_applied'])
        self.assertFalse(report['full_source_triangle_enclosure_proven'])
        self.assertEqual(report['native_stability_validation'], 'pending')
        self.assertEqual(report['penetration_limit_m'], .0005)
        self.assertFalse(source.GetPrimAtPath(EXACT_PATHS[0]).HasAttribute(PREFIX + 'maxConvexHulls'))
        for path in EXACT_PATHS:
            self.assertEqual(derived.GetPrimAtPath(path).GetAttribute(PREFIX + 'maxConvexHulls').Get(), CONFIG['maxConvexHulls'])

    def test_wrong_mapping_or_partial_scope_rejected_before_mutation(self):
        stage, manifest = fixture(); before = stage.GetRootLayer().ExportToString()
        manifest['background_compliant_foliage']['path_mapping'].pop(next(iter(EXACT_MAPPING)))
        with self.assertRaisesRegex(ValueError, 'eight original'): apply_accuracy(stage, manifest)
        self.assertEqual(stage.GetRootLayer().ExportToString(), before)

    def test_static_or_undeclared_leaf_is_not_eligible(self):
        stage, manifest = fixture()
        body = stage.GetPrimAtPath('/World/BackgroundCompliantFoliage/Plant_20/Leaf_CompoundLeaf_01')
        UsdPhysics.RigidBodyAPI(body).CreateKinematicEnabledAttr(True)
        with self.assertRaisesRegex(ValueError, 'dynamic'): apply_accuracy(stage, manifest)
        self.assertFalse(stage.GetPrimAtPath(EXACT_PATHS[0]).HasAttribute(PREFIX + 'maxConvexHulls'))

    def test_hidden_friction_filter_joint_or_offset_change_rejected(self):
        source, manifest = fixture()
        mutations = [
            lambda s: UsdPhysics.MaterialAPI(s.GetPrimAtPath('/World/Material')).GetDynamicFrictionAttr().Set(.1),
            lambda s: s.GetPrimAtPath(EXACT_PATHS[0]).CreateRelationship('physics:filteredPairs').SetTargets(['/World/Other']),
            lambda s: s.GetPrimAtPath(manifest['background_compliant_foliage']['leaves'][0]['joint']).GetAttribute('drive:rotX:physics:stiffness').Set(.1),
            lambda s: s.GetPrimAtPath(EXACT_PATHS[0]).GetAttribute('physxCollision:contactOffset').Set(.01),
        ]
        for change in mutations:
            with self.subTest(change=change):
                derived = Usd.Stage.Open(source.Flatten()); apply_accuracy(derived, manifest); change(derived)
                with self.assertRaisesRegex(ValueError, 'Properties changed'): audit(source, derived, manifest)

    def test_added_or_removed_schema_cannot_hide_in_normalization(self):
        source, manifest = fixture(); derived = Usd.Stage.Open(source.Flatten()); apply_accuracy(derived, manifest)
        prim = derived.GetPrimAtPath(EXACT_PATHS[0]); UsdPhysics.MassAPI.Apply(prim)
        with self.assertRaisesRegex(ValueError, 'schemas changed'): audit(source, derived, manifest)
        tokens = prim.GetMetadata('apiSchemas').ApplyOperations([])
        tokens.remove('PhysicsMassAPI'); tokens.remove('PhysicsCollisionAPI')
        prim.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(tokens))
        with self.assertRaises(ValueError): audit(source, derived, manifest)

    def test_wrong_cooking_attribute_detected(self):
        source, manifest = fixture(); derived = Usd.Stage.Open(source.Flatten()); apply_accuracy(derived, manifest)
        derived.GetPrimAtPath(EXACT_PATHS[0]).GetAttribute(PREFIX + 'errorPercentage').Set(20.)
        with self.assertRaisesRegex(ValueError, 'configuration'): audit(source, derived, manifest)


if __name__ == '__main__': unittest.main()
