import json
import tempfile
import unittest
from pathlib import Path
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from compact_motion_assets import compare_geometry, file_sha256, verify_compact_dependencies
from motion_collision_groups import _filtered_pairs
from rail_filter_parity import (BASE_BODY, LIFT_JOINT, NEW_CHASSIS, OLD_CHASSIS,
                                PLATFORM, restore_platform_chassis_collision, revise)


def fixture(base_shape=False):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,'Z')
    UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    for path in (BASE_BODY, PLATFORM):
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,path).GetPrim())
    def collider(path):
        prim = UsdGeom.Cube.Define(stage,path).GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
    if base_shape:collider(BASE_BODY+'/original_guide')
    for i in range(2):
        collider(OLD_CHASSIS+f'/Part_{i}')
        collider(PLATFORM+f'/collider_{i}')
    joint = UsdPhysics.PrismaticJoint.Define(stage,LIFT_JOINT)
    joint.CreateBody0Rel().SetTargets([BASE_BODY])
    joint.CreateBody1Rel().SetTargets([PLATFORM])
    joint.CreateCollisionEnabledAttr(False)
    rail = Usd.Stage.Open(stage.Flatten())
    editor = Usd.NamespaceEditor(rail)
    editor.ReparentPrim(rail.GetPrimAtPath(OLD_CHASSIS),rail.GetPrimAtPath(BASE_BODY),'S300Chassis')
    if not editor.ApplyEdits():raise ValueError('Fixture reparent failed')
    return stage,rail


class RailFilterParityTests(unittest.TestCase):
    def test_zero_original_base_shapes_enable_every_chassis_platform_pair(self):
        source,rail = fixture()
        original = Usd.Stage.Open(rail.Flatten())
        report = restore_platform_chassis_collision(rail,source)
        self.assertTrue(UsdPhysics.Joint(rail.GetPrimAtPath(LIFT_JOINT)).GetCollisionEnabledAttr().Get())
        self.assertEqual(report['newly_collision_enabled_chassis_platform_shape_pairs'],4)
        self.assertEqual(report['added_exact_filtered_shape_pairs'],[])
        self.assertEqual(report['original_base_colliders'],[])
        self.assertEqual(compare_geometry(original,rail)['collider_pose_max_abs_error'],0.)
        self.assertFalse(UsdPhysics.Joint(source.GetPrimAtPath(LIFT_JOINT)).GetCollisionEnabledAttr().Get())

    def test_only_exact_original_guide_pairs_remain_filtered(self):
        source,rail = fixture(True)
        existing = UsdPhysics.FilteredPairsAPI.Apply(rail.GetPrimAtPath(NEW_CHASSIS+'/Part_0')).CreateFilteredPairsRel()
        existing.SetTargets([NEW_CHASSIS+'/Part_1'])
        report = restore_platform_chassis_collision(rail,source)
        self.assertEqual(report['restored_old_base_shape_pairs'],2)
        self.assertEqual(report['newly_collision_enabled_chassis_platform_shape_pairs'],4)
        self.assertEqual(list(map(str,existing.GetTargets())),[NEW_CHASSIS+'/Part_1'])
        targets=UsdPhysics.FilteredPairsAPI(rail.GetPrimAtPath(BASE_BODY+'/original_guide')).GetFilteredPairsRel().GetTargets()
        self.assertEqual(set(map(str,targets)),{PLATFORM+'/collider_0',PLATFORM+'/collider_1'})
        all_shapes=report['chassis_colliders']+report['original_base_colliders']+report['platform_colliders']
        pairs=_filtered_pairs(rail,all_shapes)
        self.assertFalse(any(NEW_CHASSIS in a and PLATFORM in b or PLATFORM in a and NEW_CHASSIS in b for a,b in pairs))

    def test_invalid_mapping_does_not_change_joint_policy(self):
        for error in ('wrong_body','missing_chassis','unexpected_source_dynamic'):
            with self.subTest(error=error):
                source,rail=fixture()
                if error=='wrong_body':UsdPhysics.Joint(rail.GetPrimAtPath(LIFT_JOINT)).CreateBody1Rel().SetTargets(['/Wrong'])
                elif error=='missing_chassis':rail.RemovePrim(NEW_CHASSIS+'/Part_1')
                else:UsdPhysics.RigidBodyAPI.Apply(source.GetPrimAtPath(OLD_CHASSIS))
                with self.assertRaises(ValueError):restore_platform_chassis_collision(rail,source)
                self.assertFalse(UsdPhysics.Joint(rail.GetPrimAtPath(LIFT_JOINT)).GetCollisionEnabledAttr().Get())

    def test_immutable_derived_output_and_dependency_hash(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source,rail=fixture(True)
            baseline=root/'baseline';assembly=root/'rail';output=root/'fixed'
            for directory,stage in ((baseline,source),(assembly,rail)):
                directory.mkdir();stage.GetRootLayer().Export(str(directory/'scene.usda'))
                (directory/'skin.npz').write_bytes(b'unchanged source skin')
            baseline_sha=file_sha256(baseline/'scene.usda')
            rail_sha=file_sha256(assembly/'scene.usda')
            (baseline/'motion_manifest.json').write_text(json.dumps({'scene_sha256':baseline_sha}))
            (assembly/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=rail_sha,
                rail=dict(source_assembly=str(baseline),source_scene_sha256=baseline_sha),target={'name':'unchanged'})))
            report=revise(assembly,output)
            self.assertEqual(report['newly_collision_enabled_chassis_platform_shape_pairs'],4)
            self.assertEqual(file_sha256(baseline/'scene.usda'),baseline_sha)
            self.assertEqual(file_sha256(assembly/'scene.usda'),rail_sha)
            self.assertEqual((output/'skin.npz').read_bytes(),b'unchanged source skin')
            manifest=json.loads((output/'motion_manifest.json').read_text())
            self.assertEqual(manifest['target'],{'name':'unchanged'})
            self.assertIsNotNone(verify_compact_dependencies(output,manifest))
            self.assertEqual(manifest['rail_filter_parity']['native_validation'],'pending exact pair controls and initial-state A/B')
            with self.assertRaises(ValueError):revise(assembly,output)
            with (output/'composed.usdc').open('ab') as stream:stream.write(b'corrupt')
            with self.assertRaises(ValueError):verify_compact_dependencies(output,manifest)


if __name__=='__main__':unittest.main()
