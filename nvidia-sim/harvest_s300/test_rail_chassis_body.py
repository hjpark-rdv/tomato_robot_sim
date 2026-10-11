import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from compact_motion_assets import file_sha256, verify_compact_dependencies
from rail_chassis_body import (BASE_BODY, CARRIER, CARRIER_JOINT, CHASSIS, NEW_CHASSIS,
    LIFT_JOINT, ROBOT, ROOT_JOINT, RAIL_ANCHOR, RAIL_FIXED_ROOT,
    _mass_snapshot, _mapped_geometry, split_chassis_body, revise, build_native_pair_controls)
from rail_filter_parity import PLATFORM


def fixture():
    source=Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(source,1.);UsdGeom.SetStageUpAxis(source,'Z')
    UsdPhysics.SetStageKilogramsPerUnit(source,1.)
    UsdGeom.Xform.Define(source,'/World/HarvestRobot').AddTranslateOp().Set(Gf.Vec3d(0,1.6,0))
    base=UsdGeom.Xform.Define(source,BASE_BODY);base.AddTranslateOp().Set(Gf.Vec3d(0,-.34,.659))
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim());mass=UsdPhysics.MassAPI.Apply(base.GetPrim())
    mass.CreateMassAttr(25.);mass.CreateCenterOfMassAttr(Gf.Vec3f(.1,.2,.3))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.5,1.5,1.5));mass.CreatePrincipalAxesAttr(Gf.Quatf(1.))
    UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(source,PLATFORM).GetPrim())
    joint=UsdPhysics.PrismaticJoint.Define(source,LIFT_JOINT)
    joint.CreateBody0Rel().SetTargets([BASE_BODY]);joint.CreateBody1Rel().SetTargets([PLATFORM])
    joint.CreateCollisionEnabledAttr(False)
    rail=Usd.Stage.Open(source.Flatten())
    UsdPhysics.MassAPI(rail.GetPrimAtPath(BASE_BODY)).CreateMassAttr(175.)
    UsdPhysics.MassAPI(rail.GetPrimAtPath(BASE_BODY)).CreateCenterOfMassAttr(Gf.Vec3f(0.))
    UsdPhysics.MassAPI(rail.GetPrimAtPath(BASE_BODY)).CreateDiagonalInertiaAttr(Gf.Vec3f(10.,20.,30.))
    UsdPhysics.Joint(rail.GetPrimAtPath(LIFT_JOINT)).CreateCollisionEnabledAttr(True)
    for index in range(2):
        cube=UsdGeom.Cube.Define(rail,NEW_CHASSIS+f'/Part_{index}')
        cube.CreateSizeAttr(.2);cube.AddTranslateOp().Set(Gf.Vec3d(index*.3,0,0))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return source,rail


class RailChassisBodyTests(unittest.TestCase):
    def test_separate_external_constraint_and_original_mass_exact(self):
        source,rail=fixture();before=Usd.Stage.Open(rail.Flatten())
        report=split_chassis_body(rail,source)
        joint=UsdPhysics.Joint(rail.GetPrimAtPath(CARRIER_JOINT))
        self.assertTrue(joint.GetExcludeFromArticulationAttr().Get())
        self.assertFalse(joint.GetCollisionEnabledAttr().Get())
        self.assertEqual(list(map(str,joint.GetBody0Rel().GetTargets())),[BASE_BODY])
        self.assertEqual(list(map(str,joint.GetBody1Rel().GetTargets())),[CARRIER])
        self.assertTrue(rail.GetPrimAtPath(CARRIER).HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(rail.GetPrimAtPath(CARRIER).HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertEqual(_mass_snapshot(source.GetPrimAtPath(BASE_BODY)),_mass_snapshot(rail.GetPrimAtPath(BASE_BODY)))
        self.assertEqual(float(UsdPhysics.MassAPI(rail.GetPrimAtPath(CARRIER)).GetMassAttr().Get()),150.)
        self.assertEqual(report['combined_system_mass_kg'],175.)
        geometry=_mapped_geometry(before,rail)
        self.assertEqual(geometry['collider_pose_max_abs_error'],0.)
        self.assertEqual(geometry['carrier_base_rest_frame_max_abs_error'],0.)
        self.assertTrue(rail.GetPrimAtPath(CHASSIS+'/Part_0').HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(UsdPhysics.Joint(rail.GetPrimAtPath(LIFT_JOINT)).GetCollisionEnabledAttr().Get())

    def test_invalid_source_mass_or_lift_policy_prevents_mutation(self):
        for kind in ('wrong_mass','disabled_lift','bad_chassis_mass'):
            source,rail=fixture()
            if kind=='wrong_mass':UsdPhysics.MassAPI(rail.GetPrimAtPath(BASE_BODY)).CreateMassAttr(25.)
            elif kind=='disabled_lift':UsdPhysics.Joint(rail.GetPrimAtPath(LIFT_JOINT)).CreateCollisionEnabledAttr(False)
            kwargs={'chassis_mass_kg':float('nan')} if kind=='bad_chassis_mass' else {}
            with self.subTest(kind=kind),self.assertRaises(ValueError):split_chassis_body(rail,source,**kwargs)
            self.assertFalse(rail.GetPrimAtPath(CARRIER))

    def test_full_output_manifest_dependencies_and_source_immutable(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source,rail=fixture();baseline=root/'source';assembly=root/'rail';output=root/'separate'
            for directory,stage in ((baseline,source),(assembly,rail)):
                directory.mkdir();stage.GetRootLayer().Export(str(directory/'scene.usda'))
                (directory/'skin.npz').write_bytes(b'source skin')
            old=file_sha256(baseline/'scene.usda');compound=file_sha256(assembly/'scene.usda')
            (baseline/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=old)))
            (assembly/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=compound,
                rail=dict(source_assembly=str(baseline),source_scene_sha256=old,mass=dict(assumed_chassis_kg=150.)),
                target=dict(name='unchanged'))))
            report=revise(assembly,output)
            manifest=json.loads((output/'motion_manifest.json').read_text())
            self.assertEqual(manifest['rail']['carrier_body'],CARRIER)
            self.assertEqual(manifest['rail']['chassis_root'],CHASSIS)
            self.assertEqual(manifest['rail']['mass']['combined_base_kg'],25.)
            self.assertEqual(manifest['rail']['mass']['combined_system_kg'],175.)
            self.assertEqual(manifest['target'],{'name':'unchanged'})
            self.assertIsNotNone(verify_compact_dependencies(output,manifest))
            self.assertEqual(file_sha256(baseline/'scene.usda'),old)
            self.assertEqual(file_sha256(assembly/'scene.usda'),compound)
            self.assertEqual(report['geometry']['added_rigid_bodies'],1)
            self.assertEqual((output/'skin.npz').read_bytes(),b'source skin')
            with self.assertRaises(ValueError):revise(assembly,output)

    def test_native_pair_fixtures_use_exact_source_boxes_with_explicit_diagnostic_offset(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source,rail=fixture()
            UsdPhysics.Scene.Define(rail,'/World/PhysicsScene')
            anchor=UsdGeom.Xform.Define(rail,RAIL_ANCHOR);UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim())
            fixed=UsdPhysics.FixedJoint.Define(rail,RAIL_FIXED_ROOT)
            fixed.CreateBody1Rel().SetTargets([RAIL_ANCHOR]);UsdPhysics.ArticulationRootAPI.Apply(fixed.GetPrim())
            prism=UsdPhysics.PrismaticJoint.Define(rail,ROOT_JOINT)
            prism.CreateBody0Rel().SetTargets([RAIL_ANCHOR]);prism.CreateBody1Rel().SetTargets([BASE_BODY])
            deck=UsdGeom.Cube.Define(rail,NEW_CHASSIS+'/FlatRearDeck');deck.CreateSizeAttr(1.)
            deck.AddScaleOp().Set(Gf.Vec3f(.57,1.17,.012));UsdPhysics.CollisionAPI.Apply(deck.GetPrim())
            box=UsdGeom.Cube.Define(rail,PLATFORM+'/collisions/mesh_0/box');box.CreateSizeAttr(1.)
            box.AddScaleOp().Set(Gf.Vec3f(.3,.95,.025));UsdPhysics.CollisionAPI.Apply(box.GetPrim())
            baseline=root/'baseline';compound=root/'compound'
            for directory,stage in ((baseline,source),(compound,rail)):
                directory.mkdir();stage.GetRootLayer().Export(str(directory/'scene.usda'))
                (directory/'skin.npz').write_bytes(b'skin')
            old=file_sha256(baseline/'scene.usda');digest=file_sha256(compound/'scene.usda')
            (baseline/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=old)))
            (compound/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=digest,
                initial_robot_q=[.177],rail=dict(source_assembly=str(baseline),source_scene_sha256=old,
                    mass=dict(assumed_chassis_kg=150.)))))
            revise(compound,root/'carrier')
            file=root/'carrier/motion_manifest.json';manifest=json.loads(file.read_text())
            manifest['rail_ground_alignment']={'translation_world_m':[0,0,.003]}
            file.write_text(json.dumps(manifest))
            report=build_native_pair_controls(root/'carrier',root/'controls')
            left,right=report['controls']
            self.assertLess(left['positive_signed_distance_m'],0.)
            self.assertEqual(left['positive_signed_distance_m'],right['positive_signed_distance_m'])
            self.assertEqual(left['source_box_sizes_m'],right['source_box_sizes_m'])
            self.assertEqual(left['chassis_actor'],BASE_BODY);self.assertEqual(right['chassis_actor'],CARRIER)
            self.assertTrue(left['lift_collision_enabled']);self.assertTrue(right['lift_collision_enabled'])
            self.assertFalse(left['expected_native_intersection_detection'])
            self.assertTrue(right['expected_native_intersection_detection'])
            self.assertTrue(report['original_assets_unchanged']);self.assertFalse(report['training_eligible'])
            self.assertEqual(file_sha256(compound/'scene.usda'),digest)
            self.assertEqual(left['native_result'],'pending')
            with self.assertRaises(ValueError):build_native_pair_controls(root/'carrier',root/'controls')


if __name__=='__main__':unittest.main()
