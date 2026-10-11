import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from compact_motion_assets import file_sha256, verify_compact_dependencies
from rail_ground_alignment import (BASE_BODY,CARRIER,HARVEST_ROBOT,RAIL_ANCHOR,
    RAIL_FIXED_ROOT,TOOL_BODIES,align_ground,placement_geometry,revise,_world)


def fixture():
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,'Z');UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    UsdGeom.Xform.Define(stage,HARVEST_ROBOT).AddTranslateOp().Set(Gf.Vec3d(0,1.6,0))
    for path in (BASE_BODY,CARRIER,RAIL_ANCHOR):
        body=UsdGeom.Xform.Define(stage,path);body.AddTranslateOp().Set(Gf.Vec3d(.1,-.34,.659))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim());UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(25.)
    fixed=UsdPhysics.FixedJoint.Define(stage,RAIL_FIXED_ROOT);fixed.CreateBody1Rel().SetTargets([RAIL_ANCHOR])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(.1,1.26,.659))
    for index,path in enumerate(TOOL_BODIES):
        body=UsdGeom.Xform.Define(stage,path);body.AddTranslateOp().Set(Gf.Vec3d(.2,index*.05,1.1))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        shape=UsdGeom.Cube.Define(stage,path+'/Collider');shape.CreateSizeAttr(.1)
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    mesh=UsdGeom.Mesh.Define(stage,'/World/ElasticTarget/mesh')
    mesh.CreatePointsAttr([Gf.Vec3f(0,0,0),Gf.Vec3f(1,0,0),Gf.Vec3f(0,1,1)])
    mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return stage


class RailGroundAlignmentTests(unittest.TestCase):
    def test_instanced_house_floor_seams_are_included(self):
        from rail_ground_alignment import ground_clearance,WHEEL_PATHS
        stage=fixture();UsdGeom.Xform.Define(stage,'/FloorSource')
        for name,z in [('GroundCover_1',0.),('CoverLap_1_6',.00375)]:
            mesh=UsdGeom.Mesh.Define(stage,'/FloorSource/'+name+'/Mesh')
            mesh.CreatePointsAttr([Gf.Vec3f(0,0,z),Gf.Vec3f(.1,.1,z)])
        env=UsdGeom.Xform.Define(stage,'/World/Greenhouse/House/Environment').GetPrim()
        env.GetReferences().AddInternalReference('/FloorSource');env.SetInstanceable(True)
        wheel=UsdGeom.Mesh.Define(stage,WHEEL_PATHS[0])
        wheel.CreatePointsAttr([Gf.Vec3f(0,0,.003-.659),Gf.Vec3f(.1,.1,.003-.659)])
        report=ground_clearance(stage)
        self.assertAlmostEqual(report['wheel_bottom_to_highest_ground_top_m'],-.00075,places=7)
        self.assertTrue(any('CoverLap_1_6' in row['path'] for row in report['source_mesh_world_bounds']))

    def test_clearance_includes_raised_floor_seam_not_just_flat_panels(self):
        from rail_ground_alignment import ground_clearance,GROUND_PATHS,WHEEL_PATHS
        stage=fixture()
        for path,z in [(GROUND_PATHS[0],0.),
                ('/World/Greenhouse/House/Environment/CoverLap_1_6/Mesh',.00375),
                (WHEEL_PATHS[0],.003-.659)]:
            mesh=UsdGeom.Mesh.Define(stage,path)
            mesh.CreatePointsAttr([Gf.Vec3f(0,0,z),Gf.Vec3f(.1,.1,z)])
        self.assertAlmostEqual(ground_clearance(stage)['wheel_bottom_to_highest_ground_top_m'],-.00075,places=7)  # USD mesh points are float32.

    def test_complete_robot_and_tool_translation_keeps_joint_rest_relationships_and_target(self):
        stage=fixture();before=Usd.Stage.Open(stage.Flatten())
        old_relative=np.linalg.inv(_world(before,BASE_BODY))@_world(before,TOOL_BODIES[0])
        report=align_ground(stage)
        self.assertTrue(np.allclose(report['translation_world_m'],[0,0,.003]))
        self.assertFalse(report['geometry_or_contact_parameters_changed'])
        self.assertTrue(np.allclose(old_relative,np.linalg.inv(_world(stage,BASE_BODY))@_world(stage,TOOL_BODIES[0]),atol=1e-12))
        self.assertEqual(stage.GetPrimAtPath('/World/ActuatedTool').GetTypeName(),'')
        geometry=placement_geometry(before,stage,np.array([0,0,.003]))
        self.assertEqual(geometry['unchanged_farm_target_world_pose_max_abs_error'],0.)
        self.assertLess(geometry['translated_rigid_or_collider_pose_max_abs_error'],1e-12)
        self.assertFalse(report['training_eligible'])

    def test_invalid_requests_and_world_anchor_mapping_fail_before_mutation(self):
        for lift in (0.,-.003,float('nan'),.021):
            stage=fixture();before=_world(stage,BASE_BODY)
            with self.subTest(lift=lift),self.assertRaises(ValueError):align_ground(stage,lift_m=lift)
            self.assertTrue(np.array_equal(before,_world(stage,BASE_BODY)))
        stage=fixture();UsdPhysics.Joint(stage.GetPrimAtPath(RAIL_FIXED_ROOT)).CreateBody0Rel().SetTargets([BASE_BODY])
        with self.assertRaisesRegex(ValueError,'world-to-rail-anchor'):align_ground(stage)

    def test_export_is_immutable_and_updates_rail_rest_metadata_only(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source';source.mkdir();stage=fixture()
            stage.GetRootLayer().Export(str(source/'scene.usda'));(source/'skin.npz').write_bytes(b'skin')
            digest=file_sha256(source/'scene.usda');plant={'center':[.1,.2,.3]}
            (source/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=digest,
                rail=dict(carrier_body=CARRIER),plant=plant,initial_robot_q=[.177])))
            report=revise(source,root/'new')
            manifest=json.loads((root/'new/motion_manifest.json').read_text())
            self.assertEqual(file_sha256(source/'scene.usda'),digest)
            self.assertEqual(manifest['plant'],plant);self.assertEqual(manifest['initial_robot_q'],[.177])
            self.assertEqual(manifest['rail']['source_base_position_world_m'],report['placed_base_position_world_m'])
            self.assertIsNotNone(verify_compact_dependencies(root/'new',manifest))
            self.assertEqual((root/'new/skin.npz').read_bytes(),b'skin')
            with self.assertRaises(ValueError):revise(source,root/'new')
            with self.assertRaises(ValueError):revise(root/'new',root/'additional')
            additional=revise(root/'new',root/'additional',lift_m=.002,additional_installation_lift=True)
            self.assertEqual(additional['prior_installation_alignment'],report)
            self.assertFalse(additional['continuous_with_prior_native_trial'])
            self.assertAlmostEqual(additional['placed_base_position_world_m'][2]-report['placed_base_position_world_m'][2],.002)
            new_manifest=json.loads((root/'additional/motion_manifest.json').read_text())
            new_manifest['route_checkpoint']={'checkpoint':'cannot shift a measured continuation'}
            (root/'additional/motion_manifest.json').write_text(json.dumps(new_manifest))
            with self.assertRaisesRegex(ValueError,'fresh assembly'):
                revise(root/'additional',root/'forbidden',additional_installation_lift=True)


if __name__=='__main__':unittest.main()
