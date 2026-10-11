import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from motion_assets import rigid, mesh
from rail_assets import (BASE_BODY, NEW_CHASSIS, OLD_CHASSIS, ROOT_JOINT, RAIL_ANCHOR, RAIL_FIXED_ROOT,
                         convert, enabled_colliders, near_side_inventory, revise, sha, world_meshes)


def fixture():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,'Z')
    UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
    x = UsdGeom.Xform.Define(stage,'/World/HarvestRobot')
    x.AddTranslateOp().Set(Gf.Vec3d(0,1.6,0))
    pose = np.eye(4);pose[:3,3] = [0,-.34,.659]
    rigid(stage,BASE_BODY,pose,25.,[.5,1.5,1.5])
    UsdPhysics.MassAPI(stage.GetPrimAtPath(BASE_BODY)).CreatePrincipalAxesAttr(Gf.Quatf(1.))
    joint = UsdPhysics.FixedJoint.Define(stage,ROOT_JOINT)
    joint.CreateBody1Rel().SetTargets([BASE_BODY])
    UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())
    joint.GetPrim().CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(True)
    UsdGeom.Xform.Define(stage,OLD_CHASSIS)
    vertices = np.array([[-.3,-.6,.1],[.3,-.6,.1],[.3,.6,.1],[-.3,.6,.2]])
    faces = np.array([[0,1,2],[0,2,3]])
    mesh(stage,OLD_CHASSIS+'/Part_00',vertices,faces,(.1,.3,.2),approximation='none')
    UsdGeom.Xform.Define(stage,'/World/Greenhouse')
    mesh(stage,'/World/Greenhouse/Obstacle',vertices+10,faces,(.5,.5,.5),approximation='none')
    UsdGeom.Xform.Define(stage,'/World/ElasticTarget')
    return stage


class RailAssetsTests(unittest.TestCase):
    def test_physical_axis_frames_and_exact_geometry(self):
        stage = fixture();before=world_meshes(stage);colliders=enabled_colliders(stage)
        report = convert(stage)
        joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(ROOT_JOINT))
        self.assertTrue(joint)
        self.assertFalse(joint.GetPrim().HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertTrue(stage.GetPrimAtPath(RAIL_FIXED_ROOT).HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertEqual(list(map(str,joint.GetBody0Rel().GetTargets())),[RAIL_ANCHOR])
        self.assertTrue(stage.GetPrimAtPath(RAIL_ANCHOR).HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(stage.GetPrimAtPath(RAIL_ANCHOR).HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(joint.GetPrim().GetAttribute('physxArticulation:enabledSelfCollisions').Get())
        self.assertEqual(joint.GetAxisAttr().Get(),'Y')
        np.testing.assert_allclose(joint.GetLocalPos0Attr().Get(),[0,0,0],atol=1e-7)
        np.testing.assert_allclose(UsdPhysics.Joint(stage.GetPrimAtPath(RAIL_FIXED_ROOT)).GetLocalPos0Attr().Get(),[0,1.26,.659],atol=1e-7)
        self.assertEqual(joint.GetLocalRot0Attr().Get(),Gf.Quatf(1.))
        self.assertAlmostEqual(joint.GetLowerLimitAttr().Get(),-1.6,places=6)
        self.assertAlmostEqual(joint.GetUpperLimitAttr().Get(),3.4,places=6)
        self.assertEqual(report['added_dofs'],1)
        self.assertEqual(report['mass']['combined_base_kg'],175.)
        self.assertTrue(np.all(np.asarray(report['mass']['principal_inertia_kg_m2'])>0))
        after = world_meshes(stage)
        np.testing.assert_allclose(before[OLD_CHASSIS+'/Part_00'],after[NEW_CHASSIS+'/Part_00'],atol=1e-12)
        np.testing.assert_array_equal(before['/World/Greenhouse/Obstacle'],after['/World/Greenhouse/Obstacle'])
        self.assertEqual(stage.GetPrimAtPath('/World/Greenhouse/Obstacle').GetAttribute('physics:approximation').Get(),'none')
        self.assertEqual(stage.GetPrimAtPath(NEW_CHASSIS+'/Part_00').GetAttribute('physics:approximation').Get(),'convexDecomposition')
        self.assertEqual(enabled_colliders(stage),{p.replace(OLD_CHASSIS,NEW_CHASSIS) for p in colliders})

    def test_invalid_source_or_range_cannot_be_silently_converted(self):
        for kwargs in ({'rail_y_range':(5.,0.)},{'rail_y_range':(2.,5.)},{'chassis_mass_kg':float('nan')},{'chassis_mass_kg':0.}):
            with self.assertRaises(ValueError):convert(fixture(),**kwargs)
        stage=fixture();convert(stage)
        with self.assertRaises(ValueError):convert(stage)

    def test_mounted_robot_parent_does_not_double_translate_fixed_anchor(self):
        stage=fixture()
        parent=UsdGeom.Xform.Define(stage,'/World/HarvestRobot/Robot')
        parent.AddTranslateOp().Set(Gf.Vec3d(0,-.34,.659))
        before=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(BASE_BODY)))
        convert(stage)
        anchor=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(RAIL_ANCHOR)))
        np.testing.assert_allclose(anchor,before,atol=1e-12)

    def test_base_rotation_does_not_rotate_the_rail_axis(self):
        from scipy.spatial.transform import Rotation
        stage=fixture();base=stage.GetPrimAtPath(BASE_BODY)
        pose=np.eye(4);pose[:3,3]=[0,-.34,.659]
        pose[:3,:3]=Rotation.from_euler('z',35,degrees=True).as_matrix()
        UsdGeom.Xformable(base).MakeMatrixXform().Set(Gf.Matrix4d(pose.T.tolist()))
        base.CreateRelationship('test:chassis_part').SetTargets([OLD_CHASSIS+'/Part_00'])
        convert(stage)
        j=UsdPhysics.Joint(stage.GetPrimAtPath(ROOT_JOINT));q=j.GetLocalRot1Attr().Get()
        local=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix()
        np.testing.assert_allclose(pose[:3,:3]@local@np.array([0,1,0]),[0,1,0],atol=1e-7)
        self.assertEqual(list(map(str,base.GetRelationship('test:chassis_part').GetTargets())),[NEW_CHASSIS+'/Part_00'])

    def test_source_asset_unchanged_and_output_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source';source.mkdir();out=root/'out';stage=fixture()
            stage.GetRootLayer().Export(str(source/'scene.usda'))
            digest=sha(source/'scene.usda')
            (source/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=digest,initial_robot_q=[0.]*7)))
            np.savez(source/'skin.npz',dummy=[1])
            revise(source,out)
            self.assertEqual(sha(source/'scene.usda'),digest)
            manifest=json.loads((out/'motion_manifest.json').read_text())
            self.assertEqual(manifest['initial_robot_q'],[0.]*7)
            self.assertEqual(manifest['rail']['native_validation'],'pending')
            self.assertEqual(sha(out/'scene.usda'),manifest['scene_sha256'])
            with self.assertRaises(ValueError):revise(source,out)

    def test_near_roots_do_not_claim_placement_reachability(self):
        stage=fixture();verts=np.array([[0,0,0],[.1,0,0],[0,.1,0]])
        placements=[]
        for i,x in enumerate([-.68,-.88,.68,.88]):
            path=f'/World/Greenhouse/Plants/Plant_{i:02d}/Truss_00'
            root=UsdGeom.Xform.Define(stage,path);root.AddTranslateOp().Set(Gf.Vec3d(x,1,.8))
            m=mesh(stage,path+'/fruit',verts,[[0,1,2]],(.8,0,0))
            m.GetPrim().SetCustomDataByKey('sourceName','Fruit_01')
            placements.append(dict(id=i,position=[x,1,.32],gutter_center_x=-.775 if x<0 else .775,
                side=1 if i in [0,3] else -1,stem_variant=0,trusses=[dict(source_glb='test.glb',y_deg=10)]))
        inventory=near_side_inventory(stage,dict(neighbor_placements=placements))
        self.assertEqual(inventory['side_selected_trusses'],2)
        self.assertEqual([r['plant_id'] for r in inventory['trusses'] if r['rail_near']],[0,2])
        self.assertTrue(all(r['reachability'].startswith('unknown') for r in inventory['trusses']))
        self.assertEqual([r['fruit_count'] for r in inventory['trusses']],[1]*4)


if __name__=='__main__':unittest.main()
