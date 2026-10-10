"""Display updates must preserve physical collision inventory and skin geometry."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf, Tf, Usd, UsdGeom, UsdPhysics, Vt
from scipy.spatial.transform import Rotation

from motion_visuals import DeformationVisuals, prepare_display_meshes
from elastic_geometry import skin_points


def mesh(stage,path,enabled=None):
    result=UsdGeom.Mesh.Define(stage,path)
    result.CreatePointsAttr([Gf.Vec3f(0),Gf.Vec3f(1,0,0),Gf.Vec3f(0,1,0)])
    result.CreateExtentAttr([Gf.Vec3f(0),Gf.Vec3f(1,1,0)])
    result.CreateFaceVertexCountsAttr([3]);result.CreateFaceVertexIndicesAttr([0,1,2])
    if enabled is not None:
        UsdPhysics.CollisionAPI.Apply(result.GetPrim()).CreateCollisionEnabledAttr(enabled)
        UsdPhysics.MeshCollisionAPI.Apply(result.GetPrim()).CreateApproximationAttr('convexHull')
    return result


def active_collision_state(stage):
    return {str(prim.GetPath()):
            {attribute.GetName():str(attribute.Get()) for attribute in prim.GetAttributes()}
            for prim in stage.Traverse()
            if prim.HasAPI(UsdPhysics.CollisionAPI) and
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False}


class DisplayTests(unittest.TestCase):
    def test_dormant_schema_removal_is_scoped_and_session_only(self):
        stage=Usd.Stage.CreateInMemory()
        expected='/World/ElasticTarget/Skin/mesh_0'
        mesh(stage,expected,False)
        mesh(stage,'/World/ElasticTarget/body/Visual_1',False)
        mesh(stage,'/World/ActuatedTool/fixed/Transmission/linkL/part_2',False)
        # An enabled collider even with a visual-like name must remain physical.
        mesh(stage,'/World/ElasticTarget/body/Visual_3',True)
        mesh(stage,'/World/ElasticTarget/body/Collider',True)
        mesh(stage,'/World/Background/Visual_4',False)
        mesh(stage,'/World/ElasticTarget/disabled_obstacle',False)
        body=UsdGeom.Xform.Define(stage,'/World/ElasticTarget/body').GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body);UsdPhysics.MassAPI.Apply(body).CreateMassAttr(.15)
        source=stage.GetRootLayer().ExportToString();before=active_collision_state(stage)
        result=prepare_display_meshes(stage)
        self.assertEqual(result['mesh_count'],3)
        self.assertEqual(before,active_collision_state(stage))
        self.assertEqual(source,stage.GetRootLayer().ExportToString())
        self.assertFalse(stage.GetPrimAtPath(expected).HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(stage.GetPrimAtPath('/World/Background/Visual_4').HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(stage.GetPrimAtPath('/World/ElasticTarget/disabled_obstacle').HasAPI(UsdPhysics.CollisionAPI))
        self.assertEqual(prepare_display_meshes(stage)['mesh_count'],0)

    def test_missing_collision_flag_is_not_assumed_display_only(self):
        stage=Usd.Stage.CreateInMemory();prim=mesh(stage,'/World/ElasticTarget/Skin/mesh_0').GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        self.assertEqual(prepare_display_meshes(stage)['mesh_count'],0)

    def test_skinning_matches_existing_helper_and_emits_one_notice(self):
        with tempfile.TemporaryDirectory() as directory:
            assembly=Path(directory);stage=Usd.Stage.CreateInMemory();rng=np.random.default_rng(82)
            paths=[];arrays={};references=[]
            for i in range(3):
                path=f'/World/ElasticTarget/Skin/mesh_{i}';paths.append(path)
                mesh(stage,path,False)
                influences=1 if i==2 else 2
                vertices=rng.normal(size=(37,3));ids=rng.integers(0,2,size=(37,influences))
                weights=rng.random((37,influences));weights/=weights.sum(axis=1,keepdims=True)
                rest=rng.normal(size=(2,3))
                for key,value in dict(vertices=vertices,ids=ids,weights=weights,rest=rest,
                                      body_paths=np.array(['/body_a','/body_b'])).items():
                    arrays[f'{i}_{key}']=value
                references.append((vertices,rest,ids,weights))
            (assembly/'motion_manifest.json').write_text(json.dumps({'plant':{'skin_paths':paths}}))
            np.savez(assembly/'skin.npz',**arrays)
            for name in ('crank','linkL','linkR'):
                UsdGeom.Xform.Define(stage,'/World/ActuatedTool/fixed/Transmission/'+name)
            prepare_display_meshes(stage)
            visuals=DeformationVisuals(stage,assembly,['/body_a','/body_b'])
            rotations=Rotation.from_euler('xyz',[[30,-10,90],[-10,70,-10]],degrees=True)
            positions=np.array([[.3,.4,.5],[.7,.8,.9]])
            poses=np.column_stack([positions,rotations.as_quat()]);notices=[]
            notice=Tf.Notice.Register(Usd.Notice.ObjectsChanged,lambda event,sender:notices.append(event),stage)
            state=active_collision_state(stage)
            metrics=visuals.update(poses,.044)
            notice.Revoke()
            self.assertEqual(metrics['vertices_updated'],111)
            self.assertEqual(len(notices),1)
            for path,(vertices,rest,ids,weights) in zip(paths,references):
                expected=skin_points(vertices,rest,rotations.as_matrix(),positions,ids,weights).astype(np.float32)
                result=UsdGeom.Mesh(stage.GetPrimAtPath(path))
                np.testing.assert_array_equal(np.asarray(result.GetPointsAttr().Get()),expected)
                np.testing.assert_array_equal(np.asarray(result.GetExtentAttr().Get()),[expected.min(0),expected.max(0)])
            self.assertEqual(state,active_collision_state(stage))
            self.assertEqual(visuals.update(poses,.044)['transmission_updates'],0)


if __name__=='__main__':
    unittest.main()
