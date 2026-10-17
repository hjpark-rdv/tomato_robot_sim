import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from newton_neighbor_obstacles import add_colliders, selected_colliders, verify_preserved
import mujoco as mj


class NeighborObstacleTests(unittest.TestCase):
    def scene(self):
        stage=Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
        UsdGeom.Xform.Define(stage,'/Neighbor').AddTranslateOp().Set(Gf.Vec3d(1,2,3))
        cap=UsdGeom.Capsule.Define(stage,'/Neighbor/Stem')
        cap.CreateRadiusAttr(.01);cap.CreateHeightAttr(.2);cap.CreateAxisAttr('Y')
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim()).CreateCollisionEnabledAttr(True)
        leaf=UsdGeom.Mesh.Define(stage,'/Neighbor/Leaf')
        UsdPhysics.CollisionAPI.Apply(leaf.GetPrim()).CreateCollisionEnabledAttr(False)
        return stage

    def test_source_transform_and_leaf_omission(self):
        stage=self.scene();root=ET.fromstring('<mujoco><asset/><worldbody/></mujoco>')
        with tempfile.TemporaryDirectory() as tmp:
            report=add_colliders(stage,['/Neighbor'],root,Path(tmp))
        self.assertEqual(len(report),1)
        g=root.find('worldbody/geom')
        np.testing.assert_allclose(np.fromstring(g.get('fromto'),sep=' ').reshape(2,3),[[1,1.9,3],[1,2.1,3]])
        self.assertEqual(g.get('contype'),'8');self.assertEqual(g.get('conaffinity'),'7')

    def test_does_not_silently_convexify_original_triangle_mesh(self):
        stage=self.scene();mesh=stage.GetPrimAtPath('/Neighbor/Leaf')
        UsdPhysics.CollisionAPI(mesh).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr('none')
        with self.assertRaisesRegex(ValueError,'triangle obstacle'):
            selected_colliders(stage,'/Neighbor')

    def test_rejects_changed_existing_physics(self):
        text='<mujoco><worldbody><body name="target"><joint name="hinge"/><geom name="fruit" type="sphere" size=".01"/></body></worldbody></mujoco>'
        old=mj.MjModel.from_xml_string(text);new=mj.MjModel.from_xml_string(text)
        self.assertTrue(verify_preserved(old,new)['passed'])
        new.geom_friction[0,0]*=2
        with self.assertRaisesRegex(ValueError,'physical fields'):
            verify_preserved(old,new)

if __name__=='__main__':unittest.main()
