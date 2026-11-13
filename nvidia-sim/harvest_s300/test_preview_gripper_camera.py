"""Camera query must include rigidly attached parts and follow the tool frame."""
import unittest
from types import SimpleNamespace

import mujoco as mj
import numpy as np

from preview_gripper_camera import CameraClearance, optical_world, subtract_bounds


class GripperCameraTests(unittest.TestCase):
    def test_relief_boxes_keep_all_material_outside_pocket(self):
        bounds=np.array([[-.04]*3,[.04]*3]);cut=np.array([[-.015,-.05,-.05],[.015,.05,.05]])
        parts=np.array(subtract_bounds(bounds,cut))
        self.assertAlmostEqual(sum(np.prod(hi-lo)for lo,hi in parts),(.08-.03)*.08*.08)
        points=np.random.default_rng(4).uniform(bounds[0],bounds[1],size=(1000,3))
        expected=~np.all((points>=cut[0])&(points<=cut[1]),axis=1)
        covered=np.any([np.all((points>=lo)&(points<=hi),axis=1)for lo,hi in parts],axis=0)
        np.testing.assert_array_equal(covered,expected)

    def test_camera_relief_query_keeps_cover_walls_and_follows_slider(self):
        model=mj.MjModel.from_xml_string('''<mujoco><worldbody>
          <geom type="sphere" size=".01" pos="-3 0 0" contype="0" conaffinity="0"/>
          <geom type="sphere" size=".01" pos="-4 0 0" contype="0" conaffinity="0"/>
          <body name="Tool_fixed"><body name="Tool_L">
            <joint type="slide" axis="1 0 0"/>
            <geom name="cover" type="box" size=".04 .04 .04"/>
          </body></body>
        </worldbody></mujoco>''')
        masks=np.c_[model.geom_contype,model.geom_conaffinity].copy()
        parts=subtract_bounds([[-.04]*3,[.04]*3],[[-.015,-.05,-.05],[.015,.05,.05]])
        query=CameraClearance(model,SimpleNamespace(rows=[]),dict(body='Tool_fixed',
            body_from_camera_optical=np.eye(4).tolist(),housing_optical_bounds_m=[[-.01]*3,[.01]*3],
            cover_reliefs=[dict(geom='cover',body='Tool_L',retained_bounds_body_m=parts)]))
        data=mj.MjData(model);mj.mj_kinematics(model,data)
        self.assertAlmostEqual(query.clearance(data)[0],.005,places=7)
        data.qpos[0]=.006;mj.mj_kinematics(model,data)
        self.assertAlmostEqual(query.clearance(data)[0],-.001,places=7)
        np.testing.assert_array_equal(masks,np.c_[model.geom_contype,model.geom_conaffinity])

    def test_explicit_camera_query_detects_same_weld_interference(self):
        model=mj.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="query_slot" pos="-3 0 0"><geom type="sphere" size=".01" contype="0" conaffinity="0"/></body>
          <body name="Tool_fixed"><geom name="bracket" type="box" size=".02 .02 .02" pos=".04 0 0"/></body>
        </worldbody></mujoco>''')
        masks=np.c_[model.geom_contype,model.geom_conaffinity].copy()
        query=CameraClearance(model,SimpleNamespace(rows=[]),dict(body='Tool_fixed',
            body_from_camera_optical=np.eye(4).tolist(),
            housing_optical_bounds_m=[[-.01]*3,[.01]*3]))
        data=mj.MjData(model);mj.mj_kinematics(model,data)
        distance,other=query.clearance(data)
        self.assertAlmostEqual(distance,.01,places=7)
        self.assertEqual(other,model.geom('bracket').id)
        model.geom_pos[other,0]=.025;mj.mj_kinematics(model,data)
        self.assertAlmostEqual(query.clearance(data)[0],-.005,places=7)
        np.testing.assert_array_equal(masks,np.c_[model.geom_contype,model.geom_conaffinity])

    def test_new_camera_follows_tool_yaw_legacy_stays_on_wrist(self):
        scene=SimpleNamespace(body=lambda n:SimpleNamespace(id={'Tool_fixed':1,'Robot_link5':0}[n]))
        poses=np.tile(np.eye(4),(2,1,1));offset=np.eye(4);offset[:3,3]=[0.,.05,.10]
        spec=dict(body='Tool_fixed',source_body='Robot_link5',body_from_camera_optical=offset.tolist())
        old=dict(body_from_camera_optical=np.eye(4).tolist())
        before=optical_world(poses,scene,spec,'side',old)
        poses[1,:3,:3]=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        after=optical_world(poses,scene,spec,'side',old)
        np.testing.assert_allclose(before[:3,3],[0.,.05,.10])
        np.testing.assert_allclose(after[:3,3],[-.05,0.,.10])
        np.testing.assert_allclose(optical_world(poses,scene,spec,'conveyor',old),np.eye(4))


if __name__=='__main__':unittest.main()
