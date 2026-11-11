"""Mechanical/FK invariants; set S300_UPRIGHT_ASSETS for CAD integration."""
import json
import os
from pathlib import Path
import unittest

import numpy as np
from pxr import Usd, UsdPhysics
from scipy.spatial.transform import Rotation

from upright_yaw_mount import adapter_transform, adapter_geometry, FLANGE, FLANGE_IN_TOOL, ROBOT, PLATFORM, matrix, PARTS
from motion_kinematics import ToolKinematics
from motion_self_collision import MotionSelfCollision
from side_robot_mount import _world_mesh


class AdapterMath(unittest.TestCase):
    def test_last_wrist_axis_is_tool_vertical_and_perpendicular_to_entry(self):
        t=adapter_transform();axis=np.array([0.,1.,0.])
        np.testing.assert_allclose(t[:3,:3].T@axis,[0,0,1],atol=1e-15)
        self.assertAlmostEqual(float(np.dot(axis,t[:3,1])),0.)

    def test_exact_original_flange_and_assumed_adapter_seat_coincide(self):
        np.testing.assert_allclose((adapter_transform()@np.r_[FLANGE_IN_TOOL,1])[:3],FLANGE,atol=1e-15)
        self.assertAlmostEqual(sum(np.prod(s)*2700 for _,_,s in PARTS),1.106784)

    def test_inset_retracts_tool_without_rotating_or_moving_flange(self):
        before=adapter_transform();after=adapter_transform(.12)
        np.testing.assert_allclose(after[:3,:3],before[:3,:3],atol=1e-12)
        np.testing.assert_allclose(before[:3,:3].T@(after[:3,3]-before[:3,3]),[0,-.12,0],atol=1e-12)
        np.testing.assert_allclose((after@np.r_[adapter_geometry(.12)[0],1])[:3],FLANGE,atol=1e-12)

    def test_inset_rejects_invalid_dimensions(self):
        for value in (-.01,.16,float('nan')):
            with self.assertRaises(ValueError):adapter_transform(value)


@unittest.skipUnless(os.getenv('S300_UPRIGHT_ASSETS'),'Requires fresh upright patrol assemblies')
class UprightCAD(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=Path(os.environ['S300_UPRIGHT_ASSETS'])
        cfg=json.loads((cls.root/'up/patrol_assets.json').read_text())
        cls.folder=Path(cfg['plants'][0]['assembly'])
        cls.stage=Usd.Stage.Open(str(cls.folder/'scene.usda'))
        cls.manifest=json.loads((cls.folder/'motion_manifest.json').read_text())
        cls.kin=ToolKinematics(cls.stage)
        cls.solutions=json.loads((cls.root/'ready_solutions.json').read_text())

    def test_base_is_upright_and_seated_on_actual_plate(self):
        t=matrix(self.stage,PLATFORM+'/visuals/link0')
        np.testing.assert_allclose(t[:3,2],[0,0,1],atol=1e-8)
        base=_world_mesh(self.stage,PLATFORM+'/visuals/link0/Scene/mesh')
        plate=_world_mesh(self.stage,PLATFORM+'/visuals/real_lift_carriage/mounting_plate')
        self.assertLess(abs(base.bounds[0,2]-plate.bounds[1,2]),.0002)
        self.assertTrue(self.stage.GetPrimAtPath(PLATFORM+'/collisions/real_mounting_plate').IsActive())
        shelf=self.stage.GetPrimAtPath(PLATFORM+'/visuals/mesh_0/box')
        self.assertFalse(shelf and shelf.IsActive())

    def test_all_zero_pose_joint_frames_still_agree(self):
        from robot_adapter import joint_frame
        for root in (ROBOT,'/World/ActuatedTool'):
            for p in Usd.PrimRange(self.stage.GetPrimAtPath(root)):
                if not p.IsA(UsdPhysics.Joint):continue
                j=UsdPhysics.Joint(p);frames=[]
                for slot,targets in enumerate((j.GetBody0Rel().GetTargets(),j.GetBody1Rel().GetTargets())):
                    owner=matrix(self.stage,str(targets[0])) if targets else np.eye(4)
                    frames.append(owner@joint_frame(j,slot))
                np.testing.assert_allclose(*frames,atol=2e-6,rtol=0,err_msg=str(p.GetPath()))

    def test_both_start_modes_clear_open_closed_and_wrist_yaw_sweep(self):
        checker=MotionSelfCollision(self.stage,self.kin,clearance_m=.002)
        for row in self.solutions:
            q=np.asarray(row['ready_q']);initial=self.kin.fk(q)
            axis=self.kin.frames(q)[ROBOT+'/link6'][:3,1]
            np.testing.assert_allclose(axis,[0,0,1],atol=4e-5)
            for angle in np.linspace(-15,15,7):
                moved=q.copy();moved[-1]+=np.deg2rad(angle)
                pose=self.kin.fk(moved)
                np.testing.assert_allclose(pose[:3,2],initial[:3,2],atol=1e-5)
                measured=np.rad2deg(Rotation.from_matrix(pose[:3,:3]@initial[:3,:3].T).as_rotvec()[2])
                self.assertAlmostEqual(measured,angle,places=6)
                for gap in (.044,.096):self.assertFalse(checker.check(moved,gap),row['mode'])

    def test_lift_moves_actual_plate_base_and_wrist_together(self):
        q=np.asarray(self.solutions[0]['ready_q']);q[0]=0.
        before=self.kin.frames(q);tool=self.kin.fk(q);q[0]=.4
        after=self.kin.frames(q)
        for p in (PLATFORM,ROBOT+'/link1',ROBOT+'/link6'):
            np.testing.assert_allclose(after[p][:3,3]-before[p][:3,3],[0,0,.4],atol=1e-12)
        np.testing.assert_allclose(self.kin.fk(q)[:3,3]-tool[:3,3],[0,0,.4],atol=1e-12)

    def test_lowered_plate_has_matching_collider_and_no_triangle(self):
        report=self.manifest['upright_yaw_mount'].get('lowered_platform')
        if not report:self.skipTest('Original upper mounting plate variant')
        visible=_world_mesh(self.stage,report['plate_path'])
        collider=_world_mesh(self.stage,report['plate_collider'])
        np.testing.assert_allclose(visible.bounds,collider.bounds,atol=1e-7)
        np.testing.assert_allclose(visible.bounds,report['plate_world_bounds_zero_lift_m'],atol=1e-7)
        self.assertAlmostEqual(float(visible.extents[2]),.010,places=6)
        triangle=self.stage.GetPrimAtPath(report['removed_triangle'])
        self.assertFalse(triangle and triangle.IsActive())
        mass=UsdPhysics.MassAPI(self.stage.GetPrimAtPath(PLATFORM))
        self.assertGreater(float(mass.GetMassAttr().Get()),3.5)
        self.assertLess(float(mass.GetMassAttr().Get()),18.5)
        self.assertTrue(np.all(np.asarray(mass.GetDiagonalInertiaAttr().Get())>0))

    def test_plate_camera_center_portrait_and_independent_of_arm(self):
        report=self.manifest['upright_yaw_mount'].get('lowered_platform')
        if not report or self.manifest['mounted_rgbd']['body_path']!=PLATFORM:self.skipTest('Non-platform camera variant')
        camera=self.manifest['mounted_rgbd'];local=np.asarray(camera['body_from_camera_optical'])
        self.assertEqual(camera['body_path'],PLATFORM)
        self.assertLess(*camera['resolution_wh'])
        q=np.asarray(self.solutions[0]['ready_q']).copy();q[0]=0.
        pose=self.kin.frames(q)[PLATFORM]@local
        lo,hi=np.asarray(report['plate_world_bounds_zero_lift_m'])
        np.testing.assert_allclose(pose[:3,3],[lo[0]-.035,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2],atol=1e-7)
        np.testing.assert_allclose(pose[:3,2],[-1,0,0],atol=1e-8)
        np.testing.assert_allclose(pose[:3,1],[0,0,-1],atol=1e-8)
        for row in self.solutions:
            moved=np.asarray(row['ready_q']).copy();moved[0]=0.
            np.testing.assert_allclose(self.kin.frames(moved)[PLATFORM]@local,pose,atol=1e-10)
        q[0]=.1
        raised=self.kin.frames(q)[PLATFORM]@local
        np.testing.assert_allclose(raised[:3,3]-pose[:3,3],[0,0,.1],atol=1e-10)
        # Rendered USD camera must use the same optical frame as SensorCamera.
        authored=matrix(self.stage,camera['camera_path'])
        authored[:3,:3]=authored[:3,:3]@np.diag([1.,-1.,-1.])
        np.testing.assert_allclose(authored,pose,atol=1e-7)

    def test_wrist_camera_above_cap_tilted_and_physically_attached(self):
        camera=self.manifest['mounted_rgbd']
        if 'downward_tilt_at_reference_deg' not in camera:self.skipTest('Non-wrist camera variant')
        body=ROBOT+'/link5';self.assertEqual(camera['body_path'],body)
        mount=np.asarray(camera['body_from_camera_optical'])
        q=np.asarray(camera['reference_ready_q']);frames=self.kin.frames(q);pose=frames[body]@mount
        tilt=np.degrees(np.arcsin(-pose[2,2]))
        self.assertAlmostEqual(tilt,15.,places=6)
        self.assertAlmostEqual(pose[2,3]-camera['reference_cap_top_world_z_m'],camera['optical_center_above_cap_m'],places=6)
        housing=_world_mesh(self.stage,camera['root_path']+'/Housing/mesh')
        delta=frames[body]@np.linalg.inv(matrix(self.stage,body))
        world=housing.vertices@delta[:3,:3].T+delta[:3,3]
        self.assertAlmostEqual(float(world[:,2].min())-camera['reference_cap_top_world_z_m'],.014,places=6)
        np.testing.assert_allclose(pose[:2,3],frames[ROBOT+'/link6'][:2,3],atol=1e-7)
        # One physical mount across elbow modes, no mode-specific extrinsic rewrite.
        down=json.loads((self.root/'down/patrol_assets.json').read_text())['mounted_rgbd']
        np.testing.assert_array_equal(down['body_from_camera_optical'],mount)
        moved=q.copy();moved[-1]+=.2
        np.testing.assert_allclose(self.kin.frames(moved)[body]@mount,pose,atol=1e-10)
        moved=q.copy();moved[-2]+=.2
        self.assertGreater(np.linalg.norm((self.kin.frames(moved)[body]@mount)-pose),.1)
        local_authored=np.linalg.inv(matrix(self.stage,body))@matrix(self.stage,camera['camera_path'])
        local_authored[:3,:3]=local_authored[:3,:3]@np.diag([1.,-1.,-1.])
        np.testing.assert_allclose(local_authored,mount,atol=1e-7)


if __name__=='__main__':unittest.main()
