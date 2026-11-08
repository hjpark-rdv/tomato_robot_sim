"""Actual CAD/FK integration; set S300_SIDE_SOURCE and S300_SIDE_ASSEMBLY."""
import json
import os
from pathlib import Path
import unittest

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from real_lift_mount import ROBOT,PLATFORM,matrix
from side_robot_mount import (BASE_VISUAL,CARRIAGE,REMOVED,SIDE_READY_Q,
                             _world_mesh,rebase_initial_station)
from motion_kinematics import ToolKinematics
from mounted_rgbd import BODY,ROOT,optical_pose


@unittest.skipUnless(os.environ.get('S300_SIDE_SOURCE') and os.environ.get('S300_SIDE_ASSEMBLY'),
                     'Requires source and newly side-mounted CAD assets')
class SideMountIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source=Usd.Stage.Open(os.environ['S300_SIDE_SOURCE']+'/scene.usda')
        cls.folder=Path(os.environ['S300_SIDE_ASSEMBLY'])
        cls.stage=Usd.Stage.Open(str(cls.folder/'scene.usda'))
        cls.manifest=json.loads((cls.folder/'motion_manifest.json').read_text())
        cls.report=cls.manifest['side_robot_mount']

    def test_removed_shelf_is_absent_and_vertical_plate_is_physical(self):
        for path in REMOVED:
            prim=self.stage.GetPrimAtPath(path)
            self.assertFalse(prim and prim.IsActive())
        collider=self.stage.GetPrimAtPath(self.report['physical_carriage_collider'])
        self.assertTrue(collider.HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get())
        np.testing.assert_allclose(_world_mesh(self.stage,str(collider.GetPath())).bounds,
            _world_mesh(self.stage,CARRIAGE).bounds,atol=1e-8,rtol=0)

    def test_mount_surface_and_base_are_in_contact_without_tilt_into_plate(self):
        normal=np.asarray(self.report['mount_normal_world']);seat=np.asarray(self.report['mount_contact_world_m'])
        mesh=_world_mesh(self.stage,BASE_VISUAL+'/Scene/mesh')
        distances=(mesh.vertices-seat)@normal
        self.assertLess(abs(float(distances.min())),1e-7)
        self.assertLess(abs(normal[2]),1e-6)
        self.assertLess(normal[1],-.999)

    def test_fk_preserves_world_vertical_lift_not_rotated_lift_axis(self):
        old=ToolKinematics(self.source);new=ToolKinematics(self.stage)
        delta=np.asarray(self.report['arm_delta_world'])
        for height in (0.,.375,.75):
            q=SIDE_READY_Q.copy();q[0]=height;zero=q.copy();zero[0]=0
            expected=delta@old.fk(zero);expected[2,3]+=height
            np.testing.assert_allclose(new.fk(q),expected,atol=1e-8,rtol=0)
        path=ROBOT+'/joints/farmily_lift_height_joint'
        for attr in ('physics:axis','physics:lowerLimit','physics:upperLimit','drive:linear:physics:stiffness',
                     'drive:linear:physics:damping','drive:linear:physics:maxForce'):
            self.assertEqual(self.source.GetPrimAtPath(path).GetAttribute(attr).Get(),
                             self.stage.GetPrimAtPath(path).GetAttribute(attr).Get())

    def test_base_joint_frames_coincide_and_arm_mass_properties_are_unchanged(self):
        joint=UsdPhysics.Joint(self.stage.GetPrimAtPath(ROBOT+'/joints/base'));frames=[]
        for slot,targets in enumerate((joint.GetBody0Rel().GetTargets(),joint.GetBody1Rel().GetTargets())):
            local=np.eye(4);local[:3,3]=getattr(joint,f'GetLocalPos{slot}Attr')().Get()
            q=getattr(joint,f'GetLocalRot{slot}Attr')().Get()
            local[:3,:3]=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix()
            frames.append(matrix(self.stage,str(targets[0]))@local)
        np.testing.assert_allclose(frames[0],frames[1],atol=1e-7,rtol=0)
        for path in self.report['moving_rigid_bodies']:
            for attr in ('physics:mass','physics:diagonalInertia','physics:principalAxes','physics:centerOfMass'):
                self.assertEqual(self.source.GetPrimAtPath(path).GetAttribute(attr).Get(),
                                 self.stage.GetPrimAtPath(path).GetAttribute(attr).Get())
        mass=self.report['mass_properties']
        self.assertGreater(mass['mass_kg'],3.5);self.assertLess(mass['mass_kg'],18.5)
        self.assertTrue(np.all(np.asarray(mass['diagonal_inertia_kg_m2'])>0))

    def test_ready_state_is_physically_initialized_and_camera_faces_left(self):
        kin=ToolKinematics(self.stage);q=np.asarray(self.manifest['initial_robot_q'])
        np.testing.assert_array_equal(q,SIDE_READY_Q)
        frame=kin.frames(q)[BODY];pose=np.r_[frame[:3,3],Rotation.from_matrix(frame[:3,:3]).as_quat()]
        optical=optical_pose(pose,self.manifest['mounted_rgbd'])
        np.testing.assert_allclose(optical[:3,2],[-1,0,0],atol=.0008,rtol=0)
        np.testing.assert_allclose(optical[:3,1],[0,0,-1],atol=1e-7,rtol=0)
        for name,value in zip(kin.names,q):
            prim=self.stage.GetPrimAtPath(ROBOT+'/joints/'+name)
            kind='linear' if prim.IsA(UsdPhysics.PrismaticJoint) else 'angular'
            units=value if kind=='linear' else np.rad2deg(value)
            self.assertEqual(float(prim.GetAttribute('state:'+kind+':physics:position').Get()),float(np.float32(units)))

    def test_fresh_entry_rebase_moves_anchor_and_robot_together_not_farm(self):
        stage=Usd.Stage.CreateInMemory();stage.GetRootLayer().subLayerPaths=[str((self.folder/'composed.usdc').resolve())]
        manifest=json.loads((self.folder/'motion_manifest.json').read_text());before=matrix(stage,ROBOT+'/world')
        old=manifest['rail']['initial_station_y_m'];result=rebase_initial_station(stage,manifest,0.)
        after=matrix(stage,ROBOT+'/world');np.testing.assert_allclose(after[:3,3]-before[:3,3],[0,-old,0],atol=1e-12)
        self.assertEqual(manifest['rail']['limits_relative_m'],[0.,5.])
        self.assertFalse(result['runtime_teleport']);self.assertFalse(result['farm_geometry_changed'])
        joint=UsdPhysics.Joint(stage.GetPrimAtPath(ROBOT+'/rail_world_fixed'))
        point=np.asarray(joint.GetLocalPos0Attr().Get())
        np.testing.assert_allclose(point,matrix(stage,ROBOT+'/rail_anchor')[:3,3],atol=1e-7,rtol=0)


if __name__=='__main__':unittest.main()
