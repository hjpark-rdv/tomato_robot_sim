"""Reject non-colliding but unsuitable V5 poses and old unguarded records."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import mujoco as mj
import numpy as np

from preview_grip_workspace import BoxObstacles, checked_edge, load_storage_motion, nearest_joint_turns, PlanningError
from preview_posture import POSTURE_POLICY, PostureGuard


class WorkingPostureTests(unittest.TestCase):
    def fixture(self, x=.15, rise=0.):
        model=mj.MjModel.from_xml_string(f'''<mujoco><worldbody>
          <body name="spare" pos="-3 0 0"><geom name="slot" type="sphere"
            size=".01" contype="1" conaffinity="3"/></body>
          <body name="Robot_link3" pos="{x} 0 0">
            <joint name="elbow" axis="0 0 1" range="-165 165"/>
            <geom name="elbow_shell" type="sphere" size=".03" contype="2" conaffinity="3"/>
            <body name="Robot_link4" pos="0 .4 {rise}">
              <joint name="wrist1" axis="0 0 1" range="-360 360"/>
              <geom type="sphere" size=".03" contype="2" conaffinity="3"/>
            </body>
          </body></worldbody></mujoco>''')
        model.geom_contype[0]=model.geom_conaffinity[0]=0
        q=np.r_[np.deg2rad(-90.),0.]
        obstacles=BoxObstacles(model,q,[dict(name='lift/mast',position=[0,0,0],dimensions=[.1,.1,2.])])
        guard=PostureGuard(model,obstacles)
        data=mj.MjData(model);data.qpos[:]=q
        mj.mj_kinematics(model,data);mj.mj_collision(model,data)
        return guard,data

    def test_seven_cm_gap_is_rejected_even_without_penetration(self):
        guard,data=self.fixture()
        self.assertTrue(all(c.dist>=0 for c in data.contact))
        rejection=guard.rejection(data)
        self.assertEqual(rejection['rule'],'mast_clearance')
        self.assertAlmostEqual(rejection['clearance_m'],.07,places=6)

    def test_more_than_ten_cm_clearance_passes(self):
        guard,data=self.fixture(x=.19)
        self.assertIsNone(guard.joint_rejection(data.qpos))
        self.assertIsNone(guard.rejection(data))

    def test_low_elbow_with_upturned_forearm_is_rejected(self):
        guard,data=self.fixture(x=.3,rise=.2)
        self.assertEqual(guard.rejection(data)['rule'],'forearm_points_up_from_low_elbow')

    def test_overfolding_and_wrist_backflip_are_rejected(self):
        guard,data=self.fixture(x=.3)
        self.assertEqual(guard.joint_rejection(np.deg2rad([-156.,0.]))['rule'],'elbow_working_range')
        self.assertEqual(guard.joint_rejection(np.deg2rad([-100.,190.]))['rule'],'wrist1_working_range')
        self.assertEqual(guard.joint_rejection(np.deg2rad([-5.,0.]))['rule'],'elbow_working_range')

    def test_no_mast_fails_closed(self):
        guard,_=self.fixture()
        with self.assertRaisesRegex(ValueError,'requires the fixed lift mast'):
            PostureGuard(guard.model,type('Empty',(),{'rows':[]})())

    def test_old_v5_trajectory_cannot_be_loaded_as_guarded(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'grip_motion.json'
            path.write_text(json.dumps(dict(layout_key='conveyor')))
            with self.assertRaisesRegex(ValueError,'replan required'):
                load_storage_motion(path,None,None,None,None,None,None,None)

    def test_pre_yaw_v6_plan_requires_replanning(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'grip_motion.json'
            path.write_text(json.dumps(dict(layout_key='side',posture_policy=dict(
                version='v6-camera-above-grip-and-mast-clearance-v2'))))
            with patch('preview_grip_workspace.side_posture_policy', return_value=dict(
                    version='v6-camera-up-no-yaw-turn-v3')):
                with self.assertRaisesRegex(ValueError,'replan required'):
                    load_storage_motion(path,None,None,None,None,None,None,None)

    def test_camera_must_stay_above_capture_volume_during_rotation(self):
        guard, data = self.fixture(x=.3)
        # Use the two fixture bodies as camera and gripper to exercise the
        # world-height rule independently of the mast and elbow restrictions.
        policy = dict(POSTURE_POLICY, camera_body='Robot_link3', grip_body='Robot_link4',
            camera_bounds_body_m=[[-.01,-.01,.15],[.01,.01,.24]],
            grip_center_body_m=[0.,0.,.12], grip_half_size_body_m=[.01,.01,.01],
            minimum_camera_above_grip_m=.01)
        obstacles = type('Obstacles', (), {'rows': [dict(gid=g, box=dict(
            name='lift/mast', dimensions=[.1,.1,2.])) for g in guard.columns]})()
        guard = PostureGuard(guard.model, obstacles, policy=policy)
        self.assertIsNone(guard.geometry_rejection(data))
        self.assertAlmostEqual(guard.camera_gap(data), .02)
        original=data.xmat[guard.camera_id].copy()
        data.xmat[guard.camera_id]=np.diag([1.,-1.,-1.]).ravel()
        self.assertEqual(guard.geometry_rejection(data)['rule'],'camera_below_grip')
        data.xmat[guard.camera_id]=original
        data.xpos[guard.grip_id,2] += .03
        reason = guard.geometry_rejection(data)
        self.assertEqual(reason['rule'], 'camera_below_grip')
        self.assertAlmostEqual(reason['clearance_m'], -.01)

    def yaw_fixture(self):
        guard = object.__new__(PostureGuard)
        guard.grip_id = 0
        guard.policy = dict(grip_yaw_working_range_deg=[-30.,210.],
                            minimum_grip_forward_horizontal=.25)
        data = SimpleNamespace(xmat=np.eye(3).reshape(1,9))
        def valid(q):
            angle=q[7]
            c,s=np.cos(angle),np.sin(angle)
            data.xmat[0]=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]]).ravel()
            return guard.yaw_rejection(data) is None
        return guard, data, SimpleNamespace(initial=np.zeros(10), valid_full=valid)

    def test_central_camera_height_reference_is_capture_centre(self):
        guard,data=self.fixture(x=.3)
        guard.camera_id=guard.model.body('Robot_link3').id
        guard.grip_id=guard.model.body('Robot_link4').id
        guard.camera_corners=np.array([[0.,0.,.131],[0.,0.,.22]])
        guard.grip_corners=np.array([[0.,0.,.11],[0.,0.,.14]])
        guard.policy=dict(POSTURE_POLICY,grip_center_body_m=[0.,0.,.12],
                          minimum_camera_above_grip_m=.01)
        self.assertLess(guard.camera_gap(data),0.)
        guard.policy['camera_height_reference']='capture_center'
        self.assertAlmostEqual(guard.camera_gap(data),.011)
        self.assertIsNone(guard.geometry_rejection(data))
        data.xmat[guard.camera_id]=np.diag([1.,-1.,-1.]).ravel()
        self.assertEqual(guard.geometry_rejection(data)['rule'],'camera_below_grip')

    def test_full_yaw_turn_has_valid_endpoints_but_invalid_interior(self):
        guard,data,planner=self.yaw_fixture()
        a=np.zeros(9);b=a.copy();b[6]=2*np.pi
        self.assertTrue(planner.valid_full(np.r_[0.,a]))
        self.assertTrue(planner.valid_full(np.r_[0.,b]))
        self.assertFalse(checked_edge(planner,a,b))
        self.assertFalse(checked_edge(planner,b,a))

    def test_bilateral_half_turn_and_reverse_remain_allowed(self):
        guard,data,planner=self.yaw_fixture()
        a=np.zeros(9);b=a.copy();a[6]=-np.pi/2;b[6]=np.pi/2
        self.assertTrue(checked_edge(planner,a,b))
        self.assertTrue(checked_edge(planner,b,a))
        # No branch-cut jump at +180/-180 in the permitted heading sector.
        q=np.zeros(10);q[7]=np.deg2rad(91.)
        self.assertTrue(planner.valid_full(q))
        self.assertAlmostEqual(guard.grip_yaw(data)[0],181.)

    def test_vertical_tool_cannot_bypass_yaw_sector(self):
        guard,data,_=self.yaw_fixture()
        data.xmat[0]=np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]]).ravel()
        self.assertEqual(guard.yaw_rejection(data)['rule'],'grip_yaw_near_vertical')

    def test_wrist_winding_is_not_accepted_as_equivalent_command(self):
        guard,_=self.fixture(x=.3)
        guard.yaw_wrist=2
        guard.policy=dict(POSTURE_POLICY,wrist3_working_range_deg=[-170.,170.])
        q=np.deg2rad([90.,0.,90.])
        self.assertIsNone(guard.joint_rejection(q))
        q[2]=np.deg2rad(-270.)
        self.assertEqual(guard.joint_rejection(q)['rule'],'wrist3_no_winding')
        bounds=np.tile([-2*np.pi,2*np.pi],(10,1));bounds[7]=np.deg2rad([-170.,170.])
        state=np.zeros(9);state[6]=np.deg2rad(-270.)
        canonical=nearest_joint_turns(state,np.zeros(9),bounds)
        self.assertAlmostEqual(np.rad2deg(canonical[6]),90.)
        self.assertAlmostEqual(np.rad2deg(state[6]),-270.)
        state[6]=np.pi
        with self.assertRaises(PlanningError):nearest_joint_turns(state,np.zeros(9),bounds)


if __name__=='__main__':unittest.main()
