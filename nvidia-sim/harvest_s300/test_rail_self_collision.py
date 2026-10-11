"""Physical platform translation must not leave the chassis behind in FCL."""
import unittest
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
from motion_self_collision import MotionSelfCollision,check_plan


class Kinematics:
    def frames(self,q):
        t=np.eye(4);t[1,3]=.15
        return {'/World/HarvestRobot/Robot/arm':t}
    def fk(self,q):
        t=np.eye(4);t[:3,3]=[3.,.15,0.]
        return t


class RailSelfCollisionTests(unittest.TestCase):
    def checker(self):
        stage=Usd.Stage.CreateInMemory()
        for path,y in [('/World/HarvestRobot/Robot/world',0.),('/World/HarvestRobot/Robot/arm',.15)]:
            body=UsdGeom.Xform.Define(stage,path);body.AddTranslateOp().Set((0.,y,0.))
            UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            cube=UsdGeom.Cube.Define(stage,path+'/shape');cube.CreateSizeAttr(.1)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdGeom.Xform.Define(stage,'/World/ActuatedTool')
        return MotionSelfCollision(stage,Kinematics())

    def test_moving_chassis_and_arm_preserve_relative_collision(self):
        checker=self.checker();q=np.zeros(7)
        original=checker.poses(q);moved=checker.poses(q,rail_position_m=-.1)
        for name,t in original.items():
            np.testing.assert_allclose(moved[name][:3,3]-t[:3,3],[0.,-.1,0.])
            np.testing.assert_array_equal(moved[name][:3,:3],t[:3,:3])
        self.assertFalse(checker.check(q))
        self.assertFalse(checker.check(q,rail_position_m=-.1))
        left_behind=dict(moved);left_behind['/World/HarvestRobot/Robot/world']=original['/World/HarvestRobot/Robot/world']
        self.assertTrue(checker.check(poses=left_behind))
        with self.assertRaises(ValueError):checker.check(poses=moved,rail_position_m=-.1)

    def test_rail_samples_use_same_smooth_fraction_and_hold_station(self):
        class Checker:
            metadata={}
            def __init__(self):self.samples=[]
            def check(self,q,gap,*,rail_position_m):
                self.samples.append((float(q[0]),rail_position_m));return []
        checker=Checker()
        plan=dict(phases=[dict(name='rail',duration_s=1.,waypoint_q=[[0.]*7,[1.]*7],
                              end_gap_m=.044,rail_waypoint_positions_m=[0.,.5]),
                          dict(name='hold',duration_s=1.,waypoint_q=[[1.]*7,[1.]*7],end_gap_m=.044)])
        result=check_plan(checker,plan,hz=4)
        self.assertTrue(result['passed'])
        self.assertEqual(checker.samples[:5],[(0.,0.),(.15625,.078125),(.5,.25),(.84375,.421875),(1.,.5)])
        self.assertTrue(all(r==.5 for _,r in checker.samples[5:]))

    def test_nonrail_plan_retains_original_call_signature_and_values(self):
        class Checker:
            metadata={}
            def __init__(self):self.samples=[]
            def check(self,q,gap):self.samples.append((q.copy(),gap));return []
        checker=Checker();plan=dict(phases=[dict(name='legacy',duration_s=1.,waypoint_q=[[0.]*7,[1.]*7],end_gap_m=.096)])
        result=check_plan(checker,plan,hz=4)
        self.assertTrue(result['passed']);self.assertEqual(len(checker.samples),5)
        np.testing.assert_array_equal([q[0] for q,_ in checker.samples],[0.,.15625,.5,.84375,1.])

    def test_missing_nonfinite_or_discontinuous_rail_waypoints_rejected(self):
        for rail in ([0.],[0.,float('nan')],[.2,.4]):
            plan=dict(phases=[dict(name='rail',duration_s=1.,waypoint_q=[[0.]*7,[0.]*7],end_gap_m=.044,
                                  rail_waypoint_positions_m=rail)])
            with self.assertRaises(ValueError):check_plan(None,plan,hz=4)


if __name__=='__main__':unittest.main()
