"""Regression cases for missed gripper/arm intersections and jaw movement."""
import unittest
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Sdf
from motion_self_collision import MotionSelfCollision,excluded_body_pairs,check_plan,require_robot_self_collisions


class SelfCollisionTests(unittest.TestCase):
    def test_disabled_or_missing_robot_self_collision_is_blocked(self):
        stage=Usd.Stage.CreateInMemory()
        with self.assertRaisesRegex(ValueError,'requires robot self-collisions'):require_robot_self_collisions(stage)
        prim=stage.DefinePrim('/World/HarvestRobot/Robot/root_joint')
        a=prim.CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool)
        a.Set(False)
        with self.assertRaisesRegex(ValueError,'requires robot self-collisions'):require_robot_self_collisions(stage)
        a.Set(True);self.assertTrue(require_robot_self_collisions(stage))

    def test_empty_or_nonfinite_plan_does_not_pass(self):
        with self.assertRaises(ValueError):check_plan(None,{'phases':[]})
        plan={'phases':[dict(name='bad',duration_s=1,waypoint_q=[[float('nan')]*7,[0]*7],end_gap_m=.096)]}
        with self.assertRaises(ValueError):check_plan(None,plan)

    def test_fixed_and_adjacent_exclusions_do_not_hide_forearm_tool(self):
        bodies=['arm','wrist','flange','fixed_tool','jaw','belt']
        joints=[('arm','wrist',False,True),('wrist','flange',False,True),
                ('flange','fixed_tool',True,True),('flange','jaw',False,True),('jaw','belt',True,True)]
        excluded=excluded_body_pairs(bodies,joints)
        self.assertIn(frozenset(('jaw','belt')),excluded)
        self.assertIn(frozenset(('flange','belt')),excluded)
        self.assertNotIn(frozenset(('arm','fixed_tool')),excluded)
        self.assertNotIn(frozenset(('arm','jaw')),excluded)

    def checker(self):
        s=Usd.Stage.CreateInMemory();paths=['/World/HarvestRobot/Robot/arm','/World/ActuatedTool/jaw']
        for path in paths:
            body=UsdGeom.Xform.Define(s,path);UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            cube=UsdGeom.Cube.Define(s,path+'/shape');cube.CreateSizeAttr(.1)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        checker=MotionSelfCollision(s);poses={p:np.eye(4) for p in paths}
        return checker,poses,paths

    def test_actual_intersection_is_detected(self):
        checker,poses,paths=self.checker();poses[paths[1]][0,3]=.05
        self.assertTrue(checker.check(poses=poses))

    def test_separated_pair_is_clear(self):
        checker,poses,paths=self.checker();poses[paths[1]][0,3]=.15
        self.assertFalse(checker.check(poses=poses))

    def test_planning_clearance_is_distinct_from_intersection(self):
        checker,poses,paths=self.checker();poses[paths[1]][0,3]=.101
        self.assertFalse(checker.check(poses=poses))
        checker.clearance=.002
        hits=checker.check(poses=poses)
        self.assertTrue(hits);self.assertFalse(hits[0]['intersection'])
        self.assertAlmostEqual(hits[0]['distance_m'],.001,places=7)

    def test_same_world_transform_does_not_change_result(self):
        checker,poses,paths=self.checker();poses[paths[1]][0,3]=.05
        for t in poses.values():t[:3,3]+=[.5,-2,1]
        self.assertTrue(checker.check(poses=poses))

    def test_jaw_mid_travel_collision_is_not_missed(self):
        class Checker:
            metadata={}
            def check(self,q,gap):return [{'hit':True}] if .06<gap<.08 else []
        plan={'phases':[dict(name='open',duration_s=1,waypoint_q=[[0]*7,[0]*7],end_gap_m=.096)]}
        result=check_plan(Checker(),plan,hz=120)
        self.assertFalse(result['passed']);self.assertFalse(result['complete'])
        self.assertGreater(result['phases'][0]['first_hit_fraction'],0)
        self.assertLess(result['phases'][0]['first_hit_fraction'],1)

    def test_clear_plan_checks_endpoints(self):
        class Checker:
            metadata={}
            def __init__(self):self.gaps=[]
            def check(self,q,gap):self.gaps.append(gap);return []
        checker=Checker();plan={'phases':[dict(name='open',duration_s=1,waypoint_q=[[0]*7,[0]*7],end_gap_m=.096)]}
        result=check_plan(checker,plan,hz=120)
        self.assertTrue(result['passed']);self.assertTrue(result['complete'])
        self.assertEqual(result['samples'],121)
        self.assertAlmostEqual(checker.gaps[0],.044);self.assertAlmostEqual(checker.gaps[-1],.096)


if __name__=='__main__':unittest.main()
