"""Real Cartesian FK staging changes without changing final full-open entry."""
import hashlib
import json
import unittest
import numpy as np
from motion_kinematics import grasp_pose,plan_motion
from test_motion_branch import RedundantCartesianRobot


class EntryRetractTests(unittest.TestCase):
    def fixture(self):
        kin=RedundantCartesianRobot();center=np.array([.4,1.,1.2])
        ready=grasp_pose(center,center.copy());ready[:3,3]-=ready[:3,1]*.3
        start=kin.ik_at_lift(ready,np.zeros(7),.12)[0]
        opts=dict(preparation='horizontal_ready',harvest_profile='belt_open_close_cycle',
                  preparation_height_first=True,approach_direction_world=ready[:3,1])
        return kin,center,start,opts

    def test_default_is_bit_identical_to_prior_command_capture(self):
        kin,center,start,opts=self.fixture()
        phases,meta=plan_motion(kin,center,start,**opts)
        same,explicit=plan_motion(kin,center,start,**opts,approach_distance_m=.10)
        self.assertEqual(meta,explicit)
        self.assertEqual(hashlib.sha256(json.dumps(meta['phases'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),
                         'e3f493070c892963f3d0874c431423cf0121df2e054cfb9ac1e5832993f80b6b')
        for a,b in zip(phases,same):np.testing.assert_array_equal(a[2],b[2])

    def test_staging_and_actual_entry_fk_span_requested_distance_with_same_goal_and_full_open(self):
        kin,center,start,opts=self.fixture();_,default=plan_motion(kin,center,start,**opts)
        goal=np.asarray(default['goal_pose']);direction=np.asarray(default['approach_direction_world'])
        for distance in (.07,.15,.20,.25):
            phases,meta=plan_motion(kin,center,start,**opts,approach_distance_m=distance)
            np.testing.assert_array_equal(meta['goal_pose'],default['goal_pose'])
            self.assertEqual(meta['approach_gap_m'],.096)
            self.assertEqual(meta['belt_cycle'],default['belt_cycle'])
            self.assertEqual(meta['approach_distance_m'],distance)
            np.testing.assert_allclose(np.asarray(meta['prep_pose'])[:3,3],goal[:3,3]-direction*distance,atol=1e-14)
            poses=np.asarray(meta['approach_waypoint_poses']);self.assertEqual(len(poses),61)
            np.testing.assert_allclose(poses[0,:3,3],goal[:3,3]-direction*distance,atol=1e-14)
            np.testing.assert_allclose(poses[-1],goal,atol=1e-14)
            entry=next(p for p in phases if p[0]=='approach')
            self.assertEqual(entry[3],.096)
            for q,pose in zip(entry[2],poses):np.testing.assert_allclose(kin.fk(q),pose,atol=1e-12)
            for a,b in zip(phases,phases[1:]):np.testing.assert_allclose(a[2][-1],b[2][0],atol=1e-12)
            np.testing.assert_array_equal(phases[0][2][0],start)
            np.testing.assert_array_equal(phases[-1][2][-1],start)
            for phase in phases:
                self.assertTrue(np.all(phase[2]>=kin.bounds[0]));self.assertTrue(np.all(phase[2]<=kin.bounds[1]))

    def test_wrong_scope_nonfinite_and_out_of_budget_cannot_change_pose(self):
        kin,center,start,opts=self.fixture()
        for distance in (float('nan'),float('inf'),.049,.301,True,None,[.10]):
            with self.assertRaises((ValueError,TypeError)):
                plan_motion(kin,center,start,**opts,approach_distance_m=distance)
        for preparation,profile in [('direct','legacy'),('horizontal_ready','legacy')]:
            with self.assertRaisesRegex(ValueError,'Entry retract'):
                plan_motion(kin,center,start,preparation=preparation,harvest_profile=profile,approach_distance_m=.15)


if __name__=='__main__':unittest.main()
