import copy
import unittest

from patrol_lift_search import propose_lift_scan,remember_lift_proposal


class LiftSearchTests(unittest.TestCase):
    def item(self, goal=.32):
        return dict(key='/plant/Truss_00',visible_now=True,suggested_lift_m=goal,
            last_observation_index=3,last_observation=dict(timestamp_s=5.))

    def test_observed_height_proposes_physical_scan_without_mutating_state(self):
        item=self.item();original=copy.deepcopy(item)
        decision=propose_lift_scan(item,station_m=1.,measured_lift_m=.18,limits_m=[0.,.75])
        self.assertEqual(item,original);self.assertEqual(decision['action'],'lift_scan')
        self.assertEqual(decision['next_lift_m'],.32)
        self.assertEqual(decision['current_station_y_m'],decision['next_station_y_m'])
        self.assertFalse(decision['lift_proposal']['physical_pose_changed'])
        self.assertTrue(decision['requires_camera_observation_after_motion'])

    def test_outside_alignment_is_not_clamped_into_a_harvest_pose(self):
        decision=propose_lift_scan(self.item(-.021),station_m=1.,measured_lift_m=.16,limits_m=[0.,.75])
        self.assertEqual(decision['next_lift_m'],.03)
        self.assertFalse(decision['lift_proposal']['suggested_alignment_within_limits'])
        self.assertEqual(decision['target']['suggested_lift_m'],-.021)

    def test_same_view_jitter_cannot_spend_unbounded_scans(self):
        item=self.item(.18);current=.18
        self.assertIsNone(propose_lift_scan(item,station_m=1.,measured_lift_m=current,limits_m=[0.,.75]))
        for i in range(3):
            decision=propose_lift_scan(item,station_m=1.,measured_lift_m=current,limits_m=[0.,.75],after_planning_failure=True)
            self.assertIsNotNone(decision)
            remember_lift_proposal(item,decision);current=decision['next_lift_m']
        self.assertIsNone(propose_lift_scan(item,station_m=1.,measured_lift_m=current,limits_m=[0.,.75],after_planning_failure=True))
        self.assertEqual(len(item['camera_lift_search']),3)

    def test_no_hidden_or_out_of_limit_view_is_executed(self):
        item=self.item();item['visible_now']=False
        self.assertIsNone(propose_lift_scan(item,station_m=1.,measured_lift_m=.18,limits_m=[0.,.75]))
        with self.assertRaises(ValueError):
            propose_lift_scan(self.item(),station_m=1.,measured_lift_m=.8,limits_m=[0.,.75])


if __name__=='__main__':unittest.main()
