import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from harvest_group_policy import HarvestGroupSelector,harvest_group_outcome,load_policy,sha


class GroupPolicyTests(unittest.TestCase):
    def fixture(self, centers, eligible=None, minimum=3):
        self.fruits=[dict(name=f'Truss_02_Fruit_{i+1:02d}',path=f'/f/{i}',joint=f'/j/{i}',
                          dimensions_m=[.023]*3) for i in range(len(centers))]
        self.positions=np.array(centers,float);self.paths=[f['path'] for f in self.fruits]
        eligible=set(range(len(centers))) if eligible is None else set(eligible)
        self.policy=dict(minimum_fruits=minimum,fruit_eligibility=[dict(**f,eligible=i in eligible) for i,f in enumerate(self.fruits)])
        self.selector=HarvestGroupSelector(self.policy,lambda *args:[0,1,0],height_offset_m=.0075)

    def select(self,breaks=(),attempts=None,**kw):
        return self.selector(self.fruits,list(breaks),self.paths,self.positions,attempts or {},**kw)

    def test_three_close_ripe_fruit_select_above_lower_cluster(self):
        self.fixture([[0,0,1.1],[.015,.01,1.08],[-.01,.02,1.06],
                      [0,0,.8],[.01,0,.78],[0,.02,.76]])
        result=self.select()
        self.assertEqual(set(result['fruits']),{f['name'] for f in self.fruits[:3]})
        # The commanded grasp transform, including the existing height offset,
        # must actually put these measured centers within the documented belt.
        from motion_kinematics import grasp_pose
        center=np.array(result['aim_world_m'])+[0,0,.0075]
        pose=grasp_pose(center,[0.,1.,0.],depth=.065)
        local=(self.positions[:3]-pose[:3,3])@pose[:3,:3]
        np.testing.assert_allclose(local,result['group_selection']['fruit_centers_tool_m'],atol=1e-12)
        self.assertTrue(np.all((local[:,2]>=.013-1e-10)&(local[:,2]<=.061+1e-10)))

    def test_total_remaining_does_not_count_spatially_separate_fruit(self):
        self.fixture([[0,0,1],[.3,0,1],[.6,0,1],[.9,0,1]])
        self.assertIsNone(self.select())
        self.assertEqual(self.selector.last_decision['reason'],'no_group_meets_minimum')

    def test_asymmetric_group_is_centered_in_commanded_grasp_at_different_yaws(self):
        from motion_kinematics import grasp_pose
        for angle in (0., .7, -2.2):
            with self.subTest(yaw=angle):
                direction=np.array([np.sin(angle),np.cos(angle),0.])
                rotation=np.column_stack((np.cross(direction,[0.,0.,1.]),direction,[0.,0.,1.]))
                self.fixture(np.array([[0,0,1],[.03,.01,.99],[.03,.02,.98]])@rotation.T)
                self.selector.direction=lambda *args:direction
                result=self.select();self.assertIsNotNone(result)
                evidence=result['group_selection']['lateral_centering']
                self.assertGreater(abs(evidence['shift_tool_x_m']),.001)
                seed=np.array(evidence['seed_aim_world_m'])
                aim=np.array(result['aim_world_m'])
                delta=(aim-seed)@rotation
                np.testing.assert_allclose(delta[1:],0.,atol=1e-12)
                pose=grasp_pose(aim+[0,0,.0075],direction,depth=.065)
                local=(self.positions-pose[:3,3])@pose[:3,:3]
                self.assertAlmostEqual(local[:,0].mean(),0.,places=12)
                np.testing.assert_allclose(local,result['group_selection']['fruit_centers_tool_m'],atol=1e-12)
                # Centroid symmetry does not imply equal fruit edge clearance.
                self.assertTrue(np.all(np.abs(local[:,0])+.0115<=.048+1e-10))

    def test_centering_rechecks_selected_fruit_radii(self):
        self.fixture([[-.035,0,1],[.033,0,1],[.035,0,1]])
        self.assertIsNone(self.select())
        self.assertGreater(self.selector.last_decision['lateral_centering_rejections']['selected_fruit_outside'],0)
        self.assertEqual(self.selector.last_decision['reason'],'no_centered_group_fits')
        self.assertFalse(self.selector.last_decision['absolute_harvest_impossibility'])

    def test_centering_cannot_admit_ineligible_fruit(self):
        self.fixture([[-.016,0,1],[.019,0,1],[.025,0,1],[.044,0,1]],eligible=[0,1,2])
        self.assertIsNone(self.select())
        self.assertGreater(self.selector.last_decision['lateral_centering_rejections']['ineligible_fruit_inside'],0)

    def test_centering_uses_latest_measured_state_without_moving_fruit(self):
        self.fixture([[0,0,1],[.03,.01,.99],[.03,.02,.98]])
        first=self.select()
        self.positions[:,0]+=.02;before=self.positions.copy()
        second=self.select()
        self.assertAlmostEqual(second['aim_world_m'][0]-first['aim_world_m'][0],.02)
        np.testing.assert_array_equal(self.positions,before)

    def test_reobserve_after_detachment_stops_with_two_left(self):
        self.fixture([[0,0,1],[.01,0,.985],[-.01,0,.97],[0,0,.95]])
        self.assertIsNotNone(self.select())
        self.assertIsNone(self.select([dict(joint=f['joint']) for f in self.fruits[:2]]))
        self.assertEqual(len(self.selector.last_decision['eligible_remaining']),2)

    def test_green_never_counts_toward_minimum(self):
        self.fixture([[0,0,1],[.01,0,.985],[-.01,0,.97]],eligible=[0,1])
        self.assertIsNone(self.select())

    def test_green_center_inside_group_rejects_aim(self):
        self.fixture([[0,0,1]]*4,eligible=[0,1,2])
        self.assertIsNone(self.select())

    def test_changed_measured_positions_change_group_without_reset(self):
        self.fixture([[0,0,1],[.01,0,.985],[-.01,0,.97]])
        self.assertIsNotNone(self.select())
        self.positions[2,0]=.5
        before=self.positions.copy();self.assertIsNone(self.select())
        np.testing.assert_array_equal(before,self.positions)

    def test_exhausted_attempts_are_not_impossible(self):
        self.fixture([[0,0,1],[.01,0,.985],[-.01,0,.97]])
        self.assertIsNone(self.select(attempts={f['name']:2 for f in self.fruits}))
        self.assertEqual(self.selector.last_decision['reason'],'attempt_budget_exhausted')
        self.assertFalse(self.selector.last_decision['absolute_harvest_impossibility'])

    def test_rejected_group_does_not_fallback_to_single_fruit(self):
        self.fixture([[0,0,1],[.01,0,.985],[-.01,0,.97]])
        self.assertIsNone(self.select(rejected_groups=[[f['name'] for f in self.fruits]]))
        self.assertEqual(self.selector.last_decision['reason'],'group_planning_alternatives_exhausted')

    def test_ineligible_incidental_detachment_reported_separately(self):
        self.fixture([[0,0,1]]*4,eligible=[0,1,2])
        names=[f['name'] for f in self.fruits]
        result=harvest_group_outcome(self.policy,dict(detached=names,remaining=[]))
        self.assertEqual(result['ineligible_detached'],names[3:])

    def test_initial_planning_rejection_is_not_an_attempt_or_permanent_exclusion(self):
        self.fixture([[0,0,1.1],[.015,.01,1.08],[-.01,.02,1.06],
                      [0,0,.8],[.01,0,.78],[0,.02,.76]])
        upper = [f['name'] for f in self.fruits[:3]]
        self.policy['initial_rejected_groups'] = [upper]
        self.selector = HarvestGroupSelector(self.policy,lambda *args:[0,1,0],height_offset_m=.0075)
        attempts = {}
        result = self.select(attempts=attempts)
        self.assertEqual(set(result['fruits']), {f['name'] for f in self.fruits[3:]})
        self.assertEqual(attempts, {})
        self.assertEqual(self.selector.last_decision['initial_planning_rejected_groups'], [upper])
        self.assertEqual(set(self.select()['fruits']), set(upper))
        self.assertEqual(len(self.selector.eligible), 6)

    def test_initial_rejection_requires_unchanged_report_and_known_fruit(self):
        self.fixture([[0,0,1]]*3)
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            for name in ('motion_manifest.json','composed.usdc'):(p/name).write_text('fixture')
            self.policy.update(schema='s300-harvest-group-policy-v1',source_sha256={n:sha(p/n) for n in ('motion_manifest.json','composed.usdc')},
                               initial_rejected_groups=[[f['name'] for f in self.fruits]])
            def load():
                (p/'policy.json').write_text(json.dumps(self.policy))
                return load_policy(p/'policy.json',p,self.fruits)
            with self.assertRaisesRegex(ValueError,'planning reason'):load()
            report=p/'obstacle.json';report.write_text('{"blocked":true}')
            self.policy['initial_rejection_evidence']=dict(reason='Recorded obstacle intersection',
                reports=[dict(path=str(report),sha256=sha(report))])
            self.assertEqual(load(),self.policy)
            report.write_text('changed')
            with self.assertRaisesRegex(ValueError,'evidence hash'):load()
            self.policy['initial_rejected_groups']=[['unknown']]
            with self.assertRaisesRegex(ValueError,'unknown'):load()

    def test_policy_model_hash_and_identity_fail_closed(self):
        self.fixture([[0,0,1]]*3)
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            for name in ('motion_manifest.json','composed.usdc'):(p/name).write_text('fixture')
            self.policy.update(schema='s300-harvest-group-policy-v1',source_sha256={n:sha(p/n) for n in ('motion_manifest.json','composed.usdc')})
            (p/'policy.json').write_text(json.dumps(self.policy))
            self.assertEqual(load_policy(p/'policy.json',p,self.fruits),self.policy)
            (p/'composed.usdc').write_text('changed')
            with self.assertRaisesRegex(ValueError,'hash mismatch'):load_policy(p/'policy.json',p,self.fruits)

    def test_historical_paired_selector_remains_identical(self):
        from truss_sequence import select_group
        self.fixture([[0,0,1],[.01,0,.985],[-.01,0,.97]])
        before=select_group(self.fruits,[],self.paths,self.positions,{},order='highest-first')
        self.select()
        self.assertEqual(before,select_group(self.fruits,[],self.paths,self.positions,{},order='highest-first'))
        self.assertEqual(len(before['fruits']),2)


if __name__=='__main__':unittest.main()
