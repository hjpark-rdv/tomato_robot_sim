"""Continuity and fixed-TCP invariants for explicit lift/arm alternatives."""
import hashlib
import json
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from motion_kinematics import plan_motion,grasp_pose,ready_lift_branch


class RedundantCartesianRobot:
    world=np.eye(4)
    bounds=np.array([[-1.]*7,[1.,10.,10.,10.,10.,10.,10.]])
    def fk(self,q):
        t=np.eye(4);t[:3,3]=[q[1],q[2],q[0]+q[3]]
        t[:3,:3]=Rotation.from_rotvec(q[4:]).as_matrix();return t
    def ik(self,pose,seed):return self.ik_at_lift(pose,seed,seed[0])
    def ik_at_lift(self,pose,seed,lift):
        if not self.bounds[0,0]<=lift<=self.bounds[1,0]:raise ValueError('lift limits')
        return np.r_[lift,pose[0,3],pose[1,3],pose[2,3]-lift,
                     Rotation.from_matrix(pose[:3,:3]).as_rotvec()],0.,0.


class MotionBranchTests(unittest.TestCase):
    def setUp(self):
        self.kin=RedundantCartesianRobot();self.center=np.array([.4,1.,1.2])
        ready=grasp_pose(self.center,self.center.copy());ready[:3,3]-=ready[:3,1]*.3
        self.start=self.kin.ik_at_lift(ready,np.zeros(7),.12)[0]
        self.opts=dict(preparation='horizontal_ready',harvest_profile='belt_open_close_cycle',
                       preparation_height_first=True,approach_direction_world=ready[:3,1])

    def test_depth_is_distance_behind_nose_and_smaller_depth_retreats_tool(self):
        # Target remains within the belt span when retreating the tool.  This
        # catches a reversed interpretation of the CAD depth control before
        # selecting another physical approach.
        inward=np.array([-.98,.19,0.])
        nominal=grasp_pose(self.center,inward.copy(),depth=.065)
        retreat=grasp_pose(self.center,inward.copy(),depth=.050)
        advance=grasp_pose(self.center,inward.copy(),depth=.090)
        np.testing.assert_array_equal(retreat[:3,:3],nominal[:3,:3])
        np.testing.assert_allclose(retreat[:3,3]-nominal[:3,3],
                                   -.015*nominal[:3,1],atol=1e-14)
        np.testing.assert_allclose(advance[:3,3]-nominal[:3,3],
                                   .025*nominal[:3,1],atol=1e-14)
        point=np.r_[self.center,1.]
        np.testing.assert_allclose((np.linalg.inv(nominal)@point)[:3],
                                   [0.,.093,.037],atol=1e-14)
        np.testing.assert_allclose((np.linalg.inv(retreat)@point)[:3],
                                   [0.,.108,.037],atol=1e-14)

    def test_zero_offset_and_unspecified_gap_preserve_default_commands_exactly(self):
        a,am=plan_motion(self.kin,self.center,self.start,**self.opts)
        b,bm=plan_motion(self.kin,self.center,self.start,**self.opts,
                         ready_lift_offset_m=0.,preparation_gap_m=None)
        self.assertEqual(am['phases'],bm['phases'])
        # Fixed commands for the requested two complete close/open repetitions
        # (2026-10-11); catches unintended changes shared by both current calls.
        digest=hashlib.sha256(json.dumps(am['phases'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
        self.assertEqual(digest,'d7f3058ba28e6ec101d5404c6f06b62da57b14c3f3e8eda1ad04d563eefcca1a')
        for p,q in zip(a,b):
            self.assertEqual(p[:2],q[:2]);np.testing.assert_array_equal(p[2],q[2]);self.assertEqual(p[3:],q[3:])
        self.assertNotIn('prepare_lift_branch',[p[0] for p in a])

    def test_redistribution_changes_joints_but_preserves_tcp_and_returns_to_real_start(self):
        old,original=plan_motion(self.kin,self.center,self.start,**self.opts)
        phases,meta=plan_motion(self.kin,self.center,self.start,**self.opts,
                                ready_lift_offset_m=.2,preparation_gap_m=.044)
        previous=self.start
        for p in phases:
            np.testing.assert_allclose(p[2][0],previous,atol=1e-12);previous=p[2][-1]
            for q in p[2]:np.testing.assert_allclose(self.kin.fk(q)[:3,2],[0,0,1],atol=1e-12)
        np.testing.assert_array_equal(previous,self.start)
        lift=next(p for p in phases if p[0]=='prepare_lift_branch')
        self.assertGreater(np.ptp(lift[2][:,0]),.19)
        for q in lift[2]:np.testing.assert_allclose(self.kin.fk(q),self.kin.fk(self.start),atol=1e-12)
        entry=next(p for p in phases if p[0]=='approach');old_entry=next(p for p in old if p[0]=='approach')
        self.assertFalse(np.array_equal(entry[2],old_entry[2]))
        for newq,oldq in zip(entry[2],old_entry[2]):
            self.assertAlmostEqual(newq[0],.32);np.testing.assert_allclose(self.kin.fk(newq),self.kin.fk(oldq),atol=1e-12)
        np.testing.assert_array_equal(meta['goal_pose'],original['goal_pose'])

    def test_legacy_transfer_can_be_reproduced_explicitly(self):
        phases,meta=plan_motion(self.kin,self.center,self.start,**self.opts,preparation_gap_m=.044,
                               preparation_opening='legacy-staging')
        labels=[p[0] for p in phases]
        self.assertGreater(labels.index('prepare_open'),labels.index('prepare_horizontal'))
        self.assertLess(labels.index('prepare_open'),labels.index('approach'))
        for p in phases:
            if p[0] in ('prepare_gap','prepare_height','prepare_horizontal'):self.assertEqual(p[3],.044)
            if p[0] in ('prepare_open','approach'):self.assertEqual(p[3],.096)
        opening=next(p for p in phases if p[0]=='prepare_open')
        np.testing.assert_array_equal(opening[2][0],opening[2][-1])
        prefix=sum(p[1] for p in phases[:labels.index('approach')+1])
        self.assertEqual(meta['belt_cycle']['start_after_settle_s'],prefix)

    def test_invalid_offsets_limits_and_mode_are_rejected(self):
        for offset in (float('nan'),float('inf'),.401):
            with self.assertRaises(ValueError):plan_motion(self.kin,self.center,self.start,**self.opts,ready_lift_offset_m=offset)
        for gap in (.02,.097,float('nan')):
            with self.assertRaises(ValueError):plan_motion(self.kin,self.center,self.start,**self.opts,preparation_gap_m=gap)
        with self.assertRaises(ValueError):
            plan_motion(self.kin,self.center,self.start,preparation='direct',ready_lift_offset_m=.1)
        start=self.start.copy();start[0]=.9
        with self.assertRaisesRegex(ValueError,'lift limits'):ready_lift_branch(self.kin,start,.2)

    def test_ready_opening_precedes_every_moving_phase_even_with_old_narrow_gap(self):
        for connector in ('cartesian','joint'):
            for offset in (0.,.2):
                with self.subTest(connector=connector,offset=offset):
                    opts=dict(self.opts,horizontal_ready_connector=connector,
                              ready_lift_offset_m=offset,preparation_gap_m=.044)
                    phases,meta=plan_motion(self.kin,self.center,self.start,**opts)
                    old,_=plan_motion(self.kin,self.center,self.start,**opts,preparation_opening='legacy-staging')
                    labels=[p[0] for p in phases]
                    self.assertEqual(labels[:2],['ready_hold','prepare_open'])
                    np.testing.assert_array_equal(phases[1][2],[self.start,self.start])
                    self.assertNotIn('prepare_gap',labels)
                    contract=meta['preparation_opening']
                    self.assertEqual(contract['location'],'ready')
                    self.assertEqual(contract['requested_transfer_gap_m'],.044)
                    self.assertEqual(contract['effective_transfer_gap_m'],.096)
                    for phase in phases[2:labels.index('approach')+1]:
                        self.assertEqual(phase[3],.096)
                        self.assertIn(phase[0],contract['feedback_before_phases'])
                        original=next(p for p in old if p[0]==phase[0])
                        np.testing.assert_array_equal(phase[2],original[2])
                    # Harvest and withdrawal are identical; only preparation
                    # jaw commands, opening location and schedule change.
                    suffix=phases[labels.index('approach'):]
                    old_suffix=old[next(i for i,p in enumerate(old) if p[0]=='approach'):]
                    for new,previous in zip(suffix,old_suffix):
                        self.assertEqual(new[:2],previous[:2])
                        self.assertEqual(new[3:],previous[3:])
                        np.testing.assert_array_equal(new[2],previous[2])
                    previous=self.start
                    for phase in phases:
                        np.testing.assert_allclose(phase[2][0],previous,atol=1e-12)
                        previous=phase[2][-1]

    def test_opening_policy_rejects_unknown_values(self):
        for value in ('staging',None,True):
            with self.assertRaisesRegex(ValueError,'Preparation opening'):
                plan_motion(self.kin,self.center,self.start,**self.opts,preparation_opening=value)

    def test_long_row_aim_requires_explicit_guard_and_changes_tcp_not_plant_or_orientation(self):
        translation=np.array([0,.6,0.]);before=self.center.copy()
        with self.assertRaisesRegex(ValueError,'within 0.5'):
            plan_motion(self.kin,self.center,self.start,**self.opts,grasp_translation_world_m=translation)
        _,original=plan_motion(self.kin,self.center,self.start,**self.opts)
        _,moved=plan_motion(self.kin,self.center,self.start,**self.opts,
                            grasp_translation_world_m=translation,max_grasp_translation_m=1.5)
        oldpose=np.array(original['goal_pose']);newpose=np.array(moved['goal_pose'])
        np.testing.assert_array_equal(newpose[:3,:3],oldpose[:3,:3])
        np.testing.assert_allclose(newpose[:3,3]-oldpose[:3,3],translation,atol=1e-12)
        np.testing.assert_array_equal(self.center,before)
        for maximum in (0.,2.01,float('nan'),float('inf')):
            with self.assertRaisesRegex(ValueError,'Maximum grasp translation'):
                plan_motion(self.kin,self.center,self.start,**self.opts,max_grasp_translation_m=maximum)

    def test_measured_ready_connects_real_height_and_rotation_without_resetting_q(self):
        measured=self.start.copy();measured[3]-=.008
        actual_pose=self.kin.fk(measured)
        actual_pose[:3,:3]=Rotation.from_euler('x',.012).as_matrix()@actual_pose[:3,:3]
        measured=self.kin.ik_at_lift(actual_pose,measured,measured[0])[0]
        with self.assertRaisesRegex(ValueError,'Horizontal ready start'):
            plan_motion(self.kin,self.center,measured,**self.opts)
        phases,meta=plan_motion(self.kin,self.center,measured,**self.opts,
                                allow_measured_ready_start=True,preparation_gap_m=.044)
        np.testing.assert_array_equal(phases[0][2][0],measured)
        height=next(p for p in phases if p[0]=='prepare_height')
        np.testing.assert_allclose(self.kin.fk(height[2][0]),actual_pose,atol=1e-12)
        np.testing.assert_allclose(self.kin.fk(height[2][-1])[:3,2],[0,0,1],atol=1e-12)
        self.assertAlmostEqual(self.kin.fk(height[2][-1])[2,3],np.array(meta['prep_pose'])[2,3])
        # The intermediate poses carry actual roll/pitch error smoothly toward
        # level; replacing them with a level pose at t=0 would fail this test.
        angles=[Rotation.from_matrix(self.kin.fk(q)[:3,:3]@self.kin.fk(height[2][-1])[:3,:3].T).magnitude()
                for q in height[2]]
        self.assertGreater(angles[0],.011);self.assertGreater(angles[1],.010)
        self.assertTrue(np.all(np.diff(angles)<=1e-12))
        previous=measured
        for phase in phases:
            np.testing.assert_allclose(phase[2][0],previous,atol=1e-12);previous=phase[2][-1]
        np.testing.assert_array_equal(previous,measured)
        entry=next(p for p in phases if p[0]=='approach')
        for q in entry[2]:
            pose=self.kin.fk(q)
            np.testing.assert_allclose(pose[:3,2],[0,0,1],atol=1e-12)
            self.assertAlmostEqual(pose[2,3],np.array(meta['goal_pose'])[2,3])
        connector=meta['measured_ready_connector']
        self.assertAlmostEqual(connector['initial_height_difference_m'],-.008)
        np.testing.assert_array_equal(connector['initial_q'],measured)
        self.assertFalse(connector['collision_certified'])

    def test_measured_ready_raised_base_height_and_lift_branch_are_physical_waypoints(self):
        measured=self.start.copy();measured[3]+=.003
        phases,meta=plan_motion(self.kin,self.center,measured,**self.opts,
                                allow_measured_ready_start=True,ready_lift_offset_m=-.15)
        np.testing.assert_array_equal(phases[0][2][0],measured)
        lift=next(p for p in phases if p[0]=='prepare_lift_branch')
        for q in lift[2]:np.testing.assert_allclose(self.kin.fk(q),self.kin.fk(measured),atol=1e-12)
        self.assertAlmostEqual(meta['measured_ready_connector']['initial_height_difference_m'],.003)
        self.assertEqual(meta['belt_cycle']['start_after_settle_s'],
                         sum(p[1] for p in phases[:next(i for i,p in enumerate(phases) if p[0]=='approach')+1]))
        for invalid in (1,'yes',None):
            with self.assertRaisesRegex(ValueError,'Measured ready start'):
                plan_motion(self.kin,self.center,self.start,**self.opts,allow_measured_ready_start=invalid)
        with self.assertRaisesRegex(ValueError,'Measured ready start'):
            plan_motion(self.kin,self.center,self.start,allow_measured_ready_start=True)

    def test_joint_connector_uses_real_staging_branch_and_exact_reverse(self):
        measured=self.start.copy();measured[3]-=.012
        phases,meta=plan_motion(self.kin,self.center,measured,**self.opts,
            allow_measured_ready_start=True,horizontal_ready_connector='joint',
            ready_lift_offset_m=.2,preparation_gap_m=.044,preparation_opening='legacy-staging')
        labels=[p[0]for p in phases]
        self.assertNotIn('prepare_height',labels);self.assertNotIn('prepare_horizontal',labels)
        connector=next(p for p in phases if p[0]=='prepare_joint_branch')
        reverse=next(p for p in phases if p[0]=='return_joint_branch')
        lift=next(p for p in phases if p[0]=='prepare_lift_branch')
        np.testing.assert_array_equal(connector[2][0],lift[2][-1])
        np.testing.assert_array_equal(reverse[2],connector[2][::-1])
        np.testing.assert_allclose(self.kin.fk(connector[2][-1]),meta['prep_pose'],atol=1e-12)
        self.assertEqual(connector[1],6.);self.assertEqual(connector[3],.044)
        opening=next(p for p in phases if p[0]=='prepare_open')
        self.assertGreater(labels.index('prepare_open'),labels.index('prepare_joint_branch'))
        self.assertEqual(opening[3],.096)
        entry=next(p for p in phases if p[0]=='approach')
        for q in entry[2]:
            pose=self.kin.fk(q);np.testing.assert_allclose(pose[:3,2],[0,0,1],atol=1e-12)
            self.assertAlmostEqual(pose[2,3],np.array(meta['goal_pose'])[2,3])
        np.testing.assert_array_equal(phases[-1][2][-1],measured)
        self.assertEqual(meta['horizontal_ready_connector'],'joint')
        self.assertFalse(meta['preparation_height_first_applied'])
        self.assertEqual(meta['belt_cycle']['start_after_settle_s'],
            sum(p[1]for p in phases[:labels.index('approach')+1]))
        for invalid in ('unknown',None):
            with self.assertRaisesRegex(ValueError,'Horizontal ready connector'):
                plan_motion(self.kin,self.center,self.start,**self.opts,horizontal_ready_connector=invalid)
        with self.assertRaisesRegex(ValueError,'Horizontal ready connector'):
            plan_motion(self.kin,self.center,self.start,preparation='direct',horizontal_ready_connector='joint')

    def test_explicit_joint_branch_seed_is_selected_and_default_remains_unspecified(self):
        _,meta=plan_motion(self.kin,self.center,self.start,**self.opts,
            horizontal_ready_connector='joint',horizontal_ready_seed_index=0)
        self.assertEqual(meta['selected_seed_index'],0)
        self.assertEqual(meta['horizontal_ready_seed_index'],0)
        _,default=plan_motion(self.kin,self.center,self.start,**self.opts)
        self.assertIsNone(default['horizontal_ready_seed_index'])
        for seed in (True,-1,9,1.5):
            with self.assertRaisesRegex(ValueError,'Explicit ready seed'):
                plan_motion(self.kin,self.center,self.start,**self.opts,
                    horizontal_ready_connector='joint',horizontal_ready_seed_index=seed)
        with self.assertRaisesRegex(ValueError,'Explicit ready seed'):
            plan_motion(self.kin,self.center,self.start,**self.opts,horizontal_ready_seed_index=0)

    def test_feasible_staging_pose_is_not_misreported_when_cartesian_connector_fails(self):
        ready_position=self.kin.fk(self.start)[:3,3].copy()
        class ConnectorBlockedRobot(RedundantCartesianRobot):
            def ik(inner,pose,seed):
                q,pe,re=super().ik(pose,seed)
                if .05<np.linalg.norm(pose[:3,3]-ready_position)<.10:pe=.003
                return q,pe,re
        kin=ConnectorBlockedRobot()
        with self.assertRaisesRegex(RuntimeError,'ready-to-staging connector.*insertion was not evaluated'):
            plan_motion(kin,self.center,self.start,**self.opts,allow_measured_ready_start=True)
        phases,meta=plan_motion(kin,self.center,self.start,**self.opts,
            allow_measured_ready_start=True,horizontal_ready_connector='joint')
        connector=next(p for p in phases if p[0]=='prepare_joint_branch')
        np.testing.assert_allclose(kin.fk(connector[2][-1]),meta['prep_pose'],atol=1e-12)
        np.testing.assert_allclose(kin.fk(next(p for p in phases if p[0]=='approach')[2][-1]),meta['goal_pose'],atol=1e-12)


if __name__=='__main__':unittest.main()
