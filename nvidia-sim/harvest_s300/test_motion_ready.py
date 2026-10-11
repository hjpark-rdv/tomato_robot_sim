"""Ready-state path invariants, independent of physical success claims."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from motion_kinematics import plan_motion, grasp_pose, belt_speed_fraction, cycle_opening_check
from truss_sequence import entry_direction


class CartesianRobot:
    world=np.eye(4)
    def ik(self,pose,seed):
        return np.r_[pose[:3,3],Rotation.from_matrix(pose[:3,:3]).as_rotvec(),0.],0.,0.
    def fk(self,q):
        pose=np.eye(4);pose[:3,3]=q[:3]
        pose[:3,:3]=Rotation.from_rotvec(q[3:6]).as_matrix()
        return pose


class ReadyTests(unittest.TestCase):
    def setUp(self):
        self.kin=CartesianRobot();self.center=np.array([.4,1.,1.2])
        self.pose=grasp_pose(self.center,self.center.copy())
        self.pose[:3,3]-=self.pose[:3,1]*.3
        self.q=self.kin.ik(self.pose,np.zeros(7))[0]

    def test_stem_avoidance_yaw_changes_entry_fk_preserving_shallow_depth_and_level(self):
        axis=dict(direction_world=self.pose[:3,1].tolist())
        original=list(axis['direction_world'])
        np.testing.assert_array_equal(entry_direction(axis),original)
        for yaw in (-60.,30.):
            direction=entry_direction(axis,yaw)
            _,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',
                harvest_profile='belt_open_close_cycle',approach_direction_world=direction,
                grasp_depth_m=.015)
            goal=np.asarray(meta['goal_pose'])
            np.testing.assert_allclose(goal[:3,1],direction,atol=1e-12)
            np.testing.assert_allclose(goal[:3,2],[0,0,1],atol=1e-12)
            # Fruit center is only15mm behind the existing158mm belt nose.
            np.testing.assert_allclose(goal[:3,:3].T@(self.center-goal[:3,3]),
                [0,.143,.037],atol=1e-12)
            self.assertEqual(axis['direction_world'],original)
            np.testing.assert_array_equal(entry_direction(axis,yaw),direction)
        for yaw in (91.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):entry_direction(axis,yaw)

    def test_truss_yaw_rotates_preparation_and_actual_entry_without_pitch_roll(self):
        direction=Rotation.from_euler('z',35,degrees=True).apply(self.pose[:3,1])
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',
            harvest_profile='belt_open_close_cycle',approach_direction_world=direction)
        previous=self.q
        for phase in phases:
            np.testing.assert_allclose(phase[2][0],previous,atol=1e-12)
            previous=phase[2][-1]
            for q in phase[2]:np.testing.assert_allclose(self.kin.fk(q)[:3,2],[0,0,1],atol=1e-12)
        np.testing.assert_allclose(previous,self.q,atol=1e-12)
        entry=next(p for p in phases if p[0]=='approach')
        poses=np.array([self.kin.fk(q) for q in entry[2]])
        np.testing.assert_allclose(poses[:,:3,1],np.tile(direction,(len(poses),1)),atol=1e-12)
        np.testing.assert_allclose(poses[-1,:3,3]-poses[0,:3,3],.1*direction,atol=1e-12)
        goal=np.array(meta['goal_pose'])
        np.testing.assert_allclose(goal[:3,3]+goal[:3,:3]@np.array([0,.093,.037]),self.center,atol=1e-12)
        self.assertEqual(meta['entry_orientation_policy'],'truss_xy_yaw_only')

    def test_truss_direction_cannot_silently_add_pitch_or_yaw_offset(self):
        for bad in ({'pitch_deg':10},{'yaw_offset_deg':30},{'approach_direction_world':[1,0,.1]}):
            opts=dict(approach_direction_world=[1,0,0]);opts.update(bad)
            with self.assertRaisesRegex(ValueError,'Truss direction requires'):
                plan_motion(self.kin,self.center,self.q,'horizontal_ready',**opts)

    def test_ready_to_entry_is_level_continuous_and_retracted(self):
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready')
        self.assertEqual([p[0] for p in phases[:5]],
                         ['ready_hold','prepare_open','prepare_open_hold','prepare_horizontal','approach'])
        np.testing.assert_allclose(phases[1][2],[self.q,self.q])
        self.assertEqual(phases[2][3],.096)
        previous=self.q
        for phase in phases:
            np.testing.assert_allclose(phase[2][0],previous,atol=1e-12)
            previous=phase[2][-1]
        poses=np.array([self.kin.fk(q) for p in phases for q in p[2]])
        np.testing.assert_allclose(poses[:,2,3],self.center[2]-.037,atol=1e-12)
        np.testing.assert_allclose(poses[:,2,1],0.,atol=1e-12)
        np.testing.assert_allclose(poses[:,:3,2],np.tile([0.,0.,1.],(len(poses),1)),atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(np.array(meta['goal_pose'])[:3,3]-self.pose[:3,3]),.3)
        self.assertEqual(next(p for p in phases if p[0]=='approach')[1],4.)

    def test_wrong_ready_height_or_pitch_is_rejected(self):
        for index in (2,3):
            q=self.q.copy();q[index]+=.1
            with self.assertRaisesRegex(ValueError,'entry height and orientation'):
                plan_motion(self.kin,self.center,q,'horizontal_ready')

    def test_nonfinite_preparation_solution_is_rejected(self):
        class BrokenRobot(CartesianRobot):
            def ik(inner,pose,seed):
                q,pe,re=super().ik(pose,seed)
                if np.linalg.norm(pose[:3,3]-self.pose[:3,3])<.15:
                    return q,float('nan'),0.
                return q,pe,re
        with self.assertRaisesRegex(RuntimeError,'Fixed entry approach IK not found'):
            plan_motion(BrokenRobot(),self.center,self.q,'horizontal_ready')

    def test_entry_demonstration_returns_to_ready_without_gripping(self):
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',entry_only=True)
        self.assertTrue(meta['entry_only'])
        self.assertFalse({'close','infeed','twist'} & {p[0] for p in phases})
        self.assertEqual(next(p for p in phases if p[0]=='entry_hold')[1],2.)
        np.testing.assert_allclose(phases[-1][2][-1],self.q)
        for p in phases:
            self.assertEqual(p[4:],(0.,0.))
            if p[0] in ('approach','entry_hold','withdraw','return_ready'):
                self.assertEqual(p[3],.096)

    def test_spin_profile_preserves_entry_and_opens_before_withdrawal(self):
        original,_=plan_motion(self.kin,self.center,self.q,'horizontal_ready')
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',
                               harvest_profile='belt_spin_release')
        for old,new in zip(original[:5],phases[:5]):
            self.assertEqual(old[:2],new[:2]);np.testing.assert_array_equal(old[2],new[2])
            self.assertEqual(old[3:],new[3:])
        spin=next(p for p in phases if p[0]=='belt_spin')
        self.assertEqual(spin[1],2.);self.assertEqual(spin[4],-spin[5])
        self.assertGreater(abs(spin[4]),.015)
        np.testing.assert_array_equal(spin[2][0],spin[2][-1])
        labels=[p[0] for p in phases]
        self.assertEqual(labels[labels.index('withdraw')-1],'release_open')
        for name in ('release_open','withdraw','return_ready'):
            step=next(p for p in phases if p[0]==name)
            self.assertEqual(step[3:],(.096,0.,0.))
        previous=self.q
        for step in phases:
            np.testing.assert_allclose(step[2][0],previous,atol=1e-12)
            previous=step[2][-1]
        np.testing.assert_array_equal(previous,self.q)
        self.assertFalse(meta['belt_spin']['arm_rotates'])

    def test_spin_profile_rejects_conflicting_or_invalid_settings(self):
        for options in ({'entry_only':True},{'belt_speed_m_s':float('nan')},
                        {'belt_speed_m_s':0},{'belt_seconds':-1}):
            with self.subTest(options=options),self.assertRaises(ValueError):
                plan_motion(self.kin,self.center,self.q,'horizontal_ready',
                            harvest_profile='belt_spin_release',**options)

    def test_slow_close_changes_duration_only(self):
        original,_=plan_motion(self.kin,self.center,self.q,'horizontal_ready',harvest_profile='belt_spin_release')
        slow,_=plan_motion(self.kin,self.center,self.q,'horizontal_ready',harvest_profile='belt_spin_release',close_seconds=6.)
        for a,b in zip(original,slow):
            self.assertEqual(a[0],b[0]);np.testing.assert_array_equal(a[2],b[2]);self.assertEqual(a[3:],b[3:])
            self.assertEqual(b[1],6. if b[0]=='close' else a[1])
        with self.assertRaises(ValueError):plan_motion(self.kin,self.center,self.q,'horizontal_ready',close_seconds=6.)

    def test_grasp_gap_changes_only_grasp_jaw_commands(self):
        options=dict(harvest_profile='belt_spin_release',close_seconds=6.)
        original,_=plan_motion(self.kin,self.center,self.q,'horizontal_ready',**options)
        same,_=plan_motion(self.kin,self.center,self.q,'horizontal_ready',grasp_gap_m=.028,**options)
        wider,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',grasp_gap_m=.040,**options)
        for old,default,new in zip(original,same,wider):
            np.testing.assert_array_equal(old[2],default[2]);self.assertEqual(old[:2],default[:2]);self.assertEqual(old[3:],default[3:])
            np.testing.assert_array_equal(old[2],new[2]);self.assertEqual(old[:2],new[:2]);self.assertEqual(old[4:],new[4:])
            self.assertEqual(new[3],.040 if old[0] in ('close','belt_spin') else old[3])
        self.assertEqual(meta['belt_spin']['closed_gap_m'],.040)
        for gap in (.027,.097,float('nan')):
            with self.assertRaises(ValueError):plan_motion(self.kin,self.center,self.q,'horizontal_ready',grasp_gap_m=gap,**options)
        with self.assertRaises(ValueError):plan_motion(self.kin,self.center,self.q,'horizontal_ready',grasp_gap_m=.04)

    def test_cluster_aim_does_not_move_fruit(self):
        from motion_assets import grasp_reference
        import copy
        report={'plant':{'fruits':[{'name':'a','path':'/a','center':[0,1,2]},
                                  {'name':'b','path':'/b','center':[2,1,4]}]}}
        before=copy.deepcopy(report);ref=grasp_reference(report,['a','b'],.009)
        np.testing.assert_allclose(ref['center'],[1,1,3.009]);self.assertEqual(report,before)
        for names in (['a','a'],['unknown'],[]):
            with self.assertRaises(ValueError):grasp_reference(report,names)

    def test_height_offset_changes_cartesian_goals_preserves_pitch_and_input(self):
        center=self.center.copy();options=dict(harvest_profile='belt_spin_release')
        _,original=plan_motion(self.kin,center,self.q,'horizontal_ready',**options)
        phases,new=plan_motion(self.kin,center,self.q,'horizontal_ready',grasp_height_offset_m=-.01,**options)
        np.testing.assert_array_equal(center,self.center)
        for name in ('goal_pose','prep_pose'):
            a=np.array(original[name]);b=np.array(new[name])
            np.testing.assert_allclose(b[:3,:3],a[:3,:3],atol=1e-12)
            np.testing.assert_allclose(b[:3,3]-a[:3,3],[0,0,-.01],atol=1e-12)
        entry=next(x for x in phases if x[0]=='approach')
        poses=np.array([self.kin.fk(q) for q in entry[2]])
        np.testing.assert_allclose(poses[:,2,3],np.array(new['goal_pose'])[2,3],atol=1e-12)
        np.testing.assert_array_equal(phases[0][2][0],self.q)
        self.assertEqual(new['grasp_height_offset_m'],-.01)
        for value in (.051,float('nan')):
            with self.assertRaises(ValueError):plan_motion(self.kin,center,self.q,'horizontal_ready',grasp_height_offset_m=value,**options)
        with self.assertRaises(ValueError):
            plan_motion(self.kin,center,self.q,'horizontal_ready',grasp_height_offset_m=-.01)

    def test_belt_ramp_preserves_two_second_window(self):
        self.assertEqual(belt_speed_fraction(0,2,0),1.)
        for t,v in [(0,0),(.125,.5),(.25,1),(1,1),(1.875,.5),(2,0)]:
            self.assertAlmostEqual(belt_speed_fraction(t,2,.25),v)
        for ramp in (-.1,1.1,float('nan')):
            with self.assertRaises(ValueError):belt_speed_fraction(0,2,ramp)

    def test_belt_common_infeed_keeps_spin_differential_and_arm_stationary(self):
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',
            harvest_profile='belt_spin_release',belt_infeed_speed_m_s=.1)
        spin=next(p for p in phases if p[0]=='belt_spin')
        self.assertAlmostEqual((spin[4]+spin[5])/2,.1)
        self.assertAlmostEqual((spin[5]-spin[4])/2,.15)
        self.assertEqual(spin[1],2.);np.testing.assert_array_equal(spin[2][0],spin[2][1])

    def test_equal_inward_belts_only_reverse_left_surface_of_existing_motion(self):
        for profile in ('belt_spin_release','belt_open_close_cycle'):
            options=dict(harvest_profile=profile,close_seconds=6.,belt_ramp_seconds=.5,
                         grasp_gap_m=.028,grasp_height_offset_m=-.0015)
            previous,_=plan_motion(self.kin,self.center,self.q,'horizontal_ready',
                                  belt_speed_m_s=.05,belt_infeed_speed_m_s=0.,**options)
            inward,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',
                                   belt_speed_m_s=0.,belt_infeed_speed_m_s=.05,**options)
            self.assertEqual(len(previous),len(inward))
            for old,new in zip(previous,inward):
                self.assertEqual(old[:2],new[:2])
                np.testing.assert_array_equal(old[2],new[2])
                self.assertEqual(old[3],new[3])
                self.assertEqual(old[4],-new[4])
                self.assertEqual(old[5],new[5])
                if old[4]!=0.:
                    self.assertEqual(new[4:],(.05,.05))
            self.assertFalse(meta['belt_spin']['opposed'])
            self.assertEqual(meta['belt_spin']['infeed_speed_m_s'],.05)
            self.assertFalse(meta['belt_spin']['arm_rotates'])
            self.assertEqual(meta['belt_spin']['infeed_command_convention'],
                             'native_positive_local_y_inward_v2')
            self.assertEqual(meta['belt_spin']['inward_transport_tool_axis'],[0.,-1.,0.])

    def test_zero_differential_requires_positive_infeed_in_belt_profile(self):
        for profile,infeed in (('legacy',.05),('belt_open_close_cycle',0.),
                               ('belt_open_close_cycle',float('nan')),
                               ('belt_open_close_cycle',-.05)):
            with self.subTest(profile=profile,infeed=infeed),self.assertRaises(ValueError):
                plan_motion(self.kin,self.center,self.q,'horizontal_ready',
                            harvest_profile=profile,belt_speed_m_s=0.,belt_infeed_speed_m_s=infeed)

    def test_continuous_belt_cycle_preserves_approach_and_uses_full_jaw_travel(self):
        options=dict(close_seconds=6.,grasp_gap_m=.028,grasp_height_offset_m=-.01,
                     belt_speed_m_s=.05,belt_infeed_speed_m_s=.05,belt_ramp_seconds=.5)
        old,_=plan_motion(self.kin,self.center,self.q,'horizontal_ready',harvest_profile='belt_spin_release',**options)
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',harvest_profile='belt_open_close_cycle',**options)
        for a,b in zip(old[:5],phases[:5]):
            self.assertEqual(a[:2],b[:2]);np.testing.assert_array_equal(a[2],b[2]);self.assertEqual(a[3:],b[3:])
        window=meta['belt_cycle'];cycle=phases[5:13]
        self.assertEqual([p[0] for p in cycle],['belt_start_open','close','cycle_open','cycle_open_hold','cycle_reclose','belt_finish','cycle_open_final','cycle_open_final_hold'])
        self.assertEqual([p[3] for p in cycle],[.096,.028,.096,.096,.028,.028,.096,.096])
        elapsed=0.
        for name,duration,qs,gap,left,right in cycle:
            np.testing.assert_array_equal(qs[0],qs[1])
            self.assertEqual((left,right),(0.,.1))
            if 0<elapsed<window['duration_s']-.5:
                for delta in (-1e-6,0,1e-6):
                    self.assertEqual(belt_speed_fraction(elapsed+delta,window['duration_s'],.5),1.)
            elapsed+=duration
        self.assertEqual(window['duration_s'],elapsed)
        self.assertEqual(belt_speed_fraction(0,elapsed,.5),0.)
        self.assertEqual(belt_speed_fraction(elapsed,elapsed,.5),0.)
        self.assertTrue(window['detachment_during_close_is_allowed'])
        self.assertEqual(window['close_open_repetitions'],2)
        self.assertTrue(window['withdraw_requires_measured_open'])
        self.assertEqual(phases[13][0],'withdraw')
        self.assertEqual(phases[13][3],.096)
        self.assertNotIn('release_open',[p[0] for p in phases])
        for step in phases[13:]:self.assertEqual(step[4:],(0.,0.))
        for bad in ({'entry_only':True},{'preparation':'direct'}):
            settings=dict(preparation='horizontal_ready',harvest_profile='belt_open_close_cycle',**options);settings.update(bad)
            with self.assertRaises(ValueError):plan_motion(self.kin,self.center,self.q,**settings)

    def test_cycle_reclose_requires_measured_full_open(self):
        self.assertTrue(cycle_opening_check([.096,.0958])['reached'])
        for gaps in ([.093],[.04],[float('nan')],[],[.096,.07],[.097]):
            self.assertFalse(cycle_opening_check(gaps)['reached'])

    def test_two_close_open_repetitions_hold_same_arm_pose_and_belts_through_final_open(self):
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',
            harvest_profile='belt_open_close_cycle',belt_speed_m_s=0.,belt_infeed_speed_m_s=.05,
            close_seconds=6.,belt_ramp_seconds=.5)
        window=meta['belt_cycle'];by_name={p[0]:p for p in phases}
        previous_gap=.096;transitions=[];elapsed=0.
        for name in window['phases']:
            phase=by_name[name]
            np.testing.assert_array_equal(phase[2],[meta['goal_q'],meta['goal_q']])
            if phase[3]!=previous_gap:transitions.append('close' if phase[3]<previous_gap else 'open')
            if name in ('cycle_open','cycle_open_final'):
                for fraction in (0.,.5,1.):
                    speed=belt_speed_fraction(elapsed+phase[1]*fraction,window['duration_s'],.5)
                    self.assertAlmostEqual(speed,1.)
                    self.assertEqual(phase[4:],(.05,.05))
            previous_gap=phase[3];elapsed+=phase[1]
        self.assertEqual(transitions,['close','open','close','open'])
        self.assertEqual(by_name['withdraw'][3:],(.096,0.,0.))
        self.assertEqual(window['repetition_phases'][1]['open'],'cycle_open_final')

    def test_translated_group_preserves_ready_and_horizontal_entry(self):
        opts=dict(harvest_profile='belt_open_close_cycle',grasp_height_offset_m=-.01)
        old,oldmeta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',**opts)
        same,_=plan_motion(self.kin,self.center,self.q,'horizontal_ready',grasp_translation_world_m=[0,0,0],**opts)
        for a,b in zip(old,same):
            np.testing.assert_array_equal(a[2],b[2]);self.assertEqual(a[:2],b[:2]);self.assertEqual(a[3:],b[3:])
        delta=np.array([-.09,.025,.13])
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',grasp_translation_world_m=delta,**opts)
        previous=self.q
        for phase in phases:
            np.testing.assert_allclose(phase[2][0],previous,atol=1e-12);previous=phase[2][-1]
        np.testing.assert_allclose(previous,self.q)
        for name in ('goal_pose','prep_pose'):
            a=np.array(meta[name]);b=np.array(oldmeta[name])
            np.testing.assert_allclose(a[:3,3]-b[:3,3],delta,atol=1e-12)
            np.testing.assert_allclose(a[:3,:3],b[:3,:3],atol=1e-12)
        entry=next(p for p in phases if p[0]=='approach')
        poses=np.array([self.kin.fk(q) for q in entry[2]])
        np.testing.assert_allclose(poses[:,2,3],poses[0,2,3],atol=1e-12)
        np.testing.assert_allclose(poses[:,2,1],0,atol=1e-12)


    def test_height_first_raises_at_ready_before_horizontal_transfer_and_returns(self):
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready',
            harvest_profile='belt_open_close_cycle',grasp_translation_world_m=[-.09,.02,.13],
            preparation_height_first=True)
        by_name={p[0]:p for p in phases}
        raised=np.array([self.kin.fk(q) for q in by_name['prepare_height'][2]])
        np.testing.assert_allclose(raised[:,:2,3],np.tile(self.pose[:2,3],(len(raised),1)),atol=1e-12)
        level=np.array([self.kin.fk(q) for q in by_name['prepare_horizontal'][2]])
        np.testing.assert_allclose(level[:,2,3],np.array(meta['prep_pose'])[2,3],atol=1e-12)
        previous=self.q
        for phase in phases:
            np.testing.assert_allclose(phase[2][0],previous,atol=1e-12);previous=phase[2][-1]
        np.testing.assert_allclose(previous,self.q,atol=1e-12)
        np.testing.assert_array_equal(by_name['return_height'][2],by_name['prepare_height'][2][::-1])
        np.testing.assert_array_equal(by_name['return_ready'][2],by_name['prepare_horizontal'][2][::-1])

    def test_shallow_grasp_moves_arm_outward_without_repositioning_fruit(self):
        opts=dict(harvest_profile='belt_open_close_cycle')
        _,deep=plan_motion(self.kin,self.center,self.q,'horizontal_ready',**opts)
        _,shallow=plan_motion(self.kin,self.center,self.q,'horizontal_ready',grasp_depth_m=.035,**opts)
        direction=np.array(deep['approach_direction_world'])
        for key in ('goal_pose','prep_pose'):
            a=np.array(deep[key]);b=np.array(shallow[key])
            np.testing.assert_allclose(b[:3,3]-a[:3,3],-.03*direction,atol=1e-12)
            np.testing.assert_allclose(b[:3,:3],a[:3,:3],atol=1e-12)
        self.assertEqual(shallow['grasp_center_world_m'],deep['grasp_center_world_m'])


if __name__=='__main__':unittest.main()
