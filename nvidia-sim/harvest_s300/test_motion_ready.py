"""Ready-state path invariants, independent of physical success claims."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from motion_kinematics import plan_motion, grasp_pose, belt_speed_fraction


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

    def test_ready_to_entry_is_level_continuous_and_retracted(self):
        phases,meta=plan_motion(self.kin,self.center,self.q,'horizontal_ready')
        self.assertEqual([p[0] for p in phases[:4]],
                         ['ready_hold','prepare_open','prepare_horizontal','approach'])
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

    def test_cluster_aim_does_not_move_fruit(self):
        from motion_assets import grasp_reference
        import copy
        report={'plant':{'fruits':[{'name':'a','path':'/a','center':[0,1,2]},
                                  {'name':'b','path':'/b','center':[2,1,4]}]}}
        before=copy.deepcopy(report);ref=grasp_reference(report,['a','b'],.009)
        np.testing.assert_allclose(ref['center'],[1,1,3.009]);self.assertEqual(report,before)
        for names in (['a','a'],['unknown'],[]):
            with self.assertRaises(ValueError):grasp_reference(report,names)

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
        self.assertAlmostEqual((spin[4]+spin[5])/2,-.1)
        self.assertAlmostEqual((spin[5]-spin[4])/2,.15)
        self.assertEqual(spin[1],2.);np.testing.assert_array_equal(spin[2][0],spin[2][1])


if __name__=='__main__':unittest.main()
