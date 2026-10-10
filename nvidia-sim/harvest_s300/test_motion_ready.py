"""Ready-state path invariants, independent of physical success claims."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from motion_kinematics import plan_motion, grasp_pose


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


if __name__=='__main__':unittest.main()
