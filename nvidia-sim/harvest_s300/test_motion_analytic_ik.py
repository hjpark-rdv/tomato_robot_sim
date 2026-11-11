"""Analytic derivative checked against independent FK and finite differences."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from motion_kinematics import ToolKinematics


class AnalyticIKTests(unittest.TestCase):
    def setUp(self):
        self.kin=ToolKinematics.__new__(ToolKinematics)
        self.kin.world=np.eye(4);self.kin.world[:3,:3]=R.from_euler('xyz',[.2,.3,-.4]).as_matrix()
        self.kin.world[:3,3]=[.4,.3,.5];self.kin.chain=[]
        rng=np.random.default_rng(7)
        for i in range(7):
            origin=np.eye(4);origin[:3,:3]=R.random(random_state=rng).as_matrix()
            origin[:3,3]=rng.uniform(-.15,.15,3)
            axis=rng.normal(size=3);axis/=np.linalg.norm(axis)
            self.kin.chain.append((origin,axis,i==0,str(i)))
        self.kin.offset=np.eye(4);self.kin.offset[:3,3]=[.03,.1,.04]
        self.kin.bounds=np.array([[-1]*7,[1]*7],float)
        self.q=np.array([.2,.1,-.4,.3,-.1,.2,.3]);self.seed=self.q+.01

    def test_world_residual_jacobian_matches_finite_difference(self):
        for angle in (0.,1e-6,.7,3.13):
            target=self.kin.fk(self.q)
            target[:3,:3]=R.from_rotvec(np.array([.2,.3,.4])/np.linalg.norm([.2,.3,.4])*angle).as_matrix()@target[:3,:3]
            residual,jac=self.kin.ik_residual_jacobian(target,self.q,self.seed)
            def independent(q):
                t=self.kin.fk(q)
                return np.r_[(t[:3,3]-target[:3,3])*5,R.from_matrix(target[:3,:3]@t[:3,:3].T).as_rotvec(),(q-self.seed)*.002]
            eps=1e-6
            numeric=np.column_stack([(independent(self.q+np.eye(7)[i]*eps)-independent(self.q-np.eye(7)[i]*eps))/(2*eps) for i in range(7)])
            np.testing.assert_allclose(residual,independent(self.q),atol=1e-12)
            np.testing.assert_allclose(jac,numeric,atol=2e-7)
            np.testing.assert_allclose(self.kin.fk_jacobian(self.q)[0],self.kin.fk(self.q),atol=1e-12)

    def test_fixed_lift_and_free_lift_match_legacy_accuracy(self):
        target=self.kin.fk(self.q)
        for fixed in (False,True):
            solve=lambda:self.kin.ik_at_lift(target,self.seed,self.q[0]) if fixed else self.kin.ik(target,self.seed)
            old=solve();self.kin.ik_solver='analytic';new=solve()
            self.assertLess(new[1],2e-6);self.assertLess(new[2],2e-6)
            np.testing.assert_allclose(self.kin.fk(new[0]),self.kin.fk(old[0]),atol=1e-6)
            if fixed:self.assertEqual(new[0][0],self.q[0])
            self.kin.ik_solver='numeric'

    def test_exact_cache_is_independent_of_result_and_invalidated_by_world(self):
        self.kin.ik_solver='analytic';self.kin.ik_cache={}
        target=self.kin.fk(self.q)
        first=self.kin.ik(target,self.seed);expected=first[0].copy();first[0][:]=123
        cached=self.kin.ik(target,self.seed)
        np.testing.assert_array_equal(cached[0],expected);self.assertEqual(len(self.kin.ik_cache),1)
        self.kin.world[0,3]+=.01
        changed=self.kin.ik(target,self.seed)
        self.assertEqual(len(self.kin.ik_cache),2)
        self.assertGreater(np.linalg.norm(changed[0]-expected),.001)
        self.assertLess(changed[1],.002)

if __name__=='__main__':unittest.main()
