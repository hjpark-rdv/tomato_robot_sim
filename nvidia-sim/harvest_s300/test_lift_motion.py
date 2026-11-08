import unittest
import numpy as np
from lift_motion import plan_lift_cycle


class LiftMotionTests(unittest.TestCase):
    def setUp(self):
        self.q=np.array([.177,1.3,.2,2.5,.4,2.9,3.1])
        self.lo=np.array([0.,-6.,-6.,-6.,-6.,-6.,-6.])
        self.hi=np.array([.75,6.,6.,6.,6.,6.,6.])

    def test_only_lift_moves_and_returns_continuously(self):
        phases,plan=plan_lift_cycle(self.q,self.lo,self.hi)
        last=self.q
        for _,duration,qs,gap,left,right in phases:
            np.testing.assert_array_equal(last,qs[0])
            np.testing.assert_array_equal(qs[:,1:],np.tile(self.q[1:],(2,1)))
            self.assertTrue(np.all(qs>=self.lo) and np.all(qs<=self.hi))
            self.assertEqual((gap,left,right),(.044,0.,0.))
            last=qs[-1]
        np.testing.assert_array_equal(last,self.q)
        self.assertEqual(sum(row[1] for row in phases)+2,23)
        self.assertIsNone(plan['harvest_success'])

    def test_rejects_invalid_strokes_and_initial_states(self):
        for q,lo,hi in [(self.q,.05,.8),(self.q,.7,.05),
                        (self.q,.05,float('nan')),(np.full(7,np.nan),.05,.7)]:
            with self.assertRaises(ValueError):plan_lift_cycle(q,self.lo,self.hi,low=lo,high=hi)
