"""Model coefficient tests; full plant/GPU evidence lives in experiment outputs."""
import unittest
import numpy as np
from newton_attachment_variant import direct_reference_solref


class AttachmentSolrefTests(unittest.TestCase):
    def test_same_reference_stiffness_and_damping(self):
        source=np.array([[.002,1.],[.01,.7]])
        direct=direct_reference_solref(source,reference_hz=400)
        for index,(tc,zeta) in enumerate(source):
            tc=max(tc,.005)
            for dmax in (.5,.9999):
                old=np.array([1/(dmax**2*tc**2*zeta**2),2/(dmax*tc)])
                new=-direct[index]/np.array([dmax**2,dmax])
                np.testing.assert_allclose(new,old,rtol=1e-15)

    def test_damping_change_does_not_weaken_position_stiffness(self):
        source=np.array([[.002,1.]])
        one=direct_reference_solref(source)
        two=direct_reference_solref(source,damping_factor=2.)
        np.testing.assert_array_equal(one[:,0],two[:,0])
        np.testing.assert_array_equal(2*one[:,1],two[:,1])

    def test_discrete_connect_timeconstant_is_not_dt_clamped(self):
        source=np.array([[.002,1.]])
        for hz in (60,120,400):
            result=direct_reference_solref(source,reference_hz=hz,discrete=True)
            np.testing.assert_allclose(result,[[-250000.,-1000.]],rtol=1e-15)
        overdamped=direct_reference_solref(source,discrete=True,damping_factor=2.)
        np.testing.assert_allclose(overdamped,[[-250000.,-2000.]],rtol=1e-15)

    def test_invalid_parameters_rejected(self):
        for value in (0,-1,float('nan')):
            with self.assertRaises(ValueError):
                direct_reference_solref([[.002,1.]],damping_factor=value)
        with self.assertRaises(ValueError):direct_reference_solref([[-1.,-1.]])


if __name__=='__main__':unittest.main()
