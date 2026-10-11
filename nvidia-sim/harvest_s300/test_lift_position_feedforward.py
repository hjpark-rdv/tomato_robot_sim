import unittest
import numpy as np

from lift_position_feedforward import estimate_static_lift_bias,lift_drive_command


class LiftPositionFeedforwardTest(unittest.TestCase):
    def setUp(self):
        self.names=['base','farmily_lift_height_joint','elbow','jaw_L','jaw_R']
        self.t=np.arange(61)/120+1.5
        self.q=np.tile([.1,.16876866,.2,-.026,.026],(len(self.t),1))
        self.target=self.q.copy();self.target[:,1]+=.008467495

    def estimate(self,**changes):
        values=dict(time_s=self.t,measured_q=self.q,position_targets=self.target,
                    dof_names=self.names,physics_hz=120)
        values.update(changes)
        return estimate_static_lift_bias(**values)

    def apply(self,command,bias):
        return lift_drive_command(command,bias,self.names,lower_limit_m=0.,upper_limit_m=1.)

    def test_stable_native_position_and_target_prove_bias(self):
        bias=self.estimate()
        self.assertAlmostEqual(bias['offset_m'],.008467495)
        self.assertEqual(bias['calibration_samples'],61)
        self.assertEqual(bias['measured_position_velocity_max_m_s'],0.)
        self.assertFalse(bias['contact_free_ready_hold_verified_by_caller'])
        self.assertFalse(bias['actuator_properties_changed'])

    def test_estimation_is_fixed_and_not_accumulated_at_each_cycle(self):
        bias=self.estimate();desired=self.q[0].copy()
        for unused_cycle in range(6):
            drive=self.apply(desired,bias)
            np.testing.assert_allclose(drive[1]-.008467495,desired[1],rtol=0,atol=1e-15)
            self.assertAlmostEqual(bias['offset_m'],.008467495)
        np.testing.assert_array_equal(desired,self.q[0])

    def test_measured_resume_first_nominal_command_preserves_previous_lift_target(self):
        bias=self.estimate();drive=self.apply(self.q[-1],bias)
        self.assertAlmostEqual(drive[1],self.target[-1,1])
        self.assertNotEqual(self.q[-1,1],self.target[-1,1])

    def test_only_lift_drive_changes_nominal_and_other_dofs_are_exact(self):
        bias=self.estimate();nominal=self.q[[0,1]].copy();original=nominal.copy()
        drive=self.apply(nominal,bias)
        np.testing.assert_array_equal(nominal,original)
        np.testing.assert_array_equal(drive[:,[0,2,3,4]],original[:,[0,2,3,4]])
        self.assertFalse(np.shares_memory(drive,nominal))

    def test_disabled_or_zero_bias_preserves_exact_legacy_object(self):
        self.assertIs(self.apply(self.q,None),self.q)
        bias=self.estimate();bias['offset_m']=0.
        self.assertIs(self.apply(self.q,bias),self.q)

    def test_moving_target_or_unsettled_position_is_rejected(self):
        target=self.target.copy();target[:,1]+=np.linspace(0,.001,len(self.t))
        with self.assertRaisesRegex(ValueError,'target moved'):self.estimate(position_targets=target)
        q=self.q.copy();q[:,1]+=np.linspace(0,.001,len(self.t))
        with self.assertRaisesRegex(ValueError,'not position-settled'):self.estimate(measured_q=q)

    def test_missing_reset_short_or_nonfinite_physics_window_is_rejected(self):
        windows=[(self.t[::2],self.q[::2],self.target[::2]),
                 (self.t[:4],self.q[:4],self.target[:4]),
                 (np.r_[self.t[:-1],self.t[-2]],self.q,self.target)]
        for stamps,q,target in windows:
            with self.subTest(stamps=stamps):
                with self.assertRaises(ValueError):self.estimate(time_s=stamps,measured_q=q,position_targets=target)
        q=self.q.copy();q[2,1]=float('nan')
        with self.assertRaises(ValueError):self.estimate(measured_q=q)

    def test_excessive_bias_is_rejected_instead_of_clipped(self):
        target=self.target.copy();target[:,1]=self.q[:,1]+.025
        with self.assertRaisesRegex(ValueError,'exceeds'):self.estimate(position_targets=target)

    def test_exact_dof_mapping_and_calibration_type_are_required(self):
        with self.assertRaises(ValueError):self.estimate(dof_names=['base']*len(self.names))
        bias=self.estimate();bias['lift_dof_index']=0
        with self.assertRaises(ValueError):self.apply(self.q,bias)
        with self.assertRaises(ValueError):self.apply(self.q,{'offset_m':.008})

    def test_original_limits_are_enforced_without_clipping(self):
        bias=self.estimate();q=self.q[0].copy();q[1]=.999
        with self.assertRaisesRegex(ValueError,'travel limits'):self.apply(q,bias)
        self.assertEqual(q[1],.999)

    def test_no_success_or_calibration_claim_from_arithmetic(self):
        bias=self.estimate()
        self.assertNotIn('harvest_success',bias)
        self.assertNotIn('tracking_verified',bias)
        self.assertIn('actual physics verification',bias['scope'])


if __name__=='__main__':unittest.main()
