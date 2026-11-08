import copy
import unittest

import numpy as np

from lift_drive_reference import (LIFT_JOINT, apply_lift_drive_reference,
                                  build_lift_drive_reference)


class LiftDriveReferenceTests(unittest.TestCase):
    def setUp(self):
        self.names = ['root_joint', 'jaw_R', LIFT_JOINT, 'base', 'jaw_L']
        self.q = np.array([.2, .026, .02756459452211857, 1.18, -.026])
        self.targets = self.q.copy()
        self.targets[2] = .03587988391518593
        self.bounds = np.array([[-1, -.008, 0, -6.3, -.026], [4, .026, .75, 6.3, .008]])
        self.arrays = dict(robot_dof_names=np.array(self.names), robot_q=self.q.copy())
        self.metadata = dict(schema='native_route_checkpoint_v1', metadata_sha256='a'*64,
            time_s=56.1, sequence=dict(checkpoint_physics_valid=True), controller_state=dict(
                schema='native_robot_drive_targets_v1', robot_dof_names=self.names.copy(),
                position_source='native_get_dof_position_targets',
                velocity_source='native_get_dof_velocity_targets',
                position_targets=self.targets.tolist(), velocity_targets=[0.]*5))
        self.controller = dict(position_targets=self.targets.copy(), velocity_targets=np.zeros(5),
            provenance=dict(checkpoint_metadata_sha256='a'*64,
                source='checkpoint_native_drive_getters', controller_continuity_verified=True,
                position_targets_verified=True, velocity_targets_verified=True))

    def reference(self):
        return build_lift_drive_reference(self.metadata, self.arrays, self.controller,
            enabled=True, camera_harvest=True)

    def apply(self, desired, reference):
        return apply_lift_drive_reference(desired, reference, dof_names=self.names, bounds=self.bounds)

    def test_disabled_default_does_not_require_or_change_any_checkpoint(self):
        self.assertIsNone(build_lift_drive_reference(None, None, None))
        actual, evidence = self.apply(self.q, None)
        np.testing.assert_array_equal(actual, self.q)
        self.assertFalse(evidence['commands_changed'])

    def test_exact_actual_30_native_bias_preserves_prior_target_at_t0(self):
        reference = self.reference()
        actual, evidence = self.apply(self.q, reference)
        np.testing.assert_array_equal(actual, self.targets)
        np.testing.assert_array_equal(self.arrays['robot_q'], self.q)
        self.assertEqual(reference['retained_target_minus_measured_m'], .00831528939306736)
        self.assertEqual(evidence['desired_physical_lift_m'], self.q[2])
        self.assertEqual(evidence['native_lift_position_target_m'], self.targets[2])

    def test_named_lift_only_for_batch_arm_rail_and_asymmetric_jaws_preserved(self):
        desired = np.array([self.q, self.q + [0, 0, .1, .2, 0]])
        original = desired.copy()
        actual, evidence = self.apply(desired, self.reference())
        np.testing.assert_array_equal(desired, original)
        np.testing.assert_array_equal(actual[:, [0, 1, 3, 4]], original[:, [0, 1, 3, 4]])
        np.testing.assert_allclose(actual[:, 2]-desired[:, 2], [.00831528939306736]*2)
        self.assertTrue(evidence['other_dof_commands_unchanged'])

    def test_repeated_hold_does_not_rebase_or_accumulate_existing_bias(self):
        reference = self.reference()
        first, _ = self.apply(self.q, reference)
        for _ in range(8):
            command, _ = self.apply(self.q, reference)
            np.testing.assert_array_equal(command, first)
        np.testing.assert_array_equal(self.q, self.arrays['robot_q'])

    def test_camera_opt_in_required(self):
        with self.assertRaisesRegex(ValueError, 'camera'):
            build_lift_drive_reference(self.metadata, self.arrays, self.controller, enabled=True)

    def test_legacy_setter_or_unverified_cp_cannot_supply_reference(self):
        for key, value in [('source', 'exact_matched_logged_position_setter_command'),
                           ('controller_continuity_verified', False),
                           ('position_targets_verified', False),
                           ('velocity_targets_verified', False),
                           ('checkpoint_metadata_sha256', 'b'*64)]:
            with self.subTest(key=key):
                controller = copy.deepcopy(self.controller)
                controller['provenance'][key] = value
                with self.assertRaises(ValueError):
                    build_lift_drive_reference(self.metadata, self.arrays, controller,
                        enabled=True, camera_harvest=True)

    def test_native_name_target_and_validity_mismatches_rejected(self):
        for change in ['names', 'target', 'validity']:
            metadata = copy.deepcopy(self.metadata)
            if change == 'names': metadata['controller_state']['robot_dof_names'].reverse()
            if change == 'target': metadata['controller_state']['position_targets'][0] += .1
            if change == 'validity': metadata['sequence']['checkpoint_physics_valid'] = False
            with self.subTest(change=change), self.assertRaises(ValueError):
                build_lift_drive_reference(metadata, self.arrays, self.controller,
                    enabled=True, camera_harvest=True)

    def test_corrected_and_other_joint_bounds_reject_without_clamping(self):
        for index, value in [(2, .749), (0, 5.)]:
            desired = self.q.copy(); desired[index] = value; original = desired.copy()
            with self.subTest(index=index), self.assertRaisesRegex(ValueError, 'no clamp'):
                self.apply(desired, self.reference())
            np.testing.assert_array_equal(desired, original)

    def test_source_or_applied_limits_and_bias_tampering_rejected(self):
        for key, value in [('lift_bounds_m', [0., .8]),
                           ('retained_target_minus_measured_m', .009),
                           ('joint_name', 'base'), ('joint_kind', 'revolute')]:
            reference = self.reference(); reference[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.apply(self.q, reference)

    def test_nonfinite_duplicate_names_and_incomplete_bounds_rejected(self):
        for desired, names, bounds in [(np.r_[self.q[:-1], np.nan], self.names, self.bounds),
            (self.q, ['base']*5, self.bounds), (self.q, self.names, self.bounds[:, :-1])]:
            with self.assertRaises(ValueError):
                apply_lift_drive_reference(desired, self.reference(), dof_names=names, bounds=bounds)


if __name__ == '__main__':
    unittest.main()
