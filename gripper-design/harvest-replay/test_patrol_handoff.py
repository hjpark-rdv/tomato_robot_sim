import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import patrol_handoff as handoff


class ActualRouteHandoff(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.parent=self.root/'actual_parent';self.parent.mkdir()
        self.cp=self.parent/'cycle_checkpoints/000';self.cp.mkdir(parents=True)
        (self.cp/'checkpoint.json').write_text('{"immutable":"original"}')
        (self.parent/'summary.json').write_text('{"completed":true}')
        self.trial=self.root/'new_window';self.trial.mkdir();self.assembly=self.root/'assembly';self.assembly.mkdir()
        self.metadata={'metadata_sha256':'verified_unsigned_hash'}
        self.input=dict(schema='s300-native-active-window-start-v1',checkpoint=str(self.cp),
            checkpoint_metadata_sha256='verified_unsigned_hash',source_scene_sha256='actual_old_model',
            current_scene_sha256='actual_new_model',measured_parent_body_mapping={'robot':'robot','carrier':'carrier'},
            native_initialization_validation='pending',new_active_plant_reset=False,
            old_attached_frozen_plant_modified=False,bitwise_engine_continuity=False,training_eligible=False,harvest_success=None)
        errors=[dict(path=p,position_error_m=0.,orientation_error_rad=0.,linear_velocity_error_m_s=0.,angular_velocity_error_rad_s=0.)
            for p in ('robot','carrier')]
        self.native=copy.deepcopy(self.input)
        self.native.update(restored_controller_state=dict(controller_continuity_verified=True,
            position_targets_verified=True,velocity_targets_verified=True,unknown_position_measured_hold_fallback=False,
            velocity_drive_left_authored=False,position_targets_max_abs_error=0.,velocity_targets_max_abs_error=0.),
            preserved_body_errors=errors,preserved_body_count=2,native_articulation_kinematics_refreshed=True,
            articulated_link_transforms_set_directly=False,standalone_body_errors=[errors[1]],articulated_body_errors=[errors[0]],
            robot_q_max_abs_error=0.,robot_qvel_max_abs_error=0.,plant_q_max_abs_error=0.,plant_qvel_max_abs_error=0.,
            numerical_restore_tolerances=dict(position_m=1e-4,orientation_rad=1e-4,robot_q=1e-5,robot_qvel=1e-5,
                standalone_linear_velocity_m_s=1e-4,standalone_angular_velocity_rad_s=1e-4))
        for group in ('','standalone_','articulated_'):
            for field in ('position_error_m','orientation_error_rad','linear_velocity_error_m_s','angular_velocity_error_rad_s'):
                self.native[group+'max_'+field]=0.
        self.context=dict(report=copy.deepcopy(self.input),standalone_paths=['carrier'],unharvested_reactivation=None)
        self.save()

    def tearDown(self):self.temp.cleanup()
    def save(self):
        (self.trial/'route_start_input.json').write_text(json.dumps(self.input))
        (self.trial/'route_start_native_validation.json').write_text(json.dumps(self.native))

    def invoke(self):
        with patch.object(handoff,'load_checkpoint',return_value=(self.metadata,{})), \
             patch.object(handoff,'completed_final_checkpoint',return_value=(self.cp,self.metadata)) as final, \
             patch.object(handoff,'_route_context',return_value=self.context) as existing_checker:
            proof=handoff.measured_handoff(self.trial,self.assembly,{},dict(trial=str(self.parent)))
            final.assert_called_once();existing_checker.assert_called_once_with(self.assembly,{})
        return proof

    def test_route_start_reuses_existing_model_and_final_native_checkpoint_checker(self):
        proof=self.invoke()
        self.assertEqual(proof['handoff_kind'],'active_window_native_start')
        self.assertEqual(proof['input_file'],'route_start_input.json')
        self.assertTrue(proof['controller_continuity_verified'])
        self.assertTrue(proof['existing_route_model_checker_reused'])
        self.assertFalse(proof['bitwise_engine_continuity'])
        self.assertFalse((self.trial/'resume_input.json').exists())
        self.assertFalse((self.trial/'resume_native_validation.json').exists())

    def test_checkpoint_sha_mismatch_and_nonfinal_parent_are_rejected(self):
        self.input['checkpoint_metadata_sha256']='foreign_hash';self.save()
        with self.assertRaisesRegex(ValueError,'metadata SHA mismatch'):self.invoke()
        self.input['checkpoint_metadata_sha256']='verified_unsigned_hash';self.save()
        with patch.object(handoff,'load_checkpoint',return_value=(self.metadata,{})), \
             patch.object(handoff,'completed_final_checkpoint',return_value=(self.parent/'older_checkpoint',self.metadata)):
            with self.assertRaisesRegex(ValueError,'final valid actual parent'):
                handoff.measured_handoff(self.trial,self.assembly,{},dict(trial=str(self.parent)))

    def test_foreign_model_or_body_mapping_not_accepted(self):
        for key,value in [('current_scene_sha256','foreign'),('measured_parent_body_mapping',{'robot':'other'})]:
            native=copy.deepcopy(self.native);native[key]=value
            with self.assertRaisesRegex(ValueError,'identity/input mismatch'):
                handoff.verify_route_result(self.input,native,standalone_paths={'carrier'})

    def test_unknown_or_incomplete_native_result_is_rejected(self):
        for change in ('status','missing_metric','missing_body','missing_controller','missing_guards'):
            native=copy.deepcopy(self.native)
            if change=='status':native['native_initialization_validation']='unknown'
            if change=='missing_metric':native.pop('max_position_error_m')
            if change=='missing_body':native['preserved_body_errors'].pop()
            if change=='missing_controller':native.pop('restored_controller_state')
            if change=='missing_guards':native.pop('numerical_restore_tolerances')
            with self.subTest(change=change),self.assertRaises(ValueError):
                handoff.verify_route_result(self.input,native,standalone_paths={'carrier'})

    def test_nan_negative_and_relaxed_guard_are_not_accepted(self):
        for change in ('nan','negative','relaxed','target_mismatch'):
            native=copy.deepcopy(self.native)
            if change=='nan':native['robot_q_max_abs_error']=float('nan')
            if change=='negative':native['robot_q_max_abs_error']=-1.
            if change=='relaxed':native['numerical_restore_tolerances']['position_m']=.01
            if change=='target_mismatch':native['restored_controller_state']['position_targets_max_abs_error']=.001
            with self.subTest(change=change),self.assertRaises(ValueError):
                handoff.verify_route_result(self.input,native,standalone_paths={'carrier'})

    def test_forged_aggregate_cannot_hide_wrong_native_body_or_velocity(self):
        for field in ('position_error_m','orientation_error_rad','linear_velocity_error_m_s','angular_velocity_error_rad_s'):
            native=copy.deepcopy(self.native);native['preserved_body_errors'][0][field]=.01
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'aggregate/body restore discrepancy'):
                handoff.verify_route_result(self.input,native,standalone_paths={'carrier'})

    def test_failed_actual_restore_cannot_be_published_with_larger_tolerance(self):
        native=copy.deepcopy(self.native)
        native['robot_q_max_abs_error']=2e-5
        with self.assertRaisesRegex(ValueError,'restore guard failed'):
            handoff.verify_route_result(self.input,native,standalone_paths={'carrier'})

    def test_no_input_or_ambiguous_restore_is_explicit_error_not_silent_wait(self):
        (self.trial/'route_start_input.json').unlink()
        with self.assertRaisesRegex(ValueError,'Missing or ambiguous'):
            handoff.measured_handoff(self.trial,self.assembly,{},dict(trial=str(self.parent)))
        self.save();(self.trial/'resume_input.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'Missing or ambiguous'):
            handoff.measured_handoff(self.trial,self.assembly,{},dict(trial=str(self.parent)))


if __name__=='__main__':unittest.main()
