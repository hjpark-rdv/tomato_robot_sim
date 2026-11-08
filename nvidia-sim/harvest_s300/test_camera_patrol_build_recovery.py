"""Known pre-native build migration adapter; UNIT FIXTURE ONLY, no simulator."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import camera_patrol_runner as runner
from compact_motion_assets import file_sha256
from same_plant_target_revision import _source_files


class KnownBuildFailureRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.prior=self.root/'prior';self.prior.mkdir()
        self.assembly=self.root/'assembly';self.assembly.mkdir()
        for name in ('scene.usda','skin.npz'):(self.assembly/name).write_bytes(b'UNIT FIXTURE ONLY')
        (self.assembly/'motion_manifest.json').write_text('{}')
        self.trial=self.prior/'trials/segment_002_lift_scan/segment_002_lift_scan';self.trial.mkdir(parents=True)
        self.checkpoint=self.trial/'cycle_checkpoints/000';self.checkpoint.mkdir(parents=True)
        self.queue=self.prior/'camera_patrol_queue.json';self.write(self.queue,{'unit_fixture_only':True})
        self.controller=self.prior/'controller_source';self.controller.mkdir()
        self.runner_source=self.controller/'camera_patrol_runner.py';self.runner_source.write_text('UNIT FIXTURE DISPATCHER')
        self.helper_source=self.trial/'code/same_plant_target_revision.py';self.helper_source.parent.mkdir()
        self.helper_source.write_text('UNIT FIXTURE FAILED BUILDER')
        self.runner_sha=file_sha256(self.runner_source);self.helper_sha=file_sha256(self.helper_source)
        self.write(self.trial/'code_sha256.json',{'same_plant_target_revision.py':self.helper_sha})
        self.metadata=dict(time_s=11.,metadata_sha256='metadata',checkpoint_npz_sha256='npz',
            source_assembly=str(self.assembly),scene_sha256='scene',source_plant='/World/Greenhouse/Plants/Plant_20',
            manifest={},controller_state={'position_targets':[.03]},break_events=[])
        self.arrays={'robot_q':np.array([.03]),'robot_qvel':np.array([0.]),'plant_q':np.array([.001])}
        self.final_arrays=copy.deepcopy(self.arrays);self.final_metadata=copy.deepcopy(self.metadata)
        self.terminal=dict(verified_final_physical_state=True,checkpoint_metadata_sha256='metadata')
        self.report=dict(schema='s300-visible-camera-patrol-campaign-v1',status='blocked_preserving_physical_history',
            executed=True,training_eligible=False,harvest_success=None,target_truss_windows={'enabled':True},
            stop_reason=runner._KNOWN_BUILD_FAILURE,accepted_assembly=str(self.assembly),accepted_checkpoint=str(self.checkpoint),
            controller_source_sha256={'camera_patrol_runner.py':self.runner_sha},
            segments=[dict(index=i,decision={'action':action},status='completed_physics')for i,action in enumerate(['scan','move','lift_scan'])])
        self.report['segments'][-1].update(checkpoint=str(self.checkpoint),assembly=str(self.assembly),trial=str(self.trial),checkpoint_proof=self.terminal)
        self.report['segments'].append(dict(index=3,decision={'action':'harvest','target_key':'/World/Greenhouse/Plants/Plant_20/Truss_01'},
            status='pipeline_or_physical_blocker',parent_assembly=str(self.assembly),parent_checkpoint=str(self.checkpoint),
            blocker_type='ValueError',blocker=runner._KNOWN_BUILD_FAILURE))
        self.rejected=self.prior/'windows/003_plant_20_Truss_01_revision';self.rejected.mkdir(parents=True)
        for name in ('scene.usda','composed.usdc','skin.npz'):(self.rejected/name).write_bytes(('REJECTED UNIT FIXTURE '+name).encode())
        self.write(self.rejected/'checkpoint_materialization_identity.json',dict(schema='s300-whole-canonical-layer-storage-identity-v1',
            passed=True,canonical_equal=True,full_authored_layer_checked=True,numeric_validity={'passed':True},
            materialized_sha256=file_sha256(self.rejected/'composed.usdc'),wrapper_sha256=file_sha256(self.rejected/'scene.usda')))
        self.diff=self.root/'difference.json';self.fix=self.root/'fix.json'
        for path in (self.diff,self.fix):self.write(path,{'unit_fixture_only':True})
        self.proof_path=self.root/'proof.json'
        self.proof=dict(schema='s300-known-pre-native-target-revision-failure-proof-v1',passed=True,
            failure_class='confirmed_pre_native_target_revision_audit_implementation_error',
            corrected_implementation_verified=True,source_files_unchanged=True,post_restore_campaign_physics_started=False,
            rejected_derived_asset_accepted=False,failing_helper_sha256=self.helper_sha,failing_runner_sha256=self.runner_sha,
            source_file_sha256=_source_files(self.assembly),checkpoint_metadata_sha256='metadata',checkpoint_npz_sha256='npz',
            difference_evidence={'path':str(self.diff),'sha256':file_sha256(self.diff)},
            fix_validation={'path':str(self.fix),'sha256':file_sha256(self.fix)})
        self.refresh()
        for name,value in [('_KNOWN_BUILD_FAILURE_RUNNER_SHA',self.runner_sha),('_KNOWN_BUILD_FAILURE_HELPER_SHA',self.helper_sha)]:
            context=patch.object(runner,name,value);context.start();self.addCleanup(context.stop)
        context=patch.object(runner,'load_checkpoint',side_effect=self.load);context.start();self.addCleanup(context.stop)

    @staticmethod
    def write(path,value):path.write_text(json.dumps(value))
    def refresh(self):
        self.write(self.prior/'camera_patrol.json',self.report)
        self.proof.update(prior_report_sha256=file_sha256(self.prior/'camera_patrol.json'),
            prior_queue_sha256=file_sha256(self.queue),rejected_output_file_sha256={str(p):file_sha256(p)for p in sorted(self.rejected.iterdir())})
        self.write(self.proof_path,self.proof)
    def load(self,path):
        return (self.metadata,self.arrays)if Path(path)==self.checkpoint else(self.final_metadata,self.final_arrays)
    def run_recovery(self,proof=True):
        return runner.pre_native_target_revision_build_recovery(self.report,self.prior,assembly=self.assembly,
            checkpoint=self.checkpoint,metadata=self.metadata,proof_path=self.proof_path if proof else None,terminal_proof=self.terminal)

    def test_valid_fixture_preserves_original_queue_report_and_actual_endpoint(self):
        before={p:p.read_bytes()for p in [self.queue,self.prior/'camera_patrol.json',self.assembly/'scene.usda']}
        proof=self.run_recovery()
        self.assertEqual(proof['accepted_checkpoint'],str(self.checkpoint));self.assertTrue(proof['mandatory_fresh_scan'])
        self.assertFalse(proof['unaccepted_native_dispatch']);self.assertFalse(proof['checkpoint_rollback'])
        self.assertEqual(proof['harvest_credit'],0)
        self.assertEqual(before,{p:p.read_bytes()for p in before})
    def test_default_does_not_authorize_build_recovery(self):self.assertIsNone(self.run_recovery(False))
    def test_actual_physics_stop_is_not_migratable(self):
        self.report['status']='actual_physics_stopped_preserving_terminal_state';self.refresh()
        with self.assertRaises(ValueError):self.run_recovery()
    def test_any_native_dispatch_record_rejected(self):
        self.report['segments'][-1]['command']=['native'];self.refresh()
        with self.assertRaises(ValueError):self.run_recovery()
    def test_any_native_trial_directory_rejected(self):
        (self.prior/'trials/segment_003_harvest').mkdir()
        with self.assertRaisesRegex(ValueError,'dispatch/native'):self.run_recovery()
    def test_any_case_dispatch_file_rejected(self):
        self.write(self.prior/'segment_003_scan_cases.json',[])
        with self.assertRaisesRegex(ValueError,'dispatch/native'):self.run_recovery()
    def test_rejected_derived_manifest_or_state_forbidden(self):
        self.write(self.rejected/'motion_manifest.json',{})
        with self.assertRaisesRegex(ValueError,'unrecognized outputs'):self.run_recovery()
    def test_earlier_checkpoint_even_if_position_matches_rejected(self):
        self.metadata['time_s']=5.
        with self.assertRaises(ValueError):self.run_recovery()
    def test_terminal_velocity_mismatch_is_not_hidden_by_same_pose(self):
        self.final_arrays['robot_qvel'][0]=.01
        with self.assertRaisesRegex(ValueError,'native arrays'):self.run_recovery()
    def test_controller_and_break_metadata_must_match_exact_final(self):
        self.final_metadata['controller_state']['position_targets'][0]=.04
        with self.assertRaisesRegex(ValueError,'metadata/controller'):self.run_recovery()
    def test_unknown_dispatcher_or_builder_versions_forbidden(self):
        self.report['controller_source_sha256']['camera_patrol_runner.py']='unknown';self.refresh()
        with self.assertRaises(ValueError):self.run_recovery()
    def test_source_asset_changes_forbidden(self):
        (self.assembly/'scene.usda').write_text('CHANGED')
        with self.assertRaisesRegex(ValueError,'source asset'):self.run_recovery()
    def test_root_cause_evidence_must_stay_hash_bound(self):
        self.diff.write_text('CHANGED')
        with self.assertRaisesRegex(ValueError,'root-cause/fix evidence'):self.run_recovery()
    def test_wrong_failure_error_is_not_generic_resume_permission(self):
        self.report['stop_reason']='unknown failure';self.refresh()
        with self.assertRaises(ValueError):self.run_recovery()

if __name__=='__main__':unittest.main()
