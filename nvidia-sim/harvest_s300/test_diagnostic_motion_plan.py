"""A certificate cannot authorize a changed target, suffix, clock, or source."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from diagnostic_motion_plan import digest, load_connected_plan
from test_run_motion_conditioning import actual_cli


def row(name, q, duration=1., gap=.044):
    return dict(name=name, duration_s=duration, waypoint_q=copy.deepcopy(q),
                end_gap_m=gap, belt_local_y_m_s=[0., 0.],
                rail_waypoint_positions_m=[0., 0.])


class DiagnosticMotionPlanTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.assembly = self.root/'assembly'; self.assembly.mkdir()
        (self.assembly/'scene.usda').write_text('unchanged source')
        self.manifest = dict(scene_sha256=digest(self.assembly/'scene.usda'),
            compact_assets=dict(materialized_dependency=dict(sha256='a'*64)),
            fresh_campaign=dict(schema='s300-independent-fresh-near-campaign-v1',
                inherited_native_breaks=[], accepted_previous_route_history=False))
        self.write_manifest()
        a, b, mid = [0.]*7, [1.]*7, [.5]*7
        self.original = dict(phases=[row('ready_hold', [a,a]),
            row('prepare_joint_branch',[a,b],2.), row('approach',[b,b],4.,.096),
            row('close',[b,b],2.,.028), row('withdraw',[b,b],4.,.096),
            row('return_joint_branch',[b,a],2.,.096), row('hold',[a,a])],
            goal_q=b, prep_q=b, goal_pose=[1.], prep_pose=[2.],
            approach_direction_world=[1.,0.,0.], pitch_deg=0.,
            grasp_depth_m=.065, grasp_height_offset_m=-.0015,
            grasp_lateral_offset_m=0.,
            belt_cycle=dict(start_after_settle_s=7., duration_s=2.))
        self.plan = copy.deepcopy(self.original)
        self.plan['phases'] = copy.deepcopy([self.original['phases'][0],
            row('prepare_ompl_000',[a,mid],2.),row('prepare_ompl_001',[mid,b],2.),
            *self.original['phases'][2:5],row('return_ompl_000',[b,mid],2.,.096),
            row('return_ompl_001',[mid,a],2.,.096), self.original['phases'][-1]])
        self.plan['belt_cycle']['start_after_settle_s']=9.
        self.plan['ompl_prefix_connection']=dict(only_initial_group=True)
        self.folder=self.root/'connected'; self.folder.mkdir()
        self.source=self.root/'source.json'; self.source.write_text(json.dumps(self.original))
        self.path=self.folder/'plan.json'
        (self.folder/'self_120hz.json').write_text(json.dumps(dict(passed=True,complete=True)))
        (self.folder/'preparation_environment_120hz.json').write_text(json.dumps(dict(
            passed=True,complete=True,eligible_for_native_execution=True,requested_hz=120)))
        self.write_full_environment()
        self.report=dict(schema='s300-existing-ompl-prefix-connection-v1',
            native_eligible=True,physics_executed=False,training_eligible=False,
            source_assembly=str(self.assembly),source_scene_sha256=self.manifest['scene_sha256'],
            source_usdc_sha256='a'*64,source_plan=str(self.source))
        self.write_proof()

    def write_manifest(self):
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest))

    def write_full_environment(self, **changes):
        proof=dict(passed=True,complete=True,requested_hz=120,
            selected_phases=[row['name'] for row in self.plan['phases']])
        proof.update(changes)
        (self.folder/'full_environment_120hz.json').write_text(json.dumps(proof))

    def write_proof(self):
        self.path.write_text(json.dumps(self.plan))
        for path,key in ((self.path,'plan_sha256'),(self.source,'source_plan_sha256'),
                         (self.folder/'self_120hz.json','self_120hz_sha256'),
                         (self.folder/'preparation_environment_120hz.json',
                          'preparation_environment_120hz_sha256'),
                         (self.folder/'full_environment_120hz.json',
                          'full_environment_120hz_sha256')):
            self.report[key]=digest(path)
        (self.folder/'report.json').write_text(json.dumps(self.report))

    def load(self, baseline=None):
        return load_connected_plan(self.assembly,self.path,[],baseline or self.original)

    def test_import_preserves_suffix_and_reconstructs_existing_executor_commands(self):
        phases,plan=self.load()
        self.assertEqual(len(phases),len(plan['phases']))
        self.assertTrue(all(len(p)==6 for p in phases))
        np.testing.assert_array_equal(phases[2][2],[[.5]*7,[1.]*7])
        self.assertEqual(plan['goal_q'],self.original['goal_q'])
        self.assertEqual(plan['belt_cycle']['start_after_settle_s'],9.)
        self.assertFalse(plan['imported_diagnostic_plan']['physics_executed'])
        self.assertEqual(self.original['belt_cycle']['start_after_settle_s'],7.)

    def test_target_changed_since_offline_audit_rejected(self):
        different=copy.deepcopy(self.original)
        different['goal_q'][0]=.9
        with self.assertRaisesRegex(ValueError,'target or entry'):self.load(different)

    def test_native_tool_cache_must_match_certificate_and_runtime(self):
        cache=self.root/'native_tool.json';cache.write_text('all native pieces')
        with self.assertRaisesRegex(ValueError,'not part'):
            load_connected_plan(self.assembly,self.path,[],self.original,native_tool_cache=cache)
        self.report['native_tool_cache']=dict(path=str(cache),sha256=digest(cache))
        self.write_proof()
        with self.assertRaisesRegex(ValueError,'audited cache'):self.load()
        _,plan=load_connected_plan(self.assembly,self.path,[],self.original,native_tool_cache=cache)
        self.assertEqual(plan['imported_diagnostic_plan']['native_tool_cache']['sha256'],digest(cache))
        cache.write_text('missing a piece')
        with self.assertRaisesRegex(ValueError,'audited cache'):
            load_connected_plan(self.assembly,self.path,[],self.original,native_tool_cache=cache)

    def test_changed_suffix_rejected_even_with_updated_artifact_hash(self):
        self.plan['phases'][4]['end_gap_m']=.05
        self.write_proof()
        with self.assertRaisesRegex(ValueError,'outside'):self.load()

    def test_incorrect_continuous_belt_clock_rejected(self):
        self.plan['belt_cycle']['start_after_settle_s']=7.
        self.write_proof()
        with self.assertRaisesRegex(ValueError,'belt clock'):self.load()

    def test_disconnected_or_changed_reverse_route_rejected(self):
        self.plan['phases'][6]['waypoint_q'][1][0]=.4
        self.write_proof()
        with self.assertRaisesRegex(ValueError,'reverse'):self.load()

    def test_hash_corruption_and_incomplete_certificate_fail_closed(self):
        self.path.write_text(self.path.read_text()+' ')
        with self.assertRaisesRegex(ValueError,'hash mismatch'):self.load()
        self.write_proof()
        (self.folder/'self_120hz.json').write_text(json.dumps(dict(passed=True,complete=False)))
        self.write_proof()
        with self.assertRaisesRegex(ValueError,'Incomplete'):self.load()

    def test_old_route_history_cannot_be_imported_as_fresh_campaign(self):
        self.manifest['fresh_campaign']['inherited_native_breaks']=['Fruit_01']
        self.write_manifest()
        with self.assertRaisesRegex(ValueError,'fresh source'):self.load()

    def test_clear_preparation_does_not_authorize_blocked_or_omitted_entry(self):
        self.write_full_environment(passed=False)
        self.write_proof()
        with self.assertRaisesRegex(ValueError,'Incomplete'):self.load()
        self.write_full_environment(selected_phases=['ready_hold','prepare_ompl_000'])
        self.write_proof()
        with self.assertRaisesRegex(ValueError,'every actual phase'):self.load()

    def test_native_cli_limits_import_to_one_fresh_target_cycle(self):
        from contextlib import redirect_stderr
        import io
        args=['--mode','harvest','--preparation','horizontal_ready',
              '--horizontal-ready-connector','joint','--whole-truss',
              '--harvest-profile','belt_open_close_cycle','--hz','120',
              '--max-truss-cycles','1','--diagnostic-motion-plan','/proof/plan.json']
        self.assertEqual(actual_cli(args).max_truss_cycles,1)
        for bad in (['--max-truss-cycles','2'],['--resume-native-checkpoint','/oldcp'],
                    ['--near-trusses-metadata','/row'],['--rail-to-y','1.8'],['--hz','60']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(args+bad)

    def test_staging_distance_override_changes_only_optin_cli_scope(self):
        from contextlib import redirect_stderr
        import io
        self.assertEqual(actual_cli([]).entry_retract_mm,100.)
        base=['--mode','harvest','--preparation','horizontal_ready',
              '--harvest-profile','belt_open_close_cycle']
        self.assertEqual(actual_cli(base+['--entry-retract-mm','250']).entry_retract_mm,250.)
        for bad in (['--entry-retract-mm','250'],base+['--entry-retract-mm','nan'],
                    base+['--entry-retract-mm','49'],base+['--entry-retract-mm','301']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(bad)


if __name__=='__main__':unittest.main()
