import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch

import numpy as np

import publish_patrol as publication


class PatrolPublicationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.web=self.root/'web';self.web.mkdir()
        publication.write(self.web/'catalog.json',dict(schema='farmily-harvest-replay-catalog-v1',runs=[],
            default_run='old',default_collection='old_collection'))

    def tearDown(self):self.temp.cleanup()

    def report(self,folder,**change):
        record=dict(schema=publication.CAMPAIGN_SCHEMA,status='running',executed=True,
            goal_distinct_trusses=6,prior_campaign=None,initial_checkpoint=None,
            training_eligible=False,harvest_success=None)
        record.update(change);publication.write(folder/'camera_patrol.json',record)
        return record

    def fixture(self):
        assembly=self.root/'assembly';assembly.mkdir();(assembly/'scene.usda').write_text('unchanged scene')
        manifest=dict(scene_sha256=publication.sha(assembly/'scene.usda'),plant={'source_plant':'actual_plant'})
        publication.write(assembly/'motion_manifest.json',manifest)
        trial=self.root/'trials'/'actual';trial.mkdir(parents=True)
        np.savez(trial/'states.npz',time=np.array([0.,.5,1.]))
        summary=dict(completed=True,stop_reason=None,model_sha256=manifest['scene_sha256'])
        publication.write(trial/'summary.json',summary)
        final=trial.parent/(trial.name+'_cases_results.json')
        publication.write(final,[dict(name=trial.name,returncode=0,timeout=False,summary=summary)])
        return dict(trial=str(trial),assembly=str(assembly),index=0,campaign='actual_campaign',phase='scan'),summary,final

    def test_waiting_for_new_campaign_does_not_create_it(self):
        campaign=self.root/'controller_must_create_me'
        with self.assertRaises(publication.ProducerPending):publication.campaign_config(campaign,'fresh')
        self.assertFalse(campaign.exists())
        result=subprocess.run([sys.executable,str(Path(publication.__file__)), '--campaign',str(campaign),
            '--collection','fresh','--watch','--max-hours','.00001'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertFalse(campaign.exists())
        ledger=publication.read(campaign.with_name(campaign.name+'_web_publication')/'ledger.json')
        self.assertEqual(ledger['status'],'time_budget_exhausted');self.assertEqual(ledger['published'],[])

    def test_sha_proven_parent_chain_is_ordered_and_tampering_is_rejected(self):
        parent=self.root/'parent';child=self.root/'child'
        self.report(parent,status='finite_decision_budget_exhausted')
        publication.write(parent/'camera_patrol_queue.json',{'recorded':'queue'})
        cp=parent/'actual_checkpoint'
        self.report(child,initial_checkpoint={'checkpoint':str(cp)},prior_campaign=dict(root=str(parent),
            report_sha256=publication.sha(parent/'camera_patrol.json'),
            queue_sha256=publication.sha(parent/'camera_patrol_queue.json'),accepted_checkpoint=str(cp),rollback=False))
        result=publication.campaign_config(child,'fresh')
        self.assertEqual(result['campaigns'],[str(parent),str(child)])
        self.assertFalse(result['terminal'])
        publication.write(parent/'camera_patrol_queue.json',{'mutated':True})
        with self.assertRaisesRegex(ValueError,'SHA mismatch'):publication.campaign_config(child,'fresh')

    def test_report_does_not_promote_claimed_goal_or_unknown_status(self):
        campaign=self.root/'campaign'
        self.report(campaign,status='finite_decision_budget_exhausted',goal_reached=True,achieved_distinct_trusses=6)
        result=publication.campaign_config(campaign,'fresh')
        self.assertTrue(result['terminal']);self.assertNotIn('achieved_distinct_trusses',result)
        self.report(campaign,status='new_unknown_status')
        self.assertFalse(publication.campaign_config(campaign,'fresh')['terminal'])

    def test_wait_for_final_benchmark_and_reject_mutated_summary(self):
        row,summary,final=self.fixture();final.unlink()
        self.assertIsNone(publication.source_proof(row))
        publication.write(final,[dict(name='actual',returncode=0,timeout=False,summary=summary)])
        self.assertEqual(publication.source_proof(row)['counted_trusses'],[])
        summary['completed']=False;publication.write(Path(row['trial'])/'summary.json',summary)
        with self.assertRaisesRegex(ValueError,'summary mismatch'):publication.source_proof(row)

    def test_timeout_or_nonzero_exit_cannot_receive_harvest_credit(self):
        row,summary,final=self.fixture()
        for code,timeout in [(1,False),(0,True)]:
            publication.write(final,[dict(name='actual',returncode=code,timeout=timeout,summary=summary)])
            with patch.object(publication,'measured_harvested_trusses',return_value=['must_not_count']) as count:
                proof=publication.source_proof(row)
            self.assertEqual(proof['counted_trusses'],[]);count.assert_not_called()

    def test_unproven_checkpoint_branch_is_not_joined(self):
        row,_,_=self.fixture();trial=Path(row['trial'])
        publication.write(trial/'resume_input.json',{'checkpoint':str(self.root/'other_history')})
        with self.assertRaisesRegex(ValueError,'Unproven branch/reset'):
            publication.source_proof(row,{'trial':str(self.root/'accepted_history')})

    def test_independent_retries_are_not_silently_stitched(self):
        campaign=self.root/'campaign'
        publication.write(campaign/'replay_segments.json',dict(schema='s300-patrol-actual-replay-sources-v1',
            segments=[dict(trial=str(self.root/'actual'),independent_retry=True)]))
        with self.assertRaisesRegex(ValueError,'Independent retry'):
            publication.sources({'campaigns':[str(campaign)]})

    def fake_publisher(self):
        p=Mock();p.output=self.root/'publication';p.output.mkdir();p.ledger={'errors':[]};p.pending=[]
        return p

    def test_terminal_flush_drains_all_ready_rows_then_stops(self):
        p=self.fake_publisher();p.ledger['producer_terminal']=True
        p.publish_once.side_effect=[True,True,False]
        result=publication.run_watcher(p,watch=True,sleep=Mock(side_effect=AssertionError('No terminal wait')))
        self.assertEqual(p.publish_once.call_count,3)
        self.assertEqual(result['status'],'terminal_final_flush_complete')

    def test_terminal_missing_actual_artifact_is_reported_not_success(self):
        p=self.fake_publisher();p.ledger['producer_terminal']=True
        p.pending=[dict(trial='actual',reason='missing final benchmark')];p.publish_once.return_value=False
        with self.assertRaisesRegex(RuntimeError,'failed publication readiness'):
            publication.run_watcher(p,watch=True)
        self.assertEqual(p.ledger['status'],'terminal_unpublished_sources')

    def test_validator_failure_is_logged_and_never_marked_published(self):
        p=self.fake_publisher();p.ledger['published']=[]
        p.publish_once.side_effect=ValueError('Independent validator rejected actual bundle')
        with self.assertRaisesRegex(ValueError,'validator rejected'):
            publication.run_watcher(p,watch=True)
        self.assertEqual(p.ledger['status'],'stopped_on_publication_error')
        self.assertEqual(p.ledger['published'],[]);self.assertEqual(len(p.ledger['errors']),1)

    def synthetic_publish(self,*,partial=False,valid=True):
        row,_,_=self.fixture();proof=publication.source_proof(row)
        if partial:proof.update(record_kind='partial_diagnostic',counted_trusses=[],harvest_credit=0)
        config=dict(collection_id='fresh',campaigns=[],fixed_segments=[row],goal_distinct_trusses=6,
            default_after_verified_segments=3,terminal=False)
        p=publication.Publisher(lambda:config,self.root/'publisher',web=self.web)
        commands=[]
        def command(argv,**kwargs):
            commands.append(argv)
            if Path(argv[1]).name=='export_replay.py':
                output=Path(argv[argv.index('--output')+1]);output.mkdir(parents=True)
                publication.write(output/'replay.json',dict(source=dict(trial=proof['trial'],assembly=proof['assembly'],
                    states_sha256=proof['states_sha256'],manifest_sha256=proof['manifest_sha256'])))
            else:
                publication.write(Path(argv[argv.index('--output')+1]),dict(passed=valid,
                    source_trial=proof['trial'],source_assembly=proof['assembly'],source_states_sha256=proof['states_sha256']))
        with patch.object(publication,'source_proof',return_value=proof), \
             patch.object(publication.subprocess,'run',side_effect=command), \
             patch.object(publication,'register') as register,patch.object(p,'refresh_collection'):
            if valid:
                self.assertTrue(p.publish_once());self.assertFalse(p.publish_once())
                self.assertEqual(register.call_count,1)
                self.assertEqual(register.call_args.kwargs['allow_partial_diagnostic'],partial)
            else:
                with self.assertRaisesRegex(ValueError,'validation failed'):p.publish_once()
                register.assert_not_called()
        return p,commands

    def test_export_validate_register_order_and_publish_idempotence(self):
        p,commands=self.synthetic_publish()
        self.assertEqual([Path(row[1]).name for row in commands],['export_replay.py','validate_replay.py'])
        self.assertEqual(len(p.ledger['published']),1);self.assertEqual(p.ledger['diagnostics'],[])

    def test_failed_independent_validation_blocks_catalog_registration(self):
        p,_=self.synthetic_publish(valid=False)
        self.assertEqual(p.ledger['published'],[])
        self.assertEqual(publication.read(self.web/'catalog.json')['runs'],[])

    def test_partial_export_and_validation_both_require_existing_explicit_opt_in(self):
        p,commands=self.synthetic_publish(partial=True)
        self.assertEqual(p.ledger['published'],[]);self.assertEqual(len(p.ledger['diagnostics']),1)
        self.assertTrue(all('--allow-partial-diagnostic' in command for command in commands))

    def test_partial_record_uses_existing_validator_and_zero_credit(self):
        row,_,final=self.fixture();trial=Path(row['trial'])
        (trial/'summary.json').unlink();(trial/'states.npz').rename(trial/'states_partial.npz')
        publication.write(trial/'partial_trace.json',{'original':'descriptor'})
        publication.write(final,[dict(name='actual',returncode=1,timeout=False,summary=None)])
        partial=dict(states_path=trial/'states_partial.npz',result=dict(completed=False,
            stop_reason='software_exception_partial_trace'))
        with patch('partial_recording.partial_recording',return_value=partial) as original_validator, \
             patch('partial_recording.published_partial_source',return_value={'diagnostic_only':True}):
            proof=publication.source_proof(row)
        original_validator.assert_called_once_with(Path(row['assembly']),trial)
        self.assertTrue(proof['diagnostic_only']);self.assertEqual(proof['counted_trusses'],[])
        self.assertEqual(proof['harvest_credit'],0);self.assertFalse(proof['checkpoint_eligible'])
        self.assertIsNone(proof['summary_sha256']);self.assertFalse((trial/'summary.json').exists())

    def test_partial_collection_is_separate_nondefault_and_cannot_claim_goal(self):
        p=publication.Publisher(lambda:{},self.root/'publisher',web=self.web)
        p.ledger['diagnostics']=[dict(run_id='partial',source_proof={'trial':'actual','counted_trusses':[]})]
        with patch.object(p,'extend_collection') as extend:
            p.refresh_collection(dict(collection_id='fresh',goal_distinct_trusses=6,terminal=True))
        self.assertEqual(extend.call_count,1)
        args=extend.call_args.args
        self.assertEqual(args[0],'fresh_diagnostics')
        self.assertEqual(args[-2],'independent_trials');self.assertIs(args[-1],False)
        self.assertEqual(p.ledger['achieved_distinct_trusses'],0);self.assertFalse(p.ledger['goal_reached'])
        catalog=publication.read(self.web/'catalog.json');self.assertEqual(catalog['default_run'],'old')

    def test_numeric_lifetime_and_collection_id_limits(self):
        for value in (0,-1,9,float('inf'),float('nan')):
            with self.assertRaises(ValueError):publication.run_watcher(self.fake_publisher_unique(),max_hours=value)
        for value in ('../escape','id with space','x'*81):
            with self.assertRaises(ValueError):publication.checked_id(value)

    def fake_publisher_unique(self):
        p=Mock();p.output=self.root;p.ledger={'errors':[]};p.pending=[];return p


if __name__=='__main__':unittest.main()
