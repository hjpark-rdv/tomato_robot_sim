import json
from pathlib import Path
import tempfile
import unittest

from register_collection import register_collection


class CollectionRegistration(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.catalog=dict(schema='farmily-harvest-replay-catalog-v1',runs=[])
        for run_id in ('first','second'):
            folder=self.root/'assets'/run_id;folder.mkdir(parents=True)
            (folder/'replay.json').write_text(json.dumps(dict(schema='farmily-harvest-replay-v1',
                source=dict(all_frames_exported=True,physics_rerun=False))))
            self.catalog['runs'].append(dict(id=run_id,replay_url=f'./assets/{run_id}/replay.json'))
        (self.root/'catalog.json').write_text(json.dumps(self.catalog))

    def tearDown(self):
        self.temp.cleanup()

    def test_registration_preserves_actual_order_and_existing_runs(self):
        record=register_collection('test','Ordered test',['second','first'],root=self.root,
                                   provenance={'selected_trusses':6},continuity='measured_state_handoff')
        result=json.loads((self.root/'catalog.json').read_text())
        self.assertEqual(result['runs'],self.catalog['runs'])
        self.assertEqual(record['runs'],['second','first'])
        self.assertEqual(result['collections'],[record])
        self.assertEqual(record['provenance'],{'selected_trusses':6})
        self.assertEqual(record['continuity'],'measured_state_handoff')
        with self.assertRaisesRegex(ValueError,'already registered'):
            register_collection('test','Ordered test',['first'],root=self.root)

    def test_missing_duplicate_and_unsafe_runs_rejected(self):
        with self.assertRaisesRegex(ValueError,'Unknown collection continuity'):
            register_collection('test','test',['first'],root=self.root,continuity='fictional_continuous')
        for runs in (['first','first'],['unknown'],[]):
            with self.assertRaises(ValueError):
                register_collection('test','test',runs,root=self.root)
        self.catalog['runs'][0]['replay_url']='../../outside.json'
        (self.root/'catalog.json').write_text(json.dumps(self.catalog))
        with self.assertRaisesRegex(ValueError,'Missing local replay bundle'):
            register_collection('test','test',['first'],root=self.root)

    def test_unverified_or_missing_recording_rejected(self):
        replay=self.root/'assets/first/replay.json'
        data=json.loads(replay.read_text());data['source']['physics_rerun']=True
        replay.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError,'preserved actual states'):
            register_collection('test','test',['first'],root=self.root)
        replay.unlink()
        with self.assertRaisesRegex(ValueError,'Missing local replay bundle'):
            register_collection('test','test',['first'],root=self.root)

    def test_reports_are_actual_copied_files_without_fictional_replay(self):
        report=self.root/'actual_report.json';report.write_text('{"attempts":6}')
        log=self.root/'prephysics_failure.log';log.write_text('Rejected before physics; no recorded states')
        record=register_collection('test','test',['first'],root=self.root,
                                   source_report=report,diagnostics=[log])
        report_copy=self.root/record['report_url']
        log_copy=self.root/record['diagnostics_urls'][0]['url']
        self.assertEqual(report_copy.read_bytes(),report.read_bytes())
        self.assertEqual(log_copy.read_bytes(),log.read_bytes())
        self.assertFalse(report_copy.is_symlink());self.assertFalse(log_copy.is_symlink())
        self.assertEqual(record['runs'],['first'])
        self.assertEqual(record['continuity'],'unknown')

    def test_opt_in_default_changes_both_selectors_and_other_registration_preserves_it(self):
        register_collection('main','Measured main',['second','first'],root=self.root,make_default=True)
        register_collection('diagnostics','Independent diagnostics',['first'],root=self.root)
        result=json.loads((self.root/'catalog.json').read_text())
        self.assertEqual(result['default_collection'],'main')
        self.assertEqual(result['default_run'],'second')
        self.assertEqual(result['runs'],self.catalog['runs'])
        self.assertEqual([row['id'] for row in result['collections']],['main','diagnostics'])

    def test_partial_record_is_only_an_independent_nondefault_diagnostic(self):
        path=self.root/'assets/first/replay.json';data=json.loads(path.read_text())
        data['source']['record_kind']='partial_diagnostic';path.write_text(json.dumps(data))
        for continuity,default in [('unknown',False),('measured_state_handoff',False),('independent_trials',True)]:
            with self.assertRaisesRegex(ValueError,'Partial diagnostics'):
                register_collection('bad','bad',['first'],root=self.root,continuity=continuity,make_default=default)
        record=register_collection('faults','Partial diagnostics',['first'],root=self.root,continuity='independent_trials')
        self.assertEqual(record['continuity'],'independent_trials')
        self.assertNotIn('default_run',json.loads((self.root/'catalog.json').read_text()))


if __name__=='__main__':
    unittest.main()
