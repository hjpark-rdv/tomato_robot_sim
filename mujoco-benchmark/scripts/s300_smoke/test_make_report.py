"""Historical trials retain their measurement scope and model provenance."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from make_report import build_harvest


class HarvestReportTests(unittest.TestCase):
    def make_trial(self, root, name, corrected=True, legacy=False):
        folder=root/name
        folder.mkdir()
        model=folder/'source'
        model.mkdir()
        xml=model/'scene.xml'
        xml.write_text('<mujoco><default><geom margin="'+('0' if corrected else '.0005')+
                       '" gap=".0005"/></default></mujoco>')
        summary=dict(model_dir=str(model),source_model_xml_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),
                     mujoco_version='3.15.0',hz=300,physics_steps_by_hz={'60':2,'300':3},
                     simulated_s=1.,loop_wall_s=2.,realtime_factor=.5,stop=None,
                     breaks=[],cycles=[],active_contact_penetration_m=.0009,
                     released_penetration_m=.002,max_contact_force_n=3.,
                     active_contact_criterion_passed=True,first_1mm_violation=None,states_sha256='s'*64)
        if legacy:
            summary['active_tool_penetration_m']=.002
            del summary['active_contact_penetration_m']
            summary['active_contact_criterion_passed']=False
        (folder/'summary.json').write_text(json.dumps(summary))
        return folder

    def test_mixed_historical_scope_is_preserved_without_rewriting_raw_result(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            old=self.make_trial(root,'campaign_legacy',legacy=True,corrected=False)
            current=self.make_trial(root,'campaign_current')
            original=(old/'summary.json').read_bytes()
            report=build_harvest(root,current)
            previous=next(row for row in report['trials'] if row['trial']==old.name)
            self.assertEqual(previous['active_metric_scope'],'tool_only')
            self.assertEqual(previous['active_penetration_mm'],2.)
            self.assertFalse(previous['active_contact_criterion_passed'])
            self.assertEqual((old/'summary.json').read_bytes(),original)
            self.assertIn('완주했으나 물리 기준 초과',(root/'index.html').read_text())

    def test_mapping_comes_from_xml_instead_of_trial_folder_name(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            misleading=self.make_trial(root,'campaign_contact_fixed',corrected=False)
            corrected=self.make_trial(root,'campaign_plain_name')
            report=build_harvest(root,corrected)
            rows={row['trial']:row for row in report['trials']}
            self.assertFalse(rows[misleading.name]['contact_mapping_corrected'])
            self.assertTrue(rows[corrected.name]['contact_mapping_corrected'])

    def test_changed_model_is_rejected_before_publishing_report(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            trial=self.make_trial(root,'campaign_current')
            (trial/'source/scene.xml').write_text('<mujoco/>')
            with self.assertRaisesRegex(ValueError,'model hash mismatch'):
                build_harvest(root,trial)
            self.assertFalse((root/'index.html').exists())

    def test_fruit_numbers_do_not_substitute_for_exact_source_color_policy(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            trial=self.make_trial(root,'campaign_current')
            source=trial/'source'
            (source/'harvest_group_policy.json').write_text(json.dumps(dict(fruit_eligibility=[
                dict(name='Truss_00_Fruit_07',eligible=True),
                dict(name='Truss_00_Fruit_09',eligible=False)])))
            (source/'export.json').write_text(json.dumps(dict(windows=[dict(
                source_trial=str(source),prefix='P01_')])))
            summary=json.loads((trial/'summary.json').read_text())
            summary['breaks']=[dict(window='41',name='P01_Truss_00_Fruit_07'),
                               dict(window='41',name='P01_Truss_00_Fruit_09'),
                               dict(window='29',name='Truss_00_Fruit_07')]
            (trial/'summary.json').write_text(json.dumps(summary))
            classification=build_harvest(root,trial)['detached_eligibility']
            self.assertEqual(classification['eligible'],1)
            self.assertEqual(classification['ineligible'],1)
            self.assertEqual(classification['unknown'],1)


if __name__=='__main__':
    unittest.main()
