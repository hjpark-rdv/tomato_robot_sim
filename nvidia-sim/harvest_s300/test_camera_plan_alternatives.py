import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from camera_plan_alternatives import (alternative_key, camera_plan_alternatives,
                                      rejection_sets, selected_seed_evidence,same_group_auto_fallbacks)


class CameraPlanAlternativesTests(unittest.TestCase):
    def setUp(self):
        self.headings = [0, 15, -15, 30, -30]

    def test_default_preserves_configured_seed_and_existing_heading_coverage(self):
        rows = camera_plan_alternatives(self.headings, 8)
        self.assertEqual([alternative_key(row) for row in rows], [(float(a), 8) for a in self.headings])
        self.assertEqual([row['index'] for row in rows], list(range(5)))

    def test_none_auto_is_a_single_existing_default_not_all_nine_seeds(self):
        rows = camera_plan_alternatives(self.headings, None)
        self.assertEqual(len(rows), 5)
        self.assertTrue(all(row['seed_index'] is None for row in rows))

    def test_heading_round_robin_inner_seed_outer_deterministic(self):
        rows = camera_plan_alternatives(self.headings, 8, seed_indices=[8, 2, 0])
        expected = [(float(a), seed) for seed in [8, 2, 0] for a in self.headings]
        self.assertEqual([alternative_key(row) for row in rows], expected)
        self.assertEqual(rows, camera_plan_alternatives(self.headings, 8, seed_indices=[8, 2, 0]))

    def test_rejection_isolation_heading_and_seed_and_no_input_mutation(self):
        initial = {('fruit1', 'fruit2')}
        rows = camera_plan_alternatives([0, 15], 8, seed_indices=[8, 2])
        states = rejection_sets(rows, initial)
        states[(0., 8)].add(('fruit3', 'fruit4'))
        for key in [(15., 8), (0., 2), (15., 2)]:
            self.assertEqual(states[key], initial)
        self.assertEqual(initial, {('fruit1', 'fruit2')})

    def test_explicit_seed_actual_metadata_is_checked(self):
        row = camera_plan_alternatives([15], 8, seed_indices=[2])[0]
        evidence = selected_seed_evidence(row, dict(selected_seed_index=2))
        self.assertEqual(evidence['requested_staging_seed_index'], 2)
        self.assertEqual(evidence['selected_staging_seed_index'], 2)
        self.assertEqual(evidence['alternative_key'], [15., 2])
        with self.assertRaisesRegex(ValueError, 'differs'):
            selected_seed_evidence(row, dict(selected_seed_index=8))

    def test_auto_metadata_records_found_branch_and_does_not_confuse_seed_zero(self):
        auto = camera_plan_alternatives([0], None)[0]
        explicit = camera_plan_alternatives([0], 0)[0]
        self.assertNotEqual(alternative_key(auto), alternative_key(explicit))
        evidence = selected_seed_evidence(auto, dict(selected_seed_index=5))
        self.assertIsNone(evidence['requested_staging_seed_index'])
        self.assertEqual(evidence['selected_staging_seed_index'], 5)
        self.assertEqual(evidence['staging_seed_selection'], 'auto')

    def test_existing_nine_seeds_finite_no_budget_increase_or_mutation(self):
        seeds = list(range(9))
        rows = camera_plan_alternatives(self.headings, 8, seed_indices=seeds)
        self.assertEqual(len(rows), 45)
        self.assertEqual(seeds, list(range(9)))
        self.assertEqual(self.headings, [0, 15, -15, 30, -30])

    def test_bad_explicit_seed_lists_rejected(self):
        for seeds in [[], [0, 0], [9], [-1], [None], [True], [2.0], list(range(10))]:
            with self.subTest(seeds=seeds), self.assertRaises(ValueError):
                camera_plan_alternatives(self.headings, 8, seed_indices=seeds)

    def test_bad_configured_seed_rejected(self):
        for seed in [False, -1, 9, float('nan'), 2.5]:
            with self.subTest(seed=seed), self.assertRaises(ValueError):
                camera_plan_alternatives(self.headings, seed)

    def test_bad_or_duplicate_headings_rejected(self):
        for headings in [[], [0, -0.], [1, 1.], [np.nan], [np.inf], [True], [181]]:
            with self.subTest(headings=headings), self.assertRaises(ValueError):
                camera_plan_alternatives(headings, 8)

    def test_duplicate_alternative_keys_rejected(self):
        rows = camera_plan_alternatives([0], 8)
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            rejection_sets(rows+rows)

    def test_missing_or_invalid_actual_seed_cannot_be_success_evidence(self):
        row = camera_plan_alternatives([0], None)[0]
        for planning in [{}, {'selected_seed_index': None}, {'selected_seed_index': True}, {'selected_seed_index': 9}]:
            with self.subTest(planning=planning), self.assertRaises(ValueError):
                selected_seed_evidence(row, planning)

    def test_auto_fallback_requires_exact_audit_witness_and_retains_same_group_heading(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);plan=root/'plan.json';plan.write_text('{"fixture":true}')
            sha=hashlib.sha256(plan.read_bytes()).hexdigest();scene='a'*64
            report=dict(passed=False,complete=False,input_sha256=sha,scene_sha256=scene,
                phases=[dict(hits=[dict(a='arm',b='tool',intersection=True)])])
            audit=root/'self.json';audit.write_text(json.dumps(report))
            group=dict(fruits=['fruit1','fruit2'],truss_key='Truss_00',aim_world_m=[1.,2.,3.])
            alternative=camera_plan_alternatives([15.],None)[0]
            kw=dict(plan_path=plan,scene_sha256=scene,self_audit_path=audit,environment_audit_path=root/'absent.json')
            rows=same_group_auto_fallbacks(alternative,dict(selected_seed_index=0),group,**kw)
            self.assertEqual([alternative_key(row) for row in rows],[(15.,5),(15.,8)])
            self.assertEqual([row['same_group'] for row in rows],[group,group])
            self.assertFalse(rows[0]['fallback_evidence']['proposal_budget_increased'])
            rows[0]['same_group']['fruits'].append('mutated')
            self.assertEqual(group['fruits'],['fruit1','fruit2']);self.assertEqual(rows[1]['same_group'],group)
            rows=same_group_auto_fallbacks(alternative,dict(selected_seed_index=5),group,**kw)
            self.assertEqual([row['seed_index'] for row in rows],[8])
            self.assertFalse(same_group_auto_fallbacks(alternative,dict(selected_seed_index=0),group,
                tested={(tuple(group['fruits']),15.,5),(tuple(group['fruits']),15.,8)},**kw))
            for changed in (dict(passed=True),dict(phases=[]),dict(input_sha256='b'*64),
                    dict(scene_sha256='b'*64),dict(error='query budget exhausted')):
                audit.write_text(json.dumps(dict(report,**changed)))
                self.assertFalse(same_group_auto_fallbacks(alternative,dict(selected_seed_index=0),group,**kw))
            audit.write_text(json.dumps(report))
            self.assertFalse(same_group_auto_fallbacks(alternative,dict(selected_seed_index=0),group,
                explicit_seed_indices=[5,8],**kw))
            self.assertFalse(same_group_auto_fallbacks(camera_plan_alternatives([15.],5)[0],
                dict(selected_seed_index=5),group,**kw))
            self.assertFalse(same_group_auto_fallbacks(alternative,None,group,**kw))
            plan.unlink();self.assertFalse(same_group_auto_fallbacks(alternative,dict(selected_seed_index=0),group,**kw))

    def test_auto_environment_fallback_accepts_positive_early_stop_but_not_inconclusive(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);plan=root/'plan.json';plan.write_text('{}')
            scene='c'*64;sha=hashlib.sha256(plan.read_bytes()).hexdigest()
            report=dict(passed=False,complete=False,status='blocked',hard_hits=[dict(intersection=True)],
                source_sha256=dict(plan=sha,scene=scene))
            audit=root/'environment.json';audit.write_text(json.dumps(report))
            kw=dict(plan_path=plan,scene_sha256=scene,self_audit_path=root/'absent.json',environment_audit_path=audit)
            alternative=camera_plan_alternatives([0.],None)[0];group=dict(fruits=['a','b'])
            rows=same_group_auto_fallbacks(alternative,dict(selected_seed_index=8),group,**kw)
            self.assertEqual([row['seed_index'] for row in rows],[5])
            self.assertEqual(rows[0]['fallback_evidence']['audit']['kind'],'environment')
            for changed in (dict(status='inconclusive'),dict(hard_hits=[]),dict(error='pipeline unknown')):
                audit.write_text(json.dumps(dict(report,**changed)))
                self.assertFalse(same_group_auto_fallbacks(alternative,dict(selected_seed_index=8),group,**kw))


if __name__ == '__main__':
    unittest.main()
