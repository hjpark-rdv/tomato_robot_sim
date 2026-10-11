import copy
import unittest
import json
import tempfile
from pathlib import Path
from screen_fresh_truss_preparation import INITIAL_STATE,proposal_variants,require_fresh_campaign,require_completed_base_screen,turn_followup_variants


class FreshPreparationScreenTests(unittest.TestCase):
    def manifest(self):
        return dict(initial_robot_q=[.177,1.34,.218,2.49,.428,2.98,3.14],
            target=dict(name='Truss_01_Fruit_01'),fresh_campaign=dict(
                schema='s300-independent-fresh-near-campaign-v1',enabled=True,initial_state=INITIAL_STATE,
                source_assembly='/an/immutable/source',source_scene_sha256='a'*64,
                truss_key='Truss_01',initial_target_name='Truss_01_Fruit_01',
                inherited_native_breaks=[],accepted_previous_route_history=False))

    def test_fresh_source_keeps_exact_authored_start_and_has_bounded_distinct_connectors(self):
        m=self.manifest();before=copy.deepcopy(m);marker,q=require_fresh_campaign(m,'Truss_01')
        self.assertEqual(q.tolist(),m['initial_robot_q']);self.assertEqual(m,before)
        variants=proposal_variants();self.assertEqual(len(variants),8)
        self.assertEqual([v['horizontal_ready_seed_index']for v in variants[:6]],list(range(6)))
        self.assertEqual([v['preparation_height_first']for v in variants[6:]],[True,False])
        self.assertEqual(len({v['name']for v in variants}),8)

    def test_prior_native_history_or_checkpoint_cannot_be_presented_as_fresh(self):
        for changes in (dict(inherited_native_breaks=[dict(joint='/prior',t=1.)]),
                        dict(accepted_previous_route_history=True),dict(initial_state='restored old checkpoint')):
            m=self.manifest();m['fresh_campaign'].update(changes)
            with self.assertRaisesRegex(ValueError,'independent fresh campaign'):require_fresh_campaign(m,'Truss_01')
        for field in ('route_checkpoint','eligible_attachment_transfer'):
            m=self.manifest();m[field]={'checkpoint':'/an/old/completed/cycle'}
            with self.assertRaisesRegex(ValueError,'earlier route'):require_fresh_campaign(m,'Truss_01')

    def test_wrong_target_or_nonfinite_start_is_rejected_before_planning(self):
        m=self.manifest();m['target']['name']='Truss_02_Fruit_01'
        with self.assertRaisesRegex(ValueError,'first truss'):require_fresh_campaign(m,'Truss_01')
        m=self.manifest();m['initial_robot_q'][3]=float('nan')
        with self.assertRaisesRegex(ValueError,'normal lift/arm'):require_fresh_campaign(m,'Truss_01')


    def test_turn_followup_is_finite_and_requires_all_previous_source_matching_failures(self):
        variants=turn_followup_variants()
        self.assertEqual(len(variants),6)
        self.assertEqual([(v['horizontal_ready_seed_index'],v['horizontal_ready_normalize_turns'],v['ready_lift_offset_m'])for v in variants],
                         [(6,False,-.135),(7,False,-.135),(6,True,-.135),(7,True,-.135),(6,True,-.170),(7,True,-.170)])
        source={'scene':'abc'}
        base=dict(schema='s300-bounded-fresh-preparation-screen-v1',native_runs=0,checkpoint_imported=False,
            source_unchanged=True,source_sha256_after=source,nominal_prefix_pass_count=0,
            proposals=[dict(name=v['name'],status='self_screen_blocked_8hz')for v in proposal_variants()])
        with tempfile.TemporaryDirectory()as folder:
            path=Path(folder)/'summary.json';path.write_text(json.dumps(base))
            proof=require_completed_base_screen(path,source)
            self.assertTrue(proof['all_original_eight_completed']);self.assertEqual(proof['original_nominal_prefix_pass_count'],0)
            for change in ({'native_runs':1},{'source_unchanged':False},{'checkpoint_imported':True},
                           {'nominal_prefix_pass_count':1},{'source_sha256_after':{'scene':'different'}},
                           {'proposals':base['proposals'][:-1]}):
                invalid=dict(base,**change);path.write_text(json.dumps(invalid))
                with self.assertRaisesRegex(ValueError,'all original eight'):require_completed_base_screen(path,source)
            invalid=copy.deepcopy(base);invalid['proposals'][0]['status']='proposal_not_evaluated';path.write_text(json.dumps(invalid))
            with self.assertRaisesRegex(ValueError,'all original eight'):require_completed_base_screen(path,source)

if __name__=='__main__':unittest.main()
