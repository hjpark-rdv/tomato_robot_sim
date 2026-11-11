import copy
import unittest
import numpy as np
from row_harvest_targets import (scoped_trusses,eligible_truss_fruits,
                                group_truss_fruits,select_scoped_group,select_one_bed)
from motion_assets import select_build_target


class RowTargetTests(unittest.TestCase):
    def setUp(self):
        self.fruits=[]
        for truss,x in [(0,-.79),(1,-.59),(2,-.43)]:
            for i in range(1,11):
                name=f'Truss_{truss:02}_Fruit_{i:02}'
                self.fruits.append(dict(name=name,path='/World/ElasticTarget/'+name,
                                        joint='/World/ElasticTarget/HarvestJoints/'+name,
                                        center=[x,1.+i*.01,1.+truss*.1+i*.002]))
        self.manifest=dict(plant=dict(source_plant='/World/Greenhouse/Plants/Plant_22',
                                     root='/World/ElasticTarget',fruits=self.fruits))
        self.inventory=dict(trusses=[dict(plant_id=22,truss_index=i,
            root_position_world_m=[-.68,1.,.32],gutter_center_x=-.775,
            rail_near=True,fruit_count=10)for i in range(3)])

    def test_actual_placement_scope_excludes_back_truss_and_preserves_twenty_identities(self):
        original=copy.deepcopy(self.manifest);scope=scoped_trusses(self.manifest,self.inventory)
        self.assertEqual(scope['eligible_truss_keys'],['Truss_01','Truss_02'])
        self.assertEqual(scope['trusses'][0]['exclusion_reason'],'fruit_centroid_behind_gutter')
        self.assertEqual(scope['eligible_fruit_count'],20)
        chosen=eligible_truss_fruits(self.manifest,scope)
        self.assertEqual(chosen,self.fruits[10:]);self.assertIs(chosen[0],self.fruits[10])
        self.assertEqual(original,self.manifest)
        self.assertTrue(all(r['reachability'].startswith('unknown')for r in scope['trusses']))

    def test_opposite_stem_remains_excluded_even_if_its_fruits_face_the_rail(self):
        inv=copy.deepcopy(self.inventory)
        for r in inv['trusses']:r.update(root_position_world_m=[-.88,1.,.32],rail_near=False)
        scope=scoped_trusses(self.manifest,inv)
        self.assertEqual(scope['eligible_fruit_count'],0)
        self.assertTrue(all(r['exclusion_reason']=='source_stem_opposite_gutter_side'for r in scope['trusses']))

    def test_scope_is_not_hardcoded_to_truss_numbers_or_left_side(self):
        m=copy.deepcopy(self.manifest);inv=copy.deepcopy(self.inventory)
        for f in m['plant']['fruits']:f['center'][0]*=-1
        for r in inv['trusses']:r.update(root_position_world_m=[.68,1.,.32],gutter_center_x=.775)
        self.assertEqual(scoped_trusses(m,inv)['eligible_truss_keys'],['Truss_01','Truss_02'])
        for f in m['plant']['fruits'][:10]:f['center'][0]=.5
        self.assertEqual(scoped_trusses(m,inv)['eligible_truss_keys'],['Truss_00','Truss_01','Truss_02'])

    def test_old_covered_metadata_and_actual_inventory_agree(self):
        old=dict(neighbor_placements=[dict(id=22,position=[-.68,1.,.32],
                                           gutter_center_x=-.775,trusses=[{}, {}, {}])])
        self.assertEqual(scoped_trusses(self.manifest,old),scoped_trusses(self.manifest,self.inventory))

    def test_ambiguous_or_wrong_identity_metadata_is_rejected(self):
        inv=copy.deepcopy(self.inventory);inv['trusses'].append(inv['trusses'][0])
        with self.assertRaisesRegex(ValueError,'Duplicate source truss'):scoped_trusses(self.manifest,inv)
        inv=copy.deepcopy(self.inventory);inv['trusses'][0]['fruit_count']=9
        with self.assertRaisesRegex(ValueError,'fruit count mismatch'):scoped_trusses(self.manifest,inv)
        m=copy.deepcopy(self.manifest);m['plant']['fruits'][0]['path']='/World/OtherPlant/Truss_00_Fruit_01'
        with self.assertRaisesRegex(ValueError,'active plant'):scoped_trusses(m,self.inventory)
        inv=copy.deepcopy(self.inventory);inv['trusses'][0]['rail_near']=False
        with self.assertRaisesRegex(ValueError,'disagree'):scoped_trusses(self.manifest,inv)
        inv=copy.deepcopy(self.inventory);inv['trusses'][0]['root_position_world_m'][0]=float('nan')
        with self.assertRaisesRegex(ValueError,'Nonfinite'):scoped_trusses(self.manifest,inv)

    def test_group_selection_never_mix_two_trusses_even_when_fruits_are_nearby(self):
        fruits=[self.fruits[i]for i in (10,20,11,21)]
        paths=[f['path']for f in fruits]
        poses=np.array([[0,0,1.,0,0,0,1],[.001,0,1.001,0,0,0,1],
                        [.015,0,1.01,0,0,0,1],[.016,0,1.011,0,0,0,1]])
        group=select_scoped_group(fruits,[],paths,poses,{})
        self.assertEqual(group['fruits'],[self.fruits[10]['name'],self.fruits[11]['name']])
        self.assertEqual(group_truss_fruits(group,self.fruits),self.fruits[10:20])
        detached=[dict(joint=f['joint'])for f in self.fruits[10:20]]
        next_group=select_scoped_group(fruits,detached,paths,poses,{})
        self.assertEqual(next_group['truss_key'],'Truss_02')
        with self.assertRaisesRegex(ValueError,'crosses'):
            group_truss_fruits(dict(fruits=[self.fruits[10]['name'],self.fruits[20]['name']]),self.fruits)

    def test_base_build_target_is_exact_and_keeps_legacy_default(self):
        self.assertIs(select_build_target(self.fruits),self.fruits[21])
        self.assertIs(select_build_target(self.fruits,'Truss_01_Fruit_01'),self.fruits[10])
        with self.assertRaisesRegex(ValueError,'exactly one'):select_build_target(self.fruits,'Truss_19_Fruit_01')
        with self.assertRaisesRegex(ValueError,'exactly one'):select_build_target(self.fruits+[self.fruits[21]])

    def test_build_reference_for_a_plant_without_truss02(self):
        fruits=list(reversed(self.fruits[:20]))
        before=list(fruits)
        self.assertIs(select_build_target(fruits),self.fruits[0])
        self.assertEqual(fruits,before)
        with self.assertRaisesRegex(ValueError,'exactly one'):
            select_build_target(fruits,'Truss_02_Fruit_02')
        with self.assertRaisesRegex(ValueError,'exactly one'):select_build_target([])
        with self.assertRaisesRegex(ValueError,'exactly one'):
            select_build_target(fruits+[self.fruits[0]])


class SingleBedSelectionTests(unittest.TestCase):
    def setUp(self):
        self.stations=[dict(plant_id=p,truss_index=t,station_y_m=y,
                           clearance=dict(passed=True)) for p,t,y in
                       [(22,2,1.58),(27,0,2.652643),(41,0,3.293767),(29,0,3.222385)]]
        self.inventory=dict(trusses=[dict(plant_id=p,truss_index=t,
            gutter_center_x=gutter,root_position_world_m=[root,y,.32],
            fruit_centroid_world_m=[center,y+.1,.8],rail_near=True) for p,t,gutter,root,center,y in
            [(22,2,-.775,-.683,-.429,1.58),(27,0,-.775,-.677,-.580,2.652643),
             (41,0,.775,.679,.578,3.293767),(29,0,-.775,-.673,-.530,3.222385)]])

    def test_one_bed_preserves_order_evidence_and_input_without_changing_collision_scope(self):
        stations=copy.deepcopy(self.stations);inventory=copy.deepcopy(self.inventory)
        result=select_one_bed(self.stations,self.inventory,-.775)
        self.assertEqual([r['plant_id'] for r in result['stations']],[22,27,29])
        self.assertEqual(result['excluded'][0]['plant_id'],41)
        self.assertEqual(result['excluded'][0]['selection_exclusion_reason'],'opposite_bed')
        self.assertFalse(result['collision_scope_changed'])
        self.assertTrue(result['preserves_input_order'])
        self.assertEqual(self.stations,stations);self.assertEqual(self.inventory,inventory)
        result['stations'][0]['clearance']['passed']=False
        self.assertTrue(self.stations[0]['clearance']['passed'])
        self.assertTrue(all(r['bed_selection']['same_bed'] for r in result['stations']))
        self.assertTrue(all(r['bed_selection']['gutter_center_x']==-.775 for r in result['stations']))

    def test_near_side_of_opposite_bed_is_still_excluded_and_right_bed_can_be_selected(self):
        result=select_one_bed(self.stations,self.inventory,.775)
        self.assertEqual([r['plant_id'] for r in result['stations']],[41])
        self.assertEqual(len(result['excluded']),3)
        self.assertTrue(all(r['selection_exclusion_reason']=='opposite_bed' for r in result['excluded']))

    def test_same_bed_back_stem_and_back_fruit_centroid_are_distinct_exclusions(self):
        self.inventory['trusses'][0].update(root_position_world_m=[-.85,1.58,.32],rail_near=False)
        self.inventory['trusses'][1]['fruit_centroid_world_m'][0]=-.81
        result=select_one_bed(self.stations,self.inventory,-.775)
        self.assertEqual([r['plant_id'] for r in result['stations']],[29])
        self.assertEqual([r['selection_exclusion_reason'] for r in result['excluded']],
                         ['source_stem_opposite_gutter_side','fruit_centroid_behind_gutter','opposite_bed'])

    def test_duplicate_missing_or_nonfinite_source_and_proposals_are_rejected(self):
        bad=copy.deepcopy(self.inventory);bad['trusses'].append(bad['trusses'][0])
        with self.assertRaisesRegex(ValueError,'Duplicate source truss'):
            select_one_bed(self.stations,bad,-.775)
        with self.assertRaisesRegex(ValueError,'Duplicate station truss'):
            select_one_bed(self.stations+[self.stations[0]],self.inventory,-.775)
        with self.assertRaisesRegex(ValueError,'missing exact'):
            select_one_bed([dict(plant_id=999,truss_index=0)],self.inventory,-.775)
        for field in ['root_position_world_m','fruit_centroid_world_m']:
            bad=copy.deepcopy(self.inventory);bad['trusses'][0][field][0]=float('nan')
            with self.assertRaisesRegex(ValueError,'Nonfinite'):
                select_one_bed(self.stations,bad,-.775)
        with self.assertRaisesRegex(ValueError,'Finite'):
            select_one_bed(self.stations,self.inventory,float('inf'))
        with self.assertRaisesRegex(ValueError,'cannot coincide'):
            select_one_bed(self.stations,self.inventory,0.)
        bad=copy.deepcopy(self.inventory);bad['trusses'][0]['rail_near']=False
        with self.assertRaisesRegex(ValueError,'disagree'):
            select_one_bed(self.stations,bad,-.775)


if __name__=='__main__':unittest.main()
