import copy
import unittest
import numpy as np
from row_harvest_targets import (scoped_trusses,eligible_truss_fruits,
                                group_truss_fruits,select_scoped_group)
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


if __name__=='__main__':unittest.main()
