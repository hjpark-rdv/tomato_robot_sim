import unittest
from plan_row_stations import (station_candidates,initial_group,screen_selection,
    screen_query_proposals,validate_preparation_variants,preparation_variant_arguments)


class RowStationTests(unittest.TestCase):
    def test_unspecified_preparation_variants_preserve_exact_station_proposals(self):
        row=dict(fruit_centroid_world_m=[-.5,1.5,.8])
        self.assertEqual(screen_query_proposals(row),station_candidates(row))
        self.assertEqual(screen_query_proposals(row,variants=None,station_index=None),
                         station_candidates(row))

    def test_two_actual_controls_at_one_existing_station_are_finite(self):
        variants=[dict(name='joint0',horizontal_ready_connector='joint',horizontal_ready_seed_index=0),
                  dict(name='joint6',horizontal_ready_connector='joint',horizontal_ready_seed_index=6)]
        row=dict(fruit_centroid_world_m=[-.5,1.5,.8])
        queries=screen_query_proposals(row,variants=variants,station_index=0)
        self.assertEqual(len(queries),2)
        self.assertEqual([q['station_y_m'] for q in queries],[1.566,1.566])
        self.assertEqual([q['preparation_variant']['horizontal_ready_seed_index'] for q in queries],[0,6])
        self.assertEqual(preparation_variant_arguments(queries[1]['preparation_variant']),
            ['--horizontal-ready-connector','joint','--horizontal-ready-seed-index','6'])
        self.assertEqual(len(screen_query_proposals(row,variants=variants)),4)
        # Clamping can remove station1; never silently retry station0.
        self.assertEqual(screen_query_proposals(dict(fruit_centroid_world_m=[0,5.3,.8]),
            variants=variants,station_index=1),[])

    def test_variant_names_cannot_hide_identical_controls_or_change_physics(self):
        valid=dict(name='joint0',horizontal_ready_connector='joint',horizontal_ready_seed_index=0)
        for variants in ([],[valid,dict(valid,name='renamed')],
            [dict(valid,horizontal_ready_seed_index=True)],
            [dict(valid,horizontal_ready_seed_index=9)],
            [dict(valid,horizontal_ready_connector='cartesian')],
            [dict(valid,fruit_contact_limit_mm=1.)]):
            with self.assertRaises(ValueError):validate_preparation_variants(variants)
        for index in (True,-1,2,.5):
            with self.assertRaises(ValueError):screen_query_proposals(
                dict(fruit_centroid_world_m=[0,1,1]),station_index=index)

    def test_current_screen_scope_excludes_behind_gutter_without_claiming_impossible(self):
        def row(root,center):
            return dict(root_position_world_m=[root,1,.3],fruit_centroid_world_m=[center,1,1],
                gutter_center_x=-.775,rail_near=root>-.775)
        inventory=dict(trusses=[row(-.5,-.4),row(-.5,-.9),row(-.9,-.4)])
        selected,excluded=screen_selection(inventory,eligible_only=True)
        self.assertEqual(len(selected),1)
        self.assertEqual([r['selection_exclusion_reason'] for r in excluded],
            ['fruit_centroid_behind_gutter','opposite_side_stem'])
        self.assertTrue(all(r['reachability'].startswith('not inferred') for r in excluded))
        self.assertEqual(len(screen_selection(inventory)[0]),2)
        inventory['trusses'][0]['rail_near']=False
        with self.assertRaises(ValueError):screen_selection(inventory)
    def test_station_queries_are_bounded_and_deduplicated(self):
        self.assertEqual(len(station_candidates(dict(fruit_centroid_world_m=[.6,5.3,.8]))),1)
        self.assertEqual(station_candidates(dict(fruit_centroid_world_m=[.6,5.3,.8]))[0]['station_y_m'],5.)
        values=station_candidates(dict(fruit_centroid_world_m=[-.5,1.5,.8]))
        self.assertEqual([v['station_y_m'] for v in values],[1.566,1.386])
        for center in ([0,float('nan'),1],[0,1]):
            with self.assertRaises(ValueError):station_candidates(dict(fruit_centroid_world_m=center))

    def test_actual_group_respects_highest_pair_and_cluster_identity(self):
        row=dict(plant_id=22,truss_index=2,fruits={
            'Fruit_01':[-.4,1.5,1.2],'Fruit_02':[-.42,1.5,1.18],
            'Fruit_11':[-.35,1.5,.99]})
        fruits,paths,poses,group=initial_group(row)
        self.assertEqual(group['fruits'],['Fruit_01','Fruit_02'])
        self.assertTrue(all(p.startswith('Plant_22/Truss_02/') for p in paths))
        self.assertEqual(group['measured_centroid_world_m'],[-.41000000000000003,1.5,1.19])


if __name__=='__main__':unittest.main()
