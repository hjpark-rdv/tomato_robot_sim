import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from camera_region_policy import CameraRegionSelector


class CameraRegionPolicyTests(unittest.TestCase):
    def test_robot_forward_cone_is_absolute_not_an_offset_from_sideways_rachis(self):
        self.observation['objects'].append(dict(instance_id=5000,class_name='rachis',
            truss='Truss_02',key=self.source+'/Truss_02/Rachis',visible_pixels=100,valid_depth_pixels=100,
            visible_surface_xy_axis_world=[1.,0.,0.],visible_surface_xy_anisotropy=20.))
        for yaw in (-15.,0.,15.):
            selector=self.selector(entry_forward_world=[0.,1.,0.],yaw_offset_deg=yaw)
            self.select(selector)
            row=selector.last_decision['camera_region']['direction_estimates']['Truss_02']
            np.testing.assert_allclose(np.abs(row['observed_truss_direction_world']),[1.,0.,0.])
            direction=np.asarray(row['direction_world'])
            actual=np.rad2deg(np.arctan2(-direction[0],direction[1]))
            self.assertAlmostEqual(actual,yaw)
            self.assertEqual(row['maximum_entry_yaw_deg'],15.)
        with self.assertRaisesRegex(ValueError,'robot forward limit'):
            self.selector(entry_forward_world=[0.,1.,0.],yaw_offset_deg=15.001)
        with self.assertRaisesRegex(ValueError,'robot forward limit'):
            selector.set_yaw_offset_deg(-15.001)
        for forward in ([0.,0.,1.],[0.,0.,0.],[float('nan'),0.,0.]):
            with self.assertRaisesRegex(ValueError,'robot entry reference'):
                self.selector(entry_forward_world=forward)

    def setUp(self):
        self.source='/World/Greenhouse/Plants/Plant_22'
        self.fruits=[dict(name=f'Truss_02_Fruit_{i+1:02d}',path=f'/target/f{i}',joint=f'/joint/f{i}',
                         dimensions_m=[.023]*3) for i in range(4)]
        self.positions=np.array([[0.,0.,1.1],[.005,.02,1.085],[-.005,.04,1.07],[.5,0,1.]],float)
        self.paths=[fruit['path'] for fruit in self.fruits]
        self.policy=dict(minimum_fruits=3,fruit_eligibility=[dict(**fruit,eligible=True) for fruit in self.fruits])
        self.camera=np.array([[1.,0,0,0],[0,0,1,-.5],[0,-1,0,1.],[0,0,0,1.]])
        self.observation=dict(observation_id='frame_00001',timestamp_s=.5,current_timestamp_s=.5,
                              world_from_camera_optical=self.camera.tolist(),objects=[])
        for i in range(3):self.observation['objects'].append(self.row(i))
        self.direction_calls=[]

    def row(self,index,**changes):
        row=dict(instance_id=3000+index,key=f'{self.source}/Truss_02/Fruit_{index+1:02d}',
                 class_name='fruit',red_proxy=True,visible_pixels=35,valid_depth_pixels=33,
                 visible_surface_median_world_m=(self.positions[index]-[0.,.0115,0.]).tolist())
        row.update(changes);return row

    def selector(self,**changes):
        def direction(key,fruits,breaks,paths,poses):
            self.direction_calls.append([fruit['name'] for fruit in fruits]);return [0.,1.,0.]
        return CameraRegionSelector(self.policy,direction,lambda paths,poses:copy.deepcopy(self.observation),
            plant_source=self.source,height_offset_m=.0075,**changes)

    def select(self,selector,**changes):
        return selector(self.fruits,changes.pop('breaks',[]),self.paths,self.positions,
                        changes.pop('attempts',{}),**changes)

    def add_lower_truss(self,separated=False):
        points=np.array([[0.,0.,.7],[.005,.02,.685],[-.005,.04,.67]])
        if separated:points[:,0]=[0.,.3,.6]
        for i,point in enumerate(points):
            name=f'Truss_01_Fruit_{i+1:02d}'
            fruit=dict(name=name,path=f'/target/lower{i}',joint=f'/joint/lower{i}',dimensions_m=[.023]*3)
            self.fruits.append(fruit);self.paths.append(fruit['path'])
            self.policy['fruit_eligibility'].append(dict(**fruit,eligible=True))
            self.observation['objects'].append(dict(instance_id=4000+i,
                key=f'{self.source}/Truss_01/Fruit_{i+1:02d}',class_name='fruit',red_proxy=True,
                visible_pixels=35,valid_depth_pixels=33,
                visible_surface_median_world_m=(point-[0.,.0115,0.]).tolist()))
        self.positions=np.concatenate([self.positions,points])

    def test_zero_yaw_keeps_exact_default_output_and_returning_to_zero_does_not_compound(self):
        ordinary=self.select(self.selector())
        explicit=self.select(self.selector(yaw_offset_deg=0.))
        self.assertEqual(explicit,ordinary)
        selector=self.selector(yaw_offset_deg=15.)
        self.select(selector);selector.set_yaw_offset_deg(0.)
        self.assertEqual(self.select(selector),ordinary)
        self.assertNotIn('entry_yaw_offset_deg',ordinary['camera_region'])

    def test_yaw_is_world_z_only_once_for_every_observed_truss_before_footprint(self):
        self.add_lower_truss();before=self.positions.copy()
        ordinary=self.selector();self.select(ordinary)
        selector=self.selector(yaw_offset_deg=15.);chosen=self.select(selector)
        angle=np.deg2rad(15.);rotation=np.array([[np.cos(angle),-np.sin(angle),0.],
            [np.sin(angle),np.cos(angle),0.],[0.,0.,1.]])
        for key,original in ordinary.last_decision['camera_region']['direction_estimates'].items():
            row=selector.last_decision['camera_region']['direction_estimates'][key]
            np.testing.assert_array_equal(row['unrotated_direction_world'],original['direction_world'])
            np.testing.assert_allclose(row['direction_world'],rotation@original['direction_world'],atol=1e-15)
            self.assertEqual(row['direction_world'][2],0.)
            self.assertTrue(row['entry_direction_already_yaw_rotated'])
        self.assertTrue(chosen['camera_region']['yaw_proposal_footprint_recomputed'])
        self.assertTrue(chosen['group_selection']['lateral_centering']['footprint_rechecked'])
        np.testing.assert_array_equal(chosen['group_selection']['direction_world'],
            selector.last_decision['camera_region']['direction_estimates'][chosen['truss_key']]['direction_world'])
        self.assertEqual(self.select(selector),chosen)
        np.testing.assert_array_equal(self.positions,before)

    def test_yaw_recheck_preserves_hidden_ineligible_fruit_as_footprint_obstacle(self):
        selector=self.selector(yaw_offset_deg=15.)
        self.assertIsNotNone(self.select(selector))
        self.positions[3]=self.positions[1]
        self.policy['fruit_eligibility'][3]['eligible']=False
        selector=self.selector(yaw_offset_deg=15.)
        self.assertIsNone(self.select(selector))
        self.assertTrue(selector.last_decision['camera_region']['hidden_and_ineligible_fruit_preserved_as_footprint_obstacles'])
        self.assertNotIn(self.fruits[3]['name'],selector.last_decision['camera_region']['observable_eligible_fruits'])

    def test_yaw_configuration_and_setter_reject_nonfinite_boolean_and_out_of_bounds(self):
        for value in (True,np.bool_(False),float('nan'),float('inf'),-90.001,90.001,'15',None):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'yaw offset'):
                self.selector(yaw_offset_deg=value)
        selector=self.selector(yaw_offset_deg=-30.)
        with self.assertRaisesRegex(ValueError,'yaw offset'):selector.yaw_offset_deg=float('nan')
        self.assertEqual(selector.yaw_offset_deg,-30.)
        selector.set_yaw_offset_deg(np.float64(15.));self.assertEqual(selector.yaw_offset_deg,15.)

    def test_degenerate_observed_xy_without_rachis_uses_camera_ray_then_rotates_once(self):
        self.positions[:3,:2]=[.02,.01]
        self.observation['objects']=[self.row(i) for i in range(3)]
        selector=CameraRegionSelector(self.policy,None,lambda paths,poses:copy.deepcopy(self.observation),
            plant_source=self.source,height_offset_m=.0075,yaw_offset_deg=15.)
        selected=self.select(selector)
        inward=self.positions[:3].mean(0)-self.camera[:3,3];inward[2]=0.
        natural=inward/np.linalg.norm(inward);angle=np.deg2rad(15.)
        expected=np.array([np.cos(angle)*natural[0]-np.sin(angle)*natural[1],
            np.sin(angle)*natural[0]+np.cos(angle)*natural[1],0.])
        evidence=selected['camera_region']['direction_estimates']['Truss_02']
        self.assertEqual(evidence['method'],'camera_to_visible_centroid_horizontal_fallback')
        np.testing.assert_allclose(evidence['unrotated_direction_world'],natural,atol=1e-15)
        np.testing.assert_allclose(selected['group_selection']['direction_world'],expected,atol=1e-15)
        self.assertEqual(self.select(selector),selected)

    def test_visible_rachis_heading_preferred_and_exact_truss_scoped(self):
        stem=dict(instance_id=99,key=f'{self.source}/Truss_02/Rachis',truss='Truss_02',
            class_name='rachis',visible_pixels=50,valid_depth_pixels=50,
            visible_surface_xy_axis_world=[0.,1.,0.],visible_surface_xy_anisotropy=100.)
        self.observation['objects'].append(stem)
        chosen=self.select(self.selector())
        evidence=chosen['camera_region']['direction_estimates']['Truss_02']
        self.assertEqual(evidence['method'],'horizontal_PCA_of_visible_Rachis_depth_pixels')
        self.assertEqual(evidence['rachis_instance_id'],99)
        self.assertFalse(evidence['uses_hidden_geometry'])
        np.testing.assert_allclose(evidence['direction_world'],[0.,1.,0.])
        for change in [dict(key='/World/Greenhouse/Plants/Plant_23/Truss_02/Rachis'),
                       dict(truss='Truss_01'),dict(visible_pixels=0),
                       dict(valid_depth_pixels=0),dict(visible_surface_xy_anisotropy=1.)]:
            self.observation['objects'][-1]=dict(stem,**change)
            fallback=self.select(self.selector())['camera_region']['direction_estimates']['Truss_02']
            self.assertEqual(fallback['method'],'horizontal_PCA_of_visible_approximate_centers')

    def test_rachis_unsigned_axis_sign_uses_current_camera_ray_over_wrong_fruit_hint(self):
        self.observation['objects'].append(dict(instance_id=99,key=f'{self.source}/Truss_02/Rachis',
            truss='Truss_02',class_name='rachis',visible_pixels=50,valid_depth_pixels=50,
            visible_surface_xy_axis_world=[0.,-1.,0.],visible_surface_xy_anisotropy=100.))
        selector=self.selector();selector.direction_hint=[0.,-1.,0.]
        chosen=self.select(selector)
        d=chosen['camera_region']['direction_estimates']['Truss_02']
        np.testing.assert_allclose(d['direction_world'],[0.,1.,0.])
        self.assertGreater(d['inward_alignment_cosine'],0.)

    def test_near_vertical_rachis_uses_observed_radial_direction_and_rechecks_footprint(self):
        self.observation['objects'].append(dict(instance_id=99,key=f'{self.source}/Truss_02/Rachis',
            truss='Truss_02',class_name='rachis',visible_pixels=50,valid_depth_pixels=50,
            visible_surface_xy_axis_world=[1.,0.,0.],visible_surface_xy_anisotropy=100.,
            visible_surface_long_axis_horizontal_fraction=.2))
        chosen=self.select(self.selector())
        d=chosen['camera_region']['direction_estimates']['Truss_02']
        self.assertEqual(d['method'],'camera_to_visible_centroid_for_near_vertical_Rachis')
        inward=np.array(d['camera_to_observed_centroid_horizontal_m'])
        np.testing.assert_allclose(chosen['group_selection']['direction_world'],inward/np.linalg.norm(inward))
        self.assertFalse(d['near_vertical_threshold_is_physics_validity_gate'])

    def test_unattempted_truss_priority_is_opt_in_and_uses_physical_attempt_ledger(self):
        self.add_lower_truss()
        # The upper truss was physically attempted at another fruit. Its
        # three visible unused fruit otherwise retain the normal top priority.
        attempts={self.fruits[3]['name']:1};before=copy.deepcopy(attempts)
        ordinary=self.selector();default=self.select(ordinary,attempts=attempts)
        self.assertEqual(default['truss_key'],'Truss_02')
        exploration=self.selector(prefer_unattempted_trusses=True)
        selected=self.select(exploration,attempts=attempts)
        self.assertEqual(selected['truss_key'],'Truss_01')
        evidence=selected['camera_region']['truss_exploration']
        self.assertEqual(evidence['attempted_trusses'],['Truss_02'])
        self.assertEqual(evidence['observable_unattempted_trusses'],['Truss_01'])
        self.assertFalse(evidence['selected_truss_attempted_before'])
        self.assertEqual(attempts,before)

    def test_priority_fallback_preserves_one_decision_and_original_attempt_limits(self):
        self.add_lower_truss(separated=True)
        attempts={self.fruits[3]['name']:1}
        selector=self.selector(prefer_unattempted_trusses=True)
        selected=self.select(selector,attempts=attempts)
        self.assertEqual(selected['truss_key'],'Truss_02')
        self.assertTrue(selected['camera_region']['truss_exploration']['fallback_to_ordinary_ranking'])
        self.assertEqual(len(selector.history),1);self.assertEqual(len(selector.base.history),1)
        exhausted={fruit['name']:2 for fruit in self.fruits}
        self.assertIsNone(self.select(selector,attempts=exhausted,max_attempts=2))
        self.assertEqual(selector.last_decision['reason'],'attempt_budget_exhausted')

    def test_planning_rejections_do_not_mark_trusses_physically_attempted(self):
        self.add_lower_truss()
        self.policy['initial_rejected_groups']=[[fruit['name'] for fruit in self.fruits[:3]]]
        selector=self.selector(prefer_unattempted_trusses=True)
        selected=self.select(selector)
        self.assertEqual(selected['camera_region']['truss_exploration']['attempted_trusses'],[])
        self.assertEqual(selected['truss_key'],'Truss_01')
        self.assertEqual(len(selector.base.history),1)

    def test_visible_red_group_uses_stable_identity_and_approximate_depth_centers(self):
        selector=self.selector();result=self.select(selector)
        self.assertIsNotNone(result)
        self.assertEqual(set(result['fruits']),{fruit['name'] for fruit in self.fruits[:3]})
        evidence=result['camera_region'];self.assertFalse(evidence['position_estimate_exact'])
        self.assertFalse(evidence['uses_hidden_gt_centers_for_aim'])
        self.assertEqual([row['instance_id'] for row in evidence['accepted_observations']],[3000,3001,3002])
        for index,row in enumerate(evidence['accepted_observations']):
            np.testing.assert_allclose(row['estimated_center_world_m'],self.positions[index],rtol=0,atol=1e-12)
        self.assertTrue(all(set(call)<=set(result['fruits']) for call in self.direction_calls))

    def test_hidden_red_fruit_is_not_selected_and_input_state_is_never_mutated(self):
        before=self.positions.copy();fruits=copy.deepcopy(self.fruits);policy=copy.deepcopy(self.policy)
        selector=self.selector();result=self.select(selector)
        self.assertNotIn(self.fruits[3]['name'],result['fruits'])
        np.testing.assert_array_equal(self.positions,before)
        self.assertEqual(self.fruits,fruits);self.assertEqual(self.policy,policy)
        self.assertTrue(result['camera_region']['hidden_and_ineligible_fruit_preserved_as_footprint_obstacles'])
        # Omniscient hidden coordinates must not become an available group.
        self.observation['objects']=self.observation['objects'][:1]
        self.assertIsNone(self.select(selector))
        self.assertEqual(selector.last_decision['camera_region']['observable_eligible_fruits'],[])

    def test_outside_plant_truss_and_green_observations_never_count(self):
        self.observation['objects'] += [self.row(3,key='/World/Greenhouse/Plants/Plant_02/Truss_02/Fruit_04'),
                                      self.row(3,key=self.source+'/Truss_99/Fruit_04')]
        self.observation['objects'][2]['red_proxy']=False
        selector=self.selector();self.assertIsNone(self.select(selector))
        rejected=selector.last_decision['camera_region']['rejected_observations']
        self.assertEqual({row['reason'] for row in rejected},
                         {'other_source_plant','unknown_target_fruit_key','not_observed_red'})

    def test_selected_truss_filters_exact_identity(self):
        self.assertIsNone(self.select(self.selector(selected_truss='Truss_01')))
        selector=self.selector(selected_truss='Truss_01');self.select(selector)
        self.assertEqual({row['reason'] for row in selector.last_decision['camera_region']['rejected_observations']},
                         {'outside_selected_truss'})

    def test_stale_invalid_and_sparse_depth_never_yield_group(self):
        for change,reason in [(dict(stale=True),'stale_or_unsynchronized_observation'),
                              (dict(current_timestamp_s=2.),'stale_or_unsynchronized_observation')]:
            with self.subTest(change=change):
                saved=copy.deepcopy(self.observation);self.observation.update(change)
                selector=self.selector();self.assertIsNone(self.select(selector))
                self.assertTrue(all(row['reason']==reason for row in selector.last_decision['camera_region']['rejected_observations']))
                self.observation=saved
        for field,value,reason in [('visible_pixels',4,'insufficient_visible_pixels'),
                                   ('valid_depth_pixels',0,'insufficient_valid_depth_pixels'),
                                   ('visible_surface_median_world_m',[np.nan,0,1],'invalid_surface_depth')]:
            with self.subTest(field=field):
                saved=copy.deepcopy(self.observation);self.observation['objects'][1][field]=value
                selector=self.selector();self.assertIsNone(self.select(selector))
                self.assertIn(reason,[row['reason'] for row in selector.last_decision['camera_region']['rejected_observations']])
                self.observation=saved

    def test_detached_and_original_ineligible_fruit_do_not_reappear(self):
        selector=self.selector();self.assertIsNone(self.select(selector,breaks=[dict(joint=self.fruits[1]['joint'])]))
        self.assertIn('already_detached',[row['reason'] for row in selector.last_decision['camera_region']['rejected_observations']])
        self.policy['fruit_eligibility'][1]['eligible']=False
        selector=self.selector();self.assertIsNone(self.select(selector))
        self.assertIn('original_policy_ineligible',[row['reason'] for row in selector.last_decision['camera_region']['rejected_observations']])

    def test_observed_center_change_updates_aim_without_gt_position_change(self):
        selector=self.selector();first=self.select(selector)
        for row in self.observation['objects']:row['visible_surface_median_world_m'][0]+=.02
        second=self.select(selector)
        self.assertAlmostEqual(second['aim_world_m'][0]-first['aim_world_m'][0],.02)

    def test_duplicate_visible_identity_is_rejected(self):
        self.observation['objects'].append(copy.deepcopy(self.observation['objects'][0]))
        with self.assertRaisesRegex(ValueError,'Duplicate visible'):self.select(self.selector())

    def test_json_observation_and_decision_evidence_roundtrip(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);objects=folder/'objects.json';objects.write_text(json.dumps(dict(objects=self.observation['objects'])))
            self.observation['objects']=str(objects)
            selector=self.selector(output=folder/'decisions.json');result=self.select(selector)
            self.assertIsNotNone(result)
            saved=json.loads((folder/'decisions.json').read_text())
            self.assertEqual(saved[0]['camera_region']['observation']['observation_id'],'frame_00001')
            self.assertEqual(saved[0]['camera_region']['observable_eligible_fruits'],[fruit['name'] for fruit in self.fruits[:3]])


if __name__=='__main__':unittest.main()
