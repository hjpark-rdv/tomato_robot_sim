import copy
import json
import unittest

import numpy as np

from camera_patrol import CameraPatrol


class CameraPatrolTests(unittest.TestCase):
    def setUp(self):
        self.inventory=dict(trusses=[dict(plant_id=p,truss_index=t,gutter_center_x=g,
            root_position_world_m=[root,77.,.3],fruit_centroid_world_m=[center,99.,66.],rail_near=near)
            for p,t,g,root,center,near in [(22,0,-.775,-.68,-.65,True),
                (22,1,-.775,-.68,-.65,True),(23,0,-.775,-.68,-.65,True),
                (24,0,.775,.68,.65,True),(25,0,-.775,-.88,-.85,False)]])
        # Optical +Z points world -X; optical +Y points world -Z.
        self.camera=np.array([[0.,0.,-1.,0.],[1.,0.,0.,1.],[0.,-1.,0.,1.],[0.,0.,0.,1.]])

    def patrol(self,**changes):
        return CameraPatrol(self.inventory,-.775,**changes)

    def key(self,plant=22,truss=0,fruit=1):
        return f'/World/Greenhouse/Plants/Plant_{plant}/Truss_{truss:02d}/Fruit_{fruit:02d}'

    def observation(self,y=1.3,z=1.2,plant=22,truss=0,**changes):
        objects=[dict(key=self.key(plant,truss,i),class_name='fruit',instance_id=plant*100+truss*10+i,
            red_proxy=True,visible_pixels=40,valid_depth_pixels=35,
            visible_surface_median_world_m=[-.6,y+(i-1.5)*.02,z],known_bounding_radius_m=.012)
            for i in (1,2)]
        value=dict(objects=objects,world_from_camera_optical=self.camera.tolist(),
            timestamp_s=2.,current_timestamp_s=2.,observation_id='frame2')
        value.update(changes);return value

    def test_observed_position_and_calibrated_goal_ignore_hidden_inventory_centroid(self):
        patrol=self.patrol(camera_foreaft_m=.08,camera_to_grasp_height_m=.1)
        patrol.observe(self.observation(),current_station_y_m=1.,current_lift_m=.25)
        item=next(iter(patrol.queue.values()))
        np.testing.assert_allclose(item['aim_world_m'],[-.612,1.3,1.2])
        self.assertAlmostEqual(item['service_station_y_m'],1.22)
        self.assertAlmostEqual(item['suggested_lift_m'],.35)
        self.assertFalse(item['uses_inventory_centroid_for_aim'])
        changed=copy.deepcopy(self.inventory)
        for row in changed['trusses']:row['fruit_centroid_world_m'][1:]=[-123.,321.]
        other=CameraPatrol(changed,-.775,camera_foreaft_m=.08,camera_to_grasp_height_m=.1)
        other.observe(self.observation(),current_station_y_m=1.,current_lift_m=.25)
        self.assertEqual(other.queue,patrol.queue)

    def test_short_moves_stop_at_observed_target_and_then_request_existing_harvest(self):
        patrol=self.patrol(step_m=.2)
        patrol.observe(self.observation(y=1.27),current_station_y_m=1.)
        move=patrol.decide(current_station_y_m=1.)
        self.assertEqual(move['action'],'move');self.assertAlmostEqual(move['next_station_y_m'],1.2)
        move=patrol.decide(current_station_y_m=1.2)
        self.assertEqual(move['action'],'move');self.assertAlmostEqual(move['next_station_y_m'],1.27)
        decision=patrol.decide(current_station_y_m=1.27)
        self.assertEqual(decision['action'],'harvest');self.assertTrue(decision['requires_window_switch'])
        self.assertIsNone(decision['harvest_success'])

    def test_empty_view_keeps_pending_target_and_reacquires_before_harvest(self):
        patrol=self.patrol()
        patrol.observe(self.observation(y=1.3),current_station_y_m=1.)
        patrol.observe(self.observation(objects=[]),current_station_y_m=1.15)
        decision=patrol.decide(current_station_y_m=1.3)
        self.assertEqual(decision['action'],'scan')
        self.assertEqual(len(patrol.queue),1)
        self.assertEqual(next(iter(patrol.queue.values()))['status'],'pending')

    def test_never_silently_passes_remembered_target_or_calls_it_impossible(self):
        patrol=self.patrol()
        patrol.observe(self.observation(y=1.1),current_station_y_m=1.)
        decision=patrol.decide(current_station_y_m=1.3)
        self.assertEqual(decision['action'],'stop')
        target=next(iter(patrol.queue));self.assertIn(target,decision['pending_target_keys'])
        self.assertEqual(patrol.queue[target]['status'],'revisit_required')
        self.assertIn('unknown',patrol.queue[target]['reachability'])
        patrol.record_blocker(target,'finite_ik_not_found',dict(plan='one.json',attempts=3))
        decision=patrol.decide(current_station_y_m=1.3)
        self.assertEqual(decision['action'],'move')
        self.assertFalse(patrol.queue[target]['blockers'][0]['impossible'])

    def test_selected_bed_near_side_and_depth_filters_preserve_rejection_reasons(self):
        patrol=self.patrol()
        observation=self.observation()
        observation['objects']+=self.observation(plant=24)['objects']+self.observation(plant=25)['objects']
        observation['objects']+=[dict(self.observation(plant=23)['objects'][0],red_proxy=False),
            dict(self.observation(plant=23)['objects'][1],valid_depth_pixels=0)]
        result=patrol.observe(observation,current_station_y_m=1.)
        reasons={row['reason'] for row in result['rejected']}
        self.assertTrue({'opposite_bed','source_stem_opposite_gutter_side','not_observed_red',
            'insufficient_valid_depth_pixels'}<=reasons)
        self.assertEqual(len(patrol.queue),1)

    def test_native_detachment_changes_attached_queue_but_appearance_does_not(self):
        patrol=self.patrol();patrol.observe(self.observation(y=1.),current_station_y_m=1.)
        target=next(iter(patrol.queue))
        patrol.observe(self.observation(objects=[]),current_station_y_m=1.)
        self.assertEqual(len(patrol.queue[target]['observed_attached_fruit_keys']),2)
        patrol.record_native_detachments([self.key()]);patrol.decide(current_station_y_m=1.)
        self.assertEqual(patrol.queue[target]['status'],'below_group_minimum_after_native_detachment')
        patrol.record_native_detachments([self.key(fruit=2)]);patrol.decide(current_station_y_m=1.)
        self.assertEqual(patrol.queue[target]['status'],'observed_fruits_native_detached')
        self.assertIsNone(patrol.to_dict()['harvest_success'])

    def test_order_nearest_service_station_then_highest_current_group(self):
        patrol=self.patrol();observation=self.observation(y=1.,z=1.1)
        observation['objects']+=self.observation(y=1.,z=1.4,truss=1)['objects']
        observation['objects']+=self.observation(y=1.4,z=1.8,plant=23)['objects']
        patrol.observe(observation,current_station_y_m=1.)
        decision=patrol.decide(current_station_y_m=1.,active_source_plant='/World/Greenhouse/Plants/Plant_22')
        self.assertEqual(decision['target']['truss_key'],'Truss_01')
        self.assertFalse(decision['requires_window_switch'])

    def test_frozen_window_cannot_be_rebuilt_from_authored_rest(self):
        patrol=self.patrol();patrol.observe(self.observation(y=1.),current_station_y_m=1.)
        decision=patrol.decide(current_station_y_m=1.,frozen_source_plants=['/World/Greenhouse/Plants/Plant_22'])
        self.assertEqual(decision['action'],'stop')
        self.assertEqual(next(iter(patrol.queue.values()))['status'],'revisit_requires_measured_window_recovery')

    def test_ledger_round_trip_preserves_out_of_view_blockers_and_native_evidence(self):
        patrol=self.patrol();patrol.observe(self.observation(y=1.3),current_station_y_m=1.)
        target=next(iter(patrol.queue));patrol.record_blocker(target,'environment_audit_blocked',dict(audit='audit.json'))
        patrol.record_native_detachments([self.key()]);patrol.decide(current_station_y_m=1.)
        value=json.loads(json.dumps(patrol.to_dict()))
        restored=CameraPatrol.from_dict(self.inventory,value)
        self.assertEqual(restored.to_dict(),value)
        other=copy.deepcopy(self.inventory);other['trusses'][0]['root_position_world_m'][1]=0.
        with self.assertRaisesRegex(ValueError,'identity mismatch'):CameraPatrol.from_dict(other,value)

    def test_stale_camera_does_not_replace_observed_target_or_create_new_targets(self):
        patrol=self.patrol();patrol.observe(self.observation(y=1.3),current_station_y_m=1.)
        before=copy.deepcopy(patrol.queue)
        row=patrol.observe(self.observation(y=1.5,plant=23,current_timestamp_s=4.),current_station_y_m=1.)
        self.assertTrue(row['stale']);self.assertEqual(patrol.queue,before)
        self.assertEqual(patrol.decide(current_station_y_m=1.)['action'],'scan')

    def test_nonintersection_unripe_and_singleton_are_not_eligible_groups(self):
        patrol=self.patrol();observation=self.observation()
        observation['objects'][1]['red_proxy']=False
        row=patrol.observe(observation,current_station_y_m=1.)
        self.assertFalse(patrol.queue)
        self.assertIn('below_visible_group_minimum',{entry['reason'] for entry in row['rejected']})

    def test_duplicate_and_nonfinite_observations_and_oversized_steps_rejected(self):
        for changes in (dict(step_m=.21),dict(max_step_m=.3),dict(camera_foreaft_m=float('nan')),
                dict(forward_sign=0),dict(minimum_fruits=1)):
            with self.subTest(changes=changes),self.assertRaises(ValueError):self.patrol(**changes)
        observation=self.observation();observation['objects'].append(observation['objects'][0])
        with self.assertRaisesRegex(ValueError,'Duplicate'):self.patrol().observe(observation,current_station_y_m=1.)
        observation=self.observation();observation['world_from_camera_optical'][0][0]=float('inf')
        with self.assertRaises(ValueError):self.patrol().observe(observation,current_station_y_m=1.)

    def test_empty_forward_patrol_and_rail_end_have_finite_step_bounds(self):
        patrol=self.patrol()
        for station in np.arange(0.,5.,.15):
            decision=patrol.decide(current_station_y_m=float(station))
            self.assertEqual(decision['action'],'move')
            self.assertGreater(decision['movement_m'],0.)
            self.assertLessEqual(decision['movement_m'],.2)
            self.assertGreaterEqual(decision['next_station_y_m'],station)
            self.assertLessEqual(decision['next_station_y_m'],5.)
        self.assertEqual(patrol.decide(current_station_y_m=5.)['reason'],'physical_rail_end_reached')

    def test_float32_endpoint_measurements_are_preserved_without_out_of_bounds_commands(self):
        upper=float(np.nextafter(np.float32(5.),np.float32(np.inf)))
        lower=-float(np.float32(1e-6))
        patrol=self.patrol()
        end=patrol.decide(current_station_y_m=upper)
        self.assertEqual(end['action'],'stop');self.assertEqual(end['reason'],'physical_rail_end_reached')
        self.assertEqual(end['current_station_y_m'],upper)
        self.assertEqual(end['rail_measurement']['actual_station_y_m'],upper)
        self.assertTrue(end['rail_measurement']['outside_nominal_bounds'])
        self.assertFalse(end['rail_measurement']['actual_measurement_clamped'])
        move=patrol.decide(current_station_y_m=lower)
        self.assertEqual(move['action'],'move');self.assertEqual(move['current_station_y_m'],lower)
        self.assertGreaterEqual(move['next_station_y_m'],0.)
        self.assertLessEqual(move['next_station_y_m']-lower,.20)
        self.assertFalse(move['rail_measurement']['physical_limits_changed'])
        reverse=self.patrol(forward_sign=-1)
        end=reverse.decide(current_station_y_m=lower)
        self.assertEqual(end['action'],'stop');self.assertEqual(end['current_station_y_m'],lower)
        move=reverse.decide(current_station_y_m=upper)
        self.assertEqual(move['action'],'move');self.assertLessEqual(move['next_station_y_m'],5.)
        self.assertLessEqual(upper-move['next_station_y_m'],.20)
        for measured in (-.0021,5.0021):
            with self.assertRaisesRegex(ValueError,'measurement envelope'):patrol.decide(current_station_y_m=measured)

    def test_finite_planning_failure_retries_three_distinct_forward_views_then_defers(self):
        patrol=self.patrol()
        patrol.observe(self.observation(y=1.),current_station_y_m=1.)
        target=next(iter(patrol.queue))
        first=patrol.record_local_failure(target,'finite_ik_not_found',dict(plan='one.json'),current_station_y_m=1.)
        self.assertEqual(first['status'],'forward_view_retry_pending')
        self.assertEqual(patrol.decide(current_station_y_m=1.)['action'],'move')
        self.assertAlmostEqual(patrol.decide(current_station_y_m=1.)['next_station_y_m'],1.05)
        moved=copy.deepcopy(self.camera);moved[1,3]+= .05
        patrol.observe(self.observation(y=1.,world_from_camera_optical=moved.tolist()),current_station_y_m=1.05)
        self.assertAlmostEqual(patrol.queue[target]['service_station_y_m'],1.05)
        self.assertEqual(patrol.decide(current_station_y_m=1.05)['action'],'harvest')
        second=patrol.record_local_failure(target,'finite_path_blocked',dict(audit='two.json'),current_station_y_m=1.05)
        self.assertEqual(second['status'],'forward_view_retry_pending')
        third=patrol.record_local_failure(target,'finite_path_blocked',dict(audit='three.json'),current_station_y_m=1.1)
        self.assertEqual(third['status'],'finite_deferred')
        self.assertEqual(len(third['evaluated_distinct_stations_y_m']),3)
        self.assertEqual(patrol.queue[target]['status'],'finite_blocked')
        self.assertEqual(len(patrol.queue[target]['planning_failures']),3)

    def test_unvalidated_physical_failure_cannot_be_passed_as_distant_local_planning_failure(self):
        patrol=self.patrol();patrol.observe(self.observation(y=1.),current_station_y_m=1.)
        with self.assertRaisesRegex(ValueError,'service station'):
            patrol.record_local_failure(next(iter(patrol.queue)),'not_a_validated_plan_failure',dict(summary='stopped'),
                current_station_y_m=1.3)

    def test_post_native_scan_failure_records_actual_observation_without_harvest_motion(self):
        patrol=self.patrol();patrol.observe(self.observation(y=1.),current_station_y_m=1.)
        target=next(iter(patrol.queue))
        patrol.record_local_failure(target,'finite_all_camera_plans_rejected',dict(actual_trial='fixture'),
            current_station_y_m=1.,native_observation_executed=True)
        evidence=patrol.queue[target]['planning_failures'][0]
        self.assertTrue(evidence['native_physics_executed']);self.assertTrue(evidence['native_observation_executed'])
        self.assertFalse(evidence['harvest_motion_executed'])
        self.assertAlmostEqual(patrol.queue[target]['service_probe_station_y_m'],1.05)

    def test_interleaved_plants_finish_current_visible_active_group_before_switching(self):
        patrol=self.patrol()
        observation=self.observation(y=1.13,z=1.1)
        observation['objects']+=self.observation(y=1.,z=1.8,plant=23)['objects']
        patrol.observe(observation,current_station_y_m=1.)
        decision=patrol.decide(current_station_y_m=1.,active_source_plant='/World/Greenhouse/Plants/Plant_22')
        self.assertEqual(decision['action'],'harvest');self.assertEqual(decision['target']['plant_id'],22)
        self.assertFalse(decision['requires_window_switch']);self.assertEqual(decision['next_station_y_m'],1.)
        proposal=decision['service_offset_proposal'];self.assertAlmostEqual(proposal['along_row_offset_m'],.13)
        self.assertEqual(proposal['maximum_proposal_offset_m'],.2)
        self.assertEqual(proposal['ordinary_arrival_tolerance_m'],.06)
        self.assertFalse(proposal['arrival_tolerance_changed']);self.assertTrue(proposal['requires_existing_ik_full_audit'])
        self.assertEqual(len(patrol.queue),2)
        self.assertEqual(len(decision['active_window_priority']['other_pending_targets_retained']),1)
        patrol.observe(observation,current_station_y_m=1.,native_detached_keys=[self.key(fruit=1),self.key(fruit=2)])
        decision=patrol.decide(current_station_y_m=1.,active_source_plant='/World/Greenhouse/Plants/Plant_22')
        self.assertEqual(decision['target']['plant_id'],23);self.assertTrue(decision['requires_window_switch'])

    def test_active_pending_ahead_outranks_other_plant_current_station_without_dropping_it(self):
        patrol=self.patrol()
        observation=self.observation(y=1.35)
        observation['objects']+=self.observation(y=1.,plant=23)['objects']
        patrol.observe(observation,current_station_y_m=1.)
        decision=patrol.decide(current_station_y_m=1.,active_source_plant='/World/Greenhouse/Plants/Plant_22')
        self.assertEqual(decision['action'],'move');self.assertEqual(decision['target']['plant_id'],22)
        self.assertAlmostEqual(decision['next_station_y_m'],1.15)
        other=patrol.queue['/World/Greenhouse/Plants/Plant_23/Truss_00']
        self.assertEqual(other['status'],'pending');self.assertFalse(other['blockers'])

    def test_fresh_visible_slightly_behind_group_gets_bounded_arm_proposal_before_revisit_stop(self):
        patrol=self.patrol();observation=self.observation(y=.88)
        patrol.observe(observation,current_station_y_m=1.)
        decision=patrol.decide(current_station_y_m=1.,active_source_plant='/World/Greenhouse/Plants/Plant_22')
        self.assertEqual(decision['action'],'harvest')
        self.assertAlmostEqual(decision['service_offset_proposal']['along_row_offset_m'],-.12)
        retry=patrol.record_local_failure(decision['target_key'],'finite_ik_not_found',dict(plan='offset.json'),
            current_station_y_m=1.)
        self.assertEqual(retry['status'],'forward_view_retry_pending')
        self.assertAlmostEqual(retry['next_station_y_m'],1.05)
        self.assertEqual(patrol.decide(current_station_y_m=1.,active_source_plant='/World/Greenhouse/Plants/Plant_22')['action'],'move')

    def test_arm_proposal_budget_and_observation_station_are_not_loosened(self):
        for value in (.201,-.01,float('nan')):
            with self.assertRaises(ValueError):self.patrol(visible_service_offset_proposal_m=value)
        patrol=self.patrol();patrol.observe(self.observation(y=1.201),current_station_y_m=1.)
        self.assertEqual(patrol.decide(current_station_y_m=1.)['action'],'move')
        disabled=self.patrol(visible_service_offset_proposal_m=0.)
        disabled.observe(self.observation(y=1.13),current_station_y_m=1.)
        self.assertEqual(disabled.decide(current_station_y_m=1.)['action'],'move')
        stale_pose=self.patrol();stale_pose.observe(self.observation(y=1.23),current_station_y_m=1.)
        self.assertEqual(stale_pose.decide(current_station_y_m=1.1)['action'],'move')

    def test_fixed_absolute_forward_probe_does_not_chase_fresh_depth_median_jitter(self):
        patrol=self.patrol();patrol.observe(self.observation(y=1.),current_station_y_m=1.)
        target=next(iter(patrol.queue))
        patrol.record_local_failure(target,'finite_ik_not_found',dict(plan='one.json'),current_station_y_m=1.)
        for measured,depth_y,expected in [(1.0508,1.009,1.05),(1.0999,.993,1.1008)]:
            camera=self.camera.copy();camera[1,3]=measured
            patrol.observe(self.observation(y=depth_y,world_from_camera_optical=camera.tolist()),current_station_y_m=measured)
            self.assertAlmostEqual(patrol.queue[target]['service_station_y_m'],expected)
            self.assertAlmostEqual(patrol.queue[target]['observed_service_station_y_m'],depth_y)
            self.assertTrue(patrol.queue[target]['service_probe_arrived'])
            decision=patrol.decide(current_station_y_m=measured,active_source_plant='/World/Greenhouse/Plants/Plant_22')
            self.assertEqual(decision['action'],'harvest')
            result=patrol.record_local_failure(target,'finite_ik_not_found',dict(plan='next.json'),current_station_y_m=measured)
        self.assertEqual(result['status'],'finite_deferred')
        self.assertEqual(len(result['evaluated_distinct_stations_y_m']),3)

    def test_shared_invalid_source_quarantines_all_trusses_without_reachability_or_harvest_claim(self):
        patrol=self.patrol();observation=self.observation(y=1.)
        observation['objects']+=self.observation(y=1.1,truss=1)['objects']
        observation['objects']+=self.observation(y=1.,plant=23)['objects']
        patrol.observe(observation,current_station_y_m=1.)
        source='/World/Greenhouse/Plants/Plant_22'
        evidence=dict(startup_report='unused_22/startup.json',status='failed',
            initial_model_sha256='fixture-sha',actual_harvest_executed=False)
        saved=patrol.quarantine_plant(source,evidence)
        evidence['status']='caller-mutated'
        self.assertEqual(saved['evidence']['status'],'failed')
        self.assertTrue(saved['operational_only']);self.assertFalse(saved['impossible'])
        for truss in ('Truss_00','Truss_01'):
            item=patrol.queue[source+'/'+truss]
            self.assertEqual(item['status'],'not_attempted_shared_invalid_initial_model')
            self.assertEqual(len(item['observed_attached_fruit_keys']),2)
            self.assertTrue(item['visible_now']);self.assertFalse(item['blockers'])
            self.assertFalse(item['planning_failures']);self.assertIn('unknown',item['reachability'])
        decision=patrol.decide(current_station_y_m=1.,active_source_plant=source)
        self.assertEqual(decision['action'],'harvest');self.assertEqual(decision['target']['plant_id'],23)
        self.assertEqual(len(decision['quarantined_target_keys']),2)
        self.assertEqual(decision['operational_quarantined_source_plants'],[source])
        self.assertFalse(decision['finite_blocked_target_keys'])
        self.assertFalse(patrol.native_detached_keys);self.assertIsNone(patrol.to_dict()['harvest_success'])

    def test_quarantine_applies_to_future_visible_groups_and_survives_out_of_view(self):
        patrol=self.patrol();source='/World/Greenhouse/Plants/Plant_22'
        patrol.quarantine_plant(source,dict(startup_report='startup22.json',status='invalid_initial_model'))
        row=patrol.observe(self.observation(y=1.),current_station_y_m=1.)
        self.assertEqual(len(row['accepted']),2);self.assertEqual(len(row['queued']),1)
        self.assertEqual(row['quarantined'][0]['reason'],'not_attempted_shared_invalid_initial_model')
        patrol.observe(self.observation(y=1.1,truss=1),current_station_y_m=1.)
        patrol.observe(self.observation(objects=[]),current_station_y_m=1.)
        decision=patrol.decide(current_station_y_m=1.,active_source_plant=source,frozen_source_plants=[source])
        self.assertEqual(decision['action'],'move')
        self.assertEqual(len(patrol.queue),2);self.assertEqual(len(decision['quarantined_target_keys']),2)
        self.assertTrue(all(not row['visible_now'] for row in patrol.queue.values()))
        self.assertTrue(all(row['status']=='not_attempted_shared_invalid_initial_model' for row in patrol.queue.values()))

    def test_quarantine_round_trip_and_old_ledger_compatibility_keep_native_evidence(self):
        patrol=self.patrol();patrol.observe(self.observation(y=1.),current_station_y_m=1.)
        old=patrol.to_dict();old.pop('plant_quarantines')
        restored=CameraPatrol.from_dict(self.inventory,old)
        self.assertEqual(restored.plant_quarantines,{})
        source='/World/Greenhouse/Plants/Plant_22'
        restored.quarantine_plant(source,dict(startup_report='first.json'))
        restored.quarantine_plant(source,dict(startup_report='confirmation.json'))
        value=json.loads(json.dumps(restored.to_dict()))
        reloaded=CameraPatrol.from_dict(self.inventory,value)
        self.assertEqual(reloaded.to_dict(),value)
        self.assertEqual(len(reloaded.plant_quarantines[source]['evidence_history']),2)
        reloaded.record_native_detachments([self.key(),self.key(fruit=2)])
        reloaded.decide(current_station_y_m=1.)
        self.assertEqual(reloaded.queue[source+'/Truss_00']['status'],'observed_fruits_native_detached')
        self.assertEqual(len(reloaded.native_detached_keys),2)
        self.assertEqual(reloaded.queue[source+'/Truss_00']['operational_exclusion_reason'],
            'not_attempted_shared_invalid_initial_model')

    def test_quarantine_requires_exact_source_and_concrete_finite_evidence(self):
        patrol=self.patrol();source='/World/Greenhouse/Plants/Plant_22'
        for invalid in ('Plant_22',source+'/Truss_00',source+'/',source.replace('22','99'),None):
            with self.subTest(source=invalid),self.assertRaisesRegex(ValueError,'Exact inventory'):
                patrol.quarantine_plant(invalid,dict(report='startup.json'))
        for evidence in ({},None,[],dict(distance=float('nan'))):
            with self.subTest(evidence=evidence),self.assertRaises((ValueError,TypeError)):
                patrol.quarantine_plant(source,evidence)
        self.assertFalse(patrol.plant_quarantines)
        patrol.quarantine_plant(source,dict(report='startup.json'))
        malformed=patrol.to_dict();malformed['plant_quarantines'][source]['impossible']=True
        with self.assertRaisesRegex(ValueError,'quarantine evidence'):
            CameraPatrol.from_dict(self.inventory,malformed)


if __name__=='__main__':unittest.main()
