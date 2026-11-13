import copy
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

from side_camera_coverage import SideCoveragePatrol,plan_visible_alternatives
from side_camera_motion import PlanningError


class SideCoverageTests(unittest.TestCase):
    def observation(self,station=1.,count=2):
        optical=np.array([[0.,0.,-1.,0.],[1.,0.,0.,station],[0.,-1.,0.,1.],[0.,0.,0.,1.]])
        return dict(objects=[dict(key=f'/World/Greenhouse/Plants/Plant_17/Truss_00/Fruit_{i:02d}',
            class_name='fruit',instance_id=i,red_proxy=True,visible_pixels=40,valid_depth_pixels=40,
            visible_surface_median_world_m=[-.6,1.4,.5])for i in range(1,count+1)],
            observation_id=f'actual-view-{station}',world_from_camera_optical=optical.tolist(),
            timestamp_s=station,current_timestamp_s=station,physics_state_authoritative=True)

    def patrol(self):
        inventory=dict(trusses=[dict(plant_id=17,truss_index=0,gutter_center_x=-.775,
            root_position_world_m=[-.68,0.,.3],fruit_centroid_world_m=[-.6,1.,.5],rail_near=True)])
        return SideCoveragePatrol(inventory,-.775,camera_foreaft_m=.4)

    def test_visible_singleton_remains_pending_without_harvest_credit(self):
        patrol=self.patrol();patrol.observe(self.observation(count=1),current_station_y_m=1.)
        self.assertEqual(patrol.decide(current_station_y_m=1.)['action'],'harvest')
        self.assertFalse(patrol.native_detached_keys)

    def test_changed_view_reopens_only_local_search_once_and_keeps_failure_history(self):
        patrol=self.patrol();patrol.observe(self.observation(),current_station_y_m=1.)
        key=next(iter(patrol.queue))
        for station in (1.,1.05):
            patrol.observe(self.observation(station),current_station_y_m=station)
            patrol.record_local_failure(key,'finite_camera_goal_planning_exhausted',dict(checked='native'),
                current_station_y_m=station,max_forward_views=2)
        self.assertEqual(patrol.queue[key]['status'],'finite_blocked')
        patrol.observe(self.observation(1.10),current_station_y_m=1.10)
        self.assertFalse(patrol.reconsider_visible(1.10))
        patrol.observe(self.observation(1.20),current_station_y_m=1.20)
        self.assertEqual(patrol.reconsider_visible(1.20),[key])
        self.assertEqual(patrol.decide(current_station_y_m=1.20)['action'],'harvest')
        self.assertEqual(len(patrol.queue[key]['planning_failures']),2)
        self.assertEqual(len(patrol.queue[key]['deferred_history']),1)
        patrol.record_local_failure(key,'finite_camera_goal_planning_exhausted',dict(checked='new-view'),
            current_station_y_m=1.20,max_forward_views=2)
        patrol.observe(self.observation(1.35),current_station_y_m=1.35)
        self.assertFalse(patrol.reconsider_visible(1.35));self.assertFalse(patrol.native_detached_keys)

    def test_physical_or_quarantined_blockers_are_never_reopened(self):
        patrol=self.patrol();patrol.observe(self.observation(),current_station_y_m=1.)
        key=next(iter(patrol.queue));patrol.queue[key]['planning_failures']=[dict(station_y_m=1.)]
        patrol.record_blocker(key,'native_physics_stop',dict(contact='actual'))
        patrol.observe(self.observation(1.20),current_station_y_m=1.20)
        self.assertFalse(patrol.reconsider_visible(1.20))

    def test_failed_footprint_tries_another_visible_subset_without_consuming_attempts(self):
        from test_camera_region_policy import CameraRegionPolicyTests
        fixture=CameraRegionPolicyTests();fixture.setUp();fixture.policy['minimum_fruits']=2
        fixture.policy['harvest_mode']='main_stem_only_geometric_harvest'
        fixture.policy['tool_height_interval_m']=[-.005,.079]
        # Spread ripe fruits vertically so the next footprint is distinct.
        for row,z in zip(fixture.observation['objects'],(1.12,1.06,1.00)):
            row['visible_surface_median_world_m'][2]=z
        calls=[];attempts={};before=copy.deepcopy(attempts)
        def grip(q,group,timestamp):
            calls.append((tuple(sorted(group['fruits'])),group['group_selection']['depth_m']))
            self.assertFalse(group['camera_region']['uses_hidden_gt_centers_for_aim'])
            if len(calls)==1:raise PlanningError('Native collision of the first footprint')
            return dict(checked=True)
        with tempfile.TemporaryDirectory()as folder:
            root=Path(folder);assembly=root/'assembly';assembly.mkdir()
            (assembly/'motion_manifest.json').write_text(json.dumps(dict(plant=dict(source_plant=fixture.source,fruits=fixture.fruits))))
            policy=root/'policy.json';policy.write_text(json.dumps(fixture.policy))
            source=dict(assembly=str(assembly),policy=str(policy),prefix='')
            poses={f['name']:np.r_[fixture.positions[i],[0.,0.,0.,1.]]for i,f in enumerate(fixture.fruits)}
            # This fixture faces +Y; rotate the observation and positions so
            # the V6 fixed -X entry reference sees the same footprint.
            rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
            camera=np.array(fixture.observation['world_from_camera_optical']);camera[:3,:]=rotation@camera[:3,:]
            fixture.observation['world_from_camera_optical']=camera.tolist()
            for row in fixture.observation['objects']:row['visible_surface_median_world_m']=(rotation@row['visible_surface_median_world_m']).tolist()
            for key,value in poses.items():value[:3]=rotation@value[:3]
            plan,group,evidence=plan_visible_alternatives(SimpleNamespace(grip=grip),np.zeros(10),fixture.observation,
                source,{},poses,set(),attempts,'Truss_02',root/'selections',.5,budget=4)
        self.assertIsNotNone(plan);self.assertNotEqual(calls[0],calls[1]);self.assertEqual(attempts,before)
        self.assertTrue(evidence[0]['planning_is_not_a_physical_attempt'])


if __name__=='__main__':unittest.main()
