"""An explicit forward pass never converts attached or untried fruit to success."""
import contextlib
import copy
import io
import json
import unittest
from unittest.mock import patch
from camera_patrol import CameraPatrol
import camera_patrol_runner as runner
import run_row_harvest as row
import test_camera_patrol_runner as fixtures


class ForwardPassDecisions(unittest.TestCase):
    active='/World/Greenhouse/Plants/Plant_20'
    future='/World/Greenhouse/Plants/Plant_22'
    rear='/World/Greenhouse/Plants/Plant_18'
    frozen='/World/Greenhouse/Plants/Plant_19'
    def fixture(self):
        inventory=dict(trusses=[dict(plant_id=p,truss_index=t,gutter_center_x=-.775,
            root_position_world_m=[-.68,0.,.3],fruit_centroid_world_m=[-.6,99.,9.],rail_near=True)
            for p,t in ((20,0),(20,2),(22,1),(18,0),(19,0))])
        patrol=CameraPatrol(inventory,-.775,step_m=.1)
        camera=[[0.,0.,-1.,0.],[1.,0.,0.,1.],[0.,-1.,0.,1.],[0.,0.,0.,1.]]
        objects=[]
        for plant,truss,y in ((20,0,1.02),(20,2,1.03),(22,1,1.5),(18,0,.8),(19,0,.9)):
            objects.extend(dict(key=f'/World/Greenhouse/Plants/Plant_{plant}/Truss_{truss:02d}/Fruit_{fruit:02d}',
                instance_id=plant*1000+truss*100+fruit,class_name='fruit',red_proxy=True,visible_pixels=30,
                valid_depth_pixels=30,visible_surface_median_world_m=[-.6,y+(fruit-1.5)*.01,1.])for fruit in(1,2))
        patrol.observe(dict(objects=objects,world_from_camera_optical=camera,timestamp_s=2.,current_timestamp_s=2.),current_station_y_m=1.)
        patrol.observe(dict(objects=[obj for obj in objects if obj['key'].startswith((self.active+'/',self.future+'/'))],
            world_from_camera_optical=camera,timestamp_s=3.,current_timestamp_s=3.),current_station_y_m=1.)
        return patrol,inventory

    def test_default_and_explicit_false_are_identical(self):
        p,inventory=self.fixture();q=CameraPatrol.from_dict(inventory,p.to_dict())
        a=p.decide(current_station_y_m=1.,active_source_plant=self.active)
        b=q.decide(current_station_y_m=1.,active_source_plant=self.active,forward_pass=False)
        self.assertEqual(a,b);self.assertNotIn('forward_pass',a)

    def test_independent_finite_target_does_not_defer_other_visible_active_truss(self):
        p,_=self.fixture();target=self.active+'/Truss_00';p.decide(current_station_y_m=1.,active_source_plant=self.active)
        result=p.record_local_failure(target,'finite_current_camera_pool_exhausted',dict(actual_trial='normal_trial',accepted_terminal_checkpoint='exact_latest'),
            current_station_y_m=1.,native_observation_executed=True,max_forward_views=1)
        self.assertEqual(result['status'],'finite_deferred')
        decision=p.decide(current_station_y_m=1.,active_source_plant=self.active,forward_pass=True)
        self.assertEqual(decision['action'],'harvest');self.assertEqual(decision['target_key'],self.active+'/Truss_02')
        self.assertEqual(p.queue[self.active+'/Truss_02']['blockers'],[])
        self.assertEqual(len(p.queue[self.active+'/Truss_02']['observed_attached_fruit_keys']),2)
        self.assertEqual(p.native_detached_keys,set());self.assertIsNone(decision['harvest_success'])

    def test_rear_frozen_revisits_do_not_block_forward_after_all_active_resolved(self):
        p,inventory=self.fixture()
        for truss in(0,2):p.record_blocker(self.active+f'/Truss_{truss:02d}','finite_tested_camera_pool',dict(trial=f'actual_{truss}',completed_harvest=False))
        q=CameraPatrol.from_dict(inventory,p.to_dict())
        old=q.decide(current_station_y_m=1.,active_source_plant=self.active,frozen_source_plants=[self.frozen])
        self.assertEqual(old['action'],'stop')
        before=copy.deepcopy(p.queue)
        decision=p.decide(current_station_y_m=1.,active_source_plant=self.active,frozen_source_plants=[self.frozen],forward_pass=True)
        self.assertEqual(decision['action'],'move');self.assertAlmostEqual(decision['movement_m'],.1)
        self.assertEqual(decision['target_key'],self.future+'/Truss_01')
        self.assertEqual(set(decision['forward_pass']['inactive_revisit_target_keys_preserved']),{self.rear+'/Truss_00',self.frozen+'/Truss_00'})
        self.assertEqual(set(p.queue),set(before));self.assertEqual(p.queue[self.rear+'/Truss_00']['fruit_evidence'],before[self.rear+'/Truss_00']['fruit_evidence'])
        self.assertEqual(len(p.queue[self.future+'/Truss_01']['observed_attached_fruit_keys']),2)
        self.assertEqual(p.native_detached_keys,set());self.assertEqual(decision['forward_pass']['harvest_credit'],0)

    def test_active_untried_revisit_still_blocks_and_stale_view_still_scans(self):
        p,_=self.fixture();p.queue[self.active+'/Truss_00']['service_station_y_m']=.7
        p.record_blocker(self.active+'/Truss_02','independent_finite_failure',dict(trial='actual'))
        d=p.decide(current_station_y_m=1.,active_source_plant=self.active,forward_pass=True)
        self.assertEqual(d['action'],'stop');self.assertIn(self.active+'/Truss_00',d['pending_target_keys'])
        p.observations[-1]['stale']=True
        self.assertEqual(p.decide(current_station_y_m=1.,active_source_plant=self.active,forward_pass=True)['action'],'scan')
        with self.assertRaisesRegex(ValueError,'explicit boolean'):p.decide(current_station_y_m=1.,forward_pass=1)

    def test_actual_cli_passes_option_only_when_enabled_and_rejects_non_camera(self):
        argv=['row','/assembly','/inventory','-','/output','--camera-patrol','--target-truss-windows','--motion-arguments','/profile']
        for enabled in(False,True):
            with patch('sys.argv',argv+(['--patrol-forward-pass']if enabled else [])),patch.object(row,'_read',return_value=[]),\
                patch.object(runner,'camera_patrol_campaign',return_value={})as campaign,contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(row.main(),0)
            if enabled:self.assertIs(campaign.call_args.kwargs['forward_pass'],True)
            else:self.assertNotIn('forward_pass',campaign.call_args.kwargs)
        with patch('sys.argv',['row','/a','/i','-','/o','--patrol-forward-pass']),contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit)as error:row.main()
        self.assertEqual(error.exception.code,2)


class ForwardPassRunner(unittest.TestCase):
    setUp=fixtures.CameraPatrolRunnerTests.setUp
    write=fixtures.CameraPatrolRunnerTests.write
    completed=fixtures.CameraPatrolRunnerTests.completed
    measured=fixtures.CameraPatrolRunnerTests.measured
    observation_files=fixtures.CameraPatrolRunnerTests.observation_files
    benchmark=fixtures.CameraPatrolRunnerTests.benchmark
    planning_only_benchmark=fixtures.CameraPatrolRunnerTests.planning_only_benchmark
    def test_known_completed_pool_uses_one_view_budget_not_another_height_retry(self):
        self.manifest['plant_interaction_variant']=dict(target_truss='Truss_00')
        self.write(self.assembly/'motion_manifest.json',self.manifest)
        with patch('patrol_lift_search.lift_bounds_for_assembly',return_value=[0.,.75]),\
                patch.object(runner,'target_truss_camera_scope',return_value=dict(target_truss_window=dict(target_truss='Truss_00'))):
            result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'forward',fixtures.BASE,
                execute=True,max_decisions=4,benchmark_runner=self.planning_only_benchmark,
                adaptive_lift=True,target_truss_windows=True,forward_pass=True)
        self.assertEqual([r['decision']['action']for r in result['segments']],['scan','move','harvest','move'])
        failed=result['segments'][2]
        self.assertEqual(failed['planning_deferral']['status'],'finite_deferred')
        self.assertEqual(failed['harvest_outcome']['actual_completed_cycles'],0)
        ledger=json.loads((self.root/'forward/camera_patrol_queue.json').read_text())
        item=ledger['queue'][self.source+'/Truss_00']
        self.assertEqual(item['status'],'finite_blocked');self.assertEqual(len(item['planning_failures']),1)
        self.assertEqual(item['blockers'][-1]['evidence']['max_forward_views'],1)
        self.assertFalse(item['blockers'][-1]['impossible'])
        self.assertEqual(result['achieved_distinct_trusses'],0);self.assertEqual(ledger['native_detached_keys'],[])
        self.assertEqual(result['segments'][3]['parent_checkpoint'],failed['checkpoint'])

    def test_unknown_pipeline_error_never_becomes_forward_permission(self):
        self.manifest['plant_interaction_variant']=dict(target_truss='Truss_00')
        self.write(self.assembly/'motion_manifest.json',self.manifest)
        def unknown(command,check):return self.planning_only_benchmark(command,check,unknown=True)
        with patch.object(runner,'target_truss_camera_scope',return_value=dict(target_truss_window=dict(target_truss='Truss_00'))):
            result=runner.camera_patrol_campaign(self.assembly,self.inventory,self.root/'unknown',fixtures.BASE,
                execute=True,max_decisions=8,benchmark_runner=unknown,target_truss_windows=True,forward_pass=True)
        self.assertEqual(result['status'],'blocked_preserving_physical_history')
        self.assertEqual(len(result['segments']),3);self.assertEqual(result['achieved_distinct_trusses'],0)
        self.assertNotIn('planning_deferral',result['segments'][-1])


if __name__=='__main__':unittest.main()
