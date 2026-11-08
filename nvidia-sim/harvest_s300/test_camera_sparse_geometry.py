"""Finite current-view deferral uses visible depth and the unchanged selector."""
import ast,copy,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from camera_patrol import CameraPatrol
import camera_patrol_runner as runner
import test_camera_forward_pass as fixtures

class SparseGeometryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        original,_=fixtures.ForwardPassDecisions().fixture();self.patrol=CameraPatrol(original.inventory,-.775,step_m=.1)
        self.key='/World/Greenhouse/Plants/Plant_20/Truss_00';self.trial=self.root/'trial';self.trial.mkdir()
        self.objects=[dict(key=self.key+f'/Fruit_{i:02d}',instance_id=i,class_name='fruit',red_proxy=True,
            visible_pixels=30,valid_depth_pixels=30,visible_surface_median_world_m=[-.5,1.1,z])for i,z in((1,1.18),(3,1.12),(7,1.06))]
        self.observation=dict(objects=self.objects,objects_path=str(self.trial/'objects.json'),timestamp_s=5.,current_timestamp_s=5.,
            world_from_camera_optical=[[0.,0.,-1.,0.],[1.,0.,0.,1.],[0.,-1.,0.,1.],[0.,0.,0.,1.]])
        self.patrol.observe(self.observation,current_station_y_m=1.)
        self.fruits=[dict(name=f'Truss_00_Fruit_{i:02d}',path=f'/body{i}',joint=f'/joint{i}',dimensions_m=[.024]*3)for i in(1,3,7)]
        self.manifest=dict(scene_sha256='a'*64,plant=dict(source_plant=self.key.rsplit('/',1)[0],fruits=self.fruits),grasp_reference={})
        (self.root/'motion_manifest.json').write_text(json.dumps(self.manifest));(self.trial/'harvest_group_policy.json').write_text('{}');(self.trial/'summary.json').write_text('{}')
        self.policy=dict(minimum_fruits=2,source_sha256={},fruit_eligibility=[dict(f,eligible=True)for f in self.fruits])
        self.arrays=dict(body_paths=np.array([f['path']for f in self.fruits]),body_poses_world=np.array([[-.5,1.1,z,0.,0.,0.,1.]for z in(1.18,1.12,1.06)]))
        self.metadata=dict(time_s=5.,metadata_sha256='exact',sequence=dict(attempt_counts={}),break_events=[])
        for change in(patch.object(runner,'latest_camera_observation',side_effect=lambda *a:self.observation),
            patch.object(runner,'load_checkpoint',side_effect=lambda *a,**k:(self.metadata,self.arrays)),
            patch.object(runner,'_active_collision_target_key',return_value=self.key),
            patch('harvest_group_policy.load_policy',return_value=self.policy)):
            change.start();self.addCleanup(change.stop)
        self.arguments=['--grasp-depth-mm','15','--later-grasp-depth-mm','15','--grasp-height-offset-mm','-10','--camera-heading-offsets-deg','0','30','-30']
    def evaluate(self):return runner.complete_visible_footprint_deferral(self.patrol,self.key,assembly=self.root,
        trial=self.trial,summary=dict(simulated_s=5.),checkpoint=self.root/'cp',arguments=self.arguments)
    def test_all_visible_known_three_are_evaluated_by_real_selector_at_all_headings(self):
        before=self.arrays['body_poses_world'].copy();proof=self.evaluate();self.assertIsNotNone(proof)
        self.assertEqual([r['heading_offset_deg']for r in proof['existing_selector_evaluations']],[0.,30.,-30.])
        self.assertTrue(all(r['best_geometric_count']==1 for r in proof['existing_selector_evaluations']))
        self.assertEqual(len(proof['exact_current_visible_and_remembered_attached_fruit_keys']),3)
        self.assertFalse(proof['IK_or_physics_impossibility']);self.assertTrue(proof['partial_contact_not_evaluated'])
        self.assertEqual(proof['physics_steps'],0);self.assertEqual(proof['harvest_credit'],0)
        np.testing.assert_array_equal(before,self.arrays['body_poses_world']);self.assertEqual(self.patrol.native_detached_keys,set())
    def test_incomplete_stale_or_other_receipt_never_defers_unseen_fruit(self):
        self.patrol.queue[self.key]['last_visible_fruit_keys'].pop();self.assertIsNone(self.evaluate())
        self.patrol.queue[self.key]['last_visible_fruit_keys']=sorted(self.patrol.queue[self.key]['observed_attached_fruit_keys'])
        self.patrol.observations[-1]['stale']=True;self.assertIsNone(self.evaluate());self.patrol.observations[-1]['stale']=False
        self.observation['objects_path']='/other/trial/objects.json';self.assertIsNone(self.evaluate())
    def test_visible_depth_feasible_pair_prevents_deferral_even_when_hidden_gt_still_sparse(self):
        self.objects[0]['visible_surface_median_world_m'][2]=1.14
        self.assertIsNone(self.evaluate())  # Actual raw body array remains sparse; aim comes from visible depth.
    def test_prior_complete_view_keeps_initial_fresh_scan_and_does_not_repeat_height(self):
        function=next(n for n in ast.parse(Path(runner.__file__).read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='camera_patrol_campaign')
        before=next(n for n in function.body if isinstance(n,ast.If)and ast.unparse(n.test).startswith('forward_pass and prior_pending_plan_failure'))
        initial=next(n for n in function.body if isinstance(n,ast.If)and 'initial_lift_height_m is None'in ast.unparse(n.test))
        proof=self.evaluate();pending=dict(target_key=self.key,trial=str(self.trial),outcome=dict(requires_fresh_camera_alignment=True))
        ns=dict(Path=Path,forward_pass=True,prior_pending_plan_failure=pending,complete_visible_footprint_deferral=lambda *a,**k:proof,
            patrol=self.patrol,assembly=self.root,checkpoint=self.root/'cp',base_arguments=self.arguments,_read=lambda p:{},
            initial_lift_height_m=None,adaptive_lift=True,copy=copy,lift=.1,station=1.,lift_limits=[0.,.75],decision=dict(action='scan'),report={})
        exec(compile(ast.fix_missing_locations(ast.Module(body=[before,initial],type_ignores=[])),'actual_initial_geometry_dispatch','exec'),ns)
        self.assertEqual(ns['decision']['action'],'scan');self.assertIn('complete_visible_footprint_deferral',ns['prior_pending_plan_failure'])
        self.assertNotIn('camera_lift_search',self.patrol.queue[self.key])

if __name__=='__main__':unittest.main()
