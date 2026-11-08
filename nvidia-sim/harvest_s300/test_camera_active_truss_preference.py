"""Opt-in cost ranking cannot create camera eligibility or harvest credit."""
import contextlib
import copy
import io
import unittest
from unittest.mock import patch
import numpy as np
from camera_patrol import CameraPatrol
import camera_patrol_runner as runner
import run_row_harvest as row


class ActiveTrussPreferenceTests(unittest.TestCase):
    source='/World/Greenhouse/Plants/Plant_20'
    def setUp(self):
        self.inventory=dict(trusses=[dict(plant_id=20,truss_index=t,gutter_center_x=-.775,
            root_position_world_m=[-.68,0.,.3],fruit_centroid_world_m=[-.6,0.,9.],rail_near=True)
            for t in (0,2)])
        self.active=self.source+'/Truss_00';self.other=self.source+'/Truss_02'
        self.camera=[[0.,0.,-1.,0.],[1.,0.,0.,1.],[0.,-1.,0.,1.],[0.,0.,0.,1.]]

    def objects(self,truss,y,z):
        return [dict(key=self.source+f'/Truss_{truss:02d}/Fruit_{i:02d}',class_name='fruit',
            instance_id=truss*100+i,red_proxy=True,visible_pixels=30,valid_depth_pixels=30,
            visible_surface_median_world_m=[-.6,y+(i-1.5)*.01,z])for i in (1,2)]

    def observe(self,patrol,objects,**extra):
        view=dict(objects=objects,world_from_camera_optical=self.camera,timestamp_s=2.,current_timestamp_s=2.,
            observation_id='actual_frame');view.update(extra)
        return patrol.observe(view,current_station_y_m=1.)

    def fixture(self,active_y=1.12):
        p=CameraPatrol(self.inventory,-.775)
        self.observe(p,self.objects(0,active_y,1.)+self.objects(2,1.02,1.3));return p

    def choose(self,p,key=True):
        kwargs=dict(current_station_y_m=1.,active_source_plant=self.source)
        if key:kwargs['active_collision_target_key']=self.active
        return p.decide(**kwargs)

    def test_default_and_explicit_none_preserve_old_decisions(self):
        a=self.fixture();b=CameraPatrol.from_dict(self.inventory,a.to_dict())
        first=self.choose(a,False)
        second=b.decide(current_station_y_m=1.,active_source_plant=self.source,active_collision_target_key=None)
        self.assertEqual(first,second);self.assertEqual(first['target_key'],self.other)
        self.assertNotIn('active_truss_cost_preference',first)

    def test_prefers_only_fresh_visible_active_candidate_without_erasing_other(self):
        p=self.fixture();decision=self.choose(p)
        self.assertEqual(decision['action'],'harvest');self.assertEqual(decision['target_key'],self.active)
        self.assertEqual(decision['reason'],'current_visible_group_bounded_arm_service_proposal')
        self.assertTrue(decision['active_truss_cost_preference']['applied'])
        self.assertEqual(set(decision['active_truss_cost_preference']['eligible_current_visible_target_keys']),
            {self.active,self.other})
        self.assertEqual(p.queue[self.other]['status'],'pending');self.assertIsNone(decision['harvest_success'])

    def test_hidden_or_out_of_range_active_target_falls_back(self):
        for far in (False,True):
            p=self.fixture(active_y=1.3 if far else 1.12)
            if not far:self.observe(p,self.objects(2,1.02,1.3))
            decision=self.choose(p);self.assertEqual(decision['target_key'],self.other)
            self.assertFalse(decision['active_truss_cost_preference']['applied'])

    def test_detached_blocked_or_unknown_active_is_not_resurrected(self):
        for status in ('detached','blocked','unknown'):
            p=self.fixture();key=self.active
            if status=='detached':p.record_native_detachments([self.active+f'/Fruit_{i:02d}'for i in (1,2)])
            if status=='blocked':p.record_blocker(self.active,'finite_collision',dict(report='actual.json'))
            if status=='unknown':key=self.source+'/Truss_99'
            decision=p.decide(current_station_y_m=1.,active_source_plant=self.source,active_collision_target_key=key)
            self.assertEqual(decision['target_key'],self.other)
            self.assertFalse(decision['active_truss_cost_preference']['applied'])
            self.assertNotIn(self.source+'/Truss_99',p.queue)

    def test_stale_view_remains_scan_and_wrong_source_is_rejected(self):
        p=self.fixture();self.observe(p,self.objects(0,1.12,1.)+self.objects(2,1.02,1.3),current_timestamp_s=4.)
        self.assertEqual(self.choose(p)['action'],'scan')
        with self.assertRaisesRegex(ValueError,'exact current source'):
            p.decide(current_station_y_m=1.,active_source_plant=self.source,
                active_collision_target_key='/World/Greenhouse/Plants/Plant_99/Truss_00')

    def test_runner_identity_uses_existing_collision_scope_validator(self):
        manifest=dict(plant=dict(source_plant=self.source));inventory={'fixture':True}
        with patch.object(runner,'target_truss_camera_scope',return_value=dict(target_truss_window=dict(target_truss='Truss_00')))as scope:
            self.assertEqual(runner._active_collision_target_key(manifest,inventory),self.active)
            scope.assert_called_once_with(manifest,inventory)
        with patch.object(runner,'target_truss_camera_scope',side_effect=ValueError('invalid scope')):
            with self.assertRaisesRegex(ValueError,'invalid scope'):runner._active_collision_target_key(manifest,inventory)

    def test_real_cli_passes_opt_in_only_when_explicit_and_rejects_wrong_mode(self):
        base=['row','/assembly','/inventory','-','/output','--camera-patrol','--target-truss-windows',
            '--motion-arguments','/profile.json']
        for enabled in (False,True):
            argv=base+(['--patrol-prefer-active-truss']if enabled else [])
            with patch('sys.argv',argv),patch.object(row,'_read',return_value=[]),\
                    patch.object(runner,'camera_patrol_campaign',return_value={})as campaign,\
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(row.main(),0)
            if enabled:self.assertIs(campaign.call_args.kwargs['prefer_active_truss'],True)
            else:self.assertNotIn('prefer_active_truss',campaign.call_args.kwargs)
        with patch('sys.argv',['row','/assembly','/inventory','-','/output','--patrol-prefer-active-truss']),\
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit)as error:row.main()
        self.assertEqual(error.exception.code,2)


if __name__=='__main__':unittest.main()
