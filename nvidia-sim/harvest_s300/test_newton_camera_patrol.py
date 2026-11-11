import unittest
import numpy as np
import mujoco as mj

from run_newton_camera_patrol import scan_plan, screen_preparation, planning_failure,ordered_proposals,segment_inputs
from newton_mounted_camera import camera_world,shape_instances
from plan_adapter import sample_plan


class CameraPatrolAdapterTests(unittest.TestCase):
    def test_route_preflight_rejects_before_gui_and_records_failure(self):
        import json,tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import patch
        from run_newton_camera_patrol import campaign,preflight_patrol,route_end
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for mode,limit in (('up',3.85),('down',.85)):
                (root/mode).mkdir()
                (root/mode/'patrol_assets.json').write_text(json.dumps(dict(elbow_posture=mode,end_station_m=limit)))
            (root/'patrol_assets.json').write_text(json.dumps(dict(default_start_mode='up',
                start_modes={m:str(root/m) for m in ('up','down')})))
            args=SimpleNamespace(assets=root,output=root/'failed',headless=False,elbow_mode='up',end_station=3.85)
            with patch('run_newton_camera_patrol.select_start_assets') as gui:
                with self.assertRaisesRegex(ValueError,'0.85m'):campaign(args)
                gui.assert_not_called()
            failure=json.loads((args.output/'summary.json').read_text())
            self.assertEqual(failure['stage'],'configuration')
            self.assertFalse(failure['physics_started'])
            self.assertEqual(json.loads((args.output/'launch.json').read_text())['options']['end_station'],3.85)
            args.headless=True
            self.assertEqual(preflight_patrol(args)['up']['end_station_m'],3.85)
            args.end_station=None
            self.assertEqual(preflight_patrol(args)['up']['end_station_m'],3.85)
            for invalid in (0.,-1.,float('inf'),float('nan'),3.851):
                with self.assertRaises(ValueError):route_end(dict(end_station_m=3.85),invalid)

    def test_startup_failure_keeps_diagnostic_before_physics_summary_exists(self):
        import json,tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import patch
        from run_newton_camera_patrol import campaign
        with tempfile.TemporaryDirectory() as folder:
            args=SimpleNamespace(output=Path(folder)/'run')
            with patch('run_newton_camera_patrol.preflight_patrol',return_value={}), \
                 patch('run_newton_camera_patrol.run_campaign',side_effect=RuntimeError('camera setup failed')):
                with self.assertRaisesRegex(RuntimeError,'camera setup failed'):campaign(args)
            self.assertEqual(json.loads((args.output/'summary.json').read_text())['stage'],'startup')
            self.assertIn('camera setup failed',json.loads((args.output/'error.json').read_text())['traceback'])

    def test_registered_static_fruit_catalog_requires_signed_logical_identity(self):
        import json,tempfile
        from pathlib import Path
        from run_newton_camera_patrol import source_fruit_manifest,sha256
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'fruits.json'
            data=dict(physical_attachments=False,plant=dict(source_plant='/World/Greenhouse/Plants/Plant_24',fruits=[]))
            path.write_text(json.dumps(data))
            source=dict(fruit_catalog=str(path),fruit_catalog_sha256=sha256(path),assembly='/not/needed')
            self.assertEqual(source_fruit_manifest(source),data)
            path.write_text(json.dumps(dict(data,physical_attachments=True)))
            with self.assertRaisesRegex(ValueError,'catalog changed'):source_fruit_manifest(source)
            source['fruit_catalog_sha256']=sha256(path)
            with self.assertRaisesRegex(ValueError,'logical static'):source_fruit_manifest(source)

    def test_fresh_missing_scan_uses_bounded_forward_views_without_harvest_credit(self):
        from camera_patrol import CameraPatrol
        from run_newton_camera_patrol import record_scan_reacquisition
        inventory=dict(trusses=[dict(plant_id=17,truss_index=0,gutter_center_x=-.775,
            root_position_world_m=[-.68,0.,.3],fruit_centroid_world_m=[-.6,1.,.5],rail_near=True)])
        patrol=CameraPatrol(inventory,-.775)
        optical=np.array([[0.,0.,-1.,0.],[1.,0.,0.,1.],[0.,-1.,0.,1.],[0.,0.,0.,1.]])
        visible=dict(objects=[dict(key=f'/World/Greenhouse/Plants/Plant_17/Truss_00/Fruit_{i:02d}',
            class_name='fruit',instance_id=i,red_proxy=True,visible_pixels=40,valid_depth_pixels=35,
            visible_surface_median_world_m=[-.6,1.,.5]) for i in (1,2)],
            world_from_camera_optical=optical.tolist(),timestamp_s=1.,current_timestamp_s=1.)
        patrol.observe(visible,current_station_y_m=1.)
        target=next(iter(patrol.queue))
        # A visible target and a non-scan command must not be marked failed.
        self.assertIsNone(record_scan_reacquisition(patrol,dict(action='scan',target_key=target),visible,1.))
        empty=dict(visible,objects=[])
        for station,expected in ((1.,'forward_view_retry_pending'),(1.05,'finite_deferred')):
            patrol.observe(empty,current_station_y_m=station)
            decision=patrol.decide(current_station_y_m=station)
            self.assertEqual(decision['action'],'scan')
            result=record_scan_reacquisition(patrol,decision,empty,station)
            self.assertEqual(result['status'],expected)
            self.assertIsNone(result['harvest_success'])
            following=patrol.decide(current_station_y_m=station)
            self.assertNotEqual(following['action'],'scan')
        self.assertEqual(patrol.queue[target]['status'],'finite_blocked')
        self.assertEqual(len(patrol.queue[target]['observed_attached_fruit_keys']),2)
        self.assertFalse(patrol.native_detached_keys)

    def test_combined_audit_keeps_collision_and_does_not_pass_incomplete_angles(self):
        from unittest.mock import patch
        from run_newton_camera_patrol import audit_static_plan,planned_entry_yaw
        plan=dict(total_duration_s=.2,approach_direction_world=[0.,1.,0.],
                  phases=[dict(name=name) for name in ('approach','withdraw')])
        for obstacle in ('.015 0 0','1 0 0'):
            model=mj.MjModel.from_xml_string(f'''<mujoco><worldbody>
            <geom name="stem" pos="{obstacle}" size=".01"/>
            <body name="Robot_link2" pos="0 0 .1"/><body name="Robot_link3" pos="0 0 .2"/>
            <body name="Tool_fixed"><joint type="slide" axis="1 0 0"/>
            <geom name="tool" size=".01" mass="1"/></body></worldbody></mujoco>''')
            bank=dict(qpos=np.zeros(1));qadr=np.array([0])
            with patch('run_newton_camera_patrol.sample_plan',return_value=dict(phase='approach',command=np.zeros(1))):
                angles,screen=audit_static_plan(model,bank,qadr,plan,[0.,1.,0.],elbow_posture='up')
                old=screen_preparation(model,bank,qadr,plan,active_body_names=set(),entire_path=True)
                self.assertEqual(screen,old)
                if screen['passed']:
                    original=planned_entry_yaw(model,bank,qadr,plan,[0.,1.,0.],elbow_posture='up')
                    for key in ('passed','samples','maximum_absolute_yaw_deg','elbow_posture'):
                        self.assertEqual(angles[key],original[key])
                    self.assertTrue(angles['complete'])
                else:
                    self.assertFalse(angles['passed']);self.assertFalse(angles['complete'])

    def test_start_profiles_resolve_mode_and_reject_mismatched_entry_policy(self):
        import tempfile,json
        from pathlib import Path
        from run_newton_camera_patrol import start_profiles
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for mode in ('up','down'):
                (root/mode).mkdir();(root/mode/'patrol_assets.json').write_text(json.dumps(dict(elbow_posture=mode)))
            (root/'patrol_assets.json').write_text(json.dumps(dict(default_start_mode='up',
                start_modes={m:str(root/m) for m in ('up','down')})))
            profiles,mode=start_profiles(root);self.assertEqual(mode,'up')
            self.assertEqual(start_profiles(root,'down')[0]['down'],root/'down')
            self.assertEqual(start_profiles(root,'down')[1],'down')
            with self.assertRaises(ValueError):start_profiles(root/'up','down')
            (root/'down/patrol_assets.json').write_text('{"elbow_posture":"up"}')
            with self.assertRaisesRegex(ValueError,'do not match'):start_profiles(root)

    def test_entry_yaw_bounds_actual_fk_and_recorded_pose_independently_of_candidate_name(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from scipy.spatial.transform import Rotation
        from run_newton_camera_patrol import entry_yaw_audit,recorded_entry_yaw,planned_entry_yaw,ENTRY_YAW_CANDIDATES
        self.assertTrue(all(abs(yaw)<=15 for yaw in ENTRY_YAW_CANDIDATES))
        def axes(degrees):
            angle=np.deg2rad(degrees);return np.c_[-np.cos(angle),-np.sin(angle),np.zeros(len(angle))]
        self.assertTrue(entry_yaw_audit(axes([-15.,0.,15.]),[-1.,0.,0.])['passed'])
        for outside in (15.01,-15.01,90.):
            self.assertFalse(entry_yaw_audit(axes([outside]),[-1.,0.,0.])['passed'])
        # Metadata claims straight entry, but commanded FK actually points90deg away.
        model=mj.MjModel.from_xml_string('<mujoco><worldbody><body name="Tool_fixed"><joint type="hinge" axis="0 0 1"/><geom size=".01" mass="1"/></body></worldbody></mujoco>')
        plan=dict(initial_command=[0.]*10,initial_gap_m=.044,settle_seconds=0.,total_duration_s=.2,
            approach_direction_world=[-1.,0.,0.],phases=[dict(name=name,duration_s=.1,
                waypoint_q=[[0.]*7]*2,rail_waypoint_positions_m=[0.,0.],end_gap_m=.044,belt_local_y_m_s=[0.,0.])
                for name in ('approach','withdraw')])
        with patch('run_newton_camera_patrol.sample_plan',
            return_value=dict(phase='approach',command=np.zeros(1))):
            self.assertFalse(planned_entry_yaw(model,dict(qpos=np.zeros(1)),np.array([0]),plan,[-1.,0.,0.])['passed'])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'states.npz'
            for actual,passed in ((14.,True),(16.,False)):
                pose=np.array([[0.,0.,0.,*Rotation.from_euler('z',90+actual,degrees=True).as_quat()]])
                np.savez(path,body_paths=['/actual/tool'],poses=np.tile(pose,(2,1,1)),
                    phase=['approach','withdraw'],time=[1.,2.])
                result=recorded_entry_yaw(path,plan,{'/actual/tool':'Tool_fixed'},[-1.,0.,0.])
                self.assertEqual(result['passed'],passed)
            mapping={'/actual/tool':'Tool_fixed','/actual/elbow':'Robot_link3','/actual/shoulder':'Robot_link2'}
            for elbow_z,passed in ((.2,True),(-.2,False)):
                pose=np.zeros((3,7));pose[:,6]=1.;pose[0,3:]=Rotation.from_euler('z',90,degrees=True).as_quat()
                pose[1,2]=elbow_z
                np.savez(path,body_paths=list(mapping),poses=np.tile(pose,(2,1,1)),
                    phase=['approach','withdraw'],time=[1.,2.])
                result=recorded_entry_yaw(path,plan,mapping,[-1.,0.,0.],require_elbow_up=True)
                self.assertEqual(result['passed'],passed)
                self.assertEqual(result['elbow_posture']['passed'],passed)
                result=recorded_entry_yaw(path,plan,mapping,[-1.,0.,0.],elbow_posture='down')
                self.assertEqual(result['passed'],not passed)
                self.assertEqual(result['elbow_posture']['required'],'down')

    def test_camera_relocation_keeps_housing_optical_alignment_and_other_geometry(self):
        import copy
        import xml.etree.ElementTree as ET
        from scipy.spatial.transform import Rotation
        from prepare_newton_patrol import relocate_camera_geometry
        tree=ET.ElementTree(ET.fromstring('''<mujoco><worldbody>
          <body name="old"><geom name="housing" pos="0 .08 0" quat="1 0 0 0" contype="0" conaffinity="0"/>
          <geom name="bracket" pos="0 .06 0" contype="0" conaffinity="0"/>
          <geom name="robot" contype="2" conaffinity="3"/></body><body name="new"/>
          </worldbody></mujoco>'''))
        old=np.eye(4);old[:3,3]=[0,.1,.03]
        new=np.eye(4);new[:3,:3]=Rotation.from_euler('x',30,degrees=True).as_matrix();new[:3,3]=[0,.1,.42]
        previous=dict(body_path='old_path',root_path='/old/camera',body_from_camera_optical=old.tolist())
        mounted=dict(body_path='new_path',root_path='/new/camera',body_from_camera_optical=new.tolist())
        geometry=[dict(name=name,source='/old/camera/'+name,collision=False,owner='old') for name in ('housing','bracket')]
        robot=ET.tostring(tree.find(".//geom[@name='robot']"))
        relative=np.eye(4);relative[:3,3]=[0,-.02,-.03]
        names=relocate_camera_geometry(tree,geometry,dict(old_path='old',new_path='new'),previous,mounted)
        geom=tree.find(".//body[@name='new']/geom[@name='housing']")
        actual=np.eye(4);actual[:3,3]=np.fromstring(geom.get('pos'),sep=' ')
        q=np.fromstring(geom.get('quat'),sep=' ');actual[:3,:3]=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        np.testing.assert_allclose(np.linalg.inv(new)@actual,relative,atol=1e-12)
        self.assertEqual(names,['housing','bracket'])
        self.assertEqual(ET.tostring(tree.find(".//geom[@name='robot']")),robot)
        self.assertTrue(all(row['owner']=='new' and row['source'].startswith('/new/camera/') for row in geometry))
        bad=copy.deepcopy(geometry);bad[0]['source']='/old/camera/housing';bad[1]['source']='/old/camera/bracket';bad[0]['collision']=True
        with self.assertRaisesRegex(ValueError,'visual housing and bracket'):
            relocate_camera_geometry(tree,bad,dict(old_path='old',new_path='new'),previous,mounted)

    def test_error_finish_shows_cause_and_last_location_instead_of_completed_label(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        import threading
        from newton_patrol_viewer import PatrolViewer
        viewer=PatrolViewer.__new__(PatrolViewer)
        viewer.status=SimpleNamespace(content='');viewer.start=SimpleNamespace(label='',disabled=False)
        viewer.stop_event=threading.Event();viewer.stop_event.set();viewer.server=Mock()
        viewer.finish(dict(status='error',error='Planning source changed during hashing: stage.usdc',
                           last_station_m=.45,harvest_attempts=0,assumed_harvest_count=0))
        self.assertIn('Planning source changed during hashing',viewer.status.content)
        self.assertIn('0.45m',viewer.status.content)
        self.assertEqual(viewer.start.label,'중단')
        self.assertTrue(viewer.start.disabled)
        viewer.server.stop.assert_called_once()

    def test_static_segments_share_exact_metadata_and_preserve_measured_clock_and_warmstart(self):
        import tempfile,json
        from pathlib import Path
        from newton_inputs import sha256
        from newton_continuation import VerificationContext
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);common=root/'source';common.mkdir()
            manifest=root/'motion_manifest.json'
            manifest.write_text(json.dumps(dict(rail=dict(initial_station_y_m=0.))))
            meta=dict(active_window=dict(all_plants_static=True),source_trial=str(root),source_manifest=str(manifest))
            (common/'source_export.json').write_text(json.dumps(meta))
            initial=dict(qpos=np.zeros(10),qvel=np.zeros(10),eq_active=np.ones(1,bool),
                ctrl=np.zeros(10),runtime_dof_damping=np.zeros(10),robot_qpos_addresses=np.arange(10))
            np.savez_compressed(common/'initial_state.npz',**initial)
            (common/'input_model.mjb').write_bytes(b'immutable model fixture')
            files=('source_export.json','initial_state.npz','input_model.mjb')
            (common/'inputs.json').write_text(json.dumps(dict(input_files={name:dict(sha256=sha256(common/name)) for name in files})))
            bank=dict(initial,time=np.array([4.]),qacc_warmstart=np.ones(10)*.02)
            output=root/'segment'
            segment_inputs(common,output,scan_plan(initial['qpos'],destination=.1),bank,17,1,
                           source_metadata=meta,verification_context=VerificationContext(scope='patrol'))
            self.assertTrue((output/'source_export.json').is_symlink())
            self.assertEqual((output/'source_export.json').read_bytes(),(common/'source_export.json').read_bytes())
            with np.load(output/'initial_state.npz') as saved:
                np.testing.assert_array_equal(saved['time'],bank['time'])
                np.testing.assert_array_equal(saved['qacc_warmstart'],bank['qacc_warmstart'])

    def test_bounded_views_explore_new_directions_without_losing_fallbacks(self):
        from run_newton_camera_patrol import ENTRY_YAW_CANDIDATES
        headings=[(yaw,{'aim':yaw}) for yaw in ENTRY_YAW_CANDIDATES]
        connectors=[(name,{}) for name in ('level','combined','joint')]
        directions=[]
        for view in range(5):
            ordered=ordered_proposals(headings,connectors,(0.,'combined'),view)
            self.assertEqual(len(ordered),15)
            self.assertEqual(len({(r[0],r[2]) for r in ordered}),15)
            directions.append(ordered[0][0])
            self.assertEqual(ordered[0][2],'combined')
        self.assertEqual(directions,list(ENTRY_YAW_CANDIDATES))

    def test_fast_priority_preserves_heading_search_and_fallbacks(self):
        from run_newton_camera_patrol import ENTRY_YAW_CANDIDATES
        headings=[(yaw,{}) for yaw in ENTRY_YAW_CANDIDATES]
        connectors=[(name,{}) for name in ('level','combined','joint')]
        for view in range(5):
            rows=ordered_proposals(headings,connectors,(0.,'joint'),view)
            self.assertEqual(rows[0][2],'joint')
            self.assertEqual([r[0] for r in rows[:3]],[ENTRY_YAW_CANDIDATES[view]]*3)
            self.assertEqual({r[2] for r in rows[:3]},{'level','combined','joint'})
            self.assertEqual(len({(r[0],r[2]) for r in rows}),15)

    def test_invalid_planner_input_is_not_an_ik_failure(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as root:
            log=Path(root)/'planning.log'
            log.write_text('Traceback\nValueError: Grasp depth outside mechanical travel\n')
            with self.assertRaisesRegex(RuntimeError,'not an IK search result'):planning_failure(log)
            log.write_text('Traceback\nRuntimeError: Fixed entry approach IK not found\n')
            self.assertEqual(planning_failure(log)[0],'ik_not_found')

    def test_short_step_preserves_non_rail_axes_and_belt_off(self):
        q=np.array([.1,.2,1.,-.8,2.5,-.07,3.14,.01,-.026,.026])
        plan=scan_plan(q,destination=.25)
        for t in np.linspace(0,plan['total_duration_s'],50):
            sample=sample_plan(plan,t)
            np.testing.assert_allclose(sample['command'][1:],q[1:])
            self.assertGreaterEqual(sample['command'][0],q[0])
            self.assertLessEqual(sample['command'][0],.25)
            np.testing.assert_array_equal(sample['authored_belt_speed'],[0,0])
        self.assertAlmostEqual(sample_plan(plan,plan['total_duration_s'])['command'][0],.25)

    def test_no_large_camera_blind_move(self):
        with self.assertRaises(ValueError):scan_plan(np.zeros(10),destination=.25)

    def test_camera_extrinsic_follows_measured_body(self):
        local=np.eye(4);local[:3,3]=[.1,.2,.3]
        pose=[1.,2.,3.,0.,0.,0.,1.]
        np.testing.assert_allclose(camera_world(pose,local)[:3,3],[1.1,2.2,3.3])

    def test_background_shape_never_becomes_fruit(self):
        shape=np.array([[0,1,2,0xffffffff]],np.uint32)
        np.testing.assert_array_equal(shape_instances(shape,[11,22]),[[11,22,0,0]])

    def test_screen_blocks_rigid_but_preserves_flexible_overlap_as_diagnostic(self):
        for obstacle,expected in [('gutter',False),('Truss_02_Pedicel',True)]:
            dummy=''.join(f'<body pos="0 0 {i+1}"><joint type="slide"/><geom size=".001" contype="0" conaffinity="0"/></body>' for i in range(9))
            model=mj.MjModel.from_xml_string(f'<mujoco><worldbody><body name="Robot_link1"><joint type="slide" axis="1 0 0"/><geom size=".02"/></body>{dummy}<body name="{obstacle}" pos=".1 0 0"><geom size=".02"/></body></worldbody></mujoco>')
            q=np.zeros(10);plan=scan_plan(q,destination=.12)
            report=screen_preparation(model,dict(qpos=q),np.arange(10),plan)
            self.assertEqual(report['passed'],expected)
            self.assertFalse(report['collision_certified'])
            if expected:self.assertTrue(report['elastic_intersections'])
            else:self.assertIn('Robot_link1',report['bodies'])
            if obstacle.startswith('Truss_'):
                frozen=screen_preparation(model,dict(qpos=q),np.arange(10),plan,
                    active_body_names={'DifferentTruss'})
                self.assertFalse(frozen['passed'])


if __name__=='__main__':unittest.main()
