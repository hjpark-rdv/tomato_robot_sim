import copy
import unittest
from types import SimpleNamespace
import numpy as np

from side_camera_motion import observed_capture_goal,SideCameraMotion,PlanningError,level_entry_error,LEVEL_ENTRY_TOLERANCE_DEG,responsive_duration
from side_camera_scene import visual_pose_indices
from plan_adapter import sample_plan


class SideCameraPatrolTests(unittest.TestCase):
    def test_staging_uses_roller_mouth_and_selected_visible_fruit_bounds(self):
        from side_camera_motion import staged_entry_spec,PREGRASP_CLEARANCE_M,entry_line_error,entry_rejection
        from preview_workspace_motion import CAPTURE_CENTER
        rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        group=dict(fruits=['a'],camera_region=dict(accepted_observations=[
            dict(name='a',estimated_center_world_m=[-.60,.4,.9],known_bounding_radius_m=.015),
            dict(name='other',estimated_center_world_m=[10.,10.,10.],known_bounding_radius_m=.1)]))
        centre=np.array([-.57,.4,.9]);spec=staged_entry_spec(group,centre,rotation)
        start=np.array(spec['start_capture_m']);direction=rotation[:,1]
        mouth=start@direction+.158-CAPTURE_CENTER[1]
        self.assertGreaterEqual(spec['near_surface_projection_m']-mouth,PREGRASP_CLEARANCE_M)
        self.assertGreater(spec['stroke_m'],.06)
        for t in np.linspace(0.,1.,40):self.assertLess(entry_line_error(start+t*spec['stroke_m']*direction,spec)[0],1e-10)
        data=SimpleNamespace(xpos=np.array([centre-rotation@CAPTURE_CENTER]),xmat=rotation.reshape(1,9))
        self.assertIsNone(entry_rejection(data,0,spec))
        data.xpos[0,1]+=.01
        self.assertEqual(entry_rejection(data,0,spec)['rule'],'nonlinear_gripper_insertion')
        group['fruits']=['hidden']
        with self.assertRaises(PlanningError):staged_entry_spec(group,centre,rotation)

    def test_entry_plane_constraint_cannot_reuse_an_unconstrained_edge(self):
        from test_newton_ompl import scene
        from side_camera_motion import LocalCheckedPlanner
        from preview_grip_workspace import checked_edge
        base=scene();planner=LocalCheckedPlanner(base.model,dict(qpos=base.initial),
            SimpleNamespace(update=lambda _:None,aliases={}),model_sha256='test')
        a=base.initial.copy();b=a.copy();b[2]=-.4
        planner.begin_local_checks()
        self.assertTrue(checked_edge(planner,a[1:],b[1:]))
        def forbidden_middle(data):
            return dict(rule='crossed_staging_plane')if -.55<data.qpos[2]<-.45 else None
        with planner.constraint(forbidden_middle,'pregrasp-test'):
            self.assertTrue(planner.valid_full(a));self.assertTrue(planner.valid_full(b))
            self.assertFalse(checked_edge(planner,a[1:],b[1:]))
        self.assertIsNone(planner.state_constraint)
        self.assertTrue(checked_edge(planner,a[1:],b[1:]));planner.end_local_checks()

    def test_native_angular_error_matches_shortest_rotation(self):
        from scipy.spatial.transform import Rotation
        from preview_grip_workspace import angular_error
        rotations=Rotation.random(500,random_state=42).as_matrix()
        for target,actual in zip(rotations[:-1],rotations[1:]):
            np.testing.assert_allclose(angular_error(target,actual),
                Rotation.from_matrix(target@actual.T).as_rotvec(),rtol=1e-10,atol=1e-12)
        for degrees in (0.,1e-9,179.999999,180.):
            target=Rotation.from_euler('y',degrees,degrees=True).as_matrix()
            vector=angular_error(target,np.eye(3))
            np.testing.assert_allclose(Rotation.from_rotvec(vector).as_matrix(),target,atol=1e-12)

    def test_fk_is_shared_without_skipping_geometry_or_collision_guards(self):
        from unittest.mock import patch
        import mujoco as mj
        from test_newton_ompl import scene
        from preview_grip_workspace import GripPlanner
        base=scene();planner=GripPlanner(base.model,dict(qpos=base.initial),
            SimpleNamespace(update=lambda _:None,aliases={}),model_sha256='test')
        calls=[]
        def geometry(data):
            calls.append('geometry')
            return dict(rule='test_posture')if data.qpos[3]>.5 else None
        def guard(data,*,geometry_checked=False):
            self.assertTrue(geometry_checked);calls.append('contacts');return None
        planner.posture_guard=SimpleNamespace(joint_rejection=lambda _:None,
            geometry_rejection=geometry,rejection=guard)
        with patch('newton_ompl.mj.mj_kinematics',wraps=mj.mj_kinematics)as fk:
            self.assertTrue(planner.valid_full(base.initial));self.assertEqual(fk.call_count,1)
        self.assertEqual(calls,['geometry','contacts'])
        q=base.initial.copy();q[3]=.6
        self.assertFalse(planner.valid_full(q));self.assertEqual(planner.stats['last_posture_rejection']['rule'],'test_posture')
        q=base.initial.copy();q[2]=0.
        self.assertFalse(planner.valid_full(q));self.assertIn('last_collision',planner.stats)

    def test_level_entry_rejects_pitch_and_roll_independently(self):
        from scipy.spatial.transform import Rotation
        level=Rotation.from_euler('z',90,degrees=True).as_matrix()
        self.assertLess(level_entry_error(level),1e-6)
        for axis in ('x','y'):
            tilted=level@Rotation.from_euler(axis,14,degrees=True).as_matrix()
            self.assertGreater(level_entry_error(tilted),LEVEL_ENTRY_TOLERANCE_DEG)

    def test_stationary_reacquisition_uses_actual_current_view_without_another_motion(self):
        from camera_patrol import CameraPatrol
        from run_newton_side_camera_patrol import reuse_stationary_scan
        inventory=dict(trusses=[dict(plant_id=17,truss_index=0,gutter_center_x=-.775,
            root_position_world_m=[-.68,0.,.3],fruit_centroid_world_m=[-.6,1.,.5],rail_near=True)])
        patrol=CameraPatrol(inventory,-.775)
        optical=np.array([[0.,0.,-1.,0.],[1.,0.,0.,1.],[0.,-1.,0.,1.],[0.,0.,0.,1.]])
        visible=dict(objects=[dict(key=f'/World/Greenhouse/Plants/Plant_17/Truss_00/Fruit_{i:02d}',
            class_name='fruit',instance_id=i,red_proxy=True,visible_pixels=40,valid_depth_pixels=35,
            visible_surface_median_world_m=[-.6,1.,.5])for i in (1,2)],
            world_from_camera_optical=optical.tolist(),timestamp_s=1.,current_timestamp_s=1.,physics_state_authoritative=True)
        patrol.observe(visible,current_station_y_m=1.)
        empty=dict(visible,objects=[]);patrol.observe(empty,current_station_y_m=1.)
        decision=patrol.decide(current_station_y_m=1.);q=np.zeros(10);q[0]=1.;home=q.copy()
        other=q.copy();other[2]+=.1
        self.assertIsNone(reuse_stationary_scan(patrol,decision,empty,other,home))
        self.assertIsNone(reuse_stationary_scan(patrol,decision,dict(empty,physics_state_authoritative=False),q,home))
        result=reuse_stationary_scan(patrol,decision,empty,q,home)
        self.assertEqual(result['status'],'forward_view_retry_pending')
        self.assertEqual(patrol.decide(current_station_y_m=1.)['action'],'move')
        self.assertFalse(patrol.native_detached_keys)

    def test_responsive_timing_retains_two_full_jaw_strokes(self):
        from side_farm_motion import jaw_cycle_points
        from newton_patrol_motion import motion_duration
        q=np.zeros(10);q[8:]=[-.026,.026]
        rows=jaw_cycle_points(q,cycles=2)
        closes=[points for name,points,_ in rows if name.startswith('grip_close')]
        self.assertEqual(len(closes),2)
        for points in closes:
            self.assertLess(responsive_duration(points,.45),motion_duration(points,minimum=.55))
            self.assertAlmostEqual(.044+points[-1,9]-points[-1,8],.036)

    def test_exact_planning_reuse_preserves_native_collision_and_measured_checks(self):
        from test_newton_ompl import scene
        from side_camera_motion import LocalCheckedPlanner
        base=scene()
        obstacles=SimpleNamespace(update=lambda _:None,aliases={})
        planner=LocalCheckedPlanner(base.model,dict(qpos=base.initial),obstacles,model_sha256='test')
        a=base.initial.copy();b=a.copy();b[2]+=.02
        planner.begin_local_checks()
        self.assertTrue(planner.edge(a,b,full=True))
        checks=planner.stats['state_checks']
        self.assertTrue(planner.edge(b,a,full=True))
        self.assertEqual(planner.stats['state_checks'],checks)
        np.testing.assert_array_equal(planner.data.qpos,a)
        adjacent=a.copy();adjacent[2]=np.nextafter(adjacent[2],1.)
        self.assertTrue(planner.valid_full(adjacent))
        self.assertGreater(planner.stats['state_checks'],checks)
        collision=a.copy();collision[2]=0.
        self.assertFalse(planner.valid_full(collision));checks=planner.stats['state_checks']
        self.assertFalse(planner.valid_full(collision));self.assertGreater(planner.stats['state_checks'],checks)
        planner.end_local_checks();checks=planner.stats['state_checks']
        self.assertTrue(planner.valid_full(a,measured=True))
        self.assertGreater(planner.stats['state_checks'],checks)

    def test_hold_goal_rejects_small_contact_that_would_block_actual_servo_arrival(self):
        from test_newton_ompl import scene
        from side_camera_motion import LocalCheckedPlanner
        base=scene();obstacles=SimpleNamespace(update=lambda _:None,aliases={})
        planner=LocalCheckedPlanner(base.model,dict(qpos=base.initial),obstacles,model_sha256='test')
        motion=object.__new__(SideCameraMotion);motion.planner=planner
        q=base.initial.copy();q[2]=-.2496
        self.assertTrue(planner.valid_full(q)) # Under the unchanged 1mm motion cap.
        self.assertFalse(motion.grasp_clearance(q[1:]))
        q[2]=-.252
        self.assertTrue(motion.grasp_clearance(q[1:]))

    def group(self):
        return dict(fruits=['Truss_00_Fruit_01','Truss_00_Fruit_02'],camera_region=dict(
            observation=dict(stale=False,timestamp_s=3.,observation_id='/actual/frame/observation.json'),
            observable_eligible_fruits=['Truss_00_Fruit_01','Truss_00_Fruit_02']),
            group_selection=dict(direction_world=[-1.,0.,0.],belt_aim_world_m=[-.6,.4,.9],
                tool_height_interval_m=[-.005,.079],depth_m=.079))

    def test_camera_goal_uses_only_the_fresh_visible_estimate(self):
        group=self.group();group['authored_hidden_center']=[99.,99.,99.]
        np.testing.assert_allclose(observed_capture_goal(group,3.),[-.6,.4,.9])
        group['group_selection']['belt_aim_world_m'][0]=-.55
        np.testing.assert_allclose(observed_capture_goal(group,3.),[-.55,.4,.9])
        for edit in ('old_timestamp','stale','unobserved_fruit','missing_provenance'):
            bad=copy.deepcopy(self.group())
            if edit=='old_timestamp':bad['camera_region']['observation']['timestamp_s']=2.
            elif edit=='stale':bad['camera_region']['observation']['stale']=True
            elif edit=='unobserved_fruit':bad['fruits'].append('Truss_00_Fruit_03')
            else:bad['camera_region']['observation'].pop('observation_id')
            with self.subTest(edit=edit),self.assertRaises(PlanningError):observed_capture_goal(bad,3.)

    def test_visible_fruit_body_follows_exact_name_after_native_stems_are_inserted(self):
        native=['world','fruit_a','new_fixed_stem','fruit_b']
        visual=['world','fruit_a','fruit_b','Preview_reserved_3','Preview_RB5_base']
        indices=visual_pose_indices(native,visual)
        poses=np.tile(np.eye(4),(5,1,1));poses[:,1,3]=[0.,.4,.8,1.2,1.6]
        actual=poses[indices]
        self.assertEqual(actual[visual.index('fruit_b'),1,3],1.2)
        self.assertEqual(actual[visual.index('Preview_RB5_base'),1,3],1.6)
        with self.assertRaises(ValueError):visual_pose_indices(native,['world','missing_fruit'])

    def test_measured_connector_preserves_actual_start_and_keeps_later_commands_legal(self):
        motion=object.__new__(SideCameraMotion)
        bounds=np.array([[0.,5.],[0.,.74]]+[[-3.14,3.14]]*6+[[-.026,.004],[-.004,.026]])
        calls=[]
        def valid(q,measured=False):
            margin=.002 if measured else 1e-9
            return bool(np.all(q>=bounds[:,0]-margin)and np.all(q<=bounds[:,1]+margin))
        def edge(a,b,full=False,measured=False):
            calls.append(measured)
            return all(valid(a+u*(b-a),measured)for u in np.linspace(0,1,20))
        motion.planner=SimpleNamespace(bounds=bounds,valid_full=valid,edge=edge,stats={})
        q=np.array([-.00001,.74001,0.,0.,1.,0.,1.,0.,-.0261,.0257])
        actual=q.copy();anchor=motion._anchor(q);goal=anchor.copy();goal[0]=.15
        phase=dict(name='rail_transfer',duration_s=3.,waypoint_q=[anchor[1:8].tolist(),goal[1:8].tolist()],
            rail_waypoint_positions_m=[float(anchor[0]),float(goal[0])],end_gap_m=.096,belt_local_y_m_s=[0.,0.])
        plan=dict(initial_command=anchor.tolist(),initial_gap_m=.044+anchor[9]-anchor[8],settle_seconds=.01,
            total_duration_s=3.01,phases=[phase])
        plan=motion._check(motion._prepend_connector(plan,q,anchor))
        np.testing.assert_array_equal(q,actual)
        np.testing.assert_array_equal(sample_plan(plan,0.)['command'],actual)
        self.assertEqual(calls,[True,False])
        t=plan['settle_seconds']+plan['phases'][0]['duration_s']+1e-6
        for time in np.linspace(t,plan['total_duration_s'],100):self.assertTrue(valid(sample_plan(plan,time)['command']))


if __name__=='__main__':unittest.main()
