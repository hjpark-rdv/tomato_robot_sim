import unittest
import numpy as np
import mujoco as mj
from types import SimpleNamespace
from itertools import product

from side_farm_motion import jaw_cycle_points, camera_view_evidence
from side_farm import initial_for_plan
from plan_adapter import sample_plan
from side_farm_viewer import distant_visual_plants
from preview_robot_mount import body_poses


class SideFarmMountTests(unittest.TestCase):
    def setUp(self):
        import xml.etree.ElementTree as ET
        root=ET.fromstring('<mujoco><worldbody><body name="Robot_world" pos=".1 .2 .3" euler="0 0 30">'
            '<joint name="root_joint" type="slide" axis="0 1 0"/>'
            '<geom size=".01" mass="1"/>'
            '<body name="Robot_farmily_lift_platform" pos="0 0 .6">'
            '<joint name="farmily_lift_height_joint" type="slide" axis="0 0 1"/>'
            '<geom size=".01" mass="1"/></body></body></worldbody></mujoco>')
        platform=root.find('.//body[@name="Robot_farmily_lift_platform"]');parent=platform
        for i in range(6):
            parent=ET.SubElement(parent,'body',name=f'Robot_link{i+1}',pos='0 0 .08')
            ET.SubElement(parent,'joint',name=f'arm_{i}',axis='0 1 0')
            ET.SubElement(parent,'geom',size='.01',mass='1')
        tool=ET.SubElement(parent,'body',name='Tool_fixed',pos='0 0 .04')
        for name in ('L','R'):
            jaw=ET.SubElement(tool,'body',name=f'Tool_{name}')
            ET.SubElement(jaw,'joint',name=f'jaw_{name}',type='slide',axis='1 0 0')
            ET.SubElement(jaw,'geom',size='.01',mass='1')
        self.model=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        self.initial=dict(qpos=self.model.qpos0.copy())
        d=mj.MjData(self.model);mj.mj_kinematics(self.model,d)
        n=self.model.nbody;platform_id=self.model.body('Robot_farmily_lift_platform').id
        z=float(d.xpos[platform_id,2])
        self.meta=dict(platform_id=platform_id,chassis_id=self.model.body('Robot_world').id,
            base_id=n,plate_id=n+1,motor_id=n+2,frame_id=n+3,
            arm_bodies=[i for i in range(n)if self.model.body(i).name.startswith(('Robot_link','Tool_'))],
            base_center_local_m=[0.,0.,.04],plate_zero_lift_bounds_m=[[0.,0.,z],[.2,.2,z+.02]],
            carriage_zero_lift_bounds_m=[[0.,.2,z],[.2,.3,z+.2]],
            motor_mount_relative_to_plate=np.eye(4).tolist(),source_motor_world_bounds_m=[[0.,0.,0.],[.1,.1,.1]],
            layouts=dict(side=dict(adapter_size_m=[.2,.02,.2],mount_centre_below_carriage_top_m=.1,
                motor_clearance_below_adapter_m=.01,shift_world_m=[0.,.4,0.])))
        self.axis=d.xmat[self.meta['chassis_id']].reshape(3,3)@self.model.joint('root_joint').axis

    def poses(self,rail,lift,arm):
        self.initial['qpos'][0]=rail
        return body_poses(self.model,self.initial,self.meta,lift,True,arm,layout='side')

    def test_side_arm_and_adapter_follow_rail_with_carriage(self):
        for rail in (-.1,.163,.796,3.):
            for lift in (0.,.4):
                with self.subTest(rail=rail,lift=lift):
                    arm=np.linspace(-.2,.4,6)
                    before,seat0=self.poses(0.,lift,arm);after,seat=self.poses(rail,lift,arm)
                    shift=rail*self.axis
                    # Every chassis, carriage, flange, arm and jaw must move
                    # by exactly the same world displacement, exactly once.
                    np.testing.assert_allclose(after[1:,:3,3],before[1:,:3,3]+shift,atol=1e-12)
                    np.testing.assert_allclose(after[1:,:3,:3],before[1:,:3,:3],atol=1e-12)
                    np.testing.assert_allclose(seat,seat0+shift,atol=1e-12)

    def test_base_attachment_stays_rigid_through_rail_lift_and_arm_changes(self):
        reference=None
        for rail,lift in ((0.,0.),(.163,.4),(.796,.1),(3.,.5)):
            poses,_=self.poses(rail,lift,np.full(6,rail/10.))
            relative=np.linalg.inv(poses[self.meta['platform_id']])@poses[self.meta['base_id']]
            if reference is None:reference=relative
            else:np.testing.assert_allclose(relative,reference,atol=1e-12)


class SideFarmGripTests(unittest.TestCase):
    def test_camera_start_uses_optical_frame_and_keeps_left_crop_in_view(self):
        pose=np.eye(4)
        pose[:3,:3]=np.array([[0.,0.,-1.],[1.,0.,0.],[0.,-1.,0.]])
        mount=dict(body_from_camera_optical=np.eye(4).tolist(),K=[[100.,0.,100.],[0.,100.,100.],[0.,0.,1.]],
            resolution_wh=[200,200],depth_range_m=[.05,4.])
        points=np.array([[-.4,-.02,0.],[-.4,.02,0.]])
        good=camera_view_evidence(pose,mount,points,'left')
        self.assertTrue(good['suitable']);self.assertEqual(good['projected_fruit_count'],2)
        np.testing.assert_allclose(good['target_centre_pixel'],[100.,100.])
        self.assertFalse(camera_view_evidence(pose,mount,points,'right')['suitable'])
        self.assertFalse(camera_view_evidence(pose,mount,-points,'left')['suitable'])
        # Rotating only the housing/viewport cannot certify the optical gaze.
        mount['body_from_camera_optical']=np.diag([-1.,1.,-1.,1.]).tolist()
        self.assertFalse(camera_view_evidence(pose,mount,points,'left')['suitable'])

    def test_fresh_camera_start_sets_native_state_and_matching_actuator_hold(self):
        model=mj.MjModel.from_xml_string('<mujoco><worldbody><body name="arm"><joint name="j" axis="0 0 1"/>'
            '<geom size=".01" mass="1"/><body name="tip" pos="1 0 0"/></body></worldbody>'
            '<actuator><position joint="j"/></actuator></mujoco>')
        initial=dict(qpos=np.zeros(1),qvel=np.ones(1),ctrl=np.zeros(1),robot_qpos_addresses=np.array([0]),
            robot_dof_addresses=np.array([0]),actuator_ids=np.array([0]),body_names=np.array(['arm','tip']))
        start=initial_for_plan(model,initial,dict(initial_command=[np.pi/2]))
        np.testing.assert_allclose(start['qpos'],[np.pi/2]);np.testing.assert_allclose(start['ctrl'],start['qpos'])
        np.testing.assert_allclose(start['qvel'],0.)
        np.testing.assert_allclose(start['body_positions'][1],[0.,1.,0.],atol=1e-12)
        np.testing.assert_array_equal(initial['qpos'],[0.]);np.testing.assert_array_equal(initial['ctrl'],[0.])

    def test_display_culling_retains_geometry_crossing_section_boundary(self):
        model=mj.MjModel.from_xml_string('<mujoco><worldbody/></mujoco>')
        scene=SimpleNamespace(ngeom=3,geom_dataid=np.zeros(3,int),mesh_vertadr=np.array([0]),
            mesh_vertnum=np.array([8]),mesh_vert=np.array(list(product([-.2,.2],repeat=3))),
            geom_pos=np.array([[0.,0.,0.],[0.,3.,0.],[0.,1.5,0.]]),
            geom_quat=np.tile([1.,0.,0.,0.],(3,1)),geom_bodyid=np.zeros(3,int),
            body=lambda _:SimpleNamespace(name='world'),geom=lambda i:SimpleNamespace(name=f'g{i}'))
        catalog=dict(objects={'1':dict(plant='same authored plant')},geom_to_instance={f'g{i}':1 for i in range(3)})
        self.assertEqual(distant_visual_plants(scene,model,catalog,[-.4,1.4]),[1])

    def test_two_complete_grips_stay_inserted_with_visible_holds(self):
        q=np.array([.4,.5,0.,.5,1.,-.5,1.,0.,-.026,.026])
        strokes=jaw_cycle_points(q,cycles=2)
        phases=[]
        for name,points,duration in strokes:
            np.testing.assert_array_equal(points[:,:8],np.tile(q[:8],(2,1)))
            phases.append(dict(name=name,duration_s=duration,waypoint_q=points[:,1:8].tolist(),
                rail_waypoint_positions_m=points[:,0].tolist(),end_gap_m=float(.044+points[-1,9]-points[-1,8]),belt_local_y_m_s=[0.,0.]))
        plan=dict(initial_command=q.tolist(),initial_gap_m=.096,settle_seconds=0.,phases=phases,
            total_duration_s=sum(p['duration_s']for p in phases))
        t=0.;gaps=[]
        for phase in phases:
            t+=phase['duration_s'];gaps.append(sample_plan(plan,t)['gap_m'])
        np.testing.assert_allclose(gaps,[.036,.036,.096,.096]*2,atol=1e-10)
        self.assertEqual([p['name']for p in phases].count('geometric_capture_hold'),1)
        self.assertGreaterEqual(plan['total_duration_s'],4.)
        np.testing.assert_allclose(sample_plan(plan,plan['total_duration_s'])['command'],q,atol=1e-10)

    def test_invalid_repetition_counts_are_rejected(self):
        for value in (0,5,True,1.5):
            with self.subTest(value=value),self.assertRaises(ValueError):jaw_cycle_points(np.zeros(10),cycles=value)


if __name__=='__main__':unittest.main()
