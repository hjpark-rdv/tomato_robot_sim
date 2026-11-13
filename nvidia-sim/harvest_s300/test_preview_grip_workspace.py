"""Fail-closed obstacle activation and bilateral preview playback checks."""
import unittest
import mujoco as mj
import numpy as np

from preview_grip_workspace import (
    BoxObstacles, StorageGripMotion, nearest_joint_turns, segment_duration,
)
from newton_patrol_motion import VELOCITY


class GripPreviewTests(unittest.TestCase):
    def model(self, chassis=False):
        body='Robot_world' if chassis else 'Robot_link_probe'
        model=mj.MjModel.from_xml_string(f'''<mujoco><worldbody>
          <body name="spare" pos="-3 0 0"><geom name="slot" type="sphere"
            size=".01" contype="1" conaffinity="3"/></body>
          <body name="{body}"><joint type="slide" axis="1 0 0"/>
            <geom name="probe" type="sphere" size=".03" pos="1 0 0"
              contype="2" conaffinity="3"/></body>
        </worldbody></mujoco>''')
        # Preserve a compiled BVH while disabling the hidden source obstacle.
        model.geom_contype[0]=model.geom_conaffinity[0]=0
        return model

    def contacts(self,model):
        data=mj.MjData(model)
        mj.mj_kinematics(model,data);mj.mj_collision(model,data)
        return [float(c.dist)for c in data.contact]

    def test_new_box_updates_body_masks_and_compiled_bounds(self):
        model=self.model()
        BoxObstacles(model,np.zeros(1),[dict(name='bin',position=[1,0,0],dimensions=[.1]*3)])
        self.assertLess(min(self.contacts(model)),-.001)
        self.assertEqual(model.body_conaffinity[model.body('spare').id],4)

    def test_chassis_attachment_is_not_rejected_as_a_moving_arm_contact(self):
        model=self.model(chassis=True)
        BoxObstacles(model,np.zeros(1),[dict(name='bin',position=[1,0,0],dimensions=[.1]*3)])
        self.assertEqual(self.contacts(model),[])

    def test_motor_obstacle_follows_lift_and_refits_its_bvh(self):
        model=self.model()
        obstacles=BoxObstacles(model,np.zeros(1),[dict(name='motor',position=[1,0,.4],
            dimensions=[.1]*3,follows_lift=True,reference_lift_m=.4)])
        self.assertEqual(self.contacts(model),[])
        obstacles.update(0.)
        self.assertLess(min(self.contacts(model)),-.001)
        obstacles.update(1.)
        self.assertEqual(self.contacts(model),[])

    def test_equivalent_hinge_turns_obey_actual_limits(self):
        state=np.r_[.75,[-4.4,.3,-2.4,.5,1.57,-4.4],[-.026,.026]]
        reference=state.copy();reference[6]=1.8
        bounds=np.array([[0,5],[0,.75]]+[[-2*np.pi,2*np.pi]]*6+[[-.026,.008],[-.008,.026]])
        bounds[4]=[-2.88,2.88]
        result=nearest_joint_turns(state,reference,bounds)
        np.testing.assert_allclose(np.exp(1j*result[1:7]),np.exp(1j*state[1:7]),atol=1e-12)
        self.assertLess(abs(result[6]-reference[6]),.1)
        self.assertTrue(np.all(result[1:7]>=bounds[2:8,0]))
        self.assertTrue(np.all(result[1:7]<=bounds[2:8,1]))

    def test_nine_axis_playback_preserves_jaws_and_reverse_labels(self):
        a=np.r_[.75,np.zeros(6),[-.026,.026]]
        b=a.copy();b[0]=.45;b[1:7]=.2;b[7:]=[.004,-.004]
        points=np.array([a,b]);span=segment_duration(points)
        common=dict(start_capture_m=[0]*3,target_capture_m=[1]*3,side='left')
        motion=StorageGripMotion([dict(common,points=np.array([a,a]),duration_s=1.,phase='open'),
            dict(common,points=points,duration_s=span,phase='close')],{})
        self.assertEqual(motion.sample(1+span*.5)[1]['phase'],'close')
        self.assertEqual(motion.sample(1+span*1.5)[1]['phase'],'open')
        times=np.linspace(1,1+span,1001)
        states=np.array([motion.sample(t)[0]for t in times])
        velocity=np.diff(states,axis=0)/np.diff(times)[:,None]
        self.assertTrue(np.all(np.max(np.abs(velocity),axis=0)<=VELOCITY[1:10]))
        for t in (1.,1.+span,1.+2*span):
            np.testing.assert_allclose(motion.sample(t-1e-6)[0],motion.sample(t+1e-6)[0],atol=1e-9)


if __name__=='__main__':unittest.main()
