"""Native contact/break controls, independent of the greenhouse assets."""
import unittest
import numpy as np
import mujoco as mj
from run_smoke import Attachments, belt_velocity_from_physx


class NativeControls(unittest.TestCase):
    def conveyor(self, speed, yaw=0., separated=False):
        m=mj.MjModel.from_xml_string(f'''<mujoco>
          <option timestep=".002" cone="elliptic" integrator="implicitfast"/>
          <worldbody><body name="belt" euler="0 0 {yaw}">
            <geom name="surface" type="box" size="1 1 .05" surfacevel="0 {speed} 0 0 0 0" friction=".5 0 0"/>
          </body><body name="load" pos="0 0 {1 if separated else .075}"><freejoint/>
            <geom type="box" size=".02 .02 .025" mass=".05" friction=".5 0 0"/>
          </body></worldbody></mujoco>''')
        d=mj.MjData(m)
        for _ in range(100 if separated else 500):mj.mj_step(m,d)
        return d.qpos[:3]

    def test_native_belt_positive_negative_off(self):
        pos=self.conveyor(.05);neg=self.conveyor(-.05);off=self.conveyor(0)
        self.assertGreater(pos[1],.035);self.assertLess(neg[1],-.035)
        self.assertLess(abs(off[1]),1e-6)

    def test_native_belt_local_coordinate_rotation(self):
        p=self.conveyor(.05,yaw=90)
        self.assertLess(p[0],-.035);self.assertLess(abs(p[1]),1e-6)

    def test_belt_does_not_attract_without_contact(self):
        p=self.conveyor(.05,separated=True)
        self.assertLess(abs(p[1]),1e-10)

    def test_matches_measured_physx_inward_direction(self):
        # Saved PhysX control: authored +.02 m/s -> CAD Y moved -.039984m / 2s.
        speed=float(belt_velocity_from_physx(.02))
        p=self.conveyor(speed)
        self.assertLess(p[1],-.015)

    def fixture(self):
        m=mj.MjModel.from_xml_string('''<mujoco><option timestep=".002" gravity="0 0 0"/>
          <worldbody><body name="anchor"><site name="a0"/></body>
          <body name="fruit"><freejoint/><geom type="sphere" size=".012" mass=".008"/>
            <site name="a1"/></body></worldbody>
          <equality><connect name="fruit_attachment" site1="a0" site2="a1" solref=".002 1"/></equality>
        </mujoco>''')
        spec=dict(name='fruit',anchor='anchor',local_rot0=[0,0,0,1],local_rot1=[0,0,0,1],
                  stiffness=[.01,.03,.03],damping=.00015,deadband_deg=1,force_n=3.,torque_nm=.008)
        return m,mj.MjData(m),Attachments(m,[spec])

    def test_measured_force_break_and_below_threshold(self):
        for applied,expected in [(1.,False),(4.,True)]:
            m,d,a=self.fixture();events=[]
            for _ in range(100):
                d.xfrc_applied[:]=0;d.xfrc_applied[a.ids[0],0]=applied
                mj.mj_step(m,d)
                forces,e=a.measure_breaks(d,np.array([0.]),'force_control');events+=e
            self.assertEqual(bool(events),expected)
            if expected:self.assertGreater(events[0]['constraint_force_n'],3.)

    def test_torque_break_and_no_spontaneous_release(self):
        m,d,a=self.fixture();mj.mj_forward(m,d)
        torque,_=a.apply(d);mj.mj_step(m,d)
        _,events=a.measure_breaks(d,torque,'idle');self.assertFalse(events)
        # Known relative orientation, above the unchanged .008 Nm threshold.
        d.qpos[3:7]=[np.cos(.6/2),0,np.sin(.6/2),0]
        mj.mj_forward(m,d);d.xfrc_applied[:]=0
        torque,_=a.apply(d)
        _,events=a.measure_breaks(d,torque,'torque_control')
        self.assertEqual(events[0]['cause'],'torque')

if __name__=='__main__':unittest.main()
