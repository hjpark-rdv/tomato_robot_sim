import unittest
import numpy as np
from newton_ros_commands import Commands,Segment

NAMES=['root_joint','farmily_lift_height_joint','base','shoulder','elbow','wrist1','wrist2','wrist3','jaw_L','jaw_R']
BOUNDS=np.array([[0,3.85],[0,.75]]+[[-6.28,6.28]]*6+[[-.026,.008],[-.008,.026]])

class CommandTests(unittest.TestCase):
    def setUp(self):self.c=Commands(NAMES,np.array([0,.18]+[0.]*8),BOUNDS)
    def command(self,kind,**kwargs):
        return dict(sequence=self.c.last_sequence+1,command_type=kind,received_monotonic=10.,**kwargs)
    def path(self,name,value,duration=2.,**point):
        return self.command('joint_trajectory',joint_names=[name],start_time=0.,points=[dict(
            positions=[value],velocities=[],accelerations=[],effort=[],time_from_start=duration,**point)])
    def test_names_determine_axes_and_gpu_target_is_smooth(self):
        self.assertTrue(self.c.accept(self.path('wrist3',.2),0.,10.))
        for t in np.linspace(.01,2.,200):self.c.advance(t,10.,.01)
        self.assertAlmostEqual(self.c.q[7],.2);self.assertEqual(self.c.q[2],0.)
        self.c.observe(2.1,self.c.q,np.zeros(10));self.assertEqual(self.c.recent[-1]['status'],'completed')
    def test_limit_overshoot_and_unknown_joints_reject_atomically(self):
        for name,value,duration in [('bad',.1,2),('elbow',7.,4),('wrist1',1.,.1),('jaw_L',-.1,2)]:
            before=self.c.q.copy();self.assertFalse(self.c.accept(self.path(name,value,duration),0,10))
            np.testing.assert_array_equal(self.c.q,before);self.assertFalse(self.c.plans)
        seg=Segment(0,1,0,0,5,-5)
        with self.assertRaisesRegex(ValueError,'position'):seg.validate([-.1,.1],10,100)
    def test_timed_trajectory_derivatives_and_bad_stamp(self):
        command=self.path('wrist1',.2)
        command['points'][0].update(velocities=[0.],accelerations=[0.])
        self.assertTrue(self.c.accept(command,0,10))
        for seg in self.c.plans[5]:seg.validate(BOUNDS[5],1,2)
        c=self.path('wrist2',.1);c['start_time']=1.
        self.assertFalse(self.c.accept(c,2.,10.))
    def test_watchdog_decelerates_base_and_stop_cancels_trajectory(self):
        c=self.command('cmd_vel',linear=[.1,0,0],angular=[0,0,0]);self.assertTrue(self.c.accept(c,0,10))
        for k in range(60):self.c.advance((k+1)*.01,10+k*.01,.01)
        speed=self.c.v[0];self.assertLess(speed,.1);self.assertGreater(speed,0.)
        for k in range(50):self.c.advance(.61+k*.01,10.61+k*.01,.01)
        self.assertAlmostEqual(self.c.v[0],0.);self.assertLess(self.c.q[0],.06)
        self.assertTrue(self.c.accept(self.path('wrist1',.2),1.2,10.))
        self.c.advance(1.4,10.,.01);self.assertGreater(self.c.v[5],0)
        self.c.accept(self.command('stop',value=True),1.4,10.)
        self.assertFalse(self.c.plans);self.assertFalse(self.c.accept(self.path('wrist1',.3),1.5,10.))
        for k in range(100):self.c.advance(1.5+k*.01,10.,.01)
        self.assertAlmostEqual(self.c.v[5],0.)
        self.c.accept(self.command('stop',value=False),3.,10.)
        self.assertFalse(self.c.stopped);self.assertFalse(self.c.plans)
    def test_gripper_gap_and_lift_concurrent(self):
        self.assertTrue(self.c.accept(self.command('gripper',value=.096),0,10))
        self.assertTrue(self.c.accept(self.command('lift',value=.22),0,10))
        for t in np.linspace(.01,3,300):self.c.advance(t,10,.01)
        self.assertAlmostEqual(self.c.q[1],.22);self.assertAlmostEqual(.044+self.c.q[9]-self.c.q[8],.096)
    def test_rail_rejects_turning_and_brakes_at_boundary(self):
        self.assertFalse(self.c.accept(self.command('cmd_vel',linear=[0,0,0],angular=[0,0,.1]),0,10))
        self.c.q[0]=3.80
        for k in range(200):
            c=self.command('cmd_vel',linear=[.12,0,0],angular=[0,0,0]);self.c.accept(c,k*.01,10.)
            self.c.advance((k+1)*.01,10.,.01)
            self.assertLessEqual(self.c.q[0],3.85)
        self.assertLess(abs(self.c.v[0]),.001)
    def test_nan_duplicate_timing_and_nonzero_terminal_speed_rejected(self):
        c=self.path('wrist1',float('nan'));self.assertFalse(self.c.accept(c,0,10))
        c=self.path('wrist1',.1);c['points'][0]['velocities']=[.1]
        self.assertFalse(self.c.accept(c,0,10))
        c=self.path('wrist1',.1);c['points']*=2
        self.assertFalse(self.c.accept(c,0,10))

    def test_invalid_bounds_and_initial_positions_rejected(self):
        for upper in (float('nan'),float('inf'),-1.,0.):
            bounds=BOUNDS.copy();bounds[0,1]=upper
            with self.assertRaisesRegex(ValueError,'bounds'):Commands(NAMES,self.c.q,bounds)
        q=self.c.q.copy();q[0]=4.
        with self.assertRaisesRegex(ValueError,'positions'):Commands(NAMES,q,BOUNDS)

    def test_zero_time_acceleration_must_match(self):
        c=self.path('wrist1',.1)
        c['points'][0].update(velocities=[0.],accelerations=[0.])
        c['points'].insert(0,dict(positions=[0.],velocities=[0.],accelerations=[999.],time_from_start=0.))
        self.assertFalse(self.c.accept(c,0,10))
        c['sequence']+=1;c['points'][0]['accelerations']=[0.]
        self.assertTrue(self.c.accept(c,0,10))

    def planned(self,request_id='plan-1'):
        self.c.model_sha256='a'*64
        c=self.path('wrist3',.2)
        c.update(command_type='planned_arm_trajectory',request_id=request_id,
                 source_model_sha256=self.c.model_sha256,expected_start_positions=self.c.q.tolist())
        return c

    def test_planned_command_rejects_changed_model_or_any_stale_axis_atomically(self):
        c=self.planned();c['source_model_sha256']='b'*64
        self.assertFalse(self.c.accept(c,0,10));self.assertIn('model SHA',self.c.recent[-1]['reason'])
        c=self.planned();self.c.model_sha256=None
        self.assertFalse(self.c.accept(c,0,10))
        for axis in range(10):
            c=self.planned();c['expected_start_positions'][axis]+=.00001
            before=self.c.q.copy()
            self.assertFalse(self.c.accept(c,0,10));self.assertIn('stale',self.c.recent[-1]['reason'])
            np.testing.assert_array_equal(self.c.q,before);self.assertFalse(self.c.plans)
            self.assertEqual(self.c.recent[-1]['request_id'],'plan-1')

    def test_planned_command_requires_measured_rest_and_tracking_on_every_axis(self):
        for axis in range(10):
            velocity=np.zeros(10);velocity[axis]=.021
            self.c.observe(0,self.c.q,velocity)
            self.assertFalse(self.c.accept(self.planned(),0,10))
            self.assertIn('stationary',self.c.recent[-1]['reason'])
            measured=self.c.q.copy();measured[axis]+=self.c.tolerance[axis]*1.01
            self.c.observe(0,measured,np.zeros(10))
            self.assertFalse(self.c.accept(self.planned(),0,10))
            self.assertIn('tracking tolerance',self.c.recent[-1]['reason'])
        self.c.observe(0,self.c.q,np.zeros(10))
        self.c.v[0]=.001
        self.assertFalse(self.c.accept(self.planned(),0,10))
        self.c.v[0]=0.;self.c.a[0]=.01
        self.assertFalse(self.c.accept(self.planned(),0,10))

    def test_planned_command_freezes_unplanned_axes_until_correlated_completion(self):
        c=self.planned();self.assertTrue(self.c.accept(c,0,10))
        feedback=self.c.feedback()
        self.assertTrue(feedback['motion_active']);self.assertEqual(feedback['active_planning_request_id'],'plan-1')
        for other in (self.command('lift',value=.22),self.command('gripper',value=.096),
                      self.command('cmd_vel',linear=[.1,0,0],angular=[0,0,0]),self.path('wrist1',.1),
                      self.planned('plan-2')):
            other['sequence']=self.c.last_sequence+1
            self.assertFalse(self.c.accept(other,0,10));self.assertIn('owns all motion',self.c.recent[-1]['reason'])
        self.assertTrue(self.c.accept(self.command('stop',value=False),0,10))
        self.assertEqual(self.c.feedback()['active_planning_request_id'],'plan-1')
        for t in np.linspace(.01,2.,200):self.c.advance(t,10,.01)
        np.testing.assert_array_equal(self.c.q[[0,1,8,9]],[0,.18,0,0])
        self.c.observe(2.1,self.c.q,np.zeros(10))
        self.assertEqual(self.c.recent[-1]['status'],'completed')
        self.assertEqual(self.c.recent[-1]['request_id'],'plan-1')
        self.assertIsNone(self.c.feedback()['active_planning_request_id'])
        self.assertFalse(self.c.feedback()['motion_active'])
        self.assertTrue(self.c.accept(self.command('lift',value=.22),2.1,10))

    def test_stop_cancels_planned_lock_without_resuming_old_motion(self):
        self.assertTrue(self.c.accept(self.planned(),0,10))
        self.c.advance(.5,10,.01)
        self.assertTrue(self.c.accept(self.command('stop',value=True),.5,10))
        event=self.c.recent[-2]
        self.assertEqual((event['status'],event['request_id']),('preempted','plan-1'))
        self.assertIsNone(self.c.feedback()['active_planning_request_id'])
        self.assertTrue(self.c.accept(self.command('stop',value=False),.5,10))
        self.assertFalse(self.c.plans)
        self.assertFalse(self.c.accept(self.planned('plan-2'),.5,10))
        for t in np.linspace(.51,2.,150):self.c.advance(t,10,.01)
        self.c.observe(2,self.c.q,np.zeros(10))
        self.assertTrue(self.c.accept(self.planned('plan-3'),2,10))

    def test_planned_tracking_failure_and_rejection_are_correlated(self):
        self.assertTrue(self.c.accept(self.planned(),0,10))
        self.c.observe(5.1,self.c.q,np.zeros(10))
        failed=[e for e in self.c.recent if e['status']=='tracking_failed']
        self.assertEqual(failed[0]['request_id'],'plan-1');self.assertTrue(self.c.stopped)
        self.assertIsNone(self.c.feedback()['active_planning_request_id'])

    def test_planned_command_rejects_active_target_before_it_has_started_moving(self):
        self.assertTrue(self.c.accept(self.path('wrist1',.1),0,10))
        self.assertFalse(self.c.accept(self.planned(),0,10))
        self.assertIn('active motion',self.c.recent[-1]['reason'])
        self.assertEqual(set(self.c.plans),{5})

if __name__=='__main__':unittest.main()
