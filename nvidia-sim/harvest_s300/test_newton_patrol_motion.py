import unittest
import numpy as np
from plan_adapter import sample_plan
from newton_patrol_motion import fluid_scan,fluid_plan,rail_duration,cruise_route,VELOCITY,ACCELERATION
from run_newton_camera_patrol import scan_plan


class FluidMotionTests(unittest.TestCase):
    def test_nonuniform_route_tail_brakes_within_bounds_without_station_jump(self):
        # Recorded failure: .8482915759m start left only1.708mm after20 steps.
        for start,end,step in ((.8482915759086609,3.85,.15),(0.,.301,.15),
                               (0.,.30001,.15),(0.,.085,.01),(0.,.0001,.15)):
            with self.subTest(start=start,end=end,step=step):
                q=np.array([start,.18,0.,1.,-2.,0.,1.,0.,-.026,.026])
                plan=cruise_route(q,end,step_m=step)
                previous_station=start;previous_speed=0.
                for row in plan['phases']:
                    x0,x1=row['rail_waypoint_positions_m']
                    self.assertAlmostEqual(x0,previous_station,places=12)
                    d=x1-x0;t=row['duration_s']
                    self.assertGreaterEqual(d,0.)
                    self.assertLessEqual(d,step+1e-12)
                    v0,v1=row.get('rail_boundary_velocities_m_s',[0.,0.])
                    self.assertAlmostEqual(v0,previous_speed,places=12)
                    a=-2*d+t*(v0+v1);b=3*d-t*(2*v0+v1);c=t*v0
                    u=np.linspace(0.,1.,1001)
                    v=(3*a*u*u+2*b*u+c)/t
                    acc=(6*a*u+2*b)/t**2
                    self.assertGreaterEqual(v.min(),-1e-8)
                    self.assertLessEqual(v.max(),VELOCITY[0]+1e-5)
                    self.assertLessEqual(np.abs(acc).max(),ACCELERATION[0]+1e-9)
                    previous_station=x1;previous_speed=v1
                self.assertEqual(previous_station,end)
                self.assertEqual(previous_speed,0.)
                np.testing.assert_array_equal(sample_plan(plan,0)['command'],q)
                self.assertAlmostEqual(sample_plan(plan,plan['total_duration_s'])['command'][0],end)

    def test_cruise_keeps_forward_velocity_across_observation_and_brakes_monotonically(self):
        q=np.array([0.,.18,0.,1.,-2.,0.,1.,0.,-.026,.026])
        for v0,v1,distance in ((0.,.12,.15),(.12,.12,.15),(.12,0.,.15),(.006,0.,.03)):
            duration=rail_duration(distance,v0,v1)
            base=scan_plan(q,destination=distance)
            plan=fluid_scan(base,rail_velocity=v0,cruise=v1>0)
            row=plan['phases'][0]
            row['duration_s']=duration
            row['rail_boundary_velocities_m_s']=[v0,v1]
            dt=duration/1000
            positions=np.array([sample_plan(plan,plan['settle_seconds']+t)['command'][0]
                                for t in np.linspace(0,duration-dt,1000)])
            velocity=np.diff(positions)/dt
            self.assertGreaterEqual(velocity.min(),-1e-7)
            self.assertLessEqual(velocity.max(),VELOCITY[0]+1e-4)
            self.assertLessEqual(np.max(np.abs(np.diff(velocity)/dt)),ACCELERATION[0]+1e-3)
            self.assertAlmostEqual(velocity[0],v0,delta=.001)
            self.assertAlmostEqual(velocity[-1],v1,delta=.001)
            np.testing.assert_array_equal(sample_plan(plan,0)['command'],q)
            self.assertEqual(len(plan['phases']),1 if v1 else 2)

    def test_fluid_harvest_keeps_path_open_and_measured_initial_asymmetry(self):
        q=np.array([0.,.18,0.,1.,-2.,0.,1.,0.,.0001,.0002])
        pose=q[1:8];goal=pose+np.array([.02,0.,.1,-.1,0.,0.,0.])
        names=['ready_hold','prepare_open','prepare_open_hold','prepare_joint_branch','approach',
               'geometric_capture_hold','withdraw','return_joint_branch','ready_close','hold']
        paths=[[pose,pose]]*3+[[pose,goal],[goal,goal],[goal,goal],[goal,goal],[goal,pose],[pose,pose],[pose,pose]]
        plan=dict(harvest_mode='main_stem_only_geometric_harvest',initial_command=q.tolist(),
            initial_gap_m=.044+q[9]-q[8],total_duration_s=25.,settle_seconds=.05,
            phases=[dict(name=name,duration_s=2.,waypoint_q=np.array(path).tolist(),
                         rail_waypoint_positions_m=[0.,0.],end_gap_m=.096,belt_local_y_m_s=[0.,0.])
                    for name,path in zip(names,paths)])
        result=fluid_plan(plan)
        np.testing.assert_array_equal(sample_plan(result,0)['command'],q)
        self.assertLess(result['total_duration_s'],plan['total_duration_s'])
        self.assertNotIn('ready_close',[r['name'] for r in result['phases']])
        for row in result['phases']:
            original=next(r for r in plan['phases'] if r['name']==row['name'])
            self.assertEqual(row['waypoint_q'],original['waypoint_q'])
            self.assertEqual(row['end_gap_m'],.096)
        sample=sample_plan(result,result['total_duration_s'])
        self.assertAlmostEqual(sample['gap_m'],.096)


if __name__=='__main__':unittest.main()
