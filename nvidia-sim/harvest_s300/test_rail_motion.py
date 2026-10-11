import unittest
from types import SimpleNamespace
import numpy as np
from rail_motion import rail_destination,station_kinematics,attach_rail_commands,rail_command


class RailMotionTest(unittest.TestCase):
    def setUp(self):
        self.manifest={'rail':{'initial_station_y_m':1.6,'limits_relative_m':[-1.6,3.4],
            'dof_name':'root_joint','axis_world':[0,1,0]}}

    def test_speed_limits_and_endpoints(self):
        d=rail_destination(self.manifest,2.,.08)
        self.assertAlmostEqual(d['duration_s'],7.5)
        self.assertAlmostEqual(1.5*d['displacement_m']/d['duration_s'],.08)
        for bad in (-.001,5.001,float('nan')):
            with self.assertRaises(ValueError):rail_destination(self.manifest,bad)

    def test_world_frame_is_translated_without_mutating_source(self):
        kin=SimpleNamespace(world=np.eye(4));d=rail_destination(self.manifest,2.)
        translated=station_kinematics(kin,d)
        np.testing.assert_array_equal(kin.world,np.eye(4))
        np.testing.assert_allclose(translated.world[:3,3],[0,.4,0])

    def test_physical_move_precedes_manipulation_and_shifts_belt_clock(self):
        q=np.zeros(7);phases=[('approach',4.,np.array([q,q]),.096,0.,0.)]
        meta={'phases':[dict(name='approach',duration_s=4.,waypoint_q=[q.tolist(),q.tolist()])],
              'belt_cycle':{'start_after_settle_s':4.}}
        p,m=attach_rail_commands(phases,meta,q,rail_destination(self.manifest,2.),move_first=True)
        self.assertEqual(p[0][0],'rail_move')
        self.assertAlmostEqual(m['belt_cycle']['start_after_settle_s'],11.5)
        self.assertAlmostEqual(rail_command(m['phases'][0],0.),0.)
        self.assertAlmostEqual(rail_command(m['phases'][0],1.),.4)
        self.assertAlmostEqual(rail_command(m['phases'][1],.3),.4)
        self.assertEqual(len(meta['phases']),1)


if __name__=='__main__':unittest.main()
