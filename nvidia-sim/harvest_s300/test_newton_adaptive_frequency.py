import unittest
from unittest.mock import patch
import numpy as np
import warp as wp
from types import SimpleNamespace

from newton_adaptive_frequency import AdaptiveSchedule


def timeline(*args,**kwargs):
    rate = args[2]
    n = rate
    t = np.arange(n)/rate
    return dict(time=t,time_edges=np.arange(n+1)/rate,dt=np.full(n,1/rate),
        source_sample_time_s=t,source_command_record_time_s=t,command=t[:,None],
        phase=np.full(n,'approach'),authored_belt_speed=np.zeros((n,2)),
        belt_surface_velocity_local_y_m_s=np.zeros((n,2)),dof_names=np.array(['q'])),{},None


class AdaptiveTests(unittest.TestCase):
    @patch('newton_adaptive_frequency.command_timeline',timeline)
    def test_integer_clock_and_contact_hold(self):
        a = AdaptiveSchedule({},[0],400,60,hold_seconds=.1)
        self.assertEqual(a.prepare(0,10,False),1)
        self.assertEqual(a.decisions[-1]['hz'],60)
        self.assertEqual(a.prepare(1,10,True),10)
        self.assertEqual(a.decisions[-1]['hz'],400)
        total = 11
        while not a.complete:
            n = a.prepare(total,10,False)
            total += n
        out = a.finish(total)
        self.assertTrue(a.complete)
        self.assertTrue(np.all(np.diff(out['time_edges'])>0))
        np.testing.assert_allclose(out['command'][:,0],out['time'],rtol=0,atol=1e-12)
        self.assertLess(out['time_edges'][-1]-1,1/60+1e-12)
        self.assertEqual(len(out['phase'])+1,len(out['time_edges']))
        np.testing.assert_array_equal(out['dt'],np.diff(out['time_edges']))

    def test_rates(self):
        with self.assertRaises(ValueError):
            AdaptiveSchedule({},[0],60,400)

    @patch('newton_adaptive_frequency.command_timeline',timeline)
    def test_upload_updates_only_current_rows(self):
        a = AdaptiveSchedule({},[0],400,60)
        a.prepare(0,10,False)
        count = a.prepare(1,10,True)
        controller = SimpleNamespace(device='cpu',commands=wp.zeros((500,1),float,device='cpu'),
            belts=wp.zeros((500,2),float,device='cpu'),contact_policy=None)
        a.upload(controller,1,count)
        actual = controller.commands.numpy()[:,0]
        np.testing.assert_allclose(actual[1:1+count],a.timeline['command'][1:1+count,0],rtol=1e-7)
        self.assertEqual(actual[0],0)
        self.assertTrue(np.all(actual[1+count:]==0))


if __name__ == '__main__':
    unittest.main()
