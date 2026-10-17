import unittest
import numpy as np

from newton_independent_evidence import complete_schedule


class CompleteScheduleTests(unittest.TestCase):
    def test_full_schedule_requires_actual_clock_and_terminal_coverage(self):
        edge=np.array([0.,1/60,1/60+1/400])
        c=dict(phase=np.array(['approach','close']),dt=np.diff(edge),time_edges=edge)
        s=dict(time=edge,scheduled_time=edge,phase=np.array(['initial','approach','close']))
        report=dict(status='completed',stop=None,actual_steps=2,requested_steps=2,
                    finite_recorded_state=True,qacc_finite=True,solver_capacity_valid=True)
        self.assertTrue(complete_schedule(report,c,s))
        self.assertFalse(complete_schedule(dict(report,stop='nonfinite_physics'),c,s))
        self.assertFalse(complete_schedule(report,c,dict(s,scheduled_time=edge+.001)))
        self.assertFalse(complete_schedule(report,c,dict(s,phase=np.array(['initial','approach','approach']))))

    def test_cpu_reference_uses_native_float64_clock_without_relaxing_gpu_schedule(self):
        edge=np.array([0.,.01,.02])
        commands=dict(phase=np.array(['close','hold']),dt=np.diff(edge),time_edges=edge)
        state=dict(time=edge+np.array([0.,1e-12,1e-12]),phase=np.array(['initial','close','hold']))
        report=dict(status='completed',stop=None,actual_steps=2,requested_steps=2,
                    finite_recorded_state=True,qacc_finite=True,solver_capacity_valid=True,backend='cpu')
        self.assertTrue(complete_schedule(report,commands,state))
        self.assertFalse(complete_schedule(dict(report,backend='gpu'),commands,state))
        self.assertFalse(complete_schedule(report,commands,dict(state,time=edge+.000001)))


if __name__=='__main__':
    unittest.main()
