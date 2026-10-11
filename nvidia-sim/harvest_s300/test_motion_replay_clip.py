import unittest
import numpy as np

from motion_replay_clip import select_replay_ticks


class ReplayClipTests(unittest.TestCase):
    def test_default_preserves_original_selection(self):
        stamps=np.arange(0,3.01,.1)
        original=np.minimum(np.searchsorted(stamps,np.arange(stamps[0],stamps[-1]+1e-8,2/8)),len(stamps)-1)
        if original[-1]!=len(stamps)-1:original=np.r_[original,len(stamps)-1]
        np.testing.assert_array_equal(select_replay_ticks(stamps,fps=8,speed=2),original)

    def test_clip_uses_only_recorded_states_inside_bounds(self):
        stamps=np.array([0.,.1,.23,.4,.7,1.])
        ticks=select_replay_ticks(stamps,fps=2,speed=1,start_time=.12,end_time=.75)
        np.testing.assert_array_equal(ticks,[2,4])
        self.assertTrue(np.all(stamps[ticks]>=.12));self.assertTrue(np.all(stamps[ticks]<=.75))

    def test_short_clip_repeat_is_recorded_final_pose(self):
        ticks=select_replay_ticks([0.,1.,2.],fps=8,speed=2,start_time=1.,end_time=1.)
        np.testing.assert_array_equal(ticks,np.ones(8,dtype=int))

    def test_invalid_clip_and_time_trace_rejected(self):
        for stamps,options in [([0,0],{}),([0,np.nan],{}),([0,1],dict(start_time=-1)),
                ([0,1],dict(end_time=2)),([0,1],dict(start_time=.4,end_time=.6)),
                ([0,1],dict(start_time=1,end_time=0)),([0,1],dict(speed=0))]:
            kwargs=dict(fps=8,speed=2);kwargs.update(options)
            with self.assertRaises(ValueError):select_replay_ticks(stamps,**kwargs)


if __name__=='__main__':unittest.main()
