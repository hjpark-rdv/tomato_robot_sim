import unittest
import numpy as np
import warp as wp

from newton_kernels import save_all_world_trace


class AllWorldTraceTests(unittest.TestCase):
    def test_distinct_worlds_and_ticks_are_retained(self):
        with wp.ScopedDevice('cpu'):
            tick=wp.array([0],int)
            q=wp.array([[1.,2.,3.],[10.,20.,30.]],float)
            v=wp.array([[4.,5.],[40.,50.]],float)
            eq=wp.array([[True],[False]],bool)
            qlog=wp.zeros((2,3,3),float);vlog=wp.zeros((2,3,2),float);elog=wp.zeros((2,3,1),bool)
            def record():
                wp.launch(save_all_world_trace,(2,3),inputs=[tick,q,v,eq,qlog,vlog,elog])
            record();tick.assign(np.array([2],np.int32));q.assign(np.array([[7,8,9],[70,80,90]],np.float32));record()
            np.testing.assert_array_equal(qlog.numpy()[:,0],[[1,2,3],[10,20,30]])
            np.testing.assert_array_equal(qlog.numpy()[:,1],0)
            np.testing.assert_array_equal(qlog.numpy()[:,2],[[7,8,9],[70,80,90]])
            np.testing.assert_array_equal(vlog.numpy()[:,2],[[4,5],[40,50]])
            np.testing.assert_array_equal(elog.numpy()[:,2],[[True],[False]])


if __name__=='__main__':unittest.main()
