import numpy as np
from gpu_batch_views import BatchReadCache, BatchSliceView


def test_shared_read_and_reset_invalidation():
    class View:
        def __init__(self):
            self.values = np.array([[1., 2.], [3., 4.]])
            self.calls = 0
        def get_dof_positions(self):
            self.calls += 1
            return self.values.copy()
        def set_dof_positions(self, value, indices):
            self.values[indices] = value
    batch = View()
    cache = BatchReadCache(batch)
    # Local views have separate index spaces in production; assert forwarding.
    class Local:
        def set_dof_positions(self, values, indices):
            assert indices == [0]
            batch.values[1] = values[0]
    first = BatchSliceView(Local(), cache, 0)
    second = BatchSliceView(Local(), cache, 1)
    np.testing.assert_array_equal(first.get_dof_positions(), [[1., 2.]])
    np.testing.assert_array_equal(second.get_dof_positions(), [[3., 4.]])
    assert batch.calls == 1
    second.set_dof_positions([[7., 8.]], [0])
    np.testing.assert_array_equal(second.get_dof_positions(), [[7., 8.]])
    assert batch.calls == 2
    cache.invalidate()  # physics advanced
    first.get_dof_positions()
    assert batch.calls == 3
