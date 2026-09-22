"""Physics-rate diagnostics stay in memory; persist only replay-rate qpos."""
import numpy as np


def compact_states(arrays, fps=30):
    times=np.asarray(arrays['times_s'])
    if fps<=0 or len(times)==0:raise ValueError('fps and times must be positive/nonempty')
    indices=np.unique(np.r_[0,np.searchsorted(times,np.arange(0,times[-1],1/fps)),len(times)-1])
    return dict(times_s=times[indices],qpos=arrays['qpos'][indices])
