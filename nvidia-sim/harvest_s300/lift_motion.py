"""Lift-only phases for the existing native motion runner; no new physics loop."""
import numpy as np


def plan_lift_cycle(start, lower, upper, *, low=.05, high=.70):
    q=np.asarray(start,dtype=float)
    if q.shape!=(7,) or not np.isfinite(q).all():
        raise ValueError('Lift cycle needs seven finite source joint positions')
    lower=np.asarray(lower);upper=np.asarray(upper)
    if lower.shape!=(7,) or upper.shape!=(7,) or np.any(q<lower) or np.any(q>upper):
        raise ValueError('Initial lift/arm pose exceeds source joint limits')
    if not np.isfinite([low,high]).all() or not lower[0]<=low<high<=upper[0]:
        raise ValueError('Lift cycle must stay within the actual prismatic joint limits')
    previous=q.copy();phases=[]
    for name,duration,height in [('lift_raise',6.,high),('lift_high_hold',2.,high),
            ('lift_lower',6.,low),('lift_low_hold',2.,low),
            ('lift_return',3.,q[0]),('lift_final_hold',2.,q[0])]:
        target=q.copy();target[0]=height
        phases.append((name,duration,np.array([previous,target]),.044,0.,0.))
        previous=target
    return phases,dict(scope='real carriage raise/lower diagnostic; not harvesting',
        interpolation='existing smoothstep joint interpolation',
        training_eligible=False,harvest_success=None,
        phases=[dict(name=n,duration_s=t,waypoint_q=qs.tolist(),end_gap_m=g,
                     belt_local_y_m_s=[l,r]) for n,t,qs,g,l,r in phases])
