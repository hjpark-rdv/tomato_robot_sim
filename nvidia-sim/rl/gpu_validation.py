"""Unit-aware GPU startup screening, distinct from CPU trajectory equivalence.

Reset itself still requires 1e-6 equality of every state field. This check is
only for a short, no-action GPU evolution, where solve ordering changes rounding.
Position tolerance is 0.1 mm (1/15 of the 1.5 mm hook gap criterion). The angular
velocity bound corresponds to 0.00033 rad during one 60 Hz control interval.
These limits do not certify contact outcomes or relax the CPU replay comparison.
"""
import numpy as np

LIMITS = {
    'robot_q_max_abs': 1e-6, 'robot_dq_max_abs': 1e-6, 'robot_root_max_abs': 1e-6,
    'elastic_q_max_abs_rad': .001, 'elastic_dq_max_abs_rad_s': .01,
    'body_position_m': .0001, 'body_orientation_rad': .001,
    'body_linear_velocity_m_s': .001, 'body_angular_velocity_rad_s': .02,
    'preload_max_abs': 1e-6, 'stiffness_max_abs': 1e-6, 'damping_max_abs': 1e-6,
}


def check_clone(reference, state, broken=False):
    if not all(np.isfinite(v).all() for v in state.values()):
        return dict(passed=False,reason='non-finite state',limits=LIMITS.copy())
    metrics={}
    for key in ('robot_q','robot_dq','robot_root','preload','stiffness','damping'):
        metrics[key+'_max_abs']=float(np.max(abs(state[key]-reference[key])))
    for key,unit in [('elastic_q','rad'),('elastic_dq','rad_s')]:
        metrics[key+'_max_abs_'+unit]=float(np.max(abs(state[key]-reference[key])))
    for metric in ('body_position_m','body_orientation_rad','body_linear_velocity_m_s','body_angular_velocity_rad_s'):
        metrics[metric]=0.
    for key in ('elastic_bodies','fruits'):
        a=np.asarray(state[key],dtype=np.float64);b=np.asarray(reference[key],dtype=np.float64)
        for sl,metric in [(slice(0,3),'body_position_m'),(slice(7,10),'body_linear_velocity_m_s'),(slice(10,13),'body_angular_velocity_rad_s')]:
            metrics[metric]=max(metrics[metric],float(np.linalg.norm(a[...,sl]-b[...,sl],axis=-1).max()))
        qa=a[...,3:7];qb=b[...,3:7]
        na=np.linalg.norm(qa,axis=-1,keepdims=True);nb=np.linalg.norm(qb,axis=-1,keepdims=True)
        if np.any(na<1e-9) or np.any(nb<1e-9):
            return dict(passed=False,reason='invalid quaternion',limits=LIMITS.copy())
        qa=qa/na;qb=qb/nb
        angle=2*np.arccos(np.clip(abs(np.sum(qa*qb,axis=-1)),0,1))
        metrics['body_orientation_rad']=max(metrics['body_orientation_rad'],float(angle.max()))
    failures=[key for key,limit in LIMITS.items() if not np.isfinite(metrics[key]) or metrics[key]>limit]
    if broken:failures.append('joint_break')
    return dict(passed=not failures,metrics=metrics,limits=LIMITS.copy(),failures=failures,
                scope='GPU no-action startup screen; initial reset equality and CPU replay equivalence are separate')
