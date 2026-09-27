"""Evaluation-only phase aliases for an unchanged saved azimuth-only baseline."""
import copy

ALIASES = dict(ready='ready', preapproach='preapproach', approach='approach',
               entry='approach', insert='insert', rise='seat', rise_mid='seat',
               hold='hold', pull='verify')


def evaluation_trace(plan, trace, legacy=False):
    mode=plan.get('parameters',{}).get('trajectory_mode')
    if mode=='diagnostic_pose_waypoints_v1' and not legacy:return trace,None
    if not legacy or mode!='staged6d':
        raise ValueError('Use saved diagnostic pose paths, or explicitly opt into a staged6d baseline')
    if any(r['phase'] not in ALIASES for r in trace):raise ValueError('Unknown legacy baseline phase')
    mapped=copy.deepcopy(trace)
    for row in mapped:row['phase']=ALIASES[row['phase']]
    return mapped,dict(source_trajectory_mode=mode,phase_aliases=ALIASES,
        commands_changed=False,seconds_changed=False,
        scope='Evaluation phase aliases only; original saved azimuth-only commands are unchanged; no action14 labels')
