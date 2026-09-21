"""Offline checks of identical-command replay across GPU environment counts."""
import json
import re
from pathlib import Path
import numpy as np


def local_path(value):
    return re.sub(r'/env_\d+/', '/env_*/', value) if isinstance(value,str) else value


def compare_replays(reference, candidate):
    """Compare every candidate slot to env_0 of an independently saved replay.

    This does not assert CPU equivalence or successful harvesting. It checks
    whether changing GPU batch size changes the same commanded experiment.
    """
    reference,candidate=Path(reference),Path(candidate)
    a=json.loads((reference/'report.json').read_text())
    b=json.loads((candidate/'report.json').read_text())
    if not a.get('complete') or not b.get('complete') or 'motion' not in a or 'motion' not in b:
        return dict(passed=False,reason='incomplete replay')
    if a['fixture']['commands_sha256']!=b['fixture']['commands_sha256']:
        return dict(passed=False,reason='different commands')
    if len(b['motion']['outcomes'])!=b['num_envs']:
        return dict(passed=False,reason='not all environments executed')
    baseline=a['motion']['outcomes'][0]
    trace_a=json.loads((reference/'trace_0.json').read_text())
    final_a=np.load(reference/'final_state_0.npz')
    checks=[]
    for outcome in b['motion']['outcomes']:
        i=outcome['env_index'];trace=json.loads((candidate/f'trace_{i}.json').read_text())
        failures=[];metrics={}
        for key in ('classification','abort_reason','target_broken','other_broken','inserted','retained_hook'):
            if outcome.get(key)!=baseline.get(key):failures.append(key)
        if local_path(outcome.get('first_contact_object'))!=local_path(baseline.get('first_contact_object')):
            failures.append('first_contact_object')
        if len(trace)!=len(trace_a):failures.append('control_step_count')
        for key in ('joints','target_displacement_m','main_stem_displacement_m','gap_m'):
            if len(trace)==len(trace_a) and trace:
                metrics[key]=float(np.max(np.abs(np.asarray([r[key] for r in trace])-np.asarray([r[key] for r in trace_a]))))
                # 0.1 mm position, 0.0001 rad arm joints; lift uses metres.
                if metrics[key]>1e-4:failures.append(key)
        final=np.load(candidate/f'final_state_{i}.npz')
        for key in ('elastic_bodies','fruits'):
            metrics[key+'_position_m']=float(np.linalg.norm(final[key][...,:3]-final_a[key][...,:3],axis=-1).max())
            if metrics[key+'_position_m']>1e-4:failures.append(key+'_position_m')
        checks.append(dict(env_index=i,passed=not failures,failures=failures,metrics=metrics))
    return dict(passed=all(c['passed'] for c in checks),reference=str(reference),candidate=str(candidate),
                scope='same GPU commands at different environment counts; not CPU equivalence',checks=checks)
