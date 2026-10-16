"""Read-only convergence-tolerance evidence for the existing harvest executor.

Runs are launched through benchmark_solver_compare.py and the canonical GPU
wrapper. This module only compares their recorded states/forces/events; changing
solver convergence tolerance is never called an identical trajectory.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        while data:=f.read(1<<20):h.update(data)
    return h.hexdigest()


def qualified_full_speedup(reference, candidate):
    if not (reference['full_existing_sequence_physically_valid'] and candidate['full_existing_sequence_physically_valid']):
        return None
    return candidate['realtime_factor']/reference['realtime_factor']


def compare_phase_performance(reference, candidate):
    rows=[]
    for name, x in reference.items():
        y=candidate.get(name)
        same_duration=bool(y and x['steps']==y['steps'] and np.isclose(x['simulated_s'],y['simulated_s'],rtol=0,atol=1e-9))
        rows.append(dict(phase=name,reference_RTF=x['realtime_factor'],
            candidate_RTF=y['realtime_factor'] if y else None,
            equal_executed_duration=same_duration,
            speedup=y['realtime_factor']/x['realtime_factor'] if same_duration else None,
            reference_steps=x['steps'],candidate_steps=y['steps'] if y else 0,
            reference_simulated_s=x['simulated_s'],candidate_simulated_s=y['simulated_s'] if y else 0,
            reference_wall_s=x['wall_s'],candidate_wall_s=y['wall_s'] if y else None))
    return rows


def read_run(path):
    path=Path(path).resolve()
    summary=json.loads((path/'summary.json').read_text())
    events=json.loads((path/'break_events.json').read_text())
    state=np.load(path/'states.npz',allow_pickle=False)
    metric=np.load(path/'physics_metrics.npz',allow_pickle=False)
    command=np.load(path/'executed_commands.npz',allow_pickle=False)
    source=json.loads((Path(summary['input'])/'source_export.json').read_text())
    invocation=json.loads((path/'invocation.json').read_text())
    names=[x['name'] for x in source['attachments']]
    phase=state['phase']
    first_approach=int(np.flatnonzero(phase=='approach')[0])
    before=state['scheduled_time'][first_approach]
    plant=np.array([str(x).startswith('/World/ElasticTarget/') for x in state['body_paths']])
    fruit=np.array(['/Truss_00_Fruit_' in str(x) for x in state['body_paths']])
    displacement=np.linalg.norm(state['poses'][:first_approach,:,:3]-state['poses'][0,None,:,:3],axis=-1)
    tracking=state['robot_q'].astype(np.float64)-state['command']
    premature=[e for e in events if e['scheduled_release_time_s']<before]
    physical=bool(summary.get('status')=='completed' and summary.get('stop') is None and
        summary.get('actual_steps')==summary.get('requested_steps')==len(command['phase']) and
        len(state['phase'])==len(command['phase'])+1 and
        state['phase'][-1]==command['phase'][-1] and
        summary.get('finite_recorded_state') is True and summary.get('qacc_finite') is True and
        summary.get('solver_capacity_valid') is True and summary.get('active_contact_criterion_passed') is True)
    options=invocation.get('argv',[])
    tolerance=None
    if '--solver-tolerance' in options:
        tolerance=float(options[options.index('--solver-tolerance')+1])
    row=dict(path=str(path),tolerance_override=tolerance,
        full_existing_sequence_physically_valid=physical,
        pre_approach_detach_events=premature,
        pre_approach_plant_max_displacement_m=float(np.max(displacement[:,plant])),
        pre_approach_fruit_max_displacement_m=float(np.max(displacement[:,fruit])),
        attachment_force_max_n=float(np.max(metric['attachment_force_n'])),
        attachment_force_max_per_fruit_n=dict(zip(names,np.max(metric['attachment_force_n'],axis=(0,1)).astype(float).tolist())),
        tracking_max_abs_per_dof=dict(zip(command['dof_names'].tolist(),np.max(np.abs(tracking),axis=0).tolist())),
        tracking_rms_per_dof=dict(zip(command['dof_names'].tolist(),np.sqrt(np.mean(tracking**2,axis=0)).tolist())),
        tracking_units='rail/lift/jaws in metres; arm joints in radians',
        break_events=events,
        max_contact_force_n=summary.get('max_contact_force_n'),
        effective_gpu_options=summary.get('effective_gpu_options'),
        first_active_violations=summary.get('first_active_violations'),
        simulated_s=summary.get('simulated_s'),scheduled_simulated_s=summary.get('scheduled_simulated_s'),
        contact_force_scope='Aggregate contact-force maximum, including released-fruit contacts; no pair attribution inferred',
        max_active_penetration_m=summary.get('max_active_penetration_m'),
        detached_per_world=summary.get('detached_per_world'),
        phase_performance=summary.get('phase_performance'),
        loop_wall_s=summary.get('loop_wall_s'),realtime_factor=summary.get('realtime_factor'),
        actual_steps=summary.get('actual_steps'),requested_steps=summary.get('requested_steps'),
        stop=summary.get('stop'),status=summary.get('status'),
        input_sha256=summary.get('input_sha256'),commands_sha256=summary.get('commands_sha256'),
        executed_commands_sha256=summary.get('executed_commands_sha256'),
        source_model_array_sha256=summary.get('engine',{}).get('source_model_array_sha256'),
        invocation_sha256=sha(path/'invocation.json'),
        invocation_code_sha256=invocation.get('code_sha256',{}))
    return row,state,metric,command


def compare(reference,candidate):
    a,sa,ma,ca=read_run(reference)
    b,sb,mb,cb=read_run(candidate)
    common_steps=min(len(sa['time']),len(sb['time']))
    assert np.array_equal(sa['body_paths'],sb['body_paths'])
    assert np.array_equal(ca['dof_names'],cb['dof_names'])
    before=int(np.flatnonzero(sa['phase']=='approach')[0])
    plant=np.array([str(x).startswith('/World/ElasticTarget/') for x in sa['body_paths']])
    prefixdiff=np.linalg.norm(sa['poses'][:before,:,:3]-sb['poses'][:before,:,:3],axis=-1)
    events_a={e['name']:e for e in a['break_events']}
    events_b={e['name']:e for e in b['break_events']}
    release=[]
    for name in sorted(set(events_a)|set(events_b)):
        x,y=events_a.get(name),events_b.get(name)
        release.append(dict(name=name,reference=x,candidate=y,
            scheduled_time_delta_s=(y['scheduled_release_time_s']-x['scheduled_release_time_s']) if x and y else None,
            physics_step_delta=(y['solver_step']-x['solver_step']) if x and y else None))
    phase=compare_phase_performance(a['phase_performance'],b['phase_performance'])
    result=dict(schema='s300-solver-convergence-comparison-v1',training_eligible=False,harvest_success=None,
        identical_trajectories_claimed=False,reference=a,candidate=b,
        same_input_file=bool(a['input_sha256']==b['input_sha256']),
        same_command_file=bool(a['commands_sha256']==b['commands_sha256']),
        same_executed_command_schedule=bool(a['executed_commands_sha256']==b['executed_commands_sha256']),
        changed_code_files=[name for name in sorted(set(a['invocation_code_sha256'])|set(b['invocation_code_sha256']))
            if a['invocation_code_sha256'].get(name)!=b['invocation_code_sha256'].get(name)],
        full_qpos_exact=False if sa['qpos'].shape!=sb['qpos'].shape else bool(np.array_equal(sa['qpos'],sb['qpos'])),
        prefix_plant_max_position_difference_m=float(np.max(prefixdiff[:,plant])),
        prefix_fruit_displacement_ratio=b['pre_approach_fruit_max_displacement_m']/max(a['pre_approach_fruit_max_displacement_m'],1e-30),
        full_robot_q_max_abs_difference=np.max(np.abs(sa['robot_q'][:common_steps]-sb['robot_q'][:common_steps]),axis=0).astype(float).tolist(),
        same_detached_fruit_set=bool(set(events_a)==set(events_b)),
        release_comparison=release,phase_comparison=phase,
        full_realtime_speedup=qualified_full_speedup(a,b),
        numerical_setting_note='Same prepared model/materials/commands, changed solver convergence tolerance; approximate contacts and physical trajectory can differ.')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('reference',type=Path);p.add_argument('candidate',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise ValueError('Refusing to overwrite evidence')
    args.output.write_text(json.dumps(compare(args.reference,args.candidate),indent=2,ensure_ascii=False,allow_nan=False)+'\n')
