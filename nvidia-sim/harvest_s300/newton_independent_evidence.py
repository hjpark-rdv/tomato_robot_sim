"""Read-only evidence aggregation for independent Newton parameter trials.

Uses existing executor files. Completion, stopping policy and model fidelity
are deliberately separate: a full diagnostic observation is not automatically
a validated harvest or a recommended configuration.
"""
from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from benchmark_newton import contact_validity, write_json
from newton_inputs import sha256


def complete_schedule(summary, commands, state):
    n = len(commands['phase'])
    clock=state['scheduled_time'] if 'scheduled_time' in state else state['time']
    clock_matches=(bool(np.max(np.abs(clock-commands['time_edges']),initial=0)<=1e-9)
        if summary.get('backend')=='cpu' and clock.shape==commands['time_edges'].shape
        else np.array_equal(clock,commands['time_edges']))
    return bool(summary.get('status')=='completed' and summary.get('stop') is None
        and summary.get('actual_steps')==summary.get('requested_steps')==n
        and len(state['time'])==n+1 and str(state['phase'][-1])==str(commands['phase'][-1])
        and np.array_equal(commands['dt'],np.diff(commands['time_edges']))
        and np.all(np.diff(clock)>0) and clock_matches
        and summary.get('finite_recorded_state') is True and summary.get('qacc_finite') is True
        and summary.get('solver_capacity_valid') is True)


def scalar(value):
    return float(value) if isinstance(value,(int,float,np.number)) and np.isfinite(value) else None


def inspect_run(path):
    path = Path(path)
    summary = json.loads((path/'summary.json').read_text())
    row = dict(path=str(path),summary_sha256=sha256(path/'summary.json'),
        status=summary.get('status'),stop=summary.get('stop'),error=summary.get('error'),
        input=summary.get('input'),input_sha256=summary.get('input_sha256'),
        executed_commands_sha256=summary.get('executed_commands_sha256'),
        backend=summary.get('backend'),worlds=summary.get('worlds'),
        nv=summary.get('source_model_counts',{}).get('nv'),
        contact_hz=summary.get('physics_hz'),free_hz=summary.get('free_hz'),
        steps=summary.get('actual_steps'),requested_steps=summary.get('requested_steps'),
        completed_schedule=False,physical_fidelity_validated=False,harvest_success=None,
        RTF=scalar(summary.get('realtime_factor')),setup_s=scalar(summary.get('setup_wall_s')),
        diagnostic_instrumented=bool(summary.get('profile_instrumented') or summary.get('contact_policy',{}).get('diagnostic_instrumented')),
        loop_s=scalar(summary.get('loop_wall_s')),contact_evaluation=contact_validity(summary),
        active_penetration_m=scalar(summary.get('max_active_penetration_m')),
        max_global_contact_force_n=scalar(summary.get('max_contact_force_n')),
        detached=summary.get('detached_per_world'),first_violations=summary.get('first_active_violations'),
        phases=summary.get('phase_performance',{}),training_eligible=False)
    if not (path/'states.npz').is_file() or not (path/'executed_commands.npz').is_file():
        return row
    with np.load(path/'states.npz',allow_pickle=False) as states, \
         np.load(path/'executed_commands.npz',allow_pickle=False) as commands:
        clock=states['scheduled_time'] if 'scheduled_time' in states else states['time']
        row['completed_schedule'] = complete_schedule(summary,commands,states)
        row['schedule_duration_s'] = float(commands['time_edges'][-1])
        row['scheduled_end_s'] = float(clock[-1])
        row['recorded_steps'] = len(states['time'])-1
        finite = bool(np.isfinite(states['qpos']).all() and np.isfinite(states['qvel']).all())
        row['recorded_state_finite'] = finite
        if not finite:
            mask=np.isfinite(states['qpos']).all(axis=1)&np.isfinite(states['qvel']).all(axis=1)
            first=int(np.flatnonzero(~mask)[0])
            row['first_nonfinite_scheduled_s'] = float(clock[first])
        tracking=states['robot_q']-states['command']
        row['tracking_max_abs_per_dof'] = dict(zip(commands['dof_names'].tolist(),
            np.max(np.abs(tracking),axis=0).tolist()))
        row['tracking_units'] = 'rail/lift/jaws metres, arm radians'
        if finite and (path/'native_metadata.json').is_file() and (path/'physics_metrics.npz').is_file():
            mapping=json.loads((path/'native_metadata.json').read_text())['source_body_to_native_name']
            names=[mapping.get(str(p),str(p)) for p in states['body_paths']]
            source=json.loads((Path(summary['input'])/'source_export.json').read_text())
            belt_name=source['robot']['belt_body_names'][0]
            belt_index=names.index(belt_name)
            pose=states['poses']
            belt_rotation=Rotation.from_quat(pose[:,belt_index,3:])
            speeds=commands['belt_surface_velocity_local_y_m_s'][:,0]
            moving=np.flatnonzero(np.abs(speeds)>1e-8)
            row['belt_motion_observed']=bool(len(moving) and int(moving[0]) < len(pose)-1)
            first=min(int(moving[0]),len(pose)-1) if len(moving) else 0
            last=min(int(moving[-1])+1,len(pose)-1) if len(moving) else len(pose)-1
            nonzero=speeds[moving]
            sign=float(np.sign(nonzero[0])) if len(nonzero) else 1.
            with np.load(path/'physics_metrics.npz',allow_pickle=False) as metrics:
                break_step=metrics['break_step']
                if break_step.ndim==2:break_step=break_step[0]
                fruits=[]
                for i,spec in enumerate(source['attachments'] if row['belt_motion_observed'] else []):
                    index=names.index(spec['name'])
                    local=belt_rotation.inv().apply(pose[:,index,:3]-pose[:,belt_index,:3])
                    end=min(int(break_step[i])+1,last) if break_step[i]>=0 else last
                    early=end < first
                    end=max(first,end)
                    segment=local[first:max(first+1,end+1),1]
                    fruits.append(dict(name=spec['name'],released=bool(break_step[i]>=0),
                        premature_release_before_belt=early,
                        observed_until_step=end,inward_before_release_m=float(sign*(local[end,1]-local[first,1])) if not early else None,
                        inward_max_m=float(np.max(sign*(segment-local[first,1])))))
                row['fruit_traction']=fruits
                row['traction_start_scheduled_s']=float(clock[first])
                row['traction_scope']='Fruit centre in moving left-belt body frame, first nonzero belt command until release or belt stop, excluding approach/withdrawal. Includes jaw/contact motion, not proof of belt-only causation.'
                row['max_attachment_force_n']=scalar(np.max(metrics['attachment_force_n']))
                row['max_attachment_torque_nm']=scalar(np.max(metrics['attachment_torque_nm']))
                row['max_acceleration']=scalar(np.max(metrics['qacc_abs_max'])) if 'qacc_abs_max' in metrics else None
    row['qualified_policy_throughput'] = bool(row['completed_schedule'] and row['contact_evaluation']['passed'])
    row['clean_policy_throughput'] = bool(row['qualified_policy_throughput'] and not row['diagnostic_instrumented'])
    row['all_recorded_phases_realtime'] = bool(row['completed_schedule'] and row['phases'] and
        all(v['realtime_factor']>=1 for v in row['phases'].values()))
    return row


def build(root,output):
    root,output=Path(root).resolve(),Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    runs=[]
    invalidated={}
    for notice in sorted(root.glob('*invalidated.json')):
        data=json.loads(notice.read_text())
        for name in data.get('invalidated_runs',[]):
            invalidated[str((root/name).resolve())]=dict(notice=str(notice),reason=data.get('reason'))
    for path in sorted(root.rglob('summary.json')):
        try:
            summary=json.loads(path.read_text())
            if summary.get('schema')!='s300-newton-fixed-step-v1':continue
            row=inspect_run(path.parent)
            row['experiment_preparation_valid']=str(path.parent.resolve()) not in invalidated
            if not row['experiment_preparation_valid']:
                row['preparation_error']=invalidated[str(path.parent.resolve())]
                row['evidence_classification']='invalid_experiment_preparation'
                row['clean_policy_throughput']=False
            elif row.get('recorded_state_finite') is False:
                row['evidence_classification']='nonfinite_physics_preserved_even_if_summary_stale'
            elif row.get('completed_schedule'):
                row['evidence_classification']='full_schedule_observation'
            else:
                row['evidence_classification']='stopped_or_incomplete'
            runs.append(row)
        except Exception as error:
            runs.append(dict(path=str(path.parent),evidence_read_error=repr(error),completed_schedule=False))
    result=dict(schema='s300-independent-evidence-v1',root=str(root),training_eligible=False,
        harvest_success=None,runs=runs,
        interpretation='Independent execution evidence. Full observation and policy validity are not real-plant fidelity or harvest success. Joint-count reuse and preparation-error classifications are separate artifacts.')
    write_json(output/'evidence.json',result)
    fields=['path','nv','contact_hz','free_hz','status','stop','completed_schedule','RTF','active_penetration_m','detached','qualified_policy_throughput','all_recorded_phases_realtime']
    with (output/'runs.csv').open('x') as stream:
        writer=csv.DictWriter(stream,fields,extrasaction='ignore');writer.writeheader();writer.writerows(runs)
    rows=[]
    for row in runs:
        values=[Path(row['path']).relative_to(root),row.get('status'),row.get('RTF'),row.get('active_penetration_m'),row.get('detached'),row.get('completed_schedule'),row.get('qualified_policy_throughput')]
        rows.append('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in values)+'</tr>')
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Newton independent evidence</title><style>body{font:15px sans-serif;margin:24px}td,th{padding:8px;border:1px solid #ddd}table{border-collapse:collapse}td:first-child{max-width:600px;overflow-wrap:anywhere}</style><h1>Newton GPU 개별 시험</h1><p>전체 관측 완주 / 명시한 접촉 정책 통과 / 실제 물리 타당성은 각각 확인합니다. 실물 보정과 수확 성공은 미검증입니다.</p><table><tr><th>실행</th><th>상태</th><th>전체 RTF</th><th>활성 겹침 m</th><th>분리</th><th>전체 기록 확인</th><th>정책 통과</th></tr>'+''.join(rows)+'</table>')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(json.dumps(dict(runs=len(build(a.root,a.output)['runs']),output=str(a.output))))
