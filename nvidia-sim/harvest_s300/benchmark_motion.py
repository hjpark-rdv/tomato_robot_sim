#!/usr/bin/env python3
"""Bounded serial or synchronized CPU campaign using the motion executor."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import psutil


def sample_tree(process,tracked):
    try:
        tree=[process]+process.children(recursive=True)
        for child in tree:
            if child.pid not in tracked:tracked[child.pid]=child
        cpu=sum(tracked[p.pid].cpu_percent() for p in tree if p.is_running())
        rss=sum(p.memory_info().rss for p in tree if p.is_running())
    except psutil.Error:
        cpu=rss=0
    return dict(cpu_percent=cpu,rss_bytes=rss)


def sample_gpu():
    try:
        values=subprocess.check_output(['nvidia-smi','--query-gpu=utilization.gpu,memory.used',
            '--format=csv,noheader,nounits'],text=True,timeout=3).strip().split(',')
        gpu,memory=map(float,values)
    except (OSError,subprocess.SubprocessError,ValueError):
        gpu=memory=None
    return dict(gpu_percent=gpu,gpu_memory_mib=memory)


def sample(process,tracked):
    return dict(**sample_tree(process,tracked),**sample_gpu())


def _argument(arguments, name, default=None):
    value=default
    for i,arg in enumerate(arguments):
        if arg==name:
            if i+1>=len(arguments):raise ValueError('Missing value for '+name)
            value=arguments[i+1]
        elif arg.startswith(name+'='):value=arg.split('=',1)[1]
    return value


def validate_parallel_cases(cases, output, jobs):
    """Validate the entire bounded campaign before launching any process."""
    if jobs not in (2,4,8,16,24):raise ValueError('Parallel jobs must be 2, 4, 8, 16 or 24')
    names=[]
    for case in cases:
        name=case.get('name')
        if not isinstance(name,str) or not name or Path(name).name!=name or name in ('.','..'):
            raise ValueError('Trial names must be single path components')
        if name in names:raise ValueError('Duplicate trial name: '+name)
        names.append(name)
        arguments=case.get('arguments')
        if not isinstance(arguments,list) or not all(isinstance(x,str) for x in arguments):
            raise ValueError('Case arguments must be a list of strings')
        if _argument(arguments,'--device','cpu')!='cpu':
            raise ValueError('Concurrent native trials are CPU-only: '+name)
        if any(x=='--start-gate' or x.startswith('--start-gate=') for x in arguments):
            raise ValueError('The controller owns the synchronized start gate')
        for path in (output/name,output/(name+'.log'),output/(name+'_resources.json')):
            if path.exists():raise RuntimeError('Refusing to overwrite existing trial evidence '+str(path))
    for index in range((len(cases)+jobs-1)//jobs):
        for suffix in ('gate','resources'):
            path=output/(f'parallel_group_{index:04d}_{suffix}.json')
            if path.exists():raise RuntimeError('Refusing to overwrite '+str(path))
            if suffix=='gate' and path.with_suffix('.tmp').exists():
                raise RuntimeError('Refusing to overwrite '+str(path.with_suffix('.tmp')))


def _stop_child(child):
    if child.poll() is not None:return
    try:os.killpg(child.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:child.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:os.killpg(child.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        child.wait()


def owned_ready_process(wrapper, native_pid):
    """Validate readiness from the owned executor, including python.sh forks.

    start_new_session gives the wrapper its own process group. An alive native
    child may publish a different PID; it must be a descendant of that exact
    wrapper and remain in its group. A merely unrelated alive PID is rejected.
    """
    if not isinstance(native_pid,int) or isinstance(native_pid,bool) or native_pid<=0:
        raise ValueError('ready_native_pid_invalid')
    try:
        group=os.getpgid(native_pid)
        if group!=wrapper.pid:raise ValueError('ready_native_process_group_mismatch')
        if native_pid==wrapper.pid:
            native=wrapper;descendant=False
        else:
            matches=[p for p in wrapper.children(recursive=True) if p.pid==native_pid]
            if not matches:raise ValueError('ready_native_pid_not_descendant')
            native=matches[0];descendant=True
        if not native.is_running() or native.status() in (psutil.STATUS_ZOMBIE,psutil.STATUS_DEAD):
            raise ValueError('ready_native_pid_not_alive')
    except (OSError,psutil.Error) as exc:
        raise ValueError('ready_native_process_missing') from exc
    return dict(wrapper_pid=wrapper.pid,native_pid=native_pid,
                native_process_group=group,native_is_descendant=descendant)


def aggregate_parallel_group(rows, *, group_started, gate_released, group_finished, blocker=None):
    """Compute measured concurrent throughput only if every trial is valid.

    Cross-process monotonic loop times delimit the actual overlapping physics
    window. Summing individual rates would not measure concurrent throughput.
    """
    reasons=[]
    if blocker:reasons.append(blocker)
    starts=[];ends=[];simulated=[]
    for row in rows:
        summary=row.get('summary')
        name=row['name']
        if row.get('timeout'):reasons.append(name+':timeout')
        if row.get('returncode')!=0:reasons.append(name+':nonzero_exit')
        if not summary:
            reasons.append(name+':summary_missing');continue
        if summary.get('completed') is not True:reasons.append(name+':trial_incomplete')
        if summary.get('initialization_valid') is not True:reasons.append(name+':initialization_invalid_or_unknown')
        if summary.get('stop_reason') is not None:reasons.append(name+':'+str(summary['stop_reason']))
        values=[summary.get('loop_started_monotonic_s'),summary.get('loop_finished_monotonic_s'),
                summary.get('aggregate_simulated_s',summary.get('simulated_s'))]
        if any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in values) or \
                values[1]<=values[0] or values[2]<=0:
            reasons.append(name+':loop_timing_or_simulation_missing_invalid');continue
        if gate_released is None or values[0]<gate_released:
            reasons.append(name+':loop_started_before_gate')
        starts.append(values[0]);ends.append(values[1]);simulated.append(values[2])
    valid=bool(rows) and not reasons and len(starts)==len(rows)
    window=max(ends)-min(starts) if starts and len(starts)==len(rows) else None
    return dict(throughput_eligible=valid,exclusion_reasons=reasons,
                measured_concurrent_physics_window_s=window,
                sum_aggregate_simulated_s=sum(simulated) if len(simulated)==len(rows) else None,
                aggregate_sim_s_per_wall_s=sum(simulated)/window if valid else None,
                loop_start_spread_s=max(starts)-min(starts) if starts else None,
                group_total_wall_s=group_finished-group_started,
                group_startup_wall_s=None if gate_released is None else gate_released-group_started,
                post_loop_wall_s=None if not ends else group_finished-max(ends),
                gate_released_monotonic_s=gate_released,
                interpretation='Every trial must be valid; initialization/startup excluded from concurrent physics throughput')


def run_parallel_group(a, cases, index):
    """Launch a bounded CPU group and release one shared start gate."""
    gate=(a.output/f'parallel_group_{index:04d}_gate.json').resolve()
    resource_path=a.output/f'parallel_group_{index:04d}_resources.json'
    group_started=time.perf_counter();gate_released=None;blocker=None
    workers=[];group_samples=[];last_notice=0.;last_resource=-float('inf')
    try:
        for case in cases:
            name=case['name'];folder=(a.output/name).resolve()
            command=[str(a.python),'--no-ros-env','-u',str(Path(__file__).with_name('run_motion.py')),
                     str(a.assembly),'--output',str(folder)]+list(case['arguments'])+['--start-gate',str(gate)]
            env=os.environ.copy();env['OMNI_KIT_ALLOW_ROOT']='1';env.setdefault('DISPLAY',':0')
            env.update(case.get('environment',{}))
            for key in ('PYTHONPATH','PYTHONHOME','VIRTUAL_ENV'):env.pop(key,None)
            log=(a.output/(name+'.log')).open('x')
            launched=time.perf_counter()
            try:child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            except Exception:log.close();raise
            proc=psutil.Process(child.pid)
            workers.append(dict(name=name,folder=folder,command=command,child=child,process=proc,
                                log=log,tracked={proc.pid:proc},samples=[],started=launched,
                                ready=None,timed_out=False))
            print('[BENCH] parallel start',name,'pid',child.pid,flush=True)
        while True:
            now=time.perf_counter();elapsed=now-group_started
            children=[];collect_resources=now-last_resource>=1.
            for w in workers:
                child=w['child'];ready=w['folder']/'ready.json'
                if w['ready'] is None and ready.exists():
                    try:info=json.loads(ready.read_text())
                    except (OSError,json.JSONDecodeError):info=None
                    if info is not None:
                        try:identity=owned_ready_process(w['process'],info.get('pid'))
                        except ValueError as exc:blocker=w['name']+':'+str(exc)
                        else:
                            info=dict(info,**identity)
                            if info.get('initial_valid') is not True:blocker=w['name']+':ready_initialization_invalid_or_unknown'
                            elif not isinstance(info.get('ready_monotonic_s'),(int,float)) or \
                                not math.isfinite(info['ready_monotonic_s']):blocker=w['name']+':ready_time_missing_invalid'
                            else:w['ready']=info
                if child.poll() is None:
                    if collect_resources:
                        metrics=sample_tree(w['process'],w['tracked'])
                        record=dict(elapsed_s=now-w['started'],monotonic_s=now,
                                    phase='startup_waiting' if gate_released is None else 'released',**metrics)
                        w['samples'].append(record);children.append(dict(name=w['name'],pid=child.pid,**metrics))
                    if now-w['started']>a.timeout:w['timed_out']=True;blocker=w['name']+':timeout'
                elif gate_released is None:blocker=w['name']+':exited_before_gate'
            if collect_resources:
                group_samples.append(dict(monotonic_s=now,elapsed_s=elapsed,
                                          phase='startup_waiting' if gate_released is None else 'released',
                                          children=children,**sample_gpu()))
                last_resource=now
            if blocker:
                for w in workers:_stop_child(w['child'])
                break
            if gate_released is None and all(w['ready'] is not None for w in workers):
                gate_released=time.perf_counter()
                temporary=gate.with_suffix('.tmp')
                with temporary.open('x') as stream:
                    json.dump(dict(group=index,released_monotonic_s=gate_released,
                                   children=[dict(name=w['name'],pid=w['child'].pid,ready=w['ready']) for w in workers]),stream)
                temporary.rename(gate)
                print('[BENCH] release group',index,'startup seconds',round(gate_released-group_started,2),flush=True)
            if all(w['child'].poll() is not None for w in workers):break
            if elapsed-last_notice>=15:
                print('[BENCH] parallel group',index,round(elapsed,1),'seconds',flush=True);last_notice=elapsed
            time.sleep(.25)
    finally:
        for w in workers:
            if w['child'].poll() is None:_stop_child(w['child'])
            w['log'].close()
    finished=time.perf_counter();rows=[]
    for w in workers:
        resource=a.output/(w['name']+'_resources.json')
        resource.write_text(json.dumps(w['samples'],indent=2))
        summary_file=w['folder']/'summary.json'
        summary=json.loads(summary_file.read_text()) if summary_file.exists() else None
        rows.append(dict(name=w['name'],command=w['command'],returncode=w['child'].returncode,
                         timeout=w['timed_out'],total_wall_s=finished-w['started'],summary=summary,
                         startup_wall_s=None if w['ready'] is None else w['ready']['ready_monotonic_s']-w['started'],
                         ready_gate_wait_s=None if w['ready'] is None or gate_released is None else gate_released-w['ready']['ready_monotonic_s'],
                         ready=w['ready'],resource_samples=str(resource),
                         physics_interpretation='diagnostic component fixture' if '--isolate-background' in w['command'] else 'farm collision enabled'))
    resource_path.write_text(json.dumps(group_samples,indent=2))
    group=dict(index=index,jobs=len(workers),names=[w['name'] for w in workers],start_gate=str(gate),
               global_resource_samples=str(resource_path),
               **aggregate_parallel_group(rows,group_started=group_started,gate_released=gate_released,
                                          group_finished=finished,blocker=blocker))
    return rows,group


def main():
    p=argparse.ArgumentParser();p.add_argument('assembly',type=Path);p.add_argument('cases',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--timeout',type=float,default=240)
    p.add_argument('--python',type=Path,default=Path('/root/isaac-sim-6.1.0/python.sh'))
    p.add_argument('--jobs',type=int,choices=[1,2,4,8,16,24],default=1,help='Opt-in CPU processes; validate smaller groups before 8/16/24; each group waits for every ready.json then releases one synchronized gate')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    cases=json.loads(a.cases.read_text());results=[]
    if a.jobs>1:
        validate_parallel_cases(cases,a.output,a.jobs)
        results_path=a.output/(a.cases.stem+'_results.json')
        groups_path=a.output/(a.cases.stem+'_parallel_groups.json')
        for path in (results_path,groups_path):
            if path.exists():raise RuntimeError('Refusing to overwrite '+str(path))
        groups=[]
        for index,start in enumerate(range(0,len(cases),a.jobs)):
            rows,group=run_parallel_group(a,cases[start:start+a.jobs],index)
            results.extend(rows);groups.append(group)
            results_path.write_text(json.dumps(results,indent=2));groups_path.write_text(json.dumps(groups,indent=2))
            print('[BENCH] group done',index,'valid',group['throughput_eligible'],
                  'aggregate rate',group['aggregate_sim_s_per_wall_s'],flush=True)
        return 0
    for case in cases:
        name=case['name'];folder=a.output/name
        if folder.exists():raise RuntimeError('Refusing to overwrite existing trial '+str(folder))
        command=[str(a.python),'--no-ros-env','-u',str(Path(__file__).with_name('run_motion.py')),
                 str(a.assembly),'--output',str(folder)]+list(case['arguments'])
        env=os.environ.copy();env['OMNI_KIT_ALLOW_ROOT']='1';env.setdefault('DISPLAY',':0')
        env.update(case.get('environment',{}))
        for key in ('PYTHONPATH','PYTHONHOME','VIRTUAL_ENV'):env.pop(key,None)
        print('[BENCH] start',name,' '.join(command),flush=True)
        started=time.perf_counter();samples=[];timed_out=False
        with (a.output/(name+'.log')).open('w') as log:
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            proc=psutil.Process(child.pid);last_notice=0;tracked={proc.pid:proc}
            while child.poll() is None:
                elapsed=time.perf_counter()-started
                samples.append(dict(elapsed_s=elapsed,**sample(proc,tracked)))
                if elapsed-last_notice>=15:
                    print('[BENCH]',name,round(elapsed,1),'seconds',flush=True);last_notice=elapsed
                if elapsed>a.timeout:
                    timed_out=True;os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=15)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                    break
                time.sleep(1)
        (a.output/(name+'_resources.json')).write_text(json.dumps(samples,indent=2))
        summary=json.loads((folder/'summary.json').read_text()) if (folder/'summary.json').exists() else None
        row=dict(name=name,command=command,returncode=child.returncode,timeout=timed_out,
                 total_wall_s=time.perf_counter()-started,summary=summary,
                 resource_samples=str(a.output/(name+'_resources.json')),
                 physics_interpretation='diagnostic component fixture' if '--isolate-background' in command else 'farm collision enabled')
        results.append(row);(a.output/(a.cases.stem+'_results.json')).write_text(json.dumps(results,indent=2))
        print('[BENCH] done',name,'exit',child.returncode,'physics wall',summary.get('physics_wall_s') if summary else None,flush=True)
    return 0

if __name__=='__main__':sys.exit(main())
