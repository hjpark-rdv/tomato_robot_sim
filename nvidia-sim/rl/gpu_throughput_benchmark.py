"""Sequential full-workload GPU batch-size benchmark; never compare truncated trials.

Every configuration generates the same candidate list. Run one simulator at a
 time so they do not contend for the GPU. A faster run is eligible only if its
 commands, labels, contact identities and state traces match the baseline.
"""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
from dataset_design import write_json
from gpu_scale_validation import local_path

HERE=Path(__file__).resolve().parent


def process_identity(pid):
    try:
        start=(Path('/proc')/str(pid)/'stat').read_text().split(') ',1)[1].split()[19]
        return f'{pid}:{start}'
    except (OSError,IndexError):
        return None


def acquire_benchmark_lock(root):
    """Keep the returned handle alive; prevent concurrent resume/measurement."""
    import fcntl
    handle=(root/'benchmark.lock').open('a+')
    try:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        info=root/'controller.json'
        if info.exists():
            previous=json.loads(info.read_text())
            if previous['pid']!=os.getpid() and process_identity(previous['pid'])==previous['process_identity']:
                raise RuntimeError(f"Benchmark already running with PID {previous['pid']}")
        write_json(info,dict(pid=os.getpid(),process_identity=process_identity(os.getpid())))
        return handle
    except BaseException:
        handle.close()
        raise


def compare_datasets(reference, trial):
    reference,trial=Path(reference),Path(trial)
    failures=[];checks=[]
    if json.loads((reference/'candidates.json').read_text())!=json.loads((trial/'candidates.json').read_text()):
        return dict(passed=False,failures=['candidate_list'])
    if json.loads((reference/'source_sha256.json').read_text())!=json.loads((trial/'source_sha256.json').read_text()):
        return dict(passed=False,failures=['source_fingerprint'])
    ca=json.loads((reference/'config.json').read_text());cb=json.loads((trial/'config.json').read_text())
    for config in (ca,cb):
        for key in ('num_envs','observation_mode'):config.pop(key,None)
    if ca!=cb:return dict(passed=False,failures=['configuration'])
    expected=[p['candidate_id'] for p in json.loads((reference/'candidates.json').read_text())]
    for candidate_id in expected:
        a=reference/'results'/candidate_id;b=trial/'results'/candidate_id
        if not (a/'candidate.json').exists() or not (b/'candidate.json').exists():
            checks.append(dict(candidate_id=candidate_id,passed=False,failures=['missing_result']));continue
        ra=json.loads((a/'candidate.json').read_text());rb=json.loads((b/'candidate.json').read_text())
        bad=[];errors={}
        if ra.get('result')=='incomplete' or rb.get('result')=='incomplete':bad.append('incomplete')
        for key in ('parameters','result','executed','abort_reason','target_broken','other_broken','retained_hook','inserted','hook_success'):
            if ra.get(key)!=rb.get(key):bad.append(key)
        if local_path(ra.get('first_contact_object'))!=local_path(rb.get('first_contact_object')):bad.append('first_contact_object')
        if ra.get('executed') and rb.get('executed'):
            for name in ('planned_commands.npy',):
                if hashlib.sha256((a/name).read_bytes()).digest()!=hashlib.sha256((b/name).read_bytes()).digest():bad.append('commands')
            ta=json.loads((a/'trace.json').read_text());tb=json.loads((b/'trace.json').read_text())
            if not ta or len(ta)!=len(tb):bad.append('control_step_count')
            else:
                for key in ('joints','target_displacement_m','main_stem_displacement_m','gap_m'):
                    delta=np.asarray([x[key] for x in ta])-np.asarray([x[key] for x in tb])
                    value=float(np.max(np.abs(delta)))
                    errors[key]=value if np.isfinite(value) else None
                    if not np.isfinite(value) or value>1e-4:bad.append(key)
        checks.append(dict(candidate_id=candidate_id,passed=not bad,failures=bad,max_abs_errors=errors))
    return dict(passed=bool(checks) and all(c['passed'] for c in checks),checks=checks,
                tolerance_position_m=1e-4,tolerance_joint_rad=1e-4,
                scope='GPU batch-size consistency; not CPU equivalence or physical calibration')


def measure(run, wall_s):
    summary=json.loads((run/'summary.json').read_text())
    records=[json.loads(p.read_text()) for p in (run/'results').glob('*/candidate.json')]
    requested=len(json.loads((run/'candidates.json').read_text()))
    complete=bool(summary.get('execution_complete') and summary.get('dataset_complete') and
                  len(records)==requested and not any(r['result']=='incomplete' for r in records) and
                  not (run/'execution_error.json').exists())
    elapsed=summary['rollout_wall_s']
    physical=sum(r.get('executed',False) and r['result']!='incomplete' for r in records)
    return dict(num_envs=summary['num_envs'],complete=complete,requested=requested,recorded=len(records),
                completed_physics=physical,counts=summary['counts'],process_wall_s=wall_s,
                initialization_wall_s=summary['initialization_wall_s'],rollout_wall_s=elapsed,
                candidates_per_minute=60*len(records)/elapsed if complete and elapsed>0 else None,
                completed_physics_per_minute=60*physical/elapsed if complete and elapsed>0 else None,
                end_to_end_candidates_per_minute=60*len(records)/wall_s if complete and wall_s>0 else None,
                phase_timings=summary.get('phase_timings',{}),rollout_step_timings=summary.get('rollout_step_timings',{}),
                active_env_step_fraction=summary.get('active_env_step_fraction'))


def publish(root, rows, complete=False):
    eligible=[r for r in rows if r.get('complete') and r.get('comparison_passed')]
    best=max(eligible,key=lambda r:r['candidates_per_minute']) if complete and eligible else None
    report=dict(execution_complete=complete,runs=rows,best_num_envs=best['num_envs'] if best else None,
                selection_basis='same complete workload, matching GPU trajectories; highest candidates/minute excluding startup',
                scope='one measured fixed-scene workload, not a universal optimum or CPU-equivalence test',
                estimated_1000_minutes=(best['initialization_wall_s']/60+1000/best['candidates_per_minute']) if best else None,
                estimate_note='linear extrapolation of this workload; 1000 full trials not measured')
    write_json(root/'benchmark.json',report)
    columns=['num_envs','complete','comparison_passed','requested','recorded','completed_physics','process_wall_s',
             'initialization_wall_s','rollout_wall_s','candidates_per_minute','completed_physics_per_minute',
             'end_to_end_candidates_per_minute','active_env_step_fraction']
    with (root/'benchmark.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    lines=['# GPU 병렬 처리량 비교','','같은 후보·seed·물리 설정, 화면 없이 순차 실행. 미완료 결과는 처리량 순위에서 제외.','',
           '| 환경 수 | 완료 | 결과 일치 | 후보/분 (시작 제외) | 초기화 초 | 실행 초 | 활성 슬롯 비율 |',
           '|---:|:---:|:---:|---:|---:|---:|---:|']
    for r in rows:
        rate=r.get('candidates_per_minute');occupancy=r.get('active_env_step_fraction')
        lines.append(f"| {r['num_envs']} | {r.get('complete',False)} | {r.get('comparison_passed',False)} | {rate:.2f} | {r['initialization_wall_s']:.1f} | {r['rollout_wall_s']:.1f} | {occupancy:.1%} |" if rate is not None and occupancy is not None else f"| {r['num_envs']} | 미완료 | — | — | — | — | — |")
    if best:lines+=['',f"이번 작업량에서 가장 빠른 검증된 설정: **{best['num_envs']}환경**."]
    lines+=['','1,000후보 예상 시간은 단순 비례 추정이며 실측이 아니다. CPU TGS와 GPU PGS의 동등성은 별도 검증 대상이다.']
    (root/'benchmark.md').write_text('\n'.join(lines)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--env-counts',default='32,64,128')
    parser.add_argument('--candidates',type=int,default=128)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--target',default='Tomato_05')
    parser.add_argument('--run-dir',type=Path)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    try:counts=[int(v) for v in args.env_counts.split(',')]
    except ValueError:parser.error('env-counts must be comma-separated integers')
    if not counts or len(set(counts))!=len(counts) or any(n<1 or n>1024 for n in counts):parser.error('Invalid environment counts')
    if args.candidates<max(counts):parser.error('Use at least as many candidates as the largest batch; do not rank mostly idle batches')
    root=(args.run_dir or HERE/'runs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_gpu_throughput')).resolve()
    root.mkdir(parents=True,exist_ok=True)
    lock_handle=acquire_benchmark_lock(root)
    settings=dict(env_counts=counts,candidates=args.candidates,seed=args.seed,target=args.target,max_control_steps=0,gui=False)
    manifest=root/'experiment.json'
    if manifest.exists():
        if not args.resume or json.loads(manifest.read_text())!=settings:parser.error('Existing benchmark requires --resume with identical settings')
    else:write_json(manifest,settings)
    rows=[];baseline=None
    print('[GPU BENCH]',root,flush=True)
    def stop(signum,frame):raise KeyboardInterrupt
    for sig in (signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
    for count in counts:
        run=root/f'env_{count:04d}'
        timing=root/f'env_{count:04d}_timing.json'
        if timing.exists() and (run/'summary.json').exists():
            wall=json.loads(timing.read_text())['wall_s']
        else:
            # An interrupted partial worker is kept for inspection, not mixed
            # into an apparently fresh wall-time measurement on resume.
            if run.exists():run.rename(root/(run.name+'_interrupted_'+datetime.now().strftime('%H%M%S')))
            command=[sys.executable,'-u',str(HERE/'gpu_dataset_runner.py'),'--num-envs',str(count),
                     '--candidates',str(args.candidates),'--seed',str(args.seed),'--target',args.target,'--run-dir',str(run)]
            start=time.monotonic();last=start
            with (root/f'env_{count:04d}.console.log').open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    while process.poll() is None:
                        if time.monotonic()-last>=30:
                            progress=run/'progress.json'
                            status=json.loads(progress.read_text()) if progress.exists() else dict(stage='initialization / planning')
                            print('[GPU BENCH PROGRESS]',count,'envs',json.dumps(status),flush=True);last=time.monotonic()
                        time.sleep(.5)
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid,signal.SIGTERM)
                        try:process.wait(timeout=30)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid,signal.SIGKILL);process.wait()
                wall=time.monotonic()-start
            if process.returncode:
                write_json(root/'benchmark_error.json',dict(num_envs=count,returncode=process.returncode,run=str(run)))
                publish(root,rows);raise SystemExit('Worker failed; inspect '+str(run/'run.log'))
            write_json(timing,dict(wall_s=wall))
        row=measure(run,wall)
        comparison=compare_datasets(baseline or run,run)
        write_json(root/f'env_{count:04d}_comparison.json',comparison)
        row['comparison_passed']=comparison['passed'];row['run_dir']=str(run)
        rows.append(row);publish(root,rows)
        if baseline is None:baseline=run
        print('[GPU BENCH RESULT]',json.dumps(row),flush=True)
    report=publish(root,rows,complete=True)
    print('[GPU BENCH COMPLETE]',json.dumps(dict(best_num_envs=report['best_num_envs'],report=str(root/'benchmark.md'))),flush=True)


if __name__=='__main__':main()
