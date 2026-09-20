"""Persistent independent CPU simulators, with unchanged per-candidate physics."""
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
from dataset_design import write_json

HERE=Path(__file__).resolve().parent


def partition(candidates,workers,batch_size):
    """Keep candidate IDs and complete clone batches; never resample per worker."""
    shards=[[] for _ in range(workers)]
    for start in range(0,len(candidates),batch_size):
        shards[(start//batch_size)%workers].extend(candidates[start:start+batch_size])
    return [s for s in shards if s]


def collect(root,shards,elapsed,statuses):
    rows=[];expected={p['candidate_id'] for s in shards for p in s}
    for index,shard in enumerate(shards):
        folder=root/f'worker_{index:02d}'
        allowed={p['candidate_id'] for p in shard}
        for path in sorted((folder/'results').glob('*/candidate.json')):
            row=json.loads(path.read_text())
            if row['candidate_id'] not in allowed:raise RuntimeError('Unexpected candidate in '+str(path))
            row['worker_dataset_root']=folder.name
            row['candidate_record_path']=str(path.relative_to(root))
            # Image/command references in the index resolve from the pool root.
            for key in ('observation_path','planned_command_file'):
                if row.get(key):row[key]=str(Path(folder.name)/row[key])
            rows.append(row)
    rows.sort(key=lambda r:r['candidate_id'])
    ids=[r['candidate_id'] for r in rows]
    if len(ids)!=len(set(ids)):raise RuntimeError('Duplicate candidate ID')
    counts={label:sum(r['result']==label for r in rows) for label in sorted({r['result'] for r in rows})}
    with (root/'candidates.jsonl').open('w') as f:
        for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')
    with (root/'candidates.csv').open('w',newline='') as f:
        fields=['candidate_id','target_id','result','hook_success','worker_dataset_root','candidate_record_path',
                'target_max_displacement_mm','main_stem_max_displacement_mm','first_contact_object']
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    summary=dict(requested_candidates=len(expected),recorded_candidates=len(rows),
        pending_candidates=len(expected)-len(rows),counts=counts,workers=statuses,elapsed_wall_s=elapsed,
        execution_complete=all(s=='complete' for s in statuses.values()),
        dataset_complete=len(rows)==len(expected) and not counts.get('incomplete') and all(s=='complete' for s in statuses.values()),
        physics='CPU, unchanged 960 Hz / TGS / 64 iterations; GPU renders RGB-D',
        parallelism='persistent independent simulator processes')
    write_json(root/'summary.json',summary)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=4,help='Independent simulators: 4 validated; 5-8 experimental, monitor RAM/VRAM and throughput')
    parser.add_argument('--num-envs',type=int,default=1,help='Clones per worker; start with 1')
    parser.add_argument('--candidates',type=int,default=100)
    parser.add_argument('--run-dir',type=Path)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--app-threads',type=int,default=8)
    args,extra=parser.parse_known_args()
    if not 1<=args.workers<=8:parser.error('workers must be 1..8; only up to 4 have completed full-candidate runtime validation')
    if args.num_envs not in (1,4):parser.error('num-envs must be 1 or 4')
    if not 1<=args.app_threads<=32:parser.error('app-threads must be 1..32')
    if any(flag.split('=')[0] in ('--gui','--validate-only','--benchmark-candidates','--rebuild','--profile') for flag in extra):
        parser.error('Pool does not accept GUI/validation/benchmark/rebuild/profile flags; use a small --candidates run')
    root=(args.run_dir or HERE/'runs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_candidate_dataset_pool')).resolve()
    if root.exists() and not args.resume:parser.error('Output exists; choose a new folder or --resume')
    root.mkdir(parents=True,exist_ok=True)
    template=root/'_template'
    command=[sys.executable,str(HERE/'candidate_dataset_runner.py'),'--prepare-only','--run-dir',str(template),
        '--num-envs',str(args.num_envs),'--candidates',str(args.candidates),*extra]
    if args.resume:command.append('--resume')
    subprocess.run(command,check=True)
    config=json.loads((template/'config.json').read_text())
    candidates=json.loads((template/'candidates.json').read_text())
    shards=partition(candidates,args.workers,args.num_envs)
    manifest=dict(workers=args.workers,active_workers=len(shards),num_envs=args.num_envs,app_threads=args.app_threads,
        candidate_sha256=hashlib.sha256((template/'candidates.json').read_bytes()).hexdigest(),
        assignments=[[p['candidate_id'] for p in shard] for shard in shards])
    manifest_path=root/'pool.json'
    if manifest_path.exists() and json.loads(manifest_path.read_text())!=manifest:
        parser.error('Resume worker count or assignments differ')
    write_json(manifest_path,manifest)
    for index,shard in enumerate(shards):
        folder=root/f'worker_{index:02d}';folder.mkdir(exist_ok=True)
        worker_config=dict(config,candidates=len(shard))
        for name,value in [('config.json',worker_config),('candidates.json',shard)]:
            path=folder/name
            if path.exists() and json.loads(path.read_text())!=value:raise RuntimeError('Worker configuration changed: '+str(path))
            write_json(path,value)
    print('[POOL]',root,'workers',len(shards),'clones per worker',args.num_envs,flush=True)
    if args.prepare_only:return
    # Shell TERM/HUP otherwise bypass Python's finally and orphan the workers,
    # which intentionally have separate process groups for targeted cleanup.
    def stop_requested(signum,frame):
        raise KeyboardInterrupt(f'Pool stop requested by signal {signum}')
    previous_handlers={s:signal.signal(s,stop_requested) for s in (signal.SIGTERM,signal.SIGHUP)}
    processes=[];statuses={str(i):'running' for i in range(len(shards))};began=time.monotonic()
    try:
        for index in range(len(shards)):
            folder=root/f'worker_{index:02d}'
            # Remove stale completion, retain atomic completed candidate records.
            for name in ('summary.json','execution_error.json'):(folder/name).unlink(missing_ok=True)
            log=(folder/'run.log').open('a')
            env=dict(os.environ,OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
            launch_unix=time.time()
            p=subprocess.Popen([sys.executable,'-u',str(HERE/'dataset_sim.py'),'--run-dir',str(folder),
                '--headless','--enable_cameras','--app-threads',str(args.app_threads)],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
            processes.append((p,folder,log))
            # Kit startup and cache initialization compete heavily when every
            # process starts together. Rollouts remain concurrent after startup.
            ready=folder/'source_sha256.json';launch_started=time.monotonic();startup_last=0.
            while not ready.exists() or ready.stat().st_mtime<launch_unix:
                if p.poll() is not None:raise RuntimeError('Worker exited during startup: '+str(folder/'run.log'))
                if time.monotonic()-launch_started>300:raise RuntimeError('Worker startup timed out: '+str(folder/'run.log'))
                if time.monotonic()-startup_last>=15:
                    print('[POOL STARTUP]',index,'waiting for Kit initialization',flush=True);startup_last=time.monotonic()
                time.sleep(.5)
        last=0.
        while True:
            for i,(p,folder,_) in enumerate(processes):
                if p.poll() is not None and statuses[str(i)]=='running':
                    path=folder/'summary.json'
                    report=json.loads(path.read_text()) if path.exists() else {}
                    ok=p.returncode==0 and report.get('execution_complete') and not (folder/'execution_error.json').exists()
                    statuses[str(i)]='complete' if ok else 'failed'
                    if not ok:raise RuntimeError('Worker failed: '+str(folder/'run.log'))
            if time.monotonic()-last>=15 or all(s=='complete' for s in statuses.values()):
                summary=collect(root,shards,time.monotonic()-began,statuses)
                print('[POOL PROGRESS]',json.dumps(summary),flush=True);last=time.monotonic()
            if all(s=='complete' for s in statuses.values()):break
            time.sleep(.5)
    finally:
        # A second terminal/timeout signal must not interrupt child cleanup.
        previous_int=signal.signal(signal.SIGINT,signal.SIG_IGN)
        for s in previous_handlers:signal.signal(s,signal.SIG_IGN)
        stopping=[]
        for i,(p,folder,log) in enumerate(processes):
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                stopping.append(p)
                statuses[str(i)]='interrupted'
        # One shared grace period, rather than 20 seconds per worker. This
        # finishes before the outer timeout's kill-after even with eight workers.
        deadline=time.monotonic()+20
        for p in stopping:
            try:p.wait(timeout=max(0.,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:os.killpg(p.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                p.wait()
        for _,_,log in processes:
            log.close()
        collect(root,shards,time.monotonic()-began,statuses)
        signal.signal(signal.SIGINT,previous_int)
        for s,handler in previous_handlers.items():signal.signal(s,handler)


if __name__=='__main__':main()
