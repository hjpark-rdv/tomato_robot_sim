"""Resume only missing/corrupt results, preserving valid plans and physical states."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import argparse,json,hashlib,time,subprocess,sys,multiprocessing
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
from candidate_experiment import initialize,execute,report


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def valid_result(root,candidate):
    folder=root/'candidates'/candidate['candidate_id']
    try:
        r=json.loads((folder/'result.json').read_text())
        if r['candidate_id']!=candidate['candidate_id'] or r['parameters']!=candidate:return None
        if r['result']=='ik_or_planning_failure':return r if not r['preflight']['passed'] else None
        if r['result'] not in ('partial_center_entry','miss','excessive_displacement','invalid_physics'):return None
        for name,key in [('states.npz','states_sha256'),('trace.json','trace_sha256')]:
            if sha(folder/name)!=r[key]:return None
        if not (folder/'contacts.json').is_file():return None
        return r
    except (OSError,ValueError,KeyError):return None


def resume(root,workers=48,planning_workers=16,inspect_only=False):
    import mujoco,psutil
    root=root.resolve();m=json.loads((root/'manifest.json').read_text());assets=root/'replay_assets';params=json.loads((root/'candidates.json').read_text())
    if len(params)!=m['count'] or mujoco.__version__!=m['mujoco']:raise RuntimeError('Count or physics version mismatch')
    for name,h in m['asset_hashes'].items():
        if sha(assets/name)!=h:raise RuntimeError('Changed asset: '+name)
    # Prevent simultaneous resume processes for this run.
    import fcntl
    lock=(root/'resume.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    records=[valid_result(root,c) for c in params];done=[r for r in records if r];pending=[c for c,r in zip(params,records) if r is None]
    print('[재개 검사] 완료',len(done),'남음',len(pending),flush=True)
    if inspect_only:return dict(completed=len(done),pending=len(pending))
    # Only abandoned workers whose stdout is this exact run's log can conflict.
    stopped=[]
    for proc in psutil.process_iter(['pid','ppid','cmdline']):
        try:
            if proc.info['ppid']==1 and 'multiprocessing.spawn' in ' '.join(proc.info['cmdline'] or []):
                if Path(os.readlink(f'/proc/{proc.pid}/fd/1')).resolve()==(root.parent/'physics.log').resolve():proc.terminate();stopped.append(proc)
        except (OSError,psutil.Error):pass
    _,alive=psutil.wait_procs(stopped,timeout=3)
    for proc in alive:proc.kill()
    psutil.wait_procs(alive,timeout=3)
    start=time.perf_counter();report(root,done)
    if pending:
        planning_python=m.get('arguments',{}).get('planning_python','/root/isaaclab_env/bin/python')
        subprocess.run([planning_python,str(Path(__file__).with_name('plan_candidates.py')),str(root),'--workers',str(planning_workers),'--resume'],check=True)
        with ProcessPoolExecutor(max_workers=min(workers,len(pending)),mp_context=multiprocessing.get_context('spawn'),initializer=initialize,initargs=(str(root),)) as pool:
            fs={pool.submit(execute,c):c for c in pending};last=0.
            for f in as_completed(fs):
                r=f.result();done.append(r);folder=root/'candidates'/r['candidate_id'];tmp=folder/'result.tmp.json';tmp.write_text(json.dumps(r,indent=2));tmp.replace(folder/'result.json')
                if time.perf_counter()-last>=5:report(root,done);last=time.perf_counter()
                print('[재개 완료]',r['candidate_id'],len(done),'/',len(params),flush=True)
    report(root,done);m.setdefault('resume_sessions',[]).append(dict(wall_s=time.perf_counter()-start,reused=len(records)-len(pending),executed=len(pending),orphan_workers_stopped=[p.pid for p in stopped]));m['execution_complete']=True;(root/'manifest.json').write_text(json.dumps(m,indent=2))
    return dict(completed=len(done),pending=0)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--workers',type=int,default=48);p.add_argument('--planning-workers',type=int,default=16);p.add_argument('--inspect-only',action='store_true');a=p.parse_args();print(resume(a.run,a.workers,a.planning_workers,a.inspect_only))
