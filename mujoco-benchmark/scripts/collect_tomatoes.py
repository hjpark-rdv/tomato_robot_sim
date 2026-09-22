"""Collect per-target physics and virtual RGB-D observations sequentially."""
import argparse,datetime,json,subprocess,sys,time,os,signal,hashlib
from pathlib import Path


def main():
    def run_step(command,log):
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            if child.wait():raise subprocess.CalledProcessError(child.returncode,command)
        except BaseException:
            try:os.killpg(child.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(child.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                child.wait()
            raise
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--targets',default='1,2,3,4,6,7,8,9,10,11');p.add_argument('--candidates',type=int,default=1000);p.add_argument('--workers',type=int,default=48);p.add_argument('--planning-workers',type=int,default=16);p.add_argument('--output',type=Path);p.add_argument('--observation-limit',type=int);p.add_argument('--postprocess-workers',type=int,default=8);p.add_argument('--resume',action='store_true');a=p.parse_args()
    targets=[int(x) for x in a.targets.split(',')]
    if len(set(targets))!=len(targets) or any(t<1 or t>11 for t in targets):p.error('targets must be unique numbers in 1..11')
    if min(a.candidates,a.workers,a.planning_workers,a.postprocess_workers)<1:p.error('positive counts required')
    root=(a.output or Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_multi_tomato_collection')).resolve();root.mkdir(parents=True,exist_ok=a.resume)
    import fcntl
    collection_lock=(root/'collection.lock').open('a');fcntl.flock(collection_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    settings=dict(targets=targets,candidates_per_target=a.candidates,physics_workers=a.workers,planning_workers=a.planning_workers,seed=0,camera_mode='virtual D435: translate reference camera by target minus Tomato05 center, then existing view perturbations; robot stays at common initial state',status='running',completed=[])
    manifest=root/'collection.json'
    def save_settings():
        tmp=root/'collection.tmp.json';tmp.write_text(json.dumps(settings,indent=2));tmp.replace(manifest)
    if a.resume:
        old=json.loads(manifest.read_text())
        if old['targets']!=targets or old['candidates_per_target']!=a.candidates:raise ValueError('Resume targets/count must match original collection')
        settings['completed']=old['completed'];settings['previous_wall_s']=old.get('wall_s',0)
    settings['postprocess_workers']=a.postprocess_workers
    save_settings();print('[수집 폴더]',root,flush=True);start=time.perf_counter()
    try:
        for index,t in enumerate(targets):
            target=f'Tomato_{t:02d}';folder=root/target;folder.mkdir(exist_ok=a.resume);physics=folder/'physics';observations=folder/'observations'
            if a.resume:
                try:
                    meta=json.loads((observations/'dataset.json').read_text());saved_results=json.loads((physics/'results.json').read_text())
                    complete=meta.get('schema')=='farmily_observation_v2' and meta.get('source_results_sha256')==hashlib.sha256((physics/'results.json').read_bytes()).hexdigest() and len(saved_results)==a.candidates and meta.get('target')==target
                except (OSError,ValueError):complete=False
                if complete:
                    if not any(r['target']==target for r in settings['completed']):settings['completed'].append(dict(target=target,candidates=a.candidates,observations=meta['observations'],planned_actions=meta['pose_label_actions']))
                    print('[완료 대상 건너뜀]',target,flush=True);continue
            print('[토마토 시작]' ,index+1,'/',len(targets),target,flush=True)
            steps=[('physics',[sys.executable,str(Path(__file__).with_name('candidate_experiment.py')),'--target',target,'--candidates',str(a.candidates),'--workers',str(a.workers),'--planning-workers',str(a.planning_workers),'--output',str(physics)]),('observations',[sys.executable,str(Path(__file__).with_name('prepare_observations.py')),str(physics),'--output',str(observations),'--postprocess-workers',str(a.postprocess_workers)])]
            if a.resume and physics.exists():
                saved=json.loads((physics/'manifest.json').read_text())
                if saved['target']!=target or saved['count']!=a.candidates:raise ValueError('Resume physics target/count mismatch')
                steps[0]=('physics',[sys.executable,str(Path(__file__).with_name('resume_candidates.py')),str(physics),'--workers',str(a.workers),'--planning-workers',str(a.planning_workers)])
            if a.resume and observations.exists():
                observations.rename(folder/('observations_interrupted_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
            if a.observation_limit:steps[1][1].extend(['--limit',str(a.observation_limit)])
            for phase,command in steps:
                with (folder/f'{phase}.log').open('a' if a.resume else 'w') as log:run_step(command,log)
                print('[단계 완료]',target,phase,flush=True)
            result=json.loads((physics/'results.json').read_text());meta=json.loads((observations/'dataset.json').read_text())
            if len(result)!=a.candidates or meta['target']!=target:raise RuntimeError('Wrong target/count')
            settings['completed']=[r for r in settings['completed'] if r['target']!=target]
            settings['completed'].append(dict(target=target,candidates=len(result),observations=meta['observations'],planned_actions=meta['pose_label_actions']));save_settings()
        settings['status']='complete'
    except BaseException as error:
        settings['status']='interrupted_or_failed';settings['error']=repr(error);raise
    finally:
        settings['wall_s']=time.perf_counter()-start;save_settings()
        links=''.join(f'<li>{r["target"]}: <a href="{r["target"]}/physics/index.html">경로 결과</a> · <a href="{r["target"]}/observations/index.html">RGB-D</a> ({r["candidates"]}회 / {r["observations"]}시점)</li>' for r in settings['completed'])
        (root/'index.html').write_text('<meta charset="utf-8"><h1>토마토별 데이터 수집</h1><p>가상 카메라 관측이며 로봇은 공통 초기 상태입니다.</p><ul>'+links+'</ul><a href="collection.json">진행 상태</a>')
    print('[전체 수집 완료]',root/'index.html',flush=True)
if __name__=='__main__':main()
