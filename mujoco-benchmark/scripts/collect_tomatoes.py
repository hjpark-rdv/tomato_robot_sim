"""Collect per-target physics and virtual RGB-D observations sequentially."""
import argparse,datetime,json,subprocess,sys,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--targets',default='1,2,3,4,6,7,8,9,10,11');p.add_argument('--candidates',type=int,default=1000);p.add_argument('--workers',type=int,default=48);p.add_argument('--planning-workers',type=int,default=16);p.add_argument('--output',type=Path);p.add_argument('--observation-limit',type=int);a=p.parse_args()
    targets=[int(x) for x in a.targets.split(',')]
    if len(set(targets))!=len(targets) or any(t<1 or t>11 for t in targets):p.error('targets must be unique numbers in 1..11')
    if min(a.candidates,a.workers,a.planning_workers)<1:p.error('positive counts required')
    root=(a.output or Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_multi_tomato_collection')).resolve();root.mkdir(parents=True,exist_ok=False)
    settings=dict(targets=targets,candidates_per_target=a.candidates,physics_workers=a.workers,planning_workers=a.planning_workers,seed=0,camera_mode='virtual D435: translate reference camera by target minus Tomato05 center, then existing view perturbations; robot stays at common initial state',status='running',completed=[])
    manifest=root/'collection.json';manifest.write_text(json.dumps(settings,indent=2));print('[수집 폴더]',root,flush=True);start=time.perf_counter()
    try:
        for index,t in enumerate(targets):
            target=f'Tomato_{t:02d}';folder=root/target;folder.mkdir();physics=folder/'physics';observations=folder/'observations'
            print('[토마토 시작]',index+1,'/',len(targets),target,flush=True)
            steps=[('physics',[sys.executable,str(Path(__file__).with_name('candidate_experiment.py')),'--target',target,'--candidates',str(a.candidates),'--workers',str(a.workers),'--planning-workers',str(a.planning_workers),'--output',str(physics)]),('observations',[sys.executable,str(Path(__file__).with_name('prepare_observations.py')),str(physics),'--output',str(observations)])]
            if a.observation_limit:steps[1][1].extend(['--limit',str(a.observation_limit)])
            for phase,command in steps:
                with (folder/f'{phase}.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
                print('[단계 완료]',target,phase,flush=True)
            result=json.loads((physics/'results.json').read_text());meta=json.loads((observations/'dataset.json').read_text())
            if len(result)!=a.candidates or meta['target']!=target:raise RuntimeError('Wrong target/count')
            settings['completed'].append(dict(target=target,candidates=len(result),observations=meta['observations'],planned_actions=meta['pose_label_actions']));manifest.write_text(json.dumps(settings,indent=2))
        settings['status']='complete'
    except BaseException as error:
        settings['status']='interrupted_or_failed';settings['error']=repr(error);raise
    finally:
        settings['wall_s']=time.perf_counter()-start;manifest.write_text(json.dumps(settings,indent=2))
        links=''.join(f'<li>{r["target"]}: <a href="{r["target"]}/physics/index.html">경로 결과</a> · <a href="{r["target"]}/observations/index.html">RGB-D</a> ({r["candidates"]}회 / {r["observations"]}시점)</li>' for r in settings['completed'])
        (root/'index.html').write_text('<meta charset="utf-8"><h1>토마토별 데이터 수집</h1><p>가상 카메라 관측이며 로봇은 공통 초기 상태입니다.</p><ul>'+links+'</ul><a href="collection.json">진행 상태</a>')
    print('[전체 수집 완료]',root/'index.html',flush=True)
if __name__=='__main__':main()
