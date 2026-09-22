"""고정 장면 Sobol 경로 생성 → CPU 물리 실행 → 상태/진단/재연 데이터."""
import os
for key in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'):os.environ[key]='1'
import argparse,datetime,json,hashlib,shutil,subprocess,sys,time,csv,html
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
import multiprocessing
import numpy as np
from robot_engine import RobotEngine,HOME,DEFAULT_MODEL,DEFAULT_TRACE
ROOT=HOME.parent
sys.path.insert(0,str(ROOT/'nvidia-sim/rl'))
from trajectory_search import candidates,center_region
from suite import RING


def initialize(root):
 global ENGINE,RUN
 RUN=Path(root);ENGINE=RobotEngine(RUN/'replay_assets/model.mjb',RUN/'replay_assets/initial_trace.json',json.loads((RUN/'manifest.json').read_text())['hz'],reference=RUN/'replay_assets/reference.json')


def execute(candidate):
 folder=RUN/'candidates'/candidate['candidate_id'];info=json.loads((folder/'plan.json').read_text())
 if not info['preflight'].get('passed'):return dict(info,result='ik_or_planning_failure')
 e=ENGINE;rows=json.loads((folder/'trace.json').read_text());e.commands=np.array([r['command'] for r in rows]);e.ts=np.arange(len(rows))/60
 started=time.perf_counter();r=e.rollout(seconds=info['seconds'],record=True);arrays=r.pop('_arrays');contacts=r.pop('_contacts')
 from replay_storage import compact_states
 np.savez_compressed(folder/'states.npz',**compact_states(arrays,30))
 (folder/'contacts.json').write_text(json.dumps(contacts))
 # Partial entry is geometric progress, never a certified pedicel hook.
 from scipy.spatial.transform import Rotation
 spec=e.ref['fruit_specs'][4];fruit_index=next(i for i,b in enumerate(e.ref['bodies']) if b['name']=='Tomato_05')
 fp=arrays['poses'][:,fruit_index];hp=arrays['poses'][:,-1]
 centers=fp[:,:3]+Rotation.from_quat(fp[:,3:]).apply(spec['center'])
 rings=hp[:,:3]+Rotation.from_quat(hp[:,3:]).apply(np.broadcast_to(RING,(len(hp),3)).copy())
 local=Rotation.from_quat(hp[:,3:]).inv().apply(centers-rings)
 entry=not center_region(local[0],spec['radius']) and any(center_region(v,spec['radius']) for v in local[1:])
 displacement=float(np.linalg.norm(centers-centers[0],axis=1).max())
 label='invalid_physics' if r['unstable'] else 'excessive_displacement' if displacement>.020 else 'partial_center_entry' if entry else 'miss'
 path_by_name={shape['name']:shape['path'] for shape in e.ref['shapes']}
 first=next(({'step':i+1,'objects':pair,'prim_paths':[path_by_name.get(name,name) for name in pair]} for i,pairs in enumerate(contacts) for pair in pairs),None)
 result=dict(info,result=label,center_entered=entry,target_center_max_displacement_m=displacement,first_contact_candidate=first,metrics=r,execution_and_save_wall_s=time.perf_counter()-started,hook_success=None,scope='partial geometry/contact diagnostics; no validated hook-success evaluator')
 result['state_storage']=dict(format='qpos_only',max_fps=30,diagnostics_hz=1/e.model.opt.timestep,final_frame_preserved=True)
 result['trace_sha256']=hashlib.sha256((folder/'trace.json').read_bytes()).hexdigest()
 result['states_sha256']=hashlib.sha256((folder/'states.npz').read_bytes()).hexdigest()
 (folder/'result.json').write_text(json.dumps(result,indent=2));return result


def report(root,results):
 (root/'results.json').write_text(json.dumps(results,indent=2))
 with (root/'results.csv').open('w') as f:
  fields=['candidate_id','result','center_entered','target_center_max_displacement_m','planning_wall_s'];w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(results)
 from candidate_report import generate
 generate(root,results)


def main():
 p=argparse.ArgumentParser();p.add_argument('--candidates',type=int,default=8);p.add_argument('--workers',type=int,default=4);p.add_argument('--planning-workers',type=int,default=4);p.add_argument('--seed',type=int,default=0);p.add_argument('--hz',type=int,default=120);p.add_argument('--output',type=Path);p.add_argument('--planning-python',default='/root/isaaclab_env/bin/python');p.add_argument('--planning-model',type=Path,default=ROOT/'nvidia-sim/rl/runs/20260922_054612_gpu_env8_matched12/planning_20260922_054636_450257/model.pkl');a=p.parse_args()
 if min(a.candidates,a.workers,a.planning_workers,a.hz)<1:p.error('counts/hz must be positive')
 root=(a.output or Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_candidate_search')).resolve();root.mkdir(parents=True,exist_ok=False);assets=root/'replay_assets';assets.mkdir();began=time.perf_counter()
 for src,name in ((DEFAULT_MODEL,'model.mjb'),(DEFAULT_TRACE,'initial_trace.json'),(HOME/'assets/reference/reference.json','reference.json'),(a.planning_model,'planning_model.pkl')):shutil.copy2(src,assets/name)
 e=RobotEngine(reference=assets/'reference.json');spec=e.ref['fruit_specs'][4];rot=e.data.xmat[e.fruit].reshape(3,3);anchor=e.data.xpos[e.fruit];center=anchor+rot@np.array(spec['center']);neck=anchor+rot@np.array(spec['neck'])
 inputs=dict(inputs=dict(geometry=[center.tolist(),neck.tolist(),(rot@np.array(spec['axis'])).tolist()],start=e.initial.tolist(),step_dt=1/60,lift_id=0,rise_speed=.002,pull_speed=.004,target_radius=spec['radius']),world=e.ref['robot_fk']['world'],ring_position=(e.data.xpos[e.hook]+e.data.xmat[e.hook].reshape(3,3)@RING).tolist())
 (root/'planning_inputs.json').write_text(json.dumps(inputs));(root/'candidates.json').write_text(json.dumps(candidates(a.candidates,a.seed),indent=2))
 import mujoco,psutil
 shutil.copytree(HOME/'scripts',assets/'scripts',ignore=shutil.ignore_patterns('__pycache__'))
 legacy=assets/'isaac_planner_sources';legacy.mkdir()
 for source in (ROOT/'nvidia-sim/rl').glob('*.py'):shutil.copy2(source,legacy/source.name)
 manifest=dict(hz=a.hz,workers=a.workers,planning_workers=a.planning_workers,seed=a.seed,count=a.candidates,mujoco=mujoco.__version__,target='Tomato_05',model_sha256=hashlib.sha256((assets/'model.mjb').read_bytes()).hexdigest(),physics='optimized plant; break disabled',scope='Sobol staged6d, existing IK/FCL planner, CPU MuJoCo execution; no RL/perception/randomization',arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()})
 manifest['hardware']=dict(logical_cpus=psutil.cpu_count(),physical_cpus=psutil.cpu_count(logical=False),ram_bytes=psutil.virtual_memory().total)
 manifest['asset_hashes']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in assets.iterdir() if p.is_file()}
 (root/'manifest.json').write_text(json.dumps(manifest,indent=2));print('[경로 실험]',root,flush=True)
 subprocess.run([a.planning_python,str(HOME/'scripts/plan_candidates.py'),str(root),'--workers',str(min(a.planning_workers,a.candidates))],check=True)
 results=[]
 with ProcessPoolExecutor(mp_context=multiprocessing.get_context('spawn'),max_workers=min(a.workers,a.candidates),initializer=initialize,initargs=(str(root),)) as pool:
  futures={pool.submit(execute,c):c for c in json.loads((root/'candidates.json').read_text())}
  for future in as_completed(futures):
   try:r=future.result()
   except Exception as error:
    r=dict(candidate_id=futures[future]['candidate_id'],parameters=futures[future],result='execution_error',error=repr(error))
   results.append(r);folder=root/'candidates'/r['candidate_id'];(folder/'result.json').write_text(json.dumps(r,indent=2));report(root,results);print('[후보 완료]',r['candidate_id'],r['result'],flush=True)
 manifest['total_wall_s']=time.perf_counter()-began;(root/'manifest.json').write_text(json.dumps(manifest,indent=2));report(root,results);print('[완료]',root/'index.html',manifest['total_wall_s'],flush=True)
if __name__=='__main__':main()
