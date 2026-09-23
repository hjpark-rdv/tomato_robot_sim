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
from trajectory_search import candidates,center_region,BOUNDS,SAMPLING
from suite import RING


CLASSIFICATION_RULE='center_entry_only_v2'

def classify_result(unstable,entry,displacement,rule=CLASSIFICATION_RULE):
 if unstable:return 'invalid_physics'
 if rule==CLASSIFICATION_RULE:return 'partial_center_entry' if entry else 'miss'
 if rule!='displacement_first_v1':raise ValueError('Unknown classification rule: '+rule)
 return 'excessive_displacement' if displacement>.020 else 'partial_center_entry' if entry else 'miss'

def initialize(root):
 global ENGINE,RUN,RULE
 RUN=Path(root);RULE=json.loads((RUN/'manifest.json').read_text()).get('classification_rule','displacement_first_v1');ENGINE=RobotEngine(RUN/'replay_assets/model.mjb',RUN/'replay_assets/initial_trace.json',json.loads((RUN/'manifest.json').read_text())['hz'],reference=RUN/'replay_assets/reference.json',target=json.loads((RUN/'manifest.json').read_text()).get('target','Tomato_05'))


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
 spec=e.target_spec;fruit_index=next(i for i,b in enumerate(e.ref['bodies']) if b['name']==e.target)
 fp=arrays['poses'][:,fruit_index];hp=arrays['poses'][:,-1]
 centers=fp[:,:3]+Rotation.from_quat(fp[:,3:]).apply(spec['center'])
 rings=hp[:,:3]+Rotation.from_quat(hp[:,3:]).apply(np.broadcast_to(RING,(len(hp),3)).copy())
 local=Rotation.from_quat(hp[:,3:]).inv().apply(centers-rings)
 entry_mask=np.array([center_region(v,spec['radius']) for v in local],dtype=bool)
 entry=not bool(entry_mask[0]) and bool(entry_mask[1:].any())
 displacement=float(np.linalg.norm(centers-centers[0],axis=1).max())
 label=classify_result(r['unstable'] or not r.get('glb_physics_valid',True),entry,displacement,RULE)
 path_by_name={shape['name']:shape['path'] for shape in e.ref['shapes']}
 first=next(({'step':i+1,'objects':pair,'prim_paths':[path_by_name.get(name,name) for name in pair]} for i,pairs in enumerate(contacts) for pair in pairs),None)
 result=dict(info,result=label,classification_rule=RULE,partial_entry_success=label=='partial_center_entry',target_displacement_exceeded=displacement>.020,center_entered=entry,target_center_max_displacement_m=displacement,first_contact_candidate=first,metrics=r,execution_and_save_wall_s=time.perf_counter()-started,training_eligible=label not in ('invalid_physics','execution_error'),hook_success=None,scope='partial geometry/contact diagnostics; no validated hook-success evaluator')
 hits=np.flatnonzero(entry_mask)
 result['center_entry_diagnostics']=dict(first_time_s=float(arrays['times_s'][hits[0]]) if len(hits) else None,last_time_s=float(arrays['times_s'][hits[-1]]) if len(hits) else None,matching_steps=int(entry_mask.sum()),scope='geometric occupancy only; physics validity takes precedence')
 result['state_storage']=dict(format='qpos_only',max_fps=30,diagnostics_hz=1/e.model.opt.timestep,final_frame_preserved=True)
 result['trace_sha256']=hashlib.sha256((folder/'trace.json').read_bytes()).hexdigest()
 result['states_sha256']=hashlib.sha256((folder/'states.npz').read_bytes()).hexdigest()
 (folder/'result.json').write_text(json.dumps(result,indent=2));return result


def report(root,results):
 temporary=root/'results.tmp.json';temporary.write_text(json.dumps(results,indent=2));temporary.replace(root/'results.json')
 with (root/'results.csv').open('w') as f:
  fields=['candidate_id','result','center_entered','target_center_max_displacement_m','planning_wall_s','classification_rule','partial_entry_success','target_displacement_exceeded'];w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(results)
 from camera_action import FEATURE_NAMES
 with (root/'camera_actions.csv').open('w') as f:
  w=csv.writer(f);w.writerow(['candidate_id',*FEATURE_NAMES,'center_entered','target_max_displacement_m','result'])
  for r in sorted(results,key=lambda r:r['candidate_id']):
   if 'action_camera' in r:w.writerow([r['candidate_id'],*r['action_camera']['features'],r.get('center_entered'),r.get('target_center_max_displacement_m'),r['result']])
 from candidate_report import generate
 generate(root,results)


def main():
 p=argparse.ArgumentParser();p.add_argument('--target',default='Tomato_05');p.add_argument('--candidates',type=int,default=8);p.add_argument('--workers',type=int,default=4);p.add_argument('--planning-workers',type=int,default=4);p.add_argument('--seed',type=int,default=0);p.add_argument('--hz',type=int,default=120);p.add_argument('--output',type=Path);p.add_argument('--planning-python',default='/root/isaaclab_env/bin/python');p.add_argument('--planning-model',type=Path,default=ROOT/'nvidia-sim/rl/runs/20260922_054612_gpu_env8_matched12/planning_20260922_054636_450257/model.pkl');p.add_argument('--model',type=Path,default=DEFAULT_MODEL);p.add_argument('--reference',type=Path,default=HOME/'assets/reference/reference.json');p.add_argument('--plan-only',action='store_true',help='save the same IK/FCL candidate plans without running physics');p.add_argument('--link-model',action='store_true',help='hard-link immutable scene model into the run on the same filesystem');a=p.parse_args()
 if min(a.candidates,a.workers,a.planning_workers,a.hz)<1:p.error('counts/hz must be positive')
 if a.target not in {s['name'] for s in json.loads(a.reference.read_text())['fruit_specs']}:
  p.error(f'target {a.target!r} is absent from {a.reference}')
 root=(a.output or Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_candidate_search')).resolve();root.mkdir(parents=True,exist_ok=False);assets=root/'replay_assets';assets.mkdir();began=time.perf_counter()
 for src,name in ((a.model,'model.mjb'),(DEFAULT_TRACE,'initial_trace.json'),(a.reference,'reference.json'),(a.planning_model,'planning_model.pkl')):
  if name=='model.mjb' and a.link_model:
   try:os.link(src,assets/name)
   except OSError:shutil.copy2(src,assets/name)
  else:shutil.copy2(src,assets/name)
 e=RobotEngine(assets/'model.mjb',assets/'initial_trace.json',a.hz,reference=assets/'reference.json',target=a.target);spec=e.target_spec;rot=e.data.xmat[e.fruit].reshape(3,3);anchor=e.data.xpos[e.fruit];center=anchor+rot@np.array(spec['center']);neck=anchor+rot@np.array(spec['neck'])
 inputs=dict(inputs=dict(geometry=[center.tolist(),neck.tolist(),(rot@np.array(spec['axis'])).tolist()],start=e.initial.tolist(),step_dt=1/60,lift_id=0,rise_speed=.002,pull_speed=.004,target_radius=spec['radius']),world=e.ref['robot_fk']['world'],ring_position=(e.data.xpos[e.hook]+e.data.xmat[e.hook].reshape(3,3)@RING).tolist())
 from camera_action import optical_transform,SCHEMA,FEATURE_NAMES
 hook_pose=np.eye(4);hook_pose[:3,:3]=e.data.xmat[e.hook].reshape(3,3);hook_pose[:3,3]=e.data.xpos[e.hook]
 camera_pose=hook_pose@optical_transform('color')
 frame=dict(schema=SCHEMA,observation='initial nominal D435',world_from_color_optical=camera_pose.tolist(),target_center_world=center.tolist(),gravity_direction_camera=(camera_pose[:3,:3].T@np.array([0.,0.,-1.])).tolist(),feature_names=FEATURE_NAMES)
 (root/'action_frame.json').write_text(json.dumps(frame,indent=2))
 (root/'planning_inputs.json').write_text(json.dumps(inputs));(root/'candidates.json').write_text(json.dumps(candidates(a.candidates,a.seed),indent=2))
 import mujoco,psutil
 shutil.copytree(HOME/'scripts',assets/'scripts',ignore=shutil.ignore_patterns('__pycache__'))
 legacy=assets/'isaac_planner_sources';legacy.mkdir()
 for source in (ROOT/'nvidia-sim/rl').glob('*.py'):shutil.copy2(source,legacy/source.name)
 manifest=dict(classification_rule=CLASSIFICATION_RULE,sampling=SAMPLING,bounds=BOUNDS,hz=a.hz,workers=a.workers,planning_workers=a.planning_workers,seed=a.seed,count=a.candidates,mujoco=mujoco.__version__,target=a.target,model_sha256=hashlib.sha256((assets/'model.mjb').read_bytes()).hexdigest(),physics='optimized plant; break disabled' if a.model.resolve()==DEFAULT_MODEL.resolve() else 'custom supplied model; break/validity require independent validation',scope='Sobol staged6d, existing IK/FCL planner; no physics rollout or labels' if a.plan_only else 'Sobol staged6d, existing IK/FCL planner, CPU MuJoCo execution; no RL/perception/randomization',plan_only=a.plan_only,arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()})
 manifest['action_schema']=SCHEMA
 manifest['action_frame']='action_frame.json'
 manifest['legacy_parameters']='world-axis Sobol parameters for reproduction; use action_camera.features for model input'
 manifest['hardware']=dict(logical_cpus=psutil.cpu_count(),physical_cpus=psutil.cpu_count(logical=False),ram_bytes=psutil.virtual_memory().total)
 manifest['asset_hashes']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in assets.iterdir() if p.is_file()}
 (root/'manifest.json').write_text(json.dumps(manifest,indent=2));print('[경로 실험]',root,flush=True)
 subprocess.run([a.planning_python,str(HOME/'scripts/plan_candidates.py'),str(root),'--workers',str(min(a.planning_workers,a.candidates))],check=True)
 if a.plan_only:
  results=[]
  for c in json.loads((root/'candidates.json').read_text()):
   folder=root/'candidates'/c['candidate_id'];plan=json.loads((folder/'plan.json').read_text())
   row=dict(plan,result='not_executed' if plan['preflight'].get('passed') else 'ik_or_planning_failure',training_eligible=False,hook_success=None,scope='planned candidate only; no physics label')
   (folder/'result.json').write_text(json.dumps(row,indent=2));results.append(row)
  manifest['total_wall_s']=time.perf_counter()-began;(root/'manifest.json').write_text(json.dumps(manifest,indent=2));report(root,results)
  print('[계획만 완료]',root/'index.html',flush=True)
  return
 results=[];last_report=0.
 with ProcessPoolExecutor(mp_context=multiprocessing.get_context('spawn'),max_workers=min(a.workers,a.candidates),initializer=initialize,initargs=(str(root),)) as pool:
  futures={pool.submit(execute,c):c for c in json.loads((root/'candidates.json').read_text())}
  for future in as_completed(futures):
   try:r=future.result()
   except Exception as error:
    r=dict(candidate_id=futures[future]['candidate_id'],parameters=futures[future],result='execution_error',error=repr(error))
   results.append(r);folder=root/'candidates'/r['candidate_id'];(folder/'result.json').write_text(json.dumps(r,indent=2))
   if time.perf_counter()-last_report>=5:
    report(root,results);last_report=time.perf_counter()
   print('[후보 완료]',r['candidate_id'],r['result'],flush=True)
 manifest['total_wall_s']=time.perf_counter()-began;(root/'manifest.json').write_text(json.dumps(manifest,indent=2));report(root,results);print('[완료]',root/'index.html',manifest['total_wall_s'],flush=True)
if __name__=='__main__':main()
