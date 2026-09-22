"""기존 Isaac의 Sobol/IK/FCL 계획 코드를 시뮬레이터 없이 실행."""
import os
for key in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'):os.environ[key]='1'
import argparse,json,pickle,sys,time
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'nvidia-sim/rl'))

def initialize(root):
 global MODEL,OUT,START
 import torch
 from gpu_planning import restore_model
 torch.set_num_threads(1);OUT=Path(root)
 model=pickle.loads((OUT/'replay_assets/planning_model.pkl').read_bytes())
 values=json.loads((OUT/'planning_inputs.json').read_text())
 model['inputs'].update({k:np.asarray(v) if k in ('start','geometry') else v for k,v in values['inputs'].items()})
 if not np.allclose(model['kinematics']['world'],values['world'],atol=1e-6,rtol=0):raise RuntimeError('Exported collision model world differs from MuJoCo; re-export required')
 model['kinematics']['world']=np.array(values['world'])
 # Source robot transform must agree with the actual MuJoCo robot.
 MODEL=restore_model(model);START=np.array(values['inputs']['start'])
 expected=np.array(values['ring_position']);actual=MODEL[1].fk(START)[0]
 if np.linalg.norm(expected-actual)>1e-5:raise RuntimeError('MuJoCo/planner ring frame mismatch')

def compute(params):
 from dataset_motion import plan
 begin=time.perf_counter();folder=OUT/'candidates'/params['candidate_id'];folder.mkdir(parents=True)
 planned,check=plan(*MODEL,params)
 info={'candidate_id':params['candidate_id'],'parameters':params,'preflight':check,'planning_wall_s':time.perf_counter()-begin}
 if planned is not None:
  rows=[{'joints':START.tolist(),'command':START.tolist(),'phase':'ready'}]
  rows.extend({'joints':START.tolist(),'command':q.tolist(),'phase':phase} for q,phase in zip(planned['commands'],planned['phases']))
  (folder/'trace.json').write_text(json.dumps(rows))
  info.update(waypoints=planned['waypoints'],seconds=(len(rows)-1)/60)
 (folder/'plan.json').write_text(json.dumps(info,indent=2));return info

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--workers',type=int,default=4);a=p.parse_args()
 params=json.loads((a.root/'candidates.json').read_text())
 with ProcessPoolExecutor(max_workers=a.workers,initializer=initialize,initargs=(str(a.root),)) as pool:
  for info in pool.map(compute,params):print('[경로 계획]',info['candidate_id'],info['preflight'],flush=True)
if __name__=='__main__':main()
