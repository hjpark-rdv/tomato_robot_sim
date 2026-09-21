"""Read saved GPU trajectories and compare to one version-matched CPU replay."""
import argparse,json,time
from pathlib import Path
import numpy as np
import mujoco as mj
from robot_engine import RobotEngine,HOME

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 e=RobotEngine(HOME/'models/robot_plant_mjlab.mjb');cpu=e.rollout(seconds=16.5,record=True);arrays=cpu.pop('_arrays');cpu.pop('_contacts');np.savez_compressed(a.output/'cpu_reference.npz',**arrays);(a.output/'cpu_metrics.json').write_text(json.dumps(cpu,indent=2))
 rows=[]
 for folder in sorted(a.root.glob('gpu_*')):
  if not folder.is_dir() or not (folder/'summary.json').exists():continue
  s=json.loads((folder/'summary.json').read_text())
  if s.get('error'):continue
  for f in sorted(folder.glob('repeat_*.npz')):
   d=np.load(f);q=d['world0_qpos_every_step'];baseline=arrays['qpos'][1:]
   if q.shape!=baseline.shape:raise ValueError(f'State shape differs: {f}')
   pos_error=0.;hook_error=0.;t=time.perf_counter()
   for i in range(len(q)):
    e.data.qpos[:]=q[i];mj.mj_kinematics(e.model,e.data);pose=e.poses();delta=np.linalg.norm(pose[:,:3]-arrays['poses'][i+1,:,:3],axis=1);pos_error=max(pos_error,float(delta[:-1].max()));hook_error=max(hook_error,float(delta[-1]))
   rows.append(dict(num_envs=int(folder.name.split('_')[1]),repeat=f.stem,max_plant_body_origin_difference_m=pos_error,max_hook_origin_difference_m=hook_error,max_robot_joint_difference_m_or_rad=float(np.max(abs(q[:,e.qids]-baseline[:,e.qids]))),max_all_joint_coordinate_difference=float(np.max(abs(q-baseline))),gpu_target_peak_displacement_m=float(d['target_peak_displacement_m'].max()),cpu_target_peak_displacement_m=cpu['max_target_displacement_m'],analysis_wall_s=time.perf_counter()-t))
 (a.output/'comparison.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
if __name__=='__main__':main()
