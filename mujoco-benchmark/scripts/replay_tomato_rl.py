"""저장된 RL 독립 평가 world 0을 물리 재실행 없이 1배속 재생."""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
import mujoco as mj

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--check-only',action='store_true');a=p.parse_args()
 manifest=json.loads((a.run/'manifest.json').read_text());model=a.run/'rl_model.mjb'
 if hashlib.sha256(model.read_bytes()).hexdigest()!=manifest['model_sha256']:raise RuntimeError('Model hash mismatch')
 if mj.__version__!=manifest['versions']['mujoco']:raise RuntimeError('Use the same .mjlab-venv MuJoCo version')
 m=mj.MjModel.from_binary_path(str(model));d=mj.MjData(m);z=np.load(a.run/'evaluation_world0.npz');q=z['qpos'];ts=z['times_s']
 assert q.shape==(len(ts),m.nq) and np.isfinite(q).all() and np.all(np.diff(ts)>0)
 print('[RL 평가 재생]',len(ts),'프레임',ts[-1],'초',flush=True)
 if a.check_only:return
 import mujoco.viewer
 with mujoco.viewer.launch_passive(m,d) as v:
  d.qpos[:]=q[0];mj.mj_forward(m,d);v.cam.lookat[:]=d.xpos[m.body('Tomato_06').id];v.cam.distance=.65;v.cam.azimuth=-70;v.cam.elevation=-15
  start=time.perf_counter()
  while v.is_running():
   t=(time.perf_counter()-start)%(float(ts[-1])+1.);i=min(int(np.searchsorted(ts,t)),len(ts)-1)
   d.qpos[:]=q[i];mj.mj_forward(m,d);v.sync();time.sleep(1/30)
if __name__=='__main__':main()
