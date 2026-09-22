"""선택한 후보의 원본 물리 상태를 다시 계산하지 않고 표시."""
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np
import mujoco as mj

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--candidate',required=True);p.add_argument('--check-only',action='store_true');a=p.parse_args()
 root=a.run.resolve();folder=root/'candidates'/a.candidate
 if folder.parent!=root/'candidates':raise ValueError('invalid candidate')
 manifest=json.loads((root/'manifest.json').read_text());model=root/'replay_assets/model.mjb'
 if hashlib.sha256(model.read_bytes()).hexdigest()!=manifest['model_sha256']:raise RuntimeError('model hash mismatch')
 if mj.__version__!=manifest['mujoco']:raise RuntimeError('MuJoCo version mismatch')
 result=json.loads((folder/'result.json').read_text())
 if result.get('states_sha256') and hashlib.sha256((folder/'states.npz').read_bytes()).hexdigest()!=result['states_sha256']:raise RuntimeError('state hash mismatch')
 m=mj.MjModel.from_binary_path(str(model));d=mj.MjData(m);states=np.load(folder/'states.npz');q=states['qpos'];times=states['times_s']
 if not np.isfinite(q).all() or q.shape[1]!=m.nq:raise RuntimeError('invalid recorded states')
 print('[저장 상태 확인]',a.candidate,len(q),'프레임',times[-1],'초',flush=True)
 if a.check_only:return
 import mujoco.viewer
 with mj.viewer.launch_passive(m,d) as view:
  d.qpos[:]=q[0]
  from view_camera import target_camera
  target_camera(m,d,view.cam)
  while view.is_running():
   began=time.perf_counter()
   for i in np.unique(np.r_[np.searchsorted(times,np.arange(0,times[-1],1/30)),len(times)-1]):
    if not view.is_running():return
    time.sleep(max(0,began+times[i]-time.perf_counter()));d.qpos[:]=q[i];mj.mj_forward(m,d);view.sync()
if __name__=='__main__':main()
