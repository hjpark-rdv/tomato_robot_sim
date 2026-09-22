"""View dynamic robot + plant, or save a deterministic replay video."""
import argparse,time,json,sys
if "--output" in sys.argv:
 import render_backend
from pathlib import Path
import numpy as np
from robot_engine import RobotEngine,DEFAULT_MODEL,DEFAULT_TRACE

def main():
 p=argparse.ArgumentParser();p.add_argument('--model',type=Path,default=DEFAULT_MODEL);p.add_argument('--trace',type=Path,default=DEFAULT_TRACE);p.add_argument('--hz',type=int,default=120);p.add_argument('--output',type=Path);p.add_argument('--camera',default='robot_overview');a=p.parse_args()
 if a.output:
  a.output.mkdir(parents=True,exist_ok=False)
  import render_backend
  import mujoco as mj,imageio.v2 as imageio
  e=RobotEngine(a.model,a.trace,a.hz);r=e.rollout(record=True);arrays=r.pop('_arrays');contacts=r.pop('_contacts');t=time.perf_counter();np.savez_compressed(a.output/'states.npz',**arrays);r['state_save_s']=time.perf_counter()-t
  from metrics import audit
  t=time.perf_counter();r['offline_geometry_audit']=audit(e.ref,arrays['poses'],arrays['times_s'],contacts);r['offline_audit_s']=time.perf_counter()-t
  t=time.perf_counter()
  with mj.Renderer(e.model,height=720,width=960) as renderer, imageio.get_writer(a.output/'robot_replay_2x.mp4',fps=30) as video:
   for i in range(0,len(arrays['qpos']),max(1,a.hz//15)):
    e.data.qpos[:]=arrays['qpos'][i];mj.mj_forward(e.model,e.data);renderer.update_scene(e.data,camera=a.camera);frame=renderer.render();video.append_data(frame)
    if i==0:imageio.imwrite(a.output/'robot_initial.png',frame)
  r['offline_video_s']=time.perf_counter()-t;(a.output/'validation.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));return
 import mujoco as mj,mujoco.viewer
 e=RobotEngine(a.model,a.trace,a.hz)
 with mujoco.viewer.launch_passive(e.model,e.data) as view:
  from view_camera import target_camera
  target_camera(e.model,e.data,view.cam)
  while view.is_running():
   e.reset()
   for t in np.arange(0,e.ts[-1],1/a.hz):
    if not view.is_running():return
    start=time.perf_counter();e.data.ctrl[e.aids]=e.command(t);mj.mj_step(e.model,e.data);view.sync();time.sleep(max(0,1/a.hz-(time.perf_counter()-start)))
if __name__=='__main__':main()
