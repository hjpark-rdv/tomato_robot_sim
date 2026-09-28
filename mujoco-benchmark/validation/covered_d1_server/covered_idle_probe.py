import sys,json,time,hashlib
from pathlib import Path
import numpy as np
import mujoco as mj
sys.path.insert(0,'/root/farmily_tomato/mujoco-benchmark/scripts')
from robot_engine import RobotEngine
from environment_preflight import MuJoCoScene,Policy,screen
from add_neighbor_truss_obstacles import audit_scene
root=Path('/root/docker_share/mujoko_debugging_data/20260928_covered_d1_server')
source=Path('/root/docker_share/mujoko_debugging_data/20260927_stem_obstacle_scene')
covered=Path('/mnt/nas_rdv_md3/covered_d1_20260928/covered_scene')
run=source/'robot_checks/Tomato_02'
(root/'coverage.json').write_text(json.dumps(audit_scene(covered,True,True),indent=2))
for label,folder in [('original',source),('covered',covered)]:
 e=RobotEngine(folder/'model.mjb',run/'replay_assets/initial_trace.json',240,reference=folder/'reference.json',target='Tomato_02')
 m,d=e.model,e.data
 def category(g):
  n=m.geom(g).name or '';b=m.body(m.geom_bodyid[g]).name or ''
  if b.startswith('Robot_') or b=='Hook':return 'robot'
  if n.startswith('neighbor_'):return 'background_obstacle'
  if n.startswith('gutter_'):return 'structural_obstacle'
  return 'active_plant'
 def contacts():
  rows=[]
  for c in d.contact:
   if c.dist<0:rows.append(dict(a=m.geom(c.geom1).name,b=m.geom(c.geom2).name,categories=[category(c.geom1),category(c.geom2)],penetration_m=-float(c.dist)))
  return sorted(rows,key=lambda r:-r['penetration_m'])
 initial=contacts();x0=d.xpos.copy();q=[];ts=[];worst={};maxdisp=0.;t=time.monotonic()
 for k in range(481):
  mj.mj_forward(m,d)
  for row in contacts():
   key=(row['a'],row['b'])
   if key not in worst or row['penetration_m']>worst[key]['penetration_m']:worst[key]=dict(row,time_s=float(d.time))
  maxdisp=max(maxdisp,float(np.max(np.linalg.norm(d.xpos[1:]-x0[1:],axis=1))))
  q.append(d.qpos.copy());ts.append(float(d.time))
  if k<480:mj.mj_step(m,d)
 np.savez_compressed(root/f'{label}_idle.npz',qpos=q,times_s=ts)
 record=dict(model=str(folder/'model.mjb'),target='Tomato_02',hz=240,initial_contacts=initial,worst_contacts=sorted(worst.values(),key=lambda r:-r['penetration_m']),max_body_displacement_m=maxdisp,warning_counts=d.warning.number.tolist(),finite=bool(np.isfinite(q).all()),wall_s=time.monotonic()-t,load_s=e.load_s)
 if label=='covered':
  sc=MuJoCoScene(e,Policy(clearance_m=0))
  record['initial_robot_audit']=screen(sc,[e.initial,e.initial],['ready','ready'],1/60,Policy(clearance_m=0),True)
 (root/f'{label}_idle.json').write_text(json.dumps(record,indent=2))
 print(label,'initial',initial[:3],'worst',record['worst_contacts'][:3], 'disp',maxdisp,flush=True)
