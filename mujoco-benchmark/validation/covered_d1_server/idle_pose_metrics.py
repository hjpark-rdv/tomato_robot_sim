import json,sys
from pathlib import Path
import mujoco as mj
import numpy as np
r=Path('/root/docker_share/mujoko_debugging_data/20260928_covered_d1_server');result={}
for label in ['original','covered']:
 info=json.loads((r/f'{label}_idle.json').read_text());m=mj.MjModel.from_binary_path(info['model']);d=mj.MjData(m);arr=np.load(r/f'{label}_idle.npz')
 plant=[i for i in range(m.nbody) if (m.body(i).name or '').startswith(('STEM_','TRUSS_','Tomato_','Attachment_'))]
 robot=[i for i in range(m.nbody) if (m.body(i).name or '').startswith('Robot_') or m.body(i).name=='Hook']
 d.qpos[:]=arr['qpos'][0];mj.mj_kinematics(m,d);base=d.xpos.copy();metrics={'active_plant_max_displacement_m':0.,'robot_max_displacement_m':0.,'target_max_displacement_m':0.}
 for q in arr['qpos']:
  d.qpos[:]=q;mj.mj_kinematics(m,d);disp=np.linalg.norm(d.xpos-base,axis=1)
  metrics['active_plant_max_displacement_m']=max(metrics['active_plant_max_displacement_m'],float(disp[plant].max()))
  metrics['robot_max_displacement_m']=max(metrics['robot_max_displacement_m'],float(disp[robot].max()))
  metrics['target_max_displacement_m']=max(metrics['target_max_displacement_m'],float(disp[m.body('Tomato_02').id]))
 result[label]=metrics
(r/'idle_pose_metrics.json').write_text(json.dumps(result,indent=2));print(result)
