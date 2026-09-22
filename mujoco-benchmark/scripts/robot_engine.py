"""Dynamic seven-DOF robot replay; common CPU/MJWarp initial state and commands."""
import json,time
from pathlib import Path
import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation as R
from benchmark_mujoco import HOME
from suite import RING
DEFAULT_TRACE=HOME.parent/'nvidia-sim/rl/runs/20260921_213000_single_view_batched/results/candidate_00049/trace.json'
DEFAULT_MODEL=HOME/'models/robot_plant_optimized.mjb'

def forward(ref,q):
 t=np.array(ref['robot_fk']['world'])
 for origin,axis,kind,index in ref['robot_fk']['chain']:
  t=t@origin;axis=np.array(axis)
  if index is not None:
   if kind=='prismatic':t[:3,3]+=t[:3,:3]@(axis*q[index])
   else:t[:3,:3]=t[:3,:3]@R.from_rotvec(axis*q[index]).as_matrix()
 return t

class RobotEngine:
 def __init__(self,model=DEFAULT_MODEL,trace=DEFAULT_TRACE,hz=120,reference=None,target="Tomato_05"):
  began=time.perf_counter();self.ref=json.loads(Path(reference or HOME/'assets/reference/reference.json').read_text())
  self.model=m=mj.MjModel.from_binary_path(str(model)) if str(model).endswith('.mjb') else mj.MjModel.from_xml_path(str(model));self.data=d=mj.MjData(m);m.opt.timestep=1/hz
  self.names=self.ref['robot_fk']['joint_names'];self.qids=np.array([m.jnt_qposadr[m.joint(n).id] for n in self.names]);self.dofs=np.array([m.jnt_dofadr[m.joint(n).id] for n in self.names]);self.aids=np.array([m.actuator('drive_'+n).id for n in self.names]);self.hook=m.body('Hook').id
  rows=json.loads(Path(trace).read_text());self.initial=np.asarray(rows[0]['joints']);self.commands=np.asarray([r['command'] for r in rows]);self.ts=np.arange(len(rows))/60
  self.ids=np.array([m.body(b['name']).id for b in self.ref['bodies']]+[self.hook]);self.hookgeoms=np.array([m.geom(s['name']).id for s in self.ref['shapes'] if s['body']==self.ref['tool_path']]);self.target=target;self.target_spec=next(s for s in self.ref['fruit_specs'] if s['name']==target);self.fruit=m.body(target).id
  mj.mj_resetData(m,d);d.qpos[self.qids]=self.initial;mj.mj_forward(m,d)
  self.preload=d.qfrc_bias.copy();self.preload[self.dofs]=0 # robot gravity compensated separately; plant preload is frozen
  self.initial_fruit=d.xpos[self.fruit].copy();self.reset()
  expected=forward(self.ref,self.initial);self.fk_error_m=float(np.linalg.norm(expected[:3,3]-d.xpos[self.hook]));self.fk_error_rad=float((R.from_matrix(expected[:3,:3]).inv()*R.from_matrix(d.xmat[self.hook].reshape(3,3))).magnitude())
  if self.fk_error_m>1e-5 or self.fk_error_rad>1e-5:raise RuntimeError(f'Robot FK mismatch {self.fk_error_m}, {self.fk_error_rad}')
  self.load_s=time.perf_counter()-began
 def reset(self):
  mj.mj_resetData(self.model,self.data);self.data.qpos[self.qids]=self.initial;self.data.ctrl[self.aids]=self.initial;self.data.qfrc_applied[:]=self.preload;mj.mj_forward(self.model,self.data)
 def command(self,t):return np.array([np.interp(t,self.ts,self.commands[:,i]) for i in range(7)])
 def poses(self):return np.column_stack([self.data.xpos[self.ids],self.data.xquat[self.ids][:,[1,2,3,0]]])
 def rollout(self,seconds=None,record=False):
  seconds=self.ts[-1] if seconds is None else seconds
  if seconds>self.ts[-1]+1e-6:raise ValueError('Requested duration exceeds recorded commands')
  steps=round(seconds/self.model.opt.timestep);times=np.arange(steps+1)*self.model.opt.timestep
  t=time.perf_counter();commands=np.array([self.command(v) for v in times[1:]]);planning=time.perf_counter()-t
  t=time.perf_counter();self.reset();reset=time.perf_counter()-t
  initial_glb_penetration=max([0.]+[-float(c.dist) for c in self.data.contact if self.model.geom(c.geom1).name.startswith('glb_col_') or self.model.geom(c.geom2).name.startswith('glb_col_')]);max_glb_penetration=initial_glb_penetration
  out={'control_s':0.,'physics_s':0.,'evaluation_s':0.,'recording_s':0.,'reset_s':reset,'command_prepare_s':planning};trace=[];poses=[];pairs=[];maxdisp=0.;mincontact=0.;contactsteps=0;maxerror=0.;unstable=False
  if record:trace.append(self.data.qpos.copy());poses.append(self.poses())
  begin=time.perf_counter();cpu=time.process_time()
  for i,cmd in enumerate(commands):
   t=time.perf_counter();self.data.ctrl[self.aids]=cmd;out['control_s']+=time.perf_counter()-t
   t=time.perf_counter();mj.mj_step(self.model,self.data);mj.mj_kinematics(self.model,self.data);out['physics_s']+=time.perf_counter()-t
   t=time.perf_counter();maxdisp=max(maxdisp,float(np.linalg.norm(self.data.xpos[self.fruit]-self.initial_fruit)));maxerror=max(maxerror,float(np.max(abs(self.data.qpos[self.qids]-cmd))))
   contacts=self.data.contact
   if self.ref.get('schema')=='glb_physics_reference_v1':
    max_glb_penetration=max(max_glb_penetration,max([0.]+[-float(c.dist) for c in contacts if self.model.geom(c.geom1).name.startswith('glb_col_') or self.model.geom(c.geom2).name.startswith('glb_col_')]))
   sel=np.isin(contacts.geom1,self.hookgeoms)|np.isin(contacts.geom2,self.hookgeoms);contactsteps+=int(sel.any())
   if sel.any():mincontact=min(mincontact,float(contacts.dist[sel].min()))
   unstable=not np.isfinite(self.data.qpos).all() or np.max(abs(self.data.qvel))>1e4 or bool(np.any(self.data.warning.number))
   out['evaluation_s']+=time.perf_counter()-t
   if record:
    t=time.perf_counter();trace.append(self.data.qpos.copy());poses.append(self.poses());pairs.append([[self.model.geom(int(a)).name,self.model.geom(int(b)).name] for a,b in contacts.geom[sel]]);out['recording_s']+=time.perf_counter()-t
   if unstable:break
  wall=time.perf_counter()-begin
  out.update(rollout_wall_s=wall,process_cpu_s=time.process_time()-cpu,simulated_s=(i+1)*self.model.opt.timestep,requested_s=seconds,steps=i+1,rtf=(i+1)*self.model.opt.timestep/wall,unstable=unstable,max_target_displacement_m=maxdisp,max_hook_contact_penetration_m=-mincontact,hook_contact_steps=contactsteps,max_tracking_error_m_or_rad=maxerror,warning_counts=self.data.warning.number.tolist(),hook_final_xyz=self.data.xpos[self.hook].tolist(),break_enabled=False,success_evaluator='contact/displacement diagnostics only; no harvest success claim')
  if self.ref.get('schema')=='glb_physics_reference_v1':
   out['initial_glb_penetration_m']=initial_glb_penetration;out['max_glb_penetration_m']=max_glb_penetration;out['glb_physics_valid']=max_glb_penetration<=.0005 and not unstable
  if record:out['_arrays']=dict(times_s=times[:len(trace)],qpos=np.array(trace),poses=np.array(poses));out['_contacts']=pairs
  return out
