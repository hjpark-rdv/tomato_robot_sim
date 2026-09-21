"""mjlab Simulation (MJWarp) replay of the CPU robot benchmark, no learning."""
import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,json,time,datetime,hashlib,threading,subprocess
from pathlib import Path
import numpy as np
from robot_engine import RobotEngine,DEFAULT_TRACE,HOME

def main():
 p=argparse.ArgumentParser();p.add_argument('--num-envs',type=int,default=8);p.add_argument('--seconds',type=float,default=16.5);p.add_argument('--repeats',type=int,default=3);p.add_argument('--hz',type=int,default=120);p.add_argument('--model',type=Path,default=HOME/'models/robot_plant_mjlab.mjb');p.add_argument('--trace',type=Path,default=DEFAULT_TRACE);p.add_argument('--output',type=Path,required=True);p.add_argument('--nconmax',type=int,default=256);p.add_argument('--njmax',type=int,default=1024);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 import importlib.metadata as meta
 manifest=dict(arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),versions={n:meta.version(n) for n in ['mujoco','mujoco-warp','mjlab','warp-lang','torch']},model_sha256=hashlib.sha256(a.model.read_bytes()).hexdigest(),trace_sha256=hashlib.sha256(a.trace.read_bytes()).hexdigest(),git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=HOME,text=True).strip())
 (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2));started=time.perf_counter()
 import torch,warp as wp,mujoco as mj,mujoco_warp as mjw
 from mjlab.sim import Simulation,SimulationCfg,MujocoCfg
 from gpu_contact_metrics import contacts as contact_kernel
 wp.init();torch.set_num_threads(1)
 resource_stop=threading.Event();samples=[]
 def monitor():
  import psutil
  proc=psutil.Process();proc.cpu_percent()
  while not resource_stop.is_set():
   row=dict(monotonic_s=time.perf_counter(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rss_mib=proc.memory_info().rss/2**20,cpu_percent=proc.cpu_percent())
   try:row['gpu']=subprocess.check_output(['nvidia-smi','--query-gpu=memory.used,utilization.gpu','--format=csv,noheader,nounits'],text=True,timeout=2).strip()
   except Exception:pass
   samples.append(row);resource_stop.wait(1)
 thread=threading.Thread(target=monitor,daemon=True);thread.start();report={}
 try:
  t=time.perf_counter();e=RobotEngine(a.model,a.trace,a.hz);load=time.perf_counter()-t
  o=e.model.opt
  cfg=SimulationCfg(nconmax=a.nconmax,njmax=a.njmax,mujoco=MujocoCfg(timestep=o.timestep,integrator='implicitfast',solver='newton',jacobian='sparse',cone='elliptic',iterations=o.iterations,tolerance=o.tolerance,ls_iterations=o.ls_iterations,ls_tolerance=o.ls_tolerance,impratio=o.impratio,gravity=tuple(o.gravity),ccd_iterations=o.ccd_iterations))
  report.update(model_load_s=load)
  t=time.perf_counter();sim=Simulation(a.num_envs,cfg,model=e.model,device='cuda:0');wp.synchronize();compile_s=time.perf_counter()-t
  print('[mjlab 준비]',a.num_envs,'환경, 모델',round(load,2),'초, GPU 준비/컴파일',round(compile_s,2),'초',flush=True)
  # Assert the physics parameters, rather than accepting framework defaults.
  assert sim.mj_model.opt.timestep==1/a.hz and sim.mj_model.opt.iterations==64
  q0=torch.as_tensor(e.data.qpos.copy(),dtype=torch.float32,device='cuda:0');preload=torch.as_tensor(e.preload,dtype=torch.float32,device='cuda:0');ctrl0=torch.as_tensor(e.data.ctrl.copy(),dtype=torch.float32,device='cuda:0')
  aids=torch.tensor(e.aids,device='cuda:0');qids=torch.tensor(e.qids,device='cuda:0');fruit0=torch.as_tensor(e.initial_fruit,dtype=torch.float32,device='cuda:0')
  steps=round(a.seconds*a.hz);t=time.perf_counter();cmds=torch.as_tensor(np.array([e.command((i+1)/a.hz) for i in range(steps)]),dtype=torch.float32,device='cuda:0');torch.cuda.synchronize();command_prepare=time.perf_counter()-t
  def reset():
   sim.reset();sim.data.qpos.copy_(q0.expand(a.num_envs,-1));sim.data.qvel.zero_();sim.data.ctrl.copy_(ctrl0.expand(a.num_envs,-1));sim.data.qfrc_applied.copy_(preload.expand(a.num_envs,-1));sim.forward();wp.synchronize()
  hookmask=torch.zeros(e.model.ngeom,dtype=torch.int32,device='cuda:0');hookmask[torch.tensor(e.hookgeoms,device='cuda:0')]=1
  hook_wp=wp.from_torch(hookmask);hit=torch.zeros(a.num_envs,dtype=torch.int32,device='cuda:0');hit_wp=wp.from_torch(hit);minimum=torch.zeros(a.num_envs,device='cuda:0');minimum_wp=wp.from_torch(minimum)
  def contact_step():
   hit.zero_();c=sim.wp_data.contact
   wp.launch(contact_kernel,dim=c.dist.shape[0],inputs=[sim.wp_data.nacon,c.worldid,c.geom,c.dist,hook_wp,hit_wp,minimum_wp],device='cuda:0')
  t=time.perf_counter();contact_step();wp.synchronize();contact_setup_s=time.perf_counter()-t
  reset()
  initial_error=float((sim.data.qpos-q0).abs().max().item());assert initial_error==0
  # Model inputs are uploaded once; each frame only sends the next command index.
  t=time.perf_counter()
  for i in range(min(240,steps)):
   sim.data.ctrl[:,aids]=cmds[i];sim.step()
  wp.synchronize();warmup=time.perf_counter()-t
  rounds=[]
  report.update(contact_setup_s=contact_setup_s,gpu_setup_and_compile_s=compile_s,warmup_s=warmup,command_prepare_s=command_prepare,rounds=rounds)
  for repeat in range(a.repeats):
   t=time.perf_counter();reset();reset_s=time.perf_counter()-t
   disp=torch.zeros(a.num_envs,device='cuda:0');err=torch.zeros_like(disp);bad=torch.zeros(a.num_envs,dtype=torch.bool,device='cuda:0');qtrace=[];events=[];contact_count=torch.zeros_like(hit);minimum.zero_()
   begin=time.perf_counter();cpu=time.process_time()
   for i in range(steps):
    ev=[torch.cuda.Event(enable_timing=True) for _ in range(4)];ev[0].record()
    sim.data.ctrl[:,aids]=cmds[i];ev[1].record();sim.step();mjw.kinematics(sim.wp_model,sim.wp_data);ev[2].record()
    disp=torch.maximum(disp,torch.linalg.vector_norm(sim.data.xpos[:,e.fruit]-fruit0,dim=1));err=torch.maximum(err,(sim.data.qpos[:,qids]-cmds[i]).abs().amax(1));bad|=(~torch.isfinite(sim.data.qpos).all(1))|(sim.data.qvel.abs().amax(1)>1e4)
    contact_step();contact_count+=hit
    ev[3].record();events.append(ev)
    qtrace.append(sim.data.qpos[0].clone())
   wp.synchronize();wall=time.perf_counter()-begin;cpu=time.process_time()-cpu
   # MJWarp reports overflow separately from finite-state checks.
   overflow=np.asarray(sim.wp_data.overflow.numpy()) if hasattr(sim.wp_data,'overflow') else None
   finite=not bool(bad.any().item());overflowed=bool(overflow is not None and np.any(overflow))
   times=[sum(x[j].elapsed_time(x[j+1]) for x in events)/1000 for j in range(3)]
   t=time.perf_counter();final=sim.data.qpos.cpu().numpy();np.savez_compressed(a.output/f'repeat_{repeat:02}.npz',qpos=final,world0_qpos_every_step=torch.stack(qtrace).cpu().numpy(),target_peak_displacement_m=disp.cpu().numpy(),tracking_error=err.cpu().numpy(),hook_contact_steps=contact_count.cpu().numpy(),hook_contact_min_dist=minimum.cpu().numpy());save=time.perf_counter()-t
   row=dict(repeat=repeat,num_envs=a.num_envs,simulated_s_per_env=steps/a.hz,wall_s=wall,aggregate_rtf=a.num_envs*steps/a.hz/wall,process_cpu_s=cpu,reset_s=reset_s,save_s=save,control_gpu_s=times[0],physics_gpu_s=times[1],evaluation_gpu_s=times[2],finite=finite,overflow=overflow.tolist() if overflow is not None else None,overflowed=overflowed,max_hook_contact_penetration_m=-minimum.min().item(),hook_contact_steps_max=contact_count.max().item(),max_target_displacement_m=disp.max().item(),max_tracking_error_m_or_rad=err.max().item(),max_clone_qpos_difference=float(np.max(abs(final-final[0]))))
   rounds.append(row);print('[mjlab 측정]',json.dumps(row),flush=True)
   if not finite or overflowed:raise RuntimeError('Unstable or overflowed GPU state; performance is not valid')
  report=dict(model_load_s=load,contact_setup_s=contact_setup_s,gpu_setup_and_compile_s=compile_s,warmup_s=warmup,command_prepare_s=command_prepare,rounds=rounds,process_total_s=time.perf_counter()-started,initial_qpos_error=initial_error,scope='mjlab Simulation layer, identical joint command replay, no RL; GPU evaluates displacement/finite state and generated hook contacts each step; no IK or harvest success claim; geometry audit is separate')
 except BaseException:
  import traceback
  report.update(error=traceback.format_exc());raise
 finally:
  resource_stop.set();thread.join(timeout=4);(a.output/'summary.json').write_text(json.dumps(report,indent=2));(a.output/'resources.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in samples))
if __name__=='__main__':main()
