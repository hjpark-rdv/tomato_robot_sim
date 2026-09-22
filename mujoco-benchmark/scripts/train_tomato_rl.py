"""고정 Tomato_06 장면: mjlab Simulation/MJWarp + RSL-RL PPO, GT 관측."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'): os.environ[key]='1'
import argparse,copy,datetime,hashlib,json,time,sys,shutil
from pathlib import Path
import numpy as np
import torch
import warp as wp
import mujoco_warp as mjw
import mujoco as mj
from tensordict import TensorDict
from robot_engine import RobotEngine,HOME
from suite import RING
from mjlab.sim import Simulation,SimulationCfg,MujocoCfg
from rsl_rl.algorithms import PPO
sys.path.insert(0,str(HOME.parent/'nvidia-sim/rl'))
from geometry import RING_RADIUS,WIRE_RADIUS

@wp.kernel
def contact_flags(n:wp.array(dtype=int),world:wp.array(dtype=int),geom:wp.array(dtype=wp.vec2i),dist:wp.array(dtype=float),robot:wp.array(dtype=int),target:wp.array(dtype=int),minimum:wp.array(dtype=float),other:wp.array(dtype=int)):
 i=wp.tid()
 if i<n[0]:
  g=geom[i];w=world[i]
  if w>=0 and g[0]>=0 and g[1]>=0:
   if robot[g[0]]!=0 or robot[g[1]]!=0:
    wp.atomic_min(minimum,w,dist[i])
    if dist[i]<0.0 and target[g[0]]==0 and target[g[1]]==0:
     wp.atomic_max(other,w,1)

class TomatoEnv:
 @torch.inference_mode()
 def __init__(self,a):
  self.device='cuda:0';self.num_envs=a.num_envs;self.num_actions=7;self.dt=.05
  self.max_episode_length=round(a.episode_seconds/self.dt);self.root=a.output
  self.e=e=RobotEngine(HOME/'models/robot_plant_mjlab.mjb',hz=120,target='Tomato_06')
  # Enable non-neighbour arm self-collision only in a separate RL model.
  import xml.etree.ElementTree as ET
  tree=ET.parse(HOME/'models/robot_plant_mjlab.xml');xml=tree.getroot();contact=xml.find('contact')
  robot_names={e.model.body(i).name for i in range(e.model.nbody) if (e.model.body(i).name or '').startswith('Robot_')}|{'Hook'}
  def neighbours(a,b):
   wa=int(e.model.body_weldid[e.model.body(a).id]);wb=int(e.model.body_weldid[e.model.body(b).id])
   return wa==wb or int(e.model.body_weldid[e.model.body_parentid[wa]])==wb or int(e.model.body_weldid[e.model.body_parentid[wb]])==wa
  for pair in list(contact):
   if pair.tag=='exclude' and pair.get('body1') in robot_names and pair.get('body2') in robot_names and not neighbours(pair.get('body1'),pair.get('body2')):contact.remove(pair)
  for node in xml.iter('geom'):
   if int(node.get('contype','0'))==8:node.set('conaffinity',str(int(node.get('conaffinity','0'))|8))
  names=sorted(robot_names)
  for i,na in enumerate(names):
   for nb in names[i+1:]:
    if neighbours(na,nb):ET.SubElement(contact,'exclude',body1=na,body2=nb)
  # MJWarp checks mesh margins before explicit pair overrides. Only robot collider
  # margins/gaps become zero; plant defaults remain .5mm (pairwise max).
  for body in xml.iter('body'):
   if body.get('name') in robot_names:
    for geom in body.findall('geom'):
     if int(geom.get('contype','1'))!=0:
      geom.set('margin','0');geom.set('gap','0')
  modelpath=a.output/'rl_model.xml';tree.write(modelpath,encoding='unicode')
  e=RobotEngine(modelpath,hz=120,target='Tomato_06');self.e=e
  mj.mj_saveModel(e.model,str(a.output/'rl_model.mjb'))
  o=e.model.opt
  cfg=SimulationCfg(nconmax=512,njmax=2048,mujoco=MujocoCfg(timestep=1/120,integrator='implicitfast',solver='newton',jacobian='sparse',cone='elliptic',iterations=o.iterations,tolerance=o.tolerance,ls_iterations=o.ls_iterations,ls_tolerance=o.ls_tolerance,impratio=o.impratio,gravity=tuple(o.gravity),ccd_iterations=o.ccd_iterations))
  self.sim=Simulation(a.num_envs,cfg,model=e.model,device=self.device)
  def tensor(v,dtype=torch.float32):return torch.as_tensor(v,device=self.device,dtype=dtype)
  self.q0=tensor(e.data.qpos.copy());self.ctrl0=tensor(e.data.ctrl.copy());self.preload=tensor(e.preload)
  self.qids=tensor(e.qids,torch.long);self.dofs=tensor(e.dofs,torch.long);self.aids=tensor(e.aids,torch.long)
  jids=[e.model.joint(n).id for n in e.names];self.limits=tensor(e.model.jnt_range[jids])
  # Conservative experiment command caps, not a claim of measured hardware maxima.
  self.speed=tensor([.04 if e.model.jnt_type[j]==2 else .25 for j in jids])
  self.offset=tensor(e.target_spec['center']);self.ring=tensor(RING);self.radius=e.target_spec['radius']
  self.fruitids=tensor([e.model.body(s['name']).id for s in e.ref['fruit_specs']],torch.long)
  self.fruitoffsets=tensor([s['center'] for s in e.ref['fruit_specs']])
  self.fruit0=tensor([e.data.xpos[e.model.body(s['name']).id]+e.data.xmat[e.model.body(s['name']).id].reshape(3,3)@np.array(s['center']) for s in e.ref['fruit_specs']])
  self.target0=tensor(e.data.xpos[e.fruit]+e.data.xmat[e.fruit].reshape(3,3)@np.array(e.target_spec['center']))
  robot=np.zeros(e.model.ngeom,np.int32);target=np.zeros_like(robot)
  robot_bodies={e.model.jnt_bodyid[j] for j in jids}|{e.hook}
  for g,b in enumerate(e.model.geom_bodyid):
   ancestor=int(b)
   while ancestor:
    if ancestor in robot_bodies:robot[g]=1;break
    ancestor=int(e.model.body_parentid[ancestor])
   target[g]=int(b==e.fruit or '_06_' in (e.model.body(int(b)).name or '') and 'TRUSS_Pedicel_' in (e.model.body(int(b)).name or ''))
  self.robot=tensor(robot,torch.int32);self.targetmask=tensor(target,torch.int32)
  self.minimum=torch.zeros(a.num_envs,device=self.device);self.other=torch.zeros(a.num_envs,dtype=torch.int32,device=self.device)
  self.episode_length_buf=torch.zeros(a.num_envs,dtype=torch.long,device=self.device)
  self.command=self.ctrl0[self.aids].repeat(a.num_envs,1);self.last_action=torch.zeros_like(self.command)
  self.peak=torch.zeros(a.num_envs,device=self.device);self.hold=torch.zeros_like(self.peak);self.entered=torch.zeros(a.num_envs,dtype=torch.bool,device=self.device)
  self.stats=dict(episodes=0,success=0,excessive_displacement=0,penetration=0,timeouts=0)
  self.reset(torch.arange(a.num_envs,device=self.device));self.initial_distance=self.distance().clone()
  assert not bool(self.inside().any()), 'Initial pose must be outside the capture region'
  self.initial_qpos_error=float((self.sim.data.qpos-self.q0).abs().max())
  # Selected reset must restore plant/robot state without resetting other worlds.
  saved=self.sim.data.qpos.clone();self.sim.data.qpos[0,self.qids[0]]+=.001
  self.reset(torch.tensor([0],device=self.device))
  assert torch.equal(self.sim.data.qpos,saved), 'Selective reset mismatch'
  self.selective_reset_verified=True
 def geometry(self):
  d=self.sim.data;r=d.xmat[:,self.e.hook].reshape(-1,3,3);fr=d.xmat[:,self.e.fruit].reshape(-1,3,3)
  center=d.xpos[:,self.e.fruit]+torch.matmul(fr,self.offset)
  ring=d.xpos[:,self.e.hook]+torch.matmul(r,self.ring)
  local=torch.bmm(r.transpose(1,2),(center-ring).unsqueeze(2)).squeeze(2)
  return center,ring,r,local
 def distance(self):
  _,_,_,local=self.geometry()
  # Aim inside rear aperture, with all orientations free.
  goal=local.new_tensor([-.010,0,0]);return torch.linalg.vector_norm(local-goal,dim=1)
 def inside(self):
  v=self.geometry()[3]
  return (v[:,0]<=-.001)&(torch.linalg.vector_norm(v[:,[0,2]],dim=1)+self.radius<=RING_RADIUS-WIRE_RADIUS)&(v[:,1].abs()<=.002)
 @torch.inference_mode()
 def reset(self,ids):
  self.sim.reset(ids);d=self.sim.data
  d.qpos[ids]=self.q0;d.qvel[ids]=0;d.ctrl[ids]=self.ctrl0;d.qfrc_applied[ids]=self.preload
  self.command[ids]=self.ctrl0[self.aids];self.last_action[ids]=0;self.episode_length_buf[ids]=0;self.peak[ids]=0;self.hold[ids]=0;self.entered[ids]=False
  self.sim.forward()
 def get_observations(self):
  d=self.sim.data;c,ring,r,local=self.geometry()
  obs=torch.cat(((d.qpos[:,self.qids]-self.q0[self.qids]),d.qvel[:,self.dofs]/self.speed,(self.command-d.qpos[:,self.qids])/self.speed,local/.2,r.reshape(-1,9),(c-self.target0)/.02,self.peak[:,None]/.02,self.last_action,self.episode_length_buf[:,None]/self.max_episode_length),1)
  return TensorDict({'policy':obs},batch_size=[self.num_envs])
 def step(self,actions):
  if not bool(torch.isfinite(actions).all()):raise RuntimeError('Nonfinite policy action')
  actions=actions.clamp(-1,1);before=self.distance();d=self.sim.data
  delta=actions*self.speed*self.dt
  self.command=torch.clamp(self.command+delta,self.limits[:,0],self.limits[:,1])
  # Prevent a blocked arm from accumulating large position error and stored drive force.
  self.command=torch.maximum(torch.minimum(self.command,d.qpos[:,self.qids]+self.speed*.1),d.qpos[:,self.qids]-self.speed*.1)
  self.command=torch.clamp(self.command,self.limits[:,0],self.limits[:,1])
  self.minimum.zero_();self.other.zero_()
  for _ in range(6):
   previous_time=d.time.clone();d.ctrl[:,self.aids]=self.command;self.sim.step();
   if bool((d.time<previous_time+.5/120).any()):raise RuntimeError('Physics auto-reset detected')
   mjw.kinematics(self.sim.wp_model,self.sim.wp_data)
   if not bool(torch.isfinite(d.qpos).all() and torch.isfinite(d.qvel).all()) or bool((d.qvel.abs()>1e4).any()):raise RuntimeError('Invalid physics; refusing to train on auto-reset/unstable state')
   centers=d.xpos[:,self.fruitids]+torch.matmul(d.xmat[:,self.fruitids].reshape(self.num_envs,-1,3,3),self.fruitoffsets[...,None]).squeeze(-1)
   self.peak=torch.maximum(self.peak,torch.linalg.vector_norm(centers-self.fruit0,dim=2).amax(1))
   c=self.sim.wp_data.contact
   wp.launch(contact_flags,dim=c.dist.shape[0],inputs=[self.sim.wp_data.nacon,c.worldid,c.geom,c.dist,wp.from_torch(self.robot),wp.from_torch(self.targetmask),wp.from_torch(self.minimum),wp.from_torch(self.other)],device=self.device)
  if hasattr(self.sim.wp_data,'overflow') and np.any(self.sim.wp_data.overflow.numpy()):raise RuntimeError('MJWarp contact/constraint overflow')
  self.episode_length_buf+=1
  inside=self.inside();self.entered|=inside;self.hold=torch.where(inside,self.hold+self.dt,0.)
  displaced=self.peak>.020;penetration=self.minimum<-.002
  success=(self.hold>=.2)&~displaced&~penetration
  timeout=self.episode_length_buf>=self.max_episode_length
  done=success|displaced|penetration|timeout
  reward=20*(before-self.distance())+.05*torch.exp(-self.distance()/.05)+inside.float()*.5-2*(self.peak/.02).square()*.05-.01*self.other.float()-.005*(actions-self.last_action).square().mean(1)-.001
  reward+=success.float()*10-(displaced|penetration).float()*5
  self.last_action=actions.clone()
  info={'terminal_ring_distance':self.geometry()[3].norm(dim=1),'terminal_qpos_world0':d.qpos[0].clone() if getattr(self,'record_evaluation',False) else None,'success':success.clone(),'peak_displacement':self.peak.clone(),'entered':self.entered.clone(),'penetration_m':-self.minimum.clone()}
  if bool(done.any()):
   self.stats['episodes']+=int(done.sum());self.stats['success']+=int(success.sum());self.stats['excessive_displacement']+=int((done&displaced).sum());self.stats['penetration']+=int((done&penetration).sum());self.stats['timeouts']+=int((done&timeout).sum())
   self.reset(done.nonzero().flatten())
  return self.get_observations(),reward,done,info

def train_config(steps):
 return dict(num_steps_per_env=steps,obs_groups={'actor':['policy'],'critic':['policy']},multi_gpu=None,
 actor=dict(class_name='MLPModel',hidden_dims=[128,128],activation='elu',obs_normalization=True,distribution_cfg=dict(class_name='GaussianDistribution',init_std=.5,std_type='scalar')),
 critic=dict(class_name='MLPModel',hidden_dims=[128,128],activation='elu',obs_normalization=True),
 algorithm=dict(class_name='PPO',num_learning_epochs=4,num_mini_batches=4,learning_rate=3e-4,schedule='adaptive',gamma=.99,lam=.95,entropy_coef=.01,rnd_cfg=None))


@torch.inference_mode()
def evaluate(env,alg,root,prefix):
 alg.eval_mode();env.record_evaluation=True;env.reset(torch.arange(env.num_envs,device=env.device));started=time.perf_counter()
 active=torch.ones(env.num_envs,dtype=torch.bool,device=env.device);success=torch.zeros_like(active)
 peak=torch.zeros(env.num_envs,device=env.device);closest=torch.full_like(peak,float('inf'))
 q=[env.sim.data.qpos[0].cpu().numpy().copy()]
 for step in range(env.max_episode_length):
  local=env.geometry()[3];closest=torch.minimum(closest,torch.where(active,local.norm(dim=1),float('inf')))
  _,_,done,info=env.step(alg.actor(env.get_observations()))
  success|=active&info['success'];peak=torch.maximum(peak,torch.where(active,info['peak_displacement'],0.))
  closest=torch.minimum(closest,torch.where(active,info['terminal_ring_distance'],float('inf')))
  if bool(active[0]):q.append(info['terminal_qpos_world0'].cpu().numpy().copy())
  active&=~done
  if not bool(active.any()):break
 np.savez_compressed(root/f'{prefix}_world0.npz',qpos=np.array(q),times_s=np.arange(len(q))*env.dt)
 result=dict(wall_s=time.perf_counter()-started,successes=int(success.sum()),environments=env.num_envs,max_fruit_displacement_m=peak.tolist(),mean_closest_ring_center_distance_m=float(closest.mean()),note='identical start deterministic clones; terminal frame included; not independent generalization trials')
 env.record_evaluation=False
 return result

def main():
 p=argparse.ArgumentParser();p.add_argument('--num-envs',type=int,default=32);p.add_argument('--iterations',type=int,default=200);p.add_argument('--steps-per-env',type=int,default=32);p.add_argument('--episode-seconds',type=float,default=30);p.add_argument('--seed',type=int,default=0);p.add_argument('--output',type=Path);p.add_argument('--checkpoint',type=Path);p.add_argument('--train-seconds',type=float,help='학습 구간 제한 초; 준비/전후 평가 제외, 업데이트 경계에서 종료');p.add_argument('--compare-before',action='store_true');a=p.parse_args()
 if a.train_seconds is not None and (not np.isfinite(a.train_seconds) or a.train_seconds<=0):p.error('--train-seconds must be positive and finite')
 if min(a.num_envs,a.iterations,a.steps_per_env)<1 or a.episode_seconds<.2:p.error('positive counts and episode seconds >= .2 required')
 a.output=a.output or Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_tomato06_rl');a.output.mkdir(parents=True,exist_ok=False)
 torch.manual_seed(a.seed);np.random.seed(a.seed);torch.set_num_threads(1);wp.init();start=time.perf_counter()
 env=TomatoEnv(a);cfg=train_config(a.steps_per_env)
 import importlib.metadata as meta
 manifest=dict(arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},versions={n:meta.version(n) for n in ['mujoco','mujoco-warp','mjlab','rsl-rl-lib']},model_sha256=hashlib.sha256((a.output/'rl_model.mjb').read_bytes()).hexdigest(),model_changes=['non-adjacent robot self-collision enabled; adjacent/welded links excluded; robot collision geoms use zero margin/gap for MJWarp self-collision compatibility; plant margins unchanged'],selective_reset_verified=env.selective_reset_verified,config=copy.deepcopy(cfg),joint_names=env.e.names,speed_caps=env.speed.tolist(),initial_distance_m=env.initial_distance.tolist(),initial_qpos_error=env.initial_qpos_error,physics_hz=120,control_hz=20,target='Tomato_06',scope='GT fixed scene; finite horizon; partial entry held 0.2s; all fruit displacement <=20mm; penetration <=2mm; not pedicel hook success; no randomization',sources=['https://github.com/mujocolab/mjlab','https://github.com/leggedrobotics/rsl_rl'])
 (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
 shutil.copy2(__file__,a.output/'train_source.py')
 alg=PPO.construct_algorithm(env.get_observations(),env,cfg,env.device)
 if a.checkpoint:
  checkpoint=torch.load(a.checkpoint,map_location=env.device,weights_only=False)
  if checkpoint['manifest']['model_sha256']!=manifest['model_sha256']:raise RuntimeError('Checkpoint physics model hash differs')
  alg.load(checkpoint['algorithm'],load_cfg=None,strict=True)
 initial=[p.detach().clone() for p in alg.actor.parameters()]
 setup=time.perf_counter()-start;print('[RL 준비]',a.output,'환경',a.num_envs,'준비초',round(setup,2),flush=True)
 baseline=evaluate(env,alg,a.output,'baseline') if a.compare_before else None
 if baseline is not None:(a.output/'baseline.json').write_text(json.dumps(baseline,indent=2))
 env.reset(torch.arange(a.num_envs,device=env.device));env.stats={k:0 for k in env.stats};env.record_evaluation=False
 torch.manual_seed(a.seed)
 obs=env.get_observations();rows=[];trainstart=time.perf_counter()
 for iteration in range(a.iterations):
  t=time.perf_counter();alg.train_mode();rew=0.
  with torch.inference_mode():
   for _ in range(a.steps_per_env):
    actions=alg.act(obs);obs,reward,done,extra=env.step(actions);alg.process_env_step(obs,reward,done,{})
    rew+=float(reward.mean())
   alg.compute_returns(obs)
  torch.cuda.synchronize();rollout=time.perf_counter()-t;t=time.perf_counter();loss=alg.update();torch.cuda.synchronize()
  if any(not torch.isfinite(p).all() for p in alg.actor.parameters()):raise RuntimeError('Nonfinite trained weights')
  row=dict(iteration=iteration,rollout_wall_s=rollout,update_wall_s=time.perf_counter()-t,mean_step_reward=rew/a.steps_per_env,loss={k:float(v) for k,v in loss.items()},**env.stats)
  elapsed=time.perf_counter()-trainstart
  row.update(completed_transitions=(iteration+1)*a.steps_per_env*a.num_envs,training_elapsed_s=elapsed,transitions_per_second=(iteration+1)*a.steps_per_env*a.num_envs/elapsed,estimated_remaining_minutes=elapsed/(iteration+1)*(a.iterations-iteration-1)/60)
  if a.train_seconds is not None:row['estimated_remaining_minutes']=min(row['estimated_remaining_minutes'],max(0.,a.train_seconds-elapsed)/60)
  rows.append(row)
  with (a.output/'progress.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
  print('[RL 학습]',json.dumps(row),flush=True)
  time_done=a.train_seconds is not None and time.perf_counter()-trainstart>=a.train_seconds
  if (iteration+1)%25==0 or iteration==a.iterations-1 or time_done:torch.save({'algorithm':alg.save(),'iteration':iteration,'manifest':manifest},a.output/f'model_{iteration+1:05d}.pt')
  if time_done:break
 trainwall=time.perf_counter()-trainstart
 changed=any(not torch.equal(x,p) for x,p in zip(initial,alg.actor.parameters()))
 evaluation=evaluate(env,alg,a.output,'evaluation')
 transitions=len(rows)*a.steps_per_env*a.num_envs
 summary=dict(setup_wall_s=setup,training_wall_s=trainwall,completed_updates=len(rows),stop_reason='time_budget' if a.train_seconds is not None and trainwall>=a.train_seconds else 'iteration_limit',transitions=transitions,transitions_per_second=transitions/trainwall,weights_changed=changed,training=rows[-1],evaluation=evaluation,total_wall_s=time.perf_counter()-start)
 if baseline is not None:
  summary['baseline_evaluation']=baseline
  summary['comparison']=dict(successes_before=baseline['successes'],successes_after=evaluation['successes'],mean_closest_distance_before_m=baseline['mean_closest_ring_center_distance_m'],mean_closest_distance_after_m=evaluation['mean_closest_ring_center_distance_m'],mean_max_displacement_before_m=float(np.mean(baseline['max_fruit_displacement_m'])),mean_max_displacement_after_m=float(np.mean(evaluation['max_fruit_displacement_m'])),note='identical initial scene; deterministic GPU clones, not independent generalization trials')
 (a.output/'summary.json').write_text(json.dumps(summary,indent=2));print('[RL 완료]',json.dumps(summary),flush=True)
if __name__=='__main__':main()
