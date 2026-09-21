"""Headless physics, state recording, then offline checks: separate timings."""
import argparse,json,time,os,hashlib
from pathlib import Path
import numpy as np
import mujoco as mj
import psutil
from scipy.spatial.transform import Rotation as R
from suite import resample,RING
from metrics import audit
HOME=Path(__file__).resolve().parents[1]

class Engine:
    def __init__(self,model,reference,hz=120,integrator='implicitfast',solver='Newton'):
        self.ref=reference;self.model=m=mj.MjModel.from_xml_path(str(model));self.data=d=mj.MjData(m)
        m.opt.timestep=1/hz;m.opt.integrator=getattr(mj.mjtIntegrator,'mjINT_'+integrator.upper());m.opt.solver=getattr(mj.mjtSolver,'mjSOL_'+solver.upper())
        self.ids=np.array([m.body(b['name']).id for b in reference['bodies']]+[m.body('Hook').id]);self.hook=m.body('Hook').mocapid[0]
        self.geomnames={i:m.geom(i).name for i in range(m.ngeom)};self.hookgeoms={m.geom(s['name']).id for s in reference['shapes'] if s['body']==reference['tool_path']}
        self.preload=np.zeros(m.nv);mj.mj_forward(m,d)
        # Initial gravity load on the articulated tree, plus free fruits' loads
        # projected onto their attachment Jacobians. Fixed in time thereafter.
        self.preload[:]=d.qfrc_bias
        for fruit in reference['fruit_specs']:
            f=m.body(fruit['name']).id;a=m.body(fruit['anchor'].rsplit('/',1)[-1]).id
            if not m.body_jntnum[f]:continue # optimized rigid fruit: already in tree gravity
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mj.mj_jac(m,d,jp,jr,d.xipos[f],a)
            self.preload-=jp.T@(m.body_mass[f]*m.opt.gravity)
        for j in range(m.njnt):
            if m.jnt_type[j]==mj.mjtJoint.mjJNT_FREE:self.preload[m.jnt_dofadr[j]:m.jnt_dofadr[j]+6]=0
        self.reset()
    def reset(self):
        mj.mj_resetData(self.model,self.data);self.data.qfrc_applied[:]=self.preload;mj.mj_forward(self.model,self.data)
    def hook_pose(self,pose):
        rot=R.from_quat(pose[3:]);self.data.mocap_pos[self.hook]=pose[:3]-rot.apply(RING);self.data.mocap_quat[self.hook]=np.asarray(pose)[[6,3,4,5]]
    def poses(self):
        d=self.data
        return np.column_stack([d.xpos[self.ids],d.xquat[self.ids][:,[1,2,3,0]]])
    def contacts(self):
        return [[self.geomnames[c.geom1],self.geomnames[c.geom2]] for c in self.data.contact if c.efc_address>=0 and (c.geom1 in self.hookgeoms or c.geom2 in self.hookgeoms)]

def run_trial(engine,trial,hz,output,recovery=2.):
    times,hook=resample(trial,hz,recovery);engine.reset();engine.hook_pose(hook[0]);mj.mj_forward(engine.model,engine.data)
    poses=[engine.poses()];contacts=[];qpos=[engine.data.qpos.copy()];physics=0.;cpu=time.process_time();begin=time.perf_counter();unstable=False
    for k,pose in enumerate(hook[1:]):
        engine.hook_pose(pose);start=time.perf_counter();mj.mj_step(engine.model,engine.data);physics+=time.perf_counter()-start
        contacts.append(engine.contacts());mj.mj_kinematics(engine.model,engine.data);poses.append(engine.poses());qpos.append(engine.data.qpos.copy())
        if not np.isfinite(engine.data.qpos).all() or np.max(np.abs(engine.data.qvel))>1e4 or any(w.number for w in engine.data.warning):
            unstable=True;break
    elapsed=time.perf_counter()-begin;cpu=time.process_time()-cpu
    times=times[:len(poses)];poses=np.array(poses);sim_s=float(times[-1]);start=time.perf_counter()
    metrics=audit(engine.ref,poses,times,contacts);audit_s=time.perf_counter()-start
    row=dict(engine='MuJoCo',version=mj.__version__,trajectory_id=trial['id'],category=trial.get('category','recorded'),hz=hz,
        integrator=str(engine.model.opt.integrator),solver=str(engine.model.opt.solver),iterations=int(engine.model.opt.iterations),
        requested_sim_s=float(resample(trial,hz,recovery)[0][-1]),simulated_s=sim_s,wall_s=elapsed,physics_only_wall_s=physics,
        rtf=sim_s/elapsed,physics_only_rtf=sim_s/physics,steps_per_second=(len(poses)-1)/physics,
        process_cpu_percent=cpu/elapsed*100,rss_mib=psutil.Process().memory_info().rss/2**20,audit_wall_s=audit_s,
        unstable=unstable,break_enabled=False,**metrics)
    if unstable:row['result']='unstable'
    folder=output/trial['id'];folder.mkdir(parents=True,exist_ok=True)
    (folder/'trajectory.json').write_text(json.dumps(trial))
    np.savez_compressed(folder/'states.npz',times_s=times,poses=poses,qpos=qpos,hook=hook[:len(poses)])
    (folder/'result.json').write_text(json.dumps(row,indent=2));(folder/'contacts.json').write_text(json.dumps(contacts))
    return row

def main():
    p=argparse.ArgumentParser();p.add_argument('--reference',type=Path,default=HOME/'assets/reference/reference.json');p.add_argument('--model',type=Path,default=HOME/'models/plant_original_equivalent.xml');p.add_argument('--suite',type=Path,default=HOME/'assets/reference/suite.json')
    p.add_argument('--hz',type=int,default=120);p.add_argument('--count',type=int,default=1);p.add_argument('--start',type=int,default=0);p.add_argument('--integrator',default='implicitfast',choices=['implicitfast','implicit','Euler']);p.add_argument('--solver',default='Newton',choices=['Newton','CG','PGS']);p.add_argument('--output',type=Path,required=True);p.add_argument('--recovery',type=float,default=2.);a=p.parse_args()
    trials=json.loads(a.suite.read_text())[a.start:a.start+a.count]
    if len(trials)!=a.count:raise ValueError('Not enough distinct trajectories; refusing to call repeats unique trials')
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'inputs.json').write_text(json.dumps(dict(suite_sha256=hashlib.sha256(a.suite.read_bytes()).hexdigest(),model_sha256=hashlib.sha256(a.model.read_bytes()).hexdigest(),reference_sha256=hashlib.sha256(a.reference.read_bytes()).hexdigest(),trials=[t['id'] for t in trials]),indent=2))
    begin=time.perf_counter();e=Engine(a.model,json.loads(a.reference.read_text()),a.hz,a.integrator,a.solver);compile_s=time.perf_counter()-begin
    t=time.perf_counter()
    for _ in range(100):e.reset()
    reset_s=time.perf_counter()-t
    results=[];begin=time.perf_counter()
    for trial in trials:
        row=run_trial(e,trial,a.hz,a.output,a.recovery);results.append(row)
        summary=dict(engine='MuJoCo',version=mj.__version__,model=str(a.model),hz=a.hz,count=len(results),requested_count=a.count,compile_s=compile_s,
            reset_100_s=reset_s,reset_mean_s=reset_s/100,total_wall_s=time.perf_counter()-begin,results=results)
        (a.output/'summary.json').write_text(json.dumps(summary,indent=2))
        print('[MuJoCo 결과]',len(results),'/',a.count,trial['id'],row['result'],'RTF',round(row['rtf'],2),'관통 mm',round(row['max_penetration_m']*1000,3),flush=True)

if __name__=='__main__':main()
