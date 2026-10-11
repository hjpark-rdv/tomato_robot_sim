"""One-load MuJoCo trials, native conveyor contact, measured break adapter.

Native MuJoCo 3.15 geom_surfacevel is used for belts and tool tangential motion.
Prescribed mocap normal contact velocity is not represented; boundary fidelity
is limited. The discrete integrator handles damped connect constraints.
No direct attraction force, fruit teleport, timed detachment or visual stepping.
"""
import argparse
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

import mujoco as mj
import numpy as np
import psutil
from scipy.spatial.transform import Rotation as R, Slerp


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def belt_velocity_from_physx(command):
    """PhysX +local-Y moved the loaded control sphere toward CAD -Y.

    MuJoCo's native surfacevel is physical surface motion, so -Y reproduces
    that observed infeed. See native_positive_sign_verification.json (20261011).
    Copying numerical API signs would reverse the measured transport direction.
    """
    return -np.asarray(command)


class Attachments:
    def __init__(self, model, specs):
        self.specs = specs
        self.ids = np.array([model.body(s['name']).id for s in specs])
        self.parents = np.array([model.body(s['anchor']).id for s in specs])
        self.eqs = np.array([model.equality(s['name']+'_attachment').id for s in specs])
        self.rot0 = R.from_quat([s['local_rot0'] for s in specs])
        self.rot1 = R.from_quat([s['local_rot1'] for s in specs])
        self.k = np.array([s['stiffness'] for s in specs])
        self.damping = np.array([s['damping'] for s in specs])[:,None]
        self.dead = np.deg2rad([s['deadband_deg'] for s in specs])[:,None]
        self.force_limit = np.array([s['force_n'] for s in specs])
        self.torque_limit = np.array([s['torque_nm'] for s in specs])
        # Free-body angular damping is integrated implicitly by MuJoCo. Keeping
        # the tiny fruit's damping entirely in xfrc_applied is explicit and can
        # destabilize an otherwise idle fruit. The parent velocity term and
        # equal/opposite reaction below retain relative (not world) damping.
        for b,damping in zip(self.ids,self.damping[:,0]):
            adr=model.jnt_dofadr[model.body_jntadr[b]]
            model.dof_damping[adr+3:adr+6]=damping
        self.model=model

    def apply(self, d):
        q0 = R.from_quat(d.xquat[self.parents][:,[1,2,3,0]]) * self.rot0
        q1 = R.from_quat(d.xquat[self.ids][:,[1,2,3,0]]) * self.rot1
        angles = (q0.inv()*q1).as_rotvec()
        displacement = np.sign(angles)*np.maximum(np.abs(angles)-self.dead, 0)
        speed = q0.inv().apply(d.cvel[self.ids,:3]-d.cvel[self.parents,:3])
        torque_local = -self.k*displacement - self.damping*speed
        torque = q0.apply(torque_local) * d.eq_active[self.eqs,None]
        # Cancel the explicit self-damping contribution; dof_damping supplies
        # it implicitly. Parent reaction uses beginning-of-step velocity.
        implicit_self = self.damping*d.cvel[self.ids,:3]*d.eq_active[self.eqs,None]
        np.add.at(d.xfrc_applied[:,3:], self.ids, torque+implicit_self)
        np.add.at(d.xfrc_applied[:,3:], self.parents, -torque)
        return np.linalg.norm(torque,axis=1), np.rad2deg(np.linalg.norm(angles,axis=1))

    def measure_breaks(self, d, torque, phase):
        forces = np.zeros(len(self.eqs))
        eqrows = (d.efc_type == mj.mjtConstraint.mjCNSTR_EQUALITY)
        for i, eq in enumerate(self.eqs):
            rows = eqrows & (d.efc_id == eq)
            forces[i] = np.linalg.norm(d.efc_force[rows])
        broken = d.eq_active[self.eqs].astype(bool) & ((forces > self.force_limit) | (torque > self.torque_limit))
        events = []
        for i in np.flatnonzero(broken):
            d.eq_active[self.eqs[i]] = 0
            adr=self.model.jnt_dofadr[self.model.body_jntadr[self.ids[i]]]
            self.model.dof_damping[adr+3:adr+6]=0
            events.append(dict(name=self.specs[i]['name'], solver_time_s=float(d.time), phase=phase,
                               constraint_force_n=float(forces[i]), passive_torque_nm=float(torque[i]),
                               cause='force' if forces[i]>self.force_limit[i] else 'torque'))
        return forces, events


def resample_trace(trace, hz, mode, seconds=None):
    end = min(float(trace['time'][-1]), seconds) if seconds is not None else float(trace['time'][-1])
    t = np.arange(round(end*hz)+1)/hz
    source_t = trace['time']
    p = trace['poses']
    poses = np.empty((len(t),p.shape[1],7))
    for b in range(p.shape[1]):
        poses[:,b,:3] = np.array([np.interp(t,source_t,p[:,b,i]) for i in range(3)]).T
        poses[:,b,3:] = Slerp(source_t,R.from_quat(p[:,b,3:]))(t).as_quat()
    idx = np.clip(np.searchsorted(source_t,t,side='right')-1,0,len(source_t)-1)
    phases = trace['phase'][idx]
    speed = trace['belt_speed'][idx].copy()
    if mode == 'idle':
        poses[:] = poses[0]; speed[:] = 0; phases[:] = 'idle'
    if mode == 'off': speed[:] = 0
    velocity = np.diff(poses[:,:,:3],axis=0)*hz
    angular = np.empty_like(velocity)
    for b in range(p.shape[1]):
        rot = R.from_quat(poses[:,b,3:])
        angular[:,b] = (rot[1:]*rot[:-1].inv()).as_rotvec()*hz
    return t, poses, velocity, angular, speed, phases


def run(model_dir, output, hz=240, mode='inward', seconds=None, repeats=1):
    if hz<=0 or repeats<1 or (seconds is not None and seconds<=0):
        raise ValueError('Positive frequency, duration and repeat count required')
    output.mkdir(parents=True, exist_ok=False)
    code=output/'code';code.mkdir()
    code_hashes={}
    for source in Path(__file__).parent.glob('*.py'):
        shutil.copy2(source,code/source.name);code_hashes[source.name]=sha(source)
    (output/'invocation.json').write_text(json.dumps(dict(python=sys.executable,argv=sys.argv,
        mujoco_version=mj.__version__,code_sha256=code_hashes),indent=2)+'\n')
    meta = json.loads((model_dir/'export.json').read_text())
    if sha(model_dir/'scene.xml')!=meta['model_sha256'] or sha(model_dir/'tool_trace.npz')!=meta['trace_sha256']:
        raise ValueError('Model/trace SHA mismatch')
    start = time.perf_counter()
    model = mj.MjModel.from_xml_path(str(model_dir/'scene.xml'))
    model.opt.timestep=1/hz
    if not hasattr(model, 'geom_surfacevel'):
        raise RuntimeError('Native conveyor support (MuJoCo 3.13+) is required')
    model.flg_surfacevel=True
    load_s = time.perf_counter()-start
    attachments = Attachments(model, meta['attachments'])
    mj.mj_saveModel(model, str(output/'model.mjb'))
    toolids = [model.body(n).id for n in meta['tools']]
    mocaps = model.body_mocapid[toolids]
    belt_geoms = [np.flatnonzero(model.geom_bodyid==model.body('Tool_'+side+'_belt').id) for side in ['L','R']]
    belt_ids = set(np.concatenate(belt_geoms).tolist())
    tool_ids = set(toolids)
    tool_geoms = np.flatnonzero(np.isin(model.geom_bodyid,toolids))
    names = [model.geom(i).name for i in range(model.ngeom)]
    geometry = {g['name']:g for g in meta['geometry']}
    t, poses, velocity, angular, speed, phases = resample_trace(np.load(model_dir/'tool_trace.npz'),hz,mode,seconds)
    speed=belt_velocity_from_physx(speed)
    runs = []
    for rep in range(repeats):
        begin = time.perf_counter()
        attachments = Attachments(model, meta['attachments'])
        d = mj.MjData(model)
        model.geom_surfacevel[:]=0
        d.mocap_pos[mocaps]=poses[0,:,:3]; d.mocap_quat[mocaps]=poses[0][:,[6,3,4,5]]
        mj.mj_forward(model,d)
        # No preload copied from a different engine or a three-truss plant.
        # Source reference springs, gravity and a fixed main-stem root define this fixture.
        d.qfrc_applied[:]=0
        reset_s=time.perf_counter()-begin
        fruit_initial=d.xpos[attachments.ids].copy()
        qpos=[d.qpos.copy()];eq=[d.eq_active.copy()];saved_t=[0.]
        saved_mp=[d.mocap_pos.copy()];saved_mq=[d.mocap_quat.copy()]
        positions=[fruit_initial.copy()]
        contact_trace=[];breaks=[];metrics=[];tool_pen=0.;plant_pen=0.;released_pen=0.;worst=None
        first_violation=None;peak_force=0.;peak_belt_force=0.;physics_s=0.;adapter_s=0.;stop=None
        started=time.perf_counter();cpu=time.process_time()
        for k in range(len(t)-1):
            attached_at_solver_start=d.eq_active[attachments.eqs].copy()
            d.mocap_pos[mocaps]=poses[k,:,:3];d.mocap_quat[mocaps]=poses[k][:,[6,3,4,5]]
            # Native surface velocity is in GEOM local frame, not body local frame.
            # Build tool tangential motion including the rotating geom-origin offset.
            for ti,bid in enumerate(toolids):
                gids=tool_geoms[model.geom_bodyid[tool_geoms]==bid]
                brot=R.from_quat(poses[k,ti,3:])
                grot=brot*R.from_quat(model.geom_quat[gids][:,[1,2,3,0]])
                offset=brot.apply(model.geom_pos[gids])
                v=velocity[k,ti]+np.cross(angular[k,ti],offset)
                model.geom_surfacevel[gids,:3]=grot.inv().apply(v)
                model.geom_surfacevel[gids,3:]=grot.inv().apply(np.tile(angular[k,ti],(len(gids),1)))
            for side,gids in enumerate(belt_geoms):
                local=R.from_quat(model.geom_quat[gids][:,[1,2,3,0]]).inv().apply(np.tile([0,speed[k,side],0],(len(gids),1)))
                model.geom_surfacevel[gids,:3]+=local
            tick=time.perf_counter();mj.mj_step1(model,d);physics_s+=time.perf_counter()-tick
            tick=time.perf_counter()
            d.xfrc_applied[:]=0
            torques,angles=attachments.apply(d)
            adapter_s+=time.perf_counter()-tick
            tick=time.perf_counter();mj.mj_step2(model,d);physics_s+=time.perf_counter()-tick
            # Contact distances/Jacobians and these forces belong to pre-integration t[k].
            forces,events=attachments.measure_breaks(d,torques,str(phases[k]))
            for event in events:
                event['solver_time_s']=float(t[k]);event['release_effective_time_s']=float(d.time)
            breaks.extend(events)
            step_tool=0.;step_plant=0.;step_force=0.;step_belt=0.
            for ci,c in enumerate(d.contact):
                if c.efc_address<0:continue
                b1,b2=model.geom_bodyid[[c.geom1,c.geom2]]
                force=np.zeros(6);mj.mj_contactForce(model,d,ci,force)
                norm=float(np.linalg.norm(force[:3]));penetration=max(0.,-float(c.dist))
                released=any(b in attachments.ids and not attached_at_solver_start[np.flatnonzero(attachments.ids==b)[0]] for b in [b1,b2])
                tool = b1 in tool_ids or b2 in tool_ids
                if released:released_pen=max(released_pen,penetration)
                elif tool:step_tool=max(step_tool,penetration)
                else:step_plant=max(step_plant,penetration)
                if tool:step_force=max(step_force,norm)
                if c.geom1 in belt_ids or c.geom2 in belt_ids:step_belt=max(step_belt,norm)
                if penetration>max(tool_pen,plant_pen) and not released:
                    worst=dict(t=float(t[k]),geom1=names[c.geom1],geom2=names[c.geom2],
                               source1=geometry.get(names[c.geom1],{}).get('source',names[c.geom1]),
                               source2=geometry.get(names[c.geom2],{}).get('source',names[c.geom2]),penetration_m=penetration)
                if tool or penetration>.001:
                    contact_trace.append([float(t[k]),int(c.geom1),int(c.geom2),float(c.dist),norm,bool(released)])
            tool_pen=max(tool_pen,step_tool);plant_pen=max(plant_pen,step_plant)
            peak_force=max(peak_force,step_force);peak_belt_force=max(peak_belt_force,step_belt)
            if max(step_tool,step_plant)>.001 and first_violation is None:
                first_violation=dict(t=float(t[k]),phase=str(phases[k]),max_penetration_m=max(step_tool,step_plant))
            # Log the first 1 mm invalidity, then continue only for diagnostic visibility.
            # A 10 mm intersection or solver warning aborts; never counted as valid success.
            if max(step_tool,step_plant)>.010:stop='gross_overlap_10mm'
            if not np.isfinite(d.qpos).all() or np.max(np.abs(d.qvel))>1e4 or any(w.number for w in d.warning):stop='solver_instability'
            metrics.append([float(t[k]),step_tool,step_plant,step_force,step_belt,*forces,*torques,*angles])
            if (k+1)%max(1,round(hz/60))==0 or stop:
                mj.mj_kinematics(model,d)
                qpos.append(d.qpos.copy());eq.append(d.eq_active.copy());saved_t.append(float(d.time))
                saved_mp.append(d.mocap_pos.copy());saved_mq.append(d.mocap_quat.copy())
                positions.append(d.xpos[attachments.ids].copy())
            if (k+1)%max(1,round(hz*5))==0:
                print(f'{mode} {hz}Hz rep={rep} sim={d.time:.1f}s detached={len(breaks)} overlap={tool_pen*1000:.3f}mm',flush=True)
            if stop:break
        elapsed=time.perf_counter()-started
        cpu_s=time.process_time()-cpu
        folder=output/f'repeat_{rep:02d}';folder.mkdir()
        np.savez_compressed(folder/'states.npz',time=saved_t,qpos=qpos,eq_active=eq,fruit_positions=positions,
                            mocap_pos=saved_mp,mocap_quat=saved_mq)
        np.savez_compressed(folder/'physics_metrics.npz',values=metrics,
                            columns=['solver_time_s','active_tool_pen_m','active_plant_pen_m','tool_force_n','belt_force_n']+
                            ['force_'+s for s in meta['fruits']]+['torque_'+s for s in meta['fruits']]+['angle_deg_'+s for s in meta['fruits']])
        (folder/'contacts.json').write_text(json.dumps(dict(columns=['solver_time_s','geom1','geom2','signed_distance_m','force_norm_n','released_fruit'],rows=contact_trace)))
        summary=dict(mode=mode,hz=hz,repeat=rep,mujoco_version=mj.__version__,integrator='discrete',model_dir=str(model_dir),
                     model_sha256=meta['model_sha256'],trace_sha256=meta['trace_sha256'],load_s=load_s,reset_s=reset_s,
                     simulated_s=float(d.time),loop_wall_s=elapsed,physics_wall_s=physics_s,adapter_wall_s=adapter_s,
                     realtime_factor=float(d.time)/elapsed,process_cpu_percent=100*cpu_s/elapsed,
                     rss_mib=psutil.Process().memory_info().rss/2**20,stop=stop,completed_recorded_trace=stop is None,
                     first_1mm_violation=first_violation,
                     active_contact_criterion_passed=stop is None and first_violation is None,
                     full_physics_validated=False,
                     active_tool_penetration_m=tool_pen,active_plant_penetration_m=plant_pen,released_penetration_m=released_pen,
                     max_contact_force_n=peak_force,max_belt_contact_force_n=peak_belt_force,worst_pair=worst,
                     fruit_max_displacement_m=float(np.linalg.norm(np.asarray(positions)-fruit_initial,axis=2).max()),
                     breaks=breaks,nbody=model.nbody,nv=model.nv,ngeom=model.ngeom,
                     training_eligible=False,harvest_success=None,
                     scope='Prescribed recorded tool, selected truss/main stem; not full robot nor PhysX equivalence',
                     validity_note='Active-contact 1mm criterion retained. Released fruit reported separately. Diagnostic continuation after violation; no run establishes full physics equivalence or validated harvest.',
                     attachment_note='Uncalibrated MuJoCo passive spring/connect break adapter; source raw angular stiffness used in N m/rad. Self angular damping implicit; parent reaction sampled explicitly.',
                     boundary_note='Native surfacevel handles tangent velocity; prescribed mocap normal velocity and finite robot inertia absent.')
        summary['belt_sign_mapping']='MuJoCo local-Y = -PhysX authored local-Y; matched to measured PhysX control CAD-Y displacement, not API sign'
        (folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps({k:summary[k] for k in ['realtime_factor','simulated_s','stop','first_1mm_violation','breaks']},indent=2),flush=True)
        runs.append(summary)
    (output/'summary.json').write_text(json.dumps(dict(load_s=load_s,runs=runs),indent=2)+'\n')
    return runs


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('model',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--hz',type=int,default=240);p.add_argument('--mode',choices=['idle','off','inward'],default='inward')
    p.add_argument('--seconds',type=float);p.add_argument('--repeats',type=int,default=1)
    a=p.parse_args();run(a.model,a.output,a.hz,a.mode,a.seconds,a.repeats)
