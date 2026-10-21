"""Bounded raise/lower regression for the corrected CAD mount (no harvesting).

Uses the existing USD->MJCF articulation adapter and Newton GPU adapter.
Does not change gains, mass, contact properties or teleport bodies while running.
"""
import argparse
import json
from pathlib import Path
import time

import mujoco as mj
import numpy as np


def run(model_dir, output, gpu=False):
    output.mkdir(parents=True, exist_ok=False)
    meta=json.loads((model_dir/'robot.json').read_text())
    model=mj.MjModel.from_xml_path(str(model_dir/'scene.xml'))
    data=mj.MjData(model)
    for name,value in meta['initial_q'].items():
        data.qpos[model.joint(name).qposadr[0]]=value
    controls=np.array([data.qpos[model.joint(model.actuator(i).trnid[0]).qposadr[0]] for i in range(model.nu)])
    data.ctrl[:]=controls
    mj.mj_forward(model,data)
    lid=model.joint('farmily_lift_height_joint').qposadr[0]
    aid=model.actuator('drive_farmily_lift_height_joint').id
    height=float(data.qpos[lid])
    times=np.array([0,2,8,10,16,18,21,23.])
    heights=np.array([height,height,.70,.70,.05,.05,height,height])
    sim=None;engine='native_mujoco_cpu';setup_start=time.perf_counter()
    if gpu:
        import warp as wp
        import mujoco_warp as mjw
        from newton_mjcf import create_newton_mjcf
        sim=create_newton_mjcf(model_dir/'scene.xml',source_model=model,native_observables_only=True,nconmax=1024,njmax=4096)
        sim.solver.mjw_data.qpos.assign(data.qpos[None,:].astype(np.float32))
        sim.solver.mjw_data.qvel.assign(data.qvel[None,:].astype(np.float32))
        sim.control.mujoco.ctrl.assign(controls.astype(np.float32))
        mjw.forward(sim.solver.mjw_model,sim.solver.mjw_data)
        wp.synchronize()
        engine='Newton / MuJoCo Warp / CUDA'
    setup=time.perf_counter()-setup_start
    names=list(meta['body_names']);ids=np.array([model.body(meta['body_names'][p]).id for p in names])
    robot_joint_names=[meta['rail_joint_name'],*meta['arm_joint_names'],*meta['jaw_joint_names']]
    qids=np.array([model.joint(n).qposadr[0] for n in robot_joint_names])
    states=[];vel=[];poses=[];stamps=[];commands=[];penetrations=[]
    worst_pair=None;max_pen=0.;stop=None;start=time.perf_counter()
    for step in range(round(times[-1]/model.opt.timestep)+1):
        now=step*model.opt.timestep
        # GPU arrays are copied for this bounded mechanism diagnostic only.
        if gpu:
            data.qpos[:]=sim.solver.mjw_data.qpos.numpy()[0]
            data.qvel[:]=sim.solver.mjw_data.qvel.numpy()[0]
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            stop='nonfinite_state';break
        # Post-step poses/contact distances are evaluated on the measured state.
        # These distance checks are not substituted for GPU contact forces.
        mj.mj_forward(model,data)
        for c in data.contact:
            if -c.dist>max_pen:
                max_pen=float(-c.dist);worst_pair=[model.geom(c.geom1).name,model.geom(c.geom2).name]
        if max_pen>.001:
            stop='penetration_above_existing_1mm';break
        cmd=controls.copy();cmd[aid]=np.interp(now,times,heights)
        if step%8==0 or step==round(times[-1]/model.opt.timestep):
            states.append(data.qpos.copy());vel.append(data.qvel.copy())
            poses.append(np.c_[data.xpos[ids],data.xquat[ids][:,[1,2,3,0]]])
            stamps.append(now);commands.append(cmd);penetrations.append(max_pen)
        if step==round(times[-1]/model.opt.timestep):break
        if gpu:
            sim.control.mujoco.ctrl.assign(cmd.astype(np.float32));sim.step(model.opt.timestep)
        else:
            data.ctrl[:]=cmd;mj.mj_step(model,data)
        if np.any(data.warning.number):stop='simulator_warning';break
    if gpu:wp.synchronize()
    elapsed=time.perf_counter()-start
    q=np.array(states);v=np.array(vel);p=np.array(poses);t=np.array(stamps);c=np.array(commands)
    np.savez_compressed(output/'states.npz',time=t,qpos=q,qvel=v,poses=p,body_paths=np.array(names),robot_q=q[:,qids],command=c)
    lift_delta=q[:,lid]-q[0,lid]
    carriage=names.index('/World/HarvestRobot/Robot/farmily_lift_platform')
    chassis=names.index('/World/HarvestRobot/Robot/world')
    translation=p[:,carriage,:3]-p[0,carriage,:3]
    holds=(t>9)&(t<10)|(t>17)&(t<18)|(t>22)
    result=dict(engine=engine,scope='robot-only lift mechanism regression; farm harvest not revalidated',completed=stop is None,
        simulated_s=float(t[-1]),elapsed_s=elapsed,setup_s=setup,stop_reason=stop,
        command_height_range_m=[float(c[:,aid].min()),float(c[:,aid].max())],
        measured_height_range_m=[float(q[:,lid].min()),float(q[:,lid].max())],
        maximum_hold_tracking_error_m=float(np.max(np.abs(q[holds,lid]-c[holds,aid]))) if np.any(holds) else None,
        moving_carriage_height_consistency_m=float(np.max(np.abs(translation[:,2]-lift_delta))),
        moving_carriage_xy_change_m=float(np.max(np.linalg.norm(translation[:,:2],axis=1))),
        fixed_world_displacement_m=float(np.max(np.linalg.norm(p[:,chassis,:3]-p[0,chassis,:3],axis=1))),
        maximum_penetration_m=max_pen,worst_contact_pair=worst_pair,physics_dt_s=float(model.opt.timestep),
        finite_force_limited_source_drives=True,body_teleport_during_simulation=False,
        contact_distance_evaluation='post-step measured qpos, native mj_forward',
        training_eligible=False,harvest_success=None)
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    if stop:raise RuntimeError(stop)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('model',type=Path);p.add_argument('output',type=Path);p.add_argument('--gpu',action='store_true');a=p.parse_args()
    run(a.model,a.output,a.gpu)
