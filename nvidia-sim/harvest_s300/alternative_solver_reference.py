"""Native CPU reference of alternative_solver_probe's isolated plant fixture.

This measures physical response only. It is neither a full harvest baseline nor
a GPU performance claim. The same source constant preload, scalar spring gains,
plant collider assets, initial state, and push pulse are retained.
"""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from alternative_solver_probe import _digest, extract_plant


def run(args):
    output=Path(args.output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    inputs=Path(args.inputs).resolve()
    source=mujoco.MjModel.from_binary_path(str(inputs/'input_model.mjb'))
    initial=np.load(inputs/'initial_state.npz')
    fixture=output/'plant_fixture.xml'
    scope=extract_plant(inputs/'input_scene.xml',fixture)
    t=time.perf_counter()
    model=mujoco.MjModel.from_xml_path(str(fixture))
    model.opt.timestep=args.dt
    data=mujoco.MjData(model)
    preload=np.zeros(model.nv)
    for j in range(model.njnt):
        name=model.joint(j).name
        old=source.joint(name).id
        nq=7 if model.jnt_type[j]==mujoco.mjtJoint.mjJNT_FREE else 1
        nv=6 if nq==7 else 1
        qa,va=model.jnt_qposadr[j],model.jnt_dofadr[j]
        oldqa,oldva=source.jnt_qposadr[old],source.jnt_dofadr[old]
        data.qpos[qa:qa+nq]=initial['qpos'][oldqa:oldqa+nq]
        data.qvel[va:va+nv]=initial['qvel'][oldva:oldva+nv]
        preload[va:va+nv]=initial['plant_preload'][oldva:oldva+nv]
        model.dof_damping[va:va+nv]=initial['runtime_dof_damping'][oldva:oldva+nv]
    mujoco.mj_forward(model,data)
    names=[model.body(i).name for i in range(1,model.nbody)]
    pose0=np.c_[data.xpos[1:],data.xquat[1:][:,[1,2,3,0]]]
    sites=[]
    for connect in ET.parse(fixture).getroot().find('equality'):
        sites.append((model.site(connect.get('site1')).id,model.site(connect.get('site2')).id))
    fruit=model.body(args.push_body).id
    steps=round(args.seconds/args.dt)
    poses=np.empty((steps,model.nbody-1,7))
    velocities=np.empty((steps,model.nbody-1,6))
    gaps=np.empty((steps,len(sites)))
    initialization=time.perf_counter()-t
    t=time.perf_counter()
    for k in range(steps):
        data.qfrc_applied[:]=preload
        data.xfrc_applied[:]=0
        if args.seconds*.25<=k*args.dt<args.seconds*.5:
            data.xfrc_applied[fruit,0]=args.push_force
        mujoco.mj_step(model,data)
        mujoco.mj_forward(model,data)
        poses[k]=np.c_[data.xpos[1:],data.xquat[1:][:,[1,2,3,0]]]
        velocities[k]=data.cvel[1:]
        gaps[k]=[np.linalg.norm(data.site_xpos[a]-data.site_xpos[b]) for a,b in sites]
    elapsed=time.perf_counter()-t
    displacement=np.linalg.norm(poses[:,:,:3]-pose0[None,:,:3],axis=-1)
    report=dict(schema='s300_alternative_solver_native_reference_v1',
        solver='native MuJoCo CPU fixture reference',training_eligible=False,harvest_success=None,
        fixture_only=True,fixture=scope,source_model_sha256=_digest(inputs/'input_model.mjb'),
        fixture_sha256=_digest(fixture),code_sha256=_digest(__file__),
        dt=args.dt,physics_seconds=steps*args.dt,physics_steps=steps,
        initialization_wall_s=initialization,physics_wall_s=elapsed,fixture_RTF=steps*args.dt/elapsed,
        finite=bool(np.isfinite(poses).all() and np.isfinite(velocities).all()),
        max_idle_body_displacement_m=float(np.max(displacement[:steps//4])),
        max_idle_attachment_gap_m=float(np.max(gaps[:steps//4])),
        max_attachment_gap_m=float(np.max(gaps)),
        push_body=args.push_body,push_force_n=args.push_force,
        push_body_max_displacement_m=float(np.max(displacement[:,fruit-1])),
        push_body_final_displacement_m=float(displacement[-1,fruit-1]),
        limits=['Isolated source plant fixture, no robot/background',
                'Belt and passive fruit angular-spring/release adapter excluded from this reference and alternatives',
                'Native source CONNECT vs alternative BALL-loop solver approximation differ'])
    np.savez_compressed(output/'actual_states.npz',time=np.arange(1,steps+1)*args.dt,
        body_names=np.array(names),body_q=poses,body_qd=velocities,initial_body_q=pose0,
        attachment_gap_m=gaps)
    (output/'summary.json').write_text(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--dt',type=float,default=.0025)
    p.add_argument('--seconds',type=float,default=1.)
    p.add_argument('--push-body',default='Truss_00_Fruit_05')
    p.add_argument('--push-force',type=float,default=.05)
    print(json.dumps(run(p.parse_args()),indent=2))
