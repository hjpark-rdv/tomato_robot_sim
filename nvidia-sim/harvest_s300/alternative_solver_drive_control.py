"""One actual pedicel body's three-axis spring/damping adapter sanity control."""
import argparse
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import newton
import numpy as np
import warp as wp
from scipy.spatial.transform import Rotation


def run(args):
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    original=ET.parse(Path(args.inputs)/'input_scene.xml').getroot()
    body=copy.deepcopy(next(b for b in original.iter('body') if b.get('name')=='Truss_00_Pedicel_proximal_05_0'))
    body.set('pos','0 0 0');body.set('quat','1 0 0 0')
    for c in list(body):
        if c.tag not in ('joint','inertial','geom'):body.remove(c)
    root=ET.Element('mujoco',model='actual_pedicel_drive_control')
    ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false')
    ET.SubElement(root,'option',gravity='0 0 0',timestep=str(args.dt),integrator='implicitfast')
    world=ET.SubElement(root,'worldbody');world.append(body)
    scene=output/'drive_fixture.xml';ET.ElementTree(root).write(scene,encoding='unicode')
    native=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(native)
    data.qpos[0]=.1
    rows=[]
    for _ in range(args.steps):
        mujoco.mj_step(native,data);rows.append(data.qpos.copy())
    report={'fixture_only':True,'source_body':body.get('name'),'initial_angle_rad':.1,
            'dt':args.dt,'steps':args.steps,'device':args.device,'native_final_joint_q':rows[-1].tolist(),
            'source_stiffness':native.jnt_stiffness.tolist(),
            'source_damping':native.dof_damping.tolist(),'methods':{}}
    actual={'native':np.asarray(rows)}
    for kind in ['xpbd','vbd']:
        builder=newton.ModelBuilder(gravity=(0,0,0));builder.balance_inertia=False
        builder.add_mjcf(str(scene),parse_visuals=False,parse_meshes=False,parse_sites=False,
            parse_mujoco_options=False,convert_3d_hinge_to_ball_joints=False,ctrl_direct=True)
        if kind=='vbd':builder.color()
        model=builder.finalize(device=args.device)
        inertia=np.asarray(builder.body_inertia).reshape(-1,3,3)
        model.body_inertia.assign(inertia.astype(np.float32))
        model.body_inv_inertia.assign(np.linalg.inv(inertia.astype(np.float64)).astype(np.float32))
        model.joint_target_ke.assign(native.jnt_stiffness.astype(np.float32))
        model.joint_target_kd.assign(native.dof_damping.astype(np.float32))
        q=model.joint_q.numpy();q[0]=.1;model.joint_q.assign(q)
        newton.eval_fk(model,model.joint_q,model.joint_qd,model)
        state0,state1=model.state(),model.state()
        newton.eval_fk(model,model.joint_q,model.joint_qd,state0);state1.assign(state0)
        control=model.control();control.joint_target_q.zero_()
        solver=(newton.solvers.SolverXPBD if kind=='xpbd' else newton.solvers.SolverVBD)(model,iterations=16)
        quats=[]
        for _ in range(args.steps):
            state0.clear_forces();solver.step(state0,state1,control,None,args.dt)
            state0,state1=state1,state0;quats.append(state0.body_q.numpy()[0,3:].copy())
        quats=np.asarray(quats)
        angles=Rotation.from_quat(quats).as_rotvec()
        actual[kind]=angles
        report['methods'][kind]={'finite':bool(np.isfinite(angles).all()),
            'final_body_rotvec_rad':angles[-1].tolist(),
            'max_abs_x_angle_error_to_native_rad':float(np.max(np.abs(angles[:,0]-actual['native'][:,0]))),
            'model_joint_type':model.joint_type.numpy().tolist()}
    np.savez_compressed(output/'actual_angles.npz',time=np.arange(1,args.steps+1)*args.dt,**actual)
    (output/'summary.json').write_text(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs',required=True);p.add_argument('--output',required=True)
    p.add_argument('--steps',type=int,default=160);p.add_argument('--dt',type=float,default=.0025)
    p.add_argument('--device',default='cuda:0')
    print(json.dumps(run(p.parse_args()),indent=2))
