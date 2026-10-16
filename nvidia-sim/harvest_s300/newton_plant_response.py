"""Offline reference response checks for coordinate variants, not throughput runs."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time

import mujoco as mj
import numpy as np

from newton_inputs import sha256
from run_smoke import Attachments


def sample_post_step_poses(model, data, ids):
    """Pair integrated positions with data.time without recomputing forces."""
    mj.mj_kinematics(model, data)
    return data.xpos[ids].copy(), data.xquat[ids].copy()


def response(inputs, output, *, hz=400):
    inputs, output = Path(inputs), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    started=time.perf_counter()
    model=mj.MjModel.from_binary_path(str(inputs/'input_model.mjb'))
    initial=dict(np.load(inputs/'initial_state.npz',allow_pickle=False))
    metadata=json.loads((inputs/'source_export.json').read_text())
    model.opt.timestep=1/hz
    model.dof_damping[:]=initial['runtime_dof_damping']
    attach=Attachments(model,metadata['attachments'])
    watch=['Stem_MainStem_9','Stem_MainStem_15',*[s['anchor'] for s in metadata['attachments']],
           *[s['name'] for s in metadata['attachments']]]
    ids=np.array([model.body(n).id for n in watch])
    tests=[('idle',None,np.zeros(3),np.zeros(3)),
           ('stem_push','Stem_MainStem_9',np.array([.2,0,0]),np.zeros(3)),
           ('fruit_pull','Truss_00_Fruit_04',np.array([.05,0,0]),np.zeros(3)),
           ('fruit_twist','Truss_00_Fruit_04',np.zeros(3),np.array([0,0,.002]))]
    rows=[]
    previous=mj.get_mjcb_control()
    try:
        for label,body,force,torque in tests:
            mj.set_mjcb_control(None)
            data=mj.MjData(model)
            data.qpos[:]=initial['qpos'];data.qvel[:]=initial['qvel']
            data.eq_active[:]=initial['eq_active'];data.ctrl[:]=initial['ctrl']
            mj.mj_forward(model,data)
            bodyid=model.body(body).id if body else -1
            reference=data.xpos[ids].copy()
            times,poses,quats,forces=[],[],[],[]
            max_constraint_force=0.;max_penetration=0.;max_tau=0.
            def control(m,d):
                d.qfrc_applied[:]=initial['plant_preload']
                d.ctrl[:]=initial['ctrl']
                dofs=initial['robot_dof_addresses'];aids=initial['actuator_ids']
                d.ctrl[aids]+=d.qfrc_bias[dofs]/m.actuator_gainprm[aids,0]
                d.xfrc_applied[:]=0
                attach.apply(d)
                if bodyid>=0 and .5 <= d.time < 1.5:
                    d.xfrc_applied[bodyid,:3]+=force
                    d.xfrc_applied[bodyid,3:]+=torque
            mj.set_mjcb_control(control)
            finite=True
            for step in range(round(3*hz)):
                mj.mj_step(model,data)
                if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                    finite=False;break
                max_constraint_force=max(max_constraint_force,float(np.max(np.abs(data.efc_force),initial=0)))
                if data.ncon:max_penetration=max(max_penetration,float(-np.min(data.contact.dist,initial=0)))
                if step%4==3:
                    # mj_step leaves xpos/xquat at its pre-integration evaluation.
                    # Refresh only kinematics; preserve the solved force evidence.
                    position, quaternion = sample_post_step_poses(model,data,ids)
                    times.append(float(data.time));poses.append(position);quats.append(quaternion)
            pos=np.array(poses);displacement=np.linalg.norm(pos-reference[None],axis=-1)
            row=dict(test=label,applied_body=body,force_n=force.tolist(),torque_nm=torque.tolist(),
                load_interval_s=[.5,1.5],finite=finite,steps=step+1,
                max_penetration_m=max_penetration,max_absolute_scalar_constraint_row_force=max_constraint_force,
                constraint_force_note='Mixed constraint row maximum, not a contact or attachment force norm.',
                peak_displacement_m=dict(zip(watch,displacement.max(axis=0).tolist())),
                residual_displacement_m=dict(zip(watch,displacement[-1].tolist())),
                attachment_connections_active=bool(np.all(data.eq_active)),
                release_not_executed='Response diagnostic retains constraints; original thresholds are not bypassed in harvest runs.')
            rows.append(row)
            np.savez_compressed(output/(label+'.npz'),time=times,positions=pos,quaternions_wxyz=quats,
                                reference_positions=reference,names=np.array(watch))
    finally:
        mj.set_mjcb_control(previous)
    report=dict(schema='s300-plant-response-v2',inputs=str(inputs),mjb_sha256=sha256(inputs/'input_model.mjb'),
        code_sha256=sha256(__file__),pose_timestamp='post-integration kinematics refreshed at data.time; forces remain from the solved step',
        backend='native MuJoCo CPU offline model-response comparison; not GPU throughput evidence',
        hz=hz,wall_seconds=time.perf_counter()-started,tests=rows,
        training_eligible=False,harvest_success=None)
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('inputs',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();print(json.dumps(response(a.inputs,a.output),indent=2))
