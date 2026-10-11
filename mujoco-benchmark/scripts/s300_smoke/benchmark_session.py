"""Compare native worker counts from one measured harvest checkpoint.

The existing Session performs every control, contact and release operation.
This is a bounded performance comparison, not a new harvesting pipeline.
Each worker count starts from identical saved qpos/qvel/attachment state with
zero warmstart, because the existing replay NPZ does not save solver warmstart.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import mujoco as mj
import numpy as np

from plan_adapter import sample_plan
from run_harvest import Session, belt_velocity_from_physx, sha


def benchmark(trial, output, start_s=50., duration_s=2., workers=(0, 2, 4, 8),
              repeats=2, hz=480, model_dir=None, cpus=None):
    trial=Path(trial).resolve();output=Path(output).resolve()
    if output.exists():raise FileExistsError(output)
    if duration_s<=0 or start_s<0 or hz<=0 or repeats<1 or not workers or min(workers)<0:
        raise ValueError('Positive duration/frequency/repeats and nonnegative workers/start required')
    original_affinity=sorted(os.sched_getaffinity(0))
    if cpus is not None:
        if not cpus or not set(cpus).issubset(original_affinity):
            raise ValueError('Requested CPU affinity is outside the current allowed set')
        os.sched_setaffinity(0,set(cpus))
    try:
        summary=json.loads((trial/'summary.json').read_text())
        if sha(trial/'states.npz')!=summary['states_sha256']:
            raise ValueError('Source checkpoint states hash mismatch')
        model_dir=Path(model_dir or summary['model_dir']).resolve()
        if sha(model_dir/'scene.xml')!=summary['source_model_xml_sha256']:
            raise ValueError('Checkpoint and requested model XML differ')
        with np.load(trial/'states.npz') as archive:
            time_samples=archive['time'].copy()
            index=int(np.argmin(np.abs(time_samples-start_s)))
            checkpoint={key:archive[key][index].copy() for key in
                        ('qpos','qvel','eq_active','window_index')}
            actual_start=float(time_samples[index])
        selected=[r for r in summary['cycles'] if r.get('start_s',float('inf'))<=actual_start<=r.get('end_s',-1)]
        if len(selected)!=1:raise ValueError('Checkpoint must lie inside one executed harvesting cycle')
        cycle=selected[0]
        plan_path=trial/f'plant{cycle["window"]}_cycle{cycle["cycle"]:02d}'/'plan.json'
        plan=json.loads(plan_path.read_text())
        horizon=plan['settle_seconds']+sum(r['duration_s'] for r in plan['phases'])
        if actual_start-cycle['start_s']+duration_s>horizon+1e-9:
            raise ValueError('Benchmark would extend beyond its existing source plan')
        steps=int(round(duration_s*hz))
        reference=None;results=[];command_reference=None
        for threads in workers:
            for repeat in range(repeats):
                session=Session(model_dir,hz=hz,
                    gravity_compensation=summary['robot_bias_feedforward'],
                    released_gutter_diagnostic=summary.get('released_gutter_impacts_diagnostic',False))
                if session.data.qpos.shape!=checkpoint['qpos'].shape:
                    raise ValueError('Checkpoint/model coordinate sizes differ')
                session.data.qpos[:]=checkpoint['qpos']
                session.data.qvel[:]=checkpoint['qvel']
                session.data.eq_active[:]=checkpoint['eq_active']
                session.data.time=actual_start
                session.data.qacc_warmstart[:]=0
                for bid,eq in zip(session.at.ids,session.at.eqs):
                    if not session.data.eq_active[eq]:
                        adr=session.model.jnt_dofadr[session.model.body_jntadr[bid]]
                        session.model.dof_damping[adr+3:adr+6]=0
                session.set_window(int(checkpoint['window_index']))
                mj.mju_threadpool(session.data,threads)
                mj.mj_forward(session.model,session.data)
                session.rows.clear();session.contacts.clear();session.breaks.clear()
                session.next_log_s=float('inf')
                session.next_save_s=actual_start+1/30
                command_digest=hashlib.sha256()
                measured=[];qvel=[];attached=[];phases=set()
                cpu_begin=time.process_time();begin=time.perf_counter()
                try:
                    for tick in range(steps):
                        sample=sample_plan(plan,float(session.data.time-cycle['start_s']))
                        belt=belt_velocity_from_physx(sample['authored_belt_speed'])
                        command_digest.update(np.asarray(sample['command'],dtype='<f8').tobytes())
                        command_digest.update(np.asarray(belt,dtype='<f8').tobytes())
                        command_digest.update(sample['phase'].encode())
                        phases.add(sample['phase'])
                        session.step(sample['command'],belt,sample['phase'])
                        measured.append(session.data.qpos.copy())
                        qvel.append(session.data.qvel.copy())
                        attached.append(session.data.eq_active.copy())
                        if session.stop:break
                    wall=time.perf_counter()-begin;cpu=time.process_time()-cpu_begin
                finally:
                    # The official Python binding owns and replaces the pool.
                    mj.mju_threadpool(session.data,0)
                state=(np.asarray(measured),np.asarray(qvel),np.asarray(attached))
                commands=command_digest.hexdigest()
                if reference is None:
                    reference=state;command_reference=commands
                same_shapes=all(a.shape==b.shape for a,b in zip(state,reference))
                error=[float(np.max(np.abs(a-b))) if same_shapes and a.size else None
                       for a,b in zip(state[:2],reference[:2])]
                row=dict(workers=threads,repeat=repeat,steps=len(measured),wall_s=wall,
                    process_cpu_s=cpu,cpu_wall_ratio=cpu/wall,
                    simulated_s=float(session.data.time-actual_start),
                    realtime_factor=float(session.data.time-actual_start)/wall,
                    native_physics_s=session.physics_s,
                    max_qpos_difference_from_first=error[0],max_qvel_difference_from_first=error[1],
                    attachments_equal_to_first=same_shapes and bool(np.array_equal(state[2],reference[2])),
                    commands_sha256=commands,commands_identical_to_first=commands==command_reference,
                    phases=sorted(phases),stop=session.stop,break_events=session.breaks,
                    active_penetration_m=session.max_active_pen,
                    active_contact_rows=len(session.contacts),loaded_cached_model=session.loaded_cached_model)
                results.append(row)
                print(json.dumps(row,allow_nan=False),flush=True)
        record=dict(schema='s300-native-checkpoint-thread-benchmark-v1',mujoco_version=mj.__version__,
            trial=str(trial),model_dir=str(model_dir),source_states_sha256=summary['states_sha256'],
            model_xml_sha256=summary['source_model_xml_sha256'],source_plan_sha256=sha(plan_path),
            benchmark_code_sha256=sha(__file__),session_code_sha256=sha(Path(__file__).with_name('run_harvest.py')),
            checkpoint_index=index,requested_start_s=start_s,actual_start_s=actual_start,
            duration_s=duration_s,hz=hz,cpu_affinity=sorted(os.sched_getaffinity(0)),
            original_cpu_affinity=original_affinity,results=results,training_eligible=False,harvest_success=None,
            checkpoint_limitation='qpos/qvel/eq_active restored identically; solver warmstart is not in saved NPZ, so reset to zero for every case. Not claimed to reproduce the original continuation exactly.',
            comparison_scope='One measured closing interval; worker counts change only native execution scheduling. No physical gains/materials/colliders/caps are changed.',
            API_documentation='https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/functions.cc')
        output.parent.mkdir(parents=True,exist_ok=True)
        with output.open('x') as stream:json.dump(record,stream,indent=2,allow_nan=False);stream.write('\n')
        return record
    finally:
        if cpus is not None:os.sched_setaffinity(0,set(original_affinity))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('trial',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--start-s',type=float,default=50.);p.add_argument('--duration-s',type=float,default=2.)
    p.add_argument('--workers',type=int,nargs='+',default=[0,2,4,8]);p.add_argument('--repeats',type=int,default=2)
    p.add_argument('--hz',type=int,default=480);p.add_argument('--model-dir',type=Path)
    p.add_argument('--cpus',type=int,nargs='+')
    a=p.parse_args();benchmark(a.trial,a.output,a.start_s,a.duration_s,a.workers,a.repeats,a.hz,a.model_dir,a.cpus)
