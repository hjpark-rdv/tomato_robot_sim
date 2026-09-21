"""CPU-only planning service for fixed-scene GPU candidate experiments.

The simulator exports its actual collision model and reset inputs once. Workers
run the SAME planner without Kit/PhysX/CUDA. Atomic results are consumed in the
original candidate order; no physics runs inside a worker.
"""
import os
from pathlib import Path
import pickle
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np


def planning_inputs(slot):
    return dict(geometry=[v[0].cpu().numpy().copy() for v in slot._target_geometry()],
                start=slot.robot.data.joint_pos[0].cpu().numpy().copy(),
                step_dt=slot.step_dt, lift_id=slot.lift_id,
                rise_speed=getattr(slot.cfg,'dataset_rise_speed',.002),
                pull_speed=getattr(slot.cfg,'dataset_pull_speed',.004))


def validate_inputs(reference, actual):
    for key in ('geometry','start'):
        if not np.allclose(reference[key],actual[key],rtol=0,atol=1e-6):
            raise RuntimeError('Preplanned path inputs differ from reset slot: '+key)
    for key in ('step_dt','lift_id','rise_speed','pull_speed'):
        if reference[key]!=actual[key]:raise RuntimeError('Preplanned path setting differs: '+key)


def export_model(slot, checker):
    kin=slot.pose_search_kin
    return dict(inputs=planning_inputs(slot),kinematics={k:getattr(kin,k) for k in ('chain','world','bounds')},
                collision=checker.export_model())


def restore_model(model):
    import torch
    from hook_motion import RobotKinematics
    from pose_collision import SelfCollisionCheck
    data=model['inputs']
    env=SimpleNamespace(robot=SimpleNamespace(data=SimpleNamespace(joint_pos=torch.from_numpy(data['start'][None]))),
        step_dt=data['step_dt'],lift_id=data['lift_id'],
        cfg=SimpleNamespace(dataset_rise_speed=data['rise_speed'],dataset_pull_speed=data['pull_speed']))
    geometry=[torch.from_numpy(v[None]) for v in data['geometry']]
    env._target_geometry=lambda:geometry
    kin=RobotKinematics.__new__(RobotKinematics)
    for key,value in model['kinematics'].items():setattr(kin,key,value)
    return env,kin,SelfCollisionCheck.from_model(model['collision'])


def atomic_pickle(path, value):
    temporary=path.with_suffix('.tmp')
    with temporary.open('wb') as stream:pickle.dump(value,stream,protocol=pickle.HIGHEST_PROTOCOL)
    temporary.replace(path)


class PlanningService:
    def __init__(self, folder, model, candidates, workers):
        import json
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=False)
        self.inputs=model['inputs'];self.process=None;self.log=None
        atomic_pickle(self.folder/'model.pkl',model)
        (self.folder/'candidates.json').write_text(json.dumps(candidates))
        env=os.environ.copy()
        for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
            env[name]='1'
        # Spawn from a clean Python entry point: spawning from the Kit main
        # module would relaunch Isaac in every child.
        self.log=(self.folder/'run.log').open('w')
        try:
            self.process=subprocess.Popen([sys.executable,'-u',str(Path(__file__).with_name('gpu_planning_worker.py')),
                '--folder',str(self.folder.resolve()),'--workers',str(workers),'--parent-pid',str(os.getpid())],env=env,
                stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        except BaseException:
            self.log.close();raise

    def take(self, candidate_id, slot):
        validate_inputs(self.inputs,planning_inputs(slot))
        path=self.folder/(candidate_id+'.pkl');began=time.monotonic();last=began
        while not path.exists():
            if self.process.poll() is not None:
                raise RuntimeError('CPU planner stopped before '+candidate_id+'; see '+str(self.folder/'run.log'))
            if time.monotonic()-last>15:
                print('[DATASET PLANNING] waiting for',candidate_id,'CPU workers active',flush=True);last=time.monotonic()
            time.sleep(.02)
        with path.open('rb') as stream:value=pickle.load(stream)
        return value, time.monotonic()-began

    def close(self):
        if self.process is not None and self.process.poll() is None:
            try:os.killpg(self.process.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:os.killpg(self.process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                self.process.wait()
        if self.log:self.log.close()
