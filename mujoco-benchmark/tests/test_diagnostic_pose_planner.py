"""Planner regression with frozen legacy output and opt-in per-waypoint rotation."""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
pytest.importorskip('torch')
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nvidia-sim/rl'))
import dataset_motion as motion
from scipy.spatial.transform import Rotation

class Tensor:
    def __init__(self,x):self.x=np.array(x)
    def __getitem__(self,i):return Tensor(self.x[i])
    def cpu(self):return self
    def numpy(self):return self.x
class Kin:
    bounds=np.array([[-2.]*7,[2.]*7])
    def fk(self,q):return q[:3].copy(),Rotation.from_rotvec(q[3:6])
    def ik(self,p,r,q):return np.r_[p,r.as_rotvec(),0.],0.,0.
class Checker:
    def check(self,q):return None

def inputs():
    env=SimpleNamespace(_target_geometry=lambda:[Tensor([[0.,0.,0.]])]*3,target_spec={'radius':.01},robot=SimpleNamespace(data=SimpleNamespace(joint_pos=Tensor([[0.]*7]))),step_dt=1/60,lift_id=0,cfg=SimpleNamespace(dataset_rise_speed=.002))
    return env,Kin(),Checker()
def old_waypoints(*args):
    return Rotation.identity(),[('preapproach',np.array([.03,0,0])),('entry',np.array([.04,.01,0])),('rise',np.array([.04,.01,.01]))],np.array([1.,0,0])

def test_legacy_frozen_commands_unchanged(monkeypatch):
    monkeypatch.setattr(motion,'make_waypoints',old_waypoints)
    planned,check=motion.plan(*inputs(),{'trajectory_mode':'staged6d'})
    golden=np.load(Path(__file__).with_name('fixtures')/'legacy_pose_plan.npz')
    assert check['passed'];np.testing.assert_array_equal(planned['commands'],golden['commands']);assert planned['phases']==golden['phases'].tolist()

def explicit():
    return dict(trajectory_mode='diagnostic_pose_waypoints_v1',diagnostic_only=True,training_eligible=False,pose_waypoints=[dict(phase='preapproach',ring_position_xyz=[.03,0,0],orientation_xyzw=[0,0,0,1]),dict(phase='seat',ring_position_xyz=[.04,.01,0],orientation_xyzw=Rotation.from_euler('z',12,degrees=True).as_quat().tolist()),dict(phase='hold',ring_position_xyz=[.04,.01,0],orientation_xyzw=Rotation.from_euler('z',12,degrees=True).as_quat().tolist(),minimum_seconds=1.1)])

def test_explicit_poses_change_real_commands_and_hold():
    planned,check=motion.plan(*inputs(),explicit());assert check['passed'];_,rot=Kin().fk(planned['commands'][-1]);assert abs(rot.as_euler('xyz',degrees=True)[2]-12)<1e-8
    assert sum(p=='hold' for p in planned['phases'])/60>=1.1
    np.testing.assert_allclose(planned['commands'][-1][:3],[.04,.01,0])

def test_existing_collision_check_still_blocks():
    env,kin,checker=inputs();checker.check=lambda q:('robot_a','robot_b')
    planned,check=motion.plan(env,kin,checker,explicit());assert planned is None and check['reason']=='self_collision'

@pytest.mark.parametrize('key,value',[('diagnostic_only',False),('training_eligible',True)])
def test_reject_training_mode(key,value):
    p=explicit();p[key]=value
    with pytest.raises(ValueError):motion.plan(*inputs(),p)
