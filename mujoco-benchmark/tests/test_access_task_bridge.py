"""Exercise the real legacy task planner, with a small analytic robot fixture."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
pytest.importorskip('torch')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nvidia-sim/rl'))
from scipy.spatial.transform import Rotation
from plan_access_then_task import access_boundary,suffix_from_branch,trace_rows


class Tensor:
    def __init__(self,x):self.x=np.asarray(x)
    def __getitem__(self,i):return Tensor(self.x[i])
    def cpu(self):return self
    def numpy(self):return self.x
class Kin:
    bounds=np.array([[-2.]*7,[2.]*7])
    def fk(self,q):return np.asarray(q[:3]).copy(),Rotation.from_rotvec(q[3:6])
    def ik(self,p,r,q):return np.r_[p,r.as_rotvec(),q[6]],0.,0.


def test_new_branch_is_used_for_task_without_mutating_original_start():
    env=SimpleNamespace(_target_geometry=lambda:[Tensor([[0.,0.,0.]])]*3,
        target_spec={'radius':.01},robot=SimpleNamespace(data=SimpleNamespace(joint_pos=Tensor([[0.]*7]))),
        step_dt=1/60,lift_id=0,cfg=SimpleNamespace(dataset_rise_speed=.002))
    rows=[dict(phase=phase,ring_position_xyz=p,orientation_xyzw=[0,0,0,1]) for phase,p in
        [('preapproach',[.01,0,0]),('approach',[.02,0,0]),('insert',[.02,.01,0]),('hold',[.02,.01,0])]]
    params=dict(trajectory_mode='diagnostic_pose_waypoints_v1',diagnostic_only=True,training_eligible=False,pose_waypoints=rows)
    cut,gate=access_boundary(params);assert cut==2 and gate[0]=='approach'
    q=np.array([.02,0,0,0,0,0,.7])
    planned,check,p=suffix_from_branch(env,Kin(),SimpleNamespace(check=lambda q:None),params,q,cut)
    assert check['passed'] and (planned['commands'][:,6]==.7).all()
    assert 'preapproach' not in planned['phases']
    assert np.array_equal(env.robot.data.joint_pos.cpu().numpy(),[[0.]*7])
    assert params['pose_waypoints']==rows
    tr=trace_rows(np.zeros(7),planned['commands'],planned['phases'])
    assert tr[0]['phase']=='ready'
