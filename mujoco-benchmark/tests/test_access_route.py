"""Small independent route/continuation tests, not greenhouse success counts."""
import copy
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from access_route import (AccessConfig,Budget,AccessBudget,edge_valid,ik_goals,connect,
    retime_route,staging_index,PinnedFirstIK,access_then_local)
from plan_access_candidate import FrozenValidity


class Oracle:
    def __init__(self,f=lambda q:True,config=None):self.f=f;self.budget=Budget(config or AccessConfig())
    def __call__(self,q):self.budget.state();return self.f(np.asarray(q))


class Kin:
    bounds=np.array([[-2.,-2.],[2.,2.]])
    def fk(self,q):return np.array([q[0],abs(q[1]),0.]),Rotation.identity()
    def ik(self,p,r,seed):return np.array([p[0],p[1]*(1 if seed[1]>=0 else -1)]),0.,0.


class Array:
    def __init__(self,x):self.x=np.asarray(x,float)
    def clone(self):return Array(self.x.copy())
    def new_tensor(self,x):return np.asarray(x)
    def __getitem__(self,i):return Array(self.x[i])
    def __setitem__(self,i,x):self.x[i]=x
    def cpu(self):return self
    def numpy(self):return self.x


def environment():return SimpleNamespace(robot=SimpleNamespace(data=SimpleNamespace(joint_pos=Array([[0.,-.5]]))),step_dt=1/60,lift_id=0)


def candidate():
    def row(phase,x,y):return dict(phase=phase,ring_position_xyz=[x,y,0.],orientation_xyzw=[0,0,0,1],minimum_seconds=0.)
    return dict(trajectory_mode='diagnostic_pose_waypoints_v1',diagnostic_only=True,training_eligible=False,
                pose_waypoints=[row('preapproach',-.8,.5),row('approach',.8,.5),row('insert',1.,.5),row('seat',1.2,.5)])


def test_edge_checks_middle_and_both_endpoints():
    visited=[]
    assert not edge_valid([0.],[1.],lambda q:(visited.append(float(q[0])) or abs(q[0]-.5)>.04),[.01])
    assert any(.4 < x < .6 for x in visited)
    assert not edge_valid([0.],[1.],lambda q:q[0]>0,[.1])
    with pytest.raises(ValueError):edge_valid([0.],[1.],lambda q:None,[.1])


def test_independent_fk_and_collision_reject_one_ik_branch_not_all():
    v=Oracle(lambda q:bool(q[1]<0));cfg=AccessConfig(ik_seeds=8)
    goals,rows=ik_goals(Kin(),np.array([.8,.5,0.]),Rotation.identity(),np.array([0.,.5]),v,[.01,.01],cfg,v.budget)
    assert len(goals)==1 and goals[0][1]<0
    assert any(x['status']=='endpoint_blocked' for x in rows)
    class Liar(Kin):
        def ik(self,*args):return np.array([0.,0.]),0.,0.
    goals,rows=ik_goals(Liar(),np.array([.8,.5,0.]),Rotation.identity(),np.array([0.,.5]),v,[.01,.01],cfg,v.budget)
    assert not goals and all(x['status']=='ik_tolerance' for x in rows)


def test_direct_uses_no_ompl_and_retiming_preserves_route():
    v=Oracle();route,kind=connect([0.,0.],[1.,.4],v,Kin.bounds,[.01,.01],v.budget,'direct')
    assert kind=='direct'
    route=np.array([[0.,0.],[.2,.4],[1.,-.4]])
    original=route.copy();cmd=retime_route(route,1/60,0)
    np.testing.assert_array_equal(route,original);np.testing.assert_array_equal(cmd[-1],route[-1])
    velocities=abs(np.diff(np.vstack([route[0],cmd]),axis=0))*60
    assert np.max(velocities[:,0])<=.08+1e-10 and np.max(velocities[:,1])<=.35+1e-10
    assert np.min(np.linalg.norm(cmd-route[1],axis=1))<1e-12


def test_budget_is_not_collision_or_impossibility():
    v=Oracle(config=AccessConfig(max_state_checks=2))
    with pytest.raises(AccessBudget,match='state_budget'):edge_valid([0.],[1.],v,[.1])
    clock=[0.];b=Budget(AccessConfig(seconds=1.),clock=lambda:clock[0]);clock[0]=2.
    with pytest.raises(AccessBudget,match='wall_budget'):b.check()


def test_staging_stops_before_local_and_pinned_ik_does_not_switch_branch():
    c=candidate();assert staging_index(c['pose_waypoints'])==1
    k=PinnedFirstIK(Kin(),[.8,-.5]);q,pe,re=k.ik([.8,.5,0.],Rotation.identity(),[0.,1.])
    np.testing.assert_array_equal(q,[.8,-.5]);assert pe==0
    assert k.ik([.8,.5,0.],Rotation.identity(),[0.,1.])[0][1]>0


def test_access_rebuilds_suffix_from_selected_branch_and_preserves_inputs():
    env,c=environment(),candidate();old=copy.deepcopy(c);called=[];kin=Kin();v=Oracle()
    def local(e,k,checker,p):
        q=e.robot.data.joint_pos[0].numpy();called.append(q.copy())
        sol,pe,re=k.ik(np.array([.8,.5,0.]),Rotation.identity(),[0.,2.])
        np.testing.assert_array_equal(sol,q)
        return dict(commands=np.array([q,[1.,q[1]]]),phases=['preapproach','insert'],waypoints=p['pose_waypoints']),{'passed':True}
    audited=[]
    def audit(cmd,ph):audited.append((cmd.copy(),ph));return dict(passed=True,complete=True)
    result,report=access_then_local(env,kin,None,c,v,[.05,.05],v.budget.config,local,audit,method='direct')
    assert result is not None and report['full_motion_found']
    np.testing.assert_array_equal(env.robot.data.joint_pos.numpy(),[[0.,-.5]])
    assert c==old and called[0][1]<0 and audited[0][1][0]=='ready'
    np.testing.assert_array_equal(audited[0][0][0],[0.,-.5])


def test_access_only_reaches_without_claiming_hook_or_calling_suffix():
    v=Oracle()
    def forbidden(*args):raise AssertionError('local called for access-only')
    result,report=access_then_local(environment(),Kin(),None,candidate(),v,[.05,.05],v.budget.config,
        forbidden,lambda *a:dict(passed=True,complete=True),method='direct',access_only=True)
    assert result is not None and report['access_only_checked'] and not report['full_motion_found']
    assert report['physics_executed'] is False and report['impossible'] is None


def test_failed_local_or_unknown_audit_does_not_publish_complete_motion():
    for fail in ('local','audit'):
        v=Oracle()
        def local(e,k,ch,p):
            if fail=='local':return None,dict(reason='path_ik')
            return dict(commands=np.array([[.8,-.5],[1.,-.5]]),phases=['preapproach','insert'],waypoints=[]),dict(passed=True)
        r,report=access_then_local(environment(),Kin(),None,candidate(),v,[.05,.05],v.budget.config,
                                  local,lambda *a:dict(passed=False,complete=False),method='direct')
        assert r is None and report['access_found'] and not report['full_motion_found'] and report['impossible'] is None


def test_initial_collision_never_teleports_to_staging():
    v=Oracle(lambda q:False)
    r,report=access_then_local(environment(),Kin(),None,candidate(),v,[.05,.05],v.budget.config,None,None,'direct')
    assert r is None and report['reason']=='initial_state_blocked'


def test_config_validation():
    for kw in ({'ik_seeds':0},{'ik_seeds':True},{'seconds':float('nan')},{'seed':0},{'max_branches':33}):
        with pytest.raises(ValueError):AccessConfig(**kw)


def test_sampling_is_one_factor_not_four_random_full_pose_vectors():
    pytest.importorskip("motion_family_search",reason="full baseline dependency only in repository checkout")
    from access_sampling import low_tilt_schedule,one_factor_tilts
    base=low_tilt_schedule();assert len(base)==3
    assert all(r['parameters']['roll_deg']==r['parameters']['pitch_deg']==0 for r in base)
    parent=base[1]['parameters'];old=copy.deepcopy(parent)
    variants=one_factor_tilts(parent)
    assert len(variants)==8 and parent==old
    for r in variants:
        assert [k for k in parent if parent[k]!=r['parameters'][k]]==[r['changed_factor']]


def test_real_legacy_planner_receives_reached_start_not_old_start():
    torch=pytest.importorskip('torch')
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nvidia-sim/rl'))
    motion=pytest.importorskip('dataset_motion')
    class K7:
        bounds=np.array([[-2.]*7,[2.]*7])
        def fk(self,q):return q[:3].copy(),Rotation.from_rotvec(q[3:6])
        def ik(self,p,r,q):return np.r_[p,r.as_rotvec(),0.],0.,0.
    tensor=torch.zeros((1,7),dtype=torch.float64)
    env=SimpleNamespace(robot=SimpleNamespace(data=SimpleNamespace(joint_pos=tensor)),
        _target_geometry=lambda:[torch.zeros((1,3),dtype=torch.float64)]*3,
        target_spec=dict(radius=.01),step_dt=1/60,lift_id=0,cfg=SimpleNamespace(dataset_rise_speed=.002))
    c=candidate()
    c['pose_waypoints'][0]['ring_position_xyz']=[-.03,0.,0.]
    c['pose_waypoints'][1]['ring_position_xyz']=[.03,0.,0.]
    c['pose_waypoints'][2]['ring_position_xyz']=[.04,.005,0.]
    c['pose_waypoints'][3]['ring_position_xyz']=[.04,.005,.005]
    v=Oracle();checks=[]
    def audit(q,ph):checks.append(q);return dict(passed=True,complete=True)
    planned,report=access_then_local(env,K7(),SimpleNamespace(check=lambda q:None),c,v,[.01]*7,
        v.budget.config,motion.plan,audit,method='direct')
    assert planned is not None and report['full_motion_found']
    np.testing.assert_allclose(planned['commands'][-1][:3],[.04,.005,.005],atol=1e-12)
    np.testing.assert_array_equal(tensor.numpy(),np.zeros((1,7)))
    assert checks and checks[0].shape[1]==7
