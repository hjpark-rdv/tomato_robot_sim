"""Connector contracts and actual OMPL obstacle tests (not greenhouse success)."""
import copy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from approach_connection import (JointSpace,CheckBudget,LimitReached,check_polyline,ompl_connect,
                                 ik_pool,retime,handoff_index,suffix_parameters,stitch)
from plan_approach_connection import SceneValidity,plan_suffix


def test_edges_check_interior_and_both_endpoints():
    space=JointSpace([[0,0],[1,1]],[.03,.07])
    result=check_polyline([[0,.5],[1,.5]],space,lambda q:not (.4<q[0]<.6))
    assert not result['passed']
    assert not check_polyline([[0,0],[1,1]],space,lambda q:q[0]!=0)['passed']


def test_mixed_metre_radian_joint_resolution():
    space=JointSpace([[0,-3],[.5,3]],[.001,.01])
    samples=np.array(list(space.edge_samples([0,-3],[.5,3])))
    assert np.all(np.abs(np.diff(samples,axis=0))<=space.steps+1e-12)
    np.testing.assert_allclose(space.denormalize(space.normalize([.1,2])),[.1,2])


def test_direct_route_and_invalid_endpoint_do_not_need_ompl():
    space=JointSpace([[0,0],[1,1]],[.05,.05])
    result=ompl_connect([0,0],[1,1],space,lambda q:True)
    assert result['method']=='direct_checked' and result['status']=='exact'
    failed=ompl_connect([0,0],[1,1],space,lambda q:q[0]<.9)
    assert failed['status']=='invalid_goal' and failed['impossible'] is None


def test_budget_is_not_infeasibility():
    budget=CheckBudget(1,10);budget.tick()
    with pytest.raises(LimitReached,match='state_check'):budget.tick()


def test_retiming_preserves_vertices_bounds_velocity_and_input():
    path=np.array([[.1,0],[.2,1],[.1,.5]])
    original=path.copy();v=np.array([.08,.35]);q=retime(path,v)
    np.testing.assert_array_equal(path,original)
    np.testing.assert_array_equal(q[0],path[0]);np.testing.assert_array_equal(q[-1],path[-1])
    assert any(np.array_equal(x,path[1]) for x in q)
    assert np.all(np.abs(np.diff(q,axis=0))*60<=v+1e-8)
    with pytest.raises(LimitReached):retime(path,v,max_seconds=.01)


def candidate():
    return dict(trajectory_mode='diagnostic_pose_waypoints_v1',diagnostic_only=True,training_eligible=False,
        pose_waypoints=[dict(phase=p,ring_position_xyz=[i,0,0],orientation_xyzw=[0,0,0,1],minimum_seconds=1.)
                        for i,p in enumerate(['preapproach','approach','approach','insert','seat','hold','verify'])])


class Kin:
    def fk(self,q):return np.array([q[0],0,0]),Rotation.identity()


def test_replaces_all_common_prefix_not_only_first_segment():
    params=candidate();old=copy.deepcopy(params);kin=Kin()
    assert handoff_index(params)==2
    suffix=suffix_parameters(params,np.array([.6,.1]),kin)
    assert suffix['pose_waypoints'][0]['ring_position_xyz']==[.6,0,0]
    assert suffix['pose_waypoints'][1:]==params['pose_waypoints'][3:]
    assert params==old


def test_suffix_planning_starts_on_reached_branch_and_does_not_mutate_source(monkeypatch):
    import plan_approach_connection as bridge
    torch=SimpleNamespace(as_tensor=lambda x,dtype:np.array(x,dtype=dtype))
    monkeypatch.setitem(sys.modules,'torch',torch)
    env=SimpleNamespace(robot=SimpleNamespace(data=SimpleNamespace(joint_pos=np.array([[0.,0.]]))))
    reached=np.array([.6,.2]);seen={}
    def plan(local,kin,checker,params):
        seen['q']=local.robot.data.joint_pos.copy()
        return dict(commands=np.array([reached,reached]),phases=['preapproach','insert'],waypoints=[]),{'passed':True}
    planned,_=bridge.plan_suffix(env,Kin(),None,candidate(),reached,plan)
    np.testing.assert_array_equal(seen['q'],reached[None])
    np.testing.assert_array_equal(env.robot.data.joint_pos,[[0,0]])
    assert planned['phases']==['approach','insert']


def test_join_cannot_hide_jump_or_wrong_initial_state():
    rows=stitch([[0,0],[.1,.1]],[[.1,.1],[.101,.1]],['approach','insert'],[0,0],limits=[10,10])
    assert [r['phase'] for r in rows]==['ready','preapproach','approach','insert']
    with pytest.raises(ValueError,match='Discontinuous'):
        stitch([[0,0],[.1,.1]],[[1,1]],['insert'],[0,0],limits=[1,1])
    with pytest.raises(ValueError):stitch([[.1,0],[.2,0]],[[.2,0]],['insert'],[0,0])


def test_multistart_rejects_low_reported_error_with_wrong_actual_fk():
    class BadKin(Kin):
        def ik(self,p,r,seed):return np.array([.4,0]),0,0
    goals,records=ik_pool(BadKin(),[0,0,0],Rotation.identity(),[0,0],
                           JointSpace([[-1,-1],[1,1]],[.01,.01]),lambda q:True,count=2)
    assert not goals and all(x['status']=='ik_not_found' for x in records)


def test_multistart_deduplicates_and_filters_colliding_ik_branches():
    class ManyKin(Kin):
        def ik(self,p,r,seed):return np.array([.5,1. if seed[1]>.2 else -1.]),0,0
    goals,records=ik_pool(ManyKin(),[.5,0,0],Rotation.identity(),[0,-1],
                           JointSpace([[-1,-2],[1,2]],[.01,.01]),lambda q:q[1]<0,count=8)
    assert len(goals)==1 and goals[0][1]<0
    assert any(x['status']=='duplicate_branch' for x in records)
    assert any(x['status']=='goal_collision' for x in records)


def test_actual_ompl_routes_around_obstacle():
    pytest.importorskip('ompl')
    space=JointSpace([[0,0],[1,1]],[.01,.01])
    def valid(q):return not (.4<=q[0]<=.6 and q[1]<=.7)
    assert not check_polyline([[.1,.2],[.9,.2]],space,valid)['passed']
    result=ompl_connect([.1,.2],[.9,.2],space,valid,solve_seconds=2.)
    assert result['status']=='exact' and result['method']=='OMPL_RRTConnect'
    assert max(result['path'][:,1])>.7
    assert check_polyline(result['path'],space,valid)['passed']


def test_actual_ompl_does_not_accept_approximate_solution():
    pytest.importorskip('ompl')
    space=JointSpace([[0,0],[1,1]],[.02,.02])
    valid=lambda q:not (.4<=q[0]<=.6)
    result=ompl_connect([.1,.2],[.9,.2],space,valid,solve_seconds=.05)
    assert result['path'] is None and result['impossible'] is None
    assert result['status']=='no_exact_path_within_budget'


def test_native_scene_obstacle_connection_does_not_change_live_physics():
    mj=pytest.importorskip('mujoco');pytest.importorskip('ompl')
    from environment_preflight import MuJoCoScene,Policy
    xml='<mujoco><option gravity="0 0 0"/><worldbody><geom name="wall" type="box" pos=".5 .3 0" size=".06 .3 .2"/><body name="Robot_xy"><joint name="x" type="slide" axis="1 0 0"/><joint name="y" type="slide" axis="0 1 0"/><geom name="robot_tip" size=".02"/></body></worldbody></mujoco>'
    m=mj.MjModel.from_xml_string(xml);d=mj.MjData(m);d.qpos[:]=[.1,.2];mj.mj_forward(m,d)
    e=SimpleNamespace(model=m,data=d,qids=np.array([0,1]),initial=np.array([.1,.2]))
    policy=Policy(clearance_m=0.,max_prismatic_step_m=.02)
    scene=MuJoCoScene(e,policy);space=JointSpace([[0,0],[1,1]],scene.joint_steps)
    original=d.qpos.copy();masks=m.geom_contype.copy()
    valid=SceneValidity(scene,SimpleNamespace(check=lambda q:None),policy,space,CheckBudget(100000,10.))
    result=ompl_connect(e.initial,[.9,.2],space,valid,solve_seconds=2.)
    assert result['status']=='exact'
    np.testing.assert_array_equal(d.qpos,original);np.testing.assert_array_equal(m.geom_contype,masks)
    assert d.time==0 and valid.queries>0
