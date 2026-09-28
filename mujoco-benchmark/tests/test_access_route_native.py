"""Real OMPL + MuJoCo checks on a small 2-axis robot; NOT the user's greenhouse."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
mj=pytest.importorskip('mujoco')
pytest.importorskip('ompl')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from access_route import AccessConfig,Budget,connect,edge_valid,AccessBudget
from plan_access_candidate import FrozenValidity
from environment_preflight import Policy,MuJoCoScene


def scene(full_wall=False):
    height=2. if full_wall else .32
    m=mj.MjModel.from_xml_string(f'''<mujoco><option gravity="0 0 0"/><worldbody>
    <geom name="obstacle" type="box" pos="0 0 0" size=".15 {height} .1"/>
    <body name="Robot_tool"><joint name="x" type="slide" axis="1 0 0" range="-1 1"/>
    <joint name="y" type="slide" axis="0 1 0" range="-1 1"/>
    <geom name="wire" type="sphere" size=".04" mass="1"/></body></worldbody></mujoco>''')
    e=SimpleNamespace(model=m,qids=np.array([0,1]),initial=np.array([-.8,0.]))
    policy=Policy(clearance_m=0.,max_prismatic_step_m=.02)
    s=MuJoCoScene(e,policy)
    checker=SimpleNamespace(check=lambda q:None)
    cfg=AccessConfig(seconds=5.,solve_seconds=.25,max_state_checks=30000,max_queries=50000)
    v=FrozenValidity(s,checker,policy,[[-1,-1],[1,1]],Budget(cfg))
    return m,s,v


def test_real_rrt_detours_around_native_obstacle_where_line_is_blocked():
    m,s,v=scene();masks=m.geom_contype.copy();initial=s.data.qpos.copy()
    route,kind=connect([-.8,0.],[.8,0.],v,[[-1,-1],[1,1]],[.02,.02],v.budget,'ompl')
    assert route is not None and kind=='ompl_rrtconnect'
    assert np.max(abs(route[:,1]))>.35
    assert all(edge_valid(a,b,v,[.01,.01]) for a,b in zip(route[:-1],route[1:]))
    np.testing.assert_array_equal(m.geom_contype,masks)
    assert np.all(s.model.qpos0==0.) # data is private; model/reset unchanged


def test_real_ompl_does_not_accept_approximate_solution_or_prove_impossible():
    m,s,v=scene(True)
    route,reason=connect([-.8,0.],[.8,0.],v,[[-1,-1],[1,1]],[.02,.02],v.budget,'ompl')
    assert route is None and reason=='search_budget_no_route'


def test_full_robot_checker_is_not_optional_and_errors_are_not_clear():
    m,s,v=scene();v.checker=SimpleNamespace(check=lambda q:('elbow','link3'))
    assert not v(np.array([-.8,0.])) and v.first_failure['kind']=='self_collision'
    v.checker=SimpleNamespace(check=lambda q:None)
    s.distance=lambda *args:(float('nan'),np.zeros(6))
    with pytest.raises(ValueError,match='Nonfinite'):v(np.array([-.2,0.]))
