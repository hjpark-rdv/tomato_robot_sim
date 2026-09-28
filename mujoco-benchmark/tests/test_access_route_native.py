"""Real OMPL and native geometry, small synthetic model, NOT the greenhouse."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
mj=pytest.importorskip('mujoco')
pytest.importorskip('ompl')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from access_route import Budget,SceneValidity,checked_path,ompl_connect,timed_polyline
from environment_preflight import Policy,MuJoCoScene


def native_checker():
    xml='''<mujoco><option gravity="0 0 0"/>
    <worldbody><geom name="wall" type="box" pos=".5 .25 0" size=".05 .35 .2"/>
    <body name="Robot_x"><inertial pos="0 0 0" mass=".1" diaginertia=".001 .001 .001"/><joint name="x" type="slide" axis="1 0 0" range="0 1"/>
    <body name="Hook"><joint name="y" type="slide" axis="0 1 0" range="0 1"/>
    <geom name="wire" type="sphere" size=".02" mass="1"/></body></body></worldbody></mujoco>'''
    m=mj.MjModel.from_xml_string(xml);e=SimpleNamespace(model=m,qids=np.array([0,1]),initial=np.array([.1,.3]))
    policy=Policy(clearance_m=0.,max_prismatic_step_m=.01)
    scene=MuJoCoScene(e,policy)
    checker=SimpleNamespace(check=lambda q:None)
    valid=SceneValidity(scene,checker,policy,[[0,0],[1,1]],Budget(10,50000))
    return e,scene,valid


def test_real_ompl_routes_around_native_obstacle():
    e,scene,valid=native_checker()
    before=e.model.geom_pos.copy()
    path,meta=ompl_connect([.1,.3],[.9,.3],[[0,0],[1,1]],[.01,.01],valid,seconds=1.,seed=1)
    assert path is not None and meta['status']=='exact'
    assert path[:,1].max()>.62
    assert checked_path(path,[.005,.005],valid)
    np.testing.assert_array_equal(before,e.model.geom_pos)
    q=timed_polyline(path,[.1,.1]);assert checked_path(q,[.005,.005],valid)


def test_no_solution_is_not_an_approximate_success():
    path,meta=ompl_connect([.1,.3],[.9,.3],[[0,0],[1,1]],[.01,.01],
        lambda q:not(.4<=q[0]<=.6),seconds=.15,seed=8)
    assert path is None and meta['status']=='not_found_within_budget'


def test_collision_validity_keeps_self_checker_in_loop():
    e,scene,valid=native_checker()
    valid.self_checker=SimpleNamespace(check=lambda q:('wire','link3') if q[1]>.8 else None)
    assert not valid([.1,.9]) and valid.last_blocker['kind']=='self_collision'


def test_start_in_collision_is_not_routed_out_by_ignoring_it():
    e,scene,valid=native_checker()
    path,meta=ompl_connect([.5,.3],[.9,.3],[[0,0],[1,1]],[.01,.01],valid,seconds=.2)
    assert path is None and meta['status']=='invalid_start_or_goal'
