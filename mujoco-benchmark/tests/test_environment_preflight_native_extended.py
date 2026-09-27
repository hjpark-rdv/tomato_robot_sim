"""Native geometric fixtures for the existing preflight implementation.

These are small generated scenes, not the farm's plant/robot assets. They only
exercise frozen-scene kinematic screening; none claim successful physical hooks.
"""
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

mj = pytest.importorskip("mujoco")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from environment_preflight import Policy, MuJoCoScene, check_engine


def from_xml(xml):
    model = mj.MjModel.from_xml_string(xml)
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    joint = model.joint("j").id
    return SimpleNamespace(model=model, data=data,
                           qids=np.array([model.jnt_qposadr[joint]]),
                           initial=np.zeros(1))


def slider(obstacle_x=.5, obstacle_y=0., explicit=False, visual_only=False):
    masks = 'contype="0" conaffinity="0"' if explicit or visual_only else 'contype="16" conaffinity="16"'
    pairs = '<contact><pair geom1="elbow" geom2="obstacle"/></contact>' if explicit else ''
    return from_xml(f'''<mujoco><compiler angle="radian" fusestatic="false"/>
    <worldbody>
      <geom name="obstacle" type="sphere" pos="{obstacle_x} {obstacle_y} 0" size=".05" {masks}/>
      <body name="Robot_base">
        <geom name="base" type="sphere" pos="0 0 1" size=".01"/>
        <body name="Robot_slide">
          <joint name="j" type="slide" axis="1 0 0" range="0 1"/>
          <geom name="elbow" type="sphere" size=".02" contype="8" conaffinity="8"/>
          <body name="Hook" pos="0 .5 0">
            <geom name="wire" type="sphere" size=".01" contype="8" conaffinity="8"/>
          </body>
        </body>
      </body>
    </worldbody>{pairs}</mujoco>''')


def run_check(engine, policy=None, phases=("ready", "insert"), end=1.):
    policy = policy or Policy(clearance_m=0., max_prismatic_step_m=.01)
    rows = [dict(command=[0.], phase=phases[0]), dict(command=[end], phase=phases[1])]
    return check_engine(engine, rows, 1 / 60, policy)


def test_native_elbow_intersection_despite_source_contact_masks():
    engine = slider()
    assert engine.data.ncon == 0
    result = run_check(engine)
    assert result["status"] == "blocked", result
    hit = result["first_violation"]
    assert hit["robot_geom"] == "elbow"
    assert hit["environment_geom"] == "obstacle"
    assert 0 < hit["time_s"] < 1 / 60


def test_native_explicit_contact_pair_includes_zero_mask_geom():
    result = run_check(slider(explicit=True))
    assert result["status"] == "blocked", result
    assert "obstacle" in result["inventory"]["environment_collision_geoms"]


def test_native_initial_overlap_is_not_ignored():
    result = run_check(slider(obstacle_x=0.))
    assert result["status"] == "blocked", result
    assert result["first_violation"]["time_s"] == 0.


def test_native_hook_permission_requires_exact_phase():
    engine = slider(obstacle_y=.5)
    policy = Policy(clearance_m=0., allowed_contacts=(dict(
        robot_geom="wire", environment_geom="obstacle", phases=["rise"]),))
    assert run_check(engine, policy, ("rise", "rise"))["passed"]
    result = run_check(engine, policy, ("insert", "rise"))
    assert result["status"] == "blocked", result


def test_native_visual_only_environment_is_inconclusive_not_clear():
    result = run_check(slider(visual_only=True))
    assert not result["passed"] and result["status"] == "inconclusive", result


def test_native_distant_obstacle_passes_without_changing_live_state():
    engine = slider(obstacle_y=2.)
    engine.data.time = 12.5
    engine.data.qpos[:] = .123
    engine.data.qvel[:] = .7
    before = [v.copy() for v in (engine.data.qpos, engine.data.qvel, engine.data.ctrl,
                                 engine.model.geom_rgba)]
    flags = (engine.model.opt.enableflags, engine.model.opt.disableflags)
    result = run_check(engine)
    assert result["passed"], result
    for old, new in zip(before, (engine.data.qpos, engine.data.qvel, engine.data.ctrl,
                                 engine.model.geom_rgba)):
        np.testing.assert_array_equal(old, new)
    assert engine.data.time == 12.5
    assert flags == (engine.model.opt.enableflags, engine.model.opt.disableflags)


def test_native_open_capsule_ring_keeps_its_aperture():
    engine = from_xml('''<mujoco><worldbody>
      <geom name="target" type="sphere" size=".03"/>
      <body name="Robot_slide"><joint name="j" type="slide" axis="1 0 0" range="0 1"/>
        <body name="Hook">
          <geom name="w1" type="capsule" fromto="0 -.1 -.1 0 .1 -.1" size=".005"/>
          <geom name="w2" type="capsule" fromto="0 -.1 .1 0 .1 .1" size=".005"/>
          <geom name="w3" type="capsule" fromto="0 -.1 -.1 0 -.1 .1" size=".005"/>
          <geom name="w4" type="capsule" fromto="0 .1 -.1 0 .1 .1" size=".005"/>
        </body>
      </body>
    </worldbody></mujoco>''')
    result = run_check(engine)
    assert result["passed"], result
    assert len(result["inventory"]["robot_collision_geoms"]) == 4


def test_native_plane_clearance_margin():
    engine = from_xml('''<mujoco><worldbody>
      <geom name="ground" type="plane" pos="0 0 -.025" size="1 1 .01"/>
      <body name="Robot_slide"><joint name="j" type="slide" axis="1 0 0" range="0 1"/>
        <body name="Hook"><geom name="wire" type="sphere" size=".02"/></body>
      </body>
    </worldbody></mujoco>''')
    result = run_check(engine, Policy(clearance_m=.004, max_prismatic_step_m=.1))
    assert result["passed"], result
    result = run_check(engine, Policy(clearance_m=.006, max_prismatic_step_m=.1))
    assert result["status"] == "blocked", result
    assert result["first_violation"]["distance_m"] == pytest.approx(.005, abs=1e-6)


def test_native_rotation_sweep_checks_interior_not_only_endpoints():
    engine = from_xml('''<mujoco><compiler angle="radian"/>
    <worldbody><geom name="obstacle" type="sphere" pos="0 .4 0" size=".03"/>
      <body name="Robot_wrist"><joint name="j" type="hinge" axis="0 0 1" range="0 3.141592653589793"/>
        <body name="Hook"><geom name="wire" type="capsule" fromto=".2 0 0 .5 0 0" size=".01"/></body>
      </body>
    </worldbody></mujoco>''')
    policy = Policy(clearance_m=0., max_revolute_step_rad=.05)
    scene = MuJoCoScene(engine, policy)
    for q in (0., np.pi):
        scene.set_robot(np.array([q]))
        distance, _ = scene.distance(0, 0, 1.)
        assert distance > .1
    result = run_check(engine, policy, end=np.pi)
    assert result["status"] == "blocked", result
    assert 0 < result["first_violation"]["time_s"] < 1 / 60


def test_native_convex_mesh_box_query():
    engine = from_xml('''<mujoco><asset>
      <mesh name="cube" vertex="-.03 -.03 -.03 .03 -.03 -.03 -.03 .03 -.03 .03 .03 -.03 -.03 -.03 .03 .03 -.03 .03 -.03 .03 .03 .03 .03 .03"/>
    </asset><worldbody>
      <geom name="gutter" type="box" pos=".5 0 0" size=".04 .1 .1"/>
      <body name="Robot_slide"><joint name="j" type="slide" axis="1 0 0" range="0 1"/>
        <body name="Hook"><geom name="wire" type="mesh" mesh="cube"/></body>
      </body>
    </worldbody></mujoco>''')
    result = run_check(engine)
    assert result["status"] == "blocked", result
    assert result["first_violation"]["environment_geom"] == "gutter"


def test_native_sample_budget_never_becomes_clear():
    result = run_check(slider(obstacle_y=2.), Policy(clearance_m=0., max_samples=2))
    assert result["status"] == "inconclusive" and not result["passed"], result
    assert result["reason"] == "sample_budget"
