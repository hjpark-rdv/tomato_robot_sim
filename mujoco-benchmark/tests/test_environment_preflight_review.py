"""Regressions from the feedback ZIP; toy models are not greenhouse validation."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import environment_preflight as ep


class Scene:
    version = 'analytic_review_fixture'
    robot_names = ['arm']
    robot_radii = np.array([.1])
    joint_steps = [.1]
    inventory = {'kind': 'synthetic, not user assets'}

    def __init__(self, count=1, colliding=True):
        self.environment_names = ['glb_col_part_' + str(i) for i in range(count)]
        self.environment_radii = np.full(count, .1)
        self.environment_positions = np.zeros((count, 3))
        self.colliding = colliding
        self.calls = 0

    def set_robot(self, q):
        self.q = float(q[0])
        self.robot_positions = np.zeros((1, 3))

    def distance(self, a, b, limit):
        self.calls += 1
        return (-.01 - .01 * self.q if self.colliding else limit), np.zeros(6)


def scan(scene=None, full=True, **settings):
    return ep.screen(scene or Scene(), [[0.], [.2]], ['insert', 'insert'],
                     1 / 60, ep.Policy(clearance_m=0, **settings),
                     collect_all_violations=full)


def test_known_hit_survives_sample_budget():
    r = scan(max_samples=1)
    assert r['status'] == 'blocked' and not r['passed'] and not r['complete']
    assert r['reason'] == 'clearance_violation'
    assert r['scan_stop_reason'] == 'sample_budget'
    assert r['violation_summary']['full_path_requested']
    assert not r['violation_summary']['full_path_collected']
    assert r['last_completed_sample_time_s'] == 0


def test_no_hit_before_budget_remains_unknown():
    r = scan(Scene(colliding=False), max_samples=1)
    assert r['status'] == 'inconclusive' and not r['passed']
    assert r['scan_stop_reason'] == r['reason'] == 'sample_budget'
    assert not r['violation_summary']['full_path_collected']


def test_query_budget_after_hit_preserves_partial_sample_status():
    r = scan(Scene(count=2), max_distance_queries=1)
    assert r['status'] == 'blocked' and not r['complete']
    assert r['scan_stop_reason'] == 'distance_query_budget'
    assert r['last_sample_time_s'] == 0
    assert r['last_completed_sample_time_s'] is None


def test_later_error_does_not_erase_known_collision():
    s = Scene(count=2)
    orig = s.distance
    def distance(a, b, limit):
        if b == 1:
            raise RuntimeError('intentional test failure')
        return orig(a, b, limit)
    s.distance = distance
    r = scan(s)
    assert r['status'] == 'blocked' and not r['complete']
    assert r['scan_stop_reason'] == 'check_error'
    assert 'intentional test failure' in r['error']
    assert not r['violation_summary']['full_path_collected']


def test_later_timeout_does_not_claim_full_scan(monkeypatch):
    now = [0.]
    monkeypatch.setattr(ep.time, 'perf_counter', lambda: now[0])
    s = Scene(count=2)
    orig = s.distance
    def distance(a, b, limit):
        ret = orig(a, b, limit)
        now[0] = 2.
        return ret
    s.distance = distance
    r = scan(s, timeout_s=1.)
    assert r['status'] == 'blocked' and r['scan_stop_reason'] == 'time_budget'
    assert not r['violation_summary']['full_path_collected']


def test_cap_retains_late_class_evidence_and_true_counts():
    s = Scene(count=66)
    s.environment_names[-1] = 'gutter_collision_late'
    r = scan(s)
    summary = r['violation_summary']
    assert r['complete'] and not r['passed']
    assert len(r['violations']) == 64
    assert summary['observed_unique_pair_phase_count'] == 66
    assert summary['omitted_unique_pair_phase_count'] == 2
    assert summary['details_truncated'] and summary['full_path_collected']
    assert summary['first_by_environment_class']['gutter']['environment_geom'] == 'gutter_collision_late'
    assert summary['worst_by_environment_class']['gutter']['distance_m'] == pytest.approx(-.012)
    # Three sampled poses, not three independent contact experiments.
    assert summary['sample_hits_by_environment_class']['gutter'] == 3


def test_exact_cap_is_not_truncated():
    r = scan(Scene(count=64))
    assert not r['violation_summary']['details_truncated']
    assert r['violation_summary']['omitted_unique_pair_phase_count'] == 0


def test_repeated_pair_has_separate_first_and_worst_distance():
    r = scan()
    v = r['violations'][0]
    assert v['distance_m'] == pytest.approx(-.01)
    assert v['minimum_distance_m'] == pytest.approx(-.012)
    assert v['time_s'] == 0 and v['last_time_s'] == pytest.approx(1 / 60)
    assert v['sample_hit_count'] == 3
    assert r['first_violation']['distance_m'] == pytest.approx(-.01)


def test_clear_full_scan_has_explicit_empty_complete_summary():
    r = scan(Scene(colliding=False))
    assert r['passed'] and r['complete'] and r['scan_stop_reason'] == 'end_of_path'
    assert r['violation_summary']['full_path_collected']
    assert r['violation_summary']['observed_unique_pair_phase_count'] == 0


def test_default_gate_still_stops_at_first_hit():
    s = Scene(count=5)
    r = scan(s, full=False)
    assert r['status'] == 'blocked' and s.calls == 1
    assert not r['complete'] and r['scan_stop_reason'] == 'first_violation'
    assert not r['violation_summary']['full_path_requested']


def test_namespaced_plant_class_does_not_imply_permission():
    assert ep.classify_environment_geom('truss_02__glb_col_TRUSS_Rachis_05') == 'glb_plant'
    assert not ep.pair_allowed(ep.Policy(clearance_m=0), 'arm', 'glb_col_TRUSS_Rachis_05', ['insert'])


@pytest.mark.parametrize('distance,active', [(.00025, True), (.00075, False)])
def test_native_positive_distance_can_be_active_or_inactive(distance, active):
    # MuJoCo 3.13/3.14 semantics: pair margin is the force threshold;
    # margin+gap is the detection threshold. Never infer forces from dist<=0.
    mj = pytest.importorskip('mujoco')
    model = mj.MjModel.from_xml_string(f'''<mujoco>
      <option gravity="0 0 0" integrator="implicitfast"/>
      <worldbody><geom name="floor" type="plane" size="1 1 .01"/>
        <body pos="0 0 {0.02 + distance}"><joint type="slide" axis="0 0 1"/>
          <geom name="ball" type="sphere" size=".02" mass="1"/>
        </body>
      </worldbody>
      <contact><pair geom1="floor" geom2="ball" margin=".0005" gap=".0005"/></contact>
    </mujoco>''')
    d = mj.MjData(model)
    mj.mj_forward(model, d)
    assert d.ncon == 1
    c = d.contact[0]
    assert c.dist == pytest.approx(distance)
    assert (c.efc_address >= 0) == active
    force = np.zeros(6)
    mj.mj_contactForce(model, d, 0, force)
    if active:
        assert force[0] > 0
    else:
        np.testing.assert_array_equal(force, np.zeros(6))


def test_native_poststep_geometry_requires_resynchronized_private_contacts():
    mj = pytest.importorskip('mujoco')
    model = mj.MjModel.from_xml_string('''<mujoco>
      <option gravity="0 0 0" timestep=".001" integrator="implicitfast"/>
      <worldbody><geom name="floor" type="plane" size="1 1 .01"/>
        <body pos="0 0 .03"><joint name="j" type="slide" axis="0 0 1"/>
          <geom name="ball" type="sphere" size=".02" mass="1"/>
        </body>
      </worldbody>
      <contact><pair geom1="floor" geom2="ball" margin="0" gap=".02"/></contact>
    </mujoco>''')
    d = mj.MjData(model)
    d.qvel[0] = .1
    mj.mj_step(model, d)
    mj.mj_kinematics(model, d)  # Same order as the current RobotEngine.
    actual = mj.mj_geomDistance(model, d, model.geom('floor').id, model.geom('ball').id, .1, np.zeros(6))
    assert actual == pytest.approx(.0101)
    assert d.contact[0].dist == pytest.approx(.01)
    names = ('qpos', 'qvel', 'qacc_warmstart', 'ctrl', 'qfrc_applied', 'xfrc_applied', 'efc_force')
    before = {n: getattr(d, n).copy() for n in names}
    old_contact = float(d.contact[0].dist)
    old_time = float(d.time)
    private = mj.MjData(model)
    mj.mj_copyData(private, model, d)
    mj.mj_forward(model, private)  # Recomputed at copied state, NOT old solve force.
    assert private.contact[0].dist == pytest.approx(actual)
    for name in names:
        np.testing.assert_array_equal(getattr(d, name), before[name])
    assert d.time == old_time and d.contact[0].dist == old_contact
    # Diagnostic work must not change the subsequent live trajectory.
    baseline = mj.MjData(model)
    mj.mj_copyData(baseline, model, d)
    mj.mj_step(model, baseline)
    mj.mj_step(model, d)
    np.testing.assert_array_equal(baseline.qpos, d.qpos)
    np.testing.assert_array_equal(baseline.qvel, d.qvel)
