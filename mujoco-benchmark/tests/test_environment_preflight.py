"""Pure tests run without MuJoCo; native adapter cases skip explicitly if absent."""
import ast
import hashlib
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import environment_preflight as ep


class AnalyticScene:
    """Test double with exact sphere distance; not a greenhouse physics test."""
    version = "analytic_test_double"
    robot_names = ["arm"]
    environment_names = ["obstacle"]
    robot_radii = np.array([0.05])
    environment_radii = np.array([0.05])
    environment_positions = np.array([[0.5, 0.0, 0.0]])
    joint_steps = np.array([0.025])
    inventory = {"kind": "analytic_test"}

    def set_robot(self, q):
        self.robot_positions = np.array([[q[0], 0.0, 0.0]])

    def distance(self, a, b, limit):
        delta = self.environment_positions[b] - self.robot_positions[a]
        d = float(np.linalg.norm(delta)) - self.robot_radii[a] - self.environment_radii[b]
        return min(d, limit), np.r_[self.robot_positions[a], self.environment_positions[b]]


def policy(**kwargs):
    return ep.Policy(clearance_m=0, **kwargs)


def audit(scene=None, commands=None, phases=None, **kwargs):
    return ep.screen(scene or AnalyticScene(), commands or [[0.0], [1.0]], phases or ["insert", "insert"], 1 / 60, policy(**kwargs))


@pytest.mark.parametrize("bad", [-0.01, np.nan, np.inf])
def test_bad_clearance(bad):
    with pytest.raises(ValueError):
        ep.Policy(clearance_m=bad)


def test_clearance_must_be_explicit():
    with pytest.raises(ValueError):
        ep.Policy.from_dict({})


@pytest.mark.parametrize("setting,value", [("max_samples", 0), ("max_samples", True), ("max_distance_queries", 1.5), ("timeout_s", 0), ("max_time_step_s", np.inf)])
def test_invalid_sampling_settings(setting, value):
    with pytest.raises(ValueError):
        policy(**{setting: value})


def test_policy_round_trip_and_resume_conflict(monkeypatch, tmp_path):
    monkeypatch.delenv(ep.ENV_POLICY, raising=False)
    p = policy()
    assert ep.load_policy() is None
    manifest = {"environment_preflight_policy": p.snapshot()}
    assert ep.load_policy(manifest).snapshot() == p.snapshot()
    path = tmp_path / "policy.json"
    path.write_text(json.dumps({"clearance_m": .002}))
    monkeypatch.setenv(ep.ENV_POLICY, str(path))
    with pytest.raises(ValueError, match="differs"):
        ep.load_policy(manifest)
    assert ep.load_policy().clearance_m == .002


def test_policy_hash_tampering(monkeypatch):
    monkeypatch.delenv(ep.ENV_POLICY, raising=False)
    saved = policy().snapshot()
    saved["settings"]["clearance_m"] = .001
    with pytest.raises(ValueError, match="hash"):
        ep.load_policy({"environment_preflight_policy": saved})


def test_sampling_hits_interior_and_final_and_matches_interpolation():
    commands = np.array([[0., 0.], [1., .5], [.3, 1.]])
    samples = list(ep.plan_samples(commands, ["ready", "insert", "rise"], 2 / 60, [.1, .05], policy()))
    times = [x[0] for x in samples]
    assert times[0] == 0
    assert times[-1] == pytest.approx(2 / 60)
    assert np.all(np.diff(times) > 0)
    for t, q, phases in samples:
        expected = [np.interp(t, np.arange(3) / 60, commands[:, i]) for i in range(2)]
        np.testing.assert_allclose(q, expected, atol=1e-14)
    assert any(len(x[2]) == 2 for x in samples)


def test_interior_collision_not_only_waypoints():
    result = audit()
    assert result["status"] == "blocked"
    assert result["first_violation"]["time_s"] > 0
    assert result["first_violation"]["time_s"] < 1 / 60
    assert result["first_violation"]["robot_geom"] == "arm"
    assert not result["passed"]


def test_initial_collision_is_not_ignored():
    assert audit(commands=[[.5], [.6]])["first_violation"]["time_s"] == 0


def test_clear_trajectory():
    result = audit(commands=[[0.], [.1]])
    assert result["passed"] and result["complete"]
    assert result["status"] == "sampled_clear"
    assert result["minimum_queried_distance_m"] is None


def test_positive_margin_rejects_near_but_separate():
    result = ep.screen(AnalyticScene(), [[.38], [.39]], ["insert", "insert"], 1 / 60, ep.Policy(clearance_m=.025))
    assert result["status"] == "blocked"
    assert result["first_violation"]["distance_m"] > 0


def test_allowlist_exact_pair_and_phase():
    allowance = {"robot_geom": "arm", "environment_geom": "obstacle", "phases": ["insert"]}
    result = audit(allowed_contacts=(allowance,))
    assert result["passed"]
    mixed = audit(phases=["ready", "insert"], allowed_contacts=(allowance,))
    assert mixed["status"] == "blocked"
    assert not ep.pair_allowed(policy(allowed_contacts=(allowance,)), "other_arm", "obstacle", ("insert",))


def test_bad_allowlist_does_not_silently_match_nothing():
    result = audit(allowed_contacts=({"robot_geom": "typo", "environment_geom": "obstacle", "phases": ["insert"]},))
    assert result["status"] == "inconclusive" and not result["passed"]
    with pytest.raises(ValueError):
        policy(allowed_contacts=({"robot_geom": "*", "environment_geom": "obstacle", "phases": ["insert"]},))


def test_missing_phase_rejected():
    result = audit(allowed_contacts=({"robot_geom": "arm", "environment_geom": "obstacle", "phases": ["wrong"]},))
    assert not result["passed"] and result["reason"] == "check_error"


def test_budgets_fail_closed():
    result = audit(commands=[[0.], [.1]], max_samples=1)
    assert not result["passed"] and result["reason"] == "sample_budget"
    assert audit(timeout_s=1e-12)["reason"] == "time_budget"


def test_empty_environment_is_inconclusive():
    scene = AnalyticScene()
    scene.environment_names = []
    assert not audit(scene)["passed"]


def test_nan_input_is_not_a_clear_plan():
    assert not audit(commands=[[0.], [np.nan]])["passed"]


def test_native_distance_error_is_not_a_clear_plan():
    scene = AnalyticScene()
    scene.distance = lambda *args: (float("nan"), np.zeros(6))
    result = audit(scene)
    assert result["reason"] == "check_error" and not result["passed"]


def test_planes_not_culled():
    indices = ep.nearby_indices(np.zeros(3), .1, np.array([[10000., 0, 0], [10000., 0, 0]]), np.array([np.inf, .1]), 0)
    assert indices.tolist() == [0]


def test_report_explains_scope_and_denominator(tmp_path):
    result = audit()
    ep.save_report(tmp_path, result)
    assert json.loads((tmp_path / "environment_preflight.json").read_text())["passed"] is False
    assert "NOT a physics replay" in (tmp_path / "environment_preflight.html").read_text()
    (tmp_path / "index.html").write_text("<body></body>")
    ep.save_run_summary(tmp_path, [{"candidate_id": "test_1", "environment_preflight": result, "physics_executed": False}])
    assert "Physics executed: 0" in (tmp_path / "environment_preflight_index.html").read_text()
    assert "environment_preflight_index.html" in (tmp_path / "index.html").read_text()


def execute_function(tmp_path, monkeypatch, preflight_policy):
    # Compile the ACTUAL execute function without importing user's unavailable USD assets.
    tree = ast.parse((SCRIPTS / "candidate_experiment.py").read_text())
    function = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == "execute")
    folder = tmp_path / "candidates" / "demo"
    folder.mkdir(parents=True)
    (folder / "plan.json").write_text(json.dumps({"candidate_id": "demo", "preflight": {"passed": True}, "seconds": 1 / 60}))
    (folder / "trace.json").write_text(json.dumps([{"command": [0.], "phase": "insert"}, {"command": [.1], "phase": "insert"}]))
    (tmp_path / "manifest.json").write_text(json.dumps({"model_sha256": "test_only"}))
    calls = []
    def rollout(**kwargs):
        calls.append(kwargs)
        pose = np.tile([0., 0., 0., 0., 0., 0., 1.], (2, 2, 1))
        return {"unstable": False, "_arrays": {"poses": pose, "qpos": np.zeros((2, 1)), "times_s": np.array([0., 1 / 60])}, "_contacts": [[], []]}
    engine = SimpleNamespace(rollout=rollout, target="Tomato_05", target_spec={"center": [0, 0, 0], "radius": .01},
        ref={"bodies": [{"name": "Tomato_05"}], "shapes": []}, model=SimpleNamespace(opt=SimpleNamespace(timestep=1 / 120)))
    monkeypatch.setitem(sys.modules, "replay_storage", SimpleNamespace(compact_states=lambda arrays, rate: arrays))
    namespace = dict(RUN=tmp_path, ENGINE=engine, PREFLIGHT_POLICY=preflight_policy, RULE="center_entry_only_v2",
                     json=json, np=np, time=time, hashlib=hashlib, RING=np.zeros(3), center_region=lambda *x: False,
                     classify_result=lambda *x: "miss")
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SCRIPTS / "candidate_experiment.py"), "exec"), namespace)
    return namespace["execute"], calls, folder


@pytest.mark.parametrize("status", ["blocked", "inconclusive"])
def test_real_execute_gate_does_not_call_physics(tmp_path, monkeypatch, status):
    execute, calls, folder = execute_function(tmp_path, monkeypatch, policy())
    monkeypatch.setattr(ep, "check_engine", lambda *args: {"passed": False, "status": status})
    result = execute({"candidate_id": "demo"})
    assert calls == []
    assert result["physics_executed"] is False
    assert result["training_eligible"] is False
    assert result["center_entered"] is None
    assert result["result"] == "environment_preflight_" + status
    assert not (folder / "states.npz").exists()


@pytest.mark.parametrize("enabled", [False, True])
def test_real_execute_legacy_path_and_passed_path(tmp_path, monkeypatch, enabled):
    execute, calls, folder = execute_function(tmp_path, monkeypatch, policy() if enabled else None)
    def passed(*args):
        assert enabled
        return {"passed": True, "status": "sampled_clear"}
    monkeypatch.setattr(ep, "check_engine", passed)
    result = execute({"candidate_id": "demo"})
    assert len(calls) == 1
    assert result["result"] == "miss"
    assert result["hook_success"] is None
    assert (folder / "states.npz").exists()
    assert ("environment_preflight" in result) == enabled


def test_guard_does_not_overwrite_old_run(tmp_path, monkeypatch):
    execute, calls, folder = execute_function(tmp_path, monkeypatch, policy())
    old = '{"result":"miss"}'
    (folder / "result.json").write_text(old)
    with pytest.raises(ValueError, match="new candidate"):
        execute({"candidate_id": "demo"})
    assert not calls
    assert (folder / "result.json").read_text() == old


@pytest.mark.parametrize("obstacle", ['type="sphere" size=".1" pos=".5 0 0"', 'type="box" size=".1 .1 .1" pos=".5 0 0"'])
def test_native_adapter_when_mujoco_available(obstacle):
    mj = pytest.importorskip("mujoco", reason="Native MuJoCo dependency not installed; NOT physics-validated here")
    xml = f'''<mujoco><worldbody><geom name="obstacle" {obstacle}/>
    <body name="Robot_link"><joint name="slide" type="slide" axis="1 0 0"/>
    <geom name="arm" type="sphere" size=".05" contype="8" conaffinity="3"/>
    <geom name="visual" type="sphere" size="2" contype="0" conaffinity="0" mass="0"/>
    </body></worldbody></mujoco>'''
    model = mj.MjModel.from_xml_string(xml)
    engine = SimpleNamespace(model=model, qids=np.array([0]), initial=np.array([0.]))
    before = model.qpos0.copy()
    result = ep.check_engine(engine, [{"command": [0.], "phase": "insert"}, {"command": [1.], "phase": "insert"}], 1 / 60, policy())
    assert result["status"] == "blocked"
    assert "visual" not in result["inventory"]["robot_collision_geoms"]
    np.testing.assert_array_equal(model.qpos0, before)
