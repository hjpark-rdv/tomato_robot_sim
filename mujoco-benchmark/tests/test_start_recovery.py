"""Regression tests for canonical start recovery and new-start D1 plumbing."""
from pathlib import Path
import json
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from approach_connection import JointSpace
from initial_configuration_recovery import (
    StartSearchBudget, StartStateEvaluator, choose_canonical,
    deterministic_candidates, hold_rows, search_start)
import materialize_new_start_run as materializer
import run_new_start_d1 as newd1
import run_start_recovery_pipeline as pipeline


class FakeScene:
    """One 1-D robot sphere moving past one environment sphere."""
    def __init__(self):
        self.robot_names = ["robot"]
        self.environment_names = ["fruit"]
        self.robot_radii = np.array([.1])
        self.environment_radii = np.array([.1])
        self.robot_positions = np.array([[0., 0., 0.]])
        self.environment_positions = np.array([[.15, 0., 0.]])
        self.joint_steps = np.array([.01])
        self.inventory = {}

    def set_robot(self, q):
        self.robot_positions[0, 0] = float(q[0])

    def distance(self, i, j, limit):
        delta = self.environment_positions[j] - self.robot_positions[i]
        norm = float(np.linalg.norm(delta))
        d = norm - self.robot_radii[i] - self.environment_radii[j]
        direction = delta / norm if norm > 1e-12 else np.array([1., 0., 0.])
        a = self.robot_positions[i] + direction * self.robot_radii[i]
        b = self.environment_positions[j] - direction * self.environment_radii[j]
        return d, np.r_[a, b]


class NoSelfCollision:
    def check(self, q):
        return None


def test_candidate_stream_is_deterministic_bounded_and_original_first():
    space = JointSpace([[-1., -2.], [1., 2.]], [.01, .01])
    a = list(deterministic_candidates(
        [0., 0.], space, radii=(.05, .1), samples_per_radius=4, seed=7))
    b = list(deterministic_candidates(
        [0., 0.], space, radii=(.05, .1), samples_per_radius=4, seed=7))
    assert a[0]["source"] == "original"
    assert all(np.allclose(x["q"], y["q"]) for x, y in zip(a, b))
    assert len({x["q"].tobytes() for x in a}) == len(a)
    assert all(space.contains(x["q"]) for x in a)


def test_start_evaluator_rejects_overlap_and_search_finds_valid_q():
    scene = FakeScene()
    space = JointSpace([[-.5], [.5]], [.01])
    budget = StartSearchBudget(max_candidates=100, max_distance_queries=1000, seconds=10.)
    evaluator = StartStateEvaluator(scene, NoSelfCollision(), space, budget, probe_horizon_m=.5)
    original = evaluator.evaluate([0.])
    assert not original["valid"]
    assert original["status"] == "environment_collision"
    assert original["minimum_distance_m"] == pytest.approx(-.05)

    # Search around a colliding q.  Axis/Sobol candidates eventually move left.
    budget = StartSearchBudget(max_candidates=100, max_distance_queries=1000, seconds=10.)
    evaluator = StartStateEvaluator(scene, NoSelfCollision(), space, budget, probe_horizon_m=.5)
    result = search_start(
        [0.], space, evaluator, radii=(.1, .2, .4),
        samples_per_radius=4, seed=3, max_valid=3)
    assert result["status"] == "valid_candidates_found"
    assert result["impossible"] is None
    assert all(row["valid"] for row in result["ranked_valid"])


def test_search_exhaustion_is_not_infeasibility():
    scene = FakeScene()
    scene.environment_radii[:] = 10.
    space = JointSpace([[-.5], [.5]], [.01])
    evaluator = StartStateEvaluator(
        scene, NoSelfCollision(), space,
        StartSearchBudget(max_candidates=2, max_distance_queries=1000, seconds=10.),
        probe_horizon_m=.5)
    result = search_start(
        [0.], space, evaluator, radii=(.1,), samples_per_radius=4,
        seed=1, max_valid=1)
    assert result["status"] == "unresolved_within_budget"
    assert result["impossible"] is None
    assert result["stop_reason"] == "start_candidate_budget"


def test_canonical_selection_requires_hold_validity_and_prefers_min_change():
    result = dict(ranked_valid=[
        dict(q=[.1], normalized_change=.1, minimum_distance_m=.002),
        dict(q=[.2], normalized_change=.2, minimum_distance_m=.020),
    ])
    holds = {
        (.1,): dict(validity=dict(passed=False)),
        (.2,): dict(validity=dict(passed=True)),
    }
    chosen = choose_canonical(result, holds)
    assert chosen["q"] == [.2]

    holds[(.1,)] = dict(validity=dict(passed=True))
    assert choose_canonical(result, holds)["q"] == [.1]


def test_hold_rows_define_new_start_without_old_commands():
    rows = hold_rows([1., 2.], seconds=2., command_hz=60)
    assert len(rows) == 121
    assert all(r["joints"] == [1., 2.] and r["command"] == [1., 2.] for r in rows)
    assert {r["phase"] for r in rows} == {"ready"}


def make_source_run(tmp_path):
    source = tmp_path / "source"
    (source / "replay_assets").mkdir(parents=True)
    for name in ("model.mjb", "reference.json", "planning_model.pkl"):
        (source / "replay_assets" / name).write_bytes(b"x")
    (source / "replay_assets" / "initial_trace.json").write_text(
        json.dumps([dict(joints=[0., 0.], command=[0., 0.], phase="ready")]))
    (source / "manifest.json").write_text(json.dumps(dict(target="Tomato_01", hz=240)))
    (source / "planning_inputs.json").write_text(json.dumps(dict(
        inputs=dict(start=[0., 0.]), ring_position=[0., 0., 0.], world=np.eye(4).tolist())))
    (source / "candidates.json").write_text(json.dumps([
        dict(candidate_id="c0", trajectory_mode="diagnostic_pose_waypoints_v1",
             pose_waypoints=[])]))
    return source


def test_materialized_run_changes_start_but_never_copies_recorded_prefix(tmp_path, monkeypatch):
    source = make_source_run(tmp_path)
    start_file = tmp_path / "start.json"
    start_file.write_text(json.dumps(dict(schema="canonical_robot_start_v1", q=[.1, -.2])))

    class Kin:
        bounds = np.array([[-1., -1.], [1., 1.]])
        def fk(self, q):
            from scipy.spatial.transform import Rotation
            return np.array([q[0], q[1], 0.]), Rotation.identity()

    fake = SimpleNamespace(MODEL=None, START=None)
    def initialize(path):
        data = json.loads((Path(path) / "planning_inputs.json").read_text())
        fake.START = np.asarray(data["inputs"]["start"], dtype=float)
        fake.MODEL = (SimpleNamespace(), Kin(), SimpleNamespace())
    fake.initialize = initialize
    monkeypatch.setitem(sys.modules, "plan_candidates", fake)
    initialize(source)

    out = tmp_path / "derived"
    provenance = materializer.materialize(source, start_file, out)
    assert provenance["recorded_prefix_reuse_allowed"] is False
    planning = json.loads((out / "planning_inputs.json").read_text())
    assert planning["inputs"]["start"] == [.1, -.2]
    assert planning["ring_position"] == pytest.approx([.1, -.2, 0.])
    rows = json.loads((out / "replay_assets/initial_trace.json").read_text())
    assert rows[0]["joints"] == [.1, -.2]
    assert not (out / "candidates" / "c0" / "trace.json").exists()
    manifest = json.loads((out / "manifest.json").read_text())
    assert manifest["new_start_prefix_policy"].startswith("replan_from_new_start")


def test_new_start_classification_keeps_budget_failures_unknown():
    assert newd1.classify_connection(dict(status="invalid_initial_configuration")) == "initial_state_invalid"
    assert newd1.classify_connection(dict(status="no_valid_ik_goal_found")) == "ik_not_found_within_budget"
    assert newd1.classify_connection(dict(status="budget_exhausted")) == "unresolved_within_budget"
    assert newd1.classify_connection(dict(status="whole_path_passed")) == "whole_path_audit_passed"
    assert newd1.classify_connection(dict(
        status="prefix_clear_suffix_environment_rejected",
        attempts=[dict(first_violation=dict(phases=["insert"]))])) == "insert_environment_blocked"
    assert newd1.classify_connection(dict(
        status="prefix_clear_suffix_environment_rejected",
        attempts=[dict(first_violation=dict(phases=["seat"]))])) == "seat_environment_blocked"


def test_new_start_prepare_removes_old_recorded_prefix_requirement(tmp_path, monkeypatch):
    campaign = tmp_path / "campaign"
    run = campaign / "cases" / "s0" / "t0" / "run"
    run.mkdir(parents=True)
    (run.parent / "base_policy.json").write_text("{}")
    start = tmp_path / "canonical.json"
    start.write_text(json.dumps(dict(schema="canonical_robot_start_v1", q=[.1, .2])))
    prefix = tmp_path / "prefix.json"; prefix.write_text("{}")

    old_doc = dict(queries=[
        dict(query_id="q00_s0_t0_recorded", cohort="local_blocked",
             scene="s0", target="t0", candidate="c0", family="flank_left",
             design="recorded", command=["python", "old", "--require-recorded-prefix"])
    ])
    monkeypatch.setattr(newd1.historical, "prepare", lambda *a, **k: old_doc)

    def fake_materialize(source, start_file, destination):
        destination.mkdir(parents=True)
        return dict(destination=str(destination))
    monkeypatch.setattr(newd1, "materialize", fake_materialize)

    out = tmp_path / "out"
    doc = newd1.prepare(campaign, prefix, start, out, "/python")
    assert len(doc["queries"]) == 1
    q = doc["queries"][0]
    assert q["prefix_mode"] == "new_start_replanned_prefix"
    assert "--require-recorded-prefix" not in q["command"]
    assert q["historical_cohort"] == "local_blocked"


def test_pipeline_summary_distinguishes_start_approach_local_and_physics():
    no_start = pipeline.summarize(dict(canonical=None))
    assert no_start["case"] == "CASE0_INITIAL_CONFIGURATION"

    start = dict(canonical=dict(q=[0.]))
    approach = pipeline.summarize(start, dict(rows=[
        dict(status="no_valid_ik_goal_found", classification="ik_not_found_within_budget",
             prefix_found=False, whole_path_passed=False)
    ]))
    assert approach["case"] == "CASE1_APPROACH_PLANNING"

    local = pipeline.summarize(start, dict(rows=[
        dict(status="prefix_clear_suffix_environment_rejected",
             classification="insert_environment_blocked",
             prefix_found=True, whole_path_passed=False)
    ]))
    assert local["case"] == "CASE2_LOCAL_MANIPULATION"
    assert "not physical staging success" in local["evidence_level"]

    runtime = pipeline.summarize(start, dict(rows=[
        dict(status="whole_path_passed", classification="whole_path_audit_passed",
             prefix_found=True, whole_path_passed=True)
    ]), dict(trials=[]))
    assert runtime["case"] == "CASE4_RUNTIME_VALIDATION_PENDING"

    completed = pipeline.summarize(start, dict(rows=[
        dict(status="whole_path_passed", classification="whole_path_audit_passed",
             prefix_found=True, whole_path_passed=True)
    ]), dict(trials=[dict(physics_executed=True, completed=True)]))
    assert completed["case"] == "CASE3_LOCAL_PHYSICS_EVIDENCE_AVAILABLE"


def test_new_start_local_design_uses_historical_heading_without_mutating_reset():
    import mujoco as mj
    from types import SimpleNamespace
    from scipy.spatial.transform import Rotation
    from run_motion_family_search import approach_reference_heading
    from test_d1_recovery import geometry_and_parameters
    from recovery_design import matched_local_variant
    from approach_connection import handoff_index
    m=mj.MjModel.from_xml_string('<mujoco><worldbody><body name="arm"><joint axis="0 0 1"/><geom size=".01"/><body name="ring" pos="1 0 0"/></body></worldbody></mujoco>')
    d=mj.MjData(m);d.qpos[0]=.4;mj.mj_forward(m,d)
    e=SimpleNamespace(model=m,data=d,qids=np.array([0]),initial=np.array([.4]))
    probe=SimpleNamespace(frame=lambda data:(data.xpos[m.body('ring').id].copy(),Rotation.identity()))
    mf,geometry,values=geometry_and_parameters()
    before=d.qpos.copy();new_heading=approach_reference_heading(e,probe,geometry['center'])
    old_heading=approach_reference_heading(e,probe,geometry['center'],[0.])
    np.testing.assert_array_equal(d.qpos,before)
    np.testing.assert_allclose(old_heading,[1.,0.,0.],atol=1e-15)
    assert not np.allclose(new_heading,old_heading)
    original=mf.make_candidate('under_center',values,**geometry,candidate_id='same')
    with pytest.raises(ValueError,match='changed approach'):
        matched_local_variant(original,dict(geometry,heading=new_heading),'neutral')
    fixed=matched_local_variant(original,dict(geometry,heading=old_heading),'neutral')
    end=handoff_index(original)
    assert fixed['pose_waypoints'][:end+1]==original['pose_waypoints'][:end+1]
    assert fixed['pose_waypoints'][end+1:]!=original['pose_waypoints'][end+1:]
    with pytest.raises(ValueError,match='reference q'):
        approach_reference_heading(e,probe,geometry['center'],[float('nan')])
