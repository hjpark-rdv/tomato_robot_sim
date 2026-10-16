import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from compare_newton_runs import compare_runs


class CompareNewtonTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.a, self.b = self.root / "a", self.root / "b"
        self.a.mkdir()
        self.b.mkdir()
        n = 4
        poses = np.zeros((n, 2, 7))
        poses[..., -1] = 1.
        poses[:, 0, 0] = np.arange(n) / 100
        self.states = dict(time=np.arange(n) / 60, qpos=np.ones((n, 3)),
            qvel=np.zeros((n, 3)), eq_active=np.ones((n, 1), bool), poses=poses,
            robot_q=np.ones((n, 2)), command=np.ones((n, 10)),
            phase=np.asarray(["initial", "close", "close", "open"]),
            belt_surface_velocity_local_y_m_s=np.zeros((n, 2)),
            body_paths=np.asarray(["/World/ElasticTarget/fruit", "/World/ActuatedTool/fixed"]))
        self.states.update(scheduled_time=self.states["time"].copy(), solver_step=np.arange(n))
        self.metrics = dict(active_penetration_m=np.zeros((1, 3)),
            released_penetration_m=np.zeros((1, 3)), max_contact_force_n=np.zeros((1, 3)),
            attachment_force_n=np.zeros((1, 3, 1)), attachment_torque_nm=np.zeros((1, 3, 1)),
            break_step=np.full((1, 1), -1), first_event=np.full(1, np.iinfo(np.uint64).max, np.uint64),
            nonfinite_step=np.full(1, np.iinfo(np.int32).max))
        self.summary = dict(status="completed", stop=None, requested_steps=3, actual_steps=3,
            physics_hz=60., record_stride=1, unrecorded_warmup_steps=0, simulated_s=.05,
            input_sha256="input", commands_sha256="commands", worlds=1,
            newton_version="1.7", warp_version="1.10", mujoco_version="3.15",
            cuda_graph=True, graph_steps=10, gui_requested=False,
            active_contact_criterion_passed=True,finite_recorded_state=True,
            solver_capacity_valid=True,qacc_finite=True,max_active_penetration_m=0.,
            engine=dict(source_xml_sha256="xml", source_model_array_sha256="model",
                        backend="mujoco_warp_cuda", source_counts=dict(nq=3, nv=3, neq=1)))
        self.write(self.a)
        self.write(self.b)

    def tearDown(self):
        self.temporary.cleanup()

    def write(self, path, states=None, metrics=None, summary=None, breaks=None, schedule=None):
        np.savez(path / "states.npz", **(self.states if states is None else states))
        np.savez(path / "physics_metrics.npz", **(self.metrics if metrics is None else metrics))
        summary = copy.deepcopy(self.summary if summary is None else summary)
        summary["states_sha256"] = hashlib.sha256((path / "states.npz").read_bytes()).hexdigest()
        schedule = (dict(time_edges=self.states["time"], dt=np.diff(self.states["time"]),
            command=self.states["command"][1:], phase=self.states["phase"][1:],
            belt_surface_velocity_local_y_m_s=-self.states["belt_surface_velocity_local_y_m_s"][1:])
            if schedule is None else schedule)
        np.savez(path / "executed_commands.npz", **schedule)
        summary["executed_commands_sha256"] = hashlib.sha256((path / "executed_commands.npz").read_bytes()).hexdigest()
        (path / "summary.json").write_text(json.dumps(summary))
        (path / "break_events.json").write_text(json.dumps([] if breaks is None else breaks))

    def test_complete_identical_runs_pass_with_all_arrays_exact(self):
        result = compare_runs(self.a, self.b)
        self.assertTrue(result["passed"])
        self.assertTrue(result["all_compared_arrays_exact"])
        self.assertFalse(result["gui_physics_equivalence_verified"])

    def test_matching_common_prefix_cannot_pass(self):
        states = {key: value[:3] if key != "body_paths" else value for key, value in self.states.items()}
        summary = dict(self.summary, actual_steps=2, status="stopped", stop="contact")
        self.write(self.b, states=states, summary=summary)
        result = compare_runs(self.a, self.b)
        self.assertFalse(result["passed"])
        self.assertTrue(result["partial_common_prefix_only"])
        self.assertFalse(result["same_sample_count"])

    def test_identical_invalid_full_trials_do_not_validate_physics(self):
        summary=dict(self.summary,active_contact_criterion_passed=False,
                     max_active_penetration_m=.002)
        self.write(self.a,summary=summary)
        self.write(self.b,summary=summary)
        result=compare_runs(self.a,self.b)
        self.assertTrue(result['full_requested_physics_trace_matches'])
        self.assertFalse(result['passed'])
        self.assertFalse(result['physical_validity']['actual']['passed'])

    def test_identical_float32_quaternion_has_zero_orientation_error(self):
        states=copy.deepcopy(self.states)
        states['poses']=states['poses'].astype(np.float32)
        quaternion=np.array([.347, .129, .517, .771],np.float32)
        quaternion/=np.linalg.norm(quaternion)
        states['poses'][...,3:]=quaternion
        self.write(self.a,states=states)
        self.write(self.b,states=states)
        result=compare_runs(self.a,self.b)
        self.assertTrue(result['passed'])
        self.assertEqual(result['pose_metrics']['maximum_orientation_error_rad'],0.)

    def test_seconds_limited_matching_runs_cannot_prove_full_harvest_equivalence(self):
        states={key:value[:3] if key!='body_paths' else value
                for key,value in self.states.items()}
        metrics={key:(value[:,:2] if value.ndim>=2 and key not in ('break_step',) else value)
                 for key,value in self.metrics.items()}
        summary=dict(self.summary,requested_steps=2,actual_steps=2,simulated_s=2/60)
        self.write(self.a,states=states,metrics=metrics,summary=summary)
        self.write(self.b,states=states,metrics=metrics,summary=summary)
        result=compare_runs(self.a,self.b)
        self.assertFalse(result['passed'])

    def test_identically_stopped_full_arrays_still_cannot_pass_requested_run(self):
        summary = dict(self.summary, status="stopped", requested_steps=10, stop="contact")
        self.write(self.a, summary=summary)
        self.write(self.b, summary=summary)
        result = compare_runs(self.a, self.b)
        self.assertTrue(result["recorded_evidence_matches"])
        self.assertFalse(result["passed"])

    def test_command_equality_constraint_and_velocity_changes_are_failures(self):
        for key in ("command", "eq_active", "qvel"):
            with self.subTest(key=key):
                states = {name: value.copy() for name, value in self.states.items()}
                states[key][-1, 0] = 0 if key == "eq_active" else states[key][-1, 0] + .001
                self.write(self.b, states=states)
                result = compare_runs(self.a, self.b)
                self.assertFalse(result["passed"])
                self.assertFalse(result["state_arrays"][key]["exact"])

    def test_contact_metrics_and_break_events_are_not_hidden_by_equal_poses(self):
        metrics = {name: value.copy() for name, value in self.metrics.items()}
        metrics["attachment_force_n"][0, 2, 0] = 3.1
        self.write(self.b, metrics=metrics, breaks=[dict(world=0, name="fruit", solver_step=2)])
        result = compare_runs(self.a, self.b)
        self.assertFalse(result["passed"])
        self.assertFalse(result["break_events_exact"])
        self.assertFalse(result["physics_metrics"]["attachment_force_n"]["passed"])

    def test_holes_in_timestamps_or_identical_truncated_metrics_cannot_pass(self):
        states = {name: value.copy() for name, value in self.states.items()}
        states["time"][1] += .001
        self.write(self.b, states=states)
        self.assertFalse(compare_runs(self.a, self.b)["passed"])
        metrics = {name: value.copy() for name, value in self.metrics.items()}
        metrics["active_penetration_m"] = metrics["active_penetration_m"][:, :-1]
        self.write(self.a, metrics=metrics)
        self.write(self.b, metrics=metrics)
        self.assertFalse(compare_runs(self.a, self.b)["passed"])

    def test_nonfinite_or_states_hash_mismatch_cannot_pass(self):
        states = {name: value.copy() for name, value in self.states.items()}
        states["qpos"][0, 0] = np.nan
        self.write(self.b, states=states)
        self.assertFalse(compare_runs(self.a, self.b)["passed"])
        self.write(self.b)
        summary = json.loads((self.b / "summary.json").read_text())
        summary["states_sha256"] = "wrong"
        (self.b / "summary.json").write_text(json.dumps(summary))
        self.assertFalse(compare_runs(self.a, self.b)["passed"])

    def test_gui_flag_alone_does_not_prove_gui_equivalence(self):
        self.write(self.b, summary=dict(self.summary, gui_requested=True))
        result = compare_runs(self.a, self.b)
        self.assertTrue(result["full_requested_physics_trace_matches"])
        self.assertFalse(result["passed"])
        self.assertFalse(result["gui_physics_equivalence_verified"])
        (self.b / "newton_viewer_process.json").write_text(json.dumps(dict(ready=True)))
        (self.b / "newton_live.json").write_text(json.dumps(dict(closed=True, viewer_owns_physics=False)))
        viewer = dict(physics_steps=0, timeline_time_s=0., timeline_auto_update=False,
                      render_updates=10, last_measured_time_s=.05)
        (self.b / "live_viewer_123.json").write_text(json.dumps(viewer))
        result = compare_runs(self.a, self.b)
        self.assertTrue(result["passed"])
        self.assertTrue(result["gui_physics_equivalence_verified"])
        viewer["last_measured_time_s"] = .033
        (self.b / "live_viewer_123.json").write_text(json.dumps(viewer))
        self.assertFalse(compare_runs(self.a, self.b)["passed"])

    def test_unsigned_event_sentinel_change_is_detected_without_wraparound(self):
        metrics = {name: value.copy() for name, value in self.metrics.items()}
        metrics["first_event"][0] = 17
        self.write(self.b, metrics=metrics)
        result = compare_runs(self.a, self.b)
        self.assertFalse(result["passed"])
        self.assertEqual(result["physics_metrics"]["first_event"]["maximum_abs_error"], 2**64 - 18)

    def test_physical_model_or_input_hash_changes_do_not_pass(self):
        summary = copy.deepcopy(self.summary)
        summary["engine"]["source_model_array_sha256"] = "different"
        self.write(self.b, summary=summary)
        result = compare_runs(self.a, self.b)
        self.assertFalse(result["passed"])
        self.assertFalse(result["settings"]["engine.source_model_array_sha256"]["matches"])

    def test_nonuniform_saved_time_edges_define_step_coverage(self):
        states = {name: value.copy() for name, value in self.states.items()}
        edges = np.asarray([0., 5 / 300, 6 / 300, 11 / 300])
        states["time"] = edges
        states["scheduled_time"] = edges.copy()
        schedule = dict(time_edges=edges, dt=np.diff(edges), command=states["command"][1:],
                        phase=states["phase"][1:],
                        belt_surface_velocity_local_y_m_s=-states["belt_surface_velocity_local_y_m_s"][1:])
        summary = dict(self.summary, physics_hz=300., simulated_s=float(edges[-1]))
        for path in (self.a, self.b):
            self.write(path, states=states, summary=summary, schedule=schedule)
        result = compare_runs(self.a, self.b)
        self.assertTrue(result["passed"])
        self.assertTrue(result["coverage"]["reference"]["saved_timestep_schedule_verified"])
        # Matching state arrays cannot excuse a different signed dt schedule.
        schedule["dt"] = np.full(3, 1 / 300)
        for path in (self.a, self.b):
            self.write(path, states=states, summary=summary, schedule=schedule)
        self.assertFalse(compare_runs(self.a, self.b)["passed"])

    def test_gpu_clock_drift_needs_step_indices_and_independent_schedule(self):
        states = {name: value.copy() for name, value in self.states.items()}
        states["time"] += np.asarray([0., .001, .002, .003])
        for path in (self.a, self.b):
            self.write(path, states=states)
        result = compare_runs(self.a, self.b)
        self.assertTrue(result["passed"])
        self.assertTrue(result["coverage"]["reference"]["recorded_clock_drift_diagnostic_only"])
        self.assertFalse(result["coverage"]["reference"]["expected_timestamp_error_within_roundoff"])
        states.pop("solver_step")
        for path in (self.a, self.b):
            self.write(path, states=states)
        self.assertFalse(compare_runs(self.a, self.b)["passed"])
        states = {name: value.copy() for name, value in self.states.items()}
        states["solver_step"][2] = 1
        for path in (self.a, self.b):
            self.write(path, states=states)
        self.assertFalse(compare_runs(self.a, self.b)["passed"])

    def test_cpu_uniform_or_variable_clock_has_strict_roundoff_fallback(self):
        states = {name: value.copy() for name, value in self.states.items()
                  if name not in ("scheduled_time", "solver_step")}
        summary = copy.deepcopy(self.summary)
        summary["engine"]["backend"] = "native_mujoco_cpu"
        for path in (self.a, self.b):
            self.write(path, states=states, summary=summary)
        result = compare_runs(self.a, self.b)
        self.assertTrue(result["passed"])
        self.assertTrue(result["coverage"]["reference"]["cpu_float64_clock_coverage_fallback"])
        states["time"][-1] += 1e-5
        for path in (self.a, self.b):
            self.write(path, states=states, summary=summary)
        self.assertFalse(compare_runs(self.a, self.b)["passed"])

    def test_executed_schedule_hash_is_checked_independently(self):
        with (self.b / "executed_commands.npz").open("ab") as stream:
            stream.write(b"changed after execution")
        result = compare_runs(self.a, self.b)
        self.assertFalse(result["passed"])
        self.assertFalse(result["coverage"]["actual"]["executed_commands_sha_matches_summary"])


if __name__ == "__main__":
    unittest.main()
