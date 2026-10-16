#!/usr/bin/env python3
"""Compare complete fixed-step Newton evidence without rerunning physics.

Reuses the existing S300 quaternion and JSON diagnostics. A matching prefix,
matching summaries alone, or two identically stopped trials cannot pass this
full requested-run comparison. GUI evidence is checked separately from physics.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from compare_motion_runs import json_safe, maximum, quaternion_angle


DEFAULT_TOLERANCES = dict(qpos_abs=0., qvel_abs=0., position_m=0., rotation_rad=0.,
                          penetration_m=0., force_n=0., torque_nm=0., timestamp_s=1e-9)
STATE_KEYS = ("time", "qpos", "qvel", "eq_active", "poses", "robot_q", "phase", "command",
              "belt_surface_velocity_local_y_m_s")
METRIC_KEYS = ("active_penetration_m", "released_penetration_m", "max_contact_force_n",
               "attachment_force_n", "attachment_torque_nm", "break_step", "first_event", "nonfinite_step")


def _sha(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def _load(folder):
    folder = Path(folder)
    result = dict(folder=folder, errors=[])
    for key, name in (("summary", "summary.json"), ("breaks", "break_events.json")):
        try:
            result[key] = json.loads((folder / name).read_text())
        except (OSError, ValueError) as error:
            result[key] = {} if key == "summary" else None
            result["errors"].append(f"{name}: {error}")
    for key, name in (("states", "states.npz"), ("metrics", "physics_metrics.npz"),
                      ("schedule", "executed_commands.npz")):
        try:
            with np.load(folder / name, allow_pickle=False) as archive:
                result[key] = {name: archive[name].copy() for name in archive.files}
        except (OSError, ValueError) as error:
            result[key] = {}
            result["errors"].append(f"{name}: {error}")
    expected = result["summary"].get("states_sha256")
    result["states_sha_matches_summary"] = bool(expected and (folder / "states.npz").is_file()
                                               and _sha(folder / "states.npz") == expected)
    expected_commands = result["summary"].get("executed_commands_sha256")
    result["executed_commands_sha_matches_summary"] = bool(expected_commands and
        (folder / "executed_commands.npz").is_file() and
        _sha(folder / "executed_commands.npz") == expected_commands)
    return result


def _array_comparison(left, right, tolerance=0.):
    report = dict(reference_shape=list(left.shape), actual_shape=list(right.shape),
                  same_shape=left.shape == right.shape, exact=False, passed=False,
                  maximum_abs_error=None, tolerance=tolerance)
    numeric = left.dtype.kind in "biuf" and right.dtype.kind in "biuf"
    report["all_values_finite"] = bool(not numeric or
        (np.isfinite(left).all() and np.isfinite(right).all()))
    if left.shape != right.shape:
        return report
    report["exact"] = bool(np.array_equal(left, right))
    if not numeric:
        report["passed"] = report["exact"]
        return report
    mismatch = left != right
    if np.any(mismatch):
        first = tuple(int(x) for x in np.argwhere(mismatch)[0])
        report["first_different_index"] = first
        report["reference_first_value"] = left[first].item()
        report["actual_first_value"] = right[first].item()
    if left.dtype.kind in "biu" and right.dtype.kind in "biu":
        # Avoid unsigned sentinel wraparound and loss of uint64 event bits.
        report["maximum_abs_error"] = max((abs(int(a) - int(b)) for a, b in
            zip(left[mismatch], right[mismatch])), default=0)
        report["passed"] = report["exact"] and report["all_values_finite"]
    else:
        report["maximum_abs_error"] = maximum(np.abs(left.astype(float) - right.astype(float)))
        report["passed"] = bool(report["all_values_finite"] and report["maximum_abs_error"] <= tolerance)
    return report


def _coverage(run, timestamp_tolerance=1e-9):
    summary, states = run["summary"], run["states"]
    time = np.asarray(states.get("time", []))
    requested, actual, hz = (summary.get("requested_steps"), summary.get("actual_steps"),
                              summary.get("physics_hz"))
    stride = summary.get("record_stride")
    valid_metadata = bool(isinstance(requested, int) and requested > 0 and
                          isinstance(actual, int) and actual >= 0 and
                          isinstance(hz, (int, float)) and np.isfinite(hz) and hz > 0)
    schedule = run["schedule"]
    edges = np.asarray(schedule.get("time_edges", []))
    dt = np.asarray(schedule.get("dt", []))
    # Old fixed inputs did not save dt. Their signed executed time_edges still
    # completely define the timestep schedule and are not inferred from Hz.
    if "dt" not in schedule:
        dt = np.diff(edges)
    valid_schedule = bool(valid_metadata and edges.ndim == 1 and len(edges) > requested
        and np.isfinite(edges).all() and edges[0] == 0 and np.all(np.diff(edges) > 0)
        and dt.shape == (len(edges) - 1,) and np.isfinite(dt).all()
        and np.array_equal(dt, np.diff(edges))
        and np.asarray(schedule.get("command", [])).shape == (len(dt), 10)
        and np.asarray(schedule.get("phase", [])).shape == (len(dt),)
        and np.asarray(schedule.get("belt_surface_velocity_local_y_m_s", [])).shape == (len(dt), 2)
        and run["executed_commands_sha_matches_summary"])
    expected = edges[:actual + 1] if valid_schedule else np.asarray([])
    same_time_shape = time.shape == expected.shape and len(expected) > 0
    maximum_clock_error = (maximum(np.abs(time - expected)) if same_time_shape else None)
    measured_time_valid = bool(same_time_shape and np.isfinite(time).all() and
                               time[0] == 0 and np.all(np.diff(time) > 0))
    step_indices = np.asarray(states.get("solver_step", []))
    scheduled_time = np.asarray(states.get("scheduled_time", []))
    explicit_steps = bool(valid_metadata and step_indices.shape == (actual + 1,) and
                           step_indices.dtype.kind in "iu" and
                           np.array_equal(step_indices, np.arange(actual + 1)))
    explicit_schedule = bool(valid_schedule and scheduled_time.shape == expected.shape and
        np.isfinite(scheduled_time).all() and maximum(np.abs(scheduled_time - expected)) <= timestamp_tolerance)
    cpu_backend = summary.get("engine", {}).get("backend") == "native_mujoco_cpu"
    # CUDA's native data.time is float32 and accumulates rounding drift. Never
    # enlarge a universal timestamp tolerance to disguise that drift: require
    # explicit complete solver indices + independent signed scheduled times,
    # and compare the real measured clock exactly between the two runs.
    clock_within_roundoff = bool(measured_time_valid and maximum_clock_error <= timestamp_tolerance)
    complete_time = bool(valid_schedule and measured_time_valid and
        ((explicit_steps and explicit_schedule) or (cpu_backend and clock_within_roundoff)))
    commands_follow_schedule = bool(valid_schedule and
        np.asarray(states.get("command", [])).shape == (actual + 1, 10) and
        np.array_equal(states["command"][1:], schedule["command"][:actual]) and
        np.asarray(states.get("phase", [])).shape == (actual + 1,) and
        np.array_equal(states["phase"][1:], schedule["phase"][:actual]) and
        np.asarray(states.get("belt_surface_velocity_local_y_m_s", [])).shape == (actual + 1, 2) and
        np.array_equal(states["belt_surface_velocity_local_y_m_s"][1:],
                       -schedule["belt_surface_velocity_local_y_m_s"][:actual]))
    sample_lengths = {key: (int(states[key].shape[0]) if key in states and states[key].ndim else None)
                      for key in (*STATE_KEYS, *(key for key in ("scheduled_time", "solver_step") if key in states))}
    arrays_complete = bool(len(time) > 0 and all(value == len(time) for value in sample_lengths.values()))
    metric_lengths = {}
    worlds = summary.get("worlds")
    for key in METRIC_KEYS:
        value = run["metrics"].get(key)
        if key in ("break_step", "first_event", "nonfinite_step"):
            metric_lengths[key] = bool(value is not None and value.ndim >= 1 and value.shape[0] == worlds)
        else:
            metric_lengths[key] = bool(value is not None and value.ndim >= 2 and
                                       value.shape[0] == worlds and value.shape[1] == actual)
    metrics_complete = all(metric_lengths.values())
    full_schedule_requested = bool(valid_schedule and requested == len(dt))
    full = bool(valid_metadata and full_schedule_requested and actual == requested and summary.get("status") == "completed"
                and summary.get("stop") is None and stride == 1 and complete_time
                and arrays_complete and metrics_complete and commands_follow_schedule
                and summary.get("unrecorded_warmup_steps") == 0
                and run["states_sha_matches_summary"] and not run["errors"])
    return dict(passed=full, requested_steps=requested, actual_steps=actual,
        full_command_schedule_requested=full_schedule_requested,
        status=summary.get("status"), stop=summary.get("stop"), record_stride=stride,
        recorded_every_physics_step=stride == 1, starts_at_zero=bool(len(time) and time[0] == 0),
        samples=len(time), exact_expected_timestamps=bool(same_time_shape and np.array_equal(time, expected)),
        expected_timestamp_error_within_roundoff=clock_within_roundoff,
        measured_timestamps_strictly_increasing=measured_time_valid,
        explicit_solver_step_coverage=explicit_steps, explicit_scheduled_time_coverage=explicit_schedule,
        cpu_float64_clock_coverage_fallback=bool(cpu_backend and clock_within_roundoff and not explicit_steps),
        maximum_expected_clock_error_s=maximum_clock_error,
        recorded_clock_drift_diagnostic_only=bool(complete_time and not clock_within_roundoff),
        schedule_uses_saved_nonuniform_time_edges=True, saved_timestep_schedule_verified=valid_schedule,
        state_commands_follow_executed_schedule=commands_follow_schedule,
        executed_commands_sha_matches_summary=run["executed_commands_sha_matches_summary"],
        state_sample_lengths=sample_lengths,
        every_state_array_has_all_samples=arrays_complete,
        physics_metric_coverage=metric_lengths, all_physics_metrics_cover_run=metrics_complete,
        states_sha_matches_summary=run["states_sha_matches_summary"],
        unrecorded_warmup_steps=summary.get("unrecorded_warmup_steps"), load_errors=run["errors"])


def _gui_evidence(run):
    if not run["summary"].get("gui_requested"):
        return dict(requested=False, passed=True, scope="No GUI requested in this trial")
    folder = run["folder"]
    documents = {}
    for key, name in (("producer", "newton_live.json"), ("process", "newton_viewer_process.json")):
        try:
            documents[key] = json.loads((folder / name).read_text())
        except (OSError, ValueError):
            documents[key] = {}
    viewers = []
    for path in sorted(folder.glob("live_viewer_*.json")):
        try:
            viewers.append(dict(path=str(path), report=json.loads(path.read_text())))
        except (OSError, ValueError):
            pass
    process, producer = documents["process"], documents["producer"]
    valid = [row for row in viewers if row["report"].get("physics_steps") == 0 and
             row["report"].get("timeline_time_s") == 0 and
             row["report"].get("timeline_auto_update") is False and
             row["report"].get("render_updates", 0) > 0 and
             row["report"].get("last_measured_time_s", 0) > 0 and not row["report"].get("error")]
    last_displayed = max((row["report"].get("last_measured_time_s", 0.) for row in valid), default=0.)
    stamps = run["states"].get("time", [])
    final = float(stamps[-1]) if len(stamps) else run["summary"].get("simulated_s")
    final_seen = bool(final is not None and last_displayed >= final - 1e-9)
    passed = bool(process.get("ready") is True and producer.get("closed") is True
                  and producer.get("viewer_owns_physics") is False and valid and final_seen)
    return dict(requested=True, passed=passed, viewer_ready=process.get("ready") is True,
        live_viewer_reports=[row["path"] for row in viewers], valid_live_viewer_reports=len(valid),
        last_displayed_physics_time_s=last_displayed, final_physics_time_s=final,
        final_measured_frame_seen=final_seen,
        viewer_physics_steps=0 if valid else None,
        scope="A concurrent isolated GUI consuming measured Newton state; no second physics run")


def compare_runs(baseline, candidate, tolerances=None):
    limits = dict(DEFAULT_TOLERANCES)
    limits.update(tolerances or {})
    if any(not np.isfinite(value) or value < 0 for value in limits.values()):
        raise ValueError("Comparison tolerances must be finite and nonnegative")
    ref, new = _load(baseline), _load(candidate)
    coverage = dict(reference=_coverage(ref, limits["timestamp_s"]),
                    actual=_coverage(new, limits["timestamp_s"]))
    arrays = {}
    keys = [*STATE_KEYS, *(key for key in ("scheduled_time", "solver_step")
                          if key in ref["states"] or key in new["states"])]
    for key in keys:
        if key not in ref["states"] or key not in new["states"]:
            arrays[key] = dict(passed=False, reason="required state array missing")
            continue
        arrays[key] = _array_comparison(ref["states"][key], new["states"][key],
                                        limits.get(key + "_abs", 0.))
    left_paths = list(map(str, ref["states"].get("body_paths", [])))
    right_paths = list(map(str, new["states"].get("body_paths", [])))
    paths_match = bool(left_paths and left_paths == right_paths and
                       len(set(left_paths)) == len(left_paths))
    pose_metrics = dict(available=False)
    if paths_match and "poses" in ref["states"] and "poses" in new["states"]:
        a, b = ref["states"]["poses"], new["states"]["poses"]
        if a.shape == b.shape and a.ndim == 3 and a.shape[1:] == (len(left_paths), 7):
            finite = bool(np.isfinite(a).all() and np.isfinite(b).all() and
                           np.all(np.linalg.norm(a[..., 3:], axis=-1) > 1e-12) and
                           np.all(np.linalg.norm(b[..., 3:], axis=-1) > 1e-12))
            position = maximum(np.linalg.norm(a[..., :3] - b[..., :3], axis=-1))
            # Identical stored float32 quaternions denote exactly the same
            # orientation. Renormalizing/dotting them can round below one and
            # fabricate a nonzero angle, even when every saved bit agrees.
            angle = (0. if np.array_equal(a[..., 3:], b[..., 3:]) else
                maximum(quaternion_angle(a[..., 3:].astype(float), b[..., 3:].astype(float)))) if finite else None
            passed = bool(finite and position <= limits["position_m"] and angle <= limits["rotation_rad"])
            pose_metrics = dict(available=True, passed=passed, maximum_position_error_m=position,
                                maximum_orientation_error_rad=angle, all_values_finite=finite)
            arrays["poses"]["passed"] = passed
    metrics = {}
    for key in METRIC_KEYS:
        if key not in ref["metrics"] or key not in new["metrics"]:
            metrics[key] = dict(passed=False, reason="required physics metric missing")
            continue
        tolerance = (limits["penetration_m"] if "penetration" in key else
                     limits["torque_nm"] if "torque" in key else
                     limits["force_n"] if "force" in key else 0.)
        metrics[key] = _array_comparison(ref["metrics"][key], new["metrics"][key], tolerance)
    settings = {}
    for key in ("input_sha256", "commands_sha256", "executed_commands_sha256", "physics_hz", "worlds", "newton_version",
                "warp_version", "mujoco_version", "cuda_graph", "graph_steps"):
        a, b = ref["summary"].get(key), new["summary"].get(key)
        settings[key] = dict(reference=a, actual=b, matches=a is not None and a == b)
    for key in ("source_xml_sha256", "source_model_array_sha256", "backend", "source_counts"):
        a = ref["summary"].get("engine", {}).get(key)
        b = new["summary"].get("engine", {}).get(key)
        settings["engine." + key] = dict(reference=a, actual=b, matches=a is not None and a == b)
    for key in ('effective_gpu_options','gpu_linear_algebra','gpu_sparse_factor',
                'gpu_tree_sums','gpu_canonical_contacts','gpu_dense_hessian',
                'gpu_fixed_solver_reductions'):
        if key in ref['summary'] or key in new['summary']:
            a,b=ref['summary'].get(key),new['summary'].get(key)
            settings[key]=dict(matches=a is not None and a==b)
    breaks_match = ref["breaks"] is not None and ref["breaks"] == new["breaks"]
    trace_matches = bool(paths_match and all(row["passed"] for row in arrays.values()) and
                         all(row["passed"] for row in metrics.values()) and breaks_match)
    full_trace = bool(trace_matches and all(row["passed"] for row in coverage.values()) and
                      all(row["matches"] for row in settings.values()))
    gui = dict(reference=_gui_evidence(ref), actual=_gui_evidence(new))
    gui_pair = bool(ref["summary"].get("gui_requested") != new["summary"].get("gui_requested")
                    and any(run["summary"].get("gui_requested") for run in (ref, new)))
    validity={}
    for label,run in (('reference',ref),('actual',new)):
        flags={key:run['summary'].get(key) is True for key in
            ('active_contact_criterion_passed','finite_recorded_state','solver_capacity_valid','qacc_finite')}
        pen=run['summary'].get('max_active_penetration_m')
        flags['active_penetration_within_existing_1mm']=bool(isinstance(pen,(int,float)) and np.isfinite(pen) and 0<=pen<=.001)
        validity[label]=dict(passed=all(flags.values()),checks=flags)
    passed = bool(full_trace and all(row["passed"] for row in gui.values())
        and all(row['passed'] for row in validity.values()))
    return json_safe(dict(schema="s300-newton-full-trace-comparison-v1", baseline=str(baseline),
        candidate=str(candidate), passed=passed, full_requested_physics_trace_matches=full_trace,
        recorded_evidence_matches=trace_matches, gui_headless_pair=gui_pair,
        gui_physics_equivalence_verified=bool(passed and gui_pair), coverage=coverage,
        physical_validity=validity,
        settings=settings, body_path_order_matches=paths_match, state_arrays=arrays,
        pose_metrics=pose_metrics, physics_metrics=metrics, break_events_exact=breaks_match,
        gui_evidence=gui, numeric_tolerances=limits,
        partial_common_prefix_only=not all(row["passed"] for row in coverage.values()),
        same_sample_count=len(ref["states"].get("time", [])) == len(new["states"].get("time", [])),
        all_compared_arrays_exact=bool(all(row.get("exact", False) for row in arrays.values()) and
                                      all(row.get("exact", False) for row in metrics.values())),
        interpretation="Finite complete requested-run regression; no replayed or common-prefix success, no validated harvest claim",
        training_eligible=False, harvest_success=None))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("Use a new output file; previous evidence is preserved")
    result = compare_runs(args.baseline, args.candidate)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(output=str(args.output), passed=result["passed"],
        full_requested_physics_trace_matches=result["full_requested_physics_trace_matches"],
        gui_physics_equivalence_verified=result["gui_physics_equivalence_verified"]), indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
