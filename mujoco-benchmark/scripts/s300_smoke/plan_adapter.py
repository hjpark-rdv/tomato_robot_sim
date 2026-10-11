"""Reuse the existing S300 IK and phase planner for measured MuJoCo starts.

Only ``make_plan`` imports USD/the existing Isaac asset adapters. ``sample_plan``
is NumPy-only and can be imported by the native MuJoCo environment. Commands
are actuator targets, never recorded poses or writes to physical joint state.
The resulting plan has no self/environment collision certificate.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


REPO = Path(__file__).resolve().parents[3]
SOURCE_CODE = REPO / "nvidia-sim/harvest_s300"
DOF_NAMES = ["root_joint", "farmily_lift_height_joint", "base", "shoulder",
             "elbow", "wrist1", "wrist2", "wrist3", "jaw_L", "jaw_R"]


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _vector(value, size, name):
    result = np.asarray(value, float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must have {size} finite values")
    return result


def _request(request, *names, default=None):
    for name in names:
        if name in request:
            return request[name]
    return default


def _smooth(fraction):
    u = float(np.clip(fraction, 0., 1.))
    return u * u * (3. - 2. * u)


def sample_plan(plan, time_s, *, settle_seconds=None):
    """Return a 10-DOF command and authored belt command at simulation time.

    All rail values in this adapter's plans are relative to the *one* supplied
    ``rail_origin_station_m``. The source executor's phase smoothstep, uniform
    waypoint interpolation, gap interpolation and whole-cycle belt ramp are
    retained. Belt values use the source PhysX authoring convention: +local-Y
    command transports fruit toward CAD -Y. Native MuJoCo callers must negate
    this value when setting physical surface velocity (verified source sign).
    Feedback guards are described in the plan and must be checked by execution;
    sampling alone never certifies measured jaw opening or rail arrival.
    """
    t = float(time_s)
    if not np.isfinite(t) or t < 0:
        raise ValueError("time_s must be finite and nonnegative")
    settle = float(plan.get("settle_seconds", 2.) if settle_seconds is None else settle_seconds)
    if not np.isfinite(settle) or settle < 0:
        raise ValueError("settle_seconds must be finite and nonnegative")
    initial = _vector(plan["initial_command"], 10, "initial_command").copy()
    phases = plan["phases"]
    if not phases:
        raise ValueError("A command plan must contain phases")
    if t < settle:
        return dict(command=initial, phase="settle", phase_index=-1,
                    belt_surface_velocity_local_y_m_s=np.zeros(2),
                    authored_belt_speed=np.zeros(2), gap_m=float(.044 + initial[9] - initial[8]),
                    completed=False)
    elapsed = t - settle
    offset = elapsed
    previous_gap = float(plan.get("initial_gap_m", .044 + initial[9] - initial[8]))
    previous_rail = float(initial[0])
    for index, row in enumerate(phases):
        duration = float(row["duration_s"])
        qs = np.asarray(row["waypoint_q"], float)
        rail = np.asarray(row.get("rail_native_command_positions_m",
                                  row.get("rail_waypoint_positions_m", [previous_rail] * len(qs))), float)
        next_gap = float(row["end_gap_m"])
        if (not np.isfinite(duration) or duration <= 0 or qs.ndim != 2 or
                qs.shape[1] != 7 or len(qs) < 2 or rail.shape != (len(qs),) or
                not np.isfinite(qs).all() or not np.isfinite(rail).all() or not np.isfinite(next_gap)):
            raise ValueError("Invalid command waypoints, duration or rail mapping")
        last = index == len(phases) - 1
        if offset < duration or last:
            completed = bool(last and offset >= duration)
            u = _smooth(offset / duration)
            f = u * (len(qs) - 1)
            low = min(int(f), len(qs) - 2)
            weight = f - low
            q = qs[low] * (1. - weight) + qs[low + 1] * weight
            rail_q = rail[low] * (1. - weight) + rail[low + 1] * weight
            gap = previous_gap + (next_gap - previous_gap) * u
            speed = _vector(row["belt_local_y_m_s"], 2, "belt_local_y_m_s").copy()
            window = plan.get("belt_cycle")
            ramp = float(plan.get("belt_ramp_seconds", (plan.get("belt_spin") or {}).get("ramp_seconds", 0.)))
            if window and ramp and row["name"] in window["phases"]:
                duration_window = float(window["duration_s"])
                window_time = elapsed - float(window["start_after_settle_s"])
                if not 0 <= ramp <= duration_window / 2:
                    raise ValueError("Belt ramp exceeds the existing cycle duration")
                speed *= _smooth(min(window_time, duration_window - window_time) / ramp)
            command = np.r_[rail_q, q, -(gap - .044) / 2., (gap - .044) / 2.]
            return dict(command=command, phase="finished" if completed else row["name"],
                        phase_index=index, belt_surface_velocity_local_y_m_s=speed,
                        authored_belt_speed=speed.copy(), gap_m=float(gap), completed=completed)
        offset -= duration
        previous_gap = next_gap
        previous_rail = float(rail[-1])
    raise AssertionError("No phase was sampled")


def make_plan(assembly, request, *, source_plan=None):
    """Generate new IK from the measured seven-joint start using source code.

    Required request keys: ``measured_joint_q``, ``measured_station_y_m``,
    ``target_center`` (or ``target_center_world_m``), ``direction`` (or
    ``direction_world``). ``station_y_m`` is an optional destination; otherwise
    planning occurs at the measured station. ``rail_origin_station_m`` gives
    the shared MJB rail coordinate origin and is never reset between targets.
    ``plan_options`` may override inherited source planner settings.
    """
    from pxr import Usd
    if str(SOURCE_CODE) not in sys.path:
        sys.path.insert(0, str(SOURCE_CODE))
    from motion_kinematics import ToolKinematics, plan_motion
    from rail_motion import rail_destination, attach_rail_commands
    from truss_sequence import entry_direction

    assembly = Path(assembly).resolve()
    manifest_path = assembly / "motion_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    stage_path = assembly / "composed.usdc"
    if not stage_path.exists():
        stage_path = assembly / "scene.usda"
    stage = Usd.Stage.Open(str(stage_path))
    if stage is None:
        raise ValueError("Could not open the source zero-joint USD")
    kin = ToolKinematics(stage)
    measured_q = _vector(request["measured_joint_q"], 7, "measured_joint_q")
    measured_station = float(request["measured_station_y_m"])
    station = float(_request(request, "station_y_m", "target_station_y_m", default=measured_station))
    source_station = float(manifest["rail"]["initial_station_y_m"])
    origin_station = float(_request(request, "rail_origin_station_m", "rail_origin_station_y_m", default=source_station))
    if not np.isfinite([measured_station, station, source_station, origin_station]).all():
        raise ValueError("Source, measured, destination and rail origin stations must be finite")
    if not 0 <= measured_station <= 5 or not 0 <= station <= 5:
        raise ValueError("Stations must remain within the original 0..5m rail limits")
    # The USD has zero-joint bodies at its own saved rail origin. Do not use
    # measured source body poses as those zero frames, or shift the world twice.
    kin.world[:3, 3] += np.asarray(manifest["rail"]["axis_world"], float) * (station - source_station)
    center = _vector(_request(request, "target_center", "target_center_world_m"), 3, "target_center")
    direction = _vector(_request(request, "direction", "direction_world", "approach_direction_world"), 3, "direction")
    if abs(direction[2]) > 1e-9 or np.linalg.norm(direction) < 1e-9:
        raise ValueError("Insertion direction must be horizontal with nonzero length")
    direction /= np.linalg.norm(direction)
    yaw_offset = float(_request(request, "yaw_offset_deg", "truss_entry_yaw_offset_deg", default=0.))
    direction = entry_direction(dict(direction_world=direction.tolist()), yaw_offset)

    source = {}
    source_path = None
    if source_plan is not None:
        source_path = Path(source_plan).resolve()
        source = json.loads(source_path.read_text())
    options = dict(preparation_height_first=bool(source.get("preparation_height_first", True)),
                   horizontal_ready_connector=source.get("horizontal_ready_connector", "joint"),
                   horizontal_ready_seed_index=source.get("horizontal_ready_seed_index"),
                   ready_lift_offset_m=(source.get("ready_lift_branch") or {}).get("offset_m", -.15),
                   grasp_depth_m=source.get("grasp_depth_m", .035),
                   grasp_height_offset_m=source.get("grasp_height_offset_m", -.0015),
                   grasp_lateral_offset_m=0., grasp_translation_world_m=(0., 0., 0.),
                   max_grasp_translation_m=source.get("max_grasp_translation_m", 1.5),
                   approach_distance_m=source.get("approach_distance_m", .1),
                   horizontal_ready_normalize_turns=False)
    aliases = {"depth_m": "grasp_depth_m", "height_offset_m": "grasp_height_offset_m",
               "grasp_translation": "grasp_translation_world_m"}
    for key, value in request.items():
        mapped = aliases.get(key, key)
        if mapped in options:
            options[mapped] = value
    explicit = request.get("plan_options", {})
    if not isinstance(explicit, dict) or set(explicit) - set(options):
        raise ValueError("Unknown planner option in plan_options")
    options.update(explicit)
    if options["horizontal_ready_connector"] != "joint":
        options["horizontal_ready_seed_index"] = None
    phases, result = plan_motion(kin, center, start=measured_q,
        preparation="horizontal_ready", allow_measured_ready_start=True,
        harvest_profile="belt_open_close_cycle", preparation_opening="ready",
        preparation_gap_m=.044, pitch_deg=0., yaw_offset_deg=0.,
        approach_direction_world=direction, belt_speed_m_s=0., belt_infeed_speed_m_s=.05,
        close_seconds=6., belt_seconds=2., belt_ramp_seconds=.5, grasp_gap_m=.028,
        **options)
    # Existing rail helper produces current-origin-relative coordinates; rebase
    # its command rows once to the shared MJB origin without changing any bodies.
    rail_manifest = dict(manifest)
    rail_manifest["rail"] = dict(manifest["rail"], initial_station_y_m=measured_station,
                                 limits_relative_m=[-measured_station, 5. - measured_station])
    destination = rail_destination(rail_manifest, station, float(request.get("rail_speed_m_s", .08)))
    move_first = abs(station - measured_station) > 1e-9
    _, result = attach_rail_commands(phases, result, measured_q, destination, move_first=move_first)
    rebase = measured_station - origin_station
    for row in result["phases"]:
        row["rail_waypoint_positions_m"] = (np.asarray(row["rail_waypoint_positions_m"]) + rebase).tolist()
    jaws = _vector(request.get("measured_jaw_q", [0., 0.]), 2, "measured_jaw_q")
    initial_command = np.r_[rebase, measured_q, jaws]
    settle = float(request.get("settle_seconds", 2.))
    if not np.isfinite(settle) or settle < 0:
        raise ValueError("settle_seconds must be nonnegative")
    # Source phase scheduling starts each manipulation from the nominal closed
    # ready gap, retaining the source algorithm rather than altering its cycles.
    result.update(schema="s300-mujoco-measured-plan-adapter-v1", dof_names=DOF_NAMES,
                  initial_command=initial_command.tolist(), initial_gap_m=.044,
                  settle_seconds=settle, belt_ramp_seconds=.5,
                  total_duration_s=settle + sum(row["duration_s"] for row in result["phases"]),
                  rail_origin_station_m=origin_station, measured_station_y_m=measured_station,
                  destination_station_y_m=station, source_station_y_m=source_station,
                  measured_initial_joint_q=measured_q.tolist(), plan_options=options,
                  truss_entry_yaw_offset_deg=yaw_offset, collision_certified=False,
                  training_eligible=False, harvest_success=None,
                  execution_requirement="New finite-actuator physics; measured opening/arrival guards and collision checks required",
                  belt_sign="Authored PhysX localY +0.05; MuJoCo physical localY -0.05",
                  provenance=dict(assembly=str(assembly), stage=str(stage_path),
                                  stage_sha256=_sha(stage_path), manifest_sha256=_sha(manifest_path),
                                  source_plan=None if source_path is None else str(source_path),
                                  source_plan_sha256=None if source_path is None else _sha(source_path),
                                  source_code_sha256={name: _sha(SOURCE_CODE / name) for name in
                                      ("motion_kinematics.py", "rail_motion.py", "truss_sequence.py")},
                                  adapter_sha256=_sha(__file__),
                                  source_zero_frame_shift_world_m=[0., station - source_station, 0.]))
    # Validate the shared rail/command mapping at both ends while still offline.
    sample_plan(result, 0.)
    sample_plan(result, result["total_duration_s"])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assembly", type=Path)
    parser.add_argument("request", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--source-plan", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output path; existing plan outputs are preserved")
    request = json.loads(args.request.read_text())
    result = make_plan(args.assembly, request, source_plan=args.source_plan)
    result["provenance"].update(request=str(args.request.resolve()), request_sha256=_sha(args.request))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps(dict(output=str(args.output), duration_s=result["total_duration_s"],
                          phases=len(result["phases"]), collision_certified=False), ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
