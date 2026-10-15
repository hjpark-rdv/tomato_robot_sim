"""Fixed-step Newton CPU comparison using the existing physical controller.

This keeps Newton SolverMuJoCo.step as the integrator entry point. The native
MuJoCo control callback reuses Attachments.apply at the ordinary control phase,
and Session.released_contact_allowed supplies the existing contact exceptions.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time
import traceback
from types import SimpleNamespace

import mujoco as mj
import newton
import numpy as np
import psutil
from scipy.spatial.transform import Rotation
import warp as wp

from newton_inputs import ADAPTER, command_timeline, sha256, write_display_evidence
from newton_mjcf import create_newton_mjcf

if str(ADAPTER) not in sys.path:
    sys.path.insert(0, str(ADAPTER))
from run_harvest import Session
from run_smoke import Attachments


def _write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def native_state_finite(data):
    """Guard accelerations/time as well as integrated coordinates each step."""
    return bool(np.isfinite(data.time) and np.isfinite(data.qpos).all()
                and np.isfinite(data.qvel).all() and np.isfinite(data.qacc).all())


def record_display_error(report, stage, error, *, simulated_s, viewer=None):
    """A presentation failure must not discard the independent physics trace."""
    evidence = dict(stage=stage, error=str(error), simulated_s=float(simulated_s))
    if viewer is not None:
        evidence.update(returncode=viewer.process.poll(), log=str(viewer.log))
    report.setdefault('display_errors', []).append(evidence)
    report.update(gui_status='display_failed', gui_live_verified=False)
    print('[NEWTON GUI] display issue; preserving independent physics: ' + str(error), flush=True)
    return evidence


def run_cpu(args):
    """Run the same prepared commands through Newton's native CPU backend."""
    if args.worlds != 1:
        raise ValueError("The native Newton CPU backend supports one world per process")
    started = time.perf_counter()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = dict(schema="s300-newton-fixed-step-v1", status="initializing",
                  physics_engine="Newton SolverMuJoCo", backend="cpu",
                  newton_version=newton.__version__, warp_version=wp.__version__,
                  mujoco_version=mj.__version__, worlds=1,
                  gui_requested=args.gui, gui_advances_physics=False,
                  gui_process_independent=True, input=str(args.inputs.resolve()),
                  gui_status='requested' if args.gui else 'not_requested', gui_live_verified=False,
                  training_eligible=False, harvest_success=None)
    _write(output / "summary.json", report)
    publisher = viewer = None
    previous_control = mj.get_mjcb_control()
    previous_timer = mj.get_mjcb_time()
    profile_timers = bool(getattr(args,"profile_timers",False))
    if profile_timers:mj.set_mjcb_time(lambda:time.perf_counter()*1000.)
    q_rows, v_rows, e_rows, pose_rows, times, phases, commands, belts = [], [], [], [], [], [], [], []
    try:
        # Initialization forwards must not accidentally invoke a caller's old
        # global controller; the original callback is restored in finally.
        mj.set_mjcb_control(None)
        inputs = args.inputs.resolve()
        provenance = json.loads((inputs / "inputs.json").read_text())
        for name, evidence in provenance["input_files"].items():
            if evidence.get("sha256") and sha256(inputs / name) != evidence["sha256"]:
                raise ValueError("Prepared input SHA mismatch: " + name)
        initial = dict(np.load(inputs / "initial_state.npz", allow_pickle=False))
        timeline = dict(np.load(inputs / "commands.npz", allow_pickle=False))
        metadata = json.loads((inputs / "source_export.json").read_text())
        prepared_hz = float(provenance["timeline"]["hz"])
        hz = float(getattr(args, "hz", None) or prepared_hz)
        free_hz = getattr(args, "free_hz", None)
        if abs(prepared_hz - hz) > 1e-8 or free_hz is not None:
            initial_command = initial["qpos"][initial["robot_qpos_addresses"]]
            sampling_meta = dict(metadata, prepared_source_plan=str(inputs / "sampled_plan.json"))
            timeline, _, _ = command_timeline(sampling_meta, initial_command, hz, command_source="plan",
                                              free_hz=free_hz)
        step_dt = np.asarray(timeline.get("dt", np.diff(timeline["time_edges"])), dtype=float)
        if (len(step_dt) != len(timeline["time"]) or not np.isfinite(step_dt).all()
                or np.any(step_dt <= 0)):
            raise ValueError("Physics timesteps must be positive, finite, and match commands")
        np.savez_compressed(output / "executed_commands.npz", **timeline)
        steps = len(timeline["time"])
        if args.seconds is not None:
            steps = min(steps, int(np.searchsorted(timeline["time_edges"], args.seconds - 1e-9, side="left")))
        if steps < 1 or args.record_stride < 1:
            raise ValueError("Positive step count and recording stride required")
        source = mj.MjModel.from_binary_path(str(inputs / "input_model.mjb"))
        source.opt.timestep = 1. / hz
        if getattr(args, "solver_iterations", None) is not None:
            if args.solver_iterations<1:raise ValueError("Positive solver iteration budget required")
            source.opt.iterations=args.solver_iterations
        if getattr(args,"solver",None) is not None:
            source.opt.solver=getattr(mj.mjtSolver,"mjSOL_"+args.solver.upper())
        if getattr(args,"solver_tolerance",None) is not None:
            if args.solver_tolerance<=0:raise ValueError("Positive solver tolerance required")
            source.opt.tolerance=args.solver_tolerance
        if getattr(args,"jacobian",None) is not None:
            source.opt.jacobian=getattr(mj.mjtJacobian,"mjJAC_"+args.jacobian.upper())
        source.flg_surfacevel = True
        source.geom_surfacevel[:] = 0
        attachments = Attachments(source, metadata["attachments"])
        source.dof_damping[:] = initial["runtime_dof_damping"]
        engine = create_newton_mjcf(inputs / "input_scene.xml", source_model=source,
                                    device="cpu", use_mujoco_cpu=True,
                                    nconmax=args.nconmax, njmax=args.njmax,
                                    native_observables_only=getattr(args,"native_observables_only",False))
        model, data = engine.solver.mj_model, engine.solver.mj_data
        data.qpos[:] = initial["qpos"]
        data.qvel[:] = initial["qvel"]
        data.eq_active[:] = initial["eq_active"]
        data.ctrl[:] = initial["ctrl"]
        # No integration or unrecorded settling occurs before the first frame.
        mj.mj_forward(model, data)
        qadr = initial["robot_qpos_addresses"].astype(int)
        vadr = initial["robot_dof_addresses"].astype(int)
        aids = initial["actuator_ids"].astype(int)
        kp = model.actuator_gainprm[aids, 0]
        native_names = [model.body(i).name or "world" for i in range(model.nbody)]
        belt_geoms, belt_y = [], []
        for name in metadata["robot"]["belt_body_names"]:
            ids = np.flatnonzero((model.geom_bodyid == model.body(name).id) &
                                 ((model.geom_contype != 0) | (model.geom_conaffinity != 0)))
            belt_geoms.append(ids)
            belt_y.append(Rotation.from_quat(model.geom_quat[ids][:, [1, 2, 3, 0]]).inv().apply(
                np.tile([0., 1., 0.], (len(ids), 1))))
        current = dict(command=initial["qpos"][qadr], belt=np.zeros(2), torque=None,
                       callback_calls=0)

        def control_callback(callback_model, callback_data):
            callback_data.qfrc_applied[:] = initial["plant_preload"]
            callback_data.xfrc_applied[:] = 0
            current["torque"], _ = attachments.apply(callback_data)
            callback_data.ctrl[aids] = current["command"] + callback_data.qfrc_bias[vadr] / kp
            for side, ids in enumerate(belt_geoms):
                callback_model.geom_surfacevel[ids, :3] = belt_y[side] * current["belt"][side]
            current["callback_calls"] += 1

        mj.set_mjcb_control(control_callback)
        policy_context = SimpleNamespace(model=model, data=data, meta=metadata, at=attachments,
                                         released_gutter_diagnostic=bool(getattr(args, "released_gutter_diagnostic", False)))
        nfruit = len(attachments.ids)
        active = np.zeros((1, steps))
        released = np.zeros_like(active)
        force = np.zeros_like(active)
        force_log = np.zeros((1, steps, nfruit))
        torque_log = np.zeros_like(force_log)
        break_steps = np.full((1, nfruit), -1, dtype=np.int32)
        breaks, contacts, resources = [], [], []
        stop = first_violation = None
        nonfinite_step = np.iinfo(np.int32).max
        qacc_nonfinite_step = np.iinfo(np.int32).max
        physics_wall = 0.
        phase_performance = {}
        native_timer_names={int(v):k for k,v in mj.mjtTimer.__members__.items() if k!="mjNTIMER"}
        solver_iterations = np.zeros(steps, dtype=np.int32)
        constraint_counts = np.zeros(steps, dtype=np.int32)
        contact_counts = np.zeros(steps, dtype=np.int32)
        worker_count = int(getattr(args, "threads", 0))
        if worker_count:
            mj.mju_threadpool(data, worker_count)

        def record(phase, command, belt):
            # The Newton adapter already computes current native kinematics at
            # the end of every completed physics step (initial mj_forward above).
            times.append(float(data.time))
            q_rows.append(data.qpos.copy())
            v_rows.append(data.qvel.copy())
            e_rows.append(data.eq_active.copy())
            pose_rows.append(np.c_[data.xpos[1:], data.xquat[1:][:, [1, 2, 3, 0]]])
            phases.append(phase)
            commands.append(np.asarray(command).copy())
            belts.append(np.asarray(belt).copy())

        def live_frame():
            return (np.c_[data.xpos, data.xquat[:, [1, 2, 3, 0]]],
                    float(.044 + data.qpos[qadr[9]] - data.qpos[qadr[8]]))

        record("initial", current["command"], current["belt"])
        if args.gui:
            from newton_live import NewtonLivePublisher, NewtonViewerProcess

            publisher = NewtonLivePublisher(output, metadata["source_assembly"], native_names,
                                             provenance["source_body_to_native_name"], fps=args.viewer_fps,
                                             physical_model_sha256=sha256(inputs / "input_model.mjb"))
            pose, gap = live_frame()
            publisher.publish(pose, gap, 0., force=True)
            viewer = NewtonViewerProcess(output, seconds=args.viewer_seconds, camera=args.camera,
                                         fps=args.viewer_fps, capture=output / 'live_final.png')
            viewer.wait_ready(timeout=args.viewer_timeout)
        report.update(status="running", physics_hz=hz, requested_steps=steps,
                      free_hz=free_hz, variable_timestep=bool(np.ptp(step_dt) > 1e-12),
                      source_model_counts=provenance["model_counts"], input_sha256=sha256(inputs / "inputs.json"),
                      commands_sha256=sha256(inputs / "commands.npz"), engine=engine.evidence,
                      executed_commands_sha256=sha256(output / "executed_commands.npz"),
                      cuda_graph=False, graph_steps=0, record_stride=args.record_stride,
                      unrecorded_warmup_steps=0, native_cpu_workers=worker_count,
                      setup_wall_s=time.perf_counter() - started)
        _write(output / "summary.json", report)
        code_files = list(Path(__file__).parent.glob("newton*.py"))
        code_files.append(Path(__file__).with_name("run_newton_harvest.py"))
        _write(output / "invocation.json", dict(python=sys.executable, argv=sys.argv,
            code_sha256={f.name: sha256(f) for f in code_files if f.is_file()}))
        if getattr(args, "ready_file", None) is not None:
            _write(args.ready_file, dict(pid=__import__('os').getpid(), backend='cpu', worlds=1, physics_steps=0))
        if getattr(args, "start_gate", None) is not None:
            deadline=time.monotonic()+args.start_gate_timeout
            while not args.start_gate.exists():
                if time.monotonic()>=deadline:
                    raise TimeoutError('Newton CPU start gate timed out before any physics')
                time.sleep(.02)
        loop_started_monotonic_s=time.monotonic()
        loop = last_progress = time.perf_counter()
        actual = 0
        frequency_events, steps_by_hz = [], {}
        previous_hz = None
        for index in range(steps):
            iteration_started = time.perf_counter()
            current["command"] = timeline["command"][index]
            current["belt"] = timeline["belt_surface_velocity_local_y_m_s"][index]
            phase = str(timeline["phase"][index])
            attached_at_start = data.eq_active[attachments.eqs].copy()
            solver_time = float(data.time)
            dt = float(step_dt[index])
            frequency = int(round(1. / dt))
            steps_by_hz[str(frequency)] = steps_by_hz.get(str(frequency), 0) + 1
            if frequency != previous_hz:
                frequency_events.append(dict(t=solver_time, phase=phase, hz=frequency, dt=dt,
                                             solver_step=index))
                previous_hz = frequency
            before_calls = current["callback_calls"]
            timers_before={i:data.timer[i].duration for i in native_timer_names} if profile_timers else {}
            physics_started = time.perf_counter()
            engine.step(dt)
            step_wall = time.perf_counter() - physics_started
            physics_wall += step_wall
            solver_iterations[index] = int(data.solver_niter.max(initial=0))
            constraint_counts[index] = data.nefc
            contact_counts[index] = data.ncon
            if current["callback_calls"] != before_calls + 1:
                raise RuntimeError("Expected exactly one native control callback per fixed physics step")
            measured_force, events = attachments.measure_breaks(data, current["torque"], phase)
            force_log[0, index] = measured_force
            torque_log[0, index] = current["torque"]
            for event in events:
                fruit = next(i for i, spec in enumerate(metadata["attachments"]) if spec["name"] == event["name"])
                break_steps[0, fruit] = index
                event.update(world=0, solver_step=index, solver_time_s=solver_time,
                             release_effective_time_s=float(data.time))
                breaks.append(event)
            for contact_id, contact in enumerate(data.contact):
                if contact.efc_address < 0:
                    continue
                geom_ids = [int(contact.geom1), int(contact.geom2)]
                bodies = model.geom_bodyid[geom_ids]
                is_released = any(body in attachments.ids and not attached_at_start[
                    np.flatnonzero(attachments.ids == body)[0]] for body in bodies)
                allowed = bool(is_released and Session.released_contact_allowed(
                    policy_context, bodies, geom_ids, attached_at_start))
                penetration = max(0., -float(contact.dist))
                wrench = np.zeros(6)
                mj.mj_contactForce(model, data, contact_id, wrench)
                magnitude = float(np.linalg.norm(wrench[:3]))
                force[0, index] = max(force[0, index], magnitude)
                if is_released:
                    released[0, index] = max(released[0, index], penetration)
                if not allowed:
                    active[0, index] = max(active[0, index], penetration)
                    if penetration > .001 and first_violation is None:
                        first_violation = dict(t=solver_time, phase=phase,
                            bodies=[model.body(int(body)).name for body in bodies],
                            geoms=[model.geom(geom).name for geom in geom_ids], geom_ids=geom_ids,
                            solver_step=index, penetration_m=penetration)
                    if penetration > .010:
                        stop = "gross_active_overlap_10mm"
                    elif penetration > .001 and not args.diagnostic_continue:
                        stop = "active_contact_penetration_1mm"
                if penetration > .001 or magnitude > 1.:
                    contacts.append(dict(solver_time_s=solver_time, phase=phase,
                        geoms=geom_ids, bodies=[model.body(int(body)).name for body in bodies],
                        penetration_m=penetration, force_n=magnitude,
                        released_fruit=bool(is_released), released_contact_exception=allowed))
            if not np.isfinite(data.qacc).all():
                qacc_nonfinite_step = min(qacc_nonfinite_step, index + 1)
            if not native_state_finite(data):
                nonfinite_step = min(nonfinite_step, index + 1)
            if nonfinite_step != np.iinfo(np.int32).max or any(w.number for w in data.warning):
                stop = "solver_instability"
            actual = index + 1
            if actual % args.record_stride == 0 or stop or actual == steps:
                record(phase, current["command"], current["belt"])
            if publisher is not None and publisher.due():
                if viewer is not None and not viewer.is_alive():
                    record_display_error(report, 'live_run', 'viewer exited during physics',
                                         simulated_s=data.time, viewer=viewer)
                    publisher.close('viewer_exited')
                    viewer.close()
                    publisher = viewer = None
                elif native_state_finite(data):
                    pose, gap = live_frame()
                    publisher.publish(pose, gap, float(data.time))
            performance=phase_performance.setdefault(phase,dict(steps=0, simulated_s=0., loop_wall_s=0., physics_step_wall_s=0., max_solver_iterations=0, sum_solver_iterations=0))
            performance['steps']+=1
            performance['simulated_s']+=dt
            performance['loop_wall_s']+=time.perf_counter()-iteration_started
            performance['physics_step_wall_s']+=step_wall
            performance['max_solver_iterations']=max(performance['max_solver_iterations'],int(solver_iterations[index]))
            performance['sum_solver_iterations']+=int(solver_iterations[index])
            if profile_timers:
                timer_row=performance.setdefault('native_timer_ms',{})
                for i,name in native_timer_names.items():
                    timer_row[name]=timer_row.get(name,0.)+data.timer[i].duration-timers_before[i]
            if time.perf_counter() - last_progress >= 5 or stop:
                elapsed = time.perf_counter() - loop
                print(f"Newton CPU | {data.time:.2f}/{timeline['time_edges'][steps]:.2f}s | RTF {data.time / elapsed:.3f} | stop={stop}", flush=True)
                resources.append(dict(wall_s=elapsed, simulated_s=float(data.time),
                                      rss_mib=psutil.Process().memory_info().rss / 2**20))
                last_progress = time.perf_counter()
            if stop:
                break
        loop_wall = time.perf_counter() - loop
        loop_finished_monotonic_s=time.monotonic()
        if publisher is not None:
            try:
                pose, gap = live_frame()
                publisher.publish(pose, gap, float(data.time), force=True)
                publisher.close("stopped" if stop else "completed")
                # Acknowledge the final displayed measured state outside the
                # physical-loop timer, then close the independent viewer.
                acknowledgement = viewer.wait_displayed(float(data.time), timeout=20.)
                report.update(gui_status='final_frame_displayed', gui_live_verified=True,
                              final_display_ack=acknowledgement)
            except Exception as error:
                record_display_error(report, 'final_frame', error,
                                     simulated_s=data.time, viewer=viewer)
            finally:
                publisher.close('display_finished')
                report['viewer_process'] = viewer.close()
                viewer = publisher = None
        q, velocity, eq = np.asarray(q_rows), np.asarray(v_rows), np.asarray(e_rows)
        np.savez_compressed(output / "states.npz", time=np.asarray(times), qpos=q, qvel=velocity,
                            eq_active=eq, poses=np.asarray(pose_rows), robot_q=q[:, qadr],
                            body_paths=np.asarray([next((path for path, name in provenance["source_body_to_native_name"].items()
                                if name == body), body) for body in native_names[1:]]),
                            phase=np.asarray(phases), command=np.asarray(commands),
                            belt_surface_velocity_local_y_m_s=-np.asarray(belts))
        first_event = np.full(1, np.iinfo(np.uint64).max, dtype=np.uint64)
        if first_violation:
            g0, g1 = sorted(first_violation["geom_ids"])
            first_event[0] = np.uint64((first_violation["solver_step"] << 32) | (g0 * 65536 + g1))
        np.savez_compressed(output / "physics_metrics.npz", active_penetration_m=active[:, :actual],
                            released_penetration_m=released[:, :actual], max_contact_force_n=force[:, :actual],
                            attachment_force_n=force_log[:, :actual], attachment_torque_nm=torque_log[:, :actual],
                            break_step=break_steps, first_event=first_event,
                            nonfinite_step=np.asarray([nonfinite_step], dtype=np.int32),
                            qacc_nonfinite_step=np.asarray([qacc_nonfinite_step], dtype=np.int32),
                            solver_iterations=solver_iterations[:actual], nefc=constraint_counts[:actual], ncon=contact_counts[:actual])
        _write(output / "break_events.json", breaks)
        display_evidence = write_display_evidence(output, metadata, provenance, breaks)
        _write(output / "contact_events.json", contacts)
        _write(output / "resources.json", resources)
        _write(output / "frequency_events.json", frequency_events)
        for row in phase_performance.values():
            row['realtime_factor']=row['simulated_s']/max(row['loop_wall_s'],1e-12)
            row['mean_solver_iterations']=row.pop('sum_solver_iterations')/max(row['steps'],1)
        _write(output/'phase_performance.json',phase_performance)
        report.update(status="stopped" if stop else "completed", stop=stop,
                      simulated_s=float(data.time), actual_steps=actual, loop_wall_s=loop_wall,
                      loop_started_monotonic_s=loop_started_monotonic_s,
                      loop_finished_monotonic_s=loop_finished_monotonic_s,
                      solver_capacity_valid=not any(w.number for w in data.warning),
                      physics_step_wall_s=physics_wall, realtime_factor=float(data.time) / loop_wall,
                      aggregate_sim_s_per_wall_s=float(data.time) / loop_wall,
                      max_active_penetration_m=float(active.max(initial=0)),
                      max_released_penetration_m=float(released.max(initial=0)),
                      max_contact_force_n=float(force.max(initial=0)), breaks=breaks,
                      active_contact_criterion_passed=not np.any(active > .001),
                      finite_recorded_state=bool(np.isfinite(q).all() and np.isfinite(velocity).all()),
                      qacc_finite=qacc_nonfinite_step == np.iinfo(np.int32).max,
                      states_sha256=sha256(output / "states.npz"), final_time_per_world_s=[float(data.time)],
                      body_paths_mapped=bool(len(provenance["source_body_to_native_name"]) == model.nbody - 1),
                      detached_per_world=[int((break_steps >= 0).sum())], first_violation=first_violation,
                      source_model_mjb=str(inputs / "input_model.mjb"), model_dir=str(inputs),
                      native_control_callback_calls=current["callback_calls"],
                      steps_by_hz=steps_by_hz, frequency_events=frequency_events,
                      phase_performance=phase_performance,
                      profile_timers_enabled=profile_timers, native_timer_units="milliseconds from explicit perf_counter callback" if profile_timers else None,
                      solver_settings=dict(iterations=int(model.opt.iterations),tolerance=float(model.opt.tolerance),integrator=int(model.opt.integrator),solver=int(model.opt.solver),jacobian=int(model.opt.jacobian)),
                      limitations=["Newton native MuJoCo CPU backend; no claim of GPU execution.",
                                   "GUI displays live measured states in an independent Isaac process.",
                                   "Existing fixed commands and physical attachment hypothesis are retained.",
                                   "Released-fruit exceptions reuse the existing Session contact classification."])
        report.update(display_evidence)
        _write(output / "summary.json", report)
        return report
    except BaseException as exc:
        report.update(status="error", error=repr(exc), traceback=traceback.format_exc(),
                      total_wall_s=time.perf_counter() - started)
        _write(output / "summary.json", report)
        raise
    finally:
        mj.set_mjcb_control(previous_control)
        mj.set_mjcb_time(previous_timer)
        if publisher is not None:
            publisher.close("stopped")
        if viewer is not None:
            viewer.close()
