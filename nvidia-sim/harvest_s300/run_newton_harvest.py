#!/usr/bin/env python3
"""Fixed-step Newton execution of existing S300 commands, with live Isaac display.

Reuses the existing portable physical model, commands, attachment mechanics,
contact policy and latest-frame viewer. Physics never imports Kit. The same
Newton loop runs with and without the independent GUI consumer.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time
import traceback

import mujoco as mj
import mujoco_warp as mw
import newton
import numpy as np
import psutil
import warp as wp

from newton_inputs import sha256, command_timeline, write_display_evidence
from newton_kernels import HarvestGPUController, save_trace
from newton_mjcf import create_newton_mjcf


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def run(args):
    started = time.perf_counter()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    report = dict(schema='s300-newton-fixed-step-v1', status='initializing',
        physics_engine='Newton SolverMuJoCo', backend='gpu', newton_version=newton.__version__,
        warp_version=wp.__version__, mujoco_version=mj.__version__, worlds=args.worlds,
        gui_requested=args.gui, gui_advances_physics=False,
        gui_process_independent=True, input=str(args.inputs.resolve()),
        training_eligible=False, harvest_success=None)
    write(out/'summary.json', report)
    viewer = publisher = None
    try:
        p = args.inputs.resolve()
        provenance = json.loads((p/'inputs.json').read_text())
        for name, evidence in provenance['input_files'].items():
            expected = evidence.get('sha256')
            if expected and sha256(p/name) != expected:
                raise ValueError('Prepared input SHA mismatch: ' + name)
        initial = dict(np.load(p/'initial_state.npz', allow_pickle=False))
        timeline = dict(np.load(p/'commands.npz', allow_pickle=False))
        metadata = json.loads((p/'source_export.json').read_text())
        hz = 1. / float(timeline['time_edges'][1] - timeline['time_edges'][0])
        if (args.hz is not None and abs(hz-args.hz)>1e-8) or args.free_hz is not None:
            initial_command = initial['qpos'][initial['robot_qpos_addresses']]
            sampling_meta = dict(metadata, prepared_source_plan=str(p/'sampled_plan.json'))
            hz = args.hz or hz
            timeline, _, _ = command_timeline(sampling_meta, initial_command, hz, command_source='plan', free_hz=args.free_hz)
        np.savez_compressed(out/'executed_commands.npz', **timeline)
        edges = timeline['time_edges']
        timesteps = np.diff(edges)
        steps = len(timeline['time'])
        if args.seconds is not None:
            steps = min(steps, int(np.searchsorted(edges, args.seconds+1e-9, side='right')-1))
        if steps <= 0:
            raise ValueError('At least one physics step required')
        if args.batch < 1 or args.batch % 2:
            raise ValueError('CUDA graph batch must be a positive even number')
        # The original state and runtime damping were prepared by the existing
        # adapter without advancing physical time. Nothing is restored from an
        # already completed harvest trial.
        source = mj.MjModel.from_binary_path(str(p/'input_model.mjb'))
        source.opt.timestep = 1. / hz
        if args.solver_iterations is not None:
            if args.solver_iterations<1:raise ValueError('Positive solver iterations required')
            source.opt.iterations=args.solver_iterations
        if args.solver_tolerance is not None:
            if args.solver_tolerance<=0:raise ValueError('Positive solver tolerance required')
            source.opt.tolerance=args.solver_tolerance
        if args.solver is not None:source.opt.solver=getattr(mj.mjtSolver,'mjSOL_'+args.solver.upper())
        if args.jacobian is not None:source.opt.jacobian=getattr(mj.mjtJacobian,'mjJAC_'+args.jacobian.upper())
        source.flg_surfacevel = True
        source.dof_damping[:] = initial['runtime_dof_damping']
        source.geom_surfacevel[:] = 0
        engine = create_newton_mjcf(p/'input_scene.xml', source_model=source,
            device=args.device, world_count=args.worlds, nconmax=args.nconmax,
            njmax=args.njmax, native_observables_only=args.native_observables_only)
        solver, d = engine.solver, engine.solver.mjw_data
        d.qpos.assign(np.tile(initial['qpos'], (args.worlds, 1)).astype(np.float32))
        d.qvel.assign(np.tile(initial['qvel'], (args.worlds, 1)).astype(np.float32))
        d.eq_active.assign(np.tile(initial['eq_active'], (args.worlds, 1)))
        d.ctrl.assign(np.tile(initial['ctrl'], (args.worlds, 1)).astype(np.float32))
        controller_initial = dict(initial, preload=initial['plant_preload'])
        # Pad commands only for the unexecuted end of a captured graph. The
        # runner executes the remainder one real step at a time.
        ctrl = HarvestGPUController(solver, p, controller_initial, metadata,
            timeline['command'][:steps], timeline['belt_surface_velocity_local_y_m_s'][:steps],
            steps, worlds=args.worlds)
        solver.mjw_model.callback.control = ctrl.control
        if not solver.mjw_model.flg_surfacevel:
            raise RuntimeError('Native belt surface velocity feature disabled')
        mw.kinematics(solver.mjw_model, d)
        stride = args.record_stride
        rows = steps // stride + 1
        qlog = wp.zeros((rows, source.nq), float, device=args.device)
        vlog = wp.zeros((rows, source.nv), float, device=args.device)
        elog = wp.zeros((rows, source.neq), bool, device=args.device)
        plog = wp.zeros((rows, source.nbody), wp.transform, device=args.device)
        nonfinite = wp.array(np.full(args.worlds, np.iinfo(np.int32).max, np.int32), dtype=int, device=args.device)
        def record():
            wp.launch(save_trace, (args.worlds, max(source.nq, source.nv, source.nbody)),
                inputs=[ctrl.tick, stride, d.qpos, d.qvel, d.eq_active, d.xpos, d.xquat,
                        qlog, vlog, elog, plog, nonfinite], device=args.device)
        record()
        def step(dt):
            engine.step(dt)
            ctrl.post_step()
            record()
        # Capture does not execute the graph. Kernel compilation belongs to
        # setup, and there are zero unrecorded integrated warmup steps.
        graphs = {}
        durations = np.round(timesteps[:steps], 12)
        if not args.no_graph:
            print('Capturing fixed Newton physics graph...', flush=True)
            for dt in np.unique(durations):
                with wp.ScopedCapture(device=args.device) as capture:
                    for _ in range(args.batch):
                        step(float(dt))
                graphs[float(dt)] = capture.graph
        wp.synchronize()
        if int(ctrl.tick.numpy()[0]) != 0:
            raise RuntimeError('Graph capture advanced unrecorded physical time')
        native_names = [source.body(i).name or 'world' for i in range(source.nbody)]
        qadr = initial['robot_qpos_addresses']
        def live_frame():
            pos = d.xpos.numpy()[0]
            quat = d.xquat.numpy()[0][:, [1, 2, 3, 0]]
            q = d.qpos.numpy()[0]
            return np.c_[pos, quat], float(.044 + q[qadr[9]] - q[qadr[8]])
        if args.gui:
            from newton_live import NewtonLivePublisher, NewtonViewerProcess
            publisher = NewtonLivePublisher(out, metadata['source_assembly'], native_names,
                provenance['source_body_to_native_name'], fps=args.viewer_fps,
                physical_model_sha256=sha256(p/'input_model.mjb'))
            pose, gap = live_frame()
            publisher.publish(pose, gap, 0., force=True)
            viewer = NewtonViewerProcess(out, fps=args.viewer_fps, seconds=args.viewer_seconds, camera=args.camera, capture=out/'live_final.png')
            viewer.wait_ready(timeout=args.viewer_timeout)
        report.update(status='running', physics_hz=hz, requested_steps=steps,
            source_model_counts=provenance['model_counts'], input_sha256=sha256(p/'inputs.json'),
            commands_sha256=sha256(p/'commands.npz'), engine=engine.evidence,
            executed_commands_sha256=sha256(out/'executed_commands.npz'),
            cuda_graph=bool(graphs), graph_steps=args.batch if graphs else 0,
            free_hz=args.free_hz, timestep_counts={str(dt):int(np.sum(durations==dt)) for dt in np.unique(durations)},
            record_stride=stride, unrecorded_warmup_steps=0, setup_wall_s=time.perf_counter()-started)
        write(out/'summary.json', report)
        write(out/'invocation.json', dict(python=sys.executable, argv=sys.argv,
            code_sha256={f.name:sha256(f) for f in [Path(__file__), *Path(__file__).parent.glob('newton*.py')]}))
        actual = 0
        stop = None
        resource = []
        display_errors = []
        loop = time.perf_counter()
        gpu_started = loop
        last_progress = loop
        while actual < steps:
            dt = float(durations[actual])
            batch = min(args.batch if graphs else 1, steps-actual)
            if batch > 1:
                different = np.flatnonzero(durations[actual:actual+batch] != dt)
                if len(different):
                    batch = int(different[0])
            if graphs and batch == args.batch:
                wp.capture_launch(graphs[dt])
            else:
                for _ in range(batch):
                    step(dt)
            actual += batch
            wp.synchronize()
            invalid = min(int(nonfinite.numpy().min()), int(ctrl.first_qacc_nonfinite.numpy().min()))
            capacity = int(ctrl.first_capacity_overflow.numpy().min())
            first = ctrl.first_event.numpy()
            if invalid != np.iinfo(np.int32).max:
                stop = 'nonfinite_physics'
            elif capacity != np.iinfo(np.int32).max:
                stop = 'solver_capacity_overflow'
            elif (first != np.iinfo(np.uint64).max).any() and not args.diagnostic_continue:
                stop = 'active_contact_penetration_1mm'
            if viewer is not None and not viewer.is_alive() and not display_errors:
                display_errors.append('Independent live viewer exited during physics; producer continues unchanged')
            if publisher is not None and publisher.due():
                pose, gap = live_frame()
                publisher.publish(pose, gap, float(d.time.numpy()[0]))
            if time.perf_counter()-last_progress >= 5 or stop:
                elapsed = time.perf_counter()-loop
                print(f'Newton {args.worlds} env | {float(edges[actual]):.2f}/{float(edges[steps]):.2f}s | '
                      f'RTF {float(edges[actual])/elapsed:.3f} | stop={stop}', flush=True)
                resource.append(dict(wall_s=elapsed, simulated_s=float(edges[actual]),
                    rss_mib=psutil.Process().memory_info().rss/2**20))
                last_progress = time.perf_counter()
            if stop:
                break
        wp.synchronize()
        loop_s = time.perf_counter()-loop
        simulated_s = float(d.time.numpy()[0])
        if publisher is not None:
            pose, gap = live_frame()
            publisher.publish(pose, gap, simulated_s, force=True)
            publisher.close(stop)
            try:
                viewer.wait_displayed(simulated_s, timeout=20.)
            except (RuntimeError, TimeoutError, ValueError) as exc:
                display_errors.append(repr(exc))
            viewer.close()
            viewer = publisher = None
        count = actual//stride + 1
        q = qlog.numpy()[:count]
        vel = vlog.numpy()[:count]
        eq = elog.numpy()[:count]
        pose = plog.numpy()[:count]
        actual_times = np.concatenate(([0.], ctrl.time_log.numpy()[0, stride-1:actual:stride]))
        np.savez_compressed(out/'states.npz', time=actual_times,
            scheduled_time=edges[np.arange(count)*stride], solver_step=np.arange(count)*stride,
            qpos=q, qvel=vel, eq_active=eq, poses=pose[:, 1:],
            body_paths=np.array([next((path for path,name in provenance['source_body_to_native_name'].items()
                if name==body), body) for body in native_names[1:]]),
            robot_q=q[:, qadr], phase=np.concatenate((np.array(['initial']), timeline['phase'][stride-1:actual:stride])),
            command=np.vstack([initial['qpos'][qadr], timeline['command'][stride-1:actual:stride]]),
            belt_surface_velocity_local_y_m_s=np.vstack([np.zeros(2), -timeline['belt_surface_velocity_local_y_m_s'][stride-1:actual:stride]]))
        active = ctrl.active_max.numpy()[:, :actual]
        released = ctrl.released_max.numpy()[:, :actual]
        force = ctrl.force_max.numpy()[:, :actual]
        break_step = ctrl.break_step.numpy()
        f = ctrl.force_log.numpy()[:, :actual]
        torque = ctrl.torque_log.numpy()[:, :actual]
        np.savez_compressed(out/'physics_metrics.npz', active_penetration_m=active,
            released_penetration_m=released, max_contact_force_n=force,
            attachment_force_n=f, attachment_torque_nm=torque, break_step=break_step,
            first_event=ctrl.first_event.numpy(), nonfinite_step=nonfinite.numpy(),
            qacc_nonfinite_step=ctrl.first_qacc_nonfinite.numpy(),
            capacity_overflow_step=ctrl.first_capacity_overflow.numpy(),
            nacon=ctrl.nacon_log.numpy()[:actual], ncollision=ctrl.ncollision_log.numpy()[:actual],
            nefc=ctrl.nefc_log.numpy()[:,:actual], overflow=ctrl.overflow_log.numpy()[:,:actual],
            qacc_abs_max=ctrl.qacc_abs_max.numpy()[:,:actual], time=ctrl.time_log.numpy()[:,:actual])
        backend_times=ctrl.time_log.numpy()[:, :actual]
        breaks = []
        for w,i in np.argwhere(break_step>=0):
            tick = int(break_step[w,i])
            breaks.append(dict(world=int(w), name=metadata['attachments'][i]['name'],
                solver_step=tick, solver_time_s=0. if tick==0 else float(backend_times[w,tick-1]),
                release_effective_time_s=float(backend_times[w,tick]),
                scheduled_solver_time_s=float(edges[tick]), scheduled_release_time_s=float(edges[tick+1]),
                phase=str(timeline['phase'][tick]), constraint_force_n=float(f[w,tick,i]),
                passive_torque_nm=float(torque[w,tick,i])))
        np.savez_compressed(out/'final_states.npz', time=d.time.numpy(), qpos=d.qpos.numpy(), qvel=d.qvel.numpy(), eq_active=d.eq_active.numpy())
        write(out/'break_events.json', breaks)
        write(out/'resources.json', resource)
        report.update(write_display_evidence(out, metadata, provenance, breaks))
        report.update(status='stopped' if stop else 'completed', stop=stop,
            simulated_s=simulated_s, scheduled_simulated_s=float(edges[actual]), actual_steps=actual, loop_wall_s=loop_s,
            realtime_factor=simulated_s/loop_s, aggregate_sim_s_per_wall_s=args.worlds*simulated_s/loop_s,
            display_errors=display_errors, display_health_passed=not display_errors,
            max_active_penetration_m=float(active.max(initial=0)),
            max_released_penetration_m=float(released.max(initial=0)),
            max_contact_force_n=float(force.max(initial=0)), breaks=breaks,
            active_contact_criterion_passed=not np.any(active>.001),
            finite_recorded_state=bool(np.isfinite(q).all() and np.isfinite(vel).all()),
            solver_capacity_valid=bool(np.all(ctrl.first_capacity_overflow.numpy()==np.iinfo(np.int32).max)),
            qacc_finite=bool(np.all(ctrl.first_qacc_nonfinite.numpy()==np.iinfo(np.int32).max)),
            max_constraints=int(ctrl.nefc_log.numpy().max(initial=0)),
            max_total_contacts=int(ctrl.nacon_log.numpy().max(initial=0)),
            gpu_timestamp_drift_s=float(actual_times[-1]-edges[actual]),
            states_sha256=sha256(out/'states.npz'), final_time_per_world_s=d.time.numpy().tolist(),
            body_paths_mapped=bool(len(provenance['source_body_to_native_name'])==source.nbody-1),
            detached_per_world=(break_step>=0).sum(axis=1).tolist(),
            final_world_qpos_spread_abs=float(np.max(np.abs(d.qpos.numpy()-d.qpos.numpy()[0]))),
            final_states_sha256=sha256(out/'final_states.npz'),
            source_model_mjb=str(p/'input_model.mjb'), model_dir=str(p),
            limitations=['Uses the existing documented MuJoCo physical adaptation through Newton SolverMuJoCo.',
                'Full source visual geometry is retained in the isolated Isaac display; display is a live state consumer.',
                'Fixed recorded/plan commands benchmark the executor; adaptive route planning is not benchmarked.',
                'Native GPU data is authoritative; Newton metadata-only collision queries are not used.',
                'Dropped-fruit exceptions follow the existing diagnostic scope; collection remains unverified.'])
        write(out/'summary.json', report)
        return report
    except BaseException as exc:
        report.update(status='error', error=repr(exc), traceback=traceback.format_exc(), total_wall_s=time.perf_counter()-started)
        write(out/'summary.json', report)
        raise
    finally:
        if publisher is not None:
            publisher.close('stopped')
        if viewer is not None:
            viewer.close()


def arguments(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('inputs', type=Path); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--worlds',type=int,default=1); p.add_argument('--device',default='cuda:0')
    p.add_argument('--backend',choices=('gpu','cpu'),default='cpu')
    p.add_argument('--hz',type=float,help='Explicit frequency comparison, reuses the existing saved plan sampler')
    p.add_argument('--free-hz',type=float,help='Existing free-transfer frequency; contact phases keep --hz')
    p.add_argument('--threads',type=int,default=0,help='Native CPU threadpool workers')
    p.add_argument('--native-observables-only',action=argparse.BooleanOptionalAction,default=False,help='Skip unused Newton metadata refresh; actual native state remains authoritative (CPU only)')
    p.add_argument('--jacobian',choices=('sparse','dense'),help='Explicit native constraint storage comparison')
    p.add_argument('--profile-timers',action='store_true',help='Diagnostic native C timers; not a clean throughput benchmark')
    p.add_argument('--solver-tolerance',type=float,help='Explicit convergence tolerance comparison')
    p.add_argument('--solver',choices=('Newton','CG'),help='Explicit native solver diagnostic; original default unchanged')
    p.add_argument('--solver-iterations',type=int,help='Explicit solver budget diagnostic, original default unchanged')
    p.add_argument('--start-gate',type=Path);p.add_argument('--ready-file',type=Path)
    p.add_argument('--start-gate-timeout',type=float,default=300.)
    p.add_argument('--seconds',type=float); p.add_argument('--batch',type=int,default=10)
    p.add_argument('--record-stride',type=int,default=1)
    p.add_argument('--nconmax',type=int,default=512); p.add_argument('--njmax',type=int,default=2048)
    p.add_argument('--no-graph',action='store_true');p.add_argument('--diagnostic-continue',action='store_true')
    p.add_argument('--gui',action='store_true');p.add_argument('--viewer-fps',type=float,default=15.)
    p.add_argument('--viewer-timeout',type=float,default=300.);p.add_argument('--viewer-seconds',type=float,default=600.)
    p.add_argument('--camera',default='entry')
    args=p.parse_args(argv)
    if args.worlds<1 or args.record_stride<1:p.error('positive worlds and record stride required')
    if args.backend=='gpu' and args.record_stride!=1:p.error('GPU evidence currently requires record stride 1')
    cpu_only=('solver','solver_iterations','solver_tolerance','jacobian','profile_timers',
              'threads','native_observables_only','start_gate','ready_file')
    if args.backend=='gpu' and any(getattr(args,key) for key in cpu_only):
        p.error('CPU-only diagnostics/gates cannot be applied to the GPU backend')
    return args


def main():
    args=arguments()
    if args.backend=='cpu':
        from newton_cpu import run_cpu
        result=run_cpu(args)
    else:
        result=run(args)
    print(json.dumps({key:result.get(key) for key in ('status','stop','backend','worlds','simulated_s','loop_wall_s','realtime_factor','aggregate_sim_s_per_wall_s','detached_per_world','max_active_penetration_m')},indent=2))
    print('Saved:', args.output.resolve(), flush=True)


if __name__=='__main__':main()
