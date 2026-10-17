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
import signal
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
from newton_kernels import HarvestGPUController, save_trace, pack_runtime_status
from newton_mjcf import create_newton_mjcf
from newton_continuation import (model_identity, load_resume, verify_command_start,
                                 cumulative_breaks)
from newton_gpu_options import solver_tolerance_value, apply_explicit_solver_tolerance
from newton_prepared_assets import verify_manifest_files

# A persistent process executes the code loaded at startup. Do not attribute a
# later repeat to source files edited while that process was already running.
PROCESS_CODE_SHA256={f.name:sha256(f) for f in
    [Path(__file__),*Path(__file__).parent.glob('newton*.py')]}


def write(path, value):
    def safe(item):
        if isinstance(item, (float,np.floating)) and not np.isfinite(item):
            return str(item)
        if isinstance(item, dict):
            return {key:safe(v) for key,v in item.items()}
        if isinstance(item, (list,tuple)):
            return [safe(v) for v in item]
        return item
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False) + '\n')


def verify_prepared_inputs(inputs, provenance, local_model_cache=None, *, report_progress=False):
    """Verify all source bytes; cache staging owns the model's one SHA read."""
    inputs = Path(inputs)
    files = provenance['input_files']
    if 'input_model.mjb' not in files:
        raise ValueError('The authoritative physical MJB must have an input SHA')
    staged_model, cache_evidence = inputs / 'input_model.mjb', None
    pending = {name: evidence for name, evidence in files.items()
               if local_model_cache is None or name != 'input_model.mjb'}
    started = time.perf_counter()
    if report_progress:
        print(f'Verifying {len(pending)} prepared input SHA256 values with 8 readers...', flush=True)
    verified = verify_manifest_files(inputs, pending)
    if report_progress:
        print(f'Prepared input SHA256 check finished in {time.perf_counter() - started:.2f}s.', flush=True)
    if local_model_cache is not None:
        from newton_asset_cache import verify_or_stage_model
        if report_progress:
            print('Verifying source MJB and local model cache...', flush=True)
        staged_model, cache_evidence = verify_or_stage_model(
            inputs / 'input_model.mjb', files['input_model.mjb'].get('sha256'), local_model_cache)
        verified['input_model.mjb'] = cache_evidence['source_sha256']
        if report_progress:
            print(f'Model cache verified in {cache_evidence["total_wall_s"]:.2f}s '
                  f'(hit={cache_evidence["cache_hit"]}, '
                  f'source read/copy={cache_evidence["source_hash_and_optional_copy_wall_s"]:.2f}s, '
                  f'cache read={cache_evidence["cache_hash_wall_s"]:.2f}s).', flush=True)
    return staged_model, cache_evidence, {name: verified[name] for name in files}


def run(args, *, session_cache=None):
    started = time.perf_counter()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    write(out/'invocation.json', dict(python=sys.executable, argv=sys.argv,
        code_sha256=PROCESS_CODE_SHA256))
    report = dict(schema='s300-newton-fixed-step-v1', status='initializing',
        physics_engine='Newton SolverMuJoCo', backend='gpu', newton_version=newton.__version__,
        warp_version=wp.__version__, mujoco_version=mj.__version__, worlds=args.worlds,
        gui_requested=args.gui, gui_advances_physics=False,
        gui_process_independent=True, input=str(args.inputs.resolve()),
        training_eligible=False, harvest_success=None)
    write(out/'summary.json', report)
    viewer = publisher = linear_backend = deterministic_adapter = inertia_backend = None
    tree_backend = contact_order = None
    hessian_backend = fixed_reductions = None
    event_profile = None
    constraint_order = None
    previous_interrupt_handler = None
    try:
        p = args.inputs.resolve()
        provenance = json.loads((p/'inputs.json').read_text())
        verification_started = time.perf_counter()
        staged_model, model_cache_evidence, verified_input_hashes = verify_prepared_inputs(
            p, provenance, getattr(args, 'local_model_cache', None), report_progress=True)
        report['input_verification_wall_s'] = time.perf_counter() - verification_started
        initial = dict(np.load(p/'initial_state.npz', allow_pickle=False))
        prepared_runtime_dof_damping = initial['runtime_dof_damping'].copy()
        timeline = dict(np.load(p/'commands.npz', allow_pickle=False))
        sampling_description = dict(provenance.get('timeline', {}))
        metadata = json.loads((p/'source_export.json').read_text())
        metadata = dict(metadata, released_gutter_diagnostic=bool(getattr(args, 'released_gutter_diagnostic', False)))
        continuation_identity = model_identity(initial, metadata)
        resume_state = None
        inherited_breaks = []
        continuation = None
        clock_offset = 0.
        if getattr(args, 'resume_state', None) is not None:
            if session_cache is not None:
                raise ValueError('Measured continuation cannot use the reset-to-initial repeat cache')
            from newton_contact_policy import validate_config
            policy = validate_config(json.loads(args.contact_policy.read_text())) if args.contact_policy is not None else None
            initial, resume_state, inherited_breaks, continuation = load_resume(
                args.resume_state, p, initial, metadata, worlds=args.worlds, policy=policy)
            clock_offset = continuation['initial_clock_s']
        elif getattr(args, 'window_transition', None) is not None:
            if session_cache is not None:
                raise ValueError('Window transition cannot use the reset-to-initial repeat cache')
            from newton_window_continuation import load_transition
            from newton_contact_policy import validate_config
            policy = validate_config(json.loads(args.contact_policy.read_text())) if args.contact_policy is not None else None
            initial, resume_state, inherited_breaks, continuation = load_transition(
                args.window_transition, p, initial, metadata, policy=policy)
            clock_offset = continuation['initial_clock_s']
        elif metadata.get('active_window', {}).get('initial_clock_s', 0) > 0:
            raise ValueError('Measured active-window input requires its explicit --window-transition certificate')
        hz = 1. / float(timeline['time_edges'][1] - timeline['time_edges'][0])
        if (args.hz is not None and abs(hz-args.hz)>1e-8) or args.free_hz is not None:
            initial_command = initial['qpos'][initial['robot_qpos_addresses']]
            sampling_meta = dict(metadata, prepared_source_plan=str(p/'sampled_plan.json'))
            hz = args.hz or hz
            timeline, sampling_description, _ = command_timeline(
                sampling_meta, initial_command, hz, command_source='plan', free_hz=args.free_hz)
        belt_hold_frequency = sampling_description.get('belt_hold_frequency', {})
        if args.belt_hold_hz is not None:
            from newton_belt_hold import resample_belt_hold
            if sampling_description.get('command_source', 'plan') != 'plan' or sampling_description.get('freeze_phase') is not None:
                raise ValueError('--belt-hold-hz requires the continuous plan without freeze_phase')
            hz = float(sampling_description.get('hz', hz))
            timeline, belt_hold_frequency = resample_belt_hold(timeline,
                json.loads((p/'sampled_plan.json').read_text()), args.belt_hold_hz, hz,
                source_start_s=float(sampling_description.get('source_start_s', 0.)))
        adaptive = None
        if args.adaptive_hz:
            from newton_adaptive_frequency import AdaptiveSchedule
            sampling_meta = dict(metadata, prepared_source_plan=str(p/'sampled_plan.json'))
            initial_command = initial['qpos'][initial['robot_qpos_addresses']]
            adaptive = AdaptiveSchedule(sampling_meta,initial_command,hz,args.free_hz or 60,
                                        seconds=args.seconds)
            timeline = adaptive.timeline
        if continuation is not None:
            continuation['new_command_start_max_abs_error'] = verify_command_start(timeline, initial)
        np.savez_compressed(out/'executed_commands.npz', **timeline)
        edges = timeline['time_edges']
        timesteps = np.diff(edges)
        steps = len(timeline['time'])
        if args.seconds is not None and adaptive is None:
            steps = min(steps, int(np.searchsorted(edges, args.seconds+1e-9, side='right')-1))
        if steps <= 0:
            raise ValueError('At least one physics step required')
        if args.batch < 1 or args.batch % 2:
            raise ValueError('CUDA graph batch must be a positive even number')
        # The original state and runtime damping were prepared by the existing
        # adapter without advancing physical time. Nothing is restored from an
        # already completed harvest trial.
        def engine_factory():
            load_started = time.perf_counter()
            print(f'Loading verified MJB from {staged_model}...', flush=True)
            source = mj.MjModel.from_binary_path(str(staged_model))
            report['source_model_load_wall_s'] = time.perf_counter() - load_started
            print(f'MJB loaded in {report["source_model_load_wall_s"]:.2f}s '
                  f'(nv={source.nv}, ngeom={source.ngeom}). Creating Newton GPU solver...', flush=True)
            source.opt.timestep = 1. / hz
            if args.solver_iterations is not None:
                if args.solver_iterations<1:raise ValueError('Positive solver iterations required')
                source.opt.iterations=args.solver_iterations
            if args.solver_tolerance is not None:
                source.opt.tolerance=solver_tolerance_value(args.solver_tolerance)
            if args.solver is not None:source.opt.solver=getattr(mj.mjtSolver,'mjSOL_'+args.solver.upper())
            if args.jacobian is not None:source.opt.jacobian=getattr(mj.mjtJacobian,'mjJAC_'+args.jacobian.upper())
            if args.ls_iterations is not None:source.opt.ls_iterations=args.ls_iterations
            source.flg_surfacevel = True
            # put_model may fold actuator damping into its device damping.
            # Upload the original adapter model, then restore measured GPU
            # runtime damping directly after upload to avoid double folding.
            source.dof_damping[:] = prepared_runtime_dof_damping
            source.geom_surfacevel[:] = 0
            sleep_configuration = None
            if args.released_fruit_sleep:
                from newton_released_sleep import configure_fruit_sleep
                sleep_configuration = configure_fruit_sleep(source, metadata)
            result = create_newton_mjcf(p/'input_scene.xml', source_model=source,
                device=args.device, world_count=args.worlds, nconmax=args.nconmax,
                njmax=args.njmax, native_observables_only=args.native_observables_only,
                native_control_callback=args.native_control_callback,
                deterministic=getattr(wp.DeterministicMode, args.deterministic))
            if sleep_configuration is not None:
                result.evidence['fruit_sleep_configuration'] = sleep_configuration
            return result
        session_evidence={}
        if session_cache is not None:
            signature=dict(inputs_sha256=sha256(p/'inputs.json'),hz=hz,
                graph_timesteps_s=np.unique(np.round(timesteps[:steps], 12)).tolist(),
                options={key:getattr(args,key) for key in ('device','worlds','nconmax','njmax',
                    'solver','solver_iterations','solver_tolerance','jacobian','ls_iterations',
                    'native_observables_only','native_control_callback','deterministic','deterministic_scope',
                    'refresh_connect_velocity','cholesky_block_dim','mathdx_gemm',
                    'linear_algebra','broadphase','aabb_filter','fused_sparse_factor',
                    'inertia_linear_algebra','tree_sums','canonical_contacts','dense_hessian',
                    'fixed_solver_reductions', 'indexed_qfrc', 'cache_hessian', 'native_integer_atomics',
                    'canonical_constraints', 'experimental_multiworld_order', 'released_fruit_sleep',
                    'dense_hessian_capacity', 'dense_hessian_block_curvature', 'linesearch_block_dim',
                    'dense_hessian_overflow_capacity', 'cache_dense_jacobian',
                    'convex_ccd_block_dim', 'belt_hold_hz')})
            engine,session_evidence=session_cache.acquire(signature,engine_factory,initial)
        else:
            engine=engine_factory()
        source=engine.solver.mj_model
        solver, d = engine.solver, engine.solver.mjw_data
        from newton_frozen_constraints import configure_frozen_connects
        frozen_constraint_configuration = configure_frozen_connects(solver, metadata)
        # Native put_model applies a float32 tolerance floor. An explicit CLI
        # request is an intentional experiment, so honor it after upload. The
        # default None retains the existing uploaded value exactly.
        explicit_gpu_tolerance = apply_explicit_solver_tolerance(
            solver.mjw_model.opt, args.solver_tolerance)
        if args.deterministic_scope is not None:
            from newton_gpu_deterministic import BoundedDeterministicExecution
            deterministic_adapter=BoundedDeterministicExecution(solver,scope=args.deterministic_scope,
                native_integer_atomics=args.native_integer_atomics, allow_cg=args.solver=='CG')
        if args.fused_sparse_factor:
            from newton_gpu_sparse_factor import FusedSparseInertiaFactor
            inertia_backend=FusedSparseInertiaFactor(solver)
        if args.inertia_linear_algebra=='cusolver32':
            from newton_gpu_inertia_cusolver import CuSolverInertiaFactor
            inertia_backend=CuSolverInertiaFactor(solver)
        if args.tree_sums:
            from newton_gpu_tree_sums import FixedOrderTreeSums
            tree_backend=FixedOrderTreeSums(solver,mode=args.tree_sums)
        from mujoco_warp._src.types import BroadphaseType, BroadphaseFilter
        if args.broadphase is not None:
            solver.mjw_model.opt.broadphase=getattr(BroadphaseType,args.broadphase)
        if args.aabb_filter:
            solver.mjw_model.opt.broadphase_filter |= BroadphaseFilter.AABB
        if args.cholesky_block_dim is not None:
            solver.mjw_model.block_dim.update_gradient_cholesky_blocked=args.cholesky_block_dim
        if args.linesearch_block_dim is not None:
            solver.mjw_model.block_dim.linesearch_iterative=args.linesearch_block_dim
        convex_ccd_configuration = dict(
            pre_override_block_dim=int(solver.mjw_model.block_dim.convex_ccd),
            requested_block_dim=args.convex_ccd_block_dim,
            physical_parameters_changed=False, geometry_changed=False,
            ccd_iterations=int(solver.mjw_model.opt.ccd_iterations),
            ccd_tolerance=solver.mjw_model.opt.ccd_tolerance.numpy().tolist(),
            gpu_equivalence_verified=False)
        if args.convex_ccd_block_dim is not None:
            solver.mjw_model.block_dim.convex_ccd=args.convex_ccd_block_dim
        convex_ccd_configuration['effective_block_dim'] = int(solver.mjw_model.block_dim.convex_ccd)
        tuning = {}
        if args.mathdx_gemm is not None:
            from newton_gpu_tuning import configure_blocked_cholesky
            tuning = configure_blocked_cholesky(solver, mathdx_gemm=args.mathdx_gemm)
        compatibility = {}
        if args.refresh_connect_velocity:
            from newton_gpu_compat import install_connect_velocity_cache_refresh
            compatibility = install_connect_velocity_cache_refresh(solver.mjw_model,d)
        if args.canonical_contacts:
            from newton_gpu_deterministic import CanonicalContactOrder
            contact_order=CanonicalContactOrder(solver, allow_cg=args.solver=='CG')
        effective_gpu_options = {key: int(getattr(solver.mjw_model.opt,key)) for key in
            ('solver','integrator','iterations','ls_iterations','broadphase','broadphase_filter')}
        effective_gpu_options['tolerance'] = solver.mjw_model.opt.tolerance.numpy().tolist()
        effective_gpu_options['graph_conditional'] = bool(solver.mjw_model.opt.graph_conditional)
        effective_gpu_options['cholesky_block_dim'] = solver.mjw_model.block_dim.update_gradient_cholesky_blocked
        if args.linesearch_block_dim is not None:
            effective_gpu_options['linesearch_block_dim'] = solver.mjw_model.block_dim.linesearch_iterative
        if continuation is not None and continuation['parent_effective_gpu_options'] != effective_gpu_options:
            raise ValueError('Continuation runtime GPU solver options differ from the completed parent')
        d.qpos.assign(np.tile(initial['qpos'], (args.worlds, 1)).astype(np.float32))
        d.qvel.assign(np.tile(initial['qvel'], (args.worlds, 1)).astype(np.float32))
        d.eq_active.assign(np.tile(initial['eq_active'], (args.worlds, 1)))
        d.ctrl.assign(np.tile(initial['ctrl'], (args.worlds, 1)).astype(np.float32))
        if resume_state is not None:
            d.time.assign(resume_state['time'])
            solver.mjw_model.dof_damping.assign(resume_state['runtime_dof_damping'])
            if 'qacc_warmstart' in resume_state:
                d.qacc_warmstart.assign(resume_state['qacc_warmstart'])
        controller_initial = dict(initial, preload=initial['plant_preload'])
        # Commands are indexed by the GPU tick, including phase-boundary tails.
        ctrl = HarvestGPUController(solver, p, controller_initial, metadata,
            timeline['command'][:steps], timeline['belt_surface_velocity_local_y_m_s'][:steps],
            steps, worlds=args.worlds)
        sleep_audit = None
        if args.released_fruit_sleep:
            from newton_released_sleep import FruitSleepAudit
            sleep_audit = FruitSleepAudit(ctrl)
            ctrl.extra_audits.append(sleep_audit.audit)
        if args.contact_policy is not None:
            import hashlib
            from newton_contact_policy import ContactPolicyGPU
            policy_input = args.contact_policy.read_bytes()
            policy_input_sha256 = hashlib.sha256(policy_input).hexdigest()
            (out/'contact_policy_input.json').write_bytes(policy_input)
            ctrl.contact_policy = ContactPolicyGPU(solver, ctrl, timeline,
                json.loads(policy_input), metadata, worlds=args.worlds)
        contact_activity = None
        if adaptive is not None:
            from newton_adaptive_frequency import ContactActivityGPU
            contact_activity = ContactActivityGPU(ctrl)
            ctrl.extra_audits.append(contact_activity.audit)
        if args.native_control_callback and not np.array_equal(
                np.sort(initial['actuator_ids']), np.arange(source.nu)):
            raise ValueError('Native callback must own every source actuator')
        solver.mjw_model.callback.control = ctrl.control
        if not solver.mjw_model.flg_surfacevel:
            raise RuntimeError('Native belt surface velocity feature disabled')
        mw.kinematics(solver.mjw_model, d)
        if resume_state is not None:
            for field in ('time', 'qpos', 'qvel', 'eq_active', 'ctrl'):
                if not np.array_equal(getattr(d, field).numpy(), resume_state[field]):
                    raise ValueError('Measured continuation restore differs: ' + field)
            if not np.array_equal(solver.mjw_model.dof_damping.numpy(), resume_state['runtime_dof_damping']):
                raise ValueError('Measured continuation runtime damping differs')
            if 'qacc_warmstart' in resume_state and not np.array_equal(d.qacc_warmstart.numpy(), resume_state['qacc_warmstart']):
                raise ValueError('Measured continuation warmstart differs')
        if args.linear_algebra != 'warp':
            if args.cache_hessian:
                from newton_gpu_factor_cache import CachedCuSolverCholesky
                linear_backend=CachedCuSolverCholesky(solver)
            elif args.linear_algebra=='woodbury':
                from newton_gpu_woodbury import WoodburyCholesky
                linear_backend=WoodburyCholesky(solver,rank_capacity=128)
            elif args.linear_algebra=='cusolver32-batched':
                from newton_gpu_cusolver_batch import CuSolverCholeskyBatch
                linear_backend=CuSolverCholeskyBatch(solver,force_batched_single=True)
            elif args.linear_algebra=='cusolver32-equilibrated':
                from newton_gpu_equilibrate import EquilibratedCuSolverCholesky
                linear_backend=EquilibratedCuSolverCholesky(solver)
            else:
                from newton_gpu_cusolver import CuSolverCholesky
                linear_backend=CuSolverCholesky(solver,
                    precision='float64' if args.linear_algebra=='cusolver64' else 'float32')
        if args.dense_hessian:
            from newton_gpu_dense_hessian import DenseConstraintHessian
            hessian_backend=DenseConstraintHessian(solver,capacity=args.dense_hessian_capacity,
                **({'block_curvature': True} if args.dense_hessian_block_curvature else {}),
                **({'cache_jacobian': True} if args.cache_dense_jacobian else {}),
                **({'overflow_capacity': args.dense_hessian_overflow_capacity} if args.dense_hessian_overflow_capacity is not None else {}))
        if args.fixed_solver_reductions:
            if args.canonical_constraints:
                from newton_gpu_constraint_order import CanonicalConstraintOrder
                constraint_order=CanonicalConstraintOrder(solver,
                    allow_multiworld=args.experimental_multiworld_order)
            from newton_gpu_fixed_reductions import FixedSolverReductions
            fixed_reductions=FixedSolverReductions(solver,indexed=args.indexed_qfrc)
        if args.profile_gpu_events:
            from newton_gpu_profile import GraphEventProfile
            event_profile = GraphEventProfile(solver.mjw_model, args.device)
        stride = args.record_stride
        rows = steps // stride + 1
        qlog = wp.zeros((rows, source.nq), float, device=args.device)
        vlog = wp.zeros((rows, source.nv), float, device=args.device)
        elog = wp.zeros((rows, source.neq), bool, device=args.device)
        plog = wp.zeros((rows, source.nbody), wp.transform, device=args.device)
        world_trace = None
        if args.all_world_trace:
            world_trace = (wp.zeros((args.worlds, steps+1, source.nq), float, device=args.device),
                           wp.zeros((args.worlds, steps+1, source.nv), float, device=args.device),
                           wp.zeros((args.worlds, steps+1, source.neq), bool, device=args.device))
        nonfinite = wp.array(np.full(args.worlds, np.iinfo(np.int32).max, np.int32), dtype=int, device=args.device)
        packed_status = wp.zeros((args.worlds,4), wp.uint64, device=args.device)
        def record():
            wp.launch(save_trace, (args.worlds, max(source.nq, source.nv, source.nbody)),
                inputs=[ctrl.tick, stride, d.qpos, d.qvel, d.eq_active, d.xpos, d.xquat,
                        qlog, vlog, elog, plog, nonfinite], device=args.device)
            if world_trace is not None:
                from newton_kernels import save_all_world_trace
                wp.launch(save_all_world_trace, (args.worlds, max(source.nq, source.nv)),
                    inputs=[ctrl.tick, d.qpos, d.qvel, d.eq_active, *world_trace], device=args.device)
        record()
        def step(dt):
            ctrl.pre_step()
            engine.step(dt)
            ctrl.post_step()
            record()
        # Capture does not execute the graph. Kernel compilation belongs to
        # setup, and there are zero unrecorded integrated warmup steps.
        graphs = {}
        tail_graphs = {}
        durations = np.round(timesteps[:steps], 12)
        graph_capture_started = time.perf_counter()
        if not args.no_graph:
            print('Capturing fixed Newton physics graph...', flush=True)
            graph_durations = adaptive.graph_timesteps if adaptive is not None else np.unique(durations)
            for dt in graph_durations:
                with wp.ScopedCapture(device=args.device) as capture:
                    for _ in range(args.batch):
                        step(float(dt))
                graphs[float(dt)] = capture.graph
                if args.native_observables_only:
                    # Native MjData is authoritative in this mode; Newton's
                    # alternating metadata buffers are neither read nor written.
                    # Capture the remainder as well: an uncaptured deterministic
                    # step incurs host reductions/module switching, even though
                    # the exact same kernels already exist in the main graph.
                    states = (engine.state_0, engine.state_1)
                    with wp.ScopedCapture(device=args.device) as tail_capture:
                        step(float(dt))
                    tail_graphs[float(dt)] = tail_capture.graph
                    engine.state_0, engine.state_1 = states
        wp.synchronize()
        graph_capture_wall_s = time.perf_counter() - graph_capture_started
        if int(ctrl.tick.numpy()[0]) != 0:
            raise RuntimeError('Graph capture advanced unrecorded physical time')
        if resume_state is not None:
            for field in ('time', 'qpos', 'qvel', 'eq_active', 'ctrl', 'qacc_warmstart'):
                if field in resume_state and not np.array_equal(getattr(d, field).numpy(), resume_state[field]):
                    raise RuntimeError('Graph capture changed measured continuation state: ' + field)
            continuation['restore_and_graph_capture_preserved_state'] = True
        if linear_backend is not None and not args.no_graph and linear_backend.calls == 0:
            raise RuntimeError('Requested GPU linear algebra hook was not captured')
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
                physical_model_sha256=verified_input_hashes['input_model.mjb'])
            pose, gap = live_frame()
            publisher.publish(pose, gap, clock_offset, force=True)
            viewer = NewtonViewerProcess(out, fps=args.viewer_fps,
                seconds=0. if args.interactive_viewer else args.viewer_seconds, camera=args.camera,
                capture=out/'live_final.png',camera_test_motion=args.camera_test_motion,
                interactive=args.interactive_viewer)
            viewer.wait_ready(timeout=args.viewer_timeout)
        report.update(status='running', physics_hz=hz, requested_steps=steps,
            source_model_counts=provenance['model_counts'], input_sha256=sha256(p/'inputs.json'),
            commands_sha256=sha256(p/'commands.npz'), engine=engine.evidence,
            executed_commands_sha256=sha256(out/'executed_commands.npz'),
            cuda_graph=bool(graphs), graph_steps=args.batch if graphs else 0,
            cuda_tail_graph=bool(tail_graphs), graph_capture_wall_s=graph_capture_wall_s,
            free_hz=args.free_hz, timestep_counts={str(dt):int(np.sum(durations==dt)) for dt in np.unique(durations)},
            belt_hold_frequency=belt_hold_frequency,
            actual_timestep_limits_s=dict(min=float(durations.min()), max=float(durations.max())),
            effective_gpu_options=effective_gpu_options,
            explicit_gpu_solver_tolerance=explicit_gpu_tolerance,
            frozen_constraint_configuration=frozen_constraint_configuration,
            local_model_cache=model_cache_evidence,
            gpu_compatibility=compatibility,
            gpu_kernel_tuning=tuning,
            gpu_deterministic_bounds=deterministic_adapter.evidence if deterministic_adapter else {},
            gpu_sparse_factor=inertia_backend.evidence if inertia_backend else {},
            gpu_tree_sums=tree_backend.evidence if tree_backend else {},
            gpu_canonical_contacts=contact_order.evidence if contact_order else {},
            gpu_canonical_constraints=constraint_order.evidence if constraint_order else {},
            gpu_dense_hessian=hessian_backend.evidence if hessian_backend else {},
            gpu_convex_ccd_configuration=convex_ccd_configuration,
            gpu_fixed_solver_reductions=fixed_reductions.evidence if fixed_reductions else {},
            gpu_session=session_evidence,
            released_gutter_diagnostic=bool(metadata['released_gutter_diagnostic']),
            continuation_model_identity=continuation_identity,
            continuation=continuation, initial_physics_clock_s=clock_offset,
            source_model_mjb_sha256=verified_input_hashes['input_model.mjb'],
            gpu_linear_algebra=linear_backend.evidence if linear_backend else dict(implementation='MuJoCo Warp blocked Cholesky'),
            gpu_linear_algebra_capture_calls=linear_backend.calls if linear_backend else 0,
            record_stride=stride, unrecorded_warmup_steps=0, setup_wall_s=time.perf_counter()-started)
        write(out/'summary.json', report)
        if args.interactive_viewer:
            report.update(status='waiting_for_start', actual_steps=0,
                simulated_s=float(d.time.numpy()[0]), interactive_viewer=True,
                presentation_rate=1., timing_includes_presentation_wait=True)
            write(out/'summary.json', report)
            print(f'Scene ready. Press START (1x) in the Harvest control window. Physics is at t={clock_offset:g}.',flush=True)
            report['operator_wait_s'] = viewer.wait_user_start()
            report['status'] = 'running'
            write(out/'summary.json', report)
        actual = 0
        stop = None
        resource = []
        from benchmark_motion import sample
        monitored_process=psutil.Process()
        monitored_tree={monitored_process.pid:monitored_process}
        monitored_process.cpu_percent()
        display_errors = []
        phase_performance = {}
        gpu_profile = {}
        event_samples = {}
        execution_batches = dict(full_graph_steps=0, tail_graph_steps=0, uncaptured_steps=0)
        phase_counts = {phase:int(np.sum(timeline['phase'][:steps] == phase))
                        for phase in np.unique(timeline['phase'][:steps])}
        interruption_requested=[False]
        def request_interrupt(signum,frame):
            interruption_requested[0]=True
        previous_interrupt_handler=signal.signal(signal.SIGINT,request_interrupt)
        loop = time.perf_counter()
        from newton_live import WallClockPacer
        pacer = WallClockPacer() if args.interactive_viewer else None
        gpu_started = loop
        last_progress = loop
        while actual < steps:
            batch_started = time.perf_counter()
            adaptive_batch = None
            if adaptive is not None:
                active_contact = bool(contact_activity.flag.numpy()[0])
                adaptive_batch = adaptive.prepare(actual,args.batch if graphs else 1,active_contact)
                if not adaptive_batch:
                    break
                adaptive.upload(ctrl,actual,adaptive_batch)
                timesteps[actual:actual+adaptive_batch] = timeline['dt'][actual:actual+adaptive_batch]
                durations[actual:actual+adaptive_batch] = np.round(timeline['dt'][actual:actual+adaptive_batch],12)
            phase = str(timeline['phase'][actual])
            dt = float(durations[actual])
            batch = adaptive_batch if adaptive is not None else min(args.batch if graphs else 1, steps-actual)
            if batch > 1:
                different = np.flatnonzero(durations[actual:actual+batch] != dt)
                if len(different):
                    batch = int(different[0])
                changed_phase = np.flatnonzero(timeline['phase'][actual:actual+batch] != phase)
                if len(changed_phase):
                    batch = int(changed_phase[0])
            if (args.profile_gpu and phase not in gpu_profile
                    and phase_performance.get(phase,{}).get('steps',0) >= phase_counts[phase]//2):
                # One actual, fully recorded step per phase. This diagnostic
                # run includes event instrumentation and is not a clean speed
                # benchmark. No snapshot reset or unrecorded warmup is used.
                with wp.ScopedTimer(phase, print=False, cuda_filter=wp.TIMING_ALL) as timer:
                    step(dt)
                gpu_profile[phase] = dict(solver_step=actual, elapsed_ms=timer.elapsed,
                    solver_iterations=d.solver_niter.numpy().tolist(),
                    constraints=d.nefc.numpy().tolist(),
                    activities=[dict(name=r.name, kind=int(r.filter), elapsed_ms=r.elapsed)
                                for r in timer.timing_results])
                write(out/'gpu_profile.json',gpu_profile)
                batch = 1
                execution_batches['uncaptured_steps'] += 1
            elif graphs and batch == args.batch:
                wp.capture_launch(graphs[dt])
                execution_batches['full_graph_steps'] += batch
            elif tail_graphs:
                for _ in range(batch):
                    wp.capture_launch(tail_graphs[dt])
                execution_batches['tail_graph_steps'] += batch
            else:
                for _ in range(batch):
                    step(dt)
                execution_batches['uncaptured_steps'] += batch
            actual += batch
            stopping_event = ctrl.contact_policy.first_event if ctrl.contact_policy is not None else ctrl.first_event
            wp.launch(pack_runtime_status, args.worlds, inputs=[nonfinite,
                ctrl.first_qacc_nonfinite, ctrl.first_capacity_overflow,
                stopping_event, packed_status], device=args.device)
            # One synchronized device readback rather than four individually
            # synchronized .numpy() transfers after every graph launch.
            status = packed_status.numpy()
            invalid = int(status[:,:2].min())
            capacity = int(status[:,2].min())
            first = status[:,3]
            if linear_backend is not None:
                try:
                    linear_backend.check_health()
                except RuntimeError as exc:
                    stop='gpu_linear_solve_failure'
                    report['gpu_linear_solve_error']=str(exc)
            if inertia_backend is not None and hasattr(inertia_backend,'check_health'):
                try:
                    inertia_backend.check_health()
                except RuntimeError as exc:
                    stop='gpu_inertia_solve_failure'
                    report['gpu_inertia_solve_error']=str(exc)
            if fixed_reductions is not None:
                try:
                    fixed_reductions.check_health()
                except RuntimeError as exc:
                    stop='gpu_fixed_reduction_failure'
                    report['gpu_fixed_reduction_error']=str(exc)
            if stop:
                pass
            elif invalid != np.iinfo(np.int32).max:
                stop = 'nonfinite_physics'
            elif capacity != np.iinfo(np.int32).max:
                stop = 'solver_capacity_overflow'
            elif (first != np.iinfo(np.uint64).max).any() and not args.diagnostic_continue:
                stop = 'contact_policy_violation' if ctrl.contact_policy is not None else 'active_contact_penetration_1mm'
            if interruption_requested[0] and not stop:
                stop='operator_interrupt'
            if viewer is not None and not viewer.is_alive() and not display_errors:
                display_errors.append('Independent live viewer exited during physics; producer continues unchanged')
            if pacer is not None:
                pacer.wait(float(edges[actual]))
            if publisher is not None and publisher.due():
                pose, gap = live_frame()
                publisher.publish(pose, gap, float(d.time.numpy()[0]))
            perf = phase_performance.setdefault(phase, dict(steps=0, simulated_s=0., wall_s=0.))
            perf['steps'] += batch
            perf['simulated_s'] += float(np.sum(timesteps[actual-batch:actual]))
            perf['wall_s'] += time.perf_counter()-batch_started
            if (event_profile is not None and phase not in event_samples
                    and perf['steps'] >= phase_counts[phase]//2):
                event_samples[phase] = dict(solver_step=actual-1,
                    solver_iterations=d.solver_niter.numpy().tolist(),
                    constraints=d.nefc.numpy().tolist(),
                    last_invocation_cuda_ms=event_profile.read())
                write(out/'gpu_graph_event_profile.json', dict(
                    diagnostic_instrumented=True, nested_intervals_overlap=True,
                    samples=event_samples))
            if time.perf_counter()-last_progress >= 5 or stop:
                elapsed = time.perf_counter()-loop
                print(f'Newton {args.worlds} env | {float(edges[actual]):.2f}/{float(edges[steps]):.2f}s | '
                      f'RTF {float(edges[actual])/elapsed:.3f} | stop={stop}', flush=True)
                resource.append(dict(wall_s=elapsed, simulated_s=float(edges[actual]),
                    **sample(monitored_process,monitored_tree)))
                last_progress = time.perf_counter()
            if stop:
                break
        wp.synchronize()
        loop_s = time.perf_counter()-loop
        if pacer is not None:
            report['presentation_wait_s'] = pacer.wait_s
        for perf in phase_performance.values():
            perf['realtime_factor'] = perf['simulated_s']/perf['wall_s']
        write(out/'phase_performance.json', phase_performance)
        report['execution_batches'] = execution_batches
        if args.profile_gpu:
            write(out/'gpu_profile.json', gpu_profile)
        final_physics_clock_s = float(d.time.numpy()[0])
        simulated_s = final_physics_clock_s - clock_offset
        if adaptive is not None:
            timeline = adaptive.finish(actual)
            edges = timeline['time_edges']
            np.savez_compressed(out/'executed_commands.npz',**timeline)
            report['executed_commands_sha256'] = sha256(out/'executed_commands.npz')
            report['adaptive_frequency'] = adaptive.evidence()
            report['requested_steps'] = actual if adaptive.complete else steps
            np.save(out/'adaptive_contact_activity.npy',contact_activity.log.numpy()[:actual])
            write(out/'adaptive_frequency.json',adaptive.evidence())
            report['timestep_counts'] = {str(float(dt)):int(n) for dt,n in zip(*np.unique(timeline['dt'],return_counts=True))}
        device_commands = ctrl.commands.numpy()[:actual]
        device_belts = ctrl.belts.numpy()[:actual]
        device_inputs_match = bool(np.array_equal(device_commands,timeline['command'][:actual].astype(np.float32))
            and np.array_equal(device_belts,timeline['belt_surface_velocity_local_y_m_s'][:actual].astype(np.float32)))
        np.savez_compressed(out/'applied_command_buffers.npz',command=device_commands,
                            belt_surface_velocity_local_y_m_s=device_belts)
        report['device_command_buffers_match_schedule'] = device_inputs_match
        report['applied_command_buffers_sha256'] = sha256(out/'applied_command_buffers.npz')
        report['controller_tick_matches_recorded_steps'] = int(ctrl.tick.numpy()[0]) == actual
        if not device_inputs_match or not report['controller_tick_matches_recorded_steps']:
            stop = stop or 'device_command_schedule_mismatch'
        if publisher is not None:
            pose, gap = live_frame()
            publisher.publish(pose, gap, final_physics_clock_s, force=True)
            publisher.close(stop)
            try:
                viewer.wait_displayed(final_physics_clock_s, timeout=20.)
            except (RuntimeError, TimeoutError, ValueError) as exc:
                display_errors.append(repr(exc))
            if not args.interactive_viewer:
                viewer.close()
                viewer = None
            publisher = None
        count = actual//stride + 1
        q = qlog.numpy()[:count]
        vel = vlog.numpy()[:count]
        eq = elog.numpy()[:count]
        pose = plog.numpy()[:count]
        actual_times = np.concatenate(([clock_offset], ctrl.time_log.numpy()[0, stride-1:actual:stride]))
        np.savez_compressed(out/'states.npz', time=actual_times,
            scheduled_time=clock_offset+edges[np.arange(count)*stride], solver_step=np.arange(count)*stride,
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
            released_gutter_penetration_m=ctrl.gutter_max.numpy()[:, :actual],
            released_gutter_force_n=ctrl.gutter_force.numpy()[:, :actual],
            released_gutter_contact_pair_steps=ctrl.gutter_pair_steps.numpy()[:, :actual],
            attachment_force_n=f, attachment_torque_nm=torque, break_step=break_step,
            first_event=ctrl.first_event.numpy(), nonfinite_step=nonfinite.numpy(),
            qacc_nonfinite_step=ctrl.first_qacc_nonfinite.numpy(),
            capacity_overflow_step=ctrl.first_capacity_overflow.numpy(),
            nacon=ctrl.nacon_log.numpy()[:actual], ncollision=ctrl.ncollision_log.numpy()[:actual],
            nefc=ctrl.nefc_log.numpy()[:,:actual], overflow=ctrl.overflow_log.numpy()[:,:actual],
            qacc_abs_max=ctrl.qacc_abs_max.numpy()[:,:actual], time=ctrl.time_log.numpy()[:,:actual])
        np.save(out/'solver_iterations.npy',ctrl.solver_niter_log.numpy()[:,:actual])
        backend_times=ctrl.time_log.numpy()[:, :actual]
        violations=[]
        for w,event in enumerate(ctrl.first_event.numpy()):
            if event == np.iinfo(np.uint64).max:
                continue
            event=int(event); tick=event>>32
            pair=event & ((1<<32)-1); ids=[pair//65536,pair%65536]
            violations.append(dict(world=w,solver_step=tick,
                scheduled_start_s=clock_offset+float(edges[tick]),
                actual_end_s=float(backend_times[w,tick]),phase=str(timeline['phase'][tick]),
                geom_ids=ids,geoms=[source.geom(i).name for i in ids],
                bodies=[source.body(int(source.geom_bodyid[i])).name for i in ids],
                step_max_active_penetration_m=float(active[w,tick])))
        breaks = []
        for w,i in np.argwhere(break_step>=0):
            tick = int(break_step[w,i])
            breaks.append(dict(world=int(w), name=metadata['attachments'][i]['name'],
                solver_step=tick, solver_time_s=clock_offset if tick==0 else float(backend_times[w,tick-1]),
                release_effective_time_s=float(backend_times[w,tick]),
                scheduled_solver_time_s=clock_offset+float(edges[tick]), scheduled_release_time_s=clock_offset+float(edges[tick+1]),
                phase=str(timeline['phase'][tick]), constraint_force_n=float(f[w,tick,i]),
                passive_torque_nm=float(torque[w,tick,i])))
        np.savez_compressed(out/'final_states.npz', time=d.time.numpy(), qpos=d.qpos.numpy(),
            qvel=d.qvel.numpy(), eq_active=d.eq_active.numpy(), ctrl=d.ctrl.numpy(),
            runtime_dof_damping=solver.mjw_model.dof_damping.numpy(),
            qacc_warmstart=d.qacc_warmstart.numpy(),
            body_names=initial['body_names'], attachment_equality_ids=ctrl.eq_np)
        inherited_and_new = cumulative_breaks(inherited_breaks, breaks)
        write(out/'cumulative_break_events.json', inherited_and_new)
        if world_trace is not None:
            np.savez_compressed(out/'all_world_states.npz',
                time=ctrl.time_log.numpy()[:,:actual],
                qpos=world_trace[0].numpy()[:,:actual+1],
                qvel=world_trace[1].numpy()[:,:actual+1],
                eq_active=world_trace[2].numpy()[:,:actual+1])
        if args.diagnostic_data_snapshot:
            from newton_gpu_session import _arrays
            np.savez_compressed(out/'native_data_debug.npz',**{name:array.numpy() for name,array in _arrays(d)})
        write(out/'break_events.json', breaks)
        write(out/'resources.json', resource)
        if linear_backend is not None and hasattr(linear_backend,'statistics'):
            report['gpu_linear_algebra_statistics']=linear_backend.statistics()
        if hessian_backend is not None:
            report['gpu_dense_hessian_statistics']=hessian_backend.statistics()
        if fixed_reductions is not None:
            report['gpu_fixed_solver_reductions_statistics']=fixed_reductions.statistics()
        report.update(write_display_evidence(out, metadata, provenance, breaks))
        if sleep_audit is not None:
            report['fruit_sleep'] = sleep_audit.save(out, actual)
        if ctrl.contact_policy is not None:
            ctrl.contact_policy.save(out, actual)
            report['contact_policy'] = ctrl.contact_policy.report(actual)
            report['contact_policy_input_sha256'] = policy_input_sha256
        report.update(status='stopped' if stop else 'completed', stop=stop,
            simulated_s=simulated_s, scheduled_simulated_s=float(edges[actual]), actual_steps=actual, loop_wall_s=loop_s,
            segment_simulated_s=simulated_s, final_physics_clock_s=final_physics_clock_s,
            cumulative_break_events_sha256=sha256(out/'cumulative_break_events.json'),
            inherited_release_event_count=len(inherited_breaks),
            cumulative_release_event_count=len(inherited_and_new),
            released_gutter_contacts=dict(
                enabled=bool(metadata['released_gutter_diagnostic']),
                exact_geom_names=[source.geom(int(i)).name for i in ctrl.gutter_geom_ids],
                maximum_penetration_m=float(ctrl.gutter_max.numpy()[:, :actual].max(initial=0)),
                maximum_contact_force_n=float(ctrl.gutter_force.numpy()[:, :actual].max(initial=0)),
                contact_pair_steps=int(ctrl.gutter_pair_steps.numpy()[:, :actual].sum()),
                physics_changed=False,
                scope='Only exact world gutter colliders versus fruit already released at solver start; attached fruit, robot and all other contacts retain the existing stop policy'),
            realtime_factor=simulated_s/loop_s, aggregate_sim_s_per_wall_s=args.worlds*simulated_s/loop_s,
            phase_performance=phase_performance, profile_instrumented=bool(args.profile_gpu or args.profile_gpu_events
                or (ctrl.contact_policy is not None and ctrl.contact_policy.config.get('record_contacts'))),
            deterministic_mode=args.deterministic,
            resource_scope='CPU/RSS: owned process tree; GPU: whole device including live Isaac display',
            display_errors=display_errors, display_health_passed=not display_errors,
            max_active_penetration_m=float(active.max(initial=0)),
            max_released_penetration_m=float(released.max(initial=0)),
            max_contact_force_n=float(force.max(initial=0)), breaks=breaks,
            first_active_violations=violations,
            active_contact_criterion_passed=not np.any(active>.001),
            finite_recorded_state=bool(np.isfinite(q).all() and np.isfinite(vel).all()),
            solver_capacity_valid=bool(np.all(ctrl.first_capacity_overflow.numpy()==np.iinfo(np.int32).max)),
            qacc_finite=bool(np.all(ctrl.first_qacc_nonfinite.numpy()==np.iinfo(np.int32).max)),
            max_constraints=int(ctrl.nefc_log.numpy().max(initial=0)),
            max_total_contacts=int(ctrl.nacon_log.numpy().max(initial=0)),
            gpu_timestamp_drift_s=float(actual_times[-1]-(clock_offset+edges[actual])),
            states_sha256=sha256(out/'states.npz'), final_time_per_world_s=d.time.numpy().tolist(),
            all_world_temporal_state_recorded=world_trace is not None,
            all_world_states_sha256=sha256(out/'all_world_states.npz') if world_trace is not None else None,
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
        if args.interactive_viewer and viewer is not None:
            print('Physics finished and results saved. The final scene stays open until you close the window.',flush=True)
            while viewer.is_alive() and not interruption_requested[0]:
                time.sleep(.1)
        return report
    except BaseException as exc:
        report.update(status='error', error=repr(exc), traceback=traceback.format_exc(), total_wall_s=time.perf_counter()-started)
        write(out/'summary.json', report)
        raise
    finally:
        if previous_interrupt_handler is not None:
            signal.signal(signal.SIGINT,previous_interrupt_handler)
        # Fixed reductions wrap the Hessian hook; restore the outer owner first.
        if event_profile is not None:
            event_profile.close()
        if fixed_reductions is not None:
            fixed_reductions.close()
        if constraint_order is not None:
            constraint_order.close()
        if linear_backend is not None:
            linear_backend.close()
        if deterministic_adapter is not None:
            deterministic_adapter.close()
        if inertia_backend is not None:
            inertia_backend.close()
        if tree_backend is not None:
            tree_backend.close()
        if contact_order is not None:
            contact_order.close()
        if hessian_backend is not None:
            hessian_backend.close()
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
    p.add_argument('--belt-hold-hz',type=float,
        help='Opt-in accuracy trial: resample only stationary belt_finish, preserve other rows and hold endpoints')
    p.add_argument('--threads',type=int,default=0,help='Native CPU threadpool workers')
    p.add_argument('--native-observables-only',action=argparse.BooleanOptionalAction,default=False,help='Skip unused Newton metadata refresh; actual native state remains authoritative')
    p.add_argument('--native-control-callback',action='store_true',help='GPU callback owns every actuator; skip unused Newton control readback')
    p.add_argument('--deterministic', choices=('NOT_GUARANTEED','RUN_TO_RUN'),default='NOT_GUARANTEED')
    p.add_argument('--deterministic-scope',choices=('solver','constraints','dynamics','all'),help='Bounded pinned-backend kernels; narrowed scopes require separate tree/contact handling')
    p.add_argument('--native-integer-atomics',action='store_true',help='Avoid deterministic scatter machinery in reviewed integer-only/no-scatter solver specializations')
    p.add_argument('--profile-gpu',action='store_true',help='Instrument one recorded actual step per phase; diagnostic timing only')
    p.add_argument('--profile-gpu-events',action='store_true',help='CUDA event timings inside actual graphs, including external CUDA libraries')
    p.add_argument('--jacobian',choices=('sparse','dense'),help='Explicit native constraint storage comparison')
    p.add_argument('--profile-timers',action='store_true',help='Diagnostic native C timers; not a clean throughput benchmark')
    p.add_argument('--solver-tolerance',type=float,help='Explicit convergence tolerance comparison')
    p.add_argument('--solver',choices=('Newton','CG'),help='Explicit native solver diagnostic; original default unchanged')
    p.add_argument('--solver-iterations',type=int,help='Explicit solver budget diagnostic, original default unchanged')
    p.add_argument('--ls-iterations',type=int,help='Explicit GPU line-search budget diagnostic')
    p.add_argument('--broadphase',choices=('NXN','SAP_TILE','SAP_SEGMENTED'),help='Explicit GPU broadphase implementation comparison')
    p.add_argument('--aabb-filter',action='store_true',help='Add conservative AABB rejection before OBB filtering')
    p.add_argument('--cholesky-block-dim',type=int,choices=(32,64,128,256),help='GPU threads per blocked constraint factorization')
    p.add_argument('--linesearch-block-dim',type=int,choices=(32,64,128,256),
        help='Opt-in GPU line-search reduction block size; floating point grouping may change')
    p.add_argument('--convex-ccd-block-dim',type=int,choices=(32,64,128,256),
        help='Opt-in convex CCD thread block size; keep native collision geometry, iterations and tolerance')
    p.add_argument('--refresh-connect-velocity',action=argparse.BooleanOptionalAction,default=True,help='Refresh current CONNECT velocity cache before constraint reference assembly')
    p.add_argument('--mathdx-gemm',action=argparse.BooleanOptionalAction,default=None,help='Explicit tile GEMM compiler comparison, original default unchanged')
    p.add_argument('--linear-algebra',choices=('warp','cusolver32','cusolver32-batched','cusolver64','cusolver32-equilibrated','woodbury'),default='warp',help='Opt-in GPU SPD factor/solve implementation; same Newton equations')
    p.add_argument('--fused-sparse-factor',action='store_true',help='Opt-in one-CTA sparse inertia factor with fixed update order')
    p.add_argument('--inertia-linear-algebra',choices=('warp','cusolver32'),default='warp')
    p.add_argument('--tree-sums',choices=('levels','flat'),help='Opt-in fixed-order tree equations')
    p.add_argument('--canonical-contacts',action='store_true',help='Stable contact record order; no contact values changed')
    p.add_argument('--canonical-constraints',action='store_true',help='Canonicalize native constraint rows and dependent pointers after parallel allocation')
    p.add_argument('--experimental-multiworld-order',action='store_true',help='Opt-in world-indexed constraint canonicalization; requires separate full harvest qualification')
    p.add_argument('--dense-hessian',action='store_true',help='Opt-in equivalent dense GPU product for sparse constraint curvature')
    p.add_argument('--dense-hessian-capacity',type=int,default=128,
        help='Maximum constraint rows for dense GPU assembly; larger rows keep native fallback. No physical constraint truncation.')
    p.add_argument('--dense-hessian-block-curvature',action='store_true',
        help='Opt-in at-most3x3 curvature product instead of first dense SGEMM; final J.T*B remains cuBLAS')
    p.add_argument('--dense-hessian-overflow-capacity',type=int,
        help='Opt-in larger secondary GPU workspace only above primary capacity, preserving smaller-row calculation')
    p.add_argument('--cache-dense-jacobian',action='store_true',
        help='Opt-in reuse of the unchanged dense Jacobian only within each native Newton solve')
    p.add_argument('--fixed-solver-reductions',action='store_true',help='Opt-in fixed-order sparse force and native tiled gradient reductions')
    p.add_argument('--indexed-qfrc',action='store_true',help='Reuse per-step sparse column lookup during Newton iterations')
    p.add_argument('--cache-hessian',action='store_true',help='Reuse factors only for bit-identical upper H; GPU comparison and conditional execution')
    p.add_argument('--repeats',type=int,default=1,help='GPU trials in one process with retained model and full resets')
    p.add_argument('--resume-state',type=Path,
        help='Opt-in measured terminal state continuation from a completed same-MJB GPU run; new commands must start at measured robot q')
    p.add_argument('--window-transition',type=Path,
        help='Opt-in verified canonical state remapping across an active-window model change')
    p.add_argument('--local-model-cache',type=Path,
        help='Verify source MJB and stage identical bytes in a local content-addressed cache before loading')
    p.add_argument('--gui-repeat-index',type=int,help='Show live GUI only for this zero-based repeat')
    p.add_argument('--diagnostic-data-snapshot',action='store_true',help='Save final native GPU intermediate arrays for short reproducibility diagnostics')
    p.add_argument('--start-gate',type=Path);p.add_argument('--ready-file',type=Path)
    p.add_argument('--start-gate-timeout',type=float,default=300.)
    p.add_argument('--seconds',type=float); p.add_argument('--batch',type=int,default=10)
    p.add_argument('--record-stride',type=int,default=1)
    p.add_argument('--all-world-trace',action='store_true',help='Record every world q/v/equality at each physics step for batch qualification')
    p.add_argument('--nconmax',type=int,default=512); p.add_argument('--njmax',type=int,default=2048)
    p.add_argument('--no-graph',action='store_true');p.add_argument('--diagnostic-continue',action='store_true')
    p.add_argument('--contact-policy',type=Path,help='Opt-in exact belt pair/phase stop policy; original 1mm audit is retained')
    p.add_argument('--released-fruit-sleep',action='store_true',help='Opt-in native sleeping and wake-up for fruit trees only; no collider removal')
    p.add_argument('--released-gutter-diagnostic',action='store_true',
        help='Keep physical impacts but diagnose rather than stop only already-released fruit versus exact world gutter colliders')
    p.add_argument('--adaptive-hz',action='store_true',help='Opt-in actual tool/plant contact-driven --hz/--free-hz scheduling')
    p.add_argument('--gui',action='store_true');p.add_argument('--viewer-fps',type=float,default=15.)
    p.add_argument('--interactive-viewer',action=argparse.BooleanOptionalAction,default=False,
                   help='Wait for viewer START, pace fixed physics steps to1x, keep final scene open')
    p.add_argument('--viewer-timeout',type=float,default=300.);p.add_argument('--viewer-seconds',type=float,default=600.)
    p.add_argument('--camera',default='entry')
    p.add_argument('--camera-test-motion',action='store_true',help='Move display camera automatically and require recorded movement ACK')
    args=p.parse_args(argv)
    if args.dense_hessian and not 1 <= args.dense_hessian_capacity <= args.njmax:
        p.error('--dense-hessian-capacity must be positive and no larger than --njmax')
    if args.dense_hessian_block_curvature and not args.dense_hessian:
        p.error('--dense-hessian-block-curvature requires --dense-hessian')
    if args.cache_dense_jacobian and not args.dense_hessian:
        p.error('--cache-dense-jacobian requires --dense-hessian')
    if args.dense_hessian_overflow_capacity is not None and (not args.dense_hessian or
            not args.dense_hessian_capacity < args.dense_hessian_overflow_capacity <= args.njmax):
        p.error('Overflow Hessian capacity requires dense mode and primary<overflow<=njmax')
    try:
        solver_tolerance_value(args.solver_tolerance)
    except ValueError as error:
        p.error(str(error))
    if args.resume_state is not None and (args.backend!='gpu' or args.worlds!=1 or args.repeats!=1):
        p.error('--resume-state requires exactly one GPU world/run')
    if args.window_transition is not None and (args.resume_state is not None or args.backend!='gpu' or args.worlds!=1 or args.repeats!=1):
        p.error('--window-transition requires one GPU world/run and excludes --resume-state')
    if args.released_gutter_diagnostic and args.backend!='gpu':
        p.error('--released-gutter-diagnostic currently requires the GPU executor')
    if args.interactive_viewer and (not args.gui or args.backend!='gpu' or args.repeats!=1):
        p.error('--interactive-viewer requires one GPU run with --gui')
    if args.contact_policy is not None and args.backend!='gpu':
        p.error('--contact-policy requires the GPU executor')
    if args.released_fruit_sleep and args.backend!='gpu':
        p.error('--released-fruit-sleep requires the GPU executor')
    if args.adaptive_hz and (args.backend!='gpu' or args.worlds!=1 or args.profile_gpu or args.profile_gpu_events):
        p.error('--adaptive-hz requires one unprofiled GPU world')
    if args.belt_hold_hz is not None:
        from newton_belt_hold import validate_belt_hold_hz
        try:
            validate_belt_hold_hz(args.belt_hold_hz, args.hz)
        except ValueError as error:
            p.error(str(error))
        if args.backend != 'gpu' or args.adaptive_hz:
            p.error('--belt-hold-hz requires GPU execution and excludes --adaptive-hz')
    if args.canonical_constraints and (args.deterministic_scope!='solver' or not args.fixed_solver_reductions):
        p.error('--canonical-constraints requires --deterministic-scope solver and --fixed-solver-reductions')
    if args.experimental_multiworld_order and not args.canonical_constraints:
        p.error('--experimental-multiworld-order requires --canonical-constraints')
    if args.native_integer_atomics and not args.deterministic_scope:
        p.error('--native-integer-atomics requires --deterministic-scope')
    if args.indexed_qfrc and not args.fixed_solver_reductions:
        p.error('--indexed-qfrc requires --fixed-solver-reductions')
    if args.cache_hessian and (args.backend!='gpu' or args.linear_algebra!='cusolver32'):
        p.error('--cache-hessian requires GPU --linear-algebra cusolver32')
    if args.worlds<1 or args.record_stride<1:p.error('positive worlds and record stride required')
    if args.backend=='gpu' and args.record_stride!=1:p.error('GPU evidence currently requires record stride 1')
    if args.all_world_trace and args.backend!='gpu':p.error('--all-world-trace requires GPU')
    if args.convex_ccd_block_dim is not None and args.backend!='gpu':
        p.error('--convex-ccd-block-dim requires --backend gpu')
    cpu_only=('profile_timers','threads','start_gate','ready_file')
    if args.backend=='gpu' and any(getattr(args,key) for key in cpu_only):
        p.error('CPU-only diagnostics/gates cannot be applied to the GPU backend')
    if args.backend=='cpu' and (args.native_control_callback or args.profile_gpu or args.profile_gpu_events or args.deterministic!='NOT_GUARANTEED' or args.ls_iterations or args.broadphase or args.aabb_filter or args.cholesky_block_dim or args.mathdx_gemm is not None or args.linear_algebra!='warp'):
        p.error('GPU callback/profiling/determinism require --backend gpu')
    if args.ls_iterations is not None and args.ls_iterations<1:p.error('Positive line-search budget required')
    if args.deterministic_scope and (args.backend!='gpu' or args.deterministic!='RUN_TO_RUN'):
        p.error('Bounded deterministic scope requires GPU RUN_TO_RUN')
    if args.fused_sparse_factor and args.backend!='gpu':p.error('Sparse factor fusion requires GPU')
    if args.fused_sparse_factor and args.inertia_linear_algebra!='warp':p.error('Select one inertia implementation')
    if args.backend!='gpu' and (args.inertia_linear_algebra!='warp' or args.tree_sums or args.canonical_contacts or args.camera_test_motion or args.dense_hessian or args.fixed_solver_reductions):
        p.error('GPU math/camera diagnostics require --backend gpu')
    if args.repeats<1:p.error('Positive repeat count required')
    if args.backend=='cpu' and args.repeats!=1:p.error('Persistent repeats require --backend gpu')
    if args.gui_repeat_index is not None and not 0<=args.gui_repeat_index<args.repeats:
        p.error('GUI repeat index must be inside the requested repeat count')
    return args


def main():
    args=arguments()
    if args.backend=='cpu':
        from newton_cpu import run_cpu
        result=run_cpu(args)
    else:
        if args.repeats==1:
            result=run(args)
        else:
            from newton_gpu_session import NewtonGPUSessionCache
            cache=NewtonGPUSessionCache()
            parent=args.output.resolve()
            parent.mkdir(parents=True,exist_ok=False)
            results=[]
            for index in range(args.repeats):
                trial=argparse.Namespace(**vars(args))
                trial.output=parent/f'repeat_{index:03d}'
                if args.gui_repeat_index is not None:
                    trial.gui=index==args.gui_repeat_index
                result=run(trial,session_cache=cache)
                results.append(dict(output=str(trial.output),status=result['status'],
                    realtime_factor=result.get('realtime_factor'),gpu_session=result.get('gpu_session')))
                write(parent/'session_repeats.json',dict(trials=results))
                if result['status']!='completed':break
    print(json.dumps({key:result.get(key) for key in ('status','stop','backend','worlds','simulated_s','loop_wall_s','realtime_factor','aggregate_sim_s_per_wall_s','detached_per_world','max_active_penetration_m')},indent=2))
    print('Saved:', args.output.resolve(), flush=True)


if __name__=='__main__':main()
