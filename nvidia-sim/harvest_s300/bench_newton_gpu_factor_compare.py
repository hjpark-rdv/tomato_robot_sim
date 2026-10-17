"""Same-input single-world factor/solve comparison, with explicit validity gates.

Uses the existing factor133 benchmark's system/context constructors and actual
installed solver implementations. CUDA events surround completed graph replays,
including preparation and search-direction finalization. No physics is stepped;
passing this algebra check does not qualify a harvest trajectory.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import warp as wp
from mujoco_warp._src import solver as ms, types as mt

from bench_newton_gpu_factor import context, system
from newton_gpu_cusolver import CuSolverCholesky
from newton_gpu_cusolver_batch import CuSolverCholeskyBatch


BACKENDS = ('cusolver32', 'cusolver64', 'cusolver32-batched', 'warp32', 'warp64')


def inputs(args):
    """Return identical float32 native inputs for every implementation."""
    if args.nv <= ms._BLOCK_CHOLESKY_DIM or args.pad < args.nv or args.pad % mt.TILE_SIZE_JTDAJ_DENSE:
        raise ValueError('Need blocked nv and pad>=nv aligned to native dense tile size')
    a, h, grad = system(args.nv, args.pad, args.seed)
    if args.hessian:
        with np.load(args.hessian, allow_pickle=False) as data:
            h = np.asarray(data['h'], np.float32)
            grad = np.asarray(data['grad'], np.float32)
        if h.shape not in ((args.pad, args.pad), (1, args.pad, args.pad)):
            raise ValueError('Saved h must have shape (pad,pad) or (1,pad,pad)')
        if grad.shape not in ((args.pad,), (1, args.pad)):
            raise ValueError('Saved grad must have shape (pad,) or (1,pad)')
        h, grad = h.reshape(args.pad, args.pad).copy(), grad.reshape(args.pad).copy()
        if args.diagonal_span != 1.:
            raise ValueError('Synthetic diagonal scaling cannot be applied to saved H')
    elif args.diagonal_span != 1.:
        scale = np.geomspace(1., np.sqrt(args.diagonal_span), args.nv).astype(np.float64)
        a = (scale[:, None] * a.astype(np.float64) * scale[None, :]).astype(np.float32)
        h[:args.nv, :args.nv] = np.triu(a)
    upper = np.triu(h[:args.nv, :args.nv])
    a = (upper + upper.T - np.diag(upper.diagonal())).astype(np.float64)
    if not np.isfinite(a).all() or not np.isfinite(grad).all():
        raise ValueError('Active H and gradient must be finite')
    # cuSOLVER deliberately replaces padding by identity/zero. Native Warp
    # assumes zero cross/padding off-diagonals and only rewrites its diagonal.
    # Refuse an incompatible snapshot instead of silently changing its input.
    if np.any(np.triu(h, 1)[:, args.nv:]) or np.any(grad[args.nv:]):
        raise ValueError('Saved native padding must have zero upper off-diagonals and gradient')
    np.linalg.cholesky(a)  # Diagnostic CPU oracle only; never timed or used by a GPU solve.
    oracle = np.linalg.solve(a, grad[:args.nv].astype(np.float64))
    return a, h, grad, oracle


def accuracy(a, grad, oracle, search, search_dot, decrement, args):
    solution = -np.asarray(search, np.float64)
    finite = bool(np.isfinite(solution).all() and np.isfinite(search_dot) and np.isfinite(decrement))
    if not finite:
        return dict(numerically_valid=False, finite=False, failure='Nonfinite solution or reduction')
    rhs = np.asarray(grad[:len(solution)], np.float64)
    residual = a @ solution - rhs
    tiny = np.finfo(np.float64).tiny
    relative_residual = float(np.linalg.norm(residual) / max(float(np.linalg.norm(rhs)), tiny))
    relative_solution_error = float(np.linalg.norm(solution-oracle) / max(float(np.linalg.norm(oracle)), tiny))
    backward_error = float(np.linalg.norm(residual, np.inf) / max(
        float(np.linalg.norm(a, np.inf)*np.linalg.norm(solution, np.inf)+np.linalg.norm(rhs, np.inf)), tiny))
    expected_dot, expected_decrement = float(oracle @ oracle), float(rhs @ oracle)
    dot_error = abs(float(search_dot)-expected_dot) / max(abs(expected_dot), tiny)
    decrement_error = abs(float(decrement)-expected_decrement) / max(abs(expected_decrement), tiny)
    valid = (relative_residual <= args.max_relative_residual
             and relative_solution_error <= args.max_relative_solution_error
             and max(dot_error, decrement_error) <= args.max_relative_solution_error)
    return dict(numerically_valid=bool(valid), finite=True,
        relative_residual=relative_residual, relative_solution_error=relative_solution_error,
        backward_error_inf=backward_error, search_dot=float(search_dot),
        newton_decrement=float(decrement), oracle_search_dot=expected_dot,
        oracle_newton_decrement=expected_decrement, search_dot_relative_error=dot_error,
        newton_decrement_relative_error=decrement_error)


def clean_json(value):
    """Keep failure reports writable without encoding nonstandard NaN tokens."""
    if isinstance(value, dict):
        return {key: clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(item) for item in value]
    if isinstance(value, (float, np.floating)) and not np.isfinite(value):
        return None
    return value


def benchmark(args):
    out = Path(args.output).absolute()
    # Reserve the new report before GPU setup; never replace old experiment data.
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x') as file:
        file.write('{"status":"validating_inputs"}\n')
    report = dict(schema='s300-single-world-factor-comparison-v1', status='running',
        simulation_steps=0, harvest_realtime_factor=None, nv=args.nv, pad=args.pad,
        seed=args.seed, diagonal_span=args.diagonal_span, synthetic_hessian=not bool(args.hessian),
        hessian_origin=str(Path(args.hessian).resolve()) if args.hessian else None,
        samples_per_graph=args.samples, graph_replays=args.replays,
        timing_scope='complete prepare+factor+solve+finalize; synchronized CUDA events around graph replay',
        validity_thresholds=dict(relative_residual=args.max_relative_residual,
            relative_solution_error=args.max_relative_solution_error,
            relative_search_dot_and_decrement_error=args.max_relative_solution_error,
            repeated_identical_inputs_require_bitwise_identical_outputs=True),
        limits=['Algebra microbenchmark only; no harvest or contact-policy qualification',
                'done=True controls are excluded from speed ranking'], results=[])
    def save():
        out.write_text(json.dumps(clean_json(report), indent=2, allow_nan=False)+'\n')
    try:
        a, h, grad, oracle = inputs(args)
        report.update(input_h_sha256=hashlib.sha256(h.tobytes()).hexdigest(),
            input_grad_sha256=hashlib.sha256(grad.tobytes()).hexdigest(),
            condition_number=float(np.linalg.cond(a)),
            package_versions={name: importlib.metadata.version(name) for name in
                ('warp-lang', 'mujoco-warp', 'newton')},
            code_sha256={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                for name in (Path(__file__).name, 'bench_newton_gpu_factor.py',
                             'newton_gpu_cusolver.py', 'newton_gpu_cusolver_batch.py')},
            stock_solver_sha256=hashlib.sha256(Path(ms.__file__).read_bytes()).hexdigest())
        device = wp.get_device(args.device)
        if not device.is_cuda:
            raise ValueError('This timing benchmark requires CUDA')
        stream = wp.get_stream(device)
        report.update(device=str(device), device_name=device.name)
        native_factor = ms._cholesky_factorize_solve
        model = SimpleNamespace(nv=args.nv, nv_pad=args.pad,
            block_dim=SimpleNamespace(update_gradient_cholesky_blocked=32),
            opt=SimpleNamespace(solver=mt.SolverType.NEWTON))
        data = SimpleNamespace(nworld=1, nefc=wp.ones(1, dtype=int, device=device),
            qpos=SimpleNamespace(device=device))
        solver = SimpleNamespace(use_mujoco_cpu=False, _step=0, mjw_model=model, mjw_data=data)
        save()
        for implementation in args.backends:
            for done in ((False, True) if args.include_done else (False,)):
                row = dict(implementation=implementation, done=done, status='running',
                    graph_capture_and_replay_passed=False, numerically_valid=False,
                    gpu_ms_per_factor=[], graph_wall_s=[])
                report['results'].append(row); save()
                backend, stage = None, 'setup'
                try:
                    ctx = context(h, grad, args.nv, device)
                    ctx.done.fill_(done)
                    if implementation.startswith('warp'):
                        model.block_dim.update_gradient_cholesky_blocked = int(implementation[4:])
                        call = lambda: native_factor(model, data, ctx)
                    elif implementation == 'cusolver32-batched':
                        backend = CuSolverCholeskyBatch(solver, force_batched_single=True)
                        call = lambda: backend.factorize_solve(model, data, ctx)
                    else:
                        backend = CuSolverCholesky(solver,
                            precision='float64' if implementation == 'cusolver64' else 'float32')
                        call = lambda: backend.factorize_solve(model, data, ctx)
                    if backend is not None:
                        row['backend_evidence'] = backend.evidence
                    stage = 'warmup'
                    start = time.perf_counter()
                    for _ in range(3):
                        call()
                    wp.synchronize_stream(stream)
                    row['warmup_wall_s'] = time.perf_counter()-start
                    stage = 'graph_capture'
                    with wp.ScopedCapture(device=device) as capture:
                        for _ in range(args.samples):
                            call()
                    wp.capture_launch(capture.graph)
                    wp.synchronize_stream(stream)
                    stage = 'graph_replay'
                    checks, reference = [], None
                    for _ in range(args.replays):
                        start_event = wp.Event(device=device, enable_timing=True)
                        end_event = wp.Event(device=device, enable_timing=True)
                        before = time.perf_counter()
                        stream.record_event(start_event)
                        wp.capture_launch(capture.graph)
                        stream.record_event(end_event)
                        elapsed = wp.get_event_elapsed_time(start_event, end_event, synchronize=True)
                        row['graph_wall_s'].append(time.perf_counter()-before)
                        row['gpu_ms_per_factor'].append(elapsed/args.samples)
                        search = ctx.search.numpy()[0]
                        dot, decrement = float(ctx.search_dot.numpy()[0]), float(ctx.newton_decrement.numpy()[0])
                        snapshot = (search.copy(), dot, decrement)
                        if reference is None:
                            reference = snapshot
                            row['repeat_bitwise_identical'] = True
                        else:
                            row['repeat_bitwise_identical'] &= bool(
                                np.array_equal(search.view(np.uint32), reference[0].view(np.uint32))
                                and dot == reference[1] and decrement == reference[2])
                        if done:
                            valid = bool(np.all(search == 123.) and dot == 456. and decrement == 789.)
                            checks.append(dict(numerically_valid=valid, previous_search_preserved=valid))
                        else:
                            checks.append(accuracy(a, grad, oracle, search, dot, decrement, args))
                    row.update(graph_capture_and_replay_passed=True, replay_checks=checks,
                        median_gpu_ms_per_factor=float(np.median(row['gpu_ms_per_factor'])),
                        numerically_valid=all(check['numerically_valid'] for check in checks))
                    stage = 'device_health'
                    if backend is not None:
                        row.update(factor_info=backend.factor_info.numpy().tolist(),
                            solve_info=backend.solve_info.numpy().tolist(),
                            sticky_first_failure=backend.first_failure.numpy().tolist())
                        row['health'] = backend.check_health()
                    row['status'] = ('numerical_failure' if not row['numerically_valid'] else
                        'passed' if row['repeat_bitwise_identical'] else 'repeatability_failure')
                except Exception as error:
                    row.update(status='failed', numerically_valid=False, failure_stage=stage,
                        error_type=type(error).__name__, error=str(error))
                finally:
                    if backend is not None:
                        try:
                            backend.close()
                        except Exception as error:
                            row.update(status='failed', numerically_valid=False,
                                cleanup_error=f'{type(error).__name__}: {error}')
                    save()
                print(json.dumps(clean_json(row), allow_nan=False), flush=True)
        report['valid_active_ranking'] = sorted([
            dict(implementation=row['implementation'], median_gpu_ms_per_factor=row['median_gpu_ms_per_factor'])
            for row in report['results'] if row['status']=='passed' and not row['done']],
            key=lambda row: row['median_gpu_ms_per_factor'])
        report['status'] = ('completed' if all(row['status']=='passed' for row in report['results'])
                            else 'completed_with_failures')
    except Exception as error:
        report.update(status='failed', error_type=type(error).__name__, error=str(error))
    finally:
        save()
    return report


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--nv', type=int, default=169)
    parser.add_argument('--pad', type=int, default=176)
    parser.add_argument('--seed', type=int, default=31)
    parser.add_argument('--samples', type=int, default=60)
    parser.add_argument('--replays', type=int, default=5)
    parser.add_argument('--hessian', type=Path, help='Optional NPZ with native h and grad arrays')
    parser.add_argument('--diagonal-span', type=float, default=1.,
        help='Optional synthetic diagonal scaling span; saved H is never scaled')
    parser.add_argument('--max-relative-residual', type=float, default=1e-4)
    parser.add_argument('--max-relative-solution-error', type=float, default=1e-3)
    parser.add_argument('--include-done', action='store_true', help='Separate no-op controls, excluded from ranking')
    parser.add_argument('--backends', nargs='+', choices=BACKENDS, default=list(BACKENDS))
    args = parser.parse_args(argv)
    if args.samples < 1 or args.replays < 1:
        parser.error('Positive samples and replays required')
    for field in ('diagonal_span', 'max_relative_residual', 'max_relative_solution_error'):
        value = getattr(args, field)
        if not np.isfinite(value) or value <= 0:
            parser.error(field+' must be finite and positive')
    if args.diagonal_span < 1.:
        parser.error('diagonal_span must be at least one')
    return args


if __name__ == '__main__':
    raise SystemExit(0 if benchmark(arguments())['status']=='completed' else 2)
