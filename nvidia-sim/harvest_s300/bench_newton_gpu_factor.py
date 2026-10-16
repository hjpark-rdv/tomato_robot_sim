"""Diagnostic GPU factor/solve microbenchmark; this is not a harvest executor.

Measures complete search-direction formation on one fixed SPD system using
CUDA events around replayed CUDA graphs. Never advances simulation physics.
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
from mujoco_warp._src import solver as mw_solver
from mujoco_warp._src import types as mt

from newton_gpu_cusolver import CuSolverCholesky


def system(nv, n, seed):
    rng = np.random.default_rng(seed)
    a = rng.normal(size=(nv, nv)).astype(np.float32) / np.sqrt(nv)
    a = (a @ a.T + np.eye(nv, dtype=np.float32) * 2).astype(np.float32)
    h = np.zeros((n, n), np.float32)
    h[:nv, :nv] = np.triu(a)
    h[np.arange(nv, n), np.arange(nv, n)] = 1.
    g = np.r_[rng.normal(size=nv), np.zeros(n - nv)].astype(np.float32)
    return a, h, g


def context(h, grad, nv, device):
    n = len(grad)
    return SimpleNamespace(h=wp.array(h[None], dtype=wp.float32, device=device),
        hfactor=wp.empty((1, n, n), dtype=wp.float32, device=device),
        grad=wp.array(grad[None], dtype=wp.float32, device=device),
        done=wp.zeros(1, dtype=wp.bool, device=device),
        state_changed_count=wp.ones(1, dtype=wp.int32, device=device),
        quad_changed_count=wp.ones(1, dtype=wp.int32, device=device),
        search=wp.full((1, nv), 123., dtype=wp.float32, device=device),
        search_dot=wp.full(1, 456., dtype=wp.float32, device=device),
        newton_decrement=wp.full(1, 789., dtype=wp.float32, device=device))


def benchmark(args):
    out = Path(args.output).absolute()
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite: {out}")
    out.parent.mkdir(parents=True, exist_ok=True)
    if (args.nv <= 32 or args.pad < args.nv or args.pad % mt.TILE_SIZE_JTDAJ_DENSE
            or args.samples < 1 or args.replays < 1):
        raise ValueError("Need nv>32, valid tile-padded size and positive samples/replays")
    device = wp.get_device(args.device)
    a, h, grad = system(args.nv, args.pad, args.seed)
    if args.hessian:
        with np.load(args.hessian) as loaded:
            h = np.asarray(loaded["h"], np.float32).reshape(args.pad, args.pad)
            grad = np.asarray(loaded["grad"], np.float32).reshape(args.pad)
        upper = np.triu(h[:args.nv, :args.nv])
        a = upper + upper.T - np.diag(upper.diagonal())
    oracle = np.linalg.solve(a.astype(np.float64), grad[:args.nv].astype(np.float64))
    model = SimpleNamespace(nv=args.nv, nv_pad=args.pad,
        block_dim=SimpleNamespace(update_gradient_cholesky_blocked=args.block_dim),
        opt=SimpleNamespace(solver=mt.SolverType.NEWTON))
    data = SimpleNamespace(nworld=1, nefc=wp.ones(1, dtype=wp.int32, device=device),
        qpos=SimpleNamespace(device=device))
    solver = SimpleNamespace(use_mujoco_cpu=False, _step=0, mjw_model=model, mjw_data=data)
    original = mw_solver._cholesky_factorize_solve
    report = dict(schema="s300-newton-spd-microbenchmark-v1", simulation_steps=0,
        harvest_realtime_factor=None, nv=args.nv, pad=args.pad, seed=args.seed,
        input_h_sha256=hashlib.sha256(h.tobytes()).hexdigest(),
        input_grad_sha256=hashlib.sha256(grad.tobytes()).hexdigest(),
        synthetic_hessian=not bool(args.hessian), samples_per_graph=args.samples,
        graph_replays=args.replays, device=str(device), block_dim=args.block_dim,
        condition_number=float(np.linalg.cond(a.astype(np.float64))),
        package_versions={name: importlib.metadata.version(name) for name in
                          ("warp-lang", "mujoco-warp", "newton")},
        code_sha256={path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in
                     (Path(__file__), Path(__file__).with_name("newton_gpu_cusolver.py"))},
        stock_library_file_sha256=hashlib.sha256(Path(mw_solver.__file__).read_bytes()).hexdigest(),
        results=[])
    stream = wp.get_stream(device)
    for implementation in args.backends:
        ctx = context(h, grad, args.nv, device)
        backend = None
        try:
            if implementation != "warp":
                backend = CuSolverCholesky(solver,
                    precision="float64" if implementation == "cusolver64" else "float32")
                report.setdefault("cusolver_libraries", {})[implementation] = backend.api.path
                call = lambda: backend.factorize_solve(model, data, ctx)
            else:
                call = lambda: original(model, data, ctx)
            for done in (False, True):
                ctx.done.fill_(done)
                ctx.search.fill_(123.)
                ctx.search_dot.fill_(456.)
                ctx.newton_decrement.fill_(789.)
                init = time.perf_counter()
                for _ in range(3):
                    call()
                wp.synchronize_stream(stream)
                warmup_wall = time.perf_counter() - init
                with wp.ScopedCapture(device=device) as capture:
                    for _ in range(args.samples):
                        call()
                wp.capture_launch(capture.graph)
                wp.synchronize_stream(stream)
                times = []
                walls = []
                for _ in range(args.replays):
                    start = wp.Event(device=device, enable_timing=True)
                    end = wp.Event(device=device, enable_timing=True)
                    before = time.perf_counter()
                    stream.record_event(start)
                    wp.capture_launch(capture.graph)
                    stream.record_event(end)
                    elapsed = wp.get_event_elapsed_time(start, end, synchronize=True)
                    walls.append(time.perf_counter() - before)
                    times.append(elapsed / args.samples)
                if backend:
                    backend.check_health()
                search = ctx.search.numpy()[0]
                row = dict(implementation=implementation, done=done,
                    gpu_ms_per_factor=times, median_gpu_ms_per_factor=float(np.median(times)),
                    graph_wall_s=walls, warmup_wall_s=warmup_wall,
                    graph_capture_and_replay_passed=True)
                if done:
                    row["previous_search_preserved"] = bool(np.all(search == 123.)
                        and ctx.search_dot.numpy()[0] == 456. and ctx.newton_decrement.numpy()[0] == 789.)
                else:
                    solution = -search.astype(np.float64)
                    row.update(relative_residual=float(np.linalg.norm(a.astype(np.float64) @ solution - grad[:args.nv])
                        / np.linalg.norm(grad[:args.nv])),
                        relative_solution_error=float(np.linalg.norm(solution - oracle) / np.linalg.norm(oracle)),
                        search_dot=float(ctx.search_dot.numpy()[0]),
                        newton_decrement=float(ctx.newton_decrement.numpy()[0]))
                report["results"].append(row)
                out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
                print(json.dumps(row), flush=True)
        finally:
            if backend:
                backend.close()
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True)
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--nv", type=int, default=385)
    p.add_argument("--pad", type=int, default=400)
    p.add_argument("--seed", type=int, default=31)
    p.add_argument("--samples", type=int, default=120)
    p.add_argument("--replays", type=int, default=5)
    p.add_argument("--block-dim", type=int, default=128)
    p.add_argument("--hessian", help="Optional saved NPZ with h(pad,pad) and grad(pad)")
    p.add_argument("--backends", nargs="+", choices=["warp", "cusolver32", "cusolver64"],
                   default=["warp", "cusolver32", "cusolver64"])
    args = p.parse_args()
    benchmark(args)


if __name__ == "__main__":
    main()
