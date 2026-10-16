"""Fixed-system CUDA-graph algebra microbenchmark; never harvests/steps physics.

The synthetic matrix has the same H=A+J.T*C*J structure, with configurable
nv/constraint size and Newton iteration count. Every virtual step rebuilds
the base factor/J cache; each Newton iteration retains the actual-H residual
check. This does not establish farm performance or GUI equivalence.
"""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import warp as wp
from mujoco_warp._src import types as mt

from bench_newton_gpu_factor import context
from newton_gpu_cusolver import CuSolverCholesky
from newton_gpu_woodbury import WoodburyCholesky


@wp.kernel(enable_backward=False)
def _next_virtual_step(time: wp.array[float]):
    time[0] += 1.


def fixture(nv, padded, rows, seed=19):
    rng = np.random.default_rng(seed)
    z = rng.normal(scale=1. / np.sqrt(nv), size=(nv, nv)).astype(np.float32)
    A = (z @ z.T + np.eye(nv, dtype=np.float32) * .5).astype(np.float32)
    J = rng.normal(scale=.2, size=(rows, nv)).astype(np.float32)
    diagonal = rng.uniform(.1, 10., rows).astype(np.float32)
    H = A.astype(np.float64) + J.astype(np.float64).T @ (diagonal[:, None] * J).astype(np.float64)
    h = np.zeros((padded, padded), np.float32); h[:nv, :nv] = H
    gradient = rng.normal(size=padded).astype(np.float32)
    index = np.full((nv, nv), -1, np.int32)
    sparse = []
    for i in range(nv):
        for j in range(i + 1):
            index[i, j] = len(sparse); sparse.append(A[i, j])
    def arr(a, dtype=None):
        return wp.array(a, dtype=dtype, device="cuda:0")
    model = SimpleNamespace(nv=nv, nv_pad=padded, is_sparse=True,
        nflex=0, ntendon=0, nefmK=0, has_flex_passive=False,
        has_non_simple_flex=False, efm0_active=False, has_fluid=False,
        nmaxcondim=3, M_elemid=arr(index, int),
        opt=SimpleNamespace(solver=mt.SolverType.NEWTON,
            integrator=mt.IntegratorType.DISCRETE, cone=mt.ConeType.ELLIPTIC,
            enableflags=0, impratio_invsqrt=arr([1.], float)))
    data = SimpleNamespace(nworld=1, nefc=arr([rows], int),
        qpos=wp.zeros((1, nv), dtype=float, device="cuda:0"),
        time=arr([0.], float), moment_rownnz=wp.empty((1, 0), dtype=int, device="cuda:0"),
        qH=arr(np.array(sparse, np.float32)[None], float),
        efc=SimpleNamespace(J_rownnz=arr(np.full((1, rows), nv, np.int32), int),
            J_rowadr=arr((np.arange(rows) * nv)[None], int),
            J_colind=arr(np.tile(np.arange(nv), rows).reshape(1, 1, -1), int),
            J=arr(J.reshape(1, 1, -1), float), D=arr(diagonal[None], float),
            state=arr(np.ones((1, rows), np.int32), int),
            type=arr(np.zeros((1, rows), np.int32), int),
            id=arr(np.zeros((1, rows), np.int32), int)),
        contact=SimpleNamespace(efc_address=arr([[0]], int), dim=arr([3], int),
            friction=arr([[.5, .5, .01, .01, .01]], mt.vec5)))
    solver = SimpleNamespace(use_mujoco_cpu=False, _step=0, mjw_model=model, mjw_data=data)
    ctx = context(h, gradient, nv, "cuda:0")
    ctx.Jaref = wp.zeros((1, rows), dtype=float, device="cuda:0")
    return solver, ctx, H, gradient[:nv], h


def benchmark(args):
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with wp.ScopedDevice("cuda:0"):
        solver, ctx, H, gradient, h = fixture(args.nv, args.pad, args.constraints)
        stream = wp.get_stream("cuda:0")
        result = dict(schema="s300-woodbury-synthetic-algebra-benchmark-v1", simulation_steps=0,
            status="running",
            farm_physics_verified=False, gui_equivalence_verified=False, synthetic=True,
            nv=args.nv, padded=args.pad, constraints=args.constraints,
            rank_capacity=args.capacity, newton_iterations_per_virtual_step=args.iterations,
            virtual_steps_per_graph=args.samples, repeats=args.replays,
            h_sha256=hashlib.sha256(h.tobytes()).hexdigest(),
            code_sha256={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                for name in ("newton_gpu_woodbury.py", "newton_gpu_cusolver.py", Path(__file__).name)},
            results=[])
        for name, factory in [("full_cusolver32", lambda: CuSolverCholesky(solver)),
                ("woodbury", lambda: WoodburyCholesky(solver, rank_capacity=args.capacity))]:
            with factory() as backend:
                def virtual_step():
                    wp.launch(_next_virtual_step, 1, inputs=[solver.mjw_data.time], device="cuda:0")
                    for _ in range(args.iterations):
                        backend.factorize_solve(solver.mjw_model, solver.mjw_data, ctx)
                virtual_step(); wp.synchronize_stream(stream)
                with wp.ScopedCapture(device="cuda:0") as cap:
                    for _ in range(args.samples):
                        virtual_step()
                wp.capture_launch(cap.graph); wp.synchronize_stream(stream)
                times = []
                for _ in range(args.replays):
                    start = wp.Event(device="cuda:0", enable_timing=True)
                    end = wp.Event(device="cuda:0", enable_timing=True)
                    stream.record_event(start)
                    wp.capture_launch(cap.graph)
                    stream.record_event(end)
                    times.append(wp.get_event_elapsed_time(start, end, synchronize=True) / args.samples)
                backend.check_health()
                x = -ctx.search.numpy()[0].astype(np.float64)
                row = dict(implementation=name, gpu_ms_per_virtual_step=times,
                    median_gpu_ms_per_virtual_step=float(np.median(times)),
                    relative_actual_hessian_residual=float(np.linalg.norm(H @ x - gradient) / np.linalg.norm(gradient)),
                    graph_capture_passed=True, backend=backend.evidence)
                if hasattr(backend, "statistics"):
                    row["counters"] = backend.statistics()
                result["results"].append(row)
                output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
                print(json.dumps(row), flush=True)
        result["status"] = "complete"
        output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True)
    p.add_argument("--nv", type=int, default=385)
    p.add_argument("--pad", type=int, default=400)
    p.add_argument("--constraints", type=int, default=64)
    p.add_argument("--capacity", type=int, default=128)
    p.add_argument("--iterations", type=int, default=10)
    p.add_argument("--samples", type=int, default=8)
    p.add_argument("--replays", type=int, default=5)
    args = p.parse_args()
    try:
        benchmark(args)
    except FileExistsError:
        raise
    except BaseException as exc:
        path = Path(args.output)
        if path.exists():
            partial = json.loads(path.read_text())
            partial.update(status="error", error=repr(exc))
            path.write_text(json.dumps(partial, indent=2, allow_nan=False) + "\n")
        raise
