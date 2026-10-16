"""CUDA-graph microbenchmark of J.T*C*J only; no physics steps or harvest claim."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import warp as wp

from newton_gpu_dense_hessian import DenseConstraintHessian
from test_newton_gpu_dense_hessian import cuda_fixture


def benchmark(args):
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with wp.ScopedDevice("cuda:0"):
        solver, ctx, init, native = cuda_fixture(args.constraints, args.width)
        m, d = solver.mjw_model, solver.mjw_data
        kernel = __import__("mujoco_warp._src.solver", fromlist=["_JTDACJ_sparse"])._JTDACJ_sparse(False, m.opt.cone, 3)
        original_options = wp.get_module_options(kernel.module).copy()
        init(); native()
        reference = ctx.h.numpy().copy()
        report = dict(schema="s300-dense-hessian-synthetic-benchmark-v1", status="running",
            synthetic=True, physics_steps=0, farm_rtf=None, nv=m.nv,
            constraint_rows=args.constraints, sparse_row_width=args.width,
            allocated_constraint_capacity=d.njmax, dense_capacity=args.capacity,
            newton_iterations_per_virtual_step=args.iterations,
            code_sha256={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                for name in ("newton_gpu_dense_hessian.py", "newton_gpu_woodbury.py", Path(__file__).name)},
            results=[])
        try:
            for name in ("native", "native_deterministic", "dense_blas"):
                selected = name != "native"
                bound = ((args.constraints + 31) // 32) * ((args.width * (args.width + 1) // 2 + 31) // 32)
                wp.set_module_options(dict(deterministic=wp.DeterministicMode.RUN_TO_RUN if selected
                    else wp.DeterministicMode.NOT_GUARANTEED,
                    deterministic_max_records=bound if selected else 0), module=kernel.module)
                backend = DenseConstraintHessian(solver, capacity=args.capacity) if name == "dense_blas" else None
                def call():
                    init()
                    if backend:
                        backend.assemble_or_native(m, d, ctx, native)
                    else:
                        native()
                try:
                    call(); wp.synchronize_stream(wp.get_stream())
                    with wp.ScopedCapture(device="cuda:0") as capture:
                        for _ in range(args.iterations):
                            call()
                    wp.capture_launch(capture.graph); wp.synchronize_stream(wp.get_stream())
                    timings = []
                    for _ in range(args.replays):
                        start = wp.Event(device="cuda:0", enable_timing=True)
                        end = wp.Event(device="cuda:0", enable_timing=True)
                        wp.get_stream().record_event(start)
                        wp.capture_launch(capture.graph)
                        wp.get_stream().record_event(end)
                        timings.append(wp.get_event_elapsed_time(start, end, synchronize=True))
                    actual = ctx.h.numpy().copy()
                    delta = np.triu(actual[0] - reference[0])
                    result = dict(implementation=name, gpu_ms_per_virtual_step=timings,
                        median_gpu_ms_per_virtual_step=float(np.median(timings)),
                        maximum_upper_absolute_error=float(np.max(np.abs(delta))),
                        relative_upper_norm_error=float(np.linalg.norm(delta) / np.linalg.norm(np.triu(reference[0]))),
                        graph_capture_passed=True, native_deterministic_record_bound=bound if selected else 0)
                    if backend:
                        result.update(backend=backend.evidence, counters=backend.statistics())
                    report["results"].append(result)
                    output.write_text(json.dumps(report, indent=2) + "\n")
                    print(json.dumps(result), flush=True)
                finally:
                    if backend:
                        backend.close()
            report["status"] = "complete"
        except BaseException as exc:
            report.update(status="error", error=repr(exc))
            raise
        finally:
            wp.set_module_options(original_options, module=kernel.module)
            output.write_text(json.dumps(report, indent=2) + "\n")
        return report


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True)
    p.add_argument("--constraints", type=int, default=64)
    p.add_argument("--width", type=int, default=93)
    p.add_argument("--capacity", type=int, default=128)
    p.add_argument("--iterations", type=int, default=10)
    p.add_argument("--replays", type=int, default=5)
    benchmark(p.parse_args())
