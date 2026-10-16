"""Synthetic captured qfrc+gradient benchmark; no physics step or RTF claim."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import warp as wp
from mujoco_warp._src import solver as ms

from test_newton_gpu_fixed_reductions import fixture, qfrc, gradient


def benchmark(args):
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with wp.ScopedDevice("cuda:0"):
        d, ctx, failure, counts = fixture("cuda:0", 385, 80, 93, descending=True)
        kernel = ms._update_constraint_init_qfrc_constraint_sparse(False)
        grad_kernel = ms._update_gradient_grad(False)
        options = {k.module: wp.get_module_options(k.module).copy() for k in (kernel, grad_kernel)}
        report = dict(schema="s300-fixed-reductions-synthetic-benchmark-v1", status="running",
            nv=385, constraint_rows=80, sparse_width=93, iterations=args.iterations,
            sparse_column_order="descending, matching native farm snapshots",
            synthetic=True, physics_steps=0, farm_rtf=None,
            code_sha256={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                for name in ("newton_gpu_fixed_reductions.py", Path(__file__).name)}, results=[])
        reference = None
        try:
            for name in ("native", "native_deterministic", "fixed"):
                fixed = name == "fixed"
                for k in (kernel, grad_kernel):
                    wp.set_module_options(dict(deterministic=wp.DeterministicMode.RUN_TO_RUN
                        if name == "native_deterministic" else wp.DeterministicMode.NOT_GUARANTEED,
                        deterministic_max_records=93 if k is kernel and name == "native_deterministic"
                        else 1 if name == "native_deterministic" else 0), module=k.module)
                def call():
                    qfrc(d, ctx, failure, counts, fixed)
                    gradient(d, ctx, fixed, block_dim=256)
                call(); wp.synchronize_stream(wp.get_stream())
                if reference is None:
                    reference = d.qfrc_constraint.numpy(), ctx.grad.numpy(), ctx.grad_dot.numpy()
                with wp.ScopedCapture(device="cuda:0") as capture:
                    for _ in range(args.iterations):
                        call()
                wp.capture_launch(capture.graph); wp.synchronize_stream(wp.get_stream())
                times = []
                for _ in range(5):
                    start = wp.Event(device="cuda:0", enable_timing=True)
                    end = wp.Event(device="cuda:0", enable_timing=True)
                    wp.get_stream().record_event(start); wp.capture_launch(capture.graph)
                    wp.get_stream().record_event(end)
                    times.append(wp.get_event_elapsed_time(start, end, synchronize=True))
                actual = d.qfrc_constraint.numpy(), ctx.grad.numpy(), ctx.grad_dot.numpy()
                row = dict(implementation=name, gpu_ms_per_ten_reductions=times,
                    median_gpu_ms=float(np.median(times)), graph_capture_passed=True,
                    max_abs_qfrc_difference=float(np.max(np.abs(actual[0] - reference[0]))),
                    max_abs_gradient_difference=float(np.max(np.abs(actual[1] - reference[1]))),
                    relative_squared_norm_difference=float(np.max(np.abs(actual[2] - reference[2])) /
                                                          max(1.e-30, np.max(np.abs(reference[2])))))
                if fixed:
                    row["csr_guard_failure"] = int(failure.numpy()[0])
                report["results"].append(row)
                output.write_text(json.dumps(report, indent=2) + "\n")
                print(json.dumps(row), flush=True)
            report["status"] = "complete"
        except BaseException as exc:
            report.update(status="error", error=repr(exc)); raise
        finally:
            for module, original in options.items():
                wp.set_module_options(original, module=module)
            output.write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True)
    p.add_argument("--iterations", type=int, default=10)
    benchmark(p.parse_args())
