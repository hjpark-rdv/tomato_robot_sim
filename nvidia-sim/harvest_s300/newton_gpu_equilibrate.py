"""Opt-in diagonal congruence equilibration for the existing GPU FP32 solve.

S=diag(H)^(-1/2), (S H S)y=Sg, x=Sy. This solves the same SPD
equations; it does not change H, g, contacts, tolerance or model parameters.
Scaling products use float64 intermediate arithmetic before storing the
normalized matrix/rhs in float32; cuSOLVER still factors/solves on the GPU
in float32. This is a numerical experiment, not proof of physical accuracy.
"""
from __future__ import annotations

from functools import lru_cache

import numpy as np
import warp as wp

from newton_gpu_cusolver import CuSolverCholesky


@wp.kernel(enable_backward=False)
def _scales(nv: int, h: wp.array3d[float], grad: wp.array2d[float],
            done: wp.array[bool], changed: wp.array[int], skip_noflip: bool,
            scale: wp.array[wp.float64], bad_diagonal: wp.array[int], bad_matrix: wp.array[int]):
    j = wp.tid()
    enabled = not done[0]
    if skip_noflip:
        enabled = enabled and changed[0] != 0
    value = wp.float64(1.0)
    if enabled and j < nv:
        diagonal = h[0, j, j]
        if not wp.isfinite(diagonal) or diagonal <= 0.0:
            wp.atomic_min(bad_diagonal, 0, j)
            value = wp.float64(wp.nan)
        else:
            value = wp.float64(1.0) / wp.sqrt(wp.float64(diagonal))
        if not wp.isfinite(grad[0, j]):
            wp.atomic_min(bad_matrix, 0, j)
    scale[j] = value


@wp.kernel(enable_backward=False)
def _prepare_equilibrated(nv: int, h: wp.array3d[float], grad: wp.array2d[float],
            done: wp.array[bool], changed: wp.array[int], skip_noflip: bool,
            scale: wp.array[wp.float64], factor: wp.array2d[float],
            rhs: wp.array[float], active: wp.array[bool], bad_matrix: wp.array[int]):
    row, col = wp.tid()
    enabled = not done[0]
    if skip_noflip:
        enabled = enabled and changed[0] != 0
    value = float(0.0)
    if row == col:
        value = float(1.0)
    if enabled and row < nv and col < nv:
        if row <= col:
            original = h[0, row, col]
            if not wp.isfinite(original):
                wp.atomic_min(bad_matrix, 0, row * factor.shape[1] + col)
            value = float(wp.float64(original) * scale[row] * scale[col])
        else:
            value = float(0.0)
    factor[row, col] = value
    if row == 0:
        b = float(0.0)
        if enabled and col < nv:
            b = float(wp.float64(grad[0, col]) * scale[col])
        rhs[col] = b
        if col == 0:
            active[0] = enabled


@lru_cache(maxsize=None)
def _finalize_equilibrated(nv):
    @wp.kernel(module="unique", enable_backward=False)
    def finalize(grad: wp.array2d[float], rhs: wp.array[float], scale: wp.array[wp.float64],
                 active: wp.array[bool], factor_info: wp.array[int], solve_info: wp.array[int],
                 first_failure: wp.array[int], bad_diagonal: wp.array[int], bad_matrix: wp.array[int],
                 search: wp.array2d[float], search_dot: wp.array[float], decrement: wp.array[float]):
        if factor_info[0] != 0:
            wp.atomic_cas(first_failure, 0, 0, factor_info[0])
        if solve_info[0] != 0:
            wp.atomic_cas(first_failure, 1, 0, solve_info[0])
        if not active[0]:
            return
        if (factor_info[0] != 0 or solve_info[0] != 0
                or bad_diagonal[0] != 2147483647 or bad_matrix[0] != 2147483647):
            search[0, 0] = wp.nan
            search_dot[0] = wp.nan
            decrement[0] = wp.nan
            return
        norm = float(0.0)
        dec = float(0.0)
        for j in range(wp.static(nv)):
            # Recover the original-coordinate search and original gradient
            # products, not the norm/decrement of the scaled coordinate y.
            x = float(wp.float64(rhs[j]) * scale[j])
            search[0, j] = -x
            norm += x * x
            dec += grad[0, j] * x
        search_dot[0] = norm
        decrement[0] = dec
    return finalize


class EquilibratedCuSolverCholesky(CuSolverCholesky):
    """N=1 FP32 algebra experiment with no changes to the base implementation."""
    def __init__(self, solver, *, precision="float32", library_path=None):
        if precision != "float32":
            raise ValueError("Equilibration experiment currently requires float32 cuSOLVER")
        super().__init__(solver, precision=precision, library_path=library_path)
        try:
            with wp.ScopedDevice(self.device):
                self.scale = wp.empty(self.n, dtype=wp.float64, device=self.device)
                self.bad_diagonal = wp.array([np.iinfo(np.int32).max], dtype=wp.int32, device=self.device)
                self.bad_matrix = wp.array([np.iinfo(np.int32).max], dtype=wp.int32, device=self.device)
                for kernel in (_scales, _prepare_equilibrated, _finalize_equilibrated(self.nv)):
                    wp.load_module(module=kernel.module, device=self.device)
            self.evidence.update(schema="s300-newton-gpu-equilibrated-cusolver-v1",
                diagonal_congruence=True, scaling_formula="S=diag(H)^(-1/2); H'=SHS; b'=Sg; x=Sy",
                source_hessian_and_gradient_modified=False, diagonal_jitter_or_clamp=False,
                scaling_intermediate_precision="float64", factor_and_solve_precision="float32",
                original_coordinate_norm_and_decrement=True,
                positive_finite_diagonal_required=True)
        except BaseException:
            super().close()
            raise

    def factorize_solve(self, m, d, ctx, skip_unchanged=False, skip_noflip=False):
        if self.closed or m is not self.model or int(d.nworld) != 1:
            raise RuntimeError("Equilibrated cuSOLVER used outside selected model/world")
        if wp.get_stream(self.device).cuda_stream != self.stream.cuda_stream:
            raise RuntimeError("Equilibrated cuSOLVER requires the setup CUDA stream")
        if ctx.h.shape != (1, self.n, self.n) or ctx.grad.shape != (1, self.n):
            raise RuntimeError("Equilibrated cuSOLVER received an incompatible solver context")
        changed = ctx.state_changed_count if skip_noflip else d.nefc
        wp.launch(_scales, self.n, inputs=[self.nv, ctx.h, ctx.grad, ctx.done, changed,
            bool(skip_noflip), self.scale, self.bad_diagonal, self.bad_matrix], device=self.device)
        wp.launch(_prepare_equilibrated, (self.n, self.n),
            inputs=[self.nv, ctx.h, ctx.grad, ctx.done, changed, bool(skip_noflip),
                    self.scale, self.factor, self.rhs, self.active, self.bad_matrix], device=self.device)
        self._factor_and_solve()
        wp.launch(_finalize_equilibrated(self.nv), 1,
            inputs=[ctx.grad, self.rhs, self.scale, self.active, self.factor_info, self.solve_info,
                    self.first_failure, self.bad_diagonal, self.bad_matrix,
                    ctx.search, ctx.search_dot, ctx.newton_decrement], device=self.device)
        self.calls += 1

    def check_health(self):
        if wp.get_stream(self.device).is_capturing:
            raise RuntimeError("Do not read equilibration health during graph capture")
        diagonal = int(self.bad_diagonal.numpy()[0])
        matrix = int(self.bad_matrix.numpy()[0])
        if diagonal != np.iinfo(np.int32).max or matrix != np.iinfo(np.int32).max:
            raise RuntimeError(f"Equilibration invalid source: diagonal_dof={diagonal}, matrix_or_gradient_index={matrix}")
        return super().check_health()
