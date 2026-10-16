"""Opt-in multi-world extension; the validated single-world module is unchanged.

N=1 delegates to its existing implementation. N>1 uses independent per-world
matrix/rhs buffers with either public cuSOLVER batched POTRF/POTRS calls or
individual GPU calls on the same stream. No state or matrix is shared between
worlds. The sequential mode shares only scratch workspace ordered by that
stream. The batched POTRS API reports one global argument status; POTRF has
one positive-definiteness status per world.
"""
from __future__ import annotations

import ctypes
from functools import lru_cache
from types import SimpleNamespace

import numpy as np
import warp as wp

import newton_gpu_cusolver as single


@lru_cache(maxsize=None)
def _prepare_many(dtype):
    @wp.kernel(module="unique", enable_backward=False)
    def prepare(nv: int, h: wp.array3d[float], grad: wp.array2d[float],
                done: wp.array[bool], changed: wp.array[int], skip_noflip: bool,
                factor: wp.array3d[dtype], rhs: wp.array2d[dtype], active: wp.array[bool]):
        world, row, col = wp.tid()
        enabled = not done[world]
        if skip_noflip:
            enabled = enabled and changed[world] != 0
        value = dtype(0.0)
        if row == col:
            value = dtype(1.0)
        if enabled and row < nv and col < nv:
            if row <= col:
                value = dtype(h[world, row, col])
            else:
                value = dtype(0.0)
        factor[world, row, col] = value
        if row == 0:
            b = dtype(0.0)
            if enabled and col < nv:
                b = dtype(grad[world, col])
            rhs[world, col] = b
            if col == 0:
                active[world] = enabled
    return prepare


@lru_cache(maxsize=None)
def _identity_many(dtype):
    @wp.kernel(module="unique", enable_backward=False)
    def identity(factor: wp.array3d[dtype], rhs: wp.array2d[dtype]):
        world, row, col = wp.tid()
        value = dtype(0.0)
        if row == col:
            value = dtype(1.0)
        factor[world, row, col] = value
        if row == 0:
            rhs[world, col] = dtype(0.0)
    return identity


@lru_cache(maxsize=None)
def _finalize_many(dtype, nv):
    @wp.kernel(module="unique", enable_backward=False)
    def finalize(grad: wp.array2d[float], rhs: wp.array2d[dtype],
                 active: wp.array[bool], factor_info: wp.array[int], solve_info: wp.array[int],
                 batched_solve_status: bool, first_failure: wp.array[int],
                 search: wp.array2d[float], search_dot: wp.array[float], decrement: wp.array[float]):
        world = wp.tid()
        factor_status = factor_info[world]
        solve_status = solve_info[world]
        if batched_solve_status:
            solve_status = solve_info[0]
        if factor_status != 0:
            wp.atomic_cas(first_failure, 2 * world, 0, factor_status)
        if solve_status != 0:
            wp.atomic_cas(first_failure, 2 * world + 1, 0, solve_status)
        if not active[world]:
            return
        if factor_status != 0 or solve_status != 0:
            search[world, 0] = wp.nan
            search_dot[world] = wp.nan
            decrement[world] = wp.nan
            return
        norm = dtype(0.0)
        dec = dtype(0.0)
        for j in range(wp.static(nv)):
            x = rhs[world, j]
            norm += x * x
            dec += dtype(grad[world, j]) * x
            search[world, j] = -float(x)
        search_dot[world] = float(norm)
        decrement[world] = float(dec)
    return finalize


def _pointer_values(base, count, stride_bytes):
    """Build immutable pointer-table contents during setup, never per step."""
    if base <= 0 or count < 1 or stride_bytes < 1:
        raise ValueError("Need a positive device base pointer, count and stride")
    values = int(base) + np.arange(count, dtype=np.uint64) * np.uint64(stride_bytes)
    if np.any(values < int(base)):
        raise OverflowError("Device pointer-table arithmetic overflow")
    return values


class CuSolverCholeskyBatch(single.CuSolverCholesky):
    """Preserve single-world behavior and add explicit multi-world GPU modes."""

    def __init__(self, solver, *, precision="float32", library_path=None, batch_mode="batched", force_batched_single=False):
        worlds = int(solver.mjw_data.nworld)
        if worlds < 1:
            raise ValueError("cuSOLVER requires a positive number of worlds")
        if batch_mode not in ("batched", "loop"):
            raise ValueError("batch_mode must be batched or loop")
        self.worlds = 1
        self.force_batched_single = False  # Parent's scratch-only initialization.
        self.batch_mode = batch_mode
        # Reuse handle, public API binding, warmup, ownership and cleanup from
        # the unchanged N=1 implementation. This setup view controls only its
        # private scratch allocation; the actual model/data are not modified.
        setup_view = SimpleNamespace(use_mujoco_cpu=solver.use_mujoco_cpu,
            _step=getattr(solver, "_step", 0), mjw_model=solver.mjw_model,
            mjw_data=SimpleNamespace(nworld=1, qpos=solver.mjw_data.qpos))
        super().__init__(setup_view, precision=precision, library_path=library_path)
        if worlds == 1 and not force_batched_single:
            self.evidence["requested_batch_mode"] = batch_mode
            self.evidence["single_world_implementation_unchanged"] = True
            return
        try:
            self.force_batched_single = bool(force_batched_single)
            self.worlds = worlds
            with wp.ScopedDevice(self.device):
                self.factor = wp.empty((worlds, self.n, self.n), dtype=self.dtype, device=self.device)
                self.rhs = wp.empty((worlds, self.n), dtype=self.dtype, device=self.device)
                self.active = wp.zeros(worlds, dtype=wp.bool, device=self.device)
                self.factor_info = wp.zeros(worlds, dtype=wp.int32, device=self.device)
                self.solve_info = wp.zeros(worlds, dtype=wp.int32, device=self.device)
                self.first_failure = wp.zeros(2 * worlds, dtype=wp.int32, device=self.device)
                if batch_mode == "batched":
                    self._configure_batched_api(precision)
                    width = 4 if precision == "float32" else 8
                    self.factor_pointers = wp.array(_pointer_values(self.factor.ptr,
                        worlds, self.n * self.n * width), dtype=wp.uint64, device=self.device)
                    self.rhs_pointers = wp.array(_pointer_values(self.rhs.ptr,
                        worlds, self.n * width), dtype=wp.uint64, device=self.device)
                for kernel in (_prepare_many(self.dtype), _identity_many(self.dtype),
                               _finalize_many(self.dtype, self.nv)):
                    wp.load_module(module=kernel.module, device=self.device)
                wp.launch(_identity_many(self.dtype), (worlds, self.n, self.n),
                    inputs=[self.factor, self.rhs], device=self.device)
                self._factor_and_solve()
                wp.synchronize_stream(self.stream)
                if np.any(self.factor_info.numpy()) or np.any(self.solve_info.numpy()):
                    raise RuntimeError("cuSOLVER multi-world identity initialization failed")
            self.evidence.update(schema="s300-newton-gpu-cusolver-batch-v1",
                world_count=worlds, batch_mode=batch_mode,
                batched_public_api=batch_mode == "batched", host_solve=False,
                per_world_factor_status=True,
                solve_status_scope="global_argument_status" if batch_mode == "batched" else "per_world",
                workspace_shared_in_stream_order=batch_mode == "loop",
                single_world_implementation_unchanged=not force_batched_single)
        except BaseException:
            super().close()
            raise

    def _configure_batched_api(self, precision):
        prefix = "S" if precision == "float32" else "D"
        p, i = ctypes.c_void_p, ctypes.c_int
        self._potrf_batched = getattr(self.api.lib, f"cusolverDn{prefix}potrfBatched")
        self._potrs_batched = getattr(self.api.lib, f"cusolverDn{prefix}potrsBatched")
        self._potrf_batched.argtypes = [p, i, i, p, i, p, i]
        self._potrs_batched.argtypes = [p, i, i, i, p, i, p, i, p, i]
        self._potrf_batched.restype = i
        self._potrs_batched.restype = i

    def _factor_and_solve(self):
        if self.worlds == 1 and not getattr(self, 'force_batched_single', False):
            return super()._factor_and_solve()
        if self.batch_mode == "batched":
            self.api.checked(self._potrf_batched(self.handle, 0, self.n,
                self.factor_pointers.ptr, self.n, self.factor_info.ptr, self.worlds), "potrfBatched")
            self.api.checked(self._potrs_batched(self.handle, 0, self.n, 1,
                self.factor_pointers.ptr, self.n, self.rhs_pointers.ptr,
                self.n, self.solve_info.ptr, self.worlds), "potrsBatched")
        else:
            width = 4 if self.dtype == wp.float32 else 8
            for world in range(self.worlds):
                factor = self.factor.ptr + world * self.n * self.n * width
                rhs = self.rhs.ptr + world * self.n * width
                self.api.factor(self.handle, self.n, factor, self.workspace.ptr,
                    self.workspace_count, self.factor_info.ptr + 4 * world)
                self.api.solve(self.handle, self.n, factor, rhs, self.solve_info.ptr + 4 * world)

    def factorize_solve(self, m, d, ctx, skip_unchanged=False, skip_noflip=False):
        if self.worlds == 1 and not getattr(self, 'force_batched_single', False):
            return super().factorize_solve(m, d, ctx, skip_unchanged, skip_noflip)
        if self.closed or m is not self.model or int(d.nworld) != self.worlds:
            raise RuntimeError("cuSOLVER batch used outside its selected model/world count")
        stream = wp.get_stream(self.device)
        if stream.cuda_stream != self.stream.cuda_stream:
            raise RuntimeError("cuSOLVER batch requires the CUDA stream used at setup")
        if ctx.h.shape != (self.worlds, self.n, self.n) or ctx.grad.shape != (self.worlds, self.n):
            raise RuntimeError("cuSOLVER batch received an incompatible solver context")
        changed = ctx.state_changed_count if skip_noflip else d.nefc
        wp.launch(_prepare_many(self.dtype), (self.worlds, self.n, self.n),
            inputs=[self.nv, ctx.h, ctx.grad, ctx.done, changed, bool(skip_noflip),
                    self.factor, self.rhs, self.active], device=self.device)
        self._factor_and_solve()
        wp.launch(_finalize_many(self.dtype, self.nv), self.worlds,
            inputs=[ctx.grad, self.rhs, self.active, self.factor_info, self.solve_info,
                    self.batch_mode == "batched", self.first_failure,
                    ctx.search, ctx.search_dot, ctx.newton_decrement], device=self.device)
        self.calls += 1

    def check_health(self):
        if self.worlds == 1 and not getattr(self, 'force_batched_single', False):
            return super().check_health()
        if wp.get_stream(self.device).is_capturing:
            raise RuntimeError("Do not read cuSOLVER health during graph capture")
        failures = self.first_failure.numpy().reshape(self.worlds, 2)
        bad = np.flatnonzero(np.any(failures != 0, axis=1))
        if len(bad):
            world = int(bad[0])
            raise RuntimeError(f"cuSOLVER device failure: world={world}, "
                f"potrf_info={int(failures[world, 0])}, potrs_info={int(failures[world, 1])}")
        return dict(potrf_info=[0] * self.worlds, potrs_info=[0] * self.worlds)
