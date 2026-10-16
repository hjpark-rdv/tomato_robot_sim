"""Opt-in GPU cuSOLVER search solve for one MuJoCo Warp model and world.

This replaces only the dense SPD factor/solve inside the existing Newton
constraint solver.  The Hessian, gradient, line search, convergence criteria,
contacts, integrator and physical model are unchanged.  Results need not be
bit-identical to Warp's tiled float32 factorization.

Install before stepping/capturing.  Keep the returned object (and its device
workspace) alive until all captured graphs have finished executing.  External
cuSOLVER calls use Warp's current CUDA stream and preallocated device memory.
There is no CPU solve or device-to-host transfer in ``factorize_solve``.
"""
from __future__ import annotations

import ctypes
import ctypes.util
from functools import lru_cache
import glob
from pathlib import Path

import warp as wp
from mujoco_warp._src import solver as mw_solver
from mujoco_warp._src import types as mw_types


def find_cusolver_library(path=None):
    """Find CUDA 12's cuSOLVER ABI 11, or honor an explicit shared library."""
    if path is not None:
        candidate = Path(path).expanduser()
        if not candidate.is_file():
            raise FileNotFoundError(f"cuSOLVER library does not exist: {candidate}")
        return str(candidate.absolute())
    candidates = sorted(glob.glob(
        "/usr/local/lib/python*/dist-packages/nvidia/cusolver/lib/libcusolver.so.11"),
        reverse=True)
    candidates += sorted(glob.glob(
        "/root/isaaclab_env/lib/python*/site-packages/nvidia/cusolver/lib/libcusolver.so.11"))
    candidates += glob.glob(
        "/root/isaac-sim-*/exts/isaacsim.pip.nv/pip_prebundle/nvidia/cusolver/lib/libcusolver.so.11")
    for candidate in candidates:
        if Path(candidate).is_file():
            return str(Path(candidate).absolute())
    candidate = ctypes.util.find_library("cusolver")
    if candidate:
        return candidate
    raise FileNotFoundError("cuSOLVER is unavailable; pass an installed libcusolver.so path")


class CuSolverAPI:
    """Small typed binding to the public legacy S/D POTRF/POTRS C API."""

    def __init__(self, library_path=None, precision="float32"):
        if precision not in ("float32", "float64"):
            raise ValueError("precision must be float32 or float64")
        self.path = find_cusolver_library(library_path)
        self.lib = ctypes.CDLL(self.path)
        self.precision = precision
        p, i = ctypes.c_void_p, ctypes.c_int
        specs = {
            "cusolverDnCreate": [ctypes.POINTER(p)],
            "cusolverDnDestroy": [p],
            "cusolverDnSetStream": [p, p],
            "cusolverDnSetDeterministicMode": [p, i],
            "cusolverDnGetDeterministicMode": [p, ctypes.POINTER(i)],
        }
        prefix = "S" if precision == "float32" else "D"
        specs[f"cusolverDn{prefix}potrf_bufferSize"] = [p, i, i, p, i, ctypes.POINTER(i)]
        specs[f"cusolverDn{prefix}potrf"] = [p, i, i, p, i, p, i, p]
        specs[f"cusolverDn{prefix}potrs"] = [p, i, i, i, p, i, p, i, p]
        for name, args in specs.items():
            function = getattr(self.lib, name)
            function.argtypes = args
            function.restype = i
        self._buffer_size = getattr(self.lib, f"cusolverDn{prefix}potrf_bufferSize")
        self._potrf = getattr(self.lib, f"cusolverDn{prefix}potrf")
        self._potrs = getattr(self.lib, f"cusolverDn{prefix}potrs")

    @staticmethod
    def checked(status, operation):
        if int(status) != 0:
            raise RuntimeError(f"cuSOLVER {operation} failed with status {int(status)}")

    def create(self):
        handle = ctypes.c_void_p()
        self.checked(self.lib.cusolverDnCreate(ctypes.byref(handle)), "create")
        return handle

    def destroy(self, handle):
        self.checked(self.lib.cusolverDnDestroy(handle), "destroy")

    def set_stream(self, handle, stream):
        self.checked(self.lib.cusolverDnSetStream(handle, ctypes.c_void_p(stream or 0)), "set_stream")

    def deterministic_mode(self,handle):
        mode=ctypes.c_int()
        self.checked(self.lib.cusolverDnGetDeterministicMode(handle,ctypes.byref(mode)),
                     "get_deterministic_mode")
        return int(mode.value)

    def set_deterministic(self,handle):
        self.checked(self.lib.cusolverDnSetDeterministicMode(handle,1),"set_deterministic_mode")
        if self.deterministic_mode(handle)!=1:
            raise RuntimeError('cuSOLVER did not enable deterministic results')

    def buffer_size(self, handle, n, factor_ptr):
        count = ctypes.c_int()
        # Row-major upper triangle is the column-major lower triangle.
        self.checked(self._buffer_size(handle, 0, n, factor_ptr, n,
                                       ctypes.byref(count)), "potrf_bufferSize")
        if count.value < 0:
            raise RuntimeError("cuSOLVER returned a negative workspace size")
        return count.value

    def factor(self, handle, n, factor_ptr, workspace_ptr, count, info_ptr):
        self.checked(self._potrf(handle, 0, n, factor_ptr, n, workspace_ptr,
                                 count, info_ptr), "potrf")

    def solve(self, handle, n, factor_ptr, rhs_ptr, info_ptr):
        self.checked(self._potrs(handle, 0, n, 1, factor_ptr, n,
                                 rhs_ptr, n, info_ptr), "potrs")


@lru_cache(maxsize=None)
def _prepare_kernel(dtype):
    @wp.kernel(module="unique", enable_backward=False)
    def prepare(nv: int, h: wp.array3d[float], grad: wp.array2d[float],
                done: wp.array[bool], changed: wp.array[int], skip_noflip: bool,
                factor: wp.array2d[dtype], rhs: wp.array[dtype],
                active: wp.array[bool]):
        row, col = wp.tid()
        enabled = not done[0]
        if skip_noflip:
            enabled = enabled and changed[0] != 0
        value = dtype(0.0)
        if row == col:
            value = dtype(1.0)
        if enabled and row < nv and col < nv:
            if row <= col:
                value = dtype(h[0, row, col])
            else:
                value = dtype(0.0)
        factor[row, col] = value
        if row == 0:
            b = dtype(0.0)
            if enabled and col < nv:
                b = dtype(grad[0, col])
            rhs[col] = b
            if col == 0:
                active[0] = enabled
    return prepare


@lru_cache(maxsize=None)
def _identity_kernel(dtype):
    @wp.kernel(module="unique", enable_backward=False)
    def identity(factor: wp.array2d[dtype], rhs: wp.array[dtype]):
        row, col = wp.tid()
        value = dtype(0.0)
        if row == col:
            value = dtype(1.0)
        factor[row, col] = value
        if row == 0:
            rhs[col] = dtype(0.0)
    return identity


@lru_cache(maxsize=None)
def _finalize_kernel(dtype, nv, n):
    @wp.kernel(module="unique", enable_backward=False, grid_stride=False)
    def finalize(grad: wp.array2d[float], rhs: wp.array[dtype],
                 active: wp.array[bool], factor_info: wp.array[int], solve_info: wp.array[int],
                 first_failure: wp.array[int], search: wp.array2d[float],
                 search_dot: wp.array[float], decrement: wp.array[float]):
        if factor_info[0] != 0:
            wp.atomic_cas(first_failure, 0, 0, factor_info[0])
        if solve_info[0] != 0:
            wp.atomic_cas(first_failure, 1, 0, solve_info[0])
        if not active[0]:
            return
        if factor_info[0] != 0 or solve_info[0] != 0:
            # Existing finite-state/qacc guards must reject a failed solve even
            # if the caller has not yet read our detailed device status.
            search[0, 0] = wp.nan
            search_dot[0] = wp.nan
            decrement[0] = wp.nan
            return
        # Use an ordinary loop: 400 multiply-adds on one thread are cheap
        # compared to the full factorization; no atomics/reduction ordering.
        norm = dtype(0.0)
        dec = dtype(0.0)
        for j in range(wp.static(nv)):
            x = rhs[j]
            norm += x * x
            dec += dtype(grad[0, j]) * x
            search[0, j] = -float(x)
        search_dot[0] = float(norm)
        decrement[0] = float(dec)
    return finalize


_installed_backend = None


def _dispatch(m, d, ctx, skip_unchanged=False, skip_noflip=False):
    backend = _installed_backend
    if backend is not None and m is backend.model:
        return backend.factorize_solve(m, d, ctx, skip_unchanged, skip_noflip)
    if backend is None:
        raise RuntimeError("cuSOLVER dispatch outlived its installed backend")
    return backend.original(m, d, ctx, skip_unchanged, skip_noflip)


def validate_solver(solver, precision):
    """Host-only preconditions, checked before loading/initializing CUDA."""
    if precision not in ("float32", "float64"):
        raise ValueError("precision must be float32 or float64")
    if getattr(solver, "use_mujoco_cpu", True):
        raise ValueError("cuSOLVER requires the Newton GPU backend")
    if int(getattr(solver, "_step", 0)) != 0:
        raise ValueError("Install cuSOLVER before stepping or CUDA graph capture")
    model = solver.mjw_model
    if model.opt.solver != mw_types.SolverType.NEWTON:
        raise ValueError("cuSOLVER requires the Newton constraint solver")
    if int(solver.mjw_data.nworld) != 1:
        raise ValueError("This opt-in cuSOLVER backend currently supports exactly one world")
    if int(model.nv) <= mw_solver._BLOCK_CHOLESKY_DIM:
        raise ValueError("cuSOLVER opt-in is intended for the large blocked system")
    if int(model.nv_pad) < int(model.nv):
        raise ValueError("Invalid padded Hessian size")
    return model


class CuSolverCholesky:
    """Own one CUDA handle/workspace and a model-scoped solver hook."""

    def __init__(self, solver, *, precision="float32", library_path=None):
        global _installed_backend
        self.model = validate_solver(solver, precision)
        if _installed_backend is not None:
            raise RuntimeError("Only one model-scoped cuSOLVER hook may be installed at a time")
        self.device = solver.mjw_data.qpos.device
        if not self.device.is_cuda:
            raise ValueError("cuSOLVER requires CUDA device arrays")
        self.stream = wp.get_stream(self.device)
        if self.stream.is_capturing:
            raise RuntimeError("Create the cuSOLVER handle/workspace before graph capture")
        self.nv, self.n = int(self.model.nv), int(self.model.nv_pad)
        self.dtype = wp.float32 if precision == "float32" else wp.float64
        self.api = CuSolverAPI(library_path, precision)
        self.handle = None
        self.original = mw_solver._cholesky_factorize_solve
        self.closed = False
        self.calls = 0
        try:
            with wp.ScopedDevice(self.device):
                self.handle = self.api.create()
                self.api.set_stream(self.handle, self.stream.cuda_stream)
                initial_deterministic_mode=self.api.deterministic_mode(self.handle)
                self.api.set_deterministic(self.handle)
                self.factor = wp.empty((self.n, self.n), dtype=self.dtype, device=self.device)
                self.rhs = wp.empty(self.n, dtype=self.dtype, device=self.device)
                self.active = wp.zeros(1, dtype=wp.bool, device=self.device)
                self.factor_info = wp.zeros(1, dtype=wp.int32, device=self.device)
                self.solve_info = wp.zeros(1, dtype=wp.int32, device=self.device)
                self.first_failure = wp.zeros(2, dtype=wp.int32, device=self.device)
                self.workspace_count = self.api.buffer_size(self.handle, self.n, self.factor.ptr)
                self.workspace = wp.empty(max(1, self.workspace_count), dtype=self.dtype, device=self.device)
                for kernel in (_prepare_kernel(self.dtype), _identity_kernel(self.dtype),
                               _finalize_kernel(self.dtype, self.nv, self.n)):
                    wp.load_module(module=kernel.module, device=self.device)
                # Initialize cuSOLVER's lazy library state using only private
                # identity scratch. This performs zero simulation steps.
                wp.launch(_identity_kernel(self.dtype), (self.n, self.n),
                          inputs=[self.factor, self.rhs], device=self.device)
                self._factor_and_solve()
                wp.synchronize_stream(self.stream)
                if int(self.factor_info.numpy()[0]) != 0 or int(self.solve_info.numpy()[0]) != 0:
                    raise RuntimeError("cuSOLVER identity initialization failed")
            _installed_backend = self
            mw_solver._cholesky_factorize_solve = _dispatch
        except BaseException:
            if self.handle is not None:
                self.api.destroy(self.handle)
                self.handle = None
            raise
        self.evidence = dict(schema="s300-newton-gpu-cusolver-v1", enabled=True,
            library=self.api.path, precision=precision, native_hessian_precision="float32",
            matrix_size=self.n, nv=self.nv, world_count=1,
            initial_cusolver_deterministic_mode=initial_deterministic_mode,
            cusolver_deterministic_mode=self.api.deterministic_mode(self.handle),
            workspace_bytes=self.workspace_count * (4 if precision == "float32" else 8),
            selected_model_identity_only=True, unchanged_hessian_recomputed=True,
            done_and_noflip_preserve_search=True, host_solve=False,
            handle_workspace_initialized_before_capture=True, simulation_warmup_steps=0,
            installed_library_modified=False, physical_parameters_changed=False,
            physics_equivalence_verified=False, graph_capture_verified=False)

    def _factor_and_solve(self):
        self.api.factor(self.handle, self.n, self.factor.ptr, self.workspace.ptr,
                        self.workspace_count, self.factor_info.ptr)
        self.api.solve(self.handle, self.n, self.factor.ptr, self.rhs.ptr, self.solve_info.ptr)

    def factorize_solve(self, m, d, ctx, skip_unchanged=False, skip_noflip=False):
        if self.closed or m is not self.model or int(d.nworld) != 1:
            raise RuntimeError("cuSOLVER backend used outside its selected model/world")
        stream = wp.get_stream(self.device)
        if stream.cuda_stream != self.stream.cuda_stream:
            raise RuntimeError("cuSOLVER opt-in requires the same CUDA stream used at setup")
        if ctx.h.shape != (1, self.n, self.n) or ctx.grad.shape != (1, self.n):
            raise RuntimeError("cuSOLVER received an incompatible solver context")
        # Recompute even if skip_unchanged requests a cached factorization.
        # This is algebraically the same H^-1 g; noflips still retain the
        # previous search exactly, which is required by the stable fast path.
        changed = ctx.state_changed_count if skip_noflip else d.nefc
        wp.launch(_prepare_kernel(self.dtype), (self.n, self.n),
                  inputs=[self.nv, ctx.h, ctx.grad, ctx.done, changed, bool(skip_noflip),
                          self.factor, self.rhs, self.active], device=self.device)
        self._factor_and_solve()
        wp.launch(_finalize_kernel(self.dtype, self.nv, self.n), 1,
                  inputs=[ctx.grad, self.rhs, self.active, self.factor_info, self.solve_info,
                          self.first_failure, ctx.search, ctx.search_dot, ctx.newton_decrement],
                  device=self.device)
        self.calls += 1

    def check_health(self):
        """Read latched devInfo outside capture, e.g. after each graph batch."""
        if wp.get_stream(self.device).is_capturing:
            raise RuntimeError("Do not read cuSOLVER health during graph capture")
        factor, solve = (int(x) for x in self.first_failure.numpy())
        if factor or solve:
            raise RuntimeError(f"cuSOLVER device failure: potrf_info={factor}, potrs_info={solve}")
        return dict(potrf_info=0, potrs_info=0)

    def close(self):
        global _installed_backend
        if self.closed:
            return
        if wp.get_stream(self.device).is_capturing:
            raise RuntimeError("Do not destroy cuSOLVER during graph capture")
        wp.synchronize_stream(self.stream)
        if _installed_backend is self:
            if mw_solver._cholesky_factorize_solve is not _dispatch:
                raise RuntimeError("cuSOLVER solver hook was replaced by another owner")
            mw_solver._cholesky_factorize_solve = self.original
            _installed_backend = None
        self.api.destroy(self.handle)
        self.handle = None
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
