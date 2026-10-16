"""Opt-in GPU Woodbury search solve for the existing Newton constraint solver.

For the guarded S300 model H = A + J.T C J, A is the already-authored discrete
backbone qH. A and J stay fixed inside a physics step. Solve in constraint
space using x=A^-1 g-U (I+C S)^-1 C J A^-1 g, U=A^-1 J.T, S=J U.
Native Hessian construction, contacts, line search and all physical constants
remain in their existing pipeline. Capacity/feature exceptions use the full
cuSOLVER solve explicitly. No host linear solve exists in this module.
"""
from __future__ import annotations

import ctypes
from pathlib import Path

import warp as wp
from mujoco_warp._src import math as mm
from mujoco_warp._src import solver as ms
from mujoco_warp._src import types as mt

from newton_gpu_cusolver import CuSolverCholesky, _identity_kernel


def validate_woodbury(solver, rank_capacity):
    """Static applicability checks, before creating any CUDA handle."""
    if solver.use_mujoco_cpu:
        raise ValueError("Woodbury requires the native GPU backend")
    if int(getattr(solver, "_step", 0)):
        raise ValueError("Install Woodbury before stepping or capture")
    m = solver.mjw_model
    if solver.mjw_data.nworld != 1:
        raise ValueError("Woodbury currently supports exactly one world")
    if m.opt.solver != mt.SolverType.NEWTON or m.opt.integrator != mt.IntegratorType.DISCRETE:
        raise ValueError("Woodbury requires the discrete Newton solver")
    if m.opt.cone != mt.ConeType.ELLIPTIC or not m.is_sparse:
        raise ValueError("Woodbury currently requires sparse elliptic constraints")
    if m.nflex or m.ntendon or m.nefmK or m.has_flex_passive or m.has_non_simple_flex or m.efm0_active or m.has_fluid:
        raise ValueError("Woodbury base qH guard rejects flex, tendon or fluid metric coupling")
    if m.opt.enableflags & mt.EnableBit.SLEEP:
        raise ValueError("Woodbury does not support sleeping compact DOFs")
    if int(m.nmaxcondim) > 3:
        raise ValueError("Woodbury currently supports contact dimensions at most three")
    if isinstance(rank_capacity, bool) or not isinstance(rank_capacity, int) or not 1 <= rank_capacity < m.nv:
        raise ValueError("Constraint capacity must be a positive integer below nv")
    return m


class _DenseGPUAPI:
    """Public cuBLAS FP32 GEMM/GEMV/TRSM plus cuSOLVER POTRF and LU."""
    def __init__(self, potrf_api, handle, stream):
        self.potrf = potrf_api
        self.handle = handle
        p, i, fp = ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float)
        candidates = [Path(potrf_api.path).parents[2] / "cublas/lib/libcublas.so.12"]
        candidates += list(Path("/usr/local/lib").glob("python*/dist-packages/nvidia/cublas/lib/libcublas.so.12"))
        path = next((str(x) for x in candidates if x.is_file()), None)
        if path is None:
            raise FileNotFoundError("Installed CUDA12 cuBLAS shared library not found")
        self.blas = ctypes.CDLL(path)
        specs = dict(cublasCreate_v2=[ctypes.POINTER(p)], cublasDestroy_v2=[p],
                     cublasSetStream_v2=[p, p], cublasSetMathMode=[p, i],
                     cublasSetWorkspace_v2=[p, p, ctypes.c_size_t],
                     cublasSgemm_v2=[p, i, i, i, i, i, fp, p, i, p, i, fp, p, i],
                     cublasSgemv_v2=[p, i, i, i, fp, p, i, p, i, fp, p, i],
                     cublasStrsm_v2=[p, i, i, i, i, i, i, fp, p, i, p, i])
        for name, args in specs.items():
            fn = getattr(self.blas, name)
            fn.argtypes, fn.restype = args, i
        self.blas_handle = p()
        self.check(self.blas.cublasCreate_v2(ctypes.byref(self.blas_handle)), "cublasCreate")
        self.check(self.blas.cublasSetStream_v2(self.blas_handle, p(stream)), "cublasSetStream")
        self.workspace = wp.empty(8 * 1024 * 1024, dtype=wp.uint8)
        self.check(self.blas.cublasSetWorkspace_v2(self.blas_handle, self.workspace.ptr,
                                                self.workspace.size), "cublasSetWorkspace")
        # Explicit pedantic FP32: no TF32/half precision request.
        self.check(self.blas.cublasSetMathMode(self.blas_handle, 2), "cublasSetMathMode")
        self.library = path
        lu = dict(cusolverDnSgetrf_bufferSize=[p, i, i, p, i, ctypes.POINTER(i)],
                  cusolverDnSgetrf=[p, i, i, p, i, p, p, p],
                  cusolverDnSgetrs=[p, i, i, i, p, i, p, p, i, p])
        for name, args in lu.items():
            fn = getattr(potrf_api.lib, name)
            fn.argtypes, fn.restype = args, i

    @staticmethod
    def check(status, name):
        if int(status):
            raise RuntimeError(f"GPU Woodbury {name} returned status {int(status)}")

    def multi_solve(self, n, rhs_count, factor, rhs, info):
        # Legacy POTRS has a hidden allocation for large multi-RHS systems,
        # disallowed inside CUDA conditional graphs. Explicit workspace-backed
        # triangular solves implement the identical Cholesky equation.
        alpha = ctypes.c_float(1.)
        for transpose in (0, 1):
            self.check(self.blas.cublasStrsm_v2(self.blas_handle, 0, 0, transpose,
                0, n, rhs_count, ctypes.byref(alpha), factor.ptr, n, rhs.ptr, n), "trsm")

    def lu_workspace(self, n, matrix):
        count = ctypes.c_int()
        self.check(self.potrf.lib.cusolverDnSgetrf_bufferSize(self.handle, n, n, matrix.ptr,
                                                           n, ctypes.byref(count)), "getrf_bufferSize")
        return count.value

    def lu_factor(self, n, matrix, workspace, pivots, info):
        self.check(self.potrf.lib.cusolverDnSgetrf(self.handle, n, n, matrix.ptr, n,
                                                workspace.ptr, pivots.ptr, info.ptr), "getrf")

    def lu_solve(self, n, matrix, pivots, rhs, info):
        self.check(self.potrf.lib.cusolverDnSgetrs(self.handle, 0, n, 1, matrix.ptr, n,
                                                pivots.ptr, rhs.ptr, n, info.ptr), "getrs")

    def gemm(self, op_a, op_b, m, n, k, A, lda, B, ldb, out, ldc):
        alpha, beta = ctypes.c_float(1.), ctypes.c_float(0.)
        self.check(self.blas.cublasSgemm_v2(self.blas_handle, op_a, op_b, m, n, k,
            ctypes.byref(alpha), A.ptr, lda, B.ptr, ldb, ctypes.byref(beta), out.ptr, ldc), "gemm")

    def gemv(self, trans, rows, cols, A, lda, x, out, *, alpha=1., beta=0.):
        a, b = ctypes.c_float(alpha), ctypes.c_float(beta)
        self.check(self.blas.cublasSgemv_v2(self.blas_handle, trans, rows, cols,
            ctypes.byref(a), A.ptr, lda, x.ptr, 1, ctypes.byref(b), out.ptr, 1), "gemv")

    def close(self):
        if self.blas_handle is not None:
            self.check(self.blas.cublasDestroy_v2(self.blas_handle), "cublasDestroy")
            self.blas_handle = None


@wp.kernel(enable_backward=False)
def _flags(nefc: wp.array[int], moment_nnz: wp.array2d[int], done: wp.array[bool],
           simulation_time: wp.array[float], capacity: int, skip_noflip: bool,
           cached_time: wp.array[float], use_small: wp.array[int], refresh: wp.array[int],
           counters: wp.array[int]):
    valid = nefc[0] <= capacity and not skip_noflip
    for actuator in range(moment_nnz.shape[1]):
        if moment_nnz[0, actuator] > 1:
            valid = False
    use_small[0] = int(valid)
    refresh[0] = int(valid and simulation_time[0] != cached_time[0])
    if valid:
        cached_time[0] = simulation_time[0]
    if not done[0]:
        if valid:
            counters[0] += 1
            counters[1] += refresh[0]
        else:
            counters[2] += 1
        counters[3] = wp.max(counters[3], nefc[0])


@wp.kernel(enable_backward=False)
def _base_matrix(M_elemid: wp.array2d[int], qH: wp.array2d[float], A: wp.array2d[float]):
    i, j = wp.tid()
    value = float(0.)
    if j >= i:
        address = M_elemid[j, i]
        if address >= 0:
            value = qH[0, address]
    A[i, j] = value


@wp.kernel(enable_backward=False)
def _sparse_jacobian(nefc: wp.array[int], rownnz: wp.array2d[int], rowadr: wp.array2d[int],
                     cols: wp.array3d[int], values: wp.array3d[float],
                     J: wp.array2d[float], U: wp.array2d[float]):
    row, entry = wp.tid()
    if row < nefc[0] and entry < rownnz[0, row]:
        address = rowadr[0, row] + entry
        col = cols[0, 0, address]
        value = values[0, 0, address]
        J[row, col] = value
        U[row, col] = value


@wp.kernel(enable_backward=False)
def _curvature(nefc: wp.array[int], types: wp.array2d[int], ids: wp.array2d[int],
               state: wp.array2d[int], D: wp.array2d[float], Jaref: wp.array2d[float],
               addresses: wp.array2d[int], dimensions: wp.array[int], friction: wp.array[mt.vec5],
               impratio_invsqrt: wp.array[float], C: wp.array2d[float]):
    row, col = wp.tid()
    value = float(0.)
    if row < nefc[0] and col < nefc[0]:
        if row == col and state[0, row] == mt.ConstraintState.QUADRATIC:
            value = D[0, row]
        if state[0, row] == mt.ConstraintState.CONE and types[0, row] == mt.ConstraintType.CONTACT_ELLIPTIC:
            contact = ids[0, row]
            head = addresses[contact, 0]
            dim = dimensions[contact]
            if col >= head and col < head + dim:
                a, b = row - head, col - head
                fri = friction[contact]
                mu = fri[0] * impratio_invsqrt[0]
                mu2 = mu * mu
                dm = mm.safe_div(D[0, head], mu2 * (1. + mu2))
                u1, u2 = float(0.), float(0.)
                if head + 1 < nefc[0]:
                    u1 = Jaref[0, head + 1] * fri[0]
                if head + 2 < nefc[0]:
                    u2 = Jaref[0, head + 2] * fri[1]
                inv_t = mm.safe_div(1., wp.max(wp.sqrt(u1 * u1 + u2 * u2), mt.MJ_MINVAL))
                n = Jaref[0, head] * mu
                mu_n_over_t = mu * n * inv_t
                z0a, z0b, pa, pb, dot = float(0.), float(0.), float(0.), float(0.), float(0.)
                if a == 0:
                    z0a = mu
                elif a == 1:
                    pa = u1 * inv_t * fri[0]
                else:
                    pa = u2 * inv_t * fri[1]
                if b == 0:
                    z0b = mu
                elif b == 1:
                    pb = u1 * inv_t * fri[0]
                else:
                    pb = u2 * inv_t * fri[1]
                if a == b and a > 0:
                    dot = fri[a - 1] * fri[a - 1]
                value = ms._elliptic_hessian_entry_from_projections(dm, mu, mu_n_over_t,
                    mu2 - mu_n_over_t, z0a, z0b, pa, pb, dot)
    C[row, col] = value


@wp.kernel(enable_backward=False)
def _copy_gradient(gradient: wp.array2d[float], rhs: wp.array[float]):
    i = wp.tid()
    rhs[i] = gradient[0, i]


@wp.kernel(enable_backward=False)
def _lu_matrix(K_row: wp.array2d[float], K_col: wp.array2d[float]):
    row, col = wp.tid()
    value = K_row[col, row]
    if row == col:
        value += 1.
    K_col[row, col] = value


@wp.kernel(enable_backward=False)
def _latch(info: wp.array[int], category: int, first_failure: wp.array[int]):
    if info[0] != 0:
        wp.atomic_cas(first_failure, category, 0, info[0])


@wp.kernel(enable_backward=False)
def _finish(gradient: wp.array2d[float], solution: wp.array[float], done: wp.array[bool],
            first_failure: wp.array[int], search: wp.array2d[float],
            search_dot: wp.array[float], decrement: wp.array[float]):
    if not done[0]:
        norm, dec = float(0.), float(0.)
        for i in range(solution.shape[0]):
            x = solution[i]
            search[0, i] = -x
            norm += x * x
            dec += gradient[0, i] * x
        search_dot[0] = norm
        decrement[0] = dec
        if first_failure[0] != 0 or first_failure[1] != 0:
            search[0, 0] = wp.nan
            search_dot[0] = wp.nan
            decrement[0] = wp.nan


@wp.kernel(enable_backward=False)
def _residual_rows(h: wp.array3d[float], gradient: wp.array2d[float],
                   solution: wp.array[float], errors: wp.array[wp.float64]):
    i = wp.tid()
    dot = wp.float64(0.)
    for j in range(solution.shape[0]):
        dot += wp.float64(h[0, wp.min(i, j), wp.max(i, j)]) * wp.float64(solution[j])
    error = dot - wp.float64(gradient[0, i])
    errors[i] = error * error


@wp.kernel(enable_backward=False)
def _residual_decision(gradient: wp.array2d[float], errors: wp.array[wp.float64],
                       done: wp.array[bool], tolerance: float, numerical_fallback: wp.array[int],
                       counters: wp.array[int], maximum: wp.array[float]):
    numerator, denominator = wp.float64(0.), wp.float64(0.)
    for i in range(errors.shape[0]):
        numerator += errors[i]
        denominator += wp.float64(gradient[0, i]) * wp.float64(gradient[0, i])
    relative = wp.sqrt(numerator / wp.max(denominator, wp.float64(1.e-30)))
    failed = not wp.isfinite(relative) or relative > wp.float64(tolerance)
    numerical_fallback[0] = int(failed and not done[0])
    if not done[0]:
        counters[4] += numerical_fallback[0]
        if wp.isfinite(relative):
            maximum[0] = wp.max(maximum[0], float(relative))
        else:
            counters[5] += 1


_owner = None


def _dispatch(m, d, ctx, skip_unchanged=False, skip_noflip=False):
    if _owner is None:
        raise RuntimeError("Woodbury dispatch outlived its owner")
    if m is _owner.model:
        return _owner.factorize_solve(m, d, ctx, skip_unchanged, skip_noflip)
    return _owner.original(m, d, ctx, skip_unchanged, skip_noflip)


class WoodburyCholesky:
    """Guarded reduced solve; full GPU cuSOLVER fallback is explicit/counted."""
    def __init__(self, solver, *, rank_capacity=128, residual_tolerance=1.e-4, library_path=None):
        global _owner
        self.model = validate_woodbury(solver, rank_capacity)
        if _owner is not None:
            raise RuntimeError("Only one Woodbury owner may be installed")
        self.device = solver.mjw_data.qpos.device
        if not self.device.is_cuda:
            raise ValueError("Woodbury requires CUDA arrays")
        self.stream = wp.get_stream(self.device)
        self.nv, self.rank = int(self.model.nv), rank_capacity
        if not 0. < residual_tolerance <= 1.e-3:
            raise ValueError("Woodbury residual tolerance must be in (0, 1e-3]")
        self.residual_tolerance = float(residual_tolerance)
        self.fallback = CuSolverCholesky(solver, precision="float32", library_path=library_path)
        self.original = ms._cholesky_factorize_solve
        self.calls, self.closed = 0, False
        self.first_failure = self.fallback.first_failure
        self.api = None
        try:
            with wp.ScopedDevice(self.device):
                self.api = _DenseGPUAPI(self.fallback.api, self.fallback.handle, self.stream.cuda_stream)
                def zeros(shape, dtype=float):
                    return wp.zeros(shape, dtype=dtype, device=self.device)
                self.A = zeros((self.nv, self.nv))
                self.J = zeros((self.rank, self.nv)); self.U = zeros((self.rank, self.nv))
                self.S = zeros((self.rank, self.rank)); self.C = zeros((self.rank, self.rank))
                self.Krow = zeros((self.rank, self.rank)); self.Kcol = zeros((self.rank, self.rank))
                self.g = zeros(self.nv); self.t = zeros(self.rank); self.r = zeros(self.rank)
                self.pivots = zeros(self.rank, int); self.info = zeros(1, int)
                self.use_small = zeros(1, int); self.refresh = zeros(1, int)
                self.cached_time = wp.array([-1.], dtype=float, device=self.device)
                self.counters = zeros(6, int)
                self.residual_errors = zeros(self.nv, wp.float64)
                self.numerical_fallback = zeros(1, int)
                self.maximum_residual = zeros(1)
                self.base_workspace_count = self.fallback.api.buffer_size(self.fallback.handle, self.nv, self.A.ptr)
                self.base_workspace = zeros(max(1, self.base_workspace_count))
                self.lu_workspace = zeros(max(1, self.api.lu_workspace(self.rank, self.Kcol)))
                for kernel in (_flags, _base_matrix, _sparse_jacobian, _curvature,
                               _copy_gradient, _lu_matrix, _latch, _finish,
                               _residual_rows, _residual_decision):
                    wp.load_module(module=kernel.module, device=self.device)
                # Library preparation touches private identity scratch only.
                wp.launch(_identity_kernel(wp.float32), (self.nv, self.nv), inputs=[self.A, self.g], device=self.device)
                self.fallback.api.factor(self.fallback.handle, self.nv, self.A.ptr,
                    self.base_workspace.ptr, self.base_workspace_count, self.info.ptr)
                self.api.multi_solve(self.nv, self.rank, self.A, self.U, self.info)
                wp.launch(_lu_matrix, (self.rank, self.rank), inputs=[self.Krow, self.Kcol], device=self.device)
                self.api.lu_factor(self.rank, self.Kcol, self.lu_workspace, self.pivots, self.info)
                self.api.lu_solve(self.rank, self.Kcol, self.pivots, self.r, self.info)
                self.api.gemm(1, 0, self.rank, self.rank, self.nv,
                              self.U, self.nv, self.J, self.nv, self.S, self.rank)
                wp.synchronize_stream(self.stream)
                if int(self.info.numpy()[0]):
                    raise RuntimeError("Woodbury identity scratch initialization failed")
            _owner = self
            ms._cholesky_factorize_solve = _dispatch
        except BaseException:
            if self.api is not None:
                self.api.close()
            self.fallback.close()
            raise
        self.evidence = dict(schema="s300-newton-gpu-woodbury-v1", enabled=True,
            implementation="GPU cuBLAS FP32 GEMM/GEMV/TRSM + cuSOLVER POTRF and small LU",
            cublas_library=self.api.library, cusolver_library=self.fallback.api.path,
            cusolver_deterministic_mode=self.fallback.api.deterministic_mode(self.fallback.handle),
            precision="float32", cublas_math_mode="PEDANTIC_MATH",
            nv=self.nv, rank_capacity=self.rank, base="native discrete qH",
            curvature="native elliptic Hessian contraction function",
            unconstrained_backbone_cached_per_physics_step=True,
            cublas_preallocated_workspace_bytes=self.api.workspace.size,
            singular_curvature_supported_without_regularization=True,
            runtime_capacity_or_multi_dof_actuator_fallback="full GPU cuSOLVER Newton solve",
            cpu_solve=False, physical_parameters_changed=False,
            installed_library_modified=False, native_hessian_assembly_retained=True,
            every_solve_actual_hessian_residual=True,
            residual_metric="norm2(ctx.h*x-ctx.grad)/max(norm2(ctx.grad),1e-15)",
            residual_tolerance=self.residual_tolerance,
            residual_failure_action="full original GPU Newton cuSOLVER solve; counted",
            physics_equivalence_verified=False, graph_capture_verified=False)

    def _info(self, category):
        wp.launch(_latch, 1, inputs=[self.info, category, self.first_failure], device=self.device)

    def _refresh(self, m, d):
        wp.launch(_base_matrix, (self.nv, self.nv), inputs=[m.M_elemid, d.qH, self.A], device=self.device)
        self.J.zero_(); self.U.zero_()
        wp.launch(_sparse_jacobian, (self.rank, self.nv), inputs=[d.nefc, d.efc.J_rownnz,
            d.efc.J_rowadr, d.efc.J_colind, d.efc.J, self.J, self.U], device=self.device)
        self.fallback.api.factor(self.fallback.handle, self.nv, self.A.ptr,
            self.base_workspace.ptr, self.base_workspace_count, self.info.ptr)
        self._info(0)
        self.api.multi_solve(self.nv, self.rank, self.A, self.U, self.info)
        self._info(1)
        # Row-major S=J U.T; cuBLAS sees column-major transposes.
        self.api.gemm(1, 0, self.rank, self.rank, self.nv,
                      self.U, self.nv, self.J, self.nv, self.S, self.rank)

    def _reduced_solve(self, m, d, ctx):
        wp.capture_if(self.refresh, on_true=lambda: self._refresh(m, d))
        wp.launch(_copy_gradient, self.nv, inputs=[ctx.grad, self.g], device=self.device)
        self.api.multi_solve(self.nv, 1, self.A, self.g, self.info)
        self._info(1)
        wp.launch(_curvature, (self.rank, self.rank), inputs=[d.nefc, d.efc.type, d.efc.id,
            d.efc.state, d.efc.D, ctx.Jaref, d.contact.efc_address, d.contact.dim,
            d.contact.friction, m.opt.impratio_invsqrt, self.C], device=self.device)
        # Row-major K=C S; column-major output is S.T C.T.
        self.api.gemm(0, 0, self.rank, self.rank, self.rank,
                      self.S, self.rank, self.C, self.rank, self.Krow, self.rank)
        wp.launch(_lu_matrix, (self.rank, self.rank), inputs=[self.Krow, self.Kcol], device=self.device)
        self.api.gemv(1, self.nv, self.rank, self.J, self.nv, self.g, self.t)
        self.api.gemv(1, self.rank, self.rank, self.C, self.rank, self.t, self.r)
        self.api.lu_factor(self.rank, self.Kcol, self.lu_workspace, self.pivots, self.info)
        self._info(0)
        self.api.lu_solve(self.rank, self.Kcol, self.pivots, self.r, self.info)
        self._info(1)
        self.api.gemv(0, self.nv, self.rank, self.U, self.nv, self.r, self.g, alpha=-1., beta=1.)
        # Compare against the actual assembled native Hessian, not the assumed
        # decomposition. This catches any unsupported/omitted metric coupling.
        wp.launch(_residual_rows, self.nv, inputs=[ctx.h, ctx.grad, self.g,
            self.residual_errors], device=self.device)
        wp.launch(_residual_decision, 1, inputs=[ctx.grad, self.residual_errors,
            ctx.done, self.residual_tolerance, self.numerical_fallback,
            self.counters, self.maximum_residual], device=self.device)
        wp.launch(_finish, 1, inputs=[ctx.grad, self.g, ctx.done, self.first_failure,
            ctx.search, ctx.search_dot, ctx.newton_decrement], device=self.device)
        wp.capture_if(self.numerical_fallback,
            on_true=lambda: self.fallback.factorize_solve(m, d, ctx))

    def factorize_solve(self, m, d, ctx, skip_unchanged=False, skip_noflip=False):
        if self.closed or m is not self.model or d.nworld != 1:
            raise RuntimeError("Woodbury called outside its selected model/world")
        if wp.get_stream(self.device).cuda_stream != self.stream.cuda_stream:
            raise RuntimeError("Woodbury requires its original CUDA stream")
        wp.launch(_flags, 1, inputs=[d.nefc, d.moment_rownnz, ctx.done, d.time,
            self.rank, skip_noflip, self.cached_time, self.use_small, self.refresh,
            self.counters], device=self.device)
        wp.capture_if(self.use_small, on_true=lambda: self._reduced_solve(m, d, ctx),
            on_false=lambda: self.fallback.factorize_solve(m, d, ctx, skip_unchanged, skip_noflip))
        self.calls += 1

    def check_health(self):
        if wp.get_stream(self.device).is_capturing:
            raise RuntimeError("Do not read Woodbury health during capture")
        factor, solve = (int(v) for v in self.first_failure.numpy())
        if factor or solve:
            raise RuntimeError(f"Woodbury/full fallback GPU linear solve failed: factor={factor}, solve={solve}")
        return dict(factor_info=0, solve_info=0)

    def statistics(self):
        small, refreshes, fallback, max_rows, numerical, nonfinite = (int(v) for v in self.counters.numpy())
        values = dict(reduced_solves=small, base_refreshes=refreshes,
                      full_fallback_solves=fallback, max_observed_constraint_rows=max_rows)
        values.update(numerical_residual_fallback_solves=numerical,
                      nonfinite_residuals=nonfinite,
                      max_relative_actual_hessian_residual=float(self.maximum_residual.numpy()[0]))
        self.evidence["runtime"] = values
        return values

    def close(self):
        global _owner
        if self.closed:
            return
        if wp.get_stream(self.device).is_capturing:
            raise RuntimeError("Do not remove Woodbury during capture")
        wp.synchronize_stream(self.stream)
        self.statistics()
        if _owner is self:
            if ms._cholesky_factorize_solve is not _dispatch:
                raise RuntimeError("Woodbury hook was replaced by another owner")
            ms._cholesky_factorize_solve = self.original
            _owner = None
        self.api.close()
        self.fallback.close()
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
