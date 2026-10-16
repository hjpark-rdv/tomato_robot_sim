"""Opt-in dense BLAS assembly of the existing sparse elliptic J.T C J term.

Only the selected model's native _update_gradient call is intercepted. Native
H initialization, actuator/tendon metric additions, factorization, gradient,
line search and integration remain in their original execution order. This
changes float32 summation grouping, not any physical equation or parameter.
"""
from __future__ import annotations

import mujoco_warp as mw
from mujoco_warp._src import solver as ms, types as mt
import warp as wp

from newton_gpu_cusolver import CuSolverAPI
from newton_gpu_woodbury import _DenseGPUAPI, _curvature, _sparse_jacobian


def validate_dense_hessian(solver, capacity):
    if solver.use_mujoco_cpu or int(solver._step):
        raise ValueError("Install dense Hessian assembly on native GPU before stepping")
    m, d = solver.mjw_model, solver.mjw_data
    if mw.__version__ != "3.15.0" or d.nworld != 1:
        raise ValueError("Dense Hessian assembly currently requires MW3.15 and one world")
    if m.opt.solver != mt.SolverType.NEWTON or not m.is_sparse or m.opt.cone != mt.ConeType.ELLIPTIC:
        raise ValueError("Dense Hessian assembly requires sparse elliptic Newton constraints")
    if m.nmaxcondim > 3 or m.nflex or m.ntendon or m.opt.enableflags & mt.EnableBit.SLEEP:
        raise ValueError("Dense Hessian assembly excludes high-dimensional contacts, flex, tendon and sleep")
    if isinstance(capacity, bool) or not isinstance(capacity, int) or not 1 <= capacity <= d.njmax:
        raise ValueError("Constraint capacity must be a positive integer no larger than njmax")
    return m


@wp.kernel(enable_backward=False)
def _assembly_flags(nefc: wp.array[int], done: wp.array[bool], capacity: int,
                     selected: wp.array[int], counters: wp.array[int]):
    valid = 0 <= nefc[0] and nefc[0] <= capacity
    selected[0] = int(valid and not done[0])
    if not done[0]:
        if valid:
            counters[0] += 1
        else:
            counters[1] += 1
        counters[2] = wp.max(counters[2], nefc[0])


@wp.kernel(enable_backward=False)
def _add_upper(product: wp.array2d[float], done: wp.array[bool], h: wp.array3d[float]):
    row, col = wp.tid()
    if row <= col and not done[0]:
        h[0, row, col] += product[row, col]


_owner = None


def _dispatch(m, d, ctx, compact=False):
    if _owner is None:
        raise RuntimeError("Dense Hessian dispatcher outlived its owner")
    if m is _owner.model:
        return _owner.update_gradient(m, d, ctx, compact=compact)
    return _owner.original(m, d, ctx, compact=compact)


class DenseConstraintHessian:
    def __init__(self, solver, *, capacity=128, library_path=None):
        global _owner
        self.model = validate_dense_hessian(solver, capacity)
        if _owner is not None:
            raise RuntimeError("Only one dense Hessian owner may be installed")
        self.device = solver.mjw_data.qpos.device
        if not self.device.is_cuda:
            raise ValueError("Dense Hessian assembly requires CUDA arrays")
        self.stream = wp.get_stream(self.device)
        if self.stream.is_capturing:
            raise ValueError("Initialize dense Hessian assembly outside capture")
        self.nv, self.capacity = int(self.model.nv), capacity
        self.cusolver_api = CuSolverAPI(library_path, precision="float32")
        self.handle = self.cusolver_api.create()
        self.api = None
        try:
            with wp.ScopedDevice(self.device):
                self.cusolver_api.set_stream(self.handle, self.stream.cuda_stream)
                self.api = _DenseGPUAPI(self.cusolver_api, self.handle, self.stream.cuda_stream)
                self.J = wp.zeros((capacity, self.nv), dtype=float)
                self.B = wp.zeros_like(self.J)
                self.C = wp.zeros((capacity, capacity), dtype=float)
                self.product = wp.zeros((self.nv, self.nv), dtype=float)
                self.selected = wp.zeros(1, dtype=int)
                self.counters = wp.zeros(3, dtype=int)
                for kernel in (_assembly_flags, _add_upper, _curvature, _sparse_jacobian):
                    wp.load_module(module=kernel.module, device=self.device)
                # Warm only private zero matrices; no physics step or real H.
                self._products()
                wp.synchronize_stream(self.stream)
        except BaseException:
            if self.api is not None:
                self.api.close()
            self.cusolver_api.destroy(self.handle)
            raise
        self.original = ms._update_gradient
        self.closed, self.active, self.calls = False, False, 0
        self.evidence = dict(schema="s300-dense-constraint-hessian-v1", enabled=True,
            equation="H += J.T*C*J", gpu_blas="cuBLAS SGEMM PEDANTIC_MATH",
            capacity=capacity, nv=self.nv, float32_grouping_changed=True,
            native_h_initialization_preserved=True, native_factor_solve_preserved=True,
            only_native_JTDACJ_sparse_replaced=True, native_fallback_above_capacity=True,
            padding_and_lower_triangle_untouched=True, model_identity_scoped=True,
            physical_parameters_changed=False, cpu_solve=False,
            graph_capture_verified=False, farm_physics_verified=False)
        _owner = self
        ms._update_gradient = _dispatch

    def _products(self):
        # Physical row-major B=C*J. BLAS sees its transpose J.T*C.T.
        self.api.gemm(0, 0, self.nv, self.capacity, self.capacity,
            self.J, self.nv, self.C, self.capacity, self.B, self.nv)
        # Physical row-major product=J.T*B; BLAS sees B.T*J.
        self.api.gemm(0, 1, self.nv, self.nv, self.capacity,
            self.B, self.nv, self.J, self.nv, self.product, self.nv)

    def _assemble(self, m, d, ctx):
        self.J.zero_()
        # The shared sparse writer copies to both arrays. B is overwritten by
        # GEMM next, so the duplicate write requires no additional allocation.
        wp.launch(_sparse_jacobian, (self.capacity, self.nv), inputs=[d.nefc,
            d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J,
            self.J, self.B], device=self.device)
        wp.launch(_curvature, (self.capacity, self.capacity), inputs=[d.nefc,
            d.efc.type, d.efc.id, d.efc.state, d.efc.D, ctx.Jaref,
            d.contact.efc_address, d.contact.dim, d.contact.friction,
            m.opt.impratio_invsqrt, self.C], device=self.device)
        self._products()
        wp.launch(_add_upper, (self.nv, self.nv), inputs=[self.product, ctx.done,
            ctx.h], device=self.device)

    def assemble_or_native(self, m, d, ctx, native_call):
        if self.closed or m is not self.model or d.nworld != 1:
            raise RuntimeError("Dense Hessian model/world mismatch")
        if wp.get_stream(self.device).cuda_stream != self.stream.cuda_stream:
            raise RuntimeError("Dense Hessian stream changed")
        if ctx.h.shape != (1, m.nv_pad, m.nv_pad):
            raise RuntimeError("Dense Hessian context shape mismatch")
        wp.launch(_assembly_flags, 1, inputs=[d.nefc, ctx.done, self.capacity,
            self.selected, self.counters], device=self.device)
        wp.capture_if(self.selected, on_true=lambda: self._assemble(m, d, ctx),
            on_false=native_call)

    def update_gradient(self, m, d, ctx, compact=False):
        if compact or getattr(ctx, "compact_d_full", None) is not None:
            raise RuntimeError("Dense Hessian does not cover compacted solves")
        if self.active:
            raise RuntimeError("Nested dense Hessian launch scope")
        original_launch = wp.launch
        intercepted = 0
        def launch(*args, **kwargs):
            nonlocal intercepted
            kernel = kwargs.get("kernel", args[0] if args else None)
            if kernel is not None and "_JTDACJ_sparse" in str(kernel.key):
                intercepted += 1
                return self.assemble_or_native(m, d, ctx,
                    lambda: original_launch(*args, **kwargs))
            return original_launch(*args, **kwargs)
        self.active = True
        wp.launch = launch
        try:
            result = self.original(m, d, ctx, compact=compact)
        finally:
            wp.launch = original_launch
            self.active = False
        if intercepted != 1:
            raise RuntimeError(f"Expected one native sparse Hessian assembly, received {intercepted}")
        self.calls += 1
        return result

    def statistics(self):
        reduced, native, max_rows = (int(v) for v in self.counters.numpy())
        values = dict(dense_assemblies=reduced, native_capacity_fallbacks=native,
                      maximum_observed_constraint_rows=max_rows)
        self.evidence["runtime"] = values
        return values

    def close(self):
        global _owner
        if self.closed:
            return
        wp.synchronize_stream(self.stream)
        self.statistics()
        if _owner is self:
            if ms._update_gradient is not _dispatch:
                raise RuntimeError("Dense Hessian hook was replaced")
            ms._update_gradient = self.original
            _owner = None
        self.api.close()
        self.cusolver_api.destroy(self.handle)
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
