"""Opt-in dense BLAS assembly of the existing sparse elliptic J.T C J term.

Only the selected model's native _update_gradient call is intercepted. Native
H initialization, actuator/tendon metric additions, factorization, gradient,
line search and integration remain in their original execution order. This
changes float32 summation grouping, not any physical equation or parameter.
"""
from __future__ import annotations

from types import SimpleNamespace

import mujoco_warp as mw
from mujoco_warp._src import solver as ms, types as mt
import warp as wp

from newton_gpu_cusolver import CuSolverAPI
from newton_gpu_woodbury import _DenseGPUAPI, _curvature, _sparse_jacobian


def validate_dense_hessian(solver, capacity, block_curvature=False, overflow_capacity=None,
                           cache_jacobian=False):
    if not isinstance(block_curvature, bool):
        raise ValueError("block_curvature must be bool")
    if not isinstance(cache_jacobian, bool):
        raise ValueError("cache_jacobian must be bool")
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
    if overflow_capacity is not None and (isinstance(overflow_capacity, bool)
            or not isinstance(overflow_capacity, int) or not capacity < overflow_capacity <= d.njmax):
        raise ValueError("Overflow capacity must be an integer larger than capacity and no larger than njmax")
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
def _bucket_assembly_flags(nefc: wp.array[int], done: wp.array[bool], capacity: int,
                          overflow_capacity: int, selected: wp.array[int],
                          overflow_selected: wp.array[int], counters: wp.array[int],
                          overflow_counters: wp.array[int]):
    primary = 0 <= nefc[0] and nefc[0] <= capacity
    overflow = capacity < nefc[0] and nefc[0] <= overflow_capacity
    selected[0] = int(primary and not done[0])
    overflow_selected[0] = int(overflow and not done[0])
    if not done[0]:
        if primary or overflow:
            counters[0] += 1
        else:
            counters[1] += 1
        if overflow:
            overflow_counters[0] += 1
        counters[2] = wp.max(counters[2], nefc[0])


@wp.kernel(enable_backward=False)
def _jacobian_prepare_flags(nefc: wp.array[int], capacity: int, overflow_capacity: int,
                            selected: wp.array[int], overflow_selected: wp.array[int],
                            counters: wp.array[int]):
    # ctx.done belongs to the previous solve until _solve initializes it.
    # Bucket choice depends only on the newly constructed constraint rows.
    primary = 0 <= nefc[0] and nefc[0] <= capacity
    overflow = capacity < nefc[0] and nefc[0] <= overflow_capacity
    selected[0] = int(primary)
    overflow_selected[0] = int(overflow)
    if primary:
        counters[0] += 1
    elif overflow:
        counters[1] += 1
    else:
        counters[2] += 1


@wp.kernel(enable_backward=False)
def _add_upper(product: wp.array2d[float], done: wp.array[bool], h: wp.array3d[float]):
    row, col = wp.tid()
    if row <= col and not done[0]:
        h[0, row, col] += product[row, col]


@wp.kernel(enable_backward=False)
def _block_curvature_product(nefc: wp.array[int], types: wp.array2d[int],
                             ids: wp.array2d[int], state: wp.array2d[int],
                             addresses: wp.array2d[int], dimensions: wp.array[int],
                             C: wp.array2d[float], J: wp.array2d[float],
                             B: wp.array2d[float]):
    """Compute C*J from the exact support emitted by the existing _curvature.

    Quadratic rows have one diagonal entry. CONE elliptic rows have entries
    only within their contact block (at most three rows under the model guard).
    Other rows, including capacity padding, are zero. C itself is retained,
    so its curvature formulas and coefficients are not reimplemented here.

    For finite inputs this has the same real-valued product as dense SGEMM.
    Float32 summation/FMA grouping and signed-zero results can differ; no
    bitwise parity or identical nonlinear solver decisions are promised.
    """
    row, col = wp.tid()
    value = float(0.)
    if row < nefc[0]:
        if state[0, row] == mt.ConstraintState.QUADRATIC:
            value = C[row, row] * J[row, col]
        elif (state[0, row] == mt.ConstraintState.CONE
              and types[0, row] == mt.ConstraintType.CONTACT_ELLIPTIC):
            contact = ids[0, row]
            head = addresses[contact, 0]
            # Match _curvature's col<nefc and allocated C/J extent, including
            # a partial contact at the boundary in diagnostic controls.
            begin = wp.max(0, head)
            end = wp.min(wp.min(nefc[0], C.shape[1]), head + dimensions[contact])
            for k in range(begin, end):
                value += C[row, k] * J[k, col]
    B[row, col] = value


_owner = None


def _dispatch(m, d, ctx, compact=False):
    if _owner is None:
        raise RuntimeError("Dense Hessian dispatcher outlived its owner")
    if m is _owner.model:
        return _owner.update_gradient(m, d, ctx, compact=compact)
    return _owner.original(m, d, ctx, compact=compact)


def _solve_dispatch(m, d, ctx, compact=False):
    if _owner is None:
        raise RuntimeError("Dense Hessian solve dispatcher outlived its owner")
    if m is _owner.model:
        return _owner.solve(m, d, ctx, compact=compact)
    return _owner.original_solve(m, d, ctx, compact=compact)


class DenseConstraintHessian:
    def __init__(self, solver, *, capacity=128, library_path=None, block_curvature=False,
                 overflow_capacity=None, cache_jacobian=False):
        global _owner
        self.model = validate_dense_hessian(solver, capacity, block_curvature,
            overflow_capacity, cache_jacobian)
        self.block_curvature = block_curvature
        self.cache_jacobian = cache_jacobian
        self.jacobian_solve_active = False
        self.overflow_capacity = overflow_capacity
        self.overflow = None
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
                if self.cache_jacobian:
                    self.jacobian_overflow_selected = wp.zeros(1, dtype=int)
                    self.jacobian_counters = wp.zeros(3, dtype=int)
                if overflow_capacity is not None:
                    # Separate buffers preserve the original small bucket's
                    # exact GEMM dimensions, leading dimensions and operands.
                    # A larger allocation must not silently replace that path.
                    self.overflow = SimpleNamespace(capacity=overflow_capacity,
                        J=wp.zeros((overflow_capacity, self.nv), dtype=float),
                        B=wp.zeros((overflow_capacity, self.nv), dtype=float),
                        C=wp.zeros((overflow_capacity, overflow_capacity), dtype=float),
                        product=wp.zeros((self.nv, self.nv), dtype=float),
                        selected=wp.zeros(1, dtype=int), counters=wp.zeros(1, dtype=int))
                kernels = (_assembly_flags, _add_upper, _curvature, _sparse_jacobian)
                if self.overflow is not None:
                    kernels += (_bucket_assembly_flags,)
                if self.block_curvature:
                    kernels += (_block_curvature_product,)
                if self.cache_jacobian:
                    kernels += (_jacobian_prepare_flags,)
                for kernel in kernels:
                    wp.load_module(module=kernel.module, device=self.device)
                # Warm only private zero matrices; no physics step or real H.
                self._products()
                if self.overflow is not None:
                    self._products(workspace=self.overflow)
                wp.synchronize_stream(self.stream)
        except BaseException:
            if self.api is not None:
                self.api.close()
            self.cusolver_api.destroy(self.handle)
            raise
        self.original = ms._update_gradient
        self.original_solve = ms._solve if self.cache_jacobian else None
        self.closed, self.active, self.calls = False, False, 0
        self.evidence = dict(schema="s300-dense-constraint-hessian-v1", enabled=True,
            equation="H += J.T*C*J", gpu_blas="cuBLAS SGEMM PEDANTIC_MATH",
            capacity=capacity, nv=self.nv, float32_grouping_changed=True,
            native_h_initialization_preserved=True, native_factor_solve_preserved=True,
            only_native_JTDACJ_sparse_replaced=True, native_fallback_above_capacity=True,
            padding_and_lower_triangle_untouched=True, model_identity_scoped=True,
            physical_parameters_changed=False, cpu_solve=False,
            graph_capture_verified=False, farm_physics_verified=False)
        self.evidence.update(block_curvature=self.block_curvature,
            curvature_product=("Warp diagonal/contact-block C*J" if self.block_curvature
                               else "cuBLAS SGEMM PEDANTIC_MATH"),
            curvature_matrix_preserved=True, final_j_transpose_b_gemm_preserved=True,
            curvature_product_bitwise_equivalence_claimed=False)
        self.evidence.update(overflow_capacity=overflow_capacity,
            maximum_dense_capacity=overflow_capacity or capacity,
            overflow_bucket_enabled=overflow_capacity is not None,
            original_primary_workspace_and_gemm_dimensions_preserved=True,
            native_fallback_above_maximum_capacity=True,
            bucket_physics_equivalence_verified=False)
        self.evidence.update(cache_jacobian=self.cache_jacobian,
            jacobian_cache_scope="one native _solve invocation" if self.cache_jacobian else None,
            jacobian_rebuilt_at_every_solve=self.cache_jacobian,
            jacobian_prepared_after_constraint_construction=self.cache_jacobian,
            jacobian_cache_changes_gemm_arithmetic=False,
            jacobian_cache_gpu_equivalence_verified=False)
        _owner = self
        ms._update_gradient = _dispatch
        if self.cache_jacobian:
            ms._solve = _solve_dispatch

    def _products(self, d=None, *, workspace=None):
        bucket = self if workspace is None else workspace
        if self.block_curvature and d is not None:
            wp.launch(_block_curvature_product, (bucket.capacity, self.nv),
                inputs=[d.nefc, d.efc.type, d.efc.id, d.efc.state,
                        d.contact.efc_address, d.contact.dim, bucket.C, bucket.J, bucket.B],
                device=self.device)
        else:
            # Physical row-major B=C*J. BLAS sees its transpose J.T*C.T.
            # Setup warms private zero matrices through this branch even
            # when the runtime block product is selected; no state is read.
            self.api.gemm(0, 0, self.nv, bucket.capacity, bucket.capacity,
                bucket.J, self.nv, bucket.C, bucket.capacity, bucket.B, self.nv)
        # Physical row-major product=J.T*B; BLAS sees B.T*J.
        self.api.gemm(0, 1, self.nv, self.nv, bucket.capacity,
            bucket.B, self.nv, bucket.J, self.nv, bucket.product, self.nv)

    def _write_jacobian(self, d, workspace=None):
        bucket = self if workspace is None else workspace
        bucket.J.zero_()
        # The shared sparse writer copies to both arrays. B is overwritten by
        # the product next, so the duplicate write needs no extra allocation.
        wp.launch(_sparse_jacobian, (bucket.capacity, self.nv), inputs=[d.nefc,
            d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J,
            bucket.J, bucket.B], device=self.device)

    def _prepare_jacobian(self, d):
        overflow = self.overflow
        wp.launch(_jacobian_prepare_flags, 1, inputs=[d.nefc, self.capacity,
            overflow.capacity if overflow is not None else self.capacity,
            self.selected, self.jacobian_overflow_selected, self.jacobian_counters],
            device=self.device)
        def larger_or_native():
            wp.capture_if(self.jacobian_overflow_selected,
                on_true=lambda: self._write_jacobian(d, workspace=overflow))
        wp.capture_if(self.selected, on_true=lambda: self._write_jacobian(d),
            on_false=larger_or_native if overflow is not None else None)

    def solve(self, m, d, ctx, compact=False):
        """Build J after constraint construction, once for this native solve.

        MW3.15's _solve changes forces, states and curvature, but not the CSR
        Jacobian or nefc. Each invocation unconditionally queues fresh writes
        before its first gradient, including every step in a captured batch.
        No data survives as a valid cache across solves, graph replays, warmups
        or session resets. Calls to _update_gradient outside this scope retain
        the original per-assembly writer. The same writer and two GEMM calls
        are used; this only removes repeated copies of invariant values.
        """
        if self.closed or m is not self.model or d.nworld != 1:
            raise RuntimeError("Dense Hessian model/world mismatch")
        if compact or getattr(ctx, "compact_d_full", None) is not None:
            raise RuntimeError("Dense Hessian does not cover compacted solves")
        if self.jacobian_solve_active:
            raise RuntimeError("Nested dense Jacobian solve scope")
        if wp.get_stream(self.device).cuda_stream != self.stream.cuda_stream:
            raise RuntimeError("Dense Hessian stream changed")
        self._prepare_jacobian(d)
        self.jacobian_solve_active = True
        try:
            return self.original_solve(m, d, ctx, compact=compact)
        finally:
            self.jacobian_solve_active = False

    def _assemble(self, m, d, ctx, workspace=None):
        bucket = self if workspace is None else workspace
        if not (getattr(self, "cache_jacobian", False) and self.jacobian_solve_active):
            self._write_jacobian(d, workspace=workspace)
        wp.launch(_curvature, (bucket.capacity, bucket.capacity), inputs=[d.nefc,
            d.efc.type, d.efc.id, d.efc.state, d.efc.D, ctx.Jaref,
            d.contact.efc_address, d.contact.dim, d.contact.friction,
            m.opt.impratio_invsqrt, bucket.C], device=self.device)
        self._products(d, workspace=workspace)
        wp.launch(_add_upper, (self.nv, self.nv), inputs=[bucket.product, ctx.done,
            ctx.h], device=self.device)

    def assemble_or_native(self, m, d, ctx, native_call):
        if self.closed or m is not self.model or d.nworld != 1:
            raise RuntimeError("Dense Hessian model/world mismatch")
        if wp.get_stream(self.device).cuda_stream != self.stream.cuda_stream:
            raise RuntimeError("Dense Hessian stream changed")
        if ctx.h.shape != (1, m.nv_pad, m.nv_pad):
            raise RuntimeError("Dense Hessian context shape mismatch")
        overflow = getattr(self, "overflow", None)
        if overflow is not None:
            wp.launch(_bucket_assembly_flags, 1, inputs=[d.nefc, ctx.done, self.capacity,
                overflow.capacity, self.selected, overflow.selected, self.counters,
                overflow.counters], device=self.device)
            def larger_or_native():
                wp.capture_if(overflow.selected,
                    on_true=lambda: self._assemble(m, d, ctx, workspace=overflow),
                    on_false=native_call)
            wp.capture_if(self.selected, on_true=lambda: self._assemble(m, d, ctx),
                on_false=larger_or_native)
            return
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
        overflow = getattr(self, "overflow", None)
        if overflow is not None:
            count = int(overflow.counters.numpy()[0])
            values.update(primary_dense_assemblies=reduced-count,
                overflow_dense_assemblies=count,
                primary_capacity=self.capacity, overflow_capacity=overflow.capacity)
        if getattr(self, "cache_jacobian", False):
            primary, larger, fallback = (int(v) for v in self.jacobian_counters.numpy())
            values.update(jacobian_prepared_solves=primary+larger+fallback,
                jacobian_primary_builds=primary, jacobian_overflow_builds=larger,
                jacobian_native_fallback_solves=fallback)
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
            if self.cache_jacobian:
                if ms._solve is not _solve_dispatch:
                    raise RuntimeError("Dense Hessian solve hook was replaced")
                ms._solve = self.original_solve
            ms._update_gradient = self.original
            _owner = None
        self.api.close()
        self.cusolver_api.destroy(self.handle)
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
