"""Opt-in deterministic qfrc and gradient reductions for one native model.

The native state update, zeroing, change tracking and Hessian remain intact.
J.T*f is summed by one DOF writer in ascending constraint-row order. Gradient
and squared norm use the installed native tiled kernel, without atomics.
Float32 sum grouping changes; equations and physical parameters do not.
"""
from __future__ import annotations

import mujoco_warp as mw
from mujoco_warp._src import solver as ms, types as mt, constraint as mc
import warp as wp


@wp.kernel(enable_backward=False)
def _validate_rows(nv: int, nefc: wp.array[int], nnz: wp.array2d[int],
                   adr: wp.array2d[int], cols: wp.array3d[int], changed: wp.array[int],
                   done: wp.array[bool], failure: wp.array[int], counters: wp.array[int]):
    world, row = wp.tid()
    if done[world] or changed[world] == 0:
        return
    if row == 0:
        counters[0] += 1
        counters[1] = wp.max(counters[1], nefc[world])
        if nefc[world] < 0 or nefc[world] > nnz.shape[1]:
            wp.atomic_min(failure, 0, 0)
    if row >= nefc[world]:
        return
    count, start = nnz[world, row], adr[world, row]
    if count < 0 or start < 0 or start + count > cols.shape[2]:
        wp.atomic_min(failure, 0, row)
        return
    ascending = True
    if count > 1:
        ascending = cols[world, 0, start] < cols[world, 0, start + 1]
    previous = int(-1)
    for entry in range(count):
        col = cols[world, 0, start + entry]
        bad_order = entry > 0 and ((ascending and col <= previous) or (not ascending and col >= previous))
        if col < 0 or col >= nv or bad_order:
            wp.atomic_min(failure, 0, row)
        previous = col


@wp.kernel(enable_backward=False)
def _qfrc_fixed(nefc: wp.array[int], nnz: wp.array2d[int], adr: wp.array2d[int],
                 cols: wp.array3d[int], values: wp.array3d[float], force: wp.array2d[float],
                 changed: wp.array[int], done: wp.array[bool], failure: wp.array[int],
                 output: wp.array2d[float]):
    world, dof = wp.tid()
    if done[world] or changed[world] == 0:
        return
    if failure[0] != 2147483647:
        output[world, dof] = wp.nan
        return
    total = float(0.)
    for row in range(wp.min(nefc[world], nnz.shape[1])):
        f = force[world, row]
        if f == 0.:
            continue
        start = adr[world, row]
        ascending = True
        if nnz[world, row] > 1:
            ascending = cols[world, 0, start] < cols[world, 0, start + 1]
        low = int(0)
        high = nnz[world, row] + 0
        while low < high:
            middle = (low + high) // 2
            col = cols[world, 0, start + middle]
            if (ascending and col < dof) or (not ascending and col > dof):
                low = middle + 1
            else:
                high = middle
        if low < nnz[world, row]:
            if cols[world, 0, start + low] == dof:
                total += values[world, 0, start + low] * f
    output[world, dof] = total


@wp.kernel(enable_backward=False)
def _index_csr(nv: int, nefc: wp.array[int], nnz: wp.array2d[int], adr: wp.array2d[int],
               cols: wp.array3d[int], index: wp.array3d[int]):
    world, row = wp.tid()
    if row >= nefc[world]:
        return
    count, start = nnz[world, row], adr[world, row]
    if count < 0 or start < 0 or start + count > cols.shape[2]:
        return  # Existing per-rebuild validation latches the invalid row.
    for entry in range(count):
        col = cols[world, 0, start + entry]
        if col >= 0 and col < nv:
            index[world, row, col] = start + entry


@wp.kernel(enable_backward=False)
def _qfrc_indexed(nefc: wp.array[int], index: wp.array3d[int], values: wp.array3d[float],
                  force: wp.array2d[float], changed: wp.array[int], done: wp.array[bool],
                  failure: wp.array[int], output: wp.array2d[float]):
    world, dof = wp.tid()
    if done[world] or changed[world] == 0:
        return
    if failure[0] != 2147483647:
        output[world, dof] = wp.nan
        return
    total = float(0.)
    for row in range(wp.min(nefc[world], index.shape[1])):
        f = force[world, row]
        if f == 0.:
            continue
        address = index[world, row, dof]
        if address >= 0:
            total += values[world, 0, address] * f
    output[world, dof] = total


def validate_fixed_reductions(solver):
    if solver.use_mujoco_cpu or int(solver._step):
        raise ValueError("Install GPU fixed reductions before stepping")
    m, d = solver.mjw_model, solver.mjw_data
    if mw.__version__ != "3.15.0" or d.nworld != 1:
        raise ValueError("Fixed reductions require MW3.15 and one world")
    if (m.opt.solver != mt.SolverType.NEWTON or not m.is_sparse
            or m.opt.cone != mt.ConeType.ELLIPTIC):
        raise ValueError("Fixed reductions require sparse elliptic Newton")
    if m.nflex or m.ntendon or m.opt.enableflags & mt.EnableBit.SLEEP:
        raise ValueError("Fixed reductions exclude flex, tendon and compacted sleeping solves")
    return m


_owner = None


def _constraint_dispatch(m, d, ctx, track_changes=False, stable_fast=False):
    if _owner is None:
        raise RuntimeError("Fixed reductions dispatcher outlived its owner")
    if m is _owner.model:
        return _owner.constraint(m, d, ctx, track_changes=track_changes, stable_fast=stable_fast)
    return _owner.original_constraint(m, d, ctx, track_changes=track_changes, stable_fast=stable_fast)


def _gradient_dispatch(m, d, ctx, compact=False):
    if _owner is None:
        raise RuntimeError("Fixed reductions dispatcher outlived its owner")
    if m is _owner.model:
        return _owner.gradient(m, d, ctx, compact=compact)
    return _owner.original_gradient(m, d, ctx, compact=compact)


def _make_dispatch(m, d):
    result = _owner.original_make(m, d)
    if m is _owner.model:
        _owner.reindex(d)
    return result


class FixedSolverReductions:
    def __init__(self, solver, *, indexed=False):
        global _owner
        self.model = validate_fixed_reductions(solver)
        if _owner is not None:
            raise RuntimeError("Only one fixed reductions owner may be installed")
        self.device = solver.mjw_data.qpos.device
        if not self.device.is_cuda:
            raise ValueError("Fixed reductions require CUDA arrays")
        self.stream = wp.get_stream(self.device)
        self.indexed = indexed
        if self.stream.is_capturing:
            raise ValueError("Create fixed reductions outside capture")
        with wp.ScopedDevice(self.device):
            self.failure = wp.array([2147483647], dtype=int)
            self.counters = wp.zeros(2, dtype=int)
            self.index = wp.full((solver.mjw_data.nworld, solver.mjw_data.njmax,
                                 self.model.nv), -1, dtype=int) if indexed else None
            for kernel in (_validate_rows, _qfrc_fixed, ms._update_gradient_grad_tiled):
                wp.load_module(module=kernel.module, device=self.device)
        self.qfrc_kernel = ms._update_constraint_init_qfrc_constraint_sparse(False)
        self.gradient_kernel = ms._update_gradient_grad(False)
        self.original_constraint, self.original_gradient = ms._update_constraint, ms._update_gradient
        self.original_make = mc.make_constraint
        self.closed, self.active = False, False
        self.constraint_capture_calls, self.gradient_capture_calls = 0, 0
        self.evidence = dict(schema="s300-fixed-solver-reductions-v1", enabled=True,
            qfrc="one writer/DOF, ascending efc row sum, ascending-or-descending CSR binary lookup",
            gradient="installed native _update_gradient_grad_tiled",
            native_zeroing_and_change_stages_preserved=True,
            native_hessian_and_factorization_preserved=True,
            sorted_csr_and_capacity_checked_each_rebuild=True,
            accepted_column_orders="strict ascending or strict descending per row",
            invalid_csr_action="latched first row plus NaN; existing finite-state guards reject",
            silent_native_fallback=False,
            done_and_no_change_preserve_existing_values=True,
            float32_grouping_changed=True, physical_parameters_changed=False,
            installed_library_modified=False, farm_physics_verified=False)
        self.evidence['csr_index_rebuilt_per_physics_step'] = indexed
        _owner = self
        ms._update_constraint, ms._update_gradient = _constraint_dispatch, _gradient_dispatch
        if indexed:
            mc.make_constraint = _make_dispatch

    def reindex(self, d):
        self.index.fill_(-1)
        wp.launch(_index_csr, (d.nworld, d.njmax), inputs=[self.model.nv, d.nefc,
            d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, self.index], device=self.device)

    def _scope_call(self, original, selected_kernel, replacement, *args, **kwargs):
        if self.closed or self.active:
            raise RuntimeError("Closed or nested fixed reduction launch scope")
        if wp.get_stream(self.device).cuda_stream != self.stream.cuda_stream:
            raise RuntimeError("Fixed reductions stream changed")
        original_launch = wp.launch
        intercepted = 0
        def launch(*la, **lk):
            nonlocal intercepted
            kernel = lk.get("kernel", la[0] if la else None)
            if kernel is selected_kernel:
                intercepted += 1
                return replacement()
            return original_launch(*la, **lk)
        self.active = True
        wp.launch = launch
        try:
            result = original(*args, **kwargs)
        finally:
            wp.launch = original_launch
            self.active = False
        if intercepted != 1:
            raise RuntimeError(f"Expected one selected reduction kernel, received {intercepted}")
        return result

    def constraint(self, m, d, ctx, track_changes=False, stable_fast=False):
        if m is not self.model or d.nworld != 1:
            raise RuntimeError("Fixed reductions model/world mismatch")
        if getattr(ctx, "compact_d_full", None) is not None:
            raise RuntimeError("Fixed reductions do not cover compact solves")
        changed = ctx.state_changed_count if stable_fast else d.nefc
        def replace():
            wp.launch(_validate_rows, (d.nworld, d.njmax), inputs=[m.nv, d.nefc,
                d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, changed, ctx.done,
                self.failure, self.counters], device=self.device)
            if self.indexed:
                wp.launch(_qfrc_indexed, (d.nworld, m.nv), inputs=[d.nefc,
                    self.index, d.efc.J, d.efc.force, changed, ctx.done,
                    self.failure, d.qfrc_constraint], device=self.device)
            else:
                wp.launch(_qfrc_fixed, (d.nworld, m.nv), inputs=[d.nefc,
                    d.efc.J_rownnz, d.efc.J_rowadr, d.efc.J_colind, d.efc.J,
                    d.efc.force, changed, ctx.done, self.failure, d.qfrc_constraint], device=self.device)
        result = self._scope_call(self.original_constraint, self.qfrc_kernel,
            replace, m, d, ctx, track_changes=track_changes, stable_fast=stable_fast)
        self.constraint_capture_calls += 1
        return result

    def gradient(self, m, d, ctx, compact=False):
        if m is not self.model or d.nworld != 1:
            raise RuntimeError("Fixed reductions model/world mismatch")
        if compact or getattr(ctx, "compact_d_full", None) is not None:
            raise RuntimeError("Fixed reductions do not cover compact solves")
        def replace():
            wp.launch_tiled(ms._update_gradient_grad_tiled, dim=d.nworld,
                inputs=[m.nv, d.qfrc_smooth, d.qfrc_constraint, d.efc.Ma, ctx.done],
                outputs=[ctx.grad, ctx.grad_dot],
                block_dim=m.block_dim.update_gradient_grad, device=self.device)
        result = self._scope_call(self.original_gradient, self.gradient_kernel,
            replace, m, d, ctx, compact=compact)
        self.gradient_capture_calls += 1
        return result

    def check_health(self):
        row = int(self.failure.numpy()[0])
        if row != 2147483647:
            raise RuntimeError(f"Fixed reductions invalid/unsorted CSR or capacity at row {row}")

    def statistics(self):
        rebuilt, maximum = (int(v) for v in self.counters.numpy())
        values = dict(actual_qfrc_rebuilds=rebuilt, maximum_constraint_rows=maximum,
            constraint_capture_calls=self.constraint_capture_calls,
            gradient_capture_calls=self.gradient_capture_calls)
        failure = int(self.failure.numpy()[0])
        values["first_invalid_csr_row"] = None if failure == 2147483647 else failure
        self.evidence["runtime"] = values
        return values

    def close(self):
        global _owner
        if self.closed:
            return
        wp.synchronize_stream(self.stream)
        self.statistics()
        if _owner is self:
            if ms._update_constraint is not _constraint_dispatch or ms._update_gradient is not _gradient_dispatch:
                raise RuntimeError("Fixed reductions hooks replaced; close owners in reverse installation order")
            ms._update_constraint, ms._update_gradient = self.original_constraint, self.original_gradient
            if self.indexed:
                if mc.make_constraint is not _make_dispatch:
                    raise RuntimeError('Close profiler before indexed constraint hook')
                mc.make_constraint = self.original_make
            _owner = None
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
