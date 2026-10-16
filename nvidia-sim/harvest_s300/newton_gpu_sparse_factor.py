"""Opt-in model-scoped fusion of MuJoCo Warp's sparse inertia LDL factor.

The existing sparse factorization equations and level dependencies are retained.
One CUDA CTA/world replaces one launch per level. A unique writer per target
entry sums source updates in fixed order, avoiding atomic-order races. This
changes floating-point summation order relative to unspecified GPU atomics;
full-trial validity and repeatability must be tested separately.
"""
from __future__ import annotations

from functools import lru_cache
import hashlib

import numpy as np
import warp as wp
from mujoco_warp._src import smooth


def build_factor_schedule(rownnz, rowadr, updates, level_offsets):
    """Host-only schedule preserving every source LDL update and level."""
    rownnz = np.asarray(rownnz, dtype=np.int32)
    rowadr = np.asarray(rowadr, dtype=np.int32)
    updates = np.asarray(updates, dtype=np.int32).reshape((-1, 3))
    offsets = np.asarray(level_offsets, dtype=np.int32)
    if rownnz.shape != rowadr.shape or rownnz.ndim != 1:
        raise ValueError("Sparse row addressing must be matching one-dimensional arrays")
    if offsets.ndim != 1 or len(offsets) < 1 or offsets[0] != 0 or np.any(np.diff(offsets) < 0):
        raise ValueError("Invalid source sparse-factor level offsets")
    if offsets[-1] > len(updates):
        raise ValueError("Sparse-factor offsets exceed the update buffer")
    targets, ordered_updates, target_offsets = [], [], [0]
    for lo, hi in zip(offsets[:-1], offsets[1:]):
        level = updates[lo:hi]
        if len(level) and set(level[:, 0]) & set(level[:, 1]):
            raise ValueError("Sparse-factor level contains a row dependency")
        groups = {}
        for index in range(int(lo), int(hi)):
            i, k, address = (int(v) for v in updates[index])
            if not (0 <= i < len(rownnz) and 0 <= k < len(rownnz)):
                raise ValueError("Sparse-factor update references an invalid row")
            if not (rowadr[k] <= address < rowadr[k] + rownnz[k] - 1):
                raise ValueError("Sparse-factor update must reference a strict ancestor entry")
            if rownnz[k] <= rownnz[i]:
                raise ValueError("Sparse-factor row does not contain the target ancestor prefix")
            groups.setdefault(i, []).append(index)
        for i, indices in groups.items():
            begin = len(ordered_updates)
            ordered_updates.extend(indices)
            end = len(ordered_updates)
            for j in range(int(rownnz[i])):
                targets.append((i, j, begin, end))
        target_offsets.append(len(targets))
    return dict(targets=np.asarray(targets, np.int32).reshape((-1, 4)),
                ordered_updates=np.asarray(ordered_updates, np.int32),
                target_offsets=np.asarray(target_offsets, np.int32),
                level_offsets=offsets, updates=updates,
                rownnz=rownnz, rowadr=rowadr)


@lru_cache(maxsize=None)
def _factor_kernel(nv, nlevels):
    @wp.func_native(snippet="WP_TILE_SYNC();")
    def barrier():
        pass

    @wp.kernel(module="unique", enable_backward=False, grid_stride=False,
               module_options={"fuse_fp": False})
    def factor(rownnz: wp.array[int], rowadr: wp.array[int], updates: wp.array[wp.vec3i],
               level_offsets: wp.array[int], targets: wp.array[wp.vec4i],
               target_offsets: wp.array[int], order: wp.array[int],
               L: wp.array2d[float], D: wp.array2d[float]):
        world, tid = wp.tid()
        threads = wp.block_dim()
        for reverse in range(wp.static(nlevels)):
            level = wp.static(nlevels) - 1 - reverse
            for target in range(target_offsets[level] + tid, target_offsets[level + 1], threads):
                item = targets[target]
                i, j = item[0], item[1]
                acc = L[world, rowadr[i] + j]
                for entry in range(item[2], item[3]):
                    update = updates[order[entry]]
                    k, ki = update[1], update[2]
                    tmp = L[world, ki] / L[world, rowadr[k] + rownnz[k] - 1]
                    product = L[world, rowadr[k] + j] * tmp
                    acc = acc - product
                L[world, rowadr[i] + j] = acc
            barrier()
            # A(k,i) remains unmodified until every destination entry has
            # consumed its numerator, including the target diagonal i=i.
            for entry in range(level_offsets[level] + tid, level_offsets[level + 1], threads):
                update = updates[entry]
                k, ki = update[1], update[2]
                L[world, ki] = L[world, ki] / L[world, rowadr[k] + rownnz[k] - 1]
            barrier()
        for dof in range(tid, wp.static(nv), threads):
            D[world, dof] = 1.0 / L[world, rowadr[dof] + rownnz[dof] - 1]

    return factor


_owner = None


def _dispatch(m, d, M, L, D):
    if _owner is None:
        raise RuntimeError("Fused sparse-factor dispatch outlived its owner")
    if m is _owner.model:
        return _owner.factor(m, d, M, L, D)
    return _owner.original(m, d, M, L, D)


class FusedSparseInertiaFactor:
    """Install before capture; retain until all captured graphs finish.

    CUDA launches exactly one CTA/world, with a uniform level loop and two
    block-wide barriers per level. Every thread reaches every barrier. CPU
    regression tests use block_dim=1. Other models use the original function.
    No fallback is silently used for the selected model if its shape changes.
    """
    def __init__(self, solver, *, block_dim=128):
        global _owner
        if solver.use_mujoco_cpu:
            raise ValueError("Fused sparse factor requires the native Warp backend")
        if int(getattr(solver, "_step", 0)):
            raise ValueError("Install fused sparse factor before stepping or capture")
        if _owner is not None:
            raise RuntimeError("A fused sparse-factor owner is already installed")
        if isinstance(block_dim, bool) or not isinstance(block_dim, int) or \
                block_dim < 32 or block_dim > 1024 or block_dim % 32:
            raise ValueError("block_dim must be a warp multiple from 32 through 1024")
        self.model = solver.mjw_model
        self.device = solver.mjw_data.qpos.device
        if self.device.is_cuda and wp.get_stream(self.device).is_capturing:
            raise RuntimeError("Install fused sparse factor before CUDA graph capture")
        source = solver.mj_model
        if self.model.nv != source.nv:
            raise ValueError("Native and Warp model DOF counts differ")
        offsets = self.model.qLD_level_offsets.numpy()
        self.schedule = build_factor_schedule(source.M_rownnz, source.M_rowadr,
            self.model.qLD_all_updates.numpy(), offsets)
        self.nlevels = len(offsets) - 1
        if self.nlevels == 0:
            raise ValueError("The selected model has no sparse LDL updates to fuse")
        self.nv, self.nworld = int(source.nv), int(solver.mjw_data.nworld)
        self.block_dim = block_dim if self.device.is_cuda else 1
        self.closed = False
        self.calls = 0
        with wp.ScopedDevice(self.device):
            self.rownnz = wp.array(self.schedule["rownnz"], dtype=int, device=self.device)
            self.rowadr = wp.array(self.schedule["rowadr"], dtype=int, device=self.device)
            self.updates = wp.array(self.schedule["updates"], dtype=wp.vec3i, device=self.device)
            self.level_offsets = wp.array(offsets, dtype=int, device=self.device)
            self.targets = wp.array(self.schedule["targets"], dtype=wp.vec4i, device=self.device)
            self.target_offsets = wp.array(self.schedule["target_offsets"], dtype=int, device=self.device)
            self.order = wp.array(self.schedule["ordered_updates"], dtype=int, device=self.device)
        self.kernel = _factor_kernel(self.nv, self.nlevels)
        self.original = smooth._factor_i_sparse
        _owner = self
        smooth._factor_i_sparse = _dispatch
        digest = hashlib.sha256()
        for name in sorted(self.schedule):
            digest.update(name.encode())
            digest.update(self.schedule[name].tobytes())
        self.evidence = dict(schema="s300-newton-fused-sparse-inertia-v1", enabled=True,
            levels=self.nlevels, source_updates=int(offsets[-1]),
            destination_entries=len(self.schedule["targets"]), schedule_sha256=digest.hexdigest(),
            nv=self.nv, worlds=self.nworld, block_dim=self.block_dim,
            ctas_per_world=1, level_barriers=2 * self.nlevels,
            launches_replaced_per_factor=self.nlevels + 1, fused_launches_per_factor=1,
            fixed_source_update_order=True, atomic_additions=False, fuse_fp=False,
            scalar_dtype="float32", equations_or_physical_parameters_changed=False,
            installed_library_files_modified=False, selected_model_only=True,
            physics_equivalence_verified=False)

    def factor(self, m, d, M, L, D):
        if self.closed or m is not self.model or int(d.nworld) != self.nworld:
            raise RuntimeError("Fused factor called outside its configured model/worlds")
        if tuple(M.shape) != tuple(L.shape) or tuple(D.shape) != (self.nworld, self.nv):
            raise RuntimeError("Sparse-factor workspace shape changed")
        wp.copy(L, M)
        wp.launch(self.kernel, (self.nworld, self.block_dim), inputs=[
            self.rownnz, self.rowadr, self.updates, self.level_offsets, self.targets,
            self.target_offsets, self.order, L, D], device=self.device,
            block_dim=self.block_dim)
        self.calls += 1

    def close(self):
        global _owner
        if self.closed:
            return
        if self.device.is_cuda and wp.get_stream(self.device).is_capturing:
            raise RuntimeError("Do not remove fused sparse factor during CUDA graph capture")
        wp.synchronize_device(self.device)
        if _owner is self:
            if smooth._factor_i_sparse is not _dispatch:
                raise RuntimeError("Sparse factor hook was replaced by another owner")
            smooth._factor_i_sparse = self.original
            _owner = None
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
