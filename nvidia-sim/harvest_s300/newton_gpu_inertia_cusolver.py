"""Opt-in GPU reverse-Cholesky implementation of native sparse M=L' D L.

No model or matrix entries change. A reversed dense copy is factored by CUDA
cuSOLVER, then mapped to the native unit-lower sparse factor and reciprocal D.
This has different floating-point grouping from native atomic LDL; full-trial
validity and repeatability remain separate required checks.
"""
from __future__ import annotations

import numpy as np
import warp as wp
import mujoco_warp as mw
from mujoco_warp._src import smooth

from newton_gpu_cusolver import CuSolverAPI, _identity_kernel


@wp.kernel
def _clear(nv: int, matrix: wp.array2d[float]):
    row, col = wp.tid()
    value = float(0.)
    if row == col and row >= nv:
        value = 1.
    matrix[row, col] = value


@wp.kernel
def _expand(nv: int, entries: wp.array[wp.vec2i], sparse: wp.array2d[float],
            matrix: wp.array2d[float]):
    address = wp.tid()
    entry = entries[address]
    # Row-major upper == column-major lower, as required by POTRF(uplo=0).
    matrix[nv - 1 - entry[0], nv - 1 - entry[1]] = sparse[0, address]


@wp.kernel
def _extract(nv: int, entries: wp.array[wp.vec2i], matrix: wp.array2d[float],
             info: wp.array[int], failure: wp.array[int],
             factor: wp.array2d[float], reciprocal: wp.array2d[float]):
    address = wp.tid()
    row, col = entries[address][0], entries[address][1]
    diagonal = matrix[nv - 1 - row, nv - 1 - row]
    value = matrix[nv - 1 - row, nv - 1 - col]
    if info[0] != 0 or not wp.isfinite(diagonal) or diagonal <= 0.:
        wp.atomic_cas(failure, 0, 0, 1)
        factor[0, address] = wp.nan
        if row == col:
            reciprocal[0, row] = wp.nan
    elif row == col:
        factor[0, address] = diagonal * diagonal
        reciprocal[0, row] = 1. / (diagonal * diagonal)
    else:
        factor[0, address] = value / diagonal


@wp.kernel
def _reverse_rhs(nv: int, original: wp.array2d[float], rhs: wp.array[float]):
    index = wp.tid()
    value = float(0.)
    if index < nv:
        value = original[0, nv - 1 - index]
    rhs[index] = value


@wp.kernel
def _reverse_solution(nv: int, rhs: wp.array[float], info: wp.array[int],
                      failure: wp.array[int], output: wp.array2d[float]):
    index = wp.tid()
    value = rhs[nv - 1 - index]
    if info[0] != 0 or not wp.isfinite(value):
        wp.atomic_cas(failure, 0, 0, 1)
        value = wp.nan
    output[0, index] = value


_owner = None


def _dispatch(m, d, matrix, factor, reciprocal):
    if _owner is None:
        raise RuntimeError('Reverse-Cholesky hook outlived its owner')
    if m is _owner.model:
        return _owner.factor(m, d, matrix, factor, reciprocal)
    return _owner.original(m, d, matrix, factor, reciprocal)


def _solve_dispatch(m, d, factor, reciprocal, output, rhs):
    if _owner is None:
        raise RuntimeError('Reverse-Cholesky solve hook outlived its owner')
    if m is _owner.model:
        return _owner.solve(m, d, factor, reciprocal, output, rhs)
    return _owner.original_solve(m, d, factor, reciprocal, output, rhs)


def _public_solve_dispatch(m, d, factor, reciprocal, output, rhs):
    if _owner is None:
        raise RuntimeError('Reverse-Cholesky public solve hook outlived its owner')
    if m is _owner.model:
        return _owner.public_solve(m, d, factor, reciprocal, output, rhs)
    return _owner.original_public_solve(m, d, factor, reciprocal, output, rhs)


class CuSolverInertiaFactor:
    def __init__(self, solver):
        global _owner
        if _owner is not None or solver.use_mujoco_cpu or int(solver._step) != 0:
            raise ValueError('One reverse-Cholesky GPU owner must be installed before stepping')
        if mw.__version__ != '3.15.0' or int(solver.mjw_data.nworld) != 1:
            raise ValueError('Reverse-Cholesky currently requires MW3.15 and one world')
        self.model, self.device = solver.mjw_model, solver.mjw_data.qpos.device
        if not self.device.is_cuda:
            raise ValueError('Reverse-Cholesky requires actual CUDA')
        self.stream = wp.get_stream(self.device)
        if self.stream.is_capturing:
            raise ValueError('Initialize reverse-Cholesky outside graph capture')
        source = solver.mj_model
        self.nv, self.n = int(source.nv), int(self.model.nv_pad)
        self.block_total=int(self.model.qLD_block_total)
        self.native_dense_only=self.block_total==solver.mjw_data.qLD.shape[1]
        if self.native_dense_only:
            # Each articulation fits an existing native dense block. There is
            # no sparse suffix to intercept (an empty Warp slice is invalid).
            # Keep the native factor/solve dispatchers entirely untouched.
            self.closed,self.calls=False,0
            self.evidence=dict(schema='s300-newton-cusolver-inertia-v1',enabled=False,
                nv=self.nv,padded=self.n,packed_dense_prefix=self.block_total,
                native_dense_blocks_only=True,sparse_suffix_entries=0,
                implementation='Unmodified native MuJoCo Warp dense-block factor/solve',
                physical_parameters_changed=False,host_solve=False,simulation_warmup_steps=0)
            return
        rows = np.repeat(np.arange(self.nv, dtype=np.int32), source.M_rownnz)
        cols = np.asarray(source.M_colind, dtype=np.int32)
        if len(rows) != len(cols) or np.any(cols > rows) or np.any(cols < 0):
            raise ValueError('Unexpected native sparse lower-triangle topology')
        self.entries = wp.array(np.c_[rows, cols], dtype=wp.vec2i, device=self.device)
        self.matrix = wp.empty((self.n, self.n), float, device=self.device)
        self.block_total=int(self.model.qLD_block_total)
        self.matrices = {array[:,self.block_total:].ptr:wp.empty_like(self.matrix) for array in
            (solver.mjw_data.qLD, solver.mjw_data.qHLD)}
        self.rhs = wp.empty(self.n, float, device=self.device)
        self.unaliased_rhs = wp.empty((1, self.nv), float, device=self.device)
        self.info = wp.zeros(1, int, device=self.device)
        self.solve_info = wp.zeros(1, int, device=self.device)
        self.failure = wp.zeros(1, int, device=self.device)
        self.api = CuSolverAPI(precision='float32')
        self.handle = self.api.create()
        try:
            self.api.set_stream(self.handle, self.stream.cuda_stream)
            self.api.set_deterministic(self.handle)
            self.workspace_count = self.api.buffer_size(self.handle, self.n, self.matrix.ptr)
            self.workspace = wp.empty(max(1, self.workspace_count), float, device=self.device)
            wp.launch(_identity_kernel(wp.float32), (self.n, self.n),
                inputs=[self.matrix, self.rhs], device=self.device)
            self._potrf()
            wp.synchronize_stream(self.stream)
            if int(self.info.numpy()[0]):
                raise RuntimeError('Reverse-Cholesky identity initialization failed')
        except BaseException:
            self.api.destroy(self.handle)
            raise
        self.original, self.closed, self.calls = smooth._factor_i_sparse, False, 0
        self.original_solve = smooth._solve_LD_sparse
        self.original_public_solve = smooth.solve_LD
        self.public_aliases = [(smooth, 'solve_LD')]
        if getattr(mw, 'solve_LD', None) is self.original_public_solve:
            self.public_aliases.append((mw, 'solve_LD'))
        _owner = self
        smooth._factor_i_sparse = _dispatch
        smooth._solve_LD_sparse = _solve_dispatch
        for module, name in self.public_aliases:
            setattr(module, name, _public_solve_dispatch)
        self.evidence = dict(schema='s300-newton-cusolver-inertia-v1', enabled=True,
            nv=self.nv, padded=self.n, source_sparse_entries=len(rows),
            packed_dense_prefix=self.block_total,
            library=self.api.path, equation="M=L' D L via reversed Cholesky",
            backsubstitution='cuSOLVER POTRS on retained reverse-Cholesky factor',
            public_solve_rhs_alias_preserved=True,
            float32_grouping_changed=True, host_solve=False,
            physical_parameters_changed=False, simulation_warmup_steps=0)

    def _potrf(self):
        self.api.factor(self.handle, self.n, self.matrix.ptr,
            self.workspace.ptr, self.workspace_count, self.info.ptr)

    def factor(self, m, d, matrix, factor, reciprocal):
        if self.closed or m is not self.model or d.nworld != 1:
            raise RuntimeError('Reverse-Cholesky model/world mismatch')
        if (matrix.shape != factor.shape or matrix.shape != (1, self.entries.size)
                or reciprocal.shape != (1, self.nv)):
            raise RuntimeError('Reverse-Cholesky native workspace shape mismatch')
        if wp.get_stream(self.device).cuda_stream != self.stream.cuda_stream:
            raise RuntimeError('Reverse-Cholesky stream changed')
        if factor.ptr not in self.matrices:
            raise RuntimeError('Unreviewed native inertia factor buffer')
        self.matrix=self.matrices[factor.ptr]
        wp.launch(_clear, (self.n, self.n), inputs=[self.nv, self.matrix], device=self.device)
        wp.launch(_expand, self.entries.size, inputs=[self.nv, self.entries, matrix, self.matrix], device=self.device)
        self._potrf()
        wp.launch(_extract, self.entries.size, inputs=[self.nv, self.entries, self.matrix,
            self.info, self.failure, factor, reciprocal], device=self.device)
        self.calls += 1

    def solve(self,m,d,factor,reciprocal,output,rhs):
        if self.closed or m is not self.model or d.nworld!=1 or factor.ptr not in self.matrices:
            raise RuntimeError('Reverse-Cholesky native solve buffer/model mismatch')
        if rhs.shape[0]!=1 or output.shape[0]!=1 or min(rhs.shape[1],output.shape[1])<self.nv:
            raise RuntimeError('Reverse-Cholesky native RHS/output shape mismatch')
        if output.ptr==rhs.ptr:
            raise RuntimeError('Reverse-Cholesky forbids RHS/output aliasing after native dense-block solves')
        if wp.get_stream(self.device).cuda_stream!=self.stream.cuda_stream:
            raise RuntimeError('Reverse-Cholesky solve stream changed')
        matrix=self.matrices[factor.ptr]
        wp.launch(_reverse_rhs,self.n,inputs=[self.nv,rhs,self.rhs],device=self.device)
        self.api.solve(self.handle,self.n,matrix.ptr,self.rhs.ptr,self.solve_info.ptr)
        wp.launch(_reverse_solution,self.nv,inputs=[self.nv,self.rhs,self.solve_info,
            self.failure,output],device=self.device)

    def public_solve(self, m, d, factor, reciprocal, output, rhs):
        # Native dense blocks are solved before the sparse dispatcher. Preserve
        # y before those kernels can overwrite it when solve_LD(...,x,x) is used
        # by discrete effective-inertia derivatives. No host readback or solve.
        if self.closed or m is not self.model or d.nworld != 1:
            raise RuntimeError('Reverse-Cholesky public solve model/world mismatch')
        if (rhs.shape[0] != 1 or output.shape[0] != 1 or
                min(rhs.shape[1], output.shape[1]) < self.nv):
            raise RuntimeError('Reverse-Cholesky public solve RHS/output shape mismatch')
        rhs_end = rhs.ptr + rhs.shape[1] * wp.types.type_size_in_bytes(rhs.dtype)
        output_end = output.ptr + output.shape[1] * wp.types.type_size_in_bytes(output.dtype)
        if rhs.ptr < output_end and output.ptr < rhs_end:
            wp.copy(self.unaliased_rhs, rhs[:, :self.nv])
            rhs = self.unaliased_rhs
        return self.original_public_solve(m, d, factor, reciprocal, output, rhs)

    def check_health(self):
        if self.native_dense_only:
            return
        if int(self.failure.numpy()[0]):
            raise RuntimeError('Reverse-Cholesky inertia factor failed')

    def close(self):
        global _owner
        if self.closed:
            return
        if self.native_dense_only:
            self.closed=True
            return
        wp.synchronize_stream(self.stream)
        if _owner is self:
            smooth._factor_i_sparse, _owner = self.original, None
            smooth._solve_LD_sparse=self.original_solve
            for module, name in self.public_aliases:
                setattr(module, name, self.original_public_solve)
        self.api.destroy(self.handle)
        self.closed = True
