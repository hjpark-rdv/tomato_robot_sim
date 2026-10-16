"""Reuse cuSOLVER factors only for bit-identical authoritative upper Hessians.

Device comparisons and conditional graphs; no approximate hash/tolerance,
stale RHS, CPU solve, solver-iteration or physical-parameter change.
"""
import warp as wp
from newton_gpu_cusolver import CuSolverCholesky, _prepare_kernel, _finalize_kernel


@wp.kernel(enable_backward=False)
def _flags(nv: int, grad: wp.array2d[float], done: wp.array[bool],
           changed: wp.array[int], skip: bool, valid: wp.array[int],
           rhs: wp.array[float], enabled: wp.array[int], active: wp.array[bool],
           refactor: wp.array[int], counters: wp.array[int]):
    col = wp.tid()
    enable = not done[0] and (not skip or changed[0] != 0)
    rhs[col] = 0.
    if enable and col < nv:
        rhs[col] = grad[0, col]
    if col == 0:
        active[0] = enable
        enabled[0] = int(enable)
        refactor[0] = int(enable and valid[0] == 0)
        counters[0] += int(enable)
        counters[2] += int(not enable)


@wp.kernel(enable_backward=False)
def _compare(nv: int, current: wp.array3d[wp.uint32], cached: wp.array3d[wp.uint32],
             enabled: wp.array[int], refactor: wp.array[int]):
    row, col = wp.tid()
    if enabled[0] and row <= col and row < nv and col < nv:
        if current[0, row, col] != cached[0, row, col]:
            wp.atomic_max(refactor, 0, 1)


@wp.kernel(enable_backward=False)
def _stored(valid: wp.array[int], counters: wp.array[int]):
    valid[0] = 1
    counters[1] += 1


def words(array):
    return wp.array(ptr=array.ptr, shape=array.shape, strides=array.strides,
                    dtype=wp.uint32, device=array.device)


class CachedCuSolverCholesky(CuSolverCholesky):
    def __init__(self, solver):
        super().__init__(solver, precision='float32')
        self.cached = wp.zeros((1, self.n, self.n), float, device=self.device)
        self.valid, self.enabled, self.refactor = [wp.zeros(1, int, device=self.device) for _ in range(3)]
        self.counters = wp.zeros(3, int, device=self.device)
        self.evidence.update(unchanged_hessian_recomputed=False,
            factor_cache='GPU bitwise compare of complete authoritative upper H',
            cached_rhs=False, skip_inactive_factor_and_solve=True)

    def factorize_solve(self, m, d, ctx, skip_unchanged=False, skip_noflip=False):
        if self.closed or m is not self.model or int(d.nworld) != 1:
            raise RuntimeError('Cached cuSOLVER model/world mismatch')
        if wp.get_stream(self.device).cuda_stream != self.stream.cuda_stream:
            raise RuntimeError('Cached cuSOLVER stream changed')
        if ctx.h.shape != self.cached.shape or ctx.grad.shape != (1, self.n):
            raise RuntimeError('Cached cuSOLVER context shape mismatch')
        changed = ctx.state_changed_count if skip_noflip else d.nefc
        wp.launch(_flags, self.n, inputs=[self.nv, ctx.grad, ctx.done, changed,
            bool(skip_noflip), self.valid, self.rhs, self.enabled, self.active,
            self.refactor, self.counters], device=self.device)
        wp.launch(_compare, (self.nv, self.nv), inputs=[self.nv, words(ctx.h),
            words(self.cached), self.enabled, self.refactor], device=self.device)
        def factor():
            wp.launch(_prepare_kernel(wp.float32), (self.n, self.n), inputs=[
                self.nv, ctx.h, ctx.grad, ctx.done, changed, bool(skip_noflip),
                self.factor, self.rhs, self.active], device=self.device)
            self.api.factor(self.handle, self.n, self.factor.ptr, self.workspace.ptr,
                            self.workspace_count, self.factor_info.ptr)
            wp.copy(self.cached, ctx.h)
            wp.launch(_stored, 1, inputs=[self.valid, self.counters], device=self.device)
        wp.capture_if(self.refactor, on_true=factor)
        wp.capture_if(self.enabled, on_true=lambda: self.api.solve(self.handle,
            self.n, self.factor.ptr, self.rhs.ptr, self.solve_info.ptr))
        wp.launch(_finalize_kernel(wp.float32, self.nv, self.n), 1, inputs=[ctx.grad,
            self.rhs, self.active, self.factor_info, self.solve_info, self.first_failure,
            ctx.search, ctx.search_dot, ctx.newton_decrement], device=self.device)
        self.calls += 1

    def statistics(self):
        solves, factorizations, inactive = map(int, self.counters.numpy())
        self.evidence['runtime'] = dict(solves=solves, factorizations=factorizations,
            reused=solves-factorizations, inactive_calls=inactive)
        return self.evidence['runtime']
