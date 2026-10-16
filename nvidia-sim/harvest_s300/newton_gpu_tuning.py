"""Opt-in launch/compiler tuning for the installed MuJoCo Warp Newton solver.

Call before the first step or CUDA graph capture. No installed library files,
collision geometry, materials, solver tolerance or physical constants change.
Compiler choices may change floating-point operation order: actual physics
validity, repeatability and performance still require GPU experiments.

Upstream deliberately defaults these tiny transpose-heavy products to scalar
GEMM for runtime and JIT efficiency (not a missing acceleration setting):
https://github.com/google-deepmind/mujoco_warp/pull/1402
"""
from __future__ import annotations

import warp as wp
from mujoco_warp._src import solver as mw_solver
from mujoco_warp._src import types as mw_types


def configure_blocked_cholesky(solver, *, block_dim=None, mathdx_gemm=None,
                               mathdx_solver=None):
    """Configure only the blocked Newton Cholesky kernel, preserving defaults.

    ``mathdx_gemm`` selects cuBLASDx for tile products; ``mathdx_solver``
    selects cuSolverDx for the 16x16 diagonal factor/triangular solves.
    These flags request implementations, not proof they are accelerated:
    compilation may fall back if MathDx is unavailable.

    The native solver still uses float32 matrices/vectors. This helper does
    not request float16/bfloat16 or TF32, but cannot promise bitwise equality
    between scalar and MathDx implementations. It does not accelerate a full
    matrix factorization using multiple SMs: launch remains one CTA/world.
    """
    if getattr(solver, "use_mujoco_cpu", True):
        raise ValueError("Blocked Cholesky GPU tuning requires the GPU backend")
    if int(getattr(solver, "_step", 0)) != 0:
        raise ValueError("Configure GPU kernels before stepping or graph capture")
    for name, value in (("mathdx_gemm", mathdx_gemm), ("mathdx_solver", mathdx_solver)):
        if value is not None and not isinstance(value, bool):
            raise ValueError(f"{name} must be bool or None")
    if block_dim is not None and (isinstance(block_dim, bool) or not isinstance(block_dim, int)
                                  or block_dim < 32 or block_dim > 1024 or block_dim % 32):
        raise ValueError("block_dim must be a warp multiple from 32 through 1024")
    model = solver.mjw_model
    if model.opt.solver != mw_types.SolverType.NEWTON:
        raise ValueError("Blocked Cholesky tuning requires the Newton constraint solver")
    if model.nv <= mw_solver._BLOCK_CHOLESKY_DIM:
        raise ValueError("This model uses the small unblocked Cholesky path")

    # Elliptic cones rebuild the Hessian each iteration. Other cones use the
    # incremental/cache variant, so tune the kernel actually launched there.
    incremental = model.opt.cone != mw_types.ConeType.ELLIPTIC
    if incremental:
        kernel = mw_solver._update_gradient_cholesky_blocked_skip_unchanged(
            mw_types.TILE_SIZE_JTDAJ_DENSE, model.nv_pad, model.nv, True)
        # The first solve still uses the ordinary factorization kernel.
        kernels = [mw_solver._update_gradient_cholesky_blocked(
            mw_types.TILE_SIZE_JTDAJ_DENSE, model.nv_pad, model.nv), kernel]
    else:
        kernels = [mw_solver._update_gradient_cholesky_blocked(
            mw_types.TILE_SIZE_JTDAJ_DENSE, model.nv_pad, model.nv)]
    requested = {}
    if mathdx_gemm is not None:
        requested["enable_mathdx_gemm"] = mathdx_gemm
    if mathdx_solver is not None:
        requested["enable_mathdx_solver"] = mathdx_solver
    changes = []
    for kernel in kernels:
        options = wp.get_module_options(module=kernel.module)
        before = {name: options.get(name) for name in
                  ("enable_mathdx_gemm", "enable_mathdx_solver", "fast_math")}
        if requested:
            wp.set_module_options(requested, module=kernel.module)
        after = wp.get_module_options(module=kernel.module)
        changes.append(dict(kernel=kernel.key, module=kernel.module.name, before=before,
                            after={name: after.get(name) for name in before}))
    previous_block_dim = int(model.block_dim.update_gradient_cholesky_blocked)
    if block_dim is not None:
        model.block_dim.update_gradient_cholesky_blocked = block_dim
    return dict(schema="s300-newton-gpu-cholesky-tuning-v1",
                enabled=bool(requested or block_dim is not None),
                requested=requested, original_block_dim=previous_block_dim,
                block_dim=int(model.block_dim.update_gradient_cholesky_blocked),
                nv=int(model.nv), matrix_size=int(model.nv_pad),
                tile_size=int(mw_types.TILE_SIZE_JTDAJ_DENSE),
                ctas_per_world=1, scalar_dtype="float32", incremental=incremental,
                kernels=changes, collider_or_physical_parameter_changes=False,
                mathdx_acceleration_verified=False, physics_equivalence_verified=False)
