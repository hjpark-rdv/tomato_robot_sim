"""Opt-in bounded deterministic execution for the pinned rigid S300 backend.

Newton's global ``deterministic_max_records`` is a sparse-Hessian bound; using
it for every lazily generated broadphase kernel allocates billions of records.
This adapter scopes Warp's deterministic launch entry points to one solver and
sets bounds from the *executed kernel's* loop structure and model topology.

Supported scope is deliberately narrow: MuJoCo Warp 3.15, discrete/Newton,
sparse elliptic constraints, CONNECT equalities, rigid bodies, scalar joint
actuators, no tendons/flex/sensors/sleeping/heightfields. Unknown feature sets
are rejected. Native equations, contacts and physical parameters are unchanged.
Warp's deterministic record-overflow checks remain active.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import math
from types import MethodType

import mujoco as mj
import mujoco_warp as mw
import numpy as np
import warp as wp
from mujoco_warp._src.types import vec5


_ACTIVE = None
_SUPPORTED_MODULES = frozenset({
    "smooth", "support", "constraint", "solver", "forward", "derivative",
    "passive", "collision_driver", "collision_core", "collision_convex",
    "collision_primitive", "gjk", "epa", "island", "sensor", "io",
})
_DYNAMICS_MODULES = frozenset({"smooth", "support", "constraint", "solver",
                               "forward", "derivative", "passive", "island"})


def _selected_source(scope, source):
    short = source.rsplit(".", 1)[-1]
    return (scope == "all" or scope == "solver" and short == "solver"
            or scope == "constraints" and short in ("constraint", "solver")
            or scope == "dynamics" and short in _DYNAMICS_MODULES)


def _ceildiv(a, b):
    return (int(a) + int(b) - 1) // int(b)


def _dimensions(dim):
    if isinstance(dim, (int, np.integer)):
        return (int(dim),)
    return tuple(int(v) for v in dim)


def _chain_union_width(model):
    chains = []
    for body in range(model.nbody):
        welded = int(model.body_weldid[body])
        dof = int(model.body_dofadr[welded] + model.body_dofnum[welded] - 1)
        chain = set()
        while dof >= 0:
            chain.add(dof)
            dof = int(model.dof_parentid[dof])
        chains.append(chain)
    return max((len(a | b) for a in chains for b in chains), default=1)


def validate_features(solver):
    """Host checks only; never initialize CUDA or advance simulation."""
    if solver.use_mujoco_cpu:
        raise ValueError("Bounded deterministic mode requires MuJoCo Warp execution")
    if int(solver._step) != 0:
        raise ValueError("Install bounded deterministic execution before stepping or capture")
    if solver._deterministic != wp.DeterministicMode.RUN_TO_RUN:
        raise ValueError("Bounded execution requires deterministic=RUN_TO_RUN")
    if mw.__version__ != "3.15.0":
        raise ValueError("Bound proofs currently target exactly MuJoCo Warp 3.15.0")
    m = solver.mj_model
    if m.ntendon or m.nflex or m.nsensor or m.nhfield:
        raise ValueError("Bound proofs require no tendons, flex, sensors or heightfields")
    if int(m.opt.integrator) != int(mj.mjtIntegrator.mjINT_DISCRETE):
        raise ValueError("Bound proofs require the existing discrete integrator")
    if int(m.opt.solver) != int(mj.mjtSolver.mjSOL_NEWTON):
        raise ValueError("Bound proofs require the existing Newton constraint solver")
    if int(m.opt.cone) != int(mj.mjtCone.mjCONE_ELLIPTIC):
        raise ValueError("Bound proofs require elliptic contact groups")
    if not mj.mj_isSparse(m):
        raise ValueError("Bound proofs require sparse constraint Jacobians")
    if int(m.opt.enableflags) & int(mj.mjtEnableBit.mjENBL_SLEEP):
        raise ValueError("Bound proofs do not cover sleeping/compacted solves")
    if np.any(m.eq_type != mj.mjtEq.mjEQ_CONNECT):
        raise ValueError("Bound proofs require only CONNECT equalities")
    if np.any(m.actuator_trntype != mj.mjtTrn.mjTRN_JOINT):
        raise ValueError("Bound proofs require direct JOINT actuator transmission")
    if m.nu and np.any(~np.isin(m.jnt_type[m.actuator_trnid[:, 0]],
                               [mj.mjtJoint.mjJNT_HINGE, mj.mjtJoint.mjJNT_SLIDE])):
        raise ValueError("Bound proofs require scalar hinge/slide actuator moments")
    if not solver.mjw_model.opt.run_collision_detection:
        raise ValueError("Bound proofs require native collision detection")
    return m


@dataclass(frozen=True)
class RigidBounds:
    nv: int
    ngeom: int
    row_width: int
    group_capacity: int
    naconmax: int

    @classmethod
    def from_solver(cls, solver):
        m = validate_features(solver)
        d = solver.mjw_data
        # CONNECT: one three-row group/equality. Joint friction and limits:
        # at most one group per enabled DOF/joint. Elliptic contacts: one group
        # per contact, even when their rows have condim > 1. Contact capacity
        # is global, so it is a conservative per-world upper bound as well.
        groups = (int(m.neq) + int(np.count_nonzero(m.dof_frictionloss > 0))
                  + int(np.count_nonzero(m.jnt_limited)) + int(d.naconmax))
        return cls(int(m.nv), int(m.ngeom), max(1, _chain_union_width(m)),
                   min(int(d.njmax), groups), int(d.naconmax))

    def records(self, source_module, key, dim):
        """Maximum records per target/thread for supported executed kernels."""
        shape = _dimensions(dim)
        if "_nxn_broadphase" in key:
            return 1, "one candidate reservation per pair thread"
        if "_sap_broadphase" in key:
            return max(1, _ceildiv(self.ngeom - 1, 10)), "grid stride over at most n*(n-1)/2 pairs; 5*n threads/world"
        if "ccd_kernel" in key:
            return max(1, int(mj.mjMAXCONPAIR) * _ceildiv(self.naconmax, math.prod(shape))), "grid-stride candidate count times native maximum contacts/pair"
        if "_primitive_narrowphase" in key:
            return int(mj.mjMAXCONPAIR), "one primitive pair/thread; native maximum contacts/pair"
        if "_JTDACJ_sparse" in key:
            if len(shape) != 3 or shape[1] < 1 or shape[2] < 1:
                raise ValueError("Unsupported sparse Hessian launch topology")
            triangle = self.row_width * (self.row_width + 1) // 2
            return max(1, _ceildiv(self.group_capacity, shape[1]) * _ceildiv(triangle, shape[2])), "group stride times upper-triangular support entries/lane"
        if "_update_gradient_h_incremental_sparse" in key:
            return max(1, self.row_width * (self.row_width + 1) // 2), "one changed row/thread, upper-triangular support product"
        if "_update_constraint_init_qfrc_constraint_sparse" in key:
            return self.row_width, "one sparse constraint row/thread"
        if "_add_actuator_metric_dense" in key:
            return 1, "guarded scalar JOINT actuator moment has exactly one entry"
        if source_module == "mujoco_warp._src.constraint":
            # The reviewed rigid sparse kernels allocate one row/group block
            # per equality, friction DOF, limited joint or active contact.
            # Each thread touches a particular counter at most once; sparse
            # Jacobian construction writes its own allocated rows directly.
            return 1, "rigid sparse constraint assembly: at most one reservation per counter/thread"
        if any(token in key for token in ("_linesearch_jv_fused", "_solve_init_jaref",
                                          "_update_gradient_grad", "_update_constraint_efc")):
            return 1, "one norm/row reduction or state reservation per thread"
        short_module = source_module.rsplit(".", 1)[-1]
        if short_module not in _SUPPORTED_MODULES:
            raise ValueError("Unreviewed MuJoCo Warp deterministic kernel module: " + source_module)
        # Under the explicit feature guards, remaining dynamic atomic loops
        # walk a single DOF chain/row (at most nv). Quadratic sparse-Hessian
        # loops and grid-stride collision loops are handled above. Static
        # analysis may demand a larger fixed unrolled count; Warp takes max.
        return max(1, self.nv), "guarded rigid backend: single DOF-chain/row traversal; static codegen bound retained"


class BoundedDeterministicExecution:
    """Own an opt-in per-solver launch scope; close after captured work ends."""

    def __init__(self, solver, *, scope="solver", native_integer_atomics=False):
        if scope not in ("solver", "constraints", "dynamics", "all"):
            raise ValueError("deterministic scope must be solver, constraints, dynamics or all")
        self.selected_scope = scope
        self.bounds = RigidBounds.from_solver(solver)
        self.solver = solver
        self.native_integer_atomics = native_integer_atomics
        from mujoco_warp._src import solver as native_solver
        # These specialized sparse/elliptic kernels contain no active floating
        # scatter sum or atomic reservation whose returned order is consumed.
        self.no_scatter_kernels = {
            native_solver._update_constraint_efc(False),
            native_solver._linesearch_jv_fused_kernel(True, self.bounds.nv, self.bounds.nv, False),
        }
        self.original_scope = solver._scoped_deterministic_config
        self.original_bound = solver._deterministic_max_records
        self.original_options = {m: dict(o) for m, o in solver._module_options.items()}
        self.closed = False
        self.active = False
        self.evidence = dict(schema="s300-bounded-deterministic-v1", enabled=True,
                             deterministic="RUN_TO_RUN", scope=scope, global_max_records=0,
                             full_pipeline_determinism_claimed=False,
                             model_bound=self.bounds.__dict__, kernel_bounds={},
                             installed_library_modified=False,
                             physical_parameters_changed=False,
                             overflow_checks_preserved=True,
                             in_kernel_scatter_dependency_guard=True,
                             repeatability_verified=False)
        self.evidence['native_integer_atomics'] = native_integer_atomics
        solver._deterministic_max_records = 0
        # Override stored source-module bounds too, so Newton's normal option
        # reapplication cannot restore the old global Hessian bound to smooth.
        for module, options in list(solver._module_options.items()):
            if getattr(module, "__name__", "").startswith("mujoco_warp._src."):
                selected = _selected_source(scope, module.__name__)
                mode = wp.DeterministicMode.RUN_TO_RUN if selected else wp.DeterministicMode.NOT_GUARANTEED
                solver._set_module_options(dict(options, deterministic=mode,
                                               deterministic_max_records=max(1, self.bounds.nv) if selected else 0), module)

        owner = self

        def scoped(_solver):
            return owner.scope()

        solver._scoped_deterministic_config = MethodType(scoped, solver)

    def _configure(self, kernel, dim):
        source = getattr(getattr(kernel, "func", None), "__module__", "")
        if not source.startswith("mujoco_warp._src."):
            return
        key = str(kernel.key)
        selected = _selected_source(self.selected_scope, source)
        integer_native = self.native_integer_atomics and source == 'mujoco_warp._src.solver' and (
            kernel in self.no_scatter_kernels or key.startswith('_linesearch_iterative_kernel__')
            or key.startswith('_solve_done__'))
        if integer_native:
            selected = False
        if selected and "_solve_LD_sparse_fused" in key:
            # Warp deterministic scatter atomics with discarded return values
            # are reduced after the kernel, but this backend kernel reads its
            # atomic_sub outputs at the next depth inside the same kernel.
            # WP_TILE_SYNC is not an end-of-kernel scatter-reduce barrier.
            raise ValueError("Deferred deterministic scatter is unsafe for in-kernel sparse LD solve dependencies; "
                             "use the constraints scope and an explicitly deterministic inertia solve")
        if selected:
            bound, proof = self.bounds.records(source, key, dim)
            mode = wp.DeterministicMode.RUN_TO_RUN
        else:
            bound, proof = 0, "outside explicitly selected deterministic diagnostic scope"
            mode = wp.DeterministicMode.NOT_GUARANTEED
            if integer_native:
                proof = ('MW3.15 reviewed specialization: integer OR/decrement only, or '
                         'no active atomic scatter in sparse elliptic specialization; '
                         'floating reductions and reservation atomics remain scoped')
        if math.prod(_dimensions(dim)) * bound > 2**31 - 1:
            raise ValueError("Proven deterministic bound exceeds supported int32 capacity: " + key)
        options = dict(deterministic=mode,
                       deterministic_max_records=bound)
        current = wp.get_module_options(kernel.module)
        if any(current.get(k) != v for k, v in options.items()):
            wp.set_module_options(options, module=kernel.module)
        self.evidence["kernel_bounds"][key] = dict(records_per_thread=bound,
                                                   mode=mode.name, launch_dim=list(_dimensions(dim)), proof=proof)

    @contextmanager
    def scope(self):
        global _ACTIVE
        if self.closed:
            raise RuntimeError("Bounded deterministic execution is closed")
        if _ACTIVE is not None:
            raise RuntimeError("Concurrent/nested bounded deterministic solver scopes are unsupported")
        original_launch, original_tiled = wp.launch, wp.launch_tiled
        original_mode, original_records = wp.config.deterministic, wp.config.deterministic_max_records

        def launch(*args, **kwargs):
            kernel = kwargs.get("kernel", args[0] if args else None)
            dim = kwargs.get("dim", args[1] if len(args) > 1 else None)
            if kernel is not None and dim is not None:
                self._configure(kernel, dim)
            return original_launch(*args, **kwargs)

        def tiled(*args, **kwargs):
            kernel = kwargs.get("kernel", args[0] if args else None)
            dim = kwargs.get("dim", args[1] if len(args) > 1 else None)
            if kernel is not None and dim is not None:
                self._configure(kernel, dim)
            return original_tiled(*args, **kwargs)

        _ACTIVE, self.active = self, True
        wp.config.deterministic = (wp.DeterministicMode.RUN_TO_RUN if self.selected_scope == "all"
                                   else wp.DeterministicMode.NOT_GUARANTEED)
        wp.config.deterministic_max_records = 0
        wp.launch, wp.launch_tiled = launch, tiled
        try:
            yield
        finally:
            wp.launch, wp.launch_tiled = original_launch, original_tiled
            wp.config.deterministic, wp.config.deterministic_max_records = original_mode, original_records
            _ACTIVE, self.active = None, False

    def close(self):
        if self.closed:
            return
        if self.active:
            raise RuntimeError("Cannot close an active deterministic launch scope")
        self.solver._scoped_deterministic_config = self.original_scope
        self.solver._deterministic_max_records = self.original_bound
        for module, options in self.original_options.items():
            self.solver._set_module_options(options, module)
        self.closed = True


@wp.struct
class _ContactArrays:
    dist: wp.array[float]
    pos: wp.array[wp.vec3]
    frame: wp.array[wp.mat33]
    includemargin: wp.array[float]
    friction: wp.array[vec5]
    solref: wp.array[wp.vec2]
    solreffriction: wp.array[wp.vec2]
    solimp: wp.array[vec5]
    dim: wp.array[int]
    geom: wp.array[wp.vec2i]
    flex: wp.array[wp.vec2i]
    elem: wp.array[wp.vec2i]
    vert: wp.array[wp.vec2i]
    efc_address: wp.array2d[int]
    worldid: wp.array[int]
    type: wp.array[int]
    geomcollisionid: wp.array[int]
    adhesion: wp.array[float]


_CONTACT_FIELDS = tuple(_ContactArrays.vars)


@wp.kernel(enable_backward=False)
def _pack_contact_order_key(nacon: wp.array[int], contact: _ContactArrays,
                            integer: wp.array2d[int], scalar: wp.array2d[float]):
    cid = wp.tid()
    if cid >= nacon[0]:
        return
    integer[cid, 0] = contact.worldid[cid]
    integer[cid, 1] = contact.geom[cid][0]
    integer[cid, 2] = contact.geom[cid][1]
    for j in range(3, 9):
        integer[cid, j] = -1
    if contact.flex.shape[0] > 0:
        integer[cid, 3] = contact.flex[cid][0]
        integer[cid, 4] = contact.flex[cid][1]
    if contact.elem.shape[0] > 0:
        integer[cid, 5] = contact.elem[cid][0]
        integer[cid, 6] = contact.elem[cid][1]
    if contact.vert.shape[0] > 0:
        integer[cid, 7] = contact.vert[cid][0]
        integer[cid, 8] = contact.vert[cid][1]
    integer[cid, 9] = contact.dim[cid]
    integer[cid, 10] = contact.type[cid]
    integer[cid, 11] = contact.geomcollisionid[cid]
    scalar[cid, 0] = contact.dist[cid]
    for j in range(3):
        scalar[cid, 1 + j] = contact.pos[cid][j]
    for row in range(3):
        for col in range(3):
            scalar[cid, 4 + 3 * row + col] = contact.frame[cid][row, col]
    scalar[cid, 13] = contact.includemargin[cid]
    for j in range(5):
        scalar[cid, 14 + j] = contact.friction[cid][j]
        scalar[cid, 23 + j] = contact.solimp[cid][j]
    for j in range(2):
        scalar[cid, 19 + j] = contact.solref[cid][j]
        scalar[cid, 21 + j] = contact.solreffriction[cid][j]
    scalar[cid, 28] = contact.adhesion[cid]


@wp.kernel(enable_backward=False)
def _rank_contact_order(nacon: wp.array[int], integer: wp.array2d[int],
                        scalar_bits: wp.array2d[wp.uint32], rank: wp.array[int]):
    cid = wp.tid()
    count = wp.min(nacon[0], integer.shape[0])
    if cid >= count:
        return
    position = int(0)
    for other in range(count):
        comparison = int(0)
        for column in range(12):
            if comparison == 0:
                if integer[other, column] < integer[cid, column]:
                    comparison = -1
                elif integer[other, column] > integer[cid, column]:
                    comparison = 1
        for column in range(29):
            if comparison == 0:
                if scalar_bits[other, column] < scalar_bits[cid, column]:
                    comparison = -1
                elif scalar_bits[other, column] > scalar_bits[cid, column]:
                    comparison = 1
        # Equal keys contain bit-identical physical payload. The slot tie
        # breaker makes rank unique without changing the sorted payload.
        if comparison < 0 or comparison == 0 and other < cid:
            position += 1
    rank[cid] = position


@wp.func
def _copy_contact_row(source: _ContactArrays, destination: _ContactArrays, a: int, b: int):
    destination.dist[b] = source.dist[a]
    destination.pos[b] = source.pos[a]
    destination.frame[b] = source.frame[a]
    destination.includemargin[b] = source.includemargin[a]
    destination.friction[b] = source.friction[a]
    destination.solref[b] = source.solref[a]
    destination.solreffriction[b] = source.solreffriction[a]
    destination.solimp[b] = source.solimp[a]
    destination.dim[b] = source.dim[a]
    destination.geom[b] = source.geom[a]
    if source.flex.shape[0] > 0:
        destination.flex[b] = source.flex[a]
    if source.elem.shape[0] > 0:
        destination.elem[b] = source.elem[a]
    if source.vert.shape[0] > 0:
        destination.vert[b] = source.vert[a]
    destination.worldid[b] = source.worldid[a]
    destination.type[b] = source.type[a]
    destination.geomcollisionid[b] = source.geomcollisionid[a]
    destination.adhesion[b] = source.adhesion[a]
    for j in range(source.efc_address.shape[1]):
        destination.efc_address[b, j] = source.efc_address[a, j]


@wp.kernel(enable_backward=False)
def _gather_contact_order(nacon: wp.array[int], source: _ContactArrays,
                          destination: _ContactArrays, rank: wp.array[int]):
    cid = wp.tid()
    if cid < nacon[0]:
        _copy_contact_row(source, destination, cid, rank[cid])


@wp.kernel(enable_backward=False)
def _copy_contact_order(nacon: wp.array[int], source: _ContactArrays, destination: _ContactArrays):
    cid = wp.tid()
    if cid < nacon[0]:
        _copy_contact_row(source, destination, cid, cid)


class CanonicalContactOrder:
    """Order the exact contact multiset before constraints, without CCD lowering.

    The key includes every physical contact field; float components are ordered
    by their raw uint32 payload, preserving signed zero and NaN payloads. Only
    pre-constraint efc_address is excluded (it is rebuilt immediately afterward).
    Four small kernels perform pack/rank/gather/copy, O(capacity²) comparisons.
    Intended for the current small contact capacities, not unbounded dense
    scenes. Existing contact-filter and velocity-cache callbacks are preserved.
    """

    def __init__(self, solver):
        validate_features(solver)
        self.model, self.data = solver.mjw_model, solver.mjw_data
        if self.data.naconmax < 1 or self.data.naconmax > 4096:
            raise ValueError("Canonical rank ordering supports contact capacities 1..4096")
        expected = set(_CONTACT_FIELDS)
        actual = set(self.data.contact.__dataclass_fields__)
        if actual != expected:
            raise ValueError("Contact schema changed; canonical field permutation needs review")
        self.device = self.data.qpos.device
        self.live, self.scratch = _ContactArrays(), _ContactArrays()
        for field in _CONTACT_FIELDS:
            array = getattr(self.data.contact, field)
            setattr(self.live, field, array)
            setattr(self.scratch, field, wp.empty_like(array))
        capacity = int(self.data.naconmax)
        self.integer = wp.empty((capacity, 12), dtype=wp.int32, device=self.device)
        self.scalar = wp.empty((capacity, 29), dtype=wp.float32, device=self.device)
        self.scalar_bits = wp.array(ptr=self.scalar.ptr, shape=self.scalar.shape,
                                    dtype=wp.uint32, device=self.device)
        self.rank = wp.empty(capacity, dtype=wp.int32, device=self.device)
        self.original_callback = self.model.callback.contactfilter
        # If the stage-order fix is already installed, preserve its original
        # filter and run its pure cache refresh after permutation. That puts
        # both operations before constraints without applying either twice.
        from newton_gpu_compat import _ConnectVelocityCacheRefresh
        self.refresh_current_velocity = isinstance(self.original_callback, _ConnectVelocityCacheRefresh)
        self.previous = (self.original_callback.previous if self.refresh_current_velocity
                         else self.original_callback)
        self.model.callback.contactfilter = self._callback
        self.closed = False
        self.evidence = dict(schema="s300-canonical-contact-order-v1", enabled=True,
                             ordering="lexicographic integer fields then complete raw float32 payload",
                             contact_capacity=capacity, contact_set_changed=False,
                             physical_payload_changed=False, query_or_integration_added=False,
                             kernel_launches_per_collision_pass=4,
                             existing_contactfilter_preserved=self.previous is not None,
                             current_velocity_refresh_after_sort=self.refresh_current_velocity,
                             collision_counter_deterministic_lowering_required=False,
                             complete_repeatability_verified=False)

    def _callback(self, model, data):
        if self.previous is not None:
            self.previous(model, data)
        self.sort_contacts()
        if self.refresh_current_velocity:
            mw.com_vel(model, data)

    def sort_contacts(self):
        if self.closed:
            raise RuntimeError("Canonical contact ordering is closed")
        count = int(self.data.naconmax)
        wp.launch(_pack_contact_order_key, count,
                  inputs=[self.data.nacon, self.live, self.integer, self.scalar], device=self.device)
        wp.launch(_rank_contact_order, count,
                  inputs=[self.data.nacon, self.integer, self.scalar_bits, self.rank], device=self.device)
        wp.launch(_gather_contact_order, count,
                  inputs=[self.data.nacon, self.live, self.scratch, self.rank], device=self.device)
        wp.launch(_copy_contact_order, count,
                  inputs=[self.data.nacon, self.scratch, self.live], device=self.device)

    def close(self):
        if self.closed:
            return
        if self.model.callback.contactfilter != self._callback:
            raise RuntimeError("Canonical contact callback was replaced by another owner")
        self.model.callback.contactfilter = self.original_callback
        self.closed = True
