"""Canonicalize sparse constraint records after native parallel allocation.

No equation, contact or Jacobian value changes. Sparse arena addresses may
differ between runs; row pointers, group pointers and contact pointers move
together. This avoids deterministic replay of atomic row reservations.
"""
import warp as wp
from mujoco_warp._src import constraint
from newton_gpu_deterministic import validate_features


@wp.struct
class Rows:
    type: wp.array2d[int]
    id: wp.array2d[int]
    J_rownnz: wp.array2d[int]
    J_rowadr: wp.array2d[int]
    pos: wp.array2d[float]
    margin: wp.array2d[float]
    D: wp.array2d[float]
    vel: wp.array2d[float]
    aref: wp.array2d[float]
    frictionloss: wp.array2d[float]
    force: wp.array2d[float]
    state: wp.array2d[int]
    island: wp.array2d[int]
    Jqvel: wp.array2d[float]


@wp.kernel(enable_backward=False)
def _rank(nefc: wp.array[int], source: Rows, rank: wp.array2d[int]):
    world, row = wp.tid()
    count = wp.min(nefc[world], rank.shape[1])
    if row >= count:
        return
    position = int(0)
    t, ident = source.type[world, row], source.id[world, row]
    for other in range(count):
        ot, oi = source.type[world, other], source.id[world, other]
        if ot < t or (ot == t and (oi < ident or (oi == ident and other < row))):
            position += 1
    rank[world, row] = position


@wp.func
def _row(source: Rows, dest: Rows, w: int, a: int, b: int):
    dest.type[w, b] = source.type[w, a]
    dest.id[w, b] = source.id[w, a]
    dest.J_rownnz[w, b] = source.J_rownnz[w, a]
    dest.J_rowadr[w, b] = source.J_rowadr[w, a]
    dest.pos[w, b] = source.pos[w, a]
    dest.margin[w, b] = source.margin[w, a]
    dest.D[w, b] = source.D[w, a]
    dest.vel[w, b] = source.vel[w, a]
    dest.aref[w, b] = source.aref[w, a]
    dest.frictionloss[w, b] = source.frictionloss[w, a]
    dest.force[w, b] = source.force[w, a]
    dest.state[w, b] = source.state[w, a]
    dest.island[w, b] = source.island[w, a]
    dest.Jqvel[w, b] = source.Jqvel[w, a]


@wp.kernel(enable_backward=False)
def _gather(nefc: wp.array[int], source: Rows, dest: Rows, rank: wp.array2d[int]):
    world, row = wp.tid()
    if row < wp.min(nefc[world], rank.shape[1]):
        _row(source, dest, world, row, rank[world, row])


@wp.kernel(enable_backward=False)
def _copy(nefc: wp.array[int], source: Rows, dest: Rows):
    world, row = wp.tid()
    if row < wp.min(nefc[world], source.type.shape[1]):
        _row(source, dest, world, row, row)


@wp.kernel(enable_backward=False)
def _groups(count: wp.array[int], nefc: wp.array[int], rank: wp.array2d[int],
            adr: wp.array2d[int], nrow: wp.array2d[int],
            sorted_adr: wp.array2d[int], sorted_nrow: wp.array2d[int]):
    world, group = wp.tid()
    limit = wp.min(count[world], adr.shape[1])
    if group >= limit:
        return
    old = adr[world, group]
    if old < 0 or old >= wp.min(nefc[world], rank.shape[1]):
        return  # Existing capacity/finite-state guards reject overflow trials.
    new = rank[world, old]
    position = int(0)
    for other in range(limit):
        address = adr[world, other]
        if address >= 0 and address < wp.min(nefc[world], rank.shape[1]):
            if rank[world, address] < new:
                position += 1
    sorted_adr[world, position] = new
    sorted_nrow[world, position] = nrow[world, group]


@wp.kernel(enable_backward=False)
def _contact_addresses(nacon: wp.array[int], worldid: wp.array[int], nefc: wp.array[int],
                       rank: wp.array2d[int], adr: wp.array2d[int]):
    cid, component = wp.tid()
    if cid >= wp.min(nacon[0], adr.shape[0]):
        return
    w, old = worldid[cid], adr[cid, component]
    if old >= 0 and old < wp.min(nefc[w], rank.shape[1]):
        adr[cid, component] = rank[w, old]


class CanonicalConstraintOrder:
    def __init__(self, solver, *, allow_multiworld=False):
        validate_features(solver)
        self.model, self.data = solver.mjw_model, solver.mjw_data
        if self.data.nworld != 1 and not allow_multiworld:
            raise ValueError('Multiworld constraint canonicalization requires explicit diagnostic opt-in')
        expected = set(Rows.vars) | {'jtdaj_adr', 'jtdaj_nrow', 'jtdaj_nblock', 'J_colind', 'J', 'Ma'}
        if set(self.data.efc.__dataclass_fields__) != expected:
            raise ValueError('Constraint schema changed; permutation requires review')
        self.device = self.data.qpos.device
        self.live, self.scratch = Rows(), Rows()
        for field in Rows.vars:
            array = getattr(self.data.efc, field)
            setattr(self.live, field, array)
            setattr(self.scratch, field, wp.empty_like(array))
        self.rank = wp.empty((self.data.nworld, self.data.njmax), int, device=self.device)
        self.group_adr = wp.empty_like(self.data.efc.jtdaj_adr)
        self.group_nrow = wp.empty_like(self.data.efc.jtdaj_nrow)
        self.original = constraint.make_constraint
        self.closed = False
        constraint.make_constraint = self.make
        self.evidence = dict(schema='s300-canonical-constraint-order-v1',
            order='constraint type, identity, original component within one constraint',
            row_fields=list(Rows.vars), sparse_arena_values_unchanged=True,
            contact_and_group_addresses_remapped=True, capacity_guards_unchanged=True,
            physical_parameters_changed=False)
        if self.data.nworld != 1:
            self.evidence.update(nworld=self.data.nworld, multiworld_experimental=True)

    def make(self, m, d):
        result = self.original(m, d)
        if m is self.model:
            self.sort()
        return result

    def sort(self):
        d, device = self.data, self.device
        shape = (d.nworld, d.njmax)
        wp.launch(_rank, shape, inputs=[d.nefc, self.live, self.rank], device=device)
        wp.launch(_gather, shape, inputs=[d.nefc, self.live, self.scratch, self.rank], device=device)
        wp.launch(_groups, shape, inputs=[d.efc.jtdaj_nblock, d.nefc, self.rank,
            d.efc.jtdaj_adr, d.efc.jtdaj_nrow, self.group_adr, self.group_nrow], device=device)
        wp.launch(_contact_addresses, d.contact.efc_address.shape, inputs=[d.nacon,
            d.contact.worldid, d.nefc, self.rank, d.contact.efc_address], device=device)
        wp.launch(_copy, shape, inputs=[d.nefc, self.scratch, self.live], device=device)
        wp.copy(d.efc.jtdaj_adr, self.group_adr)
        wp.copy(d.efc.jtdaj_nrow, self.group_nrow)

    def close(self):
        if self.closed:
            return
        if constraint.make_constraint != self.make:
            raise RuntimeError('Close outer constraint hooks before canonicalization')
        constraint.make_constraint = self.original
        self.closed = True
