"""Opt-in fixed-order tree reductions for the installed MuJoCo Warp 3.15.

The backend accumulates sibling contributions with floating-point atomics.
In ``levels`` mode each parent has one writer at each existing tree level;
children are summed in increasing body-ID order. In ``flat`` mode one writer
per body directly sums immutable intrinsic values of its descendants, reducing
launch count and changing float32 grouping. COM division, inertia equations,
RNE equations and integration remain unchanged in both modes. Neither uses
Warp's deterministic-atomic record buffers or claims that collision generation
or the constraint solver is deterministic.

Installation changes only in-memory Python dispatch, scoped to one backend
model. No installed-library files or physical model arrays are modified.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import wraps

import mujoco_warp as mw
from mujoco_warp._src import smooth
from mujoco_warp._src.types import vec10
import numpy as np
import warp as wp

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _sum_vec3(parents: wp.array[int], offsets: wp.array[int], children: wp.array[int],
              values: wp.array2d[wp.vec3]):
    world, group = wp.tid()
    parent = parents[group]
    total = values[world, parent]
    for index in range(offsets[group], offsets[group + 1]):
        total += values[world, children[index]]
    values[world, parent] = total


@wp.kernel
def _sum_vec10(parents: wp.array[int], offsets: wp.array[int], children: wp.array[int],
               values: wp.array2d[vec10]):
    world, group = wp.tid()
    parent = parents[group]
    total = values[world, parent]
    for index in range(offsets[group], offsets[group + 1]):
        total += values[world, children[index]]
    values[world, parent] = total


@wp.kernel
def _sum_spatial(parents: wp.array[int], offsets: wp.array[int], children: wp.array[int],
                 values: wp.array2d[wp.spatial_vector]):
    world, group = wp.tid()
    parent = parents[group]
    total = values[world, parent]
    for index in range(offsets[group], offsets[group + 1]):
        total += values[world, children[index]]
    values[world, parent] = total


@wp.kernel
def _flat_com(offsets: wp.array[int], descendants: wp.array[int],
              mass: wp.array2d[float], xipos: wp.array2d[wp.vec3], out: wp.array2d[wp.vec3]):
    world, body = wp.tid()
    total = wp.vec3(0.0)
    for index in range(offsets[body], offsets[body + 1]):
        child = descendants[index]
        total += xipos[world, child] * mass[world % mass.shape[0], child]
    out[world, body] = total


@wp.kernel
def _flat_crb(offsets: wp.array[int], descendants: wp.array[int],
              initial: wp.array2d[vec10], out: wp.array2d[vec10]):
    world, body = wp.tid()
    total = initial[world, body]
    # The installed CRB kernel deliberately does not accumulate into world.
    if body != 0:
        for index in range(offsets[body], offsets[body + 1]):
            child = descendants[index]
            if child != body:
                total += initial[world, child]
    out[world, body] = total


@wp.kernel
def _flat_rne(offsets: wp.array[int], descendants: wp.array[int],
              initial: wp.array2d[wp.spatial_vector], out: wp.array2d[wp.spatial_vector]):
    world, body = wp.tid()
    total = initial[world, body]
    for index in range(offsets[body], offsets[body + 1]):
        child = descendants[index]
        if child != body:
            total += initial[world, child]
    out[world, body] = total


@dataclass
class _Level:
    parents: wp.array
    offsets: wp.array
    children: wp.array
    host_parents: np.ndarray
    host_offsets: np.ndarray
    host_children: np.ndarray


def _plans(parent_ids, body_tree, device, include_world):
    """Group one existing level by parent; no writer reads another writer."""
    level_of = {}
    for level, ids in enumerate(body_tree):
        for body in ids:
            if int(body) in level_of:
                raise ValueError("body_tree contains a repeated body")
            level_of[int(body)] = level
    if set(level_of) != set(range(len(parent_ids))):
        raise ValueError("body_tree does not cover every body exactly once")
    for body in range(1, len(parent_ids)):
        if level_of[int(parent_ids[body])] >= level_of[body]:
            raise ValueError("body_tree has a non-ancestral level ordering")

    plans = []
    for ids in reversed(body_tree):
        groups = {}
        for body in sorted(int(item) for item in ids if int(item) != 0):
            parent = int(parent_ids[body])
            if include_world or parent != 0:
                groups.setdefault(parent, []).append(body)
        if not groups:
            continue
        parents = np.array(sorted(groups), np.int32)
        offsets = np.array([0] + list(np.cumsum([len(groups[int(p)]) for p in parents])), np.int32)
        children = np.array([c for p in parents for c in groups[int(p)]], np.int32)
        if set(parents) & set(children):
            raise ValueError("tree level aliases a parent writer with a child reader")
        plans.append(_Level(wp.array(parents, dtype=int, device=device),
                            wp.array(offsets, dtype=int, device=device),
                            wp.array(children, dtype=int, device=device),
                            parents, offsets, children))
    return plans


class FixedOrderTreeSums:
    """Install before capture/stepping; call ``close`` to restore dispatch.

    This is deliberately restricted to the reviewed 3.15 equations. The
    fixture controls also exercise multiple worlds and a non-selected model.
    Topology is read once at installation; changing body_parentid/body_tree
    afterwards is unsupported, just as for the backend's compiled topology.
    """

    def __init__(self, solver, mode="levels"):
        if getattr(solver, "use_mujoco_cpu", False):
            raise ValueError("fixed-order tree sums require Warp execution")
        if getattr(solver, "_step", 0) != 0:
            raise ValueError("install fixed-order tree sums before stepping")
        if mw.__version__ != "3.15.0":
            raise ValueError("fixed-order tree sums were reviewed for MuJoCo Warp 3.15.0")
        if mode not in ("levels", "flat"):
            raise ValueError("tree sum mode must be levels or flat")
        self.mode = mode
        self.model = solver.mjw_model
        if int(self.model.opt.enableflags) & int(mw.EnableBit.SLEEP):
            raise ValueError("fixed-order tree sums do not cover sleeping/compacted dynamics")
        if getattr(self.model, "has_fluid", False):
            raise ValueError("fixed-order tree sums do not cover fluid dynamics")
        self._closed = False
        parent_ids = self.model.body_parentid.numpy()
        body_tree = [ids.numpy() for ids in self.model.body_tree]
        device = self.model.body_parentid.device
        self.include_world = _plans(parent_ids, body_tree, device, True)
        self.exclude_world = _plans(parent_ids, body_tree, device, False)
        # Flat mode reads immutable intrinsic values, so every body can sum
        # its descendants independently in one kernel. It changes float32
        # grouping from hierarchical child sums to ascending body-ID sums.
        descendants = []
        self.flat_host_offsets = [0]
        for body in range(self.model.nbody):
            for child in range(self.model.nbody):
                ancestor = child
                while ancestor != body and ancestor != 0:
                    ancestor = int(parent_ids[ancestor])
                if ancestor == body:
                    descendants.append(child)
            self.flat_host_offsets.append(len(descendants))
        self.flat_host_offsets = np.asarray(self.flat_host_offsets, np.int32)
        self.flat_host_descendants = np.asarray(descendants, np.int32)
        self.flat_offsets = wp.array(self.flat_host_offsets, dtype=int, device=device)
        self.flat_descendants = wp.array(self.flat_host_descendants, dtype=int, device=device)
        self._rne_scratch = wp.empty((solver.mjw_data.nworld, self.model.nbody),
                                     dtype=wp.spatial_vector, device=device)
        self._original = {name: getattr(smooth, name)
                          for name in ("com_pos", "crb", "_rne_cfrc_backward")}
        self._public_original = {}
        self._wrappers = {}
        for name in self._original:
            original = self._original[name]
            replacement = {"com_pos": self._com_pos, "crb": self._crb,
                           "_rne_cfrc_backward": self._rne_backward}[name]

            @wraps(original)
            def wrapped(model, data, _original=original, _replacement=replacement):
                if model is self.model:
                    return _replacement(model, data)
                return _original(model, data)

            self._wrappers[name] = wrapped
            setattr(smooth, name, wrapped)
            # Public aliases are otherwise bound to the original function.
            if getattr(mw, name, None) is original:
                self._public_original[name] = original
                setattr(mw, name, wrapped)
        self.evidence = dict(
            enabled=True,
            mode=("fixed_parent_writer_existing_tree_levels" if mode == "levels"
                  else "fixed_flat_intrinsic_subtree_sums"),
            backend_version=mw.__version__,
            reductions=["subtree_com", "composite_body_inertia", "rne_internal_force"],
            sibling_order=("ascending body ID; existing deepest-to-root level order" if mode == "levels"
                           else "intrinsic descendant contributions in ascending body ID order"),
            float32_grouping_changed=mode == "flat",
            flat_descendant_entries=int(len(descendants)),
            com_rne_include_world_parent=True,
            crb_include_world_parent=False,
            body_count=int(self.model.nbody),
            tree_levels=len(body_tree),
            active_levels_including_world=len(self.include_world),
            active_levels_excluding_world=len(self.exclude_world),
            maximum_siblings=max((int(np.diff(p.host_offsets).max()) for p in self.include_world), default=0),
            floating_atomics_in_replacement_kernels=0,
            deterministic_atomic_record_buffers=0,
            physics_model_modified=False,
            full_pipeline_determinism_claimed=False,
        )

    @staticmethod
    def _accumulate(kernel, plans, values, nworld):
        for plan in plans:
            wp.launch(kernel, dim=(nworld, plan.parents.size),
                      inputs=[plan.parents, plan.offsets, plan.children, values])

    def _com_pos(self, m, d):
        if self.mode == "flat":
            wp.launch(_flat_com, dim=(d.nworld, m.nbody),
                      inputs=[self.flat_offsets, self.flat_descendants, m.body_mass, d.xipos],
                      outputs=[d.subtree_com])
        else:
            wp.launch(smooth._subtree_com_init, dim=(d.nworld, m.nbody),
                      inputs=[m.body_mass, d.xipos], outputs=[d.subtree_com])
            self._accumulate(_sum_vec3, self.include_world, d.subtree_com, d.nworld)
        wp.launch(smooth._subtree_div, dim=(d.nworld, m.nbody),
                  inputs=[m.body_subtreemass, d.subtree_com], outputs=[d.subtree_com])
        wp.launch(smooth._cinert, dim=(d.nworld, m.nbody),
                  inputs=[m.body_rootid, m.body_mass, m.body_inertia, d.xipos, d.ximat, d.subtree_com],
                  outputs=[d.cinert])
        wp.launch(smooth._cdof, dim=(d.nworld, m.njnt),
                  inputs=[m.body_rootid, m.jnt_type, m.jnt_dofadr, m.jnt_bodyid,
                          d.xmat, d.xanchor, d.xaxis, d.subtree_com], outputs=[d.cdof])

    def _crb(self, m, d):
        if self.mode == "flat":
            wp.launch(_flat_crb, dim=(d.nworld, m.nbody),
                      inputs=[self.flat_offsets, self.flat_descendants, d.cinert], outputs=[d.crb])
        else:
            wp.copy(d.crb, d.cinert)
            self._accumulate(_sum_vec10, self.exclude_world, d.crb, d.nworld)
        d.M.zero_()
        wp.launch(smooth._M, dim=(d.nworld, m.nv),
                  inputs=[m.dof_bodyid, m.dof_parentid, m.dof_armature,
                          m.M_rownnz, m.M_rowadr, d.cdof, d.crb], outputs=[d.M])

    def _rne_backward(self, m, d):
        if self.mode == "flat":
            if d.nworld != self._rne_scratch.shape[0]:
                raise ValueError("flat tree sums require the installed world count")
            wp.launch(_flat_rne, dim=(d.nworld, m.nbody),
                      inputs=[self.flat_offsets, self.flat_descendants, d.cfrc_int], outputs=[self._rne_scratch])
            wp.copy(d.cfrc_int, self._rne_scratch)
        else:
            self._accumulate(_sum_spatial, self.include_world, d.cfrc_int, d.nworld)

    def close(self):
        if self._closed:
            return
        for name, original in self._original.items():
            if getattr(smooth, name) is self._wrappers[name]:
                setattr(smooth, name, original)
            if name in self._public_original and getattr(mw, name) is self._wrappers[name]:
                setattr(mw, name, self._public_original[name])
        self._closed = True
