"""Optional Isaac 6.1 actor-level raw tensor contacts.

This reader is a diagnostic alternative to native collider callbacks. It does
not identify individual shapes of compound bodies. Its scalar force is the
normal contact force, not the impulse-vector magnitude or friction force.
Native A/B contact and penetration tests are required before accepting a
suppressReadback GPU run as equivalent to the collider callback policy.
"""
from __future__ import annotations

import math
import numpy as np


class ContactDataIncomplete(RuntimeError):
    """Contact capacity or identity prevents a complete, safe interpretation."""


def _numpy(value):
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy()
    if hasattr(value, 'numpy'):
        return value.numpy()
    return np.asarray(value)


def _cpu_id_tensor(original, values):
    """PhysX actor-ID lookup requires the original frontend on the CPU."""
    values = np.ascontiguousarray(values, dtype=np.uint64)
    if isinstance(original, np.ndarray):
        return values
    if hasattr(original, 'detach'):
        import torch
        # Installed 6.1 FrontendTorch maps API uint64 to torch.int64, and its
        # get_tensor_desc does not accept torch.uint64. Preserve the original
        # 64-bit storage pattern instead of inventing an unsupported dtype.
        # Actor IDs above 2**63 therefore remain bit-identical too.
        signed_storage = values.view(np.int64)
        return torch.as_tensor(signed_storage, dtype=original.dtype, device='cpu')
    if type(original).__module__.startswith('warp'):
        import warp as wp
        return wp.array(values, dtype=wp.uint64, device='cpu')
    raise TypeError('Unsupported contact actor-ID frontend: ' + str(type(original)))


class TensorContactReader:
    """Read every populated contact point and preserve actor-level evidence.

    ``read`` returns event dictionaries compatible with run_motion's a/b,
    separation_m, force_n, t and phase fields. Added metadata records the
    difference in granularity and measurement. ``last_summary`` holds normal
    force sums, capacity use and identity issues for the most recent step.
    Call ``clear_actor_cache`` after recreating physical actors or constraints
    whose native actor identity can change. This class does not reset physics.
    """

    def __init__(self, simulation_view, body_paths, *, max_contact_data_count=65536):
        paths = [str(path) for path in body_paths]
        if not paths or len(paths) != len(set(paths)):
            raise ValueError('Contact sensor paths must be nonempty and unique')
        if not isinstance(max_contact_data_count, int) or max_contact_data_count < 1:
            raise ValueError('Contact data capacity must be a positive integer')
        self.capacity = max_contact_data_count
        self.view = simulation_view.create_rigid_contact_view(
            paths, max_contact_data_count=max_contact_data_count)
        self.sensor_paths = [str(path) for path in self.view.sensor_paths]
        if set(self.sensor_paths) != set(paths) or len(self.sensor_paths) != len(paths):
            raise ContactDataIncomplete('Tensor contact sensor identity does not match requested bodies')
        if int(self.view.sensor_count) != len(self.sensor_paths):
            raise ContactDataIncomplete('Tensor contact sensor count differs from path inventory')
        self.sensor_set = set(self.sensor_paths)
        self.actor_paths = {}
        self.last_summary = {}
        self.physics_validated = False

    def clear_actor_cache(self):
        self.actor_paths.clear()

    def _lookup_paths(self, original_ids, used_ids):
        missing = sorted(set(int(value) for value in used_ids) - self.actor_paths.keys())
        if not missing:
            return
        looked_up = self.view.get_other_actor_paths_from_ids(
            _cpu_id_tensor(original_ids, missing))
        if len(looked_up) != len(missing):
            raise ContactDataIncomplete('Actor ID lookup returned a different number of paths')
        for actor_id, path in zip(missing, looked_up):
            # Empty IDs remain explicit unknown contacts and can be rechecked
            # after another step; they are never silently called free space.
            if path:
                self.actor_paths[actor_id] = str(path)

    def read(self, *, t, phase, dt):
        if not math.isfinite(t) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('Contact timestamp and positive physics dt must be finite')
        data = self.view.get_raw_contact_data(dt)
        if len(data) != 7:
            raise ContactDataIncomplete('Unexpected raw contact data schema')
        forces, points, normals, separations, counts, starts, actor_ids = data
        counts = _numpy(counts).astype(np.int64, copy=False).reshape(-1)
        starts = _numpy(starts).astype(np.int64, copy=False).reshape(-1)
        if len(counts) != len(self.sensor_paths) or len(starts) != len(counts):
            raise ContactDataIncomplete('Raw contact sensor ranges do not match sensor inventory')
        if np.any(counts < 0) or np.any(starts[counts > 0] < 0):
            raise ContactDataIncomplete('Negative raw contact count or start index')
        populated = [(i, int(start), int(count)) for i,(start,count) in enumerate(zip(starts,counts)) if count]
        used = int(counts.sum())
        last = max((start + count for _,start,count in populated), default=0)
        if used >= self.capacity or last > self.capacity:
            self.last_summary = dict(complete=False, raw_contact_points=used,
                                     capacity=self.capacity, reason='contact_capacity_saturated')
            raise ContactDataIncomplete('Raw contact buffer capacity is saturated; increase and rerun')
        if any(start + count > len(actor_ids) for _,start,count in populated):
            raise ContactDataIncomplete('Raw contact range exceeds returned actor buffer')
        ranges = sorted((start, start + count) for _,start,count in populated)
        if any(right[0] < left[1] for left,right in zip(ranges,ranges[1:])):
            raise ContactDataIncomplete('Raw contact sensor ranges overlap')
        if not used:
            self.last_summary = dict(complete=True, raw_contact_points=0,
                retained_contact_points=0, duplicate_sensor_points_removed=0,
                capacity=self.capacity, unknown_actor_ids=[],
                minimum_separation_m=None, raw_minimum_separation_m=None,
                normal_force_sums_by_actor_pair_n=[],
                contact_granularity='rigid_actor; individual collider unknown',
                force_metric='absolute scalar normal contact force N; friction excluded',
                native_ab_validated=self.physics_validated)
            return []
        # Preserve every valid point but avoid copying unpopulated tail buffers.
        force_array = _numpy(forces[:last]).reshape(-1)
        point_array = _numpy(points[:last]).reshape(-1,3)
        normal_array = _numpy(normals[:last]).reshape(-1,3)
        separation_array = _numpy(separations[:last]).reshape(-1)
        id_array = _numpy(actor_ids[:last]).astype(np.uint64, copy=False).reshape(-1)
        valid_indices = np.concatenate([np.arange(start,start+count) for _,start,count in populated])
        if any(not np.isfinite(values[valid_indices]).all() for values in
               (force_array, point_array, normal_array, separation_array)):
            raise ContactDataIncomplete('Nonfinite raw contact evidence')
        self._lookup_paths(actor_ids, id_array[valid_indices])
        unknown = sorted(set(int(value) for value in id_array[valid_indices]) - self.actor_paths.keys())
        events = []
        removed = 0
        sums = {}
        for sensor, start, count in populated:
            first = self.sensor_paths[sensor]
            for index in range(start, start + count):
                actor_id = int(id_array[index])
                second = self.actor_paths.get(actor_id)
                if second in self.sensor_set and first > second:
                    removed += 1
                    continue
                if second == first:
                    raise ContactDataIncomplete('Raw contact reports a sensor contacting the same rigid actor')
                other = second or f'UNKNOWN_ACTOR_ID:{actor_id}'
                scalar = float(force_array[index])
                force_n = abs(scalar)
                event = dict(t=float(t), phase=str(phase), a=first, b=other,
                    separation_m=float(separation_array[index]), force_n=force_n,
                    normal_force_signed_n=scalar,
                    point_m=point_array[index].tolist(), normal=normal_array[index].tolist(),
                    other_actor_id=actor_id,
                    contact_granularity='rigid_actor; individual collider unknown',
                    force_metric='absolute scalar normal contact force N; friction excluded',
                    collider_identity_known=False, actor_identity_known=second is not None,
                    native_ab_validated=self.physics_validated)
                events.append(event)
                pair = tuple(sorted((first,other)))
                sums[pair] = sums.get(pair, 0.) + force_n
        self.last_summary = dict(complete=not unknown, raw_contact_points=used,
            retained_contact_points=len(events), duplicate_sensor_points_removed=removed,
            capacity=self.capacity, unknown_actor_ids=unknown,
            minimum_separation_m=min((e['separation_m'] for e in events), default=None),
            # Use this un-deduplicated minimum for the penetration guard. Even
            # a slight numerical difference between symmetric sensor records
            # must not make a violation disappear when deduplicating forces.
            raw_minimum_separation_m=float(separation_array[valid_indices].min()),
            normal_force_sums_by_actor_pair_n=[dict(a=pair[0], b=pair[1], normal_force_sum_n=force)
                                            for pair,force in sorted(sums.items())],
            contact_granularity='rigid_actor; individual collider unknown',
            force_metric='sum of absolute scalar point normal forces N; friction excluded',
            native_ab_validated=self.physics_validated)
        return events
