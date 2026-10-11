"""SIM-GT ripe-group selection for the existing native motion runner.

The footprint predicts possible belt engagement, not reachability or detachment.
Legacy paired selection is unchanged. Unknown maturity is never counted ripe.
"""
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_policy(path, assembly, fruits):
    policy = json.loads(Path(path).read_text())
    if policy.get('schema') != 's300-harvest-group-policy-v1':
        raise ValueError('Unknown harvest group policy')
    n = policy.get('minimum_fruits')
    if isinstance(n, bool) or not isinstance(n, int) or not 2 <= n <= 10:
        raise ValueError('Group minimum must be an integer within 2..10')
    for name, expected in policy['source_sha256'].items():
        if name not in ('motion_manifest.json', 'composed.usdc') or sha(Path(assembly)/name) != expected:
            raise ValueError('Harvest eligibility source hash mismatch: '+name)
    if set(policy['source_sha256']) != {'motion_manifest.json', 'composed.usdc'}:
        raise ValueError('Manifest and composed model hashes are required')
    entries = policy['fruit_eligibility']
    if len({e['name'] for e in entries}) != len(entries):
        raise ValueError('Duplicate eligibility identity')
    by_name = {e['name']: e for e in entries}
    for fruit in fruits:
        e = by_name.get(fruit['name'])
        if e is None or e.get('path') != fruit['path'] or e.get('joint') != fruit['joint']:
            raise ValueError('Missing or mismatched exact fruit eligibility')
        if not isinstance(e.get('eligible'), bool):
            raise ValueError('Explicit boolean fruit eligibility is required')
    return policy


def harvest_group_outcome(policy, inventory):
    eligible={row['name'] for row in policy['fruit_eligibility'] if row['eligible']}
    detached=set(inventory['detached']);remaining=set(inventory['remaining'])
    return dict(eligible_detached=sorted(detached&eligible),
                ineligible_detached=sorted(detached-eligible),
                eligible_remaining=sorted(remaining&eligible),
                ineligible_remaining=sorted(remaining-eligible),
                interpretation='Native detachment counts; unripe incidental detachments remain visible; collection unverified')


class HarvestGroupSelector:
    """Highest qualifying local group, re-evaluated on measured cycle states.

    Width uses the open 96mm jaw gap minus each fruit's bounding radius.
    Depth uses the actual Y=0..158mm belt span minus the fruit radius.
    Height counts centers within the actual Z=13..61mm belt span: partial
    vertical overlap of a sphere is possible and is NOT full enclosure.
    Noneligible fruit centers in that footprint reject a proposed aim; nearby
    sphere overlap is recorded as a risk, never a no-unripe-contact guarantee.
    """
    def __init__(self, policy, direction, *, depth_m=.065, height_offset_m=0., output=None):
        self.minimum = policy['minimum_fruits']
        self.eligible = {e['name'] for e in policy['fruit_eligibility'] if e['eligible']}
        self.direction = direction
        self.depth_m = depth_m
        self.height_offset_m = height_offset_m
        self.output = Path(output) if output else None
        self.history = []
        self.last_decision = None

    def __call__(self, fruits, breaks, body_paths, poses, attempts, max_attempts=2,
                 order='highest-first', single_fruit_when_remaining=0, rejected_groups=()):
        from row_harvest_targets import truss_key
        from truss_sequence import inventory, validate_max_fruit_attempts, validated_rejected_groups
        validate_max_fruit_attempts(max_attempts)
        if order != 'highest-first' or single_fruit_when_remaining:
            raise ValueError('Group policy requires highest-first without single-fruit cleanup')
        rejected = validated_rejected_groups(fruits, rejected_groups)
        attached = set(inventory(fruits, breaks)['remaining'])
        candidates = []
        best_geometric_count = 0
        paths = list(body_paths)
        coords = np.asarray(poses, float)
        if len(set(paths)) != len(paths):
            raise ValueError('Ambiguous body paths')
        for key in sorted({truss_key(f['name']) for f in fruits}):
            subset = [f for f in fruits if truss_key(f['name']) == key and f['name'] in attached]
            if not subset: continue
            points = np.array([coords[paths.index(f['path']), :3] for f in subset])
            # Sphere bound remains valid after rotation of the measured fruit.
            radii = np.array([max(f['dimensions_m'])/2 for f in subset])
            if not np.isfinite(points).all() or not np.isfinite(radii).all() or np.any(radii <= 0):
                raise ValueError('Invalid fruit center or radius')
            direction = np.asarray(self.direction(key, fruits, breaks, paths, coords), float)
            if direction.shape != (3,) or not np.isfinite(direction).all() or abs(direction[2]) > 1e-8 or np.linalg.norm(direction) < 1e-9:
                raise ValueError('Finite horizontal group direction required')
            y = direction/np.linalg.norm(direction)
            rotation = np.column_stack((np.cross(y, [0., 0., 1.]), y, [0., 0., 1.]))
            names = [f['name'] for f in subset]
            ripe = np.array([name in self.eligible for name in names])
            available = ripe & np.array([attempts.get(name, 0) < max_attempts for name in names])
            # Finite candidate aims from individual centers and pair midpoints.
            # Each seed also probes the upper/lower belt-height boundary; there
            # is no fixed upper/lower pass count and no chain-connected group.
            seeds = [points[i] for i in np.flatnonzero(available)]
            seeds += [(points[i]+points[j])/2 for i,j in itertools.combinations(np.flatnonzero(available), 2)]
            seen = set()
            for seed in seeds:
                for dz in (0., -.024, .024):
                    aim = seed.copy(); aim[2] += dz
                    actual_center = aim + [0., 0., self.height_offset_m]
                    local = (points-actual_center)@rotation + [0., .158-self.depth_m, .037]
                    inside = ((np.abs(local[:,0])+radii <= .048+1e-10) &
                              (local[:,1]-radii >= -1e-10) & (local[:,1]+radii <= .158+1e-10) &
                              (local[:,2] >= .013-1e-10) & (local[:,2] <= .061+1e-10))
                    count = int(np.sum(inside & available))
                    if np.any(inside & ~ripe): continue
                    best_geometric_count = max(best_geometric_count, count)
                    if count < self.minimum: continue
                    ids = np.flatnonzero(inside & available)
                    group_names = tuple(sorted(names[i] for i in ids))
                    if group_names in rejected: continue
                    identity = (group_names, tuple(np.round(aim, 10)))
                    if identity in seen: continue
                    seen.add(identity)
                    nearby = ((np.abs(local[:,0])-radii <= .048) &
                              (local[:,1]+radii >= 0) & (local[:,1]-radii <= .158) &
                              (local[:,2]+radii >= .013) & (local[:,2]-radii <= .061))
                    centers = points[ids]
                    group = dict(fruits=[names[i] for i in ids], truss_key=key,
                        measured_centers_world_m=centers.tolist(),
                        measured_centroid_world_m=centers.mean(0).tolist(),
                        aim_world_m=aim.tolist(),
                        group_selection=dict(eligible_count=count, minimum_fruits=self.minimum,
                            direction_world=y.tolist(), belt_aim_world_m=actual_center.tolist(),
                            depth_m=self.depth_m, height_offset_m=self.height_offset_m,
                            fruit_centers_tool_m=local[ids].tolist(),
                            nearby_ineligible=[names[i] for i in np.flatnonzero(nearby & ~ripe)],
                            reachability='pending_existing_IK_collision_audit',
                            detachment='unknown_until_native_physics',
                            maturity_source='explicit_SIM_GT_eligibility',
                            training_eligible=False))
                    # Try unattempted groups first, then the highest ripe group.
                    rank = (max(attempts.get(name, 0) for name in group_names),
                            -float(centers[:,2].max()), -count,
                            float(np.linalg.norm(aim-centers.mean(0))), group_names, tuple(aim))
                    candidates.append((rank, group))
        selected = min(candidates, key=lambda x:x[0])[1] if candidates else None
        eligible_remaining = sorted(attached & self.eligible)
        reason = ('group_available' if selected else
                  'no_eligible_fruit_remaining' if not eligible_remaining else
                  'attempt_budget_exhausted' if all(attempts.get(n,0)>=max_attempts for n in eligible_remaining) else
                  'group_planning_alternatives_exhausted' if best_geometric_count >= self.minimum else
                  'no_group_meets_minimum')
        self.last_decision = dict(index=len(self.history), reason=reason,
            minimum_fruits=self.minimum, remaining=sorted(attached),
            eligible_remaining=eligible_remaining, ineligible_remaining=sorted(attached-self.eligible),
            best_geometric_count=best_geometric_count, candidate_aim_count=len(candidates),
            selected=selected, absolute_harvest_impossibility=False)
        self.history.append(self.last_decision)
        if self.output:
            self.output.write_text(json.dumps(self.history, indent=2)+'\n')
        return selected
