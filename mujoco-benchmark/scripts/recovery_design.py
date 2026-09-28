"""Matched recovery experiments grounded in the d1fdc7c server evidence.

These helpers neither execute physics nor assign binary harvest labels. A clear
recorded prefix is a reusable control, not a reason to rerun global search.
"""
from __future__ import annotations
import copy
import hashlib
import math
import numpy as np
from scipy.spatial.transform import Rotation
from approach_connection import handoff_index, stitch

BASE_COMMIT = 'd1fdc7c092fda78b66a6ed5e0da09522a94a6a55'
SHARED_FAMILIES = ('under_center', 'flank_left', 'flank_right', 'pivot_sweep')
DESIGNS = ('recorded', 'neutral', 'neutral_pitch15')


def recorded_prefix(trace, start):
    """Contiguous ready/preapproach/approach only; never filter later rows in."""
    start = np.asarray(start, dtype=float)
    if start.ndim != 1 or not np.isfinite(start).all() or not trace or trace[0].get('phase') != 'ready':
        raise ValueError('A finite start and original ready row are required')
    cut = 1
    while cut < len(trace) and trace[cut].get('phase') in ('preapproach', 'approach'):
        cut += 1
    if cut < 2 or cut == len(trace):
        raise ValueError('Need a leading approach and a separate local suffix')
    if any(row.get('phase') in ('ready', 'preapproach', 'approach') for row in trace[cut:]):
        raise ValueError('Noncontiguous approach phase; review trace explicitly')
    rows = copy.deepcopy(trace[:cut])
    q = np.asarray([row['command'] for row in rows], dtype=float)
    if q.shape != (cut, len(start)) or not np.isfinite(q).all() or not np.array_equal(q[0], start):
        raise ValueError('Recorded prefix has wrong start, shape, or nonfinite values')
    # Matches the saved prefix_comparison.py: float64 commands, excluding ready.
    digest = hashlib.sha256(q[1:].tobytes()).hexdigest()
    return rows, dict(command_sha256=digest, samples=cut-1, seconds=(cut-1)/60.)


def matched_local_variant(params, geometry, design):
    """Change local orientation only, keep all approach waypoint poses/timing.

The neutral design is an ablation of local roll/pitch/entry twist together.
The next design changes ONLY local pitch relative to neutral. Final seating
position may change too because it is tied to the selected target/wire pair.
"""
    if design not in DESIGNS:
        raise ValueError('Unknown local design')
    if design == 'recorded':
        return copy.deepcopy(params)
    if params.get('family') not in SHARED_FAMILIES:
        raise ValueError('Fixed-prefix tilt comparison excludes side_mouth')
    from motion_family_search import make_candidate
    values = copy.deepcopy(params['search_parameters'])
    values.update(roll_deg=0., pitch_deg=15. if design == 'neutral_pitch15' else 0., entry_twist_deg=0.)
    result = make_candidate(params['family'], values, **geometry,
                            candidate_id=params['candidate_id'], parent_id=params.get('parent_id'))
    old_end, new_end = handoff_index(params), handoff_index(result)
    if old_end != new_end:
        raise ValueError('Local design changed approach waypoint count')
    for a, b in zip(params['pose_waypoints'][:old_end+1], result['pose_waypoints'][:new_end+1]):
        same = (a['phase'] == b['phase']
                and np.allclose(a['ring_position_xyz'], b['ring_position_xyz'], atol=1e-10, rtol=0)
                and (Rotation.from_quat(a['orientation_xyzw'])*Rotation.from_quat(b['orientation_xyzw']).inv()).magnitude() < 1e-10
                and abs(a.get('minimum_seconds', 0.)-b.get('minimum_seconds', 0.)) < 1e-9)
        if not same:
            raise ValueError('Local design changed approach; not a matched comparison')
    result['recovery_design'] = dict(design=design, source_parameters=copy.deepcopy(params['search_parameters']),
                                      approach_waypoints_unchanged=True, changed_goal_position_possible=True)
    return result


def join_recorded_prefix(prefix_rows, suffix, phases, start, limits):
    """Preserve original command AND phase history, unlike relabeling all as preapproach."""
    commands = np.asarray([row['command'] for row in prefix_rows], dtype=float)
    rows = stitch(commands, suffix, phases, start, limits=limits)
    for i, old in enumerate(prefix_rows):
        rows[i]['phase'] = old['phase']
    return rows


def select_evidence_groups(report, prefix_targets=4, local_targets=2, progress=None):
    """Dedupe shared prefixes before selecting matched, scene-spread controls.

Selection is for a diagnostic comparison, not an unbiased success-rate sample.
Unselected groups remain explicitly outside this round, not negative examples.
"""
    for n in (prefix_targets, local_targets):
        if isinstance(n, bool) or not isinstance(n, int) or not 0 <= n <= 12:
            raise ValueError('Target budgets must be integers in 0..12')
    rows = report['rows']; progress = progress or {}
    if report.get('groups') != len(rows):
        raise ValueError('Prefix report count mismatch')
    keys = set()
    for row in rows:
        members = row['members']
        if (len(members) != 4 or {m['family'] for m in members} != set(SHARED_FAMILIES)
                or len({m['command_sha256'] for m in members}) != 1
                or len({m['samples'] for m in members}) != 1
                or row.get('identical') is not True
                or not isinstance(row.get('all_four_blocked_in_prefix'), bool)):
            raise ValueError('Not a verified four-family shared prefix')
        digest = members[0]['command_sha256']
        if len(digest) != 64 or any(x not in '0123456789abcdef' for x in digest):
            raise ValueError('Malformed prefix digest')
        key = (row['scene'], row['target'], digest)
        if key in keys:
            raise ValueError('Duplicate prefix group')
        keys.add(key)
    selected, targets = [], set()
    for cohort, flag, count in (('prefix_blocked', True, prefix_targets), ('local_blocked', False, local_targets)):
        pool = [r for r in rows if r['all_four_blocked_in_prefix'] is flag]
        scenes = set()
        def advancement(r):
            return max((progress.get((r['scene'], r['target'], m['candidate']), 0) for m in r['members']), default=0)
        for _ in range(count):
            choices = [r for r in pool if (r['scene'], r['target']) not in targets]
            if not choices:
                break
            def order(r):
                az = float(r['parameters']['azimuth_deg']); el = float(r['parameters']['elevation_deg'])
                if not math.isfinite(az+el):
                    raise ValueError('Nonfinite recorded parameters')
                return (r['scene'] in scenes, -advancement(r) if not flag else 0,
                        abs(az)+abs(el), r['scene'], r['target'], r['members'][0]['command_sha256'])
            row = min(choices, key=order)
            member = (next(m for m in row['members'] if m['family'] == 'under_center') if flag else
                      min(row['members'], key=lambda m: (-progress.get((row['scene'], row['target'], m['candidate']), 0),
                                                         SHARED_FAMILIES.index(m['family']))))
            selected.append(dict(cohort=cohort, scene=row['scene'], target=row['target'],
                                 candidate=member['candidate'], family=member['family'],
                                 prefix_sha256=member['command_sha256'], prefix_samples=member['samples']))
            targets.add((row['scene'], row['target'])); scenes.add(row['scene'])
    return selected
