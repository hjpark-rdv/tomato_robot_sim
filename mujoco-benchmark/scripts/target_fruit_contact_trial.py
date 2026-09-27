"""Explicitly authorized target-fruit brushing, for bounded OFFLINE SIM trials.

No production policy, collision mask, material, or harvest label is changed.
The old nominal checker skips only exact wire/target pairs in named phases.
A passed audit is NOT proof of light contact. --execute requires force and
motion stop bounds, observes live and private-forward forces separately, and
stops the simulated loop on violations. This is not a hardware safety stop.
"""
from __future__ import annotations
import argparse
import copy
from dataclasses import asdict, dataclass
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import numpy as np
from environment_preflight import Policy, check_engine, save_report

FRUIT_PHASES = frozenset(('insert', 'seat', 'hold', 'verify'))
PEDICEL_PHASES = frozenset(('seat', 'hold', 'verify'))
PHYSICS_LIMIT_M = 0.0005  # Existing physical validity limit; NOT a skin limit.


@dataclass(frozen=True)
class ContactScope:
    fruit: str
    wires: tuple
    pedicels: tuple
    rear_wires: tuple

    def __post_init__(self):
        for values in ((self.fruit,), self.wires, self.pedicels, self.rear_wires):
            if not values or len(set(values)) != len(values) or any(
                not isinstance(n, str) or not n or any(c in n for c in '*?[') for n in values
            ):
                raise ValueError('Exact, unique contact identities required')
        if not set(self.rear_wires).issubset(self.wires):
            raise ValueError('Rear wires must belong to the full wire inventory')
        if set(self.wires) & (set(self.pedicels) | {self.fruit}) or self.fruit in self.pedicels:
            raise ValueError('Contact roles must be disjoint')


def category(contact, scope, phases):
    """Membership is not success. All boundary phases must permit the pair."""
    names = contact['geoms']
    if len(names) != 2 or len(set(names)) != 2:
        raise ValueError('Expected two distinct geom identities')
    names, phases = set(names), set(phases)
    if phases and phases <= FRUIT_PHASES and scope.fruit in names and names & set(scope.wires):
        return 'target_fruit_touch'
    if phases and phases <= PEDICEL_PHASES and names & set(scope.pedicels) and names & set(scope.rear_wires):
        return 'target_pedicel_contact'
    return 'forbidden_contact'


def contact_stats(contacts, scope, phases):
    """Do not cancel forces or discard sub-threshold/inactive contact records.

    Sum of per-contact force magnitudes is a conservative load indicator, NOT
    a world-frame resultant. Moments are retained in raw force6, not added to N.
    """
    out = {k: dict(records=0, active_records=0, force_sum_N=0., max_normal_N=0.)
           for k in ('target_fruit_touch', 'target_pedicel_contact', 'forbidden_contact')}
    for row in contacts:
        f = np.asarray(row['force6'], dtype=float)
        if f.shape != (6,) or not np.isfinite(f).all() or not math.isfinite(row['dist_m']):
            raise ValueError('Nonfinite or malformed contact measurement')
        item = out[category(row, scope, phases)]
        item['records'] += 1
        item['active_records'] += int(row['active'])
        item['force_sum_N'] += float(np.linalg.norm(f[:3]))
        item['max_normal_N'] = max(item['max_normal_N'], float(f[0]))
    return out


def trial_policy(base, scope, phases):
    """Preserve strict non-target checks; reject preexisting broad exceptions."""
    policy = Policy.from_dict(base)
    for item in policy.allowed_contacts:
        if (item['robot_geom'] not in scope.rear_wires or item['environment_geom'] not in scope.pedicels
                or not set(item['phases']) <= PEDICEL_PHASES):
            raise ValueError('Base policy has an exception outside the known pedicel/rear pairs')
    settings = copy.deepcopy(policy.snapshot()['settings'])
    present = set(phases)
    if not present & FRUIT_PHASES:
        raise ValueError('No explicitly authorized fruit-contact phase in trace')
    # Shrinking phase lists avoids permissions for phases absent in this trace.
    settings['allowed_contacts'] = [dict(item, phases=sorted(set(item['phases']) & present))
                                   for item in policy.allowed_contacts if set(item['phases']) & present]
    expected = {(w, t, p) for w in scope.rear_wires for t in scope.pedicels for p in present & PEDICEL_PHASES}
    actual = {(r['robot_geom'], r['environment_geom'], p) for r in settings['allowed_contacts'] for p in r['phases']}
    if actual != expected:
        raise ValueError('Base policy must retain every declared pedicel/rear permission in present seating phases')
    settings['allowed_contacts'] += [dict(robot_geom=w, environment_geom=scope.fruit,
                                         phases=sorted(present & FRUIT_PHASES)) for w in scope.wires]
    return Policy.from_dict(settings)


def phase_window(trace, t0, t1):
    if not 0 <= t0 <= t1 or not math.isfinite(t1):
        raise ValueError('Invalid phase interval')
    lo = max(0, int(math.floor(t0*60 + 1e-8)))
    hi = min(len(trace)-1, int(math.ceil(t1*60 - 1e-8)))
    if not trace or lo > hi:
        raise ValueError('Phase interval outside trace')
    return tuple(dict.fromkeys(r['phase'] for r in trace[lo:hi+1]))


@dataclass(frozen=True)
class TrialLimits:
    max_target_force_N: float
    max_target_displacement_m: float
    max_forbidden_force_N: float = .01  # Existing reporting floor, not zero-contact proof.

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, (bool, np.bool_)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be explicit, positive and finite')
        if self.max_forbidden_force_N > .01:
            raise ValueError('Do not weaken the existing non-target reporting floor')


def stop_reasons(snapshots, displacement_m, penetration_m, physics_valid, limits):
    if (not math.isfinite(displacement_m) or not math.isfinite(penetration_m)
            or displacement_m < 0 or penetration_m < 0):
        raise ValueError('Invalid physical measurement')
    target, forbidden = [], []
    for stats in snapshots:
        values = [stats[k]['force_sum_N'] for k in
                  ('target_fruit_touch', 'target_pedicel_contact', 'forbidden_contact')]
        if not np.isfinite(values).all() or min(values) < 0:
            raise ValueError('Invalid force totals')
        target.append(values[0] + values[1]); forbidden.append(values[2])
    if not target:
        raise ValueError('Missing force snapshots')
    reasons = []
    if max(target) > limits.max_target_force_N: reasons.append('target_force_limit')
    if max(forbidden) >= limits.max_forbidden_force_N: reasons.append('forbidden_force')
    if displacement_m > limits.max_target_displacement_m: reasons.append('target_displacement_limit')
    if penetration_m > PHYSICS_LIMIT_M: reasons.append('physics_penetration')
    if not physics_valid: reasons.append('invalid_physics')
    return reasons


def scope_from_engine(engine, mapping):
    """Match authored arc geometry, never g-number prefixes or all Hook geoms."""
    import mujoco as mj
    from hook_retention_diagnostic import capsule_endpoints
    from geometry import RING_RADIUS, WIRE_RADIUS
    p, rotation = mapping.frame(engine.data)
    m, wires = engine.model, []
    for gid in engine.hookgeoms:
        if int(m.geom_bodyid[gid]) != engine.hook or int(m.geom_type[gid]) != int(mj.mjtGeom.mjGEOM_CAPSULE):
            continue
        if not (int(m.geom_contype[gid]) or int(m.geom_conaffinity[gid])):
            continue
        a, b, radius = capsule_endpoints(m, engine.data, gid)
        points = rotation.inv().apply(np.array([a, b])-p)
        if (abs(radius-WIRE_RADIUS) < 1e-6 and np.max(abs(points[:, 1])) < 1e-5
                and np.max(abs(np.linalg.norm(points[:, [0, 2]], axis=1)-RING_RADIUS)) < 1e-5
                and np.max(points[:, 0]) <= 1e-5):
            wires.append(m.geom(int(gid)).name)
    fruits = [m.geom(g).name for g in range(m.ngeom) if int(m.geom_bodyid[g]) == engine.fruit
              and (int(m.geom_contype[g]) or int(m.geom_conaffinity[g]))]
    if len(wires) != 32 or len(fruits) != 1 or fruits[0] != 'glb_col_'+engine.target:
        raise ValueError('Expected authored 32-wire arc and one exact target fruit collider; review mapping')
    return ContactScope(fruits[0], tuple(sorted(wires)), tuple(mapping.target_names), tuple(sorted(mapping.rear_names)))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def execution_permitted(audit, requested):
    return bool(requested and audit.get('passed') is True and audit.get('complete') is True
                and audit.get('status') == 'sampled_clear')


class TrialStopped(RuntimeError):
    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--base-policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true', help='Offline simulation only, after a complete conditional audit')
    parser.add_argument('--max-target-force-n', type=float)
    parser.add_argument('--max-target-displacement-m', type=float)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.candidate): parser.error('Invalid candidate identity')
    root, out = args.run.resolve(), args.output.resolve()
    if out.exists() or out == root or root in out.parents: parser.error('Use a NEW output outside the source run')
    if args.execute and (args.max_target_force_n is None or args.max_target_displacement_m is None):
        parser.error('--execute requires explicit force/displacement stop bounds (not calibrated safety limits)')
    limits = TrialLimits(args.max_target_force_n, args.max_target_displacement_m) if args.execute else None
    from diagnose_contact_timing import load_engine, force_rows
    from measure_seating_rollout import SeatingProbe
    from seating_evaluation import timeline_evidence
    engine, trace, plan = load_engine(root, args.candidate)
    if plan.get('parameters', {}).get('trajectory_mode') != 'diagnostic_pose_waypoints_v1':
        raise ValueError('First use only the saved diagnostic pose-waypoint trials')
    probe = SeatingProbe(engine, trace)
    scope = scope_from_engine(engine, probe.mapping)
    base = json.loads(args.base_policy.read_text())
    policy = trial_policy(base, scope, [r['phase'] for r in trace])
    inputs = [args.base_policy, root/'manifest.json', root/'replay_assets/model.mjb',
              root/'replay_assets/reference.json', root/'replay_assets/initial_trace.json',
              root/'candidates'/args.candidate/'plan.json', root/'candidates'/args.candidate/'trace.json']
    hashes = {str(p): sha(p) for p in inputs}
    out.mkdir(parents=True, exist_ok=False)
    write_json(out/'contact_scope.json', dict(scope=asdict(scope), original_policy=base,
               trial_policy=policy.snapshot(), user_requirement='Allow light target fruit/wire contact; not mounting hardware or non-targets',
               diagnostic_only=True, training_eligible=False, source_sha256=hashes, code_sha256=sha(__file__)))
    audit = check_engine(engine, trace, plan['seconds'], policy, collect_all_violations=True)
    save_report(out, audit)
    result = dict(physics_executed=False, completed=False, training_eligible=False, hook_success=None,
                  audit_status=audit['status'], audit_complete=audit['complete'],
                  limits=None if limits is None else asdict(limits),
                  scope='Offline target-contact test; no calibrated skin-damage or real-safety claim')
    new_rows, times, qposes = [], [0.], [engine.data.qpos.copy()]
    try:
        if execution_permitted(audit, args.execute):
            m = engine.model
            initial = engine.data.xpos[engine.fruit].copy()
            count = 0
            def measure(model, data):
                nonlocal count
                count += 1
                result['physics_executed'] = True
                if count % max(1, round(1/model.opt.timestep/30)) == 0:
                    times.append(float(data.time)); qposes.append(data.qpos.copy())
                if (not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all()
                        or np.max(abs(data.qvel), initial=0) > 1e4 or np.any(data.warning.number)):
                    result['stop_reasons'] = ['invalid_physics']; raise TrialStopped()
                probe(model, data)
                phases = phase_window(trace, max(0., data.time-model.opt.timestep), data.time)
                live = contact_stats(force_rows(model, data, robot_ids=probe.mapping.robot_geoms), scope, phases)
                fresh = contact_stats(force_rows(model, probe.private, robot_ids=probe.mapping.robot_geoms), scope, phases)
                row = dict(probe.rows[-1])
                row.update(legacy_non_target_force_N=row['non_target_force_N'],
                           non_target_force_N=fresh['forbidden_contact']['force_sum_N'],
                           contact_categories_live=live, contact_categories_private=fresh,
                           active_permission_phases=list(phases),
                           target_displacement_m=float(np.linalg.norm(data.xpos[engine.fruit]-initial)))
                # A fruit touch must never count as the target pedicel contact.
                row['target_contact'] = bool(row['target_contact'] and set(phases) <= PEDICEL_PHASES)
                live_pen = max([0.] + [-float(c.dist) for c in data.contact])
                row['guard_penetration_m'] = max(row['max_penetration_m'], live_pen)
                row['stop_reasons'] = stop_reasons([live, fresh], row['target_displacement_m'],
                                                  row['guard_penetration_m'], row['physics_valid'], limits)
                new_rows.append(row)
                if row['stop_reasons']:
                    result['stop_reasons'] = row['stop_reasons']; raise TrialStopped()
            try:
                physics = engine.rollout(seconds=plan['seconds'], record=False, on_step=measure)
                result.update(completed=not physics['unstable'], physics_metrics=physics)
            except TrialStopped:
                result['stopped_before_next_sim_step'] = True
            finally:
                if engine.data.time > times[-1]:
                    times.append(float(engine.data.time)); qposes.append(engine.data.qpos.copy())
                np.savez_compressed(out/'trial_states.npz', times_s=times, qpos=qposes)
                if probe.rows: result['legacy_evidence'] = probe.save(out)
                if new_rows:
                    evidence = timeline_evidence(new_rows, initially_seated=probe.initial, dt=m.opt.timestep)
                    evidence['contact_retention_evidence'] &= result['completed'] and not result.get('stop_reasons')
                    result['authorized_contact_evidence'] = evidence
                (out/'authorized_contact_samples.json.gz').write_bytes(gzip.compress(json.dumps(new_rows, allow_nan=False).encode(), mtime=0))
                result['simulated_s'] = float(engine.data.time)
        else:
            result['reason'] = 'audit_only' if not args.execute else 'conditional_audit_not_complete_and_clear'
    except Exception as error:
        result.update(completed=False, error=f'{type(error).__name__}: {error}')
        raise
    finally:
        result['source_unchanged'] = all(sha(p) == digest for p, digest in hashes.items())
        if not result['source_unchanged']:
            result.update(completed=False, error='Source files changed during diagnostic')
        write_json(out/'target_contact_trial.json', result)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if audit['passed'] and audit['complete'] and (not args.execute or result['completed']) else 2


if __name__ == '__main__':
    raise SystemExit(main())
