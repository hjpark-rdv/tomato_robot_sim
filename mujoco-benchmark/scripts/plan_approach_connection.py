"""d1fdc7c-aware approach recovery and matched local-orientation comparisons.

Recheck/reuse an already-clear recorded prefix. Only a blocked prefix enters
multi-start IK + OMPL. Local variants keep the approach fixed. Nothing here
executes physics or changes the server collision/contact/material policies.
"""
from __future__ import annotations
import argparse
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import time
import numpy as np
from scipy.spatial.transform import Rotation
from approach_connection import (CheckBudget, JointSpace, LimitReached, check_polyline,
                                 handoff_index, ik_pool, ompl_connect, retime,
                                 stitch, suffix_parameters)
from recovery_design import (BASE_COMMIT, DESIGNS, recorded_prefix,
                             matched_local_variant, join_recorded_prefix)


class SceneValidity:
    """Existing native environmental shapes and exported FCL self-collision pairs."""
    def __init__(self, scene, checker, policy, space, budget, phases=('preapproach','approach')):
        self.scene, self.checker, self.policy, self.space, self.budget = scene, checker, policy, space, budget
        self.phases = frozenset(phases)
        self.cache, self.rejections = {}, {}
        self.counts, self.hits, self.queries = Counter(), [], 0
        self.allowed = {}
        for row in policy.allowed_contacts:
            if row['robot_geom'] not in scene.robot_names or row['environment_geom'] not in scene.environment_names:
                raise ValueError('Unknown contact-policy geom')
            self.allowed.setdefault((row['robot_geom'], row['environment_geom']), set()).update(row['phases'])
        if not scene.robot_names or not scene.environment_names:
            raise ValueError('Empty collision inventory')

    def __call__(self, q):
        from environment_preflight import nearby_indices
        self.budget.tick()
        q = np.asarray(q, dtype=float)
        if q.shape != (self.space.n,) or not np.isfinite(q).all():
            raise ValueError('Invalid robot configuration')
        key = q.tobytes()
        if key in self.cache:
            self.counts['cache_hit'] += 1
            return self.cache[key]
        if not self.space.contains(q):
            return self._store(key, False, 'joint_bounds')
        pair = self.checker.check(q)
        if pair is not None:
            return self._store(key, False, 'self_collision', pair=pair)
        self.scene.set_robot(q)
        limit = self.policy.clearance_m + .01
        for i, a in enumerate(self.scene.robot_names):
            indices = nearby_indices(self.scene.robot_positions[i], self.scene.robot_radii[i],
                                     self.scene.environment_positions, self.scene.environment_radii, limit)
            for j in indices:
                b = self.scene.environment_names[int(j)]
                if self.phases and self.phases <= self.allowed.get((a, b), set()):
                    continue
                if self.queries >= self.policy.max_distance_queries:
                    raise LimitReached('distance_query_budget')
                if self.budget.remaining() <= 0:
                    raise LimitReached('wall_time_budget')
                distance, segment = self.scene.distance(i, int(j), limit)
                self.queries += 1
                if not math.isfinite(distance) or not np.isfinite(segment).all():
                    raise ValueError('Invalid distance query')
                if distance <= self.policy.clearance_m:
                    return self._store(key, False, 'environment_collision', pair=[a, b], distance_m=float(distance))
        return self._store(key, True, 'valid')

    def _store(self, key, answer, kind, **details):
        self.cache[key] = answer
        self.counts[kind] += 1
        if not answer:
            self.rejections[key] = dict(kind=kind, **details)
            if len(self.hits) < 24:
                self.hits.append(self.rejections[key])
        return answer

    def summary(self):
        return dict(checks=self.budget.checks, distance_queries=self.queries, counts=dict(self.counts),
                    first_rejections=self.hits, phases=sorted(self.phases),
                    inventory=self.scene.inventory, policy=self.policy.snapshot())


def jsonable(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [jsonable(v) for v in value]
    return value


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(jsonable(value), indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def file_hash(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            value.update(block)
    return value.hexdigest()


def plan_suffix(env, kin, checker, params, q, plan_fn):
    """Keep the reached IK branch; do not mutate the planning environment."""
    import torch
    local = copy.deepcopy(env)
    local.robot.data.joint_pos = torch.as_tensor(np.asarray(q)[None].copy(), dtype=env.robot.data.joint_pos.dtype)
    planned, check = plan_fn(local, kin, checker, suffix_parameters(params, q, kin))
    if planned is not None:
        planned = copy.deepcopy(planned)
        planned['phases'] = ['approach' if p == 'preapproach' else p for p in planned['phases']]
        for pose in planned.get('waypoints', []):
            if pose['phase'] == 'preapproach': pose['phase'] = 'approach'
    return planned, check


def policy_for_trace(policy, rows):
    """Remove absent phases only. Never add a contact exception for a prefix."""
    from environment_preflight import Policy
    phases = {row['phase'] for row in rows}
    settings = copy.deepcopy(policy.snapshot()['settings'])
    settings['allowed_contacts'] = [dict(row, phases=sorted(set(row['phases']) & phases))
                                   for row in settings['allowed_contacts'] if set(row['phases']) & phases]
    return Policy.from_dict(settings)


def checked_original_prefix(root, cid, start, kin, handoff, space, selfchecker, engine, policy):
    """Old report labels are not permission to reuse: independently recheck the trace."""
    from environment_preflight import check_engine
    path = root/'candidates'/cid/'trace.json'
    if not path.is_file():
        return None, dict(available=False, passed=False, reason='no_recorded_trace')
    rows, evidence = recorded_prefix(json.loads(path.read_text()), start)
    q = np.asarray([row['command'] for row in rows], dtype=float)
    p, r = kin.fk(q[-1])
    pe = float(np.linalg.norm(p-np.asarray(handoff['ring_position_xyz'])))
    re = float((Rotation.from_quat(handoff['orientation_xyzw'])*r.inv()).magnitude())
    evidence.update(available=True, passed=False, handoff_position_error_m=pe, handoff_rotation_error_rad=re)
    if pe > .002 or re > .03:
        evidence['reason'] = 'recorded_handoff_mismatch'
        return None, evidence
    check = check_polyline(q, space, lambda x: selfchecker.check(x) is None)
    evidence['self_check'] = check
    if not check['passed']:
        evidence['reason'] = 'recorded_prefix_self_collision'
        return None, evidence
    audit = check_engine(engine, rows, (len(rows)-1)/60., policy_for_trace(policy, rows))
    evidence['environment_check'] = audit
    evidence['passed'] = bool(audit.get('passed') and audit.get('complete'))
    evidence['reason'] = 'rechecked_clear' if evidence['passed'] else audit.get('status', 'inconclusive')
    return rows if evidence['passed'] else None, evidence


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--base-policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ik-seeds', type=int, default=8)
    parser.add_argument('--max-branches', type=int, default=3)
    parser.add_argument('--solve-seconds', type=float, default=8.)
    parser.add_argument('--total-seconds', type=float, default=120.)
    parser.add_argument('--max-state-checks', type=int, default=100000)
    parser.add_argument('--max-prefix-seconds', type=float, default=180.)
    parser.add_argument('--seed', type=int, default=20260928)
    parser.add_argument('--local-design', choices=DESIGNS, default='recorded')
    parser.add_argument('--require-recorded-prefix', action='store_true', help='Matched local control: no replacement global route')
    parser.add_argument('--model-cache', type=Path, help='Reuse the server hash-verified MJB cache; restore durable links')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.candidate): parser.error('Invalid candidate id')
    root, out = args.run.resolve(), args.output.resolve()
    if out.exists() or root == out or root in out.parents: parser.error('Use a new output outside the source run')
    if not 1 <= args.max_branches <= 8 or not 1 <= args.ik_seeds <= 32 or args.seed < 0: parser.error('Invalid branch/seed budget')
    if args.max_state_checks < 1 or not all(math.isfinite(x) and x > 0 for x in (args.solve_seconds, args.total_seconds, args.max_prefix_seconds)):
        parser.error('Positive finite budgets required')
    from run_motion_family_search import REQUIRED, prepare_snapshot, native_geometry
    import plan_candidates as planner
    from dataset_motion import plan as legacy_plan, diagnostic_poses
    from robot_engine import RobotEngine
    from hook_retention_diagnostic import HookProbe
    from target_fruit_contact_trial import scope_from_engine, trial_policy
    from environment_preflight import MuJoCoScene, check_engine, save_report
    import search_model_cache

    original = next((r for r in json.loads((root/'candidates.json').read_text()) if r['candidate_id'] == args.candidate), None)
    if original is None: raise ValueError('Candidate not found')
    diagnostic_poses(original)
    inputs = [root/p for p in REQUIRED] + [root/'candidates.json', args.base_policy.resolve()]
    trace_file = root/'candidates'/args.candidate/'trace.json'
    if trace_file.is_file(): inputs.append(trace_file)
    hashes = {str(p): file_hash(p) for p in inputs}
    out.mkdir(parents=True)
    run = out/'run'
    manifest = prepare_snapshot(dict(source_run=str(root)), run)
    started = time.monotonic()
    report = dict(schema='approach_connection_v2', evidence_base=BASE_COMMIT,
                  source_run=str(root), source_candidate=args.candidate, source_sha256=hashes,
                  design=args.local_design, prefix_found=False, prefix_reused=False,
                  attempts=[], approach_only_runs=[], whole_path_passed=False, physics_executed=False,
                  training_eligible=False, hook_success=None, impossible=None, status='setup_incomplete')
    report['code_sha256'] = {n: file_hash(Path(__file__).with_name(n)) for n in
                            ('approach_connection.py', 'plan_approach_connection.py', 'recovery_design.py',
                             'motion_family_search.py', 'environment_preflight.py', 'search_model_cache.py')}
    valid = None
    def deadline_handler(signum, frame): raise LimitReached('overall_planning_deadline')
    old_handler = signal.signal(signal.SIGALRM, deadline_handler)
    signal.setitimer(signal.ITIMER_REAL, args.total_seconds)
    try:
        if args.model_cache:
            search_model_cache.attach(run, root/'replay_assets/model.mjb', manifest['model_sha256'], args.model_cache)
        geometry = native_geometry(run)[0] if args.local_design != 'recorded' else None
        params = matched_local_variant(original, geometry, args.local_design)
        diagnostic_poses(params)
        write(run/'candidates.json', [params])
        boundary = handoff_index(params)
        handoff = params['pose_waypoints'][boundary]
        report.update(handoff_index=boundary, handoff=handoff,
                      source_parameters=original.get('search_parameters'), parameters=params.get('search_parameters'))
        planner.initialize(str(run))
        env, kin, selfchecker = planner.MODEL
        start = np.asarray(planner.START, dtype=float)
        if abs(env.step_dt-1/60) > 1e-12: raise ValueError('Expected existing 60 Hz contract')
        engine = RobotEngine(run/'replay_assets/model.mjb', run/'replay_assets/initial_trace.json', manifest['hz'],
                             reference=run/'replay_assets/reference.json', target=manifest['target'])
        if manifest.get('model_sha256') != file_hash(run/'replay_assets/model.mjb'): raise ValueError('Model hash mismatch')
        if not np.allclose(start, engine.initial, atol=1e-12, rtol=0): raise ValueError('Initial joint mismatch')
        mapping = HookProbe(engine, 0.)
        scope = scope_from_engine(engine, mapping, search_policy=True)
        phases = ['ready', 'preapproach', 'approach'] + [r['phase'] for r in params['pose_waypoints'][boundary+1:]]
        base = json.loads(args.base_policy.read_text())
        policy = trial_policy(base, scope, phases)
        scene = MuJoCoScene(engine, policy)
        space = JointSpace(kin.bounds, scene.joint_steps)
        budget = CheckBudget(args.max_state_checks, args.total_seconds)
        valid = SceneValidity(scene, selfchecker, policy, space, budget)
        velocity = np.full(space.n, .35); velocity[env.lift_id] = .08
        max_velocity = np.ones(space.n); max_velocity[env.lift_id] = .25
        saved, original_check = checked_original_prefix(root, args.candidate, start, kin, handoff,
                                                        space, selfchecker, engine, policy)
        report['original_prefix'] = original_check
        write(out/'original_prefix_check.json', original_check)
        if saved is not None:
            qgoal = np.asarray(saved[-1]['command'], dtype=float)
            goals, ik_records = [qgoal], []
            report['prefix_reused'] = True
        elif args.require_recorded_prefix:
            report['status'] = 'matched_prefix_control_failed'
            goals, ik_records = [], []
        elif not valid(start):
            report['status'] = 'invalid_initial_configuration'
            goals, ik_records = [], []
        else:
            def aligned_valid(q):
                if not valid(q): return False
                scene.set_robot(q)
                p, r = mapping.frame(scene.data); kp, kr = kin.fk(q)
                if np.linalg.norm(p-kp) > 1e-5 or (r*kr.inv()).magnitude() > 1e-5:
                    raise ValueError('Native/IK FK disagreement')
                return True
            goals, ik_records = ik_pool(kin, handoff['ring_position_xyz'], Rotation.from_quat(handoff['orientation_xyzw']),
                                        start, space, aligned_valid, count=args.ik_seeds, seed=args.seed)
            for row in ik_records:
                if row['status'] == 'goal_collision':
                    row['collision'] = valid.rejections.get(np.asarray(row['q'], dtype=float).tobytes())
            report['status'] = 'no_valid_ik_goal_found' if not goals else 'no_complete_connection_found'
            from ompl import util as ou
            ou.RNG.setSeed(args.seed)
        report.update(ik_records=ik_records, valid_ik_branches=len(goals))
        for index, qgoal in enumerate(goals[:args.max_branches]):
            item = dict(branch_index=index, qgoal=qgoal.tolist())
            report['attempts'].append(item)
            if saved is not None:
                prefix = np.asarray([row['command'] for row in saved], dtype=float)
                prefix_rows = saved
                item['method'] = 'rechecked_recorded_prefix'
                joint_path = None
            else:
                if budget.remaining() <= 0: raise LimitReached('wall_time_budget')
                route = ompl_connect(start, qgoal, space, valid, solve_seconds=min(args.solve_seconds, budget.remaining()))
                item.update({k: jsonable(v) for k, v in route.items() if k != 'path'})
                if route['path'] is None: continue
                joint_path = route['path']
                prefix = retime(joint_path, velocity, max_seconds=args.max_prefix_seconds)
                prefix_rows = [dict(joints=start.tolist(), command=q.tolist(), phase='ready' if i == 0 else 'preapproach')
                               for i, q in enumerate(prefix)]
            report['prefix_found'] = True
            branch = out/f'branch_{index:02d}'; branch.mkdir()
            item['prefix_seconds'] = (len(prefix)-1)/60.
            write(branch/'prefix_joint_path.json', dict(path=joint_path, commands=prefix, phases=[r['phase'] for r in prefix_rows],
                  command_hz=60, method=item['method'], physics_executed=False, policy=policy.snapshot()))
            # Recheck the exact resampled command stream and export an arrival-only control.
            arrival = join_recorded_prefix(prefix_rows, np.repeat(qgoal[None], 60, axis=0), ['hold']*60, start, max_velocity)
            self_result = check_polyline(np.asarray([r['command'] for r in arrival]), space, lambda q: selfchecker.check(q) is None)
            item['arrival_self_check'] = self_result
            if not self_result['passed']:
                item['status'] = 'retimed_prefix_self_check_failed'; continue
            arrival_audit = check_engine(engine, arrival, (len(arrival)-1)/60., policy_for_trace(policy, arrival))
            write(branch/'approach_only_audit.json', arrival_audit)
            item['arrival_audit'] = arrival_audit['status']
            if not (arrival_audit.get('passed') and arrival_audit.get('complete')):
                item['status'] = 'retimed_prefix_environment_rejected'; continue
            arun = branch/'approach_only_run'
            prepare_snapshot(dict(source_run=str(root)), arun)
            ap = copy.deepcopy(params); ap['candidate_id'] = args.candidate+'_approach'
            ap.update(approach_only=True, manipulation_attempt=False)
            ap['pose_waypoints'] = suffix_parameters(params, qgoal, kin)['pose_waypoints'][:1]
            ap['pose_waypoints'].append(dict(ap['pose_waypoints'][0], phase='hold', minimum_seconds=1.))
            write(arun/'candidates.json', [ap])
            adest = arun/'candidates'/ap['candidate_id']
            write(adest/'trace.json', arrival)
            write(adest/'plan.json', dict(candidate_id=ap['candidate_id'], parameters=ap, seconds=(len(arrival)-1)/60.,
                  preflight=dict(passed=True), waypoints=[], approach_only=True, manipulation_attempt=False,
                  diagnostic_only=True, training_eligible=False, hook_success=None))
            report['approach_only_runs'].append(dict(run=str(arun), candidate=ap['candidate_id'], branch_index=index,
                                                      manipulation_attempt=False, prefix_reused=saved is not None))
            planned, check = plan_suffix(env, kin, selfchecker, params, qgoal, legacy_plan)
            item['suffix_planner'] = check
            if planned is None:
                item['status'] = 'prefix_clear_suffix_planning_rejected'
                report['status'] = item['status']; continue
            rows = join_recorded_prefix(prefix_rows, planned['commands'], planned['phases'], start, max_velocity)
            self_result = check_polyline(np.asarray([r['command'] for r in rows]), space, lambda q: selfchecker.check(q) is None)
            item['whole_self_check'] = self_result
            if not self_result['passed']:
                item['status'] = 'whole_self_check_failed'; report['status'] = item['status']; continue
            audit = check_engine(engine, rows, (len(rows)-1)/60., policy_for_trace(policy, rows))
            save_report(branch, audit)
            item['whole_environment_check'] = audit['status']
            if not (audit.get('passed') and audit.get('complete')):
                item.update(status='prefix_clear_suffix_environment_rejected', first_violation=audit.get('first_violation'))
                report['status'] = item['status']; continue
            dest = run/'candidates'/args.candidate
            write(dest/'trace.json', rows)
            write(dest/'plan.json', dict(candidate_id=args.candidate, parameters=params, seconds=(len(rows)-1)/60.,
                  preflight=dict(passed=True, whole_environment_passed=True), waypoints=planned['waypoints'],
                  approach_connection=dict(handoff_index=boundary, branch_index=index, prefix_reused=saved is not None,
                                           prefix_seconds=item['prefix_seconds'], method=item['method']),
                  diagnostic_only=True, training_eligible=False, hook_success=None))
            item['status'] = 'whole_path_passed'
            report.update(status='whole_path_passed', whole_path_passed=True, selected_branch=index,
                          runnable_run=str(run), seconds=(len(rows)-1)/60.)
            break
    except LimitReached as error:
        report.update(status='budget_exhausted', stop_reason=str(error))
    except Exception as error:
        report.update(status='execution_error', error=f'{type(error).__name__}: {error}')
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old_handler)
        try:
            search_model_cache.restore(run)
        except Exception as error:
            report.update(status='cache_restore_error', error=str(error), whole_path_passed=False)
        if valid is not None: report['validity'] = valid.summary()
        report['wall_s'] = time.monotonic()-started
        report['source_unchanged'] = all(file_hash(p) == h for p, h in hashes.items())
        if not report['source_unchanged']:
            report.update(status='source_changed', whole_path_passed=False, runnable_run=None, approach_only_runs=[])
        write(out/'connection_result.json', report)
        print(json.dumps({k: report[k] for k in ('status', 'prefix_found', 'prefix_reused', 'whole_path_passed', 'physics_executed', 'wall_s')}, indent=2))
    return 0 if report['whole_path_passed'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
