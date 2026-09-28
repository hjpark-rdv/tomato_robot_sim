"""Opt-in access-first planning of ONE existing explicit-pose candidate.

Run in a Python environment with the existing Torch/FCL planner, MuJoCo and
ompl==2.0.1. No simulation steps and no source-run writes occur here. Output
contains independent prefix/full run snapshots consumable by the old trial
and renderer. A prefix solution is not a complete harvesting trajectory.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys
import time
import numpy as np

from access_route import (Budget, SceneValidity, SearchBudget, checked_path,
                          ik_options, ompl_connect, timed_polyline)


def write(path, obj):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    tmp.replace(path)


def access_boundary(params):
    from dataset_motion import diagnostic_poses
    poses = diagnostic_poses(params)
    index = next((i for i, p in enumerate(poses) if p[0] not in ('preapproach','approach')), None)
    if index is None or index < 1:
        raise ValueError('Require an approach prefix followed by an insertion/task suffix')
    return index, poses[index-1]


def suffix_from_branch(env, kin, self_checker, params, q, boundary):
    """Replan the task from the SELECTED IK branch; never paste old joint traces."""
    import torch
    from dataset_motion import plan
    e = copy.copy(env); e.robot = copy.copy(env.robot); e.robot.data = copy.copy(env.robot.data)
    e.robot.data.joint_pos = torch.from_numpy(np.asarray(q, dtype=float)[None].copy())
    p, r = kin.fk(q)
    updated = copy.deepcopy(params)
    updated['pose_waypoints'] = [dict(phase='preapproach', ring_position_xyz=p.tolist(),
          orientation_xyzw=r.as_quat().tolist(), minimum_seconds=0.)] + copy.deepcopy(params['pose_waypoints'][boundary:])
    planned, check = plan(e, kin, self_checker, updated)
    if planned is not None:
        # This initial stationary segment is at the access gate, not a fresh
        # home->preapproach move. Original task phase names are unchanged.
        planned['phases'] = ['approach' if x == 'preapproach' else x for x in planned['phases']]
    return planned, check, updated


def trace_rows(start, commands, phases):
    if len(commands) != len(phases): raise ValueError('Command/phase mismatch')
    return [dict(joints=start.tolist(), command=start.tolist(), phase='ready')] + [
        dict(joints=start.tolist(), command=q.tolist(), phase=p) for q, p in zip(commands, phases)]


def publish_run(source, out, params, start, commands, phases, check, audit, metadata):
    from run_motion_family_search import prepare_snapshot
    from environment_preflight import save_report
    out = Path(out)
    manifest = prepare_snapshot(dict(source_run=str(source)), out)
    cid = params['candidate_id']; folder = out/'candidates'/cid; folder.mkdir(parents=True)
    rows = trace_rows(start, commands, phases)
    write(folder/'trace.json', rows)
    write(out/'candidates.json', [params])
    # FK/nominal poses are not execution observations.
    info = dict(candidate_id=cid, parameters=params, seconds=(len(rows)-1)/60.,
        preflight=check, environment_preflight=audit, access_planning=metadata,
        diagnostic_only=True, training_eligible=False, hook_success=None,
        action_schema='joint_trace_with_explicit_task_waypoints_v1')
    write(folder/'plan.json', info)
    save_report(folder, audit)
    manifest.update(count=1, plan_only=True, diagnostic_only=True, training_eligible=False,
                    access_stage=metadata['stage'], hook_success=None,
                    trace_is_authoritative=True)
    write(out/'manifest.json', manifest)
    return dict(run=str(out), candidate=cid, seconds=info['seconds'])


def solve(source, output, candidate_id, base_policy, *, ik_seeds=8, max_branches=3,
          route_seconds=8., total_search_seconds=60., max_checks=30000, seed=28,
          access_only=False):
    from run_motion_family_search import REQUIRED, file_hash, prepare_snapshot
    from motion_family_search import identity
    identity(candidate_id)
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists() or output == source or source in output.parents:
        raise ValueError('Choose a NEW output directory outside the source run')
    if not 1 <= max_branches <= ik_seeds <= 64:
        raise ValueError('Require 1 <= max_branches <= ik_seeds <= 64')
    if not np.isfinite(route_seconds) or not 0 < route_seconds <= total_search_seconds:
        raise ValueError('Per-route budget must fit total search budget')
    raw = json.loads((source/'candidates.json').read_text())
    matches = [p for p in raw if p['candidate_id'] == candidate_id]
    if len(matches) != 1: raise ValueError('Candidate identity missing or duplicated')
    params = copy.deepcopy(matches[0])
    if params.get('trajectory_mode') != 'diagnostic_pose_waypoints_v1':
        raise ValueError('Only existing explicit-pose candidates are accepted')
    hashed = {n:file_hash(source/n) for n in (*REQUIRED,'candidates.json')}
    source_manifest = json.loads((source/'manifest.json').read_text())
    if source_manifest.get('model_sha256') != hashed['replay_assets/model.mjb']:
        raise ValueError('Source model hash mismatch')
    base_bytes = Path(base_policy).read_bytes()
    import hashlib
    output.mkdir(parents=True)
    result = dict(schema='access_then_task_v1', source_run=str(source), candidate_id=candidate_id,
        source_sha256=hashed, base_policy_sha256=hashlib.sha256(base_bytes).hexdigest(),
        physics_executed=False, training_eligible=False, hook_success=None, impossible=None,
        prefix_found=False, full_path_found=False, branches=[],
        limits=dict(ik_seeds=ik_seeds,max_branches=max_branches,route_seconds=route_seconds,
                    total_search_seconds=total_search_seconds,max_checks=max_checks,seed=seed),
        scope='Frozen scene + sampled joint-path validation, not a dynamics/hardware success')
    began = time.monotonic()
    try:
        # SAME exported FK/FCL and SAME native geometry, no duplicate scene.
        work = output/'solver_snapshot'
        prepare_snapshot(dict(source_run=str(source)), work)
        import plan_candidates as pc
        pc.initialize(work)
        env, kin, self_checker = pc.MODEL
        if abs(env.step_dt-1/60) > 1e-12:
            raise ValueError("Existing trace contract requires 60 Hz commands")
        boundary, gate = access_boundary(params)
        from robot_engine import RobotEngine
        from hook_retention_diagnostic import HookProbe
        from target_fruit_contact_trial import scope_from_engine, trial_policy
        from environment_preflight import MuJoCoScene, check_engine
        engine = RobotEngine(work/'replay_assets/model.mjb', work/'replay_assets/initial_trace.json',
            source_manifest['hz'], reference=work/'replay_assets/reference.json', target=source_manifest['target'])
        start = np.asarray(pc.START, dtype=float)
        if not np.allclose(engine.initial, start, rtol=0, atol=1e-12):
            raise ValueError('Native and planner initial joints differ')
        probe = HookProbe(engine,0.)
        scope = scope_from_engine(engine,probe,search_policy=True)
        base = json.loads(base_bytes)
        phases = list(dict.fromkeys(['ready','preapproach','approach','hold'] + [p['phase'] for p in params['pose_waypoints']]))
        policy = trial_policy(base,scope,phases)
        write(output/'base_policy.json',base)
        write(output/'resolved_policy.json',policy.snapshot())
        scene = MuJoCoScene(engine,policy)
        result['collision_inventory'] = scene.inventory
        result['boundary'] = dict(first_task_index=boundary, gate_phase=gate[0],
            ring_position_xyz=gate[1].tolist(),orientation_xyzw=gate[2].as_quat().tolist(),
            discarded_source_prefix=params['pose_waypoints'][:boundary])
        budget = Budget(total_search_seconds,max_checks)
        valid = SceneValidity(scene,self_checker,policy,kin.bounds,budget)
        if not valid(start):
            result.update(status='initial_state_invalid', initial_blocker=valid.last_blocker)
            return result
        options, ik_records = ik_options(kin,gate[1],gate[2],start,valid,scene.joint_steps,
            seeds=ik_seeds,max_solutions=max_branches,seed=seed,budget=budget)
        result['ik_attempts'] = ik_records
        if not options:
            result['status'] = 'no_collision_free_ik_in_declared_seeds'
            return result
        velocity = np.full(len(start), .35); velocity[env.lift_id] = .08
        for index, qgoal in enumerate(options):
            if budget.expired(): raise SearchBudget('shared_access_budget')
            branch = dict(index=index,goal_joints=qgoal.tolist()); result['branches'].append(branch)
            polyline, meta = ompl_connect(start,qgoal,kin.bounds,scene.joint_steps,valid,
                seconds=min(route_seconds,budget.remaining()),seed=seed+index)
            branch['route'] = meta
            if polyline is None: continue
            timed = timed_polyline(polyline,velocity,env.step_dt)
            if not checked_path(timed,scene.joint_steps,valid):
                raise RuntimeError('Timed access path failed independent validation')
            prefix = timed[1:]
            # Publish a real home->gate + hold COMMAND trace even when the task
            # fails later. It must be physically rolled out from home to count.
            hold = np.repeat(prefix[-1:],int(round(1.1/env.step_dt)),axis=0)
            pcmd = np.concatenate([prefix,hold])
            pphase = ['preapproach']*len(prefix)+['hold']*len(hold)
            prows = trace_rows(start,pcmd,pphase)
            pp = trial_policy(base,scope,[r['phase'] for r in prows])
            paudit = check_engine(engine,prows,(len(prows)-1)/60,pp)
            branch['prefix_audit'] = {k:paudit.get(k) for k in ('status','passed','complete','first_violation','reason')}
            if not (paudit.get('passed') and paudit.get('complete')): continue
            result['prefix_found'] = True
            gatep,gater = kin.fk(qgoal)
            prefix_params = dict(params,candidate_id='access_'+candidate_id,experiment_stage='access_only')
            prefix_params['pose_waypoints'] = [dict(phase=p,ring_position_xyz=gatep.tolist(),
                orientation_xyzw=gater.as_quat().tolist(),minimum_seconds=1.1 if p=='hold' else 0.)
                for p in ('preapproach','hold')]
            if 'prefix_run' not in result:
                result['prefix_run'] = publish_run(source,output/'prefix',prefix_params,start,pcmd,pphase,
                    dict(passed=True,planner='OMPL access + original FCL'),paudit,
                    dict(stage='access_only',branch=index,polyline=polyline.tolist(),
                         gate_fk_error_m=float(np.linalg.norm(gatep-gate[1])),seed=seed+index))
            if access_only:
                result['status']='access_path_found_task_not_requested'; break
            if budget.expired(): raise SearchBudget('budget_after_prefix_validation')
            suffix, check, suffix_params = suffix_from_branch(env,kin,self_checker,params,qgoal,boundary)
            branch['task_check'] = check
            if suffix is None: continue
            commands = np.concatenate([prefix,suffix['commands']])
            full_phases = ['preapproach']*len(prefix)+suffix['phases']
            rows = trace_rows(start,commands,full_phases)
            fp = trial_policy(base,scope,[r['phase'] for r in rows])
            audit = check_engine(engine,rows,(len(rows)-1)/60,fp)
            branch['full_audit'] = {k:audit.get(k) for k in ('status','passed','complete','first_violation','reason')}
            if not (audit.get('passed') and audit.get('complete')): continue
            result['full_run'] = publish_run(source,output/'full',params,start,commands,full_phases,
                dict(passed=True,planner='OMPL access + existing Cartesian task planner',task=check),audit,
                dict(stage='access_and_task',branch=index,polyline=polyline.tolist(),
                     replanned_task_waypoints=suffix_params['pose_waypoints'],seed=seed+index))
            result.update(full_path_found=True,status='full_nominal_path_found'); break
        result.setdefault('status','access_found_task_unresolved' if result['prefix_found'] else 'access_not_found_within_budget')
        result['state_checks'] = budget.checks
    except SearchBudget as exc:
        result.update(status='access_found_task_unresolved' if result['prefix_found'] else 'access_not_found_within_budget',
                      budget_stop=str(exc))
    except Exception as exc:
        result.update(status='setup_or_planning_error',error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        result['wall_s'] = time.monotonic()-began
        result['source_unchanged'] = all(file_hash(source/n)==h for n,h in hashed.items())
        if not result['source_unchanged']:
            result.update(status='source_changed',full_path_found=False,prefix_found=False)
        write(output/'access_result.json',result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path); p.add_argument('--candidate',required=True)
    p.add_argument('--output',type=Path,required=True); p.add_argument('--base-policy',type=Path,required=True)
    p.add_argument('--ik-seeds',type=int,default=8);p.add_argument('--max-branches',type=int,default=3)
    p.add_argument('--route-seconds',type=float,default=8.);p.add_argument('--search-seconds',type=float,default=60.)
    p.add_argument('--max-checks',type=int,default=30000);p.add_argument('--seed',type=int,default=28)
    p.add_argument('--access-only',action='store_true')
    a=p.parse_args()
    r=solve(a.run,a.output,a.candidate,a.base_policy,ik_seeds=a.ik_seeds,max_branches=a.max_branches,
        route_seconds=a.route_seconds,total_search_seconds=a.search_seconds,max_checks=a.max_checks,
        seed=a.seed,access_only=a.access_only)
    print(json.dumps(r,indent=2,allow_nan=False))
    return 0 if r['full_path_found'] or (a.access_only and r['prefix_found']) else 2


if __name__=='__main__':
    raise SystemExit(main())
