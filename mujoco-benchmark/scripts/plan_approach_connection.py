"""Opt-in approach replanning on an immutable, existing target run.

Reuses exported IK/FCL and native MuJoCo environment geometry. Replaces the full
leading preapproach/approach prefix; regenerates the SAME local suffix from the
reached IK branch. It never executes physics, alters contact/material policies,
or counts a prefix-only solution as harvesting success. Requires ompl==2.0.1
in the planning environment for non-straight connections.
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
import time
import numpy as np
from scipy.spatial.transform import Rotation
from approach_connection import (CheckBudget, JointSpace, LimitReached, check_polyline,
                                 handoff_index, ik_pool, ompl_connect, retime,
                                 stitch, suffix_parameters)


class SceneValidity:
    """Existing native collider sets + existing FCL self pairs, in a private state.

    Exact configuration cache; never quantize away a small collision. Exceptions
    and missing geometry fail closed. Contact exceptions are pair AND phase exact.
    """
    def __init__(self, scene, checker, policy, space, budget, phases=('preapproach','approach')):
        self.scene,self.checker,self.policy,self.space,self.budget=scene,checker,policy,space,budget
        self.phases=frozenset(phases); self.cache={};self.counts=Counter();self.hits=[];self.queries=0
        self.allowed={}
        for r in policy.allowed_contacts:
            if r['robot_geom'] not in scene.robot_names or r['environment_geom'] not in scene.environment_names:
                raise ValueError('Unknown contact-policy geom')
            self.allowed.setdefault((r['robot_geom'],r['environment_geom']),set()).update(r['phases'])
        if not scene.robot_names or not scene.environment_names:
            raise ValueError('Empty collision inventory')

    def __call__(self,q):
        from environment_preflight import nearby_indices
        self.budget.tick()
        q=np.asarray(q,float)
        key=q.tobytes()
        if key in self.cache:
            self.counts['cache_hit']+=1; return self.cache[key]
        if not self.space.contains(q):return self._store(key,False,'joint_bounds')
        pair=self.checker.check(q)
        if pair is not None:return self._store(key,False,'self_collision',pair=pair)
        self.scene.set_robot(q)
        limit=self.policy.clearance_m+.01
        for i,a in enumerate(self.scene.robot_names):
            indices=nearby_indices(self.scene.robot_positions[i],self.scene.robot_radii[i],
                                  self.scene.environment_positions,self.scene.environment_radii,limit)
            for j in indices:
                b=self.scene.environment_names[int(j)]
                if self.phases and self.phases<=self.allowed.get((a,b),set()):continue
                if self.queries>=self.policy.max_distance_queries:raise LimitReached('distance_query_budget')
                if self.budget.remaining()<=0:raise LimitReached('wall_time_budget')
                d,segment=self.scene.distance(i,int(j),limit);self.queries+=1
                if not math.isfinite(d) or not np.isfinite(segment).all():raise ValueError('Invalid distance query')
                if d<=self.policy.clearance_m:
                    return self._store(key,False,'environment_collision',pair=[a,b],distance_m=float(d))
        return self._store(key,True,'valid')

    def _store(self,key,answer,kind,**details):
        self.cache[key]=answer;self.counts[kind]+=1
        if not answer and len(self.hits)<24:self.hits.append(dict(kind=kind,**details))
        return answer

    def summary(self):
        return dict(checks=self.budget.checks,distance_queries=self.queries,counts=dict(self.counts),
                    first_rejections=self.hits,phases=sorted(self.phases),
                    inventory=self.scene.inventory,policy=self.policy.snapshot())


def jsonable(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {str(k):jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [jsonable(v) for v in value]
    return value


def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(jsonable(value),indent=2,allow_nan=False)+'\n')
    tmp.replace(path)


def file_hash(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def plan_suffix(env,kin,checker,params,q,plan_fn):
    """Clone only the small planning environment, not live simulation state."""
    import torch
    local=copy.deepcopy(env)
    local.robot.data.joint_pos=torch.as_tensor(np.asarray(q)[None].copy(),dtype=env.robot.data.joint_pos.dtype)
    planned,check=plan_fn(local,kin,checker,suffix_parameters(params,q,kin))
    if planned is not None:
        planned=copy.deepcopy(planned)
        planned['phases']=['approach' if p=='preapproach' else p for p in planned['phases']]
        for pose in planned.get('waypoints',[]):
            if pose['phase']=='preapproach':pose['phase']='approach'
    return planned,check


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--candidate',required=True)
    parser.add_argument('--base-policy',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ik-seeds',type=int,default=8)
    parser.add_argument('--max-branches',type=int,default=3)
    parser.add_argument('--solve-seconds',type=float,default=8.)
    parser.add_argument('--total-seconds',type=float,default=120.)
    parser.add_argument('--max-state-checks',type=int,default=100000)
    parser.add_argument('--max-prefix-seconds',type=float,default=180.)
    parser.add_argument('--seed',type=int,default=20260928)
    args=parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.candidate):parser.error('Invalid candidate id')
    root,out=args.run.resolve(),args.output.resolve()
    if out.exists() or root==out or root in out.parents:parser.error('Use a new output outside the source run')
    if not 1<=args.max_branches<=8 or not 1<=args.ik_seeds<=32:parser.error('Invalid branch/seed budget')
    if not all(math.isfinite(x) and x>0 for x in (args.solve_seconds,args.total_seconds,args.max_prefix_seconds)):
        parser.error('Positive finite time budgets required')
    # All heavy imports are delayed so --help works without server assets.
    from run_motion_family_search import REQUIRED,prepare_snapshot
    import plan_candidates as planner
    from dataset_motion import plan as legacy_plan, diagnostic_poses
    from robot_engine import RobotEngine
    from hook_retention_diagnostic import HookProbe
    from target_fruit_contact_trial import scope_from_engine,trial_policy
    from environment_preflight import MuJoCoScene,check_engine,save_report
    from ompl import util as ou
    ou.RNG.setSeed(args.seed)  # CLI is a fresh process; do not reset global RNG between goals.
    params=next((r for r in json.loads((root/'candidates.json').read_text()) if r['candidate_id']==args.candidate),None)
    if params is None:raise ValueError('Candidate not found')
    diagnostic_poses(params)
    boundary=handoff_index(params);handoff=params['pose_waypoints'][boundary]
    inputs=[root/p for p in REQUIRED]+[root/'candidates.json',args.base_policy.resolve()]
    hashes={str(p):file_hash(p) for p in inputs}
    out.mkdir();run=out/'run'
    manifest=prepare_snapshot(dict(source_run=str(root)),run)
    write(run/'candidates.json',[params])
    started=time.monotonic()
    report=dict(schema='approach_connection_v1',source_run=str(root),source_candidate=args.candidate,
                source_sha256=hashes,code_sha256={n:file_hash(Path(__file__).with_name(n)) for n in
                    ('approach_connection.py','plan_approach_connection.py')},
                handoff_index=boundary,handoff=handoff,attempts=[],
                prefix_found=False,approach_only_runs=[],whole_path_passed=False,physics_executed=False,
                training_eligible=False,hook_success=None,impossible=None,
                dependency='ompl==2.0.1',status='setup_incomplete')
    valid=None
    import signal
    def deadline_handler(signum,frame):
        raise LimitReached('overall_planning_deadline')
    old_handler=signal.signal(signal.SIGALRM,deadline_handler)
    signal.setitimer(signal.ITIMER_REAL,args.total_seconds)
    try:
        planner.initialize(str(run));env,kin,selfchecker=planner.MODEL
        start=np.asarray(planner.START,float)
        if abs(env.step_dt-1/60)>1e-12:raise ValueError('Expected existing 60 Hz trace contract')
        engine=RobotEngine(run/'replay_assets/model.mjb',run/'replay_assets/initial_trace.json',manifest['hz'],
                           reference=run/'replay_assets/reference.json',target=manifest['target'])
        if manifest.get('model_sha256')!=file_hash(run/'replay_assets/model.mjb'):
            raise ValueError('Model hash mismatch')
        if not np.allclose(start,engine.initial,atol=1e-12,rtol=0):raise ValueError('Initial joint mismatch')
        mapping=HookProbe(engine,0.)
        scope=scope_from_engine(engine,mapping,search_policy=True)
        phases=['ready','preapproach','approach']+[r['phase'] for r in params['pose_waypoints'][boundary+1:]]
        base=json.loads(args.base_policy.read_text())
        policy=trial_policy(base,scope,phases)
        scene=MuJoCoScene(engine,policy)
        space=JointSpace(kin.bounds,scene.joint_steps)
        budget=CheckBudget(args.max_state_checks,args.total_seconds)
        valid=SceneValidity(scene,selfchecker,policy,space,budget)
        if not valid(start):
            report['status']='invalid_initial_configuration'
        else:
            # Check actual native RING FK agrees with exported IK for every accepted solution.
            def aligned_valid(q):
                if not valid(q):return False
                scene.set_robot(q)
                p,r=mapping.frame(scene.data);kp,kr=kin.fk(q)
                if np.linalg.norm(p-kp)>1e-5 or (r*kr.inv()).magnitude()>1e-5:
                    raise ValueError('Native/IK FK disagreement at candidate configuration')
                return True
            goals,ik_records=ik_pool(kin,handoff['ring_position_xyz'],Rotation.from_quat(handoff['orientation_xyzw']),
                                    start,space,aligned_valid,count=args.ik_seeds,seed=args.seed)
            report.update(ik_records=ik_records,valid_ik_branches=len(goals),status='no_valid_ik_goal_found')
            velocity=np.full(space.n,.35);velocity[env.lift_id]=.08
            max_velocity=np.ones(space.n);max_velocity[env.lift_id]=.25
            for index,qgoal in enumerate(goals[:args.max_branches]):
                if budget.remaining()<=0:raise LimitReached('wall_time_budget')
                item=dict(branch_index=index,qgoal=qgoal.tolist());report['attempts'].append(item)
                route=ompl_connect(start,qgoal,space,valid,solve_seconds=min(args.solve_seconds,budget.remaining()))
                item.update({k:jsonable(v) for k,v in route.items() if k!='path'})
                report['status']='no_complete_connection_found'
                if route['path'] is None:continue
                report['prefix_found']=True
                branch=out/f'branch_{index:02d}';branch.mkdir()
                prefix=retime(route['path'],velocity,max_seconds=args.max_prefix_seconds)
                write(branch/'prefix_joint_path.json',dict(path=route['path'],commands=prefix,command_hz=60,
                      geometry_only=True,physics_executed=False,policy=policy.snapshot()))
                item['prefix_seconds']=(len(prefix)-1)/60
                # Isolate actual arrival from downstream contact skill. A local
                # suffix failure must not prevent a clearly labelled approach test.
                approach_rows=stitch(prefix,np.repeat(qgoal[None],60,axis=0),['hold']*60,start,limits=max_velocity)
                approach_self=check_polyline(np.array([r['command'] for r in approach_rows]),space,
                                             lambda q:selfchecker.check(q) is None)
                item['approach_only_self_check']=approach_self
                if not approach_self['passed']:
                    item['status']='retimed_prefix_self_check_failed';continue
                approach_policy=trial_policy(base,scope,[r['phase'] for r in approach_rows])
                approach_audit=check_engine(engine,approach_rows,(len(approach_rows)-1)/60,approach_policy)
                write(branch/'approach_only_audit.json',approach_audit)
                item['approach_only_audit']=approach_audit['status']
                if approach_audit.get('passed') and approach_audit.get('complete'):
                    arun=branch/'approach_only_run'
                    prepare_snapshot(dict(source_run=str(root)),arun)
                    ap=copy.deepcopy(params);ap['candidate_id']=args.candidate+'_approach'
                    ap.update(approach_only=True,manipulation_attempt=False)
                    ap['pose_waypoints']=suffix_parameters(params,qgoal,kin)['pose_waypoints'][:1]
                    ap['pose_waypoints'].append(dict(ap['pose_waypoints'][0],phase='hold',minimum_seconds=1.))
                    write(arun/'candidates.json',[ap])
                    adest=arun/'candidates'/ap['candidate_id'];adest.mkdir(parents=True)
                    write(adest/'trace.json',approach_rows)
                    write(adest/'plan.json',dict(candidate_id=ap['candidate_id'],parameters=ap,
                          seconds=(len(approach_rows)-1)/60,preflight=dict(passed=True),
                          waypoints=[],approach_only=True,manipulation_attempt=False,
                          diagnostic_only=True,training_eligible=False,hook_success=None))
                    report['approach_only_runs'].append(dict(run=str(arun),candidate=ap['candidate_id'],
                          branch_index=index,manipulation_attempt=False))
                planned,check=plan_suffix(env,kin,selfchecker,params,qgoal,legacy_plan)
                item['suffix_planner']=check
                if planned is None:
                    item['status']='prefix_found_suffix_planning_rejected';continue
                rows=stitch(prefix,planned['commands'],planned['phases'],start,limits=max_velocity)
                # Production self and environment checks are not replaced by OMPL's verdict.
                self_result=check_polyline(np.array([r['command'] for r in rows]),space,
                                           lambda q:selfchecker.check(q) is None)
                item['whole_self_check']=self_result
                if not self_result['passed']:
                    item['status']='whole_self_check_failed';continue
                audit=check_engine(engine,rows,(len(rows)-1)/60,policy,collect_all_violations=False)
                save_report(branch,audit);item['whole_environment_check']=audit['status']
                if not (audit.get('passed') and audit.get('complete')):
                    item.update(status='prefix_found_suffix_environment_rejected',first_violation=audit.get('first_violation'))
                    continue
                dest=run/'candidates'/args.candidate;dest.mkdir(parents=True)
                write(dest/'trace.json',rows)
                write(dest/'plan.json',dict(candidate_id=args.candidate,parameters=params,seconds=(len(rows)-1)/60,
                      preflight=dict(passed=True,approach_connector='OMPL_RRTConnect',whole_environment_passed=True),
                      waypoints=planned['waypoints'],approach_connection=dict(handoff_index=boundary,
                      branch_index=index,joint_path=route['path'],prefix_seconds=item['prefix_seconds']),
                      diagnostic_only=True,training_eligible=False,hook_success=None))
                item['status']='whole_path_passed'
                report.update(status='whole_path_passed',whole_path_passed=True,selected_branch=index,
                              runnable_run=str(run),seconds=(len(rows)-1)/60)
                break
    except LimitReached as error:
        report.update(status='budget_exhausted',stop_reason=str(error))
    except Exception as error:
        report.update(status='execution_error',error=f'{type(error).__name__}: {error}')
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        signal.signal(signal.SIGALRM,old_handler)
        if valid is not None:report['validity']=valid.summary()
        report['wall_s']=time.monotonic()-started
        report['source_unchanged']=all(file_hash(p)==h for p,h in hashes.items())
        if not report['source_unchanged']:
            report.update(status='source_changed',whole_path_passed=False)
        write(out/'connection_result.json',report)
        print(json.dumps({k:report[k] for k in ('status','prefix_found','whole_path_passed','physics_executed','wall_s')},indent=2))
    return 0 if report['whole_path_passed'] and report['source_unchanged'] else 2


if __name__=='__main__':
    raise SystemExit(main())
