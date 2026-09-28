"""Offline paired test: legacy vs multi-IK direct vs OMPL access + same local suffix.

Only writes a new run snapshot. Never executes physics or sends robot commands.
Requires the existing trusted server planning pickle, MuJoCo and OMPL 2.0.1 in
one planning Python. Do not load planning pickles from untrusted repositories.
"""
from __future__ import annotations
import os
for _k in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'):
    os.environ.setdefault(_k,'1')
import argparse
import copy
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
import numpy as np
from access_route import AccessConfig, Budget, access_then_local, joint_box


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    tmp.replace(path)


class FrozenValidity:
    """FCL self checks plus the EXISTING MuJoCo environment geometry adapter."""
    def __init__(self,scene,checker,policy,bounds,budget,phases=('preapproach',)):
        self.scene=scene;self.checker=checker;self.policy=policy;self.budget=budget
        self.bounds,self.steps=joint_box(bounds,scene.joint_steps)
        self.failures={};self.first_failure=None
        if not scene.robot_names or not scene.environment_names:
            raise ValueError('Empty collider inventory')
        allowed={}
        for row in policy.allowed_contacts:
            if row['robot_geom'] not in scene.robot_names or row['environment_geom'] not in scene.environment_names:
                raise ValueError('Permission references missing collider')
            allowed.setdefault((row['robot_geom'],row['environment_geom']),set()).update(row['phases'])
        self.allowed={pair for pair,names in allowed.items() if set(phases)<=names}

    def reject(self,kind,q,pair=None):
        self.failures[kind]=self.failures.get(kind,0)+1
        if self.first_failure is None:
            self.first_failure=dict(kind=kind,q=np.asarray(q).tolist(),pair=pair)
        return False

    def __call__(self,q):
        from environment_preflight import nearby_indices
        q=np.asarray(q,float);self.budget.state()
        if q.shape!=self.steps.shape or not np.isfinite(q).all():
            raise ValueError('Malformed robot state')
        if np.any(q<self.bounds[0]) or np.any(q>self.bounds[1]):
            return self.reject('joint_limits',q)
        pair=self.checker.check(q)
        if pair:return self.reject('self_collision',q,[str(x) for x in pair])
        s=self.scene;s.set_robot(q);cap=self.policy.clearance_m+.01
        for i,name in enumerate(s.robot_names):
            indices=nearby_indices(s.robot_positions[i],s.robot_radii[i],
                                   s.environment_positions,s.environment_radii,cap)
            for j in indices:
                other=s.environment_names[j]
                if (name,other) in self.allowed:continue
                self.budget.query();distance,segment=s.distance(i,int(j),cap)
                if not np.isfinite(distance) or not np.isfinite(segment).all():
                    raise ValueError('Nonfinite collision oracle')
                if distance<=self.policy.clearance_m:
                    return self.reject('environment_collision',q,[name,other])
        return True


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('--candidate',required=True)
    p.add_argument('--base-policy',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=('legacy','multiseed-direct','ompl'),required=True)
    p.add_argument('--access-only',action='store_true',help='Reach staging pose and hold; NOT an insertion trial')
    p.add_argument('--ik-seeds',type=int,default=8);p.add_argument('--max-branches',type=int,default=3)
    p.add_argument('--seconds',type=float,default=30.);p.add_argument('--solve-seconds',type=float,default=5.)
    p.add_argument('--max-state-checks',type=int,default=12000)
    p.add_argument('--max-queries',type=int,default=2000000)
    p.add_argument('--seed',type=int,default=20260928)
    args=p.parse_args()
    root,out=args.run.resolve(),args.output.resolve()
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.candidate):p.error('Invalid candidate ID')
    if out.exists() or root==out or root in out.parents:p.error('Use a NEW directory outside source run')
    config=AccessConfig(ik_seeds=args.ik_seeds,max_branches=args.max_branches,
        max_state_checks=args.max_state_checks,max_queries=args.max_queries,
        seconds=args.seconds,solve_seconds=args.solve_seconds,seed=args.seed)
    names=('manifest.json','planning_inputs.json','candidates.json',
           'replay_assets/model.mjb','replay_assets/reference.json',
           'replay_assets/initial_trace.json','replay_assets/planning_model.pkl')
    hashes={n:digest(root/n) for n in names};policy_hash=digest(args.base_policy)
    source=read(root/'manifest.json')
    if source.get('model_sha256') and source['model_sha256']!=hashes['replay_assets/model.mjb']:
        raise ValueError('Model hash mismatch')
    matches=[x for x in read(root/'candidates.json') if x['candidate_id']==args.candidate]
    if len(matches)!=1:raise ValueError('Expected one candidate')
    params=copy.deepcopy(matches[0])
    out.mkdir(parents=True,exist_ok=False)
    info=dict(candidate_id=args.candidate,parameters=params,diagnostic_only=True,
        training_eligible=False,hook_success=None,physics_executed=False,
        source_run=str(root),source_sha256=hashes,policy_sha256=policy_hash,
        access_mode=args.mode,impossible=None,access_only=args.access_only)
    code_paths=[Path(__file__),Path(__file__).with_name('access_route.py')]
    info['code_sha256']={x.name:digest(x) for x in code_paths}
    folder=out/'candidates'/args.candidate
    try:
        import plan_candidates as pc
        pc.initialize(root)
        from dataset_motion import plan,diagnostic_poses
        from robot_engine import RobotEngine
        from hook_retention_diagnostic import HookProbe
        from environment_preflight import MuJoCoScene,check_engine,save_report,Policy
        from target_fruit_contact_trial import scope_from_engine,trial_policy
        e,kin,self_checker=pc.MODEL
        # Trusted, already exported model. No mesh conversion or altered physical parameters.
        engine=RobotEngine(root/'replay_assets/model.mjb',root/'replay_assets/initial_trace.json',
                           source['hz'],reference=root/'replay_assets/reference.json',target=source['target'])
        if not np.allclose(pc.START,engine.initial,atol=1e-12,rtol=0):
            raise ValueError('Planner and physics starts differ')
        if abs(e.step_dt-1/60)>1e-12:raise ValueError('Expected existing 60Hz command trace')
        diagnostic_poses(params)
        scope=scope_from_engine(engine,HookProbe(engine,0.),search_policy=True)
        base=read(args.base_policy)
        all_phases=['ready']+[x['phase'] for x in params['pose_waypoints']]
        policy=trial_policy(base,scope,all_phases)
        scene=MuJoCoScene(engine,policy)
        budget=Budget(config)
        valid=FrozenValidity(scene,self_checker,policy,kin.bounds,budget)
        def audit(commands,phases):
            settings=copy.deepcopy(policy.snapshot()['settings'])
            present=set(phases)
            settings['allowed_contacts']=[dict(x,phases=sorted(set(x['phases'])&present)) for x in settings['allowed_contacts'] if set(x['phases'])&present]
            exact_policy=Policy.from_dict(settings)
            rows=[dict(command=q.tolist(),phase=phase) for q,phase in zip(commands,phases)]
            return check_engine(engine,rows,(len(rows)-1)/60,exact_policy,collect_all_violations=False)
        if args.mode=='legacy':
            legacy_params=copy.deepcopy(params)
            if args.access_only:
                from access_route import staging_index
                end=staging_index(legacy_params['pose_waypoints'])
                hold=dict(legacy_params['pose_waypoints'][end],phase='hold',minimum_seconds=1.)
                legacy_params['pose_waypoints']=legacy_params['pose_waypoints'][:end+1]+[hold]
            planned,check=plan(e,kin,self_checker,legacy_params)
            report=dict(mode='legacy',original_planner_check=check,access_found=None,
                        full_motion_found=False,impossible=None)
            if planned is not None:
                a=audit(np.vstack([pc.START,planned['commands']]),['ready']+planned['phases'])
                report['full_audit']=a
                checked=bool(a.get('passed') and a.get('complete'))
                report['full_motion_found']=checked and not args.access_only
                report['access_only_checked']=checked and args.access_only
                if not checked:planned=None
        else:
            planned,report=access_then_local(e,kin,self_checker,params,valid,scene.joint_steps,
                config,plan,audit,method='direct' if args.mode=='multiseed-direct' else 'ompl',access_only=args.access_only)
        report.update(validity_rejections=valid.failures,first_validity_rejection=valid.first_failure,
                      policy=policy.snapshot(),geometry_inventory=scene.inventory)
        write(out/'access_report.json',report)
        info['preflight']=dict(passed=planned is not None,reason=report.get('reason','see_access_report'))
        info['access_report']='../../access_report.json'
        if planned is not None:
            # Independent snapshot: no hardlink or writeable alias to source assets.
            for name in names:
                if name in ('candidates.json','manifest.json'):continue
                dest=out/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/name,dest)
            if (root/'action_frame.json').is_file():shutil.copy2(root/'action_frame.json',out/'action_frame.json')
            manifest=copy.deepcopy(source);manifest.pop('environment_preflight_policy',None)
            manifest.update(count=1,diagnostic_only=True,training_eligible=False,hook_success=None,
                            planning_backend='access_route_v1',source_run=str(root),access_only=args.access_only)
            write(out/'manifest.json',manifest);write(out/'candidates.json',[params])
            rows=[dict(joints=pc.START.tolist(),command=pc.START.tolist(),phase='ready')]
            rows.extend(dict(joints=pc.START.tolist(),command=q.tolist(),phase=phase)
                        for q,phase in zip(planned['commands'],planned['phases']))
            write(folder/'trace.json',rows)
            info.update(seconds=(len(rows)-1)/60,waypoints=planned['waypoints'],
                        command_trace_sha256=digest(folder/'trace.json'),
                        note='Authoritative joint trace; ordinary plan_candidates would regenerate the legacy prefix')
        from dataclasses import asdict
        write(out/'contact_scope.json',asdict(scope))
    except Exception as exc:
        info.update(preflight=dict(passed=False,reason='setup_or_planner_error'),
                    error=f'{type(exc).__name__}: {exc}')
    finally:
        unchanged=all(digest(root/n)==h for n,h in hashes.items()) and digest(args.base_policy)==policy_hash
        info['source_unchanged']=unchanged
        if not unchanged:info['preflight']=dict(passed=False,reason='source_changed')
        write(folder/'plan.json',info)
        write(out/'result.json',info)
    print(json.dumps({k:info.get(k) for k in ('candidate_id','access_mode','preflight','error','source_unchanged')},indent=2))
    return 0 if info.get('preflight',{}).get('passed') and info['source_unchanged'] else 2


if __name__=='__main__':raise SystemExit(main())
