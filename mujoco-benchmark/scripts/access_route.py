"""Bounded free-space access planning using upstream OMPL, not a new RL agent.

Only nominal configurations are checked. No physics, contact-policy changes,
wrap-around joints, automatic 'impossible' labels, or hardware commands here.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
import copy
import math
import time
from types import SimpleNamespace
import numpy as np


class AccessBudget(RuntimeError):
    pass


@dataclass(frozen=True)
class AccessConfig:
    ik_seeds: int = 8
    max_branches: int = 3
    max_state_checks: int = 12000
    max_queries: int = 2000000
    seconds: float = 30.
    solve_seconds: float = 5.
    seed: int = 20260928
    position_tolerance_m: float = .002
    rotation_tolerance_rad: float = .03

    def __post_init__(self):
        for name in ('ik_seeds','max_branches','max_state_checks','max_queries'):
            v=getattr(self,name)
            if isinstance(v,bool) or not isinstance(v,int) or v < 1:
                raise ValueError(name+' must be a positive integer')
        if self.ik_seeds>32 or self.max_branches>self.ik_seeds:
            raise ValueError('Bounded experiment: 1..32 IK seeds and branches <= seeds')
        if isinstance(self.seed,bool) or not isinstance(self.seed,int) or self.seed<1:
            raise ValueError('seed must be a positive integer')
        for name in ('seconds','solve_seconds','position_tolerance_m','rotation_tolerance_rad'):
            v=getattr(self,name)
            if isinstance(v,bool) or not math.isfinite(v) or v<=0:
                raise ValueError(name+' must be positive and finite')


class Budget:
    def __init__(self,config,clock=time.monotonic):
        self.config=config; self.clock=clock; self.start=clock()
        self.states=0; self.queries=0

    def check(self):
        if self.clock()-self.start>=self.config.seconds:
            raise AccessBudget('access_wall_budget')

    def state(self):
        self.check()
        if self.states>=self.config.max_state_checks:
            raise AccessBudget('access_state_budget')
        self.states+=1

    def query(self):
        self.check()
        if self.queries>=self.config.max_queries:
            raise AccessBudget('access_query_budget')
        self.queries+=1

    def summary(self):
        return dict(states=self.states,queries=self.queries,wall_s=self.clock()-self.start,
                    deadline_scope='cooperative; run CLI in bounded child process too')


def joint_box(bounds, steps):
    box=np.asarray(bounds,dtype=float); steps=np.asarray(steps,dtype=float)
    if (box.shape!=(2,len(steps)) or not np.isfinite(box).all()
            or not np.isfinite(steps).all() or np.any(steps<=0) or np.any(box[1]<=box[0])):
        raise ValueError('Finite scalar joint bounds and positive check steps required')
    return box,steps


def edge_valid(a,b,valid,steps):
    a,b=np.asarray(a,float),np.asarray(b,float); steps=np.asarray(steps,float)
    if a.shape!=steps.shape or b.shape!=steps.shape or not np.isfinite([a,b]).all():
        raise ValueError('Invalid edge')
    n=max(1,int(np.ceil(np.max(abs(b-a)/steps))))
    for u in np.linspace(0,1,n+1):
        result=valid(a+(b-a)*u)
        if not isinstance(result,(bool,np.bool_)):
            raise ValueError('Validity oracle must return bool; unknown is not valid')
        if not result:return False
    return True


def ik_goals(kin,position,rotation,start,valid,steps,config,budget):
    """Try bounded seeds; independently verify FK, limits and collision per solution."""
    box,steps=joint_box(kin.bounds,steps); start=np.asarray(start,float)
    rng=np.random.default_rng(config.seed)
    seeds=[start.copy()]
    if config.ik_seeds>1:seeds.append((box[0]+box[1])/2)
    while len(seeds)<config.ik_seeds:
        if len(seeds)<(config.ik_seeds+1)//2:
            seeds.append(np.clip(start+rng.normal(0,.08,len(start))*(box[1]-box[0]),*box))
        else:seeds.append(rng.uniform(*box))
    goals,records=[],[]
    for i,seed in enumerate(seeds):
        budget.check()
        q,reported_pe,reported_re=kin.ik(position,rotation,seed.copy())
        q=np.asarray(q,float)
        record=dict(seed_index=i,seed=seed.tolist())
        if q.shape!=start.shape or not np.isfinite(q).all():
            record['status']='ik_nonfinite';records.append(record);continue
        if np.any(q<box[0]) or np.any(q>box[1]):
            record['status']='joint_limits';records.append(record);continue
        p,r=kin.fk(q)
        pe=float(np.linalg.norm(np.asarray(p)-position)); re=float((r*rotation.inv()).magnitude())
        record.update(q=q.tolist(),fk_position_error_m=pe,fk_rotation_error_rad=re)
        if not np.isfinite([pe,re]).all():
            record['status']='fk_nonfinite'
        elif pe>config.position_tolerance_m or re>config.rotation_tolerance_rad:
            record['status']='ik_tolerance'
        elif any(np.max(abs(q-g)/steps)<.1 for g in goals):record['status']='duplicate_ik'
        elif not valid(q):record['status']='endpoint_blocked'
        else:goals.append(q.copy());record['status']='valid_endpoint'
        records.append(record)
    # Prefer a small move, but do not equate the nearest IK with the only solution.
    scale=np.maximum(box[1]-box[0],1e-9)
    goals.sort(key=lambda q:float(np.linalg.norm((q-start)/scale)))
    return goals,records


def connect(start,goal,valid,bounds,steps,budget,method='ompl',seed=None):
    """Return an EXACT path or a reason. OMPL approximate solutions are rejected.

    OMPL 2.0.1 nanobind API. Each CLI handles one candidate/process. Repeated
    calls share OMPL's random stream; seed is set only before its first RNG.
    """
    box,steps=joint_box(bounds,steps)
    start,goal=np.asarray(start,float),np.asarray(goal,float)
    if method not in ('direct','ompl'):raise ValueError('Unknown connection method')
    if edge_valid(start,goal,valid,steps):return np.array([start,goal]),'direct'
    if method=='direct':return None,'direct_edge_blocked'
    from ompl import base as ob, geometric as og, util as ou
    global _OMPL_SEEDED
    if not globals().get('_OMPL_SEEDED',False):
        ou.RNG.setSeed(int(seed or budget.config.seed));_OMPL_SEEDED=True
    # Scaling in radians/metres: one validation step is at most one coordinate unit.
    lo,hi=box/steps
    space=ob.RealVectorStateSpace(len(steps)); b=ob.RealVectorBounds(len(steps))
    for i in range(len(steps)):b.setLow(i,float(lo[i]));b.setHigh(i,float(hi[i]))
    space.setBounds(b); ss=og.SimpleSetup(space)
    callback_error=[]
    def validity(state):
        if callback_error:return False
        try:return bool(valid(np.array([state[i] for i in range(len(steps))])*steps))
        except Exception as exc:
            callback_error.append(exc);return False
    ss.setStateValidityChecker(validity)
    # Euclidean segment <= 1 ensures every joint delta <= its configured step.
    ss.getSpaceInformation().setStateValidityCheckingResolution(min(.01,1./space.getMaximumExtent()))
    s,g=space.allocState(),space.allocState()
    for i in range(len(steps)):s[i]=float(start[i]/steps[i]);g[i]=float(goal[i]/steps[i])
    ss.setStartAndGoalStates(s,g,1e-9)
    planner=og.RRTConnect(ss.getSpaceInformation());planner.setRange(20.)
    ss.setPlanner(planner);ss.setup()
    stop_at=min(budget.start+budget.config.seconds,budget.clock()+budget.config.solve_seconds)
    term=ob.PlannerTerminationCondition(lambda:bool(callback_error) or budget.clock()>=stop_at)
    ss.solve(term)
    if callback_error:raise callback_error[0]
    budget.check()
    if not ss.haveExactSolutionPath():return None,'search_budget_no_route'
    path=ss.getSolutionPath()
    route=np.array([[path.getState(k)[j]*steps[j] for j in range(len(steps))]
                    for k in range(path.getStateCount())],dtype=float)
    if not np.allclose(route[0],start,atol=1e-10,rtol=0) or not np.allclose(route[-1],goal,atol=1e-10,rtol=0):
        raise RuntimeError('OMPL returned a non-exact endpoint')
    route[0]=start;route[-1]=goal
    # Independent exact-edge rescan, never trust only waypoint validity.
    if not all(edge_valid(a,b,valid,steps) for a,b in zip(route[:-1],route[1:])):
        raise RuntimeError('Returned OMPL path failed independent rescan')
    return route,'ompl_rrtconnect'


def retime_route(route,dt,lift_id):
    """Piecewise joint line with cubic timing, no corner rounding or path shortcut."""
    route=np.asarray(route,float)
    if route.ndim!=2 or len(route)<2 or not np.isfinite(route).all() or not math.isfinite(dt) or dt<=0:
        raise ValueError('Bad route/timestep')
    if not 0<=lift_id<route.shape[1]:raise ValueError('Bad lift index')
    speeds=np.full(route.shape[1],.35);speeds[lift_id]=.08
    result=[]
    for a,b in zip(route[:-1],route[1:]):
        if np.array_equal(a,b):continue
        seconds=max(1.5*float(np.max(abs(b-a)/speeds)),2*dt)
        n=int(math.ceil(seconds/dt));u=np.arange(1,n+1)/n;u=u*u*(3-2*u)
        result.extend(a+u[:,None]*(b-a))
    if not result:result=[route[-1].copy()]
    return np.asarray(result)


def staging_index(rows):
    if not rows or rows[0].get('phase')!='preapproach':raise ValueError('Missing preapproach')
    first_local=next((i for i,r in enumerate(rows) if r['phase'] in ('insert','seat')),None)
    if first_local is None or first_local==0:raise ValueError('Need nonempty local suffix')
    if any(r['phase'] not in ('preapproach','approach') for r in rows[:first_local]):
        raise ValueError('Only free-space prefix can be replaced')
    return first_local-1


def shadow_start(env,q):
    """Modify only the CPU planning facade, never a simulator or live robot."""
    e=copy.copy(env);e.robot=copy.copy(env.robot);e.robot.data=copy.copy(env.robot.data)
    jp=env.robot.data.joint_pos.clone()
    jp[0]=jp.new_tensor(q);e.robot.data.joint_pos=jp
    return e


class PinnedFirstIK:
    """Prevent the legacy first waypoint from jumping back to another IK branch."""
    def __init__(self,kin,q):self.kin=kin;self.q=np.asarray(q).copy();self.first=True
    def __getattr__(self,name):return getattr(self.kin,name)
    def ik(self,p,r,seed):
        if self.first:
            self.first=False;fp,fr=self.kin.fk(self.q)
            return self.q.copy(),float(np.linalg.norm(fp-p)),float((fr*r.inv()).magnitude())
        return self.kin.ik(p,r,seed)


def access_then_local(env,kin,checker,params,valid,steps,config,legacy_plan,full_audit,method='ompl',access_only=False):
    """Replace only access, then rebuild local IK from the actual chosen branch.

    full_audit(commands,phases) must validate the emitted path with its exact
    phase policy. Prefix success alone is never whole-motion success.
    """
    from scipy.spatial.transform import Rotation
    if params.get('trajectory_mode')!='diagnostic_pose_waypoints_v1' or params.get('diagnostic_only') is not True or params.get('training_eligible') is not False:
        raise ValueError('Use explicit offline diagnostic poses only')
    rows=params['pose_waypoints'];idx=staging_index(rows)
    target=rows[idx];p=np.array(target['ring_position_xyz']);r=Rotation.from_quat(target['orientation_xyzw'])
    start=env.robot.data.joint_pos[0].cpu().numpy().copy()
    budget=valid.budget
    report=dict(mode=method,staging_index=idx,staging_pose=copy.deepcopy(target),
                removed_prefix=copy.deepcopy(rows[:idx]),config=asdict(config),attempts=[],
                access_found=False,full_motion_found=False,impossible=None,physics_executed=False)
    planned_out=None
    try:
        if not valid(start):report['reason']='initial_state_blocked';return None,report
        goals,report['ik_trials']=ik_goals(kin,p,r,start,valid,steps,config,budget)
        if not goals:report['reason']='no_valid_ik_in_seed_budget';return None,report
        for q in goals[:config.max_branches]:
            budget.check();trial=dict(staging_q=q.tolist());report['attempts'].append(trial)
            route,kind=connect(start,q,valid,kin.bounds,steps,budget,method,config.seed)
            trial['connection']=kind
            if route is None:continue
            report['access_found']=True;trial['access_path']=route.tolist()
            prefix=retime_route(route,env.step_dt,env.lift_id)
            if access_only:
                commands=np.vstack([prefix,np.repeat(q[None],max(1,round(1./env.step_dt)),axis=0)])
                phases=['preapproach']*len(prefix)+['hold']*(len(commands)-len(prefix))
                audit=full_audit(np.vstack([start,commands]),['ready']+phases)
                trial['full_audit']=audit
                if not (audit.get('passed') is True and audit.get('complete') is True):continue
                report.update(access_only_checked=True,reason='access_only_not_hooking',full_motion_found=False)
                return dict(commands=commands,phases=phases,command_positions=np.array([kin.fk(x)[0] for x in commands]),
                            waypoints=[dict(target,phase='access_goal')],access_joint_route=route),report
            suffix_params=copy.deepcopy(params)
            first=copy.deepcopy(target);first.update(phase='preapproach',minimum_seconds=0.)
            suffix_params['pose_waypoints']=[first]+copy.deepcopy(rows[idx+1:])
            suffix,check=legacy_plan(shadow_start(env,q),PinnedFirstIK(kin,q),checker,suffix_params)
            trial['local_plan']=check
            if suffix is None:continue
            commands=np.concatenate([prefix,suffix['commands']])
            phases=['preapproach']*len(prefix)+list(suffix['phases'])
            # Recheck final retimed prefix, splice, local path and phase boundaries.
            audit=full_audit(np.vstack([start,commands]),['ready']+phases)
            trial['full_audit']=audit
            if not (audit.get('passed') is True and audit.get('complete') is True):continue
            planned_out=dict(suffix,commands=commands,phases=phases,
                command_positions=np.array([kin.fk(qi)[0] for qi in commands]),
                access_joint_route=route,access_report=report)
            report.update(full_motion_found=True,reason='planned_not_physically_executed')
            return planned_out,report
        report['reason']='access_found_local_blocked' if report['access_found'] else 'no_access_within_budget'
        return None,report
    except AccessBudget as exc:
        report['reason']=str(exc);return None,report
    except Exception as exc:
        report.update(reason='setup_or_planner_error',error=f'{type(exc).__name__}: {exc}')
        return None,report
    finally:
        report['budget']=budget.summary()
