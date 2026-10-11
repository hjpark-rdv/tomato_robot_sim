"""Opt-in CPU prefix connection using the existing bounded OMPL backend.

This imports no native checkpoint and steps no physics. It preserves the
existing staging joints and complete manipulation suffix. The resulting plan
still requires the existing full self/preparation audits and actual execution.
An unsuccessful finite query never proves that harvesting is impossible.
"""
import argparse
import copy
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
SOURCE_BYTES=Path(__file__).read_bytes()
APPROACH_SCRIPTS=ROOT/'mujoco-benchmark/scripts'
if str(APPROACH_SCRIPTS) not in sys.path:sys.path.append(str(APPROACH_SCRIPTS))
from approach_connection import (CheckBudget,JointSpace,LimitReached,
                                 check_polyline,ompl_connect,retime)

MANUAL_POLICIES=('wrist2_then_wrist1','wrist1_then_wrist2',
    'arm_then_wrist_then_base','arm_then_base_then_wrist',
    'unfold_then_wrist_then_base','high_lift_unfold_then_wrist_then_base',
    'high_lift_arm_then_wrist_then_base','high_lift_clear_yaw_unfold')


def manual_prefix(start,goal,policy):
    """Eight finite orderings of one exact staging goal, never new entry poses."""
    if policy not in MANUAL_POLICIES:raise ValueError('Unknown bounded manual connector policy')
    start=np.asarray(start,float);goal=np.asarray(goal,float)
    if start.shape!=(7,) or goal.shape!=(7,) or not np.isfinite([start,goal]).all():
        raise ValueError('Finite seven-DOF connector endpoints required')
    q=start.copy();path=[q.copy()]
    def set_goal(indices):
        q[indices]=goal[indices];path.append(q.copy())
    def unfold():
        q[2],q[3]=-.65,1.2;path.append(q.copy())
    if policy.startswith('high_lift'):
        q[0]=max(.35,start[0],goal[0]);path.append(q.copy())
    if policy=='wrist2_then_wrist1':
        set_goal([5]);set_goal([4]);set_goal([1]);set_goal([2,3,6])
    elif policy=='wrist1_then_wrist2':
        set_goal([4]);set_goal([5]);set_goal([2,3]);set_goal([1,6])
    elif policy in ('arm_then_wrist_then_base','high_lift_arm_then_wrist_then_base'):
        set_goal([2,3]);set_goal([4,5,6]);set_goal([1])
    elif policy=='arm_then_base_then_wrist':
        set_goal([2,3]);set_goal([1]);set_goal([4,5,6])
    elif policy in ('unfold_then_wrist_then_base','high_lift_unfold_then_wrist_then_base'):
        unfold();set_goal([4,5,6]);set_goal([1]);set_goal([2,3])
    elif policy=='high_lift_clear_yaw_unfold':
        q[1]=0.;path.append(q.copy());unfold();set_goal([4,5,6]);set_goal([1]);set_goal([2,3])
    path.append(goal.copy())
    return np.asarray(path)


def prefix_indices(plan):
    """Only replace the actual joint connector and its exact reversed return."""
    phases=plan.get('phases',[])
    selected={name:[i for i,p in enumerate(phases) if p.get('name')==name]
        for name in ('prepare_joint_branch','return_joint_branch')}
    if any(len(v)!=1 for v in selected.values()):
        raise ValueError('Exactly one existing joint preparation and return are required')
    begin,back=(selected[name][0] for name in ('prepare_joint_branch','return_joint_branch'))
    if not begin<back or not any(p.get('name')=='approach' for p in phases[begin+1:back]):
        raise ValueError('Existing insertion suffix between preparation and return is required')
    a=np.asarray(phases[begin]['waypoint_q'],float);b=np.asarray(phases[back]['waypoint_q'],float)
    if a.shape!=(2,7) or b.shape!=(2,7) or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('Existing seven-DOF direct connector required')
    if not np.array_equal(a,b[::-1]):raise ValueError('Return is not the exact existing reversed connector')
    for p in (phases[begin],phases[back]):
        if p.get('belt_local_y_m_s')!=[0.,0.]:raise ValueError('Connector belts must stay off')
        if p.get('rail_waypoint_positions_m') not in (None,[0.,0.]):
            raise ValueError('Independent fresh connector cannot alter the source rail station')
    return begin,back,a


def timed_edges(path,velocity,*,hz=120,name,gap):
    """Use backend retiming once; native smoothstep is authored per checked edge."""
    path=np.asarray(path,float);velocity=np.asarray(velocity,float)
    if (path.ndim!=2 or path.shape[1]!=7 or len(path)<2 or not np.isfinite(path).all() or
            velocity.shape!=(7,) or not np.isfinite(velocity).all() or np.any(velocity<=0)):
        raise ValueError('Finite seven-DOF checked path and positive speed caps required')
    phases=[]
    for a,b in zip(path[:-1],path[1:]):
        if np.array_equal(a,b):continue
        commands=retime(np.stack((a,b)),velocity,hz=hz,max_seconds=120.)
        duration=(len(commands)-1)/hz
        peak=1.5*np.abs(b-a)/duration
        if np.any(peak>velocity+1e-10):raise ValueError('Smoothstep retiming exceeds the recorded caps')
        phases.append(dict(name=f'{name}_{len(phases):03d}',duration_s=float(duration),
            waypoint_q=[a.tolist(),b.tolist()],end_gap_m=float(gap),belt_local_y_m_s=[0.,0.],
            rail_waypoint_positions_m=[0.,0.]))
    if not phases:
        phases=[dict(name=name+'_000',duration_s=1/hz,waypoint_q=[path[0].tolist(),path[-1].tolist()],
            end_gap_m=float(gap),belt_local_y_m_s=[0.,0.],rail_waypoint_positions_m=[0.,0.])]
    return phases


def splice_connected_plan(plan,path,velocity,*,hz=120):
    begin,back,original=prefix_indices(plan);path=np.asarray(path,float)
    if (path.ndim!=2 or path.shape[1]!=7 or len(path)<2 or not np.isfinite(path).all() or
            not np.array_equal(path[0],original[0]) or not np.array_equal(path[-1],original[-1])):
        raise ValueError('Connected prefix must retain exact source and staging joints')
    phases=plan['phases']
    forward=timed_edges(path,velocity,hz=hz,name='prepare_ompl',gap=phases[begin]['end_gap_m'])
    reverse=timed_edges(path[::-1],velocity,hz=hz,name='return_ompl',gap=phases[back]['end_gap_m'])
    result=copy.deepcopy(plan)
    result['phases']=copy.deepcopy(phases[:begin])+forward+copy.deepcopy(phases[begin+1:back])+reverse+copy.deepcopy(phases[back+1:])
    if result.get('belt_cycle') is not None:
        window=result['belt_cycle'];names=window.get('phases',[])
        indices=[i for i,p in enumerate(result['phases']) if p['name'] in names]
        if (not names or len(indices)!=len(names) or indices!=list(range(indices[0],indices[-1]+1)) or
                [result['phases'][i]['name'] for i in indices]!=names):
            raise ValueError('Existing continuous belt phase window is incomplete or reordered')
        delta=sum(p['duration_s'] for p in forward)-phases[begin]['duration_s']
        expected_start=plan['belt_cycle']['start_after_settle_s']+delta
        actual_start=sum(p['duration_s'] for p in result['phases'][:indices[0]])
        actual_duration=sum(result['phases'][i]['duration_s'] for i in indices)
        if (not math.isclose(expected_start,actual_start,abs_tol=1e-10,rel_tol=0) or
                not math.isclose(window['duration_s'],actual_duration,abs_tol=1e-10,rel_tol=0)):
            raise ValueError('Source belt clock does not match its actual command phase stream')
        # Preserve the original clock's arithmetic provenance, so the importer
        # can check the exact delta without float summation-order differences.
        window['start_after_settle_s']=expected_start
    return result


def reconstruct_phase_tuples(plan):
    """Existing run_motion six-field command representation; no new executor."""
    result=[]
    for phase in plan['phases']:
        q=np.asarray(phase['waypoint_q'],float)
        duration=phase['duration_s'];gap=phase['end_gap_m'];belt=phase['belt_local_y_m_s']
        if (q.ndim!=2 or q.shape[1]!=7 or len(q)<2 or not np.isfinite(q).all() or
                not np.isfinite([duration,gap,*belt]).all() or duration<=0 or len(belt)!=2):
            raise ValueError('Malformed existing native phase tuple representation')
        result.append((phase['name'],float(duration),q,float(gap),*map(float,belt)))
    return result


def full_environment_trace(adapter,plan,*,hz=120):
    """Reuse the existing checker and sampler over every original phase.

    This only extends the audited phase range. The preparation inventory's
    exact target/tool and declared-compliant-leaf classification stays intact.
    """
    from motion_self_collision import check_plan
    result=dict(schema='s300-full-plan-environment-diagnostic-v1',passed=False,
        complete=False,status='inconclusive',requested_hz=hz,
        selected_phases=[p['name'] for p in plan['phases']],native_runs=0,
        physics_unchanged=True,training_eligible=False,harvest_success=None,
        conservative_source_geometry=True,actual_arrival_proven=False,
        scope='Every unchanged plan phase, including entry, belts, and return')
    try:
        if hz!=120 or adapter.scope!='preparation':
            raise ValueError('Existing preparation contact policy and full120Hz sampling required')
        adapter.prepare_path_sweep(plan,hz)
        sampled=check_plan(adapter,plan,hz=hz,cache_exact_repeats=False)
        result.update(complete=sampled['complete'],samples=sampled['samples'],sampling=sampled,
            passed=sampled['complete'] and not adapter.total_hits,
            status='blocked' if adapter.total_hits else 'sampled_clear' if sampled['complete'] else 'inconclusive')
    except Exception as exc:
        result.update(error=type(exc).__name__+': '+str(exc),
            status='blocked' if adapter.total_hits else 'inconclusive')
    result.update(hard_intersection_sample_pairs=adapter.total_hits,hard_hits=adapter.hits,
        all_hard_pair_counts=list(adapter.hit_pairs.values()),
        diagnostic_intersection_sample_pairs=adapter.total_diagnostics,
        diagnostic_only_hits=adapter.diagnostics,
        all_diagnostic_pair_counts=list(adapter.diagnostic_pairs.values()),
        distance_queries=adapter.queries,filtered_sample_pairs=adapter.filtered_pairs,
        geometry=adapter.metadata,last_sample_time_s=adapter.current_time_s,
        existing_target_contact_policy_unchanged=True)
    return result


def existing_ompl():
    """Reuse installed matching-ABI bindings without replacing asset dependencies."""
    site=ROOT/'mujoco-benchmark/.recovery-venv/lib/python3.11/site-packages'
    if sys.version_info[:2]!=(3,11) or not (site/'ompl').is_dir():
        raise ValueError('Existing CPython3.11 OMPL recovery package is required')
    if str(site) not in sys.path:sys.path.append(str(site))
    from ompl import util
    import ompl
    if Path(ompl.__file__).resolve().parent!=site/'ompl':
        raise ValueError('Unexpected OMPL package origin')
    return util,dict(version=importlib.metadata.version('ompl'),package=str(site/'ompl'),
        installation_changes=False,dependency_scope='Existing recovery OMPL only; asset USD/FCL/NumPy retain their original origins')


def connect(assembly,source_plan,output,*,seed=2026,solve_seconds=60.,max_state_checks=10000,
            max_collision_queries=200000,velocity=None,max_prefix_seconds=120.,manual_policy=None,native_tool_cache=None):
    from pxr import Usd
    from compact_motion_assets import file_sha256,verify_compact_dependencies
    from motion_kinematics import ToolKinematics
    from motion_self_collision import MotionSelfCollision,check_plan
    from rail_environment_preflight import RailEnvironmentCheck,screen_preparation_plan
    from eligible_attachment_transfer import FRESH_SCHEMA
    assembly,source_plan,output=map(lambda p:Path(p).resolve(),(assembly,source_plan,output))
    if output.exists():raise FileExistsError(output)
    if not math.isfinite(solve_seconds) or not 0<solve_seconds<=60:
        raise ValueError('A finite solve budget at most60seconds is required')
    if not math.isfinite(max_prefix_seconds) or not 0<max_prefix_seconds<=120:
        raise ValueError('Finite prefix duration at most120seconds is required')
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    if manifest.get('fresh_campaign',{}).get('schema')!=FRESH_SCHEMA:
        raise ValueError('Only explicitly independent authored fresh campaigns are supported')
    fresh=manifest['fresh_campaign']
    if fresh.get('inherited_native_breaks')!=[] or fresh.get('accepted_previous_route_history') is not False:
        raise ValueError('Fresh connector may not import accepted prior history or native breaks')
    if file_sha256(assembly/'scene.usda')!=manifest['scene_sha256']:raise ValueError('Model SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    plan=json.loads(source_plan.read_text());begin,back,endpoints=prefix_indices(plan)
    if not np.array_equal(np.asarray(plan['phases'][0]['waypoint_q'][0]),manifest['initial_robot_q']):
        raise ValueError('Plan does not begin at the exact fresh authored normal joints')
    output.mkdir(parents=True)
    code_folder=output/'code';code_folder.mkdir()
    code_sources={'connect_s300_prefix.py':SOURCE_BYTES,
        'approach_connection.py':(APPROACH_SCRIPTS/'approach_connection.py').read_bytes(),
        'rail_environment_preflight.py':Path(__file__).with_name('rail_environment_preflight.py').read_bytes()}
    for name,data in code_sources.items():(code_folder/name).write_bytes(data)
    report=dict(schema='s300-existing-ompl-prefix-connection-v1',status='setup_pending',
        source_assembly=str(assembly),source_scene_sha256=manifest['scene_sha256'],
        source_usdc_sha256=manifest['compact_assets']['materialized_dependency']['sha256'],
        source_plan=str(source_plan),source_plan_sha256=file_sha256(source_plan),
        code_sha256=file_sha256(code_folder/'connect_s300_prefix.py'),
        backend_sha256=file_sha256(code_folder/'approach_connection.py'),
        environment_adapter_sha256=file_sha256(code_folder/'rail_environment_preflight.py'),
        seed=seed,solve_seconds=solve_seconds,max_state_checks=max_state_checks,
        max_prefix_seconds=max_prefix_seconds,
        manual_policy=manual_policy,proposal_scope=('One of eight fixed joint orderings' if manual_policy else 'Existing bounded OMPL query'),
        seven_dof=True,rail_station_unchanged=True,physics_executed=False,native_eligible=False,
        training_eligible=False,harvest_success=None,impossible=None)
    def write(name,value):
        (output/name).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    started=time.perf_counter()
    try:
        stage=Usd.Stage.Open(str(assembly/'scene.usda'));kin=ToolKinematics(stage)
        moving=MotionSelfCollision(stage,kin,clearance_m=.002,native_tool_cache=native_tool_cache,assembly=assembly)
        if native_tool_cache is not None:
            report['native_tool_cache']=dict(path=str(Path(native_tool_cache).resolve()),sha256=file_sha256(native_tool_cache))
            report['native_tool_hull_import']=moving.metadata['native_tool_hull_import']
            for name in ('motion_self_collision.py','native_tool_hulls.py'):
                (code_folder/name).write_bytes(Path(__file__).with_name(name).read_bytes())
            report['self_checker_sha256']=file_sha256(code_folder/'motion_self_collision.py')
            report['native_tool_importer_sha256']=file_sha256(code_folder/'native_tool_hulls.py')
        truss=fresh['truss_key'];environment=RailEnvironmentCheck(stage,manifest,moving,
            allowed_truss_key=truss,scope='preparation',max_distance_queries=max_collision_queries)
        if not hasattr(environment,'check_state'):
            raise ValueError('Unpruned arbitrary-state environment API is not available')
        speed=np.array([.08]+[.35]*6) if velocity is None else np.asarray(velocity,float)
        if speed.shape!=(7,) or np.any(speed<=0) or not np.isfinite(speed).all():
            raise ValueError('Finite positive seven-DOF speed caps required')
        # Existing approach backend uses these metres/radians edge steps;
        # final complete120Hz command audits are separate from RRT samples.
        space=JointSpace(kin.bounds,[.002]+[np.deg2rad(1.)]*6)
        budget=CheckBudget(max_state_checks,solve_seconds)
        gaps=sorted({float(plan['phases'][index]['end_gap_m']) for index in (begin,back)})
        rejections=[];incomplete_states=[]
        def valid(q):
            budget.tick()
            for gap in gaps:
                hits=moving.check(q,gap,rail_position_m=0.)
                if hits:
                    if len(rejections)<64:rejections.append(dict(kind='self',q=np.asarray(q).tolist(),gap_m=gap,hits=hits))
                    return False
                if manual_policy is not None and native_tool_cache is not None:
                    # Re-screen the same fixed manual proposals with complete
                    # self geometry first. The full120Hz self certificate still
                    # precedes every environment certificate below.
                    continue
                checked=environment.check_state(q,gap,rail_position_m=0.)
                if not checked.get('complete'):
                    incomplete_states.append(dict(q=np.asarray(q).tolist(),gap_m=gap,check=checked))
                    if 'Finite environment query budget exhausted' in checked.get('error',''):
                        raise LimitReached('environment_query_budget')
                    raise RuntimeError('Environment state audit incomplete: '+checked.get('error','unknown reason'))
                if not checked['hard_clear']:
                    if len(rejections)<64:rejections.append(dict(kind='environment',q=np.asarray(q).tolist(),gap_m=gap,check=checked))
                    return False
            return True
        util,ompl_metadata=existing_ompl();util.RNG.setSeed(seed)
        report.update(ompl=ompl_metadata,setup_wall_s=time.perf_counter()-started,
            source_branch_q=endpoints.tolist(),gap_checks_m=gaps,speed_caps_m_or_rad_s=speed.tolist(),
            joint_bounds=kin.bounds.tolist(),edge_steps_m_or_rad=space.steps.tolist(),
            environment_policy=environment.metadata)
        if manual_policy is None:
            route=ompl_connect(endpoints[0],endpoints[1],space,valid,
                solve_seconds=max(.001,budget.remaining()),range_fraction=.1)
        else:
            proposed=manual_prefix(endpoints[0],endpoints[1],manual_policy)
            coarse_space=JointSpace(kin.bounds,[.01]+[np.deg2rad(6.)]*6)
            coarse=check_polyline(proposed,coarse_space,valid)
            write('manual_coarse_check.json',dict(check=coarse,path=proposed.tolist(),
                steps_m_or_rad=coarse_space.steps.tolist(),samples=budget.checks,
                coarse_only=True,complete_native_period_audit_required=True))
            report['manual_coarse_check']=coarse
            route=dict(status='exact_manual_candidate' if coarse['passed'] else 'manual_coarse_blocked',
                method='Existing_joint_space_manual_ordering',path=proposed if coarse['passed'] else None,
                impossible=None)
        report.update(route={k:v for k,v in route.items() if k!='path'},state_checks=budget.checks,
            collision_queries=environment.queries,rejected_state_examples=rejections)
        if route['path'] is None:
            report['status']=route['status']
        else:
            path=np.asarray(route['path'])
            write('joint_path.json',dict(path=path.tolist(),exact_endpoints=True))
            if manual_policy is not None:
                report['coarse_state_checks']=budget.checks
                # Independent fine verification is separate from bounded
                # proposal pruning; it still has its own finite budget.
                budget=CheckBudget(max_state_checks,120.)
                report['independent_edge_budget_seconds']=120.
            independent=check_polyline(path,space,valid)
            report['independent_edge_check']=independent
            if not independent['passed']:raise ValueError('Independent bounded edge recheck failed')
            revised=splice_connected_plan(plan,path,speed,hz=120)
            reconstructed=reconstruct_phase_tuples(revised)
            if [p[0] for p in reconstructed]!=[p['name'] for p in revised['phases']]:
                raise ValueError('Existing native phase representation reconstruction failed')
            prefix_duration=sum(p['duration_s'] for p in revised['phases'] if p['name'].startswith('prepare_ompl_'))
            report['prefix_duration_s']=prefix_duration
            if prefix_duration>max_prefix_seconds:raise LimitReached('trajectory_duration_budget')
            revised['ompl_prefix_connection']=dict(source_assembly=str(assembly),source_scene_sha256=manifest['scene_sha256'],
                source_plan_sha256=report['source_plan_sha256'],method=route.get('method'),
                exact_staging_q=endpoints[-1].tolist(),gap_checks_m=gaps,speed_caps_m_or_rad_s=speed.tolist(),
                only_initial_group=True,physics_executed=False,training_eligible=False)
            write('plan.json',revised)
            report['plan_sha256']=file_sha256(output/'plan.json')
            report['tuple_reconstruction']=dict(existing_six_field_phases=True,phase_count=len(reconstructed),
                phase_names=[p[0] for p in reconstructed],native_execution=False)
            report['belt_timing']=dict(source=plan.get('belt_cycle'),retimed=revised.get('belt_cycle'))
            self_audit=check_plan(moving,revised,hz=120);write('self_120hz.json',self_audit)
            report['self_120hz_sha256']=file_sha256(output/'self_120hz.json')
            report['full_self_audit']=dict(passed=self_audit['passed'],complete=self_audit['complete'],samples=self_audit['samples'])
            if self_audit['passed'] and self_audit['complete']:
                final_environment=RailEnvironmentCheck(stage,manifest,moving,allowed_truss_key=truss,
                    scope='preparation',max_distance_queries=max_collision_queries)
                preparation=screen_preparation_plan(final_environment,revised,manifest,hz=120)
                write('preparation_environment_120hz.json',preparation)
                report['preparation_environment_120hz_sha256']=file_sha256(output/'preparation_environment_120hz.json')
                report['preparation_environment_audit']=dict(passed=preparation['passed'],complete=preparation['complete'],status=preparation['status'])
                report['prefix_eligible']=bool(preparation.get('eligible_for_native_execution'))
                if report['prefix_eligible']:
                    entire_environment=RailEnvironmentCheck(stage,manifest,moving,allowed_truss_key=truss,
                        scope='preparation',max_distance_queries=max_collision_queries)
                    entire=full_environment_trace(entire_environment,revised)
                    write('full_environment_120hz.json',entire)
                    report['full_environment_120hz_sha256']=file_sha256(output/'full_environment_120hz.json')
                    report['full_environment_audit']={key:entire[key] for key in ('passed','complete','status')}
                    report['native_eligible']=bool(entire['passed'] and entire['complete'])
            report['status']='complete_static_prefix_candidate' if report['native_eligible'] else 'retimed_complete_plan_audit_blocked_or_unknown'
    except Exception as exc:
        report.update(status='budget_exhausted' if isinstance(exc,LimitReached) else 'connection_or_asset_error',
            error=type(exc).__name__+': '+str(exc))
    if 'budget' in locals():report['state_checks']=budget.checks
    if 'environment' in locals():report['collision_queries']=environment.queries
    if 'rejections' in locals():report['rejected_state_examples']=rejections
    if 'incomplete_states' in locals():report['incomplete_state_examples']=incomplete_states
    report['wall_s']=time.perf_counter()-started
    write('report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('assembly','source_plan','output'):p.add_argument(name,type=Path)
    p.add_argument('--seed',type=int,default=2026)
    p.add_argument('--solve-seconds',type=float,default=60.)
    p.add_argument('--max-state-checks',type=int,default=10000)
    p.add_argument('--max-collision-queries',type=int,default=200000)
    p.add_argument('--max-prefix-seconds',type=float,default=120.)
    p.add_argument('--manual-policy',choices=MANUAL_POLICIES)
    p.add_argument('--native-tool-cache',type=Path)
    a=p.parse_args()
    print(json.dumps(connect(a.assembly,a.source_plan,a.output,seed=a.seed,
        solve_seconds=a.solve_seconds,max_state_checks=a.max_state_checks,
        max_collision_queries=a.max_collision_queries,max_prefix_seconds=a.max_prefix_seconds,
        manual_policy=a.manual_policy,native_tool_cache=a.native_tool_cache),ensure_ascii=False,indent=2))
