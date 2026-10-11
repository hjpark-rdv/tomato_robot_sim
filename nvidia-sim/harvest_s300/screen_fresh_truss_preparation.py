"""Bounded fresh-campaign connector proposals using existing IK/FCL screens.

No asset edits, simulator execution, checkpoint restoration, or contact-policy
changes occur here. Full manipulation environment validity remains unproven;
this screen covers preparation through staging before the first approach.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from pxr import Usd

from compact_motion_assets import file_sha256,verify_compact_dependencies
from motion_kinematics import ToolKinematics,plan_motion
from motion_self_collision import MotionSelfCollision,check_plan
from truss_sequence import horizontal_truss_direction,select_group


INITIAL_STATE='authored normal source; no native checkpoint imported'


def proposal_variants():
    """Exactly eight existing connector choices; no adaptive regeneration."""
    return [dict(name='joint_seed_'+str(seed),horizontal_ready_connector='joint',
                 horizontal_ready_seed_index=seed,preparation_height_first=False)
            for seed in range(6)]+[
        dict(name='cartesian_height_first',horizontal_ready_connector='cartesian',
             horizontal_ready_seed_index=None,preparation_height_first=True),
        dict(name='cartesian_direct',horizontal_ready_connector='cartesian',
             horizontal_ready_seed_index=None,preparation_height_first=False)]


def turn_followup_variants():
    """Six explicit proposals after the original eight failed, not a seed hunt."""
    result=[]
    for normalized,lift in ((False,-.135),(True,-.135),(True,-.170)):
        for seed in (6,7):
            result.append(dict(name='joint_seed_'+str(seed)+('_canonical'if normalized else '_raw')+
                '_lift'+str(round(-lift*1000)),horizontal_ready_connector='joint',
                horizontal_ready_seed_index=seed,preparation_height_first=False,
                horizontal_ready_normalize_turns=normalized,ready_lift_offset_m=lift))
    return result


def require_completed_base_screen(path,source_sha):
    base=json.loads(Path(path).read_text())
    proposals=base.get('proposals',[])
    if (base.get('schema')!='s300-bounded-fresh-preparation-screen-v1'
            or base.get('native_runs')!=0 or base.get('checkpoint_imported') is not False
            or base.get('source_unchanged') is not True or base.get('source_sha256_after')!=source_sha
            or base.get('nominal_prefix_pass_count')!=0 or len(proposals)!=8
            or [p.get('name')for p in proposals]!=[v['name']for v in proposal_variants()]
            or any(p.get('status') in (None,'proposal_not_evaluated')for p in proposals)):
        raise ValueError('Turn follow-up requires all original eight completed failures on the unchanged fresh source')
    return dict(source_summary=str(Path(path).resolve()),sha256=file_sha256(path),
        all_original_eight_completed=True,original_nominal_prefix_pass_count=0,
        scope='Additional explicitly bounded six; original failures retained and not reclassified')


def require_fresh_campaign(manifest,truss):
    marker=manifest.get('fresh_campaign') or {}
    if (marker.get('schema')!='s300-independent-fresh-near-campaign-v1'
            or marker.get('enabled') is not True or marker.get('initial_state')!=INITIAL_STATE
            or marker.get('truss_key')!=truss or marker.get('inherited_native_breaks')!=[]
            or marker.get('accepted_previous_route_history') is not False
            or not marker.get('source_assembly') or not marker.get('source_scene_sha256')):
        raise ValueError('An explicit independent fresh campaign is required; no old checkpoint rollback')
    if manifest.get('route_checkpoint') or manifest.get('eligible_attachment_transfer'):
        raise ValueError('Fresh proposal must not import an earlier route or attachment-transfer checkpoint')
    if manifest['target']['name']!=marker.get('initial_target_name') or not manifest['target']['name'].startswith(truss+'_Fruit_'):
        raise ValueError('Fresh campaign must explicitly target the requested first truss')
    start=np.asarray(manifest.get('initial_robot_q'),float)
    if start.shape!=(7,) or not np.isfinite(start).all():
        raise ValueError('Fresh authored normal lift/arm start q[7] is required')
    return marker,start


def write_new(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write('\n')


def screen(assembly,output,*,truss='Truss_01',ready_lift_offset_m=-.135,
           grasp_height_offset_m=-.0015,grasp_depth_m=.065,max_proposals=8,turn_followup_after=None):
    from rail_environment_preflight import audit_rail_transit
    if isinstance(max_proposals,bool) or not isinstance(max_proposals,int) or not 1<=max_proposals<=8:
        raise ValueError('Fresh connector budget must be an integer within1..8')
    assembly=Path(assembly).resolve();output=Path(output).resolve()
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    marker,start=require_fresh_campaign(manifest,truss)
    source_paths=[assembly/'scene.usda',assembly/'motion_manifest.json',assembly/'skin.npz']
    dependency=verify_compact_dependencies(assembly,manifest)
    if dependency is not None:source_paths.append(assembly/dependency['relative_path'])
    before={str(p):file_sha256(p) for p in source_paths}
    if before[str(assembly/'scene.usda')]!=manifest['scene_sha256']:
        raise ValueError('Fresh source scene SHA mismatch')
    base_proof=None
    if turn_followup_after is not None:
        if max_proposals not in (6,8):raise ValueError('Turn follow-up is exactly six predeclared proposals')
        base_proof=require_completed_base_screen(turn_followup_after,before)
        variants=turn_followup_variants()
    else:variants=proposal_variants()[:max_proposals]
    stage=Usd.Stage.Open(str(assembly/(dependency['relative_path'] if dependency else 'scene.usda')))
    kin=ToolKinematics(stage);checker=MotionSelfCollision(stage,kin,clearance_m=.002)
    fruits=[f for f in manifest['plant']['fruits'] if f['name'].startswith(truss+'_Fruit_')]
    paths=[f['path'] for f in fruits];poses=np.asarray([f['center'] for f in fruits],float)
    group=select_group(fruits,[],paths,poses,{},max_attempts=2,order='highest-first')
    if group is None:raise ValueError('Fresh requested truss has no target group')
    hint=np.mean(poses,axis=0)-kin.world[:3,3];hint[2]=0.
    axis=horizontal_truss_direction(fruits,[],paths,poses,hint,fallback_direction=hint,min_attached_fruits=5)
    axis['truss_key']=truss
    reference=np.asarray(manifest.get('grasp_reference',manifest['target'])['center'],float)
    center=np.asarray(group['measured_centroid_world_m'],float)
    center[2]+=manifest.get('grasp_reference',{}).get('height_offset_m',0.)
    parameters=dict(preparation='horizontal_ready',pitch_deg=0.,yaw_offset_deg=0.,
        harvest_profile='belt_open_close_cycle',belt_speed_m_s=.05,belt_seconds=2.,close_seconds=6.,
        belt_ramp_seconds=.5,belt_infeed_speed_m_s=0.,grasp_gap_m=.028,
        grasp_height_offset_m=grasp_height_offset_m,grasp_depth_m=grasp_depth_m,
        grasp_translation_world_m=(center-reference).tolist(),preparation_gap_m=.044,
        ready_lift_offset_m=ready_lift_offset_m,max_grasp_translation_m=1.5,
        allow_measured_ready_start=True,approach_direction_world=axis['direction_world'])
    output.mkdir(parents=True,exist_ok=False)
    source_code_paths=[Path(__file__),Path(__file__).with_name('motion_kinematics.py'),
        Path(__file__).with_name('motion_self_collision.py'),Path(__file__).with_name('rail_environment_preflight.py')]
    source_code_sha={p.name:file_sha256(p) for p in source_code_paths}
    context=dict(schema='s300-bounded-fresh-preparation-screen-v1',source_assembly=str(assembly),
        source_sha256_before=before,source_code_sha256=source_code_sha,fresh_campaign=marker,
        authored_normal_start_q=start.tolist(),target_group=group,axis=axis,parameters=parameters,
        protocol=variants,max_proposals=len(variants),turn_followup_base=base_proof,
        native_runs=0,checkpoint_imported=False,previous_accepted_history_changed=False,
        environment_scope='all phases before first approach; frozen exact fresh source geometry',
        environment_query_budget_per_screen=2000000,
        active_physics_or_geometry_changed=False,training_eligible=False,harvest_success=None)
    write_new(output/'inputs.json',context)
    results=[];started=time.perf_counter()
    for variant in variants:
        folder=output/variant['name'];folder.mkdir()
        row=dict(**variant,native_executed=False,manipulation_environment_certified=False,
                 nominal_prefix_screen_passed=False,status='proposal_not_evaluated')
        tick=time.perf_counter()
        try:
            options={k:v for k,v in variant.items() if k!='name'}
            case_parameters=dict(parameters);case_parameters.update(options)
            phases,plan=plan_motion(kin,reference,start=start,**case_parameters)
            plan['truss_axis']=axis;plan['fresh_campaign']=marker
            write_new(folder/'plan.json',plan);row['ik']='complete'
            for hz in (8,120):
                self_report=check_plan(checker,plan,hz=hz)
                write_new(folder/('self_'+str(hz)+'hz.json'),self_report)
                row['self_'+str(hz)+'hz']=dict(passed=self_report['passed'],complete=self_report['complete'],
                    samples=self_report['samples'],first_blocked=next((p for p in self_report['phases']if not p['clear']),None))
                if not self_report['passed'] or not self_report['complete']:
                    row['status']='self_screen_blocked_'+str(hz)+'hz';break
                env=audit_rail_transit(assembly,folder/'plan.json',checkpoint=None,
                    output=folder/('preparation_environment_'+str(hz)+'hz.json'),hz=hz,
                    scope='preparation',allowed_truss_key=truss,max_distance_queries=2000000)
                row['environment_'+str(hz)+'hz']=dict(passed=env.get('passed'),complete=env.get('complete'),
                    status=env.get('status'),samples=env.get('samples'),
                    hard_intersection_sample_pairs=env.get('hard_intersection_sample_pairs'),
                    hard_hits=env.get('hard_hits'),diagnostic_intersection_sample_pairs=env.get('diagnostic_intersection_sample_pairs'))
                if env.get('complete') is not True or env.get('passed') is not True:
                    row['status']='environment_screen_blocked_or_unknown_'+str(hz)+'hz';break
            else:
                row.update(status='complete_nominal_self_and_preparation_screen_pass',
                    nominal_prefix_screen_passed=True,native_execution_screen_eligible=env.get('eligible_for_native_execution') is True,
                    caveat='Native fresh initializer, tracking/contact and manipulation validity remain required')
        except (RuntimeError,ValueError) as error:
            row.update(status='proposal_or_screen_error',error=type(error).__name__+': '+str(error))
        row['elapsed_s']=time.perf_counter()-tick;results.append(row)
        write_new(folder/'proposal_result.json',row)
        print(json.dumps(dict(proposal=variant['name'],status=row['status'],elapsed_s=row['elapsed_s'])),flush=True)
    after={str(p):file_sha256(p) for p in source_paths}
    if after!=before:raise ValueError('Fresh source changed during bounded proposal screening')
    current_code_sha={p.name:file_sha256(p) for p in source_code_paths}
    report=dict(**context,source_sha256_after=after,source_unchanged=True,
        source_code_sha256_after=current_code_sha,source_code_unchanged=current_code_sha==source_code_sha,
        elapsed_s=time.perf_counter()-started,proposals=results,
        nominal_prefix_pass_count=sum(row['nominal_prefix_screen_passed']for row in results),
        execution_screen_eligible_count=sum(row.get('native_execution_screen_eligible',False)for row in results),
        finite_search_failure_is_impossible=False)
    write_new(output/'summary.json',report)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--truss',default='Truss_01')
    parser.add_argument('--ready-lift-offset-mm',type=float,default=-135.)
    parser.add_argument('--grasp-height-offset-mm',type=float,default=-1.5)
    parser.add_argument('--grasp-depth-mm',type=float,default=65.)
    parser.add_argument('--max-proposals',type=int,default=8)
    parser.add_argument('--turn-followup-after',type=Path,
                        help='Immutable original eight-failure summary; enables only the six predeclared 6/7 raw/canonical proposals')
    args=parser.parse_args();report=screen(args.assembly,args.output,truss=args.truss,
        ready_lift_offset_m=args.ready_lift_offset_mm/1000.,grasp_height_offset_m=args.grasp_height_offset_mm/1000.,
        grasp_depth_m=args.grasp_depth_mm/1000.,max_proposals=args.max_proposals,turn_followup_after=args.turn_followup_after)
    print(json.dumps({key:report[key]for key in ('nominal_prefix_pass_count','execution_screen_eligible_count','elapsed_s')},indent=2))
