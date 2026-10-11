"""Bounded SIM-GT station/arm screening using the existing harvest planner.

Reuses full composed CAD, seven-axis FK/IK, truss grouping and FCL self audit.
It performs no physics, no plant rebuilding and no environment-clearance claim.
Every side-selected truss gets at most two station queries; rejected targets
remain in the result. Coarse samples never become a full-clear certificate.
"""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import re
import time

import numpy as np
from pxr import Usd

from motion_kinematics import ToolKinematics, plan_motion
from motion_self_collision import MotionSelfCollision, check_plan
from rail_motion import rail_destination, station_kinematics, attach_rail_commands
from truss_sequence import select_group, horizontal_truss_direction
from compact_motion_assets import verify_compact_dependencies

_CONTEXT = None


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def station_candidates(row, limits_world_y_m=(0.,5.)):
    """Two explicitly bounded stations; clamping is recorded, never extrapolated."""
    center=np.asarray(row['fruit_centroid_world_m'],dtype=float)
    limits=np.asarray(limits_world_y_m,dtype=float)
    if center.shape!=(3,) or limits.shape!=(2,) or not np.isfinite(np.r_[center,limits]).all() or limits[0]>=limits[1]:
        raise ValueError('Finite actual centroid and increasing rail limits required')
    proposals=[float(center[1]+.066),float(center[1]-.114)]
    rows=[]
    for value in proposals:
        station=float(np.clip(value,*limits))
        if any(abs(station-r['station_y_m'])<1e-9 for r in rows):continue
        rows.append(dict(station_y_m=station,unclamped_station_y_m=value,clamped=value!=station))
    return rows


def validate_preparation_variant(variant):
    """Only existing horizontal-ready controls, never a contact-policy override."""
    if not isinstance(variant,dict) or set(variant)-{
            'name','horizontal_ready_connector','horizontal_ready_seed_index'}:
        raise ValueError('Preparation variant contains unsupported controls')
    name=variant.get('name');connector=variant.get('horizontal_ready_connector')
    seed=variant.get('horizontal_ready_seed_index')
    if not isinstance(name,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',name):
        raise ValueError('Preparation variant needs a unique simple name')
    if connector not in ('cartesian','joint'):
        raise ValueError('Existing cartesian or joint ready connector required')
    if seed is not None and (isinstance(seed,bool) or not isinstance(seed,int) or
                            not 0<=seed<=8 or connector!='joint'):
        raise ValueError('Explicit joint staging seed must be an integer0..8')
    result=dict(name=name,horizontal_ready_connector=connector)
    if seed is not None:result['horizontal_ready_seed_index']=seed
    return result


def validate_preparation_variants(variants):
    if not isinstance(variants,list) or not 1<=len(variants)<=9:
        raise ValueError('Finite preparation variants list must contain1..9 entries')
    result=[validate_preparation_variant(item) for item in variants]
    if len({item['name'] for item in result})!=len(result):
        raise ValueError('Preparation variant names must be unique')
    controls=[(item['horizontal_ready_connector'],item.get('horizontal_ready_seed_index')) for item in result]
    if len(set(controls))!=len(controls):
        raise ValueError('Renamed identical preparation controls are not distinct variants')
    return result


def screen_query_proposals(row,limits_world_y_m=(0.,5.),*,variants=None,station_index=None):
    """Opt-in preparation branches at existing stations; default proposals exact."""
    proposals=station_candidates(row,limits_world_y_m)
    if station_index is not None and (isinstance(station_index,bool) or
            not isinstance(station_index,int) or station_index not in (0,1)):
        raise ValueError('Existing station index must be0or1')
    if variants is None:
        if station_index is None:return proposals
        return proposals[station_index:station_index+1]
    validated=validate_preparation_variants(variants)
    stations=list(enumerate(proposals))
    if station_index is not None:stations=stations[station_index:station_index+1]
    return [dict(proposal,station_proposal_index=index,preparation_variant=dict(variant))
            for index,proposal in stations for variant in validated]


def preparation_variant_arguments(variant):
    item=validate_preparation_variant(variant)
    arguments=['--horizontal-ready-connector',item['horizontal_ready_connector']]
    if 'horizontal_ready_seed_index' in item:
        arguments+=['--horizontal-ready-seed-index',str(item['horizontal_ready_seed_index'])]
    return arguments


def screen_selection(inventory,*,eligible_only=False):
    """Exact placement policy; excluded and finite-unknown cases remain distinct."""
    selected=[];excluded=[]
    for row in inventory['trusses']:
        root=np.asarray(row['root_position_world_m'],float)
        center=np.asarray(row['fruit_centroid_world_m'],float)
        gutter=float(row['gutter_center_x']);side=float(np.sign(-gutter))
        if root.shape!=(3,) or center.shape!=(3,) or not np.isfinite(np.r_[root,center,gutter]).all() or side==0:
            raise ValueError('Malformed exact farm placement inventory')
        near=bool((root[0]-gutter)*side>0)
        if bool(row['rail_near'])!=near:raise ValueError('Root/gutter and inventory near-side policy disagree')
        reason='opposite_side_stem' if not near else (
            'fruit_centroid_behind_gutter' if eligible_only and (center[0]-gutter)*side<=0 else None)
        if reason is None:selected.append(row)
        else:excluded.append(dict(row,selection_exclusion_reason=reason,reachability='not inferred from placement exclusion'))
    return selected,excluded


def initial_group(row):
    centers=[];fruits=[];paths=[]
    for name,center in sorted(row['fruits'].items()):
        # These are grouping labels from exact mesh centers, not fabricated
        # rigid-body/joint identities. Physics mapping is deferred to assembly.
        label=f"Plant_{row['plant_id']:02d}/Truss_{row['truss_index']:02d}/{name}"
        fruits.append(dict(name=name,path=label,joint=label+'__grouping_label'))
        paths.append(label);centers.append(center)
    positions=np.asarray(centers,dtype=float)
    group=select_group(fruits,[],paths,positions,{},order='highest-first')
    if group is None:raise ValueError('No actual fruit group')
    return fruits,paths,positions,group


def _initialize(assembly,output,full_hz,measured_ready_start=False,
                preparation_variants=None,station_index=None):
    global _CONTEXT
    started=time.perf_counter();assembly=Path(assembly)
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    verify_compact_dependencies(assembly,manifest)
    stage=Usd.Stage.Open(str(assembly/'scene.usda'))
    kin=ToolKinematics(stage)
    checker=MotionSelfCollision(stage,kin,clearance_m=.002)
    _CONTEXT=dict(assembly=assembly,output=Path(output),full_hz=full_hz,manifest=manifest,stage=stage,
                  kin=kin,checker=checker,measured_ready_start=bool(measured_ready_start),
                  preparation_variants=preparation_variants,station_index=station_index,
                  initialization_wall_s=time.perf_counter()-started)


def _screen(row):
    ctx=_CONTEXT;manifest=ctx['manifest'];kin=ctx['kin'];checker=ctx['checker']
    identity=f"plant_{row['plant_id']:02d}_truss_{row['truss_index']:02d}"
    output=ctx['output']/identity;output.mkdir(exist_ok=False)
    (output/'target.json').write_text(json.dumps(row,indent=2,allow_nan=False)+'\n')
    fruits,paths,positions,group=initial_group(row)
    reference=np.asarray(manifest['grasp_reference']['center'],float)
    selected=np.asarray(group['measured_centroid_world_m'],float)
    selected[2]+=manifest['grasp_reference'].get('height_offset_m',0.)
    start=np.asarray(manifest['initial_robot_q'],float)
    results=[];begin=time.perf_counter()
    for index,proposal in enumerate(screen_query_proposals(row,manifest['rail']['limits_world_y_m'],
            variants=ctx['preparation_variants'],station_index=ctx['station_index'])):
        query=output/f'query_{index:02d}';query.mkdir()
        record=dict(index=index,**proposal,group=group,status='planning',
                    environment_audit='not performed',physics_execution=False)
        tick=time.perf_counter();queries=checker.queries
        try:
            destination=rail_destination(manifest,proposal['station_y_m'])
            station_kin=station_kinematics(kin,destination)
            axis=horizontal_truss_direction(fruits,[],paths,positions,
                selected-station_kin.world[:3,3])
            local_reference=reference+np.asarray(destination['axis_world'])*destination['displacement_m']
            translation=selected-local_reference
            variant=proposal.get('preparation_variant')
            variant_controls={} if variant is None else {
                key:value for key,value in variant.items() if key!='name'}
            phases,plan=plan_motion(station_kin,local_reference,start=start,
                preparation='horizontal_ready',harvest_profile='belt_open_close_cycle',
                preparation_gap_m=.044,ready_lift_offset_m=-.15,preparation_height_first=True,
                belt_speed_m_s=.05,belt_seconds=2.,close_seconds=6.,belt_ramp_seconds=.5,
                grasp_height_offset_m=-.010,grasp_depth_m=.065,grasp_gap_m=.028,
                grasp_translation_world_m=translation,max_grasp_translation_m=1.5,
                allow_measured_ready_start=ctx['measured_ready_start'],
                approach_direction_world=axis['direction_world'],**variant_controls)
            phases,plan=attach_rail_commands(phases,plan,start,destination,move_first=True)
            plan.update(source_scene_sha256=manifest['scene_sha256'],
                        source_manifest_sha256=sha(ctx['assembly']/'motion_manifest.json'),
                        source_materialized_dependency=manifest.get('compact_assets',{}).get('materialized_dependency'),
                        target_identity=identity,
                        target_group=group,truss_axis=axis,training_eligible=False,
                        purpose='SIM-GT finite station/IK/self screening; actual environment/physics pending')
            (query/'plan.json').write_text(json.dumps(plan,indent=2,allow_nan=False)+'\n')
            record.update(status='ik_path_found',plan=str((query/'plan.json').relative_to(ctx['output'])),
                          truss_yaw_world_deg=axis['yaw_world_deg'])
            if variant is not None:
                plan['preparation_variant']=dict(variant)
                endpoints=[]
                for label,q,gap in [('staging_open',plan['prep_q'],.096),
                                    ('grasp_open',plan['goal_q'],.096),
                                    ('grasp_closed',plan['goal_q'],.028)]:
                    hits=checker.check(q,gap,rail_position_m=destination['displacement_m'])
                    endpoints.append(dict(pose=label,clear=not hits,hits=hits,
                        scope='Robot/tool self-collision only; static pose is not an approach certificate'))
                record['endpoint_self_audit']=endpoints
                record['selected_staging_seed_index']=plan['selected_seed_index']
                record['staging_q']=plan['prep_q']
                (query/'plan.json').write_text(json.dumps(plan,indent=2,allow_nan=False)+'\n')
            coarse=check_plan(checker,plan,hz=10)
            (query/'self_coarse.json').write_text(json.dumps(coarse,indent=2,allow_nan=False)+'\n')
            if not coarse['passed'] or not coarse['complete']:
                record.update(status='self_path_blocked_coarse',first_block=next(r for r in coarse['phases'] if not r['clear']))
            else:
                full=check_plan(checker,plan,hz=ctx['full_hz'])
                (query/'self_full.json').write_text(json.dumps(full,indent=2,allow_nan=False)+'\n')
                record.update(status='self_path_clear_full' if full['passed'] and full['complete'] else 'self_path_blocked_full',
                              full_self_audit_complete=full['complete'],full_self_audit_passed=full['passed'])
                if not full['passed']:record['first_block']=next(r for r in full['phases'] if not r['clear'])
        except ValueError as error:
            record.update(status='plan_input_or_geometry_rejected',error=str(error))
        except RuntimeError as error:
            record.update(status='ik_path_not_found_finite' if record['status']=='planning' else 'self_audit_integration_error',error=str(error))
        record.update(wall_s=time.perf_counter()-tick,fcl_sample_queries=checker.queries-queries)
        (query/'result.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
        results.append(record)
        # Preserve both designated station proposals even when the first clears.
    import resource
    result=dict(identity=identity,plant_id=row['plant_id'],truss_index=row['truss_index'],
        side_selected=True,queries=results,query_count=len(results),wall_s=time.perf_counter()-begin,
        worker_initialization_s=ctx['initialization_wall_s'],worker_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        self_clear_candidates=sum(r['status']=='self_path_clear_full' for r in results),
        environment_audit='not performed; full static/dynamic farm remains in source asset',
        physics_execution=False,reachable_success=None,harvest_success=None,training_eligible=False)
    (output/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('assembly',type=Path);p.add_argument('inventory',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--workers',type=int,default=4);p.add_argument('--full-hz',type=int,default=60)
    p.add_argument('--max-trusses',type=int)
    p.add_argument('--measured-ready-start',action='store_true',help='Use existing measured-ready connector on actual current robot geometry')
    p.add_argument('--eligible-only',action='store_true',help='Exclude opposite stems and trusses whose centroid lies behind their gutter')
    p.add_argument('--preparation-variants',type=Path,help='Opt-in JSON list of existing connector/seed controls; every named variant is retained')
    p.add_argument('--station-index',type=int,choices=[0,1],help='Limit each variant to this existing station proposal; clamped duplicate station remains unevaluated')
    args=p.parse_args()
    if not 1<=args.workers<=8 or args.full_hz<10:p.error('Workers1..8 and full sampling>=10Hz required')
    manifest=json.loads((args.assembly/'motion_manifest.json').read_text())
    if sha(args.assembly/'scene.usda')!=manifest['scene_sha256'] or not manifest.get('rail'):
        raise ValueError('Source hash mismatch or missing physical rail asset')
    dependency=verify_compact_dependencies(args.assembly,manifest)
    inventory=json.loads(args.inventory.read_text())
    variants=(validate_preparation_variants(json.loads(args.preparation_variants.read_text()))
              if args.preparation_variants is not None else None)
    selected,side_excluded=screen_selection(inventory,eligible_only=args.eligible_only)
    omitted=[]
    if args.max_trusses is not None:
        if args.max_trusses<=0:p.error('Positive max-trusses required')
        omitted=selected[args.max_trusses:];selected=selected[:args.max_trusses]
    args.output.mkdir(parents=True,exist_ok=False)
    began=time.perf_counter();results=[]
    with ProcessPoolExecutor(max_workers=args.workers,initializer=_initialize,
                             initargs=(args.assembly,args.output,args.full_hz,args.measured_ready_start,variants,args.station_index)) as pool:
        futures={pool.submit(_screen,row):row for row in selected}
        for future in as_completed(futures):
            row=futures[future]
            try:result=future.result()
            except Exception as error:
                result=dict(plant_id=row['plant_id'],truss_index=row['truss_index'],status='integration_error',error=repr(error),queries=[])
            results.append(result)
            print('[ROW SCREEN]',len(results),'/',len(selected),row['plant_id'],row['truss_index'],
                  [r['status'] for r in result['queries']],flush=True)
            (args.output/'progress.json').write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
    results.sort(key=lambda r:(r['plant_id'],r['truss_index']))
    report=dict(schema='s300-row-station-finite-screen-v1',source_assembly=str(args.assembly.resolve()),
        source_scene_sha256=manifest['scene_sha256'],source_manifest_sha256=sha(args.assembly/'motion_manifest.json'),
        source_materialized_dependency=dependency,
        inventory=str(args.inventory.resolve()),inventory_sha256=sha(args.inventory),
        workers=args.workers,coarse_hz=10,full_hz=args.full_hz,
        planned_query_budget=(2*len(selected) if variants is None and args.station_index is None else
            sum(len(screen_query_proposals(row,manifest['rail']['limits_world_y_m'],
                variants=variants,station_index=args.station_index)) for row in selected)),
        total_queries=sum(len(r['queries']) for r in results),wall_s=time.perf_counter()-began,
        status_counts=dict(Counter(q['status'] for r in results for q in r['queries'])),
        measured_ready_start=args.measured_ready_start,eligible_only=args.eligible_only,
        side_excluded=side_excluded,budget_unselected=omitted,
        results=results,scope='IK/full-arm self collision only; environment, active-plant mapping, rail tracking and actual harvest pending',
        training_eligible=False,harvest_success=None)
    if variants is not None or args.station_index is not None:
        report.update(preparation_variants=variants,station_proposal_index=args.station_index,
            preparation_variants_source=(str(args.preparation_variants.resolve()) if args.preparation_variants else None),
            preparation_variants_source_sha256=(sha(args.preparation_variants) if args.preparation_variants else None))
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ['total_queries','status_counts','wall_s']},indent=2),flush=True)


if __name__=='__main__':main()
