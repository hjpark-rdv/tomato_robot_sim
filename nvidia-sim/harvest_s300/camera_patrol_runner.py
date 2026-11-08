"""Connect visible-camera patrol decisions to the existing native executor.

No physics, renderer, IK or success criterion is implemented here.  Every
accepted segment uses benchmark_motion/run_motion and a validated terminal
checkpoint.  Independent diagnostics and invalid physical stops never become
an accepted continuation state.
"""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from camera_patrol import CameraPatrol
from camera_patrol_startup_cache import StartupPreflightCache,startup_identity
from compact_motion_assets import verify_compact_dependencies,file_sha256
from native_route_checkpoint import load_checkpoint
from grasp_depth_schedule import completed_native_cycles
from route_checkpoint import switch_window
from run_row_harvest import (camera_campaign_arguments,completed_final_checkpoint,
    validated_initial_checkpoint,measured_harvested_trusses,unexecuted_planning_failure,
    startup_preflight_arguments,preflight_result,target_truss_camera_scope)


def _read(path):return json.loads(Path(path).read_text())


def _write(path,value):
    path=Path(path);pending=path.with_name(path.name+'.pending')
    pending.write_text(json.dumps(value,indent=2,allow_nan=False,ensure_ascii=False)+'\n')
    pending.replace(path)  # The replay publisher must never read half a ledger.


def patrol_motion_arguments(base,inventory_path,*,decision,checkpoint=None,new_window=False,target_truss_windows=False,
                            resume_target_truss_revision=False):
    """Copy existing controls; override only patrol-owned routing switches."""
    if type(target_truss_windows)is not bool:raise ValueError('Explicit boolean camera target-truss window mode required')
    if type(resume_target_truss_revision)is not bool:raise ValueError('Explicit boolean measured target revision required')
    if resume_target_truss_revision and (not target_truss_windows or checkpoint is None or new_window or decision['action']!='scan'):
        raise ValueError('Same-plant target revision must restore the exact checkpoint into a fresh target-only camera scan')
    result=camera_campaign_arguments(base)
    if '--camera-target-truss-window' in result and not target_truss_windows:
        raise ValueError('Exact collision-enabled camera scope requires the explicit patrol window opt-in')
    if '--allow-resume-target-truss-revision' in result:
        raise ValueError('Target-truss revision authorization belongs to one exact dispatcher checkpoint transition')
    required={'--headless','--camera-harvest','--capture-gt','--whole-truss',
        '--native-materialized-stage','--verify-full-environment-preflight',
        '--verify-harvest-contact-after-step','--save-route-checkpoint','--checkpoint-cycle-states'}
    if not required.issubset(result) or '--physics-only' in result:
        raise ValueError('Camera patrol requires rendered headless GT, full native audits and terminal checkpoints')
    values={'--near-trusses-metadata','--rail-to-y','--resume-native-checkpoint',
        '--camera-target-truss','--camera-max-harvest-cycles','--seconds','--camera-scan-lift-height-m'}
    flags={'--camera-scan-only','--allow-resume-rail-travel','--initial-truss-first','--retain-lift-drive-reference',
        '--camera-target-truss-window'}
    retained=[];i=0
    while i<len(result):
        value=result[i]
        if value in values:
            if i+1>=len(result) or result[i+1].startswith('--'):raise ValueError('Missing source value for '+value)
            i+=2;continue
        if value in flags:i+=1;continue
        if any(value.startswith(name+'=') for name in values|flags):
            raise ValueError('Use separate option/value tokens for patrol-controlled routing')
        retained.append(value);i+=1
    retained+=['--near-trusses-metadata',str(Path(inventory_path).resolve()),'--seconds','0']
    if target_truss_windows:retained.append('--camera-target-truss-window')
    if resume_target_truss_revision:retained.append('--allow-resume-target-truss-revision')
    action=decision['action']
    if action not in ('scan','move','harvest','lift_scan'):raise ValueError('Native patrol dispatch requires scan, move, lift_scan or harvest')
    if action=='harvest':
        retained+=['--camera-target-truss',decision['target']['truss_key'],'--camera-max-harvest-cycles','1']
    else:retained.append('--camera-scan-only')
    if action=='lift_scan':
        height=decision.get('next_lift_m')
        if isinstance(height,bool) or not isinstance(height,(int,float)) or not np.isfinite(height):
            raise ValueError('Lift scan requires an explicit finite absolute height')
        retained+=['--camera-scan-lift-height-m',str(float(height))]
    if checkpoint is not None and not new_window:
        retained+=['--resume-native-checkpoint',str(Path(checkpoint).resolve()),'--retain-lift-drive-reference']
    elif new_window:
        retained.append('--retain-lift-drive-reference')
    if action=='move':
        distance=float(decision['next_station_y_m'])-float(decision['current_station_y_m'])
        if not 0<distance<=.20+1e-9:raise ValueError('Patrol dispatcher requires forward world-Y rail moves <=0.20m')
        retained+=['--rail-to-y',str(float(decision['next_station_y_m']))]
        if checkpoint is not None and not new_window:retained.append('--allow-resume-rail-travel')
    return retained




def latest_camera_observation(trial,summary):
    """Read actual measured exposure metadata, never synthesize camera views."""
    trial=Path(trial);rows=[json.loads(line) for line in (trial/'rgbd/frames.jsonl').read_text().splitlines() if line.strip()]
    if not rows:raise ValueError('Completed native patrol segment has no actual camera exposure')
    row=rows[-1]
    if 'gt' not in row or row.get('render_did_not_advance_physics') is not True:
        raise ValueError('Measured synchronized RGB-D GT exposure required')
    directory=trial/'rgbd'/Path(row['rgb']).parent
    objects_path=(directory/row['gt']['objects']).resolve()
    if not objects_path.is_relative_to((trial/'rgbd').resolve()):raise ValueError('Camera objects path escapes actual RGB-D output')
    stamp=float(row['timestamp_s']);end=float(summary['simulated_s'])
    if not np.isfinite([stamp,end]).all() or stamp>end+1e-6:raise ValueError('Actual exposure clock exceeds terminal physics clock')
    return dict(objects_path=str(objects_path),world_from_camera_optical=row['world_from_camera_optical'],
        observation_id=str(directory.resolve()),timestamp_s=stamp,current_timestamp_s=end,
        physics_step=row.get('physics_step'),pose_source='Measured mounted native body and synchronized RGB-D exposure',
        synchronized=True,valid=True)


def exact_native_detached_keys(manifest,metadata):
    """Translate only this checkpoint's exact native joint identities."""
    broken={row['joint'] for row in metadata.get('break_events',[]) if isinstance(row,dict) and 'joint' in row}
    source=manifest['plant']['source_plant'];result=[]
    for fruit in manifest['plant']['fruits']:
        if fruit['joint'] in broken:
            truss,number=fruit['name'].split('_Fruit_')
            result.append(source+'/'+truss+'/Fruit_'+number)
    return sorted(result)


def _finite_completed_camera_rejection(record,trial,model_hash):
    """Recognize existing finite IK or hash-bound collision/search evidence."""
    folder=Path(trial)/'cycles'/f"{record['index']:03d}"
    error=record.get('error','')
    prefixes=('Fixed entry preparation IK not found','Explicit staging IK seed not feasible:',
        'Measured ready-to-staging connector IK not found','Fixed entry approach IK not found',
        'Planned joint limits exceeded in ','Level preparation FK orientation check failed:')
    if isinstance(error,str) and error.startswith(prefixes):
        return dict(kind='finite_existing_ik_or_waypoint_rejection',error=error)
    def positive(path,plan):
        if not path.is_file() or not plan.is_file():return None
        report=_read(path);sha=file_sha256(plan)
        if report.get('passed')is not False or report.get('error'):return None
        bound=(report.get('input_sha256')==sha and report.get('scene_sha256')==model_hash)
        hit=any(row.get('hits') for row in report.get('phases',[]))
        hashes=report.get('source_sha256',{})
        if ((bound and hit) or (report.get('status')=='blocked' and report.get('hard_hits') and
                hashes.get('plan')==sha and hashes.get('scene')==model_hash)):
            return dict(kind='positive_geometric_blocking_witness',path=str(path),sha256=file_sha256(path),
                plan_sha256=sha,complete=report.get('complete'),unexamined_suffix_clear=False)
        return None
    for name in ('self_collision_preflight.json','full_environment_preflight.json'):
        proof=positive(folder/name,folder/'plan.json')
        if proof:return proof
    path=folder/'prefix_connection/report.json';raw=folder/'plan_before_prefix_connection.json'
    if not path.is_file() or not raw.is_file():return None
    report=_read(path)
    known_wall_budget=(report.get('status')=='finite_budget_or_connection_error' and
        report.get('error')=='LimitReached: wall_time_budget')
    if (report.get('schema')!='s300-resumed-camera-prefix-connection-v1' or
            report.get('source_files_unchanged')is not True or report.get('physics_steps')!=0 or
            (report.get('error') and not known_wall_budget) or Path(report.get('source_plan','')).resolve()!=raw.resolve() or
            report.get('source_file_sha256',{}).get(str(raw.resolve()))!=file_sha256(raw)):
        return None
    if known_wall_budget:
        budget=report.get('proposal_budget',{});checks=report.get('state_checks')
        seconds=budget.get('seconds');maximum=budget.get('max_state_checks')
        scene=(Path(report.get('source_assembly',''))/'scene.usda').resolve()
        hashes=report.get('source_file_sha256',{})
        if (not isinstance(model_hash,str) or len(model_hash)!=64 or hashes.get(str(scene))!=model_hash or
                isinstance(seconds,bool) or not isinstance(seconds,(int,float)) or not np.isfinite(seconds) or seconds<=0 or
                isinstance(checks,bool) or not isinstance(checks,int) or checks<1 or
                isinstance(maximum,bool) or not isinstance(maximum,int) or not checks<=maximum or
                report.get('verified_plan')is not None or report.get('eligible_for_native_execution')is not False):
            return None
        return dict(kind='finite_existing_connector_wall_time_budget_exhausted',path=str(path),sha256=file_sha256(path),
            source_plan_sha256=file_sha256(raw),source_scene_sha256=model_hash,
            exact_error=report['error'],state_checks=checks,proposal_budget=copy.deepcopy(budget),
            reachability='unknown; finite planning wall-time budget exhausted',path_clear=False,
            remaining_search_not_evaluated=True,physics_steps=0,impossible=False)
    if report.get('status')=='finite_search_no_path_not_impossible':
        route=report.get('route',{})
        examples=report.get('rejected_state_examples',[])
        witness=any((row.get('kind')=='self' and row.get('hits')) or
            (row.get('kind')=='environment' and row.get('check',{}).get('complete')is True and
             row.get('check',{}).get('hard_clear')is False and row.get('check',{}).get('hard_hits')) for row in examples)
        if route.get('status')in ('invalid_start','invalid_goal') and not witness:return None
        if route.get('status') not in ('invalid_start','invalid_goal','no_path','timeout','not_found'):return None
        return dict(kind='finite_existing_connector_search_rejected',path=str(path),sha256=file_sha256(path),
            route_status=route['status'],state_checks=report.get('state_checks'),impossible=False)
    for name in ('full_self60.json','full_environment60.json'):
        candidate=path.parent/name
        if report.get('audit_file_sha256',{}).get(name)!= (file_sha256(candidate) if candidate.is_file() else None):continue
        proof=positive(candidate,path.parent/'connected_plan.json')
        if proof:return proof
    return None


def _completed_same_state_planning_sequence(summary,metadata,trial,checkpoint_path):
    """Read appended final planning evidence only for byte-identical dynamics.

    The accepted completed-cycle checkpoint remains the physical continuation
    path.  A final route checkpoint with rejected proposals is evidence only,
    never an alternative resume state or a way around completion validation.
    """
    if checkpoint_path is None:
        raise ValueError('Summary/checkpoint sequence mismatch requires explicit accepted checkpoint path')
    from resume_native_state import require_completed_checkpoint
    trial=Path(trial).resolve();accepted=Path(checkpoint_path).resolve();final=trial/'route_checkpoint'
    if not accepted.is_relative_to(trial) or accepted==final:
        raise ValueError('Appended planning evidence requires this trial actual completed-cycle checkpoint')
    accepted_meta,accepted_arrays=load_checkpoint(accepted)
    final_meta,final_arrays=load_checkpoint(final)
    require_completed_checkpoint(accepted_meta)
    if accepted_meta!=metadata:raise ValueError('Accepted checkpoint metadata differs from classifier input')
    ignored={'sequence','metadata_sha256'}
    if ({k:v for k,v in accepted_meta.items()if k not in ignored}!=
            {k:v for k,v in final_meta.items()if k not in ignored}):
        raise ValueError('Final planning checkpoint changed actual physical/controller/provenance metadata')
    if accepted_meta['time_s']!=final_meta['time_s'] or accepted_meta['time_s']!=summary.get('simulated_s'):
        raise ValueError('Final planning evidence must share the exact accepted physical timestamp')
    if set(accepted_arrays)!=set(final_arrays):raise ValueError('Final planning checkpoint array mapping differs')
    for key,value in accepted_arrays.items():
        other=final_arrays[key]
        if value.dtype!=other.dtype or value.shape!=other.shape or value.tobytes()!=other.tobytes():
            raise ValueError('Final planning checkpoint actual native array differs: '+key)
    sequence=final_meta.get('sequence');saved_sequence=_read(trial/'truss_sequence.json')
    completion_marker_omissions={}
    if (isinstance(sequence,dict) and isinstance(saved_sequence,dict) and sequence.get('termination') in
            ('group_no_group_meets_minimum','group_no_centered_group_fits',
             'group_no_visible_qualifying_group','group_no_eligible_fruit_remaining') and sequence.get('cycle_plans') and
            all(row.get('purpose')=='camera_scan_prefix' and row.get('status')=='motion_completed' for row in sequence['cycle_plans'])):
        for key,value in (('checkpoint_capture_reason','completed_cycle'),('checkpoint_physics_valid',True)):
            if key not in saved_sequence and sequence.get(key)==value and accepted_meta['sequence'].get(key)==value:
                completion_marker_omissions[key]=value
    normalized_saved=(dict(saved_sequence,**completion_marker_omissions) if isinstance(saved_sequence,dict) else saved_sequence)
    if (not isinstance(sequence,dict) or summary.get('truss_sequence')!=sequence or normalized_saved!=sequence):
        raise ValueError('Final planning sequence disagrees with summary or saved sequence')
    proposals=[row for row in sequence.get('cycle_plans',[]) if row.get('purpose')!='camera_scan_prefix']
    zero_footprint_scan=(not proposals and sequence.get('termination') in
        ('group_no_group_meets_minimum','group_no_centered_group_fits',
         'group_no_visible_qualifying_group','group_no_eligible_fruit_remaining') and sequence['cycle_plans'] and
        all(row.get('purpose')=='camera_scan_prefix' and row.get('status')=='motion_completed' for row in sequence['cycle_plans']))
    if (not proposals and not zero_footprint_scan) or any(row.get('status')!='planning_rejected' for row in proposals):
        raise ValueError('Same-state planning reconciliation is limited to rejected proposals or a known zero-footprint actual scan')
    proof=dict(accepted_checkpoint=str(accepted),accepted_checkpoint_metadata_sha256=accepted_meta['metadata_sha256'],
        final_planning_evidence=str(final),final_metadata_sha256=final_meta['metadata_sha256'],
        accepted_physical_checkpoint_path_changed=False,exact_physical_timestamp_s=accepted_meta['time_s'],
        all_native_array_keys=sorted(accepted_arrays),all_array_dtypes_shapes_bytes_equal=True,
        physical_controller_and_provenance_metadata_unchanged=True,
        summary_and_saved_final_sequence_agree=True,final_planning_checkpoint_used_for_resume=False,
        saved_sequence_completion_marker_omissions=completion_marker_omissions)
    return sequence,proof


def completed_patrol_harvest_outcome(summary,metadata,decision,trial,*,checkpoint_path=None):
    """Separate accepted native observation time from actual harvest cycles.

    Call only after validating/accepting this trial's actual final checkpoint.
    A finite planning-only run can propose a new view from that NEW state;
    unknown errors and physical stops are never a rollback/retry permission.
    """
    if summary.get('completed')is not True or summary.get('stop_reason')is not None:
        raise ValueError('Patrol outcome requires a valid completed native terminal state')
    result=dict(actual_completed_cycles=0,outcome='observation_only_segment',
        finite_planning_exhausted=False,requires_forward_probe=False,
        accepted_new_terminal_checkpoint_required=True,checkpoint_rollback=False,harvest_credit=0)
    if decision.get('action')!='harvest':return result
    sequence=metadata.get('sequence');selected=summary.get('selected_cycle_evidence',{}).get('cycles')
    if not isinstance(sequence,dict) or not isinstance(sequence.get('cycle_plans'),list) or not isinstance(selected,list):
        raise ValueError('Missing actual checkpoint/selected-cycle evidence for patrol harvest decision')
    planning_sequence_proof=None
    zero_footprint_termination=(summary.get('truss_sequence',{}).get('termination') in
        ('group_no_group_meets_minimum','group_no_centered_group_fits',
         'group_no_visible_qualifying_group','group_no_eligible_fruit_remaining') and
        summary['truss_sequence'].get('termination')!=sequence.get('termination'))
    if (isinstance(summary.get('truss_sequence'),dict) and
            (summary['truss_sequence'].get('cycle_plans')!=sequence['cycle_plans'] or zero_footprint_termination)):
        sequence,planning_sequence_proof=_completed_same_state_planning_sequence(summary,metadata,trial,checkpoint_path)
    records=sequence['cycle_plans'];truss=decision['target']['truss_key']
    count=completed_native_cycles(records,truss_key=truss)
    measured=[row for row in selected if row.get('status')=='motion_completed' and
        row.get('recorded_physical_cycle')is True and row.get('actual_trace_measurements_available')is True and
        set(row.get('actual_recorded_phases',[]))&{'close','cycle_reclose','belt_spin','belt_finish'}]
    if count:
        if len(measured)!=count:raise ValueError('Native completed-cycle count lacks matching actual belt phase evidence')
        return dict(result,actual_completed_cycles=count,outcome='completed_actual_harvest_cycles',
            evidence=dict(completed_native_cycles=count,selected_cycle_indices=[row.get('index') for row in measured]))
    proposals=[row for row in records if row.get('purpose')!='camera_scan_prefix']
    scans=[row for row in summary.get('selected_cycle_evidence',{}).get('scans',[]) if
        row.get('status')=='motion_completed' and row.get('harvest_attempt')is False and row.get('actual_recorded_phases')]
    if (not proposals and not selected and scans and records and
            all(row.get('purpose')=='camera_scan_prefix' and row.get('status')=='motion_completed' for row in records) and
            sequence.get('termination') in ('group_no_group_meets_minimum','group_no_centered_group_fits',
                'group_no_visible_qualifying_group','group_no_eligible_fruit_remaining')):
        path=Path(trial)/'camera_region_decisions.json'
        history=_read(path) if path.is_file() else []
        last=history[-1] if isinstance(history,list) and history else {}
        region=last.get('camera_region',{});observation=region.get('observation',{})
        if (last.get('reason')!=sequence['termination'][6:] or region.get('selected_truss')!=truss or
                observation.get('stale')is not False or not isinstance(region.get('accepted_observations'),list) or
                observation.get('timestamp_s')!=summary.get('simulated_s') or
                observation.get('current_timestamp_s')!=summary.get('simulated_s')):
            raise ValueError('Camera footprint shortage lacks its actual fresh terminal observation')
        actual_observation=latest_camera_observation(trial,summary)
        if observation.get('objects_path')!=actual_observation['objects_path']:
            raise ValueError('Camera footprint shortage must use this exact terminal camera exposure')
        visible=region['accepted_observations'];minimum=last.get('minimum_fruits');best=last.get('best_geometric_count')
        if (type(minimum)is not int or minimum<2 or type(best)is not int or not 0<=best<minimum or
                any(not isinstance(row,dict) or not str(row.get('name','')).startswith(truss+'_Fruit_') or
                    np.asarray(row.get('estimated_center_world_m'),float).shape!=(3,) or
                    not np.isfinite(np.asarray(row.get('estimated_center_world_m'),float)).all() for row in visible)):
            raise ValueError('Camera footprint shortage requires finite visible-fruit and minimum-count evidence')
        needs_alignment=len(visible)>=minimum
        return dict(result,outcome='camera_footprint_unavailable' if needs_alignment else 'camera_visible_group_below_minimum',
            requires_forward_probe=needs_alignment,requires_fresh_camera_alignment=needs_alignment,
            evidence=dict(termination=sequence['termination'],visible_fruit_count=len(visible),
                minimum_fruits=minimum,best_geometric_count=best,
                rejected_proposal_count=0,actual_completed_cycles=0,planning_exhaustion_claimed=False,
                camera_region_decisions=str(path.resolve()),camera_region_decisions_sha256=file_sha256(path),
                camera_observation=copy.deepcopy(observation),
                visible_fruit_names=[row['name'] for row in region['accepted_observations']],
                group_reason=last['reason'],actual_terminal_state_preserved=True,
                same_state_final_planning_sequence_proof=planning_sequence_proof))
    if (not proposals or len(selected)!=len(proposals) or measured or not scans or
            any(row.get('status')!='planning_rejected' for row in proposals) or
            any(row.get('status')!='planning_rejected' or row.get('recorded_physical_cycle')is not False or
                row.get('planned_manipulation_started')is not False for row in selected) or
            sequence.get('termination') not in ('attempt_budget_exhausted_remaining_fruit',
                'group_group_planning_alternatives_exhausted')):
        raise ValueError('Zero harvest cycles without conclusive finite scan/planning exhaustion evidence')
    proofs=[_finite_completed_camera_rejection(row,trial,summary.get('model_sha256')) for row in proposals]
    if any(proof is None for proof in proofs):
        raise ValueError('Unknown/inconclusive rejected camera proposal is not a forward-retry permission')
    return dict(result,outcome='finite_camera_planning_exhausted_after_actual_scan',
        finite_planning_exhausted=True,requires_forward_probe=True,evidence=dict(
            termination=sequence['termination'],rejected_proposal_count=len(proposals),
            rejected_proposals=proofs,actual_completed_cycles=0,actual_terminal_state_preserved=True,
            same_state_final_planning_sequence_proof=planning_sequence_proof))


def complete_visible_footprint_deferral(patrol,target_key,*,assembly,trial,summary,checkpoint,arguments):
    """Re-evaluate only the current fully observed set with the existing selector.

    This is an operational current-view budget decision, never proof that
    partial belt contact or a different motion cannot harvest these fruits.
    """
    from camera_region_policy import CameraRegionSelector
    from harvest_group_policy import load_policy
    assembly,trial,checkpoint=map(lambda value:Path(value).resolve(),(assembly,trial,checkpoint))
    item=patrol.queue.get(target_key);policy_path=trial/'harvest_group_policy.json'
    if item is None or not policy_path.is_file() or not patrol.observations:return None
    observation=latest_camera_observation(trial,summary);last=patrol.observations[-1]
    expected=set(item['observed_attached_fruit_keys'])
    if (last['stale'] or item['last_observation_index']!=len(patrol.observations)-1 or
            item['last_observation'].get('objects_path')!=observation['objects_path'] or
            set(item['last_visible_fruit_keys'])!=expected):return None
    manifest=_read(assembly/'motion_manifest.json')
    if _active_collision_target_key(manifest,patrol.inventory)!=target_key:return None
    fruits=[fruit for fruit in manifest['plant']['fruits'] if fruit['name'].startswith(item['truss_key']+'_Fruit_')]
    policy=load_policy(policy_path,assembly,fruits)
    if len(expected)<policy['minimum_fruits']:return None
    metadata,arrays=load_checkpoint(checkpoint,expected_scene_sha256=manifest['scene_sha256'])
    if metadata['time_s']!=summary['simulated_s']:raise ValueError('Current-view geometry requires the exact accepted terminal time')
    def value(name,default):
        return arguments[arguments.index(name)+1] if name in arguments else default
    if (float(value('--grasp-lateral-offset-mm',0.)) or
            int(value('--single-fruit-when-remaining',0)) or '--split-unproductive-pairs'in arguments):return None
    depth=float(value('--grasp-depth-mm',65.))/1000
    if float(value('--later-grasp-depth-mm',depth*1000))/1000!=depth:return None
    height=manifest.get('grasp_reference',{}).get('height_offset_m',0.)+float(value('--grasp-height-offset-mm',0.))/1000
    headings=[0.,15.,-15.,30.,-30.]
    if '--camera-heading-offsets-deg'in arguments:
        first=arguments.index('--camera-heading-offsets-deg')+1;end=first
        while end<len(arguments)and not arguments[end].startswith('--'):end+=1
        headings=list(map(float,arguments[first:end]))
    global_heading=float(value('--truss-entry-yaw-offset-deg',0.))
    headings=[global_heading+angle for angle in headings if abs(global_heading+angle)<=90.]
    attempts=metadata.get('sequence',{}).get('attempt_counts',{});rows=[]
    for heading in headings:
        selector=CameraRegionSelector(policy,None,lambda *unused:observation,
            plant_source=manifest['plant']['source_plant'],selected_truss=item['truss_key'],depth_m=depth,
            height_offset_m=height,min_visible_pixels=int(value('--camera-min-visible-pixels',12)),
            prefer_unattempted_trusses=True,yaw_offset_deg=heading)
        group=selector(fruits,metadata['break_events'],list(map(str,arrays['body_paths'])),arrays['body_poses_world'],
            attempts,max_attempts=int(value('--max-fruit-attempts',3)),order='highest-first',single_fruit_when_remaining=0)
        decision=selector.last_decision;region=decision['camera_region']
        if (group is not None or region['observation']['stale'] or
                {row['key']for row in region['accepted_observations']}!=expected or
                decision['reason'] not in ('no_group_meets_minimum','no_centered_group_fits') or
                not 0<=decision['best_geometric_count']<policy['minimum_fruits']):return None
        rows.append(dict(heading_offset_deg=heading,reason=decision['reason'],
            best_geometric_count=decision['best_geometric_count'],candidate_aim_count=decision['candidate_aim_count']))
    if not rows:return None
    return dict(reason='finite_current_view_center_footprint_unavailable',target_key=target_key,
        exact_current_visible_and_remembered_attached_fruit_keys=sorted(expected),minimum_fruits=policy['minimum_fruits'],
        observation=observation,checkpoint=str(checkpoint),checkpoint_metadata_sha256=metadata['metadata_sha256'],
        policy=str(policy_path),policy_sha256=file_sha256(policy_path),source_sha256=policy['source_sha256'],
        existing_selector_evaluations=rows,depth_m=depth,height_offset_m=height,
        view_scope='This fully observed known ripe set and configured center-based footprint only',
        hidden_centers_used_for_aim=False,partial_contact_not_evaluated=True,IK_or_physics_impossibility=False,
        physics_steps=0,harvest_credit=0,reachability='unknown; finite current-view operational deferral')


def completed_classifier_deferral_recovery(report,prior_root,*,assembly,checkpoint,metadata):
    """Reclassify only the old known-budget classifier stop at its NEW state.

    The caller has already validated this exact accepted checkpoint with the
    normal native state loader.  This grants no physical-stop recovery, old
    state selection, harvest credit or permission to ignore unknown failures.
    """
    expected=report.get('stop_reason')
    zero_footprint_error='Zero harvest cycles without conclusive finite scan/planning exhaustion evidence'
    if (report.get('status')!='blocked_preserving_physical_history' or expected not in
            ('Unknown/inconclusive rejected camera proposal is not a forward-retry permission',zero_footprint_error)):return None
    try:
        assembly=Path(assembly).resolve();checkpoint=Path(checkpoint).resolve();prior_root=Path(prior_root).resolve()
        if (Path(report['accepted_checkpoint']).resolve()!=checkpoint or
                Path(report['accepted_assembly']).resolve()!=assembly or not report.get('segments')):return None
        row=report['segments'][-1];trial=Path(row['trial']).resolve();result=row.get('benchmark_result',{})
        summary=_read(trial/'summary.json')
        if (row.get('status')!='pipeline_or_physical_blocker' or row.get('blocker_type')!='ValueError' or
                row.get('blocker')!=expected or row.get('decision',{}).get('action')!='harvest' or
                Path(row['checkpoint']).resolve()!=checkpoint or Path(row['assembly']).resolve()!=assembly or
                not trial.is_relative_to(prior_root) or result.get('returncode')!=0 or result.get('timeout')is not False or
                result.get('summary')!=summary or summary.get('completed')is not True or summary.get('stop_reason')is not None or
                summary.get('initialization_valid')is not True or summary.get('actual_self_collision_audit_passed')is not True or
                _read(checkpoint/'checkpoint.json')!=metadata or
                summary.get('model_sha256')!=metadata.get('scene_sha256') or
                summary.get('simulated_s')!=metadata.get('time_s')):return None
        outcome=completed_patrol_harvest_outcome(summary,metadata,row['decision'],trial,checkpoint_path=checkpoint)
        camera_alignment=(expected==zero_footprint_error and outcome.get('requires_fresh_camera_alignment')is True and
            outcome['finite_planning_exhausted']is False and outcome['evidence']['rejected_proposal_count']==0)
        finite_connector=(expected!=zero_footprint_error and any(
            p.get('kind')=='finite_existing_connector_wall_time_budget_exhausted'
            for p in outcome.get('evidence',{}).get('rejected_proposals',[])))
        if (outcome['actual_completed_cycles']!=0 or not outcome['requires_forward_probe'] or
                not (camera_alignment or finite_connector)):return None
        return dict(reason=('known_camera_footprint_shortage_reclassified_at_exact_new_actual_endpoint' if camera_alignment
                            else 'known_finite_planning_budget_reclassified_at_exact_new_actual_endpoint'),
            target_key=row['decision']['target_key'],outcome=outcome,trial=str(trial),checkpoint=str(checkpoint),
            prior_report_sha256=file_sha256(prior_root/'camera_patrol.json'),
            actual_summary_sha256=file_sha256(trial/'summary.json'),accepted_checkpoint_path_changed=False,
            actual_physical_stop_accepted=False,checkpoint_rollback=False,harvest_credit=0)
    except (OSError,ValueError,KeyError,TypeError):return None


_KNOWN_BUILD_FAILURE_RUNNER_SHA='10a984a545910628a3a741be82ef6ad2f1dfa95c66f87405fca7f05a701b6711'
_KNOWN_BUILD_FAILURE_HELPER_SHA='648a0ffd77bfe83475cb2496585f8500d3cfc0cab654d1a26a7bb21da5bfc002'
_KNOWN_BUILD_FAILURE='Same-plant target revision changed noncollision authored properties or topology'


def pre_native_target_revision_build_recovery(report,prior_root,*,assembly,checkpoint,metadata,proof_path,terminal_proof):
    """Opt-in migration of one diagnosed pre-native build failure, never physics.

    The caller still runs all normal prior campaign, native credit, queue,
    configuration, source identity and terminal checkpoint validations.
    Rejected derivative assets are evidence only and are never executed.
    """
    if proof_path is None:return None
    prior_root,assembly,checkpoint,proof_path=map(lambda value:Path(value).resolve(),
        (prior_root,assembly,checkpoint,proof_path))
    proof=_read(proof_path)
    if (proof.get('schema')!='s300-known-pre-native-target-revision-failure-proof-v1' or
            proof.get('passed')is not True or
            proof.get('failure_class')!='confirmed_pre_native_target_revision_audit_implementation_error' or
            proof.get('corrected_implementation_verified')is not True or
            proof.get('source_files_unchanged')is not True or
            proof.get('post_restore_campaign_physics_started')is not False or
            proof.get('rejected_derived_asset_accepted')is not False):
        raise ValueError('Explicit target-revision build recovery needs independently confirmed implementation error proof')
    if (_read(prior_root/'camera_patrol.json')!=report or
            report.get('schema')!='s300-visible-camera-patrol-campaign-v1' or
            report.get('status')!='blocked_preserving_physical_history' or report.get('executed')is not True or
            report.get('training_eligible')is not False or report.get('harvest_success')is not None or
            report.get('target_truss_windows',{}).get('enabled')is not True or
            report.get('stop_reason')!=_KNOWN_BUILD_FAILURE or
            proof.get('prior_report_sha256')!=file_sha256(prior_root/'camera_patrol.json') or
            proof.get('prior_queue_sha256')!=file_sha256(prior_root/'camera_patrol_queue.json') or
            Path(report.get('accepted_assembly','')).resolve()!=assembly or
            Path(report.get('accepted_checkpoint','')).resolve()!=checkpoint):
        raise ValueError('Target-revision recovery report or exact accepted endpoint mismatch')
    rows=report.get('segments',[])
    if (len(rows)!=4 or [row.get('index')for row in rows]!=list(range(4)) or
            [row.get('decision',{}).get('action')for row in rows[:3]]!=['scan','move','lift_scan'] or
            any(row.get('status')!='completed_physics'for row in rows[:3])):
        raise ValueError('Known build recovery requires the original three completed camera segments')
    last=rows[-1];completed=rows[-2]
    if (set(last)!={'index','decision','status','parent_assembly','parent_checkpoint','blocker_type','blocker'} or
            last.get('status')!='pipeline_or_physical_blocker' or
            last.get('blocker_type')!='ValueError' or last.get('blocker')!=_KNOWN_BUILD_FAILURE or
            Path(last.get('parent_assembly','')).resolve()!=assembly or
            Path(last.get('parent_checkpoint','')).resolve()!=checkpoint or
            Path(completed.get('checkpoint','')).resolve()!=checkpoint or
            Path(completed.get('assembly','')).resolve()!=assembly or
            last.get('decision',{}).get('action')!='harvest' or
            last['decision'].get('target_key')!='/World/Greenhouse/Plants/Plant_20/Truss_01' or
            metadata.get('time_s')!=11.0):
        raise ValueError('Known build failure was not before dispatch at the exact last measured lift checkpoint')
    terminal=terminal_proof
    if (not isinstance(terminal,dict)or terminal.get('verified_final_physical_state')is not True or
            terminal.get('checkpoint_metadata_sha256')!=metadata['metadata_sha256']or
            terminal!=completed.get('checkpoint_proof')):
        raise ValueError('Target-revision recovery terminal arrays/controller/trace/model proof changed')
    loaded,arrays=load_checkpoint(checkpoint)
    final_meta,final_arrays=load_checkpoint(Path(completed['trial'])/'route_checkpoint')
    if loaded!=metadata or any(metadata.get(key)!=final_meta.get(key)for key in
            ('source_assembly','scene_sha256','source_plant','manifest','controller_state','break_events','time_s')):
        raise ValueError('Target-revision recovery native metadata/controller/break provenance changed')
    if set(arrays)!=set(final_arrays):raise ValueError('Target-revision recovery terminal native array inventory differs')
    for key,value in arrays.items():
        other=final_arrays[key]
        if value.dtype!=other.dtype or value.shape!=other.shape or value.tobytes()!=other.tobytes():
            raise ValueError('Target-revision recovery differs from actual terminal native arrays: '+key)
    if metadata['time_s']!=final_meta['time_s']:
        raise ValueError('Target-revision recovery chose an earlier actual checkpoint')
    for name,expected in report['controller_source_sha256'].items():
        if file_sha256(prior_root/'controller_source'/name)!=expected:
            raise ValueError('Frozen failed campaign controller source changed: '+name)
    if report['controller_source_sha256'].get('camera_patrol_runner.py')!=_KNOWN_BUILD_FAILURE_RUNNER_SHA:
        raise ValueError('Unknown camera dispatcher version is not covered by this build recovery')
    native_trial=Path(completed['trial'])
    archive=_read(native_trial/'code_sha256.json')
    if (archive.get('same_plant_target_revision.py')!=_KNOWN_BUILD_FAILURE_HELPER_SHA or
            file_sha256(native_trial/'code/same_plant_target_revision.py')!=_KNOWN_BUILD_FAILURE_HELPER_SHA or
            proof.get('failing_helper_sha256')!=_KNOWN_BUILD_FAILURE_HELPER_SHA or
            proof.get('failing_runner_sha256')!=_KNOWN_BUILD_FAILURE_RUNNER_SHA):
        raise ValueError('Known failed helper source/hash mismatch')
    from same_plant_target_revision import _source_files
    source_files=_source_files(assembly)
    if proof.get('source_file_sha256')!=source_files:
        raise ValueError('Accepted source asset changed after the unused revision failure')
    rejected=prior_root/'windows/003_plant_20_Truss_01_revision'
    wanted={'scene.usda','composed.usdc','skin.npz','checkpoint_materialization_identity.json'}
    if set(p.name for p in rejected.iterdir())!=wanted or any(not p.is_file()or p.is_symlink()for p in rejected.iterdir()):
        raise ValueError('Unused rejected derivative contains unrecognized outputs or execution artifacts')
    rejected_files={str(p):file_sha256(p)for p in sorted(rejected.iterdir())}
    if proof.get('rejected_output_file_sha256')!=rejected_files:
        raise ValueError('Rejected derivative raw evidence changed')
    storage=_read(rejected/'checkpoint_materialization_identity.json')
    if (storage.get('schema')!='s300-whole-canonical-layer-storage-identity-v1' or
            storage.get('passed')is not True or storage.get('canonical_equal')is not True or
            storage.get('full_authored_layer_checked')is not True or
            storage.get('numeric_validity',{}).get('passed')is not True or
            storage.get('materialized_sha256')!=rejected_files[str(rejected/'composed.usdc')] or
            storage.get('wrapper_sha256')!=rejected_files[str(rejected/'scene.usda')]):
        raise ValueError('Known build recovery lacks exact raw materialization identity evidence')
    for phase in ('scan','harvest'):
        if (prior_root/f'segment_003_{phase}_cases.json').exists()or(prior_root/f'trials/segment_003_{phase}').exists():
            raise ValueError('Unaccepted target revision has dispatch/native artifacts; pre-native migration forbidden')
    for key in ('difference_evidence','fix_validation'):
        item=proof.get(key,{})
        path=Path(item.get('path','')).resolve()
        if not path.is_file()or file_sha256(path)!=item.get('sha256'):
            raise ValueError('Independent root-cause/fix evidence missing or changed: '+key)
    if (proof.get('checkpoint_metadata_sha256')!=metadata['metadata_sha256'] or
            proof.get('checkpoint_npz_sha256')!=metadata['checkpoint_npz_sha256']):
        raise ValueError('Build recovery proof belongs to another physical state')
    return dict(reason='explicit_confirmed_pre_native_target_revision_build_error',
        prior_report_sha256=file_sha256(prior_root/'camera_patrol.json'),
        prior_queue_sha256=file_sha256(prior_root/'camera_patrol_queue.json'),
        independent_proof_path=str(proof_path),independent_proof_sha256=file_sha256(proof_path),
        accepted_checkpoint=str(checkpoint),checkpoint_metadata_sha256=metadata['metadata_sha256'],
        actual_terminal_native_array_keys=sorted(arrays),all_terminal_array_bytes_equal=True,
        accepted_checkpoint_path_changed=False,source_file_sha256=source_files,
        rejected_output=str(rejected),rejected_output_file_sha256=rejected_files,
        unaccepted_native_dispatch=False,actual_physical_stop_accepted=False,
        rejected_derived_asset_accepted=False,prior_queue_and_three_actual_segments_preserved=True,
        mandatory_fresh_scan=True,checkpoint_rollback=False,harvest_credit=0)


def shared_initial_model_blocker(record,base_arguments,*,campaign=None):
    """Defer an unchanged full plant, without claiming its other targets failed.

    Only a conclusive independent idle/warmup failure qualifies.  This is an
    operational quarantine for this campaign, not a collision certificate or
    a reachability label.  Target-specific plant variants and partial models
    must keep their own independent checks.
    """
    probe=record.get('startup_preflight',{})
    def option(name):
        if name not in base_arguments:return None
        i=base_arguments.index(name)
        return base_arguments[i+1] if i+1<len(base_arguments) else None
    target=record.get('decision',{}).get('target',{})
    if (record.get('status')!='unused_window_startup_deferred' or
            record.get('accepted_parent_unchanged') is not True or
            record.get('target_truss_window_scope') is not None or '--camera-target-truss-window' in base_arguments or
            option('--plant-dynamics')!='full' or '--plant-interaction-variant' in base_arguments or
            probe.get('status')!='placement_initialization_or_idle_physics_invalid' or
            probe.get('can_defer_unused_placement') is not True or
            probe.get('native_physics_initialization_observed') is not True or
            probe.get('accepted_campaign_motion') is not False or
            probe.get('diagnostic_state_accepted') is not False or
            not target.get('source_plant')):
        return None
    return dict(source_plant=target['source_plant'],evidence=dict(
        reason='shared_full_plant_native_initialization_invalid',
        actually_tested_target=record['decision']['target_key'],
        campaign=None if campaign is None else str(Path(campaign).resolve()),
        segment_index=record['index'],startup_preflight=copy.deepcopy(probe),
        window_handoff=copy.deepcopy(record.get('window_handoff')),
        other_targets_individually_tested=False,reachability='unknown',
        robot_motion_attempted=False,accepted_parent_unchanged=True,
        scope='This immutable full-plant input in this campaign; no geometry or physics change'))


def _measured_state(assembly,manifest,checkpoint):
    metadata,proof=validated_initial_checkpoint(assembly,manifest,checkpoint)
    _,arrays=load_checkpoint(checkpoint,expected_scene_sha256=manifest['scene_sha256'])
    names=list(map(str,arrays['robot_dof_names']))
    lift=None
    if 'farmily_lift_height_joint' in names:lift=float(arrays['robot_q'][names.index('farmily_lift_height_joint')])
    return metadata,proof,float(proof['measured_initial_station_y_m']),lift


def _active_collision_target_key(manifest,inventory):
    scope=target_truss_camera_scope(manifest,inventory)
    return manifest['plant']['source_plant']+'/'+scope['target_truss_window']['target_truss']


def ready_camera_service_calibration(assembly,manifest,arguments,bed_gutter_center_x):
    """Derive camera-to-grasp offsets from this exact robot's ready FK.

    The synthetic point supplies only the already selected row side to the
    existing front-view yaw calculation.  No plant/fruit center is queried.
    """
    from pxr import Usd
    from motion_kinematics import ToolKinematics,ready_lift_branch
    from camera_scan_prefix import append_scan_pan
    stage=Usd.Stage.Open(str(Path(assembly)/'scene.usda'))
    kin=ToolKinematics(stage);q=np.asarray(manifest.get('initial_robot_q'),float)
    if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Exact current-model seven-joint ready pose required for camera calibration')
    def argument(name,default):
        if name not in arguments:return default
        index=arguments.index(name)
        if index+1>=len(arguments):raise ValueError('Missing calibration argument '+name)
        value=float(arguments[index+1])
        if not np.isfinite(value):raise ValueError('Finite calibration argument required')
        return value
    lift_offset=argument('--ready-lift-offset-mm',0.)/1000
    if lift_offset:q=ready_lift_branch(kin,q,lift_offset)[0][-1]
    mounted=manifest['mounted_rgbd'];mount=np.asarray(mounted['body_from_camera_optical'],float)
    phases=[('calibration_pose',.5,np.array([q,q]),.096,0.,0.)]
    meta=dict(phases=[dict(name='calibration_pose',duration_s=.5,waypoint_q=[q.tolist(),q.tolist()],end_gap_m=.096)])
    target=np.array([bed_gutter_center_x,kin.world[1,3],kin.fk(q)[2,3]])
    phases,_=append_scan_pan(phases,meta,kin,target,mount,view='row-front',
        rail_axis_world=manifest['rail'].get('axis_world',[0.,1.,0.]))
    q=np.asarray(phases[-1][2])[-1]
    camera=kin.frames(q)[mounted['body_path']]@mount
    depth=argument('--grasp-depth-mm',65.)/1000
    height=manifest.get('grasp_reference',{}).get('height_offset_m',0.)+argument('--grasp-height-offset-mm',0.)/1000
    service=kin.fk(q)@np.array([0.,.158-depth,.037+height,1.])
    return dict(camera_foreaft_m=float(service[1]-camera[1,3]),
        camera_to_grasp_height_m=float(service[2]-camera[2,3]),ready_q=q.tolist(),
        camera_world_m=camera[:3,3].tolist(),service_world_m=service[:3].tolist(),
        source='Exact current assembly FK, fixed mounted optical extrinsic and existing gripper nose/depth geometry',
        uses_hidden_plant_positions=False,scene_sha256=manifest['scene_sha256'])


def camera_patrol_campaign(assembly,inventory_path,output,base_arguments,*,
        native_python='/root/isaac-sim-6.1.0/python.sh',benchmark_python=None,
        execute=False,goal_trusses=6,max_decisions=80,step_m=.15,bed_gutter_center_x=None,
        initial_checkpoint=None,prior_campaign=None,timeout=1800.,camera_foreaft_m=None,camera_to_grasp_height_m=None,
        benchmark_runner=subprocess.run,window_switch=switch_window,adaptive_lift=False,target_truss_windows=False,
        build_failure_proof=None,initial_lift_height_m=None,prefer_active_truss=False,forward_pass=False):
    """One-bed camera patrol using unchanged native execution and guards."""
    if isinstance(goal_trusses,bool) or not isinstance(goal_trusses,int) or not 1<=goal_trusses<=33:
        raise ValueError('Finite distinct-truss goal within1..33 required')
    if isinstance(max_decisions,bool) or not isinstance(max_decisions,int) or not 1<=max_decisions<=200:
        raise ValueError('Finite camera decision budget within1..200 required')
    if isinstance(timeout,bool) or not np.isfinite(timeout) or timeout<=0:raise ValueError('Positive finite timeout required')
    assembly,inventory_path,output=map(lambda value:Path(value).resolve(),(assembly,inventory_path,output))
    benchmark_python=Path(benchmark_python or sys.executable).resolve()
    inventory=_read(inventory_path);manifest=_read(assembly/'motion_manifest.json')
    if type(target_truss_windows)is not bool:raise ValueError('Explicit boolean camera target-truss window mode required')
    if type(prefer_active_truss)is not bool or (prefer_active_truss and not target_truss_windows):
        raise ValueError('Active truss cost preference requires explicit target-only camera windows')
    if type(forward_pass)is not bool or (forward_pass and not target_truss_windows):
        raise ValueError('Forward pass requires explicit target-only camera windows')
    initial_target_scope=(target_truss_camera_scope(manifest,inventory) if target_truss_windows else None)
    if type(adaptive_lift)is not bool:raise ValueError('Adaptive lift requires an explicit boolean')
    lift_limits=None
    if adaptive_lift or initial_lift_height_m is not None:
        from patrol_lift_search import lift_bounds_for_assembly
        lift_limits=lift_bounds_for_assembly(assembly)
    if initial_lift_height_m is not None and (isinstance(initial_lift_height_m,bool) or
            not isinstance(initial_lift_height_m,(int,float)) or not np.isfinite(initial_lift_height_m) or
            not lift_limits[0]<=initial_lift_height_m<=lift_limits[1]):
        raise ValueError('Initial camera lift scan height must be finite within the actual physical lift limits')
    if not manifest.get('mounted_rgbd') or not manifest.get('rail'):raise ValueError('Mounted camera and actual rail assembly required')
    if file_sha256(assembly/'scene.usda')!=manifest['scene_sha256']:raise ValueError('Immutable camera patrol source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    if bed_gutter_center_x is None:
        gutters=sorted({float(row['gutter_center_x']) for row in inventory['trusses'] if row['gutter_center_x']<0})
        if not gutters:raise ValueError('No left-side authored gutter in actual farm inventory')
        bed_gutter_center_x=max(gutters)
    calibration=None
    if prior_campaign is not None and (camera_foreaft_m is None or camera_to_grasp_height_m is None):
        # A window's initial_robot_q is the last measured, load-deflected
        # state, not a fresh calibration pose.  Keep the already proven fixed
        # service calibration.  The exact accepted model/checkpoint/inventory
        # and full queue configuration are still checked below before launch.
        prior_queue_path=Path(prior_campaign).resolve()/'camera_patrol_queue.json'
        prior_calibration=_read(prior_queue_path)['config']
        if camera_foreaft_m is None:camera_foreaft_m=prior_calibration['camera_foreaft_m']
        if camera_to_grasp_height_m is None:camera_to_grasp_height_m=prior_calibration['camera_to_grasp_height_m']
        calibration=dict(camera_foreaft_m=camera_foreaft_m,
            camera_to_grasp_height_m=camera_to_grasp_height_m,
            source='Unchanged calibrated queue offsets; measured window start is not a calibration pose',
            prior_queue_sha256=file_sha256(prior_queue_path))
    if camera_foreaft_m is None or camera_to_grasp_height_m is None:
        calibration=ready_camera_service_calibration(assembly,manifest,base_arguments,bed_gutter_center_x)
        if camera_foreaft_m is None:camera_foreaft_m=calibration['camera_foreaft_m']
        if camera_to_grasp_height_m is None:camera_to_grasp_height_m=calibration['camera_to_grasp_height_m']
    patrol=CameraPatrol(inventory,bed_gutter_center_x,step_m=step_m,
        camera_foreaft_m=camera_foreaft_m,camera_to_grasp_height_m=camera_to_grasp_height_m)
    station=float(manifest['rail']['initial_station_y_m']);lift=None
    checkpoint=None if initial_checkpoint is None else Path(initial_checkpoint).resolve()
    initial_proof=None
    if checkpoint is not None:
        metadata,initial_proof,station,lift=_measured_state(assembly,manifest,checkpoint)
        patrol.record_native_detachments(exact_native_detached_keys(manifest,metadata))
    if build_failure_proof is not None and (prior_campaign is None or not target_truss_windows):
        raise ValueError('Explicit build failure recovery requires a prior target-only camera campaign')
    prior_proof=None;prior_harvested=set();prior_cycles={};prior_pending_plan_failure=None
    if prior_campaign is not None:
        # A code/server restart may continue only the last accepted physical
        # endpoint.  Incomplete attempts never become an accepted start, and
        # an old camera queue is reacquired by the mandatory initial scan.
        prior_root=Path(prior_campaign).resolve()
        prior_report=_read(prior_root/'camera_patrol.json')
        prior_ledger=CameraPatrol.from_dict(inventory,_read(prior_root/'camera_patrol_queue.json'))
        classifier_recovery=(completed_classifier_deferral_recovery(prior_report,prior_root,
            assembly=assembly,checkpoint=checkpoint,metadata=metadata) if checkpoint is not None else None)
        build_recovery=(pre_native_target_revision_build_recovery(prior_report,prior_root,
            assembly=assembly,checkpoint=checkpoint,metadata=metadata,proof_path=build_failure_proof,
            terminal_proof=initial_proof) if checkpoint is not None else None)
        if (checkpoint is None or prior_report.get('schema')!='s300-visible-camera-patrol-campaign-v1' or
                prior_report.get('training_eligible') is not False or prior_report.get('harvest_success') is not None or
                prior_report.get('status')=='actual_physics_stopped_preserving_terminal_state' or
                (prior_report.get('status')=='blocked_preserving_physical_history' and classifier_recovery is None and build_recovery is None) or
                Path(prior_report.get('accepted_checkpoint','')).resolve()!=checkpoint or
                Path(prior_report.get('accepted_assembly','')).resolve()!=assembly or
                prior_report.get('inventory_sha256')!=file_sha256(inventory_path)):
            raise ValueError('Patrol restart must use the exact last accepted physical checkpoint and inventory; no rollback')
        if bool(prior_report.get('target_truss_windows',{}).get('enabled',False))!=target_truss_windows:
            raise ValueError('Patrol restart may not silently change the physical target-truss scope')
        if prior_ledger.config!=patrol.config:
            raise ValueError('Patrol restart may not silently change camera calibration or queue policy')
        patrol=prior_ledger
        for item in patrol.queue.values():item['visible_now']=False
        patrol.record_native_detachments(exact_native_detached_keys(manifest,metadata))
        prior_harvested=set(prior_report.get('measured_harvested_trusses',[]))
        # Harvest credits are rederived from completed native cycles, not a
        # number in the queue.  Follow the bounded explicit continuation chain.
        chain=prior_report;chain_root=prior_root;seen=set();proven=set()
        while True:
            if chain_root in seen or len(seen)>=32:raise ValueError('Invalid or unbounded patrol continuation chain')
            seen.add(chain_root)
            for row in chain['segments']:
                shared=shared_initial_model_blocker(row,chain.get('base_motion_arguments',[]),campaign=chain_root)
                if shared is not None:patrol.quarantine_plant(shared['source_plant'],shared['evidence'])
                if row.get('status')!='completed_physics':continue
                result=row.get('benchmark_result',{});summary=result.get('summary',{})
                if result.get('returncode')!=0 or result.get('timeout') is not False or summary.get('completed') is not True or summary.get('stop_reason') is not None:
                    raise ValueError('Patrol continuation contains unproven completion credit')
                source=_read(Path(row['assembly'])/'motion_manifest.json')['plant']['source_plant']
                proven.update(measured_harvested_trusses(summary,source))
                if row['decision']['action']=='harvest':
                    prior_metadata=_read(Path(row['checkpoint'])/'checkpoint.json')
                    outcome=completed_patrol_harvest_outcome(summary,prior_metadata,row['decision'],row['trial'],
                        checkpoint_path=row['checkpoint'])
                    key=row['decision']['target_key']
                    prior_cycles[key]=prior_cycles.get(key,0)+outcome['actual_completed_cycles']
                    already_deferred=any(failure.get('evidence',{}).get('actual_trial')==row['trial'] or
                        failure.get('evidence',{}).get('prior',{}).get('trial')==row['trial']
                        for failure in patrol.queue.get(key,{}).get('planning_failures',[]))
                    if (chain_root==prior_root and Path(row['checkpoint']).resolve()==checkpoint and
                            outcome['requires_forward_probe'] and not already_deferred):
                        prior_pending_plan_failure=dict(target_key=key,outcome=outcome,
                            trial=row['trial'],checkpoint=row['checkpoint'])
            parent=chain.get('prior_campaign')
            if parent is None:break
            chain_root=Path(parent['root']).resolve()
            if file_sha256(chain_root/'camera_patrol.json')!=parent['report_sha256']:
                raise ValueError('Prior patrol report changed after continuation')
            chain=_read(chain_root/'camera_patrol.json')
        if prior_harvested!=proven:raise ValueError('Patrol continuation harvest count differs from native cycle evidence')
        if classifier_recovery is not None:
            prior_pending_plan_failure=copy.deepcopy(classifier_recovery)
        prior_proof=dict(root=str(prior_root),report_sha256=file_sha256(prior_root/'camera_patrol.json'),
            queue_sha256=file_sha256(prior_root/'camera_patrol_queue.json'),
            accepted_checkpoint=str(checkpoint),accepted_assembly=str(assembly),
            historical_distinct_native_harvests=sorted(proven),
            reobserves_before_next_decision=True,rollback=False)
        if classifier_recovery is not None:
            prior_proof['strict_completed_classifier_deferral_recovery']=copy.deepcopy(classifier_recovery)
        if build_recovery is not None:
            prior_proof['strict_pre_native_target_revision_build_recovery']=copy.deepcopy(build_recovery)
        if prior_pending_plan_failure is not None:
            prior_proof['pending_latest_actual_scan_planning_failure']=copy.deepcopy(prior_pending_plan_failure)
        old_config=_read(prior_root/'camera_patrol_queue.json')['config']
        prior_proof['new_explicit_queue_proposal_defaults']={key:value for key,value in patrol.config.items() if key not in old_config}
    # Validate the complete base controls before creating a run or initializing native physics.
    patrol_motion_arguments(base_arguments,inventory_path,decision=dict(action='scan'),checkpoint=checkpoint,
        target_truss_windows=target_truss_windows)
    output.mkdir(parents=True,exist_ok=False)
    controller_sources={}
    snapshot=output/'controller_source';snapshot.mkdir()
    for name in ('camera_patrol.py','camera_patrol_runner.py','camera_patrol_startup_cache.py',
            'run_row_harvest.py','route_checkpoint.py','native_route_start.py','camera_scan_prefix.py',
            'strict_window_storage.py','patrol_lift_search.py','camera_lift_vantage.py'):
        data=Path(__file__).with_name(name).read_bytes();(snapshot/name).write_bytes(data)
        controller_sources[name]=hashlib.sha256(data).hexdigest()
    report=dict(schema='s300-visible-camera-patrol-campaign-v1',status='planned_only',
        executed=execute,initial_assembly=str(assembly),initial_scene_sha256=manifest['scene_sha256'],
        inventory=str(inventory_path),inventory_sha256=file_sha256(inventory_path),
        controller_source_sha256=controller_sources,base_motion_arguments=list(base_arguments),
        camera_service_calibration=calibration or dict(camera_foreaft_m=camera_foreaft_m,
            camera_to_grasp_height_m=camera_to_grasp_height_m,source='Explicit calibrated caller offsets'),
        initial_checkpoint=initial_proof,goal_distinct_trusses=goal_trusses,max_decisions=max_decisions,
        prior_campaign=prior_proof,
        adaptive_lift=dict(enabled=adaptive_lift,actual_lift_limits_m=lift_limits,
            maximum_scan_proposals_per_group=3,source='Visible depth and actual measured lift; existing native motion'),
        segments=[],measured_harvested_trusses=sorted(prior_harvested),achieved_distinct_trusses=len(prior_harvested),
        goal_reached=False,training_eligible=False,harvest_success=None,
        native_continuity='Validated measured terminal checkpoint; explicit active-window freeze and native solver restart',
        count_definition='Existing measured_harvested_trusses: completed belt cycle plus exact new selected-fruit native break; not whole-truss or collection success')
    if target_truss_windows:
        report['target_truss_windows']=dict(enabled=True,initial_scope=initial_target_scope,
            collision_policy='Existing exact target truss and own stem only; proven released fruit colliders remain enabled',
            same_plant_other_trusses='Measured target/collision-only revision followed by mandatory fresh camera scan; no plant rebuild or ghost-truss execution')
    if prefer_active_truss:
        report['active_truss_cost_preference']=dict(enabled=True,
            identity_source='Existing target_truss_camera_scope exact manifest mapping',
            scope='Prefer active collision window only among fresh visible serviceable harvest candidates',
            other_pending_targets_preserved=True,physical_policy_changed=False)
    if forward_pass:
        report['forward_pass']=dict(enabled=True,max_local_forward_views=1,
            finite_planning_deferral='Only the independently exhausted camera target at its actual completed endpoint',
            active_untried_groups_preserved=True,inactive_revisits_preserved_for_measured_recovery=True,
            physical_policy_changed=False,harvest_credit=0)
    if initial_lift_height_m is not None:
        report['initial_lift_scan_requested']=dict(height_m=float(initial_lift_height_m),
            source='Explicit first physical lift-scan decision; fresh observation remains mandatory',
            actual_lift_limits_m=lift_limits,physical_initial_state_changed=False)
    replay=[]
    def save():
        _write(output/'camera_patrol.json',report);_write(output/'camera_patrol_queue.json',patrol.to_dict())
        _write(output/'replay_segments.json',dict(schema='s300-patrol-actual-replay-sources-v1',segments=replay,
            reconstruction='Actual stored states only; no nominal plan playback',training_eligible=False))
    save()
    if not execute:return report
    harvested=set(prior_harvested);decision=dict(action='scan',reason='initial_actual_camera_observation_required',
        current_station_y_m=station,next_station_y_m=station)
    if initial_lift_height_m is not None:
        decision=dict(action='lift_scan',reason='explicit_initial_physical_lift_then_fresh_camera_observation',
            current_station_y_m=station,next_station_y_m=station,next_lift_m=float(initial_lift_height_m))
    accepted_assembly=assembly;accepted_checkpoint=checkpoint;cycles=dict(prior_cycles)
    def adaptive_decision(proposed,*,after_planning_failure=False,camera_alignment=False):
        if (not adaptive_lift or proposed.get('action')!='harvest' or
                (forward_pass and after_planning_failure and not camera_alignment)):return proposed
        from patrol_lift_search import propose_lift_scan,remember_lift_proposal
        item=patrol.queue[proposed['target_key']]
        alternative=propose_lift_scan(item,station_m=station,measured_lift_m=lift,
            limits_m=lift_limits,after_planning_failure=after_planning_failure)
        if alternative is None:return proposed
        remember_lift_proposal(item,alternative)
        return alternative
    if (forward_pass and prior_pending_plan_failure is not None and
            prior_pending_plan_failure['outcome'].get('requires_fresh_camera_alignment')is True):
        prior=prior_pending_plan_failure
        geometry=complete_visible_footprint_deferral(patrol,prior['target_key'],assembly=assembly,trial=prior['trial'],
            summary=_read(Path(prior['trial'])/'summary.json'),checkpoint=checkpoint,arguments=base_arguments)
        if geometry is not None:prior['complete_visible_footprint_deferral']=geometry
    if (initial_lift_height_m is None and adaptive_lift and prior_pending_plan_failure is not None and
            prior_pending_plan_failure['outcome'].get('requires_fresh_camera_alignment')is True and
            prior_pending_plan_failure.get('complete_visible_footprint_deferral')is None):
        # Use the validated endpoint receipt only to propose a new sensor view.
        # The actual queue remains nonvisible until that physical lift-scan.
        key=prior_pending_plan_failure['target_key'];item=patrol.queue[key]
        observation=prior_pending_plan_failure['outcome']['evidence']['camera_observation']
        if (patrol.observations and not patrol.observations[-1]['stale'] and
                item.get('last_observation_index')==len(patrol.observations)-1 and
                item.get('last_observation',{}).get('objects_path')==observation.get('objects_path')):
            from patrol_lift_search import propose_lift_scan,remember_lift_proposal
            remembered=copy.deepcopy(item);remembered['visible_now']=True
            proposal=propose_lift_scan(remembered,station_m=station,measured_lift_m=lift,
                limits_m=lift_limits,after_planning_failure=True)
            if proposal is not None:
                remember_lift_proposal(item,proposal);decision=proposal
                report['initial_camera_alignment_recovery']=dict(proposal=copy.deepcopy(proposal),
                    prior=copy.deepcopy(prior_pending_plan_failure),uses_validated_terminal_observation=True,
                    requires_new_actual_observation_before_harvest=True,harvest_credit=0)
                prior_pending_plan_failure=None
    startup_cache=StartupPreflightCache()
    for index in range(max_decisions):
        record=dict(index=index,decision=copy.deepcopy(decision),status='preparing',
            parent_assembly=str(accepted_assembly),parent_checkpoint=None if accepted_checkpoint is None else str(accepted_checkpoint))
        report['segments'].append(record);report['status']='running';save()
        print('[PATROL]',index,decision['action'],'station',round(station,4),
            'target',decision.get('target_key','-'),'harvested',len(harvested),'/',goal_trusses,flush=True)
        if decision['action']=='stop':
            report.update(status='pending_observation_or_finite_budget_blocker',stop_reason=decision['reason']);save();return report
        candidate_assembly=accepted_assembly;candidate_checkpoint=accepted_checkpoint;new_window=False
        resume_target_revision=False
        name=f"segment_{index:03d}_{decision['action']}";trial=output/'trials'/name/name
        result=None;result_path=None
        try:
            manifest=_read(accepted_assembly/'motion_manifest.json');source=manifest['plant']['source_plant']
            target=decision.get('target')
            if (target_truss_windows and decision['action']=='harvest' and target['source_plant']==source and
                    target['truss_key']!=manifest['plant_interaction_variant']['target_truss']):
                if accepted_checkpoint is None:raise ValueError('Observed same-plant retarget requires the exact measured terminal checkpoint')
                from same_plant_target_revision import revise
                candidate_assembly=output/'windows'/f"{index:03d}_plant_{target['plant_id']:02d}_{target['truss_key']}_revision"
                fruit=sorted(target['last_visible_fruit_keys'])[0].rsplit('/',1)[-1]
                revision=revise(accepted_assembly,accepted_checkpoint,candidate_assembly,
                    target['truss_key']+'_'+fruit,fast_materialization=True)
                record['same_plant_target_revision']=revision
                resume_target_revision=True
                # A physical scan in the newly enabled collision scope must
                # finish before any harvest proposal uses this derived asset.
                decision=dict(action='scan',reason='same_plant_target_revision_requires_fresh_observation',
                    current_station_y_m=station,next_station_y_m=station,
                    target_key=decision['target_key'],target=copy.deepcopy(target),
                    original_harvest_decision=copy.deepcopy(decision))
                record['decision']=copy.deepcopy(decision)
                name=f'segment_{index:03d}_scan';trial=output/'trials'/name/name
                record['target_truss_window_scope']=target_truss_camera_scope(
                    _read(candidate_assembly/'motion_manifest.json'),inventory,selected_truss=target['truss_key'])
            if decision['action']=='harvest' and target['source_plant']!=source:
                if accepted_checkpoint is None:raise ValueError('Observed next window requires the actual validated terminal parent')
                unresolved=[row['key'] for row in patrol.queue.values() if row['source_plant']==source and
                    row['status'] not in ('finite_blocked','observed_fruits_native_detached',
                        'below_group_minimum_after_native_detachment','not_attempted_shared_invalid_initial_model')]
                if unresolved:
                    raise ValueError('Cannot freeze active plant with unresolved observed groups: '+str(unresolved))
                candidate_assembly=output/'windows'/f"{index:03d}_plant_{target['plant_id']:02d}"
                fruit=sorted(target['last_visible_fruit_keys'])[0].rsplit('/',1)[-1]
                switched=window_switch(accepted_assembly,accepted_checkpoint,candidate_assembly,target['plant_id'],
                    target['truss_key']+'_'+fruit,near_trusses_metadata=inventory_path,
                    allow_frozen_reactivation=True,fast_materialization=True)
                record['window_handoff']=switched;new_window=True
                if target_truss_windows:
                    record['target_truss_window_scope']=target_truss_camera_scope(
                        _read(candidate_assembly/'motion_manifest.json'),inventory,selected_truss=target['truss_key'])
                # Independent unused-window check never replaces actual continuation.
                probe_name=f'startup_{index:03d}'
                probe_cases=output/(probe_name+'_cases.json')
                probe_base=patrol_motion_arguments(base_arguments,inventory_path,
                    decision=dict(action='scan'),checkpoint=None,new_window=True,target_truss_windows=target_truss_windows)
                probe_args=startup_preflight_arguments([value for value in probe_base if value!='--camera-scan-only'],
                    physics_only=True)
                _write(probe_cases,[dict(name=probe_name,arguments=probe_args)])
                probe_trials=output/'startup_preflight'/probe_name
                command=[str(benchmark_python),'-u',str(Path(__file__).with_name('benchmark_motion.py')),
                    str(candidate_assembly),str(probe_cases),'--output',str(probe_trials),'--python',str(native_python),
                    '--jobs','1','--timeout',str(timeout)]
                identity=startup_identity(candidate_assembly,probe_args,accepted_checkpoint/'checkpoint.json',
                    runtime_identity=dict(native_python=str(native_python)))
                probe=startup_cache.lookup(identity)
                if probe is None:
                    try:benchmark_runner(command,check=True)
                    except subprocess.CalledProcessError:pass
                    probe_result_path=probe_trials/(probe_cases.stem+'_results.json')
                    probe_result=_read(probe_result_path)[0] if probe_result_path.is_file() else dict(summary=None,returncode=None,timeout=None)
                    probe=preflight_result(probe_result,probe_trials/probe_name)
                    stored=startup_cache.store(identity,probe_result,probe_trials/probe_name)
                    record['startup_probe_cache']=dict(key=identity['key'],stored=stored,actual_new_execution=True)
                else:
                    record['startup_probe_cache']=dict(key=identity['key'],actual_new_execution=False,
                        reuse='Exact independent idle diagnostic only; actual execution guards remain required')
                record['startup_preflight']=probe
                if not probe['passed']:
                    if probe['can_defer_unused_placement']:
                        patrol.record_blocker(target['key'],'unused_window_initialization_or_idle_physics_invalid',dict(startup_preflight=probe))
                        record.update(status='unused_window_startup_deferred',accepted_parent_unchanged=True)
                        shared=shared_initial_model_blocker(record,base_arguments,campaign=output)
                        if shared is not None:
                            record['shared_initial_model_quarantine']=patrol.quarantine_plant(
                                shared['source_plant'],shared['evidence'])
                        decision=adaptive_decision(patrol.decide(current_station_y_m=station,active_source_plant=source,
                            **({'forward_pass':True} if forward_pass else {})));save();continue
                    raise ValueError('Unused-window startup pipeline error or unknown: '+probe['status'])
            arguments=patrol_motion_arguments(base_arguments,inventory_path,decision=decision,
                checkpoint=candidate_checkpoint,new_window=new_window,target_truss_windows=target_truss_windows,
                resume_target_truss_revision=resume_target_revision)
            if target_truss_windows:
                target_truss_camera_scope(_read(candidate_assembly/'motion_manifest.json'),inventory,
                    selected_truss=decision['target']['truss_key'] if decision['action']=='harvest' else None)
            cases=output/(name+'_cases.json');_write(cases,[dict(name=name,arguments=arguments)])
            trials=output/'trials'/name;result_path=trials/(cases.stem+'_results.json')
            command=[str(benchmark_python),'-u',str(Path(__file__).with_name('benchmark_motion.py')),
                str(candidate_assembly),str(cases),'--output',str(trials),'--python',str(native_python),
                '--jobs','1','--timeout',str(timeout)]
            record.update(command=command,assembly=str(candidate_assembly),trial=str(trial),status='native_running');save()
            try:benchmark_runner(command,check=True)
            except subprocess.CalledProcessError:
                if not result_path.is_file():raise
            result=_read(result_path)[0];record['benchmark_result']=result
            summary=result.get('summary')
            saved_state=next((trial/name for name in ('states.npz','states_partial.npz') if (trial/name).is_file()),None)
            if saved_state is not None:
                replay.append(dict(index=index,assembly=str(candidate_assembly),trial=str(trial),
                    states=str(saved_state),phase=decision['action'],
                    physical_stop_reason=summary.get('stop_reason') if isinstance(summary,dict) else 'native_process_incomplete',
                    completed=summary.get('completed') if isinstance(summary,dict) else False,
                    independent_retry=False,actual_saved_state_present=True))
            if (result.get('timeout') is not False or result.get('returncode')!=0 or
                    not isinstance(summary,dict)):
                raise ValueError('Native process incomplete or failed; preserve exact artifacts')
            if summary.get('completed') is not True or summary.get('stop_reason') is not None:
                record.update(status='actual_physics_stopped',stop_reason=summary.get('stop_reason'))
                report.update(status='actual_physics_stopped_preserving_terminal_state',last_actual_trial=str(trial),
                    stop_reason=summary.get('stop_reason'));save();return report
            checkpoint,metadata=completed_final_checkpoint(trial,summary)
            current_manifest=_read(candidate_assembly/'motion_manifest.json')
            metadata,proof,station,lift=_measured_state(candidate_assembly,current_manifest,checkpoint)
            observation=latest_camera_observation(trial,summary)
            native_keys=exact_native_detached_keys(current_manifest,metadata)
            observed=patrol.observe(observation,current_station_y_m=station,current_lift_m=lift,native_detached_keys=native_keys)
            if target_truss_windows:
                record['target_truss_observation_scope']=dict(
                    target_truss_window=target_truss_camera_scope(current_manifest,inventory)['target_truss_window'],
                    other_same_plant_trusses='Retained in visible queue; require a measured collision-only revision and fresh scan before harvest',
                    physical_collision_policy_changed=False,harvest_credit=0)
            accepted_assembly,accepted_checkpoint=candidate_assembly,checkpoint
            # Even an unknown outcome cannot roll back these newly validated
            # actual observations/terminal dynamics to the preceding segment.
            report.update(accepted_assembly=str(accepted_assembly),accepted_checkpoint=str(accepted_checkpoint))
            record.update(checkpoint=str(checkpoint),checkpoint_proof=proof,
                measured_station_y_m=station,camera_observation=observed)
            outcome=completed_patrol_harvest_outcome(summary,metadata,decision,trial,checkpoint_path=checkpoint)
            credited=measured_harvested_trusses(summary,current_manifest['plant']['source_plant'])
            harvested.update(credited)
            record.update(status='completed_physics',checkpoint=str(checkpoint),checkpoint_proof=proof,
                measured_station_y_m=station,camera_observation=observed,measured_harvested_trusses=credited)
            record['harvest_outcome']=outcome
            next_decision=None
            if decision['action']=='lift_scan':
                record['actual_lift_scan']=dict(requested_height_m=decision['next_lift_m'],
                    measured_height_m=lift,error_m=lift-decision['next_lift_m'],
                    fresh_camera_observation=observation,actual_measured_state_used=True)
            report.update(measured_harvested_trusses=sorted(harvested),achieved_distinct_trusses=len(harvested),
                accepted_assembly=str(accepted_assembly),accepted_checkpoint=str(accepted_checkpoint))
            print('[PATROL] actual completed','station',round(station,4),'camera accepted fruits',
                len(observed['accepted']),'new trusses',credited,'harvested',len(harvested),'/',goal_trusses,flush=True)
            if len(harvested)>=goal_trusses:
                report.update(status='measured_distinct_truss_goal_reached',goal_reached=True);save();return report
            if decision['action']=='harvest':
                key=decision['target_key'];cycles[key]=cycles.get(key,0)+outcome['actual_completed_cycles']
                item=patrol.queue[key]
                record['actual_completed_physical_cycles_for_target']=cycles[key]
                if outcome['requires_forward_probe']:
                    geometry=(complete_visible_footprint_deferral(patrol,key,assembly=accepted_assembly,trial=trial,
                        summary=summary,checkpoint=checkpoint,arguments=base_arguments)
                        if forward_pass and outcome.get('requires_fresh_camera_alignment')is True else None)
                    height_proposal=adaptive_decision(dict(action='harvest',target_key=key),after_planning_failure=True,
                        camera_alignment=outcome.get('requires_fresh_camera_alignment')is True) if geometry is None else dict(action='harvest')
                    if geometry is not None:
                        record['camera_view_geometry_deferral']=patrol.record_local_failure(key,geometry['reason'],
                            dict(current_view=geometry,outcome=outcome,actual_trial=str(trial),accepted_terminal_checkpoint=str(checkpoint)),
                            current_station_y_m=station,native_observation_executed=True,max_forward_views=1)
                    elif height_proposal['action']=='lift_scan':
                        next_decision=height_proposal
                        record['camera_alignment_recovery' if outcome.get('requires_fresh_camera_alignment') else 'planning_lift_recovery']=dict(proposal=height_proposal,
                            failed_outcome=outcome,actual_trial=str(trial),
                            accepted_terminal_checkpoint=str(checkpoint),harvest_credit=0)
                    else:
                        record['camera_view_deferral' if outcome.get('requires_fresh_camera_alignment') else 'planning_deferral']=patrol.record_local_failure(key,
                            outcome['outcome'],dict(outcome=outcome,actual_trial=str(trial),
                                accepted_terminal_checkpoint=str(checkpoint)),current_station_y_m=station,
                            native_observation_executed=True,
                            **({'max_forward_views':1} if forward_pass and not outcome.get('requires_fresh_camera_alignment') else {}))
                elif cycles[key]>=3 and item['status']=='pending':
                    patrol.record_blocker(key,'finite_three_physical_cycle_budget_exhausted',dict(
                        actual_cycles=cycles[key],last_trial=str(trial),native_detached_keys=native_keys,
                        remaining_observed_attached_fruit_keys=item['observed_attached_fruit_keys']))
            elif index==0 and prior_pending_plan_failure is not None:
                key=prior_pending_plan_failure['target_key'];item=patrol.queue.get(key)
                if (item is not None and item['status'] in ('pending','revisit_required') and
                        prior_pending_plan_failure['outcome'].get('requires_fresh_camera_alignment')is True):
                    geometry=(complete_visible_footprint_deferral(patrol,key,assembly=accepted_assembly,trial=trial,
                        summary=summary,checkpoint=checkpoint,arguments=base_arguments) if forward_pass else None)
                    proposal=adaptive_decision(dict(action='harvest',target_key=key),after_planning_failure=True,camera_alignment=True) if geometry is None else dict(action='harvest')
                    if geometry is not None:
                        record['camera_view_geometry_deferral']=patrol.record_local_failure(key,geometry['reason'],
                            dict(current_view=geometry,prior=prior_pending_plan_failure,fresh_actual_trial=str(trial),accepted_terminal_checkpoint=str(checkpoint)),
                            current_station_y_m=station,native_observation_executed=True,max_forward_views=1)
                    elif proposal['action']=='lift_scan':
                        next_decision=proposal
                        record['prior_camera_alignment_recovery']=dict(proposal=copy.deepcopy(proposal),
                            prior=copy.deepcopy(prior_pending_plan_failure),harvest_credit=0)
                    else:
                        record['prior_camera_view_deferral']=patrol.record_local_failure(key,'camera_footprint_unavailable',
                            dict(prior=prior_pending_plan_failure,fresh_actual_trial=str(trial),
                                accepted_terminal_checkpoint=str(checkpoint)),current_station_y_m=station,
                            native_observation_executed=True)
                elif item is not None and item['status'] in ('pending','revisit_required'):
                    record['prior_completed_scan_planning_deferral']=patrol.record_local_failure(key,
                        'prior_finite_camera_planning_exhausted_reobserved_at_exact_accepted_endpoint',
                        dict(prior=prior_pending_plan_failure,fresh_actual_trial=str(trial),
                            accepted_terminal_checkpoint=str(checkpoint)),current_station_y_m=station,
                        native_observation_executed=True,
                        **({'max_forward_views':1} if forward_pass else {}))
                else:
                    record['prior_completed_scan_planning_deferral']=dict(status='not_serviceable_after_fresh_observation',
                        target_key=key,queue_status=None if item is None else item['status'],harvest_credit=0)
                prior_pending_plan_failure=None
            elif decision['action']=='scan' and decision.get('target_key') in patrol.queue:
                key=decision['target_key'];item=patrol.queue[key]
                if not item['visible_now'] and item['status'] in ('pending','revisit_required'):
                    # A remembered group must be seen again before motion.
                    # Bound failed reacquisition with the same forward-view
                    # budget, rather than repeatedly loading an identical scan.
                    record['camera_reacquisition']=patrol.record_local_failure(key,
                        'fresh_scan_did_not_reacquire_minimum_visible_red_group',dict(
                            actual_observation=observation,observed_attached_fruit_keys=item['observed_attached_fruit_keys'],
                            no_harvest_motion_attempted=True),current_station_y_m=station)
            # Only an exact never-detached archive is reactivatable.  General
            # revisits with this plant's own fallen fruit remain unsupported;
            # they must not be rebuilt from rest or silently lose releases.
            frozen=[row['source_plant'] for row in current_manifest.get('route_checkpoint',{}).get('outgoing_history',[])
                if row.get('unharvested_reactivation_eligible') is not True]
            decision=next_decision or adaptive_decision(patrol.decide(current_station_y_m=station,
                active_source_plant=current_manifest['plant']['source_plant'],frozen_source_plants=frozen,
                **({'active_collision_target_key':_active_collision_target_key(current_manifest,inventory)}
                    if prefer_active_truss else {}),
                **({'forward_pass':True} if forward_pass else {})))
            save()
        except Exception as error:
            deferred=None
            if decision['action']=='harvest' and accepted_checkpoint is not None and result is not None:
                deferred=unexecuted_planning_failure(result,trial,candidate_assembly)
            if deferred is not None:
                retry=patrol.record_local_failure(decision['target_key'],deferred['reason'],deferred,
                    current_station_y_m=station)
                record.update(status='unexecuted_finite_planning_proposal_rejected',planning_deferral=retry,
                    accepted_parent_unchanged=True,blocker=str(error))
                current_manifest=_read(accepted_assembly/'motion_manifest.json')
                decision=adaptive_decision(patrol.decide(current_station_y_m=station,active_source_plant=current_manifest['plant']['source_plant'],
                    **({'forward_pass':True} if forward_pass else {})))
                save();continue
            record.update(status='pipeline_or_physical_blocker',blocker_type=type(error).__name__,blocker=str(error))
            report.update(status='blocked_preserving_physical_history',last_actual_trial=str(trial),
                stop_reason=str(error),goal_reached=False);save();return report
    report.update(status='finite_decision_budget_exhausted',goal_reached=False,
        interpretation='Finite search/scan budget only; target reachability remains unknown');save();return report
