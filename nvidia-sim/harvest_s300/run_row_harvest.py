"""Finite rail-row campaign using the existing native motion benchmark runner.

Screened station poses are proposals only. Every station is replanned/audited on
its actual current asset. Travel uses the physical rail joint. A new plant
window starts from the latest valid measured native state; invalid later state
never rolls back to an earlier successful checkpoint.

An explicit camera-only station proposal may retain a blocked old harvest map
path as provenance. That path is never reused or reported clear: independent
startup, fresh initial scan audits and camera-selected harvest audits still run.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

from compact_motion_assets import file_sha256, verify_compact_dependencies
from native_route_checkpoint import load_checkpoint
from resume_native_state import require_completed_checkpoint
from route_checkpoint import switch_window
from plan_row_stations import validate_preparation_variant,preparation_variant_arguments
from row_harvest_targets import scoped_trusses,eligible_truss_fruits,truss_key
from camera_plan_alternatives import camera_plan_alternatives,alternative_key,selected_seed_evidence


DEFAULT_ARGUMENTS = ['--headless', '--physics-only', '--native-materialized-stage', '--mode', 'harvest', '--hz', '120',
    '--device', 'cpu', '--performance', 'optimized', '--threads', '2',
    '--contact-backend', 'callback', '--plant-dynamics', 'full',
    '--solver', 'TGS', '--position-iterations', '64', '--velocity-iterations', '4',
    '--scene-ccd', '--contact-limit-mm', '1', '--fruit-contact-limit-mm', '1', '--fruit-fruit-limit-mm', '1',
    '--preparation', 'horizontal_ready', '--measured-ready-start', '--approach-yaw-mode', 'truss-axis',
    '--truss-axis-min-fruits', '5', '--sparse-entry-duration-scale', '4',
    '--single-fruit-when-remaining', '2',
    '--harvest-profile', 'belt_open_close_cycle', '--belt-speed-m-s', '.05',
    '--belt-seconds', '2', '--close-seconds', '6', '--belt-ramp-seconds', '.5',
    '--grasp-gap-mm', '28', '--grasp-depth-mm', '65', '--grasp-height-offset-mm', '-10',
    '--preparation-gap-mm', '44', '--ready-lift-offset-mm', '-150',
    '--preparation-height-first', '--gravity-neutral-warmup', '--armature-setup', 'authored',
    '--released-fruit-ground-diagnostic', '--released-fruit-gripper-support-diagnostic', '--released-fruit-same-truss-diagnostic', '--verify-leaf-contact-after-step',
    '--verify-harvest-contact-after-step', '--verify-background-leaf-enclosures', '--verify-structural-scene-query',
    '--buffer-contact-events', '--whole-truss', '--truss-order', 'lowest-first',
    '--max-truss-cycles', '20', '--max-fruit-attempts', '6', '--max-aim-translation-m', '1.5',
    '--save-route-checkpoint', '--checkpoint-cycle-states']


def _read(path): return json.loads(Path(path).read_text())


def _write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def select_stations(inventory, screen, current_plant_id, current_station_y_m, max_plants=15,
                    *,pin_current_station=True):
    """Keep every exclusion and unknown, prioritizing finite self-clear proposals."""
    if not 1 <= max_plants <= 33:
        raise ValueError('Finite max-plants budget must be within1..33')
    if not isinstance(pin_current_station,bool):raise ValueError('Explicit boolean current-station pinning policy required')
    results = {(r['plant_id'], r['truss_index']): r for r in screen['results']}
    candidates = {}; excluded = []; unknown = []
    for row in inventory['trusses']:
        identity = dict(plant_id=row['plant_id'], truss_index=row['truss_index'], path=row['path'])
        root = np.asarray(row['root_position_world_m'], float)
        center = np.asarray(row['fruit_centroid_world_m'], float)
        gutter = float(row['gutter_center_x']); side = np.sign(-gutter)
        if root.shape != (3,) or center.shape != (3,) or not np.isfinite(np.r_[root, center, gutter]).all() or side == 0:
            raise ValueError('Malformed exact farm placement inventory')
        near = (root[0]-gutter)*side > 0
        if bool(row['rail_near']) != bool(near):
            raise ValueError('Root/gutter and inventory rail-side policy disagree')
        if not near or (center[0]-gutter)*side <= 0:
            excluded.append(dict(**identity, reason='opposite_side_stem' if not near else 'fruit_centroid_behind_gutter',
                                 reachability='not inferred; placement policy exclusion'))
            continue
        result = results.get((row['plant_id'], row['truss_index']))
        clear = [] if result is None else [q for q in result['queries']
            if q['status'] == 'self_path_clear_full' and q.get('full_self_audit_complete') is True and q.get('full_self_audit_passed') is True]
        if not clear:
            unknown.append(dict(**identity, reason='finite_screen_no_self_clear_proposal',
                                reachability='unknown; finite IK/self search is not impossible',
                                screen_queries=[] if result is None else result['queries']))
            continue
        for query in clear:
            station = float(query['station_y_m'])
            if not np.isfinite(station) or not 0 <= station <= 5:
                raise ValueError('Screened physical rail station outside0..5m')
            candidate=dict(**identity, station_y_m=station,
                target_name=f"Truss_{row['truss_index']:02d}_"+query['group']['fruits'][0],
                screen_query_index=query['index'], old_screen_plan=query.get('plan'),
                certificate='Finite proposal from old source; actual current-model replan and native audit required')
            if 'preparation_variant' in query:
                variant=validate_preparation_variant(query['preparation_variant'])
                seed=variant.get('horizontal_ready_seed_index')
                if seed is not None and query.get('selected_staging_seed_index')!=seed:
                    raise ValueError('Screened preparation seed disagrees with the actual selected staging branch')
                candidate['preparation_variant']=variant
            candidates.setdefault(row['plant_id'], []).append(candidate)
    route = []
    if current_plant_id in candidates:
        current=candidates.pop(current_plant_id)
        if pin_current_station:
            first=min(current,key=lambda q:abs(q['station_y_m']-current_station_y_m))
            first['station_y_m']=float(current_station_y_m)
        else:
            # Fresh camera campaigns must physically reach their screened scan
            # proposal instead of silently replacing it with the source origin.
            first=min(current,key=lambda q:(q['station_y_m'],q['truss_index']))
        first['initial_current_plant']=True
        route.append(first)
    # One physical stop per plant; the native runner considers all of its
    # eligible trusses. Duplicate station proposals do not reset its fruit.
    selected = [min(rows, key=lambda q: (q['station_y_m'], q['truss_index'])) for rows in candidates.values()]
    route.extend(sorted(selected, key=lambda q: (q['station_y_m'], q['plant_id'])))
    unselected = [dict(row, reason='physical_campaign_plant_budget', reachability='unknown') for row in route[max_plants:]]
    return dict(stations=route[:max_plants], side_excluded=excluded,
                finite_screen_unknown=unknown, budget_unselected=unselected,
                source_screen_sha256=screen['source_scene_sha256'],
                screened_plant_count=len(route), selected_plant_count=min(max_plants, len(route)))


def select_preferred_stations(inventory, screen, preferred, screen_path, *,
                              current_plant_id, current_station_y_m, max_plants=15,
                              previous_window_plants=(),allow_camera_scan_only=False):
    """Use an explicit, hash-checked order of existing self-clear proposals.

    This changes proposal order only. It neither restores state nor promotes
    old self-collision evidence into a current environment/physics certificate.
    A plant already frozen in an outgoing measured window is not rebuilt from
    its original source, which would reset its physical history.
    """
    screen_path=Path(screen_path).resolve()
    if not isinstance(max_plants,int) or isinstance(max_plants,bool) or not 1<=max_plants<=33:
        raise ValueError('Finite max-plants budget must be within1..33')
    if not isinstance(allow_camera_scan_only,bool):
        raise ValueError('Explicit boolean camera scan proposal policy required')
    if (preferred.get('schema')!='s300-next-row-fullself-proposal-selection-v1' or
            Path(preferred.get('union_report','')).resolve()!=screen_path or
            preferred.get('union_report_sha256')!=file_sha256(screen_path)):
        raise ValueError('Preferred proposal source report/path/SHA mismatch')
    # Reuse all existing side-policy, finite-search and preparation checks.
    result=select_stations(inventory,screen,current_plant_id,current_station_y_m,max_plants=33)
    placements={(row['plant_id'],row['truss_index']):row for row in inventory['trusses']}
    eligible={(row['plant_id'],row['truss_index']) for row in result['stations']}
    # select_stations returns one station per plant. Retain every eligible
    # truss identity whose union query actually has a complete self audit.
    eligible.update((row['plant_id'],row['truss_index']) for row in screen['results']
        if any(query.get('status')=='self_path_clear_full' and
               query.get('full_self_audit_complete') is True and
               query.get('full_self_audit_passed') is True for query in row['queries']))
    excluded={(row['plant_id'],row['truss_index']) for row in result['side_excluded']}
    queries={(row['plant_id'],row['truss_index'],query['index']):query
             for row in screen['results'] for query in row['queries']}
    candidates=preferred.get('candidates')
    if not isinstance(candidates,list) or not candidates:raise ValueError('Nonempty preferred proposal list required')
    prior=set(previous_window_plants);selected=[];used=set();frozen=[]
    for proposed in candidates:
        key=(proposed.get('plant_id'),proposed.get('truss_index'),proposed.get('query_index'))
        query=queries.get(key);identity=key[:2]
        kind=proposed.get('proposal_kind')
        scan_only=kind=='camera_scan_station_only'
        if kind is not None and not scan_only:raise ValueError('Unknown preferred proposal kind')
        if scan_only:
            if not allow_camera_scan_only:raise ValueError('Camera scan-only proposal requires explicit camera-mode authorization')
            if (query is None or query.get('status') not in ('self_path_blocked_coarse','self_path_blocked_full','ik_path_not_found_finite') or
                    proposed.get('source_query_status')!=query.get('status') or
                    query.get('full_self_audit_passed') is True or
                    any(proposed.get(name)!=query.get(name) for name in ('full_self_audit_complete','full_self_audit_passed'))):
                raise ValueError('Camera scan proposal must preserve the exact finite source status and audit flags')
        full_clear=(query is not None and query.get('status')=='self_path_clear_full' and
                    all(query.get(name) is True and proposed.get(name) is True
                        for name in ('full_self_audit_complete','full_self_audit_passed')))
        if (identity not in placements or identity in excluded or
                (not scan_only and (identity not in eligible or not full_clear))):
            raise ValueError('Preferred proposal is not an exact eligible complete self-clear query')
        station=proposed.get('station_y_m')
        if (not isinstance(station,(int,float)) or isinstance(station,bool) or
                not np.isfinite(station) or not 0<=station<=5 or
                float(station)!=float(query['station_y_m'])):
            raise ValueError('Preferred physical rail station disagrees with source query')
        plan=Path(query.get('plan','')).resolve()
        if (Path(proposed.get('source_plan','')).resolve()!=plan or not plan.is_file() or
                proposed.get('source_plan_sha256')!=file_sha256(plan)):
            raise ValueError('Preferred source plan/path/SHA mismatch')
        variant=validate_preparation_variant(query.get('preparation_variant',
            dict(name='baseline_cartesian',horizontal_ready_connector='cartesian')))
        requested=validate_preparation_variant(proposed.get('preparation_variant',{}))
        if scan_only and variant['horizontal_ready_connector']!='joint':
            raise ValueError('Camera scan-only proposal requires the existing direct joint/ready scan prefix')
        arguments=preparation_variant_arguments(variant)
        if (preparation_variant_arguments(requested)!=arguments or
                proposed.get('selected_preparation_arguments')!=arguments):
            raise ValueError('Preferred preparation arguments disagree with exact source branch')
        seed=variant.get('horizontal_ready_seed_index')
        if seed is not None and query.get('selected_staging_seed_index')!=seed:
            raise ValueError('Preferred staging branch disagrees with selected native planning seed')
        if scan_only and not query.get('source_report'):
            raise ValueError('Camera scan proposal requires the original source report provenance')
        if query.get('source_report'):
            source=Path(query['source_report']).resolve()
            if (Path(proposed.get('source_report','')).resolve()!=source or not source.is_file() or
                    query.get('source_report_sha256')!=file_sha256(source) or
                    (scan_only and proposed.get('source_report_sha256')!=file_sha256(source))):
                raise ValueError('Preferred underlying source report/path/SHA mismatch')
            if scan_only:
                originals=[candidate for row in _read(source).get('results',[])
                    if (row.get('plant_id'),row.get('truss_index'))==identity
                    for candidate in row.get('queries',[]) if candidate.get('index')==key[2]]
                if (len(originals)!=1 or any(originals[0].get(name)!=query.get(name)
                        for name in ('status','full_self_audit_complete','full_self_audit_passed',
                                     'preparation_variant','selected_staging_seed_index'))):
                    raise ValueError('Camera scan proposal source status differs from the original finite report')
                original_plan=Path(originals[0].get('plan',''))
                if not original_plan.is_absolute():original_plan=source.parent/original_plan
                if original_plan.resolve()!=plan:
                    raise ValueError('Camera scan proposal plan differs from the original finite report')
        if key[0] in used:continue
        used.add(key[0])
        if key[0] in prior:
            frozen.append(dict(plant_id=key[0],truss_index=key[1],
                reason='previous_measured_window_not_reactivated_from_original_source',
                reachability='unknown; exact state-preserving reactivation is not implemented'))
            continue
        row=placements[identity]
        selected.append(dict(plant_id=key[0],truss_index=key[1],path=row['path'],
            station_y_m=float(station),target_name=f"Truss_{key[1]:02d}_"+query['group']['fruits'][0],
            screen_query_index=key[2],old_screen_plan=str(plan),preparation_variant=variant,
            certificate='Explicit old self-clear proposal; fresh current-model replan/native validation required',
            initial_current_plant=key[0]==current_plant_id))
        if scan_only:
            selected[-1].update(proposal_kind=kind,source_query_status=query['status'],
                source_full_self_audit=dict(complete=query.get('full_self_audit_complete'),passed=query.get('full_self_audit_passed')),
                certificate='Unverified camera scan station only; blocked old harvest path is not reused. Fresh startup, scan self/environment and observed harvest checks required')
    omitted={row['plant_id'] for row in result['stations']}-{row['plant_id'] for row in selected}-prior
    if omitted:raise ValueError('Preferred ordering silently omitted screened plants: '+str(sorted(omitted)))
    result.update(stations=selected[:max_plants],
        budget_unselected=[dict(row,reason='physical_campaign_plant_budget',reachability='unknown')
                           for row in selected[max_plants:]],
        previous_window_unselected=frozen,selected_plant_count=min(max_plants,len(selected)),
        preferred_order=True,actual_initial_plant_id=current_plant_id,
        actual_initial_station_y_m=float(current_station_y_m),
        preferred_design_initial_plant_id=preferred.get('current_plant_id'),
        preferred_design_initial_station_y_m=preferred.get('current_station_y_m'))
    return result


def completed_final_checkpoint(trial, summary):
    """Accept only a valid cycle at the final measured time; never roll back."""
    trial=Path(trial).resolve()
    if (summary.get('completed') is not True or summary.get('stop_reason') is not None or
            summary.get('initialization_valid') is not True or
            summary.get('actual_self_collision_audit_passed') is not True):
        raise ValueError('Physical route stopped or invalid; no checkpoint rollback is allowed')
    summary_path=trial/'summary.json'
    if not summary_path.is_file() or _read(summary_path)!=summary:
        raise ValueError('Final checkpoint requires its exact on-disk completed native summary')
    directory = trial/'cycle_checkpoints'
    rows = []
    paths=list(sorted(directory.iterdir())) if directory.is_dir() else []
    # A loop ending exactly at its last planned endpoint need not enter the
    # next cycle boundary. The final snapshot remains eligible only when the
    # native runner explicitly tagged that actual completed cycle.
    if (trial/'route_checkpoint').is_dir():paths.append(trial/'route_checkpoint')
    for path in paths:
        if not path.is_dir(): continue
        metadata, arrays = load_checkpoint(path)
        try: require_completed_checkpoint(metadata)
        except ValueError: continue
        if metadata['sequence'].get('terminal_snapshot_publication') is not None:
            from terminal_cycle_checkpoint import verify_publication
            verify_publication(metadata,arrays,path)
        rows.append((metadata['time_s'], path, metadata))
    if not rows: raise ValueError('No explicitly valid completed native cycle checkpoint')
    timestamp, path, metadata = max(rows, key=lambda row: row[0])
    final_time = summary.get('simulated_s')
    if (isinstance(final_time,bool) or not isinstance(final_time, (float, int)) or
            not np.isfinite(final_time) or abs(timestamp-final_time) > 1e-8):
        raise ValueError('Latest valid checkpoint precedes the final physical state; route continuation would roll back')
    _actual_audit_proof(trial,summary,metadata['scene_sha256'])
    return path, metadata


def _actual_audit_proof(trial,summary,scene_sha256):
    """Bind a finished stored-state audit to the actual final native trace."""
    audit_path=Path(trial)/'self_collision_actual.json';states_path=Path(trial)/'states.npz'
    if (summary.get('actual_self_collision_audit_passed') is not True or
            summary.get('model_sha256')!=scene_sha256 or not audit_path.is_file() or
            not states_path.is_file()):
        raise ValueError('Final native state lacks exact completed actual-state audit provenance')
    audit=_read(audit_path);states_sha=file_sha256(states_path)
    if (audit.get('passed') is not True or audit.get('complete') is not True or
            audit.get('scene_sha256')!=scene_sha256 or audit.get('input_sha256')!=states_sha or
            audit.get('physics_rerun') is not False):
        raise ValueError('Final actual-state audit/model/stored-trace identity mismatch')
    return dict(actual_self_audit=str(audit_path.resolve()),actual_self_audit_sha256=file_sha256(audit_path),
                actual_states=str(states_path.resolve()),actual_states_sha256=states_sha)


def validated_initial_checkpoint(assembly, manifest, checkpoint, *, allow_planning_boundary_resume=False):
    """Resume only an exact model's terminal valid native state.

    A completed cycle from a run that subsequently stopped is useful as an
    independent diagnostic fixture, but cannot replace that run's final state
    in a physical campaign. The materialized dependency is part of the model
    identity: different scenes can have identical tiny USDA wrappers.
    """
    if type(allow_planning_boundary_resume) is not bool:
        raise ValueError('Explicit boolean planning-boundary resume policy required')
    assembly, checkpoint=Path(assembly).resolve(),Path(checkpoint).resolve()
    metadata,arrays=load_checkpoint(checkpoint,expected_scene_sha256=manifest['scene_sha256'])
    require_completed_checkpoint(metadata)
    if metadata['sequence'].get('terminal_snapshot_publication') is not None:
        from terminal_cycle_checkpoint import verify_publication
        verify_publication(metadata,arrays,checkpoint)
    if Path(metadata['source_assembly']).resolve()!=assembly:
        raise ValueError('Initial native checkpoint requires the exact source assembly')
    current_dependency=verify_compact_dependencies(assembly,manifest)
    saved_manifest=metadata['manifest']
    saved_dependency=verify_compact_dependencies(assembly,saved_manifest)
    if current_dependency!=saved_dependency or saved_manifest!=manifest:
        raise ValueError('Initial checkpoint/current model or materialized dependency identity mismatch')
    if checkpoint.parent.name=='cycle_checkpoints':
        trial=checkpoint.parent.parent
    elif checkpoint.name=='route_checkpoint':
        trial=checkpoint.parent
    else:
        raise ValueError('Initial checkpoint must retain its native trial/summary provenance')
    summary_path=trial/'summary.json'
    if not summary_path.is_file():raise ValueError('Initial checkpoint requires its final native trial summary')
    summary=_read(summary_path)
    planning_boundary_proof=None
    if (summary.get('completed') is not True or summary.get('stop_reason') is not None or
            summary.get('initialization_valid') is not True or
            summary.get('actual_self_collision_audit_passed') is not True):
        if not allow_planning_boundary_resume:
            raise ValueError('Initial checkpoint parent stopped or was invalid; physical history cannot roll back')
        from planning_boundary_resume import verify_planning_boundary_resume
        planning_boundary_proof=verify_planning_boundary_resume(
            assembly,manifest,checkpoint,metadata,arrays,summary,trial)
    final_time=summary.get('simulated_s')
    if (isinstance(final_time,bool) or not isinstance(final_time,(int,float)) or
            not np.isfinite(final_time) or abs(float(final_time)-metadata['time_s'])>1e-8):
        raise ValueError('Initial checkpoint is not the final measured state; physical history cannot roll back')
    if summary.get('model_sha256')!=manifest['scene_sha256']:
        raise ValueError('Initial checkpoint parent summary/model SHA mismatch')
    audit_proof=_actual_audit_proof(trial,summary,manifest['scene_sha256'])
    rail=manifest['rail'];paths=np.asarray(arrays['body_paths']).astype(str).tolist()
    base=rail['base_body']
    if base not in paths:raise ValueError('Initial checkpoint lacks its measured physical rail base')
    position=np.asarray(arrays['body_poses_world'][paths.index(base)],float)[:3]
    actual_station=float(rail['initial_station_y_m']+position[1]-rail['source_base_position_world_m'][1])
    retained_command=None
    controller=metadata.get('controller_state')
    if controller is not None:
        names=list(map(str,arrays['robot_dof_names']))
        targets=np.asarray(controller['position_targets'],float)
        if (controller.get('position_source')!='native_get_dof_position_targets' or
                controller.get('robot_dof_names')!=names or len(names)!=len(set(names)) or
                rail.get('dof_name') not in names or targets.shape!=(len(names),) or
                not np.isfinite(targets).all()):
            raise ValueError('Initial checkpoint lacks exact retained native rail controller provenance')
        target=float(targets[names.index(rail['dof_name'])])
        retained_command=dict(dof_name=rail['dof_name'],native_position_target_m=target,
            station_y_m=float(rail['initial_station_y_m']+target),
            source='Verified checkpoint native_get_dof_position_targets plus unchanged manifest rail origin',
            checkpoint_metadata_sha256=metadata['metadata_sha256'],
            measured_state_is_not_controller_target=True)
    return metadata,dict(checkpoint=str(checkpoint),checkpoint_metadata_sha256=metadata['metadata_sha256'],
        final_native_summary=str(summary_path),final_native_summary_sha256=file_sha256(summary_path),
        source_assembly=str(assembly),scene_sha256=manifest['scene_sha256'],
        materialized_dependency=current_dependency,time_s=metadata['time_s'],
        measured_initial_station_y_m=actual_station,
        retained_native_rail_command=retained_command,
        **audit_proof,
        **({'planning_boundary_resume':planning_boundary_proof} if planning_boundary_proof is not None else {}),
        verified_final_physical_state=True,rollback_performed=False,
        continuity='Measured native state/controller handoff; PhysX solver caches restart')


def same_plant_resume_station_proof(station_y_m,checkpoint_proof):
    """Allow the retained drive command without replacing measured FK state.

    Tracking error is not a request for new travel. Both comparisons retain the
    existing 1um command-identity tolerance; neither is a physics tolerance.
    """
    measured=checkpoint_proof['measured_initial_station_y_m']
    command=checkpoint_proof.get('retained_native_rail_command')
    retained=None if command is None else command['station_y_m']
    values=[station_y_m,measured]+([] if retained is None else [retained])
    if (checkpoint_proof.get('verified_final_physical_state') is not True or
            any(isinstance(value,bool) or not isinstance(value,(int,float,np.number)) or
                not np.isfinite(value) for value in values)):
        raise ValueError('Same-plant resume requires finite verified measured and retained rail identities')
    measured_error=abs(float(station_y_m)-float(measured))
    command_error=None if retained is None else abs(float(station_y_m)-float(retained))
    if measured_error>1e-6 and (command_error is None or command_error>1e-6):
        raise ValueError('Same-plant native resume with additional physical rail travel is not supported by the executor')
    return dict(requested_station_y_m=float(station_y_m),
        measured_initial_station_y_m=float(measured),retained_command_station_y_m=retained,
        measured_request_difference_m=measured_error,retained_command_request_difference_m=command_error,
        command_identity_tolerance_m=1e-6,
        unchanged_command_proven=command_error is not None and command_error<=1e-6,
        physical_planning_origin='Unchanged actual measured checkpoint base position',
        native_drive_target='Retained checkpoint target; no rail travel command added',
        physical_state_modified=False)


def current_plant_completion_evidence(manifest,inventory,metadata,proof):
    """Skip an empty current plant only with every exact eligible native joint.

    This reports native fruit detachment, not verified collection or harvest
    success. A completed attempt, a count, or a single exhausted truss is not
    evidence that all of the current plant's rail-side fruit was detached.
    """
    if proof.get('verified_final_physical_state') is not True:
        raise ValueError('Current-plant completion requires a verified final physical parent')
    summary_path=Path(proof['final_native_summary'])
    if file_sha256(summary_path)!=proof['final_native_summary_sha256']:
        raise ValueError('Current-plant completion final summary SHA changed')
    summary=_read(summary_path)
    # Real native checkpoints already require this inventory at load. Keep
    # deliberately incomplete diagnostic manifests from acquiring skip credit.
    if not manifest['plant'].get('fruits'):
        return dict(all_eligible_native_detached=False,status='exact_fruit_inventory_unavailable',
                    skipped=False,harvest_success=None,collection_verified=False)
    scope=scoped_trusses(manifest,inventory);fruits=eligible_truss_fruits(manifest,scope)
    if not fruits:raise ValueError('Current active plant has no exact eligible fruit inventory')
    root=manifest['plant']['root'].rstrip('/')
    if any(f['path']!=root+'/'+f['name'] or f['joint']!=root+'/HarvestJoints/'+f['name']for f in fruits):
        raise ValueError('Current eligible fruit/body/native harvest joint identity mismatch')
    names={fruit['name'] for fruit in fruits};joints={fruit['joint']:fruit for fruit in fruits}
    sequence=summary.get('truss_sequence')
    if not isinstance(sequence,dict):raise ValueError('Final current-plant summary lacks native truss inventory')
    recorded=sequence.get('fruit_names');mapping=sequence.get('fruit_joints')
    all_fruits={fruit['name']:fruit['joint'] for fruit in manifest['plant']['fruits']}
    if (not isinstance(recorded,list) or len(set(recorded))!=len(recorded) or
            not set(recorded)<=set(all_fruits) or
            mapping!={name:all_fruits[name] for name in recorded}):
        raise ValueError('Final current-plant selected native fruit/joint identity mismatch')
    events=metadata.get('break_events')
    if not isinstance(events,list) or summary.get('native_break_events')!=events:
        raise ValueError('Checkpoint and final summary native break evidence disagree')
    saved_events=metadata.get('sequence',{}).get('native_break_events')
    if saved_events is not None and saved_events!=events:
        raise ValueError('Checkpoint sequence and native break evidence disagree')
    evidence=[];broken=set()
    for event in events:
        if event.get('joint') not in joints:continue
        original=event
        while original.get('type')=='imported_native_prior':
            prior=original.get('original_event');stamp=original.get('original_t');capture=original.get('source_checkpoint_time_s')
            digest=original.get('source_scene_sha256')
            if (not isinstance(prior,dict) or prior.get('joint')!=event['joint'] or
                    original.get('native_new_event') is not False or
                    stamp!=prior.get('t') or isinstance(stamp,bool) or
                    not isinstance(stamp,(int,float)) or not np.isfinite(stamp) or
                    isinstance(capture,bool) or not isinstance(capture,(int,float)) or
                    not np.isfinite(capture) or not 0<=stamp<=capture or
                    not isinstance(digest,str) or len(digest)!=64 or
                    any(c not in '0123456789abcdef' for c in digest)):
                raise ValueError('Imported native fruit break lacks exact original event/source provenance')
            original=prior
        stamp=original.get('t')
        if (original.get('type') is not None or original.get('native_new_event') is False or
                not isinstance(original.get('phase'),str) or not original['phase'] or
                isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or
                not np.isfinite(stamp) or stamp<0):
            raise ValueError('Eligible fruit detachment has no genuine native callback evidence')
        broken.add(event['joint']);fruit=joints[event['joint']]
        evidence.append(dict(name=fruit['name'],body=fruit['path'],native_joint=fruit['joint'],event=event))
    native_detached={joints[joint]['name'] for joint in broken}
    inv=sequence.get('inventory',{});detached=inv.get('detached');remaining=inv.get('remaining')
    if (not isinstance(detached,list) or not isinstance(remaining,list) or
            len(set(detached))!=len(detached) or len(set(remaining))!=len(remaining) or
            set(detached)&set(remaining) or set(detached)|set(remaining)!=set(recorded)):
        raise ValueError('Final current-plant native inventory is malformed')
    expected_detached={name for name in recorded if all_fruits[name] in
                       {event.get('joint') for event in events}}
    if set(detached)!=expected_detached:
        raise ValueError('Final detached-fruit inventory differs from exact native break events')
    complete=bool(set(recorded)==names and not remaining and set(detached)==names and native_detached==names)
    return dict(schema='s300-exact-current-plant-native-detachment-v1',
        plant_id=scope['plant_id'],source_plant=scope['source_plant'],
        eligible_truss_keys=scope['eligible_truss_keys'],eligible_fruit_names=sorted(names),
        exact_eligible_fruits=[dict(name=f['name'],body=f['path'],native_joint=f['joint'])for f in fruits],
        native_detached_names=sorted(native_detached),eligible_remaining_names=sorted(names-native_detached),
        final_summary_inventory=inv,final_summary_covers_all_eligible_fruits=set(recorded)==names,
        all_eligible_native_detached=complete,skipped=complete,
        status='all_exact_eligible_native_joints_detached' if complete else 'eligible_native_fruits_remain_or_scope_incomplete',
        native_break_evidence=evidence,checkpoint=proof['checkpoint'],
        checkpoint_metadata_sha256=metadata['metadata_sha256'],
        final_native_summary=str(summary_path),final_native_summary_sha256=proof['final_native_summary_sha256'],
        actual_self_audit_sha256=proof['actual_self_audit_sha256'],
        interpretation='Already detached native inventory; no reset, no new harvest or collection credit',
        harvest_success=None,collection_verified=False,accepted_physical_state_changed=False)


def exclude_completed_current_plant(selection,evidence,max_plants):
    """Preserve all proposal/exclusion ledgers and refill only the finite budget."""
    if evidence.get('all_eligible_native_detached') is not True:return selection
    result=dict(selection);source=evidence['plant_id']
    proposals=list(selection['stations'])+[{k:v for k,v in row.items() if k not in ('reason','reachability')}
                                         for row in selection['budget_unselected']]
    completed=[dict(row,reason='all_exact_eligible_native_joints_already_detached',
                    evidence=evidence) for row in proposals if row['plant_id']==source]
    remaining=[row for row in proposals if row['plant_id']!=source]
    result.update(stations=remaining[:max_plants],selected_plant_count=min(max_plants,len(remaining)),
        budget_unselected=[dict(row,reason='physical_campaign_plant_budget',reachability='unknown')
                           for row in remaining[max_plants:]],
        already_detached_current_plant=completed,current_plant_native_completion=evidence)
    return result


def current_plant_budget_deferral_evidence(manifest,inventory,metadata,proof,*,max_attempts):
    """Operationally defer an attached fruit only after its recorded budget.

    This is separate from native detachment/completion. It cannot turn finite
    search exhaustion into geometric impossibility or new harvest credit.
    """
    from truss_sequence import validate_max_fruit_attempts
    max_attempts=validate_max_fruit_attempts(max_attempts)
    native=current_plant_completion_evidence(manifest,inventory,metadata,proof)
    if native.get('all_eligible_native_detached') is True:
        return dict(operationally_deferred=False,status='all_eligible_native_detached',
                    all_eligible_native_detached=True,harvest_success=None)
    if native.get('final_summary_covers_all_eligible_fruits') is not True:
        raise ValueError('Current-plant budget deferral requires the full eligible scope, not one exhausted truss')
    summary=_read(proof['final_native_summary'])
    current=metadata.get('sequence',{});saved=summary['truss_sequence']
    attempts=current.get('attempt_counts')
    if (not isinstance(attempts,dict) or attempts!=saved.get('attempt_counts') or
            isinstance(current.get('max_attempts_per_fruit'),bool) or
            not isinstance(current.get('max_attempts_per_fruit'),int) or
            isinstance(saved.get('max_attempts_per_fruit'),bool) or
            not isinstance(saved.get('max_attempts_per_fruit'),int) or
            current.get('max_attempts_per_fruit')!=max_attempts or
            saved.get('max_attempts_per_fruit')!=max_attempts):
        raise ValueError('Current-plant deferral requires the unchanged native attempt ledger and budget')
    names={fruit['name'] for fruit in manifest['plant']['fruits']}
    if (not set(attempts)<=names or any(isinstance(value,bool) or not isinstance(value,int) or
            not 0<=value<=max_attempts for value in attempts.values())):
        raise ValueError('Current-plant attempt ledger has invalid exact fruit identities or counts')
    remaining=native['eligible_remaining_names']
    exhausted=bool(remaining and all(attempts.get(name,0)>=max_attempts for name in remaining))
    return dict(schema='s300-current-plant-finite-budget-deferral-v1',
        operationally_deferred=exhausted,plant_id=native['plant_id'],source_plant=native['source_plant'],
        status='attached_fruits_finite_budget_exhausted' if exhausted else 'eligible_attempt_budget_remains',
        eligible_fruit_names=native['eligible_fruit_names'],
        native_detached_names=native['native_detached_names'],
        still_attached_fruit_names=remaining,
        still_attached_exact_identities=[row for row in native['exact_eligible_fruits'] if row['name'] in remaining],
        attempt_counts=attempts.copy(),max_attempts_per_fruit=max_attempts,
        checkpoint=proof['checkpoint'],checkpoint_metadata_sha256=metadata['metadata_sha256'],
        final_native_summary=proof['final_native_summary'],
        final_native_summary_sha256=proof['final_native_summary_sha256'],
        actual_self_audit_sha256=proof['actual_self_audit_sha256'],
        all_eligible_native_detached=False,reachability='unknown; finite recorded search budget only',
        prior_native_breaks_preserved=True,attempt_counts_reset=False,
        accepted_physical_state_changed=False,plant_reactivated=False,
        interpretation='Preserve this final physical parent and its attached unknown fruit while considering other row windows; no detachment/completion credit',
        training_eligible=False,harvest_success=None)


def defer_exhausted_current_plant_proposals(selection,evidence,max_plants):
    """Retain exact proposals as deferred evidence, without rebuilding a frozen plant."""
    result=dict(selection);result['current_plant_finite_budget_deferral']=evidence
    if evidence.get('operationally_deferred') is not True:return result
    source=evidence['plant_id']
    proposals=list(selection['stations'])+[{k:v for k,v in row.items() if k not in ('reason','reachability')}
                                         for row in selection['budget_unselected']]
    deferred=[dict(row,reason='attached_fruits_finite_recorded_attempt_budget_exhausted',
                   reachability='unknown; operationally deferred, not native completed',
                   evidence=evidence)for row in proposals if row['plant_id']==source]
    remaining=[row for row in proposals if row['plant_id']!=source]
    result.update(stations=remaining[:max_plants],selected_plant_count=min(max_plants,len(remaining)),
        budget_unselected=[dict(row,reason='physical_campaign_plant_budget',reachability='unknown')
                           for row in remaining[max_plants:]],deferred_current_plant=deferred)
    return result


def blocked_other_trusses_evidence(assembly,manifest,inventory,inventory_path,metadata,proof,declaration_path):
    """Verify an explicit operational deferral without inventing completion.

    SHA-bound historical blocked proposals establish the finite work already
    attempted. They are never a reachability certificate for the current
    deformed terminal state. The caller must still validate the exact final
    physical parent, every new window, and the actual physical rail move.
    """
    assembly=Path(assembly).resolve();declaration_path=Path(declaration_path).resolve()
    declaration=_read(declaration_path)
    native=current_plant_completion_evidence(manifest,inventory,metadata,proof)
    expected=dict(source_assembly=str(assembly),scene_sha256=manifest['scene_sha256'],
        manifest_sha256=file_sha256(assembly/'motion_manifest.json'),
        materialized_dependency=verify_compact_dependencies(assembly,manifest),
        inventory_sha256=file_sha256(inventory_path),checkpoint=str(Path(proof['checkpoint']).resolve()),
        checkpoint_metadata_sha256=metadata['metadata_sha256'],
        checkpoint_json_sha256=file_sha256(Path(proof['checkpoint'])/'checkpoint.json'),
        checkpoint_npz_sha256=file_sha256(Path(proof['checkpoint'])/'checkpoint.npz'),
        final_native_summary_sha256=proof['final_native_summary_sha256'],
        actual_self_audit_sha256=proof['actual_self_audit_sha256'])
    if (declaration.get('schema')!='s300-explicit-blocked-other-trusses-v1' or
            declaration.get('parent')!=expected or
            declaration.get('source_plant')!=manifest['plant']['source_plant'] or
            declaration.get('reachability')!='unknown' or
            declaration.get('plant_complete') is not False or
            declaration.get('training_eligible') is not False or
            declaration.get('harvest_success') is not None):
        raise ValueError('Explicit truss deferral parent/hash/source or noncompletion declaration mismatch')
    completed=declaration.get('completed_trusses');deferred=declaration.get('deferred_trusses')
    if not isinstance(completed,list) or not completed or not isinstance(deferred,list) or not deferred:
        raise ValueError('Explicit truss deferral needs both completed and still-attached trusses')
    eligible=set(native['eligible_truss_keys']);by_key={key:[f for f in native['exact_eligible_fruits']
        if truss_key(f['name'])==key]for key in eligible};detached=set(native['native_detached_names'])
    rows=completed+deferred;keys=[row.get('truss_key')for row in rows]
    if len(set(keys))!=len(keys) or set(keys)!=eligible:
        raise ValueError('Completed/deferred trusses must partition the exact full eligible scope')
    recorded=set(_read(proof['final_native_summary'])['truss_sequence']['fruit_names'])
    for row in completed:
        names={f['name']for f in by_key[row['truss_key']]}
        if (row.get('fruit_names')!=sorted(names) or not names<=detached or not names<=recorded):
            raise ValueError('Completed truss lacks all exact recorded genuine native fruit detachments')
    blocker_proofs=[]
    for row in deferred:
        key=row['truss_key'];names={f['name']for f in by_key[key]};remaining=names-detached
        blockers=row.get('planning_blockers')
        if (not remaining or row.get('still_attached_fruit_names')!=sorted(remaining) or
                row.get('reason')!='finite_planning_budget_blocked' or row.get('reachability')!='unknown' or
                not isinstance(blockers,list) or not 1<=len(blockers)<=64):
            raise ValueError('Deferred truss requires exact attached identities and bounded planning blockers')
        for item in blockers:
            if item.get('state_relation') not in ('exact_parent_model_proposal','separate_historical_diagnostic'):
                raise ValueError('Planning blocker must disclose its relation to the actual terminal state')
            source=Path(item.get('source_assembly','')).resolve();plan=Path(item.get('plan','')).resolve()
            audit=Path(item.get('environment_audit','')).resolve()
            if (not plan.is_file() or not audit.is_file() or
                    item.get('plan_sha256')!=file_sha256(plan) or
                    item.get('environment_audit_sha256')!=file_sha256(audit)):
                raise ValueError('Planning blocker plan/environment audit SHA mismatch')
            source_manifest=_read(source/'motion_manifest.json');dependency=verify_compact_dependencies(source,source_manifest)
            hashes=dict(scene=file_sha256(source/'scene.usda'),manifest=file_sha256(source/'motion_manifest.json'),
                materialized_dependency=None if dependency is None else dependency['sha256'],plan=file_sha256(plan))
            report=_read(audit);proposal=_read(plan);axis=proposal.get('truss_axis',{})
            if (source_manifest['plant']['source_plant']!=manifest['plant']['source_plant'] or
                    source_manifest['scene_sha256']!=hashes['scene'] or item.get('source_sha256')!=hashes or
                    report.get('source_sha256')!=hashes or report.get('allowed_truss_key')!=key or
                    axis.get('truss_key')!=key or not axis.get('fruit_names') or
                    not set(axis['fruit_names'])<=names or
                    report.get('complete') is not True or report.get('passed') is not False or
                    report.get('status')!='blocked' or report.get('native_runs')!=0 or
                    report.get('physics_unchanged') is not True or not report.get('hard_hits')):
                raise ValueError('Planning blocker is incomplete, clear, wrong target, or unproven source mapping')
            if item['state_relation']=='exact_parent_model_proposal' and source!=assembly:
                raise ValueError('Exact-parent blocker uses a different diagnostic model')
            blocker_proofs.append(dict(truss_key=key,**item,
                audit_scope=report.get('scope'),sample_hz=report.get('requested_hz'),
                hard_intersection_sample_pairs=report.get('hard_intersection_sample_pairs'),
                current_terminal_reachability_certified=False))
    return dict(schema='s300-explicit-other-truss-operational-deferral-v1',
        operationally_deferred=True,plant_id=native['plant_id'],source_plant=native['source_plant'],
        completed_trusses=completed,deferred_trusses=deferred,planning_blocker_proofs=blocker_proofs,
        still_attached_fruit_names=native['eligible_remaining_names'],
        still_attached_exact_identities=[f for f in native['exact_eligible_fruits']if f['name']not in detached],
        native_detached_names=native['native_detached_names'],all_eligible_native_detached=False,
        plant_complete=False,reachability='unknown; explicit finite planning deferral, not impossible',
        declaration=str(declaration_path),declaration_sha256=file_sha256(declaration_path),parent=expected,
        accepted_physical_state_changed=False,attempt_counts_reset=False,fabricated_attempt_counts=False,
        native_completion_evidence_unchanged=True,
        window_approximation='Outgoing attached plant becomes static at exact measured deformation; native-detached fruit retains dynamic state',
        interpretation='Proceed to another near-side window while preserving unharvested fruit and its finite blockers; no all-plant completion credit',
        training_eligible=False,harvest_success=None,collection_verified=False)


def defer_blocked_other_truss_proposals(selection,evidence,max_plants):
    """Reuse finite selection refill; leave the original completion ledger alone."""
    result=defer_exhausted_current_plant_proposals(selection,evidence,max_plants)
    result.pop('current_plant_finite_budget_deferral',None)
    result['explicit_other_truss_operational_deferral']=evidence
    for row in result.get('deferred_current_plant',[]):
        row['reason']='explicit_blocked_other_trusses_operational_deferral'
        row['reachability']='unknown; deferred after bounded planning, not native completed'
    return result


def camera_campaign_arguments(base, *, camera_capture=False):
    """Opt in to the existing rendered headless sensor path, preserving defaults.

    The CLI switch adapts the legacy physics-only defaults. Explicit motion
    argument files may instead supply capture-rgbd/capture-gt without
    physics-only; contradictory explicit combinations fail before native work.
    """
    arguments=list(base)
    if camera_capture:
        arguments=[value for value in arguments if value!='--physics-only']
        if '--capture-gt' not in arguments:arguments.append('--capture-gt')
    capture=any(value in arguments for value in ('--capture-rgbd','--capture-gt'))
    if capture:
        if '--physics-only' in arguments:
            raise ValueError('Camera capture requires rendered headless execution; omit --physics-only or use --camera-capture')
        if '--num-envs' in arguments:
            i=arguments.index('--num-envs')
            if i+1>=len(arguments) or arguments[i+1]!='1':
                raise ValueError('Camera capture requires exactly one native environment')
        if any(value.startswith('--num-envs=') and value!='--num-envs=1' for value in arguments):
            raise ValueError('Camera capture requires exactly one native environment')
    return arguments


def validate_camera_scan_proposal_mode(arguments, *, preflight_new_windows, preferred_selection_path):
    """Authorize station proposals only; every fresh execution guard stays on."""
    required={'--camera-harvest','--capture-gt','--headless','--whole-truss',
        '--native-materialized-stage','--verify-full-environment-preflight',
        '--verify-harvest-contact-after-step','--save-route-checkpoint','--checkpoint-cycle-states'}
    if (not preflight_new_windows or preferred_selection_path is None or
            not required.issubset(arguments) or '--physics-only' in arguments or
            '--entry-only' in arguments or '--diagnostic-motion-plan' in arguments):
        raise ValueError('Camera scan-only proposals require explicit GT camera mode, startup preflight and fresh full path/contact/checkpoint guards')
    def value(option,default=None):
        indices=[index for index,item in enumerate(arguments) if item==option]
        if any(item.startswith(option+'=') for item in arguments) or len(indices)>1:
            raise ValueError('Unambiguous camera scan control required: '+option)
        if not indices:return default
        index=indices[0]
        if index+1>=len(arguments) or arguments[index+1].startswith('--'):
            raise ValueError('Missing camera scan control: '+option)
        return arguments[index+1]
    controls={'--mode':'harvest','--preparation':'horizontal_ready',
        '--horizontal-ready-connector':'joint','--preparation-opening':'ready',
        '--harvest-profile':'belt_open_close_cycle'}
    defaults={'--mode':'harvest','--preparation':'cartesian',
        '--horizontal-ready-connector':'cartesian','--preparation-opening':'ready',
        '--harvest-profile':'hook'}
    if any(value(option,defaults[option])!=expected for option,expected in controls.items()):
        raise ValueError('Camera scan-only proposals require the existing horizontal-ready joint/ready belt scan mode')


def validate_target_truss_window(manifest, station=None):
    """Verify existing target-only identities; never author collision settings.

    Every native window still contains the full live plant state. Restricting
    the selected ten fruits does not certify completion of its other trusses.
    """
    plant=manifest.get('plant',{});target=manifest.get('target',{})
    policy=manifest.get('plant_interaction_variant',{})
    root=plant.get('root');source=plant.get('source_plant','')
    if (policy.get('mode')!='target_truss_and_own_stem_only' or not root or
            policy.get('target_root')!=root):
        raise ValueError('Target-truss windows require the existing exact target-truss-and-own-stem collision mode')
    try:
        plant_id=int(source.rsplit('_',1)[1])
        if source!=f'/World/Greenhouse/Plants/Plant_{plant_id:02d}':raise ValueError()
        key=truss_key(target.get('name'))
    except (ValueError,IndexError,TypeError):
        raise ValueError('Invalid target-truss window source/target identity') from None
    matches=[fruit for fruit in plant.get('fruits',[]) if fruit.get('name')==target.get('name')]
    if (policy.get('target_truss')!=key or len(matches)!=1 or
            target.get('path')!=root+'/'+target['name'] or
            target.get('joint')!=root+'/HarvestJoints/'+target['name'] or
            any(matches[0].get(field)!=target.get(field) for field in ('path','joint'))):
        raise ValueError('Target-truss window source/target fruit mapping mismatch')
    if station is not None:
        if (station.get('plant_id')!=plant_id or
                station.get('truss_index')!=int(key.rsplit('_',1)[1]) or
                station.get('target_name')!=target['name']):
            raise ValueError('Selected station disagrees with exact target-truss window source/mapping')
    return dict(source_plant=source,plant_id=plant_id,target_truss=key,
                target_name=target['name'],target_path=target['path'],target_joint=target['joint'],
                source_plant_fruit_count=len(plant.get('fruits',[])),
                entire_plant_completion_claim=False)


def target_truss_camera_scope(manifest, inventory, *, selected_truss=None):
    """Use inventory for identity/aisle filtering, never ghost-truss targets.

    This explicit camera mode narrows the existing near-side scope to the
    exact truss whose colliders are enabled. It authors no physics properties.
    """
    proof=validate_target_truss_window(manifest)
    key=proof['target_truss']
    if selected_truss is not None and selected_truss!=key:
        raise ValueError('Camera selection differs from the collision-enabled target truss')
    scope=scoped_trusses(manifest,inventory)
    exact=[row for row in scope['trusses'] if row['truss_key']==key]
    if len(exact)!=1 or exact[0]['eligible'] is not True:
        raise ValueError('Collision-enabled camera target is outside the authored near-side inventory scope')
    for row in scope['trusses']:
        if row['truss_key']!=key:
            row['eligible']=False
            row['exclusion_reason']='outside_exact_collision_enabled_target_truss'
    scope.update(eligible_truss_keys=[key],eligible_fruit_count=exact[0]['fruit_count'],
        selection='Observed camera group within the exact collision-enabled target truss; inventory verifies identity and aisle side only',
        target_truss_window=proof,physical_collision_policy_changed=False)
    return scope


def target_truss_camera_scan_seed(scope, planning_kin, initial_robot_q):
    """Empty harvest seed and structural row side for a mandatory fresh scan."""
    key=scope['target_truss_window']['target_truss']
    rows=[row for row in scope['trusses'] if row['truss_key']==key and row['eligible'] is True]
    if len(rows)!=1 or scope['eligible_truss_keys']!=[key]:
        raise ValueError('Exact collision-enabled camera scan scope required')
    target=np.array([rows[0]['gutter_center_x'],planning_kin.world[1,3],
        planning_kin.fk(initial_robot_q)[2,3]],dtype=float)
    if not np.isfinite(target).all():raise ValueError('Finite structural row-front scan direction required')
    return dict(fruits=[],truss_key=key,purpose='camera_scan_prefix',harvest_attempt=False,
        harvest_region_selected=False,uses_hidden_gt_center=False),target


def motion_arguments(base, inventory_path, station, *, target_truss_windows=False):
    if type(target_truss_windows) is not bool:raise ValueError('Explicit boolean target-truss window policy required')
    arguments = camera_campaign_arguments(base)
    forbidden = {'--output', '--rail-to-y', '--near-trusses-metadata', '--replicate', '--isolate-background',
                 '--resume-native-checkpoint', '--allow-resume-leaf-proxy-revision',
                 '--allow-resume-accuracy-proxy-revision','--allow-resume-attachment-transfer',
                 '--preparation-prefix-plan'}
    if any(value in forbidden for value in arguments):
        raise ValueError('Campaign-managed arguments or independent reset/fixture options in base arguments')
    required = ['--headless', '--native-materialized-stage', '--whole-truss', '--save-route-checkpoint', '--checkpoint-cycle-states']
    if not any(value in arguments for value in ('--capture-rgbd','--capture-gt')):
        required.append('--physics-only')
    if not all(value in arguments for value in required):
        raise ValueError('Route requires headless whole-truss native measured checkpoint capture')
    for option, value in [('--mode', 'harvest'), ('--plant-dynamics', 'full')]:
        if option not in arguments or arguments[arguments.index(option)+1] != value:
            raise ValueError('Campaign requires '+option+' '+value)
    if 'preparation_variant' in station:
        overrides=preparation_variant_arguments(station['preparation_variant'])
        # Preserve the exact screened diagnostic preparation controls, while
        # every station is still freshly planned/audited on measured geometry.
        # No physical/contact settings can be supplied through this metadata.
        for option in ('--horizontal-ready-connector','--horizontal-ready-seed-index'):
            while option in arguments:
                index=arguments.index(option)
                if index+1>=len(arguments):raise ValueError('Missing base preparation argument value')
                del arguments[index:index+2]
        arguments+=overrides
    scope=[] if target_truss_windows else ['--near-trusses-metadata',str(Path(inventory_path).resolve()),
                                          '--initial-truss-first']
    return arguments+scope+['--rail-to-y',str(station['station_y_m'])]



def validate_automatic_ripe_group_arguments(arguments):
    """Keep the optional SIM-GT selector distinct from live camera selection."""
    if any(token=='--harvest-group-policy' or token.startswith('--harvest-group-policy=')
           for token in arguments):
        raise ValueError('Automatic ripe-group policy cannot replace an explicit source-bound policy')
    if '--camera-harvest' in arguments:
        raise ValueError('Automatic authored-color SIM-GT groups cannot be combined with live camera selection')
    def value(name,default):
        hits=[arguments[i+1] for i,token in enumerate(arguments[:-1]) if token==name]
        if len(hits)>1:raise ValueError('Ambiguous automatic ripe-group control: '+name)
        return hits[0] if hits else default
    if ('--whole-truss' not in arguments or value('--truss-order','lowest-first')!='highest-first' or
            value('--approach-yaw-mode','robot-radial')!='truss-axis' or
            value('--single-fruit-when-remaining','0')!='0' or '--split-unproductive-pairs' in arguments):
        raise ValueError('Automatic ripe-group policy requires existing whole-truss highest-first truss-axis group mode without individual cleanup')


def automatic_ripe_group_arguments(assembly, policy_path, arguments):
    """Prepare the existing policy on this exact actual window, then reverify it.

    This calls the established authored-color extractor and selector loader.
    It neither captures RGB-D nor changes state, collision policy or physics.
    The policy is write-once and belongs only to the current source assembly.
    """
    validate_automatic_ripe_group_arguments(arguments)
    from prepare_harvest_group_policy import prepare
    from harvest_group_policy import load_policy
    assembly,policy_path=Path(assembly).resolve(),Path(policy_path).resolve()
    manifest=_read(assembly/'motion_manifest.json')
    generated=prepare(assembly,policy_path,minimum=2)
    checked=load_policy(policy_path,assembly,manifest['plant']['fruits'])
    if generated!=checked:raise ValueError('Automatic ripe-group policy changed after source extraction')
    proof=dict(schema='s300-per-window-authored-color-group-policy-v1',
        policy=str(policy_path),policy_sha256=file_sha256(policy_path),
        source_assembly=str(assembly),source_plant=manifest['plant']['source_plant'],
        source_sha256=checked['source_sha256'],minimum_fruits=checked['minimum_fruits'],
        eligible_names=[row['name'] for row in checked['fruit_eligibility'] if row['eligible']],
        perception='Existing SIM-GT authored-color eligibility; no new camera observation or learned detector',
        source_hashes_verified=True,
        initial_truss_priority_removed='--initial-truss-first' in arguments,
        selection='Existing highest qualifying ripe group across the exact eligible near-side active-plant trusses',
        training_eligible=False,harvest_success=None)
    grouped_arguments=[token for token in arguments if token!='--initial-truss-first']
    return grouped_arguments+['--harvest-group-policy',str(policy_path)],proof


def startup_preflight_arguments(base,*,physics_only=False):
    """Keep native physics/guards, remove motion options, then run only idle.

    This diagnostic is never a campaign state checkpoint. A passed probe is
    not used to skip initialization or the normal guards on the actual run.
    """
    if type(physics_only) is not bool:raise ValueError('Explicit startup rendering policy required')
    value_options={'--mode','--seconds','--approach-yaw-mode','--approach-pitch-deg',
        '--approach-yaw-offset-deg','--truss-entry-yaw-offset-deg','--harvest-profile','--belt-speed-m-s','--belt-seconds',
        '--close-seconds','--grasp-gap-mm','--grasp-depth-mm','--belt-entry-retract-mm','--later-grasp-depth-mm','--later-grasp-depth-scope',
        '--grasp-height-offset-mm','--belt-ramp-seconds','--belt-infeed-speed-m-s',
        '--preparation-gap-mm','--ready-lift-offset-mm','--horizontal-ready-connector',
        '--horizontal-ready-seed-index','--near-trusses-metadata','--rail-to-y',
        '--resume-native-checkpoint','--camera-max-harvest-cycles',
        '--camera-minimum-fruits','--camera-min-visible-pixels','--entry-retract-mm',
        '--grasp-lateral-offset-mm','--last-fruit-height-offset-mm','--last-fruit-depth-mm',
        '--last-fruit-grasp-sway-mm','--harvest-group-policy','--diagnostic-motion-plan',
        '--release-relief-minimum-gap-mm',
        '--preparation-prefix-plan','--camera-harvest-ready-lift-offset-mm',
        '--camera-prefix-connection','--camera-prefix-solve-seconds',
        '--camera-prefix-proposal-resolution','--camera-prefix-seed'}
    flag_options={'--whole-truss','--initial-truss-first','--preparation-height-first',
        '--measured-ready-start','--checkpoint-cycle-states','--save-route-checkpoint',
        '--split-unproductive-pairs','--belt-off-control','--entry-only',
        '--experimental-tpu-fruit-close-2mm','--release-relief-before-withdraw',
        '--released-fruit-static-farm-support-diagnostic',
        '--allow-resume-leaf-proxy-revision','--allow-resume-accuracy-proxy-revision',
        '--allow-resume-attachment-transfer','--camera-harvest','--horizontal-ready-normalize-turns',
        '--allow-resume-rail-travel','--allow-resume-conditioned-start','--retain-lift-drive-reference',
        '--use-captured-tool-colliders','--group-heading-search','--camera-target-truss-window',
        '--allow-resume-target-truss-revision'}
    variable_options={'--camera-heading-offsets-deg','--camera-ik-seed-indices','--capture-tool-colliders'}
    if physics_only:flag_options|={'--capture-gt','--capture-rgbd','--physics-only'}
    result=[];index=0
    while index<len(base):
        token=base[index]
        if token in value_options:
            if index+1>=len(base) or base[index+1].startswith('--'):
                raise ValueError('Missing startup preflight source option value: '+token)
            index+=2;continue
        if token in flag_options:index+=1;continue
        if token in variable_options:
            first=index+1;index=first
            while index<len(base)and not base[index].startswith('--'):index+=1
            if index==first:raise ValueError('Missing startup preflight source option values: '+token)
            continue
        if token.startswith('--') and token.split('=',1)[0] in value_options|flag_options|variable_options:
            raise ValueError('Use separate source option/value tokens for startup preflight')
        result.append(token);index+=1
    if physics_only:result.append('--physics-only')
    return result+['--mode','idle','--seconds','3']



_NATIVE_TOOL_CAPTURE_PATHS=['/World/ActuatedTool/L/part_209','/World/ActuatedTool/R/part_331']


def native_tool_capture_paths(arguments):
    """Merge explicit existing tool observations with the unchanged two defaults.

    Stable path deduplication includes defaults explicitly repeated by a
    profile. A second option occurrence or equals-form is rejected rather
    than guessing which variable-length request was intended. Native mesh,
    enablement, owner and complete SDK-piece validation remains in the
    existing capture_tool_shapes/load_native_tool_hulls functions.
    """
    option='--capture-tool-colliders';requested=[];seen_option=False
    for index,token in enumerate(arguments):
        if token.startswith(option+'='):
            raise ValueError('Use separate native tool capture option/value tokens')
        if token!=option:continue
        if seen_option:raise ValueError('One explicit native tool capture option required')
        seen_option=True;end=index+1
        while end<len(arguments) and not arguments[end].startswith('--'):end+=1
        values=arguments[index+1:end]
        if not values:raise ValueError('Nonempty explicit native tool capture paths required')
        for path in values:
            if (not path.startswith('/World/ActuatedTool/') or
                    any(part in ('','.','..') for part in path.split('/')[1:])):
                raise ValueError('Exact /World/ActuatedTool/ collider path required: '+path)
        requested+=values
    return list(dict.fromkeys(_NATIVE_TOOL_CAPTURE_PATHS+requested))


def verified_window_native_tool_cache(assembly,cache,*,requested_paths=None):
    """Read the existing exact-source native hull proof, never rebind its SHA."""
    from pxr import Usd
    from native_tool_hulls import load_native_tool_hulls
    _,proof=load_native_tool_hulls(Usd.Stage.Open(str(Path(assembly)/'scene.usda')),cache,assembly=assembly)
    if requested_paths is not None and not set(requested_paths)<=set(proof['scoped_colliders']):
        raise ValueError('Exact-source native tool cache lacks requested collider observations; capture a fresh cache, never rebind it')
    result=dict(cache=str(Path(cache).resolve()),cache_sha256=file_sha256(cache),loader=proof,
        source_hashes_verified=True,diagnostic_native_state_accepted=False,training_eligible=False)
    if requested_paths is not None:result['requested_capture_paths']=list(requested_paths)
    return result


def capture_window_native_tool_cache(assembly,arguments,output,name,benchmark_python,native_python,
                                    timeout,benchmark_runner):
    """Reuse the ordinary3s idle benchmark as an exact-source hull observation."""
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    cases=output/(name+'_cases.json');trials=output/'trials'
    capture_paths=native_tool_capture_paths(arguments)
    args=startup_preflight_arguments(arguments,physics_only=True)+[
        '--capture-tool-colliders',*capture_paths]
    _write(cases,[dict(name=name,arguments=args)])
    command=[str(benchmark_python),'-u',str(Path(__file__).with_name('benchmark_motion.py')),
        str(assembly),str(cases),'--output',str(trials),'--python',str(native_python),
        '--jobs','1','--timeout',str(timeout)]
    runner_error=None
    try:benchmark_runner(command,check=True)
    except subprocess.CalledProcessError as error:runner_error=str(error)
    result_path=trials/(cases.stem+'_results.json')
    result=(_read(result_path)[0] if result_path.is_file() else
        dict(summary=None,returncode=None,timeout=None,runner_error=runner_error))
    probe=preflight_result(result,trials/name)
    _write(output/'probe_result.json',dict(command=command,probe=probe,capture_paths=capture_paths,diagnostic_native_state_accepted=False))
    if not probe['passed']:raise ValueError('Independent native tool capture idle did not pass: '+probe['status'])
    proof=verified_window_native_tool_cache(assembly,trials/name/'native_tool_cooked_capture.json',requested_paths=capture_paths)
    proof['idle_probe']=probe;proof['command']=command
    return proof


def preflight_result(result, trial):
    """Classify an independent no-command initialization/idle probe only."""
    trial=Path(trial);summary=result.get('summary');passed=False
    if isinstance(summary,dict):
        passed=(result.get('timeout') is False and result.get('returncode')==0 and
            summary.get('completed') is True and summary.get('stop_reason') is None and
            summary.get('initialization_valid') is True and summary.get('mode')=='idle' and
            summary.get('rail_destination') is None and
            isinstance(summary.get('simulated_s'),(float,int)) and
            not isinstance(summary.get('simulated_s'),bool) and
            np.isfinite(summary['simulated_s']) and abs(summary['simulated_s']-3.)<1e-8 and
            summary.get('native_new_break_count')==0)
    evidence={}
    for name in ('initialization_failure.json','pre_restore_initialization.json',
                 'native_initialization_timing.json','route_start_native_validation.json',
                 'initialization_validity.json','runtime_error.json'):
        path=trial/name
        if path.is_file():evidence[name]=dict(path=str(path),sha256=file_sha256(path))
    log=trial.with_suffix('.log');log_text=''
    if log.is_file():
        log_text=log.read_text(errors='replace')
        evidence['native_process_log']=dict(path=str(log),sha256=file_sha256(log))
    native_observed=(isinstance(summary,dict)or any(name in evidence for name in (
        'native_initialization_timing.json','route_start_native_validation.json','initialization_validity.json',
        'initialization_failure.json','pre_restore_initialization.json')))
    status='native_idle_preflight_passed'if passed else 'startup_probe_inconclusive'
    can_defer=False
    if not passed:
        if result.get('timeout')is not False:status='startup_probe_timeout_or_unknown'
        elif 'run_motion.py: error:'in log_text:
            status='pipeline_cli_integration_error'
        else:
            initial_failure={}
            if 'initialization_failure.json'in evidence:
                try:initial_failure=_read(trial/'initialization_failure.json')
                except (OSError,ValueError):pass
            explicit_initial_failure=(initial_failure.get('reason')=='new_native_joint_break_during_warmup'or
                isinstance(initial_failure.get('native_new_break_count'),int)and initial_failure['native_new_break_count']>0)
            guard_stops={'initialization_penetration_over_contact_policy','initialization_penetration_over_0_5mm',
                         'penetration_over_contact_policy','penetration_over_0_5mm'}
            idle_guard_failure=(isinstance(summary,dict)and summary.get('mode')=='idle'and
                summary.get('rail_destination')is None and summary.get('stop_reason')in guard_stops)
            idle_topology_failure=(isinstance(summary,dict)and summary.get('mode')=='idle'and
                isinstance(summary.get('native_new_break_count'),int)and summary['native_new_break_count']>0)
            if explicit_initial_failure or idle_guard_failure or idle_topology_failure:
                status='placement_initialization_or_idle_physics_invalid';can_defer=True
            elif 'runtime_error.json'in evidence:status='pipeline_runtime_integration_error'
    return dict(passed=passed,status=status,can_defer_unused_placement=can_defer,
        native_physics_initialization_observed=native_observed,
        result=result,evidence=evidence,accepted_campaign_motion=False,
        diagnostic_break_harvest_credit=0,diagnostic_state_accepted=False,
        interpretation='Independent startup/3s idle probe; failure is not absolute reachability impossibility',
        actual_execution_still_requires_full_initialization_and_guards=True)


def measured_harvested_trusses(summary, plant_source):
    """Count completed belt cycles with new exact selected-fruit joint evidence.

    Scans, merely completed motion, imported breaks and incidental idle falls
    cannot satisfy the optional campaign stopping budget.
    """
    if not summary.get('completed') or summary.get('stop_reason') is not None:
        return []
    result=set()
    for cycle in summary.get('selected_cycle_evidence',{}).get('cycles',[]):
        if (cycle.get('status')!='motion_completed' or
                cycle.get('recorded_physical_cycle') is not True or
                cycle.get('actual_trace_measurements_available') is not True or
                not set(cycle.get('actual_recorded_phases',[])) &
                    {'close','cycle_reclose','belt_spin','belt_finish'}):
            continue
        for fruit in cycle.get('selected_fruits',[]):
            if (fruit.get('native_detached_at_cycle_start') is not False or
                    fruit.get('native_detached_at_cycle_end') is not True):
                continue
            if any(event.get('joint')==fruit.get('native_joint') and
                   event.get('native_new_event') is not False and
                   event.get('type')!='imported_native_prior' and
                   event.get('phase') in {'belt_start_open','close','cycle_reclose','belt_spin','belt_finish',
                                          'cycle_open','cycle_open_hold','cycle_open_final'}
                   for event in fruit.get('new_native_break_events_during_cycle',[])):
                result.add(str(plant_source)+'/'+truss_key(fruit['name']))
    return sorted(result)


def _early_blocked_environment_witness(audit):
    """Known one-collision rejection; unfinished pairs/suffix remain UNKNOWN."""
    if (audit.get('schema')!='s300-rail-environment-preflight-v1' or audit.get('complete')is not False or
            audit.get('stop_on_first_hard_hit')is not True or audit.get('early_terminated')is not True or
            audit.get('termination_reason')!='first_recorded_hard_intersection' or audit.get('error') or
            audit.get('eligible_for_native_execution')is not False or
            type(audit.get('native_runs'))is not int or audit['native_runs']!=0):return None
    hits=audit.get('hard_hits');first=audit.get('first_hard_hit');suffix=audit.get('unexamined_suffix')
    if (not isinstance(hits,list) or not hits or not isinstance(first,dict) or first!=hits[0] or
            not isinstance(suffix,dict) or suffix.get('status')!='UNKNOWN' or
            suffix.get('includes_remaining_pairs_at_first_blocked_sample')is not True or
            'other_collisions_absent'not in suffix or suffix['other_collisions_absent']is not None):return None
    if any(not isinstance(first.get(key),str) or not first[key] for key in
            ('robot_geom','environment_geom','obstacle_class','phase')):return None
    stamp=first.get('time_s');end=suffix.get('nominal_path_end_s')
    if (isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or stamp<0 or
            isinstance(end,bool) or not isinstance(end,(int,float)) or not math.isfinite(end) or end<stamp or
            suffix.get('from_time_s')!=stamp or suffix.get('from_phase')!=first['phase'] or
            audit.get('last_sample_time_s')!=stamp):return None
    if any(isinstance(audit.get(key),bool) or not isinstance(audit.get(key),int) or audit[key]<1
            for key in ('samples','hard_intersection_sample_pairs')):return None
    return dict(first_hard_hit=first,unexamined_suffix=suffix,
        only_current_proposal_rejected=True,environment_audit_complete=False,
        eligible_for_native_execution=False,other_collisions_absent=None)


def unexecuted_planning_failure(result,trial,assembly,*,allow_early_blocked_environment=False):
    """Prove a failed proposal never initialized or stepped native physics.

    Missing evidence, timeouts and unknown errors are not deferrable. A blocked
    proposal can be skipped only while the last accepted parent remains the
    physical state; this function never restores state or edits its artifacts.
    """
    if type(allow_early_blocked_environment)is not bool:return None
    trial=Path(trial);assembly=Path(assembly)
    if (not isinstance(result,dict) or result.get('timeout')is not False or
            isinstance(result.get('returncode'),bool)or result.get('returncode')not in (1,2) or
            result.get('summary')is not None):return None
    forbidden=('summary.json','states.npz','states_partial.npz','partial_trace.json',
        'partial_trace_error.json','native_metadata.json','native_scene_state.json','native_initialization_timing.json',
        'native_initialization_state.npz','invalid_initialization_state.npz','initialization_failure.json',
        'initialization_validity.json','pre_restore_initialization.json','resume_native_validation.json',
        'route_start_native_validation.json','warmup.json','route_checkpoint','cycle_checkpoints','rgbd')
    if any((trial/name).exists()for name in forbidden):return None
    error_path=trial/'runtime_error.json'
    if not error_path.is_file():return None
    try:
        error=_read(error_path).get('error','')
        if not isinstance(error,str)or not error.startswith('Traceback (most recent call last):'):return None
        log=trial.with_suffix('.log')
        if log.is_file()and any(text in log.read_text(errors='replace')for text in (
            '[MOTION] model loaded; initialize native PhysX','[MOTION] views','[MOTION] physics ready')):return None
        manifest=_read(assembly/'motion_manifest.json')
        if file_sha256(assembly/'scene.usda')!=manifest['scene_sha256']:return None
        dependency=verify_compact_dependencies(assembly,manifest)
        evidence=dict(runtime_error=dict(path=str(error_path),sha256=file_sha256(error_path)))
        if log.is_file():evidence['native_log']=dict(path=str(log),sha256=file_sha256(log))
        reason=None
        # These exact finite IK/waypoint failures occur in plan_motion before
        # the runner's native initialize_physics call. Import/setup failures
        # and arbitrary RuntimeErrors remain unknown and stop the campaign.
        prefixes=('RuntimeError: Fixed entry preparation IK not found',
            'RuntimeError: Explicit staging IK seed not feasible:',
            'RuntimeError: Measured ready-to-staging connector IK not found',
            'RuntimeError: Fixed entry approach IK not found',
            'RuntimeError: Planned joint limits exceeded in ',
            'RuntimeError: Level preparation FK orientation check failed:')
        terminal_error=error.strip().splitlines()[-1]if error.strip()else ''
        if ('motion_kinematics.py' in error and 'plan_motion' in error and
                terminal_error.startswith(prefixes)):
            reason='finite_ik_or_waypoint_proposal_rejected'
        plan=trial/'plan.json'
        if plan.is_file():
            plan_sha=file_sha256(plan)
            self_path=trial/'self_collision_preflight.json'
            if self_path.is_file():
                audit=_read(self_path)
                if (audit.get('passed')is False and audit.get('samples',0)>0 and
                        audit.get('scene_sha256')==manifest['scene_sha256']and
                        audit.get('input_sha256')==plan_sha and audit.get('physics_rerun')is False and
                        any(row.get('hits')for row in audit.get('phases',[]))):
                    reason='nominal_self_collision_proposal_blocked'
                    evidence['self_preflight']=dict(path=str(self_path),sha256=file_sha256(self_path))
            for name in ('rail_environment_preflight.json','preparation_environment_preflight.json','full_environment_preflight.json'):
                path=trial/name
                if not path.is_file():continue
                audit=_read(path);hashes=audit.get('source_sha256',{})
                early=(_early_blocked_environment_witness(audit)if allow_early_blocked_environment else None)
                if (audit.get('status')=='blocked'and audit.get('passed')is False and
                        (audit.get('complete')is True or early is not None) and audit.get('native_runs')==0 and
                        audit.get('physics_unchanged')is True and audit.get('hard_hits')and
                        hashes.get('scene')==manifest['scene_sha256']and hashes.get('plan')==plan_sha and
                        hashes.get('manifest')==file_sha256(assembly/'motion_manifest.json')and
                        hashes.get('materialized_dependency')==(None if dependency is None else dependency['sha256'])):
                    reason=('nominal_environment_proposal_blocked_early_witness'if early is not None else
                        'nominal_environment_proposal_blocked')
                    evidence['environment_preflight']=dict(path=str(path),sha256=file_sha256(path))
                    if early is not None:evidence['early_blocked_environment_witness']=early
            if reason:evidence['nominal_plan']=dict(path=str(plan),sha256=plan_sha)
        if reason is None:return None
        return dict(schema='s300-unexecuted-planning-failure-deferral-v1',reason=reason,evidence=evidence,
            proposed_assembly=str(assembly),scene_sha256=manifest['scene_sha256'],
            materialized_dependency=dependency,native_physics_started=False,accepted_physical_state_changed=False,
            checkpoint_rollback=False,harvest_credit=0,training_eligible=False,
            reachability='Finite unused proposal rejected; absolute reachability remains unknown')
    except (OSError,ValueError,KeyError,TypeError):return None


def _heading_option(arguments,option,default=None):
    values=[]
    for index,token in enumerate(arguments):
        if token==option:
            if index+1>=len(arguments) or str(arguments[index+1]).startswith('--'):
                raise ValueError('Missing heading-retry option value: '+option)
            values.append(arguments[index+1])
        elif token.startswith(option+'='):values.append(token.split('=',1)[1])
    if len(values)>1:raise ValueError('Ambiguous heading-retry option: '+option)
    return values[0] if values else default


def station_heading_choice(arguments):
    """Read the actual post-station-override proposal, never the old screen."""
    seed=_heading_option(arguments,'--horizontal-ready-seed-index')
    if seed is not None:
        seed=int(seed)
        if not 0<=seed<=8:raise ValueError('Existing heading retry seed must be0..8')
    heading=float(_heading_option(arguments,'--truss-entry-yaw-offset-deg',0.))
    choice=dict(heading_offset_deg=heading,seed_index=seed)
    alternative_key(choice)
    return choice


def station_heading_alternatives(arguments,nominal_planning=None):
    """At most nine fresh existing yaw/IK proposals; yaw0/seed5 comes first.

    Auto nominal selection is excluded only when the actual plan proves its
    selected branch. Unknown auto selection is retained as unknown, with the
    same fixed nine-proposal cap, rather than inventing a selected seed.
    """
    nominal=station_heading_choice(arguments)
    if nominal['seed_index'] is None and nominal_planning is not None:
        selected=selected_seed_evidence(nominal,nominal_planning)
        nominal=dict(nominal,seed_index=selected['selected_staging_seed_index'])
    choices=camera_plan_alternatives([0.,-30.,30.,-60.,60.],nominal['seed_index'],seed_indices=[8,5])
    choices=[row for row in choices if alternative_key(row)!=alternative_key(nominal)]
    choices.sort(key=lambda row:(alternative_key(row)!=(0.,5),row['index']))
    return choices[:9]


def station_heading_arguments(arguments,alternative):
    """Replace only actual GT entry yaw and existing staging seed controls."""
    heading,seed=alternative_key(alternative)
    if seed is None:raise ValueError('Fresh heading retry requires an explicit existing seed')
    result=list(arguments)
    for option in ('--truss-entry-yaw-offset-deg','--horizontal-ready-seed-index'):
        _heading_option(result,option)
        clean=[];index=0
        while index<len(result):
            if result[index]==option:index+=2
            elif result[index].startswith(option+'='):index+=1
            else:clean.append(result[index]);index+=1
        result=clean
    return result+['--truss-entry-yaw-offset-deg',str(heading),'--horizontal-ready-seed-index',str(seed)]


def _heading_plan_evidence(alternative,trial,assembly):
    manifest=_read(assembly/'motion_manifest.json')
    if file_sha256(assembly/'scene.usda')!=manifest['scene_sha256']:
        raise ValueError('Heading retry source model changed')
    dependency=verify_compact_dependencies(assembly,manifest)
    evidence=dict(alternative_key=list(alternative_key(alternative)),source_assembly=str(assembly),
        scene_sha256=manifest['scene_sha256'],manifest_sha256=file_sha256(assembly/'motion_manifest.json'),
        materialized_dependency=dependency,source_window_and_existing_argument_references_reused=True,
        old_screen_collision_certificate_reused=False,fresh_existing_planner_and_audits_required=True)
    path=trial/'plan.json'
    if not path.is_file():
        evidence.update(plan_available=False,selected_seed_evidence=None)
        return evidence,None
    plan=_read(path)
    evidence.update(plan_available=True,plan=str(path),plan_sha256=file_sha256(path),
        selected_seed_evidence=selected_seed_evidence(alternative,plan))
    actual_heading=float(plan.get('truss_entry_yaw_offset_deg',0.))
    if not np.isfinite(actual_heading) or abs(actual_heading-alternative['heading_offset_deg'])>1.e-9:
        raise ValueError('Fresh GT plan yaw differs from requested heading alternative')
    evidence['actual_plan_truss_entry_yaw_offset_deg']=actual_heading
    evidence['actual_plan_approach_direction_world']=plan.get('approach_direction_world')
    return evidence,plan


def run_station_heading_attempts(command,cases,trials,name,arguments,assembly,output,record,report,
                                benchmark_runner):
    """Reuse benchmark_motion sequentially only at a proven unstepped boundary.

    Source window/startup/cache/policy preparation remains outside this loop.
    Any native initialization, state, actual stop, timeout or unknown failure
    ends it. A successful run returns its actual final state to the ordinary
    completed_final_checkpoint path; no rollback or independent reset occurs.
    """
    pending=[station_heading_choice(arguments)];attempts=record['heading_attempts']=[]
    index=0
    while index<len(pending):
        alternative=pending[index]
        if index:
            heading,seed=alternative_key(alternative)
            attempt_name=name+f'_heading_{index:02d}_yaw{heading:+g}_seed{seed}'
            cases=output/(attempt_name+'_cases.json');trials=output/'trials'/attempt_name
            retry_arguments=station_heading_arguments(arguments,alternative)
            _write(cases,[dict(name=attempt_name,arguments=retry_arguments)])
            command=list(command);command[4]=str(cases)
            command[command.index('--output')+1]=str(trials)
        else:attempt_name=name
        result_path=trials/(cases.stem+'_results.json');trial=trials/attempt_name
        attempt=dict(index=index,nominal=index==0,alternative_key=list(alternative_key(alternative)),
            cases=str(cases),trial=str(trial),result_path=str(result_path),command=command,
            status='benchmark_running',physics_execution='unknown_until_evidence')
        attempts.append(attempt);record.update(command=command,status='native_running',physics_execution=True)
        _write(output/'row_campaign.json',report)
        try:
            try:benchmark_runner(command,check=True)
            except subprocess.CalledProcessError as error:
                attempt['benchmark_controller_error']=str(error)
                if not result_path.is_file():raise
            result=_read(result_path)[0]
            attempt['benchmark_result']=result;record['benchmark_result']=result
            evidence,planning=_heading_plan_evidence(alternative,trial,assembly)
            attempt['fresh_plan_evidence']=evidence
            proof=unexecuted_planning_failure(result,trial,assembly)
            if proof is None:
                attempt.update(status='no_retry_native_or_unknown_result',physics_execution='not_classified_as_unexecuted')
                return result,trial,result_path
            attempt.update(status='proven_unexecuted_planning_failure',physics_execution=False,
                unexecuted_planning_failure=proof)
            record['physics_execution']=False
            if index==0:
                pending+=station_heading_alternatives(arguments,planning)
                record['heading_retry_proposals']=[dict(row,alternative_key=list(alternative_key(row))) for row in pending[1:]]
            _write(output/'row_campaign.json',report)
            index+=1
        except Exception as error:
            attempt.update(status='unknown_or_integration_error_no_retry',error=str(error))
            record['heading_retry_fatal']=True
            raise
    record['heading_retry_budget_exhausted']=True
    return result,trial,result_path


def campaign(assembly, inventory_path, screen_path, output, *, execute=False, max_plants=15,
             base_arguments=None, native_python='/root/isaac-sim-6.1.0/python.sh', timeout=3600.,
             benchmark_python=None,
             benchmark_runner=subprocess.run, window_switch=switch_window,
             preferred_selection_path=None,initial_native_checkpoint=None,preflight_new_windows=False,
             defer_exhausted_current_plant=False,defer_blocked_other_trusses=None,camera_capture=False,
             stop_after_harvested_trusses=0,defer_unexecuted_planning_failures=False,
             allow_camera_scan_only_proposals=False,auto_ripe_group_policy=False,auto_native_tool_cache=False,
             target_truss_windows=False,fast_materialization=False,heading_retries=False,
             allow_planning_boundary_resume=False):
    if type(allow_planning_boundary_resume) is not bool:
        raise ValueError('Explicit boolean planning-boundary resume policy required')
    if allow_planning_boundary_resume and initial_native_checkpoint is None:
        raise ValueError('Planning-boundary resume requires an explicit exact initial native checkpoint')
    if type(heading_retries) is not bool:raise ValueError('Explicit boolean heading retry policy required')
    if type(target_truss_windows) is not bool:raise ValueError('Explicit boolean target-truss window policy required')
    if type(fast_materialization) is not bool:raise ValueError('Explicit boolean exact window materialization required')
    if type(auto_ripe_group_policy) is not bool:raise ValueError('Explicit boolean automatic ripe-group policy required')
    if type(auto_native_tool_cache) is not bool:raise ValueError('Explicit boolean automatic native tool cache required')
    if not isinstance(defer_unexecuted_planning_failures,bool):
        raise ValueError('Explicit boolean unexecuted planning deferral required')
    if (isinstance(stop_after_harvested_trusses,bool) or
            not isinstance(stop_after_harvested_trusses,int) or stop_after_harvested_trusses<0):
        raise ValueError('Nonnegative integer measured harvest stopping budget required')
    assembly, inventory_path, screen_path, output = [Path(p).resolve() for p in (assembly, inventory_path, screen_path, output)]
    if not np.isfinite(timeout) or timeout<=0:
        raise ValueError('Positive finite per-station timeout required')
    benchmark_python=Path(benchmark_python or sys.executable).resolve()
    if execute and not benchmark_python.is_file():raise ValueError('Benchmark Python does not exist: '+str(benchmark_python))
    manifest = _read(assembly/'motion_manifest.json')
    arguments=camera_campaign_arguments(DEFAULT_ARGUMENTS if base_arguments is None else base_arguments,
                                        camera_capture=camera_capture)
    if heading_retries:
        if ('--camera-harvest' in arguments or
                _heading_option(arguments,'--approach-yaw-mode','robot-radial')!='truss-axis'):
            raise ValueError('Heading retries require existing GT truss-axis joint preparation, not camera mode')
    if auto_ripe_group_policy:validate_automatic_ripe_group_arguments(arguments)
    initial_native_tool_cache=None
    requested_native_tool_paths=None
    if auto_native_tool_cache:
        requested_native_tool_paths=native_tool_capture_paths(arguments)
        if not preflight_new_windows or '--physics-only' not in arguments or '--camera-harvest' in arguments:
            raise ValueError('Automatic native tool cache requires GT physics-only execution with independent new-window preflights')
        if '--verify-leaf-contact-after-step' not in arguments:
            raise ValueError('Native tool capture requires the existing solved geometry reader')
        if any(token.startswith('--native-tool-collision-cache=') for token in arguments):
            raise ValueError('Use separate initial native tool cache option/value tokens')
        if '--native-tool-collision-cache' in arguments:
            if arguments.count('--native-tool-collision-cache')!=1:raise ValueError('One exact initial native tool cache required')
            cache_index=arguments.index('--native-tool-collision-cache')
            if cache_index+1>=len(arguments) or arguments[cache_index+1].startswith('--'):raise ValueError('Missing initial native tool cache')
            initial_native_tool_cache=Path(arguments[cache_index+1]).resolve()
            del arguments[cache_index:cache_index+2]
    if not isinstance(allow_camera_scan_only_proposals,bool):raise ValueError('Explicit boolean camera scan proposal policy required')
    if allow_camera_scan_only_proposals:
        validate_camera_scan_proposal_mode(arguments,preflight_new_windows=preflight_new_windows,
            preferred_selection_path=preferred_selection_path)
    capture=any(value in arguments for value in ('--capture-rgbd','--capture-gt'))
    if capture and not manifest.get('mounted_rgbd'):
        raise ValueError('Camera campaign requires the existing mounted_rgbd assembly metadata')
    if file_sha256(assembly/'scene.usda') != manifest['scene_sha256'] or not manifest.get('rail'):
        raise ValueError('Campaign requires immutable physical rail source model')
    verify_compact_dependencies(assembly, manifest)
    initial_target_window=validate_target_truss_window(manifest) if target_truss_windows else None
    initial_checkpoint_proof=None;initial_checkpoint_metadata=None
    if (defer_exhausted_current_plant or defer_blocked_other_trusses is not None) and initial_native_checkpoint is None:
        raise ValueError('Current-plant operational deferral requires a verified final native checkpoint')
    if defer_exhausted_current_plant and defer_blocked_other_trusses is not None:
        raise ValueError('Choose one explicit operational deferral policy; do not mix native exhaustion and planning blockers')
    if initial_native_checkpoint is not None:
        initial_native_checkpoint=Path(initial_native_checkpoint).resolve()
        initial_checkpoint_metadata,initial_checkpoint_proof=validated_initial_checkpoint(assembly,manifest,initial_native_checkpoint,
            **({'allow_planning_boundary_resume':True} if allow_planning_boundary_resume else {}))
    inventory, screen = _read(inventory_path), _read(screen_path)
    if screen.get('inventory_sha256') != file_sha256(inventory_path):
        raise ValueError('Screen/actual farm inventory SHA mismatch')
    source_plant = int(manifest['plant']['source_plant'].rsplit('_', 1)[1])
    actual_initial_station=(manifest['rail']['initial_station_y_m'] if initial_checkpoint_proof is None else
                            initial_checkpoint_proof['measured_initial_station_y_m'])
    pin_current_station=not ('--camera-harvest' in arguments and initial_native_checkpoint is None)
    selection = select_stations(inventory, screen, source_plant, actual_initial_station, max_plants,
                                pin_current_station=pin_current_station)
    if preferred_selection_path is not None:
        preferred_selection_path=Path(preferred_selection_path).resolve()
        prior=[int(row['source_plant'].rsplit('_',1)[1])
               for row in manifest.get('route_checkpoint',{}).get('outgoing_history',[])]
        selection=select_preferred_stations(inventory,screen,_read(preferred_selection_path),screen_path,
            current_plant_id=source_plant,current_station_y_m=actual_initial_station,
            max_plants=max_plants,previous_window_plants=prior,
            allow_camera_scan_only=allow_camera_scan_only_proposals)
    selection['initial_station_policy']=dict(pin_current_station=pin_current_station,
        actual_initial_station_y_m=float(actual_initial_station),
        fresh_camera_scan_proposal_retained=not pin_current_station,
        travel='Existing actuated --rail-to-y command; no state transform or teleport',
        resumed_same_plant_station_unchanged=initial_native_checkpoint is not None)
    if initial_checkpoint_proof is not None:
        completion=current_plant_completion_evidence(manifest,inventory,
            initial_checkpoint_metadata,initial_checkpoint_proof)
        selection=exclude_completed_current_plant(selection,completion,max_plants)
        selection['current_plant_native_completion']=completion
        if defer_exhausted_current_plant:
            source_arguments=arguments
            positions=[index for index,value in enumerate(source_arguments) if value=='--max-fruit-attempts']
            if len(positions)!=1 or positions[0]+1>=len(source_arguments):
                raise ValueError('Current-plant deferral requires one explicit unchanged per-fruit budget')
            token=source_arguments[positions[0]+1]
            if not isinstance(token,str) or token not in ('1','2','3','4','5','6'):
                raise ValueError('Current-plant deferral budget must match the recorded native1..6 policy')
            deferred=current_plant_budget_deferral_evidence(manifest,inventory,
                initial_checkpoint_metadata,initial_checkpoint_proof,max_attempts=int(token))
            selection=defer_exhausted_current_plant_proposals(selection,deferred,max_plants)
        if defer_blocked_other_trusses is not None:
            deferred=blocked_other_trusses_evidence(assembly,manifest,inventory,inventory_path,
                initial_checkpoint_metadata,initial_checkpoint_proof,defer_blocked_other_trusses)
            selection=defer_blocked_other_truss_proposals(selection,deferred,max_plants)
    if target_truss_windows:
        for station in selection['stations']:
            if station['plant_id']==source_plant:validate_target_truss_window(manifest,station)
    output.mkdir(parents=True, exist_ok=False)
    report = dict(schema='s300-physical-rail-row-campaign-v1', selection=selection, executed=execute,
        benchmark_python=str(benchmark_python),native_python=str(native_python),
        initial_assembly=str(assembly), initial_scene_sha256=manifest['scene_sha256'],
        inventory=str(inventory_path), inventory_sha256=file_sha256(inventory_path),
        screen=str(screen_path), screen_sha256=file_sha256(screen_path),
        screen_is_proposal_only=True, old_screen_not_current_collision_certificate=True,
        camera_capture=dict(enabled=capture,gt='--capture-gt' in arguments,
                            rendered_headless=capture,physics_step_control='existing native runner'),
        stations=[], status='planned_only', training_eligible=False, harvest_success=None,
        native_continuity='Measured state handoff with explicit active-window approximation and PhysX restart; rail travel is actuated, never teleported')
    if allow_planning_boundary_resume:
        report['planning_boundary_resume']=dict(enabled=True,
            proof=initial_checkpoint_proof.get('planning_boundary_resume'),
            parent_completion_unchanged=True,earlier_checkpoint_rollback_allowed=False)
    if target_truss_windows:
        report['target_truss_windows']=dict(enabled=True,initial_window=initial_target_window,
            native_selection='Existing manifest target truss only; forced near-trusses selector omitted',
            collision_policy='Existing target_truss_and_own_stem_only asset policy inherited and identity-checked per window',
            source_plant_states='All native body/joint histories retained by existing checkpoint handoff',
            unselected_source_trusses='Unattempted, not whole-plant completion or geometric impossibility',
            physics_and_collision_parameters_changed=False,entire_plant_completion_claim=False)
    if fast_materialization:
        report['window_materialization']=dict(enabled=True,mode='s300-whole-canonical-layer-storage-identity-v1',
            source='Existing route_checkpoint.switch_window fast_materialization opt-in',
            all_authored_fields_checked=True,world_vertex_inventory_recomputed=False,
            geometry_and_physics_parameters_changed=False,fresh_native_validation_still_required=True)
    if auto_native_tool_cache:
        report['automatic_native_tool_cache']=dict(enabled=True,initial_cache=str(initial_native_tool_cache) if initial_native_tool_cache else None,
            capture_paths=requested_native_tool_paths,initial_cache_never_forwarded_to_other_windows=True,
            source='Existing3s independent native idle benchmark and exact immutable-source loader',diagnostic_native_state_accepted=False)
    if auto_ripe_group_policy:
        report['automatic_ripe_group_policy']=dict(enabled=True,minimum_fruits=2,
            source='Existing authored-color extractor and exact-source hash-bound policy loader per actual window',
            camera_selected=False,new_camera_observation_required=False,training_eligible=False)
    if preferred_selection_path is not None:
        report['preferred_selection']=str(preferred_selection_path)
        report['preferred_selection_sha256']=file_sha256(preferred_selection_path)
    if allow_camera_scan_only_proposals:
        report['camera_scan_station_proposal_policy']=dict(enabled=True,
            old_harvest_path_reused=False,old_blocked_source_status_preserved=True,
            fresh_startup_and_initial_scan_audits_required=True,actual_camera_selected_harvest_audits_required=True,
            scope='Station/map scan proposal only; no initial scan, harvest feasibility or success certificate')
    if initial_checkpoint_proof is not None:
        report['initial_native_checkpoint']=initial_checkpoint_proof
    if preflight_new_windows:
        report['new_window_preflight']=dict(enabled=True,mode='idle',seconds=3,
            scope='Independent unused-window startup proposals only; no accepted rail/manipulation rollback',
            failures_preserved=True,passed_probe_never_replaces_actual_execution_state=True)
    if defer_unexecuted_planning_failures:
        report['unexecuted_planning_deferral']=dict(enabled=True,
            scope='Explicit failed planning artifacts before native initialization only; last accepted state remains untouched',
            timeouts_unknowns_initialization_and_actual_stops_never_deferred=True)
    if heading_retries:
        report['heading_retries']=dict(enabled=True,maximum_additional_proposals=9,
            headings_deg=[0.,-30.,30.,-60.,60.],staging_seed_indices=[8,5],
            first_additional_proposal_when_not_nominal=dict(heading_offset_deg=0.,seed_index=5),
            nominal_key_excluded=True,requires_each_previous_proposal_proven_unexecuted=True,
            source_window_startup_cache_and_group_policy_prepared_once=True,
            physics_or_unknown_result_ends_retries=True,physics_and_collision_parameters_changed=False)
    # Validate all command inputs without doing native work in planning mode.
    for station in selection['stations']:
        station_options=motion_arguments(arguments,inventory_path,station,target_truss_windows=target_truss_windows)
        if heading_retries:
            if _heading_option(station_options,'--horizontal-ready-connector','cartesian')!='joint':
                raise ValueError('Heading retries require existing GT truss-axis joint preparation after station overrides')
            station_heading_choice(station_options)
    if (initial_checkpoint_proof is not None and selection['stations'] and
            selection['stations'][0]['plant_id']==source_plant):
        selection['initial_station_policy']['same_plant_resume_station_proof']=same_plant_resume_station_proof(
            selection['stations'][0]['station_y_m'],initial_checkpoint_proof)
    _write(output/'row_campaign.json', report)
    if not execute: return report
    current_assembly = assembly; checkpoint = initial_native_checkpoint
    harvested_trusses=set()
    for index, station in enumerate(selection['stations']):
        parent_assembly,parent_checkpoint=current_assembly,checkpoint
        trial=None;result=None;result_path=None;station_native_cache=None
        record = dict(index=index, **station, status='preparing', physics_execution=False)
        report['stations'].append(record); report['status']='running'
        _write(output/'row_campaign.json', report)
        try:
            current_manifest = _read(current_assembly/'motion_manifest.json')
            current_id = int(current_manifest['plant']['source_plant'].rsplit('_', 1)[1])
            if station['plant_id'] != current_id:
                if checkpoint is None: raise ValueError('Next active window has no validated measured prior physical state')
                next_assembly = output/'windows'/f"{index:03d}_plant_{station['plant_id']:02d}"
                window_options=dict(near_trusses_metadata=inventory_path)
                if fast_materialization:window_options['fast_materialization']=True
                switched = window_switch(current_assembly,checkpoint,next_assembly,
                    station['plant_id'],station['target_name'],**window_options)
                record['window_handoff'] = switched
                if target_truss_windows:
                    record['target_truss_window']=validate_target_truss_window(
                        _read(next_assembly/'motion_manifest.json'),station)
                if preflight_new_windows:
                    probe_name=f"startup_{index:03d}_plant_{station['plant_id']:02d}"
                    probe_cases=output/'startup_preflight'/(probe_name+'_cases.json')
                    probe_cases.parent.mkdir(parents=True,exist_ok=True)
                    probe_source_arguments=motion_arguments(arguments,inventory_path,station,
                        target_truss_windows=target_truss_windows)
                    probe_arguments=startup_preflight_arguments(probe_source_arguments)
                    if auto_native_tool_cache:
                        probe_capture_paths=native_tool_capture_paths(probe_source_arguments)
                        probe_arguments+=['--capture-tool-colliders',*probe_capture_paths]
                        record['startup_native_tool_capture_paths']=probe_capture_paths
                    _write(probe_cases,[dict(name=probe_name,arguments=probe_arguments)])
                    probe_trials=output/'startup_preflight'/probe_name
                    probe_command=[str(benchmark_python),'-u',str(Path(__file__).with_name('benchmark_motion.py')),
                        str(next_assembly),str(probe_cases),'--output',str(probe_trials),
                        '--python',str(native_python),'--jobs','1','--timeout',str(timeout)]
                    record['startup_preflight_command']=probe_command
                    record['status']='independent_startup_preflight_running'
                    _write(output/'row_campaign.json',report)
                    runner_error=None
                    try:benchmark_runner(probe_command,check=True)
                    except subprocess.CalledProcessError as error:runner_error=str(error)
                    probe_result_path=probe_trials/(probe_cases.stem+'_results.json')
                    probe_result=(_read(probe_result_path)[0] if probe_result_path.is_file() else
                        dict(summary=None,returncode=None,timeout=None,runner_error=runner_error))
                    probe=preflight_result(probe_result,probe_trials/probe_name)
                    probe.update(parent_assembly_unchanged=str(current_assembly),
                        parent_checkpoint_unchanged=str(checkpoint),unused_proposal_assembly=str(next_assembly),
                        native_probe_executed=probe['native_physics_initialization_observed'],
                        native_process_launched=True)
                    record['startup_preflight']=probe
                    if not probe['passed']:
                        if not probe['can_defer_unused_placement']:
                            record.update(physics_execution=False,accepted_physical_history_changed=False)
                            raise ValueError('Unused-window startup pipeline failed or is inconclusive; not a scene validity rejection: '+probe['status'])
                        record.update(status='unexecuted_window_startup_excluded',physics_execution=False,
                            reachability='unknown; operational initialization exclusion, not geometric impossibility',
                            accepted_physical_history_changed=False)
                        _write(output/'row_campaign.json',report)
                        continue
                    if auto_native_tool_cache:
                        station_native_cache=verified_window_native_tool_cache(next_assembly,
                            probe_trials/probe_name/'native_tool_cooked_capture.json',requested_paths=probe_capture_paths)
                        record['native_tool_cache']=station_native_cache
                current_assembly = next_assembly
            record.update(assembly=str(current_assembly), actual_scene_sha256=_read(current_assembly/'motion_manifest.json')['scene_sha256'])
            if target_truss_windows:
                record['target_truss_window']=validate_target_truss_window(
                    _read(current_assembly/'motion_manifest.json'),station)
            name = f"station_{index:03d}_plant_{station['plant_id']:02d}"
            cases = output/(name+'_cases.json')
            station_arguments=motion_arguments(arguments,inventory_path,station,target_truss_windows=target_truss_windows)
            if auto_native_tool_cache:
                if station_native_cache is None:
                    station_native_cache=(verified_window_native_tool_cache(current_assembly,initial_native_tool_cache,
                            requested_paths=native_tool_capture_paths(station_arguments))
                        if index==0 and current_assembly.resolve()==assembly and initial_native_tool_cache is not None else
                        capture_window_native_tool_cache(current_assembly,station_arguments,output/'native_tool_capture'/name,
                            name+'_capture',benchmark_python,native_python,timeout,benchmark_runner))
                    record['native_tool_cache']=station_native_cache
                station_arguments+=['--native-tool-collision-cache',station_native_cache['cache']]
            if auto_ripe_group_policy:
                policy_path=output/'group_policies'/(name+'.json')
                station_arguments,policy_proof=automatic_ripe_group_arguments(current_assembly,policy_path,station_arguments)
                record['automatic_ripe_group_policy']=policy_proof
            if index==0 and initial_native_checkpoint is not None and station['plant_id']==current_id:
                rail_index=station_arguments.index('--rail-to-y')
                del station_arguments[rail_index:rail_index+2]
                station_arguments+=['--resume-native-checkpoint',str(initial_native_checkpoint)]
                record['same_plant_initial_native_resume']=initial_checkpoint_proof
                record['initial_rail_command']='Omitted redundant unchanged-station move; exact measured rail state/controller restored'
            _write(cases, [dict(name=name, arguments=station_arguments)])
            trials = output/'trials'/name
            command = [str(benchmark_python), '-u', str(Path(__file__).with_name('benchmark_motion.py')),
                str(current_assembly), str(cases), '--output', str(trials), '--python', str(native_python),
                '--jobs', '1', '--timeout', str(timeout)]
            record.update(command=command, status='native_running', physics_execution=True)
            _write(output/'row_campaign.json', report)
            result_path=trials/(cases.stem+'_results.json');trial=trials/name
            if heading_retries:
                result,trial,result_path=run_station_heading_attempts(command,cases,trials,name,station_arguments,
                    current_assembly,output,record,report,benchmark_runner)
            else:
                benchmark_runner(command, check=True)
                result = _read(result_path)[0]
            record['benchmark_result'] = result
            if not heading_retries:trial = trials/name
            if result.get('timeout') or result.get('returncode') != 0 or result.get('summary') is None:
                raise ValueError('Native station run failed or timed out; route physical history is preserved')
            checkpoint, metadata = completed_final_checkpoint(trial, result['summary'])
            record.update(status='completed_physics', checkpoint=str(checkpoint),
                          checkpoint_metadata_sha256=metadata['metadata_sha256'],
                          native_inventory=result['summary'].get('truss_sequence', {}).get('inventory'),
                          unharvested_interpretation='Finite attempts did not detach these fruits; absolute impossibility is unknown')
            if stop_after_harvested_trusses:
                current_source=_read(current_assembly/'motion_manifest.json')['plant']['source_plant']
                record['measured_harvested_trusses']=measured_harvested_trusses(result['summary'],current_source)
                harvested_trusses.update(record['measured_harvested_trusses'])
                report['measured_harvest_budget']=dict(requested=stop_after_harvested_trusses,
                    achieved=len(harvested_trusses),trusses=sorted(harvested_trusses),
                    scope='New selected-fruit native detachments during completed belt cycles of this campaign only; no collection/whole-truss completion claim')
                if len(harvested_trusses)>=stop_after_harvested_trusses:
                    report.update(status='measured_harvest_budget_reached',rollback_performed=False)
                    _write(output/'row_campaign.json',report)
                    return report
        except Exception as error:
            deferred=None
            if heading_retries and record.get('heading_attempts'):
                last=record['heading_attempts'][-1]
                trial=Path(last['trial']);result_path=Path(last['result_path']);result=last.get('benchmark_result')
            if (defer_unexecuted_planning_failures and parent_checkpoint is not None and trial is not None
                    and not record.get('heading_retry_fatal')):
                if result is None and result_path is not None and result_path.is_file():
                    try:result=_read(result_path)[0]
                    except (OSError,ValueError,KeyError,TypeError,IndexError):pass
                deferred=unexecuted_planning_failure(result,trial,current_assembly)
            if deferred is not None:
                deferred.update(accepted_parent_assembly=str(parent_assembly),
                    accepted_parent_checkpoint=str(parent_checkpoint),
                    executor_input_pointer_restored_only=True)
                record.update(status='unexecuted_planning_failure_deferred',physics_execution=False,
                    native_process_launched=True,blocker_type=type(error).__name__,blocker=str(error),
                    planning_deferral=deferred,accepted_physical_history_changed=False)
                current_assembly,checkpoint=parent_assembly,parent_checkpoint
                _write(output/'row_campaign.json',report)
                continue
            record.update(status='route_blocked', blocker_type=type(error).__name__, blocker=str(error))
            report.update(status='blocked_preserving_last_physical_history', rollback_performed=False)
            _write(output/'row_campaign.json', report)
            return report
        _write(output/'row_campaign.json', report)
    report.update(status=('finite_campaign_no_accepted_motion' if (preflight_new_windows or defer_unexecuted_planning_failures) and
        not any(row.get('physics_execution') for row in report['stations']) else 'finite_campaign_completed'),
        rollback_performed=False)
    _write(output/'row_campaign.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly', type=Path); parser.add_argument('inventory', type=Path)
    parser.add_argument('screen', type=Path); parser.add_argument('output', type=Path)
    parser.add_argument('--execute', action='store_true'); parser.add_argument('--max-plants', type=int, default=15)
    parser.add_argument('--target-truss-windows',action='store_true',
        help='Use existing target-truss-and-own-stem windows without the forced near-trusses selector; other source trusses remain unattempted')
    parser.add_argument('--fast-materialization',action='store_true',
        help='Reuse the existing exact whole-canonical-layer storage proof for new windows; keeps fresh startup/path/native checks')
    parser.add_argument('--heading-retries',action='store_true',
        help='GT only: after proven prephysics failure, try at most9 fresh existing yaw/IK alternatives; any actual/unknown result ends retries')
    parser.add_argument('--camera-patrol',action='store_true',
        help='Use actual camera exposures and a persistent visible-truss queue for bounded forward motion instead of the fixed screen station list')
    parser.add_argument('--patrol-step-m',type=float,default=.15)
    parser.add_argument('--patrol-max-decisions',type=int,default=80)
    parser.add_argument('--patrol-goal-trusses',type=int,default=6)
    parser.add_argument('--patrol-adaptive-lift',action='store_true',
        help='Before passing an observed group, use bounded actual lift scans and fresh camera observations to explore other heights')
    parser.add_argument('--patrol-forward-pass',action='store_true',
        help='Opt-in finite forward pass: one actual failed-view budget per independently exhausted target; preserve untried active groups and deferred rear revisits')
    parser.add_argument('--patrol-prefer-active-truss',action='store_true',
        help='Prefer the current collision-window truss only among existing fresh visible serviceable camera groups; keeps all pending groups')
    parser.add_argument('--patrol-initial-lift-height-m',type=float,
        help='Camera patrol only: first execute the existing physical lift scan at this bounded absolute height, then observe before selecting harvest')
    parser.add_argument('--patrol-prior-campaign',type=Path,
        help='Preserve a prior patrol queue and native harvest ledger at its exact last accepted checkpoint; requires --initial-native-checkpoint')
    parser.add_argument('--patrol-build-failure-proof',type=Path,
        help='Explicit immutable root-cause/fix proof for the known pre-native target-revision build failure only; preserves the exact accepted checkpoint and prior camera queue')
    parser.add_argument('--patrol-bed-x',type=float,
        help='One authored gutter center; default is the current source plant bed')
    parser.add_argument('--motion-arguments', type=Path, help='JSON list of existing run_motion options')
    parser.add_argument('--camera-capture',action='store_true',
        help='Opt in to existing mounted RGB-D/GT capture; replace legacy physics-only with rendered headless execution')
    parser.add_argument('--auto-native-tool-cache',action='store_true',
        help='GT physics-only campaign: reuse existing independent3s idle native tool capture on each exact source window; requires --preflight-new-windows')
    parser.add_argument('--auto-ripe-group-policy',action='store_true',
        help='GT teacher only: generate the existing source-hash-bound ripe-group policy on each actual active window, minimum2 fruits; no camera perception claim')
    parser.add_argument('--stop-after-harvested-trusses',type=int,default=0,
        help='Optional new distinct trusses with selected-fruit native detachment during completed belt cycles; scans/imported history never count')
    parser.add_argument('--defer-unexecuted-planning-failures',action='store_true',
        help='Skip only proven unused IK/self/environment planning failures before native initialization; never skip actual stops or unknown failures')
    parser.add_argument('--preferred-selection',type=Path,
        help='Exact source/query/SHA-checked proposal order; changes no native state or collision settings')
    parser.add_argument('--initial-native-checkpoint',type=Path,
        help='Exact-model completed terminal native state; older cycles from a later failed run are rejected')
    parser.add_argument('--allow-planning-boundary-resume',action='store_true',
        help='GT row only: resume the exact terminal completed boundary of the known planning-only early-negative classifier halt; all final raw states, controls, events and audits must match')
    parser.add_argument('--preflight-new-windows',action='store_true',
        help='Separate3s no-travel idle probe of unused active windows; startup failure preserves unchanged accepted parent state')
    parser.add_argument('--allow-camera-scan-only-proposals',action='store_true',
        help='Explicit camera mode only: accept SHA-bound blocked old harvest proposals as unverified scan stations; all fresh startup/scan/harvest guards remain required')
    parser.add_argument('--defer-exhausted-current-plant',action='store_true',
        help='Opt-in operational deferral of attached fruits with exact final all-eligible native scope and exhausted unchanged attempt ledger; never completion credit')
    parser.add_argument('--defer-blocked-other-trusses',type=Path,
        help='Opt-in SHA-bound completed-truss native proof and explicit bounded blocker ledger for all other eligible trusses; never all-plant completion or impossibility credit')
    parser.add_argument('--python', type=Path, default=Path('/root/isaac-sim-6.1.0/python.sh'))
    parser.add_argument('--benchmark-python',type=Path,
        default=Path(__file__).resolve().parents[2]/'mujoco-benchmark/.venv/bin/python',
        help='Existing Python with psutil for the reused benchmark controller; native physics uses --python separately')
    parser.add_argument('--timeout', type=float, default=3600.)
    args = parser.parse_args()
    if not np.isfinite(args.timeout) or args.timeout <= 0: parser.error('Positive finite per-station timeout required')
    if args.patrol_initial_lift_height_m is not None and not args.camera_patrol:
        parser.error('Initial lift scan height belongs to explicit camera patrol')
    if args.patrol_forward_pass and not (args.camera_patrol and args.target_truss_windows):
        parser.error('Forward pass requires explicit target-only camera patrol')
    if args.patrol_prefer_active_truss and not (args.camera_patrol and args.target_truss_windows):
        parser.error('Active truss cost preference requires explicit target-only camera patrol')
    if args.patrol_build_failure_proof is not None and not (args.camera_patrol and args.target_truss_windows and
            args.patrol_prior_campaign is not None and args.initial_native_checkpoint is not None):
        parser.error('Build failure proof requires target-only camera patrol, exact initial checkpoint and explicit prior campaign')
    if args.camera_patrol and args.auto_native_tool_cache:
        parser.error('Automatic per-window native capture belongs to the GT row campaign')
    if args.camera_patrol and args.fast_materialization:
        parser.error('Exact window materialization belongs to the explicit row campaign, not camera patrol')
    if args.camera_patrol and args.heading_retries:
        parser.error('Heading retries belong to the GT row campaign, not camera patrol')
    if args.camera_patrol and args.allow_planning_boundary_resume:
        parser.error('Planning-boundary resume belongs to the explicit GT row campaign, not camera patrol')
    if args.camera_patrol and args.auto_ripe_group_policy:
        parser.error('Automatic authored-color teacher groups belong to the existing GT row campaign, not camera patrol')
    if args.camera_patrol:
        from camera_patrol_runner import camera_patrol_campaign
        if args.motion_arguments is None:
            parser.error('--camera-patrol requires an explicit existing motion-arguments JSON profile')
        report=camera_patrol_campaign(args.assembly,args.inventory,args.output,
            base_arguments=_read(args.motion_arguments),native_python=args.python,
            benchmark_python=args.benchmark_python,execute=args.execute,
            goal_trusses=args.patrol_goal_trusses,max_decisions=args.patrol_max_decisions,
            step_m=args.patrol_step_m,bed_gutter_center_x=args.patrol_bed_x,
            initial_checkpoint=args.initial_native_checkpoint,prior_campaign=args.patrol_prior_campaign,timeout=args.timeout,
            adaptive_lift=args.patrol_adaptive_lift,target_truss_windows=args.target_truss_windows,
            **({'prefer_active_truss':True} if args.patrol_prefer_active_truss else {}),
            **({'forward_pass':True} if args.patrol_forward_pass else {}),
            build_failure_proof=args.patrol_build_failure_proof,
            **({'initial_lift_height_m':args.patrol_initial_lift_height_m} if args.patrol_initial_lift_height_m is not None else {}))
        print(json.dumps({key:report.get(key) for key in ('status','goal_reached','achieved_distinct_trusses',
            'measured_harvested_trusses','accepted_assembly','accepted_checkpoint','stop_reason')},ensure_ascii=False,indent=2))
        return 0 if report.get('goal_reached') is True or not args.execute else 1
    report = campaign(args.assembly, args.inventory, args.screen, args.output, execute=args.execute,
        max_plants=args.max_plants, base_arguments=_read(args.motion_arguments) if args.motion_arguments else None,
        native_python=args.python,benchmark_python=args.benchmark_python, timeout=args.timeout,
        preferred_selection_path=args.preferred_selection,initial_native_checkpoint=args.initial_native_checkpoint,
        preflight_new_windows=args.preflight_new_windows,defer_exhausted_current_plant=args.defer_exhausted_current_plant,
        defer_blocked_other_trusses=args.defer_blocked_other_trusses,camera_capture=args.camera_capture,
        stop_after_harvested_trusses=args.stop_after_harvested_trusses,
        defer_unexecuted_planning_failures=args.defer_unexecuted_planning_failures,
        allow_camera_scan_only_proposals=args.allow_camera_scan_only_proposals,auto_ripe_group_policy=args.auto_ripe_group_policy,auto_native_tool_cache=args.auto_native_tool_cache,
        target_truss_windows=args.target_truss_windows,fast_materialization=args.fast_materialization,
        heading_retries=args.heading_retries,
        **({'allow_planning_boundary_resume':True} if args.allow_planning_boundary_resume else {}))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if report['status'] in ('blocked_preserving_last_physical_history','finite_campaign_no_accepted_motion') else 0


if __name__ == '__main__': sys.exit(main())
