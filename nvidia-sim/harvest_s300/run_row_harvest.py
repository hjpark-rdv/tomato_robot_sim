"""Finite rail-row campaign using the existing native motion benchmark runner.

Screened station poses are proposals only. Every station is replanned/audited on
its actual current asset. Travel uses the physical rail joint. A new plant
window starts from the latest valid measured native state; invalid later state
never rolls back to an earlier successful checkpoint.
"""
import argparse
import json
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


DEFAULT_ARGUMENTS = ['--headless', '--physics-only', '--native-materialized-stage', '--mode', 'harvest', '--hz', '120',
    '--device', 'cpu', '--performance', 'optimized', '--threads', '2',
    '--contact-backend', 'callback', '--plant-dynamics', 'full',
    '--solver', 'TGS', '--position-iterations', '64', '--velocity-iterations', '4',
    '--scene-ccd', '--fruit-contact-limit-mm', '.5', '--fruit-fruit-limit-mm', '.5',
    '--preparation', 'horizontal_ready', '--measured-ready-start', '--approach-yaw-mode', 'truss-axis',
    '--truss-axis-min-fruits', '5', '--sparse-entry-duration-scale', '4',
    '--single-fruit-when-remaining', '2',
    '--harvest-profile', 'belt_open_close_cycle', '--belt-speed-m-s', '.05',
    '--belt-seconds', '2', '--close-seconds', '6', '--belt-ramp-seconds', '.5',
    '--grasp-gap-mm', '28', '--grasp-depth-mm', '65', '--grasp-height-offset-mm', '-10',
    '--preparation-gap-mm', '44', '--ready-lift-offset-mm', '-150',
    '--preparation-height-first', '--gravity-neutral-warmup', '--armature-setup', 'authored',
    '--released-fruit-ground-diagnostic', '--verify-leaf-contact-after-step',
    '--verify-harvest-contact-after-step', '--verify-background-leaf-enclosures', '--verify-structural-scene-query',
    '--buffer-contact-events', '--whole-truss', '--truss-order', 'lowest-first',
    '--max-truss-cycles', '20', '--max-fruit-attempts', '6', '--max-aim-translation-m', '1.5',
    '--save-route-checkpoint', '--checkpoint-cycle-states']


def _read(path): return json.loads(Path(path).read_text())


def _write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def select_stations(inventory, screen, current_plant_id, current_station_y_m, max_plants=15):
    """Keep every exclusion and unknown, prioritizing finite self-clear proposals."""
    if not 1 <= max_plants <= 33:
        raise ValueError('Finite max-plants budget must be within1..33')
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
        first = min(candidates.pop(current_plant_id), key=lambda q: abs(q['station_y_m']-current_station_y_m))
        first.update(station_y_m=float(current_station_y_m), initial_current_plant=True)
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
                              previous_window_plants=()):
    """Use an explicit, hash-checked order of existing self-clear proposals.

    This changes proposal order only. It neither restores state nor promotes
    old self-collision evidence into a current environment/physics certificate.
    A plant already frozen in an outgoing measured window is not rebuilt from
    its original source, which would reset its physical history.
    """
    screen_path=Path(screen_path).resolve()
    if not isinstance(max_plants,int) or isinstance(max_plants,bool) or not 1<=max_plants<=33:
        raise ValueError('Finite max-plants budget must be within1..33')
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
        if (identity not in placements or identity not in eligible or identity in excluded or
                query is None or query.get('status')!='self_path_clear_full' or
                any(query.get(name) is not True or proposed.get(name) is not True
                    for name in ('full_self_audit_complete','full_self_audit_passed'))):
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
        arguments=preparation_variant_arguments(variant)
        if (preparation_variant_arguments(requested)!=arguments or
                proposed.get('selected_preparation_arguments')!=arguments):
            raise ValueError('Preferred preparation arguments disagree with exact source branch')
        seed=variant.get('horizontal_ready_seed_index')
        if seed is not None and query.get('selected_staging_seed_index')!=seed:
            raise ValueError('Preferred staging branch disagrees with selected native planning seed')
        if query.get('source_report'):
            source=Path(query['source_report']).resolve()
            if (Path(proposed.get('source_report','')).resolve()!=source or not source.is_file() or
                    query.get('source_report_sha256')!=file_sha256(source)):
                raise ValueError('Preferred underlying source report/path/SHA mismatch')
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


def validated_initial_checkpoint(assembly, manifest, checkpoint):
    """Resume only an exact model's terminal valid native state.

    A completed cycle from a run that subsequently stopped is useful as an
    independent diagnostic fixture, but cannot replace that run's final state
    in a physical campaign. The materialized dependency is part of the model
    identity: different scenes can have identical tiny USDA wrappers.
    """
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
    if (summary.get('completed') is not True or summary.get('stop_reason') is not None or
            summary.get('initialization_valid') is not True or
            summary.get('actual_self_collision_audit_passed') is not True):
        raise ValueError('Initial checkpoint parent stopped or was invalid; physical history cannot roll back')
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
    return metadata,dict(checkpoint=str(checkpoint),checkpoint_metadata_sha256=metadata['metadata_sha256'],
        final_native_summary=str(summary_path),final_native_summary_sha256=file_sha256(summary_path),
        source_assembly=str(assembly),scene_sha256=manifest['scene_sha256'],
        materialized_dependency=current_dependency,time_s=metadata['time_s'],
        measured_initial_station_y_m=actual_station,
        **audit_proof,
        verified_final_physical_state=True,rollback_performed=False,
        continuity='Measured native state/controller handoff; PhysX solver caches restart')


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


def motion_arguments(base, inventory_path, station):
    arguments = list(base)
    forbidden = {'--output', '--rail-to-y', '--near-trusses-metadata', '--replicate', '--isolate-background',
                 '--resume-native-checkpoint', '--allow-resume-leaf-proxy-revision',
                 '--allow-resume-accuracy-proxy-revision','--allow-resume-attachment-transfer',
                 '--preparation-prefix-plan'}
    if any(value in forbidden for value in arguments):
        raise ValueError('Campaign-managed arguments or independent reset/fixture options in base arguments')
    required = ['--headless', '--physics-only', '--native-materialized-stage', '--whole-truss', '--save-route-checkpoint', '--checkpoint-cycle-states']
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
    return arguments+['--near-trusses-metadata', str(Path(inventory_path).resolve()),
                     '--initial-truss-first', '--rail-to-y', str(station['station_y_m'])]


def startup_preflight_arguments(base):
    """Keep native physics/guards, remove motion options, then run only idle.

    This diagnostic is never a campaign state checkpoint. A passed probe is
    not used to skip initialization or the normal guards on the actual run.
    """
    value_options={'--mode','--seconds','--approach-yaw-mode','--approach-pitch-deg',
        '--approach-yaw-offset-deg','--harvest-profile','--belt-speed-m-s','--belt-seconds',
        '--close-seconds','--grasp-gap-mm','--grasp-depth-mm','--later-grasp-depth-mm',
        '--grasp-height-offset-mm','--belt-ramp-seconds','--belt-infeed-speed-m-s',
        '--preparation-gap-mm','--ready-lift-offset-mm','--horizontal-ready-connector',
        '--horizontal-ready-seed-index','--near-trusses-metadata','--rail-to-y',
        '--resume-native-checkpoint'}
    flag_options={'--whole-truss','--initial-truss-first','--preparation-height-first',
        '--measured-ready-start','--checkpoint-cycle-states','--save-route-checkpoint',
        '--split-unproductive-pairs','--belt-off-control','--entry-only',
        '--allow-resume-leaf-proxy-revision','--allow-resume-accuracy-proxy-revision',
        '--allow-resume-attachment-transfer'}
    result=[];index=0
    while index<len(base):
        token=base[index]
        if token in value_options:
            if index+1>=len(base) or base[index+1].startswith('--'):
                raise ValueError('Missing startup preflight source option value: '+token)
            index+=2;continue
        if token in flag_options:index+=1;continue
        if token.startswith('--') and token.split('=',1)[0] in value_options|flag_options:
            raise ValueError('Use separate source option/value tokens for startup preflight')
        result.append(token);index+=1
    return result+['--mode','idle','--seconds','3']


def preflight_result(result, trial):
    """Classify an independent no-command initialization/idle probe only."""
    trial=Path(trial);summary=result.get('summary');passed=False
    if isinstance(summary,dict):
        passed=(result.get('timeout') is not True and result.get('returncode')==0 and
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
    return dict(passed=passed,status='native_idle_preflight_passed' if passed else
                'placement_initialization_invalid_or_unknown',
        result=result,evidence=evidence,accepted_campaign_motion=False,
        diagnostic_break_harvest_credit=0,diagnostic_state_accepted=False,
        interpretation='Independent startup/3s idle probe; failure is not absolute reachability impossibility',
        actual_execution_still_requires_full_initialization_and_guards=True)


def campaign(assembly, inventory_path, screen_path, output, *, execute=False, max_plants=15,
             base_arguments=None, native_python='/root/isaac-sim-6.1.0/python.sh', timeout=3600.,
             benchmark_python=None,
             benchmark_runner=subprocess.run, window_switch=switch_window,
             preferred_selection_path=None,initial_native_checkpoint=None,preflight_new_windows=False,
             defer_exhausted_current_plant=False,defer_blocked_other_trusses=None):
    assembly, inventory_path, screen_path, output = [Path(p).resolve() for p in (assembly, inventory_path, screen_path, output)]
    if not np.isfinite(timeout) or timeout<=0:
        raise ValueError('Positive finite per-station timeout required')
    benchmark_python=Path(benchmark_python or sys.executable).resolve()
    if execute and not benchmark_python.is_file():raise ValueError('Benchmark Python does not exist: '+str(benchmark_python))
    manifest = _read(assembly/'motion_manifest.json')
    if file_sha256(assembly/'scene.usda') != manifest['scene_sha256'] or not manifest.get('rail'):
        raise ValueError('Campaign requires immutable physical rail source model')
    verify_compact_dependencies(assembly, manifest)
    initial_checkpoint_proof=None;initial_checkpoint_metadata=None
    if (defer_exhausted_current_plant or defer_blocked_other_trusses is not None) and initial_native_checkpoint is None:
        raise ValueError('Current-plant operational deferral requires a verified final native checkpoint')
    if defer_exhausted_current_plant and defer_blocked_other_trusses is not None:
        raise ValueError('Choose one explicit operational deferral policy; do not mix native exhaustion and planning blockers')
    if initial_native_checkpoint is not None:
        initial_native_checkpoint=Path(initial_native_checkpoint).resolve()
        initial_checkpoint_metadata,initial_checkpoint_proof=validated_initial_checkpoint(assembly,manifest,initial_native_checkpoint)
    inventory, screen = _read(inventory_path), _read(screen_path)
    if screen.get('inventory_sha256') != file_sha256(inventory_path):
        raise ValueError('Screen/actual farm inventory SHA mismatch')
    source_plant = int(manifest['plant']['source_plant'].rsplit('_', 1)[1])
    actual_initial_station=(manifest['rail']['initial_station_y_m'] if initial_checkpoint_proof is None else
                            initial_checkpoint_proof['measured_initial_station_y_m'])
    selection = select_stations(inventory, screen, source_plant, actual_initial_station, max_plants)
    if preferred_selection_path is not None:
        preferred_selection_path=Path(preferred_selection_path).resolve()
        prior=[int(row['source_plant'].rsplit('_',1)[1])
               for row in manifest.get('route_checkpoint',{}).get('outgoing_history',[])]
        selection=select_preferred_stations(inventory,screen,_read(preferred_selection_path),screen_path,
            current_plant_id=source_plant,current_station_y_m=actual_initial_station,
            max_plants=max_plants,previous_window_plants=prior)
    if initial_checkpoint_proof is not None:
        completion=current_plant_completion_evidence(manifest,inventory,
            initial_checkpoint_metadata,initial_checkpoint_proof)
        selection=exclude_completed_current_plant(selection,completion,max_plants)
        selection['current_plant_native_completion']=completion
        if defer_exhausted_current_plant:
            source_arguments=DEFAULT_ARGUMENTS if base_arguments is None else base_arguments
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
    output.mkdir(parents=True, exist_ok=False)
    report = dict(schema='s300-physical-rail-row-campaign-v1', selection=selection, executed=execute,
        benchmark_python=str(benchmark_python),native_python=str(native_python),
        initial_assembly=str(assembly), initial_scene_sha256=manifest['scene_sha256'],
        inventory=str(inventory_path), inventory_sha256=file_sha256(inventory_path),
        screen=str(screen_path), screen_sha256=file_sha256(screen_path),
        screen_is_proposal_only=True, old_screen_not_current_collision_certificate=True,
        stations=[], status='planned_only', training_eligible=False, harvest_success=None,
        native_continuity='Measured state handoff with explicit active-window approximation and PhysX restart; rail travel is actuated, never teleported')
    if preferred_selection_path is not None:
        report['preferred_selection']=str(preferred_selection_path)
        report['preferred_selection_sha256']=file_sha256(preferred_selection_path)
    if initial_checkpoint_proof is not None:
        report['initial_native_checkpoint']=initial_checkpoint_proof
    if preflight_new_windows:
        report['new_window_preflight']=dict(enabled=True,mode='idle',seconds=3,
            scope='Independent unused-window startup proposals only; no accepted rail/manipulation rollback',
            failures_preserved=True,passed_probe_never_replaces_actual_execution_state=True)
    arguments = list(DEFAULT_ARGUMENTS if base_arguments is None else base_arguments)
    # Validate all command inputs without doing native work in planning mode.
    for station in selection['stations']: motion_arguments(arguments, inventory_path, station)
    if (initial_checkpoint_proof is not None and selection['stations'] and
            selection['stations'][0]['plant_id']==source_plant and
            abs(selection['stations'][0]['station_y_m']-
                initial_checkpoint_proof['measured_initial_station_y_m'])>1e-6):
        raise ValueError('Same-plant native resume with additional physical rail travel is not supported by the executor')
    _write(output/'row_campaign.json', report)
    if not execute: return report
    current_assembly = assembly; checkpoint = initial_native_checkpoint
    for index, station in enumerate(selection['stations']):
        record = dict(index=index, **station, status='preparing', physics_execution=False)
        report['stations'].append(record); report['status']='running'
        _write(output/'row_campaign.json', report)
        try:
            current_manifest = _read(current_assembly/'motion_manifest.json')
            current_id = int(current_manifest['plant']['source_plant'].rsplit('_', 1)[1])
            if station['plant_id'] != current_id:
                if checkpoint is None: raise ValueError('Next active window has no validated measured prior physical state')
                next_assembly = output/'windows'/f"{index:03d}_plant_{station['plant_id']:02d}"
                switched = window_switch(current_assembly, checkpoint, next_assembly,
                                          station['plant_id'], station['target_name'],
                                          near_trusses_metadata=inventory_path)
                record['window_handoff'] = switched
                if preflight_new_windows:
                    probe_name=f"startup_{index:03d}_plant_{station['plant_id']:02d}"
                    probe_cases=output/'startup_preflight'/(probe_name+'_cases.json')
                    probe_cases.parent.mkdir(parents=True,exist_ok=True)
                    probe_arguments=startup_preflight_arguments(motion_arguments(arguments,inventory_path,station))
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
                        native_probe_executed=True)
                    record['startup_preflight']=probe
                    if not probe['passed']:
                        record.update(status='unexecuted_window_startup_excluded',physics_execution=False,
                            reachability='unknown; operational initialization exclusion, not geometric impossibility',
                            accepted_physical_history_changed=False)
                        _write(output/'row_campaign.json',report)
                        continue
                current_assembly = next_assembly
            record.update(assembly=str(current_assembly), actual_scene_sha256=_read(current_assembly/'motion_manifest.json')['scene_sha256'])
            name = f"station_{index:03d}_plant_{station['plant_id']:02d}"
            cases = output/(name+'_cases.json')
            station_arguments=motion_arguments(arguments,inventory_path,station)
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
            benchmark_runner(command, check=True)
            result = _read(trials/(cases.stem+'_results.json'))[0]
            record['benchmark_result'] = result
            trial = trials/name
            if result.get('timeout') or result.get('returncode') != 0 or result.get('summary') is None:
                raise ValueError('Native station run failed or timed out; route physical history is preserved')
            checkpoint, metadata = completed_final_checkpoint(trial, result['summary'])
            record.update(status='completed_physics', checkpoint=str(checkpoint),
                          checkpoint_metadata_sha256=metadata['metadata_sha256'],
                          native_inventory=result['summary'].get('truss_sequence', {}).get('inventory'),
                          unharvested_interpretation='Finite attempts did not detach these fruits; absolute impossibility is unknown')
        except Exception as error:
            record.update(status='route_blocked', blocker_type=type(error).__name__, blocker=str(error))
            report.update(status='blocked_preserving_last_physical_history', rollback_performed=False)
            _write(output/'row_campaign.json', report)
            return report
        _write(output/'row_campaign.json', report)
    report.update(status=('finite_campaign_no_accepted_motion' if preflight_new_windows and
        not any(row.get('physics_execution') for row in report['stations']) else 'finite_campaign_completed'),
        rollback_performed=False)
    _write(output/'row_campaign.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly', type=Path); parser.add_argument('inventory', type=Path)
    parser.add_argument('screen', type=Path); parser.add_argument('output', type=Path)
    parser.add_argument('--execute', action='store_true'); parser.add_argument('--max-plants', type=int, default=15)
    parser.add_argument('--motion-arguments', type=Path, help='JSON list of existing run_motion options')
    parser.add_argument('--preferred-selection',type=Path,
        help='Exact source/query/SHA-checked proposal order; changes no native state or collision settings')
    parser.add_argument('--initial-native-checkpoint',type=Path,
        help='Exact-model completed terminal native state; older cycles from a later failed run are rejected')
    parser.add_argument('--preflight-new-windows',action='store_true',
        help='Separate3s no-travel idle probe of unused active windows; startup failure preserves unchanged accepted parent state')
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
    report = campaign(args.assembly, args.inventory, args.screen, args.output, execute=args.execute,
        max_plants=args.max_plants, base_arguments=_read(args.motion_arguments) if args.motion_arguments else None,
        native_python=args.python,benchmark_python=args.benchmark_python, timeout=args.timeout,
        preferred_selection_path=args.preferred_selection,initial_native_checkpoint=args.initial_native_checkpoint,
        preflight_new_windows=args.preflight_new_windows,defer_exhausted_current_plant=args.defer_exhausted_current_plant,
        defer_blocked_other_trusses=args.defer_blocked_other_trusses)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if report['status'] in ('blocked_preserving_last_physical_history','finite_campaign_no_accepted_motion') else 0


if __name__ == '__main__': sys.exit(main())
