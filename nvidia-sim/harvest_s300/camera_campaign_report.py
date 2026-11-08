"""Read existing camera-region/native-harvest results into a bounded report.

No simulator is imported or started. In-progress pilots, planning rejections,
scan prefixes, finite unexecuted windows and measured harvesting stay visible.
"""
import argparse
from collections import Counter
from datetime import datetime
import html
import hashlib
import json
import math
from pathlib import Path
from urllib.parse import quote
from zoneinfo import ZoneInfo

import numpy as np

from report_row_harvest import _native_inventory
from selected_cycle_evidence import MANIPULATION_PHASES


BELT_PHASES = frozenset(('belt_start_open', 'close', 'cycle_reclose', 'belt_spin', 'belt_finish',
                         'cycle_open', 'cycle_open_hold', 'cycle_open_final', 'cycle_open_final_hold'))
EVIDENCE_FILES = ('summary.json', 'runtime_settings.json', 'runtime_error.json', 'initialization_failure.json',
                  'truss_sequence.json', 'camera_region_decisions.json', 'break_events.json', 'states.npz',
                  'code_sha256.json', 'native_stage_selection.json', 'initialization_validity.json',
                  'selected_cycle_evidence.json', 'selected_cycle_evidence_phase_corrected.json',
                  'rgbd/frames.jsonl', 'rgbd/index.html', 'rgbd/camera.json')


def _read(path, warnings, default=None):
    if not path.is_file():
        return default
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError) as error:
        warnings.append(dict(path=str(path), error=str(error)))
        return default


def _relative(path, root):
    try:
        return Path(path).resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return None


def _truss(name):
    return name.split('_Fruit_')[0] if isinstance(name, str) and '_Fruit_' in name else None


def _key(plant, truss):
    return f'{plant}/{truss}' if plant and truss else None


def _phase_trace(folder, warnings):
    path = folder / 'states.npz'
    if not path.is_file():
        return None
    try:
        # NumPy's ZIP loading reads only these small arrays, not all body poses.
        with np.load(path, allow_pickle=False) as state:
            times = state['time']; phases = state['phase']
            cycles = state['cycle_index'] if 'cycle_index' in state else None
        if (times.ndim != 1 or not len(times) or phases.shape != times.shape or
                not np.isfinite(times).all() or np.any(np.diff(times) <= 0) or
                cycles is not None and cycles.shape != times.shape):
            raise ValueError('Measured time/phase/cycle array mismatch')
        return dict(time=times, phase=phases, cycle_index=cycles)
    except (OSError, ValueError, KeyError) as error:
        warnings.append(dict(path=str(path), error=str(error)))
        return None


def _cycle_phases(record, trace, saved):
    if trace is not None:
        times = trace['time']; index = record.get('index')
        if trace['cycle_index'] is not None and isinstance(index, int):
            mask = trace['cycle_index'] == index
            start = record.get('start_s')
            if isinstance(start, (int, float)):
                # The baseline at start belongs to the prior physical step.
                mask &= times > start + 1e-9
        else:
            first = record.get('measured_frame_index'); last = record.get('end_frame_index')
            if not isinstance(first, int) or not isinstance(last, int) or not 0 <= first <= last < len(times):
                return None
            mask = np.zeros(len(times), bool); mask[first + 1:last + 1] = True
        return list(dict.fromkeys(map(str, trace['phase'][mask])))
    row = next((row for row in saved.get('cycles', []) if row.get('index') == record.get('index')), None)
    value = row.get('actual_recorded_phases') if row else None
    return value if isinstance(value, list) else None


def _summarize(trials):
    fields = ('visible_qualifying', 'planned_regions', 'actual_approach', 'actual_belt', 'motion_completed', 'new_native_detached')
    sets = {name: set() for name in fields}
    fruit_keys = set(); cycles = Counter({name: 0 for name in ('scan_prefixes', 'recovery_cycles', 'planning_rejections',
        'accepted_regions', 'actual_approach_cycles', 'actual_belt_cycles', 'completed_harvest_cycles')}); breaks = 0
    for trial in trials:
        for name in fields:
            sets[name].update(trial['truss_keys'][name])
        fruit_keys.update(trial['new_native_fruit_keys'])
        breaks += trial['new_native_break_count']
        for row in trial['cycles']:
            if row.get('recovery'):
                cycles['recovery_cycles'] += 1
            elif row['scan_prefix']:
                cycles['scan_prefixes'] += 1
            elif row['status'] in ('planning_rejected', 'planning_failed'):
                cycles['planning_rejections'] += 1
            elif row['camera_region_selected']:
                cycles['accepted_regions'] += 1
                if row['actual_approach'] is True: cycles['actual_approach_cycles'] += 1
                if row['actual_belt'] is True: cycles['actual_belt_cycles'] += 1
                if row['motion_completed']: cycles['completed_harvest_cycles'] += 1
    return dict(trial_count=len(trials), truss_counts={name: len(keys) for name, keys in sets.items()},
                truss_keys={name: sorted(keys) for name, keys in sets.items()}, cycle_counts=dict(cycles),
                new_native_break_count=breaks, unique_native_detached_fruit_count=len(fruit_keys),
                unique_native_detached_fruit_keys=sorted(fruit_keys),
                six_distinct_trusses_with_new_native_detachment=len(sets['new_native_detached']) >= 6)


def _patrol_chain(path,warnings):
    """Follow only explicit SHA-bound parents; retain partial chains as evidence."""
    chain=[];seen=set();valid=True
    for _ in range(32):
        path=Path(path).resolve()
        if path in seen:
            warnings.append(dict(path=str(path),error='Patrol continuation cycle'));valid=False;break
        seen.add(path);data=_read(path,warnings,{})
        if data.get('schema')!='s300-visible-camera-patrol-campaign-v1':
            warnings.append(dict(path=str(path),error='Unsupported or incomplete patrol parent'));valid=False;break
        chain.append((path,data));parent=data.get('prior_campaign')
        if parent is None:break
        parent_path=Path(parent['root'])/'camera_patrol.json'
        if not parent_path.is_file() or hashlib.sha256(parent_path.read_bytes()).hexdigest()!=parent.get('report_sha256'):
            warnings.append(dict(path=str(parent_path),error='Patrol parent report SHA mismatch; accepted chain goal is unverified'))
            valid=False;break
        path=parent_path
    else:
        warnings.append(dict(path=str(path),error='Patrol continuation exceeds32campaigns'));valid=False
    return list(reversed(chain)),valid


def _patrol_campaign(path,root,trials_by_path,warnings):
    """Count accepted native evidence, keeping scans/probes/failed plans separate."""
    from run_row_harvest import measured_harvested_trusses
    chain,valid=_patrol_chain(path,warnings);data=chain[-1][1] if chain else {}
    windows=[];accepted_paths=set();credited=set();seen_trials=set();native_fruit_keys=set()
    counts=Counter();members=[];chain_paths=[]
    for report_path,campaign in chain:
        chain_paths.append(_relative(report_path,root) or str(report_path))
        for record in campaign.get('segments',[]):
            decision=record.get('decision',{});action=decision.get('action');status=record.get('status')
            declared=record.get('trial')
            trial_path=Path(declared).resolve() if declared else None
            trial=trials_by_path.get(trial_path)
            duplicate=trial_path in seen_trials if trial_path else False
            if trial_path:seen_trials.add(trial_path)
            summary=_read(trial_path/'summary.json',warnings,{}) if trial_path else {}
            result=record.get('benchmark_result',{});assembly=record.get('assembly')
            manifest=_read(Path(assembly)/'motion_manifest.json',warnings,{}) if assembly else {}
            source=manifest.get('plant',{}).get('source_plant')
            complete=(status=='completed_physics' and result.get('returncode')==0 and
                result.get('timeout') is False and summary.get('completed') is True and
                summary.get('stop_reason') is None and trial is not None and trial['measured_trace_available'])
            native=[]
            if complete and not duplicate:
                accepted_paths.add(trial_path);members.append(trial)
                # Selected native cycle evidence and exact measured phase/break
                # mapping must agree. Neither a plan nor the goal flag counts.
                helper=set(measured_harvested_trusses(summary,source)) if source else set()
                native=sorted(helper & set(trial['truss_keys']['new_native_detached'])) if action=='harvest' else []
                credited.update(native)
                if native:native_fruit_keys.update(trial['new_native_fruit_keys'])
            startup=record.get('startup_preflight');cache=record.get('startup_probe_cache',{})
            if startup and not duplicate:
                counts['startup_diagnostics']+=1
                counts['startup_failures']+=int(startup.get('passed') is not True)
                counts['startup_cache_reuses']+=int(cache.get('actual_new_execution') is False)
            if not duplicate:
                counts['decisions']+=1
                if action=='move':counts['move_decisions']+=1
                counts['initial_scans' if action=='scan' and decision.get('reason')=='initial_actual_camera_observation_required'
                    else 'observation_scans' if action=='scan' else 'physical_move_segments' if action=='move' and complete
                    else 'harvest_decisions' if action=='harvest' else 'other_decisions']+=1
                counts['accepted_actual_harvest_segments']+=int(action=='harvest' and complete)
                counts['unused_window_startup_deferred']+=int(status=='unused_window_startup_deferred')
                counts['unexecuted_planning_rejections']+=int(status=='unexecuted_finite_planning_proposal_rejected')
                counts['actual_physics_stops']+=int(status=='actual_physics_stopped')
            recorded=bool(trial and trial['measured_trace_available'] and (trial['simulated_s'] or 0)>0)
            windows.append(dict(index=record.get('index'),campaign=_relative(report_path,root) or str(report_path),
                action=action,target_key=decision.get('target_key'),
                current_station_y_m=decision.get('current_station_y_m'),
                requested_station_y_m=decision.get('next_station_y_m'),
                measured_station_y_m=record.get('measured_station_y_m'),
                status=status,blocker=record.get('blocker') or record.get('stop_reason'),
                decision_reason=decision.get('reason'),decision=decision,
                trial_path=_relative(trial_path,root) if trial_path else None,
                recorded_physics_execution=recorded,accepted_completed_physics=complete,
                duplicate_trial_reference=duplicate,credited_native_trusses=native,
                startup_preflight=startup,startup_probe_cache=cache,
                startup_is_diagnostic_only=True,startup_is_camera_capture=False,
                window_handoff=record.get('window_handoff'),planning_deferral=record.get('planning_deferral'),
                checkpoint=record.get('checkpoint')))
    goal=data.get('goal_distinct_trusses',6)
    if isinstance(goal,bool) or not isinstance(goal,int) or goal<1:goal=6;valid=False
    met=valid and len(credited)>=goal
    ongoing=data.get('status') in ('running','native_running','preparing') or str(data.get('status','')).startswith('coordinator_paused')
    ledger=_read(Path(path).parent/'camera_patrol_queue.json',warnings,{})
    queue=ledger.get('queue',{}) if isinstance(ledger,dict) else {}
    if not isinstance(queue,dict):queue={}
    queue_rows=[dict(key=key,status=item.get('status'),source_plant=item.get('source_plant'),
        service_station_y_m=item.get('service_station_y_m'),visible_now=item.get('visible_now'),
        observed_attached_fruit_keys=item.get('observed_attached_fruit_keys',[]),
        blockers=item.get('blockers',[]),planning_failures=item.get('planning_failures',[]),
        position_source=item.get('target_position_source')) for key,item in queue.items()]
    return dict(schema='s300-camera-patrol-report-v1',path=_relative(path,root) or str(path),
        status=data.get('status'),executed=data.get('executed'),
        in_progress=ongoing,
        explicit_label=('IN PROGRESS · 실제 목표 미검증/미완료' if ongoing else '목표 미완료') if not met else
            'Native 증거 기준 목표 충족 · 수거 성공 미검증',
        native_continuity=data.get('native_continuity'),continuation_chain=chain_paths,
        continuation_chain_valid=valid,finite_decisions=windows,classification_counts=dict(counts),
        accepted_actual_trial_paths=sorted(_relative(p,root) or str(p) for p in accepted_paths),
        achieved_distinct_trusses_from_native_evidence=len(credited),verified_native_trusses=sorted(credited),
        goal_distinct_trusses=goal,goal_verified_from_native_evidence=met,
        visible_camera_queue=dict(ever_queued_truss_count=len(queue_rows),rows=queue_rows,
            statuses=dict(Counter(row['status'] for row in queue_rows)),
            counts_are_observations_not_reachability_or_harvest=True),
        declared_goal_reached=data.get('goal_reached'),declared_native_trusses=data.get('measured_harvested_trusses',[]),
        declared_counts_agree=set(data.get('measured_harvested_trusses',[]))==credited,
        accepted_native_detached_fruit_keys=sorted(native_fruit_keys),actual_result_summary=_summarize(members),
        camera_service_calibration=data.get('camera_service_calibration'),
        prior_campaign=data.get('prior_campaign'),source_report_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())


def collect(root):
    root = Path(root).resolve(); warnings = []; trials = []
    folders = set()
    for filename in ('summary.json','runtime_settings.json', 'runtime_error.json', 'initialization_failure.json', 'truss_sequence.json'):
        folders.update(path.parent for path in root.rglob(filename) if 'code' not in path.relative_to(root).parts)
    for folder in sorted(folders):
        summary = _read(folder / 'summary.json', warnings, {})
        # Idle native captures legitimately have no harvesting sequence. Both
        # an embedded JSON null and a standalone null mean no sequence; they
        # must remain visible without acquiring fabricated harvest evidence.
        sequence = _read(folder / 'truss_sequence.json', warnings, summary.get('truss_sequence') or {})
        if sequence is None:
            sequence = {}
        runtime = _read(folder / 'runtime_settings.json', warnings, {})
        error = _read(folder / 'runtime_error.json', warnings)
        native = _read(folder / 'native_stage_selection.json', warnings, {})
        source = summary.get('scene') or native.get('source_scene') or runtime.get('native_scene')
        manifest = _read(Path(source).parent / 'motion_manifest.json', warnings, {}) if source else {}
        plant = manifest.get('plant', {}).get('source_plant')
        decisions = _read(folder / 'camera_region_decisions.json', warnings, [])
        if not isinstance(decisions, list):
            warnings.append(dict(path=str(folder / 'camera_region_decisions.json'), error='Expected decision list'))
            decisions = []
        if not plant:
            sources = {row.get('camera_region', {}).get('plant_source') for row in decisions} - {None}
            if len(sources) == 1: plant = next(iter(sources))
        trace = _phase_trace(folder, warnings)
        last_time = float(trace['time'][-1]) if trace is not None else summary.get('simulated_s')
        breaks = _read(folder / 'break_events.json', warnings,
                       summary.get('native_break_events', sequence.get('native_break_events', [])))
        mode = summary.get('mode') or ('harvest' if sequence.get('fruit_joints') else None)
        inventory = _native_inventory(sequence, manifest, breaks, mode, last_time)
        mapping = sequence.get('fruit_joints', {})
        reverse = {joint: name for name, joint in mapping.items()} if inventory and inventory.get('exact_mapping_valid') else {}
        fresh = {}; excluded_breaks = []
        for event in breaks:
            stamp = event.get('t'); joint = event.get('joint')
            valid_time = (not isinstance(stamp, bool) and isinstance(stamp, (int, float)) and math.isfinite(stamp) and
                          stamp >= 0 and (last_time is None or stamp <= last_time + 1e-9))
            if (not valid_time or joint not in reverse or event.get('native_new_event') is False or
                    event.get('type') == 'imported_native_prior' or event.get('phase') in ('initial', 'idle', 'settle', 'warmup')):
                excluded_breaks.append(event); continue
            fresh.setdefault(joint, event)
        selected = _read(folder / 'selected_cycle_evidence_phase_corrected.json', warnings,
                         _read(folder / 'selected_cycle_evidence.json', warnings, {}))
        truss_keys = {name: set() for name in ('visible_qualifying', 'planned_regions', 'actual_approach',
                                             'actual_belt', 'motion_completed', 'new_native_detached')}
        for decision in decisions:
            region = decision.get('camera_region', {})
            observed = set(region.get('observable_eligible_fruits', []))
            truss_keys['visible_qualifying'].update(filter(None, (_key(plant, _truss(name)) for name in observed)))
        cycles = []
        for record in sequence.get('cycle_plans', []):
            recovery = mode == 'recovery' or record.get('purpose') == 'terminal_preparation_recovery'
            scan = not recovery and (record.get('purpose') == 'camera_scan_prefix' or record.get('harvest_attempt') is False)
            names = record.get('fruits', []); exact_trusses = {_truss(name) for name in names} - {None}
            truss = next(iter(exact_trusses)) if len(exact_trusses) == 1 else None
            key = _key(plant, truss)
            status = record.get('status')
            phases = None if status in ('planning_rejected', 'planning_failed') else _cycle_phases(record, trace, selected)
            accepted = not scan and not recovery and bool(names) and status in ('executing', 'motion_completed', 'stopped')
            actual_approach = 'approach' in phases if phases is not None else None
            actual_belt = bool(set(phases) & BELT_PHASES) if phases is not None else None
            completed = accepted and status == 'motion_completed' and bool(set(phases or []) & MANIPULATION_PHASES)
            if key:
                for label, condition in (('planned_regions', accepted and bool(record.get('camera_region'))), ('actual_approach', actual_approach is True),
                                         ('actual_belt', actual_belt is True), ('motion_completed', completed)):
                    if condition: truss_keys[label].add(key)
            region = record.get('camera_region', {})
            cycles.append(dict(index=record.get('index'), status=status, scan_prefix=scan, recovery=recovery, fruits=names,
                truss_key=truss, exact_source_truss=key, plan_accepted=accepted,
                camera_region_selected=bool(region) and accepted, actual_recorded_phases=phases,
                actual_approach=actual_approach, actual_belt=actual_belt, motion_completed=completed,
                start_s=record.get('start_s'), end_s=record.get('end_s'), error=record.get('error'),
                stop_reason=record.get('stop_reason'), selection_observation=region.get('observation'),
                accepted_observation_count=len(region.get('accepted_observations', [])),
                target_aim_world_m=record.get('aim_world_m')))
        new_names = sorted(reverse[joint] for joint in fresh)
        manipulation_breaks = []
        for joint, event in fresh.items():
            name = reverse[joint]; key = _key(plant, _truss(name)); phase = event.get('phase')
            if phase is None and trace is not None:
                index = int(np.searchsorted(trace['time'], event['t']))
                if index < len(trace['time']) and abs(trace['time'][index] - event['t']) <= 1e-8:
                    phase = str(trace['phase'][index])
            if key and phase in MANIPULATION_PHASES and key in truss_keys['planned_regions']:
                truss_keys['new_native_detached'].add(key)
                manipulation_breaks.append(event)
        evidence = {name: _relative(folder / name, root) for name in EVIDENCE_FILES if (folder / name).is_file()}
        for log in (folder.with_suffix('.log'), folder.parent / (folder.name + '.log')):
            if log.is_file(): evidence['log'] = _relative(log, root)
        videos = []
        for video in sorted(folder.rglob('*.mp4')):
            if 'code' in video.relative_to(folder).parts: continue
            metadata = _read(video.with_name(video.stem + '_render.json'), warnings, {})
            videos.append(dict(path=_relative(video, root),
                source='recorded_sensor_frames' if video.parent.name == 'rgbd' else
                       'actual_saved_body_transforms' if metadata.get('source') == 'stored measured physical body transforms' else 'unclassified_video',
                metadata=_relative(video.with_name(video.stem + '_render.json'), root) if metadata else None))
        capture = _read(folder / 'rgbd/camera.json', warnings, {})
        trials.append(dict(path=_relative(folder, root), source_plant=plant,
            status='complete' if summary.get('completed') else 'stopped' if summary else 'runtime_error' if error else 'not_finalized',
            completed=summary.get('completed'), stop_reason=summary.get('stop_reason'), runtime_error=error,
            simulated_s=last_time, physics_wall_s=summary.get('physics_wall_s'),
            measured_trace_available=trace is not None,
            contact_limits_mm={name: runtime.get(name) for name in ('contact_limit_mm', 'fruit_contact_limit_mm', 'fruit_fruit_limit_mm')},
            hz=runtime.get('hz'), native_scene_sha256=runtime.get('native_scene_sha256'),
            source_scene_sha256=runtime.get('source_scene_sha256'),
            camera_frames=capture.get('frames'), camera_decision_count=len(decisions),
            sequence_termination=sequence.get('termination'), cycles=cycles, evidence=evidence, videos=videos,
            truss_keys={name: sorted(values) for name, values in truss_keys.items()},
            new_native_break_count=len(fresh), new_native_break_events=list(fresh.values()),
            manipulation_native_break_count=len(manipulation_breaks),
            manipulation_native_break_events=manipulation_breaks,
            new_native_fruit_names=new_names, new_native_fruit_keys=[f'{plant}/{name}' for name in new_names if plant],
            excluded_or_imported_break_events=excluded_breaks, native_inventory=inventory))
    campaigns = []
    for path in sorted(root.rglob('row_campaign.json')):
        data = _read(path, warnings, {})
        if data.get('schema') != 's300-physical-rail-row-campaign-v1': continue
        actual = {row.get('index'): row for row in data.get('stations', [])}
        prefix = _relative(path.parent / 'trials', root) + '/'
        members = [trial for trial in trials if trial['path'].startswith(prefix)]
        windows = []
        for index, proposal in enumerate(data.get('selection', {}).get('stations', [])):
            measured = actual.get(index, {})
            plant_id = proposal.get('plant_id')
            station_prefix = prefix + f'station_{index:03d}_plant_{plant_id:02d}/' if isinstance(plant_id, int) else None
            matching = [trial for trial in members if station_prefix and trial['path'].startswith(station_prefix)]
            recorded = True if any(trial['measured_trace_available'] and (trial['simulated_s'] or 0) > 0 for trial in matching) else None if measured.get('physics_execution') else False
            windows.append(dict(index=index, plant_id=proposal.get('plant_id'), truss_index=proposal.get('truss_index'),
                path=proposal.get('path'), station_y_m=proposal.get('station_y_m'),
                status=measured.get('status', 'not_executed'), physics_execution=measured.get('physics_execution', False),
                declared_physics_execution=measured.get('physics_execution', False), recorded_physics_execution=recorded,
                blocker=measured.get('blocker'), window_handoff=measured.get('window_handoff')))
        campaigns.append(dict(path=_relative(path, root), status=data.get('status'), executed=data.get('executed'),
            native_continuity=data.get('native_continuity'), screen_is_proposal_only=data.get('screen_is_proposal_only'),
            finite_windows=windows, not_executed_windows=sum(row['recorded_physics_execution'] is False for row in windows),
            launched_but_recorded_physics_unknown_windows=sum(row['recorded_physics_execution'] is None for row in windows),
            actual_result_summary=_summarize(members)))
    by_path={(root/trial['path']).resolve():trial for trial in trials}
    patrols=[_patrol_campaign(path,root,by_path,warnings) for path in sorted(root.rglob('camera_patrol.json'))
             if 'code' not in path.relative_to(root).parts]
    accepted_patrol_paths={path for patrol in patrols for path in patrol['accepted_actual_trial_paths']}
    accepted_patrol_trusses={key for patrol in patrols if patrol['continuation_chain_valid'] for key in patrol['verified_native_trusses']}
    return dict(schema='s300-camera-harvest-campaign-report-v1', root=str(root),
        generated_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
        interpretation='Visible GT region proposals, measured execution and exact native joint breaks are separate. Finite failure is not impossibility.',
        training_eligible=False, harvest_success=None, collection_verified=False,
        trials=trials, campaigns=campaigns,patrol_campaigns=patrols,
        accepted_patrol_chains=dict(unique_actual_trial_paths=sorted(accepted_patrol_paths),
            unique_native_trusses=sorted(accepted_patrol_trusses),distinct_native_truss_count=len(accepted_patrol_trusses),
            duplicate_parent_trial_references_counted_once=True,
            combines_independent_experiments_for_goal=False),
        totals_all_experiments=_summarize(trials), warnings=warnings)


def write_report(root):
    display_root = Path(root).absolute()
    root = Path(root).resolve(); data = collect(root)
    # Publication may be useful before the requested goal is reached. Keep the
    # accepted-route audit separate from totals that include independent tests.
    goal = _read(root / 'final_web_drafts/goal_audit.json', data['warnings'], {})
    if goal:
        data['accepted_route_goal_audit'] = goal
        data['accepted_route_native_detachments'] = sum(
            (row.get('new_native_break_count') or 0) for row in goal.get('per_run', [])
            if row.get('actual_route_history') and row.get('credited_trusses'))
    esc = lambda value: html.escape(str(value))
    def link(path, label=None):
        return f'<a href="{quote(path, safe="/")}">{esc(label or path)}</a>' if path else ''
    fragments = ['<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">',
        '<title>카메라 영역 기반 수확 · 실행 기록</title><style>body{font:14px/1.6 system-ui;background:#14201b;color:#e9f2eb;margin:28px auto;padding:0 18px;max-width:1500px}a{color:#8ee3c0}table{border-collapse:collapse;font-size:12px;display:block;overflow:auto}td,th{border:1px solid #42594c;padding:7px;text-align:left;vertical-align:top}code{overflow-wrap:anywhere}pre{white-space:pre-wrap;font-size:11px}.note{padding:12px;background:#263a2e}article{margin:24px 0}video{width:min(100%,800px)}.small{font-size:11px;color:#b2c6b8}</style>',
        '<h1>카메라 영역 기반 수확 · 실제 실행 기록</h1>',
        '<p class="note">스캔·계획 거절·실제 진입·벨트 작동·열매 분리를 따로 집계합니다. 분리 수는 새 native 조인트 파단이며 수거 성공을 뜻하지 않습니다. 송이 ID는 식물 경로까지 포함합니다. 진행 중 결과는 미완료로 남깁니다.</p>',
        '<p>' + link('camera_campaign_report.json', 'JSON 전체 원자료 집계') + '</p>']
    if goal:
        complete = goal.get('passed') is True
        heading = f"선택한 실제 수확 이력: {goal.get('distinct_truss_count', 0)}/{goal.get('minimum_distinct_trusses', 6)}송이 · " + ('목표 달성' if complete else '목표 미완료')
        fragments.append('<h2>' + esc(heading) + '</h2><p class="note">새 native 열매 분리 ' + str(data['accepted_route_native_detachments']) + '개. 이동·스캔·계획 거절·실제 중단과 독립 진단을 보존합니다. 송이 전체 수확이나 수거 완료를 뜻하지 않습니다.</p>')
        if goal.get('software_fault_independent_branch_present'):
            fragments.append('<p class="note">소프트웨어 오류의 1.45초 부분 기록과 이후 정상 스냅샷에서 시작한 독립 재시도를 구분합니다. 재생 순서는 하나의 끊김 없는 연속 물리 실행을 뜻하지 않습니다.</p>')
    labels = dict(visible_qualifying='관측 후보', planned_regions='계획 승인', actual_approach='실제 진입',
                  actual_belt='벨트 작동', motion_completed='동작 완주', new_native_detached='새 열매 분리')
    fragments.append('<h2>전체 실험의 서로 다른 송이 수</h2><table><tr>' + ''.join('<th>' + label + '</th>' for label in labels.values()) + '</tr><tr>' + ''.join('<td>' + str(data['totals_all_experiments']['truss_counts'][name]) + '</td>' for name in labels) + '</tr></table>')
    for patrol in data.get('patrol_campaigns',[]):
        count=patrol['achieved_distinct_trusses_from_native_evidence'];requested=patrol['goal_distinct_trusses']
        fragments.append('<article><h2>카메라 순회 '+esc(patrol['path'])+'</h2><p class="note">'+
            esc(patrol['explicit_label'])+' · '+str(count)+'/'+str(requested)+'송이 · '+esc(patrol['status'])+'</p>')
        fragments.append('<p>'+link(patrol['path'],'순회 실행 원자료')+' · 받아들인 실제 구간 '+
            str(len(patrol['accepted_actual_trial_paths']))+'개 · 부모 기록 SHA 검증 '+
            esc(patrol['continuation_chain_valid'])+'</p><p class="small">'+esc(patrol['native_continuity'])+'</p>')
        fragments.append('<p>서로 다른 송이는 완료된 실제 수확의 선택 열매 분리 증거로만 셉니다. 초기 촬영·이동·사용하지 않은 창의 초기화 검사·계획 거절은 수확 실적이 아닙니다. 부모 기록을 재개해도 같은 실제 구간을 두 번 세지 않습니다.</p>')
        fragments.append('<pre>'+esc(json.dumps(patrol['classification_counts'],ensure_ascii=False,indent=2))+'</pre>')
        fragments.append('<p>카메라가 관측하여 기억한 송이 '+str(patrol['visible_camera_queue']['ever_queued_truss_count'])+
            '개 · '+esc(patrol['visible_camera_queue']['statuses'])+' · 관측 수는 진입 가능성/수확 성공 수와 다릅니다.</p>')
        fragments.append('<table><tr><th>기록/결정</th><th>동작/대상</th><th>현재→명령/실제 위치(m)</th><th>상태/실제 기록</th><th>수확 증거</th><th>차단·검사</th></tr>')
        for row in patrol['finite_decisions']:
            probe=row.get('startup_preflight') or {}
            probe_text=('별도 startup 진단 · PASS' if probe.get('passed') is True else '별도 startup 진단 · FAIL/미확정') if probe else ''
            fragments.append('<tr><td>'+esc(row['campaign'])+' #'+esc(row['index'])+'</td><td>'+esc(row['action'])+'<br>'+esc(row['target_key'])+
                '</td><td>'+esc(row['current_station_y_m'])+' → '+esc(row['requested_station_y_m'])+' / '+esc(row['measured_station_y_m'])+
                '</td><td>'+esc(row['status'])+'<br>측정 기록 '+esc(row['recorded_physics_execution'])+
                ('<br>같은 실제 구간 참조·중복 집계 제외' if row['duplicate_trial_reference'] else '')+
                '<br>'+link(row['trial_path'], '실제 구간 폴더')+'</td><td>'+esc(', '.join(row['credited_native_trusses']))+
                '</td><td>'+esc(row['blocker'] or row['decision_reason'])+'<br>'+esc(probe_text)+'</td></tr>')
        fragments.append('</table></article>')
    for campaign in data['campaigns']:
        fragments.append('<article><h2>' + esc(campaign['path']) + '</h2><p>' + esc(campaign['status']) + ' · ' + link(campaign['path'], '실행/계획 창 원자료') + '</p><p class="small">' + esc(campaign['native_continuity']) + '</p>')
        fragments.append('<p>이번 캠페인 실제 결과: ' + esc(campaign['actual_result_summary']['truss_counts']) + '</p><table><tr><th>창</th><th>식물/송이</th><th>레일 위치</th><th>실행 상태</th><th>차단 원인</th></tr>')
        for row in campaign['finite_windows']:
            fragments.append('<tr>' + ''.join('<td>' + esc(value) + '</td>' for value in (row['index'], row['path'], row['station_y_m'], row['status'], row['blocker'])) + '</tr>')
        fragments.append('</table></article>')
    for trial in data['trials']:
        fragments.append('<article><h2>' + esc(trial['path']) + '</h2><p>' + esc(trial['status']) + ' · ' + esc(trial['sequence_termination'] or trial['stop_reason']) + '</p>')
        fragments.append('<p>새 분리 ' + str(trial['new_native_break_count']) + '개 · 카메라 ' + esc(trial['camera_frames']) + '프레임 · ' + esc(trial['hz']) + 'Hz · 관통 감시(mm) ' + esc(trial['contact_limits_mm']) + '</p>')
        fragments.append('<p>' + ' · '.join(link(path, name) for name, path in trial['evidence'].items()) + '</p>')
        if trial['runtime_error']: fragments.append('<pre>' + esc(json.dumps(trial['runtime_error'], ensure_ascii=False)) + '</pre>')
        fragments.append('<table><tr><th>회차</th><th>송이/선택 열매</th><th>상태</th><th>실제 진입/벨트</th><th>관측/오류</th></tr>')
        for row in trial['cycles']:
            kind = '실제 후퇴 복구·수확0' if row.get('recovery') else '스캔 전용' if row['scan_prefix'] else row['status']
            fragments.append('<tr><td>' + esc(row['index']) + '</td><td>' + esc(row['exact_source_truss']) + '<br>' + esc(', '.join(row['fruits'])) + '</td><td>' + esc(kind) + '</td><td>' + esc(row['actual_approach']) + ' / ' + esc(row['actual_belt']) + '</td><td>' + esc(row['error'] or row['selection_observation']) + '</td></tr>')
        fragments.append('</table>')
        for video in trial['videos']:
            fragments.append('<p>' + link(video['path'], str(display_root / video['path'])) + ' · ' + esc(video['source']) + '</p><video controls preload="metadata" src="' + quote(video['path'], safe='/') + '"></video>')
        fragments.append('</article>')
    if data['warnings']: fragments.append('<h2>읽기 경고</h2><pre>' + esc(json.dumps(data['warnings'], ensure_ascii=False, indent=2)) + '</pre>')
    fragments.append('</html>')
    (root / 'camera_campaign_report.json').write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    (root / 'camera_campaign_report.html').write_text('\n'.join(fragments))
    return data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    report = write_report(args.root)
    print(json.dumps(dict(report=str(args.root / 'camera_campaign_report.html'),
                          totals=report['totals_all_experiments'], warnings=report['warnings']), ensure_ascii=False, indent=2))
