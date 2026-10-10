#!/usr/bin/env python3
"""Refresh a performance report from preserved native trials, including failures.

This reads the existing executor/campaign artifacts. It does not rerun physics,
alter raw evidence, or rank shortened/invalid attempts as speed improvements.
"""
import argparse
import csv
import hashlib
import html
import json
import math
import mmap
import os
import re
import statistics
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = 'harvest-s300-performance-report-v1'
PAIR = re.compile(rb'"a"\s*:\s*"([^"\\]*)"\s*,\s*"b"\s*:\s*"([^"\\]*)"')
ENV = re.compile(rb'^/World/MotionEnvs/env_(\d+)(?=/|$)')
EVENT = re.compile(rb'\{[^{}]*\}')


def read_json(path, default=None):
    if not path.is_file():
        return default
    try:
        return json.loads(path.read_text())
    except (ValueError, OSError):
        return default


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def warning_evidence(log, error):
    lines = log.read_text(errors='replace').splitlines() if log.is_file() else []
    warnings = [line for line in lines if '[Warning]' in line]
    errors = [line for line in lines if '[Error]' in line]
    physical = [line for line in warnings if any(x in line.lower() for x in
                ('physx', 'physics.tensors', 'cuda', 'collision group', 'simulationmanager'))]
    groups = sum('collision group' in line.lower() and 'only in one' in line.lower()
                 for line in warnings)
    text = '\n'.join(errors) + '\n' + (error or '')
    cuda = any(x in text.lower() for x in ('cuda error', 'misaligned address',
                                         'illegal memory access', 'device-side assert'))
    def unique(items):
        # Remove timestamps and elapsed counters before grouping repeated warnings.
        return list(dict.fromkeys(re.sub(r'^.*?\[(?:Warning|Error)\]\s*', '', x) for x in items))[:12]
    return dict(warnings=len(warnings), errors=len(errors), native_physics_warnings=len(physical),
                collision_group_warnings=groups, cuda_error=cuda,
                native_warning_examples=unique(physical), error_examples=unique(errors))


def cross_environment_evidence(path):
    if not path.is_file():
        return dict(checked=False, point_records=None, cross_environment_point_records=None, examples=[])
    checked = cross = 0
    examples = []
    if path.stat().st_size:
        with path.open('rb') as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
            for pair in PAIR.finditer(data):
                checked += 1
                a, b = pair.group(1), pair.group(2)
                env_a, env_b = ENV.match(a), ENV.match(b)
                if env_a and env_b and env_a.group(1) != env_b.group(1):
                    cross += 1
                    value = [a.decode(), b.decode()]
                    if len(examples) < 5 and value not in examples:
                        examples.append(value)
    return dict(checked=True, point_records=checked, cross_environment_point_records=cross,
                examples=examples, scope='saved exact collider/actor paths only; not an unsampled collision proof')


def resources(path):
    samples = read_json(path, [])
    if not isinstance(samples, list):
        samples = samples.get('samples', [])
    result = {'sample_count': len(samples)}
    for key, factor in [('cpu_percent', 1.), ('rss_bytes', 1/(1024**3)),
                        ('gpu_percent', 1.), ('gpu_memory_mib', 1.)]:
        values = [float(s[key])*factor for s in samples if s.get(key) is not None]
        name = 'rss_gib' if key == 'rss_bytes' else key
        result[name+'_mean'] = sum(values)/len(values) if values else None
        result[name+'_max'] = max(values) if values else None
    result['scope'] = 'whole process lifetime including startup; GPU readings are device-wide'
    return result


def group_resources(path, started=None, finished=None):
    """Aggregate simultaneous child samples, without summing per-worker peaks."""
    data = read_json(path, [])
    samples = data if isinstance(data, list) else data.get('samples', [])
    normalized = []
    for sample in samples:
        children = sample.get('children', [])
        normalized.append(dict(sample,
            rss_bytes=sum(child.get('rss_bytes', 0) or 0 for child in children),
            cpu_percent=sum(child.get('cpu_percent', 0) or 0 for child in children)))
    def summarize(selected):
        result = {'sample_count': len(selected)}
        for key, factor in [('rss_bytes', 1/(1024**3)), ('cpu_percent', 1.),
                            ('gpu_percent', 1.), ('gpu_memory_mib', 1.)]:
            values = [float(s[key])*factor for s in selected if s.get(key) is not None]
            name = 'rss_gib' if key == 'rss_bytes' else key
            result[name+'_mean'] = sum(values)/len(values) if values else None
            result[name+'_median'] = statistics.median(values) if values else None
            result[name+'_max'] = max(values) if values else None
        return result
    window = [sample for sample in normalized if started is not None and finished is not None
              and started <= sample.get('monotonic_s', -math.inf) <= finished]
    return dict(whole_group_lifetime=summarize(normalized), concurrent_physics_window=summarize(window),
                scope='RSS/CPU summed across children at each simultaneous sample; GPU readings device-wide')


def initialization_evidence(folder, summary):
    explicit = read_json(folder/'initialization_validity.json')
    if explicit is not None:
        violations = explicit.get('violations', [])
        return dict(valid=explicit.get('valid'), source='executor initialization_validity.json',
                    physics_warmup_steps=explicit.get('physics_warmup_steps'),
                    reported_contact_points=explicit.get('reported_contact_points'),
                    violation_points=len(violations), evidence_file='initialization_validity.json',
                    scope=explicit.get('timestamp_scope'))
    if 'initialization_valid' in summary:
        return dict(valid=summary['initialization_valid'], source='executor summary', violation_points=None)
    # Earlier summaries include warmup contacts without a separate guard. Keep
    # the raw summaries untouched and annotate their captured t=0 evidence here.
    path = folder/'contacts.json'
    reported = violations = 0
    worst = []
    peak_force = 0.
    if path.is_file() and path.stat().st_size:
        with path.open('rb') as stream, mmap.mmap(stream.fileno(),0,access=mmap.ACCESS_READ) as data:
            for match in EVENT.finditer(data):
                raw = match.group()
                # This report consumes the executor's flat JSON point records.
                # Avoid decoding later trial samples when inspecting warmup.
                if not re.match(rb'\{\s*"t"\s*:\s*0(?:\.0*)?\s*,', raw):
                    continue
                event = json.loads(raw)
                if event.get('t') != 0.:
                    continue
                reported += 1
                active = any(token in event.get('a','') or token in event.get('b','')
                             for token in ('/ActuatedTool/','/HarvestRobot/','/ElasticTarget/'))
                if not active:
                    continue
                peak_force = max(peak_force, float(event.get('force_n',0.)))
                if event.get('separation_m',0.) < -.0005:
                    violations += 1
                    row = {key:event.get(key) for key in ('a','b','separation_m','force_n','phase')}
                    worst.append(row)
                    worst.sort(key=lambda item:item['separation_m'])
                    del worst[5:]
    return dict(valid=False if violations else True if reported else None,
                source='posthoc captured legacy t=0 native contact records',
                reported_contact_points=reported, violation_points=violations,
                maximum_active_point_force_n=peak_force if reported else None,
                worst_violations=worst,
                scope='captured native warmup contact evidence; no additional physics run or raw summary change')


def command_option(command, option, default=None):
    if option not in command:
        return default
    index = command.index(option)
    return command[index+1] if index+1 < len(command) else default


def case_row(root, folder, campaign):
    s = read_json(folder/'summary.json', {})
    rt = read_json(folder/'runtime_settings.json', {})
    bootstrap = read_json(folder/'app_extensions.json', {})
    native = read_json(folder/'native_metadata.json', {})
    error = read_json(folder/'runtime_error.json', {})
    cancelled = read_json(folder/'cancelled.json', {})
    command = campaign.get('command', [])
    warnings = warning_evidence(folder.parent/(folder.name+'.log'), error.get('error'))
    envs = int(s.get('num_envs', rt.get('num_envs', command_option(command, '--num-envs', 1))))
    cross = cross_environment_evidence(folder/'contacts.json') if envs > 1 else dict(
        checked=False, cross_environment_point_records=None, reason='one environment')
    initialization = initialization_evidence(folder,s)
    if error:
        status = 'runtime_error'
    elif cancelled or campaign.get('timeout'):
        status = 'cancelled_or_timeout'
    elif s and not s.get('completed'):
        status = s.get('stop_reason') or 'incomplete'
    elif s.get('completed'):
        status = 'completed_diagnostic'
    else:
        status = 'in_progress_or_incomplete'
    if s.get('completed') and initialization['valid'] is False:
        status = 'completed_but_initialization_invalid'
    separation = s.get('min_contact_separation_m')
    penetration_breach = separation is not None and separation < -.0005
    blockers = []
    if not s.get('completed'):
        blockers.append(status)
    if penetration_breach:
        blockers.append('recorded_penetration_over_0_5mm')
    if initialization['valid'] is False:
        blockers.append('initialization_invalid')
    if warnings['cuda_error']:
        blockers.append('CUDA_runtime_error')
    if warnings['collision_group_warnings']:
        blockers.append('collision_group_filtering_warning')
    if cross.get('cross_environment_point_records'):
        blockers.append('cross_environment_contacts')
    if campaign.get('returncode') not in (None, 0):
        blockers.append('nonzero_process_exit')
    requested_device=s.get('physics_settings',{}).get('device',rt.get('device'))
    native_scene=rt.get('native_scene_device',{})
    physics_only=bootstrap.get('physics_only','--physics-only' in command)
    native_device_matches=None
    if native_scene.get('enabled_gpu_dynamics') is not None and requested_device:
        requested_gpu=str(requested_device).startswith('cuda')
        broadphase_request=str(native_scene.get('requested_broadphase',rt.get('broadphase',
                              command_option(command,'--broadphase','auto')))).upper()
        expected_broadphase=('GPU' if requested_gpu else 'MBP') if broadphase_request=='AUTO' else broadphase_request
        native_device_matches=(bool(native_scene['enabled_gpu_dynamics'])==requested_gpu and
                               native_scene.get('broadphase_type')==expected_broadphase)
    if native_device_matches is False:
        blockers.append('native_scene_device_mismatch')
        if s.get('completed'):status='completed_but_native_device_mismatch'
    elif physics_only and native_device_matches is None:
        # The first minimal worker did not register the source scene in the
        # SimulationManager cache. CLI/tensor CPU alone cannot prove CPU physics.
        blockers.append('physics_only_native_scene_device_unverified')
        if s.get('completed'):status='completed_native_device_unverified'
    eligible = not blockers
    loop = s.get('physics_wall_s')
    saved = s.get('save_wall_s')
    total = campaign.get('total_wall_s', error.get('elapsed_s'))
    startup_other = max(0., total-loop-(saved or 0.)) if total is not None and loop is not None else None
    aggregate = s.get('aggregate_simulated_s')
    if aggregate is None and s.get('simulated_s') is not None:
        aggregate = s['simulated_s']*envs
    throughput = aggregate/loop if aggregate is not None and loop else None
    settings = dict(rt)
    settings.update(s.get('physics_settings', {}))
    physics_changed = (settings.get('plant_dynamics', 'full') != 'full' or
                       settings.get('hz', 240) != 240 or settings.get('solver', 'PGS') != 'PGS' or
                       settings.get('position_iterations', 64) != 64 or
                       settings.get('velocity_iterations', 4) != 4 or
                       settings.get('zero_conveyor_diagnostic', False))
    fixture = s.get('fixture_only', rt.get('isolate_background', '--isolate-background' in command))
    mode=s.get('mode',command_option(command,'--mode'))
    synthetic_belt=mode=='belt_control'
    files = {}
    for file in folder.iterdir():
        if file.is_file() and file.suffix in ('.json', '.npz', '.mp4', '.png', '.svg'):
            files[file.name] = str(file.relative_to(root))
    for suffix in ('.log', '_resources.json'):
        file = folder.parent/(folder.name+suffix)
        if file.is_file():
            files[file.name] = str(file.relative_to(root))
    return dict(name=folder.name, relative_directory=str(folder.relative_to(root)),
        status=status, completed=bool(s.get('completed')),
        timing_comparable=eligible, timing_exclusion_reasons=blockers,
        scope='component fixture; background/internal-leaf exclusions' if fixture else 'farm contacts enabled',
        fixture_only=bool(fixture), mode=mode,synthetic_belt_control=synthetic_belt,
        device=settings.get('device'), environments=envs,
        physics_only=bool(physics_only),native_scene_device=native_scene,
        native_broadphase=native_scene.get('broadphase_type'),
        native_device_matches_requested=native_device_matches,
        plant_dofs=s.get('native_plant_dofs', len(native.get('plant_dofs', [])) or None),
        robot_dofs=len(native.get('robot_dofs', [])) or None,
        bodies=len(native.get('body_paths', [])) or None,
        plant_dynamics=settings.get('plant_dynamics', 'full'), physical_variant=bool(physics_changed),
        hz=settings.get('hz'), solver=settings.get('solver'),
        position_iterations=settings.get('position_iterations', 64),
        velocity_iterations=settings.get('velocity_iterations', 4), threads=settings.get('threads', 0),
        contact_backend=s.get('contact_backend', settings.get('contact_backend', 'callback')),
        gravity_neutral_warmup=bool(settings.get('gravity_neutral_warmup',False)),
        armature_setup=settings.get('armature_setup'),
        buffer_contact_events=bool(settings.get('buffer_contact_events',False)),
        collision_filter=s.get('collision_filter', settings.get('collision_filter', 'pairs')),
        contact_evidence_scope=s.get('contact_evidence_scope', 'exact collider impulse norm'),
        performance=s.get('performance', settings.get('performance')),
        simulated_s=s.get('simulated_s'), aggregate_simulated_s=aggregate,
        loop_wall_s=loop, physics_initialization_s=native.get('physics_initialization_s'),
        loop_started_monotonic_s=s.get('loop_started_monotonic_s'),
        loop_finished_monotonic_s=s.get('loop_finished_monotonic_s'),
        startup_and_other_s=startup_other, save_wall_s=saved, process_total_wall_s=total,
        accepted_aggregate_sim_s_per_wall_s=throughput if eligible else None,
        partial_or_unvalidated_aggregate_sim_s_per_wall_s=throughput,
        target_displacement_max_mm=None if s.get('target_displacement_max_m') is None else s['target_displacement_max_m']*1000,
        target_displacement_final_mm=None if s.get('target_displacement_final_m') is None else s['target_displacement_final_m']*1000,
        min_contact_separation_mm=None if separation is None else separation*1000,
        contact_events=s.get('contact_events'), break_events=len(s.get('native_break_events', [])),
        cross_environment_evidence=cross, initialization_evidence=initialization, warnings=warnings,
        resources=resources(folder.parent/(folder.name+'_resources.json')),
        model_sha256=s.get('model_sha256', settings.get('source_scene_sha256')),
        code_sha256=read_json(folder/'code_sha256.json', {}), command=command,
        returncode=campaign.get('returncode'), timeout=campaign.get('timeout'),
        error=error.get('error'), profile=s.get('profile', read_json(folder/'profile.json', {})),
        harvest_success=s.get('harvest_success'), training_eligible=False, files=files)


def parallel_group_row(root, file, raw, rows_by_directory):
    """Accept parallel speed only when all preserved child trials are valid."""
    names = raw.get('names', [])
    members, blockers = [], list(raw.get('exclusion_reasons', []))
    if raw.get('throughput_eligible') is not True:
        blockers.append('campaign_group_not_validated')
    if not names or len(set(names)) != len(names) or len(names) != raw.get('jobs'):
        blockers.append('group_membership_missing_or_inconsistent')
    for name in names:
        relative = str((file.parent/name).relative_to(root))
        child = rows_by_directory.get(relative)
        if child is None:
            blockers.append(name+':missing_trial')
            continue
        members.append(child)
        if not child['timing_comparable']:
            blockers.append(name+':'+','.join(child['timing_exclusion_reasons']))
        if child['initialization_evidence']['valid'] is not True:
            blockers.append(name+':initialization_not_validated')
        if child['native_device_matches_requested'] is not True:
            blockers.append(name+':native_device_not_validated')
        if child['break_events']:
            blockers.append(name+':native_break_events_present')
    times = [(row.get('loop_started_monotonic_s'), row.get('loop_finished_monotonic_s')) for row in members]
    valid_times = len(times) == len(names) and bool(times) and all(
        isinstance(start, (int, float)) and isinstance(end, (int, float)) and
        math.isfinite(start) and math.isfinite(end) and end > start for start,end in times)
    started = min(start for start,end in times) if valid_times else None
    finished = max(end for start,end in times) if valid_times else None
    window = finished-started if valid_times else None
    if not valid_times:
        blockers.append('native_loop_timestamps_missing_or_invalid')
    aggregate = sum(row.get('aggregate_simulated_s') or 0. for row in members)
    if raw.get('measured_concurrent_physics_window_s') is None or window is None or not math.isclose(
            raw['measured_concurrent_physics_window_s'], window, rel_tol=0., abs_tol=1e-6):
        blockers.append('campaign_native_window_mismatch')
    if raw.get('sum_aggregate_simulated_s') is None or not math.isclose(
            raw['sum_aggregate_simulated_s'], aggregate, rel_tol=0., abs_tol=1e-9):
        blockers.append('campaign_simulated_duration_mismatch')
    signatures = {(row['model_sha256'], row['device'], row['hz'], row['solver'],
                   row['position_iterations'], row['velocity_iterations'], row['plant_dynamics'],
                   row['fixture_only'], row['contact_backend'], row['collision_filter'],
                   row['gravity_neutral_warmup'],row['armature_setup'],row['buffer_contact_events'],
                   row['native_broadphase']) for row in members}
    if len(signatures) != 1:
        blockers.append('group_model_or_physics_settings_mismatch')
    blockers = list(dict.fromkeys(blockers))
    resource_path = file.parent/f"parallel_group_{raw.get('index', 0):04d}_resources.json"
    resources_value = group_resources(resource_path, started, finished)
    return dict(name=file.parent.name+f"/group_{raw.get('index',0):04d}",
        file=str(file.relative_to(root)), jobs=raw.get('jobs'), child_names=names,
        child_directories=[row['relative_directory'] for row in members],
        throughput_eligible=not blockers, exclusion_reasons=blockers,
        measured_concurrent_physics_window_s=window,
        sum_aggregate_simulated_s=aggregate,
        accepted_aggregate_sim_s_per_wall_s=aggregate/window if window and not blockers else None,
        accepted_end_to_end_aggregate_sim_s_per_wall_s=(aggregate/raw['group_total_wall_s']
            if not blockers and isinstance(raw.get('group_total_wall_s'),(int,float))
            and raw['group_total_wall_s']>0 else None),
        diagnostic_aggregate_sim_s_per_wall_s=aggregate/window if window else None,
        group_total_wall_s=raw.get('group_total_wall_s'),
        group_startup_wall_s=raw.get('group_startup_wall_s'), post_loop_wall_s=raw.get('post_loop_wall_s'),
        loop_start_spread_s=max(start for start,end in times)-started if valid_times else None,
        model_sha256=members[0]['model_sha256'] if len(signatures)==1 else None,
        device=members[0]['device'] if len(signatures)==1 else None,
        hz=members[0]['hz'] if len(signatures)==1 else None,
        solver=members[0]['solver'] if len(signatures)==1 else None,
        threads_per_worker=sorted(set(row['threads'] for row in members)),
        resources=resources_value,
        resource_file=str(resource_path.relative_to(root)) if resource_path.is_file() else None,
        interpretation='Independent native worker processes; validated child loops share one measured wall window; not GPU scene cloning')


def relative_link(path, output):
    return os.path.relpath(path, output).replace(os.sep, '/')


def render_page(report, root, output):
    esc = html.escape
    def link(path, label):
        return '<a href="'+esc(relative_link(root/path, output), quote=True)+'">'+esc(label)+'</a>'
    def number(value, digits=3):
        return '—' if value is None else f'{value:.{digits}f}'
    table = []
    details = []
    for row in report['trials']:
        scope = '분리 fixture' if row['fixture_only'] else '농장 충돌 유지'
        if row['synthetic_belt_control']:
            scope += ' / 합성 구 벨트 fixture'
        if row['physical_variant']:
            scope += ' / 물리 설정 변화'
        warning = []
        if row['warnings']['cuda_error']:
            warning.append('CUDA 오류')
        if row['warnings']['collision_group_warnings']:
            warning.append('collision-group 경고 '+str(row['warnings']['collision_group_warnings']))
        cross = row['cross_environment_evidence'].get('cross_environment_point_records')
        if cross:
            warning.append('환경 간 접촉 '+str(cross))
        if row['initialization_evidence']['valid'] is False:
            warning.append('초기 warmup 관통')
        if row['native_device_matches_requested'] is False:
            warning.append('native device 불일치')
        elif row['physics_only'] and row['native_device_matches_requested'] is None:
            warning.append('native device 미확인')
        cells = [row['name'], row['status'], scope,
                 str(row['environments'])+' / '+str(row['plant_dofs']),
                 str(row['device'])+' / '+str(row['hz'])+'Hz / '+str(row['solver'])+' / '+str(row['contact_backend'])+' / '+str(row['collision_filter'])+' / BP:'+str(row['native_broadphase']),
                 str(row['position_iterations'])+'/'+str(row['velocity_iterations']),
                 number(row['simulated_s']), number(row['loop_wall_s']),
                 number(row['startup_and_other_s'], 2), number(row['save_wall_s']),
                 number(row['accepted_aggregate_sim_s_per_wall_s']),
                 number(row['target_displacement_max_mm']), number(row['min_contact_separation_mm']),
                 number(row['resources'].get('cpu_percent_mean'), 1),
                 number(row['resources'].get('gpu_percent_max'), 1),
                 number(row['resources'].get('rss_gib_max'), 2),
                 '; '.join(warning) or str(row['warnings']['native_physics_warnings'])+' native warnings']
        row_class = 'valid' if row['timing_comparable'] else 'invalid'
        table.append('<tr class="'+row_class+'">'+''.join('<td>'+esc(str(x))+'</td>' for x in cells)+
                     '<td><a href="#'+esc(row['name'],quote=True)+'">원자료</a></td></tr>')
        evidence = ' · '.join(link(path, filename) for filename,path in sorted(row['files'].items()))
        detail = '<article id="'+esc(row['name'],quote=True)+'"><h2>'+esc(row['name'])+'</h2><p>'+evidence+'</p>'
        detail += '<p>'+esc(row['scope'])+'; '+esc(', '.join(row['timing_exclusion_reasons']) or '동작 완주; 수확 성공 판정 없음')+'</p>'
        detail += '<p>모델 SHA: <code>'+esc(str(row['model_sha256']))+'</code></p>'
        for filename,path in row['files'].items():
            if filename.endswith('.mp4'):
                detail += '<video controls preload="metadata" src="'+esc(relative_link(root/path,output),quote=True)+'"></video>'
        if row['error']:
            detail += '<details><summary>실행 오류</summary><pre>'+esc(row['error'])+'</pre></details>'
        if row['warnings']['native_warning_examples'] or row['warnings']['error_examples']:
            detail += '<details><summary>native 경고 / 오류</summary><pre>'+esc('\n'.join(
                row['warnings']['native_warning_examples']+row['warnings']['error_examples']))+'</pre></details>'
        if row['profile']:
            detail += '<details><summary>실행 구간 프로파일</summary><pre>'+esc(json.dumps(row['profile'],indent=2))+'</pre></details>'
        detail += '<details><summary>초기 warmup 판정 · 과거 실행 사후 확인</summary><pre>'+esc(json.dumps(
            row['initialization_evidence'],ensure_ascii=False,indent=2))+'</pre></details>'
        detail += '</article>'
        details.append(detail)
    comparison = []
    for item in report['comparisons']:
        verdict='공통 timestamp 진단 · 동등성 판정 없음' if item.get('diagnostic_only') else str(item.get('passed'))
        if item.get('total_workers') is not None:
            verdict+=' · '+str(item.get('strict_pass_count'))+'/'+str(item['total_workers'])+' worker strict trace PASS'
        if item.get('comparisons_total') is not None:
            verdict+=' · '+str(item.get('comparisons_passed'))+'/'+str(item['comparisons_total'])+' strict trace PASS'
        comparison.append('<li>'+link(item['file'], item['file'])+' : '+
                          esc(verdict)+' / '+esc(item.get('interpretation', ''))+'</li>')
    visuals=[]
    for path in report.get('visual_artifacts',[]):
        url=esc(relative_link(root/path,output),quote=True)
        body=('<video controls preload="metadata" src="'+url+'"></video>' if path.endswith('.mp4') else
              '<a href="'+url+'"><img loading="lazy" style="max-width:100%;width:960px" src="'+url+'"></a>')
        visuals.append('<figure><figcaption>'+link(path,path)+'</figcaption>'+body+'</figure>')
    evidence = [link(item, item) for item in report['evidence_files']]
    groups=[]
    for group in report.get('parallel_groups',[]):
        life=group['resources']['whole_group_lifetime']
        window_resources=group['resources']['concurrent_physics_window']
        links=link(group['file'],'group JSON')
        if group['resource_file']:
            links+=' · '+link(group['resource_file'],'동시 자원 표본')
        cells=[group['name'],str(group['jobs']),
               str(group['device'])+' / '+str(group['hz'])+'Hz / '+str(group['solver']),
               '/'.join(str(x) for x in group['threads_per_worker']),
               number(group['group_startup_wall_s']),
               number(group['measured_concurrent_physics_window_s']),
               number(group['sum_aggregate_simulated_s']),
               number(group['accepted_aggregate_sim_s_per_wall_s']),
               number(group['post_loop_wall_s']),number(group['group_total_wall_s']),
               number(group['accepted_end_to_end_aggregate_sim_s_per_wall_s']),
               number(group['loop_start_spread_s'],4),
               number(life.get('rss_gib_max'),2),number(life.get('gpu_memory_mib_max'),0),
               number(window_resources.get('cpu_percent_median'),1)+' / '+number(window_resources.get('cpu_percent_max'),1),
               'VALID' if group['throughput_eligible'] else '; '.join(group['exclusion_reasons'])]
        groups.append('<tr class="'+('valid' if group['throughput_eligible'] else 'invalid')+'">'+
                      ''.join('<td>'+esc(str(x))+'</td>' for x in cells)+'<td>'+links+'</td></tr>')
    group_section=''
    if groups:
        group_section='''<h2>독립 CPU worker 동시 실행</h2><p>각 worker의 전체 농장 초기 상태와 물리 실행이 모두 유효한 집단만 처리량을 표시합니다.
동시 physics window는 최초 worker 시작부터 마지막 worker 종료까지이며, 준비·결과 저장은 별도입니다.
RSS는 같은 시각의 모든 child 프로세스 RSS 합입니다. GPU 메모리는 장치 전체 측정값으로 렌더러와 공유 라이브러리 영향을 포함합니다.
같은 7초 부하 시험의 병렬 처리량이며 로봇 수확 성공이나 다른 장면에서의 처리량을 입증하지 않습니다.</p><div class="scroll"><table><tr>
<th>집단</th><th>workers</th><th>device / Hz / solver</th><th>worker당 threads</th><th>준비 s</th><th>동시 physics s</th><th>합계 sim s</th><th>합산 sim-s/s</th><th>종료·저장 s</th><th>전체 s</th><th>준비·저장 포함 sim-s/s</th><th>시작 분산 s</th><th>합계 RSS 최대 GiB</th><th>장치 GPU MiB</th><th>physics CPU 중앙/최대 %</th><th>판정</th><th>증거</th></tr>'''+''.join(groups)+'</table></div>'
    local_docs = []
    for name in ('README.md', 'MOTION_STATUS_KO.md', 'PERFORMANCE_STATUS_KO.md'):
        path = Path(__file__).parent/name
        if path.is_file():
            local_docs.append('<a href="'+esc(relative_link(path,output),quote=True)+'">'+esc(name)+'</a>')
    return '''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="generator" content="'''+SCHEMA+'''">
<title>S300 Isaac 성능 · 전체 시도</title><style>
body{font:16px system-ui;margin:30px;background:#141a21;color:#e6edf4}a{color:#8ed2ff}table{border-collapse:collapse;font-size:12px;white-space:nowrap}td,th{border:1px solid #43505e;padding:7px}.scroll{overflow:auto}.note{padding:18px;background:#34311e;line-height:1.6}.invalid{background:#3b2528}.valid{background:#1d3031}pre{white-space:pre-wrap;overflow-wrap:anywhere}video{width:min(100%,960px)}article{padding:16px 0;border-bottom:1px solid #43505e}code{overflow-wrap:anywhere}
</style><h1>S300 + RB5 + V5.1 · Isaac 성능 및 실패 기록</h1>
<p>갱신 UTC: '''+esc(report['generated_utc'])+''' · '''+str(len(report['trials']))+'''개 실행 · <a href="performance_summary.json">JSON</a> · <a href="results.csv">CSV</a></p>
<div class="note">완주한 분리 fixture는 정상 농장의 수확 성공이 아닙니다. 주변 충돌을 끄거나 원거리 송이의 휨을 고정한 설정은 각각 표시합니다.
초기 관통, 환경 간 접촉, CUDA 오류, 실행 중단 및 native device 미확인 사례는 처리량 비교에서 제외합니다. 짧게 중단되어 빨라진 결과를 속도 개선으로 세지 않습니다.
이전 실행에 초기 상태 guard가 없었던 경우 저장된 t=0 접촉을 사후 검사해 표에 표시하며, 원래 summary.json은 수정하지 않습니다.
표의 처리량은 환경 수 × simulated seconds / loop wall seconds입니다. 앱/장면 준비 및 종료 비용과 결과 저장은 별도입니다.
Tensor normal-only force와 native collider impulse는 같은 측정량이 아니며, 물리 설정 변경은 동등한 최적화가 아닙니다.
프로파일의 callback 시간이 simulate 안에 포함될 수 있어 두 값을 합산하지 않습니다. CPU/GPU 자원값은 준비 포함 전체 프로세스, GPU 값은 장치 전체 표본입니다.
완주 여부와 저장된 관통 지표만으로 검증된 수확 성공 또는 다른 경로의 물리 정확도를 주장하지 않습니다.</div>
<p>'''+ ' · '.join(local_docs)+'''</p><div class="scroll"><table><tr>
<th>실행</th><th>결과</th><th>범위</th><th>환경 / 식물 DOF</th><th>device / Hz / solver / contact / filter</th><th>solver 반복</th><th>sim s</th><th>loop s</th><th>준비·기타 s</th><th>save s</th><th>합산 sim-s/s</th><th>최대 이동 mm</th><th>최소 separation mm</th><th>전체 CPU 평균 %</th><th>장치 GPU 최대 %</th><th>RSS 최대 GiB</th><th>경고 / 오류</th><th>증거</th></tr>
'''+''.join(table)+'''</table></div>'''+group_section+'''<h2>상태·접촉 비교</h2><ul>'''+''.join(comparison)+'''</ul>
<h2>환경·표시 진단</h2><p>'''+ ' · '.join(evidence)+'''</p>'''+''.join(visuals)+''.join(details)+'</html>'


def generate(root, output=None):
    root = Path(root).resolve()
    output = Path(output).resolve() if output else root
    output.mkdir(parents=True, exist_ok=True)
    existing = read_json(output/'performance_summary.json')
    if (output/'index.html').exists() and (not existing or existing.get('schema') != SCHEMA):
        raise FileExistsError('Refusing to overwrite a report from another generator')
    campaigns = {}
    for path in root.rglob('*_results.json'):
        data = read_json(path, [])
        if isinstance(data, list):
            for row in data:
                if isinstance(row, dict) and row.get('name'):
                    campaigns[str((path.parent/row['name']).relative_to(root))] = row
    folders = set(path.parent for name in ('summary.json', 'runtime_settings.json', 'native_metadata.json', 'runtime_error.json')
                  for path in root.rglob(name))
    rows = [case_row(root, folder, campaigns.get(str(folder.relative_to(root)), {})) for folder in sorted(folders)]
    rows_by_directory={row['relative_directory']:row for row in rows}
    groups=[]
    for path in sorted(root.rglob('*_parallel_groups.json')):
        raw_groups=read_json(path,[])
        if isinstance(raw_groups,list):
            groups.extend(parallel_group_row(root,path,raw,rows_by_directory)
                          for raw in raw_groups if isinstance(raw,dict))
    comparisons = []
    for path in sorted(set(root.glob('*comparison.json'))|set(root.glob('*equivalence.json'))):
        data = read_json(path, {})
        entry=dict(file=path.name, **{key:data.get(key) for key in
            ('baseline', 'candidate', 'passed', 'optimization_physics_equivalent', 'diagnostic_only', 'interpretation',
             'total_workers','strict_pass_count','strict_fail_count','comparisons_total','comparisons_passed')})
        if 'total_workers' in data and 'strict_pass_count' in data and 'strict_fail_count' in data:
            entry['passed']=bool(data['total_workers']>0 and data['strict_pass_count']==data['total_workers']
                                 and data['strict_fail_count']==0)
        elif 'comparisons_total' in data and 'comparisons_passed' in data:
            entry['passed']=bool(data['comparisons_total']>0 and data['comparisons_passed']==data['comparisons_total'])
        comparisons.append(entry)
    evidence = [path.name for path in sorted(root.glob('*.json')) if path.name not in
                ('performance_summary.json',) and not path.name.endswith(('_results.json', '_resources.json'))]
    for pattern in ('offline_live_stream_validation_v2/offline_validation.json',
                    'offline_plant_preflight/plant_*_fcl.json', '**/live_viewer_*.json',
                    '**/render_error.txt','*_render*.log'):
        evidence.extend(str(path.relative_to(root)) for path in sorted(root.glob(pattern)))
    for name in ('legal_layout_v2/manifest.json', 'legal_layout_v2/preservation.json',
                 'legal_layout_v2/screen.json', 'legal_motion_assembly/comparison_to_v5.json'):
        if (root/name).is_file():
            evidence.append(name)
    report = dict(schema=SCHEMA, generated_utc=datetime.now(timezone.utc).isoformat(), source_root=str(root),
        generator_sha256=sha(Path(__file__)), training_eligible=False, harvest_success=None,
        counts=dict(Counter(row['status'] for row in rows)), trials=rows, parallel_groups=groups,
        comparisons=comparisons,
        evidence_files=sorted(set(evidence)),
        visual_artifacts=sorted([str(path.relative_to(root)) for suffix in ('*.png','*.mp4') for path in root.glob(suffix)]),
        interpretation='No global speed ranking: incomplete/invalid runs excluded, physical variants and component fixtures explicit')
    (output/'performance_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    columns = ['name','status','timing_comparable','scope','device','physics_only','native_device_matches_requested',
               'hz','solver','environments','plant_dofs','plant_dynamics',
               'position_iterations','contact_backend','collision_filter','simulated_s','aggregate_simulated_s','loop_wall_s',
               'physics_initialization_s','startup_and_other_s','save_wall_s','process_total_wall_s',
               'accepted_aggregate_sim_s_per_wall_s','target_displacement_max_mm','min_contact_separation_mm','model_sha256']
    with (output/'results.csv').open('w',newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    (output/'index.html').write_text(render_page(report, root, output))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root',type=Path)
    parser.add_argument('--output',type=Path,help='defaults to source root; generated report can be refreshed')
    args = parser.parse_args()
    result = generate(args.root, args.output)
    print(json.dumps(dict(trials=len(result['trials']), counts=result['counts'], output=str(args.output or args.root)),indent=2))
