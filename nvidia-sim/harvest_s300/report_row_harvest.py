"""Aggregate existing native trials and finite rail screens; never run physics.

Exact native joint breaks are harvest evidence. Motion completion, partial
detachment, collection, sampled planning and actual rail travel stay separate.
Run this report again after new summaries/videos arrive; no background job is
started and no source result is rewritten.
"""
import argparse
from collections import Counter
from datetime import datetime,timezone
import html
import json
from pathlib import Path
from urllib.parse import quote

import numpy as np


def _read(path,warnings):
    if not path.exists():return None
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (ValueError,OSError) as error:
        warnings.append(dict(path=str(path),error=str(error)));return None


def _rel(path,root):
    try:return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:return None


def _native_inventory(sequence,manifest,breaks,mode,time_s):
    if mode!='harvest':return None
    if sequence:
        names=list(sequence.get('fruit_names',[]));mapping=sequence.get('fruit_joints',{})
    else:
        target=manifest.get('target',{}).get('name','')
        truss=target.split('_Fruit_')[0]
        selected=[f for f in manifest.get('plant',{}).get('fruits',[]) if f.get('name','').startswith(truss+'_Fruit_')]
        names=[f['name'] for f in selected];mapping={f['name']:f['joint'] for f in selected}
    declared={f.get('name'):f.get('joint') for f in manifest.get('plant',{}).get('fruits',[])}
    if (not names or len(set(names))!=len(names) or any(name not in mapping for name in names) or
            len({mapping.get(name) for name in names})!=len(names) or
            any(not isinstance(mapping.get(name),str) or not mapping[name].startswith('/World/') for name in names) or
            declared and any(declared.get(name)!=mapping[name] for name in names)):
        return dict(exact_mapping_valid=False,detached_count=None,total_count=len(names),remaining=None)
    valid=[];invalid=[]
    for event in breaks:
        t=event.get('t')
        if not isinstance(t,(int,float)) or not np.isfinite(t) or t<0 or (time_s is not None and t>time_s+1e-9):
            invalid.append(event);continue
        valid.append(event)
    joint_times={event['joint']:event['t'] for event in valid if 'joint' in event}
    detached=[name for name in names if mapping[name] in joint_times]
    remaining=[name for name in names if name not in detached]
    selected_joints=set(mapping.values())
    imported=[event for event in valid if event.get('joint') in selected_joints and
              event.get('type')=='imported_native_prior' and event.get('native_new_event') is False]
    initial=[event for event in valid if event.get('joint') in selected_joints and
             event.get('phase') in ('idle','settle','warmup') and event not in imported]
    initial_names=[name for name in names if mapping[name] in {event['joint'] for event in initial}]
    imported_names=[name for name in names if mapping[name] in {event['joint'] for event in imported}]
    new_names=[name for name in detached if name not in imported_names]
    declared=sequence.get('inventory',{}) if sequence else {}
    return dict(exact_mapping_valid=True,total_count=len(names),detached_count=len(detached),
                detached=detached,remaining=remaining,fruit_joints=mapping,
                imported_prior_detached=imported_names,imported_prior_detached_count=len(imported_names),
                new_native_detached=new_names,new_native_detached_count=len(new_names),
                initial_uncommanded_detached=initial_names,initial_uncommanded_detached_count=len(initial_names),
                initial_uncommanded_break_events=initial,
                harvest_evidence_usable=not initial_names,
                invalid_timestamp_events=invalid,
                out_of_scope_breaks=[event for event in valid if event.get('joint') not in set(mapping.values())],
                sequence_inventory_matches_native=(not declared or
                    set(declared.get('detached',[]))==set(detached) and set(declared.get('remaining',[]))==set(remaining)),
                all_selected_joints_detached=not remaining,
                collection_verified=False,harvest_success=None,
                interpretation='Exact native joint-break inventory only; initialization breaks are not harvest evidence; imported checkpoint breaks are prior evidence, not new callbacks; partial detachment is not full-truss harvest or verified collection')


def _rail_measurement(folder,manifest,warnings):
    rail=manifest.get('rail')
    if not rail or not (folder/'states.npz').exists():return None
    metadata=_read(folder/'native_metadata.json',warnings)
    if not metadata:return dict(status='native_mapping_unavailable')
    try:
        with np.load(folder/'states.npz',allow_pickle=False) as states:
            times=states['time'];q=states['robot_q'];names=metadata['robot_dofs']
            index=names.index(rail['dof_name'])
            if q.ndim!=2 or q.shape[1]!=len(names) or len(q)!=len(times) or not len(q):
                raise ValueError('Native rail DOF/trace mapping mismatch')
            values=q[:,index]
            if not np.isfinite(values).all() or not np.isfinite(times).all():raise ValueError('Nonfinite rail trace')
            initial=float(rail['initial_station_y_m'])
            result=dict(status='measured_native_physical_rail_dof',dof_name=rail['dof_name'],
                        start_time_s=float(times[0]),end_time_s=float(times[-1]),
                        start_relative_m=float(values[0]),end_relative_m=float(values[-1]),
                        measured_travel_m=float(values[-1]-values[0]),
                        measured_range_relative_m=[float(values.min()),float(values.max())],
                        final_station_y_m=initial+float(values[-1]),
                        source='states.npz robot_q in exact native_metadata robot_dofs order')
            if len(times)>1:
                dt=np.diff(times)
                if np.any(dt<=0):raise ValueError('Nonincreasing rail timestamps')
                result['maximum_measured_speed_m_s']=float(np.max(np.abs(np.diff(values)/dt)))
            return result
    except (KeyError,ValueError,OSError) as error:return dict(status='measurement_unavailable',error=str(error))


def collect_results(root):
    root=Path(root).resolve();warnings=[];trials=[];case_results={};campaigns=[]
    folders={folder.resolve() for folder in root.glob('trials*/*') if folder.is_dir()}
    # Actual route initialization controls are nested under their own result
    # directory. Runtime settings identify recorded executions; do not treat
    # assembly inputs or copied Python examples as an executed trial.
    for path in root.rglob('runtime_settings.json'):
        if any(parent.name.startswith('trials') for parent in path.parents if parent!=root and parent.is_relative_to(root)):
            folders.add(path.parent.resolve())
    for path in sorted(root.rglob('row_campaign.json')):
        data=_read(path,warnings)
        if not data or data.get('schema')!='s300-physical-rail-row-campaign-v1':continue
        records=[]
        for station in data.get('stations',[]):
            index=station.get('index');plant=station.get('plant_id')
            if not isinstance(index,int) or index<0 or not isinstance(plant,int) or plant<0:
                warnings.append(dict(path=str(path),error='Invalid declared station index/plant identity'));continue
            name=f'station_{index:03d}_plant_{plant:02d}'
            folder=(path.parent/'trials'/name/name).resolve()
            if not folder.is_relative_to(root):
                warnings.append(dict(path=str(path),error='Declared campaign trial escapes report root'));continue
            if folder.exists():folders.add(folder)
            checkpoint=station.get('checkpoint')
            records.append(dict(index=index,plant_id=plant,station_y_m=station.get('station_y_m'),
                status=station.get('status'),physics_execution=station.get('physics_execution'),
                blocker=station.get('blocker'),trial=_rel(folder,root) if folder.exists() else None,
                checkpoint=_rel(Path(checkpoint)/'checkpoint.json',root) if checkpoint else None,
                window_handoff=station.get('window_handoff'),native_inventory=station.get('native_inventory')))
        campaigns.append(dict(source=_rel(path,root),executed=data.get('executed'),status=data.get('status'),
            selection=data.get('selection'),stations=records,native_continuity=data.get('native_continuity'),
            initial_scene_sha256=data.get('initial_scene_sha256'),inventory_sha256=data.get('inventory_sha256'),
            screen_sha256=data.get('screen_sha256'),screen_is_proposal_only=data.get('screen_is_proposal_only'),
            training_eligible=False,collection_verified=False,harvest_success=None))
    result_paths=set(root.glob('trials*/*_results.json'))
    for folder in folders:result_paths.update(folder.parent.glob('*_results.json'))
    for path in sorted(result_paths):
        entries=_read(path,warnings)
        if isinstance(entries,list):
            for entry in entries:
                if isinstance(entry,dict) and entry.get('name'):
                    case_results[str((path.parent/entry['name']).resolve())]=entry
    for folder in sorted(folders):
        if not folder.is_dir() or not any((folder/name).exists() for name in
                ('summary.json','runtime_settings.json','runtime_error.json','cancelled.json','plan.json')):continue
        summary=_read(folder/'summary.json',warnings)
        initialization_failure=_read(folder/'initialization_failure.json',warnings)
        runtime=_read(folder/'runtime_settings.json',warnings) or {}
        native_stage=_read(folder/'native_stage_selection.json',warnings) or {}
        sequence=_read(folder/'truss_sequence.json',warnings) or (summary or {}).get('truss_sequence')
        manifest={}
        source_scene=(summary or {}).get('scene') or native_stage.get('source_scene')
        if source_scene:
            manifest=_read(Path(source_scene).parent/'motion_manifest.json',warnings) or {}
        lifecycle=_read(folder/'contact_lifecycle_scope.json',warnings) or (summary or {}).get('contact_lifecycle_scope') or {}
        geometry=_read(folder/'leaf_contact_resolution.json',warnings) or []
        breaks=_read(folder/'break_events.json',warnings)
        if breaks is None:breaks=(summary or {}).get('native_break_events')
        if breaks is None:breaks=(sequence or {}).get('native_break_events')
        if breaks is None:breaks=(initialization_failure or {}).get('native_break_events',[])
        mode=(summary or {}).get('mode')
        # A final evidence write can fail after the actual loop saved its trace.
        # The exact saved sequence/native events remain usable as partial
        # evidence; they do not turn a failed trial into a completed success.
        if mode is None and sequence and sequence.get('fruit_joints') and 'native_break_events' in sequence:
            mode='harvest'
        simulated_s=(summary or {}).get('simulated_s')
        if simulated_s is None and (folder/'states.npz').exists():
            try:
                with np.load(folder/'states.npz',allow_pickle=False) as state:
                    times=state['time']
                    if len(times) and np.isfinite(times).all() and np.all(np.diff(times)>0):
                        simulated_s=float(times[-1])
            except (KeyError,ValueError,OSError) as error:
                warnings.append(dict(path=str(folder/'states.npz'),error=str(error)))
        case=case_results.get(str(folder),{})
        status=('complete' if summary and summary.get('completed') else
                'stopped' if summary else 'runtime_error' if (folder/'runtime_error.json').exists() or
                    case.get('returncode') is not None and case['returncode']!=0 else
                'cancelled' if (folder/'cancelled.json').exists() else 'not_finalized')
        inventory=_native_inventory(sequence or {},manifest,breaks,mode,simulated_s)
        if any(event.get('phase') in ('idle','settle','warmup') and
               event.get('type')!='imported_native_prior' for event in breaks):status='initial_state_failure'
        if initialization_failure:status='initial_state_failure'
        evidence={}
        for name in ('summary.json','truss_sequence.json','break_events.json','states.npz','plan.json',
                     'contacts.json','runtime_settings.json','runtime_error.json','initialization_validity.json',
                     'native_metadata.json','solved_contact_policy.json','leaf_contact_resolution.json',
                     'contact_lifecycle_scope.json','self_collision_actual.json','route_checkpoint/checkpoint.json',
                     'native_stage_selection.json','resume_input.json','resume_native_validation.json'):
            if (folder/name).exists():evidence[name]=_rel(folder/name,root)
        for name in ('initialization_failure.json','pre_restore_initialization.json',
                     'native_initialization_timing.json','invalid_initialization_state.npz',
                     'invalid_initialization_geometry.json'):
            if (folder/name).exists():evidence[name]=_rel(folder/name,root)
        log=folder.parent/(folder.name+'.log')
        if log.exists():evidence['log']=_rel(log,root)
        cycle_plans=(sequence or {}).get('cycle_plans',[])
        selected_evidence=(_read(folder/'selected_cycle_evidence_phase_corrected.json',warnings) or
                           _read(folder/'selected_cycle_evidence.json',warnings) or
                           (summary or {}).get('selected_cycle_evidence'))
        if (folder/'selected_cycle_evidence_phase_corrected.json').exists():
            evidence['selected_cycle_evidence_phase_corrected.json']=_rel(folder/'selected_cycle_evidence_phase_corrected.json',root)
        if not selected_evidence and sequence and manifest.get('plant',{}).get('root'):
            try:
                from selected_cycle_evidence import build_selected_cycle_evidence
                selected_evidence=build_selected_cycle_evidence(manifest,sequence,breaks,legacy_summary=summary)
            except (KeyError,ValueError) as error:
                warnings.append(dict(path=str(folder),error='Selected cycle identity unavailable: '+str(error)))
        legacy_fields=dict(target_name=manifest.get('target',{}).get('name'),
            native_target_detached=(summary or {}).get('native_target_detached'),
            target_displacement_max_m=(summary or {}).get('target_displacement_max_m'),
            target_displacement_final_m=(summary or {}).get('target_displacement_final_m'),
            target_force_max_n=(summary or {}).get('target_force_max_n'),
            scope='Original manifest target only; never a different selected cycle fruit or group')
        if (folder/'selected_cycle_evidence.json').exists():
            evidence['selected_cycle_evidence.json']=_rel(folder/'selected_cycle_evidence.json',root)
        for cycle in cycle_plans:
            if cycle.get('plan') and (folder/cycle['plan']).exists():
                evidence['cycle_'+str(cycle.get('index'))+'_plan']=_rel(folder/cycle['plan'],root)
        videos=[]
        for video in sorted(folder.glob('*.mp4')):
            render=_read(video.with_name(video.stem+'_render.json'),warnings) or {}
            videos.append(dict(path=_rel(video,root),name=video.name,speed=render.get('speed'),
                               actual_saved_physics=render.get('source')=='stored measured physical body transforms',
                               actual_clip_start_s=render.get('actual_clip_start_s'),
                               actual_clip_end_s=render.get('actual_clip_end_s'),
                               source_trial_stop_time_s=render.get('source_trial_stop_time_s'),
                               camera=render.get('camera'),
                               render_metadata=_rel(video.with_name(video.stem+'_render.json'),root) if render else None))
        trials.append(dict(trial=_rel(folder,root),status=status,mode=mode,
            completed=(summary or {}).get('completed'),stop_reason=(summary or {}).get('stop_reason') or (sequence or {}).get('termination') or (initialization_failure or {}).get('reason'),
            source_plant=manifest.get('plant',{}).get('source_plant'),
            source_scene=source_scene,source_scene_sha256=runtime.get('source_scene_sha256'),
            native_scene=native_stage.get('native_scene') or runtime.get('native_scene'),
            native_scene_sha256=native_stage.get('native_scene_sha256') or runtime.get('native_scene_sha256'),
            native_stage_selection=native_stage.get('mode'),
            leaf_collision_geometry=manifest.get('leaf_collision_geometry'),
            native_resume=(summary or {}).get('native_resume'),
            simulated_s=simulated_s,physics_wall_s=(summary or {}).get('physics_wall_s'),
            planning_paused_wall_s=(sequence or {}).get('planning_paused_wall_s'),
            sequence_termination=(sequence or {}).get('termination'),native_inventory=inventory,
            cycle_plans=cycle_plans,failed_cycle_plans=[cycle for cycle in cycle_plans if
                cycle.get('status') in ('planning_rejected','planning_failed') or cycle.get('error')],
            selected_cycle_evidence=selected_evidence,legacy_manifest_target_fields=legacy_fields,
            initialization_failure=initialization_failure,
            active_harvest_guard=dict(limit_mm=.5,completed_without_penetration_stop=bool(summary and
                summary.get('completed') and summary.get('stop_reason') is None),
                solved_checks=len(geometry),within_limit_checks=sum(bool(row.get('certified_within_limit')) for row in geometry),
                failed_or_unverified_checks=sum(not row.get('certified_within_limit') for row in geometry),
                max_certified_pair_bound_mm=max([row['penetration_upper_bound_m']*1000 for row in geometry
                    if row.get('certified_within_limit') and row.get('penetration_upper_bound_m') is not None],default=None),
                scope='Recorded active robot/plant guard and performed motions; not a universal scene-validity certificate'),
            raw_callback_max_penetration_mm=(max(0.,-summary['min_contact_separation_m'])*1000
                if summary and summary.get('min_contact_separation_m') is not None else None),
            released_collection_diagnostics=dict(enabled=lifecycle.get('enabled'),
                deferred_contact_points=lifecycle.get('deferred_invalid_contact_points',0),
                strict_full_scene_0_5mm_passed=lifecycle.get('strict_0_5mm_passed'),
                collection_verified=False,released_fruit_physics_verified=False),
            rail=_rail_measurement(folder,manifest,warnings),videos=videos,evidence=evidence,
            returncode=case.get('returncode'),total_wall_s=case.get('total_wall_s'),
            summary_mtime=(folder/'summary.json').stat().st_mtime if summary else 0.))
    inventory=_read(root/'rail_near_inventory.json',warnings) or {}
    station=_read(root/'row_station_screen/report.json',warnings) or {}
    compact_station=dict(source='row_station_screen/report.json',total_queries=station.get('total_queries'),
        status_counts=station.get('status_counts',{}),wall_s=station.get('wall_s'),
        scope=station.get('scope'),side_excluded_count=len(station.get('side_excluded',[])),
        budget_unselected_count=len(station.get('budget_unselected',[])),
        results=station.get('results',[]),interpretation='Finite IK/self-collision screening; no absolute reachability or harvest claim. Environment audit/physics require separate evidence')
    host=Path(str(root).replace('/root/docker_share/','/home/rdv/docker_share/',1))/'index.html'
    return dict(schema='s300-row-harvest-results-v1',updated_utc=datetime.now(timezone.utc).isoformat(),
        host_url=host.as_uri(),training_eligible=False,harvest_success=None,collection_verified=False,
        native_detachment_interpretation='Exact fruit-joint native break events; all attempts retained; repeated trials are not additive harvested-fruit totals',
        row_inventory=dict(source='rail_near_inventory.json',total_trusses=inventory.get('total_trusses'),
            side_selected_trusses=inventory.get('side_selected_trusses'),side_excluded_trusses=inventory.get('side_excluded_trusses'),
            policy=inventory.get('policy'),exclusions=[dict(plant_id=row.get('plant_id'),truss_index=row.get('truss_index'),
                reason=row.get('exclusion_reason')) for row in inventory.get('trusses',[]) if not row.get('selected_by_side_policy')]),
        active_plant_scope=_read(root/'active_plant_near_truss_scope.json',warnings),
        station_screen=compact_station,row_campaigns=campaigns,trials=trials,read_warnings=warnings)


def _link(path,label=None):
    if not path:return '—'
    return '<a href="'+html.escape(quote(path,safe='/'))+'">'+html.escape(label or path)+'</a>'


def _text(value):return html.escape(str(value)) if value is not None else '—'


def write_report(root,*,refresh_subreports=False):
    root=Path(root).resolve()
    if refresh_subreports:
        from report_motion import summarize
        for folder in sorted(root.glob('trials*')):
            if folder.is_dir() and all(not (trial/'summary.json').exists() or all((trial/name).exists() for name in
                ('states.npz','native_metadata.json','contacts.json')) for trial in folder.iterdir() if trial.is_dir()):
                try:summarize(folder)
                except (KeyError,ValueError,OSError) as error:print('Subreport left unchanged:',folder,error)
    report=collect_results(root)
    (root/'row_results.json').write_text(json.dumps(report,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
    ordered=sorted(report['trials'],key=lambda row:row['summary_mtime'],reverse=True)
    table=[];details=[]
    for row in ordered:
        inv=row['native_inventory'] or {};counts='—' if inv.get('detached_count') is None else f"분리 {inv['detached_count']}/{inv['total_count']} · 남음 {len(inv['remaining'])}"
        status={'complete':'동작 종료','stopped':'중단','runtime_error':'실행 오류','cancelled':'취소','not_finalized':'최종 결과 아직 없음','initial_state_failure':'초기 상태 파단 · 수확 증거 제외'}[row['status']]
        if inv.get('initial_uncommanded_detached_count',0):
            counts+=' · 초기 상태 파단 '+str(inv['initial_uncommanded_detached_count'])+'개'
        elif inv.get('imported_prior_detached_count',0):
            counts+=' · 이전 native '+str(inv['imported_prior_detached_count'])+'개 / 이번 '+str(inv['new_native_detached_count'])+'개'
        reason=row['stop_reason'] or row['sequence_termination']
        guard=row['active_harvest_guard'];collection=row['released_collection_diagnostics']
        evidence=' · '.join(_link(path,key) for key,path in row['evidence'].items())
        table.append('<tr>'+''.join('<td>'+_text(v)+'</td>' for v in
            [row['trial'],status,counts,reason,row['simulated_s'],row['physics_wall_s'],
             f"{guard['within_limit_checks']}/{guard['solved_checks']} solved 검사",
             round(row['raw_callback_max_penetration_mm'],3) if row['raw_callback_max_penetration_mm'] is not None else None,
             collection['deferred_contact_points']])+
            '<td>'+evidence+'</td></tr>')
        cycles=''.join('<tr>'+''.join('<td>'+_text(v)+'</td>' for v in
            [cycle.get('index'),', '.join(cycle.get('fruits',[])),cycle.get('status'),cycle.get('start_s'),
             cycle.get('end_s'),cycle.get('error') or cycle.get('stop_reason')])+ '</tr>' for cycle in row['cycle_plans'])
        selected_rows=[]
        for cycle in (row.get('selected_cycle_evidence') or {}).get('cycles',[]):
            for fruit in cycle['selected_fruits']:
                displacement=fruit.get('displacement_max_from_measured_cycle_baseline_m')
                selected_rows.append('<tr>'+''.join('<td>'+_text(v)+'</td>' for v in
                    [cycle['index'],fruit['name'],fruit['body'],fruit['native_joint'],
                     fruit['native_detached_at_cycle_start'],fruit['native_detached_at_cycle_end'],
                     round(displacement*1000,3) if displacement is not None else None,
                     fruit.get('max_single_contact_point_force_n')])+ '</tr>')
        legacy=row['legacy_manifest_target_fields']
        remaining=', '.join(inv.get('remaining') or [])
        details.append('<article><h2>'+_text(row['trial'])+'</h2><p>'+_text(counts)+' · 남은 열매: '+_text(remaining or '없음/해당 없음')+'</p>'+
            '<p>source plant: '+_text(row['source_plant'])+' · 계획 중 일시 정지 '+_text(row['planning_paused_wall_s'])+' s. '+
            '진행한 active 동작의 관통 중단 없음: '+_text(guard['completed_without_penetration_stop'])+
            ' · solved 검사 최대 pair 상한 '+_text(guard['max_certified_pair_bound_mm'])+' mm. 포집/낙과 물리 검증: false.</p>'+
            '<p>기존 summary의 target 필드는 원래 manifest 타깃 '+_text(legacy['target_name'])+' 기준입니다. '+
            'native_target_detached='+_text(legacy['native_target_detached'])+'와 기존 변위/힘을 다른 회차의 조준 열매 결과로 해석하지 않습니다.</p>'+
            ('<p>실제 레일 측정: <code>'+_text(json.dumps(row['rail'],ensure_ascii=False))+'</code></p>' if row['rail'] else '')+
            ('<table><tr><th>회차</th><th>조준 열매</th><th>상태</th><th>시작 s</th><th>끝 s</th><th>오류/중단</th></tr>'+cycles+'</table>' if cycles else '')+
            ('<p>아래는 각 회차에서 실제 선정한 열매의 정확한 native body/조인트입니다. 변위는 회차의 측정 기준 상태부터 계산하며 분리 후 낙하도 포함합니다. 표에 없는 측정값은 추정하지 않습니다.</p><table><tr><th>회차</th><th>선정 열매</th><th>native body</th><th>native joint</th><th>회차 전 분리</th><th>회차 종료 분리</th><th>최대 변위 mm</th><th>최대 점 접촉력 N</th></tr>'+''.join(selected_rows)+'</table>' if selected_rows else ''))
        for video in row['videos']:
            details.append('<h3>'+_text(video['name'])+'</h3><p>'+('실제 저장 native body pose 재생' if video['actual_saved_physics'] else '영상의 원자료 구분은 render JSON 확인')+
                ' · '+_text(video['speed'])+'배속 · 기록 시각 '+_text(video['actual_clip_start_s'])+'~'+_text(video['actual_clip_end_s'])+
                ' s · 원본 종료 시각 '+_text(video['source_trial_stop_time_s'])+' s · '+_link(video['render_metadata'],'render JSON')+
                '</p><p>낙과의 바닥/받침판 접촉이 포함될 수 있으며, 이 재생 영상은 포집이나 분리된 열매의 0.5mm 물리 유효성 검증이 아닙니다.</p><video controls preload="metadata" src="'+
                html.escape(quote(video['path'],safe='/'))+'"></video><p>'+_link(video['path'])+'</p>')
        details.append('</article>')
    campaign_parts=[]
    for campaign in report['row_campaigns']:
        rows=[]
        for record in campaign['stations']:
            rows.append('<tr>'+''.join('<td>'+_text(value)+'</td>' for value in
                [record['index'],record['plant_id'],record['station_y_m'],record['status'],
                 record['physics_execution'],record['blocker']])+
                '<td>'+_link(record['trial'],'실제 station 원자료')+' · '+_link(record['checkpoint'],'측정 checkpoint')+'</td></tr>')
        campaign_parts.append('<article><h3>'+_link(campaign['source'])+'</h3><p>실제 실행 요청: '+_text(campaign['executed'])+
            ' · 상태: '+_text(campaign['status'])+'</p><p>계획 표본은 제안만 제공합니다. 각 실제 station의 레일 DOF 측정과 동작 결과는 아래 실행 표에 따로 기록합니다. '+
            '창 전환은 측정 상태 handoff와 PhysX 재시작이며, 이전 비활성 줄기는 명시적인 동역학 근사로 보존합니다.</p>'+ 
            '<table><tr><th>station</th><th>줄기</th><th>계획 y m</th><th>상태</th><th>실제 실행</th><th>blocker</th><th>원자료</th></tr>'+''.join(rows)+'</table></article>')
    station=report['station_screen'];screen_rows=[]
    for item in station['results']:
        for query in item.get('queries',[]):
            block=query.get('first_block') or {};screen_rows.append('<tr>'+''.join('<td>'+_text(v)+'</td>' for v in
                [item.get('identity'),query.get('index'),query.get('station_y_m'),query.get('status'),
                 query.get('environment_audit'),query.get('physics_execution'),block.get('phase')])+ '</tr>')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>S300 연속 송이·레일 수확 시험</title>
<style>body{font:16px system-ui;background:#141a22;color:#edf1f7;margin:32px auto;padding:0 24px;max-width:1600px;line-height:1.6}a{color:#83cdff}table{border-collapse:collapse;font-size:13px;display:block;overflow:auto}td,th{padding:9px;border:1px solid #435064;vertical-align:top}code{overflow-wrap:anywhere}video{width:min(100%,960px)}article{margin:32px 0}.note{background:#333628;padding:18px}.url{overflow-wrap:anywhere}</style>
<h1>S300 연속 송이·레일 수확 시험</h1>'''+ '<p>갱신 UTC: '+_text(report['updated_utc'])+'</p><p class="url">호스트에서 여는 주소: <a href="'+html.escape(report['host_url'])+'">'+_text(report['host_url'])+'</a></p>'+ \
        '<p>'+_link('row_results.json')+' · '+_link('rail_near_inventory.json','레일 가까운 줄기/반대편 제외 목록')+' · '+_link('active_plant_near_truss_scope.json','현재 줄기의 송이 선정/제외')+' · '+_link('row_station_screen/report.json','레일 위치별 모든 계획 검사')+'</p>'+ \
        ('<p>'+_link('COLLISION_ACCURACY_AND_NATIVE_QUERY_KO.md','충돌 형상·native 질의 검증 설명')+'</p>' if (root/'COLLISION_ACCURACY_AND_NATIVE_QUERY_KO.md').exists() else '')+ \
        '<div class="note">숫자는 정확한 열매 조인트의 native 파단 수입니다. 일부 열매의 분리를 송이 전체 수확 성공으로 표시하지 않습니다. 분리된 열매의 바닥/받침판 충돌은 별도 진단으로 보존하며 포집 성공은 미검증입니다. raw callback 관통과 post-solve 기하 상한은 측정 시점과 계산 방식이 다릅니다. 반복 실행의 분리 수는 합산하지 않습니다. 모든 데이터는 training_eligible=false입니다.</div>'+ \
        '<h2>전체 실제 실행</h2><p>active robot/plant는 기존 0.5mm 기준을 유지합니다. 아래 solved 검사는 의심 접촉의 별도 재검증 수이며 전체 scene PASS 증명이 아닙니다. 낙과 진단으로 넘긴 접촉에는 raw 0.5mm 초과가 남아 있습니다.</p>'+ \
        '<p>실행 loop wall에는 실행 도중의 다음 동작 계획 대기 시간이 포함됩니다. 순수 물리 계산 시간이나 실시간 배속으로 해석하지 않습니다.</p>'+ \
        '<table><tr><th>실행</th><th>상태</th><th>정확한 native 분리</th><th>종료 원인</th><th>sim s</th><th>실행 loop wall s</th><th>0.5mm solved 확인</th><th>raw 최대 mm</th><th>낙과 진단 점 수</th><th>원자료/계획/로그</th></tr>'+''.join(table)+'</table>'+ \
        '<h2>연속 레일 수확 campaign</h2>'+(''.join(campaign_parts) or '<p>아직 campaign 원자료가 없습니다.</p>')+ \
        '<h2>레일 가까운 줄기와 유한 계획 검사</h2><p>송이 총 '+_text(report['row_inventory'].get('total_trusses'))+' · 가까운 줄기 선정 '+_text(report['row_inventory'].get('side_selected_trusses'))+' · 반대편 제외 '+_text(report['row_inventory'].get('side_excluded_trusses'))+ \
        '. 계획 질의 '+_text(station['total_queries'])+' · 상태 집계 '+_text(station['status_counts'])+'. 표본 계획 통과는 절대 도달 가능성이나 실제 수확 완료를 뜻하지 않습니다. 미선정 표본 '+_text(station['budget_unselected_count'])+'개도 원자료에 남겼습니다.</p>'+ \
        '<details><summary>모든 레일 위치 계획 결과</summary><table><tr><th>송이</th><th>질의</th><th>레일 y m</th><th>결과</th><th>환경 검사</th><th>physics 실행</th><th>차단 단계</th></tr>'+''.join(screen_rows)+'</table></details>'+''.join(details)+ \
        '<p>새 실행/영상이 저장되면 report_row_harvest.py를 다시 실행하여 갱신합니다. 읽기 경고: '+_text(len(report['read_warnings']))+'</p></html>'
    (root/'index.html').write_text(page,encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    parser.add_argument('--refresh-subreports',action='store_true',help='Optional existing report_motion.summarize for finalized nested trial groups')
    args=parser.parse_args();result=write_report(args.root,refresh_subreports=args.refresh_subreports)
    print(json.dumps(dict(output=str(args.root/'index.html'),host_url=result['host_url'],
                         trials=len(result['trials']),videos=sum(len(row['videos']) for row in result['trials'])),ensure_ascii=False))
