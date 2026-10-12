"""Summarize the bounded one-truss experiment, retaining failed controls."""
import argparse
import hashlib
import html
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import mujoco as mj


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def write_trial_report(folder):
    """Always leave a directly readable index beside the native run evidence."""
    folder=Path(folder);result=json.loads((folder/'summary.json').read_text())
    table=''.join('<tr>'+''.join('<td>'+html.escape(str(row.get(key,'')))+'</td>'
        for key in ('window','cycle','selected_group','completed','termination','stop'))+'</tr>'
        for row in result['cycles'])
    video=''.join(f'<p>{html.escape(str(p))}</p><video controls preload="metadata" src="{html.escape(p.name)}"></video>'
        for p in sorted(folder.glob('actual*.mp4')))
    text=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>Native S300 harvest trial</title>
<style>body{{font:16px sans-serif;margin:24px}}table{{border-collapse:collapse}}td,th{{border:1px solid #aaa;padding:8px}}video{{width:min(960px,100%)}}p{{overflow-wrap:anywhere}}</style>
<h1>S300 MuJoCo 실제 실행</h1>
<p>MuJoCo {result['mujoco_version']} / sim {result['simulated_s']:.2f}s / loop {result['loop_wall_s']:.2f}s /
RTF {result['realtime_factor']:.2f} / 분리 {len(result['breaks'])}개 / 중단 {html.escape(str(result['stop']))}</p>
<p>활동 겹침 최대 {result['active_contact_penetration_m']*1000:.3f}mm.
분리 후 충돌 최대 {result['released_penetration_m']*1000:.3f}mm는 별도 기록.
training_eligible=false / harvest_success=null / 수거·실물 안전성 미검증.</p>
<table><tr><th>식물</th><th>진입</th><th>선택 집단</th><th>완주</th><th>선택 종료</th><th>물리 중단</th></tr>{table}</table>
<p><a href="summary.json">결과</a> / <a href="contacts.json">실제 접촉</a> /
<a href="break_events.json">힘·토크 분리</a> / <a href="phase_events.json">단계·시간</a> /
<a href="invocation.json">입력·코드 해시</a></p>
<p>영상은 저장 qpos를 재생합니다. GUI: <code>./mujoco-benchmark/run_s300_mujoco.sh replay {html.escape(str(folder))}</code></p>
{video}</html>'''
    (folder/'index.html').write_text(text)


def build(root):
    folders=['idle315_discrete60','inward60','verified_inward120','inward240','off120']
    rows=[];repeats={};videos=[]
    for name in folders:
        p=root/name
        result=json.loads((p/'summary.json').read_text())
        for s in result['runs']:
            folder=p/f'repeat_{s["repeat"]:02d}'
            # Reclassify against solver timestamps, before newly triggered release.
            # Early raw summaries used end-of-step release state for that one step.
            m=mj.MjModel.from_binary_path(str(p/'model.mjb'))
            release_times={b['name']:b['release_effective_time_s'] for b in s['breaks']}
            active_force=0.;released_force=0.;active_max=0.;release_max=0.;largest_force=None
            for t,g1,g2,dist,force,_ in json.loads((folder/'contacts.json').read_text())['rows']:
                bodies=[m.body(m.geom_bodyid[g]).name for g in (g1,g2)]
                released=any(t>=release_times.get(b,1e99)-1e-9 for b in bodies)
                pen=max(0.,-dist)
                if released:release_max=max(release_max,pen)
                else:active_max=max(active_max,pen)
                if any(b.startswith('Tool') for b in bodies):
                    if released:released_force=max(released_force,force)
                    else:active_force=max(active_force,force)
                    if largest_force is None or force>largest_force['force_n']:
                        largest_force=dict(t=t,bodies=bodies,force_n=force,penetration_m=pen,released=released)
            # All >1mm plant contacts were recorded; below-threshold maximum remains
            # available in the per-step raw metric even when not in contacts.json.
            active_max=max(active_max,s['active_tool_penetration_m'],s['active_plant_penetration_m'])
            rows.append(dict(experiment=name,**s,states_sha256=digest(folder/'states.npz'),
                             timestamp_reclassified=dict(active_penetration_m=active_max,released_penetration_m=release_max,
                                 active_tool_force_n=active_force,released_tool_force_n=released_force,largest_tool_force=largest_force)))
        if len(result['runs'])>1:
            a=np.load(p/'repeat_00/states.npz');b=np.load(p/'repeat_01/states.npz')
            repeats[name]={k:bool(np.array_equal(a[k],b[k])) for k in a.files}
        for video in p.rglob('*.mp4'):
            if video.with_name(video.stem+'_visible.mp4').exists():
                # Preserve the original black export as evidence, but link the
                # corrected replay. Do not present both as different trials.
                continue
            videos.append(dict(path=str(video),sha256=digest(video)))
    meta=json.loads((root/'model315/export.json').read_text())
    report=dict(schema='s300-mujoco-315-smoke-v1',root=str(root),results=rows,
                reproducibility=repeats,videos=videos,source=meta,
                training_eligible=False,harvest_success=None,full_physics_validated=False,
                conclusion='120 Hz selected-truss smoke runs faster than real time; full robot migration/equivalence is not established.',
                historical_attempts={
                    'idle240, idle240_implicit, idle315_960':'Pre-discrete attachment instability; all fruit released at idle. Rejected.',
                    'inward315_discrete60, inward315_discrete240':'API sign copied before measured PhysX/MuJoCo direction mapping. Actually outward; excluded from final inward comparison.',
                    'inward120':'First correctly directed 120Hz run. Final verified_inward120 repeats save code hashes and use start-of-step release classification.',
                },
                limitations=[
                    'Only one truss, main stem and prescribed gripper; no robot arm, lift, base, farm, robot tracking or arm collisions.',
                    'Per-mesh tool convex hulls differ from PhysX decomposition; collider equivalence not established.',
                    'Fruit-neck soft spring/break model and fruit inertia are uncalibrated porting approximations.',
                    'Prescribed mocap normal velocity and finite robot inertia are absent; native surfacevel covers tangent transport.',
                    'Gravity settlement differs from original three-truss continuation; initial poses are source geometry.',
                    '60Hz violates the retained 1mm active-contact criterion and releases nine fruit vs four at 120/240Hz. Not a valid speed setting.',
                    'Detached fruit/floor penetration is large, separately reported, not hidden as validated physics.',
                    'Prescribed jaw poses do not yield to new contact: released fruit tool-force spikes reach 315N at 120Hz and 1170N at 240Hz. Not a realistic gripper load result.',
                    'The 36.8s source trace stops during final opening. This is not a completed withdrawal or whole-truss harvest.',
                    'Belt-off still releases fruit by jaw/branch loading. Separation alone does not prove belt rotation harvesting.',
                ])
    render_fix=root/'render_fix_validation.json'
    if render_fix.exists():report['render_fix']=json.loads(render_fix.read_text())
    (root/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    table=''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in [
        s['experiment']+f' #{s["repeat"]}',s['hz'],f'{s["loop_wall_s"]:.2f}',f'{s["realtime_factor"]:.2f}',
        ', '.join(x['name'].rsplit('_',1)[1] for x in s['breaks']) or '없음',
        f'{max(s["active_tool_penetration_m"],s["active_plant_penetration_m"])*1000:.3f}',
        f'{s["released_penetration_m"]*1000:.2f}'
    ])+'</tr>' for s in rows)
    movie=''.join(f'<section><h3>{html.escape(Path(v["path"]).parent.parent.name)} / {html.escape(Path(v["path"]).name)}</h3>'
                  f'<video controls preload="metadata" src="{html.escape(str(Path(v["path"]).relative_to(root)))}"></video>'
                  f'<p>{html.escape(v["path"])}</p></section>' for v in videos)
    text=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>MuJoCo 3.15 송이 하나 시험</title>
<style>body{{font:16px sans-serif;margin:30px;background:#f5f6f8;color:#17222e}}table{{border-collapse:collapse;background:white}}td,th{{border:1px solid #ccc;padding:8px}}video{{width:min(960px,100%)}}p{{overflow-wrap:anywhere}}section{{margin-top:28px}}.note{{background:#fff0cf;padding:16px}}</style>
<h1>MuJoCo 3.15.0 — 현재 벨트 그리퍼 + 송이 하나</h1>
<p>2026-10-12 / SIM-GT 진단 / training_eligible=false / harvest_success=null</p>
<p class="note">120Hz는 실시간보다 빠르게 동작하고 4개 분리. 60Hz는 빠르지만 관통 기준 초과 및 결과 차이로 채택 불가.
로봇 팔·리프트·기체·농장 전체를 이식한 결과가 아닙니다. 낙하 열매/바닥 관통과 충돌 분해·연결부 모델 차이가 남아 있습니다.
기록 자세를 강제 재생하는 그리퍼에서 분리된 열매 접촉력이 315N(120Hz), 1170N(240Hz)까지 관측되어, 이 힘과 수확 결과를 실물 성능으로 사용할 수 없습니다.</p>
<table><tr><th>시험 / 반복</th><th>Hz</th><th>실행 초</th><th>실시간 배율</th><th>분리 번호</th><th>부착 상태 최대 겹침 mm</th><th>분리 후 최대 겹침 mm</th></tr>{table}</table>
<p>주행 조건은 모두 저장된 36.8초 동작. idle만 5초. 속도는 상태 기록·접촉 검사 포함, 로딩·영상 렌더 제외.
영상을 틀 때 물리를 다시 실행하지 않습니다. 아래는 실제 qpos의 1배속 재생입니다.</p>
<p>영상 수정: 초기 actual_top.mp4는 카메라·조명 변환 갱신 누락으로 검게 저장됐습니다.
아래 actual_top_visible.mp4는 같은 물리 기록에서 다시 렌더링한 영상입니다.
<a href="render_fix_validation.json">프레임·원본 상태 해시 검사</a></p>
{movie}<h2>한계</h2><ul>{''.join('<li>'+html.escape(x)+'</li>' for x in report['limitations'])}</ul>
<p><a href="validation.json">원자료·해시·재현성·전체 시험표 JSON</a></p></html>'''
    (root/'index.html').write_text(text)
    print(json.dumps(dict(reproducibility=repeats,videos=videos),indent=2))
    return report


def build_harvest(root,primary):
    """Report dynamic-robot trials without promoting separation to collection."""
    root=Path(root);primary=Path(primary)
    primary=primary if primary.is_absolute() else root/primary
    selected=json.loads((primary/'summary.json').read_text())
    trials=[]
    for path in sorted(root.glob('campaign*/summary.json')):
        result=json.loads(path.read_text())
        model_xml=Path(result['model_dir'])/'scene.xml'
        if digest(model_xml)!=result['source_model_xml_sha256']:
            raise ValueError('Trial/report model hash mismatch: '+str(model_xml))
        geom=ET.parse(model_xml).getroot().find('default/geom')
        corrected=(geom is not None and float(geom.get('margin','0'))==0. and
                   float(geom.get('gap','0'))==.0005)
        # The first diagnostic recorded only active tool pairs. Preserve its
        # scope rather than retroactively calling it an all-active maximum.
        metric_key=('active_contact_penetration_m' if 'active_contact_penetration_m' in result
                    else 'active_tool_penetration_m')
        trials.append(dict(trial=path.parent.name,summary=str(path),
            simulated_s=result['simulated_s'],loop_wall_s=result['loop_wall_s'],
            realtime_factor=result['realtime_factor'],stop=result['stop'],
            detached=len(result['breaks']),active_penetration_mm=1000*result[metric_key],
            active_metric_scope='all_active' if metric_key=='active_contact_penetration_m' else 'tool_only',
            released_penetration_mm=1000*result['released_penetration_m'],
            max_contact_force_n=result['max_contact_force_n'],
            active_contact_criterion_passed=result['active_contact_criterion_passed'],
            contact_mapping_corrected=corrected,
            first_violation=result['first_1mm_violation'],states_sha256=result['states_sha256']))
    videos=[]
    for video in sorted(root.glob('campaign*/actual*.mp4')):
        info=video.with_suffix('.json')
        videos.append(dict(path=str(video),sha256=digest(video),
                           replay=json.loads(info.read_text()) if info.exists() else None))
    per_window={}
    for event in selected['breaks']:
        per_window.setdefault(event['window'],[]).append(event['name'])
    eligibility={}
    metadata_path=Path(selected['model_dir'])/'export.json'
    if metadata_path.exists():
        metadata=json.loads(metadata_path.read_text())
        for window in metadata.get('windows',[]):
            policy_path=Path(window['source_trial'])/'harvest_group_policy.json'
            policy=json.loads(policy_path.read_text())
            for fruit in policy['fruit_eligibility']:
                eligibility[window['prefix']+fruit['name']]=fruit['eligible']
    detached_eligibility=dict(
        eligible=sum(eligibility.get(event['name']) is True for event in selected['breaks']),
        ineligible=sum(eligibility.get(event['name']) is False for event in selected['breaks']),
        unknown=sum(event['name'] not in eligibility for event in selected['breaks']),
        scope='Source authored RGB color proxy, not biological ripeness validation')
    limitations=[
        'SIM-GT ripe-group selection and existing IK; camera inference, optimized farm route and whole-farm harvesting are not implemented.',
        'Target-only plant collisions; neighboring plants remain full-resolution visuals without collision. Gutters, ground and robot collisions remain active.',
        'Fruit connect/angular spring, estimated inertia and per-source-mesh convex hulls are native adaptations; PhysX equivalence and biological damage limits are not established.',
        'Detached eligibility is derived from the exact source color policy, not fruit numbers. Color-selective harvesting, biological ripeness and collection are not generally validated; harvest_success remains null.',
        'Dropped-fruit impacts are retained physically but diagnosed separately. The opt-in gutter exception applies only after release; robot/attached-fruit/gutter contacts remain strict.',
        'CPU execution only. This run is not a GPU scaling or parallel-harvest validation.',
        'The floor top-plane variant preserves the original visible box and its top height but has infinite extent. It fixes thin-box tunneling, not small-penetration accuracy of falling impacts.',
    ]
    report=dict(schema='s300-mujoco-full-robot-migration-v1',primary=str(primary),
                primary_summary=selected,trials=trials,detached_by_window=per_window,
                videos=videos,detached_eligibility=detached_eligibility,limitations=limitations,training_eligible=False,
                harvest_success=None,end_to_end_collection_verified=False)
    (root/'harvest_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    def esc(value):return html.escape(str(value))
    rows=''.join('<tr>'+''.join('<td>'+esc(value)+'</td>' for value in (
        trial['trial'],f"{trial['simulated_s']:.2f}",f"{trial['loop_wall_s']:.2f}",
        f"{trial['realtime_factor']:.2f}",trial['detached'],f"{trial['active_penetration_mm']:.3f}",
        trial['active_metric_scope'],trial['active_contact_criterion_passed'],
        trial['stop'] or ('완주' if trial['active_contact_criterion_passed'] else '완주했으나 물리 기준 초과')))
        +'</tr>' for trial in trials)
    movie=''.join(f'<section><h3>{esc(Path(v["path"]).parent.name)} / {esc(Path(v["path"]).name)}</h3>'
        f'<video controls preload="metadata" src="{esc(Path(v["path"]).relative_to(root))}"></video>'
        f'<p>{esc(v["path"])}</p></section>' for v in videos)
    text=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>S300 MuJoCo 전체 로봇 이식</title>
<style>body{{font:16px sans-serif;margin:24px;background:#f6f7f9;color:#182230}}table{{border-collapse:collapse}}td,th{{border:1px solid #aaa;padding:7px}}video{{width:min(960px,100%)}}p{{overflow-wrap:anywhere}}.note{{background:#fff0ce;padding:16px}}</style>
<h1>S300 + 리프트 + RB5 + 벨트 그리퍼 + 전체 농장 — MuJoCo 3.15</h1>
<p>2026-10-12 / 실제 actuator physics / 저장 qpos 재생 / SIM-GT 진단</p>
<p class="note">대표 결과: {esc(primary.name)} — {selected['simulated_s']:.2f}초 계산에 {selected['loop_wall_s']:.2f}초,
실시간 {selected['realtime_factor']:.2f}배. {len(selected['breaks'])}개 분리.
중단: {esc(selected['stop'] or '없음')}. 활동 접촉 최대 {selected['active_contact_penetration_m']*1000:.3f}mm.
열매 분리는 수거·실물 수확 성공과 다릅니다. 아래 실패 시험도 보존했습니다.</p>
<p>source 색상 판정 대상 분리 {detached_eligibility['eligible']}개, 비대상 {detached_eligibility['ineligible']}개,
숙도 매핑 미확인 {detached_eligibility['unknown']}개. 생물학적 숙도 판정은 아닙니다.</p>
<p>대표 시험 접촉 시 {selected['hz']}Hz, 계산 주기별 step 수 {esc(selected['physics_steps_by_hz'])}.
각 진입마다 두 번 닫고 열며, 진입 전 완전 열림을 확인합니다.
속도는 물리·검사·상태 기록 포함, 초기 로딩·계획·저장·영상 제외입니다.</p>
<table><tr><th>시험</th><th>sim초</th><th>wall초</th><th>RTF</th><th>분리</th><th>활동 겹침mm</th><th>측정 범위</th><th>1mm 기준</th><th>완주/중단</th></tr>{rows}</table>
<p>이전 trial의 margin=.5mm 오류와 최종 열기 구간의 60Hz 분류 누락은 수정 후 새 초기 상태로 재시험했습니다.
이전 실패 결과를 덮어쓰지 않았습니다. 모든 영상은 실제 기록 상태의 1배속이며 물리를 다시 실행하지 않습니다.</p>
{movie}<ul>{''.join('<li>'+esc(x)+'</li>' for x in limitations)}</ul>
<p><a href="harvest_validation.json">전체 결과·해시</a> / <a href="{esc(primary.name)}/summary.json">대표 원자료</a></p></html>'''
    (root/'index.html').write_text(text)
    return report


def build_quality_report(root):
    """Present measured display presets without implying new physics trials."""
    root=Path(root).resolve()
    manifest_path=root/'quality_validation.json'
    report=json.loads(manifest_path.read_text())
    def esc(value):return html.escape(str(value),quote=True)
    def entries(value,label):
        if isinstance(value,dict):
            return [{**item,'name':item.get('name',name)} if isinstance(item,dict)
                    else dict(name=name,value=item) for name,item in value.items()]
        if isinstance(value,list):return value
        return []
    def asset_path(row):
        if isinstance(row,str):return Path(row)
        for key in ('path','output','image','video','file'):
            if row.get(key):return Path(row[key])
        return None
    def link(path):
        path=Path(path)
        absolute=path if path.is_absolute() else root/path
        try:target=str(absolute.relative_to(root))
        except ValueError:target=absolute.as_uri()
        return '<a href="'+esc(target)+'">'+esc(absolute)+'</a>'
    def target(path):
        path=Path(path)
        absolute=path if path.is_absolute() else root/path
        try:return str(absolute.relative_to(root))
        except ValueError:return absolute.as_uri()
    def detail(value):
        return '<pre>'+esc(json.dumps(value,ensure_ascii=False,indent=2))+'</pre>'
    profiles=entries(report.get('profiles',{}),'profile')
    snapshots=entries(report.get('snapshots',[]),'snapshot')
    videos=entries(report.get('videos',[]),'video')
    gui=entries(report.get('gui',[]),'gui')
    profile_rows=[]
    for row in profiles:
        name=row.get('name',row.get('quality','?'))
        values=(name,row.get('backend',row.get('renderer','')),
                row.get('video_size',row.get('size',
                    [row.get('width','?'),row.get('height','?')])),
                row.get('gui_size',''),row.get('supersample',''),
                row.get('measured_render_s',row.get('render_wall_s','')))
        profile_rows.append('<tr>'+''.join('<td>'+esc(v)+'</td>' for v in values)+'</tr>')
    image_sections=[]
    for row in snapshots:
        path=asset_path(row)
        if path is None:continue
        quality=(row.get('quality',row.get('name','')) if isinstance(row,dict) else '')
        stamp=row.get('time',row.get('t',row.get('start',''))) if isinstance(row,dict) else ''
        label=str(quality)+((' / t='+str(stamp)+'s') if stamp!='' else '')
        image_sections.append('<figure><figcaption>'+esc(label)+'</figcaption>'
            '<a href="'+esc(target(path))+'"><img loading="lazy" src="'+esc(target(path))+'"></a>'
            '<p>'+link(path)+'</p></figure>')
    video_sections=[]
    for row in videos:
        path=asset_path(row)
        if path is None:continue
        label=(row.get('quality',row.get('name',path.name)) if isinstance(row,dict) else path.name)
        video_sections.append('<section><h3>'+esc(label)+'</h3>'
            '<video controls preload="metadata" src="'+esc(target(path))+'"></video>'
            '<p>'+link(path)+'</p></section>')
    gui_rows=[]
    for row in gui:
        if not isinstance(row,dict):continue
        fps=row.get('presented_fps')
        if fps is None:fps='미계측 (native sync '+str(round(row.get('viewer_sync_fps',0),2))+'회/s)'
        values=(row.get('quality',row.get('name','')),row.get('presented_frames',''),
                row.get('display_wall_s',''),fps,
                row.get('render_wall_s',''))
        gui_rows.append('<tr>'+''.join('<td>'+esc(v)+'</td>' for v in values)+'</tr>')
    # The manifest is the complete evidence source. Unknown fields remain
    # available instead of being replaced with guessed timing or PASS values.
    text='''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Farmily MuJoCo 표시 품질 비교</title><style>
body{font:16px system-ui,sans-serif;margin:24px;background:#f6f7f9;color:#17222e}
table{border-collapse:collapse;background:white}td,th{padding:8px;border:1px solid #ccc}
.note{background:#fff1cf;padding:16px;line-height:1.55}.images{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px}
figure{margin:0;background:white;padding:10px}figcaption{font-weight:bold;margin-bottom:8px}
img{width:100%;height:auto}video{width:min(1280px,100%)}p,a,pre{overflow-wrap:anywhere}
pre{white-space:pre-wrap;background:white;padding:12px}code{font-size:14px}
@media(max-width:1000px){.images{grid-template-columns:1fr}}section{margin-top:24px}
</style><h1>MuJoCo — fast / balanced / ultra</h1>
<p class="note">동일하게 저장된 실제 qpos를 서로 다른 표시 설정으로 재생했습니다.
물리 시험을 다시 실행한 결과가 아닙니다. GUI는 저장 timestamp를 1배속으로 표시하며,
표시 FPS와 기존 물리 계산 속도는 다른 수치입니다. 최대 품질의 렌더링 비용이 더 큽니다.</p>
<p>fast: 기존 OpenGL 표시. balanced: Filament PBR, 원본 재질·줄기 표면 복원.
ultra: Filament PBR, 고품질 AO·PCSS 그림자·실제 공간 supersampling.
공식 RGB RenderTarget에서 MSAA는 비활성화되므로 MSAA 적용으로 보고하지 않습니다.
RTX 경로 추적 렌더러와 같은 결과를 보장하는 설정은 아닙니다.</p>
<h2>품질 옵션</h2><table><tr><th>옵션</th><th>렌더러</th><th>영상 출력</th>
<th>GUI 출력</th><th>가로·세로 supersampling 배율</th><th>측정 렌더 초</th></tr>'''
    if report.get('title'):
        text = text.replace('MuJoCo — fast / balanced / ultra', esc(report['title']))
    if report.get('summary'):
        text = text.replace('<h2>품질 옵션</h2>', '<p class="note">' + esc(report['summary']) + '</p><h2>품질 옵션</h2>')
    text+=''.join(profile_rows)+'''</table>
<p><code>./mujoco-benchmark/run_s300_mujoco.sh replay --quality ultra</code><br>
<code>./mujoco-benchmark/run_s300_mujoco.sh replay --quality balanced</code><br>
<code>./mujoco-benchmark/run_s300_mujoco.sh replay --quality fast</code></p>
<h2>동일 상태 이미지</h2><p>이미지를 클릭하면 원본 해상도로 확인합니다.</p><div class="images">'''
    text+=''.join(image_sections)+'</div><h2>실제 상태 영상</h2>'+''.join(video_sections)
    text+='''<h2>측정 GUI 표시 비용</h2><table><tr><th>옵션</th><th>표시 프레임</th>
<th>표시 wall초</th><th>표시 FPS</th><th>렌더 wall초</th></tr>'''+''.join(gui_rows)+'</table>'
    text+='<h2>원본·물리 보존 확인</h2>'+detail(report.get('physics_unchanged',{}))
    text+='<h2>원본 외형·재질 복원</h2>'+detail(report.get('visual_restore',report.get('assets',{})))
    text+='<h2>검증</h2>'+detail(report.get('tests',{}))
    text+='<p>'+link(manifest_path)+'</p></html>'
    (root/'index.html').write_text(text)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path)
    p.add_argument('--harvest-primary',type=Path)
    p.add_argument('--quality-report',action='store_true')
    a=p.parse_args()
    if a.quality_report and a.harvest_primary:p.error('Choose one report mode')
    if a.quality_report:build_quality_report(a.root)
    elif a.harvest_primary:build_harvest(a.root,a.harvest_primary)
    else:build(a.root)
