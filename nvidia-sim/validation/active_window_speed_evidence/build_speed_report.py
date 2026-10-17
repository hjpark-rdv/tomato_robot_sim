#!/usr/bin/env python3
"""Rebuild the Korean local HTML report from saved evidence; never runs physics.

Usage: python3 build_speed_report.py [--root /path/to/evidence]
Only index.html and speed_report_manifest.json are replaced. All media links are
relative URLs; visible labels retain the full saved source paths.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
from html import escape
import json
import math
import os
from pathlib import Path
from urllib.parse import quote


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    root = args.root.resolve()
    used = {}
    warnings = []

    def load(name, default=None):
        path = Path(name)
        if not path.is_absolute():
            path = root / path
        try:
            raw = path.read_bytes()
            value = json.loads(raw)
            used[str(path)] = hashlib.sha256(raw).hexdigest()
            return value
        except (FileNotFoundError, json.JSONDecodeError) as error:
            warnings.append(f'{path.name}: {type(error).__name__}')
            return default

    def native(path):
        # Saved reports contain both aliases for the same shared evidence root.
        value = str(path).replace('/root/docker_share/mujoko_debugging_data/',
                                  '/mnt/nas_rdv_md3/mujoko_debugging_data/', 1)
        return Path(value) if Path(value).is_absolute() else root / value

    def href(path):
        return quote(os.path.relpath(native(path), root), safe='/')

    def link(path, label=None, cls=''):
        return f'<a class="{escape(cls)}" href="{escape(href(path), quote=True)}">{escape(str(label if label is not None else path))}</a>'

    def number(value, digits=3):
        if value is None:
            return '—'
        try:
            return f'{float(value):,.{digits}f}' if math.isfinite(float(value)) else '—'
        except (ValueError, TypeError):
            return escape(str(value))

    audit = load('INDEPENDENT_THREE_STATION_AUDIT.json', {})
    metrics = load('experiment_metrics.json', [])
    gui = load('gui220_v3_actual_comparison.json', {})
    videos = load('qualified_video_list.json', [])
    video_checks = load('video_verification.json', [])
    preparation = load('prepare169_scope_ab_comparison.json', {})
    factors = load('factor169_compare.json', {})
    tests = load('final_tests_merged_inventory.json', {})
    updated_tests = load('final_tests_updates.json', {})
    hold_tests = load('tests_belt_hold_cpu.json', {})
    final_repeat = load('final_launcher_comparison.json', {})
    segments = audit.get('segments', [])
    chain = {str(native(row['run'])): row for row in segments}
    final_chain = {str(native(row['fresh_run'])): row for row in final_repeat.get('segments', [])}
    rows = {str(native(row['run'])): dict(row) for row in metrics}

    # Metrics may predate a running/final hold trial and intentionally omit setup
    # errors. Preserve those results too, without changing experiment_metrics.json.
    hold_summaries = list(root.glob('active*_belt_hold*/summary.json'))
    hold_summaries += list(root.glob('route/*belt_hold*/step*/physics/summary.json'))
    other_summaries = list(root.glob('*/summary.json'))
    other_summaries += list(root.glob('route/*/step*/physics/summary.json'))
    other_summaries += list(root.glob('active*_fresh_gui_comparison*/repeat*/summary.json'))
    other_summaries += list(root.glob('compact_hull_route*/*/step*/physics/summary.json'))
    supplementary = set(hold_summaries) | {p for p in other_summaries if str(p.parent) not in rows}
    for path in sorted(supplementary):
        summary = load(path, {})
        if not summary or summary.get('physics_engine') != 'Newton SolverMuJoCo':
            continue
        phase = summary.get('phase_performance', {})
        belt = [value for key, value in phase.items() if key in {
            'belt_start_open', 'close', 'cycle_open', 'cycle_open_hold',
            'cycle_reclose', 'belt_finish', 'cycle_open_final', 'cycle_open_final_hold'}]
        wall = sum(value['wall_s'] for value in belt)
        policy = summary.get('contact_policy') or {}
        valid = summary.get('status') == 'completed' and policy.get('policy_passed', False) and all(
            summary.get(key, False) for key in ('finite_recorded_state', 'qacc_finite', 'solver_capacity_valid'))
        rows[str(path.parent)] = dict(run=str(path.parent), status=summary.get('status', 'unknown'),
            stop=summary.get('stop'), error=summary.get('error'), eligible_complete_performance=valid,
            sim_s=summary.get('segment_simulated_s', summary.get('simulated_s')),
            loop_s=summary.get('loop_wall_s'), rtf=summary.get('realtime_factor'),
            setup_s=summary.get('setup_wall_s'), nv=summary.get('source_model_counts', {}).get('nv'),
            physics_hz=summary.get('physics_hz'), free_hz=summary.get('free_hz'),
            phase_performance=phase, minimum_phase_rtf=min((v['realtime_factor'] for v in phase.values()), default=None),
            belt_all_rtf=sum(v['simulated_s'] for v in belt) / wall if wall else None,
            new_release_names=[v['name'] for v in summary.get('breaks', [])],
            contact_policy_passed=policy.get('policy_passed'), solver_capacity_valid=summary.get('solver_capacity_valid'),
            belt_hold_frequency=summary.get('belt_hold_frequency'),
            parent_run=(summary.get('continuation') or {}).get('parent_run'),
            profile_instrumented=False)
    rows = list(rows.values())

    def short_name(run):
        path = native(run)
        try:
            return str(path.relative_to(root))
        except ValueError:
            return '기존379 / ' + '/'.join(path.parts[-3:])

    def scope(row):
        key = str(native(row['run']))
        if key in chain:
            return '검증된 연속 경로'
        if key in final_chain:
            return '최종 launcher 연속 재실행'
        name = short_name(row['run'])
        if '기존379' in name:
            return '기존379 연속 경로'
        if name.startswith('route/'):
            return '추가 연속 캠페인'
        if 'fresh_gui' in name:
            return '동일 초기상태 재실행'
        if name.startswith('active169_'):
            return '동일 Plant22 부모의 독립 시험'
        return '독립 설정 시험'

    def status(row):
        state = row.get('status')
        if row.get('unqualified_static_connect_control'):
            return '무효 대조군', 'bad'
        if state == 'completed':
            return ('완료 · 검사 통과', 'pass') if row.get('eligible_complete_performance') else ('완료 · 미검증', 'warn')
        if state == 'stopped':
            return '중단 · 완료속도 제외', 'bad'
        if state in ('initializing', 'running'):
            return '진행 중', 'warn'
        if state == 'error':
            return '오류 · 실행 미완료', 'bad'
        return str(state or '결과 없음'), 'bad'

    perf = audit.get('performance', {})
    totals = audit.get('final_release_totals', {})
    generated = datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')
    chunks = ['''<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Newton GPU 수확 — 실제 실행 및 속도 검증</title><style>
:root{--ink:#172b35;--muted:#536874;--border:#d8e1e5;--teal:#086d64;--bg:#f3f6f7;--bad:#9d3333;--warn:#855814}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.65 system-ui,-apple-system,"Noto Sans KR",sans-serif}main{max-width:1440px;margin:auto;padding:38px 26px 70px}h1{font-size:32px;letter-spacing:-1px;margin:5px 0 12px}h2{font-size:22px;margin:0 0 15px}h3{font-size:17px;margin:0 0 12px}p{margin:10px 0}a{color:#075fa3;text-decoration:none}a:hover{text-decoration:underline}small,.muted{color:var(--muted)}.eyebrow{font-size:12px;font-weight:700;letter-spacing:1.6px;color:var(--teal)}nav{display:flex;flex-wrap:wrap;gap:18px;margin:24px 0}.panel{background:white;border:1px solid var(--border);border-radius:13px;padding:24px;margin-top:22px}.notice{border-left:4px solid #b68131;padding:12px 17px;background:#fff9ee}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:13px;margin:22px 0}.card{background:white;border:1px solid var(--border);border-radius:10px;padding:16px}.big{display:block;font-size:28px;font-weight:750;line-height:1.25}.card small{display:block;margin-top:5px}.scroll{overflow:auto}table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}th,td{text-align:right;padding:10px 11px;border-bottom:1px solid #e6ecef;vertical-align:top}th{color:#415866;background:#edf3f4;font-size:12px;white-space:nowrap}th:first-child,td:first-child{text-align:left}td.scope{text-align:left;font-size:12px;min-width:150px}.run{max-width:325px;overflow-wrap:anywhere;font-size:12px}tbody tr:hover{background:#f8fbfc}.badge{border-radius:5px;display:inline-block;padding:2px 6px;font-size:11px;white-space:nowrap}.pass{background:#e7f4ef;color:#176249}.bad{background:#fff0ef;color:var(--bad)}.warn{background:#fff5e1;color:var(--warn)}.path{font-size:11px;overflow-wrap:anywhere;display:block;line-height:1.5;margin:8px 0}.videos{display:grid;grid-template-columns:repeat(2,1fr);gap:22px}video{display:block;width:100%;max-height:420px;background:#12232b;border-radius:8px}.video-card{min-width:0}.inline-code,code{background:#eef2f5;padding:2px 5px;border-radius:4px;font-size:12px}input,select{font:inherit;padding:8px 10px;border:1px solid var(--border);border-radius:6px}.filters{display:flex;gap:10px;margin:14px 0}.filters input{flex:1}.evidence{display:flex;gap:12px;flex-wrap:wrap;font-size:13px;margin-top:15px}details{margin-top:12px}summary{cursor:pointer;color:#075fa3}footer{font-size:12px;color:var(--muted);margin-top:25px}#all-results td{font-size:12px}#all-results td:nth-child(n+4){white-space:nowrap}@media(max-width:780px){main{padding:22px 12px}.cards{grid-template-columns:repeat(2,1fr)}.videos{grid-template-columns:1fr}.panel{padding:17px}h1{font-size:27px}.filters{flex-direction:column}}@media print{.filters,video{display:none}.panel{break-inside:avoid}body{background:white}main{max-width:none}}
</style></head><body><main><header><div class="eyebrow">S300 · NEWTON SOLVERMUJOCO · RTX 3090 Ti</div><h1>한쪽 베드 수확: 실제 실행과 남은 병목</h1>
<p>Plant22 → Plant27 → Plant29의 측정 상태를 이어 실행했다. 아래 결과는 완료 경로, 독립 설정 시험, GUI 검증을 구분한다.</p>''']
    chunks.append(f'<small>보고서 생성 {escape(generated)} · '+link('experiment_metrics.json','실험 원자료')+' · '+link('build_speed_report.py','재생성 스크립트')+'</small></header>')
    chunks.append('<nav><a href="#chain">원래 연속 3방문</a><a href="#final-repeat">최종 launcher 재실행</a><a href="#experiments">전체 실험</a><a href="#hold">hold 주파수 시험</a><a href="#setup">초기화 비용</a><a href="#gui">Isaac GUI</a><a href="#videos">실제 영상 4개</a><a href="#audit">검증 근거</a></nav>')
    minimum = min((s.get('minimum_phase', {}).get('realtime_factor', float('inf')) for s in segments), default=None)
    cards = [(number(perf.get('overall_loop_rtf'))+' RTF','검증된 연속 경로의 physics 루프'),
             (number(minimum)+' RTF','가장 느린 세부 구간 · belt_finish'),
             (str(totals.get('releases','—'))+'개 분리',f"색상상 익음 {totals.get('eligible','—')} · 미숙 {totals.get('ineligible','—')} · 포집 미검증"),
             (number(perf.get('setup_wall_s'),1)+'초','별도 setup 합계 · loop 속도에서 제외')]
    chunks.append('<div class="cards">'+''.join(f'<div class="card"><span class="big">{escape(a)}</span><small>{escape(b)}</small></div>' for a,b in cards)+'</div>')
    chunks.append('<div class="notice"><strong>물리 표현 변경이 포함된 가속이다.</strong> 기존379 DOF의 세 식물 동시 탄성 모델에서 현재 타깃만 탄성으로 계산하고, 다른 식물은 마지막 측정 자세의 고정 충돌체로 남긴다. 분리된 과실은 계속 동적이다. 같은 모델의 solver만 빨라졌다는 비교는 아니다. 현재 검증된 3방문은 모든 접촉 세부 구간이 실시간 이상이라는 조건을 아직 충족하지 못했다.</div>')
    chunks.append('<section class="panel" id="chain"><h2>검증된 220 Hz 연속 3방문</h2><div class="scroll"><table><thead><tr><th>대상 / 실제 결과</th><th>DOF</th><th>기존379 RTF</th><th>현재 전체 RTF</th><th>벨트 전체 RTF</th><th>최저 phase RTF</th><th>새 분리</th></tr></thead><tbody>')
    old = [r for r in rows if 'sequence_native' in r['run']]
    for s in segments:
        plant = s['plant_id']
        oldrow = next((r for r in old if f'plant{plant}' in r['run']), {})
        chunks.append('<tr><td>'+link(Path(s['run'])/'summary.json',f'Plant{plant}')+'</td>'+''.join('<td>'+v+'</td>' for v in [str(s.get('counts',{}).get('nv','—')),number(oldrow.get('rtf')),number(s.get('rtf')),number(s.get('belt_all',{}).get('realtime_factor')),number(s.get('minimum_phase',{}).get('realtime_factor')),str(s.get('new_release_count','—'))])+'</tr>')
    chunks.append('</tbody></table></div>')
    chunks.append(f'<p>물리 {number(perf.get("segment_simulated_s"),3)}초 / 루프 {number(perf.get("loop_wall_s"),3)}초. 기존379 경로 대비 루프 시간비 {number(perf.get("loop_wall_speed_ratio_old_to_new"),3)}배. 기존 분리18개 중 P27 Fruit02가 이번에는 남아 총17개이며, 잔여13개다.</p>')
    chunks.append('<p class="muted">body357·geom2244·부착30개와 주변6그루의1150충돌체를 유지한다. 자유도는 분리 과실을 계속 적분하면서133 →169 →199로 증가한다. 두 전환에서 실제 clock 차이는0이며, 저장 상태·주소 매핑·누적 분리 이력을 검증했다. 구간마다 solver와 접촉 cache는 재생성하므로 단일 solver의 무중단 실행은 아니다. 편도 방문만 검증했으며 고정 식물 재방문 시 저장 속도 취급은 후속 과제다.</p>')
    chunks.append('<p>기본1mm 감시를 유지한다. 정확한 현재 타깃 belt↔active fruit/pedicel만 <code>close / cycle_reclose / belt_finish / cycle_open / cycle_open_final</code>에서 기존2mm 예외를 적용한다. 벨트 유지 <code>belt_finish</code>는 포함하고, 완전히 열린 hold phase는 제외한다. 분리3N/0.008Nm는 그대로다. 분리 과실↔정확한 gutter는 충돌을 계속 계산하고 기존 중단 예외만 명시했다. <code>training_eligible=false</code>, <code>harvest_success=null</code>.</p>'+link('INDEPENDENT_THREE_STATION_AUDIT.json','독립 저장상태 감사 원문')+'</section>')

    chunks.append('<section class="panel" id="final-repeat"><h2>최종 row-realtime launcher · 연속 3방문 재실행</h2>')
    if final_repeat:
        measured = final_repeat.get('measured_time_subtotals', {})
        checks = [('3방문 완료·물리 검사', 'all_three_completed_and_qualified'),
                  ('저장 raw 배열 정확 일치', 'all_three_raw_archives_exact'),
                  ('실행 명령 정확 일치', 'all_three_commands_exact'),
                  ('각 구간 물리 모델 SHA 일치', 'all_three_physical_model_hashes_same')]
        chunks.append('<p>'+ ' · '.join(escape(label)+': <strong>'+str(final_repeat.get(key))+'</strong>' for label,key in checks)+'. 원래 기준 경로의 측정값을 덮어쓰지 않은 별도 새 실행이다.</p>')
        chunks.append('<div class="scroll"><table><thead><tr><th>대상 / 새 실제 결과</th><th>물리 초</th><th>새 loop 초</th><th>새 loop RTF</th><th>기준 setup 초</th><th>새 setup 초</th><th>구간 orchestration 초</th><th>새 분리</th></tr></thead><tbody>')
        for segment in final_repeat.get('segments', []):
            item = segment.get('measured_performance', {})
            orchestration = item.get('orchestration', {})
            values = [number(item.get('simulated_segment_s'),3), number(item.get('fresh_loop_wall_s'),3),
                      number(item.get('fresh_loop_rtf'),3), number(item.get('source_setup_s'),3),
                      number(item.get('fresh_setup_s'),3), number(orchestration.get('total_segment_orchestration_s'),3),
                      str(item.get('new_detachments','—'))]
            chunks.append('<tr><td>'+link(Path(segment['fresh_run'])/'summary.json',f"Plant{segment['plant_id']}")+'</td>'+''.join('<td>'+value+'</td>' for value in values)+'</tr>')
        chunks.append('</tbody></table></div>')
        chunks.append('<p>물리 '+number(measured.get('simulated_s'),6)+'초를 loop '+number(measured.get('loop_wall_s'),6)+'초에 계산해 <strong>'+number(measured.get('loop_rtf'),6)+' RTF</strong>. 실제 setup은 기준 '+number(perf.get('setup_wall_s'),3)+'→'+number(measured.get('runtime_setup_wall_s'),3)+'초였다. 새 setup+loop는 '+number(measured.get('setup_plus_loop_s'),3)+'초, <strong>'+number(measured.get('setup_plus_loop_rtf'),6)+' RTF</strong>다.</p>')
        chunks.append('<p><strong>측정된 구간 orchestration 합은 '+number(measured.get('sum_segment_orchestration_s'),3)+'초(약9분45초), '+number(measured.get('orchestration_rtf'),6)+' RTF</strong>다. 계획·명령 준비 '+number(measured.get('plan_and_command_preparation_s'),3)+'초, 모델 축소·전환 검증 '+number(measured.get('model_reduction_and_transition_validation_s'),3)+'초, physics 프로세스 전체 '+number(measured.get('physics_process_with_lock_setup_save_s'),3)+'초와 소규모 상위 오버헤드를 포함한다. physics 프로세스 시간에는 setup·loop·lock 대기·저장 등이 이미 들어 있으므로 setup+loop와 중복 합산하지 않는다.</p>')
        chunks.append('<p class="notice">최초 prepared-input 검증과 shell/Python 시작 시간은 측정하지 않아 위9분45초 밖에 추가로 존재한다. 따라서 전체 명령 실행의 최종 stopwatch 시간이 아니다. 기준 실행에는 같은 orchestration 계측이 없어 전체 캠페인 속도비를 추정하지 않는다. 한 번의 새 프로세스 캠페인이며 cold filesystem 평균이나 반복 분포를 입증하지 않는다.</p>')
        chunks.append('<p>기존 영상의 상태 재사용 검증: <strong>'+str(final_repeat.get('old_video_state_reuse_qualified'))+'</strong>. 모든 저장 상태·명령 대조에 따라 아래 기존 영상4개를 동작 증거로 재사용한다. 새 영상은 생성하지 않았으며 파일 자체는 원래 기준 경로에서 렌더링한 것이다.</p>')
        chunks.append(link('final_launcher_comparison.json','최종 재실행 독립 raw·시간 대조')+' · '+link('final_launcher_comparison_KO.md','한국어 감사 보고서'))
    else:
        chunks.append('<p>아직 최종 launcher 독립 대조 파일이 없다.</p>')
    chunks.append('</section>')

    chunks.append('<section class="panel" id="experiments"><h2>전체 실행 결과 · 완료와 중단 모두 표시</h2><p class="muted">RTF = 실제 물리 시간 / physics 루프 wall time. 중단 실행의 RTF·최저 phase는 도달한 부분만의 수치이며 완료 성능 비교에서 제외한다. 프로파일 계측과 GUI 동시 실행도 별도 표시한다. 각 행의 이름은 실제 summary 링크다.</p><div class="filters"><input id="query" type="search" aria-label="실험 검색" placeholder="실험명 / 범위 검색"><select id="state" aria-label="상태 필터"><option value="">모든 상태</option><option value="pass">완료 검사 통과</option><option value="bad">중단·무효</option><option value="warn">진행·미검증</option></select></div><div class="scroll"><table id="all-results"><thead><tr><th>실험</th><th>범위 / 상태</th><th>Hz / DOF</th><th>sim 초</th><th>loop 초</th><th>RTF</th><th>벨트 RTF</th><th>최저 phase</th><th>setup 초</th><th>새 분리</th></tr></thead><tbody>')
    for row in rows:
        label, cls = status(row)
        note = []
        if row.get('profile_instrumented'): note.append('프로파일 계측')
        if row.get('stop'): note.append(str(row['stop']))
        if row.get('error'): note.append(str(row['error']))
        if 'native239' in row['run']: note.append('R/part239 물리 hull 표현 별도')
        if row.get('belt_hold_frequency'): note.append('hold만 '+number(row['belt_hold_frequency'].get('requested_hz'),0)+' Hz')
        if row.get('unqualified_static_connect_control'): note.append('static CONNECT 수정 이전 · 채택 제외')
        if 'fresh_gui' in row['run']: note.append('GUI' if 'repeat_001' in row['run'] else 'headless')
        partial = row.get('status') == 'stopped'
        rtf = number(row.get('rtf')) + ('*' if partial else '')
        chunks.append(f'<tr data-state="{cls}"><td class="run">'+link(Path(row['run'])/'summary.json',short_name(row['run']))+('<br><small>'+escape(' · '.join(note))+'</small>' if note else '')+'</td><td class="scope">'+escape(scope(row))+f'<br><span class="badge {cls}">{escape(label)}</span></td><td>'+number(row.get('physics_hz'),0)+' / '+number(row.get('nv'),0)+'</td>'+''.join('<td>'+v+'</td>' for v in [number(row.get('sim_s'),2),number(row.get('loop_s'),2),rtf,number(row.get('belt_all_rtf')),number(row.get('minimum_phase_rtf')),number(row.get('setup_s'),2),str(len(row.get('new_release_names',[])))])+'</tr>')
    chunks.append('</tbody></table></div><p><small>* 부분 실행. 완료했다고 모두 같은 물리 결과이거나 전체3방문이 검증된 것은 아니다. 180 Hz는 첫 타깃 완료 후 두 번째 타깃에서 실제 접촉으로 중단했다. 160/200 Hz 전역 시험도 중단해 기본220 Hz를 유지한다.</small></p></section>')

    hold = [r for r in rows if r.get('belt_hold_frequency') or 'belt_hold' in r['run']]
    chunks.append('<section class="panel" id="hold"><h2>stationary belt_finish만 낮은 Hz · opt-in 정확도 시험</h2><p>접근·닫기·열기는220 Hz를 유지하고, 고정 arm·rail·jaw의 기존 belt_finish만 같은 끝점 사이를 등분한다. 외부 command/time/dt는 byte-exact 보존하며 실제 dt별 CUDA graph를 사용한다. timestep 변경에 따른 물리 결과는 별도 검증 대상이다.</p>')
    if not hold:
        chunks.append('<p>아직 기록된 GPU 결과가 없다.</p>')
    for row in hold:
        label,_ = status(row)
        h = row.get('belt_hold_frequency') or {}
        parent = row.get('parent_run')
        timing_scope = '도달한 부분만' if row.get('status') != 'completed' else '완료 구간 전체'
        chunks.append('<p><strong>'+link(Path(row['run'])/'summary.json',short_name(row['run']))+'</strong> — '+escape(label)+' · '+timing_scope+' '+number(row.get('rtf'))+' RTF · 도달한 belt_finish '+number(row.get('phase_performance',{}).get('belt_finish',{}).get('realtime_factor'))+' RTF · 새 분리 '+str(len(row.get('new_release_names',[])))+'개.</p>')
        if row.get('stop'):
            chunks.append('<p><span class="badge bad">'+escape(row['stop'])+'</span> 완료 성능·성공 결과로 채택하지 않는다.</p>')
        rate = h.get('requested_hz')
        failure_name = f'belt_hold{int(rate)}_failure_audit.json' if rate is not None else None
        if failure_name and (root / failure_name).is_file():
            failure = load(failure_name, {})
            pair = failure.get('geom_pair', [])
            pair_label = ' ↔ '.join(item.get('source_collider', item.get('name', '')) for item in pair)
            maximum = (failure.get('stop', {}).get('maximum_penetration_m_by_class') or {}).get('other')
            prefix = failure.get('prefix_through_hold_start_bytes_exact', {})
            chunks.append('<p><small>첫 위반 쌍: '+escape(pair_label)+'. other 분류 최대 '+number(maximum * 1000 if maximum is not None else None,6)+' mm / 기준1mm. 이 최대값은 개별 접촉 쌍의 저장 wrench가 아니라 분류별 기록이다. hold 진입 전 상태 byte-exact: '+str(bool(prefix) and all(prefix.values()))+'. '+link(failure_name,'실패 감사')+'</small></p>')
        if parent:
            chunks.append('<p class="muted">실제 부모: '+link(Path(parent)/'summary.json',parent)+'. 동일한 완료 부모에서 시작한 독립 분기 시험이며, 이 결과를 기존 Plant27→Plant29 연속 경로의 일부로 합산하지 않는다.</p>')
        else:
            chunks.append('<p class="muted">범위: '+escape(scope(row))+'. 별도 전체 경로 검증 없이 기존3방문의 결과를 대체하지 않는다.</p>')
        if h.get('intervals'):
            intervals='; '.join(f"{number(v['start_s'],6)}–{number(v['end_s'],6)} s: {v['original_steps']}→{v['steps']} steps, 실제 {number(v['effective_hz'],3)} Hz" for v in h['intervals'])
            chunks.append('<p><small>'+escape(intervals)+'</small></p>')
    chunks.append('<p>현재 기본값으로 채택하지 않는다. belt_finish에서도 정확한 벨트↔목표 열매·꼭지가지에만 기존2mm 예외가 적용되며, 고정 판이나 비벨트 그리퍼 부품에는1mm를 유지한다. '+link('belt_hold_active169_sampling_cpu.json','실제 명령 파일의 CPU 대조')+' · '+link('tests_belt_hold_cpu.json','옵션·샘플링 CPU41검사')+' · '+link('belt_hold_frequency_failure_comparison.json','160/180 Hz 실패·정책 종합 감사')+'</p></section>')

    chunks.append('<section class="panel" id="setup"><h2>초기화 비용은 실행 속도와 별개</h2>')
    chunks.append(f'<p>검증된3방문의 setup 합계는 <strong>{number(perf.get("setup_wall_s"),3)}초</strong>, setup+loop는 <strong>{number(perf.get("setup_plus_loop_s"),3)}초</strong>이며 해당 비율은 {number(perf.get("setup_plus_loop_rtf"),3)} RTF다. 준비·로딩까지2.414배속이라는 뜻이 아니다. 계획·입력 준비·렌더링·프로세스 대기는 이 합계에도 포함하지 않는다.</p>')
    if preparation:
        chunks.append('<p>같은169 DOF 상태 은행의 CPU 준비 A/B: '+number(preparation.get('before',{}).get('wall_s'),3)+'→'+number(preparation.get('after',{}).get('wall_s'),3)+'초 ('+number(100*preparation.get('wall_reduction_fraction',0),1)+'% 감소). 생성 물리 파일과 초기/은행 배열이 정확히 같았다. 파일 SHA·읽기전용 모델의 호출 내 중복을 줄였으며, 원본 검증을 생략하지 않았다. warm filesystem 단회 측정이고 과거3방문의 setup 값을 소급 교체하지 않는다. '+link('prepare169_scope_ab_comparison.json','A/B 원자료')+'</p>')
    chunks.append('<p class="muted">검증된 local MJB cache, 독립 파일8개 병렬 해시, 불변 mesh 디렉터리 공유를 사용한다. 설치 Newton GPU solver를 그대로 쓰며 solver/contact cache를 구간 간 지속 보존하는 새 pipeline은 만들지 않았다.</p></section>')

    chunks.append('<section class="panel" id="gui"><h2>Isaac 전체 농장 GUI · 새 physics 재실행 대조</h2>')
    if gui:
        a,b=gui.get('performance_a',{}),gui.get('performance_b',{})
        display=gui.get('display_verification',{})
        profiles=gui.get('display_profile_and_before_after',{})
        before,after=profiles.get('old_gui220',{}),profiles.get('new_gui220_v3',{})
        chunks.append('<p>동일 초기상태로 초기화한 두 새 실행에서 headless '+number(a.get('whole_rtf'))+' RTF / GUI '+number(b.get('whole_rtf'))+' RTF. 기록 배열 byte-exact: <strong>'+str(gui.get('all_recorded_array_bytes_equal'))+'</strong>, 분리 JSON exact: <strong>'+str(gui.get('all_release_json_equal'))+'</strong>. 이 첫 타깃 A/B는 연속3방문 완료와 별도 검증이다. 한 GPU session의 초기 buffer를 복원한 두 회차이며, 독립 cold-start 프로세스 대조는 아니다.</p>')
        if a.get('physics_loop_wall_s') and b.get('physics_loop_wall_s'):
            overhead = 100 * (b['physics_loop_wall_s'] / a['physics_loop_wall_s'] - 1)
            chunks.append('<p>현재 headless→GUI: '+number(a['physics_loop_wall_s'],3)+'→'+number(b['physics_loop_wall_s'],3)+'초, physics 루프 wall time <strong>+'+number(overhead,2)+'%</strong>.</p>')
        previous_overhead = 100 * (profiles.get('gui_physics_loop_wall_ratio', 1) - 1)
        chunks.append('<p>이전GUI→현재GUI: 정상 구간 render update는 '+number(before.get('steady_render_updates_per_s_excluding_first_10'),2)+'→'+number(after.get('steady_render_updates_per_s_excluding_first_10'),2)+' fps, pose/skin CPU 갱신은 '+number(1000*before.get('cpu_pose_skin_update_mean_s',0),2)+'→'+number(1000*after.get('cpu_pose_skin_update_mean_s',0),2)+' ms. 정확한 pose/skin 의존 cache와 Kit 이중 제한 제거를 함께 적용했다. 이 이전GUI→현재GUI 비교의 physics wall time은 '+number(profiles.get('gui_physics_loop_wall_ratio'),3)+'배(+'+number(previous_overhead,2)+'%)여서 표시 개선을 physics 가속으로 해석하지 않는다.</p>')
        pubs=display.get('display_publications',{})
        chunks.append(f'<p class="muted">표시한 측정 publication {pubs.get("displayed_unique_measured_publications","—")} / 발행 {pubs.get("published_measured_frames","—")}. 최신 프레임 표시에서 생략된 publication은 physics step 누락이 아니다. viewer의 physics step은{display.get("display_physics_steps","—")}이며, camera 이동·마지막 pose hash/time ACK를 검증했다. 전체 농장은 시각 USD 범위이며 물리 모델의 모든 식물을 동시 탄성 계산한다는 뜻은 아니다.</p>')
        if display.get('final_capture'):
            chunks.append(link(display['final_capture'],'실제 GUI 최종 화면'))
        chunks.append(' · '+link('gui220_v3_actual_comparison.json','GUI/headless 정확도·표시 검증')+' · '+link('gui220_display_optimization_ab.json','GUI 이전/이후 물리 배열 대조'))
    chunks.append('</section>')

    chunks.append('<section class="panel" id="videos"><h2>실제 저장 상태 영상 · 1× · 4개</h2><p>완료한 연속 경로의 기록된 몸체 pose와 실제 분리 시각으로 렌더링했다. 영상 생성은 physics를 다시 실행하지 않는다. 전체 농장 영상 일부는 arm·잎에 가려질 수 있고, 접촉 단면 영상의 잎 숨김은 표시만 바꾼다. 경로 문자열 자체가 실제 상대 URL 링크다.</p><div class="videos">')
    media_checks=[]
    for video in videos:
        path=video['path']; exists=native(path).is_file()
        contact='contact_cutaway' in video.get('role','')
        title=f"Plant{video.get('plant_id')} · "+('접촉 단면 · 표시용 잎 숨김' if contact else '전체 농장 · 실제 전체 동작')
        check=next((v for v in video_checks if str(native(v['path']))==str(native(path))),{})
        probe=check.get('probe',{}); streams=probe.get('streams',[{}]); stream=streams[0] if streams else {}
        chunks.append('<article class="video-card"><h3>'+escape(title)+'</h3>')
        if exists:
            chunks.append(f'<video controls playsinline preload="none" src="{escape(href(path),quote=True)}" aria-label="{escape(title,quote=True)}"></video>')
        else:
            chunks.append('<p class="notice">영상 파일 아직 없음 / 저장 중</p>')
        chunks.append(link(path,path,'path')+'<small>'+escape(f"1× · {stream.get('width','—')}×{stream.get('height','—')} · {stream.get('r_frame_rate','—')} fps · {probe.get('format',{}).get('duration','—')} s")+'</small></article>')
        media_checks.append(dict(source_path=path,relative_href=href(path),exists=exists))
    chunks.append('</div><p>'+link('qualified_video_list.json','영상 목록')+' · '+link('video_verification.json','영상 ffprobe·실제 프레임 확인')+'</p></section>')

    chunks.append('<section class="panel" id="audit"><h2>검증 근거와 채택하지 않은 대안</h2><p>solve-local Jacobian cache는 복사77%를 줄였지만 실제 시간은 줄지 않았다. 동일 Hessian factor cache는18.82% 재사용에도 더 느렸다. CCD32의 단일 clean 시험 차이는 약0.6%여서 기본64를 유지한다. 모두 기록된 배열을 기준과 정확히 대조했으며 단일회 속도 차이를 보편적 가속으로 확대하지 않는다.</p>')
    if factors:
        chunks.append('<details><summary>169차원 factor 마이크로벤치 — 합성 SPD, 실제 수확 RTF 아님</summary><div class="scroll"><table><thead><tr><th>구현</th><th>동기화 완료 ms/factor</th><th>수치 검사</th></tr></thead><tbody>')
        for row in factors.get('results',[]):
            if row.get('done'):continue
            name=row.get('implementation','')
            if name in ('warp32','warp64'):name+=' (FP32, suffix는 block 크기)'
            chunks.append('<tr><td>'+escape(name)+'</td><td>'+number(row.get('median_gpu_ms_per_factor'),6)+'</td><td>'+escape(str(row.get('numerically_valid')))+'</td></tr>')
        chunks.append('</tbody></table></div><p>같은 합성 입력과 CUDA graph completion을 비교했다. 실제 접촉 Hessian의 조건수/수렴 결과를 대신하지 않는다. '+link('factor169_compare.json','전체 residual·health 원자료')+'</p></details>')
    counts=tests.get('counts',{})
    updates = updated_tests.get('counts', {})
    chunks.append(f'<p>최종 CPU 중복제거 목록: 실제 test ID <strong>{tests.get("test_count","—")}개, 통과{counts.get("passed","—")} / skip{counts.get("skipped","—")}</strong>. 마지막 변경분 회귀{updates.get("passed","—")}개 통과를 기존 목록과 ID로 병합한 수치다. 별도 hold 회귀{hold_tests.get("run_count","—")}개도 중복 합산하지 않는다. GUI display15개는 이420개에 포함하지 않은 별도 목록이다.</p>')
    evidence=['INDEPENDENT_THREE_STATION_AUDIT.json','final_launcher_comparison.json','experiment_metrics.json','active169_cache_exact_comparison.json','active169_ccd32_exact_comparison.json','factor169_compare.json','final_tests_merged_inventory.json','final_tests_updates.json','tests_belt_hold_cpu.json','gui_display_cache_v3_cpu_tests.json','prepare169_scope_ab_comparison.json','belt_hold_frequency_failure_comparison.json']
    chunks.append('<div class="evidence">'+''.join(link(name,name) for name in evidence)+'</div>')
    chunks.append('<p class="muted">실제 미숙 과실 분리1개, 잔여13개, 포집 미검증을 남긴다. 비타깃 탄성 응답 고정, native convex proxy, 과실 비틀림 분리 모델의 한계가 있다. 충돌체를 보존했다는 사실만으로 모든 주변 접촉의 안전성이 검증되지는 않는다. 학습·전체 수확 성공으로 판정하지 않는다.</p></section>')
    if warnings:
        chunks.append('<details><summary>읽기 시점의 누락/갱신 중 파일</summary><pre>'+escape('\n'.join(warnings))+'</pre></details>')
    chunks.append('<footer>재생성: <code>python3 build_speed_report.py</code>. 입력 JSON과 실제 hold summary를 읽으며 physics/GPU 실행과 원본 결과 변경을 하지 않는다. 상대 링크는 보고서 폴더를 같은 구조로 복사하거나 HTTP에서 열어도 사용할 수 있다.</footer></main><script>const query=document.getElementById("query"),state=document.getElementById("state");function filter(){const q=query.value.toLowerCase();document.querySelectorAll("#all-results tbody tr").forEach(r=>{r.hidden=!(r.textContent.toLowerCase().includes(q)&&(!state.value||r.dataset.state===state.value));});}query.addEventListener("input",filter);state.addEventListener("change",filter);</script></body></html>')
    html='\n'.join(chunks)
    (root/'index.html').write_text(html,encoding='utf-8')
    manifest=dict(generated_at=generated,report='index.html',row_count=len(rows),
        status_counts=dict(Counter(row.get('status') for row in rows)),
        verified_chain_runs=[str(native(s['run'])) for s in segments],
        final_repeat_runs=list(final_chain),
        hold_runs=[str(native(r['run'])) for r in hold],source_sha256=used,
        videos=media_checks,warnings=warnings,physics_steps=0,gpu_execution=False,
        html_sha256=hashlib.sha256(html.encode()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (root/'speed_report_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:manifest[k] for k in ['report','row_count','status_counts','hold_runs','warnings']},ensure_ascii=False))


if __name__=='__main__':
    main()
