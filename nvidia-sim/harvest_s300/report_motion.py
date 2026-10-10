"""Summarize the existing native motion trial files, including failed attempts."""
import argparse
import html
import json
from collections import defaultdict
from pathlib import Path
import numpy as np


def plot_metrics(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,9),layout='constrained')
    for name,label in [('component_jaw_v1','Unloaded jaw control'),('component_pulse_v1','World +X 0.1 N pulse')]:
        folder=root/name
        d=np.load(folder/'states.npz');s=json.loads((folder/'summary.json').read_text())
        m=json.loads((Path(s['scene']).parent/'motion_manifest.json').read_text())
        tid=d['body_paths'].tolist().index(m['target']['path'])
        displacement=np.linalg.norm(d['poses'][:,tid,:3]-d['poses'][0,tid,:3],axis=1)*1000
        axes[0].plot(d['time'],displacement,label=label)
        if name=='component_jaw_v1':
            names=json.loads((folder/'native_metadata.json').read_text())['robot_dofs']
            l,r=names.index('jaw_L'),names.index('jaw_R')
            for key in ('command','robot_q'):
                axes[1].plot(d['time'],44+1000*(d[key][:,r]-d[key][:,l]),label=key)
    axes[0].axvspan(1,2,color='orange',alpha=.15);axes[0].set_ylabel('Fruit displacement (mm)');axes[0].legend()
    axes[1].set_ylabel('Jaw gap (mm)');axes[1].legend()
    folder=root/'component_motion_v3';d=np.load(folder/'states.npz')
    tid=d['body_paths'].tolist().index(m['target']['path'])
    displacement=np.linalg.norm(d['poses'][:,tid,:3]-d['poses'][0,tid,:3],axis=1)*1000
    axes[2].plot(d['time'],displacement,label='Fruit displacement after leaf contact')
    axes[2].axvline(d['time'][-1],color='red',ls='--',label='Stopped: penetration > 0.5 mm')
    axes[2].set_ylabel('Fruit displacement (mm)');axes[2].legend()
    for ax in axes:ax.set_xlabel('Recorded physics time (s)');ax.grid(alpha=.25)
    fig.suptitle('Component fixtures: measured native physics, NOT full-farm harvest')
    fig.savefig(root/'motion_metrics.png',dpi=150);plt.close(fig)


def summarize(root):
    rows=[]
    for folder in sorted(root.iterdir()):
        if not folder.is_dir() or folder.name.startswith('assembly'):continue
        if not any((folder/name).exists() for name in ('summary.json','runtime_error.json','cancelled.json','runtime_settings.json')):continue
        summary=folder/'summary.json'
        if not summary.exists():
            status='cancelled' if (folder/'cancelled.json').exists() else 'runtime_error' if (folder/'runtime_error.json').exists() else 'incomplete'
            rows.append(dict(trial=folder.name,status=status));continue
        s=json.loads(summary.read_text());s.pop('jaw_actual_gap_mm',None)
        s['trial']=folder.name
        if (folder/'cluster_evidence.json').exists():s['cluster_evidence']=json.loads((folder/'cluster_evidence.json').read_text())
        s['status']='completed_diagnostic' if s['completed'] else s['stop_reason']
        posthoc=folder/'posthoc_self_collision_audit.json'
        if posthoc.exists():
            s['posthoc_self_collision_audit']=json.loads(posthoc.read_text())
            if s['posthoc_self_collision_audit'].get('self_intersection_confirmed'):
                s['status']='invalid_self_collision (posthoc; recorded completion unchanged)'
        if folder.name in ('idle_v3','idle_v4'):
            s['status']='invalid_diagnostic_contact_readback_unavailable'
        if (folder/'states.npz').exists():
            with np.load(folder/'states.npz') as d:
                s['trace_finite']=bool(np.isfinite(d['poses']).all())
                dt=np.diff(d['time']);s['trace_max_dt_s']=float(dt.max()) if len(dt) else None
                s['trace_strictly_increasing']=bool(np.all(dt>0))
                s['phases_completed_or_entered']=list(dict.fromkeys(d['phase'].tolist()))
                names=json.loads((folder/'native_metadata.json').read_text())['robot_dofs']
                gap=.044+d['robot_q'][:,names.index('jaw_R')]-d['robot_q'][:,names.index('jaw_L')]
                s['jaw_measured_range_mm']=[float(gap.min()*1000),float(gap.max()*1000)]
        pairs=defaultdict(lambda:dict(count=0,min_separation_m=1.,max_point_force_n=0.))
        for e in json.loads((folder/'contacts.json').read_text()):
            a,b=sorted([e['a'],e['b']]);row=pairs[a+' <-> '+b]
            row['count']+=1;row['min_separation_m']=min(row['min_separation_m'],e['separation_m'])
            row['max_point_force_n']=max(row['max_point_force_n'],e['force_n'])
            if 'first_t' not in row:row['first_t']=e['t'];row['first_phase']=e['phase']
        s['worst_contact_pairs']=[dict(pair=pair,**row) for pair,row in sorted(pairs.items(),key=lambda x:x[1]['min_separation_m'])[:12]]
        s['video']='actual_motion.mp4' if (folder/'actual_motion.mp4').exists() else None
        if (folder/'actual_motion_review.mp4').exists():s['video']='actual_motion_review.mp4'
        if (folder/'actual_motion_final.mp4').exists():s['video']='actual_motion_final.mp4'
        if (folder/'actual_motion_verified.mp4').exists():s['video']='actual_motion_verified.mp4'
        if s['video']:
            render_meta=folder/(Path(s['video']).stem+'_render.json')
            s['video_speed']=json.loads(render_meta.read_text()).get('speed') if render_meta.exists() else None
        rows.append(s)
    report=dict(training_eligible=False,harvest_success=None,trials=rows,
        interpretation='Motion completion and entry-only demonstrations are separate from verified harvest success. All failed attempts remain recorded.')
    (root/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    esc=html.escape
    table=[];videos=[]
    for s in sorted(rows,key=lambda row:-(row.get('cluster_evidence',{}).get('valid_complete_run_detachment_count') or 0)):
        n=s['trial'];separation=s.get('min_contact_separation_m')
        scope='진입·유지·복귀 (수확 시도 아님)' if s.get('execution_scope')=='entry_hold_return_only' else '분리 시험' if s.get('fixture_only') else '전체 환경 물리 시험'
        cluster=s.get('cluster_evidence',{})
        if cluster.get('tread_prototype'):scope+=' · 접촉면 확장 하드웨어 시험안'
        cells=[n,s['status'],scope,cluster.get('native_detached_count','—'),
               f"{s.get('simulated_s',0):.3f}",f"{s.get('physics_wall_s',0):.2f}",
               f"{s.get('target_displacement_max_m',0)*1000:.3f}",
               '—' if separation is None else f'{max(0,-separation)*1000:.3f}']
        evidence=next((f for f in ('summary.json','runtime_error.json','cancelled.json','runtime_settings.json') if (root/n/f).exists()),None)
        link=f'<a href="{n}/{evidence}">JSON</a>' if evidence else '—'
        if s.get('posthoc_self_collision_audit'):
            link+=f' · <a href="{n}/posthoc_self_collision_audit.json">자기관통 후속 검사</a>'
        for filename,label in [('self_collision_preflight.json','계획 자기충돌 검사'),
                               ('self_collision_actual.json','실제 자세 자기충돌 검사'),
                               ('belt_evidence.json','벨트 접촉·회전·분리 근거'),
                               ('cluster_evidence.json','전체 열매 접촉·분리 집계')]:
            if (root/n/filename).exists():link+=f' · <a href="{n}/{filename}">{label}</a>'
        table.append('<tr>'+''.join('<td>'+esc(str(c))+'</td>' for c in cells)+f'<td>{link} · <a href="{n}.log">log</a></td></tr>')
        if s.get('video'):
            speed='배속 메타데이터 없음' if s.get('video_speed') is None else f'{s["video_speed"]:g}배속'
            videos.append(f'<article><h2>{esc(n)}</h2><p>{esc(s["status"])} / 실제 저장 상태 재생 / {esc(speed)} · 보조 조명 여부는 render JSON 참조</p><video controls preload="metadata" src="{n}/{s["video"]}"></video><p><a href="{n}/{s["video"]}">{n}/{s["video"]}</a></p></article>')
        if (root/n/'contact_cutaway.mp4').exists():
            videos.append(f'<article><h2>{esc(n)} · 접촉 확대</h2><p>동일한 실제 저장 상태. 관찰을 위해 재생 화면에서만 잎을 숨겼습니다. 물리 충돌체는 유지했습니다.</p><video controls preload="metadata" src="{n}/contact_cutaway.mp4"></video></article>')
        if cluster:
            details=''.join('<tr>'+''.join(f'<td>{esc(str(v))}</td>' for v in
                [f['name'],f['tool_contact_points'],f['native_detached'],round(f['max_displacement_mm'],3),round(f['max_point_force_n'],3),round(f['max_penetration_mm'],3)])+'</tr>'
                for f in cluster['fruits'] if f['tool_contact_points'] or f['native_detached'])
            videos.append(f'<article><h2>{esc(n)} · 접촉한 열매</h2><p>여러 열매의 접촉·밀림을 허용합니다. 파단 이벤트 수는 포집 완료나 실물 안전 검증과 다릅니다. 힘은 단일 접촉점의 최대값입니다.</p><table><tr><th>열매</th><th>접촉점 기록 수</th><th>분리 조인트 파단</th><th>이동 mm</th><th>힘 N</th><th>관통 mm</th></tr>{details}</table></article>')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>S300 · 실제 물리 구동 시험</title>
<style>body{font:16px system-ui;max-width:1400px;margin:32px auto;padding:0 24px;background:#151a20;color:#e8edf3}a{color:#8fd1ff}table{border-collapse:collapse;font-size:13px}td,th{border:1px solid #46515d;padding:8px;text-align:left}video{width:min(100%,960px)}article{margin:30px 0}.note{background:#313124;padding:18px;line-height:1.65}</style>
<h1>S300 + RB5 + V5.1 · 실제 물리 구동 시험</h1>
<p><a href="MOTION_STATUS_KO.md">MOTION_STATUS_KO.md</a> · <a href="results.json">results.json</a></p>
<div class="note">수확 성공은 별도 검증이 필요합니다. 진입·복귀 시연과 분리 시험의 완주를 실제 수확 성공으로 집계하지 않습니다.
각 실행의 범위와 물리 설정은 원자료 JSON과 설명 문서에 기록합니다.
영상은 실제 physics에서 저장한 body pose를 재생하며, 명령만 애니메이션한 영상이 아닙니다.</div>
<h2>전체 시도 기록</h2><p>처리 시간은 물리·제어·상태 읽기와 실행 중 화면 갱신(켜진 경우)을 포함합니다. 앱/장면 준비·결과 저장·사후 영상 렌더는 별도입니다.</p>
<table><tr><th>실행</th><th>상태</th><th>범위</th><th>열매 파단 수</th><th>sim 초</th><th>처리 초</th><th>목표 최대 변위 mm</th><th>최대 관통 mm</th><th>원자료</th></tr>'''+''.join(table)+'</table>'+''.join(videos)+'</html>'
    if (root/'motion_metrics.png').exists():page=page.replace('</html>','<h2>실제 측정값</h2><img style="width:min(100%,1000px)" src="motion_metrics.png"></html>')
    (root/'index.html').write_text(page)
    print(json.dumps(dict(output=str(root/'index.html'),trials=len(rows),videos=len(videos)),ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--plot',action='store_true');a=p.parse_args()
    if a.plot:plot_metrics(a.root)
    summarize(a.root)
