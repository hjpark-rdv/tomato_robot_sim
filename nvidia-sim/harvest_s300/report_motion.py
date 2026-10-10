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
        s['status']='completed_diagnostic' if s['completed'] else s['stop_reason']
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
        rows.append(s)
    report=dict(training_eligible=False,harvest_success=None,trials=rows,
        interpretation='Completed component fixtures are not a valid dense-farm harvest. Original initial collisions and all failed attempts remain recorded.')
    (root/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    esc=html.escape
    table=[];videos=[]
    for s in rows:
        n=s['trial'];separation=s.get('min_contact_separation_m')
        cells=[n,s['status'],'분리 시험' if s.get('fixture_only') else '원래 농장/초기 진단',
               f"{s.get('simulated_s',0):.3f}",f"{s.get('physics_wall_s',0):.2f}",
               f"{s.get('target_displacement_max_m',0)*1000:.3f}",
               '—' if separation is None else f'{max(0,-separation)*1000:.3f}']
        evidence=next((f for f in ('summary.json','runtime_error.json','cancelled.json','runtime_settings.json') if (root/n/f).exists()),None)
        link=f'<a href="{n}/{evidence}">JSON</a>' if evidence else '—'
        table.append('<tr>'+''.join('<td>'+esc(str(c))+'</td>' for c in cells)+f'<td>{link} · <a href="{n}.log">log</a></td></tr>')
        if s.get('video'):
            videos.append(f'<article><h2>{esc(n)}</h2><p>{esc(s["status"])} / 실제 저장 상태 재생 / 2배속 · review 영상에는 재생용 보조 조명 사용</p><video controls preload="metadata" src="{n}/{s["video"]}"></video><p><a href="{n}/{s["video"]}">{n}/{s["video"]}</a></p></article>')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>S300 · 실제 물리 구동 시험</title>
<style>body{font:16px system-ui;max-width:1400px;margin:32px auto;padding:0 24px;background:#151a20;color:#e8edf3}a{color:#8fd1ff}table{border-collapse:collapse;font-size:13px}td,th{border:1px solid #46515d;padding:8px;text-align:left}video{width:min(100%,960px)}article{margin:30px 0}.note{background:#313124;padding:18px;line-height:1.65}</style>
<h1>S300 + RB5 + V5.1 · 실제 물리 구동 시험</h1>
<p><a href="MOTION_STATUS_KO.md">MOTION_STATUS_KO.md</a> · <a href="results.json">results.json</a></p>
<div class="note">수확 성공은 아직 확인되지 않았습니다. 분리 시험은 주변 식물과 활성 식물의 잎↔식물 접촉 일부를 제외한 구동 확인용입니다.
원래 농장의 초기 겹침 실패를 해결한 결과로 해석하지 마세요. 열매 크기와 그리퍼 28mm 최소 간격은 그대로입니다.
영상은 실제 physics에서 저장한 body pose를 재생하며, 명령만 애니메이션한 영상이 아닙니다.</div>
<h2>전체 시도 기록</h2><p>벽시계 시간은 물리·상태 읽기·결과 저장 구간입니다. 앱/장면 준비·렌더는 별도이며, CPU/GPU 성능 비교 시험이 아닙니다.</p>
<table><tr><th>실행</th><th>상태</th><th>범위</th><th>sim 초</th><th>처리 초</th><th>목표 최대 변위 mm</th><th>최대 관통 mm</th><th>원자료</th></tr>'''+''.join(table)+'</table>'+''.join(videos)+'</html>'
    if (root/'motion_metrics.png').exists():page=page.replace('</html>','<h2>실제 측정값</h2><img style="width:min(100%,1000px)" src="motion_metrics.png"></html>')
    (root/'index.html').write_text(page)
    print(json.dumps(dict(output=str(root/'index.html'),trials=len(rows),videos=len(videos)),ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--plot',action='store_true');a=p.parse_args()
    if a.plot:plot_metrics(a.root)
    summarize(a.root)
