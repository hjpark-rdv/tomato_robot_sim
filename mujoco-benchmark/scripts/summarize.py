"""Aggregate matched runs without counting initial overlap or constraint errors as success."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

def summarize(root):
    root=Path(root);summaries=[];rows=[]
    for f in sorted(root.glob('*/summary.json')):
        d=json.loads(f.read_text());d['run']=f.parent.name;summaries.append(d)
        for r in d['results']:
            row=dict(run=d['run'],**{k:v for k,v in r.items() if isinstance(v,(str,int,float,bool)) or v is None});rows.append(row)
    if not rows:raise ValueError('No completed results')
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with (root/'results.csv').open('w',newline='',encoding='utf-8-sig') as out:
        w=csv.DictWriter(out,fieldnames=fields);w.writeheader();w.writerows(rows)
    table=[]
    for s in summaries:
        rs=s['results'];sim=sum(r['simulated_s'] for r in rs);physics=sum(r['physics_only_wall_s'] for r in rs);wall=sum(r['wall_s'] for r in rs)
        table.append(dict(run=s['run'],count=len(rs),requested_count=s['requested_count'],sim_s=sim,physics_wall_s=physics,replay_wall_s=wall,rtf=sim/wall,
            total_wall_s=s['total_wall_s'],reset_mean_ms=s['reset_mean_s']*1000,reset_100_s=s['reset_100_s'],rss_peak_mib=max(r['rss_mib'] for r in rs),process_cpu_percent=sum(r['process_cpu_percent']*r['wall_s'] for r in rs)/wall,steps_per_second=sum(r['simulated_s']*r['hz'] for r in rs)/physics,
            penetration_trials=sum(r['penetration_steps']>0 for r in rs),initial_overlap_trials=sum(r['initial_penetration_m']>.0005 for r in rs),
            attachment_invalid_trials=sum(not r.get('attachment_valid',False) for r in rs),tunneling_suspect_trials=sum(r['tunneling_suspect_count']>0 and not r['unstable'] for r in rs),
            unstable_trials=sum(r['unstable'] for r in rs),max_stem_mm=max(r['main_stem_max_displacement_m'] for r in rs)*1000,
            max_penetration_mm=max(r['max_penetration_m'] for r in rs)*1000,max_attachment_gap_mm=max(r.get('max_attachment_gap_m',0) for r in rs)*1000))
    matched=[]
    for n in (1,10,100):
        aa=next((s for s in summaries if s['run']==f'isaac_120hz_{n}_implicitfast'),None);bb=next((s for s in summaries if s['run']==f'mujoco_120hz_{n}_implicitfast'),None)
        if not aa or not bb:continue
        for a,b in zip(aa['results'],bb['results']):
            if a['trajectory_id']!=b['trajectory_id']:raise ValueError('Mismatched candidate order')
            fa=root/aa['run']/a['trajectory_id'];fb=root/bb['run']/b['trajectory_id']
            xa=np.load(fa/'states.npz');xb=np.load(fb/'states.npz')
            identical=xa['hook'].shape==xb['hook'].shape and np.allclose(xa['hook'],xb['hook'],atol=1e-12,rtol=0)
            if not identical:raise ValueError('The engines did not receive identical hook commands')
            qa=xa['poses'][0,:,:3];qb=xb['poses'][0,:,:3];error=float(np.max(abs(qa-qb)))
            angle=float(np.max((R.from_quat(xa['poses'][0,:,3:]).inv()*R.from_quat(xb['poses'][0,:,3:])).magnitude()))
            if error>1e-6 or angle>1e-5:raise ValueError('Initial poses are not equivalent within numeric tolerance')
            matched.append(dict(count=n,trajectory=a['trajectory_id'],identical_hook_commands=True,hook_command_max_abs_error=float(np.max(abs(xa['hook']-xb['hook']))),hook_command_tolerance=1e-12,initial_position_error_m=error,initial_rotation_error_rad=angle,
                isaac_result=a['result'],mujoco_result=b['result'],isaac_stem_mm=a['main_stem_max_displacement_m']*1000,mujoco_stem_mm=b['main_stem_max_displacement_m']*1000,
                isaac_target_mm=a['target_max_displacement_m']*1000,mujoco_target_mm=b['target_max_displacement_m']*1000))
    repeatability=[]
    for engine in ('isaac','mujoco'):
        short=root/f'{engine}_120hz_10_implicitfast';long=root/f'{engine}_120hz_100_implicitfast'
        if not short.exists() or not long.exists():continue
        for file in short.glob('*/states.npz'):
            other=long/file.parent.name/'states.npz'
            if not other.exists():continue
            x=np.load(file)['poses'];y=np.load(other)['poses']
            error=float(np.max(np.abs(x[:,:,:3]-y[:,:,:3]))) if x.shape==y.shape else None
            repeatability.append(dict(engine=engine,trajectory=file.parent.name,max_position_error_m=error,passed=error is not None and error<1e-5))
    report=dict(scope='Full plant + prescribed hook, CPU engines, no robot; break disabled on both; NOT a harvest success benchmark',runs=table,matched=matched,repeatability=repeatability)
    (root/'comparison.json').write_text(json.dumps(report,indent=2))
    structure=json.loads((Path(__file__).resolve().parents[1]/'models/plant_original_equivalent.json').read_text())
    report['structure']=structure
    lines=['# Isaac / MuJoCo 원본 식물 비교','',report['scope'],'',
        '| 구조 | Isaac full | MuJoCo original-equivalent |',
        '|---|---:|---:|',
        f"| 식물 강체(열매 포함) | {structure['bodies']} | {structure['bodies']} |",
        f"| 탄성 자유도 | {structure['elastic_dofs']} | {structure['elastic_dofs']} |",
        f"| 별도 열매 자유도 | {structure['fruit_free_dofs']} | {structure['fruit_free_dofs']} |",
        f"| 식물 충돌체 | {structure['plant_colliders']} | {structure['plant_colliders']} |",
        f"| 고리 충돌체 | {structure['hook_colliders']} | {structure['hook_colliders']} |",'',
        '물리 시간은 step 함수만, 실행 시간은 궤적 입력과 상태/접촉 기록까지 포함한다. 전체 시간은 추가 관통 검사·저장까지 포함한다. 초기 로딩 제외.','',
        '| 실행 | 경로 수 | 시뮬레이션 초 | 물리 초 | 실행 초 | RTF | 전체 초 | 관통 경로 | 부착 오차 경로 | 터널링 의심 | 불안정 |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for t in table:lines.append(f"| {t['run']} | {t['count']} | {t['sim_s']:.1f} | {t['physics_wall_s']:.1f} | {t['replay_wall_s']:.1f} | {t['rtf']:.2f} | {t['total_wall_s']:.1f} | {t['penetration_trials']} | {t['attachment_invalid_trials']} | {t['tunneling_suspect_trials']} | {t['unstable_trials']} |")
    lines+=['','## 판정 해석','',
        '- 관통 및 부착 오차 기준은 각각 0.5mm. 미세 접촉의 엔진 차이를 숨기지 않는다.',
        '- 스텝 사이 보간 기반 터널링은 의심 건수다. 0건이 연속 충돌 안전 인증은 아니다.',
        '- 수치 폭주한 실행의 통과 검출은 정상적인 터널링 통계에서 제외한다. RTF 역시 조기 종료한 길이에 대한 값이다.',
        '- capsule/sphere의 관통만 독립 검사한다. 잎/하우징 mesh까지 관통 없음이 보장되지는 않는다.',
        '- 처음부터 겹치는 경로는 initial_overlap으로 분리한다. 정상 성공 사례로 집계하지 않는다.',
        '- MuJoCo mocap과 PhysX kinematic target의 접촉 속도 처리가 다르다. 같은 pose를 입력해도 완전히 같은 접촉 경계 조건은 아니다.',
        '- 별도 최적화 모델은 원본과 다른 충돌 조건이며 같은 조건의 엔진 비교로 사용하지 않는다.',
        '- 초기 위치 및 모든 고리 입력의 동일성 검사는 comparison.json의 matched 참조.','',
        '상세 경로별 값: [results.csv](results.csv), [comparison.json](comparison.json).']
    (root/'comparison.md').write_text('\n'.join(lines)+'\n');print('[비교표 저장]',root)
    (root/'comparison.json').write_text(json.dumps(report,indent=2))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();summarize(a.root)
