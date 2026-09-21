"""Sequential 10s native robot/tool cost comparison with measured tool replay."""
import argparse,json,subprocess,csv,sys,os,shutil
from datetime import datetime
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
HOME=Path(__file__).resolve().parents[1]
ROOT=HOME.parent
DEFAULT_TRACE=ROOT/'nvidia-sim/rl/runs/20260921_213000_single_view_batched/results/candidate_00049/trace.json'

def report(root):
    rows=[];states={}
    for name in ('hook_headless','arm_headless','hook_gui','arm_hidden_gui','arm_visible_gui'):
        summary=json.loads((root/name/'summary.json').read_text());r=summary['results'][0];states[name]=np.load(root/name/r['trajectory_id']/'states.npz')
        rows.append(dict(case=name,simulation_context_initialization_s=summary['initialization_s'],simulated_s=r['simulated_s'],wall_s=r['wall_s'],physics_s=r['physics_only_wall_s'],render_s=r.get('render_wall_s',0),other_s=r['wall_s']-r['physics_only_wall_s']-r.get('render_wall_s',0),rtf=r['rtf'],cpu_percent=r['process_cpu_percent'],rss_mib=r['rss_mib'],result=r['result'],contact_steps=r['contact_steps'],max_penetration_mm=r['max_penetration_m']*1000,max_attachment_gap_mm=r['max_attachment_gap_m']*1000,total_with_audit_save_s=summary['total_wall_s']))
    reference=states['arm_headless'];checks={}
    for name,state in states.items():
        delta=state['actual_hook'][:,:3]-reference['actual_hook'][:,:3]
        angles=(R.from_quat(reference['actual_hook'][:,3:]).inv()*R.from_quat(state['actual_hook'][:,3:])).magnitude()
        checks[name]=dict(max_hook_position_error_m=float(np.linalg.norm(delta,axis=1).max()),max_hook_rotation_error_rad=float(angles.max()),max_plant_position_difference_m=float(np.max(abs(state['poses'][:,:-1,:3]-reference['poses'][:,:-1,:3]))))
        if name.startswith('arm_'):
            checks[name]['identical_native_states']=bool(np.array_equal(state['poses'],reference['poses']))
            checks[name]['identical_recorded_joint_positions']=bool(np.array_equal(state['robot_joints'],reference['robot_joints']))
            if not checks[name]['identical_native_states']:raise RuntimeError('Rendering changed robot/plant replay: '+name)
            if not checks[name]['identical_recorded_joint_positions']:raise RuntimeError('Rendering changed robot joints: '+name)
        if checks[name]['max_hook_position_error_m']>1e-5:raise RuntimeError('Measured hook replay mismatch: '+name)
    doc=dict(scope=f"One {rows[0]['simulated_s']:g}s candidate49 prefix, full plant, CPU PGS 240Hz 64/4 iterations, break disabled. No IK/planning. Dynamic arm follows saved commands; kinematic hook-only follows measured actual arm motion. GUI 960x720,30Hz, same wide camera. Single trial per configuration; not a statistical benchmark.",rows=rows,checks=checks)
    (root/'comparison.json').write_text(json.dumps(doc,indent=2))
    with (root/'comparison.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    labels=dict(hook_headless='고리만 · 화면 없음',arm_headless='팔 포함 · 화면 없음',hook_gui='고리만 · 화면 표시',arm_hidden_gui='팔 물리 포함 · 팔 외형 숨김',arm_visible_gui='팔 물리 포함 · 팔 외형 표시')
    lines=['# 로봇 팔의 계산 비용 비교','',doc['scope'],'','| 조건 | 실행 초 | 물리 초 | 렌더링 초 | 기타 초 | RTF |','|---|---:|---:|---:|---:|---:|']
    for r in rows:lines.append(f"| {labels[r['case']]} | {r['wall_s']:.2f} | {r['physics_s']:.2f} | {r['render_s']:.2f} | {r['other_s']:.2f} | {r['rtf']:.2f} |")
    lines+=['','초기화와 후처리/저장은 위 실행 시간에서 제외한다. 전체 값은 CSV/JSON에 별도 기록한다.',
        'simulation_context_initialization_s는 Kit 실행/원본 USD 로딩을 제외한 일부 준비 시간이다. 전체 앱 시작 시간으로 해석하지 않는다.',
        '렌더링 시간은 USD 자세 동기화와 sim.render를 포함한다. 순수 GPU 연산 시간은 아니다.',
        '식물 화면은 기존 비교용 형상(줄기/잎 충돌 proxy + 열매/꽃받침 mesh)이다. 하우스 전체 GUI나 원본 식물 skin 갱신 비용과 같지 않다.',
        '팔의 관절 구동·충돌은 실제로 계산한다. 고리만 조건은 측정한 팔 끝 경로를 직접 재생하므로 접촉 구동 방식은 다르다.',
        '세 팔 조건의 상태 일치와 고리 경로 오차는 JSON의 checks에서 확인할 수 있다.',
        '한 동작·한 번씩 측정한 수치다. 기존 전체 경로 계획/후처리 프로그램의 병목을 이 차이만으로 설명하지 않는다.']
    (root/'comparison.md').write_text('\n'.join(lines)+'\n')
    html=['<meta charset="utf-8"><title>로봇 팔 비용 비교</title><style>body{font:18px sans-serif;margin:30px;background:#17202b;color:white}a{color:skyblue}img{max-width:95vw;width:960px}pre{white-space:pre-wrap}</style><h1>로봇 팔 비용 비교</h1><a href="comparison.csv">CSV</a> · <a href="comparison.json">JSON/경로 검증</a><pre>'+ '\n'.join(lines)+'</pre>']
    for name in ('hook_gui','arm_hidden_gui','arm_visible_gui'):
        html.append(f'<h2>{labels[name]}</h2><img src="{name}/gui_final.png">')
    (root/'index.html').write_text('\n'.join(html));print('[팔 비교 완료]',root,flush=True)
    return doc

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path);p.add_argument('--resume',action='store_true');p.add_argument('--trace',type=Path,default=DEFAULT_TRACE);p.add_argument('--seconds',type=float,default=10.);a=p.parse_args()
    root=(a.output or HOME/'outputs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_robot_cost')).resolve();root.mkdir(parents=True,exist_ok=a.resume)
    if not (root/'command_suite.json').exists():
        t=next(t for t in json.loads((HOME/'assets/reference/suite.json').read_text()) if t['id']=='candidate_00049');count=round(a.seconds/t['sample_dt'])+1
        if count>len(t['poses']):raise ValueError('Recorded trajectory too short')
        t['poses']=t['poses'][:count];t['duration_s']=(count-1)*t['sample_dt']
        if 'phases' in t:t['phases']=t['phases'][:count]
        (root/'command_suite.json').write_text(json.dumps([t]))
    jobs=('arm_headless','hook_headless','arm_hidden_gui','arm_visible_gui','hook_gui')
    for name in jobs:
        dest=root/name
        if not (a.resume and (dest/'summary.json').exists()):
            cmd=['/root/isaaclab_env/bin/python','-u',str(HOME/'scripts/benchmark_isaac.py'),'--hz','240','--count','1','--recovery','0','--camera-scale','3','--output',str(dest),'--suite',str(root/('command_suite.json' if name.startswith('arm_') else 'actual_hook_suite.json'))]
            if name.startswith('arm_'):cmd+=['--robot-trace',str(a.trace),'--robot-visuals','hidden' if name=='arm_hidden_gui' else 'visible']
            if name.endswith('_gui'):cmd+=['--gui','--exit-on-finish']
            (root/(name+'_command.json')).write_text(json.dumps(cmd,indent=2));print('[팔 비교 시작]',name,flush=True)
            with (root/(name+'.log')).open('w') as log:subprocess.run(cmd,cwd=ROOT,env=dict(os.environ,DISPLAY=':0'),stdout=log,stderr=subprocess.STDOUT,check=True)
        if name=='arm_headless':
            s=np.load(dest/'candidate_00049/states.npz');trial=dict(id='candidate_00049',source='Measured native tool motion from arm_headless',times_s=s['times_s'].tolist(),poses=s['actual_hook'].tolist(),category='actual_arm_replay',duration_s=float(s['times_s'][-1]))
            (root/'actual_hook_suite.json').write_text(json.dumps([trial]))
        print('[팔 비교 결과]',name,json.loads((dest/'summary.json').read_text())['results'][0]['wall_s'],'초',flush=True)
    report(root)
if __name__=='__main__':main()
