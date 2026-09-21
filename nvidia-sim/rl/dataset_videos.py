"""Select real dataset outcomes, replay saved commands, compare, and render 2x clips.

Offline rendering never advances physics. A single-env replay is explicitly
compared with the original batched trial; discrepancies are never relabelled as
representative successes. Run with Isaac's Python and an NVIDIA X display.
"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import html
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def select(rows, limit=4):
    """Deterministic category coverage; partial entry is not full hook success."""
    usable = [r for r in rows if r.get('executed') and r.get('result') not in ('incomplete','invalid_physics')
              and r.get('physics_valid') is not False]
    groups = [
        ('success', '완전 걸림 성공', [r for r in usable if r.get('hook_success')]),
        ('partial', '부분 성공: 열매 중심 진입', [r for r in usable if r.get('center_entry_safe') and not r.get('hook_success')]),
        ('miss', '실패: 걸림 없음', [r for r in usable if r['result']=='miss' and not r.get('center_entry_safe')]),
        ('displacement', '실패: 허용 변위 초과', [r for r in usable if r['result']=='excessive_displacement' and not r.get('center_entry_safe')]),
        ('non_target', '실패: 비목표 접촉', [r for r in usable if r['result']=='non_target_contact']),
    ]
    chosen=[]; seen=set()
    for category, title, pool in groups:
        pool.sort(key=lambda r: (-float(r.get('center_entry_duration_s',0) or 0), r['candidate_id']))
        for row in pool:
            if row['candidate_id'] not in seen:
                chosen.append(dict(category=category,title=title,candidate_id=row['candidate_id']))
                seen.add(row['candidate_id']); break
        if len(chosen)>=limit: break
    return chosen


def normalize_object(value):
    return re.sub(r'/env_\d+/', '/env_N/', value) if value else value


def compare(source, replay):
    """Outcome + trajectory metrics, NOT a claim of bitwise repeatability."""
    classification=replay['classification']
    checks={}
    source_device=source.get('physical_inputs',{}).get('backend',{}).get('physics_device','gpu')
    replay_device=replay.get('physics_device','gpu')
    checks['physics_device']=dict(source=source_device,replay=replay_device,passed=source_device==replay_device)
    def model(row):
        return row.get('plant_resolution',row.get('physical_inputs',{}).get('backend',{}).get('plant_resolution','full'))
    a,b=model(source),model(replay)
    checks['plant_resolution']=dict(source=a,replay=b,passed=a==b)
    def appendages(row):
        return row.get('main_appendage_collisions',row.get('physical_inputs',{}).get('backend',{}).get('main_appendage_collisions','keep'))
    a,b=appendages(source),appendages(replay)
    checks['main_appendage_collisions']=dict(source=a,replay=b,passed=a==b)
    for key in ('result','hook_success'):
        checks[key] = dict(source=source.get(key), replay=classification.get(key), passed=source.get(key)==classification.get(key))
    for key in ('retained_hook','center_entry_achieved','center_entry_safe','lift_completed','target_broken','other_broken','abort_reason'):
        checks[key] = dict(source=source.get(key),replay=replay.get(key),passed=source.get(key)==replay.get(key))
    src=normalize_object(source.get('first_contact_object'))
    dst=normalize_object((replay.get('first_contact') or {}).get('object'))
    checks['first_contact_object']=dict(source=src,replay=dst,passed=src==dst)
    # Declared before replay: up to 2 mm maximum-displacement difference,
    # 0.1 s termination-time difference. These do not change dataset thresholds.
    for key in ('target_max_displacement_m','main_stem_max_displacement_m'):
        a,b=source.get(key),replay.get(key)
        delta=abs(a-b) if a is not None and b is not None else None
        checks[key]=dict(source=a,replay=b,absolute_difference=delta,tolerance=.002,passed=delta is not None and delta<=.002)
    ticks=abs(source['executed_steps']-replay['executed_steps'])
    checks['executed_steps']=dict(source=source['executed_steps'],replay=replay['executed_steps'],tolerance=6,passed=ticks<=6)
    return dict(passed=all(c['passed'] for c in checks.values()),scope='Outcome and aggregate metrics within declared tolerances; not bitwise trajectory equality',checks=checks)


def compare_trace(source, replay):
    import numpy as np
    n=min(len(source),len(replay))
    checks={'length':dict(passed=abs(len(source)-len(replay))<=6,source=len(source),replay=len(replay),tolerance_steps=6)}
    for key,tolerance in (('joints',.02),('target_displacement_m',.002),('main_stem_displacement_m',.002)):
        a=np.asarray([r[key] for r in source[:n]]);b=np.asarray([r[key] for r in replay[:n]])
        difference=float(np.max(np.abs(a-b))) if n else None
        checks[key]=dict(max_absolute_difference=difference,tolerance=tolerance,
                        passed=difference is not None and difference<=tolerance)
        if key=='joints' and n:
            lift_difference=float(np.max(np.abs(a[:,0]-b[:,0])))
            checks['lift']=dict(max_absolute_difference_m=lift_difference,tolerance_m=.002,passed=lift_difference<=.002)
    checks['phases']=dict(passed=n>0 and all(a['phase']==b['phase'] for a,b in zip(source,replay)))
    return dict(passed=all(c['passed'] for c in checks.values()),checks=checks,
                scope='Aligned control steps; arm joint tolerance 0.02 rad, lift/displacement tolerance 2 mm')


def run(command, log, env=None):
    print('[대표 영상 실행]', ' '.join(map(str,command)), flush=True)
    with Path(log).open('w') as stream:
        subprocess.run(list(map(str,command)), stdout=stream, stderr=subprocess.STDOUT, env=env, check=True)


def representative_video(row):
    """Replay agreement alone must not override a known physical defect."""
    return bool(row['comparison']['passed'] and row.get('physical_audit',{}).get('passed') is not False)


def gallery(root, manifest):
    e=html.escape
    cards=[]; diagnostics=[]
    for row in manifest['videos']:
        status='원본 결과와 재실행 비교 통과' if row['comparison']['passed'] else '주의: 원본과 재실행 결과 불일치 — 대표 결과로 사용하지 않음'
        if row.get('physical_audit',{}).get('passed') is False:
            status += ' · 물리 검증 실패: '+row['physical_audit'].get('reason','관통 등 물리 오류')+' — 유효 경로로 사용하지 않음'
        collection=cards if representative_video(row) else diagnostics
        collection.append(f'''<section><h2>{e(row['title'])} · {e(row['candidate_id'])}</h2><p>{status}</p>
<video controls preload="metadata" poster="{e(Path(row['video']).with_suffix('.png').as_posix())}" src="{e(row['video'])}"></video><p>원본: {e(row['source_result'])} / 재실행: {e(row['replay_result'])}</p>
<a href="{e(row['comparison_file'])}">비교 상세</a> · <a href="{e(row['candidate_file'])}">원본 판정</a></section>''')
    counts=e(json.dumps(manifest['counts'],ensure_ascii=False))
    missing='' if manifest['counts'].get('success_target_hook',0) else '<p><b>이번 시험에는 완전 걸림 성공이 없습니다. 열매 중심 진입은 별도의 부분 성공입니다.</b></p>'
    diagnostic_section=('<h1>추가 진단: 재실행 불일치 또는 물리 오류</h1>'+''.join(diagnostics)) if diagnostics else ''
    plant_label={'full':'원본 식물 모델','light':'경량 식물 모델','ultralight':'초경량 식물 모델'}[manifest.get('plant_resolution','full')]
    if manifest.get('main_appendage_collisions','keep')=='ignore':plant_label+=' · 주줄기 잎/잘린 가지 충돌 무시'
    (root/'index.html').write_text(f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>고리 경로 대표 영상</title>
<style>body{{background:#151a21;color:#eee;font:16px sans-serif;max-width:1320px;margin:30px auto;padding:20px}}video{{width:100%}}section{{background:#202833;padding:20px;margin:24px 0}}a{{color:#86c8ff}}</style>
<h1>고리 경로 시험 · 대표 영상 (2배속)</h1><p>{plant_label} · 원본 후보 {manifest['total_candidates']}개 · {counts}</p>{missing}
<p>대표 영상: {len(cards)}개 / 별도 진단: {len(diagnostics)}개.</p>
<p>저장된 로봇 관절 명령을 원본의 물리 설정으로 재실행해 실제 물리 상태를 녹화했습니다. 원본과 재실행이 같다는 것만으로 관통 등 물리 오류가 없다는 뜻은 아닙니다. 영상은 외부 관찰용이며 학습용 RGB-D와 다릅니다.</p>
{''.join(cards)}{diagnostic_section}<a href="manifest.json">전체 메타데이터</a></html>''')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--max-videos',type=int,default=4)
    parser.add_argument('--display',default=':0')
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    if not 1<=args.max_videos<=12: parser.error('max-videos must be 1..12')
    dataset=args.run_dir.resolve();cfg=read(dataset/'config.json')
    if cfg['target']!='Tomato_05' or cfg['solver']!='PGS' or cfg.get('trajectory_mode')!='staged6d':
        parser.error('This replay adapter currently requires Tomato_05 staged6d, PGS')
    summary=read(dataset/'summary.json')
    if not summary.get('execution_complete') or (dataset/'execution_error.json').exists():
        parser.error('Dataset execution must finish successfully before video processing')
    root=(args.output or dataset/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_representative_videos_2x')).resolve()
    if root.exists() and not args.resume: parser.error('Output exists; choose a new output or --resume')
    root.mkdir(parents=True,exist_ok=True)
    rows=[read(p) for p in sorted((dataset/'results').glob('*/candidate.json'))]
    manifest=dict(schema_version=1,dataset=str(dataset),created=datetime.now().isoformat(),total_candidates=len(rows),
        plant_resolution=cfg.get('plant_resolution','full'),
        main_appendage_collisions=cfg.get('main_appendage_collisions','keep'),
        counts=dict(Counter(r['result'] for r in rows)),center_entry_safe_count=sum(bool(r.get('center_entry_safe')) for r in rows),
        selection=select(rows,args.max_videos),videos=[],complete=False)
    save(root/'manifest.json',manifest)
    started=time.perf_counter()
    for selected in manifest['selection']:
        candidate_id=selected['candidate_id'];fixture=dataset/'results'/candidate_id;source=read(fixture/'candidate.json')
        command_hash=hashlib.sha256((fixture/'planned_commands.npy').read_bytes()).hexdigest()
        replay_dir=root/'replays'/candidate_id;replay_dir.mkdir(parents=True,exist_ok=True)
        identity=dict(dataset=str(dataset),command_sha256=command_hash,config_sha256=hashlib.sha256((dataset/'config.json').read_bytes()).hexdigest())
        identity_path=replay_dir/'source_identity.json'
        if identity_path.exists() and read(identity_path)!=identity: raise RuntimeError('Replay source changed')
        save(identity_path,identity)
        capture=replay_dir/'captures'/candidate_id
        if not (replay_dir/'report.json').exists() or not read(replay_dir/'report.json').get('complete'):
            command=[sys.executable,'-u',HERE/'gpu_probe_worker.py','--headless','--mode',cfg.get('physics_device','gpu'),'--solver','pgs',
                '--batched-io','--native-replication','--gpu-partitions','1','--num-envs','1','--physics-hz',str(cfg['physics_hz']),
                '--joint-armature',str(cfg['elastic_joint_armature']),'--position-iterations',str(cfg['position_iterations']),
                '--record-physics-video','--fixture',fixture,'--output',replay_dir]
            command+=['--contact-policy',cfg.get('contact_policy','legacy')]
            command+=['--velocity-iterations',str(cfg.get('velocity_iterations',4))]
            command+=['--plant-resolution',cfg.get('plant_resolution','full')]
            command+=['--main-appendage-collisions',cfg.get('main_appendage_collisions','keep')]
            if 'physical_audit' in source:command+=['--guard-tool-contacts']
            run(command,replay_dir/'run.log')
        report=read(replay_dir/'report.json');replay=report['motion']['outcomes'][0]
        replay['physics_device']=report['backend'].get('physics_device','gpu' if report['backend'].get('gpu_dynamics') else 'cpu')
        replay['plant_resolution']=report['backend'].get('plant_resolution','full')
        replay['main_appendage_collisions']=report['backend'].get('main_appendage_collisions','keep')
        if not report.get('complete') or report['fixture']['commands_sha256']!=command_hash:
            raise RuntimeError('Incomplete replay or different saved joint commands')
        comparison=compare(source,replay);comparison['command_sha256']=command_hash
        comparison['trace']=compare_trace(read(fixture/'trace.json'),read(replay_dir/'trace_0.json'))
        comparison['passed'] &= comparison['trace']['passed']
        save(replay_dir/'comparison.json',comparison)
        record=read(capture/'recording.json');record['validation']['dataset_replay']=comparison
        if not comparison['passed']:
            record['title']='REPLAY MISMATCH | '+candidate_id
            record['subtitle']='Replay outcome differs from batched dataset; diagnostic only.'
        save(capture/'recording.json',record)
        scene=read(capture/'scene.json');c=scene['target_center']
        focus=[c[0]+.02,c[1],c[2]+.025]
        close=[c[0]+.005,c[1],c[2]+.012]
        views=[dict(name='SIDE | approach and plant',target=focus,eye=[focus[0]+.13,focus[1]+.31,focus[2]+.13],lens=40),
               dict(name='CLOSE-UP | target and ring',target=close,eye=[close[0],close[1]+.18,close[2]+.10],lens=50)]
        view_path=replay_dir/'views.json';save(view_path,views)
        video=root/(selected['category']+'_'+candidate_id+'_2x.mp4')
        environment=os.environ.copy();environment.update(DISPLAY=args.display,__GLX_VENDOR_LIBRARY_NAME='nvidia');environment.pop('LIBGL_ALWAYS_SOFTWARE',None)
        base=['blender','-t','4','--python',HERE/'pose_video_render.py','--','--scene',capture/'scene.npz','--capture',capture,
              '--output',video,'--gpu','--views',view_path,'--playback-speed','2']
        if not video.exists() or not video.with_suffix('.json').exists():
            run(base,video.with_suffix('.render.log'),environment)
        # Decode every frame: an existing partial mp4 is not silently accepted.
        run(['ffmpeg','-v','error','-xerror','-i',video,'-f','null','-'],video.with_suffix('.decode.log'))
        info=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(video)]))
        stream=next(s for s in info['streams'] if s['codec_type']=='video')
        expected_frames=math.ceil(record['simulated_duration_s']/2*30)+30
        if int(stream['nb_frames'])!=expected_frames or stream['r_frame_rate']!='30/1':
            raise RuntimeError('Video frame count/rate does not match the complete recording')
        save(video.with_suffix('.ffprobe.json'),info)
        if not (root/(video.stem+'_t0.000.png')).exists():
            duration=record['simulated_duration_s']
            previews=sorted(set([0,round(duration*.65,2),round(duration*.85,2),round(duration,2)]))
            run(base+['--preview-times',','.join(map(str,previews))],video.with_suffix('.preview.log'),environment)
        manifest['videos'].append(dict(**selected,video=video.name,comparison=comparison,
            comparison_file=str((replay_dir/'comparison.json').relative_to(root)),candidate_file=os.path.relpath(fixture/'candidate.json',root),
            source_result=source['result'],replay_result=replay['classification']['result']))
        if (fixture/'physical_audit.json').exists():
            manifest['videos'][-1]['physical_audit']=read(fixture/'physical_audit.json')
        save(root/'manifest.json',manifest);gallery(root,manifest)
        print('[대표 영상 완료]',candidate_id,'원본 비교:',comparison['passed'],flush=True)
    manifest.update(complete=True,wall_s=time.perf_counter()-started,
        representative_video_count=sum(representative_video(v) for v in manifest['videos']),
        diagnostic_video_count=sum(not representative_video(v) for v in manifest['videos']))
    save(root/'manifest.json',manifest);gallery(root,manifest)
    print('[대표 영상 모음]',root/'index.html',flush=True)


if __name__=='__main__':main()
