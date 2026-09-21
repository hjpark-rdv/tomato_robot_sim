"""Sequential capacity/contact screen using the same saved trajectory in all slots.

Not a unique-pose search or a complete dataset throughput benchmark. Keep invalid
physics outcomes distinct from consistent clone execution and successful harvest.
"""
import argparse
from collections import Counter
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from dataset_design import write_json
from gpu_scale_validation import compare_replays
from gpu_physics_errors import invalid_physics_message

HERE=Path(__file__).resolve().parent


def gpu_sample():
    try:
        result=subprocess.run(['nvidia-smi','--query-gpu=memory.used,memory.total,utilization.gpu',
            '--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=5,check=True)
        used,total,util=[float(v.strip()) for v in result.stdout.splitlines()[0].split(',')]
        return dict(used_mib=used,total_mib=total,utilization_percent=util)
    except (OSError,subprocess.SubprocessError,ValueError,IndexError):return None


def measure(directory, exit_code, process_wall_s, samples):
    report=json.loads((directory/'report.json').read_text()) if (directory/'report.json').exists() else {}
    motion=report.get('motion',{});outcomes=motion.get('outcomes',[])
    errors=[s.strip() for s in (directory/'run.log').read_text(errors='replace').splitlines()
            if invalid_physics_message(s,s)]
    count=report.get('num_envs',0)
    complete=exit_code==0 and report.get('complete',False) and len(outcomes)==count and bool(count) and not errors
    prefix=report.get('motion_prefix_10s')
    invalid=sum(r.get('physics_valid') is False or r.get('classification',{}).get('result')=='invalid_physics' for r in outcomes)
    return dict(num_envs=count,complete=complete,exit_code=exit_code,process_wall_s=process_wall_s,
        initialization_wall_s=report.get('initialization_wall_s'),prefix_10s=prefix,
        motion_wall_s=motion.get('wall_s'),batch_sim_s=motion.get('batch_sim_s'),
        physics_step_timings=motion.get('physics_step_timings'),
        motion_process_cpu_percent=(100*report['motion_process_cpu_s']/motion['wall_s'] if motion.get('wall_s') and 'motion_process_cpu_s' in report else None),
        counts=dict(Counter(r.get('classification',{}).get('result') for r in outcomes)),
        valid_physics_count=sum(r.get('physics_valid') is True for r in outcomes),invalid_physics_count=invalid,
        contact_envs=sum(r.get('first_contact_object') is not None for r in outcomes),
        max_tool_penetration_mm=max((r.get('max_tool_penetration_mm',0.) for r in outcomes),default=None),
        reset_and_break_api_passed=report.get('api_checks_passed',False),
        cross_environment_contacts=len(report.get('contact_routing_faults',[])),native_errors=errors[:10],
        aggregate_prefix_sim_seconds_per_wall_second=(count*prefix['batch_sim_s']/prefix['wall_s'] if prefix and prefix['active_envs']==count else None),
        peak_total_gpu_memory_mib=max((s['gpu']['used_mib'] for s in samples if s['gpu']),default=None),
        peak_worker_cpu_percent=max((s['worker_cpu_percent'] for s in samples if s['worker_cpu_percent'] is not None),default=None),
        mean_worker_cpu_percent=(sum(s['worker_cpu_percent'] for s in samples if s['worker_cpu_percent'] is not None)/max(1,sum(s['worker_cpu_percent'] is not None for s in samples))),
        memory_scope='Whole GPU, including pre-existing user sessions; not per-process allocation')


def publish_tables(root):
    """Create a readable report without promoting an invalid replay to success."""
    root=Path(root)
    report=json.loads((root/'benchmark.json').read_text())
    rows=[];reference=None
    for run in report['runs']:
        prefix=run.get('prefix_10s') or {}
        timings=run.get('physics_step_timings') or {}
        rate=run.get('aggregate_prefix_sim_seconds_per_wall_second')
        if run['num_envs']==1:reference=rate
        row=dict(num_envs=run['num_envs'],initialization_s=run.get('initialization_wall_s'),
            prefix_10s_wall_s=prefix.get('wall_s'),aggregate_sim_s_per_wall_s=rate,
            throughput_vs_one=rate/reference if rate and reference else None,
            full_replay_wall_s=run.get('motion_wall_s'),
            native_physics_s=timings.get('physics_s'),
            outside_native_io_s=(run['motion_wall_s']-sum(timings.get(k,0.) for k in ('write_s','physics_s','read_s')) if run.get('motion_wall_s') is not None else None),
            motion_process_cpu_percent=run.get('motion_process_cpu_percent'),
            valid_physics_count=run['valid_physics_count'],invalid_physics_count=run['invalid_physics_count'],
            max_penetration_mm=run['max_tool_penetration_mm'],
            clones_match=run.get('clone_comparison_passed',False),
            contact_screen_passed=run.get('contact_screen_passed',False),
            peak_gpu_memory_mib=run.get('peak_total_gpu_memory_mib'))
        rows.append(row)
    if rows:
        with (root/'benchmark.csv').open('w',newline='',encoding='utf-8-sig') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    lines=['# 초경량 GPU 병렬 실행 비교','',
        '같은 후보 49의 저장 경로를 각 환경에 재생했다. 서로 다른 후보를 탐색한 수치가 아니다.',
        '120Hz / PGS 64회 / ultralight / GPU 물리 / 매 스텝 관통 검사 / 화면 없음.',
        '10초 구간 시간은 초기화·경로 계산을 제외하고 물리·상태 읽기·판정·관통 검사를 포함한다.',
        '사용자 세션은 중단하지 않았으며 GPU 메모리는 해당 세션을 포함한 전체 사용량이다.','',
        '| 환경 | 초기화 초 | 같은 10초 구간 처리 초 | 합산 처리량¹ | GPU 1환경 대비 | 관통 제외 | 최대 관통 mm | 복제 결과 일치 |',
        '|---:|---:|---:|---:|---:|---:|---:|:---:|']
    def number(v):return '—' if v is None else f'{v:.2f}'
    for row in rows:
        lines.append('| '+ ' | '.join([str(row['num_envs']),number(row['initialization_s']),number(row['prefix_10s_wall_s']),
            number(row['aggregate_sim_s_per_wall_s']),number(row['throughput_vs_one']),str(row['invalid_physics_count']),
            number(row['max_penetration_mm']),'예' if row['clones_match'] else '아니오'])+' |')
    lines+=['','¹ 환경 수 × 10초 / 실제 초. 전체 후보/분이나 수확 성공률이 아니다.',
        '접촉 시험은 관통·과도 변위 등 기존 종료 조건까지 실행했다. 중간에 물리 오류로 끝난 실행을 성공으로 집계하지 않는다.',
        '복제 결과 일치는 같은 잘못된 접촉을 재현한 경우에도 참일 수 있다. 접촉 통과 여부는 별도다.','',
        '접촉 검사까지 통과한 환경 수: '+(', '.join(str(r['num_envs']) for r in rows if r['contact_screen_passed']) or '없음'),
        '전체 측정 완료: '+str(report['execution_complete'])]
    lines+=['','## 전체 재생 시간의 구성','',
        '| 환경 | 전체 실행 초 | 물리 계산 초 | 쓰기·물리·읽기 외 초 | 실행 CPU % |',
        '|---:|---:|---:|---:|---:|']
    for row in rows:
        lines.append('| '+' | '.join([str(row['num_envs']),number(row['full_replay_wall_s']),
            number(row['native_physics_s']),number(row['outside_native_io_s']),number(row['motion_process_cpu_percent'])])+' |')
    lines+=['','외부 시간에는 환경별 판정·상태 처리·검사·기록 등이 포함된다. 순수 CPU 연산 시간으로 해석하지 않는다.',
        'CPU 100%는 코어 하나 분량이다. 초기화 시간은 Kit 자체 시작 시간을 제외한다.']
    (root/'benchmark.md').write_text('\n'.join(lines)+'\n')
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture',type=Path,required=True)
    parser.add_argument('--env-counts',default='1,16,32,64')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--timeout-seconds',type=float,default=1800.)
    args=parser.parse_args()
    counts=[int(n) for n in args.env_counts.split(',')]
    if not counts or counts[0]!=1 or len(set(counts))!=len(counts) or any(n<1 or n>128 for n in counts):
        parser.error('Use unique counts starting with 1, at most 128')
    if args.timeout_seconds<=0:parser.error('timeout-seconds must be positive')
    fixture=args.fixture.resolve()
    for name in ('candidate.json','planned_commands.npy'):
        if not (fixture/name).is_file():parser.error('Missing fixture '+name)
    root=(args.output or HERE/'runs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_ultralight_gpu_scale')).resolve()
    root.mkdir(parents=True,exist_ok=False)
    report=dict(scope='Identical saved candidate in every slot; capacity and contact regression, not unique pose throughput',
        fixture=str(fixture),env_counts=counts,physics=dict(device='gpu',hz=120,solver='pgs',position_iterations=64,
            velocity_iterations=4,armature=.0005,plant_resolution='ultralight',contact_policy='legacy',penetration_guard_m=.0005),
        baseline_gpu=gpu_sample(),execution_complete=False,runs=[])
    write_json(root/'benchmark.json',report)
    print('[병렬 비교 폴더]',root,flush=True)
    for count in counts:
        directory=root/f'env_{count:03d}';directory.mkdir()
        command=[sys.executable,'-u',str(HERE/'gpu_probe_worker.py'),'--output',str(directory),'--fixture',str(fixture),
            '--mode','gpu','--num-envs',str(count),'--physics-hz','120','--solver','pgs','--position-iterations','64',
            '--velocity-iterations','4','--joint-armature','.0005','--plant-resolution','ultralight',
            '--batched-io','--native-replication','--gpu-partitions','1','--manual-usd-sync','--guard-tool-contacts','--headless']
        write_json(directory/'command.json',command)
        print('[병렬 비교 시작]',count,'환경',flush=True)
        samples=[];began=time.monotonic();last_sample=began-10;last_log=began;prior_cpu=None
        with (directory/'run.log').open('w') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                while process.poll() is None:
                    now=time.monotonic()
                    if now-began>args.timeout_seconds:raise TimeoutError(f'{count} environments exceeded timeout')
                    if now-last_sample>=5:
                        cpu=None
                        try:
                            fields=Path(f'/proc/{process.pid}/stat').read_text().split(') ',1)[1].split()
                            ticks=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK')
                            if prior_cpu:cpu=100*(ticks-prior_cpu[1])/(now-prior_cpu[0])
                            prior_cpu=(now,ticks)
                        except (OSError,IndexError,ValueError):pass
                        samples.append(dict(elapsed_s=now-began,gpu=gpu_sample(),worker_cpu_percent=cpu))
                        last_sample=now
                        write_json(directory/'resource_samples.json',samples)
                    if now-last_log>=30:
                        current=json.loads((directory/'report.json').read_text()) if (directory/'report.json').exists() else {}
                        print('[병렬 비교 진행]',count,'환경',round(now-began,1),'초',current.get('phase','app_start'),flush=True)
                        last_log=now
                    time.sleep(1)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=15)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        row=measure(directory,process.returncode,time.monotonic()-began,samples)
        if row['complete']:
            comparison=compare_replays(root/'env_001',directory)
            write_json(directory/'clone_comparison.json',comparison)
            row['clone_comparison_passed']=comparison['passed']
            row['mismatched_clones']=sum(not c['passed'] for c in comparison.get('checks',[]))
        row['contact_screen_passed']=bool(row['complete'] and row.get('clone_comparison_passed') and
            row['reset_and_break_api_passed'] and row['valid_physics_count']==count and row['contact_envs']==count and not row['cross_environment_contacts'])
        report['runs'].append(row);write_json(root/'benchmark.json',report)
        print('[병렬 비교 결과]',json.dumps(row,ensure_ascii=False),flush=True)
        if not row['complete']:
            report['stopped_reason']='Native error or incomplete run; larger capacities not attempted'
            write_json(root/'benchmark.json',report);return
    report['execution_complete']=True
    write_json(root/'benchmark.json',report)
    publish_tables(root)
    print('[병렬 비교 완료]',root,flush=True)


if __name__=='__main__':main()
