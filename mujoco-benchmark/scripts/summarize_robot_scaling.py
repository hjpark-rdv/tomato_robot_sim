"""Combine CPU/MJLab raw repeat records without mixing startup and rollout."""
import argparse,json,csv,statistics,html
from pathlib import Path
HOME=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--gpu-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);rows=[];raw=[]
 for name,version in [('20260922_robot_cpu_scaling','3.13.0'),('20260922_robot_cpu_scaling64','3.13.0'),('20260922_robot_cpu_mjlab_matched','3.11.0')]:
  root=HOME/'outputs'/name
  for f in sorted(root.glob('cpu_*/timings.json')):
   d=json.loads(f.read_text());samples=[json.loads(x) for x in (f.parent/'resources.jsonl').read_text().splitlines()];peak=max(sum(y['rss_mib'] for y in x['workers']) for x in samples)
   rounds=d['rounds'];vals=[x['wall_s'] for x in rounds]
   row=dict(engine='CPU',version=version,environments=d['workers'],mean_wall_s=statistics.mean(vals),stdev_wall_s=statistics.stdev(vals) if len(vals)>1 else 0,aggregate_rtf=statistics.mean(x['aggregate_rtf'] for x in rounds),startup_and_warmup_s=d['startup_and_warmup_wall_s'],peak_ram_mib=peak,peak_gpu_system_mib=max(float(x['gpu'].split(',')[0]) for x in samples if 'gpu' in x),invalid_repeats=sum(x['unstable_count']>0 for x in rounds),raw_path=str(f))
   rows.append(row);raw.append(dict(summary=row,repeats=rounds,worker_records=d['results']))
 for f in sorted(a.gpu_root.glob('gpu_*/summary.json')):
  d=json.loads(f.read_text())
  if d.get('error') or not d.get('rounds'):continue
  samples=[json.loads(x) for x in (f.parent/'resources.jsonl').read_text().splitlines()];rounds=d['rounds'];vals=[x['wall_s'] for x in rounds]
  row=dict(engine='mjlab/MJWarp',version='3.11.0',environments=rounds[0]['num_envs'],mean_wall_s=statistics.mean(vals),stdev_wall_s=statistics.stdev(vals) if len(vals)>1 else 0,aggregate_rtf=statistics.mean(x['aggregate_rtf'] for x in rounds),startup_and_warmup_s=d['model_load_s']+d['gpu_setup_and_compile_s']+d['warmup_s']+d.get('contact_setup_s',0),peak_ram_mib=max(x['rss_mib'] for x in samples),peak_gpu_system_mib=max(float(x['gpu'].split(',')[0]) for x in samples if 'gpu' in x),invalid_repeats=sum(not x['finite'] or x['overflowed'] for x in rounds),raw_path=str(f))
  rows.append(row);raw.append(dict(summary=row,repeats=rounds))
 with (a.output/'comparison.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
 (a.output/'comparison.json').write_text(json.dumps(raw,indent=2));head=['엔진 / 버전','환경 수','한 회 평균 ± 표준편차 (초)','전체 처리량 (sim-s/s)','최대 RAM (GiB)','최대 시스템 GPU 메모리 (GiB)']
 table=[[r['engine']+' '+r['version'],str(r['environments']),f"{r['mean_wall_s']:.3f} ± {r['stdev_wall_s']:.3f}",f"{r['aggregate_rtf']:.2f}",f"{r['peak_ram_mib']/1024:.2f}",f"{r['peak_gpu_system_mib']/1024:.2f}"] for r in rows]
 intro='모든 환경에서 동일한 49번 관절 명령 16.5초를 재생하고 3회 반복. 로딩·컴파일·워밍업·저장은 평균 실행 시간에서 제외하여 별도로 보존. CPU는 프로세스별 환경 1개, GPU는 mjlab Simulation의 여러 world. 전체 처리량 = 환경 수 × 16.5 / 한 회 시간. 3.13 CPU와 3.11 GPU는 버전이 다르므로 3.11 CPU도 별도로 비교한다. 이 시험은 처리량 기준선이며 서로 다른 경로의 수확 성공 데이터셋이 아니다.'
 (a.output/'comparison.md').write_text('# 로봇 포함 CPU / mjlab 비교\n\n'+intro+'\n\n|'+'|'.join(head)+'|\n|'+'|'.join(['---']*len(head))+'|\n'+'\n'.join('|'+'|'.join(row)+'|' for row in table)+'\n')
 css='body{font:16px system-ui;background:#16202b;color:#edf2f7;margin:36px}table{border-collapse:collapse;width:100%}td,th{padding:10px;border-bottom:1px solid #455468;text-align:right}td:first-child,th:first-child{text-align:left}p{line-height:1.7;max-width:1100px}a{color:#81c9fa}'
 doc='<html lang="ko"><meta charset="utf-8"><title>로봇 병렬 성능 비교</title><style>'+css+'</style><h1>로봇 포함 CPU / mjlab 비교</h1><p>'+intro+'</p><p><a href="comparison.csv">CSV</a> · <a href="comparison.json">상세 시간 JSON</a></p><table><tr>'+''.join('<th>'+x+'</th>' for x in head)+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(x)+'</td>' for x in row)+'</tr>' for row in table)+'</table></html>'
 (a.output/'index.html').write_text(doc)
if __name__=='__main__':main()
