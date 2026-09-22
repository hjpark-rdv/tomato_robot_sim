"""Persistent CPU-process weak-scaling benchmark with raw timings/resources."""
import os
for k in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'):os.environ[k]='1'
import argparse,datetime,json,time,threading,subprocess,platform,hashlib,csv,queue
import multiprocessing as mp
from pathlib import Path
import psutil
from robot_engine import DEFAULT_MODEL,DEFAULT_TRACE,HOME

def worker(index,barrier,out,model,trace,hz,seconds,repeats):
 try:
  import mujoco
  from robot_engine import RobotEngine
  start=time.perf_counter();e=RobotEngine(model,trace,hz,reference=Path(model).parent/'reference.json' if (Path(model).parent/'reference.json').exists() else None);load=time.perf_counter()-start
  t=time.perf_counter();e.rollout(seconds=min(2.,seconds));warm=time.perf_counter()-t
  out.put(dict(kind='ready',worker=index,pid=os.getpid(),load_s=load,warmup_s=warm,rss_mib=psutil.Process().memory_info().rss/2**20,mujoco=mujoco.__version__,fk_error_m=e.fk_error_m))
  for repeat in range(repeats):
   wait=time.perf_counter();barrier.wait(timeout=600);wait=time.perf_counter()-wait
   start=time.perf_counter();r=e.rollout(seconds=seconds);end=time.perf_counter()
   out.put(dict(kind='result',worker=index,repeat=repeat,start_monotonic=start,end_monotonic=end,utc_end=datetime.datetime.now(datetime.timezone.utc).isoformat(),barrier_wait_s=wait,**r))
  barrier.wait(timeout=600)
 except BaseException:
  import traceback
  out.put(dict(kind='error',worker=index,error=traceback.format_exc()));barrier.abort()

def sample_resources(stop,rows,processes):
 parent=psutil.Process();parent.cpu_percent();psutil.cpu_percent()
 while not stop.is_set():
  row=dict(monotonic_s=time.perf_counter(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),system_cpu_percent=psutil.cpu_percent(),available_ram_mib=psutil.virtual_memory().available/2**20,workers=[])
  for p in processes:
   try:
    q=psutil.Process(p.pid);row['workers'].append(dict(pid=p.pid,rss_mib=q.memory_info().rss/2**20,cpu_user_s=q.cpu_times().user,cpu_system_s=q.cpu_times().system))
   except psutil.Error:pass
  try:row['gpu']=subprocess.check_output(['nvidia-smi','--query-gpu=memory.used,utilization.gpu','--format=csv,noheader,nounits'],text=True,timeout=2).strip()
  except Exception:pass
  rows.append(row);stop.wait(1)

def run_scale(n,a,root):
 ctx=mp.get_context('spawn');barrier=ctx.Barrier(n+1);q=ctx.Queue();procs=[ctx.Process(target=worker,args=(i,barrier,q,str(a.model),str(a.trace),a.hz,a.seconds,a.repeats)) for i in range(n)]
 start=time.perf_counter();ready=[];results=[];resources=[];stop=threading.Event();monitor=None
 try:
  for p in procs:p.start()
  monitor=threading.Thread(target=sample_resources,args=(stop,resources,procs),daemon=True);monitor.start()
  for _ in procs:
   item=q.get(timeout=600)
   if item['kind']!='ready':raise RuntimeError(item)
   ready.append(item)
  startup=time.perf_counter()-start;print('[CPU 준비]',n,'프로세스',round(startup,2),'초',flush=True)
  rounds=[]
  for repeat in range(a.repeats):
   t=time.perf_counter();barrier.wait(timeout=600)
   for _ in procs:
    item=q.get(timeout=600)
    if item['kind']!='result':raise RuntimeError(item)
    results.append(item)
   batch=results[-n:];span=max(r['end_monotonic'] for r in batch)-min(r['start_monotonic'] for r in batch)
   rounds.append(dict(repeat=repeat,wall_s=span,collect_wall_s=time.perf_counter()-t,aggregate_simulated_s=sum(r['simulated_s'] for r in batch),aggregate_rtf=sum(r['simulated_s'] for r in batch)/span,episodes_per_s=n/span,unstable_count=sum(r['unstable'] for r in batch)))
   print('[CPU 측정]',n,'환경',repeat+1,'회',round(span,3),'초, 처리량',round(rounds[-1]['aggregate_rtf'],2),'sim-s/s',flush=True)
  barrier.wait(timeout=600)
  for p in procs:p.join(timeout=30)
  report=dict(workers=n,startup_and_warmup_wall_s=startup,process_total_wall_s=time.perf_counter()-start,ready=ready,rounds=rounds,results=results,exit_codes=[p.exitcode for p in procs])
 except BaseException as e:
  report=dict(workers=n,error=str(e),ready=ready,results=results)
  raise
 finally:
  stop.set()
  if monitor:monitor.join(timeout=4)
  for p in procs:
   if p.is_alive():p.terminate();p.join(timeout=10)
  t=time.perf_counter();folder=root/f'cpu_{n:03d}';folder.mkdir()
  with (folder/'resources.jsonl').open('w') as f:
   for r in resources:f.write(json.dumps(r)+'\n')
  (folder/'timings.json').write_text(json.dumps(report,indent=2));(folder/'save_timing.json').write_text(json.dumps({'save_wall_s':time.perf_counter()-t}))
 return report

def main():
 p=argparse.ArgumentParser();p.add_argument('--workers',default='1,2,4,8,16,24,32,48');p.add_argument('--repeats',type=int,default=3);p.add_argument('--seconds',type=float,default=16.5);p.add_argument('--hz',type=int,default=120);p.add_argument('--model',type=Path,default=DEFAULT_MODEL);p.add_argument('--trace',type=Path,default=DEFAULT_TRACE);p.add_argument('--output',type=Path);a=p.parse_args();root=a.output or HOME/'outputs'/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_robot_cpu_scaling');root.mkdir(parents=True,exist_ok=False)
 manifest=dict(arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),cpu=platform.processor(),logical_cpus=psutil.cpu_count(),physical_cpus=psutil.cpu_count(logical=False),ram_bytes=psutil.virtual_memory().total,git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=HOME,text=True).strip(),model_sha256=hashlib.sha256(a.model.read_bytes()).hexdigest(),trace_sha256=hashlib.sha256(a.trace.read_bytes()).hexdigest(),scope='weak scaling: one identical full command replay per worker per repetition; no rendering, no IK, no harvest-success inference',source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')})
 # Freeze model, command trace and reference once per run, not per worker.
 import shutil
 assets=root/'replay_assets';assets.mkdir()
 if a.model.suffix!='.mjb':raise ValueError('Replay archive requires compiled .mjb model')
 for src,name in ((a.model,'model.mjb'),(a.trace,'trace.json'),(HOME/'assets/reference/reference.json','reference.json')):
  shutil.copy2(src,assets/name)
 manifest['reference_sha256']=hashlib.sha256((assets/'reference.json').read_bytes()).hexdigest()
 manifest['replay_archive']='replay_assets'
 shutil.copytree(Path(__file__).parent,assets/'scripts',ignore=shutil.ignore_patterns('__pycache__'))
 a.model=assets/'model.mjb';a.trace=assets/'trace.json'
 (root/'manifest.json').write_text(json.dumps(manifest,indent=2));print('[CPU 결과 폴더]',root,flush=True);reports=[]
 for n in map(int,a.workers.split(',')):
  if not 1<=n<=128:raise ValueError('workers must be 1..128')
  # Conservative 1.5 GiB per resident worker plus 8 GiB host reserve.
  if psutil.virtual_memory().available < (n*1.5+8)*2**30:print('[CPU 중단] 가용 메모리 예산 부족',n,flush=True);break
  reports.append(run_scale(n,a,root));(root/'summary.json').write_text(json.dumps(reports,indent=2))
 rows=[{'workers':r['workers'],**v} for r in reports for v in r['rounds']]
 with (root/'rounds.csv').open('w') as f:
  if rows:
   w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
 from cpu_report import generate
 print('[CPU HTML 보고서]',generate(root),flush=True)
 print('[CPU 완료]',root,flush=True)
if __name__=='__main__':main()
