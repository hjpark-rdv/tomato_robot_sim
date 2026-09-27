"""Experiment launcher only: measure existing command's CPU and tree RSS."""
import json,sys,time,subprocess
from pathlib import Path
import psutil
out=Path(sys.argv[1]);cmd=sys.argv[2:];start=time.monotonic(); rows=[]
with out.with_suffix('.log').open('w') as log:
 p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
 while p.poll() is None:
  rss=0;cpu=0; count=0
  try: processes=[psutil.Process(p.pid),*psutil.Process(p.pid).children(recursive=True)]
  except psutil.Error:processes=[]
  for proc in processes:
   try:rss+=proc.memory_info().rss; t=proc.cpu_times();cpu+=t.user+t.system;count+=1
   except psutil.Error:pass
  rows.append(dict(wall_s=time.monotonic()-start,tree_rss_bytes=rss,live_process_cpu_s=cpu,processes=count))
  time.sleep(2)
 result=dict(command=cmd,returncode=p.returncode,wall_s=time.monotonic()-start,peak_sampled_tree_rss_bytes=max((x['tree_rss_bytes'] for x in rows),default=0),samples=rows,interval_s=2)
 out.with_suffix('.resources.json').write_text(json.dumps(result,indent=2))
 print(json.dumps({k:v for k,v in result.items() if k!='samples'}));sys.exit(p.returncode)
