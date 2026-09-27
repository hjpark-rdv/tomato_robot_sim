"""User-authorized finite threshold comparison. No new geometry or physics settings."""
import sys,json,subprocess,hashlib,time
from pathlib import Path
REPO=Path('/root/farmily_tomato');O=Path(__file__).parent
sys.path.insert(0,str(REPO/'mujoco-benchmark/scripts'))
from summarize_target_contact_trial import summarize
PY=REPO/'mujoco-benchmark/.venv/bin/python'
RUN=Path('/root/docker_share/mujoko_debugging_data/20260927_hook_seating_server/robot_under')
SLOW=Path('/root/docker_share/mujoko_debugging_data/20260928_target_fruit_contact_server/slow_run')
POLICY=RUN.parent/'diagnostic_policy.json'
jobs=[]
for speed,root in [('original',RUN),('slow18',SLOW)]:
 for force,disp in [(2,.005),(1,.010),(2,.010)]:
  jobs.append(dict(name=f'{speed}_00_f{force}_d{round(disp*1000)}',candidate='seating_00_under',run=str(root),force=force,displacement_m=disp,speed=speed))
for cid in ['00','03','04','05','06']:
 jobs.append(dict(name=f'original_{cid}_f5_d20',candidate=f'seating_{cid}_under',run=str(RUN),force=5,displacement_m=.020,speed='original'))
jobs.append(dict(name='slow18_00_f5_d20',candidate='seating_00_under',run=str(SLOW),force=5,displacement_m=.020,speed='slow18'))
manifest=O/'sweep_manifest.json'
if manifest.exists():raise RuntimeError('Preserve existing run; no resume or overwrite')
manifest.write_text(json.dumps(dict(jobs=jobs,max_physics_executions=12,base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),training_eligible=False,hook_success=None),indent=2)+'\n')
assert len(jobs)==12
records=[]
for j in jobs:
 dest=O/j['name'];cmd=[str(PY),str(REPO/'mujoco-benchmark/scripts/target_fruit_contact_trial.py'),j['run'],'--candidate',j['candidate'],'--base-policy',str(POLICY),'--output',str(dest),'--execute','--max-target-force-n',str(j['force']),'--max-target-displacement-m',str(j['displacement_m'])]
 print('START',j['name'],flush=True);start=time.monotonic()
 with (O/(j['name']+'.log')).open('x') as log:p=subprocess.run(cmd,cwd=REPO,stdout=log,stderr=subprocess.STDOUT)
 rec=dict(**j,exit_code=p.returncode,wall_s=time.monotonic()-start,command=cmd)
 if p.returncode not in (0,2):
  records.append(rec);(O/'progress.json').write_text(json.dumps(records,indent=2));raise RuntimeError(rec)
 s=summarize(dest);(dest/'summary.json').write_text(json.dumps(s,indent=2,allow_nan=False)+'\n')
 rec['result']=s['trial'];records.append(rec)
 (O/'progress.json').write_text(json.dumps(records,indent=2,allow_nan=False)+'\n')
 print('END',j['name'],s['trial'].get('stop_reasons'),s['trial'].get('simulated_s'),flush=True)
print('DONE',len(records),flush=True)
