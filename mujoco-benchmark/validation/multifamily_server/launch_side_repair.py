"""Budget ledger + existing runner invocation after the original campaign finishes."""
import json,os,time
from pathlib import Path
root=Path('/root/docker_share/mujoko_debugging_data/20260928_multifamily_server')
start=time.monotonic()
while not (root/'pilot/video_queue.json').exists():
 if time.monotonic()-start>7200:raise TimeoutError('Original campaign not finished; no repair launched')
 time.sleep(5)
report=json.loads((root/'pilot/results.json').read_text())
source=json.loads((root/'assets/pilot_prep/cases.json').read_text())
lookup={(r['case']['scene_id'],r['case']['target']):r for r in report['reports']}
selected=[];ledger=[]
for case in source['cases']:
 old=lookup[(case['scene_id'],case['target'])]
 assert old['complete'] and old['source_unchanged'], 'Inspect original errors first'
 spent=old['physics_budget_charged'];proposals=len(old['records'])
 eligible=spent<=8 and proposals<=20
 side=[x for x in old['records'] if x['family']=='side_mouth']
 prep_blocks=sum(x.get('first_violation',{}).get('phases')==['preapproach'] for x in side if x.get('first_violation'))
 ledger.append(dict(scene=case['scene_id'],target=case['target'],original_physics=spent,
   original_proposals=proposals,side_preparation_blocks=prep_blocks,repair_proposals=4 if eligible else 0,
   physics_cap_with_repair=spent+4 if eligible else spent,
   reason='Same four Sobol parameter vectors; robot-facing mouth comparison' if eligible else 'No remaining declared budget'))
 if eligible:selected.append(case)
assert selected
manifest=root/'side_repair_cases.json';manifest.write_text(json.dumps(dict(cases=selected,repair_budget_ledger=ledger,training_eligible=False),indent=2))
original_videos=json.loads((root/'pilot/video_queue.json').read_text())['selected']
render_budget=max(0,min(2,5-len(original_videos)))
py='/root/farmily_tomato/mujoco-benchmark/.venv/bin/python'
cmd=[py,str(root/'monitor_command.py'),str(root/'side_repair'),py,'-u','mujoco-benchmark/scripts/run_motion_family_search.py','--manifest',str(manifest),'--output',str(root/'side_repair'),'--samples-per-family','4','--families','side_mouth','--robot-facing-mouth','--physics-per-target','4','--refine-parents','0','--case-workers','8','--planning-workers','2','--execute','--max-target-force-n','5','--max-target-displacement-m','0.020','--render-budget',str(render_budget),'--model-cache','/dev/shm/tomato_multifamily_20260928']
(root/'side_repair_launch.json').write_text(json.dumps(dict(command=cmd,code_commit='664f699',budget_ledger=ledger,original_campaign=str(root/'pilot')),indent=2))
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
os.execv(py,cmd)
