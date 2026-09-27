import json,os,sys,time
from pathlib import Path
root=Path('/root/docker_share/mujoko_debugging_data/20260928_multifamily_server')
manifest=root/'assets/pilot_prep/cases.json'
start=time.monotonic()
while not manifest.exists():
 if time.monotonic()-start>600:raise TimeoutError('Preparation did not finish; no partial campaign launched')
 time.sleep(2)
doc=json.loads(manifest.read_text());assert doc['generated_scenes']==6 and 0<len(doc['cases'])<=18
py='/root/farmily_tomato/mujoco-benchmark/.venv/bin/python'
cmd=[py,str(root/'monitor_command.py'),str(root/'pilot'),py,'-u','mujoco-benchmark/scripts/run_motion_family_search.py','--manifest',str(manifest),'--output',str(root/'pilot'),'--samples-per-family','4','--physics-per-target','12','--refine-parents','2','--case-workers','8','--planning-workers','2','--execute','--max-target-force-n','5','--max-target-displacement-m','0.020','--render-budget','5','--model-cache','/dev/shm/tomato_multifamily_20260928']
(root/'pilot_launch.json').write_text(json.dumps(dict(command=cmd,code_commit='65fb0e55cfa922f396c5f4d2f46bbe89ef1a637e',threads=1,requested_cases=18,prepared_cases=len(doc['cases']),source_scene_failures=doc['source_scene_failures']),indent=2))
for name in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'):os.environ[name]='1'
os.execv(py,cmd)
