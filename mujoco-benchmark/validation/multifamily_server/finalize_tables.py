import subprocess,time
from pathlib import Path
r=Path(__file__).resolve().parent;repo=Path('/root/farmily_tomato');py=str(repo/'mujoco-benchmark/.venv/bin/python')
start=time.monotonic()
while not (r/'side_repair.resources.json').exists():
 if time.monotonic()-start>7200:raise TimeoutError('Final campaign not finished')
 time.sleep(5)
subprocess.run([py,str(r/'analyze_campaigns.py')],check=True)
for n in ('pilot','side_repair'):
 subprocess.run(['/root/isaaclab_env/bin/python',str(repo/'mujoco-benchmark/scripts/verify_motion_search_fk.py'),str(r/n),'--output',str(r/(n+'_fk.json'))],check=True)
print('All final tables and FK evidence saved',flush=True)
