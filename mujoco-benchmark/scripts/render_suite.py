"""Representative recorded-state videos, with explicit original/optimized labels."""
import argparse,json,subprocess,sys
from pathlib import Path
HOME=Path(__file__).resolve().parents[1]
def render(root):
    root=Path(root);jobs=[]
    for engine in ('isaac','mujoco'):
        runs=list(root.glob(f'{engine}_120hz_*_implicitfast/summary.json'))
        if runs:
            f=max(runs,key=lambda f:json.loads(f.read_text())['count'])
            jobs.append((f.parent,'fixture_stem_push,fixture_fruit_collision,fixture_pedicel_contact,fixture_entry_lift,candidate_00049',1))
    for folder,ids,speed in [('mujoco_240hz_6_implicitfast','fixture_entry_lift',1),('mujoco_240hz_6_implicitfast','fixture_entry_lift',.5),('mujoco_60hz_6_implicitfast','fixture_stem_push',1),('mujoco_optimized_120hz_10','fixture_stem_push,fixture_entry_lift',1),('mujoco_optimized_60hz_6','fixture_stem_push',1)]:
        if (root/folder/'summary.json').exists():jobs.append((root/folder,ids,speed))
    for folder,ids,speed in jobs:
        cmd=[sys.executable,str(HOME/'scripts/render_results.py'),'--run',str(folder),'--ids',ids,'--speed',str(speed)]
        if 'optimized' in folder.name:cmd+=['--model',str(HOME/'models/plant_mujoco_optimized.xml')]
        subprocess.run(cmd,check=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();render(a.root)
