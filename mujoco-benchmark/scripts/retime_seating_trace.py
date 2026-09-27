"""Slow saved seat segments without IK or geometry replanning (offline diagnostics only)."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np


def retime(rows, factor):
    if isinstance(factor,bool) or not isinstance(factor,int) or factor < 1:
        raise ValueError('Positive integer stretch required')
    if not rows or not any(r['phase']=='seat' for r in rows):
        raise ValueError('Need a seat trace')
    output=[copy.deepcopy(rows[0])]; indices=[0]
    for a,b in zip(rows,rows[1:]):
        n=factor if 'seat' in (a['phase'],b['phase']) else 1
        for i in range(1,n+1):
            row=copy.deepcopy(b if i==n else a)
            row['command']=(np.asarray(a['command'])*(1-i/n)+np.asarray(b['command'])*(i/n)).tolist()
            output.append(row)
        indices.append(len(output)-1)
    return output,indices


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('--candidate',required=True)
    p.add_argument('--factor',type=int,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=a.run.resolve();out=a.output.resolve()
    if out.exists() or root==out or root in out.parents:p.error('New output outside source run required')
    source=root/'candidates'/a.candidate
    plan=json.loads((source/'plan.json').read_text())
    if plan.get('parameters',{}).get('trajectory_mode')!='diagnostic_pose_waypoints_v1':
        p.error('Diagnostic pose traces only')
    rows=json.loads((source/'trace.json').read_text());new,indices=retime(rows,a.factor)
    # Share immutable large assets; never write through the symlink.
    out.mkdir(parents=True);(out/'replay_assets').symlink_to(root/'replay_assets',target_is_directory=True)
    shutil.copy2(root/'manifest.json',out/'manifest.json')
    folder=out/'candidates'/a.candidate;folder.mkdir(parents=True)
    plan.update(seconds=(len(new)-1)/60,training_eligible=False,hook_success=None)
    provenance=dict(source_run=str(root),candidate=a.candidate,factor=a.factor,
        source_trace_sha256=hashlib.sha256((source/'trace.json').read_bytes()).hexdigest(),
        source_plan_sha256=hashlib.sha256((source/'plan.json').read_bytes()).hexdigest(),
        original_command_indices=indices,source_rows=len(rows),retimed_rows=len(new),
        geometric_path='Same piecewise-linear joint path; integer subdivision of seat-incident intervals',
        training_eligible=False,hook_success=None)
    plan['retiming']=provenance
    (folder/'trace.json').write_text(json.dumps(new)+'\n')
    (folder/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    provenance['retimed_trace_sha256']=hashlib.sha256((folder/'trace.json').read_bytes()).hexdigest()
    (out/'retiming.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps({k:v for k,v in provenance.items() if k!='original_command_indices'},indent=2))


if __name__=='__main__':main()
