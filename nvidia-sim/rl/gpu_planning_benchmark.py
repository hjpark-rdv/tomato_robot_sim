"""Compare live-exported CPU planner inputs with serial and multiprocess paths.

No simulator is launched. Commands and rejection reasons must match exactly.
"""
import argparse
import json
from pathlib import Path
import pickle
import time
import numpy as np
from dataset_design import candidates, write_json
from dataset_motion import plan
from gpu_planning import restore_model, PlanningService


def compare_plans(a,b):
    pa,fa=a;pb,fb=b
    if fa!=fb:return False
    if (pa is None)!=(pb is None):return False
    if pa is None:return True
    for key in ('commands','direction','prehook','target_center','target_neck'):
        if not np.array_equal(pa[key],pb[key]):return False
    return (pa['phases']==pb['phases'] and pa['waypoints']==pb['waypoints'] and
            np.array_equal(pa['orientation'].as_quat(),pb['orientation'].as_quat()))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--candidates',type=int,default=32)
    parser.add_argument('--workers',type=int,nargs='+',default=[4,8])
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--goal',choices=['rise','pull'],default='rise')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    with args.model.open('rb') as stream:model=pickle.load(stream)
    env,kin,checker=restore_model(model)
    proposals=[dict(p,goal=args.goal) for p in candidates(args.candidates,args.seed)]
    began=time.monotonic();serial=[]
    for p in proposals:
        serial.append(plan(env,kin,checker,p))
        if len(serial)%8==0:print('[PLAN BENCH] serial',len(serial),flush=True)
    elapsed=time.monotonic()-began
    summary=dict(candidates=len(proposals),serial_wall_s=elapsed,model=str(args.model),results=[])
    for workers in args.workers:
        began=time.monotonic();service=PlanningService(args.output/f'workers{workers}',model,proposals,workers)
        try:
            checks=[]
            for p,expected in zip(proposals,serial):
                actual,_=service.take(p['candidate_id'],env)
                checks.append(dict(candidate_id=p['candidate_id'],passed=compare_plans(expected,(actual['planned'],actual['preflight']))))
            # Includes child startup, serialization, IPC and child shutdown.
            if service.process.wait(timeout=30)!=0:raise RuntimeError('Planning service failed')
            wall=time.monotonic()-began
        finally:service.close()
        passed=all(v['passed'] for v in checks)
        summary['results'].append(dict(workers=workers,wall_s=wall,passed=passed,
            speedup=elapsed/wall if passed else None,checks=checks))
        write_json(args.output/'summary.json',summary)
        print('[PLAN BENCH]',workers,'workers',round(wall,3),'seconds; identical:',passed,flush=True)
    if not all(r['passed'] for r in summary['results']):raise SystemExit('Parallel planner differed from serial')


if __name__=='__main__':main()
