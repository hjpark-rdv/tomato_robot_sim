"""Dedicated safe spawn entry point; deliberately never imports Isaac/Kit."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing
import os
from pathlib import Path
import pickle
import signal
import threading
import time

_MODEL = None


def initialize(path):
    import torch
    from gpu_planning import restore_model
    global _MODEL
    torch.set_num_threads(1)
    with open(path,'rb') as stream:_MODEL=restore_model(pickle.load(stream))


def compute(params):
    import os
    from dataset_motion import plan
    began=time.monotonic()
    planned,preflight=plan(*_MODEL,params)
    return dict(planned=planned,preflight=preflight,compute_wall_s=time.monotonic()-began,worker_pid=os.getpid())


def main():
    from gpu_planning import atomic_pickle
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',type=Path,required=True)
    parser.add_argument('--workers',type=int,required=True)
    parser.add_argument('--parent-pid',type=int,default=0)
    args=parser.parse_args()
    if args.workers<1:parser.error('workers must be positive')
    if args.parent_pid:
        # The service owns its session. Stop CPU children even when Kit is
        # terminated by its controller (SIGTERM does not run Python finally).
        def stop_service(signum, frame):
            signal.signal(signal.SIGTERM,signal.SIG_IGN)
            os.killpg(os.getpid(),signal.SIGTERM)
            raise SystemExit(128+signum)
        signal.signal(signal.SIGTERM,stop_service)
        def watch_parent():
            while os.getppid()==args.parent_pid:time.sleep(.5)
            os.kill(os.getpid(),signal.SIGTERM)
        threading.Thread(target=watch_parent,daemon=True).start()
    candidates=json.loads((args.folder/'candidates.json').read_text())
    began=time.monotonic();finished=0;cpu_work=0.;pids=set()
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('spawn'),
                             initializer=initialize,initargs=(args.folder/'model.pkl',)) as pool:
        futures={pool.submit(compute,param):param['candidate_id'] for param in candidates}
        for future in as_completed(futures):
            value=future.result();atomic_pickle(args.folder/(futures[future]+'.pkl'),value)
            finished+=1;cpu_work+=value['compute_wall_s'];pids.add(value['worker_pid'])
            status=dict(completed=finished,total=len(candidates),workers=args.workers,worker_pids=sorted(pids),
                        elapsed_wall_s=time.monotonic()-began,sum_worker_planning_s=cpu_work)
            temporary=args.folder/'progress.tmp';temporary.write_text(json.dumps(status,indent=2))
            temporary.replace(args.folder/'progress.json')
            print('[CPU PLAN]',finished,'/',len(candidates),round(status['elapsed_wall_s'],2),flush=True)
    status['complete']=True;status['elapsed_wall_s']=time.monotonic()-began
    (args.folder/'summary.json').write_text(json.dumps(status,indent=2)+'\n')


if __name__=='__main__':main()
