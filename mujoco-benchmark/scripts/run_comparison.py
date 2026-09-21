"""Sequential A/B runs; avoids simultaneous benchmark CPU contention."""
import argparse,subprocess,json,sys
from datetime import datetime
from pathlib import Path
HOME=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path);p.add_argument('--counts',default='1,10,100');p.add_argument('--sweep-count',type=int,default=6);p.add_argument('--skip-videos',action='store_true');a=p.parse_args()
    out=a.output or HOME/'outputs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_engine_comparison');out.mkdir(parents=True,exist_ok=False)
    jobs=[]
    for n in [int(v) for v in a.counts.split(',')]:
        for engine in ('isaac','mujoco'):jobs.append((engine,120,n,'implicitfast'))
    for hz in (60,240,480):jobs.append(('mujoco',hz,a.sweep_count,'implicitfast'))
    jobs.append(('mujoco',120,a.sweep_count,'Euler'))
    progress=[]
    for engine,hz,n,integrator in jobs:
        name=f'{engine}_{hz}hz_{n}_{integrator}';dest=out/name
        python='/root/isaaclab_env/bin/python' if engine=='isaac' else str(HOME/'.venv/bin/python')
        cmd=[python,'-u',str(HOME/'scripts'/f'benchmark_{engine}.py'),'--count',str(n),'--hz',str(hz),'--output',str(dest)]
        if engine=='mujoco':cmd+=['--integrator',integrator]
        print('[엔진 비교 시작]',name,flush=True)
        with (out/(name+'.log')).open('w') as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
        progress.append(dict(job=name,returncode=r.returncode));(out/'progress.json').write_text(json.dumps(progress,indent=2))
        if r.returncode:raise RuntimeError(f'{name} failed; inspect its log')
        print('[엔진 비교 완료]',name,flush=True)
    for script in ('behavior_metrics.py','summarize.py'):
        subprocess.run([str(HOME/'.venv/bin/python'),str(HOME/'scripts'/script),str(out)],check=True)
    if not a.skip_videos:
        subprocess.run([str(HOME/'.venv/bin/python'),str(HOME/'scripts/render_suite.py'),str(out)],check=True)
    subprocess.run([str(HOME/'.venv/bin/python'),str(HOME/'scripts/make_index.py'),str(out)],check=True)

if __name__=='__main__':main()
