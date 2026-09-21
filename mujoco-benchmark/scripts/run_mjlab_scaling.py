"""Isolated sequential GPU scaling trials; preserve failed runs and startup times."""
import argparse,subprocess,time,datetime,json,sys,os
from pathlib import Path
HOME=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--envs',default='1,8,32,128,256,512,1024');p.add_argument('--repeats',type=int,default=3);p.add_argument('--seconds',type=float,default=16.5);p.add_argument('--output',type=Path);a=p.parse_args();root=a.output or HOME/'outputs'/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_mjlab_scaling');root.mkdir(parents=True,exist_ok=False);print('[mjlab 비교 폴더]',root,flush=True);results=[]
 for n in map(int,a.envs.split(',')):
  folder=root/f'gpu_{n:04}';cmd=[sys.executable,'-u',str(HOME/'scripts/benchmark_mjlab_robot.py'),'--num-envs',str(n),'--seconds',str(a.seconds),'--repeats',str(a.repeats),'--output',str(folder)];t=time.perf_counter();print('[mjlab 시작]',n,flush=True)
  with (root/f'gpu_{n:04}.log').open('w') as f:
   process=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
   try:code=process.wait(timeout=1800)
   except BaseException:
    import signal
    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=30);raise
  result=dict(num_envs=n,exit_code=code,process_wall_s=time.perf_counter()-t,command=cmd);(root/f'gpu_{n:04}_process.json').write_text(json.dumps(result,indent=2));results.append(result);(root/'processes.json').write_text(json.dumps(results,indent=2));print('[mjlab 종료]',n,'코드',code,'전체',round(result['process_wall_s'],2),'초',flush=True)
  if code:print('[mjlab 중단] 실패 로그 보존; 잘못된 처리량을 보고하지 않음',flush=True);break
if __name__=='__main__':main()
