"""저장한 CPU 실행을 선택하여 검증 후 화면으로 재연한다."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def digest(p):
 return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
 p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);p.add_argument('--workers',type=int,required=True);p.add_argument('--repeat',type=int,required=True,help='1부터 시작');p.add_argument('--env',type=int,required=True,help='0부터 시작');p.add_argument('--check-only',action='store_true');a=p.parse_args()
 root=a.run_dir.resolve();manifest=json.loads((root/'manifest.json').read_text());args=manifest['arguments']
 reports=json.loads((root/'summary.json').read_text());report=next(r for r in reports if r['workers']==a.workers)
 saved=next(r for r in report['results'] if r['repeat']==a.repeat-1 and r['worker']==a.env)
 assets=root/'replay_assets';model=assets/'model.mjb' if (assets/'model.mjb').exists() else Path(args['model']);trace=assets/'trace.json' if (assets/'trace.json').exists() else Path(args['trace'])
 for path,key in ((model,'model_sha256'),(trace,'trace_sha256')):
  if digest(path)!=manifest[key]:raise RuntimeError(f'원본과 달라 재연 중단: {path}')
 from robot_engine import RobotEngine,HOME
 import mujoco as mj
 version=next(r['mujoco'] for r in report['ready'] if r['worker']==a.env)
 if mj.__version__!=version:raise RuntimeError(f'MuJoCo 버전 불일치: 필요 {version}, 현재 {mj.__version__}')
 reference=assets/'reference.json' if (assets/'reference.json').exists() else HOME/'assets/reference/reference.json'
 if manifest.get('reference_sha256') and digest(reference)!=manifest['reference_sha256']:raise RuntimeError('reference 해시 불일치')
 for name in ('robot_engine.py','suite.py'):
  if digest(Path(__file__).with_name(name))!=manifest['source_sha256'].get(name):
   print('[재연 주의] 실행 코드 변경:',name,'; 아래 결과 비교 통과 여부 확인')
 e=RobotEngine(model,trace,args['hz'],reference=reference)
 e.rollout(seconds=min(2.,args['seconds']))
 actual=e.rollout(seconds=args['seconds'])
 import numpy as np
 keys=['hook_final_xyz','max_target_displacement_m','max_hook_contact_penetration_m','hook_contact_steps','unstable','warning_counts']
 comparison={k:bool(np.allclose(actual[k],saved[k],atol=1e-8,rtol=1e-6)) for k in keys}
 out=root/'replays'/f'cpu_{a.workers:03d}_repeat_{a.repeat:03d}_env_{a.env:03d}';out.mkdir(parents=True,exist_ok=True)
 (out/'verification.json').write_text(json.dumps({'matches':comparison,'original':saved,'replay':actual,'scope':'summary comparison; original per-step state not recorded'},indent=2))
 if not all(comparison.values()):raise RuntimeError(f'원본 결과와 불일치: {comparison}; {out}')
 print('[재연 검증 통과]',comparison,flush=True)
 if a.check_only:return
 import time
 import mujoco.viewer
 with mujoco.viewer.launch_passive(e.model,e.data) as view:
  e.reset()
  from view_camera import target_camera
  target_camera(e.model,e.data,view.cam)
  while view.is_running():
   e.reset()
   for t in np.arange(round(args['seconds']*args['hz']))/args['hz']:
    if not view.is_running():return
    start=time.perf_counter();e.data.ctrl[e.aids]=e.command(t);mj.mj_step(e.model,e.data);view.sync();time.sleep(max(0,e.model.opt.timestep-(time.perf_counter()-start)))

if __name__=='__main__':main()
