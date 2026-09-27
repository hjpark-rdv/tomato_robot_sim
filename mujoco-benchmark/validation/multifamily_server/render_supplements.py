"""Read-only supplement renderer: existing renderer, explicit nominal-only states."""
import sys,json,gzip
from pathlib import Path
import numpy as np
sys.path.insert(0,'/root/farmily_tomato/mujoco-benchmark/scripts')
import render_contact_diagnostic as renderer
from diagnose_contact_timing import load_engine

root=Path('/root/docker_share/mujoko_debugging_data/20260928_multifamily_server')
out=root/'smoke_videos';out.mkdir(exist_ok=True)
items=[dict(name='actual_executor_control',run='/root/docker_share/mujoko_debugging_data/20260927_hook_seating_server/robot_under',candidate='seating_00_under',trial=str(root/'executor_control'),kind='recorded_physics'),
 dict(name='nominal_side_mouth_blocked',run=str(root/'smoke_v2/cases/gutter02/Tomato_02/run'),candidate='mf_20260928_00001',kind='nominal_only'),
 dict(name='nominal_flank_left_blocked',run=str(root/'smoke_v2/cases/two_trusses/truss_01__Tomato_01/run'),candidate='mf_20260928_00002',kind='nominal_only')]
for item in items:
 run=Path(item['run']);cid=item['candidate'];dest=out/(item['name']+'.mp4')
 if item['kind']=='recorded_physics':
  trial=Path(item['trial']);states=trial/'trial_states.npz';samples=out/(item['name']+'.samples.json');samples.write_bytes(gzip.decompress((trial/'authorized_contact_samples.json.gz').read_bytes()))
  caption='ACTUAL stored qpos (executor control)';label='EXISTING CONTROL / completed, no retention'
 else:
  engine,trace,plan=load_engine(run,cid);times=np.unique(np.r_[np.arange(0,plan['seconds'],1/15),plan['seconds']]);qpos=np.tile(engine.data.qpos,(len(times),1))
  for i,t in enumerate(times):qpos[i,engine.qids]=engine.command(float(t))
  states=out/(item['name']+'.nominal_states.npz');np.savez_compressed(states,times_s=times,qpos=qpos);samples=None
  audit=json.loads((run.parent/'trials'/cid/'environment_preflight.json').read_text());hit=audit['first_violation'];item['first_violation']=hit
  caption='NOMINAL ONLY; blocked, NO PHYSICS';label=f"NOMINAL / {item['name']} / hit {hit['environment_geom']} @ {hit['time_s']:.2f}s"
  del engine,trace,qpos
 renderer.render(run,states,dest,samples=samples,label=label,candidate=cid,right_caption=caption,speed=3)
 item.update(video=str(dest),states=str(states),physics_executed_by_renderer=False)
(out/'manifest.json').write_text(json.dumps(items,indent=2))
