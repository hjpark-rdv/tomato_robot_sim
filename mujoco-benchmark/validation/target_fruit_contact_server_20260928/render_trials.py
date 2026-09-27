import sys,json,gzip
from pathlib import Path
sys.path.insert(0,'/root/farmily_tomato/mujoco-benchmark/scripts')
from render_contact_diagnostic import render
O=Path(__file__).parent
for cid in sys.argv[1:]:
 folder=O/f'trial_{cid}'
 result=json.loads((folder/'target_contact_trial.json').read_text())
 samples=O/f'video_samples_{cid}.json'
 samples.write_bytes(gzip.decompress((folder/'authorized_contact_samples.json.gz').read_bytes()))
 scope=json.loads((folder/'contact_scope.json').read_text())['scope']
 colors={scope['fruit']:[1,.3,0,1],**{n:[1,.8,0,1] for n in scope['pedicels']},**{n:[0,.8,1,1] for n in scope['rear_wires']},'glb_col_TRUSS_Rachis_05':[1,.1,.7,1]}
 run=O/'slow_run' if cid.startswith('slow_') else Path('/root/docker_share/mujoko_debugging_data/20260927_hook_seating_server/robot_under')
 name=cid.removeprefix('slow_')
 render(run,folder/'trial_states.npz',O/f'videos/{cid}.mp4',samples,label=f"{cid} | STOP @ {result['simulated_s']:.3f}s: {','.join(result['stop_reasons'])}",candidate=name,highlights=colors,right_caption='ACTUAL saved SIM state; stopped, no hook success')
