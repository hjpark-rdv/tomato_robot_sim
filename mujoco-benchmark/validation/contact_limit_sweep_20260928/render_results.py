import sys,json,gzip
from pathlib import Path
sys.path.insert(0,'/root/farmily_tomato/mujoco-benchmark/scripts')
from render_contact_diagnostic import render
O=Path(__file__).parent
jobs={j['name']:j for j in json.loads((O/'sweep_manifest.json').read_text())['jobs']}
for name in sys.argv[1:]:
 j=jobs[name];d=O/name;t=json.loads((d/'target_contact_trial.json').read_text());scope=json.loads((d/'contact_scope.json').read_text())['scope']
 samples=O/f'render_samples_{name}.json';samples.write_bytes(gzip.decompress((d/'authorized_contact_samples.json.gz').read_bytes()))
 colors={scope['fruit']:[1,.3,0,1],**{n:[1,.8,0,1] for n in scope['pedicels']},**{n:[0,.8,1,1] for n in scope['rear_wires']},'glb_col_TRUSS_Rachis_05':[1,.1,.7,1]}
 for pair in json.loads((d/'summary.json').read_text())['pair_peaks']:
  if pair['role']=='forbidden_contact' and pair['max_single_normal_N']>0:
   for geom in pair['geoms']:
    if 'Rachis' in geom:colors[geom]=[1,.1,.7,1]
 stop=','.join(t.get('stop_reasons',[])) or 'motion_complete'
 render(Path(j['run']),d/'trial_states.npz',O/f'videos/{name}.mp4',samples,label=f'{name} | END {t["simulated_s"]:.3f}s: {stop}',candidate=j['candidate'],highlights=colors,right_caption='ACTUAL saved SIM; see stop reason; no harvest claim')
