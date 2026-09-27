"""Verify saved evidence, budget, durable model links, and full MP4 decode."""
from pathlib import Path
import collections,hashlib,json
import imageio.v2 as imageio
root=Path(__file__).resolve().parent
load=lambda p:json.loads(p.read_text())
cohorts={};links=[]
for n in ('smoke_v2','pilot','side_repair'):
 d=load(root/n/'results.json');assert d['reported_cases']==d['completed_cases']==d['expected_cases']
 rs=[x for c in d['reports'] for x in c['records']]
 assert all(c['source_unchanged'] for c in d['reports'])
 assert not any(c.get('error') for c in d['reports'])
 for c in d['reports']:
  run=root/n/'cases'/c['case']['scene_id']/c['case']['target']/'run'
  p=run/'replay_assets/model.mjb';resolved=p.resolve(strict=True)
  assert not str(resolved).startswith('/dev/shm'),resolved
  links.append(dict(cohort=n,scene=c['case']['scene_id'],target=c['case']['target'],durable_model=str(resolved)))
 cohorts[n]=dict(cases=len(d['reports']),records=len(rs),outcomes=dict(collections.Counter(r['outcome'] for r in rs)),physics=sum(r.get('physics_executed',False) for r in rs),sources_unchanged=True)
ledger=load(root/'side_repair_cases.json')['repair_budget_ledger'];assert all(x['physics_cap_with_repair']<=12 and x['original_proposals']+x['repair_proposals']<=24 for x in ledger)
videos=[]
for item in load(root/'video_manifest.json'):
 p=root/item['path'];reader=imageio.get_reader(p);frames=0;shape=None
 for frame in reader:frames+=1;shape=list(frame.shape)
 reader.close();meta=load(p.with_suffix('.json'));assert frames==meta['frames'],p
 assert meta['physics_executed_by_renderer'] is False
 if item['kind']=='nominal_only':assert 'NOMINAL' in meta['label'] and 'NO PHYSICS' in meta['right_caption']
 videos.append(dict(path=item['path'],kind=item['kind'],decoded_frames=frames,shape=shape,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
assert len(videos)==8
result=dict(cohorts=cohorts,durable_model_links=links,budget_verified=True,video_full_decode=videos,training_eligible=False,hook_success=None)
(root/'delivery_verification.json').write_text(json.dumps(result,indent=2)+'\n');print('Verified',cohorts,'videos',len(videos))
