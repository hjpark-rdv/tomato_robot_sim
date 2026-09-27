"""Read stored commands to quantify shared approach prefixes across families."""
import collections,hashlib,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent;p=root/'pilot';groups=collections.defaultdict(list)
for f in p.glob('cases/*/*/run/candidates/*/plan.json'):
 d=json.loads(f.read_text());family=d['parameters']['family']
 if family=='side_mouth' or not d['preflight'].get('passed'):continue
 rs=json.loads(f.with_name('trace.json').read_text());prefix=[r for r in rs if r['phase'] in ('preapproach','approach')]
 params=d['parameters']['search_parameters'];key=(f.parts[-6],f.parts[-5],json.dumps(params,sort_keys=True))
 a=np.array([r['command'] for r in prefix]);groups[key].append(dict(family=family,candidate=d['candidate_id'],samples=len(prefix),command_sha256=hashlib.sha256(a.tobytes()).hexdigest()))
rows=[dict(scene=k[0],target=k[1],parameters=json.loads(k[2]),members=v,identical=len({r['command_sha256'] for r in v})==1) for k,v in groups.items()]
for row in rows:
 report=json.loads((p/'cases'/row['scene']/row['target']/'case_result.json').read_text())
 records={r['candidate_id']:r for r in report['records']}
 row['all_four_blocked_in_prefix']=all(set((records[m['candidate']].get('first_violation') or {}).get('phases',[])) & {'preapproach','approach'} for m in row['members'])
result=dict(prefix_blocked_groups=sum(r['all_four_blocked_in_prefix'] for r in rows),groups=len(rows),complete_four_family_groups=sum(len(r['members'])==4 for r in rows),identical_groups=sum(r['identical'] for r in rows),rows=rows,scope='Actual saved preapproach+approach joint command arrays; no physics. These four families diverge after this prefix.')
(root/'prefix_comparison.json').write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k!='rows'})
