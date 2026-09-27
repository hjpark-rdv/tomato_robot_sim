"""Read-only native body mapping, scope, and stage accounting for server evidence."""
import collections,json,sys,hashlib
from pathlib import Path
import mujoco as mj
root=Path(__file__).resolve().parent
repo=Path('/root/farmily_tomato')
sys.path.insert(0,str(repo/'mujoco-benchmark/scripts'))
from summarize_motion_search import summarize
models={}
for name in ('smoke_v2','pilot','side_repair'):
 campaign=root/name
 assert (campaign/'video_queue.json').exists(),f'{name} still running'
 summary=summarize(campaign)
 reports=json.loads((campaign/'results.json').read_text())['reports']
 cases={(r['case']['scene_id'],r['case']['target']):r for r in reports}
 mapped=[];phase=collections.defaultdict(collections.Counter);obstacles=collections.defaultdict(collections.Counter)
 for row in summary['candidates']:
  v=row.get('first_violation')
  if not v:continue
  case=cases[(row['scene'],row['target'])]['case'];sha=case['hashes']['replay_assets/model.mjb']
  if sha not in models:
   p=Path('/dev/shm/tomato_multifamily_20260928')/(sha+'.mjb')
   if not p.exists():p=Path(case['source_run'])/'replay_assets/model.mjb'
   assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
   models[sha]=mj.MjModel.from_binary_path(str(p))
  m=models[sha];details={}
  for label,key in [('robot','robot_geom'),('obstacle','environment_geom')]:
   g=m.geom(v[key]).id;b=int(m.geom_bodyid[g]);parents=[];current=b
   while current:parents.append(m.body(current).name);current=int(m.body_parentid[current])
   details[label]=dict(geom=v[key],body=m.body(b).name,ancestors=parents,contype=int(m.geom_contype[g]),conaffinity=int(m.geom_conaffinity[g]))
  scope=json.loads((campaign/'cases'/row['scene']/row['target']/'trials'/row['candidate_id']/'contact_scope.json').read_text())['scope']
  details['robot']['wire_member']=v['robot_geom'] in scope['wires']
  details['obstacle']['target_authorized_geometry']=v['environment_geom'] in [scope['fruit'],*scope['pedicels'],*scope.get('rachis',[])]
  b=details['obstacle']['body'];gn=v['environment_geom']
  category='other'
  if 'gutter' in gn.lower():category='gutter'
  elif 'Tomato' in b:category='target_fruit' if gn==scope['fruit'] else 'other_fruit'
  elif 'Pedicel' in b or 'Attachment' in b:category='mapped_target_pedicel' if gn in scope['pedicels'] else 'non_authorized_pedicel'
  elif 'Rachis' in b:category='own_rachis' if gn in scope.get('rachis',[]) else 'other_rachis'
  elif 'Peduncle' in b:category='peduncle'
  elif 'MAIN' in b.upper() or 'STEM' in b.upper():category='main_stem'
  phases='+'.join(v['phases']);phase[row['family']][phases]+=1;obstacles[row['family']][category]+=1
  mapped.append(dict(scene=row['scene'],target=row['target'],candidate=row['candidate_id'],family=row['family'],phase=phases,time_s=v['time_s'],category=category,**details))
 (root/(name+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
 (root/(name+'_blockers.json')).write_text(json.dumps(dict(first_blocked_phase=dict(phase),first_obstacle_category=dict(obstacles),rows=mapped,
    notes='First violation per complete audit; later collisions remain in raw preflight. Category uses actual native body ancestry; names of Attachment and Tomato need not have equal numeric suffix.'),indent=2)+'\n')
print('Completed read-only summaries/body mapping')
