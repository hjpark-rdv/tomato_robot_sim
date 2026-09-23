"""학습에서 제외한 토마토에 대한 확정 추천의 서로 다른 경로를 새 물리로 검증."""
import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,json,shutil,sys,time,multiprocessing,hashlib,subprocess
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,ThreadPoolExecutor,as_completed
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))

def execute_selected(path,candidate_id):
 """Run the existing MuJoCo executor in its version-matched physics environment."""
 scripts=Path(__file__).resolve().parents[1]/'scripts'
 python=Path(__file__).resolve().parents[1]/'.venv/bin/python'
 code=('import sys;sys.path.insert(0,sys.argv[3]);'
       'from pathlib import Path;from candidate_experiment import initialize,execute,report;'
       "root=Path(sys.argv[1]);initialize(root);report(root,[execute({'candidate_id':sys.argv[2]})])")
 with (Path(path)/'execution.log').open('w') as log:
  subprocess.run([str(python),'-c',code,str(path),candidate_id,str(scripts)],
                 stdout=log,stderr=subprocess.STDOUT,check=True)
 return json.loads((Path(path)/'candidates'/candidate_id/'result.json').read_text())

def test_random_scenes(training,collection,workers,output=None):
 """Score all nine saved test views, then run one selected plan per fruit afresh."""
 from features import extract
 from predict_multi import score_candidates
 import torch
 training=Path(training).resolve();collection=Path(collection).resolve()
 tm=json.loads((training/'manifest.json').read_text());cm=json.loads((collection/'collection.json').read_text())
 if tm.get('schema')!='random_glb_training_v1' or cm.get('schema')!='random_glb_collection_v1' or not cm.get('plan_only'):
  raise ValueError('Expected scene-held-out training and a fresh plan-only collection')
 if cm.get('status')!='complete':raise ValueError('Fresh random-scene collection is incomplete')
 if collection==Path(tm['collection']).resolve() or cm['seed']==json.loads((Path(tm['collection'])/'collection.json').read_text())['seed']:
  raise ValueError('Test collection must be generated independently with another seed')
 out=Path(output).resolve() if output else training/'fresh_scene_physics'
 out.mkdir(parents=True,exist_ok=False)
 name=tm['selected_model'];seed=tm['selected_seed'];ck=torch.load(training/f'{name}_seed{seed}'/'model.pt',map_location='cpu',weights_only=False)
 decisions=[];pending=[]
 for item in sorted(cm['targets'],key=lambda r:(r['scene'],r['target'])):
  scene=item['scene'];target=item['target'];source=collection/scene/'targets'/target;obsroot=source/'observations';physics=source/'physics'
  metadata=json.loads((obsroot/'dataset.json').read_text())
  if metadata.get('pose_label_actions',0)==0:
   decisions.append(dict(scene=scene,target=target,status='no_planned_candidate'));continue
  rows=[json.loads(line) for line in (obsroot/'observations.jsonl').read_text().splitlines()]
  valid=[row for row in rows if row['target']['input_usable'] and all(row['crops'][k]['available'] for k in ('local','context'))]
  if not valid:
   decisions.append(dict(scene=scene,target=target,status='no_usable_observation'));continue
  featureout=out/'features'/scene/target;featureout.mkdir(parents=True)
  features=extract(obsroot,featureout,'cuda' if torch.cuda.is_available() else 'cpu',rows=valid,backbone_store=out/'backbones') if name!='action_only' else None
  votes={}
  for index,row in enumerate(valid):
   with np.load(obsroot/'observations'/row['observation_id']/'actions_camera_v2.npz') as z:
    ids=z['candidate_ids'].tolist();action=z['features'].copy();gravity=z['gravity_direction_camera'].copy()
   observation=np.zeros((0,),dtype='float32') if name=='action_only' else np.r_[features[name.split('_')[0]][index],features['depth'][index],features['crop_intrinsics'][index]]
   scores=score_candidates(ck,action,gravity,observation)
   for cid,score in zip(ids,scores):votes.setdefault(cid,[]).append(float(score))
  if not votes:
   decisions.append(dict(scene=scene,target=target,status='no_planned_candidate'));continue
  choice=max(sorted(votes),key=lambda cid:np.mean(votes[cid]));mean=float(np.mean(votes[choice]))
  decision=dict(scene=scene,target=target,status='selected',candidate_id=choice,entry_score=mean,
                views=len(valid),selection_rule='mean score over usable saved RGB-D views; no outcomes read')
  decisions.append(decision)
  dest=out/scene/target/'physics';(dest/'replay_assets').parent.mkdir(parents=True,exist_ok=True)
  def copy_asset(source,destination):
   if Path(source).name=='model.mjb':
    try:os.link(source,destination);return destination
    except OSError:pass
   return shutil.copy2(source,destination)
  shutil.copytree(physics/'replay_assets',dest/'replay_assets',copy_function=copy_asset);(dest/'candidates'/choice).mkdir(parents=True)
  for filename in ('plan.json','trace.json'):
   shutil.copy2(physics/'candidates'/choice/filename,dest/'candidates'/choice/filename)
  manifest=json.loads((physics/'manifest.json').read_text())
  manifest.update(count=1,plan_only=False,source_run=str(physics),training_run=str(training),
                  selection=decision,scope='one fresh physics rollout of the frozen recommended joint plan')
  (dest/'manifest.json').write_text(json.dumps(manifest,indent=2))
  pending.append((str(dest),choice,decision))
 # Freeze all decisions before any fresh physical outcome is observed.
 (out/'recommendations.json').write_text(json.dumps(decisions,indent=2))
 outcomes=[]
 if pending:
  with ThreadPoolExecutor(max_workers=min(workers,len(pending))) as pool:
   futures={pool.submit(execute_selected,path,cid):decision for path,cid,decision in pending}
   for future in as_completed(futures):
    decision=futures[future];result=future.result()
    outcomes.append(dict(**decision,result=result['result'],center_entered=result.get('center_entered'),
                         max_displacement_m=result.get('target_center_max_displacement_m'),
                         physics_valid=result.get('metrics',{}).get('glb_physics_valid'),
                         physics_run=str(out/decision['scene']/decision['target']/'physics')))
    print('[새 장면 물리]',decision['scene'],decision['target'],result['result'],flush=True)
 summary=dict(schema='fresh_random_glb_scene_physics_v1',training=str(training),collection=str(collection),
              selected_model=name,selected_seed=seed,targets_in_collection=len(cm['targets']),
              selected=len(pending),fresh_physics_runs=len(outcomes),
              skipped_no_planned_candidate=sum(r['status']=='no_planned_candidate' for r in decisions),
              skipped_no_usable_observation=sum(r['status']=='no_usable_observation' for r in decisions),
              center_entries=sum(r['result']=='partial_center_entry' for r in outcomes),
              invalid_physics=sum(r['result']=='invalid_physics' for r in outcomes),
              hook_success=None,results=sorted(outcomes,key=lambda r:(r['scene'],r['target'])),
              note='one independently reset MuJoCo run per selected target; center entry only, no harvest evaluator')
 (out/'summary.json').write_text(json.dumps(summary,indent=2))
 links=''.join(f'<li>{r["scene"]} / {r["target"]}: {r["result"]} · '
               f'<a href="{r["scene"]}/{r["target"]}/physics/index.html">물리 재생</a></li>'
               for r in summary['results'])
 (out/'index.html').write_text('<meta charset="utf-8"><h1>새 랜덤 장면 추천 경로 물리 검증</h1>'
  '<p>추천 확정 후 열매당 경로 1개를 새 MuJoCo 물리로 실행했습니다. 판정은 중심 진입이며 꼭지 걸림은 미검증입니다.</p><ul>'
  +links+'</ul><a href="summary.json">요약</a> · <a href="recommendations.json">사전 확정 추천</a>')
 return summary

def main():
 p=argparse.ArgumentParser();p.add_argument('training',type=Path);p.add_argument('--workers',type=int,default=8);p.add_argument('--collection',type=Path,help='fresh random GLB plan-only collection');p.add_argument('--output',type=Path);a=p.parse_args()
 if a.collection:
  print(json.dumps(test_random_scenes(a.training,a.collection,a.workers,a.output),indent=2))
  return
 root=a.training.resolve();m=json.loads((root/'manifest.json').read_text());rec=json.loads((root/'recommendations.json').read_text());dataset=next(d for d in m['datasets'] if d['target']==m['test_target']);source=Path(dataset['source_run']);out=root/'physics';out.mkdir(exist_ok=False)
 from candidate_experiment import initialize,execute,report
 # Freeze/hash recommendations before inspecting any fresh outcome.
 shutil.copy2(root/'recommendations.json',out/'recommendations.json');shutil.copytree(source/'replay_assets',out/'replay_assets');(out/'candidates').mkdir()
 for recommendation in rec:
  view=Path(dataset['root'])/'observations'/recommendation['view'];pose=np.array(json.loads((view/'camera.json').read_text())['world_from_optical']['color']);obs=json.loads((view/'observation.json').read_text());cid=recommendation['candidate_id']
  with np.load(view/'actions_camera_v2.npz') as z:
   index=z['candidate_ids'].tolist().index(cid);xyz=z['target_relative_waypoint_xyz_camera'][index]@pose[:3,:3].T+obs['target']['center_world_gt']
  plan=json.loads((source/'candidates'/cid/'plan.json').read_text());np.testing.assert_allclose(xyz,[p['position_xyz'] for p in plan['waypoints']],atol=1e-9)
 ids=sorted(set(r['candidate_id'] for r in rec))
 for cid in ids:
  dest=out/'candidates'/cid;dest.mkdir()
  for name in ['plan.json','trace.json']:shutil.copy2(source/'candidates'/cid/name,dest/name)
 manifest=json.loads((source/'manifest.json').read_text());manifest.update(count=len(ids),workers=a.workers,training_run=str(root),source_run=str(source),recommendations_sha256=hashlib.sha256((out/'recommendations.json').read_bytes()).hexdigest(),scope='new physics for unique fixed recommendations; stored executable joint plans reused, not generated by the visual model')
 manifest.pop('total_wall_s',None);(out/'manifest.json').write_text(json.dumps(manifest,indent=2));results=[];start=time.perf_counter()
 with ProcessPoolExecutor(max_workers=min(a.workers,len(ids)),mp_context=multiprocessing.get_context('spawn'),initializer=initialize,initargs=(str(out),)) as pool:
  futures={pool.submit(execute,{'candidate_id':cid}):cid for cid in ids}
  for f in as_completed(futures):
   r=f.result();results.append(r);print('[추천 물리 검증]',r['candidate_id'],r['result'],flush=True)
 manifest['total_wall_s']=time.perf_counter()-start;(out/'manifest.json').write_text(json.dumps(manifest,indent=2));report(out,results)
 lookup={r['candidate_id']:r for r in results};summary=dict(unique_paths=len(results),unique_successes=sum(r['result']=='partial_center_entry' for r in results),view_recommendations=len(rec),view_weighted_successes=sum(lookup[r['candidate_id']]['result']=='partial_center_entry' for r in rec),wall_s=manifest['total_wall_s'],note='one fresh simulation per unique path; virtual observations share outcomes; no independent view trials')
 (out/'summary.json').write_text(json.dumps(summary,indent=2));print('[검증 완료]',json.dumps(summary),flush=True)
if __name__=='__main__':main()
