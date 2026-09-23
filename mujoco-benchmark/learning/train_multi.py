"""고정 고리 경로군 RGB-D 진입 평가기. 토마토 ID 단위 holdout, 진입만 점수화."""
import argparse,copy,csv,datetime,html,json,random,shutil,time
from pathlib import Path
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
from features import extract,sha
from train import Scorer

def random_scene_layout(collection):
 schema=collection.get('schema')
 if schema=='random_glb_collection_v1':
  return collection['scene_splits'],lambda scene:scene
 if schema=='training_dataset_v1':
  scenes=sorted({row['scene'] for row in collection['targets']})
  if len(scenes)<3:raise ValueError('Training needs at least three independent GLB scenes')
  # Split by entire GLB scene; every tomato and camera view follows its scene.
  train_count=len(scenes)-2
  splits={scene:('train' if index<train_count else 'validation' if index==train_count else 'test') for index,scene in enumerate(scenes)}
  return splits,lambda scene:'scene_'+scene
 raise ValueError('Unsupported random-scene collection schema: '+str(schema))


def train_random_scenes(a, collection):
 """The same frozen image features and Scorer, with scene-held-out ragged candidates."""
 root=a.collection.resolve()
 if collection.get('plan_only'):raise ValueError('Plan-only runs have no training labels')
 if collection.get('status')!='complete':raise ValueError('Random-scene collection is incomplete')
 splits,scene_folder=random_scene_layout(collection)
 if set(splits.values())!={'train','validation','test'}:
  raise ValueError('Training requires distinct train, validation and test scenes')
 out=a.output or root.parent/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_random_glb_training')
 out.mkdir(parents=True,exist_ok=False)
 torch.set_num_threads(4);device='cuda' if torch.cuda.is_available() else 'cpu';start=time.perf_counter()
 xs=[];ys=[];obs_features={name:[] for name in ('resnet18_rgbd','dinov2_rgbd')};groups=[];sources=[];profiles=set()
 for item in sorted(collection['targets'],key=lambda r:(r['scene'],r['target'])):
  scene=item['scene'];folder=root/scene_folder(scene)/'targets'/item['target'];obsroot=folder/'observations';physics=folder/'physics'
  meta=json.loads((obsroot/'dataset.json').read_text());run=json.loads((physics/'manifest.json').read_text())
  if meta.get('schema')!='farmily_observation_v2' or meta['source_results_sha256']!=sha(physics/'results.json'):
   sources.append(dict(scene=scene,target=item['target'],excluded='no executable paths or stale observations'));continue
  if run.get('classification_rule')!='center_entry_only_v2' or run.get('plan_only'):
   raise ValueError('Unexpected label definition: '+str(physics))
  actions=sorted(json.loads((obsroot/'actions.json').read_text()),key=lambda v:v['candidate_id'])
  physical_results=json.loads((physics/'results.json').read_text())
  # Older GLB rollouts omitted hook-to-original-stem penetration from validity.
  rejected={r['candidate_id'] for r in physical_results if 'glb_physics_valid' in r.get('metrics',{}) and (not r['metrics']['glb_physics_valid'] or r['metrics'].get('max_hook_contact_penetration_m',0)>.0005)}
  eligible=[action for action in actions if action['candidate_id'] not in rejected and action['result'] in ('partial_center_entry','miss') and len(action.get('waypoints_world',[])) in (4,5)]
  excluded={label:sum(x['result']==label for x in actions) for label in sorted({x['result'] for x in actions}) if label not in ('partial_center_entry','miss')}
  excluded['legacy_hook_penetration']=sum(x['candidate_id'] in rejected and x['result'] in ('partial_center_entry','miss') for x in actions)
  if not eligible:
   sources.append(dict(scene=scene,target=item['target'],excluded=excluded));continue
  profiles.update(a['parameters'].get('lift_profile','vertical') for a in eligible)
  rows=[json.loads(s) for s in (obsroot/'observations.jsonl').read_text().splitlines()]
  valid=[r for r in rows if r['target']['input_usable'] and all(r['crops'][k]['available'] for k in ('local','context'))]
  if not valid:
   sources.append(dict(scene=scene,target=item['target'],excluded='no usable RGB-D views'));continue
  featureout=out/'features'/scene/item['target'];featureout.mkdir(parents=True)
  feature=extract(obsroot,featureout,device,rows=valid,backbone_store=out/'backbones')
  labels={r['candidate_id']:r['result']=='partial_center_entry' for r in eligible}
  for index,row in enumerate(valid):
   with np.load(obsroot/'observations'/row['observation_id']/'actions_camera_v2.npz') as z:
    saved=z['candidate_ids'].tolist();chosen=[saved.index(r['candidate_id']) for r in eligible]
    action=np.c_[z['features'][chosen],np.repeat(z['gravity_direction_camera'][None],len(chosen),axis=0)].astype('float32')
   if not np.isfinite(action).all():raise ValueError('Non-finite camera action: '+str(obsroot))
   begin=sum(len(x) for x in xs);end=begin+len(eligible)
   xs.append(action);ys.extend(float(labels[r['candidate_id']]) for r in eligible)
   for name in obs_features:
    vector=np.r_[feature[name.split('_')[0]][index],feature['depth'][index],feature['crop_intrinsics'][index]].astype('float32')
    obs_features[name].append(np.repeat(vector[None],len(eligible),axis=0))
   groups.append(dict(scene=scene,target=item['target'],view=row['observation_id'],split=splits[scene],
                      pair_start=begin,pair_end=end,candidate_ids=[r['candidate_id'] for r in eligible],
                      observation_root=str(obsroot)))
  sources.append(dict(scene=scene,target=item['target'],actions=len(actions),eligible=len(eligible),
                      physical_rollouts=sum(v['result'] in ('partial_center_entry','miss','invalid_physics') for v in actions),
                      excluded=excluded,views=len(valid),entries=sum(labels.values()),
                      physics_results_sha256=sha(physics/'results.json'),dataset_sha256=sha(obsroot/'dataset.json')))
 if len(profiles)!=1:raise ValueError('Do not mix trajectory profiles; compact action14 cannot distinguish them')
 x=np.concatenate(xs);y=np.asarray(ys,dtype='float32')
 pair_splits={name:np.concatenate([np.arange(g['pair_start'],g['pair_end']) for g in groups if g['split']==name])
              for name in ('train','validation','test')}
 if any(not len(v) for v in pair_splits.values()):raise ValueError('No usable labeled pairs in a scene split')
 assert all(set(pair_splits[a]).isdisjoint(pair_splits[b]) for a,b in [('train','validation'),('train','test'),('validation','test')])
 assert all(set(g['scene'] for g in groups if g['split']==a).isdisjoint(g['scene'] for g in groups if g['split']==b) for a,b in [('train','validation'),('train','test'),('validation','test')])
 am=x[pair_splits['train']].mean(0);ast=np.maximum(x[pair_splits['train']].std(0),.02)
 ax=torch.tensor((x-am)/ast,device=device);yy=torch.tensor(y,device=device)
 manifest=dict(schema='random_glb_training_v1',collection=str(root),source_collection_schema=collection['schema'],scene_splits=splits,
               groups=groups,sources=sources,profile=next(iter(profiles)),
               pair_counts={name:len(v) for name,v in pair_splits.items()},
               physical_rollouts=sum(s.get('physical_rollouts',0) for s in sources),
               selection='model mean validation BCE then seed validation BCE',
               input='same camera action14 + gravity3 and frozen RGB-D features as train_multi',
               label='center_entry_only_v2; invalid physics and planning failures excluded',
               search_policy='10 sampled paths per target; zero discovered entries is allowed and is not proof of physical impossibility',
               test_scene_policy='test labels unused in normalization, training and model selection',
               hook_success=None,device=device,epochs=a.epochs,seeds=a.seeds)
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
 runs=[];test_predictions={}
 for name in ('action_only','resnet18_rgbd','dinov2_rgbd'):
  obs=np.zeros((len(x),0),dtype='float32') if name=='action_only' else np.concatenate(obs_features[name])
  om=obs[pair_splits['train']].mean(0);ost=np.maximum(obs[pair_splits['train']].std(0),.1)
  ox=torch.tensor((obs-om)/ost,device=device)
  for seed in map(int,a.seeds.split(',')):
   torch.manual_seed(seed);np.random.seed(seed);random.seed(seed)
   model=Scorer(obs.shape[1]);model.head[-1]=nn.Linear(96,1);model=model.to(device)
   optimizer=torch.optim.AdamW(model.parameters(),lr=.001,weight_decay=.001)
   best=float('inf');best_epoch=0;state=None;history=[]
   for epoch in range(1,a.epochs+1):
    model.train();total=0.
    for batch in torch.randperm(len(pair_splits['train']),device=device).split(2048):
     ix=torch.tensor(pair_splits['train'][batch.cpu().numpy()],device=device)
     loss=F.binary_cross_entropy_with_logits(model(ax[ix],ox[ix])[:,0],yy[ix]);optimizer.zero_grad(set_to_none=True);loss.backward();optimizer.step();total+=float(loss)*len(ix)
    model.eval()
    with torch.no_grad():
     ix=torch.tensor(pair_splits['validation'],device=device)
     validation=float(F.binary_cross_entropy_with_logits(model(ax[ix],ox[ix])[:,0],yy[ix]))
    history.append(dict(epoch=epoch,train_bce=total/len(pair_splits['train']),validation_bce=validation))
    if validation<best-1e-5:best=validation;best_epoch=epoch;state=copy.deepcopy(model.state_dict())
    if epoch-best_epoch>=30:break
   model.load_state_dict(state);model.eval()
   with torch.no_grad():
    score=model(ax,ox)[:,0].sigmoid().cpu().numpy()
   test=pair_splits['test'];test_groups=[g for g in groups if g['split']=='test']
   recommendations=[]
   for g in test_groups:
    selected=int(g['pair_start']+np.argmax(score[g['pair_start']:g['pair_end']]))
    recommendations.append(dict(scene=g['scene'],target=g['target'],view=g['view'],
                                candidate_id=g['candidate_ids'][selected-g['pair_start']],
                                entry_score=float(score[selected]),actual_entry=bool(y[selected])))
   all_test_targets={(r['scene'],r['target']) for r in collection['targets'] if splits[r['scene']]=='test'}
   covered_targets={(r['scene'],r['target']) for r in recommendations}
   metrics=dict(top1_entry_rate=float(np.mean([r['actual_entry'] for r in recommendations])),
                random_candidate_entry_rate=float(y[test].mean()),
                brier=float(np.square(score[test]-y[test]).mean()),
                view_recommendations=len(recommendations),
                test_targets_total=len(all_test_targets),eligible_targets=len(covered_targets),
                targets_without_eligible_input=len(all_test_targets-covered_targets),
                unique_target_candidate_pairs=len({(r['scene'],r['target'],r['candidate_id']) for r in recommendations}))
   folder=out/f'{name}_seed{seed}';folder.mkdir()
   torch.save(dict(state_dict={k:v.cpu() for k,v in state.items()},obs_dim=obs.shape[1],
                   action_mean=am,action_std=ast,observation_mean=om,observation_std=ost,
                   score='entry sigmoid only',architecture='Scorer with one output',
                   model=name,seed=seed),folder/'model.pt')
   (folder/'history.json').write_text(json.dumps(history,indent=2))
   row=dict(model=name,seed=seed,best_epoch=best_epoch,validation_bce=best,test=metrics)
   runs.append(row);test_predictions[(name,seed)]=recommendations
   print('[장면 분할 학습]',json.dumps(row),flush=True)
 selected_model=min({r['model'] for r in runs},key=lambda name:np.mean([r['validation_bce'] for r in runs if r['model']==name]))
 selected=min((r for r in runs if r['model']==selected_model),key=lambda r:r['validation_bce'])
 manifest.update(selected_model=selected_model,selected_seed=selected['seed'],wall_s=time.perf_counter()-start)
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2));(out/'results.json').write_text(json.dumps(runs,indent=2))
 (out/'recommendations.json').write_text(json.dumps(test_predictions[(selected_model,selected['seed'])],indent=2))
 summary=''.join(f'<tr><td>{html.escape(r["model"])}</td><td>{r["seed"]}</td><td>{r["validation_bce"]:.4f}</td><td>{r["test"]["top1_entry_rate"]:.1%}</td><td>{r["test"]["brier"]:.3f}</td></tr>' for r in runs)
 (out/'index.html').write_text('<meta charset="utf-8"><h1>랜덤 GLB 장면 단위 진입 학습</h1>'
  '<p>선택 모델: '+html.escape(selected_model)+f' seed {selected["seed"]}. '
  '중심 진입 판정이며 꼭지 걸림 성공이 아닙니다. 1순위 진입률은 계획 가능한 후보와 사용 가능한 관측이 있는 시점만의 값입니다. '
  '계획 불가 열매도 원본 수집과 평가 대상 목록에 남아 있습니다. 카메라 시점별 추천은 독립 물리 실행 횟수가 아닙니다.</p>'
  '<table><tr><th>모델</th><th>seed</th><th>검증 BCE</th><th>테스트 1순위 진입률</th><th>Brier</th></tr>'+summary+'</table>'
  '<p><a href="manifest.json">분할/원본</a> · <a href="results.json">수치</a> · '
  '<a href="recommendations.json">테스트 추천</a></p>')
 print('[장면 분할 학습 완료]',out/'index.html',flush=True)
 return out


def report(out):
 m=json.loads((out/'manifest.json').read_text());rs=json.loads((out/'results.json').read_text())
 rows=''.join(f"<tr><td>{html.escape(r['model'])}</td><td>{r['seed']}</td><td>{r['best_epoch']}</td><td>{r['validation_bce']:.4f}</td><td>{r['test']['top1_entry_rate']:.1%}</td><td>{r['test']['brier']:.3f}</td></tr>" for r in rs)
 selected=next(r for r in rs if r['model']==m['selected_model'] and r['seed']==m['selected_seed'])
 preds=json.loads((out/'recommendations.json').read_text());details=''.join(f"<tr><td>{html.escape(p['view'])}</td><td>{p['candidate_id']}</td><td>{p['azimuth_deg']:+.1f}°</td><td>{p['entry_score']:.3f}</td><td>{'진입' if p['actual_entry'] else '미진입'}</td><td>{p['actual_displacement_mm']:.1f}</td></tr>" for p in preds)
 page=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>RGB-D 진입각 학습 평가</title><style>body{{background:#14212c;color:#eaf1f5;font:16px/1.6 system-ui;max-width:1200px;margin:40px auto;padding:0 24px}}table{{border-collapse:collapse;width:100%;margin:24px 0}}th,td{{padding:10px;border-bottom:1px solid #40515f;text-align:left}}a{{color:#7be6c4}}.notice{{background:#263846;padding:20px;border-radius:12px}}strong{{color:#7be6c4}}</style>
 <h1>RGB-D로 후보 진입각 평가</h1><div class="notice">학습에서 제외한 <strong>{m['test_target']}</strong> 테스트 · 검증 대상 {m['validation_target']}<br>선택 모델 <strong>{m['selected_model']} / seed {m['selected_seed']}</strong> — 검증 BCE만으로 선택<br>선택 모델 1순위 진입률 <strong>{selected['test']['top1_entry_rate']:.1%}</strong> / 균등 무작위 후보 기대 진입률 {m['random_candidate_entry_rate']:.1%}<br>전체 {m['wall_s']:.1f}초</div>
 <p>입력: RGB·실수 Depth·카메라 좌표 후보. 점수는 진입 sigmoid만 사용하며 밀림을 감점하지 않습니다. 토마토 ID나 실제 결과는 입력하지 않습니다. 목표 크롭은 GT입니다.</p>
 <p>같은 식물의 다른 열매를 holdout한 실험입니다. 가상 카메라 시점은 독립 물리 시험이 아니며 새로운 식물·실물 일반화를 입증하지 않습니다. 꼭지 수확 성공도 아닙니다. 예측 점수는 교정된 확률이 아닙니다.</p>
 <h2>모델 비교</h2><table><tr><th>모델</th><th>seed</th><th>선택 epoch</th><th>검증 BCE</th><th>테스트 1순위 진입률</th><th>Brier</th></tr>{rows}</table>
 <h2>선택 모델의 관측별 추천</h2><p>{len(preds)}시점에서 서로 다른 {len(set(p['candidate_id'] for p in preds))}개 경로를 추천했습니다. 실제 결과 열은 추천 확정 후 기존 데이터를 조회한 오프라인 평가입니다.</p><table><tr><th>관측</th><th>후보</th><th>각도</th><th>점수</th><th>기존 물리 결과</th><th>열매 밀림 mm</th></tr>{details}</table>
 <p><a href="manifest.json">설정·분할·해시</a> · <a href="results.json">평가 수치</a> · <a href="recommendations.json">확정 추천</a> · <a href="physics/index.html">별도 물리 재실행 결과(실행 후)</a></p></html>'''
 (out/'index.html').write_text(page)


def main():
 p=argparse.ArgumentParser();p.add_argument('collection',type=Path);p.add_argument('--test-target',default='Tomato_06');p.add_argument('--validation-target',default='Tomato_09');p.add_argument('--epochs',type=int,default=150);p.add_argument('--seeds',default='0,1,2');p.add_argument('--output',type=Path);a=p.parse_args()
 collection_file=a.collection/'collection.json'
 if collection_file.exists():
  collection=json.loads(collection_file.read_text())
  if collection.get('schema') in ('random_glb_collection_v1','training_dataset_v1'):
   train_random_scenes(a,collection)
   return
 if a.test_target==a.validation_target:p.error('test and validation targets must differ')
 out=a.output or a.collection.parent/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_multi_tomato_training');out.mkdir(parents=True,exist_ok=False);print('[학습 폴더]',out,flush=True)
 torch.set_num_threads(4);device='cuda' if torch.cuda.is_available() else 'cpu';start=time.perf_counter();datasets=[];views=[];xs=[];ys=[];ds=[];featlist=[];targets=[];ids=[];profiles=set();param_sets=[]
 for folder in sorted(a.collection.glob('Tomato_*')):
  root=folder/'observations'
  if not (root/'dataset.json').exists():raise ValueError('Incomplete observation dataset: '+str(root))
  meta=json.loads((root/'dataset.json').read_text());acts=sorted(json.loads((root/'actions.json').read_text()),key=lambda x:x['candidate_id']);rows=[json.loads(l) for l in (root/'observations.jsonl').read_text().splitlines()]
  valid=[r for r in rows if r['target']['input_usable'] and r['split']!='visibility_test' and all(r['crops'][k]['available'] for k in ('local','context'))]
  if not valid:raise ValueError('No usable views: '+str(root))
  if meta['source_results_sha256']!=sha(folder/'physics/results.json'):raise ValueError('Stale observation labels')
  manifest=json.loads((folder/'physics/manifest.json').read_text())
  if manifest.get('classification_rule')!='center_entry_only_v2':raise ValueError('Use current center-entry labels')
  # All cases here must share the fixed path template; compact features omit the dogleg shape.
  for act in acts:
   profiles.add(act['parameters'].get('lift_profile','vertical'))
   if act['result'] not in ('partial_center_entry','miss'):raise ValueError('Invalid/unplanned cases require explicit filtering')
  param_sets.append([{k:v for k,v in act['parameters'].items() if k not in ('candidate_id','seed','sobol_index')} for act in acts])
  featureout=out/'features'/folder.name;featureout.mkdir(parents=True)
  features=extract(root,featureout,device,rows=valid,backbone_store=out/'backbones');featlist.append(features)
  candidate_ids=[x['candidate_id'] for x in acts];local=[]
  for row in valid:
   with np.load(root/'observations'/row['observation_id']/'actions_camera_v2.npz') as z:
    assert z['candidate_ids'].tolist()==candidate_ids
    local.append(np.c_[z['features'],np.repeat(z['gravity_direction_camera'][None],len(acts),axis=0)])
   views.append(dict(target=folder.name,view=row['observation_id'],observation_root=str(root)))
  xs.append(np.array(local,dtype='float32'));ys.append(np.tile([x['result']=='partial_center_entry' for x in acts],(len(valid),1)));ds.append(np.tile([x['target_center_max_displacement_m']*1000 for x in acts],(len(valid),1)))
  targets.extend([folder.name]*len(valid));ids.append(candidate_ids)
  datasets.append(dict(target=folder.name,root=str(root),source_run=meta['source_run'],actions=len(acts),successes=sum(x['result']=='partial_center_entry' for x in acts),usable_views=len(valid),excluded_views=len(rows)-len(valid),dataset_sha256=sha(root/'dataset.json'),actions_sha256=sha(root/'actions.json')))
  print('[특징 준비 완료]',folder.name,len(valid),'시점',flush=True)
 if len(profiles)!=1 or any(x!=param_sets[0] for x in param_sets[1:]) or any(x!=ids[0] for x in ids[1:]):raise ValueError('Do not mix trajectory templates or candidate grids')
 target=np.array(targets);vi={k:np.flatnonzero(mask) for k,mask in dict(train=(target!=a.test_target)&(target!=a.validation_target),validation=target==a.validation_target,test=target==a.test_target).items()}
 if any(not len(v) for v in vi.values()):raise ValueError('Missing train/validation/test targets')
 x=np.concatenate(xs);y=np.concatenate(ys).astype('float32');disp=np.concatenate(ds);assert np.isfinite(x).all()
 manifest=dict(collection=str(a.collection.resolve()),test_target=a.test_target,validation_target=a.validation_target,train_targets=sorted(set(target[vi['train']])),datasets=datasets,views=views,view_splits={k:v.tolist() for k,v in vi.items()},candidate_ids=ids[0],profile=next(iter(profiles)),input='RGBD/crop intrinsics, camera action14+gravity3 only',selection='model average validation BCE then seed validation BCE; never test outcomes',score='sigmoid(entry_logit), no displacement penalty',random_candidate_entry_rate=float(y[vi['test']].mean()),physical_rollouts=sum(d['actions'] for d in datasets),seed_list=a.seeds,epochs=a.epochs,device=device)
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2));shutil.copytree(Path(__file__).parent,out/'source',ignore=shutil.ignore_patterns('__pycache__'))
 am=x[vi['train']].reshape(-1,17).mean(0);ast=np.maximum(x[vi['train']].reshape(-1,17).std(0),.02);ax=torch.tensor((x-am)/ast,device=device);yy=torch.tensor(y,device=device);runs=[];predictions={}
 for name in ('action_only','resnet18_rgbd','dinov2_rgbd'):
  obs=np.zeros((len(views),0),dtype='float32') if name=='action_only' else np.concatenate([np.c_[f[name.split('_')[0]],f['depth'],f['crop_intrinsics']] for f in featlist]).astype('float32')
  om=obs[vi['train']].mean(0);ost=np.maximum(obs[vi['train']].std(0),.1);ox=torch.tensor((obs-om)/ost,device=device)
  trainv=torch.tensor(np.repeat(vi['train'],x.shape[1]),device=device);traina=torch.tensor(np.tile(np.arange(x.shape[1]),len(vi['train'])),device=device)
  for seed in map(int,a.seeds.split(',')):
   torch.manual_seed(seed);np.random.seed(seed);random.seed(seed);model=Scorer(obs.shape[1]);model.head[-1]=nn.Linear(96,1);model=model.to(device);opt=torch.optim.AdamW(model.parameters(),lr=.001,weight_decay=.001);best=float('inf');best_epoch=0;state=None;history=[]
   def predict(v,swap=False):
    return torch.cat([model(ax[int(i)],ox[int(vi['train'][j%len(vi['train'])]) if swap else int(i)].expand(x.shape[1],-1))[:,0][None] for j,i in enumerate(v)])
   for epoch in range(1,a.epochs+1):
    model.train();total=0
    for batch in torch.randperm(len(trainv),device=device).split(2048):
     v=trainv[batch];c=traina[batch];l=F.binary_cross_entropy_with_logits(model(ax[v,c],ox[v])[:,0],yy[v,c]);opt.zero_grad(set_to_none=True);l.backward();opt.step();total+=float(l)*len(batch)
    model.eval()
    with torch.no_grad():loss=float(F.binary_cross_entropy_with_logits(predict(vi['validation']),yy[vi['validation']]))
    history.append(dict(epoch=epoch,train_bce=total/len(trainv),validation_bce=loss))
    if loss<best-1e-5:best=loss;best_epoch=epoch;state=copy.deepcopy(model.state_dict())
    if epoch-best_epoch>=30:break
   model.load_state_dict(state);model.eval()
   with torch.no_grad():pr=predict(vi['test']).sigmoid().cpu().numpy();wrong=predict(vi['test'],True).sigmoid().cpu().numpy() if obs.shape[1] else pr
   actual=y[vi['test']];chosen=np.argmax(pr,axis=1);test=dict(top1_entry_rate=float(actual[np.arange(len(chosen)),chosen].mean()),brier=float(np.square(pr-actual).mean()),wrong_rgbd_top1_entry_rate=float(actual[np.arange(len(chosen)),np.argmax(wrong,axis=1)].mean()),unique_recommended_candidates=int(len(set(chosen.tolist()))))
   folder=out/f'{name}_seed{seed}';folder.mkdir();torch.save(dict(state_dict={k:v.cpu() for k,v in state.items()},obs_dim=obs.shape[1],action_mean=am,action_std=ast,observation_mean=om,observation_std=ost,score='entry sigmoid only',architecture='Scorer with one output',model=name,seed=seed),folder/'model.pt');(folder/'history.json').write_text(json.dumps(history,indent=2));np.savez_compressed(folder/'test_predictions.npz',entry_scores=pr,view_indices=vi['test'],candidate_ids=ids[0])
   row=dict(model=name,seed=seed,best_epoch=best_epoch,validation_bce=best,test=test);runs.append(row);predictions[(name,seed)]=pr;print('[학습 완료]',json.dumps(row),flush=True)
 chosen_model=min({r['model'] for r in runs},key=lambda n:np.mean([r['validation_bce'] for r in runs if r['model']==n]));selected=min([r for r in runs if r['model']==chosen_model],key=lambda r:r['validation_bce']);pr=predictions[(chosen_model,selected['seed'])]
 manifest.update(selected_model=chosen_model,selected_seed=selected['seed'],wall_s=time.perf_counter()-start)
 recommendations=[]
 for j,v in enumerate(vi['test']):
  c=int(np.argmax(pr[j]));recommendations.append(dict(target=a.test_target,view=views[v]['view'],candidate_id=ids[0][c],azimuth_deg=param_sets[0][c]['approach_azimuth_deg'],entry_score=float(pr[j,c]),actual_entry=bool(y[v,c]),actual_displacement_mm=float(disp[v,c])))
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2));(out/'results.json').write_text(json.dumps(runs,indent=2));(out/'recommendations.json').write_text(json.dumps(recommendations,indent=2));report(out);print('[전체 완료]',out/'index.html',round(manifest['wall_s'],2),'초',flush=True)
if __name__=='__main__':main()
