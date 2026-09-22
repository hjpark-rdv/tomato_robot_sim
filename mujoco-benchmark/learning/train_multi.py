"""고정 고리 경로군 RGB-D 진입 평가기. 토마토 ID 단위 holdout, 진입만 점수화."""
import argparse,copy,csv,datetime,html,json,random,shutil,time
from pathlib import Path
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
from features import extract,sha
from train import Scorer


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
