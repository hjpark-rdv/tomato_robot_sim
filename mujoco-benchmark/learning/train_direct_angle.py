"""Learn one world-frame entry azimuth from one nominal RGB-D view per fruit."""
import argparse,datetime,hashlib,json,random,time
from pathlib import Path
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
from features import extract
from train_multi import random_scene_layout


def label_for_target(results):
    good=[r for r in results if r['result']=='partial_center_entry'
          and r.get('metrics',{}).get('glb_physics_valid',True)
          and r.get('metrics',{}).get('max_hook_contact_penetration_m',0.)<=.0005]
    if not good:return None
    # A measured successful path with the least target displacement gives one
    # deterministic supervised angle; it does not assert other angles are bad.
    row=min(good,key=lambda r:(r['target_center_max_displacement_m'],r['candidate_id']))
    return float(row['parameters']['approach_azimuth_deg']),row['candidate_id']


def angle_model(dim):
    return nn.Sequential(nn.Linear(dim,64),nn.ReLU(),nn.Linear(64,1))


def train(collection,out,epochs=400,seeds=(0,1,2)):
    root=Path(collection).resolve();data=json.loads((root/'collection.json').read_text())
    if data.get('schema')!='training_dataset_v1' or data.get('status')!='complete':
        raise ValueError('Expected the completed five-GLB training_dataset_v1 collection')
    splits,folder_name=random_scene_layout(data)
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=False)
    input_root=out/'observation_inputs';(input_root/'observations').mkdir(parents=True)
    rows=[];labels=[];records=[]
    for item in sorted(data['targets'],key=lambda r:(r['scene'],r['target'])):
        target_root=root/folder_name(item['scene'])/'targets'/item['target']
        result_file=target_root/'physics/results.json'
        answer=label_for_target(json.loads(result_file.read_text()))
        if answer is None:continue
        source=target_root/'observations/observations/view_0000'
        row=json.loads((source/'observation.json').read_text())
        if not row.get('nominal') or not row['target']['input_usable'] or not all(row['crops'][k]['available'] for k in ('local','context')):
            continue
        identity=f"{item['scene']}__{item['target']}"; (input_root/'observations'/identity).symlink_to(source, target_is_directory=True)
        row['observation_id']=identity;rows.append(row);labels.append(answer[0]);records.append(dict(scene=item['scene'],target=item['target'],split=splits[item['scene']],view='view_0000',label_angle_deg=answer[0],source_candidate=answer[1],results_sha256=hashlib.sha256(result_file.read_bytes()).hexdigest()))
    if set(r['split'] for r in records)!={'train','validation','test'}:raise ValueError('A split has no labeled nominal images')
    torch.set_num_threads(4);device='cuda' if torch.cuda.is_available() else 'cpu'
    start=time.perf_counter();(out/'features').mkdir();features=extract(input_root,out/'features',device,rows=rows,backbone_store=out/'backbones')
    indices={name:np.array([i for i,r in enumerate(records) if r['split']==name]) for name in ('train','validation','test')}
    y=np.array(labels,dtype=np.float32);runs=[]
    for backbone in ('resnet18','dinov2'):
        x=np.c_[features[backbone],features['depth'],features['crop_intrinsics']].astype(np.float32)
        mean=x[indices['train']].mean(0);std=np.maximum(x[indices['train']].std(0),.1)
        xt=torch.tensor((x-mean)/std,device=device);yt=torch.tensor(y/90.,device=device)
        for seed in seeds:
            torch.manual_seed(seed);np.random.seed(seed);random.seed(seed)
            model=angle_model(x.shape[1]).to(device);optimizer=torch.optim.AdamW(model.parameters(),lr=.001,weight_decay=.01)
            best=float('inf');best_epoch=0;state=None
            for epoch in range(1,epochs+1):
                model.train();prediction=model(xt[indices['train']])[:,0].tanh()
                loss=F.smooth_l1_loss(prediction,yt[indices['train']]);optimizer.zero_grad();loss.backward();optimizer.step()
                model.eval()
                with torch.no_grad():
                    val=float(torch.mean(torch.abs(90*model(xt[indices['validation']])[:,0].tanh()-90*yt[indices['validation']])).item())
                if val<best-1e-4:
                    best=val;best_epoch=epoch;state={k:v.cpu().clone() for k,v in model.state_dict().items()}
                if epoch-best_epoch>=50:break
            model.load_state_dict(state);model.eval()
            with torch.no_grad():pred=90*model(xt)[:,0].tanh().cpu().numpy()
            test=indices['test'];test_mae=float(np.mean(np.abs(pred[test]-y[test])))
            folder=out/f'{backbone}_seed{seed}';folder.mkdir()
            torch.save(dict(schema='direct_rgbd_angle_v1',backbone=backbone,state_dict=state,feature_mean=mean,feature_std=std,train_collection=str(root),train_seed=data['seed'],train_scenes=[s for s,v in splits.items() if v=='train'],validation_scenes=[s for s,v in splits.items() if v=='validation'],test_scenes=[s for s,v in splits.items() if v=='test'],input='one nominal target-centered RGB-D view; frozen RGB and metric depth features; no candidate action',label='minimum-displacement valid center-entry angle among measured candidates'),folder/'model.pt')
            runs.append(dict(backbone=backbone,seed=seed,best_epoch=best_epoch,validation_mae_deg=best,test_mae_deg=test_mae,test_within_20deg=float(np.mean(np.abs(pred[test]-y[test])<=20))))
            print('[직접 각도 학습]',backbone,seed,'validation MAE',round(best,2),'test MAE',round(test_mae,2),flush=True)
    selected=min(runs,key=lambda r:r['validation_mae_deg'])
    baseline=float(np.mean(np.abs(y[indices['test']]-np.mean(y[indices['train']]))))
    manifest=dict(schema='direct_rgbd_angle_training_v1',collection=str(root),scene_splits=splits,records=records,selected=selected,runs=runs,constant_angle_test_mae_deg=baseline,wall_s=time.perf_counter()-start,scope='one nominal virtual RGB-D image to one world azimuth; original fruit GT crop used; center-entry labels, not harvest success',limitations=['Only five independent plant scenes; views of one scene are not independent samples','Angle label is one measured successful candidate, not a unique ground-truth optimum','A target with no discovered valid success has no supervised angle label'])
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2));(out/'index.html').write_text('<meta charset="utf-8"><h1>단일 RGB-D 사진 → 진입각 직접 예측</h1><p>선택: '+selected['backbone']+' seed '+str(selected['seed'])+'</p><p>검증 각도 MAE '+f"{selected['validation_mae_deg']:.1f}"+'° · 보류 GLB 각도 MAE '+f"{selected['test_mae_deg']:.1f}"+'° · 일정각 기준 '+f'{baseline:.1f}'+'°</p><a href="manifest.json">상세 결과</a>')
    return manifest


def predict(checkpoint,feature):
    ck=torch.load(checkpoint,map_location='cpu',weights_only=False)
    x=np.asarray(feature,dtype=np.float32);model=angle_model(len(x));model.load_state_dict(ck['state_dict']);model.eval()
    with torch.no_grad():return float(90*model(torch.tensor((x-ck['feature_mean'])/ck['feature_std'])[None])[:,0].tanh()[0])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('collection',type=Path);p.add_argument('--output',type=Path,default=Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_direct_angle_training'));p.add_argument('--epochs',type=int,default=400);p.add_argument('--seeds',default='0,1,2');a=p.parse_args();train(a.collection,a.output,a.epochs,tuple(map(int,a.seeds.split(','))))
