"""Tomato05 pilot: held-out views AND actions; frozen RGBD features + action scorer."""
import argparse,copy,csv,datetime,hashlib,json,os,random,shutil,time
from pathlib import Path
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
from features import extract,sha

class Scorer(nn.Module):
    def __init__(self,obs_dim):
        super().__init__();self.obs_dim=obs_dim
        self.action=nn.Sequential(nn.Linear(17,96),nn.SiLU(),nn.Linear(96,64),nn.SiLU())
        if obs_dim:self.observation=nn.Sequential(nn.Linear(obs_dim,128),nn.SiLU(),nn.Dropout(.15),nn.Linear(128,64),nn.SiLU())
        self.head=nn.Sequential(nn.Linear(128 if obs_dim else 64,96),nn.SiLU(),nn.Dropout(.1),nn.Linear(96,2))
    def forward(self,a,o):
        h=self.action(a)
        if self.obs_dim:h=torch.cat([h,self.observation(o)],-1)
        return self.head(h)

def split_actions(actions,seed):
    rng=np.random.default_rng(seed);splits={k:[] for k in ('train','validation','test')}
    for label in sorted(set(a['result'] for a in actions)):
        ids=np.array([i for i,a in enumerate(actions) if a['result']==label]);rng.shuffle(ids)
        n=len(ids);n1=int(.7*n);n2=int(.15*n)
        for k,ix in zip(splits,[ids[:n1],ids[n1:n1+n2],ids[n1+n2:]]):splits[k].extend(ix.tolist())
    return {k:np.array(sorted(v)) for k,v in splits.items()}

def metrics(y,d,p,pred):
    order=np.argsort(-p,kind='stable');positive=y.sum();ap=float(((np.cumsum(y[order])/(np.arange(len(y))+1))*y[order]).sum()/positive) if positive else None
    return dict(entry_average_precision=ap,entry_brier=float(np.mean((p-y)**2)),displacement_mae_mm=float(np.mean(abs(pred-d))*20),entry_prevalence=float(y.mean()))

def arrays(root):
    rows=[json.loads(s) for s in (root/'observations.jsonl').read_text().splitlines()];actions=json.loads((root/'actions.json').read_text());actions.sort(key=lambda a:a['candidate_id'])
    ids=[a['candidate_id'] for a in actions];x=[]
    for row in rows:
        with np.load(root/'observations'/row['observation_id']/'actions_camera_v2.npz') as z:
            assert z['candidate_ids'].tolist()==ids
            x.append(np.c_[z['features'],np.repeat(z['gravity_direction_camera'][None],len(ids),0)])
    y=np.array([a['center_entered'] for a in actions],dtype='float32');d=np.array([a['target_center_max_displacement_m']*50 for a in actions],dtype='float32')
    assert np.isfinite(x).all() and np.isfinite(d).all()
    return rows,actions,np.array(x,dtype='float32'),y,d

def train_one(name,seed,x,obs,y,d,views,splits,out,epochs,device):
    began=time.perf_counter();random.seed(seed);np.random.seed(seed);torch.manual_seed(seed);torch.cuda.manual_seed_all(seed)
    folder=out/f'{name}_seed{seed}';folder.mkdir()
    tv=views['train'];ta=splits['train'];vv=views['validation'];va=splits['validation']
    # Fit all normalizers ONLY on training views/actions.
    fit=x[tv][:,ta].reshape(-1,17);am=fit.mean(0);ast=np.maximum(fit.std(0),.02)
    om=obs[tv].mean(0);ost=np.maximum(obs[tv].std(0),.1)
    ax=torch.tensor((x-am)/ast,device=device);ox=torch.tensor((obs-om)/ost,device=device)
    yy=torch.tensor(y,device=device);dd=torch.tensor(d,device=device)
    model=Scorer(obs.shape[1]).to(device);optimizer=torch.optim.AdamW(model.parameters(),lr=.001,weight_decay=.001)
    trv=torch.tensor(np.repeat(tv,len(ta)),device=device);tra=torch.tensor(np.tile(ta,len(tv)),device=device)
    valv=torch.tensor(np.repeat(vv,len(va)),device=device);vala=torch.tensor(np.tile(va,len(vv)),device=device)
    best=float('inf');best_epoch=0;history=[];state=None
    def loss(pred,indices):return F.binary_cross_entropy_with_logits(pred[:,0],yy[indices])+.25*F.smooth_l1_loss(pred[:,1],dd[indices])
    for epoch in range(1,epochs+1):
        model.train();perm=torch.randperm(len(tra),device=device);total=0.
        for ids in perm.split(2048):
            vi=trv[ids];ai=tra[ids];pred=model(ax[vi,ai],ox[vi]);l=loss(pred,ai);optimizer.zero_grad(set_to_none=True);l.backward();optimizer.step();total+=l.item()*len(ids)
        model.eval()
        with torch.no_grad():vl=loss(model(ax[valv,vala],ox[valv]),vala).item()
        history.append(dict(epoch=epoch,train_loss=total/len(tra),validation_loss=vl))
        if vl<best-1e-5:best=vl;best_epoch=epoch;state=copy.deepcopy(model.state_dict())
        if epoch%25==0:print('[학습]',name,'seed',seed,'epoch',epoch,'검증손실',round(vl,4),flush=True)
        if epoch-best_epoch>=30:break
    model.load_state_dict(state);model.eval();predictions=[];summaries={}
    with torch.no_grad():
        for split in ('test','visibility_test'):
            for action_set,ai in [('unseen_actions',splits['test']),('seen_actions',splits['train'])]:
                for alteration in (('normal','shuffled_rgbd') if obs.shape[1] else ('normal',)):
                    values=[];pv=[];dv=[];top=[]
                    for j,vi in enumerate(views[split]):
                        # Wrong observation, SAME action and gravity. Tests dependence on aligned image/depth.
                        oi=vi if alteration=='normal' else int(tv[j%len(tv)])
                        pred=model(ax[vi,ai],ox[oi].expand(len(ai),-1));prob=pred[:,0].sigmoid().cpu().numpy();disp=pred[:,1].clamp_min(0).cpu().numpy()
                        score=prob*np.exp(-disp);rank=np.argsort(-score);best_i=int(ai[rank[0]])
                        top.append(dict(view_index=int(vi),candidate_index=best_i,actual_center_entered=bool(y[best_i]),actual_max_displacement_mm=float(d[best_i]*20),predicted_entry_probability=float(prob[rank[0]]),predicted_max_displacement_mm=float(disp[rank[0]]*20),top5_entered_fraction=float(y[ai[rank[:5]]].mean())))
                        pv.extend(prob.tolist());dv.extend(disp.tolist());values.extend(ai.tolist())
                        predictions.extend(dict(view_index=int(vi),candidate_index=int(a),split=split,action_set=action_set,alteration=alteration,entry_probability=float(p),predicted_displacement_mm=float(v*20),score=float(sc)) for a,p,v,sc in zip(ai,prob,disp,score))
                    met=metrics(y[values],d[values],np.array(pv),np.array(dv));met.update(top1_entry_rate=float(np.mean([t['actual_center_entered'] for t in top])),top1_low_displacement_entry_rate=float(np.mean([t['actual_center_entered'] and t['actual_max_displacement_mm']<=20 for t in top])),top1_mean_displacement_mm=float(np.mean([t['actual_max_displacement_mm'] for t in top])),top=top)
                    summaries[f'{split}/{action_set}/{alteration}']=met
    torch.save(dict(state_dict={k:v.cpu() for k,v in state.items()},obs_dim=obs.shape[1],action_mean=am,action_std=ast,observation_mean=om,observation_std=ost,name=name,seed=seed,best_epoch=best_epoch,selection='validation BCE + .25 smooth L1 displacement/20mm',score='sigmoid(entry_logit)*exp(-max(predicted_displacement_mm,0)/20)',feature_file='../features.npz'),folder/'model.pt')
    (folder/'history.json').write_text(json.dumps(history,indent=2));(folder/'metrics.json').write_text(json.dumps(summaries,indent=2))
    with (folder/'predictions.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(predictions[0]));w.writeheader();w.writerows(predictions)
    return dict(model=name,seed=seed,best_epoch=best_epoch,validation_loss=best,wall_s=time.perf_counter()-began,metrics=summaries)

def main():
    p=argparse.ArgumentParser();p.add_argument('dataset',type=Path);p.add_argument('--output',type=Path);p.add_argument('--epochs',type=int,default=150);p.add_argument('--seeds',default='0,1,2');a=p.parse_args()
    root=a.dataset.resolve();out=a.output or root.parent/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_camera_action_training');out.mkdir(parents=True,exist_ok=False);print('[학습 결과]',out,flush=True)
    torch.set_num_threads(4);device='cuda' if torch.cuda.is_available() else 'cpu';began=time.perf_counter();rows,actions,x,y,d=arrays(root)
    views={s:np.array([i for i,r in enumerate(rows) if r['split']==s]) for s in ('train','validation','test','visibility_test')};splits=split_actions(actions,20260922)
    assert not(set(splits['train'])&set(splits['test']));assert not(set(views['train'])&set(views['test']))
    manifest=dict(dataset=str(root),dataset_sha256=sha(root/'dataset.json'),actions_sha256=sha(root/'actions.json'),device=device,torch=torch.__version__,epochs=a.epochs,seeds=a.seeds,views={k:[rows[i]['observation_id'] for i in v] for k,v in views.items()},action_splits={k:[actions[i]['candidate_id'] for i in v] for k,v in splits.items()},selection_rule='validation loss only; tests never used for model selection',limitations=['one physical scene and 1000 unique rollouts; 71000 pairs are not independent experiments','GT target crop and simulated camera gravity assumed; no learned detector','offline outcome lookup, not new physics or real robot deployment','hook_success remains unvalidated'],source_run=json.loads((root/'dataset.json').read_text())['source_run'])
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2));shutil.copytree(Path(__file__).parent,out/'source',ignore=shutil.ignore_patterns('__pycache__'))
    feat=extract(root,out,device);runs=[]
    for name in ('action_only','resnet18_rgbd','dinov2_rgbd'):
        obs=np.zeros((len(rows),0),dtype='float32') if name=='action_only' else np.c_[feat[name.split('_')[0]],feat['depth'],feat['crop_intrinsics']]
        for seed in map(int,a.seeds.split(',')):
            result=train_one(name,seed,x,obs,y,d,views,splits,out,a.epochs,device);runs.append(result);(out/'results.json').write_text(json.dumps(runs,indent=2));print('[학습 완료]',name,seed,'best epoch',result['best_epoch'],flush=True)
    manifest['wall_s']=time.perf_counter()-began;manifest['selected_model']=min(set(r['model'] for r in runs),key=lambda name:np.mean([r['validation_loss'] for r in runs if r['model']==name]))
    # Known-scene lookup exposes the central limitation: no visual reasoning is needed here.
    train_best=min((i for i in splits['train'] if y[i]),key=lambda i:d[i]);manifest['known_scene_lookup_baseline']=dict(candidate_id=actions[train_best]['candidate_id'],max_displacement_mm=float(d[train_best]*20),entry=True,explanation='Same previously measured path works identically for every virtual view; not comparable to unseen-action ranking and not visual generalization')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    from report import generate
    generate(out,rows,actions);print('[전체 완료]',out/'index.html','시간',manifest['wall_s'],flush=True)
if __name__=='__main__':main()
