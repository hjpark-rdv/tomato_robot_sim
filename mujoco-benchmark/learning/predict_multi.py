"""저장된 관측 RGB-D 특징으로 후보를 추천. 실제 성공 라벨을 읽지 않는다."""
import argparse,json
from pathlib import Path
import numpy as np
import torch
from torch import nn
from train import Scorer

def score_candidates(checkpoint,action_features,gravity,observation):
 """Score stored camera-action v2 candidates without reading outcome labels."""
 ck=checkpoint if isinstance(checkpoint,dict) else torch.load(checkpoint,map_location='cpu',weights_only=False)
 action=np.c_[action_features,np.repeat(np.asarray(gravity)[None],len(action_features),axis=0)]
 obs=np.asarray(observation,dtype='float32')
 model=Scorer(ck['obs_dim']);model.head[-1]=nn.Linear(96,1);model.load_state_dict(ck['state_dict']);model.eval()
 with torch.no_grad():
  return model(torch.tensor((action-ck['action_mean'])/ck['action_std'],dtype=torch.float32),
               torch.tensor((obs-ck['observation_mean'])/ck['observation_std'],dtype=torch.float32).expand(len(action),-1))[:,0].sigmoid().numpy()


def predict(root,view,top=5):
 root=Path(root);m=json.loads((root/'manifest.json').read_text());target=m['test_target'];name=m['selected_model'];seed=m['selected_seed'];dataset=next(x for x in m['datasets'] if x['target']==target);folder=root/f'{name}_seed{seed}'
 ck=torch.load(folder/'model.pt',map_location='cpu',weights_only=False)
 with np.load(root/'features'/target/'features.npz') as f:
  i=f['view_ids'].tolist().index(view)
  obs=np.zeros((0,),dtype='float32') if name=='action_only' else np.r_[f[name.split('_')[0]][i],f['depth'][i],f['crop_intrinsics'][i]]
 with np.load(Path(dataset['root'])/'observations'/view/'actions_camera_v2.npz') as z:
  ids=z['candidate_ids'].tolist();features=z['features'].copy();x=np.c_[features,np.repeat(z['gravity_direction_camera'][None],len(ids),axis=0)];points=z['target_relative_waypoint_xyz_camera'].copy();phases=z['phase_names'].tolist()
 scores=score_candidates(ck,features,x[0,14:],obs)
 params={x['candidate_id']:x for x in json.loads((Path(dataset['source_run'])/'candidates.json').read_text())}
 return dict(target=target,view=view,model=name,seed=seed,score_definition='uncalibrated entry sigmoid; no displacement penalty',recommendations=[dict(candidate_id=ids[j],entry_score=float(scores[j]),azimuth_deg=params[ids[j]]['approach_azimuth_deg'],camera_action_14=features[j].tolist(),phase_names=phases,target_relative_waypoint_xyz_camera=points[j].tolist()) for j in np.argsort(-scores,kind='stable')[:top]])

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('training',type=Path);p.add_argument('--view',default='view_0000');p.add_argument('--top',type=int,default=5);a=p.parse_args();print(json.dumps(predict(a.training,a.view,a.top),indent=2))
