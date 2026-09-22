"""Rank saved camera-frame candidates for an existing observation; no outcome inputs."""
import argparse,json
from pathlib import Path
import numpy as np
import torch
from train import Scorer


def predict(run,view,model_name,seed=0,scope='test',top=5):
    run=Path(run);manifest=json.loads((run/'manifest.json').read_text());root=Path(manifest['dataset'])
    # Only load self-generated, trusted checkpoints (contains NumPy normalization arrays).
    ckpt=torch.load(run/f'{model_name}_seed{seed}'/'model.pt',map_location='cpu',weights_only=False)
    model=Scorer(ckpt['obs_dim']);model.load_state_dict(ckpt['state_dict']);model.eval()
    with np.load(run/'features.npz') as feat:
        vi=feat['view_ids'].tolist().index(view)
        o=np.zeros(0,dtype='float32') if model_name=='action_only' else np.concatenate([feat[model_name.split('_')[0]][vi],feat['depth'][vi],feat['crop_intrinsics'][vi]])
    with np.load(root/'observations'/view/'actions_camera_v2.npz') as actions:
        ids=actions['candidate_ids'].tolist();selected=set(manifest['action_splits'][scope]) if scope!='all' else set(ids);indices=[i for i,c in enumerate(ids) if c in selected]
        raw=actions['features'][indices];x=np.c_[raw,np.repeat(actions['gravity_direction_camera'][None],len(indices),0)]
        a=torch.from_numpy(((x-ckpt['action_mean'])/ckpt['action_std']).astype('float32'));o=torch.from_numpy(((o-ckpt['observation_mean'])/ckpt['observation_std']).astype('float32'))
        with torch.no_grad():pred=model(a,o.expand(len(indices),-1));p=pred[:,0].sigmoid().numpy();d=pred[:,1].clamp_min(0).numpy();score=p*np.exp(-d)
        order=np.argsort(-score)[:top]
        recommendations=[dict(candidate_id=ids[indices[i]],predicted_entry_probability=float(p[i]),predicted_max_displacement_mm=float(d[i]*20),score=float(score[i]),camera_action_14=raw[i].tolist(),target_relative_waypoint_xyz_camera=actions['target_relative_waypoint_xyz_camera'][indices[i]].tolist()) for i in order]
    return dict(model=model_name,seed=seed,view=view,candidate_scope=scope,coordinate_frame='color optical: right/down/forward; positions relative to initial target',recommendations=recommendations,scope='offline known-scene candidate ranking; no physical execution or hook-success claim')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--view',required=True);p.add_argument('--model',choices=['action_only','resnet18_rgbd','dinov2_rgbd'],default='dinov2_rgbd');p.add_argument('--seed',type=int,default=0);p.add_argument('--scope',choices=['train','validation','test','all'],default='test');p.add_argument('--top',type=int,default=5);p.add_argument('--output',type=Path);a=p.parse_args()
    result=predict(a.run,a.view,a.model,a.seed,a.scope,a.top);s=json.dumps(result,indent=2)
    if a.output:a.output.write_text(s)
    print(s)
