"""Fixed RGB encoders and metric depth features. No outcome or world-pose inputs."""
import hashlib,json
from pathlib import Path
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from torchvision import transforms,models


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def extract(root,out,device,rows=None,backbone_store=None):
    if rows is None:rows=[json.loads(x) for x in (root/'observations.jsonl').read_text().splitlines()]
    store=Path(backbone_store) if backbone_store is not None else out
    store.mkdir(parents=True,exist_ok=True)
    assert all(r['target']['input_usable'] or r['split']=='visibility_test' for r in rows),'Unusable views must be isolated from training' 
    images=[];depths=[];geometry=[]
    transform=transforms.Compose([transforms.Resize((224,224)),transforms.ToTensor(),transforms.Normalize([.485,.456,.406],[.229,.224,.225])])
    hashes={}
    for r in rows:
        folder=root/'observations'/r['observation_id'];raw=np.load(folder/'depth_aligned_to_color_m.npy');dv=[];geo=[]
        for scale in ('local','context'):
            crop=r['crops'][scale];assert crop['available']
            path=folder/f'crop_{scale}_rgb.png';images.append(transform(Image.open(path).convert('RGB')));hashes[str(path.relative_to(root))]=sha(path)
            x,y,x1,y1=crop['bounds_xyxy'];z=raw[y:y1,x:x1];valid=np.isfinite(z)&(z>0)
            # Mask-aware metric depth pooling; never use preview/GT depth or extrinsics.
            mask=torch.from_numpy(valid.astype('float32'))[None,None]
            values=torch.from_numpy(np.where(valid,np.clip(z,0,2),0).astype('float32'))[None,None]
            den=F.adaptive_avg_pool2d(mask,(16,16));num=F.adaptive_avg_pool2d(values,(16,16))
            dv.extend([(num/den.clamp_min(1e-6)).flatten().numpy(),den.flatten().numpy()])
            k=np.array(crop['K']);h,w=z.shape
            geo.extend([k[0,0]/w,k[1,1]/h,k[0,2]/w,k[1,2]/h])
        hashes[str((folder/'depth_aligned_to_color_m.npy').relative_to(root))]=sha(folder/'depth_aligned_to_color_m.npy')
        depths.append(np.concatenate(dv));geometry.append(geo)
    batch=torch.stack(images);result={'depth':np.array(depths,dtype='float32'),'crop_intrinsics':np.array(geometry,dtype='float32')}
    torch.set_num_threads(4)
    for name in ('resnet18','dinov2'):
        if name=='resnet18':
            model=models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1);model.fc=torch.nn.Identity()
        else:model=torch.hub.load('facebookresearch/dinov2','dinov2_vits14',pretrained=True,trust_repo=True)
        model=model.to(device).eval();features=[]
        with torch.inference_mode():
            for start in range(0,len(batch),16):features.append(model(batch[start:start+16].to(device)).cpu().numpy())
        result[name]=np.concatenate(features).reshape(len(rows),-1)
        weights=store/f'{name}_backbone.pt'
        if not weights.exists():torch.save(model.cpu().state_dict(),weights)
        print('[영상 특징 완료]',name,result[name].shape,flush=True)
        del model
    result['view_ids']=np.array([r['observation_id'] for r in rows]);np.savez_compressed(out/'features.npz',**result)
    (out/'feature_manifest.json').write_text(json.dumps(dict(images=hashes,inputs='unannotated local/context RGB, aligned metric depth and validity, crop K; NO world pose, labels or candidate ID',rgb_resize=[224,224],depth_pool=[16,16],depth_clip_m=2,backbones={n:sha(store/f'{n}_backbone.pt') for n in ('resnet18','dinov2')}),indent=2))
    return result
