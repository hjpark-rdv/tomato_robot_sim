#!/usr/bin/env python3
"""Offline USD skin-update comparison; does not start Isaac or physics.

Reports Python/Usd timing and change-notice count only.  Renderer/PhysX speed
must be measured separately by the native runner using identical trajectories.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from pxr import Gf, Tf, Usd, UsdGeom, Vt
from scipy.spatial.transform import Rotation

from motion_visuals import DeformationVisuals, prepare_display_meshes
from elastic_geometry import skin_points
from motion_kinematics import transmission_transforms


def benchmark(assembly,trial,frames):
    source=assembly/'scene.usda';source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    stage=Usd.Stage.Open(str(source))
    metadata=json.loads((assembly/'motion_manifest.json').read_text())
    data=np.load(trial/'states.npz');paths=data['body_paths'].tolist()
    index={path:i for i,path in enumerate(paths)}
    names=json.loads((trial/'native_metadata.json').read_text())['robot_dofs']
    jaws=[names.index('jaw_L'),names.index('jaw_R')]
    ticks=np.linspace(0,len(data['poses'])-1,frames,dtype=int)
    old=[]
    with np.load(assembly/'skin.npz') as skins:
        for i,path in enumerate(metadata['plant']['skin_paths']):
            old.append((UsdGeom.Mesh(stage.GetPrimAtPath(path)),skins[f'{i}_vertices'],
                        skins[f'{i}_ids'],skins[f'{i}_weights'],skins[f'{i}_rest'],
                        np.array([index[p] for p in skins[f'{i}_body_paths']])))
    new=DeformationVisuals(stage,assembly,paths)
    notices=[]
    subscription=Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                   lambda event,sender:notices.append(1),stage)
    old_elapsed=[];old_notices=[]
    for tick in ticks:
        poses=data['poses'][tick];gap=.044+float(data['robot_q'][tick,jaws[1]]-data['robot_q'][tick,jaws[0]])
        rotations=Rotation.from_quat(poses[:,3:7]).as_matrix();before=len(notices);started=time.perf_counter()
        for mesh,v,ids,weights,rest,body_ids in old:
            points=skin_points(v,rest,rotations[body_ids],poses[body_ids,:3],ids,weights)
            mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
        for name,transform in transmission_transforms(gap).items():
            new.transmission[name].Set(Gf.Matrix4d(transform.T.tolist()))
        old_elapsed.append(time.perf_counter()-started);old_notices.append(len(notices)-before)
    removed=prepare_display_meshes(stage)
    new_elapsed=[];new_notices=[];details=[]
    for tick in ticks:
        poses=data['poses'][tick];gap=.044+float(data['robot_q'][tick,jaws[1]]-data['robot_q'][tick,jaws[0]])
        before=len(notices);started=time.perf_counter();details.append(new.update(poses,gap))
        new_elapsed.append(time.perf_counter()-started);new_notices.append(len(notices)-before)
    subscription.Revoke()
    same=hashlib.sha256(source.read_bytes()).hexdigest()==source_hash
    if not same:raise RuntimeError('Original asset changed during offline display benchmark')
    def stats(values):
        return dict(median_s=float(np.median(values)),minimum_s=float(np.min(values)),
                    maximum_s=float(np.max(values)),total_s=float(np.sum(values)))
    return dict(assembly=str(assembly),measured_trial=str(trial),source_scene_sha256=source_hash,
                source_asset_unchanged=same,frames=frames,skin_meshes=len(old),
                skin_vertices=sum(len(item[1]) for item in old),
                original=dict(timing=stats(old_elapsed),usd_notices=old_notices),
                batched=dict(timing=stats(new_elapsed),usd_notices=new_notices,per_frame_metrics=details),
                dormant_display_schemas_removed=removed,
                interpretation='Offline CPU/Usd update cost and notice count only; no renderer or PhysX was running')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('assembly',type=Path);parser.add_argument('trial',type=Path)
    parser.add_argument('--frames',type=int,default=30);parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.frames<1:parser.error('--frames must be positive')
    result=benchmark(args.assembly,args.trial,args.frames)
    payload=json.dumps(result,indent=2)
    if args.output:
        with args.output.open('x') as stream:stream.write(payload+'\n')
    print(json.dumps({key:value for key,value in result.items()
                      if key!='dormant_display_schemas_removed'},indent=2))
