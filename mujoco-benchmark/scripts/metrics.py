"""Engine-independent capsule/sphere audit on every stored physics state.

Swept intersections are suspects, not certified tunneling: endpoint interpolation
cannot reconstruct the continuous constraint solution. Meshes are not certified.
"""
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nvidia-sim/rl'))
from connection_audit import segment_distance

def audit(reference,poses,times,contact_pairs):
    paths=[b['path'] for b in reference['bodies']]+[reference['tool_path']]
    selected=[s for s in reference['shapes'] if s['type'] in ('capsule','sphere')]
    selected.sort(key=lambda s:s['body']!=reference['tool_path'])
    tool=sum(s['body']==reference['tool_path'] for s in selected)
    ids=np.array([paths.index(s['body']) for s in selected]);rad=np.array([s['radius'] for s in selected])
    local=np.array([s['ends'] if s['type']=='capsule' else [s['center']]*2 for s in selected])
    rotations=R.from_quat(poses[:,:,3:].reshape(-1,4)).as_matrix().reshape(*poses.shape[:2],3,3)
    w=np.einsum('tnij,nkj->tnki',rotations[:,ids],local)+poses[:,ids,:3,None].transpose(0,1,3,2)
    worst=0.;worst_pair=None;violations=0;initial_pen=0.;suspects=[];undersampled=0
    for tick,ends in enumerate(w):
        prev=w[max(0,tick-1)]
        lo=np.minimum(ends.min(1),prev.min(1))-rad[:,None];hi=np.maximum(ends.max(1),prev.max(1))+rad[:,None]
        aa,bb=np.where(np.all((lo[:tool,None]<=hi[None,tool:])&(lo[None,tool:]<=hi[:tool,None]),axis=2));bb+=tool
        if not len(aa):continue
        def gaps(e):return segment_distance(e[aa,0],e[aa,1],e[bb,0],e[bb,1])-rad[aa]-rad[bb]
        g=gaps(ends);depth=max(0.,float(-g.min()))
        if tick==0:initial_pen=depth
        if depth>worst:
            worst=depth;k=g.argmin();worst_pair=[selected[aa[k]]['path'],selected[bb[k]]['path']]
        violations+=int(depth>.0005)
        if tick:
            delta=max(np.linalg.norm(ends[aa]-prev[aa],axis=-1).max(),np.linalg.norm(ends[bb]-prev[bb],axis=-1).max())
            n=max(2,int(np.ceil(delta/.0005)))
            if n>32:undersampled+=1;n=32
            old=gaps(prev);cross=np.zeros(len(g),bool)
            for u in np.arange(1,n)/n:cross|=(gaps(prev*(1-u)+ends*u)<-.0005)&(g>=0)&(old>=0)
            for k in np.flatnonzero(cross):
                pair=[selected[aa[k]]['name'],selected[bb[k]]['name']]
                known={tuple(sorted(p)) for p in contact_pairs[tick-1]}
                if tuple(sorted(pair)) not in known:
                    suspects.append(dict(time_s=float(times[tick]),pair=pair,kind='swept_intersection_without_reported_contact'))
    rest=np.array([b['pose'][:3] for b in reference['bodies']]);disp=np.linalg.norm(poses[:,:-1,:3]-rest[None],axis=-1)
    groups=dict(tomato=[i for i,p in enumerate(paths[:-1]) if '/Harvestables/Tomato_' in p],
        main_stem=[i for i,p in enumerate(paths[:-1]) if '/STEM_MainStem_' in p],pedicel=[i for i,p in enumerate(paths[:-1]) if '/TRUSS_Pedicel_proximal_' in p])
    result={}
    for group,idx in groups.items():
        series=disp[:,idx].max(axis=1);above=np.where(series>.001)[0]
        result.update({group+'_max_displacement_m':float(series.max()),group+'_residual_m':float(series[-1]),
            group+'_time_last_above_1mm_s':float(times[above[-1]]) if len(above) else 0.})
    anchor_ids=[paths.index(s['anchor']) for s in reference['fruit_specs']]
    fruit_ids=[paths.index(s['path']) for s in reference['fruit_specs']]
    attachment=np.linalg.norm(poses[:,anchor_ids,:3]-poses[:,fruit_ids,:3],axis=-1)
    centers=np.array([s['center'] for s in reference['fruit_specs']])
    fruit_centers=poses[:,fruit_ids,:3]+np.einsum('tnij,nj->tni',rotations[:,fruit_ids],centers)
    rest_poses=np.array([b['pose'] for b in reference['bodies']])[fruit_ids]
    rest_centers=rest_poses[:,:3]+R.from_quat(rest_poses[:,[4,5,6,3]]).apply(centers)
    fruit_disp=np.linalg.norm(fruit_centers-rest_centers[None],axis=-1)
    target=next(i for i,s in enumerate(reference['fruit_specs']) if s['name']=='Tomato_05')
    result.update(tomato_max_displacement_m=float(fruit_disp.max()),tomato_residual_m=float(fruit_disp[-1].max()),
        target_max_displacement_m=float(fruit_disp[:,target].max()),target_residual_m=float(fruit_disp[-1,target]),
        max_attachment_gap_m=float(attachment.max()),attachment_valid=bool(attachment.max()<=.0005))
    result.update(max_penetration_m=worst,initial_penetration_m=initial_pen,worst_pair=worst_pair,
        penetration_steps=violations,tunneling_suspect_count=len(suspects),tunneling_suspects=suspects[:20],
        sweep_sampling_capped_steps=undersampled,audit_scope='capsules/spheres only, every physics state + interpolated sweep; mesh/housing not certified',
        contact_steps=sum(bool(p) for p in contact_pairs),contact_pairs=sorted({tuple(sorted(p)) for pairs in contact_pairs for p in pairs}),
        finite=bool(np.isfinite(poses).all()))
    result['result']='initial_overlap' if initial_pen>.0005 else 'invalid_penetration' if violations else 'invalid_attachment' if not result['attachment_valid'] else 'tunneling_suspect' if suspects else 'contact_no_detected_penetration' if result['contact_steps'] else 'miss'
    return result
