"""Deterministic mechanical fixtures and replay of existing measured joint logs."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R,Slerp
HOME=Path(__file__).resolve().parents[1]
RING=np.array([-.10640287,.00616804,.00006305])

def resample(trial,hz,recovery=2.):
    poses=np.asarray(trial['poses']);times=np.asarray(trial['times_s'] if 'times_s' in trial else np.arange(len(poses))*trial['sample_dt'])
    n=int(np.ceil((times[-1]+recovery)*hz));ts=np.arange(n+1)/hz
    sample=np.clip(ts,times[0],times[-1]);rot=Slerp(times,R.from_quat(poses[:,3:]))(sample)
    pos=np.column_stack([np.interp(sample,times,poses[:,k]) for k in range(3)])
    return ts,np.column_stack([pos,rot.as_quat()])

def main():
    p=argparse.ArgumentParser();p.add_argument('--reference',type=Path,default=HOME/'assets/reference/reference.json');p.add_argument('--output',type=Path,default=HOME/'assets/reference/suite.json');p.add_argument('--recordings',type=Path);a=p.parse_args()
    d=json.loads(a.reference.read_text());bodies={b['path']:b for b in d['bodies']}
    def center(name):
        s=next(s for s in d['shapes'] if s['path'].endswith(name));b=bodies[s['body']];pos=np.asarray(b['pose']);rot=R.from_quat(pos[[4,5,6,3]])
        pt=np.asarray(s.get('center',np.mean(s.get('ends',[[0,0,0]]),axis=0)))
        return pos[:3]+rot.apply(pt)
    fruit=center('Tomato_05/FruitCollider');ped=center('TRUSS_Pedicel_proximal_05_02/StemCollider');stem=center('STEM_MainStem_12/StemCollider')
    q=R.identity().as_quat();trials=[]
    # Desired point below is the rear rim, ring centre is 27.5 mm along +X.
    for label,point,delta in [('miss',fruit+[0,.18,0],.005),('fruit_collision',fruit,.016),('pedicel_contact',ped,.008),('stem_push',stem,.018)]:
        start=point+[.0275+.001+.012,0,0];end=start+[-(.012+delta),0,0]
        pts=[start,start,end,end,start,start];times=[0,1,3,5,7,10]
        trials.append(dict(id='fixture_'+label,category=label,source='geometry-derived deterministic fixture, not recorded robot motion',
            times_s=times,sample_dt=1.,poses=[[*v,*q] for v in pts],duration_s=10.,release_complete_s=7.,frame='ring_center, world XYZ + xyzw'))
    # Approach from outside the fruit, insert underneath then lift and withdraw.
    pts=[fruit+[.12,0,-.025],fruit+[.05,0,-.025],fruit+[.005,0,-.025],fruit+[.005,0,.015],fruit+[.06,0,.015],fruit+[.12,0,.015]]
    trials.append(dict(id='fixture_entry_lift',category='entry_lift',source='deterministic staged path',sample_dt=2.,poses=[[*v,*q] for v in pts],duration_s=10.,release_complete_s=10.))
    start=stem+[.10,0,0];end=stem+[.015,0,0]
    trials.append(dict(id='fixture_large_displacement',category='large_displacement',source='deterministic overload fixture, not a harvest policy',sample_dt=2.,poses=[[*v,*q] for v in [start,start,end,end,start,start]],duration_s=10.,release_complete_s=8.))
    if a.recordings:
        trials+=json.loads((a.reference.parent/'trajectories.json').read_text())
        fk=d['robot_fk']
        def forward(joints):
            t=np.array(fk['world'])
            for origin,axis,kind,index in fk['chain']:
                t=t@origin;axis=np.array(axis)
                if index is not None:
                    if kind=='prismatic':t[:3,3]+=t[:3,:3]@(axis*joints[index])
                    elif kind in ('revolute','continuous'):t[:3,:3]=t[:3,:3]@R.from_rotvec(axis*joints[index]).as_matrix()
            return [*(t[:3,3]+t[:3,:3]@RING),*R.from_matrix(t[:3,:3]).as_quat()]
        recorded=[]
        for folder in (a.recordings/'results').glob('candidate_*'):
            if (folder/'trace.json').exists():
                rows=json.loads((folder/'trace.json').read_text())
                if len(rows)>1:recorded.append((len(rows),folder,rows))
        # The pilot includes short, complete recorded prefixes first. Preserve
        # their original speed and start; never splice into an in-contact state.
        for _,folder,rows in sorted(recorded,key=lambda x:(x[0],str(x[1]))):
            if len(trials)>=100:break
            if folder.name in {t['id'] for t in trials}:continue
            if not (folder/'trace.json').exists():continue
            c=json.loads((folder/'candidate.json').read_text())
            if len(rows)<2 or c['target_id']!='Tomato_05':continue
            dt=c['physical_inputs']['control_dt']
            trials.append(dict(id=folder.name,category='recorded_prefix',source=str(folder),sample_dt=dt,
                poses=[forward(r['joints']) for r in rows],duration_s=(len(rows)-1)*dt,
                scope='entire available recorded prefix at original speed; source stopping condition is not extended'))
    else:trials+=json.loads((a.reference.parent/'trajectories.json').read_text())
    a.output.write_text(json.dumps(trials));print('[경로 묶음]',len(trials),a.output)

if __name__=='__main__':main()
