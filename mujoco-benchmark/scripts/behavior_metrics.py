"""Read-only postprocessing of body centre motion and recovery, no re-simulation."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
HOME=Path(__file__).resolve().parents[1]

def enrich(root):
    ref=json.loads((HOME/'assets/reference/reference.json').read_text());paths=[b['path'] for b in ref['bodies']];spec=next(s for s in ref['fruit_specs'] if s['name']=='Tomato_05');idx=paths.index(spec['path'])
    rest=np.asarray(spec['pose']);origin=rest[:3]+R.from_quat(rest[[4,5,6,3]]).apply(spec['center'])
    releases=dict(fixture_miss=7.,fixture_fruit_collision=7.,fixture_pedicel_contact=7.,fixture_stem_push=7.,fixture_entry_lift=10.,fixture_large_displacement=8.)
    shapes={s['name']:s for s in ref['shapes']}
    for f in Path(root).glob('*/summary.json'):
        summary=json.loads(f.read_text())
        for row in summary['results']:
            folder=f.parent/row['trajectory_id'];data=np.load(folder/'states.npz');pose=data['poses'][:,idx];times=data['times_s']
            xyz=pose[:,:3]+R.from_quat(pose[:,3:]).apply(spec['center']);delta=xyz-origin;distance=np.linalg.norm(delta,axis=1);peak=int(distance.argmax());release=releases.get(row['trajectory_id'])
            metrics=dict(target_peak_displacement_xyz_m=delta[peak].tolist(),target_final_displacement_xyz_m=delta[-1].tolist(),
                target_recovery_fraction=float(1-distance[-1]/max(1e-12,distance[peak])),release_complete_s=release,
                recovery_scope='fixture withdrawal only; recorded prefixes hold last pose and are not a free recovery experiment',
                settling_threshold_m=max(.001,.1*float(distance.max())),settling_after_release_s=None,settling_censored=None)
            contacts=json.loads((folder/'contacts.json').read_text());first=next((pairs[0] for pairs in contacts if pairs),[])
            metrics['first_contact_objects']=[shapes[g]['path'] for g in first]
            metrics['first_contact_object']=next((shapes[g]['path'] for g in first if shapes[g]['body']!=ref['tool_path']),None)
            if release is not None:
                tail=np.where(times>=release)[0];threshold=metrics['settling_threshold_m']
                outside=tail[distance[tail]>threshold]
                metrics['settling_censored']=bool(len(tail)==0 or distance[-1]>threshold)
                if not metrics['settling_censored']:
                    metrics['settling_after_release_s']=float(max(0.,times[outside[-1]]-release)) if len(outside) else 0.
                metrics['oscillation_duration_proxy_s']=metrics['settling_after_release_s']
            row.update(metrics);(folder/'result.json').write_text(json.dumps(row,indent=2))
        f.write_text(json.dumps(summary,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();enrich(a.root)
