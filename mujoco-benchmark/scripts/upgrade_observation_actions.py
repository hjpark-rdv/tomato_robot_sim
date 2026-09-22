"""Add camera-centred v2 action inputs without re-rendering observations."""
import argparse,datetime,hashlib,json,shutil,time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from camera_action import pack_actions,batch_arrays,SCHEMA,FEATURE_NAMES


def upgrade(root,run=None,workers=8):
    began=time.perf_counter()
    root=Path(root).resolve();meta=json.loads((root/'dataset.json').read_text())
    rows=[json.loads(s) for s in (root/'observations.jsonl').read_text().splitlines()]
    backup=root/'schema_v1_backup'
    if not backup.exists():
        backup.mkdir()
        for name in ('dataset.json','actions.json','observations.jsonl','validation.json'):
            if (root/name).exists():shutil.copy2(root/name,backup/name)
    if run:
        run=Path(run).resolve();manifest=json.loads((run/'manifest.json').read_text());results=json.loads((run/'results.json').read_text())
        if manifest['target']!=meta['target']:raise RuntimeError('Target differs; do not reuse observations')
        if len(results)!=manifest['count']:raise RuntimeError('Only a fully completed run can be linked')
        if manifest['model_sha256']!=meta['source_model_sha256']:raise RuntimeError('Scene model differs; do not reuse images')
        first=next(r for r in results if (run/'candidates'/r['candidate_id']/'states.npz').exists())
        with np.load(run/'candidates'/first['candidate_id']/'states.npz') as s,np.load(root/'initial_state.npz') as initial:np.testing.assert_array_equal(s['qpos'][0],initial['qpos'])
        actions=[]
        for r in sorted(results,key=lambda r:r['candidate_id']):
            item={k:r.get(k) for k in ('candidate_id','parameters','result','center_entered','target_center_max_displacement_m','trace_sha256','states_sha256')}
            item.update(waypoints_world=r.get('waypoints',[]),hook_success=None,source_candidate_directory=str(run/'candidates'/r['candidate_id']))
            actions.append(item)
    else:
        actions=json.loads((root/'actions.json').read_text());run=Path(meta['source_run'])
    packed=pack_actions(actions)
    def process(row):
        folder=root/'observations'/row['observation_id'];pose=np.array(json.loads((folder/'camera.json').read_text())['world_from_optical']['color'])
        arrays,error=batch_arrays(packed,row['target']['center_world_gt'],pose)
        temporary=folder/'actions_camera_v2.tmp.npz';np.savez_compressed(temporary,**arrays);temporary.replace(folder/'actions_camera_v2.npz')
        row.update(action_schema=SCHEMA,action_features_camera='actions_camera_v2.npz',primary_action_input='14 compact camera-axis features; no absolute approach-start position',gravity_direction_camera=arrays['gravity_direction_camera'].tolist(),legacy_action_poses_camera='actions_camera.npz')
        row.pop('action_poses_camera',None)
        (folder/'observation.json').write_text(json.dumps(row,indent=2))
        return error
    with ThreadPoolExecutor(max_workers=workers) as pool:errors=list(pool.map(process,rows))
    paired=len(rows)*len(packed['ids']);max_error=max(errors)
    print('[카메라 좌표 갱신 완료]',len(rows),'시점 /',workers,'작업자 /',round(time.perf_counter()-began,2),'초',flush=True)
    for name in ('manifest.json','planning_inputs.json'):
        shutil.copy2(run/name,root/('source_'+name))
    (root/'actions.json').write_text(json.dumps(actions,indent=2))
    with (root/'observations.jsonl').open('w') as f:
        for row in rows:f.write(json.dumps(row)+'\n')
    meta.update(schema='farmily_observation_v2',action_schema=SCHEMA,source_run=str(run),source_results_sha256=hashlib.sha256((run/'results.json').read_bytes()).hexdigest(),actions=len(actions),primary_action_input='actions_camera_v2.npz:features',feature_names=FEATURE_NAMES,model_input_origin='target fruit center; vector axes=color optical',legacy_waypoints='actions_camera.npz retained from v1 for inspection only; use v2 target-relative waypoints',v2_validation=dict(observation_action_pairs=paired,max_world_position_error_m=max_error,directions_and_orientations='passed'),updated_at=datetime.datetime.now().isoformat(),action_processing_wall_s=time.perf_counter()-began,postprocess_workers=workers)
    (root/'dataset.json').write_text(json.dumps(meta,indent=2))
    for name in ('camera_action.py','upgrade_observation_actions.py'):
        shutil.copy2(Path(__file__).with_name(name),root/name.replace('.py','_source.py'))
    (root/'action_schema.json').write_text(json.dumps(dict(schema=SCHEMA,feature_names=FEATURE_NAMES,units=['unit vector']*3+['quaternion xyzw']*4+['m']*3+['unit vector']*3+['m'],gravity='shared per observation, unit direction in camera axes',position='target-relative waypoint offsets in camera axes; optional geometry, not 14-feature input',legacy_parameters='world-axis Sobol parameters for reproduction only'),indent=2))
    return meta['v2_validation']

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('observations',type=Path);p.add_argument('--run',type=Path);p.add_argument('--workers',type=int,default=8);a=p.parse_args();print(upgrade(a.observations,a.run,a.workers))
