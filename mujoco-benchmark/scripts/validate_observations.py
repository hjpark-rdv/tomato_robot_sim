"""Offline integrity checks for observation/action coordinate pairing."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation


def validate(root):
    root=Path(root);rows=[json.loads(s) for s in (root/'observations.jsonl').read_text().splitlines()]
    actions=json.loads((root/'actions.json').read_text());lookup={a['candidate_id']:a for a in actions}
    max_pos=0.;max_rot=0.;checks=0
    for row in rows:
        f=root/'observations'/row['observation_id'];meta=json.loads((f/'camera.json').read_text());pose=np.array(meta['world_from_optical']['color']);k=np.array(meta['K'])
        rgb=np.array(Image.open(f/'rgb.png'));depth=np.load(f/'depth_aligned_to_color_m.npy');valid=np.array(Image.open(f/'aligned_valid.png'))>0
        assert rgb.shape==(480,640,3) and depth.shape==(480,640)
        np.testing.assert_array_equal(np.isfinite(depth),valid)
        visible=np.array(Image.open(f/'target_visible_mask.png'))>0;isolated=np.array(Image.open(f/'target_isolated_mask_gt.png'))>0
        assert not (visible&~isolated).any(),row['observation_id']
        assert visible.sum()==row['target']['visible_pixels']
        point=(np.array(row['target']['center_world_gt'])-pose[:3,3])@pose[:3,:3]
        uv=k@point;uv=uv[:2]/uv[2];np.testing.assert_allclose(uv,row['target']['center_pixel_gt'],atol=1e-9)
        for crop in row['crops'].values():
            if crop['available']:
                x,y,_,_=crop['bounds_xyxy'];kc=np.array(crop['K']);p=kc@point
                np.testing.assert_allclose(p[:2]/p[2],uv-[x,y],atol=1e-9)
        is_v2=row.get('action_schema')=='farmily_camera_action_v2'
        labels=np.load(f/('actions_camera_v2.npz' if is_v2 else 'actions_camera.npz'))
        assert len(labels['candidate_ids'])==len(set(labels['candidate_ids']))
        if is_v2:
            from camera_action import encode,FEATURE_NAMES
            assert list(labels['feature_names'])==FEATURE_NAMES
            world_positions=labels['target_relative_waypoint_xyz_camera']@pose[:3,:3].T+row['target']['center_world_gt']
            camera_rot=Rotation.from_quat(labels['features'][:,3:7]).as_matrix()
            world_rot=np.repeat((pose[:3,:3]@camera_rot)[:,None],4,axis=1)
            np.testing.assert_allclose(pose[:3,:3]@labels['gravity_direction_camera'],[0,0,-1],atol=1e-10)
            for j,cid in enumerate(labels['candidate_ids']):
                action=lookup[str(cid)]
                expected=encode(action['waypoints_world'],action['parameters'],row['target']['center_world_gt'],pose)
                np.testing.assert_allclose(labels['features'][j],expected['features'],atol=1e-10)
        else:
            world_positions=labels['position_xyz']@pose[:3,:3].T+pose[:3,3]
            world_rot=pose[:3,:3]@Rotation.from_quat(labels['orientation_xyzw'].reshape(-1,4)).as_matrix().reshape(-1,4,3,3)
        for i,cid in enumerate(labels['candidate_ids']):
            orig=lookup[str(cid)]['waypoints_world'];p=np.array([w['position_xyz'] for w in orig]);r=Rotation.from_quat([w['orientation_xyzw'] for w in orig]).as_matrix()
            max_pos=max(max_pos,float(abs(p-world_positions[i]).max()));max_rot=max(max_rot,float(abs(r-world_rot[i]).max()));checks+=1
        if 'world_from_robot_tool' in meta:
            for stream in ('color','depth'):
                np.testing.assert_allclose(np.array(meta['world_from_robot_tool'])@np.array(meta['tool_from_optical'][stream]),meta['world_from_optical'][stream],atol=1e-10)
        assert abs(np.linalg.norm(np.array(meta['world_from_optical']['color'])[:3,3]-np.array(meta['world_from_optical']['depth'])[:3,3])-.015)<1e-10
    assert max_pos<1e-10 and max_rot<1e-10
    result=dict(observations=len(rows),observation_action_pairs_checked=checks,max_position_reconstruction_error_m=max_pos,max_rotation_matrix_reconstruction_error=max_rot,mask_depth_crop_and_baseline_checks='passed')
    (root/'validation.json').write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();print(validate(a.root))
