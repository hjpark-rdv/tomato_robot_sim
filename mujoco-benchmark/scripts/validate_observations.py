"""Offline integrity checks for observation/action coordinate pairing."""
import argparse,json,time
from concurrent.futures import ThreadPoolExecutor
from camera_action import pack_actions,batch_arrays
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation


def validate(root,workers=8):
    began=time.perf_counter()
    root=Path(root);rows=[json.loads(s) for s in (root/'observations.jsonl').read_text().splitlines()]
    actions=json.loads((root/'actions.json').read_text());lookup={a['candidate_id']:a for a in actions}
    packed=(pack_actions(actions) if any(len(a.get('waypoints_world',a.get('waypoints',[]))) in (4,5)
                                         for a in actions) else None)
    def check(row):
        max_pos=0.;max_rot=0.;checks=0
        f=root/'observations'/row['observation_id'];meta=json.loads((f/'camera.json').read_text());pose=np.array(meta['world_from_optical']['color']);k=np.array(meta['K'])
        rgb=np.array(Image.open(f/'rgb.png'));depth=np.load(f/'depth_aligned_to_color_m.npy')
        assert rgb.shape==(480,640,3) and depth.shape==(480,640)
        if (f/'aligned_valid.png').exists():
            valid=np.array(Image.open(f/'aligned_valid.png'))>0
            np.testing.assert_array_equal(np.isfinite(depth),valid)
        else:
            assert row['split']=='scene' and not (f/'depth_valid.png').exists()
        visible_path=f/'target_visible_mask.png';isolated_path=f/'target_isolated_mask_gt.png'
        assert visible_path.exists()==isolated_path.exists()
        if visible_path.exists():
            visible=np.array(Image.open(visible_path))>0;isolated=np.array(Image.open(isolated_path))>0
            assert not (visible&~isolated).any(),row['observation_id']
            assert visible.sum()==row['target']['visible_pixels']
        else:
            assert row['split']=='scene' and row['target']['visible_pixels']>=0
        point=(np.array(row['target']['center_world_gt'])-pose[:3,3])@pose[:3,:3]
        uv=k@point;uv=uv[:2]/uv[2];np.testing.assert_allclose(uv,row['target']['center_pixel_gt'],atol=1e-9)
        for crop in row['crops'].values():
            if crop['available']:
                x,y,_,_=crop['bounds_xyxy'];kc=np.array(crop['K']);p=kc@point
                np.testing.assert_allclose(p[:2]/p[2],uv-[x,y],atol=1e-9)
        if packed is None:
            assert row.get('action_poses_camera') is None
        else:
            is_v2=row.get('action_schema')=='farmily_camera_action_v2'
            labels=np.load(f/('actions_camera_v2.npz' if is_v2 else 'actions_camera.npz'))
            assert len(labels['candidate_ids'])==len(set(labels['candidate_ids']))
            if is_v2:
                expected,_=batch_arrays(packed,row['target']['center_world_gt'],pose)
                for key in expected:
                    if expected[key].dtype.kind in 'US':np.testing.assert_array_equal(labels[key],expected[key])
                    else:np.testing.assert_allclose(labels[key],expected[key],atol=1e-10)
                world_positions=labels['target_relative_waypoint_xyz_camera']@pose[:3,:3].T+row['target']['center_world_gt']
                world_rot=pose[:3,:3]@Rotation.from_quat(labels['features'][:,3:7]).as_matrix()
                max_rot=float(abs(world_rot-packed['rot']).max())
            else:
                np.testing.assert_array_equal(labels['candidate_ids'],packed['ids'])
                world_positions=labels['position_xyz']@pose[:3,:3].T+pose[:3,3]
                world_rot=pose[:3,:3]@Rotation.from_quat(labels['orientation_xyzw'].reshape(-1,4)).as_matrix().reshape(-1,len(packed['phases']),3,3)
                max_rot=float(abs(world_rot-packed['rot'][:,None]).max())
            max_pos=float(abs(world_positions-packed['xyz']).max());checks=len(packed['ids'])
            labels.close()
        if 'world_from_robot_tool' in meta:
            for stream in ('color','depth'):
                np.testing.assert_allclose(np.array(meta['world_from_robot_tool'])@np.array(meta['tool_from_optical'][stream]),meta['world_from_optical'][stream],atol=1e-10)
        assert abs(np.linalg.norm(np.array(meta['world_from_optical']['color'])[:3,3]-np.array(meta['world_from_optical']['depth'])[:3,3])-.015)<1e-10
        return max_pos,max_rot,checks
    with ThreadPoolExecutor(max_workers=workers) as pool:measurements=list(pool.map(check,rows))
    max_pos=max(v[0] for v in measurements);max_rot=max(v[1] for v in measurements);checks=sum(v[2] for v in measurements)
    assert max_pos<1e-10 and max_rot<1e-10
    saved=bool((root/'observations'/rows[0]['observation_id']/'target_visible_mask.png').exists())
    scope=('depth/crop/camera checks passed; no executable action paths' if packed is None else
           'passed' if saved else 'mask_files_not_saved; depth/crop/action checks passed')
    result=dict(wall_s=time.perf_counter()-began,workers=workers,observations=len(rows),observation_action_pairs_checked=checks,max_position_reconstruction_error_m=max_pos,max_rotation_matrix_reconstruction_error=max_rot,mask_depth_crop_and_baseline_checks=scope,mask_files_saved=saved)
    (root/'validation.json').write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();print(validate(a.root))
