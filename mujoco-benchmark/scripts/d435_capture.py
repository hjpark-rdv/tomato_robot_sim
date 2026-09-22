"""ROS/Isaac URDF-mounted D435 RGB + metric depth from saved MuJoCo states."""
import render_backend  # configure EGL before importing MuJoCo
import argparse,datetime,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco as mj
import imageio.v2 as imageio
HOME=Path(__file__).resolve().parents[1]
URDF=HOME.parent/'nvidia-sim/robot_usd/rb5_farmily.urdf'


from camera_action import optical_transform


def align_depth(depth,valid,k,world_depth,world_color):
    yy,xx=np.where(valid);z=depth[yy,xx]
    p=np.stack([(xx-k[0,2])*z/k[0,0],(yy-k[1,2])*z/k[1,1],z],axis=1)
    t=np.linalg.inv(world_color)@world_depth;p=p@t[:3,:3].T+t[:3,3]
    keep=p[:,2]>0;p=p[keep]
    uv=p@k.T;uv=np.rint(uv[:,:2]/uv[:,2:]).astype(int)
    h,w=depth.shape;keep=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    out=np.full(h*w,np.inf,dtype=np.float32)
    np.minimum.at(out,uv[keep,1]*w+uv[keep,0],p[keep,2])
    out=out.reshape(h,w);mask=np.isfinite(out);out[~mask]=np.nan
    return out,mask


def capture(model,data,destination,width=640,height=480,world_from_color=None,target_geom_ids=None):
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=False)
    hfov=69.4;near=.10;far=20.;f=width/(2*np.tan(np.deg2rad(hfov/2)))
    k=np.array([[f,0,width/2],[0,f,height/2],[0,0,1.]])
    cam=0;hook=model.body('Hook').id
    # Camera slots only: no body, mass, contact, qpos or control modifications.
    model.cam_bodyid[cam]=hook;model.cam_mode[cam]=mj.mjtCamLight.mjCAMLIGHT_FIXED
    model.cam_fovy[cam]=np.rad2deg(2*np.arctan(height/(2*f)))
    model.vis.map.znear=near/model.stat.extent;model.vis.map.zfar=far/model.stat.extent
    model.vis.global_.offwidth=max(width,model.vis.global_.offwidth);model.vis.global_.offheight=max(height,model.vis.global_.offheight)
    # Segmentation IDs must not be blended by multisample antialiasing.
    if target_geom_ids is not None:model.vis.quality.offsamples=0
    worlds={};extrinsics={};mj.mj_forward(model,data)
    tool=np.eye(4);tool[:3,:3]=data.xmat[hook].reshape(3,3);tool[:3,3]=data.xpos[hook]
    actual_tool=tool.copy()
    virtual=world_from_color is not None
    if virtual:
        tool=np.asarray(world_from_color)@np.linalg.inv(optical_transform('color'))
        model.cam_bodyid[cam]=0
    conversion=np.diag([1.,-1.,-1.])
    with mj.Renderer(model,height=height,width=width) as renderer:
        for stream in ('color','depth'):
            local=optical_transform(stream);extrinsics[stream]=local.tolist();worlds[stream]=tool@local
            render_pose=worlds[stream] if virtual else local
            model.cam_pos[cam]=render_pose[:3,3];xyzw=Rotation.from_matrix(render_pose[:3,:3]@conversion).as_quat();model.cam_quat[cam]=xyzw[[3,0,1,2]]
            mj.mj_forward(model,data)
            np.testing.assert_allclose(data.cam_xpos[cam],worlds[stream][:3,3],atol=1e-10)
            np.testing.assert_allclose(data.cam_xmat[cam].reshape(3,3),worlds[stream][:3,:3]@conversion,atol=1e-10)
            renderer.update_scene(data,camera=cam)
            if stream=='color':
                imageio.imwrite(destination/'rgb.png',renderer.render())
                if target_geom_ids is not None:
                    renderer.enable_segmentation_rendering()
                    seg=renderer.render().copy()
                    visible=(seg[:,:,1]==int(mj.mjtObj.mjOBJ_GEOM))&np.isin(seg[:,:,0],target_geom_ids)
                    imageio.imwrite(destination/'target_visible_mask.png',visible.astype(np.uint8)*255)
                    # Isolated render is only for GT occlusion measurement, never an input RGB.
                    rgba=model.geom_rgba.copy()
                    try:
                        model.geom_rgba[:,3]=0
                        model.geom_rgba[target_geom_ids,3]=rgba[target_geom_ids,3]
                        renderer.update_scene(data,camera=cam)
                        seg=renderer.render().copy()
                        isolated=(seg[:,:,1]==int(mj.mjtObj.mjOBJ_GEOM))&np.isin(seg[:,:,0],target_geom_ids)
                        imageio.imwrite(destination/'target_isolated_mask_gt.png',isolated.astype(np.uint8)*255)
                    finally:
                        model.geom_rgba[:]=rgba
                        renderer.disable_segmentation_rendering()
                        renderer.update_scene(data,camera=cam)

            else:
                renderer.enable_depth_rendering();depth=renderer.render().copy().astype(np.float32)
                valid=np.isfinite(depth)&(depth>=near)&(depth<far*.9999);depth[~valid]=np.nan
                np.save(destination/'depth_m.npy',depth);imageio.imwrite(destination/'depth_valid.png',valid.astype(np.uint8)*255)
                preview=np.zeros((height,width),np.uint8);preview[valid]=np.clip(255*(1-depth[valid]/1.),0,255).astype(np.uint8)
                imageio.imwrite(destination/'depth_preview.png',preview)
    aligned,mask=align_depth(depth,valid,k,worlds['depth'],worlds['color'])
    np.save(destination/'depth_aligned_to_color_m.npy',aligned);imageio.imwrite(destination/'aligned_valid.png',mask.astype(np.uint8)*255)
    metadata=dict(virtual_camera_augmentation=virtual,resolution=[width,height],K=k.tolist(),hfov_deg=hfov,clipping_m=[near,far],optical_axes='x right, y down, z forward',depth_units='metres; optical-axis Z; invalid NaN',depth_alignment='native depth reprojected to color with nearest pixel z-buffer; holes invalid',world_from_optical={s:v.tolist() for s,v in worlds.items()},tool_from_optical=extrinsics,urdf=str(URDF),urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),intrinsics_source='matches run_farm_simulation.py approximate 69.4deg HFOV; not hardware calibration',sensor_model='ideal pinhole geometry; no RealSense stereo noise/minimum-range simulation',valid_depth_fraction=float(valid.mean()),mujoco=mj.__version__)
    metadata['world_from_robot_tool']=actual_tool.tolist()
    metadata['nominal_tool_from_optical']=extrinsics
    metadata['tool_from_optical']={stream:(np.linalg.inv(actual_tool)@world).tolist() for stream,world in worlds.items()}
    (destination/'camera.json').write_text(json.dumps(metadata,indent=2))
    return metadata


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--candidate',required=True);p.add_argument('--phase',choices=['initial','preapproach','entry','insert','rise'],default='preapproach');p.add_argument('--time',type=float);p.add_argument('--output',type=Path);a=p.parse_args()
    root=a.run.resolve();folder=root/'candidates'/a.candidate
    if folder.parent!=root/'candidates':raise ValueError('invalid candidate id')
    manifest=json.loads((root/'manifest.json').read_text());modelpath=root/'replay_assets/model.mjb'
    if manifest['mujoco']!=mj.__version__:raise RuntimeError('Use matching MuJoCo version')
    if hashlib.sha256(modelpath.read_bytes()).hexdigest()!=manifest['model_sha256']:raise RuntimeError('model hash mismatch')
    result=json.loads((folder/'result.json').read_text());statepath=folder/'states.npz'
    if result.get('states_sha256') and hashlib.sha256(statepath.read_bytes()).hexdigest()!=result['states_sha256']:raise RuntimeError('state hash mismatch')
    state=np.load(statepath);times=state['times_s'];requested=a.time
    if requested is None:
        requested=0.
        if a.phase!='initial':
            found=False
            for w in json.loads((folder/'plan.json').read_text())['waypoints']:
                requested+=w['steps']/60
                if w['phase']==a.phase:found=True;break
            if not found:raise ValueError('phase not in plan')
    if requested<0 or requested>times[-1]+1e-6:raise ValueError('time outside recorded states')
    index=int(np.argmin(abs(times-requested)));m=mj.MjModel.from_binary_path(str(modelpath));d=mj.MjData(m);d.qpos[:]=state['qpos'][index]
    out=a.output or folder/'rgbd'/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+a.phase)
    meta=capture(m,d,out);meta.update(candidate_id=a.candidate,state_index=index,requested_time_s=requested,recorded_time_s=float(times[index]),model_sha256=manifest['model_sha256'],state_sha256=result.get('states_sha256'),phase=a.phase if a.time is None else 'explicit_time')
    (out/'camera.json').write_text(json.dumps(meta,indent=2));print('[D435 RGB-D 저장]',out,'유효 depth 비율',meta['valid_depth_fraction'],flush=True)
if __name__=='__main__':main()
