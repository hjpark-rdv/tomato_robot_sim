"""Render stored actual MuJoCo qpos; never run physics in a visual replay."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np


def restore_frame(mj, model, data, states, i):
    """Restore measured state and all render transforms without advancing physics."""
    data.time=states['time'][i]
    for name in ('qpos','qvel','eq_active','mocap_pos','mocap_quat'):
        if name in states:
            values=states[name][i]
            destination=getattr(data,name)
            if np.asarray(values).shape != destination.shape:
                raise ValueError(f'Stored {name} shape {np.asarray(values).shape} differs from model {destination.shape}')
            destination[:]=values
    mj.mj_kinematics(model,data)
    mj.mj_comPos(model,data)
    # Kinematics does not update cameras/lights. Their zero-initialized world
    # transforms produce an entirely black scene even with valid body poses.
    mj.mj_camlight(model,data)


def scene_pixels(rgb):
    """Check the raw scene before captions can hide a failed render."""
    low=int(rgb.min());high=int(rgb.max())
    if high-low<3:
        raise RuntimeError('Empty/uniform scene render; check camera and light transforms')
    return dict(min=low,max=high,mean=float(rgb.mean()),std=float(rgb.std()))


def resolve_model(folder, summary):
    """Prefer the exact per-run native snapshot, then the old smoke location."""
    folder=Path(folder)
    candidates=[folder/'model.mjb',folder.parent/'model.mjb']
    if summary.get('model_dir'):
        candidates.append(Path(summary['model_dir'])/'model.mjb')
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError('No saved replay model: '+', '.join(map(str,candidates)))


def stream_sha256(path):
    """Hash large native models and archives with bounded read memory."""
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):
            digest.update(chunk)
    return digest.hexdigest()


def verify_replay_integrity(folder, summary, model_path=None):
    """Validate recorded bytes before loading a model, archive or viewer.

    Older smoke summaries did not store these digests. Their existing model
    resolution and playback behavior remains available, explicitly unchecked.
    """
    folder=Path(folder)
    model_path=Path(model_path) if model_path is not None else resolve_model(folder,summary)
    verified={}
    for label,path,key in (('model',model_path,'model_mjb_sha256'),
                           ('states',folder/'states.npz','states_sha256')):
        expected=summary.get(key)
        verified[label+'_sha256_verified']=False
        if expected is None:continue
        if (not isinstance(expected,str) or len(expected)!=64 or
                any(c not in '0123456789abcdef' for c in expected.lower())):
            raise ValueError('Invalid saved replay '+label+' SHA256: '+key)
        if stream_sha256(path)!=expected.lower():
            raise ValueError('Replay '+label+' SHA256 mismatch: '+str(path))
        verified[label+'_sha256_verified']=True
    return verified


def make_camera(mj, model, data, requested, summary):
    """Existing close cameras or wide free views of the complete robot/farm."""
    name=requested
    if name=='auto':
        name='aisle' if summary.get('mode')=='dynamic_robot' else 'top'
    camera=mj.MjvCamera();mj.mjv_defaultCamera(camera)
    fixed=mj.mj_name2id(model,mj.mjtObj.mjOBJ_CAMERA,name)
    if fixed>=0:
        camera.type=mj.mjtCamera.mjCAMERA_FIXED;camera.fixedcamid=fixed
        return camera,name
    if name not in ('farm','aisle','front','belt','fruit','harvest'):
        raise ValueError('No saved camera named '+name)
    tool=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,'Tool_fixed')
    focus=data.xpos[tool].copy() if tool>=0 else np.array([0.,2.5,1.2])
    camera.type=mj.mjtCamera.mjCAMERA_FREE
    if name=='farm':
        camera.lookat[:]=[0.,2.5,1.2];camera.distance=6.5
        camera.azimuth=125;camera.elevation=-20
    elif name=='aisle':
        # Along the open rail aisle, rather than through the greenhouse side
        # glazing. This frame includes S300, lift, arm and target plant.
        camera.lookat[:]=[0.,float(focus[1])+.15,1.2]
        camera.distance=2.6;camera.azimuth=90;camera.elevation=-8
    elif name=='front':
        camera.lookat[:]=focus;camera.distance=.95
        camera.azimuth=90;camera.elevation=-12
    else:
        if tool<0:
            raise ValueError('The belt camera requires a measured Tool_fixed body')
        rotation=data.xmat[tool].reshape(3,3)
        if name=='harvest':
            # Virtual diagnostic viewpoint inside the jaw mouth. No geometry
            # is hidden; this is not a validated robot-mounted camera pose.
            eye=focus+rotation@np.array([0.,.22,.08])
            target=focus+rotation@np.array([0.,.12,.03])
            configure_diagnostic_headlight(model)
        elif name=='fruit':
            # A closer oblique view through the jaw opening, useful when the
            # preserved dense foliage obscures the overhead belt camera.
            eye=focus+rotation@np.array([-.12,.10,.30])
            target=focus+rotation@np.array([0.,.18,.04])
        else:
            eye=focus+rotation@np.array([0.,.09,.55])
            target=focus+rotation@np.array([0.,.10,.04])
        delta=eye-target
        camera.lookat[:]=target;camera.distance=float(np.linalg.norm(delta))
        camera.azimuth=float(np.degrees(np.arctan2(-delta[1],-delta[0])))
        camera.elevation=-float(np.degrees(np.arctan2(delta[2],np.linalg.norm(delta[:2]))))
    return camera,name


def configure_diagnostic_headlight(model):
    """Brighten this in-memory replay renderer, never the saved model or physics."""
    settings=dict(active=1,ambient=[.6,.6,.6],diffuse=[.8,.8,.8],specular=[.1,.1,.1])
    model.vis.headlight.active=settings['active']
    for key in ('ambient','diffuse','specular'):
        getattr(model.vis.headlight,key)[:]=settings[key]
    return settings


def caption_lines(summary, states, i, attachment_ids):
    """Explicitly identify dynamic execution versus the prescribed-tool smoke."""
    detached=int((~states['eq_active'][i,attachment_ids].astype(bool)).sum()) if len(attachment_ids) and 'eq_active' in states else 0
    phase=str(states['phase'][i]) if 'phase' in states else ''
    rates=summary.get('physics_steps_by_hz',{})
    rate='/'.join(sorted(rates,key=lambda key:float(key))) if len(rates)>1 else str(summary.get('hz','?'))
    lines=[f'MuJoCo {summary.get("mujoco_version","?")} | {summary.get("mode","stored state")} | {rate} Hz | t={states["time"][i]:.2f}s | {phase} | detached={detached}']
    if summary.get('mode')=='dynamic_robot':
        lines.append('Actual stored physics | driven rail / lift / arm / jaws | simulation diagnosis')
    else:
        lines.append('Actual stored physics | prescribed tool, one truss | simulation diagnosis')
    violation=summary.get('first_1mm_violation')
    if violation and float(states['time'][i])>=float(violation['t']):
        lines.append('DIAGNOSTIC: active 1 mm penetration criterion exceeded')
    return lines


def replay_times(start,end,fps):
    """Include the measured stopping state even between regular video samples."""
    times=np.arange(start,end+1e-8,1/fps)
    if len(times)==0 or times[-1]<end-1e-8:
        times=np.r_[times,end]
    return times


def load_replay_states(path):
    """Read compressed arrays once, rather than decompressing them every frame."""
    keys=('time','qpos','qvel','eq_active','mocap_pos','mocap_quat','phase','window_index')
    with np.load(path) as archive:
        return {key:archive[key] for key in keys if key in archive}


def main():
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path)
    p.add_argument('--gui',action='store_true');p.add_argument('--camera',default='auto',
                    help='auto, top, side, farm, aisle, front, belt, fruit, harvest '
                         '(diagnostic jaw camera with render-only headlight), or a named saved model camera')
    p.add_argument('--fps',type=int,default=30);p.add_argument('--start',type=float,default=0)
    p.add_argument('--end',type=float);p.add_argument('--output',type=Path)
    p.add_argument('--width',type=int,default=960);p.add_argument('--height',type=int,default=720)
    p.add_argument('--gui-duration-s',type=float,
                   help='Optional wall-clock limit for a graceful GUI connection check')
    a=p.parse_args()
    if a.gui_duration_s is not None and a.gui_duration_s<=0:
        p.error('--gui-duration-s must be positive')
    folder=a.trial.resolve()
    summary=json.loads((folder/'summary.json').read_text())
    model_path=resolve_model(folder,summary)
    integrity=verify_replay_integrity(folder,summary,model_path)
    if a.gui:
        os.environ['MUJOCO_GL']='glfw'
    else:
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        import render_backend
    import mujoco as mj
    model=mj.MjModel.from_binary_path(str(model_path))
    d=mj.MjData(model);z=load_replay_states(folder/'states.npz')
    ts=z['time'];end=min(ts[-1],a.end) if a.end is not None else ts[-1]
    if a.fps<=0 or a.start<ts[0] or a.start>end:
        raise ValueError('Positive fps and start/end within the saved time range are required')
    def frame(i):
        restore_frame(mj,model,d,z,i)
    frame(int(np.clip(np.searchsorted(ts,a.start),0,len(ts)-1)))
    camera,camera_name=make_camera(mj,model,d,a.camera,summary)
    attachment_ids=np.array([i for i in range(model.neq)
                             if (mj.mj_id2name(model,mj.mjtObj.mjOBJ_EQUALITY,i) or '').endswith('_attachment')],dtype=int)
    if a.gui:
        import mujoco.viewer
        prior_threads=set(threading.enumerate())
        try:
            with mujoco.viewer.launch_passive(model,d) as viewer:
                for key in ('type','fixedcamid','trackbodyid','distance','azimuth','elevation'):
                    setattr(viewer.cam,key,getattr(camera,key))
                viewer.cam.lookat[:]=camera.lookat
                origin=time.perf_counter()
                try:
                    while viewer.is_running():
                        elapsed=time.perf_counter()-origin
                        if a.gui_duration_s is not None and elapsed>=a.gui_duration_s:break
                        t=a.start+elapsed%max(end-a.start,1/a.fps)
                        i=int(np.clip(np.searchsorted(ts,t),0,len(ts)-1))
                        with viewer.lock():frame(i)
                        viewer.sync();time.sleep(1/a.fps)
                except KeyboardInterrupt:
                    pass
        finally:
            # launch_passive owns daemon render/reload threads. Keep the native
            # model alive until these new threads finish their GLFW cleanup;
            # immediate interpreter exit raced destruction on this server.
            for thread in threading.enumerate():
                if thread not in prior_threads and thread.daemon:
                    thread.join(timeout=5.)
    else:
        import imageio.v2 as imageio
        from PIL import Image,ImageDraw,ImageFont
        output=a.output or folder/f'actual_{camera_name}.mp4'
        if output.exists():raise FileExistsError(output)
        font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
        model.vis.global_.offwidth=max(model.vis.global_.offwidth,a.width)
        model.vis.global_.offheight=max(model.vis.global_.offheight,a.height)
        opt=mj.MjvOption();renderer=mj.Renderer(model,height=a.height,width=a.width)
        started=time.perf_counter();count=0;checks=[]
        with imageio.get_writer(output,fps=a.fps,codec='libx264',quality=7,macro_block_size=1,
                               ffmpeg_params=['-movflags','+faststart']) as writer:
            for t in replay_times(a.start,end,a.fps):
                i=int(np.clip(np.searchsorted(ts,t),0,len(ts)-1));frame(i)
                renderer.update_scene(d,camera=camera,scene_option=opt)
                rgb=renderer.render();pixels=scene_pixels(rgb)
                if count%(a.fps*5)==0:checks.append(dict(t=float(ts[i]),**pixels))
                im=Image.fromarray(rgb);draw=ImageDraw.Draw(im)
                lines=caption_lines(summary,z,i,attachment_ids)
                if camera_name=='harvest':
                    lines.append('Diagnostic virtual camera at jaw opening | render-only headlight')
                draw.rectangle((0,0,a.width,4+20*len(lines)),fill=(25,25,25))
                for j,line in enumerate(lines):draw.text((8,3+20*j),line,font=font,fill=(255,240,200))
                writer.append_data(np.asarray(im));count+=1
        renderer.close()
        report=dict(video=str(output),frames=count,fps=a.fps,playback_rate=1,render_wall_s=time.perf_counter()-started,
                    state_source=str(folder/'states.npz'),physics_reexecuted=False,
                    raw_scene_checked_frames=count,raw_scene_samples=checks,
                    camera_light_transforms_updated=True,model_source=str(model_path),
                    camera=camera_name,width=a.width,height=a.height,caption_font_px=15,
                    execution_mode=summary.get('mode'),
                    render_only_headlight=dict(active=int(model.vis.headlight.active),
                        ambient=model.vis.headlight.ambient.tolist(),
                        diffuse=model.vis.headlight.diffuse.tolist(),
                        specular=model.vis.headlight.specular.tolist()) if camera_name=='harvest' else None,
                    geometry_hidden=False,**integrity)
        output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))


if __name__=='__main__':main()
