"""Render stored actual MuJoCo qpos; never run physics in a visual replay."""
import argparse
import hashlib
import json
import math
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


def render_rgb(renderer, profile):
    """Real spatial antialiasing after the complete high-resolution scene."""
    rgb=renderer.render()
    scene_pixels(rgb)
    if profile['supersample']>1:
        from PIL import Image
        rgb=np.asarray(Image.fromarray(rgb).resize(
            (profile['width'],profile['height']),Image.Resampling.LANCZOS))
    return rgb


def filament_gui(model,data,camera,states,frame,start,end,profile,duration=None):
    """Interactive display of saved states through the same PBR video backend."""
    from filament_backend import FilamentRenderer,FilamentWindow
    renderer=FilamentRenderer(model,height=profile['internal_height'],
                              width=profile['internal_width'],quality=profile['name'])
    count=0;render_s=0.;origin=time.perf_counter();last=origin;clock=start
    width,height=profile['width'],profile['height']
    try:
        with FilamentWindow(camera,width=profile['width'],height=profile['height'],
                title='Farmily MuJoCo | '+profile['name']+' | Filament | saved states') as window:
            try:
                while window.is_running():
                    tick=time.perf_counter();window.poll_events()
                    if duration is not None and tick-origin>=duration:break
                    if not window.paused:clock+=tick-last
                    last=tick
                    t=start+(clock-start)%max(end-start,1/profile['fps'])
                    i=int(np.clip(np.searchsorted(states['time'],t),0,len(states['time'])-1))
                    frame(i)
                    width,height=window.size
                    if width<=0 or height<=0:
                        time.sleep(.05);continue
                    active={**profile,'width':width,'height':height}
                    renderer.resize(width*profile['supersample'],height*profile['supersample'])
                    renderer.update_scene(data,camera=camera)
                    begin=time.perf_counter();rgb=render_rgb(renderer,active)
                    render_s+=time.perf_counter()-begin
                    window.present(rgb);count+=1
                    delay=1/profile['fps']-(time.perf_counter()-tick)
                    if delay>0:time.sleep(delay)
            except KeyboardInterrupt:
                pass
    finally:
        renderer.close()
    wall=time.perf_counter()-origin
    return dict(quality=profile['name'],renderer='filament',presented_frames=count,
                display_wall_s=wall,presented_fps=count/wall,render_wall_s=render_s,
                presented_size=[width,height],supersample=profile['supersample'],
                renderer_initialization_s=renderer.initialization_s,
                color_grading_configure_calls=renderer.color_grading_configure_calls,
                physics_reexecuted=False,playback_rate=1)


def main():
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path)
    p.add_argument('--gui',action='store_true');p.add_argument('--camera',default='auto',
                    help='auto, top, side, farm, aisle, front, belt, fruit, harvest '
                         '(diagnostic jaw camera with render-only headlight), or a named saved model camera')
    p.add_argument('--fps',type=int);p.add_argument('--start',type=float,default=0)
    p.add_argument('--end',type=float);p.add_argument('--output',type=Path)
    p.add_argument('--width',type=int);p.add_argument('--height',type=int)
    p.add_argument('--quality',choices=('fast','balanced','ultra'),default='fast',
                   help='Display only: classic fast, authored PBR balanced, 4K/2x supersampled PBR ultra')
    p.add_argument('--image',action='store_true',help='Save a single PNG at --start instead of MP4')
    p.add_argument('--render-cache',type=Path,help='Separate source-visual render cache; physics files stay immutable')
    p.add_argument('--gui-metrics',type=Path,help='Save measured GUI display throughput as JSON')
    p.add_argument('--gui-duration-s',type=float,
                   help='Optional wall-clock limit for a graceful GUI connection check')
    a=p.parse_args()
    if a.gui_duration_s is not None and (not math.isfinite(a.gui_duration_s) or a.gui_duration_s<=0):
        p.error('--gui-duration-s must be positive')
    if not math.isfinite(a.start) or (a.end is not None and not math.isfinite(a.end)):
        p.error('--start and --end must be finite seconds')
    folder=a.trial.resolve()
    summary=json.loads((folder/'summary.json').read_text())
    model_path=resolve_model(folder,summary)
    print('Verifying saved model and state hashes...',flush=True)
    integrity=verify_replay_integrity(folder,summary,model_path)
    from quality_profiles import quality_profile
    profile=quality_profile(a.quality,gui=a.gui,width=a.width,height=a.height,fps=a.fps)
    a.width,a.height,a.fps=profile['width'],profile['height'],profile['fps']
    if profile['backend']=='filament' and sys.version_info<(3,12):
        render_python=Path(__file__).resolve().parents[2]/'.mujoco315-render-venv/bin/python'
        if not render_python.is_file():
            raise RuntimeError('Run mujoco-benchmark/setup_s300_render.sh to prepare the PBR Python environment')
        if Path(sys.executable).absolute()==render_python.absolute():
            raise RuntimeError('PBR display environment requires Python >=3.12; recreate it with setup_s300_render.sh')
        os.execv(str(render_python),[str(render_python),str(Path(__file__).resolve()),*sys.argv[1:]])
    if profile['backend']=='filament':
        from filament_backend import configure_graphics_environment
        configure_graphics_environment()
    elif a.gui:
        os.environ['MUJOCO_GL']='glfw'
    else:
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        import render_backend
    import mujoco as mj
    print(f'Loading saved-state display: {profile["name"]} / {profile["backend"]} / '
          f'{a.width}x{a.height}, supersampling {profile["supersample"]}x',flush=True)
    model=mj.MjModel.from_binary_path(str(model_path))
    visual_evidence=None
    if profile['restore_source_visuals']:
        print('Loading source materials and surface cache (first build takes longer)...',flush=True)
        from render_assets import load_render_model
        kwargs=dict(original_mjb=model_path,quality=profile['name'])
        if a.render_cache is not None:kwargs['cache_root']=a.render_cache
        model,visual_evidence=load_render_model(model,Path(summary.get('model_dir',model_path.parent)),**kwargs)
    d=mj.MjData(model);z=load_replay_states(folder/'states.npz')
    ts=z['time'];end=min(ts[-1],a.end) if a.end is not None else ts[-1]
    if a.fps<=0 or a.start<ts[0] or a.start>end:
        raise ValueError('Positive fps and start/end within the saved time range are required')
    def frame(i):
        restore_frame(mj,model,d,z,i)
        if visual_evidence is not None:
            from render_assets import update_render_visuals
            update_render_visuals(model,d,visual_evidence)
    frame(int(np.clip(np.searchsorted(ts,a.start),0,len(ts)-1)))
    camera,camera_name=make_camera(mj,model,d,a.camera,summary)
    attachment_ids=np.array([i for i in range(model.neq)
                             if (mj.mj_id2name(model,mj.mjtObj.mjOBJ_EQUALITY,i) or '').endswith('_attachment')],dtype=int)
    if a.gui and profile['backend']=='filament':
        print('Opening PBR viewer; mouse rotates/pans/zooms, Space pauses, Escape closes.',flush=True)
        result=filament_gui(model,d,camera,z,frame,a.start,end,profile,a.gui_duration_s)
        result.update(**integrity,source_model=str(model_path),quality_profile=profile,
                      visual_restore=visual_evidence)
        if a.gui_metrics:a.gui_metrics.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    elif a.gui:
        import mujoco.viewer
        prior_threads=set(threading.enumerate())
        sync_count=0;sync_s=0.;display_wall_s=0.
        try:
            with mujoco.viewer.launch_passive(model,d) as viewer:
                for key in ('type','fixedcamid','trackbodyid','distance','azimuth','elevation'):
                    setattr(viewer.cam,key,getattr(camera,key))
                viewer.cam.lookat[:]=camera.lookat
                origin=time.perf_counter()
                try:
                    while viewer.is_running():
                        tick=time.perf_counter()
                        elapsed=time.perf_counter()-origin
                        if a.gui_duration_s is not None and elapsed>=a.gui_duration_s:break
                        t=a.start+elapsed%max(end-a.start,1/a.fps)
                        i=int(np.clip(np.searchsorted(ts,t),0,len(ts)-1))
                        with viewer.lock():frame(i)
                        sync_start=time.perf_counter();viewer.sync()
                        sync_s+=time.perf_counter()-sync_start;sync_count+=1
                        delay=1/a.fps-(time.perf_counter()-tick)
                        if delay>0:time.sleep(delay)
                except KeyboardInterrupt:
                    pass
                finally:
                    display_wall_s=time.perf_counter()-origin
        finally:
            # launch_passive owns daemon render/reload threads. Keep the native
            # model alive until these new threads finish their GLFW cleanup;
            # immediate interpreter exit raced destruction on this server.
            for thread in threading.enumerate():
                if thread not in prior_threads and thread.daemon:
                    thread.join(timeout=5.)
        result=dict(quality='fast',renderer='classic',viewer_sync_calls=sync_count,
                    viewer_sync_fps=sync_count/max(display_wall_s,1e-9),
                    display_wall_s=display_wall_s,viewer_sync_wall_s=sync_s,
                    presented_fps=None,actual_presented_fps_measured=False,
                    physics_reexecuted=False,playback_rate=1,quality_profile=profile,**integrity)
        if a.gui_metrics:a.gui_metrics.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    else:
        import imageio.v2 as imageio
        from PIL import Image,ImageDraw,ImageFont
        suffix=('.png' if a.image else '.mp4')
        quality_suffix='' if a.quality=='fast' else '_'+a.quality
        output=a.output or folder/f'actual_{camera_name}{quality_suffix}{suffix}'
        if output.exists():raise FileExistsError(output)
        font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
        model.vis.global_.offwidth=max(model.vis.global_.offwidth,a.width)
        model.vis.global_.offheight=max(model.vis.global_.offheight,a.height)
        opt=mj.MjvOption()
        if profile['backend']=='filament':
            from filament_backend import FilamentRenderer
            renderer=FilamentRenderer(model,height=profile['internal_height'],
                width=profile['internal_width'],quality=profile['name'])
        else:renderer=mj.Renderer(model,height=a.height,width=a.width)
        started=time.perf_counter();last_progress=started;count=0;checks=[]
        writer=None if a.image else imageio.get_writer(output,fps=a.fps,codec='libx264',
            macro_block_size=1,ffmpeg_params=['-movflags','+faststart','-crf','16' if a.quality=='ultra' else '20'])
        try:
            for t in ([a.start] if a.image else replay_times(a.start,end,a.fps)):
                i=int(np.clip(np.searchsorted(ts,t),0,len(ts)-1));frame(i)
                renderer.update_scene(d,camera=camera,scene_option=opt)
                rgb=render_rgb(renderer,profile);pixels=scene_pixels(rgb)
                if count%(a.fps*5)==0:checks.append(dict(t=float(ts[i]),**pixels))
                im=Image.fromarray(rgb);draw=ImageDraw.Draw(im)
                lines=caption_lines(summary,z,i,attachment_ids)
                if a.quality!='fast':lines.append('Display: '+a.quality+' | Filament PBR | restored source surfaces')
                if camera_name=='harvest':
                    lighting='neutral daylight' if profile['backend']=='filament' else 'render-only headlight'
                    lines.append('Diagnostic virtual camera at jaw opening | '+lighting)
                draw.rectangle((0,0,a.width,4+20*len(lines)),fill=(25,25,25))
                for j,line in enumerate(lines):draw.text((8,3+20*j),line,font=font,fill=(255,240,200))
                if a.image:im.save(output)
                else:writer.append_data(np.asarray(im))
                count+=1
                now=time.perf_counter()
                if not a.image and (count%(a.fps*5)==0 or now-last_progress>=10):
                    print(f'render {a.quality}: {count} frames, saved physics t={ts[i]:.2f}s, '
                          f'{now-started:.1f}s rendering',flush=True)
                    last_progress=now
        finally:
            if writer is not None:writer.close()
            renderer.close()
        report=dict(video=str(output),frames=count,fps=a.fps,playback_rate=1,render_wall_s=time.perf_counter()-started,
                    renderer_initialization_s=getattr(renderer,'initialization_s',None),
                    color_grading_configure_calls=getattr(renderer,'color_grading_configure_calls',None),
                    state_source=str(folder/'states.npz'),physics_reexecuted=False,
                    raw_scene_checked_frames=count,raw_scene_samples=checks,
                    camera_light_transforms_updated=True,model_source=str(model_path),
                    display_model_source=visual_evidence['render_cache'] if visual_evidence else str(model_path),
                    camera=camera_name,width=a.width,height=a.height,caption_font_px=15,
                    execution_mode=summary.get('mode'),
                    quality_profile=profile,visual_restore=visual_evidence,
                    render_only_headlight=dict(active=int(model.vis.headlight.active),
                        applied_by_backend=profile['backend']=='classic',
                        ambient=model.vis.headlight.ambient.tolist(),
                        diffuse=model.vis.headlight.diffuse.tolist(),
                        specular=model.vis.headlight.specular.tolist()) if camera_name=='harvest' else None,
                    geometry_hidden=bool(visual_evidence and visual_evidence.get('hidden_source_collision_visuals')),
                    source_visual_geometry_hidden=False,
                    collision_proxies_hidden_for_visual_replacement=visual_evidence.get('hidden_source_collision_visuals',0) if visual_evidence else 0,
                    **integrity)
        output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))


if __name__=='__main__':main()
