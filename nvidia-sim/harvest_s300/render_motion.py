#!/usr/bin/env python3
"""Render saved measured rigid-body poses; never replay nominal commands as physics."""
import argparse
import json
import subprocess
import time
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('assembly',type=Path);p.add_argument('trial',type=Path)
p.add_argument('--fps',type=int,default=15);p.add_argument('--speed',type=float,default=2.)
p.add_argument('--camera',default='/World/HarvestCloseCamera',help='USD camera path, motion, entry, jaw, mouth, belt_contact or grip_front')
p.add_argument('--name',default='actual_motion',help='New output basename, never overwrites a video')
p.add_argument('--review-light',action='store_true',help='Add replay-only fill light for contact inspection')
p.add_argument('--contact-cutaway',action='store_true',help='Replay inspection only: hide background plants and active leaf visuals, record exact paths; source physics and saved poses unchanged')
a=p.parse_args()
if not a.name.replace('_','').isalnum():p.error('name must be an alphanumeric basename')
output=a.trial/(a.name+'_frames');output.mkdir(exist_ok=False)
if (a.trial/(a.name+'.mp4')).exists():raise FileExistsError(a.trial/(a.name+'.mp4'))
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':720,'renderer':'RaytracedLighting'})
exit_code=0
try:
    import sys
    import numpy as np
    import omni.usd
    import omni.kit.viewport.utility as vu
    from pxr import Gf,UsdGeom,Vt,UsdPhysics,UsdLux
    from scipy.spatial.transform import Rotation as R
    from motion_assets import REPO,sha
    from build_scene import camera
    from motion_visuals import DeformationVisuals
    from motion_pose_display import MeasuredPoseDisplay, restore_recorded_display_fixture
    sys.path.insert(0,str(REPO/'nvidia-sim/rl'))
    from elastic_geometry import skin_points
    data=np.load(a.trial/'states.npz');meta=json.loads((a.assembly/'motion_manifest.json').read_text())
    summary=json.loads((a.trial/'summary.json').read_text())
    fruit_joints={f['joint'] for f in meta['plant']['fruits']}
    breaks=json.loads((a.trial/'break_events.json').read_text())
    if summary['model_sha256']!=sha(a.assembly/'scene.usda'):raise ValueError('Replay scene hash mismatch')
    omni.usd.get_context().open_stage(str(a.assembly/'scene.usda'))
    stage=omni.usd.get_context().get_stage()
    # Match the live viewer: all replay edits, including the transient fixture,
    # belong to the strongest session layer rather than the immutable source.
    stage.SetEditTarget(stage.GetSessionLayer())
    restore_recorded_display_fixture(stage,a.trial)
    paths=data['body_paths'].tolist();index={p:i for i,p in enumerate(paths)}
    cutaway_hidden=[]
    if a.contact_cutaway:
        background=stage.GetPrimAtPath('/World/Greenhouse/Plants')
        if background:
            UsdGeom.Imageable(background).MakeInvisible();cutaway_hidden.append(str(background.GetPath()))
        from pxr import Usd
        for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/ElasticTarget')):
            source_name=str(prim.GetCustomDataByKey('sourceName') or '')
            if prim.IsA(UsdGeom.Mesh) and any(x in source_name for x in ('CompoundLeaf','Apical_Young_Leaf')):
                UsdGeom.Imageable(prim).MakeInvisible();cutaway_hidden.append(str(prim.GetPath()))
    pose_display=MeasuredPoseDisplay(stage,paths)
    native=json.loads((a.trial/'native_metadata.json').read_text());names=native['robot_dofs']
    lids=[names.index('jaw_L'),names.index('jaw_R')]
    visuals=DeformationVisuals(stage,a.assembly,paths)
    if a.camera in ('motion','entry'):
        tool=data['poses'][:,index['/World/ActuatedTool/fixed'],:3]
        points=np.vstack((tool,np.array(meta['target']['center'])[None]))
        center=(points.min(0)+points.max(0))/2
        offset=np.array([1.,-1.3,.65])
        if a.camera=='entry':
            orientation=R.from_quat(data['poses'][0,index['/World/ActuatedTool/fixed'],3:]).as_matrix()
            center=center+np.array([0.,0.,.18])
            offset=-orientation[:,0]*1.8+np.array([0.,0.,.30])
        camera(stage,'/World/MotionReplayCamera',tuple(center+offset),tuple(center))
        a.camera='/World/MotionReplayCamera'
    elif a.camera in ('jaw','mouth','belt_contact','grip_front'):
        camera_tick=0
        if a.camera in ('belt_contact','grip_front'):
            spin_ticks=np.flatnonzero(data['phase']=='belt_spin')
            camera_tick=int(spin_ticks[len(spin_ticks)//2]) if len(spin_ticks) else len(data['time'])-1
        pose=data['poses'][camera_tick,index['/World/ActuatedTool/fixed']]
        rotation=R.from_quat(pose[3:]).as_matrix()
        if a.camera=='grip_front':
            eye_local=np.array([0.,.40,.08]);center_local=np.array([0.,.10,.037])
            replay_camera='/World/GripFrontReplayCamera';focal_length=22.
        elif a.camera=='belt_contact':
            eye_local=np.array([.18,.27,.14]);center_local=np.array([0.,.1,.037])
            replay_camera='/World/BeltContactReplayCamera';focal_length=22.
        elif a.camera=='mouth':
            # Frontal CAD +Y view exposes belt/fixture contact hidden by the
            # covers in the existing oblique jaw camera. Replay display only.
            eye_local=np.array([0.,.58,.075]);center_local=np.array([0.,.08,.037])
            replay_camera='/World/MouthReplayCamera';focal_length=32.
        else:
            eye_local=np.array([.35,.37,.28]);center_local=np.array([0.,.04,.02])
            replay_camera='/World/JawReplayCamera';focal_length=22.
        eye=pose[:3]+rotation@eye_local;center=pose[:3]+rotation@center_local
        camera(stage,replay_camera,tuple(eye),tuple(center))
        UsdGeom.Camera(stage.GetPrimAtPath(replay_camera)).GetFocalLengthAttr().Set(focal_length)
        a.camera=replay_camera
    if a.review_light:
        eye=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(a.camera)))[3,:3]
        light=UsdLux.SphereLight.Define(stage,'/World/ReplayFillLight')
        light.CreateIntensityAttr(1800.);light.CreateRadiusAttr(.5)
        UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(*eye))
    vp=vu.get_active_viewport();vp.set_texture_resolution((960,720));vp.set_active_camera(a.camera)
    for _ in range(20):app.update()
    stamps=data['time'];selection=np.searchsorted(stamps,np.arange(stamps[0],stamps[-1]+1e-8,a.speed/a.fps))
    selection=np.minimum(selection,len(stamps)-1)
    if selection[-1]!=len(stamps)-1:selection=np.r_[selection,len(stamps)-1]
    if len(selection)<a.fps:selection=np.r_[selection,np.repeat(selection[-1],a.fps-len(selection))]
    subtitles=[]
    def srt_time(t):
        ms=round(t*1000);return f'{ms//3600000:02}:{ms//60000%60:02}:{ms//1000%60:02},{ms%1000:03}'
    for frame,tick in enumerate(selection):
        poses=data['poses'][tick]
        pose_display.update(poses)
        gap=.044+float(data['robot_q'][tick,lids[1]]-data['robot_q'][tick,lids[0]])
        visuals.update(poses,gap)
        for _ in range(2):app.update()
        png=output/f'{frame:05d}.png'
        capture=vu.capture_viewport_to_file(vp,str(png))
        # Capture async completion is pumped by Kit. Never advance state first.
        import asyncio
        task=asyncio.ensure_future(capture.wait_for_result())
        while not task.done():app.update()
        task.result()
        # Kit's capture future may finish before the asynchronous PNG write.
        # Wait for the PNG end chunk before advancing or starting ffmpeg.
        deadline=time.monotonic()+10
        while not png.exists() or png.read_bytes()[-12:]!=b'\x00\x00\x00\x00IEND\xaeB`\x82':
            if time.monotonic()>deadline:raise RuntimeError('PNG write incomplete: '+str(png))
            app.update()
        label=f"Actual physics | t={stamps[tick]:.3f}s | {data['phase'][tick]}"
        broken={e['joint'] for e in breaks if e['joint'] in fruit_joints and e['t']<=stamps[tick]}
        label+=f' | fruit joints broken: {len(broken)}'
        if meta['tool'].get('tread_prototype'):label+='\\NSEPARATE HARDWARE PROTOTYPE: added inner belt tread'
        if cutaway_hidden:label+='\\NCONTACT CUTAWAY: foliage hidden in replay display only'
        if (summary.get('belt_control') or {}).get('fixture_only'):
            label+='\\NLOADED BELT CONTROL FIXTURE - NOT FRUIT HARVEST'
        if summary.get('fixture_only'):label+='\\NCOMPONENT FIXTURE - NOT FARM HARVEST'
        if summary.get('stop_reason'):label+='\\NStopped: '+summary['stop_reason']
        subtitles.append(f'{frame+1}\n{srt_time(frame/a.fps)} --> {srt_time((frame+1)/a.fps)}\n'+label.replace('\\N','\n')+'\n')
        if frame%30==0:print('[RENDER]',frame,'/',len(selection),flush=True)
    caption=a.trial/(a.name+'.srt');caption.write_text('\n'.join(subtitles))
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-framerate',str(a.fps),'-i',str(output/'%05d.png'),
                    '-vf','subtitles='+str(caption.resolve()),'-frames:v',str(len(selection)),
                    '-c:v','libx264','-pix_fmt','yuv420p','-crf','22',str(a.trial/(a.name+'.mp4'))],check=True)
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-select_streams','v:0',
        '-show_entries','stream=nb_read_frames','-of','json',str(a.trial/(a.name+'.mp4'))]))
    if int(probe['streams'][0]['nb_read_frames'])!=len(selection):raise RuntimeError('Encoded frame count mismatch')
    (a.trial/(a.name+'_render.json')).write_text(json.dumps(dict(source='stored measured physical body transforms',
        frames=len(selection),fps=a.fps,speed=a.speed,render_ticks=selection.tolist(),camera=a.camera,
        physics_rerun=False,replay_only_fill_light=a.review_light,
        replay_only_hidden_paths=cutaway_hidden,source_geometry_modified=False,
        stop_reason=summary.get('stop_reason')),indent=2))
except Exception:
    import traceback
    exit_code=1;message=traceback.format_exc();print(message,flush=True)
    (a.trial/'render_error.txt').write_text(message)
finally:app.close(exit_code=exit_code)
