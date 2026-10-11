#!/usr/bin/env python3
"""Render saved measured rigid-body poses; never replay nominal commands as physics."""
import argparse
import json
import subprocess
import time
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('assembly',type=Path);p.add_argument('trial',type=Path)
p.add_argument('--fps',type=int,default=15);p.add_argument('--speed',type=float,default=2.)
p.add_argument('--camera',default='/World/HarvestCloseCamera',help='USD camera path, motion, entry, jaw, mouth, belt_contact, grip_front, truss, truss_top or leaf_pulse')
p.add_argument('--name',default='actual_motion',help='New output basename, never overwrites a video')
p.add_argument('--review-light',action='store_true',help='Add replay-only fill light for contact inspection')
p.add_argument('--contact-cutaway',action='store_true',help='Replay inspection only: hide background plants and active leaf visuals, record exact paths; source physics and saved poses unchanged')
p.add_argument('--start-time',type=float,help='Replay-only clip start in recorded simulation seconds')
p.add_argument('--end-time',type=float,help='Replay-only clip end in recorded simulation seconds')
p.add_argument('--camera-eye',type=float,nargs=3,help='Optional fixed world-space replay camera position, shared across comparisons')
p.add_argument('--camera-target',type=float,nargs=3,help='World-space look-at point; requires --camera-eye')
a=p.parse_args()
if (a.camera_eye is None)!=(a.camera_target is None):p.error('Provide both camera-eye and camera-target')
if a.camera_eye is not None:
    import math
    if not all(math.isfinite(x) for x in a.camera_eye+a.camera_target) or a.camera_eye==a.camera_target:
        p.error('Finite distinct camera position and target required')
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
    # NPZ entries are compressed: repeated indexed access would decompress the
    # full pose history for every rendered frame. Decode required measured
    # arrays once; frame selection and all recorded values remain unchanged.
    with np.load(a.trial/'states.npz',allow_pickle=False) as recorded:
        data={key:recorded[key] for key in ('body_paths','time','phase','poses','robot_q','cycle_index')
              if key in recorded}
    meta=json.loads((a.assembly/'motion_manifest.json').read_text())
    summary=json.loads((a.trial/'summary.json').read_text())
    fruit_joints={f['joint'] for f in meta['plant']['fruits']}
    breaks=json.loads((a.trial/'break_events.json').read_text())
    if summary['model_sha256']!=sha(a.assembly/'scene.usda'):raise ValueError('Replay scene hash mismatch')
    from native_stage_selection import select_native_scene
    runtime=json.loads((a.trial/'runtime_settings.json').read_text()) if (a.trial/'runtime_settings.json').exists() else {}
    selected_asset=select_native_scene(a.assembly,use_materialized=runtime.get('native_materialized_stage',False))
    if (runtime.get('native_scene_sha256') and
            selected_asset['native_scene_sha256']!=runtime['native_scene_sha256']):
        raise ValueError('Replay native materialized asset SHA mismatch')
    omni.usd.get_context().open_stage(str(a.assembly/'scene.usda'))
    stage=omni.usd.get_context().get_stage()
    # Match the live viewer: all replay edits, including the transient fixture,
    # belong to the strongest session layer rather than the immutable source.
    stage.SetEditTarget(stage.GetSessionLayer())
    restore_recorded_display_fixture(stage,a.trial)
    if a.camera_eye is not None:
        a.camera='/World/FixedComparisonReplayCamera'
        camera(stage,a.camera,tuple(a.camera_eye),tuple(a.camera_target))
        UsdGeom.Camera(stage.GetPrimAtPath(a.camera)).CreateFocalLengthAttr(24.)
    paths=data['body_paths'].tolist();index={p:i for i,p in enumerate(paths)}
    cutaway_hidden=[]
    if a.contact_cutaway:
        background=stage.GetPrimAtPath('/World/Greenhouse/Plants')
        if background:
            UsdGeom.Imageable(background).MakeInvisible();cutaway_hidden.append(str(background.GetPath()))
        from pxr import Usd
        for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/ElasticTarget')):
            source_name=str(prim.GetCustomDataByKey('sourceName') or '')
            selected_pulse_leaf=(a.camera=='leaf_pulse' and summary.get('pulse_body') and
                                 str(prim.GetPath()).startswith(summary['pulse_body']+'/'))
            if prim.IsA(UsdGeom.Mesh) and not selected_pulse_leaf and any(x in source_name for x in ('CompoundLeaf','Apical_Young_Leaf')):
                UsdGeom.Imageable(prim).MakeInvisible();cutaway_hidden.append(str(prim.GetPath()))
    pose_display=MeasuredPoseDisplay(stage,paths)
    native=json.loads((a.trial/'native_metadata.json').read_text());names=native['robot_dofs']
    lids=[names.index('jaw_L'),names.index('jaw_R')]
    visuals=DeformationVisuals(stage,a.assembly,paths)
    if a.camera=='leaf_pulse':
        leaf=next(r for r in meta['compliant_foliage']['leaves'] if r['path']==summary['pulse_body'])
        center=(np.asarray(leaf['rest_position'])+np.asarray(leaf['pivot']))/2
        branch=np.asarray(leaf['rest_position'])-np.asarray(leaf['pivot'])
        side=np.cross(branch,[0.,0.,1.]);side=side/max(np.linalg.norm(side),1e-8)
        eye=center+side*.5+np.array([0.,0.,.3])
        camera(stage,'/World/LeafPulseReplayCamera',tuple(eye),tuple(center))
        UsdGeom.Camera(stage.GetPrimAtPath('/World/LeafPulseReplayCamera')).GetFocalLengthAttr().Set(22.)
        a.camera='/World/LeafPulseReplayCamera'
    elif a.camera=='truss_top':
        from truss_sequence import truss_fruits,horizontal_truss_direction
        fruits=truss_fruits(meta)
        points=np.array([f['center'] for f in fruits])
        rotation=R.from_quat(data['poses'][0,index['/World/ActuatedTool/fixed'],3:]).as_matrix()
        direction=np.array(horizontal_truss_direction(fruits,[],[f['path'] for f in fruits],
            points,rotation[:,1])['direction_world'])
        center=points.mean(axis=0)-direction*.08
        eye=center+np.array([0.,0.,.85])
        a.camera='/World/TrussTopReplayCamera'
        cam=UsdGeom.Camera.Define(stage,a.camera)
        cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),
            Gf.Vec3d(*map(float,center)),Gf.Vec3d(*map(float,direction))).GetInverse())
        cam.CreateFocalLengthAttr(22.);cam.CreateClippingRangeAttr(Gf.Vec2f(.02,100.))
    elif a.camera=='truss':
        from truss_sequence import truss_fruits
        fruits=truss_fruits(meta)
        center=np.mean([data['poses'][0,index[f['path']],:3] for f in fruits],axis=0)
        rotation=R.from_quat(data['poses'][0,index['/World/ActuatedTool/fixed'],3:]).as_matrix()
        eye=center+rotation@np.array([.32,.65,.18])
        camera(stage,'/World/TrussReplayCamera',tuple(map(float,eye)),tuple(map(float,center)))
        UsdGeom.Camera(stage.GetPrimAtPath('/World/TrussReplayCamera')).GetFocalLengthAttr().Set(22.)
        a.camera='/World/TrussReplayCamera'
    elif a.camera in ('motion','entry'):
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
    from motion_replay_clip import select_replay_ticks
    stamps=data['time'];selection=select_replay_ticks(stamps,fps=a.fps,speed=a.speed,
        start_time=a.start_time,end_time=a.end_time)
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
        if summary.get('truss_sequence'):
            seq=summary['truss_sequence'];selected=set(seq['fruit_joints'].values())
            label+=f' | truss detached: {len(broken & selected)}/{len(selected)} | cycle: {data["cycle_index"][tick]}'
        if meta['tool'].get('tread_prototype'):label+='\\NSEPARATE HARDWARE PROTOTYPE: added inner belt tread'
        if meta.get('fruit_attachment'):label+='\\NATTACHMENT HYPOTHESIS: '+meta['fruit_attachment']['kind']+' (uncalibrated)'
        if summary.get('belt_off_control'):label+='\\NBELT OFF CONTROL'
        if cutaway_hidden:label+='\\NCONTACT CUTAWAY: foliage hidden in replay display only'
        if (summary.get('belt_control') or {}).get('fixture_only'):
            label+='\\NLOADED BELT CONTROL FIXTURE - NOT FRUIT HARVEST'
        if summary.get('fixture_only'):label+='\\NCOMPONENT FIXTURE - NOT FARM HARVEST'
        if meta.get('plant_interaction_variant',{}).get('mode')=='target_truss_and_own_stem_only':
            label+='\\NNON-TARGET PLANT CONTACTS OFF'
        elif meta.get('elastic_neighbors'):
            label+=f'\\NDYNAMIC PLANTS: {1+len(meta["elastic_neighbors"])}'
        if summary.get('stop_reason'):label+='\\NStopped: '+summary['stop_reason']
        subtitles.append(f'{frame+1}\n{srt_time(frame/a.fps)} --> {srt_time((frame+1)/a.fps)}\n'+label.replace('\\N','\n')+'\n')
        if frame%30==0:print('[RENDER]',frame,'/',len(selection),flush=True)
    caption=a.trial/(a.name+'.srt');caption.write_text('\n'.join(subtitles))
    # Keep the diagnostic captions readable without covering the harvest view.
    # libass FontSize is relative to its script resolution, not output pixels.
    subtitle_style='FontSize=10,Outline=1,Shadow=0,MarginV=6'
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-framerate',str(a.fps),'-i',str(output/'%05d.png'),
                    '-vf','subtitles='+str(caption.resolve())+":force_style='"+subtitle_style+"'",'-frames:v',str(len(selection)),
                    '-c:v','libx264','-pix_fmt','yuv420p','-crf','22',str(a.trial/(a.name+'.mp4'))],check=True)
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-select_streams','v:0',
        '-show_entries','stream=nb_read_frames','-of','json',str(a.trial/(a.name+'.mp4'))]))
    if int(probe['streams'][0]['nb_read_frames'])!=len(selection):raise RuntimeError('Encoded frame count mismatch')
    (a.trial/(a.name+'_render.json')).write_text(json.dumps(dict(source='stored measured physical body transforms',
        frames=len(selection),fps=a.fps,speed=a.speed,render_ticks=selection.tolist(),camera=a.camera,
        physics_rerun=False,replay_only_fill_light=a.review_light,subtitle_style=subtitle_style,
        replay_camera_eye=a.camera_eye,replay_camera_target=a.camera_target,
        requested_clip_start_s=a.start_time,requested_clip_end_s=a.end_time,
        actual_clip_start_s=float(stamps[selection[0]]),actual_clip_end_s=float(stamps[selection[-1]]),
        source_trial_stop_time_s=float(stamps[-1]),source_scene_sha256=summary['model_sha256'],
        source_native_scene_sha256=selected_asset['native_scene_sha256'],
        materialized_dependency_sha_verified=bool(selected_asset['verified_materialized_dependency']),
        replay_only_hidden_paths=cutaway_hidden,source_geometry_modified=False,
        stop_reason=summary.get('stop_reason')),indent=2))
except Exception:
    import traceback
    exit_code=1;message=traceback.format_exc();print(message,flush=True)
    (a.trial/'render_error.txt').write_text(message)
finally:app.close(exit_code=exit_code)
