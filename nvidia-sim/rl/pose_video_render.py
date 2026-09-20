"""Blender renderer for original USD meshes + recorded native PhysX poses.

Invoke using xvfb-run blender --python THIS -- --scene scene.npz --capture DIR
Studio lighting replaces RTX/MDL appearance only. No physics is run here.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import shutil
from concurrent.futures import ThreadPoolExecutor

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Matrix, Vector
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def rotation(q):
    """Batch wxyz quaternion to rotation, avoiding non-Blender dependencies."""
    q = np.asarray(q, dtype=float); q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = np.moveaxis(q, -1, 0)
    return np.stack((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)), axis=-1).reshape(q.shape[:-1]+(3,3))


def render_parts(args, motion_frames):
    """Parallel independent rendering only; no simulation or interpolation."""
    count = min(8, math.ceil(motion_frames/150))
    cuts = np.linspace(0,motion_frames,count+1,dtype=int)
    parts = args.output.parent/'_parts'/args.output.stem; parts.mkdir(parents=True,exist_ok=True)
    def render_part(i):
        output = parts/f'part_{i:02d}.mp4'; log = parts/f'part_{i:02d}.log'
        command = ['xvfb-run','-a','blender','-t','2','--python',str(Path(__file__).resolve()),'--',
            '--scene',str(args.scene),'--capture',str(args.capture),'--output',str(output),
            '--frame-start',str(cuts[i]),'--frame-stop',str(cuts[i+1])]
        with log.open('w') as stream:
            result = subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
        if result.returncode: raise RuntimeError('Video segment failed: '+str(log))
        print('[SEGMENT COMPLETE]',args.output.stem,i+1,count,flush=True)
        return output
    with ThreadPoolExecutor(max_workers=count) as pool:
        outputs = list(pool.map(render_part,range(count)))
    listing = parts/'concat.txt'
    listing.write_text(''.join("file '"+str(p).replace("'", "'\\''")+"'\n" for p in outputs))
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','concat','-safe','0',
        '-i',str(listing),'-c','copy','-movflags','+faststart',str(args.output)],check=True)
    summaries = [json.loads(p.with_suffix('.json').read_text()) for p in outputs]
    info = dict(summaries[-1]); info.update(video=str(args.output),frames=sum(s['frames'] for s in summaries),
        motion_frames=motion_frames,frame_start=0,frame_stop=motion_frames,render_segments=count)
    if info['frames'] != motion_frames+30: raise RuntimeError('Segment timeline has missing or duplicate frames')
    info['video_duration_s'] = info['frames']/30
    args.output.with_suffix('.json').write_text(json.dumps(info,indent=2)+'\n')
    shutil.copyfile(outputs[-1].with_suffix('.png'),args.output.with_suffix('.png'))
    # Preserve segment logs/metadata, remove redundant intermediate video files.
    for p in outputs: p.unlink()
    print('[RENDER COMPLETE]',info['candidate_id'],'segments',count,flush=True)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preview', action='store_true', help='Render initial and final frames only')
    parser.add_argument('--gpu',action='store_true',help='Require an NVIDIA OpenGL context; bypass CPU segment splitting')
    parser.add_argument('--frame-start',type=int,default=0,help=argparse.SUPPRESS)
    parser.add_argument('--frame-stop',type=int,help=argparse.SUPPRESS)
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    record = json.loads((args.capture/'recording.json').read_text())
    total_motion_frames = int(math.ceil(record['simulated_duration_s']/2*30))
    if not args.gpu and not args.preview and args.frame_stop is None and total_motion_frames>=600:
        render_parts(args,total_motion_frames); return
    with np.load(args.scene, allow_pickle=False) as archive:
        geometry = {key: archive[key] for key in archive.files}
    manifest = json.loads(args.scene.with_suffix('.json').read_text())
    data = np.load(args.capture/'motion.npz', allow_pickle=False)
    trace = json.loads((args.capture/'trace.json').read_text())
    model_update = record['validation'].get('model_update', {})
    if not (record['validation']['reexecution']['passed'] or
            (model_update.get('enabled') and model_update.get('passed'))):
        raise RuntimeError('Refusing to render a mismatched reexecution')
    bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_WORKBENCH'
    scene.render.resolution_x = 640; scene.render.resolution_y = 480; scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'; scene.render.image_settings.color_mode = 'RGB'
    scene.render.threads_mode = 'FIXED'; scene.render.threads = 4
    shading = scene.display.shading
    shading.light = 'STUDIO'; shading.studiolight_rotate_z = .6
    shading.color_type = 'MATERIAL'; shading.show_shadows = False; shading.show_cavity = True
    shading.cavity_type = 'WORLD'; shading.curvature_ridge_factor = 1.1; shading.curvature_valley_factor = 1.
    shading.background_type = 'WORLD'; scene.world.color = (.055, .065, .075)
    shading.show_specular_highlight = True
    scene.display.render_aa = '8'
    scene.view_settings.view_transform = 'Standard'; scene.view_settings.exposure = 0; scene.view_settings.gamma = 1
    materials = {}; dynamic = []
    for item in manifest['objects']:
        key = item['key']; vertices = geometry[key+'_vertices']; counts = geometry[key+'_counts']; indices = geometry[key+'_indices']
        mesh = bpy.data.meshes.new(key)
        mesh.vertices.add(len(vertices)); mesh.vertices.foreach_set('co', vertices.reshape(-1))
        mesh.loops.add(len(indices)); mesh.loops.foreach_set('vertex_index', indices)
        mesh.polygons.add(len(counts)); mesh.polygons.foreach_set('loop_start', np.r_[0, np.cumsum(counts[:-1])])
        mesh.polygons.foreach_set('loop_total', counts)
        mesh.polygons.foreach_set('use_smooth', np.ones(len(counts), dtype=bool))
        mesh.update()
        obj = bpy.data.objects.new(key, mesh); scene.collection.objects.link(obj)
        color = tuple(item['color'])
        if color not in materials:
            material = bpy.data.materials.new('USD constant '+str(color)); material.diffuse_color = (*color, 1.)
            materials[color] = material
        obj.data.materials.append(materials[color]); obj['usd_prim'] = item['path']
        if item['binding']['kind'] != 'static': dynamic.append((obj, item))
    rest = geometry['body_rest']; rest_r = rotation(rest[:, 3:7])
    elastic_rest = geometry['elastic_visual_rest']; elastic_r = rotation(elastic_rest[:, 3:7])
    center = np.asarray(manifest['target_center'], dtype=float)
    focus = center + np.array([.005, 0, .012])
    views = [dict(name='OVERVIEW', eye=center+np.array([1.1, -1.4, .75]), target=center+np.array([.30, 0, -.03]), lens=28.),
             dict(name='HOOK CLOSE-UP', eye=focus+np.array([.18, -.18, .065]), target=focus, lens=45.)]
    cameras = []
    for view in views:
        camera_data = bpy.data.cameras.new(view['name']); camera_data.lens = view['lens']; camera_data.sensor_width = 36
        camera_data.clip_start = .003; camera_data.clip_end = 100
        camera = bpy.data.objects.new(view['name'], camera_data); scene.collection.objects.link(camera)
        camera.location = view['eye']; camera.rotation_euler = Vector(view['target']-view['eye']).to_track_quat('-Z', 'Y').to_euler()
        cameras.append(camera)
    # Keep the viewport's mesh buffers alive across frames. Background render()
    # rebuilds millions of original CAD triangles for each camera and frame.
    offscreen = None
    renderer_device = 'background OpenGL'
    if not bpy.app.background:
        import gpu
        renderer_device = gpu.platform.renderer_get()
        if args.gpu and 'NVIDIA' not in renderer_device.upper():
            raise RuntimeError('GPU rendering requested, but the context is '+renderer_device)
        print('[RENDER DEVICE]',renderer_device,flush=True)
        area = next(a for a in bpy.context.screen.areas if a.type == 'VIEW_3D')
        region = next(r for r in area.regions if r.type == 'WINDOW')
        space = area.spaces.active; space.overlay.show_overlays = False; space.show_gizmo = False
        space.shading.type = 'SOLID'
        for key in ('light','color_type','show_shadows','show_cavity','cavity_type',
                    'curvature_ridge_factor','curvature_valley_factor','background_type','show_specular_highlight'):
            setattr(space.shading,key,getattr(shading,key))
        space.clip_start = .003; space.clip_end = 100
        offscreen = gpu.types.GPUOffScreen(640,480)
    elif args.gpu:
        raise RuntimeError('GPU video rendering requires the offscreen viewport context')
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font = ImageFont.truetype(font_path, 19); small = ImageFont.truetype(font_path, 16)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.capture/'render_frame.png'
    fps = 30; speed = 2.; duration = record['simulated_duration_s']
    frame_count = int(math.ceil(duration/speed*fps))
    sample_times = np.arange(frame_count)*speed/fps
    # Stored poses are at precisely 15 simulation Hz. No interpolation invents contacts.
    indices = np.searchsorted(data['times_s'], sample_times+1e-8, side='right')-1
    indices = np.maximum(indices, 0)
    stop = frame_count if args.frame_stop is None else args.frame_stop
    if not 0 <= args.frame_start < stop <= frame_count: raise ValueError('Invalid frame interval')
    last_segment = stop == frame_count
    indices = indices[args.frame_start:stop]
    if last_segment: indices = np.r_[indices, len(data['times_s'])-1]  # exact endpoint for final still
    if args.preview: indices = np.asarray([0, len(data['times_s'])-1])
    encoder = None
    if not args.preview:
        encoder = subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y',
            '-f','rawvideo','-pix_fmt','rgb24','-s','1280x612','-r',str(fps),'-i','-',
            '-an','-c:v','libx264','-preset','veryfast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(args.output)], stdin=subprocess.PIPE)
    started = time.monotonic(); final_canvas = None
    try:
        for local_frame, idx in enumerate(indices):
            frame = args.frame_start+local_frame
            poses = data['poses'][idx]; rotations = rotation(poses[:, 3:7]); delta_r = rotations @ rest_r.transpose(0,2,1)
            delta_t = poses[:, :3]-np.einsum('nij,nj->ni', delta_r, rest[:, :3])
            offset = manifest['elastic_start']; er = rotations[offset:] @ elastic_r.transpose(0,2,1)
            for obj, item in dynamic:
                key = item['key']; binding = item['binding']
                if binding['kind'] == 'rigid':
                    owner = binding['body']; transform = np.eye(4); transform[:3,:3] = delta_r[owner]; transform[:3,3] = delta_t[owner]
                    obj.matrix_world = Matrix(transform.tolist())
                else:
                    ids = geometry[key+'_skin_ids']; weights = geometry[key+'_skin_weights']; points = geometry[key+'_skin_points']
                    deformed = np.einsum('nkij,nkj->nki', er[ids], points[:,None,:]-elastic_rest[ids,:3])+poses[offset:,:3][ids]
                    deformed = (deformed*weights[:,:,None]).sum(axis=1).astype(np.float32)
                    obj.data.vertices.foreach_set('co', deformed.reshape(-1)); obj.data.update()
            time_s = float(data['times_s'][idx]); tick = min(max(int(round(time_s/record['control_dt']))-1, 0),len(trace)-1)
            current = trace[tick]; canvas = Image.new('RGB',(1280,612),(21,27,34)); draw = ImageDraw.Draw(canvas)
            if time_s == 0: current = dict(current,phase='initial',target_displacement_m=0.,main_stem_displacement_m=0.)
            p = record['parameters']
            repair_label = ' | repaired wire colliders' if model_update.get('enabled') else ''
            draw.text((14,8), f"{record['candidate_id']} | Tomato_05 | 2x | PhysX / {'GPU' if args.gpu else 'CPU'} visual replay{repair_label}", font=font, fill='white')
            draw.text((14,35), f"az {p['azimuth_deg']:+g} deg   elev {p['elevation_deg']:+g}   roll {p['roll_deg']:+g}   pitch {p['pitch_deg']:+g}   offset mm {np.round(np.asarray(p['offset_xyz_m'])*1000,1).tolist()}   pre-hook {p['pre_hook_distance_m']*1000:g} mm",font=small,fill=(195,205,218))
            for view_index, camera in enumerate(cameras):
                scene.camera = camera
                if offscreen:
                    bpy.context.view_layer.update()
                    projection = camera.calc_matrix_camera(bpy.context.evaluated_depsgraph_get(),x=640,y=480)
                    offscreen.draw_view3d(scene,bpy.context.view_layer,space,region,camera.matrix_world.inverted(),projection,do_color_management=True)
                    with offscreen.bind():
                        buffer = gpu.state.active_framebuffer_get().read_color(0,0,640,480,4,0,'UBYTE')
                        buffer.dimensions = 640*480*4
                        pixels = np.asarray(buffer,dtype=np.uint8).reshape(480,640,4)[::-1,:,:3].copy()
                    image = Image.fromarray(pixels)
                else:
                    scene.render.filepath = str(temporary); bpy.ops.render.render(write_still=True)
                    image = Image.open(temporary).convert('RGB')
                overlay = ImageDraw.Draw(image)
                overlay.rectangle((0,0,640,24),fill=(21,27,34)); overlay.text((8,2),views[view_index]['name'],font=small,fill='white')
                # Explicit GT centre annotations prevent confusing target 05 with nearby 07.
                for spec in manifest['fruit_specs']:
                    if spec['name'] not in ('Tomato_05','Tomato_07'): continue
                    body = spec['body']; point = poses[body,:3]+rotations[body] @ np.asarray(spec['center'])
                    ndc = world_to_camera_view(scene,camera,Vector(point)); x=int(ndc.x*640);y=int((1-ndc.y)*480)
                    if ndc.z>0 and 0<=x<640 and 26<=y<480:
                        color=(255,230,40) if spec['name']=='Tomato_05' else (65,220,255)
                        overlay.ellipse((x-5,y-5,x+5,y+5),outline=color,width=2)
                        overlay.text((max(5,min(505,x+10)),max(26,y-22)),spec['name']+' (GT)',font=small,fill=color,stroke_width=1,stroke_fill=(0,0,0))
                canvas.paste(image,(view_index*640,62))
            draw = ImageDraw.Draw(canvas)
            draw.text((14,549),f"sim t={time_s:.2f}s | {current['phase']} | target move {current['target_displacement_m']*1000:.2f} mm | main stem {current['main_stem_displacement_m']*1000:.2f} mm",font=small,fill='white')
            first = record.get('first_contact') or {}; first_time=(first.get('step',10**9)+1)*record['control_dt']
            name = record['first_contact_object'].split('/')[-2] if record.get('first_contact_object') and time_s>=first_time else '(none yet)'
            draw.text((14,578),f"First contact: {name} | final: {record['result']} | retained hook: {record['retained_hook']}",font=small,fill=(255,185,95))
            if args.preview: canvas.save(args.output.with_name(args.output.stem+('_initial.png' if local_frame==0 else '_final.png')))
            elif frame < frame_count: encoder.stdin.write(np.asarray(canvas).tobytes())
            final_canvas = canvas
            if local_frame%60==0: print('[RENDER PROGRESS]',record['candidate_id'],frame,frame_count,'elapsed',round(time.monotonic()-started,1),flush=True)
        if encoder and last_segment:
            # Exact terminal state shown as a labelled still, outside the 2x motion timeline.
            draw=ImageDraw.Draw(final_canvas); draw.rectangle((0,542,1280,612),fill=(21,27,34))
            draw.text((14,549),'END OF MOTION | final state held for inspection (1 s)',font=font,fill='white')
            draw.text((14,582),f"Result: {record['result']} | retained target hook: {record['retained_hook']}",font=small,fill=(255,185,95))
            for _ in range(fps): encoder.stdin.write(np.asarray(final_canvas).tobytes())
            final_canvas.save(args.output.with_suffix('.png'))
    finally:
        if encoder:
            encoder.stdin.close()
            if encoder.wait()!=0: raise RuntimeError('ffmpeg encoding failed')
        temporary.unlink(missing_ok=True)
        if offscreen: offscreen.free()
    if not args.preview:
        output_count=stop-args.frame_start+(fps if last_segment else 0)
        info=dict(candidate_id=record['candidate_id'],video=str(args.output),frames=output_count,
            motion_frames=stop-args.frame_start,final_still_seconds=int(last_segment),output_fps=fps,playback_speed=speed,
            frame_start=args.frame_start,frame_stop=stop,
            simulated_duration_s=duration,video_duration_s=output_count/fps,
            appearance=manifest['appearance'],renderer='Blender Workbench / '+renderer_device,
            render_backend='gpu' if args.gpu else 'cpu',
            reexecution_matches_original=record['validation']['reexecution']['passed'],
            collision_model_update=model_update,
            result=record['result'],retained_hook=record['retained_hook'])
        args.output.with_suffix('.json').write_text(json.dumps(info,indent=2)+'\n')
    print('[RENDER COMPLETE]',record['candidate_id'],flush=True)


if __name__ == '__main__':
    try:
        main()
    except BaseException:
        import traceback
        traceback.print_exc(); sys.stdout.flush(); sys.stderr.flush(); os._exit(1)
    # Files and encoder are closed above. Interactive GPU windows can leave
    # quit_blender waiting in the desktop event loop; no UI/save prompt is needed
    # for this disposable renderer process, and there is no .blend file to save.
    if not bpy.app.background:
        sys.stdout.flush(); sys.stderr.flush(); os._exit(0)
