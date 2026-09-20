"""Record all completed coarse candidates at 2x using actual PhysX + CPU rendering."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
import csv
import html
import json
import os
from pathlib import Path
import subprocess
import time

HERE = Path(__file__).resolve().parent


def command_run(command, log, environment=None):
    with log.open('w') as stream:
        done = subprocess.run(command, env=environment, stdout=stream, stderr=subprocess.STDOUT)
    if done.returncode: raise RuntimeError(f'Process failed ({done.returncode}): {log}')


def probe(path):
    data = json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames',
        '-select_streams','v:0','-show_entries','stream=width,height,nb_read_frames,r_frame_rate,duration',
        '-of','json',str(path)],text=True))['streams'][0]
    if (data['width'],data['height'],data['r_frame_rate']) != (1280,612,'30/1'):
        raise RuntimeError('Unexpected video stream: '+str(data))
    return data


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--run-dir',type=Path,default=HERE/'runs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_tomato05_31cases_video_2x'))
    parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--capture-only',action='store_true',help='Record all physics states before offline rendering')
    parser.add_argument('--render-only',action='store_true',help='Render saved states; wait for an active capture batch without launching duplicate physics')
    parser.add_argument('--gpu',action='store_true',help='Render missing videos with NVIDIA OpenGL on the existing physical display')
    parser.add_argument('--display',default=':0',help='Display for --gpu (default :0)')
    args=parser.parse_args(); source=args.source.resolve(); root=args.run_dir.resolve()
    if args.capture_only and args.render_only: parser.error('Choose capture-only or render-only')
    if not 1<=args.workers<=8: parser.error('workers must be 1..8')
    parameters=json.loads((source/'experiment.json').read_text())['coarse_candidates']
    if len(parameters)!=31: raise ValueError('Expected the complete 31-candidate experiment')
    for folder in ('captures','videos','logs'): (root/folder).mkdir(parents=True,exist_ok=True)
    config=dict(source_experiment=str(source),candidate_ids=[p['candidate_id'] for p in parameters],
        original_physics_unchanged=True,playback_speed=2,output_fps=30,simulation_capture_fps=15,
        renderer='Blender Workbench, original USD meshes and actual PhysX body states',
        visual_changes_only='Constant USD material colours and studio shading; not Isaac RTX camera footage',
        simulator_environment='/root/isaaclab_env',software_display_dependency='xvfb (separate from user DISPLAY=:0)')
    config_path=root/'video_experiment.json'
    if config_path.exists():
        previous=json.loads(config_path.read_text())
        if previous.get('renderer')=='CPU Blender Workbench, original USD meshes and actual PhysX body states':
            previous['renderer']=config['renderer']
        if previous!=config: raise ValueError('Output folder belongs to another recording')
    config_path.write_text(json.dumps(config,indent=2)+'\n')
    if args.gpu:
        env=os.environ.copy();env.pop('LIBGL_ALWAYS_SOFTWARE',None)
        env.update(DISPLAY=args.display,__GLX_VENDOR_LIBRARY_NAME='nvidia')
        context=subprocess.check_output(['glxinfo','-B'],env=env,text=True,stderr=subprocess.STDOUT)
        if 'OpenGL renderer string: NVIDIA' not in context: raise RuntimeError('No NVIDIA OpenGL renderer: '+context)
        (root/'gpu_resume.json').write_text(json.dumps(dict(display=args.display,context=context,
            resumed_at=datetime.now().isoformat(),preserves_completed_videos=True),indent=2)+'\n')
    def capture(p, export=False):
        name=p['candidate_id']; folder=root/'captures'/name
        if args.render_only:
            deadline=time.monotonic()+7200
            while not (folder/'recording.json').exists():
                if time.monotonic()>deadline: raise TimeoutError('No completed physics recording: '+name)
                time.sleep(2)
        if (folder/'recording.json').exists():
            existing=json.loads((folder/'recording.json').read_text())
            if not existing['validation']['reexecution']['passed'] or existing['parameters']!=p:
                raise ValueError('Existing recording mismatch: '+name)
            return
        command=[str(HERE.parent/'run_ring_rl.sh'),'--mode','pose-video','--headless',
            '--pose-candidate',str(source/'inputs'/(name+'.json')),'--pose-video-source',str(source),'--run-dir',str(folder)]
        if export: command += ['--pose-video-export',str(root/'scene.npz')]
        print('[CAPTURE START]',name,flush=True)
        command_run(command,root/'logs'/(name+'_physics.log'))
    if not (root/'scene.npz').exists(): capture(parameters[0],True)
    if not (root/'scene.json').exists(): raise RuntimeError('Scene export incomplete')
    if args.capture_only:
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures={pool.submit(capture,p):p for p in parameters}
            for future in as_completed(futures):
                future.result(); print('[CAPTURE SAVED]',futures[future]['candidate_id'],flush=True)
        return
    def trial(p):
        name=p['candidate_id']; capture(p)
        video=root/'videos'/(name+'_2x.mp4'); metadata=video.with_suffix('.json')
        if not (video.exists() and metadata.exists()):
            print('[RENDER START]',name,flush=True)
            env=os.environ.copy()
            if args.gpu:
                env.pop('LIBGL_ALWAYS_SOFTWARE',None);env.pop('LP_NUM_THREADS',None)
                env.update(DISPLAY=args.display,__GLX_VENDOR_LIBRARY_NAME='nvidia',OMP_NUM_THREADS='2')
                launcher=['blender','--window-geometry','0','0','640','480']
            else:
                env.update(LIBGL_ALWAYS_SOFTWARE='1',__GLX_VENDOR_LIBRARY_NAME='mesa',LP_NUM_THREADS='2',OMP_NUM_THREADS='2')
                launcher=['xvfb-run','-a','blender']
            command=launcher+['-t','2','--python',str(HERE/'pose_video_render.py'),'--',
                '--scene',str(root/'scene.npz'),'--capture',str(root/'captures'/name),'--output',str(video)]
            if args.gpu: command+=['--gpu']
            command_run(command,root/'logs'/(name+'_render.log'),env)
        info=json.loads(metadata.read_text()); decoded=probe(video)
        if int(decoded['nb_read_frames'])!=info['frames']: raise RuntimeError('Video truncated: '+name)
        return dict(candidate_id=name,video=str(video.relative_to(root)),thumbnail=str(video.with_suffix('.png').relative_to(root)),
            result=info['result'],retained_hook=info['retained_hook'],frames=info['frames'],duration_s=info['video_duration_s'],
            source_matches=info['reexecution_matches_original'],render_backend=info.get('render_backend','cpu'))
    results=[]
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(trial,p):p for p in parameters}
        for future in as_completed(futures):
            result=future.result();results.append(result);results.sort(key=lambda r:r['candidate_id'])
            (root/'video_manifest.json').write_text(json.dumps(dict(completed=len(results),expected=31,videos=results),indent=2)+'\n')
            print('[VIDEO COMPLETE]',result['candidate_id'],str(len(results))+'/31',flush=True)
    with (root/'videos.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(results[0]));writer.writeheader();writer.writerows(results)
    cards=[]
    for row in results:
        title=html.escape(row['candidate_id']+' | '+row['result']+' | retained='+str(row['retained_hook']))
        cards.append(f'<article><h2>{title}</h2><video controls preload="none" poster="{row["thumbnail"]}" src="{row["video"]}"></video></article>')
    (root/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><title>Tomato_05 / 31 cases / 2x</title><style>body{background:#141b22;color:#eee;font-family:sans-serif;margin:24px}article{margin-bottom:32px}h2{font-size:18px}video{width:min(100%,1280px)}</style><h1>Tomato_05 — 31 cases — 2x</h1><p>Actual PhysX reexecution, original USD meshes, offline Blender studio rendering. Backend is recorded per clip (CPU / NVIDIA GPU). Yellow: target 05. Cyan: neighbor 07. Each clip ends with a labelled 1-second final-state still.</p>'+''.join(cards)+'</html>')
    print('[ALL VIDEOS COMPLETE]',root,flush=True)


if __name__=='__main__': main()
