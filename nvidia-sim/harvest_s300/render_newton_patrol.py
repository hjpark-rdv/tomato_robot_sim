"""Render actual recorded Newton poses with the existing GPU camera; no physics."""
import sys,json,argparse,subprocess,time
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
import warp as wp
from newton_inputs import sha256
from newton_mounted_camera import MountedCamera
from newton_geometric_harvest import sensor_poses,volume_bounds
from newton.sensors import SensorCamera
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--fps',type=int,default=6)
p.add_argument('--eye-offset',type=float,nargs=3,default=[.2,-.65,1.6],help='Render camera XYZ with Y relative to the moving rail station')
p.add_argument('--look-at-offset',type=float,nargs=3,default=[-.38,.02,.95],help='Render target XYZ with Y relative to the moving rail station')
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
report=json.loads((a.run/'summary.json').read_text());camera=MountedCamera(Path(report['assets']),scale=.5)
LOW,HIGH=volume_bounds(camera.assets.get('geometric_harvest'))
w,h=800,450;K=np.array([[500.,0.,w/2],[0.,500.,h/2],[0.,0.,1.]])
camera.rays=SensorCamera.compute_camera_rays_pinhole_opencv(w,h,K[0,0],K[1,1],K[0,2],K[1,2],device=camera.model.device)
rgb=camera.sensor.allocate_image_color(1,w,h)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
cam=[]
for item in list(a.run.glob('step_*/camera/observation.json'))+list(a.run.glob('step_*/rolling_camera/*/observation.json')):
 d=json.loads(item.read_text());cam.append((d['timestamp_s'],item.parent/'recognition.png'))
cam.sort();events=report['capture_events'];ledger=set();started=time.perf_counter();frames=0;sources=[]
proc=subprocess.Popen(['ffmpeg','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1040x450','-r',str(a.fps),'-i','pipe:0','-an','-c:v','libx264','-threads','2','-crf','22','-pix_fmt','yuv420p',str(a.output)],stdin=subprocess.PIPE)
try:
 for step in sorted(a.run.glob('step_*')):
  path=step/'physics/states.npz'
  if not path.exists():continue
  with np.load(path,allow_pickle=False) as saved:
   poses=saved['poses'];times=saved['time'];phases=saved['phase'];rail=saved['robot_q'][:,0]
   proof=json.loads((step/'inputs/inputs.json').read_text());mapping=proof['source_body_to_native_name']
   names=[mapping.get(str(x),str(x)) for x in saved['body_paths']];lookup={n:i for i,n in enumerate(names)}
   sources.append(str(path))
   for frame_number in range(int(np.ceil(times[0]*a.fps)),int(np.ceil(times[-1]*a.fps))):
    t=frame_number/a.fps
    i=max(0,int(np.searchsorted(times,t,side='right')-1));station=float(rail[i])
    collected={r['name'] for e in events if e.get('reached') and e['timestamp_s']<=t for r in e['harvested']}
    body=sensor_poses(dict(zip(names,poses[i])),collected)
    camera.state.body_q.assign(np.asarray([body[n] for n in camera.body_names],np.float32))
    eye=np.asarray(a.eye_offset)+[0.,station,0.];target=np.asarray(a.look_at_offset)+[0.,station,0.]
    z=target-eye;z/=np.linalg.norm(z);x=np.cross(z,[0.,0.,1.]);x/=np.linalg.norm(x);y=np.cross(z,x)
    optical=np.column_stack((x,y,z));gl=optical@np.diag([1.,-1.,-1.])
    camera.transforms.assign(np.array([np.r_[eye,Rotation.from_matrix(gl).as_quat()]],np.float32))
    camera.model.bvh_refit_shapes(camera.state)
    camera.sensor.update(camera.state,camera.transforms,camera.rays,color_image=rgb)
    wp.synchronize_device(camera.model.device)
    raw=rgb.numpy()[0].view(np.uint8).reshape(h,w,4)[:,:,:3]
    srgb=raw.astype(float)/255.;lin=np.where(srgb<=.04045,srgb/12.92,((srgb+.055)/1.055)**2.4)
    lin=np.clip(lin,0.,1.)
    raw=(255*np.where(lin<=.0031308,lin*12.92,1.055*lin**(1/2.4)-.055)).astype(np.uint8)
    image=Image.new('RGB',(1040,h),(24,28,30));image.paste(Image.fromarray(raw),(0,0));draw=ImageDraw.Draw(image)
    from itertools import product,combinations
    vertices=np.array(list(product(*zip(LOW,HIGH))));tool=body['Tool_fixed']
    world=vertices@Rotation.from_quat(tool[3:]).as_matrix().T+tool[:3]
    points=(world-eye)@optical;projected=points@K.T
    uv=projected[:,:2]/projected[:,2:]
    for aa,bb in combinations(range(8),2):
     if np.count_nonzero(vertices[aa]!=vertices[bb])==1 and points[aa,2]>.01 and points[bb,2]>.01:
      draw.line([tuple(uv[aa]),tuple(uv[bb])],fill=(255,215,40),width=1)
    draw.rectangle((0,0,800,38),fill=(24,28,30));draw.text((8,3),'Actual recorded Newton GPU poses | 1x simulation-time replay',font=font,fill='white')
    draw.text((8,20),f't={t:.2f}s | {phases[i]} | assumed collected={len(collected)}',font=font,fill='white')
    available=[item for item in cam if item[0]<=t+1e-4]
    if available:
     ts,file=available[-1];im=Image.open(file).convert('RGB');im.thumbnail((230,380));image.paste(im,(805,48))
     draw.text((806,5),'Mounted camera: latest',font=font,fill='white');draw.text((806,23),f'observation t={ts:.2f}s',font=font,fill='white')
    if frames==140:image.save(a.output.with_suffix('.preview.png'))
    proc.stdin.write(np.asarray(image).tobytes());frames+=1
   print('Rendered',step.name,'frames',frames,flush=True)
 proc.stdin.close()
 if proc.wait():raise RuntimeError('ffmpeg failed')
 proof=dict(renderer_source_sha256=sha256(__file__),actual_recorded_states=True,physics_steps_executed=0,geometric_assumed_harvest=True,fps=a.fps,frames=frames,render_wall_s=time.perf_counter()-started,sources=sources,eye_offset=a.eye_offset,look_at_offset=a.look_at_offset)
 a.output.with_suffix('.render.json').write_text(json.dumps(proof,indent=2))
 print(proof,flush=True)
finally:
 if proc.poll() is None:proc.kill()
