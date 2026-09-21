"""Offline pose replay + separate RGB-D cost measurement; no hidden re-simulation."""
import render_backend
import argparse,json,time,copy
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation as R
import imageio.v2 as imageio
HOME=Path(__file__).resolve().parents[1]

def marked_frame(renderer,point):
    im=Image.fromarray(renderer.render().copy());c=mj.mjv_averageCamera(renderer.scene.camera[0],renderer.scene.camera[1])
    forward=np.array(c.forward);up=np.array(c.up);right=np.cross(forward,up);v=point-np.array(c.pos);z=float(v@forward)
    if z>0:
        f=im.height*c.frustum_near/(c.frustum_top-c.frustum_bottom);u=im.width/2+f*(v@right)/z;y=im.height/2-f*(v@up)/z
        if 0<u<im.width and 0<y<im.height:
            draw=ImageDraw.Draw(im);draw.ellipse((u-6,y-6,u+6,y+6),outline='yellow',width=2);draw.text((u+9,y-8),'Tomato_05 (GT)',fill='yellow')
    return np.asarray(im)

def visual_model(model):
    xml=ET.parse(model).getroot();old=xml.find('worldbody');new=ET.Element('worldbody')
    for item in old:
        if item.tag in ('light','camera'):new.append(copy.deepcopy(item))
    for item in old.iter('body'):
        b=ET.SubElement(new,'body',name=item.get('name'),mocap='true')
        for geom in item.findall('geom'):
            g=copy.deepcopy(geom);g.set('contype','0');g.set('conaffinity','0');b.append(g)
    xml.remove(old);xml.append(new)
    for name in ('equality','contact','actuator','sensor'):
        node=xml.find(name)
        if node is not None:xml.remove(node)
    return mj.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'))

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--reference',type=Path,default=HOME/'assets/reference/reference.json');p.add_argument('--model',type=Path,default=HOME/'models/plant_original_equivalent.xml');p.add_argument('--ids',default='fixture_stem_push,fixture_fruit_collision,fixture_pedicel_contact,fixture_entry_lift');p.add_argument('--speed',type=float,default=1.);a=p.parse_args()
    ref=json.loads(a.reference.read_text());m=visual_model(a.model);m.vis.headlight.ambient[:]=.4;m.vis.headlight.diffuse[:]=.6;d=mj.MjData(m);ids=np.array([m.body(b['name']).mocapid[0] for b in ref['bodies']]+[m.body('Hook').mocapid[0]])
    # Inspection view hides only leaf/stub meshes, preserving recorded physics.
    for shape in ref['shapes']:
        if shape['type']=='mesh' and shape['body']!=ref['tool_path']:m.geom_group[m.geom(shape['name']).id]=5
    overview=mj.MjvOption();overview.geomgroup[:]=1
    inspection=mj.MjvOption();inspection.geomgroup[:]=1;inspection.geomgroup[5]=0
    out=a.run/('videos' if a.speed==1. else f'videos_{a.speed:g}x');out.mkdir(exist_ok=True);renderer=mj.Renderer(m,height=480,width=640);html=['<meta charset="utf-8"><title>식물 엔진 비교 영상</title><style>body{background:#17202b;color:white;font:18px sans-serif}video{width:960px;max-width:95vw}</style><h1>물리 상태 기록 재생</h1><p>원본 물리 상태를 재생한 영상입니다. 영상 출력 시간은 physics benchmark에 포함되지 않습니다. 오른쪽은 접촉을 확인하기 위해 잎/잘린 가지 mesh만 숨긴 화면입니다. 물리 상태는 바꾸지 않았습니다.</p>']
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
    spec=next(s for s in ref['fruit_specs'] if s['name']=='Tomato_05');fruitidx=[b['path'] for b in ref['bodies']].index(spec['path'])
    summary=json.loads((a.run/'summary.json').read_text())
    variant='OPTIMIZED: leaf collisions OFF, fruit welds merged' if 'optimized' in str(summary.get('model',a.run.name)) else 'Original structure, break disabled'
    for name in a.ids.split(','):
        folder=a.run/name
        if not (folder/'states.npz').exists():continue
        states=np.load(folder/'states.npz');r=json.loads((folder/'result.json').read_text());times=states['times_s'];poses=states['poses'];index=np.unique(np.searchsorted(times,np.arange(0,times[-1],1/15)).clip(0,len(times)-1))
        writer=imageio.get_writer(out/(name+'.mp4'),fps=15*a.speed,codec='libx264',quality=7,macro_block_size=1)
        camera=mj.MjvCamera();camera.type=mj.mjtCamera.mjCAMERA_FREE;target=np.asarray(spec['pose'][:3])+R.from_quat(np.asarray(spec['pose'])[[4,5,6,3]]).apply(spec['center']);camera.lookat[:]=target;camera.distance=1.5;camera.azimuth=70;camera.elevation=-15
        close=mj.MjvCamera();close.type=mj.mjtCamera.mjCAMERA_FREE;close.lookat[:]=target;close.distance=.19;close.azimuth=-90;close.elevation=-25
        if name in ('fixture_stem_push','fixture_large_displacement'):
            focus=np.array(next(b['pose'][:3] for b in ref['bodies'] if b['name']=='STEM_MainStem_12'))
            camera.lookat[:]=(focus+target)/2
            close=mj.MjvCamera();close.type=mj.mjtCamera.mjCAMERA_FREE;close.lookat[:]=focus;close.distance=.25;close.azimuth=-90;close.elevation=-25
        began=time.perf_counter()
        for k in index:
            d.mocap_pos[ids]=poses[k,:,:3];d.mocap_quat[ids]=poses[k,:,3:][:,[3,0,1,2]];mj.mj_forward(m,d)
            fp=poses[k,fruitidx];centre=fp[:3]+R.from_quat(fp[3:]).apply(spec['center'])
            renderer.update_scene(d,camera=camera,scene_option=overview);left=marked_frame(renderer,centre);renderer.update_scene(d,camera=close,scene_option=inspection);right=marked_frame(renderer,centre)
            canvas=Image.new('RGB',(1280,540),(20,28,38));canvas.paste(Image.fromarray(left),(0,60));canvas.paste(Image.fromarray(right),(640,60));draw=ImageDraw.Draw(canvas)
            draw.text((12,3),f"{r['engine']} | {r['hz']} Hz | {name} | {a.speed:g}x | t={times[k]:.2f}s",fill='white',font=font)
            draw.text((12,25),f"physics wall={r['physics_only_wall_s']:.2f}s RTF={r['physics_only_rtf']:.2f} | max stem={r['main_stem_max_displacement_m']*1000:.2f}mm | {r['result']}",fill='white',font=small)
            draw.text((12,43),variant+' | recorded states | left: plant / right: leaves hidden for inspection',fill='white',font=small);writer.append_data(np.asarray(canvas))
        writer.close();html.append(f'<h2>{name}: {r["result"]}</h2><video controls src="{name}.mp4"></video>')
        print('[영상 저장]',name,round(time.perf_counter()-began,1),'초',flush=True)
    (out/'index.html').write_text('\n'.join(html));(out/'render_backend.json').write_text(json.dumps(render_backend.info(),indent=2));renderer.close()

if __name__=='__main__':main()
