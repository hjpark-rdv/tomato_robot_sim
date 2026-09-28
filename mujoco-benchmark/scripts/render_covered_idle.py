"""Render recorded original/covered idle qpos with the same inspection camera."""
import render_backend
import argparse,json
from pathlib import Path
import numpy as np
import mujoco as mj
import imageio.v2 as imageio
from PIL import Image,ImageDraw

def render(root,source,covered):
    pair=['g410','neighbor_truss_collision_fruit_p19_t02_g052']
    models=[];data=[];states=[];renderers=[]
    for label,path in [('original',source),('covered',covered)]:
        m=mj.MjModel.from_binary_path(str(path/'model.mjb'));d=mj.MjData(m)
        s=np.load(root/f'{label}_idle.npz');d.qpos[:]=s['qpos'][0];mj.mj_forward(m,d)
        models.append(m);data.append(d);states.append(s);renderers.append(mj.Renderer(m,width=640,height=480,max_geom=20000))
    m,d=models[1],data[1];gid=m.geom(pair[0]).id;other=m.geom(pair[1]).id
    center=(d.geom_xpos[gid]+d.geom_xpos[other])/2
    mapping=dict(robot_geom=pair[0],robot_body=m.body(m.geom_bodyid[gid]).name,neighbor_geom=pair[1],robot_geom_center=d.geom_xpos[gid].tolist(),neighbor_geom_center=d.geom_xpos[other].tolist(),scope='recorded idle physics, no approach/manipulation')
    (root/'blocker_mapping.json').write_text(json.dumps(mapping,indent=2))
    opt=mj.MjvOption();opt.geomgroup[2]=0;opt.geomgroup[3]=1
    # Render-only highlighting; no physics is stepped and saved MJB is unchanged.
    for model in models:
        model.geom_group[model.geom(pair[0]).id]=0;model.geom_rgba[model.geom(pair[0]).id]=[0,1,1,1]
    m.geom_group[other]=0;m.geom_rgba[other]=[1,0,.5,1]
    cam=mj.MjvCamera();cam.lookat[:]=center;cam.distance=.42;cam.azimuth=30;cam.elevation=-15
    writer=imageio.get_writer(root/'idle_ab.mp4',fps=10,codec='libx264',quality=8,macro_block_size=1)
    for k in range(21):
        canvas=Image.new('RGB',(1280,540),(20,20,25));draw=ImageDraw.Draw(canvas)
        draw.text((10,5),f'RECORDED IDLE PHYSICS | t={k/10:.1f}s | no approach attempt | cyan=robot collision, pink=neighbor fruit',fill='white')
        draw.text((10,25),'LEFT: ORIGINAL (neighbor fruit visual only) | RIGHT: COVERED (collision enabled)',fill='white')
        draw.text((10,42),'Initial robot-fruit penetration 2.234 mm. Background visuals hidden; proxies shown. No new physics during rendering.',fill='yellow')
        for j in range(2):
            idx=int(np.argmin(abs(states[j]['times_s']-k/10)))
            data[j].qpos[:]=states[j]['qpos'][idx];mj.mj_forward(models[j],data[j])
            renderers[j].update_scene(data[j],camera=cam,scene_option=opt)
            canvas.paste(Image.fromarray(renderers[j].render()),(j*640,60))
        if k in (0,10,20):canvas.save(root/f'idle_ab_{k:02d}.png')
        writer.append_data(np.array(canvas))
    writer.close()
    # Additional visual/proxy comparison: show the full background, same t=0.
    canvas=Image.new('RGB',(1280,540),(20,20,25));draw=ImageDraw.Draw(canvas)
    draw.text((10,8),'t=0 | full visual background | LEFT original / RIGHT covered | cyan mounting collision, pink neighboring fruit proxy',fill='white')
    opt.geomgroup[2]=1;opt.geomgroup[3]=0
    for j in range(2):
        data[j].qpos[:]=states[j]['qpos'][0];mj.mj_forward(models[j],data[j])
        renderers[j].update_scene(data[j],camera=cam,scene_option=opt)
        canvas.paste(Image.fromarray(renderers[j].render()),(j*640,60))
    canvas.save(root/'initial_full_visual.png')
    for r in renderers:r.close()
    print(json.dumps(mapping),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('source',type=Path);p.add_argument('covered',type=Path);a=p.parse_args();render(a.output,a.source,a.covered)
