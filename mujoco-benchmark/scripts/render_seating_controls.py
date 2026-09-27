"""Visualize prescribed native fixture poses, never a robot entry rollout."""
import render_backend
import argparse,json
from pathlib import Path
import numpy as np
import mujoco as mj
import imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageFont

def render_controls(root):
 root=Path(root);report=json.loads((root/'controls.json').read_text());byname={r['name']:r for r in report['fixtures']};font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
 for name,cases in [('positive_fixture',['positive']),('negative_fixtures',['fruit_only','front','penetration','wrong_pedicel','outside','rachis'])]:
  models=[]
  for case in cases:
   m=mj.MjModel.from_xml_path(str(root/f'{case}.xml'));d=mj.MjData(m);mj.mj_forward(m,d);r=mj.Renderer(m,width=400,height=300);cam=mj.MjvCamera();cam.lookat[:]=[0,0,0];cam.distance=.10;cam.azimuth=80;cam.elevation=-30
   models.append((m,d,r,cam))
  cols=1 if len(cases)==1 else 3;rows=(len(cases)+cols-1)//cols;canvas=Image.new('RGB',(cols*400,rows*360+60),(18,22,28));draw=ImageDraw.Draw(canvas)
  draw.text((8,5),'STATIC FIXTURE | NOT robot entry',font=font,fill='white')
  draw.text((8,28),'Cyan: actual rear wires | orange: test',font=font,fill='white')
  for i,(case,(m,d,r,cam)) in enumerate(zip(cases,models)):
   x=i%cols*400;y=i//cols*360+60;row=byname[case];r.update_scene(d,camera=cam);canvas.paste(Image.fromarray(r.render()),(x,y));draw.text((x+5,y+300),f'{case}: target geometry={row["accepted_target_geometry"]}',font=font,fill='white');draw.text((x+5,y+323),f'gap={row["native_gap_m"]*1000:.3f}mm | legacy={row["legacy_seated"]}',font=font,fill='yellow');r.close()
  canvas.save(root/f'{name}.png')
  with imageio.get_writer(root/f'{name}.mp4',fps=10,codec='libx264',macro_block_size=1) as w:
   for _ in range(30):w.append_data(np.asarray(canvas))
 (root/'video_scope.json').write_text(json.dumps(dict(physics_steps=0,prescribed_static_fixture=True,duration_is_display_time=True,automatic_entry_success=False,training_eligible=False),indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();render_controls(a.root)
