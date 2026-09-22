"""2x videos of freshly recorded robot/plant states, no second physics rollout."""
import sys,json,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import render_backend
import mujoco as mj
import numpy as np
import imageio.v2 as imageio
from PIL import Image,ImageDraw
from render_results import marked_frame


def render(root):
    results=sorted(json.loads((root/'results.json').read_text()),key=lambda r:r['trial_id']);chosen={}
    for r in results:chosen.setdefault(r['candidate_id'],r)
    (root/'videos').mkdir(exist_ok=True);m=mj.MjModel.from_binary_path(str(root/'replay_assets/model.mjb'));d=mj.MjData(m);m.vis.headlight.ambient[:]=.5;m.vis.headlight.diffuse[:]=.6
    renderer=mj.Renderer(m,height=360,width=480);fruit=m.body('Tomato_05').id;reference=json.loads((root/'replay_assets/reference.json').read_text());offset=np.array(reference['fruit_specs'][4]['center'])
    cameras=[]
    for angle,distance,elevation in [(-45,.48,-15),(-90,.22,-30)]:
        c=mj.MjvCamera();c.type=mj.mjtCamera.mjCAMERA_FREE;c.distance=distance;c.azimuth=angle;c.elevation=elevation;cameras.append(c)
    for cid,r in chosen.items():
        with np.load(root/'candidates'/r['trial_id']/'states.npz') as z:times=z['times_s'];q=z['qpos']
        d.qpos[:]=q[0];mj.mj_forward(m,d);point=d.xpos[fruit]+d.xmat[fruit].reshape(3,3)@offset
        for c in cameras:c.lookat[:]=point
        indices=np.unique(np.r_[np.searchsorted(times,np.arange(0,times[-1],1/15)).clip(0,len(q)-1),len(q)-1])
        with imageio.get_writer(root/'videos'/f'{r["trial_id"]}.mp4',fps=30,codec='libx264',quality=8,macro_block_size=1) as writer:
            for i in indices:
                d.qpos[:]=q[i];mj.mj_forward(m,d);point=d.xpos[fruit]+d.xmat[fruit].reshape(3,3)@offset;panels=[]
                for c in cameras:renderer.update_scene(d,camera=c);panels.append(marked_frame(renderer,point))
                im=Image.new('RGB',(960,400),'#15232d');im.paste(Image.fromarray(panels[0]),(0,40));im.paste(Image.fromarray(panels[1]),(480,40));draw=ImageDraw.Draw(im)
                draw.text((10,7),f'{cid} | fresh physics | t={times[i]:.2f}s | 2x | Tomato_05 (yellow)',fill='white');draw.text((10,23),f'Center entry: {r["physics"]["center_entered"]} | max displacement {r["physics"]["target_center_max_displacement_m"]*1000:.2f} mm | 120 Hz',fill='white')
                writer.append_data(np.array(im))
            im.save(root/'videos'/f'{r["trial_id"]}_final.jpg')
            for _ in range(30):writer.append_data(np.array(im))
        print('[영상 저장]',cid,root/'videos'/f'{r["trial_id"]}.mp4',flush=True)
    renderer.close()
    from test_physics import report
    report(root)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();render(a.run)
