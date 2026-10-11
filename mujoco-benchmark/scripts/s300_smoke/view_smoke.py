"""Render stored actual MuJoCo qpos; never run physics in a visual replay."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path)
    p.add_argument('--gui',action='store_true');p.add_argument('--camera',choices=['top','side'],default='top')
    p.add_argument('--fps',type=int,default=30);p.add_argument('--start',type=float,default=0)
    p.add_argument('--end',type=float);p.add_argument('--output',type=Path)
    a=p.parse_args()
    if a.gui:
        os.environ['MUJOCO_GL']='glfw'
    else:
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        import render_backend
    import mujoco as mj
    folder=a.trial.resolve()
    model=mj.MjModel.from_binary_path(str(folder.parent/'model.mjb'))
    d=mj.MjData(model);z=np.load(folder/'states.npz')
    summary=json.loads((folder/'summary.json').read_text())
    ts=z['time'];end=min(ts[-1],a.end) if a.end else ts[-1]
    def frame(i):
        d.time=ts[i];d.qpos[:]=z['qpos'][i];d.eq_active[:]=z['eq_active'][i]
        d.mocap_pos[:]=z['mocap_pos'][i];d.mocap_quat[:]=z['mocap_quat'][i]
        # No integration or contact solver is called for playback.
        mj.mj_kinematics(model,d);mj.mj_comPos(model,d)
    if a.gui:
        import mujoco.viewer
        frame(0)
        with mujoco.viewer.launch_passive(model,d) as viewer:
            viewer.cam.type=mj.mjtCamera.mjCAMERA_FIXED;viewer.cam.fixedcamid=model.camera(a.camera).id
            origin=time.perf_counter()
            while viewer.is_running():
                t=a.start+(time.perf_counter()-origin)%(end-a.start)
                i=int(np.clip(np.searchsorted(ts,t),0,len(ts)-1))
                with viewer.lock():frame(i)
                viewer.sync();time.sleep(1/a.fps)
    else:
        import imageio.v2 as imageio
        from PIL import Image,ImageDraw,ImageFont
        output=a.output or folder/f'actual_{a.camera}.mp4'
        if output.exists():raise FileExistsError(output)
        font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
        opt=mj.MjvOption();renderer=mj.Renderer(model,height=720,width=960)
        started=time.perf_counter();count=0
        with imageio.get_writer(output,fps=a.fps,codec='libx264',quality=7,macro_block_size=1) as writer:
            for t in np.arange(a.start,end+1e-8,1/a.fps):
                i=int(np.clip(np.searchsorted(ts,t),0,len(ts)-1));frame(i)
                renderer.update_scene(d,camera=a.camera,scene_option=opt)
                im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
                lines=[f'MuJoCo {summary["mujoco_version"]} | {summary["mode"]} | {summary["hz"]} Hz | t={ts[i]:.2f}s | detached={int((~z["eq_active"][i].astype(bool)).sum())}',
                       'Actual stored physics | prescribed tool, one truss | NOT validated robot harvest']
                if summary['first_1mm_violation'] and t>=summary['first_1mm_violation']['t']:
                    lines.append('DIAGNOSTIC: 1 mm penetration criterion exceeded')
                draw.rectangle((0,0,960,4+20*len(lines)),fill=(25,25,25))
                for j,line in enumerate(lines):draw.text((8,3+20*j),line,font=font,fill=(255,240,200))
                writer.append_data(np.asarray(im));count+=1
        renderer.close()
        report=dict(video=str(output),frames=count,fps=a.fps,playback_rate=1,render_wall_s=time.perf_counter()-started,
                    state_source=str(folder/'states.npz'),physics_reexecuted=False)
        output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))


if __name__=='__main__':main()
