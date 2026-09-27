"""Render nominal/recorded comparison. Never steps physics or edits source states."""
import render_backend
import argparse,json
from pathlib import Path
import numpy as np
import mujoco as mj
import imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageFont
from diagnose_contact_timing import load_engine


def authorized_force_caption(row):
    """Show permitted Rachis/touch loads as well as seating contact evidence."""
    c = row['contact_categories_private']
    force = lambda name: c.get(name, {}).get('force_sum_N', 0.)
    return (f"private SUM N fruit={force('target_fruit_touch'):.3f} "
            f"rachis={force('target_rachis_touch'):.3f} "
            f"ped-seat/touch={force('target_pedicel_contact'):.3f}/{force('target_pedicel_touch'):.3f} "
            f"forbidden={force('forbidden_contact'):.3f} | "
            f"disp={row['target_displacement_m']*1000:.2f}mm seated={row['seated']}")


def render(run,states,output,samples=None,pair=None,label='recorded',fps=10,speed=2,candidate='predicted_00000',right_caption='recorded fresh rollout qpos',highlights=None):
    e,trace,plan=load_engine(run,candidate);m=e.model;d=mj.MjData(m);mj.mj_resetData(m,d);d.qpos[e.qids]=e.initial;mj.mj_forward(m,d)
    start=d.qpos.copy();target=d.xpos[e.fruit].copy();arrays=np.load(states);times=arrays['times_s'];qpos=arrays['qpos']
    if pair is None and samples and (Path(samples).parent/'summary.json').exists():
        pair=json.loads((Path(samples).parent/'summary.json').read_text()).get('pair')
    if pair:
        for name,color in zip(pair,([1,.1,.1,.8],[0,.8,1,.8])):
            gid=m.geom(name).id;m.geom_group[gid]=0;m.geom_rgba[gid]=color
    for name,color in (highlights or {}).items():
        gid=m.geom(name).id;m.geom_group[gid]=0;m.geom_rgba[gid]=color
    rows=json.loads(Path(samples).read_text()) if samples else [];rowtimes=np.array([r['time_s'] for r in rows])
    renderer=mj.Renderer(m,height=288,width=480);cam=mj.MjvCamera();cam.type=mj.mjtCamera.mjCAMERA_FREE
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    frames=np.unique(np.r_[np.searchsorted(times,np.arange(times[0],times[-1],speed/fps)),len(times)-1]).clip(0,len(times)-1)
    writer=imageio.get_writer(output,fps=fps,codec='libx264',quality=7,macro_block_size=1)
    for number,k in enumerate(frames):
        t=float(times[k]);canvas=Image.new('RGB',(960,672),(18,22,28));draw=ImageDraw.Draw(canvas)
        phase=trace[min(round(t*60),len(trace)-1)]['phase'];info=[]
        if rows:
            r=rows[int(np.argmin(abs(rowtimes-t)))]
            if 'ABCD_distance_m' in r:
                contacts=r['recomputed_pair_contacts'];force=max([c['normal_force_N'] for c in contacts]+[0.])
                info=[f"{('/'.join(pair)) if pair else ''} | D geometry={r['ABCD_distance_m'][3]*1000:.3f}mm | private-forward Fn={force:.3f}N @ {r['time_s']:.4f}s"]
            elif 'contact_categories_private' in r:
                info=[authorized_force_caption(r)]
            elif 'target_contact' in r:info=[f"seated={r['seated']} target force={r['target_contact']} | private non-target Fn={r['non_target_force_N']:.3f}N @ {r['time_s']:.4f}s"]
            else:info=[f"legacy seated={r['seated']} target contact={r['intended_contact']} | hold={r['phase']=='hold'} | forbidden pairs={len(r['forbidden_contacts'])}"]
        draw.text((8,3),f'{label} | {e.target} | {phase} | t={t:.3f}s | {speed}x | NO NEW PHYSICS IN VIDEO',fill='white',font=font)
        draw.text((8,23),'LEFT: nominal command + initial plant | RIGHT: '+right_caption,fill='white',font=font)
        draw.text((8,43),'TOP: overview | BOTTOM: detail. Initial scene and cameras identical in both columns.',fill='white',font=font)
        draw.text((8,63),info[0] if info else '',fill='yellow',font=font)
        for col in range(2):
            d.qpos[:]=start if col==0 else qpos[k]
            if col==0:d.qpos[e.qids]=e.command(t)
            mj.mj_kinematics(m,d)
            for row in range(2):
                cam.lookat[:]=target+[0,0,.02];cam.azimuth=0 if row==0 else -45;cam.elevation=-15;cam.distance=.85 if row==0 else .24
                renderer.update_scene(d,camera=cam)
                # Mark the target only in rendering; collision geometry/data unchanged.
                if renderer.scene.ngeom<renderer.scene.maxgeom:
                    g=renderer.scene.geoms[renderer.scene.ngeom]
                    mj.mjv_initGeom(g,mj.mjtGeom.mjGEOM_SPHERE,np.full(3,.003),d.xpos[e.fruit],np.eye(3).ravel(),np.array([1,1,0,1]));renderer.scene.ngeom+=1
                canvas.paste(Image.fromarray(renderer.render()),(col*480,96+row*288))
        if number in (0,len(frames)//2,len(frames)-1):canvas.save(output.with_name(output.stem+f'_frame{number}.png'))
        writer.append_data(np.asarray(canvas))
    writer.close();renderer.close()
    output.with_suffix('.json').write_text(json.dumps(dict(source_run=str(run),states=str(states),samples=str(samples),frames=len(frames),fps=fps,speed=speed,label=label,physics_executed_by_renderer=False,pair=pair,right_caption=right_caption,highlights=highlights),indent=2))
    print('VIDEO',output,len(frames),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--states',type=Path,required=True);p.add_argument('--samples',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--label',default='recorded');p.add_argument('--candidate',default='predicted_00000')
    a=p.parse_args();render(a.run,a.states,a.output,a.samples,label=a.label,candidate=a.candidate)
