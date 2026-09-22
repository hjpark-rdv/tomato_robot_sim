"""Inspect a generated attachment scene; mouse camera control remains enabled."""
import argparse,json,time
from pathlib import Path
import numpy as np
import mujoco as mj
import mujoco.viewer
from attachment_axes import draw_axes

def main():
    p=argparse.ArgumentParser();p.add_argument('scene',type=Path);p.add_argument('--simulate',action='store_true');p.add_argument('--axes',action='store_true');p.add_argument('--allowed-region',action='store_true');p.add_argument('--truss-only',action='store_true');args=p.parse_args()
    if args.allowed_region or args.truss_only:args.axes=True
    m=mj.MjModel.from_xml_path(str(args.scene/'scene.xml'));d=mj.MjData(m);mj.mj_forward(m,d)
    if args.truss_only:
        for i in range(m.ngeom):
            if m.body(m.geom_bodyid[i]).name.startswith('STEM_'):m.geom_rgba[i,3]=0
    d.qfrc_applied[:]=d.qfrc_bias.copy()
    meta=json.loads((args.scene/'scene.json').read_text());centers=np.array([x['center_world_m'] for x in meta['fruits']])
    print('[미리보기] 마우스로 회전/이동/확대 가능. 물리 실행:',args.simulate,flush=True)
    with mj.viewer.launch_passive(m,d) as viewer:
        viewer.cam.lookat[:]=centers.mean(0);viewer.cam.distance=.8;viewer.cam.azimuth=125;viewer.cam.elevation=-12
        if args.axes:
            viewer.cam.lookat[:]=meta['attachment_world_m'];viewer.cam.distance=.5
            viewer.opt.label=mj.mjtLabel.mjLABEL_SELECTION
        if meta.get('attachment_rule')=='all_fruit_bounds_in_side_sector_preview':
            viewer.cam.lookat[:]=(np.asarray(meta['attachment_world_m'])+centers.mean(0))/2
            viewer.cam.distance=.65;viewer.cam.azimuth=0;viewer.cam.elevation=0
        if args.truss_only:
            viewer.cam.lookat[:]=(np.asarray(meta['attachment_world_m'])+centers.mean(0))/2
            viewer.cam.distance=.7;viewer.cam.azimuth=35;viewer.cam.elevation=-15
        while viewer.is_running():
            start=time.perf_counter()
            if args.simulate:
                for _ in range(8):mj.mj_step(m,d)
            if args.axes:
                with viewer.lock():
                    viewer.user_scn.ngeom=0;draw_axes(viewer.user_scn,m,d,meta,allowed_region=args.allowed_region and not args.truss_only,world_only=args.truss_only)
            viewer.sync();time.sleep(max(0,1/30-(time.perf_counter()-start)))
if __name__=='__main__':main()
