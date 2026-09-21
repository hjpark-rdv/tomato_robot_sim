"""Separate physics+RGB-D benchmark and analytic depth/occlusion check."""
import render_backend
import argparse,json,time
from pathlib import Path
import numpy as np
import mujoco as mj
from PIL import Image
from benchmark_mujoco import Engine,HOME
from suite import resample

def main():
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,default=HOME/'models/plant_original_equivalent.xml');p.add_argument('--reference',type=Path,default=HOME/'assets/reference/reference.json');p.add_argument('--suite',type=Path,default=HOME/'assets/reference/suite.json');p.add_argument('--output',type=Path,required=True);p.add_argument('--hz',type=int,default=120);p.add_argument('--seconds',type=float,default=10.);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    e=Engine(a.model,json.loads(a.reference.read_text()),a.hz);m,d=e.model,e.data
    m.vis.map.znear=.005/m.stat.extent;m.vis.map.zfar=5./m.stat.extent
    m.vis.quality.offsamples=0 # single-pixel depth, no MSAA depth resolve bias
    trial=next(t for t in json.loads(a.suite.read_text()) if t['id']=='fixture_fruit_collision');ts,poses=resample(trial,a.hz,0)
    renderer=mj.Renderer(m,height=480,width=640);began=time.perf_counter();frames=0;render_s=0.;physics_s=0.
    for k in range(round(a.seconds*a.hz)):
        e.hook_pose(poses[min(k+1,len(poses)-1)]);t=time.perf_counter();mj.mj_step(m,d);physics_s+=time.perf_counter()-t
        if k%max(1,round(a.hz/15))==0:
            mj.mj_forward(m,d);t=time.perf_counter();renderer.update_scene(d,camera='target');rgb=renderer.render().copy();renderer.enable_depth_rendering();depth=renderer.render().copy();renderer.disable_depth_rendering();render_s+=time.perf_counter()-t;frames+=1
            if frames==1:
                Image.fromarray(rgb).save(a.output/'rgb.png');np.save(a.output/'depth_m.npy',depth)
                # Same camera ray, nearest visible geometry; compare renderer
                # optical-Z with mj_ray Euclidean distance projected onto axis.
                cam=m.camera('target').id;origin=d.cam_xpos[cam];rotation=d.cam_xmat[cam].reshape(3,3);f=240/np.tan(np.deg2rad(m.cam_fovy[cam])/2);errors=[];hits=[]
                for v in range(120,361,40):
                    for u in range(160,481,40):
                        direction=rotation@np.array([(u+.5-320)/f,-(v+.5-240)/f,-1.]);direction/=np.linalg.norm(direction);gid=np.array([-1],np.int32)
                        distance=mj.mj_ray(m,d,origin,direction,None,1,-1,gid)
                        if distance>0:
                            expected=distance*np.dot(direction,-rotation[:,2]);errors.append(abs(float(depth[v,u])-expected));hits.append(m.geom(int(gid[0])).name)
                validation=dict(samples=len(errors),median_abs_depth_error_m=float(np.median(errors)) if errors else None,max_abs_depth_error_m=max(errors,default=None),nearest_visible_geom_ids=hits,depth_definition='metres along camera optical axis, occluded geometry excluded',clipping_m=[.005,5.],K=[[f,0,320],[0,f,240],[0,0,1]])
    elapsed=time.perf_counter()-began
    report=dict(engine='MuJoCo',mode='physics_plus_rgbd',graphics=render_backend.info(),hz=a.hz,rgbd_hz=15,resolution=[640,480],simulated_s=a.seconds,wall_s=elapsed,rtf=a.seconds/elapsed,physics_step_wall_s=physics_s,render_rgbd_wall_s=render_s,frames=frames,depth_validation=validation)
    (a.output/'rgbd_report.json').write_text(json.dumps(report,indent=2));renderer.close();print('[RGB-D 완료]',json.dumps(report,ensure_ascii=False))

if __name__=='__main__':main()
