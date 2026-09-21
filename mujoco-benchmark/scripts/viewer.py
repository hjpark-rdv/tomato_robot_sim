"""Single plant + hook interactive viewer (DISPLAY=:0)."""
import argparse,json,time
from pathlib import Path
import mujoco as mj
import mujoco.viewer
from benchmark_mujoco import Engine,HOME
from suite import resample

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',type=Path,default=HOME/'models/plant_original_equivalent.xml')
    p.add_argument('--trajectory',default='fixture_stem_push');p.add_argument('--hz',type=int,default=120);p.add_argument('--speed',type=float,default=1.)
    p.add_argument('--benchmark-seconds',type=float);p.add_argument('--output',type=Path)
    a=p.parse_args()
    if a.speed<=0:raise ValueError('speed must be positive')
    if a.benchmark_seconds is not None and (a.benchmark_seconds<=0 or a.output is None):raise ValueError('Finite benchmark needs positive seconds and --output')
    if a.output:a.output.mkdir(parents=True,exist_ok=False)
    ref=json.loads((HOME/'assets/reference/reference.json').read_text());e=Engine(a.model,ref,a.hz)
    print('[화면 테스트]',a.model.name,'|',a.trajectory,'|',a.hz,'Hz |',a.speed,'배속',flush=True)
    trial=next(t for t in json.loads((HOME/'assets/reference/suite.json').read_text()) if t['id']==a.trajectory);ts,poses=resample(trial,a.hz)
    with mujoco.viewer.launch_passive(e.model,e.data) as view:
        view.cam.type=mj.mjtCamera.mjCAMERA_FIXED;view.cam.fixedcamid=e.model.camera('target').id
        while view.is_running():
            e.reset();e.hook_pose(poses[0]);mj.mj_forward(e.model,e.data);view.sync()
            began=time.perf_counter();physics_s=sync_s=0.;frames=steps=0
            limit=min(len(poses)-1,round(a.benchmark_seconds*a.hz)) if a.benchmark_seconds else len(poses)-1
            for i,pose in enumerate(poses[1:limit+1]):
                if not view.is_running():return
                started=time.perf_counter();e.hook_pose(pose);t=time.perf_counter();mj.mj_step(e.model,e.data);physics_s+=time.perf_counter()-t;steps+=1
                if i%max(1,round(a.hz/30))==0:
                    t=time.perf_counter();view.sync();sync_s+=time.perf_counter()-t;frames+=1
                if a.benchmark_seconds is None:time.sleep(max(0.,1/a.hz/a.speed-(time.perf_counter()-started)))
            if a.benchmark_seconds is not None:
                wall=time.perf_counter()-began
                report=dict(engine='MuJoCo',model=str(a.model.resolve()),mode='physics_plus_passive_viewer',trajectory_id=trial['id'],hz=a.hz,simulated_s=steps/a.hz,wall_s=wall,rtf=steps/a.hz/wall,physics_wall_s=physics_s,viewer_sync_wall_s=sync_s,sync_calls=frames,
                    note='Unpaced physics, viewer.sync at 30 Hz of simulated time; GUI renders asynchronously, sync calls are not guaranteed displayed frame count. Startup excluded.')
                (a.output/'viewer_report.json').write_text(json.dumps(report,indent=2));print('[화면 표시 측정]',json.dumps(report,ensure_ascii=False));return

if __name__=='__main__':main()
