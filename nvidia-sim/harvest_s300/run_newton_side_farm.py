"""Run and inspect V6 two-stroke harvesting in the existing tomato farm."""
import argparse
from pathlib import Path
import subprocess
import sys

import mujoco as mj
import numpy as np

from side_farm import prepare, read, write, initial_for_plan
from newton_inputs import sha256
from side_farm_motion import prepare as plan_motion
from side_farm_viewer import serve, validate
from run_newton_camera_patrol import segment_inputs
from newton_active_window import canonical_state


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--preview',type=Path,help='Existing checked V6 preview folder')
    p.add_argument('--prepared-assets',type=Path,help='Reuse an already prepared V6 farm')
    p.add_argument('--recording',type=Path,help='Inspect an existing completed GPU run')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=5906)
    p.add_argument('--max-grips',type=int,default=4);p.add_argument('--grip-cycles',type=int,default=2)
    p.add_argument('--end-station',type=float,default=1.2)
    p.add_argument('--headless',action='store_true');p.add_argument('--no-autoplay',action='store_true')
    a=p.parse_args()
    if a.grip_cycles!=2:p.error('This verified farm profile uses exactly two jaw cycles')
    if not 1<=a.max_grips<=100 or not 0<a.end_station<=3.85:p.error('Invalid farm motion budget')
    output=a.output.resolve();output.mkdir(parents=True,exist_ok=False)
    assets=a.prepared_assets
    if assets is None:
        preview=a.preview
        if preview is None:
            live=read('/root/farmily_runtime/newton/runs/20261113_conveyor_bin/process.json')
            preview=Path(live.get('robot_preview',live['output']))
        assets=output/'assets';prepare(preview,assets)
    if a.recording:
        trial=a.recording
    else:
        plan=plan_motion(assets,output/'motion',max_grips=a.max_grips,cycles=2,end_station=a.end_station)
        source=assets/'inputs';model=mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
        initial=dict(np.load(source/'initial_state.npz',allow_pickle=False))
        initial=initial_for_plan(model,initial,plan)
        segment_inputs(source,output/'inputs',plan,canonical_state(model,initial),17,0)
        # Keep body-pose metadata consistent with the fresh crop-facing qpos,
        # rather than retaining the source assembly's parked body poses.
        np.savez_compressed(output/'inputs/initial_state.npz',**initial)
        proof=read(output/'inputs/inputs.json')
        proof['input_files']['initial_state.npz']['sha256']=sha256(output/'inputs/initial_state.npz')
        write(output/'inputs/inputs.json',proof)
        root=Path(__file__).resolve().parents[2];trial=output/'physics'
        command=[str(root/'nvidia-sim/run_s300_newton.sh'),'gpu-realtime',str(output/'inputs'),
            '--output',str(trial),'--nconmax','2048','--njmax','8192',
            '--dense-hessian-capacity','128','--dense-hessian-overflow-capacity','512',
            '--solver-tolerance','1e-8','--released-gutter-diagnostic','--linear-algebra','warp',
            '--inertia-linear-algebra','warp','--batch','2','--recording','full']
        with (output/'physics.log').open('w')as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    if a.headless:
        validate(assets,trial,output)
    else:
        serve(assets,trial,output/'viewer',port=a.port,autoplay=not a.no_autoplay)


if __name__=='__main__':main()
