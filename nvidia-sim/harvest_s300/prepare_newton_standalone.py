"""Build local reusable Newton inputs from source assets, without trial history.

Run with .asset-venv. Each output is new; logs/results are separate from assets.
Uses the existing USD, plant, robot, plan and Newton preparation adapters.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def build(output, *, plant=22, target='Truss_02_Fruit_01', assembly=None, smoke=False,
          ground_collision='top-plane'):
    from build_scene import build as base_build
    from motion_assets import build as motion_build, revise_for_horizontal_ready
    from fruit_attachment import revise as attachment
    from compliant_foliage import revise as foliage
    from rail_assets import revise as rail
    from compact_motion_assets import compact_assembly
    from side_robot_mount import prepare as side
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    if assembly is None:
        base_build(SimpleNamespace(s300=ROOT/'gripper-design/robot-assembly/models/s300_lift_rb5_gripper.glb',
            output=output/'base',farm_layout='reviewed',farm_scale=.5,position=(0.,1.6,0.)))
        motion_build(output/'base/scene.usda',output/'motion',plant=plant,target=target)
        revise_for_horizontal_ready(output/'motion',output/'hardware',hardware_only=True)
        attachment(output/'hardware',output/'attachment',kind='elastic',force_n=3.,torque_nm=.008)
        foliage(output/'attachment',output/'foliage')
        rail(output/'foliage',output/'rail')
        compact_assembly(output/'rail',output/'compact')
        side(output/'compact',output/'assembly')
        assembly=output/'assembly'
    else:
        assembly=assembly.resolve(strict=True)
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    sys.path.insert(0,str(ROOT/'mujoco-benchmark/scripts/s300_smoke'))
    from plan_adapter import make_plan, DOF_NAMES
    q=np.asarray(manifest['initial_robot_q'])
    station=manifest['rail']['initial_station_y_m']
    center=np.asarray(manifest['target']['center'])
    if smoke:
        # Explicit actuator smoke, never reported as a harvest attempt.
        lifted=q.copy();lifted[0]+=min(.02,1.-q[0])
        phases=[]
        for name,begin,end,gap in [('prepare_open',q,q,.096),('lift',q,lifted,.096),
                                   ('return_ready',lifted,q,.096),('close',q,q,.028),
                                   ('cycle_open',q,q,.096)]:
            phases.append(dict(name=name,duration_s=2.,waypoint_q=[begin.tolist(),end.tolist()],
                end_gap_m=gap,belt_local_y_m_s=[0.,0.]))
        plan=dict(schema='s300-standalone-actuator-smoke-v1',initial_command=np.r_[0.,q,0.,0.].tolist(),
                  initial_gap_m=.044,settle_seconds=2.,phases=phases,total_duration_s=12.,
                  dof_names=DOF_NAMES,harvest_attempt=False,training_eligible=False)
    else:
        direction=center-np.array([0.,station,center[2]])
        plan=make_plan(assembly,dict(measured_joint_q=q.tolist(),measured_station_y_m=station,
            target_center=center.tolist(),direction=direction.tolist(),ready_lift_offset_m=0.,
            grasp_height_offset_m=-.01,grasp_depth_m=.035))
    plan_path=output/'plan.json';plan_path.write_text(json.dumps(plan,indent=2)+'\n')
    from export_scene import export
    export(assembly,output,output/'model',truss=target.rsplit('_Fruit_',1)[0],robot=True,
           background=True,ground_collision=ground_collision,source_plan=plan_path)
    python=ROOT/'nvidia-sim/.newton-venv/bin/python'
    scripts=Path(__file__).parent
    commands=[
        [python,scripts/'newton_inputs.py',output/'model',output/'inputs_full',
         '--hz','220','--discard-visual-geoms'],
        [python,scripts/'newton_plant_variant.py',output/'inputs_full',output/'inputs_coarse', '--mode','coarse4'],
        [python,scripts/'newton_fruit_proxy.py',output/'inputs_coarse',output/'inputs']]
    for command in commands:
        print('RUN', ' '.join(map(str,command)),flush=True)
        subprocess.run(list(map(str,command)),check=True)
    (output/'build.json').write_text(json.dumps(dict(assembly=str(assembly),inputs=str(output/'inputs'),
        old_trial_data_required=False,smoke_only=smoke,plant=plant,target=target,
        scene_regenerated=True,identical_to_lost_experiment=False,
        ground_collision=ground_collision,
        collision_scope='target truss/main stem, robot, gutter and ground; background plants visual only',
        approximation='existing coarse4/fruit proxy; chassis recovered from merged local visual GLB requires collision review',
        training_eligible=False,harvest_success=None),indent=2)+'\n')
    print('Prepared local Newton inputs:',output/'inputs',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path)
    p.add_argument('--assembly',type=Path,help='Optional existing reusable assembly, never recorded states')
    p.add_argument('--plant',type=int,default=22)
    p.add_argument('--target',default='Truss_02_Fruit_01')
    p.add_argument('--smoke',action='store_true',help='Lift/jaw actuator smoke instead of a harvest trajectory')
    p.add_argument('--ground-collision',choices=('source-box','top-plane'),default='top-plane',
        help='Existing adapter option: retain floor visuals, prevent released fruit tunnelling through the thin floor')
    a=p.parse_args();build(a.output,plant=a.plant,target=a.target,assembly=a.assembly,smoke=a.smoke,
                          ground_collision=a.ground_collision)
