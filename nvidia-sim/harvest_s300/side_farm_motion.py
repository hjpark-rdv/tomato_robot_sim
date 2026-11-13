"""Plan V6 motions at existing farm fruit coordinates, including two jaw strokes."""
import json
import time
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from newton_inputs import sha256
from newton_patrol_motion import motion_duration
from preview_grip_workspace import GripPlanner, OPEN_JAWS, CLOSED_JAWS, joint_connection, approach_path, checked_edge, PlanningError
from preview_posture import side_posture_policy
from preview_side_search import SideTargetSearch
from newton_scene_chunks import PackedScene
from side_farm import read, write, FarmObstacles


def camera_view_evidence(tool_pose, mount, points, side):
    """Project authored fruit centres through the actual mounted optical frame."""
    optical=np.asarray(tool_pose)@np.asarray(mount['body_from_camera_optical'])
    points=np.asarray(points,float);centre=points.mean(axis=0)
    local=(points-optical[:3,3])@optical[:3,:3]
    centre_local=(centre-optical[:3,3])@optical[:3,:3]
    k=np.asarray(mount['K']);width,height=mount['resolution_wh']
    pixels=local@k.T
    pixels=pixels[:,:2]/np.where(np.abs(local[:,2:3])>1e-12,local[:,2:3],1e-12)
    centre_pixel=k@centre_local
    centre_pixel=centre_pixel[:2]/max(1e-12,float(centre_local[2]))
    low,high=mount['depth_range_m']
    inside=(local[:,2]>=max(.20,low))&(local[:,2]<=high)&np.all(pixels>=[.05*width,.05*height],axis=1)&np.all(pixels<=[.95*width,.95*height],axis=1)
    forward=optical[:3,2]
    alignment=float((-1. if side=='left'else 1.)*forward[0]/max(1e-12,np.linalg.norm(forward[:2])))
    centre_inside=centre_local[2]>=max(.20,low) and np.all(centre_pixel>=[.1*width,.1*height]) and np.all(centre_pixel<=[.9*width,.9*height])
    return dict(side=side,camera_position_world_m=optical[:3,3].tolist(),
        camera_forward_world=forward.tolist(),horizontal_side_alignment=alignment,
        target_centre_pixel=centre_pixel.tolist(),target_depth_m=float(centre_local[2]),
        projected_fruit_count=int(inside.sum()),fruit_count=len(points),
        suitable=bool(alignment>=np.cos(np.deg2rad(30.)) and centre_inside and inside.mean()>=.8),
        scope='Optical direction and calibrated frustum projection; no occlusion or detector claim')


def camera_ready_start(planner, mount, rows, prior):
    """Choose a checked survey pose facing the first crop side before moving."""
    from itertools import product
    first=rows[0];side='left'if first['fruit_centroid_world_m'][0]<0 else 'right'
    nearby=[r for r in rows if r['plant_id']==first['plant_id']][:2]
    points=np.array([p for row in nearby for p in row['fruits'].values()])
    centre=points.mean(axis=0);optical=np.asarray(mount['body_from_camera_optical'])
    branch=int(np.sign(planner.initial[4]))
    hints=sorted([r for r in prior if r.get('reachable') and r['side']==side and int(np.sign(r['positions'][3]))==branch],
        key=lambda r:np.linalg.norm(np.asarray(r['target_capture_m'])-centre))
    seeds=[planner.initial[2:8].copy()]+[np.array(r['positions'][1:7])for r in hints[:2]]
    attempts=0
    for distance,along,height in product((.30,.40,.50),(.15,.30,.45),(.12,.02,.22)):
        camera=centre+[(distance if side=='left'else -distance),-along,height]
        forward=centre-camera;forward/=np.linalg.norm(forward)
        right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
        camera_rotation=np.column_stack((right,np.cross(forward,right),forward))
        rotation=camera_rotation@optical[:3,:3].T
        goal=dict(position=camera-rotation@optical[:3,3],quaternion_xyzw=Rotation.from_matrix(rotation).as_quat())
        for lift in (.75,.60,.45):
            for seed in seeds:
                attempts+=1
                try:state=planner.ik_at_lift(goal,lift,seed,branch)
                except PlanningError:continue
                q=np.r_[planner.initial[0],state,OPEN_JAWS]
                if not planner.valid_full(q):continue
                pose=np.eye(4);pose[:3,:3]=planner.data.xmat[planner.tool_id].reshape(3,3);pose[:3,3]=planner.data.xpos[planner.tool_id]
                evidence=camera_view_evidence(pose,mount,points,side)
                if not evidence['suitable']:continue
                evidence.update(ik_attempts=attempts,target_points_world_m=points.tolist(),
                    first_plant_id=first['plant_id'],initial_command=q.tolist())
                return q,evidence
    raise PlanningError('No collision/posture-valid crop-facing camera start within the finite survey search')


def jaw_cycle_points(grasp, *, cycles=2):
    """Stay fully inserted through both close/hold/open cycles."""
    if type(cycles) is not int or not 1 <= cycles <= 4: raise ValueError('One to four jaw cycles required')
    opened=np.array(grasp,float).copy(); opened[8:10]=OPEN_JAWS
    closed=opened.copy(); closed[8:10]=CLOSED_JAWS
    rows=[]
    for i in range(cycles):
        rows.extend([(f'grip_close_{i+1}',np.array([opened,closed]),.55),
                     ('geometric_capture_hold' if i==cycles-1 else f'grip_hold_{i+1}',np.array([closed,closed]),.70),
                     (f'grip_open_{i+1}',np.array([closed,opened]),.55),
                     (f'grip_open_hold_{i+1}',np.array([opened,opened]),.25)])
    return rows


def prepare(assets, output, *, max_grips=6, cycles=2, end_station=1.2):
    assets,output=Path(assets),Path(output);output.mkdir(parents=True,exist_ok=False)
    config=read(assets/'patrol_assets.json'); installation=config['side_installation']
    preview=Path(installation['preview']); meta=installation['preview_metadata']
    scene=PackedScene(preview/'preview_scene.npz'); source=read(meta['grip_motion_file'])
    model=mj.MjModel.from_binary_path(str(assets/'inputs/input_model.mjb'))
    initial=dict(np.load(assets/'inputs/initial_state.npz',allow_pickle=False))
    obstacles=FarmObstacles(model,installation['obstacle_boxes'])
    planner=GripPlanner(model,initial,obstacles,posture_guard=True,
        posture_policy=side_posture_policy(scene,meta['gripper_camera_mount']),model_sha256=sha256(assets/'inputs/input_model.mjb'))
    available={r['plant_id'] for r in config['plants']}
    rows=[r for r in read(assets/'inventory.json')['trusses'] if r['selected_by_side_policy'] and r['plant_id'] in available
          and .66<r['fruit_centroid_world_m'][2]<1.22 and r['fruit_centroid_world_m'][1]<end_station+.35]
    rows.sort(key=lambda r:r['fruit_centroid_world_m'][1])
    if not rows:raise PlanningError('No authored farm crop in the requested section')
    prior=read(Path(meta['grip_motion_file']).parent/'grip_coverage.json')['grid']
    current,start_view=camera_ready_start(planner,meta['gripper_camera_mount'],rows,prior)
    initial['qpos'][:]=current;planner.initial=current.copy();planner.fixed=current.copy()
    home=current.copy()
    write(output/'camera_start.json',start_view)
    print('CROP-FACING START',json.dumps(start_view),flush=True)
    if not planner.valid_full(current): raise ValueError(f'Farm start rejected: {planner.stats}')
    phases=[];checks=0;failures=[];targets=[]
    def add(name,points,minimum=.3,target=None):
        nonlocal current,checks
        points=np.asarray(points,float)
        np.testing.assert_allclose(points[0],current,atol=1e-9,rtol=0)
        # Native full-state audit includes rail motion and moving box parents.
        for a,b in zip(points[:-1],points[1:]):
            if not planner.edge(a,b,full=True):raise PlanningError(f'{name}: invalid path: {planner.stats}')
        duration=motion_duration(points,minimum=minimum)
        phases.append(dict(name=name,duration_s=duration,waypoint_q=points[:,1:8].tolist(),
            rail_waypoint_positions_m=points[:,0].tolist(),end_gap_m=float(.044+points[-1,9]-points[-1,8]),
            belt_local_y_m_s=[0.,0.],target=target))
        current=points[-1].copy();checks=planner.stats.get('state_checks',0)
    add('prepare_open',[current,home])
    add('observe_ready',[current,current],minimum=1.)
    started=time.monotonic()
    for row in rows:
        if len(targets)>=max_grips:break
        centre=np.mean(list(row['fruits'].values())[:5],axis=0)
        station=max(float(current[0]),float(np.clip(centre[1]-.25,0.,end_station)))
        print('Farm candidate',row['plant_id'],row['truss_index'],centre.round(3).tolist(),'station',round(station,3),flush=True)
        if station>current[0]+1e-6:
            moved=current.copy(); moved[0]=station;add('rail_transfer',[current,moved])
        planner.initial=current.copy();planner.fixed=current.copy()
        hints=[]
        for hint in prior:
            hint=dict(hint);hint['target_capture_m']=(np.array(hint['target_capture_m'])+[0.,station,0.]).tolist();hints.append(hint)
        search=SideTargetSearch(planner,np.array(source['nominal_tool_rotation_world']),hints)
        side='left' if centre[0]<0 else 'right'
        region=np.array([centre-[.01,.01,.01],centre+[.01,.01,.01]])
        state,rotation,evidence=search.solve(centre,side,len(targets),region)
        if state is None:
            failures.append(dict(plant=row['plant_id'],truss=row['truss_index'],centre=centre.tolist(),**evidence));continue
        try:
            approach=approach_path(planner,state,rotation,centre,distance=.06)
            transfer,route=joint_connection(planner,current[1:10],approach[0])
        except PlanningError as e:
            failures.append(dict(plant=row['plant_id'],truss=row['truss_index'],error=str(e)));continue
        target=dict(plant=row['plant_id'],truss=row['truss_index'],centre=centre.tolist(),
            station_m=station,side=side,ik=evidence,transfer_planner=route)
        def full(points):return np.column_stack((np.full(len(points),station),points))
        add('transfer',full(transfer),target=target)
        add('approach',full(approach),minimum=1.,target=target)
        add('inserted_hold',[current,current],minimum=.45,target=target)
        for name,points,duration in jaw_cycle_points(current,cycles=cycles):add(name,points,minimum=duration,target=target)
        add('withdraw',full(approach[::-1]),minimum=.85,target=target)
        add('return',full(transfer[::-1]),target=target)
        targets.append(target);write(output/'progress.json',dict(targets=targets,failures=failures))
        print('FARM GRIP PASS',len(targets),'two jaw cycles',flush=True)
    if not targets: raise ValueError('No actual-farm target found: '+str(failures))
    plan=dict(schema='s300-v6-farm-grip-plan-v1',initial_command=initial['qpos'].tolist(),
        initial_gap_m=float(.044+initial['qpos'][9]-initial['qpos'][8]),settle_seconds=.1,
        total_duration_s=.1+sum(r['duration_s']for r in phases),phases=phases,
        harvest_mode='main_stem_only_geometric_harvest',physical_detachment_enabled=False,
        training_eligible=False,harvest_success=None,grip_cycles=cycles,
        collision_certified=True,target_selection='Existing farm fruit GT coordinates; no detector claim',
        targets=targets,failures=failures,start_view=start_view,planning_s=time.monotonic()-started,
        native_state_checks=checks,posture_policy=planner.posture_guard.policy,
        posture_audit=planner.posture_guard.audit,model_sha256=sha256(assets/'inputs/input_model.mjb'))
    write(output/'plan.json',plan)
    print(json.dumps(dict(targets=len(targets),grip_cycles=cycles,duration_s=plan['total_duration_s'],checks=checks),indent=2),flush=True)
    return plan


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('assets',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--max-grips',type=int,default=6);p.add_argument('--cycles',type=int,default=2);p.add_argument('--end-station',type=float,default=1.2)
    a=p.parse_args();prepare(a.assets,a.output,max_grips=a.max_grips,cycles=a.cycles,end_station=a.end_station)
