"""Inspect measured Newton GPU V6 motions in the unchanged visual tomato farm."""
import argparse
import json
from pathlib import Path
import threading
import time

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation
import viser

from newton_scene_chunks import PackedScene
from newton_viser import populate_scene
from newton_geometric_harvest import volume_bounds
from preview_robot_mount import body_poses, box_edges
from preview_gripper_camera import populate_camera_scene, update_camera
from preview_conveyor_bin import draw_conveyor
from side_farm import read, write, FarmObstacles
from preview_grip_workspace import GripPlanner
from preview_posture import side_posture_policy
from newton_inputs import sha256
from side_farm_motion import camera_view_evidence


def distant_visual_plants(scene, model, catalog, interval):
    """Cull only display triangles wholly outside the local test section."""
    data=mj.MjData(model);mj.mj_kinematics(model,data)
    cache={};hidden=[]
    for i in range(scene.ngeom):
        obj=catalog['objects'].get(str(catalog['geom_to_instance'].get(scene.geom(i).name,0)),{})
        if not obj.get('plant') or scene.geom_dataid[i]<0:continue
        mid=int(scene.geom_dataid[i])
        if mid not in cache:
            start=int(scene.mesh_vertadr[mid]);v=scene.mesh_vert[start:start+int(scene.mesh_vertnum[mid])]
            from itertools import product
            cache[mid]=np.array(list(product(*zip(v.min(0),v.max(0)))))
        bid=model.body(scene.body(int(scene.geom_bodyid[i])).name).id
        br=data.xmat[bid].reshape(3,3)
        r=br@Rotation.from_quat(scene.geom_quat[i][[1,2,3,0]]).as_matrix()
        p=data.xpos[bid]+br@scene.geom_pos[i]
        y=(cache[mid]@r.T+p)[:,1]
        if y.max()<interval[0] or y.min()>interval[1]:hidden.append(i)
    return hidden


def validate(assets, trial, output):
    assets,trial,output=Path(assets),Path(trial),Path(output)
    summary=read(trial/'summary.json');plan=read(Path(summary['input'])/'sampled_plan.json')
    if summary['status']!='completed' or summary.get('backend')!='gpu':raise ValueError('Only a completed GPU run may be displayed')
    if (summary['source_model_mjb_sha256']!=sha256(assets/'inputs/input_model.mjb')
            or summary['states_sha256']!=sha256(trial/'states.npz')
            or plan['model_sha256']!=summary['source_model_mjb_sha256']):
        raise ValueError('Farm model, planned motion and measured recording hashes differ')
    model=mj.MjModel.from_binary_path(str(assets/'inputs/input_model.mjb'))
    initial=dict(np.load(assets/'inputs/initial_state.npz'))
    config=read(assets/'patrol_assets.json');meta=config['side_installation']['preview_metadata']
    scene=PackedScene(Path(config['side_installation']['preview'])/'preview_scene.npz')
    planner=GripPlanner(model,initial,FarmObstacles(model,config['side_installation']['obstacle_boxes']),
        posture_guard=True,posture_policy=side_posture_policy(scene,meta['gripper_camera_mount']),
        model_sha256=sha256(assets/'inputs/input_model.mjb'))
    with np.load(trial/'states.npz',allow_pickle=False)as a:states={k:a[k]for k in ('time','qpos','qvel','phase')}
    low,high=volume_bounds(config.get('geometric_harvest'))
    names=read(assets/'inputs/source_export.json')['geometric_fruit_names']
    fruit_ids=np.array([model.body(n).id for n in names])
    captures=[];collected=set();t=plan['settle_seconds'];stroke_evidence=[]
    for phase in plan['phases']:
        start=t;t+=phase['duration_s']
        if phase['name'].startswith('grip_close_'):
            ids=np.flatnonzero((states['time']>=start)&(states['time']<=t))
            if not len(ids):raise ValueError('No measured close samples')
            q=states['qpos'][ids];gap=.044+q[:,9]-q[:,8]
            if gap.min()>.040:raise ValueError('Measured jaws did not close')
            stroke_evidence.append(dict(name=phase['name'],start_s=start,end_s=t,minimum_measured_gap_m=float(gap.min())))
        if phase['name']!='geometric_capture_hold':continue
        k=max(0,int(np.searchsorted(states['time'],t))-1)
        planner.data.qpos[:]=states['qpos'][k];mj.mj_kinematics(model,planner.data)
        tool=planner.tool_id;r=planner.data.xmat[tool].reshape(3,3)
        local=(planner.data.xpos[fruit_ids]-planner.data.xpos[tool])@r
        inside=np.all((local>=low)&(local<=high),axis=1)
        new=[n for n,hit in zip(names,inside)if hit and n not in collected]
        collected.update(new);captures.append(dict(time_s=t,names=new,target=phase['target']))
    if len(stroke_evidence)!=len(plan['targets'])*2:raise ValueError('Exactly two measured closures per target required')
    start_view=None;minimum_ready_alignment=1.;ready_intervals=[];t=plan['settle_seconds']
    if plan.get('start_view'):
        np.testing.assert_allclose(states['qpos'][0],plan['initial_command'],rtol=0,atol=1e-6)
        for phase in plan['phases']:
            if phase['name']in ('prepare_open','observe_ready','rail_transfer','return'):
                ready_intervals.append((t,t+phase['duration_s']))
            t+=phase['duration_s']
    mount=meta['gripper_camera_mount'];optical=np.asarray(mount['body_from_camera_optical'])
    for i,q in enumerate(states['qpos']):
        if not planner.valid_full(q,measured=True):raise ValueError('Measured V6 farm posture rejected: '+str(planner.stats))
        if not plan.get('start_view'):continue
        rotation=planner.data.xmat[planner.tool_id].reshape(3,3)
        if i==0:
            pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=planner.data.xpos[planner.tool_id]
            start_view=camera_view_evidence(pose,mount,plan['start_view']['target_points_world_m'],plan['start_view']['side'])
            if not start_view['suitable']:raise ValueError('Measured start does not face the crop in the camera frustum')
        if any(lo<=states['time'][i]<=hi for lo,hi in ready_intervals):
            forward=rotation@optical[:3,2]
            alignment=float((-1. if plan['start_view']['side']=='left'else 1.)*forward[0]/np.linalg.norm(forward[:2]))
            minimum_ready_alignment=min(minimum_ready_alignment,alignment)
            if alignment<np.cos(np.deg2rad(35.)):raise ValueError('Ready/rail/return camera turned away from the working crop side')
    report=dict(status='passed',actual_gpu_physics=True,recorded_state_samples=len(states['time']),
        physics_summary=str(trial/'summary.json'),model_sha256=sha256(assets/'inputs/input_model.mjb'),
        states_sha256=sha256(trial/'states.npz'),targets=len(plan['targets']),grip_cycles_per_target=2,
        measured_strokes=stroke_evidence,capture_events=captures,geometric_capture_count=len(collected),
        posture_audit=planner.posture_guard.audit,physical_fruit_detachment=False,
        target_selection='Existing farm fruit coordinates; ground-truth motion test, not camera autonomous detection')
    if start_view is not None:
        report.update(measured_start_view=start_view,minimum_ready_camera_side_alignment=minimum_ready_alignment)
    write(output/'validation.json',report)
    return states,plan,report


def serve(assets,trial,output,*,port=5906,autoplay=False):
    assets,trial,output=Path(assets),Path(trial),Path(output);output.mkdir(parents=True,exist_ok=False)
    states,plan,validation=validate(assets,trial,output)
    config=read(assets/'patrol_assets.json');install=config['side_installation'];meta=install['preview_metadata']
    preview=Path(install['preview']);robot=PackedScene(preview/'preview_scene.npz')
    old_assets=Path(config['source_patrol_assets']);manifest=read(old_assets/'scene_stream/manifest.json')
    section=manifest['chunks'][0];farm=PackedScene(old_assets/'scene_stream'/section['geometry'],section['geometry_sha256'])
    source=mj.MjModel.from_binary_path(str(old_assets/'inputs/input_model.mjb'))
    initial=dict(np.load(old_assets/'inputs/initial_state.npz'))
    display_interval=[-.4,float(states['qpos'][:,0].max())+.65]
    distant=distant_visual_plants(farm,source,read(old_assets/'sensor_catalog.json'),display_interval)
    server=viser.ViserServer(host='0.0.0.0',port=port,label='Farmily V6 · 농장 수확',verbose=False)
    if server.get_port()!=port:server.stop();raise RuntimeError('Farm viewer port busy')
    server.scene.set_up_direction('+z')
    status=server.gui.add_markdown('**농장 모델 준비 중**')
    play=server.gui.add_button('수확 모션 재생');pause=server.gui.add_button('일시정지')
    reset=server.gui.add_button('처음부터 재생')
    inspect_start=server.gui.add_button('시작 자세 보기')
    speed=server.gui.add_slider('재생 배속',min=.25,max=2.,step=.25,initial_value=1.)
    shutdown=threading.Event();server.gui.add_button('화면 닫기').on_click(lambda _:shutdown.set())
    ids=sorted(set(distant)|{i for i in range(farm.ngeom)if farm.body(int(farm.geom_bodyid[i])).name.startswith(('Robot_','Tool_'))})
    rgba=farm.geom_rgba[ids].copy();farm.geom_rgba[ids,3]=0.
    try:plants=populate_scene(server,farm,shadows=False)
    finally:farm.geom_rgba[ids]=rgba
    handles,camera=populate_camera_scene(server,robot,meta['gripper_camera_mount'])
    bin_handles=draw_conveyor(server,meta['layouts']['side']['conveyor']);bin_handles['root'].visible=True
    adapter=server.scene.add_box('/v6_side_adapter',dimensions=meta['layouts']['side']['adapter_size_m'],color=(180,184,190))
    low,high=volume_bounds(config.get('geometric_harvest'))
    volume=server.scene.add_line_segments('/capture_volume',box_edges(np.array([low,high])),(255,200,40),thickness=1.5,thickness_units='screen')
    native=mj.MjModel.from_binary_path(str(assets/'inputs/input_model.mjb'));native_data=mj.MjData(native)
    native_data.qpos[:]=states['qpos'][0];mj.mj_kinematics(native,native_data)
    start_tool=np.eye(4);start_id=native.body(meta['gripper_camera_mount']['body']).id
    start_tool[:3,:3]=native_data.xmat[start_id].reshape(3,3);start_tool[:3,3]=native_data.xpos[start_id]
    start_optical=start_tool@np.asarray(meta['gripper_camera_mount']['body_from_camera_optical'])
    duration=float(states['time'][-1]);playback=dict(time=0.,running=autoplay);lock=threading.RLock()
    def start(_=None):
        with lock:
            if playback['time']>=duration:playback['time']=0.
            playback['running']=True
    def stop(_=None):
        with lock:playback['running']=False
    def restart(_=None):
        with lock:playback.update(time=0.,running=True)
    def inspect(_=None):
        with lock:playback.update(time=0.,running=False)
    play.on_click(start);pause.on_click(stop);reset.on_click(restart)
    inspect_start.on_click(inspect)
    phase_ends=np.cumsum([plan['settle_seconds']]+[p['duration_s']for p in plan['phases']])
    def frame(t):
        k=max(0,min(int(np.searchsorted(states['time'],t))-1,len(states['time'])-2))
        u=float(np.clip((t-states['time'][k])/(states['time'][k+1]-states['time'][k]),0.,1.))
        return (1-u)*states['qpos'][k]+u*states['qpos'][k+1]
    labels={'rail_transfer':'농장 이동','prepare_open':'좌측 촬영 준비','observe_ready':'좌측 작물 관측 자세','transfer':'송이 접근 준비',
        'approach':'송이 안으로 진입','inserted_hold':'진입 후 대기','withdraw':'후퇴','return':'준비 자세 복귀',
        'grip_close_1':'1차 잡기','grip_hold_1':'1차 유지','grip_open_1':'1차 열기','grip_open_hold_1':'1차 열림 유지',
        'grip_close_2':'2차 잡기','geometric_capture_hold':'2차 유지 · 영역 확인','grip_open_2':'2차 열기','grip_open_hold_2':'2차 열림 유지'}
    plants_initialized=False;previous_hidden=set()
    def render(t):
        nonlocal plants_initialized,previous_hidden
        q=frame(t);initial['qpos'][:]=q
        poses,seat=body_poses(source,initial,meta,q[1],True,q[2:8],0.,'side',jaw_positions=q[8:])
        # The replay must display the same arm that Newton actually moved.
        # In particular, a nonzero rail position must translate both the
        # mounting adapter and every arm/tool body along with the carriage.
        native_data.qpos[:]=q;mj.mj_kinematics(native,native_data)
        arm_ids=meta['arm_bodies']
        position_error=float(np.max(np.linalg.norm(poses[arm_ids,:3,3]-native_data.xpos[arm_ids],axis=1)))
        rotation_error=float(np.max(np.abs(poses[arm_ids,:3,:3]-native_data.xmat[arm_ids].reshape(-1,3,3))))
        if position_error>1e-8 or rotation_error>1e-8:
            raise ValueError(f'Displayed arm differs from recorded native model: {position_error}m / {rotation_error}')
        hidden={n for event in validation['capture_events']if event['time_s']<=t for n in event['names']}
        with server.atomic():
            if not plants_initialized:
                for bid,handle in plants.items():
                    name=farm.body(bid).name;nbid=native.body(name).id
                    handle.position=native_data.xpos[nbid];handle.wxyz=native_data.xquat[nbid]
                plants_initialized=True
            if hidden!=previous_hidden:
                for bid,handle in plants.items():
                    name=farm.body(bid).name
                    if name in hidden^previous_hidden:handle.visible=name not in hidden
                previous_hidden=hidden
            for bid,handle in handles.items():
                handle.position=poses[bid,:3,3];handle.wxyz=Rotation.from_matrix(poses[bid,:3,:3]).as_quat()[[3,0,1,2]]
            handles[meta['plate_id']].visible=False;handles[meta['motor_id']].visible=False
            update_camera(camera,poses,robot,meta['gripper_camera_mount'],'side')
            adapter.position=seat+np.array([0.,meta['layouts']['side']['adapter_size_m'][1]/2.,0.])
            bin_handles['root'].position=[0.,q[0],0.]
        tool=poses[robot.body('Tool_fixed').id];volume.position=tool[:3,3];volume.wxyz=Rotation.from_matrix(tool[:3,:3]).as_quat()[[3,0,1,2]]
        i=max(0,min(int(np.searchsorted(phase_ends,t,side='right'))-1,len(plan['phases'])-1));p=plan['phases'][i]
        status.content=(f'**V6 농장 수확 · {labels.get(p["name"],p["name"])}**\n\n'
            f'이동 {q[0]:.2f}m · 시간 {t:.1f}/{duration:.1f}s · 매 송이 그리핑 2회\n\n'
            f'토마토 영역 수확 {len(hidden)}개 · 농장 좌표 기반 시험\n\n'
            'Newton GPU 실제 관절 동작 기록 재생 · 과실 분리 힘은 검증하지 않음')
        optical_pose=tool@np.asarray(meta['gripper_camera_mount']['body_from_camera_optical'])
        write(output/'workspace_runtime.json',dict(layout='side_farm',elapsed_s=t,running=playback['running'],
            phase=p['name'],target=p.get('target'),station_m=float(q[0]),lift_height_m=float(q[1]),
            arm_positions_rad=q[2:8].tolist(),jaw_positions_m=q[8:].tolist(),harvested_count=len(hidden),
            active_camera_body='Tool_fixed',actual_gpu_recording=True,
            visual_native_max_position_error_m=position_error,visual_native_max_rotation_error=rotation_error,
            base_mount_seat_world_m=seat.tolist(),camera_position_world_m=optical_pose[:3,3].tolist(),
            camera_forward_world=optical_pose[:3,2].tolist()))
    @server.on_client_connect
    def connect(client):
        view=client.gui.add_dropdown('시점',options=('작업 구간','수확부 확대','장착부 확대','시작 카메라 방향','측면'),initial_value='작업 구간')
        def camera_view(_=None):
            q=frame(playback['time']);station=q[0]
            client.camera.up_direction=(0.,0.,1.)
            if view.value=='시작 카메라 방향':
                client.camera.position=start_optical[:3,3]
                client.camera.look_at=start_optical[:3,3]+start_optical[:3,2]
                client.camera.up_direction=-start_optical[:3,1]
                spec=meta['gripper_camera_mount']
                client.camera.fov=2.*np.arctan(spec['resolution_wh'][1]/(2.*spec['K'][1][1]))
            elif view.value=='수확부 확대':
                target=handles[robot.body('Tool_fixed').id].position
                client.camera.position=np.array(target)+[.65 if target[0]<0 else -.65,-.45,.3]
                client.camera.look_at=target;client.camera.fov=np.deg2rad(52.)
            elif view.value=='장착부 확대':
                target=np.asarray(adapter.position)
                client.camera.position=target+[.6,-.22,.18]
                client.camera.look_at=target;client.camera.fov=np.deg2rad(45.)
            else:
                # Stay inside the aisle; the greenhouse wall otherwise covers
                # the entire scene from the former outside camera position.
                client.camera.position=([.45,station-.7,1.5]if view.value=='작업 구간'else[.45,station+.25,1.15])
                client.camera.look_at=[-.15,station+.4,.85];client.camera.fov=np.deg2rad(65.)
        view.on_update(camera_view);camera_view()
    render(0.);write(output/'ready.json',dict(port=port,actual_gpu_recording=True,validation='validation.json',
        displayed_farm_y_interval_m=display_interval,culled_distant_visual_geoms=len(distant),physical_farm_unchanged=True))
    print(f'V6 farm GPU replay ready: http://localhost:{port}',flush=True)
    before=time.monotonic();last_write=0.
    while not shutdown.wait(1/30):
        now=time.monotonic();dt=min(.15,now-before);before=now
        with lock:
            if playback['running']:
                playback['time']=min(duration,playback['time']+dt*speed.value)
                if playback['time']>=duration:playback['running']=False
            if playback['running']or now-last_write>.25:render(playback['time']);last_write=now
    server.stop()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('assets',type=Path);p.add_argument('trial',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=5906);p.add_argument('--autoplay',action='store_true')
    a=p.parse_args();serve(a.assets,a.trial,a.output,port=a.port,autoplay=a.autoplay)
