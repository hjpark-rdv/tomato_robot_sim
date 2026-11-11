"""One read-only browser scene across measured patrol segments, with camera PIP."""
import json
from pathlib import Path
import threading
import time
import numpy as np
import viser

from newton_viser import populate_scene
from motion_live import MotionLiveReader
from newton_inputs import sha256


class PatrolViewer:
    def __init__(self,model,port=5905,geometric=False,entry_yaw_limit_deg=15.,elbow_mode=None,harvest_volume=None):
        self.entry_yaw_limit_deg=entry_yaw_limit_deg
        self.model=model;self.server=viser.ViserServer(host='0.0.0.0',port=port,label='Farmily camera patrol',verbose=False)
        if self.server.get_port()!=port:self.server.stop();raise RuntimeError('Patrol viewer port is busy')
        self.server.scene.set_up_direction('+z')
        self.farm_root=self.server.scene.add_frame('/farm',show_axes=False)
        self.staged_farms={}
        self.handles=populate_scene(self.server,model)
        self.capture_box=None
        if geometric:
            from itertools import product,combinations
            from newton_geometric_harvest import volume_bounds
            low,high=volume_bounds(harvest_volume)
            vertices=np.array(list(product(*zip(low,high))))
            edges=np.array([[a,b] for a,b in combinations(vertices,2) if np.count_nonzero(a!=b)==1])
            self.capture_box=self.server.scene.add_line_segments('/harvest_volume',edges,(255,200,40),
                thickness=1.5,thickness_units='screen')
        self.status=self.server.gui.add_markdown('**입구에서 준비 중**\n\n이동 중 약 15cm 간격으로 카메라를 확인하고, 수확 위치에서 감속합니다.')
        self.elbow_mode=None
        self.mode_labels={'up':'엘보우 위','down':'엘보우 아래'}
        if elbow_mode is not None:
            self.elbow_mode=self.server.gui.add_dropdown('시작 자세',options=tuple(self.mode_labels.values()),
                initial_value=self.mode_labels[elbow_mode])
        self.start=self.server.gui.add_button('순회 시작')
        self.event=threading.Event()
        def start(_):
            if self.elbow_mode is not None:self.elbow_mode.disabled=True
            self.start.disabled=True;self.event.set()
        self.start.on_click(start)
        self.stop_event=threading.Event()
        self.stop=self.server.gui.add_button('순회 중지 / 화면 닫기')
        self.stop.on_click(lambda _:self.stop_event.set())
        self.picture=self.server.gui.add_image(np.zeros((640,480,3),np.uint8),label='장착 카메라 · 현재 관측',format='jpeg')
        if elbow_mode is not None:self.picture.visible=False
        self.reader=None;self.trial=None;self.sequence=-1
        self.last_poses={};self.frame_timing=[]
        self.harvested=set()
        @self.server.on_client_connect
        def connect(client):
            client.camera.position=(1.5,-1.7,1.8);client.camera.look_at=(-.25,.35,.8);client.camera.up_direction=(0.,0.,1.)
        print(f'Patrol viewer: http://localhost:{port} (순회 시작 버튼)',flush=True)

    def selected_elbow_mode(self):
        if self.elbow_mode is None:return None
        return next(k for k,v in self.mode_labels.items() if v==self.elbow_mode.value)

    def replace_robot_model(self,model):
        for handle in self.handles.values():handle.remove()
        self.model=model
        self.last_poses={}
        self.handles=populate_scene(self.server,model,include_static=False)

    def stage_farm(self,index,model):
        """Called on the preload thread; never build meshes on a handoff."""
        root=f'/prefetched_farm_{index}'
        handle=self.server.scene.add_frame(root,show_axes=False,visible=False)
        populate_scene(self.server,model,include_dynamic=False,static_root=root)
        self.staged_farms[index]=handle

    def activate_farm(self,index):
        handle=self.staged_farms.pop(index)
        old=self.farm_root
        with self.server.atomic():
            handle.visible=True;old.visible=False
        self.farm_root=handle
        # Removing a subtree also traverses server registries. Let the preload
        # executor do that cleanup after this visibility-only handoff.
        return old

    def wait_start(self,preview=None):
        shown=None
        while True:
            if self.stop_event.is_set():raise KeyboardInterrupt('Patrol cancelled before physics')
            selected=self.selected_elbow_mode()
            if preview is not None and shown!=selected:
                preview(selected);shown=selected
            if self.event.is_set():break
            self.event.wait(.2)
        # Require an actual rendered browser frame before advancing physics.
        for client in self.server.get_clients().values():
            client.get_render(height=360,width=640)
            break
        self.start.disabled=True
        return self.selected_elbow_mode()

    def update(self,poses):
        with self.server.atomic():
            if self.capture_box is not None:
                pose=poses['Tool_fixed'];self.capture_box.position=pose[:3];self.capture_box.wxyz=pose[[6,3,4,5]]
            for bid,handle in self.handles.items():
                handle.visible=self.model.body(bid).name not in self.harvested
                pose=poses[self.model.body(bid).name]
                if bid in self.last_poses and np.array_equal(pose,self.last_poses[bid]):continue
                handle.position=pose[:3];handle.wxyz=pose[[6,3,4,5]]
                self.last_poses[bid]=np.asarray(pose).copy()

    def mark_harvested(self,names):
        self.harvested.update(names)
        for bid,handle in self.handles.items():
            handle.visible=self.model.body(bid).name not in self.harvested

    def poll(self,trial):
        trial=Path(trial)
        if not (trial/'newton_live.json').exists():return
        if trial!=self.trial:
            reader=MotionLiveReader(trial);reader.verify_source()
            live=json.loads((trial/'newton_live.json').read_text())
            summary=json.loads((trial/'summary.json').read_text())
            inputs=Path(summary['input'])
            if reader.metadata['physical_model_sha256']!=sha256(inputs/'input_model.mjb'):
                raise ValueError('Patrol display physical model identity mismatch')
            self.names={path:row['name'] for path,row in live['mapping']['usd_to_native'].items()}
            self.reader=reader;self.trial=trial;self.sequence=-1
        snap=self.reader.read()
        if snap is not None and snap['sequence']!=self.sequence:
            self.update({self.names[path]:pose for path,pose in zip(self.reader.body_paths,snap['poses'])})
            self.sequence=snap['sequence']
            self.frame_timing.append(dict(wall_s=time.perf_counter(),physics_clock_s=float(snap['time']),
                                         segment=str(trial)))
            self.status.content=f"**Newton GPU · {snap['status']}**\n\n물리 시간 {snap['time']:.1f}s · 실제 측정 자세"

    def observation(self,images,station,harvests,assumed_count=0):
        self.picture.visible=True
        self.picture.image=images['recognition']
        credit=f' · 영역 수확 {assumed_count}개' if self.capture_box is not None else ''
        self.status.content=f'**촬영 → 판단**\n\n입구 기준 {station:.2f}m · 수확 동작 {harvests}회{credit}'
        self.status.content+=f'\n\n진입 각도: 로봇 정면 기준 ±{self.entry_yaw_limit_deg:g}°'

    def finish(self,report):
        failed=report['status'] in ('error','physics_stopped','entry_angle_stopped')
        heading='오류로 중단' if failed else '순회 종료'
        self.status.content=f"**{heading}: {report['status']}**\n\n수확 시도 {report.get('harvest_attempts',0)}회 · 영역 기준 수확 {report.get('assumed_harvest_count',0)}개"
        if report.get('last_station_m') is not None:
            self.status.content+=f"\n\n마지막 위치: 입구에서 {report['last_station_m']:.2f}m"
        detail=report.get('error') or report.get('reason') or report.get('stop')
        if detail:self.status.content+='\n\n**종료 원인**\n\n'+str(detail)
        self.start.label='중단' if failed else '완료';self.start.disabled=True
        print('Final scene retained. Click 순회 중지 / 화면 닫기 to exit.',flush=True)
        while not self.stop_event.wait(.2):pass
        self.server.stop()
