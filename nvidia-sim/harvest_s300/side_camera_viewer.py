"""Live V6 measured-state viewer and the current real GPU camera exposure."""
from pathlib import Path
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser

from side_farm import write
from side_farm_viewer import distant_visual_plants
from newton_viser import populate_scene
from newton_geometric_harvest import volume_bounds
from preview_robot_mount import box_edges


class SideCameraViewer:
    def __init__(self,scene,output,*,port=5906,end_station=1.2):
        self.scene=scene;self.output=Path(output);self.output.mkdir(exist_ok=True)
        self.server=viser.ViserServer(host='0.0.0.0',port=port,label='Farmily V6 · 카메라 수확',verbose=False)
        if self.server.get_port()!=port:self.server.stop();raise RuntimeError('Requested live viewer port is busy')
        self.server.scene.set_up_direction('+z')
        self.status=self.server.gui.add_markdown('**V6 농장 · RGB-D 카메라 준비 중**')
        self.run_event=threading.Event();self.stop_event=threading.Event();self.close_event=threading.Event()
        self.start=self.server.gui.add_button('카메라 수확 시작',disabled=True)
        self.start.on_click(lambda _:self.run_event.set())
        self.pause=self.server.gui.add_button('일시정지');self.pause.on_click(lambda _:self.run_event.clear())
        self.resume=self.server.gui.add_button('계속');self.resume.on_click(lambda _:self.run_event.set())
        self.stop=self.server.gui.add_button('순회 중지');self.stop.on_click(lambda _:self._stop())
        self.close=self.server.gui.add_button('화면 닫기');self.close.on_click(lambda _:self._stop(close=True))
        self.picture=self.server.gui.add_image(np.zeros((320,240,3),np.uint8),label='장착 카메라 · RGB / 보이는 토마토',format='jpeg')
        self.depth=self.server.gui.add_image(np.zeros((320,240,3),np.uint8),label='장착 카메라 · 깊이',format='jpeg');self.depth.visible=False
        depth_toggle=self.server.gui.add_checkbox('깊이 영상 표시',initial_value=False)
        depth_toggle.on_update(lambda _:setattr(self.depth,'visible',depth_toggle.value))
        self.farm_root=self.server.scene.add_frame('/farm',show_axes=False)
        self.staged_farms={};self.harvested=set();self.last_q=None;self.last_poses=None
        self.last_time=0.;self.last_error=0.;self.phase='initial';self.observation_count=0;self.harvests=0;self.views={}
        visual=scene.visual
        distant=distant_visual_plants(visual,scene.native,scene.catalog,[-.4,end_station+.65])
        rgba=visual.geom_rgba[distant].copy();visual.geom_rgba[distant,3]=0.
        try:self.handles=populate_scene(self.server,visual,shadows=False)
        finally:visual.geom_rgba[distant]=rgba
        low,high=volume_bounds(scene.config.get('geometric_harvest'))
        self.volume=self.server.scene.add_line_segments('/capture_volume',box_edges(np.array([low,high])),
            (255,200,40),thickness=1.5,thickness_units='screen')
        self.tool=visual.body('Tool_fixed').id;self.dynamic={i for i in self.handles if visual.body(i).name.startswith(('Robot_','Tool_','Preview_'))}
        @self.server.on_client_connect
        def connect(client):
            view=client.gui.add_dropdown('시점',options=('작업 구간','그리퍼 확대','장착 카메라'),initial_value='작업 구간')
            def camera_view(_=None):self._camera(client,view.value)
            self.views[client.client_id]=(client,view)
            view.on_update(camera_view);camera_view()
        @self.server.on_client_disconnect
        def disconnect(client):self.views.pop(client.client_id,None)

    def _stop(self,close=False):
        self.stop_event.set();self.run_event.set()
        if close:self.close_event.set()

    def _camera(self,client,mode):
        station=0. if self.last_q is None else self.last_q[0]
        client.camera.up_direction=(0.,0.,1.)
        if mode=='장착 카메라'and self.last_poses is not None:
            optical=self.last_poses[self.tool]@np.array(self.scene.config['mounted_rgbd']['body_from_camera_optical'])
            client.camera.position=optical[:3,3];client.camera.look_at=optical[:3,3]+optical[:3,2]
            client.camera.up_direction=-optical[:3,1]
            mount=self.scene.config['mounted_rgbd'];client.camera.fov=2*np.arctan(mount['resolution_wh'][1]/(2*mount['K'][1][1]))
        elif mode=='그리퍼 확대'and self.last_poses is not None:
            target=self.last_poses[self.tool,:3,3]
            client.camera.position=target+[.65,-.45,.3];client.camera.look_at=target;client.camera.fov=np.deg2rad(52.)
        else:
            client.camera.position=[.45,station-.7,1.5];client.camera.look_at=[-.15,station+.4,.85];client.camera.fov=np.deg2rad(65.)

    def _runtime(self,error=None):
        if self.last_q is None:return
        optical=self.last_poses[self.tool]@np.array(self.scene.config['mounted_rgbd']['body_from_camera_optical'])
        value=dict(mode='v6_live_camera_patrol',recorded_replay=False,physics_clock_s=self.last_time,
            running=self.run_event.is_set()and not self.stop_event.is_set(),phase=self.phase,
            station_m=float(self.last_q[0]),lift_height_m=float(self.last_q[1]),qpos=self.last_q.tolist(),
            camera_position_world_m=optical[:3,3].tolist(),camera_forward_world=optical[:3,2].tolist(),
            observation_count=self.observation_count,harvest_motions=self.harvests,geometric_harvest_count=len(self.harvested),
            visual_native_max_position_error_m=self.last_error if error is None else error)
        tmp=self.output/'workspace_runtime.next.json';write(tmp,value);tmp.replace(self.output/'workspace_runtime.json')

    def gate(self):
        while not self.run_event.wait(.1):
            self._runtime()
            if self.stop_event.is_set():return False
        return not self.stop_event.is_set()

    def update(self,q,timestamp,phase=None):
        q=np.asarray(q);first=self.last_q is None;poses,error=self.scene.transforms(q)
        with self.server.atomic():
            for bid in self.handles if first else self.dynamic:
                handle=self.handles[bid];handle.position=poses[bid,:3,3]
                handle.wxyz=Rotation.from_matrix(poses[bid,:3,:3]).as_quat()[[3,0,1,2]]
            tool=poses[self.tool];self.volume.position=tool[:3,3]
            self.volume.wxyz=Rotation.from_matrix(tool[:3,:3]).as_quat()[[3,0,1,2]]
        self.last_q=q.copy();self.last_poses=poses;self.last_time=float(timestamp);self.last_error=error
        if phase is not None:self.phase=phase
        for client,view in list(self.views.values()):
            if view.value=='장착 카메라':self._camera(client,view.value)
        self._runtime(error)

    def mark_harvested(self,names):
        self.harvested.update(names)
        for bid,handle in self.handles.items():
            if self.scene.visual.body(bid).name in self.harvested:handle.visible=False
        self._runtime()

    def observation(self,images,station,harvests,assumed_count=0,coverage=None):
        self.picture.image=images['recognition'];self.depth.image=images['depth_overlay']
        self.observation_count+=1;self.harvests=harvests;self.phase='camera_observation'
        self.status.content=(f'**RGB-D 촬영 → 보이는 익은 토마토 선택**\n\n'
            f'입구 {station:.2f}m · 관측 {self.observation_count}회 · 수확 동작 {harvests}회\n\n'
            f'영역 수확 {assumed_count}개 · 매 송이 잡기/놓기 2회\n\n'
            '기존 SIM-GT 영상 라벨과 깊이로 선택 · Newton GPU 온라인 실행')
        if coverage:
            self.status.content+=(f'\n\n보이는 익은 열매 {coverage["visible_ripe_fruits"]}개 · '
                f'수확 대기 {coverage["visible_pending_trusses"]}송이 · 접근 보류 {coverage["visible_deferred_trusses"]}송이')
        self._runtime()

    def stage_farm(self,index,model):
        root=f'/prefetched_farm_{index}';handle=self.server.scene.add_frame(root,show_axes=False,visible=False)
        populate_scene(self.server,model,include_dynamic=False,static_root=root,shadows=False)
        self.staged_farms[index]=handle

    def activate_farm(self,index):
        handle=self.staged_farms.pop(index);old=self.farm_root
        with self.server.atomic():handle.visible=True;old.visible=False
        self.farm_root=handle;return old

    def ready(self):
        self.start.disabled=False
        self.status.content+='\n\n**촬영 및 수확 준비 완료 · 카메라 수확 시작을 누르세요.**'
        self._runtime();write(self.output/'ready.json',dict(port=self.server.get_port(),actual_gpu_camera=True,
            actual_gpu_physics_ready=True,recorded_replay=False,waits_for_start=True))

    def finish(self,report):
        self.run_event.clear();self.start.disabled=True;self.pause.disabled=True;self.resume.disabled=True
        self.phase=report['status'];self._runtime()
        self.status.content=(f'**순회 종료 · {report["status"]}**\n\n'
            f'수확 동작 {report.get("completed_harvest_motions",0)}회 · 영역 수확 {report.get("assumed_harvest_count",0)}개\n\n'
            f'{report.get("reason",report.get("error",""))}')
        while not self.close_event.wait(.2):pass
        self.server.stop()
