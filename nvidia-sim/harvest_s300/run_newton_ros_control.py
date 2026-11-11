"""Live ROS authority over the existing Newton GPU actuator/controller pipeline."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import contextlib
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import mujoco as mj
import warp as wp

from run_newton_camera_patrol import (ROOT,read,write,preflight_patrol,select_start_assets,segment_inputs,
    scan_plan,poses_from_bank,canonical_state,route_end)
from newton_inputs import sha256
from newton_ros2 import PatrolROS2
from newton_ros_commands import Commands
from newton_mounted_camera import MountedCamera
from newton_kernels import pack_runtime_status


def prepare(common,out,model,initial):
    """Use the existing verified input adapter, then retain a two-step control buffer."""
    segment_inputs(common,out,scan_plan(initial['qpos'][initial['robot_qpos_addresses']]),
                   canonical_state(model,initial),17,0)
    proof=read(out/'inputs.json');old=dict(np.load(out/'commands.npz'));original_steps=len(old['command'])
    for key,value in old.items():
        if key in ('time_edges','time_edges_master_ticks'):old[key]=value[:3]
        elif value.ndim and len(value)==original_steps:old[key]=value[:2]
    # All online control uses the contact-resolution220Hz, without changing
    # model properties, the actuator callback or the original1mm guard.
    old['command']=np.tile(initial['qpos'][initial['robot_qpos_addresses']],(2,1))
    old['time_edges']=np.arange(3,dtype=float)/220
    old['time']=old['time_edges'][1:];old['dt']=np.full(2,1/220.)
    old['phase']=np.array(['ros_control']*2)
    old['belt_surface_velocity_local_y_m_s']=np.zeros((2,2))
    old.pop('time_edges_master_ticks',None)
    np.savez_compressed(out/'commands.npz',**old)
    proof['timeline'].update(hz=220.,free_hz=220.,online_control=True,buffer_steps=2)
    write(out/'timeline.json',proof['timeline'])
    for name in ('commands.npz','timeline.json'):
        proof['input_files'][name]=dict(path=str(out/name),sha256=sha256(out/name))
    write(out/'inputs.json',proof)


class OnlineSession:
    def __init__(self,args,model,initial,camera,camera_body,ros,viewer,out,end):
        self.args=args;self.model=model;self.initial=initial;self.camera=camera;self.camera_body=camera_body
        self.ros=ros;self.viewer=viewer;self.out=out;self.end=end

    def run(self,engine,ctrl,graph,dt,batch,initial,report,nonfinite,packed,health):
        d=engine.solver.mjw_data;qadr=initial['robot_qpos_addresses'];vadr=initial['robot_dof_addresses']
        assert batch==2 and self.model.nv==10
        q=d.qpos.numpy()[0];v=d.qvel.numpy()[0];stamp=float(d.time.numpy()[0])
        bounds=np.array([self.model.jnt_range[int(self.model.dof_jntid[j])] for j in vadr]);bounds[0]=[0.,self.end]
        event_file=(self.out/'commands.jsonl').open('w');state_file=(self.out/'states.jsonl').open('w')
        trace=(self.out/'control_trace.f64').open('wb') if self.args.record_control_trace else None
        columns=1+batch*len(qadr)+len(qadr)*2
        if trace:write(self.out/'control_trace.json',dict(dtype='<f8',columns=columns,batch=batch,dt=dt,
            fields=['time','command_2x10','qpos_10','qvel_10'],joint_names=self.ros.joint_names))
        def event(row):event_file.write(json.dumps(row,allow_nan=False)+'\n');event_file.flush()
        commands=Commands(self.ros.joint_names,q[qadr],bounds,watchdog=self.args.cmd_vel_timeout,event_sink=event)
        self.ros.state(q,v,stamp,force=True);self.ros.feedback(commands.feedback())
        interrupted=[False];previous=signal.signal(signal.SIGINT,lambda *_:interrupted.__setitem__(0,True))
        started=time.perf_counter();pace=started;next_state=next_camera=next_display=0.;last_events=-1
        pending=None;pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='ros-camera');exposure_state=None
        total=0;images=0;stop=None;disconnect=None;max_pen=0.;max_contacts=0;max_candidates=0
        command_digest=hashlib.sha256();realtime_started=time.perf_counter()
        report.update(control_mode='ros',requested_steps=None,buffer_steps=batch,read_only=False,
            gui_requested=self.viewer is not None,gui_backend='viser' if self.viewer else None,
            gui_process_independent=False,gui_advances_physics=False,
            continuous_gpu_state=True,automatic_harvest_disabled=True,recording=f'bounded buffers +{self.args.ros_state_hz:g}Hz joint audit',
            command_topics=['arm_controller/joint_trajectory','joint_trajectory','cmd_vel','lift/command','gripper/command','stop'])
        (self.out/'physics/executed_commands.npz').rename(self.out/'physics/initial_command_buffer.npz')
        report['initial_command_buffer_sha256']=report.pop('executed_commands_sha256')
        report['prepared_seed_commands_sha256']=report.pop('commands_sha256')
        report['command_input_mode']='Live ROS commands, no fixed prerecorded motion';report['free_hz']=None
        write(self.out/'ready.json',dict(ready=True,time=stamp,domain=self.args.ros_domain_id,namespace=self.args.ros_namespace))
        try:
            while True:
                now=time.monotonic()
                for command in self.ros.commands.drain():commands.accept(command,stamp,now)
                if (self.ros.commands.error or self.ros.sender.error) and disconnect is None:
                    disconnect=now;commands.stop('ROS connection lost')
                if interrupted[0] or (self.viewer and self.viewer.stop_event.is_set()):stop='operator_exit';break
                if self.args.seconds and time.perf_counter()-started>=self.args.seconds:stop='duration_limit';break
                targets=np.array([commands.advance(stamp+(i+1)*dt,now,dt) for i in range(batch)],np.float32)
                ctrl.tick.zero_();ctrl.commands.assign(targets)
                wp.capture_launch(graph)
                stopping=ctrl.contact_policy.first_event if ctrl.contact_policy is not None else ctrl.first_event
                wp.launch(pack_runtime_status,1,inputs=[nonfinite,ctrl.first_qacc_nonfinite,
                    ctrl.first_capacity_overflow,stopping,packed],device=engine.model.device)
                values=packed.numpy()[0]
                for adapter in health:
                    if hasattr(adapter,'check_health'):adapter.check_health()
                total+=batch
                q=d.qpos.numpy()[0];v=d.qvel.numpy()[0];stamp=float(d.time.numpy()[0])
                command_digest.update(targets.tobytes())
                if trace:trace.write(np.r_[stamp,targets.ravel(),q[qadr],v[vadr]].astype('<f8').tobytes())
                if values[:2].min()!=np.iinfo(np.int32).max:stop='nonfinite_physics'
                elif values[2]!=np.iinfo(np.int32).max:stop='solver_capacity_overflow'
                elif values[3]!=np.iinfo(np.uint64).max:stop='contact_policy_violation'
                if stop:commands.stop(stop);break
                commands.observe(stamp,q[qadr],v[vadr])
                if stamp>=next_state:
                    self.ros.state(q,v,stamp,force=True);next_state=stamp+1/self.args.ros_state_hz
                    row=dict(time=stamp,qpos=q[qadr].tolist(),qvel=v[vadr].tolist(),target=targets[-1].tolist())
                    state_file.write(json.dumps(row)+'\n');state_file.flush()
                    max_pen=max(max_pen,float(ctrl.active_max.numpy().max()))
                    max_contacts=max(max_contacts,int(ctrl.nacon_log.numpy().max()))
                    max_candidates=max(max_candidates,int(ctrl.ncollision_log.numpy().max()))
                if commands.events!=last_events:
                    self.ros.feedback(commands.feedback());last_events=commands.events
                if pending is not None and pending.done():
                    observation,frame=pending.result();self.ros.camera(observation,frame,state=exposure_state)
                    if self.viewer:self.viewer.picture.visible=True;self.viewer.picture.image=frame['recognition']
                    pending=None;images+=1
                if self.args.camera_hz and stamp>=next_camera and pending is None:
                    exposure_state=self.ros.state(q,v,stamp,force=True)
                    poses=poses_from_bank(self.model,dict(qpos=q))
                    pending=pool.submit(self.camera.capture,poses,self.camera_body,stamp,station=float(q[qadr[0]]),include_raw=True)
                    next_camera=stamp+1/self.args.camera_hz
                if self.viewer and stamp>=next_display:
                    self.viewer.update(poses_from_bank(self.model,dict(qpos=q)));next_display=stamp+1/30
                    self.viewer.status.content=(f'**ROS 제어 · {"정지 유지" if commands.stopped else "명령 수신 중"}**\n\n'
                        f'기체 {q[qadr[0]]:.3f}m · 리프트 {q[qadr[1]]:.3f}m · 물리 {stamp:.1f}s\n\n'
                        f'ROS_DOMAIN_ID={self.args.ros_domain_id} · {self.args.ros_namespace}')
                if disconnect is not None and now-disconnect>1 and np.max(np.abs(v[vadr]))<.02:
                    stop='ros_disconnected';break
                pace+=batch*dt;delay=pace-time.perf_counter()
                if delay>0:time.sleep(delay)
                elif delay<-.2:pace=time.perf_counter()  # Never skip physics steps to catch up.
            self.ros.state(q,v,stamp,force=True);self.ros.feedback(commands.feedback())
        finally:
            signal.signal(signal.SIGINT,previous)
            pool.shutdown(wait=True,cancel_futures=True)
            if pending is not None and pending.done() and not pending.cancelled():
                obs,frame=pending.result();self.ros.camera(obs,frame,state=exposure_state);images+=1
            event_file.close();state_file.close()
            if trace:trace.close()
        np.savez_compressed(self.out/'final_state.npz',qpos=q,qvel=v,time=np.array([stamp]),
                            ctrl=d.ctrl.numpy(),eq_active=d.eq_active.numpy())
        max_pen=max(max_pen,float(ctrl.active_max.numpy().max()))
        report.update(status='completed' if stop in ('operator_exit','duration_limit') else 'physics_stopped',stop=stop,
            actual_steps=total,simulated_s=stamp,wall_s=time.perf_counter()-realtime_started,
            maximum_active_penetration_m=max_pen,maximum_contacts=max_contacts,maximum_candidates=max_candidates,
            contact_statistics_scope='Penetration cumulative; contact/candidate counts sampled at state rate; overflow checked every physics step',
            actual_gpu_control=True,command_events=commands.events,camera_exposures=images,
            last_station_m=float(q[qadr[0]]),command_sha256=command_digest.hexdigest(),
            timestep_counts={str(dt):total},
            command_feedback=commands.feedback(),buffer_memory_independent_of_session_duration=True)
        self.ros.status(report)
        return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('assets',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--headless',action='store_true');p.add_argument('--port',type=int,default=5905)
    p.add_argument('--elbow-mode',choices=['auto','up','down'],default='auto');p.add_argument('--end-station',type=float)
    p.add_argument('--ros-domain-id',type=int,default=42);p.add_argument('--ros-namespace',default='/farmily/sim')
    p.add_argument('--ros-state-hz',type=float,default=20.);p.add_argument('--camera-hz',type=float,default=1.)
    p.add_argument('--cmd-vel-timeout',type=float,default=.5)
    p.add_argument('--seconds',type=float,default=0.,help='Optional wall-time limit after readiness;0 runs until stopped')
    p.add_argument('--record-control-trace',action='store_true',help='Save bounded-memory raw actuator/state audit for deterministic replay checks')
    args=p.parse_args();args.scene_loading='full';args.control_mode='ros'
    if not 0<=args.ros_domain_id<=232:p.error('ROS domain must be within0..232')
    if not 0<args.ros_state_hz<=60 or not 0<=args.camera_hz<=10:p.error('State rate0..60Hz, camera0..10Hz required')
    if not .1<=args.cmd_vel_timeout<=2 or not np.isfinite(args.seconds) or args.seconds<0:p.error('Invalid timeout/duration')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    write(out/'preflight.json',preflight_patrol(args))
    viewer=None
    try:
        assets,viewer,wait=select_start_assets(args,out);config=read(assets/'patrol_assets.json');common=assets/'inputs'
        model=mj.MjModel.from_binary_path(str(common/'input_model.mjb'));initial=dict(np.load(common/'initial_state.npz'))
        if model.nv!=10 or not read(common/'source_export.json').get('active_window',{}).get('all_plants_static'):
            raise ValueError('ROS online mode currently requires the ten-axis static-plant geometric profile')
        prepare(common,out/'inputs',model,initial)
        camera=MountedCamera(assets,scale=.5)
        proof=read(common/'inputs.json');body=proof['source_body_to_native_name'][camera.mount['body_path']]
        if viewer is not None:
            viewer.start.label='ROS 제어 중';viewer.stop.label='ROS 제어 종료'
            viewer.status.content='**Newton GPU 준비 중**\n\n준비가 끝나면 ROS 명령을 받습니다.'
        with contextlib.ExitStack() as stack:
            ros=PatrolROS2(model,initial,camera.mount,body,assets,out/'ros2',rate=args.ros_state_hz,
                domain=args.ros_domain_id,namespace=args.ros_namespace,control=True);stack.callback(ros.close)
            ros.config.update(camera_rate='Periodic asynchronous actual exposures; original acquisition stamps retained',
                              camera_hz=args.camera_hz,control_mode='ros')
            write(out/'ros2/config.json',ros.config)
            session=OnlineSession(args,model,initial,camera,body,ros,viewer,out,route_end(config,args.end_station))
            invocation=[str(ROOT/'nvidia-sim/run_s300_newton.sh'),'gpu-realtime',str(out/'inputs'),'--output',str(out/'physics'),
                '--nconmax','1024','--njmax','1024','--dense-hessian-capacity','128','--dense-hessian-overflow-capacity','512',
                '--solver-tolerance','1e-8','--released-gutter-diagnostic','--linear-algebra','warp','--inertia-linear-algebra','warp',
                '--free-hz','220','--batch','2','--recording','summary']
            import os
            result=subprocess.run(invocation,env=dict(os.environ,S300_NEWTON_PRINT_ARGS='1'),capture_output=True,text=True,check=True)
            from run_newton_harvest import arguments,run
            opts=arguments(json.loads(result.stdout))
            with (out/'physics.log').open('w') as log,contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
                report=run(opts,online_session=session)
            report['ros2']=ros.close();write(out/'summary.json',report)
        if viewer and not viewer.stop_event.is_set():
            viewer.status.content=f"**ROS 제어 종료: {report['stop']}**\n\n마지막 위치 {report['last_station_m']:.3f}m"
            while not viewer.stop_event.wait(.2):pass
    except BaseException as error:
        import traceback
        write(out/'error.json',dict(error=repr(error),traceback=traceback.format_exc()));raise
    finally:
        if viewer:viewer.server.stop()


if __name__=='__main__':main()
