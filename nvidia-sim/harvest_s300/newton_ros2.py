"""Bounded ROS telemetry and optional command transport; DDS stays outside Newton."""
from collections import Counter
import copy
from collections import deque
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/farmily_newton_bridge'))
from farmily_newton_bridge.protocol import encode,receive,ros_instance_id


class LatestSender:
    """One latest slot per data kind; slow DDS cannot accumulate simulation work."""
    def __init__(self,sock):
        self.sock=sock;self.condition=threading.Condition();self.pending={};self.sequence=0
        self.closed=False;self.error=None;self.sent=Counter();self.dropped=Counter()
        self.thread=threading.Thread(target=self.run,daemon=True,name='ros2-transport');self.thread.start()

    def put(self,message,arrays=None):
        with self.condition:
            if self.closed or self.error:return
            kind=message['kind']
            if kind in self.pending:self.dropped[kind]+=1
            self.sequence+=1;self.pending[kind]=(self.sequence,dict(message),arrays)
            self.condition.notify()

    def run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda:self.pending or self.closed)
                    if not self.pending:return
                    key=min(self.pending,key=lambda k:self.pending[k][0])
                    _,message,arrays=self.pending.pop(key)
                self.sock.sendall(encode(message,arrays));self.sent[key]+=1
        except Exception as error:
            self.error=repr(error)
            print('ROS 2 transport disconnected: '+self.error,file=sys.stderr,flush=True)

    def close(self):
        with self.condition:self.closed=True;self.condition.notify()
        self.thread.join(timeout=2.)
        try:self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:pass
        self.sock.close();self.thread.join(timeout=1.)


class PatrolROS2:
    def __init__(self,model,initial,mount,camera_body,assets,output,*,rate=10.,domain=42,namespace='/farmily/sim',control=False):
        self.model=model;self.data=mj.MjData(model);self.mount=mount;self.camera_body=camera_body
        self.rate=rate;self.last_state=-1.;self.last_time=0.;self.closed=False
        self.qadr=np.array(initial['robot_qpos_addresses']);self.vadr=np.array(initial['robot_dof_addresses'])
        self.joint_ids=[int(model.dof_jntid[i]) for i in self.vadr]
        if any(int(model.jnt_type[i]) not in (int(mj.mjtJoint.mjJNT_SLIDE),int(mj.mjtJoint.mjJNT_HINGE)) for i in self.joint_ids):
            raise ValueError('ROS adapter requires scalar robot joints')
        self.joint_names=[model.joint(i).name for i in self.joint_ids]
        self.body_ids=[i for i in range(1,model.nbody) if model.body(i).name.startswith(('Robot_','Tool_'))]
        self.output=Path(output);self.output.mkdir()
        self.temp=tempfile.TemporaryDirectory(prefix='farmily-ros2-');path=str(Path(self.temp.name)/'feed.sock')
        server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(path);server.listen(1);server.settimeout(15.)
        self.log=(self.output/'bridge.log').open('w')
        env=dict(os.environ,S300_ROS_DOMAIN_ID=str(domain));env.pop('PYTHONPATH',None);env.pop('VIRTUAL_ENV',None)
        self.process=subprocess.Popen(['bash',str(ROOT/'nvidia-sim/run_s300_ros2.sh'),'bridge',
            '--socket',path,'--namespace',namespace,'--report',str(self.output/'publisher.json')]+(['--control'] if control else []),
            stdout=self.log,stderr=subprocess.STDOUT,env=env)
        try:
            connection,_=server.accept();connection.settimeout(5.)
            ready,_=receive(connection)
            if ready['kind']!='ready':raise RuntimeError('ROS publisher handshake failed')
        except BaseException:
            self.process.terminate();self.process.wait(timeout=5);self.log.close();self.temp.cleanup();raise
        finally:server.close()
        self.sender=LatestSender(connection)
        self.commands=CommandInbox(connection) if control else None
        catalog=json.loads((Path(assets)/'sensor_catalog.json').read_text())
        from mounted_gt import CLASSES
        fields=('key','class_name','class_id','plant','truss','source_name','red_proxy','authored_rgb')
        labels=dict(classes=CLASSES,instances={str(ros_instance_id(k)):dict(
            {f:v[f] for f in fields if f in v},source_instance_id=int(k)) for k,v in catalog['objects'].items()},
            source='authored visible SIM-GT, not a detector',instance_encoding='32SC1',
            instance_id_rule='Source uint32 bits reinterpreted as signed int32; negative ROS IDs are valid')
        self.sender.put(dict(kind='labels',labels=labels))
        self.config=dict(namespace=namespace,ros_domain_id=domain,state_periodic_hz=rate,
            extra_state_samples='Actual camera exposures and phase boundaries',
            camera_rate='Existing actual exposure events only; no additional rendering or fresh-stamped repeats',
            read_only=not control,physics_control_subscriptions=control,transport='bounded latest-slot Unix socket',
            ros_python='/usr/bin/python3',clock='/clock',camera_frame=namespace.strip('/')+'/camera_optical_frame')
        from newton_inputs import sha256
        self.config.update(assets=str(Path(assets).resolve()),model_sha256=sha256(Path(assets)/'inputs/input_model.mjb'))
        self.sender.put(dict(kind='configuration',configuration=self.config))
        (self.output/'config.json').write_text(json.dumps(self.config,indent=2)+'\n')

    def state(self,qpos,qvel,timestamp,*,force=False):
        timestamp=float(timestamp)
        if not force and timestamp-self.last_state<1/self.rate-1e-9:return
        self.last_state=timestamp;self.last_time=timestamp
        self.data.qpos[:]=qpos;mj.mj_kinematics(self.model,self.data)
        transforms=[]
        for i in self.body_ids:
            parent=int(self.model.body_parentid[i]);r=self.data.xmat[parent].reshape(3,3)
            pos=r.T@(self.data.xpos[i]-self.data.xpos[parent])
            rotation=r.T@self.data.xmat[i].reshape(3,3)
            transforms.append(dict(parent=self.model.body(parent).name,child=self.model.body(i).name,
                position=pos.tolist(),quaternion_xyzw=Rotation.from_matrix(rotation).as_quat().tolist()))
        optical=np.asarray(self.mount['body_from_camera_optical'])
        transforms.append(dict(parent=self.camera_body,child='camera_optical_frame',position=optical[:3,3].tolist(),
            quaternion_xyzw=Rotation.from_matrix(optical[:3,:3]).as_quat().tolist()))
        tool=self.model.body('Tool_fixed').id
        self.latest_state=dict(kind='state',time=timestamp,joint_names=self.joint_names,
            positions=np.asarray(qpos)[self.qadr].astype(float).tolist(),
            velocities=np.asarray(qvel)[self.vadr].astype(float).tolist(),transforms=transforms,
            tool_pose=np.r_[self.data.xpos[tool],self.data.xquat[tool][[1,2,3,0]]].tolist())
        self.sender.put(self.latest_state)
        return self.latest_state

    def camera(self,observation,images,*,state=None):
        timestamp=float(observation['timestamp_s']);self.last_time=max(self.last_time,timestamp)
        state=self.latest_state if state is None else state
        if abs(state['time']-timestamp)>1e-9:raise ValueError('Camera and measured ROS state must share the exposure time')
        keys=('rgb','depth_m','instance','semantic','recognition')
        self.sender.put(dict(kind='camera',time=timestamp,observation=copy.deepcopy(observation),state=state),
                        {key:images[key] for key in keys})

    def feedback(self,value):
        self.sender.put(dict(kind='control_feedback',time=self.last_time,feedback=copy.deepcopy(value)))

    def status(self,report):
        keys=('status','reason','error','last_station_m','completed_harvest_motions',
              'assumed_harvest_count','physics_clock_s','harvest_mode','stop','control_mode','simulated_s')
        self.sender.put(dict(kind='status',time=self.last_time,status={k:report[k] for k in keys if k in report}))

    def close(self):
        if self.closed:return
        self.closed=True
        if self.commands:self.commands.closed=True
        self.sender.close()
        try:self.process.wait(timeout=5.)
        except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=5.)
        report=dict(self.config,sent=dict(self.sender.sent),dropped=dict(self.sender.dropped),
                    transport_error=self.sender.error,publisher_exit_code=self.process.returncode)
        (self.output/'transport.json').write_text(json.dumps(report,indent=2)+'\n')
        self.log.close();self.temp.cleanup()
        return report


class CommandInbox:
    """Bounded reverse transport; a lost ROS process is visible to the controller."""
    def __init__(self,sock):
        self.sock=sock;self.pending=deque();self.lock=threading.Lock();self.closed=False;self.error=None
        self.thread=threading.Thread(target=self.run,daemon=True,name='ros2-commands');self.thread.start()

    def run(self):
        try:
            while not self.closed:
                try:message,arrays=receive(self.sock)
                except socket.timeout:continue
                if message.get('kind')!='command' or arrays:raise ValueError('Invalid control packet')
                with self.lock:
                    if message['command_type']=='stop' and message['value']:self.pending.clear()
                    if len(self.pending)>=64:raise ValueError('Command intake overflow')
                    self.pending.append(message)
        except Exception as error:
            if not self.closed:self.error=repr(error)

    def drain(self):
        with self.lock:
            result=list(self.pending);self.pending.clear();return result
