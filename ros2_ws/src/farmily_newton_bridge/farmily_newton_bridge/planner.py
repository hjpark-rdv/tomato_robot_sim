"""ROS goal topics -> resident native OMPL -> guarded Newton GPU execution."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ROOT=Path(__file__).resolve().parents[4]


class Worker:
    """One loaded collision model, with bounded requests and no simulator stepping."""
    def __init__(self,assets,output):
        self.log=(output/'worker.log').open('w');self.messages=queue.Queue(maxsize=4)
        self.process=subprocess.Popen([str(ROOT/'nvidia-sim/.newton-venv/bin/python'),'-u',
            str(ROOT/'nvidia-sim/harvest_s300/run_newton_ompl_worker.py'),str(assets)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1,
            env={k:v for k,v in os.environ.items() if k not in ('PYTHONPATH','VIRTUAL_ENV')})
        def reader():
            try:
                while True:
                    line=self.process.stdout.readline(2_000_000)
                    if not line:raise RuntimeError('OMPL worker exited; see worker.log')
                    if not line.endswith('\n'):raise ValueError('OMPL response exceeds size limit')
                    self.messages.put(json.loads(line),timeout=1.)
            except BaseException as e:
                try:self.messages.put(e,timeout=1.)
                except queue.Full:pass
        self.thread=threading.Thread(target=reader,daemon=True);self.thread.start()
        try:
            self.ready=self.receive(45.)
            if self.ready.get('kind')!='ready':raise RuntimeError('OMPL worker failed to initialize: '+str(self.ready))
        except BaseException:self.close();raise

    def receive(self,timeout):
        try:value=self.messages.get(timeout=timeout)
        except queue.Empty:raise TimeoutError('OMPL worker response timeout')
        if isinstance(value,BaseException):raise value
        return value

    def plan(self,request):
        self.process.stdin.write(json.dumps(request,allow_nan=False)+'\n');self.process.stdin.flush()
        try:response=self.receive(float(request['timeout_s'])+15.)
        except BaseException:
            self.process.terminate();raise
        if response.get('id')!=request['id']:raise RuntimeError('OMPL response/request mismatch')
        return response

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=3.)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.log.close()


def trajectory_message(path):
    msg=JointTrajectory();msg.joint_names=list(path['joint_names'])
    for row in path['points']:
        point=JointTrajectoryPoint();point.positions=[float(v) for v in row['positions']]
        point.velocities=[float(v) for v in row.get('velocities',[])]
        point.accelerations=[float(v) for v in row.get('accelerations',[])]
        nanos=round(float(row['time_from_start'])*1e9)
        point.time_from_start.sec,point.time_from_start.nanosec=divmod(nanos,1_000_000_000)
        msg.points.append(point)
    return msg


def snapshot_error(names,q,v,feedback,configuration,model_hash):
    if configuration is None or configuration.get('model_sha256')!=model_hash:
        return 'Planner and live simulator model hashes do not match'
    if not configuration.get('physics_control_subscriptions'):
        return 'Simulator is not in ROS command control mode'
    if feedback is None or feedback.get('state')!='ready':return 'Controller is not ready or stop is latched'
    if feedback.get('motion_active',True):return 'Wait until the current motion has completed'
    if names!=feedback.get('joint_names'):return 'Joint order mismatch'
    targets=np.asarray(feedback.get('target_positions',[]),float)
    if targets.shape!=(10,) or q.shape!=(10,) or v.shape!=(10,) or not np.isfinite(np.r_[q,v,targets]).all():
        return 'Finite ten-axis measured state and command targets required'
    if np.max(np.abs(v))>.02:return 'Measured robot is still moving'
    if np.any(np.abs(q-targets)>np.array([.002,.002]+[.01]*6+[.001,.001])):
        return 'Measured state has not settled at the command target'
    return None


class Planner(Node):
    def __init__(self,args):
        super().__init__('newton_ompl_planner',namespace=args.namespace)
        self.args=args;self.output=args.output;self.output.mkdir(parents=True,exist_ok=False)
        self.q=self.v=self.names=self.feedback=self.configuration=None
        self.state_wall=self.feedback_wall=-float('inf')
        self.active=None;self.future=None;self.counter=0;self.announced_ready=False
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='ompl-request')
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub=self.create_publisher(String,'planner/status',retained)
        self.preview=self.create_publisher(JointTrajectory,'planner/trajectory',retained)
        self.execute=self.create_publisher(String,'planner/execute_trajectory',1)
        self.stop_pub=self.create_publisher(Bool,'stop',1)
        self.worker=Worker(args.assets,self.output)
        self.model_hash=self.worker.ready['model_sha256']
        self.create_subscription(JointState,'joint_states',self.on_state,5)
        self.create_subscription(String,'configuration',self.on_configuration,retained)
        self.create_subscription(String,'command_feedback',self.on_feedback,retained)
        self.create_subscription(JointState,'planner/joint_goal',self.on_joint_goal,1)
        self.create_subscription(PoseStamped,'planner/pose_goal',self.on_pose_goal,1)
        self.create_subscription(Bool,'stop',self.on_stop,1)
        self.timer=self.create_timer(.025,self.tick)
        self.publish('waiting_for_simulator',detail='OMPL RRTConnect ready; waiting for measured stationary state',
                     source_model_sha256=self.model_hash,worker=self.worker.ready)

    def publish(self,status,**values):
        self.counter+=1
        row=dict(event=self.counter,status=status,wall_monotonic=time.monotonic(),**values)
        self.status_pub.publish(String(data=json.dumps(row,allow_nan=False)))
        with (self.output/'events.jsonl').open('a') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
        return row

    def on_state(self,msg):
        self.names=list(msg.name);self.q=np.asarray(msg.position,float);self.v=np.asarray(msg.velocity,float)
        self.state_wall=time.monotonic()

    def on_configuration(self,msg):self.configuration=json.loads(msg.data)

    def on_feedback(self,msg):
        value=json.loads(msg.data)
        if 'state' in value:
            self.feedback=value;self.feedback_wall=time.monotonic()
        if not self.active:return
        for event in value.get('recent_events',[]):
            if event.get('request_id')!=self.active['id']:continue
            status=event['status']
            if status=='accepted' and self.active['phase']=='awaiting_acceptance':
                self.active['phase']='executing';self.publish('executing',request_id=self.active['id'])
            elif status in ('completed','rejected','preempted','tracking_failed'):
                self.publish(status,request_id=self.active['id'],execution_feedback=event);self.active=None;break

    def on_stop(self,msg):
        if msg.data and self.active:
            self.publish('cancelled',request_id=self.active['id'],reason='stop topic');self.active=None

    def ready_error(self):
        if self.q is None or time.monotonic()-self.state_wall>.5:return 'Fresh joint_states required (max age 0.5 wall seconds)'
        if time.monotonic()-self.feedback_wall>1.:return 'Fresh controller feedback required (max age 1 wall second)'
        return snapshot_error(self.names,self.q,self.v,self.feedback,self.configuration,self.model_hash)

    def request(self,goal):
        request_id=uuid.uuid4().hex
        if self.active or self.future is not None:
            self.publish('rejected',request_id=request_id,reason='Planner busy; wait or stop the current goal');return
        error=self.ready_error()
        if error:self.publish('rejected',request_id=request_id,reason=error);return
        request=dict(id=request_id,goal=goal,start_positions=list(self.feedback['target_positions']),
                     measured_positions=self.q.tolist(),timeout_s=self.args.planning_seconds)
        self.active=dict(id=request_id,phase='planning',request=request,started=time.monotonic())
        self.publish('planning',request_id=request_id,goal=goal)
        self.future=self.pool.submit(self.worker.plan,request)

    def on_joint_goal(self,msg):
        if msg.velocity or msg.effort:
            self.publish('rejected',reason='Joint goal specifies only joint names and positions');return
        if len(msg.name)!=len(msg.position) or not msg.name or not np.isfinite(msg.position).all():
            self.publish('rejected',reason='Finite named joint goal required');return
        self.request(dict(joint_names=list(msg.name),positions=list(msg.position)))

    def on_pose_goal(self,msg):
        accepted=('world',self.args.namespace.strip('/')+'/world')
        if msg.header.frame_id not in accepted:
            self.publish('rejected',reason='Pose goal frame must be '+accepted[1]);return
        p=msg.pose.position;q=msg.pose.orientation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1.)>1e-3:
            self.publish('rejected',reason='Finite pose and unit quaternion required');return
        self.request(dict(position=values[:3],quaternion_xyzw=values[3:]))

    def tick(self):
        if not self.announced_ready and self.active is None and self.ready_error() is None:
            self.announced_ready=True
            self.publish('ready',detail='Measured stationary state and model verified; goal topics ready')
        if self.future is not None and self.future.done():
            future=self.future;self.future=None
            try:response=future.result()
            except Exception as error:
                if self.active:self.publish('failed',request_id=self.active['id'],reason=str(error));self.active=None
                return
            (self.output/(response['id']+'.json')).write_text(json.dumps(response,indent=2,allow_nan=False)+'\n')
            if self.active is None or response['id']!=self.active['id']:return
            if not response.get('success'):
                self.publish('failed',request_id=response['id'],reason=response.get('error','No valid path'),report=response.get('report'));self.active=None;return
            error=self.ready_error()
            if error is None and not np.allclose(self.active['request']['start_positions'],self.feedback['target_positions'],atol=1e-6,rtol=0):
                error='Command target changed while planning; request a new plan'
            if error:
                self.publish('rejected',request_id=response['id'],reason=error);self.active=None;return
            self.preview.publish(trajectory_message(response['trajectory']))
            self.publish('planned',request_id=response['id'],report=response['report'],execute=not self.args.plan_only)
            if self.args.plan_only:self.active=None;return
            self.execute.publish(String(data=json.dumps(dict(request_id=response['id'],trajectory=response['trajectory'],
                expected_start_positions=self.active['request']['start_positions'],source_model_sha256=self.model_hash),allow_nan=False)))
            self.active.update(phase='awaiting_acceptance',submitted=time.monotonic())
        if self.active and self.active['phase'] in ('awaiting_acceptance','executing'):
            now=time.monotonic()
            if (now-self.state_wall>1. or now-self.feedback_wall>3. or
                    (self.active['phase']=='awaiting_acceptance' and now-self.active['submitted']>3.)):
                self.stop_pub.publish(Bool(data=True));self.publish('failed',request_id=self.active['id'],reason='Simulator feedback timed out; stop requested');self.active=None

    def close(self):
        if self.active and self.active['phase'] in ('executing','awaiting_acceptance'):self.stop_pub.publish(Bool(data=True))
        self.worker.close();self.pool.shutdown(wait=True,cancel_futures=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--namespace',default='/farmily/sim');parser.add_argument('--planning-seconds',type=float,default=2.)
    parser.add_argument('--plan-only',action='store_true')
    args,extra=parser.parse_known_args()
    if not np.isfinite(args.planning_seconds) or not .05<=args.planning_seconds<=10:parser.error('Planning budget must be0.05..10s')
    rclpy.init(args=extra);node=None
    signal.signal(signal.SIGTERM,signal.default_int_handler)
    try:
        node=Planner(args);rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
