"""ROS command serialization; no simulator or hardware API calls here."""
import json
import queue
import threading
import time
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from .protocol import encode


class CommandForwarder:
    def __init__(self, sock):
        self.sock=sock;self.queue=queue.Queue(maxsize=64);self.closed=threading.Event();self.error=None
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def submit(self, command):
        packet=encode(command)  # Reject NaN and oversize before accepting work.
        if self.error or self.closed.is_set():raise ValueError('Simulation command connection unavailable')
        if command['command_type']=='stop' and command['value']:
            while True:
                try:self.queue.get_nowait()
                except queue.Empty:break
        try:self.queue.put_nowait(packet)
        except queue.Full:raise ValueError('Command queue full')

    def run(self):
        try:
            while not self.closed.is_set():
                try:packet=self.queue.get(timeout=.1)
                except queue.Empty:continue
                self.sock.sendall(packet)
        except OSError as error:self.error=repr(error)


def trajectory(msg):
    if len(msg.points)>512:raise ValueError('At most 512 trajectory points')
    return dict(joint_names=list(msg.joint_names),start_time=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
        points=[dict(positions=list(p.positions),velocities=list(p.velocities),accelerations=list(p.accelerations),
            effort=list(p.effort),time_from_start=p.time_from_start.sec+p.time_from_start.nanosec*1e-9)
            for p in msg.points])


def attach(node, sink):
    node.command_sequence=0;node.command_subscriptions=[]
    def subscribe(cls,topic,kind,convert):
        def callback(msg):
            node.command_sequence+=1
            command=dict(kind='command',sequence=node.command_sequence,command_type=kind,topic=topic,
                         received_monotonic=time.monotonic())
            try:
                command.update(convert(msg));sink(command)
            except Exception as error:
                event=dict(sequence=command['sequence'],command_type=kind,status='rejected',reason=str(error))
                node.command_feedback.publish(String(data=json.dumps(dict(recent_events=[event]))))
        node.command_subscriptions.append(node.create_subscription(cls,topic,callback,1))
    subscribe(JointTrajectory,'arm_controller/joint_trajectory','arm_trajectory',trajectory)
    subscribe(JointTrajectory,'joint_trajectory','joint_trajectory',trajectory)
    subscribe(Twist,'cmd_vel','cmd_vel',lambda m:dict(linear=[m.linear.x,m.linear.y,m.linear.z],
                                                    angular=[m.angular.x,m.angular.y,m.angular.z]))
    subscribe(Float64,'lift/command','lift',lambda m:dict(value=m.data))
    subscribe(Float64,'gripper/command','gripper',lambda m:dict(value=m.data))
    subscribe(Bool,'stop','stop',lambda m:dict(value=m.data))
