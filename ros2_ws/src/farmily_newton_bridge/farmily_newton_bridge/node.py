"""Publish measured live snapshots; never command or step the simulator."""
import argparse
import copy
import json
import queue
import socket
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState, Image, CameraInfo
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from .protocol import receive, encode, ros_instance_id


def stamp(seconds):
    total = int(round(float(seconds)*1_000_000_000))
    if total < 0:raise ValueError('Negative simulation time')
    return Time(sec=total//1_000_000_000, nanosec=total%1_000_000_000)


def image_message(array, encoding, timestamp, frame):
    array = np.ascontiguousarray(array)
    result = Image()
    result.header.stamp = timestamp; result.header.frame_id = frame
    result.height, result.width = array.shape[:2]
    result.encoding = encoding; result.is_bigendian = False
    result.step = int(array.strides[0]); result.data = array.tobytes()
    return result


class Bridge(Node):
    def __init__(self, namespace):
        super().__init__('newton_bridge', namespace=namespace)
        self.prefix = namespace.strip('/')
        self.sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.joints = self.create_publisher(JointState, 'joint_states', 5)
        self.tool = self.create_publisher(PoseStamped, 'tool_pose', 5)
        self.tf = self.create_publisher(TFMessage, '/tf', 10)
        self.clock = self.create_publisher(Clock, '/clock', self.sensor_qos)
        self.status = self.create_publisher(String, 'status', retained)
        self.labels = self.create_publisher(String, 'camera/labels', retained)
        self.observations = self.create_publisher(String, 'camera/observations', self.sensor_qos)
        self.info = self.create_publisher(CameraInfo, 'camera/camera_info', self.sensor_qos)
        self.images = {key:self.create_publisher(Image, 'camera/'+topic, self.sensor_qos)
            for key,topic in [('rgb','color/image_raw'),('depth_m','depth/image_raw'),
                              ('instance','instance/image_raw'),('semantic','semantic/image_raw'),
                              ('recognition','recognition/image_raw')]}
        self.last_time = -1.; self.counts = {}; self.error = None

    def frame(self, name):return self.prefix+'/'+name

    def publish_packet(self, data, arrays):
        kind = data['kind']; self.counts[kind] = self.counts.get(kind,0)+1
        if kind == 'labels':
            self.labels.publish(String(data=json.dumps(data['labels'],allow_nan=False)));return
        timestamp = stamp(data['time'])
        if data['time'] < self.last_time-1e-9:raise ValueError('Telemetry clock moved backwards')
        self.last_time = max(self.last_time,data['time'])
        self.clock.publish(Clock(clock=timestamp))
        if kind == 'state':
            msg = JointState(); msg.header.stamp=timestamp; msg.header.frame_id=self.frame('world')
            msg.name=data['joint_names']; msg.position=data['positions']; msg.velocity=data['velocities']
            self.joints.publish(msg)
            transforms=[]
            for row in data['transforms']:
                t=TransformStamped();t.header.stamp=timestamp;t.header.frame_id=self.frame(row['parent'])
                t.child_frame_id=self.frame(row['child'])
                t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=row['position']
                t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=row['quaternion_xyzw']
                transforms.append(t)
            self.tf.publish(TFMessage(transforms=transforms))
            pose=PoseStamped();pose.header=msg.header
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=data['tool_pose'][:3]
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=data['tool_pose'][3:]
            self.tool.publish(pose)
        elif kind == 'camera':
            # Preserve an exact exposure-time TF even if intermediate state
            # samples were coalesced while the subscriber/transport was slow.
            self.publish_packet(data['state'],{})
            frame=self.frame('camera_optical_frame')
            encodings={'rgb':'rgb8','recognition':'rgb8','depth_m':'32FC1','instance':'32SC1','semantic':'mono8'}
            for key,pub in self.images.items():
                image=arrays[key]
                if key == 'instance':
                    if image.dtype!=np.dtype('<u4'):raise ValueError('Source instance image must be uint32')
                    image=image.view('<i4')
                pub.publish(image_message(image,encodings[key],timestamp,frame))
            obs=data['observation']; k=np.array(obs['K'],float)
            info=CameraInfo();info.header.stamp=timestamp;info.header.frame_id=frame
            info.width,info.height=obs['resolution_wh'];info.distortion_model='plumb_bob'
            info.d=[0.]*5;info.k=k.ravel().tolist();info.r=np.eye(3).ravel().tolist()
            info.p=np.column_stack((k,np.zeros(3))).ravel().tolist();self.info.publish(info)
            visible=copy.deepcopy(obs)
            for item in visible.get('objects',[]):
                item['source_instance_id']=item['instance_id']
                item['instance_id']=ros_instance_id(item['instance_id'])
            self.observations.publish(String(data=json.dumps(visible,allow_nan=False)))
        elif kind == 'status':self.status.publish(String(data=json.dumps(data['status'],allow_nan=False)))
        else:raise ValueError('Unknown telemetry packet '+kind)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--socket',required=True);parser.add_argument('--namespace',default='/farmily/sim')
    parser.add_argument('--report',type=Path,required=True)
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args)
    node=Bridge(args.namespace);sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    stop=threading.Event();inbox=queue.Queue(maxsize=4);failure=[]
    try:
        sock.connect(args.socket);sock.sendall(encode(dict(kind='ready')))
        def reader():
            try:
                while not stop.is_set():
                    packet=receive(sock)
                    while not stop.is_set():
                        try:inbox.put(packet,timeout=.1);break
                        except queue.Full:pass
            except (EOFError,OSError):stop.set()
            except Exception as error:failure.append(repr(error));stop.set()
        thread=threading.Thread(target=reader,daemon=True);thread.start()
        while rclpy.ok() and (not stop.is_set() or not inbox.empty()):
            try:data,arrays=inbox.get(timeout=.02)
            except queue.Empty:pass
            else:node.publish_packet(data,arrays)
            rclpy.spin_once(node,timeout_sec=0.)
    except Exception as error:
        failure.append(repr(error));raise
    finally:
        stop.set();sock.close()
        args.report.write_text(json.dumps(dict(published=node.counts,last_simulation_time=node.last_time,
            errors=failure,namespace=args.namespace),indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
