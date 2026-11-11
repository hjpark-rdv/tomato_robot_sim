"""Subscribe to the actual DDS topics and optionally save a reception audit."""
import argparse
import hashlib
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState,Image,CameraInfo
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def ns(t):return t.sec*1_000_000_000+t.nanosec


class Inspector(Node):
    def __init__(self,namespace):
        super().__init__('newton_topic_inspector')
        self.result=dict(namespace=namespace,counts={},images={},joints=[],camera_info=[],observations=[],
                         clocks=[],transforms={},status=[],labels=None)
        self.latest_images={};self.handles=[];self.ended=False
        sensor=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def sub(cls,topic,handler,qos):
            def callback(msg):
                self.result['counts'][topic]=self.result['counts'].get(topic,0)+1;handler(msg)
            self.handles.append(self.create_subscription(cls,topic,callback,qos))
        root=namespace.rstrip('/')
        sub(JointState,root+'/joint_states',lambda m:self.result['joints'].append(dict(
            stamp_ns=ns(m.header.stamp),names=list(m.name),position=list(m.position),velocity=list(m.velocity))),10)
        sub(PoseStamped,root+'/tool_pose',lambda m:None,10)
        sub(Clock,'/clock',lambda m:self.result['clocks'].append(ns(m.clock)),sensor)
        sub(TFMessage,'/tf',self.transforms,10)
        sub(String,root+'/status',self.status,retained)
        sub(String,root+'/camera/labels',lambda m:self.result.update(labels=json.loads(m.data)),retained)
        sub(String,root+'/camera/observations',lambda m:self.result['observations'].append(json.loads(m.data)),sensor)
        sub(CameraInfo,root+'/camera/camera_info',lambda m:self.result['camera_info'].append(dict(
            stamp_ns=ns(m.header.stamp),frame=m.header.frame_id,width=m.width,height=m.height,
            k=list(m.k),p=list(m.p),r=list(m.r),d=list(m.d))),sensor)
        for key,topic in [('rgb','color'),('depth_m','depth'),('instance','instance'),('semantic','semantic'),('recognition','recognition')]:
            sub(Image,root+'/camera/'+topic+'/image_raw',lambda m,key=key:self.image(key,m),sensor)

    def image(self,key,msg):
        raw=bytes(msg.data)
        record=dict(stamp_ns=ns(msg.header.stamp),frame=msg.header.frame_id,height=msg.height,width=msg.width,
            encoding=msg.encoding,step=msg.step,big_endian=msg.is_bigendian,sha256=hashlib.sha256(raw).hexdigest())
        dtype={'rgb8':'u1','mono8':'u1','32FC1':'<f4','32SC1':'<i4'}[msg.encoding]
        shape=(msg.height,msg.width,3) if msg.encoding=='rgb8' else (msg.height,msg.width)
        image=np.frombuffer(raw,dtype=dtype).reshape(shape)
        if key=='depth_m':
            valid=image[np.isfinite(image)];record.update(valid_pixels=len(valid),
                minimum_m=float(valid.min()) if len(valid) else None,maximum_m=float(valid.max()) if len(valid) else None)
        elif key in ('instance','semantic'):record['unique_ids']=np.unique(image).tolist()
        self.latest_images[key]=image.copy();self.result['images'].setdefault(key,[]).append(record)

    def transforms(self,msg):
        for item in msg.transforms:
            if not item.child_frame_id.startswith(self.result['namespace'].strip('/')+'/'):continue
            t=item.transform;key=str(ns(item.header.stamp))
            self.result['transforms'].setdefault(key,{})[item.child_frame_id]=dict(parent=item.header.frame_id,
                position=[t.translation.x,t.translation.y,t.translation.z],
                quaternion_xyzw=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])

    def status(self,msg):
        data=json.loads(msg.data);self.result['status'].append(data)
        if data.get('status') not in ('running',None):self.ended=True


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--namespace',default='/farmily/sim');parser.add_argument('--seconds',type=float,default=60.)
    parser.add_argument('--output',type=Path);parser.add_argument('--until-stopped',action='store_true')
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args);node=Inspector(args.namespace)
    start=time.monotonic();last=start
    try:
        while rclpy.ok() and time.monotonic()-start<args.seconds:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-last>2.:
                print(json.dumps(node.result['counts'],ensure_ascii=False),flush=True);last=time.monotonic()
            if args.until_stopped and node.ended:
                end=time.monotonic()+.5
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
                break
    finally:
        node.result['wall_s']=time.monotonic()-start
        if args.output:
            args.output.mkdir(parents=True,exist_ok=True)
            (args.output/'received.json').write_text(json.dumps(node.result,indent=2,allow_nan=False)+'\n')
            np.savez_compressed(args.output/'last_images.npz',**node.latest_images)
        print(json.dumps(dict(counts=node.result['counts'],received_any=bool(node.result['counts'])),indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if not node.result['counts']:raise SystemExit('No ROS messages received')


if __name__=='__main__':main()
