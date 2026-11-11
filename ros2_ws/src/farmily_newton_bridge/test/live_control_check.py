"""Explicit integration exercise against the simulator's dedicated DDS domain.

Not collected as a unit test. Requires a running `ros-control` simulator.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import JointState,Image
from std_msgs.msg import Float64,Bool,String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

class Exercise(Node):
    def __init__(self,namespace):
        super().__init__('farmily_control_validation');self.q=None;self.v=None;self.feedback=None;self.images=0
        self.samples=[];self.events={};self.checks=[];self.image_stamps=[]
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor=QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState,namespace+'/joint_states',self.state,10)
        self.create_subscription(String,namespace+'/command_feedback',self.status,retained)
        self.create_subscription(Image,namespace+'/camera/color/image_raw',self.image,sensor)
        self.arm=self.create_publisher(JointTrajectory,namespace+'/arm_controller/joint_trajectory',1)
        self.base=self.create_publisher(Twist,namespace+'/cmd_vel',1)
        self.lift=self.create_publisher(Float64,namespace+'/lift/command',1)
        self.gripper=self.create_publisher(Float64,namespace+'/gripper/command',1)
        self.stop=self.create_publisher(Bool,namespace+'/stop',1)
    def state(self,m):
        self.q=np.array(m.position);self.v=np.array(m.velocity);self.names=list(m.name)
        self.samples.append(dict(wall=time.monotonic(),time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            q=self.q.tolist(),v=self.v.tolist()))
    def status(self,m):
        self.feedback=json.loads(m.data)
        for event in self.feedback['recent_events']:self.events[(event['sequence'],event['status'])]=event
    def image(self,m):self.images+=1;self.image_stamps.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    def wait(self,condition,timeout=12):
        deadline=time.monotonic()+timeout
        while not condition():
            if time.monotonic()>deadline:raise AssertionError('Timed out: '+str(self.feedback))
            rclpy.spin_once(self,timeout_sec=.02)
    def spin(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.02)
    def trajectory(self,names,targets,seconds):
        m=JointTrajectory();m.joint_names=names;p=JointTrajectoryPoint();p.positions=[float(v) for v in targets]
        p.time_from_start.sec=int(seconds);p.time_from_start.nanosec=round((seconds-int(seconds))*1e9)
        m.points=[p];self.arm.publish(m)
    def speed(self,value):
        m=Twist();m.linear.x=float(value);self.base.publish(m)
    def new_events(self,previous,kind,status):
        return [v for k,v in self.events.items() if k not in previous and v['command_type']==kind and v['status']==status]
    def run(self):
        self.wait(lambda:self.q is not None and self.feedback and self.feedback.get('state')=='ready',90)
        self.spin(.3);initial=self.q.copy();start=time.monotonic();old=set(self.events)
        targets=initial[2:8]+np.array([.015,-.01,.01,-.01,.01,.08]);target=targets[-1]
        self.trajectory(self.names[2:8],targets,1.5)
        self.wait(lambda:bool(self.new_events(old,'arm_trajectory','accepted')))
        accepted=time.monotonic()-start
        self.wait(lambda:abs(self.q[7]-initial[7])>.002)
        first_motion=time.monotonic()-start
        self.wait(lambda:bool(self.new_events(old,'arm_trajectory','completed')))
        assert np.max(np.abs(self.q[2:8]-targets))<.01
        self.checks.append(dict(test='arm',accepted_latency_s=accepted,first_motion_s=first_motion,target_rad=target,actual_rad=float(self.q[7]),
            joint_names=self.names[2:8],targets_rad=targets.tolist(),actual_rad_all_axes=self.q[2:8].tolist()))
        old=set(self.events);self.lift.publish(Float64(data=.22));self.gripper.publish(Float64(data=.096))
        self.wait(lambda:bool(self.new_events(old,'lift','completed')) and bool(self.new_events(old,'gripper','completed')))
        self.checks.append(dict(test='lift_and_gripper',lift_m=float(self.q[1]),gap_m=float(.044+self.q[9]-self.q[8])))
        base0=float(self.q[0]);deadline=time.monotonic()+1.6
        while time.monotonic()<deadline:self.speed(.06);self.spin(.1)
        self.wait(lambda:abs(self.v[0])<.003,4)
        base1=float(self.q[0]);assert base1>base0+.03
        self.checks.append(dict(test='base_forward_and_watchdog',start_m=base0,end_m=base1,stopped_speed=float(self.v[0])))
        deadline=time.monotonic()+.7
        while time.monotonic()<deadline:self.speed(-.04);self.spin(.1)
        self.wait(lambda:abs(self.v[0])<.003,4)
        assert self.q[0]<base1-.01
        self.checks.append(dict(test='base_reverse',end_m=float(self.q[0])))
        old=set(self.events);self.trajectory(['wrist3'],[float(self.q[7]+.2)],2.5);self.spin(.45)
        self.stop.publish(Bool(data=True));self.wait(lambda:self.feedback.get('state')=='stopped')
        self.wait(lambda:np.max(np.abs(self.v))<.01)
        self.trajectory(['wrist3'],[float(self.q[7]+.1)],2.)
        self.wait(lambda:bool(self.new_events(old,'arm_trajectory','rejected')))
        self.checks.append(dict(test='stop_latch',maximum_speed=float(np.max(np.abs(self.v)))))
        self.stop.publish(Bool(data=False));self.wait(lambda:self.feedback.get('state')=='ready')
        old=set(self.events);self.lift.publish(Float64(data=10.))
        self.wait(lambda:bool(self.new_events(old,'lift','rejected')))
        old=set(self.events);self.trajectory(['invalid_joint'],[0.],2.)
        self.wait(lambda:bool(self.new_events(old,'arm_trajectory','rejected')))
        self.checks.append(dict(test='invalid_limits_and_names',passed=True))
        self.wait(lambda:self.images>=2,10)
        self.checks.append(dict(test='live_camera',frames=self.images,distinct_timestamps=len(set(self.image_stamps))))
        return dict(passed=True,checks=self.checks,events=list(self.events.values()),samples=self.samples,
                    image_stamps=self.image_stamps,final_feedback=self.feedback)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--namespace',default='/farmily/sim')
    args=p.parse_args();rclpy.init();node=Exercise(args.namespace)
    try:result=node.run()
    except BaseException as e:
        args.output.write_text(json.dumps(dict(passed=False,error=repr(e),checks=node.checks,events=list(node.events.values()),samples=node.samples),indent=2));raise
    else:args.output.write_text(json.dumps(result,indent=2));print(json.dumps(result['checks'],indent=2))
    finally:
        node.speed(0.);node.stop.publish(Bool(data=True));node.spin(.2);node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
