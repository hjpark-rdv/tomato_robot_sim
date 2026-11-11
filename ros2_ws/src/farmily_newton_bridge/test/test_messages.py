import unittest
import json
import numpy as np
from farmily_newton_bridge.node import stamp,image_message,Bridge


class Collector:
    def __init__(self):self.messages=[]
    def publish(self,message):self.messages.append(message)


class MessageTests(unittest.TestCase):
    def test_stamp_carries_nanosecond_roundoff(self):
        t=stamp(1.9999999996)
        self.assertEqual((t.sec,t.nanosec),(2,0))
        with self.assertRaises(ValueError):stamp(-1.)

    def test_depth_is_float_meters_with_nan_and_optical_header(self):
        depth=np.array([[1.25,np.nan],[2.,.5]],dtype='<f4')
        msg=image_message(depth,'32FC1',stamp(2.5),'sim/camera_optical_frame')
        self.assertEqual((msg.height,msg.width,msg.step),(2,2,8))
        self.assertEqual(msg.header.frame_id,'sim/camera_optical_frame')
        self.assertFalse(msg.is_bigendian)
        np.testing.assert_equal(np.frombuffer(bytes(msg.data),'<f4').reshape(2,2),depth)

    def test_camera_info_tf_and_images_share_exposure_time(self):
        node=Bridge.__new__(Bridge);node.prefix='sim';node.counts={};node.last_time=-1.
        for key in ('clock','joints','tf','tool','info','observations'):setattr(node,key,Collector())
        node.images={k:Collector() for k in ('rgb','recognition','depth_m','instance','semantic')}
        state=dict(kind='state',time=2.5,joint_names=['base'],positions=[.1],velocities=[.2],
            transforms=[dict(parent='world',child='camera_optical_frame',position=[1.,2.,3.],quaternion_xyzw=[0.,0.,0.,1.])],
            tool_pose=[1.,2.,3.,0.,0.,0.,1.])
        obs=dict(K=[[10.,0.,1.],[0.,11.,1.5],[0.,0.,1.]],resolution_wh=[2,3],objects=[dict(instance_id=4294967295)])
        data=dict(kind='camera',time=2.5,state=state,observation=obs)
        images=dict(rgb=np.zeros((3,2,3),np.uint8),recognition=np.ones((3,2,3),np.uint8),
            depth_m=np.ones((3,2),np.float32),instance=np.full((3,2),4294967295,np.uint32),semantic=np.ones((3,2),np.uint8))
        node.publish_packet(data,images)
        info=node.info.messages[0]
        self.assertEqual((info.width,info.height),(2,3))
        self.assertEqual(list(info.p),[10.,0.,1.,0.,0.,11.,1.5,0.,0.,0.,1.,0.])
        for publisher in node.images.values():
            self.assertEqual(publisher.messages[0].header,info.header)
        self.assertEqual(node.tf.messages[0].transforms[0].header.stamp,info.header.stamp)
        self.assertEqual(node.images['instance'].messages[0].encoding,'32SC1')
        self.assertEqual(bytes(node.images['instance'].messages[0].data),images['instance'].tobytes())
        self.assertTrue((np.frombuffer(bytes(node.images['instance'].messages[0].data),'<i4')==-1).all())
        item=json.loads(node.observations.messages[0].data)['objects'][0]
        self.assertEqual(item,dict(instance_id=-1,source_instance_id=4294967295))
        clocks=len(node.clock.messages);joints=len(node.joints.messages)
        node.last_time=3.
        node.publish_packet(data,images)
        self.assertEqual(len(node.clock.messages),clocks)
        self.assertEqual(len(node.joints.messages),joints)
        self.assertEqual(node.images['rgb'].messages[-1].header.stamp,stamp(2.5))
        self.assertEqual(node.last_time,3.)
        delayed=json.loads(node.observations.messages[-1].data)
        self.assertEqual(delayed['current_timestamp_s'],3.)
        self.assertEqual(delayed['age_s'],.5)
        with self.assertRaisesRegex(ValueError,'backwards'):node.publish_packet(dict(kind='state',time=1.),{})


if __name__=='__main__':unittest.main()
