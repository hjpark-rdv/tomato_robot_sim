import unittest
import numpy as np
from farmily_newton_bridge.planner import Planner,snapshot_error,trajectory_message
from farmily_newton_bridge.commands import planned_trajectory
from std_msgs.msg import String
import json
from types import SimpleNamespace
from unittest.mock import Mock,patch


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.names=['rail','lift','base','shoulder','elbow','wrist1','wrist2','wrist3','jaw_L','jaw_R']
        self.q=np.zeros(10);self.v=np.zeros(10)
        self.feedback=dict(state='ready',motion_active=False,joint_names=self.names,target_positions=self.q.tolist())
        self.configuration=dict(model_sha256='a'*64,physics_control_subscriptions=True)
    def error(self):
        return snapshot_error(self.names,self.q,self.v,self.feedback,self.configuration,'a'*64)
    def test_matching_stationary_snapshot(self):self.assertIsNone(self.error())
    def test_wrong_model_or_readonly_controller_cannot_execute(self):
        self.configuration['model_sha256']='b'*64
        self.assertIn('hash',self.error())
        self.configuration['model_sha256']='a'*64;self.configuration['physics_control_subscriptions']=False
        self.assertIn('control mode',self.error())
    def test_moving_unsettled_or_stopped_state_cannot_plan(self):
        self.v[0]=.03;self.assertIn('moving',self.error());self.v[:]=0.
        self.q[1]=.01;self.assertIn('settled',self.error());self.q[:]=0.
        self.feedback['motion_active']=True;self.assertIn('completed',self.error())
        self.feedback.update(motion_active=False,state='stopped');self.assertIn('latched',self.error())
    def test_serialization_retains_atomic_preconditions(self):
        path=dict(joint_names=['wrist3'],start_time=0.,points=[dict(positions=[.2],time_from_start=1.9999999999)])
        msg=trajectory_message(path)
        self.assertEqual((msg.points[0].time_from_start.sec,msg.points[0].time_from_start.nanosec),(2,0))
        payload=dict(trajectory=path,expected_start_positions=[0.]*10,source_model_sha256='a'*64,request_id='request1',sequence=99)
        decoded=planned_trajectory(String(data=json.dumps(payload)))
        self.assertEqual(decoded['expected_start_positions'],payload['expected_start_positions'])
        self.assertEqual(decoded['request_id'],'request1');self.assertNotIn('sequence',decoded)
        payload['request_id']='';
        with self.assertRaises(ValueError):planned_trajectory(String(data=json.dumps(payload)))

    def test_lost_command_feedback_stops_despite_live_joint_states(self):
        node=SimpleNamespace(future=None,active=dict(id='plan1',phase='executing'),announced_ready=True,
                             state_wall=100.,feedback_wall=96.,stop_pub=Mock(),publish=Mock())
        with patch('farmily_newton_bridge.planner.time.monotonic',return_value=100.):Planner.tick(node)
        self.assertIsNone(node.active)
        self.assertTrue(node.stop_pub.publish.call_args.args[0].data)
        self.assertEqual(node.publish.call_args.args[0],'failed')


if __name__=='__main__':unittest.main()
