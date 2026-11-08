"""Real FCL suffix screening keeps actual asymmetric jaws separate from commands."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from compact_motion_assets import file_sha256
from motion_continuation import terminal_suffix_samples,source_command_sample
from motion_self_collision import MotionSelfCollision
from rail_environment_preflight import RailEnvironmentCheck,screen_rail_plan,scoped_plan,verify_initial_alignment
from test_rail_environment_preflight import fixture,Kinematics,cube


class TerminalSuffixEnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.stage,self.manifest,_=fixture()
        cube(self.stage,'/World/Greenhouse/House/Environment/Column_far',(10,10,10),.01)
        self.source=Path(self.tmp.name)/'source_plan.json'
        q=[[0.]*7]*2
        original=dict(start=[0.]*7,phases=[dict(name=n,duration_s=d,waypoint_q=q,end_gap_m=g,
            belt_local_y_m_s=[0.,0.],rail_waypoint_positions_m=[0.,0.])for n,d,g in
            [('ready_hold',.5,.044),('close',.5,.028),('cycle_open',1.,.096),('hold',1.,.044)]])
        self.source.write_text(json.dumps(original));proof=dict(schema='s300-terminal-forward-suffix-v1',
            source_plan_path=str(self.source),source_plan_sha256=file_sha256(self.source),
            source_elapsed_in_plan_s=1.5,source_remaining_s=1.5,source_checkpoint_tick=90,
            source_cycle_start_s=0.,source_hz=60,measured_initial_q=[0.]*7)
        self.plan=copy.deepcopy(original);self.plan['terminal_forward_suffix']=dict(
            schema='s300-terminal-forward-suffix-clock-v1',proof=proof,initial_source_elapsed_s=1.5,
            remaining_duration_s=1.5,source_checkpoint_tick=90,source_cycle_start_s=0.,source_hz=60)

    def tearDown(self):self.tmp.cleanup()
    def adapter(self):
        moving=MotionSelfCollision(self.stage,Kinematics())
        adapter=RailEnvironmentCheck(self.stage,self.manifest,moving,allowed_truss_key='Truss_02',scope='full')
        poses=moving.poses(np.zeros(7),.044)
        return adapter,poses

    def test_full_suffix_has_original_timestamps_and_skips_executed_preparation(self):
        adapter,poses=self.adapter()
        report=screen_rail_plan(adapter,self.plan,self.manifest,checkpoint_poses=poses,
            checkpoint_robot_state=dict(initial_jaw_q_m=dict(jaw_L=0.,jaw_R=0.)),hz=60,native_hz=60,scope='full')
        self.assertTrue(report['passed']);self.assertTrue(report['complete']);self.assertTrue(report['eligible_for_native_execution'])
        self.assertEqual(report['samples'],91);self.assertEqual(adapter.sample_schedule[0],('cycle_open',1.5))
        self.assertEqual(adapter.sample_schedule[-1],('hold',3.))
        self.assertEqual(report['nominal_full_path_duration_s'],1.5)
        self.assertEqual([r['phase']for r in report['sampling']['phases']],['cycle_open','hold'])
        self.assertIsNone(scoped_plan(self.plan,'preparation'));self.assertIsNone(scoped_plan(self.plan,'rail'))

    def test_actual_asymmetric_jaw_intersection_cannot_hide_behind_nominal_gap(self):
        # Tiny fixed/tool geometry isolates the measured jaw from its parent.
        self.stage.GetPrimAtPath('/World/ActuatedTool/fixed/shape').GetAttribute('size').Set(.001)
        cube(self.stage,'/World/ActuatedTool/L',(3,.15,0),.005,rigid=True)
        cube(self.stage,'/World/Greenhouse/House/Environment/Column_00',(3.01,.15,0),.003)
        adapter,poses=self.adapter();poses['/World/ActuatedTool/L']=poses['/World/ActuatedTool/L'].copy()
        poses['/World/ActuatedTool/L'][0,3]+=.01
        state=dict(initial_jaw_q_m=dict(jaw_L=.01))
        # Both actual FK and asymmetry are valid under unchanged1e-4m tolerance.
        verify_initial_alignment(adapter.moving,self.plan,self.manifest,poses,checkpoint_robot_state=state)
        # Nominal current command clears the same obstacle; actual jaw does not.
        first=source_command_sample(self.plan,1.5)
        nominal=adapter.check_state(first['q'],first['gap_m'],rail_position_m=0.)
        self.assertTrue(nominal['hard_clear'])
        state['initial_jaw_q_m']['jaw_R']=0.
        report=screen_rail_plan(adapter,self.plan,self.manifest,checkpoint_poses=poses,
            checkpoint_robot_state=state,hz=60,native_hz=60,scope='full')
        self.assertFalse(report['passed']);self.assertFalse(report['eligible_for_native_execution'])
        self.assertFalse(report['initial_actual_state_audit']['passed'])
        self.assertTrue(any(hit['phase']=='terminal_initial_actual'for hit in report['hard_hits']))

    def test_actual_start_mapping_error_remains_hard_with_original_tolerance(self):
        adapter,poses=self.adapter();poses['/World/HarvestRobot/Robot/arm']=poses['/World/HarvestRobot/Robot/arm'].copy()
        poses['/World/HarvestRobot/Robot/arm'][1,3]+=.001
        report=screen_rail_plan(adapter,self.plan,self.manifest,checkpoint_poses=poses,
            checkpoint_robot_state=dict(initial_jaw_q_m={}),hz=60,native_hz=60,scope='full')
        self.assertFalse(report['complete']);self.assertIn('Initial measured/FK pose mismatch',report['error'])


if __name__=='__main__':unittest.main()
