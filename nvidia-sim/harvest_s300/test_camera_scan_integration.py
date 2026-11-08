"""Execute the real initial scanner dispatch without Kit or native physics."""
import ast
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from motion_continuation import attach_resumed_rail_travel,attach_resume_rail_hold
from rail_motion import attach_rail_commands


class CameraKinematics:
    names=['lift','base','shoulder','elbow','wrist1','wrist2','wrist3']
    bounds=np.array([[-1.]+[-np.pi]*6,[1.]+[np.pi]*6])
    def ik(self,*args):raise AssertionError('Unused initial harvest IK was called')
    def frames(self,q):
        c,s=np.cos(q[1]),np.sin(q[1]);t=np.eye(4)
        t[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]];t[:3,3]=[0.,1.,1.+q[0]]
        return {'/World/HarvestRobot/Robot/link1':t}


class ActualCameraScanIntegrationTests(unittest.TestCase):
    def dispatch(self,*,camera=True,rail=None,origin=None,mode='harvest',planner=None):
        source=Path(__file__).with_name('run_motion.py');tree=ast.parse(source.read_text())
        branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and any(
            isinstance(row,ast.Assign) and isinstance(row.value,ast.Call) and
            isinstance(row.value.func,ast.Name) and row.value.func.id=='direct_scan_prefix'
            for row in n.body))
        with tempfile.TemporaryDirectory() as directory:
            a=SimpleNamespace(camera_harvest=camera,horizontal_ready_connector='joint',
                camera_scan_view='row-front',camera_scan_lift_height_m=None,
                preparation_opening='ready',ready_lift_offset_mm=0.,preparation_gap_mm=96.,
                preparation='horizontal_ready',harvest_profile='belt_open_close_cycle',
                close_seconds=6.,belt_seconds=2.,belt_ramp_seconds=.5,sparse_entry_duration_scale=1.,
                output=Path(directory),mode=mode)
            mount=np.eye(4);mount[:3,:3]=[[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]]
            planner=planner or Mock(side_effect=AssertionError('Full harvest planner is not needed for scan'))
            context=dict(a=a,np=np,json=json,planning_kin=CameraKinematics(),initial_robot_q=np.zeros(7),
                rail_destination=rail,resume_rail_origin=origin,authored_center=np.array([-.4,1.6,1.]),
                manifest=dict(mounted_rgbd=dict(body_from_camera_optical=mount)),
                attach_rail_commands=attach_rail_commands,attach_resumed_rail_travel=attach_resumed_rail_travel,
                attach_resume_rail_hold=attach_resume_rail_hold,make_plan=planner,
                initial_translation=np.array([.01,.02,.03]),initial_axis=dict(direction_world=[0.,1.,0.]),
                initial_height_mm=-10.,initial_last_aim=dict(source='unchanged actual selector'),
                camera_reference_duration=None,resume_context=None,route_start_context=None,
                initial_sequence_group=None)
            module=ast.fix_missing_locations(ast.Module(body=[branch],type_ignores=[]))
            exec(compile(module,str(source),'exec'),context)
            context['saved_scan_proposal']=(a.output/'scan_source_proposal.json').exists()
        return context,planner

    def test_camera_dispatch_bypasses_full_harvest_ik_but_executes_actual_rail_and_pan_builders(self):
        result,planner=self.dispatch(rail=dict(duration_s=3.,displacement_m=.3))
        planner.assert_not_called()
        self.assertEqual([p[0] for p in result['phases']],
            ['rail_move','ready_hold','prepare_open','prepare_open_hold','camera_pan','camera_pan_hold'])
        self.assertEqual(result['camera_reference_duration'],45.)
        self.assertEqual(result['planning']['camera_scan']['cycle_time_budget']['components_s']['rail'],3.)
        self.assertTrue(result['saved_scan_proposal'])
        self.assertNotIn('goal_pose',result['planning'])
        self.assertFalse(result['planning']['camera_scan']['harvest_entry_executed'])
        self.assertTrue(all(p[-2:]==(0.,0.) for p in result['phases']))
        np.testing.assert_array_equal(result['initial_robot_q'],np.zeros(7))

    def test_camera_resume_hold_and_resumed_rail_preserve_measured_origin_and_drive_target_separately(self):
        origin=dict(displacement_m=.2,native_position_target_m=.2001)
        result,_=self.dispatch(origin=origin)
        self.assertEqual(result['camera_reference_duration'],42.)
        for row in result['planning']['phases']:
            self.assertTrue(all(x==.2 for x in row['rail_waypoint_positions_m']))
            self.assertTrue(all(x==.2001 for x in row['rail_native_command_positions_m']))
        result,_=self.dispatch(rail=dict(duration_s=3.,displacement_m=.3),origin=origin)
        self.assertEqual(result['camera_reference_duration'],45.)
        move=result['planning']['phases'][0]
        np.testing.assert_array_equal(move['rail_waypoint_positions_m'],[.2,.3])
        np.testing.assert_array_equal(move['rail_native_command_positions_m'],[.2001,.3])

    def test_noncamera_harvest_calls_original_planner_with_exact_arguments_and_returns_original_commands(self):
        phases=[('opaque legacy commands',)];metadata=dict(legacy_marker=object())
        planner=Mock(return_value=(phases,metadata))
        result,_=self.dispatch(camera=False,planner=planner)
        call=planner.call_args
        np.testing.assert_array_equal(call.args[0],result['initial_translation'])
        self.assertIs(call.args[1],result['initial_axis'])
        self.assertEqual(call.kwargs,dict(move_rail_first=True,grasp_height_mm=-10.,
            last_aim_evidence=result['initial_last_aim'],selected_group=None))
        self.assertIs(result['phases'],phases);self.assertIs(result['planning'],metadata)
        self.assertIsNone(result['camera_reference_duration']);self.assertFalse(result['saved_scan_proposal'])

    def test_noncamera_idle_does_not_call_harvest_planner(self):
        result,planner=self.dispatch(camera=False,mode='idle')
        planner.assert_not_called();self.assertEqual(result['phases'],[]);self.assertEqual(result['planning'],{})


if __name__=='__main__':unittest.main()
