"""Actual camera runner/environment wiring for a proven route-start hold."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from camera_scan_prefix import retained_camera_arm_hold, verify_retained_camera_arm_hold, direct_scan_prefix


ROOT = Path(__file__).parent


def actual_assignment(source, target):
    tree = ast.parse((ROOT/source).read_text())
    return next(node for node in ast.walk(tree) if isinstance(node, ast.Assign)
        and any(isinstance(item, ast.Name) and item.id == target
            for left in node.targets for item in ast.walk(left)))


def execute(node, env):
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),
        'actual camera route hold binding', 'exec'), env)


class RouteCameraArmHoldTests(unittest.TestCase):
    def setUp(self):
        from test_native_route_checkpoint import NativeRouteCheckpointTests
        self.fixture = NativeRouteCheckpointTests();self.fixture.setUp()
        names=['root_joint','lift','base','shoulder','elbow','wrist1','wrist2','wrist3','jaw_L','jaw_R']
        self.measured=np.array([0.,.03,1.57,-.86,2.51,-.05,3.14,.015,-.026,.026])
        self.targets=self.measured.copy();self.targets[1]=.036256;self.targets[2:8]+=.001
        self.fixture.save(robot_dof_names=names,robot_q=self.measured,robot_qvel=np.arange(10)/100,
            robot_position_targets=self.targets,robot_velocity_targets=np.zeros(10))
        from native_route_checkpoint import load_checkpoint
        self.cp=self.fixture.root/'checkpoint';self.metadata,self.arrays=load_checkpoint(self.cp)
        self.kin=SimpleNamespace(names=names[1:8],bounds=np.array([[0.]+[-4.]*6,[.75]+[4.]*6]))
        self.route=dict(metadata=self.metadata,source_arrays=self.arrays,
            report=dict(checkpoint=str(self.cp)))
        self.before={p:p.read_bytes()for p in self.cp.iterdir()}

    def tearDown(self):
        self.assertEqual(self.before,{p:p.read_bytes()for p in self.before})
        self.fixture.tearDown()

    def run_hold(self,resume,route):
        env=dict(retained_camera_arm_hold=retained_camera_arm_hold,planning_kin=self.kin,
            initial_robot_q=self.measured[1:8].copy(),resume_context=resume,route_start_context=route)
        execute(actual_assignment('run_motion.py','camera_hold_q'),env)
        return env

    def test_actual_route_branch_preserves_native_six_arm_targets_and_measured_lift(self):
        env=self.run_hold(None,self.route)
        np.testing.assert_array_equal(env['initial_robot_q'],self.measured[1:8])
        np.testing.assert_array_equal(env['camera_hold_q'],np.r_[.03,self.targets[2:8]])
        self.assertEqual(env['camera_hold_proof']['checkpoint'],str(self.cp))
        self.assertTrue(env['camera_hold_proof']['lift_drive_target_not_used_as_physical_height'])
        self.assertFalse(env['camera_hold_proof']['physical_state_modified'])
        np.testing.assert_array_equal(self.arrays['robot_q'],self.measured)

    def test_existing_resume_precedes_archived_route_and_no_context_is_unchanged(self):
        context=dict(metadata=self.metadata,arrays=self.arrays,report=dict(checkpoint=str(self.cp)))
        env=self.run_hold(context,dict(invalid_archive=True))
        np.testing.assert_array_equal(env['camera_hold_q'],np.r_[.03,self.targets[2:8]])
        env=self.run_hold(None,None)
        np.testing.assert_array_equal(env['camera_hold_q'],self.measured[1:8])
        self.assertIsNone(env['camera_hold_proof'])

    def test_environment_uses_exact_validated_route_checkpoint_without_explicit_resume_flag(self):
        held=self.run_hold(None,self.route)
        _,plan=direct_scan_prefix(self.kin,self.measured[1:8],
            arm_hold_command_q=held['camera_hold_q'],arm_hold_provenance=held['camera_hold_proof'])
        env=dict(verify_retained_camera_arm_hold=verify_retained_camera_arm_hold,
            moving=SimpleNamespace(kin=self.kin),plan=plan,checkpoint=None,
            checkpoint_proof=dict(checkpoint=str(self.cp)))
        statement=actual_assignment('rail_environment_preflight.py','hold_proof')
        execute(statement,env)
        self.assertEqual(env['hold_proof'],held['camera_hold_proof'])
        bad=copy.deepcopy(plan);bad['phases'][0]['waypoint_q'][0][2]+=.001
        env['plan']=bad
        with self.assertRaisesRegex(ValueError,'exact native getter'):execute(statement,env)
        env['plan']=plan;env['checkpoint_proof']=None
        with self.assertRaisesRegex(ValueError,'exact checkpoint'):execute(statement,env)


if __name__=='__main__':unittest.main()
