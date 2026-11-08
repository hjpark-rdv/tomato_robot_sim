"""Exercise the real native loop's lift-command seam without starting Kit."""
import ast
import unittest
from types import SimpleNamespace

import numpy as np

from lift_drive_reference import apply_lift_drive_reference
from test_run_motion_conditioning import LOOP, actual_cli, code


class CameraLiftReferenceIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.names=['root_joint','farmily_lift_height_joint','base','shoulder',
                    'elbow','wrist1','wrist2','wrist3','jaw_L','jaw_R']
        self.bounds=np.array([[-5.,0.]+[-7.]*6+[-.026,-.026],
                              [5.,.75]+[7.]*6+[.026,.026]])
        self.q=np.array([0.,.0192493,1.1,.23,2.49,.43,2.98,3.14,-.026,.026])
        target=.0275646
        self.reference=dict(schema='s300-lift-drive-reference-v1',joint_name=self.names[1],
            joint_kind='prismatic',source='checkpoint_native_drive_getters',
            physical_geometry_is_measured=True,measured_lift_m=float(self.q[1]),
            native_position_target_m=target,retained_target_minus_measured_m=target-self.q[1],
            lift_bounds_m=[0.,.75],checkpoint_metadata_sha256='a'*64)
        assignment=next(n for n in LOOP.body if isinstance(n,ast.Assign) and
                        any(isinstance(t,ast.Name) and t.id=='desired_lift_step' for t in n.targets))
        branch=next(n for n in LOOP.body if isinstance(n,ast.If) and
                    ast.unparse(n.test)=='lift_drive_reference is not None')
        self.seam=code([assignment,branch])

    def execute(self,command,*,resume=True,t=0.,enabled=True):
        context=dict(np=np,a=SimpleNamespace(resume_original_cycle_suffix=False),
            qcommand=command.copy(),q0=self.q[None].copy(),names=self.names,
            resume_context={} if resume else None,t=t,
            lift_drive_reference=self.reference if enabled else None,lift_drive_bounds=self.bounds,
            apply_lift_drive_reference=apply_lift_drive_reference)
        exec(self.seam,context)
        return context

    def test_resume_settle_preserves_prior_target_without_double_bias(self):
        command=self.q.copy();command[1]=self.reference['native_position_target_m']
        result=self.execute(command[None],t=.5)
        np.testing.assert_array_equal(result['qcommand'],command[None])
        self.assertEqual(result['desired_lift_step'][0,1],self.q[1])

    def test_route_settle_and_ready_hold_share_one_physical_lift_reference(self):
        for resume,t in ((False,0.),(True,2.),(False,2.)):
            result=self.execute(self.q[None],resume=resume,t=t)
            self.assertEqual(result['qcommand'][0,1],self.reference['native_position_target_m'])
            np.testing.assert_array_equal(result['qcommand'][0,[0,2,3,4,5,6,7,8,9]],
                                          self.q[[0,2,3,4,5,6,7,8,9]])

    def test_disabled_default_retains_exact_existing_command_values(self):
        result=self.execute(self.q[None],enabled=False)
        np.testing.assert_array_equal(result['qcommand'],self.q[None])
        self.assertIsNone(result['desired_lift_step'])

    def test_cli_requires_camera_joint_mode_and_validates_existing_seed_list(self):
        from contextlib import redirect_stderr
        import io
        with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            actual_cli(['--retain-lift-drive-reference'])
        with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            actual_cli(['--camera-ik-seed-indices','2','2'])


if __name__=='__main__':unittest.main()
