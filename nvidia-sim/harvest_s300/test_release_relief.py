"""Bounded physical unloading preserves the existing reverse entry and guards."""
import ast
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from motion_kinematics import plan_motion,release_relief_opening_check,cycle_opening_check


class PoseIK:
    world=np.eye(4)
    def __init__(self):self.calls=0
    def ik(self,pose,seed):
        self.calls+=1;q=np.asarray(seed).copy();q[:3]=pose[:3,3];q[3:6]=R.from_matrix(pose[:3,:3]).as_rotvec()
        return q,0.,0.
    def fk(self,q):
        result=np.eye(4);result[:3,3]=q[:3];result[:3,:3]=R.from_rotvec(q[3:6]).as_matrix();return result


class ReleaseReliefTests(unittest.TestCase):
    def plan(self,flag=None,minimum=None):
        kin=PoseIK();kwargs={} if flag is None else dict(release_relief_before_withdraw=flag)
        if minimum is not None:kwargs['release_relief_minimum_gap_m']=minimum
        phases,metadata=plan_motion(kin,[.2,1.3,1.],np.zeros(7),'horizontal_ready',
            harvest_profile='belt_open_close_cycle',horizontal_ready_connector='joint',**kwargs)
        return kin,phases,metadata

    def test_default_and_explicit_false_commands_are_bitwise_equal(self):
        _,default,dm=self.plan();_,disabled,em=self.plan(False)
        self.assertEqual(dm,em);self.assertEqual(len(default),len(disabled))
        self.assertNotIn('release_relief',dm['belt_cycle'])
        for left,right in zip(default,disabled):
            self.assertEqual(left[:2],right[:2]);np.testing.assert_array_equal(left[2],right[2]);self.assertEqual(left[3:],right[3:])

    def test_only_splits_existing_reverse_waypoints_without_new_ik_or_pose(self):
        oldkin,old,om=self.plan(False);kin,changed,metadata=self.plan(True)
        self.assertEqual(kin.calls,oldkin.calls)
        original=next(p for p in old if p[0]=='withdraw')
        relief=next(p for p in changed if p[0]=='release_relief')
        hold=next(p for p in changed if p[0]=='release_relief_hold')
        withdraw=next(p for p in changed if p[0]=='withdraw')
        np.testing.assert_array_equal(np.concatenate([relief[2],withdraw[2][1:]]),original[2])
        np.testing.assert_array_equal(hold[2],[relief[2][-1],relief[2][-1]])
        self.assertEqual(relief[1],1.);self.assertEqual(hold[1],1.)
        for phase in [relief,hold,withdraw]:self.assertEqual(phase[3:],(.096,0.,0.))
        oldprefix=old[:next(i for i,p in enumerate(old) if p[0]=='withdraw')]
        for left,right in zip(oldprefix,changed):
            self.assertEqual(left[:2],right[:2]);np.testing.assert_array_equal(left[2],right[2]);self.assertEqual(left[3:],right[3:])
        spec=metadata['belt_cycle']['release_relief']
        self.assertAlmostEqual(spec['nominal_prefix_m'],.015)
        self.assertAlmostEqual(spec['planned_fk_prefix_m'],.015)
        self.assertEqual(spec['minimum_measured_gap_m'],.09)
        self.assertEqual(metadata['belt_cycle']['full_open_gap_m'],.096)
        self.assertEqual(metadata['belt_cycle']['opening_tracking_tolerance_m'],.0005)
        self.assertTrue(metadata['belt_cycle']['withdraw_requires_measured_open'])
        self.assertFalse(spec['new_ik_solved']);self.assertFalse(spec['training_eligible'])

    def test_minimum_gap_is_only_for_limited_unload_never_full_withdrawal(self):
        for gap,minimum_ok,full_ok in [(.089999,False,False),(.090,True,False),(.0937118,True,False),(.0955001,True,True),(.096,True,True), (float('nan'),False,False)]:
            self.assertEqual(release_relief_opening_check([gap])['reached'],minimum_ok)
            self.assertEqual(cycle_opening_check([gap])['reached'],full_ok)

    def test_actual_runner_minimum_guard_stops_before_motion_and_records_native_pose(self):
        tree=ast.parse(Path(__file__).with_name('run_motion.py').read_text())
        branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=="relief and label in ('release_relief', 'release_relief_hold')")
        compiled=compile(ast.fix_missing_locations(ast.Module(body=[branch],type_ignores=[])),'actual_relief_guard','exec')
        for gap,minimum,reached in [(.0899,.09,False),(.0937118,.09,True),(.0877561,.085,True)]:
            q=np.zeros((1,7));q[0,6]=gap-.044
            ns=dict(relief={'enabled':True,'minimum_measured_gap_m':minimum},label='release_relief',q=q,jawids=[5,6],np=np,
                release_relief_opening_check=release_relief_opening_check,release_relief_feedback=None,
                t=75.,body_paths=['/World/ActuatedTool/fixed'],manifest={'tool':{'bodies':{'fixed':'/World/ActuatedTool/fixed'}}},
                poses=np.array([[1,2,3,0,0,0,1.]]),planning_kin=PoseIK(),rids=np.arange(7),stop_reason=None)
            exec(compiled,ns)
            self.assertEqual(ns['release_relief_feedback']['reached'],reached)
            self.assertEqual(ns['stop_reason'],None if reached else 'release_relief_minimum_open_not_reached')
            self.assertTrue(ns['release_relief_feedback']['checked_before_motion_command'])
            self.assertEqual(ns['release_relief_feedback']['begin_native_pose_world'][:3],[1.,2.,3.])

    def test_idle_startup_removes_both_explicit_harvest_diagnostics(self):
        from run_row_harvest import startup_preflight_arguments
        args=startup_preflight_arguments(['--headless','--experimental-tpu-fruit-close-2mm','--release-relief-before-withdraw','--release-relief-minimum-gap-mm','85'],physics_only=True)
        self.assertEqual(args,['--headless','--physics-only','--mode','idle','--seconds','3'])

    def test_explicit_85mm_unload_never_changes_existing_full_open_guard(self):
        self.assertFalse(release_relief_opening_check([.0877561])['reached'])
        self.assertTrue(release_relief_opening_check([.0877561],.085)['reached'])
        self.assertFalse(cycle_opening_check([.0877561])['reached'])
        _,original,om=self.plan(True);_,same,sm=self.plan(True,.090)
        self.assertEqual(om,sm)
        for left,right in zip(original,same):
            self.assertEqual(left[:2],right[:2]);np.testing.assert_array_equal(left[2],right[2]);self.assertEqual(left[3:],right[3:])
        _,changed,cm=self.plan(True,.085)
        for left,right in zip(original,changed):
            self.assertEqual(left[:2],right[:2]);np.testing.assert_array_equal(left[2],right[2]);self.assertEqual(left[3:],right[3:])
        self.assertEqual(cm['belt_cycle']['release_relief']['minimum_measured_gap_m'],.085)
        self.assertEqual(cm['belt_cycle']['full_open_gap_m'],.096)
        self.assertEqual(cm['belt_cycle']['opening_tracking_tolerance_m'],.0005)
        for value in [.084,float('nan'),float('inf'),.097]:
            with self.assertRaises(ValueError):self.plan(True,value)
        with self.assertRaises(ValueError):self.plan(False,.085)

    def test_cli_rejects_84_nan_and_unapproved_nondefault_before_native_launch(self):
        script=Path(__file__).with_name('run_motion.py')
        with tempfile.TemporaryDirectory() as temp:
            base=[sys.executable,str(script),temp,'--output',temp+'/out','--mode','harvest',
                '--preparation','horizontal_ready','--harvest-profile','belt_open_close_cycle']
            for extra in [['--release-relief-before-withdraw','--release-relief-minimum-gap-mm','84'],
                          ['--release-relief-before-withdraw','--release-relief-minimum-gap-mm','nan'],
                          ['--release-relief-minimum-gap-mm','85']]:
                result=subprocess.run(base+extra,capture_output=True,text=True)
                self.assertEqual(result.returncode,2,result.stderr)
                self.assertIn('Release-relief minimum must be finite85..96mm',result.stderr)
                self.assertFalse(Path(temp+'/out').exists())


if __name__=='__main__':unittest.main()
