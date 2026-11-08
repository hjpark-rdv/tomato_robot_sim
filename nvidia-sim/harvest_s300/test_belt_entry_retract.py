import ast
from contextlib import redirect_stderr
import io
from pathlib import Path
import unittest

import numpy as np

from belt_entry_retract import retract_translation,measured_entry_window
from motion_kinematics import grasp_pose,plan_motion
from test_motion_branch import RedundantCartesianRobot
from test_run_motion_conditioning import MAIN,actual_cli,code


class BeltEntryRetractTests(unittest.TestCase):
    FLAGS=['--whole-truss','--harvest-group-policy','/exact/policy.json',
        '--preparation','horizontal_ready','--preparation-height-first',
        '--approach-yaw-mode','truss-axis','--truss-order','highest-first','--harvest-profile','belt_open_close_cycle',
        '--grasp-depth-mm','15']

    def runner(self,yaw=0.):
        args=actual_cli(self.FLAGS+['--truss-entry-yaw-offset-deg',str(yaw)])
        kin=RedundantCartesianRobot();center=np.array([.4,1.,1.2])
        ready=grasp_pose(center,center.copy());ready[:3,3]-=ready[:3,1]*.3
        start=kin.ik_at_lift(ready,np.zeros(7),.12)[0]
        env=dict(a=args,np=np,plan_motion=plan_motion,planning_kin=kin,reference_center=center,
            initial_robot_q=start,recorded_prefix=None,initial_grasp_depth_mm=15.,
            initial_depth_evidence=None,rail_destination=None,resume_rail_origin=None)
        function=next(n for n in ast.walk(MAIN)if isinstance(n,ast.FunctionDef)and n.name=='make_plan')
        exec(code([function]),env)
        axis=dict(direction_world=ready[:3,1].tolist())
        return env,kin,center,start,axis

    def test_zero_default_preserves_existing_arrays_and_metadata(self):
        env,kin,center,start,axis=self.runner()
        actual,meta=env['make_plan'](axis=axis)
        original,old=plan_motion(kin,center,start,preparation='horizontal_ready',
            harvest_profile='belt_open_close_cycle',preparation_height_first=True,
            approach_direction_world=axis['direction_world'],grasp_depth_m=.015)
        self.assertEqual(meta['phases'],old['phases']);self.assertNotIn('belt_entry_retract',meta)
        for a,b in zip(actual,original):np.testing.assert_array_equal(a[2],b[2])
        self.assertEqual(env['a'].belt_entry_retract_mm,0.)

    def test_actual_runner_moves_goal_and_preparation_fk_five_and_ten_mm_without_yaw_change(self):
        for yaw in (0.,30.,-30.):
            env,kin,center,start,axis=self.runner(yaw)
            _,old=env['make_plan'](axis=axis);goal=np.asarray(old['goal_pose'])
            points=np.array([[-.01,.143,.037],[.01,.143,.037]])@goal[:3,:3].T+goal[:3,3]
            fruits=[dict(name=f'f{i}',dimensions_m=[.023]*3)for i in range(2)]
            group=dict(fruits=['f0','f1'],measured_centers_world_m=points.tolist())
            env['sequence_fruits']=fruits;original_points=points.copy()
            for amount in (5.,10.):
                env['a'].belt_entry_retract_mm=amount
                _,new=env['make_plan'](axis=axis,selected_group=group)
                for key,qkey in [('goal_pose','goal_q'),('prep_pose','prep_q')]:
                    before=np.asarray(old[key]);after=np.asarray(new[key])
                    np.testing.assert_array_equal(after[:3,:3],before[:3,:3])
                    np.testing.assert_allclose(after[:3,3]-before[:3,3],-before[:3,1]*amount/1000,atol=1e-12)
                    np.testing.assert_allclose(kin.fk(new[qkey]),after,atol=1e-5)
                info=new['belt_entry_retract'];window=info['entry_window']
                self.assertEqual(info['amount_mm'],amount)
                self.assertEqual(window['overlaps_belt_depth_span'],[True,True])
                self.assertEqual(window['fully_inside_belt_depth_span'],[False,False])
                self.assertFalse(window['all_selected_fruits_fully_inside_belt_depth_span'])
                self.assertFalse(info['plant_pose_modified']);self.assertFalse(info['physical_contact_verified'])
                np.testing.assert_array_equal(points,original_points)

    def test_already_applied_heading_is_not_rotated_again(self):
        env,_,_,_,axis=self.runner(30.)
        from truss_sequence import entry_direction
        rotated=entry_direction(axis,30.)
        actual_axis=dict(direction_world=rotated.tolist(),direction_offset_already_applied=True)
        _,first=env['make_plan'](axis=axis);_,second=env['make_plan'](axis=actual_axis)
        np.testing.assert_array_equal(first['goal_pose'],second['goal_pose'])
        delta,evidence=retract_translation([0.,0.,0.],rotated,10.)
        np.testing.assert_allclose(delta,-rotated*.010,atol=1e-12)

    def test_partial_window_evidence_is_not_success_and_outside_span_fails(self):
        fruits=[dict(name='one',dimensions_m=[.023]*3)]
        group=dict(fruits=['one'],measured_centers_world_m=[[0.,.16,.037]])
        evidence=measured_entry_window(group,fruits,np.eye(4))
        self.assertEqual(evidence['overlaps_belt_depth_span'],[True])
        self.assertEqual(evidence['fully_inside_belt_depth_span'],[False])
        self.assertIsNone(evidence['harvest_success']);self.assertFalse(evidence['training_eligible'])
        group['measured_centers_world_m']=[[0.,.18,.037]]
        with self.assertRaisesRegex(ValueError,'does not overlap'):measured_entry_window(group,fruits,np.eye(4))

    def test_invalid_values_and_nonhorizontal_directions_fail(self):
        for value in (True,-1.,15.001,float('nan'),float('inf'),'5'):
            with self.assertRaises(ValueError):retract_translation([0.,0.,0.],[0.,1.,0.],value)
        for direction in ([0.,0.,0.],[0.,1.,.1],[float('nan'),0.,0.]):
            with self.assertRaises(ValueError):retract_translation([0.,0.,0.],direction,5.)

    def test_cli_is_opt_in_and_independent_idle_capture_strips_the_control_option(self):
        from run_row_harvest import startup_preflight_arguments
        for amount in ('0','5','10','15'):
            parsed=actual_cli(self.FLAGS+['--belt-entry-retract-mm',amount])
            self.assertEqual(parsed.belt_entry_retract_mm,float(amount))
            idle=actual_cli(startup_preflight_arguments(self.FLAGS+['--belt-entry-retract-mm',amount]))
            self.assertEqual(idle.mode,'idle');self.assertEqual(idle.belt_entry_retract_mm,0.)
        for tail in (['--belt-entry-retract-mm',v]for v in ('-1','15.1','nan','inf')):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(self.FLAGS+tail)
        with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(['--belt-entry-retract-mm','5'])

    def test_suffix_fingerprint_contains_only_explicit_nonzero_backoff(self):
        source=(Path(__file__).parent/'run_motion.py').read_text()
        self.assertIn("if getattr(a,'belt_entry_retract_mm',0.):\n                        fingerprint_parameters['belt_entry_retract_mm']",source)


if __name__=='__main__':unittest.main()
