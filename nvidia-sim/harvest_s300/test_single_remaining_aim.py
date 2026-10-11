import ast
from contextlib import redirect_stderr
import io
from pathlib import Path
import unittest

import numpy as np

from single_remaining_aim import selected_aim
from test_run_motion_conditioning import actual_cli
from test_grasp_depth_schedule import GraspDepthScheduleTests
from motion_kinematics import grasp_pose, plan_motion
from test_motion_branch import RedundantCartesianRobot


class SingleRemainingAimTests(unittest.TestCase):
    def setUp(self):
        self.fruits=[dict(name=f'Truss_02_Fruit_{i:02d}',joint=f'joint{i}') for i in (9,10,11)]
        self.other=dict(name='Truss_01_Fruit_01',joint='other_joint')
        self.group=dict(fruits=['Truss_02_Fruit_10'])
        self.options=dict(height_mm=-1.5,depth_mm=50.,last_height_mm=-8.5,last_depth_mm=65.)

    def test_only_native_detachment_of_all_other_exact_truss_fruit_changes_aim(self):
        self.assertEqual(selected_aim(self.fruits,[],self.group,**self.options)[:2],(-1.5,50.))
        breaks=[dict(joint='joint9'),dict(joint='joint11')]
        h,d,e=selected_aim(self.fruits+[self.other],breaks,self.group,**self.options)
        self.assertEqual((h,d),(-8.5,65.));self.assertTrue(e['last_remaining_fruit'])
        self.assertEqual(e['remaining_exact_truss_fruits'],['Truss_02_Fruit_10'])
        self.assertTrue(e['physics_unchanged']);self.assertTrue(e['attempt_budget_unchanged'])
        # A branch release/name, or another truss's fruit, cannot count.
        invalid=[dict(joint='joint9'),dict(joint='other_joint'),dict(joint='Pedicel_11')]
        self.assertEqual(selected_aim(self.fruits+[self.other],invalid,self.group,**self.options)[:2],(-1.5,50.))

    def test_no_override_or_missing_selection_keeps_baseline(self):
        breaks=[dict(joint='joint9'),dict(joint='joint11')]
        self.assertEqual(selected_aim(self.fruits,breaks,self.group,height_mm=-1.5,depth_mm=50.)[:2],(-1.5,50.))
        self.assertEqual(selected_aim(self.fruits,breaks,None,**self.options)[:2],(-1.5,50.))
        self.assertEqual(selected_aim(self.fruits,[dict(joint=f'joint{i}') for i in (9,10,11)],self.group,**self.options)[:2],(-1.5,50.))

    def test_unknown_mixed_truss_and_nonfinite_input_rejected(self):
        for names in (['Unknown'],['Truss_02_Fruit_10','Truss_01_Fruit_01'],['Truss_02_Fruit_10']*2):
            with self.assertRaises(ValueError):selected_aim(self.fruits+[self.other],[],dict(fruits=names),**self.options)
        for key,bad in (('last_height_mm',float('nan')),('last_height_mm',51.),('last_depth_mm',14.),('last_depth_mm',True)):
            with self.assertRaises(ValueError):selected_aim(self.fruits,[],self.group,**dict(self.options,**{key:bad}))

    def test_real_cli_keeps_defaults_and_limits_existing_physical_scope(self):
        self.assertIsNone(actual_cli([]).last_fruit_height_offset_mm)
        self.assertIsNone(actual_cli([]).last_fruit_depth_mm)
        base=['--mode','harvest','--whole-truss','--preparation','horizontal_ready','--harvest-profile','belt_open_close_cycle']
        a=actual_cli(base+['--last-fruit-height-offset-mm','-8.5','--last-fruit-depth-mm','65'])
        self.assertEqual((a.last_fruit_height_offset_mm,a.last_fruit_depth_mm),(-8.5,65.))
        for args in (['--last-fruit-height-offset-mm','-8.5'],base+['--last-fruit-height-offset-mm','nan'],base+['--last-fruit-depth-mm','101']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(args)

    def test_actual_runner_fk_changes_absolute_aim_with_unchanged_default(self):
        source,tree,_=GraspDepthScheduleTests().real_parser()
        fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='make_plan')
        kin=RedundantCartesianRobot();center=np.array([.4,1.,1.2])
        ready=grasp_pose(center,center.copy());ready[:3,3]-=ready[:3,1]*.3
        start=kin.ik_at_lift(ready,np.zeros(7),.12)[0]
        a=actual_cli(['--preparation','horizontal_ready','--preparation-height-first','--harvest-profile','belt_open_close_cycle','--grasp-height-offset-mm','-1.5'])
        env=dict(a=a,plan_motion=plan_motion,planning_kin=kin,reference_center=center,
            initial_robot_q=start,recorded_prefix=None,initial_grasp_depth_mm=65.,initial_depth_evidence=None,
            rail_destination=None,resume_rail_origin=None)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),env)
        axis=dict(direction_world=ready[:3,1].tolist())
        ordinary,old=env['make_plan'](axis=axis)
        direct,direct_meta=plan_motion(kin,center,start,preparation='horizontal_ready',
            preparation_height_first=True,harvest_profile='belt_open_close_cycle',
            grasp_height_offset_m=-.0015,approach_direction_world=ready[:3,1])
        self.assertEqual(old['phases'],direct_meta['phases'])
        h,d,e=selected_aim(self.fruits,[dict(joint='joint9'),dict(joint='joint11')],self.group,**self.options)
        changed,meta=env['make_plan'](axis=axis,grasp_height_mm=h,grasp_depth_mm=d,last_aim_evidence=e)
        before,after=np.asarray(old['goal_pose']),np.asarray(meta['goal_pose'])
        np.testing.assert_array_equal(before[:3,:3],after[:3,:3])
        np.testing.assert_allclose(after[:3,3]-before[:3,3],[0.,0.,-.007],atol=1e-15)
        entry=next(row for row in changed if row[0]=='approach')
        np.testing.assert_allclose(kin.fk(entry[2][-1]),after,atol=1e-15)
        self.assertEqual(meta['last_remaining_fruit_aim'],e)
        np.testing.assert_array_equal(meta['target_center_world_m'],center)


if __name__=='__main__':unittest.main()
