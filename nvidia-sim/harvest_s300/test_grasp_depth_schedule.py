import ast
from contextlib import redirect_stderr
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

from grasp_depth_schedule import (completed_native_cycles,grasp_depth_for_cycle,grasp_depth_for_truss_cycle,
    grasp_depth_for_selected_group,configure_group_selector_depth)
from motion_kinematics import plan_motion,grasp_pose
from test_motion_branch import RedundantCartesianRobot


class GraspDepthScheduleTests(unittest.TestCase):
    def completed(self):
        return dict(status='motion_completed',start_s=2.,end_s=54.,planned_end_s=54.,
                    measured_frame_index=0,end_frame_index=6480)

    def test_only_completed_native_timeline_activates_fixed_absolute_override(self):
        complete=self.completed()
        records=[dict(complete,status='planning_rejected'),dict(complete,status='stopped'),
                 dict(complete,stop_reason='penetration_over_0_5mm'),dict(complete,end_s=53.),
                 dict(complete,end_frame_index=0),dict(complete,end_s=float('nan'))]
        self.assertEqual(completed_native_cycles(records),0)
        self.assertEqual(grasp_depth_for_cycle(65.,50.,0)[0],65.)
        records.append(complete)
        self.assertEqual(completed_native_cycles(records),1)
        self.assertEqual(grasp_depth_for_cycle(65.,50.,1)[0],50.)
        self.assertEqual(grasp_depth_for_cycle(65.,50.,9)[0],50.)
        self.assertFalse(grasp_depth_for_cycle(65.,50.,9)[1]['cumulative_translation'])

    def test_unspecified_override_preserves_global_depth_even_for_resumes(self):
        for count in (0,1,9):
            self.assertEqual(grasp_depth_for_cycle(65.,None,count)[0],65.)
            self.assertEqual(grasp_depth_for_cycle(50.,None,count)[0],50.)
        value,evidence=grasp_depth_for_cycle(65.,50.,completed_native_cycles([self.completed()]))
        self.assertEqual(value,50.);self.assertEqual(evidence['completed_native_cycles'],1)

    def test_camera_scan_and_nonharvest_timelines_do_not_consume_physical_cycle_budget(self):
        scan=dict(self.completed(),purpose='camera_scan_prefix',harvest_attempt=False,fruits=[])
        rejected=dict(self.completed(),status='planning_rejected')
        self.assertEqual(completed_native_cycles([scan,rejected]),0)
        self.assertEqual(completed_native_cycles([scan,rejected,self.completed()]),1)

    def test_invalid_depth_or_noninteger_completion_is_rejected(self):
        for bad in (None,True,14.,101.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):grasp_depth_for_cycle(bad,None,0)
        for bad in (True,14.,101.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):grasp_depth_for_cycle(65.,bad,0)
        for count in (-1,True,1.5):
            with self.assertRaises(ValueError):grasp_depth_for_cycle(65.,50.,count)

    def test_new_truss_first_cycle_preserves_base_depth_despite_completed_other_truss(self):
        completed=[dict(self.completed(),fruits=['Truss_02_Fruit_01','Truss_02_Fruit_02'])]
        # Legacy unscoped behavior remains exactly the same.
        self.assertEqual(completed_native_cycles(completed),1)
        self.assertEqual(grasp_depth_for_cycle(65.,50.,completed_native_cycles(completed))[0],50.)
        self.assertEqual(completed_native_cycles(completed,truss_key='Truss_01'),0)
        depth,evidence=grasp_depth_for_truss_cycle(65.,50.,completed,'Truss_01')
        self.assertEqual(depth,65.);self.assertEqual(evidence['selected_truss_key'],'Truss_01')
        completed.append(dict(self.completed(),fruits=['Truss_01_Fruit_01']))
        self.assertEqual(grasp_depth_for_truss_cycle(65.,50.,completed,'Truss_01')[0],50.)
        self.assertEqual(completed_native_cycles(completed,truss_key='Truss_02'),1)

    def test_scoped_depth_ignores_unknown_mixed_or_rejected_physical_records(self):
        completed=[dict(self.completed(),fruits=['Truss_01_Fruit_01','Truss_02_Fruit_01']),
            dict(self.completed(),fruits=['Truss_01_Fruit_01'],truss_key='Truss_02'),
            dict(self.completed(),fruits=['Unknown']),self.completed(),
            dict(self.completed(),fruits=['Truss_01_Fruit_01'],status='planning_rejected')]
        self.assertEqual(completed_native_cycles(completed),4)
        self.assertEqual(completed_native_cycles(completed,truss_key='Truss_01'),0)
        for bad in ('Truss_01_Fruit_01','Truss_0bad',True):
            with self.assertRaises(ValueError):completed_native_cycles(completed,truss_key=bad)

    def test_trial_scope_uses_first_completed_cycle_for_new_truss_and_checkpoint_history(self):
        parent=[dict(self.completed(),fruits=['Truss_02_Fruit_01','Truss_02_Fruit_02'])]
        new=[dict(self.completed(),purpose='camera_scan_prefix',harvest_attempt=False,fruits=[]),
             dict(self.completed(),status='planning_rejected',fruits=['Truss_00_Fruit_01'])]
        records=parent+new
        self.assertEqual(grasp_depth_for_selected_group(35,15,records,'Truss_00',
            near_trusses=True)[0],35)
        depth,proof=grasp_depth_for_selected_group(35,15,records,'Truss_00',
            scope='trial',near_trusses=True)
        self.assertEqual(depth,15);self.assertEqual(proof['global_completed_native_cycles'],1)
        self.assertEqual(proof['effective_depth_scope'],'trial')
        self.assertEqual(grasp_depth_for_selected_group(35,15,new,'Truss_00',
            scope='trial',near_trusses=True)[0],35)

    def test_selector_footprint_and_actual_second_truss_plan_use_the_same_depth(self):
        from harvest_group_policy import HarvestGroupSelector
        fruits=[dict(name='Truss_'+f'{key:02d}'+'_Fruit_'+f'{i:02d}',
            path='/fruit/'+str(key)+'/'+str(i),joint='/joint/'+str(key)+'/'+str(i),dimensions_m=[.023]*3)
            for key in [0,2]for i in [1,2]]
        positions=np.array([[0,0,1.08],[.01,0,1.06],[0,0,1.02],[.01,0,1.00]])
        policy=dict(minimum_fruits=2,fruit_eligibility=[dict(**f,eligible=True)for f in fruits])
        paths=[f['path']for f in fruits]
        records=[dict(self.completed(),fruits=['Truss_02_Fruit_01','Truss_02_Fruit_02'])]
        for scope,wanted in [('truss',35),('trial',15)]:
            selector=HarvestGroupSelector(policy,lambda *args:[0,1,0],height_offset_m=.0075)
            configure_group_selector_depth(selector,35,15,records,scope=scope,near_trusses=True)
            selected=selector(fruits,[],paths,positions,{},order='highest-first')
            self.assertEqual(selected['truss_key'],'Truss_00')
            self.assertEqual(selected['group_selection']['depth_m'],wanted/1000)
            actual,proof=grasp_depth_for_selected_group(35,15,records,selected['truss_key'],
                scope=scope,near_trusses=True)
            self.assertEqual(actual,wanted)
            if scope=='truss':self.assertEqual(selector.depth_for_truss('Truss_02'),.015)
            # Exercise the ACTUAL runner's second-group branch, not a mirror.
            from types import SimpleNamespace
            _,tree,_=self.real_parser()
            node=next(n for n in ast.walk(tree)if isinstance(n,ast.Assign)and
                any(isinstance(t,ast.Tuple)and any(isinstance(x,ast.Name)and x.id=='effective_depth_mm'for x in t.elts)for t in n.targets)
                and isinstance(n.value,ast.Call)and isinstance(n.value.func,ast.Name)and n.value.func.id=='grasp_depth_for_selected_group')
            env=dict(a=SimpleNamespace(grasp_depth_mm=35,later_grasp_depth_mm=15,later_grasp_depth_scope=scope),
                previous_cycle_records=records,sequence=dict(cycle_plans=[]),group=selected,
                grasp_depth_for_selected_group=grasp_depth_for_selected_group)
            exec(compile(ast.Module(body=[node],type_ignores=[]),'actual_depth_branch','exec'),env)
            self.assertEqual(env['effective_depth_mm']/1000,selected['group_selection']['depth_m'])

    def test_real_cli_trial_scope_is_explicit_and_requires_later_depth(self):
        source,_,code=self.real_parser();base=[str(source),'/unused','--output','/unused-output',
            '--whole-truss','--preparation','horizontal_ready','--harvest-profile','belt_open_close_cycle']
        with patch.object(sys,'argv',base):
            env={};exec(code,env);self.assertEqual(env['a'].later_grasp_depth_scope,'truss')
        with patch.object(sys,'argv',base+['--later-grasp-depth-mm','15','--later-grasp-depth-scope','trial']):
            env={};exec(code,env);self.assertEqual(env['a'].later_grasp_depth_scope,'trial')
        with patch.object(sys,'argv',base+['--later-grasp-depth-scope','trial']),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            exec(code,{})


    def real_parser(self):
        source=Path(__file__).with_name('run_motion.py');tree=ast.parse(source.read_text());prefix=[]
        for node in tree.body:
            if isinstance(node,ast.Expr) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute) and node.value.func.attr=='mkdir':break
            prefix.append(node)
        return source,tree,compile(ast.Module(body=prefix,type_ignores=[]),str(source),'exec')

    def test_real_cli_defaults_scope_and_finite_existing_depth_range(self):
        source,tree,code=self.real_parser()
        argv=[str(source),'/unused','--output','/unused-output']
        with patch.object(sys,'argv',argv):
            env={};exec(code,env);self.assertIsNone(env['a'].later_grasp_depth_mm)
            self.assertEqual(env['completed_native_cycles']([
                dict(self.completed(),purpose='camera_scan_prefix',harvest_attempt=False)]),0)
            self.assertEqual(env['a'].camera_max_harvest_cycles,0)
        scope=['--whole-truss','--preparation','horizontal_ready','--harvest-profile','belt_open_close_cycle']
        for value in ('15','50','100'):
            with patch.object(sys,'argv',argv+scope+['--later-grasp-depth-mm',value]):
                env={};exec(code,env);self.assertEqual(env['a'].later_grasp_depth_mm,float(value))
        for tail in (['--later-grasp-depth-mm','50'],*[scope+['--later-grasp-depth-mm',v] for v in ('14','101','nan','inf')]):
            with patch.object(sys,'argv',argv+tail),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):exec(code,{})

    def test_actual_runner_passes_effective_depth_and_preserves_default_commands(self):
        source,tree,code=self.real_parser();env={}
        with patch.object(sys,'argv',[str(source),'/unused','--output','/unused-output',
            '--preparation','horizontal_ready','--preparation-height-first','--harvest-profile','belt_open_close_cycle']):exec(code,env)
        function=next(n for n in ast.walk(tree)if isinstance(n,ast.FunctionDef)and n.name=='make_plan')
        kin=RedundantCartesianRobot();center=np.array([.4,1.,1.2]);ready=grasp_pose(center,center.copy());ready[:3,3]-=ready[:3,1]*.3
        start=kin.ik_at_lift(ready,np.zeros(7),.12)[0]
        env.update(plan_motion=plan_motion,planning_kin=kin,reference_center=center,
                   initial_robot_q=start,recorded_prefix=None,initial_grasp_depth_mm=65.,
                   initial_depth_evidence=None,rail_destination=None,resume_rail_origin=None)
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),env)
        axis={'direction_world':ready[:3,1].tolist()}
        old,oldmeta=plan_motion(kin,center,start,preparation='horizontal_ready',
            harvest_profile='belt_open_close_cycle',preparation_height_first=True,
            approach_direction_world=ready[:3,1])
        actual,meta=env['make_plan'](axis=axis)
        self.assertEqual(meta['phases'],oldmeta['phases']);self.assertNotIn('grasp_depth_schedule',meta)
        for a,b in zip(actual,old):np.testing.assert_array_equal(a[2],b[2])
        env['a'].later_grasp_depth_mm=50.;effective,evidence=grasp_depth_for_cycle(65.,50.,1)
        later,metadata=env['make_plan'](axis=axis,grasp_depth_mm=effective,depth_evidence=evidence)
        self.assertEqual(metadata['grasp_depth_m'],.05);self.assertEqual(metadata['grasp_depth_schedule'],evidence)
        nominal_pose=np.asarray(oldmeta['goal_pose']);new_pose=np.asarray(metadata['goal_pose'])
        np.testing.assert_array_equal(new_pose[:3,:3],nominal_pose[:3,:3])
        np.testing.assert_allclose(new_pose[:3,3]-nominal_pose[:3,3],-.015*nominal_pose[:3,1],atol=1e-12)
        # The actual runner fingerprint stores the depth used, rather than65.
        source_text=source.read_text()
        self.assertIn('fingerprint_parameters.update(grasp_depth_mm=effective_depth_mm,',source_text)


if __name__=='__main__':unittest.main()
