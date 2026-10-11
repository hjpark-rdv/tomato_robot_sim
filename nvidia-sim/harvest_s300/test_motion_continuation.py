"""Later checkpoint precedence and measured-origin rail audit regressions."""
import ast
import copy
from contextlib import redirect_stderr
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from motion_continuation import (route_restore_selection, measured_resume_rail_origin,
    attach_resume_rail_hold, initial_plan_rail_position,resumed_rail_destination,attach_resumed_rail_travel)
from motion_self_collision import check_plan
from rail_motion import rail_command, station_kinematics
import test_motion_branch as branch_fixture
import test_rail_self_collision as rail_fixture


class MotionContinuationTests(unittest.TestCase):
    def execute_actual_initial_selection(self,context,temporary_output):
        """Run the real native runner's CPU-only initial selection block."""
        source=Path(__file__).with_name('run_motion.py');tree=ast.parse(source.read_text())
        def assigned(node,name):
            return isinstance(node,ast.Assign) and any(
                isinstance(item,ast.Name) and item.id==name
                for target in node.targets for item in ast.walk(target))
        containing=next(node.body for node in ast.walk(tree) if isinstance(node,ast.Try) and
            any(assigned(item,'initial_robot_q') for item in node.body))
        first=next(i for i,node in enumerate(containing) if assigned(node,'initial_robot_q'))
        last=next(i for i,node in enumerate(containing[first:],first) if assigned(node,'phases'))
        from resume_native_state import imported_break_markers
        manifest=context['metadata']['manifest']
        names=list(map(str,context['arrays']['robot_dof_names']))
        kin=SimpleNamespace(names=names,world=np.eye(4))
        args=SimpleNamespace(output=Path(temporary_output),resume_native_checkpoint=Path('/checked/checkpoint'),
            near_trusses_metadata=None,approach_yaw_mode='truss-axis',truss_axis_min_fruits=5,
            later_grasp_depth_mm=None,grasp_depth_mm=65.,split_unproductive_pairs=False,
            grasp_height_offset_mm=-10.,last_fruit_height_offset_mm=None,last_fruit_depth_mm=None,
            whole_truss=True,truss_order='highest-first',initial_truss_first=False,
            max_fruit_attempts=4,single_fruit_when_remaining=0,harvest_group_policy=None)
        env=dict(np=np,json=__import__('json'),manifest=manifest,resume_context=context,
            kin=kin,planning_kin=kin,a=args,imported_break_markers=imported_break_markers)
        from contextlib import redirect_stdout
        with redirect_stdout(io.StringIO()):
            exec(compile(ast.Module(body=containing[first:last],type_ignores=[]),str(source),'exec'),env)
        return env

    def initial_selection_context(self,kind,broken_indices=()):
        fruits=[dict(name=f'Truss_02_Fruit_{index:02d}',path=f'/active/fruit{index}',
            joint=f'/active/joint{index}',center=[2.+.02*index,3.,1.5-.02*index])for index in range(1,7)]
        manifest=dict(target=fruits[0],plant=dict(fruits=fruits),
            grasp_reference=dict(center=[2.,3.,1.5],height_offset_m=.01))
        measured=np.array([[.5+.02*index,1.7,1.2-.02*index,0.,0.,0.,1.]for index in range(1,7)])
        names=[f'arm{index}'for index in range(7)]
        sequence=dict(cycle_plans=[],attempt_counts={},fruit_joints={})
        if kind=='completed_cycle':
            sequence.update(initial_axis=dict(direction_world=[0.,1.,0.],yaw_world_deg=90.),
                attempt_counts={fruits[0]['name']:3})
        metadata=dict(manifest=manifest,sequence=sequence,metadata_sha256='checked-metadata',
            scene_sha256='checked-scene',time_s=54.,
            break_events=[dict(t=20.+index,joint=fruits[index]['joint'])for index in broken_indices])
        arrays=dict(robot_dof_names=np.array(names),robot_q=np.arange(7)*.01,
            body_paths=np.array([f['path']for f in fruits]),body_poses_world=measured)
        return dict(checkpoint_kind=kind,metadata=metadata,arrays=arrays)

    def test_actual_runner_completed_cycle_initial_selection_keeps_native_history_and_axis(self):
        from resume_axis import resolve_resume_axis
        context=self.initial_selection_context('completed_cycle',range(5));before=copy.deepcopy(context)
        with tempfile.TemporaryDirectory() as output,patch('resume_axis.resolve_resume_axis',wraps=resolve_resume_axis) as resolver:
            env=self.execute_actual_initial_selection(context,output)
            self.assertEqual(resolver.call_count,1)
            self.assertTrue((Path(output)/'resume_axis_provenance.json').is_file())
        group=env['initial_sequence_group'];fruit=context['metadata']['manifest']['plant']['fruits'][-1]
        self.assertEqual(group['fruits'],[fruit['name']])
        self.assertEqual(env['initial_axis']['direction_world'],[0.,1.,0.])
        self.assertEqual(env['initial_axis']['fallback_reason'],'fewer_than_two_attached_fruits')
        self.assertEqual(env['initial_attempt_counts'],before['metadata']['sequence']['attempt_counts'])
        self.assertTrue(all(marker['native_new_event'] is False for marker in env['initial_inventory_breaks']))
        np.testing.assert_array_equal(env['initial_robot_q'],before['arrays']['robot_q'])
        self.assertEqual(env['initial_height_mm'],-10.)
        self.assertEqual(env['initial_grasp_depth_mm'],65.)
        self.assertIsNone(env['initial_last_aim'])
        np.testing.assert_allclose(env['initial_translation'],
            context['arrays']['body_poses_world'][-1,:3]+[0.,0.,.01]-[2.,3.,1.5])
        np.testing.assert_array_equal(context['arrays']['body_poses_world'],before['arrays']['body_poses_world'])

    def test_actual_runner_canonical_idle_initial_selection_uses_measured_fruits_without_fake_cycles(self):
        context=self.initial_selection_context('canonical_conditioned_start');before=copy.deepcopy(context)
        with tempfile.TemporaryDirectory() as output,patch('resume_axis.resolve_resume_axis',side_effect=AssertionError('idle has no entered axis')) as resolver:
            env=self.execute_actual_initial_selection(context,output)
            self.assertEqual(resolver.call_count,0)
            self.assertFalse((Path(output)/'resume_axis_provenance.json').exists())
        fruits=context['metadata']['manifest']['plant']['fruits']
        self.assertEqual(env['initial_sequence_group']['fruits'],[f['name']for f in fruits[:2]])
        np.testing.assert_allclose(env['initial_axis']['direction_world'],[1.,0.,0.],atol=1e-12)
        self.assertIsNone(env['initial_axis']['fallback_reason'])
        self.assertEqual(env['initial_inventory_breaks'],[]);self.assertEqual(env['initial_attempt_counts'],{})
        self.assertEqual(context['metadata']['sequence']['cycle_plans'],[])
        self.assertEqual(env['initial_height_mm'],-10.)
        self.assertEqual(env['initial_grasp_depth_mm'],65.)
        self.assertIsNone(env['initial_last_aim'])
        centroid=context['arrays']['body_poses_world'][:2,:3].mean(axis=0)
        np.testing.assert_allclose(env['initial_translation'],centroid+[0.,0.,.01]-[2.,3.,1.5])
        np.testing.assert_array_equal(context['arrays']['body_poses_world'],before['arrays']['body_poses_world'])

    def fixture(self):
        manifest=dict(plant=dict(source_plant='Plant40'), rail=dict(
            axis_world=[0.,1.,0.],base_body='/World/HarvestRobot/Robot/world',
            source_base_position_world_m=[-.1,1.6,.2], initial_station_y_m=1.6,
            dof_name='root_joint',limits_relative_m=[-1.6,3.4]),
            route_checkpoint=dict(checkpoint='/original/plant22/cycle_checkpoints/000',
                checkpoint_time_s=54.,outgoing=dict(source_plant='Plant22')))
        state=dict(body_paths=np.array([manifest['rail']['base_body']]),
            body_poses_world=np.array([[-.1,2.1,.2,0.,0.,0.,1.]]),
            robot_dof_names=np.array(['root_joint','lift']),robot_q=np.array([.499999,.1]))
        context=dict(metadata=dict(manifest=copy.deepcopy(manifest),source_plant='Plant40',
                metadata_sha256='later-measured-state'), arrays=state,
            report=dict(checkpoint='/new/plant40/cycle_checkpoints/002',checkpoint_time_s=20.),
            controller_state=dict(position_targets=np.array([.55,.108]),
                provenance=dict(source='checkpoint_native_drive_getters')))
        return manifest,context

    def test_later_sameplant_restore_supersedes_archive_without_erasing_provenance(self):
        manifest,context=self.fixture();before=copy.deepcopy(manifest)
        result=route_restore_selection(manifest,context)
        self.assertFalse(result['prepare_route_start'])
        proof=result['superseded_archive']
        self.assertEqual(proof['archived_route_checkpoint'],manifest['route_checkpoint'])
        self.assertEqual(proof['selected_sameplant_checkpoint'],context['report'])
        self.assertFalse(proof['archived_restore_executed'])
        # The later native instance uses its own clock (20s versus old54s).
        self.assertEqual(manifest,before)
        wrong=copy.deepcopy(context);wrong['metadata']['manifest']['route_checkpoint']['checkpoint']='other'
        with self.assertRaises(ValueError):route_restore_selection(manifest,wrong)
        wrong=copy.deepcopy(context);wrong['metadata']['source_plant']='Plant22'
        with self.assertRaises(ValueError):route_restore_selection(manifest,wrong)

    def test_first_window_and_legacy_restore_selection_remain_explicit(self):
        manifest,_=self.fixture()
        self.assertTrue(route_restore_selection(manifest,None)['prepare_route_start'])
        for kwargs in (dict(num_envs=2),dict(replicate=True)):
            with self.assertRaises(ValueError):route_restore_selection(manifest,None,**kwargs)
        self.assertEqual(route_restore_selection({},None,num_envs=32),
                         dict(prepare_route_start=False,superseded_archive=None))

    def test_measured_geometry_and_drive_target_are_distinct_and_source_fk_is_unchanged(self):
        manifest,context=self.fixture();origin=measured_resume_rail_origin(manifest,context)
        self.assertAlmostEqual(origin['displacement_m'],.5)
        self.assertEqual(origin['native_position_target_m'],.55)
        self.assertEqual(origin['native_measured_rail_q_m'],.499999)
        self.assertAlmostEqual(origin['station_y_m'],2.1)
        kin=SimpleNamespace(world=np.eye(4));translated=station_kinematics(kin,origin)
        np.testing.assert_array_equal(kin.world,np.eye(4))
        np.testing.assert_allclose(translated.world[:3,3],[0.,.5,0.])
        self.assertIsNone(measured_resume_rail_origin(manifest,None))
        self.assertIsNone(measured_resume_rail_origin({},context))

    def test_changed_or_incomplete_rail_identity_never_guesses_a_station(self):
        manifest,context=self.fixture()
        variants=[]
        missing=copy.deepcopy(context);missing['controller_state']['position_targets']=None;variants.append(missing)
        missing=copy.deepcopy(context);missing['arrays']['body_paths']=np.array(['/another/base']);variants.append(missing)
        missing=copy.deepcopy(context);missing['arrays']['body_poses_world'][0,1]=np.nan;variants.append(missing)
        missing=copy.deepcopy(context);missing['metadata']['manifest']['rail']['dof_name']='other';variants.append(missing)
        for bad in variants:
            with self.assertRaises(ValueError):measured_resume_rail_origin(manifest,bad)

    def test_actual_native_fk_gate_uses_measured_start_station_not_future_destination(self):
        source=Path(__file__).with_name('run_motion.py');tree=ast.parse(source.read_text())
        names={'initial_fk_kin','actual','expected','fk_error'}
        body=next(node.body for node in ast.walk(tree) if isinstance(node,ast.Try) and any(
            isinstance(item,ast.Assign) and any(isinstance(target,ast.Name) and target.id=='initial_fk_kin'
                for target in item.targets)for item in node.body))
        start=next(index for index,node in enumerate(body)if isinstance(node,ast.Assign)and any(
            isinstance(target,ast.Name)and target.id=='initial_fk_kin'for target in node.targets))
        end=next(index for index,node in enumerate(body[start:],start)if isinstance(node,ast.If)and
            isinstance(node.test,ast.Compare)and isinstance(node.test.left,ast.Name)and node.test.left.id=='fk_error')
        code=compile(ast.Module(body=body[start:end+1],type_ignores=[]),str(source),'exec')
        class Robot:
            def __init__(self):self.world=np.eye(4)
            def fk(self,q):
                result=self.world.copy();result[:3,3]+=np.asarray(q[:3]);return result
        kin=Robot();q=np.array([[.1,.2,.3,0.,0.,0.,0.]])
        manifest,context=self.fixture();origin=measured_resume_rail_origin(manifest,context)
        future=station_kinematics(kin,dict(axis_world=[0.,1.,0.],displacement_m=1.5))
        for selected in (None,origin):
            expected=(kin if selected is None else station_kinematics(kin,selected)).fk(q[0])
            initial=np.array([np.r_[expected[:3,3],0.,0.,0.,1.]])
            env=dict(np=np,kin=kin,planning_kin=future,resume_rail_origin=selected,
                station_kinematics=station_kinematics,q0=q,rids=list(range(7)),initial=initial,toolid=0)
            exec(code,env)
            self.assertEqual(env['fk_error'],0.)
            np.testing.assert_array_equal(env['expected'],expected)
            self.assertGreater(np.linalg.norm(env['expected'][:3,3]-future.fk(q[0])[:3,3]),.9)
            env['initial']=initial.copy();env['initial'][0,1]+=.002
            with self.assertRaisesRegex(RuntimeError,'USD FK/native pose mismatch'):exec(code,env)
        np.testing.assert_array_equal(kin.world,np.eye(4))

    def test_checker_uses_measured_station_at_every_tick_but_native_holds_prior_setpoint(self):
        manifest,context=self.fixture();origin=measured_resume_rail_origin(manifest,context)
        q=np.zeros((2,7));phases=[('approach',1.,q,.044,0.,0.)]
        metadata=dict(phases=[dict(name='approach',duration_s=1.,waypoint_q=q.tolist(),end_gap_m=.044)])
        actual,plan=attach_resume_rail_hold(phases,metadata,origin)
        self.assertIs(actual,phases)
        self.assertNotIn('rail_waypoint_positions_m',metadata['phases'][0])
        self.assertEqual([p[0] for p in actual],['approach'])
        class Checker:
            metadata={}
            def __init__(self):self.offsets=[]
            def check(self,q,gap,*,rail_position_m):self.offsets.append(rail_position_m);return []
        checker=Checker();report=check_plan(checker,plan,hz=4)
        self.assertTrue(report['complete']);self.assertEqual(report['samples'],5)
        np.testing.assert_allclose(checker.offsets,[.5]*5)
        for fraction in (0.,.3,1.):self.assertEqual(rail_command(plan['phases'][0],fraction),.55)
        bad=copy.deepcopy(plan);bad['phases'][0]['rail_waypoint_positions_m'][0]=0.
        with self.assertRaises(ValueError):check_plan(checker,bad,hz=4)

    def test_full_chassis_arm_pose_query_moves_once_at_the_measured_origin(self):
        manifest,context=self.fixture();origin=measured_resume_rail_origin(manifest,context)
        q=np.zeros((2,7));phases=[('hold',1.,q,.044,0.,0.)]
        _,plan=attach_resume_rail_hold(phases,dict(phases=[dict(name='hold',duration_s=1.,
            waypoint_q=q.tolist(),end_gap_m=.044)]),origin)
        checker=rail_fixture.RailSelfCollisionTests().checker()
        before=checker.poses(q[0]);after=checker.poses(q[0],rail_position_m=origin['displacement_m'])
        for body in before:np.testing.assert_allclose(after[body][:3,3]-before[body][:3,3],[0.,.5,0.])
        result=check_plan(checker,plan,hz=4)
        self.assertTrue(result['complete']);self.assertTrue(result['passed'])

    def test_default_native_command_key_and_initial_origin_are_unchanged(self):
        row=dict(rail_waypoint_positions_m=[0.,.4])
        self.assertEqual(rail_command(row,.5),.2)
        self.assertEqual(initial_plan_rail_position(dict(phases=[])),0.)
        for bad in ({},dict(schema='wrong',axis_world=[0.,1.,0.],displacement_m=.5),
                    dict(schema='s300-measured-resume-rail-origin-v1',axis_world=[0.,1.,0.],displacement_m=np.nan)):
            with self.assertRaises(ValueError):initial_plan_rail_position(dict(resume_rail_origin=bad))

    def test_resumed_travel_duration_bounds_both_controller_and_measured_motion(self):
        manifest,context=self.fixture();origin=measured_resume_rail_origin(manifest,context)
        destination=resumed_rail_destination(manifest,2.075,.08,origin)
        self.assertAlmostEqual(destination['displacement_m'],.475)
        self.assertAlmostEqual(destination['duration_s'],1.40625)
        for initial in (.5,.55):
            self.assertLessEqual(1.5*abs(initial-.475)/destination['duration_s'],.08+1e-12)
        self.assertEqual(destination['source_checkpoint_metadata_sha256'],'later-measured-state')
        np.testing.assert_array_equal(context['arrays']['robot_q'],[.499999,.1])
        with self.assertRaises(ValueError):resumed_rail_destination(manifest,6.,.08,origin)

    def test_resumed_travel_starts_at_old_station_and_hold_command_then_keeps_destination(self):
        manifest,context=self.fixture();origin=measured_resume_rail_origin(manifest,context)
        destination=resumed_rail_destination(manifest,2.075,.08,origin)
        q=np.zeros((2,7));phases=[('approach',1.,q,.044,0.,0.)]
        metadata=dict(phases=[dict(name='approach',duration_s=1.,waypoint_q=q.tolist(),end_gap_m=.044)])
        traveled,plan=attach_resumed_rail_travel(phases,metadata,q[0],destination,origin,move_first=True)
        self.assertEqual([p[0] for p in traveled],['rail_move','approach'])
        np.testing.assert_array_equal(traveled[0][2],q)
        self.assertAlmostEqual(plan['phases'][0]['rail_waypoint_positions_m'][0],.5)
        self.assertEqual(rail_command(plan['phases'][0],0.),.55)
        self.assertAlmostEqual(rail_command(plan['phases'][0],1.),.475)
        class Checker:
            metadata={}
            def __init__(self):self.offsets=[]
            def check(self,q,gap,*,rail_position_m):self.offsets.append(rail_position_m);return []
        checker=Checker();report=check_plan(checker,plan,hz=4)
        self.assertTrue(report['complete']);self.assertAlmostEqual(checker.offsets[0],.5)
        self.assertAlmostEqual(checker.offsets[-1],.475)
        self.assertNotIn('resume_rail_travel',metadata)
        again,later=attach_resumed_rail_travel(phases,metadata,q[0],destination,origin,move_first=False)
        self.assertEqual([p[0] for p in again],['approach'])
        self.assertAlmostEqual(initial_plan_rail_position(later),.475)
        checker=Checker();self.assertTrue(check_plan(checker,later,hz=4)['complete'])
        np.testing.assert_allclose(checker.offsets,[.475]*5)

    def test_later_existing_rail_cycle_starts_at_destination_without_retravel(self):
        from rail_motion import rail_destination,attach_rail_commands
        manifest,_=self.fixture();q=np.zeros((2,7))
        phases=[('hold',1.,q,.044,0.,0.)]
        metadata=dict(phases=[dict(name='hold',duration_s=1.,waypoint_q=q.tolist(),end_gap_m=.044)])
        _,plan=attach_rail_commands(phases,metadata,q[0],rail_destination(manifest,2.1),move_first=False)
        checker=rail_fixture.RailSelfCollisionTests().checker()
        self.assertTrue(check_plan(checker,plan,hz=4)['complete'])
        self.assertAlmostEqual(initial_plan_rail_position(plan),.5)

    def test_resumed_rail_fcl_does_not_allow_unexplained_offset_or_cycle_discontinuity(self):
        manifest,context=self.fixture();origin=measured_resume_rail_origin(manifest,context)
        destination=resumed_rail_destination(manifest,2.075,.08,origin)
        q=np.zeros((2,7));phases=[('hold',1.,q,.044,0.,0.)]
        _,plan=attach_resumed_rail_travel(phases,dict(phases=[dict(name='hold',duration_s=1.,
            waypoint_q=q.tolist(),end_gap_m=.044)]),q[0],destination,origin,move_first=True)
        checker=rail_fixture.RailSelfCollisionTests().checker()
        wrong=copy.deepcopy(plan);wrong['phases'][1]['rail_waypoint_positions_m'][0]=.4
        with self.assertRaises(ValueError):check_plan(checker,wrong,hz=4)
        wrong=copy.deepcopy(plan);wrong['resume_rail_travel']['saved_measured_start_displacement_m']=0.
        with self.assertRaises(ValueError):check_plan(checker,wrong,hz=4)

    def test_real_cli_resumed_travel_requires_explicit_optin_and_both_inputs(self):
        source=Path(__file__).with_name('run_motion.py');tree=ast.parse(source.read_text());prefix=[]
        for node in tree.body:
            if isinstance(node,ast.Expr) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute) and node.value.func.attr=='mkdir':break
            prefix.append(node)
        code=compile(ast.Module(body=prefix,type_ignores=[]),str(source),'exec')
        base=[str(source),'/unused','--output','/unused-output','--whole-truss',
            '--preparation','horizontal_ready','--harvest-profile','belt_open_close_cycle']
        for bad in (['--resume-native-checkpoint','/checkpoint','--rail-to-y','1.572'],
                    ['--allow-resume-rail-travel'],
                    ['--allow-resume-rail-travel','--resume-native-checkpoint','/checkpoint'],
                    ['--allow-resume-rail-travel','--rail-to-y','1.572']):
            with patch.object(sys,'argv',base+bad),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):exec(code,{})
        with patch.object(sys,'argv',base+['--allow-resume-rail-travel',
                '--resume-native-checkpoint','/checkpoint','--rail-to-y','1.572']):
            env={};exec(code,env);self.assertTrue(env['a'].allow_resume_rail_travel)
        with patch.object(sys,'argv',base):
            env={};exec(code,env);self.assertFalse(env['a'].allow_resume_rail_travel)

    def test_actual_native_runner_make_plan_preserves_joint_commands_and_adds_only_rail_hold(self):
        source=Path(__file__).with_name('run_motion.py');tree=ast.parse(source.read_text());prefix=[]
        for node in tree.body:
            if isinstance(node,ast.Expr) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute) and node.value.func.attr=='mkdir':break
            prefix.append(node)
        env={}
        with patch.object(sys,'argv',[str(source),'/unused','--output','/unused-output',
                '--preparation','horizontal_ready','--preparation-height-first','--harvest-profile','belt_open_close_cycle']):
            exec(compile(ast.Module(body=prefix,type_ignores=[]),str(source),'exec'),env)
        from motion_kinematics import plan_motion,grasp_pose
        kin=branch_fixture.RedundantCartesianRobot();center=np.array([.4,1.,1.2])
        ready=grasp_pose(center,center.copy());ready[:3,3]-=ready[:3,1]*.3
        start=kin.ik_at_lift(ready,np.zeros(7),.12)[0]
        env.update(plan_motion=plan_motion,planning_kin=kin,reference_center=center,initial_robot_q=start,
            recorded_prefix=None,initial_grasp_depth_mm=65.,initial_depth_evidence=None,
            rail_destination=None,resume_rail_origin=None,attach_resume_rail_hold=attach_resume_rail_hold)
        function=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='make_plan')
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),env)
        axis=dict(direction_world=ready[:3,1].tolist())
        legacy,old=env['make_plan'](axis=axis)
        manifest,context=self.fixture();env['resume_rail_origin']=measured_resume_rail_origin(manifest,context)
        continued,new=env['make_plan'](axis=axis)
        self.assertEqual([p[0] for p in continued],[p[0] for p in legacy])
        for current,original,row in zip(continued,legacy,new['phases']):
            np.testing.assert_array_equal(current[2],original[2])
            self.assertEqual(current[:2],original[:2]);self.assertEqual(current[3:],original[3:])
            self.assertTrue(all(abs(value-.5)<1e-12 for value in row['rail_waypoint_positions_m']))
            self.assertTrue(all(value==.55 for value in row['rail_native_command_positions_m']))
        self.assertNotIn('resume_rail_origin',old)


if __name__=='__main__':unittest.main()
