"""Execute the actual native runner's camera-boundary rail wiring without Kit."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from test_run_motion_conditioning import TREE,MAIN,code
from test_native_route_checkpoint import NativeRouteCheckpointTests
from motion_continuation import (attach_resume_rail_hold,attach_resumed_rail_travel,
    initial_plan_rail_position)
from rail_motion import attach_rail_commands,rail_command
from rail_environment_preflight import verify_initial_alignment

HELPER=next(n for n in TREE.body if isinstance(n,ast.FunctionDef)and n.name=='camera_boundary_rail_state')
MAKE=next(n for n in ast.walk(MAIN)if isinstance(n,ast.FunctionDef)and n.name=='make_plan')
RAIL=next(n for n in ast.walk(MAKE)if isinstance(n,ast.If)and ast.unparse(n.test)=='camera_rail_origin is not None')
BOUNDARY=next(n for n in ast.walk(MAIN)if isinstance(n,ast.If)and
    any(isinstance(stmt,ast.Assign)and isinstance(stmt.value,ast.Call)and
        isinstance(stmt.value.func,ast.Name)and stmt.value.func.id=='camera_boundary_rail_state'
        for stmt in n.body))


class CameraBoundaryRailTests(unittest.TestCase):
    def fixture(self):
        source=NativeRouteCheckpointTests();source.setUp();self.addCleanup(source.tearDown)
        manifest=copy.deepcopy(source.manifest)
        manifest['rail']=dict(dof_name='root_joint',axis_world=[0.,1.,0.],
            base_body='/World/HarvestRobot/Robot/world',source_base_position_world_m=[2.,5.,4.],
            initial_station_y_m=5.,limits_relative_m=[-5.,0.])
        poses=source.args['body_poses_world'].copy();poses[2,:3]=[2.,3.000101376,4.]
        state=dict(robot_dof_names=['root_joint','lift','jaw_L','jaw_R'],
            robot_q=[-1.99989874,.4,-.026,.026],robot_qvel=[.00001,.02,0.,0.],
            robot_position_targets=[-2.,.408,-.026,.026],robot_velocity_targets=[0.,0.,0.,0.])
        sequence=dict(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True,
            cycle_plans=[dict(status='motion_completed',end_s=2.,stop_reason=None)])
        source.save(manifest=manifest,body_poses_world=poses,sequence=sequence,**state)
        world=np.eye(4);world[:3,3]=manifest['rail']['source_base_position_world_m']
        kin=SimpleNamespace(world=world)
        context=dict(Path=Path);exec(code([HELPER]),context)
        return source,manifest,kin,context['camera_boundary_rail_state']

    def test_actual_helper_owns_measured_geometry_and_preserves_distinct_native_target(self):
        source,manifest,kin,helper=self.fixture();before=kin.world.copy()
        files=[source.root/'checkpoint'/n for n in('checkpoint.json','checkpoint.npz')]
        old=[p.read_bytes()for p in files]
        planning,origin=helper(source.assembly,manifest,source.root/'checkpoint',kin)
        self.assertAlmostEqual(planning.world[1,3],3.000101376)
        self.assertAlmostEqual(origin['displacement_m'],-1.999898624)
        self.assertEqual(origin['native_position_target_m'],-2.)
        self.assertNotEqual(origin['native_position_target_m'],origin['native_measured_rail_q_m'])
        self.assertTrue(origin['controller_target_is_not_measured_displacement'])
        np.testing.assert_array_equal(kin.world,before)
        self.assertEqual(old,[p.read_bytes()for p in files])

    def test_actual_boundary_updates_kin_from_original_source_once_not_last_destination(self):
        source,manifest,kin,helper=self.fixture();initial=np.array([[.4,0.,0.]])
        env=dict(a=SimpleNamespace(camera_harvest=True,assembly=source.assembly),manifest=manifest,
            boundary_checkpoint=source.root/'checkpoint',kin=kin,q=initial,rids=[0,1,2],
            camera_boundary_rail_state=helper,sequence={},current_time=45.9666666667)
        exec(code([BOUNDARY]),env)
        first=env['planning_kin'].world.copy()
        exec(code([BOUNDARY]),env)
        np.testing.assert_array_equal(env['planning_kin'].world,first)
        np.testing.assert_array_equal(env['initial_robot_q'],initial[0])
        self.assertEqual(env['camera_suffix_rail_origin']['native_position_target_m'],-2.)
        self.assertEqual(len(env['sequence']['camera_boundary_rail_origins']),2)
        calls=[n for n in ast.walk(MAIN)if isinstance(n,ast.Call)and isinstance(n.func,ast.Name)and n.func.id=='make_plan'and
               any(k.arg=='camera_rail_origin'for k in n.keywords)]
        self.assertEqual(len(calls),1)
        self.assertEqual(ast.unparse(next(k.value for k in calls[0].keywords if k.arg=='camera_rail_origin')),'camera_suffix_rail_origin')

    def execute_rail_branch(self,origin,*,destination):
        q=np.zeros(7);phases=[('ready_hold',.5,np.array([q,q]),.096,0.,0.)]
        metadata=dict(phases=[dict(name='ready_hold',duration_s=.5,waypoint_q=[q.tolist(),q.tolist()],end_gap_m=.096,belt_local_y_m_s=[0.,0.])])
        env=dict(result=phases,metadata=metadata,camera_rail_origin=origin,rail_destination=destination,
            resume_rail_origin=None,initial_robot_q=q,move_rail_first=False,
            attach_resume_rail_hold=attach_resume_rail_hold,attach_resumed_rail_travel=attach_resumed_rail_travel,
            attach_rail_commands=attach_rail_commands)
        exec(code([RAIL]),env)
        return env

    def test_actual_suffix_metadata_and_executor_hold_native_command_with_or_without_prior_travel(self):
        source,manifest,kin,helper=self.fixture();_,origin=helper(source.assembly,manifest,source.root/'checkpoint',kin)
        for destination in(None,dict(displacement_m=-2.,axis_world=[0.,1.,0.],dof_name='root_joint')):
            with self.subTest(prior_travel=destination is not None):
                env=self.execute_rail_branch(origin,destination=destination);metadata=env['metadata']
                self.assertAlmostEqual(initial_plan_rail_position(metadata),origin['displacement_m'])
                for u in(0.,.4,1.):self.assertEqual(rail_command(metadata['phases'][0],u),-2.)
                self.assertEqual([p[0]for p in env['result']],['ready_hold'])
                self.assertNotIn('rail_destination',metadata)
                if destination is not None:self.assertEqual(metadata['completed_rail_destination'],destination)
                else:self.assertNotIn('completed_rail_destination',metadata)

    def test_same_unmodified_point_one_mm_consistency_gate_rejects_nominal_and_accepts_measured(self):
        source,manifest,kin,helper=self.fixture();_,origin=helper(source.assembly,manifest,source.root/'checkpoint',kin)
        base=manifest['rail']['base_body'];rest=np.eye(4);rest[:3,3]=manifest['rail']['source_base_position_world_m']
        measured=np.eye(4);measured[:3,3]=[2.,3.000101376,4.]
        def poses(q,gap,*,rail_position_m):
            result=rest.copy();result[1,3]+=rail_position_m;return{base:result}
        checker=SimpleNamespace(poses=poses,bodies=[base],shapes=[],rest={base:rest})
        nominal=dict(phases=[dict(name='ready_hold',waypoint_q=[[0.]*7,[0.]*7])],rail_destination=dict(displacement_m=-2.))
        with self.assertRaisesRegex(ValueError,'measured/FK'):verify_initial_alignment(checker,nominal,manifest,{base:measured})
        plan=self.execute_rail_branch(origin,destination=nominal['rail_destination'])['metadata']
        result=verify_initial_alignment(checker,plan,manifest,{base:measured})
        self.assertEqual(result['numerical_fk_position_tolerance_m'],.0001)
        self.assertEqual(result['measured_position_errors_m'][base],0.)

    def test_legacy_branch_is_unchanged_and_exact_model_mismatch_still_rejects(self):
        destination=dict(displacement_m=-2.,axis_world=[0.,1.,0.],dof_name='root_joint')
        env=self.execute_rail_branch(None,destination=destination)
        q=np.zeros(7);expected=attach_rail_commands([('ready_hold',.5,np.array([q,q]),.096,0.,0.)],
            dict(phases=[dict(name='ready_hold',duration_s=.5,waypoint_q=[q.tolist(),q.tolist()],end_gap_m=.096,belt_local_y_m_s=[0.,0.])]),q,destination,move_first=False)
        self.assertEqual(env['metadata'],expected[1]);self.assertEqual(rail_command(env['metadata']['phases'][0],.5),-2.)
        source,manifest,kin,helper=self.fixture();wrong=copy.deepcopy(manifest);wrong['target']['other']='changed'
        with self.assertRaisesRegex(ValueError,'identity mismatch'):helper(source.assembly,wrong,source.root/'checkpoint',kin)
        with self.assertRaisesRegex(ValueError,'current boundary'):helper(source.assembly,manifest,None,kin)
        result,origin=helper(source.assembly,dict(plant={}),None,kin)
        self.assertIs(result,kin);self.assertIsNone(origin)

    def test_stopped_invalid_or_incomplete_boundaries_cannot_supply_a_new_origin(self):
        source,manifest,kin,helper=self.fixture()
        from native_route_checkpoint import load_checkpoint
        metadata,arrays=load_checkpoint(source.root/'checkpoint')
        base=dict(manifest=manifest,body_poses_world=arrays['body_poses_world'],
            robot_dof_names=arrays['robot_dof_names'],robot_q=arrays['robot_q'],robot_qvel=arrays['robot_qvel'],
            robot_position_targets=metadata['controller_state']['position_targets'],
            robot_velocity_targets=metadata['controller_state']['velocity_targets'])
        variants=[dict(checkpoint_physics_valid=False),dict(checkpoint_capture_reason='stopped'),
            dict(cycle_plans=[dict(status='stopped',end_s=2.)]),
            dict(cycle_plans=[dict(status='motion_completed',end_s=1.)]),
            dict(cycle_plans=[dict(status='motion_completed',end_s=2.,stop_reason='native_contact_stop')])]
        for index,updates in enumerate(variants):
            with self.subTest(updates=updates):
                sequence=copy.deepcopy(metadata['sequence']);sequence.update(updates)
                name='invalid_'+str(index);source.save(name,sequence=sequence,**base)
                with self.assertRaises(ValueError):helper(source.assembly,manifest,source.root/name,kin)


if __name__=='__main__':unittest.main()
