"""Run the real camera retry loop without loading Kit or advancing physics."""
import ast
import copy
from contextlib import redirect_stderr, redirect_stdout
import io
import hashlib
import json
from pathlib import Path
import tempfile
import traceback
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from test_run_motion_conditioning import LOOP, MAIN, actual_cli, code
from truss_sequence import inventory
from camera_plan_alternatives import camera_plan_alternatives


class Observations:
    def __init__(self, groups,gt=False):
        self.groups=groups;self.calls=[];self.heading=0.
        self.gt=gt
        self.last_decision=dict(reason='group_planning_alternatives_exhausted')
    def set_yaw_offset_deg(self,value):self.heading=value
    def __call__(self,fruits,breaks,paths,poses,attempts,*,rejected_groups,**kwargs):
        self.calls.append((self.heading,set(rejected_groups)))
        for names in self.groups.get(self.heading,[]):
            if tuple(names) in rejected_groups:continue
            angle=np.deg2rad(self.heading)
            group=dict(fruits=list(names),truss_key='Truss_00',aim_world_m=[0.,1.,1.],
                measured_centers_world_m=[[0.,1.,1.],[0.,1.02,1.]],
                measured_centroid_world_m=[0.,1.01,1.],
                group_selection=dict(direction_world=[np.cos(angle),np.sin(angle),0.]))
            if self.gt:group['group_selection']['unrotated_direction_world']=[1.,0.,0.]
            return group
        return None


class CameraHeadingSearchTests(unittest.TestCase):
    def retry(self,groups,limit=12,seed_indices=None,auto_geometric_rejection=False,auto_selected_seed=0,accept_seed=None,gt=False,planning_proof=True,full_environment=False):
        boundary=next(n for n in LOOP.body if isinstance(n,ast.If))
        loop=next(n for n in boundary.body if isinstance(n,ast.While))
        setup=next(n for n in boundary.body if isinstance(n,ast.If) and any(
            isinstance(child,ast.Name) and child.id=='camera_heading_rejections' for child in ast.walk(n)))
        terminate=next(n for n in boundary.body if isinstance(n,ast.If) and
            ast.unparse(n.test)=='not planned_next')
        offsets=[0.,15.,-15.,30.,-30.];selector=Observations(groups,gt=gt);plan_calls=[]
        args=dict(camera_harvest=not gt,group_heading_search=gt,camera_heading_offsets_deg=offsets,max_truss_cycles=limit,
            max_fruit_attempts=2,truss_order='highest-first',single_fruit_when_remaining=0,
            later_grasp_depth_mm=None,last_fruit_height_offset_mm=None,last_fruit_depth_mm=None,
            grasp_depth_mm=35.,grasp_height_offset_mm=-10.,preparation='horizontal_ready',
            horizontal_ready_connector='joint',horizontal_ready_seed_index=None if auto_geometric_rejection else 8,
            camera_ik_seed_indices=seed_indices,retain_lift_drive_reference=False,
            camera_harvest_ready_lift_offset_mm=None,camera_prefix_connection='off',
            camera_prefix_seed=2026,camera_prefix_proposal_resolution='fine',
            horizontal_ready_normalize_turns=False,ready_lift_offset_mm=0.,entry_retract_mm=200.,
            grasp_lateral_offset_mm=0.,preparation_gap_mm=96.,preparation_opening='ready',
            belt_seconds=2.,close_seconds=6.,belt_speed_m_s=.05,sparse_entry_duration_scale=1.,
            release_relief_before_withdraw=False,
            last_fruit_grasp_sway_mm=0.,native_tool_collision_cache=None,hz=60,
            verify_full_environment_preflight=full_environment,assembly=Path('/unit-fixture'),near_trusses_metadata=None,
            released_fruit_ground_diagnostic=False,released_fruit_gripper_support_diagnostic=False,
            released_fruit_same_truss_diagnostic=False,allow_resume_attachment_transfer=True,
            allow_resume_target_truss_revision=False,contact_limit_mm=1.)
        fruits=[dict(name=f'fruit_{i}',joint=f'/j{i}',path=f'/f{i}') for i in range(6)]
        sequence=dict(cycle_plans=[dict(index=0,fruits=[],purpose='camera_scan_prefix')])
        def blocked(*args,**kwargs):raise RuntimeError('Finite obstacle witness in fixture')
        def fake_plan(*args,**kwargs):
            seed=kwargs['camera_ik_seed_index']
            plan_calls.append(dict(axis=copy.deepcopy(args[1]),parameters=copy.deepcopy(kwargs)))
            return [],dict(selected_seed_index=auto_selected_seed if seed is None else seed,phases=[])
        def geometric_rejection(assembly,output,*,plan,**kwargs):
            if json.loads(plan.read_text())['selected_seed_index']==accept_seed:return dict(passed=True,complete=True)
            output.write_text(json.dumps(dict(passed=False,complete=False,scene_sha256='a'*64,
                input_sha256=hashlib.sha256(plan.read_bytes()).hexdigest(),
                phases=[dict(phase='preparation',hits=[dict(a='arm',b='tool',intersection=True)])])))
            raise RuntimeError('Self-collision audit blocked motion: '+str(output))
        with tempfile.TemporaryDirectory() as directory:
            args['output']=Path(directory)
            context=dict(a=SimpleNamespace(**args),np=np,json=json,sequence=sequence,
                physical_unproductive_pairs=set(),group_policy_selector=selector,sequence_selector=selector,
                rejected_groups=set(),rejected_fingerprints=set(),attempts={},selected_fruits=fruits,
                breaks=[],body_paths=[f['path'] for f in fruits],poses=np.zeros((6,7)),
                initial_robot_q=np.zeros(7),camera_suffix_rail_origin=None,
                current_time=8.5,times=[0.],planned_next=False,boundary_checkpoint=Path('/unit/current_checkpoint'),
                manifest=dict(scene_sha256='a'*64,grasp_reference=dict(height_offset_m=0.)),reference_center=np.zeros(3),
                make_plan=fake_plan if auto_geometric_rejection else blocked,audit_external=geometric_rejection,
                inventory=inventory,selection_evidence=lambda group:None,leaf_geometry=None,
                rgbd=SimpleNamespace(set_harvest_region=lambda group:None),traceback=traceback)
            context['plan_calls']=plan_calls
            context['unexecuted_proof_calls']=[]
            def planning_failure_proof(*args,**kwargs):
                context['unexecuted_proof_calls'].append(kwargs)
                return dict(native_physics_started=False) if planning_proof else None
            context['environment_calls']=[]
            def environment_audit(*args,**kwargs):
                context['environment_calls'].append(dict(args=args,kwargs=kwargs))
                return dict(passed=True,complete=True)
            context['camera_alternatives']=camera_plan_alternatives(offsets,args['horizontal_ready_seed_index'],seed_indices=seed_indices)
            if gt:
                with patch('run_row_harvest.unexecuted_planning_failure',side_effect=planning_failure_proof),patch.dict('sys.modules',{'rail_environment_preflight':SimpleNamespace(audit_external_rail=environment_audit)}),redirect_stdout(io.StringIO()):
                    exec(code([setup,loop,terminate]),context)
            else:
                with redirect_stdout(io.StringIO()):exec(code([setup,loop,terminate]),context)
        return context,selector

    def test_rejections_are_heading_local_round_robin_and_proposal_cap_includes_scan(self):
        groups=[[f'fruit_{i}',f'fruit_{i+1}'] for i in (0,2,4)]
        context,selector=self.retry({angle:copy.deepcopy(groups) for angle in [0.,15.,-15.,30.,-30.]})
        records=context['sequence']['cycle_plans'][1:]
        self.assertEqual(len(records),11)
        self.assertEqual([r['camera_heading_search']['offset_deg'] for r in records[:6]],
            [0.,15.,-15.,30.,-30.,0.])
        self.assertEqual([r['fruits'] for r in records[:5]],[groups[0]]*5)
        self.assertTrue(all(not prior for _,prior in selector.calls[:5]))
        self.assertEqual(len({id(value) for value in context['camera_heading_rejections'].values()}),5)
        self.assertEqual(context['attempts'],{})
        self.assertTrue(all(r['status']=='planning_rejected' for r in records))
        self.assertEqual(context['sequence']['termination'],'attempt_budget_exhausted_remaining_fruit')

    def test_all_empty_headings_stop_without_consuming_physical_attempts(self):
        context,selector=self.retry({})
        self.assertEqual(len(selector.calls),5)
        self.assertEqual(len(context['camera_heading_exhausted']),5)
        self.assertEqual(len(context['sequence']['cycle_plans']),1)
        self.assertEqual(context['attempts'],{})

    def test_rejected_group_can_retry_a_different_ik_branch_without_extra_budget(self):
        groups=[['fruit_0','fruit_1']]
        context,selector=self.retry({0.:groups},seed_indices=[2,8])
        records=context['sequence']['cycle_plans'][1:]
        self.assertEqual([r['camera_heading_search']['requested_staging_seed_index'] for r in records],[2,8])
        self.assertEqual([r['fruits'] for r in records],[groups[0],groups[0]])
        self.assertEqual(len({r['planning_fingerprint'] for r in records}),2)
        self.assertEqual(len(context['camera_heading_exhausted']),10)
        self.assertLess(len(selector.calls),35)
        self.assertEqual(context['attempts'],{})

    def test_branch_search_preserves_the_original_total_proposal_cap(self):
        groups=[[f'fruit_{i}',f'fruit_{i+1}'] for i in (0,2,4)]
        context,_=self.retry({angle:groups for angle in [0.,15.,-15.,30.,-30.]},seed_indices=[0,2,8])
        records=context['sequence']['cycle_plans'][1:]
        self.assertEqual(len(records),11)
        self.assertEqual([r['camera_heading_search']['requested_staging_seed_index'] for r in records[:6]],
                         [0,0,0,0,0,2])
        self.assertEqual(context['attempts'],{})

    def test_geometrically_rejected_auto_branch_retries_same_group_and_heading_before_advancing(self):
        groups=[[f'fruit_{i}',f'fruit_{i+1}'] for i in (0,2,4)]
        context,selector=self.retry({angle:copy.deepcopy(groups) for angle in [0.,15.,-15.,30.,-30.]},
            auto_geometric_rejection=True)
        records=context['sequence']['cycle_plans'][1:]
        self.assertEqual(len(records),11)  # original12 proposals includes initial scan
        self.assertEqual([(r['camera_heading_search']['offset_deg'],
            r['camera_heading_search']['requested_staging_seed_index']) for r in records[:6]],
            [(0.,None),(0.,5),(0.,8),(15.,None),(15.,5),(15.,8)])
        self.assertEqual([r['fruits'] for r in records[:3]],[groups[0]]*3)
        self.assertEqual([call[0] for call in selector.calls[:3]],[0.,15.,-15.])
        self.assertTrue(all(r['status']=='planning_rejected' for r in records))
        self.assertTrue(all(r['auto_ik_geometric_fallback']['audit']['geometric_blocking_witness']
            for r in records if r['camera_heading_search']['requested_staging_seed_index'] is not None))
        self.assertEqual(context['attempts'],{})  # no rejected proposal became a physical attempt
        self.assertEqual(context['sequence']['termination'],'attempt_budget_exhausted_remaining_fruit')

    def test_accepted_auto_branch_does_not_enqueue_fallback_or_change_selected_group(self):
        context,selector=self.retry({0.:[['fruit_0','fruit_1']]},auto_geometric_rejection=True,accept_seed=0)
        records=context['sequence']['cycle_plans'][1:]
        self.assertTrue(context['planned_next']);self.assertEqual(len(records),1)
        self.assertEqual(records[0]['status'],'executing')
        self.assertIsNone(records[0]['camera_heading_search']['requested_staging_seed_index'])
        self.assertNotIn('auto_ik_geometric_fallback',records[0])
        self.assertFalse(context['camera_pending_fallbacks']);self.assertEqual(len(selector.calls),1)

    def test_fallback_skips_the_already_auto_selected_branch_and_cannot_exceed_cap(self):
        context,_=self.retry({0.:[['fruit_0','fruit_1']]},limit=3,
            auto_geometric_rejection=True,auto_selected_seed=5)
        records=context['sequence']['cycle_plans'][1:]
        self.assertEqual([r['camera_heading_search']['requested_staging_seed_index'] for r in records],[None,8])
        self.assertEqual(len(context['sequence']['cycle_plans']),3)
        self.assertEqual(context['attempts'],{})

    def test_exhausted_headings_do_not_create_infinite_loop_around_remaining_one(self):
        groups=[[f'fruit_{i}',f'fruit_{i+1}'] for i in (0,2,4)]
        context,selector=self.retry({15.:groups})
        records=context['sequence']['cycle_plans'][1:]
        self.assertEqual([r['fruits'] for r in records],groups)
        self.assertTrue(all(r['camera_heading_search']['offset_deg']==15. for r in records))
        self.assertLess(len(selector.calls),30)
        self.assertEqual(len(context['camera_heading_exhausted']),5)
        self.assertEqual(context['attempts'],{})

    def test_cli_global_offset_is_combined_once_before_camera_selector(self):
        flags=['--camera-harvest','--whole-truss','--preparation','horizontal_ready',
            '--approach-yaw-mode','truss-axis','--near-trusses-metadata','/unused-near.json',
            '--truss-order','highest-first','--harvest-profile','belt_open_close_cycle']
        self.assertEqual(actual_cli(flags+['--truss-entry-yaw-offset-deg','30']).camera_heading_offsets_deg,
            [30.,45.,15.,60.,0.])
        self.assertEqual(actual_cli(flags+['--truss-entry-yaw-offset-deg','85']).camera_heading_offsets_deg,
            [85.,70.,55.])
        self.assertEqual(actual_cli(flags).camera_heading_offsets_deg,[0.,15.,-15.,30.,-30.])
        with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
            actual_cli(flags+['--truss-entry-yaw-offset-deg','85',
                '--camera-heading-offsets-deg','0','1e-15'])

    def test_actual_camera_constructor_has_no_hidden_or_already_rotated_axis_hint(self):
        calls=[n for n in ast.walk(MAIN) if isinstance(n,ast.Call) and
            isinstance(n.func,ast.Name) and n.func.id=='CameraRegionSelector']
        self.assertEqual(len(calls),1)
        self.assertIsInstance(calls[0].args[1],ast.Constant)
        self.assertIsNone(calls[0].args[1].value)

    def test_gt_reuses_same_loop_at_fixed_measured_state_without_camera_labels_or_attempt_credit(self):
        groups=[['fruit_0','fruit_1']]
        context,selector=self.retry({angle:groups for angle in [0.,15.,-15.,30.,-30.]},gt=True,
            seed_indices=[8,5],auto_geometric_rejection=True,accept_seed=5)
        records=context['sequence']['cycle_plans'][1:]
        self.assertTrue(context['planned_next']);self.assertEqual(len(records),6)
        self.assertEqual([r['gt_group_heading_search']['offset_deg'] for r in records],[0.,15.,-15.,30.,-30.,0.])
        self.assertTrue(all('camera_heading_search' not in row and 'camera_ik_branch' not in row for row in records))
        self.assertTrue(all(row['start_s']==8.5 and row['gt_group_heading_search']['boundary_time_s']==8.5 for row in records))
        self.assertTrue(all(row['gt_group_heading_search']['native_state_reset'] is False for row in records))
        self.assertEqual(context['attempts'],dict(fruit_0=1,fruit_1=1))
        self.assertEqual(context['current_time'],8.5);self.assertEqual(context['breaks'],[])
        self.assertTrue(all(row['status']=='planning_rejected' for row in records[:-1]))
        self.assertEqual(records[-1]['status'],'executing')
        self.assertEqual(records[-1]['gt_group_ik_branch']['selected_staging_seed_index'],5)
        self.assertNotIn('camera_heading_offset_deg',records[-1]['gt_group_ik_branch'])
        self.assertTrue(all(call['parameters']['grasp_depth_mm']==35. for call in context['plan_calls']))
        for row,call in zip(records,context['plan_calls']):
            np.testing.assert_array_equal(call['axis']['direction_world'],row['group_selection']['direction_world'])
            self.assertTrue(call['axis']['direction_offset_already_applied'])
        self.assertEqual(len({row['planning_fingerprint'] for row in records}),6)
        np.testing.assert_array_equal(context['last_axis']['direction_world'],[1.,0.,0.])
        self.assertEqual(len(context['unexecuted_proof_calls']),5)
        self.assertTrue(all(call['allow_early_blocked_environment']is True for call in context['unexecuted_proof_calls']))

    def test_gt_rejected_proposals_keep_original_budget_and_never_increment_fruit_attempts(self):
        groups=[['fruit_0','fruit_1']]
        context,_=self.retry({angle:groups for angle in [0.,15.,-15.,30.,-30.]},gt=True,
            limit=4,seed_indices=[8,5],auto_geometric_rejection=True)
        self.assertFalse(context['planned_next']);self.assertEqual(len(context['sequence']['cycle_plans']),4)
        self.assertEqual(context['attempts'],{});self.assertEqual(context['current_time'],8.5)
        self.assertEqual(context['sequence']['termination'],'attempt_budget_exhausted_remaining_fruit')

    def test_gt_no_group_at_one_yaw_still_checks_other_yaws_with_the_same_minimum(self):
        context,selector=self.retry({15.:[['fruit_0','fruit_1']]},gt=True,
            seed_indices=[8,5],auto_geometric_rejection=True,accept_seed=8)
        self.assertTrue(context['planned_next'])
        self.assertEqual([row['gt_group_heading_search']['offset_deg'] for row in context['sequence']['cycle_plans'][1:]],[15.])
        self.assertEqual([row[0] for row in selector.calls],[0.,15.])
        self.assertEqual(context['current_time'],8.5)

    def test_gt_unknown_planning_failure_stops_before_another_heading_or_any_physics_credit(self):
        context,_=self.retry({angle:[['fruit_0','fruit_1']] for angle in [0.,15.,-15.,30.,-30.]},
            gt=True,seed_indices=[8,5],planning_proof=False)
        records=context['sequence']['cycle_plans'][1:]
        self.assertFalse(context['planned_next']);self.assertEqual(len(records),1)
        self.assertEqual(records[0]['status'],'planning_inconclusive')
        self.assertTrue(records[0]['no_additional_heading_after_unknown_failure'])
        self.assertEqual(context['stop_reason'],'gt_group_planning_inconclusive')
        self.assertEqual(context['attempts'],{});self.assertEqual(context['current_time'],8.5)

    def test_gt_accepted_suffix_reuses_exact_current_checkpoint_for_full_environment_audit(self):
        context,_=self.retry({0.:[['fruit_0','fruit_1']]},gt=True,seed_indices=[8,5],
            auto_geometric_rejection=True,accept_seed=8,full_environment=True)
        self.assertTrue(context['planned_next']);self.assertEqual(len(context['environment_calls']),1)
        audit=context['environment_calls'][0]
        self.assertEqual(audit['args'][0],Path('/unit-fixture'))
        self.assertEqual(audit['kwargs']['checkpoint'],Path('/unit/current_checkpoint'))
        self.assertEqual(audit['kwargs']['scope'],'full')
        self.assertEqual(audit['kwargs']['hz'],60);self.assertEqual(audit['kwargs']['native_hz'],60)
        self.assertEqual(audit['kwargs']['allowed_truss_key'],'Truss_00')
        self.assertEqual(audit['kwargs']['wheel_floor_overlap_limit_m'],.001)
        record=context['sequence']['cycle_plans'][-1]
        self.assertEqual(record['environment_preflight']['status'],'passed_complete')
        self.assertEqual(context['attempts'],dict(fruit_0=1,fruit_1=1))

    def test_gt_boundary_reuses_current_measured_q_and_rail_checkpoint_without_native_writes(self):
        boundary=next(n for n in LOOP.body if isinstance(n,ast.If))
        node=next(n for n in boundary.body if isinstance(n,ast.If) and any(
            isinstance(child,ast.Call) and isinstance(child.func,ast.Name) and child.func.id=='camera_boundary_rail_state'
            for child in ast.walk(n)))
        q=np.arange(10,dtype=float)[None,:];original=q.copy();calls=[]
        def rail(*args):calls.append(args);return 'measured_fk',dict(station_world_y_m=1.234)
        context=dict(a=SimpleNamespace(camera_harvest=False,group_heading_search=True,assembly=Path('/exact/current')),
            q=q,rids=[1,2,3,4,5,6,7],kin='original_fk',manifest={'rail':True},boundary_checkpoint=Path('/exact/boundary'),
            camera_boundary_rail_state=rail,sequence={},current_time=12.5)
        exec(code([node]),context)
        np.testing.assert_array_equal(context['initial_robot_q'],q[0,1:8]);np.testing.assert_array_equal(q,original)
        self.assertEqual(context['planning_kin'],'measured_fk');self.assertEqual(len(calls),1)
        self.assertEqual(calls[0][2],Path('/exact/boundary'))
        self.assertNotIn('camera_boundary_rail_origins',context['sequence'])
        self.assertEqual(context['sequence']['gt_group_boundary_rail_origins'][0]['time_s'],12.5)

    def test_gt_cli_opt_in_reuses_heading_seed_controls_and_startup_strips_them(self):
        flags=['--group-heading-search','--whole-truss','--preparation','horizontal_ready',
            '--horizontal-ready-connector','joint','--harvest-group-policy','/source/policy.json',
            '--approach-yaw-mode','truss-axis','--checkpoint-cycle-states',
            '--verify-full-environment-preflight','--native-tool-collision-cache','/source/native_cache.json',
            '--truss-order','highest-first','--harvest-profile','belt_open_close_cycle','--hz','60',
            '--camera-heading-offsets-deg','0','30','-30','60','-60','--camera-ik-seed-indices','8','5',
            '--truss-entry-yaw-offset-deg','-30']
        parsed=actual_cli(flags);self.assertTrue(parsed.group_heading_search);self.assertFalse(parsed.camera_harvest)
        self.assertFalse(parsed.capture_gt);self.assertFalse(parsed.capture_rgbd)
        self.assertEqual(parsed.camera_heading_offsets_deg,[-30.,0.,-60.,30.,-90.])
        self.assertEqual(parsed.camera_ik_seed_indices,[8,5])
        from run_row_harvest import startup_preflight_arguments
        idle=actual_cli(startup_preflight_arguments(flags))
        self.assertEqual(idle.mode,'idle');self.assertFalse(idle.group_heading_search);self.assertIsNone(idle.camera_ik_seed_indices)
        self.assertFalse(actual_cli([]).group_heading_search)
        for removed in ('--harvest-group-policy','--checkpoint-cycle-states','--verify-full-environment-preflight','--native-tool-collision-cache'):
            broken=list(flags);index=broken.index(removed);del broken[index:index+(2 if removed in ('--harvest-group-policy','--native-tool-collision-cache') else 1)]
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):actual_cli(broken)


if __name__=='__main__':unittest.main()
