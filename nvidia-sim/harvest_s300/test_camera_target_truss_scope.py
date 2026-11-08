"""Camera target-only scope tests; no renderer/native physics success claim."""
import copy
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
import tempfile
from unittest.mock import patch
import json
import numpy as np
import camera_patrol_runner as runner
from run_row_harvest import target_truss_camera_scope,target_truss_camera_scan_seed,startup_preflight_arguments
from camera_patrol import CameraPatrol
from row_harvest_targets import eligible_truss_fruits

BASE=['--headless','--camera-harvest','--capture-gt','--whole-truss',
    '--native-materialized-stage','--verify-full-environment-preflight',
    '--verify-harvest-contact-after-step','--save-route-checkpoint','--checkpoint-cycle-states',
    '--mode','harvest','--plant-dynamics','full','--camera-scan-view','row-front']

def fixture():
    root='/World/ElasticTarget';source='/World/Greenhouse/Plants/Plant_20'
    fruits=[dict(name=f'Truss_{t:02d}_Fruit_{f:02d}',path=root+f'/Truss_{t:02d}_Fruit_{f:02d}',
        joint=root+f'/HarvestJoints/Truss_{t:02d}_Fruit_{f:02d}',center=[-.6,.01*f,1.+.1*t])
        for t in range(3) for f in (1,2)]
    target=copy.deepcopy(fruits[4])
    manifest=dict(scene_sha256='fixture-sha',plant=dict(root=root,source_plant=source,fruits=fruits),target=target,
        plant_interaction_variant=dict(mode='target_truss_and_own_stem_only',target_root=root,target_truss='Truss_02'))
    inventory=dict(trusses=[dict(plant_id=20,truss_index=t,gutter_center_x=-.775,
        root_position_world_m=[-.68,0.,.3],fruit_centroid_world_m=[-.6,0.,1.+.1*t],rail_near=True)
        for t in range(3)])
    return manifest,inventory

class TargetTrussScopeTests(unittest.TestCase):
    def test_live_camera_group_depth_does_not_depend_on_authored_initial_selection(self):
        # Target-only camera startup deliberately has no authored fruit group.
        # Execute the live group's production depth branch with only the module
        # imports, so a helper hidden in the bypassed initial branch is caught.
        source=Path(__file__).with_name('run_motion.py').read_text()
        tree=ast.parse(source)
        imports=[node for node in tree.body if isinstance(node,ast.ImportFrom)
                 and node.module=='grasp_depth_schedule']
        branches=[node for node in ast.walk(tree) if isinstance(node,ast.If)
                  and ast.get_source_segment(source,node.test)=='a.later_grasp_depth_mm is not None'
                  and any(isinstance(child,ast.Call) and isinstance(child.func,ast.Name)
                          and child.func.id=='grasp_depth_for_selected_group'
                          for child in ast.walk(node))]
        branch=min(branches,key=lambda node:node.end_lineno-node.lineno)
        program=compile(ast.fix_missing_locations(ast.Module(body=imports+[branch],type_ignores=[])),
                        'production_live_camera_group_depth','exec')
        scan=dict(purpose='camera_scan_prefix',harvest_attempt=False,status='motion_completed',
                  start_s=0.,end_s=5.,planned_end_s=5.,measured_frame_index=0,end_frame_index=300)
        prior=dict(status='motion_completed',start_s=2.,end_s=54.,planned_end_s=54.,
                   measured_frame_index=0,end_frame_index=3240,
                   fruits=['Truss_02_Fruit_02','Truss_02_Fruit_04'])
        group=dict(truss_key='Truss_01',fruits=['Truss_01_Fruit_01','Truss_01_Fruit_02'])
        for near,scope,history,expected in ((True,'trial',[prior],15.),
                (True,'truss',[prior],35.),(False,'trial',[prior],15.),
                (True,'trial',[],35.)):
            with self.subTest(near=near,scope=scope,history=bool(history)):
                env=dict(a=SimpleNamespace(later_grasp_depth_mm=15.,grasp_depth_mm=35.,
                         later_grasp_depth_scope=scope,near_trusses_metadata=Path('/inventory') if near else None),
                         group=group,previous_cycle_records=history,sequence=dict(cycle_plans=[scan]),
                         prior_completed_native_cycles=len(history),record={},effective_depth_mm=35.)
                exec(program,env)
                self.assertEqual(env['effective_depth_mm'],expected)
                self.assertEqual(env['record']['plan_parameters']['grasp_depth_mm'],expected)
        # Reproduce the actual resumed profile's15/15 scheduling for Fruit01/02.
        env['a'].grasp_depth_mm=15.;exec(program,env)
        self.assertEqual(env['effective_depth_mm'],15.)

    def test_native_reader_fragment_handles_empty_scan_and_real_selected_group(self):
        # Execute the actual production AST fragment, rather than a copied
        # implementation. The renderer/SDK is deliberately not imported.
        source=Path(__file__).with_name('run_motion.py').read_text()
        tree=ast.parse(source);fragments=[]
        for node in ast.walk(tree):
            if isinstance(node,ast.If) and any(isinstance(call,ast.Call) and
                    isinstance(call.func,ast.Attribute) and call.func.attr=='retarget_fruit_truss' and
                    isinstance(call.func.value,ast.Name) and call.func.value.id=='leaf_geometry'
                    for stmt in node.body for call in ast.walk(stmt)):
                # Choose the innermost initial-reader setup, not an outer
                # loop that also contains later actual selection calls.
                if "initial_sequence_group['fruits'][0]" in ast.get_source_segment(source,node):
                    fragments.append(node)
        node=min(fragments,key=lambda item:item.end_lineno-item.lineno)
        program=compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),
            'production_native_reader_setup','exec')
        calls=[]
        reader=SimpleNamespace(retarget_fruit_truss=lambda m,name:calls.append((m,name)))
        manifest={'target':{'name':'Truss_02_Fruit_01'}}
        for group in (None,dict(fruits=[],truss_key='Truss_02')):
            exec(program,dict(a=SimpleNamespace(near_trusses_metadata=Path('/tmp/inventory.json')),
                initial_sequence_group=group,leaf_geometry=reader,manifest=manifest))
        self.assertEqual(calls,[])
        exec(program,dict(a=SimpleNamespace(near_trusses_metadata=Path('/tmp/inventory.json')),
            initial_sequence_group=dict(fruits=['Truss_02_Fruit_03','Truss_02_Fruit_04']),
            leaf_geometry=reader,manifest=manifest))
        self.assertEqual(calls,[(manifest,'Truss_02_Fruit_03')])
        # A source without the near-scope option retains legacy behavior.
        exec(program,dict(a=SimpleNamespace(near_trusses_metadata=None),
            initial_sequence_group=dict(fruits=['Truss_02_Fruit_03']),
            leaf_geometry=reader,manifest=manifest))
        self.assertEqual(len(calls),1)

    def test_narrows_exact_live_truss_and_preserves_source_records(self):
        manifest,inventory=fixture();before=copy.deepcopy((manifest,inventory))
        scope=target_truss_camera_scope(manifest,inventory)
        self.assertEqual(scope['eligible_truss_keys'],['Truss_02'])
        self.assertEqual(scope['eligible_fruit_count'],2)
        fruits=eligible_truss_fruits(manifest,scope)
        self.assertEqual([f['name'] for f in fruits],['Truss_02_Fruit_01','Truss_02_Fruit_02'])
        self.assertIs(fruits[0],manifest['plant']['fruits'][4])
        self.assertEqual((manifest,inventory),before)
        self.assertFalse(scope['physical_collision_policy_changed'])

    def test_rejects_wrong_selected_or_disabled_truss(self):
        manifest,inventory=fixture()
        for wrong in ('Truss_00','Truss_01','Truss_99'):
            with self.assertRaisesRegex(ValueError,'collision-enabled'):
                target_truss_camera_scope(manifest,inventory,selected_truss=wrong)

    def test_rejects_changed_native_joint_or_collision_mapping(self):
        for mutation in ('joint','policy','mode'):
            manifest,inventory=fixture()
            if mutation=='joint':manifest['target']['joint']='/wrong/joint'
            if mutation=='policy':manifest['plant_interaction_variant']['target_truss']='Truss_01'
            if mutation=='mode':manifest['plant_interaction_variant']['mode']='active_plant_and_own_stem_only'
            with self.assertRaises(ValueError):target_truss_camera_scope(manifest,inventory)

    def test_near_side_inventory_is_not_bypassed(self):
        manifest,inventory=fixture()
        for row in inventory['trusses']:row.update(root_position_world_m=[-.9,0.,.3],rail_near=False)
        with self.assertRaisesRegex(ValueError,'near-side'):
            target_truss_camera_scope(manifest,inventory)

    def test_exhausted_group_still_has_empty_scan_seed_without_hidden_centroid(self):
        manifest,inventory=fixture();scope=target_truss_camera_scope(manifest,inventory)
        world=np.eye(4);world[1,3]=.45
        def fk(q):
            pose=np.eye(4);pose[2,3]=float(q[0]);return pose
        kin=SimpleNamespace(world=world,fk=fk)
        seed,point=target_truss_camera_scan_seed(scope,kin,[1.22])
        self.assertEqual(seed['fruits'],[]);self.assertFalse(seed['harvest_attempt'])
        self.assertFalse(seed['uses_hidden_gt_center']);np.testing.assert_array_equal(point,[-.775,.45,1.22])
        for row in scope['trusses']:
            row['fruit_centroid_world_m']=[1e8,2e8,-3e8]
        seed2,point2=target_truss_camera_scan_seed(scope,kin,[1.22])
        self.assertEqual(seed,seed2);np.testing.assert_array_equal(point,point2)

    def test_optin_dispatch_keeps_measurement_resume_and_guard_options(self):
        args=runner.patrol_motion_arguments(BASE,'/tmp/inventory.json',
            decision=dict(action='scan'),checkpoint='/tmp/actual20',target_truss_windows=True)
        self.assertEqual(args.count('--camera-target-truss-window'),1)
        self.assertIn('--retain-lift-drive-reference',args)
        self.assertIn('--resume-native-checkpoint',args)
        self.assertIn('--near-trusses-metadata',args)
        for guard in ['--verify-full-environment-preflight','--verify-harvest-contact-after-step','--capture-gt']:
            self.assertIn(guard,args)
        probe=startup_preflight_arguments(args,physics_only=True)
        self.assertNotIn('--camera-target-truss-window',probe)
        self.assertNotIn('--camera-harvest',probe);self.assertIn('--physics-only',probe)

    def test_default_dispatch_array_remains_byte_exact(self):
        actual=runner.patrol_motion_arguments(BASE,'/tmp/inventory.json',decision=dict(action='scan'))
        expected=BASE+['--near-trusses-metadata','/tmp/inventory.json','--seconds','0','--camera-scan-only']
        self.assertEqual(actual,expected)
        with self.assertRaisesRegex(ValueError,'explicit patrol'):
            runner.patrol_motion_arguments(BASE+['--camera-target-truss-window'],'/tmp/inventory.json',decision=dict(action='scan'))

    def test_same_plant_revision_dispatch_requires_fresh_scan_exact_state_and_scope(self):
        scan=dict(action='scan')
        args=runner.patrol_motion_arguments(BASE,'/tmp/inventory.json',decision=scan,
            checkpoint='/tmp/latest-measured-state',target_truss_windows=True,resume_target_truss_revision=True)
        self.assertEqual(args.count('--allow-resume-target-truss-revision'),1)
        self.assertIn('--camera-scan-only',args)
        self.assertEqual(args[args.index('--resume-native-checkpoint')+1],'/tmp/latest-measured-state')
        for kwargs in [dict(target_truss_windows=False),dict(checkpoint=None),dict(new_window=True),
                dict(decision=dict(action='harvest',target={'truss_key':'Truss_00'}))]:
            arguments=dict(decision=scan,checkpoint='/tmp/latest-measured-state',
                target_truss_windows=True,resume_target_truss_revision=True)
            arguments.update(kwargs)
            with self.assertRaisesRegex(ValueError,'fresh target-only camera scan'):
                runner.patrol_motion_arguments(BASE,'/tmp/inventory.json',**arguments)
        probe=startup_preflight_arguments(args,physics_only=True)
        self.assertNotIn('--allow-resume-target-truss-revision',probe)

    def test_derived_model_environment_uses_exact_active_checkpoint_without_freezing_alias(self):
        from test_same_plant_target_revision import fixture as revision_fixture
        from same_plant_target_revision import revise
        from rail_environment_preflight import start_checkpoint
        from native_route_checkpoint import load_checkpoint
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);_,original,checkpoint,_,_=revision_fixture(root/'source')
            derived=root/'derived';revise(root/'source',checkpoint,derived,'Truss_00_Fruit_01')
            manifest=json.loads((derived/'motion_manifest.json').read_text())
            with self.assertRaises(ValueError):start_checkpoint(derived,manifest,checkpoint)
            poses,proof=start_checkpoint(derived,manifest,checkpoint,allow_target_truss_revision=True)
            metadata,arrays=load_checkpoint(checkpoint)
            self.assertEqual(set(poses),set(map(str,arrays['body_paths'])))
            for path in ('/World/ElasticTarget/Truss_00_Fruit_01','/World/ElasticTarget/Truss_02_Fruit_01'):
                self.assertIn(path,poses)
            self.assertIn('explicit_target_truss_revision',proof)
            self.assertNotIn('explicit_attachment_revision',proof)
            self.assertEqual(proof['native_break_events'][0]['joint'],metadata['break_events'][0]['joint'])
            with self.assertRaisesRegex(ValueError,'one explicitly authorized'):
                start_checkpoint(derived,manifest,checkpoint,allow_target_truss_revision=True,allow_attachment_transfer=True)

    def test_production_dispatch_retargets_current_checkpoint_then_scans_before_harvest(self):
        source=Path(runner.__file__).read_text();tree=ast.parse(source)
        nodes=[node for node in ast.walk(tree) if isinstance(node,ast.If) and
            any(isinstance(stmt,ast.ImportFrom) and stmt.module=='same_plant_target_revision' for stmt in node.body)]
        self.assertEqual(len(nodes),1)
        code=compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])),'actual_same_plant_dispatch','exec')
        manifest,inventory=fixture();plant=manifest['plant']['source_plant']
        target=dict(source_plant=plant,plant_id=20,truss_key='Truss_00',
            last_visible_fruit_keys=[plant+'/Truss_00/Fruit_01',plant+'/Truss_00/Fruit_02'])
        decision=dict(action='harvest',target_key=plant+'/Truss_00',target=target)
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);calls=[]
            def derive(assembly,checkpoint,output,target_name,**kwargs):
                calls.append((assembly,checkpoint,target_name,kwargs));output.mkdir(parents=True)
                revised=copy.deepcopy(manifest);revised['target']=copy.deepcopy(manifest['plant']['fruits'][0])
                revised['plant_interaction_variant']['target_truss']='Truss_00'
                (output/'motion_manifest.json').write_text(json.dumps(revised));return {'checkpoint':str(checkpoint)}
            env=dict(target_truss_windows=True,decision=decision,target=target,source=plant,manifest=manifest,
                accepted_assembly=root/'current',accepted_checkpoint=root/'latest_actual_checkpoint',
                output=root,index=3,record={},station=.97,copy=copy,target_truss_camera_scope=target_truss_camera_scope,
                inventory=inventory,_read=runner._read)
            with patch('same_plant_target_revision.revise',side_effect=derive):exec(code,env)
            self.assertEqual(calls[0][1],root/'latest_actual_checkpoint')
            self.assertEqual(calls[0][2],'Truss_00_Fruit_01')
            self.assertEqual(env['decision']['action'],'scan')
            self.assertEqual(env['decision']['original_harvest_decision'],decision)
            self.assertTrue(env['resume_target_revision'])
            self.assertEqual(env['name'],'segment_003_scan')
            self.assertEqual(env['record']['target_truss_window_scope']['eligible_truss_keys'],['Truss_00'])

    def test_target_scoped_idle_failure_never_quarantines_whole_source(self):
        row=dict(index=2,status='unused_window_startup_deferred',accepted_parent_unchanged=True,
            decision=dict(target_key='/World/Greenhouse/Plants/Plant_20/Truss_00',
                target=dict(source_plant='/World/Greenhouse/Plants/Plant_20')),
            startup_preflight=dict(status='placement_initialization_or_idle_physics_invalid',
                can_defer_unused_placement=True,native_physics_initialization_observed=True,
                accepted_campaign_motion=False,diagnostic_state_accepted=False))
        self.assertIsNotNone(runner.shared_initial_model_blocker(row,BASE))
        row['target_truss_window_scope']={'target_truss_window':{'target_truss':'Truss_00'}}
        self.assertIsNone(runner.shared_initial_model_blocker(row,BASE))
        del row['target_truss_window_scope']
        self.assertIsNone(runner.shared_initial_model_blocker(row,BASE+['--camera-target-truss-window']))

if __name__=='__main__':unittest.main()
