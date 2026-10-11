"""Real CPU FCL tests for scoped nominal rail/whole-robot obstacle screening."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from motion_self_collision import MotionSelfCollision, check_plan
from rail_environment_preflight import (RailEnvironmentCheck, screen_rail_plan,
    verify_initial_alignment, collider_geometry, obstacle_class, ExactFilters,
    audit_rail_transit, start_checkpoint, audit_external, verified_release_context,
    screen_preparation_plan, preparation_leaf_class)
from compact_motion_assets import file_sha256


class Kinematics:
    def __init__(self, y=0): self.world = np.eye(4); self.world[1, 3] = y
    def frames(self, q):
        t = self.world.copy(); t[1, 3] += .15
        return {'/World/HarvestRobot/Robot/arm':t}
    def fk(self, q):
        t = self.world.copy(); t[:3, 3] += [3, .15, 0]
        return t


def cube(stage, path, xyz=(0, 0, 0), size=.1, source=None, rigid=False):
    p = UsdGeom.Xform.Define(stage, path); p.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    if rigid: UsdPhysics.RigidBodyAPI.Apply(p.GetPrim())
    shape = UsdGeom.Cube.Define(stage, path+'/shape'); shape.CreateSizeAttr(size)
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim()).CreateCollisionEnabledAttr(True)
    if source: shape.GetPrim().SetCustomDataByKey('sourceName', source)
    return shape.GetPrim()


def fixture():
    s = Usd.Stage.CreateInMemory()
    cube(s, '/World/HarvestRobot/Robot/world', rigid=True)
    cube(s, '/World/HarvestRobot/Robot/arm', (0, .15, 0), .05, rigid=True)
    cube(s, '/World/HarvestRobot/Robot/S300Carrier', (.2, 0, 0), .1, rigid=True)
    cube(s, '/World/ActuatedTool/fixed', (3, .15, 0), .1, rigid=True)
    m = dict(rail=dict(base_body='/World/HarvestRobot/Robot/world',
                      source_base_position_world_m=[0, 0, 0]),
             plant=dict(root='/World/ElasticTarget',
                        fruits=[dict(name='Truss_02_Fruit_01')]),
             target=dict(name='Truss_02_Fruit_01'))
    plan = dict(phases=[dict(name='rail_move', duration_s=1., waypoint_q=[[0.]*7]*2,
                            end_gap_m=.044, rail_waypoint_positions_m=[0., 1.])])
    return s, m, plan


def run(stage, manifest, plan, **kwargs):
    moving = MotionSelfCollision(stage, Kinematics())
    adapter = RailEnvironmentCheck(stage, manifest, moving, allowed_truss_key='Truss_02', **kwargs)
    return screen_rail_plan(adapter, plan, manifest), adapter


class WheelFloorSupportTests(unittest.TestCase):
    def scene(self, top=.00375):
        s,m,p=fixture()
        s.RemovePrim('/World/HarvestRobot/Robot/S300Carrier/shape')
        from rail_ground_alignment import WHEEL_PATHS
        for path,extents,center,source in [
            (WHEEL_PATHS[0],[.08,.08,.1],[0,0,.053],'T_01_S300_8inch_Wheel_V2NR-1'),
            ('/World/Greenhouse/House/Environment/CoverLap_1_6/Mesh',
             [.12,.12,.0015],[.2,0,top-.00075],None)]:
            mesh=trimesh.creation.box(extents=extents);v=mesh.vertices+center
            shape=UsdGeom.Mesh.Define(s,path)
            shape.CreatePointsAttr(v.tolist());shape.CreateFaceVertexCountsAttr([3]*len(mesh.faces))
            shape.CreateFaceVertexIndicesAttr(mesh.faces.reshape(-1).tolist())
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim()).CreateCollisionEnabledAttr(True)
            if source:
                shape.GetPrim().SetCustomDataByKey('sourceName',source)
                UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr('convexHull')
        return s,m,p

    def test_actual_fcl_overlap_is_recorded_under_explicit_one_mm(self):
        s,m,p=self.scene()
        report,_=run(s,m,p,wheel_floor_overlap_limit_m=.001)
        self.assertTrue(report['passed'])
        hits=report['diagnostic_only_hits']
        self.assertTrue(hits)
        self.assertEqual(hits[0]['obstacle_class'],'wheel_floor_support_within_limit')
        self.assertAlmostEqual(hits[0]['wheel_floor_support']['penetration_upper_bound_m'],.00075,places=7)
        self.assertFalse(hits[0]['wheel_floor_support']['physical_collision_disabled'])

    def test_legacy_zero_and_half_mm_still_block_and_deeper_contact_stays_hard(self):
        for limit,top in [(0.,.00375),(.0005,.00375),(.001,.0045)]:
            with self.subTest(limit=limit,top=top):
                s,m,p=self.scene(top);report,_=run(s,m,p,wheel_floor_overlap_limit_m=limit)
                self.assertFalse(report['passed'])
                self.assertGreater(report['hard_intersection_sample_pairs'],0)

    def test_no_permission_for_tool_or_gutter_or_wrong_wheel_identity(self):
        from rail_environment_preflight import wheel_floor_overlap_evidence
        from rail_ground_alignment import WHEEL_PATHS
        wheel=dict(path=WHEEL_PATHS[0],name='S300_Wheel')
        row=dict(path='/World/Greenhouse/House/Environment/CoverLap_1_6/Mesh',
                 bounds=np.array([[0,0,0],[1,1,.00375]]))
        self.assertIsNotNone(wheel_floor_overlap_evidence(wheel,row,.003,.001))
        for shape,obj in [(dict(path='/World/ActuatedTool/fixed/part_9',name='Wheel'),row),
                          (dict(wheel,name='frame'),row),
                          (wheel,dict(row,path='/World/Greenhouse/House/Environment/Gutter/Mesh'))]:
            self.assertIsNone(wheel_floor_overlap_evidence(shape,obj,.003,.001))


def completed(time_s=1.):
    return dict(time_s=time_s, sequence=dict(checkpoint_capture_reason='completed_cycle',
        checkpoint_physics_valid=True, cycle_plans=[dict(status='motion_completed',
        stop_reason=None, end_s=time_s)]))


def release_fixture(xyz=(.2, .5, 0)):
    s, m, p = fixture(); root = m['plant']['root']; name = m['target']['name']
    body = root+'/'+name; anchor = root+'/Truss_02_Pedicel_distal_01_0'; joint = root+'/HarvestJoints/'+name
    fruit = UsdGeom.Xform.Define(s, body); fruit.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    UsdPhysics.RigidBodyAPI.Apply(fruit.GetPrim())
    sphere = UsdGeom.Sphere.Define(s, body+'/Collider'); sphere.CreateRadiusAttr(.035)
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdGeom.Xform.Define(s, anchor)
    attachment = UsdPhysics.FixedJoint.Define(s, joint)
    attachment.CreateBody0Rel().SetTargets([anchor]); attachment.CreateBody1Rel().SetTargets([body])
    m['plant']['fruits'] = [dict(name=name, path=body, joint=joint, anchor=anchor)]
    passive = '/World/HarvestRobot/Robot/S300Carrier/shape'
    proof = {passive:dict(body='/World/HarvestRobot/Robot/S300Carrier', source_name='verified fixture source',
        scope='released_fruit_passive_s300_cad', collection_verified=False)}
    native = dict(native_break_events=[dict(joint=joint, t=0., original_t=.8, type='imported_native_prior')])
    return s, m, p, proof, native


class RailEnvironmentTests(unittest.TestCase):
    def test_60hz_rail_checks_actual_motion_and_rejects_period_mismatch(self):
        stage, manifest, plan = fixture()
        cube(stage, '/World/Greenhouse/House/Gutter', (0, .5, 0))
        def check(hz,native_hz):
            moving=MotionSelfCollision(stage,Kinematics())
            adapter=RailEnvironmentCheck(stage,manifest,moving,allowed_truss_key='Truss_02')
            return screen_rail_plan(adapter,plan,manifest,hz=hz,native_hz=native_hz)
        blocked=check(60,60)
        self.assertTrue(blocked['complete']);self.assertFalse(blocked['passed'])
        self.assertGreater(blocked['hard_intersection_sample_pairs'],0)
        self.assertEqual(blocked['samples'],61)
        mismatched=check(60,120)
        self.assertFalse(mismatched['complete']);self.assertFalse(mismatched['passed'])
        self.assertIn('must match',mismatched['error'])

    def test_real_mid_path_gutter_intersection_is_complete_blocked_not_impossible(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Gutter', (0, .5, 0))
        report, adapter = run(s, m, p)
        self.assertTrue(report['complete']); self.assertFalse(report['passed'])
        self.assertEqual(report['status'], 'blocked'); self.assertEqual(report['samples'], 121)
        hit = report['hard_hits'][0]
        self.assertEqual(hit['obstacle_class'], 'gutter'); self.assertGreater(hit['time_s'], 0)
        self.assertLess(hit['time_s'], 1)
        self.assertEqual(adapter.moving.clearance, 0)

    def test_positive_clearance_below_two_mm_is_clear_no_universal_margin(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Gutter', (-.101, .5, 0))
        before = s.GetRootLayer().ExportToString(); report, _ = run(s, m, p)
        self.assertTrue(report['passed']); self.assertTrue(report['complete'])
        self.assertEqual(report['hard_intersection_sample_pairs'], 0)
        self.assertEqual(s.GetRootLayer().ExportToString(), before)

    def test_separate_moving_carrier_and_tool_are_both_screened(self):
        for path, xyz, required in (
            ('/World/Greenhouse/House/CarrierBlock', (.2, .5, 0), '/S300Carrier/shape'),
            ('/World/Greenhouse/House/ToolBlock', (3, .65, 0), '/ActuatedTool/fixed/shape')):
            s, m, p = fixture(); cube(s, path, xyz)
            report, _ = run(s, m, p)
            self.assertFalse(report['passed'])
            self.assertTrue(any(required in h['robot_geom'] for h in report['hard_hits']))

    def test_soft_leaf_hits_recorded_without_blocking_rigid_other_obstacle(self):
        s, m, p = fixture()
        cube(s, '/World/Greenhouse/Plants/Plant_61/Stem/mesh_177', (0, .5, 0), source='CompoundLeaf_13_214888')
        report, _ = run(s, m, p)
        self.assertTrue(report['passed']); self.assertGreater(report['diagnostic_intersection_sample_pairs'], 0)
        self.assertEqual(report['diagnostic_only_hits'][0]['obstacle_class'], 'soft_leaf')
        cube(s, '/World/Greenhouse/Plants/Plant_61/Stem/MainStem', (0, .7, 0), source='MainStem_12')
        report, _ = run(s, m, p)
        self.assertFalse(report['passed']); self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'main_stem')

    def test_current_truss_allowance_never_crosses_foreign_or_mainstem(self):
        s, m, p = fixture()
        current = cube(s, '/World/ElasticTarget/Truss_02_Pedicel_distal_01_0', (3, .65, 0))
        report, _ = run(s, m, p)
        self.assertTrue(report['passed']); self.assertEqual(obstacle_class(current, m, 'Truss_02'), ('current_allowed_truss', False))
        foreign = cube(s, '/World/Greenhouse/Plants/Plant_61/Truss_02_Pedicel_distal_01_0', (0, .7, 0))
        report, _ = run(s, m, p)
        self.assertFalse(report['passed']); self.assertEqual(obstacle_class(foreign, m, 'Truss_02'), ('foreign_pedicel', True))
        main = cube(s, '/World/ElasticTarget/Stem_MainStem_02', (5, 0, 0))
        self.assertEqual(obstacle_class(main, m, 'Truss_02'), ('main_stem', True))

    def test_own_truss_permission_is_tool_only_never_arm_or_carrier(self):
        for xyz, required in (([0, .5, 0], '/Robot/world/shape'),
                              ([.2, .5, 0], '/S300Carrier/shape')):
            s, m, p = fixture()
            cube(s, '/World/ElasticTarget/Truss_02_Fruit_01', xyz)
            report, _ = run(s, m, p)
            self.assertFalse(report['passed']); self.assertTrue(report['complete'])
            self.assertTrue(any(required in row['robot_geom'] and
                row['obstacle_class'] == 'current_truss_robot_or_chassis' for row in report['hard_hits']))

    def test_instance_proxy_obstacle_and_exact_pair_filter_are_covered(self):
        s, m, p = fixture()
        source = Usd.Stage.CreateInMemory()
        cube(source, '/Prototype', (-.042, .5, 0), .01)
        root = UsdGeom.Xform.Define(s, '/World/Greenhouse/House/GutterInstance').GetPrim()
        root.GetReferences().AddReference(source.GetRootLayer().identifier, '/Prototype')
        root.SetInstanceable(True)
        shape_path = str(root.GetPath())+'/shape'
        self.assertTrue(s.GetPrimAtPath(shape_path).IsInstanceProxy())
        report, _ = run(s, m, p)
        self.assertFalse(report['passed'])
        self.assertTrue(any(row['environment_geom'] == shape_path for row in report['hard_hits']))
        UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath('/World/HarvestRobot/Robot/world')).CreateFilteredPairsRel().SetTargets([root.GetPath()])
        report, _ = run(s, m, p)
        self.assertTrue(report['passed']); self.assertGreater(report['filtered_sample_pairs'], 0)

    def test_exact_body_filter_keeps_unrelated_obstacle_active(self):
        s, m, p = fixture(); blocked = cube(s, '/World/Greenhouse/House/Gutter', (-.042, .5, 0), .01)
        UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath('/World/HarvestRobot/Robot/world')).CreateFilteredPairsRel().SetTargets([blocked.GetPath().GetParentPath()])
        report, _ = run(s, m, p)
        self.assertTrue(report['passed']); self.assertGreater(report['filtered_sample_pairs'], 0)
        cube(s, '/World/Greenhouse/House/Gutter_other', (-.042, .7, 0), .01)
        report, _ = run(s, m, p)
        self.assertFalse(report['passed']); self.assertTrue(report['hard_hits'][0]['environment_geom'].endswith('Gutter_other/shape'))

    def test_ground_support_is_reported_not_silently_exempted(self):
        s, m, p = fixture()
        floor = cube(s, '/World/Greenhouse/House/Environment/GroundCover_1_1/Mesh', (-.042, .5, 0), .01)
        report, _ = run(s, m, p)
        self.assertFalse(report['passed']); self.assertTrue(report['complete'])
        self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'structural_ground_support')
        UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath('/World/HarvestRobot/Robot/world')).CreateFilteredPairsRel().SetTargets([floor.GetPath()])
        report, _ = run(s, m, p)
        self.assertTrue(report['passed']); self.assertGreater(report['filtered_sample_pairs'], 0)

    def test_collision_group_collections_use_exact_include_exclude_policy(self):
        s, m, p = fixture(); shape = cube(s, '/World/Greenhouse/House/Gutter', (-.042, .5, 0), .01)
        a = UsdPhysics.CollisionGroup.Define(s, '/World/Groups/Robot')
        b = UsdPhysics.CollisionGroup.Define(s, '/World/Groups/Gutter')
        a.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/HarvestRobot/Robot/world'])
        b.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([shape.GetPath()])
        a.CreateFilteredGroupsRel().SetTargets([b.GetPath()])
        report, _ = run(s, m, p); self.assertTrue(report['passed'])
        b.GetCollidersCollectionAPI().CreateExcludesRel().SetTargets([shape.GetPath()])
        report, _ = run(s, m, p); self.assertFalse(report['passed'])

    def test_checkpoint_owner_deformation_used_instead_of_authored_rest(self):
        s, m, p = fixture(); body = '/World/Greenhouse/Plants/Plant_61/Truss_00_Fruit_01'
        cube(s, body, (0, .5, 0), rigid=True)
        report, _ = run(s, m, p); self.assertFalse(report['passed'])
        measured = np.eye(4); measured[1, 3] = 4.
        moving = MotionSelfCollision(s, Kinematics())
        adapter = RailEnvironmentCheck(s, m, moving, allowed_truss_key='Truss_02', checkpoint_poses={body:measured})
        # Test only the owner transform loader; full start parity still requires
        # base/tool pose proof, which is supplied here from the actual FK.
        cp = moving.poses(np.zeros(7)); cp[body] = measured
        report = screen_rail_plan(adapter, p, m, checkpoint_poses=cp)
        self.assertTrue(report['passed'])
        self.assertEqual(adapter.rows[0]['owner_pose_source'], 'measured_checkpoint')

    def test_closed_triangle_volume_containment_does_not_escape_surface_checker(self):
        s, m, p = fixture(); mesh = trimesh.creation.box(extents=[.5, .5, .5])
        shape = UsdGeom.Mesh.Define(s, '/World/Greenhouse/House/GutterSolid')
        shape.CreatePointsAttr(mesh.vertices.tolist()); shape.CreateFaceVertexCountsAttr([3]*len(mesh.faces))
        shape.CreateFaceVertexIndicesAttr(mesh.faces.flatten().tolist())
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr('none')
        report, _ = run(s, m, p)
        self.assertFalse(report['passed']); self.assertEqual(report['hard_hits'][0]['time_s'], 0)

    def test_axis_rotated_capsule_and_sphere_source_shapes(self):
        s, m, p = fixture()
        cap = UsdGeom.Capsule.Define(s, '/World/Greenhouse/Plants/Plant_61/Rachis')
        cap.CreateRadiusAttr(.01); cap.CreateHeightAttr(.2); cap.CreateAxisAttr('X')
        cap.AddTranslateOp().Set(Gf.Vec3d(0, .5, 0)); UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
        report, _ = run(s, m, p); self.assertFalse(report['passed'])
        cap.GetPrim().SetActive(False)
        sphere = UsdGeom.Sphere.Define(s, '/World/Greenhouse/Plants/Plant_61/Fruit_01')
        sphere.CreateRadiusAttr(.05); sphere.AddTranslateOp().Set(Gf.Vec3d(0, .5, 0))
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        report, _ = run(s, m, p); self.assertFalse(report['passed'])
        self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'foreign_fruit')

    def test_origin_double_shift_rejected_without_fake_realigning(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Far', (10, 0, 0))
        moving = MotionSelfCollision(s, Kinematics())
        correct = moving.poses(np.zeros(7), rail_position_m=.5)
        p['resume_rail_origin'] = dict(schema='s300-measured-resume-rail-origin-v1',
                                     axis_world=[0., 1., 0.], displacement_m=.5)
        p['phases'][0]['rail_waypoint_positions_m'] = [.5, 1.]
        proof = verify_initial_alignment(moving, p, m, correct)
        self.assertTrue(proof['geometry_shift_applied_once'])
        wrong = {k:v.copy() for k,v in correct.items()}
        for t in wrong.values(): t[1, 3] += .5
        with self.assertRaisesRegex(ValueError, 'rail displacement twice'):
            verify_initial_alignment(moving, p, m, wrong)

    def test_checkpoint_must_contain_carrier_arm_and_jaw_poses(self):
        s, m, p = fixture()
        moving = MotionSelfCollision(s, Kinematics()); actual = moving.poses(np.zeros(7))
        del actual['/World/HarvestRobot/Robot/S300Carrier']
        with self.assertRaisesRegex(ValueError, 'complete robot/carrier/tool'):
            verify_initial_alignment(moving, p, m, actual)

    def test_native_drive_target_is_not_used_for_geometric_rail_position(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Gutter', (0, .5, 0))
        p['phases'][0]['rail_native_command_positions_m'] = [10., 11.]
        report, _ = run(s, m, p); self.assertFalse(report['passed'])
        self.assertTrue(all(0<=h['rail_position_m']<=1 for h in report['hard_hits']))

    def test_budget_exhaustion_never_becomes_clear(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Gutter', (0, 0, 0))
        report, _ = run(s, m, p, max_distance_queries=1)
        self.assertFalse(report['passed']); self.assertFalse(report['complete'])
        self.assertEqual(report['status'], 'blocked'); self.assertIn('budget', report['error'])

    def test_unsupported_or_nan_geometry_fails_before_native_fcl(self):
        s, m, _ = fixture()
        cone = UsdGeom.Cone.Define(s, '/World/Greenhouse/House/Unsupported')
        UsdPhysics.CollisionAPI.Apply(cone.GetPrim())
        with self.assertRaisesRegex(ValueError, 'Unsupported enabled collider'):
            RailEnvironmentCheck(s, m, MotionSelfCollision(s, Kinematics()), allowed_truss_key='Truss_02')
        with self.assertRaisesRegex(ValueError, 'Finite source collider'):
            collider_geometry(cone.GetPrim(), np.full((4, 4), np.nan))

    def test_no_rail_is_not_applicable_and_midplan_rail_is_inconclusive(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Far', (10, 0, 0))
        self.assertEqual(screen_rail_plan(None, dict(phases=[]), m)['status'], 'not_applicable')
        p['phases'].insert(0, dict(name='prepare', duration_s=1, waypoint_q=[[0.]*7]*2, end_gap_m=.044))
        report, _ = run(s, m, p); self.assertEqual(report['status'], 'inconclusive')

    def test_absent_rail_returns_before_model_or_geometry_loading(self):
        with tempfile.TemporaryDirectory() as directory:
            plan = Path(directory)/'plan.json'; plan.write_text(json.dumps(dict(phases=[])))
            with patch('rail_environment_preflight.verify_compact_dependencies') as dependency, \
                    patch('rail_environment_preflight.RailEnvironmentCheck') as adapter:
                report = audit_rail_transit('/missing_model_should_not_be_opened', plan)
            self.assertEqual(report['status'], 'not_applicable'); self.assertFalse(report['passed'])
            self.assertFalse(report['complete']); self.assertFalse(report['geometry_loaded'])
            dependency.assert_not_called(); adapter.assert_not_called()

    def test_absent_rail_does_not_open_optional_native_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            plan=Path(directory)/'plan.json';plan.write_text(json.dumps(dict(phases=[])))
            with patch('rail_environment_preflight.MotionSelfCollision') as moving:
                report=audit_rail_transit('/missing_model',plan,native_tool_cache='/missing_cache')
            self.assertEqual(report['status'],'not_applicable');moving.assert_not_called()

    def test_environment_native_cache_is_opt_in_and_bound_to_current_assembly(self):
        s,m,p=fixture();cube(s,'/World/Greenhouse/House/Far',(10,0,0))
        with tempfile.TemporaryDirectory() as directory:
            assembly=Path(directory);s.Export(str(assembly/'scene.usda'));m['scene_sha256']=file_sha256(assembly/'scene.usda')
            (assembly/'motion_manifest.json').write_text(json.dumps(m));plan=assembly/'plan.json';plan.write_text(json.dumps(p))
            cache=assembly/'cache.json';cache.write_text('synthetic passthrough fixture, not native proof')
            moving=MotionSelfCollision(s,Kinematics());moving.metadata['native_tool_hull_import']=dict(native_piece_count=2)
            with patch('motion_kinematics.ToolKinematics',return_value=Kinematics()), \
                    patch('rail_environment_preflight.MotionSelfCollision',return_value=moving) as constructor:
                result=audit_rail_transit(assembly,plan,native_tool_cache=cache)
            self.assertTrue(result['complete']);self.assertTrue(result['passed'])
            self.assertEqual(constructor.call_args.kwargs,dict(native_tool_cache=cache,assembly=assembly.resolve()))
            self.assertEqual(result['native_tool_cache']['sha256'],file_sha256(cache))
            self.assertEqual(result['native_tool_hull_import']['native_piece_count'],2)
            # Cache identity failure can never become an environment PASS.
            with patch('motion_kinematics.ToolKinematics',return_value=Kinematics()):
                invalid=audit_rail_transit(assembly,plan,native_tool_cache=cache)
            self.assertFalse(invalid['complete']);self.assertFalse(invalid['passed']);self.assertEqual(invalid['status'],'inconclusive')

    def test_native_cache_mutation_during_environment_screen_is_inconclusive(self):
        s,m,p=fixture();cube(s,'/World/Greenhouse/House/Far',(10,0,0))
        with tempfile.TemporaryDirectory() as directory:
            assembly=Path(directory);s.Export(str(assembly/'scene.usda'));m['scene_sha256']=file_sha256(assembly/'scene.usda')
            (assembly/'motion_manifest.json').write_text(json.dumps(m));plan=assembly/'plan.json';plan.write_text(json.dumps(p))
            cache=assembly/'cache.json';cache.write_text('exact cache fixture')
            moving=MotionSelfCollision(s,Kinematics());moving.metadata['native_tool_hull_import']=dict(native_piece_count=2)
            def screen_then_mutate(*a,**kw):
                result=screen_rail_plan(*a,**kw);cache.write_text('changed native cache');return result
            with patch('motion_kinematics.ToolKinematics',return_value=Kinematics()), \
                    patch('rail_environment_preflight.MotionSelfCollision',return_value=moving), \
                    patch('rail_environment_preflight.screen_rail_plan',side_effect=screen_then_mutate):
                result=audit_rail_transit(assembly,plan,native_tool_cache=cache)
            self.assertFalse(result['passed']);self.assertFalse(result['complete'])
            self.assertIn('cache changed',result['error'])

    def test_external_native_cache_flag_is_absent_by_default_and_preserved_when_requested(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            for i,cache in enumerate((None,'/exact/current-model-native-cache.json')):
                output=Path(directory)/f'report{i}.json'
                def publish(command,**kwargs):
                    output.write_text(json.dumps(dict(complete=True,passed=True,status='passed')))
                    return SimpleNamespace(returncode=0,stdout='',stderr='')
                with patch('subprocess.run',side_effect=publish) as process:
                    result=audit_external('/assembly',output,plan='/plan',native_tool_cache=cache)
                self.assertTrue(result['passed']);command=process.call_args.args[0]
                self.assertEqual('--native-tool-cache'in command,cache is not None)
                if cache is not None:self.assertEqual(command[command.index('--native-tool-cache')+1],cache)

    def test_existing_self_sampler_default_commands_unchanged(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Far', (10, 0, 0))
        moving = MotionSelfCollision(s, Kinematics(), clearance_m=.002)
        before = check_plan(moving, copy.deepcopy(p)); arrays = copy.deepcopy(moving.pairs)
        adapter = RailEnvironmentCheck(s, m, moving, allowed_truss_key='Truss_02')
        report = screen_rail_plan(adapter, p, m)
        self.assertTrue(report['passed']); self.assertEqual(moving.clearance, .002)
        np.testing.assert_array_equal(moving.pairs, arrays)
        after = check_plan(moving, copy.deepcopy(p))
        self.assertEqual(before, after)

    def test_source_and_plan_model_sha_mismatch_fail_closed(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Far', (10, 0, 0))
        with tempfile.TemporaryDirectory() as directory:
            assembly = Path(directory); scene = assembly/'scene.usda'; s.Export(str(scene))
            m['scene_sha256'] = '0'*64; (assembly/'motion_manifest.json').write_text(json.dumps(m))
            plan = assembly/'plan.json'; plan.write_text(json.dumps(p))
            result = audit_rail_transit(assembly, plan); self.assertFalse(result['passed'])
            self.assertIn('Source scene SHA mismatch', result['error'])
            m['scene_sha256'] = file_sha256(scene); (assembly/'motion_manifest.json').write_text(json.dumps(m))
            p['model_sha256'] = '0'*64; plan.write_text(json.dumps(p))
            result = audit_rail_transit(assembly, plan); self.assertIn('Plan/model SHA mismatch', result['error'])

    def test_same_wrapper_sha_different_materialized_content_requires_declared_handoff(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); old = root/'old'; old.mkdir(); new = root/'new'; new.mkdir()
            cp = root/'cp'; cp.mkdir()
            (cp/'checkpoint.json').write_text('{}'); (cp/'checkpoint.npz').write_bytes(b'fixture')
            metadata = dict(scene_sha256='a'*64, metadata_sha256='c'*64, source_assembly=str(old),
                manifest=dict(plant=dict(root='/World/ElasticTarget'), compact_assets=dict(
                    materialized_dependency=dict(sha256='1'*64))), **completed())
            metadata['break_events'] = [dict(joint='/World/ElasticTarget/HarvestJoints/Truss_02_Fruit_01', t=.8)]
            names = ['farmily_lift_height_joint', 'base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
            arrays = dict(body_paths=np.array(['/World/ElasticTarget/Truss_02_Fruit_01']),
                body_poses_world=np.array([[0,0,0,0,0,0,1.]]), robot_dof_names=np.array(names), robot_q=np.arange(7.))
            manifest = dict(scene_sha256='a'*64, compact_assets=dict(materialized_dependency=dict(sha256='2'*64)))
            with patch('native_route_checkpoint.load_checkpoint', return_value=(metadata, arrays)):
                with self.assertRaisesRegex(ValueError, 'outside exact declared'):
                    start_checkpoint(new, manifest, cp)
                manifest['route_checkpoint'] = dict(checkpoint=str(cp), source_scene_sha256='a'*64,
                    outgoing=dict(checkpoint_metadata_sha256='c'*64, frozen_root='/World/FrozenHarvestHistory/Plant_22'))
                poses, proof = start_checkpoint(new, manifest, cp)
                self.assertEqual(proof['model_relation'], 'exact declared active-window handoff')
                self.assertEqual(proof['source_materialized_dependency_sha256'], '1'*64)
                self.assertEqual(proof['current_materialized_dependency_sha256'], '2'*64)
                self.assertEqual(proof['native_break_events'], [])
                self.assertNotIn('/World/ElasticTarget/Truss_02_Fruit_01', poses)
                self.assertIn('/World/FrozenHarvestHistory/Plant_22/Truss_02_Fruit_01', poses)
                np.testing.assert_array_equal(proof['initial_lift_arm_q'], np.arange(7.))

    def test_same_model_checkpoint_different_assembly_is_not_silently_accepted(self):
        manifest = dict(scene_sha256='a'*64)
        metadata = dict(scene_sha256='a'*64, source_assembly='/different', manifest={}, **completed())
        with patch('native_route_checkpoint.load_checkpoint', return_value=(metadata, {})):
            with self.assertRaisesRegex(ValueError, 'assembly identity mismatch'):
                start_checkpoint('/current', manifest, '/cp')

    def test_stopped_or_ordinary_idle_checkpoint_is_not_a_valid_rail_parent(self):
        for sequence in (None, dict(checkpoint_capture_reason='ordinary_idle', checkpoint_physics_valid=True),
                         dict(completed()['sequence'], checkpoint_physics_valid=False)):
            metadata = dict(scene_sha256='a'*64, source_assembly='/current', manifest={}, sequence=sequence)
            with patch('native_route_checkpoint.load_checkpoint', return_value=(metadata, {})):
                with self.assertRaisesRegex(ValueError, 'checkpoint|completed_cycle'):
                    start_checkpoint('/current', dict(scene_sha256='a'*64), '/cp')

    def test_mutated_source_manifest_during_screen_is_inconclusive_not_passed(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Far', (10, 0, 0))
        with tempfile.TemporaryDirectory() as directory:
            assembly = Path(directory); s.Export(str(assembly/'scene.usda'))
            m['scene_sha256'] = file_sha256(assembly/'scene.usda')
            manifest = assembly/'motion_manifest.json'; manifest.write_text(json.dumps(m))
            plan = assembly/'plan.json'; plan.write_text(json.dumps(p))
            def screen_then_mutate(*args, **kwargs):
                result = screen_rail_plan(*args, **kwargs)
                manifest.write_text(json.dumps(dict(m, unrelated_revision=True)))
                return result
            with patch('motion_kinematics.ToolKinematics', return_value=Kinematics()), \
                    patch('rail_environment_preflight.screen_rail_plan', side_effect=screen_then_mutate):
                result = audit_rail_transit(assembly, plan)
            self.assertFalse(result['passed']); self.assertFalse(result['complete'])
            self.assertEqual(result['status'], 'inconclusive')
            self.assertIn('changed during rail preflight', result['error'])

    def test_checkpoint_joint_start_mismatch_is_rejected_before_planning(self):
        s, m, p = fixture(); cube(s, '/World/Greenhouse/House/Far', (10, 0, 0))
        with tempfile.TemporaryDirectory() as directory:
            assembly = Path(directory); s.Export(str(assembly/'scene.usda'))
            m['scene_sha256'] = file_sha256(assembly/'scene.usda')
            (assembly/'motion_manifest.json').write_text(json.dumps(m))
            plan = assembly/'plan.json'; plan.write_text(json.dumps(p))
            with patch('rail_environment_preflight.start_checkpoint', return_value=(None, dict(initial_lift_arm_q=[.1]*7))):
                result = audit_rail_transit(assembly, plan)
            self.assertFalse(result['passed']); self.assertFalse(result['complete'])
            self.assertIn('exact measured checkpoint', result['error'])

    def test_external_boundary_rejects_blocked_or_incomplete_reports(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            for index, report in enumerate((dict(status='blocked', passed=False, complete=True),
                           dict(status='inconclusive', passed=False, complete=False),
                           dict(status='sampled_clear', passed=True, complete=False))):
                output = Path(directory)/('audit_'+str(index)+'.json')
                def publish(*args, **kwargs):
                    output.write_text(json.dumps(report))
                    return SimpleNamespace(returncode=2, stdout='raw stdout', stderr='raw stderr')
                with patch('subprocess.run', side_effect=publish) as process:
                    with self.assertRaisesRegex(RuntimeError, 'blocked commanded travel'):
                        audit_external('/assembly', output, plan='/actual_plan', checkpoint='/actual_cp')
                self.assertEqual(output.with_suffix('.log').read_text(), 'raw stdoutraw stderr')
                command = process.call_args.args[0]
                self.assertIn('--checkpoint', command); self.assertIn('/actual_cp', command)
                self.assertIn('--plan', command); self.assertIn('/actual_plan', command)

    def test_external_launch_error_or_missing_report_never_becomes_clear(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            for index, code in enumerate((1, 0, 2)):
                output = Path(directory)/('launch_'+str(index)+'.json')
                with patch('subprocess.run', return_value=SimpleNamespace(returncode=code, stdout='', stderr='failure')):
                    with self.assertRaisesRegex(RuntimeError, 'could not complete'):
                        audit_external('/assembly', output, plan='/plan')

    def test_prior_report_cannot_be_overwritten_or_accepted_as_stale_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'audit.json'; raw = '{"passed":true,"complete":true}'
            output.write_text(raw)
            with patch('subprocess.run') as process:
                with self.assertRaisesRegex(FileExistsError, 'new rail preflight output'):
                    audit_external('/assembly', output, plan='/plan')
            process.assert_not_called(); self.assertEqual(output.read_text(), raw)

    def test_external_collection_option_is_forwarded_and_default_absent(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            for enabled in (False, True):
                output = Path(directory)/('option_'+str(enabled)+'.json')
                def publish(*args, **kwargs):
                    output.write_text(json.dumps(dict(status='sampled_clear', passed=True, complete=True)))
                    return SimpleNamespace(returncode=0, stdout='', stderr='')
                with patch('subprocess.run', side_effect=publish) as process:
                    report = audit_external('/assembly', output, plan='/plan', released_fruit_collection_diagnostic=enabled)
                self.assertTrue(report['passed'])
                self.assertEqual('--released-fruit-collection-diagnostic' in process.call_args.args[0], enabled)

    def test_support_policy_separate_external_forwarding(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            for enabled in (False, True):
                output=Path(directory)/('support_'+str(enabled)+'.json')
                def publish(*args,**kwargs):
                    output.write_text(json.dumps(dict(status='sampled_clear',passed=True,complete=True)))
                    return SimpleNamespace(returncode=0,stdout='',stderr='')
                with patch('subprocess.run',side_effect=publish) as process:
                    audit_external('/assembly',output,plan='/plan',released_fruit_gripper_support_diagnostic=enabled)
                self.assertEqual('--released-fruit-gripper-support-diagnostic' in process.call_args.args[0],enabled)
                self.assertNotIn('--released-fruit-collection-diagnostic',process.call_args.args[0])

    def test_same_truss_policy_separate_external_forwarding(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            for enabled in (False,True):
                output=Path(directory)/('truss_'+str(enabled)+'.json')
                def publish(*args,**kwargs):
                    output.write_text(json.dumps(dict(status='sampled_clear',passed=True,complete=True)))
                    return SimpleNamespace(returncode=0,stdout='',stderr='')
                with patch('subprocess.run',side_effect=publish)as process:
                    audit_external('/assembly',output,plan='/plan',released_fruit_same_truss_diagnostic=enabled)
                self.assertEqual('--released-fruit-same-truss-diagnostic'in process.call_args.args[0],enabled)
                self.assertNotIn('--released-fruit-collection-diagnostic',process.call_args.args[0])
                self.assertNotIn('--released-fruit-gripper-support-diagnostic',process.call_args.args[0])

    def test_exact_native_released_fruit_passive_carrier_is_optional_diagnostic(self):
        s, m, p, proofs, native = release_fixture()
        strict, _ = run(s, m, p); self.assertFalse(strict['passed'])
        with patch('passive_collection_surfaces.inventory', return_value=proofs):
            context = verified_release_context(s, m, native)
        report, _ = run(s, m, p, release_context=context)
        self.assertTrue(report['passed']); self.assertTrue(report['complete'])
        self.assertGreater(report['released_fruit_collection_diagnostic_sample_pairs'], 0)
        evidence = report['diagnostic_only_hits'][0]['released_fruit_collection_evidence']
        self.assertTrue(evidence['native_release_verified']); self.assertTrue(evidence['continuation_authorized'])
        self.assertFalse(evidence['collection_verified']); self.assertFalse(evidence['released_fruit_physics_verified'])
        self.assertIsNone(report['strict_whole_scene_physics_passed'])
        self.assertIn('not actual native contact', evidence['contact_source'])

    def test_exact_existing_flat_rear_deck_is_eligible_but_clones_are_not(self):
        for suffix, expected in (('FlatRearDeck', True), ('UnapprovedFlatRearDeck', False)):
            s, m, p, proofs, native = release_fixture((.2, .5, .2))
            path = '/World/HarvestRobot/Robot/S300Carrier/S300Chassis/'+suffix
            deck = UsdGeom.Cube.Define(s, path); deck.CreateSizeAttr(.1)
            deck.AddTranslateOp().Set(Gf.Vec3d(0, 0, .2))
            UsdPhysics.CollisionAPI.Apply(deck.GetPrim()).CreateCollisionEnabledAttr(True)
            with patch('passive_collection_surfaces.inventory', return_value=proofs):
                context = verified_release_context(s, m, native)
            report, _ = run(s, m, p, release_context=context)
            self.assertEqual(report['passed'], expected)
            self.assertEqual(path in context['passive_colliders'], expected)
        s, m, p, proofs, native = release_fixture((0, .65, .2))
        path = '/World/HarvestRobot/Robot/arm/S300Chassis/FlatRearDeck'
        deck = UsdGeom.Cube.Define(s, path); deck.CreateSizeAttr(.1)
        deck.AddTranslateOp().Set(Gf.Vec3d(0, 0, .2))
        UsdPhysics.CollisionAPI.Apply(deck.GetPrim()).CreateCollisionEnabledAttr(True)
        with patch('passive_collection_surfaces.inventory', return_value=proofs):
            context = verified_release_context(s, m, native)
        report, _ = run(s, m, p, release_context=context)
        self.assertFalse(report['passed']); self.assertNotIn(path, context['passive_colliders'])

    def test_exact_native_collection_deck_with_wrong_rigid_owner_is_not_eligible(self):
        s, m, p, proofs, native = release_fixture((.2, .5, .2))
        path = '/World/HarvestRobot/Robot/S300Carrier/S300Chassis/FlatRearDeck'
        deck = UsdGeom.Cube.Define(s, path); deck.CreateSizeAttr(.1)
        deck.AddTranslateOp().Set(Gf.Vec3d(0, 0, .2))
        UsdPhysics.RigidBodyAPI.Apply(deck.GetPrim())
        UsdPhysics.CollisionAPI.Apply(deck.GetPrim()).CreateCollisionEnabledAttr(True)
        with patch('passive_collection_surfaces.inventory', return_value=proofs):
            context = verified_release_context(s, m, native)
        report, _ = run(s, m, p, release_context=context)
        self.assertFalse(report['passed']); self.assertNotIn(path, context['passive_colliders'])

    def test_pair_aggregation_preserves_late_hits_beyond_detail_limit(self):
        s, m, p = fixture()
        cube(s, '/World/Greenhouse/House/Gutter_early', (-.042, .12, 0), .01)
        cube(s, '/World/Greenhouse/House/Gutter_late', (-.042, .87, 0), .01)
        report, _ = run(s, m, p)
        self.assertTrue(report['complete']); self.assertFalse(report['passed'])
        pairs = report['all_hard_pair_counts']
        self.assertEqual({h['environment_geom'] for h in pairs}, {
            '/World/Greenhouse/House/Gutter_early/shape', '/World/Greenhouse/House/Gutter_late/shape'})
        self.assertEqual(sum(h['sample_pairs'] for h in pairs), report['hard_intersection_sample_pairs'])
        self.assertLess(pairs[0]['first_time_s'], pairs[1]['first_time_s'])

    def test_attached_fruit_arm_tool_and_unapproved_carrier_part_remain_hard(self):
        # Attached fruit never enters the same exact carrier-release policy.
        s, m, p, proofs, _ = release_fixture()
        with patch('passive_collection_surfaces.inventory', return_value=proofs):
            context = verified_release_context(s, m, dict(native_break_events=[]))
        report, _ = run(s, m, p, release_context=context); self.assertFalse(report['passed'])
        for xyz in ((0, .65, 0), (3, .65, 0)):
            s, m, p, proofs, native = release_fixture(xyz)
            with patch('passive_collection_surfaces.inventory', return_value=proofs):
                context = verified_release_context(s, m, native)
            report, _ = run(s, m, p, release_context=context)
            self.assertFalse(report['passed']); self.assertEqual(report['released_fruit_collection_diagnostic_sample_pairs'], 0)
            self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'released_fruit_noncollection_contact')
        s, m, p, proofs, native = release_fixture((.2, .5, .2))
        cube(s, '/World/HarvestRobot/Robot/S300Carrier/UnapprovedPart', (0, 0, .2), .1)
        with patch('passive_collection_surfaces.inventory', return_value=proofs):
            context = verified_release_context(s, m, native)
        report, _ = run(s, m, p, release_context=context)
        self.assertFalse(report['passed']); self.assertTrue(any('/UnapprovedPart/' in h['robot_geom'] for h in report['hard_hits']))

    def test_unmapped_native_break_or_wrong_joint_relation_never_grants_permission(self):
        s, m, p, proofs, native = release_fixture()
        with patch('passive_collection_surfaces.inventory', return_value=proofs):
            wrong = copy.deepcopy(native); wrong['native_break_events'][0]['joint'] += '_other'
            with self.assertRaisesRegex(ValueError, 'unknown active fruit'):
                verified_release_context(s, m, wrong)
            attachment = UsdPhysics.Joint(s.GetPrimAtPath(m['plant']['fruits'][0]['joint']))
            attachment.GetBody1Rel().SetTargets(['/World/OtherFruit'])
            with self.assertRaisesRegex(ValueError, 'body mapping mismatch'):
                verified_release_context(s, m, native)

    def test_frozen_prior_release_does_not_alias_new_attached_active_fruit(self):
        s, m, p, proofs, _ = release_fixture((5, .5, 0))
        original = copy.deepcopy(m['plant']['fruits'][0]); source = '/World/Greenhouse/Plants/Plant_22'
        frozen_root = '/World/RouteFrozen/Plant_22'; name = original['name']
        body = frozen_root+'/'+name; joint_path = frozen_root+'/HarvestJoints/'+name
        f = UsdGeom.Xform.Define(s, body); f.AddTranslateOp().Set(Gf.Vec3d(.2, .5, 0))
        UsdPhysics.RigidBodyAPI.Apply(f.GetPrim())
        sphere = UsdGeom.Sphere.Define(s, body+'/Collider'); sphere.CreateRadiusAttr(.035)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim()).CreateCollisionEnabledAttr(True)
        joint = UsdPhysics.FixedJoint.Define(s, joint_path); joint.CreateJointEnabledAttr(False)
        events = [dict(joint=original['joint'], t=.8, type='native_joint_break')]
        metadata = dict(source_plant=source, scene_sha256='a'*64, metadata_sha256='b'*64,
            manifest=dict(plant=dict(root=m['plant']['root'], fruits=[original])),
            break_events=events, **completed())
        release = dict(name=name, path=body, joint=joint_path, original_joint=original['joint'],
            source_plant=source, route_identity='Plant_22/'+name, source_scene_sha256='a'*64,
            checkpoint_metadata_sha256='b'*64, checkpoint_time_s=1., native_break_events=events)
        m['route_checkpoint'] = dict(outgoing_history=[dict(checkpoint='/verified/old/checkpoint',
            source_plant=source, source_scene_sha256='a'*64, checkpoint_metadata_sha256='b'*64,
            checkpoint_time_s=1., frozen_root=frozen_root, detached_fruits=[release])])
        with patch('passive_collection_surfaces.inventory', return_value=proofs), \
                patch('native_route_checkpoint.load_checkpoint', return_value=(metadata, {})):
            context = verified_release_context(s, m, dict(native_break_events=[]))
        report, _ = run(s, m, p, release_context=context)
        self.assertTrue(report['passed']); self.assertGreater(report['released_fruit_collection_diagnostic_sample_pairs'], 0)
        self.assertTrue(all(h['environment_geom'] == body+'/Collider' for h in report['diagnostic_only_hits']))
        # Identical Truss/Fruit names in the newly active plant cannot borrow
        # the old frozen fruit's actual native release proof.
        UsdGeom.Xform(s.GetPrimAtPath(original['path'])).GetOrderedXformOps()[0].Set(Gf.Vec3d(.2, .5, 0))
        report, _ = run(s, m, p, release_context=context)
        self.assertFalse(report['passed'])
        self.assertTrue(any(h['environment_geom'] == original['path']+'/Collider' for h in report['hard_hits']))


class EnvironmentStateQueryTests(unittest.TestCase):
    def adapter(self, stage, manifest, **kwargs):
        return RailEnvironmentCheck(stage, manifest, MotionSelfCollision(stage, Kinematics()),
            allowed_truss_key='Truss_02', scope='preparation', **kwargs)

    def test_exploratory_state_uses_full_inventory_outside_prior_rail_sweep(self):
        stage, manifest, plan = fixture()
        obstacle = cube(stage, '/World/Greenhouse/House/Gutter', (0, 2., 0))
        adapter = self.adapter(stage, manifest)
        adapter.prepare_sweep(plan['phases'][0], 120)
        self.assertEqual(len(adapter.candidate_indices), 0)
        sampled_context = (adapter.sample_index, adapter.current_time_s, adapter.current_phase,
                           adapter.candidate_indices.copy(), copy.deepcopy(adapter.hit_pairs))
        report = adapter.check_state(np.zeros(7), .044, rail_position_m=2.)
        self.assertTrue(report['complete']); self.assertFalse(report['hard_clear'])
        self.assertTrue(report['full_environment_inventory'])
        self.assertEqual(report['hard_hits'][0]['environment_geom'], str(obstacle.GetPath()))
        self.assertIsNone(report['hard_hits'][0]['time_s'])
        self.assertEqual((adapter.sample_index, adapter.current_time_s, adapter.current_phase), sampled_context[:3])
        np.testing.assert_array_equal(adapter.candidate_indices, sampled_context[3])
        self.assertEqual(adapter.hit_pairs, sampled_context[4]); self.assertEqual(adapter.total_hits, 0)
        clear = adapter.check_state(np.zeros(7), .044, rail_position_m=0.)
        self.assertTrue(clear['complete']); self.assertTrue(clear['hard_clear'])
        self.assertEqual(clear['hard_hits'], [])
        self.assertTrue(clear['final_path_audit_required'])

    def test_state_queries_do_not_consume_prefix_schedule_or_ledger(self):
        stage, manifest, _ = fixture()
        cube(stage, '/World/Greenhouse/House/Gutter', (0, 0, 0))
        plan = PreparationEnvironmentTests().plan()
        reference = self.adapter(stage, manifest)
        expected = screen_preparation_plan(reference, plan, manifest)
        adapter = self.adapter(stage, manifest)
        adapter.check_state(np.zeros(7), .044, rail_position_m=0.)
        actual = screen_preparation_plan(adapter, plan, manifest)
        # Query count alone includes the extra bounded exploratory query.
        for key in ('complete', 'passed', 'samples', 'hard_hits', 'all_hard_pair_counts',
                    'all_diagnostic_pair_counts', 'hard_intersection_sample_pairs',
                    'filtered_sample_pairs', 'last_sample_time_s', 'sampling'):
            self.assertEqual(actual[key], expected[key], key)
        self.assertEqual(actual['distance_queries'], expected['distance_queries']+1)

    def test_current_whole_aabb_prefilter_matches_full_per_shape_fcl_pairs(self):
        stage, manifest, _ = fixture()
        for index, xyz in enumerate(((0., 0., 0.), (.2, .5, 0.), (3., .15, 0.),
                                     (0., 2., 0.), (3., 2.15, 0.), (20., 30., 0.))):
            cube(stage, '/World/Greenhouse/House/Gutter_'+str(index), xyz, .025)
        for rail in (0., .5, 2., 10.):
            for gap in (.044, .096):
                full = self.adapter(stage, manifest)
                full.duration = 1.; full.intervals = 120
                full.check(np.zeros(7), gap, rail_position_m=rail)
                current = self.adapter(stage, manifest)
                state = current.check_state(np.zeros(7), gap, rail_position_m=rail)
                keys = lambda rows: {(h['robot_geom'], h['environment_geom'], h['obstacle_class']) for h in rows}
                self.assertTrue(state['complete'])
                self.assertEqual(keys(state['hard_hits']), keys(full.hits))
                self.assertEqual(keys(state['diagnostic_only_hits']), keys(full.diagnostics))
                self.assertEqual(state['distance_queries'], full.queries)

    def test_state_query_checks_each_actual_gap_without_reusing_closed_shape(self):
        stage, manifest, _ = fixture()
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath('/World/ActuatedTool/fixed/shape')).GetCollisionEnabledAttr().Set(False)
        cube(stage, '/World/ActuatedTool/L', (3., .15, 0), .005, rigid=True)
        cube(stage, '/World/Greenhouse/House/Gutter', (2.974, .15, 0), .005)
        adapter = self.adapter(stage, manifest)
        narrow = adapter.check_state(np.zeros(7), .044, rail_position_m=0.)
        open_ = adapter.check_state(np.zeros(7), .096, rail_position_m=0.)
        self.assertTrue(narrow['hard_clear']); self.assertTrue(narrow['complete'])
        self.assertFalse(open_['hard_clear']); self.assertTrue(open_['complete'])
        self.assertEqual(open_['hard_hits'][0]['gap_m'], .096)

    def test_static_leaf_and_exact_source_filter_remain_effective_in_state_queries(self):
        stage, manifest, _ = fixture()
        obstacle = cube(stage, '/World/Greenhouse/Plants/Plant_34/Stem/mesh_132',
            (0, 0, 0), source='CompoundLeaf_02')
        adapter = self.adapter(stage, manifest)
        report = adapter.check_state(np.zeros(7), .044, rail_position_m=0.)
        self.assertFalse(report['hard_clear'])
        self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'static_leaf_obstacle')
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/World/HarvestRobot/Robot/world')).CreateFilteredPairsRel().SetTargets([obstacle.GetPath()])
        adapter = self.adapter(stage, manifest)
        report = adapter.check_state(np.zeros(7), .044, rail_position_m=0.)
        self.assertTrue(report['hard_clear']); self.assertGreater(report['filtered_pairs'], 0)
        self.assertEqual(adapter.filtered_pairs, 0)

    def test_state_release_permission_uses_only_checkpoint_history_without_proposal_time(self):
        stage, manifest, _, proofs, native = release_fixture((.2, 0, 0))
        with patch('passive_collection_surfaces.inventory', return_value=proofs):
            context = verified_release_context(stage, manifest, native)
        adapter = self.adapter(stage, manifest, release_context=context)
        report = adapter.check_state(np.zeros(7), .044, rail_position_m=0.)
        self.assertTrue(report['hard_clear']); self.assertTrue(report['complete'])
        hit = report['diagnostic_only_hits'][0]
        self.assertIsNone(hit['time_s'])
        evidence = hit['released_fruit_collection_evidence']
        self.assertIsNone(evidence['proposal_time_s'])
        self.assertEqual(evidence['lifecycle_reference_time_s'], 0.)
        self.assertTrue(evidence['continuation_authorized'])
        self.assertFalse(evidence['collection_verified'])
        # A hypothetical future break must never grant an exploratory state
        # the permission of a measured release already in the checkpoint.
        future = copy.deepcopy(native); future['native_break_events'][0]['t'] = .8
        with patch('passive_collection_surfaces.inventory', return_value=proofs):
            context = verified_release_context(stage, manifest, future)
        adapter = self.adapter(stage, manifest, release_context=context)
        report = adapter.check_state(np.zeros(7), .044, rail_position_m=0.)
        self.assertFalse(report['hard_clear']); self.assertTrue(report['complete'])

    def test_exhausted_state_budget_and_nonfinite_input_are_never_clear(self):
        stage, manifest, _ = fixture()
        cube(stage, '/World/Greenhouse/House/Gutter', (0, 0, 0))
        adapter = self.adapter(stage, manifest, max_distance_queries=1)
        self.assertTrue(adapter.check_state(np.zeros(7), .044, rail_position_m=0.)['complete'])
        report = adapter.check_state(np.zeros(7), .044, rail_position_m=0.)
        self.assertFalse(report['hard_clear']); self.assertFalse(report['complete'])
        self.assertEqual(report['status'], 'inconclusive'); self.assertIn('budget', report['error'])
        for q, gap, rail in (([np.nan]*7, .044, 0), ([0]*6, .044, 0),
                             ([0]*7, np.inf, 0), ([0]*7, .044, np.nan)):
            report = adapter.check_state(q, gap, rail_position_m=rail)
            self.assertFalse(report['hard_clear']); self.assertFalse(report['complete'])


class PreparationEnvironmentTests(unittest.TestCase):
    def test_full_scope_blocks_obstacle_only_in_approach_and_preserves_prefix(self):
        stage, manifest, _ = fixture()
        cube(stage, '/World/Greenhouse/House/ApproachObstacle', (3., .65, 0), .1)
        class ApproachKinematics(Kinematics):
            def fk(self, q):
                transform = super().fk(q); transform[1, 3] += q[0]
                return transform
        plan = self.plan()
        plan['phases'][3]['waypoint_q'] = [[0.]*7, [1.]+[0.]*6]
        plan['phases'][4]['waypoint_q'] = [[1.]+[0.]*6]*2
        original = copy.deepcopy(plan)
        reports = {}
        for scope in ('preparation', 'full'):
            moving = MotionSelfCollision(stage, ApproachKinematics())
            adapter = RailEnvironmentCheck(stage, manifest, moving,
                allowed_truss_key='Truss_02', scope=scope)
            reports[scope] = screen_rail_plan(adapter, plan, manifest,
                scope=scope, hz=60, native_hz=60)
        self.assertTrue(reports['preparation']['passed'])
        self.assertTrue(reports['full']['complete'])
        self.assertFalse(reports['full']['passed'])
        self.assertFalse(reports['full']['eligible_for_native_execution'])
        self.assertEqual(reports['full']['all_hard_pair_counts'][0]['phase_sample_pairs'].keys(), {'approach'})
        self.assertEqual(plan, original)

    def test_full_scope_clear_coarse_result_is_not_native_certificate(self):
        stage, manifest, _ = fixture()
        cube(stage, '/World/Greenhouse/House/Far', (50, 50, 0))
        for hz in (4, 60):
            moving = MotionSelfCollision(stage, Kinematics())
            adapter = RailEnvironmentCheck(stage, manifest, moving,
                allowed_truss_key='Truss_02', scope='full')
            report = screen_rail_plan(adapter, self.plan(), manifest,
                scope='full', hz=hz, native_hz=60)
            self.assertTrue(report['passed']); self.assertTrue(report['complete'])
            self.assertEqual(report['eligible_for_native_execution'], hz == 60)
            self.assertIn('close', report['selected_phases'])

    def plan(self):
        return dict(phases=[dict(name=name, duration_s=duration, waypoint_q=[[0.]*7]*2,
                                end_gap_m=gap)
                    for name, duration, gap in [('ready_hold', .5, .044),
                        ('prepare_joint_branch', 1., .044), ('prepare_open', .25, .096),
                        ('approach', 1., .096), ('close', 1., .028)]])

    def screen(self, stage, manifest, plan=None, hz=120, native_hz=120, **kwargs):
        if not stage.GetPrimAtPath('/World/Greenhouse/House/DistantObstacle'):
            cube(stage, '/World/Greenhouse/House/DistantObstacle', (50., 50., 0.))
        moving = MotionSelfCollision(stage, Kinematics())
        adapter = RailEnvironmentCheck(stage, manifest, moving,
            allowed_truss_key='Truss_02', scope='preparation', **kwargs)
        return screen_preparation_plan(adapter, self.plan() if plan is None else plan, manifest, hz=hz, native_hz=native_hz), adapter

    def test_whole_prefix_before_approach_has_exact_all_phase_timestamps(self):
        stage, manifest, _ = fixture()
        plan = self.plan(); original = copy.deepcopy(plan)
        report, adapter = self.screen(stage, manifest, plan)
        self.assertTrue(report['passed']); self.assertTrue(report['complete'])
        self.assertTrue(report['eligible_for_native_execution'])
        self.assertEqual(report['selected_phases'], ['ready_hold', 'prepare_joint_branch', 'prepare_open'])
        self.assertEqual(report['samples'], 61+121+31)
        self.assertEqual(report['last_sample_time_s'], 1.75)
        self.assertEqual(report['geometry']['swept_bounds_sample_count'], report['samples'])
        self.assertEqual(adapter.sample_index, report['samples'])
        self.assertEqual(plan, original)

    def test_static_foreign_leaf_counterexample_is_hard_only_for_preparation(self):
        stage, manifest, rail = fixture()
        leaf = cube(stage, '/World/Greenhouse/Plants/Plant_34/Stem/mesh_132',
                    (0, .015, 0), source='CompoundLeaf_02')
        report, _ = self.screen(stage, manifest)
        self.assertTrue(report['complete']); self.assertFalse(report['passed'])
        self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'static_leaf_obstacle')
        self.assertEqual(report['hard_hits'][0]['environment_geom'], str(leaf.GetPath()))
        self.assertEqual(report['hard_hits'][0]['phase'], 'ready_hold')
        legacy, _ = run(stage, manifest, rail)
        self.assertTrue(legacy['passed'])
        self.assertEqual(legacy['diagnostic_only_hits'][0]['obstacle_class'], 'soft_leaf')

    def test_exact_declared_dynamic_d6_leaf_is_diagnostic_fixed_or_kinematic_is_not(self):
        stage, manifest, _ = fixture()
        owner = '/World/ElasticTarget/Leaf_CompoundLeaf_02'
        leaf = cube(stage, owner, (0, .015, 0), source='CompoundLeaf_02', rigid=True)
        joint = UsdPhysics.Joint.Define(stage, '/World/ElasticTarget/Joints/Leaf_CompoundLeaf_02')
        joint.CreateBody1Rel().SetTargets([owner])
        manifest['compliant_foliage'] = dict(leaves=[dict(path=owner, joint=str(joint.GetPath()),
            stiffness=.005, meshes=[str(leaf.GetPath())])])
        report, _ = self.screen(stage, manifest)
        self.assertTrue(report['passed']); self.assertGreater(report['diagnostic_intersection_sample_pairs'], 0)
        self.assertEqual(report['diagnostic_only_hits'][0]['obstacle_class'], 'verified_compliant_leaf_diagnostic')
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(owner)).CreateKinematicEnabledAttr(True)
        report, _ = self.screen(stage, manifest)
        self.assertFalse(report['passed'])
        self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'static_or_kinematic_leaf_obstacle')
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(owner)).CreateKinematicEnabledAttr(False)
        stage.RemovePrim(str(joint.GetPath()))
        fixed = UsdPhysics.FixedJoint.Define(stage, str(joint.GetPath()))
        fixed.CreateBody1Rel().SetTargets([owner])
        report, _ = self.screen(stage, manifest)
        self.assertFalse(report['passed'])
        self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'unverified_dynamic_leaf_obstacle')

    def test_declared_additional_leaf_uses_exact_dynamic_d6_proof_not_name_guess(self):
        stage, manifest, _ = fixture()
        owner = '/World/ElasticTarget/Leaf_V9_Additional_CompoundLeaf_07'
        leaf = cube(stage, owner, (0, .015, 0), source='V9_Additional_CompoundLeaf_07_220', rigid=True)
        joint = UsdPhysics.Joint.Define(stage, '/World/ElasticTarget/Joints/Leaf_V9_Additional_CompoundLeaf_07')
        joint.CreateBody1Rel().SetTargets([owner])
        manifest['compliant_foliage'] = dict(leaves=[dict(path=owner, joint=str(joint.GetPath()),
            stiffness=.005, meshes=[str(leaf.GetPath())])])
        report, adapter = self.screen(stage, manifest)
        self.assertTrue(report['passed']); self.assertTrue(report['complete'])
        self.assertEqual(report['diagnostic_only_hits'][0]['obstacle_class'], 'verified_compliant_leaf_diagnostic')
        state = adapter.check_state(np.zeros(7), .044, rail_position_m=0.)
        self.assertTrue(state['hard_clear']); self.assertTrue(state['complete'])
        # A coincident foreign or undeclared mesh cannot borrow that proof.
        foreign = cube(stage, '/World/Greenhouse/Plants/Plant_20/Stem/mesh_220',
                       (0, .015, 0), source='V9_Additional_CompoundLeaf_07_220')
        report, _ = self.screen(stage, manifest)
        self.assertFalse(report['passed'])
        self.assertTrue(any(hit['environment_geom'] == str(foreign.GetPath()) for hit in report['hard_hits']))
        foreign.SetActive(False)
        manifest['compliant_foliage']['leaves'][0]['meshes'] = [str(leaf.GetPath())+'_other']
        report, _ = self.screen(stage, manifest)
        self.assertFalse(report['passed'])
        self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'unverified_dynamic_leaf_obstacle')
    def test_rigid_dynamic_leaf_without_exact_declaration_remains_obstacle(self):
        stage, manifest, _ = fixture()
        cube(stage, '/World/OtherRigidLeaf', (0, .015, 0), source='CompoundLeaf_02', rigid=True)
        report, _ = self.screen(stage, manifest)
        self.assertFalse(report['passed'])
        self.assertEqual(report['hard_hits'][0]['obstacle_class'], 'unverified_dynamic_leaf_obstacle')

    def test_intermediate_joint_arc_is_not_pruned_by_clear_endpoints(self):
        class RotatingKinematics(Kinematics):
            def frames(self, q):
                pose = np.eye(4); pose[:3, 3] = [.4*np.cos(q[1]), .4*np.sin(q[1]), 0]
                return {'/World/HarvestRobot/Robot/arm': pose}
        stage, manifest, _ = fixture()
        UsdGeom.Xform(stage.GetPrimAtPath('/World/HarvestRobot/Robot/arm')).GetOrderedXformOps()[0].Set(Gf.Vec3d(.4, 0, 0))
        cube(stage, '/World/Greenhouse/House/Gutter', (.4, 0, 0), .07)
        first = [0., -.8, 0., 0., 0., 0., 0.]; last = [0., .8, 0., 0., 0., 0., 0.]
        plan = dict(phases=[dict(name='prepare_joint_branch', duration_s=1.,
            waypoint_q=[first, last], end_gap_m=.044),
            dict(name='approach', duration_s=1., waypoint_q=[last, last], end_gap_m=.096)])
        moving = MotionSelfCollision(stage, RotatingKinematics())
        adapter = RailEnvironmentCheck(stage, manifest, moving, allowed_truss_key='Truss_02', scope='preparation')
        report = screen_preparation_plan(adapter, plan, manifest)
        self.assertTrue(report['complete']); self.assertFalse(report['passed'])
        hits = [h for h in report['hard_hits'] if h['robot_geom'].endswith('/arm/shape')]
        self.assertTrue(hits); self.assertGreater(hits[0]['time_s'], 0.)
        self.assertLess(hits[-1]['time_s'], 1.)
        self.assertEqual(report['samples'], 121)

    def test_coarse_diagnostic_does_not_certify_native_execution(self):
        stage, manifest, _ = fixture()
        report, _ = self.screen(stage, manifest, hz=8)
        self.assertTrue(report['complete']); self.assertTrue(report['passed'])
        self.assertFalse(report['strict_native_period_sampling'])
        self.assertFalse(report['eligible_for_native_execution'])
        self.assertEqual(report['samples'], 5+9+3)

    def test_60hz_requires_explicit_matching_native_period(self):
        stage, manifest, _ = fixture()
        diagnostic, _ = self.screen(stage, manifest, hz=60)
        self.assertTrue(diagnostic['passed'])
        self.assertFalse(diagnostic['eligible_for_native_execution'])
        report, _ = self.screen(stage, manifest, hz=60, native_hz=60)
        self.assertTrue(report['passed']); self.assertTrue(report['complete'])
        self.assertTrue(report['strict_native_period_sampling'])
        self.assertTrue(report['eligible_for_native_execution'])
        self.assertEqual(report['samples'],31+61+16)
        self.assertEqual(report['last_sample_time_s'],1.75)
        self.assertEqual(report['native_hz'],60)

    def test_prefix_query_exhaustion_keeps_unknown_even_after_first_hit(self):
        stage, manifest, _ = fixture()
        cube(stage, '/World/Greenhouse/House/Gutter', (0, 0, 0))
        report, _ = self.screen(stage, manifest, max_distance_queries=1)
        self.assertFalse(report['passed']); self.assertFalse(report['complete'])
        self.assertEqual(report['status'], 'blocked')
        self.assertIn('budget exhausted', report['error'])
        self.assertFalse(report['eligible_for_native_execution'])

    def test_preparation_scope_without_approach_boundary_cannot_pass(self):
        stage, manifest, plan = fixture()
        report, _ = self.screen(stage, manifest, plan)
        self.assertFalse(report['complete']); self.assertFalse(report['passed'])
        self.assertEqual(report['status'], 'inconclusive')
        self.assertIn('first approach boundary', report['error'])

    def test_preparation_external_command_and_coarse_gate_are_explicit(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/'report.json'
            def fake(command, **kwargs):
                self.assertIn('--scope', command); self.assertEqual(command[command.index('--scope')+1], 'preparation')
                output.write_text(json.dumps(dict(passed=True, complete=True, status='sampled_clear',
                    eligible_for_native_execution=False)))
                return SimpleNamespace(returncode=0, stdout='', stderr='')
            with patch('subprocess.run', fake):
                with self.assertRaisesRegex(RuntimeError, 'blocked commanded travel'):
                    audit_external('unused', output, plan='unused', scope='preparation', hz=8)

    def test_explicit_attachment_transfer_reuses_exact_validator_and_preserves_break_identity(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); checkpoint = root/'cp'; checkpoint.mkdir()
            (checkpoint/'checkpoint.json').write_text('{}'); (checkpoint/'checkpoint.npz').write_bytes(b'fixture')
            original = dict(scene_sha256='a'*64, source_assembly=str(root/'old'), metadata_sha256='c'*64,
                manifest=dict(compact_assets=dict(materialized_dependency=dict(sha256='1'*64))),
                break_events=[dict(joint='/World/ElasticTarget/HarvestJoints/Truss_02_Fruit_01',t=.8)],
                **completed())
            revised = dict(scene_sha256='a'*64, compact_assets=dict(materialized_dependency=dict(sha256='2'*64)))
            names = ['farmily_lift_height_joint', 'base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
            arrays = dict(body_paths=np.array(['/World/ElasticTarget/Truss_02_Fruit_01']),
                body_poses_world=np.array([[0.,0.,0.,0.,0.,0.,1.]]), robot_dof_names=np.array(names),robot_q=np.arange(7.))
            with patch('native_route_checkpoint.load_checkpoint', return_value=(original, arrays)), \
                    patch('eligible_attachment_transfer.validate_resume_revision', return_value={'kind':'verified exact fixture transfer'}) as validator:
                with self.assertRaisesRegex(ValueError, 'outside exact declared'):
                    start_checkpoint(root/'new', revised, checkpoint)
                validator.assert_not_called()
                poses, proof = start_checkpoint(root/'new', revised, checkpoint, allow_attachment_transfer=True)
                validator.assert_called_once_with(root/'new', revised, original)
                self.assertIn('/World/ElasticTarget/Truss_02_Fruit_01', poses)
                self.assertEqual(proof['native_break_events'][0]['joint'], original['break_events'][0]['joint'])
                self.assertEqual(proof['explicit_attachment_revision']['kind'], 'verified exact fixture transfer')
            with patch('native_route_checkpoint.load_checkpoint', return_value=(original, arrays)), \
                    patch('eligible_attachment_transfer.validate_resume_revision', side_effect=ValueError('unrelated property revision')):
                with self.assertRaisesRegex(ValueError, 'unrelated property revision'):
                    start_checkpoint(root/'new', revised, checkpoint, allow_attachment_transfer=True)


if __name__ == '__main__': unittest.main()
