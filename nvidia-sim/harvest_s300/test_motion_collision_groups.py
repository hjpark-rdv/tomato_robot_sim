"""Offline checks that group compression preserves exact contact policies."""
import itertools
import unittest

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from motion_collision_groups import compress_leaf_collision_policy, _enabled_colliders, _filtered_pairs, _group_pairs, _pair
from motion_replication import SOURCE_ROOTS, replicate_motion


class CollisionPolicyTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        for root in (*SOURCE_ROOTS, '/World/PhysicsScene', '/World/Greenhouse/Plants'):
            UsdGeom.Xform.Define(self.stage, root)
        self.leaves = []
        self.others = []
        for label, container, count in [('Appendage_', self.leaves, 3), ('Branch_', self.others, 2)]:
            for i in range(count):
                path = '/World/ElasticTarget/' + label + str(i)
                prim = UsdGeom.Sphere.Define(self.stage, path).GetPrim()
                UsdPhysics.CollisionAPI.Apply(prim)
                prim.CreateAttribute('physxCollision:contactOffset', Sdf.ValueTypeNames.Float).Set(.001)
                container.append(path)
        for i, path in enumerate(self.leaves):
            UsdPhysics.FilteredPairsAPI.Apply(self.stage.GetPrimAtPath(path)).CreateFilteredPairsRel().SetTargets(
                self.leaves[i + 1:])
        UsdPhysics.FilteredPairsAPI.Apply(self.stage.GetPrimAtPath(self.others[0])).CreateFilteredPairsRel().SetTargets(
            [self.others[1]])
        self.original = self.stage.GetRootLayer().ExportToString()

    def test_production_compresses_only_existing_complete_leaf_leaf_pairs(self):
        result = compress_leaf_collision_policy(self.stage)
        self.assertEqual(result['removed_leaf_leaf_targets'], 3)
        self.assertEqual(result['expected_plant_excluded_pairs'], 4)
        self.assertEqual(result['retained_filtered_pairs'], 1)
        self.assertEqual(result['expected_pair_sha256'], result['actual_pair_sha256'])
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.original)
        self.assertTrue(result['dynamics_and_geometry']['passed'])
        pairs = _filtered_pairs(self.stage, self.leaves + self.others) | _group_pairs(
            self.stage, self.leaves + self.others)
        for a in self.leaves:
            for b in self.others:
                self.assertNotIn(_pair(a, b), pairs)

    def test_isolated_fixture_adds_only_existing_leaf_nonleaf_policy(self):
        active = UsdPhysics.CollisionGroup.Define(self.stage, '/World/FixtureGroups/Active')
        active.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(SOURCE_ROOTS)
        background = UsdPhysics.CollisionGroup.Define(self.stage, '/World/FixtureGroups/Background')
        background.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(['/World/Greenhouse/Plants'])
        background.CreateFilteredGroupsRel().SetTargets([active.GetPath()])
        original = self.stage.GetRootLayer().ExportToString()
        result = compress_leaf_collision_policy(self.stage, isolate_background=True)
        self.assertEqual(result['expected_plant_excluded_pairs'], 10)
        self.assertEqual(result['retained_filtered_pairs'], 1)
        self.assertEqual(active.GetCollidersCollectionAPI().GetIncludesRel().GetTargets(), [])
        self.assertEqual(background.GetFilteredGroupsRel().GetTargets(), list(map(Sdf.Path, result['groups'])))
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), original)

    def test_incomplete_original_graph_is_rejected_before_mutation(self):
        self.stage.GetPrimAtPath(self.leaves[0]).GetRelationship('physics:filteredPairs').SetTargets([])
        original = self.stage.GetRootLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, 'not complete'):
            compress_leaf_collision_policy(self.stage)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), original)
        self.assertFalse(self.stage.GetPrimAtPath('/World/FixtureGroups/Leaves'))

    def test_unrecognized_group_membership_is_rejected(self):
        group = UsdPhysics.CollisionGroup.Define(self.stage, '/World/Unexpected')
        group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([self.leaves[0]])
        with self.assertRaisesRegex(ValueError, 'overlaps active plant'):
            compress_leaf_collision_policy(self.stage)

    def test_categories_clone_with_disjoint_membership_and_cross_env_filters(self):
        background_path = '/World/Greenhouse/Plants/neighbor'
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Sphere.Define(self.stage, background_path).GetPrim())
        compress_leaf_collision_policy(self.stage, isolate_background=True)
        manifest = {'target': {'path': self.others[0], 'joint': '/World/ElasticTarget/joint'},
                    'tool': {'belts': []}, 'plant': {'bodies': []}}
        result = replicate_motion(self.stage, manifest, 2, device='cpu')
        self.assertEqual(len(result['metadata']['category_group_checks']), 2)
        all_colliders = _enabled_colliders(self.stage, ['/World'])
        pairs = _group_pairs(self.stage, all_colliders)
        a = '/World/MotionEnvs/env_0/ElasticTarget/Appendage_0'
        b = '/World/MotionEnvs/env_1/ElasticTarget/Branch_0'
        self.assertIn(_pair(a, b), pairs)
        self.assertIn(_pair(a, background_path), pairs)
        self.assertIn(_pair(b, background_path), pairs)
        for i in range(2):
            root = '/World/MotionIsolation/env_' + str(i)
            leaves = UsdPhysics.CollisionGroup(self.stage.GetPrimAtPath(root + '/Leaves'))
            self.assertIn(leaves.GetPath(), leaves.GetFilteredGroupsRel().GetTargets())
            self.assertIn(Sdf.Path(root + '/ActivePlantOther'), leaves.GetFilteredGroupsRel().GetTargets())
            self.assertNotIn(Sdf.Path(root + '/RobotTool'), leaves.GetFilteredGroupsRel().GetTargets())
        self.assertFalse(self.stage.GetPrimAtPath('/World/MotionIsolation/group_0'))

    def test_single_reference_clone_moves_category_membership_off_inactive_roots(self):
        compress_leaf_collision_policy(self.stage)
        manifest = {'target': {'path': self.others[0], 'joint': '/World/ElasticTarget/joint'},
                    'tool': {'belts': []}, 'plant': {'bodies': []}}
        result = replicate_motion(self.stage, manifest, 1, device='cpu', source_copy='reference')
        self.assertTrue(result['metadata']['category_group_checks'][0]['passed'])
        leaves = UsdPhysics.CollisionGroup(self.stage.GetPrimAtPath('/World/MotionIsolation/env_0/Leaves'))
        self.assertEqual(len(leaves.GetCollidersCollectionAPI().GetIncludesRel().GetTargets()), 3)
        old = UsdPhysics.CollisionGroup(self.stage.GetPrimAtPath('/World/FixtureGroups/Leaves'))
        self.assertEqual(old.GetCollidersCollectionAPI().GetIncludesRel().GetTargets(), [])


if __name__ == '__main__':
    unittest.main()
