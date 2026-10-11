"""Equivalent self-collision screening, including touching and real FCL hits."""
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics

from motion_self_collision import MotionSelfCollision,_aabb_candidates_xyz


def original_screen(low,high,pairs,clearance_m):
    a,b=pairs.T
    return pairs[((low[a]<=high[b]+clearance_m)&
                  (low[b]<=high[a]+clearance_m)).all(1)]


class StagedAabbTests(unittest.TestCase):
    def assert_same(self,low,high,pairs,clearance=0.):
        result=_aabb_candidates_xyz(low,high,pairs,clearance)
        np.testing.assert_array_equal(result,original_screen(low,high,pairs,clearance))
        return result

    def test_face_edge_and_corner_touching_are_inclusive(self):
        low=np.array([[0,0,0],[1,0,0],[1,1,0],[1,1,1],[1.01,0,0]],float)
        high=low+1
        pairs=np.array([[4,0],[3,0],[1,0],[2,0]])
        np.testing.assert_array_equal(self.assert_same(low,high,pairs),pairs[1:])

    def test_exact_clearance_boundary_and_adjacent_float_values(self):
        clearance=.002
        boundary=1.+clearance
        low=np.array([[0,0,0],[boundary,0,0],
                      [np.nextafter(boundary,np.inf),0,0],
                      [np.nextafter(boundary,-np.inf),0,0]])
        high=low+1
        pairs=np.array([[1,0],[2,0],[3,0]])
        np.testing.assert_array_equal(self.assert_same(low,high,pairs,clearance),pairs[[0,2]])

    def test_original_order_duplicates_and_empty_survivors_are_retained(self):
        low=np.array([[0,0,0],[.2,.2,.2],[.4,.4,.4],[3,3,3]])
        high=low+1
        pairs=np.array([[2,1],[3,0],[1,0],[2,1],[2,0]])
        np.testing.assert_array_equal(self.assert_same(low,high,pairs),pairs[[0,2,3,4]])
        self.assertEqual(self.assert_same(low,high,np.array([[3,0]])).shape,(0,2))
        self.assertEqual(self.assert_same(low,high,np.empty((0,2),int)).shape,(0,2))

    def test_nonfinite_comparison_behavior_is_unchanged(self):
        low=np.array([[0,0,0],[np.nan,0,0],[0,-np.inf,0],
                      [0,0,np.inf],[0,0,-np.inf]])
        high=np.array([[1,1,1],[1,1,1],[1,np.inf,1],
                       [1,1,np.inf],[1,1,np.nan]])
        pairs=np.array([[i,j] for i in range(len(low)) for j in range(i)])
        self.assert_same(low,high,pairs)
        self.assert_same(low,high,pairs,.002)

    def test_random_rotated_aabbs_preserve_pairs_for_both_float_types(self):
        rng=np.random.default_rng(517)
        corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
        for dtype in (np.float32,np.float64):
            for _ in range(20):
                rotation=Rotation.random(45,random_state=rng).as_matrix()
                radii=rng.uniform(.001,.4,(45,3))
                center=rng.uniform(-.3,.3,(45,3))
                world=np.einsum('nij,nkj->nki',rotation,corners[None]*radii[:,None])+center[:,None]
                low=world.min(1).astype(dtype);high=world.max(1).astype(dtype)
                pairs=np.array([[i,j] for i in range(45) for j in range(i)])
                rng.shuffle(pairs)
                for clearance in (0.,.0005,.002):self.assert_same(low,high,pairs,clearance)

    def checker(self):
        stage=Usd.Stage.CreateInMemory()
        paths=['/World/HarvestRobot/Robot/arm','/World/ActuatedTool/jaw']
        for path in paths:
            body=UsdGeom.Xform.Define(stage,path)
            UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            cube=UsdGeom.Cube.Define(stage,path+'/shape');cube.CreateSizeAttr(.1)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return MotionSelfCollision(stage),paths

    def paired_hits(self,checker,poses):
        actual=checker.check(poses=poses)
        with patch('motion_self_collision._aabb_candidates_xyz',side_effect=original_screen):
            reference=checker.check(poses=poses)
        self.assertEqual(reference,actual)
        return actual

    def test_real_fcl_positive_and_clearance_controls_are_unchanged(self):
        checker,paths=self.checker()
        poses={p:np.eye(4) for p in paths};poses[paths[1]][0,3]=.05
        self.assertTrue(self.paired_hits(checker,poses)[0]['intersection'])
        poses[paths[1]][0,3]=.15
        self.assertFalse(self.paired_hits(checker,poses))
        poses[paths[1]][0,3]=.101
        self.assertFalse(self.paired_hits(checker,poses))
        checker.clearance=.002
        hits=self.paired_hits(checker,poses)
        self.assertFalse(hits[0]['intersection'])
        self.assertAlmostEqual(hits[0]['distance_m'],.001,places=7)

    def test_real_fcl_random_measured_pose_controls_are_unchanged(self):
        checker,paths=self.checker();rng=np.random.default_rng(318)
        for clearance in (0.,.002):
            checker.clearance=clearance
            for _ in range(60):
                poses={p:np.eye(4) for p in paths}
                for p in paths:
                    poses[p][:3,:3]=Rotation.random(random_state=rng).as_matrix()
                    poses[p][:3,3]=rng.uniform(-.08,.08,3)
                self.paired_hits(checker,poses)


if __name__=='__main__':unittest.main()
