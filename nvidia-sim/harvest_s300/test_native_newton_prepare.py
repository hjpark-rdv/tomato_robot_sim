"""USD derivative preservation tests; run in an OpenUSD-capable Python."""

from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from native_newton_prepare import prepare, sha256


class NativePreparationTests(unittest.TestCase):
    def test_preserves_bodies_joints_colliders_and_original(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.usda"
            stage = Usd.Stage.CreateNew(str(source))
            UsdGeom.SetStageUpAxis(stage, "Z")
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)
            UsdPhysics.Scene.Define(stage, "/World/physics")
            body = UsdGeom.Xform.Define(stage, "/World/body").GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(body)
            collider = UsdGeom.Mesh.Define(stage, "/World/body/collider")
            collider.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)])
            UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(collider.GetPrim()).CreateApproximationAttr("convexDecomposition")
            visual = UsdGeom.Mesh.Define(stage, "/World/body/visual")
            UsdGeom.Subset.Define(stage, str(visual.GetPath()) + "/material_subset")
            joint = UsdPhysics.FixedJoint.Define(stage, "/World/joint")
            joint.CreateBody1Rel().SetTargets([body.GetPath()])
            joint.CreateBreakForceAttr(3.0)
            stage.GetRootLayer().Save()
            original_hash = sha256(source)
            result = prepare(source, root / "derived")
            derivative = Usd.Stage.Open(result["stage"])
            self.assertEqual(sha256(source), original_hash)
            self.assertTrue(derivative.GetPrimAtPath(body.GetPath()).HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertTrue(derivative.GetPrimAtPath(joint.GetPath()).IsA(UsdPhysics.FixedJoint))
            self.assertTrue(UsdPhysics.CollisionAPI(derivative.GetPrimAtPath(collider.GetPath())).GetCollisionEnabledAttr().Get())
            self.assertEqual(UsdPhysics.MeshCollisionAPI(derivative.GetPrimAtPath(collider.GetPath())).GetApproximationAttr().Get(), "convexHull")
            self.assertFalse(derivative.GetPrimAtPath(visual.GetPath()).IsActive())
            self.assertEqual(result["counts"]["source_bodies"], 1)
            self.assertEqual(result["counts"]["source_joints"], 1)
            self.assertEqual(result["counts"]["enabled_colliders"], 1)
            self.assertEqual(result["semantics_requiring_adapter"][0]["attributes"]["physics:breakForce"], "3.0")
            self.assertIsNone(result["harvest_success"])

    def test_refuses_output_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.usda"
            stage = Usd.Stage.CreateNew(str(source))
            stage.GetRootLayer().Save()
            output = root / "existing"
            output.mkdir()
            with self.assertRaises(FileExistsError):
                prepare(source, output)

    def test_display_root_pruning_preserves_static_obstacles(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.usda"
            stage = Usd.Stage.CreateNew(str(source))
            UsdGeom.Xform.Define(stage, "/World/DisplayForest")
            UsdGeom.Mesh.Define(stage, "/World/DisplayForest/visual")
            UsdGeom.Xform.Define(stage, "/World/StructuralObstacle")
            obstacle = UsdGeom.Cube.Define(stage, "/World/StructuralObstacle/collider")
            UsdPhysics.CollisionAPI.Apply(obstacle.GetPrim()).CreateCollisionEnabledAttr(True)
            stage.GetRootLayer().Save()
            result = prepare(source, root / "derived")
            derivative = Usd.Stage.Open(result["stage"])
            self.assertEqual(result["display_subtrees_deactivated"], ["/World/DisplayForest"])
            self.assertFalse(derivative.GetPrimAtPath("/World/DisplayForest").IsActive())
            self.assertTrue(derivative.GetPrimAtPath("/World/StructuralObstacle").IsActive())
            self.assertTrue(derivative.GetPrimAtPath(obstacle.GetPath()).IsActive())
            self.assertEqual(result["enabled_colliders_preserved"], [str(obstacle.GetPath())])


if __name__ == "__main__":
    unittest.main()
