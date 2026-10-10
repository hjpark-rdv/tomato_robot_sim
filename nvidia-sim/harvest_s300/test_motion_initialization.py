"""Offline conservation checks for gravity-neutral native initialization."""
import unittest
import json

from pxr import Gf, Sdf, Usd, UsdPhysics

from motion_initialization import GravityNeutralWarmup


class GravityWarmupTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.scene = UsdPhysics.Scene.Define(self.stage, '/World/PhysicsScene')
        self.scene.CreateGravityMagnitudeAttr(9.81)
        self.scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        self.root = self.stage.GetRootLayer().ExportToString()

    def test_restores_root_gravity_and_empty_session_exactly(self):
        session = self.stage.GetSessionLayer().ExportToString()
        warmup = GravityNeutralWarmup(self.stage)
        with warmup:
            self.assertEqual(self.scene.GetGravityMagnitudeAttr().Get(), 0.0)
            self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.root)
        self.assertAlmostEqual(self.scene.GetGravityMagnitudeAttr().Get(), 9.81, places=5)
        self.assertEqual(self.stage.GetSessionLayer().ExportToString(), session)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.root)
        self.assertTrue(warmup.metadata['gravity_restored_before_trial'])
        self.assertEqual(json.loads(json.dumps(warmup.metadata))['restored_resolved_gravity']
                         ['physics:gravityDirection'], [0.,0.,-1.])
        warmup.restore()

    def test_restores_existing_session_time_samples_and_metadata(self):
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            mag = self.scene.CreateGravityMagnitudeAttr(8.5)
            mag.Set(7.5, 1)
            mag.SetDocumentation('retain original gravity opinion')
            self.scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1, 0))
        session = self.stage.GetSessionLayer().ExportToString()
        with GravityNeutralWarmup(self.stage):
            self.assertEqual(self.scene.GetGravityMagnitudeAttr().Get(), 0.0)
        self.assertEqual(self.stage.GetSessionLayer().ExportToString(), session)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.root)

    def test_preserves_unrelated_initialization_session_changes(self):
        with GravityNeutralWarmup(self.stage):
            with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                self.scene.GetPrim().CreateAttribute('diagnostic:createdDuringInit', Sdf.ValueTypeNames.Int).Set(123)
        self.assertEqual(self.scene.GetPrim().GetAttribute('diagnostic:createdDuringInit').Get(), 123)
        self.assertFalse(self.stage.GetSessionLayer().GetPropertyAtPath('/World/PhysicsScene.physics:gravityMagnitude'))
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), self.root)

    def test_restores_after_initialization_exception(self):
        session = self.stage.GetSessionLayer().ExportToString()
        with self.assertRaisesRegex(RuntimeError, 'native failure'):
            with GravityNeutralWarmup(self.stage):
                raise RuntimeError('native failure')
        self.assertEqual(self.stage.GetSessionLayer().ExportToString(), session)

    def test_preserves_implicit_physics_schema_gravity(self):
        self.scene.GetPrim().RemoveProperty('physics:gravityMagnitude')
        self.scene.GetPrim().RemoveProperty('physics:gravityDirection')
        session = self.stage.GetSessionLayer().ExportToString()
        warmup = GravityNeutralWarmup(self.stage)
        with warmup:
            self.assertEqual(self.scene.GetGravityMagnitudeAttr().Get(), 0)
        self.assertEqual(warmup.metadata['source_resolved_gravity']['physics:gravityMagnitude'], '-inf')
        json.dumps(warmup.metadata)
        self.assertEqual(self.stage.GetSessionLayer().ExportToString(), session)

    def test_missing_scene_and_reentry_are_explicit_errors(self):
        with self.assertRaisesRegex(ValueError, 'missing'):
            GravityNeutralWarmup(self.stage, '/World/Missing')
        warmup = GravityNeutralWarmup(self.stage)
        with warmup:
            with self.assertRaisesRegex(RuntimeError, 're-entered'):
                warmup.__enter__()


if __name__ == '__main__':
    unittest.main()
