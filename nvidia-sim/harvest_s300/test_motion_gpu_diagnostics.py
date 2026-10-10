import unittest

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from motion_gpu_diagnostics import SURFACE_API, VELOCITY_ATTRIBUTES, disable_zero_surface_velocity_for_diagnostic


class ZeroConveyorControlTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.belt = UsdGeom.Cube.Define(self.stage, '/World/ActuatedTool/belt')
        UsdPhysics.CollisionAPI.Apply(self.belt.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(self.belt.GetPrim())
        self.belt.GetPrim().AddAppliedSchema(SURFACE_API)
        self.velocity = self.belt.GetPrim().CreateAttribute(VELOCITY_ATTRIBUTES[0], Sdf.ValueTypeNames.Vector3f)
        self.velocity.Set(Gf.Vec3f(0))

    def test_zero_api_removed_only_in_session_collision_and_attributes_retained(self):
        original = self.stage.GetRootLayer().ExportToString()
        result = disable_zero_surface_velocity_for_diagnostic(self.stage, mode='pulse')
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), original)
        self.assertNotIn(SURFACE_API, self.belt.GetPrim().GetMetadata('apiSchemas').GetAppliedItems())
        self.assertTrue(self.belt.GetPrim().HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(self.belt.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertEqual(self.velocity.Get(), Gf.Vec3f(0))
        self.assertEqual(result['collider_changes'], 0)
        self.assertFalse(result['accepted_engine_equivalence'])

    def test_moving_conveyor_is_rejected_before_mutation(self):
        self.velocity.Set(Gf.Vec3f(0, .01, 0))
        original = self.stage.GetSessionLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, 'Nonzero/nonfinite'):
            disable_zero_surface_velocity_for_diagnostic(self.stage, mode='idle')
        self.assertEqual(self.stage.GetSessionLayer().ExportToString(), original)

    def test_nonzero_time_sample_rejected(self):
        self.velocity.Set(Gf.Vec3f(0, .01, 0), 5.)
        with self.assertRaisesRegex(ValueError, 'Nonzero/nonfinite'):
            disable_zero_surface_velocity_for_diagnostic(self.stage, mode='idle')

    def test_harvest_and_belt_fixture_modes_are_rejected(self):
        for mode in ('harvest', 'belt_control', 'jaw'):
            with self.assertRaisesRegex(ValueError, 'restricted to idle/pulse'):
                disable_zero_surface_velocity_for_diagnostic(self.stage, mode=mode)


if __name__ == '__main__':
    unittest.main()
