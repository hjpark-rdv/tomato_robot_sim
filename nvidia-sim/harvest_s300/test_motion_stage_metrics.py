import unittest
from pxr import Usd, UsdGeom, UsdPhysics
from motion_stage_metrics import copy_stage_metrics, require_motion_units, stage_metrics


class StageMetricsTest(unittest.TestCase):
    def test_sublayer_does_not_supply_stage_metrics(self):
        source = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(source, 1.)
        UsdGeom.SetStageUpAxis(source, 'Z')
        UsdPhysics.SetStageKilogramsPerUnit(source, 1.)
        source.SetTimeCodesPerSecond(60.)
        source.SetFramesPerSecond(30.)
        overlay = Usd.Stage.CreateInMemory()
        overlay.GetRootLayer().subLayerPaths = [source.GetRootLayer().identifier]
        self.assertEqual(stage_metrics(overlay)['meters_per_unit'], .01)
        with self.assertRaisesRegex(ValueError, 'requires explicit'):
            require_motion_units(overlay)
        self.assertEqual(copy_stage_metrics(source, overlay), stage_metrics(source))
        self.assertEqual(stage_metrics(overlay), stage_metrics(source))
        self.assertEqual(require_motion_units(overlay)['up_axis'], 'Z')

    def test_copy_is_not_silent_unit_conversion(self):
        source = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(source, .001)
        UsdGeom.SetStageUpAxis(source, 'Y')
        UsdPhysics.SetStageKilogramsPerUnit(source, .01)
        overlay = Usd.Stage.CreateInMemory()
        copy_stage_metrics(source, overlay)
        self.assertEqual(stage_metrics(source), stage_metrics(overlay))
        with self.assertRaises(ValueError):
            require_motion_units(overlay)


if __name__ == '__main__':
    unittest.main()
