"""Offline geometry/physics checks; these are not native transport proofs."""
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema

from motion_belt_control import build_belt_control


ROOT = '/World/ActuatedTool'


def tool(stage):
    parent = UsdGeom.Xform.Define(stage, ROOT)
    parent.AddTranslateOp().Set(Gf.Vec3d(.4, -.2, .9))
    parent.AddRotateZOp().Set(90.)
    fixed = UsdGeom.Xform.Define(stage, ROOT + '/fixed')
    fixed.AddRotateYOp().Set(30.)
    UsdPhysics.RigidBodyAPI.Apply(fixed.GetPrim())
    material = UsdShade.Material.Define(stage, '/World/Materials/Belt')
    UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateDynamicFrictionAttr(.65)
    paths = []
    for name in ('L', 'R'):
        path = ROOT + '/belt' + name
        paths.append(path)
        belt = UsdGeom.Cube.Define(stage, path)
        belt.CreateSizeAttr(.1)
        UsdPhysics.RigidBodyAPI.Apply(belt.GetPrim())
        UsdPhysics.CollisionAPI.Apply(belt.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(belt.GetPrim()).CreateMassAttr(.3)
        surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(belt.GetPrim())
        surface.CreateSurfaceVelocityAttr(Gf.Vec3f(0, .12, 0))
        surface.CreateSurfaceVelocityLocalSpaceAttr(True)
        UsdShade.MaterialBindingAPI.Apply(belt.GetPrim()).Bind(material, materialPurpose='physics')
    return dict(tool=dict(belts=paths, nominal_gap_mm=44., jaw_limits_mm=[28., 96.],
        belt_bounds=[dict(name='L', minimum=[-.058, -.004, .009], maximum=[-.022, .171, .061]),
                     dict(name='R', minimum=[.022, -.018, .013], maximum=[.058, .157, .065])]))


def source_inventory(stage):
    return {str(p.GetPath()): (
        p.GetTypeName(), p.GetAppliedSchemas(),
        {a.GetName(): str(a.Get()) for a in p.GetAttributes()},
        {r.GetName(): [str(t) for t in r.GetTargets()] for r in p.GetRelationships()})
        for p in stage.Traverse() if '/Diagnostics' not in str(p.GetPath())}


class BeltControlTests(unittest.TestCase):
    def test_session_fixture_preserves_source_and_has_actual_sphere_mass(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'source.usda'
            authored = Usd.Stage.CreateNew(str(path))
            manifest = tool(authored)
            authored.GetRootLayer().Save()
            source = path.read_bytes()
            stage = Usd.Stage.Open(str(path))
            before = source_inventory(stage)
            # Preserve the explicitly requested historical 40 mm dimensions;
            # this offline test cannot establish valid native initialization.
            result = build_belt_control(stage, manifest, diameter_m=.04)
            self.assertEqual(source_inventory(stage), before)
            self.assertEqual(path.read_bytes(), source)
            self.assertEqual(stage.GetRootLayer().ExportToString(), authored.GetRootLayer().ExportToString())
            sphere = UsdGeom.Sphere(stage.GetPrimAtPath(result['body_path']))
            self.assertAlmostEqual(sphere.GetRadiusAttr().Get(), .02)
            mass = UsdPhysics.MassAPI(sphere.GetPrim())
            expected_mass = 1000. * 4. * math.pi * .02 ** 3 / 3.
            self.assertAlmostEqual(mass.GetMassAttr().Get(), expected_mass, places=8)
            np.testing.assert_allclose(mass.GetDiagonalInertiaAttr().Get(),
                                       [2. * expected_mass * .02 ** 2 / 5.] * 3, rtol=1e-7)
            self.assertTrue(UsdPhysics.CollisionAPI(sphere.GetPrim()).GetCollisionEnabledAttr().Get())
            self.assertTrue(PhysxSchema.PhysxRigidBodyAPI(sphere.GetPrim()).GetDisableGravityAttr().Get())
            self.assertFalse(sphere.GetPrim().GetCustomDataByKey('farmFruit'))
            self.assertTrue(result['fixture_only'])
            self.assertIsNone(result['harvest_success'])

    def test_guide_frees_physical_cad_y_in_rotated_world_and_contacts_on_close(self):
        stage = Usd.Stage.CreateInMemory()
        manifest = tool(stage)
        result = build_belt_control(stage, manifest)
        # Known parent Rz(90) and fixed Ry(30), independent of helper metadata.
        c, s = math.cos(math.pi / 6), math.sin(math.pi / 6)
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]) @ np.array(
            [[c, 0., s], [0., 1., 0.], [-s, 0., c]])
        center = rotation @ [0., .093, .037] + [.4, -.2, .9]
        np.testing.assert_allclose(result['start_center_world'], center, atol=1e-9)
        np.testing.assert_allclose(result['transport_axis_world'], rotation[:, 1], atol=1e-9)
        sphere_world = UsdGeom.XformCache().GetLocalToWorldTransform(
            stage.GetPrimAtPath(result['body_path']))
        np.testing.assert_allclose(sphere_world.ExtractTranslation(), center, atol=1e-9)
        guide = UsdPhysics.Joint(stage.GetPrimAtPath(result['guide_path']))
        self.assertEqual(guide.GetBody0Rel().GetTargets(), [])
        self.assertEqual([str(p) for p in guide.GetBody1Rel().GetTargets()], [result['body_path']])
        self.assertTrue(guide.GetExcludeFromArticulationAttr().Get())
        guide_rotation = np.asarray(Gf.Matrix3d(Gf.Quatd(guide.GetLocalRot0Attr().Get()))).T
        np.testing.assert_allclose(guide_rotation, rotation, atol=1e-7)
        anchor = np.asarray(guide.GetLocalPos0Attr().Get())
        # A 20 mm movement along CAD Y leaves both locked translations zero.
        free_position = center + .02 * rotation[:, 1]
        local_delta = guide_rotation.T @ (free_position - anchor)
        np.testing.assert_allclose(local_delta, [0., .02, 0.], atol=1e-7)
        for index, axis in enumerate(('transX', 'transY', 'transZ')):
            limit = UsdPhysics.LimitAPI(guide.GetPrim(), axis)
            if index == 1:
                self.assertFalse(guide.GetPrim().HasAPI(UsdPhysics.LimitAPI, axis))
            else:
                self.assertGreater(limit.GetLowAttr().Get(), limit.GetHighAttr().Get())
                restricted_delta = guide_rotation.T @ (center + .01 * rotation[:, index] - anchor)
                self.assertGreater(abs(restricted_delta[index]), .009)
        for axis in ('transX', 'transY', 'transZ', 'rotX', 'rotY', 'rotZ'):
            self.assertFalse(guide.GetPrim().HasAPI(UsdPhysics.DriveAPI, axis))
        for axis in ('rotX', 'rotY', 'rotZ'):
            self.assertFalse(guide.GetPrim().HasAPI(UsdPhysics.LimitAPI, axis))
        radius = result['diameter_m'] / 2.
        self.assertGreater(result['nominal_gap_m'] / 2., radius)
        self.assertLess(result['requested_closed_gap_m'] / 2., radius)
        self.assertAlmostEqual(result['diameter_m'], .03)
        self.assertAlmostEqual(result['initial_raw_cad_radial_clearance_m'], .007)
        self.assertAlmostEqual(result['command_closed_gap_m'], .028)
        self.assertAlmostEqual(radius - result['requested_closed_gap_m'] / 2., .001)
        # No native claim: this proves initial clearance and geometric contact
        # loading after the real jaws close, not belt friction or transport.

    def test_invalid_geometry_and_repeat_reject_before_source_edit(self):
        for case in ('too_large', 'too_small', 'bad_span', 'scaled'):
            with self.subTest(case=case):
                stage = Usd.Stage.CreateInMemory()
                manifest = tool(stage)
                kwargs = {}
                if case == 'too_large':
                    kwargs['diameter_m'] = .044
                elif case == 'too_small':
                    kwargs['diameter_m'] = .028
                elif case == 'bad_span':
                    manifest['tool']['belt_bounds'][0]['maximum'][2] = .04
                else:
                    UsdGeom.Xform(stage.GetPrimAtPath(ROOT)).AddScaleOp().Set(Gf.Vec3f(2.))
                before = stage.GetRootLayer().ExportToString()
                with self.assertRaises(ValueError):
                    build_belt_control(stage, manifest, **kwargs)
                self.assertEqual(stage.GetRootLayer().ExportToString(), before)
                self.assertTrue(stage.GetSessionLayer().empty)
        stage = Usd.Stage.CreateInMemory()
        manifest = tool(stage)
        build_belt_control(stage, manifest)
        session = stage.GetSessionLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, 'already exists'):
            build_belt_control(stage, manifest)
        self.assertEqual(stage.GetSessionLayer().ExportToString(), session)


if __name__ == '__main__':
    unittest.main()
