"""Compiled audit tests use mocked native queries; no simulator is launched."""
import unittest

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from motion_compiled_audit import collect_compiled_body_audit, compare_compiled_body_audits


class NativeQueries:
    def __init__(self, mass=1.0, missing=False):
        self.mass = mass
        self.missing = missing

    def get_mass_information(self, path):
        if self.missing:
            return {}
        return dict(mass=self.mass, inertia=[.1, .2, .3], com=[0., 0., 0.])

    def get_materials_paths(self, path):
        return ['/World/Materials/Fruit']

    def get_rigidbody_transformation(self, path):
        return dict(ret_val=True, position=[1., 2., 3.], rotation=[0., 0., 0., 1.])


class ArticulationQuery:
    count = 1
    shared_metatype = type('Meta', (), dict(dof_names=['hinge'], link_names=['root', 'child']))()
    def get_dof_positions(self):
        return np.array([[.01]])
    def get_dof_velocities(self):
        return np.array([[.02]])
    def get_dof_stiffnesses(self):
        return np.array([[.1]])


class CompiledBodyTests(unittest.TestCase):
    def make_stage(self, root):
        stage = Usd.Stage.CreateInMemory()
        body = UsdGeom.Xform.Define(stage, root + '/body').GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        # MassAPI's unauthored COM fallback is an intentional -Inf sentinel.
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(1.0)
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Sphere.Define(stage, root + '/body/collider').GetPrim())
        return stage, str(body.GetPath())

    def collect(self, stage, body, query=None, **kwargs):
        query = query or NativeQueries()
        return collect_compiled_body_audit(stage, [body], query_interface=query,
                                          pose_interface=query, **kwargs)

    def test_original_and_clone_native_properties_compare_in_canonical_namespace(self):
        source, path = self.make_stage('/World/ElasticTarget')
        clone, clone_path = self.make_stage('/World/MotionEnvs/env_0/ElasticTarget')
        before = source.GetRootLayer().ExportToString()
        left = self.collect(source, path, articulation_views={'plant': ArticulationQuery()})
        right = self.collect(clone, clone_path,
            path_mapping={'/World/MotionEnvs/env_0/ElasticTarget': '/World/ElasticTarget'},
            articulation_views={'plant': ArticulationQuery()})
        result = compare_compiled_body_audits(left, right)
        self.assertTrue(result['complete'])
        self.assertTrue(result['native_mass_material_hash_match'])
        self.assertEqual(result['body_differences'][0]['mass_maximum_absolute_difference'], 0.)
        self.assertEqual(result['articulation_differences']['plant']['fields']['get_dof_positions']['maximum_absolute_difference'], 0.)
        self.assertEqual(before, source.GetRootLayer().ExportToString())
        self.assertEqual(left['physics_steps_added'], 0)

    def test_native_compilation_difference_is_reported_without_universal_threshold(self):
        stage, path = self.make_stage('/World/ElasticTarget')
        result = compare_compiled_body_audits(self.collect(stage, path),
                                             self.collect(stage, path, NativeQueries(mass=1.01)))
        self.assertFalse(result['native_mass_material_hash_match'])
        self.assertAlmostEqual(result['body_differences'][0]['mass_maximum_absolute_difference'], .01)
        self.assertNotIn('passed', result)

    def test_missing_native_mass_data_is_incomplete_evidence(self):
        stage, path = self.make_stage('/World/ElasticTarget')
        result = self.collect(stage, path, NativeQueries(missing=True))
        self.assertFalse(result['complete'])
        self.assertIn('missing fields', result['errors'][0]['error'])

    def test_nan_in_native_mass_is_explicit_error(self):
        stage, path = self.make_stage('/World/ElasticTarget')
        result = self.collect(stage, path, NativeQueries(mass=float('nan')))
        self.assertFalse(result['complete'])
        self.assertIn('NaN/Inf', result['errors'][0]['error'])

    def test_scalar_queries_are_not_used_on_direct_gpu_diagnostic(self):
        stage, path = self.make_stage('/World/ElasticTarget')
        with self.assertRaisesRegex(ValueError, 'restricted to CPU'):
            self.collect(stage, path, device='cuda:0')


if __name__ == '__main__':
    unittest.main()
