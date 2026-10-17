"""Exact released-fruit gutter exception; CPU Warp algebra, no integration."""
import unittest

import numpy as np
import warp as wp
from mujoco_warp._src.types import vec5

from newton_kernels import (released_gutter_geom_ids, audit_contacts,
                            audit_released_gutter)


def arr(value, dtype=float):
    return wp.array(np.asarray(value), dtype=dtype, device='cpu')


class ReleasedGutterTests(unittest.TestCase):
    def test_exact_class_and_world_owner_only_default_disabled(self):
        metadata = dict(farm=dict(structural_inventory=[
            dict(name='exact', classification='gutter'),
            dict(name='moving_gutter', classification='gutter'),
            dict(name='gutter_named_mount', classification='mount')]))
        names = ['exact', 'moving_gutter', 'gutter_named_mount', 'gutter_by_name_only']
        owners = [0, 2, 0, 0]
        self.assertEqual(len(released_gutter_geom_ids(metadata, names, owners)), 0)
        metadata['released_gutter_diagnostic'] = True
        np.testing.assert_array_equal(released_gutter_geom_ids(metadata, names, owners), [0])
        metadata['farm']['structural_inventory'].append(dict(name='exact', classification='gutter'))
        with self.assertRaisesRegex(ValueError, 'Ambiguous'):
            released_gutter_geom_ids(metadata, names, owners)

    def test_missing_world_gutter_is_an_error_instead_of_silent_broad_exemption(self):
        with self.assertRaisesRegex(ValueError, 'exact world gutter'):
            released_gutter_geom_ids(dict(released_gutter_diagnostic=True), ['mount'], [0])

    def test_only_already_released_gutter_pair_is_exempt_and_separately_measured(self):
        # g0 exact gutter, g1 attached fruit, g2 dropped fruit, g3 robot,
        # g4 other structural obstacle. No source contact value is modified.
        pairs = arr([[0, 1], [2, 0], [0, 3], [2, 4]], wp.vec2i)
        distance = arr([-.003, -.012, -.002, -.004])
        forces = arr([[2., 3., 4., 5.]])
        addresses = arr([[0], [1], [2], [3]], int)
        geom_body = arr([0, 1, 3, 2, 0], int)
        fruit = arr([-1, 0, -1, 1], int)
        attached = arr([[True, False]], bool)
        allowed = np.zeros((2, 5), bool); allowed[:, 0] = True
        active = wp.zeros((1, 1), float, device='cpu')
        released = wp.zeros_like(active); force_max = wp.zeros_like(active)
        event = arr([np.iinfo(np.uint64).max], wp.uint64)
        common = [arr([0], int), arr([4], int), 4, 1, arr([0, 0, 0, 0], int),
            pairs, distance, arr(np.tile(np.eye(3), (4, 1, 1)), wp.mat33),
            arr(np.zeros((4, 5)), vec5), arr([1, 1, 1, 1], int), addresses,
            arr([0., 0., 0., 0.]), forces]
        wp.launch(audit_contacts, 4, inputs=common + [geom_body, fruit, arr(allowed, bool),
            attached, active, released, force_max, event], device='cpu')
        self.assertAlmostEqual(float(active.numpy()[0, 0]), .004, places=7)
        self.assertAlmostEqual(float(released.numpy()[0, 0]), .012, places=7)
        # Attached fruit/gutter remains the first strict violation. The dropped
        # fruit/other obstacle and robot/gutter are also in the active maximum.
        self.assertEqual(int(event.numpy()[0]), 1)
        pen = wp.zeros_like(active); f = wp.zeros_like(active)
        count = wp.zeros((1, 1), int, device='cpu')
        wp.launch(audit_released_gutter, 4, inputs=common + [geom_body, fruit, attached,
            arr([True, False, False, False, False], bool), pen, f, count], device='cpu')
        self.assertAlmostEqual(float(pen.numpy()[0, 0]), .012, places=7)
        self.assertAlmostEqual(float(f.numpy()[0, 0]), 3., places=7)
        self.assertEqual(int(count.numpy()[0, 0]), 1)
        np.testing.assert_array_equal(distance.numpy(), np.array([-.003, -.012, -.002, -.004], np.float32))
        np.testing.assert_array_equal(forces.numpy(), np.array([[2., 3., 4., 5.]], np.float32))


if __name__ == '__main__':
    unittest.main()
