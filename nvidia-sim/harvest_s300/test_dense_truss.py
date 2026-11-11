"""Geometry regression on every actual cherry-truss source template."""
import json
from pathlib import Path
import unittest

import numpy as np

from latest_farm import LAYOUT, GLB_DIR, truss_meshes, tube_centerline
from dense_truss import compact_truss


class DenseTrussTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = {}
        for p in json.loads(LAYOUT.read_text())['neighbor_placements']:
            for t in p['trusses']:
                name = Path(t['source_glb']).name
                if name not in cls.cases:
                    old, _ = truss_meshes(GLB_DIR / name, t)
                    new, report = compact_truss(old)
                    cls.cases[name] = old, new, report

    def test_all_fruit_preserved_with_rigid_shape_and_color(self):
        self.assertEqual(len(self.cases), 5)
        for name, (old, new, report) in self.cases.items():
            self.assertEqual(report['fruit_count'], 10)
            for (n, v, f, c), (nn, vv, ff, cc) in zip(old, new):
                self.assertEqual(n, nn)
                np.testing.assert_array_equal(f, ff)
                np.testing.assert_array_equal(c, cc)
                if n.startswith(('Fruit_', 'Calyx_')):
                    np.testing.assert_allclose(v - v[0], vv - vv[0], atol=1e-12)

    def test_dense_without_fruit_intersections(self):
        for old, new, report in self.cases.values():
            self.assertGreaterEqual(min(report['nearest_bounding_sphere_gap_after_m']), .002 - 1e-12)
            self.assertLess(max(report['nearest_bounding_sphere_gap_after_m']), .004)
            self.assertLess(report['fruit_center_height_span_after_m'], report['fruit_center_height_span_before_m'] * .7)

    def test_shorter_rachis_and_peduncle_with_same_attachment(self):
        for old, new, r in self.cases.values():
            a = {n: v for n, v, _, _ in old}; b = {n: v for n, v, _, _ in new}
            for n in ('Truss_01_Peduncle', 'Rachis'):
                self.assertLess(r['lengths_after_m'][n], r['lengths_before_m'][n] * .8)
            p0, r0 = tube_centerline(a['Truss_01_Peduncle'], 14)
            p1, r1 = tube_centerline(b['Truss_01_Peduncle'], 14)
            np.testing.assert_allclose(p0[0], p1[0], atol=1e-10)
            np.testing.assert_allclose(r0, r1, atol=1e-10)
            q0, _ = tube_centerline(a['Rachis'], 14); q1, _ = tube_centerline(b['Rachis'], 14)
            np.testing.assert_allclose(q0[0] - p0[-1], q1[0] - p1[-1], atol=1e-10)

    def test_calyx_and_neck_remain_attached_to_each_fruit(self):
        for old, new, r in self.cases.values():
            a = {n: v for n, v, _, _ in old}; b = {n: v for n, v, _, _ in new}
            for fruit in r['fruit_names']:
                distal = 'Pedicel_distal_' + fruit.split('_')[1]
                p, _ = tube_centerline(a[distal], 12); q, _ = tube_centerline(b[distal], 12)
                np.testing.assert_allclose(q[-1] - p[-1], b[fruit][0] - a[fruit][0], atol=1e-10)


if __name__ == '__main__':
    unittest.main()
