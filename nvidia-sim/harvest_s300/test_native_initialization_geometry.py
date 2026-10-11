import unittest
import numpy as np
from native_initialization_geometry import capture_initialization_geometry


class Reader:
    body_ids={'active':0}
    def eligible(self,a,b):return a.startswith('leaf')
    def cooked(self,path):
        if path.startswith('leaf'):
            return dict(kind='native_cooked_convex',body='active',hulls=[(np.eye(3),np.eye(3),np.eye(3))])
        if path=='missing':raise ValueError('Native triangle query unavailable')
        return dict(kind='authored_static_triangle_fallback',body=None,native_cooked_triangle_verified=False)


class NativeInitializationGeometryTests(unittest.TestCase):
    def test_native_convex_and_authored_fallback_never_conflate_or_override_break(self):
        events=[dict(a='leaf1',b='background',separation_m=-.01,t=0.),
                dict(a='leaf2',b='missing',separation_m=-.02,t=0.)]
        report=capture_initialization_geometry(Reader(),events,[[1,2,3,0,0,0,1]])
        self.assertEqual(report['records'][0]['raw_contact_index'],1)
        convex=report['records'][0]['colliders'][0]
        self.assertTrue(convex['native_cooked_convex_verified'])
        self.assertEqual(convex['measured_owner_pose_world_xyzw'],[1,2,3,0,0,0,1])
        self.assertTrue(report['records'][0]['colliders'][1]['inconclusive'])
        fallback=report['records'][1]['colliders'][1]
        self.assertFalse(fallback['native_cooked_convex_verified'])
        self.assertFalse(fallback['native_cooked_triangle_verified'])
        self.assertFalse(report['failure_overridden']);self.assertFalse(report['physics_continued'])

    def test_exact_pair_deduplication_and_budget_preserve_worst_raw_record(self):
        events=[dict(a='leaf1',b='background',separation_m=-.001),
                dict(a='leaf1',b='background',separation_m=-.002),
                dict(a='robot',b='background',separation_m=-1.)]
        report=capture_initialization_geometry(Reader(),events,[[0,0,0,0,0,0,1]],max_pairs=1)
        self.assertEqual(report['eligible_pair_count'],1)
        self.assertEqual(report['records'][0]['raw_contact_index'],1)
        with self.assertRaises(ValueError):capture_initialization_geometry(Reader(),events,[[float('nan')]*7])


if __name__=='__main__':unittest.main()
