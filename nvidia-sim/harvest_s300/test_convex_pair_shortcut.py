import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from leaf_contact_geometry import convex_axes,convex_pair_gaps,separating_gap,full_sat_axes


def box(scale=(.001,.001,.001)):
    v=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]],float)*scale
    n,e=convex_axes(v);return v,n,e


def evidence(gaps):
    return dict(clear=all(g>1e-6 for g in gaps),within=all(g>=-.0005+.000001 for g in gaps),upper=max(0.,-min(gaps)))


class ConvexPairShortcutTest(unittest.TestCase):
    def test_all_pairs_covered_positive_gap_skips_expensive_axes_only(self):
        a=box();b=(a[0]+[.01,0,0],a[1],a[2])
        with patch('leaf_contact_geometry.full_sat_axes',side_effect=AssertionError('No axes needed for exact positive certificate')):
            gaps,count=convex_pair_gaps([a],[b])
        self.assertEqual(count,1);self.assertAlmostEqual(gaps[0],.008)
        mixed=convex_pair_gaps([a],[b,(a[0]+[.0016,0,0],a[1],a[2])])
        self.assertEqual(len(mixed[0]),2);self.assertEqual(mixed[1],1)
        self.assertFalse(evidence(mixed[0])['clear']);self.assertTrue(evidence(mixed[0])['within'])
        self.assertAlmostEqual(evidence(mixed[0])['upper'],.0004)

    def test_shallow_deep_touch_and_sub_guard_positive_keep_full_axes(self):
        a=box()
        for delta in (.0016,.0014,.002,.0020005):
            b=(a[0]+[delta,0,0],a[1],a[2])
            axes=full_sat_axes(a[1],b[1],a[2],b[2])
            expected=separating_gap(a[0],b[0],axes)
            with patch('leaf_contact_geometry.full_sat_axes',wraps=full_sat_axes) as call:
                gaps,count=convex_pair_gaps([a],[b])
            self.assertEqual(count,0);self.assertEqual(call.call_count,1);self.assertEqual(gaps,[expected])

    def test_edge_cross_control_still_uses_full_sat_with_overlapping_world_aabb(self):
        av,an,ae=box((.01,.0035,.0025))
        rot=np.array([[-.20423723223779924,.9154314910157642,.34680302510863137],
                      [.7317120904781618,-.09258168804343717,.6752970070177315],
                      [.6502957554958525,.3916807581381583,-.6509236622587904]])
        bv=av@rot.T+np.array([-.8811588272089262,.9619212993310635,-.665339509876471])*.01
        bn,be=convex_axes(bv)
        self.assertLess(separating_gap(av,bv,np.r_[an,bn]),-.002)
        gaps,count=convex_pair_gaps([(av,an,ae)],[(bv,bn,be)])
        self.assertEqual(count,0);self.assertGreater(gaps[0],.003)

    def test_random_measured_poses_preserve_clear_within_limit_and_upper_bound(self):
        rng=np.random.default_rng(9381);a=box((.007,.003,.001))
        for _ in range(80):
            rot=Rotation.random(random_state=rng).as_matrix();shift=rng.uniform(-.012,.012,3)
            b=(a[0]@rot.T+shift,a[1]@rot.T,a[2]@rot.T)
            old=[separating_gap(a[0],b[0],full_sat_axes(a[1],b[1],a[2],b[2]))]
            new,_=convex_pair_gaps([a],[b])
            self.assertEqual(evidence(new),evidence(old))

    def test_missing_nonfinite_native_shapes_or_changed_guard_fail_closed(self):
        a=box();bad=(a[0].copy(),a[1],a[2]);bad[0][0,0]=np.nan
        for first,second in (([],[a]),([a],[]),([bad],[a])):
            with self.assertRaises(ValueError):convex_pair_gaps(first,second)
        with self.assertRaises(ValueError):convex_pair_gaps([a],[a],clear_guard_m=.0005)

if __name__=='__main__':unittest.main()
