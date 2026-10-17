import unittest

import numpy as np

from newton_contact_geometry_review import capsule_segment_review, segment_inside_convex_interval


def box(half):
    size=np.broadcast_to(half,(3,))
    return np.vstack([np.column_stack([np.eye(3),-size]),np.column_stack([-np.eye(3),-size])])


class ContactGeometryControls(unittest.TestCase):
    def test_full_radius_burial_positive_control(self):
        row=capsule_segment_review(box(1),[[-2,0,0],[2,0,0]],.2,guard_m=0)
        np.testing.assert_allclose(row['axis_inside_interval'],[.25,.75])
        np.testing.assert_allclose(row['fully_buried_radius_ball_interval'],[.3,.7])
        self.assertTrue(row['axis_crosses_solid_between_two_outside_ends'])

    def test_negative_clear_segment_and_endpoint_insertion(self):
        self.assertIsNone(segment_inside_convex_interval(box(1),[-2,2,0],[2,2,0]))
        row=capsule_segment_review(box(1),[[-2,0,0],[0,0,0]],.2)
        self.assertIsNotNone(row['fully_buried_radius_ball_interval'])
        self.assertFalse(row['axis_crosses_solid_between_two_outside_ends'])

    def test_thin_local_solid_crossing_is_not_negated_by_no_full_ball(self):
        row=capsule_segment_review(box([.05,1,1]),[[-2,0,0],[2,0,0]],.2)
        self.assertIsNone(row['fully_buried_radius_ball_interval'])
        self.assertTrue(row['axis_crosses_solid_between_two_outside_ends'])
        self.assertAlmostEqual(row['axis_length_inside_m'],.1)

    def test_single_point_corner_tangency_is_not_through_solid(self):
        row=capsule_segment_review(box(1),[[-2,0,0],[0,2,0]],.2)
        np.testing.assert_allclose(row['axis_inside_interval'],[.5,.5])
        self.assertEqual(row['axis_length_inside_m'],0)
        self.assertFalse(row['axis_crosses_solid_between_two_outside_ends'])

    def test_rigid_transform_invariance(self):
        planes=box(1);a=np.array([-2,0,0]);b=np.array([2,0,0])
        theta=.47;r=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1.]])
        t=np.array([.3,-.7,1.2]);normal=planes[:,:3]@r.T
        transformed=np.column_stack([normal,planes[:,3]-normal@t])
        np.testing.assert_allclose(segment_inside_convex_interval(planes,a,b,.2),
            segment_inside_convex_interval(transformed,r@a+t,r@b+t,.2),atol=1e-14)

    def test_invalid_units_and_nonfinite_input_rejected(self):
        with self.assertRaisesRegex(ValueError,'Normalized'):
            segment_inside_convex_interval(box(1)*2,[-2,0,0],[2,0,0],.2)
        with self.assertRaises(ValueError):capsule_segment_review(box(1),[[0,0,0],[float('nan'),0,0]],.2)
        with self.assertRaises(ValueError):capsule_segment_review(box(1),[[0,0,0],[0,0,0]],-.2)


if __name__=='__main__':unittest.main()
