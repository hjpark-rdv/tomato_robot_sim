import tempfile,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from leaf_contact_geometry import convex_axes,full_sat_axes,separating_gap
from native_leaf_cover_query import _box,certify_hull_cover,save_imported_source,SOURCE_BYTES

class NativeTightCoverTests(unittest.TestCase):
    def fixture(self,wirecenter):
        leaf='/World/ElasticTarget/Leaf_02/Appendage_1';wire='/World/Greenhouse/House/Environment/Wire/Mesh'
        vertices=np.eye(4,3)*.01;normals,edges=convex_axes(vertices)
        signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)],float)
        wirev=signs*.0001+np.asarray(wirecenter);wn,we=convex_axes(wirev)
        def query(box):
            axes=Rotation.from_quat(box['quaternion_xyzw']).as_matrix();bv=signs*np.asarray(box['half_extent_m'])@axes.T+np.asarray(box['center_m']);bn,be=convex_axes(bv);hits=[]
            for path,v,n,e in ((leaf,vertices,normals,edges),(wire,wirev,wn,we)):
                if separating_gap(bv,v,full_sat_axes(bn,n,be,e))<=0:hits.append(dict(collision=path))
            return dict(complete=True,hits=hits,hit_count=len(hits))
        return (vertices,normals,edges),np.array([0,0,0,0,0,0,1.]),leaf,wire,query

    def test_face_enclosure_clears_empty_pca_corner_without_removing_hull(self):
        hull,pose,leaf,wire,query=self.fixture([.0045,.0045,.0045]);result=certify_hull_cover(hull,pose,leaf,wire,1.,query,controls_passed=True)
        self.assertTrue(result['certified_clear']);self.assertEqual(result['penetration_upper_bound_m'],0.)
        for row in result['queries']:
            b=row['box'];axes=Rotation.from_quat(b['quaternion_xyzw']).as_matrix()
            if row['kind'].endswith('full_hull'):
                self.assertTrue(np.all(np.abs((hull[0]-b['center_m'])@axes)<=np.asarray(b['half_extent_m'])))
        self.assertFalse(result['physical_colliders_changed']);self.assertEqual(result['limit_m'],.0005)

    def test_deep_real_convex_intersection_and_budget_remain_fail_closed(self):
        hull,pose,leaf,wire,query=self.fixture([.002,.002,.002]);result=certify_hull_cover(hull,pose,leaf,wire,1.,query,controls_passed=True,max_queries=8,max_depth=2)
        self.assertFalse(result['certified_within_limit']);self.assertIn('Finite tighter-cover',result['error']);self.assertLessEqual(len(result['queries']),8)
        self.assertIsNone(result['penetration_upper_bound_m'])
        for row in result['pruned_cells']:self.assertGreater(row['full_sat_separation_m'],row['required_guard_m'])

    def test_missing_controls_stale_pose_or_incomplete_queries_cannot_clear(self):
        hull,pose,leaf,wire,query=self.fixture([.1,.1,.1])
        result=certify_hull_cover(hull,pose,leaf,wire,1.,query)
        self.assertFalse(result['certified_within_limit']);self.assertEqual(result['queries'],[])
        stale=lambda b:dict(complete=True,hits=[],hit_count=0)
        self.assertFalse(certify_hull_cover(hull,pose,leaf,wire,1.,stale,controls_passed=True)['certified_within_limit'])
        incomplete=lambda b:dict(complete=False,hits=[dict(collision=leaf)],hit_count=1)
        self.assertFalse(certify_hull_cover(hull,pose,leaf,wire,1.,incomplete,controls_passed=True)['certified_within_limit'])
        self.assertFalse(certify_hull_cover(hull,pose,leaf,wire,float('nan'),query,controls_passed=True)['certified_within_limit'])

    def test_authored_geometry_proposals_never_override_actual_native_intersection(self):
        hull,pose,leaf,wire,query=self.fixture([.002,.002,.002])
        # A deliberately wrong source proposes strong separation; the actual
        # native-equivalent query still sees a genuine inside intersection.
        wrong=hull[0]+.1
        result=certify_hull_cover(hull,pose,leaf,wire,1.,query,controls_passed=True,
            structural_proposal_points=wrong,max_queries=24,max_depth=2)
        self.assertFalse(result['certified_within_limit'])
        proposed=[r for r in result['queries'] if r['kind']=='authored_geometry_suggested_full_hull']
        self.assertTrue(proposed);self.assertTrue(all(r['structural_hit'] for r in proposed))
        self.assertTrue(result['orientation_proposal_is_not_native_triangle_geometry'])

    def test_runtime_source_provenance_is_immutable(self):
        with tempfile.TemporaryDirectory() as t:
            meta=save_imported_source(t);path=Path(meta['path']);self.assertEqual(path.read_bytes(),SOURCE_BYTES)
            save_imported_source(t);path.write_bytes(b'changed')
            with self.assertRaises(ValueError):save_imported_source(t)

if __name__=='__main__':unittest.main()
