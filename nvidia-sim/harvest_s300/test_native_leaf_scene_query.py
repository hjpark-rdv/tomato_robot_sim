import json
import hashlib
from pathlib import Path
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics
from native_leaf_scene_query import enclosing_box,certify_leaf_structural_clear,save_imported_source,SOURCE_BYTES,SOURCE_SHA256


class NativeLeafSceneQueryTests(unittest.TestCase):
    def fixture(self):
        stage=Usd.Stage.CreateInMemory();leaf='/World/ElasticTarget/Leaf_02/Appendage_200_2'
        structural='/World/Greenhouse/House/Environment/GutterSuspensionWire_005/Mesh'
        for path in [leaf,structural]:
            mesh=UsdGeom.Mesh.Define(stage,path)
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        mesh=UsdGeom.Mesh(stage.GetPrimAtPath(structural))
        mesh.CreatePointsAttr([[0,-.1,0],[0,.1,0],[0,0,.1]])
        mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
        hull=np.array([[x,y,z] for x in [-.001,.001] for y in [-.002,.002] for z in [-.003,.003]])
        data=dict(kind='native_cooked_convex',hulls=[(hull,np.eye(3),np.eye(3))])
        return stage,leaf,structural,data

    def test_box_contains_all_native_points_at_actual_rotated_pose(self):
        rng=np.random.default_rng(22);points=rng.normal(size=(60,3))*[.03,.01,.001]
        pose=np.r_[.5,.8,1.2,Rotation.from_euler('xyz',[25,13,65],degrees=True).as_quat()]
        box=enclosing_box(points,pose)
        world=points@Rotation.from_quat(pose[3:]).as_matrix().T+pose[:3]
        axes=Rotation.from_quat(box['quaternion_xyzw']).as_matrix()
        local=(world-box['center_m'])@axes
        self.assertTrue(np.all(np.abs(local)<=np.asarray(box['half_extent_m'])+1e-14))
        self.assertEqual(box['support_points'],60)

    def query(self,leaf,structural,intersect=False,missing_positive=False,incomplete=False):
        calls=[]
        def run(box):
            calls.append(box)
            if len(calls)==1:paths=[] if missing_positive else [structural]
            elif len(calls)==2:paths=[]
            else:paths=[leaf]+([structural] if intersect else [])
            return dict(hit_count=len(paths),complete=not incomplete,
                        hits=[dict(collision=p,rigid_body='',proto_index=4294967295) for p in paths])
        return run,calls

    def test_actual_clear_query_requires_native_controls_and_preserves_intersection(self):
        stage,leaf,structural,data=self.fixture();pose=[.04,0,0,0,0,0,1.]
        query,calls=self.query(leaf,structural)
        clear=certify_leaf_structural_clear(stage,leaf,structural,pose,data,74.075,query)
        self.assertTrue(clear['certified_clear']);self.assertTrue(clear['certified_within_limit'])
        self.assertTrue(clear['native_scene_query_verified']);self.assertEqual(clear['penetration_upper_bound_m'],0.)
        self.assertEqual(clear['t'],74.075);self.assertFalse(clear['physical_colliders_changed'])
        self.assertFalse(clear['native_cooked_triangle_vertices_verified']);self.assertEqual(len(calls),3)
        query,_=self.query(leaf,structural,intersect=True)
        overlap=certify_leaf_structural_clear(stage,leaf,structural,pose,data,74.075,query)
        self.assertFalse(overlap['certified_within_limit']);self.assertIsNone(overlap['penetration_upper_bound_m'])
        self.assertTrue(overlap['native_scene_query_verified'])

    def test_control_failure_missing_native_hull_and_incomplete_queries_fail_closed(self):
        stage,leaf,structural,data=self.fixture();pose=[.04,0,0,0,0,0,1.]
        for kwargs in [dict(missing_positive=True),dict(incomplete=True)]:
            query,_=self.query(leaf,structural,**kwargs)
            failed=certify_leaf_structural_clear(stage,leaf,structural,pose,data,0.,query)
            self.assertFalse(failed['certified_within_limit']);self.assertIn('control failed',failed['error'])
        query,_=self.query(leaf,structural)
        failed=certify_leaf_structural_clear(stage,leaf,structural,pose,dict(kind='authored_convex',hulls=data['hulls']),0.,query)
        self.assertFalse(failed['certified_clear']);self.assertIn('Actual native cooked',failed['error'])
        missing_leaf=lambda box:dict(complete=True,hit_count=1,hits=[dict(collision=structural)]) if box['center_m'][0]<100 else dict(complete=True,hit_count=0,hits=[])
        failed=certify_leaf_structural_clear(stage,leaf,structural,pose,data,0.,missing_leaf)
        self.assertFalse(failed['certified_clear']);self.assertIn('current-pose leaf',failed['error'])

    def test_large_world_float32_enclosure_evidence_is_json_serializable(self):
        stage,leaf,structural,data=self.fixture()
        pose=np.array([10.25,-12.75,3.5,0,0,0,1.],np.float32)
        query,_=self.query(leaf,structural)
        row=certify_leaf_structural_clear(stage,leaf,structural,pose,data,np.float32(74.075),query)
        restored=json.loads(json.dumps(row,allow_nan=False))
        self.assertTrue(restored['certified_clear'])
        box=restored['hull_queries'][0]['box']
        self.assertGreater(box['enclosure_guard_m'],1e-6)
        hull=data['hulls'][0][0]
        local=(hull+pose[:3]-box['center_m'])@Rotation.from_quat(box['quaternion_xyzw']).as_matrix()
        self.assertTrue(np.all(np.abs(local)<=box['half_extent_m']))

    def plane_query(self,leaf,structural,physical_center):
        calls=[]
        def run(box):
            center=np.asarray(box['center_m']);axes=Rotation.from_quat(box['quaternion_xyzw']).as_matrix()
            world_half=np.abs(axes)@np.asarray(box['half_extent_m'])
            low,high=center-world_half,center+world_half;hits=[]
            if low[0]<=0<=high[0] and low[1]<=.1 and high[1]>=-.1 and low[2]<=.1 and high[2]>=-.1:
                hits.append(structural)
            physical=np.asarray(physical_center)
            if np.all(high>=physical-.001) and np.all(low<=physical+.001):hits.append(leaf)
            calls.append(box)
            return dict(complete=True,hit_count=len(hits),hits=[dict(collision=p) for p in hits])
        return run,calls

    def test_guarded_shallow_escape_vs_deep_intersection_uses_actual_query_pose(self):
        stage,leaf,structural,data=self.fixture()
        cube=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        data['hulls']=[(cube,np.eye(3),np.eye(3))]
        for x,expected in [(.0008,True),(0.,False)]:
            pose=[x,0,0,0,0,0,1.];query,calls=self.plane_query(leaf,structural,pose[:3])
            row=certify_leaf_structural_clear(stage,leaf,structural,pose,data,12.,query)
            json.dumps(row,allow_nan=False)
            self.assertFalse(row['certified_clear']);self.assertEqual(row['certified_within_limit'],expected)
            escape=row['hull_queries'][0]['bounded_escape']
            self.assertFalse(escape['actual_geometry_translated'])
            self.assertLessEqual(len(escape['attempts']),48)
            for attempt in escape['attempts']:
                if attempt.get('queried'):
                    self.assertLessEqual(attempt['serialized_translation_norm_m']+escape['reserved_guard_m'],.0005)
            if expected:
                self.assertGreater(row['penetration_upper_bound_m'],0.)
                self.assertLessEqual(row['penetration_upper_bound_m'],.0005)
                accepted=escape['attempts'][escape['accepted_attempt_index']]
                self.assertFalse(accepted['structural_hit'])
            else:self.assertIsNone(row['penetration_upper_bound_m'])
        # The same old callback cannot authorize a stale measured pose.
        query,_=self.plane_query(leaf,structural,[0.,0.,0.])
        stale=certify_leaf_structural_clear(stage,leaf,structural,[.01,0,0,0,0,0,1.],data,12.,query)
        self.assertFalse(stale['certified_within_limit']);self.assertIn('current-pose',stale['error'])

    def test_escape_limit_and_complete_query_are_hard_requirements(self):
        stage,leaf,structural,data=self.fixture();pose=[0.,0,0,0,0,0,1.]
        query,_=self.query(leaf,structural,intersect=True)
        too_large=certify_leaf_structural_clear(stage,leaf,structural,pose,data,0.,query,max_escape_m=.001001)
        self.assertFalse(too_large['certified_within_limit']);self.assertIn('supported 1mm',too_large['error'])
        calls=[]
        def incomplete(box):
            calls.append(box)
            if len(calls)==1:paths=[structural]
            elif len(calls)==2:paths=[]
            elif len(calls)==3:paths=[structural,leaf]
            else:return dict(complete=False,hit_count=0,hits=[])
            return dict(complete=True,hit_count=len(paths),hits=[dict(collision=p) for p in paths])
        row=certify_leaf_structural_clear(stage,leaf,structural,pose,data,0.,incomplete)
        self.assertFalse(row['certified_within_limit'])
        self.assertIn('Incomplete',row['hull_queries'][0]['bounded_escape']['error'])

    def test_lazy_import_source_bytes_are_immutable_separate_provenance(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);launch=root/'code';launch.mkdir()
            (launch/'native_leaf_scene_query.py').write_bytes(b'prior launch snapshot')
            metadata=save_imported_source(root);target=Path(metadata['path'])
            self.assertEqual(target.read_bytes(),SOURCE_BYTES)
            self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(),SOURCE_SHA256)
            self.assertEqual((launch/'native_leaf_scene_query.py').read_bytes(),b'prior launch snapshot')
            self.assertEqual(save_imported_source(root),metadata)
            target.write_bytes(b'different runtime import')
            with self.assertRaisesRegex(ValueError,'never overwrite'):save_imported_source(root)


if __name__=='__main__':unittest.main()
