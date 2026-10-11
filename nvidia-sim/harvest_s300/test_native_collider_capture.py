import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from native_collider_capture import capture_tool_shapes


class NativeCaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.stage=Usd.Stage.CreateInMemory()
        self.owner='/World/ActuatedTool/fixed';self.path=self.owner+'/part_9'
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(self.stage,self.owner).GetPrim())
        p=UsdGeom.Mesh.Define(self.stage,self.path).GetPrim()
        UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('convexDecomposition')
        for file,content in (('scene.usda','source'),('composed.usdc','native')):(self.root/file).write_text(content)
        sha=lambda file:hashlib.sha256((self.root/file).read_bytes()).hexdigest()
        self.manifest=dict(scene_sha256=sha('scene.usda'),compact_assets=dict(materialized_dependency=dict(sha256=sha('composed.usdc'))))
        self.geometry=dict(body=self.owner,kind='native_cooked_convex',hulls=[dict(vertices=[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],normals=[[1.,0.,0.]],edges=[[0.,1.,0.]]),dict(vertices=[[2.,0.,0.],[3.,0.,0.],[2.,1.,0.],[2.,0.,1.]],normals=[[1.,0.,0.]],edges=[[0.,1.,0.]])])
        class Reader:
            def cooked(reader,path):return dict(body=self.owner,kind=self.geometry['kind'],hulls=self.geometry['hulls'])
            def geometry_metadata(reader):return {self.path:self.geometry}
            def policy_metadata(reader):return dict(helper_source_sha256='g'*64)
        self.reader=Reader()

    def capture(self,paths=None):
        return capture_tool_shapes(self.stage,self.manifest,self.reader,paths or [self.path],
            scene_path=self.root/'scene.usda',native_scene_path=self.root/'composed.usdc',
            body_paths=[self.owner],poses=np.array([[0.,0.,0.,0.,0.,0.,1.]]),
            timestamp_s=0.,effective_hz=120,output=self.root/'capture.json')

    def test_all_native_pieces_and_owner_frames_preserved_without_stage_authoring(self):
        before=self.stage.GetRootLayer().ExportToString();report=self.capture()
        self.assertEqual(report['geometry'][self.path],self.geometry)
        self.assertEqual(report['settings'][self.path]['hull_count'],2)
        self.assertEqual(report['owner_poses'][self.owner],[0.,0.,0.,0.,0.,0.,1.])
        self.assertEqual(before,self.stage.GetRootLayer().ExportToString())
        self.assertEqual(json.loads((self.root/'capture.json').read_text()),report)
        self.assertTrue(report['native_geometry_complete']);self.assertTrue(report['physics_unchanged'])

    def test_disabled_missing_duplicate_or_external_colliders_fail_closed(self):
        for paths in ([self.path]*2,['/World/Greenhouse/part_9'],[self.owner+'/missing']):
            with self.assertRaises(ValueError):self.capture(paths)
        UsdPhysics.CollisionAPI(self.stage.GetPrimAtPath(self.path)).CreateCollisionEnabledAttr(False)
        with self.assertRaises(ValueError):self.capture()
        self.assertFalse((self.root/'capture.json').exists())

    def test_incomplete_cooking_or_source_native_hash_mismatch_cannot_authorize(self):
        self.geometry['kind']='authored_static_triangle_fallback'
        with self.assertRaises(ValueError):self.capture()
        self.geometry['kind']='native_cooked_convex'
        (self.root/'composed.usdc').write_text('changed physics')
        with self.assertRaisesRegex(ValueError,'exact source'):self.capture()
        self.assertFalse((self.root/'capture.json').exists())


if __name__=='__main__':unittest.main()
