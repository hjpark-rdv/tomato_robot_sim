"""Real USD/FCL controls; synthetic caches are not native execution evidence."""
import copy,json,tempfile,unittest
from pathlib import Path
import numpy as np
import trimesh
from pxr import Gf,Usd,UsdGeom,UsdPhysics
from native_tool_hulls import FRAME,SCHEMA,SOURCE,digest,load_native_tool_hulls
from motion_self_collision import MotionSelfCollision

class NativeToolHullTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.assembly=self.root/'assembly';self.assembly.mkdir()
        s=Usd.Stage.CreateNew(str(self.assembly/'composed.usdc'));UsdGeom.SetStageMetersPerUnit(s,1.)
        for p in ('/World','/World/HarvestRobot','/World/HarvestRobot/Robot','/World/ActuatedTool'):UsdGeom.Xform.Define(s,p)
        b=UsdGeom.Xform.Define(s,'/World/ActuatedTool/fixed');b.AddTranslateOp().Set(Gf.Vec3d(.5,.2,0));b.AddRotateZOp().Set(90.)
        UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
        cube=trimesh.creation.box(extents=[.01]*3);left=cube.vertices+[-.03,0,0];right=cube.vertices+[.03,0,0]
        self.collider='/World/ActuatedTool/fixed/part_9';m=UsdGeom.Mesh.Define(s,self.collider)
        m.CreatePointsAttr(np.vstack([left,right]).tolist());m.CreateFaceVertexCountsAttr([3]*len(cube.faces)*2)
        m.CreateFaceVertexIndicesAttr(np.concatenate([cube.faces,cube.faces+8]).reshape(-1).tolist());m.AddTranslateOp().Set(Gf.Vec3d(0,.03,0))
        m.GetPrim().SetCustomDataByKey('sourceName','Two disconnected source solids')
        UsdPhysics.CollisionAPI.Apply(m.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexDecomposition')
        b=UsdGeom.Xform.Define(s,'/World/HarvestRobot/Robot/probe');b.AddTranslateOp().Set(Gf.Vec3d(.47,.2,0));UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
        c=UsdGeom.Cube.Define(s,'/World/HarvestRobot/Robot/probe/collider');c.CreateSizeAttr(.006);UsdPhysics.CollisionAPI.Apply(c.GetPrim()).CreateCollisionEnabledAttr(True)
        s.GetRootLayer().Save();s=None
        w=Usd.Stage.CreateNew(str(self.assembly/'scene.usda'));UsdGeom.SetStageMetersPerUnit(w,1.);w.GetRootLayer().subLayerPaths=['composed.usdc'];w.GetRootLayer().Save();w=None
        self.manifest={'scene_sha256':digest(self.assembly/'scene.usda'),'compact_assets':{'materialized_dependency':{'relative_path':'composed.usdc','sha256':digest(self.assembly/'composed.usdc')}}}
        (self.assembly/'motion_manifest.json').write_text(json.dumps(self.manifest));self.stage=Usd.Stage.Open(str(self.assembly/'scene.usda'))
        folder=self.root/'capture';folder.mkdir();snapshot=folder/'runtime_code_sources';snapshot.mkdir();reader=snapshot/'leaf_contact_geometry.py';reader.write_text('# Synthetic unit fixture.\n')
        (folder/'leaf_reader_source_provenance.json').write_text(json.dumps({'path':str(reader),'sha256':digest(reader)}));self.cache=folder/'native_tool_cooked_capture.json'
        def hull(v):return dict(vertices=v.tolist(),normals=np.eye(3).tolist(),edges=np.eye(3).tolist())
        self.proof=dict(schema=SCHEMA,source_scene_sha256=self.manifest['scene_sha256'],source_native_scene_sha256=self.manifest['compact_assets']['materialized_dependency']['sha256'],source_usdc_sha256=self.manifest['compact_assets']['materialized_dependency']['sha256'],source_assembly=str(self.assembly),effective_hz=120,timestamp_s=0.,captured_before_first_trial_step=True,native_engine='physx',native_geometry_complete=True,selected_paths=[self.collider],settings={self.collider:dict(owner='/World/ActuatedTool/fixed',approximation='convexDecomposition',hull_count=2)},geometry={self.collider:dict(body='/World/ActuatedTool/fixed',kind='native_cooked_convex',hulls=[hull(left+[0,.03,0]),hull(right+[0,.03,0])])},owner_poses={'/World/ActuatedTool/fixed':[.5,.2,0,0,0,float(np.sqrt(.5)),float(np.sqrt(.5))]},frame=FRAME,source=SOURCE,geometry_reader_source_sha256=digest(reader),physics_unchanged=True,training_eligible=False,harvest_success=None)
        self.write()
    def tearDown(self):self.stage=None;self.temp.cleanup()
    def write(self):self.cache.write_text(json.dumps(self.proof))
    def checker(self,cache=True):return MotionSelfCollision(self.stage,clearance_m=.002,native_tool_cache=self.cache if cache else None,assembly=self.assembly)
    def test_real_fcl_gap_and_both_piece_intersections(self):
        a=self.checker(False);b=self.checker();self.assertTrue(a.check(poses=a.rest)[0]['intersection']);self.assertEqual(b.check(poses=b.rest),[])
        self.assertEqual(len(b.shapes),3);self.assertEqual(b.metadata['clearance_m'],.002);self.assertEqual(b.metadata['native_tool_hull_import']['native_piece_count'],2)
        owner=b.rest['/World/ActuatedTool/fixed']
        for x in (-.03,.03):
            p={k:v.copy() for k,v in b.rest.items()};p['/World/HarvestRobot/Robot/probe'][:3,3]=owner[:3,:3]@np.array([x,.03,0])+owner[:3,3]
            self.assertTrue(b.check(poses=p)[0]['intersection'])
    def test_owner_frame_not_transformed_twice(self):
        pieces,meta=load_native_tool_hulls(self.stage,self.cache,assembly=self.assembly)
        self.assertTrue(np.allclose(np.vstack(pieces[self.collider]).mean(0),[0,.03,0]))
        self.assertTrue(np.allclose(np.array(meta['rows'][0]['collider_to_owner_matrix'])[:3,3],[0,.03,0]))
        self.assertTrue(all(np.array_equal(s['geometry_local'],np.eye(4)) for s in self.checker().shapes))
    def test_default_bit_identical(self):
        a=MotionSelfCollision(self.stage,clearance_m=.002);b=self.checker(False)
        self.assertEqual(a.metadata,b.metadata);self.assertEqual(a.check(poses=a.rest),b.check(poses=b.rest));self.assertTrue(np.array_equal(a.corners,b.corners));self.assertTrue(np.array_equal(a.pairs,b.pairs))
    def test_missing_piece_rejected(self):
        self.proof['geometry'][self.collider]['hulls'].pop();self.write()
        with self.assertRaisesRegex(ValueError,'coverage'):self.checker()
    def test_identity_and_provenance_controls(self):
        original=copy.deepcopy(self.proof)
        edits=[lambda p:p.update(source_scene_sha256='0'*64),lambda p:p.update(source_native_scene_sha256='0'*64),lambda p:p['geometry'][self.collider].update(body='/World/ActuatedTool/L'),lambda p:p['settings'][self.collider].update(owner='/World/ActuatedTool/L'),lambda p:p['geometry'][self.collider].update(kind='source_mesh'),lambda p:p.update(selected_paths=['/World/HarvestRobot/Robot/probe/collider']),lambda p:p.update(geometry_reader_source_sha256='0'*64),lambda p:p.update(frame='collider-local'),lambda p:p.update(native_geometry_complete=False)]
        for i,edit in enumerate(edits):
            with self.subTest(i=i):
                self.proof=copy.deepcopy(original);edit(self.proof);self.write()
                with self.assertRaises(ValueError):self.checker()
    def test_degenerate_and_nonfinite_hulls_rejected(self):
        original=copy.deepcopy(self.proof)
        for v in ([[0.,0.,0.]]*8,[[float('nan'),0.,0.]]*8):
            self.proof=copy.deepcopy(original);self.proof['geometry'][self.collider]['hulls'][1]['vertices']=v;self.write()
            with self.assertRaises(ValueError):self.checker()
    def test_runtime_layer_change_rejected(self):
        with (self.assembly/'composed.usdc').open('ab') as f:f.write(b'changed')
        with self.assertRaisesRegex(ValueError,'SHA'):self.checker()
    def test_invalid_owner_pose_rejected(self):
        self.proof['owner_poses']['/World/ActuatedTool/fixed'][6]=0.;self.write()
        with self.assertRaisesRegex(ValueError,'frame'):self.checker()
    def test_live_geometry_override_rejected(self):
        UsdGeom.Mesh(self.stage.GetPrimAtPath(self.collider)).GetPointsAttr().Set([[0,0,0]]*16)
        with self.assertRaisesRegex(ValueError,'immutable source stage'):self.checker()
if __name__=='__main__':unittest.main()
