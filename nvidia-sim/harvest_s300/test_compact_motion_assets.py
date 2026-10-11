import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics
from compact_motion_assets import compact_assembly,file_sha256,verify_compact_dependencies
from motion_stage_metrics import copy_stage_metrics


class CompactMotionAssetsTests(unittest.TestCase):
    def make_source(self,root):
        source=root/'source';source.mkdir();base=Usd.Stage.CreateNew(str(source/'base.usda'))
        base.SetDefaultPrim(UsdGeom.Xform.Define(base,'/World').GetPrim())
        UsdGeom.SetStageMetersPerUnit(base,1.);UsdGeom.SetStageUpAxis(base,'Z');UsdPhysics.SetStageKilogramsPerUnit(base,1.)
        body=UsdGeom.Xform.Define(base,'/World/Leaf').GetPrim()
        UsdGeom.Xformable(body).AddTranslateOp().Set(Gf.Vec3d(.12,-.2,.75))
        UsdGeom.Xformable(body).AddRotateZOp().Set(35.)
        UsdPhysics.RigidBodyAPI.Apply(body);UsdPhysics.MassAPI.Apply(body).CreateMassAttr(.003)
        body.AddAppliedSchema('UnregisteredLeafPhysicsAPI')
        mesh=UsdGeom.Mesh.Define(base,'/World/Leaf/mesh')
        mesh.CreatePointsAttr([Gf.Vec3f(0,0,0),Gf.Vec3f(.1,0,0),Gf.Vec3f(0,.08,.01)])
        mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.1,.5,.05)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        joint=UsdPhysics.Joint.Define(base,'/World/LeafJoint');joint.CreateBody1Rel().SetTargets(['/World/Leaf'])
        joint.GetPrim().CreateAttribute('physxJoint:armature',Sdf.ValueTypeNames.Float).Set(.000005)
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'rotX').CreateStiffnessAttr(.005)
        base.GetRootLayer().Save()
        stage=Usd.Stage.CreateNew(str(source/'scene.usda'));stage.GetRootLayer().subLayerPaths=['base.usda']
        copy_stage_metrics(base,stage);stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
        stage.GetPrimAtPath('/World/Leaf').CreateAttribute('diagnostic:scalar',Sdf.ValueTypeNames.Double).Set(4.2)
        stage.GetRootLayer().Save()
        np.savez_compressed(source/'skin.npz',mapping=np.arange(5))
        (source/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=file_sha256(source/'scene.usda'),
            training_eligible=False,plant={'identity':'preserved'},target={'name':'exact'})))
        return source

    def test_layered_source_materializes_losslessly_with_explicit_units_and_dependency_hash(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_source(root);original={n:file_sha256(source/n)for n in ['scene.usda','base.usda','skin.npz','motion_manifest.json']}
            report=compact_assembly(source,root/'compact')
            self.assertEqual(report['world_vertex_max_abs_error_m'],0.)
            self.assertEqual(report['rigid_pose_max_abs_error'],0.)
            self.assertEqual(report['mesh_count'],1);self.assertEqual(report['collider_count'],1)
            self.assertEqual(report['source_properties'],report['compact_properties'])
            self.assertEqual(report['native_validation'],'pending')
            self.assertEqual(Sdf.Layer.FindOrOpen(str(root/'compact/composed.usdc')).subLayerPaths,[])
            wrapper=Usd.Stage.Open(str(root/'compact/scene.usda'))
            original_stage=Usd.Stage.Open(str(source/'scene.usda'))
            self.assertEqual(list(wrapper.GetRootLayer().subLayerPaths),['composed.usdc'])
            self.assertIn('UnregisteredLeafPhysicsAPI',
                          wrapper.GetPrimAtPath('/World/Leaf').GetMetadata('apiSchemas').ApplyOperations([]))
            self.assertEqual(wrapper.GetPrimAtPath('/World/LeafJoint').GetAttribute('physxJoint:armature').Get(),
                             original_stage.GetPrimAtPath('/World/LeafJoint').GetAttribute('physxJoint:armature').Get())
            m=json.loads((root/'compact/motion_manifest.json').read_text());self.assertEqual(m['target'],{'name':'exact'})
            self.assertEqual(m['plant'],{'identity':'preserved'});self.assertFalse(m['training_eligible'])
            self.assertEqual(verify_compact_dependencies(root/'compact')['format'],'usdc')
            for name,digest in original.items():self.assertEqual(file_sha256(source/name),digest)
            with self.assertRaises(FileExistsError):compact_assembly(source,root/'compact')

    def test_corrupted_dependency_and_mismatched_source_cannot_be_used(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_source(root);compact_assembly(source,root/'compact')
            binary=root/'compact/composed.usdc';binary.write_bytes(binary.read_bytes()+b'changed')
            with self.assertRaisesRegex(ValueError,'dependency SHA mismatch'):verify_compact_dependencies(root/'compact')
            manifest=json.loads((source/'motion_manifest.json').read_text());manifest['scene_sha256']='0'*64
            (source/'motion_manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError,'Source assembly SHA mismatch'):compact_assembly(source,root/'other')
            self.assertFalse((root/'other').exists())

    def test_dependency_cannot_escape_the_assembly(self):
        with self.assertRaisesRegex(ValueError,'inside its assembly'):
            verify_compact_dependencies('/tmp',dict(compact_assets=dict(materialized_dependency=dict(relative_path='../escape'))))

    def test_existing_internal_prototype_references_are_not_external_dependencies(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_source(root);stage=Usd.Stage.Open(str(source/'scene.usda'))
            copy=UsdGeom.Xform.Define(stage,'/World/LeafCopy').GetPrim()
            copy.GetReferences().AddInternalReference('/World/Leaf');copy.SetInstanceable(True)
            stage.GetRootLayer().Save();m=json.loads((source/'motion_manifest.json').read_text())
            m['scene_sha256']=file_sha256(source/'scene.usda');(source/'motion_manifest.json').write_text(json.dumps(m))
            report=compact_assembly(source,root/'compact')
            self.assertEqual(report['mesh_count'],2)
            layer=Sdf.Layer.FindOrOpen(str(root/'compact/composed.usdc'))
            self.assertFalse(any(layer.GetExternalReferences()))


if __name__=='__main__':unittest.main()
