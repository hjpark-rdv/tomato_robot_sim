import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics

from compact_motion_assets import file_sha256,verify_compact_dependencies
from leaf_collision_geometry import apply_refinement,revise


class LeafCollisionGeometryTests(unittest.TestCase):
    def fixture(self,root):
        source=root/'source';source.mkdir();stage=Usd.Stage.CreateNew(str(source/'scene.usda'))
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/World').GetPrim())
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z');UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
        leaves=[]
        for index in (3,4):
            name=f'Leaf_CompoundLeaf_{index:02d}';body='/World/ElasticTarget/'+name
            prim=UsdGeom.Xform.Define(stage,body).GetPrim()
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(.2*index,.3,.8))
            UsdPhysics.RigidBodyAPI.Apply(prim);UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.003)
            mesh=UsdGeom.Mesh.Define(stage,body+'/Appendage_1_0')
            mesh.CreatePointsAttr([[0,0,0],[.1,0,0],[0,.08,0],[0,0,.01]])
            mesh.CreateFaceVertexCountsAttr([3,3,3,3]);mesh.CreateFaceVertexIndicesAttr([0,1,2,0,1,3,0,2,3,1,2,3])
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
            UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(['/World/ElasticTarget/Stem'])
            joint=UsdPhysics.Joint.Define(stage,'/World/ElasticTarget/Joints/'+name)
            joint.CreateBody1Rel().SetTargets([body])
            UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'rotX').CreateStiffnessAttr(.005)
            joint.GetPrim().CreateAttribute('physxJoint:armature',Sdf.ValueTypeNames.Float).Set(.000005)
            mesh.GetPrim().CreateRelationship('material:binding:physics').SetTargets(['/World/PhysicsMaterial'])
            leaves.append(dict(name=name,source_name=f'CompoundLeaf_{index:02d}',path=body,
                               meshes=[body+'/Appendage_1_0'],joint=str(joint.GetPath())))
        other=UsdGeom.Mesh.Define(stage,'/World/ElasticTarget/Fruit/Collider')
        other.CreatePointsAttr([[0,0,0],[.02,0,0],[0,.02,0]])
        other.CreateFaceVertexCountsAttr([3]);other.CreateFaceVertexIndicesAttr([0,1,2])
        UsdPhysics.CollisionAPI.Apply(other.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(other.GetPrim()).CreateApproximationAttr('convexHull')
        stage.GetRootLayer().Save();np.savez_compressed(source/'skin.npz',mapping=np.arange(4))
        manifest=dict(scene_sha256=file_sha256(source/'scene.usda'),compliant_foliage=dict(leaves=leaves),
                      plant=dict(source_plant='/World/Greenhouse/Plants/Plant_22'),training_eligible=False)
        (source/'motion_manifest.json').write_text(json.dumps(manifest))
        return source,manifest

    def test_exact_subset_changes_only_approximation_and_preserves_source(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source,manifest=self.fixture(root)
            hashes={name:file_sha256(source/name) for name in ('scene.usda','skin.npz','motion_manifest.json')}
            output=root/'refined';report=revise(source,output,['CompoundLeaf_03'])
            original=Usd.Stage.Open(str(source/'scene.usda'));derived=Usd.Stage.Open(str(output/'scene.usda'))
            for leaf in manifest['compliant_foliage']['leaves']:
                path=leaf['meshes'][0]
                self.assertEqual(UsdPhysics.MeshCollisionAPI(original.GetPrimAtPath(path)).GetApproximationAttr().Get(),'convexHull')
                self.assertEqual(UsdPhysics.MeshCollisionAPI(derived.GetPrimAtPath(path)).GetApproximationAttr().Get(),
                                 'convexDecomposition' if leaf['source_name']=='CompoundLeaf_03' else 'convexHull')
                self.assertEqual(list(UsdPhysics.FilteredPairsAPI(original.GetPrimAtPath(leaf['path'])).GetFilteredPairsRel().GetTargets()),
                                 list(UsdPhysics.FilteredPairsAPI(derived.GetPrimAtPath(leaf['path'])).GetFilteredPairsRel().GetTargets()))
            self.assertEqual(report['changed_collider_count'],1)
            self.assertEqual(report['geometry_audit']['world_vertex_max_abs_error_m'],0.)
            self.assertTrue(report['non_approximation_properties_preserved'])
            self.assertEqual(report['native_validation'],'pending')
            for name,digest in hashes.items():self.assertEqual(file_sha256(source/name),digest)
            self.assertEqual(verify_compact_dependencies(output)['relative_path'],'composed.usdc')
            self.assertFalse(any(Sdf.Layer.FindOrOpen(str(output/'composed.usdc')).GetExternalReferences()))
            self.assertEqual(UsdPhysics.MeshCollisionAPI(derived.GetPrimAtPath('/World/ElasticTarget/Fruit/Collider')).GetApproximationAttr().Get(),'convexHull')
            with self.assertRaises(FileExistsError):revise(source,output,['Leaf_CompoundLeaf_03'])

    def test_default_rule_refines_every_declared_leaf_and_can_be_reapplied_to_new_window(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source,manifest=self.fixture(root)
            report=revise(source,root/'all')
            self.assertEqual(report['selected_leaf_count'],2);self.assertEqual(report['changed_collider_count'],2)
            self.assertEqual(report['selection'],'all_compliant_leaves')
            stage=Usd.Stage.Open(str(root/'all/scene.usda'))
            repeated=apply_refinement(stage,manifest)
            self.assertEqual(repeated['changed_collider_count'],0)
            self.assertEqual(len(repeated['already_refined_colliders']),2)

    def test_partial_refined_direct_parent_records_only_actual_new_changes(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source,manifest=self.fixture(root)
            partial=root/'partial';revise(source,partial,['CompoundLeaf_03'])
            before={name:file_sha256(partial/name) for name in ('scene.usda','composed.usdc','motion_manifest.json','skin.npz')}
            report=revise(partial,root/'all_from_partial')
            self.assertEqual(report['source_assembly'],str(partial.resolve()))
            self.assertEqual(report['selected_collider_count'],2)
            self.assertEqual(report['changed_collider_count'],1)
            self.assertEqual([row['source_name'] for row in report['collider_changes']],['CompoundLeaf_04'])
            self.assertEqual(report['already_refined_colliders'],[manifest['compliant_foliage']['leaves'][0]['meshes'][0]])
            self.assertEqual(report['geometry_audit']['world_vertex_max_abs_error_m'],0.)
            for name,digest in before.items():self.assertEqual(file_sha256(partial/name),digest)
            derived=json.loads((root/'all_from_partial/motion_manifest.json').read_text())
            self.assertEqual(derived['compact_assets']['source_assembly'],str(partial.resolve()))

    def test_unknown_identity_wrong_approximation_or_changed_dependency_fails_before_output(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source,manifest=self.fixture(root)
            with self.assertRaises(ValueError):revise(source,root/'bad',['Leaf_CompoundLeaf_99'])
            self.assertFalse((root/'bad').exists())
            stage=Usd.Stage.Open(str(source/'scene.usda'))
            path=manifest['compliant_foliage']['leaves'][0]['meshes'][0]
            UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(path)).GetApproximationAttr().Set('none')
            with self.assertRaises(ValueError):apply_refinement(stage,manifest)
            UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(path)).GetApproximationAttr().Set('convexHull')
            revise(source,root/'good')
            binary=root/'good/composed.usdc';binary.write_bytes(binary.read_bytes()+b'corrupt')
            with self.assertRaisesRegex(ValueError,'dependency SHA'):revise(root/'good',root/'bad_dependency')
            self.assertFalse((root/'bad_dependency').exists())


if __name__=='__main__':unittest.main()
