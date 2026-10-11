import copy,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics
from compact_motion_assets import compact_assembly,file_sha256
from collision_proxy_accuracy import API,CONFIG,EXACT_PATH,PREFIX,apply_accuracy,revise,validate_resume_revision

class ProxyAccuracyTests(unittest.TestCase):
    def source(self,root):
        ascii=root/'ascii';ascii.mkdir();stage=Usd.Stage.CreateNew(str(ascii/'scene.usda'))
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/World').GetPrim());UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z');UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
        body=UsdGeom.Xform.Define(stage,'/World/ActuatedTool/L');body.AddTranslateOp().Set(Gf.Vec3d(.2,.4,1.));UsdPhysics.RigidBodyAPI.Apply(body.GetPrim());UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(.6869)
        mesh=UsdGeom.Mesh.Define(stage,EXACT_PATH);mesh.CreatePointsAttr([[0,0,0],[.1,0,0],[0,.1,0],[0,0,.004]]);mesh.CreateFaceVertexCountsAttr([3]*4);mesh.CreateFaceVertexIndicesAttr([0,1,2,0,1,3,0,2,3,1,2,3]);mesh.GetPrim().SetCustomDataByKey('sourceName','L_C02_lower_deck')
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True);UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexDecomposition');UsdPhysics.FilteredPairsAPI.Apply(body.GetPrim()).CreateFilteredPairsRel().SetTargets(['/World/Arm/link6'])
        mesh.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.0001);mesh.GetPrim().CreateRelationship('material:binding:physics').SetTargets(['/World/Material'])
        material=UsdPhysics.MaterialAPI.Apply(UsdGeom.Scope.Define(stage,'/World/Material').GetPrim());material.CreateStaticFrictionAttr(.8);material.CreateDynamicFrictionAttr(.7)
        stage.GetRootLayer().Save();np.savez(ascii/'skin.npz',v=np.arange(3));manifest=dict(scene_sha256=file_sha256(ascii/'scene.usda'),plant=dict(source_plant='/World/Greenhouse/Plants/Plant_22'),target=dict(name='Truss_02_Fruit_02'),tool=dict(bodies=dict(L='/World/ActuatedTool/L')),training_eligible=False,harvest_success=None)
        (ascii/'motion_manifest.json').write_text(json.dumps(manifest));source=root/'source';compact_assembly(ascii,source)
        return source

    def test_exact_scope_and_non_accuracy_material_mass_geometry_preserved(self):
        with tempfile.TemporaryDirectory() as t:
            source=self.source(Path(t));stage=Usd.Stage.Open(str(source/'scene.usda'));before=file_sha256(source/'composed.usdc')
            derived=Usd.Stage.Open(stage.Flatten());report=apply_accuracy(derived)
            self.assertTrue(report['non_accuracy_properties_preserved']);self.assertEqual(report['geometry_audit']['world_vertex_max_abs_error_m'],0.)
            self.assertEqual(report['geometry_audit']['rigid_pose_max_abs_error'],0.);self.assertEqual(file_sha256(source/'composed.usdc'),before)
            self.assertEqual(UsdPhysics.MaterialAPI(derived.GetPrimAtPath('/World/Material')).GetDynamicFrictionAttr().Get(),UsdPhysics.MaterialAPI(stage.GetPrimAtPath('/World/Material')).GetDynamicFrictionAttr().Get())
            self.assertEqual(UsdPhysics.CollisionAPI(derived.GetPrimAtPath(EXACT_PATH)).GetCollisionEnabledAttr().Get(),True)
            self.assertEqual(UsdPhysics.MeshCollisionAPI(derived.GetPrimAtPath(EXACT_PATH)).GetApproximationAttr().Get(),'convexDecomposition')
            self.assertEqual(report['exact_collider_paths'],[EXACT_PATH]);self.assertFalse(report['full_source_triangle_enclosure_proven'])

    def test_unverified_mirror_source_identity_or_triangle_none_rejected_before_write(self):
        with tempfile.TemporaryDirectory() as t:
            source=self.source(Path(t));stage=Usd.Stage.Open(str(source/'scene.usda'));derived=Usd.Stage.Open(stage.Flatten())
            with self.assertRaisesRegex(ValueError,'independently verified'):apply_accuracy(derived,['/World/ActuatedTool/R/part_244'])
            prim=derived.GetPrimAtPath(EXACT_PATH);prim.SetCustomDataByKey('sourceName','R_C02_lower_deck')
            with self.assertRaisesRegex(ValueError,'identity'):apply_accuracy(derived)
            prim.SetCustomDataByKey('sourceName','L_C02_lower_deck');UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Set('none')
            with self.assertRaisesRegex(ValueError,'identity'):apply_accuracy(derived)
            self.assertFalse(prim.HasAttribute(PREFIX+'maxConvexHulls'))

    def test_resume_independent_current_property_audit_rejects_hidden_filter_or_friction_change(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);source=self.source(root);evidence=dict(folder='synthetic-unit-control',file_sha256={},fixture_only=True)
            with patch('collision_proxy_accuracy._fixture_evidence',return_value=evidence):revise(source,root/'derived',evidence=root/'unit-fixture')
            old=json.loads((source/'motion_manifest.json').read_text());new=json.loads((root/'derived/motion_manifest.json').read_text());metadata=dict(source_assembly=str(source),source_scene=str(source/'scene.usda'),scene_sha256=old['scene_sha256'],manifest=old)
            with patch('collision_proxy_accuracy._fixture_evidence',return_value=evidence):proof=validate_resume_revision(root/'derived',new,metadata)
            self.assertTrue(proof['authored_geometry_unchanged']);self.assertEqual(proof['penetration_limit_m'],.0005)
            stage=Usd.Stage.Open(str(root/'derived/composed.usdc'));UsdPhysics.MaterialAPI(stage.GetPrimAtPath('/World/Material')).GetDynamicFrictionAttr().Set(.01);stage.GetRootLayer().Save()
            # Even after the current manifest dependency SHA is honestly updated,
            # a forged report cannot authorize an unrelated physics change.
            new['compact_assets']['materialized_dependency']['sha256']=file_sha256(root/'derived/composed.usdc')
            with patch('collision_proxy_accuracy._fixture_evidence',return_value=evidence):
                with self.assertRaisesRegex(ValueError,'beyond the approved'):validate_resume_revision(root/'derived',new,metadata)

    def test_extra_or_removed_applied_schema_cannot_be_hidden_by_normalization(self):
        from collision_proxy_accuracy import _audit
        with tempfile.TemporaryDirectory() as t:
            source=self.source(Path(t));original=Usd.Stage.Open(str(source/'scene.usda'))
            derived=Usd.Stage.Open(original.Flatten());apply_accuracy(derived)
            prim=derived.GetPrimAtPath(EXACT_PATH)
            UsdPhysics.MassAPI.Apply(prim)
            with self.assertRaisesRegex(ValueError,'API schemas changed'):_audit(original,derived,[EXACT_PATH])
            tokens=prim.GetMetadata('apiSchemas').ApplyOperations([])
            tokens.remove('PhysicsMassAPI');tokens.remove('PhysicsCollisionAPI')
            prim.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(tokens))
            with self.assertRaises(ValueError):_audit(original,derived,[EXACT_PATH])

    def test_parent_sha_manifest_and_output_immutability(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);source=self.source(root);evidence=dict(folder='synthetic-unit-control',file_sha256={},fixture_only=True)
            with patch('collision_proxy_accuracy._fixture_evidence',return_value=evidence):revise(source,root/'derived',evidence=root/'unit-fixture')
            with self.assertRaises(FileExistsError):revise(source,root/'derived',evidence=root/'unit-fixture')
            old=json.loads((source/'motion_manifest.json').read_text());new=json.loads((root/'derived/motion_manifest.json').read_text());metadata=dict(source_assembly=str(source),source_scene=str(source/'scene.usda'),scene_sha256=old['scene_sha256'],manifest=old)
            changed=copy.deepcopy(new);changed['target']['name']='Truss_02_Fruit_01'
            with self.assertRaisesRegex(ValueError,'unrelated manifest'):validate_resume_revision(root/'derived',changed,metadata)
            (source/'skin.npz').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'parent asset SHA'):validate_resume_revision(root/'derived',new,metadata)

if __name__=='__main__':unittest.main()
