"""Exact-byte compaction controls; no native physics or full-house benchmarks."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
import compact_motion_assets as compact
import test_compact_motion_assets as fixtures


class CompactByteCopyTests(unittest.TestCase):
    def make_compact(self,root):
        source=fixtures.CompactMotionAssetsTests().make_source(root)
        stage=Usd.Stage.Open(str(source/'scene.usda'))
        material=UsdShade.Material.Define(stage,'/World/Material')
        shader=UsdShade.Shader.Define(stage,'/World/Material/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.12,.61,.08))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/World/Leaf/mesh')).Bind(material)
        body=stage.GetPrimAtPath('/World/Leaf')
        body.SetCustomDataByKey('unknown:mustRemain',{'meaning':'exact','count':7})
        attr=body.CreateAttribute('diagnostic:time',Sdf.ValueTypeNames.Double)
        attr.Set(2.,Usd.TimeCode(0));attr.Set(5.,Usd.TimeCode(3))
        copy=UsdGeom.Xform.Define(stage,'/World/LeafCopy').GetPrim()
        copy.GetReferences().AddInternalReference('/World/Leaf');copy.SetInstanceable(True)
        stage.SetStartTimeCode(0.);stage.SetEndTimeCode(3.);stage.GetRootLayer().Save()
        manifest=json.loads((source/'motion_manifest.json').read_text())
        manifest['scene_sha256']=compact.file_sha256(source/'scene.usda')
        (source/'motion_manifest.json').write_text(json.dumps(manifest))
        compact.compact_assembly(source,root/'first')
        return root/'first'

    def refresh_scene_hash(self,assembly):
        manifest=json.loads((assembly/'motion_manifest.json').read_text())
        manifest['scene_sha256']=compact.file_sha256(assembly/'scene.usda')
        (assembly/'motion_manifest.json').write_text(json.dumps(manifest))

    def test_bytes_preserve_instances_materials_unknown_schema_physics_and_time_samples(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_compact(root)
            before={name:compact.file_sha256(source/name) for name in
                    ('scene.usda','composed.usdc','skin.npz','motion_manifest.json')}
            # The optimized storage path must not secretly repeat the costly
            # exhaustive inventory/geometry traversals it is meant to avoid.
            with (patch.object(compact,'property_inventory',side_effect=AssertionError('full inventory')),
                  patch.object(compact,'compare_geometry',side_effect=AssertionError('full geometry'))):
                report=compact.compact_assembly(source,root/'second')
            self.assertEqual(report['storage_copy_mode'],'verified-materialized-byte-copy-v1')
            self.assertFalse(report['world_geometry_recomputed'])
            self.assertNotIn('source_properties',report)
            for name in ('scene.usda','composed.usdc','skin.npz'):
                self.assertEqual((source/name).read_bytes(),(root/'second'/name).read_bytes())
            self.assertEqual(before,{name:compact.file_sha256(source/name) for name in before})
            left=Usd.Stage.Open(str(source/'scene.usda'));right=Usd.Stage.Open(str(root/'second/scene.usda'))
            self.assertEqual(compact.property_inventory(left),compact.property_inventory(right))
            geometry=compact.compare_geometry(left,right)
            self.assertEqual(geometry['mesh_count'],2);self.assertEqual(geometry['world_vertex_max_abs_error_m'],0.)
            self.assertTrue(right.GetPrimAtPath('/World/LeafCopy').IsInstance())
            body=right.GetPrimAtPath('/World/Leaf')
            self.assertIn('UnregisteredLeafPhysicsAPI',body.GetMetadata('apiSchemas').ApplyOperations([]))
            self.assertEqual(body.GetCustomDataByKey('unknown:mustRemain'),{'meaning':'exact','count':7})
            self.assertEqual(body.GetAttribute('diagnostic:time').GetTimeSamples(),[0.,3.])
            self.assertEqual(UsdPhysics.MassAPI(body).GetMassAttr().Get(),UsdPhysics.MassAPI(left.GetPrimAtPath('/World/Leaf')).GetMassAttr().Get())
            self.assertEqual(UsdShade.MaterialBindingAPI(right.GetPrimAtPath('/World/Leaf/mesh')).GetDirectBindingRel().GetTargets(),[Sdf.Path('/World/Material')])
            self.assertEqual(report['native_validation'],'pending');self.assertFalse(report['training_eligible'])
            self.assertEqual(compact.verify_compact_dependencies(root/'second')['sha256'],before['composed.usdc'])
            with self.assertRaises(FileExistsError):compact.compact_assembly(source,root/'second')

    def test_wrapper_runtime_opinion_forces_original_full_audit(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_compact(root)
            stage=Usd.Stage.Open(str(source/'scene.usda'))
            UsdPhysics.MassAPI(stage.GetPrimAtPath('/World/Leaf')).CreateMassAttr(.007)
            stage.GetRootLayer().Save();self.refresh_scene_hash(source)
            with patch.object(compact,'property_inventory',wraps=compact.property_inventory) as inventory:
                report=compact.compact_assembly(source,root/'second')
            self.assertEqual(inventory.call_count,2);self.assertNotIn('storage_copy_mode',report)
            stage=Usd.Stage.Open(str(root/'second/scene.usda'))
            self.assertAlmostEqual(UsdPhysics.MassAPI(stage.GetPrimAtPath('/World/Leaf')).GetMassAttr().Get(),.007)

    def test_clean_but_stale_layer_cache_does_not_hide_on_disk_wrapper_opinions(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_compact(root)
            cached=Sdf.Layer.FindOrOpen(str(source/'scene.usda'))
            isolated=Sdf.Layer.OpenAsAnonymous(str(source/'scene.usda'))
            body=Sdf.CreatePrimInLayer(isolated,'/World/Leaf')
            attr=Sdf.AttributeSpec(body,'diagnostic:diskOnly',Sdf.ValueTypeNames.Double);attr.default=8.
            isolated.Export(str(source/'scene.usda'));self.refresh_scene_hash(source)
            self.assertFalse(cached.dirty);self.assertFalse(list(cached.rootPrims))
            manifest=json.loads((source/'motion_manifest.json').read_text())
            self.assertIsNone(compact._byte_copy_candidate(source,manifest))

    def test_unknown_wrapper_metadata_and_external_asset_path_cannot_use_fast_path(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_compact(root)
            layer=Sdf.Layer.FindOrOpen(str(source/'scene.usda'))
            layer.customLayerData={'unrecognizedRuntimeMeaning':'do not infer'};layer.Save();self.refresh_scene_hash(source)
            manifest=json.loads((source/'motion_manifest.json').read_text())
            self.assertIsNone(compact._byte_copy_candidate(source,manifest))
            layer.customLayerData={};layer.Save();self.refresh_scene_hash(source)
            data=Sdf.Layer.FindOrOpen(str(source/'composed.usdc'))
            prim=data.GetPrimAtPath('/World/Material/Shader')
            attr=Sdf.AttributeSpec(prim,'inputs:file',Sdf.ValueTypeNames.Asset);attr.default=Sdf.AssetPath('local.png')
            data.Save();manifest=json.loads((source/'motion_manifest.json').read_text())
            manifest['compact_assets']['materialized_dependency']['sha256']=compact.file_sha256(source/'composed.usdc')
            (source/'motion_manifest.json').write_text(json.dumps(manifest))
            self.assertIsNone(compact._byte_copy_candidate(source,manifest))

    def test_bad_declared_hash_is_hard_error_before_output_not_fallback(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_compact(root)
            binary=source/'composed.usdc';binary.write_bytes(binary.read_bytes()+b'corrupted')
            with self.assertRaisesRegex(ValueError,'dependency SHA mismatch'):
                compact.compact_assembly(source,root/'second')
            self.assertFalse((root/'second').exists())

    def test_wrapper_time_or_unit_override_uses_full_path_instead_of_direct_copy(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_compact(root)
            stage=Usd.Stage.Open(str(source/'scene.usda'))
            stage.SetTimeCodesPerSecond(48.);stage.GetRootLayer().Save();self.refresh_scene_hash(source)
            with patch.object(compact,'property_inventory',wraps=compact.property_inventory) as inventory:
                report=compact.compact_assembly(source,root/'second')
            self.assertEqual(inventory.call_count,2);self.assertNotIn('storage_copy_mode',report)
            self.assertEqual(Usd.Stage.Open(str(root/'second/scene.usda')).GetTimeCodesPerSecond(),48.)
            # A wrapper correcting legacy dependency units is valid for the
            # original full materialization path, but not direct native reuse.
            data=Sdf.Layer.FindOrOpen(str(source/'composed.usdc'))
            data.pseudoRoot.SetInfo('metersPerUnit',.01);data.Save()
            manifest=json.loads((source/'motion_manifest.json').read_text())
            manifest['compact_assets']['materialized_dependency']['sha256']=compact.file_sha256(source/'composed.usdc')
            (source/'motion_manifest.json').write_text(json.dumps(manifest))
            self.assertIsNone(compact._byte_copy_candidate(source,manifest))

    def test_symlinked_wrapper_composition_anchor_is_not_assumed_identical(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_compact(root)
            scene=source/'scene.usda';saved=source/'original_wrapper.usda'
            scene.rename(saved);scene.symlink_to(saved.name)
            manifest=json.loads((source/'motion_manifest.json').read_text())
            self.assertEqual(compact.file_sha256(scene),manifest['scene_sha256'])
            self.assertIsNone(compact._byte_copy_candidate(source,manifest))

    def test_source_mutation_during_copy_is_hard_error_without_success_report(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=self.make_compact(root);real_copy=compact.shutil.copy2
            def mutate(src,dst):
                result=real_copy(src,dst)
                if Path(src).name=='skin.npz':
                    (source/'skin.npz').write_bytes((source/'skin.npz').read_bytes()+b'changed')
                return result
            with patch.object(compact.shutil,'copy2',side_effect=mutate):
                with self.assertRaisesRegex(ValueError,'Source assembly changed during'):
                    compact.compact_assembly(source,root/'second')
            self.assertFalse((root/'second/compact_validation.json').exists())


if __name__=='__main__':unittest.main()
