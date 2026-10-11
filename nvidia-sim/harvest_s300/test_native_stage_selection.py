import copy
import json
from pathlib import Path
import tempfile
import unittest

from pxr import Sdf,Usd,UsdGeom,UsdPhysics

from compact_motion_assets import file_sha256
from motion_stage_metrics import copy_stage_metrics
from native_stage_selection import select_native_scene,verify_native_scene_composition


class NativeStageSelectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        native=Usd.Stage.CreateNew(str(self.root/'composed.usdc'))
        world=UsdGeom.Xform.Define(native,'/World').GetPrim();native.SetDefaultPrim(world)
        UsdGeom.SetStageMetersPerUnit(native,1.);UsdGeom.SetStageUpAxis(native,'Z');UsdPhysics.SetStageKilogramsPerUnit(native,1.)
        cube=UsdGeom.Cube.Define(native,'/World/Fruit');cube.CreateSizeAttr(.0232)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        native.GetRootLayer().Save()
        wrapper=Usd.Stage.CreateNew(str(self.root/'scene.usda'));wrapper.GetRootLayer().subLayerPaths=['composed.usdc']
        copy_stage_metrics(native,wrapper);wrapper.SetDefaultPrim(wrapper.GetPrimAtPath('/World'));wrapper.GetRootLayer().Save()
        self.manifest=dict(scene_sha256=file_sha256(self.root/'scene.usda'),
            compact_assets=dict(materialized_dependency=dict(relative_path='composed.usdc',
                sha256=file_sha256(self.root/'composed.usdc'),format='usdc',materialized=True)))
        self.write_manifest(self.manifest)

    def tearDown(self):self.tmp.cleanup()
    def write_manifest(self,value):
        (self.root/'motion_manifest.json').write_text(json.dumps(value))

    def test_default_is_unchanged_and_explicit_direct_layer_matches_properties(self):
        default=select_native_scene(self.root)
        self.assertEqual(default['native_scene'],str(self.root/'scene.usda'))
        self.assertEqual(default['mode'],'existing_public_wrapper')
        direct=select_native_scene(self.root,use_materialized=True)
        self.assertEqual(direct['native_scene'],str(self.root/'composed.usdc'))
        self.assertNotEqual(direct['native_scene_sha256'],direct['source_scene_sha256'])
        report=verify_native_scene_composition(self.root,report_path=self.root/'proof.json')
        self.assertTrue(report['wrapper_resolved_properties_match_native_layer'])
        self.assertEqual(report['source_properties'],report['native_properties'])
        self.assertIsNone(report['harvest_success']);self.assertFalse(report['source_asset_modified'])
        with self.assertRaises(FileExistsError):verify_native_scene_composition(self.root,report_path=self.root/'proof.json')

    def test_wrapper_override_cannot_be_silently_dropped(self):
        wrapper=Usd.Stage.Open(str(self.root/'scene.usda'));wrapper.GetPrimAtPath('/World/Fruit').GetAttribute('size').Set(.05)
        wrapper.GetRootLayer().Save();manifest=copy.deepcopy(self.manifest)
        manifest['scene_sha256']=file_sha256(self.root/'scene.usda');self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError,'wrapper overrides'):
            select_native_scene(self.root,use_materialized=True)
        # The default continues to use the deliberately modified public wrapper.
        self.assertEqual(select_native_scene(self.root)['native_scene'],str(self.root/'scene.usda'))

    def test_modified_dependency_or_source_and_missing_materialization_fail_closed(self):
        path=self.root/'composed.usdc';path.write_bytes(path.read_bytes()+b'tamper')
        with self.assertRaisesRegex(ValueError,'dependency SHA'):select_native_scene(self.root,use_materialized=True)
        manifest=copy.deepcopy(self.manifest);manifest.pop('compact_assets');self.write_manifest(manifest)
        with self.assertRaisesRegex(ValueError,'materialized USDC'):select_native_scene(self.root,use_materialized=True)
        (self.root/'scene.usda').write_text('# changed\n')
        with self.assertRaisesRegex(ValueError,'source wrapper SHA'):select_native_scene(self.root)


if __name__=='__main__':unittest.main()
