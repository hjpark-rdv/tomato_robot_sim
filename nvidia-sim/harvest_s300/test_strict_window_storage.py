import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import json

import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,Vt

from strict_window_storage import (authored_numeric_validity,canonical_layer_digest,
    materialize_exact_layer,require_identity_wrapper,require_self_contained)


class StrictWindowStorageTests(unittest.TestCase):
    def fixture(self):
        stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1.)
        UsdGeom.SetStageUpAxis(stage,'Z');UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
        world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
        actor=UsdGeom.Xform.Define(stage,'/World/Actor');actor.AddTranslateOp().Set(Gf.Vec3d(.2,.3,.4))
        mesh=UsdGeom.Mesh.Define(stage,'/World/Actor/Mesh')
        mesh.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0)])
        mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
        mesh.GetPrim().CreateRelationship('test:target').SetTargets(['/World/Actor'])
        return stage

    def test_all_authored_content_roundtrips_with_identity_wrapper_and_exact_sha(self):
        stage=self.fixture();prim=stage.GetPrimAtPath('/World/Actor')
        q=prim.CreateAttribute('test:quat',Sdf.ValueTypeNames.Quatf);q.Set(Gf.Quatf(1,Gf.Vec3f(0)))
        sample=prim.CreateAttribute('test:samples',Sdf.ValueTypeNames.Float);sample.Set(2.,1.)
        with tempfile.TemporaryDirectory() as folder:
            report=materialize_exact_layer(stage,Path(folder)/'asset')
            self.assertTrue(report['passed']);self.assertEqual(report['source_canonical'],report['roundtrip_canonical'])
            self.assertGreater(report['numeric_validity']['numeric_values'],0)
            self.assertEqual(report['numeric_validity']['time_samples'],1)
            self.assertTrue(report['identity_wrapper']['no_prim_opinions'])
            self.assertFalse(report['world_vertices_recomputed'])

    def test_nonfinite_authored_default_and_time_sample_are_rejected(self):
        for timed in (False,True):
            stage=self.fixture();p=stage.GetPrimAtPath('/World/Actor')
            a=p.CreateAttribute('test:bad',Sdf.ValueTypeNames.Float)
            a.Set(float('nan'),1.) if timed else a.Set(float('inf'))
            with self.assertRaisesRegex(ValueError,'Nonfinite authored'):
                authored_numeric_validity(stage.GetRootLayer())

    def test_nonfinite_numeric_array_and_quaternion_are_rejected(self):
        for kind,value in [(Sdf.ValueTypeNames.FloatArray,Vt.FloatArray([1,float('nan')])),
                           (Sdf.ValueTypeNames.Quatf,Gf.Quatf(float('inf'),Gf.Vec3f(0)))]:
            stage=self.fixture();stage.GetPrimAtPath('/World/Actor').CreateAttribute('test:bad',kind).Set(value)
            with self.assertRaisesRegex(ValueError,'Nonfinite authored'):
                authored_numeric_validity(stage.GetRootLayer())

    def test_one_topology_transform_or_metadata_change_changes_the_full_digest(self):
        for kind in ('topology','transform','metadata'):
            stage=self.fixture();before=canonical_layer_digest(stage.GetRootLayer())
            if kind=='topology':UsdGeom.Mesh(stage.GetPrimAtPath('/World/Actor/Mesh')).GetFaceVertexIndicesAttr().Set([0,2,1])
            if kind=='transform':stage.GetPrimAtPath('/World/Actor').GetAttribute('xformOp:translate').Set(Gf.Vec3d(.2,.3,.40001))
            if kind=='metadata':stage.GetPrimAtPath('/World/Actor').SetCustomDataByKey('unknown_schema_value',3.)
            self.assertNotEqual(before,canonical_layer_digest(stage.GetRootLayer()))

    def test_external_dependencies_and_existing_outputs_are_rejected(self):
        stage=self.fixture();stage.GetRootLayer().subLayerPaths=['external.usdc']
        with self.assertRaisesRegex(ValueError,'self-contained'):require_self_contained(stage.GetRootLayer())
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'composed.usdc').write_bytes(b'preserved')
            with self.assertRaises(FileExistsError):materialize_exact_layer(self.fixture(),p)
            self.assertEqual((p/'composed.usdc').read_bytes(),b'preserved')

    def test_wrapper_prim_opinions_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);stage=self.fixture();materialize_exact_layer(stage,root)
            wrapper=Usd.Stage.Open(str(root/'scene.usda'));wrapper.OverridePrim('/World/Actor').SetHidden(True)
            with self.assertRaisesRegex(ValueError,'only an identity'):require_identity_wrapper(wrapper,root/'composed.usdc')

    def test_opt_in_window_matches_legacy_all_resolved_properties_and_world_geometry(self):
        import route_checkpoint as rc
        from test_route_checkpoint import fixture,Kinematics,body
        from compact_motion_assets import compact_assembly,property_inventory,compare_geometry,file_sha256
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'source';_,_,metadata,arrays=fixture(source)
            assembly=root/'compact';compact_assembly(source,assembly)
            metadata.update(scene_sha256=file_sha256(assembly/'scene.usda'),source_assembly=str(assembly.resolve()),
                manifest=json.loads((assembly/'motion_manifest.json').read_text()),metadata_sha256='b'*64)
            def plant(stage,path,report,output):
                active='/World/ElasticTarget';UsdGeom.Xform.Define(stage,active)
                body(stage,active+'/rod',(1,1,0));fruit=active+'/Truss_00_Fruit_01';body(stage,fruit,(1,1,.2))
                f=dict(name='Truss_00_Fruit_01',path=fruit,joint=active+'/HarvestJoints/new',anchor=active+'/rod',center=[1,1,.2],mass=.001)
                report['plant']=dict(root=active,source_plant=path,fruits=[f],bodies=[],skin_paths=[])
                stage.GetPrimAtPath(path).SetActive(False);np.savez_compressed(output/'skin.npz')
            with patch('native_route_checkpoint.load_checkpoint',return_value=(metadata,arrays)),patch.object(rc,'ToolKinematics',Kinematics),patch.object(rc,'build_plant',plant):
                legacy=root/'legacy';fast=root/'fast'
                rc.switch_window(assembly,root/'checkpoint',legacy,19,'Truss_00_Fruit_01')
                rc.switch_window(assembly,root/'checkpoint',fast,19,'Truss_00_Fruit_01',fast_materialization=True)
            a=Usd.Stage.Open(str(legacy/'scene.usda'));b=Usd.Stage.Open(str(fast/'scene.usda'))
            self.assertEqual(property_inventory(a),property_inventory(b))
            proof=compare_geometry(a,b);self.assertEqual(proof['world_vertex_max_abs_error_m'],0.)
            self.assertEqual(proof['rigid_pose_max_abs_error'],0.)
            self.assertTrue((fast/'checkpoint_materialization_identity.json').is_file())
            self.assertFalse((fast/'checkpoint_geometry_inventory.json').exists())


if __name__=='__main__':unittest.main()
