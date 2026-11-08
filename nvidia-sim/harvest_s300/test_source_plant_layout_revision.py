import unittest
import numpy as np
from pxr import Gf,Usd,UsdGeom,UsdPhysics
from source_plant_layout_revision import apply,layout_inventory,PLANT17,TRUSS21,TRUSS18,_world


class SourceLayoutRevisionTests(unittest.TestCase):
    def fixture(self):
        stage=Usd.Stage.CreateInMemory()
        for path,position in [(PLANT17,[-.689,.9,.32]),(TRUSS21.rsplit('/',1)[0],[-.6,.7,.32]),
                (TRUSS21,[.03,.07,.8]),(TRUSS18.rsplit('/',1)[0],[-.65,.7,.32]),
                (TRUSS18,[0,0,.4]),('/World/ElasticTarget',[1,2,3])]:
            x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*position));x.AddRotateZOp().Set(17.)
            mesh=UsdGeom.Mesh.Define(stage,path+'/mesh')
            mesh.CreatePointsAttr([Gf.Vec3f(0,0,0),Gf.Vec3f(.01,0,0),Gf.Vec3f(0,.01,0)])
            mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2]);mesh.CreateDisplayColorAttr([Gf.Vec3f(.1,.6,.1)])
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        body=stage.GetPrimAtPath('/World/ElasticTarget');UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(.045)
        manifest=dict(plant=dict(source_plant='/World/Greenhouse/Plants/Plant_22'))
        return stage,manifest

    def test_rigid_source_placement_only_preserves_active_physics_and_attachment(self):
        stage,manifest=self.fixture();active=_world(stage,'/World/ElasticTarget').copy();pivot=_world(stage,TRUSS21)[:3,3].copy()
        oldroot=_world(stage,PLANT17).copy()
        report=apply(stage,manifest,expected21pivot=pivot)
        np.testing.assert_array_equal(_world(stage,'/World/ElasticTarget'),active)
        np.testing.assert_allclose(_world(stage,PLANT17)[:3,3]-oldroot[:3,3],[.015,0,0],rtol=0,atol=1e-12)
        np.testing.assert_allclose(_world(stage,TRUSS21)[:3,3],pivot,rtol=0,atol=1e-12)
        self.assertLess(report['world_vertex_rigid_transform_max_error_m'],1e-12)
        self.assertEqual(report['normalized_source_canonical'],report['normalized_derived_canonical'])
        self.assertTrue(report['all_other_authored_fields_exact'])
        self.assertEqual(UsdPhysics.MassAPI(stage.GetPrimAtPath('/World/ElasticTarget')).GetMassAttr().Get(),np.float32(.045))
        self.assertEqual(report['native_validation'],'pending');self.assertIsNone(report['harvest_success'])
        self.assertNotIn('layout_revision',manifest)

    def test_does_not_move_an_active_or_previously_visited_source_plant(self):
        for condition in ('active','visited','already_revised'):
            stage,m=self.fixture();before=stage.GetRootLayer().ExportToString()
            if condition=='active':m['plant']['source_plant']=PLANT17
            elif condition=='visited':m['route_checkpoint']=dict(outgoing_history=[dict(source_plant=TRUSS21.rsplit('/',1)[0])])
            else:m['layout_revision']={'schema':'old'}
            with self.assertRaises(ValueError):apply(stage,m)
            self.assertEqual(before,stage.GetRootLayer().ExportToString())

    def test_reviewed_pivot_and_finite_bounded_translation_are_required(self):
        for options in [dict(expected21pivot=[99,0,0]),dict(plant17_translation=[np.nan,0,0]),
                        dict(plant17_translation=[.1,0,0]),dict(plant21_yaw_deg=45)]:
            stage,m=self.fixture();before=stage.GetRootLayer().ExportToString()
            with self.assertRaises(ValueError):apply(stage,m,**options)
            self.assertEqual(before,stage.GetRootLayer().ExportToString())

    def test_inactive_missing_source_is_not_silently_repaired(self):
        stage,m=self.fixture();stage.GetPrimAtPath(PLANT17).SetActive(False)
        with self.assertRaisesRegex(ValueError,'uninitialized'):apply(stage,m)

    def test_combined_source18_yaw_keeps_attachment_and_active_plant_exact(self):
        stage,m=self.fixture();pivot=_world(stage,TRUSS18)[:3,3].copy();active=_world(stage,'/World/ElasticTarget').copy()
        r=apply(stage,m,plant17_translation=[.018,0,0],plant18_yaw_deg=-15,expected18pivot=pivot)
        self.assertEqual(r['plant18_truss02_yaw_world_z_deg'],-15)
        self.assertLess(r['plant18_attachment_pivot_error_m'],1e-12)
        self.assertEqual(len(r['source_plant_transforms']),3)
        np.testing.assert_array_equal(active,_world(stage,'/World/ElasticTarget'))
        self.assertEqual(r['normalized_source_canonical'],r['normalized_derived_canonical'])
        stage,m=self.fixture();m['plant']['source_plant']=TRUSS18.rsplit('/',1)[0]
        with self.assertRaisesRegex(ValueError,'active plant'):apply(stage,m,plant18_yaw_deg=-15)

    def test_frozen_inventory_uses_actual_mesh_pose_without_mutating_execution_manifest(self):
        import copy
        from unittest.mock import patch
        stage,m=self.fixture();m['plant']['source_plant']='/World/Greenhouse/Plants/Plant_20'
        body=UsdGeom.Xform.Define(stage,'/World/RouteFrozen/Plant_22/Truss_00_Fruit_01')
        body.AddTranslateOp().Set(Gf.Vec3d(2,3,4))
        mesh=UsdGeom.Mesh.Define(stage,str(body.GetPath())+'/Collider')
        mesh.CreatePointsAttr([Gf.Vec3f(0,0,0),Gf.Vec3f(.01,0,0),Gf.Vec3f(0,.01,0)])
        mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2]);mesh.CreateDisplayColorAttr([Gf.Vec3f(.8,.1,.1)])
        row=dict(source_plant='/World/Greenhouse/Plants/Plant_22',frozen_root='/World/RouteFrozen/Plant_22',
                 unharvested_reactivation_eligible=True,detached_fruits=[],measured_shape_preserved=True,
                 checkpoint_metadata_sha256='a'*64)
        m['route_checkpoint']=dict(outgoing_history=[row]);original=copy.deepcopy(m)
        layout=dict(neighbor_placements=[dict(id=17),dict(id=22)])
        calls=[]
        def collector(stage,layout,*,active_manifest):
            calls.append((copy.deepcopy(layout),copy.deepcopy(active_manifest)))
            return dict(trusses=[dict(plant_id=p['id'],truss_index=0,rail_near=True) for p in layout['neighbor_placements']])
        with patch('rail_assets.near_side_inventory',side_effect=collector):result=layout_inventory(stage,layout,m)
        self.assertEqual(m,original)
        self.assertEqual([r['plant_id'] for r in result['trusses']],[17,22])
        self.assertEqual(result['total_trusses'],2)
        actual=calls[1][1]['plant']['fruits'][0]['center']
        np.testing.assert_allclose(actual,[2.005,3.005,4],rtol=0,atol=1e-9)
        self.assertTrue(result['frozen_source_snapshot_provenance'][0]['coordinates_from_current_frozen_usd'])
        row['detached_fruits']=['released']
        with patch('rail_assets.near_side_inventory',side_effect=collector):
            with self.assertRaisesRegex(ValueError,'unharvested'):layout_inventory(stage,layout,m)


if __name__=='__main__':unittest.main()
