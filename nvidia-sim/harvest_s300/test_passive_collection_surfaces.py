import unittest

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from passive_collection_surfaces import BODY, PATH, SOURCE_NAME, inventory


class PassiveCollectionSurfaceTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(self.stage, BODY).GetPrim())
        self.mesh = UsdGeom.Mesh.Define(self.stage, PATH)
        p = np.array([[x, y, z] for x in (-.304, .304)
                      for y in (-.730512, .6854881) for z in (.28258565, .28458565)])
        self.mesh.GetPointsAttr().Set(np.tile(p, (12, 1)))
        self.mesh.GetFaceVertexCountsAttr().Set([3]*32)
        self.mesh.GetFaceVertexIndicesAttr().Set(list(range(96)))
        self.prim = self.mesh.GetPrim()
        self.prim.SetCustomDataByKey('sourceName', SOURCE_NAME)
        UsdPhysics.CollisionAPI.Apply(self.prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(self.prim).CreateApproximationAttr('convexDecomposition')

    def test_exact_existing_passive_cover_is_observation_only(self):
        before = self.stage.GetRootLayer().ExportToString()
        result = inventory(self.stage)
        self.assertEqual(set(result), {PATH})
        self.assertEqual(result[PATH]['body'], BODY)
        self.assertFalse(result[PATH]['collection_verified'])
        self.assertFalse(result[PATH]['physical_collision_response_changed'])
        self.assertEqual(before, self.stage.GetRootLayer().ExportToString())

    def test_wrong_source_owner_and_topology_reject(self):
        self.prim.SetCustomDataByKey('sourceName', 'arm mount')
        with self.assertRaisesRegex(ValueError, 'identity'): inventory(self.stage)
        self.prim.SetCustomDataByKey('sourceName', SOURCE_NAME)
        UsdPhysics.RigidBodyAPI.Apply(self.prim)
        with self.assertRaisesRegex(ValueError, 'passive rail carrier'): inventory(self.stage)
        self.prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        self.mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2])
        with self.assertRaisesRegex(ValueError, 'topology'): inventory(self.stage)

    def test_rotated_arm_like_surface_is_not_a_collection_cover(self):
        UsdGeom.Xformable(self.prim).AddRotateYOp().Set(30.)
        with self.assertRaisesRegex(ValueError, 'horizontal'): inventory(self.stage)

    def test_no_existing_cover_means_no_new_permission(self):
        self.assertEqual(inventory(None), {})
        self.assertEqual(inventory(Usd.Stage.CreateInMemory()), {})


class PassiveCarrierSourceInventoryTests(unittest.TestCase):
    """Small source-proven CAD fixture uses the production validation path."""
    def setUp(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        import hashlib
        from passive_collection_surfaces import CHASSIS, JOINT, BASE, _geometry_identity
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        base=PassiveCollectionSurfaceTests();base.setUp();self.stage=base.stage
        carrier=self.stage.GetPrimAtPath(BODY);api=UsdPhysics.RigidBodyAPI(carrier)
        api.CreateRigidBodyEnabledAttr(True);api.CreateKinematicEnabledAttr(False)
        UsdPhysics.MassAPI.Apply(carrier).CreateMassAttr(150.)
        joint=UsdPhysics.FixedJoint.Define(self.stage,JOINT)
        joint.CreateBody0Rel().SetTargets([BASE]);joint.CreateBody1Rel().SetTargets([BODY])
        joint.CreateJointEnabledAttr(True);joint.CreateExcludeFromArticulationAttr(True);joint.CreateCollisionEnabledAttr(False)
        self.side=CHASSIS+'/Part_0492';mesh=UsdGeom.Mesh.Define(self.stage,self.side)
        mesh.GetPointsAttr().Set([(-.3,-.7,0),(-.3,.7,0),(-.3,.7,.28),(-.3,-.7,.28)])
        mesh.GetFaceVertexCountsAttr().Set([3,3]);mesh.GetFaceVertexIndicesAttr().Set([0,1,2,0,2,3])
        mesh.GetPrim().SetCustomDataByKey('sourceName','하부 · 밀착형 좌측 측판')
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexDecomposition')
        source=self.root/'source';source.mkdir();(source/'scene.usda').write_text('#usda 1.0\n')
        digest=hashlib.sha256((source/'scene.usda').read_bytes()).hexdigest()
        rows=[];cache=UsdGeom.XformCache();frame=cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(CHASSIS)).GetInverse()
        for path in (PATH,self.side):
            prim=self.stage.GetPrimAtPath(path);geom,counts=_geometry_identity(UsdGeom.Mesh(prim))
            relative=np.asarray(cache.GetLocalToWorldTransform(prim)*frame,dtype=np.float64)
            rows.append(dict(path=path,source_path=path.replace('/S300Carrier/','/world/'),
                source_name=prim.GetCustomDataByKey('sourceName'),authored_geometry_sha256=geom,
                source_vertices=counts[0],source_triangles=counts[1],source_indices=counts[2],
                chassis_relative_matrix_sha256=hashlib.sha256(relative.tobytes()).hexdigest()))
        ref=dict(source_scene_sha256=digest,original_base_source_scene_sha256='a'*64,
                 source_geometry_sha256='b'*64,source_dependencies=[],colliders=rows)
        self.manifest=dict(rail_chassis_body=dict(source_assembly=str(source),
            source_scene_sha256=digest,original_base_source_scene_sha256='a'*64,
            carrier_body=BODY,carrier_joint=JOINT,chassis_root=CHASSIS,exclude_from_articulation=True,
            geometry=dict(source_geometry_sha256='b'*64,original_geometry_unchanged=True)),
            plant=dict(root='/World/ElasticTarget',fruits=[dict(name='Truss_02_Fruit_09',
                path='/World/ElasticTarget/Truss_02_Fruit_09',joint='/World/ElasticTarget/HarvestJoints/Truss_02_Fruit_09')]))
        self.patch=patch('passive_collection_surfaces._reference',return_value=ref)
        self.patch.start();self.addCleanup(self.patch.stop);self.source=source

    def test_all_exact_passive_source_cad_including_vertical_side_observation_only(self):
        before=self.stage.GetRootLayer().ExportToString();rows=inventory(self.stage,self.manifest)
        self.assertEqual(set(rows),{PATH,self.side})
        self.assertEqual(rows[self.side]['scope'],'released_fruit_passive_chassis_impact')
        self.assertFalse(rows[self.side]['collection_verified']);self.assertFalse(rows[self.side]['released_fruit_physics_verified'])
        self.assertEqual(before,self.stage.GetRootLayer().ExportToString())

    def test_measured_carrier_rotation_translation_does_not_change_local_source_identity(self):
        from pxr import Gf
        before=inventory(self.stage,self.manifest)
        body=UsdGeom.Xformable(self.stage.GetPrimAtPath(BODY))
        body.AddTranslateOp().Set((-.437116421,2.154978241,.008716214))
        body.AddOrientOp().Set(Gf.Quatf(.99999994,Gf.Vec3f(.000001,.000017,.000023)))
        after=inventory(self.stage,self.manifest)
        self.assertEqual(before,after)

    def test_imported_source_and_approved_input_are_immutable_provenance(self):
        from passive_collection_surfaces import save_imported_source,SOURCE_BYTES,REFERENCE_BYTES
        from pathlib import Path
        result=save_imported_source(self.root/'output')
        self.assertEqual(Path(result['path']).read_bytes(),SOURCE_BYTES)
        self.assertEqual(Path(result['approved_inventory_path']).read_bytes(),REFERENCE_BYTES)
        self.assertFalse(result['launch_snapshot_overwritten'])
        self.assertEqual(save_imported_source(self.root/'output'),result)
        Path(result['path']).write_bytes(b'foreign bytes')
        with self.assertRaisesRegex(ValueError,'differs'):save_imported_source(self.root/'output')

    def test_current_source_hash_geometry_transform_and_name_mismatches_fail_closed(self):
        import copy
        side=self.stage.GetPrimAtPath(self.side);bad=copy.deepcopy(self.manifest)
        bad['rail_chassis_body']['source_scene_sha256']='e'*64
        with self.assertRaisesRegex(ValueError,'lineage'):inventory(self.stage,bad)
        side.SetCustomDataByKey('sourceName','arm mount')
        with self.assertRaisesRegex(ValueError,'source name'):inventory(self.stage,self.manifest)
        side.SetCustomDataByKey('sourceName','하부 · 밀착형 좌측 측판');UsdGeom.Xformable(side).AddTranslateOp().Set((0.,0.,.001))
        with self.assertRaisesRegex(ValueError,'relative-transform'):inventory(self.stage,self.manifest)
        side.RemoveProperty('xformOp:translate');side.RemoveProperty('xformOpOrder')
        points=UsdGeom.Mesh(side).GetPointsAttr();data=list(points.Get());data[0]=(-.2,-.7,0);points.Set(data)
        with self.assertRaisesRegex(ValueError,'geometry/topology'):inventory(self.stage,self.manifest)
        (self.source/'scene.usda').write_text('# changed\n')
        with self.assertRaisesRegex(ValueError,'source scene SHA'):inventory(self.stage,self.manifest)

    def test_enabled_foreign_copy_wrong_owner_and_active_constraint_reject(self):
        from passive_collection_surfaces import CHASSIS,JOINT
        other=UsdGeom.Mesh.Define(self.stage,CHASSIS+'/Part_9999')
        UsdPhysics.CollisionAPI.Apply(other.GetPrim()).CreateCollisionEnabledAttr(True)
        with self.assertRaisesRegex(ValueError,'inventory changed'):inventory(self.stage,self.manifest)
        self.stage.RemovePrim(str(other.GetPath()));UsdPhysics.RigidBodyAPI.Apply(self.stage.GetPrimAtPath(self.side))
        with self.assertRaisesRegex(ValueError,'owner'):inventory(self.stage,self.manifest)
        self.stage.GetPrimAtPath(self.side).RemoveAPI(UsdPhysics.RigidBodyAPI)
        UsdPhysics.Joint(self.stage.GetPrimAtPath(JOINT)).GetBody1Rel().SetTargets(['/World/HarvestRobot/Robot/link4'])
        with self.assertRaisesRegex(ValueError,'fixed-constraint'):inventory(self.stage,self.manifest)

    def test_scope_requires_actual_native_release_in_both_orders_and_keeps_full_scene_invalid(self):
        from harvest_contact_scope import HarvestContactScope,collection_colliders_from_stage
        fruit='/World/ElasticTarget/Truss_02_Fruit_09/Collider';joint='/World/ElasticTarget/HarvestJoints/Truss_02_Fruit_09'
        paths=collection_colliders_from_stage(self.stage,self.manifest)
        for a,b in ((fruit,self.side),(self.side,fruit)):
            event=dict(a=a,b=b,t=130.6167,separation_m=-.0027065,force_n=.03594)
            scope=HarvestContactScope(self.manifest,paths,enabled=True,stage=self.stage)
            for breaks in ([],[dict(joint=joint,t=130.62)],[dict(joint=joint+'_other',t=1.)]):
                self.assertEqual(scope.filter_invalid([event],breaks),[event])
            self.assertEqual(scope.filter_invalid([event],[dict(joint=joint,t=129.)]),[])
            self.assertEqual(scope.records[0]['scope'],'released_fruit_passive_chassis_impact')
            self.assertEqual(scope.records[0]['raw_penetration_m'],.0027065)
            self.assertFalse(scope.summarize()['strict_0_5mm_passed']);self.assertFalse(scope.records[0]['collection_verified'])
            for other in ('/World/HarvestRobot/Robot/link4/Part_0492','/World/HarvestRobot/Lift/Part_0492',
                          '/World/ActuatedTool/L/part_122','/World/ElasticTarget/Stem/Collider'):
                wrong=dict(event,a=fruit,b=other)
                self.assertEqual(scope.filter_invalid([wrong],[dict(joint=joint,t=1.)]),[wrong])
            strict=HarvestContactScope(self.manifest,paths,stage=self.stage)
            self.assertEqual(strict.filter_invalid([event],[dict(joint=joint,t=129.)]),[event])


if __name__ == '__main__': unittest.main()
