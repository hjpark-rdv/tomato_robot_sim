import copy
import unittest

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from background_foliage import ROOT, apply_scope, collider_paths
from compliant_foliage import world_vertices
from scipy.spatial.transform import Rotation


def mesh(stage,path,points,name):
    m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    m.CreateFaceVertexCountsAttr([3,3]);m.CreateFaceVertexIndicesAttr([0,1,2,0,2,3])
    m.CreateDisplayColorAttr([Gf.Vec3f(.2,.4,.1)]);m.GetPrim().SetCustomDataByKey('sourceName',name)
    UsdPhysics.CollisionAPI.Apply(m.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('none')
    return m.GetPrim()


def fixture():
    s=Usd.Stage.CreateInMemory();UsdGeom.Xform.Define(s,'/World')
    stem='/World/Greenhouse/Plants/Plant_23/Stem'
    source=UsdGeom.Xform.Define(s,'/World/ProtoStem')
    main=mesh(s,'/World/ProtoStem/MainStem',[[-.01,0,0],[.01,0,0],[.01,0,.2],[-.01,0,.2]],'MainStem')
    first=mesh(s,'/World/ProtoStem/mesh_0',[[.01,0,.1],[.06,0,.1],[.06,.01,.12],[.01,.01,.12]],'CompoundLeaf_03_214888')
    mesh(s,'/World/ProtoStem/mesh_1',[[.01,0,.12],[.08,0,.12],[.08,.01,.14],[.01,.01,.14]],'CompoundLeaf_03_7ed887')
    first.CreateRelationship('physics:filteredPairs').SetTargets([main.GetPath()])
    inst=UsdGeom.Xform.Define(s,stem);inst.GetPrim().GetReferences().AddInternalReference('/World/ProtoStem')
    inst.GetPrim().SetInstanceable(True);inst.AddTranslateOp().Set(Gf.Vec3d(.4,.5,.6))
    scope=dict(critical_neighbor_leaf_groups=[dict(stem=stem,group='CompoundLeaf_03',exact_intersected_meshes=[stem+'/mesh_0'])])
    return s,scope


class BackgroundFoliageTest(unittest.TestCase):
    def test_exact_world_meshes_filters_and_standalone_joint(self):
        s,scope=fixture();before=collider_paths(s);old={p:world_vertices(s,p).copy() for p in before}
        report=apply_scope(s,scope);self.assertEqual(report['selected_leaf_count'],1)
        self.assertEqual(report['main_articulation_added_dofs'],0)
        self.assertEqual(report['additional_filter_targets'],[])
        self.assertEqual(collider_paths(s),{report['path_mapping'].get(p,p) for p in before})
        for source,new in report['path_mapping'].items():
            np.testing.assert_allclose(world_vertices(s,new),old[source],rtol=0,atol=1e-9)
            prim=s.GetPrimAtPath(new);self.assertFalse(s.GetPrimAtPath(source).IsActive())
            self.assertEqual(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(),'convexDecomposition')
            self.assertEqual(list(UsdGeom.Mesh(prim).GetFaceVertexIndicesAttr().Get()),[0,1,2,0,2,3])
            self.assertEqual(list(UsdGeom.Mesh(prim).GetDisplayColorAttr().Get()),[Gf.Vec3f(.2,.4,.1)])
        row=report['leaves'][0];j=UsdPhysics.Joint(s.GetPrimAtPath(row['joint']))
        self.assertEqual(j.GetBody0Rel().GetTargets(),[])
        self.assertEqual([str(p) for p in j.GetBody1Rel().GetTargets()],[row['path']])
        self.assertTrue(j.GetExcludeFromArticulationAttr().Get())
        self.assertFalse(s.GetPrimAtPath(ROOT).HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertEqual(j.GetCollisionEnabledAttr().Get(),False)
        for axis in ('rotX','rotY','rotZ'):
            limit=UsdPhysics.LimitAPI(j.GetPrim(),axis)
            self.assertEqual([limit.GetLowAttr().Get(),limit.GetHighAttr().Get()],[-85.,85.])
        copied=s.GetPrimAtPath(report['path_mapping'][scope['critical_neighbor_leaf_groups'][0]['stem']+'/mesh_0'])
        self.assertEqual([str(x) for x in copied.GetRelationship('physics:filteredPairs').GetTargets()],
                         [scope['critical_neighbor_leaf_groups'][0]['stem']+'/MainStem'])
        self.assertTrue(report['uninstanced_source_stems'])

    def test_rejects_missing_intersection_identity_without_mutating(self):
        s,scope=fixture();before=s.GetRootLayer().ExportToString()
        scope['critical_neighbor_leaf_groups'][0]['exact_intersected_meshes']=['/World/Missing']
        with self.assertRaisesRegex(ValueError,'identity mismatch'):apply_scope(s,scope)
        self.assertEqual(before,s.GetRootLayer().ExportToString())

    def test_source_group_scope_does_not_convert_other_group(self):
        s,scope=fixture();prim=s.GetPrimAtPath('/World/ProtoStem');prim.SetInstanceable(False)
        mesh(s,'/World/ProtoStem/other',[[.01,0,.15],[.06,0,.15],[.06,.01,.17],[.01,.01,.17]],'CompoundLeaf_04')
        report=apply_scope(s,scope);other=scope['critical_neighbor_leaf_groups'][0]['stem']+'/other'
        self.assertIn(other,collider_paths(s));self.assertNotIn(other,report['path_mapping'])
        self.assertEqual(UsdPhysics.MeshCollisionAPI(s.GetPrimAtPath(other)).GetApproximationAttr().Get(),'none')

    def test_measured_background_state_preserves_world_anchor_and_no_activation_reset(self):
        from route_checkpoint import author_background_state
        s,scope=fixture();report=apply_scope(s,scope);manifest=dict(background_compliant_foliage=report)
        row=report['bodies'][0];j=UsdPhysics.Joint(s.GetPrimAtPath(row['joint']))
        pivot=j.GetLocalPos0Attr().Get();rotation=j.GetLocalRot0Attr().Get()
        pose=np.r_[.3,.4,.7,Rotation.from_euler('xyz',[.1,-.2,.3]).as_quat()]
        velocity=np.array([.01,.02,.03,.04,.05,.06])
        arrays=dict(body_paths=np.array([row['path']]),body_poses_world=pose[None],body_velocities_world=velocity[None])
        proof=author_background_state(s,manifest,arrays,'/World/Greenhouse/Plants/Plant_29')
        self.assertEqual(proof[0]['pose_world_xyzw'],pose.tolist())
        self.assertEqual(j.GetLocalPos0Attr().Get(),pivot);self.assertEqual(j.GetLocalRot0Attr().Get(),rotation)
        matrix=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(s.GetPrimAtPath(row['path']))).T
        np.testing.assert_allclose(matrix[:3,3],pose[:3],rtol=0,atol=1e-12)
        np.testing.assert_allclose(matrix[:3,:3],Rotation.from_quat(pose[3:]).as_matrix(),rtol=0,atol=1e-12)
        api=UsdPhysics.RigidBodyAPI(s.GetPrimAtPath(row['path']))
        np.testing.assert_allclose(api.GetVelocityAttr().Get(),velocity[:3],rtol=0,atol=1e-8)
        np.testing.assert_allclose(api.GetAngularVelocityAttr().Get(),np.rad2deg(velocity[3:]),rtol=0,atol=1e-6)
        with self.assertRaisesRegex(ValueError,'do not reset'):author_background_state(s,manifest,arrays,'/World/Greenhouse/Plants/Plant_23')


if __name__=='__main__':unittest.main()
