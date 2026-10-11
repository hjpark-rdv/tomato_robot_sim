import unittest
import numpy as np
from leaf_contact_geometry import separating_gap, convex_axes, full_sat_axes, convex_capsule_gap, enclosing_component_hulls, CookedLeafContactGeometry
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


class LeafContactGeometryTests(unittest.TestCase):
    def test_clear_intersection_and_touching_are_distinct(self):
        a=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]],float)
        self.assertAlmostEqual(separating_gap(a,a+[3,0,0],np.eye(3)),1.)
        self.assertLess(separating_gap(a,a+[1,0,0],np.eye(3)),0)
        self.assertEqual(separating_gap(a,a+[2,0,0],np.eye(3)),0)

    def test_edge_cross_axis_resolves_face_normal_false_intersection(self):
        a=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]],float)*[.01,.0035,.0025]
        rot=np.array([[-.20423723223779924,.9154314910157642,.34680302510863137],
                      [.7317120904781618,-.09258168804343717,.6752970070177315],
                      [.6502957554958525,.3916807581381583,-.6509236622587904]])
        b=a@rot.T+np.array([-.8811588272089262,.9619212993310635,-.665339509876471])*.01
        an,ae=convex_axes(a);bn,be=convex_axes(b)
        self.assertLess(separating_gap(a,b,np.r_[an,bn]),-.002)
        self.assertGreater(separating_gap(a,b,full_sat_axes(an,bn,ae,be)),.003)

    def test_policy_is_exact_leaf_and_tool_only(self):
        leaf='/World/ElasticTarget/Leaf_02/Appendage_1_0'
        c=CookedLeafContactGeometry(None,dict(compliant_foliage=dict(leaves=[dict(meshes=[leaf])])),[])
        self.assertTrue(c.eligible(leaf,'/World/ActuatedTool/R/part_244'))
        self.assertFalse(c.eligible(leaf,'/World/HarvestRobot/Robot/link4'))
        self.assertFalse(c.eligible('/World/ElasticTarget/Stem/Collider','/World/ActuatedTool/R/part_244'))

    def test_background_leaf_identity_does_not_authorize_stem_or_fruit(self):
        stage=Usd.Stage.CreateInMemory()
        leaf='/World/ElasticTarget/Leaf_02/Appendage_1_0'
        other='/World/Greenhouse/Plants/Plant_02/Stem/mesh_149'
        p=UsdGeom.Mesh.Define(stage,other).GetPrim()
        c=CookedLeafContactGeometry(stage,dict(compliant_foliage=dict(leaves=[dict(meshes=[leaf])])),[])
        for name, allowed in [('CompoundLeaf_06_b72fb0',True),('MainStem_06',False),('Fruit_06',False),('',False)]:
            p.SetCustomDataByKey('sourceName',name)
            self.assertEqual(c.eligible(leaf,other),allowed)

    def test_poststep_certificate_preserves_half_mm_limit(self):
        leaf='/World/ElasticTarget/Leaf_02/Appendage_1_0';tool='/World/ActuatedTool/R/part_244'
        c=CookedLeafContactGeometry(None,dict(compliant_foliage=dict(leaves=[dict(meshes=[leaf])])),['leaf','tool'])
        v=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        c.cooked=lambda path:dict(body='leaf' if path==leaf else 'tool',hulls=[(v,np.eye(3))])
        poses=np.array([[0,0,0,0,0,0,1],[.0016,0,0,0,0,0,1.]])
        r=c.verify(leaf,tool,poses,1.,-.0014)
        self.assertTrue(r['certified_within_limit']);self.assertFalse(r['certified_clear'])
        self.assertAlmostEqual(r['penetration_upper_bound_m'],.0004)
        poses[1,0]=.0014
        self.assertFalse(c.verify(leaf,tool,poses,1.,-.0014)['certified_within_limit'])
        c.cooked=lambda path: (_ for _ in ()).throw(ValueError('unavailable'))
        self.assertFalse(c.verify(leaf,tool,poses,1.,-.0014)['certified_within_limit'])

    def triangle_fixture(self, closed=False):
        stage=Usd.Stage.CreateInMemory()
        leaf='/World/ElasticTarget/Leaf_02/Appendage_1_0'
        other='/World/Greenhouse/Plants/Plant_02/Stem/mesh_149'
        mesh=UsdGeom.Mesh.Define(stage,other);prim=mesh.GetPrim()
        prim.SetCustomDataByKey('sourceName','CompoundLeaf_06')
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
        if closed:
            v=np.array([[x,y,z] for x in [-.01,.01] for y in [-.01,.01] for z in [-.01,.01]])
            f=ConvexHull(v).simplices
        else:
            v=np.array([[-.1,-.1,0],[.1,-.1,0],[0,.1,0]])
            f=np.array([[0,1,2]])
        mesh.CreatePointsAttr(v.tolist());mesh.CreateFaceVertexIndicesAttr(f.ravel().tolist())
        mesh.CreateFaceVertexCountsAttr([3]*len(f))
        c=CookedLeafContactGeometry(stage,dict(compliant_foliage=dict(leaves=[dict(meshes=[leaf])])),['leaf'])
        small=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        n,e=convex_axes(small)
        triangle=c.static_triangle_data(other)
        c.cooked=lambda path:dict(body='leaf',hulls=[(small,n,e)]) if path==leaf else triangle
        return c,leaf,other,mesh

    def test_static_triangle_fallback_certifies_only_clear_surface(self):
        c,leaf,other,_=self.triangle_fixture()
        pose=np.array([[0.,0.,.002,0.,0.,0.,1.]])
        clear=c.verify(leaf,other,pose,1.,-.002)
        self.assertTrue(clear['certified_clear'])
        self.assertTrue(clear['certified_within_limit'])
        self.assertFalse(clear['native_cooked_triangle_verified'])
        pose[0,2]=.0009
        crossing=c.verify(leaf,other,pose,2.,-.002)
        self.assertFalse(crossing['certified_within_limit'])
        self.assertIsNone(crossing['penetration_upper_bound_m'])

    def test_closed_background_volume_containment_cannot_look_clear(self):
        c,leaf,other,_=self.triangle_fixture(closed=True)
        pose=np.array([[0.,0.,0.,0.,0.,0.,1.]])
        contained=c.verify(leaf,other,pose,1.,-.002)
        self.assertFalse(contained['certified_within_limit'])
        self.assertIn('inside a closed',contained['error'])
        pose[0,0]=.03
        self.assertTrue(c.verify(leaf,other,pose,2.,-.002)['certified_clear'])

    def test_component_enclosure_is_opt_in_shallow_pass_and_deep_fail(self):
        c,leaf,other,_=self.triangle_fixture(closed=True)
        pose=np.array([[.0106,0.,0.,0.,0.,0.,1.]])
        self.assertFalse(c.verify_background_component_hulls)
        self.assertFalse(c.verify(leaf,other,pose,1.,-.002)['certified_within_limit'])
        c.verify_background_component_hulls=True
        shallow=c.verify(leaf,other,pose,2.,-.002)
        self.assertTrue(shallow['certified_within_limit']);self.assertFalse(shallow['certified_clear'])
        self.assertAlmostEqual(shallow['penetration_upper_bound_m'],.0004)
        self.assertFalse(shallow['native_cooked_triangle_verified'])
        self.assertEqual(shallow['enclosing_component_hull_certificate']['hull_component_pair_count'],1)
        pose[0,0]=.0104
        deep=c.verify(leaf,other,pose,3.,-.002)
        self.assertFalse(deep['certified_within_limit'])
        self.assertFalse(deep['enclosing_component_hull_certificate']['certified_within_limit'])
        pose[0,0]=0.
        inside=c.verify(leaf,other,pose,4.,-.002)
        self.assertFalse(inside['certified_within_limit'])
        self.assertIn('inside a closed',inside['error'])

    def test_component_enclosure_checks_every_disconnected_component(self):
        c,leaf,other,mesh=self.triangle_fixture(closed=True)
        vertices=np.asarray(mesh.GetPointsAttr().Get(),float)
        faces=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),int).reshape(-1,3)
        mesh.GetPointsAttr().Set(np.r_[vertices,vertices+[.05,0,0]].tolist())
        mesh.GetFaceVertexIndicesAttr().Set(np.r_[faces,faces+len(vertices)].ravel().tolist())
        mesh.GetFaceVertexCountsAttr().Set([3]*(2*len(faces)))
        triangle=c.static_triangle_data(other)
        small=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        n,e=convex_axes(small)
        c.cooked=lambda path:dict(body='leaf',hulls=[(small,n,e)]) if path==leaf else triangle
        c.verify_background_component_hulls=True
        pose=np.array([[.0106,0.,0.,0.,0.,0.,1.]])
        first=c.verify(leaf,other,pose,1.,-.002)
        self.assertTrue(first['certified_within_limit'])
        self.assertEqual(first['enclosing_component_hull_certificate']['component_count'],2)
        self.assertEqual(first['enclosing_component_hull_certificate']['hull_component_pair_count'],2)
        pose[0,0]=.0604
        second_deep=c.verify(leaf,other,pose,2.,-.002)
        self.assertFalse(second_deep['certified_within_limit'])
        gaps=second_deep['enclosing_component_hull_certificate']['signed_gap_m'][0]
        self.assertGreater(gaps[0],.03);self.assertLess(gaps[1],-.0005)
        # Support projection retains every authored point, including interiors.
        components=enclosing_component_hulls(triangle)
        self.assertEqual(sum(len(x['vertices']) for x in components),16)
        self.assertIn('not a global union',first['enclosing_component_hull_certificate']['interpretation'])

    def test_degenerate_component_keeps_existing_clear_only_fallback(self):
        c,leaf,other,_=self.triangle_fixture()
        c.verify_background_component_hulls=True
        pose=np.array([[0.,0.,.002,0.,0.,0.,1.]])
        clear=c.verify(leaf,other,pose,1.,-.002)
        self.assertTrue(clear['certified_clear'])
        self.assertIn('enclosing_component_hull_error',clear)
        pose[0,2]=.0009
        crossing=c.verify(leaf,other,pose,2.,-.002)
        self.assertFalse(crossing['certified_within_limit'])
        self.assertIsNone(crossing['penetration_upper_bound_m'])

    def test_overconservative_component_hull_keeps_clear_triangle_fallback(self):
        c,leaf,other,mesh=self.triangle_fixture(closed=True)
        faces=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),int).reshape(-1,3)[1:]
        mesh.GetFaceVertexIndicesAttr().Set(faces.ravel().tolist())
        mesh.GetFaceVertexCountsAttr().Set([3]*len(faces))
        triangle=c.static_triangle_data(other)
        self.assertFalse(triangle['components'][0][1])
        small=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        n,e=convex_axes(small)
        c.cooked=lambda path:dict(body='leaf',hulls=[(small,n,e)]) if path==leaf else triangle
        c.verify_background_component_hulls=True
        pose=np.array([[0.,0.,0.,0.,0.,0.,1.]])
        result=c.verify(leaf,other,pose,1.,-.002)
        self.assertFalse(result['enclosing_component_hull_certificate']['certified_within_limit'])
        self.assertTrue(result['certified_clear']);self.assertTrue(result['certified_within_limit'])
        self.assertIsNone(result['penetration_upper_bound_m'])

    def test_triangle_fallback_rejects_source_change_or_wrong_geometry(self):
        c,leaf,other,mesh=self.triangle_fixture()
        pose=np.array([[0.,0.,.002,0.,0.,0.,1.]])
        mesh.GetPointsAttr().Set([[-.1,-.1,.0001],[.1,-.1,0],[0,.1,0]])
        changed=c.verify(leaf,other,pose,1.,-.002)
        self.assertFalse(changed['certified_within_limit'])
        self.assertIn('source geometry or transform changed',changed['error'])
        UsdPhysics.MeshCollisionAPI(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        with self.assertRaises(ValueError):c.static_triangle_data(other)

    def test_fruit_verification_is_opt_in_and_exact_current_truss_tool(self):
        stage=Usd.Stage.CreateInMemory()
        fruit='/World/ElasticTarget/Truss_02_Fruit_07/Collider'
        other='/World/ElasticTarget/Truss_01_Fruit_07/Collider'
        tool='/World/ActuatedTool/R_belt/PrototypeTread'
        for path in [fruit,other,tool]:
            UsdPhysics.CollisionAPI.Apply(UsdGeom.Mesh.Define(stage,path).GetPrim()).CreateCollisionEnabledAttr(True)
        manifest=dict(target=dict(name='Truss_02_Fruit_07',path=fruit.rsplit('/',1)[0]),
                      plant=dict(root='/World/ElasticTarget',fruits=[
                          dict(name='Truss_02_Fruit_07',path=fruit.rsplit('/',1)[0]),
                          dict(name='Truss_01_Fruit_07',path=other.rsplit('/',1)[0])]))
        default=CookedLeafContactGeometry(stage,manifest,['fruit','tool'])
        self.assertFalse(default.eligible(fruit,tool))
        c=CookedLeafContactGeometry(stage,manifest,['fruit','tool'],verify_fruit_contacts=True)
        self.assertTrue(c.eligible(fruit,tool));self.assertTrue(c.eligible(tool,fruit))
        self.assertFalse(c.eligible(other,tool))
        self.assertFalse(c.eligible(fruit,'/World/HarvestRobot/Robot/link4'))
        self.assertFalse(c.eligible(fruit,'/World/ActuatedTool/unknown'))
        v=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        n,e=convex_axes(v)
        c.cooked=lambda path:dict(body='fruit' if path==fruit else 'tool',hulls=[(v,n,e)])
        poses=np.array([[0,0,0,0,0,0,1],[.0016,0,0,0,0,0,1.]])
        row=c.verify(fruit,tool,poses,1.,-.0006)
        self.assertTrue(row['certified_within_limit'])
        self.assertEqual(row['contact_scope'],'current_truss_fruit_tool')
        poses[1,0]=.0014
        self.assertFalse(c.verify(fruit,tool,poses,1.,-.0006)['certified_within_limit'])
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(tool)).CreateCollisionEnabledAttr(False)
        self.assertFalse(c.eligible(fruit,tool))

    def capsule_fixture(self):
        stage=Usd.Stage.CreateInMemory();root='/World/ElasticTarget'
        fruit=root+'/Truss_02_Fruit_07/Collider'
        owner=root+'/Truss_02_Pedicel_proximal_09_1';capsule_path=owner+'/Collider'
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,owner).GetPrim())
        capsule=UsdGeom.Capsule.Define(stage,capsule_path)
        capsule.CreateRadiusAttr(.001);capsule.CreateHeightAttr(.004);capsule.CreateAxisAttr('Z')
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim()).CreateCollisionEnabledAttr(True)
        manifest=dict(target=dict(name='Truss_02_Fruit_07',path=fruit.rsplit('/',1)[0]),
                      plant=dict(root=root,fruits=[dict(name='Truss_02_Fruit_07',path=fruit.rsplit('/',1)[0])],
                                 bodies=[dict(name=owner.rsplit('/',1)[1],path=owner)]))
        c=CookedLeafContactGeometry(stage,manifest,['fruit',owner],verify_fruit_contacts=True)
        v=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        n,e=convex_axes(v)
        c.cooked=lambda path:dict(body='fruit',hulls=[(v,n,e)]) if path==fruit else c.capsule_data(path)
        return c,fruit,capsule_path,capsule,v,n,e

    def test_exact_capsule_support_clear_shallow_and_deep(self):
        _,_,_,_,v,n,e=self.capsule_fixture()
        segment=np.array([[.0021,0,-.002],[.0021,0,.002]])
        self.assertAlmostEqual(convex_capsule_gap(v,n,e,segment,.001),.0001)
        segment[:,0]=.0016
        self.assertAlmostEqual(convex_capsule_gap(v,n,e,segment,.001),-.0004)
        segment[:,0]=.0014
        self.assertAlmostEqual(convex_capsule_gap(v,n,e,segment,.001),-.0006)

    def test_capsule_postsolve_threshold_and_measured_rotation(self):
        c,fruit,path,_,_,_,_=self.capsule_fixture()
        self.assertTrue(c.eligible(fruit,path))
        poses=np.array([[0,0,0,0,0,0,1],[.0016,0,0,0,0,0,1.]])
        result=c.verify(fruit,path,poses,1.,-.0006)
        self.assertTrue(result['certified_within_limit'])
        self.assertAlmostEqual(result['penetration_upper_bound_m'],.0004)
        self.assertEqual(result['contact_scope'],'current_truss_fruit_own_capsule')
        poses[1,0]=.0014
        self.assertFalse(c.verify(fruit,path,poses,1.,-.0006)['certified_within_limit'])
        # Rotating the segment into X extends its radius envelope toward the
        # fruit despite unchanged origin; an origin-only check would miss it.
        poses[1,0]=.0031
        self.assertTrue(c.verify(fruit,path,poses,1.,-.0006)['certified_clear'])
        poses[1,3:]=Rotation.from_euler('y',90,degrees=True).as_quat()
        self.assertFalse(c.verify(fruit,path,poses,1.,-.0006)['certified_within_limit'])

    def test_capsule_scope_scale_axis_and_owner_fail_closed(self):
        c,fruit,path,capsule,_,_,_=self.capsule_fixture()
        self.assertFalse(c.eligible(fruit,'/World/ElasticTarget/Stem_MainStem_9/Collider'))
        self.assertFalse(c.eligible(fruit,'/World/ElasticTarget/Truss_01_Pedicel_proximal_09_1/Collider'))
        capsule.GetAxisAttr().Set('Q')
        with self.assertRaises(ValueError):c.capsule_data(path)
        capsule.GetAxisAttr().Set('Z')
        capsule.AddScaleOp().Set((2.,1.,1.))
        with self.assertRaises(ValueError):c.capsule_data(path)
        capsule.ClearXformOpOrder()
        owner=UsdGeom.Xformable(capsule.GetPrim().GetParent())
        owner.AddScaleOp().Set((-1.,1.,1.))
        with self.assertRaises(ValueError):c.capsule_data(path)
        owner.ClearXformOpOrder()
        c.body_ids={}
        with self.assertRaises(ValueError):c.capsule_data(path)

    def test_cycle_retarget_changes_exact_scope_without_resetting_cache_or_evidence(self):
        stage=Usd.Stage.CreateInMemory();root='/World/ElasticTarget';tool='/World/ActuatedTool/R/part_244'
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Mesh.Define(stage,tool).GetPrim()).CreateCollisionEnabledAttr(True)
        fruits=[dict(name=f'Truss_{truss:02d}_Fruit_07',path=f'{root}/Truss_{truss:02d}_Fruit_07') for truss in [1,2]]
        bodies=[dict(name=f'Truss_{truss:02d}_Rachis_0',path=f'{root}/Truss_{truss:02d}_Rachis_0') for truss in [1,2]]
        manifest=dict(target=dict(name=fruits[1]['name'],path=fruits[1]['path']),
                      plant=dict(root=root,fruits=fruits,bodies=bodies))
        c=CookedLeafContactGeometry(stage,manifest,[],verify_fruit_contacts=True)
        c.cache['previous_shape']={'native_cooked':True};c.records.append({'t':1.})
        self.assertTrue(c.eligible(fruits[1]['path']+'/Collider',tool))
        c.retarget_fruit_truss(manifest,fruits[0]['name'])
        self.assertTrue(c.eligible(fruits[0]['path']+'/Collider',tool))
        self.assertFalse(c.eligible(fruits[1]['path']+'/Collider',tool))
        self.assertEqual(c.capsules,{bodies[0]['path']+'/Collider'})
        self.assertEqual(c.cache,{'previous_shape':{'native_cooked':True}})
        self.assertEqual(c.records,[{'t':1.}]);self.assertEqual(len(c.retarget_history),2)
        before=set(c.fruits)
        with self.assertRaises(ValueError):c.retarget_fruit_truss(manifest,'Truss_04_Fruit_07')
        self.assertEqual(c.fruits,before)


class CurrentTrussFruitPairTests(unittest.TestCase):
    def test_exact_same_truss_pair_opt_in_solved_pose_and_deep_control(self):
        stage=Usd.Stage.CreateInMemory();root='/World/ElasticTarget'
        names=['Truss_02_Fruit_09','Truss_02_Fruit_07','Truss_01_Fruit_07']
        paths=[root+'/'+name+'/Collider' for name in names]
        for path in paths:
            UsdPhysics.CollisionAPI.Apply(UsdGeom.Mesh.Define(stage,path).GetPrim()).CreateCollisionEnabledAttr(True)
        manifest=dict(target=dict(name=names[0],path=paths[0].rsplit('/',1)[0]),plant=dict(root=root,
            fruits=[dict(name=name,path=path.rsplit('/',1)[0]) for name,path in zip(names,paths)]))
        default=CookedLeafContactGeometry(stage,manifest,['fruit09','fruit07'])
        self.assertFalse(default.eligible(paths[0],paths[1]))
        c=CookedLeafContactGeometry(stage,manifest,['fruit09','fruit07'],verify_fruit_contacts=True)
        self.assertTrue(c.eligible(paths[0],paths[1]));self.assertTrue(c.eligible(paths[1],paths[0]))
        self.assertFalse(c.eligible(paths[0],paths[2]));self.assertFalse(c.eligible(paths[0],paths[0]))
        vertices=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        normals,edges=convex_axes(vertices)
        c.cooked=lambda path:dict(body='fruit09' if path==paths[0] else 'fruit07',
                                 hulls=[(vertices,normals,edges)],kind='native_cooked_convex')
        poses=np.array([[0,0,0,0,0,0,1],[.0016,0,0,0,0,0,1.]])
        row=c.verify(paths[0],paths[1],poses,129.2667,-.000923)
        self.assertEqual(row['contact_scope'],'current_truss_fruit_fruit')
        self.assertTrue(row['certified_within_limit'])
        self.assertEqual(row['native_callback_separation_m'],-.000923)
        self.assertAlmostEqual(row['penetration_upper_bound_m'],.0004)
        poses[1,0]=.0014
        self.assertFalse(c.verify(paths[0],paths[1],poses,129.2667,-.000923)['certified_within_limit'])
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(paths[1])).CreateCollisionEnabledAttr(False)
        self.assertFalse(c.eligible(paths[0],paths[1]))


class CurrentTrussToolCapsuleTests(unittest.TestCase):
    def test_exact_tool_mesh_own_capsule_shallow_deep_and_identity_controls(self):
        c,fruit,path,capsule,v,n,e=LeafContactGeometryTests().capsule_fixture()
        tool='/World/ActuatedTool/R_belt/PrototypeTread'
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Mesh.Define(c.stage,tool).GetPrim()).CreateCollisionEnabledAttr(True)
        c.body_ids={'tool':0,c.capsule_data(path)['body']:1}
        c.cooked=lambda p:dict(body='tool',hulls=[(v,n,e)],kind='native_cooked_convex') if p==tool else c.capsule_data(p)
        self.assertTrue(c.eligible(tool,path));self.assertTrue(c.eligible(path,tool))
        poses=np.array([[0,0,0,0,0,0,1],[.0016,0,0,0,0,0,1.]])
        row=c.verify(tool,path,poses,32.6667,-.000571027)
        self.assertEqual(row['contact_scope'],'current_truss_tool_own_capsule')
        self.assertTrue(row['certified_within_limit']);self.assertAlmostEqual(row['penetration_upper_bound_m'],.0004)
        self.assertEqual(row['native_callback_separation_m'],-.000571027)
        poses[1,0]=.0014
        self.assertFalse(c.verify(tool,path,poses,32.6667,-.000571027)['certified_within_limit'])
        for name in ['/World/ElasticTarget/Truss_01_Pedicel_distal_11_0/Collider',
                     '/World/ElasticTarget/Stem_MainStem_9/Collider',
                     '/World/Greenhouse/House/Gutter/Collider',
                     '/World/HarvestRobot/Robot/arm_link/Collider']:
            self.assertFalse(c.eligible(tool,name));self.assertFalse(c.eligible(name,path))
        UsdPhysics.CollisionAPI(c.stage.GetPrimAtPath(tool)).CreateCollisionEnabledAttr(False)
        self.assertFalse(c.eligible(tool,path))
        UsdPhysics.CollisionAPI(c.stage.GetPrimAtPath(tool)).CreateCollisionEnabledAttr(True)
        UsdPhysics.CollisionAPI(capsule.GetPrim()).CreateCollisionEnabledAttr(False)
        self.assertFalse(c.eligible(tool,path))
        UsdPhysics.CollisionAPI(capsule.GetPrim()).CreateCollisionEnabledAttr(True)
        c.verify_fruit_contacts=False
        self.assertFalse(c.eligible(tool,path))


class CurrentTrussFruitLeafTests(unittest.TestCase):
    def test_exact_enabled_fruit_compliant_leaf_opt_in_shallow_deep_and_rejections(self):
        stage=Usd.Stage.CreateInMemory();root='/World/ElasticTarget'
        fruit=root+'/Truss_02_Fruit_01/Collider'
        other=root+'/Truss_01_Fruit_01/Collider'
        leaf=root+'/Leaf_V9_Additional_CompoundLeaf_07/Appendage_220_0'
        unknown=root+'/Leaf_V9_Additional_CompoundLeaf_07/Appendage_unknown'
        for path in [fruit,other,leaf,unknown]:
            UsdPhysics.CollisionAPI.Apply(UsdGeom.Mesh.Define(stage,path).GetPrim()).CreateCollisionEnabledAttr(True)
        manifest=dict(target=dict(name='Truss_02_Fruit_01',path=fruit.rsplit('/',1)[0]),
            plant=dict(root=root,fruits=[dict(name=path.rsplit('/',2)[1],path=path.rsplit('/',1)[0]) for path in [fruit,other]]),
            compliant_foliage=dict(leaves=[dict(meshes=[leaf])]))
        default=CookedLeafContactGeometry(stage,manifest,['fruit','leaf'])
        self.assertFalse(default.eligible(fruit,leaf))
        c=CookedLeafContactGeometry(stage,manifest,['fruit','leaf'],verify_fruit_contacts=True)
        self.assertTrue(c.eligible(fruit,leaf));self.assertTrue(c.eligible(leaf,fruit))
        self.assertTrue(c.eligible(other,leaf));self.assertFalse(c.eligible(fruit,unknown))
        self.assertFalse(c.eligible(fruit,'/World/Greenhouse/Plants/Plant_23/Leaf/Collider'))
        vertices=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        normals,edges=convex_axes(vertices)
        c.cooked=lambda path:dict(body='fruit' if path==fruit else 'leaf',
                                 hulls=[(vertices,normals,edges)],kind='native_cooked_convex')
        poses=np.array([[0,0,0,0,0,0,1],[.0016,0,0,0,0,0,1.]])
        row=c.verify(fruit,leaf,poses,39.058333,-.009493)
        self.assertEqual(row['contact_scope'],'active_plant_fruit_leaf')
        self.assertTrue(row['certified_within_limit']);self.assertAlmostEqual(row['penetration_upper_bound_m'],.0004)
        self.assertEqual(row['native_callback_separation_m'],-.009493)
        poses[1,0]=.0014
        self.assertFalse(c.verify(fruit,leaf,poses,39.058333,-.009493)['certified_within_limit'])
        for path in [fruit,leaf]:
            UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path)).CreateCollisionEnabledAttr(False)
            self.assertFalse(c.eligible(fruit,leaf))
            UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path)).CreateCollisionEnabledAttr(True)
        # Missing native geometry never authorizes a released fruit contact.
        c.cooked=lambda path:(_ for _ in ()).throw(ValueError('No native cooked convex hulls'))
        unknown_geometry=c.verify(fruit,leaf,poses,39.058333,-.009493)
        self.assertFalse(unknown_geometry['certified_within_limit']);self.assertIn('error',unknown_geometry)

    def test_active_plant_capsule_leaf_and_other_truss_fruit_capsule_do_not_expand_tool_scope(self):
        stage=Usd.Stage.CreateInMemory();root='/World/ElasticTarget'
        fruits=[root+'/Truss_02_Fruit_01/Collider',root+'/Truss_01_Fruit_01/Collider']
        leaf=root+'/Leaf_V9_Additional_CompoundLeaf_07/Appendage_220_1'
        stem=root+'/Stem_MainStem_9';own=root+'/Truss_02_Pedicel_distal_01_0'
        foreign='/World/Greenhouse/Plants/Plant_23/Stem_MainStem_9'
        foreign_leaf='/World/Greenhouse/Plants/Plant_23/Leaf/Appendage_1_0'
        tool='/World/ActuatedTool/R_belt/PrototypeTread'
        for path in fruits+[leaf,tool,foreign_leaf]:
            UsdPhysics.CollisionAPI.Apply(UsdGeom.Mesh.Define(stage,path).GetPrim()).CreateCollisionEnabledAttr(True)
        for path in [stem,own,foreign]:
            UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,path).GetPrim())
            capsule=UsdGeom.Capsule.Define(stage,path+'/Collider');capsule.CreateRadiusAttr(.001);capsule.CreateHeightAttr(.004);capsule.CreateAxisAttr('Z')
            UsdPhysics.CollisionAPI.Apply(capsule.GetPrim()).CreateCollisionEnabledAttr(True)
        manifest=dict(target=dict(name='Truss_02_Fruit_01',path=fruits[0].rsplit('/',1)[0]),
            plant=dict(root=root,fruits=[dict(name=path.rsplit('/',2)[1],path=path.rsplit('/',1)[0]) for path in fruits],
                       bodies=[dict(name=path.rsplit('/',1)[1],path=path) for path in [stem,own,foreign]]),
            compliant_foliage=dict(leaves=[dict(meshes=[leaf,foreign_leaf])]))
        c=CookedLeafContactGeometry(stage,manifest,['fruit','leaf',stem,own],verify_fruit_contacts=True)
        self.assertEqual(c.active_plant_capsules,{stem+'/Collider',own+'/Collider'})
        for a,b in [(stem+'/Collider',leaf),(fruits[1],stem+'/Collider'),(fruits[1],leaf)]:
            self.assertTrue(c.eligible(a,b));self.assertTrue(c.eligible(b,a))
        self.assertFalse(c.eligible(tool,stem+'/Collider'))
        self.assertFalse(c.eligible(tool,fruits[1]))
        self.assertFalse(c.eligible(fruits[0],foreign+'/Collider'))
        self.assertFalse(c.eligible(foreign+'/Collider',leaf))
        self.assertFalse(c.eligible(fruits[0],foreign_leaf))
        self.assertFalse(c.eligible(stem+'/Collider',foreign_leaf))
        self.assertTrue(c.eligible(tool,own+'/Collider'))
        vertices=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        normals,edges=convex_axes(vertices)
        c.cooked=lambda path:c.capsule_data(path) if path in c.active_plant_capsules else dict(
            body='leaf' if path==leaf else 'fruit',hulls=[(vertices,normals,edges)],kind='native_cooked_convex')
        poses=np.array([[0,0,0,0,0,0,1],[0,0,0,0,0,0,1],[.0016,0,0,0,0,0,1],[.05,0,0,0,0,0,1.]])
        for a,b,scope in [(stem+'/Collider',leaf,'active_plant_capsule_leaf'),
                          (fruits[1],stem+'/Collider','active_plant_fruit_capsule')]:
            shallow=c.verify(a,b,poses,39.075,-.001759)
            self.assertTrue(shallow['certified_within_limit']);self.assertAlmostEqual(shallow['penetration_upper_bound_m'],.0004)
            self.assertEqual(shallow['contact_scope'],scope)
            poses[2,0]=.0014
            self.assertFalse(c.verify(a,b,poses,39.075,-.001759)['certified_within_limit'])
            poses[2,0]=.0016
        c.verify_fruit_contacts=False
        self.assertFalse(c.eligible(stem+'/Collider',leaf));self.assertFalse(c.eligible(fruits[1],leaf))


class StructuralLeafGeometryTests(unittest.TestCase):
    def fixture(self):
        stage=Usd.Stage.CreateInMemory();root='/World/ElasticTarget'
        leaf=root+'/Leaf_02/Appendage_200_2';fruit=root+'/Truss_02_Fruit_01'
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,leaf.rsplit('/',1)[0]).GetPrim())
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Mesh.Define(stage,leaf).GetPrim()).CreateCollisionEnabledAttr(True)
        prefix='/World/Greenhouse/House/Environment'
        wire=prefix+'/GutterSuspensionWire_005/Mesh'
        mesh=UsdGeom.Mesh.Define(stage,wire)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
        capsule_path=prefix+'/StructuralCapsule/Collider'
        capsule=UsdGeom.Capsule.Define(stage,capsule_path)
        capsule.CreateRadiusAttr(.001);capsule.CreateHeightAttr(.004);capsule.CreateAxisAttr('Z')
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim()).CreateCollisionEnabledAttr(True)
        translation=capsule.AddTranslateOp();translation.Set((.0016,0.,0.))
        manifest=dict(target=dict(name='Truss_02_Fruit_01',path=fruit),
            plant=dict(root=root,fruits=[dict(name='Truss_02_Fruit_01',path=fruit)]),
            compliant_foliage=dict(leaves=[dict(meshes=[leaf])]))
        c=CookedLeafContactGeometry(stage,manifest,[leaf.rsplit('/',1)[0]],verify_fruit_contacts=True)
        return c,leaf,wire,capsule_path,capsule,translation,manifest

    def test_structural_triangle_is_inconclusive_and_inventory_not_permission(self):
        c,leaf,wire,cap,_,_,manifest=self.fixture()
        self.assertTrue(c.eligible(leaf,wire));self.assertTrue(c.eligible(wire,leaf))
        self.assertTrue(c.eligible(leaf,cap))
        self.assertFalse(CookedLeafContactGeometry(c.stage,manifest,[]).eligible(leaf,wire))
        for path in ['/World/Greenhouse/Plants/Plant_23/GutterSuspensionWire_005/Mesh',
                     '/World/HarvestRobot/Robot/Environment/Mesh',
                     '/World/ActuatedTool/Environment/Mesh']:
            if not path.startswith('/World/ActuatedTool/'):
                self.assertFalse(c.eligible(leaf,path))
        vertices=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        normals,edges=convex_axes(vertices);original=c.cooked
        c.cooked=lambda path:dict(body=leaf.rsplit('/',1)[0],hulls=[(vertices,normals,edges)]) if path==leaf else original(path)
        result=c.verify(leaf,wire,np.array([[0,0,0,0,0,0,1.]]),74.075,-.001276)
        self.assertFalse(result['certified_within_limit']);self.assertEqual(result['contact_scope'],'active_plant_leaf_structural')
        self.assertIn('native triangle representation unavailable',result['error'])
        self.assertEqual(result['native_callback_separation_m'],-.001276)
        UsdPhysics.CollisionAPI(c.stage.GetPrimAtPath(wire)).CreateCollisionEnabledAttr(False)
        self.assertFalse(c.eligible(leaf,wire))

    def test_static_structural_capsule_shallow_deep_and_source_change(self):
        c,leaf,_,path,cap,translation,_=self.fixture()
        vertices=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]])
        normals,edges=convex_axes(vertices);original=c.cooked
        c.cooked=lambda p:dict(body=leaf.rsplit('/',1)[0],hulls=[(vertices,normals,edges)]) if p==leaf else original(p)
        poses=np.array([[0,0,0,0,0,0,1.]])
        shallow=c.verify(leaf,path,poses,1.,-.001276)
        self.assertTrue(shallow['certified_within_limit']);self.assertAlmostEqual(shallow['penetration_upper_bound_m'],.0004)
        translation.Set((.0014,0.,0.));c.cache.clear()
        self.assertFalse(c.verify(leaf,path,poses,2.,-.001276)['certified_within_limit'])
        translation.Set((.0016,0.,0.))
        changed=c.verify(leaf,path,poses,3.,-.001276)
        self.assertFalse(changed['certified_within_limit']);self.assertIn('transform changed',changed['error'])
        c.cache.clear();cap.AddScaleOp().Set((2.,1.,1.))
        self.assertFalse(c.verify(leaf,path,poses,4.,-.001276)['certified_within_limit'])
        cap.ClearXformOpOrder();cap.AddTranslateOp().Set((.0016,0,0),1.)
        self.assertFalse(c.verify(leaf,path,poses,5.,-.001276)['certified_within_limit'])


if __name__=='__main__':unittest.main()
