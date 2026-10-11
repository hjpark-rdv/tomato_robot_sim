import numpy as np
import unittest
import tempfile
import json
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics
from motion_assets import mesh, rigid, collision
from compliant_foliage import convert, enabled_colliders, world_vertices, revise, armature_values, leaf_groups,canonical_leaf_group
from motion_assets import sha
from motion_stage_metrics import require_motion_units


def fixture():
    s=Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(s,'/World/ElasticTarget')
    owner='/World/ElasticTarget/Stem_MainStem_9'
    pose=np.eye(4);pose[:3,3]=[.4,.5,.6]
    rigid(s,owner,pose,.03,[.01]*3)
    UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath(owner)).CreateFilteredPairsRel().AddTarget('/World/NeighborStem')
    c=UsdGeom.Capsule.Define(s,owner+'/Collider');c.CreateHeightAttr(.1);c.CreateRadiusAttr(.006);collision(c.GetPrim())
    v=np.array([[.006,0,0],[.16,-.04,0],[.16,.04,0],[.1,0,.005]])
    f=np.array([[0,1,2],[0,2,3],[0,3,1],[1,3,2]])
    m=mesh(s,owner+'/Visual_132',v,f,(0,1,0))
    m.GetPrim().SetCustomDataByKey('sourceName','CompoundLeaf_02')
    UsdPhysics.CollisionAPI(m).CreateCollisionEnabledAttr(False)
    m=mesh(s,owner+'/Appendage_132_0',v,f,(0,1,0));collision(m.GetPrim())
    UsdPhysics.FilteredPairsAPI.Apply(m.GetPrim()).CreateFilteredPairsRel().AddTarget('/World/OtherLeaf')
    return s,owner


class CompliantFoliageTests(unittest.TestCase):
 def test_actual_hashed_glb_material_submeshes_form_one_compliant_leaf(self):
    from pxr import Sdf
    s,owner=fixture();names=['CompoundLeaf_01_214888','CompoundLeaf_01_7ed887','CompoundLeaf_01_9cc7c7','CompoundLeaf_01_ebc72d']
    for index,name in enumerate(names):
        number=132+index
        if index:
            for kind in ('Visual','Appendage'):
                tail='' if kind=='Visual' else '_0'
                Sdf.CopySpec(s.GetRootLayer(),f'{owner}/{kind}_132{tail}',s.GetRootLayer(),f'{owner}/{kind}_{number}{tail}')
        s.GetPrimAtPath(f'{owner}/Visual_{number}').SetCustomDataByKey('sourceName',name)
    old_colliders=enabled_colliders(s);old_world={p:world_vertices(s,p) for p in old_colliders if '/Appendage_' in p}
    self.assertEqual(list(leaf_groups(s)),['CompoundLeaf_01'])
    self.assertEqual(len(leaf_groups(s)['CompoundLeaf_01']),4)
    report=convert(s);self.assertEqual(len(report['leaves']),1)
    row=report['leaves'][0];self.assertEqual(row['source_names'],sorted(names))
    self.assertEqual(report['added_rotational_dofs'],3)
    self.assertEqual(len(enabled_colliders(s)),len(old_colliders))
    for path,world in old_world.items():np.testing.assert_array_equal(world_vertices(s,report['path_mapping'][path]),world)
    self.assertEqual(report['source_name_normalization']['mapping'],dict.fromkeys(names,'CompoundLeaf_01'))
    for old,name in zip(range(132,136),names):
        self.assertEqual(s.GetPrimAtPath(row['path']+f'/Visual_{old}').GetCustomDataByKey('sourceName'),name)

 def test_source_suffix_normalization_is_narrow_and_does_not_alias_other_groups(self):
    for name,expected in [('CompoundLeaf_01_214888','CompoundLeaf_01'),
                          ('CompoundLeaf_02_A1F3E6','CompoundLeaf_02'),
                          ('CompoundLeaf_01_7ed887_Midrib','CompoundLeaf_01'),
                          ('V9_Additional_CompoundLeaf_07','V9_Additional_CompoundLeaf_07'),
                          ('Apical_Young_Leaf_3_Vein_4_-1','Apical_Young_Leaf_3'),
                          ('CompoundLeaf_01_214888_other',None),
                          ('CompoundLeaf_01_xyzxyz',None),('CompoundLeaf_01_21488',None),
                          ('Pruned_Branch_01_214888',None),('Fruit_01_214888',None)]:
        self.assertEqual(canonical_leaf_group(name),expected)

 def test_former_same_body_leaf_pair_is_still_excluded(self):
    from pxr import Sdf
    s,owner=fixture()
    for kind in ('Visual','Appendage'):
        tail='' if kind=='Visual' else '_0'
        old=f'{owner}/{kind}_132{tail}';new=f'{owner}/{kind}_133{tail}'
        Sdf.CopySpec(s.GetRootLayer(),old,s.GetRootLayer(),new)
    s.GetPrimAtPath(owner+'/Visual_133').SetCustomDataByKey('sourceName','V9_Additional_CompoundLeaf_02')
    report=convert(s);a,b=report['leaves']
    for one,other in ((a,b),(b,a)):
        targets=list(map(str,UsdPhysics.FilteredPairsAPI(s.GetPrimAtPath(one['path'])).GetFilteredPairsRel().GetTargets()))
        self.assertIn(other['path'],targets)

 def test_added_v9_and_apical_leaf_names_are_not_left_rigid(self):
    for name,expected in [('V9_Additional_CompoundLeaf_02','V9_Additional_CompoundLeaf_02'),
                          ('Apical_Young_Leaf_3_Vein_4_-1','Apical_Young_Leaf_3'),
                          ('Apical_Young_Leaf_3_Midrib','Apical_Young_Leaf_3')]:
        s,owner=fixture();s.GetPrimAtPath(owner+'/Visual_132').SetCustomDataByKey('sourceName',name)
        self.assertEqual(list(leaf_groups(s)),[expected])
        r=convert(s);self.assertEqual(r['leaves'][0]['source_name'],expected)
        self.assertIn(owner+'/Appendage_132_0',r['path_mapping'])

 def test_leaf_becomes_independent_without_geometry_or_collision_loss(self):
    s,owner=fixture();old=owner+'/Appendage_132_0';before=world_vertices(s,old)
    main_mass=UsdPhysics.MassAPI(s.GetPrimAtPath(owner)).GetMassAttr().Get()
    report=convert(s);row=report['leaves'][0];new=report['path_mapping'][old]
    np.testing.assert_allclose(world_vertices(s,new),before,atol=1e-12)
    assert len(enabled_colliders(s))==2
    assert not s.GetPrimAtPath(old).IsActive()
    assert s.GetPrimAtPath(row['path']).HasAPI(UsdPhysics.RigidBodyAPI)
    assert UsdPhysics.MassAPI(s.GetPrimAtPath(owner)).GetMassAttr().Get()==main_mass
    j=UsdPhysics.Joint(s.GetPrimAtPath(row['joint']))
    assert list(map(str,j.GetBody0Rel().GetTargets()))==[owner]
    assert not j.GetExcludeFromArticulationAttr().Get()
    self.assertAlmostEqual(UsdPhysics.DriveAPI(j.GetPrim(),'rotX').GetStiffnessAttr().Get(), .005*np.pi/180, places=10)
    assert '/World/OtherLeaf' in list(map(str,UsdPhysics.FilteredPairsAPI(s.GetPrimAtPath(new)).GetFilteredPairsRel().GetTargets()))
    assert report['added_rotational_dofs']==3
    assert '/World/NeighborStem' in row['inherited_source_body_exclusions']


 def test_invalid_selection_and_stiffness_fail(self):
    for kwargs in ({'names':['unknown']},{'stiffness':float('nan')},{'damping':0}):
        s,_=fixture()
        with self.assertRaises(ValueError):convert(s,**kwargs)

 def test_leaf_armature_cannot_change_existing_plant_dofs(self):
    names=['Stem_MainStem_9:0','Leaf_CompoundLeaf_02:0','Truss_02_Rachis_0:1']
    np.testing.assert_array_equal(armature_values({},names),[.0005]*3)
    manifest=dict(compliant_foliage=dict(numerical_armature_kg_m2=.000005,
        leaves=[dict(joint='/World/ElasticTarget/Joints/Leaf_CompoundLeaf_02')]))
    np.testing.assert_array_equal(armature_values(manifest,names),[.0005,.000005,.0005])

 def test_saved_assembly_has_explicit_units_and_no_dynamic_sublayers(self):
    s,owner=fixture()
    UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z')
    UsdPhysics.SetStageKilogramsPerUnit(s,1.)
    with tempfile.TemporaryDirectory() as temp:
        root=Path(temp);a=root/'source';a.mkdir();out=root/'result'
        s.GetRootLayer().Export(str(a/'scene.usda'));before=sha(a/'scene.usda')
        (a/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=before,plant=dict(bodies=[]))))
        np.savez(a/'skin.npz',dummy=np.array([1]))
        r=revise(a,out);saved=Usd.Stage.Open(str(out/'scene.usda'))
        require_motion_units(saved)
        self.assertEqual(list(saved.GetRootLayer().subLayerPaths),[])
        self.assertEqual(sha(a/'scene.usda'),before)
        self.assertTrue(r['materialized'])
        self.assertEqual(enabled_colliders(saved),enabled_colliders(s)-{owner+'/Appendage_132_0'}|{'/World/ElasticTarget/Leaf_CompoundLeaf_02/Appendage_132_0'})

if __name__ == "__main__": unittest.main()
