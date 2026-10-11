import unittest
from pxr import Sdf, Usd, UsdGeom, UsdPhysics
from leaf_contact_policy import apply_policy, collision_inventory


class LeafPolicyTest(unittest.TestCase):
    def test_exact_leaves_only_with_shared_instance_sources(self):
        s=Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(s,'/Source')
        for name, source in [('leaf','CompoundLeaf_01_214888'), ('cut','Pruned_Branch_01'),
                             ('fruit','Fruit_01'), ('calyx','Calyx_01'), ('stem','MainStem_01')]:
            p=UsdGeom.Mesh.Define(s,'/Source/'+name).GetPrim()
            p.SetCustomDataByKey('sourceName',source)
            UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
        # The hidden source itself is not a plant and should not be altered by
        # an alias silently: real compact assets use non-composed prototype roots.
        s.GetPrimAtPath('/Source').SetSpecifier(Sdf.SpecifierClass)
        for n in (0,1):
            p=UsdGeom.Xform.Define(s,f'/World/Greenhouse/Plants/Plant_{n}/Stem').GetPrim()
            p.GetReferences().AddInternalReference('/Source');p.SetInstanceable(True)
        body='/World/ElasticTarget/Leaf_CompoundLeaf_04'
        p=UsdGeom.Mesh.Define(s,body+'/Appendage_140_0').GetPrim()
        UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
        manifest={'compliant_foliage':{'leaves':[dict(path=body,source_name='CompoundLeaf_04',meshes=[str(p.GetPath())])]}}
        report=apply_policy(s,manifest)
        self.assertEqual(report['disabled_count'],3)
        for path,p in collision_inventory(s).items():
            expected=not (path.endswith('/leaf') or path.endswith('/Appendage_140_0'))
            self.assertEqual(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get(),expected)

    def test_reject_wrong_leaf_owner(self):
        s=Usd.Stage.CreateInMemory()
        with self.assertRaises(ValueError):
            apply_policy(s,{'compliant_foliage':{'leaves':[dict(path='/World/ElasticTarget/Leaf_CompoundLeaf_01',source_name='CompoundLeaf_01',meshes=['/World/HarvestRobot/Collider'])]}})

    def test_existing_environment_planner_ignores_leaf_but_keeps_gutter(self):
        from test_rail_environment_preflight import fixture, cube, PreparationEnvironmentTests
        screen = PreparationEnvironmentTests().screen
        s, m, p = fixture()
        leaf = cube(s, '/World/Greenhouse/Plants/Plant_01/Stem/leaf', (0,.015,0), source='CompoundLeaf_01')
        report, _ = screen(s,m)
        self.assertFalse(report['passed'])
        apply_policy(s,m)
        report, adapter = screen(s,m)
        self.assertTrue(report['passed'])
        self.assertNotIn(str(leaf.GetPath()), [row['path'] for row in adapter.rows])
        cube(s, '/World/Greenhouse/House/Gutter', (0,.015,0))
        report, _ = screen(s,m)
        self.assertFalse(report['passed'])
        self.assertEqual(report['hard_hits'][0]['obstacle_class'],'gutter')


if __name__=='__main__':unittest.main()
