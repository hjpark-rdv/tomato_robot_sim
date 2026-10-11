import unittest
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from plant_interaction_variants import disable_other_plant_contacts, inherit_target_only_contacts, remap_values, target_only_keep, verified_disabled_colliders
from neighbor_plant_runtime import gravity_preload
from motion_contact_guard import FruitContactGuard


class PlantInteractionVariantsTest(unittest.TestCase):
    def test_disabled_visual_api_cleanup_remains_verifiably_noncolliding(self):
        from motion_visuals import prepare_display_meshes
        stage=Usd.Stage.CreateInMemory();path='/World/ElasticTarget/Skin/mesh_0'
        prim=UsdGeom.Mesh.Define(stage,path).GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(False)
        m=dict(plant=dict(root='/World/ElasticTarget'),target=dict(name='Truss_00_Fruit_01'))
        m['plant_interaction_variant']=disable_other_plant_contacts(stage,m,include_already_disabled=True)
        self.assertEqual(verified_disabled_colliders(stage,m),{path})
        report=prepare_display_meshes(stage)
        self.assertEqual(report['paths'],[path]);self.assertFalse(prim.HasAPI(UsdPhysics.CollisionAPI))
        self.assertEqual(verified_disabled_colliders(stage,m),{path})
        prim.GetAttribute('physics:collisionEnabled').Set(True)
        with self.assertRaises(ValueError):verified_disabled_colliders(stage,m)
        prim.GetAttribute('physics:collisionEnabled').Set(False)
        stage.RemovePrim(path)
        with self.assertRaises(ValueError):verified_disabled_colliders(stage,m)

    def test_native_runner_rejects_broader_selection_of_noncolliding_trusses(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace
        tree=ast.parse(Path(__file__).with_name('run_motion.py').read_text())
        guards=[node for node in ast.walk(tree) if isinstance(node,ast.If) and
                any(isinstance(stmt,ast.Raise) and isinstance(stmt.exc,ast.Call)
                    and any(isinstance(arg,ast.Constant) and isinstance(arg.value,str)
                            and arg.value.startswith('Target-only collision assets require')
                            for arg in stmt.exc.args) for stmt in node.body)]
        self.assertEqual(len(guards),1)
        guard=compile(ast.fix_missing_locations(ast.Module(body=guards,type_ignores=[])),'native_target_scope','exec')
        for mode,scope,rejected in [('target_truss_and_own_stem_only','inventory.json',True),
                ('target_truss_and_own_stem_only',None,False),('current_baseline','inventory.json',False)]:
            context=dict(manifest=dict(plant_interaction_variant=dict(mode=mode)),a=SimpleNamespace(near_trusses_metadata=scope))
            if rejected:
                with self.assertRaisesRegex(ValueError,'one exact target truss'):exec(guard,context)
            else:exec(guard,context)

    def test_window_rebind_preserves_old_omissions_and_disables_newly_frozen_fruit(self):
        stage=Usd.Stage.CreateInMemory()
        disabled='/World/Greenhouse/Plants/Plant_43/Truss_00/Collider'
        frozen='/World/RouteFrozen/Plant_29/Truss_00_Fruit_03/Collider'
        target='/World/ElasticTarget/Truss_01_Fruit_01/Collider'
        other='/World/ElasticTarget/Truss_00_Fruit_01/Collider'
        stem='/World/ElasticTarget/Stem_MainStem_0/Collider'
        house='/World/Greenhouse/House/Gutter/Collider'
        for path in [disabled,frozen,target,other,stem,house]:
            UsdPhysics.CollisionAPI.Apply(UsdGeom.Sphere.Define(stage,path).GetPrim()).CreateCollisionEnabledAttr(path!=disabled)
        m=dict(plant=dict(root='/World/ElasticTarget'),target=dict(name='Truss_01_Fruit_01'))
        parent=dict(mode='target_truss_and_own_stem_only',target_root='/World/ElasticTarget',
                    target_truss='Truss_00',disabled_count=1,disabled_colliders=[disabled])
        policy=inherit_target_only_contacts(stage,m,parent)
        self.assertEqual(verified_disabled_colliders(stage,m),{disabled,frozen,other})
        self.assertEqual(policy['newly_disabled_count'],2)
        self.assertEqual(policy['enabled_after'],3)
        self.assertEqual(parent['disabled_colliders'],[disabled])
        for path in [target,stem,house]:
            self.assertTrue(UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path)).GetCollisionEnabledAttr().Get())
        # Refreshing a frozen-history declaration must not drop already disabled
        # fruit or incorrectly subtract them from the active collider count.
        again=inherit_target_only_contacts(stage,m,policy)
        self.assertEqual(verified_disabled_colliders(stage,m),{disabled,frozen,other})
        self.assertEqual(again['newly_disabled_count'],0)
        self.assertEqual(again['enabled_before'],again['enabled_after'])

    def test_window_inheritance_does_not_enable_target_only_mode_by_default(self):
        for parent in [None,dict(mode='current_baseline')]:
            m={}
            self.assertIsNone(inherit_target_only_contacts(Usd.Stage.CreateInMemory(),m,parent))
            self.assertEqual(m,{})

    def test_exact_scope_preserves_target_stem_robot_gutter(self):
        stage=Usd.Stage.CreateInMemory()
        paths=['/World/ElasticTarget/Stem_MainStem_3/Collider',
               '/World/ElasticTarget/Truss_00_Fruit_01/Collider',
               '/World/ElasticTarget/Truss_00_Pedicel_01/Collider',
               '/World/ElasticTarget/Truss_01_Fruit_01/Collider',
               '/World/ElasticTarget/Truss_000_Rachis/Collider',
               '/World/Greenhouse/Plants/Plant_01/Stem/Collider',
               '/World/RouteFrozen/Plant_22/Truss_02_Fruit_01/Collider',
               '/World/Greenhouse/House/Gutter/Collider',
               '/World/HarvestRobot/Arm/Collider','/World/ActuatedTool/L/Collider']
        for path in paths:
            UsdPhysics.CollisionAPI.Apply(UsdGeom.Sphere.Define(stage,path).GetPrim()).CreateCollisionEnabledAttr(True)
        before={p:str(UsdGeom.Sphere(stage.GetPrimAtPath(p)).GetRadiusAttr().Get()) for p in paths}
        report=disable_other_plant_contacts(stage,dict(plant=dict(root='/World/ElasticTarget'),
            target=dict(name='Truss_00_Fruit_01')))
        self.assertEqual(set(report['disabled_colliders']),set(paths[3:7]))
        self.assertTrue(report['planner_and_native_same_saved_collision_states'])
        for i,p in enumerate(paths):
            self.assertEqual(UsdPhysics.CollisionAPI(stage.GetPrimAtPath(p)).GetCollisionEnabledAttr().Get(),i not in range(3,7))
            self.assertEqual(before[p],str(UsdGeom.Sphere(stage.GetPrimAtPath(p)).GetRadiusAttr().Get()))

    def test_namespace_remap_does_not_change_similarly_named_root(self):
        result=remap_values({'paths':['/World/ElasticTarget/Joints/a','/World/ElasticTargetExtra/a']},
                           {'/World/ElasticTarget':'/World/ElasticNeighbors/Plant_28'})
        self.assertEqual(result['paths'],['/World/ElasticNeighbors/Plant_28/Joints/a','/World/ElasticTargetExtra/a'])

    def test_declared_omission_accepts_only_exact_disabled_non_target_plant(self):
        stage=Usd.Stage.CreateInMemory();path='/World/RouteFrozen/Plant_22/Truss_00_Fruit_01/Collider'
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Sphere.Define(stage,path).GetPrim()).CreateCollisionEnabledAttr(False)
        m=dict(plant=dict(root='/World/ElasticTarget'),target=dict(name='Truss_00_Fruit_01'),
               plant_interaction_variant=dict(mode='target_truss_and_own_stem_only',target_root='/World/ElasticTarget',
               target_truss='Truss_00',disabled_count=1,disabled_colliders=[path]))
        self.assertEqual(verified_disabled_colliders(stage,m),{path})
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path)).CreateCollisionEnabledAttr(True)
        with self.assertRaises(ValueError):verified_disabled_colliders(stage,m)
        path='/World/HarvestRobot/Stem_01/Collider'
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Sphere.Define(stage,path).GetPrim()).CreateCollisionEnabledAttr(False)
        m['plant_interaction_variant']['disabled_colliders']=[path]
        with self.assertRaises(ValueError):verified_disabled_colliders(stage,m)

    def test_preload_accounts_for_branch_and_external_fruit_torque(self):
        jac=np.zeros((1,6,2));jac[0,2,0]=1;jac[0,4,1]=1
        plant=dict(bodies=[dict(path='/anchor',rest_position=[0,0,0])],
                   fruits=[dict(anchor='/anchor',center=[1,0,0],mass=2)])
        np.testing.assert_allclose(gravity_preload(jac,[0,3],['root','anchor'],plant),[49.05,-19.62])

    def test_neighbor_contacts_are_guarded_without_target_fruit_authorization(self):
        from motion_contact_buffer import ACTIVE_PATH_MARKERS
        self.assertIn('/ElasticNeighbors/',ACTIVE_PATH_MARKERS)
        manifest=dict(target=dict(name='Truss_00_Fruit_01',path='/World/ElasticTarget/Truss_00_Fruit_01'),
            plant=dict(root='/World/ElasticTarget',fruits=[dict(name='Truss_00_Fruit_01',path='/World/ElasticTarget/Truss_00_Fruit_01')]))
        guard=FruitContactGuard(manifest,['/World/ActuatedTool/L/Collider'],fruit_limit_m=.001,
            fruit_fruit_limit_m=.001,default_limit_m=.001)
        self.assertTrue(guard.invalid(dict(a='/World/ElasticNeighbors/Plant_28/Truss_00/Collider',
            b='/World/Greenhouse/House/Gutter/Collider',separation_m=-.0011)))
        self.assertIsNone(target_only_keep('/World/HarvestRobot/Stem_01','/World/ElasticTarget','Truss_00'))


if __name__=='__main__':unittest.main()
