import unittest
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from plant_interaction_variants import (ACTIVE_PLANT_MODE, active_plant_only_keep,
    disable_other_plant_contacts, inherit_target_only_contacts, remap_values,
    target_only_keep, verified_disabled_colliders)
from neighbor_plant_runtime import gravity_preload
from motion_contact_guard import FruitContactGuard


class ReleasedFruitLifecyclePreservationTests(unittest.TestCase):
    def fixture(self,root,legacy_disabled=False):
        from test_harvest_contact_scope import HarvestContactScopeTests,ROOT,FRUIT,JOINT,FLOOR
        m,stage,frozen,source=HarvestContactScopeTests().route_fixture(root)
        m['plant']['source_plant']='/World/Greenhouse/Plants/Plant_19'
        m['target']=dict(name='Truss_02_Fruit_09')
        def mesh(path):
            p=UsdGeom.Mesh.Define(stage,path);p.CreatePointsAttr([(0,0,0),(.01,0,0),(0,.01,0)])
            p.CreateFaceVertexCountsAttr([3]);p.CreateFaceVertexIndicesAttr([0,1,2])
            UsdPhysics.CollisionAPI.Apply(p.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,FRUIT).GetPrim());mesh(FRUIT+'/Collider')
        mesh(ROOT+'/Truss_01_Fruit_01/Collider');mesh(ROOT+'/Stem_MainStem_1/Collider')
        mesh('/World/RouteFrozen/Plant_22/Truss_02_Fruit_08/Collider')
        mesh('/World/Greenhouse/Plants/Plant_44/Truss_00_Fruit_01/Collider');mesh(FLOOR)
        mass=UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(frozen));mass.CreateMassAttr(.007)
        rigid=UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(frozen));rigid.CreateVelocityAttr((.1,.2,.3));rigid.CreateAngularVelocityAttr((4.,5.,6.))
        collider=frozen+'/Collider';m['plant_interaction_variant']=dict(mode='target_truss_and_own_stem_only',
            target_root=ROOT,target_truss='Truss_02',disabled_count=1,
            disabled_colliders=[collider]if legacy_disabled else [ROOT+'/Truss_01_Fruit_01/Collider'])
        stage.GetPrimAtPath(m['plant_interaction_variant']['disabled_colliders'][0]).GetAttribute('physics:collisionEnabled').Set(False)
        return m,stage,frozen,source

    def test_only_native_proven_frozen_fruit_stays_physical_all_other_ghosts_and_properties_preserved(self):
        import tempfile,hashlib
        from pathlib import Path
        from strict_window_storage import canonical_layer_digest
        for legacy in (False,True):
            with self.subTest(legacy_disabled=legacy),tempfile.TemporaryDirectory()as temp:
                m,stage,frozen,source=self.fixture(Path(temp),legacy)
                before=canonical_layer_digest(stage.GetRootLayer());source_bytes=(source/'scene.usda').read_bytes()
                p=stage.GetPrimAtPath(frozen);before_mass=p.GetAttribute('physics:mass').Get()
                velocity=p.GetAttribute('physics:velocity').Get();angular=p.GetAttribute('physics:angularVelocity').Get()
                # This old selector caused the bug: it treats every frozen fruit as a plant ghost.
                self.assertFalse(target_only_keep(frozen+'/Collider',m['plant']['root'],'Truss_02'))
                policy=disable_other_plant_contacts(stage,m,include_already_disabled=True);m['plant_interaction_variant']=policy
                self.assertTrue(stage.GetPrimAtPath(frozen+'/Collider').GetAttribute('physics:collisionEnabled').Get())
                self.assertNotIn(frozen+'/Collider',verified_disabled_colliders(stage,m))
                self.assertEqual(policy['newly_enabled_verified_released_count'],int(legacy))
                for path in ['/World/ElasticTarget/Truss_01_Fruit_01/Collider',
                             '/World/RouteFrozen/Plant_22/Truss_02_Fruit_08/Collider',
                             '/World/Greenhouse/Plants/Plant_44/Truss_00_Fruit_01/Collider']:
                    self.assertFalse(stage.GetPrimAtPath(path).GetAttribute('physics:collisionEnabled').Get())
                self.assertTrue(stage.GetPrimAtPath('/World/Greenhouse/House/Environment/GroundCover_1_2/Mesh').GetAttribute('physics:collisionEnabled').Get())
                self.assertEqual(p.GetAttribute('physics:mass').Get(),before_mass)
                self.assertEqual(p.GetAttribute('physics:velocity').Get(),velocity)
                self.assertEqual(p.GetAttribute('physics:angularVelocity').Get(),angular)
                self.assertEqual(policy['exact_property_preservation']['source_canonical'],before)
                self.assertEqual(policy['exact_property_preservation']['normalized_derived_canonical'],before)
                self.assertEqual((source/'scene.usda').read_bytes(),source_bytes)

    def test_historical_release_is_already_released_at_new_stage_zero_but_new_alias_stays_attached(self):
        import tempfile
        from pathlib import Path
        from harvest_contact_scope import HarvestContactScope
        from test_harvest_contact_scope import contact,FRUIT,FLOOR
        with tempfile.TemporaryDirectory()as temp:
            m,stage,frozen,_=self.fixture(Path(temp));m['plant_interaction_variant']=disable_other_plant_contacts(stage,m,include_already_disabled=True)
            scope=HarvestContactScope(m,[FLOOR],enabled=True,stage=stage)
            event=contact(a=frozen+'/Collider',t=0.)
            self.assertEqual(scope.filter_invalid([event],[]),[])
            proof=scope.records[-1];self.assertEqual(proof['native_break_t'],0.)
            self.assertEqual(proof['prior_route_release_proof']['original_break_events'][0]['t'],1.)
            self.assertFalse(proof['prior_route_release_proof']['native_new_event'])
            self.assertEqual(scope.records[-1]['raw_separation_m'],event['separation_m'])
            self.assertEqual(scope.filter_invalid([contact(a=FRUIT+'/Collider',t=0.)],[]),[contact(a=FRUIT+'/Collider',t=0.)])

    def test_new_outgoing_history_is_verified_before_final_manifest_history_assignment(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory()as temp:
            m,stage,frozen,_=self.fixture(Path(temp));history=m['route_checkpoint'].pop('outgoing_history')
            parent=m['plant_interaction_variant'].copy()
            with self.assertRaisesRegex(ValueError,'authoritative native release history'):
                disable_other_plant_contacts(stage,m,include_already_disabled=True)
            policy=inherit_target_only_contacts(stage,m,parent,release_history=history)
            self.assertTrue(stage.GetPrimAtPath(frozen+'/Collider').GetAttribute('physics:collisionEnabled').Get())
            m['route_checkpoint']['outgoing_history']=history
            self.assertNotIn(frozen+'/Collider',verified_disabled_colliders(stage,m))

    def test_missing_or_forged_release_inventory_and_source_fail_before_any_collider_edit(self):
        import tempfile,copy
        from pathlib import Path
        cases=['missing_fruit','all_history_missing','wrong_event','wrong_cp_sha','wrong_path','wrong_source']
        for case in cases:
            with self.subTest(case=case),tempfile.TemporaryDirectory()as temp:
                m,stage,frozen,source=self.fixture(Path(temp));r=m['route_checkpoint']['outgoing_history'][0]
                if case=='missing_fruit':r['detached_fruits']=[]
                elif case=='all_history_missing':m['route_checkpoint']['outgoing_history']=[]
                elif case=='wrong_event':r['detached_fruits'][0]['native_break_events']=[]
                elif case=='wrong_cp_sha':r['checkpoint_metadata_sha256']='c'*64
                elif case=='wrong_path':r['detached_fruits'][0]['path']='/World/ElasticTarget/Truss_02_Fruit_09'
                elif case=='wrong_source':(source/'scene.usda').write_text('# wrong source\n')
                before=stage.GetRootLayer().ExportToString()
                with self.assertRaises(ValueError):disable_other_plant_contacts(stage,m,include_already_disabled=True)
                self.assertEqual(stage.GetRootLayer().ExportToString(),before)

    def test_disabled_joint_must_keep_exact_original_anchor_and_fruit_body_relationships(self):
        import tempfile
        from pathlib import Path
        for which in ('body0','body1','enabled'):
            with self.subTest(which=which),tempfile.TemporaryDirectory()as temp:
                m,stage,frozen,_=self.fixture(Path(temp));row=m['route_checkpoint']['outgoing_history'][0]['detached_fruits'][0]
                prim=stage.GetPrimAtPath(row['joint']);j=UsdPhysics.Joint(prim)
                if which=='body0':j.CreateBody0Rel().SetTargets([frozen])
                elif which=='body1':j.CreateBody1Rel().SetTargets(['/World/ElasticTarget/Truss_02_Fruit_09'])
                else:prim.SetActive(True);j.CreateJointEnabledAttr(True)
                with self.assertRaises(ValueError):disable_other_plant_contacts(stage,m,include_already_disabled=True)

    def test_saved_protected_declaration_is_revalidated_against_actual_collision_flag(self):
        import tempfile,copy
        from pathlib import Path
        with tempfile.TemporaryDirectory()as temp:
            m,stage,frozen,_=self.fixture(Path(temp));m['plant_interaction_variant']=disable_other_plant_contacts(stage,m,include_already_disabled=True)
            stage.GetPrimAtPath(frozen+'/Collider').GetAttribute('physics:collisionEnabled').Set(False)
            with self.assertRaises(ValueError):verified_disabled_colliders(stage,m)


class PlantInteractionVariantsTest(unittest.TestCase):
    def test_target_scope_retains_existing_ghosts_and_proves_only_collision_flags_changed(self):
        from strict_window_storage import canonical_layer_digest
        stage,paths,m=self.active_scope_fixture()
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(paths[5])).GetCollisionEnabledAttr().Set(False)
        before=canonical_layer_digest(stage.GetRootLayer())
        policy=disable_other_plant_contacts(stage,m,include_already_disabled=True)
        m['plant_interaction_variant']=policy
        self.assertEqual(verified_disabled_colliders(stage,m),set(paths[2:6]))
        self.assertEqual(policy['disabled_count'],4)
        self.assertEqual(policy['newly_disabled_count'],3)
        proof=policy['exact_property_preservation']
        self.assertEqual(proof['source_canonical'],before)
        self.assertEqual(proof['normalized_derived_canonical'],before)
        self.assertEqual(set(proof['exact_changed_spec_paths']),set(paths[2:5]))
        for path in paths[:2]+paths[6:]:
            self.assertTrue(UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path)).GetCollisionEnabledAttr().Get())

    def test_revise_target_only_carries_forward_prior_disabled_inventory(self):
        import hashlib,json,shutil,tempfile
        from pathlib import Path
        from unittest.mock import patch
        from plant_interaction_variants import revise
        stage,paths,m=self.active_scope_fixture()
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(paths[5])).GetCollisionEnabledAttr().Set(False)
        with tempfile.TemporaryDirectory() as temporary:
            source=Path(temporary)/'source';output=Path(temporary)/'derived';source.mkdir()
            stage.GetRootLayer().Export(str(source/'composed.usdc'))
            (source/'scene.usda').write_text('#usda 1.0\n( subLayers = [@composed.usdc@] )\n')
            (source/'skin.npz').write_bytes(b'unchanged storage fixture')
            m['compact_assets']={'materialized_dependency':{'relative_path':'composed.usdc'}}
            (source/'motion_manifest.json').write_text(json.dumps(m))
            def copy_source(src,dst):
                dst.mkdir()
                for path in src.iterdir():shutil.copy2(path,dst/path.name)
            with patch('plant_interaction_variants.compact_assembly',side_effect=copy_source),patch('plant_interaction_variants.verify_compact_dependencies'):
                report=revise(source,output,target_only=True)
            self.assertIn(paths[5],report['disabled_colliders'])
            loaded=Usd.Stage.Open(str(output/'composed.usdc'))
            derived=json.loads((output/'motion_manifest.json').read_text())
            self.assertEqual(verified_disabled_colliders(loaded,derived),set(paths[2:6]))
            self.assertEqual((source/'skin.npz').read_bytes(),(output/'skin.npz').read_bytes())
            self.assertTrue(report['exact_property_preservation']['passed'])

    def active_scope_fixture(self):
        stage=Usd.Stage.CreateInMemory()
        paths=['/World/ElasticTarget/Stem_MainStem_3/Collider',
               '/World/ElasticTarget/Truss_00_Fruit_01/Collider',
               '/World/ElasticTarget/Truss_01_Pedicel_01/Collider',
               '/World/ElasticTarget/Truss_02_Fruit_01/Collider',
               '/World/Greenhouse/Plants/Plant_18/Truss_00/Collider',
               '/World/RouteFrozen/Plant_22/Truss_02_Fruit_01/Collider',
               '/World/Greenhouse/House/Gutter/Collider',
               '/World/HarvestRobot/Arm/Collider','/World/ActuatedTool/L/Collider']
        for path in paths:
            prim=UsdGeom.Sphere.Define(stage,path).GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        manifest=dict(plant=dict(root='/World/ElasticTarget'),target=dict(name='Truss_00_Fruit_01'))
        return stage,paths,manifest

    def test_active_plant_mode_preserves_all_own_trusses_and_every_other_property(self):
        from strict_window_storage import canonical_layer_digest
        stage,paths,manifest=self.active_scope_fixture()
        before=canonical_layer_digest(stage.GetRootLayer())
        policy=disable_other_plant_contacts(stage,manifest,mode=ACTIVE_PLANT_MODE,include_already_disabled=True)
        manifest['plant_interaction_variant']=policy
        self.assertEqual(verified_disabled_colliders(stage,manifest),set(paths[4:6]))
        for i,path in enumerate(paths):
            self.assertEqual(stage.GetPrimAtPath(path).GetAttribute('physics:collisionEnabled').Get(),i not in (4,5))
        proof=policy['exact_property_preservation']
        self.assertEqual(proof['source_canonical'],before)
        self.assertEqual(proof['normalized_derived_canonical'],before)
        self.assertEqual(proof['exact_changed_spec_paths'],paths[4:6])
        self.assertTrue(proof['visuals_dynamics_materials_and_all_other_authored_fields_exact'])
        self.assertIn('unverified',policy['interpretation'])

    def test_active_plant_scope_rejects_missing_foreign_declarations_and_active_alias(self):
        stage,paths,manifest=self.active_scope_fixture()
        manifest['plant_interaction_variant']=disable_other_plant_contacts(stage,manifest,mode=ACTIVE_PLANT_MODE,
                                                                         include_already_disabled=True)
        policy=manifest['plant_interaction_variant'];omitted=policy['disabled_colliders'].pop()
        policy['disabled_count']-=1
        with self.assertRaisesRegex(ValueError,'Undeclared'):verified_disabled_colliders(stage,manifest)
        policy['disabled_colliders'].append(omitted);policy['disabled_count']+=1
        stage.GetPrimAtPath(omitted).GetAttribute('physics:collisionEnabled').Set(True)
        with self.assertRaises(ValueError):verified_disabled_colliders(stage,manifest)
        stage.GetPrimAtPath(omitted).GetAttribute('physics:collisionEnabled').Set(False)
        policy['disabled_colliders'][0]=paths[1]
        with self.assertRaisesRegex(ValueError,'Invalid declared'):verified_disabled_colliders(stage,manifest)
        self.assertIsNone(active_plant_only_keep('/World/ElasticTargetExtra/Collider','/World/ElasticTarget'))
        self.assertIsNone(active_plant_only_keep('/World/HarvestRobot/Stem_01','/World/ElasticTarget'))

    def test_active_plant_mode_preserves_un_authored_fallback_flag(self):
        stage,paths,manifest=self.active_scope_fixture()
        prim=stage.GetPrimAtPath(paths[4]);prim.RemoveProperty('physics:collisionEnabled')
        self.assertTrue(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())
        policy=disable_other_plant_contacts(stage,manifest,mode=ACTIVE_PLANT_MODE,include_already_disabled=True)
        self.assertTrue(policy['exact_property_preservation']['passed'])
        self.assertFalse(prim.GetAttribute('physics:collisionEnabled').Get())

    def test_active_plant_scope_inherits_to_new_window_without_losing_own_trusses(self):
        stage,paths,manifest=self.active_scope_fixture()
        parent=disable_other_plant_contacts(stage,manifest,mode=ACTIVE_PLANT_MODE,include_already_disabled=True)
        manifest['target']['name']='Truss_02_Fruit_01'
        policy=inherit_target_only_contacts(stage,manifest,parent)
        self.assertEqual(policy['mode'],ACTIVE_PLANT_MODE)
        self.assertEqual(policy['target_truss'],'Truss_02')
        self.assertEqual(policy['newly_disabled_count'],0)
        self.assertEqual(verified_disabled_colliders(stage,manifest),set(paths[4:6]))
        for path in paths[:4]:self.assertTrue(stage.GetPrimAtPath(path).GetAttribute('physics:collisionEnabled').Get())

    def test_unknown_scope_and_mutually_exclusive_variants_fail_before_writes(self):
        from plant_interaction_variants import revise
        stage,paths,manifest=self.active_scope_fixture()
        with self.assertRaisesRegex(ValueError,'Unknown explicit'):disable_other_plant_contacts(stage,manifest,mode='all_off')
        with self.assertRaisesRegex(ValueError,'Select one'):revise('not_read','not_written',target_only=True,active_plant_only=True)

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
            context=dict(manifest=dict(plant_interaction_variant=dict(mode=mode)),a=SimpleNamespace(
                near_trusses_metadata=scope,camera_target_truss_window=False))
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
