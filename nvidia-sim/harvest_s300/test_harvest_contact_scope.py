import unittest
import copy
import hashlib
import tempfile
from pathlib import Path

import numpy as np

from harvest_contact_scope import HarvestContactScope, ground_colliders_from_stage, collection_colliders_from_stage


ROOT = '/World/ElasticTarget'
FRUIT = ROOT + '/Truss_02_Fruit_09'
JOINT = ROOT + '/HarvestJoints/Truss_02_Fruit_09'
FLOOR = '/World/Greenhouse/House/Environment/GroundCover_1_2/Mesh'


def manifest():
    return dict(plant=dict(root=ROOT, fruits=[dict(name='Truss_02_Fruit_09', path=FRUIT,
                                                joint=JOINT)]))


def contact(a=FRUIT + '/Collider', b=FLOOR, t=2.):
    return dict(a=a, b=b, t=t, phase='belt_finish', separation_m=-.0013,
                force_n=.7, impulse_ns=[0., 0., .01])


class HarvestContactScopeTests(unittest.TestCase):
    def route_fixture(self,root):
        from pxr import Usd,UsdPhysics
        from native_route_checkpoint import save_checkpoint
        source=root/'source';source.mkdir();scene=source/'scene.usda';scene.write_text('#usda 1.0\n')
        source_sha=hashlib.sha256(scene.read_bytes()).hexdigest();original=manifest()
        original['scene_sha256']=source_sha
        original['plant'].update(source_plant='/World/Greenhouse/Plants/Plant_22',bodies=[dict(path=ROOT+'/Stem')])
        events=[dict(joint=JOINT,t=1.,phase='belt_close')]
        checkpoint=root/'checkpoint'
        metadata=save_checkpoint(checkpoint,assembly=source,scene_sha256=source_sha,time_s=2.,
            body_paths=[FRUIT,ROOT+'/Stem'],body_poses_world=[[0,0,1,0,0,0,1],[0,0,0,0,0,0,1]],
            body_velocities_world=np.zeros((2,6)),robot_dof_names=['jaw_L','jaw_R'],robot_q=[0,0],robot_qvel=[0,0],
            plant_dof_names=['stem:0'],plant_q=[0],plant_qvel=[0],preload=[0],breaks=events,manifest=original,
            sequence=dict(checkpoint_capture_reason='completed_cycle',checkpoint_physics_valid=True,
                          cycle_plans=[dict(status='motion_completed',end_s=2.,stop_reason=None)]))
        frozen='/World/RouteFrozen/Plant_22/Truss_02_Fruit_09'
        frozen_joint='/World/RouteFrozen/Plant_22/HarvestJoints/Truss_02_Fruit_09'
        row=dict(source_plant=original['plant']['source_plant'],frozen_root='/World/RouteFrozen/Plant_22',
                 checkpoint=str(checkpoint),source_scene_sha256=source_sha,
                 checkpoint_metadata_sha256=metadata['metadata_sha256'],checkpoint_time_s=2.)
        row['detached_fruits']=[dict(original['plant']['fruits'][0],path=frozen,joint=frozen_joint,
            original_joint=JOINT,source_plant=row['source_plant'],route_identity='Plant_22/Truss_02_Fruit_09',
            native_break_events=events,source_scene_sha256=source_sha,checkpoint_time_s=2.,
            checkpoint_metadata_sha256=metadata['metadata_sha256'])]
        current=manifest();current['route_checkpoint']=dict(outgoing_history=[row])
        stage=Usd.Stage.CreateInMemory();UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim(frozen,'Xform'))
        UsdPhysics.CollisionAPI.Apply(stage.DefinePrim(frozen+'/Collider','Mesh')).CreateCollisionEnabledAttr(True)
        UsdPhysics.FixedJoint.Define(stage,frozen_joint).GetPrim().SetActive(False)
        return current,stage,frozen,source

    def test_verified_frozen_route_release_retains_original_evidence_without_new_callbacks(self):
        with tempfile.TemporaryDirectory() as temp:
            current,stage,frozen,_=self.route_fixture(Path(temp))
            scope=HarvestContactScope(current,[FLOOR],enabled=True,stage=stage)
            event=contact(a=frozen+'/Collider',t=.1)
            self.assertEqual(scope.filter_invalid([event],[]),[])
            proof=scope.records[0]['prior_route_release_proof']
            self.assertFalse(proof['native_new_event'])
            self.assertEqual(proof['original_break_events'][0]['t'],1.)
            self.assertEqual(scope.records[0]['native_break_t'],0.)
            self.assertFalse(scope.records[0]['collection_verified'])
            for other in ['/World/ActuatedTool/R/part_244',ROOT+'/Stem/Collider']:
                event=contact(a=frozen+'/Collider',b=other)
                self.assertEqual(scope.filter_invalid([event],[]),[event])
            attached=contact(a='/World/RouteFrozen/Plant_22/Truss_02_Fruit_01/Collider')
            self.assertEqual(scope.filter_invalid([attached],[]),[attached])

    def test_prior_history_identity_event_and_stage_or_source_hash_mismatch_fail_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            current,stage,frozen,source=self.route_fixture(Path(temp))
            for changes in [dict(path=FRUIT),dict(original_joint=JOINT+'_wrong'),
                            dict(native_break_events=[]),dict(checkpoint_metadata_sha256='e'*64)]:
                bad=copy.deepcopy(current);bad['route_checkpoint']['outgoing_history'][0]['detached_fruits'][0].update(changes)
                with self.assertRaises(ValueError):HarvestContactScope(bad,[FLOOR],enabled=True,stage=stage)
            disabled=stage.GetPrimAtPath('/World/RouteFrozen/Plant_22/HarvestJoints/Truss_02_Fruit_09')
            disabled.SetActive(True)
            with self.assertRaises(ValueError):HarvestContactScope(current,[FLOOR],enabled=True,stage=stage)
            disabled.SetActive(False);(source/'scene.usda').write_text('# changed source\n')
            with self.assertRaises(ValueError):HarvestContactScope(current,[FLOOR],enabled=True,stage=stage)

    def test_default_strict_ignores_prior_history_and_enabled_mode_requires_verified_stage(self):
        current=manifest();current['route_checkpoint']=dict(outgoing_history=[dict(checkpoint='/missing')])
        scope=HarvestContactScope(current,[FLOOR])
        self.assertEqual(scope.prior_releases,{})
        event=contact(a='/World/RouteFrozen/Plant_22/Truss_02_Fruit_09/Collider')
        self.assertEqual(scope.filter_invalid([event],[]),[event])
        with self.assertRaisesRegex(ValueError,'exact current USD stage'):
            HarvestContactScope(current,[FLOOR],enabled=True)

    def test_explicitly_disabled_declared_joint_survives_flatten_usdc_but_missing_or_enabled_fails(self):
        from pxr import Usd,UsdPhysics
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);current,stage,frozen,_=self.route_fixture(root)
            joint='/World/RouteFrozen/Plant_22/HarvestJoints/Truss_02_Fruit_09'
            declaration=stage.GetPrimAtPath(joint);declaration.SetActive(True)
            UsdPhysics.Joint(declaration).CreateJointEnabledAttr(False)
            binary=root/'frozen.usdc';stage.Flatten(addSourceFileComment=False).Export(str(binary))
            loaded=Usd.Stage.Open(str(binary))
            self.assertTrue(loaded.GetPrimAtPath(joint).IsActive())
            scope=HarvestContactScope(current,[FLOOR],enabled=True,stage=loaded)
            event=contact(a=frozen+'/Collider',t=.1)
            self.assertEqual(scope.filter_invalid([event],[]),[])
            proof=scope.records[0]['prior_route_release_proof']
            self.assertEqual(proof['disabled_joint_representation'],'existing_active_joint_explicitly_disabled')
            self.assertEqual(proof['original_break_events'][0]['t'],1.)
            attr=UsdPhysics.Joint(loaded.GetPrimAtPath(joint)).GetJointEnabledAttr()
            attr.Set(True)
            with self.assertRaises(ValueError):HarvestContactScope(current,[FLOOR],enabled=True,stage=loaded)
            attr.Set(False);attr.Set(False,1.)
            with self.assertRaises(ValueError):HarvestContactScope(current,[FLOOR],enabled=True,stage=loaded)
            loaded.RemovePrim(joint)
            with self.assertRaises(ValueError):HarvestContactScope(current,[FLOOR],enabled=True,stage=loaded)

    def test_default_strict_even_after_native_break(self):
        scope = HarvestContactScope(manifest(), [FLOOR])
        event = contact()
        self.assertEqual(scope.filter_invalid([event], [dict(joint=JOINT, t=1.)]), [event])
        self.assertFalse(scope.records)

    def test_exact_native_release_precedes_floor_contact(self):
        scope = HarvestContactScope(manifest(), [FLOOR], enabled=True)
        event = contact()
        self.assertEqual(scope.filter_invalid([event], [dict(joint=JOINT, t=1.)]), [])
        row = scope.records[0]
        self.assertEqual(row['raw_penetration_m'], .0013)
        self.assertEqual(row['native_break_t'], 1.)
        self.assertFalse(row['collection_verified'])
        self.assertFalse(scope.summarize()['strict_0_5mm_passed'])
        self.assertEqual(scope.summarize()['released_fruit_ground_impacts'][0]['max_point_force_n'], .7)

    def test_attached_future_unknown_or_bad_timestamp_stay_strict(self):
        for breaks in [[], [dict(joint=JOINT, t=3.)],
                       [dict(joint=JOINT + '_other', t=1.)],
                       [dict(joint=JOINT, t=float('nan'))]]:
            scope = HarvestContactScope(manifest(), [FLOOR], enabled=True)
            event = contact()
            self.assertEqual(scope.filter_invalid([event], breaks), [event])
        event = contact(t=float('nan'))
        self.assertEqual(scope.filter_invalid([event], [dict(joint=JOINT, t=1.)]), [event])

    def test_robot_active_plant_gutter_and_unknown_floor_remain_strict(self):
        scope = HarvestContactScope(manifest(), [FLOOR], enabled=True)
        for other in ['/World/ActuatedTool/R/part_244',
                      ROOT + '/Truss_02_Pedicel_proximal_09_1/Collider',
                      '/World/Greenhouse/House/Environment/Gutter_1/Mesh',
                      '/World/Greenhouse/House/Environment/GroundCover_1_3/Mesh']:
            event = contact(b=other)
            self.assertEqual(scope.filter_invalid([event], [dict(joint=JOINT, t=1.)]), [event])
        event = contact(a=FRUIT + '/UnknownCollider')
        self.assertEqual(scope.filter_invalid([event], [dict(joint=JOINT, t=1.)]), [event])

    def test_order_symmetry_and_mixed_invalid_events(self):
        scope = HarvestContactScope(manifest(), [FLOOR], enabled=True)
        floor = contact(a=FLOOR, b=FRUIT + '/Collider')
        active = contact(b=ROOT + '/Stem_MainStem_0/Collider')
        self.assertEqual(scope.filter_invalid([floor, active], [dict(joint=JOINT, t=1.)]), [active])

    def test_reject_identity_or_floor_scope_expansion(self):
        with self.assertRaises(ValueError):
            HarvestContactScope(manifest(), ['/World/Greenhouse/House/Environment/Gutter_1/Mesh'])
        bad = manifest()
        bad['plant']['fruits'][0]['joint'] += '_mismatch'
        with self.assertRaises(ValueError):
            HarvestContactScope(bad, [FLOOR])

    def test_stage_inventory_requires_static_enabled_exact_floor(self):
        from pxr import Usd, UsdPhysics
        stage = Usd.Stage.CreateInMemory()
        for path in [FLOOR, '/World/Greenhouse/House/Environment/GroundCover_Pin_1/Mesh',
                     '/World/Greenhouse/House/Environment/GroundCover_1_3/Mesh',
                     '/World/Greenhouse/House/Environment/GroundCover_1_4/Mesh']:
            prim = stage.DefinePrim(path, 'Mesh')
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.CollisionAPI(stage.GetPrimAtPath(
            '/World/Greenhouse/House/Environment/GroundCover_1_3/Mesh')).CreateCollisionEnabledAttr(False)
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(
            '/World/Greenhouse/House/Environment/GroundCover_1_4'))
        self.assertEqual(ground_colliders_from_stage(stage), (FLOOR,))

    def test_existing_foundation_continuation_requires_native_release_and_opt_in(self):
        from pxr import Usd,UsdPhysics
        floor='/World/Greenhouse/House/Environment/Foundation_6x10m/Mesh'
        stage=Usd.Stage.CreateInMemory()
        prim=stage.DefinePrim(floor,'Mesh')
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        self.assertEqual(ground_colliders_from_stage(stage),(floor,))
        scope=HarvestContactScope(manifest(),[floor],enabled=True,stage=stage)
        event=contact(b=floor)
        self.assertEqual(scope.filter_invalid([event],[]),[event])
        self.assertEqual(scope.filter_invalid([event],[dict(joint=JOINT,t=3.)]),[event])
        self.assertEqual(scope.filter_invalid([event],[dict(joint=JOINT,t=1.)]),[])
        self.assertEqual(scope.records[-1]['scope'],'released_fruit_ground')
        self.assertFalse(scope.records[-1]['released_fruit_physics_verified'])
        arm=contact(a='/World/ActuatedTool/R_belt/PrototypeTread',b=floor)
        self.assertEqual(scope.filter_invalid([arm],[dict(joint=JOINT,t=1.)]),[arm])
        strict=HarvestContactScope(manifest(),[floor],stage=stage)
        self.assertEqual(strict.filter_invalid([event],[dict(joint=JOINT,t=1.)]),[event])

    def test_foundation_must_be_exact_enabled_static_stage_collider(self):
        from pxr import Usd,UsdPhysics
        floor='/World/Greenhouse/House/Environment/Foundation_6x10m/Mesh'
        with self.assertRaises(ValueError):HarvestContactScope(manifest(),[floor],enabled=True)
        stage=Usd.Stage.CreateInMemory()
        prim=stage.DefinePrim(floor,'Mesh')
        api=UsdPhysics.CollisionAPI.Apply(prim);api.CreateCollisionEnabledAttr(False)
        self.assertEqual(ground_colliders_from_stage(stage),())
        with self.assertRaises(ValueError):HarvestContactScope(manifest(),[floor],enabled=True,stage=stage)
        api.CreateCollisionEnabledAttr(True)
        UsdPhysics.RigidBodyAPI.Apply(prim.GetParent())
        self.assertEqual(ground_colliders_from_stage(stage),())
        with self.assertRaises(ValueError):HarvestContactScope(manifest(),[floor],enabled=True,stage=stage)

    def test_collection_deck_inventory_is_exact_and_passive(self):
        from pxr import Usd, UsdPhysics
        stage=Usd.Stage.CreateInMemory()
        old='/World/HarvestRobot/S300/FlatRearDeck'
        rail='/World/HarvestRobot/Robot/world/S300Chassis/FlatRearDeck'
        for path in [old,rail,'/World/HarvestRobot/Robot/link4/FlatRearDeck',
                     '/World/HarvestRobot/Other/FlatRearDeck','/World/HarvestRobot/S300/FlatRearDeckCover']:
            UsdPhysics.CollisionAPI.Apply(stage.DefinePrim(path,'Cube')).CreateCollisionEnabledAttr(True)
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath('/World/HarvestRobot/Robot/world'))
        self.assertEqual(collection_colliders_from_stage(stage),tuple(sorted((old,rail))))
        # Adding a distinct movable owner to the deck removes rail-carrier eligibility.
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath('/World/HarvestRobot/Robot/world/S300Chassis'))
        self.assertEqual(collection_colliders_from_stage(stage),(old,))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath('/World/HarvestRobot/S300'))
        self.assertEqual(collection_colliders_from_stage(stage),())

    def test_released_fruit_passive_deck_is_diagnostic_not_verified_collection(self):
        deck='/World/HarvestRobot/S300/FlatRearDeck';event=contact(b=deck)
        breaks=[dict(joint=JOINT,t=1.)]
        self.assertEqual(HarvestContactScope(manifest(),[deck]).filter_invalid([event],breaks),[event])
        scope=HarvestContactScope(manifest(),[deck],enabled=True)
        self.assertEqual(scope.filter_invalid([event],breaks),[])
        self.assertEqual(scope.records[0]['scope'],'released_fruit_passive_collection_deck')
        self.assertFalse(scope.records[0]['collection_verified'])
        self.assertEqual(scope.summarize()['released_fruit_ground_impacts'],[])
        self.assertEqual(len(scope.summarize()['released_fruit_collection_impacts']),1)

    def test_separate_s300_carrier_deck_requires_exact_owner(self):
        from pxr import Usd,UsdPhysics
        stage=Usd.Stage.CreateInMemory()
        carrier='/World/HarvestRobot/Robot/S300Carrier'
        deck=carrier+'/S300Chassis/FlatRearDeck'
        for path in [deck,'/World/HarvestRobot/Robot/link4/S300Chassis/FlatRearDeck',
                     '/World/HarvestRobot/Robot/OtherCarrier/S300Chassis/FlatRearDeck']:
            UsdPhysics.CollisionAPI.Apply(stage.DefinePrim(path,'Cube')).CreateCollisionEnabledAttr(True)
        # An unowned static copy at the moving carrier path is not accepted.
        self.assertEqual(collection_colliders_from_stage(stage),())
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(carrier))
        self.assertEqual(collection_colliders_from_stage(stage),(deck,))
        scope=HarvestContactScope(manifest(),collection_colliders_from_stage(stage),enabled=True)
        self.assertEqual(scope.filter_invalid([contact(b=deck)],[dict(joint=JOINT,t=1.)]),[])
        self.assertFalse(scope.records[0]['collection_verified'])
        # A separately dynamic deck/chassis is outside the exact carrier scope.
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(carrier+'/S300Chassis'))
        self.assertEqual(collection_colliders_from_stage(stage),())

    def test_exact_sealed_cover_requires_stage_and_native_release_in_both_orders(self):
        from passive_collection_surfaces import PATH
        from test_passive_collection_surfaces import PassiveCollectionSurfaceTests
        fixture=PassiveCollectionSurfaceTests();fixture.setUp();stage=fixture.stage
        inventory=collection_colliders_from_stage(stage)
        self.assertEqual(inventory,(PATH,))
        with self.assertRaisesRegex(ValueError,'verified current USD stage'):
            HarvestContactScope(manifest(),inventory,enabled=True)
        for a,b in ((FRUIT+'/Collider',PATH),(PATH,FRUIT+'/Collider')):
            event=contact(a=a,b=b,t=79.225)
            scope=HarvestContactScope(manifest(),inventory,enabled=True,stage=stage)
            for events in ([],[dict(joint=JOINT,t=79.226)]):
                self.assertEqual(scope.filter_invalid([event],events),[event])
            self.assertEqual(scope.filter_invalid([event],[dict(joint=JOINT,t=76.82)]),[])
            row=scope.records[0]
            self.assertEqual(row['native_break_t'],76.82)
            self.assertEqual(row['collection_surface_proof']['source_vertices'],96)
            self.assertEqual(row['raw_penetration_m'],.0013)
            self.assertFalse(row['collection_verified'])
            self.assertFalse(row['released_fruit_physics_verified'])
            self.assertFalse(scope.summarize()['strict_0_5mm_passed'])
        strict=HarvestContactScope(manifest(),inventory,stage=stage)
        self.assertEqual(strict.filter_invalid([contact(b=PATH)],[dict(joint=JOINT,t=1.)]),[contact(b=PATH)])
        fixture.prim.SetCustomDataByKey('sourceName','arm mount')
        with self.assertRaises(ValueError):HarvestContactScope(manifest(),inventory,enabled=True,stage=stage)



class ReleasedGripperSupportTests(unittest.TestCase):
    def fixture(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        stage=Usd.Stage.CreateInMemory()
        for side,number,source in [('L',117,'C01_carriage_saddle'),('L',122,'C02_lower_deck'),
                                   ('R',239,'C01_carriage_saddle'),('R',244,'C02_lower_deck')]:
            body='/World/ActuatedTool/'+side
            UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,body).GetPrim())
            mesh=UsdGeom.Mesh.Define(stage,body+'/part_'+str(number))
            mesh.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0)])
            mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
            mesh.GetPrim().SetCustomDataByKey('sourceName',side+'_'+source)
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        return stage

    def test_native_release_and_separate_opt_in_required_both_contact_orders(self):
        stage=self.fixture();old=HarvestContactScope(manifest(),[FLOOR],enabled=True,stage=stage)
        scope=HarvestContactScope(manifest(),[FLOOR],stage=stage,gripper_support_enabled=True)
        for path in scope.gripper_supports:
            for reverse in (False,True):
                event=contact(b=path)
                if reverse:event['a'],event['b']=event['b'],event['a']
                self.assertEqual(old.filter_invalid([event],[dict(joint=JOINT,t=1.)]),[event])
                for breaks in ([],[dict(joint=JOINT,t=3.)],[dict(joint=JOINT+'_other',t=1.)]):
                    self.assertEqual(scope.filter_invalid([event],breaks),[event])
                self.assertEqual(scope.filter_invalid([event],[dict(joint=JOINT,t=1.)]),[])
        summary=scope.summarize();self.assertEqual(len(summary['released_fruit_gripper_support_impacts']),4)
        self.assertFalse(summary['collection_verified']);self.assertFalse(summary['released_fruit_physics_verified'])
        # Enabling supports alone cannot accidentally authorize even a known floor.
        self.assertEqual(scope.filter_invalid([contact()],[dict(joint=JOINT,t=1.)]),[contact()])

    def test_other_contacts_and_nonfinite_evidence_remain_strict(self):
        scope=HarvestContactScope(manifest(),stage=self.fixture(),gripper_support_enabled=True)
        breaks=[dict(joint=JOINT,t=1.)]
        for path in ['/World/ActuatedTool/L/part_127','/World/ActuatedTool/L_belt/PrototypeTread',
                     '/World/HarvestRobot/Robot/link4/Collider',ROOT+'/Stem/Collider',
                     ROOT+'/Truss_02_Fruit_10/Collider','/World/Greenhouse/House/Gutter/Mesh']:
            event=contact(b=path);self.assertEqual(scope.filter_invalid([event],breaks),[event])
        for key in ['t','separation_m']:
            event=contact(b='/World/ActuatedTool/L/part_122');event[key]=float('nan')
            self.assertFalse(scope.classify(event,breaks)['continuation_authorized'])
        event=contact(a='/World/ActuatedTool/L/part_122',b='/World/HarvestRobot/Robot/link4/Collider')
        self.assertEqual(scope.filter_invalid([event],breaks),[event])

    def test_wrong_source_owner_disabled_or_missing_mesh_rejected(self):
        from pxr import UsdPhysics
        from harvest_contact_scope import gripper_support_inventory
        with self.assertRaises(ValueError):gripper_support_inventory(None)
        path='/World/ActuatedTool/L/part_122'
        for mutation in ['name','owner','disabled','missing']:
            stage=self.fixture();prim=stage.GetPrimAtPath(path)
            if mutation=='name':prim.SetCustomDataByKey('sourceName','L_C02_upper_deck')
            elif mutation=='owner':UsdPhysics.RigidBodyAPI.Apply(prim)
            elif mutation=='disabled':UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
            else:stage.RemovePrim(path)
            with self.assertRaises(ValueError):gripper_support_inventory(stage)


class ReleasedSameTrussTests(unittest.TestCase):
    def fixture(self):
        from pxr import Usd,UsdPhysics
        stage=Usd.Stage.CreateInMemory();m=manifest();m['plant']['bodies']=[]
        for name in ['Truss_02_Fruit_03','Truss_03_Fruit_03']:
            m['plant']['fruits'].append(dict(name=name,path=ROOT+'/'+name,joint=ROOT+'/HarvestJoints/'+name))
        for name in ['Truss_02_Pedicel_proximal_09_2','Truss_02_Rachis_0','Truss_02_Truss_01_Peduncle_0','Truss_03_Rachis_0','Stem_MainStem_0']:
            m['plant']['bodies'].append(dict(name=name,path=ROOT+'/'+name))
        for row in m['plant']['fruits']+m['plant']['bodies']:
            UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim(row['path'],'Xform'))
            prim=stage.DefinePrim(row['path']+'/Collider','Mesh' if '_Fruit_'in row['name']else 'Capsule')
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        return stage,m

    def test_own_branch_and_fruit_require_native_release_both_callback_orders(self):
        stage,m=self.fixture();scope=HarvestContactScope(m,stage=stage,same_truss_enabled=True)
        joint=ROOT+'/HarvestJoints/Truss_02_Fruit_03';fruit=ROOT+'/Truss_02_Fruit_03/Collider'
        for other in [FRUIT+'/Collider',ROOT+'/Truss_02_Pedicel_proximal_09_2/Collider',
                      ROOT+'/Truss_02_Rachis_0/Collider',ROOT+'/Truss_02_Truss_01_Peduncle_0/Collider']:
            for reverse in (False,True):
                event=contact(a=other if reverse else fruit,b=fruit if reverse else other)
                for breaks in ([],[dict(joint=joint,t=3.)],[dict(joint=joint+'_bad',t=1.)]):
                    self.assertEqual(scope.filter_invalid([event],breaks),[event])
                self.assertEqual(scope.filter_invalid([event],[dict(joint=joint,t=1.)]),[])
                self.assertEqual(scope.records[-1]['fruit_collider'],fruit)
        self.assertEqual(len(scope.summarize()['released_fruit_same_truss_impacts']),4)
        self.assertEqual(scope.summarize()['released_fruit_collection_impacts'],[])
        self.assertFalse(scope.summarize()['released_fruit_physics_verified'])

    def test_default_strict_and_other_truss_stem_robot_unknown_remain_strict(self):
        stage,m=self.fixture();strict=HarvestContactScope(m,stage=stage);scope=HarvestContactScope(m,stage=stage,same_truss_enabled=True)
        breaks=[dict(joint=JOINT,t=1.)];own=contact(b=ROOT+'/Truss_02_Rachis_0/Collider')
        self.assertEqual(strict.filter_invalid([own],breaks),[own])
        for other in [ROOT+'/Truss_03_Rachis_0/Collider',ROOT+'/Truss_03_Fruit_03/Collider',ROOT+'/Stem_MainStem_0/Collider',
                      ROOT+'/Truss_02_Rachis_999/Collider','/World/ActuatedTool/L_belt/PrototypeTread',
                      '/World/Greenhouse/Plants/Plant_22/Truss_02_Rachis_0/Collider',FLOOR]:
            event=contact(b=other);self.assertEqual(scope.filter_invalid([event],breaks),[event])
        for key in ['t','separation_m']:
            event=own.copy();event[key]=float('nan');self.assertFalse(scope.classify(event,breaks)['continuation_authorized'])

    def test_source_mapping_collider_owner_or_missing_stage_fail(self):
        from pxr import UsdPhysics
        stage,m=self.fixture()
        with self.assertRaises(ValueError):HarvestContactScope(m,same_truss_enabled=True)
        for mutation in ['path','disabled','owner']:
            stage,m=self.fixture();path=ROOT+'/Truss_02_Rachis_0/Collider';prim=stage.GetPrimAtPath(path)
            if mutation=='path':m['plant']['bodies'][0]['path']=ROOT+'/Stem_MainStem_0'
            elif mutation=='disabled':UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
            else:UsdPhysics.RigidBodyAPI.Apply(prim)
            with self.assertRaises(ValueError):HarvestContactScope(m,stage=stage,same_truss_enabled=True)

    def test_frozen_release_loads_without_floor_permission_and_never_aliases_active_truss(self):
        from pxr import UsdPhysics
        with tempfile.TemporaryDirectory() as temp:
            m,stage,frozen,_=HarvestContactScopeTests().route_fixture(Path(temp))
            UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim(FRUIT,'Xform'))
            UsdPhysics.CollisionAPI.Apply(stage.DefinePrim(FRUIT+'/Collider','Mesh')).CreateCollisionEnabledAttr(True)
            scope=HarvestContactScope(m,stage=stage,same_truss_enabled=True)
            self.assertEqual(len(scope.prior_releases),1)
            event=contact(a=frozen+'/Collider',b=FRUIT+'/Collider')
            self.assertEqual(scope.filter_invalid([event],[]),[event])
            self.assertNotEqual(scope.same_truss_contacts[frozen+'/Collider']['truss_identity'],scope.same_truss_contacts[FRUIT+'/Collider']['truss_identity'])


class DeclaredPassiveLeafTests(unittest.TestCase):
    def fixture(self):
        from pxr import Usd,UsdGeom,UsdPhysics
        stage=Usd.Stage.CreateInMemory();body='/World/BackgroundCompliantFoliage/Plant_24/Leaf_CompoundLeaf_01';leaf=body+'/Appendage_mesh_4';joint='/World/BackgroundCompliantFoliage/Plant_24/Joints/Leaf_CompoundLeaf_01'
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,body).GetPrim())
        j=UsdPhysics.Joint.Define(stage,joint);j.CreateBody1Rel().SetTargets([body]);j.CreateExcludeFromArticulationAttr(True)
        mesh=UsdGeom.Mesh.Define(stage,leaf);mesh.GetPrim().SetCustomDataByKey('sourceName','CompoundLeaf_01_214888')
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True);UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexDecomposition')
        row=dict(path=body,joint=joint,source_stem='/World/Greenhouse/Plants/Plant_24/Stem',source_group='CompoundLeaf_01',meshes=[leaf],source_meshes=['/World/Greenhouse/Plants/Plant_24/Stem/mesh_4'])
        m=manifest();m['background_compliant_foliage']=dict(bodies=[copy.deepcopy(row)],leaves=[row],path_mapping={row['source_meshes'][0]:leaf})
        return stage,m,body,leaf

    def test_exact_mapping_registered_without_released_fruit_exemption(self):
        from harvest_contact_scope import background_leaf_colliders_from_manifest
        stage,m,body,leaf=self.fixture();inventory=background_leaf_colliders_from_manifest(stage,m,[body])
        self.assertEqual(set(inventory),{leaf});scope=HarvestContactScope(m,[FLOOR],enabled=True,stage=stage)
        event=contact(b=leaf);breaks=[dict(joint=JOINT,t=1.)]
        self.assertEqual(scope.filter_invalid([event],breaks),[event]);record=scope.classify(event,breaks)
        self.assertFalse(record['continuation_authorized']);self.assertFalse(record['passive_leaf_contact_exemption'])
        self.assertEqual(record['exact_declared_passive_leaf_colliders'],[leaf]);self.assertEqual(scope.metadata()['exact_declared_passive_leaf_inventory'],inventory)

    def test_missing_body_bad_source_or_unmapped_mesh_fail_closed(self):
        from harvest_contact_scope import background_leaf_colliders_from_manifest
        stage,m,body,leaf=self.fixture()
        with self.assertRaisesRegex(ValueError,'Missing measured'):background_leaf_colliders_from_manifest(stage,m,[])
        bad=copy.deepcopy(m);bad['background_compliant_foliage']['path_mapping']={}
        with self.assertRaises(ValueError):background_leaf_colliders_from_manifest(stage,bad,[body])
        stage.GetPrimAtPath(leaf).SetCustomDataByKey('sourceName','MainStem')
        with self.assertRaises(ValueError):background_leaf_colliders_from_manifest(stage,m,[body])

    def test_cooked_shallow_deep_same_limit_and_nonleaf_strict(self):
        from pxr import UsdGeom,UsdPhysics
        from leaf_contact_geometry import CookedLeafContactGeometry,convex_axes
        stage,m,body,leaf=self.fixture();tool='/World/ActuatedTool/L';part=tool+'/part_122'
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,tool).GetPrim());UsdPhysics.CollisionAPI.Apply(UsdGeom.Mesh.Define(stage,part).GetPrim()).CreateCollisionEnabledAttr(True)
        reader=CookedLeafContactGeometry(stage,m,[body,tool]);v=np.array([[x,y,z] for x in [-.001,.001] for y in [-.001,.001] for z in [-.001,.001]]);n,e=convex_axes(v)
        reader.cooked=lambda path:dict(body=body if path==leaf else tool,hulls=[(v,n,e)])
        poses=np.array([[0,0,0,0,0,0,1],[.0016,0,0,0,0,0,1.]])
        shallow=reader.verify(leaf,part,poses,1.,-.0015)
        self.assertTrue(shallow['certified_within_limit']);self.assertEqual(shallow['contact_scope'],'declared_passive_leaf_measured_geometry')
        self.assertEqual(shallow['limit_m'],.0005);self.assertEqual(shallow['native_callback_separation_m'],-.0015)
        poses[1,0]=.0014;self.assertFalse(reader.verify(leaf,part,poses,2.,-.0015)['certified_within_limit'])
        self.assertFalse(reader.eligible(leaf,ROOT+'/Stem/Collider'));self.assertFalse(reader.eligible(leaf,'/World/HarvestRobot/Robot/link4/Collider'))

if __name__ == '__main__':
    unittest.main()
