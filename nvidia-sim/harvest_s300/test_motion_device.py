"""Device intent must control the composed scene, not only tensor readback."""
import unittest
from unittest.mock import patch
from pxr import Usd, UsdPhysics, PhysxSchema
from motion_app import configure_scene_device, configure_gpu_pair_capacities, configure_contact_readback


class SceneDeviceTests(unittest.TestCase):
    def test_cpu_corrects_an_authored_gpu_scene(self):
        stage=Usd.Stage.CreateInMemory()
        prim=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene').GetPrim()
        api=PhysxSchema.PhysxSceneAPI.Apply(prim)
        api.CreateEnableGPUDynamicsAttr(True)
        api.CreateBroadphaseTypeAttr('GPU')
        actual=configure_scene_device(stage,'cpu')
        self.assertEqual(actual['enabled_gpu_dynamics'],False)
        self.assertEqual(actual['broadphase_type'],'MBP')
        # Changing the explicit request back must also configure native dynamics.
        actual=configure_scene_device(stage,'cuda:0')
        self.assertTrue(actual['enabled_gpu_dynamics'])
        self.assertEqual(actual['broadphase_type'],'GPU')

    def test_rejects_missing_scene_or_unknown_device(self):
        stage=Usd.Stage.CreateInMemory()
        with self.assertRaisesRegex(ValueError,'Missing'):
            configure_scene_device(stage,'cpu')
        with self.assertRaisesRegex(ValueError,'Unsupported'):
            configure_scene_device(stage,'cuda:1')

    def test_explicit_broadphase_does_not_change_requested_dynamics(self):
        for device in ('cpu','cuda:0'):
            for broadphase in ('MBP','SAP','GPU'):
                with self.subTest(device=device,broadphase=broadphase):
                    stage=Usd.Stage.CreateInMemory()
                    UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
                    actual=configure_scene_device(stage,device,broadphase=broadphase)
                    self.assertEqual(actual['enabled_gpu_dynamics'],device=='cuda:0')
                    self.assertEqual(actual['broadphase_type'],broadphase)
                    self.assertEqual(actual['requested_broadphase'],broadphase)
                    self.assertEqual(actual['dynamics_backend'],'GPU' if device=='cuda:0' else 'CPU')
                    self.assertEqual(actual['broadphase_backend'],'GPU' if broadphase=='GPU' else 'CPU')

    def test_rejects_wrong_broadphase_without_mutating_scene(self):
        for broadphase in ('mbp','ABP','',None,4):
            with self.subTest(broadphase=broadphase):
                stage=Usd.Stage.CreateInMemory()
                prim=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene').GetPrim()
                before=stage.GetRootLayer().ExportToString()
                with self.assertRaisesRegex(ValueError,'broadphase'):
                    configure_scene_device(stage,'cpu',broadphase=broadphase)
                self.assertEqual(stage.GetRootLayer().ExportToString(),before)
                self.assertFalse(prim.HasAPI(PhysxSchema.PhysxSceneAPI))

    def test_rejects_a_non_scene_prim(self):
        stage=Usd.Stage.CreateInMemory()
        stage.DefinePrim('/World/PhysicsScene','Xform')
        with self.assertRaisesRegex(ValueError,'Missing native physics scene'):
            configure_scene_device(stage,'cpu',broadphase='SAP')

    def test_configuration_preserves_other_scene_properties(self):
        stage=Usd.Stage.CreateInMemory()
        prim=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene').GetPrim()
        scene=UsdPhysics.Scene(prim)
        scene.CreateGravityMagnitudeAttr(9.81)
        api=PhysxSchema.PhysxSceneAPI.Apply(prim)
        api.CreateSolverTypeAttr('PGS')
        api.CreateTimeStepsPerSecondAttr(120)
        before={str(attr.GetName()):attr.Get() for attr in prim.GetAttributes()
                if str(attr.GetName()) not in ('physxScene:broadphaseType','physxScene:enableGPUDynamics')}
        configure_scene_device(stage,'cpu',broadphase='GPU')
        after={str(attr.GetName()):attr.Get() for attr in prim.GetAttributes()
               if str(attr.GetName()) not in ('physxScene:broadphaseType','physxScene:enableGPUDynamics')}
        self.assertEqual(before,after)

    def test_composed_value_mismatch_is_reported(self):
        stage=Usd.Stage.CreateInMemory()
        UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
        with patch('pxr.PhysxSchema.PhysxSceneAPI') as schema:
            api=schema.Apply.return_value
            api.GetEnableGPUDynamicsAttr.return_value.Get.return_value=False
            api.GetBroadphaseTypeAttr.return_value.Get.return_value='MBP'
            with self.assertRaisesRegex(RuntimeError,'mismatch'):
                configure_scene_device(stage,'cpu',broadphase='GPU')


class GpuPairCapacityTests(unittest.TestCase):
    CAPACITY_ATTRS = ('physxScene:gpuFoundLostAggregatePairsCapacity',
                      'physxScene:gpuTotalAggregatePairsCapacity')

    def test_sets_actual_unsigned_values_including_upper_bound(self):
        for capacity in (1, 16384, 131072, 2**31-1):
            with self.subTest(capacity=capacity):
                stage=Usd.Stage.CreateInMemory()
                prim=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene').GetPrim()
                actual=configure_gpu_pair_capacities(stage,capacity)
                self.assertEqual(actual,dict(scene='/World/PhysicsScene',requested_capacity=capacity,
                    found_lost_aggregate_pairs_capacity=capacity,total_aggregate_pairs_capacity=capacity))
                for name in self.CAPACITY_ATTRS:
                    self.assertEqual(prim.GetAttribute(name).Get(),capacity)
                    self.assertEqual(str(prim.GetAttribute(name).GetTypeName()),'uint')

    def test_rejects_invalid_capacity_without_mutating_scene(self):
        for capacity in (True,False,0,-1,2**31,1.0,float('nan'),float('inf'),'131072',None):
            with self.subTest(capacity=capacity):
                stage=Usd.Stage.CreateInMemory()
                prim=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene').GetPrim()
                before=stage.GetRootLayer().ExportToString()
                with self.assertRaisesRegex(ValueError,'positive integer'):
                    configure_gpu_pair_capacities(stage,capacity)
                self.assertEqual(stage.GetRootLayer().ExportToString(),before)
                self.assertFalse(prim.HasAPI(PhysxSchema.PhysxSceneAPI))

    def test_rejects_missing_or_non_scene_prim_without_mutation(self):
        for type_name in (None,'Xform'):
            with self.subTest(type_name=type_name):
                stage=Usd.Stage.CreateInMemory()
                if type_name:
                    stage.DefinePrim('/World/PhysicsScene',type_name)
                before=stage.GetRootLayer().ExportToString()
                with self.assertRaisesRegex(ValueError,'Missing native physics scene'):
                    configure_gpu_pair_capacities(stage,131072)
                self.assertEqual(stage.GetRootLayer().ExportToString(),before)

    def test_preserves_all_other_scene_and_body_physics(self):
        stage=Usd.Stage.CreateInMemory()
        prim=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene').GetPrim()
        scene=UsdPhysics.Scene(prim)
        scene.CreateGravityMagnitudeAttr(9.81)
        api=PhysxSchema.PhysxSceneAPI.Apply(prim)
        api.CreateSolverTypeAttr('PGS')
        api.CreateTimeStepsPerSecondAttr(120)
        api.CreateEnableGPUDynamicsAttr(True)
        api.CreateBroadphaseTypeAttr('GPU')
        api.CreateGpuFoundLostPairsCapacityAttr(1024)
        body=stage.DefinePrim('/World/Body','Cube')
        UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.CollisionAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(0.25)
        before={str(p.GetPath()):{str(a.GetName()):a.Get() for a in p.GetAttributes()
                if str(a.GetName()) not in self.CAPACITY_ATTRS} for p in stage.Traverse()}
        configure_gpu_pair_capacities(stage,131072)
        after={str(p.GetPath()):{str(a.GetName()):a.Get() for a in p.GetAttributes()
               if str(a.GetName()) not in self.CAPACITY_ATTRS} for p in stage.Traverse()}
        self.assertEqual(before,after)

    def test_stronger_composed_capacity_override_is_reported(self):
        stage=Usd.Stage.CreateInMemory()
        prim=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene').GetPrim()
        api=PhysxSchema.PhysxSceneAPI.Apply(prim)
        with Usd.EditContext(stage,stage.GetSessionLayer()):
            api.CreateGpuFoundLostAggregatePairsCapacityAttr(16384)
        with self.assertRaisesRegex(RuntimeError,'capacity mismatch'):
            configure_gpu_pair_capacities(stage,131072)
        self.assertEqual(api.GetGpuFoundLostAggregatePairsCapacityAttr().Get(),16384)
        self.assertEqual(api.GetGpuTotalAggregatePairsCapacityAttr().Get(),131072)


class ContactReadbackTests(unittest.TestCase):
    class Settings:
        def __init__(self, suppress_readback):
            self.values={'/physics/suppressReadback':suppress_readback,
                         '/physics/updateToUsd':False, '/unrelated/enabled':True}
            self.writes=[]

        def set_bool(self, path, value):
            self.writes.append((path,value))
            self.values[path]=value

        def get_as_bool(self, path):
            return self.values[path]

    def test_backend_intent_overrides_final_sdk_device_setting_only(self):
        for device in ('cpu','cuda:0'):
            for backend in ('callback','tensor'):
                with self.subTest(device=device,backend=backend):
                    expected=device=='cuda:0' and backend=='tensor'
                    # Start with the opposite value to represent a device setter
                    # or prior backend overriding the intended current mode.
                    settings=self.Settings(not expected)
                    actual=configure_contact_readback(settings,device,backend)
                    self.assertEqual(actual,dict(requested_device=device,contact_backend=backend,
                                                  suppress_readback=expected))
                    self.assertEqual(settings.writes,[('/physics/suppressReadback',expected)])
                    self.assertEqual(settings.values,{'/physics/suppressReadback':expected,
                        '/physics/updateToUsd':False,'/unrelated/enabled':True})

    def test_invalid_device_or_backend_does_not_mutate_settings(self):
        for device,backend in (('cuda:1','callback'),('gpu','tensor'),(None,'callback'),
                               ('cpu','Callback'),('cuda:0',''),('cpu',None)):
            with self.subTest(device=device,backend=backend):
                settings=self.Settings(True)
                before=dict(settings.values)
                with self.assertRaisesRegex(ValueError,'Unsupported'):
                    configure_contact_readback(settings,device,backend)
                self.assertEqual(settings.values,before)
                self.assertEqual(settings.writes,[])

    def test_effective_settings_mismatch_is_reported(self):
        class LockedSettings(self.Settings):
            def set_bool(self, path, value):
                self.writes.append((path,value))
        settings=LockedSettings(True)
        with self.assertRaisesRegex(RuntimeError,'readback mismatch'):
            configure_contact_readback(settings,'cuda:0','callback')
        self.assertEqual(settings.get_as_bool('/physics/suppressReadback'),True)


if __name__=='__main__':unittest.main()
