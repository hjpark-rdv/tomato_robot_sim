"""Device intent must control the composed scene, not only tensor readback."""
import unittest
from unittest.mock import patch
from pxr import Usd, UsdPhysics, PhysxSchema
from motion_app import configure_scene_device


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


if __name__=='__main__':unittest.main()
