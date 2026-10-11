import unittest
import tempfile
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from analyze_belt_trial import measure,measure_cluster


class EvidenceTests(unittest.TestCase):
    def fixture(self):
        pose=np.zeros((5,2,7));pose[:,:,6]=1.
        return dict(time=np.array([0.,.5,1.,1.5,2.]),phase=np.array(['initial']+['belt_spin']*4),
            poses=pose,body_paths=['/plant/fruit11','/plant/neck11'],robot_q=np.zeros((5,2)))

    def evidence(self,states,contacts=(),breaks=()):
        return measure(states,contacts,breaks,
            dict(path='/plant/fruit11',anchor='/plant/neck11',joint='/joints/fruit11'),
            ['/tool/L','/tool/R'],['jaw_L','jaw_R'],[np.zeros(3),np.zeros(3)])

    def test_whole_truss_rotation_is_not_relative_fruit_spin(self):
        states=self.fixture();quats=Rotation.from_euler('z',np.array([0,20,40,60,80])[:,None],degrees=True).as_quat()
        states['poses'][:,0,3:]=quats;states['poses'][:,1,3:]=quats
        e=self.evidence(states)
        self.assertAlmostEqual(e['fruit_rotation_world_max_deg'],80.)
        self.assertLess(e['fruit_rotation_relative_to_pedicel_max_deg'],1e-10)
        self.assertFalse(e['native_target_joint_broken']);self.assertIsNone(e['harvest_success'])
        self.assertEqual(e['phase_evidence']['belt_spin']['executed_s'],2.)

    def test_real_relative_rotation_and_exact_break_and_contact_identity(self):
        states=self.fixture();states['poses'][:,0,3:]=Rotation.from_euler('z',np.array([0,60,120,180,240])[:,None],degrees=True).as_quat()
        contacts=[dict(a='/plant/fruit110/Collider',b='/tool/L/part',force_n=9.,t=1.,separation_m=-.002),
                  dict(a='/plant/fruit11/Collider',b='/tool/R/part',force_n=.5,t=1.,separation_m=-.0001)]
        breaks=[dict(joint='/joints/fruit110',t=1.)]
        e=self.evidence(states,contacts,breaks)
        self.assertFalse(e['native_target_joint_broken'])
        self.assertEqual(e['belt_target_contacts']['/tool/L']['points'],0)
        self.assertEqual(e['belt_target_contacts']['/tool/R']['points'],1)
        self.assertAlmostEqual(e['fruit_rotation_relative_to_pedicel_travel_deg'],240.)
        e=self.evidence(states,contacts,breaks+[dict(joint='/joints/fruit11',t=1.5)])
        self.assertTrue(e['native_target_joint_broken']);self.assertIsNone(e['harvest_success'])

    def test_incomplete_spin_and_timestamp_gap_are_preserved(self):
        states=self.fixture();states['time']=np.array([0.,.1,.2,.3,1.2]);states['phase'][-1]='release_open'
        e=self.evidence(states)
        self.assertAlmostEqual(e['phase_evidence']['belt_spin']['executed_s'],.3)
        self.assertAlmostEqual(e['timestamp_max_gap_s'],.9)
        states['time'][-1]=.3
        with self.assertRaises(ValueError):self.evidence(states)

    def test_neighbor_contact_is_recorded_but_only_exact_fruit_break_counts(self):
        states=self.fixture();fruits=[dict(name='11',path='/plant/fruit11',joint='/joint/11'),
                                    dict(name='110',path='/plant/neck11',joint='/joint/110')]
        events=[dict(a='/World/ActuatedTool/L/part',b='/plant/fruit11/Collider',t=.5,force_n=1.,separation_m=-.0001),
                dict(a='/World/ActuatedTool/R/part',b='/plant/neck11/Collider',t=.5,force_n=2.,separation_m=-.0002)]
        result=measure_cluster(states,events,[dict(joint='/joint/110',t=1.)],fruits,
                               ['/World/ActuatedTool/L','/World/ActuatedTool/R'])
        self.assertEqual(result['native_detached_count'],1);self.assertFalse(result['fruits'][0]['native_detached'])
        self.assertEqual(result['contacted_fruit_count'],2)
        self.assertEqual(result['both_belts_touch_any_fruit_during_spin_s'],.5)
        self.assertIsNone(result['harvest_success'])
        states['phase']=np.array(['initial','close','cycle_open','cycle_reclose','hold'])
        states['belt_surface_velocity_local_y_m_s']=np.array([[0,0],[-.05,.05],[-.05,.05],[-.05,.05],[0,0]])
        result=measure_cluster(states,events,[],fruits,['/World/ActuatedTool/L','/World/ActuatedTool/R'])
        self.assertEqual(result['both_belts_touch_any_fruit_during_spin_s'],0.)
        self.assertEqual(result['both_belts_touch_any_fruit_while_commanded_s'],.5)
        states['belt_surface_velocity_local_y_m_s'][:]=0
        result=measure_cluster(states,events,[],fruits,['/World/ActuatedTool/L','/World/ActuatedTool/R'])
        self.assertEqual(result['both_belts_touch_any_fruit_while_commanded_s'],0.)


class TreadPrototypeTests(unittest.TestCase):
    def test_prototype_preserves_source_and_actuator_and_adds_mass_with_visible_colliders(self):
        from pxr import Usd,UsdGeom,UsdPhysics
        from motion_assets import rigid,revise_belt_tread,sha
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'source';source.mkdir();dest=Path(tmp)/'prototype'
            stage=Usd.Stage.CreateNew(str(source/'scene.usda'))
            UsdGeom.Xform.Define(stage,'/World');UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
            for side in ('L','R'):rigid(stage,'/World/ActuatedTool/'+side+'_belt',np.eye(4),.1)
            j=UsdPhysics.PrismaticJoint.Define(stage,'/World/ActuatedTool/Joints/jaw_L')
            j.CreateLowerLimitAttr(-.026);j.CreateUpperLimitAttr(.008)
            UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear').CreateMaxForceAttr(10.)
            stage.GetRootLayer().Save();before=sha(source/'scene.usda')
            manifest=dict(scene_sha256=before,tool=dict(jaw_limits_mm=[28.,96.],mass_estimates_kg={'L_belt':.1,'R_belt':.1}))
            (source/'motion_manifest.json').write_text(json.dumps(manifest));np.savez(source/'skin.npz',a=[1])
            revise_belt_tread(source,dest,5.)
            self.assertEqual(sha(source/'scene.usda'),before)
            result=Usd.Stage.Open(str(dest/'scene.usda'))
            joint=UsdPhysics.PrismaticJoint(result.GetPrimAtPath(str(j.GetPath())))
            self.assertAlmostEqual(joint.GetUpperLimitAttr().Get(),.008)
            self.assertEqual(UsdPhysics.DriveAPI(joint.GetPrim(),'linear').GetMaxForceAttr().Get(),10.)
            for side in ('L','R'):
                body='/World/ActuatedTool/'+side+'_belt';prim=result.GetPrimAtPath(body+'/PrototypeTread')
                self.assertTrue(prim.IsA(UsdGeom.Mesh));self.assertTrue(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())
                self.assertAlmostEqual(UsdPhysics.MassAPI(result.GetPrimAtPath(body)).GetMassAttr().Get(),.13744,places=6)
            with self.assertRaises(ValueError):revise_belt_tread(dest,Path(tmp)/'again',5.)


if __name__=='__main__':unittest.main()
