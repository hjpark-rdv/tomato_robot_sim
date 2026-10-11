import unittest
import json
import tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import Gf,Usd,UsdPhysics
from fruit_attachment import configure_joint,frame_from_axis,relative_angles,analyze_rotations,revise,sha


class AttachmentTests(unittest.TestCase):
    def test_local_frame_tracks_neck_not_world_axis(self):
        for axis in ([1,2,-3],[0,0,1],[0,-1,0]):
            frame=frame_from_axis(axis)
            np.testing.assert_allclose(frame[:,0],np.array(axis)/np.linalg.norm(axis))
            np.testing.assert_allclose(frame.T@frame,np.eye(3),atol=1e-14)
            self.assertAlmostEqual(np.linalg.det(frame),1.)
        for axis in ([0,0,0],[1,float('nan'),0]):
            with self.assertRaises(ValueError):frame_from_axis(axis)

    def test_twist_and_bend_separate_in_arbitrary_world_pose(self):
        base=R.from_euler('xyz',[23,-40,17],degrees=True)
        twist,swing=relative_angles(base.as_quat(),(base*R.from_euler('x',40,degrees=True)).as_quat())
        self.assertAlmostEqual(np.rad2deg(twist),40.);self.assertAlmostEqual(swing,0.)
        twist,swing=relative_angles(base.as_quat(),(base*R.from_euler('y',30,degrees=True)).as_quat())
        self.assertAlmostEqual(twist,0.);self.assertAlmostEqual(np.rad2deg(swing),30.)

    def test_joint_preserves_endpoints_translations_native_release(self):
        stage=Usd.Stage.CreateInMemory();j=UsdPhysics.FixedJoint.Define(stage,'/joint')
        j.CreateBody0Rel().SetTargets(['/neck']);j.CreateBody1Rel().SetTargets(['/fruit'])
        j.CreateLocalPos0Attr(Gf.Vec3f(.01,0,0))
        configure_joint(stage,'/joint',kind='elastic',force_n=3.,torque_nm=.008)
        p=stage.GetPrimAtPath('/joint');j=UsdPhysics.Joint(p)
        self.assertEqual(p.GetTypeName(),'PhysicsJoint')
        self.assertEqual(list(map(str,j.GetBody1Rel().GetTargets())),['/fruit'])
        self.assertEqual(j.GetLocalPos0Attr().Get(),Gf.Vec3f(.01,0,0))
        self.assertTrue(j.GetExcludeFromArticulationAttr().Get())
        self.assertAlmostEqual(j.GetBreakTorqueAttr().Get(),.008)
        self.assertFalse(any('DriveAPI' in s for s in p.GetAppliedSchemas()))
        for axis in ('transX','transY','transZ'):
            api=UsdPhysics.LimitAPI(p,axis);self.assertGreater(api.GetLowAttr().Get(),api.GetHighAttr().Get())
        self.assertAlmostEqual(p.GetAttribute('physxLimit:rotX:stiffness').Get(),.01,places=8)

    def test_invalid_parameters_rejected(self):
        stage=Usd.Stage.CreateInMemory();UsdPhysics.FixedJoint.Define(stage,'/joint')
        for v in (0.,-1.,float('nan')):
            with self.assertRaises(ValueError):configure_joint(stage,'/joint',kind='elastic',force_n=3.,torque_nm=v)

    def test_rotation_evidence_excludes_detached_fall(self):
        stage=Usd.Stage.CreateInMemory()
        for path in ('/neck','/fruit'):stage.DefinePrim(path,'Xform')
        j=UsdPhysics.FixedJoint.Define(stage,'/joint');j.CreateLocalPos0Attr(Gf.Vec3f(.01,0,0))
        f=dict(name='Truss_02_Fruit_10',path='/fruit',anchor='/neck',joint='/joint')
        manifest=dict(target=f,plant=dict(fruits=[f]))
        poses=np.zeros((4,2,7));poses[:,:,6]=1.
        poses[:,1,3:]=R.from_euler('x',np.array([0,10,20,150])[:,None],degrees=True).as_quat()
        states=dict(time=np.array([0.,.1,.2,.3]),poses=poses,body_paths=np.array(['/neck','/fruit']),
                    phase=np.array(['close','belt_spin','belt_spin','open']))
        row=analyze_rotations(stage,manifest,states,[dict(joint='/joint',t=.2)])['joints'][0]
        self.assertAlmostEqual(row['max_abs_twist_before_or_at_break_deg'],20.)
        self.assertAlmostEqual(row['sample_before_break_twist_deg'],10.)
        self.assertNotIn('open',row['phase_evidence'])

    def test_revision_preserves_source_and_nonattachment_specs(self):
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'source';source.mkdir();out=Path(folder)/'variant'
            stage=Usd.Stage.CreateNew(str(source/'scene.usda'))
            for path in ('/neck','/fruit','/untouched'):stage.DefinePrim(path,'Xform')
            j=UsdPhysics.FixedJoint.Define(stage,'/joint')
            j.CreateBody0Rel().SetTargets(['/neck']);j.CreateBody1Rel().SetTargets(['/fruit'])
            j.CreateLocalPos0Attr(Gf.Vec3f(.01,0,0));j.CreateLocalPos1Attr(Gf.Vec3f(.01,0,0))
            stage.GetRootLayer().Save();original=sha(source/'scene.usda')
            fruit=dict(name='Truss_02_Fruit_10',path='/fruit',anchor='/neck',joint='/joint')
            (source/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=original,target=fruit,
                plant=dict(fruits=[fruit]))))
            np.savez(source/'skin.npz',data=np.array([1]))
            result=revise(source,out)
            self.assertTrue(result['preservation_audit']['passed'])
            self.assertEqual(sha(source/'scene.usda'),original)
            self.assertEqual(sha(source/'skin.npz'),sha(out/'skin.npz'))
            updated=json.loads((out/'motion_manifest.json').read_text())
            self.assertEqual(updated['target']['break_torque_nm'],.008)
            self.assertEqual(updated['plant']['fruits'][0]['break_torque_nm'],.008)
            self.assertEqual(stage.GetPrimAtPath('/joint').GetTypeName(),'PhysicsFixedJoint')
            with self.assertRaises(FileExistsError):revise(source,out)


if __name__=='__main__':unittest.main()
