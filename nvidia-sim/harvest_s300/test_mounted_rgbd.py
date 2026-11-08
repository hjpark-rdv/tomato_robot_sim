import copy
import ast
import hashlib
import inspect
import json
from pathlib import Path
import tempfile
import unittest
import textwrap
from types import SimpleNamespace
import numpy as np
from recording_storage import load_array, save_array, save_shared_labels
from PIL import Image
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation
from mounted_rgbd import (BODY, ROOT, CAMERA, K, WIDTH, HEIGHT, mount, optical_pose,RGBDRecorder,
                          synchronized_render_sample,CaptureSynchronizationError,
                          calibrated_mount,recalibrate_mount)


class MountedRGBDTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stage=Usd.Stage.CreateInMemory()
        body=UsdGeom.Xform.Define(cls.stage,BODY)
        cls.body_op=body.AddTransformOp()
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        cls.meta=mount(cls.stage)

    def test_follows_parent_yaw_and_lift_with_correct_optical_axes(self):
        for yaw,height in [(0,0),(1.34,.177),(-.5,.70)]:
            matrix=np.eye(4);matrix[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
            matrix[:3,3]=[0,1.26,height]
            self.body_op.Set(Gf.Matrix4d(matrix.T.tolist()))
            actual=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(CAMERA))).T
            actual[:3,1:3]*=-1
            q=np.r_[matrix[:3,3],Rotation.from_matrix(matrix[:3,:3]).as_quat()]
            np.testing.assert_allclose(actual,optical_pose(q),atol=1e-12)
            np.testing.assert_allclose(actual[:3,1],[0,0,-1],atol=1e-12)

    def test_portrait_intrinsics_are_rotated_not_stretched(self):
        c=UsdGeom.Camera(self.stage.GetPrimAtPath(CAMERA))
        fx=WIDTH*c.GetFocalLengthAttr().Get()/c.GetHorizontalApertureAttr().Get()
        fy=HEIGHT*c.GetFocalLengthAttr().Get()/c.GetVerticalApertureAttr().Get()
        np.testing.assert_allclose([fx,fy],[K[0,0],K[1,1]],rtol=1e-7)
        self.assertEqual((WIDTH,HEIGHT),(480,640))

    def test_camera_adds_no_physics_or_segmentation(self):
        for p in Usd.PrimRange(self.stage.GetPrimAtPath(ROOT)):
            self.assertFalse(p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertFalse(self.meta['segmentation_saved'])
        self.assertFalse(self.meta['physical_payload_changed'])

    def test_original_housing_long_axis_is_vertical(self):
        x=UsdGeom.Xformable(self.stage.GetPrimAtPath(ROOT+'/Housing')).GetLocalTransformation()
        np.testing.assert_array_equal(np.array(x).T[:3,0],[0,0,1])
        with self.assertRaisesRegex(ValueError,'already exists'):mount(self.stage)


class CalibratedSideCameraTests(unittest.TestCase):
    def test_fixed_side_mount_keeps_portrait_axes_on_a_rotated_robot_base(self):
        stage=Usd.Stage.CreateInMemory();body=UsdGeom.Xform.Define(stage,BODY)
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        metadata=mount(stage)
        calibration=np.eye(4);calibration[:3,:3]=[[0,-1,0],[0,0,1],[-1,0,0]]
        calibration[:3,3]=[.0325,.098,0.]
        metadata=recalibrate_mount(stage,metadata,calibration)
        pose=np.eye(4)
        pose[:3,:3]=Rotation.from_euler('x',90,degrees=True).as_matrix()@Rotation.from_euler('z',90,degrees=True).as_matrix()
        pose[:3,3]=[0,.25,1.]
        body.MakeMatrixXform().Set(Gf.Matrix4d(pose.T.tolist()))
        measured=np.r_[pose[:3,3],Rotation.from_matrix(pose[:3,:3]).as_quat()]
        expected=optical_pose(measured,metadata)
        actual=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(CAMERA))).T
        actual[:3,1:3]*=-1
        np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=0)
        np.testing.assert_allclose(actual[:3,2],[-1,0,0],atol=1e-12)
        np.testing.assert_allclose(actual[:3,1],[0,0,-1],atol=1e-12)
        housing=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(ROOT+'/Housing'))).T
        np.testing.assert_allclose(housing[:3,0],[0,0,1],atol=1e-12)
        for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):
            self.assertFalse(p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.RigidBodyAPI))

    def test_nonrigid_or_nonfinite_metadata_is_rejected(self):
        for bad in (np.full((4,4),np.nan),np.eye(3),np.diag([1.,1.,-1.,1.]),np.diag([2.,1.,1.,1.])):
            with self.assertRaises(ValueError):calibrated_mount({'body_from_camera_optical':bad})


class RenderSynchronizationTests(unittest.TestCase):
    def parameters(self,camera):
        return dict(cameraViewTransform=np.linalg.inv(camera).T.reshape(-1).tolist())

    def test_invalid_gt_readback_retries_the_whole_bundle_at_the_same_physics_state(self):
        class PayloadError(RuntimeError):
            details={'reason':'renderer_ids_missing_from_labels','missing_count':12}
        renders=[];reads=[]
        def read():
            reads.append(len(renders))
            if len(reads)==1:raise PayloadError('bad copied renderer payload')
            return ('new_rgb','new_depth',self.parameters(np.eye(4)),{'new_gt':True})
        bundle,evidence=synchronized_render_sample(np.eye(4),lambda p:renders.append(p.copy()),read,
            lambda:(30,.5),retryable_read_errors=(PayloadError,))
        self.assertEqual(reads,[1,2]);self.assertEqual(bundle[3],{'new_gt':True})
        self.assertEqual(evidence['attempt_count'],2);self.assertEqual(evidence['attempt_errors'],[None,0.])
        self.assertEqual(evidence['attempts'][0]['payload_error']['missing_count'],12)
        self.assertTrue(all(row['physics_before']==row['physics_after'] for row in evidence['attempts']))

    def test_actual_bundle_reader_owns_data_and_preserves_rejected_gt_payload(self):
        from mounted_gt import owned_gt_snapshot,GroundTruthReadbackError
        class Annotator:
            def __init__(self,array):self.array=array;self.calls=[]
            def get_data(self,**kwargs):self.calls.append(kwargs);return self.array
        class GT:
            def read_snapshot(self,expected_shape):
                return owned_gt_snapshot({'data':np.array([[2139095040,0],[0,0]],np.uint32),
                    'info':{'idToLabels':{'0':'BACKGROUND'}}},expected_shape)
        tree=ast.parse(textwrap.dedent(inspect.getsource(RGBDRecorder.capture)))
        callback=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='read_bundle')
        with tempfile.TemporaryDirectory() as directory:
            rgb=Annotator(np.full((2,2,4),20,np.uint8));depth=Annotator(np.ones((2,2),np.float32))
            params=Annotator(self.parameters(np.eye(4)))
            recorder=SimpleNamespace(rgb=rgb,depth=depth,params=params,gt=GT(),output=Path(directory),rows=[])
            failed=[];context=dict(self=recorder,np=np,copy=copy,Image=Image,json=json,save_array=save_array,save_shared_labels=save_shared_labels,
                retryable_read_errors=(GroundTruthReadbackError,),failed_reads=failed,step=30,timestamp=.5,phase='settle')
            exec(compile(ast.fix_missing_locations(ast.Module(body=[callback],type_ignores=[])),'<actual read_bundle>','exec'),context)
            with self.assertRaises(GroundTruthReadbackError):context['read_bundle']()
            saved=Path(directory)/failed[0]['evidence_directory']
            np.testing.assert_array_equal(load_array(saved/'renderer_instance.npy'),[[2139095040,0],[0,0]])
            self.assertTrue((saved/'depth.npy.gz').is_file());self.assertTrue((saved/'rgb.png').is_file())
            self.assertEqual(json.loads((saved/'error.json').read_text())['timestamp_s'],.5)
            for annotator in (rgb,depth,params):
                self.assertEqual(annotator.calls,[dict(device='cpu',do_array_copy=True)])
            recorder.gt=None
            bundle=context['read_bundle']()
            rgb.array[:]=0;depth.array[:]=4
            self.assertTrue(np.all(bundle[0]==20));self.assertTrue(np.all(bundle[1]==1))

    def test_persistent_invalid_gt_readback_is_bounded_and_never_ignored(self):
        class PayloadError(RuntimeError):details={'missing_count':10}
        renders=[]
        def read():raise PayloadError('invalid renderer IDs')
        with self.assertRaises(CaptureSynchronizationError) as caught:
            synchronized_render_sample(np.eye(4),lambda p:renders.append(p.copy()),read,
                lambda:(30,.5),retryable_read_errors=(PayloadError,))
        self.assertEqual(len(renders),3)
        self.assertEqual(caught.exception.reason,'render_payload_invalid_after_bounded_render_sync')
        self.assertFalse(caught.exception.details['physics_state_changed'])
        json.dumps(caught.exception.details,allow_nan=False)

    def test_payload_retry_cannot_hide_a_physics_clock_change(self):
        class PayloadError(RuntimeError):details={'missing_count':10}
        clock=[30,.5];renders=[]
        def read():clock[0]+=1;raise PayloadError('bad IDs after physics changed')
        with self.assertRaises(CaptureSynchronizationError) as caught:
            synchronized_render_sample(np.eye(4),lambda p:renders.append(p.copy()),read,
                lambda:tuple(clock),retryable_read_errors=(PayloadError,))
        self.assertEqual(len(renders),1)
        self.assertEqual(caught.exception.reason,'render_advanced_physics')

    def test_stale_camera_retries_same_pose_and_rereads_whole_bundle(self):
        expected=np.eye(4);original=expected.copy();stale=expected.copy();stale[0,3]=.0076804
        rendered=[];reads=[]
        def render(camera):rendered.append(camera.copy())
        def read():
            i=len(rendered);reads.append(i)
            return np.full((2,2,3),i,np.uint8),np.full((2,2),i,np.float32),self.parameters(stale if i==1 else expected)
        bundle,evidence=synchronized_render_sample(expected,render,read,lambda:(750,12.5))
        self.assertEqual(len(rendered),2);self.assertEqual(reads,[1,2])
        self.assertTrue(np.all(bundle[0]==2));self.assertTrue(np.all(bundle[1]==2))
        self.assertEqual(evidence['attempt_count'],2)
        np.testing.assert_allclose(evidence['attempt_errors'],[.0076804,0.],rtol=0,atol=1e-12)
        for camera in rendered:np.testing.assert_array_equal(camera,original)
        np.testing.assert_array_equal(expected,original)
        self.assertEqual(evidence['tolerance'],2e-5)

    def test_camera_never_accepts_a_looser_pose_or_unbounded_render_loop(self):
        camera=np.eye(4);camera[2,3]=.0076804;renders=[]
        with self.assertRaises(CaptureSynchronizationError) as caught:
            synchronized_render_sample(np.eye(4),lambda pose:renders.append(pose.copy()),
                lambda:(np.zeros((2,2,3)),np.ones((2,2)),self.parameters(camera)),lambda:(750,12.5))
        self.assertEqual(len(renders),3)
        detail=caught.exception.details
        self.assertEqual(detail['reason'],'camera_matrix_stale_after_bounded_render_sync')
        self.assertFalse(detail['physics_state_changed'])
        self.assertEqual(len(detail['attempts']),3)
        for attempt in detail['attempts']:
            self.assertEqual(attempt['physics_before'],attempt['physics_after'])
            np.testing.assert_allclose(attempt['rendered_camera_world'],camera)
        json.dumps(detail,allow_nan=False)
        with self.assertRaisesRegex(ValueError,'not be loosened'):
            synchronized_render_sample(np.eye(4),None,None,None,tolerance=.001)

    def test_physics_clock_change_is_an_immediate_error_and_is_never_retried(self):
        clock=[750,12.5];renders=[]
        def render(pose):
            renders.append(pose.copy());clock[0]+=1;clock[1]+=.01
        with self.assertRaises(CaptureSynchronizationError) as caught:
            synchronized_render_sample(np.eye(4),render,
                lambda:(None,None,self.parameters(np.eye(4))),lambda:tuple(clock))
        self.assertEqual(len(renders),1)
        self.assertEqual(caught.exception.reason,'render_advanced_physics')
        self.assertTrue(caught.exception.details['physics_state_changed'])

    def test_failed_capture_read_checks_physics_clock_before_returning_error(self):
        clock=[750,12.5]
        def render(pose):clock[0]+=1
        def fail_read():raise RuntimeError('simulated failed annotator read')
        with self.assertRaises(CaptureSynchronizationError) as caught:
            synchronized_render_sample(np.eye(4),render,fail_read,lambda:tuple(clock))
        self.assertEqual(caught.exception.reason,'render_advanced_physics')


class RecorderRegionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.output=Path(self.temp.name)
        self.recorder=RGBDRecorder.__new__(RGBDRecorder)
        self.recorder.output=self.output;self.recorder.metadata=dict(sensor='fixture')
        self.recorder.gt=object();self.recorder.harvest_region=None
        self.recorder.rows=[self.frame(0)]
        self.source='/World/Greenhouse/Plants/Plant_22/Truss_02/Fruit_01'
        self.group=dict(fruits=['Truss_02_Fruit_01'],truss_key='Truss_02',camera_region=dict(
            accepted_observations=[dict(name='Truss_02_Fruit_01',instance_id=101,key=self.source)]))

    def frame(self,index):
        directory=self.output/f'frame_{index:05d}';directory.mkdir()
        mask=np.zeros((64,64),np.uint32);mask[25:50,15:45]=101
        np.save(directory/'instance.npy',mask)
        Image.new('RGB',(64,64),(30,70,50)).save(directory/'recognition.png')
        Image.new('RGB',(192,94),(30,70,50)).save(directory/'analysis.png')
        (directory/'objects.json').write_text(json.dumps(dict(timestamp_s=float(index),objects=[
            dict(instance_id=101,key='/World/Greenhouse/Plants/Plant_22/Truss_02/Fruit_01',
                 class_name='fruit',truss='Truss_02')])))
        row=dict(frame_id=index,timestamp_s=float(index),physics_step=index*120,rgb=directory.name+'/rgb.png',
                 gt=dict(recognition='recognition.png',analysis='analysis.png',visible_fruits=1))
        (directory/'camera.json').write_text(json.dumps(row))
        return row

    def test_setting_group_updates_last_sample_metadata_without_recapture_or_source_edits(self):
        directory=self.output/'frame_00000'
        names=('instance.npy','objects.json','recognition.png','analysis.png')
        before={name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in names}
        original=copy.deepcopy(self.group)
        self.recorder.set_harvest_region(self.group)
        self.assertEqual(len(self.recorder.rows),1)
        row=self.recorder.rows[0];self.assertEqual(row['timestamp_s'],0.);self.assertEqual(row['physics_step'],0)
        self.assertEqual(row['gt']['analysis'],'analysis_region.png')
        self.assertEqual(row['gt']['recognition'],'harvest_region.png')
        self.assertEqual(row['gt']['original_analysis'],'analysis.png')
        self.assertEqual(json.loads((directory/'camera.json').read_text())['gt'],row['gt'])
        self.assertEqual(json.loads((self.output/'frames.jsonl').read_text()),row)
        self.assertEqual(before,{name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in names})
        self.assertEqual(self.group,original)
        self.group['fruits'].clear()
        self.assertEqual(self.recorder.harvest_region['fruits'],original['fruits'])

    def test_active_region_applies_to_later_capture_and_clearing_preserves_history(self):
        self.recorder.set_harvest_region(self.group)
        following=self.frame(1)
        self.recorder._annotate_harvest_region(following,self.output/'frame_00001')
        self.assertEqual(following['gt']['analysis'],'analysis_region.png')
        previous=copy.deepcopy(self.recorder.rows)
        self.recorder.set_harvest_region(None)
        self.assertEqual(self.recorder.rows,previous)
        next_observation=self.frame(2)
        self.recorder._annotate_harvest_region(next_observation,self.output/'frame_00002')
        self.assertEqual(next_observation['gt']['analysis'],'analysis.png')
        self.assertNotIn('harvest_region',next_observation)

    def test_region_requires_gt_enabled_recording(self):
        self.recorder.gt=None
        with self.assertRaisesRegex(ValueError,'GT-enabled'):self.recorder.set_harvest_region(self.group)
        self.recorder.set_harvest_region(None)


if __name__=='__main__':unittest.main()
