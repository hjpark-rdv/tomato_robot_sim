"""Portrait D435-style RGB-D on the outward lower RB5 joint cover.

Uses the existing USD camera / Replicator RGB + optical-depth convention.
Housing is the original D435 visual, not new collision or payload calibration.
"""
import argparse
import copy
import hashlib
import json
import shutil
import time
from pathlib import Path

import numpy as np
from recording_storage import save_array, save_shared_labels
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[2]
BODY = '/World/HarvestRobot/Robot/link1'
ROOT = BODY + '/ShoulderRGBD'
CAMERA = ROOT + '/Camera'
CAPTURE_CAMERA = '/World/MountedRGBDCapture'
WIDTH, HEIGHT = 480, 640
HFOV_LANDSCAPE = 69.4
FOCAL_PX = 640 / (2 * np.tan(np.deg2rad(HFOV_LANDSCAPE / 2)))
K = np.array([[FOCAL_PX, 0, WIDTH/2], [0, FOCAL_PX, HEIGHT/2], [0, 0, 1.]])
SOURCE = REPO / 'nvidia-sim/robot_usd/rb5_farmily.usd'
SOURCE_HOUSING = '/rb5_farmily/link6/visuals/d435/ID1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def body_from_optical():
    # link1 +Y is the exposed round cover and faces the left bed in ready pose.
    # Optical +X right, +Y down, +Z forward. Long housing axis is link1 +Z.
    t = np.eye(4)
    t[:3, :3] = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    t[:3, 3] = [0., .098, .0325]
    return t


def calibrated_mount(metadata=None):
    """One fixed, validated rigid optical mount shared by USD and recording."""
    if metadata is None:
        return body_from_optical()
    value = metadata.get('body_from_camera_optical') if isinstance(metadata, dict) else metadata
    mount = np.asarray(value, dtype=float)
    if (mount.shape != (4, 4) or not np.isfinite(mount).all() or
            not np.allclose(mount[3], [0, 0, 0, 1], atol=1e-12, rtol=0) or
            not np.allclose(mount[:3, :3].T @ mount[:3, :3], np.eye(3), atol=1e-9, rtol=0) or
            not np.isclose(np.linalg.det(mount[:3, :3]), 1., atol=1e-9, rtol=0)):
        raise ValueError('Camera mount must be a finite proper rigid transform')
    return mount.copy()


def optical_pose(body_pose, metadata=None):
    p = np.asarray(body_pose)
    if p.shape != (7,) or not np.isfinite(p).all():
        raise ValueError('Expected measured body xyz + quaternion xyzw')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
    t[:3, 3] = p[:3]
    return t @ calibrated_mount(metadata)


class CaptureSynchronizationError(RuntimeError):
    def __init__(self,reason,details):
        super().__init__('RGB-D capture synchronization failed: '+reason)
        self.reason=reason;self.details=dict(details,reason=reason)


def synchronized_render_sample(expected,render,read_bundle,physics_clock,*,max_attempts=3,tolerance=2e-5,
                               retryable_read_errors=()):
    """Retry render propagation only; never advance/alter a physics snapshot.

    Every attempt rereads the complete RGB/depth/camera bundle. A valid late
    camera matrix can never be paired with imagery from an earlier attempt.
    """
    expected=np.asarray(expected,float).copy()
    if expected.shape!=(4,4) or not np.isfinite(expected).all():
        raise ValueError('Finite expected render camera matrix required')
    if isinstance(max_attempts,bool) or not isinstance(max_attempts,int) or not 1<=max_attempts<=3:
        raise ValueError('Bounded1..3 render attempts required')
    if not np.isfinite(tolerance) or tolerance<=0 or tolerance>2e-5:
        raise ValueError('Existing2e-5 camera tolerance must not be loosened')
    initial=tuple(physics_clock());attempts=[]
    details=dict(expected_camera_world=expected.tolist(),initial_physics_clock=list(initial),
        tolerance=tolerance,max_attempts=max_attempts,attempts=attempts,
        physics_state_changed=False,expected_camera_changed_between_attempts=False)
    for index in range(max_attempts):
        before=tuple(physics_clock())
        entry=dict(attempt=index+1,physics_before=list(before))
        attempts.append(entry)
        if before!=initial:
            details['physics_state_changed']=True
            raise CaptureSynchronizationError('physics_clock_changed_before_render',details)
        try:
            render(expected.copy())
            bundle=read_bundle()
        except Exception as error:
            after_error=tuple(physics_clock())
            entry.update(capture_error=type(error).__name__+': '+str(error),
                         physics_after=list(after_error))
            if after_error!=before:
                details['physics_state_changed']=True
                raise CaptureSynchronizationError('render_advanced_physics',details) from error
            if isinstance(error,retryable_read_errors):
                entry['payload_error']=getattr(error,'details',{})
                continue
            raise CaptureSynchronizationError('render_or_bundle_read_error',details) from error
        after=tuple(physics_clock());entry['physics_after']=list(after)
        if after!=before:
            details['physics_state_changed']=True
            raise CaptureSynchronizationError('render_advanced_physics',details)
        try:
            parameters=bundle[2]
            view=np.asarray(parameters['cameraViewTransform'],float).reshape(4,4).T
            if not np.isfinite(view).all():raise ValueError('Nonfinite renderer view matrix')
            actual=np.linalg.inv(view)
            error=float(np.max(np.abs(actual-expected)))
            if not np.isfinite(error):raise ValueError('Nonfinite rendered camera error')
            entry.update(renderer_view_matrix=view.tolist(),rendered_camera_world=actual.tolist(),
                         renderer_extrinsic_max_error=error)
        except (ValueError,KeyError,TypeError,np.linalg.LinAlgError) as error:
            entry.update(renderer_view_matrix=None,rendered_camera_world=None,
                         renderer_extrinsic_max_error=None,annotation_error=type(error).__name__+': '+str(error))
            continue
        if error<=tolerance:
            return bundle,dict(attempt_count=len(attempts),attempt_errors=[x.get('renderer_extrinsic_max_error') for x in attempts],
                attempts=attempts,tolerance=tolerance,physics_time_unchanged=True)
    reason=('render_payload_invalid_after_bounded_render_sync' if attempts and
            'payload_error' in attempts[-1] else 'camera_matrix_stale_after_bounded_render_sync')
    raise CaptureSynchronizationError(reason,details)


def mount(stage, *, body_from_camera_optical=None):
    if not stage.GetPrimAtPath(BODY):
        raise ValueError('Missing lower rotating base link: ' + BODY)
    if stage.GetPrimAtPath(ROOT):
        raise ValueError('RGB-D mount already exists')
    from build_scene import box
    rig = UsdGeom.Xform.Define(stage, ROOT)
    rig.AddTranslateOp().Set(Gf.Vec3d(0, .097, 0))
    # Original visual front plane is z=0; the housing extends 25.05mm behind it.
    source = Usd.Stage.Open(str(SOURCE)).Flatten()
    Sdf.CopySpec(source, SOURCE_HOUSING, stage.GetEditTarget().GetLayer(), ROOT+'/Housing')
    housing = stage.GetPrimAtPath(ROOT+'/Housing')
    calibration = calibrated_mount(body_from_camera_optical)
    rotation = np.eye(4)
    rotation[:3,:3] = np.column_stack((-calibration[:3,1], calibration[:3,0], calibration[:3,2]))
    UsdGeom.Xformable(housing).MakeMatrixXform().Set(Gf.Matrix4d(rotation.T.tolist()))
    bracket=box(stage, ROOT+'/Bracket', (0,-.0305,0), (.022,.009,.075), (.16,.18,.20))
    bracket.GetPrim().RemoveAPI(UsdPhysics.CollisionAPI)
    delta=np.eye(4);delta[:3,:3]=calibration[:3,:3]@body_from_optical()[:3,:3].T
    bracket_tf=np.array(UsdGeom.Xformable(bracket).GetLocalTransformation()).T
    UsdGeom.Xformable(bracket).MakeMatrixXform().Set(Gf.Matrix4d((delta@bracket_tf).T.tolist()))
    camera = UsdGeom.Camera.Define(stage, CAMERA)
    local = calibration.copy()
    local[:3,3] -= [0,.097,0]
    local[:3,1:3] *= -1  # optical -> USD camera
    camera.MakeMatrixXform().Set(Gf.Matrix4d(local.T.tolist()))
    # Rotate the original 640x480 intrinsics into a portrait sensor, rather than
    # stretching or cropping its image after capture.
    camera.CreateHorizontalApertureAttr(20.955 * WIDTH/HEIGHT)
    camera.CreateVerticalApertureAttr(20.955)
    camera.CreateFocalLengthAttr(20.955 * FOCAL_PX/640)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.1, 20.))
    camera.CreateFocusDistanceAttr(.8)
    camera.CreateFStopAttr(0.)
    camera.CreateExposureAttr(2.)
    for prim in Usd.PrimRange(rig.GetPrim()):
        if prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError('Optical sensor attachment must not change physical bodies/colliders')
    result = dict(schema='s300-mounted-rgbd-v1', body_path=BODY, root_path=ROOT,
        camera_path=CAMERA, body_from_camera_optical=calibration.tolist(),
        resolution_wh=[WIDTH,HEIGHT], K=K.tolist(), depth_units='m',
        exposure_ev=2.,
        depth_definition='optical +Z distance_to_image_plane, not radial distance',
        depth_range_m=[.1,5.], invalid_depth='NaN',
        camera_model='D435-style ideal aligned RGB-D; existing 69.4deg landscape FOV rotated to portrait',
        sensor_model_limit='No stereo matching, holes/noise, exposure or real-device calibration',
        housing_source=str(SOURCE),housing_source_sha256=sha(SOURCE),
        housing_source_prim=SOURCE_HOUSING,housing_size_m=[.025,.02505,.09],
        mount_description='link1 outward +Y round shoulder cover; follows base yaw and lift, not shoulder pitch',
        housing_and_bracket_collision=False,physical_payload_changed=False,
        segmentation_saved=False,training_eligible=False)
    return result


def recalibrate_mount(stage, metadata, body_from_camera_optical):
    """Rotate an existing rigid camera/housing/bracket together on its cover."""
    calibration=calibrated_mount(body_from_camera_optical)
    previous=calibrated_mount(metadata)
    rig=stage.GetPrimAtPath(ROOT)
    if not rig or not stage.GetPrimAtPath(CAMERA):
        raise ValueError('Existing mounted camera required for rigid recalibration')
    delta=np.eye(4);delta[:3,:3]=calibration[:3,:3]@previous[:3,:3].T
    for suffix in ('Housing','Bracket'):
        prim=stage.GetPrimAtPath(ROOT+'/'+suffix)
        if not prim:raise ValueError('Missing calibrated camera '+suffix)
        pose=np.array(UsdGeom.Xformable(prim).GetLocalTransformation()).T
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d((delta@pose).T.tolist()))
    rig_pose=np.array(UsdGeom.Xformable(rig).GetLocalTransformation()).T
    local=np.linalg.inv(rig_pose)@calibration
    local[:3,1:3]*=-1
    UsdGeom.Xformable(stage.GetPrimAtPath(CAMERA)).MakeMatrixXform().Set(Gf.Matrix4d(local.T.tolist()))
    return dict(metadata,body_from_camera_optical=calibration.tolist(),
                mount_calibration_revision='Physical rigid housing/bracket roll on existing link1 cover')


def prepare(assembly, output):
    source=assembly/'composed.usdc'
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    if sha(assembly/'scene.usda') != manifest['scene_sha256']:
        raise ValueError('Source scene SHA mismatch')
    from compact_motion_assets import verify_compact_dependencies
    verify_compact_dependencies(assembly,manifest)
    output.mkdir(parents=True,exist_ok=False)
    stage=Usd.Stage.CreateNew(str(output/'scene.usda'))
    stage.GetRootLayer().subLayerPaths=[str(source.resolve())]
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    result=mount(stage)
    stage.Flatten().Export(str(output/'composed.usdc'))
    stage.GetRootLayer().Clear();stage.GetRootLayer().subLayerPaths=['composed.usdc']
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'));stage.GetRootLayer().Save()
    manifest['mounted_rgbd']=result
    manifest['sensor_source']=dict(assembly=str(assembly),composed_sha256=sha(source))
    manifest['scene_sha256']=sha(output/'scene.usda')
    manifest['compact_assets']['materialized_dependency']['sha256']=sha(output/'composed.usdc')
    manifest['compact_assets']['geometry_unchanged']=False
    manifest['compact_assets']['native_validation']='Added optical sensor visual only; see separate RGB-D validation'
    (output/'motion_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'mounted_rgbd.json').write_text(json.dumps(result,indent=2)+'\n')
    for pattern in ('skin*.npz','real_lift_mount.json'):
        for path in assembly.glob(pattern):shutil.copy2(path,output/path.name)
    print(json.dumps(result,indent=2),flush=True)


class RGBDRecorder:
    """Sample a fixed physics state; render updates must not advance physics."""
    def __init__(self,stage,output,metadata,physics_hz,fps,body_paths,*,gt_assembly=None):
        import omni.replicator.core as rep
        import carb
        self.stage=stage;self.output=Path(output);self.metadata=metadata
        self.output.mkdir(parents=True,exist_ok=False)
        self.body_index=list(body_paths).index(BODY)
        self.stride=int(round(physics_hz/fps))
        self.actual_fps=physics_hz/self.stride;self.rows=[];self.harvest_region=None
        # PhysX Fabric can move a rigid parent without refreshing the inherited
        # USD camera matrix. Mirror the *measured* fixed mount pose into a world
        # camera, as the existing tool-camera capture does. This edits no body,
        # joint or collider, and never aims at a GT fruit or nominal waypoint.
        source=UsdGeom.Camera(stage.GetPrimAtPath(CAMERA))
        sensor=UsdGeom.Camera.Define(stage,CAPTURE_CAMERA)
        for name in ('projection','horizontalAperture','verticalAperture','focalLength',
                     'clippingRange','focusDistance','fStop'):
            attr=source.GetPrim().GetAttribute(name)
            sensor.GetPrim().GetAttribute(name).Set(attr.Get())
        # Current RTX ignores plain USD exposure on this render-product path.
        # Use its supported tone-mapper ISO and record/restore the override.
        sensor.CreateExposureAttr(0.)
        self.settings=carb.settings.get_settings()
        self.iso_key='/rtx/post/tonemap/filmIso'
        self.original_iso=float(self.settings.get(self.iso_key))
        sensor_iso=self.original_iso*2**metadata.get('exposure_ev',0.)
        self.settings.set_float(self.iso_key,sensor_iso)
        self.metadata=dict(metadata,render_exposure=dict(method='RTX tone-mapper ISO; USD camera exposure zero to avoid double application',
            original_iso=self.original_iso,capture_iso=sensor_iso,
            exposure_time_s=self.settings.get('/rtx/post/tonemap/exposureTime'),
            f_number=self.settings.get('/rtx/post/tonemap/fNumber'),
            scope='this capture process; restored when recorder closes'))
        self.camera_op=sensor.MakeMatrixXform()
        self.product=rep.create.render_product(CAPTURE_CAMERA,(WIDTH,HEIGHT))
        self.rgb=rep.AnnotatorRegistry.get_annotator('rgb')
        self.depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
        self.params=rep.AnnotatorRegistry.get_annotator('camera_params')
        for a in (self.rgb,self.depth,self.params):a.attach(self.product)
        self.gt=None
        if gt_assembly is not None:
            from mounted_gt import GroundTruthOverlay
            self.gt=GroundTruthOverlay(stage,self.product,self.output,gt_assembly)
            self.metadata.update(segmentation_saved=True,segmentation_source='visible SIM-GT renderer IDs + USD identity',
                                 gt_catalog='gt_catalog.json')

    def due(self,step):
        return step % self.stride == 0

    def _annotate_harvest_region(self,row,directory):
        """Derive visible contours without changing a captured sensor sample."""
        if self.harvest_region is None:return
        if 'gt' not in row:raise ValueError('Harvest-region display requires captured visible GT')
        from camera_region_visual import annotate_region
        derived=annotate_region(directory,self.harvest_region)
        row['gt'].setdefault('original_recognition',row['gt'].get('recognition','recognition.png'))
        row['gt'].setdefault('original_analysis',row['gt'].get('analysis','analysis.png'))
        row['gt'].update(derived)
        row['harvest_region']=dict(selected_fruits=list(self.harvest_region.get('fruits',[])),
            truss=self.harvest_region.get('truss_key'),region=derived['region'],
            visible_selected_count=derived['visible_selected_count'],
            source='accepted existing plan group; contours from current visible instance masks')

    def set_harvest_region(self,group):
        """Record an accepted plan's region on the latest measured exposure.

        Clearing stops future annotation and preserves historical frames. No
        new exposure, renderer update, physics step or invented mask is used.
        """
        if group is not None and self.gt is None:
            raise ValueError('Harvest-region display requires GT-enabled RGB-D capture')
        self.harvest_region=copy.deepcopy(group)
        if group is None or not self.rows:return
        row=self.rows[-1];directory=self.output/Path(row['rgb']).parent
        self._annotate_harvest_region(row,directory)
        camera=directory/'camera.json'
        temporary=directory/'camera.json.tmp'
        temporary.write_text(json.dumps(dict(self.metadata,**row),indent=2,allow_nan=False)+'\n')
        temporary.replace(camera)
        timeline=self.output/'frames.jsonl.tmp'
        timeline.write_text(''.join(json.dumps(value,allow_nan=False)+'\n' for value in self.rows))
        timeline.replace(self.output/'frames.jsonl')

    def capture(self,app,sim,poses,q,step,timestamp,phase,*,sync):
        from PIL import Image
        import omni.replicator.core as rep
        started=time.perf_counter()
        pose=optical_pose(poses[self.body_index],self.metadata)
        expected=pose.copy();expected[:3,1:3]*=-1
        retryable_read_errors=()
        if self.gt is not None:
            from mounted_gt import GroundTruthReadbackError
            retryable_read_errors=(GroundTruthReadbackError,)
        failed_reads=[]
        def render(camera):
            sync()
            self.camera_op.Set(Gf.Matrix4d(camera.T.tolist()))
            # USD/Fabric camera propagation may lag a newly issued pose. Hold
            # exactly the same physical state while flushing render products.
            rep.orchestrator.step(rt_subframes=2,delta_time=0.0,pause_timeline=False)
        def read_bundle():
            rgb=np.asarray(self.rgb.get_data(device='cpu',do_array_copy=True))[:,:,:3].copy()
            depth=np.asarray(self.depth.get_data(device='cpu',do_array_copy=True)).squeeze().astype(np.float32).copy()
            parameters=copy.deepcopy(self.params.get_data(device='cpu',do_array_copy=True))
            snapshot=None
            if self.gt is not None:
                try:snapshot=self.gt.read_snapshot(expected_shape=depth.shape)
                except retryable_read_errors as error:
                    diagnostic=self.output/'readback_failures'/f'frame_{len(self.rows):05d}_attempt_{len(failed_reads)+1:02d}'
                    diagnostic.mkdir(parents=True,exist_ok=False)
                    if isinstance(error.raw,np.ndarray) and not error.raw.dtype.hasobject:
                        save_array(diagnostic/'renderer_instance.npy',error.raw)
                    else:error.details['raw_numeric_array_available']=False
                    save_array(diagnostic/'depth.npy',depth)
                    Image.fromarray(rgb).save(diagnostic/'rgb.png')
                    save_shared_labels(diagnostic/'renderer_labels.json',error.labels,self.output/'shared_labels')
                    error.details.update(evidence_directory=str(diagnostic.relative_to(self.output)),
                        requested_physics_step=int(step),timestamp_s=float(timestamp),phase=phase)
                    (diagnostic/'error.json').write_text(json.dumps(error.details,indent=2,allow_nan=False)+'\n')
                    failed_reads.append(dict(error.details))
                    raise
            return rgb,depth,parameters,snapshot
        try:
            (rgb,depth,parameters,gt_snapshot),render_sync=synchronized_render_sample(expected,render,read_bundle,
                lambda:(sim.get_num_physics_steps(),sim.get_simulation_time()),
                retryable_read_errors=retryable_read_errors)
        except CaptureSynchronizationError as error:
            diagnostic=dict(error.details,requested_physics_step=int(step),timestamp_s=float(timestamp),
                phase=phase,frames_already_saved=len(self.rows),
                measured_mount_body_pose=np.asarray(poses[self.body_index]).tolist(),
                camera_path=CAPTURE_CAMERA,physics_step_changed_by_capture=error.details['physics_state_changed'])
            (self.output/'capture_sync_error.json').write_text(json.dumps(diagnostic,indent=2,allow_nan=False)+'\n')
            raise
        if rgb.shape!=(HEIGHT,WIDTH,3) or depth.shape!=(HEIGHT,WIDTH):
            raise RuntimeError('Invalid mounted RGB-D frame shape: '+str((rgb.shape,depth.shape)))
        if rgb.std()<1:raise RuntimeError('RGB frame is uniform; inspect camera occlusion/render readiness')
        extrinsic_error=render_sync['attempt_errors'][-1]
        lo,hi=self.metadata['depth_range_m']
        depth[~np.isfinite(depth)|(depth<lo)|(depth>hi)]=np.nan
        n=len(self.rows);directory=self.output/f'frame_{n:05d}';directory.mkdir()
        Image.fromarray(rgb).save(directory/'rgb.png')
        if self.gt is None:save_array(directory/'depth.npy',depth)
        # Preview only, never used as metric depth or segmentation.
        valid=np.isfinite(depth);grey=np.zeros_like(depth,dtype=np.uint8)
        grey[valid]=np.clip(255*(1-(depth[valid]-lo)/(hi-lo)),0,255).astype(np.uint8)
        Image.fromarray(grey).save(directory/'depth_preview.png')
        row=dict(frame_id=n,physics_step=step,timestamp_s=float(timestamp),phase=phase,
            capture_camera_path=CAPTURE_CAMERA,
            pose_source='measured native mount-body pose times fixed mount transform; world camera mirror for Fabric rendering',
            rgb=f'frame_{n:05d}/rgb.png',depth=f'frame_{n:05d}/depth.npy.gz',depth_encoding='gzip-npy',
            depth_preview=f'frame_{n:05d}/depth_preview.png',
            world_from_camera_optical=pose.tolist(),measured_mount_body_pose=np.asarray(poses[self.body_index]).tolist(),
            robot_q=np.asarray(q).tolist(),valid_depth_fraction=float(valid.mean()),
            valid_depth_min_m=float(np.nanmin(depth)) if valid.any() else None,
            valid_depth_max_m=float(np.nanmax(depth)) if valid.any() else None,
            renderer_extrinsic_max_error=extrinsic_error,render_did_not_advance_physics=True,
            render_sync_attempt_count=render_sync['attempt_count'],
            renderer_extrinsic_attempt_errors=render_sync['attempt_errors'],
            render_payload_retry_count=len(failed_reads),render_payload_failures=failed_reads,
            render_sync_tolerance=render_sync['tolerance'],
            capture_wall_s=time.perf_counter()-started)
        if self.gt is not None:
            row['gt']=self.gt.save(directory,rgb,depth,self.metadata['K'],pose,timestamp,snapshot=gt_snapshot,save_depth=True)
            self._annotate_harvest_region(row,directory)
            row['capture_wall_s']=time.perf_counter()-started
        (directory/'camera.json').write_text(json.dumps(dict(self.metadata,**row),indent=2)+'\n')
        self.rows.append(row)
        with (self.output/'frames.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')

    def close(self):
        if self.gt is not None:self.gt.close()
        for a in (self.rgb,self.depth,self.params):a.detach(self.product)
        self.product.destroy()
        self.settings.set_float(self.iso_key,self.original_iso)
        result=dict(self.metadata,frames=len(self.rows),fps=self.actual_fps,
            capture_wall_s=sum(r['capture_wall_s'] for r in self.rows),
            captures='frames.jsonl',physics_time_unchanged_by_all_captures=True)
        (self.output/'camera.json').write_text(json.dumps(result,indent=2)+'\n')
        if self.gt is not None:
            from mounted_gt import write_review
            result['gt_review']=write_review(self.output,self.actual_fps)
        return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();prepare(args.assembly,args.output)
