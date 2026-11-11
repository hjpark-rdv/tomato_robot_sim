"""Occlusion-aware mounted RGB/depth/SIM-GT using Newton's GPU SensorCamera.

The render-only model contains the original full-farm triangles. It never
advances physics. Every exposure uses the same measured body transforms for
RGB, optical-Z depth and instance IDs. No detector or hidden fruit projection.
"""
import json
from pathlib import Path
import time

import mujoco as mj
import newton
from newton.sensors import SensorCamera
import numpy as np
from scipy.spatial.transform import Rotation
import warp as wp

from newton_viser import geom_mesh, geom_color
from mounted_gt import visible_objects, overlays
from newton_inputs import sha256


def camera_world(body_pose, body_from_optical):
    pose=np.asarray(body_pose,float)
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(pose[3:]).as_matrix();t[:3,3]=pose[:3]
    return t@np.asarray(body_from_optical,float)


def shape_instances(shape_image, lut):
    shapes=np.asarray(shape_image)
    result=np.zeros(shapes.shape,np.uint32)
    valid=shapes<len(lut)
    result[valid]=np.asarray(lut,np.uint32)[shapes[valid]]
    return result


class MountedCamera:
    def __init__(self, assets, *, device='cuda:0', scale=1., packed=None):
        assets=Path(assets);started=time.perf_counter()
        self.assets=json.loads((assets/'patrol_assets.json').read_text())
        if packed is None:
            catalog=json.loads((assets/'sensor_catalog.json').read_text())
            self.visual=mj.MjModel.from_xml_path(str(assets/'model/scene.xml'))
        else:
            from newton_scene_chunks import PackedScene
            folder,section,manifest=packed
            if sha256(Path(folder)/section['catalog'])!=section['catalog_sha256']:
                raise ValueError('Packed sensor catalog SHA mismatch')
            catalog=json.loads((Path(folder)/section['catalog']).read_text())
            self.visual=PackedScene(Path(folder)/section['geometry'],section['geometry_sha256'])
        self.objects={int(k):v for k,v in catalog['objects'].items()}
        self.mount=self.assets.get('mounted_rgbd') or json.loads((Path(self.assets['plants'][0]['assembly'])/'mounted_rgbd.json').read_text())
        names={self.visual.geom(i).name for i in range(self.visual.ngeom)}
        if set(catalog['geom_to_instance'])-names:
            raise ValueError('Sensor catalog references absent visual geometry')
        if any(int(i) not in self.objects for i in catalog['geom_to_instance'].values()):
            raise ValueError('Sensor geometry references absent instance identity')
        self.provenance=dict(visual_xml_sha256=sha256(assets/'model/scene.xml') if packed is None else manifest['source_hashes']['model/scene.xml'],
            sensor_catalog_sha256=sha256(assets/'sensor_catalog.json') if packed is None else manifest['source_hashes']['sensor_catalog.json'],
            mounted_camera_sha256=(sha256(assets/'patrol_assets.json') if 'mounted_rgbd' in self.assets else
                sha256(Path(self.assets['plants'][0]['assembly'])/'mounted_rgbd.json')))
        if packed is not None:self.provenance['packed_geometry_sha256']=section['geometry_sha256']
        if 'camera_relocation' in self.assets:
            relocation=self.assets['camera_relocation']
            if self.provenance['visual_xml_sha256']!=relocation['visual_xml_sha256']:
                raise ValueError('Relocated camera visual model SHA mismatch')
        if not 0 < scale <= 1:raise ValueError('Sensor scale must be within (0,1]')
        self.width,self.height=[int(round(x*scale)) for x in self.mount['resolution_wh']]
        self.K=np.array(self.mount['K'],float);self.K[:2]*=scale
        builder=newton.ModelBuilder()
        cfg=newton.ModelBuilder.ShapeConfig(density=0.,has_shape_collision=False,has_particle_collision=False)
        self.body_names=[self.visual.body(i).name for i in range(1,self.visual.nbody)]
        for name in self.body_names:builder.add_body(mass=0.,label=name)
        cache={};lut=[]
        for i in range(self.visual.ngeom):
            rgba=geom_color(self.visual,i)
            if rgba[3]<=0:continue
            key=(int(self.visual.geom_type[i]),int(self.visual.geom_dataid[i]),tuple(self.visual.geom_size[i]))
            if key not in cache:
                mesh=geom_mesh(self.visual,i,local_transform=False)
                cache[key]=newton.Mesh(mesh.vertices.astype(np.float32),mesh.faces.flatten().astype(np.int32),compute_inertia=False)
            q=self.visual.geom_quat[i]
            builder.add_shape_mesh(int(self.visual.geom_bodyid[i])-1,
                xform=wp.transform(self.visual.geom_pos[i],q[[1,2,3,0]]),mesh=cache[key],
                color=rgba[:3],opacity=float(rgba[3]),cfg=cfg,label=self.visual.geom(i).name)
            lut.append(catalog['geom_to_instance'].get(self.visual.geom(i).name,0))
        self.lut=np.asarray(lut,np.uint32)
        self.model=builder.finalize(device=device);self.state=self.model.state()
        config=SensorCamera.RenderConfig(enable_backface_culling=False)
        self.sensor=SensorCamera(self.model,default_render_config=config)
        self.sensor.create_default_light()
        self.rays=SensorCamera.compute_camera_rays_pinhole_opencv(self.width,self.height,
            self.K[0,0],self.K[1,1],self.K[0,2],self.K[1,2],device=device)
        self.transforms=wp.empty(1,dtype=wp.transformf,device=device)
        self.rgb=self.sensor.allocate_image_color(1,self.width,self.height)
        self.depth=self.sensor.allocate_image_depth_forward(1,self.width,self.height)
        self.ids=self.sensor.allocate_image_shape_index(1,self.width,self.height)
        self.setup_wall_s=time.perf_counter()-started

    def capture(self, body_poses, camera_body_name, timestamp, *, output=None, station=None, include_raw=False):
        started=time.perf_counter()
        missing=set(self.body_names)-body_poses.keys()
        if missing:raise ValueError('Measured camera scene body poses missing: '+str(sorted(missing)[:5]))
        pose=np.asarray([body_poses[n] for n in self.body_names],np.float32)
        if not np.isfinite(pose).all():raise ValueError('Nonfinite camera scene')
        self.state.body_q.assign(pose)
        optical=camera_world(body_poses[camera_body_name],self.mount['body_from_camera_optical'])
        gl=optical.copy();gl[:3,:3]=optical[:3,:3]@np.diag([1.,-1.,-1.])
        self.transforms.assign(np.array([np.r_[gl[:3,3],Rotation.from_matrix(gl[:3,:3]).as_quat()]],np.float32))
        self.model.bvh_refit_shapes(self.state)
        self.sensor.update(self.state,self.transforms,self.rays,color_image=self.rgb,
            forward_depth_image=self.depth,shape_index_image=self.ids)
        wp.synchronize_device(self.model.device)
        rgb=self.rgb.numpy()[0].view(np.uint8).reshape(self.height,self.width,4)[:,:,:3].copy()
        # Same authored camera exposure metadata; display RGB only. Depth,
        # visible IDs and ripeness labels never depend on this tone adjustment.
        srgb=rgb.astype(float)/255.
        linear=np.where(srgb<=.04045,srgb/12.92,((srgb+.055)/1.055)**2.4)
        linear=np.clip(linear*2.**float(self.mount.get('exposure_ev',0.)),0.,1.)
        rgb=(255*np.where(linear<=.0031308,linear*12.92,1.055*linear**(1/2.4)-.055)).astype(np.uint8)
        depth=self.depth.numpy()[0].copy()
        lo,hi=self.mount['depth_range_m'];depth[(depth<lo)|(depth>hi)]=np.nan
        instance=shape_instances(self.ids.numpy()[0],self.lut)
        semantic=np.zeros(instance.shape,np.uint8)
        for identity in np.unique(instance):
            if identity:semantic[instance==identity]=self.objects[int(identity)]['class_id']
        visible=visible_objects(instance,depth,self.objects,self.K,optical)
        observation=dict(schema='s300-newton-mounted-observation-v1',objects=visible,
            world_from_camera_optical=optical.tolist(),timestamp_s=float(timestamp),current_timestamp_s=float(timestamp),
            synchronized=True,valid=True,age_s=0.,uses_hidden_gt_centers=False,
            pose_source='measured Newton GPU state, CPU FK only; render-only Newton SensorCamera CUDA ray tracing',
            render_wall_s=time.perf_counter()-started,K=self.K.tolist(),resolution_wh=[self.width,self.height],
            source_hashes=self.provenance)
        rec,dep,trip=overlays(rgb,depth,instance,semantic,self.objects,visible,timestamp)
        if output is not None:
            output=Path(output);output.mkdir(parents=True,exist_ok=False)
            from PIL import Image
            Image.fromarray(rgb).save(output/'rgb.png');rec.save(output/'recognition.png');trip.save(output/'camera.png')
            np.savez_compressed(output/'rgbd_gt.npz',depth_m=depth,instance=instance,semantic=semantic)
            (output/'observation.json').write_text(json.dumps(observation,indent=2,allow_nan=False)+'\n')
        images=dict(rgb=rgb,recognition=np.asarray(rec),depth_overlay=np.asarray(dep))
        if include_raw:images.update(depth_m=depth,instance=instance,semantic=semantic)
        return observation,images
