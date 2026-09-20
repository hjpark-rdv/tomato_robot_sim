"""Pre-action full/context RGB-D from a rigidly tool-mounted camera."""
import json
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from geometry import RING_CENTER
from rgb_camera import camera_profiles, optical_look_at, world_from_camera, intrinsics
from dataset_design import crop_rgbd


def plant_obstacles(env):
    import fcl
    import trimesh
    from pxr import Usd,UsdGeom,UsdPhysics
    cache=UsdGeom.XformCache(); result=[]
    # Actual enabled native capsule/sphere proxies, including all 11 fruits.
    for prim in Usd.PrimRange(env.stage.GetPrimAtPath(env.root)):
        path = str(prim.GetPath())
        if not any(k in path for k in ('/ElasticPlant/', '/HarvestableStem/')) or not prim.HasAPI(UsdPhysics.CollisionAPI): continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False: continue
        if not (prim.IsA(UsdGeom.Capsule) or prim.IsA(UsdGeom.Sphere) or prim.IsA(UsdGeom.Mesh)): continue
        tf=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        if prim.IsA(UsdGeom.Mesh):
            vertices=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
            hull=trimesh.convex.convex_hull(vertices@tf[:3,:3].T+tf[:3,3])
            geometry=fcl.Convex(hull.vertices,len(hull.faces),np.column_stack((np.full(len(hull.faces),3),hull.faces)).reshape(-1))
            result.append((fcl.CollisionObject(geometry),hull.bounds[0],hull.bounds[1],path))
            continue
        scale=np.linalg.norm(tf[:3,:3],axis=0)
        if not np.allclose(scale,scale[0]): raise RuntimeError('Non-uniform plant collider scale')
        rotation=tf[:3,:3]/scale
        if prim.IsA(UsdGeom.Capsule):
            shape=UsdGeom.Capsule(prim);h=float(shape.GetHeightAttr().Get())*scale[0]
            axis=str(shape.GetAxisAttr().Get())
            align=Rotation.from_euler('y',90,degrees=True).as_matrix() if axis=='X' else Rotation.from_euler('x',-90,degrees=True).as_matrix() if axis=='Y' else np.eye(3)
            rotation=rotation@align;r=float(shape.GetRadiusAttr().Get())*scale[0]
            geometry=fcl.Capsule(r,h);extent=abs(rotation[:,2])*h/2+r
        else:
            r=float(UsdGeom.Sphere(prim).GetRadiusAttr().Get())*scale[0]
            geometry=fcl.Sphere(r);extent=np.full(3,r)
        result.append((fcl.CollisionObject(geometry,fcl.Transform(rotation,tf[:3,3])),tf[:3,3]-extent,tf[:3,3]+extent,str(prim.GetPath())))
    return result


def plant_collision(checker,obstacles,q):
    transforms=checker.transforms(q)
    for shape in checker.shapes:
        tf=transforms[shape['link']]
        corners=shape['corners']@tf[:3,:3].T+tf[:3,3]
        low,high=corners.min(0),corners.max(0)
        candidates=[o for o in obstacles if (low<=o[2]).all() and (o[1]<=high).all()]
        if not candidates: continue
        tf=tf@shape['local']; shape['object'].setTransform(checker.fcl.Transform(tf[:3,:3],tf[:3,3]))
        for obj,_,_,path in candidates:
            if checker.fcl.collide(shape['object'],obj,checker.fcl.CollisionRequest(),checker.fcl.CollisionResult()):
                return [shape['path'],path]
    return None


def choose_observation(env,checker,config):
    """Reachable mounted-camera pose; no camera teleport independent of robot.

    Joint-path screening includes robot self collision and plant proxies. Real
    deployment still needs a planner for the entire actual workplace.
    """
    from omni.physx import get_physx_interface
    env.sim.physics_sim_view.update_articulations_kinematic()
    get_physx_interface().update_transformations(False,True,True)
    profile=camera_profiles()[config['camera_profile']]
    obstacles=plant_obstacles(env)
    kin=env.pose_search_kin; center=env._target_geometry()[0][0].cpu().numpy()
    ready=env.robot.data.joint_pos[0].cpu().numpy().copy()
    mount=profile['tool_from_camera']; attempts=[]
    for yaw in (-45.,-20.,0.,20.,45.,-70.,70.,110.,-110.,180.):
        for dz in (.08,.16,0.):
            a=np.deg2rad(yaw);eye=center+[config['camera_distance']*np.cos(a),config['camera_distance']*np.sin(a),dz]
            camera=optical_look_at(eye,center,up=(0.,0.,1.))
            tool=camera@np.linalg.inv(mount)
            rotation=Rotation.from_matrix(tool[:3,:3]);ring=tool[:3,3]+rotation.apply(RING_CENTER)
            q,pe,re=kin.ik(ring,rotation,ready)
            reason=None
            if pe>.002 or re>.03: reason='ik'
            else:
                n=max(2,int(np.ceil(np.max(abs(q-ready))/.015)))
                for u in np.linspace(0,1,n+1):
                    pose=ready+(q-ready)*u
                    hit=checker.check(pose) or plant_collision(checker,obstacles,pose)
                    if hit: reason=dict(collision=hit);break
            attempts.append(dict(yaw_deg=yaw,height_offset_m=dz,rejection=reason))
            if reason is None:
                return q,dict(profile=config['camera_profile'],tool_from_camera_optical=mount.tolist(),
                    world_from_camera_optical=world_from_camera(kin,q,profile).tolist(),
                    joint_positions= q.tolist(),ready_joint_positions=ready.tolist(),
                    camera_mount_status='existing_URDF' if config['camera_profile']=='current' else 'proposed_mount_not_hardware_validated',
                    ready_to_observation_path='joint interpolation screened against self collision and plant proxies; no house/real workspace certificate',
                    pose_application='simulation reset to screened observation pose; no real robot command sent',
                    selection='target-centred camera optical axis; no pedicel visibility requirement',attempts=attempts)
    raise RuntimeError('No collision-free mounted-camera observation pose. Inspect camera mount/distance; attempts='+json.dumps(attempts))


def capture_observation(env,directory,config,selection):
    import omni.replicator.core as rep
    from pxr import UsdGeom,Gf
    from omni.physx import get_physx_interface
    from PIL import Image
    directory.mkdir(parents=True,exist_ok=True)
    profile=camera_profiles()[config['camera_profile']];w,h=960,720
    pose=world_from_camera(env.pose_search_kin,env.robot.data.joint_pos[0].cpu().numpy(),profile)
    K=intrinsics(profile,w,h)
    camera=UsdGeom.Camera.Define(env.stage,'/World/DatasetObservationCamera')
    camera.CreateHorizontalApertureAttr(20.955);camera.CreateVerticalApertureAttr(20.955*h/w)
    camera.CreateFocalLengthAttr(K[0,0]*20.955/w)
    camera.CreateClippingRangeAttr(Gf.Vec2f(profile['near_m'],config['depth_max']+1.))
    usd=pose.copy();usd[:3,1:3]*=-1
    UsdGeom.Xformable(camera).MakeMatrixXform().Set(Gf.Matrix4d(*usd.T.flatten().tolist()))
    product=rep.create.render_product(str(camera.GetPath()),(w,h))
    rgb=rep.AnnotatorRegistry.get_annotator('rgb'); depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
    rgb.attach(product);depth.attach(product)
    env.elastic.sync_visuals();env.sim.physics_sim_view.update_articulations_kinematic()
    get_physx_interface().update_transformations(False,True,True)
    try:
        for _ in range(12): env.sim.render()
        pixels=rgb.get_data()[:,:,:3].copy();z=depth.get_data().copy().squeeze().astype(np.float32)
    finally:
        rgb.detach(product);depth.detach(product);product.destroy()
    if pixels.shape!=(h,w,3) or z.shape!=(h,w): raise RuntimeError('Invalid RGB-D frame shape')
    if float(pixels.std()) < 1.:
        raise RuntimeError('RGB frame is nearly uniform; inspect camera housing/clip range/rendering. This is not a pedicel visibility test.')
    # Store invalid/missing measurements as NaN, never synthesize hidden depth.
    z[~np.isfinite(z)|(z<config['depth_min'])|(z>config['depth_max'])]=np.nan
    center=env._target_geometry()[0][0].cpu().numpy()
    local_rgb,local_depth,valid,crop=crop_rgbd(pixels,z,K,pose,center,config['crop_extent'])
    x0,y0,x1,y1=crop['bounds_xyxy']
    Image.fromarray(pixels).save(directory/'rgb.png');np.save(directory/'depth.npy',z)
    Image.fromarray(valid.astype(np.uint8)*255).save(directory/'depth_valid.png')
    Image.fromarray(local_rgb).save(directory/'local_rgb.png');np.save(directory/'local_depth.npy',local_depth)
    Image.fromarray(valid[y0:y1,x0:x1].astype(np.uint8)*255).save(directory/'local_depth_valid.png')
    metadata=dict(observation_id='observation_0001',target_id=env.target_spec['name'],
        rgb='rgb.png',depth='depth.npy',depth_valid='depth_valid.png',local_rgb='local_rgb.png',
        local_depth='local_depth.npy',local_depth_valid='local_depth_valid.png',
        width=w,height=h,depth_units='metres',depth_dtype='float32',invalid_depth='NaN',
        depth_definition='optical +Z distance_to_image_plane, not Euclidean range',
        depth_range_m=[config['depth_min'],config['depth_max']],sensor_noise_model='ideal rendered depth plus range mask; not calibrated real sensor',
        rgb_clip_range_m=[profile['near_m'],config['depth_max']+1.],
        K=K.tolist(),world_from_camera_optical=pose.tolist(),crop=crop,
        depth_valid_fraction=float(valid.mean()),target_center_world_gt=center.tolist(),
        target_crop_source='simulator GT centre; no segmentation and no hidden-surface reconstruction',
        joint_names=env.robot.joint_names,joint_positions_at_capture=env.robot.data.joint_pos[0].tolist(),
        selection=selection,visibility_requirement='target centre in frame; pedicel/ring may be occluded',
        warning='Low or zero valid local depth is retained and flagged, not replaced with GT geometry')
    (directory/'observation.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return metadata
