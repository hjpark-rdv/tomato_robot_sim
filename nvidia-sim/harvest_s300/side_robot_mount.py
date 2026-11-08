"""Bolt RB5 to the existing vertical moving carriage in a new assembly.

The lift frame/axis remain upright. The arm, link0 geometry and base joint
parent frame rotate together. No old command trace is reused after this change.
The retained CAD carriage is a physical obstacle, not a display-only plate.
"""
import argparse
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from real_lift_mount import ROBOT, PLATFORM, matrix, sha, resolve_carrier_archive
from compact_motion_assets import verify_compact_dependencies

CARRIAGE = PLATFORM+'/visuals/real_lift_carriage/back_carriage'
BASE_VISUAL = PLATFORM+'/visuals/link0'
BASE_COLLIDER = PLATFORM+'/collisions/link0'
REMOVED = (PLATFORM+'/visuals/real_lift_carriage/mounting_plate',
           PLATFORM+'/visuals/real_lift_carriage/cantilever_bracket',
           PLATFORM+'/collisions/real_mounting_plate')
SIDE_READY_Q = np.array([.1772361584508187,np.pi/2,-.8433796855771658,
                        2.507443674795259,-.07737928186451346,
                        3.14079650682554,.015888372798979956])


def _world_mesh(stage, path):
    mesh=UsdGeom.Mesh(stage.GetPrimAtPath(path))
    if not mesh or not np.all(np.asarray(mesh.GetFaceVertexCountsAttr().Get())==3):
        raise ValueError('Expected exact triangulated CAD '+path)
    points=np.asarray(mesh.GetPointsAttr().Get(),float);tf=matrix(stage,path)
    points=points@tf[:3,:3].T+tf[:3,3]
    faces=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),int).reshape(-1,3)
    return trimesh.Trimesh(points,faces,process=True)


def _set_world(stage,path,world):
    prim=stage.GetPrimAtPath(path)
    local=np.linalg.inv(matrix(stage,str(prim.GetParent().GetPath())))@world
    UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(local.T.tolist()))


def mounting_plane(stage):
    """Select the largest -Y planar CAD surface, not a bolt head/bounding box."""
    mesh=_world_mesh(stage,CARRIAGE)
    triangles=mesh.triangles
    normals=mesh.face_normals;areas=mesh.area_faces
    ids=np.flatnonzero(normals[:,1]<-.999)
    if not len(ids):raise ValueError('No front-facing carriage plane')
    normal=np.average(normals[ids],weights=areas[ids],axis=0);normal/=np.linalg.norm(normal)
    distances=np.round(triangles[ids].mean(1)@normal,5)
    bins=np.unique(distances)
    best=bins[np.argmax([areas[ids[distances==b]].sum() for b in bins])]
    faces=ids[distances==best];points=triangles[faces].reshape(-1,3)
    coef=np.linalg.lstsq(np.c_[points[:,[0,2]],np.ones(len(points))],points[:,1],rcond=None)[0]
    error=float(np.max(np.abs(np.c_[points[:,[0,2]],np.ones(len(points))]@coef-points[:,1])))
    if error>1e-6 or areas[faces].sum()<.05:
        raise ValueError('Carriage front plane was not identified reliably')
    normal=np.array([coef[0],-1.,coef[1]]);normal/=np.linalg.norm(normal)
    return mesh,coef,normal,error


def _platform_mass(stage, retained, old_parts, new_base, platform_pose):
    """Retain the source 3.5kg base estimate; remove proportional CAD mass.

    The source's 15kg carriage estimate is apportioned by actual closed CAD
    volume. Uniform density and the source base inertia remain assumptions.
    """
    if not all(mesh.is_watertight and mesh.volume>0 for mesh in old_parts):
        raise ValueError('Closed positive-volume original carriage CAD required')
    source_urdf=Path(__file__).resolve().parents[1]/'robot_usd/rb5_farmily.urdf'
    source=ET.parse(source_urdf).getroot()
    base=source.find("link[@name='link0']/inertial")
    carriage=source.find("link[@name='farmily_lift_platform']/inertial")
    base_mass=float(base.find('mass').attrib['value'])
    old_carriage_mass=float(carriage.find('mass').attrib['value'])
    if not np.isclose(float(UsdPhysics.MassAPI(stage.GetPrimAtPath(PLATFORM)).GetMassAttr().Get()),base_mass+old_carriage_mass,atol=1e-5):
        raise ValueError('Original merged platform mass no longer matches declared URDF estimates')
    carriage_mass=old_carriage_mass*retained.volume/sum(mesh.volume for mesh in old_parts)
    com_base=np.r_[np.fromstring(base.find('origin').attrib['xyz'],sep=' '),1.]
    com_base=(new_base@com_base)[:3]
    principal=base.find('inertia').attrib
    base_inertia=np.array([[float(principal['ixx']),float(principal['ixy']),float(principal['ixz'])],
                          [float(principal['ixy']),float(principal['iyy']),float(principal['iyz'])],
                          [float(principal['ixz']),float(principal['iyz']),float(principal['izz'])]])
    rpy=np.fromstring(base.find('origin').attrib.get('rpy','0 0 0'),sep=' ')
    rot=new_base[:3,:3]@Rotation.from_euler('xyz',rpy).as_matrix()
    base_inertia=rot@base_inertia@rot.T
    total=base_mass+carriage_mass
    center=(base_mass*com_base+carriage_mass*retained.center_mass)/total
    def pa(m,c):
        d=c-center;return m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    inertia=base_inertia+pa(base_mass,com_base)+retained.moment_inertia*(carriage_mass/retained.volume)+pa(carriage_mass,retained.center_mass)
    rotation=platform_pose[:3,:3]
    local_center=rotation.T@(center-platform_pose[:3,3])
    eigen,axes=np.linalg.eigh(rotation.T@inertia@rotation)
    if np.linalg.det(axes)<0:axes[:,0]*=-1
    if np.any(eigen<=0):raise ValueError('Invalid derived platform inertia')
    quat=Rotation.from_matrix(axes).as_quat()
    mass=UsdPhysics.MassAPI(stage.GetPrimAtPath(PLATFORM))
    mass.GetMassAttr().Set(total);mass.GetCenterOfMassAttr().Set(Gf.Vec3f(*map(float,local_center)))
    mass.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*map(float,eigen)))
    mass.GetPrincipalAxesAttr().Set(Gf.Quatf(float(quat[3]),Gf.Vec3f(*map(float,quat[:3]))))
    return dict(mass_kg=total,carriage_mass_kg=carriage_mass,base_mass_kg=base_mass,
        removed_mass_kg=base_mass+old_carriage_mass-total,center_of_mass_local_m=local_center.tolist(),
        diagonal_inertia_kg_m2=eigen.tolist(),principal_axes_xyzw=quat.tolist(),
        model='Source 15kg carriage estimate allocated by closed original CAD volume; retained 3.5kg RB5 link0 URDF estimate and inertia; removed bracket/plate mass excluded',
        source_urdf=str(source_urdf),source_urdf_sha256=sha(source_urdf),hardware_mass_calibrated=False)


def apply(stage, *, mount_height_from_plate_top_m=.09, camera_metadata=None):
    if stage.GetPrimAtPath(PLATFORM+'/collisions/side_carriage'):
        raise ValueError('Side mount already present')
    for path in (CARRIAGE,*REMOVED,BASE_VISUAL,BASE_COLLIDER):
        if not stage.GetPrimAtPath(path):raise ValueError('Expected corrected real-lift assembly '+path)
    retained,coef,normal,plane_error=mounting_plane(stage)
    old_parts=[_world_mesh(stage,p) for p in (CARRIAGE,*REMOVED[:2])]
    old_base=matrix(stage,PLATFORM).copy();old_base[2,3]+=.0125
    base_mesh=_world_mesh(stage,BASE_VISUAL+'/Scene/mesh')
    local_base=(base_mesh.vertices-old_base[:3,3])@old_base[:3,:3]
    min_base=float(local_base[:,2].min())
    # Mount high on the retained plate, leaving its measured 86.4mm flange
    # radius inside the top/side edges. Hole pattern/bolt strength uncalibrated.
    z=float(retained.bounds[1,2]-mount_height_from_plate_top_m)
    x=float(np.mean(retained.bounds[:,0]));y=float(coef@[x,z,1.])
    seat=np.array([x,y,z]);up=np.array([0.,0.,1.])
    axis_x=np.cross(up,normal);axis_x/=np.linalg.norm(axis_x)
    axis_y=np.cross(normal,axis_x)
    new_base=np.eye(4);new_base[:3,:3]=np.column_stack((axis_x,axis_y,normal))
    new_base[:3,3]=seat-normal*min_base
    delta=new_base@np.linalg.inv(old_base)
    old={str(p.GetPath()):matrix(stage,str(p.GetPath())) for root in (ROBOT,'/World/ActuatedTool')
         for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    moved=[path for path in old if path.startswith(ROBOT+'/link') or path.startswith('/World/ActuatedTool/')]
    for path in moved:_set_world(stage,path,delta@old[path])
    for path in (BASE_VISUAL,BASE_COLLIDER):_set_world(stage,path,delta@matrix(stage,path))
    # Only the base joint straddles moved arm and unmoved vertical lift frame.
    joint=UsdPhysics.Joint(stage.GetPrimAtPath(ROBOT+'/joints/base'))
    q=joint.GetLocalRot0Attr().Get();frame=np.eye(4)
    frame[:3,:3]=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix()
    frame[:3,3]=joint.GetLocalPos0Attr().Get()
    local=np.linalg.inv(old[PLATFORM])@delta@old[PLATFORM]@frame
    q=Rotation.from_matrix(local[:3,:3]).as_quat()
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*map(float,local[:3,3])))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
    for path in REMOVED:stage.GetPrimAtPath(path).SetActive(False)
    # The retained mounting plate is a real rigid obstacle for planner/physics.
    from pxr import Sdf
    collider_path=PLATFORM+'/collisions/side_carriage'
    Sdf.CopySpec(stage.Flatten(),CARRIAGE,stage.GetEditTarget().GetLayer(),collider_path)
    source_world=matrix(stage,CARRIAGE);_set_world(stage,collider_path,source_world)
    collider=stage.GetPrimAtPath(collider_path)
    UsdGeom.Imageable(collider).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(collider).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(collider).CreateApproximationAttr('convexHull')
    mass=_platform_mass(stage,retained,old_parts,new_base,old[PLATFORM])
    from mounted_rgbd import ROOT,mount,recalibrate_mount
    camera=np.eye(4);camera[:3,:3]=np.array([[0,-1,0],[0,0,1],[-1,0,0]])
    camera[:3,3]=[.0325,.098,0.]
    if stage.GetPrimAtPath(ROOT):
        if camera_metadata is None:raise ValueError('Existing sensor calibration metadata required')
        mounted=recalibrate_mount(stage,camera_metadata,camera)
    else:
        mounted=mount(stage,body_from_camera_optical=camera)
    mounted['mount_description']='link1 outward +Y cover; camera housing rolled 90deg on side-mounted base so long housing axis is world vertical at base=+90deg; all fixed to actual measured link1'
    report=dict(schema='s300-vertical-carriage-side-mount-v1',removed_paths=list(REMOVED),
        retained_carriage=CARRIAGE,physical_carriage_collider=collider_path,
        old_base_world=old_base.tolist(),new_base_world=new_base.tolist(),arm_delta_world=delta.tolist(),
        mount_contact_world_m=seat.tolist(),mount_normal_world=normal.tolist(),
        front_plane_y_ax_bz_c=coef.tolist(),plane_fit_max_error_m=plane_error,
        mount_height_from_plate_top_m=mount_height_from_plate_top_m,
        base_surface_min_local_z_m=min_base,moving_rigid_bodies=moved,mass_properties=mass,
        lift_joint_unchanged=True,base_joint_parent_frame_changed=True,
        arm_and_tool_local_inertias_and_drives_preserved=True,
        camera=mounted,bolt_pattern_and_strength_validated=False,
        old_harvest_plans_revalidated=False,training_eligible=False)
    return report


def author_ready_state(stage, manifest, q=SIDE_READY_Q):
    """New start only; prove FK/open/closed clearance before authoring state."""
    from motion_kinematics import ToolKinematics
    from motion_self_collision import MotionSelfCollision
    from motion_assets import attr
    from pxr import Sdf
    kin=ToolKinematics(stage);q=np.asarray(q,float)
    if q.shape!=(7,) or not np.isfinite(q).all() or np.any(q<kin.bounds[0]) or np.any(q>kin.bounds[1]):
        raise ValueError('Finite in-range seven-joint side-ready state required')
    checker=MotionSelfCollision(stage,kin,clearance_m=.002)
    for gap in (.044,.096):
        hits=checker.check(q,gap)
        if hits:raise ValueError('Side-ready state has self collision '+str(hits))
    for name,value in zip(kin.names,q):
        prim=stage.GetPrimAtPath(ROBOT+'/joints/'+name)
        slider=prim.IsA(UsdPhysics.PrismaticJoint);kind='linear' if slider else 'angular'
        units=float(value if slider else np.rad2deg(value))
        prim.AddAppliedSchema('PhysicsJointStateAPI:'+kind)
        attr(prim,'state:'+kind+':physics:position',Sdf.ValueTypeNames.Float,units)
        attr(prim,'state:'+kind+':physics:velocity',Sdf.ValueTypeNames.Float,0.)
        UsdPhysics.DriveAPI(prim,kind).CreateTargetPositionAttr(units)
    previous=dict(manifest.get('ready_revision',{}))
    manifest['initial_robot_q']=q.tolist();manifest['initial_robot_qvel']=[0.]*7
    manifest['ready_revision']=dict(previous,source_shelf_ready_revision=previous,
        ready_pose=kin.fk(q).tolist(),initialization='New side-mount JointStateAPI before native startup; no runtime teleport',
        side_mount=True,side_mount_fcl_open_closed_clearance_m=.002,
        native_gravity_tracking_verified=False)
    return dict(q=q.tolist(),tool_world=kin.fk(q).tolist(),self_clearance_m=.002,
                gaps_checked_m=[.044,.096],native_gravity_tracking_verified=False)


def rebase_initial_station(stage,manifest,station_y_m):
    """Author a fresh initial scenario at the rail entrance; never a resume."""
    if manifest.get('route_checkpoint') or manifest.get('resume_native_checkpoint'):
        raise ValueError('Initial scenario rebase must not change a continued actual state')
    rail=manifest['rail'];old=float(rail['initial_station_y_m']);station=float(station_y_m)
    limits=np.asarray(rail['limits_world_y_m'],float)
    if not np.isfinite(station) or not limits[0]<=station<=limits[1]:
        raise ValueError('New initial station exceeds existing world rail limits')
    delta=np.eye(4);delta[1,3]=station-old
    roots=(ROBOT,'/World/ActuatedTool')
    rigid={str(p.GetPath()):matrix(stage,str(p.GetPath())) for root in roots
           for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    for root in roots:_set_world(stage,root,delta@matrix(stage,root))
    # Also account for resetXformStack actors without trusting hierarchy alone.
    for path,pose in rigid.items():
        expected=delta@pose
        if not np.allclose(matrix(stage,path),expected,atol=1e-12,rtol=0):_set_world(stage,path,expected)
    anchors=[]
    for root in roots:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if not prim.IsA(UsdPhysics.Joint):continue
            joint=UsdPhysics.Joint(prim)
            for index,targets in enumerate((joint.GetBody0Rel().GetTargets(),joint.GetBody1Rel().GetTargets())):
                if targets:continue
                attr=getattr(joint,f'GetLocalPos{index}Attr')();p=np.asarray(attr.Get(),float)
                attr.Set(Gf.Vec3f(*map(float,p+delta[:3,3])))
                anchors.append(dict(joint=str(prim.GetPath()),slot=index))
    rail['initial_station_y_m']=station;rail['limits_relative_m']=(limits-station).tolist()
    rail['initial_displacement_m']=0.
    rail['source_base_position_world_m']=matrix(stage,rail['base_body'])[:3,3].tolist()
    rail['source_robot_root_position_world_m']=(np.asarray(rail['source_robot_root_position_world_m'],float)+delta[:3,3]).tolist()
    joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(rail['joint']))
    joint.GetLowerLimitAttr().Set(float(limits[0]-station));joint.GetUpperLimitAttr().Set(float(limits[1]-station))
    if manifest.get('ready_revision',{}).get('ready_pose'):
        manifest['ready_revision']['ready_pose']=(delta@np.asarray(manifest['ready_revision']['ready_pose'])).tolist()
    result=dict(schema='s300-fresh-initial-station-rebase-v1',old_station_y_m=old,new_station_y_m=station,
        world_translation_m=delta[:3,3].tolist(),world_anchor_joint_frames_changed=anchors,
        physical_rail_limits_world_y_m=limits.tolist(),fresh_initial_scenario=True,
        runtime_teleport=False,old_execution_reused=False,farm_geometry_changed=False)
    manifest['initial_station_rebase']=result
    return result


def prepare(assembly,output,*,mount_height_from_plate_top_m=.09,initial_station_y_m=None):
    assembly=Path(assembly);output=Path(output)
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    if sha(assembly/'scene.usda')!=manifest['scene_sha256']:raise ValueError('Source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    source=assembly/'composed.usdc';before=sha(source)
    output.mkdir(parents=True,exist_ok=False)
    stage=Usd.Stage.CreateNew(str(output/'scene.usda'));stage.GetRootLayer().subLayerPaths=[str(source.resolve())]
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    report=apply(stage,mount_height_from_plate_top_m=mount_height_from_plate_top_m,
                 camera_metadata=manifest.get('mounted_rgbd'))
    report['ready_state']=author_ready_state(stage,manifest)
    if initial_station_y_m is not None:
        report['initial_station_rebase']=rebase_initial_station(stage,manifest,initial_station_y_m)
    stage.Flatten().Export(str(output/'composed.usdc'))
    stage.GetRootLayer().Clear();stage.GetRootLayer().subLayerPaths=['composed.usdc']
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'));stage.GetRootLayer().Save()
    resolve_carrier_archive(manifest)
    manifest['side_robot_mount']=report;manifest['mounted_rgbd']=report['camera']
    manifest['scene_sha256']=sha(output/'scene.usda')
    manifest['compact_assets']['materialized_dependency']['sha256']=sha(output/'composed.usdc')
    manifest['compact_assets']['geometry_unchanged']=False
    manifest['compact_assets']['native_validation']='Side-mounted robot requires new plan/FK/native gravity validation; old shelf traces are inapplicable'
    manifest['side_mount_source']=dict(assembly=str(assembly),composed_sha256=before)
    (output/'motion_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'side_robot_mount.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'mounted_rgbd.json').write_text(json.dumps(report['camera'],indent=2)+'\n')
    for pattern in ('skin*.npz','real_lift_mount.json'):
        for path in assembly.glob(pattern):shutil.copy2(path,output/path.name)
    if sha(source)!=before:raise RuntimeError('Source changed')
    print(json.dumps({k:report[k] for k in ('mount_contact_world_m','mount_normal_world','mass_properties')},indent=2))
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('assembly',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--mount-height-from-plate-top-m',type=float,default=.09)
    p.add_argument('--initial-station-y-m',type=float)
    a=p.parse_args();prepare(a.assembly,a.output,mount_height_from_plate_top_m=a.mount_height_from_plate_top_m,
                            initial_station_y_m=a.initial_station_y_m)
