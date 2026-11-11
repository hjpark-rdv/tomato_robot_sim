"""Real lift top mount and fixed 90-degree wrist adapter (asset Python).

Fresh zero-pose USD geometry; no playback or runtime body teleport. The adapter
is attached to Tool_fixed, retaining the existing six arm axes and jaw sliders.
Dimensions/density are explicit prototype assumptions, not a fabrication design.
"""
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

from real_lift_mount import ROBOT, PLATFORM, matrix, sha
from side_robot_mount import _set_world, _world_mesh, rebase_initial_station
from motion_assets import joint

FIXED = '/World/ActuatedTool/fixed'
# Tool +Y projects horizontally; tool +Z coincides with the last wrist axis.
# Original URDF flange is link6 (0,-96.7,0) mm. The gripper origin is
# 150 mm forward and 120 mm below that flange, in the new gripper frame.
FLANGE = np.array([0., -.0967, 0.])
FLANGE_IN_TOOL = np.array([0., -.150, .120])
PARTS = (
    ('flange_plate', [0., -.150, .114], [.090, .100, .012]),
    ('rear_plate', [0., -.200, .034], [.170, .012, .148]),
)


def adapter_geometry(inset_m=0.):
    if not np.isfinite(inset_m) or not 0 <= inset_m <= .15:
        raise ValueError('Gripper inset must be finite and within 0..150mm')
    flange=FLANGE_IN_TOOL.copy();flange[1]+=inset_m
    # Keep the rear attachment, extending its upper leg beneath the wrist.
    parts=(('flange_plate',[0.,-.150+inset_m/2,.114],[.090,.100+inset_m,.012]),PARTS[1])
    return flange,parts


def adapter_transform(inset_m=0.):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('x', -90, degrees=True).as_matrix()
    t[:3, 3] = FLANGE - t[:3, :3] @ adapter_geometry(inset_m)[0]
    return t


def apply_adapter(stage, inset_m=0.):
    """Reattach all existing tool bodies as a rigid assembly, update the weld."""
    if stage.GetPrimAtPath(FIXED+'/YawAdapter'):
        raise ValueError('Wrist adapter already authored')
    old = matrix(stage, FIXED)
    flange_body = matrix(stage, ROBOT+'/link6')
    flange_in_tool,parts=adapter_geometry(inset_m)
    new = flange_body @ adapter_transform(inset_m)
    delta = new @ np.linalg.inv(old)
    bodies = {str(p.GetPath()): matrix(stage, str(p.GetPath()))
              for p in Usd.PrimRange(stage.GetPrimAtPath('/World/ActuatedTool'))
              if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    for path, pose in bodies.items():
        _set_world(stage, path, delta @ pose)
    joint(stage, '/World/ActuatedTool/Joints/jaw_fixed', ROBOT+'/link6', FIXED,
          flange_body, new, (flange_body @ np.r_[FLANGE, 1])[:3], 'fixed',
          rotation=flange_body[:3, :3])
    from build_scene import box
    masses = []
    for name, center, dimensions in parts:
        part = box(stage, FIXED+'/YawAdapter/'+name, center, dimensions, (.28,.34,.39))
        UsdPhysics.CollisionAPI.Apply(part.GetPrim()).CreateCollisionEnabledAttr(True)
        masses.append(float(np.prod(dimensions)*2700.))
    mass = UsdPhysics.MassAPI(stage.GetPrimAtPath(FIXED))
    previous = float(mass.GetMassAttr().Get())
    mass.GetMassAttr().Set(previous+sum(masses))
    # Tool inertia was automatic in the source. The existing native exporter
    # recomputes its declared conservative CAD-bounds estimate including adapter.
    return dict(schema='s300-upright-wrist-yaw-v1', link6_from_tool=adapter_transform(inset_m).tolist(),
        flange_in_link6_m=FLANGE.tolist(), flange_in_tool_m=flange_in_tool.tolist(),
        gripper_inset_m=inset_m,
        parts=[dict(name=n,center_tool_m=p,dimensions_m=s,mass_kg=m)
               for (n,p,s),m in zip(parts,masses)], adapter_mass_kg=sum(masses),
        previous_fixed_tool_mass_kg=previous, fixed_tool_mass_kg=previous+sum(masses),
        density_kg_m3=2700., fixed_attachment=True, extra_actuated_axes=0,
        gripper_entry_axis='Tool_fixed +Y', wrist3_axis='Tool_fixed +Z',
        dimensions_are_assumed=True, strength_and_bolt_pattern_validated=False,
        tool_inertia='Existing native CAD-bounds estimate, recomputed with adapter',
        old_link6_from_tool=(np.linalg.inv(flange_body)@old).tolist(),
        training_eligible=False)


def preserve_trimmed_supports(stage, previous_assets):
    """Carry the reviewed diagonal removal into USD, including its normal data."""
    import trimesh
    previous_assets=Path(previous_assets)
    meta=json.loads((previous_assets/'model/export.json').read_text())
    tree=ET.parse(previous_assets/'model/scene.xml')
    rows=[r for r in meta['geometry'] if r.get('visual_revision')=='diagonal removed; upright/base retained']
    if len(rows)!=2: raise ValueError('Two previously trimmed lift supports required')
    for row in rows:
        geom=tree.find(".//geom[@name='%s']"%row['name'])
        asset=tree.find("./asset/mesh[@name='%s']"%geom.get('mesh'))
        path=previous_assets/'model'/tree.find('compiler').get('meshdir')/asset.get('file')
        if sha(path)!=row['mesh_sha256']: raise ValueError('Trimmed support SHA mismatch')
        mesh=trimesh.load(path,force='mesh',process=False)
        # robot_adapter writes mesh vertices in rigid owner coordinates.
        local=np.linalg.inv(matrix(stage,row['source']))@matrix(stage,ROBOT+'/world')
        vertices=mesh.vertices@local[:3,:3].T+local[:3,3]
        dest=UsdGeom.Mesh(stage.GetPrimAtPath(row['source']))
        dest.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(vertices.astype(np.float32)))
        dest.GetFaceVertexCountsAttr().Set(Vt.IntArray([3]*len(mesh.faces)))
        dest.GetFaceVertexIndicesAttr().Set(Vt.IntArray(mesh.faces.ravel().tolist()))
        normals=mesh.face_normals@local[:3,:3].T
        dest.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(normals.astype(np.float32)))
        dest.SetNormalsInterpolation('uniform')


def author_ready(stage, manifest, q):
    from motion_assets import attr
    from motion_kinematics import ToolKinematics
    kin=ToolKinematics(stage)
    q=np.asarray(q,float)
    if q.shape!=(7,) or not np.all((q>=kin.bounds[0])&(q<=kin.bounds[1])):
        raise ValueError('Invalid new ready state')
    for name,value in zip(kin.names,q):
        prim=stage.GetPrimAtPath(ROBOT+'/joints/'+name)
        kind='linear' if prim.IsA(UsdPhysics.PrismaticJoint) else 'angular'
        units=float(value if kind=='linear' else np.rad2deg(value))
        prim.AddAppliedSchema('PhysicsJointStateAPI:'+kind)
        attr(prim,'state:'+kind+':physics:position',Sdf.ValueTypeNames.Float,units)
        attr(prim,'state:'+kind+':physics:velocity',Sdf.ValueTypeNames.Float,0.)
        UsdPhysics.DriveAPI(prim,kind).CreateTargetPositionAttr(units)
    manifest.update(initial_robot_q=q.tolist(),initial_robot_qvel=[0.]*7,
        ready_revision=dict(ready_pose=kin.fk(q).tolist(),
            initialization='Fresh upright + 90-degree wrist adapter zero-time state; no old trace reused'))


def lower_platform(stage, drop_m):
    """Move the mounting surface to the carriage foot, without moving the rail.

    Replace the short original perforated plate with an explicit flat plate
    extending to the retained back carriage. No diagonal or invisible shelf
    remains. The arm base frame and platform inertia change with the geometry.
    """
    import trimesh
    from side_robot_mount import CARRIAGE, BASE_VISUAL, BASE_COLLIDER, _platform_mass
    if not np.isfinite(drop_m) or not 0 < drop_m <= .31:
        raise ValueError('Platform drop must be finite, positive and <=310mm')
    plate=PLATFORM+'/visuals/real_lift_carriage/mounting_plate'
    triangle=PLATFORM+'/visuals/real_lift_carriage/cantilever_bracket'
    collider=PLATFORM+'/collisions/real_mounting_plate'
    original=[_world_mesh(stage,p) for p in (CARRIAGE,plate,triangle)]
    base=_world_mesh(stage,BASE_VISUAL+'/Scene/mesh')
    lo,hi=original[1].bounds.copy()
    hi[1]=original[0].bounds[0,1]  # meet the vertical carriage, close old 50mm gap
    hi[2]=base.bounds[0,2]-drop_m;lo[2]=hi[2]-.010
    mesh=trimesh.creation.box(extents=hi-lo,
        transform=trimesh.transformations.translation_matrix((lo+hi)/2))
    inv=np.linalg.inv(matrix(stage,plate))
    vertices=mesh.vertices@inv[:3,:3].T+inv[:3,3]
    for path in (plate,collider):
        dest=UsdGeom.Mesh(stage.GetPrimAtPath(path))
        _set_world(stage,path,matrix(stage,plate))
        dest.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(vertices.astype(np.float32)))
        dest.GetFaceVertexCountsAttr().Set(Vt.IntArray([3]*len(mesh.faces)))
        dest.GetFaceVertexIndicesAttr().Set(Vt.IntArray(mesh.faces.ravel().tolist()))
        dest.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy((mesh.face_normals@inv[:3,:3].T).astype(np.float32)))
        dest.SetNormalsInterpolation('uniform')
    stage.GetPrimAtPath(triangle).SetActive(False)
    delta=np.eye(4);delta[2,3]=-drop_m
    poses={str(p.GetPath()):matrix(stage,str(p.GetPath()))
        for root in (ROBOT,'/World/ActuatedTool')
        for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    moved=[p for p in poses if p.startswith(ROBOT+'/link') or p.startswith('/World/ActuatedTool/')]
    for path in moved:_set_world(stage,path,delta@poses[path])
    for path in (BASE_VISUAL,BASE_COLLIDER):_set_world(stage,path,delta@matrix(stage,path))
    j=UsdPhysics.Joint(stage.GetPrimAtPath(ROBOT+'/joints/base'))
    point=np.asarray(j.GetLocalPos0Attr().Get(),float)+poses[PLATFORM][:3,:3].T@delta[:3,3]
    j.GetLocalPos0Attr().Set(Gf.Vec3f(*map(float,point)))
    base_frame=poses[PLATFORM].copy();base_frame[2,3]+=.0125-drop_m
    retained=trimesh.util.concatenate([original[0],mesh])
    mass=_platform_mass(stage,retained,original,base_frame,poses[PLATFORM])
    mass['model']='Original 15kg carriage estimate allocated by closed CAD volume; retained back carriage + replacement flat plate, removed triangular bracket; original 3.5kg RB5 base estimate translated with its mounting plane'
    return dict(schema='s300-lowered-flat-platform-v1',drop_m=drop_m,
        plate_path=plate,plate_collider=collider,removed_triangle=triangle,
        plate_world_bounds_zero_lift_m=[lo.tolist(),hi.tolist()],plate_dimensions_m=(hi-lo).tolist(),
        plate_flat=True,plate_extended_to_retained_carriage=True,
        moving_rigid_bodies=moved,platform_mass=mass,lift_axis_limits_drives_unchanged=True,
        base_joint_parent_frame_changed=True,source_plate_hole_pattern_replaced=True,
        fabrication_and_load_strength_verified=False)


def prepare_assembly(source, output, previous_assets, ready_q=None, *, plate_drop_m=0., gripper_inset_m=0.):
    from compact_motion_assets import verify_compact_dependencies
    source=Path(source);output=Path(output)
    manifest=json.loads((source/'motion_manifest.json').read_text())
    verify_compact_dependencies(source,manifest)
    if manifest.get('side_robot_mount'): raise ValueError('Use the original upright compact assembly')
    output.mkdir(parents=True,exist_ok=False)
    stage=Usd.Stage.CreateNew(str(output/'scene.usda'))
    stage.GetRootLayer().subLayerPaths=[str((source/'composed.usdc').resolve())]
    report=apply_adapter(stage,gripper_inset_m)
    report['initial_station_rebase']=rebase_initial_station(stage,manifest,0.)
    preserve_trimmed_supports(stage,previous_assets)
    report['base_mount']='Original upright RB5 seated on actual moving CAD mounting_plate; original real-lift prismatic joint'
    if plate_drop_m:
        report['lowered_platform']=lower_platform(stage,plate_drop_m)
        report['base_mount']='Upright RB5 on lowered flat plate at carriage foot; triangular cantilever removed; original real-lift prismatic joint'
    report['preserved_diagonal_removal']=True
    manifest['upright_yaw_mount']=report
    if ready_q is not None: author_ready(stage,manifest,ready_q)
    return stage,manifest


def save_assembly(stage, manifest, source, output):
    output=Path(output)
    stage.Flatten().Export(str(output/'composed.usdc'))
    stage.GetRootLayer().Clear();stage.GetRootLayer().subLayerPaths=['composed.usdc']
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'));stage.GetRootLayer().Save()
    manifest['scene_sha256']=sha(output/'scene.usda')
    manifest['compact_assets']['materialized_dependency']['sha256']=sha(output/'composed.usdc')
    manifest['compact_assets']['geometry_unchanged']=False
    manifest['upright_yaw_source']=dict(assembly=str(source),sha256=sha(Path(source)/'composed.usdc'))
    (output/'motion_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    for path in Path(source).glob('skin*.npz'): shutil.copy2(path,output/path.name)


def mount_forearm_camera(stage, manifest, q):
    from motion_kinematics import ToolKinematics
    old=manifest['mounted_rgbd']
    body=ROBOT+'/link3';root=body+'/ForearmRGBD'
    frames=ToolKinematics(stage).frames(q)
    normal=np.array([-1.,0.,0.]);down=np.array([0.,0.,-1.])
    local=np.eye(4)
    local[:3,:3]=frames[body][:3,:3].T@np.column_stack((np.cross(down,normal),down,normal))
    pivot=np.linalg.inv(frames[body])@frames[ROBOT+'/link4']
    local[:3,3]=pivot[:3,3]+.098*local[:3,2]-.0325*local[:3,1]
    original_root=np.linalg.inv(matrix(stage,old['body_path']))@matrix(stage,old['root_path'])
    UsdGeom.Xform.Define(stage,root)
    Sdf.CopySpec(stage.Flatten(),old['root_path'],stage.GetEditTarget().GetLayer(),root)
    revised_root=local@np.linalg.inv(old['body_from_camera_optical'])@original_root
    UsdGeom.Xformable(stage.GetPrimAtPath(root)).MakeMatrixXform().Set(Gf.Matrix4d(revised_root.T.tolist()))
    stage.GetPrimAtPath(old['root_path']).SetActive(False)
    mounted=dict(old,body_path=body,root_path=root,camera_path=root+'/Camera',
        body_from_camera_optical=local.tolist(),reference_ready_q=np.asarray(q).tolist(),
        mount_description='Portrait RGB-D at distal forearm cover; horizontal left-bed view at new upright ready pose',
        mount_calibration_revision='20261110 upright lift mount with wrist yaw adapter')
    manifest['mounted_rgbd']=mounted
    return mounted


def mount_platform_camera(stage, manifest, q):
    """Portrait RGB-D on the middle of the crop-facing long plate edge."""
    old=manifest['mounted_rgbd'];body=PLATFORM;root=body+'/PlatformRGBD'
    plate=manifest['upright_yaw_mount']['lowered_platform']
    lo,hi=np.asarray(plate['plate_world_bounds_zero_lift_m'])
    optical=np.eye(4)
    optical[:3,:3]=np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
    optical[:3,3]=[lo[0]-.035,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2]
    local=np.linalg.inv(matrix(stage,body))@optical
    original_root=np.linalg.inv(matrix(stage,old['body_path']))@matrix(stage,old['root_path'])
    UsdGeom.Xform.Define(stage,root)
    Sdf.CopySpec(stage.Flatten(),old['root_path'],stage.GetEditTarget().GetLayer(),root)
    revised_root=local@np.linalg.inv(old['body_from_camera_optical'])@original_root
    UsdGeom.Xformable(stage.GetPrimAtPath(root)).MakeMatrixXform().Set(Gf.Matrix4d(revised_root.T.tolist()))
    stage.GetPrimAtPath(old['root_path']).SetActive(False)
    # Replace the round-cover bracket with a short plate-edge standoff.
    stage.GetPrimAtPath(root+'/Bracket').SetActive(False)
    from build_scene import box
    support=body+'/PlatformRGBDSupport'
    center=np.array([lo[0]-.005,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2])
    position=(np.linalg.inv(matrix(stage,body))@np.r_[center,1])[:3]
    part=box(stage,support,position,[.012,.025,.010],(.18,.20,.22))
    part.GetPrim().RemoveAPI(UsdPhysics.CollisionAPI)
    mounted=dict(old,body_path=body,root_path=root,camera_path=root+'/Camera',
        body_from_camera_optical=local.tolist(),reference_ready_q=np.asarray(q).tolist(),
        mount_description='Portrait RGB-D centered on crop-facing long edge of lowered lift plate; rigid to carriage, independent of arm joints',
        mount_calibration_revision='20261110 lowered flat plate central portrait camera',
        optical_world_zero_lift=optical.tolist(),housing_support_path=support,
        plate_edge_optical_standoff_m=.035)
    manifest['mounted_rgbd']=mounted
    return mounted


def mount_wrist_top_camera(stage, manifest, q, tilt_deg=15.):
    """Fixed portrait camera on the upper cap of link5, before wrist3 rotation."""
    from motion_kinematics import ToolKinematics
    if not np.isfinite(tilt_deg) or not 0 <= tilt_deg <= 30:
        raise ValueError('Wrist camera downward pitch must be within 0..30deg')
    old=manifest['mounted_rgbd'];body=ROBOT+'/link5';root=body+'/WristTopRGBD'
    frames=ToolKinematics(stage).frames(q)
    cap=_world_mesh(stage,body+'/visuals/link5/Scene/mesh')
    delta=frames[body]@np.linalg.inv(matrix(stage,body))
    vertices=cap.vertices@delta[:3,:3].T+delta[:3,3]
    cap_top=float(vertices[:,2].max());axis=frames[ROBOT+'/link6'][:3,3]
    angle=np.deg2rad(tilt_deg)
    forward=np.array([-np.cos(angle),0.,-np.sin(angle)])
    down=np.array([np.sin(angle),0.,-np.cos(angle)])
    optical=np.eye(4);optical[:3,:3]=np.column_stack((np.cross(down,forward),down,forward))
    optical[:3,3]=[axis[0],axis[1],cap_top+.055]
    local=np.linalg.inv(frames[body])@optical
    original_root=np.linalg.inv(matrix(stage,old['body_path']))@matrix(stage,old['root_path'])
    UsdGeom.Xform.Define(stage,root)
    Sdf.CopySpec(stage.Flatten(),old['root_path'],stage.GetEditTarget().GetLayer(),root)
    revised_root=local@np.linalg.inv(old['body_from_camera_optical'])@original_root
    UsdGeom.Xformable(stage.GetPrimAtPath(root)).MakeMatrixXform().Set(Gf.Matrix4d(revised_root.T.tolist()))
    stage.GetPrimAtPath(old['root_path']).SetActive(False)
    bracket=stage.GetPrimAtPath(root+'/Bracket')
    if bracket:bracket.SetActive(False)
    # The optical center is offset within the supplied housing. Seat its actual
    # lower mesh, not an assumed lens-centered bounding box, above the cap.
    housing=_world_mesh(stage,root+'/Housing/mesh')
    moved=frames[body]@np.linalg.inv(matrix(stage,body))
    housing_vertices=housing.vertices@moved[:3,:3].T+moved[:3,3]
    seat_height=.014
    optical[2,3]+=cap_top+seat_height-float(housing_vertices[:,2].min())
    local=np.linalg.inv(frames[body])@optical
    revised_root=local@np.linalg.inv(old['body_from_camera_optical'])@original_root
    UsdGeom.Xformable(stage.GetPrimAtPath(root)).MakeMatrixXform().Set(Gf.Matrix4d(revised_root.T.tolist()))
    from build_scene import box
    support=body+'/WristTopRGBDSupport'
    support_world=np.eye(4);support_world[:3,3]=[axis[0]+.020,axis[1],cap_top+.007]
    support_local=np.linalg.inv(frames[body])@support_world
    part=box(stage,support,[0,0,0],[.060,.030,.014],(.18,.20,.22))
    UsdGeom.Xformable(part).MakeMatrixXform().Set(Gf.Matrix4d((support_local@np.diag([.060,.030,.014,1])).T.tolist()))
    part.GetPrim().RemoveAPI(UsdPhysics.CollisionAPI)
    mounted=dict(old,body_path=body,root_path=root,camera_path=root+'/Camera',
        body_from_camera_optical=local.tolist(),reference_ready_q=np.asarray(q).tolist(),
        mount_description='Portrait RGB-D fixed on upper wrist link5 cap; downward pitch at upright ready, follows physical link5 and not wrist3 yaw',
        mount_calibration_revision='20261110 inset gripper and wrist-top camera',
        downward_tilt_at_reference_deg=tilt_deg,reference_cap_top_world_z_m=cap_top,
        reference_world_from_camera_optical=optical.tolist(),housing_support_path=support,
        optical_center_above_cap_m=float(optical[2,3]-cap_top),housing_bottom_above_cap_m=seat_height,
        fixed_mount_shared_between_elbow_modes=True)
    manifest['mounted_rgbd']=mounted
    return mounted


def prepare_assets(source, output, compact_cache, ready_solutions, *, plate_drop_m=0., gripper_inset_m=0., camera_mount=None, camera_down_tilt_deg=15., support_assets=None):
    """Reuse farm/plant meshes; re-export the changed robot with the existing adapter."""
    import copy
    import subprocess
    import sys
    from prepare_newton_patrol import idle_plan, write, ROOT, HERE, relocate_camera_assets, repose_robot_inputs
    from camera_patrol_runner import ready_camera_service_calibration
    from robot_adapter import add_robot
    source=Path(source).resolve();output=Path(output).resolve();compact_cache=Path(compact_cache).resolve()
    config=json.loads((source/'patrol_assets.json').read_text())
    solutions={r['mode']:r for r in json.loads(Path(ready_solutions).read_text())}
    if set(solutions)!={'up','down'}:raise ValueError('Both collision-checked ready modes required')
    output.mkdir(parents=True,exist_ok=False);up=output/'up';up.mkdir()
    sources=[]
    for item in config['plants']:
        p=up/f"plant_{item['plant_id']:02d}";p.mkdir()
        original=compact_cache/p.name/'compact';assembly=p/'assembly'
        stage,manifest=prepare_assembly(original,assembly,support_assets or source,solutions['up']['ready_q'],plate_drop_m=plate_drop_m,gripper_inset_m=gripper_inset_m)
        if camera_mount=='wrist-top':
            mounted=mount_wrist_top_camera(stage,manifest,solutions['up']['ready_q'],camera_down_tilt_deg)
        else:
            mounted=(mount_platform_camera if plate_drop_m else mount_forearm_camera)(stage,manifest,solutions['up']['ready_q'])
        save_assembly(stage,manifest,original,assembly)
        write(assembly/'mounted_rgbd.json',mounted)
        calibration=ready_camera_service_calibration(assembly,manifest,
            ['--ready-lift-offset-mm','0','--grasp-depth-mm','35'],config['bed_gutter_center_x'])
        write(p/'camera_calibration.json',calibration);write(p/'plan.json',idle_plan(manifest))
        sources.append(dict(item,assembly=str(assembly),camera_calibration=str(p/'camera_calibration.json')))
        print('New upright assembly',item['plant_id'],flush=True)
    # Robot export is independent of plant topology. Preserve every original
    # farm/plant XML subtree and mesh byte; replace only the robot tree/assets.
    model=up/'model';model.mkdir();(model/'meshes').mkdir()
    old_model=source/'model';tree=ET.parse(old_model/'scene.xml');root=tree.getroot()
    meta=json.loads((old_model/'export.json').read_text())
    if sha(old_model/'scene.xml')!=meta['model_sha256']:raise ValueError('Source visual SHA mismatch')
    old_robot_names=set(meta['robot']['body_names'].values())
    assets=root.find('asset');world=root.find('worldbody');contact=root.find('contact')
    for b in list(world):
        if b.tag=='body' and b.get('name') in old_robot_names: world.remove(b)
    for e in list(contact):
        if e.get('body1') in old_robot_names or e.get('body2') in old_robot_names:contact.remove(e)
    root.remove(root.find('actuator'))
    for e in list(assets):
        if e.get('name','').startswith('s300_g'): assets.remove(e)
        elif e.get('file'):
            e.set('file',str((old_model/root.find('compiler').get('meshdir')/e.get('file')).resolve()))
    first=Path(sources[0]['assembly']);manifest=json.loads((first/'motion_manifest.json').read_text())
    stage=Usd.Stage.Open(str(first/'scene.usda'))
    robot=add_robot(stage,manifest,root=root,world=world,assets=assets,contact=contact,meshdir=model/'meshes')
    tree.write(model/'scene.xml',encoding='unicode')
    for key in ('camera_relocation','lift_brace_removal'):meta.pop(key,None)
    meta['geometry']=[g for g in meta['geometry'] if g['owner'] not in old_robot_names]+robot['geometry']
    meta.update(robot=robot['metadata'],source_assembly=str(first),source_trial=str(first.parent),
        source_manifest=str(first/'motion_manifest.json'),source_manifest_sha256=sha(first/'motion_manifest.json'),
        source_usdc_sha256=sha(first/'composed.usdc'),model_sha256=sha(model/'scene.xml'),
        upright_yaw_mount=manifest['upright_yaw_mount'])
    shutil.copy2(first.parent/'plan.json',model/'source_plan.json');meta['source_plan_sha256']=sha(model/'source_plan.json')
    for w,s in zip(meta['windows'],sources):
        a=Path(s['assembly']);plan=a.parent/'plan.json'
        w.update(source_assembly=str(a),source_manifest=str(a/'motion_manifest.json'),
            source_manifest_sha256=sha(a/'motion_manifest.json'),source_plan=str(plan),source_plan_sha256=sha(plan))
    write(model/'export.json',meta)
    (up/'inventory.json').symlink_to(source/'inventory.json')
    catalog=json.loads((source/'sensor_catalog.json').read_text())
    from mounted_gt import build_catalog
    meshes,objects=build_catalog(stage,first)
    catalog['geom_to_instance']={n:i for n,i in catalog['geom_to_instance'].items() if not n.startswith('s300_g')}
    for row in robot['geometry']:
        identity=meshes[row['source']]
        catalog['geom_to_instance'][row['name']]=identity
        catalog['objects'][str(identity)]=objects[identity]
    write(up/'sensor_catalog.json',catalog)
    py=ROOT/'nvidia-sim/.newton-venv/bin/python'
    commands=([py,HERE/'newton_inputs.py',model,up/'inputs_full','--hz','220','--discard-visual-geoms','--allow-multiple-windows'],
        [py,HERE/'newton_plant_variant.py',up/'inputs_full',up/'inputs_coarse','--mode','coarse4'],
        [py,HERE/'newton_fruit_proxy.py',up/'inputs_coarse',up/'inputs_proxy'],
        [py,HERE/'newton_geometric_harvest.py','--inputs-only',up/'inputs_proxy',up/'inputs'])
    for command in commands:subprocess.run(list(map(str,command)),check=True)
    revised=copy.deepcopy(config)
    for key in ('camera_relocation','elbow_up','initial_pose_revision','lift_brace_removal'):revised.pop(key,None)
    revised.update(plants=sources,inputs=str(up/'inputs'),elbow_posture='up',
        mounted_rgbd=manifest['mounted_rgbd'],upright_yaw_mount=manifest['upright_yaw_mount'],
        source_patrol_assets=str(source))
    write(up/'patrol_assets.json',revised)
    down=output/'down'
    if plate_drop_m or camera_mount=='wrist-top':
        down.mkdir()
        for name in ('inputs','model','inventory.json','sensor_catalog.json'):(down/name).symlink_to(up/name)
        relocated=copy.deepcopy(revised)
        # Keep the pose used to calibrate this fixed mount. The other mode's
        # initial arm pose is separate; it does not reattach/turn the camera.
        relocated['mounted_rgbd']['profile_initial_q']=solutions['down']['ready_q']
        for plant in relocated['plants']:
            assembly=Path(plant['assembly']);m=json.loads((assembly/'motion_manifest.json').read_text())
            m['initial_robot_q']=solutions['down']['ready_q']
            calibration=ready_camera_service_calibration(assembly,m,
                ['--ready-lift-offset-mm','0','--grasp-depth-mm','35'],config['bed_gutter_center_x'])
            path=down/f"plant_{plant['plant_id']:02d}_camera_calibration.json"
            write(path,calibration);plant['camera_calibration']=str(path)
    else:
        relocated=relocate_camera_assets(up,down,ready_q=solutions['down']['ready_q'])
    ready=output/'down_ready.json';write(ready,solutions['down'])
    # Sensor helper intentionally shares physics. Replace its symlink with a
    # fresh initial-state directory, keeping the exact same compiled model.
    (down/'inputs').unlink()
    subprocess.run(list(map(str,[py,HERE/'prepare_newton_patrol.py',down/'inputs',
        '--repose-inputs-from',up/'inputs','--ready-q-json',ready])),check=True)
    relocated.update(inputs=str(down/'inputs'),elbow_posture='down')
    write(down/'patrol_assets.json',relocated)
    write(output/'patrol_assets.json',dict(schema='s300-patrol-start-modes-v1',
        start_modes=dict(up=str(up),down=str(down)),default_start_mode='up',training_eligible=False))
    write(output/'ready_solutions.json',list(solutions.values()))


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--compact-cache',type=Path,required=True)
    p.add_argument('--ready-solutions',type=Path,required=True)
    p.add_argument('--plate-drop-m',type=float,default=0.,help='Lower the mounting plate/base, remove triangular bracket and mount the camera centrally on its crop-facing edge')
    p.add_argument('--gripper-inset-m',type=float,default=0.)
    p.add_argument('--camera-mount',choices=['wrist-top'])
    p.add_argument('--camera-down-tilt-deg',type=float,default=15.)
    p.add_argument('--support-assets',type=Path,help='Previously trimmed lift supports when rebuilding plant assets')
    args=p.parse_args()
    prepare_assets(args.source,args.output,args.compact_cache,args.ready_solutions,plate_drop_m=args.plate_drop_m,gripper_inset_m=args.gripper_inset_m,camera_mount=args.camera_mount,camera_down_tilt_deg=args.camera_down_tilt_deg,support_assets=args.support_assets)
