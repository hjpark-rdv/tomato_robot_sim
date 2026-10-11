"""Separate rigid S300 carrier to restore native chassis/platform collisions.

PhysX can ignore per-joint collision enabling inside an articulation. Moving
S300 CAD into its base compound therefore hides chassis/platform contacts.
This derived model uses a regular external fixed constraint instead: the CAD
belongs to a separate carrier actor and the original articulation base mass
is restored exactly. Native pair controls remain required after export.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from compact_motion_assets import (file_sha256, property_inventory,
                                  verify_compact_dependencies)
from motion_stage_metrics import copy_stage_metrics, require_motion_units, stage_metrics
from rail_assets import (BASE_BODY, NEW_CHASSIS, ROBOT, ROOT_JOINT, RAIL_ANCHOR,
                        RAIL_FIXED_ROOT, _inertia_tensor, _parallel_axis)
from rail_filter_parity import PLATFORM, LIFT_JOINT, _colliders, _lift, _owner

CARRIER = ROBOT + '/S300Carrier'
CHASSIS = CARRIER + '/S300Chassis'
CARRIER_JOINT = ROBOT + '/S300CarrierJoint'
MASS_ATTRIBUTES = ('physics:mass','physics:density','physics:centerOfMass',
                   'physics:diagonalInertia','physics:principalAxes')


def _mass_snapshot(prim):
    return {name:str(prim.GetAttribute(name).Get()) for name in MASS_ATTRIBUTES}


def _copy_original_mass(source, destination):
    if not source.HasAPI(UsdPhysics.MassAPI):
        raise ValueError('Original base needs explicit MassAPI')
    for name in MASS_ATTRIBUTES:
        old=source.GetAttribute(name)
        destination.RemoveProperty(name)
        if old and old.HasAuthoredValue():
            value=old.Get()
            destination.CreateAttribute(name,old.GetTypeName(),custom=old.IsCustom(),
                                        variability=old.GetVariability()).Set(value)
    UsdPhysics.MassAPI.Apply(destination)
    if _mass_snapshot(source)!=_mass_snapshot(destination):
        raise RuntimeError('Original base mass properties were not restored exactly')


def _bounding_points(stage, root, frame):
    cache=UsdGeom.XformCache();points=[];cube_points=[]
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root),Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh):vertices=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),float)
        elif prim.IsA(UsdGeom.Cube):
            half=float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2
            vertices=np.array([[x,y,z] for x in (-half,half) for y in (-half,half) for z in (-half,half)])
        else:continue
        pose=np.linalg.inv(frame)@np.asarray(cache.GetLocalToWorldTransform(prim)).T
        transformed=vertices@pose[:3,:3].T+pose[:3,3]
        (points if prim.IsA(UsdGeom.Mesh) else cube_points).append(transformed)
    # Keep the original rail conversion's CAD-mesh bounding-box mass model.
    # A deck cube must not silently change the existing inertia/COM assumption.
    # Cube-only fixtures retain a useful simple geometry equivalent.
    if not points:points=cube_points
    if not points:raise ValueError('Chassis has no actual geometry for its mass assumption')
    vertices=np.concatenate(points)
    if not np.isfinite(vertices).all():raise ValueError('Nonfinite chassis geometry')
    return vertices


def split_chassis_body(stage, source_stage, *, chassis_mass_kg=150.):
    """Modify only a caller-owned stage, preserving original CAD/robot properties."""
    if not np.isfinite(chassis_mass_kg) or chassis_mass_kg<=0:
        raise ValueError('Explicit chassis mass must be finite and positive')
    if stage.GetPrimAtPath(CARRIER) or stage.GetPrimAtPath(CARRIER_JOINT):
        raise ValueError('Separate carrier already exists')
    chassis=stage.GetPrimAtPath(NEW_CHASSIS)
    base=stage.GetPrimAtPath(BASE_BODY)
    source_base=source_stage.GetPrimAtPath(BASE_BODY)
    if not chassis or not base or not source_base or not base.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('Missing original base or exact compound rail chassis')
    if _lift(stage).GetCollisionEnabledAttr().Get() is not True:
        raise ValueError('Keep the checked enabled lift-joint FCL policy before splitting')
    colliders=_colliders(stage,NEW_CHASSIS)
    if not colliders or any(_owner(stage.GetPrimAtPath(path))!=BASE_BODY for path in colliders):
        raise ValueError('Unexpected compound chassis collider ownership')
    source_mass=float(UsdPhysics.MassAPI(source_base).GetMassAttr().Get())
    compound_mass=float(UsdPhysics.MassAPI(base).GetMassAttr().Get())
    if not np.isfinite([source_mass,compound_mass]).all() or source_mass<=0 or \
            not np.isclose(compound_mass,source_mass+chassis_mass_kg,rtol=1e-9,atol=1e-6):
        raise ValueError('Source/compound mass does not match the explicit chassis assumption')
    cache=UsdGeom.XformCache()
    base_pose=np.asarray(cache.GetLocalToWorldTransform(base)).T
    vertices=_bounding_points(stage,NEW_CHASSIS,base_pose)
    low,high=vertices.min(0),vertices.max(0);center=(low+high)/2;size=high-low
    inertia=chassis_mass_kg/12*np.array([size[1]**2+size[2]**2,
        size[0]**2+size[2]**2,size[0]**2+size[1]**2])
    if (inertia<=0).any():raise ValueError('Chassis bounding box has invalid inertia')
    old_mass=_mass_snapshot(base)
    old_center=np.asarray(UsdPhysics.MassAPI(base).GetCenterOfMassAttr().Get(),float)
    old_inertia=_inertia_tensor(UsdPhysics.MassAPI(base))
    local=UsdGeom.Xformable(base).GetLocalTransformation()
    carrier=UsdGeom.Xform.Define(stage,CARRIER)
    carrier.MakeMatrixXform().Set(local)
    carrier.SetResetXformStack(UsdGeom.Xformable(base).GetResetXformStack())
    rigid=UsdPhysics.RigidBodyAPI.Apply(carrier.GetPrim())
    rigid.CreateRigidBodyEnabledAttr(True)
    rigid.CreateKinematicEnabledAttr(False)
    mass=UsdPhysics.MassAPI.Apply(carrier.GetPrim())
    mass.CreateMassAttr(float(chassis_mass_kg))
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,center)))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,inertia)))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1.))
    editor=Usd.NamespaceEditor(stage)
    if not editor.ReparentPrim(chassis,carrier.GetPrim(),'S300Chassis') or \
            not editor.CanApplyEdits() or not editor.ApplyEdits():
        raise RuntimeError('Cannot reparent original compound chassis into separate carrier')
    _copy_original_mass(source_base,base)
    restored=UsdPhysics.MassAPI(base)
    original_center=np.asarray(restored.GetCenterOfMassAttr().Get(),float)
    aggregate_center=(source_mass*original_center+chassis_mass_kg*center)/compound_mass
    aggregate_inertia=(_inertia_tensor(restored)+np.diag(inertia)+
        _parallel_axis(source_mass,original_center-aggregate_center)+
        _parallel_axis(chassis_mass_kg,center-aggregate_center))
    joint=UsdPhysics.FixedJoint.Define(stage,CARRIER_JOINT)
    joint.CreateBody0Rel().SetTargets([BASE_BODY])
    joint.CreateBody1Rel().SetTargets([CARRIER])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.))
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)
    mapped=[CHASSIS+p[len(NEW_CHASSIS):] for p in colliders]
    if set(_colliders(stage,CHASSIS))!=set(mapped) or \
            any(_owner(stage.GetPrimAtPath(path))!=CARRIER for path in mapped):
        raise RuntimeError('Separate carrier lost exact original collider coverage')
    return dict(schema='s300-separate-external-chassis-body-v1',carrier_body=CARRIER,
        carrier_joint=CARRIER_JOINT,chassis_root=CHASSIS,previous_chassis_root=NEW_CHASSIS,
        exclude_from_articulation=True,base_carrier_collision_enabled=False,
        original_base_mass_properties=_mass_snapshot(source_base),
        previous_compound_base_mass_properties=old_mass,
        restored_base_mass_kg=source_mass,chassis_mass_kg=float(chassis_mass_kg),
        combined_system_mass_kg=source_mass+float(chassis_mass_kg),
        carrier_center_of_mass_local_m=center.tolist(),carrier_diagonal_inertia_kg_m2=inertia.tolist(),
        carrier_mass_assumption='Same explicit uncalibrated chassis mass and original CAD-mesh bounding-box inertia; deck cube does not widen the existing assumption',
        aggregate_compound_com_max_abs_difference_m=float(np.max(np.abs(aggregate_center-old_center))),
        aggregate_compound_inertia_max_abs_difference_kg_m2=float(np.max(np.abs(aggregate_inertia-old_inertia))),
        aggregate_comparison_scope='Nominal tensor reconstruction versus previous authored float32 mass properties; exact original base restoration and unchanged total mass',
        chassis_enabled_colliders=len(mapped),lift_joint_collision_enabled=True,
        native_collision_pair_validation='pending; an authored attribute is not native pair evidence',
        dynamics_limit='External fixed constraint replaces the compound body. Mass/geometry are preserved but solver response and native stability require validation.',
        training_eligible=False,harvest_success=None)


def _mapped_geometry(source, derived):
    def mapped(path):return CHASSIS+path[len(NEW_CHASSIS):] if path==NEW_CHASSIS or path.startswith(NEW_CHASSIS+'/') else path
    old={str(p.GetPath()):p for p in Usd.PrimRange.Stage(source,Usd.TraverseInstanceProxies())}
    new={str(p.GetPath()):p for p in Usd.PrimRange.Stage(derived,Usd.TraverseInstanceProxies())}
    if {mapped(p) for p in old}|{CARRIER,CARRIER_JOINT}!=set(new):
        raise RuntimeError('Carrier introduced unexpected topology changes')
    a=UsdGeom.XformCache();b=UsdGeom.XformCache();mesh_count=vertices=collider_count=rigid_count=0
    max_vertex=max_collision=max_rigid=0.;digest=hashlib.sha256()
    for path,prim in old.items():
        other=new[mapped(path)]
        if prim.GetTypeName()!=other.GetTypeName():raise RuntimeError('Carrier changed original prim type')
        if prim.IsA(UsdGeom.Mesh):
            lm=UsdGeom.Mesh(prim);rm=UsdGeom.Mesh(other)
            points=np.asarray(lm.GetPointsAttr().Get(),float);next_points=np.asarray(rm.GetPointsAttr().Get(),float)
            for getter in ('GetFaceVertexCountsAttr','GetFaceVertexIndicesAttr'):
                if not np.array_equal(np.asarray(getattr(lm,getter)().Get()),np.asarray(getattr(rm,getter)().Get())):
                    raise RuntimeError('Carrier changed source mesh topology')
            if points.shape!=next_points.shape:raise RuntimeError('Carrier changed source vertex count')
            t=np.asarray(a.GetLocalToWorldTransform(prim)).T;u=np.asarray(b.GetLocalToWorldTransform(other)).T
            world=points@t[:3,:3].T+t[:3,3];next_world=next_points@u[:3,:3].T+u[:3,3]
            if world.size:max_vertex=max(max_vertex,float(np.max(np.abs(world-next_world))))
            if not np.isfinite(world).all() or not np.isfinite(next_world).all():raise RuntimeError('Nonfinite carrier geometry')
            digest.update(path.encode());digest.update(np.ascontiguousarray(world).tobytes())
            mesh_count+=1;vertices+=len(points)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            for name in ('physics:collisionEnabled','physics:approximation'):
                if prim.GetAttribute(name).Get()!=other.GetAttribute(name).Get():raise RuntimeError('Carrier changed collider enable/approximation')
            max_collision=max(max_collision,float(np.max(np.abs(np.asarray(a.GetLocalToWorldTransform(prim))-np.asarray(b.GetLocalToWorldTransform(other))))))
            collider_count+=1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            max_rigid=max(max_rigid,float(np.max(np.abs(np.asarray(a.GetLocalToWorldTransform(prim))-np.asarray(b.GetLocalToWorldTransform(other))))))
            rigid_count+=1
    if max(max_vertex,max_collision,max_rigid)>1e-8:raise RuntimeError('Carrier changed original geometry/rest poses')
    carrier_pose=np.asarray(b.GetLocalToWorldTransform(new[CARRIER]))
    base_pose=np.asarray(b.GetLocalToWorldTransform(new[BASE_BODY]))
    error=float(np.max(np.abs(carrier_pose-base_pose)))
    if error>1e-8:raise RuntimeError('Carrier/base rest frames differ')
    return dict(mesh_count=mesh_count,world_vertices=vertices,world_vertex_max_abs_error_m=max_vertex,
        collider_count=collider_count,collider_pose_max_abs_error=max_collision,
        original_rigid_body_count=rigid_count,original_rigid_pose_max_abs_error=max_rigid,
        carrier_base_rest_frame_max_abs_error=error,source_geometry_sha256=digest.hexdigest(),
        added_rigid_bodies=1,added_regular_fixed_constraints=1,original_geometry_unchanged=True)


def revise(assembly, output):
    assembly,output=Path(assembly).resolve(),Path(output).resolve()
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    digest=file_sha256(assembly/'scene.usda')
    if digest!=manifest['scene_sha256']:raise ValueError('Rail source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    if output.exists() or 'rail' not in manifest or manifest['rail'].get('carrier_body'):
        raise ValueError('Use original compound rail input and new output')
    source=Path(manifest['rail']['source_assembly']).resolve()
    source_manifest=json.loads((source/'motion_manifest.json').read_text())
    source_digest=file_sha256(source/'scene.usda')
    if source_digest!=source_manifest['scene_sha256'] or source_digest!=manifest['rail']['source_scene_sha256']:
        raise ValueError('Original base source SHA mismatch')
    verify_compact_dependencies(source,source_manifest)
    original=Usd.Stage.Open(str(assembly/'scene.usda'));baseline=Usd.Stage.Open(str(source/'scene.usda'))
    require_motion_units(original);require_motion_units(baseline)
    stage=Usd.Stage.Open(original.Flatten(addSourceFileComment=False));copy_stage_metrics(original,stage)
    report=split_chassis_body(stage,baseline,chassis_mass_kg=float(manifest['rail']['mass']['assumed_chassis_kg']))
    properties=property_inventory(stage)
    output.mkdir(parents=True,exist_ok=False)
    layer=stage.Flatten(addSourceFileComment=False)
    if layer.subLayerPaths or any(layer.GetExternalReferences()):raise RuntimeError('External carrier materialization dependencies remain')
    layer.Export(str(output/'composed.usdc'))
    wrapper=Usd.Stage.CreateNew(str(output/'scene.usda'))
    wrapper.GetRootLayer().subLayerPaths=['composed.usdc'];copy_stage_metrics(original,wrapper)
    if original.GetDefaultPrim():wrapper.SetDefaultPrim(wrapper.GetPrimAtPath(original.GetDefaultPrim().GetPath()))
    if original.HasAuthoredTimeCodeRange():wrapper.SetStartTimeCode(original.GetStartTimeCode());wrapper.SetEndTimeCode(original.GetEndTimeCode())
    wrapper.GetRootLayer().Save()
    if stage_metrics(original)!=stage_metrics(wrapper) or property_inventory(wrapper)!=properties:
        raise RuntimeError('Separate carrier export changed effective properties or units')
    geometry=_mapped_geometry(original,wrapper)
    report.update(source_assembly=str(assembly),source_scene_sha256=digest,
                  original_base_source=str(source),original_base_source_scene_sha256=source_digest,
                  geometry=geometry)
    shutil.copy2(assembly/'skin.npz',output/'skin.npz')
    dependency=dict(relative_path='composed.usdc',sha256=file_sha256(output/'composed.usdc'),
        format='usdc',materialized=True,external_usd_sublayers=[])
    old_mass=manifest['rail']['mass'];manifest['rail']['previous_compound_mass']=old_mass
    manifest['rail']['mass']=dict(original_base_kg=report['restored_base_mass_kg'],
        restored_base_kg=report['restored_base_mass_kg'],assumed_chassis_kg=report['chassis_mass_kg'],
        combined_base_kg=report['restored_base_mass_kg'],combined_system_kg=report['combined_system_mass_kg'],
        assumption=report['carrier_mass_assumption'])
    manifest['rail'].update(chassis_root=CHASSIS,carrier_body=CARRIER,carrier_joint=CARRIER_JOINT,
        native_validation='pending external carrier actor/contact pair controls')
    for row in manifest['rail'].get('chassis_collider_approximation_changes',[]):
        if row['path'].startswith(NEW_CHASSIS+'/'):row['path']=CHASSIS+row['path'][len(NEW_CHASSIS):]
    manifest['rail_chassis_body']=report
    manifest['scene_sha256']=file_sha256(output/'scene.usda')
    manifest['compact_assets']=dict(schema='s300-lossless-materialized-usdc-v1',source_assembly=str(assembly),
        source_scene_sha256=digest,source_manifest_sha256=file_sha256(assembly/'motion_manifest.json'),
        materialized_dependency=dependency,geometry_unchanged=True,native_validation='pending',
        note='Lossless composed storage; chassis actor split is a separately recorded dynamics/topology change')
    manifest['training_eligible']=False
    (output/'motion_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    (output/'rail_chassis_body.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if file_sha256(assembly/'scene.usda')!=digest or file_sha256(source/'scene.usda')!=source_digest:
        raise RuntimeError('Original assets changed during carrier revision')
    return report


def _copy_prim_only(source, destination, path, source_layer):
    """Copy one materialized prim's actual properties, without its descendants."""
    old=source.GetPrimAtPath(path)
    if not old:raise ValueError('Missing exact pair-control source prim: '+path)
    new=destination.DefinePrim(path,old.GetTypeName())
    for key,value in old.GetAllAuthoredMetadata().items():
        if key not in ('typeName','specifier'):new.SetMetadata(key,value)
    for prop in old.GetProperties():
        if prop.IsAuthored():
            if not Sdf.CopySpec(source_layer,prop.GetPath(),destination.GetRootLayer(),prop.GetPath()):
                raise RuntimeError('Cannot copy pair-control source property')
    return new


def _copy_prim_chain(source,destination,path,source_layer):
    for parent in Sdf.Path(path).GetPrefixes():
        if parent==Sdf.Path.absoluteRootPath:continue
        if not destination.GetPrimAtPath(parent):_copy_prim_only(source,destination,str(parent),source_layer)


def _cube_object(stage,path):
    import fcl
    cube=UsdGeom.Cube(stage.GetPrimAtPath(path))
    if not cube:raise ValueError('Native pair control requires exact original source cubes')
    tf=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(cube.GetPrim())).T
    scale=np.linalg.norm(tf[:3,:3],axis=0);rotation=tf[:3,:3]/scale
    if np.max(np.abs(rotation.T@rotation-np.eye(3)))>1e-8:
        raise ValueError('Sheared source box is not an exact box control')
    size=float(cube.GetSizeAttr().Get())*scale
    return fcl.CollisionObject(fcl.Box(*map(float,size)),fcl.Transform(rotation,tf[:3,3])),size,tf


def build_native_pair_controls(assembly,output):
    """Exact-size isolated boxes comparing native articulation adjacency.

    The platform box is deliberately moved down to intersect the original
    FlatRearDeck box. This is solely a detector calibration fixture, not a
    valid operational pose or a harvesting success. No original assembly is
    edited. The full operating lift-range negative screen remains separate.
    """
    import fcl
    assembly,output=Path(assembly).resolve(),Path(output).resolve()
    manifest=json.loads((assembly/'motion_manifest.json').read_text());digest=file_sha256(assembly/'scene.usda')
    if digest!=manifest['scene_sha256']:raise ValueError('Pair-control source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    if output.exists():raise ValueError('Native pair controls require a new immutable output')
    if not manifest.get('rail_ground_alignment'):raise ValueError('Use the corrected physical rail placement')
    source=Usd.Stage.Open(str(assembly/'scene.usda'));require_motion_units(source)
    source_layer=source.Flatten(addSourceFileComment=False);source=Usd.Stage.Open(source_layer)
    deck=CHASSIS+'/FlatRearDeck';platform=PLATFORM+'/collisions/mesh_0/box'
    # Known source boxes avoid confusing a conservative convex envelope with
    # the native per-part convex decomposition of a curved chassis mesh.
    deck_obj,deck_size,deck_tf=_cube_object(source,deck)
    platform_obj,platform_size,platform_tf=_cube_object(source,platform)
    baseline=fcl.distance(deck_obj,platform_obj,fcl.DistanceRequest(enable_signed_distance=True),fcl.DistanceResult())
    initial_lift=float(manifest['initial_robot_q'][0])
    offset=np.array([0.,0.,deck_tf[2,3]-platform_tf[2,3]-initial_lift])
    controls=[];output.mkdir(parents=True,exist_ok=False)
    for case in ('compound_adjacent','external_carrier'):
        stage=Usd.Stage.CreateInMemory();copy_stage_metrics(source,stage)
        required=['/World/PhysicsScene',BASE_BODY,PLATFORM,RAIL_ANCHOR,RAIL_FIXED_ROOT,ROOT_JOINT,LIFT_JOINT]
        if case=='external_carrier':required += [CARRIER,CARRIER_JOINT,CHASSIS,deck]
        for path in required+['/World/HarvestRobot',platform]:
            _copy_prim_chain(source,stage,path,source_layer)
            _copy_prim_only(source,stage,path,source_layer)
        # Match the actual source's ready DOF state: its body rest transforms
        # are authored at zero, so this isolated actor is placed at ready q.
        body=UsdGeom.Xformable(stage.GetPrimAtPath(PLATFORM))
        local=np.asarray(body.GetLocalTransformation()).T;local[2,3]+=initial_lift
        body.MakeMatrixXform().Set(Gf.Matrix4d(local.T.tolist()))
        shape=stage.GetPrimAtPath(platform);shape_tf=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(shape)).T
        shape_tf[:3,3]+=offset
        parent_tf=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(shape.GetParent())).T
        UsdGeom.Xformable(shape).MakeMatrixXform().Set(Gf.Matrix4d((np.linalg.inv(parent_tf)@shape_tf).T.tolist()))
        if case=='compound_adjacent':
            # Same original source deck and world pose, different rigid owner.
            _copy_prim_chain(source,stage,BASE_BODY,source_layer)
            new_root=UsdGeom.Xform.Define(stage,NEW_CHASSIS)
            owner=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(BASE_BODY))).T
            UsdGeom.Xformable(new_root).MakeMatrixXform().Set(Gf.Matrix4d(np.linalg.inv(owner).T.tolist()))
            current=NEW_CHASSIS+'/FlatRearDeck';stage.DefinePrim(current,'Cube')
            if not Sdf.CopySpec(source_layer,Sdf.Path(deck),stage.GetRootLayer(),Sdf.Path(current)):
                raise RuntimeError('Cannot copy the exact original deck control shape')
            parent=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(new_root.GetPrim())).T
            UsdGeom.Xformable(stage.GetPrimAtPath(current)).MakeMatrixXform().Set(Gf.Matrix4d((np.linalg.inv(parent)@deck_tf).T.tolist()))
            # Use the source compound actor's exact mass API, separately from
            # source geometry. It is already preserved as an immutable asset.
            compound=Path(manifest['rail_chassis_body']['source_assembly'])
            cm=json.loads((compound/'motion_manifest.json').read_text())
            if file_sha256(compound/'scene.usda')!=cm['scene_sha256']:raise ValueError('Compound source SHA mismatch')
            verify_compact_dependencies(compound,cm)
            cs=Usd.Stage.Open(str(compound/'scene.usda'))
            _copy_original_mass(cs.GetPrimAtPath(BASE_BODY),stage.GetPrimAtPath(BASE_BODY))
        else:current=deck
        shapes=_colliders(stage,'/World')
        if set(shapes)!={current,platform}:raise RuntimeError('Unexpected diagnostic collider inventory')
        a,actual_size_a,a_tf=_cube_object(stage,current);b,actual_size_b,b_tf=_cube_object(stage,platform)
        if not np.array_equal(actual_size_a,deck_size) or not np.array_equal(actual_size_b,platform_size):
            raise RuntimeError('Native pair-control changed source box dimensions')
        result=fcl.CollisionResult();fcl.collide(a,b,fcl.CollisionRequest(enable_contact=True,num_max_contacts=16),result)
        signed=float(fcl.distance(a,b,fcl.DistanceRequest(enable_signed_distance=True),fcl.DistanceResult()))
        if not result.is_collision or not signed<0.:raise RuntimeError('Chosen native positive control is not an exact geometric intersection')
        file=output/(case+'.usda');stage.GetRootLayer().Export(str(file))
        controls.append(dict(case=case,stage=str(file),scene_sha256=file_sha256(file),
            exact_source_chassis_shape=deck,chassis_shape=current,platform_shape=platform,
            chassis_actor=_owner(stage.GetPrimAtPath(current)),platform_actor=PLATFORM,
            source_box_sizes_m=[deck_size.tolist(),platform_size.tolist()],
            deliberate_platform_shape_offset_world_m=offset.tolist(),initial_lift_q_m=initial_lift,
            source_zero_pose_signed_distance_m=float(baseline),positive_signed_distance_m=signed,
            fcl_contact_depths_m=[float(c.penetration_depth) for c in result.contacts],
            lift_collision_enabled=_lift(stage).GetCollisionEnabledAttr().Get(),
            expected_native_intersection_detection=case=='external_carrier',
            native_result='pending',training_eligible=False,harvest_success=None))
    report=dict(schema='s300-native-chassis-platform-positive-controls-v1',source_assembly=str(assembly),
        source_scene_sha256=digest,source_materialized_dependency=manifest['compact_assets']['materialized_dependency'],
        controls=controls,scope='Two exact-source-box detector fixtures with a deliberate shape placement offset. Full farm and other robot CAD are omitted; neither fixture is a legal harvest trace.',
        original_assets_unchanged=True,native_validation='pending actual PhysX collider callbacks',training_eligible=False)
    (output/'controls.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if file_sha256(assembly/'scene.usda')!=digest:raise RuntimeError('Original pair-control assembly changed')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('assembly',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();print(json.dumps(revise(args.assembly,args.output),indent=2))
