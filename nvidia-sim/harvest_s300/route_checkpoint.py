"""Immutable active-plant window switch at a measured native checkpoint.

The cart does not move here. The next asset starts at the measured old station,
and its existing physical rail joint subsequently performs actual travel.
Outgoing attached plant geometry is frozen at its measured deformation; detached
fruit remains dynamic with measured velocities. This is a documented active
window approximation and native-engine restart, not uninterrupted all-plant
elastic simulation. Source CAD/tool/mount and prior harvest evidence survive.
"""
import argparse
import copy
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from motion_assets import REPO, ROBOT, attr, build_plant, grasp_reference, select_build_target
from motion_kinematics import ToolKinematics
from motion_stage_metrics import copy_stage_metrics, require_motion_units
from rail_assets import BASE_BODY, ROOT_JOINT, sha


def _matrix(pose):
    pose=np.asarray(pose,float)
    if pose.shape!=(7,) or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1.)>1e-4:
        raise ValueError('Finite XYZ + normalized XYZW native body pose required')
    result=np.eye(4);result[:3,:3]=Rotation.from_quat(pose[3:]).as_matrix();result[:3,3]=pose[:3]
    return result


def _world(stage,path):
    p=stage.GetPrimAtPath(path)
    if not p:raise ValueError('Checkpoint path absent from composed asset: '+path)
    return np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(p)).T


def _set_world(stage,path,world):
    p=stage.GetPrimAtPath(path)
    parent=p.GetParent()
    parent_world=np.eye(4) if parent.IsPseudoRoot() else _world(stage,str(parent.GetPath()))
    local=np.linalg.inv(parent_world)@world
    UsdGeom.Xformable(p).MakeMatrixXform().Set(Gf.Matrix4d(local.T.tolist()))


def _canonicalize_transmission_zero_signs(stage):
    """USD serializes signed-zero identity matrices as its unsigned identity.

    Canonicalize only the three freshly recomputed display transforms before
    the exact byte-property audit. Every numeric matrix entry is identical;
    no epsilon rounding, pose/collider correction or audit weakening occurs.
    """
    rows=[]
    for name in ('crank','linkL','linkR'):
        path='/World/ActuatedTool/fixed/Transmission/'+name
        prim=stage.GetPrimAtPath(path)
        if not prim:continue
        attr=prim.GetAttribute('xformOp:transform')
        if not attr or attr.Get() is None:continue
        before=np.array(attr.Get(),dtype=np.float64,copy=True)
        count=int(np.count_nonzero((before==0.)&np.signbit(before)))
        if not count:continue
        after=before.copy();after[after==0.]=0.
        if not np.array_equal(before,after):raise ValueError('Zero-sign canonicalization changed a numeric transform')
        # Sdf considers -0 == +0 and can skip an equal Set. Clear this newly
        # authored display default first so the canonical bits are stored.
        attr.Clear();attr.Set(Gf.Matrix4d(after.tolist()))
        rows.append(dict(path=path,signed_zero_entries=count,numeric_transform_change=False))
    return rows


def _property_differences(source,derived,limit=30):
    """Bounded exact diagnostics for a failed lossless storage assertion."""
    from compact_motion_assets import _value_digest
    left={str(p.GetPath()):p for p in Usd.PrimRange.Stage(source,Usd.TraverseInstanceProxies())}
    right={str(p.GetPath()):p for p in Usd.PrimRange.Stage(derived,Usd.TraverseInstanceProxies())}
    differences=[]
    for path in sorted(set(left)|set(right)):
        if path not in left or path not in right:
            differences.append(dict(path=path,field='prim',source=path in left,derived=path in right));continue
        a,b=left[path],right[path]
        for key,getter in [('type',lambda p:p.GetTypeName()),('schemas',lambda p:p.GetAppliedSchemas()),
                           ('custom_data',lambda p:p.GetCustomData()),('active',lambda p:p.IsActive()),
                           ('effective_apiSchemas',lambda p:p.GetMetadata('apiSchemas').ApplyOperations([]) if p.GetMetadata('apiSchemas') is not None else None)]:
            old,new=str(getter(a)),str(getter(b))
            if old!=new:differences.append(dict(path=path,field=key,source=old,derived=new))
        aa={x.GetName():x for x in a.GetAttributes()};bb={x.GetName():x for x in b.GetAttributes()}
        for name in sorted(set(aa)|set(bb)):
            if name not in aa or name not in bb:
                differences.append(dict(path=path,field=name+' presence',source=name in aa,derived=name in bb));continue
            x,y=aa[name],bb[name]
            for key,old,new in [('value',_value_digest(x.Get()),_value_digest(y.Get())),
                                ('metadata',str(x.GetAllMetadata()),str(y.GetAllMetadata())),
                                ('connections',str(x.GetConnections()),str(y.GetConnections()))]:
                if old!=new:differences.append(dict(path=path,field=name+' '+key,source=old[:400],derived=new[:400]))
        if len(differences)>=limit:return differences[:limit]
    return differences[:limit]


def validate_snapshot(metadata,arrays,manifest):
    """Reject missing/invented native state; no guessed zero velocities."""
    if metadata['scene_sha256']!=manifest['scene_sha256']:
        raise ValueError('Checkpoint/model SHA mismatch')
    if metadata['source_plant']!=manifest['plant']['source_plant']:
        raise ValueError('Checkpoint active plant mismatch')
    paths=list(map(str,arrays['body_paths']))
    if len(set(paths))!=len(paths) or BASE_BODY not in paths:
        raise ValueError('Duplicate body identities or missing measured moving base')
    poses=np.asarray(arrays['body_poses_world'],float);vel=np.asarray(arrays['body_velocities_world'],float)
    if poses.shape!=(len(paths),7) or vel.shape!=(len(paths),6) or not np.isfinite(np.r_[poses.ravel(),vel.ravel()]).all():
        raise ValueError('Complete finite measured body poses and velocities required')
    for pose in poses:_matrix(pose)
    for kind in ('robot','plant'):
        names=list(map(str,arrays[kind+'_dof_names']));q=np.asarray(arrays[kind+'_q'],float);v=np.asarray(arrays[kind+'_qvel'],float)
        if len(set(names))!=len(names) or q.shape!=(len(names),) or v.shape!=q.shape or not np.isfinite(np.r_[q,v]).all():
            raise ValueError('Complete finite native '+kind+' DOF states required')
    if not np.isfinite(metadata['time_s']) or metadata['time_s']<0:
        raise ValueError('Finite native timestamp required')
    return {path:i for i,path in enumerate(paths)}


def author_robot_state(stage,manifest,arrays,controller_state=None):
    """Keep zero-pose FK geometry and initialize native q at the old station."""
    paths=list(map(str,arrays['body_paths']));poses=np.asarray(arrays['body_poses_world'],float)
    velocities=np.asarray(arrays['body_velocities_world'],float)
    base_index=paths.index(BASE_BODY);measured=_matrix(poses[base_index]);source=_world(stage,BASE_BODY)
    delta=measured@np.linalg.inv(source)
    old_rail=copy.deepcopy(manifest['rail'])
    if old_rail.get('carrier_body') and old_rail['carrier_body'] not in paths:
        raise ValueError('Checkpoint lacks measured moving S300 carrier state')
    station=float(old_rail['initial_station_y_m']+measured[1,3]-source[1,3])
    low,high=old_rail['limits_world_y_m']
    if not low<=station<=high:raise ValueError('Measured cart station is outside physical rail')
    # ToolKinematics derives ZERO-joint transforms from these bodies. Setting
    # measured arm transforms would double-apply q; preserve zero poses, shifted
    # with the measured root, then author the exact native joint state.
    anchor_path=old_rail.get('anchor_body')
    rest={str(p.GetPath()):_world(stage,str(p.GetPath())) for root in (ROBOT,'/World/ActuatedTool')
          for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    for path in sorted(rest,key=lambda p:p.count('/')):
        if path!=anchor_path:_set_world(stage,path,delta@rest[path])
    robot_names=list(map(str,arrays['robot_dof_names']));q=np.asarray(arrays['robot_q'],float)
    controller_position=None if controller_state is None else controller_state['position_targets']
    controller_velocity=None if controller_state is None else controller_state['velocity_targets']
    qvel=np.asarray(arrays['robot_qvel'],float);mapping={n:(float(a),float(b)) for n,a,b in zip(robot_names,q,qvel)}
    joints={p.GetName():p for root in (ROBOT,'/World/ActuatedTool')
            for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.IsA(UsdPhysics.Joint)}
    if set(mapping)-set(joints):raise ValueError('Native robot DOF missing from USD: '+str(sorted(set(mapping)-set(joints))))
    for name,(value,speed) in mapping.items():
        p=joints[name];linear=p.IsA(UsdPhysics.PrismaticJoint);kind='linear' if linear else 'angular'
        if name==old_rail['dof_name']:value=0.
        position=value if linear else np.rad2deg(value);velocity=speed if linear else np.rad2deg(speed)
        p.AddAppliedSchema('PhysicsJointStateAPI:'+kind)
        attr(p,'state:'+kind+':physics:position',Sdf.ValueTypeNames.Float,float(position))
        attr(p,'state:'+kind+':physics:velocity',Sdf.ValueTypeNames.Float,float(velocity))
        drive=UsdPhysics.DriveAPI(p,kind)
        if drive:
            index=robot_names.index(name)
            target=value if controller_position is None else controller_position[index]
            drive.CreateTargetPositionAttr(float(target if linear else np.rad2deg(target)))
            # JointState velocity is measured motion. It must NEVER become a
            # velocity-drive target. Unknown legacy commands stay authored.
            if controller_velocity is not None:
                target_speed=controller_velocity[index]
                drive.CreateTargetVelocityAttr(float(target_speed if linear else np.rad2deg(target_speed)))
    j=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(ROOT_JOINT))
    targets=j.GetBody0Rel().GetTargets()
    frame=np.eye(4);frame[:3,3]=measured[:3,3]
    frame0=np.linalg.inv(_world(stage,str(targets[0])))@frame if targets else frame
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,frame0[:3,3])))
    q0=Rotation.from_matrix(frame0[:3,:3]).as_quat()
    j.CreateLocalRot0Attr(Gf.Quatf(float(q0[3]),Gf.Vec3f(*map(float,q0[:3]))))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.))
    quat=Rotation.from_matrix(measured[:3,:3].T).as_quat()
    j.CreateLocalRot1Attr(Gf.Quatf(float(quat[3]),Gf.Vec3f(*map(float,quat[:3]))))
    j.CreateLowerLimitAttr(low-station);j.CreateUpperLimitAttr(high-station)
    manifest['rail'].update(initial_station_y_m=station,limits_relative_m=[low-station,high-station],
        source_base_position_world_m=measured[:3,3].tolist(),initial_displacement_m=0.,
        checkpoint_rebase=dict(previous_initial_station_y_m=old_rail['initial_station_y_m'],
                               previous_native_q_m=mapping[old_rail['dof_name']][0],
                               world_station_preserved=True,native_initial_pose_verification='pending'))
    kin=ToolKinematics(stage)
    if set(kin.names)-set(mapping):raise ValueError('Missing native lift/arm DOFs')
    manifest['initial_robot_q']=[mapping[n][0] for n in kin.names]
    manifest['initial_robot_qvel']=[mapping[n][1] for n in kin.names]
    manifest['initial_jaw_q']={n:mapping[n][0] for n in ('jaw_L','jaw_R')}
    manifest['initial_jaw_qvel']={n:mapping[n][1] for n in ('jaw_L','jaw_R')}
    # Root/standalone body velocities are authored in USD units (angular deg/s).
    for path in rest:
        if path==anchor_path:continue
        if path not in paths:raise ValueError('Checkpoint is missing robot/tool body '+path)
        velocity=velocities[paths.index(path)];api=UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path))
        api.CreateVelocityAttr(Gf.Vec3f(*map(float,velocity[:3])))
        api.CreateAngularVelocityAttr(Gf.Vec3f(*map(float,np.rad2deg(velocity[3:]))))
    return dict(station_world_y_m=station,source_zero_pose_shift_world=delta.tolist(),
        measured_body_poses_authored_directly=False,
        robot_native_q_preserved=True,robot_native_qvel_preserved=True,
        controller_state=None if controller_state is None else controller_state['provenance'],
        unknown_velocity_target_left_authored=controller_velocity is None,
        note='Zero-pose rig translated with measured base; exact q/qvel JointState. Native startup pose/velocity comparison is mandatory.')


def author_background_state(stage,manifest,arrays,next_source_plant=None):
    """Preserve separately modeled passive leaves at the measured handoff."""
    from background_foliage import declared_bodies
    rows=declared_bodies(stage,manifest)
    source={str(p):i for i,p in enumerate(arrays['body_paths'])};proof=[]
    for path,row in rows.items():
        if next_source_plant and row['source_stem'].startswith(next_source_plant+'/'):
            raise ValueError('Active-plant switch into previously converted passive foliage requires an explicit joint/state transition; do not reset its leaf state')
        if path not in source:raise ValueError('Checkpoint missing measured passive background leaf '+path)
        i=source[path];pose=np.asarray(arrays['body_poses_world'][i],float)
        velocity=np.asarray(arrays['body_velocities_world'][i],float)
        if velocity.shape!=(6,) or not np.isfinite(velocity).all():raise ValueError('Finite measured background leaf velocities required')
        _set_world(stage,path,_matrix(pose));api=UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path))
        api.CreateVelocityAttr(Gf.Vec3f(*map(float,velocity[:3])))
        api.CreateAngularVelocityAttr(Gf.Vec3f(*map(float,np.rad2deg(velocity[3:]))))
        proof.append(dict(path=path,pose_world_xyzw=pose.tolist(),velocity_world_si=velocity.tolist(),
            joint=row['joint'],drive_rest_and_world_anchor_unchanged=True))
    if rows:manifest['background_compliant_foliage']['measured_checkpoint_state']=proof
    return proof


def freeze_outgoing(stage,assembly,manifest,metadata,arrays):
    """Bake exact measured shape, retain detached fruit native dynamics."""
    from motion_visuals import DeformationVisuals
    paths=list(map(str,arrays['body_paths']));poses=np.asarray(arrays['body_poses_world'],float)
    velocities=np.asarray(arrays['body_velocities_world'],float);index={p:i for i,p in enumerate(paths)}
    root=manifest['plant']['root'];source_plant=manifest['plant']['source_plant']
    broken={event['joint'] for event in metadata['break_events']}
    fruits=manifest['plant']['fruits'];detached=[f for f in fruits if f['joint'] in broken]
    detached_paths={f['path'] for f in detached};body_prims=[p for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for p in sorted(body_prims,key=lambda p:str(p.GetPath()).count('/')):
        path=str(p.GetPath())
        if path not in index:raise ValueError('Checkpoint missing outgoing body '+path)
        _set_world(stage,path,_matrix(poses[index[path]]))
    robot_names=list(map(str,arrays['robot_dof_names']));q=np.asarray(arrays['robot_q'],float)
    gap=.044+q[robot_names.index('jaw_R')]-q[robot_names.index('jaw_L')]
    DeformationVisuals(stage,assembly,paths).update(poses,float(gap))
    # All outgoing constraints are inactive, including already-broken ones.
    # Static old attached geometry never recreates an attachment or a leaf pose.
    for p in list(Usd.PrimRange(stage.GetPrimAtPath(root))):
        if p.IsA(UsdPhysics.Joint):
            # Flatten prunes inactive prims. Keep an exact declared, explicitly
            # disabled constraint so immutable release proof survives USDC.
            UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            p.RemoveAppliedSchema('PhysxArticulationAPI')
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            path=str(p.GetPath())
            if path in detached_paths:
                velocity=velocities[index[path]];api=UsdPhysics.RigidBodyAPI(p)
                api.CreateVelocityAttr(Gf.Vec3f(*map(float,velocity[:3])))
                api.CreateAngularVelocityAttr(Gf.Vec3f(*map(float,np.rad2deg(velocity[3:]))))
            else:
                p.RemoveAPI(UsdPhysics.RigidBodyAPI)
                p.RemoveAppliedSchema('PhysxRigidBodyAPI')
                p.RemoveAppliedSchema('PhysxContactReportAPI')
    # A stable identity per source plant avoids active-window aliases colliding.
    destination='/World/RouteFrozen/'+source_plant.rsplit('/',1)[-1]
    if stage.GetPrimAtPath(destination):raise ValueError('This plant was already frozen; revisits require an explicit recovery policy')
    parent=UsdGeom.Xform.Define(stage,'/World/RouteFrozen').GetPrim()
    editor=Usd.NamespaceEditor(stage)
    if not editor.ReparentPrim(stage.GetPrimAtPath(root),parent,destination.rsplit('/',1)[-1]) or not editor.CanApplyEdits() or not editor.ApplyEdits():
        raise RuntimeError('Cannot preserve outgoing measured plant')
    def remap(path):return destination+path[len(root):] if str(path).startswith(root) else str(path)
    detached_records=[]
    for f in detached:
        record=copy.deepcopy(f)
        for key in ('path','joint','anchor'):record[key]=remap(record[key])
        record.update(source_plant=source_plant,original_joint=f['joint'],
            route_identity=source_plant.rsplit('/',1)[-1]+'/'+f['name'],
        native_break_events=[e for e in metadata['break_events'] if e['joint']==f['joint']],
            source_scene_sha256=metadata['scene_sha256'],
            checkpoint_metadata_sha256=metadata.get('metadata_sha256'),
            checkpoint_time_s=metadata['time_s'],
            pose_world=poses[index[f['path']]].tolist(),velocity_world=velocities[index[f['path']]].tolist())
        detached_records.append(record)
    return dict(source_plant=source_plant,frozen_root=destination,
        source_scene_sha256=metadata['scene_sha256'],
        checkpoint_metadata_sha256=metadata.get('metadata_sha256'),checkpoint_time_s=metadata['time_s'],
        frozen_attached_bodies=len(body_prims)-len(detached),detached_fruits=detached_records,
        measured_shape_preserved=True,native_detach_evidence_preserved=True,
        approximation='Outgoing attached plant bodies become static at measured deformation; their velocities remain evidence, not continued elastic dynamics.',
        archived_plant_dof_names=list(map(str,arrays['plant_dof_names'])),
        archived_plant_q=np.asarray(arrays['plant_q']).tolist(),archived_plant_qvel=np.asarray(arrays['plant_qvel']).tolist())


def restore_prior_detached(stage,manifest,metadata,arrays):
    """Carry earlier plants' fallen fruit at their CURRENT measured state.

    Their original break checkpoints remain the release proof. A later state
    checkpoint updates position/velocity without resetting already fallen fruit
    to the earlier window's authored pose.
    """
    indices={str(path):i for i,path in enumerate(arrays['body_paths'])}
    poses=np.asarray(arrays['body_poses_world'],float)
    velocities=np.asarray(arrays['body_velocities_world'],float)
    records=[]
    for outgoing in manifest.get('route_checkpoint',{}).get('outgoing_history',[]):
        for fruit in outgoing.get('detached_fruits',[]):
            path=fruit['path'];prim=stage.GetPrimAtPath(path)
            if path not in indices or not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError('Prior native-detached route fruit lacks current measured dynamic state: '+path)
            if not path.startswith(outgoing['frozen_root']+'/'):
                raise ValueError('Prior detached route fruit identity aliases the new active plant')
            joint=stage.GetPrimAtPath(fruit['joint'])
            disabled=UsdPhysics.Joint(joint).GetJointEnabledAttr() if joint and joint.IsA(UsdPhysics.Joint) else None
            if not joint or (joint.IsActive() and (not disabled or disabled.Get() is not False or disabled.GetNumTimeSamples())):
                raise ValueError('Prior fruit attachment must remain explicitly disabled')
            records.append((fruit,prim,indices[path]))
    for fruit,prim,i in records:
        _set_world(stage,fruit['path'],_matrix(poses[i]))
        api=UsdPhysics.RigidBodyAPI(prim)
        api.CreateVelocityAttr(Gf.Vec3f(*map(float,velocities[i,:3])))
        api.CreateAngularVelocityAttr(Gf.Vec3f(*map(float,np.rad2deg(velocities[i,3:]))))
        fruit.update(pose_world=poses[i].tolist(),velocity_world=velocities[i].tolist(),
            measured_state_update=dict(source_scene_sha256=metadata['scene_sha256'],
                checkpoint_metadata_sha256=metadata.get('metadata_sha256'),time_s=metadata['time_s'],
                original_native_break_evidence_retained=True))
    return dict(updated_dynamic_fruit_count=len(records),
                paths=[row[0]['path'] for row in records],native_break_proof_unchanged=True)


def _joint_snapshot(prim):
    """Exact resolved joint properties, including frames and all drive/limit APIs."""
    return dict(type_name=prim.GetTypeName(),applied_schemas=list(prim.GetAppliedSchemas()),
        attributes={a.GetName():dict(value=str(a.Get()),type_name=str(a.GetTypeName()),
            samples=[(t,str(a.Get(t))) for t in a.GetTimeSamples()]) for a in prim.GetAttributes()},
        relationships={r.GetName():list(map(str,r.GetTargets())) for r in prim.GetRelationships()})


def _apply_attachment_template(stage,manifest,template,*,selected_names=None,selection_provenance=None):
    from fruit_attachment import configure_joint,frame_from_axis
    config=template['joints'][0];rows=[]
    keys=('kind','force_n','torque_nm','twist_stiffness_authored','swing_stiffness_authored','damping_authored','deadband_deg')
    if any(any(row[key]!=config[key] for key in keys) for row in template['joints']):
        raise ValueError('A heterogeneous source attachment template requires exact per-joint mapping')
    fruits=manifest['plant']['fruits'];by_name={f['name']:f for f in fruits}
    selected=list(by_name) if selected_names is None else list(selected_names)
    if not selected or len(selected)!=len(set(selected)) or not set(selected)<=set(by_name):
        raise ValueError('Attachment scope must select exact nonempty unique active fruit identities')
    before={}
    for f in fruits:
        prim=stage.GetPrimAtPath(f['joint'])
        if not prim or not prim.IsA(UsdPhysics.Joint):raise ValueError('Missing exact new fruit attachment')
        joint=UsdPhysics.Joint(prim)
        if (list(map(str,joint.GetBody0Rel().GetTargets()))!=[f['anchor']] or
                list(map(str,joint.GetBody1Rel().GetTargets()))!=[f['path']]):
            raise ValueError('New fruit/neck attachment body mapping mismatch')
        before[f['name']]=_joint_snapshot(prim)
    parameters=dict(kind=config['kind'],force_n=config['force_n'],torque_nm=config['torque_nm'],
        twist_k=config['twist_stiffness_authored'],swing_k=config['swing_stiffness_authored'],
        damping=config['damping_authored'],deadband_deg=config['deadband_deg'])
    for f in fruits:
        if f['name'] not in selected:continue
        j=UsdPhysics.Joint(stage.GetPrimAtPath(f['joint']))
        poses=[_world(stage,path) for path in (f['anchor'],f['path'])]
        pivot=poses[0][:3,:3]@np.asarray(j.GetLocalPos0Attr().Get())+poses[0][:3,3]
        frame=frame_from_axis(pivot-poses[0][:3,3])
        if config['kind']=='elastic':
            for i,pose in enumerate(poses):
                q=Rotation.from_matrix(pose[:3,:3].T@frame).as_quat()
                getattr(j,f'CreateLocalRot{i}Attr')().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
        actual=configure_joint(stage,f['joint'],**parameters)
        f.update(break_force_n=config['force_n'],break_torque_nm=config['torque_nm'])
        rows.append(dict(name=f['name'],joint=f['joint'],anchor=f['anchor'],fruit=f['path'],
                         pivot_world=pivot.tolist(),neck_axis_world=frame[:,0].tolist(),**actual))
    excluded=[f['name'] for f in fruits if f['name'] not in selected]
    for name in excluded:
        if _joint_snapshot(stage.GetPrimAtPath(by_name[name]['joint']))!=before[name]:
            raise ValueError('Excluded obstacle-truss attachment was changed')
    return dict(kind=config['kind'],joints=rows,
        scope=('Same approved neck hypothesis applied to every fruit of the new active plant' if selected_names is None else
               'Same approved neck hypothesis applied only to exact eligible rail-side target trusses; excluded obstacle-truss attachments unchanged'),
        selection='all_active_fruit_hypothesis' if selected_names is None else 'exact_eligible_near_trusses',
        selection_provenance=selection_provenance,source_template_scope=template.get('scope'),
        selected_fruit_names=selected,excluded_fruit_names=excluded,
        source_joint_properties=before,excluded_joint_properties_unchanged=True,
        model_revision='Explicit target attachment hypothesis; native startup validity remains required',
        calibrated=False,training_eligible=False)


def switch_window(assembly,checkpoint,output,next_plant_id,target_name,*,near_trusses_metadata=None):
    """Build new immutable asset; caller performs native startup and rail travel."""
    from native_route_checkpoint import load_checkpoint
    assembly,checkpoint,output=map(lambda p:Path(p).resolve(),(assembly,checkpoint,output))
    source=assembly/'scene.usda';manifest=json.loads((assembly/'motion_manifest.json').read_text())
    if sha(source)!=manifest['scene_sha256'] or not manifest.get('rail'):raise ValueError('Immutable physical rail source/hash required')
    from compact_motion_assets import verify_compact_dependencies,property_inventory,compare_geometry
    verify_compact_dependencies(assembly,manifest)
    metadata,arrays=load_checkpoint(checkpoint,expected_scene_sha256=manifest['scene_sha256'])
    if Path(metadata.get('source_assembly','')).resolve()!=assembly or metadata.get('manifest')!=manifest:
        raise ValueError('Route window requires the exact checkpoint parent assembly/manifest, including materialized dependency SHA')
    from resume_native_state import require_completed_checkpoint
    require_completed_checkpoint(metadata)
    validate_snapshot(metadata,arrays,manifest)
    from robot_controller_state import resolve_robot_controller_state,rebase_rail_controller_state
    controller_state=rebase_rail_controller_state(
        resolve_robot_controller_state(metadata,arrays,checkpoint),arrays,manifest['rail']['dof_name'])
    if controller_state['position_targets'] is None:
        raise ValueError('Route continuation requires proven original controller position targets')
    near_metadata=None;near_provenance=None
    if near_trusses_metadata is not None:
        near_path=Path(near_trusses_metadata).resolve()
        near_metadata=json.loads(near_path.read_text())
        near_provenance=dict(path=str(near_path),sha256=sha(near_path),
            policy='Exact root/gutter near-side stem and fruit-centroid selection; not a reachability certificate')
    next_path=f'/World/Greenhouse/Plants/Plant_{int(next_plant_id):02d}'
    if next_path==manifest['plant']['source_plant']:raise ValueError('Use continuous current-plant runner; do not reset its fruit inventory')
    original=Usd.Stage.Open(str(source));require_motion_units(original)
    stage=Usd.Stage.Open(original.Flatten());next_prim=stage.GetPrimAtPath(next_path)
    if not next_prim or not next_prim.IsActive():raise ValueError('Next source plant unavailable or already visited')
    output.mkdir(parents=True,exist_ok=False)
    robot=author_robot_state(stage,manifest,arrays,controller_state)
    background_state=author_background_state(stage,manifest,arrays,next_path)
    previous_dynamic=restore_prior_detached(stage,manifest,metadata,arrays)
    outgoing=freeze_outgoing(stage,assembly,manifest,metadata,arrays)
    outgoing['checkpoint']=str(checkpoint)
    attachment=copy.deepcopy(manifest.get('fruit_attachment'))
    foliage=copy.deepcopy(manifest.get('compliant_foliage'))
    leaf_proxy=copy.deepcopy(manifest.get('leaf_collision_geometry'))
    # Same planning/GLB asset helper path used by motion_assets.build; calling
    # only build_plant avoids its full-build mount/tool side effects.
    if str(REPO/'mujoco-benchmark/scripts') not in sys.path:
        sys.path.insert(0,str(REPO/'mujoco-benchmark/scripts'))
    build_plant(stage,next_path,manifest,output)
    if attachment:
        selected_names=None
        if near_metadata is not None:
            from row_harvest_targets import scoped_trusses,eligible_truss_fruits
            scope=scoped_trusses(manifest,near_metadata)
            selected_names=[f['name'] for f in eligible_truss_fruits(manifest,scope)]
            if target_name not in selected_names:raise ValueError('Initial route target is excluded by exact near-side attachment scope')
            near_provenance.update(scope=scope)
        manifest['fruit_attachment']=_apply_attachment_template(stage,manifest,attachment,
            selected_names=selected_names,selection_provenance=near_provenance)
    if foliage:
        from compliant_foliage import convert
        first=foliage['leaves'][0]
        new_foliage=convert(stage,stiffness=first['stiffness'],damping=first['damping'],armature=foliage['numerical_armature_kg_m2'])
        manifest['compliant_foliage']=new_foliage;manifest['plant']['bodies'].extend(new_foliage['leaves'])
    if leaf_proxy and leaf_proxy.get('enabled'):
        from leaf_collision_geometry import apply_refinement
        selection=None if leaf_proxy['selection']=='all_compliant_leaves' else leaf_proxy['leaf_source_names']
        manifest['leaf_collision_geometry']=apply_refinement(stage,manifest,selection)
        manifest['leaf_collision_geometry'].update(source_window_rule=leaf_proxy,
            scope='Same explicit compliant-leaf collision rule applied to the next active window; native cooking remains pending')
    target=select_build_target(manifest['plant']['fruits'],target_name);manifest['target']=target
    prefix=target['name'].split('_Fruit_')[0]+'_Fruit_'
    selected=sorted([f for f in manifest['plant']['fruits'] if f['name'].startswith(prefix)],key=lambda f:f['center'][2],reverse=True)[:2]
    manifest['grasp_reference']=grasp_reference(manifest,[f['name'] for f in selected],manifest['grasp_reference'].get('height_offset_m',0.))
    history=manifest.get('route_checkpoint',{}).get('outgoing_history',[])
    manifest['route_checkpoint']=dict(schema='s300-active-window-checkpoint-v1',source_assembly=str(assembly),
        source_scene_sha256=sha(source),checkpoint=str(checkpoint),checkpoint_time_s=metadata['time_s'],
        robot=robot,outgoing=outgoing,outgoing_history=history+[outgoing],new_source_plant=next_path,
        prior_detached_state_preservation=previous_dynamic,
        passive_background_state_preservation=background_state,
        restart='Native engine load boundary; same measured world cart state; actual subsequent travel uses the rail joint',
        native_initialization_validation='pending',training_eligible=False,harvest_success=None)
    manifest['route_checkpoint']['transmission_zero_sign_canonicalization']=_canonicalize_transmission_zero_signs(stage)
    # Reuse lossless compact storage; stale source composed.usdc provenance must
    # never survive in the next window's runtime dependency record.
    before=property_inventory(stage)
    flat=stage.Flatten(addSourceFileComment=False)
    if flat.subLayerPaths or any(flat.GetExternalReferences()):raise ValueError('Window retained external USD dependencies')
    flat.Export(str(output/'composed.usdc'))
    wrapper=Usd.Stage.CreateNew(str(output/'scene.usda'))
    wrapper.GetRootLayer().subLayerPaths=['composed.usdc'];copy_stage_metrics(stage,wrapper)
    wrapper.SetDefaultPrim(wrapper.GetPrimAtPath(stage.GetDefaultPrim().GetPath()))
    wrapper.GetRootLayer().Save()
    after=property_inventory(wrapper)
    if before!=after:
        (output/'checkpoint_materialization_property_differences.json').write_text(json.dumps(
            dict(source_inventory=before,derived_inventory=after,differences=_property_differences(stage,wrapper)),indent=2)+'\n')
        raise ValueError('Window USDC materialization changed effective properties; see checkpoint_materialization_property_differences.json')
    geometry=compare_geometry(stage,wrapper)
    (output/'checkpoint_geometry_inventory.json').write_text(json.dumps(geometry,indent=2,allow_nan=False)+'\n')
    manifest['compact_assets']=dict(schema='s300-lossless-materialized-usdc-v1',source_assembly=str(assembly),
        source_scene_sha256=sha(source),source_manifest_sha256=sha(assembly/'motion_manifest.json'),
        materialized_dependency=dict(relative_path='composed.usdc',sha256=sha(output/'composed.usdc'),
            format='usdc',materialized=True,external_usd_sublayers=[]),
        geometry_unchanged=True,native_validation='pending',
        note='Storage is lossless relative to the derived active-window stage; source-to-window dynamic scope and measured poses changed explicitly.')
    if sha(source)!=manifest['scene_sha256']:raise RuntimeError('Source was changed during checkpoint switch')
    manifest['scene_sha256']=sha(output/'scene.usda');manifest.update(training_eligible=False,harvest_success=None)
    (output/'motion_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return manifest['route_checkpoint']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('assembly',type=Path);p.add_argument('checkpoint',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--next-plant',type=int,required=True);p.add_argument('--target',required=True)
    p.add_argument('--near-trusses-metadata',type=Path,help='Opt-in exact rail-side attachment scope; excluded obstacle trusses retain original joints')
    a=p.parse_args();print(json.dumps(switch_window(a.assembly,a.checkpoint,a.output,a.next_plant,a.target,
        near_trusses_metadata=a.near_trusses_metadata),indent=2))


if __name__=='__main__':main()
