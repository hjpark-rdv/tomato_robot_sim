"""Immutable physical rail placement correction above the original farm floor.

Only the complete robot/tool rest placement and the world-fixed anchor frame
move. CAD, farm geometry, contacts, margins, materials and joint parameters are
preserved. Authored clearance is diagnostic; native initialization must verify
the cooked shapes and the existing 0.5 mm physical-validity criterion.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from compact_motion_assets import (file_sha256, property_inventory,
                                  verify_compact_dependencies, _value_digest)
from motion_stage_metrics import copy_stage_metrics, require_motion_units, stage_metrics
from rail_assets import BASE_BODY, RAIL_ANCHOR, RAIL_FIXED_ROOT
from rail_chassis_body import CARRIER, CHASSIS

HARVEST_ROBOT = '/World/HarvestRobot'
TOOL_ROOT = '/World/ActuatedTool'
TOOL_BODIES = tuple(TOOL_ROOT+'/'+name for name in ('fixed','L','R','L_belt','R_belt'))
GROUND_PATHS = ('/World/Greenhouse/House/Environment/GroundCover_1/Mesh',
                '/World/Greenhouse/House/Environment/GroundCover_1_2/Mesh')
WHEEL_PATHS = tuple(CHASSIS+'/Part_%04d'%number for number in (8,26,44,62))


def _world(stage, path):
    return np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T


def _translate_world(stage, path, delta):
    prim=stage.GetPrimAtPath(path);xf=UsdGeom.Xformable(prim)
    if not prim or not xf:raise ValueError('Missing exact Xform placement root: '+path)
    cache=UsdGeom.XformCache();world=np.asarray(cache.GetLocalToWorldTransform(prim)).T
    parent=np.eye(4) if xf.GetResetXformStack() else np.asarray(cache.GetLocalToWorldTransform(prim.GetParent())).T
    world[:3,3]+=delta
    xf.MakeMatrixXform().Set(Gf.Matrix4d((np.linalg.inv(parent)@world).T.tolist()))


def align_ground(stage, *, lift_m=.003):
    """Translate a caller-owned carrier assembly; preserve all joint-relative frames."""
    if not np.isfinite(lift_m) or not 0.<lift_m<=.02:
        raise ValueError('Explicit ground placement lift must be positive and <=20 mm')
    carrier=stage.GetPrimAtPath(CARRIER);anchor=stage.GetPrimAtPath(RAIL_ANCHOR)
    joint=UsdPhysics.FixedJoint(stage.GetPrimAtPath(RAIL_FIXED_ROOT))
    if not carrier or not carrier.HasAPI(UsdPhysics.RigidBodyAPI) or not anchor or not joint:
        raise ValueError('Ground placement requires the separate physical carrier and rail anchor')
    if joint.GetBody0Rel().GetTargets() or list(map(str,joint.GetBody1Rel().GetTargets()))!=[RAIL_ANCHOR]:
        raise ValueError('Expected exact world-to-rail-anchor fixed constraint')
    if not all(stage.GetPrimAtPath(p).HasAPI(UsdPhysics.RigidBodyAPI) for p in TOOL_BODIES):
        raise ValueError('Missing original five physical tool bodies')
    delta=np.array([0.,0.,float(lift_m)]);old_anchor=np.array(joint.GetLocalPos0Attr().Get(),float)
    before_base=_world(stage,BASE_BODY);before_robot=_world(stage,HARVEST_ROBOT)
    _translate_world(stage,HARVEST_ROBOT,delta)
    # The tool's typeless grouping prim cannot supply a USD transform. Move
    # its five existing Xform roots, keeping joint-relative rest frames exact.
    for path in TOOL_BODIES:_translate_world(stage,path,delta)
    joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,old_anchor+delta)))
    native_anchor=np.array(joint.GetLocalPos0Attr().Get(),float)
    return dict(schema='s300-physical-ground-placement-v1',translation_world_m=delta.tolist(),
        moved_xform_roots=[HARVEST_ROBOT,*TOOL_BODIES],world_fixed_joint=RAIL_FIXED_ROOT,
        original_world_fixed_local_pos0_m=old_anchor.tolist(),world_fixed_local_pos0_m=native_anchor.tolist(),
        world_fixed_frame_float32_rounding_m=float(np.max(np.abs(native_anchor-old_anchor-delta))),
        source_base_position_world_m=before_base[:3,3].tolist(),
        placed_base_position_world_m=_world(stage,BASE_BODY)[:3,3].tolist(),
        source_robot_root_position_world_m=before_robot[:3,3].tolist(),
        placed_robot_root_position_world_m=_world(stage,HARVEST_ROBOT)[:3,3].tolist(),
        geometry_or_contact_parameters_changed=False,training_eligible=False,harvest_success=None,
        native_validation='pending cooked-shape initialization and actual rail motion')


def _moved(path):
    return path==HARVEST_ROBOT or path.startswith(HARVEST_ROBOT+'/') or any(
        path==root or path.startswith(root+'/') for root in TOOL_BODIES)


def placement_geometry(source,placed,delta):
    old={str(p.GetPath()):p for p in Usd.PrimRange.Stage(source,Usd.TraverseInstanceProxies())}
    new={str(p.GetPath()):p for p in Usd.PrimRange.Stage(placed,Usd.TraverseInstanceProxies())}
    if set(old)!=set(new):raise RuntimeError('Ground placement changed effective prim topology')
    a=UsdGeom.XformCache();b=UsdGeom.XformCache();count=vertices=colliders=rigids=0
    max_vertex=max_pose=max_static=0.;digest=hashlib.sha256()
    moved_roots={HARVEST_ROBOT,*TOOL_BODIES}
    for path,prim in old.items():
        other=new[path]
        if prim.GetTypeName()!=other.GetTypeName() or prim.GetAppliedSchemas()!=other.GetAppliedSchemas():
            raise RuntimeError('Ground placement changed original prim/schema: '+path)
        left={p.GetName():_value_digest(p.Get()) for p in prim.GetAttributes()}
        right={p.GetName():_value_digest(p.Get()) for p in other.GetAttributes()}
        allowed={'xformOp:transform','xformOpOrder'} if path in moved_roots else set()
        if path==RAIL_FIXED_ROOT:allowed.add('physics:localPos0')
        if {k:v for k,v in left.items() if k not in allowed}!={k:v for k,v in right.items() if k not in allowed}:
            raise RuntimeError('Ground placement changed physical/visual properties: '+path)
        if {r.GetName():list(map(str,r.GetTargets())) for r in prim.GetRelationships()}!={
                r.GetName():list(map(str,r.GetTargets())) for r in other.GetRelationships()}:
            raise RuntimeError('Ground placement changed a relationship: '+path)
        t=np.asarray(a.GetLocalToWorldTransform(prim)).T;u=np.asarray(b.GetLocalToWorldTransform(other)).T
        expected=t.copy()
        if _moved(path):expected[:3,3]+=delta
        error=float(np.max(np.abs(u-expected)))
        if prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.RigidBodyAPI):max_pose=max(max_pose,error)
        if not _moved(path):max_static=max(max_static,error)
        if prim.IsA(UsdGeom.Mesh):
            points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),float)
            world=points@t[:3,:3].T+t[:3,3]
            actual=np.asarray(UsdGeom.Mesh(other).GetPointsAttr().Get(),float)@u[:3,:3].T+u[:3,3]
            expected_vertices=world+delta if _moved(path) else world
            if world.size:max_vertex=max(max_vertex,float(np.max(np.abs(expected_vertices-actual))))
            if not np.isfinite(actual).all():raise RuntimeError('Nonfinite translated geometry')
            digest.update(path.encode());digest.update(np.ascontiguousarray(world).tobytes())
            count+=1;vertices+=len(points)
        colliders+=prim.HasAPI(UsdPhysics.CollisionAPI);rigids+=prim.HasAPI(UsdPhysics.RigidBodyAPI)
    if max(max_vertex,max_pose)>1e-8 or max_static!=0.:
        raise RuntimeError('Robot placement changed geometry beyond the declared translation')
    return dict(mesh_count=count,world_vertices=vertices,collider_count=colliders,rigid_body_count=rigids,
        translated_world_vertex_max_abs_error_m=max_vertex,translated_rigid_or_collider_pose_max_abs_error=max_pose,
        unchanged_farm_target_world_pose_max_abs_error=max_static,source_geometry_sha256=digest.hexdigest(),
        geometry_and_joint_properties_preserved=True)


def ground_clearance(stage):
    cache=UsdGeom.XformCache();rows=[]
    for path in (*GROUND_PATHS,*WHEEL_PATHS):
        prim=stage.GetPrimAtPath(path)
        if not prim or not prim.IsA(UsdGeom.Mesh):continue
        points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),float)
        tf=np.asarray(cache.GetLocalToWorldTransform(prim)).T;world=points@tf[:3,:3].T+tf[:3,3]
        rows.append(dict(path=path,bounds_world_m=[world.min(0).tolist(),world.max(0).tolist()]))
    ground=[r for r in rows if r['path'] in GROUND_PATHS];wheels=[r for r in rows if r['path'] in WHEEL_PATHS]
    clearance=min(r['bounds_world_m'][0][2] for r in wheels)-max(r['bounds_world_m'][1][2] for r in ground) if ground and wheels else None
    return dict(source_mesh_world_bounds=rows,wheel_bottom_to_highest_ground_top_m=clearance,
        scope='Authored exact mesh bounds; native cooked convex/triangle contacts must be checked separately')


def revise(assembly,output,*,lift_m=.003):
    assembly,output=Path(assembly).resolve(),Path(output).resolve()
    manifest=json.loads((assembly/'motion_manifest.json').read_text());digest=file_sha256(assembly/'scene.usda')
    if digest!=manifest['scene_sha256']:raise ValueError('Source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    if output.exists() or manifest.get('rail_ground_alignment'):raise ValueError('Use original placement and immutable new output')
    original=Usd.Stage.Open(str(assembly/'scene.usda'));require_motion_units(original)
    stage=Usd.Stage.Open(original.Flatten(addSourceFileComment=False));copy_stage_metrics(original,stage)
    report=align_ground(stage,lift_m=lift_m);report['before']=ground_clearance(original);report['after']=ground_clearance(stage)
    properties=property_inventory(stage);output.mkdir(parents=True,exist_ok=False)
    layer=stage.Flatten(addSourceFileComment=False)
    if layer.subLayerPaths or any(layer.GetExternalReferences()):raise RuntimeError('Ground placement must be fully materialized')
    layer.Export(str(output/'composed.usdc'))
    wrapper=Usd.Stage.CreateNew(str(output/'scene.usda'));wrapper.GetRootLayer().subLayerPaths=['composed.usdc']
    copy_stage_metrics(original,wrapper)
    if original.GetDefaultPrim():wrapper.SetDefaultPrim(wrapper.GetPrimAtPath(original.GetDefaultPrim().GetPath()))
    if original.HasAuthoredTimeCodeRange():
        wrapper.SetStartTimeCode(original.GetStartTimeCode());wrapper.SetEndTimeCode(original.GetEndTimeCode())
    wrapper.GetRootLayer().Save()
    if stage_metrics(original)!=stage_metrics(wrapper) or property_inventory(wrapper)!=properties:
        raise RuntimeError('Ground placement export changed effective properties or units')
    report['geometry']=placement_geometry(original,wrapper,np.array(report['translation_world_m']))
    report.update(source_assembly=str(assembly),source_scene_sha256=digest)
    shutil.copy2(assembly/'skin.npz',output/'skin.npz')
    manifest['rail']['source_base_position_world_m']=report['placed_base_position_world_m']
    manifest['rail']['source_robot_root_position_world_m']=report['placed_robot_root_position_world_m']
    manifest['rail']['native_validation']='pending corrected ground placement/cooked-shape contacts'
    manifest['rail_ground_alignment']=report;manifest['scene_sha256']=file_sha256(output/'scene.usda')
    manifest['compact_assets']=dict(schema='s300-lossless-materialized-usdc-v1',source_assembly=str(assembly),
        source_scene_sha256=digest,source_manifest_sha256=file_sha256(assembly/'motion_manifest.json'),
        materialized_dependency=dict(relative_path='composed.usdc',sha256=file_sha256(output/'composed.usdc'),
            format='usdc',materialized=True,external_usd_sublayers=[]),geometry_unchanged=True,
        native_validation='pending',note='Materialized export; full robot/tool placement correction is separately recorded')
    manifest['training_eligible']=False
    (output/'motion_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    (output/'rail_ground_alignment.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if file_sha256(assembly/'scene.usda')!=digest:raise RuntimeError('Original placement changed')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('assembly',type=Path)
    parser.add_argument('output',type=Path);parser.add_argument('--lift-mm',type=float,default=3.)
    args=parser.parse_args();print(json.dumps(revise(args.assembly,args.output,lift_m=args.lift_mm/1000),indent=2))
