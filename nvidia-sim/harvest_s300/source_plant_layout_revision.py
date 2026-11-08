"""Explicit immutable layout repair of two uninitialized source plants.

Only source-plant Xform attributes change. Active plant/robot/native state,
mesh shapes, collision geometry, joints, material and all other authored fields
remain exact. This is a different initial layout, not a resumed old physics run.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf,Sdf,Usd,UsdGeom

from compact_motion_assets import compact_assembly,file_sha256,verify_compact_dependencies
from strict_window_storage import canonical_layer_digest
from motion_assets import geometry

PLANT17='/World/Greenhouse/Plants/Plant_17'
TRUSS21='/World/Greenhouse/Plants/Plant_21/Truss_02'
TRUSS18='/World/Greenhouse/Plants/Plant_18/Truss_02'
SCHEMA='s300-explicit-unvisited-source-layout-revision-v1'


def _world(stage,path):
    return np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T


def _xform_names(prim):
    return [a.GetName() for a in prim.GetAttributes() if a.GetName()=='xformOpOrder' or a.GetName().startswith('xformOp:')]


def _backup_xforms(stage,paths,label):
    backup=Sdf.Layer.CreateAnonymous(label);names={}
    for path in paths:
        prim=stage.GetPrimAtPath(path);Sdf.CreatePrimInLayer(backup,path)
        names[path]=_xform_names(prim)
        for name in names[path]:
            p=Sdf.Path(path).AppendProperty(name)
            if not Sdf.CopySpec(stage.GetRootLayer(),p,backup,p):raise ValueError('Could not preserve transform attribute: '+str(p))
    return backup,names


def _restore_xforms(stage,backup,names):
    for path,attrs in names.items():
        prim=stage.GetPrimAtPath(path)
        for name in _xform_names(prim):prim.RemoveProperty(name)
        for name in attrs:
            p=Sdf.Path(path).AppendProperty(name)
            if not Sdf.CopySpec(backup,p,stage.GetRootLayer(),p):raise ValueError('Could not restore transform attribute: '+str(p))


def _set_world(stage,path,world):
    prim=stage.GetPrimAtPath(path);parent=prim.GetParent()
    parent_world=np.eye(4) if parent.IsPseudoRoot() else _world(stage,str(parent.GetPath()))
    local=np.linalg.inv(parent_world)@world
    UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(local.T.tolist()))


def apply(stage,manifest,*,plant17_translation=(.015,0.,0.),plant21_yaw_deg=15.,expected21pivot=None,
          plant18_yaw_deg=None,expected18pivot=None):
    if manifest.get('layout_revision'):raise ValueError('Refusing to compound an existing layout revision')
    paths=[PLANT17,TRUSS21]+([TRUSS18] if plant18_yaw_deg is not None else [])
    protected={path.split('/Truss_')[0] for path in paths}
    if manifest['plant']['source_plant'] in protected:
        raise ValueError('Only uninitialized source plants may be repaired; active plant is protected')
    shifts=np.asarray(plant17_translation,float)
    if shifts.shape!=(3,) or not np.isfinite(shifts).all() or not np.isfinite(plant21_yaw_deg):
        raise ValueError('Finite rigid source-layout transforms required')
    if plant18_yaw_deg is not None and (not np.isfinite(plant18_yaw_deg) or abs(plant18_yaw_deg)>30):
        raise ValueError('Finite bounded source18 yaw required')
    if np.linalg.norm(shifts)>.05 or abs(plant21_yaw_deg)>30:
        raise ValueError('This bounded source repair supports <=50mm translation and <=30deg yaw')
    frozen={x['source_plant'] for x in manifest.get('route_checkpoint',{}).get('outgoing_history',[])}
    if frozen & protected:raise ValueError('Already visited source plant cannot be repositioned')
    for path in paths:
        p=stage.GetPrimAtPath(path)
        if not p or not p.IsActive() or not p.IsA(UsdGeom.Xform) or p.IsInstanceProxy():
            raise ValueError('Exact active uninitialized source Xform is required: '+path)
    worlds={p:_world(stage,p) for p in paths};pivot=worlds[TRUSS21][:3,3].copy()
    if expected21pivot is not None and not np.allclose(pivot,expected21pivot,rtol=0.,atol=1e-8):
        raise ValueError('Peduncle attachment pivot does not match the reviewed source model')
    deltas={p:np.eye(4) for p in paths};deltas[PLANT17][:3,3]=shifts
    turn=Rotation.from_euler('z',plant21_yaw_deg,degrees=True).as_matrix()
    deltas[TRUSS21][:3,:3]=turn;deltas[TRUSS21][:3,3]=pivot-turn@pivot
    pivot18=None
    if plant18_yaw_deg is not None:
        pivot18=worlds[TRUSS18][:3,3].copy()
        if expected18pivot is not None and not np.allclose(pivot18,expected18pivot,rtol=0.,atol=1e-8):
            raise ValueError('Source18 peduncle attachment pivot does not match reviewed source model')
        turn18=Rotation.from_euler('z',plant18_yaw_deg,degrees=True).as_matrix()
        deltas[TRUSS18][:3,:3]=turn18;deltas[TRUSS18][:3,3]=pivot18-turn18@pivot18
    original={};cache=UsdGeom.XformCache()
    for root in paths:
        for p in Usd.PrimRange(stage.GetPrimAtPath(root),Usd.TraverseInstanceProxies()):
            if p.IsA(UsdGeom.Mesh):
                v,f,color=geometry(p,cache);original[str(p.GetPath())]=(root,v,hashlib.sha256(np.asarray(f).tobytes()).hexdigest())
    before=canonical_layer_digest(stage.GetRootLayer());backup,names=_backup_xforms(stage,paths,'layout_original_xforms.usda')
    for path in paths:_set_world(stage,path,deltas[path]@worlds[path])
    changed,changed_names=_backup_xforms(stage,paths,'layout_revised_xforms.usda')
    max_error=0.;vertices=0;cache=UsdGeom.XformCache()
    for path,(root,old,faces) in original.items():
        new,f,_=geometry(stage.GetPrimAtPath(path),cache)
        expected=old@deltas[root][:3,:3].T+deltas[root][:3,3]
        if new.shape!=old.shape or hashlib.sha256(np.asarray(f).tobytes()).hexdigest()!=faces:
            raise ValueError('Layout repair changed source mesh topology: '+path)
        max_error=max(max_error,float(np.max(np.abs(new-expected))) if len(new) else 0.)
        vertices+=len(new)
    if max_error>1e-9:raise ValueError('Source geometry did not follow the exact rigid placement transform')
    pivot_error=float(np.linalg.norm(_world(stage,TRUSS21)[:3,3]-pivot))
    if pivot_error>1e-9:raise ValueError('Peduncle attachment pivot moved')
    pivot18_error=None if pivot18 is None else float(np.linalg.norm(_world(stage,TRUSS18)[:3,3]-pivot18))
    if pivot18_error is not None and pivot18_error>1e-9:raise ValueError('Source18 peduncle attachment pivot moved')
    _restore_xforms(stage,backup,names);normalized=canonical_layer_digest(stage.GetRootLayer())
    if before!=normalized:raise ValueError('Layout repair changed fields beyond the exact two source Xform opinions')
    _restore_xforms(stage,changed,changed_names)
    return dict(schema=SCHEMA,independent_initial_layout=True,
        source_plant_transforms=[dict(path=p,world_before=worlds[p].tolist(),world_after=_world(stage,p).tolist(),
            rigid_world_delta=deltas[p].tolist()) for p in paths],
        plant17_translation_world_m=shifts.tolist(),plant21_truss02_yaw_world_z_deg=float(plant21_yaw_deg),
        plant21_attachment_pivot_world_m=pivot.tolist(),attachment_pivot_error_m=pivot_error,
        plant18_truss02_yaw_world_z_deg=plant18_yaw_deg,
        plant18_attachment_pivot_world_m=None if pivot18 is None else pivot18.tolist(),
        plant18_attachment_pivot_error_m=pivot18_error,
        mesh_count=len(original),all_world_vertices_checked=vertices,world_vertex_rigid_transform_max_error_m=max_error,
        normalized_source_canonical=before,normalized_derived_canonical=normalized,
        all_other_authored_fields_exact=True,active_plant_robot_geometry_dynamics_and_joints_unchanged=True,
        native_validation='pending',geometry_changes_are_explicit_placement_changes=True,
        current_target_pose_teleported=False,old_physics_replayed=False,training_eligible=False,harvest_success=None)


def layout_inventory(stage,layout,manifest):
    """Reuse the inventory collector with exact measured frozen fruit snapshots.

    The collector accepts a fruit-coordinate snapshot for its non-source active
    plant. Here that same input format represents a verified unharvested frozen
    plant; it does not modify or replace the execution manifest.
    """
    from rail_assets import near_side_inventory
    current=manifest['plant']['source_plant'];frozen={}
    for row in manifest.get('route_checkpoint',{}).get('outgoing_history',[]):frozen[row['source_plant']]=row
    remaining=copy.deepcopy(layout)
    remaining['neighbor_placements']=[p for p in layout['neighbor_placements']
        if f"/World/Greenhouse/Plants/Plant_{p['id']:02d}" not in frozen or f"/World/Greenhouse/Plants/Plant_{p['id']:02d}"==current]
    inventory=near_side_inventory(stage,remaining,active_manifest=manifest);proof=[]
    for source,row in frozen.items():
        if source==current:continue
        match=re.fullmatch(r'/World/Greenhouse/Plants/Plant_(\d+)',source)
        if (not match or row['frozen_root']!=f"/World/RouteFrozen/Plant_{int(match[1]):02d}" or
                row.get('unharvested_reactivation_eligible') is not True or row.get('detached_fruits') or
                row.get('measured_shape_preserved') is not True):
            raise ValueError('Frozen source inventory requires exact unharvested measured shape proof')
        root=stage.GetPrimAtPath(row['frozen_root']);fruits=[];cache=UsdGeom.XformCache()
        if not root:raise ValueError('Missing frozen source plant for inventory')
        for body in root.GetChildren():
            if not re.fullmatch(r'Truss_\d+_Fruit_\d+',body.GetName()):continue
            collider=stage.GetPrimAtPath(str(body.GetPath())+'/Collider')
            if not collider or not collider.IsA(UsdGeom.Mesh):raise ValueError('Missing exact frozen source fruit mesh')
            v,_,_=geometry(collider,cache)
            fruits.append(dict(name=body.GetName(),center=((v.min(0)+v.max(0))/2).tolist()))
        one=copy.deepcopy(layout);one['neighbor_placements']=[p for p in layout['neighbor_placements'] if p['id']==int(match[1])]
        if len(one['neighbor_placements'])!=1 or not fruits:raise ValueError('Frozen source is missing actual inventory geometry')
        snapshot=dict(plant=dict(source_plant=source,fruits=fruits))
        inventory['trusses'].extend(near_side_inventory(stage,one,active_manifest=snapshot)['trusses'])
        proof.append(dict(source_plant=source,frozen_root=row['frozen_root'],fruit_count=len(fruits),
            checkpoint_metadata_sha256=row['checkpoint_metadata_sha256'],coordinates_from_current_frozen_usd=True,
            current_execution_manifest_changed=False))
    inventory['trusses'].sort(key=lambda row:(row['plant_id'],row['truss_index']))
    inventory['total_trusses']=len(inventory['trusses']);inventory['side_selected_trusses']=sum(r['rail_near'] for r in inventory['trusses'])
    inventory['side_excluded_trusses']=sum(not r['rail_near'] for r in inventory['trusses'])
    inventory['frozen_source_snapshot_provenance']=proof
    return inventory


def revise(source,output,*,expected21pivot=None,plant17_translation=(.015,0.,0.),
           plant18_yaw_deg=None,expected18pivot=None):
    source,output=Path(source).resolve(),Path(output).resolve()
    source_hashes={n:file_sha256(source/n) for n in ['scene.usda','composed.usdc','motion_manifest.json','skin.npz']}
    verify_compact_dependencies(source);compact_assembly(source,output)
    manifest=json.loads((output/'motion_manifest.json').read_text());stage=Usd.Stage.Open(str(output/'composed.usdc'))
    report=apply(stage,manifest,expected21pivot=expected21pivot,plant17_translation=plant17_translation,
                 plant18_yaw_deg=plant18_yaw_deg,expected18pivot=expected18pivot)
    stage.GetRootLayer().Save();manifest['layout_revision']=report
    manifest['compact_assets']['geometry_unchanged']=False
    manifest['compact_assets']['materialized_dependency']['sha256']=file_sha256(output/'composed.usdc')
    manifest['compact_assets']['note']=('Explicit new source-plant layout. All non-transform fields exact; '
        'source17 world translation '+repr(report['plant17_translation_world_m'])+
        'm, source21 Truss02 world-Z yaw+15deg, source18 optional yaw '+repr(plant18_yaw_deg)+'deg. New native validation required.')
    manifest['compact_assets']['native_validation']='pending explicit new layout validation'
    manifest.update(training_eligible=False,harvest_success=None)
    report.update(source_assembly=str(source),source_sha256=source_hashes)
    (output/'motion_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    (output/'layout_revision.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    # Reuse the existing inventory collector with its exact declared root input
    # corrected by the same explicit source translation. Fruit centers come
    # from the resulting USD meshes, never from the old placement centroids.
    from rail_assets import LAYOUT
    layout=json.loads(Path(LAYOUT).read_text());layout=copy.deepcopy(layout)
    plant=next(x for x in layout['neighbor_placements'] if x['id']==17)
    plant['position']=(np.asarray(plant['position'])+np.asarray(report['plant17_translation_world_m'])).tolist()
    inventory=layout_inventory(stage,layout,manifest)
    inventory['layout_revision']=dict(schema=SCHEMA,source_assembly=str(output),
        scene_sha256=manifest['scene_sha256'],materialized_sha256=manifest['compact_assets']['materialized_dependency']['sha256'],
        source17_root_translation_world_m=report['plant17_translation_world_m'],
        source21_truss02_world_z_yaw_delta_deg=report['plant21_truss02_yaw_world_z_deg'],
        source18_truss02_world_z_yaw_delta_deg=report['plant18_truss02_yaw_world_z_deg'],
        original_y_deg_field_is_pre_revision=True,fruit_centers_recomputed_from_actual_usd=True)
    (output/'full_farm_inventory.json').write_text(json.dumps(inventory,indent=2,allow_nan=False)+'\n')
    (output/'layout_source_metadata.json').write_text(json.dumps(layout,indent=2,allow_nan=False)+'\n')
    verify_compact_dependencies(output,manifest)
    if {n:file_sha256(source/n) for n in source_hashes}!=source_hashes:raise ValueError('Original source was modified')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--expected21pivot',nargs=3,type=float)
    p.add_argument('--plant17-world-x-mm',type=float,default=15.,help='Explicit bounded new source placement; original root Z is retained')
    p.add_argument('--plant18-world-yaw-deg',type=float)
    p.add_argument('--expected18pivot',nargs=3,type=float)
    a=p.parse_args();r=revise(a.source,a.output,expected21pivot=a.expected21pivot,
                            plant17_translation=(a.plant17_world_x_mm/1000,0.,0.),
                            plant18_yaw_deg=a.plant18_world_yaw_deg,expected18pivot=a.expected18pivot)
    print(json.dumps({k:v for k,v in r.items() if k!='source_plant_transforms'},indent=2))
