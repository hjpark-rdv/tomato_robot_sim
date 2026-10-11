"""Immutable opt-in leaf collider refinement, retaining every authored triangle.

Only exact enabled compliant-leaf Mesh collider approximation tokens change
from convexHull to convexDecomposition. This is a declared physical collision
representation revision, not a detector override or lossless physics claim.
Authored surfaces, bodies, joints, masks, materials and tolerance stay intact.
Native decomposition coverage/contact/runtime validation remains required.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil
import time

from pxr import Usd,UsdGeom,UsdPhysics

from compact_motion_assets import (compare_geometry,file_sha256,property_inventory,
                                   verify_compact_dependencies)
from motion_stage_metrics import copy_stage_metrics,require_motion_units,stage_metrics


def apply_refinement(stage,manifest,leaf_names=None):
    """Apply the exact rule to a caller-owned derived stage, including route windows.

    Names may be exact manifest leaf names or exact source names. An omitted
    selection means all compliant leaves; unmatched requested names fail closed.
    The caller owns this stage and must never pass its immutable source stage.
    """
    leaves=manifest.get('compliant_foliage',{}).get('leaves',[])
    if not leaves:raise ValueError('Explicit compliant leaf inventory required')
    aliases={}
    for row in leaves:
        for name in (row.get('name'),row.get('source_name')):
            if not name:continue
            if name in aliases and aliases[name] is not row:raise ValueError('Ambiguous exact leaf identity')
            aliases[name]=row
    requested=list(leaf_names) if leaf_names is not None else None
    if requested is not None and (not requested or len(set(requested))!=len(requested) or any(name not in aliases for name in requested)):
        raise ValueError('Unknown, empty or duplicate exact compliant leaf selection')
    selected=leaves if requested is None else [aliases[name] for name in requested]
    if len({row['path'] for row in selected})!=len(selected):raise ValueError('Duplicate selected leaf owner')
    paths=[];unchanged=[];changes=[]
    # Validate the entire selected inventory before authoring any token.
    for row in selected:
        body=stage.GetPrimAtPath(row['path'])
        if not body or not body.HasAPI(UsdPhysics.RigidBodyAPI):raise ValueError('Selected leaf has no exact rigid owner')
        colliders=[]
        for path in row.get('meshes',[]):
            prim=stage.GetPrimAtPath(path)
            if not prim or not path.startswith(row['path']+'/'):raise ValueError('Leaf mesh owner/path mismatch')
            if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True:continue
            if not prim.IsA(UsdGeom.Mesh):raise ValueError('Selected leaf collider is not an authored Mesh')
            if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):raise ValueError('Selected leaf collider has no exact mesh approximation API')
            approximation=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            if approximation not in ('convexHull','convexDecomposition'):
                raise ValueError('Unsupported selected leaf approximation: '+str(approximation))
            if path in paths:raise ValueError('Duplicate selected collider path')
            paths.append(path);colliders.append(path)
            if approximation=='convexHull':
                changes.append(dict(path=path,leaf_name=row['name'],source_name=row.get('source_name'),
                                    before='convexHull',after='convexDecomposition'))
            else:unchanged.append(path)
        if not colliders:raise ValueError('Selected compliant leaf has no enabled Mesh collider')
    before=property_inventory(stage)
    for row in changes:
        UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(row['path'])).GetApproximationAttr().Set(row['after'])
    # Reuse the established resolved-property audit. Temporarily restore ONLY
    # the permitted tokens on this derived stage, then verify every remaining
    # property and relationship against the source inventory.
    try:
        for row in changes:UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(row['path'])).GetApproximationAttr().Set(row['before'])
        restored=property_inventory(stage)
        if restored!=before:raise ValueError('Unexpected property change beyond leaf collider approximation')
    finally:
        for row in changes:UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(row['path'])).GetApproximationAttr().Set(row['after'])
    result=dict(schema='s300-compliant-leaf-collision-refinement-v1',enabled=True,
                approximation='convexDecomposition',selection='all_compliant_leaves' if requested is None else 'exact_subset',
                leaf_names=[row['name'] for row in selected],
                leaf_source_names=[row.get('source_name') or row['name'] for row in selected],
                selected_leaf_count=len(selected),selected_collider_count=len(paths),
                changed_collider_count=len(changes),collider_changes=changes,already_refined_colliders=unchanged,
                authored_triangles_retained=True,non_approximation_properties_preserved=True,
                native_cooking_parameters='Existing defaults; no authored cooking parameters changed',
                native_validation='pending',training_eligible=False,harvest_success=None,
                note='Declared native collider approximation change; all authored geometry/rigid/joint/material/filter values retained. Native hull coverage/runtime require validation.')
    return result


def revise(assembly,output,leaf_names=None):
    """Create a fresh materialized USDC assembly; never mutate or overwrite source."""
    started=time.perf_counter();assembly=Path(assembly).resolve();output=Path(output).resolve()
    source=assembly/'scene.usda';manifest_path=assembly/'motion_manifest.json'
    manifest=json.loads(manifest_path.read_text());source_sha=file_sha256(source)
    if source_sha!=manifest.get('scene_sha256'):raise ValueError('Source assembly scene SHA mismatch')
    dependency=verify_compact_dependencies(assembly,manifest)
    source_files={str(path):file_sha256(path) for path in (source,manifest_path,assembly/'skin.npz')}
    if dependency:
        path=assembly/dependency['relative_path'];source_files[str(path)]=file_sha256(path)
    original=Usd.Stage.Open(str(source));require_motion_units(original)
    # Flatten into a new anonymous layer before any writes. The source Sdf layer
    # cache and disk files remain untouched, even for already compact assets.
    stage=Usd.Stage.Open(original.Flatten(addSourceFileComment=False))
    report=apply_refinement(stage,manifest,leaf_names)
    geometry=compare_geometry(original,stage)
    report.update(source_assembly=str(assembly),source_scene_sha256=source_sha,
                  source_manifest_sha256=source_files[str(manifest_path)],source_file_sha256=source_files,
                  geometry_audit={key:value for key,value in geometry.items() if key not in ('colliders','rigid_bodies')},
                  physical_collision_representation_changed=bool(report['changed_collider_count']))
    output.mkdir(parents=True,exist_ok=False)
    materialized=output/'composed.usdc';layer=stage.Flatten(addSourceFileComment=False)
    if layer.subLayerPaths or any(layer.GetExternalReferences()):raise ValueError('Refined materialized layer retained external USD dependencies')
    layer.Export(str(materialized))
    wrapper=Usd.Stage.CreateNew(str(output/'scene.usda'))
    wrapper.GetRootLayer().subLayerPaths=['composed.usdc'];copy_stage_metrics(stage,wrapper)
    if stage.HasAuthoredTimeCodeRange():
        wrapper.SetStartTimeCode(stage.GetStartTimeCode());wrapper.SetEndTimeCode(stage.GetEndTimeCode())
    if stage.GetDefaultPrim():wrapper.SetDefaultPrim(wrapper.GetPrimAtPath(stage.GetDefaultPrim().GetPath()))
    wrapper.GetRootLayer().Save();require_motion_units(wrapper)
    if stage_metrics(wrapper)!=stage_metrics(stage):raise ValueError('Refinement changed stage units')
    if property_inventory(stage)!=property_inventory(wrapper):raise ValueError('Refined USDC serialization changed resolved properties')
    compare_geometry(stage,wrapper)
    shutil.copy2(assembly/'skin.npz',output/'skin.npz')
    fresh_dependency=dict(relative_path='composed.usdc',sha256=file_sha256(materialized),format='usdc',
                          materialized=True,external_usd_sublayers=[])
    manifest=copy.deepcopy(manifest);manifest['scene_sha256']=file_sha256(output/'scene.usda')
    manifest['leaf_collision_geometry']=report
    manifest['compact_assets']=dict(schema='s300-lossless-materialized-usdc-v1',source_assembly=str(assembly),
        source_scene_sha256=source_sha,source_manifest_sha256=source_files[str(manifest_path)],
        materialized_dependency=fresh_dependency,geometry_unchanged=True,native_validation='pending',
        note='Storage preserves the derived stage exactly; leaf collision approximation was revised explicitly.')
    manifest.update(training_eligible=False,harvest_success=None)
    (output/'motion_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    report['elapsed_s']=time.perf_counter()-started
    (output/'leaf_collision_geometry_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    for path,digest in source_files.items():
        if file_sha256(path)!=digest:raise ValueError('Source asset changed during leaf collision refinement')
    verify_compact_dependencies(output)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('assembly',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--leaves',nargs='+',help='Exact manifest leaf names or source names; omitted means all compliant leaf colliders')
    args=parser.parse_args();print(json.dumps(revise(args.assembly,args.output,args.leaves),ensure_ascii=False,indent=2))
