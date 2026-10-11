"""Lossless materialized USDC storage for an immutable motion assembly.

The public scene.usda remains the normal runner entry point. Its sole sublayer
contains the fully composed source stage, including every physical and visual
spec. Geometry equivalence does not imply native PhysX runtime equivalence;
the resulting asset explicitly remains pending native validation.
"""
import argparse
import hashlib
import json
import shutil
import time
from pathlib import Path
import numpy as np
from pxr import Sdf,Usd,UsdGeom,UsdPhysics
from motion_stage_metrics import copy_stage_metrics,require_motion_units,stage_metrics


def file_sha256(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def _value_digest(value):
    if value is None:return 'none'
    if isinstance(value,Sdf.AssetPath):
        return 'asset:'+str(value.resolvedPath or value.path)
    try:
        array=np.asarray(value)
        if array.dtype.kind in 'biufc' and array.dtype!=object:
            return str((array.dtype.str,array.shape))+':'+hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()
    except (TypeError,ValueError):pass
    return str(value)


def _prims(stage):
    return list(Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()))


def property_inventory(stage):
    """Resolved properties and relationship targets, independent of layer stack."""
    digest=hashlib.sha256();prim_count=attribute_count=relationship_count=0
    for prim in _prims(stage):
        prim_count+=1
        fields=[str(prim.GetPath()),prim.GetTypeName(),str(prim.IsActive()),
                str(prim.GetAppliedSchemas()),str(prim.GetCustomData())]
        schemas=prim.GetMetadata('apiSchemas')
        if schemas is not None:fields.append(str(schemas.ApplyOperations([])))
        for attr in sorted(prim.GetAttributes(),key=lambda a:a.GetName()):
            attribute_count+=1
            fields += [attr.GetName(),str(attr.GetTypeName()),str(attr.GetVariability()),
                       _value_digest(attr.Get()),str(attr.GetConnections())]
            for stamp in attr.GetTimeSamples():fields += [repr(stamp),_value_digest(attr.Get(stamp))]
            # Primvar interpolation and other semantic property metadata matter.
            for key,value in sorted(attr.GetAllMetadata().items()):
                if key not in ('default','timeSamples','typeName','variability'):
                    fields += [key,_value_digest(value)]
        for rel in sorted(prim.GetRelationships(),key=lambda r:r.GetName()):
            relationship_count+=1;fields += [rel.GetName(),str(rel.GetTargets())]
        digest.update(json.dumps(fields,ensure_ascii=False,separators=(',',':')).encode())
    return dict(prim_count=prim_count,attribute_count=attribute_count,
                relationship_count=relationship_count,sha256=digest.hexdigest())


def compare_geometry(source,compact):
    """Compare every world mesh vertex, topology, collider and rigid-body pose."""
    left={str(p.GetPath()):p for p in _prims(source)};right={str(p.GetPath()):p for p in _prims(compact)}
    if set(left)!=set(right):raise ValueError('Compact asset changed effective prim topology')
    a=UsdGeom.XformCache();b=UsdGeom.XformCache();mesh_count=vertices=rigid_count=collider_count=0
    max_vertex=max_pose=max_collider_pose=0.;geometry_hash=hashlib.sha256();colliders=[];rigids=[]
    for path,prim in left.items():
        other=right[path]
        if prim.GetTypeName()!=other.GetTypeName():raise ValueError('Compact prim type changed: '+path)
        if prim.IsA(UsdGeom.Mesh):
            mesh_count+=1;lm=UsdGeom.Mesh(prim);rm=UsdGeom.Mesh(other)
            points=np.asarray(lm.GetPointsAttr().Get(),dtype=float);new=np.asarray(rm.GetPointsAttr().Get(),dtype=float)
            if points.shape!=new.shape:raise ValueError('Compact mesh vertex count changed: '+path)
            for getter in ('GetFaceVertexCountsAttr','GetFaceVertexIndicesAttr'):
                if not np.array_equal(np.asarray(getattr(lm,getter)().Get()),np.asarray(getattr(rm,getter)().Get())):
                    raise ValueError('Compact mesh topology changed: '+path)
            t=np.asarray(a.GetLocalToWorldTransform(prim)).T;u=np.asarray(b.GetLocalToWorldTransform(other)).T
            world=points@t[:3,:3].T+t[:3,3];next_world=new@u[:3,:3].T+u[:3,3]
            if not np.isfinite(world).all() or not np.isfinite(next_world).all():raise ValueError('Nonfinite compact geometry')
            if world.size:max_vertex=max(max_vertex,float(np.max(np.abs(world-next_world))))
            vertices+=len(points);geometry_hash.update(path.encode());geometry_hash.update(np.ascontiguousarray(world).tobytes())
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            enabled=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            if enabled!=UsdPhysics.CollisionAPI(other).GetCollisionEnabledAttr().Get():raise ValueError('Compact collider enable state changed')
            t=np.asarray(a.GetLocalToWorldTransform(prim));u=np.asarray(b.GetLocalToWorldTransform(other))
            max_collider_pose=max(max_collider_pose,float(np.max(np.abs(t-u))))
            colliders.append(dict(path=path,enabled=enabled,approximation=prim.GetAttribute('physics:approximation').Get(),
                                  world_pose_sha256=_value_digest(t)))
            collider_count+=1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_count+=1;t=np.asarray(a.GetLocalToWorldTransform(prim));u=np.asarray(b.GetLocalToWorldTransform(other))
            max_pose=max(max_pose,float(np.max(np.abs(t-u))));rigids.append(dict(path=path,world_pose_sha256=_value_digest(t)))
    if max_vertex!=0. or max_pose!=0. or max_collider_pose!=0.:raise ValueError('Lossless compact geometry/rest transform differs from source')
    return dict(mesh_count=mesh_count,world_vertices=vertices,world_vertex_max_abs_error_m=max_vertex,
                rigid_body_count=rigid_count,rigid_pose_max_abs_error=max_pose,collider_count=collider_count,
                collider_pose_max_abs_error=max_collider_pose,
                geometry_sha256=geometry_hash.hexdigest(),colliders=colliders,rigid_bodies=rigids)


def verify_compact_dependencies(assembly,manifest=None):
    """Verify the runtime materialized layer; hashing the wrapper alone is insufficient."""
    assembly=Path(assembly)
    manifest=manifest or json.loads((assembly/'motion_manifest.json').read_text())
    spec=manifest.get('compact_assets')
    if spec is None:return None
    dependency=spec['materialized_dependency'];relative=Path(dependency['relative_path'])
    if relative.is_absolute() or '..' in relative.parts:raise ValueError('Compact dependency must be inside its assembly')
    path=assembly/relative
    if not path.is_file() or file_sha256(path)!=dependency['sha256']:
        raise ValueError('Compact materialized dependency SHA mismatch')
    return dependency


def _byte_copy_candidate(source,manifest):
    """Recognize an already materialized, self-contained asset without walking geometry.

    This is a storage identity proof, not a fresh geometry or native-physics
    validation. A wrapper with any prim opinions or unknown composition must
    take the original full compaction path. Hash mismatches are checked by the
    caller and never become fallback eligibility decisions.
    """
    spec=manifest.get('compact_assets') or {}
    dependency=spec.get('materialized_dependency') or {}
    if (spec.get('schema')!='s300-lossless-materialized-usdc-v1'
            or spec.get('geometry_unchanged') is not True
            or dependency.get('relative_path')!='composed.usdc'
            or dependency.get('format')!='usdc'
            or dependency.get('materialized') is not True
            or dependency.get('external_usd_sublayers')!=[]):return None
    scene=source/'scene.usda';materialized=source/'composed.usdc'
    # A symlinked wrapper can resolve relative composition from a different
    # anchor directory. Copying its bytes into an ordinary file must not be
    # mistaken for an identical composition without the full legacy audit.
    if scene.is_symlink() or materialized.is_symlink():return None
    cached_root=Sdf.Layer.Find(str(scene));cached_data=Sdf.Layer.Find(str(materialized))
    if (cached_root and cached_root.dirty) or (cached_data and cached_data.dirty):return None
    # Read isolated on-disk snapshots, rather than trusting a clean but stale
    # process-wide USD layer cache after some other writer replaces a file.
    root=Sdf.Layer.OpenAsAnonymous(str(scene));data=Sdf.Layer.OpenAsAnonymous(str(materialized))
    if not root or not data:return None
    allowed={'subLayers','subLayerOffsets','defaultPrim','metersPerUnit','kilogramsPerUnit',
             'upAxis','timeCodesPerSecond','framesPerSecond','startTimeCode','endTimeCode'}
    if (list(root.rootPrims) or set(root.pseudoRoot.ListInfoKeys())-allowed
            or list(root.subLayerPaths)!=['composed.usdc']
            or list(root.subLayerOffsets)!=[Sdf.LayerOffset()]
            or set(root.GetExternalReferences())!={'composed.usdc'}
            or root.GetExternalAssetDependencies()):return None
    # Empty reference paths are internal prototype references and are preserved
    # byte-for-byte. External composition and asset paths cannot be relocated by
    # this narrowly scoped fast path.
    if (data.GetFileFormat().formatId!='usdc' or data.subLayerPaths
            or any(data.GetExternalReferences()) or data.GetExternalAssetDependencies()):return None
    root.subLayerPaths=[data.identifier]
    stage=Usd.Stage.Open(root);direct=Usd.Stage.Open(data)
    metrics=require_motion_units(stage)
    def identity(value):
        return (stage_metrics(value),str(value.GetDefaultPrim().GetPath()) if value.GetDefaultPrim() else None,
                value.HasAuthoredTimeCodeRange(),value.GetStartTimeCode(),value.GetEndTimeCode())
    if identity(stage)!=identity(direct):return None
    return dict(stage_metrics=metrics,wrapper_metadata_keys=sorted(root.pseudoRoot.ListInfoKeys()),
                materialized_external_usd_dependencies=[],materialized_external_asset_dependencies=[],
                wrapper_has_prim_opinions=False,identity_sublayer_offset=True)


def _copy_materialized_assembly(source,output,manifest,source_sha,started,eligibility):
    """Copy an immutable self-contained wrapper/USDC/skin with before/after SHA checks.

    No property count or geometry digest is copied from possibly stale reports.
    Exact bytes preserve every unknown schema, prototype, material, collider,
    and authored numeric value. Native validation remains pending for the new
    output; no previously failed runtime is reclassified by this storage proof.
    """
    names=('scene.usda','composed.usdc','skin.npz','motion_manifest.json')
    before={name:file_sha256(source/name) for name in names}
    if json.loads((source/'motion_manifest.json').read_text())!=manifest:
        raise ValueError('Source manifest changed before materialized byte copy')
    if before['scene.usda']!=source_sha or before['composed.usdc']!=manifest['compact_assets']['materialized_dependency']['sha256']:
        raise ValueError('Source assembly changed before materialized byte copy')
    output.mkdir(parents=True,exist_ok=False)
    copied={}
    for name in names[:-1]:
        shutil.copy2(source/name,output/name)
        copied[name]=file_sha256(output/name)
        if copied[name]!=before[name]:raise ValueError('Materialized byte copy SHA mismatch: '+name)
    after={name:file_sha256(source/name) for name in names}
    if after!=before:raise ValueError('Source assembly changed during materialized byte copy')
    dependency=dict(relative_path='composed.usdc',sha256=copied['composed.usdc'],
                    format='usdc',materialized=True,external_usd_sublayers=[])
    manifest['scene_sha256']=copied['scene.usda']
    manifest['compact_assets']=dict(schema='s300-lossless-materialized-usdc-v1',source_assembly=str(source),
        source_scene_sha256=source_sha,source_manifest_sha256=before['motion_manifest.json'],
        materialized_dependency=dependency,geometry_unchanged=True,native_validation='pending',
        storage_copy_mode='verified-materialized-byte-copy-v1',
        note='Existing self-contained wrapper/USDC/skin copied exactly; fresh inventory not recomputed')
    (output/'motion_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    proof=dict(schema='s300-materialized-byte-identity-v1',source_assembly=str(source),
        source_sha256_before=before,source_sha256_after=after,copied_sha256=copied,
        all_copied_bytes_identical=True,source_unchanged=True,**eligibility,
        property_inventory_recomputed=False,world_geometry_recomputed=False)
    (output/'compact_byte_identity.json').write_text(json.dumps(proof,indent=2)+'\n')
    report=dict(source_scene_bytes=(source/'scene.usda').stat().st_size,
        compact_usdc_bytes=(output/'composed.usdc').stat().st_size,
        wrapper_bytes=(output/'scene.usda').stat().st_size,stage_metrics=eligibility['stage_metrics'],
        elapsed_s=time.perf_counter()-started,geometry_unchanged=True,native_validation='pending',
        training_eligible=False,harvest_success=None,materialized_dependency=dependency,
        storage_copy_mode='verified-materialized-byte-copy-v1',byte_identity=proof,
        property_inventory_recomputed=False,world_geometry_recomputed=False)
    report['scene_storage_fraction']=(report['compact_usdc_bytes']+report['wrapper_bytes'])/report['source_scene_bytes']
    (output/'compact_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    # Include report publication in the immutable source check. A changed input
    # is a hard error, never a reason to silently retry through the full path.
    if {name:file_sha256(source/name) for name in names}!=before:
        raise ValueError('Source assembly changed during materialized byte copy')
    return report


def compact_assembly(source,output):
    started=time.perf_counter()
    source=Path(source).resolve();output=Path(output).resolve();scene=source/'scene.usda'
    manifest=json.loads((source/'motion_manifest.json').read_text());source_sha=file_sha256(scene)
    if source_sha!=manifest['scene_sha256']:raise ValueError('Source assembly SHA mismatch')
    verify_compact_dependencies(source,manifest)
    eligibility=_byte_copy_candidate(source,manifest)
    if eligibility is not None:
        return _copy_materialized_assembly(source,output,manifest,source_sha,started,eligibility)
    stage=Usd.Stage.Open(str(scene));require_motion_units(stage)
    before=property_inventory(stage)
    output.mkdir(parents=True,exist_ok=False)
    materialized=output/'composed.usdc';flattened=stage.Flatten(addSourceFileComment=False)
    # An empty reference asset path is an internal reference, not an external
    # dependency; USD reports it in GetExternalReferences for prototype data.
    if flattened.subLayerPaths or any(flattened.GetExternalReferences()):
        raise ValueError('Materialized compact layer retained external USD dependencies')
    flattened.Export(str(materialized))
    wrapper=Usd.Stage.CreateNew(str(output/'scene.usda'))
    wrapper.GetRootLayer().subLayerPaths=['composed.usdc'];copy_stage_metrics(stage,wrapper)
    if stage.HasAuthoredTimeCodeRange():
        wrapper.SetStartTimeCode(stage.GetStartTimeCode());wrapper.SetEndTimeCode(stage.GetEndTimeCode())
    if stage.GetDefaultPrim():wrapper.SetDefaultPrim(wrapper.GetPrimAtPath(stage.GetDefaultPrim().GetPath()))
    wrapper.GetRootLayer().Save();require_motion_units(wrapper)
    if stage_metrics(stage)!=stage_metrics(wrapper):raise ValueError('Compact stage units changed')
    after=property_inventory(wrapper)
    if before!=after:raise ValueError('Compact resolved property inventory differs from source')
    geometry=compare_geometry(stage,wrapper)
    (output/'compact_geometry_inventory.json').write_text(json.dumps(geometry,indent=2)+'\n')
    shutil.copy2(source/'skin.npz',output/'skin.npz')
    dependency=dict(relative_path='composed.usdc',sha256=file_sha256(materialized),
                    format='usdc',materialized=True,external_usd_sublayers=[])
    manifest['scene_sha256']=file_sha256(output/'scene.usda')
    manifest['compact_assets']=dict(schema='s300-lossless-materialized-usdc-v1',source_assembly=str(source),
        source_scene_sha256=source_sha,source_manifest_sha256=file_sha256(source/'motion_manifest.json'),
        materialized_dependency=dependency,geometry_unchanged=True,native_validation='pending',
        note='Storage representation change only; fully composed physics definitions in one USDC layer')
    (output/'motion_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    report=dict(source_scene_bytes=scene.stat().st_size,compact_usdc_bytes=materialized.stat().st_size,
        wrapper_bytes=(output/'scene.usda').stat().st_size,source_properties=before,compact_properties=after,
        stage_metrics=stage_metrics(wrapper),**{k:v for k,v in geometry.items()if k not in('colliders','rigid_bodies')},
        elapsed_s=time.perf_counter()-started,geometry_unchanged=True,native_validation='pending',
        training_eligible=False,harvest_success=None,materialized_dependency=dependency)
    report['scene_storage_fraction']=(report['compact_usdc_bytes']+report['wrapper_bytes'])/report['source_scene_bytes']
    (output/'compact_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    if file_sha256(scene)!=source_sha:raise ValueError('Source scene changed during compaction')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();print(json.dumps(compact_assembly(args.source,args.output),indent=2))
