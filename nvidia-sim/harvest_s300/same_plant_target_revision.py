"""Opt-in target/collision-only revision within an already measured active plant.

No plant construction, freezing, attachment editing or native stepping occurs.
The derived asset retains the original REST geometry, topology and drives. Its
caller MUST use the existing full native resume, with exact checkpoint q/qvel,
preload, controller targets and prior-break markers; this is an engine restart.
"""
import argparse
import copy
import json
from pathlib import Path
import re
import shutil

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from compact_motion_assets import file_sha256, verify_compact_dependencies
from native_route_checkpoint import load_checkpoint
from motion_assets import grasp_reference, select_build_target
from motion_stage_metrics import require_motion_units
from strict_window_storage import canonical_layer_digest, materialize_exact_layer, require_self_contained

SCHEMA='s300-same-active-plant-target-collision-revision-v1'
FIELD='same_plant_target_revision'
MODE='target_truss_and_own_stem_only'
RELEASE_RULE='preserve_verified_native_released_frozen_and_active_fruit'


def _write(path,value):
    with Path(path).open('x') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')


def _source_files(assembly):
    assembly=Path(assembly)
    return {str(path):file_sha256(path) for path in
        [assembly/'scene.usda',assembly/'motion_manifest.json',assembly/'skin.npz']+
        ([assembly/'composed.usdc'] if (assembly/'composed.usdc').is_file() else [])}


def _open_asset(assembly,manifest):
    """Read disk through isolated USD layers, not another runner's session cache."""
    assembly=Path(assembly)
    if file_sha256(assembly/'scene.usda')!=manifest['scene_sha256']:
        raise ValueError('Same-plant target source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    root=Sdf.Layer.OpenAsAnonymous(str(assembly/'scene.usda'))
    dependency=manifest.get('compact_assets',{}).get('materialized_dependency')
    if dependency:
        from strict_window_storage import require_identity_wrapper
        # Verify the exact wrapper before substituting its isolated dependency.
        wrapper=Usd.Stage.Open(str(assembly/'scene.usda'))
        require_identity_wrapper(wrapper,assembly/'composed.usdc')
        data=Sdf.Layer.OpenAsAnonymous(str(assembly/'composed.usdc'))
        # The verified identity wrapper adds no prim opinions. Preserve the
        # original self-contained authored namespace; Flatten would duplicate
        # and renumber existing internal instance prototypes on every pass.
        root=data
    elif root.subLayerPaths or any(root.GetExternalReferences()):
        raise ValueError('Same-plant target revision requires a self-contained source')
    stage=Usd.Stage.Open(root);require_motion_units(stage)
    return stage


def _validated_parent(assembly,checkpoint):
    assembly,checkpoint=Path(assembly).resolve(),Path(checkpoint).resolve()
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    # Reuse the existing terminal/no-rollback/source/self-audit verifier.
    from run_row_harvest import validated_initial_checkpoint
    metadata,terminal=validated_initial_checkpoint(assembly,manifest,checkpoint)
    loaded,arrays=load_checkpoint(checkpoint,expected_scene_sha256=manifest['scene_sha256'])
    if loaded!=metadata or metadata['manifest']!=manifest:
        raise ValueError('Same-plant target checkpoint/parent manifest mismatch')
    from robot_controller_state import resolve_robot_controller_state
    controller=resolve_robot_controller_state(metadata,arrays,checkpoint)
    if controller['position_targets'] is None or controller['velocity_targets'] is None:
        raise ValueError('Same-plant target requires proven native controller targets')
    if manifest.get('plant_interaction_variant',{}).get('mode')!=MODE:
        raise ValueError('Same-plant target revision requires the existing exact target-truss collision policy')
    return manifest,metadata,arrays,terminal


def _active_release_inventory(stage,manifest,metadata,checkpoint):
    root=manifest['plant']['root'];fruits={f['joint']:f for f in manifest['plant']['fruits']}
    if (metadata['source_plant']!=manifest['plant']['source_plant'] or
            metadata['manifest']['plant']!=manifest['plant']):
        raise ValueError('Same-plant native release source/body identity mismatch')
    result={}
    for event in metadata['break_events']:
        joint=event['joint'];fruit=fruits.get(joint)
        if fruit is None:raise ValueError('Active native release is not an exact current fruit joint')
        name=fruit.get('name','');body=fruit['path'];collider=body+'/Collider'
        if (not re.fullmatch(r'Truss_\d+_Fruit_\d+',name) or body!=root+'/'+name or
                joint!=root+'/HarvestJoints/'+name):
            raise ValueError('Active native release namespace mapping mismatch')
        prim=stage.GetPrimAtPath(body);shape=stage.GetPrimAtPath(collider);j=stage.GetPrimAtPath(joint)
        if (not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI) or not shape or
                not shape.IsA(UsdGeom.Mesh) or not shape.HasAPI(UsdPhysics.CollisionAPI) or
                not j or not j.IsA(UsdPhysics.Joint)):
            raise ValueError('Active native release exact Mesh/rigid/joint inventory missing')
        owner=shape
        while owner and not owner.IsPseudoRoot() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        native=UsdPhysics.Joint(j)
        if (str(owner.GetPath())!=body or
                list(map(str,native.GetBody0Rel().GetTargets()))!=[fruit['anchor']] or
                list(map(str,native.GetBody1Rel().GetTargets()))!=[body]):
            raise ValueError('Active native release rigid owner/attachment body mapping mismatch')
        if native.GetJointEnabledAttr().GetNumTimeSamples():
            raise ValueError('Animated native attachment cannot enter active release proof')
        record=result.setdefault(collider,dict(native_joint=joint,body=body,source_plant=metadata['source_plant'],
            checkpoint=str(Path(checkpoint).resolve()),checkpoint_metadata_sha256=metadata['metadata_sha256'],
            checkpoint_npz_sha256=metadata['checkpoint_npz_sha256'],checkpoint_time_s=metadata['time_s'],
            original_native_break_events=[],scope='exact_checkpoint_proven_native_released_active_fruit'))
        record['original_native_break_events'].append(copy.deepcopy(event))
    return dict(sorted(result.items()))


def verified_active_release_inventory(stage,manifest):
    """Read-only authority for active released fruit outside a newly selected truss.

    A target-only flag or fruit name is insufficient. The immutable native
    checkpoint and its exact parent plant/model/source hashes must all match.
    """
    revision=manifest.get(FIELD)
    if revision is None:return {}
    if (revision.get('schema')!=SCHEMA or revision.get('enabled')is not True or
            revision.get('source_plant')!=manifest['plant']['source_plant'] or
            revision.get('target_name')!=manifest['target']['name']):
        raise ValueError('Exact same-active-plant target revision declaration required')
    checkpoint=Path(revision.get('checkpoint',''))
    if not checkpoint.is_absolute():raise ValueError('Active release requires absolute native checkpoint provenance')
    metadata,_=load_checkpoint(checkpoint)
    from resume_native_state import require_completed_checkpoint
    require_completed_checkpoint(metadata)
    if (metadata['metadata_sha256']!=revision.get('checkpoint_metadata_sha256') or
            metadata['checkpoint_npz_sha256']!=revision.get('checkpoint_npz_sha256') or
            metadata['source_assembly']!=revision.get('source_assembly') or
            metadata['scene_sha256']!=revision.get('source_scene_sha256')):
        raise ValueError('Active release checkpoint/source SHA mismatch')
    files=revision.get('source_file_sha256',{})
    expected=_source_files(metadata['source_assembly'])
    if files!=expected:raise ValueError('Active release parent source file SHA mismatch')
    result=_active_release_inventory(stage,manifest,metadata,checkpoint)
    if result!=revision.get('preserved_active_native_released_fruit_colliders'):
        raise ValueError('Active native release inventory differs from exact checkpoint proof')
    return result


def _primary_colliders(stage,manifest,truss):
    """Exact declared physical primary shapes only; never Skin/Visual/leaf APIs."""
    root=manifest['plant']['root'];fruits={f['path']:f for f in manifest['plant']['fruits']}
    rows=manifest['plant'].get('bodies',[])+manifest['plant']['fruits'];result={}
    for row in rows:
        body=row['path'];name=body.rsplit('/',1)[-1]
        if not name.startswith(truss+'_'):continue
        if body!=root+'/'+name or row.get('name',name)!=name:
            raise ValueError('Retarget primary collider source body namespace mismatch')
        owner=stage.GetPrimAtPath(body);path=body+'/Collider';shape=stage.GetPrimAtPath(path)
        expected=UsdGeom.Mesh if body in fruits else UsdGeom.Capsule
        if (not owner or not owner.HasAPI(UsdPhysics.RigidBodyAPI) or not shape or
                not shape.IsA(expected) or not shape.HasAPI(UsdPhysics.CollisionAPI) or
                shape.GetAttribute('physics:collisionEnabled').GetNumTimeSamples()):
            raise ValueError('Retarget exact declared primary Mesh/Capsule collider missing or animated: '+path)
        result[path]=dict(body=body,type=shape.GetTypeName(),fruit=body in fruits)
    if not result or not any(row['fruit'] for row in result.values()):
        raise ValueError('Retarget requires existing primary fruit and branch collision coverage')
    return result


def _recipe(source,old,metadata,checkpoint,target_name):
    from harvest_contact_scope import verified_route_release_inventory
    from plant_interaction_variants import target_only_keep,verified_disabled_colliders
    verified_disabled_colliders(source,old)
    target=copy.deepcopy(select_build_target(old['plant']['fruits'],target_name))
    old_truss=old['target']['name'].split('_Fruit_')[0];truss=target_name.split('_Fruit_')[0]
    if truss==old_truss:raise ValueError('Same-plant target revision requires a DIFFERENT existing target truss')
    broken={e['joint'] for e in metadata['break_events']}
    if target['joint'] in broken:raise ValueError('Retarget may not select an already native-released fruit')
    primary=_primary_colliders(source,old,truss)
    old_primary=_primary_colliders(source,old,old_truss)
    active=_active_release_inventory(source,old,metadata,checkpoint)
    frozen=verified_route_release_inventory(source,old.get('route_checkpoint',{}).get('outgoing_history',[]),
        disabled_colliders=old['plant_interaction_variant']['disabled_colliders'])
    released=dict(frozen,**active)
    desired={};before={};disabled=[];root=old['plant']['root']
    for prim in Usd.PrimRange.Stage(source,Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
        path=str(prim.GetPath());value=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        before[path]=value
        if path in primary or path in released:wanted=True
        elif target_only_keep(path,root,truss)is False:wanted=False
        else:wanted=value
        desired[path]=wanted
        if target_only_keep(path,root,truss)is False and path not in released:disabled.append(path)
    if any(path not in desired for path in released):raise ValueError('Retarget lost a prior released fruit collider')
    changes=[dict(path=p,before=before[p],after=v) for p,v in sorted(desired.items()) if v!=before[p]]
    if (not any(row['after']is True for row in changes) or
            any(row['after']is True and row['path']not in primary and row['path']not in released for row in changes)):
        raise ValueError('Retarget collision activation is not the exact new primary/released whitelist')
    for path in set(old_primary)-set(released):
        if desired[path]is not False:raise ValueError('Old attached target collider was not omitted')
    policy=dict(mode=MODE,target_root=root,target_truss=truss,disabled_colliders=sorted(disabled),disabled_count=len(disabled),
        released_fruit_lifecycle_rule=RELEASE_RULE,preserved_native_released_fruit_colliders=released,
        native_released_fruit_floor_and_structure_collision_response_preserved=True,
        inherited_after_measured_window_switch=False,same_active_plant_target_revision=True,
        previous_target_root=root,previous_target_truss=old_truss,
        previous_disabled_count=old['plant_interaction_variant']['disabled_count'],
        planner_and_native_same_saved_collision_states=True,
        visuals_and_dynamics_preserved=True,robot_house_gutter_preserved=True,
        training_eligible=False,harvest_success=None,
        interpretation='Explicit current-plant target-truss collision window; other attached trusses remain ghosted; all proven released fruit remain physical')
    revised=copy.deepcopy(old);revised['target']=target;revised['plant_interaction_variant']=policy
    selected=sorted([f for f in old['plant']['fruits'] if f['name'].startswith(truss+'_Fruit_') and f['joint']not in broken],key=lambda f:f['center'][2],reverse=True)[:2]
    revised['grasp_reference']=grasp_reference(old,[f['name'] for f in selected],old.get('grasp_reference',{}).get('height_offset_m',0.))
    revised.update(training_eligible=False,harvest_success=None)
    return dict(manifest=revised,changes=changes,desired=desired,primary=primary,active=active,frozen=frozen,
        previous_truss=old_truss,target_truss=truss)


def _set_flags(stage,changes,*,restore=False):
    for row in changes:
        prim=stage.GetPrimAtPath(row['path'])
        if not prim or not prim.HasAPI(UsdPhysics.CollisionAPI):raise ValueError('Retarget collider lost its physical API')
        attr=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
        attr.Set(row['before' if restore else 'after'])


def _audit(source,derived,recipe):
    """Compare ALL authored fields, restoring only exact collision flag opinions.

    Do not Flatten an already self-contained layer: USD may duplicate/reindex
    its internal instance prototypes. Unknown fields and namespace stay in the
    full canonical digest. Missing defaults are restored as missing opinions.
    """
    left=source.GetRootLayer();require_self_contained(left)
    require_self_contained(derived.GetRootLayer())
    right=Sdf.Layer.CreateAnonymous();right.TransferContent(derived.GetRootLayer())
    normalized=Usd.Stage.Open(right)
    for path,wanted in recipe['desired'].items():
        p=normalized.GetPrimAtPath(path)
        if not p or not p.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()!=wanted:
            raise ValueError('Actual derived collision flags differ from exact target/release whitelist: '+path)
    for row in recipe['changes']:
        attrpath=row['path']+'.physics:collisionEnabled'
        old=left.GetAttributeAtPath(attrpath);new=right.GetAttributeAtPath(attrpath)
        if new is None:raise ValueError('Retarget changed flag has no exact authored opinion')
        if old is None:
            # Compare the complete expected NEW flag descriptor, not just its
            # default. Metadata/time-sample or unrelated property edits cannot
            # be hidden by removing this deliberately newly authored flag.
            scratch=Usd.Stage.CreateInMemory();prim=scratch.DefinePrim('/Collider','Mesh')
            UsdPhysics.CollisionAPI.Apply(prim).GetCollisionEnabledAttr().Set(row['after'])
            expected=scratch.GetRootLayer().GetAttributeAtPath('/Collider.physics:collisionEnabled')
            keys=set(new.ListInfoKeys())
            if keys!=set(expected.ListInfoKeys()) or any(new.GetInfo(k)!=expected.GetInfo(k) for k in keys):
                raise ValueError('Retarget collision flag gained nondefault authored metadata')
            right.GetPrimAtPath(row['path']).RemoveProperty(new)
        else:
            oldkeys=set(old.ListInfoKeys())-{'default'};newkeys=set(new.ListInfoKeys())-{'default'}
            if oldkeys!=newkeys or any(new.GetInfo(k)!=old.GetInfo(k) for k in oldkeys):
                raise ValueError('Retarget collision flag changed nondefault authored metadata')
            if old.HasInfo('default'):new.default=old.default
            else:new.ClearInfo('default')
    a,b=canonical_layer_digest(left),canonical_layer_digest(right)
    if a!=b:raise ValueError('Same-plant target revision changed noncollision authored properties or topology')
    return dict(passed=True,source_canonical=a,normalized_derived_canonical=b,
        only_changed_attribute='physics:collisionEnabled',collision_changes=recipe['changes'],
        full_authored_layer_verified=True,source_authored_namespace_preserved=True,
        source_rest_geometry_joints_drives_mass_materials_preserved=True,
        plant_body_dof_identity_unchanged=True,all_native_released_colliders_preserved=True)


def revise(assembly,checkpoint,output,target_name,*,fast_materialization=True):
    """Create immutable asset; actual motion MUST explicitly resume this checkpoint."""
    if fast_materialization is not True:raise ValueError('Same-plant retarget requires exact whole-layer materialization')
    assembly,checkpoint,output=map(lambda p:Path(p).resolve(),(assembly,checkpoint,output))
    if output.exists():raise FileExistsError(output)
    old,metadata,arrays,terminal=_validated_parent(assembly,checkpoint)
    source_files=_source_files(assembly);source=_open_asset(assembly,old)
    recipe=_recipe(source,old,metadata,checkpoint,target_name)
    copied=Sdf.Layer.CreateAnonymous();copied.TransferContent(source.GetRootLayer())
    stage=Usd.Stage.Open(copied);_set_flags(stage,recipe['changes'])
    output.mkdir(parents=True,exist_ok=False);storage=materialize_exact_layer(stage,output,preserve_authored_layer=True)
    shutil.copyfile(assembly/'skin.npz',output/'skin.npz')
    revised=recipe['manifest'];revised['scene_sha256']=file_sha256(output/'scene.usda')
    revised['compact_assets']=dict(schema='s300-lossless-materialized-usdc-v1',source_assembly=str(assembly),
        source_scene_sha256=old['scene_sha256'],source_manifest_sha256=source_files[str(assembly/'motion_manifest.json')],
        materialized_dependency=dict(relative_path='composed.usdc',sha256=file_sha256(output/'composed.usdc'),format='usdc',materialized=True,external_usd_sublayers=[]),
        geometry_unchanged=True,native_validation='pending',
        note='Only exact target-truss/released collisionEnabled flags change; full measured same-plant native resume mandatory; no geometry, rest transforms, dynamics or attachments changed')
    audit=_audit(source,_open_asset(output,revised),recipe)
    revision=dict(schema=SCHEMA,enabled=True,source_assembly=str(assembly),source_plant=metadata['source_plant'],
        source_scene_sha256=metadata['scene_sha256'],source_file_sha256=source_files,
        checkpoint=str(checkpoint),checkpoint_metadata_sha256=metadata['metadata_sha256'],checkpoint_npz_sha256=metadata['checkpoint_npz_sha256'],
        checkpoint_time_s=metadata['time_s'],terminal_source_proof=terminal,target_name=target_name,
        previous_target_truss=recipe['previous_truss'],target_truss=recipe['target_truss'],
        exact_new_target_primary_colliders=recipe['primary'],
        preserved_active_native_released_fruit_colliders=recipe['active'],
        frozen_native_released_count=len(recipe['frozen']),active_native_released_count=len(recipe['active']),
        native_resume_required=True,native_state_and_controller_changed=False,native_breaks_reattached=False,
        rail_rebased=False,plant_rebuilt_or_frozen=False,skin_sha256=file_sha256(output/'skin.npz'),
        explicit_active_collision_window_change=True,bitwise_engine_continuity=False,
        storage_proof=storage['schema'],property_proof=audit,
        training_eligible=False,harvest_success=None)
    revised[FIELD]=revision
    _write(output/'motion_manifest.json',revised);_write(output/'same_plant_target_revision_validation.json',revision)
    if _source_files(assembly)!=source_files:raise ValueError('Source changed during immutable target revision')
    verify_compact_dependencies(output,revised)
    return revision


def validate_resume_revision(assembly,manifest,metadata):
    """Narrow opt-in authorization, independently rechecking whitelist and full layer."""
    assembly=Path(assembly).resolve();revision=manifest.get(FIELD,{})
    if (revision.get('schema')!=SCHEMA or revision.get('enabled')is not True or
            revision.get('source_assembly')!=metadata['source_assembly'] or
            revision.get('source_scene_sha256')!=metadata['scene_sha256'] or
            revision.get('checkpoint_metadata_sha256')!=metadata['metadata_sha256'] or
            revision.get('checkpoint_npz_sha256')!=metadata['checkpoint_npz_sha256'] or
            revision.get('checkpoint_time_s')!=metadata['time_s']):
        raise ValueError('Same-plant target revision lacks exact checkpoint/source parent proof')
    parent=Path(metadata['source_assembly']);old,verified,_,terminal=_validated_parent(parent,revision['checkpoint'])
    if verified!=metadata or terminal!=revision.get('terminal_source_proof'):
        raise ValueError('Same-plant target terminal checkpoint proof changed')
    if revision.get('source_file_sha256')!=_source_files(parent):raise ValueError('Same-plant target parent source SHA mismatch')
    if file_sha256(assembly/'skin.npz')!=file_sha256(parent/'skin.npz') or revision.get('skin_sha256')!=file_sha256(parent/'skin.npz'):
        raise ValueError('Same-plant target deformation skin changed')
    source=_open_asset(parent,old);derived=_open_asset(assembly,manifest)
    recipe=_recipe(source,old,metadata,revision['checkpoint'],revision.get('target_name'))
    proof=_audit(source,derived,recipe)
    if revision.get('property_proof')!=proof or revision.get('exact_new_target_primary_colliders')!=recipe['primary']:
        raise ValueError('Same-plant target property/primary whitelist proof mismatch')
    if (revision.get('preserved_active_native_released_fruit_colliders')!=recipe['active'] or
            revision.get('active_native_released_count')!=len(recipe['active']) or
            revision.get('frozen_native_released_count')!=len(recipe['frozen']) or
            revision.get('source_plant')!=metadata['source_plant'] or
            revision.get('previous_target_truss')!=recipe['previous_truss'] or
            revision.get('target_truss')!=recipe['target_truss']):
        raise ValueError('Same-plant target exact release/truss inventory proof mismatch')
    for key,wanted in dict(native_resume_required=True,native_state_and_controller_changed=False,
            native_breaks_reattached=False,rail_rebased=False,plant_rebuilt_or_frozen=False,
            explicit_active_collision_window_change=True,bitwise_engine_continuity=False,
            training_eligible=False,harvest_success=None).items():
        if revision.get(key)!=wanted:raise ValueError('Same-plant target revision state/physics declaration mismatch: '+key)
    expected=recipe['manifest'];actual=copy.deepcopy(manifest)
    for obj in (expected,actual):
        for key in ('scene_sha256','compact_assets',FIELD):obj.pop(key,None)
    if actual!=expected:raise ValueError('Same-plant target revision changed unrelated manifest/state identities or contact exemptions')
    return dict(kind='explicit_same_active_plant_target_collision_revision',source_assembly=str(parent),revised_assembly=str(assembly),
        source_checkpoint_metadata_sha256=metadata['metadata_sha256'],source_checkpoint_npz_sha256=metadata['checkpoint_npz_sha256'],
        previous_target_truss=recipe['previous_truss'],target_truss=recipe['target_truss'],
        active_native_prior_break_count=len(recipe['active']),frozen_native_prior_release_count=len(recipe['frozen']),
        exact_primary_enable_count=sum(r['after']is True for r in recipe['changes']),
        exact_attached_disable_count=sum(r['after']is False for r in recipe['changes']),
        noncollision_properties_unchanged=True,full_native_resume_required=True,
        collision_window_change=True,training_eligible=False,harvest_success=None)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly',type=Path);parser.add_argument('checkpoint',type=Path)
    parser.add_argument('output',type=Path);parser.add_argument('--target',required=True)
    args=parser.parse_args();print(json.dumps(revise(args.assembly,args.checkpoint,args.output,args.target),indent=2))
