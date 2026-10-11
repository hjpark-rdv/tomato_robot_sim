"""Explicit immutable accuracy revision for a verified convex-decomposed CAD part.

This changes native physical proxy cooking, never contact authorization or a
penetration detector. Full authored surfaces and all other physics properties
remain exact. The selected configuration has isolated native cooking positive
and negative controls; real resumed dynamics still require fresh validation.
"""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
from pxr import Sdf,Usd,UsdGeom,UsdPhysics

from compact_motion_assets import compact_assembly,compare_geometry,file_sha256,property_inventory,verify_compact_dependencies

API='PhysxConvexDecompositionCollisionAPI'
PREFIX='physxConvexDecompositionCollision:'
CONFIG=dict(hullVertexLimit=64,maxConvexHulls=128,minThickness=.001,
            voxelResolution=5000000,errorPercentage=.5,shrinkWrap=False)
TYPES={key:Sdf.ValueTypeNames.Int if type(value) is int else Sdf.ValueTypeNames.Bool if type(value) is bool else Sdf.ValueTypeNames.Float for key,value in CONFIG.items()}
# A mirror-looking part is not implicitly authorized: the R part has different
# source topology and was not tested by this controlled native fixture.
EXACT_PATH='/World/ActuatedTool/L/part_122'
EXACT_SOURCE_NAME='L_C02_lower_deck'


def _selected(stage,paths):
    if list(paths)!=[EXACT_PATH]:raise ValueError('Only the independently verified exact L lower-deck collider is approved')
    prim=stage.GetPrimAtPath(EXACT_PATH)
    if (not prim or not prim.IsA(UsdGeom.Mesh) or
            prim.GetCustomDataByKey('sourceName')!=EXACT_SOURCE_NAME or
            not prim.HasAPI(UsdPhysics.CollisionAPI) or
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True or
            not prim.HasAPI(UsdPhysics.MeshCollisionAPI) or
            UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()!='convexDecomposition'):
        raise ValueError('Exact enabled source CAD convex-decomposition identity is required')
    owner=prim.GetParent()
    if str(owner.GetPath())!='/World/ActuatedTool/L' or not owner.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('Exact native tool jaw owner required')
    return prim


def _normalized_inventory(source,derived,paths):
    """Remove only approved API metadata and six properties on a throwaway stage."""
    normalized=Usd.Stage.Open(derived.Flatten(addSourceFileComment=False))
    for path in paths:
        before=source.GetPrimAtPath(path);after=normalized.GetPrimAtPath(path)
        if before.HasAuthoredMetadata('apiSchemas'):after.SetMetadata('apiSchemas',before.GetMetadata('apiSchemas'))
        else:after.ClearMetadata('apiSchemas')
        for key in CONFIG:
            name=PREFIX+key;property_path=Sdf.Path(path).AppendProperty(name)
            if before.HasProperty(name):
                if not Sdf.CopySpec(source.GetRootLayer(),property_path,normalized.GetRootLayer(),property_path):
                    raise ValueError('Unable to restore source cooking property for independent audit')
            else:after.RemoveProperty(name)
    return property_inventory(normalized)


def _audit(source,derived,paths):
    source_flat=Usd.Stage.Open(source.Flatten(addSourceFileComment=False))
    _selected(source_flat,paths);selected=_selected(derived,paths)
    tokens=selected.GetMetadata('apiSchemas')
    source_tokens=source_flat.GetPrimAtPath(EXACT_PATH).GetMetadata('apiSchemas')
    expected_tokens=source_tokens.ApplyOperations([]) if source_tokens else []
    if API not in expected_tokens:expected_tokens.append(API)
    if tokens is None or tokens.ApplyOperations([])!=expected_tokens:
        raise ValueError('Derived API schemas changed beyond exactly the approved decomposition accuracy API')
    actual={key:selected.GetAttribute(PREFIX+key).Get() for key in CONFIG}
    expected={key:float(np.float32(value)) if type(value) is float else value for key,value in CONFIG.items()}
    if actual!=expected:raise ValueError('Derived cooking values differ from the exact native-validated configuration')
    before=property_inventory(source_flat);restored=_normalized_inventory(source_flat,derived,paths)
    if before!=restored:raise ValueError('Physical properties changed beyond the approved cooking fields')
    geometry=compare_geometry(source_flat,derived)
    return dict(source_property_inventory=before,normalized_derived_property_inventory=restored,
                non_accuracy_properties_preserved=True,authored_triangles_retained=True,
                geometry_audit={k:v for k,v in geometry.items() if k not in ('colliders','rigid_bodies')},
                actual_cooking_attributes=actual)


def apply_accuracy(stage,paths=(EXACT_PATH,)):
    """Caller-owned derived stage only; validates before any authoring."""
    prim=_selected(stage,paths);before=Usd.Stage.Open(stage.Flatten(addSourceFileComment=False))
    old={key:prim.GetAttribute(PREFIX+key).Get() for key in CONFIG}
    schemas=prim.GetMetadata('apiSchemas');tokens=schemas.ApplyOperations([]) if schemas else []
    if API not in tokens:tokens.append(API)
    prim.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(tokens))
    for key,value in CONFIG.items():prim.CreateAttribute(PREFIX+key,TYPES[key],custom=False).Set(value)
    audit=_audit(before,stage,paths)
    return dict(schema='s300-explicit-collision-proxy-accuracy-v1',enabled=True,
        exact_collider_paths=list(paths),configuration=copy.deepcopy(CONFIG),
        allowed_attribute_names=[PREFIX+key for key in CONFIG],allowed_api_schema=API,
        collider_changes=[dict(path=EXACT_PATH,source_name=EXACT_SOURCE_NAME,before=old,
                               after=audit['actual_cooking_attributes'])],
        physical_proxy_representation_changed=True,contact_authorization_unchanged=True,
        penetration_limit_m=.0005,native_runtime_validation='pending',
        full_source_triangle_enclosure_proven=False,
        note='Declared native proxy accuracy revision; source-surface sampling is diagnostic, not a lossless native collision claim.',
        training_eligible=False,harvest_success=None,**audit)


def _fixture_evidence(assembly,evidence):
    evidence=Path(evidence).resolve();result=json.loads((evidence/'result.json').read_text());selected=json.loads((evidence/'selected_tool_only_review.json').read_text());inp=json.loads((evidence/'input.json').read_text())
    if (result.get('passed') is not True or selected.get('passed') is not True or
            selected.get('selected_actual_collider_paths')!=[EXACT_PATH] or
            selected.get('selected_native_configuration',{}).get('attributes')!=CONFIG or
            selected.get('same_pose_native_clearance_m',0)<=0 or
            not selected.get('deep_native_controls') or
            any(row.get('native_penetration_upper_bound_m',0)<=.0005 for row in selected['deep_native_controls'])):
        raise ValueError('Native cooking controls/configuration were not independently verified')
    source=assembly/'composed.usdc'
    if not source.is_file() or file_sha256(source)!=result.get('source_sha256') or result.get('source')!=str(source):
        raise ValueError('Cooking fixture is not from this exact immutable source model')
    if file_sha256(evidence/'input.json')!=result.get('input_sha256'):raise ValueError('Fixture input SHA mismatch')
    text=(evidence/'run_control.log').read_text()
    if 'Invalid max hull vertices' in text:raise ValueError('Native cooker rejected configuration')
    return dict(folder=str(evidence),file_sha256={str(evidence/name):file_sha256(evidence/name) for name in
                    ('input.json','result.json','selected_tool_only_review.json','source_coverage_diagnostic.json','fixture.usdc','run_control.py','run_control.log')},
                fixture_only=True,selected_clearance_m=selected['same_pose_native_clearance_m'],
                same_source_triangles=True,deep_controls_passed=True,production_dynamics_validated=False)


def revise(assembly,output,*,evidence):
    """Reuse existing lossless compact writer, then revise exact fresh proxy attrs."""
    assembly=Path(assembly).resolve();output=Path(output).resolve()
    if output.exists():raise FileExistsError(output)
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    if file_sha256(assembly/'scene.usda')!=manifest.get('scene_sha256'):raise ValueError('Source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest);fixture=_fixture_evidence(assembly,evidence)
    source_files={str(assembly/name):file_sha256(assembly/name) for name in ('scene.usda','composed.usdc','motion_manifest.json','skin.npz')}
    original=Usd.Stage.Open(str(assembly/'scene.usda'));_selected(original,[EXACT_PATH])
    compact_assembly(assembly,output)
    derived=Usd.Stage.Open(str(output/'composed.usdc'));report=apply_accuracy(derived)
    derived.GetRootLayer().Save()
    wrapper=Usd.Stage.Open(str(output/'scene.usda'));audit=_audit(original,wrapper,[EXACT_PATH])
    report.update(audit,source_assembly=str(assembly),source_scene_sha256=manifest['scene_sha256'],
                  source_manifest_sha256=source_files[str(assembly/'motion_manifest.json')],
                  source_file_sha256=source_files,native_cooking_fixture=fixture)
    revised=copy.deepcopy(manifest);revised['scene_sha256']=file_sha256(output/'scene.usda')
    compact=json.loads((output/'motion_manifest.json').read_text())['compact_assets']
    compact['materialized_dependency']['sha256']=file_sha256(output/'composed.usdc')
    compact['note']='Storage preserves every derived property exactly; native proxy accuracy changed explicitly on one approved CAD collider.'
    revised['compact_assets']=compact;revised['collision_proxy_accuracy']=report
    (output/'motion_manifest.json').write_text(json.dumps(revised,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (output/'collision_proxy_accuracy_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    for path,digest in source_files.items():
        if file_sha256(path)!=digest:raise ValueError('Source changed during immutable accuracy revision')
    verify_compact_dependencies(output)
    return report


def validate_resume_revision(assembly,manifest,metadata):
    """No mutations: independently prove checkpoint parent and exact property scope."""
    assembly=Path(assembly).resolve();revision=manifest.get('collision_proxy_accuracy',{})
    if (revision.get('enabled') is not True or revision.get('source_assembly')!=metadata['source_assembly'] or
            revision.get('source_scene_sha256')!=metadata['scene_sha256'] or
            revision.get('exact_collider_paths')!=[EXACT_PATH] or revision.get('configuration')!=CONFIG):
        raise ValueError('Resume accuracy revision lacks exact original checkpoint parent proof')
    files=revision.get('source_file_sha256',{})
    if not files or files.get(metadata['source_scene'])!=metadata['scene_sha256']:
        raise ValueError('Resume accuracy revision source SHA inventory is missing')
    for path,digest in files.items():
        if not Path(path).is_absolute() or file_sha256(path)!=digest:raise ValueError('Resume accuracy parent asset SHA mismatch')
    parent=Path(metadata['source_assembly']);old=json.loads((parent/'motion_manifest.json').read_text())
    if old!=metadata['manifest']:raise ValueError('Resume checkpoint parent manifest differs from immutable source')
    left,right=copy.deepcopy(old),copy.deepcopy(manifest)
    for obj in (left,right):
        for key in ('scene_sha256','compact_assets','collision_proxy_accuracy'):obj.pop(key,None)
    if left!=right:raise ValueError('Resume accuracy revision changed unrelated manifest state or identity')
    if file_sha256(assembly/'scene.usda')!=manifest['scene_sha256']:raise ValueError('Resume revised scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    # Compare the actual current files, rather than trusting report booleans.
    audit=_audit(Usd.Stage.Open(str(parent/'scene.usda')),Usd.Stage.Open(str(assembly/'scene.usda')),[EXACT_PATH])
    fixture=_fixture_evidence(parent,revision.get('native_cooking_fixture',{}).get('folder',''))
    if fixture!=revision.get('native_cooking_fixture'):raise ValueError('Resume native cooking evidence changed')
    return dict(kind='explicit_targeted_convex_decomposition_accuracy_revision',
                original_assembly=str(parent),original_scene_sha256=metadata['scene_sha256'],
                revised_assembly=str(assembly),revised_scene_sha256=manifest['scene_sha256'],
                exact_collider_paths=[EXACT_PATH],configuration=copy.deepcopy(CONFIG),
                authored_geometry_unchanged=True,non_accuracy_properties_preserved=True,
                independent_current_property_audit=audit['source_property_inventory'],
                native_proxy_validation='Isolated cooking controls pass; actual restored physics requires fresh validation',
                contact_authorization_unchanged=True,penetration_limit_m=.0005)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('assembly',type=Path);parser.add_argument('output',type=Path);parser.add_argument('--evidence',type=Path,required=True)
    args=parser.parse_args();print(json.dumps(revise(args.assembly,args.output,evidence=args.evidence),ensure_ascii=False,indent=2))
