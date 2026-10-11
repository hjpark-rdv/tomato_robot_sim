"""Explicit approved-neck transfer with distinct resumed and fresh entry points.

This is a declared attachment hypothesis revision, not a geometry/contact
waiver. Only unbroken eligible Truss01 fruit-neck joints of original Plant22
receive the already used elastic template. Exact prior native breaks remain
jointEnabled=false declarations. Native startup/state/DOF validation is still
required after a dedicated opt-in resume authorization.
revise_fresh starts an explicitly separate authored normal-start experiment;
it imports no checkpoint and cannot continue a stopped physical route.
"""
import argparse
import copy
import json
import math
from pathlib import Path

from pxr import Sdf,Usd,UsdPhysics

from compact_motion_assets import (compact_assembly,compare_geometry,file_sha256,
                                  property_inventory,verify_compact_dependencies)
from route_checkpoint import _apply_attachment_template,_joint_snapshot
from row_harvest_targets import scoped_trusses,eligible_truss_fruits,truss_key

SCHEMA='s300-approved-eligible-attachment-transfer-v1'
FRESH_SCHEMA='s300-independent-fresh-near-campaign-v1'
SOURCE_PLANT='/World/Greenhouse/Plants/Plant_22'
APPROVED_FORCE_N=3.
APPROVED_TORQUE_NM=.008


def _write(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


def apply_transfer(stage,manifest,metadata,inventory,*,truss_index=1,selection_provenance=None):
    """Caller-owned derived stage only; validate all exact identities first."""
    if truss_index!=1 or manifest['plant']['source_plant']!=SOURCE_PLANT:
        raise ValueError('Only explicitly approved original Plant22/Truss01 transfer is supported')
    template=manifest.get('fruit_attachment')
    if not template or not template.get('joints'):
        raise ValueError('Existing approved target-neck template is required')
    if any(row.get('kind')!='elastic' or row.get('force_n')!=APPROVED_FORCE_N or
           row.get('torque_nm')!=APPROVED_TORQUE_NM for row in template['joints']):
        raise ValueError('Transfer must preserve the existing approved3N/.008Nm elastic hypothesis')
    fruits=manifest['plant']['fruits'];by_joint={fruit['joint']:fruit for fruit in fruits}
    broken=sorted({event['joint'] for event in metadata['break_events']})
    if not set(broken)<=set(by_joint):raise ValueError('Native break is outside exact active fruit inventory')
    scope=scoped_trusses(manifest,inventory)
    key=f'Truss_{truss_index:02d}'
    eligible=eligible_truss_fruits(manifest,scope)
    chosen=[fruit for fruit in eligible if truss_key(fruit['name'])==key and fruit['joint'] not in broken]
    if not chosen:raise ValueError('No unbroken eligible Truss01 fruit-neck joints to transfer')
    selected=[fruit['name'] for fruit in chosen]
    if set(selected)&{row['name'] for row in template['joints']}:
        raise ValueError('Refusing to reconfigure an already revised target-neck joint')
    # Verify every break relation before any mutation. Original break callbacks
    # are inventory proof, not newly emitted callbacks in the revised run.
    for path in broken:
        fruit=by_joint[path];prim=stage.GetPrimAtPath(path)
        if not prim or not prim.IsA(UsdPhysics.Joint):raise ValueError('Missing exact native-broken attachment')
        joint=UsdPhysics.Joint(prim)
        if ([str(p) for p in joint.GetBody0Rel().GetTargets()]!=[fruit['anchor']] or
                [str(p) for p in joint.GetBody1Rel().GetTargets()]!=[fruit['path']]):
            raise ValueError('Native-broken fruit-neck relation mismatch')
    before={fruit['joint']:_joint_snapshot(stage.GetPrimAtPath(fruit['joint'])) for fruit in fruits}
    transferred=_apply_attachment_template(stage,manifest,template,selected_names=selected,
        selection_provenance=selection_provenance)
    for path in broken:UsdPhysics.Joint(stage.GetPrimAtPath(path)).CreateJointEnabledAttr(False)
    changed=[fruit['joint'] for fruit in chosen]
    for path,snapshot in before.items():
        if path not in set(changed)|set(broken) and _joint_snapshot(stage.GetPrimAtPath(path))!=snapshot:
            raise ValueError('Transfer changed an excluded/native-unrelated attachment')
    joint_rows=copy.deepcopy(template['joints'])+transferred['joints']
    manifest['fruit_attachment']=dict(template,joints=joint_rows,
        scope='Original approved target-truss hypothesis plus explicit unbroken eligible Plant22Truss01 transfer; Truss00 unchanged',
        selection='original_template_plus_unbroken_eligible_truss01',
        selected_fruit_names=[row['name'] for row in joint_rows],
        excluded_fruit_names=[fruit['name'] for fruit in fruits if fruit['name'] not in {row['name'] for row in joint_rows}],
        calibrated=False,training_eligible=False)
    return dict(schema=SCHEMA,enabled=True,source_plant=SOURCE_PLANT,truss_index=truss_index,
        selected_fruit_names=selected,selected_joint_paths=changed,
        preserved_native_broken_joints=broken,original_native_break_events=copy.deepcopy(metadata['break_events']),
        approved_source_parameters={key:template['joints'][0][key] for key in
            ('kind','force_n','torque_nm','twist_stiffness_authored','swing_stiffness_authored','damping_authored','deadband_deg')},
        selection_scope=scope,selection_provenance=selection_provenance,
        preserved_excluded_joint_paths=[path for path in before if path not in set(changed)|set(broken)],
        joint_type_revision='PhysicsFixedJoint to passive external PhysicsJoint only on selected attachments',
        geometry_collision_material_mass_actuator_unchanged=True,
        active_plant_body_identity_unchanged=True,articulation_dof_identity_validation='pending native; attachments excluded from articulation',
        native_physics_validation='pending',plant_state_reset=False,source_native_break_credit_unchanged=True,
        model_revision='Explicit approved attachment hypothesis transfer; previous target dynamics may change through coupling',
        training_eligible=False,harvest_success=None)


def _audit(source,derived,old_manifest,metadata,inventory,report):
    """Rebuild the exact allowed joint changes and verify actual USD properties."""
    original=Usd.Stage.Open(source.Flatten(addSourceFileComment=False))
    expected=Usd.Stage.Open(original.Flatten(addSourceFileComment=False))
    expected_manifest=copy.deepcopy(old_manifest)
    expected_report=apply_transfer(expected,expected_manifest,metadata,inventory,
        truss_index=report['truss_index'],selection_provenance=report['selection_provenance'])
    actual_inventory=property_inventory(derived);expected_inventory=property_inventory(expected)
    if actual_inventory!=expected_inventory:
        raise ValueError('Actual derived properties differ from exact approved transfer/native-break declarations')
    normalized=Usd.Stage.Open(derived.Flatten(addSourceFileComment=False))
    for path in expected_report['selected_joint_paths']+expected_report['preserved_native_broken_joints']:
        if not Sdf.CopySpec(original.GetRootLayer(),path,normalized.GetRootLayer(),path):
            raise ValueError('Unable to restore exact source joint specification for independent preservation audit')
    before=property_inventory(original);restored=property_inventory(normalized)
    if before!=restored:raise ValueError('Transfer changed properties outside the exact selected/broken joint specifications')
    # Restoring joint specs affects no geometric prim. The joint type change is
    # intentional; compare_geometry also validates primitive types, so compare
    # this normalized throwaway stage to verify every physical/visual vertex.
    geometry=compare_geometry(original,normalized)
    return dict(nonselected_properties_unchanged=True,source_property_inventory=before,
        normalized_property_inventory=restored,exact_derived_property_inventory=actual_inventory,
        authored_geometry_audit={key:value for key,value in geometry.items() if key not in ('colliders','rigid_bodies')},
        expected_manifest=expected_manifest,expected_report=expected_report)


def revise(assembly,checkpoint,output,inventory_path,*,truss_index=1):
    assembly,checkpoint,output,inventory_path=map(lambda path:Path(path).resolve(),
        (assembly,checkpoint,output,inventory_path))
    if output.exists():raise FileExistsError(output)
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    if file_sha256(assembly/'scene.usda')!=manifest['scene_sha256']:raise ValueError('Source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    from run_row_harvest import validated_initial_checkpoint
    metadata,parent_proof=validated_initial_checkpoint(assembly,manifest,checkpoint)
    inventory=json.loads(inventory_path.read_text())
    provenance=dict(path=str(inventory_path),sha256=file_sha256(inventory_path))
    source_files={str(assembly/name):file_sha256(assembly/name) for name in
                  ('scene.usda','motion_manifest.json','skin.npz')}
    dependency=manifest.get('compact_assets',{}).get('materialized_dependency')
    if dependency:source_files[str(assembly/dependency['relative_path'])]=dependency['sha256']
    original=Usd.Stage.Open(str(assembly/'scene.usda'))
    # Validate on a throwaway stage before allocating a derived output.
    probe=Usd.Stage.Open(original.Flatten(addSourceFileComment=False))
    apply_transfer(probe,copy.deepcopy(manifest),metadata,inventory,truss_index=truss_index,
                   selection_provenance=provenance)
    compact_assembly(assembly,output)
    derived=Usd.Stage.Open(str(output/'composed.usdc'));revised=copy.deepcopy(manifest)
    report=apply_transfer(derived,revised,metadata,inventory,truss_index=truss_index,
                          selection_provenance=provenance)
    derived.GetRootLayer().Save()
    wrapper=Usd.Stage.Open(str(output/'scene.usda'))
    audit=_audit(original,wrapper,manifest,metadata,inventory,report)
    audit.pop('expected_manifest');audit.pop('expected_report');report.update(audit,
        source_assembly=str(assembly),source_scene_sha256=manifest['scene_sha256'],
        source_file_sha256=source_files,checkpoint=str(checkpoint),
        checkpoint_metadata_sha256=metadata['metadata_sha256'],
        terminal_source_proof=parent_proof)
    compact=json.loads((output/'motion_manifest.json').read_text())['compact_assets']
    compact['materialized_dependency']['sha256']=file_sha256(output/'composed.usdc')
    compact['note']='Exact immutable approved-neck transfer only; native broken-joint declarations preserved; all geometry/material/mass/collider properties unchanged.'
    revised['compact_assets']=compact;revised['scene_sha256']=file_sha256(output/'scene.usda')
    revised['eligible_attachment_transfer']=report
    _write(output/'motion_manifest.json',revised)
    _write(output/'eligible_attachment_transfer_validation.json',report)
    for path,digest in source_files.items():
        if file_sha256(path)!=digest:raise ValueError('Source changed during immutable eligible attachment transfer')
    verify_compact_dependencies(output,revised)
    return report


def _fresh_source(stage,manifest,inventory,*,target_name):
    """Reject measured/history assets; the fresh campaign starts at authored rest."""
    if any(key in manifest for key in ('route_checkpoint','route_window_switch',
            'eligible_attachment_transfer','fresh_campaign')):
        raise ValueError('Fresh campaign requires an unvisited authored normal-start assembly')
    q=manifest.get('initial_robot_q')
    if (not isinstance(q,list) or len(q)!=7 or any(isinstance(value,bool) or
            not isinstance(value,(int,float)) or not math.isfinite(value) for value in q)):
        raise ValueError('Fresh campaign requires the exact finite authored normal robot q')
    scope=scoped_trusses(manifest,inventory)
    eligible=eligible_truss_fruits(manifest,scope)
    targets=[fruit for fruit in eligible if fruit['name']==target_name]
    if len(targets)!=1 or truss_key(target_name)!='Truss_01':
        raise ValueError('Fresh initial target must be exactly one eligible Truss01 fruit')
    for fruit in manifest['plant']['fruits']:
        prim=stage.GetPrimAtPath(fruit['joint'])
        if not prim or not prim.IsActive() or not prim.IsA(UsdPhysics.Joint):
            raise ValueError('Fresh normal start has a missing/inactive fruit attachment')
        joint=UsdPhysics.Joint(prim)
        if joint.GetJointEnabledAttr().Get() is False:
            raise ValueError('Fresh normal start may not import a native-disabled attachment')
        if ([str(path) for path in joint.GetBody0Rel().GetTargets()]!=[fruit['anchor']] or
                [str(path) for path in joint.GetBody1Rel().GetTargets()]!=[fruit['path']]):
            raise ValueError('Fresh source exact fruit attachment identity mismatch')
    return copy.deepcopy(targets[0]),scope


def revise_fresh(assembly,output,inventory_path,*,target_name='Truss_01_Fruit_01'):
    """Independent normal-start hypothesis; imports no checkpoint or old break.

    Reuses the approved transfer/property audit, compact storage, and source
    authored robot/plant state. This creates a new experiment; it cannot resume
    a stopped prior route and receives no credit for prior native detachment.
    """
    assembly,output,inventory_path=map(lambda path:Path(path).resolve(),
        (assembly,output,inventory_path))
    if output.exists():raise FileExistsError(output)
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    if file_sha256(assembly/'scene.usda')!=manifest['scene_sha256']:
        raise ValueError('Fresh source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    inventory=json.loads(inventory_path.read_text())
    provenance=dict(path=str(inventory_path),sha256=file_sha256(inventory_path))
    source_files={str(assembly/name):file_sha256(assembly/name) for name in
        ('scene.usda','motion_manifest.json','skin.npz')}
    dependency=manifest.get('compact_assets',{}).get('materialized_dependency')
    if dependency:source_files[str(assembly/dependency['relative_path'])]=dependency['sha256']
    original=Usd.Stage.Open(str(assembly/'scene.usda'))
    target,scope=_fresh_source(original,manifest,inventory,target_name=target_name)
    empty_history=dict(break_events=[])
    probe=Usd.Stage.Open(original.Flatten(addSourceFileComment=False))
    apply_transfer(probe,copy.deepcopy(manifest),empty_history,inventory,
        selection_provenance=provenance)
    compact_assembly(assembly,output)
    derived=Usd.Stage.Open(str(output/'composed.usdc'));revised=copy.deepcopy(manifest)
    transfer=apply_transfer(derived,revised,empty_history,inventory,selection_provenance=provenance)
    derived.GetRootLayer().Save()
    wrapper=Usd.Stage.Open(str(output/'scene.usda'))
    audit=_audit(original,wrapper,manifest,empty_history,inventory,transfer)
    audit.pop('expected_manifest');audit.pop('expected_report')
    from motion_assets import grasp_reference
    revised['target']=target
    initial_fruits=sorted([fruit for fruit in revised['plant']['fruits']
        if truss_key(fruit['name'])=='Truss_01'],key=lambda fruit:fruit['center'][2],reverse=True)[:2]
    revised['grasp_reference']=grasp_reference(revised,[fruit['name'] for fruit in initial_fruits],
        manifest.get('grasp_reference',{}).get('height_offset_m',0.))
    compact=json.loads((output/'motion_manifest.json').read_text())['compact_assets']
    compact['materialized_dependency']['sha256']=file_sha256(output/'composed.usdc')
    compact['note']='Independent fresh authored normal start; exact approved Truss01 attachment transfer only; no checkpoint or prior native breaks imported.'
    revised['compact_assets']=compact;revised['scene_sha256']=file_sha256(output/'scene.usda')
    report=dict(schema=FRESH_SCHEMA,enabled=True,source_assembly=str(assembly),
        source_scene_sha256=manifest['scene_sha256'],source_file_sha256=source_files,
        initial_state='authored normal source; no native checkpoint imported',
        truss_key='Truss_01',initial_target_name=target_name,
        initial_robot_q=copy.deepcopy(manifest.get('initial_robot_q')),
        inherited_native_breaks=[],accepted_previous_route_history=False,
        all_fruit_attachments_enabled_at_authored_start=True,
        eligible_fruit_names=[fruit['name'] for fruit in eligible_truss_fruits(revised,scope)],
        attachment_hypothesis=transfer,selection_provenance=provenance,
        previous_route='Separate historical experiment; this fresh campaign does not continue or roll back its accepted physical history',
        native_initialization_validation='pending',native_physics_validation='pending',
        training_eligible=False,harvest_success=None,**audit)
    revised['fresh_campaign']=report
    revised.update(training_eligible=False,harvest_success=None)
    _write(output/'motion_manifest.json',revised)
    _write(output/'fresh_campaign_validation.json',report)
    for path,digest in source_files.items():
        if file_sha256(path)!=digest:raise ValueError('Source changed during immutable fresh attachment transfer')
    verify_compact_dependencies(output,revised)
    return report


def validate_resume_revision(assembly,manifest,metadata):
    """Dedicated authorization proof; unrelated leaf/tool revisions are not accepted."""
    assembly=Path(assembly).resolve();revision=manifest.get('eligible_attachment_transfer',{})
    if (revision.get('schema')!=SCHEMA or revision.get('enabled') is not True or
            revision.get('source_assembly')!=metadata['source_assembly'] or
            revision.get('source_scene_sha256')!=metadata['scene_sha256'] or
            revision.get('checkpoint_metadata_sha256')!=metadata['metadata_sha256']):
        raise ValueError('Attachment transfer lacks exact checkpoint/source parent proof')
    source_files=revision.get('source_file_sha256',{})
    if not source_files or source_files.get(metadata['source_scene'])!=metadata['scene_sha256']:
        raise ValueError('Attachment transfer source file SHA inventory missing')
    for path,digest in source_files.items():
        if not Path(path).is_absolute() or file_sha256(path)!=digest:raise ValueError('Attachment transfer parent file SHA mismatch')
    provenance=revision['selection_provenance'];inventory_path=Path(provenance['path'])
    if not inventory_path.is_absolute() or file_sha256(inventory_path)!=provenance['sha256']:
        raise ValueError('Attachment transfer exact near-side inventory SHA mismatch')
    parent=Path(metadata['source_assembly']);old=json.loads((parent/'motion_manifest.json').read_text())
    if old!=metadata['manifest']:raise ValueError('Attachment transfer checkpoint parent manifest mismatch')
    from run_row_harvest import validated_initial_checkpoint
    current_metadata,parent_proof=validated_initial_checkpoint(parent,old,revision['checkpoint'])
    if current_metadata['metadata_sha256']!=metadata['metadata_sha256'] or parent_proof!=revision['terminal_source_proof']:
        raise ValueError('Attachment transfer terminal source proof changed')
    if file_sha256(assembly/'scene.usda')!=manifest['scene_sha256']:raise ValueError('Revised attachment transfer scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    audit=_audit(Usd.Stage.Open(str(parent/'scene.usda')),Usd.Stage.Open(str(assembly/'scene.usda')),
        old,metadata,json.loads(inventory_path.read_text()),revision)
    expected=audit['expected_manifest'];actual=copy.deepcopy(manifest)
    for obj in (expected,actual):
        for key in ('scene_sha256','compact_assets','eligible_attachment_transfer'):obj.pop(key,None)
    if actual!=expected:raise ValueError('Attachment transfer changed unrelated manifest/body/state identities')
    exact=audit['expected_report']
    for key,value in exact.items():
        if revision.get(key)!=value:raise ValueError('Attachment transfer exact selection/parameter/break proof mismatch: '+key)
    return dict(kind='explicit_approved_unbroken_eligible_attachment_transfer',
        source_assembly=str(parent),revised_assembly=str(assembly),
        source_checkpoint_metadata_sha256=metadata['metadata_sha256'],
        exact_selected_joint_paths=exact['selected_joint_paths'],
        exact_preserved_native_broken_joints=exact['preserved_native_broken_joints'],
        approved_source_parameters=exact['approved_source_parameters'],
        all_nonselected_properties_unchanged=True,authored_geometry_unchanged=True,
        native_state_and_dof_validation='required before movement',native_physics_validation='pending',
        previous_target_coupled_dynamics_unchanged='not assumed',training_eligible=False,harvest_success=None)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('assembly','checkpoint','output','inventory'):parser.add_argument(name,type=Path)
    args=parser.parse_args();print(json.dumps(revise(args.assembly,args.checkpoint,args.output,args.inventory),ensure_ascii=False,indent=2))
