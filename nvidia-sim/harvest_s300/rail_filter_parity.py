"""Restore original static-chassis/platform collision coverage after rail conversion.

Reparenting S300 CAD beneath the moving world body broadens its disabled lift
joint's adjacency exclusion to every chassis part. This opt-in derived asset
enables that one joint's collision and retains the old base-shape exclusion
only as exact shape pairs. It does not alter geometry or current run assets.
"""
import argparse
import json
import shutil
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics

from compact_motion_assets import (compare_geometry, file_sha256,
                                  property_inventory, verify_compact_dependencies)
from motion_collision_groups import _filtered_pairs
from motion_stage_metrics import copy_stage_metrics, require_motion_units, stage_metrics
from rail_assets import BASE_BODY, NEW_CHASSIS, OLD_CHASSIS, ROBOT

LIFT_JOINT = ROBOT + '/joints/farmily_lift_height_joint'
PLATFORM = ROBOT + '/farmily_lift_platform'


def _owner(prim):
    while prim and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        prim = prim.GetParent()
    return str(prim.GetPath()) if prim else None


def _colliders(stage, root):
    prim = stage.GetPrimAtPath(root)
    if not prim:
        raise ValueError('Missing exact collision root: ' + root)
    return [str(p.GetPath()) for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
            if p.HasAPI(UsdPhysics.CollisionAPI) and
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False]


def _lift(stage):
    prim = stage.GetPrimAtPath(LIFT_JOINT)
    if not prim or not prim.IsA(UsdPhysics.PrismaticJoint):
        raise ValueError('Missing exact lift prismatic joint')
    joint = UsdPhysics.Joint(prim)
    if list(map(str, joint.GetBody0Rel().GetTargets())) != [BASE_BODY] or \
            list(map(str, joint.GetBody1Rel().GetTargets())) != [PLATFORM]:
        raise ValueError('Lift body mapping differs from the original base/platform')
    if joint.GetJointEnabledAttr().Get() is False:
        raise ValueError('Lift joint is disabled')
    return joint


def restore_platform_chassis_collision(stage, source_stage):
    """Modify only a caller-owned stage, after checking exact original coverage."""
    old_joint = _lift(source_stage)
    joint = _lift(stage)
    if old_joint.GetCollisionEnabledAttr().Get() is not False or \
            joint.GetCollisionEnabledAttr().Get() is not False:
        raise ValueError('Expected original and current disabled lift-body adjacency')
    source_chassis = _colliders(source_stage, OLD_CHASSIS)
    chassis = _colliders(stage, NEW_CHASSIS)
    mapped = [NEW_CHASSIS + path[len(OLD_CHASSIS):] for path in source_chassis]
    if not chassis or set(mapped) != set(chassis):
        raise ValueError('Chassis collider identity/coverage mismatch')
    if any(_owner(source_stage.GetPrimAtPath(path)) is not None for path in source_chassis):
        raise ValueError('Original S300 must consist of independent static colliders')
    if any(_owner(stage.GetPrimAtPath(path)) != BASE_BODY for path in chassis):
        raise ValueError('Moving chassis is not owned by the exact world base')
    source_base = [path for path in _colliders(source_stage, BASE_BODY)
                   if _owner(source_stage.GetPrimAtPath(path)) == BASE_BODY]
    new_base = [path for path in _colliders(stage, BASE_BODY)
                if _owner(stage.GetPrimAtPath(path)) == BASE_BODY and path not in set(chassis)]
    platform = [path for path in _colliders(stage, PLATFORM)
                if _owner(stage.GetPrimAtPath(path)) == PLATFORM]
    old_platform = [path for path in _colliders(source_stage, PLATFORM)
                    if _owner(source_stage.GetPrimAtPath(path)) == PLATFORM]
    if set(source_base) != set(new_base) or set(platform) != set(old_platform) or not platform:
        raise ValueError('Original base/platform exact collider mapping changed')
    relevant = sorted(set(chassis + new_base + platform))
    before = _filtered_pairs(stage, relevant)
    retained = {tuple(sorted((a, b))) for a in source_base for b in platform}
    chassis_pairs = {tuple(sorted((a, b))) for a in chassis for b in platform}
    # Preserve all existing graph entries. Add only the old, genuinely original
    # base shapes' exact exclusions, never a chassis/body/root prefix exclusion.
    joint.CreateCollisionEnabledAttr(True)
    for path in source_base:
        relation = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(path)).CreateFilteredPairsRel()
        current = set(map(str, relation.GetTargets()))
        for target in platform:
            if target not in current:
                relation.AddTarget(target)
    after = _filtered_pairs(stage, relevant)
    if after != before | retained:
        raise RuntimeError('Exact lift-pair restoration changed unintended filter pairs')
    added = sorted(after - before)
    return dict(schema='s300-rail-platform-collision-parity-v1',
        joint=LIFT_JOINT, old_joint_collision_enabled=False, new_joint_collision_enabled=True,
        original_base_colliders=source_base, platform_colliders=platform,
        chassis_colliders=chassis, chassis_shape_count=len(chassis),
        platform_shape_count=len(platform), restored_old_base_shape_pairs=len(retained),
        added_exact_filtered_shape_pairs=[list(pair) for pair in added],
        chassis_platform_shape_pairs=len(chassis_pairs),
        preexisting_explicit_chassis_platform_exclusions=[list(pair) for pair in sorted(before & chassis_pairs)],
        newly_collision_enabled_chassis_platform_shape_pairs=len(chassis_pairs - after),
        preserved_exact_preexisting_filter_graph=True,
        collision_geometry_changed=False, collider_enabled_state_changed=False,
        arm_tool_joint_filters_changed=False,
        planner_note='Existing FCL adjacency screen honors the enabled lift joint; any original base-shape filteredPairs may require a stricter planner adapter. Actual baseline has no original base colliders.',
        native_validation='pending exact pair controls and initial-state A/B',
        training_eligible=False, harvest_success=None)


def revise(assembly, output, *, source_assembly=None):
    """Save an immutable corrected rail assembly as materialized USDC."""
    assembly, output = Path(assembly).resolve(), Path(output).resolve()
    manifest = json.loads((assembly / 'motion_manifest.json').read_text())
    source_scene = assembly / 'scene.usda'
    digest = file_sha256(source_scene)
    if digest != manifest['scene_sha256']:
        raise ValueError('Rail assembly SHA mismatch')
    verify_compact_dependencies(assembly, manifest)
    if 'rail' not in manifest or 'rail_filter_parity' in manifest or output.exists():
        raise ValueError('Use a checked rail assembly and a new output directory')
    baseline = Path(source_assembly or manifest['rail']['source_assembly']).resolve()
    baseline_manifest = json.loads((baseline / 'motion_manifest.json').read_text())
    baseline_sha = file_sha256(baseline / 'scene.usda')
    if baseline_sha != baseline_manifest['scene_sha256'] or \
            baseline_sha != manifest['rail']['source_scene_sha256']:
        raise ValueError('Original static-chassis source SHA mismatch')
    verify_compact_dependencies(baseline, baseline_manifest)
    original = Usd.Stage.Open(str(source_scene))
    baseline_stage = Usd.Stage.Open(str(baseline / 'scene.usda'))
    require_motion_units(original)
    require_motion_units(baseline_stage)
    stage = Usd.Stage.Open(original.Flatten(addSourceFileComment=False))
    copy_stage_metrics(original, stage)
    report = restore_platform_chassis_collision(stage, baseline_stage)
    properties = property_inventory(stage)
    output.mkdir(parents=True, exist_ok=False)
    flattened = stage.Flatten(addSourceFileComment=False)
    if flattened.subLayerPaths or any(flattened.GetExternalReferences()):
        raise ValueError('Corrected materialized layer retained external USD references')
    materialized = output / 'composed.usdc'
    flattened.Export(str(materialized))
    wrapper = Usd.Stage.CreateNew(str(output / 'scene.usda'))
    wrapper.GetRootLayer().subLayerPaths = ['composed.usdc']
    copy_stage_metrics(original, wrapper)
    if original.HasAuthoredTimeCodeRange():
        wrapper.SetStartTimeCode(original.GetStartTimeCode())
        wrapper.SetEndTimeCode(original.GetEndTimeCode())
    if original.GetDefaultPrim():
        wrapper.SetDefaultPrim(wrapper.GetPrimAtPath(original.GetDefaultPrim().GetPath()))
    wrapper.GetRootLayer().Save()
    if stage_metrics(original) != stage_metrics(wrapper) or properties != property_inventory(wrapper):
        raise RuntimeError('Corrected asset export changed resolved properties/units')
    geometry = compare_geometry(original, wrapper)
    (output / 'rail_filter_geometry_inventory.json').write_text(json.dumps(geometry, indent=2) + '\n')
    shutil.copy2(assembly / 'skin.npz', output / 'skin.npz')
    dependency = dict(relative_path='composed.usdc', sha256=file_sha256(materialized),
        format='usdc', materialized=True, external_usd_sublayers=[])
    report.update(source_rail_assembly=str(assembly), source_rail_scene_sha256=digest,
        original_static_assembly=str(baseline), original_static_scene_sha256=baseline_sha,
        materialized_dependency=dependency,
        geometry={k:v for k,v in geometry.items() if k not in ('colliders','rigid_bodies')})
    manifest['scene_sha256'] = file_sha256(output / 'scene.usda')
    manifest['rail_filter_parity'] = report
    manifest['compact_assets'] = dict(schema='s300-lossless-materialized-usdc-v1',
        source_assembly=str(assembly), source_scene_sha256=digest,
        source_manifest_sha256=file_sha256(assembly / 'motion_manifest.json'),
        materialized_dependency=dependency, geometry_unchanged=True, native_validation='pending',
        note='Exact original lift adjacency restored as shape pairs; composed storage is lossless')
    manifest['training_eligible'] = False
    (output / 'motion_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    (output / 'rail_filter_parity.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    if file_sha256(source_scene) != digest or file_sha256(baseline / 'scene.usda') != baseline_sha:
        raise RuntimeError('Source asset changed during collision parity revision')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('assembly', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(json.dumps(revise(args.assembly, args.output), indent=2))
