"""Immutable plant-interaction A/B assets for the existing native executor.

No alternate simulator or trajectory planner. Neighbor plants use build_plant,
compliant_foliage and the established leaf-ignore policy. The target-only
variant changes collisionEnabled only, not visual geometry or dynamics.
"""
import argparse
import copy
import json
import sys
import time
from pathlib import Path

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from compact_motion_assets import compact_assembly, file_sha256, verify_compact_dependencies
from motion_assets import REPO, build_plant, physx
from motion_replication import remap_path, _fix_targets, _snapshot, _compare
from motion_stage_metrics import copy_stage_metrics


NEIGHBORS = '/World/ElasticNeighbors'


def remap_values(value, mapping):
    if isinstance(value, str):
        return remap_path(value, mapping)
    if isinstance(value, list):
        return [remap_values(x, mapping) for x in value]
    if isinstance(value, dict):
        return {remap_path(k, mapping): remap_values(v, mapping) for k, v in value.items()}
    return value


def nearest_plants(stage, manifest):
    """Rank existing unvisited plants by nearest fruit center to target center."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'], useExtentsHint=False)
    center = np.asarray(manifest['target']['center'])
    rows = []
    for plant in stage.GetPrimAtPath('/World/Greenhouse/Plants').GetChildren():
        if not plant.IsActive() or str(plant.GetPath()) == manifest['plant']['source_plant']:
            continue
        fruits = []
        for prim in Usd.PrimRange(plant, Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdGeom.Mesh) and (prim.GetCustomDataByKey('sourceName') or '').startswith('Fruit_'):
                box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                c = np.asarray(box.GetMidpoint())
                fruits.append((float(np.linalg.norm(c-center)), str(prim.GetPath()), c.tolist()))
        if fruits:
            distance, path, c = min(fruits)
            rows.append(dict(source_plant=str(plant.GetPath()), nearest_fruit=path,
                             nearest_center=c, distance_m=distance))
    return sorted(rows, key=lambda x: (x['distance_m'], x['source_plant']))


def target_only_keep(path, target_root, target_truss):
    """Return None for nonplant paths; exact target stem/truss ownership only."""
    if path.startswith(target_root + '/'):
        body = path[len(target_root)+1:].split('/')[0]
        return body.startswith('Stem_') or body.startswith(target_truss + '_')
    if path.startswith(('/World/Greenhouse/Plants/', '/World/RouteFrozen/',
                        '/World/BackgroundCompliantFoliage/', NEIGHBORS + '/')):
        return False
    return None


def disable_other_plant_contacts(stage, manifest, *, include_already_disabled=False):
    root = manifest['plant']['root']
    truss = manifest['target']['name'].split('_Fruit_')[0]
    before = {}
    disabled = []
    specs = set()
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path = str(prim.GetPath())
        enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        before[path] = enabled
        if target_only_keep(path, root, truss) is False and (enabled is not False or include_already_disabled):
            stack = prim.GetPrimStack()
            if not stack or stack[0].layer != stage.GetRootLayer():
                raise ValueError('Self-contained copied stage required: ' + path)
            specs.add(str(stack[0].path))
            disabled.append(path)
    if not disabled:
        raise ValueError('No non-target plant contacts found')
    for path in sorted(specs):
        spec = stage.GetRootLayer().GetPrimAtPath(path)
        attr = spec.attributes.get('physics:collisionEnabled')
        if attr is None:
            attr = Sdf.AttributeSpec(spec, 'physics:collisionEnabled', Sdf.ValueTypeNames.Bool)
        attr.default = False
    selected = set(disabled)
    for path, old in before.items():
        actual = UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path)).GetCollisionEnabledAttr().Get()
        if actual != (False if path in selected else old):
            raise ValueError('Unexpected collider edit: ' + path)
    newly_disabled=sum(before[p] is not False for p in disabled)
    return dict(mode='target_truss_and_own_stem_only', target_root=root, target_truss=truss,
                disabled_colliders=disabled, disabled_count=len(disabled),
                newly_disabled_count=newly_disabled,
                enabled_before=sum(x is not False for x in before.values()),
                enabled_after=sum(x is not False for x in before.values())-newly_disabled,
                robot_house_gutter_preserved=True, visuals_and_dynamics_preserved=True,
                planner_and_native_same_saved_collision_states=True,
                interpretation='Non-target plants may be traversed; not obstacle avoidance success')


def inherit_target_only_contacts(stage, manifest, parent_policy):
    """Rebind the explicit policy AFTER a measured active-plant window switch.

    Build_plant creates the next target colliders. Old background omissions and
    newly frozen outgoing bodies must both be named in the fresh declaration.
    Copying the old list aliases /ElasticTarget names to different physical fruit.
    """
    if not parent_policy or parent_policy.get('mode') != 'target_truss_and_own_stem_only':
        return None
    policy=disable_other_plant_contacts(stage,manifest,include_already_disabled=True)
    policy.update(schema='s300-plant-interaction-variant-v1',
        inherited_after_measured_window_switch=True,
        previous_target_root=parent_policy['target_root'],previous_target_truss=parent_policy['target_truss'],
        previous_disabled_count=parent_policy['disabled_count'],
        training_eligible=False,harvest_success=None)
    manifest['plant_interaction_variant']=policy
    verified_disabled_colliders(stage,manifest)
    return policy


def verified_disabled_colliders(stage, manifest):
    """Exact omissions, never permissions for contacts with enabled geometry."""
    policy=manifest.get('plant_interaction_variant',{})
    if policy.get('mode')!='target_truss_and_own_stem_only':
        return frozenset()
    root=manifest['plant']['root'];truss=manifest['target']['name'].split('_Fruit_')[0]
    paths=policy.get('disabled_colliders',[])
    if (policy.get('target_root')!=root or policy.get('target_truss')!=truss or
            not paths or len(set(paths))!=len(paths) or policy.get('disabled_count')!=len(paths)):
        raise ValueError('Exact target-only collision declaration required')
    for path in paths:
        prim=stage.GetPrimAtPath(path)
        # prepare_display_meshes removes dormant APIs from disabled Skin/Visual
        # meshes in the session layer. Their explicit false attribute remains;
        # absence of the API cannot re-enable a collider. Still reject missing
        # prims, absent/true attributes and any target/structural declaration.
        if (target_only_keep(path,root,truss) is not False or not prim or
                prim.GetAttribute('physics:collisionEnabled').Get() is not False):
            raise ValueError('Invalid declared non-target disabled collider: '+path)
    return frozenset(paths)


def build_neighbor(source, source_plant, folder, template):
    """Build one placed plant using existing converters in a masked source stage."""
    folder.mkdir(parents=True, exist_ok=False)
    mask = Usd.StagePopulationMask([source_plant])
    masked = Usd.Stage.OpenMasked(str(source/'scene.usda'), mask)
    stage = Usd.Stage.Open(masked.Flatten())
    copy_stage_metrics(masked, stage)
    if str(REPO/'mujoco-benchmark/scripts') not in sys.path:
        sys.path.insert(0, str(REPO/'mujoco-benchmark/scripts'))
    report = {}
    build_plant(stage, source_plant, report, folder)
    if template.get('compliant_foliage'):
        from compliant_foliage import convert
        old = template['compliant_foliage']
        leaf = old['leaves'][0]
        report['compliant_foliage'] = convert(stage, stiffness=leaf['stiffness'],
            damping=leaf['damping'], armature=old['numerical_armature_kg_m2'])
        report['plant']['bodies'].extend(report['compliant_foliage']['leaves'])
    # Flatten reparented leaf bodies exactly as in the existing converter.
    stage = Usd.Stage.Open(stage.Flatten())
    if template.get('leaf_contact_policy', {}).get('mode') == 'visual_only_leaf_collision':
        from leaf_contact_policy import apply_policy
        report['leaf_contact_policy'] = apply_policy(stage, report)
    leaf_armature = {r['joint']: report['compliant_foliage']['numerical_armature_kg_m2']
                    for r in report.get('compliant_foliage', {}).get('leaves', [])}
    for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/ElasticTarget/Joints')):
        if prim.IsA(UsdPhysics.Joint):
            physx(prim, 'PhysxJointAPI', **{'physxJoint:armature':
                (Sdf.ValueTypeNames.Float, leaf_armature.get(str(prim.GetPath()), .0005))})
    stage.GetRootLayer().Export(str(folder/'plant.usdc'))
    (folder/'manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    return stage, report


def add_neighbor(stage, source, source_plant, output, manifest):
    name = Sdf.Path(source_plant).name
    root = NEIGHBORS + '/' + name
    if stage.GetPrimAtPath(root):
        raise ValueError('Duplicate neighbor: ' + root)
    started = time.perf_counter()
    original, report = build_neighbor(source, source_plant, output/('build_'+name), manifest)
    mapping = {'/World/ElasticTarget': root}
    UsdGeom.Xform.Define(stage, NEIGHBORS)
    Sdf.CopySpec(original.GetRootLayer(), '/World/ElasticTarget', stage.GetRootLayer(), root)
    _fix_targets(stage, root, mapping)
    # Compare physics, transform and topology after canonicalizing namespace.
    before = _snapshot(original, ['/World/ElasticTarget'])
    after = _snapshot(stage, [root])
    after = json.loads(json.dumps(after).replace('motion://'+name, 'motion://ElasticTarget'))
    proof = _compare(before, after, name)
    stage.OverridePrim(source_plant).SetActive(False)
    row = remap_values(report, mapping)
    row['skin_file'] = 'skin_'+name+'.npz'
    with np.load(output/('build_'+name)/'skin.npz') as arrays:
        values = {k: (np.array([remap_path(p, mapping) for p in arrays[k]])
                       if k.endswith('_body_paths') else arrays[k]) for k in arrays.files}
    np.savez_compressed(output/row['skin_file'], **values)
    row.update(source_plant=source_plant, root=root, conversion_proof=proof,
               build_wall_s=time.perf_counter()-started,
               fruit_attachment='Existing full converter fixed breakable joint, 3N/0.08Nm; no neighbor harvest-law tuning')
    return row


def revise(source, output, *, neighbors=(), target_only=False):
    source = Path(source).resolve(); output = Path(output).resolve()
    if neighbors and target_only:
        raise ValueError('Separate neighbor dynamics and target-only contact experiments')
    if len(neighbors) > 2 or len(set(neighbors)) != len(neighbors):
        raise ValueError('This bounded comparison supports at most two distinct neighbors')
    started = time.perf_counter()
    hashes = {name:file_sha256(source/name) for name in ('scene.usda','composed.usdc','motion_manifest.json','skin.npz')}
    compact_assembly(source, output)
    manifest = json.loads((output/'motion_manifest.json').read_text())
    if manifest.get('elastic_neighbors'):
        raise ValueError('Source must be the common single-active-plant baseline')
    stage = Usd.Stage.Open(str(output/'composed.usdc'))
    policy = dict(schema='s300-plant-interaction-variant-v1',source_assembly=str(source),
                  source_sha256=hashes, mode='current_baseline',training_eligible=False,harvest_success=None)
    if target_only:
        policy.update(disable_other_plant_contacts(stage, manifest))
    if neighbors:
        manifest['elastic_neighbors'] = []
        for path in neighbors:
            if path == manifest['plant']['source_plant'] or not stage.GetPrimAtPath(path).IsActive():
                raise ValueError('Neighbor must be an existing unvisited non-target plant')
            manifest['elastic_neighbors'].append(add_neighbor(stage, source, path, output, manifest))
        policy.update(mode='elastic_neighbors', neighbor_sources=list(neighbors))
    stage.GetRootLayer().Save()
    policy['preparation_wall_s'] = time.perf_counter()-started
    manifest['plant_interaction_variant'] = policy
    manifest['compact_assets']['materialized_dependency']['sha256'] = file_sha256(output/'composed.usdc')
    manifest['compact_assets'].update(native_validation='pending',
        note='Immutable copied asset with explicit plant-interaction variant; original copy proof applies before these declared edits')
    manifest.update(training_eligible=False, harvest_success=None)
    (output/'motion_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output/'plant_interaction_variant.json').write_text(json.dumps(policy, indent=2)+'\n')
    verify_compact_dependencies(output)
    if hashes != {name:file_sha256(source/name) for name in hashes}:
        raise ValueError('Source asset modified')
    return policy


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--neighbors',nargs='*',default=[]);p.add_argument('--target-only',action='store_true')
    a=p.parse_args();r=revise(a.source,a.output,neighbors=a.neighbors,target_only=a.target_only)
    print(json.dumps({k:v for k,v in r.items() if k!='disabled_colliders'},indent=2))
