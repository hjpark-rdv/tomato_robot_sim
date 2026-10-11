"""Explicit visual-only leaf contact policy on a separate motion assembly.

This approximates negligible leaf resistance; it does not simulate leaf folding.
Stem, cut branches, fruit, calyx, rachis, pedicels and the robot keep their physics.
"""
import argparse
import json
from pathlib import Path
from pxr import Sdf, Usd, UsdPhysics
from compliant_foliage import canonical_leaf_group
from compact_motion_assets import compact_assembly, file_sha256, verify_compact_dependencies


def collision_inventory(stage):
    return {str(p.GetPath()): p for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
            if p.HasAPI(UsdPhysics.CollisionAPI)}


def apply_policy(stage, manifest):
    """Modify exact collisionEnabled opinions, including shared source instances."""
    declared = {}
    for key in ('compliant_foliage', 'background_compliant_foliage'):
        for row in manifest.get(key, {}).get('leaves', []):
            group = canonical_leaf_group(row.get('source_name') or row.get('source_group') or '')
            if not group:
                raise ValueError('Unrecognized declared leaf identity')
            for path in row.get('meshes', []):
                if not path.startswith(row['path'] + '/'):
                    raise ValueError('Declared leaf mesh escaped its owner')
                declared[path] = group
    before = collision_inventory(stage)
    states = {path: UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() for path, p in before.items()}
    selected = {}
    for path, prim in before.items():
        name = prim.GetCustomDataByKey('sourceName') or ''
        group = canonical_leaf_group(name) or declared.get(path)
        in_plant = path.startswith(('/World/ElasticTarget/', '/World/Greenhouse/Plants/', '/World/BackgroundCompliantFoliage/'))
        if group and in_plant and states[path] is not False:
            # An instance proxy cannot accept USD edits. Author the exact source
            # collider's existing strongest opinion in this copied materialized layer.
            stack = prim.GetPrimStack()
            if not stack or stack[0].layer != stage.GetRootLayer():
                raise ValueError('Self-contained materialized collider required: ' + path)
            selected[path] = dict(group=group, source_name=name, spec_path=str(stack[0].path))
    if not selected:
        raise ValueError('No exact enabled leaf colliders')
    for spec_path in sorted({row['spec_path'] for row in selected.values()}):
        spec = stage.GetRootLayer().GetPrimAtPath(spec_path)
        attr = spec.attributes.get('physics:collisionEnabled')
        if attr is None:
            attr = Sdf.AttributeSpec(spec, 'physics:collisionEnabled', Sdf.ValueTypeNames.Bool)
        attr.default = False
    after = collision_inventory(stage)
    if before.keys() != after.keys():
        raise ValueError('Policy changed collider topology')
    for path, prim in after.items():
        expected = False if path in selected else states[path]
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() != expected:
            raise ValueError('Unexpected collider change: ' + path)
    return dict(schema='s300-leaf-contact-ignore-v1', mode='visual_only_leaf_collision',
                exact_disabled_colliders=selected, disabled_count=len(selected),
                unchanged_nonleaf_and_already_disabled_count=len(before)-len(selected),
                source_spec_count=len({r['spec_path'] for r in selected.values()}),
                topology_preserved=True, visuals_preserved=True,
                nonleaf_collision_states_preserved=True, nonleaf_penetration_limit_m=.0005,
                approximation='Leaf resistance omitted, not a validated flexible leaf contact model',
                training_eligible=False, harvest_success=None)


def revise(source, output):
    source = Path(source).resolve(); output = Path(output).resolve()
    hashes = {name: file_sha256(source/name) for name in ('scene.usda', 'composed.usdc', 'skin.npz', 'motion_manifest.json')}
    # Reuse the existing verified immutable asset copier and executor format.
    compact_assembly(source, output)
    manifest = json.loads((output/'motion_manifest.json').read_text())
    stage = Usd.Stage.Open(str(output/'composed.usdc'))
    report = apply_policy(stage, manifest)
    report.update(source_assembly=str(source), source_sha256=hashes)
    stage.GetRootLayer().Save()
    manifest['leaf_contact_policy'] = report
    manifest['compact_assets']['materialized_dependency']['sha256'] = file_sha256(output/'composed.usdc')
    manifest['compact_assets']['note'] = 'Copied storage, then explicit leaf collision policy applied; leaf visual geometry retained, leaf physics omitted.'
    manifest['compact_assets']['native_validation'] = 'pending'
    manifest.update(training_eligible=False, harvest_success=None)
    (output/'motion_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    (output/'leaf_contact_policy.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    verify_compact_dependencies(output)
    if hashes != {name: file_sha256(source/name) for name in hashes}:
        raise ValueError('Source assembly mutated')
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('source',type=Path); parser.add_argument('output',type=Path)
    args=parser.parse_args(); result=revise(args.source,args.output)
    print(json.dumps({k:v for k,v in result.items() if k!='exact_disabled_colliders'},indent=2))
