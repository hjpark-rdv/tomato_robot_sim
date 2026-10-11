"""Experimental cooking refinement of exactly eight declared Plant20 leaf meshes.

The six cooking values come from the existing CAD accuracy experiment, but its
native certificate does not apply to these leaves. This helper preserves source
surfaces, joints, masses, filters, materials and offsets. Native cooking,
background contact validity, stability and performance remain unverified.
"""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from background_foliage import declared_bodies
from collision_proxy_accuracy import API, CONFIG, PREFIX, TYPES, _normalized_inventory
from compact_motion_assets import (compact_assembly, compare_geometry, file_sha256,
                                   property_inventory, verify_compact_dependencies)
from harvest_contact_scope import background_leaf_colliders_from_manifest


EXACT_SOURCE_STEM = '/World/Greenhouse/Plants/Plant_20/Stem'
EXACT_ROOT = '/World/BackgroundCompliantFoliage/Plant_20'
EXACT_MAPPING = {
    f'{EXACT_SOURCE_STEM}/mesh_{index}':
        f'{EXACT_ROOT}/Leaf_CompoundLeaf_{group:02}/Appendage_mesh_{index}'
    for group, indices in ((1, range(127, 131)), (2, range(131, 135)))
    for index in indices
}
EXACT_PATHS = list(EXACT_MAPPING.values())


def _selected(stage, manifest):
    """Accept complete declared source mapping, never a leaf-looking path alone."""
    report = manifest.get('background_compliant_foliage', {})
    if report.get('path_mapping') != EXACT_MAPPING:
        raise ValueError('Exactly the eight original Plant20 Leaf01/02 mappings are required')
    inventory = background_leaf_colliders_from_manifest(stage, manifest)
    bodies = declared_bodies(stage, manifest)
    expected_bodies = {f'{EXACT_ROOT}/Leaf_CompoundLeaf_{i:02}' for i in (1, 2)}
    if set(inventory) != set(EXACT_PATHS) or set(bodies) != expected_bodies:
        raise ValueError('Exact complete eight-mesh/two-world-D6-body inventory required')
    for path in EXACT_PATHS:
        prim = stage.GetPrimAtPath(path)
        body = stage.GetPrimAtPath(inventory[path]['body'])
        if (not prim.IsA(UsdGeom.Mesh) or not body.HasAPI(UsdPhysics.RigidBodyAPI) or
                UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get() is True or
                UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() != 'convexDecomposition'):
            raise ValueError('Exact enabled dynamic convex-decomposition leaf identity required')
    return inventory


def audit(source, derived, manifest):
    """Restore only six fields/API on a throwaway stage; compare every property."""
    _selected(source, manifest); _selected(derived, manifest)
    expected_values = {key: float(np.float32(value)) if type(value) is float else value
                       for key, value in CONFIG.items()}
    attributes = {}
    for path in EXACT_PATHS:
        old, new = source.GetPrimAtPath(path), derived.GetPrimAtPath(path)
        tokens = old.GetMetadata('apiSchemas')
        expected = tokens.ApplyOperations([]) if tokens else []
        if API not in expected: expected.append(API)
        authored = new.GetMetadata('apiSchemas')
        if authored is None or authored.ApplyOperations([]) != expected:
            raise ValueError('Applied schemas changed beyond the exact cooking API')
        values = {key: new.GetAttribute(PREFIX + key).Get() for key in CONFIG}
        if values != expected_values:
            raise ValueError('Cooking values differ from the declared experimental configuration')
        attributes[path] = values
    original = property_inventory(source)
    restored = _normalized_inventory(source, derived, EXACT_PATHS)
    if restored != original:
        raise ValueError('Properties changed beyond eight exact leaf cooking APIs and fields')
    geometry = compare_geometry(source, derived)
    return dict(source_property_inventory=original, normalized_derived_property_inventory=restored,
                non_cooking_properties_preserved=True, authored_triangles_retained=True,
                geometry_audit={key: value for key, value in geometry.items()
                                if key not in ('colliders', 'rigid_bodies')},
                actual_cooking_attributes=attributes)


def apply_accuracy(stage, manifest):
    _selected(stage, manifest)
    before = Usd.Stage.Open(stage.Flatten(addSourceFileComment=False))
    changes = []
    for path in EXACT_PATHS:
        prim = stage.GetPrimAtPath(path)
        old = {key: prim.GetAttribute(PREFIX + key).Get() for key in CONFIG}
        schemas = prim.GetMetadata('apiSchemas')
        tokens = schemas.ApplyOperations([]) if schemas else []
        if API not in tokens: tokens.append(API)
        prim.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(tokens))
        for key, value in CONFIG.items():
            prim.CreateAttribute(PREFIX + key, TYPES[key], custom=False).Set(value)
        changes.append(dict(path=path, before=old, after=copy.deepcopy(CONFIG)))
    proof = audit(before, stage, manifest)
    return dict(schema='s300-exact-passive-leaf-proxy-accuracy-hypothesis-v1', enabled=True,
                exact_collider_paths=EXACT_PATHS.copy(), exact_source_mapping=copy.deepcopy(EXACT_MAPPING),
                configuration=copy.deepcopy(CONFIG), collider_changes=changes,
                allowed_api_schema=API, allowed_attribute_names=[PREFIX + key for key in CONFIG],
                physical_proxy_representation_changed=True, dynamics_properties_changed=False,
                contact_authorization_unchanged=True, penetration_limit_m=.0005,
                native_cooking_validation='pending_exact_leaf_positive_negative_controls',
                native_stability_validation='pending', native_performance_validation='pending',
                cad_fixture_certificate_applied=False, full_source_triangle_enclosure_proven=False,
                training_eligible=False, harvest_success=None, **proof)


def revise(assembly, output):
    assembly, output = map(lambda value: Path(value).resolve(), (assembly, output))
    if output.exists(): raise FileExistsError(output)
    manifest = json.loads((assembly / 'motion_manifest.json').read_text())
    if file_sha256(assembly / 'scene.usda') != manifest.get('scene_sha256'):
        raise ValueError('Source scene SHA mismatch')
    verify_compact_dependencies(assembly, manifest)
    original = Usd.Stage.Open(str(assembly / 'composed.usdc'))
    _selected(original, manifest)
    hashes = {str(assembly / name): file_sha256(assembly / name)
              for name in ('scene.usda', 'composed.usdc', 'motion_manifest.json', 'skin.npz')}
    compact_assembly(assembly, output)
    derived = Usd.Stage.Open(str(output / 'composed.usdc'))
    report = apply_accuracy(derived, manifest)
    derived.GetRootLayer().Save()
    # A fresh on-disk read verifies that serialization retained the audited stage.
    serialized = Usd.Stage.Open(Sdf.Layer.OpenAsAnonymous(str(output / 'composed.usdc')))
    if property_inventory(serialized) != property_inventory(derived):
        raise ValueError('Materialized leaf revision changed effective properties')
    report.update(source_assembly=str(assembly), source_file_sha256=hashes,
                  source_scene_sha256=manifest['scene_sha256'],
                  source_manifest_sha256=hashes[str(assembly / 'motion_manifest.json')])
    copied = json.loads((output / 'motion_manifest.json').read_text())
    revised = copy.deepcopy(manifest)
    revised['compact_assets'] = copied['compact_assets']
    revised['compact_assets']['materialized_dependency']['sha256'] = file_sha256(output / 'composed.usdc')
    revised['compact_assets']['note'] = 'Lossless storage of explicit eight-leaf experimental cooking revision; native leaf proof pending.'
    revised['scene_sha256'] = file_sha256(output / 'scene.usda')
    revised['background_leaf_proxy_accuracy'] = report
    (output / 'motion_manifest.json').write_text(json.dumps(revised, indent=2, allow_nan=False) + '\n')
    (output / 'background_leaf_proxy_accuracy_validation.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for path, digest in hashes.items():
        if file_sha256(path) != digest: raise ValueError('Immutable source changed during leaf revision')
    verify_compact_dependencies(output)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(json.dumps(revise(args.assembly, args.output), indent=2))
