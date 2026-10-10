"""Opt-in joint-resolution diagnostic for the existing full S300 plant.

All authored bodies, mass/inertia, collider geometry, materials and contact
filters remain present. This variant locks springs on non-target trusses and
therefore changes plant dynamics. It is not a full-model speedup or a replacement
for the full plant. The original USD is never saved or rewritten.
"""
from __future__ import annotations

from collections import Counter
import hashlib
import json
import re

from pxr import Usd, UsdPhysics

from motion_replication import _snapshot


ROTATION_AXES = ('rotX', 'rotY', 'rotZ')


def _spring_axes(prim):
    if not prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.FixedJoint):
        return []
    return [axis for axis in ROTATION_AXES if prim.HasAPI(UsdPhysics.DriveAPI, axis)]


def _resolution_snapshot(stage, root):
    # The existing clone inventory covers bodies/shapes, standard physics and
    # PhysX attributes. Include multi-apply joint axes explicitly here too.
    result = _snapshot(stage, (root,))
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        key = 'motion://' + root.rsplit('/', 1)[-1] + str(prim.GetPath())[len(root):]
        for attr in prim.GetAttributes():
            if attr.GetName().startswith(('limit:', 'drive:', 'state:')) and attr.HasValue():
                result[key]['attrs'][attr.GetName()] = attr.Get()
    return result


def _preserved_snapshot(snapshot, changed_keys):
    """Reuse the existing exact geometry/physics inventory, omitting overrides."""
    result = {}
    for key, original in snapshot.items():
        row = dict(original)
        attrs = dict(row['attrs'])
        if key in changed_keys:
            for axis in ROTATION_AXES:
                for suffix in ('low', 'high'):
                    attrs.pop(f'limit:{axis}:physics:{suffix}', None)
        row['attrs'] = attrs
        result[key] = row
    return result


def lock_far_trusses(stage, manifest):
    """Lock only non-target truss spring axes in the anonymous session layer.

    Main-stem and every spring in the selected target truss stay full. Harvest
    joints remain native breakable fixed joints. No collision is disabled and no
    rigid body is fused or removed in USD. Native link identity must still be
    checked after initialization before using the existing preload/body views.

    The prior ``light keep`` model establishes that locking distant hinges while
    retaining collider bodies is a useful diagnostic. This variant is different:
    it locks every non-target truss spring and does not rescale any remaining
    stiffness/damping or ignore appendages. Equivalence requires actual tests.
    """
    root = str(manifest['plant']['root']).rstrip('/')
    if not root.startswith('/World/') or not stage.GetPrimAtPath(root):
        raise ValueError('Existing active plant root is required')
    target = manifest['target']
    target_path = str(target['path'])
    if not target_path.startswith(root + '/'):
        raise ValueError('Target is outside the active plant')
    target_name = target_path.rsplit('/', 1)[-1]
    matched = re.fullmatch(r'(Truss_\d+)_Fruit_\d+', target_name)
    if not matched or target.get('name', target_name) != target_name:
        raise ValueError('Target must identify an exact placed truss fruit')
    target_prefix = matched.group(1) + '_'
    anchor_path = str(target['anchor'])
    if not anchor_path.startswith(root + '/' + target_prefix):
        raise ValueError('Target fruit and attachment anchor have different truss identities')
    if not stage.GetPrimAtPath(target_path) or not stage.GetPrimAtPath(anchor_path):
        raise ValueError('Target fruit or attachment body is missing')
    joints_root = stage.GetPrimAtPath(root + '/Joints')
    if not joints_root:
        raise ValueError('Existing elastic joint namespace is required')
    affected = []
    kept = []
    originals = []
    counts = Counter()
    for prim in Usd.PrimRange(joints_root):
        axes = _spring_axes(prim)
        if not axes:
            continue
        if axes != list(ROTATION_AXES):
            raise ValueError(f'Incomplete three-axis spring: {prim.GetPath()}')
        name = prim.GetName()
        body1 = UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()
        if list(map(str, body1)) != [root + '/' + name]:
            raise ValueError(f'Joint/body identity does not match the original plant: {prim.GetPath()}')
        for axis in axes:
            limit = UsdPhysics.LimitAPI(prim, axis)
            low, high = limit.GetLowAttr().Get(), limit.GetHighAttr().Get()
            if low is None or high is None or low >= high:
                raise ValueError(f'Expected original unlocked spring axis: {prim.GetPath()} {axis}')
        if name.startswith('Stem_') or name.startswith(target_prefix):
            kept.append(str(prim.GetPath()))
            continue
        truss = re.match(r'^(Truss_\d+)_', name)
        if not truss:
            raise ValueError(f'Unclassified elastic spring: {prim.GetPath()}')
        affected.append(prim)
        counts[truss.group(1)] += 1
        originals.append(dict(path=str(prim.GetPath()), child_body=str(body1[0]),
                              truss=truss.group(1), axes={axis: dict(
                                  low=UsdPhysics.LimitAPI(prim, axis).GetLowAttr().Get(),
                                  high=UsdPhysics.LimitAPI(prim, axis).GetHighAttr().Get())
                                  for axis in axes}))
    if not affected or not any(path.rsplit('/', 1)[-1].startswith(target_prefix) for path in kept):
        raise ValueError('Both the complete target truss and a non-target truss are required')
    before = _resolution_snapshot(stage, root)
    changed_keys = {f'motion://{root.rsplit("/", 1)[-1]}/Joints/{prim.GetName()}' for prim in affected}
    preserved_before = _preserved_snapshot(before, changed_keys)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in affected:
            for axis in ROTATION_AXES:
                limit = UsdPhysics.LimitAPI(prim, axis)
                limit.CreateLowAttr(1.)
                limit.CreateHighAttr(-1.)
    preserved_after = _preserved_snapshot(_resolution_snapshot(stage, root), changed_keys)
    if preserved_before != preserved_after:
        raise RuntimeError('Joint locking changed preserved body/collision/physics inventory')
    digest = hashlib.sha256(json.dumps(preserved_before, sort_keys=True).encode()).hexdigest()
    return dict(variant='lock_far_trusses', different_dynamics=True,
                target_path=target_path, target_truss_prefix=target_prefix,
                affected_joints=originals, affected_joint_count=len(affected),
                affected_joints_by_truss=dict(counts),
                removed_rotational_dofs=3 * len(affected),
                original_articulation_rotational_dofs=3 * (len(affected) + len(kept)),
                expected_articulation_rotational_dofs=3 * len(kept),
                retained_full_joint_paths=kept,
                preservation=dict(passed=True, source_assets_saved=False,
                                  original_body_paths_and_mass_inertia=True,
                                  original_collider_geometry_materials_filters=True,
                                  original_drives_and_break_limits=True,
                                  except_rotational_limit_sha256=digest,
                                  authored_physics_object_count=len(before)),
                edit_layer=stage.GetSessionLayer().identifier,
                approximation='All non-target truss bending locked; main stem and entire target truss remain full',
                previous_light_model='LIGHT_PLANT.md light keep: distant hinge locking with collider bodies retained; this is not the same resolution policy',
                native_validation=dict(link_identity='pending initialization check',
                                       mass_and_collider_coverage='pending native check',
                                       target_deflection_and_recovery='pending full/variant pulse comparison'),
                harvest_success=None, training_eligible=False)
