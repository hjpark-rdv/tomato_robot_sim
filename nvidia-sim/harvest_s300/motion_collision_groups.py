"""Compress an existing leaf exclusion graph into disjoint collision groups.

The production model already disables every leaf-collider/leaf-collider pair.
The isolated component fixture additionally disables leaf/nonleaf active-plant
pairs and all background-plant/active-system pairs. This adapter represents
exactly those policies with groups, preserving every other authored pair.
It changes neither collision shapes nor dynamics/material parameters.
"""
from __future__ import annotations

import hashlib
import itertools
import json

from pxr import Sdf, Usd, UsdPhysics

from motion_replication import ENVS_ROOT, SOURCE_ROOTS, _compare, _snapshot


GROUP_ROOT = '/World/FixtureGroups'


def _pair(a, b):
    return tuple(sorted((str(a), str(b))))


def _enabled_colliders(stage, roots):
    paths = []
    for root in roots:
        prim = stage.GetPrimAtPath(root)
        if not prim:
            continue
        for candidate in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            if candidate.HasAPI(UsdPhysics.CollisionAPI) and \
                    UsdPhysics.CollisionAPI(candidate).GetCollisionEnabledAttr().Get() is not False:
                paths.append(str(candidate.GetPath()))
    return sorted(paths)


def _filtered_pairs(stage, collider_paths):
    """Expand body-or-shape filteredPairs into enabled collider pair identities."""
    expansion_cache = {}
    def expand(path):
        path = str(path)
        if path not in expansion_cache:
            expansion_cache[path] = [p for p in collider_paths if p == path or p.startswith(path + '/')]
        return expansion_cache[path]
    pairs = set()
    for prim in stage.Traverse():
        rel = prim.GetRelationship('physics:filteredPairs')
        if not rel or not prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            continue
        left = expand(prim.GetPath())
        if not left:
            continue
        for target in rel.GetTargets():
            for a in left:
                for b in expand(target):
                    if a != b:
                        pairs.add(_pair(a, b))
    return pairs


def _group_pairs(stage, collider_paths):
    groups = {}
    memberships = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.CollisionGroup):
            continue
        group = UsdPhysics.CollisionGroup(prim)
        members = set()
        for root in group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets():
            root = str(root)
            members.update(p for p in collider_paths if p == root or p.startswith(root + '/'))
        for path in members:
            if path in memberships:
                raise ValueError(f'Collider belongs to multiple collision groups: {path}')
            memberships[path] = str(group.GetPath())
        groups[str(group.GetPath())] = (members, group.GetFilteredGroupsRel().GetTargets())
    pairs = set()
    for path, (members, targets) in groups.items():
        for target in targets:
            other = groups.get(str(target), (set(), []))[0]
            if str(target) == path:
                pairs.update(_pair(a, b) for a, b in itertools.combinations(sorted(members), 2))
            else:
                pairs.update(_pair(a, b) for a in members for b in other if a != b)
    return pairs


def _digest(pairs):
    return hashlib.sha256(json.dumps(sorted(pairs), separators=(',', ':')).encode()).hexdigest()


def compress_leaf_collision_policy(stage, *, isolate_background=False):
    """Return exact policy equivalence evidence; native A/B remains required.

    Call before the legacy fixture authors leaf/nonleaf pair lists, and before
    replication/PhysX initialization. Production use compresses only the
    already-authored complete leaf/leaf graph. The opt-in component fixture
    retains its separately labeled additional exclusions.
    """
    if stage.GetPrimAtPath(ENVS_ROOT):
        raise ValueError('Compress source collision policy before replication')
    scene = stage.GetPrimAtPath('/World/PhysicsScene')
    inverted = scene.GetAttribute('physxScene:invertCollisionGroupFilter') if scene else None
    if inverted and inverted.Get() is True:
        raise ValueError('Inverted collision groups require a separate policy adapter')
    plant = _enabled_colliders(stage, ['/World/ElasticTarget'])
    leaves = [path for path in plant if '/Appendage_' in path]
    others = [path for path in plant if path not in set(leaves)]
    if not leaves or not others:
        raise ValueError('Active plant leaf/nonleaf collider inventory is missing')
    leaf_set = set(leaves)
    existing_pairs = _filtered_pairs(stage, plant) | _group_pairs(stage, plant)
    leaf_pairs = {_pair(a, b) for a, b in itertools.combinations(leaves, 2)}
    missing = leaf_pairs - existing_pairs
    if missing:
        raise ValueError(f'Original leaf/leaf exclusions are not complete: {len(missing)} missing')
    expected = existing_pairs | ({_pair(a, b) for a in leaves for b in others}
                                 if isolate_background else set())
    before = _snapshot(stage, SOURCE_ROOTS)
    root_text = stage.GetRootLayer().ExportToString()
    original_pair_targets = {}
    removals = {}
    for path in leaves:
        prim = stage.GetPrimAtPath(path)
        rel = prim.GetRelationship('physics:filteredPairs')
        targets = rel.GetTargets() if rel else []
        original_pair_targets[path] = [str(target) for target in targets]
        removals[path] = [target for target in targets if str(target) in leaf_set]
    active = stage.GetPrimAtPath(GROUP_ROOT + '/Active')
    if active:
        includes = UsdPhysics.CollisionGroup(active).GetCollidersCollectionAPI().GetIncludesRel().GetTargets()
        if set(map(str, includes)) != set(SOURCE_ROOTS):
            raise ValueError('Existing Active group does not cover the expected three dynamic roots')
    # No newly authored group may overlap a collider's existing membership.
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.CollisionGroup) or str(prim.GetPath()) == GROUP_ROOT + '/Active':
            continue
        includes = UsdPhysics.CollisionGroup(prim).GetCollidersCollectionAPI().GetIncludesRel().GetTargets()
        if any(any(p == str(root) or p.startswith(str(root) + '/') for p in plant)
               for root in includes):
            raise ValueError(f'Existing collision group overlaps active plant: {prim.GetPath()}')
    new_paths = [GROUP_ROOT + '/Leaves', GROUP_ROOT + '/ActivePlantOther', GROUP_ROOT + '/RobotTool']
    if any(stage.GetPrimAtPath(path) for path in new_paths):
        raise ValueError('Compressed collision policy groups already exist')
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for path, targets in removals.items():
            if targets:
                remaining = [Sdf.Path(target) for target in original_pair_targets[path]
                             if target not in leaf_set]
                stage.GetPrimAtPath(path).GetRelationship('physics:filteredPairs').SetTargets(remaining)
        groups = [UsdPhysics.CollisionGroup.Define(stage, path) for path in new_paths]
        for group, members in zip(groups, [leaves, others, list(SOURCE_ROOTS[:2])]):
            group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(members)
        groups[0].CreateFilteredGroupsRel().SetTargets(
            [groups[0].GetPath()] + ([groups[1].GetPath()] if isolate_background else []))
        if active:
            old = UsdPhysics.CollisionGroup(active)
            old_filters = old.GetFilteredGroupsRel().GetTargets()
            old.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([])
            for group in groups:
                for target in old_filters:
                    group.CreateFilteredGroupsRel().AddTarget(target)
            for prim in list(stage.Traverse()):
                if not prim.IsA(UsdPhysics.CollisionGroup):
                    continue
                rel = UsdPhysics.CollisionGroup(prim).GetFilteredGroupsRel()
                targets = rel.GetTargets()
                if old.GetPath() in targets:
                    rel.SetTargets([target for target in targets if target != old.GetPath()] +
                                   [group.GetPath() for group in groups])
        if isolate_background:
            background = UsdPhysics.CollisionGroup.Define(stage, GROUP_ROOT + '/Background')
            background.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(
                ['/World/Greenhouse/Plants'])
            background.CreateFilteredGroupsRel().SetTargets([group.GetPath() for group in groups])
    actual = _filtered_pairs(stage, plant) | _group_pairs(stage, plant)
    if expected != actual:
        raise RuntimeError(f'Compressed policy differs: missing={len(expected-actual)}, '
                           f'new={len(actual-expected)}')
    after = _snapshot(stage, SOURCE_ROOTS)
    # Only the representation of this explicitly verified pair graph changes.
    # Other physics relationships and every physical attribute stay identical.
    for rows in (before, after):
        for row in rows.values():
            row['rels'].pop('physics:filteredPairs', None)
    conservation = _compare(before, after, 'pair-list -> collision groups')
    if stage.GetRootLayer().ExportToString() != root_text:
        raise RuntimeError('Collision group compression modified source root layer')
    retained_targets = sum(len(prim.GetRelationship('physics:filteredPairs').GetTargets())
                           for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/ElasticTarget'))
                           if prim.HasAPI(UsdPhysics.FilteredPairsAPI))
    return dict(policy='same leaf exclusions represented by disjoint native groups',
                leaves=leaves, other_colliders=others, groups=new_paths,
                isolate_background=bool(isolate_background), fixture_only=bool(isolate_background),
                original_plant_excluded_pairs=len(existing_pairs),
                expected_plant_excluded_pairs=len(expected), actual_plant_excluded_pairs=len(actual),
                expected_pair_sha256=_digest(expected), actual_pair_sha256=_digest(actual),
                removed_leaf_leaf_targets=sum(map(len, removals.values())),
                retained_filtered_pairs=len(_filtered_pairs(stage, plant)),
                retained_filtered_relationship_targets=retained_targets,
                retained_pair_metric='effective enabled-collider pairs; a body relation may cover multiple colliders',
                added_leaf_nonleaf_pair_lists=0, exact_excluded_pair_graph_match=True,
                dynamics_and_geometry=conservation, source_assets_saved=False,
                original_leaf_pair_targets_sha256=hashlib.sha256(json.dumps(
                    original_pair_targets, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
                native_validation='pending CPU/GPU A-B; authored policy equivalence only',
                training_eligible=False)
