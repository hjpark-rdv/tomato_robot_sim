"""Opt-in native PhysX copies of the existing S300 motion model.

Only the robot, tool and active plant are copied. Immutable greenhouse and
ground colliders stay in their original positions and are shared by co-located
environments. This is an identical-scene benchmark adapter, not a random-scene
generator. No collider or physical parameter is removed or simplified.

Call ``replicate_motion`` after runtime USD overrides and before initializing
PhysX. Edits are made in the session layer and are not saved to source assets.
"""
from __future__ import annotations

import hashlib
import json
import math

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


SOURCE_ROOTS = ('/World/HarvestRobot', '/World/ActuatedTool', '/World/ElasticTarget')
ENVS_ROOT = '/World/MotionEnvs'


def remap_path(path, mapping):
    """Replace a complete root prefix, including property paths if present."""
    value = str(path)
    for old, new in sorted(mapping.items(), key=lambda item: -len(item[0])):
        if value == old or value.startswith(old + '/') or value.startswith(old + '.'):
            return new + value[len(old):]
    return value


def _canonical(value, roots):
    if isinstance(value, Sdf.Path):
        return remap_path(value, {root: 'motion://' + root.rsplit('/', 1)[1] for root in roots})
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    try:
        return [_canonical(item, roots) for item in value]
    except TypeError:
        return str(value)


def _geometry_digest(prim):
    fields = ('points', 'faceVertexCounts', 'faceVertexIndices', 'radius',
              'height', 'axis', 'size', 'widths', 'curveVertexCounts')
    h = hashlib.sha256()
    for name in fields:
        attr = prim.GetAttribute(name)
        if not attr or not attr.HasValue():
            continue
        h.update(name.encode())
        value = attr.Get()
        try:
            array = np.asarray(value)
            if array.dtype.kind in 'bifu':
                h.update(str(array.dtype).encode())
                h.update(str(array.shape).encode())
                h.update(array.tobytes())
                continue
        except (ValueError, TypeError):
            pass
        h.update(str(value).encode())
    return h.hexdigest()


def _snapshot(stage, roots):
    """Physics inventory with exact shape data and separate world matrices."""
    cache = UsdGeom.XformCache()
    rows = {}
    for root in roots:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
            kinds = []
            if prim.HasAPI(UsdPhysics.CollisionAPI): kinds.append('collider')
            if prim.HasAPI(UsdPhysics.RigidBodyAPI): kinds.append('body')
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI): kinds.append('articulation')
            if prim.IsA(UsdPhysics.Joint): kinds.append('joint')
            if not kinds:
                continue
            key = remap_path(prim.GetPath(), {r: 'motion://' + r.rsplit('/', 1)[1] for r in roots})
            attrs = {}
            for attr in prim.GetAttributes():
                name = attr.GetName()
                if (name.startswith(('physics:', 'physx')) or ':physics:' in name) and attr.HasValue():
                    attrs[name] = _canonical(attr.Get(), roots)
            rels = {}
            for rel in prim.GetRelationships():
                name = rel.GetName()
                if name.startswith(('physics:', 'physx')) or ':physics:' in name or name.endswith(':physics'):
                    rels[name] = [_canonical(path, roots) for path in rel.GetTargets()]
            schema_op = prim.GetMetadata('apiSchemas')
            rows[key] = dict(kinds=kinds, type=prim.GetTypeName(),
                             applied_schemas=sorted(schema_op.GetAppliedItems()) if schema_op else [],
                             attrs=attrs, rels=rels,
                             geometry_sha256=_geometry_digest(prim) if 'collider' in kinds else None,
                             world=np.asarray(cache.GetLocalToWorldTransform(prim), float).tolist())
    return rows


def _compare(reference, actual, label, tolerance=1e-9):
    if reference.keys() != actual.keys():
        raise RuntimeError(f'{label}: physics inventory changed; '
                           f'missing={sorted(reference.keys()-actual.keys())[:8]}, '
                           f'extra={sorted(actual.keys()-reference.keys())[:8]}')
    maximum = 0.
    for key, expected in reference.items():
        found = actual[key]
        delta = float(np.max(np.abs(np.asarray(expected['world'])-np.asarray(found['world']))))
        maximum = max(maximum, delta)
        if delta > tolerance:
            raise RuntimeError(f'{label}: world transform differs at {key}: {delta}')
        for field in ('kinds', 'type', 'applied_schemas', 'attrs', 'rels', 'geometry_sha256'):
            if expected[field] != found[field]:
                raise RuntimeError(f'{label}: {field} differs at {key}')
    physics_only = {key: {k:v for k,v in value.items() if k != 'world'}
                    for key,value in reference.items()}
    digest = hashlib.sha256(json.dumps(physics_only, sort_keys=True).encode()).hexdigest()
    return dict(passed=True, objects=len(actual),
                colliders=sum('collider' in row['kinds'] for row in actual.values()),
                bodies=sum('body' in row['kinds'] for row in actual.values()),
                joints=sum('joint' in row['kinds'] for row in actual.values()),
                geometry_and_physics_sha256=digest,
                maximum_world_matrix_error=maximum, numerical_tolerance=tolerance)


def _fix_targets(stage, root, mapping):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if prim.IsInstanceProxy():
            # Masked flatten expands source instances, so this would be an
            # unexpected composition change rather than a writable copy.
            continue
        for rel in prim.GetRelationships():
            targets = rel.GetTargets()
            changed = [Sdf.Path(remap_path(path, mapping)) for path in targets]
            if changed != targets:
                rel.SetTargets(changed)
        for attr in prim.GetAttributes():
            if not attr.HasAuthoredConnections():
                continue
            connections = attr.GetConnections()
            changed = [Sdf.Path(remap_path(path, mapping)) for path in connections]
            if changed != connections:
                attr.SetConnections(changed)


def _assert_joint_scope(stage, env_root):
    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(env_root)):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()):
            for target in rel.GetTargets():
                if not str(target).startswith(env_root + '/'):
                    raise RuntimeError(f'Cross-environment joint body: {prim.GetPath()} -> {target}')
        count += 1
    return count


def _assert_relationship_scope(stage, env_root):
    """Reject dangling source-root and cross-environment targets of any API."""
    relation_count = pair_relations = pair_targets = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(env_root)):
        for rel in prim.GetRelationships():
            targets = rel.GetTargets()
            relation_count += 1
            if rel.GetName() == 'physics:filteredPairs':
                pair_relations += 1
                pair_targets += len(targets)
            for target in targets:
                value = str(target)
                if any(value == root or value.startswith(root + '/') or value.startswith(root + '.')
                       for root in SOURCE_ROOTS):
                    raise RuntimeError(f'Relationship still refers to original dynamic root: '
                                       f'{rel.GetPath()} -> {target}')
                if value.startswith(ENVS_ROOT + '/') and not (
                        value == env_root or value.startswith(env_root + '/') or value.startswith(env_root + '.')):
                    raise RuntimeError(f'Cross-environment relationship: {rel.GetPath()} -> {target}')
    return dict(relationships_checked=relation_count, filtered_pair_relations=pair_relations,
                filtered_pair_targets=pair_targets)


def _migrate_dynamic_groups(stage, paths):
    """Merge full-dynamic fixture policies into disjoint environment groups.

    PhysX supports only one collision group per collider. Keeping an existing
    group over all active roots in addition to environment groups is therefore
    invalid, even if the two negative-filter policies appear compatible.
    """
    env_groups = [Sdf.Path(f'/World/MotionIsolation/group_{i}') for i in range(len(paths))]
    groups = [UsdPhysics.CollisionGroup(prim) for prim in stage.Traverse()
              if prim.IsA(UsdPhysics.CollisionGroup)]
    migrated = []
    aliases = {}
    for group in groups:
        includes = group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets()
        dynamic = [target for target in includes if any(
            str(target) == root or str(target).startswith(root + '/') for root in SOURCE_ROOTS)]
        if not dynamic:
            continue
        if set(str(target) for target in includes) != set(SOURCE_ROOTS):
            raise ValueError(f'Existing partial/mixed dynamic collision group requires '
                             f'an explicit policy adapter: {group.GetPath()}')
        old_path = group.GetPath()
        aliases[str(old_path)] = env_groups
        original_filters = group.GetFilteredGroupsRel().GetTargets()
        for i in range(len(paths)):
            dest = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(env_groups[i]))
            rel = dest.GetFilteredGroupsRel()
            for target in original_filters:
                rel.AddTarget(target)
        group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([])
        migrated.append(dict(source_group=str(old_path), destination_groups=[str(x) for x in env_groups],
                             original_includes=[str(x) for x in includes],
                             original_filtered_groups=[str(x) for x in original_filters]))
    if aliases:
        for group in groups:
            rel = group.GetFilteredGroupsRel()
            targets = rel.GetTargets()
            expanded = []
            for target in targets:
                for replacement in aliases.get(str(target), [target]):
                    if replacement not in expanded:
                        expanded.append(replacement)
            if expanded != targets:
                rel.SetTargets(expanded)
    return migrated


def _compression_categories(stage):
    """Recognize only the verified disjoint leaf-policy adapter's categories."""
    names = ('Leaves', 'ActivePlantOther', 'RobotTool')
    paths = ['/World/FixtureGroups/' + name for name in names]
    found = [bool(stage.GetPrimAtPath(path)) for path in paths]
    if not any(found):
        return None
    if not all(found):
        raise ValueError('Incomplete compressed collision-group categories')
    from motion_collision_groups import _enabled_colliders
    plant = _enabled_colliders(stage, ['/World/ElasticTarget'])
    leaves = [path for path in plant if '/Appendage_' in path]
    expected = [leaves, [path for path in plant if path not in set(leaves)], list(SOURCE_ROOTS[:2])]
    records = []
    for name, path, members in zip(names, paths, expected):
        group = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(path))
        includes = group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets()
        if set(map(str, includes)) != set(members):
            raise ValueError(f'Compressed group membership changed: {path}')
        if group.GetMergeGroupNameAttr().Get():
            raise ValueError('Merged collision groups require a separate replication adapter')
        records.append(dict(name=name, path=path, includes=includes,
                            filters=group.GetFilteredGroupsRel().GetTargets()))
    return records


def _migrate_category_groups(stage, paths, maps, records):
    """Clone each category once per environment, with no blanket group overlap."""
    aliases = {record['path']: [Sdf.Path('/World/MotionIsolation/env_' + str(i) + '/' + record['name'])
                               for i in range(len(paths))] for record in records}
    all_paths = [path for values in aliases.values() for path in values]
    original_groups = [UsdPhysics.CollisionGroup(prim) for prim in stage.Traverse()
                       if prim.IsA(UsdPhysics.CollisionGroup)]
    migrated = []
    for record in records:
        for i, mapping in enumerate(maps):
            dest = UsdPhysics.CollisionGroup.Define(stage, aliases[record['path']][i])
            includes = [Sdf.Path(remap_path(path, mapping)) for path in record['includes']]
            filters = [aliases[str(path)][i] if str(path) in aliases else path
                       for path in record['filters']]
            filters += [path for path in all_paths if path.GetParentPath() != dest.GetPath().GetParentPath()]
            filters = list(dict.fromkeys(filters))
            dest.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(includes)
            dest.CreateFilteredGroupsRel().SetTargets(filters)
        UsdPhysics.CollisionGroup(stage.GetPrimAtPath(record['path'])).GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([])
        migrated.append(dict(source_group=record['path'],
                             destination_groups=[str(path) for path in aliases[record['path']]],
                             original_includes=list(map(str, record['includes'])),
                             original_filtered_groups=list(map(str, record['filters']))))
    # A shared background group excluded all original active categories. Keep
    # that same global obstacle policy for every cloned category, and preserve
    # any other external group reference rather than dropping it.
    for group in original_groups:
        rel = group.GetFilteredGroupsRel()
        targets = rel.GetTargets()
        expanded = []
        for path in targets:
            expanded.extend(aliases.get(str(path), [path]))
        expanded = list(dict.fromkeys(expanded))
        if expanded != targets:
            rel.SetTargets(expanded)
    # Confirm the authored category filters and membership, including all
    # cross-environment group exclusions. Native collision controls remain
    # mandatory before accepting the multi-environment physics result.
    for record in records:
        for i, mapping in enumerate(maps):
            group = UsdPhysics.CollisionGroup(stage.GetPrimAtPath(aliases[record['path']][i]))
            expected_members = {remap_path(path, mapping) for path in record['includes']}
            if set(map(str, group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets())) != expected_members:
                raise RuntimeError('Per-environment category membership differs')
            expected_filters = {str(aliases[str(path)][i]) if str(path) in aliases else str(path)
                                for path in record['filters']}
            expected_filters.update(str(path) for path in all_paths
                                    if path.GetParentPath() != group.GetPath().GetParentPath())
            if set(map(str, group.GetFilteredGroupsRel().GetTargets())) != expected_filters:
                raise RuntimeError('Per-environment category exclusion graph differs')
    return migrated


def replicate_motion(stage, manifest, num_envs, *, device='cuda:0', native_replication=True,
                     source_copy='flatten'):
    """Return concrete per-environment paths and verified clone metadata.

    Global obstacle contact and native independence need a two-environment
    physics control before scaling. Metadata explicitly leaves that validation
    pending; this function verifies authored geometry and mapping only.
    """
    if not isinstance(num_envs, int) or not 1 <= num_envs <= 32768:
        raise ValueError('num_envs must be an integer in 1..32768')
    if source_copy not in {'flatten', 'reference'}:
        raise ValueError("source_copy must be 'flatten' or 'reference'")
    if stage.GetPrimAtPath(ENVS_ROOT):
        raise ValueError('Motion environment namespace already exists')
    if any(not stage.GetPrimAtPath(root) for root in SOURCE_ROOTS):
        raise ValueError('Source motion roots are missing')
    scene = stage.GetPrimAtPath('/World/PhysicsScene')
    inverted = scene.GetAttribute('physxScene:invertCollisionGroupFilter')
    if inverted and inverted.Get() is True:
        raise ValueError('Existing inverted collision groups require an explicit policy adapter')
    categories = _compression_categories(stage)
    category_reference_pairs = None
    if categories:
        from motion_collision_groups import _enabled_colliders, _group_pairs
        category_reference_pairs = _group_pairs(stage, _enabled_colliders(stage, SOURCE_ROOTS))
    reference = _snapshot(stage, SOURCE_ROOTS)
    source_relationships = []
    source_connections = []
    if source_copy == 'flatten':
        mask = Usd.StagePopulationMask(SOURCE_ROOTS)
        selected = Usd.Stage.OpenMasked(stage.GetRootLayer(), stage.GetSessionLayer(), mask)
        composed = selected.Flatten()
    else:
        # USD drops reference targets outside an individual referenced root.
        # Preserve these explicitly, including cross-root robot/tool joints
        # and shared physics/render material bindings.
        for root in SOURCE_ROOTS:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
                for rel in prim.GetRelationships():
                    source_relationships.append((str(rel.GetPath()), rel.GetTargets(), rel.IsCustom()))
                for attr in prim.GetAttributes():
                    if attr.HasAuthoredConnections():
                        source_connections.append((str(attr.GetPath()), attr.GetConnections()))
    paths = [f'{ENVS_ROOT}/env_{i}' for i in range(num_envs)]
    maps = [{root: env + '/' + root.rsplit('/', 1)[1] for root in SOURCE_ROOTS} for env in paths]
    session = stage.GetSessionLayer()
    old_target = stage.GetEditTarget()
    try:
        stage.SetEditTarget(session)
        migrated_groups = []
        UsdGeom.Xform.Define(stage, ENVS_ROOT)
        UsdGeom.Xform.Define(stage, paths[0])
        for root in SOURCE_ROOTS:
            dest = maps[0][root]
            if source_copy == 'flatten':
                if not Sdf.CopySpec(composed, root, session, dest):
                    raise RuntimeError('CopySpec failed for ' + root)
            else:
                # Referencing the source root layer preserves the original
                # reference/cooking composition. The session layer's runtime
                # overrides must be copied explicitly: external references do
                # not inherit the source stage's session edits. Original-root
                # deactivation is authored only after these copies.
                if session.GetPrimAtPath(root):
                    if not Sdf.CopySpec(session, root, session, dest):
                        raise RuntimeError('Session override CopySpec failed for ' + root)
                stage.OverridePrim(dest).GetReferences().AddReference(
                    stage.GetRootLayer().identifier, Sdf.Path(root))
        if source_copy == 'reference':
            for path, targets, custom in source_relationships:
                destination = Sdf.Path(remap_path(path, maps[0]))
                stage.GetPrimAtPath(destination.GetPrimPath()).CreateRelationship(
                    destination.name, custom=custom).SetTargets(
                        [Sdf.Path(remap_path(target, maps[0])) for target in targets])
            for path, connections in source_connections:
                stage.GetAttributeAtPath(remap_path(path, maps[0])).SetConnections(
                    [Sdf.Path(remap_path(target, maps[0])) for target in connections])
        _fix_targets(stage, paths[0], maps[0])
        env0_check = _compare(reference, _snapshot(stage, tuple(maps[0].values())), 'source -> env_0')
        for root in SOURCE_ROOTS:
            stage.OverridePrim(root).SetActive(False)
        if num_envs > 1:
            use_native = native_replication and device.startswith('cuda')
            # Cloner.clone hard-codes USD edits to GetRootLayer(). Author its
            # equivalent inheritance copies in the anonymous session layer,
            # then reuse Cloner's public native physics replication method.
            # All origins are identity, preserving world-fixed anchors exactly.
            for env in paths[1:]:
                UsdGeom.Xform.Define(stage, env).GetPrim().GetInherits().AddInherit(paths[0])
            if use_native:
                from isaacsim.core.cloner import Cloner
                cloner = Cloner(stage=stage)
                cloner.replicate_physics(source_prim_path=paths[0], prim_paths=paths,
                    base_env_path=ENVS_ROOT, root_path=ENVS_ROOT + '/env_',
                    enable_env_ids=True, clone_in_fabric=False)
            if use_native:
                scene.CreateAttribute('physxScene:envIdInBoundsBitCount', Sdf.ValueTypeNames.Int).Set(
                    max(4, math.ceil(math.log2(num_envs + 1))))
            if not categories:
                # Default negative filters preserve global obstacle contacts
                # and do not reinterpret existing diagnostic groups.
                for i, env in enumerate(paths):
                    group = UsdPhysics.CollisionGroup.Define(stage, f'/World/MotionIsolation/group_{i}')
                    group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([env])
                    group.CreateFilteredGroupsRel().SetTargets([
                        f'/World/MotionIsolation/group_{j}' for j in range(num_envs) if j != i])
                migrated_groups = _migrate_dynamic_groups(stage, paths)
        if categories:
            migrated_groups = _migrate_category_groups(stage, paths, maps, categories)
        # Existing fixture group includes must follow the new roots, if present.
        # Their exclusions are preserved and still explicitly fixture-only.
        for prim in list(stage.Traverse()):
            if not prim.IsA(UsdPhysics.CollisionGroup):
                continue
            for rel in prim.GetRelationships():
                targets = rel.GetTargets()
                if not any(str(path) in SOURCE_ROOTS for path in targets):
                    continue
                new_targets = []
                for path in targets:
                    if str(path) in SOURCE_ROOTS:
                        new_targets.extend(Sdf.Path(mapping[str(path)]) for mapping in maps)
                    else:
                        new_targets.append(path)
                rel.SetTargets(new_targets)
        # Native Cloner's USD inherits remap targets, but verify and repair
        # explicitly so no standalone breakable joint points to env_0.
        clone_checks = []
        relation_checks = []
        category_checks = []
        for i, env in enumerate(paths):
            if i:
                _fix_targets(stage, env, {maps[0][root]: maps[i][root] for root in SOURCE_ROOTS})
            _assert_joint_scope(stage, env)
            relation_checks.append(dict(index=i, **_assert_relationship_scope(stage, env)))
            # Checking first and last clone bounds startup work while every
            # clone still receives the joint-scope check.
            if i in {0, 1, num_envs - 1}:
                clone_checks.append(dict(index=i, **_compare(
                    reference, _snapshot(stage, tuple(maps[i].values())), f'source -> env_{i}')))
                if categories:
                    actual = _group_pairs(stage, _enabled_colliders(stage, tuple(maps[i].values())))
                    reverse = {new: old for old, new in maps[i].items()}
                    canonical = {tuple(sorted(remap_path(path, reverse) for path in pair)) for pair in actual}
                    if canonical != category_reference_pairs:
                        raise RuntimeError(f'env_{i}: compressed collision-group pair graph changed')
                    digest = hashlib.sha256(json.dumps(sorted(canonical), separators=(',', ':')).encode()).hexdigest()
                    category_checks.append(dict(index=i, passed=True, excluded_pairs=len(canonical),
                                                original_namespace_pair_sha256=digest))
        entries = []
        for i, env in enumerate(paths):
            mapping = maps[i]
            entries.append(dict(index=i, root=env, mapping=mapping,
                robot=mapping['/World/HarvestRobot'] + '/Robot/root_joint',
                plant=mapping['/World/ElasticTarget'],
                tool=mapping['/World/ActuatedTool'] + '/fixed',
                target=remap_path(manifest['target']['path'], mapping),
                target_joint=remap_path(manifest['target']['joint'], mapping),
                belts=[remap_path(path, mapping) for path in manifest['tool']['belts']],
                plant_bodies=[remap_path(item['path'], mapping) for item in manifest['plant']['bodies']],
                origin=[0., 0., 0.]))
        return dict(environments=entries, metadata=dict(
            num_envs=num_envs, source_roots=list(SOURCE_ROOTS),
            source_copy=source_copy,
            native_replication=bool(num_envs > 1 and native_replication and device.startswith('cuda')),
            source_to_env0=env0_check, clone_checks=clone_checks,
            relationship_scope_checks=relation_checks,
            shared_static_roots=['/World/Greenhouse', '/World/Ground'],
            static_obstacles_duplicated=False, scene_variation='identical scene only',
            source_assets_saved=False, edit_layer=session.identifier,
            collision_isolation='native environment IDs (GPU) and negative USD collision groups',
            migrated_collision_groups=migrated_groups,
            category_group_checks=category_checks,
            physics_validation=dict(shared_obstacle_contacts='pending native control',
                                    cross_environment_contacts='pending native control',
                                    independent_reset='not implemented')))
    finally:
        stage.SetEditTarget(old_target)
