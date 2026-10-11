"""Native-break-aware harvest stop scope; all collision responses remain active.

The optional scope permits continuing manipulation after a natively released
fruit contacts a verified floor/deck or exact source-proven passive S300 chassis
CAD collider. It does not certify falling-fruit physics or collection. Released
fruit versus active arm/lift/tool, attached plant, gutter or unknown structure
remains strict. The default remains strict.
"""
import math
import re
from pathlib import Path


_GROUND = re.compile(r'/World/Greenhouse/House/Environment/GroundCover_\d+_\d+/Mesh')
_FOUNDATION = '/World/Greenhouse/House/Environment/Foundation_6x10m/Mesh'
_SEALED_DECK = '/World/HarvestRobot/Robot/S300Carrier/S300Chassis/Part_0496'
_DECKS = frozenset(('/World/HarvestRobot/S300/FlatRearDeck',
                    '/World/HarvestRobot/Robot/world/S300Chassis/FlatRearDeck',
                    '/World/HarvestRobot/Robot/S300Carrier/S300Chassis/FlatRearDeck',
                    _SEALED_DECK))


def _is_ground(path):
    return bool(_GROUND.fullmatch(path) or path == _FOUNDATION)


def collection_colliders_from_stage(stage, manifest=None):
    """Verified floors and exact passive S300 rear-deck identities.

    The original deck must be static. The rail version may belong only to the
    existing world rail carrier or the explicitly derived separate S300Carrier
    rigid body, never an arm, wrist or jaw body.
    """
    from pxr import UsdPhysics
    paths=list(ground_colliders_from_stage(stage))
    for path in sorted(_DECKS-{_SEALED_DECK}):
        prim=stage.GetPrimAtPath(path)
        if (not prim or not prim.HasAPI(UsdPhysics.CollisionAPI) or
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True):continue
        owner=prim
        while owner and not owner.IsPseudoRoot() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        body=str(owner.GetPath()) if owner and not owner.IsPseudoRoot() else None
        if path=='/World/HarvestRobot/S300/FlatRearDeck' and body is None:
            paths.append(path)
        elif path=='/World/HarvestRobot/Robot/world/S300Chassis/FlatRearDeck' and body=='/World/HarvestRobot/Robot/world':
            paths.append(path)
        elif path=='/World/HarvestRobot/Robot/S300Carrier/S300Chassis/FlatRearDeck' and body=='/World/HarvestRobot/Robot/S300Carrier':
            paths.append(path)
    from passive_collection_surfaces import inventory
    paths.extend(inventory(stage, manifest))
    return tuple(sorted(paths))


def ground_colliders_from_stage(stage):
    """Inventory the existing ground-cover mesh colliders, including instances.

    Names identify the existing floor family, while native authored collision
    and static ownership are verified. Pins, gutters, rails and other meshes
    are deliberately absent from this allowlist.
    """
    from pxr import Usd, UsdPhysics
    paths = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if not _is_ground(path) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True:
            continue
        parent, dynamic = prim, False
        while parent and not parent.IsPseudoRoot():
            if parent.HasAPI(UsdPhysics.RigidBodyAPI):
                dynamic = True
                break
            parent = parent.GetParent()
        if not dynamic:
            paths.append(path)
    return tuple(sorted(paths))


def copy_inventory(inventory):
    return {path:dict(row) for path,row in sorted(inventory.items())}


def background_leaf_colliders_from_manifest(stage,manifest,body_paths=None):
    """Exact declared passive leaf inventory, not a collision exemption.

    Every Mesh must retain its source leaf identity/path mapping and belong to
    the exact declared standalone world-connected body. An undeclared stem,
    fruit, tool or missing native body can never enter this inventory.
    """
    spec=manifest.get('background_compliant_foliage')
    if not spec:return {}
    if stage is None:raise ValueError('Declared passive leaf inventory requires the exact USD stage')
    from pxr import UsdGeom,UsdPhysics
    from background_foliage import declared_bodies
    from compliant_foliage import canonical_leaf_group
    bodies=declared_bodies(stage,manifest);mapping=spec.get('path_mapping',{})
    rows=spec.get('leaves',[]);inventory={}
    if not rows or len(rows)!=len(bodies):raise ValueError('Passive leaf body/leaf declaration mismatch')
    owners=set()
    for row in rows:
        path=row.get('path');stem=row.get('source_stem');group=row.get('source_group')
        if (path not in bodies or path in owners or
                not isinstance(stem,str) or not re.fullmatch(r'/World/Greenhouse/Plants/Plant_\d+/Stem',stem) or
                canonical_leaf_group(group or '')!=group or
                not row.get('meshes') or len(row['meshes'])!=len(row.get('source_meshes',[]))):
            raise ValueError('Exact passive leaf/source group declaration mismatch')
        if body_paths is not None and path not in set(body_paths):raise ValueError('Missing measured passive leaf native body')
        owners.add(path)
        for current,source in zip(row['meshes'],row['source_meshes']):
            prim=stage.GetPrimAtPath(current)
            if (current in inventory or mapping.get(source)!=current or
                    not source.startswith(stem+'/') or not current.startswith(path+'/Appendage_') or
                    not prim or not prim.IsA(UsdGeom.Mesh) or
                    not prim.HasAPI(UsdPhysics.CollisionAPI) or
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True or
                    not prim.HasAPI(UsdPhysics.MeshCollisionAPI) or
                    UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()!='convexDecomposition' or
                    canonical_leaf_group(prim.GetCustomDataByKey('sourceName') or '')!=group):
                raise ValueError('Exact enabled passive leaf Mesh source/owner mapping mismatch')
            owner=prim
            while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
            if not owner or str(owner.GetPath())!=path:raise ValueError('Passive leaf has an unexpected native rigid owner')
            inventory[current]=dict(body=path,source_mesh=source,source_stem=stem,source_group=group,
                                    joint=row['joint'],native_physics_geometry='convexDecomposition')
    return inventory


class HarvestContactScope:
    """Separate active manipulation failures from released-fruit collection impacts."""
    def __init__(self, manifest, ground_colliders=(), *, enabled=False, stage=None):
        self.enabled = bool(enabled)
        self.ground = frozenset(ground_colliders)
        from passive_collection_surfaces import inventory
        self.collection_surface_proofs=inventory(stage, manifest)
        if any(not _is_ground(path) and path not in _DECKS and path not in self.collection_surface_proofs for path in self.ground):
            raise ValueError('Only exact verified floor, passive rear deck or source-proven passive S300 CAD collider paths are accepted')
        if _FOUNDATION in self.ground and (stage is None or _FOUNDATION not in ground_colliders_from_stage(stage)):
            raise ValueError('Foundation floor requires its enabled static collider in the current USD stage')
        if _SEALED_DECK in self.ground and _SEALED_DECK not in self.collection_surface_proofs:
            raise ValueError('Exact sealed S300 collection cover requires verified current USD stage geometry')
        root = manifest['plant']['root'].rstrip('/')
        self.fruit_joints = {}
        for fruit in manifest['plant']['fruits']:
            name, body, joint = fruit['name'], fruit['path'], fruit['joint']
            if (not re.fullmatch(r'Truss_\d+_Fruit_\d+', name) or
                    body != root + '/' + name or
                    joint != root + '/HarvestJoints/' + name):
                raise ValueError('Native fruit/body/harvest-joint identity mismatch: ' + name)
            collider = body + '/Collider'
            if collider in self.fruit_joints or joint in self.fruit_joints.values():
                raise ValueError('Duplicate native fruit/joint identity')
            self.fruit_joints[collider] = joint
        self.prior_releases={}
        history=manifest.get('route_checkpoint',{}).get('outgoing_history',[])
        if self.enabled and history:
            if stage is None:raise ValueError('Verified route history requires the exact current USD stage')
            self._add_prior_history(history,stage)
        self.background_leaves=background_leaf_colliders_from_manifest(stage,manifest)
        self.records = []

    def _add_prior_history(self,history,stage):
        """Import exact frozen fruit identity from a verified measured checkpoint.

        These are t=0 prior release proofs, not fabricated current native break
        callbacks. Source files, NPZ and metadata hashes, completed-cycle state,
        exact original event and remapped disabled joint are all checked.
        """
        from pxr import UsdPhysics
        from native_route_checkpoint import load_checkpoint
        from resume_native_state import require_completed_checkpoint
        checkpoints={}
        for row in history:
            checkpoint=row.get('checkpoint')
            if not isinstance(checkpoint,str) or not Path(checkpoint).is_absolute():
                raise ValueError('Route history lacks an absolute verified native checkpoint')
            if checkpoint not in checkpoints:
                metadata,_=load_checkpoint(checkpoint);require_completed_checkpoint(metadata)
                checkpoints[checkpoint]=metadata
            metadata=checkpoints[checkpoint]
            source=row.get('source_plant');plant_id=source.rsplit('/',1)[-1] if isinstance(source,str) else ''
            if (not re.fullmatch(r'Plant_\d+',plant_id) or source!=metadata['source_plant'] or
                    row.get('source_scene_sha256')!=metadata['scene_sha256'] or
                    row.get('checkpoint_metadata_sha256')!=metadata['metadata_sha256'] or
                    row.get('checkpoint_time_s')!=metadata['time_s'] or
                    row.get('frozen_root')!='/World/RouteFrozen/'+plant_id):
                raise ValueError('Route history/source checkpoint identity mismatch')
            source_root=metadata['manifest']['plant']['root'].rstrip('/')
            source_fruits={fruit.get('name'):fruit for fruit in metadata['manifest']['plant']['fruits']}
            for fruit in row.get('detached_fruits',[]):
                name=fruit.get('name');original=source_fruits.get(name)
                frozen='/World/RouteFrozen/'+plant_id+'/'+str(name)
                joint='/World/RouteFrozen/'+plant_id+'/HarvestJoints/'+str(name)
                if (not isinstance(name,str) or not re.fullmatch(r'Truss_\d+_Fruit_\d+',name) or
                        original is None or original.get('path')!=source_root+'/'+name or
                        original.get('joint')!=source_root+'/HarvestJoints/'+name or
                        fruit.get('path')!=frozen or fruit.get('joint')!=joint or
                        fruit.get('original_joint')!=original['joint'] or
                        fruit.get('source_plant')!=source or fruit.get('route_identity')!=plant_id+'/'+name or
                        fruit.get('source_scene_sha256')!=metadata['scene_sha256'] or
                        fruit.get('checkpoint_metadata_sha256')!=metadata['metadata_sha256'] or
                        fruit.get('checkpoint_time_s')!=metadata['time_s']):
                    raise ValueError('Exact prior frozen fruit mapping mismatch')
                events=[event for event in metadata['break_events'] if event['joint']==original['joint']]
                if not events or fruit.get('native_break_events')!=events:
                    raise ValueError('Frozen detached fruit lacks exact saved native release evidence')
                collider=frozen+'/Collider';prim=stage.GetPrimAtPath(collider)
                owner=stage.GetPrimAtPath(frozen);disabled=stage.GetPrimAtPath(joint)
                disabled_kind=None
                if disabled and disabled.IsA(UsdPhysics.Joint):
                    if not disabled.IsActive():disabled_kind='existing_inactive_joint_declaration'
                    else:
                        enabled_attr=UsdPhysics.Joint(disabled).GetJointEnabledAttr()
                        if (enabled_attr.HasAuthoredValueOpinion() and enabled_attr.GetNumTimeSamples()==0 and
                                enabled_attr.Get() is False):
                            disabled_kind='existing_active_joint_explicitly_disabled'
                if (not prim or not owner or not owner.HasAPI(UsdPhysics.RigidBodyAPI) or
                        not prim.HasAPI(UsdPhysics.CollisionAPI) or
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True or
                        disabled_kind is None):
                    raise ValueError('Frozen released fruit collider/body/disabled-joint stage mismatch')
                body=prim
                while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):body=body.GetParent()
                if not body or str(body.GetPath())!=frozen:
                    raise ValueError('Frozen fruit collider has an unexpected rigid owner')
                if collider in self.fruit_joints or joint in self.fruit_joints.values():
                    raise ValueError('Duplicate prior fruit/joint route identity')
                self.fruit_joints[collider]=joint
                self.prior_releases[joint]=dict(local_release_t=0.,original_joint=original['joint'],
                    original_break_events=events,original_checkpoint_time_s=metadata['time_s'],
                    checkpoint=checkpoint,checkpoint_metadata_sha256=metadata['metadata_sha256'],
                    source_scene_sha256=metadata['scene_sha256'],source_plant=source,
                    disabled_joint_representation=disabled_kind,
                    route_identity=fruit['route_identity'],native_new_event=False)

    def _break_times(self, breaks):
        times = {joint:proof['local_release_t'] for joint,proof in self.prior_releases.items()}
        known = set(self.fruit_joints.values())
        for event in breaks:
            joint = event.get('joint')
            t = event.get('t')
            if (joint in known and joint not in self.prior_releases and not isinstance(t,bool) and isinstance(t, (int, float)) and
                    math.isfinite(t) and t >= 0):
                times[joint] = min(times.get(joint, float('inf')), float(t))
        return times

    def classify(self, event, breaks):
        """Use exact native break evidence at/before the contact's own timestamp."""
        left, right = event['a'], event['b']
        t = event.get('t')
        separation = event.get('separation_m')
        valid_time = isinstance(t, (int, float)) and math.isfinite(t) and t >= 0
        valid_distance = isinstance(separation, (int, float)) and math.isfinite(separation)
        known_breaks = self._break_times(breaks)
        fruit, ground = None, None
        for one, other in ((left, right), (right, left)):
            if one in self.fruit_joints and other in self.ground:
                fruit, ground = one, other
                break
        joint = self.fruit_joints.get(fruit)
        break_t = known_breaks.get(joint)
        released = bool(valid_time and break_t is not None and break_t <= t)
        eligible = bool(fruit is not None and released and valid_distance)
        return dict(t=t, phase=event.get('phase'), a=left, b=right,
                    scope=('released_fruit_ground' if ground and _is_ground(ground)
                           else self.collection_surface_proofs.get(ground, {}).get('scope', 'released_fruit_passive_collection_deck')) if eligible else 'active_or_unverified',
                    native_release_verified=released, fruit_collider=fruit,
                    native_joint=joint, native_break_t=break_t, ground_collider=ground,
                    prior_route_release_proof=self.prior_releases.get(joint),
                    collection_collider=ground,
                    collection_surface_proof=self.collection_surface_proofs.get(ground),
                    exact_declared_passive_leaf_colliders=[path for path in (left,right) if path in self.background_leaves],
                    passive_leaf_contact_exemption=False,
                    raw_separation_m=separation,
                    raw_penetration_m=max(0., -separation) if valid_distance else None,
                    force_n=event.get('force_n'), impulse_ns=event.get('impulse_ns'),
                    continuation_authorized=bool(self.enabled and eligible),
                    reason=('Exact native fruit-joint break preceded verified floor/passive S300 impact; '
                            'impact retained as released-object diagnostic') if eligible
                           else 'Active, attached, unknown or non-floor contact remains strict',
                    collection_verified=False,
                    released_fruit_physics_verified=False)

    def filter_invalid(self, events, breaks):
        """Filter an already-invalid event list, retaining every deferred event."""
        remaining = []
        for event in events:
            classified = self.classify(event, breaks)
            if classified['continuation_authorized']:
                self.records.append(classified)
            else:
                remaining.append(event)
        return remaining

    def metadata(self, breaks=()):
        times = self._break_times(breaks)
        return dict(enabled=self.enabled, default_mode='strict',
                    exact_ground_colliders=sorted(p for p in self.ground if _GROUND.fullmatch(p)),
                    exact_declared_passive_leaf_inventory=copy_inventory(self.background_leaves),
                    exact_collection_surfaces=sorted(self.ground),
                    exact_collection_surface_proofs=copy_inventory(self.collection_surface_proofs),
                    exact_fruit_joint_mapping=dict(sorted(self.fruit_joints.items())),
                    exact_verified_prior_route_releases=dict(sorted(self.prior_releases.items())),
                    native_releases=[dict(fruit_collider=fruit, joint=joint, t=times[joint])
                                     for fruit, joint in sorted(self.fruit_joints.items())
                                     if joint in times],
                    scope='Only natively released fruit versus verified floor/deck or exact source-proven passive S300 carrier CAD; arm/lift/tool and attached-plant contacts remain strict',
                    collision_response='unchanged; no collider/mask/material/threshold edits',
                    active_robot_plant_guard='unchanged for active robot arm/lift/tool and attached plant; released-fruit passive-carrier impacts are separately reported',
                    training_eligible=False, collection_verified=False,
                    released_fruit_physics_verified=False,
                    deferred_invalid_contact_points=len(self.records))

    def summarize(self, breaks=()):
        groups = {}
        for row in self.records:
            key = (row['fruit_collider'], row['ground_collider'])
            group = groups.setdefault(key, dict(fruit_collider=key[0], ground_collider=key[1],
                                               first_t=row['t'], last_t=row['t'],
                                               contact_points=0, max_penetration_m=0.,
                                               max_point_force_n=0.))
            group['first_t'] = min(group['first_t'], row['t'])
            group['last_t'] = max(group['last_t'], row['t'])
            group['contact_points'] += 1
            group['max_penetration_m'] = max(group['max_penetration_m'], row['raw_penetration_m'])
            force = row.get('force_n')
            if isinstance(force, (int, float)) and math.isfinite(force):
                group['max_point_force_n'] = max(group['max_point_force_n'], force)
        # This scope sees only violations submitted by the existing guard;
        # absence of a deferred impact cannot certify complete-run validity.
        return dict(**self.metadata(breaks), strict_0_5mm_passed=False if self.records else None,
                    deferred_0_5mm_exceedance_present=bool(self.records),
                    released_fruit_ground_impacts=[row for row in groups.values() if _GROUND.fullmatch(row['ground_collider'])],
                    released_fruit_collection_impacts=list(groups.values()),
                    records=self.records)
