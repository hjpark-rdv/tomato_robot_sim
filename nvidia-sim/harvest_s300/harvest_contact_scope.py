"""Native-break-aware harvest stop scope; all collision responses remain active.

The optional scope permits continuing manipulation after a natively released
fruit contacts a verified floor/deck or exact source-proven passive S300 chassis
CAD collider. It does not certify falling-fruit physics or collection. Released
fruit versus active arm/lift/tool, attached plant, gutter or unknown structure
remains strict. A separate opt-in authorizes only released-fruit impacts on
the exact gripper lower decks and carriage saddles. The default remains strict.
An independent same-truss option permits a verified released fruit to contact
its own truss fruit/rachis/pedicel/peduncle, including verified route history.
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


def gripper_support_inventory(stage):
    """Read-only identity check of the four user-authorized landing parts.

    These are moving jaw parts, not passive collection surfaces. No permission
    extends to a belt, upper deck, arm or attached fruit. Geometry hashes record
    the actual CAD being used; they are not a fruit-damage certification.
    """
    from pxr import UsdGeom, UsdPhysics
    from passive_collection_surfaces import _geometry_identity
    if stage is None:raise ValueError('Gripper support policy requires the current USD stage')
    result={}
    for side,number,source in [('L',117,'C01_carriage_saddle'),('L',122,'C02_lower_deck'),
                               ('R',239,'C01_carriage_saddle'),('R',244,'C02_lower_deck')]:
        body='/World/ActuatedTool/'+side;path=body+'/part_'+str(number)
        prim=stage.GetPrimAtPath(path)
        if (not prim or not prim.IsA(UsdGeom.Mesh) or
                prim.GetCustomDataByKey('sourceName')!=side+'_'+source or
                not prim.HasAPI(UsdPhysics.CollisionAPI) or
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True):
            raise ValueError('Exact gripper support collider identity mismatch: '+path)
        owner=prim
        while owner and not owner.IsPseudoRoot() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner=owner.GetParent()
        if not owner or str(owner.GetPath())!=body:
            raise ValueError('Gripper support rigid owner mismatch: '+path)
        digest,counts=_geometry_identity(UsdGeom.Mesh(prim))
        result[path]=dict(body=body,source_name=side+'_'+source,authored_geometry_sha256=digest,
            source_vertices=counts[0],source_triangles=counts[1],scope='released_fruit_gripper_support',
            authorization='User-authorized post-detachment lower-deck/saddle impact only',
            physical_collision_response_changed=False,collection_verified=False)
    return result


def same_truss_inventory(stage, manifest, *, frozen_root=None, disabled_colliders=frozenset()):
    """Exact enabled own-truss fruit/branch colliders; no stem or other plant.

    The namespace is part of identity: Truss_02 in two plants is not the same
    truss. Frozen namespaces are accepted only by the verified history loader.
    """
    from pxr import UsdGeom, UsdPhysics
    if stage is None:raise ValueError('Same-truss contact policy requires the current USD stage')
    source_root=manifest['plant']['root'].rstrip('/')
    root=source_root if frozen_root is None else frozen_root
    pattern=re.compile(r'(Truss_\d+)_(Fruit_\d+|Pedicel_(?:proximal|distal)_\d+_\d+|Rachis_\d+|Truss_\d+_Peduncle_\d+)')
    result={}
    for row in manifest['plant'].get('bodies',[])+manifest['plant']['fruits']:
        name=row.get('name','');match=pattern.fullmatch(name)
        if not match:continue
        if row.get('path')!=source_root+'/'+name:raise ValueError('Same-truss source body identity mismatch')
        body=root+'/'+name;path=body+'/Collider';prim=stage.GetPrimAtPath(path)
        if path in disabled_colliders:
            if not prim or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False:
                raise ValueError('Declared non-target collider is not disabled: '+path)
            continue
        kind=UsdGeom.Mesh if match[2].startswith('Fruit_') else UsdGeom.Capsule
        if (not prim or not prim.IsA(kind) or not prim.HasAPI(UsdPhysics.CollisionAPI) or
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True):
            raise ValueError('Same-truss exact collider missing or disabled: '+path)
        owner=prim
        while owner and not owner.IsPseudoRoot() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        owner_path=str(owner.GetPath()) if owner and not owner.IsPseudoRoot() else None
        if owner_path!=body and not (frozen_root is not None and owner_path is None):
            raise ValueError('Same-truss collider rigid owner mismatch: '+path)
        result[path]=dict(body=body,source_body=row['path'],truss_identity=root+'/'+match[1],
            truss_key=match[1],part=match[2],frozen_history=frozen_root is not None)
    return result


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
    def __init__(self, manifest, ground_colliders=(), *, enabled=False, stage=None, gripper_support_enabled=False,
                 same_truss_enabled=False):
        self.enabled = bool(enabled)
        self.gripper_support_enabled=bool(gripper_support_enabled)
        self.gripper_supports=gripper_support_inventory(stage) if self.gripper_support_enabled else {}
        self.same_truss_enabled=bool(same_truss_enabled)
        self.disabled_plant_colliders=frozenset()
        if manifest.get('plant_interaction_variant',{}).get('mode')=='target_truss_and_own_stem_only':
            from plant_interaction_variants import verified_disabled_colliders
            self.disabled_plant_colliders=verified_disabled_colliders(stage,manifest)
        self.same_truss_contacts=same_truss_inventory(stage,manifest,
            disabled_colliders=self.disabled_plant_colliders) if self.same_truss_enabled else {}
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
        if (self.enabled or self.gripper_support_enabled or self.same_truss_enabled) and history:
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
            if self.same_truss_enabled:
                self.same_truss_contacts.update(same_truss_inventory(stage,metadata['manifest'],
                    frozen_root=row['frozen_root'],disabled_colliders=self.disabled_plant_colliders))
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
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not (False if collider in self.disabled_plant_colliders else True) or
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
        candidates=[]
        for one, other in ((left, right), (right, left)):
            same=(one!=other and one in self.same_truss_contacts and other in self.same_truss_contacts and
                  self.same_truss_contacts[one]['truss_identity']==self.same_truss_contacts[other]['truss_identity'])
            if one in self.fruit_joints and (other in self.ground or other in self.gripper_supports or same):
                release_t=known_breaks.get(self.fruit_joints[one])
                candidates.append((bool(valid_time and release_t is not None and release_t<=t),one,other,same))
        # A fruit-fruit callback can put the still-attached fruit first.
        selected=next((row for row in candidates if row[0]),candidates[0] if candidates else (False,None,None,False))
        _,fruit,ground,same=selected
        joint = self.fruit_joints.get(fruit)
        break_t = known_breaks.get(joint)
        released = bool(valid_time and break_t is not None and break_t <= t)
        eligible = bool(fruit is not None and released and valid_distance)
        support=ground in self.gripper_supports
        authorized=eligible and (self.same_truss_enabled if same else self.gripper_support_enabled if support else self.enabled)
        return dict(t=t, phase=event.get('phase'), a=left, b=right,
                    scope=('released_fruit_same_truss' if same else 'released_fruit_gripper_support' if support else 'released_fruit_ground' if ground and _is_ground(ground)
                           else self.collection_surface_proofs.get(ground, {}).get('scope', 'released_fruit_passive_collection_deck')) if eligible else 'active_or_unverified',
                    native_release_verified=released, fruit_collider=fruit,
                    native_joint=joint, native_break_t=break_t, ground_collider=ground,
                    prior_route_release_proof=self.prior_releases.get(joint),
                    collection_collider=ground,
                    collection_surface_proof=self.collection_surface_proofs.get(ground),
                    gripper_support_proof=self.gripper_supports.get(ground),
                    same_truss_proof=dict(fruit=self.same_truss_contacts[fruit],other=self.same_truss_contacts[ground]) if same else None,
                    exact_declared_passive_leaf_colliders=[path for path in (left,right) if path in self.background_leaves],
                    passive_leaf_contact_exemption=False,
                    raw_separation_m=separation,
                    raw_penetration_m=max(0., -separation) if valid_distance else None,
                    force_n=event.get('force_n'), impulse_ns=event.get('impulse_ns'),
                    continuation_authorized=bool(authorized),
                    reason=('Exact native fruit-joint break preceded an explicitly authorized impact; '
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
                    gripper_support_enabled=self.gripper_support_enabled,
                    same_truss_enabled=self.same_truss_enabled,
                    explicitly_disabled_non_target_plant_colliders=len(self.disabled_plant_colliders),
                    exact_same_truss_inventory=copy_inventory(self.same_truss_contacts),
                    exact_gripper_support_proofs=copy_inventory(self.gripper_supports),
                    exact_ground_colliders=sorted(p for p in self.ground if _GROUND.fullmatch(p)),
                    exact_declared_passive_leaf_inventory=copy_inventory(self.background_leaves),
                    exact_collection_surfaces=sorted(self.ground),
                    exact_collection_surface_proofs=copy_inventory(self.collection_surface_proofs),
                    exact_fruit_joint_mapping=dict(sorted(self.fruit_joints.items())),
                    exact_verified_prior_route_releases=dict(sorted(self.prior_releases.items())),
                    native_releases=[dict(fruit_collider=fruit, joint=joint, t=times[joint])
                                     for fruit, joint in sorted(self.fruit_joints.items())
                                     if joint in times],
                    scope='Native-released fruit versus separately enabled floor/carrier, lower-deck/saddle or exact same-truss fruit/branch policies; all other pairs remain strict',
                    collision_response='unchanged; no collider/mask/material/threshold edits',
                    active_robot_plant_guard='No release evidence means unchanged guard; belt, arm, lift, other tool parts, main stem, gutter and other-truss contacts remain strict',
                    training_eligible=False, collection_verified=False,
                    released_fruit_physics_verified=False,
                    deferred_invalid_contact_points=len(self.records))

    def summarize(self, breaks=()):
        groups = {}
        for row in self.records:
            key = (row['fruit_collider'], row['ground_collider'])
            group = groups.setdefault(key, dict(fruit_collider=key[0], ground_collider=key[1],
                                               scope=row['scope'],
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
                    active_guard_exceedance_deferred=bool(self.records),
                    deferred_0_5mm_exceedance_present=bool(self.records),
                    released_fruit_ground_impacts=[row for row in groups.values() if _GROUND.fullmatch(row['ground_collider'])],
                    released_fruit_collection_impacts=[row for row in groups.values() if row['scope']not in ('released_fruit_gripper_support','released_fruit_same_truss')],
                    released_fruit_gripper_support_impacts=[row for row in groups.values() if row['scope']=='released_fruit_gripper_support'],
                    released_fruit_same_truss_impacts=[row for row in groups.values() if row['scope']=='released_fruit_same_truss'],
                    records=self.records)
