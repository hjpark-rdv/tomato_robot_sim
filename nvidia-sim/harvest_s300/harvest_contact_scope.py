"""Native-break-aware harvest stop scope; all collision responses remain active.

The optional scope permits continuing manipulation after a natively released
fruit contacts a verified floor/deck or exact source-proven passive S300 chassis
CAD collider. It does not certify falling-fruit physics or collection. Released
fruit versus active arm/lift/tool, attached plant, gutter or unknown structure
remains strict. A separate opt-in authorizes only released-fruit impacts on
the exact gripper lower decks and carriage saddles. The default remains strict.
An independent same-truss option permits a verified released fruit to contact
its own truss fruit/rachis/pedicel/peduncle, including verified route history.
The separate same-plant option expands only that post-release diagnostic to
exact other trusses on the same source plant, never to the main stem or robot.
The separate static-farm-support option inventories only source-pinned enabled
unanimated structural Mesh colliders, and never authorizes attached fruit.
The separate released-pair option authorizes only an impact between two exact
fruit colliders whose independent native release evidence precedes the impact.
"""
import math
import re
import hashlib
from pathlib import Path


_GROUND = re.compile(r'/World/Greenhouse/House/Environment/GroundCover_\d+_\d+/Mesh')
_GUTTER_BASE = re.compile(r'/World/Greenhouse/House/Environment/Gutter_\d+_Base_\d+/Mesh')
# Authored fixed farm structure roles, never a plant/robot name or an arbitrary
# Environment descendant. The exact enabled colliders are pinned below.
_FARM_SUPPORT_ROLES = frozenset(('GrowBag','CoirSlab','PlantingPort','Gutter',
    'GutterSuspensionWire','GutterSupportLeg','GutterCradle','GutterEndCap',
    'GutterFoldedLip','GroundCover','Foundation','Rail','HeatingRail',
    'SupportFoot','AnchorBolt','BasePlate','Column','ConcreteControlJoint',
    'DrainageGroove','DrainOutlet','CropSupport','CropSupportCrossbar',
    'CropBarSuspension','LongitudinalTrellisWire','CoverLap','RowSupply'))
_FARM_SUPPORT = re.compile(r'/World/Greenhouse/House/Environment/([A-Za-z]+)(?:_[A-Za-z0-9]+)*/Mesh')
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


def _static_farm_mesh_inventory(stage, *, gutter_only=False, source_scene_sha256=None):
    """Exact fixed structural meshes, solely post-release impact diagnostics.

    The compatibility option restricts this inventory to gutter bases. Dynamic
    owners, animated geometry/transforms and unknown roles stay strict in both.
    Record actual geometry and world transform without editing either.
    """
    from pxr import Usd, UsdGeom, UsdPhysics
    import numpy as np
    from passive_collection_surfaces import _geometry_identity
    if stage is None:raise ValueError('Static farm-support policy requires the current USD stage')
    result={};cache=UsdGeom.XformCache()
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        path=str(prim.GetPath())
        role=_FARM_SUPPORT.fullmatch(path)
        if (not (_GUTTER_BASE.fullmatch(path) if gutter_only else role and role[1] in _FARM_SUPPORT_ROLES) or not prim.IsA(UsdGeom.Mesh) or
                not prim.HasAPI(UsdPhysics.CollisionAPI) or
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True):continue
        owner=prim;nonstatic=False
        while owner and not owner.IsPseudoRoot():
            if (owner.HasAPI(UsdPhysics.RigidBodyAPI) or
                    any(attr.GetNumTimeSamples() for attr in owner.GetAttributes())):
                nonstatic=True;break
            owner=owner.GetParent()
        if nonstatic:continue
        digest,counts=_geometry_identity(UsdGeom.Mesh(prim))
        matrix=np.asarray(cache.GetLocalToWorldTransform(prim),dtype=np.float64)
        if not np.isfinite(matrix).all():raise ValueError('Static farm support has a nonfinite world transform')
        result[path]=dict(body=None,authored_geometry_sha256=digest,
            source_vertices=counts[0],source_triangles=counts[1],
            world_transform_sha256=hashlib.sha256(matrix.tobytes()).hexdigest(),
            scope='released_fruit_static_gutter_base' if gutter_only else 'released_fruit_static_farm_support',
            source_scene_sha256=source_scene_sha256,structural_role=role[1] if role else 'Gutter',
            authorization='Exact native-broken fruit impact only; attached plant and robot remain strict',
            physical_collision_response_changed=False,collection_verified=False,
            released_fruit_physics_verified=False)
    return result


def static_gutter_base_inventory(stage):
    """Compatibility scope: exact static gutter BASE only, never sides/wires."""
    return _static_farm_mesh_inventory(stage,gutter_only=True)


def static_farm_support_inventory(stage, manifest):
    """Exact native-static, unanimated farm Mesh inventory from the pinned scene.

    This only classifies an already detached fruit's impact. Attached fruit,
    plant/arm/tool contacts, unknown roles and movable structures stay strict.
    The runner verifies this source scene hash before constructing the scope.
    """
    source_sha=manifest.get('scene_sha256')
    if not isinstance(source_sha,str) or not re.fullmatch('[0-9a-f]{64}',source_sha):
        raise ValueError('Static farm-support diagnostic requires the verified source scene SHA256')
    return _static_farm_mesh_inventory(stage,source_scene_sha256=source_sha)


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
            truss_key=match[1],plant_identity=root,part=match[2],frozen_history=frozen_root is not None)
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


def _read_verified_prior_history(self,history,stage):
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
        if self.same_truss_enabled or self.same_plant_enabled:
            self.same_truss_contacts.update(same_truss_inventory(stage,metadata['manifest'],
                frozen_root=row['frozen_root'],disabled_colliders=self.disabled_plant_colliders))
        source_root=metadata['manifest']['plant']['root'].rstrip('/')
        source_fruits={fruit.get('name'):fruit for fruit in metadata['manifest']['plant']['fruits']}
        expected_released={name for name,fruit in source_fruits.items()
            if any(e['joint']==fruit['joint'] for e in metadata['break_events'])}
        declared=row.get('detached_fruits',[])
        if (not isinstance(declared,list) or len(declared)!=len(expected_released) or
                {fruit.get('name') for fruit in declared}!=expected_released):
            raise ValueError('Frozen detached inventory must include every exact checkpoint native-released fruit')
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
            anchor=original.get('anchor')
            if not isinstance(anchor,str) or not anchor.startswith(source_root+'/'):
                raise ValueError('Source native fruit lacks exact attachment anchor identity')
            frozen_anchor=row['frozen_root']+anchor[len(source_root):]
            disabled_kind=None
            if disabled and disabled.IsA(UsdPhysics.Joint):
                relation=UsdPhysics.Joint(disabled)
                if (fruit.get('anchor')!=frozen_anchor or not stage.GetPrimAtPath(frozen_anchor) or
                        list(map(str,relation.GetBody0Rel().GetTargets()))!=[frozen_anchor] or
                        list(map(str,relation.GetBody1Rel().GetTargets()))!=[frozen]):
                    raise ValueError('Frozen disabled harvest joint body/anchor mapping mismatch')
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


def verified_route_release_inventory(stage, history, *, disabled_colliders=frozenset()):
    """Read-only exact frozen native-release identities from the existing verifier.

    Collision-policy derivation and contact-stop classification share this
    authority: completed checkpoint/hash, exact original callback and disabled
    joint, fruit namespace and rigid owner. A name alone never proves release.
    Disabled flags may only be acknowledged through the caller's exact inventory;
    this does not itself edit or exempt any collider.
    """
    from types import SimpleNamespace
    reader=SimpleNamespace(same_truss_enabled=False,same_plant_enabled=False,
        same_truss_contacts={},disabled_plant_colliders=frozenset(disabled_colliders),
        fruit_joints={},prior_releases={})
    _read_verified_prior_history(reader,history,stage)
    from pxr import Usd,UsdGeom,UsdPhysics
    if any(not stage.GetPrimAtPath(path).IsA(UsdGeom.Mesh) for path in reader.fruit_joints):
        raise ValueError('Verified released fruit requires its exact frozen Mesh collider')
    # A frozen fruit with a retained native rigid body must have release proof;
    # attached outgoing fruit had their RigidBodyAPI removed by freeze_outgoing.
    # Missing an entire history row must not silently turn free fruit into ghosts.
    root=stage.GetPrimAtPath('/World/RouteFrozen')
    if root:
        for prim in Usd.PrimRange(root):
            path=str(prim.GetPath())
            if (re.fullmatch(r'/World/RouteFrozen/Plant_\d+/Truss_\d+_Fruit_\d+',path) and
                    prim.HasAPI(UsdPhysics.RigidBodyAPI) and path+'/Collider' not in reader.fruit_joints):
                raise ValueError('Frozen dynamic fruit lacks authoritative native release history: '+path)
    return {path:dict(native_joint=joint,**reader.prior_releases[joint])
            for path,joint in sorted(reader.fruit_joints.items())}


class HarvestContactScope:
    """Separate active manipulation failures from released-fruit collection impacts."""
    def __init__(self, manifest, ground_colliders=(), *, enabled=False, stage=None, gripper_support_enabled=False,
                 same_truss_enabled=False, same_plant_enabled=False, static_gutter_base_enabled=False,
                 static_farm_support_enabled=False, released_pair_enabled=False):
        self.enabled = bool(enabled)
        self.gripper_support_enabled=bool(gripper_support_enabled)
        self.gripper_supports=gripper_support_inventory(stage) if self.gripper_support_enabled else {}
        self.same_truss_enabled=bool(same_truss_enabled)
        self.same_plant_enabled=bool(same_plant_enabled)
        self.released_pair_enabled=bool(released_pair_enabled)
        self.static_gutter_base_enabled=bool(static_gutter_base_enabled)
        self.static_gutter_bases=static_gutter_base_inventory(stage) if self.static_gutter_base_enabled else {}
        self.static_farm_support_enabled=bool(static_farm_support_enabled)
        self.static_farm_supports=static_farm_support_inventory(stage,manifest) if self.static_farm_support_enabled else {}
        self.disabled_plant_colliders=frozenset()
        if manifest.get('plant_interaction_variant',{}).get('mode') in (
                'target_truss_and_own_stem_only','active_plant_and_own_stem_only'):
            from plant_interaction_variants import verified_disabled_colliders
            self.disabled_plant_colliders=verified_disabled_colliders(stage,manifest)
        self.same_truss_contacts=same_truss_inventory(stage,manifest,
            disabled_colliders=self.disabled_plant_colliders) if (self.same_truss_enabled or self.same_plant_enabled) else {}
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
        if (self.enabled or self.gripper_support_enabled or self.same_truss_enabled or self.same_plant_enabled or self.static_gutter_base_enabled or self.static_farm_support_enabled or self.released_pair_enabled) and history:
            if stage is None:raise ValueError('Verified route history requires the exact current USD stage')
            self._add_prior_history(history,stage)
        self.released_pair_colliders={}
        if self.released_pair_enabled:
            from pxr import UsdGeom,UsdPhysics
            if stage is None:raise ValueError('Released fruit-pair policy requires the exact current USD stage')
            for collider,joint in self.fruit_joints.items():
                if collider in self.disabled_plant_colliders:continue
                prim=stage.GetPrimAtPath(collider);body=collider.rsplit('/',1)[0]
                if (not prim or not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI) or
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True):
                    raise ValueError('Released fruit-pair exact enabled Mesh collider missing: '+collider)
                owner=prim
                while owner and not owner.IsPseudoRoot() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
                if not owner or str(owner.GetPath())!=body:
                    raise ValueError('Released fruit-pair exact rigid owner mismatch: '+collider)
                self.released_pair_colliders[collider]=dict(body=body,native_joint=joint)
        self.background_leaves=background_leaf_colliders_from_manifest(stage,manifest)
        self.records = []

    def _add_prior_history(self,history,stage):
        _read_verified_prior_history(self,history,stage)

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
        # This independent opt-in never infers release from a fruit name,
        # displacement or the other fruit's event. Frozen history was verified
        # above; active callback times must each precede this contact.
        pair_valid=(self.released_pair_enabled and left!=right and
            left in self.released_pair_colliders and right in self.released_pair_colliders and
            not isinstance(t,bool) and valid_time and not isinstance(separation,bool) and valid_distance)
        force=event.get('force_n');impulse=event.get('impulse_ns')
        pair_valid=bool(pair_valid and not isinstance(force,bool) and isinstance(force,(int,float)) and
            math.isfinite(force) and force>=0 and
            (impulse is None or isinstance(impulse,(list,tuple)) and len(impulse)==3 and
             all(not isinstance(v,bool) and isinstance(v,(int,float)) and math.isfinite(v) for v in impulse)))
        if pair_valid:
            proofs=[]
            for collider in (left,right):
                joint=self.fruit_joints[collider];stamp=known_breaks.get(joint)
                if stamp is None or stamp>t:break
                proofs.append(dict(collider=collider,**self.released_pair_colliders[collider],native_break_t=stamp,
                    native_release_verified=True,prior_route_release_proof=self.prior_releases.get(joint)))
            if len(proofs)==2:
                return dict(t=t,phase=event.get('phase'),a=left,b=right,scope='released_fruit_pair',
                    native_release_verified=True,both_native_releases_verified=True,released_pair_proof=proofs,
                    fruit_collider=left,native_joint=self.fruit_joints[left],native_break_t=proofs[0]['native_break_t'],
                    ground_collider=right,collection_collider=None,prior_route_release_proof=self.prior_releases.get(self.fruit_joints[left]),
                    raw_separation_m=separation,raw_penetration_m=max(0.,-separation),force_n=force,impulse_ns=impulse,
                    continuation_authorized=True,reason='Both exact fruit colliders independently natively released before this impact; retained released-object diagnostic only',
                    collection_verified=False,released_fruit_physics_verified=False,physical_collision_response_changed=False)
        candidates=[]
        for one, other in ((left, right), (right, left)):
            same=(one!=other and one in self.same_truss_contacts and other in self.same_truss_contacts and
                  self.same_truss_contacts[one]['truss_identity']==self.same_truss_contacts[other]['truss_identity'])
            same_plant=(one!=other and one in self.same_truss_contacts and other in self.same_truss_contacts and
                        self.same_truss_contacts[one]['plant_identity']==self.same_truss_contacts[other]['plant_identity'])
            if one in self.fruit_joints and (other in self.ground or other in self.gripper_supports or other in self.static_gutter_bases or other in self.static_farm_supports or same or
                                             (self.same_plant_enabled and same_plant)):
                release_t=known_breaks.get(self.fruit_joints[one])
                candidates.append((bool(valid_time and release_t is not None and release_t<=t),one,other,same,same_plant))
        # A fruit-fruit callback can put the still-attached fruit first.
        selected=next((row for row in candidates if row[0]),candidates[0] if candidates else (False,None,None,False,False))
        _,fruit,ground,same,same_plant=selected
        joint = self.fruit_joints.get(fruit)
        break_t = known_breaks.get(joint)
        released = bool(valid_time and break_t is not None and break_t <= t)
        eligible = bool(fruit is not None and released and valid_distance)
        support=ground in self.gripper_supports
        gutter_base=ground in self.static_gutter_bases
        farm_support=ground in self.static_farm_supports
        authorized=eligible and ((self.same_truss_enabled or self.same_plant_enabled) if same else
                                self.same_plant_enabled if same_plant else
                                self.static_farm_support_enabled if farm_support else
                                self.static_gutter_base_enabled if gutter_base else
                                self.gripper_support_enabled if support else self.enabled)
        return dict(t=t, phase=event.get('phase'), a=left, b=right,
                    scope=('released_fruit_same_truss' if same else 'released_fruit_same_plant' if same_plant else 'released_fruit_static_farm_support' if farm_support else 'released_fruit_static_gutter_base' if gutter_base else 'released_fruit_gripper_support' if support else 'released_fruit_ground' if ground and _is_ground(ground)
                           else self.collection_surface_proofs.get(ground, {}).get('scope', 'released_fruit_passive_collection_deck')) if eligible else 'active_or_unverified',
                    native_release_verified=released, fruit_collider=fruit,
                    native_joint=joint, native_break_t=break_t, ground_collider=ground,
                    prior_route_release_proof=self.prior_releases.get(joint),
                    collection_collider=ground,
                    collection_surface_proof=self.collection_surface_proofs.get(ground),
                    gripper_support_proof=self.gripper_supports.get(ground),
                    static_gutter_base_proof=self.static_gutter_bases.get(ground),
                    static_farm_support_proof=self.static_farm_supports.get(ground),
                    same_truss_proof=dict(fruit=self.same_truss_contacts[fruit],other=self.same_truss_contacts[ground]) if same else None,
                    same_plant_proof=dict(fruit=self.same_truss_contacts[fruit],other=self.same_truss_contacts[ground]) if same_plant else None,
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
                    same_plant_enabled=self.same_plant_enabled,
                    released_pair_enabled=self.released_pair_enabled,
                    exact_released_pair_collider_inventory=copy_inventory(self.released_pair_colliders),
                    static_gutter_base_enabled=self.static_gutter_base_enabled,
                    exact_static_gutter_base_proofs=copy_inventory(self.static_gutter_bases),
                    static_farm_support_enabled=self.static_farm_support_enabled,
                    exact_static_farm_support_proofs=copy_inventory(self.static_farm_supports),
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
                    scope='Native-released fruit versus separately enabled floor/carrier, lower-deck/saddle, exact static gutter base/farm support or exact same-truss/same-source-plant fruit/branch policies; separate opt-in for two independently native-released exact fruit colliders; all other pairs remain strict',
                    collision_response='unchanged; no collider/mask/material/threshold edits',
                    active_robot_plant_guard='No release evidence means unchanged guard; belt, arm, lift, other tool parts, main stem and unverified/other-plant contacts remain strict; fixed gutter sides/wires require the separate source-pinned static-farm-support opt-in, never an attached fruit/robot exemption',
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
                    released_fruit_collection_impacts=[row for row in groups.values() if row['scope']not in ('released_fruit_gripper_support','released_fruit_same_truss','released_fruit_same_plant','released_fruit_static_gutter_base','released_fruit_static_farm_support','released_fruit_pair')],
                    released_fruit_pair_impacts=[row for row in groups.values() if row['scope']=='released_fruit_pair'],
                    released_fruit_static_farm_support_impacts=[row for row in groups.values() if row['scope']=='released_fruit_static_farm_support'],
                    released_fruit_static_gutter_base_impacts=[row for row in groups.values() if row['scope']=='released_fruit_static_gutter_base'],
                    released_fruit_gripper_support_impacts=[row for row in groups.values() if row['scope']=='released_fruit_gripper_support'],
                    released_fruit_same_truss_impacts=[row for row in groups.values() if row['scope']=='released_fruit_same_truss'],
                    released_fruit_same_plant_impacts=[row for row in groups.values() if row['scope']=='released_fruit_same_plant'],
                    records=self.records)
