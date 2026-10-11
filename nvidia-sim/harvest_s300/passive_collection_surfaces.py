"""Exact source-proven passive S300 chassis impacts, never an arm exemption.

Only exact native-broken fruit impacts may become opt-in lifecycle diagnostics.
All physical colliders and raw contact evidence remain. Vertical covers are
not verified collection surfaces. Attached fruit and active robot/plant stay
strict. The legacy no-manifest API verifies only the original horizontal cover.
"""
import hashlib
import json
import re
from pathlib import Path

import numpy as np
from pxr import UsdGeom, UsdPhysics

PATH = '/World/HarvestRobot/Robot/S300Carrier/S300Chassis/Part_0496'
BODY = '/World/HarvestRobot/Robot/S300Carrier'
SOURCE_NAME = '하부 · 밀착형 틈새 밀폐 상판'


def inventory(stage, manifest=None):
    """Source-proven passive CAD impacts, with legacy exact-deck fallback.

    A manifest carrying the verified separate-carrier lineage enables the pinned
    source CAD inventory. No path pattern alone confers permission. Other callers
    retain the prior single horizontal cover verification.
    """
    if manifest and manifest.get('rail_chassis_body'):
        return passive_carrier_inventory(stage, manifest)
    return _legacy_cover_inventory(stage)


def _legacy_cover_inventory(stage):
    if stage is None:
        return {}
    prim = stage.GetPrimAtPath(PATH)
    if not prim:
        return {}
    if (not prim.IsA(UsdGeom.Mesh) or
            prim.GetCustomDataByKey('sourceName') != SOURCE_NAME or
            not prim.HasAPI(UsdPhysics.CollisionAPI) or
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True or
            not prim.HasAPI(UsdPhysics.MeshCollisionAPI) or
            UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() != 'convexDecomposition'):
        raise ValueError('Declared S300 collection cover lacks exact source collider identity')
    owner = prim
    while owner and not owner.IsPseudoRoot() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
        owner = owner.GetParent()
    if not owner or str(owner.GetPath()) != BODY:
        raise ValueError('S300 collection cover is not owned by the passive rail carrier')
    mesh = UsdGeom.Mesh(prim)
    points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    if (points.shape != (96, 3) or counts.shape != (32,) or indices.shape != (96,) or
            not np.isfinite(points).all() or not np.all(counts == 3) or
            indices.min() < 0 or indices.max() >= len(points)):
        raise ValueError('S300 collection cover source topology changed')
    bounds = np.stack([points.min(0), points.max(0)])
    # This is source-identity verification of the existing 2mm CAD plate, not
    # a collision/biological tolerance. Rendering and native geometry retain
    # all original triangles, gaps and exact physical cooking parameters.
    if not np.allclose(bounds[1]-bounds[0], [.608, 1.4160001, .002], rtol=0, atol=2e-6):
        raise ValueError('S300 collection cover source dimensions changed')
    xform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    axis = np.asarray(xform.TransformDir((0., 0., 1.)), dtype=float)
    if not np.isfinite(axis).all() or np.linalg.norm(axis) <= 0:
        raise ValueError('S300 collection cover has an invalid transform')
    axis /= np.linalg.norm(axis)
    if abs(axis[2]) < .999999:
        raise ValueError('S300 collection cover is no longer a horizontal landing surface')
    digest = hashlib.sha256()
    for data in (points, indices, counts):
        digest.update(data.tobytes())
    return {PATH: dict(body=BODY, source_name=SOURCE_NAME,
        authored_geometry_sha256=digest.hexdigest(), source_bounds_m=bounds.tolist(),
        source_vertices=96, source_triangles=32,
        authorization='Only native-broken fruit landing diagnostic; no active contact permission',
        physical_collision_response_changed=False, collection_verified=False,
        released_fruit_physics_verified=False)}


CHASSIS = BODY + '/S300Chassis'
JOINT = '/World/HarvestRobot/Robot/S300CarrierJoint'
BASE = '/World/HarvestRobot/Robot/world'
REFERENCE_FILE = Path(__file__).parent / 'validation/passive_s300_chassis_source_inventory.json'
SOURCE_BYTES = Path(__file__).read_bytes()
SOURCE_SHA256 = hashlib.sha256(SOURCE_BYTES).hexdigest()
REFERENCE_BYTES = REFERENCE_FILE.read_bytes()
REFERENCE_SHA256 = hashlib.sha256(REFERENCE_BYTES).hexdigest()


def _reference():
    return json.loads(REFERENCE_BYTES)


def _file_sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''): digest.update(block)
    return digest.hexdigest()


def _geometry_identity(mesh):
    arrays = (np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64),
              np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64),
              np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64))
    points, indices, counts = arrays
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) == 0 or
            not np.isfinite(points).all() or counts.ndim != 1 or indices.ndim != 1 or
            not np.all(counts == 3) or len(indices) != int(counts.sum()) or
            len(indices) == 0 or indices.min() < 0 or indices.max() >= len(points)):
        raise ValueError('Passive S300 CAD source topology or finite geometry mismatch')
    digest = hashlib.sha256()
    for data in arrays: digest.update(data.tobytes())
    return digest.hexdigest(), (len(points), len(counts), len(indices))


def passive_carrier_inventory(stage, manifest):
    """Verify every approved CAD collider against the pinned unchanged source.

    This is only an impact-lifecycle classification AFTER exact native release.
    Vertical side covers and wheels are not claimed to collect fruit safely.
    The entire native scene can still violate 0.5mm through released impacts.
    Attached fruit, other plants, arms, lift and gripper remain strictly guarded.
    The current source geometry, topology, enabled state and passive rigid owner
    must match. No collider, solver, force, threshold or physical pose is edited.
    """
    from pxr import Usd
    if stage is None:
        raise ValueError('Passive S300 CAD inventory requires verified current USD stage geometry')
    ref = _reference()
    lineage = manifest.get('rail_chassis_body', {})
    required = ('source_scene_sha256', 'original_base_source_scene_sha256')
    if (any(lineage.get(key) != ref[key] for key in required) or
            lineage.get('carrier_body') != BODY or lineage.get('carrier_joint') != JOINT or
            lineage.get('chassis_root') != CHASSIS or
            lineage.get('geometry', {}).get('source_geometry_sha256') != ref['source_geometry_sha256'] or
            lineage.get('geometry', {}).get('original_geometry_unchanged') is not True or
            lineage.get('exclude_from_articulation') is not True):
        raise ValueError('Passive S300 CAD immutable source lineage mismatch')
    source = Path(lineage.get('source_assembly', ''))
    if not source.is_absolute() or _file_sha(source / 'scene.usda') != ref['source_scene_sha256']:
        raise ValueError('Passive S300 CAD actual source scene SHA mismatch')
    for dependency in ref['source_dependencies']:
        relative = dependency['relative_path']
        if Path(relative).is_absolute() or '..' in Path(relative).parts or \
                _file_sha(source / relative) != dependency['sha256']:
            raise ValueError('Passive S300 CAD source dependency SHA mismatch')
    carrier = stage.GetPrimAtPath(BODY)
    chassis = stage.GetPrimAtPath(CHASSIS)
    joint = stage.GetPrimAtPath(JOINT)
    if (not carrier or not chassis or not joint or
            not carrier.HasAPI(UsdPhysics.RigidBodyAPI) or
            UsdPhysics.RigidBodyAPI(carrier).GetRigidBodyEnabledAttr().Get() is not True or
            UsdPhysics.RigidBodyAPI(carrier).GetKinematicEnabledAttr().Get() is not False or
            carrier.HasAPI(UsdPhysics.ArticulationRootAPI) or
            not carrier.HasAPI(UsdPhysics.MassAPI) or
            float(UsdPhysics.MassAPI(carrier).GetMassAttr().Get()) != 150. or
            not joint.IsA(UsdPhysics.FixedJoint)):
        raise ValueError('Passive S300 CAD separate carrier rigid identity mismatch')
    constraint = UsdPhysics.Joint(joint)
    if (list(map(str, constraint.GetBody0Rel().GetTargets())) != [BASE] or
            list(map(str, constraint.GetBody1Rel().GetTargets())) != [BODY] or
            constraint.GetJointEnabledAttr().Get() is not True or
            constraint.GetExcludeFromArticulationAttr().Get() is not True or
            constraint.GetCollisionEnabledAttr().Get() is not False or
            any(a.GetName().startswith('drive:') for a in joint.GetAttributes())):
        raise ValueError('Passive S300 CAD external fixed-constraint identity mismatch')
    approved = {row['path']: row for row in ref['colliders']}
    if len(approved) != len(ref['colliders']) or any(
            not re.fullmatch(re.escape(CHASSIS) + r'/Part_\d{4}', path) for path in approved):
        raise ValueError('Pinned passive S300 source inventory contains an invalid exact path')
    current = {str(p.GetPath()) for p in Usd.PrimRange(chassis, Usd.TraverseInstanceProxies())
               if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI) and
               UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is True}
    if current != set(approved):
        raise ValueError('Passive S300 CAD source collider inventory changed')
    cache = UsdGeom.XformCache()
    proofs = {}
    for path, row in sorted(approved.items()):
        prim = stage.GetPrimAtPath(path)
        owner = prim
        while owner and not owner.IsPseudoRoot() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner = owner.GetParent()
        if (not owner or str(owner.GetPath()) != BODY or
                prim.GetCustomDataByKey('sourceName') != row['source_name'] or
                not prim.HasAPI(UsdPhysics.MeshCollisionAPI) or
                UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() != 'convexDecomposition'):
            raise ValueError('Passive S300 CAD exact source name, collider or owner mismatch: ' + path)
        geometry, counts = _geometry_identity(UsdGeom.Mesh(prim))
        # Walk authored local transforms within the chassis ancestor. Inverting
        # world poses would introduce cancellation error under measured carrier
        # rotation, falsely changing an unchanged source identity hash.
        relative_matrix, resets_ancestor = cache.ComputeRelativeTransform(prim, chassis)
        relative = np.asarray(relative_matrix, dtype=np.float64)
        if (geometry != row['authored_geometry_sha256'] or
                counts != (row['source_vertices'], row['source_triangles'], row['source_indices']) or
                resets_ancestor or not np.isfinite(relative).all() or
                hashlib.sha256(relative.tobytes()).hexdigest() != row['chassis_relative_matrix_sha256']):
            raise ValueError('Passive S300 CAD source geometry/topology/relative-transform mismatch: ' + path)
        proofs[path] = dict(body=BODY, source_name=row['source_name'], source_path=row['source_path'],
            authored_geometry_sha256=geometry, source_vertices=counts[0], source_triangles=counts[1],
            source_scene_sha256=ref['source_scene_sha256'],
            source_geometry_sha256=ref['source_geometry_sha256'],
            source_dependencies=ref['source_dependencies'],
            chassis_relative_transform_sha256=row['chassis_relative_matrix_sha256'],
            scope='released_fruit_passive_chassis_impact',
            helper_import_source_sha256=SOURCE_SHA256,
            approved_inventory_import_sha256=REFERENCE_SHA256,
            authorization='Only exact native-broken fruit impacts; no active robot/plant permission',
            physical_collision_response_changed=False, collection_verified=False,
            released_fruit_physics_verified=False)
    return proofs


def save_imported_source(output):
    """Preserve actual imported bytes and exact approved input, never overwrite."""
    destination = Path(output) / 'runtime_code_sources'
    destination.mkdir(parents=True, exist_ok=True)
    paths = ((destination / 'passive_collection_surfaces.py', SOURCE_BYTES),
             (destination / 'validation/passive_s300_chassis_source_inventory.json', REFERENCE_BYTES))
    for path, contents in paths:
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            if path.read_bytes() != contents:
                raise ValueError('Existing passive source provenance differs from imported bytes')
        else:
            with path.open('xb') as stream: stream.write(contents)
    return dict(path=str(paths[0][0]), module_import_source_sha256=SOURCE_SHA256,
                source_bytes_captured_at_import=True, launch_snapshot_overwritten=False,
                approved_inventory_path=str(paths[1][0]),
                approved_inventory_import_sha256=REFERENCE_SHA256)
