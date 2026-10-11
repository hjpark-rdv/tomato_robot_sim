"""Hash-bound native convex pieces for an exact, opt-in tool collider scope.

This imports observed collision geometry into the existing FCL checker. It
never edits USD, native physics, contact permissions, or planning clearance.
Every SDK-returned piece is retained; incomplete/foreign caches fail closed.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

SCHEMA='s300-native-cooked-tool-capture-v1'
FRAME='All native convex vertices/normals/edges in the exact rigid-owner local frame'
SOURCE='Existing synchronous request_convex_collision_representation RESULT_VALID; all returned hulls retained'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite_vectors(value,*,minimum=1,label='vectors'):
    if (not isinstance(value,list) or len(value)<minimum or
            any(not isinstance(row,list) or len(row)!=3 or
                any(isinstance(x,bool) or not isinstance(x,(int,float)) for x in row) for row in value)):
        raise ValueError('Malformed native '+label)
    result=np.asarray(value,float)
    if not np.isfinite(result).all():raise ValueError('Nonfinite native '+label)
    return result


def load_native_tool_hulls(stage,path,*,assembly=None):
    """Return body-local piece arrays plus validated observational provenance."""
    from pxr import UsdGeom,UsdPhysics
    from compact_motion_assets import verify_compact_dependencies
    path=Path(path).resolve();proof=json.loads(path.read_text())
    if assembly is None:
        real=stage.GetRootLayer().realPath
        if not real:raise ValueError('Native tool cache requires an immutable source assembly')
        assembly=Path(real).resolve().parent
    assembly=Path(assembly).resolve()
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    source=assembly/'scene.usda';dependency=manifest.get('compact_assets',{}).get('materialized_dependency',{})
    verify_compact_dependencies(assembly,manifest)
    root_path=Path(stage.GetRootLayer().realPath).resolve() if stage.GetRootLayer().realPath else None
    allowed_roots={source.resolve(),(assembly/dependency.get('relative_path','composed.usdc')).resolve()}
    if (root_path not in allowed_roots or any(layer.dirty for layer in stage.GetUsedLayers()) or
            list(stage.GetSessionLayer().rootPrims)):
        raise ValueError('Native tool import requires the exact immutable source stage, without live USD overrides')
    if (proof.get('schema')!=SCHEMA or proof.get('native_engine')!='physx' or
            proof.get('native_geometry_complete') is not True or
            proof.get('captured_before_first_trial_step') is not True or
            proof.get('physics_unchanged') is not True or proof.get('training_eligible') is not False or
            proof.get('harvest_success') is not None or proof.get('frame')!=FRAME or proof.get('source')!=SOURCE):
        raise ValueError('Unverified or incomplete native tool cooking provenance')
    if (Path(proof.get('source_assembly','')).resolve()!=assembly or
            proof.get('source_scene_sha256')!=digest(source) or
            proof['source_scene_sha256']!=manifest.get('scene_sha256') or
            proof.get('source_usdc_sha256')!=dependency.get('sha256') or
            proof.get('source_native_scene_sha256')!=dependency.get('sha256')):
        raise ValueError('Native tool cache source scene/USDC SHA mismatch')
    if not np.isclose(UsdGeom.GetStageMetersPerUnit(stage),1.,atol=0,rtol=0):
        raise ValueError('Native tool body-local vertices require the source metre unit convention')
    for key in ('effective_hz','timestamp_s'):
        value=proof.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value) or value<0:
            raise ValueError('Finite native capture time/HZ provenance required')
    if proof['effective_hz']<=0:raise ValueError('Positive native capture Hz required')
    provenance_path=path.parent/'leaf_reader_source_provenance.json'
    provenance=json.loads(provenance_path.read_text())
    reader_path=Path(provenance.get('path','')).resolve()
    if (path.parent not in reader_path.parents or not reader_path.is_file() or
            provenance.get('sha256')!=proof.get('geometry_reader_source_sha256') or
            digest(reader_path)!=provenance['sha256']):
        raise ValueError('Native geometry-reader imported source proof mismatch')
    # The exact imported reader is retained independently of the current code.
    # Its cooked() transforms all SDK hulls into the named rigid-owner frame;
    # source geometry and native pose inventories below verify that ownership.
    requested=proof.get('selected_paths')
    if (not isinstance(requested,list) or not requested or
            any(not isinstance(p,str) or not p.startswith('/World/ActuatedTool/') for p in requested) or
            len(requested)!=len(set(requested))):
        raise ValueError('Exact unique tool collider scope required')
    if set(proof.get('geometry',{}))!=set(requested) or set(proof.get('settings',{}))!=set(requested):
        raise ValueError('Native tool cache collider coverage mismatch')
    cache=UsdGeom.XformCache();pieces={};rows=[];owners=set()
    for collider in requested:
        prim=stage.GetPrimAtPath(collider)
        if (not prim or not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI) or
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True or
                not prim.HasAPI(UsdPhysics.MeshCollisionAPI)):
            raise ValueError('Missing exact enabled source tool Mesh: '+collider)
        owner=prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        if not owner or not str(owner.GetPath()).startswith('/World/ActuatedTool/'):
            raise ValueError('Native tool mesh lacks its exact tool rigid owner')
        body=str(owner.GetPath());owners.add(body)
        setting=proof['settings'][collider];data=proof['geometry'][collider]
        approximation=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        count=setting.get('hull_count')
        if (approximation not in ('convexHull','convexDecomposition') or setting.get('approximation')!=approximation or
                setting.get('owner')!=body or data.get('body')!=body or data.get('kind')!='native_cooked_convex' or
                isinstance(count,bool) or not isinstance(count,int) or count<1 or
                not isinstance(data.get('hulls'),list) or len(data['hulls'])!=count):
            raise ValueError('Native tool owner/representation/full hull coverage mismatch: '+collider)
        owner_world=np.asarray(cache.GetLocalToWorldTransform(owner)).T
        collider_world=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        if (not np.isfinite([owner_world,collider_world]).all() or
                not np.allclose(owner_world[3],[0,0,0,1],atol=1e-9,rtol=0) or
                np.linalg.det(owner_world[:3,:3])<=0 or
                not np.allclose(owner_world[:3,:3].T@owner_world[:3,:3],np.eye(3),atol=1e-6,rtol=0)):
            raise ValueError('Source tool rigid-owner coordinate frame is not rigid/finite')
        local=np.linalg.inv(owner_world)@collider_world
        mesh=UsdGeom.Mesh(prim);source_vertices=np.asarray(mesh.GetPointsAttr().Get(),float)
        source_counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get());source_indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
        if (source_vertices.ndim!=2 or source_vertices.shape[1]!=3 or not len(source_vertices) or
                not np.isfinite(source_vertices).all() or not np.all(source_counts==3) or
                len(source_indices)!=source_counts.sum() or np.any(source_indices<0) or
                np.any(source_indices>=len(source_vertices))):
            raise ValueError('Source collider topology/coordinates are invalid')
        source_vertices=source_vertices@local[:3,:3].T+local[:3,3]
        collected=[]
        for index,hull in enumerate(data['hulls']):
            vertices=finite_vectors(hull.get('vertices'),minimum=4,label='convex hull vertices')
            normals=finite_vectors(hull.get('normals'),label='face normals')
            edges=finite_vectors(hull.get('edges'),label='edge directions')
            if np.any(np.linalg.norm(normals,axis=1)<1e-10) or np.any(np.linalg.norm(edges,axis=1)<1e-10):
                raise ValueError('Degenerate native hull direction evidence')
            try:convex=ConvexHull(vertices)
            except Exception as error:raise ValueError('Degenerate native convex hull: '+str(error)) from error
            if not np.isfinite(convex.volume) or convex.volume<=0:
                raise ValueError('Native convex hull must have finite positive volume')
            collected.append(vertices.copy())
        pieces[collider]=collected
        all_vertices=np.concatenate(collected)
        rows.append(dict(collider=collider,body=body,piece_count=count,
            collider_to_owner_matrix=local.tolist(),frame_validation='Exact enabled owner + hash-verified native reader; vertices are already owner-local and receive no second source transform',
            source_body_local_aabb=[source_vertices.min(0).tolist(),source_vertices.max(0).tolist()],
            native_body_local_aabb=[all_vertices.min(0).tolist(),all_vertices.max(0).tolist()],
            aabb_discrepancy_m=float(max(np.abs(source_vertices.min(0)-all_vertices.min(0)).max(),np.abs(source_vertices.max(0)-all_vertices.max(0)).max())),
            approximation_discrepancy_is_diagnostic=True))
    if set(proof.get('owner_poses',{}))!=owners:
        raise ValueError('Native measured owner pose inventory mismatch')
    for body,pose in proof['owner_poses'].items():
        if (not isinstance(pose,list) or any(isinstance(x,bool) or not isinstance(x,(int,float)) for x in pose)):
            raise ValueError('Native owner pose must contain finite numeric XYZW values')
        q=np.asarray(pose,float)
        if (q.shape!=(7,) or not np.isfinite(q).all() or
                not np.isclose(np.linalg.norm(q[3:]),1.,atol=1e-5,rtol=0)):
            raise ValueError('Native owner pose has an invalid XYZW rigid frame')
    metadata=dict(schema='s300-native-tool-hull-import-v1',capture=str(path),capture_sha256=digest(path),
        source_assembly=str(assembly),source_scene_sha256=proof['source_scene_sha256'],
        source_usdc_sha256=proof['source_usdc_sha256'],native_reader_source_sha256=provenance['sha256'],
        native_reader_source=str(reader_path),scoped_colliders=requested,
        source_shape_count=len(requested),native_piece_count=sum(len(v) for v in pieces.values()),
        all_sdk_returned_pieces_retained=True,rigid_owner_local_frame=True,
        physical_geometry_modified=False,planning_clearance_modified=False,rows=rows,
        native_initialization_or_harvest_success_inferred=False)
    return pieces,metadata
