"""Save complete native cooked tool shapes using the existing geometry reader.

Read-only observation after native initialization. This never changes authoring,
contact permissions or physical collision geometry.
"""
import hashlib
import json
from pathlib import Path

import numpy as np


def capture_tool_shapes(stage, manifest, reader, paths, *, scene_path, native_scene_path,
                        body_paths, poses, timestamp_s, effective_hz, output):
    from pxr import UsdGeom, UsdPhysics
    paths=list(paths)
    if not paths or len(paths)!=len(set(paths)):
        raise ValueError('Nonempty exact unique native tool capture paths required')
    output=Path(output)
    if output.exists():raise FileExistsError(output)
    captured={};settings={}
    for path in paths:
        prim=stage.GetPrimAtPath(path)
        if (not path.startswith('/World/ActuatedTool/') or not prim or
                not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI) or
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True or
                not prim.HasAPI(UsdPhysics.MeshCollisionAPI)):
            raise ValueError('Exact enabled existing tool Mesh collider required: '+path)
        approximation=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        if approximation not in ('convexHull','convexDecomposition'):
            raise ValueError('Only complete native convex representations supported')
        cooked=reader.cooked(path)
        if cooked.get('kind')!='native_cooked_convex' or not cooked.get('hulls'):
            raise ValueError('Complete native convex representation unavailable: '+path)
        settings[path]=dict(approximation=approximation,
            owner=cooked['body'],hull_count=len(cooked['hulls']))
        captured[path]=reader.geometry_metadata()[path]
    selected_bodies=sorted({row['body'] for row in captured.values()})
    poses=np.asarray(poses,float);body_paths=list(body_paths)
    if poses.shape!=(len(body_paths),7) or not np.isfinite(poses).all():
        raise ValueError('Exact measured native owner pose inventory required')
    digest=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
    if (digest(scene_path)!=manifest['scene_sha256'] or
            digest(native_scene_path)!=manifest['compact_assets']['materialized_dependency']['sha256']):
        raise ValueError('Native tool capture requires the exact source wrapper and materialized USD dependency')
    proof=dict(schema='s300-native-cooked-tool-capture-v1',
        source_scene_sha256=digest(scene_path),source_native_scene_sha256=digest(native_scene_path),
        source_usdc_sha256=manifest['compact_assets']['materialized_dependency']['sha256'],
        source_assembly=str(Path(scene_path).resolve().parent),
        effective_hz=float(effective_hz),timestamp_s=float(timestamp_s),
        captured_before_first_trial_step=True,native_engine='physx',
        native_geometry_complete=True,selected_paths=paths,settings=settings,geometry=captured,
        owner_poses={path:poses[body_paths.index(path)].tolist() for path in selected_bodies},
        frame='All native convex vertices/normals/edges in the exact rigid-owner local frame',
        source='Existing synchronous request_convex_collision_representation RESULT_VALID; all returned hulls retained',
        geometry_reader_source_sha256=reader.policy_metadata()['helper_source_sha256'],
        physics_unchanged=True,training_eligible=False,harvest_success=None)
    output.write_text(json.dumps(proof,indent=2,allow_nan=False)+'\n')
    return proof
