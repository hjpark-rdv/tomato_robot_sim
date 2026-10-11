"""Clear-only native structural collision proof using enclosing leaf boxes.

No USD collider or simulated pose is edited. Each actual native cooked leaf
hull is enclosed by a slightly enlarged oriented box at the measured post-fetch
rigid pose. A completed native scene query without the exact structural hit
proves that hull does not intersect that actual scene collider, including a
triangle collider whose cooked vertices are not exposed by the Python SDK.
If an enclosing box overlaps, a bounded translated query can provide an
explicit escape certificate. This moves only the query volume, never physics.
Without such a completed escape query, an overlap remains inconclusive.
"""
import hashlib
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import UsdGeom,UsdPhysics

SOURCE_BYTES=Path(__file__).read_bytes()
SOURCE_SHA256=hashlib.sha256(SOURCE_BYTES).hexdigest()


def save_imported_source(run_output):
    """Preserve lazy-imported bytes separately from the launch snapshot."""
    target=Path(run_output)/'runtime_code_sources'/'native_leaf_scene_query.py'
    target.parent.mkdir(parents=True,exist_ok=True)
    try:
        with target.open('xb') as stream:stream.write(SOURCE_BYTES)
    except FileExistsError:
        if target.read_bytes()!=SOURCE_BYTES:
            raise ValueError('Existing runtime query helper source differs; never overwrite provenance')
    metadata=dict(path=str(target),module_import_source_sha256=SOURCE_SHA256,
                source_bytes_captured_at_import=True,launch_snapshot_overwritten=False)
    cover=sys.modules.get('native_leaf_cover_query')
    if cover is not None:metadata['tighter_cover_runtime_source']=cover.save_imported_source(run_output)
    return metadata


def enclosing_box(vertices,pose,guard_m=1e-6):
    """PCA affects tightness only: every original hull support point is kept."""
    vertices=np.asarray(vertices,float);pose=np.asarray(pose,float)
    if (vertices.ndim!=2 or vertices.shape[1]!=3 or len(vertices)<4 or
            pose.shape!=(7,) or not np.isfinite(np.r_[vertices.ravel(),pose]).all() or
            abs(np.linalg.norm(pose[3:])-1.)>1e-4):
        raise ValueError('Complete finite native hull and measured XYZW pose required')
    _,_,basis=np.linalg.svd(vertices-vertices.mean(0),full_matrices=False)
    axes=basis.T
    if np.linalg.det(axes)<0:axes[:,-1]*=-1
    projected=vertices@axes;low,high=projected.min(0),projected.max(0)
    local_center=axes@((low+high)/2)
    body_rotation=Rotation.from_quat(pose[3:]).as_matrix()
    world=vertices@body_rotation.T+pose[:3]
    # The binding consumes Float3/Float4. Enclose after that serialization,
    # rather than assuming the double-precision PCA box survives unchanged.
    center=np.asarray(body_rotation@local_center+pose[:3],np.float32).astype(float)
    quaternion=np.asarray(Rotation.from_matrix(body_rotation@axes).as_quat(),np.float32).astype(float)
    serialized_axes=Rotation.from_quat(quaternion).as_matrix()
    local=(world-center)@serialized_axes
    serialization_guard=float(max(guard_m,8*np.finfo(np.float32).eps*max(1.,float(np.abs(world).max()))))
    half=np.nextafter(np.asarray(np.abs(local).max(0)+serialization_guard,np.float32),
                      np.float32(np.inf)).astype(float)
    if not np.all(np.abs(local)<=half):
        raise ValueError('Enclosing box lost native support points')
    return dict(center_m=center.tolist(),half_extent_m=half.tolist(),quaternion_xyzw=quaternion.tolist(),
                support_points=len(vertices),enclosure_guard_m=serialization_guard,
                native_argument_precision='float32; all original world support points re-enclosed after center/quaternion serialization')


def native_overlap_box(box):
    import carb
    from omni.physx import get_physx_scene_query_interface
    rows=[]
    def hit(item):
        rows.append(dict(collision=str(item.collision),rigid_body=str(item.rigid_body),
                         proto_index=int(item.protoIndex)))
        return True
    count=get_physx_scene_query_interface().overlap_box(
        carb.Float3(*box['half_extent_m']),carb.Float3(*box['center_m']),
        carb.Float4(*box['quaternion_xyzw']),hit,False)
    if int(count)!=len(rows):raise ValueError('Native scene query returned incomplete hit inventory')
    if any(not item['collision'] or not item['collision'].startswith('/') for item in rows):
        raise ValueError('Native scene query has an unresolved collider identity')
    return dict(hit_count=int(count),hits=rows,complete=True)


def _escape_directions(box,hull,pose,budget):
    """Finite proposals affect discovery only; each accepted escape is queried."""
    axes=Rotation.from_quat(box['quaternion_xyzw']).as_matrix()
    body=Rotation.from_quat(np.asarray(pose,float)[3:]).as_matrix()
    candidates=list(axes.T)+list(np.eye(3))
    if len(hull)>1:
        normals=np.asarray(hull[1],float)
        if normals.ndim==2 and normals.shape[1]==3 and np.isfinite(normals).all():
            candidates.extend(normals@body.T)
    directions=[];seen=set()
    for axis in candidates:
        norm=np.linalg.norm(axis)
        if norm<1e-12:continue
        axis=axis/norm
        for signed in (axis,-axis):
            key=tuple(np.round(signed,8))
            if key not in seen:
                seen.add(key);directions.append(signed)
                if len(directions)>=budget:return directions
    return directions


def _bounded_escape(box,hull,pose,structural,query,limit_m,budget):
    """An enclosing hull translated clear by <=limit gives an upper bound.

    The actual serialized translation norm plus the enclosure/serialization
    guard must fit the unchanged limit. An empty finite search is unknown,
    not evidence of a deep intersection. Each convex component has its own
    escape proof; no single translation of the entire decomposed leaf is
    claimed.
    """
    guard=float(box['enclosure_guard_m']);available=limit_m-2*guard
    result=dict(limit_m=float(limit_m),reserved_guard_m=guard,proposal_rounding_reserve_m=guard,candidate_budget=int(budget),
                attempts=[],certified=False,penetration_upper_bound_m=None,
                actual_geometry_translated=False,
                interpretation='Per enclosing native hull component escape upper bound; no common whole-leaf translation or physical collider edit')
    if available<=0:
        result['error']='Serialization/enclosure guard leaves no bounded escape distance';return result
    original=np.asarray(box['center_m'],float)
    for direction in _escape_directions(box,hull,pose,budget):
        center=np.asarray(original+direction*available,np.float32).astype(float)
        delta=center-original;distance=float(np.linalg.norm(delta));bound=distance+guard
        attempt=dict(direction_world=direction.tolist(),serialized_translation_world_m=delta.tolist(),
                     serialized_translation_norm_m=distance,guarded_bound_m=bound)
        result['attempts'].append(attempt)
        if bound>limit_m:
            # Float32 rounding can lengthen a requested translation. Never
            # certify that out-of-budget query by rounding the result down.
            attempt['queried']=False;attempt['reason']='Serialized displacement exceeds unchanged limit';continue
        shifted=dict(box,center_m=center.tolist());answer=query(shifted)
        hit=any(item['collision']==structural for item in answer.get('hits',[]))
        attempt.update(queried=True,box=shifted,result=answer,structural_hit=hit)
        if answer.get('complete') is not True:
            result['error']='Incomplete translated native query';return result
        if not hit:
            result.update(certified=True,penetration_upper_bound_m=float(bound),
                          accepted_attempt_index=len(result['attempts'])-1)
            return result
    result['error']='Finite translated enclosing-box search found no bounded escape; depth remains unknown'
    return result


def certify_leaf_structural_clear(stage,leaf,structural,pose,native_data,timestamp,query=None,
                                  max_escape_m=.0005,escape_direction_budget=48):
    """Run fresh controls and complete queries at the same post-fetch timestamp."""
    query=native_overlap_box if query is None else query
    row=dict(schema='native-post-fetch-enclosing-leaf-structural-query-v1',t=float(timestamp),
        leaf_collider=leaf,structural_collider=structural,certified_clear=False,
        certified_within_limit=False,penetration_upper_bound_m=None,
        native_cooked_triangle_vertices_verified=False,native_scene_query_verified=False,
        physical_colliders_changed=False,contact_authorization_changed=False,
        executed_query_helper=dict(module_import_source_path=str(Path(__file__).resolve()),
            module_import_source_sha256=SOURCE_SHA256,source_bytes_captured_at_import=True,
            launch_snapshot_is_separate=True),
        method='Complete native overlap_box queries of native cooked leaf hull enclosing OBBs; clear or guarded escape within the recorded limit')
    row['measured_leaf_pose_world_xyzw']=np.asarray(pose,float).tolist()
    try:
        if not np.isfinite(max_escape_m) or not 0<=max_escape_m<=.001:
            raise ValueError('Escape limit must stay within supported 1mm maximum')
        if not isinstance(escape_direction_budget,int) or not 1<=escape_direction_budget<=96:
            raise ValueError('Finite native escape direction budget must be 1..96')
        row.update(escape_limit_m=float(max_escape_m),escape_direction_budget=escape_direction_budget)
        lp=stage.GetPrimAtPath(leaf);sp=stage.GetPrimAtPath(structural)
        if (not structural.startswith('/World/Greenhouse/House/Environment/') or
                not lp or not lp.IsA(UsdGeom.Mesh) or not sp or not sp.IsA(UsdGeom.Mesh) or
                any(not p.HasAPI(UsdPhysics.CollisionAPI) or
                    UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not True for p in (lp,sp))):
            raise ValueError('Exact enabled leaf/structural Mesh inventory required')
        owner=sp
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        if owner:raise ValueError('Only static structural triangle collider query is supported')
        if UsdPhysics.MeshCollisionAPI(sp).GetApproximationAttr().Get()!='none':
            raise ValueError('Native query fallback is scoped to static triangle structural meshes')
        if native_data.get('kind')!='native_cooked_convex' or not native_data.get('hulls'):
            raise ValueError('Actual native cooked leaf geometry is required')
        mesh=UsdGeom.Mesh(sp);vertices=np.asarray(mesh.GetPointsAttr().Get(),float)
        if vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or not np.isfinite(vertices).all():
            raise ValueError('Finite structural source points required for query controls')
        ancestor=sp
        while ancestor:
            xf=UsdGeom.Xformable(ancestor)
            if xf and any(op.GetAttr().GetNumTimeSamples() for op in xf.GetOrderedXformOps()):
                raise ValueError('Animated structural transform is unsupported')
            ancestor=ancestor.GetParent()
        matrix=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(sp)).T
        world=vertices@matrix[:3,:3].T+matrix[:3,3]
        lo,hi=world.min(0),world.max(0)
        positive=dict(center_m=((lo+hi)/2).tolist(),half_extent_m=((hi-lo)/2+1e-4).tolist(),
                      quaternion_xyzw=[0.,0.,0.,1.])
        negative=dict(positive,center_m=(np.asarray(positive['center_m'])+[100.+float(np.ptp(world,axis=0).max()),0,0]).tolist())
        row['structural_source_control_geometry_sha256']=hashlib.sha256(
            np.ascontiguousarray(world,dtype=np.float64).tobytes()).hexdigest()
        # The source points construct controls only. The physics scene itself
        # answers whether the actual structural collider is present/intersected.
        positive_result=query(positive);negative_result=query(negative)
        positive_pass=positive_result.get('complete') is True and any(
            hit['collision']==structural for hit in positive_result['hits'])
        negative_pass=negative_result.get('complete') is True and all(
            hit['collision']!=structural for hit in negative_result['hits'])
        row['controls']=dict(positive=dict(box=positive,result=positive_result,passed=positive_pass),
                             negative=dict(box=negative,result=negative_result,passed=negative_pass))
        if not positive_pass or not negative_pass:raise ValueError('Native structural query positive/negative control failed')
        boxes=[];row['hull_queries']=boxes;tighter_budget=512
        for hull in native_data['hulls']:
            vertices=hull[0]
            box=enclosing_box(vertices,pose);result=query(box)
            leaf_hit=any(hit['collision']==leaf for hit in result.get('hits',[]))
            structural_hit=any(hit['collision']==structural for hit in result.get('hits',[]))
            item=dict(box=box,result=result,current_pose_leaf_positive_control_passed=leaf_hit,
                      structural_hit=structural_hit)
            boxes.append(item)
            if result.get('complete') is not True or not leaf_hit:
                raise ValueError('Native current-pose leaf query control failed or query incomplete')
            if structural_hit and max_escape_m:
                item['bounded_escape']=_bounded_escape(box,hull,pose,structural,query,max_escape_m,escape_direction_budget)
                if item['bounded_escape'].get('error','').startswith('Incomplete'):
                    raise ValueError('Incomplete translated native query')
            if structural_hit and not item.get('bounded_escape',{}).get('certified'):
                from native_leaf_cover_query import certify_hull_cover
                if tighter_budget<=0:raise ValueError('Finite per-contact tighter native cover query budget exhausted')
                item['tighter_cover']=certify_hull_cover(hull,pose,leaf,structural,timestamp,query,
                    controls_passed=positive_pass and negative_pass,max_queries=tighter_budget,
                    max_depth=16,structural_proposal_points=world)
                tighter_budget-=len(item['tighter_cover']['queries'])
                row['tighter_cover_query_budget']=512
                row['tighter_cover_native_queries']=512-tighter_budget
        row['native_scene_query_verified']=True
        def cleared(item):return not item['structural_hit'] or item.get('tighter_cover',{}).get('certified_clear') is True
        clear=bool(boxes and all(cleared(item) for item in boxes))
        bounded=bool(boxes and all(cleared(item) or item.get('bounded_escape',{}).get('certified') for item in boxes))
        bounds=[0. if cleared(item) else item.get('bounded_escape',{}).get('penetration_upper_bound_m') for item in boxes]
        row.update(certified_clear=clear,certified_within_limit=bounded,
                   penetration_upper_bound_m=float(max(bounds)) if bounded else None,
                   interpretation='A complete native query clearing an enclosing box or tighter complete closed-cell hull cover proves actual native hull nonintersection. Authored structural geometry proposes box axes only; it is not a native triangle certificate. Otherwise each component needs a completed native escape query whose serialized displacement plus reserved guard is within the recorded escape limit; maximum component bound is reported. Failed finite cover/escape searches remain inconclusive.')
    except Exception as error:row['error']=str(error)
    return row
