"""Bounded tighter native query enclosures; physical shapes remain unchanged.

A completed native query clearing an entire face-aligned hull enclosure proves
nonintersection. If these fail, closed binary cells cover the original box;
only full convex-SAT-separated cells are discarded. Every retained leaf cell
must clear the exact structural collider through a complete native query.
Finite budget/depth failure remains unknown, never evidence of impossibility.
"""
import hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import QhullError
from leaf_contact_geometry import convex_axes,full_sat_axes,separating_gap

SOURCE_BYTES=Path(__file__).read_bytes()
SOURCE_SHA256=hashlib.sha256(SOURCE_BYTES).hexdigest()


def save_imported_source(run_output):
    target=Path(run_output)/'runtime_code_sources'/'native_leaf_cover_query.py'
    target.parent.mkdir(parents=True,exist_ok=True)
    try:
        with target.open('xb') as stream:stream.write(SOURCE_BYTES)
    except FileExistsError:
        if target.read_bytes()!=SOURCE_BYTES:raise ValueError('Existing tighter-cover runtime source differs; never overwrite')
    return dict(path=str(target),module_import_source_sha256=SOURCE_SHA256,
                source_bytes_captured_at_import=True,launch_snapshot_overwritten=False)


def _box(vertices,axes,guard_m=1e-6):
    """Float32 box enclosing every original point after argument serialization."""
    vertices=np.asarray(vertices,float);axes=np.asarray(axes,float)
    if (vertices.ndim!=2 or vertices.shape[1]!=3 or len(vertices)<4 or
            axes.shape!=(3,3) or not np.isfinite(np.r_[vertices.ravel(),axes.ravel()]).all() or
            not np.allclose(axes.T@axes,np.eye(3),atol=1e-9,rtol=0) or np.linalg.det(axes)<0):
        raise ValueError('Finite full hull and proper orthogonal box axes required')
    projected=vertices@axes;center=np.asarray(axes@((projected.min(0)+projected.max(0))/2),np.float32).astype(float)
    quaternion=np.asarray(Rotation.from_matrix(axes).as_quat(),np.float32).astype(float)
    actual_axes=Rotation.from_quat(quaternion).as_matrix();local=(vertices-center)@actual_axes
    guard=float(max(guard_m,8*np.finfo(np.float32).eps*max(1.,float(np.abs(vertices).max()))))
    half=np.nextafter(np.asarray(np.abs(local).max(0)+guard,np.float32),np.float32(np.inf)).astype(float)
    if not np.all(np.abs(local)<=half):raise ValueError('Serialized box lost source support points')
    return dict(center_m=center.tolist(),half_extent_m=half.tolist(),quaternion_xyzw=quaternion.tolist(),enclosure_guard_m=guard,support_points=len(vertices),native_argument_precision='float32; original points re-enclosed after center/quaternion serialization')


def _face_frames(normals,pca):
    seen=set()
    for n in normals:
        n=np.asarray(n,float);n/=np.linalg.norm(n)
        key=tuple(np.round(n if n[np.argmax(np.abs(n))]>=0 else -n,8))
        if key in seen:continue
        seen.add(key);choices=[axis-n*np.dot(n,axis) for axis in pca.T]
        tangent=max(choices,key=lambda a:np.dot(a,a));tangent/=np.linalg.norm(tangent)
        yield np.column_stack((n,tangent,np.cross(n,tangent)))


def certify_hull_cover(hull,pose,leaf,structural,timestamp,query,*,
                       controls_passed=False,max_queries=512,max_depth=12,max_face_boxes=48,
                       structural_proposal_points=None,max_proposal_boxes=8):
    row=dict(schema='native-tight-cooked-hull-cover-v1',t=float(timestamp),leaf_collider=leaf,structural_collider=structural,
        certified_clear=False,certified_within_limit=False,penetration_upper_bound_m=None,
        physical_colliders_changed=False,actual_geometry_translated=False,contact_authorization_changed=False,
        limit_m=.0005,query_budget=max_queries,max_depth=max_depth,queries=[],pruned_cells=[],
        executed_cover_helper=dict(module_import_source_path=str(Path(__file__).resolve()),module_import_source_sha256=SOURCE_SHA256,source_bytes_captured_at_import=True,launch_snapshot_is_separate=True),
        proof='Clear complete full-hull enclosure, or complete closed-cell union cover with full-SAT-proven outside cells discarded')
    try:
        pose=np.asarray(pose,float);vertices=np.asarray(hull[0],float)
        if (not controls_passed or not np.isfinite(timestamp) or timestamp<0 or
                not isinstance(max_queries,int) or not 1<=max_queries<=512 or
                not isinstance(max_depth,int) or not 0<=max_depth<=16 or pose.shape!=(7,) or
                not np.isfinite(np.r_[pose,vertices.ravel()]).all() or abs(np.linalg.norm(pose[3:])-1)>1e-4):
            raise ValueError('Verified same-scene controls, finite measured pose and bounded budget are required')
        rotation=Rotation.from_quat(pose[3:]).as_matrix();world=vertices@rotation.T+pose[:3]
        normals,edges=convex_axes(world)
        _,_,basis=np.linalg.svd(world-world.mean(0),full_matrices=False);pca=basis.T
        if np.linalg.det(pca)<0:pca[:,-1]*=-1
        row['native_hull_world_sha256']=hashlib.sha256(np.ascontiguousarray(world).tobytes()).hexdigest()
        def query_box(box,kind,**details):
            if len(row['queries'])>=max_queries:raise ValueError('Finite tighter-cover native query budget exhausted')
            answer=query(box)
            if answer.get('complete') is not True:raise ValueError('Incomplete tighter-cover native query')
            hit=any(item['collision']==structural for item in answer.get('hits',[]))
            row['queries'].append(dict(box=box,kind=kind,result=answer,structural_hit=hit,**details))
            return hit,answer
        # The whole-hull enclosure is a current-pose control, not merely a
        # source triangle query. Every candidate retains all native support.
        original=_box(world,pca);hit,answer=query_box(original,'pca_full_hull')
        if not any(item['collision']==leaf for item in answer.get('hits',[])):
            raise ValueError('Measured-pose native leaf positive control failed')
        if not hit:
            row.update(certified_clear=True,certified_within_limit=True,penetration_upper_bound_m=0.,method='Complete native PCA hull enclosure');return row
        if structural_proposal_points is not None:
            # Authored structural points propose orientations only. Their SAT
            # gap is never a native distance certificate: even a changed or
            # differently cooked source must clear a complete actual query.
            proposals=[]
            try:
                proposed=np.asarray(structural_proposal_points,float)
                if (proposed.ndim!=2 or proposed.shape[1]!=3 or len(proposed)<4 or
                        not np.isfinite(proposed).all() or not isinstance(max_proposal_boxes,int) or
                        not 0<=max_proposal_boxes<=16):raise ValueError('Finite bounded proposal geometry required')
                pn,pe=convex_axes(proposed);candidate_axes=full_sat_axes(normals,pn,edges,pe)
                scores=[]
                for offset in range(0,len(candidate_axes),128):
                    chunk=candidate_axes[offset:offset+128];a=world@chunk.T;b=proposed@chunk.T
                    scores.extend(np.maximum(a.min(0)-b.max(0),b.min(0)-a.max(0)).tolist())
                order=np.argsort(scores)[::-1][:max_proposal_boxes]
                row['orientation_proposal_source_sha256']=hashlib.sha256(np.ascontiguousarray(proposed).tobytes()).hexdigest()
                row['orientation_proposal_is_not_native_triangle_geometry']=True
                proposals=[(candidate_axes[i],scores[i]) for i in order]
            except (np.linalg.LinAlgError,QhullError,ValueError) as error:
                # Bad orientation proposals cannot authorize anything.
                row['orientation_proposal_error']=str(error)
            for pi,(axis,score) in enumerate(proposals):
                axes=next(_face_frames([axis],pca));box=_box(world,axes)
                hit,answer=query_box(box,'authored_geometry_suggested_full_hull',proposal_index=pi,authored_proposal_axis_gap_m=score)
                if not any(item['collision']==leaf for item in answer.get('hits',[])):
                    raise ValueError('Authored-proposal native current-pose leaf control failed')
                if not hit:
                    row.update(certified_clear=True,certified_within_limit=True,penetration_upper_bound_m=0.,method='Complete native full-hull enclosure oriented by authored-geometry proposal');return row
        for fi,axes in enumerate(_face_frames(normals,pca)):
            if fi>=max_face_boxes:break
            box=_box(world,axes);hit,answer=query_box(box,'face_aligned_full_hull',face_index=fi)
            if not any(item['collision']==leaf for item in answer.get('hits',[])):
                raise ValueError('Face-aligned native current-pose leaf control failed')
            if not hit:
                row.update(certified_clear=True,certified_within_limit=True,penetration_upper_bound_m=0.,method='Complete native face-aligned full-hull enclosure');return row
        # Closed children share their split plane and union to the parent.
        # Query boxes enclose all cell corners, including serialization guard.
        axes=Rotation.from_quat(original['quaternion_xyzw']).as_matrix()
        stack=[(np.asarray(original['center_m']),np.asarray(original['half_extent_m']),0,'root')]
        signs=np.array([[x,y,z] for x in (-1.,1.) for y in (-1.,1.) for z in (-1.,1.)])
        clear_cells=0
        while stack:
            center,half,depth,identity=stack.pop();corners=signs*half@axes.T+center
            cn,ce=convex_axes(corners);gap=separating_gap(world,corners,full_sat_axes(normals,cn,edges,ce))
            guard=float(max(original['enclosure_guard_m'],8*np.finfo(np.float32).eps*max(1.,float(np.abs(corners).max()))))
            if gap>4*guard:
                row['pruned_cells'].append(dict(identity=identity,depth=depth,full_sat_separation_m=gap,required_guard_m=4*guard));continue
            box=_box(corners,axes,guard);hit,_=query_box(box,'closed_cell',cell=identity,depth=depth,full_sat_hull_cell_gap_m=gap)
            if not hit:clear_cells+=1;continue
            if depth>=max_depth:raise ValueError('Finite tighter-cover subdivision depth exhausted; geometry unknown')
            axis=int(np.argmax(half));child_half=half.copy();child_half[axis]/=2
            delta=axes[:,axis]*child_half[axis]
            stack.append((center-delta,child_half.copy(),depth+1,identity+'0'))
            stack.append((center+delta,child_half.copy(),depth+1,identity+'1'))
        row.update(certified_clear=True,certified_within_limit=True,penetration_upper_bound_m=0.,
                   method='Complete native closed binary-cell union enclosure',cleared_leaf_cells=clear_cells,complete_union_cover=True)
    except Exception as error:row['error']=str(error)
    return row
