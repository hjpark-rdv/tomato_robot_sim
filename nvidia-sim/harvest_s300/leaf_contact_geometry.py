"""Conservative post-solve separation certificates for compliant leaf contacts.

Native callback separation is retained verbatim. Exact leaf/tool and
active-leaf/background-leaf pairs are supported. An additional opt-in supports
the selected truss's fruit/tool, fruit/fruit, fruit/own-branch-capsule and
tool/own-branch-capsule pairs. Exact active-plant fruit/leaf, fruit/capsule and
capsule/leaf pairs use a separate full-plant geometry inventory. These are
post-solve geometry checks; contact authorization remains a separate policy.
A projected overlap bounds the translation needed to separate that convex
pair from above. Face normals and edge-cross-edge axes are both required for
complete convex SAT. Every cooked hull pair must meet the original limit.
"""
import hashlib
from pathlib import Path
import re
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics

SOURCE_BYTES=Path(__file__).read_bytes()
SOURCE_SHA256=hashlib.sha256(SOURCE_BYTES).hexdigest()


def separating_gap(a, b, axes):
    axes=np.asarray(axes,float)
    length=np.linalg.norm(axes,axis=1);axes=axes[length>1e-10]/length[length>1e-10,None]
    if not len(axes):return float('-inf')
    result=float('-inf')
    # Edge-cross-edge axes can number in the thousands. Bound temporary
    # support matrices without omitting axes or loosening the threshold.
    for offset in range(0,len(axes),128):
        chunk=axes[offset:offset+128]
        pa=np.asarray(a)@chunk.T;pb=np.asarray(b)@chunk.T
        result=max(result,float(np.maximum(pa.min(0)-pb.max(0),pb.min(0)-pa.max(0)).max()))
    return result


def unique_directions(vectors):
    vectors=np.asarray(vectors,float).reshape(-1,3)
    lengths=np.linalg.norm(vectors,axis=1)
    vectors=vectors[lengths>1e-10]/lengths[lengths>1e-10,None]
    if not len(vectors):return np.empty((0,3))
    # Axis sign does not change support separation.
    largest=np.argmax(np.abs(vectors),axis=1)
    vectors*=np.where(vectors[np.arange(len(vectors)),largest]<0,-1.,1.)[:,None]
    _,indices=np.unique(np.round(vectors,8),axis=0,return_index=True)
    return vectors[np.sort(indices)]


def convex_axes(vertices):
    hull=ConvexHull(vertices)
    edges={}
    for fi,face in enumerate(hull.simplices):
        for one,two in ((face[0],face[1]),(face[1],face[2]),(face[2],face[0])):
            edges.setdefault(tuple(sorted((int(one),int(two)))),[]).append(fi)
    true_edges=[]
    for (one,two),faces in edges.items():
        # Ignore triangulation diagonals within a single planar hull face.
        if len(faces)==2 and np.linalg.norm(np.cross(
                hull.equations[faces[0],:3],hull.equations[faces[1],:3]))<1e-8:
            continue
        true_edges.append(vertices[two]-vertices[one])
    return unique_directions(hull.equations[:,:3]),unique_directions(true_edges)


def full_sat_axes(an,bn,ae,be):
    cross=np.cross(np.asarray(ae)[:,None,:],np.asarray(be)[None,:,:]).reshape(-1,3)
    return unique_directions(np.concatenate((an,bn,cross)))



def convex_pair_gaps(first, second, clear_guard_m=.000001):
    """Full pair coverage with an exact positive world-axis shortcut.

    World AABB separation is itself a valid support-axis certificate. Only a
    strictly positive gap beyond the existing numerical guard avoids complete
    face-normal/edge-cross SAT; overlapping or ambiguous pairs keep every old
    axis. This leaves deep controls and penetration upper bounds unchanged.
    The reported positive minimum support gap can differ from the old SAT-axis
    set and is only a separation certificate, not an exact Euclidean distance.
    """
    if clear_guard_m!=.000001:raise ValueError('Existing numerical clear guard must remain 1um')
    bounds=[]
    for hulls in (first,second):
        rows=[]
        for v,n,e in hulls:
            v=np.asarray(v,float)
            if v.ndim!=2 or v.shape[1]!=3 or not len(v) or not np.isfinite(v).all():
                raise ValueError('Finite complete native convex vertices required')
            rows.append((v.min(0),v.max(0)))
        if not rows:raise ValueError('Missing native convex hull representation')
        bounds.append(rows)
    gaps=[];separated=0
    for ai,(av,an,ae) in enumerate(first):
        alo,ahi=bounds[0][ai]
        for bi,(bv,bn,be) in enumerate(second):
            blo,bhi=bounds[1][bi]
            gap=float(np.maximum(alo-bhi,blo-ahi).max())
            if gap>clear_guard_m:separated+=1
            else:gap=separating_gap(av,bv,full_sat_axes(an,bn,ae,be))
            gaps.append(gap)
    return gaps,separated


def triangle_components(vertices, faces):
    """Connected exact-coordinate mesh components and closed surface flags."""
    _,inverse=np.unique(vertices,axis=0,return_inverse=True)
    welded=inverse[faces]
    parents=np.arange(len(faces));first={}
    def find(i):
        while parents[i]!=i:
            parents[i]=parents[parents[i]];i=parents[i]
        return i
    for fi,face in enumerate(welded):
        for vertex in face:
            if vertex in first:parents[find(fi)]=find(first[vertex])
            else:first[vertex]=fi
    groups={}
    for fi in range(len(faces)):groups.setdefault(find(fi),[]).append(fi)
    result=[]
    for group in groups.values():
        part=welded[group]
        edges=np.sort(np.concatenate((part[:,[0,1]],part[:,[1,2]],part[:,[2,0]])),axis=1)
        _,counts=np.unique(edges,axis=0,return_counts=True)
        result.append((np.asarray(group,int),bool(len(counts) and np.all(counts==2))))
    return result


def point_inside_closed_triangles(point, triangles):
    """Ray parity with independent directions; ambiguous topology fails closed."""
    answers=[]
    edge1=triangles[:,1]-triangles[:,0];edge2=triangles[:,2]-triangles[:,0]
    for direction in ([1.,.371,.617],[.293,1.,.537],[.431,.229,1.]):
        direction=np.asarray(direction);direction/=np.linalg.norm(direction)
        h=np.cross(np.broadcast_to(direction,edge2.shape),edge2)
        det=np.einsum('ij,ij->i',edge1,h);nonparallel=np.abs(det)>1e-14
        inv=np.zeros_like(det);inv[nonparallel]=1./det[nonparallel]
        delta=point-triangles[:,0]
        u=inv*np.einsum('ij,ij->i',delta,h)
        q=np.cross(delta,edge1)
        v=inv*(q@direction)
        distance=inv*np.einsum('ij,ij->i',edge2,q)
        hits=nonparallel&(u>=-1e-10)&(v>=-1e-10)&(u+v<=1+1e-10)&(distance>1e-8)
        if np.any(hits&((u<1e-8)|(v<1e-8)|(u+v>1-1e-8))):continue
        answers.append(bool(np.count_nonzero(hits)%2))
    if not answers or any(value!=answers[0] for value in answers):
        raise ValueError('Ambiguous closed triangle mesh containment')
    return answers[0]


def convex_triangle_mesh_gap(vertices, normals, edges, data):
    """Full SAT against each triangle, with closed-volume containment rejection.

    Triangle surfaces have zero thickness. We certify only positive separation,
    never a mesh penetration depth from their support interval.
    """
    triangles=data['triangles'];triangle_normals=data['triangle_normals'];triangle_edges=data['triangle_edges']
    a_min,a_max=vertices.min(0),vertices.max(0)
    lo=triangles.min(1);hi=triangles.max(1)
    bbox=np.maximum(a_min-hi,lo-a_max).max(1)
    gaps=bbox.copy()
    # A positive world-axis separation already certifies a disjoint pair.
    for ti in np.flatnonzero(bbox<=1e-6):
        tn=triangle_normals[ti:ti+1];te=triangle_edges[ti]
        gaps[ti]=separating_gap(vertices,triangles[ti],full_sat_axes(normals,tn,edges,te))
    minimum=float(gaps.min())
    if minimum>1e-6:
        centre=vertices.mean(0)
        for indices,closed in data['components']:
            if closed and point_inside_closed_triangles(centre,triangles[indices]):
                raise ValueError('Leaf convex hull is inside a closed background leaf component')
    return minimum


def enclosing_component_hulls(data):
    """Convex supersets of every authored mesh surface/closed component volume.

    All source component points are retained for support projection; the hull
    vertices are used only to derive full SAT face/edge axes. Thus no cooking,
    simplification or triangulation can under-cover an authored triangle.
    Degenerate components cannot provide this certificate and fail closed.
    """
    result=[]
    for component_index,(indices,closed) in enumerate(data['components']):
        points=np.unique(data['triangles'][indices].reshape(-1,3),axis=0)
        hull=ConvexHull(points)
        normals,edges=convex_axes(points[hull.vertices])
        if not len(normals) or not len(edges):raise ValueError('Degenerate component enclosing hull')
        result.append(dict(component_index=component_index,closed=closed,
                           triangle_count=len(indices),vertices=points,normals=normals,edges=edges,
                           hull_vertex_count=len(hull.vertices)))
    if not result:raise ValueError('Missing authored triangle components')
    return result


def convex_component_hull_gaps(vertices,normals,edges,components):
    """Per-component translation upper bounds, not a global union MTV.

    Separating from a convex superset separates from its actual surface and
    enclosed volume. Every active hull/component pair must independently meet
    the original bound, matching the existing cooked convex hull-pair check.
    """
    return [separating_gap(vertices,c['vertices'],
                          full_sat_axes(normals,c['normals'],edges,c['edges']))
            for c in components]


def convex_capsule_gap(vertices, normals, edges, endpoints, radius):
    """Conservative signed support separation from an exact analytic capsule.

    A capsule is the segment Minkowski-summed with a radius-r ball, hence its
    support interval along a unit axis is endpoint min/max plus/minus r. Any
    negative projected gap supplies an upper bound on the translation needed
    to separate the entire convex/capsule pair along that axis. Sampling extra
    axes improves that bound; missing an axis can reject, never understate deep
    intersection or certify intersecting geometry as clear.
    """
    first,last=np.asarray(endpoints,float);segment=last-first
    length2=float(segment@segment)
    fraction=np.clip((vertices-first)@segment/length2,0.,1.) if length2>1e-20 else np.zeros(len(vertices))
    toward=vertices-(first+fraction[:,None]*segment)
    cross=np.cross(edges,segment)
    axes=unique_directions(np.concatenate((normals,segment[None],toward,cross)))
    if not len(axes):return float('-inf')
    av=vertices@axes.T;caps=np.asarray(endpoints)@axes.T
    lo=caps.min(0)-radius;hi=caps.max(0)+radius
    return float(np.maximum(av.min(0)-hi,lo-av.max(0)).max())


class CookedLeafContactGeometry:
    def __init__(self, stage, manifest, body_paths, *, verify_fruit_contacts=False,
                 verify_background_component_hulls=False,verify_structural_scene_queries=False):
        self.stage=stage;self.body_ids={p:i for i,p in enumerate(body_paths)}
        self.leaves={p for r in manifest.get('compliant_foliage',{}).get('leaves',[]) for p in r['meshes'] if '/Appendage_' in p}
        from harvest_contact_scope import background_leaf_colliders_from_manifest
        self.background_leaf_inventory=background_leaf_colliders_from_manifest(stage,manifest,body_paths)
        self.background_leaves=set(self.background_leaf_inventory)
        self.leaves.update(self.background_leaves)
        self.verify_fruit_contacts=bool(verify_fruit_contacts)
        self.verify_background_component_hulls=bool(verify_background_component_hulls)
        self.verify_structural_scene_queries=bool(verify_structural_scene_queries)
        self.fruits=set();self.capsules=set();self.retarget_history=[]
        self.active_plant_fruits=set();self.active_plant_capsules=set();self.active_plant_leaves=set()
        self.structural_colliders=set();self.structural_capsules=set()
        if self.verify_fruit_contacts:
            target=manifest.get('target',{});match=re.fullmatch(r'(Truss_\d+)_Fruit_\d+',target.get('name',''))
            root=manifest.get('plant',{}).get('root','').rstrip('/')
            if not match or not root or target.get('path')!=root+'/'+target['name']:
                raise ValueError('Exact target truss identity required for opt-in fruit verification')
            self.active_plant_fruits={f['path']+'/Collider' for f in manifest['plant']['fruits']
                if re.fullmatch(r'Truss_\d+_Fruit_\d+',f['name']) and f['path']==root+'/'+f['name']}
            self.active_plant_leaves={path for path in self.leaves if path.startswith(root+'/')}
            if stage is not None:
                for body in manifest['plant'].get('bodies',[]):
                    if body['path']!=root+'/'+body['name']:continue
                    path=body['path']+'/Collider';prim=stage.GetPrimAtPath(path)
                    if (prim and prim.IsA(UsdGeom.Capsule) and prim.HasAPI(UsdPhysics.CollisionAPI) and
                            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True):
                        self.active_plant_capsules.add(path)
                # Exact authored enabled collider inventory, not a permission
                # to contact the house. Unsupported native representations
                # remain eligible for an explicit inconclusive record.
                environment=stage.GetPrimAtPath('/World/Greenhouse/House/Environment')
                if environment:
                    for prim in Usd.PrimRange(environment,Usd.TraverseInstanceProxies()):
                        if (prim.HasAPI(UsdPhysics.CollisionAPI) and
                                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True and
                                (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Capsule))):
                            path=str(prim.GetPath());self.structural_colliders.add(path)
                            if prim.IsA(UsdGeom.Capsule):self.structural_capsules.add(path)
            self.retarget_fruit_truss(manifest,target['name'])
        self.cache={};self.records=[]

    def retarget_fruit_truss(self,manifest,target_name):
        """Change only exact inventory scope at a measured cycle boundary.

        Physics, cached shapes and previously recorded certificates are retained.
        This is geometry validity scope, not a change in contact authorization.
        """
        if not self.verify_fruit_contacts:return dict(enabled=False)
        match=re.fullmatch(r'(Truss_\d+)_Fruit_\d+',target_name)
        root=manifest.get('plant',{}).get('root','').rstrip('/')
        if not match or not root:raise ValueError('Exact retarget fruit/truss identity required')
        fruits={f['path']+'/Collider' for f in manifest['plant']['fruits']
                if f['name'].startswith(match[1]+'_Fruit_') and f['path']==root+'/'+f['name']}
        if root+'/'+target_name+'/Collider' not in fruits:raise ValueError('Retarget fruit collider inventory is missing')
        pattern=re.compile(re.escape(match[1])+r'_(?:Pedicel_(?:proximal|distal)_\d+_\d+|Rachis_\d+|Truss_\d+_Peduncle_\d+)')
        capsules={body['path']+'/Collider' for body in manifest['plant'].get('bodies',[])
                  if pattern.fullmatch(body['name']) and body['path']==root+'/'+body['name']}
        self.fruits,self.capsules=fruits,capsules
        row=dict(index=len(self.retarget_history),target_name=target_name,truss=match[1],
                 exact_fruit_colliders=sorted(fruits),exact_own_truss_capsules=sorted(capsules),
                 reason='Cycle-boundary exact geometry inventory selection; contact policy unchanged')
        self.retarget_history.append(row)
        return row

    def eligible(self, a, b):
        if a in self.leaves and b in self.background_leaves or b in self.leaves and a in self.background_leaves:
            return a!=b
        if self.verify_fruit_contacts:
            for leaf,other in ((a,b),(b,a)):
                if leaf not in self.active_plant_leaves or other not in self.structural_colliders:continue
                first=self.stage.GetPrimAtPath(leaf);second=self.stage.GetPrimAtPath(other)
                if (first and first.IsA(UsdGeom.Mesh) and
                        all(prim and prim.HasAPI(UsdPhysics.CollisionAPI) and
                            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True
                            for prim in (first,second))):return True
            # This inventory supports measured physical validity of the active
            # plant only. It never permits tool contact with another truss,
            # nor changes contact authorization or any physical collider.
            for one,other in ((a,b),(b,a)):
                active_pair=((one in self.active_plant_fruits and other in self.active_plant_leaves|self.background_leaves|self.active_plant_capsules) or
                             (one in self.active_plant_capsules and other in self.active_plant_leaves|self.background_leaves))
                if not active_pair or self.stage is None:continue
                first=self.stage.GetPrimAtPath(one);second=self.stage.GetPrimAtPath(other)
                if not all(prim and prim.HasAPI(UsdPhysics.CollisionAPI) and
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True for prim in (first,second)):continue
                if ((first.IsA(UsdGeom.Mesh) if one in self.active_plant_fruits else first.IsA(UsdGeom.Capsule)) and
                        (second.IsA(UsdGeom.Capsule) if other in self.active_plant_capsules else second.IsA(UsdGeom.Mesh))):
                    return True
            for capsule,tool in ((a,b),(b,a)):
                if capsule not in self.capsules or not tool.startswith('/World/ActuatedTool/') or self.stage is None:continue
                cp=self.stage.GetPrimAtPath(capsule);tp=self.stage.GetPrimAtPath(tool)
                if (cp and cp.IsA(UsdGeom.Capsule) and tp and tp.IsA(UsdGeom.Mesh) and
                        all(prim.HasAPI(UsdPhysics.CollisionAPI) and
                            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True for prim in (cp,tp))):
                    return True
            for fruit,other in ((a,b),(b,a)):
                if fruit not in self.fruits or self.stage is None:continue
                is_tool=other.startswith('/World/ActuatedTool/')
                is_same_truss_fruit=other in self.fruits and other!=fruit
                if not is_tool and other not in self.capsules and not is_same_truss_fruit:continue
                prim=self.stage.GetPrimAtPath(other)
                if (prim and prim.HasAPI(UsdPhysics.CollisionAPI) and
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True and
                        (is_tool or prim.IsA(UsdGeom.Capsule) or is_same_truss_fruit and prim.IsA(UsdGeom.Mesh))):
                    if is_same_truss_fruit:
                        one=self.stage.GetPrimAtPath(fruit)
                        if (not one or not one.IsA(UsdGeom.Mesh) or not one.HasAPI(UsdPhysics.CollisionAPI) or
                                UsdPhysics.CollisionAPI(one).GetCollisionEnabledAttr().Get() is not True):continue
                    return True
        for leaf, other in ((a,b),(b,a)):
            if leaf not in self.leaves:continue
            if other.startswith('/World/ActuatedTool/'):return True
            if self.stage is not None and other.startswith('/World/Greenhouse/Plants/'):
                prim=self.stage.GetPrimAtPath(other)
                name=str(prim.GetCustomDataByKey('sourceName') or '') if prim else ''
                if name.startswith(('CompoundLeaf_','Apical_Young_Leaf_')):return True
        return False

    def cooked(self, path):
        if path in self.cache:return self.cache[path]
        p=self.stage.GetPrimAtPath(path);owner=p
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        if owner and str(owner.GetPath()) not in self.body_ids:raise ValueError('Missing measured collider owner')
        if not owner and not path.startswith('/World/Greenhouse/Plants/') and path not in self.structural_colliders:
            raise ValueError('Unexpected static collider')
        if path in self.capsules|self.active_plant_capsules|self.structural_capsules:
            data=self.capsule_data(path)
            self.cache[path]=data
            return data
        if path in self.structural_colliders:
            approximation=UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()
            if approximation not in ('convexHull','convexDecomposition'):
                raise ValueError('Structural collider native triangle representation unavailable; authored source is not a certificate: '+str(approximation))
        from pxr import UsdUtils, PhysicsSchemaTools
        from omni.physx import get_physx_cooking_interface
        from omni.physx.bindings._physx import PhysxCollisionRepresentationResult
        result=[]; cooking_errors=[]
        def ready(status, convexes):
            if status!=PhysxCollisionRepresentationResult.RESULT_VALID:
                cooking_errors.append(str(status));return
            result.extend([np.array([[v.x,v.y,v.z] for v in c.vertices],float) for c in convexes])
        get_physx_cooking_interface().request_convex_collision_representation(
            stage_id=UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt(),
            collision_prim_id=PhysicsSchemaTools.sdfPathToInt(path),run_asynchronously=False,on_result=ready)
        if not result:
            if not owner and path.startswith('/World/Greenhouse/Plants/'):
                data=self.static_triangle_data(path)
                data['native_convex_request_status']=cooking_errors
                self.cache[path]=data
                return data
            raise ValueError('No native cooked convex hulls: '+', '.join(cooking_errors))
        cache=UsdGeom.XformCache()
        local=np.asarray(cache.GetLocalToWorldTransform(p)).T
        if owner:local=np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(owner)).T)@local
        hulls=[]
        for vertices in result:
            vertices=vertices@local[:3,:3].T+local[:3,3]
            normals,edges=convex_axes(vertices)
            hulls.append((vertices,normals,edges))
        data=dict(body=str(owner.GetPath()) if owner else None,hulls=hulls,kind='native_cooked_convex')
        self.cache[path]=data
        return data

    def capsule_data(self,path):
        """Authored exact primitive geometry in its measured rigid-owner frame."""
        prim=self.stage.GetPrimAtPath(path)
        if path not in self.capsules|self.active_plant_capsules|self.structural_capsules or not prim or not prim.IsA(UsdGeom.Capsule):
            raise ValueError('Exact active-plant or structural capsule inventory required')
        capsule=UsdGeom.Capsule(prim)
        axis=capsule.GetAxisAttr().Get();radius=capsule.GetRadiusAttr().Get();height=capsule.GetHeightAttr().Get()
        if axis not in ('X','Y','Z') or radius is None or height is None or not np.isfinite([radius,height]).all() or radius<=0 or height<0:
            raise ValueError('Finite supported capsule dimensions/axis required')
        if any(attr.GetNumTimeSamples() for attr in (capsule.GetAxisAttr(),capsule.GetRadiusAttr(),capsule.GetHeightAttr())):
            raise ValueError('Animated capsule dimensions are unsupported')
        owner=prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        structural_static=not owner and path in self.structural_capsules
        if not structural_static and (not owner or str(owner.GetPath()) not in self.body_ids):
            raise ValueError('Capsule has no exact measured rigid owner')
        cache=UsdGeom.XformCache()
        owner_transform=np.eye(4) if structural_static else np.asarray(cache.GetLocalToWorldTransform(owner)).T
        collider_transform=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        if structural_static:
            ancestor=prim
            while ancestor:
                xform=UsdGeom.Xformable(ancestor)
                if xform and any(op.GetAttr().GetNumTimeSamples() for op in xform.GetOrderedXformOps()):
                    raise ValueError('Animated structural capsule transform is unsupported')
                ancestor=ancestor.GetParent()
        if not np.isfinite(owner_transform).all() or not np.isfinite(collider_transform).all():
            raise ValueError('Nonfinite capsule transform')
        # Tensor rigid poses carry no scale: accept only rigid owner frames.
        if (np.linalg.det(owner_transform[:3,:3])<=0 or
                not np.allclose(owner_transform[:3,:3].T@owner_transform[:3,:3],np.eye(3),atol=1e-6,rtol=0)):
            raise ValueError('Scaled capsule rigid owner is unsupported')
        local=np.linalg.inv(owner_transform)@collider_transform;linear=local[:3,:3]
        scale=float(np.linalg.norm(linear[:,0]))
        if (scale<=0 or np.linalg.det(linear)<=0 or
                not np.allclose(linear.T@linear,np.eye(3)*scale**2,atol=1e-9,rtol=1e-6)):
            raise ValueError('Nonuniform or reflected capsule scale is unsupported')
        direction=linear[:,('X','Y','Z').index(axis)]
        endpoints=local[:3,3]+np.array([-.5,.5])[:,None]*float(height)*direction
        return dict(body=None if structural_static else str(owner.GetPath()),kind='exact_usd_capsule',endpoints=endpoints,
                    radius=float(radius)*scale,source_axis=axis,source_height=float(height),source_radius=float(radius),
                    collider_local_transform=local,primitive_consistency='Exact Capsule radius/height/axis with rigid owner and uniform local scale')

    def static_triangle_data(self,path):
        """Exact authored static leaf triangles when native API cannot expose them.

        The native SDK exposes a convex representation query only. This fallback
        explicitly records that cooked triangle geometry is unverified and
        accepts only approximation=none. The default certificate is clear-only;
        opt-in enclosing component hulls provide conservative per-pair bounds.
        """
        prim=self.stage.GetPrimAtPath(path)
        name=str(prim.GetCustomDataByKey('sourceName') or '') if prim else ''
        if (not prim or not path.startswith('/World/Greenhouse/Plants/') or
                not name.startswith(('CompoundLeaf_','Apical_Young_Leaf_')) or
                not prim.HasAPI(UsdPhysics.CollisionAPI) or
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True or
                UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()!='none'):
            raise ValueError('Unsupported static triangle source identity or approximation')
        parent=prim
        while parent and not parent.IsPseudoRoot():
            if parent.HasAPI(UsdPhysics.RigidBodyAPI):raise ValueError('Triangle fallback must be static')
            if any(attr.GetNumTimeSamples() for attr in parent.GetAttributes()
                   if attr.GetName().startswith('xformOp:')):
                raise ValueError('Animated static triangle transform')
            parent=parent.GetParent()
        mesh=UsdGeom.Mesh(prim)
        if not mesh:raise ValueError('Static triangle source is not a mesh')
        attributes=(mesh.GetPointsAttr(),mesh.GetFaceVertexCountsAttr(),mesh.GetFaceVertexIndicesAttr())
        if any(attr.GetNumTimeSamples() for attr in attributes):raise ValueError('Animated triangle topology/points')
        points=np.asarray(attributes[0].Get(),float);counts=np.asarray(attributes[1].Get(),int)
        indices=np.asarray(attributes[2].Get(),int)
        if (points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all() or
                not len(counts) or np.any(counts!=3) or len(indices)!=3*len(counts) or
                indices.min()<0 or indices.max()>=len(points)):
            raise ValueError('Finite explicit triangle mesh required')
        transform=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
        if not np.isfinite(transform).all():raise ValueError('Nonfinite static triangle transform')
        weld=prim.GetAttribute('physxTriangleMeshCollision:weldTolerance').Get()
        if weld is not None and np.isfinite(weld) and weld>0:
            raise ValueError('Explicit native triangle welding changes source geometry')
        source_hash=hashlib.sha256(points.tobytes()+indices.tobytes()+transform.tobytes()).hexdigest()
        vertices=points@transform[:3,:3].T+transform[:3,3];faces=indices.reshape(-1,3)
        triangles=vertices[faces]
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        lengths=np.linalg.norm(normals,axis=1)
        if np.any(lengths<=1e-14):raise ValueError('Degenerate authored collision triangles')
        normals/=lengths[:,None]
        edges=np.stack((triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,1],
                        triangles[:,0]-triangles[:,2]),axis=1)
        return dict(body=None,kind='authored_static_triangle_fallback',vertices=vertices,faces=faces,
                    triangles=triangles,triangle_normals=normals,triangle_edges=edges,
                    components=triangle_components(vertices,faces),source_geometry_sha256=source_hash,
                    source_name=name,native_cooked_triangle_verified=False,
                    geometry_consistency='Static source triangle points/indices/world transform hashed; approximation=none',
                    uncertainty='Native cooked triangle query unavailable; exact authored source used for clear separation or opt-in enclosing-component hull per-pair bounds',
                    native_weld_tolerance=repr(weld))

    def verify(self, a, b, poses, timestamp, reported_separation):
        active_fruit_leaf=(a in self.active_plant_fruits and b in self.active_plant_leaves or b in self.active_plant_fruits and a in self.active_plant_leaves)
        active_capsule_leaf=(a in self.active_plant_capsules and b in self.active_plant_leaves or b in self.active_plant_capsules and a in self.active_plant_leaves)
        active_fruit_capsule=(a in self.active_plant_fruits and b in self.active_plant_capsules or b in self.active_plant_fruits and a in self.active_plant_capsules)
        own_fruit_capsule=(a in self.fruits and b in self.capsules or b in self.fruits and a in self.capsules)
        structural_leaf=(a in self.active_plant_leaves and b in self.structural_colliders or
                         b in self.active_plant_leaves and a in self.structural_colliders)
        passive_leaf=(a in self.background_leaves or b in self.background_leaves)
        row=dict(t=float(timestamp),a=a,b=b,native_callback_separation_m=float(reported_separation),
                 certified_clear=False,certified_within_limit=False,
                 limit_m=.0005,numerical_guard_m=.000001,
                 contact_scope='declared_passive_leaf_measured_geometry' if passive_leaf else
                               'active_plant_leaf_structural' if structural_leaf else
                               'active_plant_fruit_leaf' if active_fruit_leaf else
                               'active_plant_capsule_leaf' if active_capsule_leaf else
                               'active_plant_fruit_capsule' if active_fruit_capsule and not own_fruit_capsule else
                               ('current_truss_fruit_fruit' if a in self.fruits and b in self.fruits else
                                'current_truss_fruit_own_capsule' if a in self.capsules or b in self.capsules else
                                'current_truss_fruit_tool') if a in self.fruits or b in self.fruits else
                               'current_truss_tool_own_capsule' if a in self.capsules or b in self.capsules else 'compliant_leaf',
                 method='full convex SAT (face normals and edge cross products) at measured post-fetch poses')
        if not self.eligible(a,b):return row
        if structural_leaf and self.verify_structural_scene_queries:
            leaf,structural=(a,b) if a in self.active_plant_leaves else (b,a)
            prim=self.stage.GetPrimAtPath(structural)
            if prim.IsA(UsdGeom.Mesh) and UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()=='none':
                try:
                    from native_leaf_scene_query import certify_leaf_structural_clear
                    native_data=self.cooked(leaf)
                    proof=certify_leaf_structural_clear(self.stage,leaf,structural,
                        poses[self.body_ids[native_data['body']]],native_data,timestamp)
                    row.update(native_scene_query_certificate=proof,
                        certified_clear=proof['certified_clear'],certified_within_limit=proof['certified_within_limit'],
                        penetration_upper_bound_m=proof['penetration_upper_bound_m'],
                        method=proof['method'],geometry_source='native_scene_query_with_native_cooked_leaf_enclosures')
                    if 'error' in proof:row['error']=proof['error']
                except Exception as error:row['error']=str(error)
                self.records.append(row)
                return row
        try:
            hulls=[];triangle_data=None;capsule_data=None
            for path in (a,b):
                data=self.cooked(path)
                if data.get('kind')=='exact_usd_capsule':
                    if capsule_data is not None:raise ValueError('Two capsules are outside the harvest diagnostic scope')
                    if data['body'] is None:
                        fresh=self.capsule_data(path)
                        if (fresh['radius']!=data['radius'] or not np.array_equal(fresh['endpoints'],data['endpoints'])):
                            raise ValueError('Static structural capsule geometry or transform changed')
                    pose=poses[self.body_ids[data['body']]] if data['body'] else np.array([0,0,0,0,0,0,1.])
                    rot=Rotation.from_quat(pose[3:]).as_matrix()
                    capsule_data=(data['endpoints']@rot.T+pose[:3],data['radius'])
                    row.update(geometry_source='native_cooked_convex_and_exact_usd_capsule',capsule_collider=path,
                               capsule_radius_m=data['radius'])
                    continue
                if data.get('kind')=='authored_static_triangle_fallback':
                    fresh=self.static_triangle_data(path)
                    if fresh['source_geometry_sha256']!=data['source_geometry_sha256']:
                        raise ValueError('Static triangle source geometry or transform changed')
                    if triangle_data is not None:raise ValueError('Two static triangle colliders are outside the leaf diagnostic scope')
                    triangle_data=data
                    row.update(geometry_source=data['kind'],native_cooked_triangle_verified=False,
                               source_geometry_sha256=data['source_geometry_sha256'],
                               geometry_uncertainty=data['uncertainty'])
                    continue
                pose=poses[self.body_ids[data['body']]] if data['body'] else np.array([0,0,0,0,0,0,1.])
                rot=Rotation.from_quat(pose[3:]).as_matrix()
                world=[]
                for hull in data['hulls']:
                    v,n=hull[:2];e=hull[2] if len(hull)>2 else convex_axes(v)[1]
                    world.append((v@rot.T+pose[:3],n@rot.T,e@rot.T))
                hulls.append(world)
            if capsule_data is not None:
                if len(hulls)!=1 or triangle_data is not None:raise ValueError('Missing exact native convex representation')
                ends,radius=capsule_data
                gaps=[convex_capsule_gap(v,n,e,ends,radius) for v,n,e in hulls[0]]
                row.update(hull_capsule_pair_count=len(gaps),minimum_certified_gap_m=min(gaps),
                           penetration_upper_bound_m=max(0.,-min(gaps)),
                           certified_clear=bool(all(g>1e-6 for g in gaps)),
                           certified_within_limit=bool(all(g>=-.0005+.000001 for g in gaps)),
                           method='Exact analytic capsule support versus native cooked convex at measured post-fetch poses')
                self.records.append(row)
                return row
            if triangle_data is not None:
                if len(hulls)!=1:raise ValueError('Missing active leaf convex representation')
                if self.verify_background_component_hulls:
                    try:
                        components=triangle_data.get('enclosing_component_hulls')
                        if components is None:
                            components=enclosing_component_hulls(triangle_data)
                            triangle_data['enclosing_component_hulls']=components
                        component_gaps=[convex_component_hull_gaps(v,n,e,components)
                                        for v,n,e in hulls[0]]
                        flat=[gap for values in component_gaps for gap in values]
                        passed=bool(flat and all(g>=-.0005+.000001 for g in flat))
                        row.update(enclosing_component_hull_certificate=dict(
                            attempted=True,certified_within_limit=passed,
                            hull_component_pair_count=len(flat),component_count=len(components),
                            signed_gap_m=component_gaps,
                            per_pair_penetration_upper_bound_m=[[max(0.,-g) for g in values]
                                                              for values in component_gaps],
                            interpretation='Per convex/component pair bound; not a global union minimum translation',
                            native_cooked_triangle_verified=False))
                        if passed:
                            row.update(hull_component_pair_count=len(flat),
                                       minimum_certified_gap_m=min(flat),
                                       penetration_upper_bound_m=max(0.,-min(flat)),
                                       certified_clear=bool(all(g>1e-6 for g in flat)),
                                       certified_within_limit=True,
                                       method='Full convex SAT against authored connected-component enclosing hulls; conservative per-pair upper bounds with unchanged 0.5mm limit')
                            self.records.append(row)
                            return row
                    except Exception as error:
                        row['enclosing_component_hull_error']=str(error)
                    # Supersets may reject shallow or concave clear geometry.
                    # Preserve the existing exact authored clear-only fallback.
                gaps=[convex_triangle_mesh_gap(v,n,e,triangle_data) for v,n,e in hulls[0]]
                clear=bool(gaps and all(g>1e-6 for g in gaps))
                row.update(hull_triangle_pair_count=len(hulls[0])*len(triangle_data['triangles']),
                           minimum_certified_gap_m=min(gaps),certified_clear=clear,
                           certified_within_limit=clear,penetration_upper_bound_m=None,
                           method='Full convex/triangle SAT with closed-component containment rejection; clear-only authored static leaf fallback')
                self.records.append(row)
                return row
            gaps,aabb_separated=convex_pair_gaps(hulls[0],hulls[1])
            row.update(hull_pair_count=len(gaps),positive_world_aabb_separated_pair_count=aabb_separated,
                       complete_full_sat_pair_count=len(gaps)-aabb_separated,
                       minimum_positive_support_gap_is_not_euclidean_distance=True,
                       method='Complete native convex pair coverage: strictly positive world-axis separation, otherwise full face-normal and edge-cross SAT at measured post-fetch poses',
                       helper_source_sha256=SOURCE_SHA256,minimum_certified_gap_m=min(gaps),
                       penetration_upper_bound_m=max(0.,-min(gaps)),
                       certified_clear=bool(all(g>1e-6 for g in gaps)),
                       certified_within_limit=bool(all(g>=-.0005+.000001 for g in gaps)))
        except Exception as error:row['error']=str(error)
        self.records.append(row)
        return row

    def geometry_metadata(self):
        out={}
        for p,d in self.cache.items():
            if d.get('kind')=='exact_usd_capsule':
                out[p]={key:value.tolist() if isinstance(value,np.ndarray) else value for key,value in d.items()}
            elif d.get('kind')=='authored_static_triangle_fallback':
                out[p]={key:value for key,value in d.items() if key not in
                        ('vertices','faces','triangles','triangle_normals','triangle_edges','components','enclosing_component_hulls')}
                out[p].update(vertices=d['vertices'].tolist(),faces=d['faces'].tolist(),
                              closed_components=sum(closed for _,closed in d['components']))
                if 'enclosing_component_hulls' in d:
                    out[p]['enclosing_component_hulls']=[dict(
                        component_index=c['component_index'],closed=c['closed'],
                        triangle_count=c['triangle_count'],support_point_count=len(c['vertices']),
                        hull_vertex_count=c['hull_vertex_count'],
                        vertices=c['vertices'].tolist(),normals=c['normals'].tolist(),edges=c['edges'].tolist())
                        for c in d['enclosing_component_hulls']]
            else:
                out[p]=dict(body=d['body'],kind=d.get('kind','native_cooked_convex'),
                            hulls=[dict(vertices=h[0].tolist(),normals=h[1].tolist(),
                                        edges=(h[2] if len(h)>2 else convex_axes(h[0])[1]).tolist()) for h in d['hulls']])
        return out

    def policy_metadata(self):
        return dict(helper_source_sha256=SOURCE_SHA256,
                    positive_world_aabb_shortcut='Exact native vertices at current measured pose; only support gap >1um skips full SAT. Ambiguous/overlapping pairs retain full face and edge-cross axes. No geometry or0.5mm change.',
                    verify_current_truss_fruit_contacts=self.verify_fruit_contacts,
                    verify_background_component_hulls=self.verify_background_component_hulls,
                    verify_structural_scene_queries=self.verify_structural_scene_queries,
                    exact_fruit_colliders=sorted(self.fruits),exact_active_leaf_colliders=sorted(self.leaves),
                    exact_declared_passive_leaf_colliders=sorted(self.background_leaves),
                    exact_declared_passive_leaf_inventory=self.background_leaf_inventory,
                    passive_leaf_geometry_scope='Exact separately declared standalone neighbor leaves only; measured native body and cooked hulls required. No permission, collider, force or 0.5mm validity change.',
                    exact_all_active_plant_fruit_colliders=sorted(self.active_plant_fruits),
                    exact_all_active_plant_capsule_colliders=sorted(self.active_plant_capsules),
                    exact_all_active_plant_leaf_colliders=sorted(self.active_plant_leaves),
                    exact_structural_colliders=sorted(self.structural_colliders),
                    exact_structural_capsule_colliders=sorted(self.structural_capsules),
                    exact_own_truss_capsule_colliders=sorted(self.capsules),
                    retarget_history=self.retarget_history,
                    limit_m=.0005,numerical_guard_m=.000001,
                    fruit_scope='Exact current-truss fruit versus another exact current-truss fruit, enabled actuated-tool collider or exact own-truss pedicel/rachis/peduncle capsule; enabled actuated-tool Mesh versus exact own-truss capsule only; geometry validity, not contact permission',
                    active_plant_geometry_scope='Exact manifest active-plant fruit/leaf, fruit/capsule and capsule/leaf only; exact enabled Mesh/Capsule types and measured owners required. Tool/background contact authorization is unchanged.',
                    structural_geometry_scope='Exact active-plant compliant leaf versus enabled House/Environment Mesh/Capsule inventory only. Native convex or uniform static/rigid-owned exact Capsule required; static triangle Mesh remains inconclusive without native cooked triangle evidence. Contact authorization unchanged.',
                    raw_callback_evidence='unchanged; solved geometry certificate is recorded separately',
                    geometry_basis='Native cooked convex hulls; static background leaf fallback is explicitly marked',
                    component_hull_scope='Opt-in authored static leaf enclosing convex component hulls; per-pair upper bounds, not native triangle cooking verification or a global union MTV',
                    default_fruit_verification=False)


def save_imported_source(output):
    path=Path(output)/'runtime_code_sources'/'leaf_contact_geometry.py'
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists() and path.read_bytes()!=SOURCE_BYTES:raise ValueError('Refusing to overwrite imported geometry-reader source evidence')
    if not path.exists():path.write_bytes(SOURCE_BYTES)
    return dict(path=str(path),sha256=SOURCE_SHA256)
