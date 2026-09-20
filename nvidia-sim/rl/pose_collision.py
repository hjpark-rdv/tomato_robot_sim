"""FCL self-collision screening of the current USD collision shapes using URDF FK.

Uses the existing SRDF exclusions and fixed-link groups. Plant contact is left
to PhysX. This is a discrete path check, not a continuous collision certificate.
"""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

SIM_DIR=Path(__file__).resolve().parents[1]


class SelfCollisionCheck:
    def __init__(self,env,kin):
        import fcl
        import trimesh
        from pxr import Usd,UsdGeom,UsdPhysics
        self.fcl=fcl;self.world=kin.world
        urdf=ET.parse(SIM_DIR/'robot_usd/rb5_farmily.urdf').getroot()
        links={x.get('name') for x in urdf.findall('link')}
        self.joints=[]; parent_group={x:x for x in links}
        def group(x):
            while parent_group[x]!=x: x=parent_group[x]
            return x
        for j in urdf.findall('joint'):
            p=j.find('parent').get('link');c=j.find('child').get('link');origin=j.find('origin')
            t=np.eye(4)
            if origin is not None:
                t[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            axis=j.find('axis');axis=np.fromstring(axis.get('xyz'),sep=' ') if axis is not None else np.zeros(3)
            name=j.get('name');idx=env.robot.joint_names.index(name) if name in env.robot.joint_names else None
            self.joints.append((p,c,t,axis,j.get('type'),idx))
            if j.get('type')=='fixed': parent_group[group(c)]=group(p)
        self.root=next(iter(links-{j[1] for j in self.joints}))
        excluded=set()
        srdf=ET.parse(SIM_DIR.parent/'ros2_ws/src/rbpodo_ros2/rbpodo_moveit_config/config/rbpodo.srdf').getroot()
        for pair in srdf.findall('disable_collisions'):
            a,b=pair.get('link1'),pair.get('link2')
            if a in links and b in links: excluded.add(frozenset((a,b)))
        cache=UsdGeom.XformCache();self.shapes=[]
        root=env.stage.GetPrimAtPath('/World/envs/env_0/Robot')
        adjacent=set();joint_filters=[]
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdPhysics.Joint): continue
            joint=UsdPhysics.Joint(prim)
            a=joint.GetBody0Rel().GetTargets();b=joint.GetBody1Rel().GetTargets()
            if not a or not b or joint.GetCollisionEnabledAttr().Get() is not False: continue
            la,lb=env.stage.GetPrimAtPath(a[0]).GetName(),env.stage.GetPrimAtPath(b[0]).GetName()
            if la in links and lb in links:
                adjacent.add(frozenset((group(la),group(lb))))
                joint_filters.append(dict(joint=str(prim.GetPath()),body0=str(a[0]),body1=str(b[0]),collision_enabled=False))
        for prim in Usd.PrimRange(root):
            if not prim.HasAPI(UsdPhysics.CollisionAPI): continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False: continue
            ancestor=prim
            while ancestor and ancestor.GetName() not in links: ancestor=ancestor.GetParent()
            if not ancestor: raise ValueError('Cannot map collision shape to URDF link: '+str(prim.GetPath()))
            link=ancestor.GetName()
            relative=np.asarray(cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(ancestor).GetInverse()).T
            local=np.eye(4)
            if prim.IsA(UsdGeom.Mesh):
                mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get())
                v=v@relative[:3,:3].T+relative[:3,3]
                hull=trimesh.convex.convex_hull(v)
                faces=np.column_stack((np.full(len(hull.faces),3),hull.faces)).reshape(-1)
                geometry=fcl.Convex(hull.vertices,len(hull.faces),faces)
                bounds=hull.bounds
            elif prim.IsA(UsdGeom.Cube):
                half=float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2
                v=np.array([[x,y,z] for x in [-half,half] for y in [-half,half] for z in [-half,half]])
                v=v@relative[:3,:3].T+relative[:3,3];hull=trimesh.convex.convex_hull(v)
                geometry=fcl.Convex(hull.vertices,len(hull.faces),np.column_stack((np.full(len(hull.faces),3),hull.faces)).reshape(-1));bounds=hull.bounds
            elif prim.IsA(UsdGeom.Capsule):
                cap=UsdGeom.Capsule(prim);scale=np.linalg.norm(relative[:3,:3],axis=0)
                if not np.allclose(scale,scale[0]): raise ValueError('Nonuniform capsule scale')
                radius=float(cap.GetRadiusAttr().Get())*scale[0];height=float(cap.GetHeightAttr().Get())*scale[0]
                geometry=fcl.Capsule(radius,height);local=relative.copy();local[:3,:3]/=scale
                axis=str(cap.GetAxisAttr().Get())
                align=Rotation.from_euler('y',90,degrees=True).as_matrix() if axis=='X' else Rotation.from_euler('x',-90,degrees=True).as_matrix() if axis=='Y' else np.eye(3)
                local[:3,:3]=local[:3,:3]@align
                extent=np.abs(local[:3,2])*height/2+radius;bounds=np.array([local[:3,3]-extent,local[:3,3]+extent])
            else: raise ValueError('Unsupported robot collider: '+str(prim.GetPath()))
            corners=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
            self.shapes.append(dict(path=str(prim.GetPath()),link=link,group=group(link),local=local,
                object=fcl.CollisionObject(geometry),corners=corners))
        self.pairs=np.array([(i,j) for i,a in enumerate(self.shapes) for j,b in enumerate(self.shapes[:i])
            if a['group']!=b['group'] and frozenset((a['group'],b['group'])) not in adjacent
            and frozenset((a['link'],b['link'])) not in excluded],dtype=int)
        if not len(self.shapes) or not len(self.pairs): raise ValueError('Empty self-collision model')
        self.manifest=dict(shape_count=len(self.shapes),checked_pair_count=len(self.pairs),
            shapes=[dict(path=s['path'],link=s['link']) for s in self.shapes],
            exclusions='exact SRDF link pairs, same fixed-link rigid body, and USD joints with collisionEnabled=false',
            native_joint_collision_filters=joint_filters,
            method='FCL convex hulls/cubes/capsules from active USD colliders; discrete joint path samples')
        fk=self.transforms(env.robot.data.joint_pos[0].cpu().numpy());errors={}
        for prim in Usd.PrimRange(root):
            if prim.GetName() in {s['link'] for s in self.shapes}:
                tf=np.asarray(cache.GetLocalToWorldTransform(prim)).T
                if prim.GetName() not in fk: continue
                errors[str(prim.GetPath())]=float(np.max(np.abs(tf-fk[prim.GetName()])))
        self.manifest['initial_fk_usd_transform_max_abs_errors']=errors
        if not errors or max(errors.values())>.001:
            raise ValueError('URDF self-collision FK differs from initial USD transforms: '+str(errors))

    def transforms(self,q):
        poses={self.root:self.world.copy()};pending=list(self.joints)
        while pending:
            remaining=[]
            for p,c,origin,axis,kind,idx in pending:
                if p not in poses: remaining.append((p,c,origin,axis,kind,idx));continue
                t=poses[p]@origin
                if idx is not None:
                    if kind=='prismatic': t[:3,3]+=t[:3,:3]@(axis*q[idx])
                    else: t[:3,:3]=t[:3,:3]@Rotation.from_rotvec(axis*q[idx]).as_matrix()
                poses[c]=t
            if len(remaining)==len(pending): raise ValueError('Disconnected URDF')
            pending=remaining
        return poses

    def check(self,q):
        poses=self.transforms(q)
        corners=np.asarray([s['corners']@poses[s['link']][:3,:3].T+poses[s['link']][:3,3] for s in self.shapes])
        low,high=corners.min(axis=1),corners.max(axis=1);a,b=self.pairs.T
        mask=((low[a]<=high[b]) & (low[b]<=high[a])).all(axis=1)
        updated=set()
        for i,j in self.pairs[mask]:
            for k in (i,j):
                if k in updated: continue
                shape=self.shapes[k];tf=poses[shape['link']]@shape['local']
                shape['object'].setTransform(self.fcl.Transform(tf[:3,:3],tf[:3,3]));updated.add(k)
            if self.fcl.collide(self.shapes[i]['object'],self.shapes[j]['object'],self.fcl.CollisionRequest(),self.fcl.CollisionResult()):
                return [self.shapes[i]['path'],self.shapes[j]['path']]
        return None
