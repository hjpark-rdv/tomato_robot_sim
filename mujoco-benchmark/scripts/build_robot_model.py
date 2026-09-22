"""Attach the actual USD robot articulation to an existing plant MJCF."""
import argparse,json,hashlib
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import Usd,UsdGeom,UsdPhysics
from convert_model import nums,quat,HOME

def build(plant,output):
 ref=json.loads((HOME/'assets/reference/reference.json').read_text())
 source=HOME/'assets/reference/robot_export_only.usda';stage=Usd.Stage.Open(str(source));cache=UsdGeom.XformCache()
 tf=lambda p:np.asarray(cache.GetLocalToWorldTransform(p)).T
 prims=list(Usd.PrimRange(stage.GetDefaultPrim(),Usd.TraverseInstanceProxies()))
 bodies={str(p.GetPath()):p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)}
 joints={str(UsdPhysics.Joint(p).GetBody1Rel().GetTargets()[0]):p for p in prims if p.IsA(UsdPhysics.Joint) and UsdPhysics.Joint(p).GetBody1Rel().GetTargets()}
 tree=E.parse(plant);root=tree.getroot();root.set('model','farmily_robot_'+Path(plant).stem)
 world=root.find('worldbody');assets=root.find('asset');contact=root.find('contact');act=E.SubElement(root,'actuator')
 hook=world.find("body[@name='Hook']");world.remove(hook);hook.attrib.pop('mocap');nodes={};record=[]
 def body(path):
  if path in nodes:return nodes[path]
  p=bodies[path];j=UsdPhysics.Joint(joints[path]);parents=j.GetBody0Rel().GetTargets();parent=str(parents[0]) if parents else None
  rel=np.linalg.inv(tf(bodies[parent]))@tf(p) if parent else np.asarray(ref['robot_fk']['world'])@tf(p)
  n=E.SubElement(body(parent) if parent else world,'body',name='Robot_'+p.GetName(),pos=nums(rel[:3,3]),quat=nums(quat(R.from_matrix(rel[:3,:3]))),gravcomp='1')
  ma=UsdPhysics.MassAPI(p);q=ma.GetPrincipalAxesAttr().Get()
  E.SubElement(n,'inertial',mass=str(ma.GetMassAttr().Get()),pos=nums(ma.GetCenterOfMassAttr().Get()),quat=nums([q.GetReal(),*q.GetImaginary()]),diaginertia=nums(ma.GetDiagonalInertiaAttr().Get()))
  jp=j.GetPrim()
  if parent:
   prismatic=jp.IsA(UsdPhysics.PrismaticJoint);typ=UsdPhysics.PrismaticJoint(jp) if prismatic else UsdPhysics.RevoluteJoint(jp)
   q=j.GetLocalRot1Attr().Get();rot=R.from_quat([*q.GetImaginary(),q.GetReal()]);axis=rot.apply(np.eye(3)['XYZ'.index(typ.GetAxisAttr().Get())])
   limits=np.array([typ.GetLowerLimitAttr().Get(),typ.GetUpperLimitAttr().Get()]);limits=limits if prismatic else np.deg2rad(limits)
   name=jp.GetName();E.SubElement(n,'joint',name=name,type='slide' if prismatic else 'hinge',pos=nums(j.GetLocalPos1Attr().Get()),axis=nums(axis),limited='true',range=nums(limits))
   E.SubElement(act,'position',name='drive_'+name,joint=name,kp='50000' if prismatic else '800',kv='2500' if prismatic else '40',forcerange='-10000 10000' if prismatic else '-80 80')
   record.append(dict(name=name,body=n.get('name'),limits=limits.tolist()))
  nodes[path]=n;return n
 for path in bodies:body(path)
 tool=stage.GetPrimAtPath('/Robot/link6/tcp/tomato_gripper');local=np.linalg.inv(tf(bodies['/Robot/link6']))@tf(tool)
 hook.set('pos',nums(local[:3,3]));hook.set('quat',nums(quat(R.from_matrix(local[:3,:3]))));nodes['/Robot/link6'].append(hook)
 # The source URDF uses ver.6 in millimetres with no visual-origin offset.
 # Keep the aperture-preserving collision proxies, but render the original CAD.
 stl=HOME.parent/'ros2_ws/src/rbpodo_ros2/rbpodo_description/meshes/tomato_gripper/assy_gripper_ver_6.stl'
 for geom in hook.findall('geom'):
  geom.set('rgba','0.65 0.68 0.72 0')
  geom.set('group','3')
 # Binary STL exceeds MuJoCo's direct STL decoder face limit. Preserve
 # every triangle via inline mesh data; only merge identical vertices.
 raw=stl.read_bytes();n=int.from_bytes(raw[80:84],'little')
 assert len(raw)==84+50*n, 'Expected binary STL'
 triangles=np.frombuffer(raw,offset=84,count=n,dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]))['vertices']
 vertices,inverse=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
 E.SubElement(assets,'mesh',name='gripper_original_stl',vertex=nums(vertices*0.001),face=nums(inverse.reshape(-1,3)))
 E.SubElement(hook,'geom',name='gripper_original_visual',type='mesh',mesh='gripper_original_stl',contype='0',conaffinity='0',density='0',group='2',rgba='0.65 0.68 0.72 1')
 counts={'collision':0,'visual':0};manifest=[]
 for p in prims:
  path=str(p.GetPath())
  if path.startswith(str(tool.GetPath())+'/'):continue # existing split ring geoms, not a solid convex hull
  if not isinstance(p,Usd.Prim):continue
  collision=p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
  if not (p.IsA(UsdGeom.Mesh) or p.IsA(UsdGeom.Cube)) or (not collision and '/visuals/' not in path):continue
  owner=p
  while owner and str(owner.GetPath()) not in bodies:owner=owner.GetParent()
  if not owner:continue
  rel=np.linalg.inv(tf(owner))@tf(p);name='robot_geom_'+str(len(manifest))
  # Robot self collision is disabled in the source articulation. Robot-plant stays enabled.
  a=dict(name=name,contype='8' if collision else '0',conaffinity='3' if collision else '0',density='0',group='0' if collision else '2',rgba='.65 .68 .72 1')
  if p.IsA(UsdGeom.Mesh):
   mesh=UsdGeom.Mesh(p);v=np.array(mesh.GetPointsAttr().Get());v=v@rel[:3,:3].T+rel[:3,3];ids=mesh.GetFaceVertexIndicesAttr().Get();faces=[];off=0
   for count in mesh.GetFaceVertexCountsAttr().Get():faces.extend([[ids[off],ids[off+k],ids[off+k+1]] for k in range(1,count-1)]);off+=count
   E.SubElement(assets,'mesh',name=name+'_mesh',vertex=nums(v),face=nums(faces));a.update(type='mesh',mesh=name+'_mesh')
  else:
   scale=np.linalg.norm(rel[:3,:3],axis=0);rot=rel[:3,:3]/scale
   a.update(type='box',pos=nums(rel[:3,3]),quat=nums(quat(R.from_matrix(rot))),size=nums(scale*UsdGeom.Cube(p).GetSizeAttr().Get()/2))
  E.SubElement(nodes[str(owner.GetPath())],'geom',**a);counts['collision' if collision else 'visual']+=1;manifest.append({'path':path,'geom':name,'collision':bool(collision)})
 # Source disables intra-articulation contacts, including attached ring against arm.
 for node in nodes.values():E.SubElement(contact,'exclude',body1=node.get('name'),body2='Hook')
 E.SubElement(hook,'site',name='ring_center',pos=nums(__import__('suite').RING),size='.002',rgba='1 1 0 1')
 target=np.array(ref['fruit_specs'][4]['pose'][:3]);eye=target+np.array([.9,1.4,.65]);z=(eye-target)/np.linalg.norm(eye-target);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
 E.SubElement(world,'camera',name='robot_overview',pos=nums(eye),xyaxes=nums([x,y]),fovy='55')
 E.indent(root);output=Path(output);tree.write(output,encoding='unicode')
 import mujoco as mj
 m=mj.MjModel.from_xml_path(str(output));mj.mj_saveModel(m,str(output.with_suffix('.mjb')))
 info=dict(gripper_visual_stl=str(stl),gripper_visual_sha256=hashlib.sha256(stl.read_bytes()).hexdigest(),gripper_collision='Existing Isaac split ring/rail and proximal hull proxies; unchanged',source_usd=str(source),source_plant=str(plant),source_plant_sha256=hashlib.sha256(Path(plant).read_bytes()).hexdigest(),model_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),joints=record,geometry=manifest,counts=counts,nq=m.nq,nv=m.nv,nbody=m.nbody,ngeom=m.ngeom,nu=m.nu,robot_self_collision=False,robot_gravity_compensation=True,break_enabled=False)
 output.with_suffix('.json').write_text(json.dumps(info,indent=2));print('[로봇 모델 생성]',output,counts,m.nv,flush=True)
 return info
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--plant',type=Path,default=HOME/'models/plant_mujoco_optimized.xml');p.add_argument('--output',type=Path,default=HOME/'models/robot_plant_optimized.xml');a=p.parse_args();build(a.plant,a.output)
