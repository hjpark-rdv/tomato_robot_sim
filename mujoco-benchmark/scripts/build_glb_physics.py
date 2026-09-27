"""GLB Y-only articulated plant migration using existing elastic topology/settings."""
import argparse,copy,hashlib,json,sys
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation as R
from view_glb_truss import read_glb
from convert_model import nums
ROOT=Path(__file__).resolve().parents[2];HOME=ROOT/'mujoco-benchmark'
sys.path.insert(0,str(ROOT/'nvidia-sim/rl'))
from elastic_geometry import tube_centerline,resample_rod

def junction_neighbors(points,radii,root,root_radius,parent_index):
 """Only adjacent rachis capsules containing the authored branch root."""
 delta=points[1:]-points[:-1]
 t=np.clip(np.sum((root-points[:-1])*delta,axis=1)/np.sum(delta*delta,axis=1),0,1)
 distance=np.linalg.norm(points[:-1]+t[:,None]*delta-root,axis=1)
 return [i for i in range(len(delta)) if abs(i-parent_index)==1 and distance[i]<=max(radii[i:i+2])+root_radius]

def attachment_frame(model,data,gid,ped,stem_fraction,y_deg):
 rotation=R.from_euler('x',90,degrees=True).as_matrix()@R.from_euler('y',y_deg,degrees=True).as_matrix()
 stemaxis=data.geom_xmat[gid].reshape(3,3)[:,2];heading=rotation@(ped[3]-ped[0]);normal=heading-stemaxis*np.dot(stemaxis,heading)
 if np.linalg.norm(normal)<1e-9:
  # A branch exactly parallel to the stem has no unique radial side. Use the
  # transformed GLB X axis to choose a side without rotating the branch.
  normal=rotation[:,0]-stemaxis*np.dot(stemaxis,rotation[:,0])
 if np.linalg.norm(normal)<1e-9:raise ValueError('Cannot determine a radial stem attachment side')
 normal/=np.linalg.norm(normal)
 # Anchor the actual proximal cross-section on the stem surface; no extra rotation.
 stem_center=data.geom_xpos[gid]+(2*stem_fraction-1)*model.geom_size[gid,1]*stemaxis
 attachment=stem_center+normal*model.geom_size[gid,0]
 return rotation,attachment

def build(glb,output,y_deg=90,segment=11,target='Tomato_05',remove_fruits=(),fruit_offsets=None,rachis_stiffness_scale=1.,stem_fraction=.5,truss_scale=1.):
 if not np.isfinite(rachis_stiffness_scale) or rachis_stiffness_scale<=0:raise ValueError("Invalid rachis stiffness scale")
 if not np.isfinite(truss_scale) or truss_scale<=0:raise ValueError('truss_scale must be finite and positive')
 remove_fruits=set(remove_fruits)
 if any(i not in range(1,12) for i in remove_fruits):raise ValueError('Removed fruit IDs must be in [1, 11]')
 if int(target.rsplit('_',1)[-1]) in remove_fruits:raise ValueError('Cannot target a removed fruit')
 if not np.isfinite(y_deg) or not 0<=y_deg<=180:raise ValueError('GLB Y rotation must be in [0, 180] degrees')
 if not 0<=segment<16:raise ValueError('Stem segment must be in [0, 15]')
 if not np.isfinite(stem_fraction) or not 0<=stem_fraction<=1:raise ValueError('Stem fraction must be in [0, 1]')
 if target not in [f'Tomato_{i:02d}' for i in range(1,12)]:raise ValueError('Unknown fruit target')
 import mujoco as mj
 source=HOME/'models/robot_plant_optimized.xml';reference=json.loads((HOME/'assets/reference/reference.json').read_text());old=mj.MjModel.from_xml_path(str(source));od=mj.MjData(old);mj.mj_forward(old,od)
 meshes={name:(v.copy(),f,c) for name,v,f,c in read_glb(glb,named=True)}
 fruit_offsets=fruit_offsets or {}
 for fruit_id,offset in fruit_offsets.items():
  offset=np.asarray(offset,dtype=float)
  if fruit_id not in range(1,12) or offset.shape!=(3,) or not np.isfinite(offset).all():raise ValueError('Invalid fruit offset')
  suffix=f'{fruit_id:02d}';prox='Pedicel_proximal_'+suffix
  centerline,_=tube_centerline(meshes[prox][0],12)
  lengths=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(centerline,axis=0),axis=1))]
  for meshname,(v,f,c) in meshes.items():
   tokens=meshname.split('_')
   calyx=meshname.startswith('Calyx_') and int(tokens[2] if tokens[1]=='Hub' else tokens[1])==fruit_id
   if meshname.startswith(('Fruit_'+suffix,'Pedicel_distal_'+suffix)) or calyx:v+=offset
   elif meshname.startswith(prox):
    # Keep the branch root attached; smoothly shift towards the distal end.
    nearest_ring=np.argmin(np.linalg.norm(v[:,None,:]-centerline[None,:,:],axis=2),axis=1)
    v+=offset*(lengths[nearest_ring]/lengths[-1])[:,None]

 # Scale only authored truss geometry about its attachment, including rod radii.
 pivot=tube_centerline(meshes['Truss_01_Peduncle'][0],14)[0][0].copy()
 for v,_,_ in meshes.values():v[:]=pivot+(v-pivot)*truss_scale
 tree=E.parse(source);root=tree.getroot();root.set('model','GLB_ELASTIC_ROBOT_PILOT');root.find('compiler').set('inertiafromgeom','auto');root.find('option').set('timestep',str(1/240))
 stem=root.find("worldbody/body[@name='STEM_MainStem_00']")
 for node in list(stem.iter('body')):
  for child in list(node.findall('body')):
   if child.get('name','').startswith('TRUSS_'):node.remove(child)
 contact=root.find('contact');retained={n.get('name') for n in root.iter('body')}
 for item in list(contact):
  if item.get('body1') not in retained or item.get('body2') not in retained:contact.remove(item)
 for tag in ['equality','keyframe']:
  for item in root.findall(tag):root.remove(item)
 assets=root.find('asset');positions={};nodes={n.get('name'):n for n in root.iter('body')};chains={};joints={j['child'].split('/')[-1]:j for j in reference['joints']};bodies={b['name']:b for b in reference['bodies']};exclusions=set();rod_bodies=[]
 name=f'STEM_MainStem_{segment:02d}';parent=nodes[name];bid=old.body(name).id;gid=next(g for g in range(old.ngeom) if old.geom_bodyid[g]==bid and old.geom_type[g]==mj.mjtGeom.mjGEOM_CAPSULE)
 ped,rped=tube_centerline(meshes['Truss_01_Peduncle'][0],14);ped0=ped[0].copy()
 rotation,attachment=attachment_frame(old,od,gid,ped,stem_fraction,y_deg)
 tf=lambda v:(np.asarray(v)-ped0)@rotation.T+attachment
 for n in nodes:positions[n]=od.xpos[old.body(n).id].copy()
 def exclude(a,b):
  pair=tuple(sorted([a,b]))
  if pair not in exclusions:E.SubElement(contact,'exclude',body1=a,body2=b);exclusions.add(pair)
 def chain(meshname,sides,count,names,parent_name):
  points,radii=tube_centerline(meshes[meshname][0],sides);points,radii=resample_rod(tf(points),radii,count);result=[]
  for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
   name=names[i];owner=parent_name if i==0 else names[i-1];parentpos=positions[owner]
   # All newly constructed frames align to world; only original stem parent rotates.
   pr=od.xmat[old.body(owner).id].reshape(3,3) if owner.startswith('STEM_') else np.eye(3)
   q=R.from_matrix(pr.T).as_quat();node=E.SubElement(nodes[owner],'body',name=name,pos=nums(pr.T@(a-parentpos)),quat=nums(q[[3,0,1,2]]));nodes[name]=node;positions[name]=a
   spec=joints[name]
   for j,axis in enumerate(np.eye(3)):
    params=dict(spec['axes'][j])
    if meshname=='Rachis':params['k']*=rachis_stiffness_scale;params['d']*=np.sqrt(rachis_stiffness_scale)
    E.SubElement(node,'joint',name=name+'_'+str(j),type='hinge',axis=nums(axis),pos='0 0 0',stiffness=str(params['k']),damping=str(params['d']),armature=str(reference['armature']),limited='true',range=nums(np.deg2rad(params['limit_deg'])))
   E.SubElement(node,'geom',name='glb_col_'+name,type='capsule',fromto=nums([np.zeros(3),b-a]),size=str(float(max(radii[i:i+2]))),mass=str(bodies[name]['mass']*truss_scale**3),contype='1',conaffinity='15',group='3',rgba='0.15 0.4 0.05 0')
   exclude(owner,name);rod_bodies.append(name);result.append(name)
  chains[meshname]=(points,result);return result
 def nearest(chain_name,point):
  points,names=chains[chain_name];delta=points[1:]-points[:-1];f=np.clip(np.sum((point-points[:-1])*delta,axis=1)/np.sum(delta*delta,axis=1),0,1);return names[int(np.argmin(np.linalg.norm(points[:-1]+delta*f[:,None]-point,axis=1)))]
 chain('Truss_01_Peduncle',14,5,[f'TRUSS_Truss_01_Peduncle_{i:02d}' for i in range(5)],name)
 chain('Rachis',14,14,[f'TRUSS_Rachis_{i:02d}' for i in range(14)],'TRUSS_Truss_01_Peduncle_04')
 fruit_specs=[];junction_filters=[]
 for i in range(1,12):
  suffix=f'{i:02d}';prox='Pedicel_proximal_'+suffix;dist='Pedicel_distal_'+suffix
  proximal,_=tube_centerline(meshes[prox][0],12);owner=nearest('Rachis',tf(proximal[0]));names=[f'TRUSS_{prox}_{j:02d}' for j in range(3)]
  chain(prox,12,3,names,owner);anchor=f'Attachment_{i-1:02d}';chain(dist,12,1,[anchor],names[-1])
  # A bifurcation can straddle a rachis segmentation boundary. Exclude only
  # its first proximal capsule and immediate neighboring rachis capsules
  # containing that root, never fruit contacts or the entire branch.
  rp,rr=tube_centerline(meshes['Rachis'][0],14);rp,rr=resample_rod(tf(rp),rr,14)
  _,pr=tube_centerline(meshes[prox][0],12);rachis_names=chains['Rachis'][1]
  for j in junction_neighbors(rp,rr,tf(proximal[0]),pr[0],rachis_names.index(owner)):
   exclude(rachis_names[j],names[0]);junction_filters.append(dict(rachis=rachis_names[j],proximal=names[0],reason='adjacent capsule contains authored branch root'))
  if i in remove_fruits:continue
  v=meshes['Fruit_'+suffix][0];center=tf((v.max(0)+v.min(0))/2);fruit='Tomato_'+suffix;node=E.SubElement(nodes[anchor],'body',name=fruit,pos=nums(center-positions[anchor]));nodes[fruit]=node;positions[fruit]=center
  vv=tf(v)-center;meshname='glb_fruit_collision_'+suffix
  E.SubElement(assets,'mesh',name=meshname,vertex=nums(vv),face=nums(meshes['Fruit_'+suffix][1]))
  E.SubElement(node,'geom',name='glb_col_'+fruit,type='mesh',mesh=meshname,mass=str(bodies[fruit]['mass']*truss_scale**3),contype='1',conaffinity='15',group='3',rgba='0 0 0 0')
  exclude(anchor,fruit)
  # These adjacent proximal pieces are mechanically joined to this fruit's pedicel.
  # Only direct fruit-parent excluded here; nonadjacent contacts remain active.
  dp,_=tube_centerline(meshes[dist][0],12);neck=tf(dp[-1]);axis=neck-center;axis/=np.linalg.norm(axis)
  fruit_specs.append(dict(name=fruit,path='/GLB/'+fruit,anchor='/GLB/'+anchor,pose=[*center.tolist(),1,0,0,0],center=[0,0,0],neck=(neck-center).tolist(),axis=axis.tolist(),radius=float(np.max(np.linalg.norm(vv,axis=1))),force=3.,torque=.08))
 # Preserve every original visual triangle. Rod triangle groups follow corresponding rigid segments.
 visual_count=0
 for meshname,(v,faces,color) in meshes.items():
  if meshname.startswith('Fruit_') and int(meshname.split('_')[1]) in remove_fruits:continue
  if meshname.startswith('Calyx_'):
   tokens=meshname.split('_');fruit_id=int(tokens[2] if tokens[1]=='Hub' else tokens[1])
   if fruit_id in remove_fruits:continue
  worldv=tf(v);base=meshname.replace('_Trichomes','')
  if base in chains:
   points,names=chains[base];mid=(points[:-1]+points[1:])/2;tri=worldv[faces].mean(1);choice=np.argmin(np.linalg.norm(tri[:,None,:]-mid[None,:,:],axis=2),axis=1);groups=[(owner,faces[choice==j]) for j,owner in enumerate(names)]
  else:
   if meshname.startswith('Fruit_'):owner='Tomato_'+meshname.split('_')[1]
   elif meshname.startswith('Calyx_'):
    tokens=meshname.split('_');suffix=tokens[2] if tokens[1]=='Hub' else tokens[1];owner='Tomato_'+suffix if 'Tomato_'+suffix in nodes else nearest('Rachis',worldv.mean(0))
   else:owner=nearest('Rachis',worldv.mean(0))
   groups=[(owner,faces)]
  for owner,ff in groups:
   if not len(ff):continue
   ids,inverse=np.unique(ff,return_inverse=True);mesh='glb_vis_mesh_'+str(visual_count);gn='glb_vis_'+str(visual_count);visual_count+=1
   E.SubElement(assets,'mesh',name=mesh,vertex=nums(worldv[ids]-positions[owner]),face=nums(inverse.reshape(-1,3)))
   E.SubElement(nodes[owner],'geom',name=gn,type='mesh',mesh=mesh,contype='0',conaffinity='0',density='0',group='2',rgba=nums(color))
 from visual_colors import apply_stem_palette
 apply_stem_palette(tree.getroot())
 output.mkdir(parents=True,exist_ok=False);tree.write(output/'model.xml',encoding='unicode');m=mj.MjModel.from_xml_path(str(output/'model.xml'));d=mj.MjData(m);mj.mj_forward(m,d);mj.mj_saveModel(m,str(output/'model.mjb'))
 # Reference used by the existing RobotEngine: all plant bodies and real collision identities.
 ref=copy.deepcopy(reference);ref['bodies']=[];ref['shapes']=[];ref['visuals']=[];ref['joints']=[];ref['filters']=[];ref['fruit_specs']=fruit_specs
 plant_names=[m.body(i).name for i in range(m.nbody) if m.body(i).name.startswith(('STEM_','TRUSS_','Attachment_','Tomato_'))]
 for name in plant_names:
  i=m.body(name).id;ref['bodies'].append(dict(name=name,path='/GLB/'+name,pose=[*d.xpos[i].tolist(),*d.xquat[i].tolist()],mass=float(m.body_mass[i])))
 for i in range(m.ngeom):
  name=m.geom(i).name;body=m.body(m.geom_bodyid[i]).name
  if body not in plant_names and body!='Hook':continue
  item=dict(name=name,path='/GLB/'+body+'/'+name,body=ref['tool_path'] if body=='Hook' else '/GLB/'+body)
  (ref['shapes'] if m.geom_contype[i] or m.geom_conaffinity[i] else ref['visuals']).append(item)
 ref['schema']='glb_physics_reference_v1';ref['source_glb']=str(glb.resolve());ref['target']=target
 (output/'reference.json').write_text(json.dumps(ref,indent=2))
 meta=dict(source_glb=str(glb.resolve()),source_sha256=hashlib.sha256(glb.read_bytes()).hexdigest(),baseline_model=str(source),y_deg=y_deg,glb_to_world_rotation=rotation.tolist(),attachment_world_m=attachment.tolist(),scale=truss_scale,body_count=m.nbody,dof_count=m.nv,rod_bodies=rod_bodies,visual_triangle_count=sum(len(f) for _,f,_ in meshes.values()),armature=reference['armature'],physics_ready=False,scope='elastic GLB migration pilot; validation pending',limitations=['Spring damping and armature retained; truss masses scale cubically to preserve density; response requires validation','Visual rod triangles rigidly follow nearest segments; bending can expose seams','Fruit convex hulls; rods capsule chains; trichomes visual only','Break disabled as in optimized baseline; no detachment','Existing main-stem appendage collision policy preserved'])
 meta['truss_mass_scale']=truss_scale**3
 meta['rachis_stiffness_scale']=rachis_stiffness_scale
 meta['stem_segment']=segment
 meta['stem_fraction']=stem_fraction
 meta['rachis_damping_scale']=float(np.sqrt(rachis_stiffness_scale))
 meta['junction_filters']=junction_filters
 meta['removed_fruit_ids']=sorted(remove_fruits)
 meta['fruit_offsets_glb_m']={str(k):list(v) for k,v in fruit_offsets.items()}
 meta['removal_scope']='fruit body, collision, mass, calyx visuals and target spec; pedicel retained'
 meta['visual_triangle_count']=sum(len(node.get('face','').split())//3 for node in assets.findall('mesh') if node.get('name','').startswith('glb_vis_mesh_'))
 (output/'build.json').write_text(json.dumps(meta,indent=2));print('GLB physics model',output,m.nv,m.ngeom,flush=True);return output
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--glb',type=Path,default=ROOT/'nvidia-sim/env_usd/tomato_rotate_glb/tomato_master_v10_cluster_curve_cyan.glb');p.add_argument('--output',type=Path,required=True);p.add_argument('--truss-scale',type=float,default=.5);p.add_argument('--y-deg',type=float,default=90);p.add_argument('--segment',type=int,default=11);p.add_argument('--stem-fraction',type=float,default=.5);p.add_argument('--remove-fruit',type=int,action='append',default=[]);p.add_argument('--rachis-stiffness-scale',type=float,default=1.);p.add_argument('--fruit-offset',nargs=4,action='append',default=[],metavar=('ID','DX','DY','DZ'),help='Fruit ID and offset in GLB meters; keep proximal root fixed');a=p.parse_args();build(a.glb,a.output,a.y_deg,a.segment,remove_fruits=a.remove_fruit,fruit_offsets={int(v[0]):[float(x) for x in v[1:]] for v in a.fruit_offset},rachis_stiffness_scale=a.rachis_stiffness_scale,stem_fraction=a.stem_fraction,truss_scale=a.truss_scale)
