"""Full geometry/topology transfer. No automatic decimation or collision removal."""
import argparse,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R
from visual_colors import shape_color

HOME=Path(__file__).resolve().parents[1]
def nums(x):return ' '.join(f'{v:.12g}' for v in np.asarray(x).reshape(-1))
def quat(r):return r.as_quat()[[3,0,1,2]]
def rotation(p):return R.from_quat(np.asarray(p)[[4,5,6,3]])
def build(reference,output):
    d=json.loads(Path(reference).read_text());output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    root=ET.Element('mujoco',model='plant_original_equivalent')
    ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false',fusestatic='false')
    opt=ET.SubElement(root,'option',timestep=nums([1/120]),gravity=nums(d['gravity']),integrator='implicitfast',solver='Newton',iterations='64',tolerance='1e-8',jacobian='sparse',cone='elliptic')
    ET.SubElement(opt,'flag',filterparent='disable',autoreset='disable')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='960',offheight='720')
    ET.SubElement(visual,'map',znear='.0005',zfar='10')
    default=ET.SubElement(root,'default')
    ET.SubElement(default,'geom',friction=nums([d['friction']['dynamic'],0,0]),condim='3',margin='.0005',gap='.0005',solref='.002 1',solimp='.99 .999 .001',density='0')
    assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody');contact=ET.SubElement(root,'contact');eq=ET.SubElement(root,'equality')
    ET.SubElement(world,'light',pos='0 0 3',dir='0 0 -1',diffuse='.9 .9 .9')
    bodies={b['path']:b for b in d['bodies']};joints={j['child']:j for j in d['joints']};elements={};joint_names={}
    def body(path):
        if path in elements:return elements[path]
        b=bodies[path];j=joints[path];parent=j['parent'];p=np.asarray(b['pose']);r=rotation(p)
        if j['exclude_articulation']:
            node=ET.SubElement(world,'body',name=b['name'],pos=nums(p[:3]),quat=nums(p[3:]));ET.SubElement(node,'freejoint',name=b['name']+'_free')
        else:
            if parent:
                pb=bodies[parent];pp=np.asarray(pb['pose']);pr=rotation(pp);parentnode=body(parent)
                pos=pr.inv().apply(p[:3]-pp[:3]);q=quat(pr.inv()*r)
            else:parentnode=world;pos=p[:3];q=p[3:]
            node=ET.SubElement(parentnode,'body',name=b['name'],pos=nums(pos),quat=nums(q))
            jr=R.from_quat(np.array(j['rot1'])[[1,2,3,0]])
            for axis,a in enumerate(j['axes']):
                name=b['name']+'_'+str(axis);joint_names[name]=dict(source_joint=j['path'],axis=axis)
                ET.SubElement(node,'joint',name=name,type='hinge',pos=nums(j['pos1']),axis=nums(jr.apply(np.eye(3)[axis])),
                    stiffness=str(a['k']),damping=str(a['d']),armature=str(d['armature']),limited='true',range=nums(np.deg2rad(a['limit_deg'])))
        com=np.asarray(b['com_pose_xyzw']);inertia=np.asarray(b['inertia']).reshape(3,3)
        ET.SubElement(node,'inertial',pos=nums(com[:3]),quat=nums(com[[6,3,4,5]]),mass=str(b['mass']),diaginertia=nums(np.diag(inertia)))
        elements[path]=node
        return node
    for path in bodies:body(path)
    elements[d['tool_path']]=ET.SubElement(world,'body',name='Hook',mocap='true',pos='0 0 10')
    for shape in d['shapes']+d['visuals']:
        is_visual=shape in d['visuals'];isleaf='/Appendage_' in shape['path'];ishook=shape['body']==d['tool_path']
        attrs=dict(name=shape['name'],type=shape['type'],group='2' if is_visual else '0',
            contype='0' if is_visual else '2' if isleaf else '4' if ishook else '1',
            conaffinity='0' if is_visual else '5' if isleaf else '3' if ishook else '7',
            rgba=nums([*( [.85,.85,.85] if ishook else shape_color(shape)),1]))
        if shape['type']=='capsule':attrs.update(fromto=nums(shape['ends']),size=nums([shape['radius']]))
        elif shape['type']=='sphere':attrs.update(pos=nums(shape['center']),size=nums([shape['radius']]))
        else:
            # Keep all vertices for collision hull cooking; visual and collision
            # are separate geoms, so no hull ever fills the ring aperture.
            ET.SubElement(assets,'mesh',name=shape['name']+'_mesh',vertex=nums(shape['vertices']),face=nums(shape['faces']))
            attrs['mesh']=shape['name']+'_mesh'
        ET.SubElement(elements[shape['body']],'geom',**attrs)
    exclusions=set()
    for j in d['joints']:
        if j['parent']:exclusions.add(tuple(sorted((j['parent'],j['child']))))
        if j['exclude_articulation']:
            ET.SubElement(eq,'weld',name=bodies[j['child']]['name']+'_attachment',body1=bodies[j['parent']]['name'],body2=bodies[j['child']]['name'],solref='.002 1',solimp='.999 .9999 .001')
    unmatched=[]
    for a,b in d['filters']:
        if '/Appendage_' in a and '/Appendage_' in b:continue # exactly reproduced by masks
        if a in bodies and b in bodies:exclusions.add(tuple(sorted((a,b))))
        elif '/Robot' not in a and '/Robot' not in b and '/collisions' not in a:unmatched.append([a,b])
    for a,b in sorted(exclusions):ET.SubElement(contact,'exclude',body1=bodies[a]['name'],body2=bodies[b]['name'])
    target=next(s for s in d['fruit_specs'] if s['name']=='Tomato_05');p=np.asarray(target['pose']);center=p[:3]+rotation(p).apply(target['center'])
    eye=center+np.array([.13,.38,.16]);z=(eye-center)/np.linalg.norm(eye-center);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    ET.SubElement(world,'camera',name='target',pos=nums(eye),xyaxes=nums([x,y]),fovy='48')
    ET.indent(root);ET.ElementTree(root).write(output,encoding='unicode')
    audit=dict(source=str(reference),bodies=len(bodies),elastic_dofs=len(joint_names),fruit_free_dofs=66,
        plant_colliders=sum(s['body']!=d['tool_path'] for s in d['shapes']),hook_colliders=sum(s['body']==d['tool_path'] for s in d['shapes']),
        mass_kg=sum(b['mass'] for b in bodies.values()),unmapped_filters=unmatched,joint_names=joint_names,
        differences=['D6/spherical rotational coordinates approximated by three colocated Euler hinges; same per-axis spring/damping/limits.',
        '11 native fixed break joints replaced by free fruit bodies plus weld constraints; breaking disabled in BOTH Phase 1 engines.',
        'MuJoCo has one sliding friction coefficient; source dynamic 0.4 used, static 0.5 cannot be represented separately.',
        'solref/solimp are MuJoCo-specific, not a conversion of PhysX restitution or iteration count.',
        'Triangle hull cooking differs by engine; full input vertices retained. Original fruit visuals retained, stems/leaves display collision proxies.',
        'MuJoCo preload computed once from initial gravitational load with fruit mass at attachment, then held constant; no live gravity compensation.',
        'Hook mocap has no solver-integrated inertia/velocity; PhysX kinematic targets do supply contact velocity. Same pose commands do not imply identical contact boundary conditions. No robot arm.'])
    output.with_suffix('.json').write_text(json.dumps(audit,indent=2))
    if unmatched:raise ValueError('Unmapped collision filters: '+str(unmatched[:5]))
    print('[MuJoCo 변환]',json.dumps({k:v for k,v in audit.items() if k not in ('joint_names','differences')},ensure_ascii=False))
    return audit

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--reference',type=Path,default=HOME/'assets/reference/reference.json');p.add_argument('--output',type=Path,default=HOME/'models/plant_original_equivalent.xml');a=p.parse_args();build(a.reference,a.output)
