"""Place fixed visual copies of the existing plant along the greenhouse gutters."""
import argparse,json,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco as mj
from greenhouse_visual import nums


def template(m, d, asset, variant):
    bid=m.body('STEM_MainStem_00').id
    gid=next(i for i in range(m.ngeom) if m.geom_bodyid[i]==bid and m.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
    anchor=d.geom_xpos[gid]+d.geom_xmat[gid].reshape(3,3)[:,2]*m.geom_size[gid,1]
    groups={};capsules=[]
    for i in range(m.ngeom):
        if not (m.body(m.geom_bodyid[i]).name or '').startswith(('STEM_','TRUSS_','Tomato_','Attachment_')):continue
        mat=int(m.geom_matid[i]);rgba=m.mat_rgba[mat].copy() if mat>=0 else m.geom_rgba[i].copy()
        # Explicit transparent collision proxies must stay excluded.
        if m.geom_rgba[i,3]==0 or rgba[3]<.1:continue
        pos=d.geom_xpos[i]-anchor;rot=d.geom_xmat[i].reshape(3,3)
        if m.geom_type[i]==mj.mjtGeom.mjGEOM_MESH:
            mid=m.geom_dataid[i];v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]].astype(float)
            f=m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid]+m.mesh_facenum[mid]].copy()
            key=tuple(float(x) for x in rgba);g=groups.setdefault(key,dict(v=[],f=[],count=0))
            g['v'].append(v@rot.T+pos);g['f'].append(f+g['count']);g['count']+=len(v)
        elif m.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE:
            capsules.append(dict(pos=pos,rot=rot,size=m.geom_size[i,:2].copy(),rgba=rgba))
        else:raise ValueError(f'Unsupported visible plant geom {m.geom(i).name}')
    for k,(rgba,g) in enumerate(groups.items()):
        ET.SubElement(asset,'mesh',name=f'neighbor_template_{variant}_{k}',vertex=nums(np.concatenate(g['v'])),face=' '.join(map(str,np.concatenate(g['f']).ravel())))
    return anchor,groups,capsules


def build(source,output,count=16,seed=27,spacing=.53125,alternate=None,paired=False):
    if count<1:raise ValueError('count must be positive')
    if not np.isfinite(spacing) or spacing<=0:raise ValueError('spacing must be positive')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    source=Path(source).resolve();m=mj.MjModel.from_xml_path(str(source));d=mj.MjData(m);mj.mj_forward(m,d)
    tree=ET.parse(source);root=tree.getroot();asset=root.find('asset');world=root.find('worldbody')
    sources=[source]+([Path(alternate).resolve()] if alternate else [])
    templates=[]
    for variant,path in enumerate(sources):
        model=m if variant==0 else mj.MjModel.from_xml_path(str(path))
        data=d if variant==0 else mj.MjData(model)
        mj.mj_forward(model,data)
        templates.append(template(model,data,asset,variant))
    anchor=templates[0][0]
    rng=np.random.default_rng(seed)
    slots=[(x,float(y)) for x in [-.775,.775] for y in np.arange(.4,4.65+1e-6,spacing) if np.linalg.norm(np.array([x,y])-anchor[:2])>max(.14,spacing*.65)]
    if paired:
        # Each gutter has plants on both edges, bent along opposite row directions.
        slots=[(center+side*.10,float(y),side,center) for center in [-.775,.775]
               for side in [-1,1] for y in np.arange(.4,4.65+1e-6,spacing)
               if np.linalg.norm(np.array([center+side*.10,y])-anchor[:2])>.18]
    if count>len(slots):raise ValueError(f'Only {len(slots)} slots available at spacing {spacing}')
    # Fill both rows uniformly, with nearby neighbors retained.
    chosen=np.linspace(0,len(slots)-1,count).round().astype(int);placements=[]
    for j,index in enumerate(chosen):
        slot=slots[index];x,y=slot[:2]
        variant=int(rng.integers(len(templates)));_,groups,capsules=templates[variant]
        position=np.array([x+rng.uniform(-.015,.015),y+rng.uniform(-.025,.025),anchor[2]])
        yaw=float((0 if slot[2]<0 else 180) if paired else rng.uniform(-18,18));r=Rotation.from_euler('z',yaw,degrees=True);quat=r.as_quat()[[3,0,1,2]]
        for k,rgba in enumerate(groups):
            ET.SubElement(world,'geom',name=f'neighbor_{j:02d}_mesh_{k}',type='mesh',mesh=f'neighbor_template_{variant}_{k}',pos=nums(position),quat=nums(quat),rgba=nums(rgba),contype='0',conaffinity='0',density='0',group='2',margin='0',gap='0')
        for k,c in enumerate(capsules):
            q=(r*Rotation.from_matrix(c['rot'])).as_quat()[[3,0,1,2]]
            ET.SubElement(world,'geom',name=f'neighbor_{j:02d}_rod_{k}',type='capsule',pos=nums(position+r.apply(c['pos'])),quat=nums(q),size=nums(c['size']),rgba=nums(c['rgba']),contype='0',conaffinity='0',density='0',group='2',margin='0',gap='0')
        placements.append(dict(id=j,position=position.tolist(),yaw_deg=yaw,stem_variant=variant,source=str(sources[variant]),gutter_center_x=slot[3] if paired else x,side=slot[2] if paired else None))
    tree.write(output/'model.xml',encoding='unicode');new=mj.MjModel.from_xml_path(str(output/'model.xml'));mj.mj_saveModel(new,str(output/'model.mjb'))
    assert (new.nq,new.nv,new.nbody,new.nu)==(m.nq,m.nv,m.nbody,m.nu)
    ids=[i for i in range(new.ngeom) if (new.geom(i).name or '').startswith('neighbor_')]
    assert np.all(new.geom_contype[ids]==0) and np.all(new.geom_conaffinity[ids]==0) and np.all(new.geom_bodyid[ids]==0)
    for attr in ('body_mass','body_inertia','dof_damping','jnt_stiffness'):np.testing.assert_array_equal(getattr(m,attr),getattr(new,attr))
    info=dict(source=str(source),seed=seed,neighbors=count,spacing_m=spacing,placements=placements,added_geoms=len(ids),shared_meshes=sum(len(t[1]) for t in templates),visible_capsules_per_variant=[len(t[2]) for t in templates],template_triangles=[sum(sum(len(f) for f in g['f']) for g in t[1].values()) for t in templates],paired_gutter_layout=paired,physics_added=False,scope='Static repeated existing plant visuals with placement/yaw jitter. Scene review only; nearby plants have no collision or elasticity; not collection-ready.')
    (output/'scene.json').write_text(json.dumps(info,indent=2));print(json.dumps({k:v for k,v in info.items() if k!='placements'}),flush=True)
    return output/'model.mjb'

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('model',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--count',type=int,default=16);p.add_argument('--seed',type=int,default=27);p.add_argument('--spacing',type=float,default=.53125);p.add_argument('--alternate-model',type=Path);p.add_argument('--paired-gutters',action='store_true');a=p.parse_args();build(a.model,a.output,a.count,a.seed,a.spacing,a.alternate_model,a.paired_gutters)
