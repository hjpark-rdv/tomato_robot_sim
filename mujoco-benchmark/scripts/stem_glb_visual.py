"""Register matching GLB stem foliage on the existing elastic stem skeleton."""
import argparse,hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
from view_glb_truss import read_glb
from build_glb_physics import tube_centerline,resample_rod
from convert_model import nums


def add_stem(model_xml,output_xml,glb,scale=.5):
    model_xml=Path(model_xml);output_xml=Path(output_xml);glb=Path(glb)
    if not np.isfinite(scale) or scale <= 0:raise ValueError('Scale must be finite and positive')
    m=mj.MjModel.from_xml_path(str(model_xml));d=mj.MjData(m);mj.mj_forward(m,d)
    tree=ET.parse(model_xml);root=tree.getroot();asset=root.find('asset')
    if any((g.get('name') or '').startswith('stem_glb_visual_') for g in root.iter('geom')):raise ValueError('Stem GLB already applied')
    meshes=read_glb(glb,named=True);main=[v for n,v,f,c in meshes if n=='MainStem']
    if len(main)!=1:raise ValueError('Expected one MainStem primitive')
    rotation=np.array([[1.,0,0],[0,0,-1],[0,1,0]])
    rawpoints,rawradius=tube_centerline(main[0]@rotation.T*scale,24)
    points,radius=resample_rod(rawpoints,rawradius,16)
    bids=[m.body(f'STEM_MainStem_{i:02d}').id for i in range(16)]
    starts=[];ends=[];sizes=[]
    for bid in bids:
        gid=next(i for i in range(m.ngeom) if m.geom_bodyid[i]==bid and m.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
        half=d.geom_xmat[gid].reshape(3,3)[:,2]*m.geom_size[gid,1]
        starts.append(d.geom_xpos[gid]+half);ends.append(d.geom_xpos[gid]-half);sizes.append(m.geom_size[gid,0])
    starts=np.array(starts);ends=np.array(ends);offset=starts[0]-points[0]
    # Existing capsules point from tip to root. Reject a different stem shape;
    # visual replacement must not misrepresent the physical collision skeleton.
    error=max(np.max(np.abs(points[:-1]+offset-starts)),np.max(np.abs(points[1:]+offset-ends)))
    radius_error=float(np.max(np.abs(np.maximum(radius[:-1],radius[1:])-sizes)))
    if error>1e-5 or radius_error>1e-5:raise ValueError(f'Stem differs from existing skeleton: centerline {error} m, radius {radius_error} m. Separate physical conversion required.')
    nodes={n.get('name'):n for n in root.iter('body')};hidden=0
    for name,node in nodes.items():
        if not name.startswith('STEM_'):continue
        for g in node.findall('geom'):
            color=list(map(float,g.get('rgba','.5 .5 .5 1').split()));color[3]=0;g.set('rgba',nums(color));hidden+=1
    delta=ends-starts
    def distances(v):
        t=np.clip(np.einsum('nsi,si->ns',v[:,None,:]-starts,delta)/np.sum(delta*delta,axis=1),0,1)
        return np.linalg.norm(v[:,None,:]-(starts+delta*t[:,:,None]),axis=2)
    transformed=[(n,v@rotation.T*scale+offset,f,c) for n,v,f,c in meshes]
    owners={}
    for n in dict.fromkeys(n for n,_,_,_ in transformed):
        if n=='MainStem':continue
        v=np.concatenate([v for name,v,f,c in transformed if name==n]);dist=distances(v);owners[n]=int(np.unravel_index(np.argmin(dist),dist.shape)[1])
    inventory=[];count=0
    for primitive,(name,v,f,color) in enumerate(transformed):
        assigned=distances(v[f].mean(axis=1)).argmin(axis=1) if name=='MainStem' else np.full(len(f),owners[name])
        for segment in np.unique(assigned):
            faces=f[assigned==segment];unique,inv=np.unique(faces,return_inverse=True);bid=bids[int(segment)];local=(v[unique]-d.xpos[bid])@d.xmat[bid].reshape(3,3)
            geom=f'stem_glb_visual_{count:04d}';ET.SubElement(asset,'mesh',name=geom,vertex=nums(local),face=' '.join(map(str,inv.reshape(-1))))
            ET.SubElement(nodes[m.body(bid).name],'geom',name=geom,type='mesh',mesh=geom,rgba=nums(color),contype='0',conaffinity='0',density='0',group='2')
            inventory.append(dict(primitive=primitive,name=name,body=m.body(bid).name,triangles=len(faces)));count+=1
    output_xml.parent.mkdir(parents=True,exist_ok=True);tree.write(output_xml,encoding='unicode')
    new=mj.MjModel.from_xml_path(str(output_xml));mj.mj_saveModel(new,str(output_xml.with_suffix('.mjb')))
    for i in range(new.ngeom):
        if (new.geom(i).name or '').startswith('stem_glb_visual_'):
            assert new.geom_contype[i]==0 and new.geom_conaffinity[i]==0
    for attr in ('nq','nv','nbody','nu'):assert getattr(new,attr)==getattr(m,attr)
    for attr in ('body_mass','body_inertia','dof_damping','jnt_stiffness'):np.testing.assert_array_equal(getattr(new,attr),getattr(m,attr))
    assert sum(x['triangles'] for x in inventory)==sum(len(f) for n,v,f,c in meshes)
    info=dict(source=str(glb.resolve()),sha256=hashlib.sha256(glb.read_bytes()).hexdigest(),scale=scale,glb_to_world_rotation=rotation.tolist(),translation=offset.tolist(),source_primitives=len(meshes),source_triangles=sum(len(f) for n,v,f,c in meshes),added_visual_geoms=count,hidden_old_stem_geoms=hidden,centerline_error_m=float(error),radius_error_m=radius_error,inventory=inventory,scope='New GLB visuals attached to existing matching 16-segment elastic stem; all source primitives preserved. Leaf mass/collision/flex not added. Main stem physical parameters unchanged; rigid visual segmentation is not smooth skinning.')
    output_xml.with_suffix('.stem.json').write_text(json.dumps(info,indent=2));return info

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('model',type=Path);p.add_argument('--glb',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--scale',type=float,default=.5);a=p.parse_args()
    if a.output.exists():p.error('Use a new output file')
    result=add_stem(a.model,a.output,a.glb,a.scale);print({k:v for k,v in result.items() if k!='inventory'},flush=True)
