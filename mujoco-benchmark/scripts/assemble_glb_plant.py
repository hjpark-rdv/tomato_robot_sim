"""Reproducible multiple GLB truss attachments, Y-only 0..180 degrees; visual only."""
import argparse,copy,hashlib,json,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from view_glb_truss import build
from attach_glb_preview import ROOT,draw_glb_axes

def assemble(source_dir,output,seed):
    import mujoco as mj
    files=sorted(source_dir.glob('*.glb'))
    if not files:raise ValueError('GLB files missing')
    old=ROOT/'mujoco-benchmark/models/plant_mujoco_optimized.xml'
    tree=ET.parse(old);root=tree.getroot();world=root.find('worldbody');stem=world.find("body[@name='STEM_MainStem_00']")
    m=mj.MjModel.from_xml_path(str(old));d=mj.MjData(m);mj.mj_forward(m,d)
    for node in list(stem.iter('body')):
        for child in list(node.findall('body')):
            if child.get('name','').startswith('TRUSS_'):node.remove(child)
    for child in list(world):
        if child.tag=='body' and child is not stem:world.remove(child)
    retained={node.get('name') for node in stem.iter('body')}
    for item in list(root.find('contact')):
        if item.get('body1') not in retained or item.get('body2') not in retained:root.find('contact').remove(item)
    for tag in ['equality','actuator','sensor','tendon','keyframe']:
        for item in root.findall(tag):root.remove(item)
    nodes=sorted([node for node in stem.iter('body') if node.get('name','').startswith('STEM_MainStem_')],key=lambda n:n.get('name'))
    segments=[]
    for node in nodes:
        bid=m.body(node.get('name')).id
        gid=next(i for i in range(m.ngeom) if m.geom_bodyid[i]==bid and m.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
        geom=node.find("geom[@name='"+m.geom(gid).name+"']")
        ends=np.fromstring(geom.get('fromto'),sep=' ').reshape(2,3)@d.xmat[bid].reshape(3,3).T+d.xpos[bid]
        segments.append((bid,gid,ends))
    lengths=np.array([np.linalg.norm(x[2][1]-x[2][0]) for x in segments]);cumulative=np.r_[0,np.cumsum(lengths)]
    rng=np.random.default_rng(seed);rng.shuffle(files);basis=Rotation.from_euler('x',90,degrees=True).as_matrix();records=[]
    # Stratified random positions keep attachments distributed along the existing stem.
    limits=np.linspace(.15*cumulative[-1],.92*cumulative[-1],len(files)+1)
    for i,path in enumerate(files):
        arc=float(rng.uniform(limits[i]+.15*(limits[i+1]-limits[i]),limits[i+1]-.15*(limits[i+1]-limits[i])))
        seg=min(int(np.searchsorted(cumulative,arc,side='right')-1),len(nodes)-1);bid,gid,ends=segments[seg];fraction=(arc-cumulative[seg])/lengths[seg]
        center=ends[0]+fraction*(ends[1]-ends[0]);axis=(ends[1]-ends[0])/lengths[seg]
        angle=float(rng.uniform(0,180));rotation=basis@Rotation.from_euler('y',angle,degrees=True).as_matrix()
        xml,vertices=build(path);glb=ET.fromstring(xml)
        heading=basis@vertices.mean(0);normal=heading-axis*np.dot(axis,heading);normal/=np.linalg.norm(normal)
        attachment=center+normal*m.geom_size[gid,0];prefix=f'truss_{i:02d}_';body=glb.find("worldbody/body[@name='glb_origin']");body.set('name',prefix+'origin')
        for asset in glb.find('asset'):
            asset.set('name',prefix+asset.get('name'));root.find('asset').append(copy.deepcopy(asset))
        for geom in body.findall('geom'):
            geom.set('mesh',prefix+geom.get('mesh'));geom.set('density','0')
        localrot=d.xmat[bid].reshape(3,3).T@rotation;q=Rotation.from_matrix(localrot).as_quat()
        body.set('quat',' '.join(map(str,q[[3,0,1,2]])));body.set('pos',' '.join(map(str,d.xmat[bid].reshape(3,3).T@(attachment-d.xpos[bid]))));nodes[seg].append(body)
        records.append(dict(id=prefix+'origin',source_glb=str(path.resolve()),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),parent=nodes[seg].get('name'),arclength_m=arc,attachment_world_m=attachment.tolist(),y_deg=angle,world_rotation_matrix=rotation.tolist()))
    root.set('model',f'GLB_FIVE_TRUSSES_Y_ONLY_SEED_{seed}');output.mkdir(parents=True,exist_ok=False);tree.write(output/'scene.xml',encoding='unicode')
    a=mj.MjModel.from_xml_path(str(output/'scene.xml'));ad=mj.MjData(a);mj.mj_forward(a,ad)
    for record in records:
        bid=a.body(record['id']).id;np.testing.assert_allclose(ad.xpos[bid],record['attachment_world_m'],atol=1e-10);np.testing.assert_allclose(ad.xmat[bid].reshape(3,3),record['world_rotation_matrix'],atol=1e-10);np.testing.assert_allclose(ad.xmat[bid].reshape(3,3)[:,1],[0,0,1],atol=1e-10)
    preserved_geoms=0
    for name in retained:
        i=m.body(name).id;j=a.body(name).id
        for attr in ['xpos','xmat']:np.testing.assert_allclose(getattr(d,attr)[i],getattr(ad,attr)[j],atol=1e-10)
        for attr in ['body_mass','body_inertia']:np.testing.assert_allclose(getattr(m,attr)[i],getattr(a,attr)[j],atol=1e-10)
        for g in range(m.ngeom):
            if m.geom_bodyid[g]!=i:continue
            h=a.geom(m.geom(g).name).id;preserved_geoms+=1
            for attr in ['geom_xpos','geom_xmat']:np.testing.assert_allclose(getattr(d,attr)[g],getattr(ad,attr)[h],atol=1e-10)
    meta=dict(seed=seed,source_stem=str(old),source_stem_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),scale=1,rotation_rule='Rx90_basis @ Ry_uniform_0_180',trusses=records,visual_only=True,physics_ready=False,validation=dict(truss_transforms_passed=True,stem_geoms_preserved=preserved_geoms),limitations=['No truss elasticity or collision; overlaps not filtered'])
    (output/'scene.json').write_text(json.dumps(meta,indent=2));return a,ad,meta

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=ROOT/'nvidia-sim/env_usd/tomato_rotate_glb');p.add_argument('--output',type=Path,required=True);p.add_argument('--seed',type=int,default=23);p.add_argument('--view',action='store_true');p.add_argument('--axes',action='store_true');args=p.parse_args()
    import mujoco as mj
    import mujoco.viewer
    m,d,meta=assemble(args.source,args.output,args.seed)
    for r in meta['trusses']:print(f"{r['id']} {Path(r['source_glb']).stem}: Y={r['y_deg']:.1f}°, arc={r['arclength_m']:.3f}m",flush=True)
    if args.view:
        with mj.viewer.launch_passive(m,d) as viewer:
            viewer.cam.lookat[:]=np.mean([r['attachment_world_m'] for r in meta['trusses']],axis=0);viewer.cam.distance=3.4;viewer.cam.azimuth=110;viewer.cam.elevation=-8
            viewer.opt.label=mj.mjtLabel.mjLABEL_SELECTION
            while viewer.is_running():
                if args.axes:
                    with viewer.lock():
                        viewer.user_scn.ngeom=0
                        for r in meta['trusses']:draw_glb_axes(viewer.user_scn,np.array(r['attachment_world_m']),np.array(r['world_rotation_matrix']))
                viewer.sync();time.sleep(1/30)
if __name__=='__main__':main()
