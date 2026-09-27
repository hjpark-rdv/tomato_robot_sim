"""Place fixed visual copies of the existing plant along the greenhouse gutters."""
import argparse,json,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco as mj
from greenhouse_visual import nums


def template(m, d, asset, variant, prefixes=None, origin=None):
    bid=m.body('STEM_MainStem_00').id
    gid=next(i for i in range(m.ngeom) if m.geom_bodyid[i]==bid and m.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
    anchor=d.geom_xpos[gid]+d.geom_xmat[gid].reshape(3,3)[:,2]*m.geom_size[gid,1]
    if origin is not None:anchor=np.asarray(origin)
    groups={};capsules=[]
    for i in range(m.ngeom):
        if not (m.body(m.geom_bodyid[i]).name or '').startswith(prefixes or ('STEM_','TRUSS_','Tomato_','Attachment_')):continue
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


def build(source,output,count=16,seed=27,spacing=.53125,alternate=None,paired=False,random_trusses=False):
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
        templates.append(template(model,data,asset,variant,prefixes=('STEM_',) if random_trusses else None))
    truss_templates=[]
    if random_trusses:
        import tempfile
        from build_glb_physics import build as build_truss, attachment_frame, tube_centerline
        from generate_random_glb_scenes import profiles, sample_placements, GLB_DIR
        from view_glb_truss import read_glb
        variants=profiles(GLB_DIR)
        for k,v in enumerate(variants):
            with tempfile.TemporaryDirectory(prefix='truss_visual_') as temp:
                part=Path(temp)/'model'
                build_truss(v['path'],part,y_deg=0,segment=4,remove_fruits=v['remove_fruits'],fruit_offsets=v['fruit_offsets'],rachis_stiffness_scale=v['rachis_stiffness_scale'],truss_scale=.5)
                tm=mj.MjModel.from_binary_path(str(part/'model.mjb'));td=mj.MjData(tm);mj.mj_forward(tm,td)
                meta=json.loads((part/'build.json').read_text())
                truss_templates.append(template(tm,td,asset,f'truss_{k}',prefixes=('TRUSS_','Tomato_','Attachment_'),origin=meta['attachment_world_m']))
                (output/f'truss_template_{k}.json').write_text(json.dumps(meta,indent=2))
        peduncles=[tube_centerline(next(v for n,v,f,c in read_glb(vr['path'],True) if n=='Truss_01_Peduncle'),14)[0] for vr in variants]
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
        trusses=[]
        if random_trusses:
            placement=sample_placements(rng,variants,1,4,13,0,180)[0]
            k=next(k for k,v in enumerate(variants) if str(v['path'].resolve())==placement['source_glb'])
            bid=m.body(f"STEM_MainStem_{placement['stem_segment']:02d}").id
            gid=next(i for i in range(m.ngeom) if m.geom_bodyid[i]==bid and m.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
            localrot,attachment=attachment_frame(m,d,gid,peduncles[k],placement['stem_fraction'],placement['y_deg'])
            # Templates already contain the fixed GLB Y-up to world Z-up conversion.
            turn=Rotation.from_euler('z',placement['y_deg'],degrees=True)
            tr=r*turn;tp=position+r.apply(attachment-anchor)
            _,tg,tc=truss_templates[k]
            for q,rgba in enumerate(tg):
                ET.SubElement(world,'geom',name=f'neighbor_{j:02d}_truss_mesh_{q}',type='mesh',mesh=f'neighbor_template_truss_{k}_{q}',pos=nums(tp),quat=nums(tr.as_quat()[[3,0,1,2]]),rgba=nums(rgba),contype='0',conaffinity='0',density='0',group='2')
            assert not tc, 'Truss template must contain visible meshes only'
            placement.update(attachment_world_m=tp.tolist(),truss_scale=.5)
            trusses.append(placement)
        placements.append(dict(id=j,position=position.tolist(),yaw_deg=yaw,trusses=trusses,stem_variant=variant,source=str(sources[variant]),gutter_center_x=slot[3] if paired else x,side=slot[2] if paired else None))
    tree.write(output/'model.xml',encoding='unicode');new=mj.MjModel.from_xml_path(str(output/'model.xml'));mj.mj_saveModel(new,str(output/'model.mjb'))
    assert (new.nq,new.nv,new.nbody,new.nu)==(m.nq,m.nv,m.nbody,m.nu)
    ids=[i for i in range(new.ngeom) if (new.geom(i).name or '').startswith('neighbor_')]
    assert np.all(new.geom_contype[ids]==0) and np.all(new.geom_conaffinity[ids]==0) and np.all(new.geom_bodyid[ids]==0)
    for attr in ('body_mass','body_inertia','dof_damping','jnt_stiffness'):np.testing.assert_array_equal(getattr(m,attr),getattr(new,attr))
    info=dict(source=str(source),seed=seed,neighbors=count,spacing_m=spacing,placements=placements,added_geoms=len(ids),shared_meshes=sum(len(t[1]) for t in templates+truss_templates),visible_capsules_per_variant=[len(t[2]) for t in templates],template_triangles=[sum(sum(len(f) for f in g['f']) for g in t[1].values()) for t in templates],paired_gutter_layout=paired,random_trusses=random_trusses,physics_added=False,scope='Static plant visuals, paired gutter directions when requested, optional rule-based random truss attachments. Scene review only; nearby plants have no collision or elasticity; not collection-ready.')
    (output/'scene.json').write_text(json.dumps(info,indent=2));print(json.dumps({k:v for k,v in info.items() if k!='placements'}),flush=True)
    return output/'model.mjb'

def add_robot_facing_trusses(scene, output, extra_per_plant=2, seed=127):
    """Add visual trusses near the robot, preserving the existing scene layout."""
    import copy
    from build_glb_physics import attachment_frame, tube_centerline
    from generate_random_glb_scenes import profiles, sample_placements, GLB_DIR
    from view_glb_truss import read_glb
    scene=Path(scene);output=Path(output)
    if extra_per_plant<1:raise ValueError('extra_per_plant must be positive')
    info=json.loads((scene/'scene.json').read_text())
    tree=ET.parse(scene/'model.xml');root=tree.getroot();world=root.find('worldbody')
    model=mj.MjModel.from_binary_path(str(scene/'model.mjb'));data=mj.MjData(model);mj.mj_forward(model,data)
    bid=model.body('STEM_MainStem_00').id
    gid=next(i for i in range(model.ngeom) if model.geom_bodyid[i]==bid and model.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
    anchor=data.geom_xpos[gid]+data.geom_xmat[gid].reshape(3,3)[:,2]*model.geom_size[gid,1]
    variants=profiles(GLB_DIR);peduncles=[];centers=[];templates=[]
    fixed=Rotation.from_euler('x',90,degrees=True)
    for k,v in enumerate(variants):
        meshes=read_glb(v['path'],True)
        ped=tube_centerline(next(vv for n,vv,f,c in meshes if n=='Truss_01_Peduncle'),14)[0];peduncles.append(ped)
        fruit=[]
        for n,vv,f,c in meshes:
            if n.startswith('Fruit_'):
                fruit_id=int(n.split('_')[1])
                if fruit_id in v['remove_fruits']:continue
                fruit.append(vv.mean(axis=0)+np.asarray(v['fruit_offsets'].get(fruit_id,[0,0,0])))
        centers.append(fixed.apply((np.mean(fruit,axis=0)-ped[0])*.5))
        by_mesh={g.get('mesh'):g for g in world.findall('geom') if g.get('mesh','').startswith(f'neighbor_template_truss_{k}_')}
        if not by_mesh:raise ValueError(f'Missing truss template {k}')
        templates.append(list(by_mesh.values()))
    rng=np.random.default_rng(seed);added=[];skipped=[]
    for plant in info['placements']:
        position=np.array(plant['position']);r=Rotation.from_euler('z',plant['yaw_deg'],degrees=True)
        # Current robot is near the beginning of the aisle; distant plants unchanged.
        if position[1]>2.25:continue
        accepted=[]
        for attempt in range(600):
            if len(accepted)>=extra_per_plant:break
            p=sample_placements(rng,variants,1,4,13,0,180)[0]
            k=next(k for k,v in enumerate(variants) if str(v['path'].resolve())==p['source_glb'])
            bid=model.body(f"STEM_MainStem_{p['stem_segment']:02d}").id
            gid=next(i for i in range(model.ngeom) if model.geom_bodyid[i]==bid and model.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
            _,a=attachment_frame(model,data,gid,peduncles[k],p['stem_fraction'],p['y_deg'])
            tp=position+r.apply(a-anchor);tr=r*Rotation.from_euler('z',p['y_deg'],degrees=True)
            center=tp+tr.apply(centers[k])
            inward=-np.sign(position[0])*(center[0]-tp[0])
            if not (.55<=center[2]<=1.10 and .1<=center[1]<=1.8 and inward>=.04 and abs(center[0])<=.85):continue
            if any(np.linalg.norm(tp-other)<.12 for other in accepted):continue
            number=len(accepted)
            for q,geom in enumerate(templates[k]):
                g=copy.deepcopy(geom);g.set('name',f"neighbor_{plant['id']:02d}_extra_{number}_mesh_{q}")
                g.set('pos',nums(tp));g.set('quat',nums(tr.as_quat()[[3,0,1,2]]));world.append(g)
            p.update(attachment_world_m=tp.tolist(),fruit_centroid_world_m=center.tolist(),truss_scale=.5,placement_policy='aisle_facing_height_bias',robot_ik_checked=False)
            plant['trusses'].append(p);added.append(dict(plant_id=plant['id'],**p));accepted.append(tp)
        if len(accepted)<extra_per_plant:skipped.append(dict(plant_id=plant['id'],added=len(accepted)))
    output.mkdir(parents=True,exist_ok=False);tree.write(output/'model.xml',encoding='unicode')
    new=mj.MjModel.from_xml_path(str(output/'model.xml'));mj.mj_saveModel(new,str(output/'model.mjb'))
    for attr in ('body_mass','body_inertia','dof_damping','jnt_stiffness'):np.testing.assert_array_equal(getattr(model,attr),getattr(new,attr))
    assert (new.nq,new.nv,new.nbody)==(model.nq,model.nv,model.nbody)
    extra=[i for i in range(new.ngeom) if '_extra_' in (new.geom(i).name or '')]
    assert np.all(new.geom_contype[extra]==0) and np.all(new.geom_conaffinity[extra]==0)
    info.update(parent_scene=str(scene.resolve()),density_seed=seed,extra_trusses=len(added),added_trusses=added,underfilled_plants=skipped,density_policy=dict(fruit_centroid_z_m=[.55,1.10],fruit_centroid_y_m=[.1,1.8],inward_min_m=.04,attachment_spacing_m=.12,extra_per_plant=extra_per_plant,robot_ik_checked=False))
    (output/'scene.json').write_text(json.dumps(info,indent=2))
    print('Added trusses:',len(added),'underfilled:',skipped,flush=True)
    return output/'model.mjb'

def add_stem_obstacles(scene, layout, output):
    """Reuse the physical stem capsules for both rows; background stems are rigid."""
    import shutil,hashlib
    scene=Path(scene);layout=Path(layout);output=Path(output)
    metadata=json.loads((layout/'scene.json').read_text())
    tree=ET.parse(scene/'model.xml');root=tree.getroot();world=root.find('worldbody')
    if any(g.get('name','').startswith('neighbor_stem_collision_') for g in world.findall('geom')):raise ValueError('Stem obstacles already present')
    model=mj.MjModel.from_binary_path(str(scene/'model.mjb'));data=mj.MjData(model);mj.mj_forward(model,data)
    rods=[]
    for segment in range(16):
        bid=model.body(f'STEM_MainStem_{segment:02d}').id
        gid=next(i for i in range(model.ngeom) if model.geom_bodyid[i]==bid and model.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
        axis=data.geom_xmat[gid].reshape(3,3)[:,2]*model.geom_size[gid,1]
        rods.append((data.geom_xpos[gid]+axis,data.geom_xpos[gid]-axis,float(model.geom_size[gid,0])))
    anchor=rods[0][0];inventory=[];registry=[]
    for plant in metadata['placements']:
        pos=np.array(plant['position']);rot=Rotation.from_euler('z',plant['yaw_deg'],degrees=True)
        center=plant['gutter_center_x'];near=(pos[0]-center)*(-np.sign(center))>0
        registry.append(dict(plant_id=plant['id'],row='robot_side' if near else 'opposite_side',target_eligible=bool(near),physics_targets_available=False,trusses=plant['trusses']))
        for segment,(a,b,radius) in enumerate(rods):
            endpoints=rot.apply(np.array([a,b])-anchor)+pos
            name=f"neighbor_stem_collision_{plant['id']:02d}_{segment:02d}"
            ET.SubElement(world,'geom',name=name,type='capsule',fromto=nums(endpoints),size=str(radius),contype='32',conaffinity='15',density='0',group='3',rgba='.2 .5 1 0',friction='.5 .005 .0001',condim='3',margin='0',gap='0',solref='.004 1',solimp='.99 .999 .001')
            inventory.append(dict(name=name,plant_id=plant['id'],segment=segment,endpoints=endpoints.tolist(),radius_m=radius))
    output.mkdir(parents=True,exist_ok=False);tree.write(output/'model.xml',encoding='unicode')
    new=mj.MjModel.from_xml_path(str(output/'model.xml'));mj.mj_saveModel(new,str(output/'model.mjb'))
    for attr in ('body_mass','body_inertia','dof_damping','jnt_stiffness'):np.testing.assert_array_equal(getattr(model,attr),getattr(new,attr))
    shutil.copy2(scene/'reference.json',output/'reference.json')
    record=json.loads((scene/'scene.json').read_text())
    gutter=min([-.775,.775],key=lambda x:abs(anchor[0]-x))
    active_near=(anchor[0]-gutter)*(-np.sign(gutter))>0
    specs=json.loads((scene/'reference.json').read_text())['fruit_specs']
    record.update(model_sha256=hashlib.sha256((output/'model.mjb').read_bytes()).hexdigest(),eligible_targets=[x['name'] for x in specs] if active_near else [],target_policy='robot-side stem roots relative to aisle x=0; opposite-side fruits excluded',background_stem_physics='fixed capsules; no background bending',background_stem_colliders=len(inventory),validation={'scene_screen_passed':False,'note':'Needs fresh initial screen'})
    (output/'scene.json').write_text(json.dumps(record,indent=2))
    (output/'stem_obstacles.json').write_text(json.dumps(dict(source_scene=str(scene.resolve()),layout_scene=str(layout.resolve()),inventory=inventory,plants=registry,active_target_stem_robot_side=bool(active_near)),indent=2))
    print('Stem obstacles',len(inventory),'eligible active fruits',len(record['eligible_targets']),flush=True)
    return output/'model.mjb'

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('model',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--count',type=int,default=16);p.add_argument('--seed',type=int,default=27);p.add_argument('--spacing',type=float,default=.53125);p.add_argument('--alternate-model',type=Path);p.add_argument('--paired-gutters',action='store_true');p.add_argument('--random-trusses',action='store_true');p.add_argument('--densify-robot-side',action='store_true',help='model 인자 대신 기존 장면 폴더 사용');p.add_argument('--extra-per-plant',type=int,default=2);p.add_argument('--stem-obstacles-layout',type=Path,help='주변 줄기 배치 scene.json이 있는 폴더');a=p.parse_args()
    if a.stem_obstacles_layout:add_stem_obstacles(a.model,a.stem_obstacles_layout,a.output)
    elif a.densify_robot_side:add_robot_facing_trusses(a.model,a.output,a.extra_per_plant,a.seed)
    else:build(a.model,a.output,a.count,a.seed,a.spacing,a.alternate_model,a.paired_gutters,a.random_trusses)
