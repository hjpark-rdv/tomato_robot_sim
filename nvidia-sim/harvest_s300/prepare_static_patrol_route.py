"""Extend geometric patrol by registering existing farm instances, not rebuilding plants.

Reuse all robot, camera, farm meshes, colors and placements. Read only the
existing MainStem surface for the same16 capsule approximation, and register
existing fruit instances as static, individually hideable harvest identities.
No plant articulation, attachment constraint, elastic conversion or new mesh.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent

def read(p):return json.loads(Path(p).read_text())
def write(p,v):Path(p).write_text(json.dumps(v,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def vector(v):return ' '.join(format(float(x),'.17g') for x in v)


def prepare(reference,output,end):
    from pxr import Usd,UsdGeom
    from motion_assets import geometry,tube_centerline,resample_rod
    from row_harvest_targets import select_one_bed
    reference=Path(reference).resolve(strict=True);output=Path(output).resolve()
    root=read(reference/'patrol_assets.json');profiles={k:Path(v) for k,v in root['start_modes'].items()}
    if set(profiles)!={'up','down'}:raise ValueError('Reviewed up/down profiles required')
    config=read(profiles['up']/'patrol_assets.json')
    if config.get('harvest_mode')!='main_stem_only_geometric_harvest':raise ValueError('Static geometric harvest required')
    if config.get('geometric_harvest',{}).get('all_plants_static') is not True:
        raise ValueError('This adapter extends an already static plant scene only')
    if not np.isfinite(end) or not config['end_station_m']<=end<=5:raise ValueError('Extend within the original0..5m rail')
    output.mkdir(parents=True,exist_ok=False);start=time.perf_counter()
    inventory=read(profiles['up']/'inventory.json');catalog=read(profiles['up']/'sensor_catalog.json')
    selection=select_one_bed([dict(plant_id=r['plant_id'],truss_index=r['truss_index']) for r in inventory['trusses']],inventory,config['bed_gutter_center_x'])
    allowed={(r['plant_id'],r['truss_index']) for r in selection['stations']}
    chosen=sorted({r['plant_id'] for r in inventory['trusses'] if (r['plant_id'],r['truss_index']) in allowed and 0<=r['fruit_centroid_world_m'][1]<=end})
    existing={r['plant_id'] for r in config['plants']}
    if not existing<=set(chosen):raise ValueError('Extension must retain all previously prepared plants')
    stage=Usd.Stage.Open(config['source_scene']);cache=UsdGeom.XformCache();plants=[]
    for plant in sorted(set(chosen)-existing):
        path=f'/World/Greenhouse/Plants/Plant_{plant:02d}'
        objects=[r for r in catalog['objects'].values() if r.get('plant')==path]
        stem=[r for r in objects if r['class_name']=='main_stem' and r['source_name']=='MainStem']
        paths=[p for obj in stem for p in obj['mesh_paths']
               if stage.GetPrimAtPath(p).GetCustomDataByKey('sourceName')=='MainStem']
        if len(paths)!=1:raise ValueError('Exact existing main stem surface required')
        vertices,_,_=geometry(stage.GetPrimAtPath(paths[0]),cache)
        points,radii=resample_rod(*tube_centerline(vertices,24),16)
        fruits=[]
        for obj in sorted(objects,key=lambda r:r['key']):
            if obj['class_name']!='fruit':continue
            parts=[geometry(stage.GetPrimAtPath(p),cache)[0] for p in obj['mesh_paths']]
            vertices=np.concatenate(parts);center=(vertices.max(0)+vertices.min(0))/2
            source=next(r for r in inventory['trusses'] if r['plant_id']==plant and r['path']==path+'/'+obj['truss'])
            np.testing.assert_allclose(center,source['fruits'][obj['source_name']],rtol=0,atol=1e-9)
            name=obj['truss']+'_'+obj['source_name'];body=f'p{plant:02d}_'+name
            fruits.append(dict(name=name,path=f'/World/StaticTargets/Plant_{plant:02d}/{name}',
                joint=f'logical_capture_only/{plant}/{name}',center=center.tolist(),rest_position=center.tolist(),
                dimensions_m=np.ptp(vertices,axis=0).tolist(),body=body,instance_id=obj['instance_id'],
                source_paths=obj['mesh_paths'],authored_rgb=obj['authored_rgb'],hue_deg=obj['hue_deg'],eligible=obj['red_proxy']))
        if not fruits:raise ValueError('No existing fruit instances for '+path)
        plants.append(dict(plant_id=plant,source_plant=path,fruits=fruits,
                           stem_points=points.tolist(),stem_radii=radii.tolist(),stem_source=paths[0]))
        print('Reused existing plant',plant,'fruit instances',len(fruits),flush=True)
    registration=dict(schema='s300-static-farm-registration-v1',reference=str(reference),profiles={k:str(p.resolve()) for k,p in profiles.items()},
        source_scene=config['source_scene'],source_scene_sha256=sha(config['source_scene']),end_station_m=end,
        chosen_plants=chosen,plants=plants,mesh_files_created=0,plant_joints_created=0,
        extraction_wall_s=time.perf_counter()-start)
    write(output/'registration.json',registration)
    subprocess.run([str(ROOT/'nvidia-sim/.newton-venv/bin/python'),str(Path(__file__).resolve()),str(output),'--compile-registration'],check=True)
    result=read(output/'route_preparation.json');result['preparation_wall_s']=time.perf_counter()-start
    write(output/'route_preparation.json',result);return result


def absolutize_assets(tree,source):
    compiler=tree.getroot().find('compiler');meshdir=Path(source)/compiler.get('meshdir','')
    texturedir=Path(source)/compiler.get('texturedir','')
    for item in tree.getroot().find('asset'):
        if item.get('file'):
            base=texturedir if item.tag=='texture' else meshdir
            item.set('file',str((base/item.get('file')).resolve(strict=True)))


def compile_registration(output):
    import mujoco as mj
    from newton_prepared_assets import verify_manifest_files,link_manifest_assets
    reg=read(output/'registration.json');old=Path(reg['profiles']['up']);cfg=read(old/'patrol_assets.json')
    if sha(reg['source_scene'])!=reg['source_scene_sha256']:raise ValueError('Farm source changed during registration')
    up=output/'up';up.mkdir();(up/'model').mkdir();inputs=up/'inputs';inputs.mkdir()
    physical=ET.parse(old/'inputs/input_scene.xml');absolutize_assets(physical,old/'inputs')
    visual=ET.parse(old/'model/scene.xml');absolutize_assets(visual,old/'model')
    pworld=physical.getroot().find('worldbody');vworld=visual.getroot().find('worldbody')
    geoms={g.get('name'):g for g in visual.getroot().iter('geom')};parents={c:p for p in visual.getroot().iter() for c in p}
    catalog=read(old/'sensor_catalog.json');by_instance={}
    for name,i in catalog['geom_to_instance'].items():by_instance.setdefault(i,[]).append(name)
    proof=read(old/'inputs/inputs.json');verify_manifest_files(old/'inputs',proof['input_files'])
    meta=read(old/'inputs/source_export.json');policy=copy.deepcopy(proof['geometric_harvest'])
    original=mj.MjModel.from_binary_path(str(old/'inputs/input_model.mjb'))
    template=next(g for b in physical.getroot().iter('body') if b.get('name')=='Stem_MainStem_0' for g in b.findall('geom'))
    mapping=dict(proof['source_body_to_native_name']);new_bodies=[];stem_names=[]
    existing_fruits=meta.get('geometric_fruit_names',[a['name'] for a in meta['attachments']])
    fruits=list(existing_fruits)
    new_sources=[];first=cfg['plants'][0]
    for plant in reg['plants']:
        pid=plant['plant_id'];p=up/f'plant_{pid:02d}';p.mkdir()
        for i,(a,b) in enumerate(zip(plant['stem_points'][:-1],plant['stem_points'][1:])):
            a,b=np.asarray(a),np.asarray(b);center=(a+b)/2;name=f'p{pid:02d}_Stem_MainStem_{i}'
            body=ET.SubElement(pworld,'body',name=name,pos=vector(center))
            attrs={k:v for k,v in template.attrib.items() if k not in ('name','size','pos','quat','fromto','mesh')}
            attrs.update(name=name+'_Collider',type='capsule',fromto=vector(np.r_[a-center,b-center]),size=vector([max(plant['stem_radii'][i:i+2])]))
            ET.SubElement(body,'geom',attrs);stem_names.append(attrs['name']);new_bodies.append(name)
            mapping[f'/World/StaticTargets/Plant_{pid:02d}/Stem_MainStem_{i}']=name
        for f in plant['fruits']:
            center=np.asarray(f['center']);name=f['body'];fruits.append(name);new_bodies.append(name)
            ET.SubElement(pworld,'body',name=name,pos=vector(center))
            vb=ET.SubElement(vworld,'body',name=name,pos=vector(center));mapping[f['path']]=name
            moved=by_instance.get(f['instance_id'],[])
            if not moved:raise ValueError('Fruit has no existing visual instance')
            for n in moved:
                geom=geoms[n]
                if parents[geom] is not vworld:raise ValueError('Expected background fruit on world body')
                geom.set('pos',vector(np.fromstring(geom.get('pos','0 0 0'),sep=' ')-center))
                vworld.remove(geom);vb.append(geom)
        manifest=dict(schema='s300-static-fruit-catalog-v1',plant=dict(source_plant=plant['source_plant'],fruits=plant['fruits']),
            registration_sha256=sha(output/'registration.json'),physical_attachments=False)
        write(p/'fruit_catalog.json',manifest)
        rows=[dict(name=f['name'],path=f['path'],joint=f['joint'],eligible=f['eligible'],authored_rgb=f['authored_rgb'],
                   hue_deg=f['hue_deg'],matching_visuals=f['source_paths']) for f in plant['fruits']]
        write(p/'policy.json',dict(schema='s300-harvest-group-policy-v1',minimum_fruits=2,fruit_eligibility=rows,
            source_sha256={},static_catalog_sha256=sha(p/'fruit_catalog.json'),training_eligible=False,
            scope='Existing authored RGB SIM-GT; logical capture only; no plant joints'))
        new_sources.append(dict(plant_id=pid,prefix=f'p{pid:02d}_',assembly=first['assembly'],
            fruit_catalog=str(p/'fruit_catalog.json'),fruit_catalog_sha256=sha(p/'fruit_catalog.json'),
            policy=str(p/'policy.json'),camera_calibration=first['camera_calibration'],shared_robot_assembly=True))
    physical.write(inputs/'input_scene.xml',encoding='unicode');visual.write(up/'model/scene.xml',encoding='unicode')
    model=mj.MjModel.from_xml_path(str(inputs/'input_scene.xml'))
    # Compiling static additions must not change the original dynamic system.
    if (model.nq,model.nv,model.neq)!=(original.nq,original.nv,original.neq):raise ValueError('Original DOF/equality counts changed')
    model.opt.timestep=original.opt.timestep;model.flg_surfacevel=original.flg_surfacevel
    for field in ('dof_damping','dof_frictionloss','geom_surfacevel'):
        before=getattr(original,field);getattr(model,field)[:len(before)]=before
    for name in ('qpos0','jnt_range','actuator_gainprm','actuator_biasprm','dof_damping'):
        np.testing.assert_allclose(getattr(original,name),getattr(model,name),rtol=0,atol=1e-12,err_msg=name)
    for i in range(original.nbody):
        if model.body(i).name!=original.body(i).name:raise ValueError('Original body numbering changed')
        for field in ('body_pos','body_quat','body_mass','body_inertia','body_ipos','body_iquat'):
            np.testing.assert_allclose(getattr(model,field)[i],getattr(original,field)[i],rtol=0,atol=2e-11,err_msg=field+':'+str(i))
    for i in range(original.ngeom):
        if model.geom(i).name!=original.geom(i).name:raise ValueError('Original collider numbering changed')
        for field in ('geom_contype','geom_conaffinity','geom_size','geom_pos','geom_quat','geom_friction','geom_solref','geom_solimp'):
            np.testing.assert_allclose(getattr(model,field)[i],getattr(original,field)[i],rtol=0,atol=2e-11,err_msg=field+':'+str(i))
    mj.mj_saveModel(model,str(inputs/'input_model.mjb'))
    for name in stem_names:
        i=model.geom(name).id;policy['main_stem_geoms'].append(dict(geom=name,body=model.body(int(model.geom_bodyid[i])).name,geom_id=i,
            source_masks=[int(model.geom_contype[i]),int(model.geom_conaffinity[i])]))
        if not (model.geom_contype[i] and model.geom_conaffinity[i]):raise ValueError('Main stem collision was disabled')
    policy['model_sha256']=sha(inputs/'input_model.mjb')
    evidence=dict(source_inputs=str(old/'inputs'),source_model_sha256=sha(old/'inputs/input_model.mjb'),
        registration=str(output/'registration.json'),registration_sha256=sha(output/'registration.json'),
        added_static_bodies=len(new_bodies),added_main_stem_colliders=len(stem_names),
        added_fruit_instances=len(fruits)-len(existing_fruits),plant_joints_created=0,mesh_files_created=0,
        original_robot_and_colliders_preserved=True,physical_detachment_enabled=False)
    meta.update(geometric_harvest=policy,geometric_fruit_names=fruits,static_route_registration=evidence)
    meta['active_window']['frozen_body_names']+=new_bodies
    counts={k:int(getattr(model,k)) for k in ('nq','nv','nbody','ngeom','neq')}
    meta['active_window']['reduced_counts']=counts;meta['active_window']['static_route_registration']=evidence
    changed={'input_scene.xml','input_model.mjb','initial_state.npz','source_export.json','geometric_harvest.json'}
    link_manifest_assets(old/'inputs',inputs,set(proof['input_files'])-changed)
    if not (inputs/'sampled_plan.json').exists():(inputs/'sampled_plan.json').symlink_to((old/'inputs/sampled_plan.json').resolve())
    write(inputs/'source_export.json',meta);write(inputs/'geometric_harvest.json',policy)
    initial=dict(np.load(old/'inputs/initial_state.npz',allow_pickle=False));d=mj.MjData(model);d.qpos[:]=initial['qpos'];mj.mj_kinematics(model,d)
    initial.update(body_positions=d.xpos.copy(),body_quaternions_wxyz=d.xquat.copy(),body_names=np.asarray([model.body(i).name for i in range(model.nbody)]))
    np.savez_compressed(inputs/'initial_state.npz',**initial)
    proof.update(source_body_to_native_name=mapping,geometric_harvest=policy,static_route_registration=evidence,model_counts=counts)
    for name in sorted(set(proof['input_files'])|changed):proof['input_files'][name]=dict(path=str(inputs/name),sha256=sha(inputs/name))
    write(inputs/'inputs.json',proof)
    visual_meta=read(old/'model/export.json');visual_meta.update(model_sha256=sha(up/'model/scene.xml'),static_route_registration=evidence)
    write(up/'model/export.json',visual_meta)
    (up/'model/source_plan.json').symlink_to((old/'model/source_plan.json').resolve())
    for name in ('sensor_catalog.json','inventory.json'):(up/name).symlink_to((old/name).resolve())
    revised=copy.deepcopy(cfg);revised.update(end_station_m=reg['end_station_m'],inputs=str(inputs),plants=cfg['plants']+new_sources,
        geometric_harvest=policy,static_route_registration=evidence)
    write(up/'patrol_assets.json',revised)
    down=output/'down';down.mkdir()
    for name in ('model','sensor_catalog.json','inventory.json'):(down/name).symlink_to(up/name)
    dest=down/'inputs';dest.mkdir();old_down=Path(reg['profiles']['down']);down_cfg=read(old_down/'patrol_assets.json')
    for f in inputs.iterdir():
        if f.name not in ('initial_state.npz','inputs.json'):(dest/f.name).symlink_to(f)
    saved=dict(np.load(old_down/'inputs/initial_state.npz',allow_pickle=False));d.qpos[:]=saved['qpos'];mj.mj_kinematics(model,d)
    saved.update(body_positions=d.xpos.copy(),body_quaternions_wxyz=d.xquat.copy(),body_names=initial['body_names'])
    np.savez_compressed(dest/'initial_state.npz',**saved)
    down_proof=copy.deepcopy(proof)
    for name in down_proof['input_files']:down_proof['input_files'][name]=dict(path=str(dest/name),sha256=sha(dest/name))
    write(dest/'inputs.json',down_proof)
    down_cfg.update(end_station_m=reg['end_station_m'],inputs=str(dest),geometric_harvest=policy,static_route_registration=evidence)
    first_down=down_cfg['plants'][0]
    down_cfg['plants']+=[dict(s,assembly=first_down['assembly'],camera_calibration=first_down['camera_calibration']) for s in new_sources]
    write(down/'patrol_assets.json',down_cfg)
    for path in (up,down):
        subprocess.run([str(ROOT/'nvidia-sim/.newton-venv/bin/python'),str(HERE/'newton_scene_chunks.py'),str(path),str(path/'scene_stream')],check=True)
    write(output/'patrol_assets.json',dict(schema='s300-patrol-start-modes-v1',start_modes=dict(up=str(up),down=str(down)),default_start_mode='up',training_eligible=False))
    result=dict(assets=str(output),end_station_m=reg['end_station_m'],plants=reg['chosen_plants'],model_counts=counts,
        **evidence,harvest_success=None,route_execution_still_requires_validation=True)
    write(output/'route_preparation.json',result);return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('reference',type=Path);p.add_argument('output',type=Path,nargs='?')
    p.add_argument('--end-station',type=float);p.add_argument('--compile-registration',action='store_true')
    a=p.parse_args()
    if a.compile_registration:result=compile_registration(a.reference)
    else:
        if a.output is None or a.end_station is None:p.error('Output and --end-station required')
        result=prepare(a.reference,a.output,a.end_station)
    print(json.dumps(result,indent=2,ensure_ascii=False))
