"""USD-derived modular plant preview. Experimental physics, not collection-ready."""
import argparse, hashlib, json, sys, time, copy
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'nvidia-sim/rl'))
from elastic_geometry import tube_centerline, resample_rod
from convert_model import nums
EXISTING = ROOT/'mujoco-benchmark/models/plant_mujoco_optimized.xml'
SOURCE = ROOT/'nvidia-sim/env_usd/tomato_greenhouse_upgraded_with_stems_and_clusters_v2_isaac.usd'


def project(points, point):
    a, b = points[:-1], points[1:]
    delta = b-a
    lengths = np.linalg.norm(delta, axis=1)
    f = np.clip(np.sum((point-a)*delta, axis=1)/lengths**2, 0, 1)
    hits = a+f[:, None]*delta
    i = np.argmin(np.linalg.norm(hits-point, axis=1))
    return float(np.sum(lengths[:i])+f[i]*lengths[i]), hits[i], delta[i]/lengths[i]


def along(points, distance):
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.r_[0., np.cumsum(lengths)]
    if not 0 <= distance <= cumulative[-1]:
        raise ValueError('부착 위치가 주줄기 범위를 벗어났습니다')
    i = min(np.searchsorted(cumulative, distance, side='right')-1, len(lengths)-1)
    tangent = (points[i+1]-points[i])/lengths[i]
    return points[i]+(distance-cumulative[i])*tangent, tangent


def transport(a, b):
    cross = np.cross(a, b)
    if np.linalg.norm(cross) < 1e-12:
        if np.dot(a, b) < 0: raise ValueError('반대 방향 주줄기 프레임은 지원하지 않습니다')
        return np.eye(3)
    return Rotation.from_rotvec(cross/np.linalg.norm(cross)*np.arctan2(np.linalg.norm(cross),np.dot(a,b))).as_matrix()


def tipward_rotation(direction, tangent):
    """Rigidly rotate an initially rootward branch to the tipward hemisphere."""
    direction=np.asarray(direction,dtype=float);direction=direction/np.linalg.norm(direction)
    tangent=np.asarray(tangent,dtype=float);tangent=tangent/np.linalg.norm(tangent)
    axial=float(np.dot(direction,tangent))
    target=direction-2*min(axial,0.)*tangent
    return transport(direction,target)


def require_tipward(direction,tangent):
    cosine=float(np.dot(direction,tangent)/(np.linalg.norm(direction)*np.linalg.norm(tangent)))
    if not np.isfinite(cosine) or cosine < -1e-10:
        raise ValueError('부착 금지: 송이 시작 가지가 주줄기 뿌리 방향으로 뻗습니다')
    return cosine


def surface_attachment(axis_point, tangent, stem_radius, branch_radius, direction):
    """Place the branch root cap tangent to the stem, pointing outward.

    axis_point is on the straight part of the selected stem capsule.
    The branch's first capsule has a hemispherical cap of branch_radius.
    """
    tangent=np.asarray(tangent,dtype=float)
    tangent=tangent/np.linalg.norm(tangent)
    direction=np.asarray(direction,dtype=float)
    radial=direction-tangent*np.dot(direction,tangent)
    length=np.linalg.norm(radial)
    if not np.isfinite(length) or length<1e-10:
        raise ValueError('송이 시작 방향이 주줄기와 평행하여 표면 부착 방향을 정할 수 없습니다')
    if stem_radius<=0 or branch_radius<=0:
        raise ValueError('부착 반경은 양수여야 합니다')
    normal=radial/length
    surface=np.asarray(axis_point)+normal*stem_radius
    root=surface+normal*branch_radius
    return root,surface,normal


class Source:
    def __init__(self, path=SOURCE, scale=.5):
        self.path=Path(path); self.scale=scale
        self.stage=Usd.Stage.Open(str(path)); self.cache=UsdGeom.XformCache()
        self.plants=self.stage.GetPrimAtPath('/World/Plants')
        if not self.plants: raise ValueError('USD에 /World/Plants가 없습니다')

    def mesh(self, path):
        p=self.stage.GetPrimAtPath(str(path)+'/Mesh')
        if not p or not p.IsA(UsdGeom.Mesh): raise ValueError('메시 없음: '+str(path))
        m=UsdGeom.Mesh(p); v=np.asarray(m.GetPointsAttr().Get(),dtype=float)
        t=np.asarray(self.cache.GetLocalToWorldTransform(p))
        v=(v@t[:3,:3]+t[3,:3])*self.scale
        counts=np.asarray(m.GetFaceVertexCountsAttr().Get()); indices=np.asarray(m.GetFaceVertexIndicesAttr().Get())
        faces=[]; offset=0
        for n in counts:
            f=indices[offset:offset+n]; faces.extend([[f[0],f[i],f[i+1]] for i in range(1,n-1)]);offset+=n
        return v,np.asarray(faces)

    def tube(self,path,sides):
        return tube_centerline(self.mesh(path)[0],sides)

    def catalog(self):
        records=[]
        for plant in self.plants.GetChildren():
            for truss in plant.GetChildren():
                if not truss.GetName().startswith('Truss_'): continue
                children=list(truss.GetChildren())
                fruit=[p for p in children if p.GetName().startswith('Fruit_')]
                refs=[]
                for p in children:
                    refs.extend(str(x.assetPath)+str(x.primPath) for x in p.GetMetadata('references').GetAddedOrExplicitItems()) if p.GetMetadata('references') else None
                records.append(dict(id=plant.GetName()+'__'+truss.GetName(),path=str(truss.GetPath()),plant=str(plant.GetPath()),fruit_count=len(fruit),prototype_signature=hashlib.sha256('|'.join(sorted(refs)).encode()).hexdigest()))
        return records


def assemble(source, record, shift, yaw, tilt, output):
    """Preserve authored truss geometry relative to its attachment and stem tangent."""
    stem_path=record['plant']+'/Stem/MainStem_Long_Gentle_Curve'
    main,rmain=source.tube(stem_path,24)
    truss=record['path']; ped,rped=source.tube(truss+'/Truss_01_Peduncle',14)
    s0, anchor0, tangent0=project(main,ped[0])
    import render_backend
    import mujoco as mj
    oldtree=ET.parse(EXISTING); oldroot=oldtree.getroot()
    oldmodel=mj.MjModel.from_xml_path(str(EXISTING)); olddata=mj.MjData(oldmodel);mj.mj_forward(oldmodel,olddata)
    oldmain=copy.deepcopy(oldroot.find("worldbody/body[@name='STEM_MainStem_00']"))
    for node in list(oldmain.iter('body')):
        for child in list(node.findall('body')):
            if child.get('name','').startswith('TRUSS_'):node.remove(child)
    mainnodes=[n for n in oldmain.iter('body') if n.get('name','').startswith('STEM_MainStem_')]
    segments=[]; stem_radii=[]
    for node in mainnodes:
        bid=oldmodel.body(node.get('name')).id
        gids=[g for g in range(oldmodel.ngeom) if oldmodel.geom_bodyid[g]==bid and oldmodel.geom_type[g]==mj.mjtGeom.mjGEOM_CAPSULE]
        if len(gids)!=1:raise ValueError('기존 주줄기 캡슐 구조 변경')
        g=gids[0];axis=olddata.geom_xmat[g].reshape(3,3)[:,2]*oldmodel.geom_size[g,1]
        segments.append([olddata.geom_xpos[g]-axis,olddata.geom_xpos[g]+axis]);stem_radii.append(float(oldmodel.geom_size[g,0]))
    # Preserve the existing stem exactly; transfer the source attachment's relative arclength.
    original_length=np.linalg.norm(np.diff(main,axis=0),axis=1).sum()
    main=np.array([segments[0][0]]+[pair[1] for pair in segments])
    target_length=np.linalg.norm(np.diff(main,axis=0),axis=1).sum()
    target_s=s0/original_length*target_length+shift
    anchor,tangent=along(main,target_s)
    sampled_ped,sampled_radii=resample_rod(ped,rped,5)
    start_direction=sampled_ped[1]-sampled_ped[0]
    frame=transport(tangent0,tangent)
    frame=tipward_rotation(frame@start_direction,tangent)@frame
    yaw_rot=Rotation.from_rotvec(tangent*np.deg2rad(yaw)).as_matrix()
    outward=frame@(ped[-1]-ped[0]);tilt_axis=np.cross(tangent,outward);tilt_axis/=np.linalg.norm(tilt_axis)
    rot=Rotation.from_rotvec(tilt_axis*np.deg2rad(tilt)).as_matrix()@yaw_rot@frame
    tipward_cosine=require_tipward(rot@start_direction,tangent)
    # Match the actual generated first capsule, including its rounded cap.
    sampled_ped,sampled_radii=resample_rod(ped,rped,5)
    cumulative=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(main,axis=0),axis=1))]
    attachment_segment=min(int(np.searchsorted(cumulative,target_s,side='right')-1),len(segments)-1)
    root_radius=float(max(sampled_radii[:2]))
    root_position,surface_point,surface_normal=surface_attachment(
        anchor,tangent,stem_radii[attachment_segment],root_radius,
        rot@(sampled_ped[1]-sampled_ped[0]))
    source_root=ped[0].copy()
    origin=np.zeros(3)
    tf=lambda v:(np.asarray(v)-source_root)@rot.T+root_position
    main=main-origin; ped=tf(ped)
    root=ET.Element('mujoco',model='usd_modular_plant_preview')
    ET.SubElement(root,'compiler',angle='radian',autolimits='true')
    opt=ET.SubElement(root,'option',timestep=str(1/240),integrator='implicitfast',solver='Newton',iterations='64',gravity='0 0 -9.81')
    ET.SubElement(opt,'flag',autoreset='disable')
    default=ET.SubElement(root,'default');ET.SubElement(default,'geom',friction='.5 .005 .0001',solref='.008 1',density='700')
    ET.SubElement(default,'joint',armature='.0005',limited='true',range='-.35 .35')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='960',offheight='720')
    ET.SubElement(default,'default',{'class':'new_truss'})
    asset=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody');contacts=ET.SubElement(root,'contact')
    ET.SubElement(world,'light',pos='0 0 3',dir='0 0 -1',diffuse='.9 .9 .9',ambient='.35 .35 .35')
    nodes=[]; positions={}; endpoints={};attachments=[]
    def chain(name,points,radii,segments,parent=None,k=2,d=.15):
        points,radii=resample_rod(points,radii,segments); result=[]
        for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
            owner=parent if i==0 else result[-1]
            pos=positions[owner.get('name')] if owner is not None else np.zeros(3)
            n=ET.SubElement(owner if owner is not None else world,'body',name=f'{name}_{i:02d}',pos=nums(a-pos))
            positions[n.get('name')]=a;endpoints[n.get('name')]=b
            if owner is not None:
                for j,axis in enumerate(np.eye(3)):
                    ET.SubElement(n,'joint',name=f'{name}_{i:02d}_{j}',type='hinge',axis=nums(axis),stiffness=str(k),damping=str(d))
                ET.SubElement(contacts,'exclude',body1=owner.get('name'),body2=n.get('name'))
            ET.SubElement(n,'geom',name=n.get('name')+'_collision',type='capsule',fromto=nums([np.zeros(3),b-a]),size=str(float(max(radii[i:i+2]))),rgba='.16 .32 .04 1')
            result.append(n);nodes.append(n)
        return result
    def nearest(chain_nodes,p):
        return min(chain_nodes,key=lambda n:np.linalg.norm(p-(positions[n.get('name')]+endpoints[n.get('name')])/2))
    # Copy all original stem/leaf geoms, masses, joints and materials unchanged.
    inherited=ET.SubElement(default,'default',{'class':'existing_stem'})
    for child in oldroot.find('default'):inherited.append(copy.deepcopy(child))
    oldmain.set('childclass','existing_stem');world.append(oldmain)
    mesh_names={g.get('mesh') for g in oldmain.iter('geom') if g.get('mesh')}
    for item in oldroot.find('asset'):
        if item.tag!='mesh' or item.get('name') in mesh_names:asset.append(copy.deepcopy(item))
    retained={n.get('name') for n in oldmain.iter('body')}
    for item in oldroot.find('contact'):
        if item.get('body1') in retained and item.get('body2') in retained:contacts.append(copy.deepcopy(item))
    mains=mainnodes
    for n in mains:
        bid=oldmodel.body(n.get('name')).id
        positions[n.get('name')]=olddata.xpos[bid].copy()
        endpoints[n.get('name')]=positions[n.get('name')]
    parent=mains[attachment_segment]
    support=chain('Peduncle',ped,rped,5,parent,25,1)
    # Existing parent has a rotated frame; generated truss frames are world-aligned.
    bid=oldmodel.body(parent.get('name')).id;pr=olddata.xmat[bid].reshape(3,3)
    support[0].set('pos',nums(pr.T@(ped[0]-olddata.xpos[bid])))
    q=Rotation.from_matrix(pr.T).as_quat();support[0].set('quat',nums(q[[3,0,1,2]]))
    support[0].set('childclass','new_truss')
    rach,rr=source.tube(truss+'/Rachis',14);rach=tf(rach)
    rachis=chain('Rachis',rach,rr,14,support[-1])
    attachments.append(dict(name='peduncle_rachis',gap_m=float(np.linalg.norm(ped[-1]-rach[0]))))
    fruit_records=[]
    for fp in source.stage.GetPrimAtPath(truss).GetChildren():
        if not fp.GetName().startswith('Fruit_'):continue
        suffix=fp.GetName().split('_')[-1]
        prox,rp=source.tube(truss+'/Pedicel_proximal_'+suffix,12);dist,rd=source.tube(truss+'/Pedicel_distal_'+suffix,12)
        prox,dist=tf(prox),tf(dist)
        pc=chain('Proximal_'+suffix,prox,rp,3,nearest(rachis,prox[0]),.25,.02)
        dc=chain('Distal_'+suffix,dist,rd,1,pc[-1],.25,.02)
        vertices,faces=source.mesh(str(fp.GetPath()));vertices=tf(vertices);center=(vertices.max(0)+vertices.min(0))/2
        radius=float(np.max(np.linalg.norm(vertices-center,axis=1)))
        owner=dc[-1];n=ET.SubElement(owner,'body',name='Tomato_'+suffix,pos=nums(center-positions[owner.get('name')]))
        ET.SubElement(n,'geom',name='Fruit_'+suffix+'_collision',type='mesh',mesh='mesh_'+fp.GetName(),rgba='0 0 0 0',group='3',mass='.025')
        for obj in source.stage.GetPrimAtPath(truss).GetChildren():
            name=obj.GetName()
            if name!=fp.GetName() and not (name.startswith('Calyx_'+suffix+'_') or name=='Calyx_Hub_'+suffix):continue
            v,f=source.mesh(str(obj.GetPath()));v=tf(v)-center
            meshname='mesh_'+name
            ET.SubElement(asset,'mesh',name=meshname,vertex=nums(v),face=nums(f))
            ET.SubElement(n,'geom',name='visual_'+name,type='mesh',mesh=meshname,contype='0',conaffinity='0',density='0',group='2',rgba='.75 .035 .015 1' if name.startswith('Fruit') else '.16 .32 .04 1')
        attachments.append(dict(name='pedicel_'+suffix,gap_m=float(np.linalg.norm(prox[-1]-dist[0]))))
        fruit_records.append(dict(id='Tomato_'+suffix,center_world_m=center.tolist(),collision_radius_m=radius))
    output.mkdir(parents=True,exist_ok=False)
    ET.indent(root);ET.ElementTree(root).write(output/'scene.xml',encoding='unicode')
    meta=dict(schema='usd_attachment_preview_v1',source_usd=str(source.path),source_record=record,scale=source.scale,shift_m=shift,yaw_deg=yaw,tilt_deg=tilt,source_arclength_m=s0,attachment_arclength_m=target_s,existing_stem_model=str(EXISTING),existing_stem_sha256=hashlib.sha256(EXISTING.read_bytes()).hexdigest(),attachment_world_m=surface_point.tolist(),branch_root_world_m=root_position.tolist(),attachment_surface_normal=surface_normal.tolist(),attachment_parent=parent.get('name'),attachment_root_radius_m=root_radius,attachment_rule='capsule_surface_tipward_v3',attachment_tipward_cosine=tipward_cosine,stem_tangent=tangent.tolist(),rotation_matrix=rot.tolist(),truss_world_translation_m=(root_position-rot@source_root).tolist(),fruits=fruit_records,attachments=attachments,collection_ready=False,limitations=['Experimental capsule/convex fruit physics; not equivalent to existing collection model','Existing main stem and leaves preserved; no robot, breaking or truss hairs/empty pedicels','Authored USD attachment plus bounded perturbation; not a measured biological distribution','No candidate execution or action14/RGB-D dataset generated'])
    (output/'scene.json').write_text(json.dumps(meta,indent=2,ensure_ascii=False))
    return meta


def validate_render(folder):
    import render_backend
    import mujoco as mj
    from PIL import Image
    m=mj.MjModel.from_xml_path(str(folder/'scene.xml'));d=mj.MjData(m);mj.mj_forward(m,d)
    initial=d.xpos.copy();d.qfrc_applied[:]=d.qfrc_bias.copy()
    initial_contacts=[dict(geom1=m.geom(c.geom1).name,geom2=m.geom(c.geom2).name,penetration_m=float(-c.dist)) for c in d.contact if c.dist < -.0005]
    for _ in range(480):mj.mj_step(m,d)
    stable=bool(np.isfinite(d.qpos).all() and np.max(np.abs(d.qvel))<100 and d.time>1.99)
    movement=float(np.max(np.linalg.norm(d.xpos-initial,axis=1)))
    meta=json.loads((folder/'scene.json').read_text());centers=np.array([f['center_world_m'] for f in meta['fruits']])
    mj.mj_resetData(m,d);mj.mj_forward(m,d)
    cam=mj.MjvCamera();cam.lookat[:]=centers.mean(0);cam.distance=max(.65,float(np.ptp(centers,axis=0).max()*2.2));cam.azimuth=125;cam.elevation=-12
    renderer=mj.Renderer(m,height=720,width=960);renderer.update_scene(d,camera=cam);Image.fromarray(renderer.render()).save(folder/'preview.png')
    lo=np.min(d.geom_xpos-m.geom_rbound[:,None],axis=0);hi=np.max(d.geom_xpos+m.geom_rbound[:,None],axis=0)
    cam.lookat[:]=(lo+hi)/2;cam.distance=float(np.linalg.norm(hi-lo)*1.5)
    renderer.update_scene(d,camera=cam);Image.fromarray(renderer.render()).save(folder/'overview.png');renderer.close()
    result=dict(stable_2s_with_frozen_gravity_preload=stable,max_body_displacement_m=movement,nbody=m.nbody,nv=m.nv,initial_max_penetration_m=float(max([0.]+[-c.dist for c in d.contact])),candidate_physics_validated=False,initial_overlap_check_passed=not initial_contacts,initial_overlap_pairs=initial_contacts)
    (folder/'validation.json').write_text(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description='USD 송이 부착 장면 미리보기 생성기 (학습 수집 전 단계)')
    p.add_argument('--source',type=Path,default=SOURCE);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--count',type=int,default=6);p.add_argument('--seed',type=int,default=0)
    p.add_argument('--plant',default='TOMATO_STEM_L_00');p.add_argument('--truss',choices=['Truss_01','Truss_02','Truss_03'])
    p.add_argument('--shift-range',type=float,default=.08);p.add_argument('--yaw-range',type=float,default=10);p.add_argument('--tilt-range',type=float,default=5)
    p.add_argument('--shift',type=float,help='고정 부착 이동(m), 범위 샘플링 대신 사용')
    p.add_argument('--yaw',type=float,help='고정 주줄기 축 회전(도)')
    p.add_argument('--tilt',type=float,help='고정 기울기(도)')
    p.add_argument('--render',action='store_true');args=p.parse_args()
    if any(v is not None and not np.isfinite(v) for v in [args.shift,args.yaw,args.tilt]):p.error('유한값을 입력하세요')
    if (args.yaw is not None and abs(args.yaw)>20) or (args.tilt is not None and abs(args.tilt)>10):p.error('yaw ±20°, tilt ±10° 이내')
    if args.count<1 or any(not np.isfinite(v) or v<0 for v in [args.shift_range,args.yaw_range,args.tilt_range]):p.error('개수와 변화 범위를 확인하세요')
    if args.yaw_range>20 or args.tilt_range>10:p.error('미리보기 회전 제한: yaw 20°, tilt 10° 이내')
    source=Source(args.source);catalog=source.catalog();choices=[r for r in catalog if r['plant'].endswith('/'+args.plant) and (not args.truss or r['path'].endswith('/'+args.truss))]
    if not choices:p.error('선택한 식물/송이가 없습니다')
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'catalog.json').write_text(json.dumps(catalog,indent=2))
    rng=np.random.default_rng(args.seed);rows=[];start=time.perf_counter()
    for i in range(args.count):
        record=choices[i%len(choices)];shift,yaw,tilt=rng.uniform(-1,1,3)*[args.shift_range,args.yaw_range,args.tilt_range]
        shift=args.shift if args.shift is not None else shift; yaw=args.yaw if args.yaw is not None else yaw; tilt=args.tilt if args.tilt is not None else tilt
        folder=args.output/f'scene_{i:04d}';meta=assemble(source,record,shift,yaw,tilt,folder)
        validation=validate_render(folder) if args.render else None
        rows.append(dict(scene_id=folder.name,source=record['id'],shift_m=shift,yaw_deg=yaw,tilt_deg=tilt,validation=validation))
        print(f'[장면 생성] {i+1}/{args.count} {record["id"]} 이동 {shift*1000:.1f}mm, 회전 {yaw:.1f}°',flush=True)
    (args.output/'manifest.json').write_text(json.dumps(dict(seed=args.seed,args={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),scenes=rows,elapsed_s=time.perf_counter()-start),indent=2))
    cards=''.join(f'<article><h2>{r["scene_id"]} · {r["source"]}</h2><p>이동 {r["shift_m"]*1000:.1f}mm / yaw {r["yaw_deg"]:.1f}° / tilt {r["tilt_deg"]:.1f}°</p><img src="{r["scene_id"]}/overview.png"><img src="{r["scene_id"]}/preview.png"><p><a href="{r["scene_id"]}/scene.json">장면 설정</a> · <a href="{r["scene_id"]}/validation.json">검사 결과</a></p></article>' for r in rows)
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>USD 송이 부착 장면</title><style>body{background:#18212b;color:#eee;font:16px sans-serif;margin:30px}main{display:grid;grid-template-columns:repeat(2,1fr);gap:20px}article{background:#263544;padding:16px}img{width:100%}a{color:#8ddcfe}</style><h1>USD 송이 부착 장면</h1><p>형태·연결 미리보기입니다. 기존 수집기의 물리 동등성 및 진입 경로는 아직 검증하지 않았습니다.</p><main>'+cards+'</main>')
    print('[완료]',args.output/'index.html')
if __name__=='__main__':main()
