"""Small headless comparison of identical neighbouring plant collision geometry."""
import argparse,copy,json,tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation
from build_glb_physics import build
from robot_engine import RobotEngine


def strip(root):
    for parent in root.iter():
        for g in list(parent.findall('geom')):
            if int(g.get('contype','1'))==0 and int(g.get('conaffinity','1'))==0:parent.remove(g)
    used={g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
    for g in root.iter('geom'):g.attrib.pop('material',None)
    asset=root.find('asset')
    for a in list(asset):
        if a.tag!='mesh' or a.get('name') not in used:asset.remove(a)
    return root


def graft(root, source, prefix, destination, yaw, anchor, stem=True):
    original=source.find(".//body[@name='STEM_MainStem_00']") if stem else source.find(".//body[@name='TRUSS_Truss_01_Peduncle_00']")
    branch=copy.deepcopy(original)
    if stem:
        rot=Rotation.from_euler('z',yaw,degrees=True)
        pos=np.fromstring(branch.get('pos'),sep=' ')
        q=np.fromstring(branch.get('quat','1 0 0 0'),sep=' ')
        branch.set('pos',' '.join(map(str,np.array(destination)+rot.apply(pos-anchor))))
        branch.set('quat',' '.join(map(str,(rot*Rotation.from_quat(q[[1,2,3,0]])).as_quat()[[3,0,1,2]])))
    names={b.get('name') for b in branch.iter('body')}
    meshes={g.get('mesh') for g in branch.iter('geom') if g.get('mesh')}
    for a in source.find('asset'):
        if a.get('name') in meshes:
            a=copy.deepcopy(a);a.set('name',prefix+a.get('name'));root.find('asset').append(a)
    for node in branch.iter():
        if node.get('name'):node.set('name',prefix+node.get('name'))
        if node.get('mesh'):node.set('mesh',prefix+node.get('mesh'))
    for c in source.find('contact'):
        if (c.get('body1') in names and c.get('body2') in names) or (not stem and ((c.get('body1') in names and c.get('body2','').startswith('STEM_')) or (c.get('body2') in names and c.get('body1','').startswith('STEM_')))):
            c=copy.deepcopy(c)
            for key in ['name','body1','body2']:
                if c.get(key) and (key=='name' or c.get(key) in names):c.set(key,prefix+c.get(key))
            root.find('contact').append(c)
    return branch


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--seconds',type=float,default=10);ap.add_argument('--reuse-models',type=Path);ap.add_argument('--idle',action='store_true');args=ap.parse_args()
    out=args.output;out.mkdir(parents=True,exist_ok=False)
    base=Path('/root/docker_share/mujoko_debugging_data')
    source=base/'20260927_stem_obstacle_scene';layout=json.loads((base/'20260927_robot_side_dense_trusses/scene.json').read_text())
    selected=sorted(layout['placements'],key=lambda x:np.linalg.norm(np.array(x['position'])-[-.75,.8,.32]))[:4]
    if not args.reuse_models:
        roots=[]
        for plant in selected:
            assembled=None
            for index,t in enumerate(plant['trusses']):
                with tempfile.TemporaryDirectory(prefix='elastic_probe_') as tmp:
                    part=Path(tmp)/'part'
                    build(Path(t['source_glb']),part,y_deg=t['y_deg'],segment=t['stem_segment'],stem_fraction=t['stem_fraction'],remove_fruits=t['remove_fruits'],fruit_offsets={int(k):v for k,v in t['fruit_offsets'].items()},rachis_stiffness_scale=t['rachis_stiffness_scale'],truss_scale=.5)
                    r=strip(ET.parse(part/'model.xml').getroot())
                    if assembled is None:assembled=r
                    else:
                        branch=graft(assembled,r,f'truss{index}__',None,None,None,False)
                        assembled.find(f".//body[@name='STEM_MainStem_{t['stem_segment']:02d}']").append(branch)
            roots.append(assembled)
            print('prepared plant',plant['id'],'trusses',len(plant['trusses']),flush=True)
        source_root=strip(ET.parse(source/'model.xml').getroot())
        model=mj.MjModel.from_xml_string(ET.tostring(source_root,encoding='unicode'));data=mj.MjData(model);mj.mj_forward(model,data)
        bid=model.body('STEM_MainStem_00').id;gid=next(i for i in range(model.ngeom) if model.geom_bodyid[i]==bid and model.geom_type[i]==mj.mjtGeom.mjGEOM_CAPSULE)
        anchor=data.geom_xpos[gid]+data.geom_xmat[gid].reshape(3,3)[:,2]*model.geom_size[gid,1]
        results=[]
        for count in [1,4]:
            root=copy.deepcopy(source_root)
            ids={p['id'] for p in selected[:count]}
            for g in list(root.find('worldbody').findall('geom')):
                if g.get('name','').startswith('neighbor_stem_collision_') and int(g.get('name').split('_')[-2]) in ids:root.find('worldbody').remove(g)
            for plant,r in zip(selected[:count],roots[:count]):
                prefix=f"bg{plant['id']}__"
                branch=graft(root,r,prefix,plant['position'],plant['yaw_deg'],anchor)
                root.find('worldbody').append(branch)
            for mode in ['elastic','fixed']:
                r=copy.deepcopy(root)
                if mode=='fixed':
                    for b in r.iter('body'):
                        if b.get('name','').startswith('bg'):
                            for j in list(b.findall('joint')):b.remove(j)
                path=out/f'{count}_{mode}.xml';ET.ElementTree(r).write(path,encoding='unicode')
                m=mj.MjModel.from_xml_path(str(path));mj.mj_saveModel(m,str(path.with_suffix('.mjb')))
                print('compiled',count,mode,m.nv,flush=True)
    results=[]
    for count in [1,4]:
        if args.reuse_models:
            for mode in ['fixed','elastic']:
                r=ET.parse(args.reuse_models/f'{count}_{mode}.xml').getroot()
                # Restore only authored adjacent parent-stem/branch exclusions.
                contact=r.find('contact')
                pairs={frozenset([c.get('body1'),c.get('body2')]) for c in contact}
                for parent in r.iter('body'):
                    for child in parent.findall('body'):
                        if parent.get('name','').startswith('bg') and 'STEM_MainStem_' in parent.get('name','') and child.get('name','').endswith('TRUSS_Truss_01_Peduncle_00'):
                            pair=frozenset([parent.get('name'),child.get('name')])
                            if pair not in pairs:ET.SubElement(contact,'exclude',body1=parent.get('name'),body2=child.get('name'));pairs.add(pair)
                path=out/f'{count}_{mode}.xml';ET.ElementTree(r).write(path,encoding='unicode')
                m=mj.MjModel.from_xml_path(str(path));mj.mj_saveModel(m,str(path.with_suffix('.mjb')))
        models=[mj.MjModel.from_binary_path(str(out/f'{count}_{mode}.mjb')) for mode in ['fixed','elastic']]
        datas=[mj.MjData(m) for m in models]
        for m,d in zip(models,datas):mj.mj_forward(m,d)
        assert [models[0].geom(i).name for i in range(models[0].ngeom)]==[models[1].geom(i).name for i in range(models[1].ngeom)]
        error=max(float(np.max(abs(datas[0].geom_xpos-datas[1].geom_xpos))),float(np.max(abs(datas[0].geom_xmat-datas[1].geom_xmat))))
        assert error<1e-10,error
        for field in ['geom_type','geom_size','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp']:
            np.testing.assert_array_equal(getattr(models[0],field),getattr(models[1],field))
        print('GEOMETRY',count,'error',error,flush=True)
        for repeat in range(2):
            for mode in (['fixed','elastic'] if repeat==0 else ['elastic','fixed']):
                trace=base/'20260927_dense_house_harvest_test/targets/Tomato_01/physics'
                e=RobotEngine(out/f'{count}_{mode}.mjb',trace/'replay_assets/initial_trace.json',240,reference=source/'reference.json',target='Tomato_01')
                commands=json.loads((trace/'candidates/predicted_00000/trace.json').read_text());e.commands=np.array([x['command'] for x in commands]);e.ts=np.arange(len(commands))/60
                if args.idle:e.commands[:]=e.initial
                newids=[i for i in range(e.model.nbody) if (e.model.body(i).name or '').startswith('bg')]
                start=e.data.xpos[newids].copy();initial_pen=max([0.]+[-float(c.dist) for c in e.data.contact]);maxmotion=[0.]
                maxpen=[initial_pen];pairs=set()
                def measure(m,d):
                    maxmotion[0]=max(maxmotion[0],float(np.linalg.norm(d.xpos[newids]-start,axis=1).max()))
                    if len(d.contact):
                        maxpen[0]=max(maxpen[0],-float(d.contact.dist.min()))
                        for c in d.contact:
                            if c.dist<-.0005:pairs.add((m.geom(c.geom1).name,m.geom(c.geom2).name))
                result=e.rollout(seconds=args.seconds,record=False,on_step=measure)
                row=dict(background_trusses=sum((e.model.body(i).name or "").startswith("bg") and (e.model.body(i).name or "").endswith("TRUSS_Truss_01_Peduncle_00") for i in range(e.model.nbody)),geometry_error=error,all_contact_max_penetration_m=maxpen[0],overlap_pairs=sorted(pairs),benchmark_valid=not result["unstable"] and maxpen[0]<=.0005,count=count,mode=mode,repeat=repeat,nv=e.model.nv,initial_penetration_m=initial_pen,max_background_motion_m=maxmotion[0],**result);results.append(row)
                (out/'results.json').write_text(json.dumps(results,indent=2));print('BENCH',count,mode,repeat,round(result['rollout_wall_s'],3),result['glb_physics_valid'],maxmotion[0],flush=True)
    (out/'setup.json').write_text(json.dumps(dict(selected=selected,seconds=args.seconds,idle=args.idle,scope='1 and 4 nearest background plants, see results background_trusses; same collision geometry fixed versus articulated, original preload method. Visuals stripped only for headless benchmark; remaining background stems fixed. Not a full-house validation.'),indent=2))
    summary=[]
    for count in [1,4]:
        row={'background_plants':count,'background_trusses':next(r['background_trusses'] for r in results if r['count']==count)}
        for mode in ['fixed','elastic']:
            runs=[r for r in results if r['count']==count and r['mode']==mode]
            row[mode]={'mean_wall_s':float(np.mean([r['rollout_wall_s'] for r in runs])),
                       'mean_physics_s':float(np.mean([r['physics_s'] for r in runs])),
                       'nv':runs[0]['nv'], 'valid_runs':sum(r['benchmark_valid'] for r in runs),
                       'max_penetration_mm':1000*max(r['all_contact_max_penetration_m'] for r in runs),
                       'max_background_motion_mm':1000*max(r['max_background_motion_m'] for r in runs)}
        row['elastic_over_fixed']=row['elastic']['mean_wall_s']/row['fixed']['mean_wall_s']
        summary.append(row)
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    rows=''.join(f"<tr><td>{r['background_plants']} / {r['background_trusses']}</td><td>{r['fixed']['mean_wall_s']:.3f}</td><td>{r['elastic']['mean_wall_s']:.3f}</td><td>{r['elastic_over_fixed']:.2f}</td><td>{r['fixed']['max_penetration_mm']:.3f} / {r['elastic']['max_penetration_mm']:.3f}</td></tr>" for r in summary)
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>주변 식물 탄성 비용 비교</title><h1>주변 식물 고정 / 탄성 비교</h1>'
        +f'<p>240Hz, {("로봇 정지" if args.idle else "동일 로봇 명령")} {args.seconds:g}초 구간, 각 2회. GUI/촬영/저장 제외. 초기 형상 일치 확인. 원래 타깃 식물은 양쪽 모두 탄성.</p>'
        +'<table border="1"><tr><th>주변 식물 / 송이</th><th>고정 실행 초</th><th>탄성 실행 초</th><th>시간 배율</th><th>최대 침투 mm 고정 / 탄성</th></tr>'+rows+'</table>'
        +'<p>0.5mm 초과 침투는 물리 기준 실패. 이 수치는 처리 비용 진단이며 정상 수확 성능 검증이 아닙니다. 전체 68개 탄성화, 런타임 전환, 주변 잎 충돌은 검증하지 않았습니다.</p><a href="results.json">원본 측정</a> <a href="setup.json">구성</a>',encoding='utf-8')


if __name__=='__main__':main()
