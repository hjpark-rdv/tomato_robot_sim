"""Tomato_05 fixed-state, non-centred virtual D435 observations + existing actions.

No physics rollout or training. RGB contains no GT markers; GT is saved separately.
"""
import render_backend
import argparse,datetime,hashlib,json,shutil,time,html
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
from PIL import Image,ImageDraw
import mujoco as mj
from d435_capture import capture,optical_transform,HOME


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def project(point,world,k):
    optical=(np.asarray(point)-world[:3,3])@world[:3,:3]
    return (k@optical)[:2]/optical[2],float(optical[2])
def bbox(mask):
    y,x=np.where(mask)
    return [int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)] if len(x) else None


def views(nominal,center,k,seed):
    # Independent rotations/translations; no look-at operation.
    u=qmc.Sobol(6,scramble=True,seed=seed).random_base2(12)*2-1
    groups={i:[] for i in range(9)};outside=[]
    for index,row in enumerate(u):
        offset=row[:3]*[.05,.05,.025]
        angles=row[3:]*[20.,20.,5.] # camera local x/y tilt, z roll
        delta=np.eye(4);delta[:3,3]=offset;delta[:3,:3]=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
        pose=nominal@delta;uv,z=project(center,pose,k)
        item=dict(sobol_index=index,translation_camera_m=offset.tolist(),rotation_camera_xyz_deg=angles.tolist(),world_from_color=pose.tolist(),projected_target_center_px=uv.tolist(),target_optical_z_m=z)
        if 0<=uv[0]<640 and 0<=uv[1]<480:
            cell=min(2,int(uv[1]/160))*3+min(2,int(uv[0]/(640/3)));item['screen_cell']=cell;groups[cell].append(item)
        elif z>.1:item['screen_cell']=-1;outside.append(item)
    if any(len(g)<7 for g in groups.values()):raise RuntimeError('Insufficient coverage within declared camera bounds')
    uv,z=project(center,nominal,k)
    selected=[dict(sobol_index=None,translation_camera_m=[0]*3,rotation_camera_xyz_deg=[0]*3,world_from_color=nominal.tolist(),projected_target_center_px=uv.tolist(),target_optical_z_m=z,screen_cell=4,nominal=True)]
    for cell in range(9):selected.extend(groups[cell][:7])
    # Explicit negative observations, retained rather than silently dropped.
    selected.extend(outside[:8])
    for i,item in enumerate(selected):
        item['observation_id']=f'view_{i:04d}'
        item['split']='test' if item['screen_cell']==2 else 'validation' if item['screen_cell']==6 else 'train' if item['screen_cell']>=0 else 'visibility_test'
    return selected


def save_crops(folder,meta,center_px,diameter):
    rgb=Image.open(folder/'rgb.png');depth=np.load(folder/'depth_aligned_to_color_m.npy');out={}
    for name,factor in [('local',3),('context',6)]:
        side=max(16,int(np.ceil(diameter*factor)));cx,cy=center_px
        requested=[int(np.floor(cx-side/2)),int(np.floor(cy-side/2))];requested+=[requested[0]+side,requested[1]+side]
        box=[max(0,min(640,requested[0])),max(0,min(480,requested[1])),max(0,min(640,requested[2])),max(0,min(480,requested[3]))]
        if box[2]<=box[0] or box[3]<=box[1]:out[name]=dict(available=False,requested_xyxy=requested);continue
        rgb.crop(box).save(folder/f'crop_{name}_rgb.png');np.savez_compressed(folder/f'crop_{name}_depth.npz',depth_m=depth[box[1]:box[3],box[0]:box[2]])
        k=np.array(meta['K']);k[0,2]-=box[0];k[1,2]-=box[1]
        out[name]=dict(available=True,requested_xyxy=requested,bounds_xyxy=box,truncated=box!=requested,resized=False,K=k.tolist(),width_in_fruit_diameters=factor,center_source='GT fruit center projection; not a detector output')
    return out


def gallery(root,rows):
    cards=[]
    for r in rows:
        oid=r['observation_id'];a=r['target'];cards.append(f'<article data-state="{a["visibility"]}" data-split="{r["split"]}"><a href="observations/{oid}/rgb.png"><img loading="lazy" src="observations/{oid}/annotated_preview.jpg"></a><h3>{oid} <small>{r["split"]}</small></h3><p>{a["visibility"]} · 보이는 면적 {a["visible_pixels"]}px</p><p>중심 ({a["center_pixel_gt"][0]:.0f}, {a["center_pixel_gt"][1]:.0f}) · 가림 {a["occlusion_fraction_in_frame"] if a["occlusion_fraction_in_frame"] is not None else "—"}</p><div><a href="observations/{oid}/rgb.png">원본 RGB</a> · <a href="observations/{oid}/depth_preview.png">Depth</a> · <a href="observations/{oid}/observation.json">메타데이터</a></div></article>')
    counts=Counter(r['target']['visibility'] for r in rows)
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Tomato_05 · 관측 데이터</title><style>body{background:#f3f6f2;color:#193c31;font:14px/1.7 system-ui;margin:0;padding:32px;max-width:1600px;margin:auto}h1{font-size:30px;margin:10px 0}header{background:#173e33;color:white;padding:24px 30px;border-radius:12px;margin-bottom:22px}.notice{background:#e7eee1;padding:14px 20px;border-radius:8px;margin:16px 0}.toolbar{display:flex;gap:20px;margin:20px 0;flex-wrap:wrap}select{padding:8px;border:1px solid #b9cbbb;border-radius:6px}#grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(270px,1fr));gap:18px}article{background:#fff;border:1px solid #dae4d9;border-radius:10px;overflow:hidden;padding-bottom:16px}img{width:100%;display:block}h3,p,article div{margin:7px 15px}small{float:right;color:#79977f;font-size:10px}a{color:#298466}article p{font-size:12px}footer{margin:30px 0;font-size:12px;color:#648474}[hidden]{display:none}</style><header><small>FARMILY / OBSERVATION DATASET</small><h1>중앙에 고정하지 않은 Tomato_05 관측</h1><p>같은 초기 장면 · 가상 D435 이동 · 기존 경로 결과 재사용</p></header>'''
    page+=f'<div class="notice">{len(rows)}개 관측 · {html.escape(str(dict(counts)))}<br>초록 표시와 마스크는 시뮬레이터 GT입니다. 학습 RGB에는 그려져 있지 않습니다. 카메라만 가상 이동했으며 실제 로봇 도달성은 검증하지 않았습니다.</div><p><a href="coverage_preview.jpg">9개 구역 미리보기</a> · <a href="dataset.json">데이터셋 설명</a> · <a href="observations.jsonl">관측 목록</a> · <a href="actions.json">공유 경로·결과</a> · <a href="action_schema.json">카메라 기준 입력 형식</a></p>'
    page+='''<div class="toolbar"><label>가시성 <select id="state"><option value="all">전체</option><option>visible</option><option>partially_occluded</option><option>truncated</option><option>heavily_occluded</option><option>not_visible</option></select></label><label>평가 분할 <select id="split"><option value="all">전체</option><option>train</option><option>validation</option><option>test</option><option>visibility_test</option></select></label><span id="count"></span></div><div id="grid">'''+''.join(cards)+'''</div><footer>목표 위치 분포는 3×3 구역으로 나누어 수집했습니다. 테스트 분할은 시점 검증용이며 다른 토마토로의 일반화 검증이 아닙니다.</footer><script>function filter(){let n=0;document.querySelectorAll('article').forEach(c=>{c.hidden=!(document.getElementById('state').value==='all'||c.dataset.state===document.getElementById('state').value)||!(document.getElementById('split').value==='all'||c.dataset.split===document.getElementById('split').value);if(!c.hidden)n++});document.getElementById('count').textContent=n+'개 표시'}document.getElementById('state').onchange=filter;document.getElementById('split').onchange=filter;filter();</script></html>'''
    page=page.replace('Tomato_05',html.escape(rows[0]['target']['id']))
    (root/'index.html').write_text(page)


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--seed',type=int,default=42);p.add_argument('--output',type=Path);p.add_argument('--limit',type=int);p.add_argument('--postprocess-workers',type=int,default=8);args=p.parse_args()
    source=args.run.resolve();manifest=json.loads((source/'manifest.json').read_text());results=json.loads((source/'results.json').read_text());assets=source/'replay_assets'
    if mj.__version__!=manifest['mujoco']:raise RuntimeError('Matching MuJoCo required')
    target_name=manifest['target']
    if sha(assets/'model.mjb')!=manifest['model_sha256']:raise RuntimeError('model hash mismatch')
    for name in ('initial_trace.json','reference.json'):
        expected=manifest.get('asset_hashes',{}).get(name)
        if expected and sha(assets/name)!=expected:raise RuntimeError('Source asset hash mismatch: '+name)
    from robot_engine import RobotEngine
    engine=RobotEngine(assets/'model.mjb',assets/'initial_trace.json',manifest['hz'],reference=assets/'reference.json',target=target_name);m=engine.model;d=engine.data
    frozen=d.qpos.copy();reference=engine.ref;spec=engine.target_spec
    center=d.xpos[engine.fruit]+d.xmat[engine.fruit].reshape(3,3)@np.array(spec['center'])
    hook=np.eye(4);hook[:3,:3]=d.xmat[engine.hook].reshape(3,3);hook[:3,3]=d.xpos[engine.hook];nominal=hook@optical_transform('color')
    actual_nominal=nominal.copy()
    reference_spec=next(s for s in reference['fruit_specs'] if s['name']=='Tomato_05')
    reference_body=m.body('Tomato_05').id
    reference_center=d.xpos[reference_body]+d.xmat[reference_body].reshape(3,3)@np.array(reference_spec['center'])
    nominal[:3,3]+=center-reference_center  # Virtual observation translation only; physical robot stays unchanged.
    k=np.array([[640/(2*np.tan(np.deg2rad(69.4/2))),0,320],[0,640/(2*np.tan(np.deg2rad(69.4/2))),240],[0,0,1]])
    plan=views(nominal,center,k,args.seed)
    if args.limit:plan=plan[:args.limit]
    out=(args.output or Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+target_name.lower()+'_observations')).resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'observations').mkdir();np.savez_compressed(out/'initial_state.npz',qpos=frozen,qvel=d.qvel,ctrl=d.ctrl,robot_joint_positions=engine.initial)
    shutil.copy2(__file__,out/'prepare_observations_source.py')
    shutil.copy2(Path(__file__).with_name('d435_capture.py'),out/'d435_capture_source.py')
    shutil.copy2(source/'manifest.json',out/'source_manifest.json');shutil.copy2(source/'planning_inputs.json',out/'source_planning_inputs.json')
    (out/'view_plan.json').write_text(json.dumps(plan,indent=2))
    target_ids=[m.geom(s['name']).id for s in reference['shapes'] if s['path']==spec['path']+'/FruitCollider']+[m.geom(s['name']).id for s in reference['visuals'] if s['path']==spec['path']+'/TRUSS_Fruit_'+target_name[-2:]]
    if len(target_ids)!=2:raise RuntimeError('Unexpected target fruit geometry')
    from camera_action import motion_phases
    actions=[];world_poses=[];action_ids=[];action_phases=None
    for r in sorted(results,key=lambda r:r['candidate_id']):
        poses=r.get('waypoints',[])
        item={key:r.get(key) for key in ('candidate_id','parameters','result','center_entered','target_center_max_displacement_m','trace_sha256','states_sha256')}
        item['source_candidate_directory']=str(source/'candidates'/r['candidate_id'])
        item['waypoints_world']=poses
        item['hook_success']=None
        actions.append(item)
        if len(poses) in (4,5):
            names=motion_phases(poses)
            if action_phases is not None and names!=action_phases:raise ValueError('Mixed trajectory phases')
            action_phases=names
            world_poses.append([[*w['position_xyz'],*w['orientation_xyzw']] for w in poses]);action_ids.append(r['candidate_id'])
    (out/'actions.json').write_text(json.dumps(actions,indent=2))
    positions=np.array(world_poses)[:,:,:3];rotations=Rotation.from_quat(np.array(world_poses)[:,:,3:].reshape(-1,4)).as_matrix().reshape(-1,len(action_phases),3,3)
    rows=[];began=time.perf_counter();print('[관측 데이터]',out,flush=True)
    for v in plan:
        oid=v['observation_id'];folder=out/'observations'/oid;pose=np.array(v['world_from_color'])
        meta=capture(m,d,folder,world_from_color=pose,target_geom_ids=target_ids)
        np.testing.assert_array_equal(d.qpos,frozen)
        mask=np.array(Image.open(folder/'target_visible_mask.png'))>0;isolated=np.array(Image.open(folder/'target_isolated_mask_gt.png'))>0
        visible=int(mask.sum());expected=int(isolated.sum());fraction=min(1.,visible/expected) if expected else None
        truncated=bool(isolated[0].any() or isolated[-1].any() or isolated[:,0].any() or isolated[:,-1].any())
        classification='not_visible' if visible==0 else 'truncated' if truncated else 'heavily_occluded' if fraction<.2 else 'partially_occluded' if fraction<.9 else 'visible'
        uv=v['projected_target_center_px'];y,x=np.where(mask)
        target=dict(id=target_name,center_pixel_gt=uv,center_world_gt=center.tolist(),visible_bbox_xyxy=bbox(mask),isolated_bbox_xyxy_gt=bbox(isolated),visible_centroid_px=[float(x.mean()),float(y.mean())] if visible else None,visible_pixels=visible,isolated_pixels_in_frame_gt=expected,occlusion_fraction_in_frame=round(1-fraction,4) if fraction is not None else None,truncated=truncated,visibility=classification,label_source='MuJoCo geom-ID segmentation, fruit only; no vision detector',input_usable=visible>=20)
        crop=save_crops(folder,meta,uv,2*spec['radius']*k[0,0]/v['target_optical_z_m'])
        preview=Image.open(folder/'rgb.png').copy();draw=ImageDraw.Draw(preview)
        if target['visible_bbox_xyxy']:
            box=target['visible_bbox_xyxy'];draw.rectangle(box,outline='#58ffba',width=2);draw.text((max(0,box[0]),max(0,box[1]-14)),target_name+' (GT)',fill='#58ffba')
        if 0<=uv[0]<640 and 0<=uv[1]<480:draw.ellipse((uv[0]-3,uv[1]-3,uv[0]+3,uv[1]+3),outline='#ffff00',width=1)
        preview.save(folder/'annotated_preview.jpg',quality=85)
        camera_positions=(positions-pose[:3,3])@pose[:3,:3]
        camera_rotations=pose[:3,:3].T@rotations
        quats=Rotation.from_matrix(camera_rotations.reshape(-1,3,3)).as_quat().reshape(-1,len(action_phases),4)
        np.savez_compressed(folder/'actions_camera.npz',candidate_ids=np.array(action_ids),position_xyz=camera_positions,orientation_xyzw=quats,phase_names=np.array(action_phases))
        # Check camera labels reconstruct the original physical paths.
        np.testing.assert_allclose(camera_positions@pose[:3,:3].T+pose[:3,3],positions,atol=1e-10)
        np.testing.assert_allclose(pose[:3,:3]@camera_rotations,rotations,atol=1e-10)
        row=dict(**v,target=target,crops=crop,rgb=f'observations/{oid}/rgb.png',depth=f'observations/{oid}/depth_aligned_to_color_m.npy',camera_metadata=f'observations/{oid}/camera.json',shared_actions='../../actions.json',action_poses_camera='actions_camera.npz',initial_state='../../initial_state.npz',geometry_unchanged=True)
        (folder/'observation.json').write_text(json.dumps(row,indent=2));rows.append(row)
        print('[촬영]',len(rows),'/',len(plan),oid,classification,'위치',np.round(uv).astype(int).tolist(),flush=True)
    with (out/'observations.jsonl').open('w') as f:
        for r in rows:f.write(json.dumps(r)+'\n')
    counts=Counter(r['target']['visibility'] for r in rows);cells=Counter(str(r['screen_cell']) for r in rows)
    metadata=dict(schema='farmily_observation_v1',source_run=str(source),source_model_sha256=manifest['model_sha256'],source_results_sha256=sha(source/'results.json'),initial_state_sha256=sha(out/'initial_state.npz'),source_camera_urdf_sha256=sha(HOME.parent/'nvidia-sim/robot_usd/rb5_farmily.urdf'),target=target_name,actual_robot_world_from_color=actual_nominal.tolist(),virtual_reference_translation_world=(center-reference_center).tolist(),observations=len(rows),actions=len(actions),pose_label_actions=len(action_ids),seed=args.seed,nominal_world_from_color=nominal.tolist(),translation_bounds_camera_m=[.05,.05,.025],rotation_bounds_camera_xyz_deg=[20,20,5],sampling='independent Sobol pose perturbations, selection stratified by target projected 3x3 image cell; no look-at recentering',visibility_counts=dict(counts),screen_cell_counts=dict(cells),split_rule='upper-right cell test; lower-left validation; other in-frame train; out-of-frame visibility_test',split_caveat='All observations share one plant scene and action outcomes. This split tests view variation only, not unseen tomatoes or proof of image necessity. Do not randomly split image-action pairs.',task='predict center entry and maximum displacement; hook success unvalidated',physical_robot_motion=False,plant_randomization=False,physical_rollouts_performed=0,label_semantics='same world-frame paths/outcomes for every view; per-view camera-frame ring waypoints provided. Virtual observation camera pose is an input, not a physically reachable robot pose.',occlusion_definition='1 - visible fruit pixels / isolated fruit pixels inside image; does not measure offscreen fraction',depth_model='ideal D435 intrinsics approximation; clip 0.10m; no real sensor noise',wall_s=time.perf_counter()-began)
    (out/'dataset.json').write_text(json.dumps(metadata,indent=2))
    from validate_observations import validate

    sheet=Image.new('RGB',(960,792),'#e9efea');draw=ImageDraw.Draw(sheet)
    for cell in range(9):
        sample=next((r for r in rows if r['screen_cell']==cell and not r.get('nominal')),None)
        if sample is None:continue
        image=Image.open(out/'observations'/sample['observation_id']/'annotated_preview.jpg').resize((320,240))
        x,y=(cell%3)*320,(cell//3)*264;sheet.paste(image,(x,y));draw.text((x+8,y+244),sample['observation_id']+' / cell '+str(cell),fill='#234e3e')
    sheet.save(out/'coverage_preview.jpg',quality=90)
    from upgrade_observation_actions import upgrade
    upgrade(out,workers=args.postprocess_workers)
    validation=validate(out,workers=args.postprocess_workers)
    final_meta=json.loads((out/'dataset.json').read_text());final_meta.update(validation=validation,total_observation_wall_s=time.perf_counter()-began);(out/'dataset.json').write_text(json.dumps(final_meta,indent=2))
    gallery(out,rows)
    print('[관측 준비 완료]',out/'index.html',dict(counts),flush=True)
if __name__=='__main__':main()
