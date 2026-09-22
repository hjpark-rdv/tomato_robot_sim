"""Single GLB scene observation pilot with fail-closed collection preflight.

Saves actual RGB, metric depth, and target mask. Refuses action-labelled data
while the GLB model is visual-only and robot/planning integration is absent.
"""
import argparse,datetime,hashlib,html,json,subprocess,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path);p.add_argument('--glb',type=Path,default=ROOT/'nvidia-sim/env_usd/tomato_rotate_glb/tomato_master_v10_cluster_curve_cyan.glb');p.add_argument('--y-deg',type=float,default=90);p.add_argument('--target',default='Fruit_05');args=p.parse_args()
    output=args.output or Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_glb_single_collection_pilot')
    output.mkdir(parents=True,exist_ok=False);scene=output/'scene'
    subprocess.run([sys.executable,str(Path(__file__).with_name('attach_glb_preview.py')),str(args.glb),'--y-deg',str(args.y_deg),'--output',str(scene)],check=True)
    import render_backend
    import mujoco as mj
    from PIL import Image
    from audit_glb_collisions import glb_labels,audit
    m=mj.MjModel.from_xml_path(str(scene/'scene.xml'));d=mj.MjData(m);mj.mj_forward(m,d)
    meta=json.loads((scene/'scene.json').read_text());bid=m.body('glb_origin').id;gids=np.where(m.geom_bodyid==bid)[0];labels=glb_labels(args.glb)
    target_gids=[int(g) for g,label in zip(gids,labels) if label.split('/')[0]==args.target]
    if not target_gids:raise ValueError('Target mesh not found: '+args.target)
    target_center=d.geom_xpos[target_gids].mean(0)
    cam=mj.MjvCamera();cam.lookat[:]=target_center;cam.distance=.55;cam.azimuth=110;cam.elevation=-12
    width,height=640,480;m.vis.quality.offsamples=0;r=mj.Renderer(m,height=height,width=width)
    r.update_scene(d,camera=cam);rgb=r.render().copy()
    # Renderer depth is linear distance in metres along the optical axis.
    r.enable_depth_rendering();r.update_scene(d,camera=cam);depth=r.render().copy();r.disable_depth_rendering()
    r.enable_segmentation_rendering();r.update_scene(d,camera=cam);seg=r.render().copy();r.disable_segmentation_rendering()
    mask=np.isin(seg[:,:,0],target_gids)&(seg[:,:,1]==int(mj.mjtObj.mjOBJ_GEOM))
    camera=r.scene.camera[0];position=(r.scene.camera[0].pos+r.scene.camera[1].pos)/2
    forward=camera.forward.copy();up=camera.up.copy();right=np.cross(forward,up)
    optical=np.eye(4);optical[:3,:3]=np.column_stack([right,-up,forward]);optical[:3,3]=position
    fy=(height/2)/np.tan(np.deg2rad(m.vis.global_.fovy)/2);K=[[fy,0,width/2],[0,fy,height/2],[0,0,1]]
    r.close();obs=output/'observation';obs.mkdir()
    Image.fromarray(rgb).save(obs/'rgb.png');Image.fromarray(mask.astype(np.uint8)*255).save(obs/'target_mask.png');np.save(obs/'depth_m.npy',depth);np.save(obs/'segmentation.npy',seg)
    finite=np.isfinite(depth)&(depth>0)&(seg[:,:,0]>=0);np.save(obs/'depth_valid.npy',finite);scaled=np.clip(depth/2,0,1);Image.fromarray((scaled*255).astype(np.uint8)).save(obs/'depth_preview.png')
    (obs/'camera.json').write_text(json.dumps(dict(width=width,height=height,K=K,world_from_optical=optical.tolist(),target=args.target,target_geom_ids=target_gids,target_reference_world_m=target_center.tolist(),depth_units='metres',depth_type='rendered_optical_axis_depth',validity_file='depth_valid.npy',background='far-plane depth retained; exclude using validity mask',scope='virtual camera; not D435 calibration or robot reachable observation'),indent=2))
    # Use the same scene for the read-only hull screen, without changing masks.
    auditmeta=dict(meta,trusses=[dict(id='glb_origin',source_glb=str(args.glb.resolve()))]);(scene/'scene.json').write_text(json.dumps(auditmeta,indent=2));collision=audit(scene);(output/'collision_audit.json').write_text(json.dumps(collision,indent=2))
    robot_present=any(m.body(i).name=='Hook' for i in range(m.nbody))
    collision_enabled=bool(np.any((m.geom_contype[gids]!=0)|(m.geom_conaffinity[gids]!=0)))
    elastic=bool(np.any(m.jnt_bodyid==bid))
    failures=[]
    if not meta.get('physics_ready',False):failures.append('scene_metadata_physics_ready_false')
    if not collision_enabled:failures.append('all_glb_collision_masks_disabled')
    if not elastic:failures.append('no_glb_elastic_joints')
    if not robot_present:failures.append('robot_and_hook_missing')
    failures.append('scene_specific_planner_and_target_reference_not_integrated')
    if not mask.any():failures.append('target_not_visible')
    result=dict(status='rejected_before_rollout',scene=str(scene),source_glb_sha256=hashlib.sha256(args.glb.read_bytes()).hexdigest(),target=args.target,observations_saved=1,target_visible_pixels=int(mask.sum()),valid_depth_pixels=int(finite.sum()),depth_shape=list(depth.shape),preflight_failures=failures,physics_rollouts=0,candidate_labels_saved=0,training_eligible=False,collision_groups=collision['groups'])
    (output/'summary.json').write_text(json.dumps(result,indent=2))
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>GLB 단일 장면 수집 연결 검사</title><style>body{font:18px sans-serif;background:#18212b;color:white}img{width:32%}</style><h1>GLB 단일 장면: 관측 저장 / 실행 전 검사에서 제외</h1><p>RGB·Depth·목표 마스크 실제 저장. 로봇 물리 실행0회, 성공/실패 라벨0개. 학습 데이터로 사용하지 않습니다.</p><img src="observation/rgb.png"><img src="observation/depth_preview.png"><img src="observation/target_mask.png"><pre>'+html.escape(json.dumps(result,ensure_ascii=False,indent=2))+'</pre>',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='collision_groups'},indent=2));print(output/'index.html')
if __name__=='__main__':main()
