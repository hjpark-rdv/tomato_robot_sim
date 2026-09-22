"""Freeze model recommendations first, then run fresh physics from identical resets."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import argparse,datetime,json,shutil,sys,time,hashlib,html,multiprocessing
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
import numpy as np
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))

def prepare(training,out):
    from predict import predict
    from scipy.spatial.transform import Rotation
    manifest=json.loads((training/'manifest.json').read_text());source=Path(manifest['source_run']);dataset=Path(manifest['dataset']);out.mkdir(parents=True,exist_ok=False)
    (out/'candidates').mkdir();shutil.copytree(source/'replay_assets',out/'replay_assets')
    trials=[]
    for name in ('action_only','resnet18_rgbd','dinov2_rgbd'):
        for view in manifest['views']['test']:
            rec=predict(training,view,name,seed=0,scope='test',top=1)['recommendations'][0];cid=rec['candidate_id'];tid=f'trial_{len(trials):03d}'
            assert cid not in manifest['action_splits']['train']
            folder=out/'candidates'/tid;folder.mkdir();info=json.loads((source/'candidates'/cid/'plan.json').read_text())
            # Camera-frame selection must map back to the exact executable world path.
            vr=dataset/'observations'/view;obs=json.loads((vr/'observation.json').read_text());cam=np.array(json.loads((vr/'camera.json').read_text())['world_from_optical']['color'])
            positions=np.array(rec['target_relative_waypoint_xyz_camera'])@cam[:3,:3].T+obs['target']['center_world_gt']
            np.testing.assert_allclose(positions,[w['position_xyz'] for w in info['waypoints']],atol=1e-9)
            np.testing.assert_allclose(cam[:3,:3]@Rotation.from_quat(rec['camera_action_14'][3:7]).as_matrix(),Rotation.from_quat(info['waypoints'][0]['orientation_xyzw']).as_matrix(),atol=1e-9)
            info['candidate_id']=tid;info['source_candidate_id']=cid;(folder/'plan.json').write_text(json.dumps(info,indent=2));shutil.copy2(source/'candidates'/cid/'trace.json',folder/'trace.json')
            trials.append(dict(trial_id=tid,model=name,seed=0,view=view,**rec))
    original=json.loads((source/'manifest.json').read_text());original.update(count=len(trials),training_run=str(training),source_run=str(source),dataset=str(dataset),scope='fresh 120Hz physics after frozen held-out-view/held-out-action recommendation; same known Tomato05 scene; seed0 predetermined',prediction_rule='entry_probability*exp(-predicted_displacement_mm/20)',reused_trace=True)
    original.pop('total_wall_s',None);original['arguments']={'training':str(training),'models':['action_only','resnet18_rgbd','dinov2_rgbd'],'seed':0};original['checkpoint_hashes']={name:hashlib.sha256((training/f'{name}_seed0/model.pt').read_bytes()).hexdigest() for name in ('action_only','resnet18_rgbd','dinov2_rgbd')}
    (out/'manifest.json').write_text(json.dumps(original,indent=2));(out/'recommendations.json').write_text(json.dumps(trials,indent=2));shutil.copy2(__file__,out/'test_physics_source.py')
    print('[추천 고정]',len(trials),'회 / 서로 다른 경로',len({t['candidate_id'] for t in trials}),out,flush=True)

def execute(out,workers):
    from candidate_experiment import initialize,execute as rollout
    start=time.perf_counter();trials=json.loads((out/'recommendations.json').read_text());results=[]
    with ProcessPoolExecutor(max_workers=workers,mp_context=multiprocessing.get_context('spawn'),initializer=initialize,initargs=(str(out),)) as pool:
        fs={pool.submit(rollout,{'candidate_id':t['trial_id']}):t for t in trials}
        for f in as_completed(fs):
            t=fs[f];r=f.result();results.append(dict(**t,physics=r));print('[물리 검증]',t['trial_id'],t['model'],t['candidate_id'],r['result'],round(r['target_center_max_displacement_m']*1000,2),'mm',flush=True)
    (out/'results.json').write_text(json.dumps(results,indent=2));summary={}
    for name in sorted({r['model'] for r in results}):
        rr=[r['physics'] for r in results if r['model']==name];summary[name]=dict(trials=len(rr),center_entered=sum(bool(r['center_entered']) for r in rr),low_displacement_entry=sum(r['center_entered'] and r['target_center_max_displacement_m']<=.02 for r in rr),max_displacement_mm=[r['target_center_max_displacement_m']*1000 for r in rr],invalid_physics=sum(r['result']=='invalid_physics' for r in rr))
    manifest=json.loads((out/'manifest.json').read_text());manifest.update(workers=workers,total_wall_s=time.perf_counter()-start);(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    summary['wall_s']=manifest['total_wall_s'];(out/'summary.json').write_text(json.dumps(summary,indent=2));report(out);print('[검증 완료]',summary,flush=True)

def report(out):
    rr=sorted(json.loads((out/'results.json').read_text()),key=lambda r:r['trial_id']);m=json.loads((out/'manifest.json').read_text());summary=json.loads((out/'summary.json').read_text());body=[]
    videos={t['candidate_id']:f'videos/{t["trial_id"]}.mp4' for t in rr if (out/'videos'/f'{t["trial_id"]}.mp4').exists()}
    for t in rr:
        r=t['physics'];v=videos.get(t['candidate_id'],'missing.mp4');link=f'<a href="{v}">2배속 영상</a>' if (out/v).exists() else ''
        body.append(f'<tr><td>{t["model"]}</td><td>{t["view"]}</td><td>{t["candidate_id"]}</td><td>{t["predicted_entry_probability"]:.1%}</td><td>{"진입" if r["center_entered"] else "미진입"}</td><td>{r["target_center_max_displacement_m"]*1000:.2f}</td><td>{link} <a href="candidates/{t["trial_id"]}/result.json">결과</a></td></tr>')
    page='<!doctype html><meta charset="utf-8"><title>모델 추천 실제 물리 검증</title><style>body{background:#14212c;color:#eaf1f5;font:16px/1.6 system-ui;margin:35px}table{border-collapse:collapse;width:100%}td,th{padding:10px;border-bottom:1px solid #465561;text-align:left}a{color:#7be6c4}video{width:100%;max-width:1100px}.notice{background:#293b47;padding:20px}</style><h1>모델 추천 → 로봇 물리 재실행</h1>'
    page+=f'<p class="notice">학습에 없는7개시점 ×3개모델(seed0고정), 매 회 같은초기상태로새물리계산. 경로선택후에결과를판정했습니다. 기존카탈로그의관절경로를실행했으며새경로생성은아닙니다. 같은Tomato05장면의반복으로실물일반화나꼭지걸림성공을뜻하지않습니다. 물리실행시간 {summary["wall_s"]:.1f}초.</p>'
    page+='<p>21회 추천의 서로 다른 경로는3개입니다. 중복 추천도각각초기화해실행했으며영상은경로별대표1개를공유합니다. 중심진입은부분진입판정이며꼭지걸림판정이아닙니다.</p>'
    page+='<table><tr><th>모델</th><th>관측</th><th>추천 후보</th><th>예측 진입확률</th><th>실제 중심 진입</th><th>최대 이동 mm</th><th>확인</th></tr>'+''.join(body)+'</table>'
    for t in rr:
        video=out/'videos'/f'{t["trial_id"]}.mp4'
        if video.exists():page+=f'<h2>대표 경로 {t["candidate_id"]} · {t["view"]}</h2><video controls preload="metadata" src="videos/{video.name}"></video>'
    page+='<p><a href="recommendations.json">실행 전 고정한 추천</a> · <a href="summary.json">요약</a> · <a href="results.json">전체 결과</a></p>'; (out/'index.html').write_text(page)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','execute','report']);p.add_argument('output',type=Path);p.add_argument('--training',type=Path);p.add_argument('--workers',type=int,default=8);a=p.parse_args()
    if a.stage=='prepare':prepare(a.training,a.output)
    elif a.stage=='execute':execute(a.output,a.workers)
    else:report(a.output)
