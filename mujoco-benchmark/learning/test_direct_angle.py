"""One nominal RGB-D image -> one predicted azimuth -> one physical attempt per fruit."""
import argparse,datetime,html,json,os,subprocess,sys,time
from pathlib import Path
import numpy as np
import torch
from features import extract
from train_direct_angle import predict

HOME=Path(__file__).resolve().parents[1]
ROOT=HOME.parent
PHYSICS_PY=HOME/'.venv/bin/python'
PLANNING_PY=Path('/root/isaaclab_env/bin/python')


def run(command,log):
    with Path(log).open('w') as stream:
        subprocess.run([str(x) for x in command],stdout=stream,stderr=subprocess.STDOUT,check=True)


def direct_candidate(angle,seed):
    # Same fixed staged6d motion as the collection, with a learned azimuth.
    return dict(candidate_id='predicted_00000',approach_azimuth_deg=float(angle),
                entry_clearance_m=.002,lateral_offset_m=0.,insertion_distance_m=.0425,
                lift_forward_angle_deg=0.,lift_distance_m=.0325,
                sampling='direct_rgbd_angle_v1',lift_profile='diagonal_45_return',
                trajectory_mode='staged6d',stage='coarse',parent_id=None,seed=seed,
                sobol_index=None,pre_hook_distance_m=.17,hook_roll_deg=0.,approach_elevation_deg=0.)


def evaluate(training,output,seed,angle_min=0.,angle_max=90.,max_targets=None):
    training=Path(training).resolve();output=Path(output).resolve()
    meta=json.loads((training/'manifest.json').read_text())
    if meta.get('schema')!='direct_rgbd_angle_training_v1':raise ValueError('Use a direct RGB-D angle model')
    source=json.loads((Path(meta['collection'])/'collection.json').read_text())
    if seed==source['seed']:raise ValueError('Use a different scene seed from training')
    if not 0<=angle_min<=angle_max<=180:raise ValueError('GLB Y rotation range must be within 0..180')
    output.mkdir(parents=True,exist_ok=False);started=time.perf_counter()
    scene_root=output/'scenes'
    run([PHYSICS_PY,HOME/'scripts/generate_random_glb_scenes.py','--seed',seed,
         '--scenes',1,'--trusses',1,'--truss-scale',.5,
         '--angle-min',angle_min,'--angle-max',angle_max,'--output',scene_root],output/'scene_generation.log')
    scene=scene_root/'scene_0000';record=json.loads((scene/'scene.json').read_text())
    if not record['validation']['scene_screen_passed']:
        summary=dict(status='scene_screen_failed',seed=seed,scene=record,results=[])
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
        (output/'index.html').write_text('<meta charset="utf-8"><h1>장면 초기 물리 검사 실패</h1><p>로봇 실행 없이 기록했습니다.</p><a href="summary.json">진단</a>')
        return summary
    reference=json.loads((scene/'reference.json').read_text())
    specs=reference['fruit_specs'][:max_targets] if max_targets else reference['fruit_specs']
    captures=[];input_root=output/'single_images';(input_root/'observations').mkdir(parents=True)
    for spec in specs:
        target=spec['name'];folder=output/'targets'/target;folder.mkdir(parents=True)
        physics=folder/'physics';observation=folder/'observation'
        run([PHYSICS_PY,HOME/'scripts/candidate_experiment.py','--model',scene/'model.mjb',
             '--reference',scene/'reference.json','--target',target,'--scene-only',
             '--link-model','--output',physics],folder/'scene_setup.log')
        run([PHYSICS_PY,HOME/'scripts/prepare_observations.py',physics,'--views',1,
             '--seed',seed,'--output',observation],folder/'capture.log')
        row=json.loads((observation/'observations/view_0000/observation.json').read_text())
        if not row['target']['input_usable'] or not all(row['crops'][kind]['available'] for kind in ('local','context')):
            captures.append(dict(target=target,status='no_usable_observation',folder=str(folder)));continue
        identity=target;(input_root/'observations'/identity).symlink_to(observation/'observations/view_0000',target_is_directory=True)
        row['observation_id']=identity;captures.append(dict(target=target,status='captured',folder=str(folder),row=row))
    usable=[c for c in captures if c['status']=='captured']
    (output/'capture_status.json').write_text(json.dumps([{k:v for k,v in item.items() if k!='row'} for item in captures],indent=2))
    selected=meta['selected'];backbone=selected['backbone'];ck=training/f"{backbone}_seed{selected['seed']}"/'model.pt'
    (output/'features').mkdir();features=extract(input_root,output/'features','cuda' if torch.cuda.is_available() else 'cpu',
                     rows=[c['row'] for c in usable],backbone_store=output/'backbones') if usable else None
    decisions=[]
    for index,item in enumerate(usable):
        target=item['target'];folder=Path(item['folder']);physics=folder/'physics'
        vector=np.r_[features[backbone][index],features['depth'][index],features['crop_intrinsics'][index]]
        angle=predict(ck,vector)
        decision=dict(target=target,view='view_0000',predicted_approach_azimuth_deg=angle,
                      model=backbone,model_seed=selected['seed'],image=str(folder/'observation/observations/view_0000/rgb.png'),
                      scope='angle predicted from one RGB-D observation; no candidate angles scored')
        (folder/'prediction.json').write_text(json.dumps(decision,indent=2));decisions.append(decision)
    # Freeze all image-only predictions before planning or physical outcomes.
    (output/'predictions.json').write_text(json.dumps(decisions,indent=2))
    outcomes=[]
    for decision in decisions:
        target=decision['target'];folder=output/'targets'/target;physics=folder/'physics'
        candidate=direct_candidate(decision['predicted_approach_azimuth_deg'],seed)
        (physics/'candidates.json').write_text(json.dumps([candidate],indent=2))
        manifest=json.loads((physics/'manifest.json').read_text());manifest.update(count=1,scene_only=False,direct_prediction=decision,
            scope='one image-only predicted angle, one planned path, at most one fresh physics rollout')
        (physics/'manifest.json').write_text(json.dumps(manifest,indent=2))
        run([PLANNING_PY,HOME/'scripts/plan_candidates.py',physics,'--workers',1],folder/'planning.log')
        plan=json.loads((physics/'candidates/predicted_00000/plan.json').read_text())
        code=('import sys,json;from pathlib import Path;sys.path.insert(0,sys.argv[2]);'
              'from candidate_experiment import initialize,execute,report;'
              'root=Path(sys.argv[1]);initialize(root);'
              "plan=json.loads((root/'candidates/predicted_00000/plan.json').read_text());"
              "result=execute({'candidate_id':'predicted_00000'}) if plan['preflight'].get('passed') else dict(plan,result='ik_or_planning_failure');"
              "(root/'candidates/predicted_00000/result.json').write_text(json.dumps(result,indent=2));report(root,[result])")
        run([PHYSICS_PY,'-c',code,physics,HOME/'scripts'],folder/'execution.log')
        result=json.loads((physics/'candidates/predicted_00000/result.json').read_text())
        outcomes.append(dict(**decision,planning_passed=bool(plan['preflight'].get('passed')),
                             result=result['result'],physics_run=str(physics),
                             max_displacement_m=result.get('target_center_max_displacement_m'),
                             physics_valid=result.get('metrics',{}).get('glb_physics_valid')))
        print('[사진→각도→물리]',target,f"{decision['predicted_approach_azimuth_deg']:+.2f}°",result['result'],flush=True)
        (output/'summary.json').write_text(json.dumps(dict(status='running',seed=seed,results=outcomes),indent=2))
    summary=dict(schema='direct_rgbd_angle_test_v1',status='complete',seed=seed,
                 truss_y_range_deg=[angle_min,angle_max],truss_y_deg=record['placements'][0]['y_deg'],
                 trained_model=str(ck),targets_in_scene=len(reference['fruit_specs']),targets_attempted=len(outcomes),
                 skipped=len(specs)-len(outcomes),skipped_targets=[dict(target=c['target'],reason=c['status']) for c in captures if c['status']!='captured'],
                 center_entries=sum(x['result']=='partial_center_entry' for x in outcomes),
                 invalid_physics=sum(x['result']=='invalid_physics' for x in outcomes),
                 wall_s=time.perf_counter()-started,results=outcomes,
                 note='one nominal virtual RGB-D image, one predicted azimuth and at most one fresh rollout per target; center entry only, no harvest-success evaluator')
    (output/'summary.json').write_text(json.dumps(summary,indent=2))
    cards=''.join('<li>'+html.escape(x['target'])+f": {x['predicted_approach_azimuth_deg']:+.2f}° → "+html.escape(x['result'])+
                  ' · <a href="targets/'+html.escape(x['target'])+'/observation/observations/view_0000/rgb.png">RGB</a> · '+
                  '<a href="targets/'+html.escape(x['target'])+'/physics/index.html">물리</a></li>' for x in outcomes)
    (output/'index.html').write_text('<meta charset="utf-8"><h1>사진 한 장 → 각도 하나 → 물리 실행</h1><p>중심 진입 '+str(summary['center_entries'])+'/'+str(len(outcomes))+'; 꼭지 걸림 판정은 별도</p><ul>'+cards+'</ul><a href="summary.json">결과 JSON</a>')
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('training',type=Path);p.add_argument('--seed',type=int,required=True);p.add_argument('--angle-min',type=float,default=0.);p.add_argument('--angle-max',type=float,default=90.);p.add_argument('--max-targets',type=int);p.add_argument('--repeat',type=int,default=1,help='generate and finish each fresh scene before starting the next');p.add_argument('--output',type=Path,default=Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_direct_angle_test'));a=p.parse_args()
    if a.repeat<1:p.error('--repeat must be positive')
    if a.repeat==1:
        print(json.dumps(evaluate(a.training,a.output,a.seed,a.angle_min,a.angle_max,a.max_targets),indent=2))
    else:
        a.output.mkdir(parents=True,exist_ok=False);summaries=[]
        for index in range(a.repeat):
            result=evaluate(a.training,a.output/f'test_{index:04d}',a.seed+index,a.angle_min,a.angle_max,a.max_targets)
            summaries.append(dict(test=index,seed=a.seed+index,status=result['status'],
                                  targets_attempted=result.get('targets_attempted',0),
                                  center_entries=result.get('center_entries',0),
                                  output=str(a.output/f'test_{index:04d}')))
            (a.output/'batch_summary.json').write_text(json.dumps(summaries,indent=2))
        links=''.join(f'<li>seed {r["seed"]}: {r["status"]}, 진입 {r["center_entries"]}/{r["targets_attempted"]} · <a href="test_{r["test"]:04d}/index.html">결과</a></li>' for r in summaries)
        (a.output/'index.html').write_text('<meta charset="utf-8"><h1>사진 한 장 → 각도 하나 → 새 장면 반복</h1><ul>'+links+'</ul><a href="batch_summary.json">요약</a>')
        print(json.dumps(summaries,indent=2))
