"""Fixed-scene coarse pose experiment, fresh PhysX process per candidate.

Run from the existing Isaac Python environment; this parent does not launch
Isaac itself. Each isolated worker gets the same scene; execution is sequential
by default, with optional independent concurrent processes.
"""
import argparse
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor,as_completed

from pose_candidates import coarse_candidates,tolerance_candidates,CATEGORIES,DEFAULT_LIMITS

HERE=Path(__file__).resolve().parent


def write(path,value):
    temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');temp.replace(path)


def run_worker(directory,root,candidate=None,physics_only=False):
    command=[str(HERE.parent/'run_ring_rl.sh'),'--mode','pose','--headless','--run-dir',str(directory)]
    if physics_only: command+=['--pose-physics-only']
    if candidate:
        source=root/'inputs'/(candidate['candidate_id']+'.json');write(source,candidate)
        command+=['--pose-candidate',str(source),'--pose-reference-state',str(root/'audit/initial_state.npz')]
    log=root/'logs'/((candidate['candidate_id'] if candidate else 'audit')+'.log')
    with log.open('w') as output:
        result=subprocess.run(command,stdout=output,stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f'Worker failed ({result.returncode}); inspect {log}. This is not a candidate failure label.')


def summarize(results):
    valid=[r for r in results if r['result']!='ik_or_planning_failure']
    success=[r for r in results if r['hook_success']]
    primary={k:sum(r['result']==k for r in results) for k in CATEGORIES}
    event_counts={k:sum(k in r['events'] for r in results) for k in CATEGORIES}
    def ranges(rows):
        if not rows: return None
        values={key:[r['parameters'][key] for r in rows] for key in ('azimuth_deg','elevation_deg','roll_deg','pitch_deg','pre_hook_distance_m')}
        for axis in range(3): values['offset_'+'xyz'[axis]+'_m']=[r['parameters']['offset_xyz_m'][axis] for r in rows]
        for axis in range(3):
            values['orientation_delta_'+'xyz'[axis]+'_deg']=[r['parameters']['orientation_delta_rpy_deg'][axis] for r in rows]
            values['pre_hook_world_'+'xyz'[axis]+'_m']=[r['candidate_position_xyz'][axis] for r in rows]
        return {k:dict(min=min(v),max=max(v)) for k,v in values.items()}
    groups={}
    for r in results:
        p=r['parameters']
        if p['stage']!='tolerance': continue
        key=(p['parent_id'],p['perturbation_kind'],p['perturbation_magnitude'])
        groups.setdefault(key,[]).append(r)
    tolerance=[]
    for (parent,kind,magnitude),rows in groups.items():
        successes=sum(r['hook_success'] for r in rows)
        tolerance.append(dict(parent=parent,kind=kind,magnitude=magnitude,units='m' if kind=='position' else 'deg',
            attempted=len(rows),expected=6,completed=len(rows)==6,successes=successes,success_rate=successes/len(rows),
            design='systematic +/- world X,Y,Z or local orientation X,Y,Z; each candidate fresh reset',
            interpretation='sampled axis perturbations, not a confidence bound or guaranteed connected success region'))
    best=sorted(success,key=lambda r:(r['target_max_displacement_m'],r['main_stem_max_displacement_m']))[:5]
    return dict(total_candidates=len(results),valid_candidates=len(valid),success_count=len(success),
        success_rate_all=len(success)/len(results) if results else None,
        success_rate_valid=len(success)/len(valid) if valid else None,
        primary_category_counts=primary,event_counts=event_counts,
        best_successful_candidates=[dict(candidate_id=r['candidate_id'],parameters=r['parameters'],
            target_max_displacement_m=r['target_max_displacement_m']) for r in best],
        successful_pose_ranges=ranges(success),successful_ranges_note='observed extrema only, not a proven continuous feasible volume',
        tolerance_test_results=tolerance,
        tolerance_status='completed' if tolerance and all(x['completed'] for x in tolerance) else 'partial' if tolerance else 'not_run_no_successful_coarse_candidate')


def export(root,results):
    with (root/'metadata.jsonl').open('w') as stream:
        for row in results: stream.write(json.dumps(row,allow_nan=False)+'\n')
    fields=['candidate_id','stage','parent_id','perturbation_kind','perturbation_magnitude','perturbation_axis','perturbation_sign',
        'azimuth_deg','elevation_deg','roll_deg','pitch_deg',
        'offset_x_m','offset_y_m','offset_z_m','pre_hook_distance_m','result','hook_success',
        'orientation_delta_x_deg','orientation_delta_y_deg','orientation_delta_z_deg',
        'position_x_m','position_y_m','position_z_m','quat_x','quat_y','quat_z','quat_w',
        'direction_x','direction_y','direction_z','first_contact_object','target_max_displacement_m',
        'main_stem_max_displacement_m','events','metadata_path']
    with (root/'results.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for r in results:
            p=r['parameters'];row={k:p.get(k) for k in fields if k in p}
            row.update({k:r.get(k) for k in fields if k in r})
            row.update({f'offset_{a}_m':v for a,v in zip('xyz',p['offset_xyz_m'])});row['events']=';'.join(r['events'])
            row.update({f'orientation_delta_{a}_deg':v for a,v in zip('xyz',p['orientation_delta_rpy_deg'])})
            row.update({f'position_{a}_m':v for a,v in zip('xyz',r['candidate_position_xyz'])})
            row.update({f'quat_{a}':v for a,v in zip('xyzw',r['candidate_orientation_quaternion'])})
            row.update({f'direction_{a}':v for a,v in zip('xyz',r['approach_direction'])})
            writer.writerow(row)
    summary=summarize(results);write(root/'summary.json',summary);return summary


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--run-dir',type=Path,default=HERE/'runs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_tomato05_pose_search'))
    parser.add_argument('--limit',type=int,help='Run the first N coarse candidates for a smoke test')
    parser.add_argument('--tolerance-top',type=int,default=2)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--physics-only',action='store_true',help='Incomplete CPU validation only; no RGB-D dataset')
    parser.add_argument('--workers',type=int,default=1,help='Independent fresh-process workers; default 1 for GPU memory usage')
    args=parser.parse_args();root=args.run_dir.resolve()
    candidates=coarse_candidates()
    if args.limit is not None:
        if args.limit<1: parser.error('limit must be positive')
        candidates=candidates[:args.limit]
    if args.tolerance_top<1: parser.error('tolerance-top must be positive')
    if args.workers<1 or args.workers>(8 if args.physics_only else 4): parser.error('workers must be 1..4 (1..8 for explicit CPU-only validation)')
    if (root/'experiment.json').exists() and not args.resume: parser.error('Experiment exists; use --resume or a new directory')
    for folder in ('inputs','logs','dataset/scene_0001'): (root/folder).mkdir(parents=True,exist_ok=True)
    if not args.physics_only:
        import torch
        if not torch.cuda.is_available():
            failure=dict(stage='RGB-D infrastructure preflight',cuda_available=False,dataset_complete=False,
                         error='No accessible CUDA device; RGB-D rendering cannot run. --physics-only is incomplete validation, not a substitute dataset.')
            write(root/'infrastructure_failure.json',failure)
            raise RuntimeError(failure['error'])
    experiment=dict(scene_id='scene_0001',target='Tomato_05',coarse_candidates=candidates,
        tolerance_top=args.tolerance_top,thresholds=DEFAULT_LIMITS,domain_randomization=False,physics_only=args.physics_only,
        reset_strategy='fresh process + repeated reset equality + cross-process state comparison',
        perception=False,learning=False,
        source_sha256={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in
            ['pose_candidates.py','pose_collision.py','pose_worker.py','greenhouse_env.py','elastic_plant.py']})
    if args.resume and (root/'experiment.json').exists():
        if json.loads((root/'experiment.json').read_text())!=experiment: raise ValueError('Resume configuration/source changed; create a new run')
    else: write(root/'experiment.json',experiment)
    if not (root/'audit/observation.json').exists():
        print('[SEARCH] Auditing fixed scene and reset',flush=True);run_worker(root/'audit',root,physics_only=args.physics_only)
    results=[]
    def candidate_run(p):
        directory=root/'dataset/scene_0001'/p['candidate_id'];path=directory/'candidate.json'
        if not path.exists():
            print('[SEARCH] Start '+p['candidate_id']+' '+json.dumps(p),flush=True)
            run_worker(directory,root,p,physics_only=args.physics_only)
        row=json.loads(path.read_text());row['metadata_path']=str(path.relative_to(root))
        for key in ('rgb','depth'):
            if row[key] is not None: row[key]=str((directory/row[key]).relative_to(root))
        return row
    def collect(row):
        results.append(row);results.sort(key=lambda r:r['candidate_id']);summary=export(root,results)
        print('[SEARCH RESULT] '+json.dumps(dict(candidate=row['candidate_id'],result=row['result'],
            total=summary['total_candidates'],valid=summary['valid_candidates'],successes=summary['success_count'])),flush=True)
    def batch(items):
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures=[pool.submit(candidate_run,p) for p in items]
            try:
                for future in as_completed(futures): collect(future.result())
            except Exception as error:
                for future in futures: future.cancel()
                write(root/'execution_error.json',dict(error=str(error),candidate_failure_label=False))
                raise
    batch(candidates)
    coarse_success=sorted([r for r in results if r['hook_success']],key=lambda r:r['target_max_displacement_m'])[:args.tolerance_top]
    for parent in coarse_success:
        batch(tolerance_candidates(parent['parameters']))
    summary=export(root,results);summary['candidate_execution_complete']=True
    summary['experiment_complete']=not args.physics_only;summary['coarse_count']=len(candidates)
    summary['tolerance_count']=len(results)-len(candidates)
    summary['dataset_complete']=not args.physics_only
    summary['missing_requirements']=['RGB-D capture unavailable in explicit physics-only run'] if args.physics_only else []
    write(root/'summary.json',summary)
    print('[SEARCH COMPLETE] '+json.dumps(summary),flush=True)


if __name__=='__main__': main()
