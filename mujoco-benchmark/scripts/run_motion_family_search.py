"""Bounded multi-scene/multi-family OFFLINE search using the existing executors.

Input: a manifest of immutable target run snapshots (or an existing collection).
Output: independent run roots, native preflight/physics evidence, HTML and a
bounded representative-video queue. No binary harvest labels are fabricated.
"""
from __future__ import annotations
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
import copy
import csv
import gzip
import hashlib
import html
import json
import math
import multiprocessing
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from motion_family_search import (FAMILIES, SCHEMA, digest, generate_candidates,
                                  identity, summarize_target, trial_outcome)

SCRIPTS = Path(__file__).resolve().parent
REQUIRED = ('manifest.json', 'planning_inputs.json', 'replay_assets/model.mjb',
            'replay_assets/reference.json', 'replay_assets/initial_trace.json',
            'replay_assets/planning_model.pkl')


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, obj):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    tmp.replace(path)


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''): h.update(block)
    return h.hexdigest()


def run_command(command, log, timeout):
    """Kill only this newly-created process group on timeout, including workers."""
    with Path(log).open('w', encoding='utf-8') as f:
        child = subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait()
            raise TimeoutError('Child command exceeded declared time budget')


def load_cases(path):
    """Keep source split assignment fixed; views never create independent scenes."""
    path = Path(path).resolve(); doc = read(path)
    if 'cases' not in doc:
        raise ValueError('Manifest requires cases; use --discover-collection for old collections')
    seen, scene_models, scene_splits, result = set(), {}, {}, []
    for row in doc['cases']:
        scene, target = identity(row['scene_id']), identity(row['target'])
        if (scene, target) in seen: raise ValueError('Duplicate scene/target')
        seen.add((scene, target))
        root = Path(row['source_run'])
        root = (path.parent/root).resolve() if not root.is_absolute() else root.resolve()
        hashes = {name: file_hash(root/name) for name in REQUIRED}
        source = read(root/'manifest.json')
        if source['target'] != target: raise ValueError('Snapshot target differs from manifest')
        actual_hash = hashes['replay_assets/model.mjb']
        if source.get('model_sha256') and source['model_sha256'] != actual_hash:
            raise ValueError('Snapshot model hash mismatch')
        split = row.get('split', 'pilot')
        if split not in ('pilot', 'train', 'validation', 'test'): raise ValueError('Unknown split')
        if scene in scene_models and scene_models[scene] != actual_hash:
            raise ValueError('Different scene models share a scene_id')
        if scene in scene_splits and scene_splits[scene] != split:
            raise ValueError('One scene cannot cross data splits')
        scene_models[scene], scene_splits[scene] = actual_hash, split
        result.append(dict(scene_id=scene, target=target, split=split, source_run=str(root),
                           hashes=hashes, observation_path=row.get('observation_path'),
                           collision_coverage=row.get('collision_coverage', 'unreviewed')))
    # Identical full scene bytes across different split IDs are detectable leakage.
    by_model = {}
    for row in result:
        key = row['hashes']['replay_assets/model.mjb']
        if key in by_model and by_model[key] != row['split']:
            raise ValueError('Identical model snapshots cross splits')
        by_model[key] = row['split']
    if not result: raise ValueError('No cases')
    return result


def discover_collection(root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    if output.exists(): raise ValueError('Do not overwrite a manifest')
    collection = read(root/'collection.json')
    cases = []
    for path in sorted(root.glob('scene_*/targets/*/physics/manifest.json')):
        run = path.parent; target = read(path)['target']; scene = run.parents[2].name
        cases.append(dict(scene_id=scene, target=target, source_run=str(run),
                          split=('pilot' if collection.get('scene_splits', {}).get(scene, 'pilot') == 'smoke' else collection.get('scene_splits', {}).get(scene, 'pilot')),
                          observation_path=str(run.parent/'observations')))
    write(output, dict(schema=SCHEMA, cases=cases,
                       source_collection=str(root), source_collection_sha256=file_hash(root/'collection.json'),
                       source_scene_failures=collection.get('failures', []),
                       scope='Existing immutable snapshots; no new scene generated by discovery'))
    return output


def prepare_snapshot(case, destination):
    source, destination = Path(case['source_run']), Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    assets = destination/'replay_assets'; assets.mkdir()
    # Links are read-only by convention and checked by source hashes after use.
    # No chmod is performed on shared source inodes.
    for name in REQUIRED:
        if name.startswith('replay_assets/'):
            (destination/name).symlink_to((source/name).resolve())
    for name in ('planning_inputs.json', 'action_frame.json'):
        if (source/name).is_file():
            (destination/name).write_bytes((source/name).read_bytes())
    manifest = copy.deepcopy(read(source/'manifest.json'))
    manifest.pop('environment_preflight_policy', None)
    manifest['source_candidate_count'] = manifest.pop('count', None)
    manifest.update(search_schema=SCHEMA, source_run=str(source), diagnostic_only=True,
                    training_eligible=False, hook_success=None,
                    scope='SIM-GT motion-family search, not a new RGB-D inference API')
    write(destination/'manifest.json', manifest)
    return manifest


def native_geometry(run):
    import numpy as np
    from robot_engine import RobotEngine
    from hook_retention_diagnostic import HookProbe, capsule_endpoints
    from hook_seating_geometry import Capsule
    from target_fruit_contact_trial import scope_from_engine
    from geometry import RING_RADIUS
    manifest = read(run/'manifest.json')
    e = RobotEngine(run/'replay_assets/model.mjb', run/'replay_assets/initial_trace.json',
                    manifest['hz'], reference=run/'replay_assets/reference.json', target=manifest['target'])
    probe = HookProbe(e, 0.)
    scope = scope_from_engine(e, probe, search_policy=True)
    ring, rotation = probe.frame(e.data)
    center = e.data.xpos[e.fruit]+e.data.xmat[e.fruit].reshape(3,3)@np.asarray(e.target_spec['center'])
    expected = np.asarray(read(run/'planning_inputs.json')['inputs']['geometry'][0])
    if np.linalg.norm(center-expected) > 1e-5:
        raise ValueError('Planner target center does not match this reset model')
    heading = ring-center; heading[2] = 0.
    if np.linalg.norm(heading) < 1e-8: raise ValueError('Undefined horizontal approach reference')
    heading /= np.linalg.norm(heading)
    targets = [Capsule(e.model.geom(g).name, *capsule_endpoints(e.model,e.data,g)) for g in probe.target_ids]
    wires = []
    for g in probe.rear_ids:
        a,b,r = capsule_endpoints(e.model,e.data,g)
        wires.append(Capsule(e.model.geom(g).name, rotation.inv().apply(a-ring), rotation.inv().apply(b-ring), r))
    geometry = dict(center=center, radius=float(e.target_spec['radius']), heading=heading,
                    targets=targets, rear_wires=wires, ring_radius=RING_RADIUS)
    # Same native collider inventory drives new contact permissions; no global Rachis wildcard.
    policy = dict(clearance_m=0., max_distance_queries=12000000, timeout_s=120.,
                  allowed_contacts=[dict(robot_geom=w, environment_geom=t, phases=['seat','hold','verify'])
                                    for w in scope.rear_wires for t in scope.pedicels])
    from dataclasses import asdict
    write(run.parent/'native_geometry.json', dict(center=center.tolist(), radius=geometry['radius'],
          heading=heading.tolist(), targets=[t.json() for t in targets], rear_wires=[w.json() for w in wires],
          contact_scope=asdict(scope), state_source='unchanged reset model', geometric_ground_truth=True))
    return geometry, policy


def read_trial(trial, cid, family, returncode):
    record = dict(candidate_id=cid, family=family, trial_dir=str(trial), returncode=returncode,
                  physics_executed=False, training_eligible=False)
    result_path = trial/'target_contact_trial.json'
    if not result_path.exists():
        return dict(record, outcome='execution_error', reason='No atomic trial result; see log')
    result = read(result_path)
    samples_path = trial/'authorized_contact_samples.json.gz'
    samples = json.loads(gzip.decompress(samples_path.read_bytes())) if samples_path.exists() else []
    audit = read(trial/'environment_preflight.json') if (trial/'environment_preflight.json').exists() else {}
    record.update(outcome=trial_outcome(result, samples), physics_executed=result.get('physics_executed',False),
                  completed=result.get('completed',False), stop_reasons=result.get('stop_reasons',[]),
                  audit_status=result.get('audit_status'), audit_complete=result.get('audit_complete'),
                  first_violation=audit.get('first_violation'),
                  geometric_samples=sum(bool(r.get('seated')) for r in samples),
                  observed_samples=len(samples), simulated_s=result.get('simulated_s',0.),
                  source_unchanged=result.get('source_unchanged'),
                  contact_retention_evidence=result.get('authorized_contact_evidence',{}).get('contact_retention_evidence',False))
    return record


def run_case(case, config, output):
    """One case per spawned worker: no shared native state or planner globals."""
    out = Path(output)/'cases'/case['scene_id']/case['target']
    if out.exists():
        if (out/'case_result.json').exists(): return read(out/'case_result.json')
        # Never assume a process that disappeared consumed zero rollouts.
        raise ValueError('Interrupted case exists; inspect it or use a new campaign output, never overwrite')
    out.mkdir(parents=True)
    records, attempted, render_queue = [], 0, []
    report = dict(case=case, records=records, render_queue=render_queue, complete=False)
    start = time.monotonic()
    try:
        run = out/'run'; manifest = prepare_snapshot(case, run)
        if config.get('model_cache'):
            from search_model_cache import attach
            attach(run, Path(case['source_run'])/'replay_assets/model.mjb',
                   case['hashes']['replay_assets/model.mjb'], config['model_cache'])
        geometry, policy = native_geometry(run); write(out/'base_policy.json',policy)
        candidates, rejected = generate_candidates(geometry, config['samples_per_family'], config['seed'])
        records.extend(dict(x, outcome='proposal_rejected') for x in rejected)
        write(out/'proposal_rejections.json',rejected)
        by_id = {x['candidate_id']: x for x in candidates}

        def wave(items, attempt_cap):
            nonlocal attempted
            if not items: return
            write(run/'candidates.json', items)
            code = run_command([config['planning_python'],str(SCRIPTS/'plan_candidates.py'),str(run),
                                '--workers',str(config['planning_workers'])],
                               out/f"planning_{len(records):04d}.log", config['planning_timeout_s'])
            for candidate in items:
                cid, family = candidate['candidate_id'], candidate['family']
                planpath = run/'candidates'/cid/'plan.json'
                if not planpath.exists():
                    records.append(dict(candidate_id=cid, family=family, outcome='planning_error',
                                        reason=f'No plan, planner exit {code}', physics_executed=False)); continue
                plan = read(planpath)
                if not plan['preflight'].get('passed'):
                    reason = plan['preflight'].get('reason','unknown')
                    records.append(dict(candidate_id=cid, family=family, outcome='ik_not_found' if 'ik' in reason else 'planning_rejected',
                                        reason=plan['preflight'], physics_executed=False)); continue
                if attempted >= attempt_cap and config['execute']:
                    records.append(dict(candidate_id=cid,family=family,outcome='not_evaluated_budget',physics_executed=False));continue
                trial = out/'trials'/cid; trial.parent.mkdir(exist_ok=True)
                command = [config['native_python'],str(SCRIPTS/'target_fruit_contact_trial.py'),str(run),
                           '--candidate',cid,'--base-policy',str(out/'base_policy.json'),
                           '--output',str(trial),'--search-contact-policy']
                if config['execute']:
                    command += ['--execute','--max-target-force-n',str(config['max_target_force_N']),
                                '--max-target-displacement-m',str(config['max_target_displacement_m'])]
                write(out/'progress.json',dict(records=records,next_candidate=cid,physics_attempts=attempted))
                try:
                    code = run_command(command,out/f'{cid}.log',config['trial_timeout_s'])
                    row = read_trial(trial,cid,family,code)
                    attempted += int(row['physics_executed'])
                    if code not in (0,2):
                        row.update(outcome='execution_error',reason=f'Trial exit {code}')
                        # Missing summary: charge a conservative attempt instead of exceeding the budget.
                        if config['execute'] and not row['physics_executed']: attempted += 1
                except TimeoutError as error:
                    if config['execute']: attempted += 1
                    row = dict(candidate_id=cid,family=family,outcome='execution_timeout',physics_executed=False,
                               execution_uncertain=True,reason=str(error))
                records.append(row)
                if (trial/'trial_states.npz').exists():
                    render_queue.append(dict(run=str(run),candidate=cid,family=family,states=str(trial/'trial_states.npz'),
                        samples=str(trial/'authorized_contact_samples.json.gz'),outcome=row['outcome'],
                        output=str(out/'videos'/f'{cid}.mp4'),kind='recorded_physics_not_nominal'))
                if time.monotonic()-start > config['case_timeout_s']:
                    raise TimeoutError('Case budget exceeded; remaining proposals are not failures')

        coarse_cap = max(min(len(FAMILIES),config['physics_per_target']), config['physics_per_target']-2*config['refine_parents'])
        wave(candidates,coarse_cap)
        if config['refine_parents'] and config['execute'] and attempted < config['physics_per_target']:
            score = {'contact_retention_evidence':4,'geometric_candidate':3,'motion_completed_no_capture_evidence':2,
                     'experimental_limit_stop':1}
            chosen, families = [], set()
            for row in sorted(records,key=lambda r:(score.get(r['outcome'],-1),r.get('geometric_samples',0)),reverse=True):
                if row['outcome'] not in score or row['family'] in families: continue
                chosen.append(by_id[row['candidate_id']]);families.add(row['family'])
                if len(chosen)>=config['refine_parents']:break
            more, failed = generate_candidates(geometry,config['samples_per_family'],config['seed'],parents=chosen)
            more = [x for x in more if x['parent_id'] is not None]
            records.extend(dict(x,outcome='proposal_rejected') for x in failed if x.get('parent_id'))
            candidates += more; wave(more,config['physics_per_target'])
        write(run/'candidates.json',candidates)
        report['complete']=True
    except Exception as error:
        report.update(error=f'{type(error).__name__}: {error}', complete=False)
    finally:
        from search_model_cache import restore
        restore(out/'run')
        report.update(summary=summarize_target(records),physics_budget_charged=attempted,
                      wall_s=time.monotonic()-start,
                      source_unchanged=all(file_hash(Path(case['source_run'])/p)==h for p,h in case['hashes'].items()))
        if not report['source_unchanged']:report.update(complete=False,error='Source inputs changed')
        write(out/'case_result.json',report)
    return report


def publish(output, reports, expected_cases):
    output = Path(output)
    rows = [dict(scene=r['case']['scene_id'],target=r['case']['target'],**x) for r in reports for x in r['records']]
    summary = dict(schema=SCHEMA, expected_cases=expected_cases, reported_cases=len(reports),
                   completed_cases=sum(r['complete'] for r in reports),
                   case_statuses=Counter(r['summary']['status'] for r in reports),
                   candidate_outcomes=Counter(x['outcome'] for x in rows),
                   physics_attempts=sum(r['summary']['physics_attempts'] for r in reports),
                   physics_budget_charged=sum(r['physics_budget_charged'] for r in reports),
                   success_definition='No calibrated binary harvest label; evidence levels are separate',
                   reports=reports, training_eligible=False, hook_success=None)
    write(output/'results.json',summary)
    fields=['scene','target','candidate_id','family','outcome','physics_executed','completed','geometric_samples','simulated_s']
    with (output/'results.csv').open('w') as f:
        writer=csv.DictWriter(f,fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    body=[]
    for row in rows:
        link=''
        if row.get('trial_dir'):
            path=Path(row['trial_dir'])/'target_contact_trial.json'
            link=f"<a href='{html.escape(os.path.relpath(path,output))}'>evidence</a>"
        video=output/'cases'/row['scene']/row['target']/'videos'/(row['candidate_id']+'.mp4')
        if video.is_file():link += f" <a href='{html.escape(os.path.relpath(video,output))}'>video</a>"
        body.append('<tr>'+''.join('<td>'+html.escape(str(row.get(k,'')))+'</td>' for k in fields)+f'<td>{link}</td></tr>')
    page=("<!doctype html><meta charset='utf-8'><title>Motion-family search</title>"
          "<style>body{font-family:sans-serif;margin:2em}td,th{padding:.4em;border-bottom:1px solid #ccc}pre{white-space:pre-wrap}</style>"
          "<h1>Multi-scene motion-family search</h1><p>Geometry candidate is NOT hook success. "
          "Contact evidence is NOT real-robot validation. Search exhaustion is NOT impossibility.</p>"
          '<pre>'+html.escape(json.dumps({k:v for k,v in summary.items() if k!='reports'},indent=2))+'</pre>'
          '<table><tr>'+''.join('<th>'+k+'</th>' for k in fields)+'<th>detail</th></tr>'+''.join(body)+'</table>')
    (output/'index.html').write_text(page,encoding='utf-8')
    return summary


def render_representatives(reports,config,output):
    """Small balanced selection; record rendering failures without deleting physics."""
    queue=[q for r in reports for q in r['render_queue']]
    priority={'contact_retention_evidence':0,'geometric_candidate':1,'motion_completed_no_capture_evidence':2}
    queue.sort(key=lambda q:(priority.get(q['outcome'],3),q['candidate']))
    chosen=[];seen=set()
    for item in queue:
        if item['family'] not in seen:
            chosen.append(item);seen.add(item['family'])
    chosen += [x for x in queue if x not in chosen]
    selected=chosen[:config['render_budget']]
    for item in selected:
        dest=Path(item['output']);dest.parent.mkdir(parents=True,exist_ok=True)
        # Renderer needs JSON, not gzip; these are immutable derived copies.
        samples=Path(item['samples']);derived=dest.with_suffix('.samples.json')
        derived.write_bytes(gzip.decompress(samples.read_bytes()))
        try:
            item['render_returncode']=run_command([config['native_python'],str(SCRIPTS/'render_contact_diagnostic.py'),
                item['run'],'--candidate',item['candidate'],'--states',item['states'],'--samples',str(derived),
                '--output',str(dest),'--label',item['family']+' / '+item['outcome']],dest.with_suffix('.log'),config['trial_timeout_s'])
        except Exception as error:item['render_error']=str(error)
    write(Path(output)/'video_queue.json',dict(selected=selected,available=len(queue),budget=config['render_budget']))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path)
    p.add_argument('--discover-collection',type=Path)
    p.add_argument('--manifest-output',type=Path)
    p.add_argument('--output',type=Path)
    p.add_argument('--samples-per-family',type=int,default=4)
    p.add_argument('--seed',type=int,default=20260928)
    p.add_argument('--case-workers',type=int,default=1)
    p.add_argument('--planning-workers',type=int,default=2)
    p.add_argument('--planning-python',default='/root/isaaclab_env/bin/python')
    p.add_argument('--native-python',default=sys.executable)
    p.add_argument('--physics-per-target',type=int,default=12)
    p.add_argument('--refine-parents',type=int,default=2)
    p.add_argument('--planning-timeout-s',type=float,default=600.)
    p.add_argument('--trial-timeout-s',type=float,default=600.)
    p.add_argument('--case-timeout-s',type=float,default=3600.)
    p.add_argument('--render-budget',type=int,default=0)
    p.add_argument('--model-cache',type=Path,help='Optional hash-verified binary cache; durable model links restored after each case')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--max-target-force-n',type=float)
    p.add_argument('--max-target-displacement-m',type=float)
    p.add_argument('--resume',action='store_true')
    a=p.parse_args()
    if a.discover_collection:
        if not a.manifest_output:p.error('--manifest-output required')
        print(discover_collection(a.discover_collection,a.manifest_output));return
    if not a.manifest or not a.output:p.error('--manifest and --output required')
    from motion_family_search import parameters
    parameters(a.samples_per_family,a.seed)
    if not 1<=a.case_workers<=8 or not 1<=a.planning_workers<=8:p.error('worker counts must be 1..8')
    if not 1<=a.physics_per_target<=128 or not 0<=a.refine_parents<=4 or not 0<=a.render_budget<=20:p.error('Invalid declared budget')
    if any(not math.isfinite(v) or v<=0 for v in (a.planning_timeout_s,a.trial_timeout_s,a.case_timeout_s)):p.error('Invalid time budget')
    if a.execute and (a.max_target_force_n is None or a.max_target_displacement_m is None):p.error('Explicit experimental limits required for --execute')
    config=dict(samples_per_family=a.samples_per_family,seed=a.seed,planning_workers=a.planning_workers,
                planning_python=a.planning_python,native_python=a.native_python,physics_per_target=a.physics_per_target,
                refine_parents=a.refine_parents,planning_timeout_s=a.planning_timeout_s,trial_timeout_s=a.trial_timeout_s,
                case_timeout_s=a.case_timeout_s,render_budget=a.render_budget,execute=a.execute,
                max_target_force_N=a.max_target_force_n,max_target_displacement_m=a.max_target_displacement_m,
                model_cache=str(a.model_cache.resolve()) if a.model_cache else None)
    if a.execute:
        from target_fruit_contact_trial import TrialLimits
        TrialLimits(a.max_target_force_n,a.max_target_displacement_m)
    cases=load_cases(a.manifest)
    out=a.output.resolve()
    if any(out==Path(c['source_run']) or Path(c['source_run']) in out.parents for c in cases):p.error('Output must be outside source runs')
    code_hashes={name:file_hash(SCRIPTS/name) for name in (
        'run_motion_family_search.py','motion_family_search.py','target_fruit_contact_trial.py',
        'target_truss_identity.py','hook_retention_diagnostic.py','environment_preflight.py',
        'plan_candidates.py','robot_engine.py','measure_seating_rollout.py',
        'hook_seating_geometry.py','seating_evaluation.py','render_contact_diagnostic.py',
        'search_model_cache.py','search_legacy_baseline.py')}
    code_hashes['nvidia-sim/rl/dataset_motion.py']=file_hash(SCRIPTS.parents[1]/'nvidia-sim/rl/dataset_motion.py')
    signature=digest(dict(cases=cases,config=config,code_sha256=code_hashes))
    if out.exists():
        if not a.resume or read(out/'campaign.json')['signature']!=signature:p.error('Existing output/config conflict')
    else:
        out.mkdir(parents=True)
        write(out/'campaign.json',dict(schema=SCHEMA,cases=cases,config=config,signature=signature,
              source_manifest=str(a.manifest.resolve()),source_manifest_sha256=file_hash(a.manifest),
              code_sha256=code_hashes,
              source_scene_failures=read(a.manifest).get('source_scene_failures',[])))
    # Prevent two invocations from racing or spending the same campaign budget.
    import fcntl
    with (out/'campaign.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        reports=[]
        with ProcessPoolExecutor(max_workers=a.case_workers,mp_context=multiprocessing.get_context('spawn')) as pool:
            futures={pool.submit(run_case,case,config,str(out)):case for case in cases}
            for f in as_completed(futures):
                report=f.result();reports.append(report);publish(out,reports,len(cases))
                print(report['case']['scene_id'],report['case']['target'],report['summary'],flush=True)
        if a.render_budget:
            render_representatives(reports,config,out)
            publish(out,reports,len(cases))
    print(out/'index.html')


if __name__=='__main__':
    main()
