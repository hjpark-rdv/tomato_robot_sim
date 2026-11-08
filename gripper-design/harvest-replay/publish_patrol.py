#!/usr/bin/env python3
"""Bounded one-worker publication of finalized actual patrol recordings.

Uses the existing exporter, independent validator, run registration and initial
collection registration. Never starts physics, edits source records or renders.
Only this experiment's collection can be extended; catalog revisions are kept.

  nvidia-sim/.asset-venv/bin/python gripper-design/harvest-replay/publish_patrol.py \
      --campaign /path/to/new_controller_output --collection camera_patrol_ID --watch

Logs/STOP/ledger default to a sibling named new_controller_output_web_publication.
The controller alone creates its output. A terminal report triggers a final
flush and exit; unknown/nonterminal states wait only within the finite budget.
Original partial traces use the existing explicit diagnostic exporter and an
independent, non-default collection with zero harvest/checkpoint credit.
"""
import argparse
import math
import os
import re
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

WEB=Path(__file__).resolve().parent
REPO=WEB.parents[1]
sys.path[:0]=[str(WEB),str(REPO/'nvidia-sim/harvest_s300')]
from register_run import register
from register_collection import register_collection
from run_row_harvest import measured_harvested_trusses

CAMPAIGN_SCHEMA='s300-visible-camera-patrol-campaign-v1'
TERMINAL_STATUSES=frozenset(('pending_observation_or_finite_budget_blocker',
    'actual_physics_stopped_preserving_terminal_state','blocked_preserving_physical_history',
    'measured_distinct_truss_goal_reached','finite_decision_budget_exhausted'))


class ProducerPending(Exception):
    """The controller has not atomically finalized the next publication input."""


def checked_id(value):
    if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]{0,79}',value):
        raise ValueError('Use a short alphanumeric collection id with underscores/hyphens')
    return value


def campaign_config(campaign,collection):
    """Read an immutable SHA-proven parent chain; never create campaign files.

    Old manually coordinated runs without prior_campaign need an explicit
    --config. No directories are inferred from checkpoint paths or globs.
    """
    current=Path(campaign).resolve();root=current;seen=set();chain=[]
    while True:
        if root in seen or len(seen)>=32:
            raise ValueError('Invalid or unbounded patrol publication continuation chain')
        seen.add(root);path=root/'camera_patrol.json'
        if not path.is_file():
            if root==current:raise ProducerPending('Waiting for '+str(path))
            raise ValueError('Missing SHA-proven parent report: '+str(path))
        try:report=read(path)
        except json.JSONDecodeError:
            if root==current:raise ProducerPending('Controller report is being written')
            raise
        if (report.get('schema')!=CAMPAIGN_SCHEMA or report.get('training_eligible') is not False
                or report.get('harvest_success') is not None):
            raise ValueError('Unknown or ineligible patrol report: '+str(path))
        chain.append((root,report));parent=report.get('prior_campaign')
        if parent is None:break
        root=Path(parent['root']).resolve()
        for filename,key in [('camera_patrol.json','report_sha256'),('camera_patrol_queue.json','queue_sha256')]:
            if not (root/filename).is_file() or sha(root/filename)!=parent.get(key):
                raise ValueError('Prior patrol '+filename+' SHA mismatch')
        if parent.get('rollback') is not False:
            raise ValueError('Unproven prior campaign rollback policy')
        initial=report.get('initial_checkpoint') or {}
        if Path(initial.get('checkpoint','')).resolve()!=Path(parent['accepted_checkpoint']).resolve():
            raise ValueError('Prior campaign checkpoint differs from measured initial state')
    report=chain[0][1]
    return dict(schema='s300-patrol-publication-config-v1',campaigns=[str(p) for p,_ in reversed(chain)],
        collection_id=checked_id(collection),goal_distinct_trusses=report['goal_distinct_trusses'],
        default_after_verified_segments=3,reuse_verified_scenes=True,
        observed_campaign_status=report['status'],terminal=report['status'] in TERMINAL_STATUSES or
            (report['status']=='planned_only' and report.get('executed') is False),
        current_campaign=str(current),inherited_initial_checkpoint=chain[-1][1].get('initial_checkpoint'))


def read(path):return json.loads(Path(path).read_text())
def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def stamp():return datetime.now(timezone.utc).isoformat()
def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.tmp');temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');temporary.replace(path)


def source_proof(row,previous=None):
    """Do not publish early summary files before the native child has exited."""
    trial=Path(row['trial']).resolve();assembly=Path(row['assembly']).resolve()
    summary_path=trial/'summary.json';states_path=trial/'states.npz'
    results_path=trial.parent/(trial.name+'_cases_results.json')
    if not results_path.is_file():
        return None
    results=read(results_path);result=next((r for r in results if r.get('name')==trial.name),None)
    if (not result or type(result.get('timeout')) is not bool or type(result.get('returncode')) is not int):
        return None
    partial=None
    if not all(path.is_file() for path in (summary_path,states_path)):
        if not (trial/'partial_trace.json').is_file():return None
        from partial_recording import partial_recording,published_partial_source
        partial=partial_recording(assembly,trial);states_path=partial['states_path'];summary=partial['result']
        if result.get('summary') is not None:raise ValueError('Partial benchmark may not claim a finalized summary')
    else:
        summary=read(summary_path)
        if summary!=result.get('summary'):raise ValueError('Final benchmark/summary mismatch: '+str(trial))
    with __import__('numpy').load(states_path,allow_pickle=False) as data:
        times=data['time'];samples=len(times)
        if samples<2 or not __import__('numpy').isfinite(times).all() or not (__import__('numpy').diff(times)>0).all():
            raise ValueError('Actual state timestamps invalid: '+str(trial))
        duration=float(times[-1]-times[0])
    manifest=read(assembly/'motion_manifest.json')
    if (sha(assembly/'scene.usda')!=manifest['scene_sha256'] or
            (partial is None and summary['model_sha256']!=manifest['scene_sha256'])):
        raise ValueError('Actual scene identity mismatch: '+str(trial))
    proof=dict(trial=str(trial),assembly=str(assembly),summary_sha256=sha(summary_path) if partial is None else None,
        states_sha256=sha(states_path),manifest_sha256=sha(assembly/'motion_manifest.json'),
        benchmark_final=str(results_path),benchmark_final_sha256=sha(results_path),
        native_returncode=result['returncode'],native_timeout=result['timeout'],completed=summary.get('completed'),
        physical_stop_reason=summary.get('stop_reason'),frames=samples,duration_s=duration,
        counted_trusses=measured_harvested_trusses(summary,manifest['plant']['source_plant'])
            if partial is None and result['returncode']==0 and result['timeout'] is False else [],
        physics_rerun=False,training_eligible=False,harvest_success=None)
    if partial is not None:
        proof.update(record_kind='partial_diagnostic',diagnostic_only=True,harvest_credit=0,
            checkpoint_eligible=False,partial_source=published_partial_source(partial,trial))
    if previous is not None:
        from patrol_handoff import measured_handoff
        proof['handoff']=measured_handoff(trial,assembly,manifest,previous)
    return proof


def sources(config):
    rows=list(config.get('fixed_segments',[]))
    for folder in config['campaigns']:
        folder=Path(folder);path=folder/'replay_segments.json'
        if not path.is_file():continue
        record=read(path)
        if record.get('schema')!='s300-patrol-actual-replay-sources-v1':
            raise ValueError('Unknown actual replay source schema: '+str(path))
        for row in record['segments']:
            if row.get('independent_retry') is True:
                raise ValueError('Independent retry belongs in a separate collection')
            rows.append(dict(row,campaign=folder.name))
    unique=[];seen=set()
    for row in rows:
        key=str(Path(row['trial']).resolve())
        if key not in seen:unique.append(row);seen.add(key)
    return unique


class Publisher:
    def __init__(self,config,output,*,web=WEB):
        self.config=config;self.web=Path(web);self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.ledger_path=self.output/'ledger.json'
        self.ledger=read(self.ledger_path) if self.ledger_path.is_file() else dict(
            schema='s300-one-worker-actual-replay-publication-v1',created_at=stamp(),
            script=str(Path(__file__).resolve()),script_sha256=sha(__file__),
            tool_sha256={name:sha(WEB/name) for name in ['export_replay.py','validate_replay.py','register_run.py','register_collection.py']},
            published=[],diagnostics=[],errors=[],catalog_revisions=[],workers=1,physics_rerun=False,
            training_eligible=False,harvest_success=None)
        self.ledger.setdefault('diagnostics',[])
        self.pending=[];self.deadline=None
        self.save()

    def save(self):
        self.ledger['updated_at']=stamp();write(self.ledger_path,self.ledger)

    def subprocess_timeout(self):
        remaining=1200. if self.deadline is None else min(1200.,self.deadline-time.monotonic())
        if remaining<=0:raise TimeoutError('Bounded publication time budget expired')
        return remaining

    def audit_catalog(self,why):
        path=self.web/'catalog.json';before=self.output/'catalog_audit'/f"{len(self.ledger['catalog_revisions']):04d}_{why}.json"
        before.parent.mkdir(exist_ok=True);shutil.copy2(path,before)
        self.ledger['catalog_revisions'].append(dict(reason=why,before=str(before),sha256=sha(before),timestamp=stamp()))
        self.save()

    def refresh_collection(self,config):
        published=self.ledger['published']
        credited=sorted({key for row in published for key in row['source_proof']['counted_trusses']})
        goal=config.get('goal_distinct_trusses',6);count=len(credited)
        terminal=config.get('terminal') is True
        label=f"카메라 순회 · {count}/{goal}송이 · {len(published)}구간 · "+('목표 도달' if count>=goal else
            ('시험 종료 / 목표 미완료' if terminal else '진행 중 / 목표 미완료'))
        provenance=dict(schema='s300-incremental-actual-patrol-collection-v1',publication_ledger=str(self.ledger_path),
            measured_harvested_trusses=credited,achieved_distinct_trusses=count,goal_distinct_trusses=goal,
            goal_reached=count>=goal,training_eligible=False,harvest_success=None,
            continuity_note='실제 측정 상태와 제어 명령을 이어받은 구간입니다. 구간 시작 시 PhysX 솔버·접촉 캐시는 다시 초기화됩니다.',
            native_engine_cache_restart=True,bitwise_engine_continuity=False,
            observed_campaign_status=config.get('observed_campaign_status'),
            inherited_initial_checkpoint=config.get('inherited_initial_checkpoint'),
            source_trials=[row['source_proof']['trial'] for row in published])
        if published:self.extend_collection(config['collection_id'],label,published,provenance,
            'measured_state_handoff',len(published)>=config.get('default_after_verified_segments',3))
        diagnostics=self.ledger['diagnostics']
        if diagnostics:
            self.extend_collection(config['collection_id'][:67]+'_diagnostics','카메라 순회 · 중단된 실제 부분 기록 · 수확 집계 제외',
                diagnostics,dict(publication_ledger=str(self.ledger_path),harvest_credit=0,
                    checkpoint_eligible=False,diagnostic_only=True,training_eligible=False,
                    harvest_success=None),'independent_trials',False)
        self.ledger.update(collection_id=config['collection_id'],measured_harvested_trusses=credited,
            achieved_distinct_trusses=count,goal_reached=count>=goal,
            collection_url='./?collection='+config['collection_id'])
        self.save()

    def extend_collection(self,collection_id,label,published,provenance,continuity,make_default):
        ids=[row['run_id'] for row in published]
        catalog=read(self.web/'catalog.json')
        owned=next((row for row in catalog.get('collections',[]) if row['id']==collection_id),None)
        if owned is None:
            self.audit_catalog('create_collection')
            register_collection(collection_id,label,ids,root=self.web,continuity=continuity,
                provenance=provenance,make_default=make_default)
        else:
            if owned.get('provenance',{}).get('publication_ledger')!=str(self.ledger_path):
                raise ValueError('Refusing to modify another collection with the same id')
            if ids[:len(owned['runs'])]!=owned['runs']:
                raise ValueError('Published collection source order may only be extended')
            if (owned.get('runs')==ids and owned.get('label')==label and owned.get('provenance')==provenance
                    and (not make_default or catalog.get('default_collection')==collection_id)):
                return
            self.audit_catalog('extend_owned_collection')
            owned.update(runs=ids,label=label,provenance=provenance,updated_at=stamp())
            if make_default:
                catalog['default_collection']=collection_id;catalog['default_run']=ids[0]
            catalog['updated_at']=stamp();write(self.web/'catalog.json',catalog)

    def publish_once(self):
        config=self.config() if callable(self.config) else read(self.config)
        self.ledger.pop('waiting_for_producer',None)
        checked_id(config['collection_id']);rows=sources(config);self.pending=[]
        if self.ledger.get('collection_id',config['collection_id'])!=config['collection_id']:
            raise ValueError('Publication ledger already belongs to another collection')
        self.ledger.update(observed_campaign_status=config.get('observed_campaign_status'),
            producer_terminal=config.get('terminal') is True)
        existing={row['source_proof']['trial'] for row in self.ledger['published']+self.ledger['diagnostics']}
        for row in rows:
            key=str(Path(row['trial']).resolve())
            if key in existing:continue
            previous=self.ledger['published'][-1]['source_proof'] if self.ledger['published'] else None
            proof=source_proof(row,previous)
            if proof is None:
                self.pending.append(dict(trial=key,reason='Waiting for finalized actual native benchmark and source arrays'))
                break
            unchanged_proof=dict(proof)
            if row.get('requires_controller_upgrade_declaration'):
                declaration=Path(config['controller_upgrade_declaration']);declared=read(declaration)
                if (Path(declared['last_actual_trial']).resolve()!=Path(proof['trial']) or
                        declared['summary_sha256']!=proof['summary_sha256'] or declared['states_sha256']!=proof['states_sha256'] or
                        declared.get('physics_was_not_interrupted') is not True):
                    raise ValueError('Coordinator pause declaration does not identify finalized actual segment')
                proof['coordination_upgrade_declaration']=dict(path=str(declaration),sha256=sha(declaration))
            phase=row.get('phase','actual');campaign=row['campaign'];index=int(row['index'])
            identity=hashlib.sha256(key.encode()).hexdigest()[:10]
            suffix=f"_{index:03d}_{re.sub('[^A-Za-z0-9_-]','_',phase)[:12]}_{identity}"
            run_id=config['collection_id'][:80-len(suffix)]+suffix
            from patrol_labels import segment_label
            label,description,interpretation=segment_label(row,proof)
            partial=proof.get('record_kind')=='partial_diagnostic'
            bundle=self.web/'assets'/run_id;log=self.output/(run_id+'.log');validation=self.output/(run_id+'_validation.json')
            if not (bundle/'replay.json').is_file():
                if bundle.exists():
                    quarantine=self.output/'incomplete_exports'/f'{run_id}_{time.time_ns()}'
                    quarantine.parent.mkdir(exist_ok=True);shutil.move(bundle,quarantine)
                with log.open('a') as stream:
                    command=[sys.executable,str(WEB/'export_replay.py'),proof['assembly'],proof['trial'],
                        '--output',str(bundle),'--title',label]
                    if partial:command.append('--allow-partial-diagnostic')
                    if config.get('reuse_verified_scenes') and not partial:
                        runtime=read(Path(proof['trial'])/'runtime_settings.json')
                        for published in self.ledger['published']:
                            reference=self.web/'assets'/published['run_id']/'replay.json'
                            cached=read(reference)
                            if (not cached.get('geometry_reuse') and
                                    cached['source']['scene_sha256']==runtime['native_scene_sha256'] and
                                    cached['source']['manifest_sha256']==proof['manifest_sha256']):
                                command += ['--reuse-scene-from',str(reference)];break
                    subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=self.subprocess_timeout())
            exported_source=read(bundle/'replay.json').get('source',{})
            if (Path(exported_source.get('trial','')).resolve()!=Path(proof['trial']) or
                    Path(exported_source.get('assembly','')).resolve()!=Path(proof['assembly']) or
                    exported_source.get('states_sha256')!=proof['states_sha256'] or
                    exported_source.get('manifest_sha256')!=proof['manifest_sha256']):
                raise ValueError('Existing exported bundle is not bound to the selected actual source')
            # A crashed worker may leave a validation file before registration.
            # Preserve it, but independently validate current bytes again rather
            # than treating that old filename as a trustworthy cache.
            if validation.exists():
                history=self.output/'validation_history';history.mkdir(exist_ok=True)
                shutil.move(validation,history/(validation.stem+'_'+str(time.time_ns())+'.json'))
            with log.open('a') as stream:
                command=[sys.executable,str(WEB/'validate_replay.py'),str(bundle),'--output',str(validation)]
                if partial:command.append('--allow-partial-diagnostic')
                subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=self.subprocess_timeout())
            validated=read(validation)
            if (validated.get('passed') is not True or
                    Path(validated.get('source_trial','')).resolve()!=Path(proof['trial']) or
                    Path(validated.get('source_assembly','')).resolve()!=Path(proof['assembly']) or
                    validated.get('source_states_sha256')!=proof['states_sha256']):
                raise ValueError('Replay independent validation failed or identifies another actual source')
            # Readiness cannot mutate original data while a bundle is being exported.
            if source_proof(row,previous)!=unchanged_proof:
                raise ValueError('Source changed during export')
            current=read(self.web/'catalog.json')
            registered=next((item for item in current['runs'] if item['id']==run_id),None)
            if registered is None:
                self.audit_catalog('register_'+run_id)
                register(proof['assembly'],proof['trial'],run_id,label,root=self.web,existing=True,
                    allow_partial_diagnostic=partial)
            elif Path(registered['source_trial']).resolve()!=Path(proof['trial']):
                raise ValueError('Existing catalog id maps to another actual source')
            if description:
                self.audit_catalog('describe_'+run_id)
                described=read(self.web/'catalog.json')
                target=next(item for item in described['runs'] if item['id']==run_id)
                target.update(label=label,description=description,publication_interpretation=interpretation)
                write(self.web/'catalog.json',described)
            item=dict(run_id=run_id,source_row=row,source_proof=proof,validation=str(validation),
                validation_sha256=sha(validation),published_at=stamp())
            self.ledger['diagnostics' if partial else 'published'].append(item);self.save();self.refresh_collection(config)
            print('[PUBLISHED]',run_id,proof['frames'],'frames',proof['duration_s'],'seconds',flush=True)
            return True
        self.refresh_collection(config)
        videos=[]
        for item in self.ledger['published']+self.ledger['diagnostics']:
            videos.extend(str(p) for p in Path(item['source_proof']['trial']).rglob('*.mp4'))
        write(self.output/'videos.json',dict(videos=videos,actual_recorded_state_videos_only=True))
        (self.output/'videos.md').write_text('\n'.join('- ['+p+']('+p+')' for p in videos)+'\n')
        self.ledger['unpublished_sources']=self.pending;self.save()
        return False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    inputs=parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--campaign',type=Path,help='New controller output; waits without creating it')
    inputs.add_argument('--config',type=Path,help='Explicit ordered campaign/fixed-segment configuration')
    parser.add_argument('--collection',help='Required with --campaign; generated safely by the calling shell')
    parser.add_argument('--output',type=Path,help='Publication logs/ledger; defaults to a sibling of the campaign')
    modes=parser.add_mutually_exclusive_group();modes.add_argument('--watch',action='store_true')
    modes.add_argument('--once',action='store_true',help='Publish all currently finalized sources, then return')
    parser.add_argument('--max-hours',type=float,default=8.,help='Finite watcher lifetime, at most 8 hours')
    args=parser.parse_args()
    if args.campaign is not None:
        if not args.collection:parser.error('--campaign requires --collection')
        checked_id(args.collection)
        campaign=args.campaign.resolve()
        output=args.output or campaign.with_name(campaign.name+'_web_publication')
        if not campaign.exists() and Path(output).resolve().is_relative_to(campaign):
            parser.error('Publication output must be outside the not-yet-created campaign root')
        configuration=lambda:campaign_config(campaign,args.collection)
    else:
        if args.collection:parser.error('--collection is defined by --config')
        output=args.output or args.config.resolve().parent/'publication'
        configuration=lambda:explicit_config(args.config)
    if not math.isfinite(args.max_hours) or not 0<args.max_hours<=8:
        parser.error('--max-hours must be finite and within (0, 8]')
    # A publisher never starts/steps physics. Its own lock also prevents two
    # workers from exporting or mutating the same collection concurrently.
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    import fcntl
    with (output/'worker.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Another publisher owns this output ledger')
        publisher=Publisher(configuration,output)
        run_watcher(publisher,watch=args.watch,max_hours=args.max_hours)


def explicit_config(path):
    config=read(path)
    checked_id(config['collection_id'])
    campaigns=config.get('campaigns',[])
    if campaigns and (Path(campaigns[-1])/'camera_patrol.json').is_file():
        report=read(Path(campaigns[-1])/'camera_patrol.json')
        config.update(observed_campaign_status=report['status'],terminal=report['status'] in TERMINAL_STATUSES or
            (report['status']=='planned_only' and report.get('executed') is False))
    return config


def run_watcher(publisher,*,watch=False,max_hours=8.,poll_seconds=3.,clock=time.monotonic,sleep=time.sleep):
    if not math.isfinite(max_hours) or not 0<max_hours<=8:
        raise ValueError('Finite publication budget must be within (0, 8] hours')
    output=publisher.output
    publisher.ledger['status']='running';publisher.ledger['current_script_sha256']=sha(__file__)
    publisher.ledger['handoff_adapter']=dict(path=str(WEB/'patrol_handoff.py'),sha256=sha(WEB/'patrol_handoff.py'))
    publisher.ledger['label_adapter']=dict(path=str(WEB/'patrol_labels.py'),sha256=sha(WEB/'patrol_labels.py'))
    publisher.save()
    write(output/'process.json',dict(pid=os.getpid(),started_at=stamp(),
        script=str(Path(__file__).resolve()),script_sha256=sha(__file__),max_hours=max_hours,
        stop_file=str(output/'STOP'),workers=1,physics_processes_started=0))
    deadline=clock()+max_hours*3600;publisher.deadline=time.monotonic()+max_hours*3600;status='time_budget_exhausted'
    while clock()<deadline:
        if (output/'STOP').exists():status='stopped_by_control';break
        try:
            changed=publisher.publish_once()
        except ProducerPending as error:
            publisher.ledger['waiting_for_producer']=str(error);publisher.save();changed=False
        except json.JSONDecodeError as error:
            if publisher.ledger.get('producer_terminal'):
                publisher.ledger['errors'].append(dict(timestamp=stamp(),type=type(error).__name__,error=str(error)))
                publisher.ledger['status']='stopped_on_publication_error';publisher.save();raise
            # Active controller writes short JSON reports; retry a torn write.
            publisher.ledger['waiting_for_producer']='Controller JSON write in progress';publisher.save();changed=False
        except Exception as error:
            publisher.ledger['errors'].append(dict(timestamp=stamp(),type=type(error).__name__,error=str(error)))
            publisher.ledger['status']='stopped_on_publication_error';publisher.save()
            raise
        if not changed:
            if publisher.ledger.get('producer_terminal'):
                status='terminal_unpublished_sources' if publisher.pending else 'terminal_final_flush_complete'
                break
            if not watch:
                status='waiting_for_producer' if publisher.ledger.get('waiting_for_producer') else 'current_finalized_sources_published'
                break
            sleep(min(poll_seconds,max(0.,deadline-clock())))
    publisher.ledger['status']=status
    publisher.save();print('[WATCHER STOP]',publisher.ledger['status'],flush=True)
    if status=='terminal_unpublished_sources':
        raise RuntimeError('Terminal campaign retains actual sources that failed publication readiness; inspect ledger')
    return publisher.ledger


if __name__=='__main__':main()
