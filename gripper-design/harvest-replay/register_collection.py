#!/usr/bin/env python3
"""Group already registered actual runs, preserving each scene and time axis.

Creates a catalog selector only. It neither stitches states nor renders or
starts physics. Run after register_run.py has published every actual segment.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
from urllib.parse import urlsplit, unquote, quote

ROOT = Path(__file__).resolve().parent


def register_collection(collection_id, label, runs, *, root=ROOT, source_report=None, provenance=None,
                        continuity='unknown', diagnostics=(), make_default=False):
    if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]{0,79}',collection_id):
        raise ValueError('Use a short alphanumeric collection id with underscores/hyphens')
    if not label.strip():
        raise ValueError('Collection label is required')
    if not runs or len(set(runs)) != len(runs):
        raise ValueError('Supply one or more unique registered run ids in actual execution order')
    if continuity not in ('measured_state_handoff','independent_trials','unknown'):
        raise ValueError('Unknown collection continuity; do not imply continuous physics without evidence')
    if not isinstance(make_default,bool):
        raise ValueError('make_default must be an explicit boolean')
    root = Path(root).resolve()
    catalog_path = root/'catalog.json'
    catalog = json.loads(catalog_path.read_text())
    if catalog.get('schema') != 'farmily-harvest-replay-catalog-v1':
        raise ValueError('Unknown replay catalog schema')
    registered = {row['id']:row for row in catalog['runs']}
    unknown = set(runs)-registered.keys()
    if unknown:
        raise ValueError('Register actual run bundles first: '+', '.join(sorted(unknown)))
    collections = catalog.setdefault('collections',[])
    if any(row['id'] == collection_id for row in collections):
        raise ValueError('This collection id is already registered; use a new id to preserve it')
    for run_id in runs:
        # Match register_run's safe, local bundle contract and ensure catalog
        # membership is not mistaken for evidence of an existing recording.
        url = urlsplit(registered[run_id]['replay_url'])
        if url.scheme or url.netloc:
            raise ValueError('Collection requires locally registered actual replay bundles')
        bundle = (root/unquote(url.path)).resolve()
        if not bundle.is_relative_to(root) or not bundle.is_file():
            raise ValueError('Missing local replay bundle: '+run_id)
        replay = json.loads(bundle.read_text())
        if replay.get('schema') != 'farmily-harvest-replay-v1':
            raise ValueError('Unknown replay bundle schema: '+run_id)
        if not replay.get('source',{}).get('all_frames_exported') or replay.get('source',{}).get('physics_rerun') is not False:
            raise ValueError('Collection requires preserved actual states: '+run_id)
        if replay.get('source',{}).get('record_kind') == 'partial_diagnostic':
            if continuity != 'independent_trials' or make_default:
                raise ValueError('Partial diagnostics belong only to a non-default independent diagnostic collection')
    now = datetime.now(timezone.utc).isoformat()
    record = dict(id=collection_id,label=label,runs=list(runs),created_at=now,
                  continuity=continuity,
                  playback='ordered individual recorded scenes; no stitched or simulated state',
                  physics_rerun=False)
    if source_report is not None:
        source_report = Path(source_report).resolve()
        if not source_report.is_file():
            raise FileNotFoundError(source_report)
        record['source_report'] = str(source_report)
    if provenance is not None:
        if not isinstance(provenance,dict):
            raise ValueError('Collection provenance must be a JSON object')
        record['provenance'] = provenance
    attachments=[]
    if source_report is not None:
        attachments.append(('report',source_report))
    for index,filename in enumerate(diagnostics):
        path=Path(filename).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        attachments.append((f'diagnostic_{index:02d}',path))
    if attachments:
        directory=root/'assets'/collection_id
        if not directory.resolve().is_relative_to(root):
            raise ValueError('Collection report assets must stay inside the web root')
        directory.mkdir(parents=True,exist_ok=False)
        diagnostics_urls=[]
        for category,source in attachments:
            suffix=re.sub(r'[^A-Za-z0-9_.-]','_',source.name)
            name=category+'_'+suffix
            target=directory/name
            before=hashlib.sha256(source.read_bytes()).hexdigest()
            shutil.copyfile(source,target)
            if hashlib.sha256(target.read_bytes()).hexdigest()!=before:
                raise ValueError('Report file changed during copying: '+str(source))
            url='./assets/'+collection_id+'/'+quote(name)
            entry=dict(label=source.name,url=url,sha256=before)
            if category=='report':
                record['report_url']=url
                record['report_sha256']=before
            else:
                diagnostics_urls.append(entry)
        if diagnostics_urls:
            record['diagnostics_urls']=diagnostics_urls
    collections.append(record)
    if make_default:
        # Set both selector defaults in the same catalog transaction as the
        # actual collection. Ordinary registrations preserve existing defaults.
        catalog['default_collection'] = collection_id
        catalog['default_run'] = runs[0]
    catalog['updated_at'] = now
    temporary = catalog_path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(catalog,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(catalog_path)
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--id',required=True)
    p.add_argument('--label',required=True)
    p.add_argument('--runs',nargs='+',required=True)
    p.add_argument('--source-report','--report',dest='source_report',type=Path,
                   help='Copy this report into the web bundle; catalog report_url is served locally')
    p.add_argument('--diagnostics',type=Path,nargs='+',default=[],help='Explicit log/ledger files to copy into the collection web assets')
    p.add_argument('--continuity',choices=['measured_state_handoff','independent_trials','unknown'],default='unknown')
    p.add_argument('--provenance',type=Path,help='JSON object identifying experiment and actual execution order')
    p.add_argument('--make-default',action='store_true',help='Atomically select this actual collection and its first run for visitors without a selection query')
    a=p.parse_args()
    provenance=json.loads(a.provenance.read_text()) if a.provenance else None
    print(json.dumps(register_collection(a.id,a.label,a.runs,source_report=a.source_report,
                     provenance=provenance,continuity=a.continuity,diagnostics=a.diagnostics,
                     make_default=a.make_default),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
