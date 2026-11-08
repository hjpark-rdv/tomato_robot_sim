#!/usr/bin/env python3
"""Publish one existing saved physics run in the harvest replay selector.

Uses export_replay.py; never runs physics or changes a recorded result.
Run with nvidia-sim/.asset-venv/bin/python.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def register(assembly, trial, run_id, label, *, root=ROOT, existing=False, sensor_data=None,
             allow_partial_diagnostic=False):
    if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]{0,79}', run_id):
        raise ValueError('Use a short alphanumeric run id with underscores/hyphens')
    root=Path(root).resolve()
    catalog_path=root/'catalog.json'
    catalog=json.loads(catalog_path.read_text()) if catalog_path.exists() else dict(schema='farmily-harvest-replay-catalog-v1',runs=[])
    if any(row['id']==run_id for row in catalog['runs']):
        raise ValueError('This run id is already registered; use a new id to preserve it')
    output=root/'assets'/run_id
    if not existing:
        if output.exists():raise FileExistsError(output)
        output.parent.mkdir(parents=True,exist_ok=True)
        command=[sys.executable,str(ROOT/'export_replay.py'),str(assembly),str(trial),'--output',str(output),'--title',label]
        if sensor_data is not None:
            command += ['--sensor-data',str(sensor_data)]
        if allow_partial_diagnostic:
            command += ['--allow-partial-diagnostic']
        subprocess.run(command,check=True)
    replay=output/'replay.json'
    if not replay.is_file():raise FileNotFoundError(replay)
    record=json.loads(replay.read_text())
    partial = record.get('source',{}).get('record_kind') == 'partial_diagnostic'
    if partial:
        if not allow_partial_diagnostic:
            raise ValueError('Partial diagnostic registration requires explicit opt-in')
        from partial_recording import partial_recording, published_partial_source
        original=partial_recording(assembly,trial)
        expected=published_partial_source(original,trial)
        if (any(record.get('source',{}).get(key)!=value for key,value in expected.items()) or
                record.get('frames')!=original['descriptor']['samples'] or
                record.get('duration_s')!=original['descriptor']['last_timestamp_s']-original['descriptor']['first_timestamp_s']):
            raise ValueError('Partial registration source/sample identity mismatch')
        result=record.get('result_summary',{})
        if (result.get('completed') is not False or result.get('diagnostic_only') is not True or
                result.get('harvest_credit') != 0 or result.get('checkpoint_eligible') is not False or
                result.get('training_eligible') is not False or result.get('harvest_success') is not None):
            raise ValueError('Partial recording must remain diagnostic-only with zero harvest/checkpoint credit')
        if result!=original['result']:
            raise ValueError('Partial registration result differs from its original incomplete evidence')
    engine=record.get('engine','Recorded physics')
    if isinstance(engine,dict):
        engine=engine.get('physics_engine',engine.get('name','Recorded physics'))
    for directory in (output,output.parent):
        if not directory.resolve().is_relative_to(root):raise ValueError('Publish files inside the web root, without outside symlinks')
    catalog['runs'].append(dict(id=run_id,label=label,replay_url='./assets/'+run_id+'/replay.json',
                               engine=engine,duration_s=record.get('duration_s'),frames=record.get('frames'),
                               source_trial=str(Path(trial).resolve()),source_assembly=str(Path(assembly).resolve())))
    if partial:
        catalog['runs'][-1].update(record_kind='partial_diagnostic',diagnostic_only=True,
                                   harvest_credit=0,checkpoint_eligible=False,training_eligible=False)
    catalog['updated_at']=datetime.now(timezone.utc).isoformat()
    temporary=catalog_path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(catalog,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(catalog_path)
    print(json.dumps(dict(id=run_id,replay_url=catalog['runs'][-1]['replay_url'],physics_rerun=False),ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('assembly',type=Path);p.add_argument('trial',type=Path)
    p.add_argument('--id',required=True);p.add_argument('--label',required=True)
    p.add_argument('--existing',action='store_true',help='Register an already exported assets/ID bundle')
    p.add_argument('--sensor-data',type=Path,help='Use a source-SHA identified camera capture directory')
    p.add_argument('--allow-partial-diagnostic',action='store_true',help='Publish incomplete actual samples only as an explicit diagnostic record')
    a=p.parse_args();register(a.assembly,a.trial,a.id,a.label,existing=a.existing,sensor_data=a.sensor_data,
                             allow_partial_diagnostic=a.allow_partial_diagnostic)
