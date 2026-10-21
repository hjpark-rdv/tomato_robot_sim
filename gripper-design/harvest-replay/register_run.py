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


def register(assembly, trial, run_id, label, *, root=ROOT, existing=False):
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
        subprocess.run([sys.executable,str(ROOT/'export_replay.py'),str(assembly),str(trial),'--output',str(output),'--title',label],check=True)
    replay=output/'replay.json'
    if not replay.is_file():raise FileNotFoundError(replay)
    record=json.loads(replay.read_text())
    engine=record.get('engine','Recorded physics')
    if isinstance(engine,dict):
        engine=engine.get('physics_engine',engine.get('name','Recorded physics'))
    for directory in (output,output.parent):
        if not directory.resolve().is_relative_to(root):raise ValueError('Publish files inside the web root, without outside symlinks')
    catalog['runs'].append(dict(id=run_id,label=label,replay_url='./assets/'+run_id+'/replay.json',
                               engine=engine,duration_s=record.get('duration_s'),frames=record.get('frames'),
                               source_trial=str(Path(trial).resolve()),source_assembly=str(Path(assembly).resolve())))
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
    a=p.parse_args();register(a.assembly,a.trial,a.id,a.label,existing=a.existing)
