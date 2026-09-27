"""Optional immutable binary cache for search on network-backed snapshots.

Only newly created run symlinks change. Persisted source assets are never edited.
Restore durable links after each case so completed results survive cache removal.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def attach(run, source, expected, directory):
    source=Path(source).resolve(); directory=Path(directory).resolve()
    directory.mkdir(parents=True,exist_ok=True)
    cached=directory/(expected+'.mjb')
    with (directory/(expected+'.lock')).open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if cached.exists():
            if sha(cached)!=expected:raise ValueError('Corrupt model cache; not overwritten')
        else:
            temp=cached.with_suffix(f'.{os.getpid()}.tmp')
            try:
                with source.open('rb') as a,temp.open('xb') as b:shutil.copyfileobj(a,b,1<<20)
                if sha(temp)!=expected:raise ValueError('Model source differs from manifest')
                temp.replace(cached)
            finally:
                if temp.exists():temp.unlink()
    link=Path(run)/'replay_assets/model.mjb'
    if not link.is_symlink() or link.resolve()!=source:raise ValueError('Cache only a new snapshot source symlink')
    meta=link.with_name('model_cache.json')
    meta.write_text(json.dumps(dict(durable_source=str(source),sha256=expected,cache=str(cached),
                                   restored=False),indent=2))
    link.unlink();link.symlink_to(cached)


def restore(run):
    link=Path(run)/'replay_assets/model.mjb';meta=link.with_name('model_cache.json')
    if not meta.exists():return
    info=json.loads(meta.read_text())
    if not link.is_symlink() or link.resolve() not in (Path(info['cache']),Path(info['durable_source'])):
        raise ValueError('Unexpected model link while restoring cache')
    link.unlink();link.symlink_to(info['durable_source'])
    info['restored']=True;meta.write_text(json.dumps(info,indent=2))
