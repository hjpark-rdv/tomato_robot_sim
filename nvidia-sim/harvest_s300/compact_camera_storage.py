#!/usr/bin/env python3
"""Compact completed RGB-D captures / web replays, preserving numerical bytes.

Dry-run by default. --apply is an offline migration; do not run on active
recordings. Source frames.jsonl, physics states, input USDs, contacts, result
JSON, images and movies are not rewritten. Per-frame storage.json preserves
old NPY hashes and resolves their old logical names. --restore-arrays restores
ordinary NPY files without deleting the compressed archives.
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import gzip
import hashlib
import json
import os
from pathlib import Path
import time

from recording_storage import (ARRAY_NAMES, atomic_bytes, compact_array,
                               restore_array, save_shared_labels, share_file)


def sha(path, compressed=False):
    with (gzip.open(path, 'rb') if compressed else Path(path).open('rb')) as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def inventory(root):
    files = 0
    unique = {}
    apparent = 0
    for folder, dirs, names in os.walk(root, followlinks=False):
        dirs[:] = [d for d in dirs if not Path(folder, d).is_symlink()]
        for name in names:
            path = Path(folder, name)
            if path.is_symlink():
                continue
            s = path.stat()
            files += 1
            apparent += s.st_size
            unique[(s.st_dev, s.st_ino)] = (s.st_size, s.st_blocks * 512)
    return dict(files=files, logical_bytes=apparent,
                unique_inode_bytes=sum(n for n, _ in unique.values()),
                allocated_bytes=sum(n for _, n in unique.values()))


def protected_hashes(root):
    """Recorded results, poses, clocks and camera observations stay unchanged."""
    names = {'states.npz', 'states_partial.npz', 'partial_states.npz', 'summary.json', 'runtime_settings.json',
             'break_events.json', 'cumulative_break_events.json', 'frames.jsonl',
             'camera.json', 'objects.json', 'contacts.json', 'checkpoint.npz',
             'gt_catalog.json', 'region.json', 'renderer_labels.json',
             'checkpoint.json', 'motion_manifest.json', 'composed.usdc', 'scene.usda'}
    return {str(p.relative_to(root)): sha(p) for p in root.rglob('*')
            if p.is_file() and (p.name in names or p.suffix in ('.mp4', '.png'))}


def backup(path, report):
    raw = path.read_bytes()
    report['metadata_backups'][str(path)] = dict(encoding='gzip-base64',
        sha256=hashlib.sha256(raw).hexdigest(),
        data=base64.b64encode(gzip.compress(raw, mtime=0)).decode())


def compact_camera(root, report, apply=False, workers=4):
    # An interruption may have already compacted depth.npy while masks remain.
    frames = sorted({p.parent for p in root.rglob('*.npy')
                     if p.name in ARRAY_NAMES and p.parent.name.startswith('frame_')})
    def compact_frame(frame):
        result = []
        for name in sorted(ARRAY_NAMES):
            path = frame / name
            if path.is_file():
                result.append(compact_array(path) if apply else
                              dict(path=str(path), original_bytes=path.stat().st_size))
        return result
    # Independent completed frames only; metadata writes within one frame stay
    # sequential. Bound NAS I/O concurrency rather than spawn simulators.
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for index, result in enumerate(pool.map(compact_frame, frames)):
            report['arrays'].extend(result)
            if index % 100 == 0:
                print(f'[arrays] {index}/{len(frames)}', flush=True)
    # Share exact label dictionaries only inside their own capture root. This
    # works on SSHFS, which does not report remote hardlink inode identity.
    label_paths=sorted(root.rglob('renderer_labels.json'))
    for index, path in enumerate(label_paths):
        if index % 100 == 0:
            print(f'[labels] {index}/{len(label_paths)}', flush=True)
        if path.is_symlink():
            continue
        raw = path.read_bytes()
        labels = json.loads(raw)
        if (json.dumps(labels, indent=2)+'\n').encode() != raw:
            report['unmodified_noncanonical_labels'].append(str(path))
            continue
        capture = next((p for p in path.parents if p.name == 'rgbd'), None)
        if capture is None:
            continue
        report['labels'].append(dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest(), bytes=len(raw)))
        if apply:
            save_shared_labels(path, labels, capture/'shared_labels')
    if apply:
        for page in root.rglob('rgbd/index.html'):
            # Existing review pages have fixed legacy raw-array links. Keep
            # their image/video data and replace only array download URLs.
            text = page.read_text()
            token = "ids:'instance.npy',classes:'semantic.npy',depth:'depth.npy'"
            frame_dirs = list(page.parent.glob('frame_*'))
            if token in text and frame_dirs and all(
                    all((f/(n+'.gz')).is_file() for n in ('instance.npy','semantic.npy','depth.npy'))
                    for f in frame_dirs if f.is_dir()):
                backup(page, report)
                text = text.replace(token, "ids:'instance.npy.gz',classes:'semantic.npy.gz',depth:'depth.npy.gz'")
                text = text.replace('ID NPY</a>', 'ID NPY/GZ</a>').replace('깊이(m) NPY</a>', '깊이(m) NPY/GZ</a>')
                atomic_bytes(page, text.encode())


def compact_web(root, report, apply=False):
    for path in sorted(root.glob('*/replay.json')):
        replay = json.loads(path.read_text())
        raw = path.parent/'scene.glb'
        packed = path.parent/'scene.glb.gz'
        if not raw.is_file():
            continue
        info_path = path.parent / replay['scene_info_url']
        info = json.loads(info_path.read_text())
        raw_sha = sha(raw)
        if raw_sha != info['sha256'] or sha(packed, True) != raw_sha or sha(packed) != info['compressed_sha256']:
            raise ValueError('Web geometry identity mismatch: '+str(path))
        report['models'].append(dict(path=str(raw), original_sha256=raw_sha,
                                     original_bytes=raw.stat().st_size, packed_bytes=packed.stat().st_size))
        if apply:
            backup(path, report)
            backup(info_path, report)
            replay.update(scene_url='scene.glb.gz', scene_encoding='gzip', scene_gzip_url='scene.glb.gz')
            info.update(file='scene.glb.gz', encoding='gzip', bytes=raw.stat().st_size)
            atomic_bytes(info_path, (json.dumps(info, ensure_ascii=False, indent=2)+'\n').encode())
            atomic_bytes(path, (json.dumps(replay, ensure_ascii=False, indent=2)+'\n').encode())
            # Persist originals of the small manifests before removing a
            # redundant scene; an interruption can still be reversed.
            atomic_bytes(report['report_path'], (json.dumps(report, indent=2)+'\n').encode())
            raw.unlink()
    if apply:
        pool = root/'_shared'
        # Hash same-size assets, including models, camera images/JSON and MP4s.
        # Do not hardlink mutable manifests or source physics recordings.
        allowed = {'.png', '.mp4', '.gz'}
        names = {'objects.json', 'gt_catalog.json'}
        candidates = [p for p in root.rglob('*') if pool not in p.parents and p.is_file()
                      and not p.is_symlink() and (p.suffix in allowed or p.name in names)]
        sizes = Counter(p.stat().st_size for p in candidates)
        for index, p in enumerate(candidates):
            if sizes[p.stat().st_size] > 1:
                share_file(p, pool)
                report['shared_files'] += 1
            if index % 500 == 0:
                print(f'[share] {index}/{len(candidates)}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--web', action='store_true')
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--restore-arrays', action='store_true')
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--workers', type=int, choices=range(1,9), default=4)
    args = parser.parse_args()
    root = args.root.resolve(strict=True)
    if args.report.exists():
        raise FileExistsError('Use a new audit report: '+str(args.report))
    report = dict(schema='s300-storage-compaction-v1', root=str(root), apply=args.apply,
        report_path=str(args.report.resolve()), before=inventory(root), arrays=[], labels=[], models=[],
        unmodified_noncanonical_labels=[], metadata_backups={}, shared_files=0, completed=False)
    started = time.monotonic()
    # All protected SHA values are captured before the first mutation.
    if args.apply:
        print('[audit] hashing original evidence', flush=True)
        report['protected_sha256'] = protected_hashes(root)
    atomic_bytes(args.report, (json.dumps(report, indent=2)+'\n').encode())
    try:
        if args.restore_arrays:
            if not args.apply:
                raise ValueError('--restore-arrays requires --apply')
            for path in root.rglob('storage.json'):
                record = json.loads(path.read_text())
                if record.get('schema') == 's300-lossless-camera-storage-v1':
                    for name in record['arrays']:
                        restore_array(path.parent/name)
        elif args.web:
            compact_web(root, report, args.apply)
        else:
            compact_camera(root, report, args.apply, args.workers)
        if args.apply:
            print('[audit] verifying preserved evidence', flush=True)
            for index, (relative, digest) in enumerate(report['protected_sha256'].items()):
                if sha(root/relative) != digest:
                    raise ValueError('Protected evidence changed: '+relative)
                if index % 1000 == 0:
                    print(f'[verify] {index}/{len(report["protected_sha256"])}', flush=True)
            report['protected_evidence_unchanged'] = True
        report.update(completed=True, after=inventory(root), wall_s=time.monotonic()-started)
    finally:
        atomic_bytes(args.report, (json.dumps(report, indent=2)+'\n').encode())
    print(json.dumps({k:report[k] for k in ('completed','before','after','wall_s')}, indent=2))


if __name__ == '__main__':
    main()
