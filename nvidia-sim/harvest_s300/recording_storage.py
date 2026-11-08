"""Lossless camera storage and byte-identical sharing (no simulator imports).

Old captures keep their original manifests and SHA evidence. load_array accepts
both legacy .npy paths and gzip-backed logical .npy paths; storage.json records
the exact original file hash. gzip -dk file.npy.gz also restores an ordinary NPY.
"""
from __future__ import annotations

import gzip
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
import os
import stat
from pathlib import Path
import tempfile

import numpy as np

ARRAY_NAMES = {'depth.npy', 'instance.npy', 'renderer_instance.npy', 'semantic.npy'}


def atomic_bytes(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    try:
        os.fchmod(fd, stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o644)
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def array_path(path):
    path = Path(path)
    if path.is_file():
        return path
    compressed = path.with_name(path.name + '.gz')
    if path.suffix == '.npy' and compressed.is_file():
        return compressed
    raise FileNotFoundError(path)


def original_bytes(path):
    actual = array_path(path)
    data = actual.read_bytes()
    return gzip.decompress(data) if actual.name.endswith('.npy.gz') else data


def load_array(path):
    """Retain dtype, shape, endian, NaN payloads and every floating-point bit."""
    actual = array_path(path)
    if actual.name.endswith('.gz'):
        with gzip.open(actual, 'rb') as stream:
            return np.load(stream, allow_pickle=False)
    return np.load(actual, allow_pickle=False)


def _save_raw_array(path, raw, *, record=True):
    path = Path(path)
    compressed = path.with_name(path.name + '.gz')
    packed = gzip.compress(raw, compresslevel=1, mtime=0)
    if compressed.exists() and gzip.decompress(compressed.read_bytes()) != raw:
        raise ValueError('Existing compressed array differs: ' + str(compressed))
    atomic_bytes(compressed, packed)
    # Verify the file read back from storage, before ever unlinking a legacy NPY.
    if gzip.decompress(compressed.read_bytes()) != raw:
        raise ValueError('Lossless verification failed: ' + str(compressed))
    if not record:
        return compressed
    manifest = path.parent / 'storage.json'
    record = json.loads(manifest.read_text()) if manifest.exists() else dict(
        schema='s300-lossless-camera-storage-v1', arrays={})
    record['arrays'][path.name] = dict(file=compressed.name, encoding='gzip-npy',
        original_sha256=hashlib.sha256(raw).hexdigest(), original_bytes=len(raw),
        stored_sha256=hashlib.sha256(packed).hexdigest(), stored_bytes=len(packed))
    atomic_bytes(manifest, (json.dumps(record, indent=2) + '\n').encode())
    return compressed


def save_array(path, array):
    path = Path(path)
    if path.exists():
        raise FileExistsError(path)
    stream = io.BytesIO()
    np.save(stream, array, allow_pickle=False)
    # New recordings already name .npy.gz in their camera/GT manifests. Only
    # legacy migrations need storage.json to preserve old logical NPY hashes.
    # Match the old recorder's new-file write semantics: no NAS fsync/readback
    # round trips per sensor channel. The frame is published only after all
    # files close. Migration, which removes existing originals, stays stricter.
    packed = gzip.compress(stream.getvalue(), compresslevel=1, mtime=0)
    compressed = path.with_name(path.name + '.gz')
    with compressed.open('xb') as output:
        output.write(packed)
    return compressed


def save_arrays(directory, arrays, *, labels=None, label_pool=None, verified=None):
    """Bounded independent file I/O; wait before publishing a sensor frame.

    No physics/renderer calls occur on these threads. This avoids serial NAS
    round-trip latency for each of the four already-owned sensor arrays.
    """
    directory=Path(directory)
    with ThreadPoolExecutor(max_workers=min(4, len(arrays)+(labels is not None))) as pool:
        jobs=[pool.submit(save_array,directory/name,array) for name,array in arrays.items()]
        if labels is not None:
            jobs.append(pool.submit(save_shared_labels,directory/'renderer_labels.json',labels,label_pool,verified))
        for job in jobs:
            job.result()


def compact_array(path):
    """Migrate only known sensor arrays; original manifests remain unchanged."""
    path = Path(path)
    if path.name not in ARRAY_NAMES or path.is_symlink():
        raise ValueError('Not a regular supported sensor array: ' + str(path))
    raw = path.read_bytes()
    # Reject corrupt/object data before migration, including malformed NPY files.
    np.load(io.BytesIO(raw), allow_pickle=False)
    compressed = _save_raw_array(path, raw)
    if path.read_bytes() != raw:
        raise ValueError('Source changed during compression: ' + str(path))
    path.unlink()
    return dict(path=str(path), original_bytes=len(raw), stored_bytes=compressed.stat().st_size,
                original_sha256=hashlib.sha256(raw).hexdigest())


def restore_array(path):
    path = Path(path)
    if path.name.endswith('.gz'):
        path = path.with_suffix('')
    raw = original_bytes(path)
    record = json.loads((path.parent / 'storage.json').read_text())['arrays'][path.name]
    if hashlib.sha256(raw).hexdigest() != record['original_sha256']:
        raise ValueError('Original array SHA mismatch')
    if path.exists() and path.read_bytes() != raw:
        raise ValueError('Refusing to overwrite a different NPY')
    atomic_bytes(path, raw)
    return path


def save_shared_labels(path, labels, pool, verified=None):
    """Stable per-frame JSON path, shared exact bytes within one RGB-D dataset."""
    path, pool = Path(path), Path(pool)
    raw = (json.dumps(labels, indent=2) + '\n').encode()
    digest = hashlib.sha256(raw).hexdigest()
    target = pool / (digest + '.json')
    if target.exists():
        if (verified is None or digest not in verified) and target.read_bytes() != raw:
            raise ValueError('Shared label SHA collision/corruption')
    else:
        atomic_bytes(target, raw)
    if verified is not None:
        verified.add(digest)
    relative = os.path.relpath(target, path.parent)
    try:
        # The normal recorder creates an entirely new frame. Avoid multiple
        # NAS metadata round trips needed only by migration of an old file.
        os.symlink(relative, path)
        return
    except FileExistsError:
        if path.read_bytes() != raw:
            raise ValueError('Refusing to replace different renderer labels')
        if path.is_symlink() and os.readlink(path) == relative:
            return
    fd, temp = tempfile.mkstemp(prefix='.label-link-', dir=path.parent)
    os.close(fd)
    os.unlink(temp)
    try:
        os.symlink(relative, temp)
        os.replace(temp, path)
    finally:
        if os.path.lexists(temp):
            os.unlink(temp)
    if path.read_bytes() != raw:
        raise ValueError('Shared labels verification failed')


def share_file(path, pool):
    """Hardlink only immutable exported bytes; never hardlink a source capture.

    URLs stay inside each export; duplicate scenes/images share disk blocks.
    Pool and destination must be on one filesystem. Atomic replacement avoids
    exposing a partial file to an active web reader.
    """
    path, pool = Path(path), Path(pool)
    with path.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    pool.mkdir(parents=True, exist_ok=True)
    target = pool / (digest + ''.join(path.suffixes))
    if not target.exists():
        os.link(path, target)
    else:
        with target.open('rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != digest:
                raise ValueError('Shared asset corruption: ' + str(target))
        if not os.path.samefile(path, target):
            fd, temp = tempfile.mkstemp(prefix='.shared-link-', dir=path.parent)
            os.close(fd)
            os.unlink(temp)
            try:
                os.link(target, temp)
                os.replace(temp, path)
            finally:
                if os.path.exists(temp):
                    os.unlink(temp)
    return digest
