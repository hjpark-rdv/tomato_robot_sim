"""Verified local copies of authoritative MJB bytes; no physics or model edits.

Every acquisition verifies the original source. Existing cache entries are
independently verified and never overwritten, so a read-only valid cache works.
New entries are published atomically without replacing any existing pathname.
Only temporary files created by this call are removed.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re
import stat
import time
import uuid


_CHUNK = 1024 * 1024


def _open_cache_directory(path):
    """Open/create every directory component without following symlinks."""
    path = Path(os.path.abspath(os.fspath(path)))
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
    fd = os.open(path.anchor, flags)
    try:
        for component in path.parts[1:]:
            try:
                child = os.open(component, flags, dir_fd=fd)
            except FileNotFoundError:
                try:
                    os.mkdir(component, mode=0o700, dir_fd=fd)
                except FileExistsError:
                    pass
                child = os.open(component, flags, dir_fd=fd)
            os.close(fd)
            fd = child
        return path, fd
    except BaseException:
        os.close(fd)
        raise


def _open_regular(path, *, dir_fd=None):
    # NONBLOCK prevents a malicious/non-file cache entry such as a FIFO from
    # blocking before fstat can reject it. It has no effect on regular files.
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK,
                 dir_fd=dir_fd)
    if not stat.S_ISREG(os.fstat(fd).st_mode):
        os.close(fd)
        raise ValueError("Model source/cache entry must be a regular file: " + str(path))
    return fd


def _hash_stream(stream, destination=None):
    before = os.fstat(stream.fileno())
    digest = hashlib.sha256()
    count = 0
    while block := stream.read(_CHUNK):
        digest.update(block)
        count += len(block)
        if destination is not None:
            destination.write(block)
    after = os.fstat(stream.fileno())
    identity = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)
    if identity(before) != identity(after) or count != before.st_size:
        raise ValueError("Model source/cache changed while being read")
    return digest.hexdigest(), count


def _verify_cache(directory_fd, name, expected):
    with os.fdopen(_open_regular(name, dir_fd=directory_fd), "rb") as stream:
        digest, count = _hash_stream(stream)
    if digest != expected:
        raise ValueError("Local model cache SHA mismatch; existing entry was not modified")
    return digest, count


def verify_or_stage_model(source, expected_sha, cache_dir):
    """Return ``(local_path, evidence)`` after source and cache SHA verification.

    Cache filenames are content-addressed SHA256 values. Cache directory
    components and the entry itself must not be symlinks. A source symlink is
    resolved once to its actual regular file, whose bytes must match the SHA.
    Existing invalid entries fail without replacement.
    Atomic hard-link publication avoids clobbering concurrent cache writers;
    a concurrent winner is independently verified before it can be used.
    No MuJoCo import, model conversion, controller or GPU work occurs here.
    """
    if not isinstance(expected_sha, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected_sha):
        raise ValueError("Expected model SHA256 must contain exactly 64 hexadecimal characters")
    expected = expected_sha.lower()
    requested_source = Path(os.path.abspath(os.fspath(source)))
    source = requested_source.resolve(strict=True)
    started = time.perf_counter()
    directory, directory_fd = _open_cache_directory(cache_dir)
    name = expected + ".mjb"
    temporary = None
    published = False
    raced = False
    try:
        try:
            entry = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        except FileNotFoundError:
            entry = None
        if entry is not None and not stat.S_ISREG(entry.st_mode):
            raise ValueError("Local model cache entry must be a regular non-symlink file")
        source_started = time.perf_counter()
        with os.fdopen(_open_regular(source), "rb") as source_stream:
            if entry is None:
                candidate = ".model-" + uuid.uuid4().hex + ".tmp"
                fd = os.open(candidate, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                             os.O_NOFOLLOW | os.O_CLOEXEC, 0o600, dir_fd=directory_fd)
                # Claim ownership only after exclusive creation succeeds. A
                # pre-existing collision must never be removed during cleanup.
                temporary = candidate
                with os.fdopen(fd, "wb") as destination:
                    source_digest, source_bytes = _hash_stream(source_stream, destination)
                    if source_digest != expected:
                        raise ValueError("Source model SHA mismatch")
                    destination.flush()
                    os.fsync(destination.fileno())
                    os.fchmod(destination.fileno(), 0o444)
            else:
                source_digest, source_bytes = _hash_stream(source_stream)
                if source_digest != expected:
                    raise ValueError("Source model SHA mismatch")
        source_s = time.perf_counter() - source_started
        if temporary is not None:
            try:
                os.link(temporary, name, src_dir_fd=directory_fd,
                        dst_dir_fd=directory_fd, follow_symlinks=False)
                published = True
            except FileExistsError:
                raced = True
        cache_started = time.perf_counter()
        cache_digest, cache_bytes = _verify_cache(directory_fd, name, expected)
        cache_s = time.perf_counter() - cache_started
        local = directory / name
        return local, dict(schema="s300-local-model-cache-v1",
            source_path=str(requested_source), cache_path=str(local),
            origin_requested=str(requested_source), origin_resolved=str(source),
            source_sha256=source_digest, cache_sha256=cache_digest,
            expected_sha256=expected, source_bytes=source_bytes, cache_bytes=cache_bytes,
            source_verified=True, cache_verified=True, cache_hit=not published,
            cache_populated=published, concurrent_cache_winner=raced,
            source_hash_and_optional_copy_wall_s=source_s, cache_hash_wall_s=cache_s,
            total_wall_s=time.perf_counter() - started,
            publication="atomic no-clobber hard link of verified completed temporary file",
            existing_files_overwritten=False, model_bytes_changed=False)
    finally:
        try:
            if temporary is not None:
                try:
                    os.unlink(temporary, dir_fd=directory_fd)
                except FileNotFoundError:
                    pass
        finally:
            os.close(directory_fd)
