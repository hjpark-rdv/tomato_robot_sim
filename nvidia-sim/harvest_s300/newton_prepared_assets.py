"""Prepare unchanged Newton assets with bounded hashing and directory sharing.

These helpers do not import a physics engine or change asset bytes. Callers
must verify the manifest before sharing assets and exclude mutable files from
the link list. A shared top-level directory must contain immutable assets:
linking it also exposes siblings that are not named in the manifest.
"""
from __future__ import annotations

from collections.abc import Mapping
from concurrent.futures import ThreadPoolExecutor
import hashlib
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import stat


_MAX_WORKERS = 8
_CHUNK = 1024 * 1024


def _relative_name(name):
    """Require one canonical relative POSIX manifest pathname."""
    if (not isinstance(name, str) or not name or "\x00" in name or "\\" in name
            or PurePosixPath(name).is_absolute() or PureWindowsPath(name).drive
            or any(part in ("", ".", "..") for part in name.split("/"))):
        raise ValueError(f"Manifest path must be a canonical relative path: {name!r}")
    return PurePosixPath(name)


def _source_directory(root):
    root = Path(root).resolve(strict=True)
    if not root.is_dir():
        raise NotADirectoryError(root)
    return root


def _hash_file(path):
    # Manifest sources can be symlinks to shared verified assets. NONBLOCK
    # lets us reject a FIFO/device via fstat without waiting for a writer.
    fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NONBLOCK)
    with os.fdopen(fd, "rb") as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"Manifest source must be a regular file: {path}")
        digest = hashlib.sha256()
        size = 0
        while block := stream.read(_CHUNK):
            digest.update(block)
            size += len(block)
        after = os.fstat(stream.fileno())
    identity = lambda value: (value.st_dev, value.st_ino, value.st_size,
                              value.st_mtime_ns, value.st_ctime_ns)
    if identity(before) != identity(after) or size != before.st_size:
        raise ValueError(f"Manifest source changed while being hashed: {path}")
    return digest.hexdigest()


def verify_manifest_files(root, input_files, workers=8, *, verification_context=None):
    """Hash every ``{relative_name: {'sha256': expected}}`` entry; return digests.

    At most eight independent file reads run concurrently. Results preserve
    manifest order and exceptions propagate; no digest cache substitutes for
    reading the original bytes by default. An explicit caller-owned verification
    context may reuse original digests only after unchanged file-identity checks.
    Source file/directory symlinks are supported.
    """
    if type(workers) is not int or not 1 <= workers <= _MAX_WORKERS:
        raise ValueError("Manifest hash workers must be an integer between 1 and 8")
    if not isinstance(input_files, Mapping):
        raise TypeError("input_files must map relative names to SHA256 evidence")
    entries = []
    for name, evidence in input_files.items():
        relative = _relative_name(name)
        expected = evidence.get("sha256") if isinstance(evidence, Mapping) else None
        if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected):
            raise ValueError(f"Manifest file has no valid SHA256: {name}")
        entries.append((name, relative, expected.lower()))
    root = _source_directory(root)

    def verify(entry):
        name, relative, expected = entry
        actual = (_hash_file(root / relative) if verification_context is None else
                  verification_context.file_sha256(root / relative))
        if actual != expected:
            raise ValueError(f"Input SHA256 mismatch: {name}")
        return name, actual

    with ThreadPoolExecutor(max_workers=workers, thread_name_prefix="newton-asset-hash") as pool:
        return dict(pool.map(verify, entries))


def link_manifest_assets(source, dest, names):
    """Link immutable manifest assets without modifying existing destinations.

    Each root-level file receives its own resolved-target symlink. Nested
    files retain their relative paths through one symlink per top-level
    directory. Every named source must exist and be a regular file; those
    checks run in at most eight threads. SHA verification belongs to the
    caller's preceding ``verify_manifest_files`` call. Exclude mutable root
    files and never place mutable outputs inside a shared asset directory.

    All source/path/collision checks precede link creation. A concurrent
    destination creation still fails atomically without overwriting it; an
    I/O failure during creation may leave this call's earlier new links.
    """
    relatives = [_relative_name(name) for name in names]
    if len(set(relatives)) != len(relatives):
        raise ValueError("Duplicate manifest asset paths")
    source = _source_directory(source)
    dest = Path(os.path.abspath(os.fspath(dest)))
    if source == dest.resolve(strict=False):
        raise ValueError("Asset source and destination must differ")

    links = {}
    directories = set()
    for relative in relatives:
        top = relative.parts[0]
        if top in links:
            continue
        target = (source / top).resolve(strict=True)
        if len(relative.parts) > 1:
            if not target.is_dir():
                raise NotADirectoryError(target)
            directories.add(top)
            # Avoid creating a directory link back into an ancestor of itself.
            if dest.resolve(strict=False).is_relative_to(target):
                raise ValueError("Asset destination is inside a shared source directory")
        links[top] = target

    def check_source(relative):
        target = links[relative.parts[0]].joinpath(*relative.parts[1:])
        if not stat.S_ISREG(target.stat().st_mode):
            raise ValueError(f"Manifest source must be a regular file: {source / relative}")

    with ThreadPoolExecutor(max_workers=_MAX_WORKERS, thread_name_prefix="newton-asset-stat") as pool:
        # Consume every result so worker exceptions cannot be discarded.
        tuple(pool.map(check_source, relatives))

    if dest.is_symlink():
        raise ValueError("Asset destination directory must not be a symlink")
    dest.mkdir(parents=True, exist_ok=True)
    directory_fd = os.open(dest, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        for name in links:
            try:
                os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
            except FileNotFoundError:
                continue
            raise FileExistsError(f"Asset destination already exists: {dest / name}")
        for name, target in links.items():
            os.symlink(str(target), name, dir_fd=directory_fd,
                       target_is_directory=name in directories)
    finally:
        os.close(directory_fd)
    return dict(schema="s300-prepared-asset-links-v1", source_root=str(source),
                destination_root=str(dest), asset_count=len(relatives),
                link_count=len(links), directory_link_count=len(directories),
                root_file_link_count=len(links) - len(directories),
                links={name: str(target) for name, target in links.items()},
                existing_files_overwritten=False, asset_bytes_changed=False,
                sha256_verification="caller must verify the source manifest before linking")
