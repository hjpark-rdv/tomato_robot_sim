"""Small latest-frame transport; the viewer never controls or advances physics.

Only one completed, measured native frame is kept.  The producer owns the
writer; a separate display process reads it using an odd/even sequence check.
There is no compression, frame history, USD import, or per-frame filesystem sync.
"""
import hashlib
import json
import re
from pathlib import Path

import numpy as np

HEADER_FLOATS = 8
SCHEMA = 'measured-native-latest-frame-v1'
STATUS = {0: 'waiting', 1: 'running', 2: 'completed', 3: 'stopped'}
_ENV = re.compile(r'^/World/MotionEnvs/env_(\d+)(?=/|$)')


def file_sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path, value):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


class MotionLiveWriter:
    """Publish env0 plus shared bodies without changing saved physics histories.

    ``publish`` accepts the complete native body-pose array in xyzw quaternion
    order.  The caller controls publication pacing, normally 10 Hz wall time.
    """
    def __init__(self, output, assembly, body_paths):
        self.output = Path(output).resolve()
        self.assembly = Path(assembly).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        source = self.assembly / 'scene.usda'
        if not source.is_file():
            raise FileNotFoundError(source)
        original = [str(path) for path in body_paths]
        if not original or len(set(original)) != len(original):
            raise ValueError('Native body paths must be nonempty and unique')
        selected = []
        paths = []
        for index, path in enumerate(original):
            match = _ENV.match(path)
            if match and int(match.group(1)) != 0:
                continue
            selected.append(index)
            paths.append(_ENV.sub('/World', path, count=1))
        if not paths or len(set(paths)) != len(paths):
            raise ValueError('Env0/shared display body mapping must be nonempty and unique')
        binary = self.output / 'live.bin'
        metadata = self.output / 'live.json'
        if binary.exists() or metadata.exists():
            raise FileExistsError('Live stream already exists: ' + str(self.output))
        self.indices = np.asarray(selected, dtype=np.int64)
        self.native_count = len(original)
        self.sequence = 0
        self.closed = False
        self.publications = 0
        self.last_time = None
        self.metadata = dict(schema=SCHEMA, assembly=str(self.assembly),
            model_sha256=file_sha(source), native_body_count=len(original),
            body_paths=paths, native_body_paths=[original[i] for i in selected],
            display_environment=0, dtype='<f8', header_floats=HEADER_FLOATS,
            quaternion_order='xyzw', physics_rerun=False,
            source='latest completed measured native physics frame',
            history_saved_separately='states.npz', status='waiting')
        size = (HEADER_FLOATS + len(paths) * 7) * 8
        # Exclusive creation protects previous results, including unfinished runs.
        with binary.open('xb') as stream:
            stream.truncate(size)
        self.buffer = np.memmap(binary, dtype='<f8', mode='r+', shape=(size // 8,))
        self.buffer[:] = 0.
        self.poses = self.buffer[HEADER_FLOATS:].reshape(len(paths), 7)
        _write_json(metadata, self.metadata)

    def publish(self, poses, gap, sim_time_s):
        if self.closed:
            raise RuntimeError('Live writer is closed')
        poses = np.asarray(poses)
        if poses.shape != (self.native_count, 7):
            raise ValueError('Native pose array does not match body mapping')
        selected = poses[self.indices]
        gap = float(gap)
        stamp = float(sim_time_s)
        if not np.isfinite(selected).all() or not np.isfinite([gap, stamp]).all():
            raise ValueError('Nonfinite live measured frame')
        if np.any(np.linalg.norm(selected[:, 3:7], axis=1) < 1e-12):
            raise ValueError('Zero measured orientation quaternion')
        if self.last_time is not None and stamp < self.last_time:
            raise ValueError('Measured physics timestamp moved backwards')
        self.buffer[0] = self.sequence + 1  # odd: write in progress
        self.poses[:] = selected
        self.buffer[1:4] = (stamp, gap, 1)
        self.sequence += 2
        self.buffer[0] = self.sequence      # even: complete measured frame
        self.last_time = stamp
        self.publications += 1

    def close(self, reason=None):
        if self.closed:
            return
        completed = reason in (None, '', 'complete', 'completed')
        self.buffer[0] = self.sequence + 1
        self.buffer[3] = 2 if completed else 3
        self.sequence += 2
        self.buffer[0] = self.sequence
        self.buffer.flush()
        self.metadata.update(status=STATUS[int(self.buffer[3])], reason=reason,
                             publications=self.publications,
                             final_sim_time_s=self.last_time)
        _write_json(self.output / 'live.json', self.metadata)
        self.closed = True


class MotionLiveReader:
    def __init__(self, output):
        self.output = Path(output).resolve()
        self.metadata = json.loads((self.output / 'live.json').read_text())
        if self.metadata.get('schema') != SCHEMA:
            raise ValueError('Unsupported live stream schema')
        self.body_paths = self.metadata['body_paths']
        if not self.body_paths or len(set(self.body_paths)) != len(self.body_paths):
            raise ValueError('Invalid live body mapping')
        size = HEADER_FLOATS + len(self.body_paths) * 7
        binary = self.output / 'live.bin'
        if binary.stat().st_size != size * 8:
            raise ValueError('Live buffer length does not match body mapping')
        self.buffer = np.memmap(binary, dtype='<f8', mode='r', shape=(size,))

    def verify_source(self):
        scene = Path(self.metadata['assembly']) / 'scene.usda'
        if file_sha(scene) != self.metadata['model_sha256']:
            raise ValueError('Live viewer scene hash mismatch')
        return scene

    def read(self, attempts=3):
        for _ in range(attempts):
            before = float(self.buffer[0])
            if before == 0 or int(before) % 2:
                return None
            header = self.buffer[:HEADER_FLOATS].copy()
            poses = self.buffer[HEADER_FLOATS:].reshape(len(self.body_paths), 7).copy()
            after = float(self.buffer[0])
            if before == after and not int(after) % 2:
                # A producer closed before its first publish has no measured frame.
                if not np.any(poses[:, 3:7]):
                    return None
                return dict(sequence=int(after), time=float(header[1]),
                            gap=float(header[2]), status=STATUS.get(int(header[3]), 'unknown'),
                            poses=poses)
        return None
