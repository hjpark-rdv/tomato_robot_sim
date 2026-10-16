"""Present measured Newton poses through the existing isolated Isaac viewer.

The producer imports only NumPy and the existing latest-frame transport. It
never imports Kit, starts a timeline, edits USD, steps physics, or changes a
Newton state. ``due`` lets a CUDA caller avoid downloading display-only poses
on every physics step. GUI sampling may skip frames; the physics trace must be
recorded independently at its existing fixed step frequency.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import hashlib
import json
import math
from numbers import Integral
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np

from motion_live import MotionLiveReader, MotionLiveWriter


def _write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def process_is_descendant(pid, ancestor_pid):
    """Verify a shell launcher readiness PID using Linux process ancestry."""
    current = int(pid)
    ancestor = int(ancestor_pid)
    seen = set()
    while current > 0 and current not in seen:
        if current == ancestor:
            return True
        seen.add(current)
        try:
            status = (Path('/proc') / str(current) / 'status').read_text()
            parent = next(line.split(':', 1)[1].strip()
                          for line in status.splitlines() if line.startswith('PPid:'))
            current = int(parent)
        except (FileNotFoundError, ProcessLookupError, PermissionError, StopIteration):
            return False
    return False


def process_is_running(pid):
    """A child can outlive its shell while finishing a PNG/report; zombies exited."""
    try:
        status = (Path('/proc') / str(int(pid)) / 'status').read_text()
        state = next(line.split(':', 1)[1].strip().split()[0]
                     for line in status.splitlines() if line.startswith('State:'))
        return state not in ('Z', 'X')
    except (FileNotFoundError, ProcessLookupError, PermissionError, StopIteration):
        return False


def camera_test_position(base_position, right, up, elapsed_wall_s):
    """Small display-camera motion, independent of simulation state/time."""
    base, right, up = (np.asarray(v, dtype=float) for v in (base_position, right, up))
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in (base, right, up)):
        raise ValueError('Camera test needs finite xyz vectors')
    if (not math.isfinite(elapsed_wall_s) or elapsed_wall_s < 0
            or np.linalg.norm(right) < 1e-12 or np.linalg.norm(up) < 1e-12):
        raise ValueError('Camera test elapsed time/basis is invalid')
    phase = 2. * math.pi * elapsed_wall_s / 6.
    return (base + .15 * math.sin(phase) * right / np.linalg.norm(right)
            + .08 * math.sin(.7 * phase) * up / np.linalg.norm(up))


class CameraMotionEvidence:
    """Record actual post-render camera positions; never touch physics data."""
    def __init__(self, camera_path, base_position):
        self.camera_path = str(camera_path)
        self.base = np.asarray(base_position, dtype=float).copy()
        if not self.camera_path.startswith('/') or self.base.shape != (3,) or not np.isfinite(self.base).all():
            raise ValueError('Camera evidence needs an absolute USD path and finite position')
        self.samples = []
        self.movement_count = 0
        self.maximum_displacement_m = 0.

    def record(self, actual_position, *, render_update, elapsed_wall_s,
               measured_time_s, timeline_time_s, physics_steps=0):
        if physics_steps != 0 or timeline_time_s != 0.:
            raise RuntimeError('Camera-motion evidence requires zero display physics steps and stopped timeline')
        position = np.asarray(actual_position, dtype=float)
        if (position.shape != (3,) or not np.isfinite(position).all()
                or not math.isfinite(elapsed_wall_s) or elapsed_wall_s < 0
                or not math.isfinite(measured_time_s) or measured_time_s < 0
                or render_update <= 0
                or (self.samples and render_update <= self.samples[-1]['render_update'])):
            raise ValueError('Camera observation must follow a new completed render update')
        previous = np.asarray(self.samples[-1]['world_position_m']) if self.samples else self.base
        if np.linalg.norm(position - previous) > 1e-9:
            self.movement_count += 1
        self.maximum_displacement_m = max(self.maximum_displacement_m,
                                         float(np.linalg.norm(position - self.base)))
        self.samples.append(dict(render_update=int(render_update), elapsed_wall_s=float(elapsed_wall_s),
            measured_time_s=float(measured_time_s), world_position_m=position.tolist(),
            physics_steps=0, timeline_time_s=0.))

    def summary(self):
        return dict(enabled=True, camera_path=self.camera_path,
            source='Actual USD camera world transform after completed render update',
            edit_layer='isolated display USD session layer',
            sample_count=len(self.samples), movement_count=self.movement_count,
            initial_world_position_m=self.base.tolist(),
            last_world_position_m=None if not self.samples else self.samples[-1]['world_position_m'],
            maximum_displacement_m=self.maximum_displacement_m,
            physics_steps=0, timeline_time_s=0.)


def write_display_ack(output, snapshot, *, displayed_count, render_updates,
                      timeline_time_s, physics_steps=0, pid=None, camera_motion=None):
    """Acknowledge a new measured pose only after its app/render update returned.

    Called by the isolated viewer, never the producer. This records the
    presented measured state, not a physics step or an inferred future frame.
    """
    if physics_steps != 0 or timeline_time_s != 0.:
        raise RuntimeError('Measured display acknowledgement requires zero physics steps and stopped timeline')
    sequence = int(snapshot['sequence'])
    stamp = float(snapshot['time'])
    if sequence <= 0 or sequence % 2 or not math.isfinite(stamp):
        raise ValueError('Invalid displayed sequence/timestamp')
    if displayed_count <= 0 or render_updates <= 0:
        raise ValueError('Display acknowledgement requires a completed render update')
    poses = np.asarray(snapshot['poses'])
    if poses.ndim != 2 or poses.shape[1] != 7 or not np.isfinite(poses).all():
        raise ValueError('Invalid displayed measured poses')
    process_id = os.getpid() if pid is None else int(pid)
    acknowledgement = dict(schema='measured-live-display-ack-v1', pid=process_id,
        sequence=sequence, last_sim_time_s=stamp, displayed_count=int(displayed_count),
        render_updates=int(render_updates), status=snapshot['status'],
        physics_steps=0, timeline_time_s=0., render_update_completed=True,
        pose_sha256=hashlib.sha256(poses.astype('<f8').tobytes()).hexdigest(),
        acknowledged_monotonic_s=time.monotonic())
    if camera_motion is not None:
        if (camera_motion.get('physics_steps') != 0 or camera_motion.get('timeline_time_s') != 0.
                or camera_motion.get('enabled') is not True):
            raise RuntimeError('Camera-motion acknowledgement lacks stopped-physics proof')
        acknowledgement['camera_test_motion'] = dict(camera_motion)
    output = Path(output)
    # A distinct temporary path prevents another review viewer from corrupting
    # this tiny file. Consumers also verify the actual launched viewer PID.
    temporary = output / f'live_display_ack.{process_id}.json.tmp'
    temporary.write_text(json.dumps(acknowledgement, indent=2, allow_nan=False) + '\n')
    temporary.replace(output / 'live_display_ack.json')
    return acknowledgement


def required_skin_body_paths(assembly):
    """Return every measured body required by the unchanged rod-skin viewer."""
    assembly = Path(assembly)
    manifest = json.loads((assembly / 'motion_manifest.json').read_text())
    sources = [(manifest['plant'], 'skin.npz')]
    sources.extend((row['plant'], row['skin_file'])
                   for row in manifest.get('elastic_neighbors', []))
    paths = set()
    for plant, filename in sources:
        with np.load(assembly / filename) as archive:
            for index, _ in enumerate(plant['skin_paths']):
                paths.update(str(path) for path in archive[f'{index}_body_paths'])
    return sorted(paths)


def display_publication_metrics(observed, producer_metadata):
    """Count GUI losses independently from fixed-step physics or publication caps.

    MotionLiveWriter uses one sequence increment per measured publication and
    an additional increment when closing. That final status change must not be
    counted as an extra physics/display frame. A still-running producer has an
    unknown future publication count, so its loss count is a lower bound.
    """
    indices = set()
    for frame in observed:
        sequence = int(frame['sequence'])
        if sequence <= 0 or sequence % 2:
            raise ValueError('Displayed stream sequence must be positive and even')
        index = sequence // 2 - (frame['status'] in ('completed', 'stopped'))
        if index > 0:
            indices.add(index)
    completed = producer_metadata.get('status') in ('completed', 'stopped')
    total = (int(producer_metadata['publications']) if completed and
             'publications' in producer_metadata else max(indices, default=0))
    if indices and max(indices) > total:
        raise ValueError('Displayed sequence exceeds producer publication count')
    return dict(displayed_unique_measured_publications=len(indices),
                published_measured_frames=total,
                undisplayed_published_frames=total - len(indices),
                undisplayed_count_complete=bool(completed),
                first_displayed_publication=min(indices, default=None),
                last_displayed_publication=max(indices, default=None),
                scope='latest-frame GUI sampling only; skipped publications are not skipped physics steps',
                final_status_update_counted_as_frame=False)


class NewtonPoseMapping:
    """Map native body frames to original USD bodies without writing input state.

    ``usd_to_native`` maps absolute USD body paths to native body names or array
    indices. Newton transforms conventionally store xyz + xyzw; wxyz input is
    accepted explicitly. ``native_from_usd`` optionally contains rigid 4x4
    transforms: world_from_usd = world_from_native @ native_from_usd. Use these
    only when the portable model explicitly changed a body's coordinate frame.
    """
    def __init__(self, native_body_names: Sequence[str], usd_to_native: Mapping,
                 *, quaternion_order='xyzw', native_from_usd=None,
                 required_source_paths=()):
        self.native_names = [str(name) for name in native_body_names]
        if not self.native_names or len(set(self.native_names)) != len(self.native_names):
            raise ValueError('Native body names must be nonempty and unique')
        if quaternion_order not in ('xyzw', 'wxyz'):
            raise ValueError('Quaternion order must be xyzw or wxyz')
        self.quaternion_order = quaternion_order
        if not usd_to_native:
            raise ValueError('An explicit USD-to-Newton body mapping is required')
        self.body_paths = [str(path) for path in usd_to_native]
        if len(set(self.body_paths)) != len(self.body_paths) or any(
                not path.startswith('/') for path in self.body_paths):
            raise ValueError('USD body paths must be unique absolute paths')
        indices = []
        for path, value in usd_to_native.items():
            if isinstance(value, Integral) and not isinstance(value, bool):
                index = int(value)
                if not 0 <= index < len(self.native_names):
                    raise ValueError('Native body index out of range for ' + str(path))
            elif isinstance(value, str) and value in self.native_names:
                index = self.native_names.index(value)
            else:
                raise ValueError('Unknown native body for ' + str(path) + ': ' + str(value))
            indices.append(index)
        self.indices = np.asarray(indices, dtype=np.int64)
        missing = sorted(set(required_source_paths) - set(self.body_paths))
        if missing:
            raise ValueError('Live mapping misses required skin bodies: ' + ', '.join(missing[:12]))
        self.offsets = {}
        for path, transform in (native_from_usd or {}).items():
            if path not in self.body_paths:
                raise ValueError('Frame offset has no mapped source body: ' + str(path))
            matrix = np.asarray(transform, dtype=float)
            if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                    or not np.allclose(matrix[3], [0., 0., 0., 1.], atol=1e-12, rtol=0.)
                    or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-9, rtol=0.)
                    or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-9, rtol=0.)):
                raise ValueError('Frame offset must be a finite rigid 4x4 transform')
            # Import SciPy only for the optional coordinate-frame correction.
            from scipy.spatial.transform import Rotation
            self.offsets[self.body_paths.index(path)] = (
                matrix[:3, 3].copy(), Rotation.from_matrix(matrix[:3, :3]).as_quat())

    def convert(self, body_q):
        poses = np.asarray(body_q)
        if poses.shape != (len(self.native_names), 7):
            raise ValueError('Newton body_q must have shape (native_body_count, 7)')
        selected = np.array(poses[self.indices], dtype=float, copy=True)
        if self.quaternion_order == 'wxyz':
            selected[:, 3:7] = selected[:, [4, 5, 6, 3]]
        if not np.isfinite(selected).all():
            raise ValueError('Nonfinite measured Newton display pose')
        lengths = np.linalg.norm(selected[:, 3:7], axis=1)
        if np.any(lengths < 1e-12):
            raise ValueError('Zero measured Newton orientation quaternion')
        if self.offsets:
            from scipy.spatial.transform import Rotation
            for index, (translation, quaternion) in self.offsets.items():
                rotation = Rotation.from_quat(selected[index, 3:7])
                selected[index, :3] += rotation.apply(translation)
                selected[index, 3:7] = (rotation * Rotation.from_quat(quaternion)).as_quat()
        return selected

    def report(self):
        return dict(quaternion_order=self.quaternion_order,
                    native_body_count=len(self.native_names),
                    displayed_body_count=len(self.body_paths),
                    usd_to_native={path: dict(index=int(index), name=self.native_names[index])
                                   for path, index in zip(self.body_paths, self.indices)},
                    bodies_with_frame_offsets=[self.body_paths[i] for i in self.offsets])


class NewtonLivePublisher:
    """Opt-in display side effect of an otherwise unchanged fixed-step runner."""
    def __init__(self, output, assembly, native_body_names, usd_to_native, *,
                 fps=15., quaternion_order='xyzw', native_from_usd=None,
                 verify_skin_mapping=True, physical_model_sha256=None,
                 clock=time.monotonic):
        if not math.isfinite(fps) or not 0 < fps <= 60:
            raise ValueError('Live display publication FPS must be in (0, 60]')
        self.output = Path(output).resolve()
        self.assembly = Path(assembly).resolve()
        self.report_path = self.output / 'newton_live.json'
        if self.report_path.exists():
            raise FileExistsError(self.report_path)
        required = required_skin_body_paths(self.assembly) if verify_skin_mapping else ()
        self.mapping = NewtonPoseMapping(native_body_names, usd_to_native,
            quaternion_order=quaternion_order, native_from_usd=native_from_usd,
            required_source_paths=required)
        self.writer = MotionLiveWriter(self.output, self.assembly, self.mapping.body_paths)
        self.writer.metadata.update(
            source='latest completed measured Newton physics frame',
            physics_engine='newton', physical_model_sha256=physical_model_sha256,
            display_source_model_sha256=self.writer.metadata['model_sha256'],
            model_hash_scope='model_sha256 identifies display USD; physical_model_sha256 identifies Newton model',
            physics_stepping_owner='independent Newton producer',
            display_backpressure=False)
        _write_json(self.output / 'live.json', self.writer.metadata)
        self.clock = clock
        self.period = 1. / fps
        self.last_publish_wall = None
        self.last_seen_time = None
        self.offered = self.skipped = 0
        self.closed = False
        self.last_pose_sha256 = None
        self.report_data = dict(
            schema='s300-newton-live-v1', physics_engine='newton',
            physical_model_sha256=physical_model_sha256,
            display_source_model_sha256=self.writer.metadata['model_sha256'],
            mapping=self.mapping.report(), publication_fps_cap=fps,
            source='measured Newton body_q, no replay or second simulation',
            producer_imports_kit=False, viewer_owns_physics=False,
            physics_trace_scope='independent fixed-step trace; display stream has no history',
            training_eligible=False, harvest_success=None)
        self._save()

    def due(self):
        """Check before GPU readback. Does not mutate timing or simulation state."""
        if self.closed:
            return False
        return self.last_publish_wall is None or self.clock() - self.last_publish_wall >= self.period

    def publish(self, body_q, gap, sim_time_s, *, force=False):
        if self.closed:
            raise RuntimeError('Newton live publisher is closed')
        stamp = float(sim_time_s)
        if not math.isfinite(stamp) or (self.last_seen_time is not None and stamp < self.last_seen_time):
            raise ValueError('Measured Newton display timestamp is nonfinite or moved backwards')
        self.last_seen_time = stamp
        self.offered += 1
        if not force and not self.due():
            self.skipped += 1
            return False
        poses = self.mapping.convert(body_q)
        self.writer.publish(poses, gap, stamp)
        self.last_pose_sha256 = hashlib.sha256(poses.astype('<f8').tobytes()).hexdigest()
        self.last_publish_wall = self.clock()
        return True

    def _save(self):
        self.report_data.update(
            offered_display_updates=self.offered,
            published_measured_frames=self.writer.publications,
            skipped_by_publication_throttle=self.skipped,
            last_published_time_s=self.writer.last_time,
            last_pose_sha256=self.last_pose_sha256, closed=self.closed)
        _write_json(self.report_path, self.report_data)

    def close(self, reason=None):
        if self.closed:
            return
        self.writer.close(reason)
        self.closed = True
        self.report_data['reason'] = reason
        self._save()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        self.close(None if exception is None else type(exception).__name__ + ': ' + str(exception))


class NewtonViewerProcess:
    """Launch/stop the existing measured-state viewer in its own process group.

    Readiness waiting is explicitly bounded and should happen before starting
    measured physics timing. Viewer exit never asks the physics runner to step
    or reset. A caller can report the error and continue headless.
    """
    def __init__(self, trial, *, isaac_python='/root/isaac-sim-6.1.0/python.sh',
                 display=':0', fps=15., seconds=0., camera='entry', capture=None,
                 viewer_script=None, extra_env=None, review_light=True, camera_test_motion=False):
        if not math.isfinite(fps) or not 0 < fps <= 60 or not math.isfinite(seconds) or seconds < 0:
            raise ValueError('Viewer FPS/seconds are invalid')
        if not isinstance(camera_test_motion, bool):
            raise ValueError('camera_test_motion must be bool')
        self.trial = Path(trial).resolve()
        if not (self.trial / 'live.json').is_file():
            raise FileNotFoundError('Publish an initial measured frame before launching its viewer')
        reader = MotionLiveReader(self.trial)
        if reader.read() is None:
            raise ValueError('Publish an initial measured frame before launching its viewer')
        self.display_source_sha256 = reader.metadata['model_sha256']
        self.ready = self.trial / 'newton_viewer_ready.json'
        self.log = self.trial / 'newton_viewer.log'
        self.report_path = self.trial / 'newton_viewer_process.json'
        for path in (self.ready, self.log, self.report_path):
            if path.exists():
                raise FileExistsError(path)
        python = Path(isaac_python).resolve()
        script = Path(viewer_script) if viewer_script else Path(__file__).with_name('view_motion_live.py')
        if not python.is_file() or not script.is_file():
            raise FileNotFoundError('Isaac Python or existing measured-state viewer is missing')
        command = [str(python)]
        if python.name.endswith('.sh'):
            command.append('--no-ros-env')
        command.extend(['-u', str(script), str(self.trial), '--fps', str(fps),
                        '--seconds', str(seconds), '--camera', str(camera),
                        '--ready-file', str(self.ready)])
        if capture:
            command.extend(['--capture', str(capture)])
        if review_light:
            command.append('--review-light')
        if camera_test_motion:
            command.append('--camera-test-motion')
        environment = os.environ.copy()
        environment.update(extra_env or {})
        environment.update(DISPLAY=str(display), OMNI_KIT_ALLOW_ROOT='1')
        self.started = time.monotonic()
        self.closed = False
        self.process = None
        self.report = dict(schema='s300-newton-viewer-process-v1', command=command,
            display=str(display), viewer_owns_physics=False, producer_owns_physics=True,
            readiness_timeout_is_bounded=True, log=str(self.log), ready=False,
            display_review_fill_light=bool(review_light), camera_test_motion_requested=camera_test_motion)
        with self.log.open('x') as stream:
            self.process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True)
        self.report['pid'] = self.process.pid
        _write_json(self.report_path, self.report)

    def wait_ready(self, timeout=300.):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Viewer readiness timeout must be finite and positive')
        deadline = time.monotonic() + timeout
        while True:
            if self.ready.is_file():
                try:
                    readiness = json.loads(self.ready.read_text())
                except json.JSONDecodeError:
                    readiness = None  # Existing viewer readiness is a tiny direct write.
                if readiness is None:
                    if time.monotonic() >= deadline:
                        self.close()
                        raise TimeoutError('Newton viewer wrote an incomplete readiness file: ' + str(self.ready))
                    time.sleep(.01)
                    continue
                if readiness.get('physics_steps') != 0:
                    self.report['readiness_error'] = 'viewer_reported_physics_stepping'
                    self.close()
                    raise RuntimeError('Measured-state viewer unexpectedly reported physics stepping')
                if readiness.get('model_sha256') != self.display_source_sha256:
                    self.report['readiness_error'] = 'viewer_scene_hash_mismatch'
                    self.close()
                    raise RuntimeError('Measured-state viewer display scene hash mismatch')
                viewer_pid = int(readiness.get('pid', self.process.pid))
                if not process_is_descendant(viewer_pid, self.process.pid):
                    self.report['readiness_error'] = 'viewer_pid_not_descendant_of_launcher'
                    self.close()
                    raise RuntimeError('Measured-state viewer readiness PID is not a child of its launcher')
                self.report.update(ready=True, readiness=readiness, viewer_python_pid=viewer_pid,
                                   readiness_wall_s=time.monotonic() - self.started)
                _write_json(self.report_path, self.report)
                return readiness
            code = self.process.poll()
            if code is not None:
                self.report.update(readiness_error='viewer_exited_before_ready', returncode=code)
                _write_json(self.report_path, self.report)
                raise RuntimeError('Newton viewer exited before readiness; inspect ' + str(self.log))
            if time.monotonic() >= deadline:
                self.report['readiness_error'] = 'viewer_readiness_timeout'
                _write_json(self.report_path, self.report)
                self.close()
                raise TimeoutError('Newton viewer readiness timed out; physics can continue headless; log: ' + str(self.log))
            time.sleep(.05)

    def is_alive(self):
        viewer_pid = self.report.get('viewer_python_pid')
        return (process_is_running(viewer_pid) if viewer_pid is not None
                else self.process.poll() is None)

    def wait_displayed(self, sim_time_s, timeout=10., *, minimum_sequence=None):
        """Wait for a completed viewer render of the requested measured time.

        This presentation-only wait belongs outside measured physics timing.
        A viewer deadline/early exit is an explicit display failure; it does
        not invalidate or truncate the independent saved physics trace.
        """
        stamp = float(sim_time_s)
        if not math.isfinite(stamp) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Display timestamp/timeout must be finite and timeout positive')
        if minimum_sequence is not None and (minimum_sequence <= 0 or minimum_sequence % 2):
            raise ValueError('Minimum display sequence must be positive and even')
        deadline = time.monotonic() + timeout
        path = self.trial / 'live_display_ack.json'
        last_ack = None
        while True:
            if path.is_file():
                try:
                    acknowledgement = json.loads(path.read_text())
                except json.JSONDecodeError:
                    acknowledgement = None
                viewer_pid = self.report.get('viewer_python_pid', self.process.pid)
                if acknowledgement is not None and acknowledgement.get('pid') == viewer_pid:
                    if (acknowledgement.get('physics_steps') != 0
                            or acknowledgement.get('timeline_time_s') != 0.
                            or acknowledgement.get('render_update_completed') is not True):
                        self.report['display_error'] = 'viewer_acknowledged_unverified_render'
                        _write_json(self.report_path, self.report)
                        raise RuntimeError('Newton viewer acknowledgement lacks stopped-timeline render proof')
                    last_ack = acknowledgement
                    if (acknowledgement['last_sim_time_s'] >= stamp
                            and (minimum_sequence is None or acknowledgement['sequence'] >= minimum_sequence)):
                        motion_ready = True
                        if self.report.get('camera_test_motion_requested'):
                            motion = acknowledgement.get('camera_test_motion') or {}
                            if (motion.get('enabled') is not True
                                    or motion.get('physics_steps') != 0
                                    or motion.get('timeline_time_s') != 0.):
                                self.report['display_error'] = 'viewer_camera_motion_proof_missing'
                                _write_json(self.report_path, self.report)
                                raise RuntimeError('Requested camera test lacks stopped-physics camera evidence')
                            # A camera setup flag or one sampled position alone
                            # is not proof that it actually moved after rendering.
                            motion_ready = (motion.get('sample_count', 0) >= 2
                                and motion.get('movement_count', 0) >= 1
                                and motion.get('maximum_displacement_m', 0.) > 0.)
                        if motion_ready:
                            self.report.update(final_display_ack=acknowledgement,
                                               requested_display_time_s=stamp)
                            _write_json(self.report_path, self.report)
                            return acknowledgement
            code = self.process.poll()
            if code is not None:
                self.report.update(display_error='viewer_exited_before_requested_frame',
                    returncode=code, requested_display_time_s=stamp, last_display_ack=last_ack)
                _write_json(self.report_path, self.report)
                raise RuntimeError('Newton viewer exited before requested measured frame; inspect ' + str(self.log))
            if time.monotonic() >= deadline:
                self.report.update(display_error='requested_frame_timeout',
                    requested_display_time_s=stamp, last_display_ack=last_ack)
                _write_json(self.report_path, self.report)
                raise TimeoutError('Newton viewer did not display requested measured frame before timeout; log: ' + str(self.log))
            time.sleep(.02)

    def close(self, grace_seconds=10.):
        if self.closed:
            return self.report
        if not math.isfinite(grace_seconds) or grace_seconds < 0:
            raise ValueError('Viewer cleanup grace must be finite and nonnegative')
        if self.process.poll() is None or self.is_alive():
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + (grace_seconds if sig != signal.SIGKILL else 5.)
                while self.process.poll() is None or self.is_alive():
                    if time.monotonic() >= deadline:
                        break
                    time.sleep(.02)
                if self.process.poll() is not None and not self.is_alive():
                    break
        self.closed = True
        self.report.update(returncode=self.process.poll(), closed=True,
                           verified_viewer_exited=not self.is_alive(),
                           elapsed_wall_s=time.monotonic() - self.started,
                           viewer_reports=[str(path) for path in sorted(self.trial.glob('live_viewer_*.json'))])
        _write_json(self.report_path, self.report)
        return self.report

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        self.close()
