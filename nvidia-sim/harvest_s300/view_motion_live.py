#!/usr/bin/env python3
"""Display the producer's latest measured frame in a separate Isaac process.

This process has no physics initialization/step loop, robot control, or replay of
commands. The full source visual geometry is retained. Camera movement remains
available while the producer is paused or has completed its run.
"""
import argparse
import asyncio
import json
import os
import signal
import time
import traceback
from pathlib import Path

from motion_live import MotionLiveReader
from newton_live import (CameraMotionEvidence, camera_test_position,
                        display_publication_metrics, write_display_ack)

p = argparse.ArgumentParser()
p.add_argument('trial', type=Path)
p.add_argument('--fps', type=float, default=10., help='wall-clock display cap only')
p.add_argument('--seconds', type=float, default=0., help='0 leaves the window open; otherwise bounded wall time')
p.add_argument('--camera', default='/World/HarvestCloseCamera')
p.add_argument('--camera-test-motion', action='store_true',
               help='Move only the display camera and record actual post-render transforms; physics stays stopped')
p.add_argument('--ready-file', type=Path, help='Publish display readiness after showing the first measured frame; an external launcher may then release the producer start gate')
p.add_argument('--headless', action='store_true', help='render/capture validation without a display window')
p.add_argument('--capture', type=Path, help='save a PNG of the final displayed measured frame')
p.add_argument('--review-light', action='store_true',
               help='Add the existing replay inspection fill light in this display session only')
p.add_argument('--wait-seconds', type=float, default=60., help='wait for the producer to create its live metadata')
a = p.parse_args()
if not 0 < a.fps <= 60 or a.seconds < 0 or a.wait_seconds < 0:
    p.error('fps must be in (0,60], seconds and wait-seconds must be nonnegative')
if a.headless and a.seconds == 0:
    p.error('--headless requires a bounded --seconds value')
if a.capture and (a.capture.exists() or a.capture.with_suffix('.json').exists()):
    raise FileExistsError(a.capture)
if a.ready_file and a.ready_file.exists():
    raise FileExistsError(a.ready_file)
deadline = time.monotonic() + a.wait_seconds
while not (a.trial / 'live.json').is_file():
    if time.monotonic() >= deadline:
        raise TimeoutError('Producer has not published live.json: ' + str(a.trial))
    time.sleep(.1)
reader = MotionLiveReader(a.trial)
scene = reader.verify_source()
report_path = a.trial / ('live_viewer_' + str(os.getpid()) + '.json')
if report_path.exists():
    raise FileExistsError(report_path)
print('[LIVE] opening measured-frame display; physics runs in the producer', flush=True)
from isaacsim import SimulationApp
app = SimulationApp({'headless': a.headless, 'width': 960, 'height': 720,
                     'renderer': 'RaytracedLighting'})
exit_code = 0
report = dict(source=reader.metadata.get('source', 'latest completed measured native physics frame'),
              model_sha256=reader.metadata['model_sha256'], scene=str(scene),
              physics_rerun=False, physics_steps=0, frame_cap_fps=a.fps,
              original_visual_geometry=True, display_environment=0,
              body_count=len(reader.body_paths))
try:
    import numpy as np
    import carb
    import omni.timeline
    import omni.usd
    import omni.ui as ui
    import omni.kit.viewport.utility as vu
    from pxr import Usd
    from motion_pose_display import MeasuredPoseDisplay, restore_recorded_display_fixture
    from motion_visuals import DeformationVisuals, prepare_display_meshes

    # Prevent timeline playback from turning this display process into a simulator.
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_auto_update(False)
    timeline.stop()
    blocked_play_events = 0
    def keep_display_stopped(event):
        global blocked_play_events
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            blocked_play_events += 1
            timeline.stop()
    subscription = timeline.get_timeline_event_stream().create_subscription_to_pop(
        keep_display_stopped, name='Measured live display: no simulation playback')
    settings = carb.settings.get_settings()
    settings.set('/app/runLoops/main/rateLimitFrequency', a.fps)
    settings.set('/app/runLoops/main/rateLimitUseBusyLoop', False)
    settings.set('/app/runLoops/main/rateLimitEnabled', True)
    if not omni.usd.get_context().open_stage(str(scene)):
        raise RuntimeError('Live source USD did not open')
    stage = omni.usd.get_context().get_stage()
    # All display edits are transient. The source asset and producer stay intact.
    stage.SetEditTarget(stage.GetSessionLayer())
    report['recorded_display_fixture'] = restore_recorded_display_fixture(stage,a.trial)
    runtime_path = a.trial / 'runtime_settings.json'
    runtime = json.loads(runtime_path.read_text()) if runtime_path.is_file() else {}
    report['producer_physics_settings'] = {k:runtime.get(k) for k in ('hz','solver','plant_dynamics','realtime_cap')}
    report['fixture_only'] = bool(runtime.get('isolate_background') or report['recorded_display_fixture'])
    report['display_mesh_preparation'] = prepare_display_meshes(stage)
    pose_display = MeasuredPoseDisplay(stage, reader.body_paths)
    visuals = DeformationVisuals(stage, Path(reader.metadata['assembly']), reader.body_paths)
    viewport = vu.get_active_viewport()
    viewport.set_texture_resolution((960, 720))
    if a.camera == 'entry':
        from build_scene import camera
        from scipy.spatial.transform import Rotation
        snapshot = reader.read()
        if snapshot is None:
            raise RuntimeError('Entry camera requires an initial measured frame')
        pose = snapshot['poses'][reader.body_paths.index('/World/ActuatedTool/fixed')]
        manifest = json.loads((Path(reader.metadata['assembly'])/'motion_manifest.json').read_text())
        center = (pose[:3]+np.asarray(manifest['target']['center']))/2+np.array([0.,0.,.18])
        eye = center-Rotation.from_quat(pose[3:]).as_matrix()[:,0]*1.8+np.array([0.,0.,.30])
        a.camera = '/World/LiveEntryCamera'
        camera(stage,a.camera,tuple(eye),tuple(center))
    if not stage.GetPrimAtPath(a.camera):
        raise ValueError('Live camera does not exist: ' + a.camera)
    viewport.set_active_camera(a.camera)
    camera_motion = None
    if a.camera_test_motion:
        from pxr import Gf, UsdGeom
        camera_prim = stage.GetPrimAtPath(a.camera)
        if not camera_prim.IsA(UsdGeom.Camera):
            raise ValueError('Camera motion test requires an actual USD Camera prim')
        base_camera_world = UsdGeom.XformCache().GetLocalToWorldTransform(camera_prim)
        base_camera_array = np.asarray(base_camera_world)
        base_camera_eye = base_camera_array[3, :3].copy()
        camera_right = base_camera_array[0, :3].copy()
        camera_up = base_camera_array[1, :3].copy()
        camera_backward = base_camera_array[2, :3].copy()
        camera_target = base_camera_eye - 1.8 * camera_backward / np.linalg.norm(camera_backward)
        camera_parent_inverse = UsdGeom.XformCache().GetLocalToWorldTransform(
            camera_prim.GetParent()).GetInverse()
        camera_transform = UsdGeom.Xformable(camera_prim).MakeMatrixXform()
        camera_motion = CameraMotionEvidence(a.camera, base_camera_eye)
    report['camera_test_motion'] = dict(enabled=False) if camera_motion is None else camera_motion.summary()
    if a.review_light:
        # Reuse render_motion.py's inspection light, with all edits confined to
        # this independent viewer's anonymous session layer.
        from pxr import Gf, UsdGeom, UsdLux
        eye = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(
            stage.GetPrimAtPath(a.camera)))[3, :3]
        light = UsdLux.SphereLight.Define(stage, '/World/LiveReviewFillLight')
        light.CreateIntensityAttr(1800.)
        light.CreateRadiusAttr(.5)
        UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(*eye))
        report['display_review_fill_light'] = dict(path=str(light.GetPath()),
            intensity=1800., radius_m=.5, world_position=eye.tolist(),
            source='existing render_motion.py --review-light parameters',
            physical_source_modified=False)
    window = ui.Window('Measured physics live display', width=450, height=140)
    with window.frame:
        with ui.VStack():
            ui.Label('Separate viewer · measured poses · physics remains in producer')
            ui.Label(f"Physics: {runtime.get('solver','?')} / {runtime.get('hz','?')} Hz / {runtime.get('plant_dynamics','?')} plant")
            if report['fixture_only']:
                ui.Label('COMPONENT FIXTURE — NOT A VALIDATED FARM HARVEST')
            status_label = ui.Label('Waiting for the first completed measured frame')
            ui.Label('Space / Play does not start physics in this viewer')
    last = None
    observed_sequences = []
    updates = 0
    render_updates = 0
    update_wall = []
    first_running = None
    last_notice = -float('inf')
    ready_written = False
    last_ack_sequence = None
    received_stop_signal = None
    def request_viewer_stop(signum, frame):
        global received_stop_signal
        received_stop_signal = signum
    previous_signal_handlers = {signum: signal.signal(signum, request_viewer_stop)
                                for signum in (signal.SIGINT, signal.SIGTERM)}
    started = time.monotonic()
    while (app.is_running() and received_stop_signal is None
           and (not a.seconds or time.monotonic() - started < a.seconds)):
        frame_start = time.monotonic()
        snapshot = reader.read()
        if snapshot is not None and (last is None or snapshot['sequence'] != last['sequence']):
            update_start = time.perf_counter()
            pose_display.update(snapshot['poses'])
            visuals.update(snapshot['poses'], snapshot['gap'])
            update_wall.append(time.perf_counter() - update_start)
            last = snapshot
            observed_sequences.append(dict(sequence=last['sequence'], time_s=last['time'],
                                           status=last['status']))
            updates += 1
            now = time.monotonic()
            if first_running is None and last['time'] > 0:
                first_running = (now,last['time'])
            rate = ((last['time']-first_running[1])/max(now-first_running[0],1e-9)
                    if first_running else 0.)
            status_label.text = (f"Measured t={last['time']:.3f}s · {rate:.2f}x realtime"
                                 f" · {last['status']}")
            if now-last_notice >= 1. or last['status'] in ('completed','stopped'):
                print(f"[LIVE] measured t={last['time']:.3f}s · {rate:.2f}x realtime · {last['status']}", flush=True)
                last_notice = now
        if camera_motion is not None:
            moved_eye = camera_test_position(base_camera_eye, camera_right, camera_up,
                                            time.monotonic() - started)
            moved_world = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*moved_eye),
                Gf.Vec3d(*camera_target), Gf.Vec3d(*camera_up)).GetInverse()
            # USD row-vector convention: local * parent_world = world.
            camera_transform.Set(moved_world * camera_parent_inverse)
        timeline.stop()
        app.update()
        render_updates += 1
        if timeline.get_current_time() != 0.:
            raise RuntimeError('Live display timeline unexpectedly advanced')
        if camera_motion is not None and last is not None:
            actual_camera_world = UsdGeom.XformCache().GetLocalToWorldTransform(camera_prim)
            camera_motion.record(np.asarray(actual_camera_world)[3, :3],
                render_update=render_updates, elapsed_wall_s=time.monotonic() - started,
                measured_time_s=last['time'], timeline_time_s=timeline.get_current_time())
        if last is not None and (last['sequence'] != last_ack_sequence or camera_motion is not None):
            write_display_ack(a.trial, last, displayed_count=updates,
                              render_updates=render_updates,
                              timeline_time_s=timeline.get_current_time(),
                              camera_motion=None if camera_motion is None else camera_motion.summary())
            last_ack_sequence = last['sequence']
        if a.ready_file and not ready_written and last is not None:
            a.ready_file.parent.mkdir(parents=True,exist_ok=True)
            # This is a readiness signal, not a command or physics step.
            with a.ready_file.open('x') as stream:
                json.dump(dict(pid=os.getpid(),model_sha256=reader.metadata['model_sha256'],
                               measured_time_s=last['time'],physics_steps=0),stream)
            ready_written = True
        remaining = 1 / a.fps - (time.monotonic() - frame_start)
        if remaining > 0:
            time.sleep(remaining)
    if a.capture:
        if last is None:
            raise RuntimeError('No measured frame was available for PNG capture')
        a.capture.parent.mkdir(parents=True, exist_ok=True)
        for _ in range(3):
            timeline.stop()
            app.update()
        capture = vu.capture_viewport_to_file(viewport, str(a.capture))
        task = asyncio.ensure_future(capture.wait_for_result())
        deadline = time.monotonic() + 20.
        while not task.done():
            if time.monotonic() > deadline:
                raise TimeoutError('Live PNG capture did not complete')
            timeline.stop()
            app.update()
        task.result()
        while not a.capture.exists() or a.capture.read_bytes()[-12:] != b'\x00\x00\x00\x00IEND\xaeB`\x82':
            if time.monotonic() > deadline:
                raise TimeoutError('Live PNG file write did not complete')
            timeline.stop()
            app.update()
        import hashlib
        capture_report = dict(source=reader.metadata.get('source', 'latest completed measured native physics frame'),
            model_sha256=reader.metadata['model_sha256'], camera=a.camera,
            sequence=last['sequence'], measured_time_s=last['time'], measured_gap_m=last['gap'],
            status=last['status'], pose_sha256=hashlib.sha256(last['poses'].astype('<f8').tobytes()).hexdigest(),
            physics_rerun=False, physics_steps=0)
        if camera_motion is not None:
            capture_report['camera_test_motion'] = camera_motion.summary()
        a.capture.with_suffix('.json').write_text(json.dumps(capture_report, indent=2) + '\n')
    report.update(displayed_measured_frames=updates, render_updates=render_updates,
                  elapsed_wall_s=time.monotonic()-started,
                  last_sequence=None if last is None else last['sequence'],
                  last_measured_time_s=None if last is None else last['time'],
                  last_status=None if last is None else last['status'],
                  display_update_mean_s=float(np.mean(update_wall)) if update_wall else None,
                  display_update_max_s=max(update_wall) if update_wall else None,
                  timeline_time_s=timeline.get_current_time(), timeline_auto_update=False,
                  blocked_play_events=blocked_play_events,
                  last_acknowledged_sequence=last_ack_sequence,
                  received_stop_signal=received_stop_signal,
                  display_publications=display_publication_metrics(observed_sequences,
                      json.loads((a.trial / 'live.json').read_text())),
                  displayed_sequence_timestamps=observed_sequences)
    if camera_motion is not None:
        motion_report = camera_motion.summary()
        motion_report['samples'] = camera_motion.samples
        motion_path = a.trial / 'live_camera_motion.json'
        motion_path.write_text(json.dumps(motion_report, indent=2, allow_nan=False) + '\n')
        report['camera_test_motion'] = camera_motion.summary()
        report['camera_test_motion']['evidence_file'] = str(motion_path)
    for signum, handler in previous_signal_handlers.items():
        signal.signal(signum, handler)
except Exception:
    exit_code = 1
    report['error'] = traceback.format_exc()
    print(report['error'], flush=True)
finally:
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    app.close(exit_code=exit_code)
