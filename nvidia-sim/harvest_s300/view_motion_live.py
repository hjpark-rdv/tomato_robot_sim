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
import time
import traceback
from pathlib import Path

from motion_live import MotionLiveReader

p = argparse.ArgumentParser()
p.add_argument('trial', type=Path)
p.add_argument('--fps', type=float, default=10., help='wall-clock display cap only')
p.add_argument('--seconds', type=float, default=0., help='0 leaves the window open; otherwise bounded wall time')
p.add_argument('--camera', default='/World/HarvestCloseCamera')
p.add_argument('--headless', action='store_true', help='render/capture validation without a display window')
p.add_argument('--capture', type=Path, help='save a PNG of the final displayed measured frame')
p.add_argument('--wait-seconds', type=float, default=60., help='wait for the producer to create its live metadata')
a = p.parse_args()
if not 0 < a.fps <= 60 or a.seconds < 0 or a.wait_seconds < 0:
    p.error('fps must be in (0,60], seconds and wait-seconds must be nonnegative')
if a.headless and a.seconds == 0:
    p.error('--headless requires a bounded --seconds value')
if a.capture and (a.capture.exists() or a.capture.with_suffix('.json').exists()):
    raise FileExistsError(a.capture)
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
report = dict(source='latest completed measured native physics frame',
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
    def keep_display_stopped(event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
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
    report['fixture_only'] = bool(runtime.get('isolate_background') or report['recorded_display_fixture'])
    report['display_mesh_preparation'] = prepare_display_meshes(stage)
    pose_display = MeasuredPoseDisplay(stage, reader.body_paths)
    visuals = DeformationVisuals(stage, Path(reader.metadata['assembly']), reader.body_paths)
    viewport = vu.get_active_viewport()
    viewport.set_texture_resolution((960, 720))
    if not stage.GetPrimAtPath(a.camera):
        raise ValueError('Live camera does not exist: ' + a.camera)
    viewport.set_active_camera(a.camera)
    window = ui.Window('Measured physics live display', width=410, height=110)
    with window.frame:
        with ui.VStack():
            ui.Label('Separate viewer · measured native poses · physics remains in producer')
            if report['fixture_only']:
                ui.Label('COMPONENT FIXTURE — NOT A VALIDATED FARM HARVEST')
            status_label = ui.Label('Waiting for the first completed measured frame')
            ui.Label('Space / Play does not start physics in this viewer')
    last = None
    updates = 0
    render_updates = 0
    update_wall = []
    started = time.monotonic()
    while app.is_running() and (not a.seconds or time.monotonic() - started < a.seconds):
        frame_start = time.monotonic()
        snapshot = reader.read()
        if snapshot is not None and (last is None or snapshot['sequence'] != last['sequence']):
            update_start = time.perf_counter()
            pose_display.update(snapshot['poses'])
            visuals.update(snapshot['poses'], snapshot['gap'])
            update_wall.append(time.perf_counter() - update_start)
            last = snapshot
            updates += 1
            status_label.text = (f"Measured t={last['time']:.4f}s · gap={last['gap']*1000:.2f} mm"
                                 f" · {last['status']}")
            print(f"[LIVE] measured t={last['time']:.4f}s {last['status']}", flush=True)
        timeline.stop()
        app.update()
        render_updates += 1
        if timeline.get_current_time() != 0.:
            raise RuntimeError('Live display timeline unexpectedly advanced')
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
        capture_report = dict(source='latest completed measured native physics frame',
            model_sha256=reader.metadata['model_sha256'], camera=a.camera,
            sequence=last['sequence'], measured_time_s=last['time'], measured_gap_m=last['gap'],
            status=last['status'], pose_sha256=hashlib.sha256(last['poses'].astype('<f8').tobytes()).hexdigest(),
            physics_rerun=False, physics_steps=0)
        a.capture.with_suffix('.json').write_text(json.dumps(capture_report, indent=2) + '\n')
    report.update(displayed_measured_frames=updates, render_updates=render_updates,
                  elapsed_wall_s=time.monotonic()-started,
                  last_sequence=None if last is None else last['sequence'],
                  last_measured_time_s=None if last is None else last['time'],
                  last_status=None if last is None else last['status'],
                  display_update_mean_s=float(np.mean(update_wall)) if update_wall else None,
                  display_update_max_s=max(update_wall) if update_wall else None,
                  timeline_time_s=timeline.get_current_time())
except Exception:
    exit_code = 1
    report['error'] = traceback.format_exc()
    print(report['error'], flush=True)
finally:
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    app.close(exit_code=exit_code)
