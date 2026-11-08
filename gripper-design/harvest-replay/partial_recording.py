"""Opt-in bindings for an incomplete actual native trace, never a summary shim.

This reads the original partial descriptor, arrays and error. It neither makes
a normal summary/checkpoint nor changes or resumes the failed simulation.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


KIND = 'partial_diagnostic'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def partial_recording(assembly, trial):
    assembly, trial = Path(assembly).resolve(), Path(trial).resolve()
    if (trial/'states.npz').exists() or (trial/'summary.json').exists():
        raise ValueError('Partial diagnostics cannot replace canonical states or a summary')
    descriptor_path = trial/'partial_trace.json'
    descriptor = read(descriptor_path)
    if (descriptor.get('schema') != 's300-interrupted-actual-trace-v1' or
            any(descriptor.get(k) is not False for k in
                ('complete', 'completed', 'checkpoint_eligible', 'training_eligible',
                 'finalized_native_audit_claimed'))):
        raise ValueError('Partial descriptor must remain incomplete, non-training and checkpoint-ineligible')
    if Path(descriptor.get('source_assembly', '')).resolve() != assembly:
        raise ValueError('Partial recording source assembly differs')
    wrapper = assembly/'scene.usda'
    manifest_path = assembly/'motion_manifest.json'
    manifest = read(manifest_path)
    wrapper_sha = sha(wrapper)
    if (descriptor.get('actual_source_scene_sha256') != wrapper_sha or
            descriptor.get('declared_wrapper_scene_sha256') != wrapper_sha or
            manifest.get('scene_sha256') != wrapper_sha or
            descriptor.get('actual_manifest_sha256') != sha(manifest_path)):
        raise ValueError('Partial model/manifest/source SHA mismatch')
    if descriptor.get('states') != 'states_partial.npz' or descriptor.get('break_events') != 'break_events_partial.json':
        raise ValueError('Partial data must retain its explicit original filenames')
    states_path = trial/'states_partial.npz'
    events_path = trial/'break_events_partial.json'
    for path in (descriptor_path, states_path, events_path, trial/'runtime_error.json',
                 trial/'native_metadata.json', trial/'runtime_settings.json'):
        if not path.is_file() or path.is_symlink() or not path.resolve().is_relative_to(trial):
            raise ValueError('Missing or unsafe original partial evidence: '+str(path))
    if descriptor.get('states_sha256') != sha(states_path):
        raise ValueError('Partial states SHA mismatch')
    runtime = read(trial/'runtime_settings.json')
    native = read(trial/'native_metadata.json')
    scene = assembly/('composed.usdc' if (assembly/'composed.usdc').is_file() else 'scene.usda')
    if (native.get('model_sha256') != wrapper_sha or
            runtime.get('source_scene_sha256') != wrapper_sha or
            runtime.get('native_scene_sha256') != sha(scene)):
        raise ValueError('Partial native/runtime physical source mismatch')
    error = read(trial/'runtime_error.json')
    if not isinstance(error, dict) or not isinstance(error.get('error'), str) or not error['error'].strip():
        raise ValueError('Partial diagnostic requires its actual recorded runtime error')
    with np.load(states_path, allow_pickle=False) as arrays:
        times, poses, robot = arrays['time'], arrays['poses'], arrays['robot_q']
        paths = arrays['body_paths'].tolist()
        phases = arrays['phase']
        dofs = native.get('robot_dofs', [])
        if (times.ndim != 1 or len(times) < 2 or poses.shape != (len(times), len(paths), 7) or
                robot.shape != (len(times), len(dofs)) or phases.shape != times.shape or
                len(set(paths)) != len(paths) or paths != native.get('body_paths') or
                len(set(dofs)) != len(dofs) or not {'jaw_L', 'jaw_R'} <= set(dofs)):
            raise ValueError('Partial measured arrays/native body or joint mapping mismatch')
        if (not all(np.isfinite(a).all() for a in (times, poses, robot)) or
                np.any(np.diff(times) <= 0) or np.any(np.linalg.norm(poses[:, :, 3:], axis=2) <= 1e-12)):
            raise ValueError('Partial recorded values/timestamps are invalid')
        if (descriptor.get('samples') != len(times) or
                descriptor.get('first_timestamp_s') != float(times[0]) or
                descriptor.get('last_timestamp_s') != float(times[-1])):
            raise ValueError('Partial descriptor sample/time bounds differ from actual arrays')
        for key in ('time', 'poses', 'robot_q', 'body_paths', 'phase'):
            if descriptor.get('array_shapes', {}).get(key) != list(arrays[key].shape):
                raise ValueError('Partial descriptor array shape differs: '+key)
    events = read(events_path)
    if not isinstance(events, list):
        raise ValueError('Partial break events must be the original recorded list')
    return dict(states_path=states_path, events_path=events_path, descriptor_path=descriptor_path,
                descriptor=descriptor, runtime_error=error, runtime=runtime, native=native,
                result=dict(record_kind=KIND, completed=False,
                    stop_reason='software_exception_partial_trace', harvest_success=None,
                    released_fruits=len(events), new_break_count=None, physics_steps=None,
                    physics_wall_s=None, simulated_s=descriptor['last_timestamp_s'],
                    training_eligible=False, diagnostic_only=True, checkpoint_eligible=False,
                    harvest_credit=0, finalized_native_audit_claimed=False,
                    recorded_samples=descriptor['samples'], runtime_error=error['error']))


def published_partial_source(partial, trial):
    trial = Path(trial)
    return dict(record_kind=KIND, states_file='states_partial.npz', summary_sha256=None,
                partial_trace_file='partial_trace.json',
                partial_trace_sha256=sha(partial['descriptor_path']),
                runtime_error_sha256=sha(trial/'runtime_error.json'),
                native_metadata_sha256=sha(trial/'native_metadata.json'),
                break_events_sha256=sha(partial['events_path']),
                diagnostic_only=True, checkpoint_eligible=False, harvest_credit=0,
                finalized_summary_claimed=False, finalized_native_audit_claimed=False)
