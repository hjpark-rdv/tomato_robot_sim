"""Copy and verify measured, time-aligned camera data for the existing replay.

This module never renders, advances physics, or infers pixels from body centers.
Image paths in frames.json are local to that manifest, so the ordinary web
server can serve them without external symlinks.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'nvidia-sim/harvest_s300'))
from recording_storage import share_file

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def safe_file(root, relative):
    require(isinstance(relative, str) and relative, 'Missing relative sensor file')
    path = Path(relative)
    require(not path.is_absolute() and '..' not in path.parts, 'Unsafe sensor file: '+relative)
    answer = (Path(root)/path).resolve()
    require(answer.is_relative_to(Path(root).resolve()) and answer.is_file(),
            'Missing sensor file or outside symlink: '+relative)
    return answer


def source_frames(assembly, trial, sensor_data=None):
    """Validate camera identity against actual recorded body/q samples first."""
    assembly, trial = Path(assembly).resolve(), Path(trial).resolve()
    source = Path(sensor_data).resolve() if sensor_data is not None else trial/'rgbd'
    if not source.is_dir():
        require(sensor_data is None, 'Requested sensor directory is missing: '+str(source))
        return None
    frame_log = source/'frames.jsonl'
    require(frame_log.is_file(), 'Missing actual camera frames.jsonl: '+str(source))
    camera = read_json(source/'camera.json')
    rows = [json.loads(line) for line in frame_log.read_text().splitlines() if line.strip()]
    require(rows and camera.get('frames') == len(rows), 'Camera frame count mismatch')
    require(camera.get('schema') == 's300-mounted-rgbd-v1', 'Unknown mounted camera schema')
    source_states_sha = sha(trial/'states.npz')
    native = read_json(trial/'runtime_settings.json')
    actual_scene = assembly/('composed.usdc' if (assembly/'composed.usdc').is_file() else 'scene.usda')
    source_scene_sha = sha(actual_scene)
    require(native.get('native_scene_sha256') == source_scene_sha,
            'Sensor source assembly differs from recorded physical scene')
    provenance = read_json(source/'provenance.json') if (source/'provenance.json').is_file() else {}
    mode = provenance.get('mode', 'live_physics_capture')
    require(mode in ('live_physics_capture', 'measured_state_render'), 'Unknown camera provenance mode')
    if source != (trial/'rgbd').resolve() or mode == 'measured_state_render':
        require(provenance.get('states_sha256') == source_states_sha,
                'External camera data must identify the exact source states SHA')
        require(provenance.get('scene_sha256') == source_scene_sha,
                'External camera data must identify the exact source scene SHA')
    for name, expected in [('states_sha256', source_states_sha), ('scene_sha256', source_scene_sha)]:
        if name in provenance:
            require(provenance[name] == expected, 'Camera provenance '+name+' mismatch')
    times = np.array([row['timestamp_s'] for row in rows], dtype=float)
    require(np.isfinite(times).all() and np.all(np.diff(times) > 0),
            'Sensor timestamps must be finite and strictly increasing')
    matrix = np.asarray(camera['body_from_camera_optical'], dtype=float)
    require(matrix.shape == (4,4) and np.isfinite(matrix).all(), 'Invalid optical mount transform')
    resolution = tuple(camera['resolution_wh'])
    require(len(resolution) == 2 and min(resolution) > 0, 'Invalid sensor resolution')
    maximum_pose_error = 0.
    assets = []
    with np.load(trial/'states.npz', allow_pickle=False) as states:
        ticks, poses, q = states['time'], states['poses'], states['robot_q']
        body_paths = states['body_paths'].tolist()
        require(camera['body_path'] in body_paths, 'Camera mount body absent from recorded states')
        body_index = body_paths.index(camera['body_path'])
        for row in rows:
            t = float(row['timestamp_s'])
            insertion = int(np.searchsorted(ticks, t))
            index = min((max(0,min(len(ticks)-1,k)) for k in (insertion-1,insertion)),
                        key=lambda k:abs(float(ticks[k])-t))
            require(abs(float(ticks[index])-t) <= 1e-9,
                    'Camera sample has no exact recorded state at '+str(t))
            require(np.array_equal(np.asarray(row['measured_mount_body_pose'], dtype=poses.dtype), poses[index,body_index]),
                    'Camera mount pose differs from recorded physics at '+str(t))
            require(np.array_equal(np.asarray(row['robot_q'], dtype=q.dtype), q[index]),
                    'Camera robot q differs from recorded physics at '+str(t))
            body = poses[index,body_index]
            transform = np.eye(4)
            transform[:3,:3] = Rotation.from_quat(body[3:]).as_matrix()
            transform[:3,3] = body[:3]
            error = float(np.max(np.abs(transform@matrix-np.asarray(row['world_from_camera_optical']))))
            require(error <= 1e-6, 'Optical camera pose mismatch at '+str(t))
            maximum_pose_error = max(maximum_pose_error, error)
            frame_path = Path(row['rgb']).parent
            item = dict(row=row, state_index=index, assets={})
            paths = {'rgb':row['rgb']}
            if row.get('gt'):
                for key in ('recognition','depth_overlay','objects','region','original_recognition'):
                    if row['gt'].get(key):
                        paths[key] = str(frame_path/row['gt'][key])
            elif row.get('depth_preview'):
                paths['depth_overlay'] = row['depth_preview']
            paths['camera_metadata'] = str(frame_path/'camera.json')
            for key, relative in paths.items():
                asset = safe_file(source, relative)
                if key in ('rgb','recognition','depth_overlay','original_recognition'):
                    with Image.open(asset) as image:
                        require(image.size == resolution, 'Camera image dimensions mismatch: '+str(asset))
                item['assets'][key] = dict(relative=relative, sha256=sha(asset))
            assets.append(item)
    return dict(source=source,camera=camera,rows=rows,assets=assets,mode=mode,
                states_sha256=source_states_sha,scene_sha256=source_scene_sha,
                maximum_optical_pose_error=maximum_pose_error,
                frames_sha256=sha(frame_log),camera_sha256=sha(source/'camera.json'),
                provenance=provenance)


def export_camera(assembly, trial, output, sensor_data=None):
    checked = source_frames(assembly, trial, sensor_data)
    if checked is None:
        return None
    destination = Path(output)/'camera'
    destination.mkdir(exist_ok=False)
    frames = []
    for item in checked['assets']:
        row = item['row']
        exported = {key:row[key] for key in ('timestamp_s','phase','physics_step','frame_id') if key in row}
        exported['state_index'] = item['state_index']
        exported['sha256'] = {}
        for key, asset in item['assets'].items():
            target = destination/asset['relative']
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(safe_file(checked['source'],asset['relative']), target)
            require(sha(target) == asset['sha256'], 'Camera asset changed while copying')
            share_file(target, Path(output).parent/'_shared')
            exported[key] = asset['relative']
            exported['sha256'][key] = asset['sha256']
        frames.append(exported)
    for name in ('camera.json','gt_catalog.json'):
        if (checked['source']/name).is_file():
            shutil.copyfile(checked['source']/name,destination/name)
            share_file(destination/name, Path(output).parent/'_shared')
    intervals = np.diff([row['timestamp_s'] for row in frames])
    record = dict(schema='mounted-rgbd-replay-frames-v1',asset_contract_version=2,frames=frames,
        source=dict(sensor_data=str(checked['source']),mode=checked['mode'],
                    states_sha256=checked['states_sha256'],scene_sha256=checked['scene_sha256'],
                    frames_jsonl_sha256=checked['frames_sha256'],camera_json_sha256=checked['camera_sha256'],
                    provenance=checked['provenance']),
        resolution_wh=checked['camera']['resolution_wh'],
        validation=dict(actual_timestamps_preserved=True,exact_recorded_pose_and_q=True,
                        maximum_optical_pose_error=checked['maximum_optical_pose_error']),
        pixel_source='actual renderer acquisition; no reconstructed/drawn camera pixels')
    (destination/'frames.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    require(sha(checked['source']/'frames.jsonl') == checked['frames_sha256'], 'Source camera frames changed during export')
    return dict(schema='mounted-rgbd-replay-v1',frames='camera/frames.json',
                frame_count=len(frames),mode=checked['mode'],
                frames_sha256=sha(destination/'frames.json'),
                timestamp_start_s=frames[0]['timestamp_s'],timestamp_end_s=frames[-1]['timestamp_s'],
                max_frame_age_s=float(np.median(intervals)*1.6) if len(intervals) else 1.,
                available_modes=[key for key in ('rgb','recognition','depth_overlay') if all(key in row for row in frames)])


def validate_camera(replay_path, descriptor, assembly, trial):
    """Independently compare exported bytes and timestamps to original capture."""
    require(descriptor['schema'] == 'mounted-rgbd-replay-v1', 'Unknown replay camera schema')
    base = Path(replay_path).resolve().parent
    manifest = safe_file(base, descriptor['frames'])
    require(sha(manifest) == descriptor['frames_sha256'], 'Published camera manifest SHA mismatch')
    record = read_json(manifest)
    require(record['schema'] == 'mounted-rgbd-replay-frames-v1','Unknown camera frames schema')
    source = record['source']
    checked = source_frames(assembly,trial,source['sensor_data'])
    require(source['mode'] == descriptor['mode'] == checked['mode'],
            'Published camera acquisition mode differs from original provenance')
    require(source['states_sha256'] == checked['states_sha256'] and source['scene_sha256'] == checked['scene_sha256'],
            'Published camera source identity mismatch')
    require(source['frames_jsonl_sha256'] == checked['frames_sha256'] and source['camera_json_sha256'] == checked['camera_sha256'],
            'Published source camera log SHA mismatch')
    require(sha(safe_file(manifest.parent,'camera.json')) == checked['camera_sha256'],
            'Served camera calibration SHA mismatch')
    require(record['frames'] and len(record['frames']) == len(checked['rows']) == descriptor['frame_count'],
            'Published camera frame count mismatch')
    require(descriptor['timestamp_start_s'] == checked['rows'][0]['timestamp_s'] and
            descriptor['timestamp_end_s'] == checked['rows'][-1]['timestamp_s'],
            'Published camera timestamp range mismatch')
    images = 0
    for published, item in zip(record['frames'], checked['assets']):
        original = item['row']
        for key in ('timestamp_s','phase','physics_step','frame_id'):
            if key in original:
                require(published[key] == original[key], 'Published camera '+key+' differs from original')
        require(published['state_index'] == item['state_index'], 'Camera state index mismatch')
        for key, asset in item['assets'].items():
            # Version1 bundles predate optional selected-region JSON/original
            # GT copies. Preserve their served-byte checks without requiring
            # assets that the old immutable exporter never included.
            if record.get('asset_contract_version',1)<2 and key in ('region','original_recognition') and key not in published:
                continue
            target = safe_file(manifest.parent,published[key])
            require(published[key] == asset['relative'], 'Published camera asset path mismatch')
            require(sha(target) == asset['sha256'] == published['sha256'][key],
                    'Published camera '+key+' SHA mismatch')
            images += key in ('rgb','recognition','depth_overlay','original_recognition')
    return dict(frames=len(record['frames']),images=images,actual_timestamps_exact=True,
                recorded_mount_and_q_verified=True,served_asset_hashes_exact=True,
                mode=checked['mode'],source_states_sha256=checked['states_sha256'])
