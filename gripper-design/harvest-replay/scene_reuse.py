"""Strict opt-in reuse of an independently verified identical physical scene.

Only immutable geometry is shared. New poses/timestamps/jaws/releases/camera
remain those of the new recording. Cached first poses retain their own source.
"""
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def reference_identity(reference, *, scene_sha256, manifest_sha256, body_paths):
    reference=Path(reference).resolve()
    if reference.is_dir():reference=reference/'replay.json'
    replay=json.loads(reference.read_text());source=replay['source']
    if (replay.get('schema')!='farmily-harvest-replay-v1' or replay.get('geometry_reuse') or
            source.get('record_kind')=='partial_diagnostic' or source.get('all_frames_exported') is not True or
            source.get('physics_rerun') is not False or replay.get('source_geometry_modified') is not False):
        raise ValueError('Scene reference must be an original complete preserved replay, never a cache chain or partial trace')
    if source['scene_sha256']!=scene_sha256 or source['manifest_sha256']!=manifest_sha256:
        raise ValueError('Scene reuse requires identical actual physical scene AND manifest SHA')
    trial=Path(source['trial']);assembly=Path(source['assembly'])
    if (trial/'belt_fixture.json').exists():raise ValueError('Transient visual fixtures cannot share scene geometry')
    actual_scene=assembly/('composed.usdc' if (assembly/'composed.usdc').is_file() else 'scene.usda')
    if sha(actual_scene)!=scene_sha256 or sha(assembly/'motion_manifest.json')!=manifest_sha256:
        raise ValueError('Reference physical source or manifest changed')
    if sha(trial/'states.npz')!=source['states_sha256']:
        raise ValueError('Reference actual state source changed')
    with np.load(trial/'states.npz',allow_pickle=False) as states:
        if states['body_paths'].tolist()!=list(body_paths):raise ValueError('Scene reuse requires exact complete body path order')
        reference_poses=np.array(states['poses'][0],copy=True)
    from validate_replay import local_url
    info_path=local_url(reference,replay['scene_info_url']);info=json.loads(info_path.read_text())
    glb_path=local_url(reference,replay['scene_url'])
    if (info.get('source_scene_sha256')!=scene_sha256 or info.get('fixture') is not None or
            info.get('geometry_reuse') or sha(glb_path)!=info['compressed_sha256']):
        raise ValueError('Cached scene identity/fixture/compressed bytes differ')
    proof=dict(schema='farmily-exact-model-scene-reuse-v1',reference_replay=str(reference),
        reference_replay_sha256=sha(reference),reference_scene_info=str(info_path),
        reference_scene_info_sha256=sha(info_path),physical_scene_sha256=scene_sha256,
        manifest_sha256=manifest_sha256,reference_states_sha256=source['states_sha256'],
        compressed_scene_sha256=info['compressed_sha256'],glb_sha256=info['sha256'],
        complete_body_mapping_exact=True,all_new_poses_applied_before_display=True,
        geometry_changed=False,physics_rerun=False)
    return proof,info,glb_path,reference_poses


def verify_reference_proof(proof, *, scene_sha256, manifest_sha256, body_paths):
    expected,info,glb,poses=reference_identity(proof['reference_replay'],scene_sha256=scene_sha256,
        manifest_sha256=manifest_sha256,body_paths=body_paths)
    if proof!=expected:raise ValueError('Scene reuse reference evidence changed')
    return info,glb,poses
