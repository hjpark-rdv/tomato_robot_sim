"""Match native static/static CONNECT handling for opt-in frozen plants.

MW3.15 emits three constraint rows even with no movable DOF on either side.
Native MuJoCo omits those rows. On frozen plants tiny anchor differences then
produce enormous meaningless constraint forces. Retain equality identity and
attachment activity, but omit exactly these zero-DOF equalities from the GPU
CONNECT launch list. No collider, dynamic constraint or release state changes.
"""
import hashlib
import json

import numpy as np
import mujoco as mj
import warp as wp


def frozen_connect_ids(source, metadata):
    names = metadata.get('active_window', {}).get('frozen_attachment_names', [])
    known = {row['name'] for row in metadata['attachments']}
    if len(names) != len(set(names)) or not set(names) <= known:
        raise ValueError('Unknown or duplicate frozen attachment identity')
    ids = []
    for name in names:
        eqid = source.equality(name + '_attachment').id
        if source.eq_type[eqid] != mj.mjtEq.mjEQ_CONNECT:
            raise ValueError('Frozen attachment is not CONNECT')
        pair = [int(source.eq_obj1id[eqid]), int(source.eq_obj2id[eqid])]
        if source.eq_objtype[eqid] == mj.mjtObj.mjOBJ_SITE:
            pair = [int(source.site_bodyid[i]) for i in pair]
        elif source.eq_objtype[eqid] != mj.mjtObj.mjOBJ_BODY:
            raise ValueError('Unsupported frozen CONNECT object type')
        if any(i < 0 or source.body_weldid[i] != 0 for i in pair):
            raise ValueError('Refusing to omit a CONNECT with any movable ancestor: ' + name)
        if source.body(name).id not in pair:
            raise ValueError('Frozen CONNECT is not attached to its named fruit')
        ids.append(eqid)
    return ids


def _source_identity(source):
    """Hash the exact CPU model arrays/constants without copying mesh payloads.

    Session reuse already binds prepared-file hashes. This additional check
    rejects an in-memory source mutation between compatibility calls, including
    collider vertices, rather than treating matching frozen names as enough.
    """
    digest = hashlib.sha256()
    for name in sorted(dir(source)):
        if name.startswith('_'):
            continue
        value = getattr(source, name)
        if isinstance(value, np.ndarray):
            array = np.ascontiguousarray(value)
            digest.update(str((name, array.dtype.str, array.shape)).encode())
            if array.nbytes:
                digest.update(memoryview(array).cast('B'))
        elif isinstance(value, (int, float, bool, np.integer, np.floating)):
            digest.update(repr((name, value)).encode())
        elif name == 'names':
            digest.update(bytes(value))
    for group_name in ('opt', 'stat'):
        group = getattr(source, group_name)
        for name in sorted(dir(group)):
            if name.startswith('_'):
                continue
            value = getattr(group, name)
            if isinstance(value, np.ndarray):
                array = np.ascontiguousarray(value)
                digest.update(str((group_name, name, array.dtype.str, array.shape)).encode())
                if array.nbytes:
                    digest.update(memoryview(array).cast('B'))
            elif isinstance(value, (int, float, bool, np.integer, np.floating)):
                digest.update(repr((group_name, name, value)).encode())
    return digest.hexdigest()


def _metadata_identity(metadata):
    return hashlib.sha256(json.dumps(metadata, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def configure_frozen_connects(solver, metadata):
    ids = frozen_connect_ids(solver.mj_model, metadata)
    previous = getattr(solver, '_s300_frozen_connect_configuration', None)
    if not ids and previous is None:
        return dict(enabled=False, omitted_static_connect_ids=[])
    if solver.use_mujoco_cpu or int(solver._step) != 0:
        raise ValueError('Configure static CONNECT GPU compatibility before stepping')
    model = solver.mjw_model
    signature = _metadata_identity(metadata)
    source_signature = _source_identity(solver.mj_model)
    if previous is not None:
        # A naturally missing ID without this first-call provenance remains an
        # error. Cached sessions retain this exact helper-created array object.
        if (previous['source_model'] is not solver.mj_model or
                previous['gpu_model'] is not model or
                previous['metadata_sha256'] != signature or
                previous['source_model_sha256'] != source_signature or
                previous['frozen_ids'] != tuple(ids)):
            raise ValueError('Frozen CONNECT idempotent reuse source/metadata identity mismatch')
        if (model.eq_connect_adr is not previous['filtered_array'] or
                not np.array_equal(model.eq_connect_adr.numpy(), previous['filtered_values']) or
                not np.array_equal(previous['original_array'].numpy(), previous['original_values'])):
            raise ValueError('Frozen CONNECT idempotent reuse launch list was modified')
        return dict(previous['evidence'], idempotent_reuse=True)
    old_array = model.eq_connect_adr
    old = old_array.numpy()
    if len(set(old.tolist())) != len(old) or not set(ids) <= set(old.tolist()):
        raise ValueError('Frozen CONNECT identities absent from GPU launch list or duplicated')
    keep = np.asarray([i for i in old if int(i) not in set(ids)], dtype=np.int32)
    filtered = wp.array(keep, dtype=int, device=solver.mjw_data.qpos.device)
    model.eq_connect_adr = filtered
    evidence = dict(enabled=True, omitted_static_connect_ids=ids,
        original_connect_count=len(old), dynamic_connect_count=len(keep),
        attachment_activity_unchanged=True, collider_changes=False,
        original_connect_launch_ids=old.tolist(), retained_connect_launch_ids=keep.tolist(),
        source_model_arrays_and_options_sha256=source_signature,
        exact_metadata_sha256=signature, idempotent_reuse=False,
        scope='Only explicit frozen attachment CONNECTs with both weld roots in world; native MuJoCo emits no rows for these static constraints')
    solver._s300_frozen_connect_configuration = dict(source_model=solver.mj_model,
        gpu_model=model, original_array=old_array, original_values=old.copy(),
        filtered_array=filtered, filtered_values=keep.copy(), frozen_ids=tuple(ids),
        metadata_sha256=signature, source_model_sha256=source_signature,
        evidence=evidence)
    return evidence
