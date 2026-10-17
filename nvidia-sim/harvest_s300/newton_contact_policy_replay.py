"""Replay stop logic over stored GPU contacts; never advances physics.

The strict result must agree with the actual GPU audit before counterfactual
thresholds are interpreted.  A complete diagnostic-continuation trajectory is
needed: a previously stopped trajectory cannot establish its unseen suffix.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from newton_contact_policy import OPEN_PHASES, validate_config


NO_EVENT = int(np.iinfo(np.uint64).max)


def model_hash_evidence(summary):
    """Keep physical input and display compatibility hashes in separate fields."""
    return dict(physical_input_mjb_sha256=hashlib.sha256(
        (Path(summary['input'])/'input_model.mjb').read_bytes()).hexdigest(),
        physical_model_arrays_sha256=summary.get('engine',{}).get('source_model_array_sha256'),
        display_source_model_sha256=summary.get('display_source_model_sha256'))


def replay_stop_policy(witness, phases, schema, metadata, config):
    """CPU reproduction of the opt-in GPU pair/phase policy, using float32.

    Geometry/force/attached states are read from the original recorded solve.
    This computes a counterfactual stop decision on that same trajectory, not
    a new trajectory that continued after different physical conditions.
    """
    config = validate_config(config)
    pairs = witness['geom_pairs']
    depth = np.maximum(np.float32(0), -witness['signed_distance_m'])
    active = witness['constraint_active']
    attached = witness['fruit_attached_before_release']
    times = witness['actual_step_end_time_s'].astype(np.float32)
    steps = len(times)
    if len(phases) != steps or pairs.shape[:2] != active.shape or depth.shape != active.shape:
        raise ValueError('Witness shape or phase count mismatch')
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError('Actual witness timestamps must be finite and strictly increasing')
    geoms = {int(row['id']): row for row in schema['geoms']}
    names = {row['name']: g for g, row in geoms.items()}
    belts = {g for g, row in geoms.items() if row['body'] in metadata['robot']['belt_body_names']}
    fruit = {s['name'] for s in metadata['attachments']}
    trusses = {name.split('_Fruit_', 1)[0] for name in fruit}
    roles = {g: 1 if row['body'] in fruit else 2 if any(
        row['body'].startswith(truss+'_Pedicel_') for truss in trusses) else 0
        for g, row in geoms.items()}
    tools = set(metadata['tools']) | {'Robot_world', 'Robot_S300Carrier'}
    release_exceptions = set()
    for g, row in geoms.items():
        if row['body'] not in fruit:
            continue
        truss = row['body'].split('_Fruit_', 1)[0]
        for other, record in geoms.items():
            if (record['body'] in tools or record['body'].startswith(truss+'_') or
                    (record['body'] == 'world' and
                     ('ground' in record['name'].lower() or record['name'] == 'CatchFloor'))):
                release_exceptions.add((g, other))
    explicit = {}
    for row in config['explicit_other_pairs']:
        key = tuple(sorted(names[name] for name in row['geoms']))
        if key[0] in belts or key[1] in belts:
            raise ValueError('Exact other-pair policy overlaps a belt')
        explicit[key] = (np.float32(row['limit_m']), row.get('phases', ['*']))
    threshold = [np.float32(config[name]) for name in ('default_m', 'belt_fruit_m', 'belt_pedicel_m')]
    residual_s = np.float32(config['residual_open_seconds'])
    latch, open_start = {}, {}
    first = NO_EVENT
    maxima = np.zeros((steps, 4), np.float32)
    allowed, residual = np.zeros(3, np.int64), np.zeros(3, np.int64)
    pair_stats = {}
    for t in range(steps):
        pair_depth = {}
        n = int(witness['nacon'][t])
        if n < 0 or n > pairs.shape[1]:
            raise ValueError('Witness contact capacity overflow or invalid nacon')
        for c in range(n):
            if not active[t, c]:
                continue
            a, b = (int(v) for v in pairs[t, c])
            if a < 0 or b < 0:
                continue
            if ((attached[t, c, 0] == 0 and (a, b) in release_exceptions) or
                    (attached[t, c, 1] == 0 and (b, a) in release_exceptions)):
                continue
            key = tuple(sorted((a, b)))
            pen = depth[t, c]
            belt_other = (a, b) if a in belts and roles[b] > 0 else (b, a) if b in belts and roles[a] > 0 else None
            category = roles[belt_other[1]] if belt_other else 3 if key in explicit else 0
            maxima[t, category] = max(maxima[t, category], pen)
            stat = pair_stats.setdefault(key, dict(pair=[geoms[a]['name'], geoms[b]['name']],
                bodies=[geoms[a]['body'], geoms[b]['body']], category=int(category), active_contact_samples=0,
                over_reference_samples=0, max_depth_m=0., max_force_n=0.,
                max_depth_step=None, max_depth_time_s=None, max_depth_phase=None,
                fruit_attached_at_max_depth=None, allowed_pair_steps=0, residual_pair_steps=0))
            stat['active_contact_samples'] += 1
            stat['over_reference_samples'] += int(pen > threshold[0])
            stat['max_force_n'] = max(stat['max_force_n'], float(np.linalg.norm(
                witness['world_wrench_on_second_geom'][t, c, :3])))
            if pen > stat['max_depth_m']:
                stat.update(max_depth_m=float(pen), max_depth_step=t, max_depth_time_s=float(times[t]),
                    max_depth_phase=str(phases[t]), fruit_attached_at_max_depth=attached[t, c].tolist(),
                    force_at_max_depth_n=float(np.linalg.norm(witness['world_wrench_on_second_geom'][t,c,:3])))
            if belt_other:
                pair_depth[belt_other] = max(pair_depth.get(belt_other, np.float32(0)), pen)
            else:
                limit = threshold[0]
                if key in explicit:
                    value, selected = explicit[key]
                    if selected == ['*'] or phases[t] in selected:
                        limit = value
                if pen > limit:
                    first = min(first, (t << 32) | (key[0]*65536+key[1]))
        for key in set(latch) | set(pair_depth):
            pen = pair_depth.get(key, np.float32(0))
            role = roles[key[1]]
            limit = threshold[0]
            is_residual = False
            if pen <= threshold[0]:
                latch[key], open_start[key] = False, np.float32(-1)
            elif phases[t] in config['relaxed_phases']:
                limit = threshold[role]
                latch[key], open_start[key] = bool(pen <= limit), np.float32(-1)
            elif phases[t] in OPEN_PHASES and latch.get(key, False) and residual_s > 0:
                if open_start.get(key, np.float32(-1)) < 0:
                    open_start[key] = times[t]
                if np.float32(times[t]-open_start[key]) < residual_s:
                    limit, is_residual = threshold[role], True
                else:
                    latch[key] = False
            else:
                latch[key], open_start[key] = False, np.float32(-1)
            ordered = tuple(sorted(key))
            if pen > threshold[0] and pen <= limit:
                allowed[role] += 1
                residual[role] += int(is_residual)
                stat = pair_stats[ordered]
                stat['allowed_pair_steps'] += 1
                stat['residual_pair_steps'] += int(is_residual)
            if pen > limit:
                first = min(first, (t << 32) | (ordered[0]*65536+ordered[1]))
    event = None
    if first != NO_EVENT:
        tick = first >> 32
        ids = ((first & 0xffffffff)//65536, (first & 0xffffffff) % 65536)
        event = dict(step=tick, actual_step_end_time_s=float(times[tick]), phase=str(phases[tick]),
            geoms=[geoms[g]['name'] for g in ids], bodies=[geoms[g]['body'] for g in ids])
    return dict(schema='s300-contact-policy-posthoc-v1', config=config, physics_rerun=False,
        throughput_eligible=False, training_eligible=False, harvest_success=None,
        first_policy_event=first, first_event_detail=event, policy_passed=first == NO_EVENT,
        allowed_pair_steps=allowed.tolist(), residual_allowed_pair_steps=residual.tolist(),
        maximum_penetration_m_by_class={name: float(maxima[:, i].max(initial=0)) for i, name in
            enumerate(('other', 'belt_fruit', 'belt_pedicel', 'explicit_other_pair'))},
        pairs=sorted(pair_stats.values(), key=lambda r: r['max_depth_m'], reverse=True),
        max_by_class=maxima,
        scope='Counterfactual stop logic over one recorded diagnostic-continuation solve trajectory. No forces, model parameters, or physical states changed.')


def replay_folder(folder, policy_paths, output):
    folder, output = Path(folder), Path(output)
    summary = json.loads((folder/'summary.json').read_text())
    schema = json.loads((folder/'contact_witness_schema.json').read_text())
    metadata = json.loads((Path(summary['input'])/'source_export.json').read_text())
    witness = dict(np.load(folder/'contact_witness.npz'))
    states = np.load(folder/'states.npz')
    if summary['record_stride'] != 1 or len(states['phase']) != len(witness['nacon'])+1:
        raise ValueError('Exact recorded phases require full-rate states with an initial sample')
    phases = states['phase'][1:]
    strict = replay_stop_policy(witness, phases, schema, metadata, {})
    metrics = np.load(folder/'physics_metrics.npz')
    raw_event = int(metrics['first_event'][0])
    if strict['first_policy_event'] != raw_event:
        raise ValueError(f"Strict post-hoc/GPU first event mismatch: {strict['first_policy_event']} != {raw_event}")
    saved_policy=json.loads((folder/'contact_policy.json').read_text())
    saved=replay_stop_policy(witness,phases,schema,metadata,saved_policy['config'])
    saved_metrics=np.load(folder/'contact_policy_metrics.npz')
    saved_checks=dict(first_event=saved['first_policy_event']==int(saved_metrics['first_policy_event'][0]),
        class_maxima=np.array_equal(saved['max_by_class'],saved_metrics['class_max_penetration_m'][0]),
        allowed_pair_steps=saved['allowed_pair_steps']==saved_metrics['allowed_pair_steps'][0].tolist(),
        residual_pair_steps=saved['residual_allowed_pair_steps']==saved_metrics['residual_allowed_pair_steps'][0].tolist())
    if not all(saved_checks.values()):
        raise ValueError('Saved post-hoc/GPU policy reproduction mismatch: '+repr(saved_checks))
    strict.pop('max_by_class')
    rows = [dict(name='reference_1mm', **strict)]
    for path in policy_paths:
        path = Path(path)
        row = replay_stop_policy(witness, phases, schema, metadata, json.loads(path.read_text()))
        row.pop('max_by_class')
        rows.append(dict(name=path.stem, policy_path=str(path), **row))
    # Actual CONNECT forces remain distinct from external contact forces.
    # Use exact anchor-body mapping; no guessed pedicel numbering is required.
    for row in rows:
        for pair in row['pairs']:
            t=pair['max_depth_step']
            related=[]
            if t is None:
                continue
            for index,spec in enumerate(metadata['attachments']):
                if spec['anchor'] in pair['bodies'] or spec['name'] in pair['bodies']:
                    broken=int(metrics['break_step'][0,index])
                    related.append(dict(name=spec['name'],anchor=spec['anchor'],
                        cached_attached_at_peak=broken<0 or t<=broken,
                        eq_active_after_release_at_peak=broken<0 or t<broken,
                        break_step=broken,
                        attachment_constraint_force_at_peak_n=float(metrics['attachment_force_n'][0,t,index]),
                        attachment_torque_at_peak_nm=float(metrics['attachment_torque_nm'][0,t,index])))
            pair['related_exact_attachments']=related
    sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
    result = dict(source=str(folder), strict_first_event_matches_actual_gpu=True,
        saved_policy_reproduction_checks=saved_checks,
        source_witness_sha256=sha(folder/'contact_witness.npz'),
        source_states_sha256=sha(folder/'states.npz'),
        **model_hash_evidence(summary),
        replay_code_sha256=sha(__file__),
        attachment_state_scope='Cached control-callback c.attached: pre-current-release. Equality state may already change in release_attachments before audit.',
        strict_gpu_first_event=raw_event, rows=rows, physics_rerun=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--policy', type=Path, action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = replay_folder(args.folder, args.policy, args.output)
    print(json.dumps(dict(output=str(args.output), count=len(result['rows']),
        strict_first_event_matches_actual_gpu=result['strict_first_event_matches_actual_gpu']), indent=2))
