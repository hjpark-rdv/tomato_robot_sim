"""Read-only summaries of bounded trials; sums, individual loads and times stay separate."""
import argparse
import gzip
import json
from pathlib import Path

from target_fruit_contact_trial import ContactScope, category


def load_gz(path):
    return json.loads(gzip.decompress(Path(path).read_bytes()))


def force_interval(rows, field, role, dt):
    times = [r['time_s'] - (dt if field.endswith('live') else 0.) for r in rows]
    values = [r[field][role]['force_sum_N'] for r in rows]
    hit = [i for i, f in enumerate(values) if f > 0.]
    peak = max(range(len(values)), key=values.__getitem__)
    # Elapsed observed duration within contiguous positive samples, not N * dt.
    duration = sum(times[b] - times[a] for a, b in zip(hit, hit[1:])
                   if b == a + 1 and times[b] - times[a] <= dt * 1.5)
    return dict(max_force_sum_N=values[peak], max_time_s=times[peak],
                first_positive_time_s=times[hit[0]] if hit else None,
                last_positive_time_s=times[hit[-1]] if hit else None,
                positive_samples=len(hit), observed_positive_elapsed_s=duration,
                max_single_normal_N=max(r[field][role]['max_normal_N'] for r in rows))


def summarize(folder):
    folder = Path(folder)
    result = json.loads((folder/'target_contact_trial.json').read_text())
    audit = json.loads((folder/'environment_preflight.json').read_text())
    out = dict(trial=result, audit={k: audit[k] for k in
               ('passed', 'complete', 'status', 'samples_checked', 'last_sample_time_s',
                'first_violation', 'violation_summary')})
    if not result['physics_executed']:
        return out
    rows = load_gz(folder/'authorized_contact_samples.json.gz')
    events = load_gz(folder/'contact_events_240hz.json.gz')
    scope = ContactScope(**json.loads((folder/'contact_scope.json').read_text())['scope'])
    dt = 1. / result['legacy_evidence']['sample_hz']
    out['sample_timing'] = dict(first_s=rows[0]['time_s'], last_s=rows[-1]['time_s'],
        count=len(rows), expected_dt_s=dt,
        max_gap_s=max((b['time_s']-a['time_s'] for a,b in zip(rows,rows[1:])), default=None),
        missing_gap_count=sum(b['time_s']-a['time_s'] > dt*1.5 for a,b in zip(rows,rows[1:])))
    out['forces'] = {field: {role: force_interval(rows, field, role, dt)
                            for role in rows[0][field]}
                     for field in ('contact_categories_live','contact_categories_private')}
    for key in ('target_displacement_m', 'guard_penetration_m', 'prismatic_error_m', 'revolute_error_rad'):
        r = max(rows, key=lambda r:r[key])
        out[key] = dict(max_value=r[key], time_s=r['time_s'], phase=r['phase'])
    out['last_phase'] = rows[-1]['phase']
    out['geometric_seated_samples'] = sum(r['seated'] for r in rows)
    out['seated_with_pedicel_force_samples'] = sum(r['seated'] and r['target_contact'] for r in rows)
    out['stop_sample'] = rows[-1]
    lookup = {round(r['time_s'], 9): r for r in rows}
    peaks = {}
    for event in events:
        phases = lookup[round(event['poststep_time_s'],9)]['active_permission_phases']
        for field, timekey in [('live_previous_solve','live_solve_time_s'),('private_forward','private_forward_time_s')]:
            for contact in event[field]:
                role = category(contact, scope, phases)
                key = (field, role, tuple(sorted(contact['geoms'])))
                entry = peaks.setdefault(key, dict(source=field, role=role, geoms=list(key[2]),
                    max_single_normal_N=0., max_penetration_m=0., peak_time_s=None))
                if contact['normal_force_N'] > entry['max_single_normal_N']:
                    entry.update(max_single_normal_N=contact['normal_force_N'],peak_time_s=event[timekey])
                entry['max_penetration_m'] = max(entry['max_penetration_m'], -contact['dist_m'])
    out['pair_peaks'] = sorted(peaks.values(),key=lambda r:r['max_single_normal_N'],reverse=True)
    out['target_fruit_penetration_m'] = max([r['max_penetration_m'] for r in peaks.values()
                                            if scope.fruit in r['geoms']]+[0.])
    out['units_note'] = ('Force sum = sum of per-contact 3D force magnitudes, not resultant or single Fn. '
        'Live force timestamp precedes poststep geometry by dt; private forces recomputed at poststep. '
        'Positive elapsed durations sum consecutive positive sample intervals; no gap filling.')
    if 'actual_tcp_speed_m_s' in rows[0]:
        seat = [r for r in rows if r['phase']=='seat']
        out['seat_actual_tcp_speed_m_s'] = max((r['actual_tcp_speed_m_s'] for r in seat), default=None)
    return out


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial', type=Path); p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists(): p.error('Use a new output; preserve original evidence')
    a.output.write_text(json.dumps(summarize(a.trial),indent=2,allow_nan=False)+'\n')
