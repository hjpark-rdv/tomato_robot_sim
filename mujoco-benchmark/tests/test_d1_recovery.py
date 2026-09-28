"""Source-grounded recovery controls. Native greenhouse replay remains external."""
import copy
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from recovery_design import (SHARED_FAMILIES, recorded_prefix, matched_local_variant,
                             join_recorded_prefix, select_evidence_groups)
import prepare_recovery_batch as batch
import plan_approach_connection as adapter
from approach_connection import JointSpace, handoff_index


def trace():
    return [dict(command=[i*.001, 0.], joints=[0., 0.], phase=p)
            for i, p in enumerate(('ready', 'preapproach', 'approach', 'insert', 'hold', 'verify'))]


def test_recorded_prefix_uses_contiguous_rows_and_original_digest():
    original = trace(); before = copy.deepcopy(original)
    rows, evidence = recorded_prefix(original, [0., 0.])
    assert [r['phase'] for r in rows] == ['ready', 'preapproach', 'approach']
    assert evidence['command_sha256'] == hashlib.sha256(np.array([[.001, 0.], [.002, 0.]]).tobytes()).hexdigest()
    assert evidence['samples'] == 2 and evidence['seconds'] == pytest.approx(2/60)
    assert original == before


@pytest.mark.parametrize('fault', ['wrong_start', 'nonfinite', 'later_approach', 'empty', 'no_suffix', 'not_ready'])
def test_malformed_prefix_is_not_silently_rerouted(fault):
    data = trace(); start = [0., 0.]
    if fault == 'wrong_start': start[0] = .1
    elif fault == 'nonfinite': data[1]['command'][1] = float('nan')
    elif fault == 'later_approach': data[-1]['phase'] = 'approach'
    elif fault == 'empty': data = []
    elif fault == 'no_suffix': data = data[:3]
    elif fault == 'not_ready': data[0]['phase'] = 'approach'
    with pytest.raises(ValueError): recorded_prefix(data, start)


def test_stitch_preserves_prefix_phases_and_commands_exactly():
    rows, _ = recorded_prefix(trace(), [0., 0.]); old = copy.deepcopy(rows)
    joined = join_recorded_prefix(rows, np.array([[.002, 0.], [.003, 0.]]), ['approach', 'insert'], [0., 0.], [1., 1.])
    assert joined[:3] == rows
    assert rows == old
    with pytest.raises(ValueError):
        join_recorded_prefix(rows, np.array([[.5, 0.]]), ['insert'], [0., 0.], [1., 1.])


def test_recorded_design_is_exact_deep_copy():
    original = dict(candidate_id='x', extra={'values': [1, 2]})
    result = matched_local_variant(original, None, 'recorded')
    assert result == original and result is not original
    result['extra']['values'].append(3)
    assert original['extra']['values'] == [1, 2]


def geometry_and_parameters():
    mf = pytest.importorskip('motion_family_search')
    from hook_seating_geometry import Capsule
    angles = np.linspace(3*np.pi/4, 5*np.pi/4, 17)
    rear = [Capsule(f'w{i}', .0275*np.array([np.cos(a), 0., np.sin(a)]),
                    .0275*np.array([np.cos(b), 0., np.sin(b)]), .001)
            for i, (a, b) in enumerate(zip(angles[:-1], angles[1:]))]
    geometry = dict(center=np.array([0., 0., .5]), radius=.014, heading=np.array([1., 0., 0.]),
                    targets=[Capsule('ped', np.array([0., 0., .515]), np.array([.01, .002, .535]), .001)],
                    rear_wires=rear, ring_radius=.0275)
    params = {k: (lo+hi)/2 for k, (lo, hi) in mf.RANGES.items()}
    params.update(azimuth_deg=20., elevation_deg=7., roll_deg=12., pitch_deg=-8., entry_twist_deg=30.,
                  target_selector=0., wire_selector=.5)
    return mf, geometry, params


@pytest.mark.parametrize('family', SHARED_FAMILIES)
@pytest.mark.parametrize('design', ['neutral', 'neutral_pitch15'])
def test_real_generator_local_variants_keep_approach_fixed(family, design):
    mf, geometry, values = geometry_and_parameters()
    original = mf.make_candidate(family, values, **geometry, candidate_id='control')
    before = copy.deepcopy(original)
    result = matched_local_variant(original, geometry, design)
    end = handoff_index(original)
    assert result['pose_waypoints'][:end+1] == original['pose_waypoints'][:end+1]
    assert result['search_parameters']['azimuth_deg'] == 20.
    assert result['search_parameters']['elevation_deg'] == 7.
    assert result['search_parameters']['roll_deg'] == 0.
    assert result['search_parameters']['entry_twist_deg'] == 0.
    assert result['search_parameters']['pitch_deg'] == (15. if design.endswith('15') else 0.)
    assert result['pose_waypoints'][end+1:] != original['pose_waypoints'][end+1:]
    assert original == before and result['training_eligible'] is False


def test_side_mouth_is_not_a_false_fixed_prefix_control():
    with pytest.raises(ValueError, match='excludes side_mouth'):
        matched_local_variant(dict(family='side_mouth'), None, 'neutral')


def group(scene, target, blocked, offset=0):
    digest = hashlib.sha256(str((scene, target, offset)).encode()).hexdigest()
    return dict(scene=scene, target=target, identical=True, all_four_blocked_in_prefix=blocked,
                parameters=dict(azimuth_deg=float(offset), elevation_deg=0.),
                members=[dict(family=f, candidate=f'c{offset}_{i}', command_sha256=digest, samples=2)
                         for i, f in enumerate(SHARED_FAMILIES)])


def test_selection_counts_shared_prefix_once_and_spreads_scenes():
    groups = [group(s, t, b, i) for i, (s, t, b) in enumerate([
        ('s0', 't0', True), ('s0', 't0', True), ('s0', 't1', True),
        ('s1', 't0', True), ('s2', 't0', True), ('s2', 't2', False), ('s3', 't1', False)])]
    chosen = select_evidence_groups(dict(groups=len(groups), rows=groups))
    assert len(chosen) == 6
    assert len({(r['scene'], r['target']) for r in chosen}) == 6
    assert sum(r['cohort'] == 'prefix_blocked' for r in chosen) == 4
    assert chosen[0]['scene'] != chosen[1]['scene']


def test_local_selection_prefers_recorded_seat_progress_not_first_row():
    g = group('s0', 't0', False)
    progress = {('s0', 't0', 'c0_1'): 3}
    selected = select_evidence_groups(dict(groups=1, rows=[g]), 0, 1, progress)
    assert selected[0]['family'] == 'flank_left'


@pytest.mark.parametrize('fault', ['count', 'member', 'hash', 'duplicate', 'not_identical'])
def test_invalid_dedup_evidence_is_rejected(fault):
    rows = [group('s0', 't0', True)]
    report = dict(groups=1, rows=rows)
    if fault == 'count': report['groups'] = 2
    elif fault == 'member': rows[0]['members'].pop()
    elif fault == 'hash': rows[0]['members'][0]['command_sha256'] = 'z'*64
    elif fault == 'duplicate': rows.append(copy.deepcopy(rows[0])); report['groups'] = 2
    elif fault == 'not_identical': rows[0]['identical'] = False
    with pytest.raises(ValueError): select_evidence_groups(report)


def test_recheck_uses_actual_prefix_not_report_flag(tmp_path, monkeypatch):
    folder = tmp_path/'candidates'/'x'; folder.mkdir(parents=True)
    (folder/'trace.json').write_text(json.dumps(trace()))
    monkeypatch.setitem(sys.modules, 'environment_preflight', SimpleNamespace(
        check_engine=lambda *args: dict(status='blocked', passed=False, complete=True)))
    monkeypatch.setattr(adapter, 'policy_for_trace', lambda policy, rows: policy)
    kin = SimpleNamespace(fk=lambda q: (np.array([q[0], 0., 0.]), Rotation.identity()))
    handoff = dict(ring_position_xyz=[.002, 0., 0.], orientation_xyzw=[0., 0., 0., 1.])
    rows, evidence = adapter.checked_original_prefix(tmp_path, 'x', [0., 0.], kin, handoff,
        JointSpace([[-1, -1], [1, 1]], [.01, .01]), SimpleNamespace(check=lambda q: None), None, None)
    assert rows is None and evidence['passed'] is False and evidence['reason'] == 'blocked'


def test_matched_prefix_actual_fk_mismatch_is_not_reused(tmp_path, monkeypatch):
    folder = tmp_path/'candidates'/'x'; folder.mkdir(parents=True)
    (folder/'trace.json').write_text(json.dumps(trace()))
    monkeypatch.setitem(sys.modules, 'environment_preflight', SimpleNamespace(check_engine=lambda *a: pytest.fail('should not audit')))
    kin = SimpleNamespace(fk=lambda q: (np.array([1., 0., 0.]), Rotation.identity()))
    rows, evidence = adapter.checked_original_prefix(tmp_path, 'x', [0., 0.], kin,
        dict(ring_position_xyz=[0., 0., 0.], orientation_xyzw=[0., 0., 0., 1.]), None, None, None, None)
    assert rows is None and evidence['reason'] == 'recorded_handoff_mismatch'


def make_campaign(tmp_path):
    campaign = tmp_path/'pilot'; campaign.mkdir()
    groups, cases = [], []
    for i, blocked in enumerate((True, False)):
        scene, target = f'scene_{i}', 'Tomato_01'
        g = group(scene, target, blocked)
        run = campaign/'cases'/scene/target/'run'
        run.mkdir(parents=True)
        (run/'manifest.json').write_text(json.dumps(dict(target=target)))
        (run/'planning_inputs.json').write_text(json.dumps(dict(inputs=dict(start=[0., 0.]))))
        (run.parent/'base_policy.json').write_text('{}')
        records = []
        for member in g['members']:
            path = run/'candidates'/member['candidate']; path.mkdir(parents=True)
            (path/'trace.json').write_text(json.dumps(trace()))
            _, evidence = recorded_prefix(trace(), [0., 0.])
            member.update(command_sha256=evidence['command_sha256'], samples=evidence['samples'])
            records.append(dict(candidate_id=member['candidate'], first_violation=dict(phases=['approach' if blocked else 'insert'])))
        (run.parent/'case_result.json').write_text(json.dumps(dict(records=records)))
        groups.append(g); cases.append(dict(scene_id=scene, target=target))
    (campaign/'campaign.json').write_text(json.dumps(dict(cases=cases)))
    report = tmp_path/'prefix.json'; report.write_text(json.dumps(dict(groups=len(groups), rows=groups)))
    return campaign, report


def test_batch_preparation_pins_actual_prefix_and_separates_two_bottlenecks(tmp_path):
    campaign, report = make_campaign(tmp_path)
    out = tmp_path/'output'
    result = batch.prepare(campaign, report, out, '/python', prefix_targets=1, local_targets=1)
    assert len(result['queries']) == 4
    assert [q['design'] for q in result['queries']] == ['recorded', 'recorded', 'neutral', 'neutral_pitch15']
    assert '--require-recorded-prefix' not in result['queries'][0]['command']
    assert all('--require-recorded-prefix' in q['command'] for q in result['queries'][1:])
    assert all(q['physics_executed'] is False and q['impossible'] is None for q in result['queries'])
    with pytest.raises(ValueError): batch.prepare(campaign, report, out, '/python')


def test_batch_refuses_stale_recorded_command_hash(tmp_path):
    campaign, report = make_campaign(tmp_path)
    doc = json.loads(report.read_text())
    for row in doc['rows']:
        for member in row['members']: member['command_sha256'] = 'a'*64
    report.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='differs from committed report'):
        batch.prepare(campaign, report, tmp_path/'out', '/python', prefix_targets=1, local_targets=1)


def test_batch_budget_skips_are_not_failures_or_physics(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, 'run_motion_family_search', SimpleNamespace(run_command=lambda *a: pytest.fail('budget forbids spawn')))
    query = dict(query_id='q0', cohort='local_blocked', scene='s', target='t', candidate='c', design='recorded')
    rows = batch.run_plans(dict(queries=[query]), tmp_path, batch_seconds=1.)
    assert rows[0]['status'] == 'not_evaluated_batch_budget' and rows[0]['physics_executed'] is False
    summary = json.loads((tmp_path/'recovery_plan_results.json').read_text())
    assert summary['evaluated_queries'] == 0 and summary['reported_queries'] == 1 and summary['budget_skipped'] == 1


def test_side_control_is_one_recorded_query_not_an_angle_sweep(tmp_path):
    campaign, report = make_campaign(tmp_path)
    side = tmp_path/'side'; side.mkdir()
    (side/'campaign.json').write_text(json.dumps(dict(cases=[dict(scene_id='s0', target='t0')])))
    folder = side/'cases/s0/t0'; (folder/'run/candidates/side3').mkdir(parents=True)
    (folder/'base_policy.json').write_text('{}')
    (folder/'case_result.json').write_text(json.dumps(dict(records=[dict(candidate_id='side3', family='side_mouth',
        reason=dict(reason='self_collision', phase='approach'))])))
    (folder/'run/candidates/side3/plan.json').write_text(json.dumps(dict(preflight=dict(reason='self_collision'))))
    result = batch.prepare(campaign, report, tmp_path/'out', '/python', prefix_targets=1, local_targets=1, side_campaign=side)
    assert len(result['queries']) == 5
    assert result['queries'][-1]['cohort'] == 'side_self_collision'
    assert '--require-recorded-prefix' not in result['queries'][-1]['command']
    assert result['queries'][-1]['design'] == 'recorded'
