#!/usr/bin/env python3
"""Serial comparison controller for the existing Newton harvest executor.

This controller adds no simulator or physics policy. A JSON manifest specifies
prepared inputs and opt-in runner settings. The canonical GPU wrapper owns all
physics defaults; each case retains its complete invocation and original result.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import time

import psutil
import numpy as np

from benchmark_motion import _stop_child, sample_gpu, sample_tree
from benchmark_newton import case_assessment, contact_validity, resource_summary, sha256, write_json


ROOT = Path(__file__).resolve().parent


def validate_cases(manifest, output):
    if output.exists():
        raise ValueError('Refusing to overwrite comparison output: ' + str(output))
    cases = manifest.get('cases')
    if not isinstance(cases, list) or not cases:
        raise ValueError('cases must be a nonempty list')
    names = set()
    owned = {'--output', '--worlds', '--repeats', '--gui-repeat-index'}
    for case in cases:
        name = case.get('name')
        if not isinstance(name, str) or not name or name in ('.', '..') or Path(name).name != name or name in names:
            raise ValueError('Unique single-component case name required')
        names.add(name)
        inputs = Path(case['inputs'])
        for filename in ('inputs.json', 'input_model.mjb', 'commands.npz', 'initial_state.npz'):
            if not (inputs / filename).is_file():
                raise ValueError('Missing prepared input: ' + str(inputs / filename))
        options = case.get('runner_options', [])
        if not isinstance(options, list) or not all(isinstance(value, str) for value in options):
            raise ValueError('runner_options must contain strings')
        if any(value.split('=', 1)[0] in owned for value in options):
            raise ValueError('Controller owns output/world/repeat arguments')
        worlds, repeats = case.get('worlds', 1), case.get('repeats', 1)
        if type(worlds) is not int or worlds < 1 or type(repeats) is not int or repeats < 1:
            raise ValueError('Positive integer worlds/repeats required')
        if case.get('comparison_class') not in ('identical_model', 'changed_representation', 'connection_path'):
            raise ValueError('Explicit comparison_class required')
        if case.get('preset', 'canonical') not in ('canonical', 'batched', 'native'):
            raise ValueError('preset must be canonical, batched or native')
    return cases


def execution_command(case, args, output):
    """Reuse one executor; batch diagnostic avoids all N=1-only solver hooks."""
    worlds, repeats = case.get('worlds', 1), case.get('repeats', 1)
    if case.get('preset', 'canonical') == 'canonical':
        command = [str(args.wrapper), 'gpu', str(Path(case['inputs']).resolve())]
    elif case['preset'] == 'native':
        command = [str(args.python), '-u', str(ROOT / 'run_newton_harvest.py'),
                   str(Path(case['inputs']).resolve()), '--backend', 'gpu',
                   '--native-observables-only', '--native-control-callback',
                   '--hz', '400', '--free-hz', '60', '--nconmax', '128', '--njmax', '256']
    else:
        command = [str(args.python), '-u', str(ROOT / 'run_newton_harvest.py'),
                   str(Path(case['inputs']).resolve()), '--backend', 'gpu',
                   '--native-observables-only', '--native-control-callback',
                   '--linear-algebra', 'warp', '--inertia-linear-algebra', 'warp',
                   '--hz', '400', '--free-hz', '60', '--tree-sums', 'flat',
                   '--canonical-contacts', '--deterministic', 'RUN_TO_RUN',
                   '--deterministic-scope', 'solver', '--native-integer-atomics',
                   '--nconmax', '128', '--njmax', '256']
    command += ['--output', str(output), '--worlds', str(worlds), '--repeats', str(repeats)]
    if worlds > 1:
        command += ['--all-world-trace']
    command += case.get('runner_options', [])
    return ['flock', '-x', '-w', str(args.lock_timeout), str(args.lock), *command]


def summarize_run(path, *, worlds, returncode=0, termination=None):
    source = path / 'summary.json'
    row = dict(output=str(path), worlds=worlds, returncode=returncode, termination=termination)
    if source.is_file():
        row['summary'] = json.loads(source.read_text())
        row['summary_sha256'] = sha256(source)
    row.update(case_assessment(row))
    summary = row.get('summary', {})
    row['backend'] = summary.get('backend')
    row['phase_performance'] = summary.get('phase_performance')
    row['setup_wall_s'] = summary.get('setup_wall_s')
    row['loop_wall_s'] = summary.get('loop_wall_s')
    row['realtime_factor'] = summary.get('realtime_factor')
    instrumented = summary.get('profile_instrumented')
    if instrumented is None and summary.get('backend') == 'cpu':
        instrumented = summary.get('profile_timers_enabled')
    row['profile_instrumented'] = instrumented
    command_file = path / 'executed_commands.npz'
    full_steps = None
    expected_end_phase = None
    if command_file.is_file():
        with np.load(command_file, allow_pickle=False) as commands:
            if 'time_edges' in commands: full_steps = len(commands['time_edges']) - 1
            if 'phase' in commands and len(commands['phase']):
                expected_end_phase = str(commands['phase'][-1])
    row['full_command_schedule_requested'] = bool(full_steps and summary.get('requested_steps') == full_steps)
    actual_end_phase, state_samples = None, None
    if (path / 'states.npz').is_file():
        with np.load(path / 'states.npz', allow_pickle=False) as states:
            if 'phase' in states and len(states['phase']):
                actual_end_phase = str(states['phase'][-1])
                state_samples = len(states['phase'])
    row['full_existing_sequence_completed'] = bool(row['full_command_schedule_requested']
        and summary.get('actual_steps') == full_steps and summary.get('status') == 'completed'
        and summary.get('stop') is None and expected_end_phase is not None
        and actual_end_phase == expected_end_phase and state_samples == full_steps + 1)
    row['expected_final_phase'] = expected_end_phase
    row['recorded_final_phase'] = actual_end_phase
    row['clean_throughput_eligible'] = bool(row['throughput_eligible'] and instrumented is False
                                          and row['full_existing_sequence_completed'])
    row['clean_harvest_throughput_eligible'] = row['clean_throughput_eligible']
    row['max_active_penetration_m'] = summary.get('max_active_penetration_m')
    row['all_world_temporal_state_recorded'] = summary.get('all_world_temporal_state_recorded')
    row['all_world_states_sha256'] = summary.get('all_world_states_sha256')
    row['final_world_qpos_spread_abs'] = summary.get('final_world_qpos_spread_abs')
    row['physics_equivalence_confirmed'] = False
    row['model_fidelity_note'] = 'Validity of one run is distinct from equivalence to another representation.'
    # Keep the original full summary at its path without duplicating large trees.
    row.pop('summary', None)
    return row


def run_case(case, args):
    name, worlds, repeats = case['name'], case.get('worlds', 1), case.get('repeats', 1)
    output = args.output / name
    command = execution_command(case, args, output)
    row = dict(case=case, command=command, output=str(output), termination=None)
    samples, started = [], time.monotonic()
    environment = os.environ.copy()
    for field in ('PYTHONPATH', 'PYTHONHOME', 'VIRTUAL_ENV'):
        environment.pop(field, None)
    environment['PYTHONNOUSERSITE'] = '1'
    with (args.output / (name + '.log')).open('x') as stream:
        child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                 env=environment, start_new_session=True)
        process = psutil.Process(child.pid)
        tracked = {process.pid: process}
        try:
            while child.poll() is None:
                sample = dict(monotonic_s=time.monotonic(), **sample_tree(process, tracked), **sample_gpu())
                samples.append(sample)
                write_json(args.output / (name + '_resources.json'), dict(
                    scope='whole owned process tree including lock wait/setup/save; GPU is global device sample',
                    samples=samples, summary=resource_summary(samples)))
                if time.monotonic() - started > args.timeout:
                    row['termination'] = 'case_wall_timeout'
                    _stop_child(child)
                    break
                time.sleep(args.sample_interval)
        except BaseException:
            _stop_child(child)
            raise
        row['returncode'] = child.wait()
    row['controller_wall_s'] = time.monotonic() - started
    row['resources'] = resource_summary(samples)
    paths = [output] if repeats == 1 else [output / f'repeat_{index:03d}' for index in range(repeats)]
    row['trials'] = [summarize_run(path, worlds=worlds, returncode=row['returncode'],
                                 termination=row['termination']) for path in paths]
    write_json(args.output / (name + '_assessment.json'), row)
    return row


def write_comparison_summary(paths, output):
    """Collect preserved executor evidence; never rerun or alter any trial."""
    if output.exists():
        raise ValueError('Refusing to overwrite summary output: ' + str(output))
    output.mkdir(parents=True)
    runs, phases = [], []
    for path in paths:
        path = path.resolve()
        row = summarize_run(path, worlds=1)
        source = path / 'summary.json'
        summary = json.loads(source.read_text()) if source.is_file() else {}
        if summary.get('worlds', 1) != 1:
            row = summarize_run(path, worlds=summary['worlds'])
        row['name'] = path.parent.name + '/' + path.name if path.name.startswith('repeat_') else path.name
        for field in ('status', 'stop', 'requested_steps', 'actual_steps', 'source_model_counts',
                      'detached_per_world', 'max_contact_force_n', 'training_eligible', 'harvest_success'):
            row[field] = summary.get(field)
        row['worlds'] = summary.get('worlds', 1)
        row['nv'] = summary.get('source_model_counts', {}).get('nv')
        row['aggregate_sim_s_per_wall_s'] = summary.get('aggregate_sim_s_per_wall_s')
        resources = path / 'resources.json'
        if resources.is_file():
            samples = json.loads(resources.read_text())
            if isinstance(samples, list): row['sampled_resources'] = resource_summary(samples)
        for name, phase in (summary.get('phase_performance') or {}).items():
            phases.append(dict(run=row['name'], output=str(path), phase=name, worlds=row['worlds'],
                aggregate_sim_s_per_wall_s=row['worlds']*phase['realtime_factor'], **phase))
        runs.append(row)
    write_json(output / 'summary.json', dict(schema='s300-solver-comparison-summary-v1',
        training_eligible=False, harvest_success=None, runs=runs,
        interpretation='Per-run validity and rates only; changed representations require separate calibration and dynamic qualification.'))
    keys = ('name', 'output', 'backend', 'status', 'stop', 'worlds', 'nv', 'actual_steps', 'requested_steps',
            'setup_wall_s', 'loop_wall_s', 'realtime_factor', 'aggregate_sim_s_per_wall_s',
            'max_active_penetration_m', 'detached_per_world', 'throughput_eligible',
            'profile_instrumented', 'clean_throughput_eligible',
            'full_command_schedule_requested',
            'full_existing_sequence_completed', 'clean_harvest_throughput_eligible',
            'physics_equivalence_confirmed', 'all_world_temporal_state_recorded')
    with (output / 'runs.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(runs)
    with (output / 'phases.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=('run', 'output', 'phase', 'worlds', 'steps', 'simulated_s',
            'wall_s', 'realtime_factor', 'aggregate_sim_s_per_wall_s'), extrasaction='ignore')
        writer.writeheader()
        writer.writerows(phases)
    return runs


def compare_batch_evidence(reference, candidate):
    """Exact all-world controls for identical replicated initial states/commands."""
    from compare_newton_runs import METRIC_KEYS
    summaries = [json.loads((path/'summary.json').read_text()) for path in (reference, candidate)]
    count = summaries[1]['worlds']
    identity = {}
    for field in ('input_sha256', 'commands_sha256', 'executed_commands_sha256'):
        a, b = (summary.get(field) for summary in summaries)
        identity[field] = bool(a and a == b)
    a, b = (summary.get('engine', {}).get('source_model_array_sha256') for summary in summaries)
    identity['source_model_array_sha256'] = bool(a and a == b)
    policies = [summary.get('contact_policy') for summary in summaries]
    uses_policy = any(policy is not None for policy in policies)
    if uses_policy:
        a, b = (summary.get('contact_policy_input_sha256') for summary in summaries)
        identity['contact_policy_input_sha256'] = bool(a and a == b)
        for field in ('config', 'exact_belt_geometries', 'exact_other_pairs', 'role_geom_counts'):
            a, b = (policy.get(field) if isinstance(policy, dict) else None for policy in policies)
            identity['contact_policy_' + field] = bool(a is not None and a == b)
    checks = {}
    contact_evaluation = {}
    for label, path, summary in zip(('reference', 'candidate'), (reference, candidate), summaries):
        contact_evaluation[label] = contact_validity(summary)
        checks[label] = dict(completed=summary.get('status') == 'completed' and summary.get('stop') is None,
            full_steps=summary.get('actual_steps') == summary.get('requested_steps'),
            full_existing_sequence=summarize_run(path, worlds=summary['worlds'])['full_existing_sequence_completed'],
            state_file_hash=sha256(path/'all_world_states.npz') == summary.get('all_world_states_sha256'),
            finite=summary.get('finite_recorded_state') is True,
            qacc_finite=summary.get('qacc_finite') is True,
            capacity=summary.get('solver_capacity_valid') is True,
            active_physics=contact_evaluation[label]['passed'])
        if uses_policy:
            snapshot = path/'contact_policy_input.json'
            checks[label]['policy_input_snapshot_hash'] = bool(snapshot.is_file() and
                sha256(snapshot) == summary.get('contact_policy_input_sha256'))
    worlds, shapes = [], {}
    # NpzFile reopens/decompresses a member on every __getitem__. Cache each
    # array once; otherwise checking N worlds repeatedly decodes N-world data.
    with np.load(reference/'all_world_states.npz', allow_pickle=False) as archive:
        left = {field: archive[field] for field in ('qpos', 'qvel', 'eq_active', 'time')}
    with np.load(candidate/'all_world_states.npz', allow_pickle=False) as archive:
        right = {field: archive[field] for field in ('qpos', 'qvel', 'eq_active', 'time')}
    for field in ('qpos', 'qvel', 'eq_active', 'time'):
        a, b = left[field], right[field]
        shapes[field] = dict(reference=list(a.shape), candidate=list(b.shape),
            compatible=a.shape[0] == 1 and b.shape[0] == count and a.shape[1:] == b.shape[1:])
    for index in range(count):
        fields = {}
        for field in shapes:
            a, b = left[field], right[field]
            if not shapes[field]['compatible']:
                fields[field] = dict(compatible=False, finite=False, exact_reference=False, exact_world0=False)
                continue
            fields[field] = dict(compatible=True, finite=bool(np.isfinite(b[index]).all()),
                exact_reference=bool(np.array_equal(a[0], b[index])),
                exact_world0=bool(np.array_equal(b[0], b[index])),
                maximum_absolute_difference_from_reference=float(np.max(np.abs(
                    b[index].astype(np.float64)-a[0].astype(np.float64)), initial=0)))
        worlds.append(dict(world=index, fields=fields))
    with np.load(reference/'physics_metrics.npz', allow_pickle=False) as archive:
        left = {field: archive[field] for field in METRIC_KEYS if field in archive.files}
    with np.load(candidate/'physics_metrics.npz', allow_pickle=False) as archive:
        right = {field: archive[field] for field in METRIC_KEYS if field in archive.files}
    for world in worlds:
        metrics = {}
        for field in METRIC_KEYS:
            if field not in left or field not in right:
                metrics[field] = dict(compatible=False, finite=False, exact_reference=False)
                continue
            a, b = left[field], right[field]
            compatible = a.shape[0] == 1 and b.shape[0] == count and a.shape[1:] == b.shape[1:]
            metrics[field] = dict(compatible=compatible,
                finite=bool(compatible and np.isfinite(b[world['world']]).all()),
                exact_reference=bool(compatible and np.array_equal(a[0], b[world['world']])))
        world['physics_metrics'] = metrics
    policy_metrics_exact = True
    policy_metric_errors = []
    if uses_policy:
        archives = []
        for label, path, summary in zip(('reference', 'candidate'), (reference, candidate), summaries):
            try:
                with np.load(path/'contact_policy_metrics.npz', allow_pickle=False) as archive:
                    arrays = dict(archive)
            except (OSError, ValueError) as error:
                policy_metric_errors.append(f'{label}: {error}')
                arrays = {}
            archives.append(arrays)
            expected_shapes = dict(class_max_penetration_m=(summary['worlds'], summary['actual_steps'], 4),
                first_policy_event=(summary['worlds'],), allowed_pair_steps=(summary['worlds'], 3),
                residual_allowed_pair_steps=(summary['worlds'], 3))
            checks[label]['policy_metrics_full_coverage'] = all(
                field in arrays and arrays[field].shape == shape
                for field, shape in expected_shapes.items())
            events = arrays.get('first_policy_event')
            checks[label]['policy_metric_events_passed'] = bool(events is not None and
                events.shape == (summary['worlds'],) and
                np.all(events == np.iinfo(np.uint64).max))
        for world in worlds:
            metrics = {}
            for field in ('class_max_penetration_m', 'first_policy_event',
                          'allowed_pair_steps', 'residual_allowed_pair_steps'):
                a, b = (archive.get(field) for archive in archives)
                compatible = bool(a is not None and b is not None and a.ndim >= 1 and
                    b.ndim >= 1 and a.shape[0] == 1 and b.shape[0] == count and
                    a.shape[1:] == b.shape[1:])
                metrics[field] = dict(compatible=compatible,
                    finite=bool(compatible and np.isfinite(b[world['world']]).all()),
                    exact_reference=bool(compatible and np.array_equal(a[0], b[world['world']])))
            world['contact_policy_metrics'] = metrics
        policy_metrics_exact = bool(not policy_metric_errors and all(
            metric['finite'] and metric['exact_reference']
            for world in worlds for metric in world['contact_policy_metrics'].values()))
    event_paths = [path/('newton_break_events_all_worlds.json'
        if (path/'newton_break_events_all_worlds.json').is_file() else 'break_events.json')
        for path in (reference, candidate)]
    events = [json.loads(path.read_text()) for path in event_paths]
    reference_events = [{key:value for key,value in event.items() if key != 'world'} for event in events[0]]
    for world in worlds:
        own = [{key:value for key,value in event.items() if key != 'world'} for event in events[1]
               if event['world'] == world['world']]
        world['break_events_exact_reference'] = own == reference_events
    all_finite = all(field['finite'] for world in worlds for field in world['fields'].values())
    all_exact = all(field['exact_reference'] for world in worlds for field in world['fields'].values())
    same_within_batch = all(field['exact_world0'] for world in worlds for field in world['fields'].values())
    validity = all(value for group in checks.values() for value in group.values())
    metric_checks = all(metric['finite'] and metric['exact_reference']
        for world in worlds for metric in world['physics_metrics'].values())
    passed = bool(validity and all(identity.values()) and all_finite and all_exact
                  and metric_checks and policy_metrics_exact
                  and all(world['break_events_exact_reference'] for world in worlds))
    return dict(schema='s300-native-batch-comparison-v1', reference=str(reference), candidate=str(candidate),
        training_eligible=False, harvest_success=None, physical_fidelity_validated=False,
        passed=passed, identity=identity,
        physical_checks=checks, shapes=shapes, worlds=worlds, every_world_finite=all_finite,
        physical_event_sources=[str(path) for path in event_paths],
        all_worlds_exactly_match_n1=all_exact, worlds_exactly_match_world0=same_within_batch,
        physics_metrics_exact_n1=metric_checks,
        contact_evaluation=contact_evaluation, explicit_contact_policy_used=uses_policy,
        contact_policy_metrics_exact_n1=policy_metrics_exact,
        contact_policy_metric_errors=policy_metric_errors,
        interpretation='Exact replicated-state/command batch control. It does not qualify different scene inputs or arbitrary independent controls. A mismatch alone does not prove cross-world collision.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path, nargs='?')
    parser.add_argument('--summarize', type=Path, nargs='+', help='Read existing run directories and write runs.csv/phases.csv')
    parser.add_argument('--compare-worlds', type=Path, nargs=2, help='Compare complete N1 and batched all-world state/event evidence')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--wrapper', type=Path, default=ROOT.parent / 'run_s300_newton.sh')
    parser.add_argument('--python', type=Path, default=ROOT.parent / '.newton-venv/bin/python')
    parser.add_argument('--lock', type=Path, default=Path('/tmp/s300_solver_compare_gpu.lock'))
    parser.add_argument('--lock-timeout', type=float, default=900)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--sample-interval', type=float, default=2)
    args = parser.parse_args(argv)
    if args.compare_worlds:
        if args.manifest is not None or args.summarize:
            parser.error('Use --compare-worlds by itself')
        if args.output.exists(): parser.error('Refusing to overwrite comparison output')
        result = compare_batch_evidence(*args.compare_worlds)
        args.output.mkdir(parents=True)
        write_json(args.output/'comparison.json', result)
        return result
    if args.summarize:
        if args.manifest is not None: parser.error('Use a manifest or --summarize, not both')
        return write_comparison_summary(args.summarize, args.output)
    if args.manifest is None:
        parser.error('A case manifest or --summarize is required')
    if min(args.timeout, args.lock_timeout, args.sample_interval) <= 0:
        parser.error('Timeouts and sample interval must be positive')
    manifest = json.loads(args.manifest.read_text())
    cases = validate_cases(manifest, args.output)
    args.output.mkdir(parents=True)
    write_json(args.output / 'manifest.json', manifest)
    campaign = dict(schema='s300-solver-comparison-v1', training_eligible=False,
                    harvest_success=None, cases=[])
    for case in cases:
        print('Starting comparison case:', case['name'], flush=True)
        campaign['cases'].append(run_case(case, args))
        write_json(args.output / 'comparison.json', campaign)
        print('Finished comparison case:', case['name'], flush=True)
    return campaign


if __name__ == '__main__':
    main()
