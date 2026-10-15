#!/usr/bin/env python3
"""Bounded Newton GPU-world or synchronized CPU-process scaling comparison.

Each case uses the same saved inputs, complete commands and physics settings.
Process startup, CUDA compilation and evidence saving are reported separately
from executor physics-loop throughput. No Isaac or Newton runtime is imported
by this controller, and it never changes collision/contact policy.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import psutil

from benchmark_motion import _stop_child, owned_ready_process, sample_gpu, sample_tree


ROOT = Path(__file__).resolve().parent


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    def safe(item):
        if isinstance(item, float) and not math.isfinite(item):
            return repr(item)
        if isinstance(item, dict):
            return {key: safe(value) for key, value in item.items()}
        if isinstance(item, (tuple, list)):
            return [safe(value) for value in item]
        return item
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(safe(value), indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    temporary.replace(path)


def positive_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0


def validate_args(args):
    if not args.inputs.is_dir():
        raise ValueError('Prepared inputs directory is missing: ' + str(args.inputs))
    for name in ('inputs.json', 'input_model.mjb', 'initial_state.npz', 'commands.npz'):
        if not (args.inputs / name).is_file():
            raise ValueError('Prepared input is missing: ' + name)
    if args.output.exists():
        raise ValueError('Refusing to overwrite an existing campaign: ' + str(args.output))
    if not args.python.is_file():
        raise ValueError('Python executable is missing: ' + str(args.python))
    counts = args.jobs if args.backend == 'cpu' else args.worlds
    if not counts or any(type(value) is not int or value < 1 for value in counts):
        raise ValueError('World counts must be positive integers')
    if counts != sorted(set(counts)):
        raise ValueError('World counts must be unique and increasing')
    for name in ('hz', 'free_hz', 'timeout', 'max_rss_mib', 'max_gpu_mib', 'sample_interval'):
        if not positive_number(getattr(args, name)):
            raise ValueError(name + ' must be finite and positive')
    if args.free_hz > args.hz:
        raise ValueError('free_hz must not exceed contact hz')
    if args.seconds is not None and not positive_number(args.seconds):
        raise ValueError('seconds must be finite and positive')
    if args.batch < 2 or args.batch % 2:
        raise ValueError('batch must be a positive even number')
    if min(args.nconmax, args.njmax) < 1:
        raise ValueError('Contact and constraint capacities must be positive')
    if args.threads < 0 or args.record_stride < 1:
        raise ValueError('threads must be nonnegative and record_stride positive')
    if args.setup_workers < 1:
        raise ValueError('setup_workers must be positive')


def child_command(args, worlds, output):
    command = [str(args.python), '-u', str(ROOT / 'run_newton_harvest.py'),
               str(args.inputs), '--output', str(output), '--backend', args.backend,
               '--worlds', str(worlds), '--device', args.device,
               '--hz', str(args.hz), '--free-hz', str(args.free_hz),
               '--batch', str(args.batch), '--record-stride', str(args.record_stride),
               '--nconmax', str(args.nconmax), '--njmax', str(args.njmax)]
    if args.backend == 'cpu':
        command += ['--threads', str(args.threads)]
    if getattr(args, 'native_observables_only', False):
        command += ['--native-observables-only']
    if args.seconds is not None:
        command += ['--seconds', str(args.seconds)]
    return command


def resource_blocker(sample, args):
    rss = sample.get('rss_bytes')
    gpu = sample.get('gpu_memory_mib')
    if isinstance(rss, (int, float)) and rss > args.max_rss_mib * 1024 ** 2:
        return 'process_tree_rss_limit'
    if args.backend == 'gpu' and isinstance(gpu, (int, float)) and gpu > args.max_gpu_mib:
        return 'global_gpu_memory_limit'
    return None


def case_assessment(row, reference=None):
    """Keep incomplete/invalid cases, but exclude them from rate comparisons."""
    reasons = []
    summary = row.get('summary')
    if row.get('termination'):
        reasons.append(row['termination'])
    if row.get('returncode') != 0:
        reasons.append('executor_nonzero_exit')
    if not isinstance(summary, dict):
        return dict(throughput_eligible=False, exclusion_reasons=reasons + ['summary_missing_or_invalid'])
    if summary.get('status') != 'completed' or summary.get('stop') is not None:
        reasons.append('executor_not_completed_without_stop')
    for name in ('active_contact_criterion_passed', 'finite_recorded_state',
                 'solver_capacity_valid', 'qacc_finite'):
        if summary.get(name) is not True:
            reasons.append(name + '_false_or_unknown')
    if summary.get('worlds') != row['worlds']:
        reasons.append('world_count_mismatch')
    actual, requested = summary.get('actual_steps'), summary.get('requested_steps')
    if type(actual) is not int or actual <= 0 or actual != requested:
        reasons.append('requested_trace_not_completed')
    engine = summary.get('engine', {})
    if engine.get('engine') != 'newton.solvers.SolverMuJoCo.step':
        reasons.append('actual_newton_solver_not_confirmed')
    if engine.get('world_count') != row['worlds']:
        reasons.append('engine_world_count_mismatch')
    if engine.get('source_fields_preserved') is not True or engine.get('physics_meshes_and_colliders_preserved') is not True:
        reasons.append('source_physics_preservation_not_confirmed')
    identities = {name: summary.get(name) for name in
                  ('input_sha256', 'commands_sha256', 'executed_commands_sha256')}
    identities['source_model_array_sha256'] = engine.get('source_model_array_sha256')
    for name, value in identities.items():
        if not isinstance(value, str) or not value:
            reasons.append(name + '_missing')
        elif reference is not None and value != reference.get(name):
            reasons.append(name + '_mismatch')
    loop = summary.get('loop_wall_s')
    simulated = summary.get('scheduled_simulated_s', summary.get('simulated_s'))
    if not positive_number(loop) or not positive_number(simulated):
        reasons.append('physics_timing_missing_or_invalid')
    times = summary.get('final_time_per_world_s')
    if not isinstance(times, list) or len(times) != row['worlds'] or not all(positive_number(x) for x in times):
        reasons.append('final_world_times_missing_or_nonfinite')
        drift = None
    else:
        drift = [value - simulated for value in times] if positive_number(simulated) else None
    penetration = summary.get('max_active_penetration_m')
    if not isinstance(penetration, (int, float)) or not math.isfinite(penetration) or penetration > .001:
        reasons.append('active_penetration_invalid_or_above_existing_1mm')
    eligible = not reasons
    return dict(throughput_eligible=eligible, exclusion_reasons=reasons,
                input_identity=identities,
                scheduled_simulated_s_per_world=simulated,
                physics_loop_wall_s=loop,
                measured_per_environment_realtime_factor=simulated / loop if eligible else None,
                measured_aggregate_sim_s_per_wall_s=row['worlds'] * simulated / loop if eligible else None,
                final_gpu_timestamp_minus_schedule_s=drift,
                detached_per_world=summary.get('detached_per_world'),
                interpretation='Loop throughput only; float32 GPU clock drift is diagnostic. '
                               'Fruit release counts are reported, not treated as verified harvest success.')


def resource_summary(samples):
    result = dict(sample_count=len(samples))
    for key in ('cpu_percent', 'rss_bytes', 'gpu_percent', 'gpu_memory_mib'):
        values = [row[key] for row in samples if isinstance(row.get(key), (int, float)) and math.isfinite(row[key])]
        result[key + '_max'] = max(values) if values else None
        result[key + '_mean'] = sum(values) / len(values) if values else None
    return result


def assess_cpu_group(row, reference=None):
    """Measure concurrent CPU throughput from owned, synchronized loop clocks."""
    reasons = []
    members = row.get('members', [])
    if row.get('termination'):
        reasons.append(row['termination'])
    if len(members) != row['worlds']:
        reasons.append('process_count_incomplete')
    starts, ends, durations = [], [], []
    member_reference = reference
    for index, member in enumerate(members):
        member.update(case_assessment(member, member_reference))
        if not member['throughput_eligible']:
            reasons += [f'worker_{index:03d}:' + reason for reason in member['exclusion_reasons']]
        elif member_reference is None:
            member_reference = member['input_identity']
        summary = member.get('summary') or {}
        begin, end = summary.get('loop_started_monotonic_s'), summary.get('loop_finished_monotonic_s')
        gate = row.get('gate_released_monotonic_s')
        if not positive_number(begin) or not positive_number(end) or end <= begin:
            reasons.append(f'worker_{index:03d}:loop_clock_missing_or_invalid')
        elif not positive_number(gate) or begin < gate:
            reasons.append(f'worker_{index:03d}:loop_started_before_gate')
        else:
            starts.append(begin)
            ends.append(end)
            durations.append(member.get('scheduled_simulated_s_per_world'))
    valid_window = len(starts) == row['worlds'] and bool(starts)
    window = max(ends) - min(starts) if valid_window else None
    eligible = not reasons and valid_window
    total = sum(durations) if valid_window and all(positive_number(x) for x in durations) else None
    eligible = eligible and positive_number(total)
    state_hashes = [member.get('summary', {}).get('states_sha256') for member in members]
    return dict(throughput_eligible=bool(eligible), exclusion_reasons=reasons,
                input_identity=member_reference,
                measured_concurrent_physics_window_s=window,
                measured_loop_start_spread_s=max(starts)-min(starts) if valid_window else None,
                sum_simulated_s=total,
                measured_aggregate_sim_s_per_wall_s=total/window if eligible else None,
                measured_per_environment_realtime_factor=total/window/row['worlds'] if eligible else None,
                all_member_states_sha256_identical=bool(state_hashes) and all(isinstance(value, str) for value in state_hashes)
                                                    and len(set(state_hashes)) == 1,
                detached_per_world=[member.get('summary', {}).get('detached_per_world') for member in members],
                interpretation='Concurrent CPU rate is summed simulated duration / union of actual synchronized '
                               'physics-loop clocks. Individual rates are not summed. Startup and saving are excluded.')


def run_cpu_group(args, jobs, on_update=None):
    """Reuse the executor's zero-step gates; never create another physics loop."""
    name = f'cpu_processes_{jobs:03d}'
    row = dict(name=name, worlds=jobs, backend='cpu', status='launching', termination=None, members=[])
    gate = args.output / (name + '_gate.json')
    resources_path = args.output / (name + '_resources.jsonl')
    row['resources_file'] = str(resources_path)
    workers, samples = [], []
    started = time.perf_counter()

    def sample_group():
        now = time.perf_counter()
        readings = [sample_tree(worker['process'], worker['tracked']) for worker in workers]
        return dict(wall_s=now-started, cpu_percent=sum(value['cpu_percent'] for value in readings),
                    rss_bytes=sum(value['rss_bytes'] for value in readings), **sample_gpu())

    try:
        with resources_path.open('x') as resources:
            def monitor():
                sample = sample_group()
                samples.append(sample)
                resources.write(json.dumps(sample, allow_nan=False) + '\n')
                resources.flush()
                blocker = resource_blocker(sample, args)
                if time.perf_counter() - started > args.timeout:
                    blocker = 'case_wall_timeout'
                if on_update:
                    on_update(dict(row, current_resources=sample, launched_processes=len(workers)))
                return blocker

            env = os.environ.copy()
            for key in ('PYTHONPATH', 'PYTHONHOME', 'VIRTUAL_ENV'):
                env.pop(key, None)
            env['PYTHONNOUSERSITE'] = '1'
            def launch_worker(index):
                worker_name = name + f'_worker_{index:03d}'
                output = args.output / worker_name
                ready = args.output / (worker_name + '_ready.json')
                command = child_command(args, 1, output) + ['--start-gate', str(gate),
                          '--ready-file', str(ready), '--start-gate-timeout', str(args.timeout)]
                log = args.output / (worker_name + '.log')
                stream = log.open('x')
                try:
                    child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                             env=env, start_new_session=True)
                except BaseException:
                    stream.close()
                    raise
                process = psutil.Process(child.pid)
                member = dict(name=worker_name, worlds=1, backend='cpu', output=str(output),
                              command=command, pid=child.pid, log=str(log), termination=None)
                row['members'].append(member)
                workers.append(dict(child=child, process=process, tracked={process.pid: process},
                                    stream=stream, ready=ready, member=member))
            row['status'] = 'initializing'
            while not row['termination']:
                all_ready = len(workers) == jobs
                for worker in workers:
                    if worker['child'].poll() is not None:
                        row['termination'] = 'executor_exited_before_gate'
                        break
                    if 'ready' not in worker['member']:
                        if not worker['ready'].is_file():
                            all_ready = False
                            continue
                        try:
                            evidence = json.loads(worker['ready'].read_text())
                        except ValueError:
                            all_ready = False
                            continue
                        ownership = owned_ready_process(worker['process'], evidence.get('pid'))
                        if evidence.get('physics_steps') != 0 or evidence.get('backend') != 'cpu' or evidence.get('worlds') != 1:
                            raise ValueError('Executor readiness is not a zero-step CPU state')
                        worker['member']['ready'] = dict(evidence, ownership=ownership)
                if row['termination']:
                    break
                row['termination'] = monitor()
                if row['termination']:
                    break
                if all_ready and len(workers) == jobs:
                    row['gate_released_monotonic_s'] = time.monotonic()
                    write_json(gate, dict(released_monotonic_s=row['gate_released_monotonic_s']))
                    row['status'] = 'running'
                    print(f'[NEWTON BENCH] {name}: {jobs} owned CPU processes released', flush=True)
                    break
                # Loading the native MJB plus Newton metadata briefly uses more
                # RAM than a ready world. Bound that transient work separately
                # from the final concurrent physics group size.
                unready = sum('ready' not in worker['member'] for worker in workers)
                for _ in range(min(max(0, args.setup_workers-unready), jobs-len(workers))):
                    launch_worker(len(workers))
                    row['termination'] = monitor()
                    if row['termination']:
                        break
                time.sleep(min(.25, args.sample_interval))
            while not row['termination'] and any(worker['child'].poll() is None for worker in workers):
                row['termination'] = monitor()
                if row['termination']:
                    break
                time.sleep(args.sample_interval)
    except KeyboardInterrupt:
        row['termination'] = 'user_interrupt'
    except Exception as error:
        row.update(termination='controller_or_launch_error', error=repr(error))
    finally:
        if row['termination']:
            for worker in workers:
                if worker['child'].poll() is None:
                    try:
                        os.killpg(worker['child'].pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
            for worker in workers:
                _stop_child(worker['child'])
        for worker in workers:
            member = worker['member']
            member['returncode'] = worker['child'].wait()
            member['termination'] = row['termination']
            worker['stream'].close()
            path = Path(member['output']) / 'summary.json'
            if path.is_file():
                try:
                    member['summary'] = json.loads(path.read_text())
                    member['summary_sha256'] = sha256(path)
                except (OSError, ValueError) as error:
                    member['summary_error'] = repr(error)
    row.update(status='finished', process_total_wall_s=time.perf_counter()-started,
               resources=resource_summary(samples))
    return row


def run_case(args, worlds, on_update=None):
    name = f'gpu_worlds_{worlds:03d}'
    folder = args.output / name
    row = dict(name=name, worlds=worlds, output=str(folder), status='launching', termination=None)
    row['command'] = child_command(args, worlds, folder)
    row['global_gpu_before_launch'] = sample_gpu()
    initial_blocker = resource_blocker(row['global_gpu_before_launch'], args)
    if row['global_gpu_before_launch'].get('gpu_memory_mib') is None:
        initial_blocker = 'gpu_memory_monitor_unavailable'
    if initial_blocker:
        row.update(status='not_launched', termination=initial_blocker, returncode=None)
        row.update(case_assessment(row))
        return row
    env = os.environ.copy()
    for key in ('PYTHONPATH', 'PYTHONHOME', 'VIRTUAL_ENV'):
        env.pop(key, None)
    env['PYTHONNOUSERSITE'] = '1'
    row['log'] = str(args.output / (name + '.log'))
    row['resources_file'] = str(args.output / (name + '_resources.jsonl'))
    started = time.perf_counter()
    samples = []
    child = None
    try:
        with Path(row['log']).open('x') as stream, Path(row['resources_file']).open('x') as resources:
            child = subprocess.Popen(row['command'], stdout=stream, stderr=subprocess.STDOUT,
                                     env=env, start_new_session=True)
            row['pid'] = child.pid
            process = psutil.Process(child.pid)
            tracked = {child.pid: process}
            row['status'] = 'running'
            if on_update:
                on_update(row)
            next_sample = started
            print(f'[NEWTON BENCH] {name}: pid={child.pid}', flush=True)
            while child.poll() is None:
                now = time.perf_counter()
                if now - started > args.timeout:
                    row['termination'] = 'case_wall_timeout'
                    break
                if now >= next_sample:
                    sample = dict(wall_s=now-started, **sample_tree(process, tracked), **sample_gpu())
                    samples.append(sample)
                    resources.write(json.dumps(sample, allow_nan=False) + '\n')
                    resources.flush()
                    if on_update:
                        on_update(dict(row, current_resources=sample))
                    next_sample = now + args.sample_interval
                    row['termination'] = resource_blocker(sample, args)
                    if sample.get('gpu_memory_mib') is None:
                        row['termination'] = 'gpu_memory_monitor_unavailable'
                    if row['termination']:
                        break
                time.sleep(min(.25, args.sample_interval))
            if row['termination']:
                _stop_child(child)
            row['returncode'] = child.wait()
    except KeyboardInterrupt:
        row['termination'] = 'user_interrupt'
        if child is not None:
            _stop_child(child)
            row['returncode'] = child.returncode
    except Exception as error:
        row['termination'] = 'controller_or_launch_error'
        row['error'] = repr(error)
        if child is not None:
            _stop_child(child)
            row['returncode'] = child.returncode
    row['process_total_wall_s'] = time.perf_counter() - started
    row['resources'] = resource_summary(samples)
    summary_path = folder / 'summary.json'
    if summary_path.is_file():
        try:
            row['summary'] = json.loads(summary_path.read_text())
            row['summary_sha256'] = sha256(summary_path)
        except (OSError, ValueError) as error:
            row['summary_error'] = repr(error)
    row['status'] = 'finished'
    return row


def run_campaign(args):
    validate_args(args)
    args.output.mkdir(parents=True)
    counts = args.jobs if args.backend == 'cpu' else args.worlds
    report = dict(schema='s300-newton-world-scaling-v1', status='running',
                  backend=args.backend,
                  inputs=str(args.inputs), output=str(args.output),
                  requested_world_counts=counts, settings=vars(args).copy(), cases=[],
                  training_eligible=False, hook_success=None,
                  resource_scope='CPU/RSS: owned process tree. GPU memory/utilization: entire GPU, including other processes.',
                  measurement='Same complete saved command timeline by default. Actual executor physics-loop wall time excludes '
                              'startup, compilation and final evidence saving; process_total_wall_s includes these costs.',
                  limitations=['Replicated worlds replay identical commands; this measures execution throughput, not policy diversity.',
                               'Newton source-MJCF adaptation and existing break/contact hypotheses remain unchanged.',
                               'No throughput claim is made for stopped, incomplete, invalid or resource-limited cases.'])
    report['settings'] = {key: str(value) if isinstance(value, Path) else value for key, value in report['settings'].items()}
    report['code_sha256'] = {path.name: sha256(path) for path in sorted(ROOT.glob('newton_*.py'))}
    for filename in ('benchmark_newton.py', 'benchmark_motion.py', 'run_newton_harvest.py'):
        report['code_sha256'][filename] = sha256(ROOT / filename)
    report['inputs_json_sha256'] = sha256(args.inputs / 'inputs.json')
    destination = args.output / 'summary.json'
    write_json(destination, report)
    reference = None
    try:
        for worlds in counts:
            def update(active):
                report['active_case'] = active
                write_json(destination, report)
            row = (run_cpu_group(args, worlds, on_update=update) if args.backend == 'cpu'
                   else run_case(args, worlds, on_update=update))
            report.pop('active_case', None)
            row.update(assess_cpu_group(row, reference) if args.backend == 'cpu'
                       else case_assessment(row, reference))
            if row['throughput_eligible'] and reference is None:
                reference = row['input_identity']
            report['cases'].append(row)
            write_json(destination, report)
            print(json.dumps({key: row.get(key) for key in ('name', 'throughput_eligible', 'exclusion_reasons',
                  'measured_per_environment_realtime_factor', 'measured_aggregate_sim_s_per_wall_s')}, indent=2), flush=True)
            if not row['throughput_eligible']:
                report['status'] = 'stopped'
                report['stop_reason'] = row.get('termination') or 'case_failed_validity_or_identity_check'
                break
        else:
            report['status'] = 'completed'
    except KeyboardInterrupt:
        report.update(status='interrupted', stop_reason='user_interrupt')
    except Exception as error:
        report.update(status='error', error=repr(error))
    report['not_evaluated_world_counts'] = counts[len(report['cases']):]
    valid = [row for row in report['cases'] if row['throughput_eligible']]
    report['largest_valid_world_count'] = max((row['worlds'] for row in valid), default=None)
    report['best_aggregate_throughput_world_count'] = max(valid, key=lambda row: row['measured_aggregate_sim_s_per_wall_s'])['worlds'] if valid else None
    write_json(destination, report)
    return report


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', type=lambda value: Path(value).resolve())
    parser.add_argument('--output', type=lambda value: Path(value).resolve(), required=True)
    # A virtualenv interpreter is normally a symlink. Resolving that link would
    # launch its base interpreter and silently lose the installed Newton stack.
    parser.add_argument('--python', type=lambda value: Path(value).absolute(), default=Path(sys.executable).absolute())
    parser.add_argument('--backend', choices=('gpu', 'cpu'), default='gpu')
    parser.add_argument('--worlds', type=int, nargs='+', default=[1, 8, 32])
    parser.add_argument('--jobs', type=int, nargs='+', default=[1, 8, 24, 48], help='Sequential CPU group sizes, one native Newton world per process')
    parser.add_argument('--threads', type=int, default=0, help='Native CPU workers inside each process; 0 preserves the single-thread baseline')
    parser.add_argument('--setup-workers', type=int, default=4, help='Maximum CPU processes loading native assets at once; ready worlds wait at the zero-step gate')
    parser.add_argument('--record-stride', type=int, help='State save interval; physics/contact auditing still runs every step (GPU default1, CPU default5)')
    parser.add_argument('--native-observables-only',action='store_true')
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--hz', type=float, default=400.)
    parser.add_argument('--free-hz', type=float, default=60.)
    parser.add_argument('--seconds', type=float, help='Optional explicit prefix comparison; default executes the complete saved plan')
    parser.add_argument('--batch', type=int, default=10)
    parser.add_argument('--nconmax', type=int, default=512)
    parser.add_argument('--njmax', type=int, default=2048)
    parser.add_argument('--timeout', type=float, default=900., help='Whole process wall-time limit for each world-count case')
    parser.add_argument('--max-rss-mib', type=float, help='Sum process-tree RSS cap (GPU default64GiB, CPU default100GiB)')
    parser.add_argument('--max-gpu-mib', type=float, default=20000., help='Global used GPU memory cap, not only this process')
    parser.add_argument('--sample-interval', type=float, default=1.)
    args = parser.parse_args(argv)
    if args.max_rss_mib is None:
        args.max_rss_mib = 102400. if args.backend == 'cpu' else 65536.
    if args.record_stride is None:
        args.record_stride = 5 if args.backend == 'cpu' else 1
    return args


if __name__ == '__main__':
    result = run_campaign(arguments())
    print(str(Path(result['output']) / 'summary.json'), flush=True)
    raise SystemExit(0 if result['status'] == 'completed' else 1)
