import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from benchmark_newton import (arguments, assess_cpu_group, case_assessment, child_command, resource_blocker,
                              resource_summary, run_campaign, run_cpu_group, validate_args, write_json)


def completed_case(worlds=1):
    return dict(worlds=worlds, termination=None, returncode=0,
                summary=dict(status='completed', stop=None, worlds=worlds,
                active_contact_criterion_passed=True, finite_recorded_state=True,
                solver_capacity_valid=True, qacc_finite=True,
                actual_steps=500, requested_steps=500, input_sha256='input',
                commands_sha256='original', executed_commands_sha256='executed',
                engine=dict(engine='newton.solvers.SolverMuJoCo.step', world_count=worlds,
                            source_model_array_sha256='physical', source_fields_preserved=True,
                            physics_meshes_and_colliders_preserved=True),
                loop_wall_s=10., simulated_s=19.998, scheduled_simulated_s=20.,
                final_time_per_world_s=[19.998] * worlds, detached_per_world=[6] * worlds,
                max_active_penetration_m=.0008))


class BenchmarkNewtonTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.inputs = self.root / 'inputs'
        self.inputs.mkdir()
        for name in ('inputs.json', 'input_model.mjb', 'initial_state.npz', 'commands.npz'):
            (self.inputs / name).write_text('{}')
        self.args = arguments([str(self.inputs), '--output', str(self.root / 'output')])

    def tearDown(self):
        self.temporary.cleanup()

    def test_actual_commands_preserve_full_replay_and_same_settings(self):
        command = child_command(self.args, 8, self.root / 'case')
        self.assertIn('run_newton_harvest.py', command[2])
        self.assertEqual(command[command.index('--worlds') + 1], '8')
        self.assertEqual(command[command.index('--hz') + 1], '400.0')
        self.assertEqual(command[command.index('--free-hz') + 1], '60.0')
        self.assertNotIn('--seconds', command)
        self.assertNotIn('--gui', command)
        self.assertNotIn('--diagnostic-continue', command)
        self.assertNotIn('--no-graph', command)

    def test_python_virtualenv_symlink_path_is_preserved(self):
        base = self.root / 'base_python'
        base.write_text('base')
        virtual = self.root / 'virtualenv' / 'bin' / 'python'
        virtual.parent.mkdir(parents=True)
        virtual.symlink_to(base)
        args = arguments([str(self.inputs), '--output', str(self.root / 'output'), '--python', str(virtual)])
        self.assertEqual(args.python, virtual)
        self.assertEqual(child_command(args, 1, self.root / 'case')[0], str(virtual))

    def test_reject_overwrite_disordered_counts_and_nonfinite_budget(self):
        validate_args(self.args)
        for worlds in ([1, 1], [8, 1], [0]):
            with self.subTest(worlds=worlds), self.assertRaises(ValueError):
                args = copy.copy(self.args)
                args.worlds = worlds
                validate_args(args)
        args = copy.copy(self.args)
        args.timeout = float('nan')
        with self.assertRaises(ValueError):
            validate_args(args)
        self.args.output.mkdir()
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            validate_args(self.args)

    def test_valid_rate_uses_scheduled_duration_and_physics_loop_only(self):
        row = completed_case(8)
        row['process_total_wall_s'] = 1000.
        result = case_assessment(row)
        self.assertTrue(result['throughput_eligible'])
        self.assertEqual(result['measured_per_environment_realtime_factor'], 2.)
        self.assertEqual(result['measured_aggregate_sim_s_per_wall_s'], 16.)
        self.assertAlmostEqual(result['final_gpu_timestamp_minus_schedule_s'][0], -.002)

    def test_incomplete_invalid_and_capacity_cases_have_no_rate(self):
        alterations = [dict(status='stopped', stop='contact'), dict(actual_steps=499),
                       dict(finite_recorded_state=False), dict(solver_capacity_valid=False),
                       dict(qacc_finite=False), dict(max_active_penetration_m=.00101),
                       dict(loop_wall_s=float('nan')), dict(final_time_per_world_s=[float('nan')])]
        for changes in alterations:
            with self.subTest(changes=changes):
                row = completed_case()
                row['summary'].update(changes)
                result = case_assessment(row)
                self.assertFalse(result['throughput_eligible'])
                self.assertIsNone(result['measured_per_environment_realtime_factor'])
        row = completed_case()
        row['termination'] = 'case_wall_timeout'
        self.assertFalse(case_assessment(row)['throughput_eligible'])

    def test_copies_must_use_identical_inputs_commands_and_native_model(self):
        reference = case_assessment(completed_case())['input_identity']
        row = completed_case(8)
        self.assertTrue(case_assessment(row, reference)['throughput_eligible'])
        row['summary']['executed_commands_sha256'] = 'different'
        self.assertIn('executed_commands_sha256_mismatch', case_assessment(row, reference)['exclusion_reasons'])
        row = completed_case(8)
        row['summary']['engine']['source_model_array_sha256'] = 'different'
        self.assertIn('source_model_array_sha256_mismatch', case_assessment(row, reference)['exclusion_reasons'])

    def test_missing_flags_do_not_count_as_validity_and_release_counts_are_diagnostic(self):
        row = completed_case(8)
        row['summary']['detached_per_world'] = [0, 1, 2, 3, 4, 5, 6, 7]
        self.assertTrue(case_assessment(row)['throughput_eligible'])
        del row['summary']['qacc_finite']
        self.assertIn('qacc_finite_false_or_unknown', case_assessment(row)['exclusion_reasons'])

    def test_cpu_defaults_keep_existing_single_thread_and_stride_all_physics(self):
        args = arguments([str(self.inputs), '--output', str(self.root / 'cpu'), '--backend', 'cpu'])
        self.assertEqual(args.jobs, [1, 8, 24, 48])
        self.assertEqual(args.max_rss_mib, 102400.)
        command = child_command(args, 1, self.root / 'worker')
        self.assertEqual(command[command.index('--backend') + 1], 'cpu')
        self.assertEqual(command[command.index('--threads') + 1], '0')
        self.assertEqual(command[command.index('--record-stride') + 1], '5')
        self.assertEqual(args.setup_workers, 4)

    def test_concurrent_cpu_throughput_uses_union_clock_not_summed_rates(self):
        first, second = completed_case(), completed_case()
        first['summary'].update(loop_started_monotonic_s=100., loop_finished_monotonic_s=110., states_sha256='state')
        second['summary'].update(loop_started_monotonic_s=101., loop_finished_monotonic_s=113., loop_wall_s=12., states_sha256='state')
        row = dict(worlds=2, members=[first, second], gate_released_monotonic_s=99., termination=None)
        result = assess_cpu_group(row)
        self.assertTrue(result['throughput_eligible'])
        self.assertEqual(result['measured_concurrent_physics_window_s'], 13.)
        self.assertAlmostEqual(result['measured_aggregate_sim_s_per_wall_s'], 40./13.)
        self.assertNotAlmostEqual(result['measured_aggregate_sim_s_per_wall_s'], 2. + 20./12.)
        self.assertTrue(result['all_member_states_sha256_identical'])

    def test_cpu_group_without_gate_or_one_incomplete_member_has_no_rate(self):
        first, second = completed_case(), completed_case()
        first['summary'].update(loop_started_monotonic_s=100., loop_finished_monotonic_s=110.)
        second['summary'].update(loop_started_monotonic_s=100., loop_finished_monotonic_s=110.)
        row = dict(worlds=2, members=[first, second], termination=None)
        self.assertFalse(assess_cpu_group(row)['throughput_eligible'])
        row['gate_released_monotonic_s'] = 99.
        second['summary']['finite_recorded_state'] = False
        result = assess_cpu_group(row)
        self.assertFalse(result['throughput_eligible'])
        self.assertIn('worker_001:finite_recorded_state_false_or_unknown', result['exclusion_reasons'])
        self.assertIsNone(result['measured_aggregate_sim_s_per_wall_s'])

    def test_cpu_controller_bounds_setup_and_releases_owned_zero_step_workers(self):
        # This tiny stand-in uses only stdlib, no simulator or GPU. It exercises
        # the real process ownership/gate protocol without doing any physics.
        fake = self.root / 'stand_in.py'
        template = completed_case()['summary']
        fake.write_text("""import json, os, sys, time
from pathlib import Path
out = Path(sys.argv[1]); out.mkdir()
gate = Path(sys.argv[sys.argv.index('--start-gate') + 1])
ready = Path(sys.argv[sys.argv.index('--ready-file') + 1])
time.sleep(.03)
ready.write_text(json.dumps(dict(pid=os.getpid(), backend='cpu', worlds=1, physics_steps=0)))
while not gate.exists(): time.sleep(.005)
begin = time.monotonic(); time.sleep(.02); end = time.monotonic()
summary = json.loads(sys.argv[2])
summary.update(loop_started_monotonic_s=begin, loop_finished_monotonic_s=end,
               loop_wall_s=end-begin, states_sha256='same')
(out/'summary.json').write_text(json.dumps(summary))
""")
        args = arguments([str(self.inputs), '--output', str(self.root/'parallel'), '--backend', 'cpu',
                          '--setup-workers', '1', '--sample-interval', '.01', '--timeout', '10'])
        args.output.mkdir()
        observed = []
        def command(arguments, worlds, output):
            return [str(args.python), str(fake), str(output), json.dumps(template)]
        def progress(row):
            if row['status'] == 'initializing':
                observed.append(sum('ready' not in member for member in row['members']))
        with patch('benchmark_newton.child_command', side_effect=command), \
             patch('benchmark_newton.sample_gpu', return_value=dict(gpu_percent=0., gpu_memory_mib=0.)):
            row = run_cpu_group(args, 3, on_update=progress)
        self.assertEqual(max(observed), 1)
        self.assertIsNone(row['termination'])
        self.assertEqual(len(row['members']), 3)
        self.assertTrue(assess_cpu_group(row)['throughput_eligible'])
        self.assertTrue(all(member['ready']['physics_steps'] == 0 for member in row['members']))

    def test_resource_caps_are_explicit_and_global_gpu_scope(self):
        self.assertIsNone(resource_blocker(dict(rss_bytes=100, gpu_memory_mib=100), self.args))
        self.assertEqual(resource_blocker(dict(rss_bytes=65537 * 1024**2), self.args), 'process_tree_rss_limit')
        self.assertEqual(resource_blocker(dict(gpu_memory_mib=20001), self.args), 'global_gpu_memory_limit')
        summary = resource_summary([dict(cpu_percent=100., rss_bytes=1024),
                                    dict(cpu_percent=200., rss_bytes=2048, gpu_memory_mib=None)])
        self.assertEqual(summary['cpu_percent_mean'], 150.)
        self.assertEqual(summary['rss_bytes_max'], 2048)
        self.assertIsNone(summary['gpu_memory_mib_max'])

    def test_campaign_stops_before_larger_worlds_on_invalid_case(self):
        row = completed_case()
        row.update(name='gpu_worlds_001', summary=dict(row['summary'], status='stopped', stop='contact'))
        with patch('benchmark_newton.run_case', return_value=row) as run:
            report = run_campaign(self.args)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(report['status'], 'stopped')
        self.assertEqual(report['not_evaluated_world_counts'], [8, 32])
        self.assertIsNone(report['largest_valid_world_count'])
        self.assertFalse(report['training_eligible'])
        self.assertEqual(json.loads((self.args.output / 'summary.json').read_text())['status'], 'stopped')

    def test_nonfinite_failed_evidence_is_preserved_without_invalid_json(self):
        destination = self.root / 'failed.json'
        write_json(destination, dict(raw_summary=dict(error_value=float('nan'))))
        self.assertEqual(json.loads(destination.read_text())['raw_summary']['error_value'], 'nan')


if __name__ == '__main__':
    unittest.main()
