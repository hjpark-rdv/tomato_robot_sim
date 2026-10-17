"""Protect evidence ownership and incomplete-trial reporting in comparisons."""
import json
import csv
import hashlib
from collections import Counter
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

from benchmark_solver_compare import (compare_batch_evidence, execution_command,
    summarize_run, validate_cases, write_comparison_summary)


class SolverComparisonTests(unittest.TestCase):
    def test_batched_preset_uses_same_executor_without_single_world_hooks(self):
        args = SimpleNamespace(wrapper=Path('/example/run_s300_newton.sh'), python=Path('/example/python'),
                               lock_timeout=900, lock=Path('/tmp/example.lock'))
        case = dict(inputs='/example/inputs', preset='batched', worlds=4, repeats=1)
        command = execution_command(case, args, Path('/example/output'))
        self.assertTrue(any(value.endswith('run_newton_harvest.py') for value in command))
        for option in ('--dense-hessian', '--fixed-solver-reductions', '--indexed-qfrc', '--canonical-constraints'):
            self.assertNotIn(option, command)
        self.assertEqual(command[command.index('--worlds')+1], '4')
        self.assertEqual(command[command.index('--linear-algebra')+1], 'warp')
        self.assertIn('--all-world-trace', command)

    def test_manifest_owns_output_and_cannot_escape_case_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = root / 'inputs'
            inputs.mkdir()
            for name in ('inputs.json', 'input_model.mjb', 'commands.npz', 'initial_state.npz'):
                (inputs / name).touch()
            case = dict(name='baseline', inputs=str(inputs), comparison_class='identical_model')
            self.assertEqual(validate_cases(dict(cases=[case]), root/'output'), [case])
            for change in (dict(name='../escape'), dict(runner_options=['--output=elsewhere']),
                           dict(worlds=0), dict(repeats=True), dict(comparison_class='same_enough')):
                with self.subTest(change=change), self.assertRaises(ValueError):
                    validate_cases(dict(cases=[dict(case, **change)]), root/'output')
            with self.assertRaises(ValueError):
                validate_cases(dict(cases=[case]), inputs)

    def test_missing_summary_and_stopped_execution_never_get_throughput_credit(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            result = summarize_run(root, worlds=2)
            self.assertFalse(result['throughput_eligible'])
            self.assertIn('summary_missing_or_invalid', result['exclusion_reasons'])
            (root/'summary.json').write_text(json.dumps(dict(status='stopped', stop='penetration')))
            result = summarize_run(root, worlds=2)
            self.assertFalse(result['throughput_eligible'])
            self.assertIn('executor_not_completed_without_stop', result['exclusion_reasons'])
            self.assertFalse(result['physics_equivalence_confirmed'])

    def test_summary_csv_keeps_failed_runs_and_actual_phase_rates(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            run = root / 'failed_run'
            run.mkdir()
            (run/'summary.json').write_text(json.dumps(dict(status='stopped', stop='penetration',
                worlds=2, source_model_counts=dict(nv=190), phase_performance=dict(
                    close=dict(steps=4, simulated_s=.01, wall_s=.02, realtime_factor=.5)))))
            rows = write_comparison_summary([run], root/'summary')
            self.assertEqual(rows[0]['worlds'], 2)
            self.assertFalse(rows[0]['throughput_eligible'])
            with (root/'summary/phases.csv').open() as stream:
                phase = list(csv.DictReader(stream))[0]
            self.assertEqual(phase['realtime_factor'], '0.5')
            with self.assertRaises(ValueError):
                write_comparison_summary([run], root/'summary')

    def test_three_step_control_cannot_rank_as_complete_harvest(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/'summary.json').write_text(json.dumps(dict(status='completed', stop=None,
                actual_steps=3, requested_steps=3, profile_instrumented=False)))
            np.savez(root/'executed_commands.npz', time_edges=np.arange(13807),
                     phase=np.array(['settle']*13805+['hold']))
            np.savez(root/'states.npz', phase=np.array(['settle']*4))
            row = summarize_run(root, worlds=1)
            self.assertFalse(row['full_command_schedule_requested'])
            self.assertFalse(row['full_existing_sequence_completed'])
            self.assertFalse(row['clean_harvest_throughput_eligible'])
            self.assertEqual(row['expected_final_phase'], 'hold')
            self.assertEqual(row['recorded_final_phase'], 'settle')

    def test_all_world_comparison_detects_hidden_nonzero_world_change(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, worlds in [('reference', 1), ('candidate', 2)]:
                path = root/name
                path.mkdir()
                np.savez(path/'executed_commands.npz', time_edges=np.arange(3), phase=['settle', 'hold'])
                np.savez(path/'states.npz', phase=['settle', 'settle', 'hold'])
                np.savez(path/'all_world_states.npz', qpos=np.zeros((worlds, 3, 2)),
                    qvel=np.zeros((worlds, 3, 2)), eq_active=np.ones((worlds, 3, 1), bool),
                    time=np.tile([1., 2.], (worlds, 1)))
                from compare_newton_runs import METRIC_KEYS
                np.savez(path/'physics_metrics.npz', **{field:np.zeros((worlds, 2)) for field in METRIC_KEYS})
                summary = dict(status='completed', stop=None, worlds=worlds, actual_steps=2,
                    requested_steps=2, finite_recorded_state=True, qacc_finite=True,
                    solver_capacity_valid=True, active_contact_criterion_passed=True,
                    max_active_penetration_m=0.,
                    engine=dict(source_model_array_sha256='source'), input_sha256='input',
                    commands_sha256='commands', executed_commands_sha256='executed',
                    all_world_states_sha256=hashlib.sha256((path/'all_world_states.npz').read_bytes()).hexdigest())
                (path/'summary.json').write_text(json.dumps(summary))
                raw_events = [dict(world=world, name='fruit', solver_step=1) for world in range(worlds)]
                (path/'newton_break_events_all_worlds.json').write_text(json.dumps(raw_events))
                display_events = [dict(event, display_world=0, joint='/display/joint')
                                  if event['world'] == 0 else event for event in raw_events]
                (path/'break_events.json').write_text(json.dumps(display_events))
            # Reading an NPZ member decompresses it again. Full-world checks
            # must decode each state/metric once, independent of world count.
            with np.load(root/'reference/all_world_states.npz') as archive:
                archive_type = type(archive)
            original_getitem = archive_type.__getitem__
            reads = Counter()
            def counted_getitem(archive, key):
                filename = str(archive.zip.filename)
                if filename.endswith(('all_world_states.npz', 'physics_metrics.npz')):
                    reads[filename, key] += 1
                return original_getitem(archive, key)
            with patch.object(archive_type, '__getitem__', counted_getitem):
                result = compare_batch_evidence(root/'reference', root/'candidate')
            self.assertTrue(reads)
            self.assertTrue(all(count == 1 for count in reads.values()), reads)
            self.assertTrue(result['passed'])
            path = root/'candidate'
            with np.load(path/'all_world_states.npz') as archive:
                arrays = dict(archive)
            arrays['qvel'][1, 1, 0] = 1.
            np.savez(path/'all_world_states.npz', **arrays)
            # Keep the file hash valid: the changed physical record must still
            # fail even when no provenance/hash error is present.
            summary = json.loads((path/'summary.json').read_text())
            summary['all_world_states_sha256'] = hashlib.sha256((path/'all_world_states.npz').read_bytes()).hexdigest()
            (path/'summary.json').write_text(json.dumps(summary))
            result = compare_batch_evidence(root/'reference', root/'candidate')
            self.assertFalse(result['passed'])
            self.assertTrue(result['every_world_finite'])
            self.assertFalse(result['worlds_exactly_match_world0'])
            self.assertFalse(result['worlds'][1]['fields']['qvel']['exact_reference'])

    def test_explicit_belt_policy_requires_same_input_and_every_world_audit(self):
        """A valid belt-only band can pass while a hidden world/policy edit cannot."""
        from compare_newton_runs import METRIC_KEYS
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, worlds in [('reference', 1), ('candidate', 2)]:
                path = root/name
                path.mkdir()
                np.savez(path/'executed_commands.npz', time_edges=np.arange(3), phase=['close', 'hold'])
                np.savez(path/'states.npz', phase=['initial', 'close', 'hold'])
                np.savez(path/'all_world_states.npz', qpos=np.zeros((worlds, 3, 2)),
                    qvel=np.zeros((worlds, 3, 2)), eq_active=np.ones((worlds, 3, 1), bool),
                    time=np.tile([1., 2.], (worlds, 1)))
                np.savez(path/'physics_metrics.npz',
                    **{field:np.zeros((worlds, 2)) for field in METRIC_KEYS})
                config = dict(default_m=.001, fruit_close_m=.002, pedicel_close_m=.001)
                policy_input = json.dumps(config).encode()
                (path/'contact_policy_input.json').write_bytes(policy_input)
                depth = np.zeros((worlds, 2, 4))
                depth[:, 0, 1] = .0015  # Belt-fruit only; all other classes stay below 1mm.
                events = np.full(worlds, np.iinfo(np.uint64).max, np.uint64)
                np.savez(path/'contact_policy_metrics.npz', class_max_penetration_m=depth,
                    first_policy_event=events, allowed_pair_steps=np.tile([0, 1, 0], (worlds, 1)),
                    residual_allowed_pair_steps=np.zeros((worlds, 3), int))
                summary = dict(status='completed', stop=None, worlds=worlds, actual_steps=2,
                    requested_steps=2, finite_recorded_state=True, qacc_finite=True,
                    solver_capacity_valid=True, active_contact_criterion_passed=False,
                    max_active_penetration_m=.0015,
                    contact_policy_input_sha256=hashlib.sha256(policy_input).hexdigest(),
                    contact_policy=dict(raw_reference_1mm_preserved=True, physics_changed=False,
                        stopping_policy_changed=True, policy_passed=True, config=config,
                        exact_belt_geometries=[dict(name='belt', geom_id=1)], exact_other_pairs=[],
                        role_geom_counts=dict(fruit=1, pedicel=0)),
                    engine=dict(source_model_array_sha256='source'), input_sha256='input',
                    commands_sha256='commands', executed_commands_sha256='executed',
                    all_world_states_sha256=hashlib.sha256((path/'all_world_states.npz').read_bytes()).hexdigest())
                (path/'summary.json').write_text(json.dumps(summary))
                (path/'newton_break_events_all_worlds.json').write_text('[]')
            reference, candidate = root/'reference', root/'candidate'
            result = compare_batch_evidence(reference, candidate)
            self.assertTrue(result['passed'])
            self.assertFalse(result['contact_evaluation']['candidate']['reference_1mm_passed'])
            self.assertEqual(result['contact_evaluation']['candidate']['scope'], 'explicit_pair_phase_policy')
            self.assertTrue(result['contact_policy_metrics_exact_n1'])
            original = json.loads((candidate/'summary.json').read_text())
            changed = json.loads(json.dumps(original))
            changed['contact_policy']['config']['fruit_close_m'] = .003
            (candidate/'summary.json').write_text(json.dumps(changed))
            result = compare_batch_evidence(reference, candidate)
            self.assertFalse(result['passed'])
            self.assertFalse(result['identity']['contact_policy_config'])
            (candidate/'summary.json').write_text(json.dumps(original))
            (candidate/'contact_policy_input.json').write_text('{}')
            result = compare_batch_evidence(reference, candidate)
            self.assertFalse(result['passed'])
            self.assertFalse(result['physical_checks']['candidate']['policy_input_snapshot_hash'])
            (candidate/'contact_policy_input.json').write_bytes(policy_input)
            with np.load(candidate/'contact_policy_metrics.npz') as archive:
                arrays = dict(archive)
            arrays['allowed_pair_steps'][1, 1] += 1
            np.savez(candidate/'contact_policy_metrics.npz', **arrays)
            result = compare_batch_evidence(reference, candidate)
            self.assertFalse(result['passed'])
            self.assertFalse(result['worlds'][1]['contact_policy_metrics']['allowed_pair_steps']['exact_reference'])
            arrays['allowed_pair_steps'][1, 1] -= 1
            arrays['first_policy_event'][1] = 0
            np.savez(candidate/'contact_policy_metrics.npz', **arrays)
            result = compare_batch_evidence(reference, candidate)
            self.assertFalse(result['passed'])
            self.assertFalse(result['physical_checks']['candidate']['policy_metric_events_passed'])


if __name__ == '__main__':
    unittest.main()
