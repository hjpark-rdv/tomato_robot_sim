"""One-bed orchestration contracts without plans, simulators or GPU physics."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import run_newton_row_harvest as route
from row_harvest_targets import select_one_bed


class RowOrchestrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name); self.common = self.root / 'inputs_route'; self.common.mkdir()
        proposals = [dict(plant_id=7, truss_index=1, station_y_m=.2),
                     dict(plant_id=9, truss_index=0, station_y_m=.8)]
        self.inventory = dict(trusses=[dict(**row, gutter_center_x=.7, rail_near=True,
            root_position_world_m=[.6, row['station_y_m'], .3],
            fruit_centroid_world_m=[.5, row['station_y_m'], .9]) for row in proposals])
        self.inventory_path = self.root / 'inventory.json'; route.write(self.inventory_path, self.inventory)
        self.selected = select_one_bed(proposals, self.inventory, .7)
        self.selected.update(source_inventory=str(self.inventory_path),
                             source_inventory_sha256=route.sha256(self.inventory_path))
        route.write(self.root / 'one_bed_selection.json', self.selected)
        self.meta = dict(source_assembly=str(self.root / 'display'), windows=[], attachments=[], fruit_specs=[])
        display = self.root / 'display'; display.mkdir()
        route.write(display / 'motion_manifest.json', dict(rail=dict(initial_station_y_m=.2)))
        for i, row in enumerate(proposals):
            plant, truss = row['plant_id'], row['truss_index']
            assembly = self.root / f'assembly{plant}'; assembly.mkdir()
            (assembly / 'composed.usdc').write_bytes(b'unchanged source USD')
            root = '/World/ElasticTarget'
            fruits = [dict(name=f'Truss_{truss:02d}_Fruit_{j:02d}', path=f'{root}/Truss_{truss:02d}_Fruit_{j:02d}',
                           joint=f'{root}/fruit_joint{j}') for j in range(1, 4)]
            manifest = dict(plant=dict(source_plant=f'/World/Plants/Plant_{plant:02d}', root=root, fruits=fruits))
            route.write(assembly / 'motion_manifest.json', manifest)
            plan = dict(rail_destination=dict(station_y_m=row['station_y_m']), grasp_depth_m=.035,
                grasp_height_offset_m=0., approach_direction_world=[1., 0., 0.], initial_command=[0.] * 10)
            route.write(assembly / 'plan.json', plan)
            route.write(assembly / 'truss_sequence.json', dict(cycle_plans=[dict(fruits=[f['name'] for f in fruits])]))
            policy = dict(schema='s300-harvest-group-policy-v1', minimum_fruits=3,
                source_sha256={name: route.sha256(assembly / name) for name in ('motion_manifest.json', 'composed.usdc')},
                fruit_eligibility=[dict(**f, eligible=True) for f in fruits])
            route.write(self.root / f'policy_plant{plant}.json', policy)
            prefix = '' if i == 0 else f'P{plant}_'
            self.meta['windows'].append(dict(id=str(plant), prefix=prefix, source_assembly=str(assembly),
                source_trial=str(assembly), source_manifest=str(assembly / 'motion_manifest.json'),
                source_plan=str(assembly / 'plan.json'), source_plan_sha256=route.sha256(assembly / 'plan.json'),
                fruit_specs=fruits, display_root=f'/World/Plant{plant}'))
            self.meta['attachments'].extend(dict(name=prefix + f['name'], anchor='stem') for f in fruits)
        (self.common / 'input_model.mjb').write_bytes(b'immutable common physical model')
        np.savez_compressed(self.common / 'initial_state.npz', qpos=np.zeros(10), robot_qpos_addresses=np.arange(10))
        route.write(self.common / 'sampled_plan.json', plan)
        self.save_meta()

    def save_meta(self):
        route.write(self.common / 'source_export.json', self.meta)
        names = ('source_export.json', 'input_model.mjb', 'initial_state.npz', 'sampled_plan.json')
        route.write(self.common / 'inputs.json', dict(input_files={name: dict(sha256=route.sha256(self.common / name)) for name in names}))

    def test_rechecks_generic_one_bed_ids_without_hardcoding_and_preserves_order(self):
        _, _, meta, selected = route.validate_prepared(self.root)
        self.assertEqual([s['plant_id'] for s in selected['stations']], [7, 9])
        self.assertEqual([w['id'] for w in meta['windows']], ['7', '9'])
        self.assertFalse(selected['collision_scope_changed'])

    def test_window_swap_or_wrong_source_station_is_rejected_even_when_input_hashes_match(self):
        self.meta['windows'].reverse(); self.save_meta()
        with self.assertRaisesRegex(ValueError, 'order or plant'): route.validate_prepared(self.root)
        self.meta['windows'].reverse(); self.save_meta()
        self.selected['stations'][1]['station_y_m'] = .9
        route.write(self.root / 'one_bed_selection.json', self.selected)
        with self.assertRaisesRegex(ValueError, 'rail destination'): route.validate_prepared(self.root)

    def test_inventory_change_and_model_tampering_are_detected(self):
        (self.common / 'input_model.mjb').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'input SHA'): route.validate_prepared(self.root)
        self.save_meta()
        self.inventory['trusses'][0]['gutter_center_x'] = -.7
        route.write(self.inventory_path, self.inventory)
        with self.assertRaisesRegex(ValueError, 'inventory SHA'): route.validate_prepared(self.root)

    def test_fresh_campaign_never_reuses_an_existing_directory(self):
        (self.root / 'existing').mkdir()
        with patch.object(route, 'prepare_segment') as prepare:
            with self.assertRaises(FileExistsError):
                route.main(['--prepared-root', str(self.root), '--campaign', 'existing'])
        prepare.assert_not_called()
        for name in ('../overwrite', '.', 'a/b'):
            with self.assertRaises(ValueError): route.safe_name(name)

    def test_index0_depth_change_and_missing_predecessor_are_rejected_before_work(self):
        with self.assertRaises(SystemExit):
            route.arguments(['--prepared-root', str(self.root), '--depth-offset-m', '-.01'])
        with self.assertRaises(SystemExit):
            route.arguments(['--prepared-root', str(self.root), '--start-index', '1'])

    def test_depth_offset_changes_group_footprint_and_measured_plan_request_together(self):
        args = route.arguments(['--prepared-root', str(self.root), '--start-index', '1',
            '--parent', str(self.root / 'parent'), '--depth-offset-m', '-.01'])
        output = self.root / 'segment'; output.mkdir()
        restored = dict(qpos=np.arange(10, dtype=float) / 100, robot_qpos_addresses=np.arange(10))
        state = dict(eq_active=np.ones((1, 6), bool), attachment_equality_ids=np.arange(6))
        checked = (restored, state, [], dict(parent_run='measured'), np.zeros((1, 7)), ['/World/test'])
        observed = {}
        class Selector:
            def __init__(self, policy, direction, **options): observed['selector_depth'] = options['depth_m']
            def __call__(self, *args, **kwargs): return dict(fruits=['Truss_00_Fruit_01'], aim_world_m=[.5, .8, .9])
        def subprocess_fake(command, **kwargs):
            if '--source-plan' in command:
                observed['request'] = route.read(output / 'request.json')
                route.write(output / 'plan.json', dict(initial_command=restored['qpos'].tolist()))
            else:
                inputs = output / 'inputs'
                np.savez_compressed(inputs / 'commands.npz', time=np.array([0.]))
                route.write(inputs / 'timeline.json', dict(physical_steps=0))
        before = {p: p.read_bytes() for p in self.common.iterdir() if p.is_file()}
        with patch.object(route, 'checked_parent', return_value=checked), \
             patch.object(route, 'HarvestGroupSelector', Selector), \
             patch.object(route.subprocess, 'run', side_effect=subprocess_fake):
            inputs = route.prepare_segment(1, self.root / 'parent', output, self.common, self.root, self.selected, args)
        self.assertAlmostEqual(observed['selector_depth'], .025)
        self.assertAlmostEqual(observed['request']['depth_m'], .025)
        self.assertEqual(observed['request']['measured_joint_q'], restored['qpos'][1:8].tolist())
        self.assertAlmostEqual(observed['request']['measured_station_y_m'], .2)
        self.assertEqual(route.read(inputs / 'source_export.json')['same_bed_current_target']['gutter_center_x'], .7)
        self.assertEqual(route.sha256(inputs / 'input_model.mjb'), route.sha256(self.common / 'input_model.mjb'))
        for path, value in before.items(): self.assertEqual(path.read_bytes(), value)

    def test_solver_tolerance_is_optional_and_forwarded_to_the_existing_physics_runner(self):
        for index, value in enumerate((None, '1e-8')):
            seen = {}
            def fake_run(command, **options):
                seen['command'] = command
                seen['python'] = options['env']['S300_NEWTON_PYTHON']
                out = Path(command[command.index('--output') + 1]); out.mkdir()
                route.write(out / 'summary.json', dict(status='stopped', stop='bounded-test',
                    simulated_s=1., realtime_factor=1., breaks=[]))
            argv = ['--prepared-root', str(self.root), '--campaign', f'case{index}', '--end-index', '1']
            if value is not None: argv += ['--solver-tolerance', value]
            with patch.object(route, 'prepare_segment', return_value=self.common), \
                 patch.object(route.subprocess, 'run', side_effect=fake_run):
                route.main(argv)
            command = seen['command']
            if value is None:
                self.assertNotIn('--solver-tolerance', command)
            else:
                self.assertEqual(float(command[command.index('--solver-tolerance') + 1]), 1e-8)
            self.assertEqual(seen['python'], str(route.REPO / 'nvidia-sim/.newton-venv/bin/python'))

    def test_shared_mesh_directory_keeps_mutable_timeline_and_commands_private(self):
        meshes = self.common / 'neighbor_meshes'; meshes.mkdir()
        (meshes / 'a.obj').write_bytes(b'mesh a')
        (meshes / 'b.obj').write_bytes(b'mesh b')
        route.write(self.common / 'timeline.json', dict(original=True))
        np.savez_compressed(self.common / 'commands.npz', time=np.array([123.]))
        proof = route.read(self.common / 'inputs.json')
        for name in ('neighbor_meshes/a.obj', 'neighbor_meshes/b.obj', 'timeline.json', 'commands.npz'):
            proof['input_files'][name] = dict(sha256=route.sha256(self.common / name))
        route.write(self.common / 'inputs.json', proof)
        route.validate_prepared(self.root)
        before = {name: (self.common / name).read_bytes() for name in proof['input_files']}
        output = self.root / 'shared_assets_segment'; output.mkdir()
        args = route.arguments(['--prepared-root', str(self.root)])

        def sample(command, **kwargs):
            inputs = Path(command[command.index('--sample-inputs') + 1])
            np.savez_compressed(inputs / 'commands.npz', time=np.array([0.]))
            route.write(inputs / 'timeline.json', dict(original=False))

        with patch.object(route.subprocess, 'run', side_effect=sample):
            inputs = route.prepare_segment(0, None, output, self.common, self.root, self.selected, args)
        self.assertTrue((inputs / 'neighbor_meshes').is_symlink())
        self.assertEqual((inputs / 'neighbor_meshes').resolve(), meshes)
        for name in ('timeline.json', 'commands.npz', 'source_export.json', 'sampled_plan.json'):
            self.assertFalse((inputs / name).is_symlink())
        self.assertEqual(route.read(inputs / 'timeline.json'), dict(original=False))
        for name, content in before.items():
            self.assertEqual((self.common / name).read_bytes(), content)
        prepared = route.read(inputs / 'inputs.json')
        self.assertEqual(set(prepared['input_files']), set(proof['input_files']))
        for name, evidence in prepared['input_files'].items():
            self.assertEqual(evidence['sha256'], route.sha256(inputs / name))

    def test_released_gutter_exception_requires_explicit_opt_in_and_is_forwarded(self):
        for index, (flags, enabled) in enumerate((([], False),
                (['--released-gutter-diagnostic'], True),
                (['--no-released-gutter-diagnostic'], False))):
            argv = ['--prepared-root', str(self.root), '--campaign', f'gutter_case{index}',
                    '--end-index', '1', *flags]
            self.assertIs(route.arguments(argv).released_gutter_diagnostic, enabled)
            seen = {}
            def fake_run(command, **options):
                seen['command'] = command
                out = Path(command[command.index('--output') + 1]); out.mkdir()
                route.write(out / 'summary.json', dict(status='stopped', stop='bounded-test',
                    simulated_s=1., realtime_factor=1., breaks=[]))
            with patch.object(route, 'prepare_segment', return_value=self.common), \
                 patch.object(route.subprocess, 'run', side_effect=fake_run):
                route.main(argv)
            self.assertEqual('--released-gutter-diagnostic' in seen['command'], enabled)

    def test_belt_hold_rate_is_forwarded_to_physics_and_sampling(self):
        argv = ['--prepared-root', str(self.root), '--campaign', 'belt_hold',
                '--end-index', '1', '--belt-hold-hz', '160']
        args = route.arguments(argv)
        self.assertIsNone(route.arguments(['--prepared-root', str(self.root)]).belt_hold_hz)
        output = self.root / 'sample_segment'; output.mkdir()
        def sampled(command, **options):
            inputs = Path(command[command.index('--sample-inputs') + 1])
            np.savez_compressed(inputs / 'commands.npz', time=np.array([0.]))
            route.write(inputs / 'timeline.json', dict(belt_hold_frequency=dict(requested_hz=160.)))
        with patch.object(route.subprocess, 'run', side_effect=sampled) as sample:
            route.prepare_segment(0, None, output, self.common, self.root, self.selected, args)
        command = sample.call_args.args[0]
        self.assertEqual(command[command.index('--belt-hold-hz') + 1], '160.0')
        seen = {}
        def fake_run(command, **options):
            seen['command'] = command
            out = Path(command[command.index('--output') + 1]); out.mkdir()
            route.write(out / 'summary.json', dict(status='stopped', stop='bounded-test',
                simulated_s=1., realtime_factor=1., breaks=[]))
        with patch.object(route, 'prepare_segment', return_value=self.common), \
             patch.object(route.subprocess, 'run', side_effect=fake_run):
            route.main(argv)
        command = seen['command']
        self.assertEqual(command[command.index('--belt-hold-hz') + 1], '160.0')

    def test_belt_hold_invalid_rates_are_rejected_by_row_cli(self):
        for rate in ('0', '-1', 'nan', 'inf', '221'):
            with self.subTest(rate=rate), self.assertRaises(SystemExit):
                route.arguments(['--prepared-root', str(self.root), '--belt-hold-hz', rate])


if __name__ == '__main__':
    unittest.main()
