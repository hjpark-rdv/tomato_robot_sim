#!/usr/bin/env python3
"""Run prepared targets on one bed using existing planners and Newton physics.

This only coordinates the existing measured-state group selector, plan adapter,
command sampler and GPU runner. Each segment preserves its actual predecessor;
the solver instance/contact caches restart at that documented boundary.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
ADAPTER = REPO / 'mujoco-benchmark/scripts/s300_smoke'
if str(ADAPTER) not in sys.path:
    sys.path.insert(0, str(ADAPTER))

from newton_inputs import sha256
from newton_continuation import load_resume
from row_harvest_targets import select_one_bed, truss_key
from harvest_group_policy import HarvestGroupSelector, load_policy
from newton_gpu_options import solver_tolerance_value
from newton_prepared_assets import verify_manifest_files, link_manifest_assets


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def safe_name(value):
    if not value or value in ('.', '..') or Path(value).name != value or '/' in value or '\\' in value:
        raise ValueError('Use a new single directory name for --campaign')
    return value


def input_file(common, name):
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('Prepared input filenames must be relative without traversal')
    return common / relative


def validate_prepared(root):
    """Recheck authoritative placement/order and every existing input hash."""
    root = Path(root).resolve(strict=True)
    common = root / 'inputs_route'
    proof, meta = read(common / 'inputs.json'), read(common / 'source_export.json')
    try:
        verify_manifest_files(common, proof['input_files'])
    except ValueError as error:
        raise ValueError('Prepared input SHA/path verification failed: ' + str(error)) from error
    if 'input_model.mjb' not in proof['input_files']:
        raise ValueError('The authoritative physical MJB must have an input SHA')
    selection_path = root / 'one_bed_selection.json'
    selected = read(selection_path)
    if selected.get('schema') != 's300-single-bed-target-selection-v1':
        raise ValueError('Authoritative one-bed selection is required')
    inventory = Path(selected['source_inventory'])
    if sha256(inventory) != selected.get('source_inventory_sha256'):
        raise ValueError('One-bed source inventory SHA mismatch')
    checked = select_one_bed(selected['stations'] + selected.get('excluded', []), read(inventory),
        selected['bed_gutter_center_x'], selected.get('rail_x_m', 0.))
    if checked['stations'] != selected['stations'] or checked['excluded'] != selected.get('excluded', []):
        raise ValueError('Authoritative one-bed selection differs from the exact placement recheck')
    stations, windows = selected['stations'], meta.get('windows', [])
    if not stations or len(stations) != len(windows):
        raise ValueError('Prepared windows and selected station count differ')
    for station, window in zip(stations, windows):
        if int(window['id']) != station['plant_id']:
            raise ValueError('Prepared window/station order or plant identity differs')
        manifest = read(window['source_manifest'])
        if int(manifest['plant']['source_plant'].rsplit('_', 1)[-1]) != station['plant_id']:
            raise ValueError('Window source manifest belongs to another plant')
        keys = {truss_key(row['name']) for row in window['fruit_specs']}
        if keys != {f'Truss_{station["truss_index"]:02d}'}:
            raise ValueError('Prepared window fruit scope differs from the selected truss')
        plan = read(window['source_plan'])
        if sha256(window['source_plan']) != window.get('source_plan_sha256'):
            raise ValueError('Prepared source plan SHA mismatch')
        target_station = float(plan['rail_destination']['station_y_m'])
        if not 0 <= target_station <= 5 or abs(target_station - station['station_y_m']) > 1e-8:
            raise ValueError('Source plan rail destination differs from the selected station')
        load_policy(root / f'policy_plant{station["plant_id"]}.json',
                    window['source_assembly'], window['fruit_specs'])
    return common, proof, meta, selected


def sample_inputs(inputs, hz, free_hz, belt_hold_hz=None):
    """Native Python subprocess entry: reuse the established command sampler."""
    from newton_inputs import command_timeline
    inputs = Path(inputs)
    meta, plan = read(inputs / 'source_export.json'), read(inputs / 'sampled_plan.json')
    timeline, description, _ = command_timeline(
        dict(meta, prepared_source_plan=str(inputs / 'sampled_plan.json')),
        np.asarray(plan['initial_command']), hz, free_hz=free_hz, belt_hold_hz=belt_hold_hz)
    np.savez_compressed(inputs / 'commands.npz', **timeline)
    write(inputs / 'timeline.json', description)


def checked_parent(parent, common, initial, meta, contact_policy, expected_index):
    """Validate actual terminal state before group selection or replanning."""
    from newton_contact_policy import validate_config
    parent = Path(parent).resolve(strict=True)
    summary = read(parent / 'summary.json')
    parent_proof = read(Path(summary['input']) / 'inputs.json')
    sequence = parent_proof.get('same_bed_sequence', {})
    if sequence.get('index') != expected_index:
        raise ValueError('Parent is not the preceding selected route segment')
    if sequence.get('source_common') != str(common.resolve()):
        raise ValueError('Parent belongs to another prepared route')
    restored, state, history, evidence = load_resume(parent, common, initial, meta,
        policy=validate_config(read(contact_policy)))
    if sha256(parent / 'states.npz') != summary['states_sha256']:
        raise ValueError('Parent actual states SHA mismatch')
    with np.load(parent / 'states.npz', allow_pickle=False) as states:
        for field in ('qpos', 'qvel', 'eq_active'):
            if not np.array_equal(states[field][-1], state[field][0]):
                raise ValueError('Parent recorded final state differs from terminal snapshot: ' + field)
        if float(states['time'][-1]) != float(state['time'][0]):
            raise ValueError('Parent recorded and terminal clocks differ')
        poses, paths = states['poses'][-1].copy(), states['body_paths'].tolist()
    return restored, state, history, evidence, poses, paths


def prepare_segment(index, parent, output, common, root, selected, args):
    proof, meta = read(common / 'inputs.json'), read(common / 'source_export.json')
    window, station = meta['windows'][index], selected['stations'][index]
    plant = station['plant_id']
    source, manifest = read(window['source_plan']), read(window['source_manifest'])
    display = Path(meta['source_assembly'])
    origin = read(display / 'motion_manifest.json')['rail']['initial_station_y_m']
    with np.load(common / 'initial_state.npz', allow_pickle=False) as archive:
        initial = {key: archive[key].copy() for key in archive.files}
    meta['released_gutter_diagnostic'] = args.released_gutter_diagnostic
    depth = float(source['grasp_depth_m']) + args.depth_offset_m
    if not 0 < depth <= .158:
        raise ValueError('Adjusted depth must remain within the existing belt span (0,.158]m')
    if parent is None:
        if index != 0 or args.depth_offset_m != 0:
            raise ValueError('Index0 must reuse the original plan; depth adjustment requires a later measured segment')
        plan = read(common / 'sampled_plan.json')
        sequence = read(Path(window['source_trial']) / 'truss_sequence.json')
        group = dict(fruits=sequence['cycle_plans'][0]['fruits'], source_first_exact_plan_reused=True)
        if not set(group['fruits']) <= {row['name'] for row in window['fruit_specs']}:
            raise ValueError('First source plan group is outside the selected target truss')
    else:
        parent_checker = checked_parent
        if getattr(args, 'active_window', False):
            from newton_window_continuation import checked_window_parent
            parent_checker = checked_window_parent
        restored, state, history, evidence, poses, paths = parent_checker(
            parent, common, initial, meta, args.contact_policy, index - 1)
        write(output / 'incoming_handoff.json', evidence)
        q = restored['qpos'][restored['robot_qpos_addresses']]
        fruits = copy.deepcopy(window['fruit_specs'])
        original_root, alias = manifest['plant']['root'], window['display_root']
        for fruit in fruits:
            if not fruit['path'].startswith(original_root + '/'):
                raise ValueError('Source fruit does not belong to the exact source root')
            fruit['path'] = alias + fruit['path'][len(original_root):]
        policy = load_policy(root / f'policy_plant{plant}.json', window['source_assembly'], window['fruit_specs'])
        direction = np.asarray(source['approach_direction_world'])
        height = manifest.get('grasp_reference', {}).get('height_offset_m', 0.) + source['grasp_height_offset_m']
        selector = HarvestGroupSelector(policy, lambda *args: direction, depth_m=depth,
            height_offset_m=height, output=output / 'group_decision.json')
        names = [row['name'] for row in meta['attachments']]
        eq, eq_ids = state['eq_active'][0], state['attachment_equality_ids']
        breaks = [dict(name=fruit['name'], joint=fruit['joint']) for fruit in fruits
            if not eq[eq_ids[names.index(window['prefix'] + fruit['name'])]]]
        group = selector(fruits, breaks, paths, poses, {}, max_attempts=2)
        if group is None:
            write(output / 'not_attempted.json', dict(reason=selector.last_decision, physics_executed=False))
            return None
        request = dict(measured_joint_q=q[1:8].tolist(), measured_jaw_q=q[8:].tolist(),
            measured_station_y_m=origin + float(q[0]), rail_origin_station_m=origin,
            station_y_m=station['station_y_m'], target_center=group['aim_world_m'],
            direction=direction.tolist(), depth_m=depth, height_offset_m=height,
            plan_options=dict(horizontal_ready_connector='joint',
                horizontal_ready_seed_index=source.get('horizontal_ready_seed_index', 8),
                horizontal_ready_normalize_turns=False))
        write(output / 'request.json', request)
        command = [args.planning_python, ADAPTER / 'plan_adapter.py', display,
            output / 'request.json', output / 'plan.json', '--source-plan', window['source_plan']]
        with (output / 'planning.log').open('w') as stream:
            subprocess.run(list(map(str, command)), cwd=REPO, check=True,
                           stdout=stream, stderr=subprocess.STDOUT)
        plan = read(output / 'plan.json')
    write(output / 'selected_group.json', group)
    write(output / 'plan.json', plan)
    write(output / 'depth_adjustment.json', dict(source_depth_m=source['grasp_depth_m'],
        offset_m=args.depth_offset_m, selected_and_planned_depth_m=depth,
        original_plan_preserved=True, measured_replan=parent is not None))
    inputs = output / 'inputs'; inputs.mkdir()
    mutable = {'commands.npz', 'timeline.json', 'source_export.json', 'sampled_plan.json'}
    measured_initial = getattr(args, 'active_window', False) and parent is not None
    if measured_initial:
        mutable.add('initial_state.npz')
    shared = {name: evidence for name, evidence in proof['input_files'].items() if name not in mutable}
    link_manifest_assets(common, inputs, shared)
    if measured_initial:
        # Canonical bank, not a reset to the original unharvested plants.
        # The reduced model below is compiled from this actual measured state.
        np.savez_compressed(inputs / 'initial_state.npz', **restored)
    meta['active_harvest_attachment_names'] = [window['prefix'] + row['name'] for row in window['fruit_specs']]
    meta['same_bed_current_target'] = dict(plant_id=plant,
        gutter_center_x=selected['bed_gutter_center_x'], sequence_index=index)
    write(inputs / 'source_export.json', meta)
    write(inputs / 'sampled_plan.json', plan)
    sample_command = [str(args.physics_python), str(Path(__file__).resolve()), '--sample-inputs',
        str(inputs), '--hz', str(args.hz), '--free-hz', str(args.free_hz)]
    if args.belt_hold_hz is not None:
        sample_command.extend(['--belt-hold-hz', str(args.belt_hold_hz)])
    subprocess.run(sample_command, cwd=REPO, check=True)
    newproof = copy.deepcopy(proof)
    shared_hashes = verify_manifest_files(inputs, shared)
    for name in newproof['input_files']:
        digest = shared_hashes[name] if name in shared_hashes else sha256(inputs / name)
        newproof['input_files'][name] = dict(path=str(inputs / name), sha256=digest)
    newproof['input_files']['sampled_plan.json'] = dict(path=str(inputs / 'sampled_plan.json'), sha256=sha256(inputs / 'sampled_plan.json'))
    newproof.update(timeline=read(inputs / 'timeline.json'), same_bed_sequence=dict(index=index,
        plant_id=plant, source_common=str(common.resolve()), parent_run=None if parent is None else str(parent),
        depth_offset_m=args.depth_offset_m, current_target_only_belt_relaxation=True, training_eligible=False))
    write(inputs / 'inputs.json', newproof)
    if shared_hashes['input_model.mjb'] != proof['input_files']['input_model.mjb']['sha256'].lower():
        raise ValueError('Segment creation changed the common physical MJB')
    return inputs


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-root', type=Path, help='Prepared route assets containing inputs_route and one_bed_selection.json')
    parser.add_argument('--campaign', default='sequence_gpu', help='New child directory name; existing directories are refused')
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--end-index', type=int, help='Exclusive window index; default all selected windows')
    parser.add_argument('--parent', type=Path, help='Completed immediately preceding segment, for start-index>0')
    parser.add_argument('--depth-offset-m', type=float, default=0.,
        help='Opt-in depth change for measured replanning at index>0; index0 original-plan changes are refused')
    parser.add_argument('--planning-python', type=Path, default=REPO / 'nvidia-sim/.asset-venv/bin/python')
    parser.add_argument('--physics-python', type=Path, default=REPO / 'nvidia-sim/.newton-venv/bin/python')
    parser.add_argument('--hz', type=int, default=220)
    parser.add_argument('--free-hz', type=int, default=60)
    parser.add_argument('--belt-hold-hz', type=float,
        help='Opt-in accuracy trial only for stationary belt_finish; retain other phase samples')
    parser.add_argument('--nconmax', type=int, default=256)
    parser.add_argument('--njmax', type=int, default=512)
    parser.add_argument('--dense-hessian-capacity', type=int, default=128)
    parser.add_argument('--dense-hessian-overflow-capacity', type=int)
    parser.add_argument('--local-model-cache', type=Path)
    parser.add_argument('--active-window', action='store_true',
        help='Opt-in current plant elastic, others fixed at measured poses; retain detached fruit dynamics and full canonical state bank')
    parser.add_argument('--solver-tolerance', type=float,
        help='Explicit GPU convergence tolerance after upload; default preserves the existing GPU setting')
    parser.add_argument('--contact-policy', type=Path, default=HERE / 'profiles/belt_strokes_2mm.json')
    parser.add_argument('--released-gutter-diagnostic', action=argparse.BooleanOptionalAction, default=False,
        help='Opt in to stop-only exemption for already released fruit against exact gutter colliders; collisions and force recording remain active')
    parser.add_argument('--validate-only', action='store_true', help='Read/check prepared assets without planning, physics or output creation')
    parser.add_argument('--sample-inputs', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        solver_tolerance_value(args.solver_tolerance)
    except ValueError as error:
        parser.error(str(error))
    if not np.isfinite(args.depth_offset_m): parser.error('Finite depth offset required')
    if not 0 < args.free_hz <= args.hz or min(args.nconmax, args.njmax) < 1:
        parser.error('Positive capacities and 0<free-hz<=hz are required')
    from newton_belt_hold import validate_belt_hold_hz
    try:
        validate_belt_hold_hz(args.belt_hold_hz, args.hz)
    except ValueError as error:
        parser.error(str(error))
    if not 1 <= args.dense_hessian_capacity <= args.njmax:
        parser.error('Dense Hessian capacity must be positive and no larger than njmax')
    if args.sample_inputs is None and args.prepared_root is None:
        parser.error('--prepared-root is required')
    if args.sample_inputs is None:
        try: safe_name(args.campaign)
        except ValueError as error: parser.error(str(error))
        if (args.start_index < 0 or args.end_index is not None and args.end_index <= args.start_index or
                (args.start_index == 0) != (args.parent is None)):
            parser.error('Start at index0 without a parent, or a later index with its completed predecessor')
        if args.start_index == 0 and args.depth_offset_m != 0:
            parser.error('Index0 original plan cannot use --depth-offset-m')
    return args


def main(argv=None):
    args = arguments(argv)
    if args.sample_inputs is not None:
        sample_inputs(args.sample_inputs, args.hz, args.free_hz, args.belt_hold_hz)
        return
    root = args.prepared_root.resolve(strict=True)
    common, proof, meta, selected = validate_prepared(root)
    end = len(selected['stations']) if args.end_index is None else args.end_index
    if not args.start_index < end <= len(selected['stations']):
        raise ValueError('Requested route range is outside the authoritative selected windows')
    if args.validate_only:
        print(json.dumps(dict(prepared_root=str(root), selection=selected,
            physical_model_sha256=proof['input_files']['input_model.mjb']['sha256'], physical_steps=0), indent=2))
        return
    campaign = root / safe_name(args.campaign)
    campaign.mkdir(exist_ok=False)
    write(campaign / 'invocation.json', dict(argv=sys.argv, code_sha256=sha256(__file__),
        prepared_root=str(root), selection_sha256=sha256(root / 'one_bed_selection.json'),
        physical_model_sha256=proof['input_files']['input_model.mjb']['sha256'],
        planning_python=str(args.planning_python), physics_python=str(args.physics_python),
        options={key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        training_eligible=False, harvest_success=None))
    parent, rows = args.parent, []
    for index in range(args.start_index, end):
        station = selected['stations'][index]
        output = campaign / f'step_{index:02d}_plant{station["plant_id"]}'
        output.mkdir(exist_ok=False)
        try:
            segment_started = time.perf_counter()
            timings = {}
            print('Preparing measured plan', index, 'plant', station['plant_id'], flush=True)
            inputs = prepare_segment(index, parent, output, common, root, selected, args)
            timings['plan_and_command_preparation_s'] = time.perf_counter() - segment_started
            if inputs is None:
                rows.append(dict(index=index, plant_id=station['plant_id'], status='not_attempted'))
                write(campaign / 'campaign.json', dict(rows=rows, training_eligible=False, harvest_success=None))
                break
            transition = None
            if args.active_window:
                reduction_started = time.perf_counter()
                print('Compiling active plant and fixed neighboring colliders', station['plant_id'], flush=True)
                from newton_active_window import prepare
                from newton_window_continuation import create_transition
                with np.load(inputs / 'initial_state.npz', allow_pickle=False) as archive:
                    bank = {key: archive[key].copy() for key in archive.files}
                prepare(inputs, output / 'active_inputs', station['plant_id'], bank_state=bank)
                inputs = output / 'active_inputs'
                if parent is not None:
                    transition = create_transition(inputs, parent, common)
                timings['model_reduction_and_transition_validation_s'] = time.perf_counter() - reduction_started
            command = ['flock', '-x', '/tmp/s300_solver_compare_gpu.lock',
                str(REPO / 'nvidia-sim/run_s300_newton.sh'), 'gpu-realtime', str(inputs),
                '--hz', str(args.hz), '--free-hz', str(args.free_hz),
                '--nconmax', str(args.nconmax), '--njmax', str(args.njmax),
                '--dense-hessian-capacity', str(args.dense_hessian_capacity),
                '--contact-policy', str(args.contact_policy), '--output', str(output / 'physics')]
            if args.released_gutter_diagnostic: command.append('--released-gutter-diagnostic')
            if args.belt_hold_hz is not None:
                command.extend(['--belt-hold-hz', str(args.belt_hold_hz)])
            if args.solver_tolerance is not None:
                command.extend(['--solver-tolerance', str(args.solver_tolerance)])
            if args.dense_hessian_overflow_capacity is not None:
                command.extend(['--dense-hessian-overflow-capacity', str(args.dense_hessian_overflow_capacity)])
            if args.local_model_cache is not None:
                command.extend(['--local-model-cache', str(args.local_model_cache)])
            if transition is not None:
                command.extend(['--window-transition', str(transition)])
            elif parent is not None: command.extend(['--resume-state', str(parent.resolve())])
            write(output / 'execution_command.json', command)
            print('Executing', index, 'plant', station['plant_id'], flush=True)
            execution_started = time.perf_counter()
            with (output / 'physics.log').open('w') as stream:
                physics_env = dict(os.environ, S300_NEWTON_PYTHON=str(args.physics_python))
                subprocess.run(command, cwd=REPO, env=physics_env, check=True,
                               stdout=stream, stderr=subprocess.STDOUT)
            timings['physics_process_including_lock_wait_setup_and_save_s'] = time.perf_counter() - execution_started
            timings['total_segment_orchestration_s'] = time.perf_counter() - segment_started
            write(output / 'orchestration_timings.json', timings)
            summary = read(output / 'physics/summary.json')
            rows.append(dict(index=index, plant_id=station['plant_id'], run=str(output / 'physics'),
                status=summary['status'], stop=summary.get('stop'), simulated_s=summary['simulated_s'],
                realtime_factor=summary['realtime_factor'], new_detachments=len(summary['breaks'])))
            write(campaign / 'campaign.json', dict(rows=rows, training_eligible=False, harvest_success=None))
            print(rows[-1], flush=True)
            if summary['status'] != 'completed': break
            parent = output / 'physics'
        except Exception as error:
            write(output / 'orchestration_error.json', dict(type=type(error).__name__, error=str(error),
                physics_started=(output / 'physics/summary.json').exists()))
            raise


if __name__ == '__main__':
    main()
