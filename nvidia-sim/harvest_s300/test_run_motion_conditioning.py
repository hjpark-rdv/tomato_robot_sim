"""Execute the real runner's CLI and conditioning wiring without importing Kit.

The helper's geometric inventory and checkpoint validator have separate tests.
These tests cover their integration: raw first-step evidence, proof ordering,
unchanged operational guards, and promotion only after the actual-pose audit.
"""
import ast
from contextlib import redirect_stderr, redirect_stdout
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).with_name('run_motion.py')
TREE = ast.parse(SOURCE.read_text())
MAIN = next(node for node in TREE.body if isinstance(node, ast.Try))
LOOP = next(node for node in MAIN.body if isinstance(node, ast.For) and
            isinstance(node.target, ast.Name) and node.target.id == 'tick')


def code(nodes, inside_loop=False):
    if inside_loop:
        # The actual guard contains break statements. One synthetic iteration
        # exercises those statements without executing a simulation step.
        nodes = [ast.For(target=ast.Name(id='_test_step', ctx=ast.Store()),
                         iter=ast.List(elts=[ast.Constant(0)], ctx=ast.Load()),
                         body=copy.deepcopy(nodes), orelse=[])]
    tree = ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[]))
    return compile(tree, str(SOURCE), 'exec')


def actual_cli(arguments):
    prefix = []
    for node in TREE.body:
        if (isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and
                isinstance(node.value.func, ast.Attribute) and node.value.func.attr == 'mkdir'):
            break
        prefix.append(node)
    namespace = {}
    with patch.object(sys, 'argv', [str(SOURCE), '/unused-assembly', '--output',
                                   '/unused-output', *arguments]):
        exec(code(prefix), namespace)
    return namespace['a']


CONDITIONING = ['--mode', 'idle', '--seconds', '4', '--condition-foliage-seconds', '3',
                '--buffer-contact-events', '--verify-leaf-contact-after-step',
                '--save-route-checkpoint']


class RunMotionConditioningTests(unittest.TestCase):
    def test_default_and_explicit_zero_preserve_legacy_cli(self):
        default = vars(actual_cli([]))
        explicit = vars(actual_cli(['--condition-foliage-seconds', '0']))
        self.assertEqual(default, explicit)
        self.assertEqual(default['mode'], 'harvest')
        self.assertEqual(default['plant_dynamics'], 'full')
        self.assertEqual(default['fruit_contact_limit_mm'], .5)
        self.assertFalse(default['allow_resume_conditioned_start'])
        # Existing opt-in contact diagnostics remain available outside the new
        # conditioning mode; the new mode must not rewrite their CLI meaning.
        self.assertEqual(actual_cli(['--fruit-contact-limit-mm', '.75']).fruit_contact_limit_mm, .75)

    def test_conditioning_accepts_only_bounded_stationary_full_strict_configuration(self):
        accepted = actual_cli(CONDITIONING)
        self.assertEqual(accepted.condition_foliage_seconds, 3.)
        self.assertEqual(accepted.seconds, 4.)
        rejected = [
            ['--condition-foliage-seconds', '-.1'],
            ['--condition-foliage-seconds', '3.1'],
            ['--condition-foliage-seconds', 'nan'],
            ['--condition-foliage-seconds', 'inf'],
            ['--seconds', '3.99'], ['--mode', 'harvest'], ['--num-envs', '2'],
            ['--replicate'], ['--rail-to-y', '1.8'], ['--contact-backend', 'tensor'],
            ['--fruit-contact-limit-mm', '.75'], ['--fruit-fruit-limit-mm', '.75'],
            ['--isolate-background'], ['--plant-dynamics', 'target-truss'],
            ['--zero-conveyor-diagnostic'],
        ]
        for flags in rejected:
            with self.subTest(flags=flags), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                actual_cli(CONDITIONING + flags)
        for required in ('--buffer-contact-events', '--verify-leaf-contact-after-step',
                         '--save-route-checkpoint'):
            flags = [flag for flag in CONDITIONING if flag != required]
            with self.subTest(missing=required), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                actual_cli(flags)

    def test_conditioned_resume_requires_explicit_same_model_without_revision_modes(self):
        resume = ['--whole-truss', '--harvest-profile', 'belt_open_close_cycle',
                  '--preparation', 'horizontal_ready', '--resume-native-checkpoint',
                  '/unused-checkpoint', '--allow-resume-conditioned-start']
        self.assertTrue(actual_cli(resume).allow_resume_conditioned_start)
        for flag in ('--allow-resume-leaf-proxy-revision',
                     '--allow-resume-accuracy-proxy-revision', '--allow-resume-attachment-transfer'):
            with self.subTest(flag=flag), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                actual_cli(resume + [flag])
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            actual_cli(['--allow-resume-conditioned-start'])

    def test_conditioning_total_observation_time_must_be_finite(self):
        # NaN bypasses a plain '< conditioning + strict_suffix' comparison;
        # infinity cannot form a finite native step count either.
        for seconds in ('nan', 'inf', '-inf'):
            with self.subTest(seconds=seconds), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                actual_cli(CONDITIONING + ['--seconds', seconds])

    def run_actual_guard(self, output, *, conditioning=True, defer=False,
                         certified=False, abort_reason=None, strict_error=None):
        first = next(i for i, node in enumerate(LOOP.body) if isinstance(node, ast.Assign) and
                     any(isinstance(t, ast.Name) and t.id == 'invalid' for t in node.targets))
        last = next(i for i in range(first, len(LOOP.body)) if isinstance(LOOP.body[i], ast.Assign) and
                    any(isinstance(t, ast.Name) and t.id == 'draw' for t in LOOP.body[i].targets))
        order = []
        event = dict(t=.25, a='/World/ElasticTarget/Leaf_CompoundLeaf_01/Mesh',
                     b='/World/Greenhouse/Plants/Plant_41/Stem/mesh_129',
                     separation_m=-.004, impulse_ns=[.1, .2, .3], force_n=12.)
        raw = copy.deepcopy(event)

        class Conditioner:
            leaves = {event['a']}
            step_records = []

            def before_geometry(self, stamp, invalid, breaks, poses, velocities):
                order.append(('record_raw', copy.deepcopy(invalid)))
                self.step_records.append(dict(deferred_count=len(invalid) if defer else 0))
                return dict(remaining_invalid=[] if defer else invalid, abort_reason=abort_reason)

            def after_geometry(self, stamp, invalid):
                order.append(('strict_after_geometry', copy.deepcopy(invalid)))
                return strict_error

        class Geometry:
            def eligible(self, *pair):
                return True

            def verify(self, *args):
                order.append(('native_geometry', args[-1]))
                return dict(certified_within_limit=certified)

        class Scope:
            def filter_invalid(self, invalid, breaks):
                order.append(('contact_scope', copy.deepcopy(invalid)))
                return invalid

        namespace = dict(a=SimpleNamespace(output=output), step_events=[event], contact_guard=None,
            foliage_conditioner=Conditioner() if conditioning else None, leaf_geometry=Geometry(),
            poses=[[0., 0., 0., 0., 0., 0., 1.]], breaks=[],
            bodies=SimpleNamespace(get_velocities=lambda: [[0.] * 6]), arr=lambda value: value,
            contact_scope=Scope(), current_time=.25, restored_initialization=True, tick=0,
            tensor_contacts=None, contact_buffer=None, stop_reason=None,
            initialization_validity=dict(valid=None), json=json)
        with redirect_stdout(io.StringIO()):
            exec(code(LOOP.body[first:last], inside_loop=True), namespace)
        self.assertEqual(event, raw, 'Runner must retain raw per-point impulse/separation evidence')
        return namespace, order, raw

    def test_deferred_raw_first_step_is_saved_invalid_before_skipping_geometry(self):
        with tempfile.TemporaryDirectory() as temp:
            result, order, raw = self.run_actual_guard(Path(temp), defer=True)
            self.assertEqual([row[0] for row in order],
                             ['record_raw', 'contact_scope', 'strict_after_geometry'])
            evidence = json.loads((Path(temp) / 'initialization_validity.json').read_text())
            self.assertFalse(evidence['valid'])
            self.assertFalse(evidence['operational_physics_valid'])
            self.assertEqual(evidence['state_scope'], 'initial_conditioning_state')
            self.assertEqual(evidence['violations'], [raw])
            self.assertEqual(evidence['deferred_count'], 1)
            self.assertTrue(evidence['conditional_continuation_is_not_clearance'])
            self.assertIsNone(result['stop_reason'])

    def test_strict_suffix_checks_native_geometry_before_guard_and_cannot_defer_collision(self):
        with tempfile.TemporaryDirectory() as temp:
            result, order, _ = self.run_actual_guard(Path(temp), strict_error='strict_penetration_over_0_5mm')
            self.assertEqual([row[0] for row in order],
                             ['record_raw', 'native_geometry', 'contact_scope', 'strict_after_geometry'])
            self.assertTrue(order[-1][1])
            self.assertEqual(result['stop_reason'], 'strict_penetration_over_0_5mm')
            self.assertFalse((Path(temp) / 'initialization_validity.json').exists())
            cleared, order, _ = self.run_actual_guard(Path(temp), certified=True)
            self.assertEqual(order[-1][1], [])
            self.assertIsNone(cleared['stop_reason'])
            # An actual clearance certificate may pass a strict solved step,
            # but it must never overwrite the raw authored-start evidence.
            self.assertFalse(cleared['initialization_validity']['valid'])

    def test_nonconditioning_guard_preserves_existing_proof_then_stop_behavior(self):
        with tempfile.TemporaryDirectory() as temp:
            failed, order, _ = self.run_actual_guard(Path(temp), conditioning=False)
            self.assertEqual([row[0] for row in order], ['native_geometry', 'contact_scope'])
            self.assertEqual(failed['stop_reason'], 'penetration_over_0_5mm')
            self.assertFalse(failed['initialization_validity']['valid'])
            cleared, _, _ = self.run_actual_guard(Path(temp), conditioning=False, certified=True)
            self.assertTrue(cleared['initialization_validity']['valid'])
            self.assertIsNone(cleared['stop_reason'])

    def test_hard_conditioning_abort_precedes_clearance_query_or_validity_promotion(self):
        with tempfile.TemporaryDirectory() as temp:
            failed, order, _ = self.run_actual_guard(Path(temp), abort_reason='new_native_joint_break')
            self.assertEqual([row[0] for row in order], ['record_raw'])
            self.assertEqual(failed['stop_reason'], 'new_native_joint_break')
            self.assertFalse((Path(temp) / 'initialization_validity.json').exists())

    def test_report_rejection_never_promotes_initial_raw_state(self):
        report_node = next(node for node in MAIN.body if isinstance(node, ast.If) and
                           ast.unparse(node.test) == 'foliage_conditioner is not None' and
                           any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and
                               n.func.id == 'validate_report' for n in ast.walk(node)))
        with tempfile.TemporaryDirectory() as temp:
            raw = dict(valid=False, state_scope='initial_conditioning_state', violations=['raw'])
            def reject(report):
                raise ValueError('No complete strict timestamp window')
            namespace = dict(a=SimpleNamespace(output=Path(temp)), json=json,
                foliage_conditioner=SimpleNamespace(report=lambda: dict(strict_window={})),
                stop_reason=None, initialization_validity=copy.deepcopy(raw))
            with patch.dict(sys.modules, {'foliage_conditioning': SimpleNamespace(validate_report=reject)}):
                exec(code([report_node]), namespace)
            self.assertEqual(namespace['initialization_validity'], raw)
            self.assertEqual(namespace['stop_reason'], 'canonical_conditioned_start_not_verified')
            self.assertFalse((Path(temp) / 'conditioning_initialization_evidence.json').exists())
            self.assertTrue(json.loads((Path(temp) / 'conditioning_rejection.json').read_text())['no_harvest_credit'])

    def test_verified_terminal_start_preserves_separate_invalid_beginning(self):
        report_node = next(node for node in MAIN.body if isinstance(node, ast.If) and
                           ast.unparse(node.test) == 'foliage_conditioner is not None' and
                           any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and
                               n.func.id == 'validate_report' for n in ast.walk(node)))
        with tempfile.TemporaryDirectory() as temp:
            raw = dict(valid=False, state_scope='initial_conditioning_state',
                       operational_physics_valid=False, violations=['raw_first_step'])
            report = dict(strict_window=dict(duration_s=1., sample_count=121, maximum_gap_s=1 / 120))
            namespace = dict(a=SimpleNamespace(output=Path(temp)), json=json,
                foliage_conditioner=SimpleNamespace(report=lambda: report),
                stop_reason=None, initialization_validity=copy.deepcopy(raw))
            with patch.dict(sys.modules, {'foliage_conditioning': SimpleNamespace(validate_report=lambda value: value)}):
                exec(code([report_node]), namespace)
            self.assertEqual(json.loads((Path(temp) / 'conditioning_initialization_evidence.json').read_text()), raw)
            terminal = namespace['initialization_validity']
            self.assertTrue(terminal['valid'])
            self.assertEqual(terminal['state_scope'], 'canonical_conditioned_terminal_start')
            self.assertEqual(terminal['authored_start_evidence'], 'conditioning_initialization_evidence.json')
            self.assertEqual(terminal['strict_window'], report['strict_window'])
            self.assertTrue(terminal['contact_guard_unchanged'])

    def test_final_canonical_marker_is_separate_from_cycles_and_requires_actual_pose_audit(self):
        checkpoint_node = next(node for node in MAIN.body if isinstance(node, ast.If) and
                               ast.unparse(node.test) == 'a.save_route_checkpoint')
        audit_node = next(node for node in MAIN.body if isinstance(node, ast.If) and
                          any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and
                              n.func.id == 'audit_external' for n in ast.walk(node)) and
                          'foliage_conditioner is not None' in ast.unparse(node.test))
        marker_node = next(node for node in MAIN.body if isinstance(node, ast.If) and
                           any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and
                               n.func.id == 'write_conditioned_start_marker' for n in ast.walk(node)))
        for audit_passes in (True, False):
            with self.subTest(audit_passes=audit_passes), tempfile.TemporaryDirectory() as temp:
                order = []
                report = dict(strict_window=dict(duration_s=1.))
                canonical_sequence = dict(cycle_plans=[], completed_cycle=False, harvest_attempt=False)
                def audit(*args, **kwargs):
                    order.append('actual_pose_audit')
                    if not audit_passes:
                        raise ValueError('Measured arm pose collides')
                namespace = dict(a=SimpleNamespace(output=Path(temp), assembly=Path('/unused'),
                    mode='idle', rail_to_y=None, hz=120, save_route_checkpoint=True,
                    native_tool_collision_cache=None),
                    conditioning_report=report, stop_reason=None, sequence=None, current_time=4.,
                    foliage_conditioner=object(), json=json,
                    summary=dict(canonical_conditioned_start=True, completed=True),
                    save_native_route_state=lambda path, stamp: order.append(
                        ('checkpoint', stamp, copy.deepcopy(namespace['sequence']))),
                    audit_external=audit)
                helper = SimpleNamespace(conditioned_checkpoint_sequence=lambda value: canonical_sequence,
                    write_conditioned_start_marker=lambda *args, **kwargs: order.append('canonical_marker'))
                with patch.dict(sys.modules, {'conditioned_start_checkpoint': helper,
                                             'motion_self_collision': SimpleNamespace(audit_external=audit)}), \
                        redirect_stdout(io.StringIO()):
                    exec(code([checkpoint_node]), namespace)
                    try:
                        exec(code([audit_node, marker_node]), namespace)
                    except ValueError:
                        self.assertFalse(audit_passes)
                self.assertEqual(order[0], ('checkpoint', 4., canonical_sequence))
                self.assertEqual(order[1], 'actual_pose_audit')
                self.assertEqual('canonical_marker' in order, audit_passes)
                if not audit_passes:
                    self.assertFalse(namespace['summary']['completed'])

    def test_final_route_capture_tags_only_an_exact_completed_native_cycle(self):
        checkpoint_node = next(node for node in MAIN.body if isinstance(node, ast.If) and
                               ast.unparse(node.test) == 'a.save_route_checkpoint')
        final = dict(status='motion_completed', start_s=2., end_s=54., planned_end_s=54.,
                     measured_frame_index=0, end_frame_index=6480, fruits=['Truss_02_Fruit_10'])
        cases = [
            ('exact_complete', final, None, 54., True),
            ('stopped', dict(final, status='stopped'), 'penetration_over_0_5mm', 54., False),
            ('unfinished', dict(final, status='executing'), None, 54., False),
            ('rejected_plan', dict(final, status='planning_rejected'), None, 54., False),
            ('stopped_after_record', final, 'actual_self_collision_audit_failed', 54., False),
            ('later_state_without_completed_cycle', final, None, 54. + 1 / 120, False),
            ('before_recorded_cycle_end', final, None, 54. - 1 / 120, False),
            ('ordinary_idle', None, None, 3., False),
        ]
        for name, record, stop, stamp, expected_tag in cases:
            with self.subTest(case=name):
                sequence = None if record is None else dict(cycle_plans=[copy.deepcopy(record)])
                captures = []
                namespace = dict(a=SimpleNamespace(save_route_checkpoint=True, output=Path('/unused')),
                    conditioning_report=None, stop_reason=stop, sequence=sequence, current_time=stamp,
                    save_native_route_state=lambda path, time: captures.append(
                        dict(time_s=time, sequence=copy.deepcopy(namespace['sequence']))))
                exec(code([checkpoint_node]), namespace)
                self.assertEqual(len(captures), 1)
                captured = captures[0]['sequence']
                if captured is None:
                    self.assertFalse(expected_tag)
                else:
                    self.assertEqual(captured.get('checkpoint_capture_reason') == 'completed_cycle', expected_tag)
                    self.assertEqual(captured.get('checkpoint_physics_valid') is True, expected_tag)
                    self.assertEqual(captured['cycle_plans'], [record])
                self.assertEqual(captures[0]['time_s'], stamp)

    def test_final_route_capture_opt_out_has_no_checkpoint_or_new_completion_tag(self):
        checkpoint_node = next(node for node in MAIN.body if isinstance(node, ast.If) and
                               ast.unparse(node.test) == 'a.save_route_checkpoint')
        sequence = dict(cycle_plans=[dict(status='motion_completed', end_s=54.)])
        before = copy.deepcopy(sequence)
        namespace = dict(a=SimpleNamespace(save_route_checkpoint=False), sequence=sequence,
            conditioning_report=None, stop_reason=None, current_time=54.,
            save_native_route_state=lambda *args: self.fail('Disabled checkpoint capture executed'))
        exec(code([checkpoint_node]), namespace)
        self.assertEqual(namespace['sequence'], before)


if __name__ == '__main__':
    unittest.main()
