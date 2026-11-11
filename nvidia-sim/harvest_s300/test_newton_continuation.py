"""Measured continuation provenance/state tests without a simulator or GPU."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from newton_continuation import (file_sha256, model_identity, load_resume,
                                 verify_command_start, cumulative_breaks, VerificationContext)


class MeasuredContinuationTests(unittest.TestCase):
    def test_readonly_json_reuses_parsing_but_rejects_changed_input_bytes(self):
        path=self.root/'immutable.json';path.write_text('{"value": [1, 2]}')
        scope=VerificationContext(scope='patrol')
        first=scope.readonly_json(path)
        self.assertIs(scope.readonly_json(path),first)
        alias=self.root/'alias.json';alias.symlink_to(path)
        self.assertIs(scope.readonly_json(alias),first)
        path.write_text('{"value": [3, 4, 5]}')
        with self.assertRaisesRegex(ValueError,'changed within scope'):scope.readonly_json(alias)

    def test_online_camera_endpoint_preserves_exact_proposed_prefix(self):
        full=dict(time=np.arange(30)/10.,dt=np.full(30,.1),command=np.zeros((30,2)),
                  phase=np.full(30,'rail_transfer'),belt_surface_velocity_local_y_m_s=np.zeros((30,2)))
        np.savez(self.parent/'proposed_commands.npz',**full)
        prefix={k:v[:17] for k,v in full.items()}
        np.savez(self.parent/'executed_commands.npz',**prefix)
        self.summary['executed_commands_sha256']=file_sha256(self.parent/'executed_commands.npz')
        self.summary['online_camera_boundary']=dict(completion_kind='online_observation_endpoint',
            executed_steps=17,maximum_planned_steps=30,
            proposed_commands_sha256=file_sha256(self.parent/'proposed_commands.npz'))
        self.save();self.load()
        prefix['command'][8,0]=.1
        np.savez(self.parent/'executed_commands.npz',**prefix)
        self.summary['executed_commands_sha256']=file_sha256(self.parent/'executed_commands.npz')
        self.save()
        with self.assertRaisesRegex(ValueError,'exact proposed prefix'):self.load()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.inputs = self.root / 'inputs'; self.inputs.mkdir()
        self.parent = self.root / 'parent'; self.parent.mkdir()
        (self.inputs / 'input_model.mjb').write_bytes(b'exact-signed-physical-model')
        self.initial = dict(qpos=np.zeros(4), qvel=np.zeros(3), eq_active=np.array([True, True]),
            ctrl=np.zeros(2), runtime_dof_damping=np.ones(3), body_names=np.array(['world', 'fruitA', 'fruitB']),
            robot_qpos_addresses=np.array([0, 2]), robot_dof_addresses=np.array([0, 2]),
            actuator_ids=np.array([0, 1]))
        self.meta = dict(attachments=[dict(name='fruitA', anchor='stemA'), dict(name='fruitB', anchor='stemB')])
        self.policy = dict(default_m=.001, relaxed_phases=['close'])
        self.state = dict(time=np.array([12.75], np.float32), qpos=np.array([[1., 2., 3., 4.]], np.float32),
            qvel=np.array([[.1, .2, .3]], np.float32), eq_active=np.array([[True, False]]),
            ctrl=np.array([[1.2, 3.4]], np.float32), runtime_dof_damping=np.array([[.3, .2, 0]], np.float32),
            qacc_warmstart=np.array([[.7, .8, .9]], np.float32), body_names=self.initial['body_names'],
            attachment_equality_ids=np.array([1, 0]))
        self.breaks = [dict(world=0, name='fruitA', release_effective_time_s=10.25)]
        self.summary = dict(status='completed', stop=None, backend='gpu', worlds=1,
            finite_recorded_state=True, qacc_finite=True, solver_capacity_valid=True,
            device_command_buffers_match_schedule=True, controller_tick_matches_recorded_steps=True,
            actual_steps=17, requested_steps=17, input=str(self.inputs),
            contact_policy=dict(config=self.policy, policy_passed=True),
            continuation_model_identity=model_identity(self.initial, self.meta),
            final_physics_clock_s=12.75)
        (self.inputs / 'inputs.json').write_text(json.dumps(dict(input_files={
            'input_model.mjb': dict(sha256=file_sha256(self.inputs / 'input_model.mjb'))})))
        self.summary['input_sha256'] = file_sha256(self.inputs / 'inputs.json')
        np.savez_compressed(self.parent / 'executed_commands.npz', time=np.arange(17) / 10.)
        self.summary['executed_commands_sha256'] = file_sha256(self.parent / 'executed_commands.npz')
        self.save()

    def save(self):
        np.savez_compressed(self.parent / 'final_states.npz', **self.state)
        (self.parent / 'cumulative_break_events.json').write_text(json.dumps(self.breaks))
        self.summary['final_states_sha256'] = file_sha256(self.parent / 'final_states.npz')
        self.summary['cumulative_break_events_sha256'] = file_sha256(self.parent / 'cumulative_break_events.json')
        (self.parent / 'summary.json').write_text(json.dumps(self.summary))

    def load(self, **kwargs):
        return load_resume(self.parent, self.inputs, self.initial, self.meta,
                           policy=kwargs.pop('policy', self.policy), **kwargs)

    def test_preserves_measured_dynamic_state_and_nonzero_clock_without_touching_source(self):
        before = copy.deepcopy(self.initial)
        restored, state, history, proof = self.load()
        for key in ('qpos', 'qvel', 'eq_active', 'ctrl', 'runtime_dof_damping'):
            np.testing.assert_array_equal(restored[key], self.state[key][0])
        for key in before:
            np.testing.assert_array_equal(self.initial[key], before[key])
        self.assertEqual(proof['initial_clock_s'], 12.75)
        self.assertTrue(proof['qacc_warmstart_restored'])
        self.assertTrue(proof['contact_constraint_cache_rebuilt'])
        self.assertEqual(history, self.breaks)
        np.testing.assert_array_equal(state['qacc_warmstart'], self.state['qacc_warmstart'])

    def test_refuses_failed_incomplete_or_unverified_parent_even_with_usable_state(self):
        for key, value in [('status', 'stopped'), ('stop', 'contact_policy_violation'),
                ('actual_steps', 16), ('qacc_finite', False), ('solver_capacity_valid', False),
                ('device_command_buffers_match_schedule', False)]:
            original = self.summary[key]; self.summary[key] = value; self.save()
            with self.assertRaises(ValueError): self.load()
            self.summary[key] = original
        self.summary['contact_policy']['policy_passed'] = False; self.save()
        with self.assertRaisesRegex(ValueError, 'policy'): self.load()

    def test_rejects_changed_mjb_or_body_identity_and_state_tampering(self):
        other = self.root / 'other'; other.mkdir()
        (other / 'input_model.mjb').write_bytes(b'different-model')
        with self.assertRaisesRegex(ValueError, 'MJB'):
            load_resume(self.parent, other, self.initial, self.meta, policy=self.policy)
        wrong = copy.deepcopy(self.meta); wrong['attachments'][0]['anchor'] = 'wrong'
        with self.assertRaisesRegex(ValueError, 'identity'):
            load_resume(self.parent, self.inputs, self.initial, wrong, policy=self.policy)
        with (self.parent / 'final_states.npz').open('ab') as stream: stream.write(b'tampered')
        with self.assertRaisesRegex(ValueError, 'SHA'): self.load()

    def test_snapshot_cannot_omit_actual_control_or_use_nonfinite_state(self):
        del self.state['ctrl']; self.save()
        with self.assertRaisesRegex(ValueError, 'control/damping'): self.load()
        self.state['ctrl'] = np.array([[np.nan, 1.]])
        self.save()
        with self.assertRaisesRegex(ValueError, 'nonfinite'): self.load()

    def test_equity_order_is_explicit_and_events_cannot_recount_or_claim_attached_fruit(self):
        # fruitA is equality1, not metadata index0. Valid fixture proves this
        # order is used when checking inherited release evidence.
        self.load()
        self.breaks.append(dict(world=0, name='fruitB', release_effective_time_s=11.))
        self.save()
        with self.assertRaisesRegex(ValueError, 'active attachment'): self.load()
        with self.assertRaisesRegex(ValueError, 'recount'):
            cumulative_breaks(self.breaks, [self.breaks[0]])
        self.assertEqual(len(cumulative_breaks(self.breaks[:1], self.breaks[1:])), 2)

    def test_new_commands_must_start_at_measured_final_robot_configuration(self):
        restored, *_ = self.load()
        correct = restored['qpos'][restored['robot_qpos_addresses']]
        self.assertEqual(verify_command_start(dict(command=np.array([correct])), restored), 0.)
        with self.assertRaisesRegex(ValueError, 'measured final robot'):
            verify_command_start(dict(command=np.zeros((1, 2))), restored)

    def test_same_contact_policy_and_terminal_clock_required(self):
        with self.assertRaisesRegex(ValueError, 'policy differs'):
            self.load(policy=dict(default_m=.01))
        self.summary['final_physics_clock_s'] = 12.5; self.save()
        with self.assertRaisesRegex(ValueError, 'final clock'): self.load()

    def test_optional_scope_preserves_legacy_checks_and_refuses_changed_original(self):
        context=VerificationContext();restored,_,_,evidence=self.load(verification_context=context)
        np.testing.assert_array_equal(restored['qpos'],self.state['qpos'][0])
        scope=evidence['verification_scope']
        self.assertGreater(scope['stable_sha_reuses'],0)
        self.assertEqual(scope['unique_file_sha_reads'],len(scope['sources']))
        (self.inputs/'input_model.mjb').write_bytes(b'different')
        with self.assertRaisesRegex(ValueError,'changed within scope'):
            self.load(verification_context=context)

    def test_completed_short_prefix_is_not_a_completed_harvest_timeline(self):
        np.savez_compressed(self.parent / 'executed_commands.npz', time=np.arange(30) / 10.)
        self.summary['executed_commands_sha256'] = file_sha256(self.parent / 'executed_commands.npz')
        self.save()
        with self.assertRaisesRegex(ValueError, 'complete command timeline'): self.load()

    def test_released_gutter_scope_cannot_change_at_a_continuation_boundary(self):
        changed = dict(self.meta, released_gutter_diagnostic=True)
        with self.assertRaisesRegex(ValueError, 'released-gutter policy differs'):
            load_resume(self.parent, self.inputs, self.initial, changed, policy=self.policy)


if __name__ == '__main__':
    unittest.main()
