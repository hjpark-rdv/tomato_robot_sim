import unittest
import hashlib
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import warp as wp

from newton_contact_policy import (ContactPolicyGPU, clear_pair_depth,
    classify_contacts, evaluate_pair_policy)
from newton_contact_policy_replay import replay_stop_policy, NO_EVENT, model_hash_evidence
from test_newton_contact_policy import fixture, arr


class PolicyReplayTests(unittest.TestCase):
    def test_physical_mjb_array_and_display_hashes_are_distinct(self):
        with tempfile.TemporaryDirectory() as directory:
            content=b'physical input MJB bytes'
            (Path(directory)/'input_model.mjb').write_bytes(content)
            evidence=model_hash_evidence(dict(input=directory,model_sha256='display USD compatibility',
                display_source_model_sha256='display USD compatibility',
                engine=dict(source_model_array_sha256='physical compiled arrays')))
        self.assertEqual(evidence,dict(physical_input_mjb_sha256=hashlib.sha256(content).hexdigest(),
            physical_model_arrays_sha256='physical compiled arrays',
            display_source_model_sha256='display USD compatibility'))
        self.assertNotIn('source_model_sha256',evidence)

    def test_replay_matches_warp_pair_classes_events_and_hysteresis(self):
        model, metadata = fixture()
        metadata['tools'] = ['Tool_L', 'Tool_L_belt', 'Tool_R_belt']
        belt = model.geom('beltL').id
        fruit = model.geom('fruit').id
        plate = model.geom('plate').id
        phases = np.array(['close', 'cycle_open', 'cycle_open', 'cycle_open', 'close', 'cycle_open'])
        times = np.array([0.05, .1, .2, .4, .5, .6], np.float32)
        depths = np.array([.0014, .0014, .0014, .0014, .0013, .0013], np.float32)
        n = len(phases)
        config = {'belt_fruit_m': .002, 'residual_open_seconds': .2}
        helper = ContactPolicyGPU(SimpleNamespace(mj_model=model),
            SimpleNamespace(device='cpu', steps=n),
            dict(phase=phases, time_edges=np.r_[0, times]), config, metadata)
        body_fruit = np.full(model.nbody, -1, np.int32)
        body_fruit[model.body('Truss_00_Fruit_01').id] = 0
        allowed = np.zeros((1, model.ngeom), bool)
        allowed[0, plate] = True
        pairs = np.tile(np.array([[belt, fruit], [plate, fruit]], np.int32), (n, 1, 1))
        witness = dict(geom_pairs=pairs,
            signed_distance_m=np.c_[-depths, np.full(n, -.003, np.float32)],
            constraint_active=np.ones((n, 2), bool),
            fruit_attached_before_release=np.tile(np.array([[-1, 1], [-1, 0]], np.int32), (n, 1, 1)),
            actual_step_end_time_s=times, nacon=np.full(n, 2, np.int32),
            world_wrench_on_second_geom=np.zeros((n, 2, 6), np.float32))
        # Both contacts are active, but released fruit/plate is excepted.
        for t in range(n):
            wp.launch(clear_pair_depth, helper.pair_depth.shape, inputs=[helper.pair_depth], device='cpu')
            # The controller attached state is shared by all contacts of a fruit.
            # Here use an attached fruit for the belt and a separate inactive
            # contact slot for the released exception in this kernel equality
            # control; released pair logic has a separate fixture below.
            witness['constraint_active'][t, 1] = False
            wp.launch(classify_contacts, 2, inputs=[arr([t], int), arr([2], int),
                arr([0, 0], int), arr(pairs[t], wp.vec2i), arr(witness['signed_distance_m'][t]),
                arr([[0], [-1]], int), arr(model.geom_bodyid, int), arr(body_fruit, int),
                arr(allowed, bool), arr([[True]], bool), helper.belt_index, helper.roles,
                model.ngeom, helper.explicit_lookup, helper.explicit_limits, helper.pair_depth,
                helper.class_max, .001, helper.first_event], device='cpu')
            wp.launch(evaluate_pair_policy, helper.pair_depth.shape, inputs=[arr([t], int), helper.phase,
                arr([times[t]]), helper.thresholds, helper.pair_depth, helper.latched,
                helper.open_start, .2, .001, helper.belt_geoms, helper.roles, model.ngeom,
                helper.allowed_steps, helper.residual_steps, helper.first_event], device='cpu')
        schema = dict(geoms=[dict(id=g, name=model.geom(g).name,
            body=model.body(int(model.geom_bodyid[g])).name) for g in range(model.ngeom)])
        replay = replay_stop_policy(witness, phases, schema, metadata, config)
        self.assertEqual(replay['first_policy_event'], int(helper.first_event.numpy()[0]))
        np.testing.assert_array_equal(replay['max_by_class'], helper.class_max.numpy()[0])
        self.assertEqual(replay['allowed_pair_steps'], helper.allowed_steps.numpy()[0].tolist())
        self.assertEqual(replay['residual_allowed_pair_steps'], helper.residual_steps.numpy()[0].tolist())
        self.assertEqual(replay['first_event_detail']['step'], 3)

    def test_active_target_scope_matches_gpu_and_retains_released_future_fruit_exception(self):
        model, metadata = fixture(multiple_trusses=True)
        metadata['active_harvest_attachment_names'] = ['Truss_00_Fruit_01']
        metadata['tools'] = ['Tool_L', 'Tool_L_belt', 'Tool_R_belt']
        belt = model.geom('beltL').id
        current, future, pedicel = [model.geom(name).id
            for name in ('fruit', 'future_fruit', 'future_pedicel')]
        pairs = np.array([[[belt, current]], [[belt, pedicel]],
                          [[belt, future]], [[belt, future]]], np.int32)
        phases = np.array(['close'] * 4)
        times = np.array([.05, .1, .15, .2], np.float32)
        config = dict(belt_fruit_m=.002, belt_pedicel_m=.002)
        helper = ContactPolicyGPU(SimpleNamespace(mj_model=model),
            SimpleNamespace(device='cpu', steps=4),
            dict(phase=phases, time_edges=np.r_[0, times]), config, metadata)
        fruit_map = np.full(model.nbody, -1, np.int32)
        for i, item in enumerate(metadata['attachments']):
            fruit_map[model.body(item['name']).id] = i
        allowed = np.zeros((3, model.ngeom), bool)
        allowed[:, belt] = True
        witness = dict(geom_pairs=pairs,
            signed_distance_m=np.array([[-.0015], [-.0015], [-.0015], [-.009]], np.float32),
            constraint_active=np.ones((4, 1), bool),
            fruit_attached_before_release=np.array([[[-1, 1]], [[-1, -1]],
                                                   [[-1, 1]], [[-1, 0]]], np.int32),
            actual_step_end_time_s=times, nacon=np.ones(4, np.int32),
            world_wrench_on_second_geom=np.zeros((4, 1, 6), np.float32))
        for tick in range(4):
            attached = np.ones((1, 3), bool)
            if tick == 3:
                attached[0, 1] = False
            wp.launch(clear_pair_depth, helper.pair_depth.shape,
                inputs=[helper.pair_depth], device='cpu')
            wp.launch(classify_contacts, 1, inputs=[arr([tick], int), arr([1], int),
                arr([0], int), arr(pairs[tick], wp.vec2i), arr(witness['signed_distance_m'][tick]),
                arr([[0]], int), arr(model.geom_bodyid, int), arr(fruit_map, int),
                arr(allowed, bool), arr(attached, bool), helper.belt_index, helper.roles,
                model.ngeom, helper.explicit_lookup, helper.explicit_limits,
                helper.pair_depth, helper.class_max, .001, helper.first_event], device='cpu')
            wp.launch(evaluate_pair_policy, helper.pair_depth.shape,
                inputs=[arr([tick], int), helper.phase, arr([times[tick]]), helper.thresholds,
                    helper.pair_depth, helper.latched, helper.open_start, 0., .001,
                    helper.belt_geoms, helper.roles, model.ngeom, helper.allowed_steps,
                    helper.residual_steps, helper.first_event], device='cpu')
        schema = dict(geoms=[dict(id=g, name=model.geom(g).name,
            body=model.body(int(model.geom_bodyid[g])).name) for g in range(model.ngeom)])
        replay = replay_stop_policy(witness, phases, schema, metadata, config)
        self.assertEqual(replay['first_policy_event'], int(helper.first_event.numpy()[0]))
        self.assertEqual(replay['first_event_detail']['step'], 1)
        np.testing.assert_array_equal(replay['max_by_class'], helper.class_max.numpy()[0])
        self.assertAlmostEqual(replay['max_by_class'][0][1], .0015, places=8)
        self.assertAlmostEqual(replay['max_by_class'][1][0], .0015, places=8)
        self.assertAlmostEqual(replay['max_by_class'][2][0], .0015, places=8)
        np.testing.assert_array_equal(replay['max_by_class'][3], 0.)
        self.assertEqual(replay['allowed_pair_steps'], helper.allowed_steps.numpy()[0].tolist())
        self.assertEqual(len(metadata['attachments']), 3)

    def test_released_gutter_diagnostic_is_exact_and_does_not_excuse_attached_fruit_or_robot(self):
        model, metadata = fixture()
        metadata['tools'] = ['Tool_L']
        metadata['farm'] = dict(structural_inventory=[
            dict(name='ExactGutter', classification='gutter'),
            dict(name='StructuralFrame', classification='house')])
        fruit, plate = [model.geom(name).id for name in ('fruit', 'plate')]
        gutter, other, misleading = model.ngeom, model.ngeom + 1, model.ngeom + 2
        schema = dict(geoms=[dict(id=g, name=model.geom(g).name,
            body=model.body(int(model.geom_bodyid[g])).name) for g in range(model.ngeom)] + [
            dict(id=gutter, name='ExactGutter', body='world'),
            dict(id=other, name='StructuralFrame', body='world'),
            dict(id=misleading, name='UnregisteredGutter', body='world')])
        def check(pair, attached):
            witness = dict(geom_pairs=np.asarray([[pair]], np.int32),
                signed_distance_m=np.full((1, 1), -.003, np.float32),
                constraint_active=np.ones((1, 1), bool),
                fruit_attached_before_release=np.asarray([[attached]], np.int32),
                actual_step_end_time_s=np.asarray([.1], np.float32),
                nacon=np.ones(1, np.int32),
                world_wrench_on_second_geom=np.zeros((1, 1, 6), np.float32))
            return replay_stop_policy(witness, ['close'], schema, metadata, {})
        self.assertNotEqual(check([fruit, gutter], [0, -1])['first_policy_event'], NO_EVENT)
        metadata['released_gutter_diagnostic'] = True
        self.assertEqual(check([fruit, gutter], [0, -1])['first_policy_event'], NO_EVENT)
        self.assertEqual(check([gutter, fruit], [-1, 0])['first_policy_event'], NO_EVENT)
        for pair, attached in (([fruit, gutter], [1, -1]), ([plate, gutter], [-1, -1]),
                               ([fruit, other], [0, -1]), ([fruit, misleading], [0, -1])):
            self.assertNotEqual(check(pair, attached)['first_policy_event'], NO_EVENT)
        metadata['released_gutter_diagnostic'] = False
        self.assertNotEqual(check([fruit, gutter], [0, -1])['first_policy_event'], NO_EVENT)

    def test_released_pair_exception_and_exact_other_pair_phase(self):
        model, metadata = fixture()
        metadata['tools'] = ['Tool_L']
        fruit = model.geom('fruit').id
        plate = model.geom('plate').id
        schema = dict(geoms=[dict(id=g, name=model.geom(g).name,
            body=model.body(int(model.geom_bodyid[g])).name) for g in range(model.ngeom)])
        witness = dict(geom_pairs=np.array([[[fruit, plate]], [[fruit, plate]]], np.int32),
            signed_distance_m=np.full((2, 1), -.0015, np.float32),
            constraint_active=np.ones((2, 1), bool), fruit_attached_before_release=np.array([[[0, -1]], [[1, -1]]]),
            actual_step_end_time_s=np.array([.1, .2], np.float32), nacon=np.ones(2, np.int32),
            world_wrench_on_second_geom=np.zeros((2, 1, 6), np.float32))
        config = {'explicit_other_pairs': [dict(geoms=['fruit', 'plate'], limit_m=.002, phases=['close'])]}
        result = replay_stop_policy(witness, ['close', 'cycle_open'], schema, metadata, config)
        self.assertEqual(result['first_event_detail']['step'], 1)
        witness['fruit_attached_before_release'][1, 0, 0] = 0
        self.assertEqual(replay_stop_policy(witness, ['close', 'cycle_open'], schema,
            metadata, config)['first_policy_event'], NO_EVENT)


if __name__ == '__main__':
    unittest.main()
