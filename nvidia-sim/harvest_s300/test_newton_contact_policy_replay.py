import unittest
from types import SimpleNamespace

import numpy as np
import warp as wp

from newton_contact_policy import (ContactPolicyGPU, clear_pair_depth,
    classify_contacts, evaluate_pair_policy)
from newton_contact_policy_replay import replay_stop_policy, NO_EVENT
from test_newton_contact_policy import fixture, arr


class PolicyReplayTests(unittest.TestCase):
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
