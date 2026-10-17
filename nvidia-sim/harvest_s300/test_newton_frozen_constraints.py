"""CPU checks for exact zero-DOF frozen CONNECT GPU compatibility.

No GPU solver or physics integration runs here. Native MuJoCo forward and
CPU-backed Warp arrays verify identity, launch-list filtering and state scope.
"""
import copy
from types import SimpleNamespace
import unittest

import mujoco as mj
import numpy as np
import warp as wp

from newton_frozen_constraints import frozen_connect_ids, configure_frozen_connects


class FrozenConnectTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()

    def fixture(self, *, sites=False, movable_ancestor=False, wrong_pair=False,
                equality_kind='connect'):
        # The dynamic CONNECT must survive; the unmarked static CONNECT must
        # also survive because this compatibility fix has exact opt-in scope.
        frozen_body = '''<body name="frozen" pos=".2 0 0">
          <geom type="sphere" size=".01"/><site name="frozen_site"/></body>'''
        if movable_ancestor:
            frozen_body = '<body name="moving"><freejoint/>' + frozen_body + '</body>'
        if equality_kind == 'connect':
            if sites:
                target = 'other_site' if wrong_pair else 'frozen_site'
                frozen_eq = '<connect name="frozen_attachment" site1="anchor_site" site2="' + target + '"/>'
            else:
                target = 'other' if wrong_pair else 'frozen'
                frozen_eq = '<connect name="frozen_attachment" body1="anchor" body2="' + target + '" anchor="0 0 0"/>'
        else:
            frozen_eq = '<weld name="frozen_attachment" body1="anchor" body2="frozen"/>'
        model = mj.MjModel.from_xml_string('''<mujoco>
          <default><geom contype="0" conaffinity="0"/></default>
          <option gravity="0 0 0"/><worldbody>
            <body name="anchor"><geom type="sphere" size=".01"/>
              <site name="anchor_site"/></body>
            <body name="dynamic" pos=".1 0 0"><freejoint/>
              <geom type="sphere" size=".01" mass=".1"/></body>
            ''' + frozen_body + '''
            <body name="other" pos=".3 0 0"><geom type="sphere" size=".01"/>
              <site name="other_site"/></body>
          </worldbody><equality>
            <connect name="dynamic_attachment" body1="anchor" body2="dynamic" anchor="0 0 0"/>
            ''' + frozen_eq + '''
            <connect name="other_attachment" body1="anchor" body2="other" anchor="0 0 0"/>
          </equality></mujoco>''')
        metadata = dict(attachments=[dict(name='dynamic'), dict(name='frozen'), dict(name='other')],
            active_window=dict(frozen_attachment_names=['frozen']))
        return model, metadata

    def test_body_and_site_connect_resolve_exact_static_fruit_and_native_omits_rows(self):
        for sites in (False, True):
            with self.subTest(sites=sites):
                model, metadata = self.fixture(sites=sites)
                expected = model.equality('frozen_attachment').id
                self.assertEqual(frozen_connect_ids(model, metadata), [expected])
                data = mj.MjData(model)
                mj.mj_forward(model, data)
                # Only the retained dynamic equality has three real rows.
                self.assertEqual(data.nefc, 3)
                np.testing.assert_array_equal(data.efc_id[:data.nefc],
                    np.full(3, model.equality('dynamic_attachment').id))
                np.testing.assert_array_equal(data.eq_active, True)

    def test_body_or_site_with_movable_ancestor_cannot_be_omitted(self):
        for sites in (False, True):
            with self.subTest(sites=sites):
                model, metadata = self.fixture(sites=sites, movable_ancestor=True)
                fruit = model.body('frozen').id
                self.assertEqual(model.body_jntnum[fruit], 0)
                self.assertNotEqual(model.body_weldid[fruit], 0)
                with self.assertRaisesRegex(ValueError, 'movable ancestor'):
                    frozen_connect_ids(model, metadata)

    def test_unknown_duplicate_nonconnect_and_wrong_named_fruit_are_rejected(self):
        model, metadata = self.fixture()
        for names in (['missing'], ['frozen', 'frozen']):
            changed = copy.deepcopy(metadata)
            changed['active_window']['frozen_attachment_names'] = names
            with self.subTest(names=names), self.assertRaisesRegex(ValueError, 'Unknown or duplicate'):
                frozen_connect_ids(model, changed)
        model, metadata = self.fixture(equality_kind='weld')
        with self.assertRaisesRegex(ValueError, 'not CONNECT'):
            frozen_connect_ids(model, metadata)
        for sites in (False, True):
            model, metadata = self.fixture(sites=sites, wrong_pair=True)
            with self.subTest(wrong_pair_sites=sites), self.assertRaisesRegex(ValueError, 'named fruit'):
                frozen_connect_ids(model, metadata)

    def fake_gpu(self, model, *, addresses=(2, 1, 0)):
        state = mj.MjData(model)
        return SimpleNamespace(mj_model=model, use_mujoco_cpu=False, _step=0,
            mjw_model=SimpleNamespace(eq_connect_adr=wp.array(addresses, dtype=int, device='cpu')),
            mjw_data=SimpleNamespace(qpos=wp.array(state.qpos[None], dtype=float, device='cpu'),
                eq_active=wp.array(state.eq_active[None], dtype=bool, device='cpu')))

    def test_cpu_backed_gpu_list_filters_only_marked_id_and_preserves_all_activity(self):
        model, metadata = self.fixture()
        solver = self.fake_gpu(model)
        original_list = solver.mjw_model.eq_connect_adr
        q_before = solver.mjw_data.qpos.numpy().copy()
        activity_before = solver.mjw_data.eq_active.numpy().copy()
        physical_before = {name: getattr(model, name).copy() for name in
            ('eq_active0', 'eq_type', 'eq_obj1id', 'eq_obj2id', 'eq_data', 'geom_contype',
             'geom_conaffinity', 'body_mass', 'body_inertia')}
        proof = configure_frozen_connects(solver, metadata)
        np.testing.assert_array_equal(solver.mjw_model.eq_connect_adr.numpy(), [2, 0])
        np.testing.assert_array_equal(original_list.numpy(), [2, 1, 0])
        np.testing.assert_array_equal(solver.mjw_data.eq_active.numpy(), activity_before)
        np.testing.assert_array_equal(solver.mjw_data.qpos.numpy(), q_before)
        for name, previous in physical_before.items():
            np.testing.assert_array_equal(getattr(model, name), previous)
        self.assertEqual(model.neq, 3)
        self.assertTrue(proof['enabled'])
        self.assertEqual(proof['omitted_static_connect_ids'], [1])
        self.assertEqual(proof['original_connect_count'], 3)
        self.assertEqual(proof['dynamic_connect_count'], 2)
        self.assertTrue(proof['attachment_activity_unchanged'])
        self.assertFalse(proof['collider_changes'])

    def test_cached_configuration_is_idempotent_only_with_its_original_provenance(self):
        model, metadata = self.fixture(sites=True)
        solver = self.fake_gpu(model)
        original = solver.mjw_model.eq_connect_adr
        first = configure_frozen_connects(solver, metadata)
        filtered = solver.mjw_model.eq_connect_adr
        activity = solver.mjw_data.eq_active.numpy().copy()
        second = configure_frozen_connects(solver, copy.deepcopy(metadata))
        self.assertIs(solver.mjw_model.eq_connect_adr, filtered)
        self.assertIs(solver._s300_frozen_connect_configuration['original_array'], original)
        self.assertFalse(first['idempotent_reuse']); self.assertTrue(second['idempotent_reuse'])
        self.assertEqual(first['original_connect_launch_ids'], [2, 1, 0])
        self.assertEqual(first['retained_connect_launch_ids'], [2, 0])
        np.testing.assert_array_equal(activity, solver.mjw_data.eq_active.numpy())
        self.assertEqual({k:v for k,v in second.items() if k != 'idempotent_reuse'},
                         {k:v for k,v in first.items() if k != 'idempotent_reuse'})

    def test_manual_array_replacement_reordering_and_saved_original_mutation_are_rejected(self):
        for mutation in ('replacement', 'reorder', 'original'):
            with self.subTest(mutation=mutation):
                model, metadata = self.fixture(); solver = self.fake_gpu(model)
                configure_frozen_connects(solver, metadata)
                if mutation == 'replacement':
                    solver.mjw_model.eq_connect_adr = wp.array([2, 0], dtype=int, device='cpu')
                elif mutation == 'reorder':
                    solver.mjw_model.eq_connect_adr.assign(np.array([0, 2], np.int32))
                else:
                    original = solver._s300_frozen_connect_configuration['original_array']
                    original.assign(np.array([0, 1, 2], np.int32))
                with self.assertRaisesRegex(ValueError, 'launch list was modified'):
                    configure_frozen_connects(solver, metadata)

    def test_changed_metadata_or_source_cannot_reuse_prior_filtered_list(self):
        for mutation in ('empty', 'removed', 'metadata', 'source_values', 'source_object'):
            with self.subTest(mutation=mutation):
                model, metadata = self.fixture(); solver = self.fake_gpu(model)
                configure_frozen_connects(solver, metadata)
                changed = copy.deepcopy(metadata)
                if mutation == 'empty': changed['active_window']['frozen_attachment_names'] = []
                elif mutation == 'removed': changed.pop('active_window')
                elif mutation == 'metadata': changed['diagnostic_label'] = 'different source metadata'
                elif mutation == 'source_values': model.body_mass[model.body('dynamic').id] += .001
                else: solver.mj_model = self.fixture()[0]
                with self.assertRaisesRegex(ValueError, 'source/metadata identity mismatch'):
                    configure_frozen_connects(solver, changed)

    def test_no_active_window_is_noop_even_for_cpu_or_already_started_solver(self):
        model, metadata = self.fixture()
        metadata.pop('active_window')
        solver = SimpleNamespace(mj_model=model, use_mujoco_cpu=True, _step=7)
        self.assertEqual(frozen_connect_ids(model, metadata), [])
        # No GPU fields are provided: a legacy run must never inspect them.
        self.assertEqual(configure_frozen_connects(solver, metadata),
            dict(enabled=False, omitted_static_connect_ids=[]))

    def test_configuration_requires_pre_step_gpu_and_complete_exact_launch_identity(self):
        model, metadata = self.fixture()
        for cpu, step in ((True, 0), (False, 1)):
            solver = self.fake_gpu(model)
            solver.use_mujoco_cpu, solver._step = cpu, step
            with self.subTest(cpu=cpu, step=step), self.assertRaisesRegex(ValueError, 'before stepping'):
                configure_frozen_connects(solver, metadata)
        solver = self.fake_gpu(model, addresses=(0, 2))
        with self.assertRaisesRegex(ValueError, 'absent from GPU launch list'):
            configure_frozen_connects(solver, metadata)


if __name__ == '__main__':
    unittest.main()
