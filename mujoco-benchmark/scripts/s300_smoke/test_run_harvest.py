"""Dropped-fruit scope and conservative physics-rate regressions."""
from types import SimpleNamespace
import unittest

import mujoco as mj
import numpy as np

from run_harvest import Session, phase_frequency, run


def policy_fixture(enabled=False):
    model = mj.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom name="farm_ground" type="plane" size="1 1 .1"/>
      <geom name="farm_gutter" type="box" size=".1 .1 .1" pos="0 0 2"/>
      <geom name="farm_support" type="box" size=".1 .1 .1" pos="0 0 3"/>
      <body name="Truss_00_Fruit_03"><geom name="fruit" type="sphere" size=".01"/></body>
      <body name="Truss_01_Fruit_03"><geom name="other_fruit" type="sphere" size=".01"/></body>
      <body name="Truss_00_Pedicel_distal_09_0"><geom name="own_pedicel" type="sphere" size=".001"/></body>
      <body name="Truss_01_Pedicel_distal_09_0"><geom name="other_pedicel" type="sphere" size=".001"/></body>
      <body name="Tool_fixed"><geom name="tool" type="sphere" size=".01"/></body>
      <body name="Robot_arm"><geom name="arm" type="sphere" size=".01"/></body>
      <body name="Robot_S300Carrier"><geom name="carrier" type="sphere" size=".01"/></body>
    </worldbody></mujoco>''')
    session = Session.__new__(Session)
    session.model = model
    session.data = SimpleNamespace(eq_active=np.array([False, True]))
    session.at = SimpleNamespace(ids=np.array([model.body('Truss_00_Fruit_03').id,
                                              model.body('Truss_01_Fruit_03').id]),
                                 eqs=np.array([0, 1]))
    session.meta = dict(tools=['Tool_fixed'], farm=dict(structural_inventory=[
        dict(name='farm_gutter', classification='gutter'),
        dict(name='farm_support', classification='support')]))
    session.released_gutter_diagnostic = enabled
    return session


def allowed(session, other_body, other_geom, solver_start=None):
    fruit = session.model.body('Truss_00_Fruit_03').id
    bodies = [fruit, session.model.body(other_body).id]
    geoms = [session.model.geom('fruit').id, session.model.geom(other_geom).id]
    return session.released_contact_allowed(bodies, geoms, solver_start)


class ReleasedContactPolicyTests(unittest.TestCase):
    def test_released_gutter_exception_requires_explicit_opt_in(self):
        session = policy_fixture(False)
        self.assertFalse(allowed(session, 'world', 'farm_gutter'))
        session.released_gutter_diagnostic = True
        self.assertTrue(allowed(session, 'world', 'farm_gutter'))
        session.released_gutter_diagnostic = False
        self.assertFalse(allowed(session, 'world', 'farm_gutter'))

    def test_attached_fruit_and_same_step_break_remain_strict(self):
        session = policy_fixture(True)
        self.assertFalse(allowed(session, 'world', 'farm_gutter', np.array([True, True])))
        session.data.eq_active[0] = True
        self.assertFalse(allowed(session, 'world', 'farm_gutter'))
        self.assertFalse(allowed(session, 'Tool_fixed', 'tool'))

    def test_gutter_arm_and_other_structure_are_not_authorized(self):
        session = policy_fixture(True)
        self.assertFalse(allowed(session, 'Robot_arm', 'arm'))
        self.assertFalse(allowed(session, 'world', 'farm_support'))
        self.assertFalse(session.released_contact_allowed(
            [session.model.body('Robot_arm').id, 0],
            [session.model.geom('arm').id, session.model.geom('farm_gutter').id]))

    def test_released_own_truss_is_allowed_but_other_truss_remains_strict(self):
        session = policy_fixture(True)
        self.assertTrue(allowed(session, 'Truss_00_Pedicel_distal_09_0', 'own_pedicel'))
        self.assertFalse(allowed(session, 'Truss_01_Pedicel_distal_09_0', 'other_pedicel'))
        self.assertFalse(allowed(session, 'Truss_01_Fruit_03', 'other_fruit'))

    def test_existing_drop_exceptions_keep_physical_colliders_enabled(self):
        session = policy_fixture(False)
        masks = (session.model.geom_contype.copy(), session.model.geom_conaffinity.copy())
        self.assertTrue(allowed(session, 'world', 'farm_ground'))
        self.assertTrue(allowed(session, 'Tool_fixed', 'tool'))
        self.assertTrue(allowed(session, 'Robot_S300Carrier', 'carrier'))
        session.released_gutter_diagnostic = True
        self.assertTrue(allowed(session, 'world', 'farm_gutter'))
        np.testing.assert_array_equal(session.model.geom_contype, masks[0])
        np.testing.assert_array_equal(session.model.geom_conaffinity, masks[1])


class PhysicsFrequencyTests(unittest.TestCase):
    def test_all_belt_cycles_and_unknown_future_phases_keep_contact_rate(self):
        for phase in ('approach', 'belt_start_open', 'close', 'cycle_open',
                      'cycle_open_hold', 'cycle_reclose', 'belt_finish',
                      'cycle_open_final', 'cycle_open_final_hold', 'withdraw',
                      'new_future_contact_phase'):
            with self.subTest(phase=phase):
                self.assertEqual(phase_frequency(phase, 240, 60), 240)

    def test_only_declared_transfer_phases_use_free_rate(self):
        for phase in ('settle', 'idle', 'rail_move', 'rail_settle',
                      'prepare_lift_branch', 'prepare_joint_branch',
                      'return_lift_branch', 'return_joint_branch', 'hold'):
            with self.subTest(phase=phase):
                self.assertEqual(phase_frequency(phase, 240, 60), 60)
                self.assertEqual(phase_frequency(phase, 240, None), 240)

    def test_invalid_free_rates_are_rejected_before_loading_or_creating_output(self):
        for rate in (0, -1, 241, float('inf'), float('nan')):
            with self.subTest(rate=rate), self.assertRaisesRegex(ValueError, 'Free-motion frequency'):
                run('/nonexistent/model', '/nonexistent/output', hz=240, free_hz=rate)


if __name__ == '__main__':
    unittest.main()
