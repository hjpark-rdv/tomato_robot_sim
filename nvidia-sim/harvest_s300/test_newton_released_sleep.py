import unittest

import mujoco as mj
import numpy as np

from newton_released_sleep import configure_fruit_sleep


class FruitSleepTests(unittest.TestCase):
    def test_only_isolated_fruit_tree_can_sleep(self):
        m = mj.MjModel.from_xml_string('<mujoco><worldbody><body name="plant"><joint/><geom type="sphere" size=".1"/></body><body name="fruit"><freejoint/><geom type="sphere" size=".02"/></body></worldbody></mujoco>')
        mass = m.body_mass.copy()
        collision = m.geom_contype.copy()
        proof = configure_fruit_sleep(m, {'attachments':[{'name':'fruit'}]})
        self.assertEqual(proof['fruit_tree_ids'], [1])
        self.assertEqual(m.tree_sleep_policy.tolist(), [1,2])
        np.testing.assert_array_equal(m.body_mass, mass)
        np.testing.assert_array_equal(m.geom_contype, collision)

    def test_rejects_shared_tree(self):
        m = mj.MjModel.from_xml_string('<mujoco><worldbody><body name="plant"><joint/><geom type="sphere" size=".1"/><body name="fruit"><joint/><geom type="sphere" size=".02"/></body></body></worldbody></mujoco>')
        with self.assertRaises(ValueError):
            configure_fruit_sleep(m, {'attachments':[{'name':'fruit'}]})


if __name__ == '__main__':
    unittest.main()
