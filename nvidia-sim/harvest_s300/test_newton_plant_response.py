import unittest
import mujoco as mj
import numpy as np
from newton_plant_response import sample_post_step_poses


class ResponseTimestampTests(unittest.TestCase):
    def test_saved_pose_uses_integrated_timestamp_and_retains_solved_force(self):
        model = mj.MjModel.from_xml_string('''<mujoco><option timestep="0.01" gravity="0 0 0"/>
          <worldbody><body><freejoint/><geom type="sphere" size=".01" mass=".1"/>
          </body></worldbody></mujoco>''')
        data = mj.MjData(model)
        data.qvel[0] = 1.
        mj.mj_step(model, data)
        force = data.efc_force.copy()
        position, quaternion = sample_post_step_poses(model, data, [1])
        self.assertAlmostEqual(data.time, .01)
        self.assertAlmostEqual(position[0, 0], .01)
        self.assertAlmostEqual(position[0, 0], data.qpos[0])
        np.testing.assert_array_equal(force, data.efc_force)
        np.testing.assert_allclose(quaternion[0], [1, 0, 0, 0])


if __name__ == '__main__':
    unittest.main()
