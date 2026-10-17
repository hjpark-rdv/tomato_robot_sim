"""Frozen inactive fruit are attached obstacles, not dynamic release targets.

All Warp launches explicitly use CPU. These are controller/identity checks,
not a CUDA speed test or a claim of full active-window physics equivalence.
"""
import copy
import unittest

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation
import warp as wp

from newton_kernels import (attachment_damping_addresses, attachment_torques,
                            release_attachments)


def array(value, dtype=float):
    return wp.array(np.asarray(value), dtype=dtype, device='cpu')


class ActiveWindowControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()

    def fixture(self):
        # The final hinge DOF is a sentinel for a mistaken damping[-1] write.
        model = mj.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
          <worldbody>
            <body name="anchor"><geom type="sphere" size=".01"/></body>
            <body name="dynamic" pos=".1 0 0"><freejoint/>
              <geom type="sphere" size=".01" mass=".1"/></body>
            <body name="frozen" pos=".2 0 0">
              <geom type="sphere" size=".01" mass=".1"/></body>
            <body name="sentinel" pos=".3 0 0"><joint name="last"/>
              <geom type="sphere" size=".01" mass=".1"/></body>
          </worldbody><equality>
            <connect name="dynamic_attachment" body1="anchor" body2="dynamic" anchor="0 0 0"/>
            <connect name="frozen_attachment" body1="anchor" body2="frozen" anchor="0 0 0"/>
          </equality></mujoco>''')
        metadata = dict(attachments=[dict(name='dynamic'), dict(name='frozen')],
            active_window=dict(frozen_attachment_names=['frozen']))
        return model, metadata

    def test_free_joint_maps_rotational_damping_and_only_explicit_frozen_static_uses_sentinel(self):
        model, metadata = self.fixture()
        before = model.dof_damping.copy()
        address = attachment_damping_addresses(model, metadata)
        joint = int(model.body_jntadr[model.body('dynamic').id])
        np.testing.assert_array_equal(address, [model.jnt_dofadr[joint] + 3, -1])
        self.assertEqual(address.dtype, np.int32)
        self.assertEqual(int(model.body_jntnum[model.body('frozen').id]), 0)
        np.testing.assert_array_equal(model.dof_damping, before)

    def test_unknown_frozen_unmarked_static_and_frozen_dynamic_are_rejected(self):
        model, metadata = self.fixture()
        cases = []
        unknown = copy.deepcopy(metadata)
        unknown['active_window']['frozen_attachment_names'].append('unknown')
        cases.append((unknown, 'Unknown frozen'))
        unmarked = copy.deepcopy(metadata)
        unmarked.pop('active_window')
        cases.append((unmarked, 'original free joint'))
        frozen_dynamic = copy.deepcopy(metadata)
        frozen_dynamic['active_window']['frozen_attachment_names'].append('dynamic')
        cases.append((frozen_dynamic, 'still has a dynamic joint'))
        nonfree = dict(attachments=[dict(name='sentinel')])
        cases.append((nonfree, 'original free joint'))
        for value, reason in cases:
            with self.subTest(reason=reason), self.assertRaisesRegex(ValueError, reason):
                attachment_damping_addresses(model, value)

    def test_frozen_has_no_elastic_or_implicit_torque_but_stays_attached(self):
        model, metadata = self.fixture()
        addresses = attachment_damping_addresses(model, metadata)
        bodies = np.array([model.body(spec['name']).id for spec in metadata['attachments']])
        parents = np.full(2, model.body('anchor').id)
        quats = np.tile([1., 0., 0., 0.], (1, model.nbody, 1))
        quats[0, bodies] = Rotation.from_rotvec([[.2, 0., 0.], [.6, .3, .1]]).as_quat()[:, [3, 0, 1, 2]]
        velocities = np.zeros((1, model.nbody, 6))
        velocities[0, bodies[0], 0] = .2
        velocities[0, bodies[1], :3] = 99.
        torque = wp.full((1, 2), wp.vec3(88.), dtype=wp.vec3, device='cpu')
        implicit = wp.full_like(torque, wp.vec3(77.))
        magnitude = wp.full((1, 2), 66., float, device='cpu')
        attached = wp.zeros((1, 2), bool, device='cpu')
        eq = array([[True, True]], bool)
        wp.launch(attachment_torques, (1, 2), inputs=[
            array(quats, wp.quat), array(velocities, wp.spatial_vector), eq,
            array([0, 1], int), array(bodies, int), array(parents, int),
            array(addresses, int), array(np.tile([1., 0., 0., 0.], (2, 1)), wp.quat),
            array(np.tile([1., 0., 0., 0.], (2, 1)), wp.quat),
            array(np.ones((2, 3)), wp.vec3), array([.5, .5]), array([0., 0.]),
            torque, implicit, magnitude, attached], device='cpu')
        np.testing.assert_allclose(torque.numpy()[0, 0], [-.3, 0., 0.], atol=1e-6)
        np.testing.assert_allclose(implicit.numpy()[0, 0], [.1, 0., 0.], atol=1e-7)
        np.testing.assert_array_equal(torque.numpy()[0, 1], 0.)
        np.testing.assert_array_equal(implicit.numpy()[0, 1], 0.)
        self.assertEqual(float(magnitude.numpy()[0, 1]), 0.)
        np.testing.assert_array_equal(attached.numpy(), [[True, True]])
        np.testing.assert_array_equal(eq.numpy(), [[True, True]])

    def test_high_force_releases_dynamic_only_and_preserves_frozen_and_last_dof(self):
        model, metadata = self.fixture()
        addresses = attachment_damping_addresses(model, metadata)
        eq = array([[True, True]], bool)
        original_damping = np.arange(11., 11. + model.nv)[None, :]
        damping = array(original_damping)
        broken = array([[-1, -1]], int)
        forces = wp.zeros((1, 3, 2), float, device='cpu')
        torques = wp.zeros_like(forces)
        wp.launch(release_attachments, (1, 2), inputs=[
            array([1], int), .01, array([model.body('dynamic').id, model.body('frozen').id], int),
            array([0, 1], int), array(addresses, int), array([[True, True]], bool),
            array([[.001, 999.]]), array([3., 3.]), array([.02, .02]),
            array([6], int), array([[0, 0, 0, 0, 0, 0]], int),
            array([[0, 0, 0, 1, 1, 1]], int), array([[3., 4., 0., 300., 400., 0.]]),
            eq, damping, broken, forces, torques], device='cpu')
        np.testing.assert_array_equal(eq.numpy(), [[False, True]])
        np.testing.assert_array_equal(broken.numpy(), [[1, -1]])
        expected = original_damping.copy()
        expected[0, addresses[0]:addresses[0] + 3] = 0.
        np.testing.assert_array_equal(damping.numpy(), expected)
        self.assertEqual(float(damping.numpy()[0, -1]), original_damping[0, -1])
        np.testing.assert_allclose(forces.numpy()[0, 1], [5., 500.], atol=1e-6)
        np.testing.assert_allclose(torques.numpy()[0, 1], [.001, 999.], atol=1e-6)


if __name__ == '__main__':
    unittest.main()
