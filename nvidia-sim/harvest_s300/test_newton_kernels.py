"""CPU Warp checks against the existing native MuJoCo harvest mechanics.

All launches select device='cpu'. These tests validate controller algebra and
per-world mutable storage, not CUDA speed or full integration equivalence.
"""
from pathlib import Path
import sys
import unittest

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation
import warp as wp
from mujoco_warp._src.types import OverflowType, vec5, vec6

import newton_kernels as kernels

_SMOKE = Path(__file__).resolve().parents[2] / 'mujoco-benchmark' / 'scripts' / 's300_smoke'
sys.path.insert(0, str(_SMOKE))
from run_smoke import Attachments


def array(value, dtype=float):
    return wp.array(np.asarray(value), dtype=dtype, device='cpu')


class NewtonKernelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()

    def fixture(self):
        xml = '''<mujoco><option gravity="0 0 0"/><worldbody>
          <body name="parent"><freejoint/><geom type="sphere" size=".01" mass="1"/></body>
          <body name="fruit0" pos=".1 0 0"><freejoint/><geom type="sphere" size=".01" mass=".1"/></body>
          <body name="fruit1" pos=".2 0 0"><freejoint/><geom type="sphere" size=".01" mass=".1"/></body>
          <body name="fruit2" pos=".3 0 0"><freejoint/><geom type="sphere" size=".01" mass=".1"/></body>
          </worldbody><equality>
          <connect name="fruit0_attachment" body1="parent" body2="fruit0" anchor="0 0 0"/>
          <connect name="fruit1_attachment" body1="parent" body2="fruit1" anchor="0 0 0"/>
          <connect name="fruit2_attachment" body1="parent" body2="fruit2" anchor="0 0 0"/>
          </equality></mujoco>'''
        model = mj.MjModel.from_xml_string(xml)
        specs = [dict(name=f'fruit{i}', anchor='parent',
                      local_rot0=Rotation.from_rotvec([.2, -.1, .15]).as_quat().tolist(),
                      local_rot1=Rotation.from_rotvec([-.1, .07 * (i+1), .05]).as_quat().tolist(),
                      stiffness=[.3, .2, .1], damping=.017,
                      deadband_deg=10., force_n=3., torque_nm=.02) for i in range(3)]
        reference = Attachments(model, specs)
        return model, specs, reference

    def compare_torque(self, model, specs, reference, quaternions, velocities, active):
        worlds = len(quaternions)
        data = mj.MjData(model)
        expected_xfrc, expected_magnitude = [], []
        for w in range(worlds):
            data.xquat[:] = quaternions[w]
            data.cvel[:] = velocities[w]
            data.eq_active[:] = active[w]
            data.xfrc_applied[:] = 0.
            magnitude, _ = reference.apply(data)
            expected_xfrc.append(data.xfrc_applied.copy())
            expected_magnitude.append(magnitude)
        bodies = array(reference.ids, int)
        parents = array(reference.parents, int)
        torque = wp.zeros((worlds, len(specs)), wp.vec3, device='cpu')
        implicit = wp.zeros_like(torque)
        magnitude = wp.zeros((worlds, len(specs)), float, device='cpu')
        attached = wp.zeros((worlds, len(specs)), bool, device='cpu')
        applied = wp.zeros((worlds, model.nbody), wp.spatial_vector, device='cpu')
        wp.launch(kernels.attachment_torques, torque.shape, inputs=[
            array(quaternions, wp.quat), array(velocities, wp.spatial_vector), array(active, bool),
            array(reference.eqs, int), bodies, parents,
            array(kernels.attachment_damping_addresses(model, {'attachments': specs}), int),
            array(np.asarray([s['local_rot0'] for s in specs])[:, [3, 0, 1, 2]], wp.quat),
            array(np.asarray([s['local_rot1'] for s in specs])[:, [3, 0, 1, 2]], wp.quat),
            array([s['stiffness'] for s in specs], wp.vec3), array([s['damping'] for s in specs]),
            array(np.deg2rad([s['deadband_deg'] for s in specs])),
            torque, implicit, magnitude, attached], device='cpu')
        wp.launch(kernels.apply_torques, (worlds, model.nbody), inputs=[
            bodies, parents, torque, implicit, applied], device='cpu')
        np.testing.assert_allclose(applied.numpy(), expected_xfrc, atol=2e-7, rtol=3e-5)
        np.testing.assert_allclose(magnitude.numpy(), expected_magnitude, atol=2e-7, rtol=3e-5)
        np.testing.assert_array_equal(attached.numpy(), active)
        return torque.numpy(), implicit.numpy(), applied.numpy()

    def test_attachment_torque_matches_reference_wxyz_relative_damping_and_shared_reaction(self):
        model, specs, reference = self.fixture()
        rng = np.random.default_rng(20261016)
        quaternions = Rotation.from_rotvec(rng.normal(0, .55, (7*model.nbody, 3))).as_quat()
        quaternions = quaternions[:, [3, 0, 1, 2]].reshape(7, model.nbody, 4)
        velocities = rng.normal(0, .9, (7, model.nbody, 6))
        active = np.ones((7, model.neq), bool)
        active[1, 1] = False
        active[3, :] = False
        torque, implicit, applied = self.compare_torque(
            model, specs, reference, quaternions, velocities, active)
        np.testing.assert_array_equal(torque[3], 0.)
        np.testing.assert_array_equal(implicit[3], 0.)
        np.testing.assert_array_equal(applied[:, :, :3], 0.)
        # Explicit torques sum to the self-damping cancellation only. MuJoCo's
        # implicit DOF damping supplies its complementary physical contribution.
        np.testing.assert_allclose(applied[:, :, 3:].sum(axis=1), implicit.sum(axis=1), atol=2e-7)

    def test_attachment_deadband_is_axiswise_and_inactive_body_is_force_free(self):
        model, specs, _ = self.fixture()
        for spec in specs:
            spec['local_rot0'] = spec['local_rot1'] = [0., 0., 0., 1.]
        reference = Attachments(model, specs)
        quaternions = np.tile([1., 0., 0., 0.], (2, model.nbody, 1)).astype(float)
        for w, angle in enumerate((9., 11.)):
            quat = Rotation.from_rotvec(np.deg2rad([angle, 0., 0.])).as_quat()[[3, 0, 1, 2]]
            quaternions[w, reference.ids] = quat
        active = np.ones((2, 3), bool)
        active[1, 2] = False
        torque, _, _ = self.compare_torque(model, specs, reference, quaternions,
                                         np.zeros((2, model.nbody, 6)), active)
        np.testing.assert_allclose(torque[0], 0., atol=1e-8)
        np.testing.assert_allclose(torque[1, :2, 0], -.3 * np.deg2rad(1.), atol=1e-7)
        np.testing.assert_array_equal(torque[1, 2], 0.)

    def test_finite_actuator_bias_and_preload_are_per_world(self):
        qfrc = wp.full((2, 5), -99., float, device='cpu')
        ctrl = wp.full((2, 4), -7., float, device='cpu')
        bias = np.array([[0., 1., 2., 3., 4.], [0., 2., 4., 6., 8.]])
        commands = np.array([[1., 2.], [10., 20.]])
        wp.launch(kernels.set_input, (2, 5), inputs=[
            array([.1, .2, .3, .4, .5]), qfrc, array(commands), array([1], int),
            array([3, 1], int), array([4, 2], int), array([2., 4.]), array(bias), ctrl], device='cpu')
        np.testing.assert_allclose(qfrc.numpy(), np.tile([.1, .2, .3, .4, .5], (2, 1)))
        expected = np.full((2, 4), -7.)
        expected[:, 3] = 10. + bias[:, 4] / 2.
        expected[:, 1] = 20. + bias[:, 2] / 4.
        np.testing.assert_allclose(ctrl.numpy(), expected)

    def test_belt_surface_vectors_use_geom_frame_and_both_worlds(self):
        surface = wp.zeros((2, 5), vec6, device='cpu')
        vectors = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
        wp.launch(kernels.belt_velocities, (2, 3), inputs=[
            array([1], int), array([[0., 0.], [-.15, .2]]),
            array([1, 3, 4], int), array([0, 1, 0], int), array(vectors, wp.vec3), surface], device='cpu')
        expected = np.zeros((2, 5, 6))
        expected[:, [1, 3, 4], :3] = vectors * np.array([-.15, .2, -.15])[:, None]
        np.testing.assert_allclose(surface.numpy(), expected, atol=1e-8)

    def test_release_uses_force_norm_and_torque_and_does_not_change_other_world_damping(self):
        eq = np.array([[True, True], [True, True], [False, False]])
        attached = eq.copy()
        # w0: force release on fruit0 and torque release on fruit1.
        # w1: exactly at thresholds, strict '>' must not release.
        # w2: already detached; large stale forces may be logged, not re-release.
        efc_force = np.array([[3., 4., 0., .1, 0., 0., 1000.],
                              [3., 0., 0., 3., 0., 0., 1000.],
                              [30., 40., 0., 30., 40., 0., 1000.]])
        types = np.tile([0, 0, 0, 0, 0, 0, 6], (3, 1))
        object_ids = np.tile([0, 0, 0, 1, 1, 1, 0], (3, 1))
        torque = np.array([[0., .021], [.02, .02], [100., 100.]])
        damping_np = np.tile(np.arange(1., 13.), (3, 1))
        damping = array(damping_np)
        eq_wp = array(eq, bool)
        break_np = np.array([[-1, -1], [-1, -1], [1, 2]], np.int32)
        broken = array(break_np, int)
        force_log = wp.zeros((3, 5, 2), float, device='cpu')
        torque_log = wp.zeros_like(force_log)
        wp.launch(kernels.release_attachments, (3, 2), inputs=[
            array([3], int), .01, array([1, 2], int), array([0, 1], int),
            array([3, 9], int), array(attached, bool), array(torque), array([3., 3.]),
            array([.02, .02]), array([7, 7, 7], int), array(types, int),
            array(object_ids, int), array(efc_force), eq_wp, damping, broken,
            force_log, torque_log], device='cpu')
        expected_eq = eq.copy()
        expected_eq[0] = False
        np.testing.assert_array_equal(eq_wp.numpy(), expected_eq)
        expected_break = break_np.copy()
        expected_break[0] = 3
        np.testing.assert_array_equal(broken.numpy(), expected_break)
        expected_damping = damping_np.copy()
        expected_damping[0, 3:6] = 0.
        expected_damping[0, 9:12] = 0.
        np.testing.assert_array_equal(damping.numpy(), expected_damping)
        np.testing.assert_allclose(force_log.numpy()[:, 3], [[5., .1], [3., 3.], [50., 50.]], atol=1e-7)
        np.testing.assert_allclose(torque_log.numpy()[:, 3], torque, atol=1e-7)

    def test_contact_audit_keeps_attached_guard_and_scopes_released_exemptions(self):
        # The same fruit/support overlap remains strict while attached, becomes
        # a released-fruit diagnostic after release, and does not exempt stems.
        geometry = np.array([[1, 2], [1, 2], [1, 3], [0, 3], [1, 2], [1, 3]])
        distances = np.array([-.002, -.005, -.003, -.001, .001, -.1])
        worlds = np.array([0, 1, 1, 2, 2, 2])
        addresses = np.zeros((6, 6), np.int32)
        addresses[:, 0] = np.arange(6)
        addresses[-1, 0] = -1  # Unsolved contact must not count as physical force/penetration.
        forces = np.tile(np.arange(1., 7.), (3, 1))
        allowed = np.zeros((1, 4), bool)
        allowed[0, 2] = True
        active_max = wp.zeros((3, 8), float, device='cpu')
        released_max = wp.zeros_like(active_max)
        force_max = wp.zeros_like(active_max)
        sentinel = np.iinfo(np.uint64).max
        first = array(np.full(3, sentinel, np.uint64), wp.uint64)
        wp.launch(kernels.audit_contacts, 6, inputs=[
            array([4], int), array([6], int), 6, int(mj.mjtCone.mjCONE_ELLIPTIC),
            array(worlds, int), array(geometry, wp.vec2i), array(distances),
            array(np.tile(np.eye(3), (6, 1, 1)), wp.mat33), array(np.ones((6, 5)), vec5),
            array(np.ones(6, np.int32), int), array(addresses, int), array(np.zeros(6)),
            array(forces), array([0, 1, 2, 3], int), array([-1, 0, -1, -1], int),
            array(allowed, bool), array([[True], [False], [True]], bool),
            active_max, released_max, force_max, first], device='cpu')
        np.testing.assert_allclose(active_max.numpy()[:, 4], [.002, .003, .001], atol=1e-9)
        np.testing.assert_allclose(released_max.numpy()[:, 4], [0., .005, 0.], atol=1e-9)
        np.testing.assert_allclose(force_max.numpy()[:, 4], [1., 3., 5.], atol=1e-7)
        np.testing.assert_array_equal(first.numpy(), [
            (4 << 32) | (1 * 65536 + 2),
            (4 << 32) | (1 * 65536 + 3), sentinel])

    def health_fixture(self, qacc, *, nacon, ncollision, nefc, overflow):
        worlds = len(qacc)
        maximum = np.iinfo(np.int32).max
        state = dict(tick=array([2], int), time=array(np.arange(worlds) + 2.5),
            qacc=array(qacc), nacon=array([nacon], int), ncollision=array([ncollision], int),
            nefc=array(nefc, int), overflow=array(overflow, int),
            nacon_log=wp.zeros(5, int, device='cpu'),
            ncollision_log=wp.zeros(5, int, device='cpu'),
            nefc_log=wp.zeros((worlds, 5), int, device='cpu'),
            overflow_log=wp.zeros((worlds, 5), int, device='cpu'),
            qacc_abs_max=wp.zeros((worlds, 5), float, device='cpu'),
            time_log=wp.zeros((worlds, 5), float, device='cpu'),
            first_qacc_nonfinite=array(np.full(worlds, maximum, np.int32), int),
            first_capacity_overflow=array(np.full(worlds, maximum, np.int32), int))
        return state

    def launch_health(self, state):
        wp.launch(kernels.audit_runtime_health,
            (state['qacc'].shape[0], max(1, state['qacc'].shape[1])), inputs=[
                state['tick'], state['time'], state['qacc'], state['nacon'], state['ncollision'],
                state['nefc'], state['overflow'], 10, 8,
                state['nacon_log'], state['ncollision_log'], state['nefc_log'], state['overflow_log'],
                state['qacc_abs_max'], state['time_log'],
                state['first_qacc_nonfinite'], state['first_capacity_overflow']], device='cpu')

    def test_runtime_health_detects_nonfinite_acceleration_and_logs_backend_time(self):
        state = self.health_fixture([[1., -5.], [np.inf, 7.], [np.nan, -9.]],
            nacon=10, ncollision=10, nefc=[8, 8, 8],
            overflow=[int(OverflowType.ITERATIONS), int(OverflowType.LS_ITERATIONS), 0])
        self.launch_health(state)
        np.testing.assert_array_equal(state['first_qacc_nonfinite'].numpy(),
                                      [np.iinfo(np.int32).max, 2, 2])
        np.testing.assert_array_equal(state['first_capacity_overflow'].numpy(), np.iinfo(np.int32).max)
        np.testing.assert_allclose(state['qacc_abs_max'].numpy()[:, 2], [5., 7., 9.])
        np.testing.assert_array_equal(state['time_log'].numpy()[:, 2], [2.5, 3.5, 4.5])
        np.testing.assert_array_equal(state['nacon_log'].numpy(), [0, 0, 10, 0, 0])
        np.testing.assert_array_equal(state['ncollision_log'].numpy(), [0, 0, 10, 0, 0])
        np.testing.assert_array_equal(state['overflow_log'].numpy()[:, 2], [512, 1024, 0])

    def test_runtime_capacity_counts_and_flags_are_scoped_per_world_and_preserve_first_step(self):
        state = self.health_fixture([[1.], [2.], [3.]], nacon=5, ncollision=6,
            nefc=[8, 9, 8], overflow=[0, 0, int(OverflowType.NEFC)])
        self.launch_health(state)
        np.testing.assert_array_equal(state['first_capacity_overflow'].numpy(),
                                      [np.iinfo(np.int32).max, 2, 2])
        np.testing.assert_array_equal(state['nefc_log'].numpy()[:, 2], [8, 9, 8])
        state['tick'].assign(np.array([3], np.int32))
        state['nacon'].assign(np.array([11], np.int32))
        state['ncollision'].assign(np.array([12], np.int32))
        self.launch_health(state)
        np.testing.assert_array_equal(state['first_capacity_overflow'].numpy(), [3, 2, 2])
        np.testing.assert_array_equal(state['nacon_log'].numpy(), [0, 0, 5, 11, 0])


if __name__ == '__main__':
    unittest.main()
