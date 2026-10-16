"""Backend-stage regressions, using real MuJoCo Warp on CPU by default.

These run the same Warp constraint assembly as CUDA without claiming GPU
performance. For an actual CUDA rerun, set NEWTON_GPU_STAGE_DEVICE=cuda:0.
The CONNECT case records an upstream velocity-cache ordering limitation; it
does not enable a speculative workaround in the harvest executor.
"""

import json
import os
from types import SimpleNamespace
import unittest

import mujoco as mj
import mujoco_warp as mw
import numpy as np
import warp as wp

from newton_kernels import HarvestGPUController
from newton_gpu_compat import install_connect_velocity_cache_refresh


BELT_XML = """<mujoco model="belt_stage_control">
  <option timestep=".0025" integrator="discrete" solver="Newton"
          jacobian="dense" cone="elliptic" tolerance="1e-8"/>
  <worldbody>
    <geom name="belt" type="plane" size="1 1 .1" friction=".7 .01 .001"/>
    <body name="fruit" pos="0 0 .049">
      <freejoint/><geom type="sphere" size=".05" mass=".1"/>
    </body>
  </worldbody>
</mujoco>"""

CONNECT_XML = """<mujoco model="connect_velocity_stage_control">
  <option timestep=".0025" integrator="discrete" solver="Newton"
          jacobian="dense" cone="elliptic" gravity="0 0 0" tolerance="1e-8"/>
  <worldbody>
    <body name="branch" pos="0 0 0">
      <freejoint/><geom type="sphere" size=".05" mass=".3"
                       contype="0" conaffinity="0"/>
    </body>
    <body name="fruit" pos="1 0 0">
      <freejoint/><geom type="sphere" size=".05" mass=".1"
                       contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality><connect name="attachment" body1="branch" body2="fruit"
                     anchor=".5 .2 .1"/></equality>
</mujoco>"""


class TestNewtonGPUStageOrder(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()
        cls.device = os.environ.get("NEWTON_GPU_STAGE_DEVICE", "cpu")
        if cls.device.startswith("cuda") and not wp.is_cuda_available():
            raise unittest.SkipTest("CUDA stage control requested without a CUDA device")

    def array(self, values, dtype=float):
        return wp.array(np.asarray(values), dtype=dtype, device=self.device)

    def model_pair(self, xml):
        model = mj.MjModel.from_xml_string(xml)
        data = mj.MjData(model)
        model.flg_surfacevel = True
        mj.mj_forward(model, data)
        with wp.ScopedDevice(self.device):
            backend_model = mw.put_model(model)
            backend_data = mw.put_data(model, data, nconmax=8, njmax=32)
        return model, data, backend_model, backend_data

    def belt_controller(self, backend_model, speeds):
        # Exercise the production pre_step method without synthesizing fruit
        # attachment metadata irrelevant to this one-contact control.
        controller = object.__new__(HarvestGPUController)
        controller.device = self.device
        controller.solver = SimpleNamespace(mjw_model=backend_model)
        controller.tick = self.array([0], int)
        controller.belts = self.array(np.c_[speeds, np.zeros(len(speeds))])
        controller.beltids = self.array([0], int)
        controller.beltside = self.array([0], int)
        controller.local_y = self.array([[0., 1., 0.]], wp.vec3)
        return controller

    def test_native_discrete_callback_belt_uses_current_command(self):
        model = mj.MjModel.from_xml_string(BELT_XML)
        model.flg_surfacevel = True
        data = mj.MjData(model)
        old_callback = mj.get_mjcb_control()
        try:
            def control(m, _d):
                m.geom_surfacevel[0, :3] = [0., .2, 0.]

            mj.set_mjcb_control(control)
            mj.mj_step(model, data)
            self.assertEqual(data.ncon, 1)
            self.assertAlmostEqual(float(np.linalg.norm(data.efc_vel[1:3])), .2, places=12)
        finally:
            mj.set_mjcb_control(old_callback)

    def test_pre_step_applies_current_belt_tick_before_contact_reference(self):
        speeds = np.array([.2, -.15, .0])
        _, _, early_model, early_data = self.model_pair(BELT_XML)
        _, _, late_model, late_data = self.model_pair(BELT_XML)
        early = self.belt_controller(early_model, speeds)
        late = self.belt_controller(late_model, speeds)
        late_model.callback.control = lambda _m, _d: late.pre_step()
        previous = 0.
        for tick, speed in enumerate(speeds):
            early.tick.assign(np.array([tick], np.int32))
            late.tick.assign(np.array([tick], np.int32))
            # No integration between these controls: identical fixed contact,
            # so only the timing of the prescribed belt command can differ.
            with wp.ScopedDevice(self.device):
                early.pre_step()
                mw.forward(early_model, early_data)
                mw.forward(late_model, late_data)
            current_vel = early_data.efc.vel.numpy()[0, :3]
            stale_vel = late_data.efc.vel.numpy()[0, :3]
            self.assertAlmostEqual(float(np.linalg.norm(current_vel[1:])), abs(speed), places=6)
            self.assertAlmostEqual(float(np.linalg.norm(stale_vel[1:])), abs(previous), places=6)
            self.assertGreater(float(np.max(np.abs(current_vel - stale_vel))), .1)
            previous = speed

    def test_connect_velocity_cache_diagnostic_identifies_stale_first_reference(self):
        model, native, backend_model, backend_data = self.model_pair(CONNECT_XML)
        velocity = np.array([1., -.4, .2, 3., -2., 4.,
                             -.6, .9, -.2, -2., 1., -3.])
        native.qvel[:] = velocity
        mj.mj_forward(model, native)
        backend_data.qvel.assign(velocity[None, :].astype(np.float32))
        with wp.ScopedDevice(self.device):
            mw.forward(backend_model, backend_data)
        first = backend_data.efc.aref.numpy()[0, :3].copy()
        with wp.ScopedDevice(self.device):
            # First forward refreshed cvel/cdof_dot. Re-evaluate the very same
            # qpos/qvel without integration, isolating cache age from motion.
            mw.forward(backend_model, backend_data)
        refreshed = backend_data.efc.aref.numpy()[0, :3].copy()
        expected = native.efc_aref[:3]
        stale_error = float(np.max(np.abs(first - expected)))
        fresh_error = float(np.max(np.abs(refreshed - expected)))
        print(json.dumps(dict(diagnostic="connect_velocity_cache", device=self.device,
                              first_reference=first.tolist(), refreshed_reference=refreshed.tolist(),
                              native_reference=expected.tolist(), stale_error=stale_error,
                              refreshed_error=fresh_error)), flush=True)
        self.assertGreater(stale_error, .1)
        np.testing.assert_allclose(refreshed, expected, rtol=2e-5, atol=2e-5)

    def test_connect_pre_constraint_refresh_matches_native_on_first_forward(self):
        model, native, backend_model, backend_data = self.model_pair(CONNECT_XML)
        velocity = np.array([1., -.4, .2, 3., -2., 4.,
                             -.6, .9, -.2, -2., 1., -3.])
        native.qvel[:] = velocity
        mj.mj_forward(model, native)
        backend_data.qvel.assign(velocity[None, :].astype(np.float32))
        before_qpos = backend_data.qpos.numpy().copy()
        before_qvel = backend_data.qvel.numpy().copy()
        before_eq = backend_data.eq_active.numpy().copy()
        previous_calls = []
        backend_model.callback.contactfilter = lambda _m, _d: previous_calls.append(True)
        evidence = install_connect_velocity_cache_refresh(backend_model, backend_data)
        self.assertTrue(evidence["previous_contactfilter_preserved"])
        self.assertFalse(evidence["contacts_modified"])
        self.assertFalse(evidence["integration_or_model_fields_modified"])
        with wp.ScopedDevice(self.device):
            mw.forward(backend_model, backend_data)
        fixed = backend_data.efc.aref.numpy()[0, :3]
        error = float(np.max(np.abs(fixed - native.efc_aref[:3])))
        print(json.dumps(dict(diagnostic="connect_velocity_cache_fixed", device=self.device,
                              fixed_reference=fixed.tolist(), max_reference_error=error,
                              contacts=int(backend_data.nacon.numpy()[0]))), flush=True)
        np.testing.assert_allclose(fixed, native.efc_aref[:3], rtol=2e-5, atol=2e-5)
        self.assertEqual(previous_calls, [True])
        self.assertEqual(int(backend_data.nacon.numpy()[0]), 0,
                         "Hook must also run for a CONNECT scene without actual contacts")
        np.testing.assert_array_equal(backend_data.qpos.numpy(), before_qpos)
        np.testing.assert_array_equal(backend_data.qvel.numpy(), before_qvel)
        np.testing.assert_array_equal(backend_data.eq_active.numpy(), before_eq)

    def test_connect_cache_refresh_install_is_idempotent(self):
        _, _, backend_model, backend_data = self.model_pair(CONNECT_XML)
        first_evidence = install_connect_velocity_cache_refresh(backend_model, backend_data)
        first_hook = backend_model.callback.contactfilter
        second_evidence = install_connect_velocity_cache_refresh(backend_model, backend_data)
        self.assertIs(backend_model.callback.contactfilter, first_hook)
        self.assertEqual(first_evidence, second_evidence)
        self.assertIsNone(first_hook.previous)

    def test_connect_pre_constraint_refresh_matches_native_sparse_first_reference(self):
        xml = CONNECT_XML.replace('jacobian="dense"', 'jacobian="sparse"')
        model, native, backend_model, backend_data = self.model_pair(xml)
        velocity = np.array([1., -.4, .2, 3., -2., 4.,
                             -.6, .9, -.2, -2., 1., -3.])
        native.qvel[:] = velocity
        backend_data.qvel.assign(velocity[None, :].astype(np.float32))
        install_connect_velocity_cache_refresh(backend_model, backend_data)
        mj.mj_forward(model, native)
        with wp.ScopedDevice(self.device):
            mw.forward(backend_model, backend_data)
        np.testing.assert_allclose(backend_data.efc.aref.numpy()[0, :3],
                                   native.efc_aref[:3], rtol=2e-5, atol=2e-5)
        # Scope is the reference-cache fix, not native float64 vs Warp float32
        # integration equivalence. qpos/qvel must remain untouched by forward.
        np.testing.assert_array_equal(backend_data.qpos.numpy()[0], native.qpos.astype(np.float32))
        np.testing.assert_array_equal(backend_data.qvel.numpy()[0], velocity.astype(np.float32))

    def test_connect_cache_refresh_refuses_unreachable_callback_configurations(self):
        _, _, backend_model, backend_data = self.model_pair(CONNECT_XML)
        backend_model.opt.run_collision_detection = False
        with self.assertRaisesRegex(ValueError, "native collision detection"):
            install_connect_velocity_cache_refresh(backend_model, backend_data)
        backend_model.opt.run_collision_detection = True
        backend_model.opt.disableflags |= mw.DisableBit.CONTACT
        with self.assertRaisesRegex(ValueError, "unreachable"):
            install_connect_velocity_cache_refresh(backend_model, backend_data)
        backend_model.opt.disableflags &= ~mw.DisableBit.CONTACT
        backend_data.naconmax = 0
        with self.assertRaisesRegex(ValueError, "positive native contact capacity"):
            install_connect_velocity_cache_refresh(backend_model, backend_data)


if __name__ == "__main__":
    unittest.main()
