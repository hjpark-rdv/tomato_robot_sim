"""Session-reset regression on CPU-backed Warp arrays; no CUDA workload."""
from types import SimpleNamespace
import unittest

import mujoco as mj
import mujoco_warp as mw
import numpy as np
import warp as wp

from newton_gpu_session import NewtonGPUSessionCache


XML = """<mujoco><option gravity="0 0 0"/>
<worldbody><body><joint name="hinge" damping=".2"/><geom type="sphere" size=".02" mass="1"/></body></worldbody>
<equality><joint name="attachment" joint1="hinge"/></equality>
<actuator><position joint="hinge" kp="10"/></actuator></mujoco>"""


class NewtonGPUSessionTests(unittest.TestCase):
    def factory(self):
        self.loads += 1
        native = mj.MjModel.from_xml_string(XML)
        native.flg_surfacevel = True
        data = mj.MjData(native)
        mj.mj_forward(native, data)
        with wp.ScopedDevice("cpu"):
            model = mw.put_model(native, batch_sizes={"dof_damping": 2, "geom_surfacevel": 2})
            device_data = mw.put_data(native, data, nworld=2, nconmax=8, njmax=32)
            # Fake only the Newton metadata shell. Public reset and native
            # kinematics are the real installed MW functions on CPU arrays.
            return SimpleNamespace(
                model=SimpleNamespace(device=wp.get_device("cpu")),
                solver=SimpleNamespace(use_mujoco_cpu=False, _step=0,
                    mj_model=native, mjw_model=model, mjw_data=device_data),
                state_0=SimpleNamespace(joint_q=wp.array([0., 0.], dtype=float)),
                state_1=SimpleNamespace(joint_q=wp.array([0., 0.], dtype=float)),
                control=SimpleNamespace(mujoco=SimpleNamespace(ctrl=wp.array([0., 0.], dtype=float))))

    def setUp(self):
        self.loads = 0
        self.cache = NewtonGPUSessionCache()
        self.signature = {"source_model_sha256": "a" * 64, "source_xml_sha256": "b" * 64,
                          "config": {"worlds": 2, "device": "cpu-test-only", "hz": 400}}
        self.initial = dict(qpos=np.array([.03], np.float32), qvel=np.array([.02], np.float32),
                            ctrl=np.array([.04], np.float32), eq_active=np.array([True]),
                            runtime_dof_damping=np.array([.2]))

    def test_reuses_model_and_fully_restores_mutable_trial_state(self):
        engine, first = self.cache.acquire(self.signature, self.factory, self.initial)
        solver = engine.solver
        model, data = solver.mjw_model, solver.mjw_data
        baseline_cvel = data.cvel.numpy().copy()
        baseline_cdof_dot = data.cdof_dot.numpy().copy()
        baseline_damping = model.dof_damping.numpy().copy()
        baseline_surface = model.geom_surfacevel.numpy().copy()
        baseline_q = data.qpos.numpy().copy()
        baseline_v = data.qvel.numpy().copy()
        model.dof_damping.fill_(0.)
        model.geom_surfacevel.fill_(3.)
        data.qpos.fill_(99.)
        data.qvel.fill_(1.)
        data.qacc.fill_(7.)
        data.qacc_warmstart.fill_(123.)
        data.cvel.fill_(2.)
        data.cdof_dot.fill_(2.)
        data.time.fill_(44.)
        data.ctrl.fill_(8.)
        data.eq_active.fill_(False)
        data.solver_niter.fill_(9)
        engine.state_0.joint_q.fill_(17.)
        engine.state_0, engine.state_1 = engine.state_1, engine.state_0
        engine.control.mujoco.ctrl.fill_(21.)
        solver._step = 125
        model.callback.control = lambda m, d: None
        again, repeated = self.cache.acquire(self.signature, self.factory, self.initial)
        self.assertIs(again, engine)
        self.assertIs(again.solver.mj_model, solver.mj_model)
        self.assertEqual(self.loads, 1)
        self.assertFalse(first["reused"])
        self.assertTrue(repeated["reused"])
        self.assertEqual(repeated["acquisition"], 2)
        self.assertEqual(repeated["model_load_wall_s"], 0.)
        self.assertGreater(repeated["initial_buffer_snapshot_bytes"], 0)
        for value in (data.time, data.qacc, data.qacc_warmstart, data.solver_niter,
                      engine.state_0.joint_q, engine.state_1.joint_q, engine.control.mujoco.ctrl):
            np.testing.assert_array_equal(value.numpy(), np.zeros_like(value.numpy()))
        np.testing.assert_array_equal(data.qpos.numpy(), baseline_q)
        np.testing.assert_array_equal(data.qvel.numpy(), baseline_v)
        np.testing.assert_array_equal(data.cvel.numpy(), baseline_cvel)
        np.testing.assert_array_equal(data.cdof_dot.numpy(), baseline_cdof_dot)
        np.testing.assert_array_equal(model.dof_damping.numpy(), baseline_damping)
        np.testing.assert_array_equal(model.geom_surfacevel.numpy(), baseline_surface)
        np.testing.assert_array_equal(data.eq_active.numpy(), [[True], [True]])
        np.testing.assert_array_equal(data.ctrl.numpy(), [[np.float32(.04)], [np.float32(.04)]])
        self.assertIsNone(model.callback.control)
        self.assertEqual(solver._step, 0)

    def test_different_source_config_or_initial_state_is_refused_before_reset(self):
        engine, _ = self.cache.acquire(self.signature, self.factory, self.initial)
        engine.solver.mjw_data.time.fill_(2.)
        changed = dict(self.signature, source_model_sha256="c" * 64)
        with self.assertRaisesRegex(ValueError, "mismatch"):
            self.cache.acquire(changed, self.factory, self.initial)
        different = dict(self.initial, qpos=np.array([.11], np.float32))
        with self.assertRaisesRegex(ValueError, "mismatch"):
            self.cache.acquire(self.signature, self.factory, different)
        np.testing.assert_array_equal(engine.solver.mjw_data.time.numpy(), [2., 2.])
        self.assertEqual(self.loads, 1)

    def test_rejects_native_cpu_engine_and_empty_identity(self):
        with self.assertRaisesRegex(ValueError, "nonempty mapping"):
            self.cache.acquire({}, self.factory, self.initial)
        engine = SimpleNamespace(solver=SimpleNamespace(use_mujoco_cpu=True))
        with self.assertRaisesRegex(ValueError, "native Warp backend"):
            self.cache.acquire(self.signature, lambda: engine, self.initial)


if __name__ == "__main__":
    unittest.main()
