"""CPU controls for preserving native physics inside Newton SolverMuJoCo."""

from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import mujoco
import newton
import numpy as np
import warp as wp

from newton_mjcf import SourceModelSolverMuJoCo, create_newton_mjcf, model_array_digest


XML = """<mujoco model="preservation_control">
  <option timestep="0.0033333333333333335" integrator="discrete" solver="Newton" iterations="31" cone="elliptic">
    <flag autoreset="disable" filterparent="disable"/>
  </option>
  <default><geom friction=".42 .007 .0003" gap=".0005" solref=".003 1.2"/></default>
  <worldbody>
    <geom name="landing" type="plane" size=".5 .5 .1" pos="0 0 1.065"/>
    <body name="stem" pos="0 0 1">
      <joint name="hinge" axis="0 1 0" damping=".07" armature=".0002" stiffness="2"/>
      <geom name="branch" type="capsule" size=".02 .1" mass="1" contype="0" conaffinity="0"/>
      <site name="neck" pos="0 0 .1"/>
    </body>
    <body name="fruit" pos="0 0 1.1">
      <freejoint name="fruit_free"/>
      <geom name="fruit_geom" type="sphere" size=".03" mass=".1"/>
      <site name="fruit_neck"/>
    </body>
  </worldbody>
  <contact><exclude body1="stem" body2="fruit"/></contact>
  <equality><connect name="attachment" site1="neck" site2="fruit_neck" solref=".002 1" solimp=".999 .9999 .001"/></equality>
  <actuator><position name="finite_motor" joint="hinge" kp="20" kv="3" forcelimited="true" forcerange="-2 2"/></actuator>
</mujoco>"""


class TestNewtonMJCF(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "scene.xml"
        self.path.write_text(XML)

    def create(self, **kwargs):
        return create_newton_mjcf(self.path, device="cpu", use_mujoco_cpu=True, **kwargs)

    def test_source_arrays_options_actuators_and_site_equalities_preserved(self):
        original = mujoco.MjModel.from_xml_path(str(self.path))
        digest = model_array_digest(original)
        run = self.create()
        self.assertEqual(model_array_digest(run.solver.mj_model), digest)
        self.assertEqual(run.solver.mj_model.nu, 1)
        self.assertEqual(run.solver.mj_model.neq, 1)
        self.assertEqual(run.solver.mj_model.nexclude, 1)
        self.assertEqual(run.evidence["mappings"]["equality"], [[0]])
        self.assertEqual(run.evidence["mappings"]["actuator"], [0])
        self.assertIs(SourceModelSolverMuJoCo.step, newton.solvers.SolverMuJoCo.step)
        self.assertEqual(run.solver.update_data_interval, 0)

    def test_inherited_newton_steps_match_native_cpu_with_contact_and_release(self):
        run = self.create()
        baseline = mujoco.MjModel.from_xml_path(str(self.path))
        data = mujoco.MjData(baseline)
        target = np.asarray([.035], dtype=np.float32)
        run.control.mujoco.ctrl.assign(target)
        data.ctrl[:] = target
        for index in range(20):
            if index == 8:
                run.solver.mj_data.eq_active[0] = False
                data.eq_active[0] = False
            run.step(baseline.opt.timestep)
            mujoco.mj_step(baseline, data)
            np.testing.assert_array_equal(run.solver.mj_data.qpos, data.qpos)
            np.testing.assert_array_equal(run.solver.mj_data.qvel, data.qvel)
        self.assertGreater(data.ncon, 0, "Released fruit must actually contact the landing plane")
        mujoco.mj_kinematics(baseline, data)
        body_map = run.evidence["mappings"]["body"][0]
        poses = run.state_0.body_q.numpy()
        for native_body, body in enumerate(body_map):
            if body >= 0:
                np.testing.assert_allclose(poses[body, :3], data.xpos[native_body], atol=1e-7)

    def test_runtime_source_configuration_applies_before_upload(self):
        def configure(model):
            model.dof_damping[0] = .22

        run = self.create(configure_source=configure)
        self.assertEqual(run.solver.mj_model.dof_damping[0], .22)
        self.assertIsNone(run.solver.mjw_model)
        self.assertIsNone(run.solver.mjw_data)
        self.assertTrue(run.evidence["gpu_upload_compatibility"]["not_applicable_cpu"])
        self.assertEqual(run.evidence["gpu_upload_compatibility"]["unsupported_disableflags_removed"],
                         [])

    def test_runtime_raw_state_is_not_overwritten_from_newton_metadata(self):
        run = self.create()
        run.solver.mj_data.qvel[-1] = .25
        run.step(run.solver.mj_model.opt.timestep)
        self.assertNotEqual(float(run.solver.mj_data.qvel[-1]), 0.0)
        with self.assertRaisesRegex(RuntimeError, "authoritative"):
            run.solver._update_mjc_data(None, None)

    def test_variable_step_sizes_match_native_without_hidden_fixed_dt(self):
        run = self.create()
        baseline = mujoco.MjModel.from_xml_path(str(self.path))
        data = mujoco.MjData(baseline)
        intervals = [1. / 60.] * 4 + [1. / 300.] * 8 + [1. / 60.] * 2
        for dt in intervals:
            baseline.opt.timestep = dt
            run.step(dt)
            mujoco.mj_step(baseline, data)
            np.testing.assert_array_equal(run.solver.mj_data.qpos, data.qpos)
            np.testing.assert_array_equal(run.solver.mj_data.qvel, data.qvel)
        self.assertAlmostEqual(run.solver.mj_data.time, sum(intervals), places=14)

    def test_refuse_implicit_model_changes(self):
        run = self.create()
        with self.assertRaisesRegex(RuntimeError, "authoritative"):
            run.solver.notify_model_changed(newton.ModelFlags.ALL)

    def test_native_observables_only_preserves_physics_contact_and_release(self):
        baseline = self.create()
        optimized = self.create(native_observables_only=True)
        self.assertFalse(baseline.evidence["native_observables_only"])
        self.assertTrue(optimized.evidence["native_observables_only"])
        self.assertFalse(optimized.evidence["newton_state_metadata_current"])
        self.assertTrue(optimized.evidence["native_kinematics_refreshed_after_each_step"])
        self.assertEqual(baseline.evidence["source_model_array_sha256"],
                         optimized.evidence["source_model_array_sha256"])
        fields = ("qpos", "qvel", "qacc", "qfrc_actuator", "qfrc_constraint", "actuator_force",
                  "ctrl", "eq_active", "xpos", "xquat", "efc_force")
        contact_steps = 0
        metadata_initial = optimized.state_0.joint_q.numpy().copy()
        for index in range(100):
            command = np.asarray([.035 if index < 60 else -.02], dtype=np.float32)
            for run in (baseline, optimized):
                run.control.mujoco.ctrl.assign(command)
                if index == 8:
                    run.solver.mj_data.eq_active[0] = False
                run.step(1. / 300 if index < 80 else 1. / 60)
            data_a, data_b = baseline.solver.mj_data, optimized.solver.mj_data
            self.assertEqual(data_a.time, data_b.time)
            self.assertEqual(data_a.ncon, data_b.ncon)
            self.assertEqual(data_a.nefc, data_b.nefc)
            contact_steps += int(data_a.ncon > 0)
            for field in fields:
                np.testing.assert_array_equal(getattr(data_a, field), getattr(data_b, field),
                                               err_msg=f"{field} changed at step {index}")
            np.testing.assert_array_equal(data_a.contact.dist, data_b.contact.dist)
        self.assertGreater(contact_steps, 0)
        # Stale Newton metadata is explicit; physics must never consume it in
        # this mode. Native qpos/poses above remain fresh at every comparison.
        np.testing.assert_array_equal(optimized.state_0.joint_q.numpy(), metadata_initial)

    def test_native_observables_only_rejects_metadata_feedback_and_observables(self):
        run = self.create(native_observables_only=True)
        before = float(run.solver.mj_data.time)
        with self.assertRaisesRegex(RuntimeError, "cannot populate Newton observables"):
            run.solver.step(run.state_0, run.state_1, run.control, None,
                            run.solver.mj_model.opt.timestep, observables=object())
        self.assertEqual(run.solver.mj_data.time, before)
        with self.assertRaisesRegex(RuntimeError, "cannot populate Newton observables"):
            run.solver._update_newton_state(run.model, run.state_1, run.solver.mj_data,
                                            run.state_0, observables=object())
        run.solver.update_data_interval = 1
        with self.assertRaisesRegex(RuntimeError, "update_data_interval=0"):
            run.solver._update_newton_state(run.model, run.state_1, run.solver.mj_data, run.state_0)

    def test_native_callback_control_is_opt_in_and_rejects_cpu_backend(self):
        run = self.create()
        self.assertFalse(run.evidence["native_control_callback"])
        self.assertFalse(run.evidence["native_control_callback_writes_every_actuator"])
        run.control.mujoco.ctrl.assign(np.asarray([.025], dtype=np.float32))
        run.solver._apply_mjc_control(run.model, run.state_0, run.control, run.solver.mj_data)
        np.testing.assert_array_equal(run.solver.mj_data.ctrl, np.asarray([.025], dtype=np.float32))
        with self.assertRaisesRegex(ValueError, "requires the MuJoCo Warp GPU backend"):
            self.create(native_control_callback=True)

    def test_native_callback_control_skips_metadata_copy_with_explicit_owner(self):
        # These ownership checks use CPU data and a fake GPU callback container;
        # the test does not initialize or launch a CUDA workload.
        run = self.create()
        run.solver.mj_data.ctrl[:] = -.015
        callback_model = SimpleNamespace(callback=SimpleNamespace(control=lambda m, d: None),
                                         opt=SimpleNamespace(disableflags=0))
        unreadable_metadata = SimpleNamespace()
        with mock.patch.object(run.solver, "native_control_callback", True), \
             mock.patch.object(run.solver, "use_mujoco_cpu", False), \
             mock.patch.object(run.solver, "mjw_model", callback_model):
            run.solver._apply_mjc_control(run.model, run.state_0, unreadable_metadata,
                                          run.solver.mj_data)
            np.testing.assert_array_equal(run.solver.mj_data.ctrl, [-.015])

    def test_native_callback_control_guard_runs_before_physics(self):
        run = self.create()
        initial_time = float(run.solver.mj_data.time)
        callback_model = SimpleNamespace(callback=SimpleNamespace(control=None),
                                         opt=SimpleNamespace(disableflags=0))
        with mock.patch.object(run.solver, "native_control_callback", True), \
             mock.patch.object(run.solver, "use_mujoco_cpu", False), \
             mock.patch.object(run.solver, "mjw_model", callback_model):
            with self.assertRaisesRegex(RuntimeError, "installed control callback"):
                run.step(run.solver.mj_model.opt.timestep)
            self.assertEqual(float(run.solver.mj_data.time), initial_time)
            callback_model.callback.control = lambda m, d: None
            callback_model.opt.disableflags = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
            with self.assertRaisesRegex(RuntimeError, "ACTUATION is disabled"):
                run.step(run.solver.mj_model.opt.timestep)
            self.assertEqual(float(run.solver.mj_data.time), initial_time)

    def test_deterministic_mode_is_explicit_enum_and_preserves_source_model(self):
        ordinary = self.create()
        deterministic = self.create(deterministic=wp.DeterministicMode.RUN_TO_RUN)
        self.assertEqual(ordinary.evidence["deterministic_mode"], "NOT_GUARANTEED")
        self.assertEqual(deterministic.evidence["deterministic_mode"], "RUN_TO_RUN")
        self.assertEqual(ordinary.evidence["source_model_array_sha256"],
                         deterministic.evidence["source_model_array_sha256"])
        self.assertIs(SourceModelSolverMuJoCo.step, newton.solvers.SolverMuJoCo.step)
        with self.assertRaisesRegex(ValueError, "warp.DeterministicMode enum"):
            self.create(deterministic=True)

    def test_deterministic_gpu_compilation_restores_dynamic_record_bound(self):
        run = self.create(deterministic=wp.DeterministicMode.RUN_TO_RUN)
        # No GPU work: the bound is a host-side function of the source topology
        # and allocated constraint capacity. Both compiler hooks are mocked.
        with mock.patch.object(run.solver, "mjw_data", SimpleNamespace(njmax=31)), \
             mock.patch.object(run.solver, "_set_mujoco_warp_module_options") as options, \
             mock.patch.object(run.solver, "_prepare_generated_kernels") as generated:
            run.solver._configure_native_gpu_compilation()
            self.assertEqual(run.solver._deterministic_max_records, 31)
            options.assert_called_once_with()
            generated.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
