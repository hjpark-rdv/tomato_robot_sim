"""Host and CPU Warp controls for the bounded CUDA determinism adapter.

CPU controls prove bound derivation, scope restoration and unmodified physics
inputs; they do not establish CUDA repeatability or performance. Set
NEWTON_GPU_STAGE_DEVICE=cuda:0 for the same small backend integration control.
"""

from contextlib import contextmanager
import os
from types import SimpleNamespace
import unittest
from unittest import mock

import mujoco as mj
import mujoco_warp as mw
from mujoco_warp._src import smooth, solver as native_solver
import numpy as np
import warp as wp

from newton_gpu_compat import install_connect_velocity_cache_refresh
from newton_gpu_deterministic import BoundedDeterministicExecution, RigidBounds, _selected_source
from test_newton_gpu_stage_order import CONNECT_XML


class FixtureSolver:
    def __init__(self, model, backend_model, backend_data):
        self.use_mujoco_cpu = False
        self._step = 0
        self._deterministic = wp.DeterministicMode.RUN_TO_RUN
        self._deterministic_max_records = 74305
        self.mj_model, self.mjw_model, self.mjw_data = model, backend_model, backend_data
        self._module_options = {
            smooth: dict(deterministic=wp.DeterministicMode.RUN_TO_RUN, deterministic_max_records=74305),
            native_solver: dict(deterministic=wp.DeterministicMode.RUN_TO_RUN, deterministic_max_records=0),
        }

    def _set_module_options(self, options, module):
        self._module_options[module] = dict(options)
        wp.set_module_options(options, module=module)

    @contextmanager
    def _scoped_deterministic_config(self):
        yield


class TestBoundedDeterministicExecution(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wp.init()
        cls.device = os.environ.get("NEWTON_GPU_STAGE_DEVICE", "cpu")

    def fixture(self):
        model = mj.MjModel.from_xml_string(CONNECT_XML.replace('jacobian="dense"', 'jacobian="sparse"'))
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        with wp.ScopedDevice(self.device):
            backend_model = mw.put_model(model)
            backend_data = mw.put_data(model, data, nconmax=8, njmax=32)
        return FixtureSolver(model, backend_model, backend_data)

    def test_broadphase_bounds_follow_actual_reservation_loops(self):
        bounds = RigidBounds(385, 901, 114, 847, 512)
        self.assertEqual(bounds.records("mujoco_warp._src.collision_driver", "_nxn_broadphase__kernel", (1, 264283))[0], 1)
        self.assertEqual(bounds.records("mujoco_warp._src.collision_driver", "_sap_broadphase__kernel", (4505,))[0], 90)

    def test_sparse_hessian_bound_includes_group_stride_and_lanes(self):
        bounds = RigidBounds(385, 901, 114, 847, 512)
        self.assertEqual(bounds.records("mujoco_warp._src.solver", "_JTDACJ_sparse__kernel", (1, 1008, 32))[0], 205)
        self.assertEqual(bounds.records("mujoco_warp._src.solver", "_JTDACJ_sparse__kernel", (1, 100, 32))[0], 1845)
        self.assertEqual(bounds.records("mujoco_warp._src.solver", "_JTDACJ_sparse__kernel", (1, 1008, 1))[0], 6555)

    def test_chain_union_uses_topology_instead_of_actuator_exists_nv_fallback(self):
        run = self.fixture()
        bounds = RigidBounds.from_solver(run)
        self.assertEqual(bounds.nv, 12)
        self.assertEqual(bounds.row_width, 12)
        self.assertEqual(bounds.group_capacity, 9)

    def test_unreviewed_feature_set_is_rejected_before_launch(self):
        run = self.fixture()
        with mock.patch.object(run, "use_mujoco_cpu", True):
            with self.assertRaisesRegex(ValueError, "Warp execution"):
                BoundedDeterministicExecution(run)
        run._step = 1
        with self.assertRaisesRegex(ValueError, "before stepping"):
            BoundedDeterministicExecution(run)
        run._step = 0
        run.mj_model.opt.cone = mj.mjtCone.mjCONE_PYRAMIDAL
        with self.assertRaisesRegex(ValueError, "elliptic"):
            BoundedDeterministicExecution(run)

    def test_scope_restores_global_launches_and_settings_even_on_error(self):
        run = self.fixture()
        old_scope = run._scoped_deterministic_config
        old_launch, old_tiled = wp.launch, wp.launch_tiled
        old_mode, old_records = wp.config.deterministic, wp.config.deterministic_max_records
        adapter = BoundedDeterministicExecution(run, scope="solver")
        try:
            with self.assertRaisesRegex(RuntimeError, "test body"):
                with run._scoped_deterministic_config():
                    self.assertIsNot(wp.launch, old_launch)
                    self.assertEqual(wp.config.deterministic_max_records, 0)
                    self.assertEqual(wp.config.deterministic, wp.DeterministicMode.NOT_GUARANTEED)
                    raise RuntimeError("test body")
            self.assertIs(wp.launch, old_launch)
            self.assertIs(wp.launch_tiled, old_tiled)
            self.assertEqual(wp.config.deterministic, old_mode)
            self.assertEqual(wp.config.deterministic_max_records, old_records)
        finally:
            adapter.close()
        self.assertEqual(run._scoped_deterministic_config, old_scope)
        self.assertEqual(run._deterministic_max_records, 74305)

    def test_solver_only_scope_does_not_claim_full_pipeline_determinism(self):
        run = self.fixture()
        adapter = BoundedDeterministicExecution(run, scope="solver")
        try:
            fake = SimpleNamespace(func=SimpleNamespace(__module__="mujoco_warp._src.collision_driver"),
                                   key="_nxn_broadphase__kernel", module=object())
            with mock.patch.object(wp, "get_module_options", return_value={}), \
                 mock.patch.object(wp, "set_module_options") as setter:
                adapter._configure(fake, (1, 264283))
            self.assertEqual(setter.call_args.args[0]["deterministic"], wp.DeterministicMode.NOT_GUARANTEED)
            self.assertFalse(adapter.evidence["full_pipeline_determinism_claimed"])
        finally:
            adapter.close()

    def test_dynamics_scope_covers_inertia_rne_and_constraint_math_without_ccd_codegen(self):
        for source in ("smooth", "support", "solver", "forward", "derivative", "passive", "constraint"):
            self.assertTrue(_selected_source("dynamics", "mujoco_warp._src." + source))
        for source in ("collision_driver", "collision_convex", "collision_primitive", "collision_core"):
            self.assertFalse(_selected_source("dynamics", "mujoco_warp._src." + source))
        run = self.fixture()
        adapter = BoundedDeterministicExecution(run, scope="dynamics")
        try:
            with run._scoped_deterministic_config():
                self.assertEqual(wp.config.deterministic, wp.DeterministicMode.NOT_GUARANTEED)
                self.assertEqual(wp.config.deterministic_max_records, 0)
            self.assertFalse(adapter.evidence["full_pipeline_determinism_claimed"])
        finally:
            adapter.close()

    def test_constraints_scope_covers_row_allocation_and_solver_without_smooth_or_ccd(self):
        self.assertTrue(_selected_source("constraints", "mujoco_warp._src.constraint"))
        self.assertTrue(_selected_source("constraints", "mujoco_warp._src.solver"))
        for source in ("smooth", "support", "passive", "collision_driver", "collision_convex"):
            self.assertFalse(_selected_source("constraints", "mujoco_warp._src." + source))
        bounds = RigidBounds(385, 901, 114, 847, 512)
        for key in ("_equality_connect__locals__kernel", "_friction_dof__locals__kernel",
                    "_limit_ball__locals__kernel", "_limit_slide_hinge__locals__kernel",
                    "_efc_contact_init__locals__kernel", "_efc_contact_jac_sparse__locals__kernel"):
            self.assertEqual(bounds.records("mujoco_warp._src.constraint", key, (1, 512))[0], 1)
        run = self.fixture()
        adapter = BoundedDeterministicExecution(run, scope="constraints")
        try:
            with run._scoped_deterministic_config():
                self.assertEqual(wp.config.deterministic, wp.DeterministicMode.NOT_GUARANTEED)
            self.assertEqual(adapter.evidence["scope"], "constraints")
        finally:
            adapter.close()

    def test_deferred_scatter_does_not_silently_break_in_kernel_ld_dependencies(self):
        run = self.fixture()
        adapter = BoundedDeterministicExecution(run, scope="dynamics")
        try:
            kernel = SimpleNamespace(func=SimpleNamespace(__module__="mujoco_warp._src.smooth"),
                                     key="_solve_LD_sparse_fused__locals__kernel", module=object())
            with self.assertRaisesRegex(ValueError, "in-kernel sparse LD solve dependencies"):
                adapter._configure(kernel, (1, 128))
        finally:
            adapter.close()

    def test_actual_backend_reference_with_bounded_scope_preserves_inputs(self):
        run = self.fixture()
        velocity = np.array([1., -.4, .2, 3., -2., 4., -.6, .9, -.2, -2., 1., -3.], np.float32)
        run.mjw_data.qvel.assign(velocity[None, :])
        before_qpos = run.mjw_data.qpos.numpy().copy()
        before_qvel = run.mjw_data.qvel.numpy().copy()
        install_connect_velocity_cache_refresh(run.mjw_model, run.mjw_data)
        native = mj.MjData(run.mj_model)
        native.qvel[:] = velocity
        mj.mj_forward(run.mj_model, native)
        adapter = BoundedDeterministicExecution(run, scope="all")
        try:
            with wp.ScopedDevice(self.device), run._scoped_deterministic_config():
                mw.forward(run.mjw_model, run.mjw_data)
            wp.synchronize_device(self.device)
            np.testing.assert_allclose(run.mjw_data.efc.aref.numpy()[0, :3], native.efc_aref[:3],
                                       rtol=2e-5, atol=2e-5)
            np.testing.assert_array_equal(run.mjw_data.qpos.numpy(), before_qpos)
            np.testing.assert_array_equal(run.mjw_data.qvel.numpy(), before_qvel)
            self.assertTrue(adapter.evidence["kernel_bounds"])
            self.assertLess(max(v["records_per_thread"] for v in adapter.evidence["kernel_bounds"].values()), 74305)
        finally:
            adapter.close()


if __name__ == "__main__":
    unittest.main()
