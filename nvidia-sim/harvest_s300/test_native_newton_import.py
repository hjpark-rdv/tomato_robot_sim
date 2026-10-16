"""Public parser-option override is scoped and does not alter other options."""

from __future__ import annotations

import unittest

from native_newton_import import preserve_unordered_bodies, restore_loop_endpoint_roots


class ImportOptionTests(unittest.TestCase):
    def test_scoped_override_preserves_source_and_other_options(self):
        class Builder:
            joint_parent, joint_child, joint_articulation = [], [], []
            body_mass, body_label = [], []

            def add_usd(self, source, **kwargs):
                return source, kwargs

        original = Builder.add_usd
        with preserve_unordered_bodies(Builder, True):
            source, kwargs = Builder().add_usd("original.usda", collapse_fixed_joints=False,
                                              bodies_follow_joint_ordering=True)
            self.assertEqual(source, "original.usda")
            self.assertEqual(kwargs, {"collapse_fixed_joints": False, "bodies_follow_joint_ordering": False})
        self.assertIs(Builder.add_usd, original)

    def test_restores_after_failure(self):
        class Builder:
            def add_usd(self, source, **kwargs):
                raise RuntimeError("import failed")

        original = Builder.add_usd
        with self.assertRaises(RuntimeError):
            with preserve_unordered_bodies(Builder, True):
                Builder().add_usd("original.usda")
        self.assertIs(Builder.add_usd, original)

    def test_disabled_leaves_original_function(self):
        class Builder:
            def add_usd(self, source, **kwargs):
                return kwargs

        original = Builder.add_usd
        with preserve_unordered_bodies(Builder, False):
            self.assertIs(Builder.add_usd, original)
            self.assertEqual(Builder().add_usd("original.usda"), {})

    def test_adds_only_unowned_dynamic_loop_endpoint(self):
        class Builder:
            joint_parent = [-1, 0, 1, -1, 1]
            joint_child = [0, 1, 2, 3, 4]
            joint_articulation = [0, 0, -1, 1, -1]
            body_mass = [1.0, 1.0, 0.01, 0.01, 0.0]
            body_label = ["root", "stem", "fruit", "existing_free", "massless"]

            def __init__(self):
                self.free = []
                self.articulations = []

            def add_joint_free(self, **kwargs):
                self.free.append(kwargs)
                return 5

            def add_articulation(self, joints):
                self.articulations.append(joints)

        builder = Builder()
        restored = restore_loop_endpoint_roots(builder)
        self.assertEqual(restored, [{"body_index": 2, "body": "fruit", "joint_index": 5}])
        self.assertEqual(builder.free, [{"child": 2, "parent": -1, "label": "fruit/native_import_free_root"}])
        self.assertEqual(builder.articulations, [[5]])

    def test_real_loop_endpoint_keeps_dynamic_body_and_connect(self):
        try:
            import mujoco
            import newton
            import numpy as np
            import warp as wp
        except ImportError:
            self.skipTest("Optional Newton/MuJoCo environment required")
        with wp.ScopedDevice("cpu"):
            builder = newton.ModelBuilder()
            stem = builder.add_link(xform=wp.transform((0, 0, 1), wp.quat_identity()),
                                    mass=1.0, inertia=wp.mat33(np.eye(3)), label="stem")
            fruit = builder.add_link(xform=wp.transform((0, 0, 0.8), wp.quat_identity()),
                                     mass=0.01, inertia=wp.mat33(np.eye(3) * 1e-5), label="fruit")
            root_joint = builder.add_joint_fixed(parent=-1, child=stem)
            builder.add_articulation([root_joint])
            builder.add_joint_ball(parent=stem, child=fruit,
                                   parent_xform=wp.transform((0, 0, -0.2), wp.quat_identity()),
                                   label="separate_attachment")
            restored = restore_loop_endpoint_roots(builder)
            self.assertEqual([x["body"] for x in restored], ["fruit"])
            model = builder.finalize(device="cpu")
            solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=True)
            mj_model = solver.mj_model
            self.assertEqual(mj_model.nv, 6)
            self.assertEqual(mj_model.neq, 1)
            self.assertEqual(int(mj_model.eq_type[0]), int(mujoco.mjtEq.mjEQ_CONNECT))
            mj_data = mujoco.MjData(mj_model)
            mujoco.mj_forward(mj_model, mj_data)
            fruit_id = mj_model.joint(0).bodyid[0]
            before = mj_data.xpos[fruit_id].copy()
            mj_model.opt.timestep = 0.001
            for _ in range(100):
                mujoco.mj_step(mj_model, mj_data)
            attached_displacement = np.linalg.norm(mj_data.xpos[fruit_id] - before)
            self.assertLess(attached_displacement, 0.001)
            mj_data.eq_active[0] = 0
            for _ in range(100):
                mujoco.mj_step(mj_model, mj_data)
            self.assertGreater(before[2] - mj_data.xpos[fruit_id, 2], 0.02)


if __name__ == "__main__":
    unittest.main()
