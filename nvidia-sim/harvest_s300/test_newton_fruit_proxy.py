from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import mujoco as mj
import numpy as np
from scipy.spatial import ConvexHull

from newton_fruit_proxy import (conservative_proxy, geom_body_vertices, prepare,
                               sha256, sphere_directions, verify_compiled_proxy)


def points(count=2048):
    return sphere_directions(count) * np.array([0.010, 0.012, 0.014])


def fixture(root, shared=False):
    source = root / "source"
    source.mkdir()
    vertices = points(800)
    vertex_text = " ".join(format(float(x), ".17g") for x in vertices.ravel())
    other = '<geom name="other_mesh" type="mesh" mesh="fruit"/>' if shared else '<geom name="other" type="sphere" size=".006" pos=".04 0 0"/>'
    xml = '''<mujoco><compiler angle="radian" inertiafromgeom="false" fusestatic="false"/>
    <option timestep=".0025" integrator="implicitfast"/>
    <asset><mesh name="fruit" vertex="''' + vertex_text + '''" scale="1.1 .9 1.2"
      refpos=".001 -.002 .0005" refquat=".9238795325 .3826834324 0 0"/></asset>
    <worldbody><body name="anchor"><inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
    ''' + other + '''</body><body name="Truss_00_Fruit_01" pos=".1 .2 .8">
    <freejoint name="fruit_free"/><inertial pos=".0001 .0002 0" mass=".01" diaginertia=".000002 .000003 .000004"/>
    <geom name="fruit_geom" type="mesh" mesh="fruit" pos=".004 -.003 .002" quat=".9659258263 0 .2588190451 0"
     friction=".7 .003 .0002" margin="0" gap=".0005" solref=".002 1"/>
    </body></worldbody><equality><connect body1="anchor" body2="Truss_00_Fruit_01" anchor=".1 .2 .8"/></equality></mujoco>'''
    (source / "input_scene.xml").write_text(xml)
    model = mj.MjModel.from_xml_path(str(source / "input_scene.xml"))
    model.dof_length[:] = np.arange(model.nv) * 0.001 + 0.01
    model.dof_damping[:] = np.arange(model.nv) * 0.0001
    mj.mj_saveModel(model, str(source / "input_model.mjb"))
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    np.savez_compressed(source / "initial_state.npz", qpos=data.qpos, qvel=data.qvel,
                        eq_active=data.eq_active, ctrl=data.ctrl,
                        runtime_dof_damping=model.dof_damping, runtime_dof_length=model.dof_length)
    np.savez_compressed(source / "commands.npz", time=np.array([0.0]), time_edges=np.array([0.0, .0025]))
    (source / "sampled_plan.json").write_text('{"unchanged": true}\n')
    (source / "source_export.json").write_text(json.dumps({"fruits": ["Truss_00_Fruit_01"], "attachments": []}))
    names = ("input_scene.xml", "input_model.mjb", "initial_state.npz", "commands.npz", "source_export.json")
    (source / "inputs.json").write_text(json.dumps({"input_files": {name: {"sha256": sha256(source / name)} for name in names}}))
    return source


class ConservativeProxyTests(unittest.TestCase):
    def test_encloses_complete_ellipsoid_with_outward_bound(self):
        source = points()
        vertices, faces, certificate = conservative_proxy(source)
        hull = ConvexHull(vertices)
        self.assertLess(float(np.max(source @ hull.equations[:, :3].T + hull.equations[:, 3])), 0)
        self.assertLessEqual(certificate["analytic_outward_bound_m"] + certificate["compile_reserve_m"], 0.0001)
        self.assertLess(len(vertices), len(source))
        self.assertGreater(len(faces), 3)
        self.assertIn(certificate["support_budget"], [128, 256, 512, 1024])

    def test_wrong_compiled_transform_is_rejected(self):
        source = points()
        proxy, _, certificate = conservative_proxy(source)
        with self.assertRaises(ValueError):
            verify_compiled_proxy(source, proxy, proxy + [0.002, 0, 0],
                                  certificate["analytic_outward_bound_m"], 0.0001)

    def test_invalid_or_unachievable_bounds_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "Finite"):
            conservative_proxy([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, np.nan]])
        with self.assertRaisesRegex(ValueError, "nondegenerate"):
            conservative_proxy([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]])
        with self.assertRaisesRegex(ValueError, "No proxy"):
            conservative_proxy(points(), max_outward_m=0.000001, budgets=(128,))
        with self.assertRaisesRegex(ValueError, "Positive"):
            conservative_proxy(points(), max_outward_m=float("inf"))

    def test_prepared_input_preserves_physics_and_compiled_body_frame(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = fixture(root)
            source_files = {p.name: sha256(p) for p in source.iterdir()}
            result = prepare(source, root / "proxy", budgets=(128, 256, 512, 1024, 2048))
            new = mj.MjModel.from_binary_path(str(root / "proxy/input_model.mjb"))
            old = mj.MjModel.from_binary_path(str(source / "input_model.mjb"))
            self.assertEqual(source_files, {p.name: sha256(p) for p in source.iterdir()})
            self.assertEqual(sha256(source / "initial_state.npz"), sha256(root / "proxy/initial_state.npz"))
            self.assertEqual(sha256(source / "commands.npz"), sha256(root / "proxy/commands.npz"))
            self.assertEqual(sha256(source / "sampled_plan.json"), sha256(root / "proxy/sampled_plan.json"))
            for name in ("body_mass", "body_inertia", "body_ipos", "body_iquat", "dof_length", "dof_damping", "eq_data", "geom_friction", "geom_margin", "geom_gap"):
                np.testing.assert_array_equal(getattr(old, name), getattr(new, name))
            self.assertTrue(result["invariant_proof"]["passed"])
            self.assertTrue(result["fruits"][0]["source_enclosed"])
            self.assertLessEqual(result["fruits"][0]["compiled_outward_bound_m"], .0001)
            hull = ConvexHull(geom_body_vertices(new, new.geom("fruit_geom").id))
            source_vertices = geom_body_vertices(old, old.geom("fruit_geom").id)
            self.assertLess(float(np.max(source_vertices @ hull.equations[:, :3].T + hull.equations[:, 3])), 0)
            self.assertFalse(result["retained_all_collision_geometry"])
            self.assertTrue(result["retained_all_nonfruit_collision_geometry"])
            self.assertFalse(result["training_eligible"])
            proof = json.loads((root / "proxy/inputs.json").read_text())
            self.assertTrue(proof["source_prepared_files_unchanged"])
            for name, evidence in proof["input_files"].items():
                self.assertEqual(sha256(root / "proxy" / name), evidence["sha256"])
            with self.assertRaises(FileExistsError):
                prepare(source, root / "proxy")

    def test_shared_mesh_cannot_change_other_collider(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = fixture(root, shared=True)
            with self.assertRaisesRegex(ValueError, "Shared target mesh"):
                prepare(source, root / "proxy")
            self.assertFalse((root / "proxy").exists())


if __name__ == "__main__":
    unittest.main()
