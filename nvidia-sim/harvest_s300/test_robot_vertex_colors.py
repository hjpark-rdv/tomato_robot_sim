"""Per-part colors must survive scene packing and the actual mesh builder."""
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

import mujoco as mj
import numpy as np
import trimesh

from newton_scene_chunks import PackedScene, pack_scene
from newton_viser import geom_mesh, populate_scene


class RobotColorTests(unittest.TestCase):
    def fixture(self, path):
        model = mj.MjModel.from_xml_string('''<mujoco><asset>
          <mesh name="part" vertex="0 0 0 .1 0 0 0 .1 0 0 0 .1"
            face="0 2 1 0 1 3 0 3 2 1 2 3"/>
          </asset><worldbody><body name="Robot_world">
          <geom name="colored_part" type="mesh" mesh="part" rgba="1 1 .2 1"/>
          </body></worldbody></mujoco>''')
        pack_scene(model, [0], path)
        scene = PackedScene(path)
        colors = np.array([[8,8,8,255], [160,166,174,255], [112,2,2,255], [3,3,3,255]],np.uint8)
        scene.vertex_color_lookup = {'colored_part': colors}
        return scene, colors

    def test_multiple_colors_survive_repacking_and_body_mesh_merge(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            scene, colors = self.fixture(root/'source.npz')
            pack_scene(scene,[0],root/'repacked.npz')
            loaded=PackedScene(root/'repacked.npz')
            colored=geom_mesh(loaded,0)
            np.testing.assert_array_equal(colored.visual.vertex_colors,colors)
            plain=geom_mesh(loaded,0)
            plain.visual.vertex_colors=np.tile([255,255,255,255],(4,1))
            body=trimesh.util.concatenate([plain,colored])
            np.testing.assert_array_equal(body.visual.vertex_colors[-4:],colors)

    def test_colored_static_mesh_avoids_single_color_instancing(self):
        with tempfile.TemporaryDirectory() as folder:
            scene,colors=self.fixture(Path(folder)/'source.npz')
            scene.geom_bodyid[:]=0
            calls=[]
            server=SimpleNamespace(scene=SimpleNamespace(add_mesh_trimesh=lambda name,mesh,**kwargs:
                calls.append(mesh.visual.vertex_colors.copy())))
            populate_scene(server,scene,include_dynamic=False)
            self.assertEqual(len(calls),1)
            np.testing.assert_array_equal(calls[0],colors)

    def test_mismatched_vertex_colors_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            scene,colors=self.fixture(Path(folder)/'source.npz')
            scene.vertex_color_lookup['colored_part']=colors[:-1]
            with self.assertRaisesRegex(ValueError,'count mismatch'):
                geom_mesh(scene,0)


if __name__=='__main__':
    unittest.main()
