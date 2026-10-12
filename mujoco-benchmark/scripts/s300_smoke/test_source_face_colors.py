"""Palette assignment preserves physical arrays and visible face identity."""
import os
from pathlib import Path
import tempfile
import unittest

import mujoco as mj
import numpy as np

from quality_profiles import quality_profile
from render_assets import compile_render_model, physics_parameter_hash
from source_face_colors import verify_surface_mapping


def fixture(folder):
    vertices = np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0], [0, 0, -.1]], float)
    faces = np.array([[0, 1, 2], [0, 2, 3], [0, 4, 1], [1, 4, 2], [2, 4, 3], [3, 4, 0]], np.int32)
    xml = '<mujoco><asset><mesh name="shape" vertex="' + ' '.join(map(str, vertices.ravel())) + '" face="' + ' '.join(map(str, faces.ravel())) + '"/></asset><worldbody><light pos="0 0 3"/><geom name="surface" type="mesh" mesh="shape" contype="4" conaffinity="7"/></worldbody></mujoco>'
    (folder / 'scene.xml').write_text(xml)
    original = mj.MjModel.from_xml_string(xml)
    material = dict(rgba=[.5, .5, .5, 1.], roughness=.5, metallic=0., color_space='linear_rec709', connected_channels=[])
    record = dict(key='colours', geometry='surface', source='/mesh', method='material_subsets', subsets=[{}, {}], material=material)
    assets = dict(geometry_materials={'surface': material}, source_invisible_geometry=[], skins=[], face_colors=[record])
    arrays = dict(colours_vertices=vertices, colours_faces=faces,
                  colours_labels=np.array([0, 1, 0, 0, 0, 0], np.int32),
                  colours_palette=np.array([[1, 0, 0, 1], [0, 0, 1, 1]], float))
    return original, assets, arrays


class FaceColorTests(unittest.TestCase):
    def test_face_palette_changes_only_appearance(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp); original, assets, arrays = fixture(folder)
            model, details = compile_render_model(original, folder, assets, arrays, profile=quality_profile('balanced'))
            self.assertEqual(physics_parameter_hash(model), physics_parameter_hash(original))
            np.testing.assert_array_equal(model.mesh_normal, original.mesh_normal)
            np.testing.assert_array_equal(model.mesh_facenormal, original.mesh_facenormal)
            self.assertEqual(len(details['face_colour_recovery']), 1)
            self.assertEqual(model.mesh_texcoordnum[0], 2)
            uv = model.mesh_texcoord[model.mesh_facetexcoord[:2]]
            np.testing.assert_array_equal(uv[0], [[.5, .25]] * 3)
            np.testing.assert_array_equal(uv[1], [[.5, .75]] * 3)
            mat = model.geom_matid[0]
            tex = model.mat_texid[mat, mj.mjtTextureRole.mjTEXROLE_RGB]
            self.assertGreaterEqual(tex, 0)
            begin = model.tex_adr[tex]
            image = model.tex_data[begin:begin + 16 * 32 * 3].reshape(32, 16, 3)
            np.testing.assert_array_equal(image[8, 8], [255, 0, 0])
            np.testing.assert_array_equal(image[24, 8], [0, 0, 255])

    def test_reordered_faces_are_rejected_even_with_same_vertices(self):
        with tempfile.TemporaryDirectory() as tmp:
            original, assets, arrays = fixture(Path(tmp))
            with self.assertRaisesRegex(ValueError, 'mapping mismatch'):
                verify_surface_mapping(original, 0, arrays['colours_vertices'], arrays['colours_faces'][::-1])

    @unittest.skipUnless(os.environ.get('S300_FILAMENT_GPU_TEST') == '1', 'actual GPU test')
    def test_actual_filament_pixels_follow_face_palette_not_mesh_first_color(self):
        from filament_backend import FilamentRenderer
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp); original, assets, arrays = fixture(folder)
            model, _ = compile_render_model(original, folder, assets, arrays, profile=quality_profile('balanced'))
            data = mj.MjData(model)
            mj.mj_kinematics(model, data); mj.mj_comPos(model, data); mj.mj_camlight(model, data)
            cam = mj.MjvCamera(); mj.mjv_defaultFreeCamera(model, cam)
            cam.lookat[:] = [0, 0, 0]; cam.distance = 4; cam.azimuth = 90; cam.elevation = -89.9
            with FilamentRenderer(model, width=256, height=256) as renderer:
                renderer.update_scene(data, cam)
                image = renderer.render().copy().astype(int)
            red = (image[..., 0] > image[..., 2] + 70) & (image[..., 0] > image[..., 1] + 70)
            blue = (image[..., 2] > image[..., 0] + 70) & (image[..., 2] > image[..., 1] + 70)
            self.assertGreater(red.sum(), 1000)
            self.assertGreater(blue.sum(), 1000)
            # Camera azimuth 90 maps world +X to screen right; first triangle
            # lies at +X,-Y. Check orientation, not only presence of two colours.
            self.assertGreater(np.nonzero(red)[1].mean(), np.nonzero(blue)[1].mean())
            self.assertEqual(data.time, 0.)


if __name__ == '__main__':
    unittest.main()
