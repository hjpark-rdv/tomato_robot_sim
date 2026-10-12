"""Camera behavior and real Filament saved-state rendering regression tests."""
import os
import unittest
from types import SimpleNamespace

import numpy as np

from filament_backend import FilamentRenderer, orbit_camera, pan_camera, zoom_camera


class CameraControlTests(unittest.TestCase):
    def camera(self):
        return SimpleNamespace(azimuth=0., elevation=-20., distance=2.,
                               lookat=np.array([1., 2., 3.]))

    def test_orbit_clamps_elevation(self):
        c = self.camera()
        orbit_camera(c, 40, -10000)
        self.assertEqual(c.azimuth, 10)
        self.assertEqual(c.elevation, -89)

    def test_pan_and_zoom_are_finite_and_do_not_rotate(self):
        c = self.camera()
        pan_camera(c, 100, 20, 1000)
        np.testing.assert_allclose(c.lookat, [1., 2.26, 3.052])
        zoom_camera(c, 3)
        self.assertLess(c.distance, 2)
        self.assertEqual(c.azimuth, 0)
        self.assertEqual(c.elevation, -20)
        zoom_camera(c, -1000)
        self.assertEqual(c.distance, 1000)

    def test_invalid_quality_and_resolution_are_rejected(self):
        with self.assertRaises(ValueError):
            FilamentRenderer(None, quality='pretend-ray-tracing')
        with self.assertRaises(ValueError):
            FilamentRenderer(None, width=0)


@unittest.skipUnless(os.environ.get('S300_FILAMENT_GPU_TEST') == '1',
                     'Set S300_FILAMENT_GPU_TEST=1 to run actual NVIDIA rendering')
class FilamentRenderingTests(unittest.TestCase):
    def test_color_grading_exposure_is_applied_on_first_render(self):
        import mujoco as mj
        images = []
        for exposure in (0., 1.):
            model = mj.MjModel.from_xml_string('''<mujoco><custom>
            <numeric name="filament.cg.exposure" data="'''+str(exposure)+'''"/>
            <text name="filament.cg.tone_mapping" data="pbr_neutral"/>
            </custom><asset><material name="red" rgba=".6 .04 .02 1" roughness=".5"/></asset>
            <worldbody><light pos="0 0 3"/>
            <geom type="plane" size="3 3 .1" rgba=".8 .8 .8 1"/>
            <geom type="sphere" pos="0 0 .5" size=".4" material="red"/>
            </worldbody></mujoco>''')
            data = mj.MjData(model)
            mj.mj_kinematics(model, data)
            mj.mj_comPos(model, data)
            mj.mj_camlight(model, data)
            camera = mj.MjvCamera()
            mj.mjv_defaultFreeCamera(model, camera)
            camera.distance = 2
            camera.elevation = -25
            with FilamentRenderer(model, height=192, width=256) as renderer:
                renderer.update_scene(data, camera)
                images.append(renderer.render().copy())
                self.assertEqual(renderer.color_grading_configure_calls, 2)
                self.assertEqual(data.time, 0)
        # In 3.15 one scene configure silently used previous/default EV=0.
        self.assertGreater(images[1].mean(), images[0].mean() + 10)

    def test_hidden_collision_material_does_not_override_zero_alpha(self):
        import mujoco as mj
        xml = '''<mujoco><asset><material name="white" rgba="1 1 1 1"/></asset>
        <worldbody><light pos="0 0 3"/>
        <geom type="plane" size="3 3 .1" rgba=".8 .8 .8 1"/>
        <geom type="sphere" pos="0 0 .5" size=".4" rgba=".55 .55 .55 1"/>
        <geom name="hidden" type="box" pos="0 0 .5" size=".28 .28 .36"
              rgba="1 1 1 0" material="white"/>
        </worldbody></mujoco>'''
        images = []
        masks = []
        for corrected in (False, True):
            model = mj.MjModel.from_xml_string(xml)
            hidden = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, 'hidden')
            if corrected:
                # Filament prioritizes nondefault material RGBA, including
                # alpha. Unbind that invisible visual's material explicitly.
                model.geom_matid[hidden] = -1
            masks.append((model.geom_contype.copy(), model.geom_conaffinity.copy()))
            self.assertEqual(model.geom_rgba[hidden, 3], 0)
            data = mj.MjData(model)
            mj.mj_kinematics(model, data)
            mj.mj_comPos(model, data)
            mj.mj_camlight(model, data)
            camera = mj.MjvCamera()
            mj.mjv_defaultFreeCamera(model, camera)
            camera.lookat[:] = [0, 0, .4]
            camera.distance = 2
            camera.azimuth = 90
            camera.elevation = -20
            with FilamentRenderer(model, height=192, width=256) as renderer:
                renderer.update_scene(data, camera)
                images.append(renderer.render().copy())
                self.assertEqual(data.time, 0)
        for old, corrected in zip(masks[0], masks[1]):
            np.testing.assert_array_equal(old, corrected)
        # The bound white collider was visibly present despite geom alpha=0.
        # Removing only its render material hides it and changes these pixels.
        self.assertGreater(np.mean(np.abs(images[0].astype(float)-images[1])), 1)

    def test_indexed_skin_padding_renders_without_changing_shared_faces(self):
        import mujoco as mj
        # Four indexed rectangle vertices, two triangles. Two unreferenced
        # copies make native 3.15's allocated index capacity equal six while
        # preserving the original triangles and smooth shared vertices.
        model = mj.MjModel.from_xml_string('''<mujoco><asset>
        <material name="skin_pbr" rgba=".9 .03 .02 1" roughness=".5" metallic="0"/>
        <skin name="indexed" rgba=".9 .03 .02 1" material="skin_pbr"
          vertex="-.5 -.2 .5  .5 -.2 .5  .5 .2 .5  -.5 .2 .5  -.5 -.2 .5  -.5 -.2 .5"
          face="0 1 2 0 2 3" texcoord="0 0 0 0 0 0 0 0 0 0 0 0">
          <bone body="branch" bindpos="0 0 .5" bindquat="1 0 0 0"
             vertid="0 1 2 3 4 5" vertweight="1 1 1 1 1 1"/>
        </skin></asset><worldbody><light pos="0 0 3"/>
        <geom type="plane" size="3 3 .1" rgba=".8 .8 .8 1"/>
        <body name="branch" pos="0 0 .5"><joint name="hinge" axis="0 0 1"/>
        <geom type="sphere" size=".05" rgba="0 0 0 0"/></body>
        </worldbody></mujoco>''')
        data = mj.MjData(model)
        camera = mj.MjvCamera()
        mj.mjv_defaultFreeCamera(model, camera)
        camera.lookat[:] = [0., 0., .5]
        camera.distance = 2
        camera.elevation = -65
        original_faces = model.skin_face.copy()
        self.assertGreaterEqual(model.skin_texcoordadr[0], 0)
        with FilamentRenderer(model, height=192, width=256) as renderer:
            images = []
            for q in (0., .8):
                data.qpos[0] = q
                mj.mj_kinematics(model, data)
                mj.mj_comPos(model, data)
                mj.mj_camlight(model, data)
                renderer.update_scene(data, camera)
                images.append(renderer.render().copy())
                self.assertEqual(data.time, 0)
                self.assertEqual(data.qpos[0], q)
                self.assertGreater(images[-1].std(), 5)
        np.testing.assert_array_equal(model.skin_face, original_faces)
        self.assertGreater(np.mean(np.abs(images[0].astype(float)-images[1])), 1)

    def test_measured_state_changes_pixels_without_advancing_physics(self):
        import mujoco as mj
        model = mj.MjModel.from_xml_string('''<mujoco><worldbody>
            <light pos="0 0 3"/><geom type="plane" size="3 3 .1"/>
            <body pos="0 0 .5"><freejoint/>
            <geom type="sphere" size=".2" rgba="1 .05 .02 1"/>
            </body></worldbody></mujoco>''')
        data = mj.MjData(model)
        camera = mj.MjvCamera()
        mj.mjv_defaultFreeCamera(model, camera)
        camera.distance = 3
        camera.elevation = -25
        with FilamentRenderer(model, height=192, width=256) as renderer:
            with self.assertRaises(RuntimeError):
                renderer.render()
            mj.mj_kinematics(model, data)
            mj.mj_comPos(model, data)
            mj.mj_camlight(model, data)
            before = data.qpos.copy()
            before_velocity = data.qvel.copy()
            renderer.update_scene(data, camera)
            image1 = renderer.render().copy()
            self.assertEqual(image1.shape, (192, 256, 3))
            self.assertGreater(image1.std(), 5)
            self.assertEqual(data.time, 0)
            np.testing.assert_array_equal(data.qpos, before)
            np.testing.assert_array_equal(data.qvel, before_velocity)
            data.qpos[0] = .8
            mj.mj_kinematics(model, data)
            mj.mj_comPos(model, data)
            mj.mj_camlight(model, data)
            renderer.update_scene(data, camera)
            image2 = renderer.render()
            self.assertGreater(np.mean(np.abs(image2.astype(float) - image1)), 1)
            self.assertEqual(data.time, 0)
        with self.assertRaises(RuntimeError):
            renderer.render()


if __name__ == '__main__':
    unittest.main()
