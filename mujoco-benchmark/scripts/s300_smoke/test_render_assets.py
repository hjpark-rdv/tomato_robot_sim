"""Appearance recovery must keep exact recorded physical parameters/state."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import mujoco as mj
import numpy as np

from render_assets import (compile_render_model, file_sha, load_render_model,
                           physics_parameter_hash, restore_physics_parameters, update_render_visuals, prepare_filament_skin_arrays, filament_rgba)
from quality_profiles import quality_profile

XML = '''<mujoco><option timestep=".004" gravity="0 0 -9.81"/>
<worldbody><body name="branch" pos="1 2 3"><joint name="hinge" type="hinge" axis="0 0 1"/>
<geom name="proxy" type="sphere" size=".1" contype="4" conaffinity="7" rgba=".2 .5 .1 1"/>
<geom name="surface" type="box" size=".1 .1 .1" contype="0" conaffinity="0" rgba=".7 .1 .1 1"/>
</body></worldbody><actuator><position name="motor" joint="hinge" kp="20" kv="2"/></actuator></mujoco>'''


def fixture(folder):
    (folder / 'scene.xml').write_text(XML)
    model = mj.MjModel.from_xml_path(str(folder / 'scene.xml'))
    # Runtime snapshots may differ from exported XML. Rendering must retain
    # the exact active-window masks/timestep of the recorded model.
    model.geom_conaffinity[0] = 0
    model.geom_contype[0] = 0
    model.opt.timestep = .003
    values = dict(rgba=[.7, .1, .1, 1.], roughness=.22, metallic=.6,
                  connected_channels=[])
    assets = dict(geometry_materials={'surface': values, 'proxy': dict(values, rgba=[1., 1., 1., 1.])}, source_invisible_geometry=['proxy'],
                  skins=[dict(key='skin', material=values, bodies=['branch'],
                              vertices=3, triangles=1)], limitations=[], skipped=[])
    arrays = dict(skin_vertices=np.array([[1., 2., 3.], [2., 2., 3.], [1., 3., 3.]]),
                  skin_faces=np.array([[0, 1, 2]], np.int32),
                  skin_rest=np.array([[1., 2., 3.]]),
                  skin_ids=np.zeros((3, 1), np.int32), skin_weights=np.ones((3, 1)))
    return model, assets, arrays


class RenderAssetTests(unittest.TestCase):
    def test_materials_and_skin_preserve_recorded_physics(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp); original, assets, arrays = fixture(folder)
            before = physics_parameter_hash(original)
            with patch.object(mj, 'mj_step', side_effect=AssertionError('Replay advanced physics')):
                rendered, details = compile_render_model(original, folder, assets, arrays,
                                                          profile=quality_profile('ultra'))
            self.assertEqual(before, physics_parameter_hash(original))
            self.assertEqual(before, physics_parameter_hash(rendered))
            for kind, count in ((mj.mjtObj.mjOBJ_BODY, original.nbody),
                                (mj.mjtObj.mjOBJ_JOINT, original.njnt),
                                (mj.mjtObj.mjOBJ_GEOM, original.ngeom),
                                (mj.mjtObj.mjOBJ_ACTUATOR, original.nu)):
                self.assertEqual([mj.mj_id2name(original, kind, i) for i in range(count)],
                                 [mj.mj_id2name(rendered, kind, i) for i in range(count)])
            self.assertEqual(rendered.nskin, 1)
            self.assertGreaterEqual(rendered.skin_texcoordadr[0], 0)
            self.assertTrue(np.all(rendered.skin_texcoord == 0))
            self.assertEqual(details['hidden_source_collision_visuals'], 1)
            np.testing.assert_array_equal(rendered.geom_contype, original.geom_contype)
            self.assertEqual(original.geom_rgba[0, 3], 1.)
            self.assertEqual(rendered.geom_rgba[0, 3], 0.)
            self.assertEqual(rendered.geom_matid[0], -1)  # Native Filament matRGBA otherwise overrides alpha.
            self.assertEqual(details['lighting_policy']['name'], 'neutral_daylight')
            self.assertEqual(rendered.ntex, 1)
            self.assertEqual(rendered.light_intensity[-1], 15000.)
            np.testing.assert_allclose(rendered.geom_rgba[1], [.7, .1, .1, 1.])
            matid = rendered.geom_matid[1]
            self.assertAlmostEqual(rendered.mat_roughness[matid], .22)
            self.assertAlmostEqual(rendered.mat_metallic[matid], .6)
            shadow_id = mj.mj_name2id(rendered, mj.mjtObj.mjOBJ_NUMERIC, 'filament.shadows.map_size')
            self.assertEqual(rendered.numeric_data[rendered.numeric_adr[shadow_id]], 8192)

    def test_native_skin_matches_original_body_transform_rule(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp); original, assets, arrays = fixture(folder)
            rendered, _ = compile_render_model(original, folder, assets, arrays)
            data = mj.MjData(rendered); data.qpos[0] = np.pi / 2
            mj.mj_kinematics(rendered, data); mj.mj_comPos(rendered, data); mj.mj_camlight(rendered, data)
            scene = mj.MjvScene(rendered, maxgeom=10)
            mj.mjv_updateScene(rendered, data, mj.MjvOption(), None, mj.MjvCamera(),
                              mj.mjtCatBit.mjCAT_ALL, scene)
            body = mj.mj_name2id(rendered, mj.mjtObj.mjOBJ_BODY, 'branch')
            expected = ((arrays['skin_vertices'] - arrays['skin_rest'][0]) @
                        data.xmat[body].reshape(3, 3).T + data.xpos[body])
            np.testing.assert_allclose(scene.skinvert.reshape(-1, 3), expected, atol=1e-7)
            self.assertEqual(data.time, 0.)

    def test_physics_topology_change_refused(self):
        a = mj.MjModel.from_xml_string(XML)
        b = mj.MjModel.from_xml_string(XML.replace('<joint name="hinge" type="hinge" axis="0 0 1"/>', '').replace('<actuator><position name="motor" joint="hinge" kp="20" kv="2"/></actuator>', ''))
        with self.assertRaises(ValueError):
            restore_physics_parameters(a, b)


    def test_linkage_updates_derived_visual_poses_only(self):
        # Resolve from the source repository, independent of asset environment.
        helper = Path(__file__).resolve().parents[3] / 'nvidia-sim/harvest_s300/transmission_geometry.py'
        xml = '<mujoco><worldbody><body name="Tool_fixed" pos="1 2 3"><geom name="link" type="box" size=".01 .01 .01" pos=".01 .02 .03" contype="0" conaffinity="0"/><body name="left"><joint name="jaw_L" type="slide" axis="1 0 0"/><geom type="sphere" size=".01"/></body><body name="right"><joint name="jaw_R" type="slide" axis="1 0 0"/><geom type="sphere" size=".01"/></body></body></worldbody></mujoco>'
        model = mj.MjModel.from_xml_string(xml); data = mj.MjData(model)
        evidence = dict(transmission=dict(body='Tool_fixed', nominal_gap_m=.044,
            jaws=['jaw_L', 'jaw_R'], parent_in_tool=np.eye(4).tolist(),
            components=dict(crank=dict(source_component_in_tool=np.eye(4).tolist())),
            geoms=[dict(name='link', component='crank')], function_source=str(helper),
            function_source_sha256=file_sha(helper)))
        before_model = physics_parameter_hash(model)
        mj.mj_kinematics(model, data); mj.mj_comPos(model, data)
        nominal = data.geom_xpos.copy()
        actual = update_render_visuals(model, data, evidence)
        np.testing.assert_allclose(data.geom_xpos, nominal, atol=1e-12)
        self.assertEqual(actual['transmission_visual_geoms'], 1)
        left = model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'jaw_L')]
        right = model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'jaw_R')]
        data.qpos[left] = -.026; data.qpos[right] = .026
        mj.mj_kinematics(model, data); mj.mj_comPos(model, data)
        physics_before = {name: getattr(data, name).copy() for name in ('qpos', 'qvel', 'xpos', 'xquat', 'eq_active')}
        actual = update_render_visuals(model, data, evidence)
        self.assertAlmostEqual(actual['actual_gap_m'], .096)
        self.assertGreater(np.linalg.norm(data.geom_xpos[0] - nominal[0]), .001)
        for name, value in physics_before.items():
            np.testing.assert_array_equal(getattr(data, name), value)
        self.assertEqual(before_model, physics_parameter_hash(model))
        json.dumps(evidence)  # Runtime animation context must not contaminate report JSON.


    def test_filament_index_padding_preserves_original_surface_and_normals(self):
        vertices = np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., .3], [0., 1., 0.]])
        faces = np.array([[0, 1, 2], [0, 2, 3]], np.int32)
        ids = np.zeros((4, 1), np.int32); weights = np.ones((4, 1))
        v, f, i, w, details = prepare_filament_skin_arrays(vertices, faces, ids, weights)
        self.assertEqual(len(v), faces.size)
        self.assertEqual(details['padding_vertices'], 2)
        np.testing.assert_array_equal(v[:4], vertices)
        np.testing.assert_array_equal(f, faces)
        def scene_normals(vv, ff, ii, ww):
            spec = mj.MjSpec.from_string('<mujoco><worldbody><body name="branch"><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
            spec.add_skin(name='skin', vert=vv.ravel().tolist(), face=ff.ravel().tolist(),
                bodyname=['branch'], bindpos=[0, 0, 0], bindquat=[1, 0, 0, 0],
                vertid=[list(range(len(vv)))], vertweight=[ww.ravel().tolist()], inflate=0.)
            model = spec.compile(); data = mj.MjData(model); mj.mj_kinematics(model, data)
            scene = mj.MjvScene(model, maxgeom=10)
            mj.mjv_updateScene(model, data, mj.MjvOption(), None, mj.MjvCamera(), mj.mjtCatBit.mjCAT_ALL, scene)
            return scene.skinnormal.reshape(-1, 3).copy()
        original = scene_normals(vertices, faces, ids, weights)
        padded = scene_normals(v, f, i, w)
        np.testing.assert_array_equal(original, padded[:4])
        self.assertTrue(np.isfinite(padded).all())

    def test_unreferenced_skin_vertices_can_be_compacted_for_index_buffer(self):
        vertices = np.arange(15).reshape(5, 3).astype(float)
        faces = np.array([[1, 3, 4]], np.int32)
        ids = np.zeros((5, 1), np.int32); weights = np.ones((5, 1))
        v, f, _, _, details = prepare_filament_skin_arrays(vertices, faces, ids, weights)
        self.assertEqual(len(v), 3)
        np.testing.assert_array_equal(v[f], vertices[faces])
        self.assertEqual(details['removed_unreferenced_vertices'], 2)


    def test_source_linear_rgb_encoding_and_alpha_preserved(self):
        raw = dict(rgba=[.121, .0031308, 0., .25], color_space='lin_rec709_scene')
        encoded = filament_rgba(raw)
        self.assertAlmostEqual(encoded[0], 1.055 * .121 ** (1 / 2.4) - .055)
        self.assertAlmostEqual(encoded[1], 12.92 * .0031308)
        self.assertEqual(encoded[2:], [0., .25])
        self.assertEqual(raw['rgba'], [.121, .0031308, 0., .25])
        for label in ('srgb_rec709_scene', 'raw', 'render_fallback_srgb'):
            np.testing.assert_array_equal(filament_rgba(dict(raw, color_space=label)), raw['rgba'])
        with self.assertRaisesRegex(ValueError, 'Unsupported source material colour space'):
            filament_rgba(dict(raw, color_space='acescg'))


    def test_uv0_compatibility_preserves_geometry_and_existing_uvs(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for authored_uv in (False, True):
                texcoord = ' texcoord=".1 .2 .3 .4 .5 .6 .7 .8"' if authored_uv else ''
                xml = '<mujoco><asset><mesh name="shape" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 3 0 2 3 1 2 3"' + texcoord + '/></asset><worldbody><geom name="surface" type="mesh" mesh="shape"/></worldbody></mujoco>'
                (folder / 'scene.xml').write_text(xml)
                original = mj.MjModel.from_xml_string(xml)
                values = dict(rgba=[.7, .1, .1, 1.], roughness=.5, metallic=0., connected_channels=[])
                assets = dict(geometry_materials={'surface': values}, source_invisible_geometry=[], skins=[])
                rendered, details = compile_render_model(original, folder, assets, {}, profile=quality_profile('balanced'))
                self.assertEqual(physics_parameter_hash(original), physics_parameter_hash(rendered))
                np.testing.assert_array_equal(original.mesh_vert, rendered.mesh_vert)
                np.testing.assert_array_equal(original.mesh_face, rendered.mesh_face)
                np.testing.assert_array_equal(original.mesh_normal, rendered.mesh_normal)
                self.assertGreaterEqual(rendered.mesh_texcoordadr[0], 0)
                self.assertEqual(rendered.mesh_texcoordnum[0], rendered.mesh_vertnum[0])
                if authored_uv:
                    np.testing.assert_array_equal(original.mesh_texcoord, rendered.mesh_texcoord)
                    np.testing.assert_array_equal(original.mesh_facetexcoord, rendered.mesh_facetexcoord)
                    self.assertEqual(details['untextured_mesh_zero_uv0_assets'], 0)
                else:
                    self.assertTrue(np.all(rendered.mesh_texcoord == 0))
                    self.assertEqual(details['untextured_mesh_zero_uv0_assets'], 1)

    def test_connected_textures_are_not_silently_replaced_by_zero_uv(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp); original, assets, arrays = fixture(folder)
            assets['geometry_materials']['surface']['connected_channels'] = ['diffuseColor']
            with self.assertRaisesRegex(ValueError, 'Connected source textures'):
                compile_render_model(original, folder, assets, arrays, profile=quality_profile('balanced'))

    def test_fast_quality_is_no_change(self):
        original = mj.MjModel.from_xml_string(XML)
        returned, report = load_render_model(original, '/unused', quality='fast')
        self.assertIs(returned, original)
        self.assertFalse(report['source_appearance_recovery'])

    def test_cache_reuse_is_bound_to_input_and_quality(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp); original, assets, arrays = fixture(folder)
            cache = folder / 'cache'; cache.mkdir()
            np.savez_compressed(cache / 'skin_arrays.npz', **arrays)
            (cache / 'render_assets.json').write_text(json.dumps(assets))
            original_mjb = folder / 'original.mjb'; mj.mj_saveModel(original, str(original_mjb), None)
            original_sha = file_sha(original_mjb)
            with patch('render_assets.ensure_source_assets', return_value=(cache, assets)):
                first, a = load_render_model(original, folder, original_mjb=original_mjb, quality='balanced')
                second, b = load_render_model(original, folder, original_mjb=original_mjb, quality='balanced')
                ultra, c = load_render_model(original, folder, original_mjb=original_mjb, quality='ultra')
            self.assertFalse(a['cache_reused']); self.assertTrue(b['cache_reused'])
            self.assertNotEqual(a['render_cache'], c['render_cache'])
            self.assertEqual(file_sha(original_mjb), original_sha)
            self.assertEqual(physics_parameter_hash(first), physics_parameter_hash(second))
            self.assertEqual(physics_parameter_hash(first), physics_parameter_hash(ultra))
            Path(b['render_cache']).write_bytes(b'bad')
            with patch('render_assets.ensure_source_assets', return_value=(cache, assets)):
                with self.assertRaisesRegex(ValueError, 'Cached render model SHA'):
                    load_render_model(original, folder, original_mjb=original_mjb, quality='balanced')


if __name__ == '__main__':
    unittest.main()
