"""Viewer coordinate/identity regression checks; no browser or physics stepping."""
import unittest
import numpy as np
import mujoco as mj
from newton_viser import body_mapping, geom_mesh, geom_color


class ViserMappingTests(unittest.TestCase):
    def model(self):
        return mj.MjModel.from_xml_string('''<mujoco><worldbody>
        <body name="tool" pos="1 2 3"><geom type="box" size=".1 .2 .3" pos=".4 0 0"/></body>
        </worldbody></mujoco>''')

    def test_body_identity_not_xml_index(self):
        m = self.model()
        report = {'mapping': {'usd_to_native': {'/Tool': {'name': 'tool', 'index': 999}}}}
        self.assertEqual(body_mapping(m, report), {1: '/Tool'})

    def test_missing_body_is_error(self):
        with self.assertRaises(ValueError):
            body_mapping(self.model(), {'mapping': {'usd_to_native': {}}})

    def test_nonidentity_source_frames_rejected(self):
        with self.assertRaises(ValueError):
            body_mapping(self.model(), {'mapping': {'bodies_with_frame_offsets': ['/Tool']}})

    def test_mesh_in_body_frame_no_double_world_transform(self):
        m = self.model()
        before = m.geom_pos.copy()
        mesh = geom_mesh(m, 0)
        np.testing.assert_allclose(mesh.bounds, [[.3,-.2,-.3],[.5,.2,.3]])
        np.testing.assert_array_equal(m.geom_pos, before)

    def test_material_color_and_glass_opacity(self):
        model=mj.MjModel.from_xml_string('''<mujoco><asset>
        <material name="leaf" rgba=".1 .6 .2 .3"/></asset><worldbody>
        <geom type="box" size="1 1 1" material="leaf"/></worldbody></mujoco>''')
        np.testing.assert_allclose(geom_color(model,0),[.1,.6,.2,.3])


if __name__ == '__main__': unittest.main()
