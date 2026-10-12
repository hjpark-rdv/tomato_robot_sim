"""USD source semantics: subsets, indexed colours and explicit refusal."""
import unittest

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from render_assets import read_usd_material
from source_face_colors import extract_face_colors


class FaceColorUsdTests(unittest.TestCase):
    def fixture(self):
        stage = Usd.Stage.CreateInMemory()
        mesh = UsdGeom.Mesh.Define(stage, '/mesh')
        mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0),
                              (1, 1, 0), (2, 1, 0), (1, 2, 0)])
        mesh.CreateFaceVertexCountsAttr([3, 3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3, 4, 5])
        return stage, mesh

    def test_two_bound_face_subsets_survive_no_mesh_binding(self):
        stage, mesh = self.fixture()
        for i, rgb in enumerate(((.8, .8, .8), (.0, .05, .1))):
            mat = UsdShade.Material.Define(stage, '/mat' + str(i))
            shader = UsdShade.Shader.Define(stage, '/mat' + str(i) + '/shader')
            shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
            subset = UsdShade.MaterialBindingAPI(mesh.GetPrim()).CreateMaterialBindSubset('part' + str(i), [i])
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(mat)
        got = extract_face_colors(mesh.GetPrim(), read_usd_material)
        self.assertEqual(got['subset_faces'], 2)
        self.assertEqual(len(got['subsets']), 2)
        np.testing.assert_allclose(got['palette'][got['labels'], :3], [[.8, .8, .8], [0, .05, .1]], atol=1e-7)

    def test_indexed_vertex_colors_preserve_both_regions(self):
        stage, mesh = self.fixture()
        color = mesh.GetDisplayColorPrimvar()
        color.Set([(1, 0, 0), (0, 0, 1)])
        color.SetInterpolation('vertex')
        color.SetIndices([0, 0, 0, 1, 1, 1])
        got = extract_face_colors(mesh.GetPrim(), read_usd_material)
        np.testing.assert_array_equal(got['palette'][got['labels'], :3], [[1, 0, 0], [0, 0, 1]])
        self.assertEqual(got['method'], 'face_constant_vertex_colors')

    def test_gradient_is_not_silently_replaced_by_first_color(self):
        stage, mesh = self.fixture()
        color = mesh.GetDisplayColorPrimvar()
        color.Set([(1, 0, 0), (0, 1, 0), (0, 0, 1)] * 2)
        color.SetInterpolation('vertex')
        with self.assertRaisesRegex(ValueError, 'gradient'):
            extract_face_colors(mesh.GetPrim(), read_usd_material)


if __name__ == '__main__':
    unittest.main()
