"""USD-side source constants are recovered without arbitrary recolouring."""
import unittest
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from render_assets import read_usd_material


class SourceMaterialTests(unittest.TestCase):
    def mesh(self):
        stage = Usd.Stage.CreateInMemory()
        mesh = UsdGeom.Mesh.Define(stage, '/World/mesh')
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.2, .3, .4)])
        return stage, mesh.GetPrim()

    def test_preview_authored_pbr_overrides_display_color(self):
        stage, prim = self.mesh()
        material = UsdShade.Material.Define(stage, '/World/Looks/a')
        shader = UsdShade.Shader.Define(stage, '/World/Looks/a/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.7, .1, .1))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.22)
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(.6)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
        got = read_usd_material(prim)
        self.assertAlmostEqual(got['rgba'][0], .7)
        self.assertAlmostEqual(got['roughness'], .22)
        self.assertAlmostEqual(got['metallic'], .6)
        self.assertEqual(got['shader_kind'], 'UsdPreviewSurface')
        self.assertEqual(len(got['source_material_sha256']), 64)

    def test_omnipbr_mdl_white_cad_uses_authored_constant(self):
        stage, prim = self.mesh()
        material = UsdShade.Material.Define(stage, '/World/Looks/cad')
        shader = UsdShade.Shader.Define(stage, '/World/Looks/cad/DefaultMaterial')
        shader.CreateInput('diffuse_color_constant', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1., 1., 1.))
        shader.GetPrim().CreateAttribute('info:mdl:sourceAsset:subIdentifier', Sdf.ValueTypeNames.Token).Set('OmniPBR')
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
        got = read_usd_material(prim)
        self.assertEqual(got['rgba'], [1., 1., 1., 1.])
        self.assertEqual(got['shader_kind'], 'OmniPBR')
        self.assertNotIn('roughness', got['authored_channels'])
        self.assertEqual(got['roughness'], .5)


    def test_attribute_and_inherited_colorspace_are_resolved(self):
        stage, prim = self.mesh()
        parent = stage.GetPrimAtPath('/World')
        Usd.ColorSpaceAPI.Apply(parent).CreateColorSpaceNameAttr('srgb_rec709_scene')
        got = read_usd_material(prim)
        self.assertEqual(got['color_space'], 'srgb_rec709_scene')
        self.assertEqual(got['explicit_color_space'], '')
        color = UsdGeom.Gprim(prim).GetDisplayColorAttr()
        color.SetColorSpace('lin_rec709_scene')
        got = read_usd_material(prim)
        self.assertEqual(got['color_space'], 'lin_rec709_scene')
        self.assertEqual(got['explicit_color_space'], 'lin_rec709_scene')

    def test_display_color_is_preserved_without_material(self):
        _, prim = self.mesh()
        got = read_usd_material(prim)
        self.assertAlmostEqual(got['rgba'][0], .2)
        self.assertEqual(got['shader_kind'], 'displayColor')
        self.assertEqual(got['material_path'], '')
        self.assertEqual(got['color_space'], 'linear_rec709')
        self.assertFalse(got['authored_channels'])


if __name__ == '__main__':
    unittest.main()
