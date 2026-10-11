"""USD fixtures for exact batched visuals and target-only plant contacts."""
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from farm_adapter import add_background, _pose


class FarmAdapterTests(unittest.TestCase):
    def fixture(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, 'Z')
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdGeom.Xform.Define(stage, '/World')
        UsdGeom.Xform.Define(stage, '/World/Greenhouse')
        source = UsdGeom.Xform.Define(stage, '/SourceStem')
        for name, offset in [('leaf0', 0.), ('leaf1', .3)]:
            mesh = UsdGeom.Mesh.Define(stage, '/SourceStem/' + name)
            mesh.CreatePointsAttr([(offset, 0., 0.), (offset+.1, 0., 0.), (offset, .1, 0.)])
            mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
            mesh.CreateDisplayColorAttr([(.1, .5, .2)])
        source.GetPrim().SetActive(False)
        for i in range(2):
            root = UsdGeom.Xform.Define(stage, f'/World/Greenhouse/Plants/Plant_{i:02d}/Stem')
            root.GetPrim().GetReferences().AddInternalReference('/SourceStem')
            root.GetPrim().SetActive(True);root.GetPrim().SetInstanceable(True)
            root.AddTranslateOp().Set(Gf.Vec3d(i, 0., 0.))
            root.AddRotateZOp().Set(90.*i)
        house = UsdGeom.Xform.Define(stage, '/World/Greenhouse/House')
        for name in ['Gutter_1_Base', 'Window']:
            mesh = UsdGeom.Mesh.Define(stage, str(house.GetPath()) + '/' + name + '/Mesh')
            mesh.CreatePointsAttr([(-.5,-.1,-.05),(.5,-.1,-.05),(.5,.1,-.05),(-.5,.1,-.05),
                                   (-.5,-.1,.05),(.5,-.1,.05),(.5,.1,.05),(-.5,.1,.05)])
            mesh.CreateFaceVertexCountsAttr([4]*6)
            mesh.CreateFaceVertexIndicesAttr([0,3,2,1,4,5,6,7,0,1,5,4,3,7,6,2,0,4,7,3,1,2,6,5])
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        ground = UsdGeom.Cube.Define(stage, '/World/Ground');ground.CreateSizeAttr(2.)
        ground.AddScaleOp().Set(Gf.Vec3f(3.,3.,.01))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim()).CreateCollisionEnabledAttr(True)
        return stage

    def export(self, stage, directory, **kwargs):
        root = ET.Element('mujoco');assets=ET.SubElement(root, 'asset');world=ET.SubElement(root, 'worldbody')
        report=add_background(stage, {}, world=world, assets=assets, output=directory, **kwargs)
        return root, report

    def test_exact_triangles_batched_and_shared_after_rotation(self):
        with tempfile.TemporaryDirectory() as directory:
            root, report = self.export(self.fixture(), directory)
            plants = [r for r in report['geometry_inventory'] if '/Plants/' in r['source_root']]
            self.assertEqual(len(plants), 2)
            self.assertEqual(plants[0]['mesh'], plants[1]['mesh'])
            self.assertEqual(plants[0]['source_meshes'], 2)
            self.assertEqual(plants[0]['triangles'], 2)
            self.assertEqual(report['counts']['source_meshes'], 6)
            self.assertFalse(report['triangle_decimation'])
            for geom in root.find('worldbody').findall('geom'):
                if geom.get('name','').startswith('farm_visual_'):
                    self.assertEqual(geom.get('contype'), '0')
                    self.assertEqual(geom.get('conaffinity'), '0')

    def test_gutter_ground_scope_leaves_house_and_other_plants_visual_only(self):
        with tempfile.TemporaryDirectory() as directory:
            _, report = self.export(self.fixture(), directory)
            self.assertEqual([r['classification'] for r in report['structural_inventory']], ['gutter', 'ground'])
            self.assertFalse(report['background_plant_collision'])
            self.assertEqual(report['background_plant_dofs'], 0)
            self.assertTrue(all(r['approximation'] == 'exact_source_box' for r in report['structural_inventory']))

    def test_authored_house_scope_and_source_remains_unchanged(self):
        stage = self.fixture();before=stage.GetRootLayer().ExportToString()
        with tempfile.TemporaryDirectory() as directory:
            _, report=self.export(stage, directory, structural_collision='authored-house')
            self.assertEqual(len(report['structural_inventory']), 3)
        self.assertEqual(before, stage.GetRootLayer().ExportToString())

    def test_excluded_target_geometry_is_not_duplicated(self):
        with tempfile.TemporaryDirectory() as directory:
            _, report=self.export(self.fixture(), directory,
                                  excluded_roots=('/World/Greenhouse/Plants/Plant_00',))
            self.assertEqual(report['counts']['source_meshes'], 4)
            self.assertFalse(any('/Plant_00/' in r['source_root'] for r in report['geometry_inventory']))

    def test_triangle_compatibility_preserves_original_face(self):
        stage=self.fixture()
        with tempfile.TemporaryDirectory() as directory:
            _, report=self.export(stage, directory,
                                  excluded_roots=('/World/Greenhouse/Plants',),
                                  structural_collision='visual-only')
            self.assertEqual(report['counts']['structural_collision_geoms'], 0)
            # Independent leaf triangle ensures the native four-vertex minimum
            # can be satisfied without adding any triangle or displacing it.
            stage2=Usd.Stage.CreateInMemory();UsdGeom.SetStageUpAxis(stage2, 'Z');UsdGeom.SetStageMetersPerUnit(stage2,1.)
            UsdGeom.Xform.Define(stage2,'/World/Greenhouse')
            leaf=UsdGeom.Mesh.Define(stage2,'/World/Greenhouse/leaf')
            leaf.CreatePointsAttr([(0.,0.,0.),(1.,0.,0.),(0.,1.,0.)])
            leaf.CreateFaceVertexCountsAttr([3]);leaf.CreateFaceVertexIndicesAttr([0,1,2])
            _, r=self.export(stage2, Path(directory)/'single')
            asset=r['mesh_asset_inventory'][0];text=Path(asset['file']).read_text()
            self.assertEqual(sum(line.startswith('v ') for line in text.splitlines()),4)
            self.assertEqual(sum(line.startswith('f ') for line in text.splitlines()),1)
            self.assertEqual(asset['unreferenced_compatibility_vertices'],1)

    def test_pose_keeps_reflection_and_nonuniform_scale_in_mesh(self):
        source=np.eye(4);source[:3,:3]=np.array([[0.,-2.,0.],[-1.,0.,0.],[0.,0.,.5]])
        source[:3,3]=[1.,2.,3.]
        pos, q, affine=_pose(source)
        from scipy.spatial.transform import Rotation
        rotation=Rotation.from_quat(np.r_[q[1:],q[0]]).as_matrix()
        np.testing.assert_allclose(rotation@affine,source[:3,:3],atol=1e-11)
        np.testing.assert_array_equal(pos,source[:3,3])
        self.assertGreater(np.linalg.det(rotation),0.)
        self.assertLess(np.linalg.det(affine),0.)

    def test_top_plane_keeps_visual_box_and_authored_top_after_yaw(self):
        stage=self.fixture();ground=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Ground'))
        ground.ClearXformOpOrder()
        ground.AddTranslateOp().Set(Gf.Vec3d(1.,4.,-.12))
        ground.AddRotateZOp().Set(37.)
        ground.AddScaleOp().Set(Gf.Vec3f(8.,12.,.02))
        with tempfile.TemporaryDirectory() as directory:
            xml, report=self.export(stage,directory,ground_collision='top-plane')
            plane=xml.find('worldbody').find("geom[@name='farm_ground']")
            visual=xml.find('worldbody').find("geom[@name='farm_ground_visual']")
            self.assertEqual(plane.get('type'),'plane')
            np.testing.assert_allclose(np.fromstring(plane.get('pos'),sep=' '),[1.,4.,-.1],atol=1e-9)
            self.assertEqual(visual.get('type'),'box');self.assertEqual(visual.get('contype'),'0')
            np.testing.assert_allclose(np.fromstring(visual.get('size'),sep=' '),[8.,12.,.02],atol=1e-9)
            self.assertEqual(plane.get('contype'),'8')
            metadata=next(r for r in report['structural_inventory'] if r['classification']=='ground')
            self.assertTrue(metadata['infinite_collision_extent'])
            self.assertTrue(metadata['source_box_retained_visually'])
            self.assertAlmostEqual(metadata['top_world_z_m'],-.1)

    def test_tilted_ground_is_not_silently_replaced_by_horizontal_plane(self):
        stage=self.fixture();ground=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Ground'))
        ground.ClearXformOpOrder()
        ground.AddRotateXOp().Set(20.)
        ground.AddScaleOp().Set(Gf.Vec3f(3.,3.,.01))
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError,'horizontal source top face'):
                self.export(stage,directory,ground_collision='top-plane')


if __name__ == '__main__':
    unittest.main()
