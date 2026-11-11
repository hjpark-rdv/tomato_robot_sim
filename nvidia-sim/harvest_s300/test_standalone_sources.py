"""USD/GLB source reconstruction tests; run with the asset Python."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import trimesh
from pxr import Usd, UsdGeom

from build_scene import s300_from_assembly_glb


class StandaloneSourceTests(unittest.TestCase):
    def fixture(self, root):
        root=Path(root)
        scene=trimesh.Scene()
        chassis=trimesh.creation.box(extents=[.2,.4,.6])
        chassis.apply_translation([1.,2.,3.])
        scene.add_geometry(chassis,node_name='chassis_green')
        scene.add_geometry(trimesh.creation.box(),node_name='robot_ignored')
        file=root/'assembly.glb'
        scene.export(file)
        metadata=dict(sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                      export_axes='glTF Y-up; x=x, y=z, z=-y; metres')
        (root/'model.json').write_text(json.dumps(metadata))
        return file,metadata

    def test_only_chassis_and_exact_axis_transform(self):
        with tempfile.TemporaryDirectory() as directory:
            source,_=self.fixture(directory)
            output=Path(directory)/'chassis.usdc'
            report=s300_from_assembly_glb(source,output)
            stage=Usd.Stage.Open(str(output))
            meshes=[p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
            self.assertEqual([p.GetName() for p in meshes],['chassis_green'])
            points=np.asarray(UsdGeom.Mesh(meshes[0]).GetPointsAttr().Get())
            np.testing.assert_allclose(points.mean(axis=0),[1.,-3.,2.],atol=1e-6)
            np.testing.assert_allclose(np.ptp(points,axis=0),[.2,.6,.4],atol=1e-6)
            self.assertFalse(report['robot_or_tool_physics_from_glb'])

    def test_unknown_axes_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source,metadata=self.fixture(directory)
            metadata['export_axes']='unknown'
            source.with_name('model.json').write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError,'coordinate convention'):
                s300_from_assembly_glb(source,Path(directory)/'out.usdc')

    def test_changed_source_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source,_=self.fixture(directory)
            source.write_bytes(source.read_bytes()+b'changed')
            with self.assertRaisesRegex(ValueError,'hash mismatch'):
                s300_from_assembly_glb(source,Path(directory)/'out.usdc')


if __name__=='__main__':
    unittest.main()
