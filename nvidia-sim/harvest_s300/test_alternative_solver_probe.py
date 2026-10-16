"""Scope/invariance checks for the solver qualification fixture extraction."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np


MODULE = Path(__file__).resolve().with_name('alternative_solver_probe.py')
spec = importlib.util.spec_from_file_location('alternative_solver_probe', MODULE)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class FixtureExtraction(unittest.TestCase):
    XML = '''<mujoco><asset><mesh name="fruit" file="fruit.obj"/>
    <mesh name="visual" file="visual.obj"/></asset><worldbody>
    <body name="Stem_root"><inertial mass=".1" diaginertia=".1 .1 .1"/>
    <body name="Truss_branch"><joint name="bend" stiffness=".25" damping=".02"/>
    <geom name="branch" type="capsule" contype="1" conaffinity="3" size=".001"/>
    <geom name="leaf_visual" type="mesh" mesh="visual" contype="0" conaffinity="0"/>
    <site name="anchor"/></body></body>
    <body name="Truss_fruit"><freejoint/><inertial mass=".007" diaginertia=".1 .1 .1"/>
    <geom name="fruit" type="mesh" mesh="fruit" contype="1" conaffinity="3"/>
    <site name="fruit_anchor"/></body>
    <body name="Robot"><body name="Tool_L_belt"><inertial mass=".084" diaginertia=".1 .1 .1"/>
    <geom name="belt" type="box" size=".0025 .06 .026" contype="2" conaffinity="3"/>
    <geom name="belt_other" type="mesh" mesh="visual"/></body></body>
    <geom name="background" type="box"/></worldbody>
    <equality><connect name="attachment" site1="anchor" site2="fruit_anchor"/></equality>
    <contact><exclude body1="Stem_root" body2="Truss_fruit"/>
    <exclude body1="Robot" body2="Truss_fruit"/></contact>
    <actuator><motor joint="robot_joint"/></actuator></mujoco>'''

    def extract(self, directory, **kwargs):
        source = Path(directory)/'source.xml'
        output = Path(directory)/'output.xml'
        source.write_text(self.XML)
        before = source.read_bytes()
        report = probe.extract_plant(source, output, **kwargs)
        self.assertEqual(source.read_bytes(), before)
        return ET.parse(output).getroot(), report

    def test_physical_plant_and_attachment_survive(self):
        with tempfile.TemporaryDirectory() as directory:
            root, report = self.extract(directory)
        self.assertEqual(report['active_geom_count'], 2)
        self.assertEqual(report['mesh_assets'], 1)
        self.assertEqual({g.get('name') for g in root.iter('geom')}, {'fruit', 'branch'})
        self.assertEqual(root.find('.//joint').get('stiffness'), '.25')
        self.assertEqual(root.find('.//connect').get('site2'), 'fruit_anchor')
        self.assertIsNone(root.find('actuator'))
        self.assertEqual(len(list(root.iter('exclude'))), 1)

    def test_coupled_specimen_is_dynamic_and_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            root, report = self.extract(directory, probe_center=[1.,2.,3.])
        specimen = next(b for b in root.iter('body') if b.get('name')=='Qualification_probe')
        self.assertEqual(specimen.find('joint').get('type'), 'slide')
        self.assertEqual(specimen.find('inertial').get('mass'), '.084')
        self.assertEqual([g.get('name') for g in specimen.iter('geom')], ['belt'])
        self.assertIn('isolated real inner-belt box', report['meaning'])

    def test_preload_adapter_preserves_force_for_every_position(self):
        stiffness=np.array([.25,0.,.005])
        preload=np.array([.01,.03,-.0002])
        target=np.array([.2,8.,-.1])
        mapped,remaining=probe.fold_constant_preload(stiffness,target,preload,np.array([0,-1,2]))
        for position in [np.array([0.,0.,0.]),np.array([-.5,.1,.7]),np.array([.3,-.2,.5])]:
            original=-stiffness*(position-target)+preload
            adapted=-stiffness*(position-mapped)+remaining
            np.testing.assert_allclose(adapted,original,atol=1e-15,rtol=1e-15)
        self.assertEqual(remaining[1],preload[1])

    def test_preload_exceeding_source_limit_keeps_residual_force(self):
        stiffness=np.array([.005])
        preload=np.array([.0041])
        target=np.array([0.])
        mapped,remaining=probe.fold_constant_preload(stiffness,target,preload,np.array([0]),
            lower=np.array([-.785]),upper=np.array([.785]))
        self.assertEqual(mapped[0],.785)
        self.assertGreater(remaining[0],0)
        for position in [-.4,0.,.3,.7]:
            self.assertAlmostEqual(-stiffness[0]*position+preload[0],
                -stiffness[0]*(position-mapped[0])+remaining[0],places=15)

    def test_valid_tiny_inertia_is_preserved_without_absolute_inflation(self):
        moments=np.array([[8.e-11,2.e-9,2.e-9]])
        tensor,inverse=probe.validated_source_inertia(moments,np.array([[1.,0.,0.,0.]]))
        np.testing.assert_allclose(np.diagonal(tensor,axis1=1,axis2=2),moments,rtol=1e-15,atol=0)
        np.testing.assert_allclose(tensor@inverse,np.eye(3)[None],rtol=1e-15,atol=1e-15)

    def test_invalid_source_inertia_is_rejected_instead_of_restored(self):
        for moments in [[-1.e-10,2.e-9,2.e-9],[1.e-10,1.e-10,1.e-6]]:
            with self.assertRaises(ValueError):
                probe.validated_source_inertia(np.array([moments]),np.array([[1.,0.,0.,0.]]))


if __name__ == '__main__':
    unittest.main()
