import json
from pathlib import Path
import tempfile
import unittest

import mujoco as mj
import numpy as np

from newton_fruit_ellipsoid import enclosing_ellipsoid, prepare, protected_fields
from newton_fruit_proxy import geom_body_vertices, sha256, sphere_directions
from test_newton_fruit_proxy import fixture


class EnclosingEllipsoidTests(unittest.TestCase):
    def test_asymmetric_hull_is_fully_enclosed(self):
        p=sphere_directions(512)*[.011,.013,.009]
        p[:,0]*=1.+.08*p[:,2]/.009
        p+=np.array([.003,-.002,.001])
        c,a,r,proof=enclosing_ellipsoid(p)
        level=np.sum((((p-c)@r)/a)**2,axis=1)
        self.assertLess(level.max(),1.)
        self.assertTrue(proof['source_enclosed'])
        self.assertTrue(np.all(a>0))
        self.assertAlmostEqual(np.linalg.det(r),1.)

    def test_degenerate_and_nonfinite_source_rejected(self):
        for p in (np.zeros((4,3)),np.array([[0,0,0],[1,0,0],[0,1,0],[1,1,0]]),
                  np.full((4,3),np.nan)):
            with self.assertRaisesRegex(ValueError,'nondegenerate'):
                enclosing_ellipsoid(p)

    def test_compiled_transform_protected_fields_and_source_immutability(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=fixture(root)
            hashes={p.name:sha256(p) for p in source.iterdir()}
            result=prepare(source,root/'analytic')
            self.assertEqual(hashes,{p.name:sha256(p) for p in source.iterdir()})
            old=mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
            new=mj.MjModel.from_binary_path(str(root/'analytic/input_model.mjb'))
            g=new.geom('fruit_geom').id
            self.assertEqual(new.geom_type[g],mj.mjtGeom.mjGEOM_ELLIPSOID)
            matrix=np.zeros(9);mj.mju_quat2Mat(matrix,new.geom_quat[g])
            points=geom_body_vertices(old,old.geom('fruit_geom').id)
            self.assertLess(np.max(np.sum((((points-new.geom_pos[g])@matrix.reshape(3,3))/new.geom_size[g])**2,axis=1)),1.)
            self.assertTrue(result['invariant_proof']['passed'])
            self.assertEqual(sha256(source/'initial_state.npz'),sha256(root/'analytic/initial_state.npz'))
            self.assertEqual(sha256(source/'commands.npz'),sha256(root/'analytic/commands.npz'))
            proof=json.loads((root/'analytic/inputs.json').read_text())
            for name,evidence in proof['input_files'].items():
                self.assertEqual(evidence['sha256'],sha256(root/'analytic'/name))
            new.body_mass[1]+=.01
            with self.assertRaisesRegex(ValueError,'body_mass'):
                protected_fields(old,new,{g})


if __name__=='__main__':unittest.main()
